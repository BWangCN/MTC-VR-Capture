"""First-person video per take: the headset camera rendered in the 3DGS of the room, course objects composited.

    PYTHONNOUSERSITE=1 CUDA_HOME=/usr/local/cuda-12.9 analysis/data/venv-gsplat/bin/python analysis/fpv_3dgs.py \
        analysis/data/raw/2026-10-30_recordings/recordings --ckpt analysis/data/3dgs/ckpt_final.pt \
        --manifest analysis/data/3dgs/manifest.json [--out analysis/data/out/fpv] [--size 1280x720] [--hfov 90] [--fps 50]

Camera: the Pico rigid body as Unity placed it (player scale and alignment, cameraOffsetFromRigidBody = 0), so the
viewpoint is the one shown in VR (eye at about 1.30 m). Orientation: the rigid body's +X is the headset's front
(it points along the walking direction in every walk) and +Z its up. Pinhole, --hfov horizontal field of view.
Checked against the camera rotations Unity logged ([XR Camera] Rot) at the matched frames.

Rendering: the 3DGS (gsplat 1.5.3; checkpoint of the MTC_Next scene package, Gaussians in gsplat's normalised
frame x_norm = T_mesh2norm x_scene, same convention as MTC_Next python/mtcnext/render/gs.py) gives colour and
z-depth; the course objects and route markers (scene.glb, markers.glb) are rasterised with PyTorch3D (vertex
colours, unlit as in VR) and drawn where they are nearer than the 3DGS. The room is ScanNet++ data (research use
only, not to be redistributed): keep the videos local.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mocaprec as mr  # noqa: E402
import scene_env as se  # noqa: E402
from fit_smplx import quat_xyzw_to_matrix  # noqa: E402

GLTF_TO_SCENE_R = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float)     # (x, y, z)_scene = (z, x, y)_gltf + floor


# ------------------------------------------------------------------------------------------------ 3DGS
def find_key(obj, key):
    """First value stored under key anywhere in a nested dict/list."""
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for v in obj.values():
            r = find_key(v, key)
            if r is not None:
                return r
    elif isinstance(obj, list):
        for v in obj:
            r = find_key(v, key)
            if r is not None:
                return r
    return None


class Gaussians:
    def __init__(self, ckpt_path, manifest_path=None, device="cuda"):
        import gsplat
        self.gsplat = gsplat
        ck = torch.load(ckpt_path, map_location=device, weights_only=False)
        sp = ck["splats"]
        self.means = sp["means"].float()
        self.quats = sp["quats"].float()
        self.scales = torch.exp(sp["scales"].float())
        self.opacities = torch.sigmoid(sp["opacities"].float())
        self.colors = torch.cat([sp["sh0"], sp["shN"]], 1).float()
        self.sh_degree = int(round(np.sqrt(self.colors.shape[1]))) - 1
        T = find_key({k: v for k, v in ck.items() if k != "splats"}, "T_mesh2norm")
        src = "checkpoint"
        if manifest_path and os.path.exists(manifest_path):
            with open(manifest_path) as f:
                Tm = find_key(json.load(f), "T_mesh2norm")
            if Tm is not None:
                if T is not None and not np.allclose(np.asarray(T, float), np.asarray(Tm, float), atol=1e-6):
                    raise ValueError("T_mesh2norm differs between the checkpoint and the manifest")
                T, src = Tm, ("checkpoint + manifest" if T is not None else "manifest")
        if T is None:
            raise ValueError("no T_mesh2norm in the checkpoint or the manifest")
        T = np.asarray(T.cpu() if torch.is_tensor(T) else T, float)
        M = T[:3, :3]
        self.s = float(np.cbrt(np.linalg.det(M)))
        self.R = M / self.s
        self.t = T[:3, 3]
        self.device = device
        self.info = {"gaussians": int(self.means.shape[0]), "sh_degree": self.sh_degree, "scale": self.s, "T_from": src}

    def render(self, K, W, H, t_scene, R_scene_cam, near=0.03, far=30.0):
        """RGB (H, W, 3) in 0..1 (torch, GPU), metric z-depth (inf where alpha < 0.5)."""
        Rn = self.R @ R_scene_cam
        cn = self.s * self.R @ t_scene + self.t
        V = np.eye(4)
        V[:3, :3] = Rn.T
        V[:3, 3] = -Rn.T @ cn
        Vt = torch.tensor(V, dtype=torch.float32, device=self.device)[None]
        Kt = torch.tensor(K, dtype=torch.float32, device=self.device)[None]
        with torch.no_grad():
            img, alpha, _ = self.gsplat.rasterization(
                self.means, self.quats, self.scales, self.opacities, self.colors, Vt, Kt, W, H, sh_degree=self.sh_degree,
                render_mode="RGB+ED", near_plane=near * self.s, far_plane=far * self.s, packed=False)
        rgb = img[0, ..., :3].clamp(0, 1)
        z = img[0, ..., 3] / self.s
        z = torch.where(alpha[0, ..., 0] > 0.5, z, torch.full_like(z, float("inf")))
        return rgb, z


# ------------------------------------------------------------------------------------------------ meshes
class Objects:
    """Course objects and route markers, rasterised with PyTorch3D in the scene frame."""

    def __init__(self, env, device="cuda"):
        from pytorch3d.structures import Meshes
        Vs, Fs, Cs, n = [], [], [], 0
        for p in env.parts:
            if p.kind in ("room", "collision"):
                continue
            Vs.append(gltf_to_scene(p.V, env.floor_z))
            Fs.append(p.F.astype(np.int64) + n)
            Cs.append(p.C.astype(np.float32) / 255.0)
            n += len(p.V)
        self.V = torch.tensor(np.concatenate(Vs), dtype=torch.float32, device=device)
        self.F = torch.tensor(np.concatenate(Fs), dtype=torch.int64, device=device)
        self.C = torch.tensor(np.concatenate(Cs), dtype=torch.float32, device=device)
        self.Meshes = Meshes
        self.device = device

    def render(self, K, W, H, t_scene, R_scene_cam):
        from pytorch3d.renderer import MeshRasterizer, RasterizationSettings
        from pytorch3d.utils import cameras_from_opencv_projection
        Rw2c = torch.tensor(R_scene_cam.T, dtype=torch.float32, device=self.device)[None]
        tw2c = torch.tensor(-R_scene_cam.T @ t_scene, dtype=torch.float32, device=self.device)[None]
        cams = cameras_from_opencv_projection(Rw2c, tw2c, torch.tensor(K, dtype=torch.float32, device=self.device)[None],
                                              torch.tensor([[H, W]], device=self.device))
        rs = RasterizationSettings(image_size=(H, W), faces_per_pixel=1, blur_radius=0.0, bin_size=0, cull_backfaces=False)
        frag = MeshRasterizer(cameras=cams, raster_settings=rs)(self.Meshes([self.V], [self.F]))
        pf = frag.pix_to_face[0, ..., 0]
        hit = pf >= 0
        bary = frag.bary_coords[0, ..., 0, :]
        col = torch.zeros(H, W, 3, device=self.device)
        if hit.any():
            fv = self.F[pf[hit]]                                         # (n, 3) vertex ids
            col[hit] = (bary[hit][..., None] * self.C[fv]).sum(1)
        z = torch.where(hit, frag.zbuf[0, ..., 0], torch.full_like(frag.zbuf[0, ..., 0], float("inf")))
        return col, z


def gltf_to_scene(P, floor_z):
    return np.asarray(P, float) @ GLTF_TO_SCENE_R.T + np.array([0.0, 0.0, floor_z])


# ------------------------------------------------------------------------------------------------ camera
def headset_cameras(take, al, frames, floor_z):
    """Per frame: camera centre (scene frame) and R_scene_cam (OpenCV axes: x right, y down, z forward)."""
    pos = take.rb[mr.PICO_RB_ID]["pos"]
    quat = take.rb[mr.PICO_RB_ID]["quat_xyzw"]
    out_t, out_R, last = [], [], None
    for i in frames:
        ok = np.isfinite(pos[i]).all() and np.isfinite(quat[i]).all()
        if not ok and last is not None:
            out_t.append(out_t[-1]); out_R.append(out_R[-1]); continue
        Rq = quat_xyzw_to_matrix(quat[i])
        A = mr.UNITY_TO_GLTF @ al.M[i]                                    # native direction -> glTF (with scale)
        f = A @ (Rq @ np.array([1.0, 0, 0]))
        u = A @ (Rq @ np.array([0, 0, 1.0]))
        f /= np.linalg.norm(f)
        r = np.cross(f, u)
        r /= np.linalg.norm(r)
        u = np.cross(r, f)
        R_gltf = np.stack([r, -u, f], 1)                                  # columns: camera x, y, z in glTF
        out_R.append(GLTF_TO_SCENE_R @ R_gltf)
        out_t.append(gltf_to_scene(al.to_gltf(i, pos[i]), floor_z))
        last = i
    return np.array(out_t), np.array(out_R)


def unity_forward(euler_deg):
    """Forward (+Z) of a Unity rotation given as eulerAngles (applied Z, then X, then Y)."""
    x, y, z = np.radians(euler_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    Ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    Rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return (Ry @ Rx @ Rz) @ np.array([0, 0, 1.0])


def orientation_check(take, al, floor_z):
    """Angle between our camera forward and the forward of the camera rotation Unity logged, per log sample."""
    idx = [s["index"] for s in al.samples]
    _, Rs = headset_cameras(take, al, idx, floor_z)
    out = []
    for s, R in zip(al.samples, Rs):
        f_scene = R[:, 2]
        f_gltf = GLTF_TO_SCENE_R.T @ f_scene
        f_unity = mr.UNITY_TO_GLTF @ f_gltf                              # mirror x back
        g = unity_forward(s["cam_euler_deg"])
        out.append(np.degrees(np.arccos(np.clip(f_unity @ g, -1, 1))))
    return np.array(out)


# ------------------------------------------------------------------------------------------------ main
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--ckpt", default=os.path.join(HERE, "data", "3dgs", "ckpt_final.pt"))
    ap.add_argument("--manifest", default=os.path.join(HERE, "data", "3dgs", "manifest.json"))
    ap.add_argument("--env", default=se.DEFAULT_ENV)
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "fpv"))
    ap.add_argument("--size", default="1280x720")
    ap.add_argument("--hfov", type=float, default=90.0)
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--takes", nargs="*")
    ap.add_argument("--check-only", action="store_true", help="only compare the camera orientation with the log")
    a = ap.parse_args(argv)
    W, H = (int(x) for x in a.size.split("x"))
    f = (W / 2) / np.tan(np.radians(a.hfov) / 2)
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    env = se.load_environment(a.env, room_cell=0.05, with_collision=False)
    logs = mr.parse_editor_log(os.path.join(a.root, "Editor.log"))
    takes = []
    for p in mr.find_takes(a.root):
        take = mr.load_take(p)
        if take.log_name in logs and (not a.takes or take.name in a.takes):
            takes.append((take, mr.estimate_alignment(take, logs[take.log_name])))
    for take, al in takes:
        ang = orientation_check(take, al, env.floor_z)
        print(f"{take.name}: camera forward vs Unity's logged camera: median {np.median(ang):.1f} deg, "
              f"max {ang.max():.1f} deg over {len(ang)} log lines", flush=True)
    if a.check_only:
        return 0
    gs = Gaussians(a.ckpt, a.manifest)
    print("3DGS:", gs.info, flush=True)
    obj = Objects(env)
    os.makedirs(a.out, exist_ok=True)
    step = max(1, round(100 / a.fps))
    for take, al in takes:
        frames = np.arange(0, len(take.frame), step)
        ts, Rs = headset_cameras(take, al, frames, env.floor_z)
        np.savez(os.path.join(a.out, f"{take.name}_camera.npz"), frames=frames, t_scene=ts, R_scene_cam=Rs, K=K,
                 size=np.array([W, H]), note="scene frame of the MTC_Next package (Z up); OpenCV camera axes")
        path = os.path.join(a.out, f"{take.name}_fpv3dgs.mp4")
        ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                               "-r", str(a.fps), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18",
                               "-preset", "medium", "-movflags", "+faststart", path], stdin=subprocess.PIPE)
        t0 = time.time()
        for t_s, R in zip(ts, Rs):
            rgb, z = gs.render(K, W, H, t_s, R)
            ocol, oz = obj.render(K, W, H, t_s, R)
            front = oz < z
            img = torch.where(front[..., None], ocol, rgb)
            ff.stdin.write((img.clamp(0, 1) * 255).round().byte().cpu().numpy().tobytes())
        ff.stdin.close()
        ff.wait()
        print(f"{path}: {len(frames)} frames in {time.time() - t0:.0f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
