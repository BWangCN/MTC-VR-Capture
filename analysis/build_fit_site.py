"""Website for the SMPL-X fits: body mesh, markers and fit residuals, in the course (VR scale) or at true size.

    PYTHONNOUSERSITE=1 python analysis/build_fit_site.py analysis/data/raw/2026-10-30_recordings/recordings \
        [--fits analysis/data/out/smplx] [--out analysis/data/out/site] [--fps 25] [--with-room]

Writes <out>/index.html, <out>/data/env.json, data/body_faces.json and one data/<take>.json per take
(gzip + base64 inside JSON; see Blob). The page loads three.js
0.170.0 from jsDelivr and its data files by relative path, so it works as a published artifact (files) or
from a local web server. --with-room also writes data/room.json (the ScanNet++ scan, research use only, not to be
redistributed): use it only for a local copy, never for a published one.

Per take (sampled at --fps from 100 Hz):
- mesh: SMPL-X posed with the fitted parameters, reduced by vertex clustering of the template (2.5 cm cells)
  to about 3,000 vertices; int16 mm in the fit frame (native Motive turned Y up; true size);
- course transform per frame (3x4, float32): fit frame -> scene glTF, i.e. Unity's mapping (player scale,
  alignment) followed by the x mirror, so the body stands where VR showed it;
- observed markers (body points and the 4 headset markers), predicted anchors and per-anchor residuals (mm).
The model files themselves are never written: only posed, reduced meshes.
"""
import argparse
import datetime
import json
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mocaprec as mr  # noqa: E402
import scene_env as se  # noqa: E402
import smplx_model as sm  # noqa: E402
from fit_smplx import TakeData, Z_UP_TO_Y_UP  # noqa: E402

MISSING = -32768
MESH_CELL_M = 0.025


def mm16(P):
    P = np.asarray(P, float)
    out = np.clip(np.round(P * 1000), -32767, 32767)
    out[~np.isfinite(P)] = MISSING
    return out.astype(np.int16)


class Blob:
    """Concatenated typed arrays + an index of (name, dtype, shape, offset[, delta]) for the page.

    Written as JSON {"gz64": base64(gzip(bytes))} (the artifact host serves no raw binary files); the page
    unpacks it with DecompressionStream. Arrays added with delta=True are stored as differences between
    consecutive frames (axis 0), which compresses body meshes about 4x; the page sums them back.
    """

    def __init__(self):
        self.parts, self.index, self.n = [], {}, 0

    def add(self, name, a, delta=False):
        a = np.ascontiguousarray(a)
        pad = (-self.n) % 8
        if pad:
            self.parts.append(b"\0" * pad)
            self.n += pad
        self.index[name] = {"dtype": a.dtype.name, "shape": list(a.shape), "offset": self.n}
        if delta:
            d = a.astype(np.int32)
            d[1:] = d[1:] - d[:-1]
            assert np.abs(d).max() < 32767, name
            a = d.astype(a.dtype)
            self.index[name]["delta"] = int(np.prod(a.shape[1:]))      # elements per frame
        b = a.tobytes()
        self.parts.append(b)
        self.n += len(b)

    def write(self, path):
        import base64
        import gzip
        with open(path, "w") as f:
            json.dump({"gz64": base64.b64encode(gzip.compress(b"".join(self.parts), 9)).decode("ascii")}, f)
        return self.index


def mesh_reduction(bm):
    """Vertex clustering of the rest template: cluster id per vertex and faces of the reduced mesh."""
    V = bm.v_template.cpu().numpy()
    F = bm.faces
    key = np.floor(V / MESH_CELL_M).astype(np.int64)
    key -= key.min(0)
    dims = key.max(0) + 1
    lin = (key[:, 0] * dims[1] + key[:, 1]) * dims[2] + key[:, 2]
    _, inv = np.unique(lin, return_inverse=True)
    Fn = inv[F]
    ok = (Fn[:, 0] != Fn[:, 1]) & (Fn[:, 1] != Fn[:, 2]) & (Fn[:, 0] != Fn[:, 2])
    Fn = Fn[ok]
    _, first = np.unique(np.sort(Fn, 1), axis=0, return_index=True)
    Fn = Fn[np.sort(first)]
    return inv, Fn.astype(np.uint16 if inv.max() < 65535 else np.uint32), int(inv.max() + 1)


def course_transform(al, frames):
    """(n, 3, 4): fit frame (Y up) -> scene glTF, per frame (Unity's M and t, then the x mirror)."""
    back = Z_UP_TO_Y_UP.T                                       # fit frame -> native Motive
    out = np.zeros((len(frames), 3, 4), np.float32)
    for k, i in enumerate(frames):
        A = mr.UNITY_TO_GLTF @ al.M[i] @ back
        out[k, :, :3] = A
        out[k, :, 3] = mr.UNITY_TO_GLTF @ al.t
    return out


def take_data(take, al, fit, bm, red, step, out_dir, contact_hulls, gates=()):
    inv, faces, n_red = red
    d = TakeData(take)
    F = d.F
    frames = np.arange(0, F, step)
    dev = bm.device
    P = torch.tensor(fit["poses"], device=dev)
    tr = torch.tensor(fit["trans"], device=dev)
    betas = torch.tensor(fit["betas"][None], device=dev)
    vids = torch.tensor(fit["anchor_vids"], device=dev)
    offs = torch.tensor(fit["anchor_offsets"], device=dev)
    pico = torch.tensor(fit["pico_offsets"], device=dev)
    inv_t = torch.tensor(inv, device=dev)
    cnt = torch.bincount(inv_t, minlength=n_red).float()[None, :, None]
    meshes, anchors = [], []
    with torch.no_grad():
        for a in range(0, len(frames), 128):
            fr = torch.tensor(frames[a:a + 128], device=dev)
            Rg = sm.axis_angle_to_matrix(P[fr, :3])
            Rb = sm.axis_angle_to_matrix(P[fr, 3:66].reshape(-1, 21, 3))
            v, _ = bm.forward_full(betas, Rg, Rb, tr[fr])
            red_v = torch.zeros(len(fr), n_red, 3, device=dev).index_add_(1, inv_t, v) / cnt
            meshes.append(red_v.cpu().numpy())
            anchors.append(bm.anchors(betas, Rg, Rb, tr[fr], vids, offs, sm.HEAD, pico).cpu().numpy())
    mesh = np.concatenate(meshes)
    anc = np.concatenate(anchors)                              # (n, M+4, 3)
    M = len(fit["anchor_vids"])
    res = fit["residual_m"][frames]                            # (n, M)
    resp = fit["pico_residual_m"][frames]                      # (n, 4)
    resid = np.concatenate([res, resp], 1) * 1000
    resid_u8 = np.where(np.isfinite(resid), np.clip(np.round(resid), 0, 254), 255).astype(np.uint8)
    # observed points (body + headset), variable count per frame, and which observed point each anchor matched
    obs, counts, match = [], [], np.full((len(frames), M + 4), -1, np.int16)
    for k, i in enumerate(frames):
        ok = np.isfinite(d.pico[i]).all(1)
        o = np.concatenate([d.pts[i], d.pico[i][ok]])
        obs.append(o)
        counts.append(len(o))
        match[k, :M] = fit["matches"][i]
        nb = len(d.pts[i])
        match[k, M:][ok] = nb + np.arange(ok.sum())
    T = course_transform(al, frames)
    # body mesh vs course hulls at the VR placement (reduced mesh, so a lower bound on contact)
    depth = np.zeros(len(frames), np.float32)
    for k in range(len(frames)):
        Vs = mesh[k] @ T[k, :, :3].T + T[k, :, 3]
        dmin = 0.0
        for n_, dd, (lo, hi) in contact_hulls:
            near = np.all((Vs > lo - 0.02) & (Vs < hi + 0.02), axis=1)
            if near.any():
                sd = (Vs[near] @ n_.T - dd).max(1)
                dmin = min(dmin, float(sd.min()))
        depth[k] = -dmin
    blob = Blob()
    blob.add("mesh", mm16(mesh), delta=True)
    blob.add("course", T)
    blob.add("obs_n", np.array(counts, np.uint8))
    blob.add("obs", mm16(np.concatenate(obs)))
    blob.add("anchors", mm16(anc), delta=True)
    blob.add("resid", resid_u8)
    blob.add("match", match)
    blob.add("depth_mm", np.round(depth * 1000).astype(np.int16))
    name = take.name
    index = blob.write(os.path.join(out_dir, "data", f"{name}.json"))
    cen = np.einsum("nij,nj->ni", T[:, :, :3], mesh.mean(1)) + T[:, :, 3]     # body centre in the scene
    key = {}
    for g in gates:
        c = np.asarray(g.get("centre_gltf", [np.nan] * 3), float)
        dist = np.linalg.norm(cen[:, [0, 2]] - c[[0, 2]], axis=1)
        if np.isfinite(dist).any() and dist.min() < 0.35:
            key[g["mode"]] = int(np.argmin(dist))
    per_frame_med = np.nanmedian(fit["residual_m"], 1) * 1000
    return {"name": name, "file": f"data/{name}.json", "index": index, "frames": int(len(frames)), "step": int(step),
            "fps": 100 / step, "n_anchor": int(M), "n_verts": n_red, "duration_s": round(F / 100, 2),
            "median_resid_mm": [None if not np.isfinite(x) else round(float(x), 2) for x in per_frame_med],
            "key_frames": key,
            "max_depth_mm": float(depth.max() * 1000)}


def env_blob(env, out_dir, with_room):
    blob, parts, room = Blob(), [], None
    coll_V, coll_F, n = [], [], 0
    for p in env.parts:
        if p.kind == "room":
            room = p
            continue
        if p.kind == "collision":
            coll_V.append(p.V)
            coll_F.append(p.F + n)
            n += len(p.V)
            continue
        blob.add(p.name + ".V", p.V.astype(np.float32))
        blob.add(p.name + ".F", p.F.astype(np.uint32))
        blob.add(p.name + ".C", p.C.astype(np.uint8))
        parts.append({"name": p.name, "kind": p.kind})
    idx = blob.write(os.path.join(out_dir, "data", "env.json"))
    out = {"file": "data/env.json", "index": idx, "parts": parts, "route": env.route_gltf.round(4).tolist()}
    if with_room and room is not None:
        rb = Blob()
        rb.add("room.V", room.V.astype(np.float32))
        rb.add("room.F", room.F.astype(np.uint32))
        rb.add("room.C", room.C.astype(np.uint8))
        out["room"] = {"file": "data/room.json", "index": rb.write(os.path.join(out_dir, "data", "room.json"))}
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--fits", default=os.path.join(HERE, "data", "out", "smplx"))
    ap.add_argument("--env", default=se.DEFAULT_ENV)
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "site"))
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--with-room", action="store_true")
    a = ap.parse_args(argv)
    os.makedirs(os.path.join(a.out, "data"), exist_ok=True)
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    bm = sm.SMPLX("neutral", dev)
    red = mesh_reduction(bm)
    env = se.load_environment(a.env, room_cell=0.025)
    import contact_check as cc
    hulls, _ = cc.hull_groups(a.env)
    gate_hulls = [h for item, hs in hulls.items() for h in hs]
    logs = mr.parse_editor_log(os.path.join(a.root, "Editor.log"))
    takes = []
    step = max(1, round(100 / a.fps))
    for p in mr.find_takes(a.root):
        take = mr.load_take(p)
        fpath = os.path.join(a.fits, f"{take.name}.npz")
        if take.log_name not in logs or not os.path.exists(fpath):
            continue
        al = mr.estimate_alignment(take, logs[take.log_name])
        fit = np.load(fpath)
        info = take_data(take, al, fit, bm, red, step, a.out, gate_hulls, env.gates)
        with open(os.path.join(a.fits, f"{take.name}.json")) as f:
            info["report"] = json.load(f)
        info["scale"] = round(float(np.median(al.scale)), 4)
        info["label"] = take.name
        takes.append(info)
        print(f"{take.name}: {info['frames']} frames, max body-in-obstacle depth {info['max_depth_mm']:.0f} mm", flush=True)
    meta = {"takes": takes, "env": env_blob(env, a.out, a.with_room), "faces": None,
            "built": datetime.datetime.now().isoformat(timespec="minutes"), "mesh_cell_m": MESH_CELL_M}
    faces = red[1]
    fb = Blob()
    fb.add("faces", faces)
    meta["faces"] = {"file": "data/body_faces.json", "index": fb.write(os.path.join(a.out, "data", "body_faces.json")),
                     "n_verts": red[2]}
    with open(os.path.join(HERE, "fit_site_template.html")) as f:
        html = f.read()
    with open(os.path.join(a.out, "index.html"), "w") as f:
        f.write(html.replace("/*META*/", json.dumps(meta, separators=(",", ":"))))
    sizes = {n: os.path.getsize(os.path.join(a.out, "data", n)) for n in os.listdir(os.path.join(a.out, "data"))}
    print("wrote", a.out, {k: f"{v / 1e6:.1f} MB" for k, v in sizes.items()})
    return 0


if __name__ == "__main__":
    sys.exit(main())
