"""One self-contained HTML page: the obstacle-course scene with every take's markers placed as Unity showed them.

    python analysis/build_viewer.py analysis/data/raw/2026-10-30_recordings/recordings \
        [--env VR_G1_RealTime_PC/Assets/RealEnvs/7b6477cb95_seed0] [--out analysis/data/out/viewer]

Markers are mapped with mocaprec.estimate_alignment (Unity's own native->Unity transform from the take, the
translation from the session's Editor.log) and converted to the scene's glTF frame. Positions are stored as int16
millimetres. The page loads three.js from jsDelivr. The room is a ScanNet++ scan (research use only, not to be
redistributed): keep the page local. With --fits the SMPL-X fits are added as a reduced, posed body mesh per
video frame (every --body-step frames), placed as VR showed the person (player scale, alignment).
"""
import argparse
import base64
import datetime
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mocaprec as mr  # noqa: E402
import scene_env as se  # noqa: E402

MISSING = -32768
MARKER_RADIUS_DISPLAY_M = 0.009      # drawn sphere radius (true 14 mm markers have 7 mm)


def b64(a):
    return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode("ascii")


def mm(P):
    """glTF metres -> int16 millimetres; NaN -> MISSING."""
    P = np.asarray(P, float)
    out = np.clip(np.round(P * 1000), -32767, 32767)
    out[~np.isfinite(P)] = MISSING
    return out.astype(np.int16)


def _np_json(o):
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def env_payload(env):
    parts, coll_V, coll_F, n = [], [], [], 0
    for p in env.parts:
        if p.kind == "collision":
            coll_V.append(p.V)
            coll_F.append(p.F + n)
            n += len(p.V)
            continue
        parts.append({"name": p.name, "kind": p.kind, "V": b64(p.V.astype(np.float32)), "F": b64(p.F.astype(np.uint32)),
                      "C": b64(p.C.astype(np.uint8))})
    if coll_V:
        parts.append({"name": "collision", "kind": "collision", "V": b64(np.concatenate(coll_V).astype(np.float32)),
                      "F": b64(np.concatenate(coll_F).astype(np.uint32))})
    return {"parts": parts, "route": env.route_gltf.tolist(), "gates": env.gates, "meta": env.meta}


def take_label(take):
    t = datetime.datetime.fromtimestamp(take.started_ns / 1e9, datetime.timezone.utc).strftime("%H:%M:%S UTC")
    return f"{take.name} ({t})" if take.name == take.log_name else f"{take.name} = {take.log_name} ({t})"


def take_payload(take, lt, al):
    F = len(take.frame)
    n = np.array([len(x) for x in take.unl_id], np.uint16)
    unl, flags, ids, lowest = [], [], [], np.full(F, np.nan)
    low_true = np.full(F, np.nan)
    for i in range(F):
        P = take.unl_pos[i]
        g = al.to_gltf(i, P) if len(P) else np.zeros((0, 3))
        kept = take.unl_kept[i]
        unl.append(g)
        flags.append(kept.astype(np.uint8))
        ids.append(take.unl_id[i].astype(np.uint32))
        if kept.any():
            lowest[i] = g[kept, 1].min()
            low_true[i] = P[kept, 2].min()
    unl = np.concatenate(unl) if unl else np.zeros((0, 3))
    other = [i for i in take.rb if i != mr.PICO_RB_ID]

    def obs_mm(rid, k):
        """Observed members of a rigid body, padded to k slots (NaN -> MISSING)."""
        out = np.full((F, k, 3), np.nan)
        if rid is not None:
            O = take.rb_obs[rid]
            n = min(k, O.shape[1])
            out[:, :n] = np.stack([al.to_gltf(i, O[i, :n]) for i in range(F)])
        return mm(out)

    head = np.stack([al.to_gltf(i, take.rb[mr.PICO_RB_ID]["pos"][i]) for i in range(F)])
    res = al.residuals
    s = float(np.median(al.scale))
    pico_err = take.rb[mr.PICO_RB_ID]["err"]
    info = {
        "frames": f"{F} at 100 Hz ({take.t[-1]:.1f} s)",
        "unlabeled / frame": f"median {int(np.median(n))} (range {n.min()}–{n.max()})",
        "trajectory ids": str(len(np.unique(np.concatenate(take.unl_id)))),
        "headset box crop": "on" if take.filt_on.mean() > 0.5 else "off",
        "Unity player scale": f"x{s:.4f}" + (f" (log: x{lt.scale})" if lt.scale else ""),
        "yaw of the mapping": f"{al.yaw_deg.min():.1f}–{al.yaw_deg.max():.1f}°",
        "t_align (Unity)": "(" + ", ".join(f"{x:.3f}" for x in al.t) + ") m",
        "camera log lines": str(len(al.samples)),
        "camera vs headset": f"median {np.median(res) * 100:.1f} cm, max {res.max() * 100:.1f} cm",
        "Unity's alignment error": f"median {np.median([o[3] for o in lt.opti]) * 100:.1f} cm",
        "Pico fit error": f"median {np.nanmedian(pico_err) * 1000:.2f} mm",
        "taken by v4w2": (f"median {np.median(np.isfinite(take.rb_obs[other[0]]).all(2).sum(1)):.0f} body markers / frame"
                          if other else "–"),
        "lowest marker (true)": f"median {np.nanmedian(low_true) * 100:.1f} cm over the lab floor",
    }
    return {"name": take.name, "label": take_label(take), "frames": F, "fps": 100, "info": info,
            "n": b64(n), "unl": b64(mm(unl)), "flags": b64(np.concatenate(flags) if flags else np.zeros(0, np.uint8)),
            "ids": b64(np.concatenate(ids) if ids else np.zeros(0, np.uint32)),
            "pico": b64(obs_mm(mr.PICO_RB_ID, 4)), "v4w2": b64(obs_mm(other[0] if other else None, 8)), "head": b64(mm(head)),
            "lowest": b64(np.where(np.isfinite(lowest), np.clip(np.round(lowest * 1000), -32767, 32767), MISSING).astype(np.int16)),
            "cams": [{"i": int(x["index"]), "p": mr.unity_to_gltf(x["cam"]).round(4).tolist(), "r": round(x["residual_m"], 4)}
                     for x in al.samples]}


METHOD = ("Markers are drawn where the VR app showed them. The native Motive positions are mapped with the "
          "transform Unity used at that frame (rotation and player scale, saved in filtered_frames.jsonl) plus "
          "Unity's alignment translation, recovered from the Editor.log: each logged Unity camera position (yellow) "
          "is matched to the Pico rigid body in the frame window between its neighbouring OptiTrack log lines. "
          "The scene is the course's glTF frame (Unity world with x mirrored). The person is shown at the scale "
          "Unity applied (G1 headset height 1.30 m); the room is metric.")


def body_payload(take, al, fit_path, step, cache):
    """SMPL-X fit of a take as a reduced mesh per video frame, in the scene glTF frame (VR placement and scale),
    int16 mm, frame-delta coded, gzip + base64. None if the take has no fit."""
    if not os.path.exists(fit_path):
        return None
    import gzip
    import torch
    import build_fit_site as bfs
    import smplx_model as sm
    if "bm" not in cache:
        dev = "cuda" if torch.cuda.is_available() else "cpu"
        cache["bm"] = sm.SMPLX("neutral", dev)
        cache["red"] = bfs.mesh_reduction(cache["bm"])
    bm, (inv, faces, n_red) = cache["bm"], cache["red"]
    fit = np.load(fit_path)
    F = len(take.frame)
    frames = np.arange(0, F, step)
    dev = bm.device
    P = torch.tensor(fit["poses"], device=dev)
    tr = torch.tensor(fit["trans"], device=dev)
    betas = torch.tensor(fit["betas"][None], device=dev)
    inv_t = torch.tensor(inv, device=dev)
    cnt = torch.bincount(inv_t, minlength=n_red).float()[None, :, None]
    T = torch.tensor(bfs.course_transform(al, frames), device=dev)
    out = []
    with torch.no_grad():
        for a in range(0, len(frames), 128):
            fr = torch.tensor(frames[a:a + 128], device=dev)
            v, _ = bm.forward_full(betas, sm.axis_angle_to_matrix(P[fr, :3]),
                                   sm.axis_angle_to_matrix(P[fr, 3:66].reshape(-1, 21, 3)), tr[fr])
            red = torch.zeros(len(fr), n_red, 3, device=dev).index_add_(1, inv_t, v) / cnt
            Tk = T[a:a + 128]
            out.append((torch.einsum("nij,nvj->nvi", Tk[:, :, :3], red) + Tk[:, None, :, 3]).cpu().numpy())
    V = np.concatenate(out)
    q = np.clip(np.round(V * 1000), -32767, 32767).astype(np.int32)
    q[1:] = q[1:] - q[:-1]
    data = base64.b64encode(gzip.compress(q.astype(np.int16).tobytes(), 9)).decode("ascii")
    cache["faces"], cache["n_verts"] = faces, n_red
    return {"step": int(step), "frames": int(len(frames)), "gz64": data}


def build(root, env_path, out_dir, room_cell, fits_dir=None, body_step=4):
    env = se.load_environment(env_path, room_cell=room_cell)
    cache = {}
    logs = mr.parse_editor_log(os.path.join(root, "Editor.log"))
    takes = []
    for p in mr.find_takes(root):
        take = mr.load_take(p)
        lt = logs.get(take.log_name)
        if lt is None:
            print(f"{take.name}: not in the Editor.log, skipped")
            continue
        al = mr.estimate_alignment(take, lt)
        takes.append(take_payload(take, lt, al))
        if fits_dir:
            body = body_payload(take, al, os.path.join(fits_dir, f"{take.name}.npz"), body_step, cache)
            if body:
                takes[-1]["body"] = body
        print(f"{take.name}: {len(take.frame)} frames, camera residual median {np.median(al.residuals) * 100:.1f} cm")
    payload = {"env": env_payload(env), "takes": takes, "marker_radius_display_m": MARKER_RADIUS_DISPLAY_M,
               "method": METHOD, "built": datetime.datetime.now().isoformat(timespec="seconds")}
    if "faces" in cache:
        payload["body_faces"] = b64(cache["faces"].astype(np.uint16))
        payload["body_n_verts"] = int(cache["n_verts"])
    with open(os.path.join(HERE, "viewer_template.html")) as f:
        html = f.read()
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, "mocap_scene_viewer.html")
    with open(out, "w") as f:
        f.write(html.replace("/*PAYLOAD*/", json.dumps(payload, separators=(",", ":"), default=_np_json)))
    print(f"wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--env", default=se.DEFAULT_ENV)
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "viewer"))
    ap.add_argument("--room-cell", type=float, default=0.025, help="room decimation grid (m)")
    ap.add_argument("--fits", default=None, help="folder with fit_smplx.py results: adds the SMPL-X body")
    ap.add_argument("--body-step", type=int, default=4, help="body mesh every N frames of the 100 Hz take")
    a = ap.parse_args(argv)
    build(a.root, a.env, a.out, a.room_cell, a.fits, a.body_step)
    return 0


if __name__ == "__main__":
    sys.exit(main())
