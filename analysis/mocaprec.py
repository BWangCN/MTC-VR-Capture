"""Native Motive takes (Motive_Client recorder, schema mtc_natnet_recording_v1) and their place in the VR scene.

A take folder holds frames.jsonl (every NatNet frame, native Motive coordinates: right-handed, Z up, metres),
filtered_frames.jsonl (same frames, markers cropped to the headset box when B was on, plus Unity's
native->headset-relative transform), pico.csv, metadata.json and summary.json (Motive_Client/README.md).

Where the markers were in the Unity scene is not stored in the take. Unity showed a marker at
    u = M p + t_align        (OptiTrackPointReceiver.MotivePointToWorld; Unity world, left-handed, Y up)
with M = R_yaw * positionScale * C (C: Motive Z-up -> Unity, (x, y, z) -> (-y, z, x)). The take stores, at 20 Hz,
T = [M | t_align - c] with c the headset camera position in Unity (filtered frames, `motive_to_headset_delta`).
The Unity Editor.log of the session logs c every 60 Unity frames ("[XR Camera] Pos", 2 decimals), and once per
second the Motive frame Unity had last received ("OptiTrack | ... last frame N"). The two are not logged in the same
update, so `estimate_alignment` places each camera line in the frame window between its neighbouring OptiTrack lines
and matches it to the mapped Pico rigid body there.

The Unity world of the obstacle-course scene equals the scene's glTF frame with x mirrored (environment root at
the origin, scale 1: Assets/RealEnvs/7b6477cb95_seed0/README.md); `unity_to_gltf` converts.
"""
import csv
import json
import os
import re
from dataclasses import dataclass, field

import numpy as np

PICO_RB_ID = 20                      # metadata: rigid body "Pico"
UNLABELED_FLAG = 0x10                # NatNet labeled-marker params bit: unlabeled point
UNITY_TO_GLTF = np.diag([-1.0, 1.0, 1.0])


# ------------------------------------------------------------------------------------------------ takes
@dataclass
class Take:
    name: str                        # folder name
    path: str
    meta: dict
    summary: dict
    frame: np.ndarray                # (F,) Motive frame ids
    t: np.ndarray                    # (F,) seconds since the first frame (Motive timestamp)
    unl_pos: list                    # per frame: (n, 3) unlabeled markers, native
    unl_id: list                     # per frame: (n,) Motive trajectory ids of those markers
    unl_kept: list                   # per frame: (n,) bool, kept by the headset-box crop (all True when it was off)
    rb: dict                         # rigid-body id -> {"name", "pos" (F, 3), "quat_xyzw" (F, 4), "err" (F,)}; NaN = missing
    rb_obs: dict                     # rigid-body id -> (F, n_members, 3): member markers Motive actually saw, NaN = not seen
    sets: dict                       # marker-set name -> (F, n, 3): NatNet marker sets, which fill unseen rigid-body
                                     # members with the model's expected position (do not use them as observations)
    T: np.ndarray                    # (F, 3, 4) native -> Unity headset-relative transform; NaN rows = invalid
    filt_on: np.ndarray              # (F,) bool, crop enabled
    rb_names: dict = field(default_factory=dict)

    @property
    def started_ns(self):
        return int(self.meta["started_unix_ns"])

    @property
    def log_name(self):
        """Folder name the recorder gave the take (the folder may have been renamed since)."""
        return f"take_{self.started_ns}"

    def index_of(self, frame_id):
        i = int(frame_id) - int(self.frame[0])
        return i if 0 <= i < len(self.frame) and self.frame[i] == frame_id else None


def _vec(v, n):
    return np.array([np.nan if x is None else x for x in v], float) if v is not None else np.full(n, np.nan)


def load_take(path):
    """Read one take folder."""
    with open(os.path.join(path, "metadata.json")) as f:
        meta = json.load(f)
    with open(os.path.join(path, "summary.json")) as f:
        summary = json.load(f)
    models = meta["source"]["models"]
    rb_names = {m["id"]: m["name"] for m in models if m["type"] == 1}
    set_names = [m["name"] for m in models if m["type"] == 0 and m["name"] != "all"]

    frame, ts, unl_pos, unl_id = [], [], [], []
    rb = {i: {"name": n, "pos": [], "quat_xyzw": [], "err": []} for i, n in rb_names.items()}
    n_members = {m["id"]: len(m["markers"]) for m in models if m["type"] == 1}
    rb_obs = {i: [] for i in rb_names}
    sets = {n: [] for n in set_names}
    with open(os.path.join(path, "frames.jsonl")) as f:
        for line in f:
            d = json.loads(line)
            frame.append(d["frame"])
            ts.append(d["timestamp"])
            ids, pos = [], []
            obs = {i: np.full((n_members[i], 3), np.nan) for i in rb_obs}
            for m in d["labeled_markers"]:
                if m["params"] & UNLABELED_FLAG:             # Motive trajectory ids pass 65535, so the id's
                    ids.append(m["id"])                      # high bits are not an asset id for these
                    pos.append(_vec(m["position"], 3))
                elif m["id"] >> 16 in obs and 1 <= (m["id"] & 0xFFFF) <= n_members[m["id"] >> 16]:
                    obs[m["id"] >> 16][(m["id"] & 0xFFFF) - 1] = _vec(m["position"], 3)   # member ids are 1-based
            for i in rb_obs:
                rb_obs[i].append(obs[i])
            unl_id.append(np.array(ids, np.int64))
            unl_pos.append(np.array(pos, float).reshape(-1, 3))
            seen = set()
            for r in d["rigid_bodies"]:
                if r["id"] in rb:
                    rb[r["id"]]["pos"].append(_vec(r["position"], 3))
                    rb[r["id"]]["quat_xyzw"].append(_vec(r["quat_xyzw"] if "quat_xyzw" in r else r["quaternion_xyzw"], 4))
                    rb[r["id"]]["err"].append(np.nan if r["mean_error"] is None else r["mean_error"])
                    seen.add(r["id"])
            for i in rb:
                if i not in seen:
                    rb[i]["pos"].append(np.full(3, np.nan))
                    rb[i]["quat_xyzw"].append(np.full(4, np.nan))
                    rb[i]["err"].append(np.nan)
            ms = {s["name"]: s["positions"] for s in d["marker_sets"]}
            for n in set_names:
                sets[n].append(np.array([_vec(p, 3) for p in ms.get(n, [])], float).reshape(-1, 3))

    F = len(frame)
    T = np.full((F, 3, 4), np.nan)
    filt_on = np.zeros(F, bool)
    unl_kept = [np.ones(len(x), bool) for x in unl_id]
    fpath = os.path.join(path, "filtered_frames.jsonl")
    if os.path.exists(fpath):
        with open(fpath) as f:
            for i, line in enumerate(f):
                d = json.loads(line)
                assert d["frame"] == frame[i], "filtered_frames.jsonl is not frame-aligned with frames.jsonl"
                bf = d.get("body_filter") or {}
                filt_on[i] = bool(bf.get("enabled"))
                m = bf.get("motive_to_headset_delta")
                if bf.get("valid") and m is not None and None not in m:
                    T[i] = np.array(m, float).reshape(3, 4)
                kept = {m["id"] for m in d["labeled_markers"] if m["params"] & UNLABELED_FLAG}
                unl_kept[i] = np.isin(unl_id[i], list(kept))

    for i in rb:
        for k in ("pos", "quat_xyzw", "err"):
            rb[i][k] = np.array(rb[i][k], float)
    ts = np.array(ts, float)
    return Take(os.path.basename(os.path.normpath(path)), path, meta, summary, np.array(frame, np.int64), ts - ts[0],
                unl_pos, unl_id, unl_kept, rb, {i: np.stack(v) for i, v in rb_obs.items()},
                {n: np.stack(v) if v and len({x.shape for x in v}) == 1 else v for n, v in sets.items()},
                T, filt_on, rb_names)


def find_takes(root):
    """Take folders (those with a metadata.json) under root, sorted by start time."""
    out = []
    for n in sorted(os.listdir(root)):
        p = os.path.join(root, n)
        if os.path.isfile(os.path.join(p, "metadata.json")):
            with open(os.path.join(p, "metadata.json")) as f:
                out.append((int(json.load(f)["started_unix_ns"]), p))
    return [p for _, p in sorted(out)]


# ------------------------------------------------------------------------------------------------ Editor.log
RE_ON = re.compile(r"^Mocap recording: ON \| \d+ frames \| .*[\\/](take_\d+)\s*$")
RE_OFF = re.compile(r"^Mocap recording: OFF \| (\d+) frames \| .*[\\/](take_\d+)\s*$")
RE_XR = re.compile(r"^\[XR Camera\] Pos: \(([-\d.]+), ([-\d.]+), ([-\d.]+)\), Rot: \(([-\d.]+), ([-\d.]+), ([-\d.]+)\)")
RE_OPTI = re.compile(r"^OptiTrack \|.*\| last frame (\d+) \|.*\| local frame age ([-\d.]+) ms \| "
                     r"alignment error ([-\d.]+) m")
RE_SCALE = re.compile(r"^Player resized uniformly: headset ([\d.]+) m -> ([\d.]+) m, scale x([\d.]+)\.")
RE_FILTER = re.compile(r"^Mocap marker filter: (ON|OFF) ")


@dataclass
class LogTake:
    name: str                        # take_<started_unix_ns>
    line_on: int
    line_off: int
    frames_logged: int
    scale: float                     # last "Player resized" before the take (None if never)
    scale_line: int
    filter_state: str                # last "Mocap marker filter" before the take
    xr: list                         # [(line, (x, y, z) Unity camera position, (ex, ey, ez))]
    opti: list                       # [(line, last_frame, local_age_ms, alignment_error_m)]


def parse_editor_log(path):
    """Takes found in a Unity Editor.log, with the camera and OptiTrack lines logged while each was recording."""
    with open(path, encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()
    out, cur, scale, scale_line, filt = {}, None, None, None, "unknown"
    for ln, s in enumerate(lines, 1):
        m = RE_SCALE.match(s)
        if m:
            scale, scale_line = float(m.group(3)), ln
            continue
        m = RE_FILTER.match(s)
        if m:
            filt = m.group(1)
            continue
        m = RE_ON.match(s)
        if m:
            cur = LogTake(m.group(1), ln, None, None, scale, scale_line, filt, [], [])
            continue
        m = RE_OFF.match(s)
        if m and cur is not None and m.group(2) == cur.name:
            cur.line_off, cur.frames_logged = ln, int(m.group(1))
            out[cur.name] = cur
            cur = None
            continue
        if cur is None:
            continue
        m = RE_XR.match(s)
        if m:
            v = [float(x) for x in m.groups()]
            cur.xr.append((ln, tuple(v[:3]), tuple(v[3:])))
            continue
        m = RE_OPTI.match(s)
        if m:
            cur.opti.append((ln, int(m.group(1)), float(m.group(2)), float(m.group(3))))
    return out


# ------------------------------------------------------------------------------------------------ alignment
@dataclass
class Alignment:
    """Native Motive -> Unity world for one take: u = M[i] p + t (M per frame from the take, t from the log)."""
    M: np.ndarray                    # (F, 3, 3)
    t: np.ndarray                    # (3,) Unity world
    scale: np.ndarray                # (F,) |M| (positionScale Unity applied)
    yaw_deg: np.ndarray              # (F,) yaw of M about Unity up
    samples: list                    # per [XR Camera] line: dict(index, window, cam, residual_m, t_from_T)
    frames_with_T: float             # fraction of frames with a valid T

    def to_unity(self, i, P):
        return np.asarray(P, float) @ self.M[i].T + self.t

    def to_gltf(self, i, P):
        return unity_to_gltf(self.to_unity(i, P))

    @property
    def residuals(self):
        return np.array([s["residual_m"] for s in self.samples])


def unity_to_gltf(U):
    return np.asarray(U, float) @ UNITY_TO_GLTF.T


def _fill_nan_rows(T):
    """Forward/backward fill frames without a valid transform."""
    ok = np.isfinite(T.reshape(len(T), -1)).all(1)
    if not ok.any():
        raise ValueError("no valid native->headset transform in this take")
    idx = np.where(ok, np.arange(len(T)), -1)
    np.maximum.accumulate(idx, out=idx)
    idx[idx < 0] = np.argmax(ok)
    return T[idx], ok.mean()


def estimate_alignment(take, logtake, pad_frames=5, iters=8):
    """t_align from the logged Unity camera positions.

    Each [XR Camera] line was logged between two OptiTrack lines (log order = time order), so its moment lies in the
    frame window [last frame of the line before, last frame of the line after] (+- pad). Within that window the camera
    is matched to the closest mapped Pico rigid-body position M p (Unity aligns that rigid body onto the camera,
    cameraOffsetFromRigidBody = 0 in the course scene); t = median(c - M p) over all lines, iterated.
    """
    T, frac = _fill_nan_rows(take.T)
    M = T[:, :, :3]
    pico = take.rb[PICO_RB_ID]["pos"]
    ok = np.isfinite(pico).all(1)
    Q = np.einsum("fij,fj->fi", M, np.where(ok[:, None], pico, 0.0))
    o_line = np.array([o[0] for o in logtake.opti])
    o_frame = np.array([o[1] for o in logtake.opti])
    f0, F = int(take.frame[0]), len(take.frame)
    win = []
    for ln, cam, rot in logtake.xr:
        j = int(np.searchsorted(o_line, ln))
        lo = o_frame[j - 1] - f0 if j > 0 else 0
        hi = o_frame[j] - f0 if j < len(o_frame) else F - 1
        i0, i1 = max(0, lo - pad_frames), min(F - 1, hi + pad_frames)
        if i1 < i0 or not ok[i0:i1 + 1].any():
            continue
        win.append((np.array(cam, float), i0, i1, rot))
    if not win:
        raise ValueError(f"{take.name}: no [XR Camera] line inside the take")
    t = np.median([c - Q[(i0 + i1) // 2] for c, i0, i1, _ in win], 0)
    for _ in range(iters):
        best = []
        for c, i0, i1, _ in win:
            d = np.linalg.norm(Q[i0:i1 + 1] + t - c, axis=1)
            d[~ok[i0:i1 + 1]] = np.inf
            best.append(i0 + int(np.argmin(d)))
        t_new = np.median([c - Q[i] for (c, _, _, _), i in zip(win, best)], 0)
        if np.allclose(t_new, t, atol=1e-5):
            break
        t = t_new
    samples = [{"index": i, "frame": int(take.frame[i]), "window": (i0, i1), "cam": c, "cam_euler_deg": rot,
                "residual_m": float(np.linalg.norm(Q[i] + t - c)), "t_from_T": T[i, :, 3] + c}
               for (c, i0, i1, rot), i in zip(win, best)]
    scale = np.linalg.norm(M[:, :, 0], axis=1)      # every column of R_yaw * s * C has length s
    ex = M[:, :, 0] / scale[:, None]                 # where Motive +X points in Unity
    yaw = np.degrees(np.arctan2(ex[:, 0], ex[:, 2]))
    return Alignment(M, t, scale, yaw, samples, frac)
