"""Find sudden jumps of the fitted arm pose and look at the markers behind them.

    PYTHONNOUSERSITE=1 python analysis/check_fit_jumps.py analysis/data/raw/2026-10-30_recordings/recordings \
        [--takes jumping] [--fits analysis/data/out/smplx] [--out analysis/data/out/jumps]

Per take:
- angular speed of each arm joint (collar, shoulder, elbow, wrist; rotation relative to its parent) from frame to
  frame, and frames above --deg-per-frame (default 8 deg in 10 ms = 800 deg/s);
- which anchors sit on each arm segment (by the joint that skins their vertex most) and how many: a segment needs
  3 markers that are not on one line to fix its rotation including the twist about its own axis;
- around each jump: per arm anchor, matched or not, residual, whether the matched point is the same Motive
  trajectory as in the frame before, and whether it came from the stray v4w2 rigid body.
Writes <out>/<take>_arm_jumps.json and <out>/<take>_arm_jumps.png.
"""
import argparse
import json
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import mocaprec as mr  # noqa: E402
import smplx_model as sm  # noqa: E402
from fit_smplx import TakeData  # noqa: E402

ARM = {"left": {"collar": 13, "shoulder": 16, "elbow": 18, "wrist": 20},
       "right": {"collar": 14, "shoulder": 17, "elbow": 19, "wrist": 21}}
# segment driven by each joint: the bone that the joint rotates
SEGMENT = {"collar": "clavicle", "shoulder": "upper arm", "elbow": "forearm", "wrist": "hand"}


def rel_angle_deg(Ra, Rb):
    rel = np.einsum("...ji,...jk->...ik", Ra, Rb)
    c = np.clip((np.trace(rel, axis1=-2, axis2=-1) - 1) / 2, -1, 1)
    return np.degrees(np.arccos(c))


def anchors_by_segment(bm, vids):
    w = bm.weights[torch.as_tensor(vids, device=bm.device)].cpu().numpy()
    dom = w.argmax(-1)
    out = {}
    for side, joints in ARM.items():
        for name, j in joints.items():
            hand_joints = list(range(25, 40)) if side == "left" else list(range(40, 55))
            sel = np.where((dom == j) | (np.isin(dom, hand_joints) if name == "wrist" else False))[0]
            out[(side, name)] = sel.tolist()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root")
    ap.add_argument("--takes", nargs="*", default=["jumping"])
    ap.add_argument("--fits", default=os.path.join(HERE, "data", "out", "smplx"))
    ap.add_argument("--out", default=os.path.join(HERE, "data", "out", "jumps"))
    ap.add_argument("--deg-per-frame", type=float, default=8.0)
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    bm = sm.SMPLX("neutral", "cpu")
    for p in mr.find_takes(a.root):
        take = mr.load_take(p)
        if a.takes and take.name not in a.takes:
            continue
        fit = np.load(os.path.join(a.fits, f"{take.name}.npz"))
        d = TakeData(take)
        P = torch.tensor(fit["poses"])
        Rb = sm.axis_angle_to_matrix(P[:, 3:66].reshape(-1, 21, 3)).numpy()       # body joints 1..21
        F = len(Rb)
        seg = anchors_by_segment(bm, fit["anchor_vids"])
        res = fit["residual_m"] * 1000
        M = fit["matches"]
        report = {"take": take.name, "frames": F, "threshold_deg_per_frame": a.deg_per_frame, "arms": {}}
        speeds = {}
        for side, joints in ARM.items():
            for name, j in joints.items():
                R = Rb[:, j - 1]
                sp = np.r_[0, rel_angle_deg(R[:-1], R[1:])]
                speeds[(side, name)] = sp
                jump = np.where(sp > a.deg_per_frame)[0]
                anchors = seg[(side, name)]
                entry = {"segment": SEGMENT[name], "markers_on_segment": len(anchors), "anchor_ids": anchors,
                         "speed_deg_per_frame": {"p50": round(float(np.median(sp)), 2), "p99": round(float(np.percentile(sp, 99)), 2),
                                                 "max": round(float(sp.max()), 1)},
                         "jump_frames": int(len(jump)), "jumps": []}
                # group consecutive jump frames into events
                events = []
                for f in jump:
                    if events and f - events[-1][-1] <= 3:
                        events[-1].append(int(f))
                    else:
                        events.append([int(f)])
                for ev in events[:12]:
                    f0, f1 = ev[0], ev[-1]
                    w0, w1 = max(1, f0 - 3), min(F - 1, f1 + 3)
                    limb = sorted(set(seg[(side, "shoulder")] + seg[(side, "elbow")] + seg[(side, "wrist")]))
                    detail = []
                    for k in limb:
                        miss = int(sum(M[t, k] < 0 for t in range(w0, w1 + 1)))
                        swaps, from_v4w2 = 0, 0
                        for t in range(w0, w1 + 1):
                            if M[t, k] >= 0 and M[t - 1, k] >= 0:
                                id_now, id_prev = d.ids[t][M[t, k]], d.ids[t - 1][M[t - 1, k]]
                                if id_now >= 0 and id_prev >= 0 and id_now != id_prev:
                                    swaps += 1
                            if M[t, k] >= 0 and d.ids[t][M[t, k]] < 0:
                                from_v4w2 += 1
                        r = res[w0:w1 + 1, k]
                        detail.append({"anchor": k, "missing_frames": miss, "motive_id_changes": swaps,
                                       "frames_from_v4w2": from_v4w2,
                                       "residual_mm_max": None if np.all(np.isnan(r)) else round(float(np.nanmax(r)), 1)})
                    entry["jumps"].append({"frames": [f0, f1], "t_s": round(f0 / 100, 2), "peak_deg": round(float(sp[f0:f1 + 1].max()), 1),
                                           "limb_markers": detail})
                report["arms"][f"{side} {name}"] = entry
        with open(os.path.join(a.out, f"{take.name}_arm_jumps.json"), "w") as fh:
            json.dump(report, fh, indent=1)
        plot(take, speeds, res, seg, M, d, a, os.path.join(a.out, f"{take.name}_arm_jumps.png"))
        print(json.dumps({k: {kk: v[kk] for kk in ("markers_on_segment", "speed_deg_per_frame", "jump_frames")}
                          for k, v in report["arms"].items()}, indent=1))
    return 0


def plot(take, speeds, res, seg, M, d, a, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    F = len(next(iter(speeds.values())))
    t = np.arange(F) / 100
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    for ax, side in zip(axes[:2], ("left", "right")):
        for name, c in zip(("shoulder", "elbow", "wrist"), ("#1f77b4", "#ff7f0e", "#2ca02c")):
            ax.plot(t, speeds[(side, name)], color=c, lw=0.9, label=f"{name} ({SEGMENT[name]})")
        ax.axhline(a.deg_per_frame, color="#d62728", ls="--", lw=0.8)
        ax.set_ylabel(f"{side} arm\ndeg per frame"); ax.legend(loc="upper right", fontsize=8); ax.set_ylim(0, None)
    for ax, side in zip(axes[2:], ("left", "right")):
        limb = sorted(set(seg[(side, "shoulder")] + seg[(side, "elbow")] + seg[(side, "wrist")]))
        for k in limb:
            ax.plot(t, res[:, k], lw=0.8, label=f"anchor {k}")
            miss = np.where(M[:, k] < 0)[0]
            ax.plot(miss / 100, np.full(len(miss), -3), "|", ms=6)
        ax.set_ylabel(f"{side} arm markers\nresidual mm"); ax.legend(loc="upper right", fontsize=7, ncol=3); ax.set_ylim(-6, 60)
    axes[-1].set_xlabel("time (s)  |  ticks below 0 = marker not matched in that frame")
    fig.suptitle(f"{take.name}: arm joint angular speed and arm-marker residuals (fit at 100 Hz)")
    fig.tight_layout()
    fig.savefig(path, dpi=90)


if __name__ == "__main__":
    sys.exit(main())
