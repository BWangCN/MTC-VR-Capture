"""Per-take summary and the native->Unity alignment recovered from the session's Editor.log.

    python analysis/inspect_takes.py analysis/data/raw/2026-10-30_recordings/recordings [--out analysis/data/out]

Prints, per take: frames, duration, unlabeled-marker counts, rigid-body tracking, the Unity scale and yaw of the
mapping, the translation t_align estimated from each [XR Camera]/OptiTrack log pair, and how far the logged Unity
camera lies from the mapped Pico rigid body. Writes the same to <out>/alignment.json (read by the viewers).
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mocaprec as mr  # noqa: E402


def summarize(take, lt, al):
    n_unl = np.array([len(x) for x in take.unl_id])
    res = al.residuals
    tT = np.array([x["t_from_T"] for x in al.samples])
    lag = np.array([x["index"] - (x["window"][0] + x["window"][1]) / 2 for x in al.samples])
    return {
        "take": take.name, "log_name": take.log_name, "frames": int(len(take.frame)),
        "duration_s": round(float(take.t[-1]), 2), "summary": take.summary,
        "unlabeled_per_frame": {"median": int(np.median(n_unl)), "min": int(n_unl.min()), "max": int(n_unl.max())},
        "unique_unlabeled_ids": int(len(np.unique(np.concatenate(take.unl_id)))),
        "rigid_bodies": {take.rb_names[i]: {"tracked_frac": round(float(np.isfinite(r["pos"]).all(1).mean()), 4),
                                            "mean_error_mm_median": round(float(np.nanmedian(r["err"]) * 1000), 3)}
                         for i, r in take.rb.items()},
        "crop_enabled_frac": round(float(take.filt_on.mean()), 3),
        "log": {"scale_logged": lt.scale, "scale_line": lt.scale_line, "filter_state_logged": lt.filter_state,
                "frames_logged": lt.frames_logged, "xr_camera_lines": len(lt.xr), "optitrack_lines": len(lt.opti),
                "camera_lines_used": len(al.samples)},
        "alignment": {
            "M_scale": [round(float(al.scale.min()), 5), round(float(al.scale.max()), 5)],
            "yaw_deg": [round(float(al.yaw_deg.min()), 2), round(float(al.yaw_deg.max()), 2)],
            "frames_with_T": round(al.frames_with_T, 4),
            "t_unity": [round(float(x), 4) for x in al.t],
            "camera_residual_m": {"median": round(float(np.median(res)), 4), "p90": round(float(np.percentile(res, 90)), 4),
                                  "max": round(float(res.max()), 4)},
            "t_from_T_at_matched_frame_minus_t_m": {
                "median": round(float(np.median(np.linalg.norm(tT - al.t, axis=1))), 4),
                "max": round(float(np.linalg.norm(tT - al.t, axis=1).max()), 4)},
            "matched_frame_offset_from_window_centre": [int(lag.min()), int(np.median(lag)), int(lag.max())],
            "logged_alignment_error_m": {"median": round(float(np.median([o[3] for o in lt.opti])), 4),
                                         "max": round(float(max(o[3] for o in lt.opti)), 4)},
        },
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", help="folder with the take folders and Editor.log")
    ap.add_argument("--log", default=None, help="Editor.log (default: <root>/Editor.log)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "out"))
    a = ap.parse_args(argv)
    logs = mr.parse_editor_log(a.log or os.path.join(a.root, "Editor.log"))
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for p in mr.find_takes(a.root):
        take = mr.load_take(p)
        lt = logs.get(take.log_name)
        if lt is None:
            print(f"{take.name}: not in the Editor.log, skipped")
            continue
        al = mr.estimate_alignment(take, lt)
        rows.append(summarize(take, lt, al))
        r = rows[-1]
        print(json.dumps(r, indent=None)[:4000], "\n")
    with open(os.path.join(a.out, "alignment.json"), "w") as f:
        json.dump(rows, f, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
