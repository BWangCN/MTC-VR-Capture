"""Generate the 09-01 candidate envs and dump their geometry for review.

Writes JSON only -- no env folders, no FBX, nothing under the capture root. The
point is to look at the layouts before any of this is committed to a capture set.

    python scripts/preview_new_envs.py --out preview.json
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from new_obstacles import make_half_wall, make_step_row  # noqa: E402

WIDTH = 2.25
LENGTH = 4.0
WALL_H = 2.5
START_X = 0.5
APPROACH = 1.8  # start -> obstacle FRONT face, held constant across behaviours
SEED0 = 950000

# Harder to the right in each row: less clearance to step over, narrower gap to
# thread. The gap bands stay above HALF_WALL_MIN_GAP_M (0.55 m).
STEP_BANDS = [(0.20, 0.21), (0.235, 0.245), (0.27, 0.28)]
GAP_BANDS = [(1.05, 1.20), (0.80, 0.95), (0.60, 0.72)]

PLAN = (
    [(f"steprow_{i:02d}", "step_row", None, b) for i, b in enumerate(STEP_BANDS)]
    + [(f"wleft_{i:02d}", "half_wall", "left", b) for i, b in enumerate(GAP_BANDS)]
    + [(f"wright_{i:02d}", "half_wall", "right", b) for i, b in enumerate(GAP_BANDS)]
    + [(f"wslit_{i:02d}", "half_wall", "slit", b) for i, b in enumerate(GAP_BANDS)]
)
# Seed by position in PLAN. `hash(str)` is salted per process, so using it here
# would make the same env_id generate different geometry run to run -- the repo
# treats determinism as a tested guarantee.
SEEDS = {slug: SEED0 + 10 * i for i, (slug, *_rest) in enumerate(PLAN)}


def _corners(cx, cy, sx, sy, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    hx, hy = sx / 2.0, sy / 2.0
    return [
        [cx + dx * c - dy * s, cy + dx * s + dy * c]
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))
    ]


def _footprint(obs):
    """Every drawable box of an obstacle as (corners_xy, z0, z1, label)."""
    g = obs.geometric_params
    out = []
    if "items" in g:
        for it in g["items"]:
            cx, cy, cz = it["position"]
            sx, sy, sz = it["size_xyz"]
            out.append(
                {
                    "poly": _corners(cx, cy, sx, sy, it.get("yaw_deg", 0.0)),
                    "z0": cz - sz / 2.0,
                    "z1": cz + sz / 2.0,
                    "kind": it.get("primitive", "box"),
                    "label": it.get("asset_name") or it["name"],
                }
            )
    for w in g.get("walls", []):
        cx, cy, cz = w["position"]
        sx, sy, sz = w["size_xyz"]
        out.append(
            {
                "poly": _corners(cx, cy, sx, sy, 0.0),
                "z0": cz - sz / 2.0,
                "z1": cz + sz / 2.0,
                "kind": "wall",
                "label": w["side"],
            }
        )
    return out


def build(slug, kind, variant, band):
    """Place the obstacle so its FRONT face sits `APPROACH` from the start."""
    seed = SEEDS[slug]
    front = START_X + APPROACH

    def at(x):
        rng = random.Random(seed)
        if kind == "step_row":
            return make_step_row(rng, x, WIDTH, h_max_range=band)
        return make_half_wall(rng, x, WIDTH, variant, band)

    probe = at(front)
    lo = min(p[0] for f in _footprint(probe) for p in f["poly"])
    obs = at(front + (front - lo))
    fps = _footprint(obs)
    xs = [p[0] for f in fps for p in f["poly"]]
    ys = [p[1] for f in fps for p in f["poly"]]
    g = obs.geometric_params

    rec = {
        "env_id": f"corridor_atomic_{slug}",
        "slug": slug,
        "family": "step_over" if kind == "step_row" else "half_wall",
        "variant": variant or "row",
        "seed": seed,
        "corridor": {"length": LENGTH, "width": WIDTH, "wall_height": WALL_H},
        "start": [START_X, WIDTH / 2.0],
        "goal": [LENGTH - START_X, WIDTH / 2.0],
        "parts": fps,
        "x_span": [round(min(xs), 4), round(max(xs), 4)],
        "y_span": [round(min(ys), 4), round(max(ys), 4)],
        "approach_m": round(min(xs) - START_X, 4),
        "constraints": {k: round(float(v), 4) for k, v in obs.constraint_params.items()},
    }
    if kind == "step_row":
        edges = sorted((min(p[1] for p in f["poly"]), max(p[1] for p in f["poly"])) for f in fps)
        gaps = (
            [edges[0][0]]
            + [edges[i + 1][0] - edges[i][1] for i in range(len(edges) - 1)]
            + [WIDTH - edges[-1][1]]
        )
        rec["n_items"] = len(fps)
        rec["max_gap_mm"] = round(max(max(gaps, default=0.0), 0.0) * 1000, 1)
        rec["outside_wall"] = sum(
            1 for f in fps for p in f["poly"] if p[1] < -1e-9 or p[1] > WIDTH + 1e-9
        )
        rec["height_m"] = round(max(f["z1"] for f in fps), 4)
        rec["assets"] = sorted({f["label"] for f in fps if f["kind"] == "asset"})
    else:
        rec["gap_m"] = round(g["gap_m"], 4)
        rec["depth_x_m"] = round(g["depth_x_m"], 4)
        rec["occupancy"] = round(g["occupancy"], 4)
        rec["walls"] = [
            {
                "side": w["side"],
                "span_y": round(w["span_y"], 4),
                "y0": round(w["center_y"] - w["span_y"] / 2, 4),
                "y1": round(w["center_y"] + w["span_y"] / 2, 4),
            }
            for w in g["walls"]
        ]
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="preview_new_envs.json")
    a = ap.parse_args()
    envs = [build(*p) for p in PLAN]
    Path(a.out).write_text(json.dumps({"envs": envs}, indent=1), encoding="utf-8")
    for e in envs:
        extra = (
            f"n={e['n_items']:>2} 最大间隙 {e['max_gap_mm']:>5.0f}mm 越界 {e['outside_wall']} "
            f"高 {e['height_m']:.3f}"
            if e["family"] == "step_over"
            else f"gap {e['gap_m']:.3f} 厚 {e['depth_x_m']:.3f} 占比 {e['occupancy']:.3f}"
        )
        print(f"  {e['env_id']:<32}{e['variant']:<7}{extra}  approach {e['approach_m']:.3f}")
    print(f"\n{len(envs)} envs -> {a.out}")


if __name__ == "__main__":
    main()
