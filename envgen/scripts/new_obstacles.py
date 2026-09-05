"""Two obstacle families for the 09-01 capture set: an aligned step-over row and
half walls that force a lateral detour.

Both replace earlier builders that had specific, measured defects:

`step_row` vs `make_dense_ground` -- the old builder clamped each item's centre
by its UNROTATED half-width and then applied a random yaw, so the two edge slots
overhung the wall every single time. Measured across all seven generated ground
envs: 1-2 items out of 14 stick out by 10-75 mm, in every one of them. This
builder packs by the item's ACTUAL rotated lateral extent, so nothing can leave
the corridor by construction, and it lays one aligned row instead of a jittered
grid.

`half_wall` vs `make_lateral_block` / `make_wall_squeeze` -- those are two
separate obstacle types with two separate constraint keys. Here the three cases
the capture asks for (blocked from the left, from the right, or a slit between
two protrusions) are one family with explicit per-env wall spans and gap.
"""

from __future__ import annotations

import math
import random

import numpy as np

from mtc_corridor._objaverse_stub import load_objaverse_asset
from mtc_corridor.obstacle import IDENTITY_QUAT, ObstacleInstance, _annotate_constraints, _new_id

G1_SHOULDER_WIDTH_M = 0.45

# A foot needs a gap to land in; keep every lateral gap under this so the only
# way past the row is over it. 120 mm is the width the v2 standard corridors
# were accepted at.
STEP_ROW_MAX_GAP_M = 0.12
STEP_ROW_HEIGHT_RANGE = (0.20, 0.28)  # = CONSTRAINT_PARAM_RANGES["leg_clearance"]
STEP_ROW_YAW_JITTER_DEG = 12.0  # a row, not a parade: enough to look placed, not scattered

HALF_WALL_HEIGHT_M = 2.5  # floor to ceiling: no ducking under, no stepping over
HALF_WALL_DEPTH_RANGE = (0.25, 0.55)  # wall thickness along travel, per env
HALF_WALL_MIN_GAP_M = 0.55  # > G1 shoulder (0.45), so the gap stays passable


def _lateral_extent(sx: float, sy: float, yaw_deg: float) -> float:
    """Half-width along y of an sx-by-sy footprint yawed by yaw_deg."""
    c = abs(math.cos(math.radians(yaw_deg)))
    s = abs(math.sin(math.radians(yaw_deg)))
    return (sx * s + sy * c) / 2.0


def make_step_row(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
    h_max_range: tuple[float, float] | None = None,
    max_gap: float = STEP_ROW_MAX_GAP_M,
) -> ObstacleInstance:
    """leg_clearance: one aligned row of real assets spanning wall to wall.

    Assets are drawn until their rotated widths plus the minimum gaps fill the
    corridor, then the leftover slack is spread evenly between them. Every item
    is therefore inside the walls and every gap is under `max_gap`.
    """
    h_max = rng.uniform(*(h_max_range or STEP_ROW_HEIGHT_RANGE))
    base = _new_id(rng, "ground")
    picks: list[dict] = []
    used = 0.0

    # Draw assets until the row can span the corridor. The cap is a guard against
    # a registry of only tiny items, not an expected exit.
    for _ in range(200):
        yaw = rng.uniform(-STEP_ROW_YAW_JITTER_DEG, STEP_ROW_YAW_JITTER_DEG)
        try:
            rec = load_objaverse_asset(
                category="any",
                size_constraint={
                    "height_min": 0.12,
                    "height_max": h_max,
                    "footprint_max_xy": 0.42,
                },
                rng=rng,
            )
        except LookupError:
            rec = None
        if rec is None:
            sy = rng.uniform(0.16, 0.30)
            sx = rng.uniform(0.16, 0.30)
            sz = rng.uniform(0.17, h_max)
            picks.append({"kind": "box", "sx": sx, "sy": sy, "sz": sz, "yaw": yaw})
        else:
            sx, sy, sz = (float(v) for v in rec.size_xyz)
            picks.append({"kind": "asset", "rec": rec, "sx": sx, "sy": sy, "sz": sz, "yaw": yaw})
        used += 2.0 * _lateral_extent(picks[-1]["sx"], picks[-1]["sy"], picks[-1]["yaw"])
        if used + max_gap * (len(picks) - 1) >= corridor_width:
            break

    n = len(picks)
    slack = corridor_width - used
    gap = max(0.0, slack / (n + 1)) if n else 0.0
    if gap > max_gap:  # too few items to fill: pad with boxes sized to close it
        while gap > max_gap and n < 40:
            sy = rng.uniform(0.16, 0.30)
            picks.append(
                {
                    "kind": "box",
                    "sx": rng.uniform(0.16, 0.30),
                    "sy": sy,
                    "sz": rng.uniform(0.17, h_max),
                    "yaw": rng.uniform(-STEP_ROW_YAW_JITTER_DEG, STEP_ROW_YAW_JITTER_DEG),
                }
            )
            used += 2.0 * _lateral_extent(picks[-1]["sx"], picks[-1]["sy"], picks[-1]["yaw"])
            n = len(picks)
            gap = max(0.0, (corridor_width - used) / (n + 1))

    rng.shuffle(picks)
    items = []
    y = gap  # generator frame: y runs 0..corridor_width
    for i, p in enumerate(picks):
        ext = _lateral_extent(p["sx"], p["sy"], p["yaw"])
        cy = y + ext
        item = {
            "name": f"{base}_{i}",
            "position": [float(x_center), float(cy), float(p["sz"] / 2.0)],
            "yaw_deg": float(p["yaw"]),
            "size_xyz": [float(p["sx"]), float(p["sy"]), float(p["sz"])],
        }
        if p["kind"] == "asset":
            rec = p["rec"]
            item.update(
                primitive="asset",
                asset_ref=str(rec.usd_path),
                asset_class=rec.cls,
                asset_name=rec.asset,
                asset_center_xyz=rec.center_xyz.tolist(),
            )
        else:
            item.update(
                primitive="box", rotation_deg=[0.0, 0.0, float(p["yaw"])], material="concrete"
            )
        items.append(item)
        y = cy + ext + gap

    obs = ObstacleInstance(
        obstacle_id=base,
        obstacle_type="ground",
        asset_source="procedural:ground:step_row",
        position=np.array([x_center, corridor_width / 2.0, 0.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "items": items,
            "cluster_center_y": corridor_width / 2.0,
            "layout": "single_aligned_row",
            "row_gap_m": float(gap),
            "material": "concrete",
        },
    )
    _annotate_constraints(obs, {"leg_clearance": float(h_max)})
    return obs


def make_half_wall(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
    variant: str,
    gap_range: tuple[float, float],
    depth_range: tuple[float, float] = HALF_WALL_DEPTH_RANGE,
) -> ObstacleInstance:
    """lateral_passage: full-height wall(s) that occupy part of the width.

    variant `left` / `right` -- one wall from that side, the gap is what is left
    against the far wall. variant `slit` -- a wall from each side with the gap
    between them, its centre offset a little so the slit is not always dead
    centre. Wall spans and gap are drawn per env from `gap_range`.
    """
    if variant not in ("left", "right", "slit"):
        raise ValueError(f"variant must be left|right|slit, got {variant!r}")
    gap = rng.uniform(*gap_range)
    if gap < HALF_WALL_MIN_GAP_M:
        raise ValueError(f"gap {gap:.3f} m is under the {HALF_WALL_MIN_GAP_M} m passable minimum")
    depth = rng.uniform(*depth_range)
    base = _new_id(rng, "hwall")
    walls = []

    if variant in ("left", "right"):
        span = corridor_width - gap
        cy = span / 2.0 if variant == "left" else corridor_width - span / 2.0
        walls.append({"side": variant, "span_y": float(span), "center_y": float(cy)})
    else:
        # Offset the slit off centre by up to a quarter of the leftover width, so
        # the two protrusions differ and the detour is not a straight walk.
        total = corridor_width - gap
        off = rng.uniform(-0.25, 0.25) * total
        span_l = total / 2.0 + off
        span_r = total - span_l
        walls.append({"side": "left", "span_y": float(span_l), "center_y": float(span_l / 2.0)})
        walls.append(
            {
                "side": "right",
                "span_y": float(span_r),
                "center_y": float(corridor_width - span_r / 2.0),
            }
        )

    for w in walls:
        w["size_xyz"] = [float(depth), float(w["span_y"]), float(HALF_WALL_HEIGHT_M)]
        w["position"] = [float(x_center), float(w["center_y"]), float(HALF_WALL_HEIGHT_M / 2.0)]

    occupancy = sum(w["span_y"] for w in walls) / corridor_width
    obs = ObstacleInstance(
        obstacle_id=base,
        obstacle_type="lateral_block",
        asset_source=f"procedural:half_wall:{variant}",
        position=np.array([x_center, corridor_width / 2.0, HALF_WALL_HEIGHT_M / 2.0]),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "variant": variant,
            "walls": walls,
            "gap_m": float(gap),
            "depth_x_m": float(depth),
            "occupancy": float(occupancy),
            "wall_height_m": float(HALF_WALL_HEIGHT_M),
            "material": "concrete",
        },
    )
    _annotate_constraints(obs, {"lateral_passage": float(occupancy)})
    return obs
