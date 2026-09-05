"""Build the curated STANDARD corridors for the multi-env / multi-motion
distillation sanity check.

Two layout versions live here, selected with `--version`:

  v1 (FROZEN)  L=6.8 m, 4 behaviour slots on a centred grid. This is the exact
               geometry the 30 published clips in `standard_corridors/` were
               recorded on. Do not change it -- the dataset pairs every motion
               with the FBX it was captured against.
  v2           L=8.0 m, 3 obstacles, uniform 2.08 m spacing, 0.75 m run-out.
  atomic       L=4.0 m, ONE obstacle, 1.8 m approach. 5 behaviours x 3 pinned
               difficulty levels + an obstacle-free baseline = 16 envs. Built
               for on-headset rapid capture: one take == one primitive == one
               clip, so the trimming step disappears and pelvis drift only
               integrates over ~4 m instead of 8. Its dog_hole is a wall-to-wall
               lintel with the whole bottom band clear, not a narrow hole.

Why v2 exists: v1's obstacle spacing (1.10-1.20 m, and 0.55 m from start to the
first beam on std_01) is smaller than the pelvis drift in PICO body tracking.
The pilot could only start ducking once effectively under the beam, so the drift
turned into a collision once the clip was retargeted onto G1. v2 spends the
whole L<=8 m budget on approach distance instead: lead-in == inter-obstacle
spacing, and the empty run to the goal is cut to the shortest value that still
leaves the 0.5 m goal clearance `placement.violates_start_goal_clearance`
enforces (measured to the obstacle SURFACE, not its center).

Both versions share the single-behaviour obstacle design: every obstacle blocks
the FULL corridor width, so it forces exactly one behaviour and there is no
"go around vs. step over" ambiguity.
    beam(duck, forced I-beam)   dog_hole(crawl through wall opening)
    dense_ground(step over)     lateral_block(side-step)
    wall_squeeze(shoulder-turn)

Outputs to <MTC_CAPTURE_ROOT>\\<env_id>\\ (mesh + yaml + gen_meta).
"""

import argparse
import json
import os
import random
import sys
from datetime import datetime
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from mtc_corridor import export_to_yaml  # noqa: E402
from mtc_corridor.constraint_rules import CONSTRAINT_PARAM_RANGES  # noqa: E402
from mtc_corridor.corridor import WALL_HEIGHT_M, CorridorEnv  # noqa: E402
from mtc_corridor.difficulty import compute_env_difficulty  # noqa: E402
from mtc_corridor.obstacle import (  # noqa: E402
    DOG_HOLE_WALL_THICKNESS_X,
    IDENTITY_QUAT,
    ObstacleInstance,
    _annotate_constraints,
    _new_id,
    load_objaverse_asset,
    make_beam,
    make_lateral_block,
    make_pillars,
)
from mtc_corridor.placement import violates_start_goal_clearance  # noqa: E402
from scripts.export_to_unity import build_one  # noqa: E402

W = 2.25
H = WALL_HEIGHT_M
START_X = 0.5
GOAL_INSET = 0.5  # goal sits at L - GOAL_INSET
# Capture root. Point MTC_CAPTURE_ROOT at your data drive; the repo-local
# default keeps a fresh clone working with no configuration.
ROOT = Path(os.environ.get("MTC_CAPTURE_ROOT") or Path(__file__).resolve().parents[2] / "captures")


def make_ibeam_beam(rng, x_center, corridor_width, height_range=None, tries=60, strict=False):
    """beam forced to the chunky wall_ibeam_full variant (not thin rebar/pipe).

    `height_range` additionally pins the clearance under the beam. The band in
    CONSTRAINT_PARAM_RANGES reaches down to 0.85 m, which is a squat-walk rather
    than a duck; a curated env should not get one by luck.

    `strict` raises instead of returning an out-of-band beam. The atomic set
    uses narrow per-level bands where a silent miss would put the wrong
    difficulty level under the env id, so it opts in (and raises `tries`).
    """
    obs = None
    for _ in range(tries):
        r = random.Random(rng.randint(0, 10**9))
        obs = make_beam(r, x_center, corridor_width)
        if obs.geometric_params.get("variant") != "wall_ibeam_full":
            continue
        if height_range is None:
            return obs
        h = obs.constraint_params.get("height_constraint")
        if h is not None and height_range[0] <= h <= height_range[1]:
            return obs
    if strict:
        raise ValueError(f"no wall_ibeam_full beam in {height_range} after {tries} tries")
    return obs


def pinned(build, rng, key, band, tries=2000):
    """Retry `build(rng)` until its `key` constraint param lands inside `band`.

    The shared builders in `obstacle.py` sample their constraint uniformly over
    the whole CONSTRAINT_PARAM_RANGES band, and pinning a curated env to a
    specific difficulty LEVEL is the only thing the atomic set needs from them --
    so rejection-sample here instead of adding parameters to the shared module.

    `band=None` calls `build(rng)` once with the parent rng, leaving the stream
    byte-identical for v1/v2.
    """
    if band is None:
        return build(rng)
    for _ in range(tries):
        r = random.Random(rng.randint(0, 10**9))
        obs = build(r)
        v = (obs.constraint_params or {}).get(key)
        if v is not None and band[0] <= v <= band[1]:
            return obs
    raise ValueError(f"no obstacle with {key} in {band} after {tries} tries")


def make_double_beam(rng, x_center, corridor_width, gap=1.5):
    """Two consecutive I-beams `gap` m apart along travel -- a double duck. Used
    in place of a dog_hole where a crawl isn't wanted. Returns TWO obstacles."""
    return [
        make_ibeam_beam(rng, x_center - gap / 2.0, corridor_width),
        make_ibeam_beam(rng, x_center + gap / 2.0, corridor_width),
    ]


def make_clean_dog_hole(
    rng, x_center, corridor_width, wall_height, opening_range=None, full_width=False
):
    """dog_hole as intended: one wall sealing the WHOLE corridor cross-section
    (full width x full height) with only a small rectangular crawl hole at the
    bottom-centre. The small, centred opening keeps the wall reading as solid
    with a dog door -- no wide gap, and no floating lintel.

    `full_width` opens the ENTIRE bottom band instead: the wall becomes a lintel
    hung from the ceiling, spanning wall to wall, with clear floor underneath.
    The side walls already remove the go-around, so a narrow hole adds a lateral
    aiming task on top of the crawl without making the crawl any harder -- and a
    hung panel between two walls is far simpler to build in wood. Both the AABB
    builder and the FBX exporter drop the side stubs on their own once the
    opening spans the full width, so this needs no changes downstream.
    """
    lo, hi = opening_range or CONSTRAINT_PARAM_RANGES["full_body_lowering"]
    opening_top = rng.uniform(lo, hi)  # 0.60-0.70 m tall hole
    obs = ObstacleInstance(
        obstacle_id=_new_id(rng, "doghole"),
        obstacle_type="dog_hole",
        asset_source="procedural:dog_hole:" + ("full_width" if full_width else "clean"),
        position=np.array([x_center, corridor_width / 2.0, wall_height / 2.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "wall_thickness_x": DOG_HOLE_WALL_THICKNESS_X,
            "opening_top_z": float(opening_top),
            "opening_width_y": float(corridor_width) if full_width else 0.75,
            "wall_height_z": float(wall_height),
            "corridor_width": float(corridor_width),
            "material": "concrete",
        },
    )
    _annotate_constraints(obs, {"full_body_lowering": float(opening_top)})
    return obs


def make_dense_ground(
    rng, x_center, corridor_width, nrows=None, cell=0.36, hug_walls=False, h_max_range=None
):
    """leg_clearance: scattered objects across the FULL width so the only option
    is to step over -- no go-around. Not packed solid: a jittered grid (column
    spacing < G1 shoulder width) keeps every lateral gap too narrow for G1 to
    slip through, while leaving small gaps and a natural look. Each slot is a
    real asset when available (good voxel geometry), a box otherwise. `nrows`
    (depth rows) and `cell` (column spacing) control density.

    `hug_walls` clamps each item by its own half-width instead of a fixed 0.22 m
    inset. The fixed inset leaves a ~165 mm gap against each wall that no amount
    of extra density can close, which is wide enough to walk a foot through --
    it defeats the whole point of the obstacle. Off by default so v1 geometry
    stays byte-identical.
    """
    h_max = rng.uniform(*(h_max_range or (0.20, 0.28)))  # low end of the leg_clearance
    #                                  band: a real step-over but easy to clear (was
    #                                  0.36-0.45, which read as brutally tall to step over)
    cw = corridor_width  # span exactly wall-to-wall (edge slots cover the walls)
    cy = corridor_width / 2.0
    base = _new_id(rng, "ground")
    h_min = 0.12  # no flat items, but low enough that the foot clears easily
    if nrows is None:
        nrows = rng.choice([1, 2])
    depth = rng.uniform(0.45, 0.65) + 0.15 * max(0, nrows - 2)  # deeper for more rows
    ncols = max(3, round(cw / cell))  # cell < G1_SHOULDER_WIDTH (0.45)
    colw = cw / ncols
    roww = depth / nrows
    jit = colw * 0.12
    items = []
    idx = 0
    for c in range(ncols):
        col_y = -cw / 2.0 + (c + 0.5) * colw
        for r in range(nrows):
            row_x = (-depth / 2.0 + (r + 0.5) * roww) if nrows > 1 else 0.0
            stagger = (colw * 0.5) if (r % 2 and nrows > 1) else 0.0
            px = x_center + row_x + rng.uniform(-jit, jit)
            # Draw the jitter here regardless, so the RNG stream is unchanged;
            # the wall clamp is applied once the item's width is known.
            py_raw = cy + col_y + stagger + rng.uniform(-jit, jit)

            def _clamp_y(width_y, _p=py_raw):
                if hug_walls:  # item edge to the wall, so no fixed dead zone
                    half = width_y / 2.0
                    return min(cw - half, max(half, _p))
                return min(cw - 0.22, max(0.22, _p))

            rec = None
            try:
                rec = load_objaverse_asset(
                    category="any",
                    size_constraint={
                        "height_min": h_min,
                        "height_max": h_max,
                        "footprint_max_xy": 0.42,
                    },
                    rng=rng,
                )
            except LookupError:
                rec = None
            if rec is not None:
                sx, sy, sz = rec.size_xyz
                py = _clamp_y(sy)
                item = {
                    "primitive": "asset",
                    "name": f"{base}_{idx}",
                    "asset_ref": str(rec.usd_path),
                    "asset_class": rec.cls,
                    "asset_name": rec.asset,
                    "size_xyz": [float(sx), float(sy), float(sz)],
                    "asset_center_xyz": rec.center_xyz.tolist(),
                    "position": [float(px), float(py), float(sz / 2.0)],
                    "yaw_deg": float(rng.uniform(0.0, 360.0)),
                }
            else:
                sz = rng.uniform(h_min + 0.05, h_max)  # fallback fills the cell
                py = _clamp_y(roww * 1.25)
                item = {
                    "primitive": "box",
                    "name": f"{base}_{idx}",
                    "size_xyz": [colw * 1.25, roww * 1.25, sz],
                    "position": [float(px), float(py), float(sz / 2.0)],
                    "rotation_deg": [0.0, 0.0, 0.0],
                    "material": "concrete",
                }
            items.append(item)
            idx += 1
    obs = ObstacleInstance(
        obstacle_id=base,
        obstacle_type="ground",
        asset_source="procedural:ground:dense_block",
        position=np.array([x_center, cy, 0.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={"items": items, "cluster_center_y": cy, "material": "concrete"},
    )
    _annotate_constraints(obs, {"leg_clearance": float(h_max)})
    return obs


# The 09-01 families live in their own module: the older ground/lateral builders
# are load-bearing for v1/v2 byte-identity, so they are extended by addition
# rather than edited. See new_obstacles.py for what each one fixes.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from new_obstacles import make_half_wall, make_step_row  # noqa: E402


def builders(spec, env_id=None):
    bh = spec.get("beam_height")
    # Per-env ground density: only the envs listed get a tighter row, so tuning
    # one curated env never silently re-rolls the others.
    g = spec.get("ground", {}).get(env_id)
    # Per-env constraint pin {constraint_key: (lo, hi)}. The atomic set uses it
    # to put a named difficulty LEVEL behind each env id; v1/v2 leave it empty,
    # which keeps every rng draw below byte-identical to what they shipped.
    pin = spec.get("params", {}).get(env_id, {})
    # Half walls need a shape (which side) and a gap band, not a constraint pin.
    hw = spec.get("half_wall", {}).get(env_id, {})
    beam_band = pin.get("height_constraint", bh)
    return {
        "beam": lambda rng, x: make_ibeam_beam(
            rng, x, W, beam_band, **({"tries": 4000, "strict": True} if pin else {})
        ),
        "dog_hole": lambda rng, x: make_clean_dog_hole(
            rng, x, W, H, pin.get("full_body_lowering"), spec.get("dog_hole_full_width", False)
        ),
        "dense_ground": (
            (lambda rng, x: make_dense_ground(rng, x, W, h_max_range=pin.get("leg_clearance")))
            if g is None
            else (
                lambda rng, x: make_dense_ground(
                    rng, x, W, h_max_range=pin.get("leg_clearance"), **g
                )
            )
        ),
        "lateral_block": lambda rng, x: pinned(
            lambda r: make_lateral_block(r, x, W),
            rng,
            "lateral_passage",
            pin.get("lateral_passage"),
        ),
        "wall_squeeze": lambda rng, x: pinned(
            lambda r: make_pillars(r, x, W, H),
            rng,
            "lateral_clearance",
            pin.get("lateral_clearance"),
        ),
        "step_row": lambda rng, x: make_step_row(rng, x, W, h_max_range=pin.get("leg_clearance")),
        "half_wall": lambda rng, x: make_half_wall(
            rng, x, W, hw.get("variant", "left"), hw.get("gap", (0.80, 0.95))
        ),
        "double_beam": lambda rng, x: make_double_beam(rng, x, W),  # -> two obstacles
        "dense_ground2": lambda rng, x: make_dense_ground(
            rng, x, W, nrows=1, cell=0.20
        ),  # 1 row, denser
    }


# v1: std_01 has no dog_hole -- its crawl is replaced by two consecutive beams.
# v2: 3 obstacles each, so the double beam is dropped and the 9 slots are spread
# 2/2/2/2/1 over the five behaviours (wall_squeeze is the single, being the type
# whose margin depends on LATERAL drift, which wider spacing does not help).
SPECS = {
    "v1": {
        "length": 6.8,
        "run_out": None,  # centred-grid layout, run-out not a free parameter
        "plans": {
            "corridor_std_01": (900001, ["double_beam", "dense_ground", "lateral_block"]),
            "corridor_std_02": (900002, ["wall_squeeze", "beam", "dog_hole", "dense_ground2"]),
            "corridor_std_03": (900003, ["lateral_block", "wall_squeeze", "beam", "dog_hole"]),
        },
    },
    "v2": {
        "length": 8.0,
        "run_out": 0.75,
        # the clearance band the pilot actually walked in v1 (1.17-1.34 m)
        "beam_height": (1.15, 1.35),
        # std_01's step-over was a single row of 6 objects with ~290 mm lateral
        # gaps -- wide enough to walk a foot straight through, so it never forced
        # a step. Tighten that ONE row. std_03's ground is already 2 staggered
        # rows and is deliberately left alone.
        "ground": {
            "corridor_std_01_v2": {"nrows": 1, "cell": 0.16, "hug_walls": True},
        },
        "plans": {
            "corridor_std_01_v2": (900011, ["beam", "dense_ground", "lateral_block"]),
            "corridor_std_02_v2": (900012, ["wall_squeeze", "beam", "dog_hole"]),
            "corridor_std_03_v2": (900013, ["lateral_block", "dense_ground", "dog_hole"]),
        },
    },
}


# ---------------------------------------------------------------------------
# atomic: ONE obstacle per env, short corridor, three difficulty levels each.
# ---------------------------------------------------------------------------
# Why: a full corridor take couples long-horizon planning with pelvis drift --
# the drift integrates over the whole 8 m walk, and a bad clip loses all four
# behaviours at once. One obstacle per take makes every clip a single primitive,
# caps the drift at ~4 m of walking, and removes the trimming step entirely
# (one take == one clip).
ATOMIC_LENGTH = 4.0
ATOMIC_APPROACH = 1.8  # start (x=0.5) -> obstacle FRONT face; the rest is run-out
ATOMIC_SEED0 = 910000
# Same tightened row std_01_v2 uses: 14 items, max lateral gap 80 mm. A shoe is
# ~100 mm, so this is what actually forces a step-over instead of a walk-through.
ATOMIC_GROUND = {"nrows": 1, "cell": 0.16, "hug_walls": True}

# slug, obstacle type, constraint key, three bands ordered EASY -> HARD.
# Bands are narrow on purpose: the env id names the level, so the geometry must
# actually be at that level rather than anywhere in CONSTRAINT_PARAM_RANGES.
ATOMIC_LEVELS = [
    ("beam", "beam", "height_constraint", [(1.34, 1.42), (1.22, 1.30), (1.10, 1.18)]),
    ("doghole", "dog_hole", "full_body_lowering", [(0.68, 0.70), (0.64, 0.66), (0.60, 0.62)]),
    ("ground", "dense_ground", "leg_clearance", [(0.20, 0.21), (0.235, 0.245), (0.27, 0.28)]),
    # lateral_passage is an OCCUPANCY FRACTION -- bigger p = narrower gap = harder.
    ("lblock", "lateral_block", "lateral_passage", [(0.33, 0.37), (0.43, 0.47), (0.53, 0.57)]),
    # lateral_clearance IS an absolute gap -- smaller = harder.
    ("squeeze", "wall_squeeze", "lateral_clearance", [(0.68, 0.70), (0.59, 0.61), (0.50, 0.52)]),
]


def _atomic_spec():
    plans, params, ground = {}, {}, {}
    i = 0
    for slug, otype, key, bands in ATOMIC_LEVELS:
        for level, band in enumerate(bands):
            env_id = f"corridor_atomic_{slug}_{level:02d}"
            i += 1
            plans[env_id] = (ATOMIC_SEED0 + i, [otype])
            params[env_id] = {key: band}
            if otype == "dense_ground":
                ground[env_id] = dict(ATOMIC_GROUND)
    # Obstacle-free baseline: plain walking over the same distance, needed both
    # as a control clip and as the "no constraint active" primitive.
    plans["corridor_atomic_free_00"] = (ATOMIC_SEED0, [])
    return {
        "length": ATOMIC_LENGTH,
        "run_out": None,  # falls out of length - approach - obstacle depth
        "approach": ATOMIC_APPROACH,
        # Lintel hung wall-to-wall, whole bottom band clear -- see
        # make_clean_dog_hole. v1/v2 keep the narrow centred hole.
        "dog_hole_full_width": True,
        "ground": ground,
        "params": params,
        "plans": plans,
    }


SPECS["atomic"] = _atomic_spec()


# ---------------------------------------------------------------------------
# atomic2: the rungs the first set left empty, plus the first compounds.
# ---------------------------------------------------------------------------
# Two things the 16-env set does not cover. First, "how low" is a continuous
# parameter that makes the GAIT jump discretely -- walk, duck, squat, hands-and-
# knees, forearms, prone -- and the first set only sampled three of those six
# rungs, missing the whole squat-to-prone span. Second, every obstacle there
# constrains exactly one thing; a real corridor makes you duck and pick your
# footing at the same time.
#
# NOT in this batch, and not by oversight:
#   * corners -- CorridorEnv is length x width x wall_height, a rectangle. A turn
#     needs a different environment type, not a different obstacle.
#   * sloped ceilings -- expressible as a rotated box, but the AABB of a tilted
#     slab is a hugely conservative brick, so MuJoCo collision would be wrong.
#     Needs a real wedge primitive first.
ATOMIC2_SEED0 = 920000

# slug, types placed at the SAME x, per-level constraint pins
ATOMIC2 = [
    # -- posture ladder: squat sits between ducking and crawling ---------------
    ("squat", ["beam"], [{"height_constraint": (0.98, 1.04)}, {"height_constraint": (0.86, 0.92)}]),
    # -- below the hands-and-knees band the gait changes again ----------------
    # Floored at G1's ~0.50 m minimum crawl height (constraint_rules), so both
    # rungs stay passable. A prone/commando rung was designed and then dropped:
    # G1 has no prone crawl to learn -- shoulder and hip range rule it out, and
    # it could not stand back up -- so the motion would have nothing to train.
    (
        "lowcrawl",
        ["dog_hole"],
        [{"full_body_lowering": (0.55, 0.59)}, {"full_body_lowering": (0.51, 0.55)}],
    ),
    # -- compounds: two constraints at one place ------------------------------
    (
        "duckstep",
        ["beam", "dense_ground"],
        [
            {"height_constraint": (1.30, 1.38), "leg_clearance": (0.20, 0.21)},
            {"height_constraint": (1.18, 1.26), "leg_clearance": (0.235, 0.245)},
            {"height_constraint": (1.06, 1.14), "leg_clearance": (0.27, 0.28)},
        ],
    ),
    # crawling with the hands landing among debris -- the only env where the
    # HANDS have a placement constraint, which nothing else in the set tests
    (
        "crawlstep",
        ["dog_hole", "dense_ground"],
        [
            {"full_body_lowering": (0.66, 0.70), "leg_clearance": (0.20, 0.21)},
            {"full_body_lowering": (0.58, 0.62), "leg_clearance": (0.235, 0.245)},
        ],
    ),
]


def _atomic2_spec():
    plans, params, ground = {}, {}, {}
    i = 0
    for slug, types, levels in ATOMIC2:
        for level, pin in enumerate(levels):
            env_id = f"corridor_atomic_{slug}_{level:02d}"
            i += 1
            plans[env_id] = (ATOMIC2_SEED0 + i, list(types))
            params[env_id] = dict(pin)
            if "dense_ground" in types:
                ground[env_id] = dict(ATOMIC_GROUND)
    return {
        "length": ATOMIC_LENGTH,
        "run_out": None,
        "approach": ATOMIC_APPROACH,
        "dog_hole_full_width": True,
        "ground": ground,
        "params": params,
        "plans": plans,
    }


SPECS["atomic2"] = _atomic2_spec()


# ---------------------------------------------------------------------------
# atomic3: the 09-01 set -- step over a row, or go around a half wall.
# ---------------------------------------------------------------------------
# Both families exist because their predecessors had measured defects. The
# ground rows put 1-2 of their 14 items through the wall in every env that was
# ever generated (the centre was clamped by the unrotated half-width, then a
# random yaw was applied). And "blocked from one side" vs "a slit between two
# protrusions" were two obstacle types with two constraint keys, so a session
# could not walk the same behaviour across all three shapes.
ATOMIC3_SEED0 = 950000

# slug, obstacle type, per-level pin, per-level half-wall shape
ATOMIC3 = [
    (
        "steprow",
        "step_row",
        [{"leg_clearance": b} for b in ((0.20, 0.21), (0.235, 0.245), (0.27, 0.28))],
        None,
    ),
    ("wleft", "half_wall", None, "left"),
    ("wright", "half_wall", None, "right"),
    ("wslit", "half_wall", None, "slit"),
]
# Gap left to walk through, narrowing by level. Floor is HALF_WALL_MIN_GAP_M
# (0.55 m), which clears G1's 0.45 m shoulder -- these force a detour, not a
# squeeze; the squeeze rungs are the wall_squeeze envs.
ATOMIC3_GAPS = [(1.05, 1.20), (0.80, 0.95), (0.60, 0.72)]


def _atomic3_spec():
    plans, params, half_wall = {}, {}, {}
    i = 0
    for slug, otype, pins, variant in ATOMIC3:
        levels = pins if pins is not None else ATOMIC3_GAPS
        for level, item in enumerate(levels):
            env_id = f"corridor_atomic_{slug}_{level:02d}"
            i += 1
            plans[env_id] = (ATOMIC3_SEED0 + 10 * i, [otype])
            if pins is not None:
                params[env_id] = dict(item)
            else:
                half_wall[env_id] = {"variant": variant, "gap": item}
    return {
        "length": ATOMIC_LENGTH,
        "run_out": None,
        "approach": ATOMIC_APPROACH,
        "dog_hole_full_width": True,
        "ground": {},
        "params": params,
        "half_wall": half_wall,
        "plans": plans,
    }


SPECS["atomic3"] = _atomic3_spec()


# Two crawl postures, separated by what the opening physically permits. The
# bands come from the 08-24..08-28 doghole/lowcrawl captures (111k frames,
# G1-scaled): on hands and knees the body tops out at 0.52 m (p50) / 0.79 (p90);
# flat on the ground it is 0.35 (p50) / 0.45 (p90). So an opening at or below
# 0.45 leaves no room for hands-and-knees and forces the prone crawl, while
# 0.66-0.78 clears the hands-and-knees p50 with enough headroom for the arms to
# stay straight. Existing doghole (0.60/0.66/0.69) and lowcrawl (0.55/0.57) sit
# inside the high-crawl band, which is why those takes are 15% hands-and-knees.
ATOMIC4_SEED0 = 960000

ATOMIC4 = [
    ("highcrawl", [(0.77, 0.79), (0.71, 0.73), (0.65, 0.67)]),
    ("prone", [(0.44, 0.46), (0.38, 0.40), (0.32, 0.34)]),
]


def _atomic4_spec():
    plans, params = {}, {}
    i = 0
    for slug, bands in ATOMIC4:
        for level, band in enumerate(bands):
            env_id = f"corridor_atomic_{slug}_{level:02d}"
            i += 1
            plans[env_id] = (ATOMIC4_SEED0 + 10 * i, ["dog_hole"])
            params[env_id] = {"full_body_lowering": band}
    return {
        "length": ATOMIC_LENGTH,
        "run_out": None,
        "approach": ATOMIC_APPROACH,
        "dog_hole_full_width": True,
        "ground": {},
        "params": params,
        "plans": plans,
    }


SPECS["atomic4"] = _atomic4_spec()


def layout_x(version, length, n, run_out):
    """Obstacle x centers. v2 makes the lead-in equal to the inter-obstacle
    spacing, so every obstacle gets the same approach distance:
        s = (L - START_X - GOAL_INSET - run_out) / n
    """
    if version == "v1":
        return [1.0 + (length - 2.0) * (i + 0.5) / n for i in range(n)]
    s = (length - START_X - GOAL_INSET - run_out) / n
    return [START_X + s * (i + 1) for i in range(n)]


def atomic_obstacles(env_id, seed, types, spec):
    """The single atomic obstacle, placed by its FRONT face rather than its center.

    Obstacle depth along travel runs 0.18 m (beam) to 0.47 m (dense_ground), so
    a fixed center x would hand each behaviour a different approach distance --
    exactly the quantity the atomic set exists to hold constant. Build once to
    measure the depth, then rebuild from the same seed (identical geometry, only
    translated) at the center that puts the front face `approach` from the start.
    """
    if not types:
        return []  # corridor_atomic_free_00: plain walking baseline
    front = START_X + spec["approach"]

    def build_at(x):
        """All requested types at the same x, from one deterministic stream."""
        rng = random.Random(seed)
        b = builders(spec, env_id)
        out = []
        for t in types:
            r = b[t](rng, x)
            out.extend(r if isinstance(r, list) else [r])
        return out

    probe = build_at(front)
    # Pin the front face of the WHOLE group, so a compound env gets the same
    # approach distance as a single-obstacle one.
    lo = min(float(o.x_extent()[0]) for o in probe)
    # float(): x_extent() comes back as np.float64, and some builders stash the
    # center straight into geometric_params, which the YAML dumper cannot encode.
    return build_at(float(front + (front - lo)))


def build_env(env_id, seed, types, spec, version):
    rng = random.Random(seed)
    length = spec["length"]
    build = builders(spec, env_id)
    obs_list = []
    if version.startswith("atomic"):
        obs_list = atomic_obstacles(env_id, seed, types, spec)
    else:
        xs = layout_x(version, length, len(types), spec["run_out"])
        for t, x in zip(types, xs, strict=True):
            r = build[t](rng, x)
            obs_list.extend(r if isinstance(r, list) else [r])
    if version != "v1":
        # layout_x spaces behaviour SLOTS; a builder that expands to several
        # physical obstacles (double_beam) would silently break the spacing.
        if len(obs_list) != len(types):
            raise ValueError(f"{env_id}: {len(types)} slots produced {len(obs_list)} obstacles")
        # These envs bypass place_in_slot, so its start/goal rule is unchecked
        # unless we assert it -- v1 shipped with a beam 0.55 m from the start.
        for o in obs_list:
            if violates_start_goal_clearance(o, length):
                lo, hi = o.x_extent()
                raise ValueError(
                    f"{env_id}: {o.obstacle_type} x-extent [{lo:.2f},{hi:.2f}] "
                    f"intrudes on the 0.5 m start/goal clearance (L={length})"
                )
    obs_list.sort(key=lambda o: float(o.position[0]))
    d_list = [o.difficulty for o in obs_list]
    counts = {}
    for o in obs_list:
        for c in o.constraints:
            counts[c] = counts.get(c, 0) + 1
    return CorridorEnv(
        env_id=env_id,
        seed=seed,
        tier="medium",
        length=length,
        width=W,
        wall_height=H,
        start_pos=np.array([START_X, W / 2.0, 0.0], dtype=np.float64),
        goal_pos=np.array([length - GOAL_INSET, W / 2.0, 0.0], dtype=np.float64),
        obstacles=obs_list,
        env_difficulty=float(compute_env_difficulty(d_list)),
        constraint_counts=counts,
    )


def gaps(env):
    """(center-to-center, surface-to-surface) gaps along travel, start -> goal."""
    xs = [float(o.position[0]) for o in env.obstacles]
    ext = [o.x_extent() for o in env.obstacles]
    sx, gx = float(env.start_pos[0]), float(env.goal_pos[0])
    if not xs:  # corridor_atomic_free_00 -- the whole corridor is one gap
        return [gx - sx], [gx - sx]
    centers = [xs[0] - sx] + [b - a for a, b in zip(xs, xs[1:], strict=False)] + [gx - xs[-1]]
    surfaces = (
        [ext[0][0] - sx]
        + [b[0] - a[1] for a, b in zip(ext, ext[1:], strict=False)]
        + [gx - ext[-1][1]]
    )
    return centers, surfaces


def export(env, types, version, spec, build_fbx=True):
    d = ROOT / env.env_id
    for sub in ("mesh", "raw", "trimmed"):
        (d / sub).mkdir(parents=True, exist_ok=True)
    export_to_yaml(env, d / f"{env.env_id}.yaml")
    centers, surfaces = gaps(env)
    extra = {}
    if version.startswith("atomic"):
        # The Unity session controller and the ingest step both key off these:
        # `behavior` picks the on-headset ordering, `level` names the difficulty
        # rung, and run_out_m is what is actually left after the obstacle depth.
        slug = env.env_id.rsplit("_", 2)
        extra = {
            "kind": "atomic_primitive",
            "behavior": types[0] if types else "free",
            "level": int(slug[-1]),
            "slug": slug[-2],
            "approach_m": round(spec["approach"], 3),
            "run_out_m": round(surfaces[-1], 3),
        }
    (d / "gen_meta.json").write_text(
        json.dumps(
            {
                "env_id": env.env_id,
                "seed": env.seed,
                "tier": "medium",
                "kind": "standard_curated",
                "layout_version": version,
                "length": env.length,
                "width": W,
                "run_out_m": spec["run_out"],
                "gap_centers_m": [round(g, 3) for g in centers],
                "gap_surfaces_m": [round(g, 3) for g in surfaces],
                "difficulty": round(env.env_difficulty, 4),
                "constraint_params": [
                    {
                        "obstacle_type": o.obstacle_type,
                        **{k: round(v, 4) for k, v in (o.constraint_params or {}).items()},
                    }
                    for o in env.obstacles
                ],
                "obstacle_types": sorted({o.obstacle_type for o in env.obstacles}),
                "behavior_plan": types,
                "note": "curated single-behavior obstacles for distillation sanity check",
                "generated_at": datetime.now().isoformat(timespec="seconds"),
                **extra,
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    if build_fbx:
        build_one(env, d / "mesh")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--version", choices=sorted(SPECS), default="v2")
    ap.add_argument("--no-fbx", action="store_true", help="yaml + meta only (fast layout check)")
    ap.add_argument(
        "--only",
        default="",
        help="substring filter on env_id -- rebuild one behaviour without re-running "
        "Blender for the whole atomic set (e.g. --only beam)",
    )
    args = ap.parse_args()

    spec = SPECS[args.version]
    if args.version == "v1":
        print("WARNING: v1 is the frozen geometry of the published clips.", flush=True)
    for env_id, (seed, types) in spec["plans"].items():
        if args.only and args.only not in env_id:
            continue
        env = build_env(env_id, seed, types, spec, args.version)
        export(env, types, args.version, spec, build_fbx=not args.no_fbx)
        centers, surfaces = gaps(env)
        cp = [round(v, 3) for o in env.obstacles for v in (o.constraint_params or {}).values()]
        print(
            f"[{env_id}] L={env.length} D={env.env_difficulty:.2f} "
            f"{[o.obstacle_type for o in env.obstacles]} {cp}\n"
            f"    x       = {[round(float(o.position[0]), 2) for o in env.obstacles]}\n"
            f"    center  = {[round(g, 2) for g in centers]}\n"
            f"    surface = {[round(g, 2) for g in surfaces]}",
            flush=True,
        )
    print("DONE")


if __name__ == "__main__":
    main()
