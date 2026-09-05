"""
Obstacle data model + per-type procedural geometry generators.

v2 (2026-05-20): structural obstacles now mirror disaster_scene_generator.py
patterns (rebar cylinders, I-beams, cross beams, pipe bundles, concrete wall
extensions). Floor obstacles (leg_clearance, block_with_clutter clutter)
load real Objaverse USD assets via the registry loader.

Each generator below:
  1. Samples its geometric parameter(s).
  2. Builds an ObstacleInstance with type-specific geometric_params.
  3. Computes AABBs that the placement layer uses for overlap checks.
  4. Annotates the instance with its constraint(s) and per-instance d_i.

GEOMETRY CONVENTIONS (corridor frame):
  +x = travel direction (origin -> goal)
  +y = lateral (0 .. corridor_width)
  +z = up (floor at z=0)
  Position fields store the CENTER of the bounding box.
  AABBs are axis-aligned in the corridor frame and stored as (min_xyz, max_xyz).

ELEMENT PRIMITIVES (used inside geometric_params["elements"]):
  - box      : {primitive, size_xyz, position, rotation_deg}
  - cylinder : {primitive, radius, length, position, rotation_deg}
               axis along local X before rotation (matching disaster code).
  - ibeam    : {primitive, flange_w, flange_t, web_h, web_t, length, position, rotation_deg}
  - asset    : {primitive, asset_ref, asset_class, size_xyz, position, yaw_deg}
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Iterator

import numpy as np

from ._objaverse_stub import AssetRecord, load_objaverse_asset
from .constraint_rules import (
    CONSTRAINT_PARAM_RANGES,
    compute_constraint_difficulty,
    constraints_for_geometry,
)

# Identity quaternion (x, y, z, w)
IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)


# G1 morphology
G1_SHOULDER_WIDTH_M = 0.45
LATERAL_BYPASS_MARGIN = 0.40  # free side width must be < shoulder - safety


@dataclass
class ObstacleInstance:
    obstacle_id: str
    obstacle_type: str  # "beam" | "dog_hole" | "ground" | "lateral_block" | "wall_squeeze" | "block_with_clutter"
    asset_source: str  # "procedural:<type>:<variant>" | "objaverse:<class>"
    position: np.ndarray  # (3,) world frame, semantic center
    orientation: np.ndarray  # (4,) quaternion (x, y, z, w); identity for v1
    geometric_params: dict  # type-specific (see builders)

    constraints: list[str] = field(default_factory=list)
    constraint_params: dict = field(default_factory=dict)
    difficulty: float = 0.0

    human_demonstrated_intents: list[str] = field(default_factory=list)

    def aabbs(self) -> list[tuple[np.ndarray, np.ndarray]]:
        """All axis-aligned bounding boxes in corridor world frame."""
        return _AABB_BUILDERS[self.obstacle_type](self)

    def x_extent(self) -> tuple[float, float]:
        xs = [(lo[0], hi[0]) for lo, hi in self.aabbs()]
        return min(lo for lo, _ in xs), max(hi for _, hi in xs)


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------


def _euler_xyz_matrix(rot_deg) -> np.ndarray:
    """Build a 3x3 rotation matrix from Euler XYZ degrees (R = Rz @ Ry @ Rx)."""
    rx, ry, rz = (math.radians(a) for a in rot_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _rotated_box_aabb(size_xyz, center_xyz, rot_deg) -> tuple[np.ndarray, np.ndarray]:
    """AABB of a rotated box. Enumerate 8 corners, transform, envelope."""
    R = _euler_xyz_matrix(rot_deg)
    sx, sy, sz = (s / 2.0 for s in size_xyz)
    corners = np.array(
        [
            [-sx, -sy, -sz],
            [+sx, -sy, -sz],
            [+sx, +sy, -sz],
            [-sx, +sy, -sz],
            [-sx, -sy, +sz],
            [+sx, -sy, +sz],
            [+sx, +sy, +sz],
            [-sx, +sy, +sz],
        ]
    )
    rotated = corners @ R.T + np.asarray(center_xyz, dtype=np.float64)
    return rotated.min(axis=0), rotated.max(axis=0)


def _x_aligned_cylinder_aabb(center_xyz, radius, length, rot_deg) -> tuple[np.ndarray, np.ndarray]:
    """Conservative AABB of an X-aligned cylinder after rotation. Treats it
    as a thin rod between two endpoints with `radius` padding per axis."""
    R = _euler_xyz_matrix(rot_deg)
    half_l = length / 2.0
    p1 = R @ np.array([-half_l, 0.0, 0.0]) + np.asarray(center_xyz, dtype=np.float64)
    p2 = R @ np.array([+half_l, 0.0, 0.0]) + np.asarray(center_xyz, dtype=np.float64)
    lo = np.minimum(p1, p2) - radius
    hi = np.maximum(p1, p2) + radius
    return lo, hi


# ---------------------------------------------------------------------------
# ID helper
# ---------------------------------------------------------------------------
def _new_id(rng: random.Random, prefix: str) -> str:
    return f"{prefix}_{rng.randrange(16**6):06x}"


# ===========================================================================
# height_constraint -- 4-way uniform mix of disaster-style overhead barriers
# ===========================================================================

REBAR_RADIUS_RANGE = (0.008, 0.013)
PIPE_RADIUS_RANGE = (0.025, 0.055)
BEAM_BOTTOM_THICKNESS = 0.10  # vertical extent of cross_beam / ibeam approx


def make_beam(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
) -> ObstacleInstance:
    """height_constraint realized as one of 4 disaster-style variants:
        - wall_rebar_mirrored : rebar cylinders from BOTH walls meeting at center
        - wall_ibeam_full     : single H-profile beam spanning corridor
        - cross_beam_full     : thick rectangular beam spanning corridor
        - pipe_bundle_mirrored: pipes from both walls meeting at center

    All variants enforce the same constraint: passage_height H at bottom of
    the obstacle. Spans full corridor width so the centerline is blocked at H.
    """
    h_lo, h_hi = CONSTRAINT_PARAM_RANGES["height_constraint"]
    H = rng.uniform(h_lo, h_hi)

    variant = rng.choice(
        [
            "wall_rebar_mirrored",
            "wall_ibeam_full",
            "cross_beam_full",
            "pipe_bundle_mirrored",
        ]
    )

    if variant == "wall_rebar_mirrored":
        elements = _build_rebar_mirrored(rng, x_center, corridor_width, H)
    elif variant == "wall_ibeam_full":
        elements = _build_ibeam_full(rng, x_center, corridor_width, H)
    elif variant == "cross_beam_full":
        elements = _build_cross_beam_full(rng, x_center, corridor_width, H)
    else:  # pipe_bundle_mirrored
        elements = _build_pipe_bundle_mirrored(rng, x_center, corridor_width, H)

    obs = ObstacleInstance(
        obstacle_id=_new_id(rng, "beam"),
        obstacle_type="beam",
        asset_source=f"procedural:beam:{variant}",
        position=np.array([x_center, corridor_width / 2.0, H], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "variant": variant,
            "passage_height": float(H),
            "elements": elements,
        },
    )
    _annotate_constraints(obs, {"height_constraint": float(H)})
    return obs


def _build_rebar_mirrored(rng, x_center, W, H) -> list[dict]:
    """Rebar from both walls. Each side fires 3-5 rebar; lengths span past
    centerline so the cluster blocks the corridor centerline at height H."""
    elements: list[dict] = []
    half_w = W / 2.0
    for side in ("left", "right"):
        n = rng.randint(3, 5)
        # Length: at least 60% past centerline so opposite-wall coverage overlaps.
        # Per side: 60-100% of half-width on that side.
        for i in range(n):
            radius = rng.uniform(*REBAR_RADIUS_RANGE)
            length = rng.uniform(half_w * 1.10, half_w * 1.55)
            z_off = rng.uniform(-0.05, 0.05)
            y_off = rng.uniform(-0.12, 0.12)
            pitch = rng.uniform(-3.0, 3.0)
            yaw_jit = rng.uniform(-3.0, 3.0)
            if side == "left":
                bar_x = x_center
                # Bar centered at length/2 from y=0
                bar_y = length / 2.0 + y_off
                rot_z = 90.0 + yaw_jit  # axis from -Y to +Y
            else:
                bar_x = x_center
                bar_y = W - length / 2.0 + y_off
                rot_z = 90.0 + yaw_jit
            bar_z = H + z_off + radius  # bottom at H, center slightly above
            elements.append(
                {
                    "primitive": "cylinder",
                    "radius": round(radius, 4),
                    "length": round(length, 4),
                    "position": [round(bar_x, 4), round(bar_y, 4), round(bar_z, 4)],
                    "rotation_deg": [round(pitch, 2), 0.0, round(rot_z, 2)],
                    "material": "rebar",
                }
            )
    return elements


def _build_ibeam_full(rng, x_center, W, H) -> list[dict]:
    """One H-profile beam spanning the full corridor width."""
    flange_w = rng.uniform(0.14, 0.22)
    flange_t = rng.uniform(0.010, 0.015)
    web_h = rng.uniform(0.14, 0.25)
    web_t = rng.uniform(0.008, 0.012)
    total_h = web_h + 2 * flange_t
    droop = rng.uniform(-4.0, 4.0)  # small droop along corridor

    # Beam axis along Y (cross-corridor). It is X-aligned in local frame
    # then yawed 90 deg to span Y.
    return [
        {
            "primitive": "ibeam",
            "flange_w": round(flange_w, 4),
            "flange_t": round(flange_t, 4),
            "web_h": round(web_h, 4),
            "web_t": round(web_t, 4),
            "length": round(W + 0.10, 4),  # slightly past walls
            "position": [round(x_center, 4), round(W / 2.0, 4), round(H + total_h / 2.0, 4)],
            "rotation_deg": [0.0, round(droop, 2), 90.0],
            "material": "steel",
        }
    ]


def _build_cross_beam_full(rng, x_center, W, H) -> list[dict]:
    """Heavy rectangular cross beam spanning corridor."""
    beam_h = rng.uniform(0.12, 0.22)
    beam_depth = rng.uniform(0.16, 0.28)
    droop_x = rng.uniform(-2.0, 2.0)
    yaw_jit = rng.uniform(-3.0, 3.0)
    material = rng.choice(["concrete", "steel", "wood"])
    return [
        {
            "primitive": "box",
            "size_xyz": [round(beam_depth, 4), round(W + 0.10, 4), round(beam_h, 4)],
            "position": [round(x_center, 4), round(W / 2.0, 4), round(H + beam_h / 2.0, 4)],
            "rotation_deg": [round(droop_x, 2), 0.0, round(yaw_jit, 2)],
            "material": material,
        }
    ]


def _build_pipe_bundle_mirrored(rng, x_center, W, H) -> list[dict]:
    """Pipes from both walls meeting in the middle."""
    elements: list[dict] = []
    half_w = W / 2.0
    for side in ("left", "right"):
        n = rng.randint(2, 4)
        for i in range(n):
            radius = rng.uniform(*PIPE_RADIUS_RANGE)
            length = rng.uniform(half_w * 1.10, half_w * 1.40)
            z_off = rng.uniform(-0.05, 0.05)
            y_off = rng.uniform(-0.08, 0.08)
            pitch = rng.uniform(-3.0, 3.0)
            yaw_jit = rng.uniform(-3.0, 3.0)
            if side == "left":
                pipe_x = x_center
                pipe_y = length / 2.0 + y_off
                rot_z = 90.0 + yaw_jit
            else:
                pipe_x = x_center
                pipe_y = W - length / 2.0 + y_off
                rot_z = 90.0 + yaw_jit
            pipe_z = H + z_off + radius
            elements.append(
                {
                    "primitive": "cylinder",
                    "radius": round(radius, 4),
                    "length": round(length, 4),
                    "position": [round(pipe_x, 4), round(pipe_y, 4), round(pipe_z, 4)],
                    "rotation_deg": [round(pitch, 2), 0.0, round(rot_z, 2)],
                    "material": "pipe_metal",
                }
            )
    return elements


# ===========================================================================
# full_body_lowering -- dog_hole (procedural wall with floor-level opening)
# ===========================================================================

DOG_HOLE_WALL_THICKNESS_X = 0.30
DOG_HOLE_OPENING_WIDTH_Y = 1.20
DOG_HOLE_MIN_STUB_Y = 0.15  # keep >= this wall on each side of the opening


def _jitter_lateral_center(
    rng: random.Random, corridor_width: float, span_y: float, min_side: float
) -> float:
    """Sample a lateral center for a span of width `span_y` inside the corridor,
    keeping at least `min_side` of clearance to each wall. Falls back to the
    corridor centerline when the corridor is too narrow to jitter."""
    lo = span_y / 2.0 + min_side
    hi = corridor_width - span_y / 2.0 - min_side
    return rng.uniform(lo, hi) if hi > lo else corridor_width / 2.0


def make_dog_hole(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
    wall_height: float,
) -> ObstacleInstance:
    """Solid concrete wall with a rectangular floor-level opening.

    The opening is jittered laterally (not always centered) so left/right stubs
    are asymmetric; a min stub width is kept on each side so it stays a hole in
    a wall rather than an edge gap. `corridor_width` is stored explicitly so
    downstream AABB/export code no longer has to assume the opening is centered.
    """
    h_lo, h_hi = CONSTRAINT_PARAM_RANGES["full_body_lowering"]
    H = rng.uniform(h_lo, h_hi)
    cy_center = _jitter_lateral_center(
        rng, corridor_width, DOG_HOLE_OPENING_WIDTH_Y, min_side=DOG_HOLE_MIN_STUB_Y
    )
    obs = ObstacleInstance(
        obstacle_id=_new_id(rng, "doghole"),
        obstacle_type="dog_hole",
        asset_source="procedural:dog_hole",
        position=np.array([x_center, cy_center, wall_height / 2.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "wall_thickness_x": DOG_HOLE_WALL_THICKNESS_X,
            "opening_top_z": float(H),
            "opening_width_y": DOG_HOLE_OPENING_WIDTH_Y,
            "wall_height_z": float(wall_height),
            "corridor_width": float(corridor_width),
            "material": "concrete",
        },
    )
    _annotate_constraints(obs, {"full_body_lowering": float(H)})
    return obs


# ===========================================================================
# leg_clearance -- Objaverse asset cluster
# ===========================================================================

GROUND_CLUSTER_DEPTH_X = (0.55, 0.90)
GROUND_CLUSTER_WIDTH_FRACTION = (0.70, 0.95)
GROUND_CLUSTER_NUM_ITEMS = (6, 12)
# Per-item asset filter (height in metres). Lower bound is the constraint
# floor; upper bound is the constraint cap, so items match H_max budget.
# Bumped from 0.05 -> 0.15 so a Livox MID-360 mounted at ~1.2m can reliably
# pick up the obstacle (sub-15cm assets fall below the LiDAR's ray density
# near the ground at typical corridor distances).
GROUND_ASSET_HEIGHT_MIN = 0.15
GROUND_ASSET_FOOTPRINT_MAX = 0.55  # don't pick assets bigger than this in XY


def make_ground(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
) -> ObstacleInstance:
    """leg_clearance: cluster of REAL Objaverse assets piled across the corridor.

    Each item is a USD asset reference (from bedroom/livingroom/kitchen
    registries). Heights filtered to <= H_max (the sampled constraint
    parameter). Item asset class is drawn uniformly across registry.
    """
    h_lo, h_hi = CONSTRAINT_PARAM_RANGES["leg_clearance"]
    H_max = rng.uniform(h_lo, h_hi)

    cluster_depth_x = rng.uniform(*GROUND_CLUSTER_DEPTH_X)
    width_frac = rng.uniform(*GROUND_CLUSTER_WIDTH_FRACTION)
    cluster_width_y = width_frac * corridor_width

    half_w = cluster_width_y / 2.0
    margin = 0.05
    cy_lo = half_w + margin
    cy_hi = corridor_width - half_w - margin
    cy_center = rng.uniform(cy_lo, cy_hi) if cy_hi > cy_lo else corridor_width / 2.0

    n_items = rng.randint(*GROUND_CLUSTER_NUM_ITEMS)
    items: list[dict] = []
    for _ in range(n_items):
        try:
            rec = load_objaverse_asset(
                category="any",
                size_constraint={
                    "height_min": GROUND_ASSET_HEIGHT_MIN,
                    "height_max": H_max,
                    "footprint_max_xy": GROUND_ASSET_FOOTPRINT_MAX,
                },
                rng=rng,
            )
        except LookupError:
            continue  # registry exhausted by filter -- skip this slot
        sx, sy, sz = rec.size_xyz
        # Item position within cluster footprint; item must fit.
        ox_max = max(0.0, cluster_depth_x / 2.0 - sx / 2.0)
        oy_max = max(0.0, cluster_width_y / 2.0 - sy / 2.0)
        ox = rng.uniform(-ox_max, ox_max) if ox_max > 0 else 0.0
        oy = rng.uniform(-oy_max, oy_max) if oy_max > 0 else 0.0
        yaw = rng.uniform(0.0, 360.0)
        # Asset center placed at (x_center+ox, cy_center+oy, sz/2 - asset_center_z)
        # The USD asset has its own AABB center; when imported into Blender,
        # if the asset's local center is `rec.center_xyz`, we offset so the
        # asset's AABB bottom sits at z=0.
        items.append(
            {
                "primitive": "asset",
                "asset_ref": str(rec.usd_path),
                "asset_class": rec.cls,
                "asset_name": rec.asset,
                "size_xyz": [float(sx), float(sy), float(sz)],
                "asset_center_xyz": rec.center_xyz.tolist(),
                "position": [float(x_center + ox), float(cy_center + oy), float(sz / 2.0)],
                "yaw_deg": float(yaw),
            }
        )

    # Fallback: no Objaverse assets available (registry empty / not yet
    # downloaded) -> build a procedural box cluster so the obstacle is
    # never geometry-less. This keeps the benchmark fully standalone.
    if not items:
        asset_source = "procedural:ground_boxes"
        for _ in range(n_items):
            bz = rng.uniform(GROUND_ASSET_HEIGHT_MIN, H_max)
            bx = rng.uniform(0.12, min(GROUND_ASSET_FOOTPRINT_MAX, cluster_depth_x))
            by = rng.uniform(0.12, min(GROUND_ASSET_FOOTPRINT_MAX, cluster_width_y))
            ox_max = max(0.0, cluster_depth_x / 2.0 - bx / 2.0)
            oy_max = max(0.0, cluster_width_y / 2.0 - by / 2.0)
            ox = rng.uniform(-ox_max, ox_max) if ox_max > 0 else 0.0
            oy = rng.uniform(-oy_max, oy_max) if oy_max > 0 else 0.0
            items.append(
                {
                    "primitive": "box",
                    "size_xyz": [float(bx), float(by), float(bz)],
                    "position": [float(x_center + ox), float(cy_center + oy), float(bz / 2.0)],
                    "rotation_deg": [0.0, 0.0, float(rng.uniform(0.0, 360.0))],
                }
            )
    else:
        asset_source = "objaverse:cluster"

    obs = ObstacleInstance(
        obstacle_id=_new_id(rng, "ground"),
        obstacle_type="ground",
        asset_source=asset_source,
        position=np.array([x_center, cy_center, 0.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "cluster_depth_x": float(cluster_depth_x),
            "cluster_width_y": float(cluster_width_y),
            "cluster_center_y": float(cy_center),
            "height_max": float(H_max),
            "items": items,
        },
    )
    _annotate_constraints(obs, {"leg_clearance": float(H_max)})
    return obs


# ===========================================================================
# lateral_passage -- procedural concrete block against one wall
# ===========================================================================

LATERAL_BLOCK_DEPTH_X = 0.40
LATERAL_BLOCK_HEIGHT = 1.80


def make_lateral_block(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
) -> ObstacleInstance:
    p_lo, p_hi = CONSTRAINT_PARAM_RANGES["lateral_passage"]
    p = rng.uniform(p_lo, p_hi)

    span_y = p * corridor_width
    side = rng.choice(["left", "right"])
    cy = span_y / 2.0 if side == "left" else corridor_width - span_y / 2.0

    obs = ObstacleInstance(
        obstacle_id=_new_id(rng, "lblk"),
        obstacle_type="lateral_block",
        asset_source="procedural:lateral_block",
        position=np.array([x_center, cy, LATERAL_BLOCK_HEIGHT / 2.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "size_xyz": [LATERAL_BLOCK_DEPTH_X, float(span_y), LATERAL_BLOCK_HEIGHT],
            "wall_side": side,
            "occupancy": float(p),
            "material": "concrete",
        },
    )
    _annotate_constraints(obs, {"lateral_passage": float(p)})
    return obs


# ===========================================================================
# lateral_clearance -- TWO wall extensions (was: pillars).
# Per user: don't use load-bearing pillars; use wall protrusions that
# narrow the corridor to gap g around centerline. Floor-to-ceiling so
# stepping over isn't an option.
# ===========================================================================

WALL_SQUEEZE_DEPTH_X = 0.30  # along travel
WALL_SQUEEZE_MIN_EXT_Y = 0.10  # keep >= this extension from each wall


def make_pillars(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
    wall_height: float = 2.5,
) -> ObstacleInstance:
    """lateral_clearance realized as two wall extensions from y=0 and y=W,
    leaving a centered gap g.

    Each extension is a concrete box, floor-to-ceiling, depth WALL_SQUEEZE_DEPTH_X
    along travel.

        wall y=0 ──────────┐                       ┌────────── wall y=W
                           │                       │
                           │   <- gap g (centered) │
                           │                       │
        wall y=0 ──────────┘                       └────────── wall y=W

    The obstacle_type stays "wall_squeeze" but the SOLE constraint is
    `lateral_clearance` (gap g requires shoulder turn).
    """
    g_lo, g_hi = CONSTRAINT_PARAM_RANGES["lateral_clearance"]
    g = rng.uniform(g_lo, g_hi)

    # Gap center is jittered laterally (not fixed to the centerline), so the two
    # wall extensions are asymmetric -- one long, one short -- while the gap
    # width g (the constraint) is unchanged.
    half_gap = g / 2.0
    cy_center = _jitter_lateral_center(rng, corridor_width, g, min_side=WALL_SQUEEZE_MIN_EXT_Y)
    left_ext_y = max(0.10, cy_center - half_gap)
    right_ext_y = max(0.10, corridor_width - (cy_center + half_gap))

    obs = ObstacleInstance(
        obstacle_id=_new_id(rng, "squeeze"),
        obstacle_type="wall_squeeze",
        asset_source="procedural:wall_squeeze",
        position=np.array([x_center, cy_center, wall_height / 2.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "gap_y": float(g),
            "left_size_xyz": [WALL_SQUEEZE_DEPTH_X, float(left_ext_y), float(wall_height)],
            "right_size_xyz": [WALL_SQUEEZE_DEPTH_X, float(right_ext_y), float(wall_height)],
            "wall_height": float(wall_height),
            "corridor_width": float(corridor_width),
            "material": "concrete",
        },
    )
    _annotate_constraints(obs, {"lateral_clearance": float(g)})
    return obs


# ===========================================================================
# block_with_clutter -- procedural block + Objaverse clutter in free passage
# ===========================================================================

BLOCK_CLUTTER_DEPTH_X = 0.55
BLOCK_CLUTTER_NUM_ITEMS = (4, 8)


def make_block_with_clutter(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
) -> ObstacleInstance:
    p_lo, p_hi = CONSTRAINT_PARAM_RANGES["lateral_passage"]
    p = rng.uniform(p_lo, p_hi)

    h_lo, h_hi = CONSTRAINT_PARAM_RANGES["leg_clearance"]
    H_max = rng.uniform(h_lo, h_hi)

    side = rng.choice(["left", "right"])
    span_y = p * corridor_width
    if side == "left":
        block_cy = span_y / 2.0
        passage_lo, passage_hi = span_y, corridor_width
    else:
        block_cy = corridor_width - span_y / 2.0
        passage_lo, passage_hi = 0.0, corridor_width - span_y

    block_size = np.array([LATERAL_BLOCK_DEPTH_X, span_y, LATERAL_BLOCK_HEIGHT], dtype=np.float64)
    block_center = np.array([x_center, block_cy, LATERAL_BLOCK_HEIGHT / 2.0], dtype=np.float64)

    passage_width = passage_hi - passage_lo
    asset_footprint_cap = min(GROUND_ASSET_FOOTPRINT_MAX, passage_width * 0.55)

    n_items = rng.randint(*BLOCK_CLUTTER_NUM_ITEMS)
    clutter: list[dict] = []
    for _ in range(n_items):
        try:
            rec = load_objaverse_asset(
                category="any",
                size_constraint={
                    "height_min": GROUND_ASSET_HEIGHT_MIN,
                    "height_max": H_max,
                    "footprint_max_xy": asset_footprint_cap,
                },
                rng=rng,
            )
        except LookupError:
            continue
        sx, sy, sz = rec.size_xyz
        iy_lo = passage_lo + sy / 2.0
        iy_hi = passage_hi - sy / 2.0
        if iy_hi <= iy_lo:
            continue
        iy = rng.uniform(iy_lo, iy_hi)
        ix_lo = x_center - BLOCK_CLUTTER_DEPTH_X / 2.0 + sx / 2.0
        ix_hi = x_center + BLOCK_CLUTTER_DEPTH_X / 2.0 - sx / 2.0
        ix = rng.uniform(ix_lo, ix_hi) if ix_hi > ix_lo else float(x_center)
        yaw = rng.uniform(0.0, 360.0)
        clutter.append(
            {
                "primitive": "asset",
                "asset_ref": str(rec.usd_path),
                "asset_class": rec.cls,
                "asset_name": rec.asset,
                "size_xyz": [float(sx), float(sy), float(sz)],
                "asset_center_xyz": rec.center_xyz.tolist(),
                "position": [float(ix), float(iy), float(sz / 2.0)],
                "yaw_deg": float(yaw),
            }
        )

    obs = ObstacleInstance(
        obstacle_id=_new_id(rng, "bwclut"),
        obstacle_type="block_with_clutter",
        asset_source="procedural:block+objaverse:clutter",
        position=np.array([x_center, corridor_width / 2.0, 0.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "block_size_xyz": block_size.tolist(),
            "block_center_xyz": block_center.tolist(),
            "wall_side": side,
            "block_occupancy": float(p),
            "clutter_items": clutter,
            "clutter_height": float(H_max),
        },
    )
    _annotate_constraints(
        obs,
        {
            "lateral_passage": float(p),
            "leg_clearance": float(H_max),
        },
    )
    return obs


# ===========================================================================
# leaning_columns -- tilted columns rising from the floor to form a triangular
# constrained space (A-frame tent, or a single lean-to against a wall).
# GEOMETRY ONLY: deliberately NOT tied to a scored constraint, so difficulty
# stays 0 and it does not perturb the benchmark difficulty scoring. Injected
# purely for environment variety.
# ===========================================================================

LEANING_COL_DEPTH_X = 0.28  # thickness along travel
LEANING_COL_THICK_Y = 0.24  # thickness across the tilt
LEANING_TILT_DEG = (26.0, 40.0)  # tilt from vertical
LEANING_BASE_MARGIN = 0.12  # base offset from the wall
LEANING_APEX_MAX_FRAC = 0.92  # apex height cap as fraction of wall height


def _tilted_column_element(name, x_center, y_base, lean_plus_y, tilt_deg, col_len):
    """One tilted box column with its base on the floor at y_base, leaning
    toward +y (lean_plus_y) or -y. Tilt is about the corridor's x-axis, which
    composes correctly with the downstream export yaw (see export_to_unity)."""
    a = np.radians(tilt_deg)
    h = col_len / 2.0
    dy = h * np.sin(a)
    cy = y_base + dy if lean_plus_y else y_base - dy
    cz = h * np.cos(a)
    rot_x = -tilt_deg if lean_plus_y else tilt_deg
    return {
        "primitive": "box",
        "name": name,
        "size_xyz": [LEANING_COL_DEPTH_X, LEANING_COL_THICK_Y, float(col_len)],
        "position": [float(x_center), float(cy), float(cz)],
        "rotation_deg": [float(rot_x), 0.0, 0.0],
        "material": "concrete",
    }


def make_leaning_columns(
    rng: random.Random,
    x_center: float,
    corridor_width: float,
    wall_height: float = 2.5,
) -> ObstacleInstance:
    """Tilted columns forming a triangular constrained space (geometry only)."""
    W = corridor_width
    tilt = rng.uniform(*LEANING_TILT_DEG)
    a = np.radians(tilt)
    apex_cap = wall_height * LEANING_APEX_MAX_FRAC
    variant = rng.choice(["a_frame", "lean_to"])
    base = _new_id(rng, "lean")
    elements: list[dict] = []

    if variant == "a_frame":
        yb_l, yb_r = LEANING_BASE_MARGIN, W - LEANING_BASE_MARGIN
        col_len = (W / 2.0 - yb_l) / max(np.sin(a), 1e-3)
        col_len = min(col_len, apex_cap / max(np.cos(a), 1e-3))
        elements.append(_tilted_column_element(f"{base}_L", x_center, yb_l, True, tilt, col_len))
        elements.append(_tilted_column_element(f"{base}_R", x_center, yb_r, False, tilt, col_len))
    else:  # lean_to: single column leaning against one wall
        if rng.choice(["left", "right"]) == "left":
            yb, lean_plus, reach = min(W * 0.60, W - LEANING_BASE_MARGIN), False, None
            reach = yb
        else:
            yb, lean_plus = max(W * 0.40, LEANING_BASE_MARGIN), True
            reach = W - yb
        col_len = reach / max(np.sin(a), 1e-3)
        col_len = min(col_len, apex_cap / max(np.cos(a), 1e-3))
        elements.append(_tilted_column_element(f"{base}_C", x_center, yb, lean_plus, tilt, col_len))

    return ObstacleInstance(
        obstacle_id=base,
        obstacle_type="leaning_columns",
        asset_source=f"procedural:leaning_columns:{variant}",
        position=np.array([x_center, W / 2.0, 0.0], dtype=np.float64),
        orientation=IDENTITY_QUAT.copy(),
        geometric_params={
            "variant": variant,
            "tilt_deg": float(tilt),
            "elements": elements,
            "material": "concrete",
        },
    )
    # No _annotate_constraints: geometry-only, difficulty stays 0.


# ---------------------------------------------------------------------------
# Builder dispatch
# ---------------------------------------------------------------------------

BUILDER_BY_OBSTACLE_TYPE = {
    "beam": make_beam,
    "dog_hole": make_dog_hole,
    "ground": make_ground,
    "lateral_block": make_lateral_block,
    "wall_squeeze": make_pillars,  # function renamed but key uses obstacle_type
    "block_with_clutter": make_block_with_clutter,
    "leaning_columns": make_leaning_columns,
}


# ===========================================================================
# AABB builders
# ===========================================================================


def _aabb_from_center_size(center, size):
    half = np.asarray(size, dtype=np.float64) / 2.0
    c = np.asarray(center, dtype=np.float64)
    return c - half, c + half


def _aabb_for_element(el: dict) -> tuple[np.ndarray, np.ndarray]:
    prim = el["primitive"]
    if prim == "box":
        return _rotated_box_aabb(el["size_xyz"], el["position"], el.get("rotation_deg", [0, 0, 0]))
    if prim == "cylinder":
        return _x_aligned_cylinder_aabb(
            el["position"],
            el["radius"],
            el["length"],
            el.get("rotation_deg", [0, 0, 0]),
        )
    if prim == "ibeam":
        flange_w = el["flange_w"]
        web_h = el["web_h"]
        flange_t = el["flange_t"]
        total_h = web_h + 2 * flange_t
        # Approximate as a box [length, flange_w, total_h].
        size = [el["length"], flange_w, total_h]
        return _rotated_box_aabb(size, el["position"], el.get("rotation_deg", [0, 0, 0]))
    if prim == "asset":
        size = el["size_xyz"]
        center = el["position"]
        return _rotated_box_aabb(size, center, [0, 0, el.get("yaw_deg", 0.0)])
    raise ValueError(f"Unknown primitive: {prim}")


def _aabbs_beam(obs: ObstacleInstance) -> list[tuple[np.ndarray, np.ndarray]]:
    return [_aabb_for_element(el) for el in obs.geometric_params["elements"]]


def _aabbs_dog_hole(obs: ObstacleInstance) -> list[tuple[np.ndarray, np.ndarray]]:
    gp = obs.geometric_params
    thickness_x = gp["wall_thickness_x"]
    opening_top = gp["opening_top_z"]
    opening_w_y = gp["opening_width_y"]
    wall_h_z = gp["wall_height_z"]

    cx = obs.position[0]
    cy_center = obs.position[1]
    cw = gp.get("corridor_width", cy_center * 2.0)

    opening_lo_y = cy_center - opening_w_y / 2.0
    opening_hi_y = cy_center + opening_w_y / 2.0
    half_x = thickness_x / 2.0
    x_lo = cx - half_x
    x_hi = cx + half_x

    aabbs = []
    if opening_lo_y > 0:
        aabbs.append(
            (
                np.array([x_lo, 0.0, 0.0]),
                np.array([x_hi, opening_lo_y, wall_h_z]),
            )
        )
    if opening_hi_y < cw:
        aabbs.append(
            (
                np.array([x_lo, opening_hi_y, 0.0]),
                np.array([x_hi, cw, wall_h_z]),
            )
        )
    aabbs.append(
        (
            np.array([x_lo, opening_lo_y, opening_top]),
            np.array([x_hi, opening_hi_y, wall_h_z]),
        )
    )
    return aabbs


def _aabbs_ground(obs: ObstacleInstance) -> list[tuple[np.ndarray, np.ndarray]]:
    return [_aabb_for_element(item) for item in obs.geometric_params["items"]]


def _aabbs_lateral_block(obs: ObstacleInstance) -> list[tuple[np.ndarray, np.ndarray]]:
    gp = obs.geometric_params
    # half_wall carries one box per protrusion under `walls`; the original
    # single-sided block carries one `size_xyz` at the obstacle position.
    if "walls" in gp:
        return [
            _aabb_from_center_size(
                np.array(w["position"], dtype=np.float64),
                np.array(w["size_xyz"], dtype=np.float64),
            )
            for w in gp["walls"]
        ]
    size = np.array(gp["size_xyz"], dtype=np.float64)
    return [_aabb_from_center_size(obs.position, size)]


def _aabbs_wall_squeeze(obs: ObstacleInstance) -> list[tuple[np.ndarray, np.ndarray]]:
    """Two wall-extension boxes."""
    gp = obs.geometric_params
    cx = obs.position[0]
    cy_center = obs.position[1]
    cw = gp.get("corridor_width", cy_center * 2.0)
    wall_h = gp["wall_height"]

    left_size = np.array(gp["left_size_xyz"], dtype=np.float64)
    right_size = np.array(gp["right_size_xyz"], dtype=np.float64)

    # Left extension: from y=0 to y=left_size[1]
    left_center = np.array([cx, left_size[1] / 2.0, wall_h / 2.0])
    # Right extension: from y = cw - right_size[1] to y = cw
    right_center = np.array([cx, cw - right_size[1] / 2.0, wall_h / 2.0])

    return [
        _aabb_from_center_size(left_center, left_size),
        _aabb_from_center_size(right_center, right_size),
    ]


def _aabbs_block_with_clutter(obs: ObstacleInstance) -> list[tuple[np.ndarray, np.ndarray]]:
    gp = obs.geometric_params
    block_center = np.array(gp["block_center_xyz"], dtype=np.float64)
    block_size = np.array(gp["block_size_xyz"], dtype=np.float64)
    aabbs = [_aabb_from_center_size(block_center, block_size)]
    for item in gp["clutter_items"]:
        aabbs.append(_aabb_for_element(item))
    return aabbs


def _aabbs_leaning(obs: ObstacleInstance) -> list[tuple[np.ndarray, np.ndarray]]:
    return [_aabb_for_element(el) for el in obs.geometric_params["elements"]]


_AABB_BUILDERS = {
    "beam": _aabbs_beam,
    "dog_hole": _aabbs_dog_hole,
    "ground": _aabbs_ground,
    "lateral_block": _aabbs_lateral_block,
    "wall_squeeze": _aabbs_wall_squeeze,
    "block_with_clutter": _aabbs_block_with_clutter,
    "leaning_columns": _aabbs_leaning,
}


# ---------------------------------------------------------------------------
# Constraint annotation
# ---------------------------------------------------------------------------


def _annotate_constraints(obs: ObstacleInstance, params: dict[str, float]) -> None:
    declared = constraints_for_geometry(obs.obstacle_type)
    if set(params.keys()) != set(declared):
        raise AssertionError(
            f"Obstacle {obs.obstacle_id} declared constraints {declared} but "
            f"got params for {tuple(params.keys())}"
        )
    obs.constraints = list(declared)
    obs.constraint_params = dict(params)
    obs.difficulty = max(compute_constraint_difficulty(c, v) for c, v in params.items())


def iter_obstacle_aabbs(obstacles) -> Iterator[tuple[ObstacleInstance, np.ndarray, np.ndarray]]:
    for o in obstacles:
        for lo, hi in o.aabbs():
            yield o, lo, hi
