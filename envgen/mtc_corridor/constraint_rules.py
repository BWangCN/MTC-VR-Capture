"""
Constraint ontology for MTC Challenge v1 (corridor scenario).

This module is the AUTHORITATIVE definition of the five geometric constraints
that an obstacle can impose on the Unitree G1 robot's body. It is intentionally
free of imports so it can be referenced from anywhere (generator, evaluator,
human-data annotation pipeline) without circular deps.

PAPER NARRATIVE (do not lose sight of this when extending):
    Human motion is a natural annotation of environment geometry. The
    constraint set below describes WHAT THE GEOMETRY RESTRICTS — never WHAT
    ACTION A HUMAN CHOOSES. The same constraint may admit multiple valid
    motions (e.g. height_constraint can be satisfied by ducking, by deep
    squat-walk, or by hands-and-knees crawl). Motion labels are downstream;
    constraints are upstream.

Constraint rules (closed set of 5):

  1. height_constraint
        Head/upper body must stay below height H.
        Triggered by overhead obstacles where H in [0.7m, 1.5m].
        Geometric realization: horizontal beam spanning corridor width,
        bottom edge at height H. Beam depth (along travel) kept thin
        (~0.1m) so the duck is brief; thick beams that would force
        sustained low posture are instead realized as dog_hole geometry
        (see full_body_lowering).

  2. full_body_lowering
        The ENTIRE body must be lowered, with strong implication of
        contact with the ground or near-ground support (crawl / deep
        squat-walk). Triggered by dog_hole geometry: a solid wall
        spanning the full corridor width, with a rectangular opening
        whose bottom is at the floor (z=0) and whose top is at
        height H in [0.35m, 0.6m].

  3. leg_clearance
        At least one leg must clear a minimum height. Triggered by
        ground obstacles of height H in [0.20m, 0.45m] sitting on the
        robot's forward path. Footprint may be procedural or (later)
        Objaverse asset.

  4. lateral_passage
        Lateral free width is reduced but normal upright walking still
        works by detouring laterally. Triggered when an obstacle
        occupies fraction p in [0.30, 0.70] of corridor width, placed
        against one wall (random side).

  5. lateral_clearance
        Robot's lateral body width must itself be reduced — sideways
        shoulder turn or shoulder-pull-in. Triggered by two-pillar
        geometry symmetric about corridor centerline, gap g in
        [0.5m, 0.7m]. Pillars are 1.5m tall so ducking under is not an
        option.

Multi-label semantics: each obstacle is annotated with a SET of constraints.
In v1 the geometry -> constraint-set mapping is one-to-one (see
GEOMETRY_TO_CONSTRAINTS below); we retain multi-label data structures so
future obstacle types (e.g. a low + narrow combined gap) can declare
multiple constraints without API changes.

Difficulty (per obstacle):

    height_constraint    d = clip((1.32 - H) / 1.32, 0, 1)
    full_body_lowering   d = 0.7 + 0.3 * clip((0.6 - H) / 0.25, 0, 1)
    leg_clearance        d = clip(H / 0.5, 0, 1)
    lateral_passage      d = clip(p, 0, 1)
    lateral_clearance    d = clip(1 - (g - 0.45) / 0.25, 0, 1)

When an obstacle declares multiple constraints, the obstacle's per-instance
d_i is the max over its constraint-specific d values.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

# --- Robot calibration constants ---------------------------------------------
# All difficulty formulas below are CALIBRATED TO THE G1 MORPHOLOGY. Do not
# reuse these constants for a different robot without re-deriving the
# normalizers.

G1_STANDING_HEIGHT_M = 1.32  # crown of head when upright
G1_SHOULDER_WIDTH_M = 0.45
G1_BODY_THICKNESS_M = 0.20
G1_MIN_KNEE_HEIGHT_M = 0.50  # asymptote for leg_clearance saturation


# --- Constraint name registry -------------------------------------------------
CONSTRAINT_TYPES = (
    "height_constraint",
    "full_body_lowering",
    "leg_clearance",
    "lateral_passage",
    "lateral_clearance",
)


# --- Constraint geometric parameter ranges ------------------------------------
# Used by the obstacle generators to SAMPLE realizable geometries, and by the
# tests to verify generated values fall within the declared ranges.
CONSTRAINT_PARAM_RANGES: dict[str, tuple[float, float]] = {
    # All ranges tightened on 2026-05-21 to ensure G1 retargetability.
    # Reasoning per bound is logged inline.
    "height_constraint": (0.85, 1.5),  # G1 deep-squat head height ~0.85m (per HOMIE).
    # (The 2026-07-20 +10cm raise to 0.95 was
    # reverted -- beam back to its earlier height.)
    "full_body_lowering": (0.60, 0.70),  # Whole range raised +10cm on
    # 2026-07-20 (0.50-0.60 -> 0.60-0.70):
    # taller dog-hole opening. G1 min crawl
    # height ~0.50m, so 0.60 keeps margin.
    "leg_clearance": (0.20, 0.28),  # Upper bound cut 0.45 -> 0.28 on 2026-07-22:
    # 0.45m sits at pilot/G1 knee height and
    # proved too high to step over in VR
    # capture. 0.28 matches the curated
    # standard envs. Lower bound stays 0.20 so
    # H_max stays above the
    # GROUND_ASSET_HEIGHT_MIN=0.15 filter
    # (keeps a non-empty asset pool, and keeps
    # the uniform(MIN, H_max) box fallback from
    # inverting).
    "lateral_passage": (0.30, 0.60),  # p<=0.60 in W>=1.8m -> free passage
    # >=0.72m, comfortably > shoulder 0.45m.
    # Compound block_with_clutter step-over
    # then doesn't require lateral squeeze.
    "lateral_clearance": (0.5, 0.7),  # g_min=0.5 forces shoulder rotation
    # (intended constraint design).
}


# --- Difficulty formulas ------------------------------------------------------
def _d_height_constraint(H: float) -> float:
    """Beam bottom at height H; lower beam => harder."""
    return _clip01((G1_STANDING_HEIGHT_M - H) / G1_STANDING_HEIGHT_M)


def _d_full_body_lowering(H: float) -> float:
    """Dog-hole opening top at H; baseline 0.7 because any crawl is hard."""
    return 0.7 + 0.3 * _clip01((0.6 - H) / 0.25)


def _d_leg_clearance(H: float) -> float:
    """Ground obstacle of height H; harder as H approaches knee height."""
    return _clip01(H / G1_MIN_KNEE_HEIGHT_M)


def _d_lateral_passage(p: float) -> float:
    """Fraction p of corridor occupied; more occupation => more detour."""
    return _clip01(p)


def _d_lateral_clearance(g: float) -> float:
    """Gap g between two pillars; gap at shoulder width = max difficulty."""
    return _clip01(1.0 - (g - G1_SHOULDER_WIDTH_M) / 0.25)


def _clip01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


DIFFICULTY_FORMULAS: dict[str, Callable[[float], float]] = {
    "height_constraint": _d_height_constraint,
    "full_body_lowering": _d_full_body_lowering,
    "leg_clearance": _d_leg_clearance,
    "lateral_passage": _d_lateral_passage,
    "lateral_clearance": _d_lateral_clearance,
}


# --- Geometry -> constraints mapping ------------------------------------------
# obstacle_type names are the strings stored on ObstacleInstance.obstacle_type.
# v1 is mostly 1-to-1; `block_with_clutter` declares TWO constraints because
# its geometry forces both side-step and step-over.
GEOMETRY_TO_CONSTRAINTS: dict[str, tuple[str, ...]] = {
    "beam": ("height_constraint",),
    "dog_hole": ("full_body_lowering",),
    "ground": ("leg_clearance",),
    "lateral_block": ("lateral_passage",),
    "wall_squeeze": ("lateral_clearance",),
    "block_with_clutter": ("lateral_passage", "leg_clearance"),
}


# When a sampled constraint can be realized by more than one obstacle type,
# the generator picks among the candidates by these weights (sums need not
# equal 1; sampled proportionally). Constraints not listed here use the
# single mapping in GEOMETRY_TO_CONSTRAINTS.
CONSTRAINT_TO_OBSTACLE_OPTIONS: dict[str, dict[str, float]] = {
    "lateral_passage": {
        "lateral_block": 0.55,
        "block_with_clutter": 0.45,  # compound forces side + step-over
    },
    "leg_clearance": {
        "ground": 0.70,
        "block_with_clutter": 0.30,  # compound counts toward both constraints
    },
}


# --- Tier mixtures ------------------------------------------------------------
# Probability of sampling each constraint type when building a corridor of a
# given tier. Must sum to 1.0 within each tier. Anchor enforcement (the
# guaranteed-present constraints) is handled in corridor.py, NOT here.
TIER_CONSTRAINT_MIXTURE: dict[str, dict[str, float]] = {
    "easy": {
        "lateral_passage": 0.40,
        "leg_clearance": 0.30,
        "height_constraint": 0.20,
        "lateral_clearance": 0.10,
        "full_body_lowering": 0.00,
    },
    "medium": {
        "height_constraint": 0.35,
        "leg_clearance": 0.25,
        "lateral_passage": 0.20,
        "lateral_clearance": 0.15,
        "full_body_lowering": 0.05,
    },
    "hard": {
        "full_body_lowering": 0.25,
        "lateral_clearance": 0.20,
        "height_constraint": 0.25,
        "leg_clearance": 0.20,
        "lateral_passage": 0.10,
    },
}


# Per-tier obstacle counts. Reduced from the original spec values to satisfy
# the hard rule corridor_length <= 8.0m (see TIER_LENGTH_RANGE below) WITH a
# strict 1.0m minimum spacing between adjacent obstacle centers.
# Slot width = (L - 2.0) / n >= 1.0 forces n <= L - 2 per env.
TIER_OBSTACLE_COUNT_RANGE: dict[str, tuple[int, int]] = {
    "easy": (2, 3),
    "medium": (4, 5),
    "hard": (5, 6),
}


# Per-tier corridor length ranges. The lower bound is chosen so that
# slot_width >= 1.0m always holds for that tier's MAX obstacle count.
# All upper bounds are capped at 8.0m (data-collection environment hard rule).
TIER_LENGTH_RANGE: dict[str, tuple[float, float]] = {
    "easy": (5.0, 6.0),
    "medium": (6.0, 7.0),
    "hard": (7.0, 8.0),
}


# Global maximum (hard rule from data-collection environment).
MAX_CORRIDOR_LENGTH_M = 8.0


# --- Anchor enforcement -------------------------------------------------------
# Each tier guarantees the presence of certain constraints in every generated
# environment. The generator post-processes the sampled list to enforce these.
TIER_ANCHORS: dict[str, dict] = {
    "easy": {"required": (), "one_of": ()},
    "medium": {"required": ("height_constraint",), "one_of": ()},
    # For "hard", exactly one of {full_body_lowering, lateral_clearance}
    # is mandatory (sampled uniformly). Both being present is fine.
    "hard": {"required": (), "one_of": ("full_body_lowering", "lateral_clearance")},
}


# --- Color palette (used by viz) ----------------------------------------------
# Single source of truth so top-down, side-view and 3D renders agree.
CONSTRAINT_COLORS: dict[str, str] = {
    "height_constraint": "#ff8c1a",  # orange
    "full_body_lowering": "#d62728",  # red
    "leg_clearance": "#2ca02c",  # green
    "lateral_passage": "#1f77b4",  # blue
    "lateral_clearance": "#9467bd",  # purple
}
CORRIDOR_COLOR = "#d0d0d0"  # walls / floor light gray
START_MARKER_COLOR = "#17becf"  # cyan
GOAL_MARKER_COLOR = "#ffdd33"  # yellow

# Radius of the goal zone. This is simultaneously the success criterion
# (challenge/runner.py GOAL_TOLERANCE_M) and the radius of the ring rendered
# into the FBX for VR capture, so the circle a human sees in the headset is
# literally the circle the benchmark scores against.
GOAL_RADIUS_M = 0.50


# --- Helpers ------------------------------------------------------------------
def compute_constraint_difficulty(constraint_type: str, param: float) -> float:
    """Apply the per-constraint difficulty formula. Raises on unknown type."""
    if constraint_type not in DIFFICULTY_FORMULAS:
        raise ValueError(f"Unknown constraint type: {constraint_type!r}")
    return DIFFICULTY_FORMULAS[constraint_type](param)


def constraints_for_geometry(obstacle_type: str) -> tuple[str, ...]:
    """Return the constraint tuple declared by a procedural obstacle type."""
    if obstacle_type not in GEOMETRY_TO_CONSTRAINTS:
        raise ValueError(f"Unknown obstacle type: {obstacle_type!r}")
    return GEOMETRY_TO_CONSTRAINTS[obstacle_type]


def constraint_to_obstacle_type(constraint_type: str) -> str:
    """Single-option lookup: only well-defined when exactly one obstacle type
    declares this constraint as its sole constraint. For constraints with
    multiple candidate geometries, use `sample_obstacle_type_for_constraint`."""
    candidates = [
        obs_type for obs_type, cs in GEOMETRY_TO_CONSTRAINTS.items() if cs == (constraint_type,)
    ]
    if len(candidates) != 1:
        raise ValueError(
            f"constraint_to_obstacle_type({constraint_type!r}) has "
            f"{len(candidates)} candidate(s); use "
            f"sample_obstacle_type_for_constraint instead."
        )
    return candidates[0]


def sample_obstacle_type_for_constraint(
    constraint_type: str,
    rng,
) -> str:
    """Pick which obstacle TYPE realizes a sampled constraint.

    If the constraint appears in CONSTRAINT_TO_OBSTACLE_OPTIONS, sample
    from those options by weight. Otherwise fall back to the single 1-to-1
    mapping in GEOMETRY_TO_CONSTRAINTS.
    """
    if constraint_type in CONSTRAINT_TO_OBSTACLE_OPTIONS:
        opts = CONSTRAINT_TO_OBSTACLE_OPTIONS[constraint_type]
        keys = list(opts.keys())
        weights = [opts[k] for k in keys]
        total = sum(weights)
        r = rng.uniform(0.0, total)
        cumul = 0.0
        for k, w in zip(keys, weights):
            cumul += w
            if r <= cumul:
                return k
        return keys[-1]
    return constraint_to_obstacle_type(constraint_type)
