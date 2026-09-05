"""
CorridorEnv container + the main `generate_corridor()` entry point.

Build sequence (per env):
  1. Sample corridor length & width from tier-agnostic ranges (or override).
  2. Sample n_obstacles from the tier's range.
  3. Sample n_obstacles constraint types from the tier's mixture.
  4. Enforce tier ANCHORS: replace random slots with required constraints so
     every env satisfies the tier's guarantee.
  5. For each sampled constraint, build the corresponding obstacle with
     placement retries (x-spacing, AABB overlap, start/goal clearance).
  6. Sort obstacles by x, assign per-obstacle d_i (already done by builder),
     compute D_env via difficulty.compute_env_difficulty().
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from functools import partial
from typing import Literal, Optional

import numpy as np

from . import obstacle as _obstacle_mod
from .constraint_rules import (
    MAX_CORRIDOR_LENGTH_M,
    TIER_ANCHORS,
    TIER_CONSTRAINT_MIXTURE,
    TIER_LENGTH_RANGE,
    TIER_OBSTACLE_COUNT_RANGE,
    sample_obstacle_type_for_constraint,
)
from .difficulty import compute_env_difficulty
from .obstacle import ObstacleInstance
from .placement import (
    MIN_SLOT_WIDTH,
    START_GOAL_CLEARANCE,
    PlacementError,
    compute_x_slots,
    place_in_slot,
)

# --- Corridor body parameter ranges -----------------------------------------
# Length is sampled per-tier (see TIER_LENGTH_RANGE in constraint_rules.py)
# to maintain >= 1.0m obstacle spacing under the L <= 8.0m hard rule.
CORRIDOR_WIDTH_RANGE = (1.8, 2.5)
WALL_HEIGHT_M = 2.5

# Geometry-only leaning-columns injection (see obstacle.make_leaning_columns).
# Not a scored constraint -- a sentinel added to the constraint plan so the
# placement loop builds one extra tilted-column obstacle, when the corridor has
# room for an additional slot. Per-tier probability.
_LEANING_TOKEN = "__leaning_geometry__"
LEANING_INJECT_PROB = {"easy": 0.15, "medium": 0.30, "hard": 0.40}

Tier = Literal["easy", "medium", "hard"]


@dataclass
class CorridorEnv:
    env_id: str
    seed: int
    tier: str
    length: float
    width: float
    wall_height: float
    start_pos: np.ndarray
    goal_pos: np.ndarray
    obstacles: list[ObstacleInstance]

    env_difficulty: float  # max(d_i) until alpha/beta calibrated
    constraint_counts: dict[str, int]  # quick filter index

    optimal_traversal_time: Optional[float] = None  # filled later from human demos

    # ---- convenience views -------------------------------------------------
    def __post_init__(self) -> None:
        # Ensure numpy arrays even if reconstructed from YAML
        self.start_pos = np.asarray(self.start_pos, dtype=np.float64)
        self.goal_pos = np.asarray(self.goal_pos, dtype=np.float64)


# ---------------------------------------------------------------------------
# Public entry point
# ---------------------------------------------------------------------------


def generate_corridor(
    seed: int,
    tier: Tier,
    length: Optional[float] = None,
    width: Optional[float] = None,
) -> CorridorEnv:
    """Generate a single corridor environment.

    Args:
        seed: PRNG seed. Same seed + tier + length/width override =>
            byte-identical CorridorEnv (modulo np vs list serialization).
        tier: "easy" | "medium" | "hard". Picks obstacle count range,
            constraint mixture, and required anchors.
        length: Override corridor length (meters). None => sample from
            CORRIDOR_LENGTH_RANGE.
        width: Override corridor width. None => sample from CORRIDOR_WIDTH_RANGE.

    Returns:
        CorridorEnv with placed obstacles and computed D_env.
    """
    if tier not in TIER_CONSTRAINT_MIXTURE:
        raise ValueError(f"Unknown tier {tier!r}; expected easy|medium|hard")

    rng = random.Random(seed)

    # Corridor body. Length range is per-tier so we can always satisfy
    # >= 1.0m obstacle spacing under the L <= 8m hard rule.
    L_lo, L_hi = TIER_LENGTH_RANGE[tier]
    L = float(length) if length is not None else rng.uniform(L_lo, L_hi)
    if L > MAX_CORRIDOR_LENGTH_M + 1e-6:
        raise ValueError(f"Corridor length {L:.3f} exceeds hard cap {MAX_CORRIDOR_LENGTH_M} m.")
    W = float(width) if width is not None else rng.uniform(*CORRIDOR_WIDTH_RANGE)
    H = WALL_HEIGHT_M

    start_pos = np.array([0.5, W / 2.0, 0.0], dtype=np.float64)
    goal_pos = np.array([L - 0.5, W / 2.0, 0.0], dtype=np.float64)

    # Obstacle count + constraint sampling
    n_lo, n_hi = TIER_OBSTACLE_COUNT_RANGE[tier]
    n_obstacles = rng.randint(n_lo, n_hi)

    constraint_plan = _sample_constraint_plan(rng, tier, n_obstacles)

    # Optionally inject one geometry-only leaning-columns obstacle, but only
    # when an extra slot still fits at >= MIN_SLOT_WIDTH (keeps placement safe).
    avail = (L - 0.5 - START_GOAL_CLEARANCE) - (0.5 + START_GOAL_CLEARANCE)
    if rng.random() < LEANING_INJECT_PROB.get(tier, 0.0) and (
        avail / (len(constraint_plan) + 1) >= MIN_SLOT_WIDTH
    ):
        constraint_plan = list(constraint_plan) + [_LEANING_TOKEN]

    # DESIGN-NOTE: Slot-based x placement with Dirichlet-sampled slot widths.
    # Each slot is >= MIN_SLOT_WIDTH (0.70m). The Dirichlet partition gives
    # some narrow slots (tight pairs) and some wide slots (lots of jitter
    # headroom), so obstacle x positions don't sit on a uniform grid. See
    # placement.py module docstring.
    slots = compute_x_slots(L, len(constraint_plan), rng=rng)

    placed: list[ObstacleInstance] = []
    for ctype, (slot_lo, slot_hi) in zip(constraint_plan, slots):
        # Some constraints can be realized by multiple obstacle types
        # (e.g. lateral_passage -> lateral_block OR block_with_clutter).
        # The selector samples among the candidates by declared weights.
        if ctype == _LEANING_TOKEN:
            obs_type = "leaning_columns"
        else:
            obs_type = sample_obstacle_type_for_constraint(ctype, rng)
        builder_fn = _obstacle_mod.BUILDER_BY_OBSTACLE_TYPE[obs_type]
        if obs_type in ("dog_hole", "wall_squeeze", "leaning_columns"):
            local_builder = partial(builder_fn, corridor_width=W, wall_height=H)
        else:
            local_builder = partial(builder_fn, corridor_width=W)

        def _build(_rng, _x, _bldr=local_builder):
            return _bldr(_rng, _x)

        placed.append(
            place_in_slot(
                rng,
                _build,
                slot_lo,
                slot_hi,
                corridor_length=L,
                existing=placed,
            )
        )

    placed.sort(key=lambda o: float(o.position[0]))

    # D_env
    d_list = [o.difficulty for o in placed]
    D_env = compute_env_difficulty(d_list)  # alpha/beta=None => max(d_i)

    # Constraint counts for quick filtering
    counts: dict[str, int] = {}
    for o in placed:
        for c in o.constraints:
            counts[c] = counts.get(c, 0) + 1

    env_id = f"corridor_{tier}_{seed:08d}"

    return CorridorEnv(
        env_id=env_id,
        seed=int(seed),
        tier=tier,
        length=float(L),
        width=float(W),
        wall_height=float(H),
        start_pos=start_pos,
        goal_pos=goal_pos,
        obstacles=placed,
        env_difficulty=float(D_env),
        constraint_counts=counts,
    )


# ---------------------------------------------------------------------------
# Constraint plan: anchored, then padded with mixture samples.
# ---------------------------------------------------------------------------


def _sample_constraint_plan(
    rng: random.Random,
    tier: Tier,
    n_obstacles: int,
) -> list[str]:
    """Return a list of n_obstacles constraint type strings.

    Steps:
      1. Add tier anchors (`required` always present, `one_of` picks exactly
         one if neither already chosen).
      2. Fill remaining slots by sampling from the tier mixture.
      3. Shuffle so the anchor isn't always the first obstacle.
    """
    mixture = TIER_CONSTRAINT_MIXTURE[tier]
    anchors = TIER_ANCHORS[tier]

    plan: list[str] = list(anchors["required"])

    one_of = anchors.get("one_of", ())
    if one_of:
        plan.append(rng.choice(one_of))

    if len(plan) > n_obstacles:
        # Edge case: anchor count exceeds obstacle budget. Trim by keeping
        # required ones first.
        plan = plan[:n_obstacles]

    # Fill the rest from the mixture, sampling proportional to declared probs.
    keys = list(mixture.keys())
    weights = [mixture[k] for k in keys]
    while len(plan) < n_obstacles:
        plan.append(_weighted_choice(rng, keys, weights))

    rng.shuffle(plan)
    return plan


def _weighted_choice(
    rng: random.Random,
    keys: list[str],
    weights: list[float],
) -> str:
    """Discrete sample over `keys` with `weights` (need not sum to exactly 1)."""
    total = sum(weights)
    if total <= 0.0:
        raise ValueError("Constraint mixture weights sum to zero or negative.")
    r = rng.uniform(0.0, total)
    cumul = 0.0
    for k, w in zip(keys, weights):
        cumul += w
        if r <= cumul:
            return k
    return keys[-1]
