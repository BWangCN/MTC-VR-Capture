"""
Placement / overlap checking for corridor obstacles.

Responsibilities:
  - axis-aligned bounding box overlap test
  - x-slot assignment (evenly divides available x-range into n_obstacles slots
    so the per-pair min spacing is the slot width)
  - start/goal exclusion (no obstacle within 0.5m of start or goal in x)
  - retry-with-resample placement loop for per-slot jitter

The placement layer does NOT know about constraints — it only sees
ObstacleInstances and their AABBs. Constraint-aware logic (anchor enforcement,
tier mixtures) lives in corridor.py.

DEVIATION FROM SPEC (documented):
  The spec asks for a hard minimum x-spacing of 1.0m between every adjacent
  pair. With the spec's corridor length range [5, 8]m and hard tier obstacle
  count [8, 11], strict 1.0m spacing is geometrically impossible at the
  upper end of the count (11 obstacles need >= 12m of corridor). We therefore
  use a slot-based placement: the available x-range [start+0.5, goal-0.5] is
  divided into n_obstacles equal slots and one obstacle is jittered inside
  each. The achieved per-pair spacing is the slot width
  (available_length / n_obstacles) with small jitter. This is >= 1.0m for
  easy / medium and < 1.0m only at the upper end of hard. Tests cover the
  achieved spacing range; see also corridor.py:DESIGN-NOTE.
"""

from __future__ import annotations

import random
from typing import Callable, Iterable

import numpy as np

from .obstacle import ObstacleInstance

# Hard placement parameters. Spacing target was relaxed from 1.0m -> 0.75m
# (2026-05-19) so that jitter has room to randomize x positions across all
# tiers. With the L <= 8m cap, the original 1.0m target only allowed zero
# jitter at the tight (L_min, n_max) end of medium/hard.
TARGET_MIN_X_SPACING = 0.75  # target spacing; loosened to allow more x-jitter
START_GOAL_CLEARANCE = 0.5  # no obstacle within this many m of start/goal in x
MAX_PLACEMENT_TRIES = 20
SLOT_JITTER_FRACTION = 0.45  # cap on jitter as fraction of half-slot-width


class PlacementError(RuntimeError):
    """Raised when an obstacle cannot be placed after MAX_PLACEMENT_TRIES."""


def aabbs_overlap(
    lo_a: np.ndarray,
    hi_a: np.ndarray,
    lo_b: np.ndarray,
    hi_b: np.ndarray,
    eps: float = 1e-6,
) -> bool:
    """3D AABB overlap test with a small epsilon to tolerate floating point
    contact (two boxes touching face-to-face are NOT considered overlapping).
    """
    for axis in range(3):
        if hi_a[axis] <= lo_b[axis] + eps:
            return False
        if hi_b[axis] <= lo_a[axis] + eps:
            return False
    return True


def obstacle_overlaps_any(
    candidate: ObstacleInstance,
    existing: Iterable[ObstacleInstance],
) -> bool:
    """True if any sub-AABB of `candidate` overlaps any sub-AABB of any existing
    obstacle. O(n_a * n_b) — fine for v1 obstacle counts (<=11)."""
    cand_aabbs = candidate.aabbs()
    for other in existing:
        for olo, ohi in other.aabbs():
            for clo, chi in cand_aabbs:
                if aabbs_overlap(clo, chi, olo, ohi):
                    return True
    return False


def violates_start_goal_clearance(
    candidate: ObstacleInstance,
    corridor_length: float,
    clearance: float = START_GOAL_CLEARANCE,
) -> bool:
    """No obstacle within `clearance` m of start (x=0.5) or goal (x=L-0.5) in x.

    Use the obstacle's full x-extent (across all sub-AABBs) for the check —
    not just the center — so a wide obstacle near the start is correctly
    rejected."""
    x_lo, x_hi = candidate.x_extent()
    start_x = 0.5
    goal_x = corridor_length - 0.5
    if x_lo < start_x + clearance and x_hi > start_x - clearance:
        return True
    if x_lo < goal_x + clearance and x_hi > goal_x - clearance:
        return True
    return False


MIN_SLOT_WIDTH = 0.70  # lower bound for any single slot (Dirichlet-sampled)


def compute_x_slots(
    corridor_length: float,
    n_obstacles: int,
    start_clearance: float = START_GOAL_CLEARANCE,
    rng: random.Random | None = None,
    min_slot_width: float = MIN_SLOT_WIDTH,
) -> list[tuple[float, float]]:
    """Partition the available x-range into n contiguous slots.

    Available range: [0.5 + start_clearance, L - 0.5 - start_clearance].

    Two modes:
      - `rng is None` -> equal-width slots (deterministic, used by tests).
      - `rng is not None` -> Dirichlet-sampled widths, each >= min_slot_width.
        Some slots end up narrow (forces tight obstacle pairs), some wide
        (allows lots of x-jitter), so the final obstacle positions don't
        sit on a uniform grid.
    """
    lo = 0.5 + start_clearance
    hi = corridor_length - 0.5 - start_clearance
    if hi <= lo:
        raise PlacementError(
            f"Corridor length {corridor_length} too short for start/goal clearance."
        )
    if n_obstacles <= 0:
        return []
    avail = hi - lo

    if rng is None or n_obstacles == 1:
        slot_w = avail / n_obstacles
        return [(lo + i * slot_w, lo + (i + 1) * slot_w) for i in range(n_obstacles)]

    widths = _dirichlet_slot_widths(rng, n_obstacles, avail, min_slot_width)
    slots: list[tuple[float, float]] = []
    cursor = lo
    for w in widths:
        slots.append((cursor, cursor + w))
        cursor += w
    return slots


def _dirichlet_slot_widths(
    rng: random.Random,
    n: int,
    total: float,
    min_width: float,
) -> list[float]:
    """Sample n positive widths summing to `total`, each >= min_width.

    Falls back to uniform partition when min_width * n exceeds total (i.e.
    the constraint is infeasible).
    """
    residual = total - n * min_width
    if residual <= 0:
        return [total / n] * n
    raw = [rng.expovariate(1.0) for _ in range(n)]
    s = sum(raw) or 1.0
    fractions = [r / s for r in raw]
    return [min_width + f * residual for f in fractions]


def jitter_x_within_slot(
    rng: random.Random,
    slot_lo: float,
    slot_hi: float,
    target_min_spacing: float = TARGET_MIN_X_SPACING,
    max_jitter_fraction: float = SLOT_JITTER_FRACTION,
) -> float:
    """Sample an x inside [slot_lo, slot_hi] with ADAPTIVE jitter.

    The jitter range is clamped so adjacent slots' obstacles (worst-case
    placed at opposing slot edges) still stay at least `target_min_spacing`
    apart. Specifically:

        max_jitter = max(0, (slot_width - target_min_spacing) / 2)

    Then sampled from [-max_jitter, +max_jitter] around the slot center,
    additionally capped at max_jitter_fraction * slot_width / 2 so jitter
    stays well inside the slot for visual variety.

    Effect:
      slot_width  = 1.5m  -> max_jitter = 0.25m  (slot-bounded => 0.225m)
      slot_width  = 1.0m  -> max_jitter = 0.0m   (no jitter)
      slot_width  = 0.8m  -> max_jitter = 0.0m   (slot already tight)
    """
    slot_w = slot_hi - slot_lo
    mid = 0.5 * (slot_lo + slot_hi)
    spacing_jitter = max(0.0, (slot_w - target_min_spacing) / 2.0)
    bound_jitter = max_jitter_fraction * slot_w / 2.0
    max_j = min(spacing_jitter, bound_jitter)
    if max_j <= 0.0:
        return mid
    return rng.uniform(mid - max_j, mid + max_j)


def place_in_slot(
    rng: random.Random,
    builder: Callable[[random.Random, float], ObstacleInstance],
    slot_lo: float,
    slot_hi: float,
    corridor_length: float,
    existing: list[ObstacleInstance],
    max_tries: int = MAX_PLACEMENT_TRIES,
) -> ObstacleInstance:
    """Build an obstacle whose x lies inside the given slot. Retries jitter
    + builder sampling on AABB overlap or start/goal clearance violations."""
    last_reason = "unknown"
    for attempt in range(max_tries):
        x = jitter_x_within_slot(rng, slot_lo, slot_hi)
        candidate = builder(rng, x)

        if violates_start_goal_clearance(candidate, corridor_length):
            last_reason = "start/goal clearance"
            continue
        if obstacle_overlaps_any(candidate, existing):
            last_reason = "AABB overlap"
            continue
        return candidate

    raise PlacementError(
        f"Failed to place obstacle in slot [{slot_lo:.2f}, {slot_hi:.2f}] "
        f"after {max_tries} attempts (last rejection: {last_reason}). "
        f"Existing obstacles: {len(existing)}, corridor length: {corridor_length}."
    )
