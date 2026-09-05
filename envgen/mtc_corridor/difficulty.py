"""
Environment-level difficulty aggregation.

The exact formula combining per-obstacle d_i values into a single D_env
is intentionally UNSET in v1. It will be calibrated post-data-collection by
maximizing correlation between D_env and human completion time (see paper
section: "Calibrated difficulty function").

When `alpha` and `beta` are both None (default), D_env := max(d_i). This
matches the spec's documented v1 fallback.
"""

from __future__ import annotations

from typing import Optional


def compute_env_difficulty(
    per_obstacle_difficulties: list[float],
    constraint_transitions: Optional[list[tuple[set[str], set[str], float]]] = None,
    alpha: Optional[float] = None,
    beta: Optional[float] = None,
) -> float:
    """Aggregate per-obstacle difficulties into a single D_env.

    Args:
        per_obstacle_difficulties: d_i for each obstacle (in any order).
        constraint_transitions: optional list of
            (prev_constraints, next_constraints, x_spacing) tuples for adjacent
            obstacle pairs. Reserved for the transition_penalty term.
        alpha: weight on sum(d_i) term. UNSET in v1.
        beta: weight on transition penalty. UNSET in v1.

    Returns:
        D_env in [0, 1] when alpha/beta are None; unbounded once they are set.
    """
    if not per_obstacle_difficulties:
        return 0.0

    max_d = max(per_obstacle_difficulties)

    if alpha is None and beta is None:
        # v1 default — see module docstring.
        return float(max_d)

    sum_d = sum(per_obstacle_difficulties)
    penalty = _transition_penalty(constraint_transitions or [])
    return float(max_d + (alpha or 0.0) * sum_d + (beta or 0.0) * penalty)


def _transition_penalty(
    transitions: list[tuple[set[str], set[str], float]],
) -> float:
    """Placeholder. The intent is: closely-spaced obstacles that demand
    DIFFERENT constraint regimes (e.g. duck immediately followed by side-turn)
    add penalty because the robot must re-plan whole-body posture quickly.

    v1 returns 0.0. Future versions will reference this function and
    test_difficulty_monotonicity may extend to assert this term moves D_env
    upward when transitions are added with constant per-obstacle d_i.
    """
    return 0.0
