"""MTC Challenge v1 — corridor environment generator.

Public API:
    generate_corridor(seed, tier, length=None, width=None) -> CorridorEnv
    export_to_yaml / load_from_yaml / export_to_mjcf
    render_top_down / render_side_view / render_3d
"""

from .constraint_rules import (
    CONSTRAINT_COLORS,
    CONSTRAINT_PARAM_RANGES,
    CONSTRAINT_TYPES,
    TIER_CONSTRAINT_MIXTURE,
    TIER_OBSTACLE_COUNT_RANGE,
    compute_constraint_difficulty,
    constraints_for_geometry,
)
from .corridor import CorridorEnv, generate_corridor
from .difficulty import compute_env_difficulty
from .export import export_to_mjcf, export_to_yaml, load_from_yaml
from .obstacle import ObstacleInstance

__version__ = "0.1.0"

__all__ = [
    "CONSTRAINT_TYPES",
    "CONSTRAINT_PARAM_RANGES",
    "CONSTRAINT_COLORS",
    "TIER_CONSTRAINT_MIXTURE",
    "TIER_OBSTACLE_COUNT_RANGE",
    "ObstacleInstance",
    "CorridorEnv",
    "generate_corridor",
    "export_to_yaml",
    "load_from_yaml",
    "export_to_mjcf",
    "compute_constraint_difficulty",
    "constraints_for_geometry",
    "compute_env_difficulty",
]
