"""
Serialization: YAML (lossless) and MJCF (geometry-only minimal).

YAML schema version: v0.1.

This module is filled out in step 6 of the build plan. Phase-1 stub exposes
the three public functions so __init__.py can import them safely.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
import yaml

from .constraint_rules import GOAL_RADIUS_M
from .obstacle import ObstacleInstance

if TYPE_CHECKING:
    from .corridor import CorridorEnv


YAML_SCHEMA_VERSION = "0.1"


# ---------------------------------------------------------------------------
# YAML
# ---------------------------------------------------------------------------


def export_to_yaml(env: "CorridorEnv", path) -> None:
    """Lossless dump of a CorridorEnv to YAML."""
    path = Path(path)
    data = {
        "schema_version": YAML_SCHEMA_VERSION,
        "env_id": env.env_id,
        "seed": env.seed,
        "tier": env.tier,
        "length": env.length,
        "width": env.width,
        "wall_height": env.wall_height,
        "start_pos": _arr(env.start_pos),
        "goal_pos": _arr(env.goal_pos),
        "env_difficulty": env.env_difficulty,
        "constraint_counts": dict(env.constraint_counts),
        "optimal_traversal_time": env.optimal_traversal_time,
        "obstacles": [_obstacle_to_dict(o) for o in env.obstacles],
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def load_from_yaml(path) -> "CorridorEnv":
    """Inverse of export_to_yaml. Raises on schema_version mismatch."""
    # Local import avoids circular at module-load time.
    from .corridor import CorridorEnv

    path = Path(path)
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data.get("schema_version") != YAML_SCHEMA_VERSION:
        raise ValueError(
            f"YAML schema version {data.get('schema_version')!r} does not "
            f"match expected {YAML_SCHEMA_VERSION!r}"
        )
    obstacles = [_obstacle_from_dict(o) for o in data["obstacles"]]
    return CorridorEnv(
        env_id=data["env_id"],
        seed=int(data["seed"]),
        tier=data["tier"],
        length=float(data["length"]),
        width=float(data["width"]),
        wall_height=float(data["wall_height"]),
        start_pos=np.asarray(data["start_pos"], dtype=np.float64),
        goal_pos=np.asarray(data["goal_pos"], dtype=np.float64),
        obstacles=obstacles,
        env_difficulty=float(data["env_difficulty"]),
        constraint_counts=dict(data["constraint_counts"]),
        optimal_traversal_time=data.get("optimal_traversal_time"),
    )


# ---------------------------------------------------------------------------
# MJCF (minimal, v1 stub)
# ---------------------------------------------------------------------------


def export_to_mjcf(env: "CorridorEnv", path) -> None:
    """Minimal MJCF: floor + 2 walls + every sub-AABB as a `<geom type="box">`.

    Objaverse placeholders also become boxes. The output is hand-checked to
    load in `mujoco.MjModel.from_xml_path` but is not optimized for simulation
    (no contacts beyond default, no actuators)."""
    path = Path(path)

    lines: list[str] = []
    lines.append('<mujoco model="mtc_corridor">')
    lines.append('  <compiler angle="degree"/>')
    lines.append('  <option timestep="0.005"/>')
    lines.append("  <worldbody>")

    # Floor
    lines.append(
        f'    <geom name="floor" type="plane" size="{env.length / 2:.4f} '
        f'{env.width / 2:.4f} 0.1" pos="{env.length / 2:.4f} {env.width / 2:.4f} 0" '
        f'rgba="0.85 0.85 0.85 1"/>'
    )
    # Two side walls (full corridor length, wall_height tall, 0.05 thick)
    wt = 0.05
    h = env.wall_height
    lines.append(
        f'    <geom name="wall_y0" type="box" '
        f'size="{env.length / 2:.4f} {wt / 2:.4f} {h / 2:.4f}" '
        f'pos="{env.length / 2:.4f} {-wt / 2:.4f} {h / 2:.4f}" '
        f'rgba="0.8 0.8 0.8 1"/>'
    )
    lines.append(
        f'    <geom name="wall_y1" type="box" '
        f'size="{env.length / 2:.4f} {wt / 2:.4f} {h / 2:.4f}" '
        f'pos="{env.length / 2:.4f} {env.width + wt / 2:.4f} {h / 2:.4f}" '
        f'rgba="0.8 0.8 0.8 1"/>'
    )

    # Obstacles — every sub-AABB becomes a colored box.
    for o in env.obstacles:
        color = _obstacle_color_rgba(o)
        for idx, (lo, hi) in enumerate(o.aabbs()):
            cx = (lo[0] + hi[0]) / 2.0
            cy = (lo[1] + hi[1]) / 2.0
            cz = (lo[2] + hi[2]) / 2.0
            sx = (hi[0] - lo[0]) / 2.0
            sy = (hi[1] - lo[1]) / 2.0
            sz = (hi[2] - lo[2]) / 2.0
            lines.append(
                f'    <geom name="{o.obstacle_id}_{idx}" type="box" '
                f'size="{sx:.4f} {sy:.4f} {sz:.4f}" '
                f'pos="{cx:.4f} {cy:.4f} {cz:.4f}" rgba="{color}"/>'
            )

    # Start / goal sites (no collision, visualization only).
    s = env.start_pos
    g = env.goal_pos
    lines.append(
        f'    <site name="start" pos="{s[0]:.4f} {s[1]:.4f} 0.05" '
        f'size="0.10" rgba="0.1 0.7 0.8 1"/>'
    )
    # Goal disc is drawn at GOAL_RADIUS_M so the MuJoCo view, the FBX ring the
    # VR subject sees, and the success test all show the same circle.
    lines.append(
        f'    <site name="goal" type="cylinder" pos="{g[0]:.4f} {g[1]:.4f} 0.01" '
        f'size="{GOAL_RADIUS_M:.4f} 0.005" rgba="1.0 0.85 0.2 0.45"/>'
    )

    lines.append("  </worldbody>")
    lines.append("</mujoco>")

    path.write_text("\n".join(lines), encoding="utf-8")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _arr(x) -> list:
    return np.asarray(x, dtype=np.float64).tolist()


def _obstacle_to_dict(o: ObstacleInstance) -> dict:
    return {
        "obstacle_id": o.obstacle_id,
        "obstacle_type": o.obstacle_type,
        "asset_source": o.asset_source,
        "position": _arr(o.position),
        "orientation": _arr(o.orientation),
        "geometric_params": _jsonify_params(o.geometric_params),
        "constraints": list(o.constraints),
        "constraint_params": dict(o.constraint_params),
        "difficulty": float(o.difficulty),
        "human_demonstrated_intents": list(o.human_demonstrated_intents),
    }


def _obstacle_from_dict(d: dict) -> ObstacleInstance:
    return ObstacleInstance(
        obstacle_id=d["obstacle_id"],
        obstacle_type=d["obstacle_type"],
        asset_source=d["asset_source"],
        position=np.asarray(d["position"], dtype=np.float64),
        orientation=np.asarray(d["orientation"], dtype=np.float64),
        geometric_params=dict(d["geometric_params"]),
        constraints=list(d["constraints"]),
        constraint_params=dict(d["constraint_params"]),
        difficulty=float(d["difficulty"]),
        human_demonstrated_intents=list(d.get("human_demonstrated_intents", [])),
    )


def _jsonify_params(p: dict) -> dict:
    """Convert numpy values inside geometric_params to plain lists/floats for YAML."""
    out: dict = {}
    for k, v in p.items():
        if isinstance(v, np.ndarray):
            out[k] = v.tolist()
        elif isinstance(v, (list, tuple)):
            out[k] = [float(x) if isinstance(x, (int, float, np.floating)) else x for x in v]
        elif isinstance(v, (np.floating, np.integer)):
            out[k] = float(v)
        else:
            out[k] = v
    return out


def _obstacle_color_rgba(o: ObstacleInstance) -> str:
    """Hex CONSTRAINT_COLORS -> 'r g b 1.0' string for MJCF rgba attr."""
    from .constraint_rules import CONSTRAINT_COLORS

    if not o.constraints:
        return "0.5 0.5 0.5 1.0"
    hexstr = CONSTRAINT_COLORS[o.constraints[0]].lstrip("#")
    r = int(hexstr[0:2], 16) / 255.0
    g = int(hexstr[2:4], 16) / 255.0
    b = int(hexstr[4:6], 16) / 255.0
    return f"{r:.3f} {g:.3f} {b:.3f} 1.0"
