"""Export all generated corridor envs to Unity-ready FBX files.

Pipeline (per env):
    YAML  ->  flat intermediate JSON  ->  Blender headless  ->  FBX

The output FBX uses Y-up Unity-native orientation. The whole corridor is
SHIFTED in x and y so the START position lands at FBX local (0, 0, 0).
This way, when Unity places the FBX as a child of WorldOrigin (identity
transform), the player spawning at WorldOrigin appears at the corridor
entrance.

Markers (start/goal cones) are intentionally NOT exported -- per user
request, the spawn position is implied by the corridor's origin alignment.

Usage:
    python scripts/export_to_unity.py            # build + copy to Unity
    python scripts/export_to_unity.py --no-copy  # build only
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from mtc_corridor import export_to_yaml, generate_corridor  # noqa: E402
from mtc_corridor.constraint_rules import GOAL_RADIUS_M  # noqa: E402
from mtc_corridor.corridor import CorridorEnv  # noqa: E402
from mtc_corridor.obstacle import ObstacleInstance  # noqa: E402


def _find_blender() -> Path:
    """Resolve Blender: MTC_BLENDER_EXE env var > PATH > platform defaults."""
    env = os.environ.get("MTC_BLENDER_EXE")
    if env:
        return Path(env)
    on_path = shutil.which("blender")
    if on_path:
        return Path(on_path)
    for candidate in (
        Path("/Applications/Blender.app/Contents/MacOS/Blender"),
        Path(r"C:\Program Files\Blender Foundation\Blender 4.5\blender.exe"),
        Path("/usr/bin/blender"),
    ):
        if candidate.exists():
            return candidate
    return Path("blender")  # last resort; .exists() checks downstream will report


BLENDER_EXE = _find_blender()
FBX_BUILDER = Path(__file__).resolve().parent / "fbx_builder.py"
UNITY_FBX_DIR = Path(
    os.environ.get("MTC_UNITY_FBX_DIR") or (PROJECT_ROOT / "fbx_out")
)
OUT_FBX_DIR = Path(os.environ.get("MTC_ENV_OUT_DIR", PROJECT_ROOT / "viz" / "v1_examples"))

SEEDS = {
    "easy": [101, 102, 103],
    "medium": [201, 202, 203],
    "hard": [301, 302, 303],
}

WALL_BLENDER_THICK = 0.05
FLOOR_THICK = 0.05


# ---------------------------------------------------------------------------
# YAML -> intermediate JSON conversion
# ---------------------------------------------------------------------------


def env_to_blender_scene(env: CorridorEnv) -> dict:
    """Flatten env into {boxes, elements} for fbx_builder.

    Origin shift: subtract (start_x, start_y, 0) from every position so the
    corridor entrance ends up at local origin.
    """
    L, W, H = env.length, env.width, env.wall_height
    # Shift everything so the FBX local origin sits at the corridor START.
    dx = -float(env.start_pos[0])
    dy = -float(env.start_pos[1])

    def shift(p):
        return [float(p[0]) + dx, float(p[1]) + dy, float(p[2])]

    boxes: list[dict] = []

    # Floor (corridor world): center is at (L/2, W/2). After shift => (L/2+dx, W/2+dy).
    boxes.append(
        {
            "name": "floor",
            "class": "floor",
            "mat_key": "floor",
            "size": [L, W, FLOOR_THICK],
            "position": shift([L / 2.0, W / 2.0, -FLOOR_THICK / 2.0]),
        }
    )

    wt = WALL_BLENDER_THICK
    boxes.append(
        {
            "name": "wall_y0",
            "class": "wall",
            "mat_key": "wall",
            "size": [L, wt, H],
            "position": shift([L / 2.0, -wt / 2.0, H / 2.0]),
        }
    )
    boxes.append(
        {
            "name": "wall_y1",
            "class": "wall",
            "mat_key": "wall",
            "size": [L, wt, H],
            "position": shift([L / 2.0, W + wt / 2.0, H / 2.0]),
        }
    )

    elements: list[dict] = []
    for obs in env.obstacles:
        elements.extend(_obstacle_to_elements(obs, dx, dy))

    goal_local = shift([float(env.goal_pos[0]), float(env.goal_pos[1]), 0.0])
    elements.append(
        {
            "name": "goal_ring",
            "class": "goal",
            "primitive": "goal_ring",
            "material": "goal",
            "radius": GOAL_RADIUS_M,
            "position": goal_local,
            "rotation_deg": [0.0, 0.0, 0.0],
        }
    )

    # Bake a -90deg rotation around the up axis (Z) into every position and
    # every yaw, so the corridor's long axis (originally source +X) ends up on
    # Unity +Z (player forward) after FBX export. Result: Unity placement at
    # identity (0,0,0) shows the corridor extending away from the player.
    # Empirically, +90 here sends the corridor to -Z (behind the player) --
    # Blender +Y maps to Unity -Z through the FBX axis_forward="-Z" export,
    # so we go the other way.
    def _rotm90z(p):
        # -90 deg around Z (RHR): (x, y, z) -> (y, -x, z)
        return [float(p[1]), -float(p[0]), float(p[2])]

    for b in boxes:
        b["position"] = _rotm90z(b["position"])
        b["rotation_deg"] = [0.0, 0.0, -90.0]
    for el in elements:
        el["position"] = _rotm90z(el["position"])
        # Route the -90deg spin to the field the builder actually reads: USD
        # assets use yaw_deg (import_usd_asset), every other primitive (box /
        # cylinder / ibeam / goal_ring) uses rotation_deg (make_box etc. IGNORE
        # yaw_deg -- boxes given only yaw_deg were silently left un-rotated, so a
        # wall meant to span the width came out as a thin pillar along travel).
        if el.get("primitive") == "asset":
            el["yaw_deg"] = float(el.get("yaw_deg", 0.0)) - 90.0
        else:
            rd = el.get("rotation_deg", [0.0, 0.0, 0.0])
            el["rotation_deg"] = [float(rd[0]), float(rd[1]), float(rd[2]) - 90.0]

    goal_el = next(e for e in elements if e["name"] == "goal_ring")

    return {
        "scene_name": env.env_id,
        "corridor": {"length": L, "width": W, "wall_height": H},
        "origin_shift": [dx, dy, 0.0],
        # Post-rotation, so this is the goal in the same frame as the FBX.
        "goal": {"position": list(goal_el["position"]), "radius": GOAL_RADIUS_M},
        "boxes": boxes,
        "elements": elements,
    }


def _shift_position(p, dx, dy):
    return [float(p[0]) + dx, float(p[1]) + dy, float(p[2])]


def _obstacle_to_elements(obs: ObstacleInstance, dx: float, dy: float) -> list[dict]:
    """Emit one or more `element` dicts (box/cylinder/ibeam/asset)."""
    t = obs.obstacle_type
    gp = obs.geometric_params
    out: list[dict] = []
    base = obs.obstacle_id

    if t in ("beam", "leaning_columns"):
        for i, el in enumerate(gp["elements"]):
            shifted = copy.deepcopy(el)
            shifted["position"] = _shift_position(el["position"], dx, dy)
            shifted.setdefault("name", f"{base}_{i}")
            shifted.setdefault("material", "default")
            out.append(shifted)

    elif t == "dog_hole":
        # Render the 3 sub-walls directly as boxes (matches AABB layout).
        thickness_x = gp["wall_thickness_x"]
        opening_top = gp["opening_top_z"]
        opening_w_y = gp["opening_width_y"]
        wall_h_z = gp["wall_height_z"]
        cx_w = float(obs.position[0])
        cy_center_w = float(obs.position[1])
        cw = float(gp.get("corridor_width", cy_center_w * 2.0))
        opening_lo_y = cy_center_w - opening_w_y / 2.0
        opening_hi_y = cy_center_w + opening_w_y / 2.0

        # left stub
        if opening_lo_y > 0:
            sy = opening_lo_y
            out.append(
                {
                    "primitive": "box",
                    "name": f"{base}_L",
                    "size_xyz": [thickness_x, sy, wall_h_z],
                    "position": _shift_position([cx_w, sy / 2.0, wall_h_z / 2.0], dx, dy),
                    "material": gp.get("material", "concrete"),
                }
            )
        # right stub
        if opening_hi_y < cw:
            sy = cw - opening_hi_y
            out.append(
                {
                    "primitive": "box",
                    "name": f"{base}_R",
                    "size_xyz": [thickness_x, sy, wall_h_z],
                    "position": _shift_position(
                        [cx_w, opening_hi_y + sy / 2.0, wall_h_z / 2.0], dx, dy
                    ),
                    "material": gp.get("material", "concrete"),
                }
            )
        # top lintel
        lintel_h = wall_h_z - opening_top
        out.append(
            {
                "primitive": "box",
                "name": f"{base}_T",
                "size_xyz": [thickness_x, opening_w_y, lintel_h],
                "position": _shift_position(
                    [cx_w, cy_center_w, opening_top + lintel_h / 2.0], dx, dy
                ),
                "material": gp.get("material", "concrete"),
            }
        )

    elif t == "ground":
        for i, item in enumerate(gp["items"]):
            shifted = copy.deepcopy(item)
            shifted["position"] = _shift_position(item["position"], dx, dy)
            shifted["name"] = f"{base}_{i}_{item.get('asset_class', 'asset')}"
            shifted.setdefault("material", "default")
            out.append(shifted)

    elif t == "lateral_block":
        if "walls" in gp:  # half_wall: one box per protrusion
            for w in gp["walls"]:
                out.append(
                    {
                        "primitive": "box",
                        "name": f"{base}_{w['side']}",
                        "size_xyz": list(w["size_xyz"]),
                        "position": _shift_position(w["position"], dx, dy),
                        "material": gp.get("material", "concrete"),
                    }
                )
        else:
            out.append(
                {
                    "primitive": "box",
                    "name": base,
                    "size_xyz": gp["size_xyz"],
                    "position": _shift_position(obs.position, dx, dy),
                    "material": gp.get("material", "concrete"),
                }
            )

    elif t == "wall_squeeze":
        cx_w = float(obs.position[0])
        cy_center_w = float(obs.position[1])
        cw = float(gp.get("corridor_width", cy_center_w * 2.0))
        wall_h = gp["wall_height"]
        left = gp["left_size_xyz"]
        right = gp["right_size_xyz"]
        out.append(
            {
                "primitive": "box",
                "name": f"{base}_L",
                "size_xyz": left,
                "position": _shift_position([cx_w, left[1] / 2.0, wall_h / 2.0], dx, dy),
                "material": gp.get("material", "concrete"),
            }
        )
        out.append(
            {
                "primitive": "box",
                "name": f"{base}_R",
                "size_xyz": right,
                "position": _shift_position([cx_w, cw - right[1] / 2.0, wall_h / 2.0], dx, dy),
                "material": gp.get("material", "concrete"),
            }
        )

    elif t == "block_with_clutter":
        # Block
        out.append(
            {
                "primitive": "box",
                "name": f"{base}_block",
                "size_xyz": gp["block_size_xyz"],
                "position": _shift_position(gp["block_center_xyz"], dx, dy),
                "material": "concrete",
            }
        )
        for i, item in enumerate(gp["clutter_items"]):
            shifted = copy.deepcopy(item)
            shifted["position"] = _shift_position(item["position"], dx, dy)
            shifted["name"] = f"{base}_clut{i}_{item.get('asset_class', 'asset')}"
            shifted.setdefault("material", "default")
            out.append(shifted)

    else:
        print(f"  WARNING: no element converter for obstacle_type {t!r}")

    return out


# ---------------------------------------------------------------------------
# Per-env conversion
# ---------------------------------------------------------------------------


def build_one(env: CorridorEnv, out_dir: Path) -> Path:
    scene_json = env_to_blender_scene(env)
    fbx_path = out_dir / f"{env.env_id}_simplified.fbx"
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False, encoding="utf-8") as tf:
        json.dump(scene_json, tf, indent=2)
        json_path = Path(tf.name)
    try:
        cmd = [
            str(BLENDER_EXE),
            "--background",
            "--python",
            str(FBX_BUILDER),
            "--",
            str(json_path),
            str(fbx_path),
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(result.stdout)
            print(result.stderr, file=sys.stderr)
            raise RuntimeError(f"Blender failed for {env.env_id}")
        for ln in result.stdout.splitlines():
            if "[fbx_builder]" in ln or "Exported FBX" in ln or "WARNING" in ln:
                print(ln)
    finally:
        json_path.unlink(missing_ok=True)
    return fbx_path


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-copy", action="store_true")
    ap.add_argument("--unity-dir", type=Path, default=UNITY_FBX_DIR)
    args = ap.parse_args()

    if not BLENDER_EXE.exists():
        sys.exit(f"Blender not found at {BLENDER_EXE}")

    OUT_FBX_DIR.mkdir(parents=True, exist_ok=True)

    built: list[Path] = []
    for tier, seeds in SEEDS.items():
        for seed in seeds:
            env = generate_corridor(seed=seed, tier=tier)
            export_to_yaml(env, OUT_FBX_DIR / f"{env.env_id}.yaml")
            fbx = build_one(env, OUT_FBX_DIR)
            built.append(fbx)

    print(f"\nBuilt {len(built)} FBX in {OUT_FBX_DIR}")
    for p in built:
        print(f"  - {p.name}")

    if args.no_copy:
        print("\n--no-copy: skipping Unity copy.")
        return
    if not args.unity_dir.exists():
        print(f"\nWARNING: Unity FBX dir {args.unity_dir} not found.")
        return
    print(f"\nCopying to Unity: {args.unity_dir}")
    for src in built:
        dst = args.unity_dir / src.name
        shutil.copy2(src, dst)
        print(f"  -> {dst.name}")


if __name__ == "__main__":
    main()
