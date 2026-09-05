"""Build a Unity-ready FBX from an (edited) scene JSON.

Pair with export_scene_json.py. After you edit object positions in the
JSON, this rebuilds the FBX via Blender headless and (optionally) copies
it into the Unity project.

Usage:
    python scripts/build_from_json.py scenes_json/corridor_medium_00000211.json
    python scripts/build_from_json.py my_scene.json --to-unity
    python scripts/build_from_json.py my_scene.json --out custom.fbx --to-unity
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.export_to_unity import (  # noqa: E402
    BLENDER_EXE,
    FBX_BUILDER,
    OUT_FBX_DIR,
    UNITY_FBX_DIR,
)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("json", type=Path, help="scene JSON to build (possibly edited)")
    ap.add_argument(
        "--out",
        type=Path,
        default=None,
        help="output FBX path (default: viz/v1_examples/<scene_name>_simplified.fbx)",
    )
    ap.add_argument(
        "--to-unity", action="store_true", help="also copy the FBX into the Unity project"
    )
    ap.add_argument("--unity-dir", type=Path, default=UNITY_FBX_DIR)
    args = ap.parse_args()

    if not args.json.exists():
        sys.exit(f"scene JSON not found: {args.json}")
    if not BLENDER_EXE.exists():
        sys.exit(f"Blender not found at {BLENDER_EXE}")

    with open(args.json, "r", encoding="utf-8") as f:
        scene = json.load(f)
    scene_name = scene.get("scene_name", args.json.stem)

    fbx_path = args.out or (OUT_FBX_DIR / f"{scene_name}_simplified.fbx")
    fbx_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        str(BLENDER_EXE),
        "--background",
        "--python",
        str(FBX_BUILDER),
        "--",
        str(args.json.resolve()),
        str(fbx_path.resolve()),
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        sys.exit(f"Blender failed for {args.json}")
    for ln in result.stdout.splitlines():
        if "[fbx_builder]" in ln or "Exported FBX" in ln or "WARNING" in ln:
            print(ln)
    print(f"Built {fbx_path}")

    if args.to_unity:
        if not args.unity_dir.exists():
            print(f"WARNING: Unity dir {args.unity_dir} not found; skipped copy.")
            return
        dst = args.unity_dir / fbx_path.name
        shutil.copy2(fbx_path, dst)
        print(f"Copied to Unity: {dst}")


if __name__ == "__main__":
    main()
