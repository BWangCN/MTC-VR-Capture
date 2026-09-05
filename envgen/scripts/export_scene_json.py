"""Export ONE corridor as an editable scene JSON.

This is the teaching / LLM-editing entry point. The JSON it writes is the
*flat object list* (floor, walls, obstacle elements) with every object's
size + position. Editing a `position` and re-running `build_from_json.py`
moves that object in the rebuilt FBX -- no Python needed.

Usage:
    python scripts/export_scene_json.py --seed 211 --tier medium
    python scripts/export_scene_json.py --seed 211 --tier medium --out my_scene.json

Then edit the JSON (move objects) and rebuild:
    python scripts/build_from_json.py my_scene.json --to-unity
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from mtc_corridor import generate_corridor  # noqa: E402
from scripts.export_to_unity import env_to_blender_scene  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--tier", default="medium", choices=["easy", "medium", "hard"])
    ap.add_argument(
        "--out",
        type=Path,
        default=None,
        help="output JSON path (default: scenes_json/<env_id>.json)",
    )
    args = ap.parse_args()

    env = generate_corridor(seed=args.seed, tier=args.tier)
    scene = env_to_blender_scene(env)

    out = args.out or (PROJECT_ROOT / "scenes_json" / f"{env.env_id}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        json.dump(scene, f, indent=2)

    n_obj = len(scene["boxes"]) + len(scene["elements"])
    print(f"Wrote {out}")
    print(f"  corridor {env.env_id}: L={env.length:.2f} W={env.width:.2f}")
    print(
        f"  {len(scene['boxes'])} structure boxes + {len(scene['elements'])} "
        f"obstacle elements = {n_obj} editable objects"
    )
    print(f"  edit a 'position' field, then: python scripts/build_from_json.py {out} --to-unity")


if __name__ == "__main__":
    main()
