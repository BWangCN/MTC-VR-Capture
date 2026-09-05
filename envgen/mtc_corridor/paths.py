"""Path discovery for the corridor benchmark.

Two distinct categories of path:

1. BUNDLED ROBOT ASSET (``ROBOT_ASSET_DIR`` and friends). The G1 model
   (MJCF + meshes + USD) ships INSIDE this package under ``assets/g1/``.
   The benchmark is self-contained: it does NOT need humanoid_maobu to
   instantiate the robot. These paths are always valid.

2. OPTIONAL MAOBU TEST CLIENT (``MAOBU_*``). humanoid_maobu is a *test
   client* that provides trainable policies + reference motions to run
   THROUGH the benchmark. It is NOT a structural dependency -- only
   needed when you actually run the maobu policy (examples 05/07/10).
   Discovered by walking up for a dir containing both ``SimEnv/`` and
   ``humanoid_maobu/``; absent => those examples skip, the rest works.

Env var overrides (handy for Linux desktop migration / relocating maobu):
- ``MTC_REPO_ROOT``  - repo root override (for the maobu test client)
- ``MTC_MAOBU_ROOT`` - maobu checkout override (lets maobu live anywhere)
"""

from __future__ import annotations

import os
from pathlib import Path

# --------------------------------------------------------------------------
# 1. Bundled robot asset (self-contained -- no external dependency)
# --------------------------------------------------------------------------
# paths.py lives at  <benchmark>/mtc_corridor/paths.py
# benchmark root is two levels up.
BENCHMARK_ROOT: Path = Path(__file__).resolve().parent.parent
ROBOT_ASSET_DIR: Path = BENCHMARK_ROOT / "assets" / "g1"
G1_MJCF_PATH: Path = ROBOT_ASSET_DIR / "mjcf" / "g1.xml"
G1_MESHES_DIR: Path = ROBOT_ASSET_DIR / "meshes" / "g1"
G1_USD_PATH: Path = ROBOT_ASSET_DIR / "usd" / "main.usd"


# --------------------------------------------------------------------------
# 2. Optional maobu test client (NOT required to run the benchmark)
# --------------------------------------------------------------------------
def _find_repo_root(start: Path) -> Path:
    p = start.resolve()
    ancestors = [p, *p.parents]
    for ancestor in ancestors:
        if (ancestor / "SimEnv").is_dir() and (ancestor / "humanoid_maobu").exists():
            return ancestor
    for ancestor in ancestors:
        if (ancestor / ".git").exists():
            return ancestor
    return start.resolve().parent.parent.parent


REPO_ROOT: Path = Path(os.environ.get("MTC_REPO_ROOT") or _find_repo_root(Path(__file__)))
MAOBU_ROOT: Path = Path(os.environ.get("MTC_MAOBU_ROOT") or (REPO_ROOT / "humanoid_maobu"))

# Standard sub-paths (don't fail at import time if they're missing -- callers check)
MAOBU_LOGS_DIR: Path = MAOBU_ROOT / "logs" / "rsl_rl" / "g1_tracking"
MAOBU_MOTIONS_DIR: Path = MAOBU_ROOT / "motions" / "datadriven_v1"
DEFAULT_MAOBU_CHECKPOINT: Path = (
    MAOBU_LOGS_DIR / "2026-03-21_01-57-52_C0_forward_walk" / "model_2999.pt"
)
