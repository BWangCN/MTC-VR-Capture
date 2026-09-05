# Vendored from MTC-Corridor-Benchmark

`envgen/mtc_corridor/` and `envgen/scripts/` are a copy of the environment
generator from the private `BWangCN/MTC-Corridor-Benchmark` repo, so that this
repo is self-contained for data collection.

Everything here is a byte-for-byte copy of the source except **three path
defaults**, changed so a fresh clone works without configuration:

| File | Change |
|---|---|
| `scripts/make_standard_envs.py` | `MTC_CAPTURE_ROOT` falls back to `<repo>/captures` instead of a hard-coded `E:\MTC_data\capture` |
| `scripts/export_to_unity.py` | `MTC_UNITY_FBX_DIR` falls back to `envgen/fbx_out` instead of a hard-coded Unity project path |
| `tools/capture_pipeline.py` | same capture-root change, plus `sys.path` points at `envgen/` since the package moved |

Set the environment variables (see [../docs/RUNBOOK.md](../docs/RUNBOOK.md)) and
the behaviour is identical to the source.

**Syncing.** To pull changes down from the benchmark repo, copy the files over and
re-apply those three patches. Do not run `ruff --fix` across this directory: the
source is not lint-clean by design (its own `CLAUDE.md` says to lint only files you
modify), so a bulk autofix creates a large spurious diff against upstream.
