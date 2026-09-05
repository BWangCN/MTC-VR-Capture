# Architecture & script reference

Data flow:

```
PICO Motion Tracking SDK  (Unity.XR.PXR)
        │  24-joint local poses (pos + quat), every frame
        ▼
PXR_BodyTrackingFeed        Core/  — starts body tracking, auto-launches tracker calibration,
        │                            exposes RolePose[] LatestRolePoses
        ▼
PXR_24PointMapper           Core/  — local→world transform, axis/yaw adapter, embodiment scaling,
        │                            CSV writer (pxr24_csv_v2), OnPointsUpdated event
        ▼
   CSV on device            Application.persistentDataPath/*.csv
        │  (copy to PC)
        ▼
CsvReplayTrimmer            Trimming/ — Editor scrub + trim into categorized clips
```

Upstream of all of that, and downstream of the CSV, two more stages:

```
make_standard_envs.py       envgen/  — (seed, tier) -> CorridorEnv -> YAML + gen_meta.json
        │                             obstacle builders in new_obstacles.py
        ▼
export_to_unity.py          envgen/  — CorridorEnv -> flat scene JSON -> FBX (Blender headless)
        │                             the JSON is the human/LLM-editable contract
        ▼
AtomicEnvSet.ImportAll()    Editor/  — import this session's FBX, retire the previous set
        │                             selection driven by _index/session_filter.txt
        ▼
MTC_AtomicSession           Session/ — on-headset: walk the env list, switch, record
        │                             writes mtc_session_manifest.jsonl per take
        ▼   (adb pull)
capture_pipeline.py         tools/   — stow-batch: manifest -> <day>/<env_id>/raw/,
        │                             each take paired with its own collision log
        ▼
segment_turning_point.py    tools/experimental/ — reference impl: cut on pelvis turning points
```

## Environment generation (`envgen/`)

`generate_corridor(seed, tier)` is the single entry point and is **deterministic** —
the same seed yields the same corridor, and downstream analysis relies on it.
Thread any new randomness through the seeded `rng` argument; module-level
`random`/`np.random` and `hash()` both silently break it.

Obstacles carry a multi-label set of geometric constraints
(`mtc_corridor/constraint_rules.py`) calibrated to the Unitree G1: standing height
1.32 m, shoulder width 0.45 m, body thickness 0.20 m. Changing those formulas or
the tier ranges changes what every previously generated environment means.

Named env sets live in `SPECS` in `make_standard_envs.py`. `_atomic4_spec()` is the
smallest complete example of adding one: a slug, an obstacle type, and a per-level
parameter band.

Coordinate frames: the generator works in `+x` = start→goal, `+y` = lateral,
`+z` = up. FBX export is Y-up for Unity. The exported scene shifts the corridor so
start = (0,0,0). See CSV_FORMAT.md for the capture-side frames.

## Filing and segmentation (`tools/`)

`capture_pipeline.py stow-batch` is the only correct way to file a multi-env
session: it reads `mtc_session_manifest.jsonl` and routes each take to the env it
was recorded in. Plain `stow` handles a single file and pairs the collision log by
picking the newest in the folder — right for one long take, wrong for a session.

`segment_turning_point.py` cuts on the turning points of the pelvis trajectory
rather than on standing intervals, because people do not reliably stop between
passes. Its output is candidates for review, not a finished dataset.

Session orchestration wraps the mapper:
- **PXR_SessionAutoFlow** — hands-free: warmup → tracker calib → 5 s height calib → embodiment scale → (optional) spawn/align → BeginRecording.
- **PXR_SessionControlPanel** — manual VR UI: Calibrate → Enter World → Start/Stop Recording buttons (uses `UIFollowHMD` to keep the canvas in front of the head).

## Files in this repo (all our own code)

| File | Role | PICO SDK? |
|---|---|---|
| `Core/PXR_BodyTrackingFeed.cs` | SDK reader → `LatestRolePoses[]` | **yes** (`Unity.XR.PXR`) |
| `Core/PXR_24PointMapper.cs` | mapping + `pxr24_csv_v2` writer | no |
| `Core/PXR_SessionAutoFlow.cs` | hands-free session | no |
| `Core/PXR_SessionControlPanel.cs` | manual VR-UI session | no |
| `Core/UIFollowHMD.cs` | world-space UI follows the HMD | no |
| `Core/PositionIndex.cs` | **new** standalone 24-joint enum (removes SMPL-X dep) | no |
| `Trimming/CsvReplayTrimmer.cs` | desktop/Editor trimmer (fully self-contained) | no |
| `Replay/PXR_24Point_CSV_Replay.cs` | Gizmos stick-figure replay (QA, optional) | no |
| `tools/csv_trimmer.py` | Python desktop trimmer (alternative to the Unity one) | — |

**Only one file (`PXR_BodyTrackingFeed.cs`) touches the PICO SDK.** That is the single porting surface for a Meta Quest 3 migration — everything downstream consumes the platform-neutral `RolePose[]` and knows nothing about PICO.

## Class ↔ file name mismatches (copy by file, wire by class)

The original project has several scripts whose class name differs from the file name. Keep this in mind when adding components in the Inspector:

| File | Actual class name |
|---|---|
| `PXR_24Point_CSV_Replay.cs` | `PXR_24PointCsvPlayer` |
| (original) `RealtimeSMPLXBinder.cs` | `SMPLXRealtimeBinder_FPV` |
| (original) `PicoToSMPLBridge.cs` | `PicoToSMPLXBridge` |
| (original) `PXR_SmplxBridge.cs` | `PXR_SmplxBridgeSimple` |
| (original) `HeadlessClipperBinder.cs` | `HeadlessClipBinder` |

`PXR_BodyTrackingFeed`, `PXR_24PointMapper`, `PXR_SessionAutoFlow`,
`PXR_SessionControlPanel`, `UIFollowHMD`, `CsvReplayTrimmer` all match their file names.

## Known cruft (inherited, harmless)

- `PXR_24PointMapper.cs` contains two **commented-out** older copies of the class (~lines 531–1252). Only the top class (~16–528) is live. Left verbatim to match the validated project; safe to delete.
- Capture logic in the feed/mapper is wrapped in `#if UNITY_ANDROID`, so **nothing records in the Unity Editor** — the mapper/feed only produce data on-device.

## NOT included (and why)

- **`Assets/SMPLX/` (SMPL-X for Unity)** — MPI research license, ~250 MB of FBX/textures. Only needed for the *optional* live first-person avatar, not for CSV capture. Obtain separately (see main README).
- **PICO Unity Integration SDK 3.3.1** — download from the PICO developer site.
- **`Assets/PicoTracking/`** — an unrelated passthrough-camera streaming client (namespace `Robot.V2.Network`, defined inside those files, not an external DLL). No capture/trim script references it.
- Optional live-avatar scripts (`RealtimeSMPLXBinder.cs`, `PicoToSMPLBridge.cs`, `PXR_SmplxBridge.cs`, `RealTimeSMPLX.cs`), G1 retarget drivers, collision/streaming experiments — not part of the capture+trim core. If you want the avatar, pull those from the original project *and delete `Core/PositionIndex.cs`* (RealTimeSMPLX.cs re-defines it).
