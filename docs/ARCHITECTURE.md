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
