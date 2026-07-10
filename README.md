# MTC-VR-Capture

**24-point SMPL body-motion capture + trimming, on a PICO VR headset (Unity 2022.3).**

This repository contains the *capture* and *trimming* code from the MTC data-collection
pipeline: it reads full-body 24-joint tracking from a PICO headset + motion trackers,
transforms it into world / room-local coordinates, and records it to CSV
(`pxr24_csv_v2`). A desktop trimmer then cuts long recordings into clean per-take clips.

> **Migration note.** The intended next step is to port this from PICO to **Meta Quest 3**.
> Only **one** file touches the PICO SDK — `Core/PXR_BodyTrackingFeed.cs`. Everything
> downstream consumes a platform-neutral `RolePose[]` and is engine-agnostic. That file
> is the entire porting surface. See [Quest 3 migration notes](#9-notes-for-quest-3-migration).

---

## Table of contents
1. [What's in here / what's not](#1-whats-in-here--whats-not)
2. [Prerequisites](#2-prerequisites)
3. [Third-party dependencies you must obtain](#3-third-party-dependencies-you-must-obtain)
4. [Unity project setup](#4-unity-project-setup)
5. [Scene setup (minimal capture rig)](#5-scene-setup-minimal-capture-rig)
6. [Build & deploy to the PICO](#6-build--deploy-to-the-pico)
7. [On-device capture flow](#7-on-device-capture-flow)
8. [Retrieving & trimming the data](#8-retrieving--trimming-the-data)
9. [Notes for Quest 3 migration](#9-notes-for-quest-3-migration)

Also see: **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** (per-script reference + dependency map)
and **[docs/CSV_FORMAT.md](docs/CSV_FORMAT.md)** (the `pxr24_csv_v2` schema).

---

## 1. What's in here / what's not

```
Assets/MTCCapture/
├── Core/
│   ├── PXR_BodyTrackingFeed.cs      # PICO SDK reader (the ONLY PICO-coupled file)
│   ├── PXR_24PointMapper.cs         # local→world + adapter + CSV writer
│   ├── PXR_SessionAutoFlow.cs       # hands-free session orchestration
│   ├── PXR_SessionControlPanel.cs   # manual VR-UI session
│   ├── UIFollowHMD.cs               # world-space UI that tracks the head
│   └── PositionIndex.cs             # standalone 24-joint enum (removes SMPL-X dep)
├── Trimming/
│   └── CsvReplayTrimmer.cs          # Editor scrub + trim tool (self-contained)
└── Replay/
    └── PXR_24Point_CSV_Replay.cs    # Gizmos stick-figure replay (optional QA)

tools/csv_trimmer.py                 # Python desktop trimmer (alternative)
docs/                                # architecture + CSV spec
```

**NOT included** (obtain separately — §3):
- **PICO Unity Integration SDK 3.3.1** (proprietary, download from PICO).
- **SMPL-X for Unity** (MPI research license; only for the *optional* first-person avatar — **not needed to record CSV**).

Recording CSV needs **no** SMPL-X. The avatar is a visualization nicety only.

---

## 2. Prerequisites

| Requirement | Version used |
|---|---|
| Unity | **2022.3.21f1** (LTS; any 2022.3.x should work) |
| Unity module | **Android Build Support** (+ OpenJDK, Android SDK/NDK) |
| PICO Unity Integration SDK | **3.3.1** |
| OpenXR plugin (`com.unity.xr.openxr`) | 1.15.1 |
| Headset | PICO 4 Ultra (or a PICO with **body tracking** support) + **PICO Motion Trackers** |
| PICO system | Motion Tracker / body-tracking feature enabled, trackers paired |
| Desktop (for trimming) | Unity Editor, or Python 3 + matplotlib for `tools/csv_trimmer.py` |

Body tracking on PICO uses **worn motion trackers** (typically 2 for basic, up to a
full-body high-precision set). The SDK always returns a fixed **24 joints**; tracker
count affects *fidelity*, not the data shape. Calibration is done on-device.

---

## 3. Third-party dependencies you must obtain

### PICO Unity Integration SDK 3.3.1 (required)
1. Download from the PICO developer portal (developer.picoxr.com → Unity SDK), version **3.3.1**.
2. Add it to your project. In the original project it is referenced as a **local package**
   in `Packages/manifest.json`:
   ```json
   "com.unity.xr.picoxr": "file:../PICO-Unity-Integration-SDK",
   "com.unity.xr.openxr": "1.15.1"
   ```
   (You can instead import it as a `.unitypackage` into `Assets/`.)
3. This provides the `Unity.XR.PXR` namespace + `PXR_MotionTracking` API used by
   `PXR_BodyTrackingFeed.cs`. The body-tracking call is
   `PXR_MotionTracking.StartBodyTracking(BodyJointSet.BODY_JOINT_SET_BODY_FULL_START, bones)`
   (this overload/enum exists in **3.3.x**; older 3.1.x differs).

### SMPL-X for Unity (optional — only for the live avatar)
- Register + download "SMPL-X for Unity" from <https://smpl-x.is.tue.mpg.de> (MPI, research license).
- Import into `Assets/SMPLX/`. **Do not redistribute** the model/textures.
- Then add the original project's avatar scripts (`RealTimeSMPLX.cs`, `RealtimeSMPLXBinder.cs`,
  `PicoToSMPLBridge.cs`) — and **delete `Core/PositionIndex.cs`** (RealTimeSMPLX.cs re-defines it).
- Skip this entirely if you only need CSV capture.

---

## 4. Unity project setup

1. **Create** a new Unity **2022.3.21f1** project (3D / URP both fine).
2. **Add Android Build Support** via Unity Hub if not already installed.
3. **Import the PICO SDK** (§3). Run the PICO **project-setup / validation** wizard it
   offers (it configures XR Plug-in Management, color space, etc.).
4. **XR Plug-in Management** (`Edit ▸ Project Settings ▸ XR Plug-in Management`):
   enable the **PICO** provider under the **Android** tab.
5. **Player settings (Android):**
   - Minimum API level per PICO SDK docs (Android 10+/API 29+).
   - Scripting backend **IL2CPP**, target architecture **ARM64**.
   - Active input handling: as your project needs (the capture scripts don't require the new Input System).
6. **Enable body tracking / motion tracker** in the PICO settings section (the SDK exposes
   a toggle; body tracking also requires the corresponding on-device permission).
7. **Copy `Assets/MTCCapture/` from this repo into your project's `Assets/`.**
   With the PICO SDK present it should compile with no errors. (If Unity complains about a
   duplicate `PositionIndex`, you have both this repo's `PositionIndex.cs` **and** the
   original `RealTimeSMPLX.cs` — keep only one.)

---

## 5. Scene setup (minimal capture rig)

Create a scene with the following objects. (The original project ships one large
"kitchen-sink" scene; this is the trimmed-down capture rig — build it fresh.)

```
XR Origin (XR Rig)            ← from XR Interaction Toolkit / PICO rig prefab
└── Camera Offset
    └── Main Camera           ← the HMD camera (CenterEye)

TrackingOrigin  (empty)       ← reference frame for PICO local→world (often = XR Origin)

BodyTrackingFeed  (empty)
└── PXR_BodyTrackingFeed
      autoStartTracking = true

24PointMapper  (empty)
└── PXR_24PointMapper
      feed          → BodyTrackingFeed (PXR_BodyTrackingFeed)
      trackingOrigin→ TrackingOrigin (Transform)
      hmdCamera     → Main Camera (Transform)
      roomSpaceRef  → (optional) your environment root, to record in room-local coords
      writeEveryNFrames = 1
      recordCsv     = false   (a session script flips this on)

Session  (empty)   ← pick ONE of:
└── PXR_SessionAutoFlow            (hands-free)
      mapper   → 24PointMapper
      hmd      → Main Camera
      worldRoot→ your world/environment root
      autoRun / autoCalibrate / autoEnterWorld / autoRecord  (toggle to taste)
   ── or ──
└── PXR_SessionControlPanel        (manual VR buttons)
      mapper   → 24PointMapper
      wire the Calibrate / EnterWorld / StartRec / StopRec / Back buttons
      uiFollow → a world-space Canvas with UIFollowHMD
```

**Wiring rules of thumb**
- `PXR_24PointMapper.feed` must point at the `PXR_BodyTrackingFeed` component.
- `trackingOrigin` is the transform PICO-local joints are expressed relative to; using
  your XR Origin (or a dedicated child) is typical.
- Set `roomSpaceRef` **only** if you want positions recorded relative to an environment
  root (writes `coordinate_space=room_local`); otherwise positions are world-space.
- Height/embodiment calibration and `SetRecordingPrefix()` are driven by the session
  script — you normally do **not** call `BeginRecording()` yourself.

Trimming runs in a **separate desktop scene** (§8) — just one object with `CsvReplayTrimmer`.

---

## 6. Build & deploy to the PICO

1. `File ▸ Build Settings ▸ Android ▸ Switch Platform`.
2. Add your capture scene to **Scenes In Build**.
3. Connect the PICO via USB, enable **Developer mode + USB debugging** on the headset.
4. **Build And Run** (builds APK + installs + launches), or **Build** then:
   ```bash
   adb install -r MTCCapture.apk
   ```
5. Grant the body-tracking / motion-tracker permission when the app first requests it.

Bundle id in the original project was `com.DefaultCompany.Myproject` — set your own in
Player Settings (it also determines the CSV output path, §8).

---

## 7. On-device capture flow

Capture only runs on-device (`#if UNITY_ANDROID`; nothing records in the Editor).

**First, calibrate the trackers.** On launch, `PXR_BodyTrackingFeed.StartBodyTracking()`
runs; if the trackers aren't calibrated it auto-launches PICO's **Motion Tracker
calibration app** (`StartMotionTrackerCalibApp`). Put on / stand as instructed, calibrate,
and return to the app.

Then pick the session style you wired in §5:

### A. Hands-free — `PXR_SessionAutoFlow` (`autoRun = true`)
1. App launches → world hidden → short warmup.
2. Tracker calibration (above) if needed.
3. **5-second height calibration** — stand up straight. Eye height (`hmd.position.y`) +
   `eyeToTopOfHeadOffset` (≈0.12 m) → your height → embodiment `scale = 1.27 / height`
   (1.27 m = Unitree G1 target).
4. Optional spawn-corner placement + `AlignWorldForwardToHmd` orient the environment in
   front of you; `worldRoot` / `avatarRoot` activate.
5. If `autoRecord`, recording starts immediately. Files are named
   `{scene}_{corner|manual}_{NN}.csv` (auto-incrementing, never overwrites).

### B. Manual VR UI — `PXR_SessionControlPanel`
A world-space canvas (kept in front of your head by `UIFollowHMD`) shows:
**Calibrate → Enter World → Start Recording / Stop Recording** (+ status text). Start/Stop
map to `mapper.BeginRecording()` / `EndRecording()`.

**Where recordings land:** `Application.persistentDataPath`, i.e.
```
/storage/emulated/0/Android/data/<your.bundle.id>/files/
```
Also written there: one-shot diagnostic dumps `pxr_24pts_raw_local_*.txt` and `bd_dump_*.txt`.

---

## 8. Retrieving & trimming the data

### Pull the CSVs to your PC
```bash
adb pull /storage/emulated/0/Android/data/<your.bundle.id>/files/  ./captures/
```
(or use MTP / the headset file manager).

### Trim — Unity Editor (`CsvReplayTrimmer`)
1. In a desktop Unity scene, add an empty GameObject with **`CsvReplayTrimmer`**.
2. Set `csvFilePath` to a pulled CSV (absolute path). Optionally set `outputDir`,
   `outputNamePrefix`, and `environmentRoot` (a mesh with `MeshCollider`s → live collision
   read-out while scrubbing).
3. Press **Play**. An IMGUI timeline appears (playhead + green/red trim handles + a live
   `Clear/NEAR/WARNING/COLLISION` label). Keyboard:

   | Key | Action |
   |---|---|
   | **Space** | play / pause |
   | **← / →** | −1 / +1 frame |
   | **[ / ]** | −10 / +10 frames |
   | **A** | set trim **start** |
   | **D** | set trim **end** |
   | **1 / 2 / 3 / 4** | category = easy / avoid / near / bad |
   | **S** | **save** clip `{prefix}_{category}{NN}.csv` |
   | **R** | reset markers |

   Saved clips preserve the `#` metadata block, add `# clip_category=<cat>`, and re-write
   only the `[start,end]` rows (lossless).

### Trim — Python alternative (`tools/csv_trimmer.py`)
```bash
python tools/csv_trimmer.py path/to/recording.csv -o trimmed/
```
Same shortcuts (Space / ←→ / [ ] / A / D / S / R); 3D matplotlib view with a scrub slider
and auto-detected trajectory boundaries.

---

## 9. Notes for Quest 3 migration

- **Single porting surface:** `Core/PXR_BodyTrackingFeed.cs` is the only file that uses the
  PICO SDK (`Unity.XR.PXR`). Replace it with a Quest equivalent that fills the same contract:
  populate `RolePose[] LatestRolePoses` (24 entries of `{ Vector3 pos (local, m), Quaternion rot (local), bool valid }`),
  in the **same `PositionIndex` joint order**, and start/calibrate tracking. Everything
  downstream (`PXR_24PointMapper`, both session scripts, trimmer, CSV format) is unchanged.
- Meta's full-body input comes from the **Movement SDK (Body Tracking)** — map its body
  joints onto the 24-joint `PositionIndex` ordering and Unity local transforms.
- Keep the **CSV schema identical** (`docs/CSV_FORMAT.md`) so the same trimmer and all
  downstream MTC tooling keep working across both headsets.
- The height/embodiment calibration in `PXR_SessionAutoFlow` is pure Unity math (HMD eye
  height) — portable as-is.

---

### License / attribution
The code in `Assets/MTCCapture/` and `tools/` is MTC project code. Third-party components
(PICO SDK, SMPL-X for Unity, and the JSON/Matrix helpers in the original SMPLX package) are
**not** included here and carry their own licenses — obtain them from their sources.
