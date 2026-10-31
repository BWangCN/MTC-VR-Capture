# Mocap recordings in the obstacle course: sanity-check tools

Branch `mocap-analysis` (off `vr-g1-realtime-streaming`). Offline tools for the native Motive takes written by
`Motive_Client` (`MocapRecording.h`, schema `mtc_natnet_recording_v1`) while the VR app ran
`ObstacleCourse_7b6477cb95_seed0`. Nothing here changes the Unity project or the recorder.

## Data

Takes and outputs live in `analysis/data/` (git-ignored):

| Path | What |
|---|---|
| `data/raw/2026-10-30_recordings/recordings/` | `recordings.zip` from 2026-10-30, unzipped as is (SHA-256 `19d64b64f26b0971881c16132231e516d36ae302d0ae7a5c2742995c16526d93`): 6 takes + the session's Unity `Editor.log` |
| `data/out/alignment.json` | per-take summary and alignment numbers (`inspect_takes.py`) |
| `data/out/contact_check.json` | markers touching or inside obstacles (`contact_check.py`) |
| `data/out/viewer/mocap_scene_viewer.html` | interactive 3D viewer, all takes (`build_viewer.py`) |
| `data/out/videos/*.mp4` | one video per take (`render_videos.py`) |

The room is a ScanNet++ scan (research use only, not to be redistributed): keep the viewer and videos local.

## Environment

`/home/bwang25/miniconda3/envs/soma/bin/python` (Python 3.10, numpy, scipy, trimesh, Pillow; it also picks up
trimesh from `~/.local`). The viewer and videos need `google-chrome` (headless, WebGL through SwiftShader) and
`ffmpeg`; the page loads three.js 0.170.0 from jsDelivr.

```bash
R=analysis/data/raw/2026-10-30_recordings/recordings
PY=/home/bwang25/miniconda3/envs/soma/bin/python
$PY analysis/inspect_takes.py $R          # per-take summary + alignment check
$PY analysis/contact_check.py $R          # contacts with the course's collision proxies
$PY analysis/build_viewer.py $R           # data/out/viewer/mocap_scene_viewer.html
$PY analysis/render_videos.py             # data/out/videos/, from the viewer page (~0.35 s per video frame)
```

## Frames

| Frame | Axes | Where |
|---|---|---|
| Motive native | right-handed, Z up, metres | everything in the take files |
| Unity world | left-handed, Y up | the VR app; `ConvertPosition`: (x, y, z) -> (-y, z, x) |
| Scene glTF | right-handed, Y up, floor y = 0 | the course GLBs; = Unity world with x mirrored (Environment root at the origin, scale 1) |

## How the markers are placed in the scene

The take stores native Motive coordinates only. Unity showed a marker at `u = M p + t_align`
(`OptiTrackPointReceiver.MotivePointToWorld`), with `M = R_yaw * positionScale * C`:

- `M` per frame comes from the take: `filtered_frames.jsonl` -> `body_filter.motive_to_headset_delta` is
  `[M | t_align - c]` (c = Unity camera position), sent by Unity at 20 Hz, saved even when the crop is off.
- `t_align` comes from the `Editor.log`: `[XR Camera] Pos` lines (every 60 Unity frames) give `c` in Unity world;
  each is placed in the frame window between its neighbouring `OptiTrack | ... last frame N` lines and matched to
  the mapped Pico rigid body there (Unity aligns that rigid body onto the camera; `cameraOffsetFromRigidBody` is 0
  in the course scene). Adjacent log lines are not from the same Unity update, so they are not paired directly.

Checks (`inspect_takes.py`, all six takes): the logged camera lies 1.0-2.4 cm (median) from the mapped headset,
the same size as Unity's own logged alignment error (1.8-3.1 cm); `t_align` from the transform's translation at the
matched frames agrees to 1.3-2.5 cm; the Motive floor lands within 1.6 cm of the scene floor.

`positionScale` was not 1: an uncommitted `PlayerHeightScaler.cs` (in the log, not in this repository) resized the
player to a 1.30 m headset height, x0.8765 for the first two takes and x0.8229 for the last four, and Unity scaled
the mocap points by the same factor. The viewer shows the person at that scale (as seen in VR); the room is metric.
Fitting a body must use the native, unscaled positions.

## Takes (2026-10-30 session)

| Take | Frames | Content | Crop | Scale |
|---|---|---|---|---|
| `take_1793414086680206800` | 2166 | course walk | off | 0.8765 |
| `jumping` (= `take_1793414444108142100`) | 1936 | named "jumping"; stays around the squeeze gate (headset path 3.6 m) | off | 0.8765 |
| `take_1793415980024001300` | 1948 | course walk | on | 0.8229 |
| `take_1793416015722837300` | 1779 | course walk | on | 0.8229 |
| `take_1793416052198176000` | 1638 | course walk | on | 0.8229 |
| `take_1793416088017881100` | 1524 | course walk | on | 0.8229 |

Two short takes in the log (`take_1793414430330192100`, 189 frames; `take_1793414434033795300`, 309 frames) are
not in the zip.
