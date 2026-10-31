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

## Findings on these takes

- **Placement.** All five course walks start on the route start, follow the route through both gates and end on the
  goal; `jumping` stays around the squeeze gate. Stills: `render_stills.py` -> `data/out/stills/`.
- **Contacts** (`contact_check.py`, Unity's 2 cm contact radius, person at the VR scale): at the squeeze gate the
  markers touch the panels for 1.1-2.0 s per walk and reach at most 0.55-0.98 cm inside them, never 2 cm; nothing
  else is touched (cabinet, room scan, clutter).
- **Rigid body `v4w2` is a false lock.** It is defined in Motive (8 markers, 39 cm long) but no such object is in
  the capture: its pose jumps up to 1.3 m between 10 ms frames (46-102 jumps over 10 cm per take) and alternates
  between floor height and about 1.2 m. Motive fits it to 3 (sometimes 4-5) body markers per frame, which are
  then missing from the unlabeled markers. The NatNet marker set fills its other 5 members with expected
  positions; `mocaprec.Take.rb_obs` keeps only the members Motive saw. For labeling and fitting, merge
  `rb_obs` of `v4w2` back into the body markers; for new captures, deactivate the `v4w2` asset in Motive.
- **Unlabeled markers**: 41-54 per frame, 288-510 distinct Motive trajectory ids per take (trajectories break
  often), so labeling cannot rely on Motive's ids alone.

## SMPL-X fit (`fit_smplx.py`)

```bash
PYTHONNOUSERSITE=1 $PY analysis/fit_smplx.py $R [--takes NAME ...]     # -> data/out/smplx/<take>.npz/.json
```

Fits SMPL-X (neutral, 16 betas, VPoser v1 body pose, flat hands) to the **native, true-size** markers; the
model files are read from `/data/Humanoid/Marker2SMPLX/body_models` (override `MTCVR_BODY_MODELS`) and never
copied into this repository. No marker names are needed:

1. Body points per frame = unlabeled markers within 0.9 m of the headset + the members Motive gave to `v4w2`;
   the 4 headset markers are rigid points on the head joint.
2. Start pose in the quietest upright 0.5 s (label-free: each point ~9 mm off the nearest skin vertex; facing
   from the headset's +X axis, which points along the walking direction in every walk).
3. Anchors: nearest vertex + rest-frame offset per point (MoSh style).
4. Tracking frame by frame (Hungarian matching to predicted anchors, gated at 8 cm, Motive trajectory ids as a
   tie-break), then a batch fit of all frames with temporal smoothing.
5. Re-anchoring: each anchor's vertex is re-picked from the marker's own motion (most rigid near-surface vertex).
   Needed because a still start pose cannot tell a hand hanging at the hip from a hip marker.
6. Batch fit, betas + offsets re-estimated on all frames, final fit.

Output `.npz`: AMASS-style `poses` (F, 165), `trans`, `betas`, `gender`, `mocap_framerate`, plus anchors,
per-frame residuals and matches. Frame: native Motive turned Y up, `p = (x, z, -y)`, metres.

## Website (`build_fit_site.py`)

```bash
PYTHONNOUSERSITE=1 $PY analysis/build_fit_site.py $R                    # data/out/site/ (no room scan)
PYTHONNOUSERSITE=1 $PY analysis/build_fit_site.py $R --with-room --out analysis/data/out/site_local
cd analysis/data/out/site_local && python3 -m http.server 8000           # then open http://localhost:8000
```

The page needs to be served over http (it fetches its data files). The published version leaves out the
ScanNet++ room; the `--with-room` copy is for local use only.

### Fit results, 2026-10-30 takes (run 2026-10-31)

| Take | Marker residual median / p95 | Headset | Re-anchored | Body height | Body mesh into obstacles (VR scale) |
|---|---|---|---|---|---|
| `take_1793414086680206800` | 4.2 / 13.1 mm | 0.8 mm | 1 | 1.76 m | 24 mm |
| `jumping` | 4.5 / 14.2 mm | 1.0 mm | 1 | 1.79 m | none |
| `take_1793415980024001300` | 3.6 / 10.8 mm | 1.2 mm | 1 | 1.76 m | 10 mm |
| `take_1793416015722837300` | 3.8 / 11.4 mm | 0.7 mm | 2 | 1.77 m | 10 mm |
| `take_1793416052198176000` | 3.7 / 10.8 mm | 0.7 mm | 2 | 1.76 m | 10 mm |
| `take_1793416088017881100` | 4.4 / 12.0 mm | 1.0 mm | 1 | 1.79 m | 10 mm |

- Every take: 43 body markers anchored. In each take one hand marker that hung next to the hip in the start pose
  was first anchored on the pelvis and moved to the hand/forearm by the re-anchoring step.
- Weak spots left: `…086` and `…015` have 2-3 anchors matched in only 12-25 % of frames (likely duplicate or
  ghost points in the start frame) and 1-2 body points per frame without an anchor; `…052` has one anchor at
  31.6 mm (matched 14 %). The right-hand marker in `…015` and `…052` ended on the forearm near the wrist.
  Possible refinement: add anchors for persistent unmatched points, drop rarely matched ones, re-anchor twice.
- The `jumping` start window had picked a crouch (96 mm start-pose RMS); windows now require an upright stance
  (2.2 mm). Body heights agree within 3 cm across the walks.
- Website: published privately at https://claude.ai/artifact/XvqQfY8Nbm2NzBStR5Bqnw (no room scan);
  `data/out/site_local/` is the same page with the room, for local use only.

## SMPL-X videos with the full environment, and the first-person 3DGS view

```bash
# viewer with the SMPL-X body (reduced mesh every 2nd frame = 50 fps), then 1080p50 videos on the GPU
PYTHONNOUSERSITE=1 $PY analysis/build_viewer.py $R --fits analysis/data/out/smplx --body-step 2 --out analysis/data/out/viewer_body
PYTHONNOUSERSITE=1 $PY analysis/render_videos.py --viewer analysis/data/out/viewer_body/mocap_scene_viewer.html \
    --out analysis/data/out/fit_videos --size 1920x1080 --fps 50 --query hide=cam --angle vulkan --quality 95 --crf 18 --suffix _smplx_main
# first-person 3DGS (needs analysis/data/3dgs/ckpt_final.pt + manifest.json from the MTC_Next scene package
# 7b6477cb95_test_only, primary 3dgs_meshdepth_w0.3), then picture-in-picture
PYTHONNOUSERSITE=1 CUDA_HOME=/usr/local/cuda-12.9 analysis/data/venv-gsplat/bin/python analysis/fpv_3dgs.py $R
$PY analysis/compose_pip.py                                               # -> data/out/pip/
```

- Headless Chrome renders WebGL on the RTX 5090 with `--angle vulkan` (0.05 s per 1080p frame, about 15x faster
  than SwiftShader).
- `venv-gsplat` (git-ignored, in `analysis/data/`): the `soma` env's Python with `--system-site-packages` plus
  `gsplat==1.5.3`; its CUDA kernels build on first use with CUDA 12.9 (about 40 s).
- First-person camera: the Pico rigid body as Unity placed it (VR scale, eye at about 1.30 m), front = rigid body
  +X, up = +Z. Its forward matches the camera rotation Unity logged to a median of 1.9-3.8 deg per take.
  Course objects and route markers are rasterised with PyTorch3D and composited by depth over the 3DGS.
- Hands in the videos and on the website: the MANO mean ("relaxed") hand pose, read at run time from an official
  SMPL-X v1.1 file (`MTCVR_HANDS_MEAN`, default `/data/Humanoid/UMR/smpl/SMPLX_NEUTRAL.npz`; the Marker2SMPLX model
  files lack `hands_meanl/r`). The fits keep hand pose 0 (no finger markers); a standard `smplx` loader with its
  default `flat_hand_mean=False` shows the same relaxed hands. The display mesh keeps hand vertices at full
  resolution (vertex clustering at 2 cm elsewhere); clustering fingers at 2.5 cm had torn them apart.
