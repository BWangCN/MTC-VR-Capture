# Obstacle course: 7b6477cb95, seed 0

The supplied `7b6477cb95_seed0.zip` is extracted unchanged into
`Assets/RealEnvs/7b6477cb95_seed0/`. Its README, previews, layout, and task remain
with the three GLB assets. glTFast 6.14.1 is already a project dependency.

## Open and run

1. Open this project with Unity **2022.3.21f1** and allow asset import to finish.
2. Open `Assets/Scenes/ObstacleCourse_7b6477cb95_seed0.unity`.
3. Connect SteamVR/ALVR as for `StreamingEnv`, then press Play.

To regenerate, run **MTC > Build Obstacle Course (7b6477cb95 seed 0)**. This
replaces the generated course scene using the saved `StreamingEnv` as the XR/mocap
rig template. Save custom course changes separately before rebuilding.
The apartment scene and its assets are preserved.

## Scene behavior

- `Environment/Visuals` renders `scene.glb`, including the vertex-colored room.
- The room, primitive gate, and route markers use the persistent `VertexColors.mat`
  material and `MTC/Scan Vertex Color` shader. These meshes have vertex colors but
  no explicit material in the GLB; relying on the importer's default material can
  leave them pink. The shader supports stereo instancing and preserves their
  scanned colors without adding lighting. Textured objects retain their materials.
  **MTC > Repair Obstacle Course Materials** repairs an existing course scene;
  the same repair runs when that scene opens. Raw GLB previews may still show
  pink because the material overrides belong to the generated Unity scene.
- `Environment/Collision` uses only `collision.glb` as invisible stationary mesh
  proxies. Room floor triangles near y=0 are removed from contact feedback;
  object hulls are retained. The generated room contact mesh is saved as an asset.
- `Environment/Route Markers (optional)` renders `markers.glb`. Disable it to hide
  the start, goal, planned route, and mug destination overlay.
- OpenXR, OSC body, UDP body, OptiTrack, and contact-feedback settings are inherited
  from `StreamingEnv`. Mocap contact references point to this course.
- Spawn is the exported route start, Unity **(-3.353, 0, 3.5137)**, facing toward
  the route goal. Scene JSON positions convert as
  `(x, y, z) = (-scene_y, scene_z - 0.0769, scene_x)`.
- Scale is 1 with no extra rotation: the GLBs already use metres and Y up;
  glTFast applies the handedness conversion. `RealEnvEmbodiment` starts disabled
  to preserve metric environment scale. Keep it disabled when using the automatic
  player resize described below.

This provides a VR scene for viewing and contact feedback. The mug and clutter
stay stationary; grabbing, task scoring, route/gate completion, and the 180-second
timer are metadata only and are not implemented by this importer.

## Automatic 1.3 m headset height

The mocap receiver adds `PlayerHeightScaler` to XR Origin at runtime. Stand upright
for the initial **10 seconds**; the headset HUD shows the countdown. With valid
headset tracking and a plausible floor-relative height, the rig is resized once so
the headset is **1.3 m above the XR Origin floor plane**. This is eye/headset height,
with no extra top-of-head offset. If tracking is unavailable, resizing waits for it.

One uniform factor (`1.3 / measured headset height`) applies to all three axes of
the rig and the displayed mocap body, including controllers, marker spacing and
body visuals. The floor point beneath the headset stays fixed, and the environment
retains its metric scale. After calibration, walking and crouching remain natural;
the camera is not continually forced to a fixed height. Script reloads do not apply
the factor again. Start a new Play session to calibrate again.

Mocap alignment still works in the original tracking coordinates, then the uniform
presentation transform is applied. This keeps later Pico updates from undoing the
resize. Raw NatNet/Pico recordings retain their original physical coordinates.
The existing native crop protocol receives the resized headset-relative transform,
so `filtered_frames.jsonl` agrees with the displayed crop. No native-client update
or restart is needed for this height feature. The automatic scale requires an
upright, uniformly scaled XR Origin and a headset beneath that rig.

## Mocap marker alignment

Valid Motive positions update the alignment target at any distance. A rotation
outlier retains the previous rotation while position correction continues; after
a tracking timeout, rotation can be reacquired too. Tracking-quality checks and
the pose timeout still apply.

`Limit Position Correction Speed` is off by default, including for existing scenes
that saved the old 0.25 m/s limit. Position correction retains exponential smoothing,
so larger offsets catch up faster and nearby corrections ease in. To impose a speed
limit deliberately, enable that option and set `Maximum Correction Speed Meters
Per Second`; zero means unlimited. `Position Correction Rate` controls convergence.

Run **MTC > Run Mocap Alignment Checks** to exercise the actual receiver with
synthetic near/far poses, rotation rejection, tracking reacquisition, and speed-limit
settings without connecting to UDP or moving the scene's XR rig. The report is
written to `Logs/OptiTrackAlignmentChecks.txt`.

For motion latency, the receiver's once-per-second Console line reports `local
frame age` in milliseconds from UDP receipt to the current Unity update, alignment
error in metres, accepted rigid-body pose age, active smoothing rates, and the PICO
pose's availability/quality/error. Local age excludes capture, Motive processing,
and any queue upstream of Unity; it does not measure camera-to-headset latency.
Compare marker movement in the PC Game view with the headset before changing
streaming settings. The receiver releases and reopens its socket on disable/enable,
including Play-mode script reloads, so edits do not silently stop incoming data.

## Source restrictions

The source marks this as **test data for research only**, with a ScanNet++ room
that must not be redistributed. Keep it local unless you have the necessary
rights. Object provenance and attribution notes are in the supplied package
README; the per-object source licence files referenced there are not in this zip.

## Batch generation / validation

With Unity closed, invoke its editor executable with:

```text
-batchmode -quit -projectPath <absolute path to VR_G1_RealTime_PC> -executeMethod ObstacleCourseSceneBuilder.Build -logFile <log path>
```

`ObstacleCourseSceneBuilder.Validate` reopens the generated scene and checks that
collision meshes persist, collision renderers are hidden, render geometry has no
colliders, metric scale is preserved, and mocap contact bindings target the course.
It also rejects missing materials, missing/unsupported shaders, and shader errors
on visible course renderers.
