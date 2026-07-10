# `pxr24_csv_v2` — CSV format spec

Every recording written by `PXR_24PointMapper` is a UTF-8 CSV with a metadata
header block (comment lines starting with `#`) followed by one header row and one
row per recorded frame.

## Metadata block (lines beginning with `#`, parsers skip them)

```
# pxr24_csv_v2
# units=m
# time=realtimeSinceStartup_relative_seconds
# saved_at=YYYY-MM-DD HH:mm:ss.fff
# embodiment_scale=<float>            # scale applied to bring the pilot to target height (e.g. G1 1.27 m)
# yaw_offset_deg=<float>              # global Y-axis yaw offset applied to the pose adapter
# record_origin_pos=<x>,<y>,<z>       # trackingOrigin.position at BeginRecording()
# record_origin_rot_xyzw=<x>,<y>,<z>,<w>   # trackingOrigin.rotation at BeginRecording()
# hmd_camera=<name|null>
# coordinate_space=room_local|world   # room_local if roomSpaceRef was set, else world
# room_space_ref=<name>               # only present when coordinate_space=room_local
# scene_prefix=<prefix>               # only present when a recording prefix was set
```

`record_origin_*` enable exact origin remapping on replay
(`p_replay = T_replayOrigin · inv(T_recordOrigin) · p_record`).

## Column layout

```
frame, time,
cam_px, cam_py, cam_pz, cam_qw, cam_qx, cam_qy, cam_qz,
p0x,  p0y,  p0z,  q0w,  q0x,  q0y,  q0z,
p1x,  p1y,  p1z,  q1w,  q1x,  q1y,  q1z,
...
p23x, p23y, p23z, q23w, q23x, q23y, q23z
```

- **2** leading columns: `frame` (int), `time` (relative seconds = `Time.realtimeSinceStartup - recordStart`).
- **7** columns for the HMD camera 6DOF: position `(px,py,pz)` + quaternion `(qw,qx,qy,qz)`.
- **24 joints × 7** columns each: position `(x,y,z)` + quaternion `(qw,qx,qy,qz)`.

Total = 2 + 7 + 24×7 = **177 columns**.

### Notes

- **Joint order** follows `PositionIndex` (see `Assets/MTCCapture/Core/PositionIndex.cs`): `p0` = pelvis/hip, … `p23` = right_hand.
- **Positions** are in world space, or room-local when `roomSpaceRef` is set (`coordinate_space` tells you which). Units metres.
- **Quaternions** `q{i}` are the **raw PICO local** joint rotations, order `w,x,y,z`. They are NOT re-adapted by the pose-adapter flips (those only affect positions).
- **Frame rate** ≈ headset refresh (record every `writeEveryNFrames`, default 1 → ~72 fps on PICO).
- **Legacy 170-column files**: older recordings written before the camera block have no `cam_*` columns → 170 columns. Readers auto-detect: `baseCol = numCols >= 177 ? 9 : 2`.
- Sample first data column of a trimmed clip also gets `# clip_category=<cat>` added by the trimmer.

## Coordinate convention (important for pairing with environment meshes)

The MTC corridor meshes are exported so the **corridor long axis = Unity +Z**
(start at origin → goal at +Z, length L), lateral = X, up = Y. Recordings whose
walking axis is not +Z were captured before the coordinate fix; rotate the whole
recording about world Y by the appropriate multiple of 90° to align (see the
`rotate_motion_y90.py` tool on the MTC side). Aligned motion walks +Z from ~0 to
+L with X within the corridor width.
