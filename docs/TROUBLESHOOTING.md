# Troubleshooting

Failures this pipeline has actually hit, and what they looked like. Ordered by
how much time they cost.

## Capture silently does nothing

**Symptom.** Controller buttons do nothing on the headset. No CSV appears. The
session appears to hang for about a minute, then gives up.

**Cause.** `PXR_SessionAutoFlow` aborts its whole coroutine on
`pivotAtHMD && hmd == null`, so nothing downstream of it runs — no calibration,
no `worldRoot` rescale, and critically `mapper.roomSpaceRef` is never assigned.
`MTC_AtomicSession.Boot()` then spins on that reference for its full 60 s timeout.

**Fix.** Point `AutoFlow.hmd` at the *active* camera under the XR rig
(`XR Origin/Camera Offset/Main Camera`). A scene can hold a second, inactive
`Main Camera`; picking that one reproduces the failure silently, so match on the
rig path rather than on the name. `MTC > Fix Capture Session` does this.

**Why it can look like a regression.** An APK built before the reference was lost
keeps working. The scene can be broken for days while capture still succeeds,
and then the next rebuild breaks everything at once.

## Scene saved in Trim mode

**Symptom.** Capture rig components are disabled and a `_MTC_Trimmer` object is
present. Nothing records.

**Fix.** Switch back to Capture mode before saving the scene, then rebuild. See
`TrimCaptureMode.cs`.

## adb pull truncates every filename

**Symptom.** Pulled files are named `foo.cs` instead of `foo.csv`,
`manifest.json` instead of `manifest.jsonl`. Contents are complete.

**Cause.** An off-by-one in how adb.exe derives the destination basename when the
target is a directory, under MSYS-style shells on Windows.

**Fix.** Do not re-pull; just restore the last character.

```bash
for f in "$STAGE"/*.cs; do mv "$f" "${f}v"; done
```

Verify the transfer by byte count against the device rather than by name:
`adb shell "ls -l $DEV"`.

## adb cannot create the destination directory

**Symptom.** `adb: error: cannot create file/directory '/d/...'`.

**Cause.** `MSYS_NO_PATHCONV=1` is required so the *remote* path is not mangled
into a Windows path, but it also stops the *local* path being converted.

**Fix.** Keep `MSYS_NO_PATHCONV=1` and pass the local destination in Windows form
(`D:\MTC_data\...`).

## Collision log is all zeros

The `hit` column has been 0 across every take for extended periods, including
clips with geometrically proven penetration. Do not use it as evidence of
clearance. Derive contact from geometry — foot/body position against the
obstacle AABB — and treat the collision CSV as unverified.

## The person walks outside the corridor

The generated corridor is 4 m long, but a real capture room is bigger and people
routinely walk 2 m past the end — in one session 36% of frames were outside the
corridor. Two consequences:

- Segment boundaries derived from "where the person stood still" land at the
  *room's* turnaround points, not the corridor's.
- A segment can satisfy "moved forward more than a metre" while lying entirely
  beyond the obstacle, having never crossed it.

Always require an actual crossing of the obstacle band, not just net forward
displacement.

## Segmentation returns far fewer clips than expected

Stand-based segmentation assumes the person stops between passes. When they walk
continuously, several passes merge into one span whose net displacement cancels
out, and the span is discarded as a reposition. One session recovered 27 of 46
real crossings this way.

Cross-check against an independent count: how many times does the pelvis cross
the obstacle band in the +z direction? Segment count should equal crossing count.
`tools/segment_turning_point.py` cuts on turning points for this reason.

## FBX does not match the YAML

The YAML is what the generator intended; the FBX is what the headset renders and
what motion gets captured against. They have diverged — one env shipped with a
sub-mesh a metre off the intended row, which widened the measured obstacle band
and polluted every downstream metric keyed to it.

Read the built FBX and check the dimension that defines the behaviour before
capturing on it.

## CSV lateral axis is sign-flipped versus the FBX

Positions in `pxr24_csv_v2` are corridor-frame; the lateral component is negated
relative to FBX x. Established empirically: without the flip, 0% of pelvis frames
fall inside the gap of a one-sided-wall env; with it, 66-100%.

Quaternions are world-frame while positions are corridor-frame — reconcile with
`room_yaw_deg` from the manifest. Analyses using positions only are unaffected.

## Determinism broke between runs

Thread randomness through the seeded `rng` argument. Two traps that have bitten:
module-level `random`/`np.random`, and `hash()` for seeding — the latter is
salted per process, so it silently changes every run.
