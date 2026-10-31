# Motive to Unity sender

Use MSVC and the installed NatNet SDK to build. From this sender's directory
(`Motive_Client` in the repository, or `C:\Users\Ryan\Projects\ASSIP`):

```powershell
cmake --preset windows-msvc
cmake --build --preset release
ctest --preset release
```

Override `NATNET_SDK_DIR` at CMake configuration if the SDK is elsewhere.
The preset keeps MSVC build files in `build-msvc`, independently of any old MinGW
cache under `build`. Release executables and the NatNet DLL still go into
`build/Release`, preserving the established application/firewall path and default
recordings directory. Stop the sender yourself before rebuilding its executable.

Stop the previous sender before launching the new one; two senders would both
publish to Unity's UDP port 5005. Launch from the repository root with this PC's
capture-network IPv4 address, for example:

```powershell
.\Motive_Client\build\Release\motive_client.exe 192.168.0.238
```

The server address and Unity destination remain configured in `motive_client.cpp`.
Marker positions, IDs, activation filtering, and Unity packet format are unchanged.

On Ryan's current workstation, the existing inbound NatNet application rule is
bound to `C:\Users\Ryan\Projects\ASSIP\build\Release\motive_client.exe`.
That project's sender uses that established path. A new executable path
can answer Motive control requests while receiving no frame data if it lacks an
app-specific inbound rule. Current summary output is redirected to
`Motive_Client/build/Release/assip-sender.stdout.log` in this repository; errors
are in the adjacent `assip-sender.stderr.log`. Backups of the previous ASSIP
source and executable are under `Motive_Client/build/backups/`.

## Recording from the headset

In Unity Play mode, press the **right controller's A button** once to start a new
take and again to stop and finalize it. `OptiTrackPointReceiver` automatically adds
`MotiveRecordingController`; no scene setup or separate Python recorder is needed.
The headset label changes to red `REC` only after the native client acknowledges
the command, and shows the saved frame count. A short controller pulse confirms
start/stop. `Mocap: no recorder` means control is unavailable or its status is stale;
`recording ERROR` means the take needs inspection. Unity also logs the take path.

For ALVR's `Quest2Touch` controller emulation, the Standalone **Oculus Touch
Controller Profile** must be enabled in Unity's OpenXR settings. A Vive-only
profile can provide working controller poses without exposing an A button. This
project enables the Touch profile. Stop and restart Unity Play mode after changing
interaction profiles so OpenXR creates the correct actions for the new session.
The recording component selects a physical right-hand device that exposes
`primaryButton`, excluding hand/palm pose devices sharing that hand. It shows
`Mocap: A unavailable` if no suitable button is exposed, rather than reporting
ready. Unity logs the chosen input device and each requested start/stop command.
The component's **Toggle Mocap Recording** context-menu command in Play mode can
also test recording control independently of the physical button.

The native client listens on **127.0.0.1:5011**. UDP commands are acknowledged and
retried with sequence IDs, so lost replies do not create duplicate takes or toggle
the state twice. The older `StartStopRecording.cs` / Python control on port 5010 is
separate and is not used here. Unity must run on the same PC as the client.

### Right B: headset-following marker box

Press **right-controller B** to toggle the marker crop on/off. It starts off.
The box follows the tracked headset position and extends **2 m down, 0.5 m up,
and 0.3 m left/right/forward/backward** (a 0.6 m × 0.6 m × 2.5 m box). It stays aligned
with world-up; looking down or turning your head does not rotate it. The headset
label shows `B: filter ON/OFF`; the button works independently of recording.

Markers outside the box disappear from live rendering and apartment contact/
haptics, but their tracking continues so they return when they enter the box or
filtering is disabled. Filtering uses the same aligned world positions as the
live point cloud and does not move the XR rig or change alignment/drift rates.
The headset reference works even when Motive's Pico tracking-valid flag is false;
the crop still depends on the existing mocap-to-Unity alignment being accurate.

During recording, `filtered_frames.jsonl` saves a second stream with the same
frame numbers/timestamps as `frames.jsonl`. Only marker arrays are cropped;
Pico and other rigid-body/skeleton/device context remain available. Selected
points retain native Motive coordinates and marker IDs. In marker sets, rejected
positions become `null` so marker indices still match metadata names. Filtering
is applied to **all raw markers**, including stationary points, before the
separate movement filter used by the Unity display. With B off, the second stream
retains every marker. The raw file and `pico.csv` always remain complete.

Unity sends the current headset-relative transform and B state at 20 Hz. Each
filtered frame includes `body_filter.enabled`, `valid`, box bounds, and the 3×4
row-major native-to-headset-relative transform. If updates stop for over 0.5 s or
the reference is unavailable while filtering is on, that frame's crop is marked
invalid and contains no selected markers; raw data remains available. The summary
counts these frames as `invalid_filter_frames`. `Filter recording not synced` in
the headset means the new native filter protocol has not been acknowledged: install
the staged native update and restart the sender yourself before relying on the
cropped recording. The source of the box is the headset's Unity pose, not Motive's
Pico rigid-body validity flag. This is a spatial crop, not suit-marker identification;
other reflective objects inside the box will still pass.

The new native executable must be installed before this feature works. We do not
start, stop or restart the sender. If an update was staged beside the running
ASSIP executable, stop the sender yourself, then run:

```powershell
& 'C:\Users\Ryan\Projects\ASSIP\Install-RecordingUpdate.ps1'
```

This copies `build/Release/motive_client.recording-update.exe` over the original
`motive_client.exe` and backs up the previous executable. It refuses to proceed
while that executable is running and launches nothing. Then start the original
`motive_client.exe` yourself with your usual local IP argument, allowing its
existing firewall rule to continue working.

The default output is `recordings` **beside the executable**, independent of the
terminal's working directory. For the ASSIP build this is:
`C:\Users\Ryan\Projects\ASSIP\build\Release\recordings\take_<UTC Unix nanoseconds>\`.
Override it with an optional second argument, for example:

```powershell
.\motive_client.exe 192.168.0.238 'D:\MocapRecordings'
```

Each take contains:

- `frames.jsonl`: one complete decoded NatNet frame per line: marker sets,
  **all** unlabeled and labeled markers (including stationary points), all rigid
  bodies, skeletons, assets, force plates, device analog samples, IMU and GPIO.
  Source frame numbers, timecodes, precision/host timestamps, raw flags and local
  receive timestamps are retained. Nonfinite numeric values become JSON `null`.
- `pico.csv`: one row per received frame, containing Pico's native position in
  metres, quaternion in **x,y,z,w** order, mean marker error, raw flags, availability,
  Motive's tracked flag and the existing 10 mm Unity quality decision. Untracked or
  poor-quality poses are saved without filtering. Missing poses have blank pose
  fields, not an invented zero/identity pose.
- `filtered_frames.jsonl`: frame-synchronized copy with the B-button marker crop
  and per-frame filter state/transform. Original rigid-body and other non-marker
  context is preserved. See the headset box details above.
- `metadata.json`: format/version, coordinate/units information, server version,
  configured frame rate, host clock frequency, and a model-description snapshot
  queried at the start of each take. The exact rigid-body name `Pico` is resolved
  again there. Avoid editing model definitions during a take; frame flags still
  preserve Motive's model-change indication.
- `summary.json`: written when stopped or on graceful client exit, with frame count,
  gaps in source frame IDs, recording drops, discontinuities and any error. A take
  without a summary was not finalized. Inspect errors/gaps before analysis.

Recording preserves **native Motive coordinates** before Unity's alignment,
smoothing or movement filtering. It records the decoded data delivered by NatNet;
it cannot recover frames Motive did not stream or packets lost before receipt.
The capture callback deep-copies frames into a bounded queue; serialization and
disk writes run on a dedicated worker. Queue overflow or a file error stops the
take and reports an error instead of silently dropping samples or blocking live
tracking on disk. Files flush once a second and on stop. Abrupt process termination
can lose the unflushed tail; stop the take before closing the application.

A take continues through Unity disconnects/script reloads; reconnecting queries
the actual native state. Press A after reconnection to stop it, or exit the native
client gracefully. This avoids silently truncating a capture on a Unity reload.

These raw marker clouds and the synchronized Pico sidecar preserve inputs for
later [SOMA processing](https://github.com/nghorbani/soma). They are not themselves
a SOMA inference run; offline input conversion, subject/marker selection and the
SOMA environment remain a separate step. The older Python C3D recorder only sees
filtered Unity packets and ignores Pico, so use the native take for that conversion.

The recording regression test uses an isolated loopback port and synthetic data;
it never connects to Motive or controls the live sender. It checks acknowledgments,
start/stop retries, stale commands, full JSON/CSV readback, deep-copy ownership,
missing and poor-quality Pico poses, graceful shutdown, queue overflow and file
errors, plus a paced 100 Hz / 1000-marker recording. Python is optional for building;
when available CTest also validates saved files using only its standard library.

## Diagnosing camera-dependent latency

The frame callback performs no console output, including on UDP send errors.
Per-marker printing previously happened every ten frames on that callback. A slow
terminal could block the callback and let old frames accumulate before Unity ever
received them. The main thread now prints one compact summary each second. A slow
terminal can stall that summary thread without blocking the callback's console I/O.

Each summary reports:

- `points raw/active`: whether entering the inner camera ring increases point count.
- `callback max`: maximum callback processing time since the previous summary.
- `Motive software`: processing/packaging time reported by the camera-server timestamps.
- `transit/queue`: age since Motive transmitted the frame when its callback begins.
  This includes any network/client backlog before the callback.
- `camera age`: age since camera mid-exposure when the callback begins.
- `send errors`: failed UDP sends since the previous summary.

Timing uses NatNet's synchronized host clock. `-1` means unavailable (including
unsupported camera timestamps or playback); clock synchronization needs time to
settle after connecting. Compare steady readings outside and inside the inner ring.
High Motive software time points upstream; low software time with high transit/queue
time points to delivery/client backlog. This instrumentation does not itself prove
which stage caused a previously observed delay.

The synthetic callback checks exercise 10, 100, and 1000 moving markers while
guarding stdout/stderr, plus UDP send errors and timestamp conversions. They do
not replace a live inner-ring/outer-ring comparison.
