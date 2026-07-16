# Milestone 1 — Get PC → PICO streaming working (simple scene)

Goal: prove the **PC renders, the PICO headset displays it head-tracked** path,
using the most minimal scene possible. No G1, no networking, no Python — so if
something fails, the only suspect is the streaming connection itself.

> Device: **PICO 4 Ultra Enterprise/Business**.
> Path: **PICO Business Streaming → SteamVR → OpenXR (PCVR)**.
> (The old "PICO Live Preview" Unity plugin is a legacy consumer-PICO-4 dev tool,
> unsupported here — do NOT use it. Earlier runbook revisions that said "switch
> the XR provider to PICO Live Preview" are SUPERSEDED.)
>
> Scope: only touches `VR_G1_RealTime_PC`. Does NOT modify `My project` or this
> project's `SampleScene`. Whole milestone adds two files:
> `Assets/Editor/StreamingTestSceneBuilder.cs` + this runbook.

---

## Already in place on this PC (verified 2026-06-17)

- **Business Streaming 1.2.11.1** + **Streaming Service 2.2.4.4** (PICO) — installed
- **SteamVR** (Valve) — installed, and it is the **active OpenXR runtime**
  (`...\SteamVR\steamxr_win64.json`)
- PICO Connect 10.6.6 + PICO Developer Center — also installed (fallbacks)
- The headset is recognized over **USB** (`adb devices` shows it)

So nothing needs installing — this is "connect + press Play".

---

## Step 1 — Build the test scene

Open `VR_G1_RealTime_PC` in **Unity 2022.3.21f1**, then:

- Menu: **MTC → Build Streaming Test Scene** → creates/opens
  `Assets/Scenes/StreamingTest.unity` (XR Origin + head-tracked camera + an
  8×8 m room with walls + obstacle boxes + a light + on-screen XR diagnostics).
- If asked to save changes to SampleScene → **Don't Save**.

## Step 2 — Unity XR settings (two quick checks)

**Edit → Project Settings → XR Plug-in Management → Standalone tab (monitor icon):**

1. Provider = **OpenXR** ✅ (and **PICO Live Preview unchecked**). This is the
   project's original setting — if a previous step had you switch to PICO Live
   Preview, switch it back to OpenXR.
2. Click **OpenXR** (under XR Plug-in Management) → in the feature list, make
   sure **"Mock Runtime" is unchecked** (it fakes a headset and would block the
   real one). At least one **Interaction Profile** should be listed — HTC Vive +
   Khronos Simple are already enabled, which SteamVR supports. Good.

## Step 3 — Start the streaming session (headset is already USB-connected)

1. On the **PC**, launch the **Business Streaming** app (its Streaming Service is
   already running in the background).
2. Put on the headset and open its **streaming client** (the PICO-side Business
   Streaming / "Streaming Assistant"). Choose **USB / this PC** and **Connect**.
   - (If you don't see it on the headset, tell me — the headset is connected via
     adb so I can pull the exact app/package name.)
3. **SteamVR** should launch on the PC and show the **headset HMD active (green)**.
   If it doesn't auto-launch, start SteamVR manually from Steam.

> Use **one** streaming app only. Business Streaming is the enterprise match; if
> it misbehaves, PICO Connect 10.6.6 is an installed fallback (same SteamVR/OpenXR
> bridge) — but don't run both at once.

## Step 4 — Play

- Open `Assets/Scenes/StreamingTest.unity` → press **▶ Play**.
- Look around. You should see the room in stereo in the headset; turning your
  head looks around.

---

## Success criteria

- **Console** (`Window → General → Console`, from `XRStatusCheck`):
  `XR Enabled: True`, a non-empty `Loaded XR Device`,
  `Active Loader: OpenXRLoader`, `Input Devices Count ≥ 1`. No
  `Failed to load display/input subsystem`.
- **On-screen overlay** (top-left, from `XROriginDebug`): the **XR Camera
  Position / Rotation values change as you move your head**.
- **In the headset**: the gray room with orange obstacle boxes, in stereo,
  look-around works.

All three → streaming works → next we swap the primitive room for a light FBX,
then a real ScanNet/Matterport reconstructed mesh.

---

## Troubleshooting

| Symptom | Most likely cause / fix |
|---|---|
| SteamVR shows no headset / gray | Business Streaming not connected (Step 3). Reconnect on the headset; check USB cable; restart Business Streaming. |
| `XR Enabled: False` / `Failed to load ... subsystem` | OpenXR didn't start. Confirm Step 2 (provider = OpenXR; **Mock Runtime OFF**; ≥1 interaction profile). Make sure SteamVR is running & shows the HMD first. |
| Editor renders to monitor but **headset is black** | OpenXR active runtime isn't SteamVR. Verify in SteamVR → Settings → Developer → "Set SteamVR as OpenXR runtime" (registry already points there). |
| You see the room but it **doesn't track your head** | `<XRHMD>` not delivering pose. Check Console `Input Devices Count`; if 0, the headset isn't fully connected in SteamVR. |
| Room renders **magenta** | Harmless shader/pipeline mismatch — still proves streaming. (Tell me the render pipeline and I'll fix the material.) |
| Compile error mentioning `StreamingTestSceneBuilder` | Delete the single file `Assets/Editor/StreamingTestSceneBuilder.cs` (project returns to original) and send me the exact error. |

## After it works (not part of Milestone 1)

- Swap primitive room → light FBX → real reconstructed mesh
  (ScanNet/Matterport/etc.), aligned/scaled to the capture convention.
- **Body-data path** is a separate channel (DISPLAY vs CAPTURE): your
  XRoboToolkit 5-tracker work covers capture; this runbook covers display.
