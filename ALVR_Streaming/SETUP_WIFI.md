# ALVR WiFi streaming — setup & run (PICO 4 Ultra Enterprise)

Open-source PCVR streaming that **renders your Unity scene from the headset's live
6DoF pose on the PC and streams it to the headset** (true immersive VR, not flat
video). Bypasses PICO Business Streaming entirely.

**Self-contained folder — does not touch any existing code.**
Binaries are gitignored.

## What's already done (by Claude, 2026-06-17)

- ALVR **v20.14.1** downloaded here:
  - `pc_streamer/streamer/ALVR Dashboard.exe`  ← run this on the PC
  - `headset_apk/alvr_client_android.apk`       ← already **sideloaded to the headset**
  - `pc_streamer/alvr_launcher_windows.zip`     ← alternative entry point (not needed)
- The client APK is **installed on the PICO** (adb `Success`).
- SteamVR is already installed on this PC (ALVR needs it).

So all that's left is GUI + headset steps (Claude can't drive VR hardware).

---

## One-time network requirement (the thing that matters for WiFi)

- PC and headset must be on the **same network with NO client/AP isolation**.
  - ✅ home router / a dedicated travel router / phone hotspot
  - ❌ **GMU campus WiFi / eduroam** (these isolate clients → headset & PC can't see each other)
- Use **5 GHz** (not 2.4 GHz). WiFi 6/6E is ideal.
- **Best latency: PC on Ethernet (wired) + headset on 5 GHz**, same router. PC-on-WiFi
  also works, just a bit more latency.

---

## Step 1 — PC: launch ALVR

1. Run **`pc_streamer\streamer\ALVR Dashboard.exe`**.
2. First run shows a short setup wizard → **allow the Windows Firewall prompt**
   (Private networks at least). It registers the SteamVR driver.
3. **SteamVR first-launch gotcha** (from ALVR docs): the very first time, SteamVR may
   come up **blank and ALVR won't connect** — just **close SteamVR + ALVR and start
   ALVR again**. Second launch is fine.

## Step 2 — Headset: launch the ALVR app

1. On the PICO, open the **Library → look under "All apps" / "Unknown sources"** and
   launch **ALVR** (it's sideloaded, so it may not be on the main shelf).
2. It shows a **"Waiting for connection"** screen with the headset name + its IP address.

## Step 3 — Connect

1. Back on the PC ALVR Dashboard → **Devices** tab → your headset should appear →
   click **Trust / Connect**.
2. If it does **not** auto-appear: in the Dashboard add it by the **IP** shown on the
   headset (Devices → add manually). (Auto-discovery uses LAN broadcast, which some
   networks block; by-IP always works if they're on the same subnet.)
3. On connect, SteamVR fully starts and you land in the **SteamVR void/home inside the
   headset**, head-tracked. ✅ That alone proves streaming works.

## Step 4 — Render our scene through it

1. Open **`VR_G1_RealTime_PC`** in Unity 2022.3.21f1.
2. Project Settings → XR Plug-in Management → **Standalone = OpenXR** (SteamVR is the
   active OpenXR runtime; that's what ALVR feeds).
3. Open `Assets/Scenes/StreamingTest.unity` → press **Play**.
4. You should see the room in the headset and be able to **walk around it** (room-scale
   6DoF), with correct depth/parallax — the PC re-renders from your moving viewpoint.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Headset not in Dashboard "Devices" | Firewall (allow ALVR), same network, **no campus WiFi/AP isolation**; add by **IP** manually. |
| SteamVR opens blank, ALVR won't connect | Known first-launch quirk — close SteamVR+ALVR, relaunch ALVR. |
| Connected but laggy/stutter | Use 5 GHz; wire the PC; lower bitrate in ALVR Settings (e.g. 80–120 Mbps); set encoder to your GPU (NVENC). |
| ALVR app won't launch on headset | MDM may be blocking sideloaded apps — check enterprise policy (you're likely admin). |
| Headset image but no head tracking | You're not actually in the ALVR/SteamVR session — confirm SteamVR shows the HMD active. |

## Notes

- Streamer and client are both **v20.14.1** (they must match). To update later:
  re-download both from the same release and re-run `adb install -r headset_apk\alvr_client_android.apk`.
- USB is **not** needed for streaming — it was only for the one-time sideload.
- ALVR can also do USB streaming (adb-tunnel) later if you ever want lower latency.
- Body-tracking capture is a **separate** channel (your XRoboToolkit work); this is the
  display side only.
