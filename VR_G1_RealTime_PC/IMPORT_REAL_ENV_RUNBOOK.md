# Import a real reconstructed environment → streamable VR scene

Prereq: ALVR streaming already works (see `SETUP_WIFI.md`). Since the **PC** renders
and streams, heavy scanned meshes are fine — the headset GPU is no longer the limit.

## 1. Get a ready-made scanned indoor scene (fastest)

No dataset paperwork:
- **Sketchfab** (sketchfab.com): search `room scan` / `scanned interior` /
  `apartment scan`. Filter **Downloadable** (+ optionally **Free**). Open a model →
  **Download** → pick **FBX** or **OBJ** if offered (else glTF/GLB). Free login needed.
  First test: pick a full room with textures, ideally < a few hundred MB.

Bigger research datasets (sign-up / scripts, slower — do later):
- Replica (Meta), ScanNet / ScanNet++, Matterport3D, ARKitScenes.

Formats vs Unity:
- **FBX / OBJ (+ textures)** → native import, easiest. ← prefer these.
- **GLB / glTF** → install glTFast: Window → Package Manager → `+` → *Add package by
  name* → `com.unity.cloud.gltfast`.
- **PLY** (common from scanners) → convert to FBX/OBJ in Blender or MeshLab first.

## 2. Import into Unity

1. Make a folder `Assets/RealEnvs/`.
2. Unzip the model; drag the **whole folder** (mesh + textures) into `Assets/RealEnvs/`.
3. Click the model and check the preview shows textures (for OBJ you may need to
   assign the texture to its material if it looks gray).

## 3. Build the streamable scene — one click

1. In the Project window, **select the imported model** (the FBX/OBJ asset or its prefab).
2. Menu: **MTC → Build Streaming Scene From Selected Mesh**.
   - Makes `Assets/Scenes/StreamingEnv.unity`: the mesh + MeshColliders + the OpenXR
     head-tracked rig, with you dropped at the centre of the scene on its floor.

## 4. Play → walk through

1. Headset on, ALVR connected (SteamVR running).
2. Confirm `StreamingEnv` is the open scene, XR provider = **OpenXR**, and
   **Play Mode OpenXR Runtime = SteamVR/OpenXR** (the Mock-Runtime fix).
3. Press **▶ Play** → you're inside the scanned room; walk around.

## 5. If it looks wrong

| Problem | Fix |
|---|---|
| Giant / tiny | Scale the **Environment** object. LiDAR = real metres (1:1); photogrammetry/arbitrary needs scaling. |
| Floor is a wall / tilted | Rotate **Environment** (Z-up meshes often need **-90 on X**). |
| You're outside / under it | Nudge the **XR Origin** position into the room. |
| Dark / magenta / no texture | Lighting or shader/material mismatch (OBJ mats, or needs URP) — tell me, I'll fix. |
| Low FPS | PC renders it so usually fine; if needed, decimate in Blender (Decimate modifier). |

## Next: 1:1 scale + alignment (for real capture)

For obstacle-avoidance capture (physical 1 m = virtual 1 m) the mesh must be true
metric scale, floor at y=0, defined origin — that's where the verified My-project
coordinate/scale logic gets ported in. We do that once a scene streams correctly.
