# POINTCLOUD_VIZ_SURVEY.md — real-time marker point-cloud viz recon (READ-ONLY)

_2026-06-30. Reconnaissance for a PC-rendered, headset-streamed OptiTrack marker point-cloud
visualization (holographic glowing skeleton) + reuse of the existing capsule collision. **Nothing
in the project was modified.** This file sits at the project ROOT (outside `Assets/`), so Unity
does not import it and the build is untouched._

**Headline conclusion:** the project is **Built-in Render Pipeline** (constrains the glow approach
— no VFX Graph / Shader Graph / post-processing present), streaming is PC-rendered via **PICO Live
Preview / ALVR over OpenXR**, and there is **NO live OptiTrack/NatNet marker ingestion at all** —
the only body input today is PICO/ALVR-derived (24-joint UDP + 8 OSC trackers). So the raw-marker
point cloud, the NatNet stream, and the 41-marker→24-joint mapping are all **net-new**; the capsule
collision exists but exposes **no public collision signal** yet. Details + citations below.

Project root: `C:\Users\Wayne\Desktop\GMU\PhD\Humanoid\VR_G1_RealTime_PC`. Citations are relative to it.

---

## 1. Unity version + render pipeline + key packages
- **Unity 2022.3.21f1 LTS** — `ProjectSettings/ProjectVersion.txt:1`.
- **Render pipeline = Built-in** (NOT URP/HDRP): `ProjectSettings/GraphicsSettings.asset:41`
  `m_CustomRenderPipeline: {fileID: 0}`; no SRP asset in QualitySettings; no
  `com.unity.render-pipelines.*` in `Packages/packages-lock.json`.
- **Glow-relevant packages — what's NOT here (decides the approach):**
  - ❌ no **VFX Graph** (`com.unity.visualeffectgraph`) — and it requires an SRP, so not an option
    without migrating off Built-in.
  - ❌ no **Shader Graph** (`com.unity.shadergraph`) — also SRP-oriented.
  - ❌ no **Post-Processing** (neither PPv2 `com.unity.postprocessing` nor SRP volumes) → **bloom is
    not currently available**; it must be added (legacy PPv2 for Built-in) or done with a custom
    image effect, or migrate to URP.
- **Glow-relevant packages — what IS here (usable):**
  - ✅ **Burst 1.8.24**, **Mathematics 1.2.6**, **Collections 1.2.4** (transitive via glTFast) —
    available for fast per-frame marker processing / instanced draws.
  - ✅ **ParticleSystem** module (built-in) — Shuriken particles usable for a point cloud / trails.
  - ✅ `Graphics.DrawMeshInstanced(Indirect)` always available in Built-in (no package needed).
  - All `packages-lock.json` lines above.
- **XR / streaming packages:** OpenXR **1.10.0**, XR Management 4.4.0, XR Interaction Toolkit 2.5.2,
  XR Core-Utils 2.3.0, Input System 1.7.0, **PICO Live Preview** (local pkg
  `com.unity.pico.livepreview`, `manifest.json:6`). Also glTFast 6.14.1, URDF-Importer v0.5.2,
  Timeline 1.7.6, TextMeshPro 3.0.6, Visual Scripting 1.9.1.

**Glow approach implication:** stay on Built-in and do glow with **emissive materials + legacy
Post-Processing Stack v2 bloom** (lowest friction, VR-safe), or a custom fullscreen bloom; point
cloud via instanced emissive quads/meshes or particles; bones via `LineRenderer`/tube mesh; trails
via `TrailRenderer`/particles. VFX Graph would require a URP/HDRP migration of a working VR build —
flag as a bigger decision, not assumed.

## 2. PC→VR streaming stack (PC renders, headset displays)
- **Confirmed PC-rendered + streamed** (not headset-native). Transport = **PICO Live Preview**
  (`Assets/Editor/StreamingTestSceneBuilder.cs:108-118`: set XR Plug-in Management Standalone
  provider to "PICO Live Preview"… "you see the room in the headset") and/or **ALVR**
  (`Assets/Editor/StreamingEnvSceneBuilder.cs:13`; `Assets/Scripts/OscBodyReceiver.cs:7-9`).
- **What must exist in the scene to stream:** an **`XROrigin`** (Unity.XR.CoreUtils) → `Camera
  Offset` → `Main Camera` (tagged MainCamera) with a **`TrackedPoseDriver`** bound to
  `<XRHMD>/centerEyePosition`+`centerEyeRotation`, tracking-origin = Floor —
  `StreamingTestSceneBuilder.cs:67-96`. Both scene builders build this identical rig.
- **Where rendered content hooks the stream:** the stream carries *whatever the PC camera renders*
  — there is no per-object stream hook. A new viz just needs to render in the PC scene; the glow
  must therefore survive whatever the streamer captures (post-processing bloom on the PC camera is
  fine, it's part of the rendered frame).
- **Latency/framerate config:** none in-project beyond the XR rig + provider choice; framerate/MTP
  is governed by PICO Live Preview / ALVR settings (external). `TrackerProbe.cs` /
  `XRStatusCheck.cs` / `XROriginDebug.cs` are diagnostics for the XR runtime, not config.

## 3. Real-time OptiTrack / marker input  ⚠️ ABSENT
- **There is NO NatNet / OptiTrack / Motive ingestion anywhere.** Greps for
  `NatNet|OptiTrack|Motive|MarkerSet|RigidBody|unlabeled|labeledMarker` over `Assets/` → **no
  matches**; no native plugin/DLL (`Assets/Plugins` absent; no `*.dll`; no `*natnet*`).
- The only live body input today is **PICO/ALVR-derived**, two paths:
  1. **`Assets/Scripts/Network/BodyDataReceiver.cs`** — threaded **UDP on port 9876**, magic
     `0xFEEDBEEF`, 24 joints as **position+rotation already in Unity world space**
     (`:15-25,100-161`). Source = PICO `PXR_NetworkStreamer` on the headset.
  2. **`Assets/Scripts/OscBodyReceiver.cs`** — **OSC/UDP 9000**, ALVR "VRChat Body OSC", up to **8
     trackers** in play-space, exposed via `GetBodyTrackerWorldPositions()` (`:26-29,134-174`).
- **Implication:** the raw OptiTrack marker point cloud (41 CMU markers, the actual high-accuracy
  data) needs a **new NatNet→Unity stream**. Mirror the existing threaded-receiver pattern
  (`BodyDataReceiver`): background thread fills a marker array, main thread consumes. **Coordinate
  caveat:** unlike `BodyDataReceiver` (PICO already sends Unity-space), raw Motive NatNet is
  **Z-up, right-handed, mm-or-m** → must convert to Unity (Y-up, left-handed, m) in the new reader.

## 4. 24-joint→capsule collision code + what signal we can subscribe to
- **File:** `Assets/Scripts/BodyCollisionHaptics.cs`.
- **Inputs (today):** 8 **OSC trackers** from `OscBodyReceiver.GetBodyTrackerWorldPositions()` + XR
  **head** (`Camera.main`) + 2 XR **controllers**; trackers are **unlabeled**, so it **classifies by
  geometry** (height sort + lateral dot) into hip/chest/knees/feet/elbows (`:111-155`). (Note: it
  does NOT consume the 24-joint stream; `BodySkeletonRenderer` does.)
- **Capsules built (`:144-154`):** ~11 named segments — `torso, neck_head, L_thigh, R_thigh,
  L_shin, R_shin, L_uarm, R_uarm, L_farm, R_farm, head` — as `(a, b, radius)` line segments with
  per-part radii (`:40-46`); they are **logical capsules, not `CapsuleCollider` components**.
- **Collision detection (`:181-206`):** per capsule, **`Physics.OverlapCapsuleNonAlloc`** against
  scene **`MeshCollider`s** (env set in `RefreshEnv`, `:83-98`; floor excluded). Body-vs-environment
  only — **no self-collision** queried.
- **⚠️ What is EXPOSED for the viz:** **nothing public.** Hits live in a local `hitNames` list and a
  private `_status` string (`:184,193,204`); reaction is controller haptics (`:208-214`) + private
  visual tint (`TintVisual`). There is **no public event, no per-part bool/enum, no API**. → To
  drive collision color/glow per body part we must **add a public per-capsule hit signal** (e.g.
  `event Action<string,bool>` or a `Dictionary<string,bool>` snapshot) — a small additive change at
  build time. The good news: the per-part hit info already exists internally and is keyed by the
  capsule names above, so the signal we want is one accessor away.

## 5. Marker ↔ joint mapping (41 CMU → 24 joints)  ⚠️ ABSENT
- **No CMU-41 layout and no marker→joint mapping exist.** The project only has:
  - the **24-joint PICO skeleton** in `BodySkeletonRenderer.cs:42-49` (standard **SMPL order**,
    `HEAD=15`), consumed from `BodyDataReceiver`; and
  - the **8-tracker geometric classification** in `BodyCollisionHaptics.cs:111-155`.
- Neither maps 41 labeled CMU markers to anything. So **"41 markers → 24 capsule anchors" is
  net-new.** Two feasible designs for later: (a) map specific CMU labels → the 11 capsule endpoints
  directly (cleanest, since CMU markers are labeled, unlike the 8 ambiguous OSC trackers); or
  (b) reconstruct ~24 joint centers from marker groups, then feed the existing capsule builder.
  Either way the capsule **names/topology to target are the 11 listed in Q4**.

## 6. Environment / obstacle representation
- Obstacles are an **imported environment mesh** (e.g. `Assets/RealEnvs/apartment_1.fbx`)
  instantiated as "Environment" by `Assets/Editor/StreamingEnvSceneBuilder.cs:53-60`, which **adds a
  `MeshCollider` to every mesh part** (`:7-9` "adds MeshColliders to every mesh part") so you can
  walk/collide.
- Floor is split out at the collider level by **`Assets/Editor/FloorColliderSplitter.cs`** (referenced
  in `BodyCollisionHaptics.cs:13-17`) so feet-on-floor never counts as a collision.
- The whole environment root is uniformly **scaled by `RealEnvEmbodiment.cs:26-66`** (embodiment:
  scale world by 1/scale so a real person feels G1-height) — any obstacle highlight must live under,
  and inherit, this scaled `Environment` root.
- **Alignment guarantee for highlighting:** collision uses these **same MeshColliders/renderers**
  (`BodyCollisionHaptics.RefreshEnv` gathers `MeshCollider`s, `:83-98`), so adding a Fresnel/rim-glow
  material to the obstacle `MeshRenderer`s will line up exactly with the collision geometry. (Glow
  again constrained by Built-in RP — emissive/rim shader, bloom via added PPv2.)

## 7. Existing visualization of markers
- **GameObject-primitive based only — no particles/instancing/point-cloud today.**
  - `Assets/Scripts/BodySkeletonRenderer.cs` renders the 24 joints as **primitive `Sphere`
    GameObjects** (`:64-74`) + **`LineRenderer` bones** (`:76-88`), with real meshes "so it is
    visible in the headset over ALVR" (`:5-9`); exposes `WorldJoints[24]` (`:39`).
  - `OscBodyReceiver.cs:176-187` spawns one **`Sphere` per OSC tracker**.
  - `BodyCollisionHaptics.cs:233-243` spawns **`Capsule` primitives** as collision visuals.
- Grep for `ParticleSystem|PointCloud|MeshTopology.Points|DrawMeshInstanced|Graphics.DrawMesh` over
  `Assets/` → **no matches**. So the "cool point cloud" is a clean-slate render: the existing
  sphere/line approach is the proven, stream-visible baseline to upgrade (emissive instanced nodes,
  tube bones, trails) rather than reuse as-is.

---

## Integration seams (where the new viz plugs in)
1. **Live marker stream ENTRY — must be ADDED.** No NatNet today. New `NatNetMarkerReceiver`
   (mirror `BodyDataReceiver`'s threaded-UDP→array→main-thread pattern, `BodyDataReceiver.cs:50-98`)
   producing per-frame `Vector3[] markersWorld` (N≈41). **Do the OptiTrack Z-up/RH/mm → Unity
   Y-up/LH/m conversion here** (unlike the PICO path which is pre-converted). This array is the
   point-cloud render source AND the capsule-anchor source.
2. **Collision SIGNAL source — exists internally, must be EXPOSED.** Add a public per-capsule hit
   readout to `BodyCollisionHaptics` (keys = the 11 capsule names, Q4). The viz subscribes to color
   nodes/bones/obstacles on hit. Also note the collision code currently eats 8 OSC trackers — to
   drive it from markers we feed capsule anchors from the new marker→anchor map (Q5).
3. **Render-pipeline CONSTRAINT.** Built-in RP → glow = emissive materials + **added** PPv2 bloom
   (or custom image effect); point cloud = instanced emissive meshes / particles; bones =
   LineRenderer/tube; trails = TrailRenderer. **VFX Graph/Shader Graph are unavailable without an
   SRP migration** — a separate decision if we want their authoring power. Bloom is a fullscreen
   PC-camera effect, captured by the stream, cost is stable (fits the "latency-stable, compute-not-
   bottleneck" constraint).
4. **Scene rig.** The viz renders under the existing `XROrigin` camera (Q2) and, for obstacle
   highlights, under the scaled `Environment` root (Q6); the marker cloud should anchor in the same
   world space the OptiTrack→Unity conversion targets (align it with the XR floor origin).

## What's net-new vs reusable (summary)
| Need | Status |
|---|---|
| PC→VR streaming, XR rig | ✅ reuse (PICO Live Preview/ALVR, XROrigin+TrackedPoseDriver) |
| Live OptiTrack marker stream | ❌ build new (NatNet→Unity, Z-up/RH/mm→Unity convert) |
| 41-marker → 24/anchor mapping | ❌ build new (no mapping exists) |
| Capsule collision mechanics | ✅ reuse `BodyCollisionHaptics` (OverlapCapsule + 11 named capsules) |
| Per-part collision signal for viz | ⚠️ add public event/accessor (exists internally only) |
| Obstacle geometry for highlight | ✅ reuse env MeshColliders/renderers (aligned with collision) |
| Glow/point-cloud rendering | ❌ build new; Built-in RP → emissive + add PPv2 bloom (no VFX Graph) |

**STOP — survey only. No project files modified, nothing added to the build.**
