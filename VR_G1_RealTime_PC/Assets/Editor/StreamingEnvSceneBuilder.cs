// StreamingEnvSceneBuilder.cs
// -----------------------------------------------------------------------------
// Turns ANY imported environment mesh (a scanned room FBX/OBJ/GLB-as-prefab, or a
// reconstructed mesh) into a ready-to-stream VR scene in one click:
//   * instantiates the selected model,
//   * adds MeshColliders to every mesh part (so you can walk / collide),
//   * builds the same OpenXR head-tracked XR rig as StreamingTest,
//   * drops the "player" at the horizontal centre of the mesh, on its floor,
//   * saves Assets/Scenes/StreamingEnv.unity.
//
// Usage: in the Project window select the imported model (the FBX/OBJ asset or its
// prefab), then run  MTC > Build Streaming Scene From Selected Mesh.  Press Play
// (headset on, ALVR connected) to walk through it.
//
// Additive + self-contained. Does NOT touch My project or SampleScene/StreamingTest.
// If it ever fails to compile, delete this one file.
// -----------------------------------------------------------------------------
#if UNITY_EDITOR
using Unity.XR.CoreUtils;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.XR;

public static class StreamingEnvSceneBuilder
{
    const string ScenePath = "Assets/Scenes/StreamingEnv.unity";

    [MenuItem("MTC/Build Streaming Scene From Selected Mesh")]
    public static void Build()
    {
        GameObject sel = Selection.activeGameObject;
        if (sel == null) sel = Selection.activeObject as GameObject;
        if (sel == null)
        {
            EditorUtility.DisplayDialog("Select a mesh first",
                "In the Project window, click the imported environment model/prefab " +
                "(the FBX/OBJ asset, or a prefab), then run this menu again.", "OK");
            return;
        }

        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        // --- Light + ambient (scanned meshes under lit shaders can render dark) ---
        var lightGO = new GameObject("Directional Light");
        var light = lightGO.AddComponent<Light>();
        light.type = LightType.Directional; light.intensity = 1.0f;
        lightGO.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.5f, 0.5f, 0.5f);

        // --- Instantiate the environment ---
        GameObject env = PrefabUtility.InstantiatePrefab(sel) as GameObject;
        if (env == null) env = Object.Instantiate(sel);
        env.name = "Environment";
        env.transform.position = Vector3.zero;
        env.transform.rotation = Quaternion.identity;

        // --- Auto-orient up: make the smallest-extent axis vertical (Y) ---
        // Rooms have ceiling-height as their smallest dimension, so this lifts a
        // Z-up glTF/GLB (e.g. blendergltf exports import lying on their side) upright.
        // No-op for already-upright (Y-up) FBX imports.
        {
            Bounds ob = new Bounds(); bool obHas = false;
            foreach (var r in env.GetComponentsInChildren<Renderer>())
            { if (!obHas) { ob = r.bounds; obHas = true; } else ob.Encapsulate(r.bounds); }
            if (obHas)
            {
                Vector3 os = ob.size;
                if (!(os.y <= os.x && os.y <= os.z))   // Y is NOT the smallest -> needs reorienting
                {
                    if (os.z <= os.x)
                    { env.transform.Rotate(-90f, 0f, 0f, Space.World);
                      Debug.Log($"[StreamingEnvSceneBuilder] Z-up mesh ({os.x:F1}x{os.y:F1}x{os.z:F1} m) -> rotated -90° X to stand upright."); }
                    else
                    { env.transform.Rotate(0f, 0f, 90f, Space.World);
                      Debug.Log($"[StreamingEnvSceneBuilder] X-up mesh ({os.x:F1}x{os.y:F1}x{os.z:F1} m) -> rotated +90° Z to stand upright."); }
                }
            }
        }

        // --- MeshColliders on every mesh part ---
        int colliders = 0;
        foreach (var mf in env.GetComponentsInChildren<MeshFilter>())
        {
            if (mf.sharedMesh == null) continue;
            var mc = mf.GetComponent<MeshCollider>();
            if (mc == null) mc = mf.gameObject.AddComponent<MeshCollider>();
            mc.sharedMesh = mf.sharedMesh;
            colliders++;
        }

        // Remove the floor surface from the COLLISION meshes (render meshes untouched) so
        // standing/feet never count as a collision, while walls + furniture (incl. low
        // furniture sitting on the floor) still do.
        FloorColliderSplitter.Split(env, 0.08f);

        // --- World-space bounds (to drop the player inside) ---
        bool hasBounds = false;
        Bounds b = new Bounds(Vector3.zero, Vector3.zero);
        foreach (var r in env.GetComponentsInChildren<Renderer>())
        {
            if (!hasBounds) { b = r.bounds; hasBounds = true; }
            else b.Encapsulate(r.bounds);
        }

        // --- OpenXR head-tracked rig (same as StreamingTest) ---
        var originGO = new GameObject("XR Origin");
        var xrOrigin = originGO.AddComponent<XROrigin>();
        var offsetGO = new GameObject("Camera Offset");
        offsetGO.transform.SetParent(originGO.transform, false);
        var camGO = new GameObject("Main Camera");
        camGO.tag = "MainCamera";
        camGO.transform.SetParent(offsetGO.transform, false);
        var cam = camGO.AddComponent<Camera>();
        cam.nearClipPlane = 0.05f;
        cam.farClipPlane = 1000f; // scanned scenes can be large
        camGO.AddComponent<AudioListener>();

        var tpd = camGO.AddComponent<TrackedPoseDriver>();
        tpd.trackingType = TrackedPoseDriver.TrackingType.RotationAndPosition;
        tpd.updateType   = TrackedPoseDriver.UpdateType.UpdateAndBeforeRender;
        tpd.positionInput = new InputActionProperty(
            new InputAction("HMD Position", InputActionType.Value, "<XRHMD>/centerEyePosition", null, null, "Vector3"));
        tpd.rotationInput = new InputActionProperty(
            new InputAction("HMD Rotation", InputActionType.Value, "<XRHMD>/centerEyeRotation", null, null, "Quaternion"));

        xrOrigin.Camera = cam;
        xrOrigin.CameraFloorOffsetObject = offsetGO;
        xrOrigin.RequestedTrackingOriginMode = XROrigin.TrackingOriginMode.Floor;

        // Stand at the horizontal centre of the scene, on its lowest point (≈ floor).
        if (hasBounds)
            originGO.transform.position = new Vector3(b.center.x, b.min.y, b.center.z);

        // --- Diagnostics overlay ---
        var diag = new GameObject("Diagnostics");
        diag.AddComponent<XRStatusCheck>();
        var od = diag.AddComponent<XROriginDebug>(); od.xrCamera = cam;

        // --- Embodiment scaling (ported from My project's PXR_SessionAutoFlow) ---
        // After a short standing-height calibration at Play, scales the Environment
        // UP by 1/scale around the player's feet, so a real ~1.7 m person perceives
        // the scanned room at Unitree G1 (1.27 m) scale ("you feel short").
        // Assumes the mesh is true metric scale (Replica/ScanNet/LiDAR = 1:1).
        var embGO = new GameObject("Embodiment");
        var emb = embGO.AddComponent<RealEnvEmbodiment>();
        emb.hmd = cam.transform;
        emb.worldRoot = env.transform;
        emb.floorRef = originGO.transform;
        emb.targetHeightMeters = 1.27f;

        // --- G1 height reference (1.27 m) ---
        // A proportional G1-sized figure as a visible height reference. Parented under
        // the Environment so it scales WITH the embodiment (stays 1.27 m relative to the
        // room) -> after calibration it should look ~your height, confirming G1 scale.
        if (hasBounds)
        {
            Vector3 floorCentre = new Vector3(b.center.x, b.min.y, b.center.z);
            Vector3 g1Pos = floorCentre + new Vector3(0f, 0f, 2.0f);   // ~2 m from spawn
            G1ReferenceBuilder.Build(g1Pos, Quaternion.Euler(0f, 180f, 0f), env.transform, true);
        }

        // --- Live 24-joint body: custom UDP receiver + skeleton renderer ---
        // BodyDataReceiver listens on UDP 9876 for the PICO PXR_NetworkStreamer.
        // BodySkeletonRenderer draws the joints anchored to the XR head pose.
        var bodyNet = new GameObject("BodyNetwork");
        var recv = bodyNet.AddComponent<BodyDataReceiver>();
        var bodyGO = new GameObject("BodySkeleton");
        var skel = bodyGO.AddComponent<BodySkeletonRenderer>();
        skel.receiver = recv;
        skel.hmd = cam.transform;

        EditorSceneManager.SaveScene(scene, ScenePath);
        AssetDatabase.Refresh();

        string sizeStr = hasBounds ? $"{b.size.x:F1} x {b.size.y:F1} x {b.size.z:F1} m" : "unknown";
        Debug.Log($"[StreamingEnvSceneBuilder] Built {ScenePath} from '{sel.name}': " +
                  $"{colliders} MeshColliders, scene size {sizeStr}. Press Play.");
        EditorUtility.DisplayDialog("Streaming Env scene built",
            $"Built {ScenePath} from '{sel.name}'.\n\n" +
            $"MeshColliders added: {colliders}\n" +
            $"Scene size: ~{sizeStr}  (player dropped at floor centre)\n" +
            "Embodiment: G1 1.27 m scaling enabled — stand still ~5 s at Play to calibrate.\n\n" +
            "Press Play (headset on / ALVR connected) to walk through it.\n\n" +
            "If it looks wrong:\n" +
            "  • giant or tiny  -> scale the 'Environment' object (LiDAR is 1:1; photogrammetry isn't)\n" +
            "  • floor is a wall -> rotate 'Environment' (often -90 on X for Z-up meshes)\n" +
            "  • you're outside it -> nudge the 'XR Origin' position into the room",
            "OK");
    }
}
#endif
