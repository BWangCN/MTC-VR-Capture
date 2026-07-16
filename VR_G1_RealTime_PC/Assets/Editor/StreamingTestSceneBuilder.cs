// StreamingTestSceneBuilder.cs
// -----------------------------------------------------------------------------
// Builds a brand-new MINIMAL scene whose ONLY purpose is to validate the
// PC -> PICO "Live Preview" streaming path in isolation:
//   * no G1 robot, no UDP networking, no Python retarget -- nothing that could
//     confound "is the headset showing the PC-rendered, head-tracked view?".
//   * just an XR Origin (head-tracked camera) + a primitive room + a light
//     + the project's existing on-screen XR diagnostics.
//
// Menu:  MTC > Build Streaming Test Scene   ->  creates Assets/Scenes/StreamingTest.unity
//
// This file is ADDITIVE and self-contained. It does NOT modify SampleScene or
// any existing script. If it ever fails to compile, just delete THIS one file
// and the project is back to exactly how it was.
//
// Milestone 1 run steps live in ../STREAMING_TEST_RUNBOOK.md (project root).
// -----------------------------------------------------------------------------
#if UNITY_EDITOR
using Unity.XR.CoreUtils;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.InputSystem;
using UnityEngine.InputSystem.XR;

public static class StreamingTestSceneBuilder
{
    const string ScenePath = "Assets/Scenes/StreamingTest.unity";

    [MenuItem("MTC/Build Streaming Test Scene")]
    public static void Build()
    {
        // Fresh empty scene (Unity will offer to save the current one first).
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        // ---- Lighting ----
        var lightGO = new GameObject("Directional Light");
        var light = lightGO.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 1.1f;
        lightGO.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

        // ---- A simple, unmistakable "room": floor + 4 walls + a few obstacles ----
        // Obstacles give strong depth/parallax cues so head motion is obvious.
        var floorMat = MakeMat(new Color(0.55f, 0.55f, 0.58f));
        var wallMat  = MakeMat(new Color(0.70f, 0.72f, 0.78f));
        var boxMat   = MakeMat(new Color(0.85f, 0.45f, 0.30f));

        var room = new GameObject("Room").transform;

        var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.name = "Floor";
        floor.transform.SetParent(room, false);
        floor.transform.localScale = new Vector3(0.8f, 1f, 0.8f); // Plane is 10m; 0.8 -> 8m x 8m
        SetMat(floor, floorMat);

        CreateBox(room, "Wall_N", new Vector3(0f, 1.25f,  4f), new Vector3(8f,   2.5f, 0.2f), wallMat);
        CreateBox(room, "Wall_S", new Vector3(0f, 1.25f, -4f), new Vector3(8f,   2.5f, 0.2f), wallMat);
        CreateBox(room, "Wall_E", new Vector3(4f, 1.25f,  0f), new Vector3(0.2f, 2.5f, 8f),   wallMat);
        CreateBox(room, "Wall_W", new Vector3(-4f,1.25f,  0f), new Vector3(0.2f, 2.5f, 8f),   wallMat);

        CreateBox(room, "Box_A",  new Vector3(1.5f, 0.5f,  2.0f), new Vector3(1f,   1f,   1f),  boxMat);
        CreateBox(room, "Box_B",  new Vector3(-2.0f,0.35f, 1.0f), new Vector3(0.7f, 0.7f, 0.7f),boxMat);
        CreateBox(room, "Pillar", new Vector3(-1.0f,1.0f, -2.5f), new Vector3(0.4f, 2.0f, 0.4f),boxMat);

        // ---- XR rig: XR Origin + head-tracked camera ----
        var originGO = new GameObject("XR Origin");
        var xrOrigin = originGO.AddComponent<XROrigin>();

        var offsetGO = new GameObject("Camera Offset");
        offsetGO.transform.SetParent(originGO.transform, false);

        var camGO = new GameObject("Main Camera");
        camGO.tag = "MainCamera";
        camGO.transform.SetParent(offsetGO.transform, false);
        var cam = camGO.AddComponent<Camera>();
        cam.nearClipPlane = 0.05f;
        camGO.AddComponent<AudioListener>();

        // TrackedPoseDriver (Input System) drives the camera from the HMD pose
        // delivered by the Live Preview input subsystem.
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

        // Stand near the south wall, facing +Z into the room.
        originGO.transform.position = new Vector3(0f, 0f, -3f);

        // ---- On-screen diagnostics (reuse the project's existing debug scripts) ----
        var diag = new GameObject("Diagnostics");
        diag.AddComponent<XRStatusCheck>();          // shows "XR Enabled / Device / Active Loader"
        var od = diag.AddComponent<XROriginDebug>();  // shows live XR camera pose
        od.xrCamera = cam;

        // ---- Save ----
        EditorSceneManager.SaveScene(scene, ScenePath);
        AssetDatabase.Refresh();

        Debug.Log($"[StreamingTestSceneBuilder] Created {ScenePath}. Next: set XR Plug-in Management " +
                  "> Standalone provider to 'PICO Live Preview', connect the headset, then press Play.");
        EditorUtility.DisplayDialog("Streaming Test Scene created",
            "Created:\n" + ScenePath + "\n\n" +
            "NEXT (see STREAMING_TEST_RUNBOOK.md):\n" +
            "1) Project Settings > XR Plug-in Management > Standalone tab >\n" +
            "   check 'PICO Live Preview'  (uncheck OpenXR while previewing).\n" +
            "2) Start the PICO Live Preview app on the headset (PC + headset on same network/USB).\n" +
            "3) Open " + ScenePath + " and press Play.\n\n" +
            "SUCCESS = you see the room in the headset and can look around.",
            "OK");
    }

    // Pipeline-agnostic material (works under Built-in or URP; falls back gracefully).
    static Material MakeMat(Color c)
    {
        Shader s = null;
        if (GraphicsSettings.currentRenderPipeline != null)
            s = Shader.Find("Universal Render Pipeline/Lit");
        if (s == null) s = Shader.Find("Standard");
        if (s == null) s = Shader.Find("Sprites/Default");
        var m = new Material(s);
        m.color = c;
        return m;
    }

    static void SetMat(GameObject go, Material m)
    {
        var r = go.GetComponent<Renderer>();
        if (r != null) r.sharedMaterial = m;
    }

    static void CreateBox(Transform parent, string name, Vector3 pos, Vector3 scale, Material m)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        go.transform.SetParent(parent, false);
        go.transform.localPosition = pos;
        go.transform.localScale = scale;
        SetMat(go, m);
    }
}
#endif
