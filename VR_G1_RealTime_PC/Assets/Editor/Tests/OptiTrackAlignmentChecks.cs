#if UNITY_EDITOR
using System;
using System.Globalization;
using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEngine;
using Object = UnityEngine.Object;

// Exercises the receiver itself using synthetic poses, without opening UDP ports
// or changing the active XR rig. Run from the menu or Unity -executeMethod.
[InitializeOnLoad]
public static class OptiTrackAlignmentChecks
{
    const string RequestPath = "Logs/RunOptiTrackAlignmentChecks";
    const string ReportPath = "Logs/OptiTrackAlignmentChecks.txt";
    const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
    static readonly Type ReceiverType = typeof(OptiTrackPointReceiver);

    static OptiTrackAlignmentChecks()
    {
        EditorApplication.delayCall += () => { if (File.Exists(RequestPath)) Run(); };
    }

    [MenuItem("MTC/Run Mocap Alignment Checks")]
    public static void Run()
    {
        var root = new GameObject("Mocap Alignment Checks") { hideFlags = HideFlags.HideAndDontSave };
        root.SetActive(false); // Prevent Start/network/haptics in Edit or Play mode.
        try
        {
            var receiver = root.AddComponent<OptiTrackPointReceiver>();
            var origin = new GameObject("Test Origin").transform;
            origin.SetParent(root.transform, false);
            origin.position = new Vector3(2, 0, -1);
            var camera = new GameObject("Test Camera").transform;
            camera.SetParent(origin, false);
            camera.position = new Vector3(7, 1, -3);
            Set(receiver, "vrCamera", camera);
            Set(receiver, "xrOrigin", origin);
            Set(receiver, "convertFromMotiveZUp", false);
            Set(receiver, "motiveLocalForwardAxis", Vector3.forward);
            Set(receiver, "motiveLocalUpAxis", Vector3.up);
            Set(receiver, "positionCorrectionRate", 0.25f); // Actual scene setting.
            Set(receiver, "maximumCorrectionSpeedMetersPerSecond", 0.25f); // Legacy saved cap.
            Require(!(bool)Get(receiver, "limitPositionCorrectionSpeed"), "New receivers must allow unrestricted catch-up.");
            var originalOrigin = origin.position;
            var originalCamera = camera.position;
            var originalRotation = camera.rotation;

            // Valid positions must recover even when a simultaneous 180-degree rotation
            // is rejected; a jump must not permanently latch the last accepted position.
            Call(receiver, "TryAcceptMotivePose", Frame(new Vector3(0, 1, 0), false));
            Call(receiver, "TryAcceptMotivePose", Frame(new Vector3(1000, 1, 0), true));
            Require(Vector3.Distance((Vector3)Get(receiver, "acceptedMotiveCameraPosition"), new Vector3(1000, 1, 0)) < 0.001f,
                "A far-away position was blocked by rotation rejection.");
            Require(Quaternion.Angle((Quaternion)Get(receiver, "acceptedMotiveCameraRotation"), Quaternion.identity) < 0.01f,
                "The rotation outlier should still be filtered.");
            Set(receiver, "acceptedPoseArrivalTime", float.NegativeInfinity);
            Call(receiver, "TryAcceptMotivePose", Frame(new Vector3(-1000, 1, 0), true));
            Require(Quaternion.Angle((Quaternion)Get(receiver, "acceptedMotiveCameraRotation"), Quaternion.Euler(0, 180, 0)) < 0.01f,
                "Rotation must reacquire after tracking loss.");

            // Test both sides of the player, near/far, at the slow scene smoothing rate.
            foreach (float offset in new[] { -1000f, -5f, 0.01f, 5f, 1000f })
            {
                Set(receiver, "hasPointAlignment", true);
                Set(receiver, "mocapToUnityRotation", Quaternion.identity);
                Set(receiver, "mocapToUnityTranslation", Vector3.zero);
                Set(receiver, "acceptedMotiveCameraRotation", Quaternion.identity);
                Set(receiver, "acceptedMotiveCameraPosition", camera.position + Vector3.right * offset);
                float before = Mathf.Abs(offset);
                for (int step = 0; step < 3000; step++)
                {
                    Call(receiver, "UpdatePointAlignment", 0.02f);
                    float error = Mathf.Abs(offset + ((Vector3)Get(receiver, "mocapToUnityTranslation")).x);
                    Require(error <= before + 0.0001f, "Correction moved away from the player.");
                    if (step == 0) Require(error < before && error > 0, "Correction must drift smoothly without freezing or snapping.");
                    before = error;
                }
                Require(before < 0.02f, "Large-offset correction did not converge: " + offset);
            }

            Set(receiver, "acceptedMotiveCameraPosition", camera.position + Vector3.right * 100);
            Set(receiver, "mocapToUnityTranslation", Vector3.zero);
            Set(receiver, "limitPositionCorrectionSpeed", true);
            Call(receiver, "UpdatePointAlignment", 0.02f);
            Require(((Vector3)Get(receiver, "mocapToUnityTranslation")).magnitude <= 0.0051f, "Optional speed limit is not respected.");
            Set(receiver, "mocapToUnityTranslation", Vector3.zero);
            Set(receiver, "maximumCorrectionSpeedMetersPerSecond", 0f);
            Call(receiver, "UpdatePointAlignment", 0.02f);
            Require(((Vector3)Get(receiver, "mocapToUnityTranslation")).magnitude > 0.0051f, "Zero speed limit must mean unlimited, not frozen.");

            var badFrame = Frame(Vector3.zero, false);
            badFrame.GetType().GetField("picoGoodQuality").SetValue(badFrame, false);
            Require(!(bool)Call(receiver, "PosePassesQualityChecks", badFrame), "Tracking-quality rejection was bypassed.");
            Require(origin.position == originalOrigin && camera.position == originalCamera && camera.rotation == originalRotation,
                "Marker alignment must not move the XR rig or camera.");
            // A first sample must appear at its aligned position before any smoothing
            // update. Subsequent samples still smooth; reacquired markers snap too.
            var markerPrefab = new GameObject("Test Marker Prefab");
            markerPrefab.transform.SetParent(root.transform, false);
            Set(receiver, "pointPrefab", markerPrefab);
            Set(receiver, "pointParent", root.transform);
            Set(receiver, "hasPointAlignment", true);
            var alignmentRotation = Quaternion.Euler(0, 90, 0);
            var alignmentTranslation = new Vector3(4, 0, 5);
            Set(receiver, "mocapToUnityRotation", alignmentRotation);
            Set(receiver, "mocapToUnityTranslation", alignmentTranslation);
            Call(receiver, "DisplayPoints", MarkerFrame(new Vector3(500, 1, 2)));
            var visuals = (System.Collections.IDictionary)Get(receiver, "pointVisuals");
            var visual = visuals[7];
            var markerObject = (GameObject)visual.GetType().GetField("gameObject").GetValue(visual);
            var firstPosition = alignmentRotation * new Vector3(500, 1, 2) + alignmentTranslation;
            Require(Vector3.Distance(markerObject.transform.position, firstPosition) < 0.001f,
                "New marker flew in from the parent origin instead of spawning at its measured position.");
            Call(receiver, "DisplayPoints", MarkerFrame(new Vector3(510, 1, 2)));
            Require(Vector3.Distance(markerObject.transform.position, firstPosition) < 0.001f,
                "Ordinary marker updates must retain their configured smoothing.");
            markerObject.SetActive(false);
            Call(receiver, "DisplayPoints", MarkerFrame(new Vector3(520, 1, 2)));
            Require(Vector3.Distance(markerObject.transform.position,
                alignmentRotation * new Vector3(520, 1, 2) + alignmentTranslation) < 0.001f,
                "Reacquired marker did not appear at its latest measured position.");
            Object.DestroyImmediate(markerObject);
            visuals.Clear();
            // Script reloads/disable-enable cycles must release and restore a receiver.
            // Use an ephemeral local port, never the running capture session's port 5005.
            Set(receiver, "listenPort", 0);
            for (int cycle = 0; cycle < 2; cycle++)
            {
                Call(receiver, "StartReceiver");
                Require(Get(receiver, "udpClient") != null && (bool)Get(receiver, "running"), "Receiver failed to restart.");
                Call(receiver, "OnDisable");
                Require(Get(receiver, "udpClient") == null && Get(receiver, "receiveThread") == null && !(bool)Get(receiver, "running"),
                    "Receiver socket/thread was not released.");
            }
            string report = "PASS: position updates survive rotation rejection; tracking reacquires; " +
                "smooth convergence from +/-5 m and +/-1000 m; optional/zero speed limits; " +
                "tracking quality and XR camera pose preserved; receiver socket/thread restart cycles; " +
                "instant first/reacquired marker placement with subsequent smoothing preserved.";
            File.WriteAllText(ReportPath, report);
            Debug.Log("[MocapAlignmentChecks] " + report);
        }
        catch (Exception error)
        {
            File.WriteAllText(ReportPath, "FAIL: " + error);
            throw;
        }
        finally
        {
            Object.DestroyImmediate(root);
            if (File.Exists(RequestPath)) File.Delete(RequestPath);
        }
    }

    static object Frame(Vector3 position, bool rotated)
    {
        string packet = string.Format(CultureInfo.InvariantCulture, "1;0;PICO;1;1;0.001;{0},{1},{2};{3}",
            position.x, position.y, position.z, rotated ? "0,1,0,0" : "0,0,0,1");
        return ReceiverType.GetMethod("ParseFrame", BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, new object[] { packet });
    }
    static object MarkerFrame(Vector3 position)
    {
        string packet = string.Format(CultureInfo.InvariantCulture, "2;1;7,{0},{1},{2}", position.x, position.y, position.z);
        return ReceiverType.GetMethod("ParseFrame", BindingFlags.Static | BindingFlags.NonPublic).Invoke(null, new object[] { packet });
    }
    static object Call(object target, string name, params object[] args) => ReceiverType.GetMethod(name, Private).Invoke(target, args);
    static object Get(object target, string name) => ReceiverType.GetField(name, Private).GetValue(target);
    static void Set(object target, string name, object value) => ReceiverType.GetField(name, Private).SetValue(target, value);
    static void Require(bool condition, string message) { if (!condition) throw new Exception(message); }
}
#endif
