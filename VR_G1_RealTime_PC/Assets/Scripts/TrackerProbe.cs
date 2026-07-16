// TrackerProbe.cs
// -----------------------------------------------------------------------------
// DIAGNOSTIC. Enumerates every XR input device Unity can see over the active
// runtime (here: OpenXR -> SteamVR -> ALVR) and logs name + characteristics +
// live pose, every couple seconds, to the PC Console AND an on-screen list.
//
// Purpose: once PICO body tracking is enabled in ALVR, run this during streaming to
// discover WHAT the PC side actually exposes (HMD, 2 controllers, and hopefully a set
// of body TRACKERS). The names/characteristics tell us how to wire the body reader so
// we target the right devices instead of guessing.
//
// Add via menu  MTC > Add XR Tracker Probe to Scene  (or drop on any GameObject).
// -----------------------------------------------------------------------------
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

public class TrackerProbe : MonoBehaviour
{
    [Tooltip("How often to re-enumerate + log (seconds).")]
    public float logEverySeconds = 2f;
    public bool showOnScreen = true;     // PC monitor (IMGUI) — not visible in headset

    float _next;
    string _summary = "(probing…)";
    readonly List<InputDevice> _devices = new List<InputDevice>();

    void Update()
    {
        if (Time.time < _next) return;
        _next = Time.time + Mathf.Max(0.5f, logEverySeconds);

        InputDevices.GetDevices(_devices);

        var sb = new StringBuilder();
        sb.AppendLine($"[TrackerProbe] {_devices.Count} XR device(s) visible to Unity:");
        int trackers = 0;
        foreach (var d in _devices)
        {
            bool isHmd  = (d.characteristics & InputDeviceCharacteristics.HeadMounted) != 0;
            bool isCtrl = (d.characteristics & InputDeviceCharacteristics.Controller)  != 0;
            bool isTrk  = (d.characteristics & InputDeviceCharacteristics.TrackedDevice) != 0;

            d.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 pos);
            d.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked);

            string kind = isHmd ? "HMD" : isCtrl ? "Controller" : isTrk ? "TRACKER?" : "other";
            if (!isHmd && !isCtrl && isTrk) trackers++;

            sb.AppendLine($"  - '{d.name}'  [{kind}]  tracked={tracked}  pos={pos}");
            sb.AppendLine($"        characteristics = {d.characteristics}");
        }
        sb.AppendLine($"  => candidate BODY trackers (tracked, not HMD/controller): {trackers}");
        if (trackers == 0)
            sb.AppendLine("  (If this stays 0 with body tracking ON, Unity OpenXR isn't surfacing the trackers -> we'll use the OpenVR/VIVE-tracker path instead.)");

        _summary = sb.ToString();
        Debug.Log(_summary);
    }

    void OnGUI()
    {
        if (!showOnScreen) return;
        var style = new GUIStyle(GUI.skin.label) { fontSize = 12, wordWrap = false };
        style.normal.textColor = Color.white;
        GUI.Label(new Rect(10, 260, 1000, 500), _summary, style);
    }

#if UNITY_EDITOR
    [UnityEditor.MenuItem("MTC/Add XR Tracker Probe to Scene")]
    static void AddToScene()
    {
        var go = new GameObject("TrackerProbe");
        go.AddComponent<TrackerProbe>();
        UnityEditor.Undo.RegisterCreatedObjectUndo(go, "Add TrackerProbe");
        UnityEditor.Selection.activeGameObject = go;
    }
#endif
}
