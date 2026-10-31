using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

// One standing-height calibration. Subsequent crouching and movement remain
// proportional; headset tracking continues to own the camera's local pose.
[DefaultExecutionOrder(-100)]
[DisallowMultipleComponent]
public sealed class PlayerHeightScaler : MonoBehaviour
{
    [SerializeField, Min(0f)] private float delaySeconds = 10f;
    [SerializeField, Min(0.1f)] private float targetHeadsetHeightMeters = 1.3f;
    [SerializeField, HideInInspector] private Transform rig;
    [SerializeField, HideInInspector] private Transform headset;
    [SerializeField, HideInInspector] private bool initialized;
    [SerializeField, HideInInspector] private float startedAt;
    [SerializeField, HideInInspector] private bool applied;
    [SerializeField, HideInInspector] private float scaleFactor = 1f;
    [SerializeField, HideInInspector] private Vector3 floorPivot;
    private readonly List<OptiTrackPointReceiver> receivers = new List<OptiTrackPointReceiver>();

    public bool HasApplied => applied;
    public float ScaleFactor => applied ? scaleFactor : 1f;
    public string Status => applied ? $"Headset height set: {targetHeadsetHeightMeters:F2} m" :
        Time.unscaledTime - startedAt < delaySeconds ?
        $"Stand upright: resize in {Mathf.CeilToInt(delaySeconds - (Time.unscaledTime - startedAt))}s" :
        "Resize: waiting for headset tracking";

    public void Initialize(Transform origin, Transform camera, OptiTrackPointReceiver receiver)
    {
        rig = origin;
        headset = camera;
        if (receiver != null && !receivers.Contains(receiver)) receivers.Add(receiver);
        if (initialized) return;
        initialized = true;
        startedAt = Time.unscaledTime;
    }

    public void Unregister(OptiTrackPointReceiver receiver) => receivers.Remove(receiver);
    public Vector3 ToDisplayWorld(Vector3 trackingWorld) =>
        applied ? floorPivot + (trackingWorld - floorPivot) * scaleFactor : trackingWorld;
    public Vector3 ToTrackingWorld(Vector3 displayWorld) =>
        applied ? floorPivot + (displayWorld - floorPivot) / scaleFactor : displayWorld;

    private void LateUpdate()
    {
        if (!initialized || applied || Time.unscaledTime - startedAt < delaySeconds) return;
        InputDevice hmd = InputDevices.GetDeviceAtXRNode(XRNode.Head);
        if (!hmd.isValid || !hmd.TryGetFeatureValue(CommonUsages.isTracked, out bool tracked) || !tracked) return;
        TryApplyMeasuredHeight(headset != null && rig != null ? headset.position.y - rig.position.y : 0f);
    }

    // Kept separate from device polling so the actual transform behavior can be
    // checked in an isolated Unity scene without a headset or network connection.
    private bool TryApplyMeasuredHeight(float measuredHeight)
    {
        if (applied || rig == null || headset == null || !headset.IsChildOf(rig) ||
            float.IsNaN(measuredHeight) || float.IsInfinity(measuredHeight) || measuredHeight < 0.5f || measuredHeight > 3f)
            return false;
        Vector3 worldScale = rig.lossyScale;
        if (worldScale.x <= 0f || Mathf.Abs(worldScale.x - worldScale.y) > 0.001f ||
            Mathf.Abs(worldScale.x - worldScale.z) > 0.001f || Vector3.Dot(rig.up, Vector3.up) < 0.999f)
        {
            Debug.LogWarning("Player resize requires an upright XR Origin with a uniform positive scale.", this);
            return false;
        }
        scaleFactor = targetHeadsetHeightMeters / measuredHeight;
        floorPivot = new Vector3(headset.position.x, rig.position.y, headset.position.z);
        Vector3 previousOrigin = rig.position;
        rig.localScale *= scaleFactor; // all axes use exactly the same factor
        rig.position = floorPivot + (previousOrigin - floorPivot) * scaleFactor;
        applied = true;
        foreach (OptiTrackPointReceiver receiver in receivers)
            if (receiver != null) receiver.ApplyPlayerPresentationScale(this, scaleFactor, floorPivot, rig);
        Debug.Log($"Player resized uniformly: headset {measuredHeight:F3} m -> {targetHeadsetHeightMeters:F3} m, " +
            $"scale x{scaleFactor:F4}. Floor/headset horizontal position preserved.", this);
        return true;
    }
}
