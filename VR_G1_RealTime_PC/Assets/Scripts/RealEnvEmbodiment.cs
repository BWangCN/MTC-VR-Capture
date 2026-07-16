// RealEnvEmbodiment.cs
// -----------------------------------------------------------------------------
// Ports the "embodiment scaling" logic from My project's PXR_SessionAutoFlow into
// the PC-rendered / streamed real-environment walkthrough (the StreamingEnv scene).
//
// Idea (identical to My project): DON'T shrink the human. Instead, after a short
// standing-height calibration, SCALE THE WORLD UP by 1/scale around the player's
// feet, so a real ~1.7 m person perceives the reconstructed room from the eye
// height / scale of a 1.27 m Unitree G1 robot ("I became short").
//
//     scale       = targetHeight (G1 1.27 m) / measuredHumanHeight     (~0.74)
//     worldScale  = 1 / scale                                          (~1.35)
//     Environment.localScale *= worldScale     (about the feet pivot; feet stay put)
//
// NOTE on feedback in VR: Unity OnGUI/IMGUI does NOT render into the VR headset (it
// only shows on the PC monitor). So this script ALSO shows a WORLD-SPACE TextMesh
// HUD (a child of the XR camera) that IS visible in the headset, plus verbose
// Debug.Log lines on the PC Console.
//
// IMPORTANT: assumes the Environment mesh is true metric (1 unit = 1 m). Replica /
// ScanNet / glTF(meters) are 1:1. The scene builder wires hmd / worldRoot / floorRef.
// -----------------------------------------------------------------------------
using System.Collections;
using UnityEngine;

public class RealEnvEmbodiment : MonoBehaviour
{
    [Header("References")]
    [Tooltip("XR head camera (center eye). Used to measure standing eye height.")]
    public Transform hmd;

    [Tooltip("The environment root to scale (the 'Environment' object built from the scanned mesh).")]
    public Transform worldRoot;

    [Tooltip("Optional: XR Origin / floor reference. Its world Y is treated as the floor (y=0 if unset).")]
    public Transform floorRef;

    [Header("Embodiment target")]
    [Tooltip("Target robot height in meters. Unitree G1 = 1.27 m.")]
    public float targetHeightMeters = 1.27f;

    [Tooltip("Eye -> top-of-head offset (m), added to measured eye height to get full stature.")]
    public float eyeToTopOfHeadOffset = 0.12f;

    [Header("Calibration")]
    [Tooltip("Average the live HMD height before scaling. Turn OFF to use manualHumanHeightMeters.")]
    public bool autoCalibrate = true;

    [Tooltip("Seconds to average the standing eye height before applying the scale.")]
    public float calibrationSeconds = 5f;

    [Tooltip("Seconds to wait for tracking to settle before calibrating.")]
    public float trackingWarmupSeconds = 1.0f;

    [Tooltip("If autoCalibrate is OFF (or no valid HMD samples), the human stature (m) used.")]
    public float manualHumanHeightMeters = 1.70f;

    [Header("Scale pivot")]
    [Tooltip("Scale the world around the point under the HMD (recommended). Otherwise around floorRef / world origin.")]
    public bool pivotAtHmd = true;

    [Header("Status (read-only)")]
    public bool applied;
    public float measuredHeight;
    public float embodimentScale = 1f;   // target / human   (<1 for G1)
    public float worldScale = 1f;        // 1 / embodimentScale (>1)

    [Header("Debug")]
    public bool showOnScreenInfo = true;     // PC monitor (IMGUI) — NOT visible in VR
    public bool showInVrIndicator = true;    // world-space HUD — visible in the headset
    public float indicatorHideAfterSeconds = 12f;

    // Baseline of the Environment transform captured BEFORE any scaling, so repeated
    // calibration is idempotent (we always re-derive from baseline, never compound).
    private Vector3 _baseScale = Vector3.one;
    private Vector3 _basePos = Vector3.zero;
    private bool _baseCaptured;

    // VR world-space HUD
    private TextMesh _vrText;
    private string _status = "";
    private float _hideAt = -1f;

    private float GroundY => floorRef != null ? floorRef.position.y : 0f;

    void Start()
    {
        if (worldRoot == null)
        {
            Debug.LogError("[RealEnvEmbodiment] worldRoot (Environment) not assigned. Disabling.");
            enabled = false;
            return;
        }
        Debug.Log($"[RealEnvEmbodiment] Start: worldRoot='{worldRoot.name}', hmd='{(hmd ? hmd.name : "NULL")}', " +
                  $"floorRef='{(floorRef ? floorRef.name : "NULL")}', GroundY={GroundY:F2}, target={targetHeightMeters:F2}m");
        CaptureBaseline();
        CreateVrIndicator();
        StartCoroutine(RunFlow());
    }

    void CaptureBaseline()
    {
        if (_baseCaptured) return;
        _baseScale = worldRoot.localScale;
        _basePos = worldRoot.position;
        _baseCaptured = true;
    }

    void CreateVrIndicator()
    {
        if (!showInVrIndicator || hmd == null) return;
        var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (font == null) font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        if (font == null) return;

        var go = new GameObject("EmbodimentVrHUD");
        go.transform.SetParent(hmd, false);
        go.transform.localPosition = new Vector3(0f, -0.25f, 0.9f); // 0.9 m in front, slightly down
        go.transform.localRotation = Quaternion.Euler(0f, 180f, 0f); // face the user
        _vrText = go.AddComponent<TextMesh>();
        _vrText.font = font;
        _vrText.fontSize = 72;
        _vrText.characterSize = 0.012f;
        _vrText.anchor = TextAnchor.MiddleCenter;
        _vrText.alignment = TextAlignment.Center;
        _vrText.color = Color.white;
        var mr = go.GetComponent<MeshRenderer>();
        if (mr != null) mr.sharedMaterial = font.material;
    }

    private IEnumerator RunFlow()
    {
        _status = "Embodiment\nwaiting for tracking…";
        if (trackingWarmupSeconds > 0f)
            yield return new WaitForSeconds(trackingWarmupSeconds);

        if (autoCalibrate && hmd != null)
        {
            float t0 = Time.time;
            float sum = 0f;
            int count = 0;
            while (Time.time - t0 < calibrationSeconds)
            {
                float eye = hmd.position.y - GroundY;
                if (eye > 0.5f && eye < 3.0f) { sum += eye; count++; }
                _status = $"Embodiment: calibrating…\nstand still  {Mathf.Max(0f, calibrationSeconds - (Time.time - t0)):F0}s\neye ≈ {eye:F2} m";
                yield return null;
            }

            measuredHeight = (count > 0) ? (sum / count) + eyeToTopOfHeadOffset : manualHumanHeightMeters;
            Debug.Log($"[RealEnvEmbodiment] Calibration: {count} valid samples, GroundY={GroundY:F2}, " +
                      $"hmdY={hmd.position.y:F2}, measuredHeight={measuredHeight:F3}m" +
                      (count == 0 ? "  >>> NO valid samples (eye outside 0.5–3.0 m). Are you floating? Check spawn / SteamVR floor. Used manual height." : ""));
        }
        else
        {
            measuredHeight = manualHumanHeightMeters;
            if (autoCalibrate && hmd == null)
                Debug.LogWarning("[RealEnvEmbodiment] autoCalibrate=true but hmd is NULL; using manualHumanHeightMeters.");
        }

        float scale = targetHeightMeters / Mathf.Max(0.5f, measuredHeight);
        ApplyEmbodimentScale(scale);

        _status = $"Embodiment ON\nG1 {targetHeightMeters:F2} m | you {measuredHeight:F2} m\nworld ×{worldScale:F2}";
        _hideAt = Time.time + indicatorHideAfterSeconds;
        Debug.Log($"[RealEnvEmbodiment] APPLIED: embodimentScale={embodimentScale:F3}, world ×{worldScale:F3} " +
                  $"around pivot {GetPivotWorld()}. worldRoot.localScale now {worldRoot.localScale}, pos {worldRoot.position}.");
    }

    /// Apply scale = target/human. The world is scaled by 1/scale about the feet pivot,
    /// keeping the player's footing fixed. Idempotent (always re-derives from baseline).
    public void ApplyEmbodimentScale(float scale)
    {
        CaptureBaseline();
        embodimentScale = Mathf.Max(1e-3f, scale);
        worldScale = 1f / embodimentScale;

        worldRoot.localScale = _baseScale;     // reset to baseline so re-calibration doesn't compound
        worldRoot.position = _basePos;

        Vector3 pivot = GetPivotWorld();
        Vector3 dir = worldRoot.position - pivot;
        worldRoot.localScale = _baseScale * worldScale;
        worldRoot.position = pivot + dir * worldScale;
        applied = true;
    }

    /// Restore the Environment to its un-scaled baseline.
    public void ResetScale()
    {
        if (!_baseCaptured) return;
        worldRoot.localScale = _baseScale;
        worldRoot.position = _basePos;
        applied = false;
        embodimentScale = 1f;
        worldScale = 1f;
    }

    [ContextMenu("Recalibrate")]
    public void Recalibrate()
    {
        ResetScale();
        StopAllCoroutines();
        if (_vrText != null) _vrText.gameObject.SetActive(true);
        StartCoroutine(RunFlow());
    }

    private Vector3 GetPivotWorld()
    {
        float gy = GroundY;
        if (pivotAtHmd && hmd != null)
            return new Vector3(hmd.position.x, gy, hmd.position.z);
        if (floorRef != null)
            return new Vector3(floorRef.position.x, gy, floorRef.position.z);
        return new Vector3(0f, gy, 0f);
    }

    void Update()
    {
        if (_vrText != null)
        {
            _vrText.text = _status;
            if (_hideAt > 0f && Time.time > _hideAt && _vrText.gameObject.activeSelf)
                _vrText.gameObject.SetActive(false);
        }
    }

    void OnGUI()
    {
        if (!showOnScreenInfo) return;   // PC monitor only — invisible in the headset
        GUI.color = applied ? Color.green : Color.yellow;
        string s = applied
            ? $"[Embodiment] G1 {targetHeightMeters:F2}m  |  you {measuredHeight:F2}m  |  world x{worldScale:F2}"
            : $"[Embodiment] calibrating... stand still ({calibrationSeconds:F0}s)";
        GUI.Label(new Rect(10, 220, 640, 24), s);
    }
}
