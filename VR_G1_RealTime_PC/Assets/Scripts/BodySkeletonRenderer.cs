// BodySkeletonRenderer.cs
// -----------------------------------------------------------------------------
// Renders the live 24-joint body (from the custom UDP BodyDataReceiver) as spheres
// + bone lines, anchored to the live XR head pose so it sits on the player inside
// the streamed apartment. Renders with real meshes/LineRenderers (NOT Gizmos), so
// it is visible in the headset over ALVR.
//
// Exposes WorldJoints[24] (Unity world space) so a downstream collision/haptics
// component can reuse the same aligned joints.
//
// Alignment (v1, tweak in Inspector): pin the body's HEAD joint to the XR camera and
// yaw-align the body to where you look. Body pitch/roll come from the tracked data.
// This needs in-headset verification — adjust headToEyeOffset / alignToHead if the
// skeleton sits slightly off.
// -----------------------------------------------------------------------------
using UnityEngine;

public class BodySkeletonRenderer : MonoBehaviour
{
    [Header("Source")]
    [Tooltip("Custom UDP 24-joint receiver (port 9876). Auto-found if not set.")]
    public BodyDataReceiver receiver;
    [Tooltip("XR camera (center eye) to anchor the body to. Auto = Camera.main.")]
    public Transform hmd;

    [Header("Alignment")]
    [Tooltip("Pin the body's head joint to the XR camera and yaw-align to where you look.")]
    public bool alignToHead = true;
    [Tooltip("Offset (camera frame) from the eye to the body's head joint. Tune in-headset.")]
    public Vector3 headToEyeOffset = Vector3.zero;

    [Header("Look")]
    public float jointRadius = 0.03f;
    [Tooltip("Hide joint spheres whose centers are within this distance of the camera, in meters.")]
    [Min(0f)]
    public float minimumCameraDistanceMeters = 0.15f;
    public float boneWidth = 0.018f;
    public Color skeletonColor = new Color(0.2f, 0.9f, 1f, 1f);

    [Header("Status (read-only)")]
    public bool hasData;
    public Vector3[] WorldJoints = new Vector3[24];

    // 24-joint bone topology (matches PositionIndex / BodyDataReceiver order)
    static readonly (int a, int b)[] Bones = {
        (0,3),(3,6),(6,9),(9,12),(12,15),                 // spine
        (0,1),(1,4),(4,7),(7,10),                         // left leg
        (0,2),(2,5),(5,8),(8,11),                         // right leg
        (9,13),(13,16),(16,18),(18,20),(20,22),           // left arm
        (9,14),(14,17),(17,19),(19,21),(21,23),           // right arm
    };
    const int HEAD = 15;

    Transform[] _joints;
    LineRenderer[] _bones;
    Material _mat;
    float _warnAt = -1f;

    void Start()
    {
        if (receiver == null) receiver = FindObjectOfType<BodyDataReceiver>();
        if (receiver == null) { Debug.LogError("[BodySkeleton] No BodyDataReceiver in scene. Disabling."); enabled = false; return; }
        if (hmd == null && Camera.main != null) hmd = Camera.main.transform;

        _mat = MakeMat(skeletonColor);

        _joints = new Transform[24];
        for (int i = 0; i < 24; i++)
        {
            var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s.name = $"joint_{i}";
            var col = s.GetComponent<Collider>(); if (col) Destroy(col);
            s.transform.SetParent(transform, false);
            s.transform.localScale = Vector3.one * (jointRadius * 2f);
            var r = s.GetComponent<Renderer>(); if (r) r.sharedMaterial = _mat;
            _joints[i] = s.transform;
        }

        _bones = new LineRenderer[Bones.Length];
        for (int i = 0; i < Bones.Length; i++)
        {
            var go = new GameObject($"bone_{i}");
            go.transform.SetParent(transform, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            lr.positionCount = 2;
            lr.startWidth = lr.endWidth = boneWidth;
            lr.numCapVertices = 2;
            lr.material = _mat;
            _bones[i] = lr;
        }

        SetVisible(false);
        _warnAt = Time.time + 6f;
        Debug.Log($"[BodySkeleton] Ready. Waiting for UDP body data on port {receiver.port} (PICO PXR_NetworkStreamer must target this PC).");
    }

    void LateUpdate()
    {
        bool live = receiver != null && receiver.isReceiving && receiver.framesReceived > 0
                    && receiver.receivedJoints != null && receiver.receivedJoints.Length >= 24;

        if (!live)
        {
            if (hasData) { hasData = false; SetVisible(false); }
            if (_warnAt > 0f && Time.time > _warnAt)
            {
                _warnAt = Time.time + 6f;
                Debug.LogWarning($"[BodySkeleton] No body data yet on UDP {(receiver != null ? receiver.port : 0)}. " +
                                 "Is the PICO body-streamer (PXR_NetworkStreamer) running and pointing at this PC's IP?");
            }
            return;
        }

        if (!hasData) { hasData = true; SetVisible(true); Debug.Log("[BodySkeleton] Body data flowing — rendering 24 joints."); }

        ComputeWorldJoints();
        for (int i = 0; i < 24; i++)
        {
            _joints[i].position = WorldJoints[i];
            bool tooCloseToCamera = hmd != null &&
                (WorldJoints[i] - hmd.position).sqrMagnitude <=
                minimumCameraDistanceMeters * minimumCameraDistanceMeters;
            _joints[i].gameObject.SetActive(!tooCloseToCamera);
        }
        for (int i = 0; i < _bones.Length; i++)
        {
            _bones[i].SetPosition(0, WorldJoints[Bones[i].a]);
            _bones[i].SetPosition(1, WorldJoints[Bones[i].b]);
        }
    }

    void ComputeWorldJoints()
    {
        var src = receiver.receivedJoints;
        if (!alignToHead || hmd == null)
        {
            for (int i = 0; i < 24; i++) WorldJoints[i] = src[i];
            return;
        }

        // yaw-only alignment of body facing -> camera facing, head pinned to camera
        Quaternion bHeadRot = (receiver.receivedRotations != null && receiver.receivedRotations.Length >= 24)
            ? receiver.receivedRotations[HEAD] : Quaternion.identity;
        Vector3 bFwd = bHeadRot * Vector3.forward; bFwd.y = 0f;
        Vector3 cFwd = hmd.forward; cFwd.y = 0f;
        Quaternion yaw = (bFwd.sqrMagnitude > 1e-4f && cFwd.sqrMagnitude > 1e-4f)
            ? Quaternion.FromToRotation(bFwd.normalized, cFwd.normalized)
            : Quaternion.identity;

        Vector3 targetHead = hmd.position + hmd.rotation * headToEyeOffset;
        Vector3 bHead = src[HEAD];
        for (int i = 0; i < 24; i++)
            WorldJoints[i] = targetHead + yaw * (src[i] - bHead);
    }

    void SetVisible(bool v)
    {
        if (_joints != null) foreach (var j in _joints) if (j) j.gameObject.SetActive(v);
        if (_bones != null) foreach (var b in _bones) if (b) b.gameObject.SetActive(v);
    }

    static Material MakeMat(Color c)
    {
        Shader sh = Shader.Find("Unlit/Color");
        if (sh == null) sh = Shader.Find("Standard");
        if (sh == null) sh = Shader.Find("Sprites/Default");
        var m = new Material(sh) { color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        return m;
    }
}
