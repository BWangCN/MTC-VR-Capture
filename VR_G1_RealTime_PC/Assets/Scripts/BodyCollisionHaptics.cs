// BodyCollisionHaptics.cs
// -----------------------------------------------------------------------------
// Builds G1-sized collision CAPSULES from the available body points and buzzes the
// CONTROLLERS when the body penetrates the environment.
//
// Body points (no ALVR re-flash needed):
//   * 8 OSC trackers (from OscBodyReceiver): hip, chest, 2 feet, 2 knees, 2 elbows
//   * head  (XR camera)        * 2 hands (XR controllers)
// The 8 OSC trackers carry no role labels, so we classify them by geometry
// (height + left/right). Tweak swapLeftRight if L/R is flipped.
//
// Collision uses Physics.OverlapCapsule (works with the apartment's NON-convex mesh).
//
// FLOOR: handled at the COLLIDER level by FloorColliderSplitter (the floor triangles are
// removed from the collision mesh), so feet on the floor never collide while EVERYTHING
// ELSE — walls AND furniture, including low furniture sitting on the floor — still does.
// >>> Run  MTC > Split Floor From Environment Collision  once (or rebuild the scene). <<<
//
// Add via  MTC > Add Body Collision + Haptics.  Needs OscBodyReceiver in the scene.
// -----------------------------------------------------------------------------
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;

public class BodyCollisionHaptics : MonoBehaviour
{
    [Header("Sources")]
    public OscBodyReceiver oscReceiver;          // auto-found
    public Transform head;                       // XR camera; auto = Camera.main
    [Tooltip("Play-space origin used to convert controller poses to world (XR Origin). Auto by name.")]
    public Transform xrOrigin;
    [Tooltip("Root of the apartment colliders. Empty = use all MeshColliders in the scene.")]
    public Transform environmentRoot;
    [Tooltip("Optional: a collider to exclude entirely (e.g. a separate floor object).")]
    public Collider excludeCollider;

    [Header("Classification")]
    public bool swapLeftRight = false;

    [Header("G1 capsule radii (m)")]
    public float torsoRadius = 0.11f;
    public float thighRadius = 0.06f;
    public float shinRadius  = 0.055f;
    public float upperArmRadius = 0.055f;
    public float forearmRadius  = 0.05f;
    public float neckHeadRadius = 0.10f;

    [Header("Haptics")]
    public float hapticAmplitude = 0.7f;
    public float hapticDuration = 0.08f;
    [Tooltip("Min seconds between buzzes while penetrating.")]
    public float hapticCooldown = 0.15f;

    [Header("Visuals / debug")]
    public bool showCapsules = true;
    public bool showStatus = true;

    // ---- internal ----
    struct Cap { public string name; public Vector3 a, b; public float r; }
    readonly List<Cap> _caps = new List<Cap>();
    readonly Dictionary<string, Transform> _vis = new Dictionary<string, Transform>();
    Material _matOk, _matHit;

    MeshCollider[] _env;
    float _envRefreshAt;
    readonly Collider[] _overlap = new Collider[16];
    HashSet<Collider> _envSet = new HashSet<Collider>();

    float _lastHaptic;
    readonly List<InputDevice> _ctrls = new List<InputDevice>();
    string _status = "";
    PlayerHeightScaler _heightScaler;
    float _bodyScale = 1f;

    void Start()
    {
        if (oscReceiver == null) oscReceiver = FindObjectOfType<OscBodyReceiver>();
        if (head == null && Camera.main != null) head = Camera.main.transform;
        if (xrOrigin == null) { var xo = GameObject.Find("XR Origin"); if (xo) xrOrigin = xo.transform; }
        _matOk  = MakeMat(new Color(0.2f, 0.9f, 1f, 1f));
        _matHit = MakeMat(new Color(1f, 0.15f, 0.1f, 1f));
        RefreshEnv();
    }

    void RefreshEnv()
    {
        _env = environmentRoot != null
            ? environmentRoot.GetComponentsInChildren<MeshCollider>(false)
            : FindObjectsOfType<MeshCollider>();
        _envSet = new HashSet<Collider>(_env);
        // belt-and-suspenders: also drop anything still named like a floor + a user exclusion
        foreach (var mc in _env)
        {
            if (mc == null) continue;
            string n = mc.gameObject.name.ToLowerInvariant();
            if (n.Contains("floorcollider") || n.Contains("groundcollider")) _envSet.Remove(mc);
        }
        if (excludeCollider != null) _envSet.Remove(excludeCollider);
        _envRefreshAt = Time.time + 2f;
    }

    void LateUpdate()
    {
        if (Time.time >= _envRefreshAt && (_env == null || _env.Length == 0)) RefreshEnv();
        if (head == null || oscReceiver == null) return;

        BuildCapsules();
        UpdateVisuals();
        CheckCollisionAndHaptics();
    }

    // ---------- classify the 8 trackers + head + hands into body roles ----------
    void BuildCapsules()
    {
        if (_heightScaler == null && head != null) _heightScaler = head.GetComponentInParent<PlayerHeightScaler>();
        _bodyScale = _heightScaler != null ? _heightScaler.ScaleFactor : 1f;
        _caps.Clear();

        var pts = oscReceiver.GetBodyTrackerWorldPositions();
        Vector3 right = head.right; right.y = 0f;
        if (right.sqrMagnitude < 1e-4f) right = Vector3.right; else right.Normalize();
        float Lat(Vector3 p) => Vector3.Dot(p - head.position, right);

        Vector3? hip = null, chest = null, lFoot = null, rFoot = null,
                 lKnee = null, rKnee = null, lElbow = null, rElbow = null;

        if (pts.Count >= 2)
        {
            pts.Sort((x, y) => x.y.CompareTo(y.y));
            AssignLR(pts[0], pts[1], Lat, ref lFoot, ref rFoot);
            if (pts.Count >= 4) AssignLR(pts[2], pts[3], Lat, ref lKnee, ref rKnee);
            if (pts.Count >= 5)
            {
                var rem = pts.GetRange(4, pts.Count - 4);
                rem.Sort((x, y) => Mathf.Abs(Lat(y)).CompareTo(Mathf.Abs(Lat(x))));   // most lateral first
                if (rem.Count >= 2) { AssignLR(rem[0], rem[1], Lat, ref lElbow, ref rElbow); rem.RemoveRange(0, 2); }
                if (rem.Count >= 1) { rem.Sort((x, y) => x.y.CompareTo(y.y)); hip = rem[0]; if (rem.Count >= 2) chest = rem[rem.Count - 1]; }
            }
        }

        Vector3 headP = head.position;
        Vector3? lHand = ControllerWorld(InputDeviceCharacteristics.Left);
        Vector3? rHand = ControllerWorld(InputDeviceCharacteristics.Right);

        Vector3? lSh = chest.HasValue ? chest.Value - right * (0.17f * _bodyScale) + Vector3.up * (0.02f * _bodyScale) : (Vector3?)null;
        Vector3? rSh = chest.HasValue ? chest.Value + right * (0.17f * _bodyScale) + Vector3.up * (0.02f * _bodyScale) : (Vector3?)null;

        Add("torso", hip, chest, torsoRadius);
        Add("neck_head", chest, headP, neckHeadRadius);
        Add("L_thigh", hip, lKnee, thighRadius);
        Add("R_thigh", hip, rKnee, thighRadius);
        Add("L_shin", lKnee, lFoot, shinRadius);
        Add("R_shin", rKnee, rFoot, shinRadius);
        Add("L_uarm", lSh, lElbow, upperArmRadius);
        Add("R_uarm", rSh, rElbow, upperArmRadius);
        Add("L_farm", lElbow, lHand, forearmRadius);
        Add("R_farm", rElbow, rHand, forearmRadius);
        Add("head", headP + Vector3.up * (0.02f * _bodyScale), headP + Vector3.up * (0.16f * _bodyScale), neckHeadRadius);
    }

    void AssignLR(Vector3 a, Vector3 b, System.Func<Vector3, float> lat, ref Vector3? left, ref Vector3? rightSide)
    {
        bool aIsLeft = lat(a) < lat(b);
        Vector3 l = aIsLeft ? a : b, r = aIsLeft ? b : a;
        if (swapLeftRight) { var t = l; l = r; r = t; }
        left = l; rightSide = r;
    }

    void Add(string name, Vector3? a, Vector3? b, float r)
    {
        if (a.HasValue && b.HasValue) _caps.Add(new Cap { name = name, a = a.Value, b = b.Value, r = r * _bodyScale });
    }

    Vector3? ControllerWorld(InputDeviceCharacteristics side)
    {
        InputDevices.GetDevicesWithCharacteristics(InputDeviceCharacteristics.Controller | side, _ctrls);
        foreach (var d in _ctrls)
            if (d.TryGetFeatureValue(CommonUsages.isTracked, out bool tr) && tr &&
                d.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 p))
                return xrOrigin != null ? xrOrigin.TransformPoint(p) : p;
        return null;
    }

    // ---------- collision (floor already removed from the collision mesh) ----------
    void CheckCollisionAndHaptics()
    {
        bool anyHit = false;
        var hitNames = new List<string>();

        foreach (var c in _caps)
        {
            bool hit = false;
            int n = Physics.OverlapCapsuleNonAlloc(c.a, c.b, c.r, _overlap, ~0, QueryTriggerInteraction.Ignore);
            for (int i = 0; i < n; i++)
                if (_overlap[i] != null && _envSet.Contains(_overlap[i])) { hit = true; break; }

            if (hit) { anyHit = true; hitNames.Add(c.name); }
            TintVisual(c.name, hit);
        }

        if (anyHit && Time.time - _lastHaptic >= hapticCooldown)
        {
            _lastHaptic = Time.time;
            Buzz(InputDeviceCharacteristics.Left);
            Buzz(InputDeviceCharacteristics.Right);
        }

        _status = $"[BodyCollision] caps={_caps.Count} envColliders={_envSet.Count} " +
                  (anyHit ? $"PENETRATING: {string.Join(",", hitNames)}" : "clear");
    }

    void Buzz(InputDeviceCharacteristics side)
    {
        InputDevices.GetDevicesWithCharacteristics(InputDeviceCharacteristics.Controller | side, _ctrls);
        foreach (var d in _ctrls)
            if (d.TryGetHapticCapabilities(out var caps) && caps.supportsImpulse)
                d.SendHapticImpulse(0u, hapticAmplitude, hapticDuration);
    }

    // ---------- visuals ----------
    void UpdateVisuals()
    {
        if (!showCapsules) return;
        foreach (var c in _caps)
        {
            var t = GetVis(c.name);
            Vector3 mid = (c.a + c.b) * 0.5f;
            Vector3 dir = c.b - c.a;
            float len = dir.magnitude;
            t.position = mid;
            t.up = (len > 1e-4f) ? dir / len : Vector3.up;
            t.localScale = new Vector3(c.r * 2f, len * 0.5f + c.r, c.r * 2f);
            t.gameObject.SetActive(true);
        }
    }

    Transform GetVis(string name)
    {
        if (_vis.TryGetValue(name, out var t) && t) return t;
        var go = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        go.name = "cap:" + name;
        var col = go.GetComponent<Collider>(); if (col) Destroy(col);   // visual only
        go.GetComponent<Renderer>().sharedMaterial = _matOk;
        go.transform.SetParent(transform, true);
        _vis[name] = go.transform;
        return go.transform;
    }

    void TintVisual(string name, bool hit)
    {
        if (!showCapsules) return;
        if (_vis.TryGetValue(name, out var t) && t)
        {
            var r = t.GetComponent<Renderer>();
            if (r) r.sharedMaterial = hit ? _matHit : _matOk;
        }
    }

    void OnGUI()
    {
        if (!showStatus) return;
        var st = new GUIStyle(GUI.skin.label) { fontSize = 13 };
        st.normal.textColor = _status.Contains("PENETRAT") ? Color.red : Color.green;
        GUI.Label(new Rect(10, 360, 1000, 24), _status, st);
    }

    static Material MakeMat(Color c)
    {
        Shader sh = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
        var m = new Material(sh) { color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        return m;
    }

#if UNITY_EDITOR
    [UnityEditor.MenuItem("MTC/Add Body Collision + Haptics")]
    static void AddToScene()
    {
        var go = new GameObject("BodyCollisionHaptics");
        go.AddComponent<BodyCollisionHaptics>();
        UnityEditor.Undo.RegisterCreatedObjectUndo(go, "Add BodyCollisionHaptics");
        UnityEditor.Selection.activeGameObject = go;
    }
#endif
}
