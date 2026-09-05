// Body24Collision.cs
// -----------------------------------------------------------------------------
// ADDITIVE collision warning + bad-data logger for the 24-point VR capture.
// Does NOT modify the existing capture / coordinate pipeline -- it only READS
// PXR_BodyTrackingFeed.LatestRolePoses.
//
// Each frame it puts a detection SPHERE at every one of the 24 SMPL joints
// (world = cubesRoot.TransformPoint(joint)), tests Physics.OverlapSphere against
// the environment MeshColliders, flags the offending joints red on any
// penetration, and records the per-frame collision state. The log auto-starts/stops with the
// recorder (PXR_24PointMapper.recordCsv) and is written to
// Application.persistentDataPath on Stop, so `capture_pipeline.py stow` pairs it
// 1:1 with the motion CSV (joined later on `frame` = Time.frameCount).
//
// FLOOR: colliders whose name contains "floor"/"ground" are ignored, so feet on
// the floor never trip -- but a foot driven INTO an obstacle base still does.
//
// Setup (one-time -- it persists in the saved scene, and self-configures each run):
//   * Add via  MTC > Add 24-Point Collision Logger, then save the scene. Done.
//   * It auto-finds PXR_BodyTrackingFeed + PXR_24PointMapper, auto-detects the
//     ACTIVE environment by name (autoFindEnv), and auto-adds MeshColliders. So
//     you never touch it again -- just keep exactly one environment mesh active
//     and it collides against whatever that is (other nodes under WorldOrigin,
//     and any deactivated envs, are ignored).
//   * Logging auto-starts/stops with the recorder (Start/Stop Recording), so
//     each collision log pairs 1:1 with its motion CSV.
// -----------------------------------------------------------------------------
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

public class Body24Collision : MonoBehaviour
{
    [Header("Sources")]
    public PXR_BodyTrackingFeed feed;             // auto-found
    [Tooltip("The recorder. When set, the collision log auto-starts/stops with " +
             "recordCsv and pairs 1:1 with each motion CSV. Auto-found.")]
    public PXR_24PointMapper mapper;              // auto-found
    public Transform cubesRoot;                    // auto = feed.cubesRoot (world ref)
    [Tooltip("Root of the environment mesh. Empty = every MeshCollider in the scene.")]
    public Transform environmentRoot;

    [Header("Detection")]
    [Tooltip("Detection sphere radius per joint (m). Bigger = trips more easily. Default 0.08.")]
    public float sphereRadius = 0.08f;
    [Tooltip("Auto-add MeshColliders to env meshes at runtime if missing.")]
    public bool autoAddColliders = true;
    [Tooltip("Auto-detect the ACTIVE environment by name -- no manual Environment Root " +
             "needed. Only active meshes are scanned, so deactivating an env drops it and " +
             "activating another picks it up automatically.")]
    public bool autoFindEnv = true;
    [Tooltip("Name substrings marking a GameObject (or any ancestor) as an environment. " +
             "Matched case-insensitively over the whole parent chain.")]
    public string[] envNameContains = new[] {
        "_simplified", "corridor_", "bedroom_", "livingroom_", "disaster_", "kitchen_"
    };
    [Tooltip("Ignore colliders whose name STARTS WITH one of these (case-insensitive): " +
             "the corridor floor, and the goal ring + its marker beam.")]
    // Was a SUBSTRING list {floor, ground, goal} and the "ground" token -- there to
    // catch a floor named "ground" -- silently swallowed every clutter mesh:
    // make_dense_ground names its items ground_<id>_<n>_<asset>. Stepping INTO the
    // clutter was therefore never logged, which is the one thing the dense_ground
    // obstacle exists to measure. Prefix match without "ground" instead; the
    // corridor floor is always named "floor" (fbx_builder), and "goal_ring" still
    // covers goal_ring_beam. RENAMED so the stale array serialized into the scene
    // is dropped rather than inherited.
    public string[] ignoreNamePrefixes = new[] { "floor", "goal_ring" };
    [Tooltip("Re-scan the active environment every N seconds (picks up env swaps).")]
    public float envRefreshInterval = 1.5f;

    [Header("Logging")]
    [Tooltip("Fallback manual dump key, only used when no PXR_24PointMapper is set.")]
    public KeyCode dumpHotkey = KeyCode.F9;
    public string logPrefix = "collisions";

    [Header("Visuals / debug")]
    public bool showSpheres = true;
    public bool showStatus = true;

    // ---- internal ----
    HashSet<Collider> _envSet = new HashSet<Collider>();
    float _envRefreshAt;
    readonly Collider[] _hits = new Collider[16];
    Material _matOk, _matHit;
    readonly Dictionary<int, Transform> _vis = new Dictionary<int, Transform>();
    readonly List<string> _log = new List<string>();
    int _frameIdx = 0;
    bool _wasRecording = false;
    string _status = "";

    // SMPL 24-joint order (matches PositionIndex / BODY_JOINT_SET_BODY_FULL_START)
    static readonly string[] JN = {
        "hip", "l_hip", "r_hip", "spine1", "l_knee", "r_knee", "spine2", "l_ankle",
        "r_ankle", "spine3", "l_foot", "r_foot", "neck", "l_collar", "r_collar", "head",
        "l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist", "l_hand", "r_hand"
    };

    void Start()
    {
        if (feed == null) feed = FindObjectOfType<PXR_BodyTrackingFeed>();
        if (mapper == null) mapper = FindObjectOfType<PXR_24PointMapper>();
        if (cubesRoot == null && feed != null) cubesRoot = feed.cubesRoot;
        _matOk = MakeMat(new Color(0.2f, 0.9f, 0.3f, 0.45f));
        _matHit = MakeMat(new Color(1f, 0.12f, 0.1f, 0.75f));
        RefreshEnv();
    }

    // Build the environment collider set from the currently ACTIVE meshes:
    //   - manual environmentRoot if set, else auto-detected by name;
    //   - floor/ground names ignored;
    //   - MeshColliders auto-added where missing.
    // Only active MeshFilters are scanned, so swapping which env is active
    // automatically swaps what we collide against -- no per-env setup.
    void RefreshEnv()
    {
        _envSet = new HashSet<Collider>();
        MeshFilter[] mfs = environmentRoot != null
            ? environmentRoot.GetComponentsInChildren<MeshFilter>(false)
            : FindObjectsOfType<MeshFilter>(false);   // false = active only

        foreach (var mf in mfs)
        {
            if (mf == null || mf.sharedMesh == null) continue;
            if (environmentRoot == null && autoFindEnv && !IsEnvMesh(mf.transform)) continue;
            if (NameStartsWith(mf.gameObject.name, ignoreNamePrefixes)) continue;   // floor / goal
            var mc = mf.GetComponent<MeshCollider>();
            if (mc == null && autoAddColliders)
            {
                mc = mf.gameObject.AddComponent<MeshCollider>();
                mc.sharedMesh = mf.sharedMesh;   // non-convex OK for OverlapSphere
            }
            if (mc != null && mc.enabled) _envSet.Add(mc);
        }
        _envRefreshAt = Time.time + Mathf.Max(0.25f, envRefreshInterval);
    }

    bool IsEnvMesh(Transform t)
    {
        for (var p = t; p != null; p = p.parent)
            if (NameMatches(p.name, envNameContains)) return true;
        return false;
    }

    static bool NameStartsWith(string name, string[] prefixes)
    {
        if (prefixes == null) return false;
        foreach (var s in prefixes)
            if (!string.IsNullOrEmpty(s) &&
                name.StartsWith(s, System.StringComparison.OrdinalIgnoreCase)) return true;
        return false;
    }

    static bool NameMatches(string name, string[] subs)
    {
        string n = name.ToLowerInvariant();
        foreach (var s in subs)
            if (!string.IsNullOrEmpty(s) && n.Contains(s.ToLowerInvariant())) return true;
        return false;
    }

    void LateUpdate()
    {
        if (feed == null || feed.LatestRolePoses == null) return;
        if (Time.time >= _envRefreshAt) RefreshEnv();   // periodic; picks up env swaps

        var poses = feed.LatestRolePoses;
        var hitJoints = new List<string>();
        bool anyValid = false;

        int count = Mathf.Min(poses.Length, JN.Length);
        for (int i = 0; i < count; i++)
        {
            if (!poses[i].valid) { SetVisActive(i, false); continue; }
            anyValid = true;
            Vector3 w = cubesRoot != null ? cubesRoot.TransformPoint(poses[i].pos) : poses[i].pos;
            bool hit = false;
            int n = Physics.OverlapSphereNonAlloc(w, sphereRadius, _hits, ~0, QueryTriggerInteraction.Ignore);
            for (int k = 0; k < n; k++)
                if (_hits[k] != null && _envSet.Contains(_hits[k])) { hit = true; break; }
            if (hit) hitJoints.Add(JN[i]);
            UpdateSphere(i, w, hit);
        }

        bool anyHit = hitJoints.Count > 0;

        // Log in lockstep with the recorder: start a fresh log when recording
        // begins, log every valid frame while recording, dump when it ends. The
        // row key is Time.frameCount + realtimeSinceStartup -- EXACTLY the motion
        // CSV's `frame` and `time` columns -- so the log joins on `frame` with
        // zero ambiguity. No mapper => fall back to continuous log + F9 dump.
        bool rec = mapper != null && mapper.recordCsv;
        if (mapper != null)
        {
            if (rec && !_wasRecording) { RefreshEnv(); StartLog(); }
            if (rec && anyValid) LogRow(anyHit, hitJoints);
            if (!rec && _wasRecording) DumpLog();
            _wasRecording = rec;
        }
        else
        {
            if (_log.Count == 0) StartLog();
            if (anyValid) LogRow(anyHit, hitJoints);
            if (Input.GetKeyDown(dumpHotkey)) DumpLog();
        }

        _status = $"[Body24Collision] r={sphereRadius:F2}m env={_envSet.Count} " +
                  (rec ? "REC " : "") +
                  (anyHit ? $"COLLIDING: {string.Join(",", hitJoints)}" : "clear") +
                  $"  frames={_frameIdx}";
    }

    void StartLog()
    {
        _log.Clear();
        _log.Add("frame,time,hit,joints");
        _frameIdx = 0;
    }

    void LogRow(bool anyHit, List<string> hitJoints)
    {
        _log.Add($"{Time.frameCount},{Time.realtimeSinceStartup:F6},{(anyHit ? 1 : 0)},{string.Join("|", hitJoints)}");
        _frameIdx++;
    }

    void DumpLog()
    {
        string ts = System.DateTime.Now.ToString("yyyyMMdd-HHmmss");
        string path = Path.Combine(Application.persistentDataPath, $"{logPrefix}_{ts}.csv");
        try
        {
            File.WriteAllText(path, string.Join("\n", _log), Encoding.UTF8);
            Debug.Log($"[Body24Collision] wrote {_log.Count - 1} frames -> {path}");
        }
        catch (System.Exception e) { Debug.LogError($"[Body24Collision] dump failed: {e.Message}"); }
    }

    // ---------- visuals ----------
    void UpdateSphere(int i, Vector3 world, bool hit)
    {
        if (!showSpheres) return;
        var t = GetVis(i);
        t.position = world;
        t.localScale = Vector3.one * (sphereRadius * 2f);
        t.gameObject.SetActive(true);
        var r = t.GetComponent<Renderer>();
        if (r) r.sharedMaterial = hit ? _matHit : _matOk;
    }

    void SetVisActive(int i, bool on)
    {
        if (_vis.TryGetValue(i, out var t) && t) t.gameObject.SetActive(on);
    }

    Transform GetVis(int i)
    {
        if (_vis.TryGetValue(i, out var t) && t) return t;
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = "hitsphere:" + JN[i];
        var col = go.GetComponent<Collider>();
        if (col) Destroy(col);   // visual only
        go.GetComponent<Renderer>().sharedMaterial = _matOk;
        go.transform.SetParent(transform, true);
        _vis[i] = go.transform;
        return go.transform;
    }

    void OnGUI()
    {
        if (!showStatus) return;
        var st = new GUIStyle(GUI.skin.label) { fontSize = 13 };
        st.normal.textColor = _status.Contains("COLLIDING") ? Color.red : Color.green;
        GUI.Label(new Rect(10, 384, 1200, 24), _status, st);
    }

    static Material MakeMat(Color c)
    {
        Shader sh = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
        var m = new Material(sh) { color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        return m;
    }

#if UNITY_EDITOR
    [UnityEditor.MenuItem("MTC/Add 24-Point Collision Logger")]
    static void AddToScene()
    {
        var go = new GameObject("Body24Collision");
        go.AddComponent<Body24Collision>();
        UnityEditor.Undo.RegisterCreatedObjectUndo(go, "Add Body24Collision");
        UnityEditor.Selection.activeGameObject = go;
    }
#endif
}
