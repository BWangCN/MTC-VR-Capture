// GoalZone.cs
// -----------------------------------------------------------------------------
// ADDITIVE goal detection + feedback for the 24-point VR capture. Does NOT touch
// the existing capture pipeline.
//
// It auto-finds the "goal_ring" baked into the active environment FBX, renders a
// BRIGHT always-visible marker there (so the end-position ring shows in Unity
// even if the imported FBX material looks dull), and declares SUCCESS when the
// pelvis enters the ring. On success it plays a chime and turns the ring green.
//
// Like Body24Collision it uses world positions, so the environment's calibration
// scale is handled automatically. Add once via  MTC > Add Goal Zone,  save the
// scene, and it self-configures every run.
// -----------------------------------------------------------------------------
using UnityEngine;

public class GoalZone : MonoBehaviour
{
    /// Fired once per entry into the goal ring, after the dwell time. Used by
    /// MTC_AtomicSession to stop the take (and optionally advance) without the
    /// pilot pressing anything.
    public event System.Action onReached;

    [Header("Sources")]
    public PXR_BodyTrackingFeed feed;                 // auto-found
    public Transform cubesRoot;                        // auto = feed.cubesRoot
    [Tooltip("Name (substring) of the goal object baked into the env FBX.")]
    public string goalObjectName = "goal_ring";

    [Header("Success rule")]
    [Tooltip("Body joint used as the 'you' position. pelvis/hip = 0.")]
    public int successJoint = 0;
    [Tooltip("Extra margin added to the ring radius (m). Negative = must be well inside.")]
    public float radiusMargin = 0f;
    [Tooltip("Seconds the pelvis must stay inside before it counts (anti-flicker).")]
    public float dwellSeconds = 0.3f;
    [Tooltip("Fallback ring radius if it can't be read from the mesh (m).")]
    public float fallbackRadius = 0.5f;

    [Header("Feedback")]
    public bool sound = true;

    [Header("Marker")]
    public bool showMarker = true;
    public bool hideBakedRing = true;                  // hide the dull FBX ring, show ours
    public Color pendingColor = new Color(1f, 0.85f, 0.2f);   // yellow
    public Color successColor = new Color(0.2f, 1f, 0.35f);   // green
    [Tooltip("Height of the light beam above the ring (m).")]
    public float beamHeight = 2.0f;

    [Header("Debug")]
    public bool showStatus = true;

    // ---- internal ----
    Transform _goal;
    Renderer _goalRenderer;
    float _radius;
    GameObject _marker, _disc, _beam;
    Material _mat;
    bool _reached, _inside;
    float _insideSince = -1f;
    AudioSource _audio;
    AudioClip _chime;
    string _status = "";

    void Start()
    {
        if (feed == null) feed = FindObjectOfType<PXR_BodyTrackingFeed>();
        if (cubesRoot == null && feed != null) cubesRoot = feed.cubesRoot;
        _mat = MakeUnlit(pendingColor);
        if (sound)
        {
            _audio = gameObject.AddComponent<AudioSource>();
            _audio.playOnAwake = false;
            _audio.spatialBlend = 0f;
            _chime = MakeChime();
        }
        if (showMarker) BuildMarker();
    }

    void LateUpdate()
    {
        if (!FindGoal()) { SetStatus("no goal_ring found in active env"); return; }
        UpdateMarker();

        if (feed == null || feed.LatestRolePoses == null) return;
        var poses = feed.LatestRolePoses;
        if (successJoint < 0 || successJoint >= poses.Length || !poses[successJoint].valid) return;

        Vector3 you = cubesRoot != null ? cubesRoot.TransformPoint(poses[successJoint].pos)
                                        : poses[successJoint].pos;
        Vector3 g = _goal.position;
        float horiz = new Vector2(you.x - g.x, you.z - g.z).magnitude;   // Unity Y = up
        _inside = horiz <= _radius + radiusMargin;

        if (_inside)
        {
            if (_insideSince < 0f) _insideSince = Time.time;
            if (!_reached && Time.time - _insideSince >= dwellSeconds) OnReached();
        }
        else
        {
            _insideSince = -1f;
            _reached = false;   // allow re-trigger if they step out and back in
        }

        TintMarker(_inside || _reached);
        SetStatus($"dist={horiz:F2}m / r={_radius:F2}m " + (_reached ? "REACHED" : _inside ? "inside..." : ""));
    }

    void OnReached()
    {
        _reached = true;
        if (sound && _audio != null && _chime != null) _audio.PlayOneShot(_chime);
        Debug.Log($"[GoalZone] SUCCESS: pelvis in goal at frame {Time.frameCount}, t={Time.realtimeSinceStartup:F3}");
        if (onReached != null) onReached();
    }

    // ---------- goal lookup ----------
    bool FindGoal()
    {
        if (_goal != null && _goal.gameObject.activeInHierarchy) { ReadRadius(); return true; }
        _goal = null; _goalRenderer = null;
        foreach (var r in FindObjectsOfType<MeshRenderer>(false))   // active only
        {
            string n = r.gameObject.name.ToLowerInvariant();
            if (n.Contains(goalObjectName.ToLowerInvariant()) && !n.Contains("beam"))
            {
                _goal = r.transform; _goalRenderer = r;
                if (hideBakedRing) r.enabled = false;   // show ours instead
                ReadRadius();
                return true;
            }
        }
        return false;
    }

    void ReadRadius()
    {
        if (_goalRenderer != null)
        {
            var e = _goalRenderer.bounds.extents;     // world-space, already scaled
            _radius = Mathf.Max(e.x, e.z);
            if (_radius < 1e-3f) _radius = fallbackRadius;
        }
        else _radius = fallbackRadius;
    }

    // ---------- marker ----------
    void BuildMarker()
    {
        _marker = new GameObject("GoalZoneMarker");
        _disc = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        _disc.name = "goalDisc";
        Destroy(_disc.GetComponent<Collider>());
        _disc.transform.SetParent(_marker.transform, false);
        _disc.GetComponent<Renderer>().sharedMaterial = _mat;

        _beam = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        _beam.name = "goalBeam";
        Destroy(_beam.GetComponent<Collider>());
        _beam.transform.SetParent(_marker.transform, false);
        _beam.GetComponent<Renderer>().sharedMaterial = _mat;
    }

    void UpdateMarker()
    {
        if (!showMarker || _marker == null || _goal == null) return;
        Vector3 g = _goal.position;
        _marker.transform.position = g;
        // flat disc on the floor
        _disc.transform.localPosition = new Vector3(0, 0.01f, 0);
        _disc.transform.localScale = new Vector3(_radius * 2f, 0.01f, _radius * 2f);
        // thin light beam rising from the centre
        _beam.transform.localPosition = new Vector3(0, beamHeight * 0.5f, 0);
        _beam.transform.localScale = new Vector3(_radius * 0.5f, beamHeight * 0.5f, _radius * 0.5f);
    }

    void TintMarker(bool success)
    {
        if (_mat == null) return;
        var c = success ? successColor : pendingColor;
        _mat.color = c;
        if (_mat.HasProperty("_BaseColor")) _mat.SetColor("_BaseColor", c);
    }

    // ---------- helpers ----------
    AudioClip MakeChime()
    {
        const int sr = 44100;
        const float dur = 0.22f;
        int n = (int)(sr * dur);
        var data = new float[n];
        for (int i = 0; i < n; i++)
        {
            float t = (float)i / sr;
            float freq = t < dur * 0.5f ? 880f : 1320f;    // two-tone rising chime
            float env = Mathf.Clamp01(Mathf.Min(t * 60f, (dur - t) * 60f));
            data[i] = Mathf.Sin(2f * Mathf.PI * freq * t) * 0.5f * env;
        }
        var clip = AudioClip.Create("goalChime", n, 1, sr, false);
        clip.SetData(data, 0);
        return clip;
    }

    static Material MakeUnlit(Color c)
    {
        Shader sh = Shader.Find("Unlit/Color") ?? Shader.Find("Standard");
        var m = new Material(sh) { color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        return m;
    }

    void SetStatus(string s) { _status = "[GoalZone] " + s; }

    void OnGUI()
    {
        if (!showStatus) return;
        var st = new GUIStyle(GUI.skin.label) { fontSize = 13 };
        st.normal.textColor = _reached ? Color.green : Color.yellow;
        GUI.Label(new Rect(10, 408, 1200, 24), _status, st);
    }

    void OnDrawGizmos()
    {
        // Edit-mode visibility: draw the goal ring in the scene view too.
        var t = _goal;
        if (t == null)
        {
            foreach (var r in FindObjectsOfType<MeshRenderer>(false))
                if (r.gameObject.name.ToLowerInvariant().Contains("goal_ring") &&
                    !r.gameObject.name.ToLowerInvariant().Contains("beam")) { t = r.transform; break; }
        }
        if (t == null) return;
        Gizmos.color = new Color(1f, 0.85f, 0.2f, 0.9f);
        Gizmos.DrawWireSphere(t.position, fallbackRadius);
    }

#if UNITY_EDITOR
    [UnityEditor.MenuItem("MTC/Add Goal Zone")]
    static void AddToScene()
    {
        var go = new GameObject("GoalZone");
        go.AddComponent<GoalZone>();
        UnityEditor.Undo.RegisterCreatedObjectUndo(go, "Add GoalZone");
        UnityEditor.Selection.activeGameObject = go;
    }
#endif
}
