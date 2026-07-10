// ===================== CsvReplayTrimmer.cs =====================
// In-scene CSV trajectory trimmer with collision proximity feedback.
//
// Usage:
// 1) Add this script to any GameObject in your replay scene.
// 2) Set csvFilePath to your recording CSV (absolute path).
// 3) Assign replayOrigin (TrackingOrigin / XR Rig root) if needed.
// 4) Optionally assign environmentRoot (parent of FBX scene objects)
//    so the script can auto-find MeshColliders.
// 5) Press Play in Unity Editor.
//
// Runtime controls (IMGUI overlay at bottom of Game view):
//   - Timeline slider to scrub frames
//   - Play / Pause / Step buttons
//   - Set Start / Set End / Save / Reset buttons
//   - Playback speed control
//   - Collision proximity readout
//
// Clip categories (selectable per save):
//   easy  -- 无难度动作 (plain walking on open floor)
//   avoid -- 避障动作   (active obstacle avoidance)
//   near  -- 危险动作   (no collision but very close to wall, flagged)
//   bad   -- 问题动作   (sanity check fails -- clip collides)
// Output filename: <prefix>_<category><NN>.csv   (per-category counter).
// A '# clip_category=<cat>' meta line is also written into the saved CSV.
//
// Keyboard shortcuts:
//   Space       Play / Pause
//   Left/Right  Step -1 / +1 frame
//   [  /  ]     Step -10 / +10 frames
//   A           Set trim START at current frame
//   D           Set trim END   at current frame
//   1/2/3/4     Category = easy / avoid / near / bad
//   S           Save current [start, end] segment to disk
//   R           Reset start / end markers

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public class CsvReplayTrimmer : MonoBehaviour
{
    // ── Inspector fields ──────────────────────────────────────────────
    [Header("CSV Input")]
    [Tooltip("Absolute path to the 24-joint body-tracking CSV file.")]
    public string csvFilePath = "";

    [Tooltip("Output directory for trimmed CSVs (defaults to same folder as input).")]
    public string outputDir = "";

    [Tooltip("Filename prefix for saved segments. Leave EMPTY (default) to use the " +
             "input CSV's PARENT FOLDER NAME (e.g. 'corridor_medium_00000201') -- " +
             "much cleaner than the raw file stem when many recordings share generic " +
             "names like 'AvatarRoot_manual_19.csv'. Set explicitly only to override.")]
    public string outputNamePrefix = "";

    [Header("Origin Mapping")]
    [Tooltip("Enable origin mapping (record_origin → replay scene origin).")]
    public bool applyOriginMapping = true;
    public Transform replayOrigin;

    [Header("Collision Detection")]
    [Tooltip("Parent transform containing the FBX environment meshes. " +
             "If null, the script searches for MeshColliders in the scene.")]
    public Transform environmentRoot;
    [Tooltip("Safety margin added to G1 capsule radii (m).")]
    public float safetyMargin = 0.02f;

    [Header("Skeleton Visualisation")]
    public bool drawGizmos = true;
    public float jointSize = 0.025f;
    public Color jointColor = new Color(0.1f, 0.8f, 1f, 1f);
    public Color boneColor = new Color(1f, 0.9f, 0.2f, 1f);
    public Color collisionColor = new Color(1f, 0.2f, 0.1f, 1f);

    [Header("Bone Thickness")]
    public bool thickBones = true;
    public float boneThickness = 0.01f;
    public int boneRays = 6;

    [Header("Editor Camera (auto-created in Editor)")]
    [Tooltip("Auto-create a fly camera in Editor so XR camera is not needed.")]
    public bool autoEditorCamera = true;
    [Tooltip("WASD move speed (m/s).")]
    public float camMoveSpeed = 3f;
    [Tooltip("Mouse look sensitivity.")]
    public float camLookSpeed = 2f;

    // ── Clip category ─────────────────────────────────────────────────
    public enum ClipCategory { Easy, Avoid, Near, Bad }

    [Header("Clip Category")]
    [Tooltip("Default category for saved trimmed clips. " +
             "Switch live in-scene with keys 1/2/3/4 or the toolbar buttons.")]
    [SerializeField] private ClipCategory _category = ClipCategory.Easy;

    // ── CSV data ──────────────────────────────────────────────────────
    private string[] _headerLine;       // raw header columns
    private List<string> _metaLines;    // raw '#'-prefixed lines from the input (preserved on save)
    private List<string> _rawLines;     // raw CSV lines (no header / meta)
    private List<Vector3[]> _frames;    // parsed 24-joint positions per frame
    private List<float> _times;         // time column per frame
    private List<int> _frameIds;        // frame column per frame
    private int _totalFrames;

    // record origin meta
    private bool _hasRecordOrigin;
    private Vector3 _recordOriginPos;
    private Quaternion _recordOriginRot;
    private Matrix4x4 _M = Matrix4x4.identity;
    private bool _MReady;

    // ── Playback state ────────────────────────────────────────────────
    private int _current;               // current frame index (0-based)
    private bool _playing;
    private float _playTimer;
    private float _playbackSpeed = 1f;

    // ── Trim state ────────────────────────────────────────────────────
    private int _trimStart = -1;
    private int _trimEnd = -1;
    private int _saveCounter;

    // ── Collision state ───────────────────────────────────────────────
    private MeshCollider[] _envColliders;
    private float _minCollisionDist = float.MaxValue;
    private string _closestBodyPart = "";
    private Vector3 _closestPointOnEnv;
    private Vector3 _closestBodyPoint;
    private int _collisionCount;        // capsules within warning distance

    // ── Current displayed joints (after mapping) ──────────────────────
    private readonly Vector3[] _displayJoints = new Vector3[24];

    // ── Skeleton topology ─────────────────────────────────────────────
    private static readonly (int a, int b)[] Bones = {
        // spine
        (0,3),(3,6),(6,9),(9,12),(12,15),
        // left leg
        (0,1),(1,4),(4,7),(7,10),
        // right leg
        (0,2),(2,5),(5,8),(8,11),
        // left arm
        (9,13),(13,16),(16,18),(18,20),(20,22),
        // right arm
        (9,14),(14,17),(17,19),(19,21),(21,23),
    };

    // G1 body-part capsules: (startJoint, endJoint, radius, name)
    private static readonly (int s, int e, float r, string n)[] BodyCapsules = {
        (0,3, 0.11f, "torso_lower"),
        (3,6, 0.11f, "torso_mid"),
        (6,9, 0.11f, "torso_upper"),
        (9,12, 0.088f, "neck_base"),
        (12,15, 0.10f, "head"),
        (16,18, 0.055f, "L_upper_arm"), (18,20, 0.05f, "L_forearm"),
        (17,19, 0.055f, "R_upper_arm"), (19,21, 0.05f, "R_forearm"),
        (1,4, 0.06f, "L_thigh"), (4,7, 0.055f, "L_shin"),
        (2,5, 0.06f, "R_thigh"), (5,8, 0.055f, "R_shin"),
    };

    // ── GUI layout ────────────────────────────────────────────────────
    private GUIStyle _boxStyle;
    private GUIStyle _labelStyle;
    private GUIStyle _btnStyle;
    private GUIStyle _labelBold;
    private bool _stylesInit;
    private string _statusMsg = "";
    private float _statusTime;

    // ── Draggable handle state ────────────────────────────────────────
    private enum DragTarget { None, Playhead, TrimStart, TrimEnd }
    private DragTarget _dragging = DragTarget.None;

    // timeline geometry (computed once per OnGUI)
    private float _tlLeft, _tlRight, _tlY, _tlH;
    private const float HANDLE_W = 10f;   // half-width of grab handle
    private const float TL_HEIGHT = 32f;  // timeline track height

    // ── Editor fly camera ─────────────────────────────────────────────
    private Camera _editorCam;
    private float _camYaw, _camPitch;
    private bool _mouseHeld;            // right-mouse held for look

    // ==================================================================
    // MonoBehaviour lifecycle
    // ==================================================================
    void Start()
    {
        _frames = new List<Vector3[]>();
        _metaLines = new List<string>();
        _rawLines = new List<string>();
        _times = new List<float>();
        _frameIds = new List<int>();

        if (!string.IsNullOrEmpty(csvFilePath))
            LoadCsv(csvFilePath);

        FindEnvironmentColliders();
        SetupEditorCamera();
    }

    void OnDestroy()
    {
        // clean up the auto-created camera
        if (_editorCam != null)
            Destroy(_editorCam.gameObject);
    }

    void Update()
    {
        // ── editor fly camera ──
        UpdateEditorCamera();

        if (_totalFrames == 0) return;

        HandleKeyboard();

        // playback
        if (_playing)
        {
            _playTimer += Time.deltaTime * _playbackSpeed;
            float dt = (_times.Count > 1)
                ? Mathf.Max(0.001f, _times[1] - _times[0])
                : 1f / 60f;
            int advance = Mathf.FloorToInt(_playTimer / dt);
            if (advance > 0)
            {
                _playTimer -= advance * dt;
                _current = Mathf.Min(_current + advance, _totalFrames - 1);
                if (_current >= _totalFrames - 1)
                    _playing = false;
            }
        }

        // build display joints
        EnsureMappingMatrix();
        var src = _frames[_current];
        for (int i = 0; i < 24; i++)
        {
            Vector3 p = src[i];
            if (applyOriginMapping && _MReady)
                p = _M.MultiplyPoint3x4(p);
            _displayJoints[i] = p;
        }

        // collision check
        CheckCollision();
    }

    // ==================================================================
    // Editor fly camera  (WASD + right-mouse look)
    // ==================================================================
    void SetupEditorCamera()
    {
        #if UNITY_EDITOR
        if (!autoEditorCamera) return;

        // create camera GameObject
        var go = new GameObject("_TrimmerEditorCam");
        _editorCam = go.AddComponent<Camera>();
        _editorCam.depth = 100;              // render on top of everything
        _editorCam.nearClipPlane = 0.05f;
        _editorCam.farClipPlane = 500f;
        _editorCam.fieldOfView = 60f;
        _editorCam.clearFlags = CameraClearFlags.Skybox;
        go.AddComponent<AudioListener>();

        // disable other AudioListeners to avoid warning
        foreach (var al in FindObjectsOfType<AudioListener>())
        {
            if (al.gameObject != go)
                al.enabled = false;
        }

        // position: if we have CSV data, look at first-frame pelvis;
        // otherwise put it at a sensible default
        Vector3 target = new Vector3(0, 1.2f, 0);
        if (_totalFrames > 0)
            target = _frames[0][0]; // pelvis of first frame

        go.transform.position = target + new Vector3(0, 0.5f, -2f);
        go.transform.LookAt(target);

        _camYaw   = go.transform.eulerAngles.y;
        _camPitch = go.transform.eulerAngles.x;
        if (_camPitch > 180f) _camPitch -= 360f;

        Debug.Log("[CsvTrimmer] Editor fly camera created. Right-click + WASD to navigate.");
        #endif
    }

    void UpdateEditorCamera()
    {
        #if UNITY_EDITOR
        if (_editorCam == null) return;

        // right-mouse look
        if (Input.GetMouseButtonDown(1)) _mouseHeld = true;
        if (Input.GetMouseButtonUp(1))   _mouseHeld = false;

        if (_mouseHeld)
        {
            _camYaw   += Input.GetAxis("Mouse X") * camLookSpeed;
            _camPitch -= Input.GetAxis("Mouse Y") * camLookSpeed;
            _camPitch  = Mathf.Clamp(_camPitch, -89f, 89f);
        }

        _editorCam.transform.rotation = Quaternion.Euler(_camPitch, _camYaw, 0);

        // WASD + QE move (only when right-mouse held, so keys don't conflict with trim shortcuts)
        if (_mouseHeld)
        {
            float speed = camMoveSpeed * Time.deltaTime;
            if (Input.GetKey(KeyCode.LeftShift)) speed *= 3f;

            Transform t = _editorCam.transform;
            if (Input.GetKey(KeyCode.W)) t.position += t.forward * speed;
            if (Input.GetKey(KeyCode.S)) t.position -= t.forward * speed;
            if (Input.GetKey(KeyCode.A)) t.position -= t.right   * speed;
            if (Input.GetKey(KeyCode.D)) t.position += t.right   * speed;
            if (Input.GetKey(KeyCode.E)) t.position += Vector3.up * speed;
            if (Input.GetKey(KeyCode.Q)) t.position -= Vector3.up * speed;
        }

        // scroll wheel zoom
        float scroll = Input.GetAxis("Mouse ScrollWheel");
        if (Mathf.Abs(scroll) > 0.001f)
            _editorCam.transform.position += _editorCam.transform.forward * scroll * camMoveSpeed * 2f;
        #endif
    }

    // ==================================================================
    // CSV Loading (same format as PXR_24PointCsvPlayer)
    // ==================================================================
    public void LoadCsv(string path)
    {
        _frames.Clear();
        _metaLines.Clear();
        _rawLines.Clear();
        _times.Clear();
        _frameIds.Clear();
        _totalFrames = 0;
        _current = 0;
        _playing = false;
        _playTimer = 0;
        _MReady = false;
        _hasRecordOrigin = false;
        _recordOriginPos = Vector3.zero;
        _recordOriginRot = Quaternion.identity;
        _trimStart = -1;
        _trimEnd = -1;

        if (!File.Exists(path))
        {
            Debug.LogError($"[CsvTrimmer] CSV not found: {path}");
            return;
        }

        var allLines = File.ReadAllLines(path);
        if (allLines.Length <= 1) { Debug.LogError("[CsvTrimmer] CSV is empty."); return; }

        // parse meta + find header
        int headerIdx = -1;
        for (int i = 0; i < allLines.Length; i++)
        {
            string line = allLines[i].Trim();
            if (string.IsNullOrEmpty(line)) continue;
            if (line.StartsWith("#")) { _metaLines.Add(allLines[i]); ParseMeta(line); continue; }
            headerIdx = i;
            break;
        }
        if (headerIdx < 0) { Debug.LogError("[CsvTrimmer] No CSV header found."); return; }

        // store header
        // Remove BOM if present
        string headerRaw = allLines[headerIdx];
        if (headerRaw.Length > 0 && headerRaw[0] == '\uFEFF')
            headerRaw = headerRaw.Substring(1);
        _headerLine = headerRaw.Split(',');

        // parse data
        for (int li = headerIdx + 1; li < allLines.Length; li++)
        {
            string line = allLines[li].Trim();
            if (string.IsNullOrEmpty(line) || line.StartsWith("#")) continue;

            string[] cols = line.Split(',');
            if (cols.Length < 170) continue;

            _rawLines.Add(allLines[li]);   // keep raw line for saving

            int frameId = ParseInt(cols[0]);
            float time = ParseF(cols[1]);
            _frameIds.Add(frameId);
            _times.Add(time);

            Vector3[] joints = new Vector3[24];
            int baseCol = (cols.Length >= 177) ? 9 : 2;
            for (int j = 0; j < 24; j++)
            {
                int idx = baseCol + j * 7;
                joints[j] = new Vector3(ParseF(cols[idx]), ParseF(cols[idx + 1]), ParseF(cols[idx + 2]));
            }
            _frames.Add(joints);
        }

        _totalFrames = _frames.Count;
        if (_totalFrames > 0)
            Array.Copy(_frames[0], _displayJoints, 24);

        if (string.IsNullOrEmpty(outputDir))
            outputDir = Path.GetDirectoryName(path);

        Debug.Log($"[CsvTrimmer] Loaded {_totalFrames} frames from {Path.GetFileName(path)}  " +
                  $"time {_times[0]:F3}–{_times[_totalFrames - 1]:F3}s  hasOrigin={_hasRecordOrigin}");
    }

    // ==================================================================
    // Collision detection against environment meshes
    // ==================================================================
    void FindEnvironmentColliders()
    {
        if (environmentRoot != null)
        {
            _envColliders = environmentRoot.GetComponentsInChildren<MeshCollider>(false);
        }
        else
        {
            _envColliders = FindObjectsOfType<MeshCollider>();
        }
        Debug.Log($"[CsvTrimmer] Found {_envColliders.Length} MeshColliders for collision check.");
    }

    void CheckCollision()
    {
        _minCollisionDist = float.MaxValue;
        _closestBodyPart = "";
        _collisionCount = 0;

        if (_envColliders == null || _envColliders.Length == 0) return;

        foreach (var (s, e, r, name) in BodyCapsules)
        {
            Vector3 startP = _displayJoints[s];
            Vector3 endP = _displayJoints[e];
            float radius = r + safetyMargin;

            // sample 5 points along capsule
            for (int i = 0; i < 5; i++)
            {
                Vector3 pt = Vector3.Lerp(startP, endP, i / 4f);
                foreach (var mc in _envColliders)
                {
                    if (mc == null || !mc.gameObject.activeInHierarchy) continue;
                    Vector3 closest = mc.ClosestPoint(pt);
                    float dist = Vector3.Distance(pt, closest) - radius;
                    if (dist < _minCollisionDist)
                    {
                        _minCollisionDist = dist;
                        _closestBodyPart = name;
                        _closestPointOnEnv = closest;
                        _closestBodyPoint = pt;
                    }
                }
            }

            // count collisions (dist < 0 means overlap)
            if (_minCollisionDist < 0.05f)
                _collisionCount++;
        }
    }

    // ==================================================================
    // Keyboard input
    // ==================================================================
    void HandleKeyboard()
    {
        if (Event.current == null) return; // OnGUI handles key events
    }

    // ==================================================================
    // IMGUI Overlay
    // ==================================================================
    void OnGUI()
    {
        if (_totalFrames == 0)
        {
            GUI.Label(new Rect(10, 10, 400, 30),
                "CsvReplayTrimmer: No CSV loaded. Set csvFilePath in Inspector.");
            return;
        }

        InitStyles();

        // ── keyboard shortcuts (must live in OnGUI) ──
        // Only process when NOT in fly-camera mode (right-mouse held)
        Event e = Event.current;
        if (e.type == EventType.KeyDown && !_mouseHeld)
        {
            switch (e.keyCode)
            {
                case KeyCode.Space:        _playing = !_playing; e.Use(); break;
                case KeyCode.RightArrow:   Step(1);   e.Use(); break;
                case KeyCode.LeftArrow:    Step(-1);  e.Use(); break;
                case KeyCode.RightBracket: Step(10);  e.Use(); break;
                case KeyCode.LeftBracket:  Step(-10); e.Use(); break;
                case KeyCode.A: SetStart();    e.Use(); break;
                case KeyCode.D: SetEnd();      e.Use(); break;
                case KeyCode.Alpha1: SetCategory(ClipCategory.Easy);  e.Use(); break;
                case KeyCode.Alpha2: SetCategory(ClipCategory.Avoid); e.Use(); break;
                case KeyCode.Alpha3: SetCategory(ClipCategory.Near);  e.Use(); break;
                case KeyCode.Alpha4: SetCategory(ClipCategory.Bad);   e.Use(); break;
                case KeyCode.S: SaveSegment(); e.Use(); break;
                case KeyCode.R: ResetMarkers();e.Use(); break;
            }
        }

        float sw  = Screen.width;
        float pad = 10f;

        // total panel height (bumped from 190 to fit the Category row)
        float panelH = 225f;
        float panelY = Screen.height - panelH;

        // ── background ──
        GUI.Box(new Rect(0, panelY, sw, panelH), "", _boxStyle);

        float y = panelY + 6;

        // ==============================================================
        // Row 1 — Frame info  |  Collision info
        // ==============================================================
        string frameInfo =
            $"Frame {_current} / {_totalFrames - 1}    " +
            $"id={_frameIds[_current]}    " +
            $"t={_times[_current]:F3}s    " +
            $"Speed {_playbackSpeed:F1}x";
        GUI.Label(new Rect(pad, y, sw * 0.52f, 20), frameInfo, _labelStyle);

        // collision
        string collInfo; Color collC;
        CollisionLabel(out collInfo, out collC);
        var savedCC = GUI.contentColor;
        GUI.contentColor = collC;
        GUI.Label(new Rect(sw * 0.54f, y, sw * 0.45f, 20), collInfo, _labelStyle);
        GUI.contentColor = savedCC;

        y += 22;

        // ==============================================================
        // Row 2 — Visual timeline bar with draggable handles
        // ==============================================================
        float tlLeft  = pad + 30;           // leave room for "[" handle
        float tlRight = sw - pad - 30;
        float tlW     = tlRight - tlLeft;
        _tlLeft  = tlLeft;
        _tlRight = tlRight;
        _tlY     = y;
        _tlH     = TL_HEIGHT;

        // -- track background (dark grey) --
        DrawRect(new Rect(tlLeft, y, tlW, TL_HEIGHT), new Color(0.15f, 0.15f, 0.15f, 1f));

        // -- selected range highlight (blue tint) --
        if (_trimStart >= 0 && _trimEnd >= 0)
        {
            int lo = Mathf.Min(_trimStart, _trimEnd);
            int hi = Mathf.Max(_trimStart, _trimEnd);
            float xLo = FrameToTlX(lo);
            float xHi = FrameToTlX(hi);
            DrawRect(new Rect(xLo, y, xHi - xLo, TL_HEIGHT),
                     new Color(0.2f, 0.5f, 0.9f, 0.30f));
        }

        // -- tick marks (every 10% of total frames) --
        for (int pct = 0; pct <= 100; pct += 10)
        {
            float tx = tlLeft + tlW * pct / 100f;
            bool major = (pct % 50 == 0);
            DrawRect(new Rect(tx, y + (major ? 0 : TL_HEIGHT * 0.6f),
                              1, major ? TL_HEIGHT : TL_HEIGHT * 0.4f),
                     new Color(1, 1, 1, major ? 0.35f : 0.15f));
        }

        // -- playhead (white vertical line) --
        float phX = FrameToTlX(_current);
        DrawRect(new Rect(phX - 1, y, 3, TL_HEIGHT), Color.white);
        // playhead triangle at top
        DrawTriangle(phX, y - 2, 6, Color.white);

        // -- trim START handle (green bracket) --
        if (_trimStart >= 0)
        {
            float sx = FrameToTlX(_trimStart);
            DrawTrimHandle(sx, y, TL_HEIGHT, new Color(0.1f, 0.9f, 0.2f, 1f), true);
            // time label
            GUI.contentColor = new Color(0.1f, 0.9f, 0.2f);
            GUI.Label(new Rect(sx - 30, y + TL_HEIGHT + 1, 70, 16),
                      $"{_times[_trimStart]:F2}s", _labelSmall);
            GUI.contentColor = savedCC;
        }

        // -- trim END handle (red bracket) --
        if (_trimEnd >= 0)
        {
            float ex = FrameToTlX(_trimEnd);
            DrawTrimHandle(ex, y, TL_HEIGHT, new Color(0.95f, 0.2f, 0.15f, 1f), false);
            // time label
            GUI.contentColor = new Color(0.95f, 0.2f, 0.15f);
            GUI.Label(new Rect(ex - 10, y + TL_HEIGHT + 1, 70, 16),
                      $"{_times[_trimEnd]:F2}s", _labelSmall);
            GUI.contentColor = savedCC;
        }

        // -- handle dragging --
        HandleTimelineDrag(e, y);

        y += TL_HEIGHT + 18;

        // ==============================================================
        // Row 3 — Playback buttons  +  Speed
        // ==============================================================
        float bh = 26, gap = 4;
        float bx = pad;

        if (GUI.Button(new Rect(bx, y, 70, bh), _playing ? "Pause" : "Play", _btnStyle))
            _playing = !_playing;
        bx += 74;
        if (GUI.Button(new Rect(bx, y, 36, bh), "<<",  _btnStyle)) Step(-10);
        bx += 40;
        if (GUI.Button(new Rect(bx, y, 28, bh), "<",   _btnStyle)) Step(-1);
        bx += 32;
        if (GUI.Button(new Rect(bx, y, 28, bh), ">",   _btnStyle)) Step(1);
        bx += 32;
        if (GUI.Button(new Rect(bx, y, 36, bh), ">>",  _btnStyle)) Step(10);
        bx += 46;

        GUI.Label(new Rect(bx, y + 4, 40, 20), "Spd:", _labelStyle);
        bx += 36;
        if (GUI.Button(new Rect(bx, y, 35, bh), "0.5", _btnStyle)) _playbackSpeed = 0.5f;
        bx += 39;
        if (GUI.Button(new Rect(bx, y, 28, bh), "1",   _btnStyle)) _playbackSpeed = 1f;
        bx += 32;
        if (GUI.Button(new Rect(bx, y, 28, bh), "2",   _btnStyle)) _playbackSpeed = 2f;

        y += bh + 5;

        // ==============================================================
        // Row 3.5 — Category selector (easy / avoid / near / bad)
        // ==============================================================
        GUI.Label(new Rect(pad, y + 4, 70, 20), "Category:", _labelStyle);
        string[] catLabels = { "Easy [1]", "Avoid [2]", "Near [3]", "Bad [4]" };
        int newCat = GUI.Toolbar(new Rect(pad + 70, y, 360, bh),
                                 (int)_category, catLabels, _btnStyle);
        if (newCat != (int)_category)
            SetCategory((ClipCategory)newCat);

        y += bh + 5;

        // ==============================================================
        // Row 4 — Trim buttons  +  Trim info
        // ==============================================================
        bx = pad;
        if (GUI.Button(new Rect(bx, y, 90, bh), "Set Start [A]", _btnStyle)) SetStart();
        bx += 94;
        if (GUI.Button(new Rect(bx, y, 90, bh), "Set End [D]",   _btnStyle)) SetEnd();
        bx += 94;
        if (GUI.Button(new Rect(bx, y, 80, bh), "Save [S]",      _btnStyle)) SaveSegment();
        bx += 84;
        if (GUI.Button(new Rect(bx, y, 80, bh), "Reset [R]",     _btnStyle)) ResetMarkers();
        bx += 84;

        // trim summary
        string trimInfo = BuildTrimInfo();
        GUI.Label(new Rect(bx + 6, y + 4, 400, 20), trimInfo, _labelStyle);

        y += bh + 4;

        // ==============================================================
        // Row 5 — Status toast
        // ==============================================================
        if (Time.time - _statusTime < 4f && !string.IsNullOrEmpty(_statusMsg))
        {
            GUI.contentColor = Color.cyan;
            GUI.Label(new Rect(pad, y, sw - pad * 2, 20), _statusMsg, _labelStyle);
            GUI.contentColor = savedCC;
        }
    }

    // ── Timeline helpers ──────────────────────────────────────────────

    /// Map frame index → screen X on the timeline track
    float FrameToTlX(int frame)
    {
        float t = (_totalFrames > 1) ? (float)frame / (_totalFrames - 1) : 0f;
        return Mathf.Lerp(_tlLeft, _tlRight, t);
    }

    /// Map screen X → frame index
    int TlXToFrame(float x)
    {
        float t = Mathf.InverseLerp(_tlLeft, _tlRight, x);
        return Mathf.Clamp(Mathf.RoundToInt(t * (_totalFrames - 1)), 0, _totalFrames - 1);
    }

    /// Draw a trim handle (bracket shape) at x
    void DrawTrimHandle(float x, float y, float h, Color c, bool isStart)
    {
        // vertical bar
        DrawRect(new Rect(x - 2, y, 4, h), c);

        // horizontal flag (bracket arm)
        float flagW = 12f;
        float armX  = isStart ? (x - flagW) : x;

        // top arm
        DrawRect(new Rect(armX, y, flagW, 3), c);
        // bottom arm
        DrawRect(new Rect(armX, y + h - 3, flagW, 3), c);

        // grab region indicator (small filled rect)
        float knobX = isStart ? (x - flagW) : (x + 2);
        DrawRect(new Rect(knobX, y + h * 0.3f, flagW - 2, h * 0.4f),
                 new Color(c.r, c.g, c.b, 0.45f));
    }

    /// Draw a small downward-pointing triangle (playhead indicator)
    void DrawTriangle(float cx, float tipY, float size, Color c)
    {
        // approximate with 3 thin rects (IMGUI has no triangle primitive)
        var prev = GUI.color;
        GUI.color = c;
        for (int i = 0; i < (int)size; i++)
        {
            float w = size - i;
            GUI.DrawTexture(new Rect(cx - w, tipY - size + i, w * 2, 1),
                            Texture2D.whiteTexture);
        }
        GUI.color = prev;
    }

    /// Handle mouse drag on the timeline
    void HandleTimelineDrag(Event e, float tlTop)
    {
        Rect tlRect = new Rect(_tlLeft - HANDLE_W, tlTop - 4,
                               _tlRight - _tlLeft + HANDLE_W * 2,
                               TL_HEIGHT + 8);

        if (e.type == EventType.MouseDown && e.button == 0 && tlRect.Contains(e.mousePosition))
        {
            float mx = e.mousePosition.x;

            // check which handle is closest
            _dragging = DragTarget.Playhead;  // default

            float bestDist = Mathf.Abs(mx - FrameToTlX(_current));

            if (_trimStart >= 0)
            {
                float d = Mathf.Abs(mx - FrameToTlX(_trimStart));
                if (d < HANDLE_W + 6 && d < bestDist) { _dragging = DragTarget.TrimStart; bestDist = d; }
            }
            if (_trimEnd >= 0)
            {
                float d = Mathf.Abs(mx - FrameToTlX(_trimEnd));
                if (d < HANDLE_W + 6 && d < bestDist) { _dragging = DragTarget.TrimEnd; bestDist = d; }
            }

            ApplyDrag(mx);
            e.Use();
        }
        else if (e.type == EventType.MouseDrag && e.button == 0 && _dragging != DragTarget.None)
        {
            ApplyDrag(e.mousePosition.x);
            e.Use();
        }
        else if (e.type == EventType.MouseUp && e.button == 0)
        {
            _dragging = DragTarget.None;
        }
    }

    void ApplyDrag(float mx)
    {
        int frame = TlXToFrame(mx);
        switch (_dragging)
        {
            case DragTarget.Playhead:
                _current = frame;
                _playing = false;
                break;
            case DragTarget.TrimStart:
                _trimStart = frame;
                break;
            case DragTarget.TrimEnd:
                _trimEnd = frame;
                break;
        }
    }

    // ── Drawing primitives ────────────────────────────────────────────

    void DrawRect(Rect r, Color c)
    {
        var prev = GUI.color;
        GUI.color = c;
        GUI.DrawTexture(r, Texture2D.whiteTexture);
        GUI.color = prev;
    }

    // ── Info strings ──────────────────────────────────────────────────

    void CollisionLabel(out string text, out Color color)
    {
        if (_minCollisionDist < 0)
        {
            text  = $"COLLISION  dist={_minCollisionDist * 100:F1}cm  part={_closestBodyPart}";
            color = Color.red;
        }
        else if (_minCollisionDist < 0.05f)
        {
            text  = $"WARNING  dist={_minCollisionDist * 100:F1}cm  part={_closestBodyPart}";
            color = Color.yellow;
        }
        else if (_minCollisionDist < 0.10f)
        {
            text  = $"NEAR  dist={_minCollisionDist * 100:F1}cm  part={_closestBodyPart}";
            color = new Color(1, 0.6f, 0);
        }
        else
        {
            text  = $"Clear  minDist={_minCollisionDist * 100:F1}cm";
            color = Color.green;
        }
    }

    string BuildTrimInfo()
    {
        string s = $"[{CategoryToken(_category)}]  ";
        if (_trimStart >= 0) s += $"Start={_trimStart}";
        if (_trimEnd >= 0)   s += $"  End={_trimEnd}";
        if (_trimStart >= 0 && _trimEnd >= 0)
        {
            int lo  = Mathf.Min(_trimStart, _trimEnd);
            int hi  = Mathf.Max(_trimStart, _trimEnd);
            int len = hi - lo + 1;
            float dur = _times[hi] - _times[lo];
            s += $"  ({len} frames, {dur:F2}s)";
        }
        return s;
    }

    // ── Style init ────────────────────────────────────────────────────
    private GUIStyle _labelSmall;

    void InitStyles()
    {
        if (_stylesInit) return;
        _stylesInit = true;

        _boxStyle = new GUIStyle(GUI.skin.box);
        var bgTex = new Texture2D(1, 1);
        bgTex.SetPixel(0, 0, new Color(0, 0, 0, 0.80f));
        bgTex.Apply();
        _boxStyle.normal.background = bgTex;

        _labelStyle = new GUIStyle(GUI.skin.label)
        { fontSize = 13 };
        _labelStyle.normal.textColor = Color.white;

        _labelBold = new GUIStyle(_labelStyle)
        { fontStyle = FontStyle.Bold };

        _labelSmall = new GUIStyle(GUI.skin.label)
        { fontSize = 10 };
        _labelSmall.normal.textColor = Color.white;

        _btnStyle = new GUIStyle(GUI.skin.button)
        { fontSize = 12 };
    }

    // ==================================================================
    // Trim actions
    // ==================================================================
    void Step(int delta)
    {
        _current = Mathf.Clamp(_current + delta, 0, _totalFrames - 1);
        _playing = false;
    }

    void SetStart()
    {
        _trimStart = _current;
        ShowStatus($"Trim START set to frame {_current} (t={_times[_current]:F3}s)");
    }

    void SetEnd()
    {
        _trimEnd = _current;
        ShowStatus($"Trim END set to frame {_current} (t={_times[_current]:F3}s)");
    }

    void ResetMarkers()
    {
        _trimStart = -1;
        _trimEnd = -1;
        ShowStatus("Trim markers reset.");
    }

    void SetCategory(ClipCategory cat)
    {
        _category = cat;
        ShowStatus($"Category set to [{CategoryToken(cat)}].");
    }

    static string CategoryToken(ClipCategory c)
    {
        switch (c)
        {
            case ClipCategory.Easy:  return "easy";
            case ClipCategory.Avoid: return "avoid";
            case ClipCategory.Near:  return "near";
            case ClipCategory.Bad:   return "bad";
            default: return "unknown";
        }
    }

    void SaveSegment()
    {
        if (_trimStart < 0 || _trimEnd < 0)
        {
            ShowStatus("ERROR: Set both Start and End before saving.");
            return;
        }

        int lo = Mathf.Min(_trimStart, _trimEnd);
        int hi = Mathf.Max(_trimStart, _trimEnd);

        // Build the filename prefix:
        //   1) explicit outputNamePrefix override (Inspector)
        //   2) parent folder name of csvFilePath (e.g. "corridor_medium_00000201")
        //   3) fall back to the input file stem (if no folder above)
        string prefix = outputNamePrefix;
        if (string.IsNullOrWhiteSpace(prefix))
        {
            string parentDir = Path.GetDirectoryName(csvFilePath);
            prefix = string.IsNullOrEmpty(parentDir) ? "" : Path.GetFileName(parentDir);
        }
        if (string.IsNullOrWhiteSpace(prefix))
            prefix = Path.GetFileNameWithoutExtension(csvFilePath);

        // Category token: easy / avoid / near / bad. Used as filename infix + meta tag.
        string cat = CategoryToken(_category);

        // Find the next free index by SCANNING outputDir for "<prefix>_<cat>##.csv".
        // Each category keeps its OWN counter, so e.g. easy01..easy07 and
        // avoid01..avoid03 coexist and stay easy to inventory.
        // This survives Unity restarts: counter no longer starts back at 01 and
        // overwrites your previous saves.
        int nextIdx = 1;
        if (Directory.Exists(outputDir))
        {
            foreach (var existing in Directory.GetFiles(outputDir, $"{prefix}_{cat}*.csv"))
            {
                string s = Path.GetFileNameWithoutExtension(existing);
                int at = s.LastIndexOf($"_{cat}", StringComparison.Ordinal);
                if (at < 0) continue;
                if (int.TryParse(s.Substring(at + 1 + cat.Length), NumberStyles.Integer,
                                 CultureInfo.InvariantCulture, out int n) && n >= nextIdx)
                    nextIdx = n + 1;
            }
        }
        _saveCounter = nextIdx;
        string outName = $"{prefix}_{cat}{nextIdx:D2}.csv";
        string outPath = Path.Combine(outputDir, outName);

        try
        {
            Directory.CreateDirectory(outputDir);
            using (var sw = new StreamWriter(outPath, false, System.Text.Encoding.UTF8))
            {
                // preserve '#' meta (record_origin_pos / record_origin_rot_xyzw etc.)
                // so downstream readers (PXR replay, GMR pico_csv_to_robot) can
                // normalise the coords to corridor-local frame and align with the
                // env without any manual rotation.
                // BUT drop any prior 'clip_category=' line so re-trimming an
                // already-trimmed clip doesn't pile up duplicate category tags.
                if (_metaLines != null)
                {
                    foreach (var m in _metaLines)
                    {
                        string body = m.TrimStart('#', ' ');
                        if (body.StartsWith("clip_category")) continue;
                        sw.WriteLine(m);
                    }
                }
                // tag the saved clip with its category so downstream pipelines
                // can filter without parsing the filename.
                sw.WriteLine($"# clip_category={cat}");
                // write header
                sw.WriteLine(string.Join(",", _headerLine));
                // write selected rows
                for (int i = lo; i <= hi; i++)
                    sw.WriteLine(_rawLines[i]);
            }

            int segLen = hi - lo + 1;
            float segDur = _times[hi] - _times[lo];
            ShowStatus($"SAVED [{cat}]: {outName}  ({segLen} frames, {segDur:F2}s)");
            Debug.Log($"[CsvTrimmer] Saved {outPath}  cat={cat}  rows {lo}-{hi}  ({segLen} frames, {segDur:F2}s)");
        }
        catch (Exception ex)
        {
            ShowStatus($"Save failed: {ex.Message}");
            Debug.LogError($"[CsvTrimmer] Save failed: {ex}");
        }
    }

    void ShowStatus(string msg)
    {
        _statusMsg = msg;
        _statusTime = Time.time;
        Debug.Log($"[CsvTrimmer] {msg}");
    }

    // ==================================================================
    // Origin mapping (same as PXR_24PointCsvPlayer)
    // ==================================================================
    void ParseMeta(string rawLine)
    {
        string line = rawLine.Substring(1).Trim();
        int eq = line.IndexOf('=');
        if (eq < 0) return;
        string key = line.Substring(0, eq).Trim();
        string val = line.Substring(eq + 1).Trim();

        if (key == "record_origin_pos")
        {
            var v = val.Split(',');
            if (v.Length == 3)
            {
                _recordOriginPos = new Vector3(ParseF(v[0]), ParseF(v[1]), ParseF(v[2]));
                _hasRecordOrigin = true;
            }
        }
        else if (key == "record_origin_rot_xyzw")
        {
            var q = val.Split(',');
            if (q.Length == 4)
            {
                _recordOriginRot = new Quaternion(ParseF(q[0]), ParseF(q[1]), ParseF(q[2]), ParseF(q[3]));
                _hasRecordOrigin = true;
            }
        }
    }

    void EnsureMappingMatrix()
    {
        if (_MReady) return;
        if (!applyOriginMapping || !_hasRecordOrigin || replayOrigin == null)
        {
            _M = Matrix4x4.identity;
            _MReady = true;
            return;
        }
        Matrix4x4 Trec = Matrix4x4.TRS(_recordOriginPos, _recordOriginRot, Vector3.one);
        _M = replayOrigin.localToWorldMatrix * Trec.inverse;
        _MReady = true;
        Debug.Log("[CsvTrimmer] Origin mapping ready.");
    }

    // ==================================================================
    // Gizmos: skeleton + collision feedback
    // ==================================================================
    void OnDrawGizmos()
    {
        if (!drawGizmos || _totalFrames == 0) return;

        // joints
        Gizmos.color = jointColor;
        for (int i = 0; i < 24; i++)
            Gizmos.DrawSphere(_displayJoints[i], jointSize);

        // bones
        Gizmos.color = boneColor;
        foreach (var (a, b) in Bones)
            DrawBoneGizmo(_displayJoints[a], _displayJoints[b]);

        // collision proximity line
        if (_minCollisionDist < 0.10f)
        {
            Gizmos.color = _minCollisionDist < 0 ? Color.red : Color.yellow;
            Gizmos.DrawLine(_closestBodyPoint, _closestPointOnEnv);
            Gizmos.DrawWireSphere(_closestPointOnEnv, 0.03f);
        }

        // trim range highlight (green/red markers at pelvis height)
        if (_trimStart >= 0 && _trimStart < _totalFrames)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(_frames[_trimStart][0], 0.06f);
        }
        if (_trimEnd >= 0 && _trimEnd < _totalFrames)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(_frames[_trimEnd][0], 0.06f);
        }
    }

    void DrawBoneGizmo(Vector3 pa, Vector3 pb)
    {
        if (!thickBones || boneThickness <= 0 || boneRays <= 1)
        {
            Gizmos.DrawLine(pa, pb);
            return;
        }

        Vector3 dir = pb - pa;
        float len = dir.magnitude;
        if (len < 1e-6f) return;
        dir /= len;

        Vector3 refUp = (Mathf.Abs(Vector3.Dot(dir, Vector3.up)) > 0.95f) ? Vector3.right : Vector3.up;
        Vector3 n1 = Vector3.Cross(dir, refUp).normalized;
        Vector3 n2 = Vector3.Cross(dir, n1).normalized;

        Gizmos.DrawLine(pa, pb);
        for (int k = 0; k < boneRays; k++)
        {
            float t = (float)k / boneRays * Mathf.PI * 2f;
            Vector3 off = (Mathf.Cos(t) * n1 + Mathf.Sin(t) * n2) * boneThickness;
            Gizmos.DrawLine(pa + off, pb + off);
        }
    }

    // ==================================================================
    // Parsing helpers
    // ==================================================================
    static float ParseF(string s)
    {
        if (float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v)) return v;
        return 0f;
    }

    static int ParseInt(string s)
    {
        if (int.TryParse(s, out int v)) return v;
        return 0;
    }
}
