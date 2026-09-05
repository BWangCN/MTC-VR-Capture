// MTC_AtomicSession.cs
// -----------------------------------------------------------------------------
// ADDITIVE on-headset capture driver for the ATOMIC corridor set. Does NOT touch
// the existing capture / coordinate pipeline -- it only calls the public control
// API of PXR_24PointMapper (SetRecordingPrefix / BeginRecording / EndRecording)
// and toggles which env child is active. It never moves or rotates worldRoot.
//
// WHY: a full 8 m corridor take couples long-horizon planning with pelvis drift,
// and one stumble loses all the behaviours in that take. The atomic set is one
// obstacle per env, so a take IS a primitive and IS a finished clip -- no
// trimming step. That only pays off if you can switch env and start/stop
// recording WITHOUT rebuilding the APK, which is what this script is for.
//
// CONTROLS (PICO controllers)
//   right A          short press           start recording
//   right A          hold 0.35 s           stop recording + save
//   right B          -                     next environment
//   left  Y          -                     previous environment
//   both grips       hold 1.0 s            mark the LAST take as reject
//
// COORDINATE CONTRACT (do not "simplify" this)
//   The FBX local origin is the corridor START and travel is +Z (baked in
//   export_to_unity.env_to_blender_scene). Every env child sits at IDENTITY
//   under worldRoot, and mapper.roomSpaceRef == worldRoot, so recorded joint
//   positions are corridor-local at G1 scale. The corridor therefore stays
//   exactly where AutoFlow's calibration put it for the whole session -- this
//   script never touches worldRoot's transform, and env children stay at
//   identity. Switching env is a SetActive and nothing more.
//
//   CAVEAT: PXR_24PointMapper converts POSITIONS to room space but writes joint
//   QUATERNIONS in world/tracking frame, so the two differ by worldRoot's yaw.
//   With the corridor fixed that yaw is one value per session rather than per
//   take, but it is still non-zero: AutoFlow aligns worldRoot to the HMD once at
//   calibration. It is written to every manifest row as `room_yaw_deg`, stamped
//   into the stowed CSV header by `capture_pipeline.py stow-batch`. Do not turn
//   off the manifest.
//
// OUTPUT (Application.persistentDataPath, next to the motion CSVs)
//   mtc_session_manifest.jsonl -- one append-only JSON row per take, plus a
//   {"op":"reject"} row when a take is rejected. Nothing is ever deleted
//   on-headset; the ingest step routes rejects into rejected/.
// -----------------------------------------------------------------------------
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.XR;

public class MTC_AtomicSession : MonoBehaviour
{
    [Header("References (auto-found if left empty)")]
    public PXR_24PointMapper mapper;
    public PXR_SessionAutoFlow autoFlow;
    public Body24Collision collisionLogger;
    public GoalZone goalZone;
    public Transform hmd;
    public Transform worldRoot;

    [Header("Environments")]
    [Tooltip("Children of worldRoot whose name starts with this are the capture set.")]
    public string envNamePrefix = "corridor_atomic_";
    [Tooltip("Explicit order. Leave empty to use every matching child, name-sorted. " +
             "The Editor tool (MTC > Atomic Envs) fills this in.")]
    public List<string> envOrder = new List<string>();
    [Tooltip("Add MeshColliders to EVERY env at startup instead of on first activation. " +
             "Pays the mesh-cook cost once at load so switching env is a pure SetActive.")]
    public bool prewarmColliders = true;

    [Header("Recording")]
    [Tooltip("Seconds the record button must be HELD to stop. Short taps are ignored " +
             "while recording, so gripping the controller mid-crawl cannot end a take.")]
    public float stopHoldSeconds = 0.35f;
    [Tooltip("Stop recording automatically when the goal ring is reached.")]
    public bool autoStopOnGoal = true;
    [Tooltip("Also advance to the next env when the goal ring is reached.")]
    public bool autoAdvanceOnGoal = false;

    [Header("Feedback")]
    public bool hud = true;
    public bool beeps = true;
    public bool haptics = true;

    [Header("Status (read-only)")]
    public string currentEnv = "";
    public int takesThisSession = 0;

    const string ManifestName = "mtc_session_manifest.jsonl";

    readonly List<Transform> _envs = new List<Transform>();
    int _idx = -1;
    bool _ready;

    // recording state
    bool _recording;
    float _recStart;
    string _recStartedAt = "";
    HashSet<string> _filesBefore;
    string _lastMotionCsv = "";
    string _lastEnv = "";
    float _stopHeldSince = -1f;
    float _rejectHeldSince = -1f;

    // input edge detection
    bool _pA, _pB, _pY;
    InputDevice _left, _right;
    float _deviceRefreshAt;

    AudioSource _audio;
    AudioClip _beepStart, _beepStop, _beepSwitch, _beepDeny;
    TextMesh _hudText;
    string _msg = "";
    float _msgUntil;

    // ---------------------------------------------------------------- boot ---

    void Start()
    {
        // TRIM MODE: a _MTC_Trimmer in the scene means this Play session is a CSV
        // replay, not a capture. Boot() would deactivate every env and force
        // envOrder[0] active, so the take would replay against the wrong corridor
        // no matter which one the trim window selected. Stand down.
        var trimmer = GameObject.Find("_MTC_Trimmer");
        if (trimmer != null && trimmer.activeInHierarchy)
        {
            Debug.Log("[AtomicSession] _MTC_Trimmer present -- trim mode, staying out of the way.");
            enabled = false;
            return;
        }

        if (mapper == null) mapper = FindObjectOfType<PXR_24PointMapper>();
        if (autoFlow == null) autoFlow = FindObjectOfType<PXR_SessionAutoFlow>();
        if (collisionLogger == null) collisionLogger = FindObjectOfType<Body24Collision>();
        if (goalZone == null) goalZone = FindObjectOfType<GoalZone>();
        if (hmd == null) hmd = autoFlow != null ? autoFlow.hmd : null;
        if (hmd == null && Camera.main != null) hmd = Camera.main.transform;
        if (worldRoot == null) worldRoot = autoFlow != null ? autoFlow.worldRoot : null;
        if (worldRoot == null)
        {
            var wo = GameObject.Find("WorldOrigin");
            if (wo != null) worldRoot = wo.transform;
        }

        if (beeps) BuildAudio();
        if (hud) BuildHud();
        StartCoroutine(Boot());
    }

    // AutoFlow calibrates, scales worldRoot, sets roomSpaceRef and only THEN sets
    // the recording prefix. Starting before it finishes would have our prefix
    // overwritten by its scene-name guess, so wait for the tail of its flow.
    IEnumerator Boot()
    {
        if (mapper == null || worldRoot == null)
        {
            Say("MTC_AtomicSession: no mapper / worldRoot -- disabled", 60f);
            Debug.LogError("[AtomicSession] mapper or worldRoot missing; disabling.");
            enabled = false;
            yield break;
        }

        float t0 = Time.time;
        while (Time.time - t0 < 60f)
        {
            if (worldRoot.gameObject.activeInHierarchy && mapper.roomSpaceRef != null) break;
            yield return null;
        }
        yield return new WaitForSeconds(0.25f);

        if (mapper.recordCsv) mapper.EndRecording();   // AutoFlow.autoRecord, if it was on

        CollectEnvs();
        if (_envs.Count == 0)
        {
            Say("no '" + envNamePrefix + "*' env under " + worldRoot.name, 60f);
            Debug.LogError("[AtomicSession] no atomic envs found under " + worldRoot.name);
            enabled = false;
            yield break;
        }

        if (prewarmColliders) yield return StartCoroutine(PrewarmColliders());

        if (goalZone != null) goalZone.onReached += OnGoalReached;

        _ready = true;
        GoTo(0);
        Debug.Log($"[AtomicSession] ready with {_envs.Count} envs. Manifest: " +
                  Path.Combine(Application.persistentDataPath, ManifestName));
    }

    void OnDestroy()
    {
        if (goalZone != null) goalZone.onReached -= OnGoalReached;
    }

    void CollectEnvs()
    {
        _envs.Clear();
        var byName = new Dictionary<string, Transform>();
        var found = new List<string>();
        foreach (Transform c in worldRoot)
        {
            if (!c.name.StartsWith(envNamePrefix, StringComparison.OrdinalIgnoreCase)) continue;
            byName[c.name] = c;
            found.Add(c.name);
        }
        found.Sort(StringComparer.Ordinal);

        if (envOrder != null && envOrder.Count > 0)
        {
            foreach (var n in envOrder)
                if (byName.TryGetValue(n, out var t)) _envs.Add(t);
                else Debug.LogWarning("[AtomicSession] envOrder lists missing env: " + n);
        }
        else
        {
            foreach (var n in found) _envs.Add(byName[n]);
        }

        // The whole coordinate contract assumes the env child is at identity under
        // worldRoot (see header). A stray offset from hand-placing an FBX would
        // put the recorded frame somewhere other than the corridor start.
        foreach (var e in _envs)
        {
            if (e.localPosition.sqrMagnitude > 1e-6f ||
                Quaternion.Angle(e.localRotation, Quaternion.identity) > 0.01f)
            {
                Debug.LogWarning($"[AtomicSession] {e.name} was not at identity under " +
                                 $"{worldRoot.name} (pos={e.localPosition}, " +
                                 $"rot={e.localRotation.eulerAngles}); resetting.");
                e.localPosition = Vector3.zero;
                e.localRotation = Quaternion.identity;
            }
            e.gameObject.SetActive(false);
        }
    }

    // Body24Collision adds MeshColliders lazily to whatever env is active, which
    // costs a mesh cook on the first frame after every switch -- a visible hitch
    // on the ground envs (14 Objaverse meshes). Cook them all up front instead.
    IEnumerator PrewarmColliders()
    {
        Say("preparing environments...", 30f);
        int cooked = 0;
        foreach (var e in _envs)
        {
            // Envs stay INACTIVE: assigning sharedMesh is what cooks the physics
            // shape, and activating each in turn would flash 16 corridors through
            // the pilot's view. GetComponentsInChildren(true) sees them anyway.
            foreach (var mf in e.GetComponentsInChildren<MeshFilter>(true))
            {
                if (mf == null || mf.sharedMesh == null) continue;
                // Same rule as Body24Collision.ignoreNamePrefixes. Must NOT filter
                // on "ground": the clutter meshes are ground_<id>_<n>_<asset> and
                // they are precisely the heaviest thing we came here to pre-cook.
                string n = mf.gameObject.name.ToLowerInvariant();
                if (n.StartsWith("floor") || n.StartsWith("goal_ring")) continue;
                if (mf.GetComponent<MeshCollider>() != null) continue;
                var mc = mf.gameObject.AddComponent<MeshCollider>();
                mc.sharedMesh = mf.sharedMesh;   // non-convex is fine for OverlapSphere
                cooked++;
            }
            yield return null;   // one env per frame, so the cook never stalls a single frame
        }
        Debug.Log($"[AtomicSession] pre-cooked {cooked} MeshColliders over {_envs.Count} envs.");
    }

    // --------------------------------------------------------------- input ---

    void Update()
    {
        if (!_ready) { UpdateHud(); return; }   // still shows "preparing environments..."
        RefreshDevices();

        bool a = Btn(_right, CommonUsages.primaryButton);     // A
        bool b = Btn(_right, CommonUsages.secondaryButton);   // B
        bool y = Btn(_left, CommonUsages.secondaryButton);    // Y
        bool grips = Btn(_left, CommonUsages.gripButton) && Btn(_right, CommonUsages.gripButton);

        // record: tap to start, HOLD to stop
        if (!_recording)
        {
            if (a && !_pA) StartRecording();
            _stopHeldSince = -1f;
        }
        else
        {
            if (a)
            {
                if (_stopHeldSince < 0f) _stopHeldSince = Time.time;
                else if (Time.time - _stopHeldSince >= stopHoldSeconds) StopRecording("ok");
            }
            else
            {
                if (_stopHeldSince >= 0f) Say("hold A to stop", 1.2f);
                _stopHeldSince = -1f;
            }
        }

        if (b && !_pB) Step(+1);
        if (y && !_pY) Step(-1);

        if (grips)
        {
            if (_rejectHeldSince < 0f) _rejectHeldSince = Time.time;
            else if (Time.time - _rejectHeldSince >= 1.0f) { RejectLastTake(); _rejectHeldSince = -1f; }
        }
        else _rejectHeldSince = -1f;

        _pA = a; _pB = b; _pY = y;
        UpdateHud();
    }

    void RefreshDevices()
    {
        if (Time.time < _deviceRefreshAt && _left.isValid && _right.isValid) return;
        _deviceRefreshAt = Time.time + 1f;
        var l = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Left, l);
        _left = l.Count > 0 ? l[0] : default;
        var r = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, r);
        _right = r.Count > 0 ? r[0] : default;
    }

    static bool Btn(InputDevice d, InputFeatureUsage<bool> usage)
    {
        return d.isValid && d.TryGetFeatureValue(usage, out bool v) && v;
    }

    // ----------------------------------------------------------- env switch ---

    void Step(int delta)
    {
        int n = _envs.Count;
        GoTo(((_idx + delta) % n + n) % n);
    }

    // Switching swaps which env is ACTIVE and nothing else. worldRoot is never
    // moved or rotated: the corridor stays exactly where AutoFlow's calibration
    // put it, for the whole session. An earlier version re-spawned it at the
    // pilot's feet and facing on every switch, which saved the walk back to the
    // start but silently re-oriented the room -- removed at the user's
    // instruction, deliberately without leaving a toggle.
    void GoTo(int index)
    {
        if (index < 0 || index >= _envs.Count) return;
        bool stoppedATake = _recording;
        if (_recording) StopRecording("ok");   // never leave a take open across a switch

        for (int i = 0; i < _envs.Count; i++) _envs[i].gameObject.SetActive(i == index);
        _idx = index;
        currentEnv = _envs[index].name;

        // Both file names get the env id, so the motion CSV and the collision log
        // are already paired by name before the manifest is even consulted.
        mapper.SetRecordingPrefix(currentEnv);
        // ...but NOT the collision prefix, not yet. Body24Collision dumps its log
        // on the LateUpdate AFTER recordCsv goes false, so switching env mid-take
        // (StopRecording above, then this) would stamp the NEXT env's name onto
        // the take we just finished -- observed live on 2026-08-24, where
        // doghole_00's log came out named doghole_01 and FinishTake then failed to
        // pair it at all. Hand it to a coroutine that waits out the dump.
        StartCoroutine(ApplyLogPrefix(currentEnv, stoppedATake));

        Beep(_beepSwitch);
        Pulse(0.35f, 0.06f);
        Say($"{index + 1}/{_envs.Count}  {currentEnv}", 2.5f);
        Debug.Log($"[AtomicSession] env -> {currentEnv} ({index + 1}/{_envs.Count})");
    }

    // Body24Collision names its dump from logPrefix at dump time, one LateUpdate
    // after recordCsv goes false. FinishTake waits 5 frames for that file; hold the
    // new prefix a frame longer so the just-finished take keeps its own name.
    IEnumerator ApplyLogPrefix(string envId, bool waitForDump)
    {
        if (waitForDump)
            for (int i = 0; i < 6; i++) yield return null;
        if (collisionLogger != null && currentEnv == envId)
            collisionLogger.logPrefix = "collisions_" + envId;
    }

    void OnGoalReached()
    {
        if (!_ready) return;
        if (_recording && autoStopOnGoal) StopRecording("goal");
        if (autoAdvanceOnGoal) StartCoroutine(AdvanceAfterGoal());
    }

    IEnumerator AdvanceAfterGoal()
    {
        yield return new WaitForSeconds(1.0f);   // let the chime land and you turn round
        Step(+1);
    }

    // ------------------------------------------------------------ recording ---

    void StartRecording()
    {
        if (_recording) return;
        _filesBefore = SnapshotCsvs();
        mapper.BeginRecording();
        _recording = true;
        _recStart = Time.time;
        _recStartedAt = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff");
        _lastMotionCsv = NewCsvSince(_filesBefore, currentEnv);
        Beep(_beepStart);
        Pulse(0.6f, 0.08f);
        Say("REC  " + currentEnv, 2f);
    }

    void StopRecording(string reason)
    {
        if (!_recording) return;
        float dur = Time.time - _recStart;
        mapper.EndRecording();
        _recording = false;
        _stopHeldSince = -1f;
        takesThisSession++;
        _lastEnv = currentEnv;

        // Snapshot the take HERE, not in the coroutine. GoTo() stops an open take
        // before switching, so anything read after the first yield would describe
        // the env we are moving TO rather than the one just recorded.
        var rec = new TakeRecord
        {
            op = "take",
            env_id = currentEnv,
            take_index = takesThisSession,
            motion_csv = _lastMotionCsv,
            status = reason,
            started_at = _recStartedAt,
            ended_at = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"),
            duration_s = dur,
            room_yaw_deg = worldRoot.rotation.eulerAngles.y,
            world_scale = worldRoot.lossyScale.x,
            worldroot_pos = V3(worldRoot.position),
            worldroot_rot_xyzw = Q(worldRoot.rotation),
        };

        Beep(_beepStop);
        Pulse(0.6f, 0.12f);
        Say($"saved {dur:F1}s ({reason})", 2.5f);
        // Body24Collision dumps its log on the frame AFTER recordCsv goes false,
        // so the collision file does not exist yet.
        StartCoroutine(FinishTake(rec, _filesBefore));
    }

    IEnumerator FinishTake(TakeRecord rec, HashSet<string> before)
    {
        for (int i = 0; i < 5; i++) yield return null;
        if (string.IsNullOrEmpty(rec.motion_csv))
        {
            rec.motion_csv = NewCsvSince(before, rec.env_id);
            if (_lastEnv == rec.env_id) _lastMotionCsv = rec.motion_csv;
        }
        rec.collision_csv = NewCsvSince(before, "collisions_" + rec.env_id);
        AppendManifest(JsonUtility.ToJson(rec));
        Debug.Log($"[AtomicSession] take {rec.take_index} {rec.env_id}: {rec.motion_csv} " +
                  $"({rec.duration_s:F1}s, yaw={rec.room_yaw_deg:F1}deg, col={rec.collision_csv})");
    }

    void RejectLastTake()
    {
        if (_recording) { Say("stop the take first", 2f); Beep(_beepDeny); return; }
        if (string.IsNullOrEmpty(_lastMotionCsv)) { Say("nothing to reject", 2f); Beep(_beepDeny); return; }
        // Append-only: nothing on the headset is deleted or renamed. stow-batch
        // reads the last op for a file and routes rejects into rejected/.
        AppendManifest(JsonUtility.ToJson(new TakeRecord
        {
            op = "reject",
            env_id = _lastEnv,
            motion_csv = _lastMotionCsv,
            status = "reject",
            ended_at = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"),
        }));
        Say("REJECTED " + Path.GetFileName(_lastMotionCsv), 2.5f);
        Beep(_beepStop);
        Pulse(0.8f, 0.25f);
        _lastMotionCsv = "";
    }

    // ---------------------------------------------------------------- files ---

    HashSet<string> SnapshotCsvs()
    {
        var s = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        try
        {
            foreach (var f in Directory.GetFiles(Application.persistentDataPath, "*.csv"))
                s.Add(Path.GetFileName(f));
        }
        catch (Exception e) { Debug.LogWarning("[AtomicSession] snapshot failed: " + e.Message); }
        return s;
    }

    string NewCsvSince(HashSet<string> before, string startsWith)
    {
        try
        {
            string best = "";
            DateTime bestT = DateTime.MinValue;
            foreach (var f in Directory.GetFiles(Application.persistentDataPath, "*.csv"))
            {
                string n = Path.GetFileName(f);
                if (before != null && before.Contains(n)) continue;
                if (!n.StartsWith(startsWith, StringComparison.OrdinalIgnoreCase)) continue;
                var t = File.GetLastWriteTime(f);
                if (t >= bestT) { bestT = t; best = n; }
            }
            return best;
        }
        catch (Exception e)
        {
            Debug.LogWarning("[AtomicSession] file scan failed: " + e.Message);
            return "";
        }
    }

    void AppendManifest(string line)
    {
        try
        {
            File.AppendAllText(Path.Combine(Application.persistentDataPath, ManifestName),
                               line + "\n", System.Text.Encoding.UTF8);
        }
        catch (Exception e) { Debug.LogError("[AtomicSession] manifest write failed: " + e); }
    }

    [Serializable]
    class TakeRecord
    {
        public string op;
        public string env_id;
        public int take_index;
        public string motion_csv;
        public string collision_csv;
        public string status;
        public string started_at;
        public string ended_at;
        public float duration_s;
        public float room_yaw_deg;
        public float world_scale;
        public string worldroot_pos;
        public string worldroot_rot_xyzw;
    }

    static string V3(Vector3 v) => $"{v.x:F6},{v.y:F6},{v.z:F6}";
    static string Q(Quaternion q) => $"{q.x:F6},{q.y:F6},{q.z:F6},{q.w:F6}";

    // ------------------------------------------------------------- feedback ---

    void BuildAudio()
    {
        _audio = gameObject.AddComponent<AudioSource>();
        _audio.playOnAwake = false;
        _audio.spatialBlend = 0f;
        _beepStart = Tone("start", 880f, 0.10f);
        _beepStop = Tone("stop", 440f, 0.16f);
        _beepSwitch = Tone("switch", 660f, 0.07f);
        _beepDeny = Tone("deny", 220f, 0.20f);
    }

    static AudioClip Tone(string name, float freq, float dur, float vol = 0.45f)
    {
        const int sr = 44100;
        int n = Mathf.Max(16, Mathf.RoundToInt(sr * dur));
        var clip = AudioClip.Create(name, n, 1, sr, false);
        var data = new float[n];
        int fade = Mathf.Max(1, Mathf.RoundToInt(sr * 0.008f));
        for (int i = 0; i < n; i++)
        {
            float env = Mathf.Min(1f, Mathf.Min(i, n - 1 - i) / (float)fade);
            data[i] = Mathf.Sin(2f * Mathf.PI * freq * i / sr) * vol * env;
        }
        clip.SetData(data, 0);
        return clip;
    }

    void Beep(AudioClip c)
    {
        if (!beeps || _audio == null || c == null) return;
        _audio.PlayOneShot(c);
    }

    void Pulse(float amplitude, float seconds)
    {
        if (!haptics) return;
        SendHaptic(_left, amplitude, seconds);
        SendHaptic(_right, amplitude, seconds);
    }

    static void SendHaptic(InputDevice d, float amplitude, float seconds)
    {
        if (!d.isValid) return;
        if (d.TryGetHapticCapabilities(out var caps) && caps.supportsImpulse)
            d.SendHapticImpulse(0u, Mathf.Clamp01(amplitude), seconds);
    }

    void BuildHud()
    {
        if (hmd == null) return;
        try
        {
            var go = new GameObject("MTC_HUD");
            go.transform.SetParent(hmd, false);
            go.transform.localPosition = new Vector3(0f, -0.20f, 0.60f);
            go.transform.localRotation = Quaternion.identity;
            _hudText = go.AddComponent<TextMesh>();
            _hudText.anchor = TextAnchor.MiddleCenter;
            _hudText.alignment = TextAlignment.Center;
            _hudText.characterSize = 0.010f;
            _hudText.fontSize = 96;
            _hudText.color = new Color(0.85f, 1f, 0.85f);
            var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
            if (font == null) font = Resources.GetBuiltinResource<Font>("Arial.ttf");
            if (font != null)
            {
                _hudText.font = font;
                go.GetComponent<MeshRenderer>().sharedMaterial = font.material;
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning("[AtomicSession] HUD unavailable: " + e.Message);
            _hudText = null;
        }
    }

    void Say(string msg, float seconds)
    {
        _msg = msg;
        _msgUntil = Time.time + seconds;
    }

    void UpdateHud()
    {
        if (_hudText == null) return;
        string line1 = _recording
            ? $"● REC  {Time.time - _recStart:F1}s"
            : $"{(_idx + 1)}/{_envs.Count}  takes:{takesThisSession}";
        string line2 = currentEnv;
        string line3 = Time.time < _msgUntil ? _msg : "A rec | B next | Y prev";
        _hudText.text = line1 + "\n" + line2 + "\n" + line3;
        _hudText.color = _recording ? new Color(1f, 0.45f, 0.4f) : new Color(0.8f, 1f, 0.85f);
    }

#if UNITY_EDITOR
    [UnityEditor.MenuItem("MTC/Atomic Envs/Add Session Controller to Scene")]
    static void AddToScene()
    {
        if (FindObjectOfType<MTC_AtomicSession>() != null)
        {
            Debug.Log("[AtomicSession] already in scene.");
            return;
        }
        var go = new GameObject("MTC_AtomicSession");
        go.AddComponent<MTC_AtomicSession>();
        UnityEditor.Undo.RegisterCreatedObjectUndo(go, "Add MTC_AtomicSession");
        UnityEditor.Selection.activeGameObject = go;
    }
#endif
}
