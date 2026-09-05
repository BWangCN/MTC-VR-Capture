// TrimCaptureMode.cs  (Editor window)
// -----------------------------------------------------------------------------
// One-click switch between the two ways this scene gets used:
//
//   CAPTURE mode  -- the live VR rig (PXR_BodyTrackingFeed + PXR_24PointMapper)
//                    drives + records the avatar. This is the state you build the
//                    on-device APK from.
//   TRIM mode     -- a CsvReplayTrimmer replays a recorded raw CSV so you can cut
//                    it into clips (press Play, then A/D = start/end, 1/2/3/4 =
//                    category, S = save).
//
//   MTC > Trim-Capture Mode   opens this window.
//     * Pick an env + take, "Enter Trim Mode": spawns a "_MTC_Trimmer" object,
//       fills csvFilePath / outputNamePrefix(=env_id) / outputDir(=<env>\trimmed),
//       auto-wires replayOrigin to the feed's cubesRoot, and (optionally) hides
//       the live capture rig so it doesn't clutter the replay.
//     * "Return to Capture Mode": deletes the trimmer and re-enables the rig.
//
// Purely additive -- it never edits CsvReplayTrimmer or any capture/coord script;
// it only toggles GameObject active-state and configures a trimmer object it owns.
// -----------------------------------------------------------------------------
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public class TrimCaptureMode : EditorWindow
{
    const string TrimObjName = "_MTC_Trimmer";
    const string EnvPrefKey = "MTC_TrimMode_LastEnv";
    const string RigPrefKey = "MTC_TrimMode_RigWasActive";
    const string CapPrefKey = "MTC_TrimMode_LastCaptureEnv";
    const string FbxDir = "Assets/FBX_Scenes";
    const string WorldOriginName = "WorldOrigin";

    static string CaptureRoot =>
        System.Environment.GetEnvironmentVariable("MTC_CAPTURE_ROOT") ?? @"E:\MTC_data\capture";

    readonly Dictionary<string, List<string>> _takes = new Dictionary<string, List<string>>();
    string[] _envIds = new string[0];
    int _envIdx, _takeIdx;
    bool _hideCaptureRig = true;

    // Worklist: every stowed take, its length, and how many clips its env already
    // has. Duration is read from the LAST line of the CSV (an 8 KB tail seek), so
    // listing 40 MB takes costs nothing.
    readonly Dictionary<string, float> _takeDur = new Dictionary<string, float>();
    readonly Dictionary<string, int> _clipCount = new Dictionary<string, int>();
    Vector2 _wlScroll;
    Vector2 _mainScroll;
    bool _wlPendingOnly = true;
    string _pendingTrimEnv;
    int _pendingTrimTake = -1;

    readonly List<string> _meshEnvs = new List<string>();
    int _capIdx;
    // 250+ generated envs also have a mesh; without a filter the handful of
    // curated ones sort to the bottom of an unusable dropdown.
    string _capFilter = "std";

    [MenuItem("MTC/Trim-Capture Mode")]
    static void Open()
    {
        var w = GetWindow<TrimCaptureMode>("MTC Mode");
        w.minSize = new Vector2(440, 520);
        w.Rescan();
        w.Show();
    }

    void OnEnable() => Rescan();

    // Scan E:\MTC_data\capture\*\raw\*.csv (skipping paired collision logs).
    void Rescan()
    {
        _takes.Clear();
        try
        {
            foreach (var kv in MtcCaptureRoot.EnvDirs())
                {
                    string envId = kv.Key;
                    string raw = Path.Combine(kv.Value, "raw");
                    if (!Directory.Exists(raw)) continue;
                    var takes = Directory.GetFiles(raw, "*.csv")
                        .Where(p => !p.EndsWith(".collisions.csv"))
                        .OrderBy(p => p).ToList();
                    if (takes.Count > 0) _takes[envId] = takes;
                }
        }
        catch (System.Exception e) { Debug.LogWarning($"[MTC] capture scan failed: {e.Message}"); }

        _takeDur.Clear();
        _clipCount.Clear();
        foreach (var kv in _takes)
        {
            string trimmed = Path.Combine(MtcCaptureRoot.EnvDir(kv.Key), "trimmed");
            _clipCount[kv.Key] = Directory.Exists(trimmed)
                ? Directory.GetFiles(trimmed, "*.csv").Length : 0;
            foreach (var t in kv.Value) _takeDur[t] = TakeDurationSeconds(t);
        }

        _envIds = _takes.Keys.OrderBy(s => s).ToArray();
        string last = EditorPrefs.GetString(EnvPrefKey, "");
        int at = System.Array.IndexOf(_envIds, last);
        _envIdx = at >= 0 ? at : Mathf.Clamp(_envIdx, 0, Mathf.Max(0, _envIds.Length - 1));
        _takeIdx = 0;

        RescanMeshEnvs();
    }

    // Envs that can be CAPTURED in: those with a built mesh. Different filter from
    // the trim list above, which needs a recorded take instead.
    void RescanMeshEnvs()
    {
        _meshEnvs.Clear();
        try
        {
            foreach (var kv in MtcCaptureRoot.EnvDirs())
                    if (File.Exists(Path.Combine(kv.Value, "mesh", kv.Key + "_simplified.fbx")))
                        _meshEnvs.Add(kv.Key);
        }
        catch (System.Exception e) { Debug.LogWarning($"[MTC] mesh scan failed: {e.Message}"); }

        _meshEnvs.Sort();
        int i = _meshEnvs.IndexOf(EditorPrefs.GetString(CapPrefKey, ""));
        _capIdx = i >= 0 ? i : Mathf.Clamp(_capIdx, 0, Mathf.Max(0, _meshEnvs.Count - 1));
    }

    /// Clip length from the LAST data row's `time` column (always field 1), read
    /// via an 8 KB tail seek so a 40 MB take costs nothing to list.
    static float TakeDurationSeconds(string path)
    {
        try
        {
            using (var fs = new FileStream(path, FileMode.Open, FileAccess.Read))
            {
                int n = (int)System.Math.Min(8192, fs.Length);
                if (n <= 0) return 0f;
                fs.Seek(-n, SeekOrigin.End);
                var buf = new byte[n];
                int got = fs.Read(buf, 0, n);
                var lines = System.Text.Encoding.UTF8.GetString(buf, 0, got).Split('\n');
                for (int i = lines.Length - 1; i >= 0; i--)
                {
                    var f = lines[i].Trim().Split(',');
                    if (f.Length > 2 && float.TryParse(f[1], NumberStyles.Float,
                                                       CultureInfo.InvariantCulture, out float t))
                        return t;
                }
            }
        }
        catch (System.Exception) { }
        return 0f;
    }

    static bool InTrimMode()
    {
        var go = GameObject.Find(TrimObjName);
        return go != null && go.activeInHierarchy;
    }

    void OnGUI()
    {
        // Outer scroll: the worklist made the content taller than the window, and
        // without this the bottom controls ("Rescan takes", "Return to Capture
        // Mode") are simply unreachable.
        _mainScroll = EditorGUILayout.BeginScrollView(_mainScroll);

        EditorGUILayout.Space();
        bool trim = InTrimMode();
        EditorGUILayout.HelpBox(
            "Current mode: " + (trim
                ? "TRIM  (replaying a CSV to cut clips -- press Play)"
                : "CAPTURE  (live VR rig active)"),
            trim ? MessageType.Info : MessageType.None);

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Trim Mode", EditorStyles.boldLabel);

        if (_envIds.Length == 0)
        {
            EditorGUILayout.HelpBox("No envs with raw takes under:\n" + CaptureRoot +
                "\nCapture + stow first, then Rescan.", MessageType.Warning);
        }
        else
        {
            DrawWorklist();
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Or pick manually", EditorStyles.miniBoldLabel);
            _envIdx = Mathf.Clamp(_envIdx, 0, _envIds.Length - 1);
            _envIdx = EditorGUILayout.Popup("Environment", _envIdx, _envIds);
            var takes = _takes[_envIds[_envIdx]];
            var takeNames = takes.Select(Path.GetFileName).ToArray();
            _takeIdx = Mathf.Clamp(_takeIdx, 0, takeNames.Length - 1);
            _takeIdx = EditorGUILayout.Popup("Take", _takeIdx, takeNames);
            long bytes = new FileInfo(takes[_takeIdx]).Length;
            EditorGUILayout.LabelField(" ", $"{bytes / (1024f * 1024f):F1} MB  ->  {_envIds[_envIdx]}\\trimmed\\");
        }

        _hideCaptureRig = EditorGUILayout.ToggleLeft(
            "Hide capture rig (PXR feed/mapper) while trimming", _hideCaptureRig);

        using (new EditorGUI.DisabledScope(_envIds.Length == 0))
            if (GUILayout.Button("Enter Trim Mode  ▶", GUILayout.Height(30)))
                EnterTrim();

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Capture Mode", EditorStyles.boldLabel);

        if (_meshEnvs.Count == 0)
        {
            EditorGUILayout.HelpBox("No env with a built mesh under:\n" + CaptureRoot +
                "\nGenerate + export an env first, then Rescan.", MessageType.Warning);
        }
        else
        {
            _capFilter = EditorGUILayout.TextField("Filter", _capFilter);
            var shown = string.IsNullOrEmpty(_capFilter)
                ? _meshEnvs
                : _meshEnvs.Where(e => e.IndexOf(
                      _capFilter, System.StringComparison.OrdinalIgnoreCase) >= 0).ToList();

            if (shown.Count == 0)
            {
                EditorGUILayout.HelpBox(
                    $"No env matches \"{_capFilter}\" ({_meshEnvs.Count} available). "
                    + "Clear the filter to see all.", MessageType.Info);
            }
            else
            {
                _capIdx = Mathf.Clamp(_capIdx, 0, shown.Count - 1);
                _capIdx = EditorGUILayout.Popup("Environment", _capIdx, shown.ToArray());
                EditorGUILayout.LabelField(" ",
                    $"{shown.Count} shown of {_meshEnvs.Count}", EditorStyles.miniLabel);
                if (GUILayout.Button("Load Environment + Capture Mode", GUILayout.Height(30)))
                    LoadEnvForCapture(shown[_capIdx]);
                EditorGUILayout.LabelField(" ",
                    "Imports the FBX, parents it to " + WorldOriginName +
                    " at identity, hides any other corridor, and restores the rig.",
                    EditorStyles.wordWrappedMiniLabel);
            }
        }

        if (GUILayout.Button("Return to Capture Mode (leave env as-is)", GUILayout.Height(24)))
            EnterCapture();

        EditorGUILayout.Space();
        using (new EditorGUILayout.HorizontalScope())
        {
            if (GUILayout.Button("Rescan takes")) Rescan();
            EditorGUILayout.LabelField(CaptureRoot, EditorStyles.miniLabel);
        }

        EditorGUILayout.EndScrollView();

        // Deferred so the click does not tear down layout groups mid-pass, which
        // GUIUtility.ExitGUI() inside the nested worklist scroll view would do.
        if (_pendingTrimTake >= 0)
        {
            int at = System.Array.IndexOf(_envIds, _pendingTrimEnv);
            int take = _pendingTrimTake;
            _pendingTrimEnv = null;
            _pendingTrimTake = -1;
            if (at >= 0)
            {
                _envIdx = at;
                _takeIdx = take;
                EnterTrim();
            }
        }
    }

    // One row per stowed take, one button to start cutting it. Replaces
    // env-dropdown -> take-dropdown -> Enter with a single click, and shows at a
    // glance which envs still have nothing in trimmed/.
    void DrawWorklist()
    {
        int pending = 0, total = 0;
        foreach (var kv in _takes)
            foreach (var t in kv.Value)
            {
                total++;
                if (_clipCount.TryGetValue(kv.Key, out int c) && c == 0) pending++;
            }

        using (new EditorGUILayout.HorizontalScope())
        {
            EditorGUILayout.LabelField($"Takes: {total}   pending: {pending}",
                                       EditorStyles.miniBoldLabel);
            _wlPendingOnly = EditorGUILayout.ToggleLeft("pending only", _wlPendingOnly,
                                                        GUILayout.Width(100));
        }

        _wlScroll = EditorGUILayout.BeginScrollView(_wlScroll, GUILayout.Height(180));
        foreach (var envId in _envIds)
        {
            int clips = _clipCount.TryGetValue(envId, out int c) ? c : 0;
            if (_wlPendingOnly && clips > 0) continue;
            var takes = _takes[envId];
            for (int i = 0; i < takes.Count; i++)
            {
                string path = takes[i];
                float dur = _takeDur.TryGetValue(path, out float d) ? d : 0f;
                float mb = new FileInfo(path).Length / (1024f * 1024f);
                using (new EditorGUILayout.HorizontalScope(EditorStyles.helpBox))
                {
                    using (new EditorGUILayout.VerticalScope())
                    {
                        EditorGUILayout.LabelField(
                            envId + (takes.Count > 1 ? $"   [take {i + 1}/{takes.Count}]" : ""),
                            EditorStyles.boldLabel);
                        EditorGUILayout.LabelField(
                            $"{dur:F0} s   {mb:F1} MB   " +
                            (clips > 0 ? $"trimmed/: {clips} clip(s)" : "trimmed/: EMPTY"),
                            EditorStyles.miniLabel);
                    }
                    if (GUILayout.Button("Trim  ▶", GUILayout.Width(74), GUILayout.Height(32)))
                    {
                        _pendingTrimEnv = envId;
                        _pendingTrimTake = i;   // run it after the layout pass
                    }
                }
            }
        }
        EditorGUILayout.EndScrollView();
    }

    /// Make `envId` the one active corridor under WorldOrigin, using what is
    /// already instantiated in the scene. Returns false if that env is not in the
    /// scene (use the Capture half, which imports the FBX first).
    static bool ActivateSceneCorridor(string envId)
    {
        var worldOrigin = GameObject.Find(WorldOriginName);
        if (worldOrigin == null) return false;

        GameObject target = null;
        foreach (Transform child in worldOrigin.transform)
        {
            if (!child.name.StartsWith("corridor_")) continue;
            if (child.name == envId || child.name == envId + "_simplified")
                target = child.gameObject;
            else if (child.gameObject.activeSelf)
            {
                Undo.RecordObject(child.gameObject, "Hide other corridor");
                child.gameObject.SetActive(false);
            }
        }
        if (target == null)
        {
            // Not in the scene. That is the NORMAL state for a just-captured env:
            // MTC > Atomic Envs retires an env from the scene once it has a take,
            // which is right for capture and wrong for trimming the take that
            // retired it. Bring the mesh back in rather than replaying the clip
            // against whatever corridor happens to be loaded.
            string fbxName = envId + "_simplified.fbx";
            string assetPath = FbxDir + "/" + fbxName;
            string dst = Path.Combine(Application.dataPath, "FBX_Scenes", fbxName);
            string src = Path.Combine(MtcCaptureRoot.EnvDir(envId), "mesh", fbxName);
            bool stale = !File.Exists(dst) ||
                         (File.Exists(src) && new FileInfo(src).Length != new FileInfo(dst).Length);
            if (stale && File.Exists(src))
            {
                Directory.CreateDirectory(Path.GetDirectoryName(dst));
                File.Copy(src, dst, true);
                AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceSynchronousImport);
            }
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
            if (prefab == null) return false;
            target = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            target.name = envId;
            target.transform.SetParent(worldOrigin.transform, false);
            Undo.RegisterCreatedObjectUndo(target, "Load corridor for trim");
            Debug.Log($"[MTC] re-imported '{envId}' for trimming (it had been retired " +
                      "from the scene after capture).");
        }

        Undo.RecordObject(target, "Show corridor for trim");
        // IDENTITY: the -90deg long-axis rotation is baked into the FBX.
        target.transform.localPosition = Vector3.zero;
        target.transform.localRotation = Quaternion.identity;
        target.transform.localScale = Vector3.one;
        target.SetActive(true);
        return true;
    }

    void EnterTrim()
    {
        string envId = _envIds[_envIdx];
        string take = _takes[envId][_takeIdx];
        EditorPrefs.SetString(EnvPrefKey, envId);

        // Replaying a take against the WRONG corridor is silently misleading --
        // the skeleton ducks under a beam that isn't there. EnterTrim used to
        // leave whatever corridor happened to be active, so every take replayed
        // against env #1.
        if (!ActivateSceneCorridor(envId))
            Debug.LogWarning($"[MTC] '{envId}' is not instantiated under {WorldOriginName}; " +
                             "the replay will show whatever corridor is active. Load it once " +
                             "via the Capture half (or MTC > Atomic Envs > Import All).");

        if (_hideCaptureRig) SetCaptureRig(false);

        var go = GameObject.Find(TrimObjName);
        if (go == null)
        {
            go = new GameObject(TrimObjName);
            Undo.RegisterCreatedObjectUndo(go, "Enter Trim Mode");
        }
        var tr = go.GetComponent<CsvReplayTrimmer>() ?? go.AddComponent<CsvReplayTrimmer>();
        Undo.RecordObject(tr, "Configure Trimmer");
        tr.csvFilePath = take;
        tr.outputNamePrefix = envId;                                   // else prefix defaults to "raw"
        tr.outputDir = Path.Combine(MtcCaptureRoot.EnvDir(envId), "trimmed");
        tr.applyOriginMapping = true;

        var feed = FindSceneObject<PXR_BodyTrackingFeed>();
        if (feed != null)                                             // align replay to the capture origin
            tr.replayOrigin = feed.cubesRoot != null ? feed.cubesRoot : feed.transform;
        // environmentRoot left null on purpose -> trimmer auto-finds all MeshColliders.

        // Only _MTC_Trimmer should draw a skeleton -- silence any OTHER
        // CsvReplayTrimmer left in the scene (e.g. one added by hand earlier).
        // OnDrawGizmos runs even for disabled components, so flip drawGizmos.
        foreach (var other in Object.FindObjectsOfType<CsvReplayTrimmer>(true))
            if (other != tr && other.drawGizmos)
            {
                Undo.RecordObject(other, "Silence stray trimmer");
                other.drawGizmos = false;
            }

        go.SetActive(true);
        Selection.activeGameObject = go;
        EditorGUIUtility.PingObject(go);
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log($"[MTC] TRIM mode: {envId} / {Path.GetFileName(take)}  ->  {tr.outputDir}\n" +
                  "Press Play, then A/D = start/end, 1/2/3/4 = easy/avoid/near/bad, S = save.");
    }

    void EnterCapture()
    {
        var go = GameObject.Find(TrimObjName);
        if (go != null) Undo.DestroyObjectImmediate(go);
        SetCaptureRig(true);
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log("[MTC] CAPTURE mode: trimmer removed, capture rig re-enabled.");
    }

    // One click from "nothing in the scene" to "ready to record this env":
    // copy + import the FBX if needed, parent it under WorldOrigin, make it the
    // only visible corridor, and put the rig back into capture state.
    void LoadEnvForCapture(string envId)
    {
        var worldOrigin = GameObject.Find(WorldOriginName);
        if (worldOrigin == null)
        {
            EditorUtility.DisplayDialog("No " + WorldOriginName,
                "This scene has no '" + WorldOriginName + "' object to parent the corridor to.\n" +
                "Open the capture scene first.", "OK");
            return;
        }

        string fbxName = envId + "_simplified.fbx";
        string assetPath = FbxDir + "/" + fbxName;
        string dst = Path.Combine(Application.dataPath, "FBX_Scenes", fbxName);
        string src = Path.Combine(MtcCaptureRoot.EnvDir(envId), "mesh", fbxName);

        // Re-copy when the source is newer/different so a rebuilt env cannot be
        // captured against a stale mesh sitting in the project.
        bool stale = !File.Exists(dst) ||
                     (File.Exists(src) && new FileInfo(src).Length != new FileInfo(dst).Length);
        if (stale && File.Exists(src))
        {
            Directory.CreateDirectory(Path.GetDirectoryName(dst));
            File.Copy(src, dst, true);
            AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceSynchronousImport);
            Debug.Log($"[MTC] imported mesh for {envId} from {src}");
        }

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
        if (prefab == null)
        {
            EditorUtility.DisplayDialog("FBX not found",
                "Could not load " + assetPath + "\nSource looked for:\n" + src, "OK");
            return;
        }

        GameObject target = null;
        foreach (Transform child in worldOrigin.transform)
        {
            if (!child.name.StartsWith("corridor_")) continue;
            if (child.name == envId || child.name == envId + "_simplified")
            {
                target = child.gameObject;
            }
            else if (child.gameObject.activeSelf)
            {
                Undo.RecordObject(child.gameObject, "Hide other corridor");
                child.gameObject.SetActive(false);
            }
        }

        if (target == null)
        {
            target = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            target.name = envId;
            target.transform.SetParent(worldOrigin.transform, false);
            Undo.RegisterCreatedObjectUndo(target, "Load Corridor For Capture");
        }
        // IDENTITY on purpose: the -90deg long-axis rotation is baked into the
        // exported FBX by export_to_unity.env_to_blender_scene. Adding a rotation
        // here would put the corridor 90deg off from the recorded motion.
        target.transform.localPosition = Vector3.zero;
        target.transform.localRotation = Quaternion.identity;
        target.transform.localScale = Vector3.one;
        target.SetActive(true);

        EditorPrefs.SetString(CapPrefKey, envId);
        EnterCapture();

        Selection.activeGameObject = target;
        EditorGUIUtility.PingObject(target);
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log($"[MTC] CAPTURE ready: {envId} under {WorldOriginName}, identity, rig restored.\n" +
                  "Build + deploy to the headset to record.");
    }

    // Enable/disable every GameObject that renders a live/reference body skeleton,
    // so Trim mode leaves ONLY the trimmer's own skeleton on screen. A second,
    // frozen "stick man" is almost always one of these on a GameObject that is NOT
    // the feed/mapper (e.g. PXR_SkeletonGenerator's generated T-pose cubes, or a
    // PXR_24PointCsvPlayer paused at frame 0), so it survived the feed/mapper-only
    // hide.
    //
    // NOT symmetric: PXR_SkeletonGenerator and PXR_24PointCsvPlayer are normally
    // OFF during capture. Blindly re-enabling everything on the way out would
    // switch them on and bring the frozen second skeleton back, so record which
    // objects were active before hiding and restore exactly that set.
    static void SetCaptureRig(bool on)
    {
        var set = new HashSet<GameObject>();
        AddGOs<PXR_24PointMapper>(set);
        AddGOs<PXR_BodyTrackingFeed>(set);
        AddGOs<PXR_SkeletonGenerator>(set);
        AddGOs<PXR_24PointCsvPlayer>(set);
        // AutoFlow is capture-time only: on Play it calibrates against the HMD,
        // rescales worldRoot and opens a recording. During a trim replay none of
        // that applies, and it errors out with "hmd is null but pivotAtHMD=true"
        // because the trimmer's own editor camera has taken over. Park it too.
        AddGOs<PXR_SessionAutoFlow>(set);

        if (!on)
        {
            var wasOn = set.Where(g => g.activeSelf).Select(HierPath).ToArray();
            EditorPrefs.SetString(RigPrefKey, string.Join("\n", wasOn));
            foreach (var g in set.Where(g => g.activeSelf))
            {
                Undo.RecordObject(g, "Hide Capture Rig");
                g.SetActive(false);
            }
            Debug.Log($"[MTC] Hid {wasOn.Length} skeleton/rig object(s); state recorded.");
            return;
        }

        // Restore. With no record (first run, or the scene was left mid-trim by an
        // older build) fall back to the minimum a capture needs: feed + mapper.
        string saved = EditorPrefs.GetString(RigPrefKey, "");
        var want = new HashSet<string>(
            saved.Split('\n').Where(s => !string.IsNullOrEmpty(s)));
        bool fallback = want.Count == 0;
        if (fallback)
        {
            var min = new HashSet<GameObject>();
            AddGOs<PXR_24PointMapper>(min);
            AddGOs<PXR_BodyTrackingFeed>(min);
            foreach (var g in min) want.Add(HierPath(g));
        }
        // Always restore AutoFlow, even when the saved record predates it being
        // parked -- without this, a scene put into trim mode by an older build
        // comes back with no calibration and every capture records nothing.
        foreach (var g in set) if (g.GetComponent<PXR_SessionAutoFlow>() != null) want.Add(HierPath(g));
        int n = 0;
        foreach (var g in set)
        {
            bool target = want.Contains(HierPath(g));
            if (g.activeSelf != target)
            {
                Undo.RecordObject(g, "Restore Capture Rig");
                g.SetActive(target);
            }
            if (target) n++;
        }
        EditorPrefs.DeleteKey(RigPrefKey);
        Debug.Log($"[MTC] Restored {n} rig object(s)"
                  + (fallback ? " (no record found -- feed + mapper only)." : "."));
    }

    static string HierPath(GameObject g)
    {
        string p = g.name;
        for (var t = g.transform.parent; t != null; t = t.parent) p = t.name + "/" + p;
        return p;
    }

    static void AddGOs<T>(HashSet<GameObject> set) where T : Component
    {
        foreach (var c in Object.FindObjectsOfType<T>(true))
            if (c.gameObject.scene.IsValid()) set.Add(c.gameObject);
    }

    static T FindSceneObject<T>() where T : Component
    {
        foreach (var c in Object.FindObjectsOfType<T>(true))
            if (c.gameObject.scene.IsValid()) return c;
        return null;
    }
}
