// AtomicEnvSet.cs  (Editor tool)
// -----------------------------------------------------------------------------
// One-click assembly of the ATOMIC capture build:
//
//   MTC > Atomic Envs > Import All + Wire Session
//       Rebuilds the scene to hold exactly the envs still WAITING for capture:
//       copies their FBX from <MTC_CAPTURE_ROOT> into Assets/FBX_Scenes,
//       instantiates each under WorldOrigin at IDENTITY, leaves one active,
//       deactivates the non-atomic corridors, and fills MTC_AtomicSession's
//       envOrder. Any env that already has a stowed take is REMOVED from the
//       scene -- re-run this after each capture session and the set shrinks to
//       what is left to do. The FBX assets stay in the project either way.
//       Save the scene, build the APK, and every remaining env is switchable on
//       the headset with a controller button.
//
//   MTC > Atomic Envs > Remove All
//       Deletes the instantiated atomic envs from the scene. The imported FBX
//       assets stay in Assets/FBX_Scenes.
//
// Identity placement is load-bearing, not cosmetic: the -90 deg travel-axis fix
// is baked into the FBX by export_to_unity.env_to_blender_scene, and the runtime
// coordinate contract assumes worldRoot's local origin IS the corridor start.
// Never add a rotation or offset here.
// -----------------------------------------------------------------------------
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class AtomicEnvSet
{
    const string DstDir = "Assets/FBX_Scenes";
    const string WorldOriginName = "WorldOrigin";
    const string EnvPrefix = "corridor_atomic_";

    static string CaptureRoot =>
        System.Environment.GetEnvironmentVariable("MTC_CAPTURE_ROOT") ?? @"E:\MTC_data\capture";

    // Capture order. The five clearance rungs run tallest-first so a session
    // walks the posture ladder -- duck, squat, hands-and-knees, forearms, prone --
    // instead of jumping between gaits. Compounds last, baseline at the end.
    static readonly string[] BehaviourOrder = {
        "beam", "squat", "doghole", "lowcrawl", "highcrawl", "prone",
        "ground", "lblock", "squeeze", "duckstep", "crawlstep", "free",
        "steprow", "wleft", "wright", "wslit",
    };

    /// Optional comma-separated behaviour list, e.g. "steprow,wleft,wright,wslit".
    /// A session usually targets a couple of behaviours, while "everything not yet
    /// captured" also drags in whatever rungs earlier sets left unfinished. Absent
    /// means the old behaviour: every pending env.
    ///
    /// The file is read first because an Editor that is already open cannot see an
    /// environment variable set after it launched -- editing the file works without
    /// restarting Unity.
    const string FilterFile = "_index/session_filter.txt";

    static string[] Filter()
    {
        string s = null;
        string f = Path.Combine(CaptureRoot, FilterFile);
        if (File.Exists(f))
            s = string.Join(",", File.ReadAllLines(f)
                                     .Select(l => l.Split('#')[0].Trim())
                                     .Where(l => l.Length > 0));
        if (string.IsNullOrWhiteSpace(s))
            s = System.Environment.GetEnvironmentVariable("MTC_ENV_FILTER");
        return string.IsNullOrWhiteSpace(s)
            ? null
            : s.Split(',').Select(x => x.Trim()).Where(x => x.Length > 0).ToArray();
    }

    static bool Wanted(string envId, string[] filter)
    {
        if (filter == null) return true;
        string slug = envId.Substring(EnvPrefix.Length);
        int cut = slug.LastIndexOf('_');
        string beh = cut > 0 ? slug.Substring(0, cut) : slug;
        return filter.Contains(beh);
    }

    /// An env is DONE once it has a stowed take; the session should stop
    /// offering it and the scene should stop carrying its mesh.
    static bool IsCaptured(string envId)
    {
        string raw = Path.Combine(MtcCaptureRoot.EnvDir(envId), "raw");
        return Directory.Exists(raw) &&
               Directory.GetFiles(raw, "*.csv")
                        .Any(f => !f.EndsWith(".collisions.csv"));
    }

    static List<string> DiscoverEnvs()
    {
        if (!Directory.Exists(CaptureRoot)) return new List<string>();
        var ids = MtcCaptureRoot.EnvDirs()
            .Where(kv => kv.Key.StartsWith(EnvPrefix) &&
                         File.Exists(Path.Combine(kv.Value, "mesh", kv.Key + "_simplified.fbx")))
            .Select(kv => kv.Key)
            .ToList();

        int Rank(string id)
        {
            string slug = id.Substring(EnvPrefix.Length);
            int cut = slug.LastIndexOf('_');
            string beh = cut > 0 ? slug.Substring(0, cut) : slug;
            int i = System.Array.IndexOf(BehaviourOrder, beh);
            return i < 0 ? BehaviourOrder.Length : i;
        }
        ids.Sort((a, b) =>
        {
            int r = Rank(a).CompareTo(Rank(b));
            return r != 0 ? r : string.CompareOrdinal(a, b);
        });
        return ids;
    }

    [MenuItem("MTC/Atomic Envs/Import All + Wire Session")]
    public static void ImportAll()
    {
        var worldOrigin = GameObject.Find(WorldOriginName);
        if (worldOrigin == null)
        {
            EditorUtility.DisplayDialog("No " + WorldOriginName,
                "This scene has no '" + WorldOriginName + "' object to parent the corridors to.\n" +
                "Open the capture scene first.", "OK");
            return;
        }

        var filter = Filter();
        var all = DiscoverEnvs();
        var envs = all.Where(e => !IsCaptured(e) && Wanted(e, filter)).ToList();
        // Anything the session is not going to record comes OUT of the scene:
        // already captured, or filtered out of this session's behaviour list.
        var done = all.Where(e => IsCaptured(e) || !Wanted(e, filter)).ToList();
        if (all.Count == 0)
        {
            EditorUtility.DisplayDialog("No atomic envs",
                "Found no " + EnvPrefix + "* with a mesh under:\n" + CaptureRoot +
                "\n\nGenerate them first:\n" +
                "  python scripts/make_standard_envs.py --version atomic\n" +
                "  python scripts/make_standard_envs.py --version atomic2", "OK");
            return;
        }

        Directory.CreateDirectory(DstDir);
        var existing = new Dictionary<string, Transform>();
        foreach (Transform c in worldOrigin.transform)
            if (c.name.StartsWith("corridor_")) existing[c.name] = c;

        // An env this session will not record comes out of the scene AND out of the
        // project: the mesh so the build stops carrying it and the controller stops
        // offering it, the FBX so the asset list stays the size of the session. The
        // capture root keeps the authoritative copy under <env>/mesh/, so a retired
        // env is one re-import away from coming back.
        int retired = 0, purged = 0;
        foreach (var env in done)
        {
            if (existing.TryGetValue(env, out var t0) && t0 != null)
            {
                Undo.DestroyObjectImmediate(t0.gameObject);
                existing.Remove(env);
                retired++;
            }
            string stale = DstDir + "/" + env + "_simplified.fbx";
            if (File.Exists(Path.Combine(Application.dataPath, "FBX_Scenes", env + "_simplified.fbx"))
                && AssetDatabase.DeleteAsset(stale))
                purged++;
        }

        int ok = 0;
        try
        {
            for (int i = 0; i < envs.Count; i++)
            {
                string env = envs[i];
                EditorUtility.DisplayProgressBar("MTC Atomic Envs", env, (float)i / envs.Count);

                string fbxName = env + "_simplified.fbx";
                string src = Path.Combine(MtcCaptureRoot.EnvDir(env), "mesh", fbxName);
                string assetPath = DstDir + "/" + fbxName;
                string dst = Path.Combine(Application.dataPath, "FBX_Scenes", fbxName);

                // Re-copy when the sizes differ so a regenerated env can never be
                // captured against a stale mesh left in the project.
                bool stale = !File.Exists(dst) || new FileInfo(src).Length != new FileInfo(dst).Length;
                if (stale)
                {
                    File.Copy(src, dst, true);
                    AssetDatabase.ImportAsset(assetPath, ImportAssetOptions.ForceSynchronousImport);
                }

                var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
                if (prefab == null) { Debug.LogWarning("[MTC] import failed: " + assetPath); continue; }

                if (!existing.TryGetValue(env, out var t) || t == null)
                {
                    var go = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
                    go.name = env;
                    go.transform.SetParent(worldOrigin.transform, false);
                    Undo.RegisterCreatedObjectUndo(go, "Import atomic env");
                    t = go.transform;
                    existing[env] = t;
                }
                t.localPosition = Vector3.zero;      // see header: identity is load-bearing
                t.localRotation = Quaternion.identity;
                t.localScale = Vector3.one;
                t.gameObject.SetActive(i == 0);
                ok++;
            }
        }
        finally { EditorUtility.ClearProgressBar(); }

        // Non-atomic corridors stay in the scene but off, so they neither render
        // nor confuse Body24Collision / GoalZone (both key off ACTIVE meshes).
        int hidden = 0;
        foreach (var kv in existing)
            if (!kv.Key.StartsWith(EnvPrefix) && kv.Value != null && kv.Value.gameObject.activeSelf)
            {
                Undo.RecordObject(kv.Value.gameObject, "Hide non-atomic corridor");
                kv.Value.gameObject.SetActive(false);
                hidden++;
            }

        var session = Object.FindObjectOfType<MTC_AtomicSession>();
        if (session == null)
        {
            var go = new GameObject("MTC_AtomicSession");
            session = go.AddComponent<MTC_AtomicSession>();
            Undo.RegisterCreatedObjectUndo(go, "Add MTC_AtomicSession");
        }
        Undo.RecordObject(session, "Wire atomic env order");
        session.envNamePrefix = EnvPrefix;
        session.envOrder = new List<string>(envs);
        EditorUtility.SetDirty(session);

        AssetDatabase.Refresh();
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Selection.activeGameObject = session.gameObject;
        Debug.Log($"[MTC] atomic set ready: {ok}/{envs.Count} pending env(s) under " +
                  $"{WorldOriginName}" +
                  (filter != null ? $" [filter: {string.Join(",", filter)}]" : "") +
                  $", {retired} env(s) retired from the scene, {purged} FBX purged, " +
                  $"{hidden} non-atomic corridor(s) deactivated, " +
                  $"'{(envs.Count > 0 ? envs[0] : "-")}' active. " +
                  "Save the scene, then build the APK once.");
    }

    /// Batchmode entry: opens the capture scene, rebuilds the set, saves.
    /// `-executeMethod AtomicEnvSet.BatchImport`, with MTC_ENV_FILTER set to the
    /// session's behaviours. Exits non-zero on anything that would otherwise be a
    /// silent no-op, so a failed run cannot look like a successful one.
    public static void BatchImport()
    {
        const string scene = "Assets/Scenes/SampleScene.unity";
        try
        {
            EditorSceneManager.OpenScene(scene, OpenSceneMode.Single);
            ImportAll();
            var s = EditorSceneManager.GetActiveScene();
            if (!EditorSceneManager.SaveScene(s))
                throw new System.Exception("SaveScene returned false for " + scene);
            var session = Object.FindObjectOfType<MTC_AtomicSession>();
            int n = session == null ? 0 : session.envOrder.Count;
            Debug.Log($"[MTC] BatchImport saved {scene}; session carries {n} env(s).");
            if (n == 0) throw new System.Exception("session envOrder is empty");
            EditorApplication.Exit(0);
        }
        catch (System.Exception e)
        {
            Debug.LogError("[MTC] BatchImport FAILED: " + e);
            EditorApplication.Exit(2);
        }
    }

    [MenuItem("MTC/Atomic Envs/Remove All")]
    static void RemoveAll()
    {
        var worldOrigin = GameObject.Find(WorldOriginName);
        if (worldOrigin == null) return;
        var doomed = new List<GameObject>();
        foreach (Transform c in worldOrigin.transform)
            if (c.name.StartsWith(EnvPrefix)) doomed.Add(c.gameObject);
        foreach (var g in doomed) Undo.DestroyObjectImmediate(g);
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log($"[MTC] removed {doomed.Count} atomic env instance(s); FBX assets kept in {DstDir}.");
    }

    [MenuItem("MTC/Atomic Envs/List Envs on Disk")]
    static void ListEnvs()
    {
        var envs = DiscoverEnvs();
        Debug.Log($"[MTC] {envs.Count} atomic env(s) under {CaptureRoot}:\n  " + string.Join("\n  ", envs));
    }
}
