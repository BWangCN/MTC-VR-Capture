// ImportStandardEnvs.cs  (Editor tool)
// -----------------------------------------------------------------------------
// One-click import + side-by-side layout of the 3 curated STANDARD corridors,
// so you can inspect their quality in the Unity scene without dragging FBX in
// one at a time.
//
//   MTC > Standard Envs > Import + Lay Out for Review
//       Copies the 3 std FBX from E:\MTC_data\capture into Assets\FBX_Scenes,
//       imports them, and instantiates all 3 side-by-side under "STD_REVIEW".
//   MTC > Standard Envs > Clear Review
//       Removes the STD_REVIEW instances (keeps the imported FBX assets).
//
// The FBX stay in FBX_Scenes afterwards, so you can capture in them directly.
// -----------------------------------------------------------------------------
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class ImportStandardEnvs
{
    const string SrcRoot = @"E:\MTC_data\capture";
    const string DstDir = "Assets/FBX_Scenes";
    const string ReviewRoot = "STD_REVIEW";
    const float SpacingX = 3.5f;   // lateral gap between the parallel corridors

    // Scanned, not hardcoded: layout versions arrive as new env ids
    // (corridor_std_01 -> corridor_std_01_v2) and a fixed list silently hides them.
    static string[] Envs =>
        Directory.Exists(SrcRoot)
            ? Directory.GetDirectories(SrcRoot, "corridor_std_*")
                .Select(Path.GetFileName)
                .Where(e => File.Exists(Path.Combine(SrcRoot, e, "mesh", e + "_simplified.fbx")))
                .OrderBy(e => e)
                .ToArray()
            : new string[0];

    [MenuItem("MTC/Standard Envs/Import + Lay Out for Review")]
    static void ImportReview()
    {
        ClearReview();
        Directory.CreateDirectory(DstDir);
        var parent = new GameObject(ReviewRoot);
        Undo.RegisterCreatedObjectUndo(parent, "Import Standard Envs");

        float x = 0f;
        int ok = 0;
        var envs = Envs;
        foreach (var env in envs)
        {
            string src = Path.Combine(SrcRoot, env, "mesh", env + "_simplified.fbx");
            if (!File.Exists(src)) { Debug.LogWarning($"[MTC] missing FBX: {src}"); continue; }

            string dst = $"{DstDir}/{env}_simplified.fbx";
            File.Copy(src, Path.GetFullPath(dst), true);
            AssetDatabase.ImportAsset(dst, ImportAssetOptions.ForceSynchronousImport);

            var asset = AssetDatabase.LoadAssetAtPath<GameObject>(dst);
            if (asset == null) { Debug.LogWarning($"[MTC] import failed: {dst}"); continue; }

            var go = (GameObject)PrefabUtility.InstantiatePrefab(asset);
            go.name = "REVIEW_" + env;
            go.transform.SetParent(parent.transform, false);
            go.transform.position = new Vector3(x, 0f, 0f);
            x += SpacingX;
            ok++;
        }

        AssetDatabase.Refresh();
        Selection.activeGameObject = parent;
        EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
        Debug.Log($"[MTC] Imported + laid out {ok}/{envs.Length} standard env(s) under " +
                  $"'{ReviewRoot}'. Select it and press F in the Scene view to frame. " +
                  "Run 'Clear Review' (FBX stay in FBX_Scenes) when done.");
    }

    [MenuItem("MTC/Standard Envs/Clear Review")]
    static void ClearReview()
    {
        var root = GameObject.Find(ReviewRoot);
        if (root != null)
        {
            Undo.DestroyObjectImmediate(root);
            EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
            Debug.Log("[MTC] Cleared STD_REVIEW (imported FBX kept in FBX_Scenes).");
        }
    }
}
