// RemoveMissingEnvObjects.cs  (Editor tool)
// -----------------------------------------------------------------------------
// One-click cleanup for scene objects that turned red after their source model
// (FBX) was deleted from the project. These are prefab/model instances whose
// asset is now missing (PrefabInstanceStatus.MissingAsset).
//
//   MTC > Missing Env Objects > Count      -- report how many, without changing
//   MTC > Missing Env Objects > Remove      -- delete them (Undo-able), then Ctrl+S
//
// Operates on the ACTIVE scene only, so you see exactly what changed. Run it once
// per open scene if more than one is affected.
// -----------------------------------------------------------------------------
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class RemoveMissingEnvObjects
{
    [MenuItem("MTC/Missing Env Objects/Count")]
    static void Count()
    {
        var found = Collect();
        if (found.Count == 0) { Debug.Log("[MTC] No missing prefab instances in the active scene."); return; }
        var names = new List<string>();
        for (int i = 0; i < found.Count && i < 20; i++) names.Add(found[i].name);
        Debug.Log($"[MTC] {found.Count} missing prefab instance(s) in the active scene.\n" +
                  $"First few: {string.Join(", ", names)}{(found.Count > 20 ? " ..." : "")}\n" +
                  "Run  MTC > Missing Env Objects > Remove  to delete them.");
    }

    [MenuItem("MTC/Missing Env Objects/Remove")]
    static void Remove()
    {
        var found = Collect();
        if (found.Count == 0) { Debug.Log("[MTC] Nothing to remove."); return; }

        Undo.SetCurrentGroupName("Remove missing env objects");
        int grp = Undo.GetCurrentGroup();
        foreach (var go in found)
            if (go != null) Undo.DestroyObjectImmediate(go);
        Undo.CollapseUndoOperations(grp);

        var scene = SceneManager.GetActiveScene();
        EditorSceneManager.MarkSceneDirty(scene);
        Debug.Log($"[MTC] Removed {found.Count} missing prefab instance(s). " +
                  "Save the scene (Ctrl+S) to make it permanent. Undo (Ctrl+Z) if needed.");
    }

    static List<GameObject> Collect()
    {
        var list = new List<GameObject>();
        var scene = SceneManager.GetActiveScene();
        if (!scene.IsValid() || !scene.isLoaded) return list;
        foreach (var root in scene.GetRootGameObjects())
            Walk(root, list);
        return list;
    }

    static void Walk(GameObject go, List<GameObject> list)
    {
        // A model/prefab instance whose source asset was deleted shows here.
        if (PrefabUtility.GetPrefabInstanceStatus(go) == PrefabInstanceStatus.MissingAsset)
        {
            list.Add(go);
            return; // take the whole broken instance; don't recurse into it
        }
        var children = new List<Transform>();
        foreach (Transform c in go.transform) children.Add(c);
        foreach (var c in children) Walk(c.gameObject, list);
    }
}
