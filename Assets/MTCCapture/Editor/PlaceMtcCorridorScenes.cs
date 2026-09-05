using UnityEngine;
using UnityEditor;
using System.IO;
using System.Linq;

// Places MTC Challenge v1 corridor FBX scenes under WorldOrigin.
//
// The corridor FBX files (produced by SimEnv/mtc_corridor/scripts/export_to_unity.py)
// are exported from Blender with axis_up=Y so they import IDENTITY into Unity.
// That is different from the older bedroom/livingroom/kitchen/disaster FBX
// files (exported from Isaac Sim USD) which need a (270, 180, 0) rotation
// compensation -- those are handled by PlaceDisasterScenes.cs.
//
// File naming convention: corridor_<tier>_<seed:08d>_simplified.fbx
public class PlaceMtcCorridorScenes : EditorWindow
{
    private const string FBX_PREFIX  = "corridor_";
    private const string FBX_SUFFIX  = "_simplified.fbx";
    private const string FBX_FOLDER  = "Assets/FBX_Scenes";

    [MenuItem("Tools/Place MTC Corridor Scenes Under WorldOrigin")]
    static void Execute()
    {
        GameObject worldOrigin = GameObject.Find("WorldOrigin");
        if (worldOrigin == null)
        {
            EditorUtility.DisplayDialog("Error",
                "Cannot find 'WorldOrigin' in the current scene.\n" +
                "Open the scene that contains WorldOrigin first.",
                "OK");
            return;
        }

        string[] guids = AssetDatabase.FindAssets("t:GameObject", new[] { FBX_FOLDER });
        var fbxAssets = guids
            .Select(g => AssetDatabase.GUIDToAssetPath(g))
            .Where(p =>
                Path.GetFileName(p).StartsWith(FBX_PREFIX, System.StringComparison.OrdinalIgnoreCase) &&
                p.EndsWith(FBX_SUFFIX, System.StringComparison.OrdinalIgnoreCase))
            .OrderBy(p => Path.GetFileNameWithoutExtension(p))
            .ToArray();

        if (fbxAssets.Length == 0)
        {
            EditorUtility.DisplayDialog("Nothing to place",
                $"No {FBX_PREFIX}*{FBX_SUFFIX} found in {FBX_FOLDER}.\n" +
                "Run SimEnv/mtc_corridor/scripts/export_to_unity.py first.",
                "OK");
            return;
        }

        int added = 0;
        int skipped = 0;

        foreach (string assetPath in fbxAssets)
        {
            string name = Path.GetFileNameWithoutExtension(assetPath);

            Transform existing = worldOrigin.transform.Find(name);
            if (existing != null)
            {
                skipped++;
                continue;
            }

            GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(assetPath);
            if (prefab == null) continue;

            GameObject instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
            instance.name = name;
            instance.transform.SetParent(worldOrigin.transform, false);
            instance.transform.localPosition = Vector3.zero;
            // IDENTITY -- the FBX export now bakes the corridor's long axis to
            // Unity +Z (see fbx_builder.py's +90deg-around-Z pre-export rotation),
            // so the corridor faces the player's forward in PICO VR without any
            // rotation applied here.
            instance.transform.localRotation = Quaternion.identity;
            instance.transform.localScale = Vector3.one;

            // Disable by default so only one corridor is visible at a time.
            instance.SetActive(false);

            Undo.RegisterCreatedObjectUndo(instance, "Place MTC Corridor Scene");
            added++;
        }

        Debug.Log($"[PlaceMtcCorridorScenes] Done: {added} added, {skipped} skipped (already exist).");
        EditorUtility.DisplayDialog("Done",
            $"Placed {added} corridor scenes under WorldOrigin (all disabled).\n" +
            $"Skipped {skipped} (already existed).\n\n" +
            "Activate one at a time from the Hierarchy to test in PICO.",
            "OK");
    }
}
