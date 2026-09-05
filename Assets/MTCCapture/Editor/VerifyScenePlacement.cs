using UnityEngine;
using UnityEditor;
using System.Text;

// Reports world-space AABB for scene GameObjects placed under WorldOrigin.
// Compare the printed AABB against the expected AABB from:
//   python DP_raw/scripts/verify_scene_placement.py <scene.json>
public class VerifyScenePlacement : EditorWindow
{
    [MenuItem("Tools/Verify Scene Placement (All WorldOrigin Children)")]
    static void VerifyAll()
    {
        GameObject worldOrigin = GameObject.Find("WorldOrigin");
        if (worldOrigin == null)
        {
            EditorUtility.DisplayDialog("Error",
                "No 'WorldOrigin' in the active scene.", "OK");
            return;
        }

        Debug.Log($"[VSP-Header] WorldOrigin pos={Fmt(worldOrigin.transform.position)} " +
                  $"euler={Fmt(worldOrigin.transform.eulerAngles)} " +
                  $"children={worldOrigin.transform.childCount}");

        int reported = 0, skipped = 0;
        foreach (Transform child in worldOrigin.transform)
        {
            if (ReportAabb(child.gameObject)) reported++;
            else skipped++;
        }
        Debug.Log($"[VSP-Footer] reported={reported}  skipped={skipped}");
    }

    [MenuItem("Tools/Verify Scene Placement (Selected)")]
    static void VerifySelected()
    {
        var sel = Selection.activeGameObject;
        if (sel == null)
        {
            EditorUtility.DisplayDialog("Error", "Select a GameObject in the Hierarchy first.", "OK");
            return;
        }
        if (!ReportAabb(sel))
            Debug.LogWarning($"[VSP] No mesh data found in '{sel.name}'.");
    }

    [MenuItem("Tools/Verify Scene Placement (Find disaster_easy_3x4_000)")]
    static void VerifyDisaster000()
    {
        GameObject worldOrigin = GameObject.Find("WorldOrigin");
        if (worldOrigin == null) { Debug.LogError("[VSP] No WorldOrigin"); return; }

        Transform t = worldOrigin.transform.Find("disaster_easy_3x4_000_simplified");
        if (t == null)
        {
            Debug.LogError("[VSP] Could not find 'disaster_easy_3x4_000_simplified' under WorldOrigin. " +
                           "Listing all children...");
            foreach (Transform c in worldOrigin.transform)
                Debug.Log($"[VSP-Child] '{c.name}'");
            return;
        }
        ReportAabb(t.gameObject);
    }

    static bool ReportAabb(GameObject go)
    {
        bool wasActive = go.activeSelf;
        if (!wasActive) go.SetActive(true);
        try
        {
            var filters = go.GetComponentsInChildren<MeshFilter>(includeInactive: true);
            var renderers = go.GetComponentsInChildren<Renderer>(includeInactive: true);

            Vector3 mn = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
            Vector3 mx = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);
            int pts = 0;

            foreach (var mf in filters)
            {
                var mesh = mf.sharedMesh;
                if (mesh == null) continue;
                var ltw = mf.transform.localToWorldMatrix;
                Vector3 c = mesh.bounds.center;
                Vector3 e = mesh.bounds.extents;
                for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                for (int sz = -1; sz <= 1; sz += 2)
                {
                    Vector3 local = c + new Vector3(sx * e.x, sy * e.y, sz * e.z);
                    Vector3 world = ltw.MultiplyPoint3x4(local);
                    mn = Vector3.Min(mn, world); mx = Vector3.Max(mx, world); pts++;
                }
            }

            // Fallback: if no MeshFilter, try SkinnedMeshRenderer / Renderer.bounds
            if (pts == 0)
            {
                foreach (var r in renderers)
                {
                    var b = r.bounds;
                    if (b.size == Vector3.zero) continue;
                    mn = Vector3.Min(mn, b.min); mx = Vector3.Max(mx, b.max); pts += 2;
                }
            }

            var sb = new StringBuilder();
            sb.AppendLine($"[VSP] === {go.name} ===");
            sb.AppendLine($"  meshFilters={filters.Length}  renderers={renderers.Length}  samplePts={pts}");
            sb.AppendLine($"  localPos    = {Fmt(go.transform.localPosition)}");
            sb.AppendLine($"  localEuler  = {Fmt(go.transform.localEulerAngles)}");
            sb.AppendLine($"  localScale  = {Fmt(go.transform.localScale)}");
            if (pts > 0)
            {
                sb.AppendLine($"  world min   = ({mn.x:F4}, {mn.y:F4}, {mn.z:F4})");
                sb.AppendLine($"  world max   = ({mx.x:F4}, {mx.y:F4}, {mx.z:F4})");
                Vector3 sz = mx - mn;
                sb.AppendLine($"  world size  = ({sz.x:F4}, {sz.y:F4}, {sz.z:F4})");
                Debug.Log(sb.ToString());
                return true;
            }
            else
            {
                sb.AppendLine("  (no mesh data — listing first-level children)");
                foreach (Transform c in go.transform)
                    sb.AppendLine($"    - '{c.name}'  active={c.gameObject.activeSelf}");
                Debug.LogWarning(sb.ToString());
                return false;
            }
        }
        finally
        {
            if (!wasActive) go.SetActive(false);
        }
    }

    static string Fmt(Vector3 v) => $"({v.x:F4}, {v.y:F4}, {v.z:F4})";
}
