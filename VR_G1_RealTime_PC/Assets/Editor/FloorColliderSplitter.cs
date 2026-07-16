// FloorColliderSplitter.cs  (Editor)
// -----------------------------------------------------------------------------
// The apartment is ONE combined mesh (floor + walls + furniture). We want the body
// collision to ignore ONLY the floor surface — NOT a height band — so low furniture is
// still detected.
//
// Floor detection: the floor is the LARGEST horizontal up-facing surface. We bin every
// up-facing triangle's height (area-weighted) and take the dominant bin -> robust to the
// stray low geometry common in real scans (which fooled a naive "lowest surface").
//
// Two tools (run in EDIT mode, not Play):
//   * MTC > Split Floor From Environment Collision  -> removes the floor triangles from
//        every MeshCollider's collision mesh (REBUILT from the render mesh each time, so it
//        is re-runnable). Render mesh untouched -> floor still visible.
//   * MTC > Snap Player + G1 To Floor               -> moves XR Origin + G1 reference onto
//        the detected floor top (the builder spawns at the bounding-box MIN, which can be
//        below the real floor -> you + the G1 sink).
//
// NOTE: meshes must be Read/Write enabled (apartment_1 import settings) for either tool.
// -----------------------------------------------------------------------------
#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

public static class FloorColliderSplitter
{
    const float UP = 0.85f;   // normal.y above this = horizontal, facing up

    // =========================== Split ===========================
    [MenuItem("MTC/Split Floor From Environment Collision")]
    static void MenuSplit()
    {
        var mcs = Object.FindObjectsOfType<MeshCollider>();
        if (mcs == null || mcs.Length == 0)
        {
            EditorUtility.DisplayDialog("No MeshColliders",
                "No MeshColliders in the open scene. Open StreamingEnv or rebuild it first.", "OK");
            return;
        }
        int n = SplitColliders(mcs, 0.10f);
        EditorUtility.DisplayDialog("Floor split",
            $"Processed {mcs.Length} collider(s), removed the floor from {n}. See Console for details.", "OK");
    }

    public static int Split(GameObject root, float bandMeters)
        => SplitColliders(root.GetComponentsInChildren<MeshCollider>(true), bandMeters);

    /// Rebuilds each collider's mesh from the RENDER mesh with the dominant floor band removed.
    public static int SplitColliders(MeshCollider[] mcs, float bandMeters)
    {
        int modified = 0, removedTotal = 0;
        foreach (var mc in mcs)
        {
            if (mc == null) continue;
            // SOURCE = the render mesh (always the full mesh) so this is re-runnable.
            var mf = mc.GetComponent<MeshFilter>();
            Mesh src = (mf != null && mf.sharedMesh != null) ? mf.sharedMesh : mc.sharedMesh;
            if (src == null) continue;
            if (!src.isReadable)
            {
                Debug.LogError($"[FloorSplit] Mesh '{src.name}' on '{mc.name}' is not readable. Enable Read/Write and re-run.");
                continue;
            }

            Matrix4x4 ltw = mc.transform.localToWorldMatrix;
            Vector3[] v = src.vertices;
            int[] tris = src.triangles;

            var wv = new Vector3[v.Length];
            for (int i = 0; i < v.Length; i++) wv[i] = ltw.MultiplyPoint3x4(v[i]);

            float floorY = DominantFloorY(wv, tris);
            if (float.IsNaN(floorY)) { Debug.LogWarning($"[FloorSplit] '{mc.name}': no horizontal surface found."); continue; }

            var keep = new List<int>(tris.Length);
            int removed = 0;
            for (int t = 0; t < tris.Length; t += 3)
            {
                Vector3 a = wv[tris[t]], b = wv[tris[t + 1]], c = wv[tris[t + 2]];
                Vector3 nrm = Vector3.Cross(b - a, c - a);
                float mag = nrm.magnitude;
                float cy = (a.y + b.y + c.y) / 3f;
                bool isFloor = mag > 1e-9f && (nrm.y / mag) > UP && Mathf.Abs(cy - floorY) < bandMeters;
                if (isFloor) removed++;
                else { keep.Add(tris[t]); keep.Add(tris[t + 1]); keep.Add(tris[t + 2]); }
            }
            if (removed == 0) { Debug.Log($"[FloorSplit] '{mc.name}': nothing matched (floorY={floorY:F3})."); continue; }

            var noFloor = new Mesh { name = src.name + "_NoFloorCollision", indexFormat = IndexFormat.UInt32 };
            noFloor.vertices = v;
            noFloor.triangles = keep.ToArray();
            noFloor.RecalculateBounds();
            mc.sharedMesh = noFloor;
            modified++; removedTotal += removed;
            Debug.Log($"[FloorSplit] '{mc.name}': floorY={floorY:F3}, removed {removed} floor tris, kept {keep.Count / 3}.");
        }
        Debug.Log($"[FloorSplit] Done: modified {modified} collider(s), {removedTotal} floor triangles removed total.");
        return modified;
    }

    // =========================== Snap ===========================
    [MenuItem("MTC/Snap Player + G1 To Floor")]
    static void SnapToFloor()
    {
        // collect the big render meshes (apartment)
        var verts = new List<Vector3>(); var tris = new List<int>();
        foreach (var mf in Object.FindObjectsOfType<MeshFilter>())
        {
            var m = mf.sharedMesh;
            if (m == null || !m.isReadable || m.vertexCount < 5000) continue;   // skip primitives/helpers
            var ltw = mf.transform.localToWorldMatrix;
            int baseI = verts.Count;
            var mv = m.vertices;
            for (int i = 0; i < mv.Length; i++) verts.Add(ltw.MultiplyPoint3x4(mv[i]));
            var mt = m.triangles;
            for (int i = 0; i < mt.Length; i++) tris.Add(baseI + mt[i]);
        }
        if (verts.Count == 0)
        {
            EditorUtility.DisplayDialog("No floor found",
                "No large readable mesh in the scene (apartment). Make sure it's loaded and Read/Write enabled.", "OK");
            return;
        }

        float floorY = DominantFloorY(verts.ToArray(), tris.ToArray());
        if (float.IsNaN(floorY)) { EditorUtility.DisplayDialog("No floor found", "Couldn't find a horizontal floor surface.", "OK"); return; }

        int moved = 0;
        var xo = GameObject.Find("XR Origin");
        if (xo != null) { var p = xo.transform.position; xo.transform.position = new Vector3(p.x, floorY, p.z); moved++; }
        foreach (var t in Object.FindObjectsOfType<Transform>())
            if (t.name.StartsWith("G1_Reference")) { var p = t.position; t.position = new Vector3(p.x, floorY, p.z); moved++; }

        Debug.Log($"[FloorSplit] Snapped {moved} object(s) to floorY={floorY:F3}.");
        EditorUtility.DisplayDialog("Snapped to floor",
            $"Floor (largest horizontal surface) detected at y = {floorY:F3}.\n" +
            $"Moved XR Origin + G1 reference onto it ({moved} object(s)).\n\n" +
            "Check the Scene view — no Play needed. If y still looks wrong, tell me the value " +
            "or just set XR Origin / G1_Reference Y by hand in the Inspector.", "OK");
    }

    // =========================== shared floor detection ===========================
    // Largest (area-weighted) up-facing horizontal surface = the floor.
    static float DominantFloorY(Vector3[] wv, int[] tris)
    {
        const float BIN = 0.05f;
        var area = new Dictionary<int, float>();
        var wYsum = new Dictionary<int, float>();
        for (int t = 0; t < tris.Length; t += 3)
        {
            Vector3 a = wv[tris[t]], b = wv[tris[t + 1]], c = wv[tris[t + 2]];
            Vector3 nrm = Vector3.Cross(b - a, c - a);
            float mag = nrm.magnitude;
            if (mag < 1e-9f || (nrm.y / mag) <= UP) continue;
            float ar = 0.5f * mag;
            float cy = (a.y + b.y + c.y) / 3f;
            int bin = Mathf.RoundToInt(cy / BIN);
            area.TryGetValue(bin, out float pa); area[bin] = pa + ar;
            wYsum.TryGetValue(bin, out float py); wYsum[bin] = py + ar * cy;
        }
        if (area.Count == 0) return float.NaN;
        int best = 0; float bestA = -1f;
        foreach (var kv in area) if (kv.Value > bestA) { bestA = kv.Value; best = kv.Key; }
        return wYsum[best] / area[best];   // area-weighted mean Y of the dominant surface
    }
}
#endif
