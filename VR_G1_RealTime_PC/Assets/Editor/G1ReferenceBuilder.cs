// G1ReferenceBuilder.cs
// -----------------------------------------------------------------------------
// Builds a static, ~1.27 m tall G1-PROPORTIONED humanoid as a HEIGHT REFERENCE
// (no physics, no animation, no colliders) from Unity primitives. This is a
// zero-dependency, physics-free stand-in. The REAL G1 (URDF + STL meshes) is also
// in Assets/unitree_g1 and can be imported via the URDF Importer for exact geometry.
//
// Meant to be parented under the scaled "Environment" so it tracks the embodiment
// scaling (stays 1.27 m relative to the room) — after calibration it should look
// about the player's height, confirming you are at G1 scale.
//
// Used by StreamingEnvSceneBuilder, and available via  MTC > Add G1 Height Reference.
// -----------------------------------------------------------------------------
#if UNITY_EDITOR
using UnityEditor;
using UnityEngine;

public static class G1ReferenceBuilder
{
    public const float G1Height = 1.27f;

    [MenuItem("MTC/Add G1 Height Reference (1.27 m)")]
    public static void AddToSceneMenu()
    {
        var go = Build(Vector3.zero, Quaternion.identity, null, true);
        Undo.RegisterCreatedObjectUndo(go, "Add G1 Reference");
        Selection.activeGameObject = go;
    }

    /// Build the reference figure. footWorldPos = where the feet touch the floor.
    /// rot = facing. If parent != null it is parented keeping world pose (so it
    /// inherits the parent's runtime scaling).
    public static GameObject Build(Vector3 footWorldPos, Quaternion rot, Transform parent, bool withLabel)
    {
        Material body = MakeMat(new Color(1.00f, 0.45f, 0.10f)); // G1-ish orange
        Material dark = MakeMat(new Color(0.16f, 0.16f, 0.19f)); // joints / head

        var root = new GameObject("G1_Reference (1.27m)");
        root.transform.SetPositionAndRotation(footWorldPos, rot);

        // Body parts (metres; feet at y=0, head top ~1.27).  Capsules are vertical.
        AddBox(root, "pelvis", body,  0.00f, 0.620f, 0.00f, 0.22f, 0.14f, 0.18f);
        AddBox(root, "torso",  body,  0.00f, 0.920f, 0.00f, 0.26f, 0.46f, 0.18f);
        AddSphere(root, "head", dark, 0.00f, 1.205f, 0.00f, 0.13f);              // top ~1.27

        AddCapsule(root, "L_thigh", body, 0.09f, 0.480f, 0.0f, 0.045f, 0.30f);
        AddCapsule(root, "R_thigh", body,-0.09f, 0.480f, 0.0f, 0.045f, 0.30f);
        AddCapsule(root, "L_shin",  dark, 0.09f, 0.190f, 0.0f, 0.040f, 0.30f);
        AddCapsule(root, "R_shin",  dark,-0.09f, 0.190f, 0.0f, 0.040f, 0.30f);
        AddBox(root, "L_foot", dark, 0.09f, 0.030f, 0.05f, 0.10f, 0.06f, 0.20f);
        AddBox(root, "R_foot", dark,-0.09f, 0.030f, 0.05f, 0.10f, 0.06f, 0.20f);

        AddCapsule(root, "L_uarm", body, 0.175f, 0.920f, 0.0f, 0.035f, 0.26f);
        AddCapsule(root, "R_uarm", body,-0.175f, 0.920f, 0.0f, 0.035f, 0.26f);
        AddCapsule(root, "L_farm", dark, 0.175f, 0.660f, 0.0f, 0.030f, 0.24f);
        AddCapsule(root, "R_farm", dark,-0.175f, 0.660f, 0.0f, 0.030f, 0.24f);

        if (withLabel) AddLabel(root, "G1  1.27 m", 1.42f);

        if (parent != null)
            root.transform.SetParent(parent, true);  // keep world pose; inherit scaling

        return root;
    }

    // ---- helpers ----------------------------------------------------------------

    static Material MakeMat(Color c)
    {
        Shader sh = Shader.Find("Universal Render Pipeline/Lit");
        if (sh == null) sh = Shader.Find("Standard");
        if (sh == null) sh = Shader.Find("Unlit/Color");
        var m = new Material(sh) { color = c };
        if (m.HasProperty("_BaseColor")) m.SetColor("_BaseColor", c);
        return m;
    }

    static void Prep(GameObject go, GameObject root, Material m, Vector3 pos, Vector3 scale)
    {
        var col = go.GetComponent<Collider>();
        if (col != null) Object.DestroyImmediate(col);          // height ref: no physics
        var r = go.GetComponent<Renderer>();
        if (r != null) r.sharedMaterial = m;
        go.transform.SetParent(root.transform, false);
        go.transform.localPosition = pos;
        go.transform.localScale = scale;
    }

    static void AddBox(GameObject root, string name, Material m, float x, float y, float z, float sx, float sy, float sz)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube); go.name = name;
        Prep(go, root, m, new Vector3(x, y, z), new Vector3(sx, sy, sz));
    }

    static void AddSphere(GameObject root, string name, Material m, float x, float y, float z, float d)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere); go.name = name;
        Prep(go, root, m, new Vector3(x, y, z), new Vector3(d, d, d));
    }

    // vertical capsule, total length `len`, radius `r`. Unity capsule = 2 tall, r 0.5.
    static void AddCapsule(GameObject root, string name, Material m, float x, float y, float z, float r, float len)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Capsule); go.name = name;
        Prep(go, root, m, new Vector3(x, y, z), new Vector3(r * 2f, len * 0.5f, r * 2f));
    }

    static void AddLabel(GameObject root, string text, float y)
    {
        var font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (font == null) font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        if (font == null) return;  // no built-in font -> skip label (figure still works)

        var label = new GameObject("Label");
        label.transform.SetParent(root.transform, false);
        label.transform.localPosition = new Vector3(0f, y, 0f);
        label.transform.localRotation = Quaternion.Euler(0f, 180f, 0f); // face same way as figure
        var tm = label.AddComponent<TextMesh>();
        tm.font = font;
        tm.text = text;
        tm.anchor = TextAnchor.LowerCenter;
        tm.alignment = TextAlignment.Center;
        tm.fontSize = 48;
        tm.characterSize = 0.06f;
        tm.color = Color.white;
        var mr = label.GetComponent<MeshRenderer>();
        if (mr != null) mr.sharedMaterial = font.material;
    }
}
#endif
