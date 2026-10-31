#if UNITY_EDITOR
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

// These exported meshes have COLOR_0 but no glTF material. Give them a standalone
// material instead of relying on glTFast's shared default material across imports.
[InitializeOnLoad]
public static class ObstacleCourseMaterials
{
    const string MaterialPath = "Assets/RealEnvs/7b6477cb95_seed0/VertexColors.mat";
    const string ShaderPath = "Assets/Shaders/ScanVertexColor.shader";

    static ObstacleCourseMaterials()
    {
        EditorApplication.delayCall += RepairLoadedCourse;
        EditorSceneManager.sceneOpened += (scene, mode) => RepairLoadedCourse();
    }

    [MenuItem("MTC/Repair Obstacle Course Materials")]
    public static void RepairLoadedCourse()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode) return;
        var scene = SceneManager.GetActiveScene();
        if (scene.path != ObstacleCourseSceneBuilder.ScenePath) return;
        var environment = scene.GetRootGameObjects().FirstOrDefault(go => go.name == "Environment");
        if (environment == null) return;
        bool alreadyDirty = scene.isDirty;
        int repaired = Configure(environment);
        if (repaired == 0) return;
        EditorSceneManager.MarkSceneDirty(scene);
        // Preserve any unsaved scene work for the user to save deliberately.
        if (!alreadyDirty) EditorSceneManager.SaveScene(scene);
        AssetDatabase.SaveAssets();
        Validate(environment);
        Debug.Log($"[ObstacleCourse] Repaired {repaired} vertex-color material slots. " +
                  (alreadyDirty ? "Save the scene to retain this change." : "Scene saved."));
    }

    public static int Configure(GameObject environment)
    {
        var shader = AssetDatabase.LoadAssetAtPath<Shader>(ShaderPath);
        if (shader == null || !shader.isSupported || ShaderUtil.ShaderHasError(shader))
            throw new InvalidDataException("The scan vertex-color shader could not compile.");
        var material = AssetDatabase.LoadAssetAtPath<Material>(MaterialPath);
        if (material == null)
        {
            material = new Material(shader) { name = "Course Vertex Colors", enableInstancing = true };
            AssetDatabase.CreateAsset(material, MaterialPath);
        }
        int count = 0;
        var markers = environment.transform.Find("Route Markers (optional)");
        foreach (var renderer in environment.GetComponentsInChildren<MeshRenderer>(true))
        {
            bool vertexColored = renderer.name == "room" || renderer.name == "item_01_gate_narrow_gate" ||
                                 (markers != null && renderer.transform.IsChildOf(markers));
            if (!vertexColored) continue;
            var mesh = renderer.GetComponent<MeshFilter>()?.sharedMesh;
            if (mesh == null || !mesh.HasVertexAttribute(UnityEngine.Rendering.VertexAttribute.Color))
                throw new InvalidDataException("Expected vertex colors on " + renderer.name);
            var materials = renderer.sharedMaterials;
            for (int i = 0; i < materials.Length; i++)
                if (materials[i] != material) { materials[i] = material; count++; }
            renderer.sharedMaterials = materials;
        }
        return count;
    }

    public static void Validate(GameObject environment)
    {
        foreach (var renderer in environment.GetComponentsInChildren<Renderer>(true).Where(r => r.enabled))
        {
            if (renderer.sharedMaterials.Length == 0) throw new InvalidDataException("No material on " + renderer.name);
            foreach (var material in renderer.sharedMaterials)
                if (material == null || material.shader == null || !material.shader.isSupported ||
                    ShaderUtil.ShaderHasError(material.shader))
                    throw new InvalidDataException("Missing or unsupported material on " + renderer.name);
        }
    }
}
#endif
