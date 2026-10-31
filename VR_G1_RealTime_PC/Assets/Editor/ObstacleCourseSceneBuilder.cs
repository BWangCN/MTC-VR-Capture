#if UNITY_EDITOR
using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

/// <summary>Imports the course using the existing streaming scene's XR/mocap rig.</summary>
public static class ObstacleCourseSceneBuilder
{
    public const string ScenePath = "Assets/Scenes/ObstacleCourse_7b6477cb95_seed0.unity";
    const string PackagePath = "Assets/RealEnvs/7b6477cb95_seed0";
    const string TemplatePath = "Assets/Scenes/StreamingEnv.unity";
    const string FloorMeshPath = PackagePath + "/RoomCollision_NoFloor.asset";

    [Serializable] class Layout { public float floor_z_m; }
    [Serializable] class Route { public float[] start_m; public float[] goal_m; }
    [Serializable] class CourseTask { public Route route; }

    [MenuItem("MTC/Build Obstacle Course (7b6477cb95 seed 0)")]
    public static void Build()
    {
        if (!Application.isBatchMode)
        {
            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo()) return;
            if (File.Exists(ScenePath) && !EditorUtility.DisplayDialog("Rebuild obstacle course?",
                    "This regenerates " + ScenePath + ". Save any custom scene changes separately first.",
                    "Rebuild", "Cancel")) return;
        }

        AssetDatabase.Refresh();
        var visualAsset = LoadModel("scene.glb");
        var collisionAsset = LoadModel("collision.glb");
        var markerAsset = LoadModel("markers.glb");
        var layout = JsonUtility.FromJson<Layout>(File.ReadAllText(PackagePath + "/layout.json"));
        var task = JsonUtility.FromJson<CourseTask>(File.ReadAllText(PackagePath + "/task.json"));
        if (task?.route?.start_m?.Length != 3 || task.route.goal_m?.Length != 3)
            throw new InvalidDataException("The course must define three-dimensional route start and goal positions.");

        // Save separately BEFORE modifying the template's environment.
        var scene = EditorSceneManager.OpenScene(TemplatePath, OpenSceneMode.Single);
        if (!EditorSceneManager.SaveScene(scene, ScenePath)) throw new IOException("Could not save " + ScenePath);
        var embodiment = FindInScene<RealEnvEmbodiment>(scene);
        var origin = scene.GetRootGameObjects().Single(go => go.name == "XR Origin");
        if (embodiment == null || embodiment.worldRoot == null)
            throw new InvalidOperationException("StreamingEnv must contain an Embodiment with a worldRoot.");
        var oldEnvironment = embodiment.worldRoot.gameObject;
        var environment = new GameObject("Environment");
        SceneManager.MoveGameObjectToScene(environment, scene);

        // The export is metric/Y-up; glTFast handles handedness. No orientation heuristic.
        InstantiateModel(visualAsset, "Visuals", environment.transform);
        var collision = InstantiateModel(collisionAsset, "Collision", environment.transform);
        InstantiateModel(markerAsset, "Route Markers (optional)", environment.transform);
        ObstacleCourseMaterials.Configure(environment);
        int colliderCount = 0;
        foreach (var filter in collision.GetComponentsInChildren<MeshFilter>(true))
        {
            var mesh = filter.sharedMesh;
            if (mesh == null) continue;
            var collider = filter.gameObject.AddComponent<MeshCollider>();
            // Strip only the room floor from contact feedback, preserving all object hulls.
            collider.sharedMesh = filter.name.StartsWith("room_collision", StringComparison.Ordinal)
                ? CreateRoomContactMesh(filter) : mesh;
            collider.convex = false; // Stationary proxies; no Rigidbody/grabbing in this viewer.
            colliderCount++;
        }
        foreach (var renderer in collision.GetComponentsInChildren<Renderer>(true)) renderer.enabled = false;
        if (colliderCount == 0) throw new InvalidDataException("No collision proxy meshes were imported.");

        var start = SceneToUnity(task.route.start_m, layout.floor_z_m);
        var goal = SceneToUnity(task.route.goal_m, layout.floor_z_m);
        var direction = goal - start;
        direction.y = 0;
        origin.transform.SetPositionAndRotation(start, Quaternion.LookRotation(direction, Vector3.up));
        embodiment.worldRoot = environment.transform;
        // Preserve the metric export by default. Enable explicitly for G1 scaling.
        embodiment.enabled = false;
        foreach (var receiver in FindAllInScene<OptiTrackPointReceiver>(scene))
        {
            var serialized = new SerializedObject(receiver);
            serialized.FindProperty("apartmentRoot").objectReferenceValue = environment;
            serialized.ApplyModifiedPropertiesWithoutUndo();
        }
        foreach (var haptics in FindAllInScene<BodyCollisionHaptics>(scene)) haptics.environmentRoot = environment.transform;
        Object.DestroyImmediate(oldEnvironment);

        if (!EditorSceneManager.SaveScene(scene)) throw new IOException("Could not save obstacle course scene.");
        AssetDatabase.SaveAssets();
        Validate();
        Debug.Log($"[ObstacleCourse] Built {ScenePath}: {colliderCount} collision proxies, spawn {start}. " +
                  "Metric scale, route markers enabled. Connect ALVR/SteamVR and press Play.");
    }

    static GameObject LoadModel(string name)
    {
        var model = AssetDatabase.LoadAssetAtPath<GameObject>(PackagePath + "/" + name);
        if (model == null) throw new InvalidDataException("glTFast could not import " + name);
        return model;
    }

    static GameObject InstantiateModel(GameObject asset, string name, Transform parent)
    {
        var instance = (GameObject)PrefabUtility.InstantiatePrefab(asset, parent);
        instance.name = name;
        instance.transform.localPosition = Vector3.zero;
        instance.transform.localRotation = Quaternion.identity;
        instance.transform.localScale = Vector3.one;
        return instance;
    }

    static Vector3 SceneToUnity(float[] position, float floor)
        => new Vector3(-position[1], position[2] - floor, position[0]);

    static Mesh CreateRoomContactMesh(MeshFilter filter)
    {
        var source = filter.sharedMesh;
        if (!source.isReadable) throw new InvalidDataException("Room collision mesh must be readable.");
        var vertices = source.vertices;
        var triangles = source.triangles;
        var kept = new System.Collections.Generic.List<int>(triangles.Length);
        var matrix = filter.transform.localToWorldMatrix;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            var a = matrix.MultiplyPoint3x4(vertices[triangles[i]]);
            var b = matrix.MultiplyPoint3x4(vertices[triangles[i + 1]]);
            var c = matrix.MultiplyPoint3x4(vertices[triangles[i + 2]]);
            var normal = Vector3.Cross(b - a, c - a).normalized;
            // Floor is known to be y=0. Test all vertices to retain low furniture.
            bool floor = Mathf.Abs(normal.y) > 0.85f &&
                         Mathf.Abs(a.y) < 0.03f && Mathf.Abs(b.y) < 0.03f && Mathf.Abs(c.y) < 0.03f;
            if (!floor) { kept.Add(triangles[i]); kept.Add(triangles[i + 1]); kept.Add(triangles[i + 2]); }
        }
        var mesh = new Mesh { name = "RoomCollision_NoFloor", indexFormat = IndexFormat.UInt32 };
        mesh.vertices = vertices;
        mesh.triangles = kept.ToArray();
        mesh.RecalculateBounds();
        var saved = AssetDatabase.LoadAssetAtPath<Mesh>(FloorMeshPath);
        if (saved == null) { AssetDatabase.CreateAsset(mesh, FloorMeshPath); saved = mesh; }
        else { EditorUtility.CopySerialized(mesh, saved); Object.DestroyImmediate(mesh); }
        Debug.Log($"[ObstacleCourse] Room contact mesh: removed {(triangles.Length - kept.Count) / 3} floor triangles.");
        return saved;
    }

    static T[] FindAllInScene<T>(Scene scene) where T : Component
        => scene.GetRootGameObjects().SelectMany(go => go.GetComponentsInChildren<T>(true)).ToArray();

    static T FindInScene<T>(Scene scene) where T : Component => FindAllInScene<T>(scene).FirstOrDefault();

    public static void Validate()
    {
        var scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        var environment = scene.GetRootGameObjects().Single(go => go.name == "Environment");
        ObstacleCourseMaterials.Validate(environment);
        var colliders = environment.GetComponentsInChildren<MeshCollider>(true);
        if (colliders.Length == 0 || colliders.Any(c => c.sharedMesh == null))
            throw new InvalidDataException("Course has missing collision meshes.");
        if (environment.transform.Find("Visuals").GetComponentsInChildren<MeshCollider>(true).Length != 0)
            throw new InvalidDataException("Display meshes must not be used for collision.");
        if (environment.transform.Find("Collision").GetComponentsInChildren<Renderer>(true).Any(r => r.enabled))
            throw new InvalidDataException("Collision proxies must be invisible.");
        foreach (var receiver in FindAllInScene<OptiTrackPointReceiver>(scene))
            if (new SerializedObject(receiver).FindProperty("apartmentRoot").objectReferenceValue != environment)
                throw new InvalidDataException("Mocap contact feedback is not bound to the course.");
        if (FindInScene<RealEnvEmbodiment>(scene).enabled)
            throw new InvalidDataException("The course must start at metric scale.");
        Debug.Log($"[ObstacleCourse] Validation passed: {colliders.Length} persistent collision meshes and mocap bindings.");
    }
}
#endif
