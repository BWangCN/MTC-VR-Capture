// MTC_FixCaptureSession.cs  (Editor, one-shot)
// -----------------------------------------------------------------------------
// Puts the capture scene back into a recordable state and loads this session's
// envs, in one pass:
//
//   1. AutoFlow.hmd -> XR Origin (XR Rig)/Camera Offset/Main Camera
//   2. AtomicEnvSet.ImportAll()   (honours _index/session_filter.txt)
//   3. save the scene
//
// Why step 1 matters: PXR_SessionAutoFlow aborts its whole coroutine on
// `pivotAtHMD && hmd == null`, so nothing downstream of it ever runs -- no
// calibration, no worldRoot rescale, and critically `mapper.roomSpaceRef` is
// never assigned. MTC_AtomicSession.Boot() then spins on that reference for its
// full 60 s timeout and the controller buttons do nothing. The reference was
// present on 2026-08-22 (fileID 119422673) and is empty now; the sessions on
// 08-24/25/28 kept working only because they ran an APK built before it was lost.
//
// Runs itself once on script reload so an already-open Editor fixes itself, and
// is also on the menu. The guard key makes "once" mean once per project.
// -----------------------------------------------------------------------------
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[InitializeOnLoad]
public static class MTC_FixCaptureSession
{
    const string GuardKey = "MTC_FixCaptureSession_20260904";
    const string ScenePath = "Assets/Scenes/SampleScene.unity";

    static MTC_FixCaptureSession()
    {
        if (EditorPrefs.GetBool(GuardKey, false)) return;
        // The guard is set by Run() only once the scene is saved: a run that bails
        // early (wrong scene open, camera not found) must not burn the one shot.
        EditorApplication.delayCall += () => Run();
    }

    [MenuItem("MTC/Fix Capture Session (hmd + envs)")]
    public static void Run()
    {
        var scene = EditorSceneManager.GetActiveScene();
        if (scene.path != ScenePath)
        {
            Debug.LogWarning($"[MTC-FIX] active scene is '{scene.path}', expected {ScenePath}. " +
                             "Open the capture scene and run MTC > Fix Capture Session again.");
            return;
        }

        int changed = 0;

        // --- 1. AutoFlow.hmd -----------------------------------------------
        var flow = Object.FindObjectsOfType<PXR_SessionAutoFlow>(true).FirstOrDefault();
        if (flow == null)
        {
            Debug.LogError("[MTC-FIX] no PXR_SessionAutoFlow in the scene.");
            return;
        }
        if (flow.hmd == null)
        {
            var cam = HmdCamera();
            if (cam == null)
            {
                Debug.LogError("[MTC-FIX] could not find an active Main Camera under " +
                               "XR Origin/Camera Offset. Assign AutoFlow.hmd by hand.");
                return;
            }
            Undo.RecordObject(flow, "Wire AutoFlow.hmd");
            flow.hmd = cam;
            EditorUtility.SetDirty(flow);
            changed++;
            Debug.Log($"[MTC-FIX] AutoFlow.hmd -> {Path(cam)}");
        }
        else
        {
            Debug.Log($"[MTC-FIX] AutoFlow.hmd already set to {Path(flow.hmd)}; left alone.");
        }

        // --- 2. this session's envs ----------------------------------------
        AtomicEnvSet.ImportAll();

        // --- 3. save --------------------------------------------------------
        if (!EditorSceneManager.SaveScene(EditorSceneManager.GetActiveScene()))
        {
            Debug.LogError("[MTC-FIX] SaveScene returned false -- scene NOT saved.");
            return;
        }

        EditorPrefs.SetBool(GuardKey, true);
        var session = Object.FindObjectsOfType<MTC_AtomicSession>(true).FirstOrDefault();
        int n = session == null ? 0 : session.envOrder.Count;
        Debug.Log($"[MTC-FIX] done. hmd fixes: {changed}. Session carries {n} env(s). " +
                  "Scene saved -- REBUILD THE APK before capturing.");
        if (n == 0)
            Debug.LogError("[MTC-FIX] session envOrder is EMPTY -- the build would have nothing " +
                           "to record. Check _index/session_filter.txt against the generated envs.");
    }

    /// The headset camera is the active one under the XR rig. The scene also holds
    /// a second, inactive "Main Camera"; picking that one silently reproduces the
    /// null-hmd failure, so match on the rig path rather than on the name.
    static Transform HmdCamera()
    {
        var cams = Object.FindObjectsOfType<Camera>(true)
            .Select(c => c.transform)
            .Where(t => t.gameObject.activeInHierarchy)
            .ToList();
        var under = cams.FirstOrDefault(t => Path(t).Contains("Camera Offset"));
        return under != null ? under : cams.FirstOrDefault(t => t.name == "Main Camera");
    }

    static string Path(Transform t)
    {
        var s = t.name;
        for (var p = t.parent; p != null; p = p.parent) s = p.name + "/" + s;
        return s;
    }
}
