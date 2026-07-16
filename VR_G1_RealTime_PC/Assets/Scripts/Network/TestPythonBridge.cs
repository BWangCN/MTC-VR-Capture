using UnityEngine;

public class TestPythonBridge : MonoBehaviour
{
    void Start()
    {
        Debug.Log("=== TestPythonBridge Start ===");

        var bridge = GetComponent<PythonRetargetBridge>();

        if (bridge == null)
        {
            Debug.LogError("❌ PythonRetargetBridge component NOT FOUND on this GameObject!");
        }
        else
        {
            Debug.Log("✅ PythonRetargetBridge component found!");
            Debug.Log($"   - isReady: {bridge.isReady}");
            Debug.Log($"   - Python Path: {bridge.pythonPath}");
            Debug.Log($"   - Script Path: {bridge.scriptPath}");
            Debug.Log($"   - Retarget FPS: {bridge.retargetFPS}");
        }
    }
}
