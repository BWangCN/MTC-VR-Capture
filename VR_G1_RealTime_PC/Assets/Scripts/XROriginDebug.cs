using UnityEngine;

/// <summary>
/// 调试脚本：验证XR Origin是否正常工作
/// 显示Camera的位置和旋转，确认追踪数据正在更新
/// </summary>
public class XROriginDebug : MonoBehaviour
{
    public Camera xrCamera;
    public bool showDebug = true;

    void Start()
    {
        // 自动查找Main Camera
        if (xrCamera == null)
        {
            xrCamera = Camera.main;
        }

        if (xrCamera == null)
        {
            Debug.LogError("[XROriginDebug] No camera found!");
            enabled = false;
        }
    }

    void Update()
    {
        if (!showDebug || xrCamera == null) return;

        // 每60帧打印一次Camera位置和旋转
        if (Time.frameCount % 60 == 0)
        {
            Debug.Log($"[XR Camera] Pos: {xrCamera.transform.position}, Rot: {xrCamera.transform.rotation.eulerAngles}");
        }
    }

    void OnGUI()
    {
        if (!showDebug || xrCamera == null) return;

        GUI.color = Color.cyan;
        GUI.Label(new Rect(10, 200, 500, 30), $"[XR Camera] Position: {xrCamera.transform.position}");
        GUI.Label(new Rect(10, 230, 500, 30), $"[XR Camera] Rotation: {xrCamera.transform.rotation.eulerAngles}");
    }
}
