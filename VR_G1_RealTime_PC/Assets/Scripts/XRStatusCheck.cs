using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Management;

/// <summary>
/// 检查XR系统状态，诊断OpenXR是否正常工作
/// </summary>
public class XRStatusCheck : MonoBehaviour
{
    void Start()
    {
        Debug.Log("=== XR Status Check ===");

        // 检查XR是否已启用
        bool xrEnabled = XRSettings.enabled;
        Debug.Log($"XR Enabled: {xrEnabled}");

        // 检查XR设备
        string deviceName = XRSettings.loadedDeviceName;
        Debug.Log($"Loaded XR Device: {deviceName}");

        // 检查XR Manager
        var xrManager = XRGeneralSettings.Instance;
        if (xrManager != null && xrManager.Manager != null)
        {
            Debug.Log($"XR Manager Active Loader: {xrManager.Manager.activeLoader}");
        }
        else
        {
            Debug.LogWarning("XR Manager not found!");
        }

        // 检查输入设备
        var inputDevices = new System.Collections.Generic.List<InputDevice>();
        InputDevices.GetDevices(inputDevices);
        Debug.Log($"Input Devices Count: {inputDevices.Count}");
        foreach (var device in inputDevices)
        {
            Debug.Log($"  - {device.name} ({device.characteristics})");
        }

        Debug.Log("======================");
    }

    void OnGUI()
    {
        GUI.color = Color.yellow;
        GUI.Label(new Rect(10, 300, 600, 30), $"XR Enabled: {XRSettings.enabled}");
        GUI.Label(new Rect(10, 330, 600, 30), $"XR Device: {XRSettings.loadedDeviceName}");

        var xrManager = XRGeneralSettings.Instance;
        if (xrManager != null && xrManager.Manager != null)
        {
            string loaderName = xrManager.Manager.activeLoader != null
                ? xrManager.Manager.activeLoader.GetType().Name
                : "None";
            GUI.Label(new Rect(10, 360, 600, 30), $"Active Loader: {loaderName}");
        }
    }
}
