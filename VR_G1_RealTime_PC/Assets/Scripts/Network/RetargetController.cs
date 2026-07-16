using UnityEngine;

/// <summary>
/// 连接BodyDataReceiver和PythonRetargetBridge
/// 将接收到的PICO数据发送给Python进行retarget
/// </summary>
public class RetargetController : MonoBehaviour
{
    [Header("References")]
    [Tooltip("拖入BodyDataReceiver组件")]
    public BodyDataReceiver bodyReceiver;

    [Tooltip("拖入PythonRetargetBridge组件")]
    public PythonRetargetBridge retargetBridge;

    [Header("Settings")]
    [Tooltip("每N帧retarget一次（降采样以提升性能）")]
    public int retargetEveryNFrames = 2;  // 72Hz → 36Hz

    [Header("Status - READ ONLY")]
    public bool isProcessing = false;
    public int framesSkipped = 0;

    private int frameCounter = 0;

    void Start()
    {
        // 自动查找组件（如果未手动设置）
        if (bodyReceiver == null)
        {
            bodyReceiver = GetComponent<BodyDataReceiver>();
            if (bodyReceiver == null)
            {
                Debug.LogError("[RetargetController] BodyDataReceiver not found!");
                enabled = false;
                return;
            }
        }

        if (retargetBridge == null)
        {
            retargetBridge = GetComponent<PythonRetargetBridge>();
            if (retargetBridge == null)
            {
                Debug.LogError("[RetargetController] PythonRetargetBridge not found!");
                enabled = false;
                return;
            }
        }

        Debug.Log("[RetargetController] Initialized successfully!");
    }

    void Update()
    {
        // 只等待BodyReceiver准备好
        // 不检查retargetBridge.isReady，因为它会在首次调用时自动启动
        if (!bodyReceiver.isReceiving)
        {
            // 调试：每60帧输出一次状态
            if (Time.frameCount % 60 == 0)
            {
                Debug.LogWarning($"[RetargetController] Waiting for PICO connection... BodyReceiver.isReceiving={bodyReceiver.isReceiving}");
            }
            return;
        }

        // 降采样：每N帧处理一次
        frameCounter++;
        if (frameCounter % retargetEveryNFrames != 0)
        {
            framesSkipped++;
            return;
        }

        // 处理retarget
        isProcessing = true;

        try
        {
            // 从BodyDataReceiver获取24点数据
            Vector3[] positions = bodyReceiver.receivedJoints;
            Quaternion[] rotations = bodyReceiver.receivedRotations;

            if (positions == null || rotations == null || positions.Length != 24 || rotations.Length != 24)
            {
                Debug.LogWarning("[RetargetController] Invalid body data received");
                isProcessing = false;
                return;
            }

            // 调试：每60帧打印一次输入数据（检查是否在变化）
            if (Time.frameCount % 60 == 0)
            {
                Debug.Log($"[RetargetController DEBUG] Sending to Python: Pelvis={positions[0]}, LeftKnee={positions[4]}");
            }

            // 调用PythonRetargetBridge进行retarget
            retargetBridge.Retarget(positions, rotations);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[RetargetController] Error during retarget: {e.Message}");
        }
        finally
        {
            isProcessing = false;
        }
    }

    void OnGUI()
    {
        if (!enabled) return;

        GUI.color = Color.cyan;
        GUI.Label(new Rect(10, 100, 400, 30), $"[RetargetController] Processing: {isProcessing}");
        GUI.Label(new Rect(10, 130, 400, 30), $"Frames Skipped: {framesSkipped} (Downsampling 1/{retargetEveryNFrames})");
    }
}
