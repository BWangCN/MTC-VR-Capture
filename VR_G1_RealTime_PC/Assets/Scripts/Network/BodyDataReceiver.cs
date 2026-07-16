using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System;

/// <summary>
/// PC端网络接收器：接收PICO发送的24点body tracking数据
///
/// 使用方法:
/// 1. 添加到PC Unity项目的GameObject
/// 2. 设置监听端口（默认9000）
/// 3. 运行，等待PICO连接
/// </summary>
public class BodyDataReceiver : MonoBehaviour
{
    [Header("Network Settings")]
    public int port = 9876;

    [Header("Received Data - READ ONLY")]
    [Tooltip("接收到的24个关节位置（Unity world space）")]
    public Vector3[] receivedJoints = new Vector3[24];

    [Tooltip("接收到的24个关节旋转（Unity quaternion xyzw）")]
    public Quaternion[] receivedRotations = new Quaternion[24];

    [Header("Status")]
    public bool isReceiving = false;
    public int framesReceived = 0;
    public float receivedFPS = 0f;
    public float networkLatency = 0f;  // 网络延迟估算 (ms)

    [Header("Debug")]
    public bool showDebugInfo = true;
    public bool logFirstFrame = true;

    private UdpClient udpClient;
    private Thread receiveThread;
    private bool running = false;

    private float lastFPSUpdate = 0f;
    private int frameCount = 0;
    private uint lastFrameNumber = 0;

    // 线程安全的临时缓冲区
    private Vector3[] tempPositions = new Vector3[24];
    private Quaternion[] tempRotations = new Quaternion[24];
    private readonly object dataLock = new object();

    void Start()
    {
        try
        {
            udpClient = new UdpClient(port);
            running = true;
            isReceiving = true;

            receiveThread = new Thread(ReceiveData);
            receiveThread.IsBackground = true;
            receiveThread.Start();

            Debug.Log($"[BodyReceiver] Listening on port {port}...");
        }
        catch (Exception e)
        {
            Debug.LogError($"[BodyReceiver] Failed to start: {e.Message}");
            enabled = false;
        }
    }

    void ReceiveData()
    {
        IPEndPoint remoteEP = new IPEndPoint(IPAddress.Any, port);
        byte[] receivedBytes;

        while (running)
        {
            try
            {
                receivedBytes = udpClient.Receive(ref remoteEP);
                UnpackBodyData(receivedBytes);
            }
            catch (SocketException e)
            {
                if (running)  // 只在非正常关闭时报错
                {
                    Debug.LogError($"[BodyReceiver] Socket error: {e.Message}");
                }
            }
            catch (Exception e)
            {
                if (running)
                {
                    Debug.LogError($"[BodyReceiver] Receive error: {e.Message}");
                }
            }
        }
    }

    void UnpackBodyData(byte[] data)
    {
        const int HEADER_SIZE = 8;
        const int BYTES_PER_JOINT = 28;
        const int NUM_JOINTS = 24;
        const int EXPECTED_SIZE = HEADER_SIZE + NUM_JOINTS * BYTES_PER_JOINT;
        const uint MAGIC_NUMBER = 0xFEEDBEEF;

        if (data.Length != EXPECTED_SIZE)
        {
            Debug.LogWarning($"[BodyReceiver] Invalid data size: {data.Length} (expected {EXPECTED_SIZE})");
            return;
        }

        int offset = 0;

        // === Verify Magic Number ===
        uint magic = BitConverter.ToUInt32(data, offset);
        if (magic != MAGIC_NUMBER)
        {
            Debug.LogWarning($"[BodyReceiver] Invalid magic number: 0x{magic:X8}");
            return;
        }
        offset += 4;

        // === Frame Number ===
        uint frameNumber = BitConverter.ToUInt32(data, offset);
        offset += 4;

        // 检测丢帧
        if (lastFrameNumber > 0 && frameNumber != lastFrameNumber + 1)
        {
            int dropped = (int)(frameNumber - lastFrameNumber - 1);
            if (dropped > 0)
            {
                Debug.LogWarning($"[BodyReceiver] Dropped {dropped} frame(s) (jump from {lastFrameNumber} to {frameNumber})");
            }
        }
        lastFrameNumber = frameNumber;

        // === Unpack 24 Joints ===
        Vector3[] positions = new Vector3[NUM_JOINTS];
        Quaternion[] rotations = new Quaternion[NUM_JOINTS];

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            // Position
            float px = BitConverter.ToSingle(data, offset);
            float py = BitConverter.ToSingle(data, offset + 4);
            float pz = BitConverter.ToSingle(data, offset + 8);
            offset += 12;

            // Rotation (xyzw)
            float rx = BitConverter.ToSingle(data, offset);
            float ry = BitConverter.ToSingle(data, offset + 4);
            float rz = BitConverter.ToSingle(data, offset + 8);
            float rw = BitConverter.ToSingle(data, offset + 12);
            offset += 16;

            positions[i] = new Vector3(px, py, pz);
            rotations[i] = new Quaternion(rx, ry, rz, rw);
        }

        // 线程安全地更新数据
        lock (dataLock)
        {
            Array.Copy(positions, tempPositions, NUM_JOINTS);
            Array.Copy(rotations, tempRotations, NUM_JOINTS);
        }

        // Log first frame for debugging
        if (logFirstFrame && framesReceived == 0)
        {
            Debug.Log($"[BodyReceiver] First frame received! Frame #{frameNumber}");
            Debug.Log($"  Pelvis pos: {positions[0]}");
            Debug.Log($"  Pelvis rot: {rotations[0]}");
            logFirstFrame = false;
        }

        framesReceived++;
        frameCount++;
    }

    void Update()
    {
        if (!isReceiving)
            return;

        // 从后台线程复制数据到主线程
        lock (dataLock)
        {
            Array.Copy(tempPositions, receivedJoints, 24);
            Array.Copy(tempRotations, receivedRotations, 24);
        }

        // 调试：每60帧打印一次Pelvis位置（检查数据是否在变化）
        if (showDebugInfo && Time.frameCount % 60 == 0)
        {
            Debug.Log($"[BodyReceiver DEBUG] Pelvis pos: {receivedJoints[0]}, Left knee pos: {receivedJoints[4]}");
        }

        // 更新FPS计数
        if (Time.time - lastFPSUpdate >= 1.0f)
        {
            receivedFPS = frameCount / (Time.time - lastFPSUpdate);
            frameCount = 0;
            lastFPSUpdate = Time.time;

            if (showDebugInfo && framesReceived > 0)
            {
                Debug.Log($"[BodyReceiver] Receiving @ {receivedFPS:F1} FPS, Total frames: {framesReceived}");
            }
        }
    }

    void OnDestroy()
    {
        running = false;

        if (receiveThread != null && receiveThread.IsAlive)
        {
            receiveThread.Join(1000);  // 等待最多1秒
        }

        udpClient?.Close();
        Debug.Log($"[BodyReceiver] Stopped. Total frames received: {framesReceived}");
    }

    void OnGUI()
    {
        if (!showDebugInfo || !isReceiving)
            return;

        // 在屏幕显示状态
        GUI.color = framesReceived > 0 ? Color.green : Color.yellow;
        GUI.Label(new Rect(10, 10, 400, 30), $"Listening on port {port}");
        GUI.Label(new Rect(10, 40, 400, 30), $"FPS: {receivedFPS:F1} | Frames: {framesReceived}");

        if (framesReceived > 0)
        {
            GUI.Label(new Rect(10, 70, 400, 30), $"Pelvis: {receivedJoints[0]}");
        }
        else
        {
            GUI.color = Color.yellow;
            GUI.Label(new Rect(10, 70, 400, 30), "Waiting for PICO connection...");
        }
    }
}
