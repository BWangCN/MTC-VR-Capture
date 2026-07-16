using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using UnityEngine;
using Debug = UnityEngine.Debug;

/// <summary>
/// Unity-Python桥接：调用Python retarget服务
///
/// 使用方法:
/// 1. 确保安装了Python和GeneralMotionRetargeting库
/// 2. 在Inspector中设置Python路径和脚本路径
/// 3. 调用Retarget()方法传入PICO 24点数据
/// 4. 读取retargeted qpos from rootPos, rootRot, dofPos
/// </summary>
public class PythonRetargetBridge : MonoBehaviour
{
    [Header("Python Settings")]
    [Tooltip("Python可执行文件路径（或'python'如果在PATH中）")]
    public string pythonPath = "python";

    [Tooltip("pico_retarget_server.py的完整路径")]
    public string scriptPath = @"C:\Users\Wayne\Desktop\GMU\PhD\VR_G1_RealTime\Python\pico_retarget_server.py";

    [Header("Retargeted Output - READ ONLY")]
    [Tooltip("G1 root position (meters)")]
    public Vector3 rootPos;

    [Tooltip("G1 root rotation (MuJoCo wxyz quaternion)")]
    public Vector4 rootRotWxyz;

    [Tooltip("G1 29个关节角度 (radians)")]
    public float[] dofPos = new float[29];

    [Header("Status")]
    public bool isReady = false;
    public int framesProcessed = 0;
    public float retargetFPS = 0f;

    [Header("Debug")]
    public bool showDebugInfo = true;

    private Process pythonProcess;
    private StreamWriter pythonStdin;
    private StreamReader pythonStdout;
    private Thread stderrThread;
    private Thread stdoutThread;

    // 线程安全队列：存储Python返回的JSON结果
    private Queue<string> resultQueue = new Queue<string>();
    private object queueLock = new object();

    private float lastFPSUpdate = 0f;
    private int frameCount = 0;

    void Start()
    {
        // 延迟启动：等待首次调用Retarget()时再启动Python
        // 这样避免：1) PICO未连接时浪费资源  2) 主线程阻塞2秒
        Debug.Log("[PythonBridge] Initialized (Python server will start on first retarget call)");
    }

    void StartPythonServer()
    {
        if (!File.Exists(scriptPath))
        {
            Debug.LogError($"[PythonBridge] Script not found: {scriptPath}");
            enabled = false;
            return;
        }

        try
        {
            pythonProcess = new Process();
            pythonProcess.StartInfo.FileName = pythonPath;
            pythonProcess.StartInfo.Arguments = $"\"{scriptPath}\"";
            pythonProcess.StartInfo.UseShellExecute = false;
            pythonProcess.StartInfo.RedirectStandardInput = true;
            pythonProcess.StartInfo.RedirectStandardOutput = true;
            pythonProcess.StartInfo.RedirectStandardError = true;
            pythonProcess.StartInfo.CreateNoWindow = true;

            // 设置working directory为GMR项目目录
            string gmrPath = Path.GetDirectoryName(Path.GetDirectoryName(scriptPath));
            pythonProcess.StartInfo.WorkingDirectory = gmrPath;

            Debug.Log($"[PythonBridge] Starting Python server: {pythonPath} {scriptPath}");
            pythonProcess.Start();

            pythonStdin = pythonProcess.StandardInput;
            pythonStdout = pythonProcess.StandardOutput;

            // 启动stderr读取线程（用于监控Python日志）
            stderrThread = new Thread(ReadStderr);
            stderrThread.IsBackground = true;
            stderrThread.Start();

            // 启动stdout读取线程（异步读取retarget结果，避免阻塞主线程）
            stdoutThread = new Thread(ReadStdout);
            stdoutThread.IsBackground = true;
            stdoutThread.Start();

            // 不再阻塞主线程等待！
            // Python进程已启动，stderr/stdout线程会异步处理I/O
            // 立即标记为ready，首次retarget可能会失败但会重试
            isReady = true;
            Debug.Log("[PythonBridge] Python server started (initializing in background...)");
        }
        catch (Exception e)
        {
            Debug.LogError($"[PythonBridge] Failed to start Python: {e.Message}");
            enabled = false;
        }
    }

    void ReadStderr()
    {
        try
        {
            while (!pythonProcess.HasExited)
            {
                string line = pythonProcess.StandardError.ReadLine();
                if (line != null)
                {
                    Debug.Log($"[Python] {line}");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[PythonBridge] Stderr thread error: {e.Message}");
        }
    }

    void ReadStdout()
    {
        try
        {
            while (!pythonProcess.HasExited)
            {
                string line = pythonStdout.ReadLine();
                if (line != null && line.Length > 0)
                {
                    // 只将JSON格式的结果加入队列（跳过非JSON行）
                    if (line.Contains("\"root_pos\"") && line.Contains("\"dof_pos\""))
                    {
                        lock (queueLock)
                        {
                            resultQueue.Enqueue(line);
                        }
                    }
                    else if (showDebugInfo)
                    {
                        Debug.Log($"[Python stdout] {line}");
                    }
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[PythonBridge] Stdout thread error: {e.Message}");
        }
    }

    /// <summary>
    /// Retarget PICO 24点数据到G1
    /// </summary>
    /// <param name="positions">24个关节位置 (Unity Vector3)</param>
    /// <param name="rotations">24个关节旋转 (Unity Quaternion xyzw)</param>
    public void Retarget(Vector3[] positions, Quaternion[] rotations)
    {
        // 延迟启动：首次调用时才启动Python进程
        if (pythonProcess == null && !isReady)
        {
            Debug.Log("[PythonBridge] First retarget call - starting Python server now...");
            StartPythonServer();

            // StartPythonServer会设置isReady，如果失败则直接返回
            if (!isReady)
            {
                Debug.LogError("[PythonBridge] Failed to start Python server");
                return;
            }
        }

        if (!isReady || pythonProcess == null || pythonProcess.HasExited)
        {
            if (showDebugInfo && Time.frameCount % 60 == 0)
            {
                Debug.LogWarning("[PythonBridge] Not ready or Python process exited");
            }
            return;
        }

        try
        {
            // === 1. 准备JSON数据 ===
            StringBuilder sb = new StringBuilder();
            sb.Append("{\"positions\":[");

            for (int i = 0; i < positions.Length; i++)
            {
                sb.AppendFormat("[{0},{1},{2}]", positions[i].x, positions[i].y, positions[i].z);
                if (i < positions.Length - 1) sb.Append(",");
            }

            sb.Append("],\"rotations\":[");

            for (int i = 0; i < rotations.Length; i++)
            {
                // Unity quaternion: xyzw order
                sb.AppendFormat("[{0},{1},{2},{3}]",
                    rotations[i].x, rotations[i].y, rotations[i].z, rotations[i].w);
                if (i < rotations.Length - 1) sb.Append(",");
            }

            sb.Append("]}");

            string json = sb.ToString();

            // === 2. 发送给Python ===
            pythonStdin.WriteLine(json);
            pythonStdin.Flush();

            // === 3. 从队列读取结果（非阻塞）===
            // 后台线程异步读取stdout并加入队列，主线程在这里取出结果
            // 重要：只使用最新的结果，丢弃旧结果（避免延迟累积）
            string resultLine = null;
            int queueSize = 0;
            lock (queueLock)
            {
                queueSize = resultQueue.Count;
                // 排空队列，只保留最新的结果
                while (resultQueue.Count > 0)
                {
                    resultLine = resultQueue.Dequeue();
                }
            }

            // 调试：每60帧报告队列状态
            if (showDebugInfo && Time.frameCount % 60 == 0)
            {
                Debug.Log($"[PythonBridge DEBUG] Queue size: {queueSize}, Got result: {resultLine != null}");
            }

            if (resultLine != null)
            {
                ParseRetargetResult(resultLine);
                framesProcessed++;
                frameCount++;
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[PythonBridge] Retarget error: {e.Message}");
        }
    }

    void ParseRetargetResult(string json)
    {
        try
        {
            // 简单的JSON解析（避免依赖Newtonsoft.Json）
            // 格式: {"root_pos": [x,y,z], "root_rot_wxyz": [w,x,y,z], "dof_pos": [...]}
            // 注意：Python输出的JSON在冒号后有空格

            int rootPosStart = json.IndexOf("\"root_pos\": [") + 14;
            int rootPosEnd = json.IndexOf("]", rootPosStart);
            string rootPosStr = json.Substring(rootPosStart, rootPosEnd - rootPosStart);
            string[] rootPosVals = rootPosStr.Split(',');
            rootPos = new Vector3(
                float.Parse(rootPosVals[0].Trim(), CultureInfo.InvariantCulture),
                float.Parse(rootPosVals[1].Trim(), CultureInfo.InvariantCulture),
                float.Parse(rootPosVals[2].Trim(), CultureInfo.InvariantCulture)
            );

            int rootRotStart = json.IndexOf("\"root_rot_wxyz\": [") + 19;
            int rootRotEnd = json.IndexOf("]", rootRotStart);
            string rootRotStr = json.Substring(rootRotStart, rootRotEnd - rootRotStart);
            string[] rootRotVals = rootRotStr.Split(',');
            rootRotWxyz = new Vector4(
                float.Parse(rootRotVals[0].Trim(), CultureInfo.InvariantCulture),  // w
                float.Parse(rootRotVals[1].Trim(), CultureInfo.InvariantCulture),  // x
                float.Parse(rootRotVals[2].Trim(), CultureInfo.InvariantCulture),  // y
                float.Parse(rootRotVals[3].Trim(), CultureInfo.InvariantCulture)   // z
            );

            int dofPosStart = json.IndexOf("\"dof_pos\": [") + 13;
            int dofPosEnd = json.IndexOf("]", dofPosStart);
            string dofPosStr = json.Substring(dofPosStart, dofPosEnd - dofPosStart);
            string[] dofPosVals = dofPosStr.Split(',');

            int numDof = Mathf.Min(dofPosVals.Length, dofPos.Length);
            for (int i = 0; i < numDof; i++)
            {
                dofPos[i] = float.Parse(dofPosVals[i].Trim(), CultureInfo.InvariantCulture);
            }

            // 调试：打印某个关节的值（检查是否在更新）
            if (showDebugInfo && framesProcessed % 60 == 0)
            {
                Debug.Log($"[PythonBridge DEBUG] Parsed dofPos[3] = {dofPos[3]:F3} rad");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[PythonBridge] JSON parse error: {e.Message}\nJSON: {json}");
        }
    }

    void Update()
    {
        // 更新FPS计数
        if (Time.time - lastFPSUpdate >= 1.0f)
        {
            retargetFPS = frameCount / (Time.time - lastFPSUpdate);
            frameCount = 0;
            lastFPSUpdate = Time.time;

            if (showDebugInfo && framesProcessed > 0)
            {
                Debug.Log($"[PythonBridge] Retarget @ {retargetFPS:F1} FPS, Total frames: {framesProcessed}");
            }
        }
    }

    void OnDestroy()
    {
        if (pythonProcess != null && !pythonProcess.HasExited)
        {
            pythonStdin?.Close();
            pythonProcess.Kill();
            Debug.Log($"[PythonBridge] Stopped Python server. Total frames: {framesProcessed}");
        }
    }

    void OnGUI()
    {
        if (!showDebugInfo)
            return;

        GUI.color = isReady ? Color.green : Color.red;
        GUI.Label(new Rect(10, 100, 400, 30), $"Python Retarget: {(isReady ? "Ready" : "NOT READY")}");

        if (isReady)
        {
            GUI.color = Color.green;
            GUI.Label(new Rect(10, 130, 400, 30), $"Retarget FPS: {retargetFPS:F1} | Frames: {framesProcessed}");
            GUI.Label(new Rect(10, 160, 400, 30), $"Root Pos: {rootPos}");
        }
    }
}
