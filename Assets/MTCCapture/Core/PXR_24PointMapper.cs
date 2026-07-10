// ========================= PXR_24PointMapper.cs (FULL) =========================
// Changes vs your version:
// 1) Write META lines at top of CSV (record_origin_pos / record_origin_rot_xyzw / embodiment_scale / yaw_offset_deg)
// 2) Nothing else in your logic is removed; only minimal additions + small cleanups.
// Notes:
// - META lines start with '#', so your CSV remains valid to parse.
// - record_origin_* are taken from trackingOrigin at the time recording starts (BeginRecording -> OpenWriterIfNeeded).
// - This enables strict method-3 replay: p_replay = T_replayOrigin * inv(T_recordOrigin) * p_record

using System;
using System.IO;
using System.Globalization;
using UnityEngine;
using UnityEngine.Events;

public class PXR_24PointMapper : MonoBehaviour
{
    [Header("Source")]
    public PXR_BodyTrackingFeed feed;

    [Tooltip("feed.LatestRolePoses 是否是 PICO local 坐标。如果是，则使用 trackingOrigin.TransformPoint 转世界。")]
    public bool sourceIsLocalSpace = true;

    [Tooltip("PICO 追踪坐标系在 Unity 中的参考根，通常等于 feed.cubesRoot（或你自己的 TrackingOrigin）。")]
    public Transform trackingOrigin;

    [Header("Pose Adapter (optional)")]
    [Tooltip("围绕 Y 轴的全局偏置（度）。常用 180 可解决前后反。")]
    public float yawOffsetDeg = 0f;
    public bool flipX = false;
    public bool flipY = false;
    public bool flipZ = false;     // 膝/肘反向常见修正：先试 true
    public bool swapXZ = false;    // 坐标系 Z-up 等情形可尝试
    public bool swapLeftRight = false;

    [Header("Gizmos")]
    public bool drawGizmos = true;
    public float gizmoPointSize = 0.025f;
    public Color gizmoPointColor = new Color(0.1f, 0.8f, 1f, 1f);
    public Color gizmoBoneColor = new Color(1f, 0.9f, 0.2f, 1f);

    [Header("HMD Camera (for recording view direction)")]
    [Tooltip("XR Origin 下的 Main Camera（Camera Offset/Main Camera），用于录制 HMD 6DOF 视角。")]
    public Transform hmdCamera;

    [Header("Room Space (for recording in room coordinates)")]
    [Tooltip("Active scene child transform. If set, CSV positions are recorded in room local space instead of world space.\n" +
             "Room coords: x=width(0..W), y=length(0..L), z=height. Matches generator JSON coordinate system.")]
    public Transform roomSpaceRef;

    [Header("Recording (optional)")]
    [Tooltip("是否允许录制（UI 面板通常会在 BeginRecording() 时置 true）。")]
    public bool recordCsv = false;

    [Tooltip("输出文件名基底（自动加时间戳）。")]
    public string fileBaseName = "pxr_24pts_world";

    [Tooltip("每 N 帧写一次。")]
    public int writeEveryNFrames = 1;

    [Header("Embodiment Scale (optional)")]
    [Tooltip("将输出位置按指定 embodiment 比例缩放（以 trackingOrigin 为参考系做局部缩放）。")]
    public bool applyEmbodimentScale = true;

    [Header("Embodiment Export (scale positions before writing CSV)")]
    [Tooltip("如果为 true，会在写 CSV 时再围绕 pivot 做一次缩放。一般不建议与 applyEmbodimentScale 同时开启（会缩放两次）。")]
    public bool exportEmbodimentScaledPositions = false; // <-- default safer

    [SerializeField] private float embodimentScale = 1f;
    private Vector3 embodimentPivotWorld = Vector3.zero;
    private bool hasEmbodimentPivot = false;

    public void SetEmbodimentScale(float scale)
    {
        embodimentScale = Mathf.Max(1e-6f, scale);
    }

    public void SetEmbodimentPivotWorld(Vector3 pivotWorld)
    {
        embodimentPivotWorld = pivotWorld;
        hasEmbodimentPivot = true;
    }

    [Header("Raw TXT (PICO local, one-shot)")]
    public bool recordRawTxt = false;
    public string rawTxtBaseName = "pxr_24pts_raw_local";
    private bool _rawTxtDumped = false;

    [Serializable] public class PointsUpdatedEvent : UnityEvent<Vector3[]> { }
    [Header("Events")]
    public PointsUpdatedEvent OnPointsUpdated;

    // 输出：世界坐标 24 点（注意：若 applyEmbodimentScale=true 且已设置 scale，则这里存的是“缩放后”的世界点）
    public Vector3[] WorldJoints24 { get; private set; } = new Vector3[24];

    // 额外：保存对应 24 点的 local 旋转（不做适配，原始 PICO local）
    public Quaternion[] LocalRotations24 { get; private set; } = new Quaternion[24];

    private StreamWriter _writer;
    private int _frameCounter;
    private string _filePath;
    private bool _writerOpened = false;
    private float _recordStartRealtime;   // Time.realtimeSinceStartup at recording start
    private string _recordingPrefix = "";  // e.g. "disaster_easy_3x4_000_corner0"
    private int _recordingSeqNum = 0;      // auto-increment per prefix

    // 左右对调的索引（按你的 PositionIndex 定义）
    private static readonly int[] L = {
        (int)PositionIndex.left_hip, (int)PositionIndex.left_knee, (int)PositionIndex.left_ankle,
        (int)PositionIndex.left_foot_index, (int)PositionIndex.left_collar, (int)PositionIndex.left_shoulder,
        (int)PositionIndex.left_elbow, (int)PositionIndex.left_wrist, (int)PositionIndex.left_hand
    };
    private static readonly int[] R = {
        (int)PositionIndex.right_hip, (int)PositionIndex.right_knee, (int)PositionIndex.right_ankle,
        (int)PositionIndex.right_foot_index, (int)PositionIndex.right_collar, (int)PositionIndex.right_shoulder,
        (int)PositionIndex.right_elbow, (int)PositionIndex.right_wrist, (int)PositionIndex.right_hand
    };

    // -------------------- Public Control API (for UI) --------------------

    /// <summary>
    /// 设置录制文件名前缀。调用后文件名格式为: {prefix}_{序号:02d}.csv
    /// 例如: disaster_easy_3x4_000_corner0_00.csv
    /// 每次 BeginRecording 自动递增序号。
    /// </summary>
    public void SetRecordingPrefix(string prefix)
    {
        if (_recordingPrefix != prefix)
        {
            _recordingPrefix = prefix ?? "";
            _recordingSeqNum = 0;  // 新前缀重置序号
        }
    }

    public void BeginRecording()
    {
        recordCsv = true;
        OpenWriterIfNeeded();
    }

    public void EndRecording()
    {
        recordCsv = false;
        CloseWriter();
    }

    // -------------------- Unity Lifecycle --------------------
    void Start()
    {
        // ✅ 不再在 Start() 自动打开 writer
    }

    void OnDestroy()
    {
        CloseWriter();
    }

    // Android 上直接退出 app 时 OnDestroy 可能不被调用，用 OnApplicationPause 兜底
    void OnApplicationPause(bool pauseStatus)
    {
        if (pauseStatus && _writerOpened)
        {
            try { _writer?.Flush(); } catch { }
        }
    }

    void OnApplicationQuit()
    {
        CloseWriter();
    }

    void Update()
    {
#if UNITY_ANDROID
        if (feed == null)
        {
            if (recordCsv && Time.frameCount % 300 == 0)
                Debug.LogWarning("[24PointMapper] feed is null — no data to record!");
            return;
        }
        var poses = feed.LatestRolePoses;
        if (poses == null || poses.Length < 24)
        {
            if (recordCsv && Time.frameCount % 300 == 0)
                Debug.LogWarning($"[24PointMapper] poses invalid (null={poses == null}, len={poses?.Length ?? 0}) — waiting for body tracking data...");
            return;
        }

        if (recordRawTxt && !_rawTxtDumped)
        {
            TryDumpRawTxt(poses);
        }

        // 1) 取 24 点并转世界；同时记录对应 local 旋转（不做适配）
        for (int i = 0; i < 24; i++)
        {
            Vector3 p = poses[i].valid ? poses[i].pos : Vector3.zero;
            Quaternion qLocal = poses[i].valid ? poses[i].rot : Quaternion.identity;

            if (sourceIsLocalSpace && trackingOrigin != null)
                p = trackingOrigin.TransformPoint(p);

            p = AdaptPoint(p);

            WorldJoints24[i] = p;
            LocalRotations24[i] = qLocal;
        }

        // 2) 左右对调（可选）——这里只对点做 swap，不改 quaternion
        if (swapLeftRight)
        {
            for (int i = 0; i < L.Length; i++)
            {
                int li = L[i], ri = R[i];
                (WorldJoints24[li], WorldJoints24[ri]) = (WorldJoints24[ri], WorldJoints24[li]);
            }
        }

        // 3) embodiment scale（只对 position；以 trackingOrigin 为参考做局部缩放）
        if (applyEmbodimentScale)
        {
            ApplyEmbodimentScaleInPlace(WorldJoints24);
        }

        // 4) 事件回调
        OnPointsUpdated?.Invoke(WorldJoints24);

        // 5) 记录 CSV（position + local quaternion）
        if (recordCsv && (++_frameCounter % writeEveryNFrames == 0))
        {
            OpenWriterIfNeeded();
            WriteCsv(WorldJoints24, LocalRotations24);
        }
#endif
    }

    // ----------------- Writer helpers -----------------
    private void OpenWriterIfNeeded()
    {
        if (_writerOpened) return;
        if (!recordCsv) return;

        try
        {
            // 文件命名：如果有 prefix（由 AutoFlow 设置），用 prefix_序号 格式
            // 否则退回默认的 fileBaseName_时间戳 格式
            string fileName;
            if (!string.IsNullOrEmpty(_recordingPrefix))
            {
                // 自动查找下一个可用序号，避免覆盖
                while (true)
                {
                    fileName = $"{_recordingPrefix}_{_recordingSeqNum:D2}.csv";
                    string testPath = Path.Combine(Application.persistentDataPath, fileName);
                    if (!File.Exists(testPath)) break;
                    _recordingSeqNum++;
                }
                _recordingSeqNum++; // 下次录制用下一个序号
            }
            else
            {
                string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                fileName = $"{fileBaseName}_{ts}.csv";
            }

            _filePath = Path.Combine(Application.persistentDataPath, fileName);
            _writer = new StreamWriter(_filePath, false, System.Text.Encoding.UTF8);
            _recordStartRealtime = Time.realtimeSinceStartup;

            // ---------- META lines (method-3 replay) ----------
            var inv = CultureInfo.InvariantCulture;

            _writer.WriteLine("# pxr24_csv_v2");
            _writer.WriteLine("# units=m");
            _writer.WriteLine("# time=realtimeSinceStartup_relative_seconds");
            _writer.WriteLine("# saved_at=" + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff", inv));
            _writer.WriteLine("# embodiment_scale=" + embodimentScale.ToString("F6", inv));
            _writer.WriteLine("# yaw_offset_deg=" + yawOffsetDeg.ToString("F3", inv));

            Vector3 oPos = trackingOrigin ? trackingOrigin.position : Vector3.zero;
            Quaternion oRot = trackingOrigin ? trackingOrigin.rotation : Quaternion.identity;

            _writer.WriteLine(
                $"# record_origin_pos={oPos.x.ToString("F6", inv)},{oPos.y.ToString("F6", inv)},{oPos.z.ToString("F6", inv)}"
            );
            _writer.WriteLine(
                $"# record_origin_rot_xyzw={oRot.x.ToString("F6", inv)},{oRot.y.ToString("F6", inv)},{oRot.z.ToString("F6", inv)},{oRot.w.ToString("F6", inv)}"
            );
            _writer.WriteLine("# hmd_camera=" + (hmdCamera != null ? hmdCamera.name : "null"));
            _writer.WriteLine("# coordinate_space=" + (roomSpaceRef != null ? "room_local" : "world"));
            if (roomSpaceRef != null)
                _writer.WriteLine("# room_space_ref=" + roomSpaceRef.name);
            if (!string.IsNullOrEmpty(_recordingPrefix))
                _writer.WriteLine("# scene_prefix=" + _recordingPrefix);
            // ---------- END META ----------

            // header: time + camera 6DOF + 24 joints (position + rotation)
            _writer.Write("frame,time");
            _writer.Write(",cam_px,cam_py,cam_pz,cam_qw,cam_qx,cam_qy,cam_qz");
            for (int i = 0; i < 24; i++)
            {
                _writer.Write($",p{i}x,p{i}y,p{i}z,q{i}w,q{i}x,q{i}y,q{i}z");
            }
            _writer.WriteLine();
            _writer.Flush();

            _writerOpened = true;
            Debug.Log($"[24PointMapper] Recording to {_filePath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[24PointMapper] Cannot open writer: {e}");
            recordCsv = false;
            _writerOpened = false;
        }
    }

    private void CloseWriter()
    {
        try { _writer?.Flush(); _writer?.Close(); } catch { }
        _writer = null;
        _writerOpened = false;
    }

    // ----------------- Raw dump -----------------
    private void TryDumpRawTxt(PXR_BodyTrackingFeed.RolePose[] poses)
    {
        try
        {
            string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string rawPath = Path.Combine(Application.persistentDataPath, $"{rawTxtBaseName}_{ts}.txt");
            using (var sw = new StreamWriter(rawPath, false, System.Text.Encoding.UTF8))
            {
                sw.WriteLine("# PICO raw poses dump (LOCAL coordinates, NO transforms applied)");
                sw.WriteLine("# Saved at: " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"));
                sw.WriteLine("# persistentDataPath: " + Application.persistentDataPath);
                sw.WriteLine("# Format per line: index, valid, px, py, pz, qw, qx, qy, qz");
                sw.WriteLine("# NOTE: quaternion components taken directly from Unity Quaternion (x,y,z,w),");
                sw.WriteLine("#       only output order is rearranged to w,x,y,z for MuJoCo-style conventions.");
                sw.WriteLine();

                for (int i = 0; i < 24; i++)
                {
                    bool valid = poses[i].valid;
                    Vector3 v = poses[i].pos;
                    Quaternion q = poses[i].rot;

                    sw.WriteLine(
                        $"{i}, {valid}, " +
                        $"{v.x:F6}, {v.y:F6}, {v.z:F6}, " +
                        $"{q.w:F6}, {q.x:F6}, {q.y:F6}, {q.z:F6}"
                    );
                }
            }
            _rawTxtDumped = true;
            Debug.Log($"[24PointMapper] Raw PICO local poses (pos+rot) saved (one-shot) to: {rawPath}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[24PointMapper] Dump raw TXT failed: {e}");
        }
    }

    // ----------------- Embodiment scaling (position only) -----------------
    private void ApplyEmbodimentScaleInPlace(Vector3[] ptsWorld)
    {
        if (ptsWorld == null) return;
        if (Mathf.Abs(embodimentScale - 1f) < 1e-6f) return;

        if (trackingOrigin != null)
        {
            for (int i = 0; i < ptsWorld.Length; i++)
            {
                Vector3 local = trackingOrigin.InverseTransformPoint(ptsWorld[i]);
                local *= embodimentScale;
                ptsWorld[i] = trackingOrigin.TransformPoint(local);
            }
        }
        else
        {
            for (int i = 0; i < ptsWorld.Length; i++)
                ptsWorld[i] *= embodimentScale;
        }
    }

    // ----------------- 辅助：姿态适配（仅 position） -----------------
    private Vector3 AdaptPoint(Vector3 pWorld)
    {
        Transform T = trackingOrigin != null ? trackingOrigin : null;
        Vector3 local = T ? T.InverseTransformPoint(pWorld) : pWorld;

        if (swapXZ) local = new Vector3(local.z, local.y, local.x);

        local = new Vector3(
            flipX ? -local.x : local.x,
            flipY ? -local.y : local.y,
            flipZ ? -local.z : local.z
        );

        Vector3 w = T ? T.TransformPoint(local) : local;

        if (Mathf.Abs(yawOffsetDeg) > 0.001f)
        {
            Quaternion yaw = Quaternion.AngleAxis(yawOffsetDeg, Vector3.up);
            if (T)
            {
                Vector3 rel = w - T.position;
                w = T.position + yaw * rel;
            }
            else
            {
                w = yaw * w;
            }
        }
        return w;
    }

    private void WriteCsv(Vector3[] pts, Quaternion[] quatsLocal)
    {
        if (_writer == null) return;

        try
        {
            float t = Time.realtimeSinceStartup - _recordStartRealtime;
            _writer.Write($"{Time.frameCount},{t:F6}");

            // HMD camera 6DOF
            Vector3 camP = hmdCamera ? hmdCamera.position : Vector3.zero;
            Quaternion camQ = hmdCamera ? hmdCamera.rotation : Quaternion.identity;
            // Convert camera to room space if available
            if (roomSpaceRef != null)
                camP = roomSpaceRef.InverseTransformPoint(camP);
            _writer.Write($",{camP.x:F6},{camP.y:F6},{camP.z:F6},{camQ.w:F6},{camQ.x:F6},{camQ.y:F6},{camQ.z:F6}");

            for (int i = 0; i < 24; i++)
            {
                Vector3 p = pts[i];

                // 导出时缩放：基于 trackingOrigin 做局部缩放（与 ApplyEmbodimentScaleInPlace 一致）
                if (exportEmbodimentScaledPositions && Mathf.Abs(embodimentScale - 1f) > 1e-6f)
                {
                    if (trackingOrigin != null)
                    {
                        Vector3 local = trackingOrigin.InverseTransformPoint(p);
                        local *= embodimentScale;
                        p = trackingOrigin.TransformPoint(local);
                    }
                    else
                    {
                        p *= embodimentScale;
                    }
                }

                // Convert to room local coordinates (matches generator JSON coordinate system)
                // Room coords: x = width (0..W), y = length (0..L), z = height
                if (roomSpaceRef != null)
                    p = roomSpaceRef.InverseTransformPoint(p);

                Quaternion q = (quatsLocal != null && i < quatsLocal.Length) ? quatsLocal[i] : Quaternion.identity;

                _writer.Write(
                    $",{p.x:F6},{p.y:F6},{p.z:F6}," +
                    $"{q.w:F6},{q.x:F6},{q.y:F6},{q.z:F6}"
                );
            }
            _writer.WriteLine();
            _writer.Flush();
        }
        catch (Exception e)
        {
            Debug.LogError($"[24PointMapper] WriteCsv failed: {e}");
            recordCsv = false;
            CloseWriter();
        }
    }

    // ----------------- Gizmos 可视化 -----------------
    void OnDrawGizmos()
    {
        if (!drawGizmos || WorldJoints24 == null || WorldJoints24.Length < 24) return;

        Gizmos.color = gizmoPointColor;
        foreach (var p in WorldJoints24)
            Gizmos.DrawSphere(p, gizmoPointSize);

        Gizmos.color = gizmoBoneColor;
        // 脊柱
        DrawBone(PositionIndex.hip, PositionIndex.spine1);
        DrawBone(PositionIndex.spine1, PositionIndex.spine2);
        DrawBone(PositionIndex.spine2, PositionIndex.spine3);
        DrawBone(PositionIndex.spine3, PositionIndex.neck);
        DrawBone(PositionIndex.neck, PositionIndex.head);

        // 左腿（hip → left_hip 连接骨盆）
        DrawBone(PositionIndex.hip, PositionIndex.left_hip);
        DrawBone(PositionIndex.left_hip, PositionIndex.left_knee);
        DrawBone(PositionIndex.left_knee, PositionIndex.left_ankle);
        DrawBone(PositionIndex.left_ankle, PositionIndex.left_foot_index);

        // 右腿（hip → right_hip 连接骨盆）
        DrawBone(PositionIndex.hip, PositionIndex.right_hip);
        DrawBone(PositionIndex.right_hip, PositionIndex.right_knee);
        DrawBone(PositionIndex.right_knee, PositionIndex.right_ankle);
        DrawBone(PositionIndex.right_ankle, PositionIndex.right_foot_index);

        // 左臂（spine3 → left_collar 连接躯干）
        DrawBone(PositionIndex.spine3, PositionIndex.left_collar);
        DrawBone(PositionIndex.left_collar, PositionIndex.left_shoulder);
        DrawBone(PositionIndex.left_shoulder, PositionIndex.left_elbow);
        DrawBone(PositionIndex.left_elbow, PositionIndex.left_wrist);
        DrawBone(PositionIndex.left_wrist, PositionIndex.left_hand);

        // 右臂（spine3 → right_collar 连接躯干）
        DrawBone(PositionIndex.spine3, PositionIndex.right_collar);
        DrawBone(PositionIndex.right_collar, PositionIndex.right_shoulder);
        DrawBone(PositionIndex.right_shoulder, PositionIndex.right_elbow);
        DrawBone(PositionIndex.right_elbow, PositionIndex.right_wrist);
        DrawBone(PositionIndex.right_wrist, PositionIndex.right_hand);
    }

    private void DrawBone(PositionIndex a, PositionIndex b)
    {
        int ia = (int)a, ib = (int)b;
        if (WorldJoints24 == null || ia >= WorldJoints24.Length || ib >= WorldJoints24.Length) return;
        Vector3 pa = WorldJoints24[ia], pb = WorldJoints24[ib];
        Gizmos.DrawLine(pa, pb);
    }
}


//--------------------------------------------
//using System;
//using System.IO;
//using UnityEngine;
//using UnityEngine.Events;

///// <summary>
///// 仅做 24 点的坐标转换与可选适配：
///// - 从 PXR_BodyTrackingFeed 取 24 点
///// - 可将 PICO local 转为 Unity world（trackingOrigin.TransformPoint）
///// - 可做 yaw 偏置 / 轴翻转 / XZ 交换 / 左右对调
///// - 输出 WorldJoints24（Vector3[24]）
///// - 可视化与可选 CSV 记录
///// - 额外：支持导出 position + rotation（四元数，以 wxyz 顺序写入文件）
///// 不涉及 SMPLX 驱动。
/////
///// ✅ 2026-01 版本改动：
///// - 增加 SetEmbodimentScale / BeginRecording / EndRecording 接口，便于 UI 面板控制
///// - CSV Writer 生命周期从 Start() 移到 BeginRecording()
///// - 支持对 position 做 embodiment 尺度缩放（以 trackingOrigin 为参考）
///// </summary>
//public class PXR_24PointMapper : MonoBehaviour
//{
//    [Header("Source")]
//    public PXR_BodyTrackingFeed feed;

//    [Tooltip("feed.LatestRolePoses 是否是 PICO local 坐标。如果是，则使用 trackingOrigin.TransformPoint 转世界。")]
//    public bool sourceIsLocalSpace = true;

//    [Tooltip("PICO 追踪坐标系在 Unity 中的参考根，通常等于 feed.cubesRoot（或你自己的 TrackingOrigin）。")]
//    public Transform trackingOrigin;

//    [Header("Pose Adapter (optional)")]
//    [Tooltip("围绕 Y 轴的全局偏置（度）。常用 180 可解决前后反。")]
//    public float yawOffsetDeg = 0f;
//    public bool flipX = false;
//    public bool flipY = false;
//    public bool flipZ = false;     // 膝/肘反向常见修正：先试 true
//    public bool swapXZ = false;    // 坐标系 Z-up 等情形可尝试
//    public bool swapLeftRight = false;

//    [Header("Gizmos")]
//    public bool drawGizmos = true;
//    public float gizmoPointSize = 0.025f;
//    public Color gizmoPointColor = new Color(0.1f, 0.8f, 1f, 1f);
//    public Color gizmoBoneColor = new Color(1f, 0.9f, 0.2f, 1f);

//    [Header("Recording (optional)")]
//    [Tooltip("是否允许录制（UI 面板通常会在 BeginRecording() 时置 true）。")]
//    public bool recordCsv = false;

//    [Tooltip("输出文件名基底（自动加时间戳）。")]
//    public string fileBaseName = "pxr_24pts_world";

//    [Tooltip("每 N 帧写一次。")]
//    public int writeEveryNFrames = 1;

//    [Header("Embodiment Scale (optional)")]
//    [Tooltip("将输出位置按指定 embodiment 比例缩放（以 trackingOrigin 为参考系做局部缩放）。")]
//    public bool applyEmbodimentScale = true;

//    //[Tooltip("默认为 1.0；UI 标定后调用 SetEmbodimentScale(scale)。")]
//    //[SerializeField] private float embodimentScale = 1.0f;

//    [Header("Embodiment Export (scale positions before writing CSV)")]
//    public bool exportEmbodimentScaledPositions = true;

//    [SerializeField] private float embodimentScale = 1f;
//    private Vector3 embodimentPivotWorld = Vector3.zero;
//    private bool hasEmbodimentPivot = false;

//    public void SetEmbodimentScale(float scale)
//    {
//        embodimentScale = Mathf.Max(1e-6f, scale);
//    }

//    public void SetEmbodimentPivotWorld(Vector3 pivotWorld)
//    {
//        embodimentPivotWorld = pivotWorld;
//        hasEmbodimentPivot = true;
//    }


//    [Header("Raw TXT (PICO local, one-shot)")]
//    public bool recordRawTxt = false;
//    public string rawTxtBaseName = "pxr_24pts_raw_local";
//    private bool _rawTxtDumped = false;

//    [Serializable] public class PointsUpdatedEvent : UnityEvent<Vector3[]> { }
//    [Header("Events")]
//    public PointsUpdatedEvent OnPointsUpdated;

//    // 输出：世界坐标 24 点（注意：若 applyEmbodimentScale=true 且已设置 scale，则这里存的是“缩放后”的世界点）
//    public Vector3[] WorldJoints24 { get; private set; } = new Vector3[24];

//    // 额外：保存对应 24 点的 local 旋转（不做适配，原始 PICO local）
//    public Quaternion[] LocalRotations24 { get; private set; } = new Quaternion[24];

//    private StreamWriter _writer;
//    private int _frameCounter;
//    private string _filePath;
//    private bool _writerOpened = false;

//    // 左右对调的索引（按你的 PositionIndex 定义）
//    private static readonly int[] L = {
//        (int)PositionIndex.left_hip, (int)PositionIndex.left_knee, (int)PositionIndex.left_ankle,
//        (int)PositionIndex.left_foot_index, (int)PositionIndex.left_collar, (int)PositionIndex.left_shoulder,
//        (int)PositionIndex.left_elbow, (int)PositionIndex.left_wrist, (int)PositionIndex.left_hand
//    };
//    private static readonly int[] R = {
//        (int)PositionIndex.right_hip, (int)PositionIndex.right_knee, (int)PositionIndex.right_ankle,
//        (int)PositionIndex.right_foot_index, (int)PositionIndex.right_collar, (int)PositionIndex.right_shoulder,
//        (int)PositionIndex.right_elbow, (int)PositionIndex.right_wrist, (int)PositionIndex.right_hand
//    };

//    // -------------------- Public Control API (for UI) --------------------
//    /// <summary>
//    /// 设置 embodiment 缩放比例（例如 G1：targetHeight / measuredHumanHeight）。
//    /// </summary>

//    /// <summary>
//    /// 开始录制（打开 CSV writer，写 header）。
//    /// </summary>
//    public void BeginRecording()
//    {
//        recordCsv = true;
//        OpenWriterIfNeeded();
//    }

//    /// <summary>
//    /// 停止录制（关闭 writer）。
//    /// </summary>
//    public void EndRecording()
//    {
//        recordCsv = false;
//        CloseWriter();
//    }

//    // -------------------- Unity Lifecycle --------------------
//    void Start()
//    {
//        // ✅ 不再在 Start() 自动打开 writer
//        // 由 UI 或外部脚本调用 BeginRecording() 控制
//    }

//    void OnDestroy()
//    {
//        CloseWriter();
//    }

//    void Update()
//    {
//#if UNITY_ANDROID
//        if (feed == null) return;
//        var poses = feed.LatestRolePoses;
//        if (poses == null || poses.Length < 24) return;

//        // ---- 一次性导出 PICO 原始 local 数据为 TXT（仅首帧成功获取时） ----
//        if (recordRawTxt && !_rawTxtDumped)
//        {
//            TryDumpRawTxt(poses);
//        }

//        // 1) 取 24 点并转世界；同时记录对应 local 旋转（不做适配）
//        for (int i = 0; i < 24; i++)
//        {
//            Vector3 p = poses[i].valid ? poses[i].pos : Vector3.zero;
//            Quaternion qLocal = poses[i].valid ? poses[i].rot : Quaternion.identity;

//            // local -> world 只对 position 处理
//            if (sourceIsLocalSpace && trackingOrigin != null)
//                p = trackingOrigin.TransformPoint(p);

//            // 姿态适配（只对 position 做 swap/flip/yaw）
//            p = AdaptPoint(p);

//            WorldJoints24[i] = p;
//            LocalRotations24[i] = qLocal;  // 保留原始 local quaternion，不做额外变换
//        }

//        // 2) 左右对调（可选）——这里只对点做 swap，不改 quaternion
//        if (swapLeftRight)
//        {
//            for (int i = 0; i < L.Length; i++)
//            {
//                int li = L[i], ri = R[i];
//                (WorldJoints24[li], WorldJoints24[ri]) = (WorldJoints24[ri], WorldJoints24[li]);
//            }
//        }

//        // 3) embodiment scale（只对 position；以 trackingOrigin 为参考做局部缩放）
//        if (applyEmbodimentScale)
//        {
//            ApplyEmbodimentScaleInPlace(WorldJoints24);
//        }

//        // 4) 事件回调（回调到的是最终点：已适配 +（可选）已缩放）
//        OnPointsUpdated?.Invoke(WorldJoints24);

//        // 5) 记录 CSV（position + local quaternion）
//        if (recordCsv && (++_frameCounter % writeEveryNFrames == 0))
//        {
//            OpenWriterIfNeeded();
//            WriteCsv(WorldJoints24, LocalRotations24);
//        }
//#endif
//    }

//    // ----------------- Writer helpers -----------------
//    private void OpenWriterIfNeeded()
//    {
//        if (_writerOpened) return;
//        if (!recordCsv) return;

//        try
//        {
//            string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
//            _filePath = Path.Combine(Application.persistentDataPath, $"{fileBaseName}_{ts}.csv");
//            _writer = new StreamWriter(_filePath, false, System.Text.Encoding.UTF8);

//            // header：包含 position + rotation（四元数以 wxyz 顺序写出）
//            _writer.Write("frame,time");
//            for (int i = 0; i < 24; i++)
//            {
//                _writer.Write($",p{i}x,p{i}y,p{i}z,q{i}w,q{i}x,q{i}y,q{i}z");
//            }
//            _writer.WriteLine();
//            _writer.Flush();

//            _writerOpened = true;
//            Debug.Log($"[24PointMapper] Recording to {_filePath}");
//        }
//        catch (Exception e)
//        {
//            Debug.LogError($"[24PointMapper] Cannot open writer: {e}");
//            recordCsv = false;
//            _writerOpened = false;
//        }
//    }

//    private void CloseWriter()
//    {
//        try { _writer?.Flush(); _writer?.Close(); } catch { }
//        _writer = null;
//        _writerOpened = false;
//    }

//    // ----------------- Raw dump -----------------
//    private void TryDumpRawTxt(PXR_BodyTrackingFeed.RolePose[] poses)
//    {
//        try
//        {
//            string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
//            string rawPath = Path.Combine(Application.persistentDataPath, $"{rawTxtBaseName}_{ts}.txt");
//            using (var sw = new StreamWriter(rawPath, false, System.Text.Encoding.UTF8))
//            {
//                sw.WriteLine("# PICO raw poses dump (LOCAL coordinates, NO transforms applied)");
//                sw.WriteLine("# Saved at: " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"));
//                sw.WriteLine("# persistentDataPath: " + Application.persistentDataPath);
//                sw.WriteLine("# Format per line: index, valid, px, py, pz, qw, qx, qy, qz");
//                sw.WriteLine("# NOTE: quaternion components taken directly from Unity Quaternion (x,y,z,w),");
//                sw.WriteLine("#       only output order is rearranged to w,x,y,z for MuJoCo-style conventions.");
//                sw.WriteLine();

//                for (int i = 0; i < 24; i++)
//                {
//                    bool valid = poses[i].valid;
//                    Vector3 v = poses[i].pos;     // 原始 PICO local 坐标
//                    Quaternion q = poses[i].rot;  // 原始 PICO local 旋转（Unity Quaternion）

//                    sw.WriteLine(
//                        $"{i}, {valid}, " +
//                        $"{v.x:F6}, {v.y:F6}, {v.z:F6}, " +
//                        $"{q.w:F6}, {q.x:F6}, {q.y:F6}, {q.z:F6}"
//                    );
//                }
//            }
//            _rawTxtDumped = true;
//            Debug.Log($"[24PointMapper] Raw PICO local poses (pos+rot) saved (one-shot) to: {rawPath}");
//        }
//        catch (Exception e)
//        {
//            Debug.LogError($"[24PointMapper] Dump raw TXT failed: {e}");
//        }
//    }

//    // ----------------- Embodiment scaling (position only) -----------------
//    private void ApplyEmbodimentScaleInPlace(Vector3[] ptsWorld)
//    {
//        if (ptsWorld == null) return;
//        if (Mathf.Abs(embodimentScale - 1f) < 1e-6f) return;

//        // 以 trackingOrigin 为参考系做局部缩放最稳
//        if (trackingOrigin != null)
//        {
//            for (int i = 0; i < ptsWorld.Length; i++)
//            {
//                Vector3 local = trackingOrigin.InverseTransformPoint(ptsWorld[i]);
//                local *= embodimentScale;
//                ptsWorld[i] = trackingOrigin.TransformPoint(local);
//            }
//        }
//        else
//        {
//            // 没有 trackingOrigin 时只能在 world 原点缩放（通常不推荐）
//            for (int i = 0; i < ptsWorld.Length; i++)
//                ptsWorld[i] *= embodimentScale;
//        }
//    }

//    // ----------------- 辅助：姿态适配（仅 position） -----------------
//    private Vector3 AdaptPoint(Vector3 pWorld)
//    {
//        Transform T = trackingOrigin != null ? trackingOrigin : null;
//        Vector3 local = T ? T.InverseTransformPoint(pWorld) : pWorld;

//        if (swapXZ) local = new Vector3(local.z, local.y, local.x);

//        local = new Vector3(
//            flipX ? -local.x : local.x,
//            flipY ? -local.y : local.y,
//            flipZ ? -local.z : local.z
//        );

//        Vector3 w = T ? T.TransformPoint(local) : local;

//        if (Mathf.Abs(yawOffsetDeg) > 0.001f)
//        {
//            Quaternion yaw = Quaternion.AngleAxis(yawOffsetDeg, Vector3.up);
//            if (T)
//            {
//                Vector3 rel = w - T.position;
//                w = T.position + yaw * rel;
//            }
//            else
//            {
//                w = yaw * w;
//            }
//        }
//        return w;
//    }

//    private void WriteCsv(Vector3[] pts, Quaternion[] quatsLocal)
//    {
//        if (_writer == null) return;

//        try
//        {
//            _writer.Write($"{Time.frameCount},{Time.time:F6}");
//            for (int i = 0; i < 24; i++)
//            {
//                Vector3 p = pts[i];

//                if (exportEmbodimentScaledPositions && Mathf.Abs(embodimentScale - 1f) > 1e-6f)
//                {
//                    Vector3 pivot = hasEmbodimentPivot ? embodimentPivotWorld : (trackingOrigin ? trackingOrigin.position : Vector3.zero);
//                    p = pivot + (p - pivot) * embodimentScale;
//                }

//                Quaternion q = (quatsLocal != null && i < quatsLocal.Length) ? quatsLocal[i] : Quaternion.identity;

//                // position: world space after适配 +（可选）embodiment scale
//                // quaternion: 原始 local PICO Quaternion，只按 wxyz 顺序输出，不做额外处理
//                _writer.Write(
//                    $",{p.x:F6},{p.y:F6},{p.z:F6}," +
//                    $"{q.w:F6},{q.x:F6},{q.y:F6},{q.z:F6}"
//                );
//            }
//            _writer.WriteLine();
//            _writer.Flush();
//        }
//        catch (Exception e)
//        {
//            Debug.LogError($"[24PointMapper] WriteCsv failed: {e}");
//            recordCsv = false;
//            CloseWriter();
//        }
//    }

//    // ----------------- Gizmos 可视化 -----------------
//    void OnDrawGizmos()
//    {
//        if (!drawGizmos || WorldJoints24 == null || WorldJoints24.Length < 24) return;

//        Gizmos.color = gizmoPointColor;
//        foreach (var p in WorldJoints24)
//            Gizmos.DrawSphere(p, gizmoPointSize);

//        Gizmos.color = gizmoBoneColor;
//        DrawBone(PositionIndex.hip, PositionIndex.spine1);
//        DrawBone(PositionIndex.spine1, PositionIndex.spine2);
//        DrawBone(PositionIndex.spine2, PositionIndex.spine3);
//        DrawBone(PositionIndex.spine3, PositionIndex.neck);
//        DrawBone(PositionIndex.neck, PositionIndex.head);

//        DrawBone(PositionIndex.left_hip, PositionIndex.left_knee);
//        DrawBone(PositionIndex.left_knee, PositionIndex.left_ankle);
//        DrawBone(PositionIndex.left_ankle, PositionIndex.left_foot_index);

//        DrawBone(PositionIndex.right_hip, PositionIndex.right_knee);
//        DrawBone(PositionIndex.right_knee, PositionIndex.right_ankle);
//        DrawBone(PositionIndex.right_ankle, PositionIndex.right_foot_index);

//        DrawBone(PositionIndex.left_collar, PositionIndex.left_shoulder);
//        DrawBone(PositionIndex.left_shoulder, PositionIndex.left_elbow);
//        DrawBone(PositionIndex.left_elbow, PositionIndex.left_wrist);
//        DrawBone(PositionIndex.left_wrist, PositionIndex.left_hand);

//        DrawBone(PositionIndex.right_collar, PositionIndex.right_shoulder);
//        DrawBone(PositionIndex.right_shoulder, PositionIndex.right_elbow);
//        DrawBone(PositionIndex.right_elbow, PositionIndex.right_wrist);
//        DrawBone(PositionIndex.right_wrist, PositionIndex.right_hand);
//    }

//    private void DrawBone(PositionIndex a, PositionIndex b)
//    {
//        int ia = (int)a, ib = (int)b;
//        if (WorldJoints24 == null || ia >= WorldJoints24.Length || ib >= WorldJoints24.Length) return;
//        Vector3 pa = WorldJoints24[ia], pb = WorldJoints24[ib];
//        Gizmos.DrawLine(pa, pb);
//    }
//}

//------------------------------------------------------------------
//using System;
//using System.IO;
//using UnityEngine;
//using UnityEngine.Events;

///// <summary>
///// 仅做 24 点的坐标转换与可选适配：
///// - 从 PXR_BodyTrackingFeed 取 24 点
///// - 可将 PICO local 转为 Unity world（trackingOrigin.TransformPoint）
///// - 可做 yaw 偏置 / 轴翻转 / XZ 交换 / 左右对调
///// - 输出 WorldJoints24（Vector3[24]）
///// - 可视化与可选 CSV 记录
///// - 额外：支持导出 position + rotation（四元数，以 wxyz 顺序写入文件）
///// 不涉及 SMPLX 驱动。
///// </summary>
//public class PXR_24PointMapper : MonoBehaviour
//{
//    [Header("Source")]
//    public PXR_BodyTrackingFeed feed;

//    [Tooltip("feed.LatestRolePoses 是否是 PICO local 坐标。如果是，则使用 trackingOrigin.TransformPoint 转世界。")]
//    public bool sourceIsLocalSpace = true;

//    [Tooltip("PICO 追踪坐标系在 Unity 中的参考根，通常等于 feed.cubesRoot（或你自己的 TrackingOrigin）。")]
//    public Transform trackingOrigin;

//    [Header("Pose Adapter (optional)")]
//    [Tooltip("围绕 Y 轴的全局偏置（度）。常用 180 可解决前后反。")]
//    public float yawOffsetDeg = 0f;
//    public bool flipX = false;
//    public bool flipY = false;
//    public bool flipZ = false;     // 膝/肘反向常见修正：先试 true
//    public bool swapXZ = false;    // 坐标系 Z-up 等情形可尝试
//    public bool swapLeftRight = false;

//    [Header("Gizmos")]
//    public bool drawGizmos = true;
//    public float gizmoPointSize = 0.025f;
//    public Color gizmoPointColor = new Color(0.1f, 0.8f, 1f, 1f);
//    public Color gizmoBoneColor = new Color(1f, 0.9f, 0.2f, 1f);

//    [Header("Recording (optional)")]
//    public bool recordCsv = false;
//    public string fileBaseName = "pxr_24pts_world";
//    public int writeEveryNFrames = 1;

//    [Header("Raw TXT (PICO local, one-shot)")]
//    public bool recordRawTxt = false;
//    public string rawTxtBaseName = "pxr_24pts_raw_local";
//    private bool _rawTxtDumped = false;

//    [Serializable] public class PointsUpdatedEvent : UnityEvent<Vector3[]> { }
//    [Header("Events")]
//    public PointsUpdatedEvent OnPointsUpdated;

//    // 输出：世界坐标 24 点
//    public Vector3[] WorldJoints24 { get; private set; } = new Vector3[24];

//    // 额外：保存对应 24 点的 local 旋转（不做适配，原始 PICO local）
//    public Quaternion[] LocalRotations24 { get; private set; } = new Quaternion[24];

//    private StreamWriter _writer;
//    private int _frameCounter;
//    private string _filePath;

//    // 左右对调的索引（按你的 PositionIndex 定义）
//    private static readonly int[] L = {
//        (int)PositionIndex.left_hip, (int)PositionIndex.left_knee, (int)PositionIndex.left_ankle,
//        (int)PositionIndex.left_foot_index, (int)PositionIndex.left_collar, (int)PositionIndex.left_shoulder,
//        (int)PositionIndex.left_elbow, (int)PositionIndex.left_wrist, (int)PositionIndex.left_hand
//    };
//    private static readonly int[] R = {
//        (int)PositionIndex.right_hip, (int)PositionIndex.right_knee, (int)PositionIndex.right_ankle,
//        (int)PositionIndex.right_foot_index, (int)PositionIndex.right_collar, (int)PositionIndex.right_shoulder,
//        (int)PositionIndex.right_elbow, (int)PositionIndex.right_wrist, (int)PositionIndex.right_hand
//    };

//    void Start()
//    {
//        if (recordCsv)
//        {
//            try
//            {
//                string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
//                _filePath = Path.Combine(Application.persistentDataPath, $"{fileBaseName}_{ts}.csv");
//                _writer = new StreamWriter(_filePath, false, System.Text.Encoding.UTF8);
//                // header：现在包含 position + rotation（四元数以 wxyz 顺序写出）
//                _writer.Write("frame,time");
//                for (int i = 0; i < 24; i++)
//                {
//                    _writer.Write($",p{i}x,p{i}y,p{i}z,q{i}w,q{i}x,q{i}y,q{i}z");
//                }
//                _writer.WriteLine();
//                _writer.Flush();
//                Debug.Log($"[24PointMapper] Recording to {_filePath}");
//            }
//            catch (Exception e)
//            {
//                Debug.LogError($"[24PointMapper] Cannot open writer: {e}");
//                recordCsv = false;
//            }
//        }
//    }

//    void OnDestroy()
//    {
//        try { _writer?.Flush(); _writer?.Close(); } catch { }
//    }

//    void Update()
//    {
//#if UNITY_ANDROID
//        if (feed == null) return;
//        var poses = feed.LatestRolePoses;
//        if (poses == null || poses.Length < 24) return;

//        // ---- 一次性导出 PICO 原始 local 数据为 TXT（仅首帧成功获取时） ----
//        if (recordRawTxt && !_rawTxtDumped)
//        {
//            try
//            {
//                string ts = DateTime.Now.ToString("yyyyMMdd_HHmmss");
//                string rawPath = Path.Combine(Application.persistentDataPath, $"{rawTxtBaseName}_{ts}.txt");
//                using (var sw = new StreamWriter(rawPath, false, System.Text.Encoding.UTF8))
//                {
//                    sw.WriteLine("# PICO raw poses dump (LOCAL coordinates, NO transforms applied)");
//                    sw.WriteLine("# Saved at: " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"));
//                    sw.WriteLine("# persistentDataPath: " + Application.persistentDataPath);
//                    sw.WriteLine("# Format per line: index, valid, px, py, pz, qw, qx, qy, qz");
//                    sw.WriteLine("# NOTE: quaternion components taken directly from Unity Quaternion (x,y,z,w),");
//                    sw.WriteLine("#       only output order is rearranged to w,x,y,z for MuJoCo-style conventions.");
//                    sw.WriteLine();

//                    for (int i = 0; i < 24; i++)
//                    {
//                        bool valid = poses[i].valid;
//                        Vector3 v = poses[i].pos;     // 原始 PICO local 坐标
//                        Quaternion q = poses[i].rot;  // 原始 PICO local 旋转（Unity Quaternion）

//                        sw.WriteLine(
//                            $"{i}, {valid}, " +
//                            $"{v.x:F6}, {v.y:F6}, {v.z:F6}, " +
//                            $"{q.w:F6}, {q.x:F6}, {q.y:F6}, {q.z:F6}"
//                        );
//                    }
//                }
//                _rawTxtDumped = true;
//                Debug.Log($"[24PointMapper] Raw PICO local poses (pos+rot) saved (one-shot) to: {rawPath}");
//            }
//            catch (Exception e)
//            {
//                Debug.LogError($"[24PointMapper] Dump raw TXT failed: {e}");
//            }
//        }
//        // ---- 一次性导出结束 ----

//        // 1) 取 24 点并转世界；同时记录对应 local 旋转（不做适配）
//        for (int i = 0; i < 24; i++)
//        {
//            Vector3 p = poses[i].valid ? poses[i].pos : Vector3.zero;
//            Quaternion qLocal = poses[i].valid ? poses[i].rot : Quaternion.identity;

//            // local -> world 只对 position 处理
//            if (sourceIsLocalSpace && trackingOrigin != null)
//                p = trackingOrigin.TransformPoint(p);

//            // 姿态适配（只对 position 做 swap/flip/yaw）
//            p = AdaptPoint(p);

//            WorldJoints24[i] = p;
//            LocalRotations24[i] = qLocal;  // 保留原始 local quaternion，不做额外变换
//        }

//        // 2) 左右对调（可选）——这里只对点做 swap，不改 quaternion
//        if (swapLeftRight)
//        {
//            for (int i = 0; i < L.Length; i++)
//            {
//                int li = L[i], ri = R[i];
//                (WorldJoints24[li], WorldJoints24[ri]) = (WorldJoints24[ri], WorldJoints24[li]);
//                // 如未来需要对旋转也左右对调，可在这里同步交换 LocalRotations24[li]/[ri]
//            }
//        }

//        // 3) 事件回调
//        OnPointsUpdated?.Invoke(WorldJoints24);

//        // 4) 记录 CSV（position + local quaternion）
//        if (recordCsv && (++_frameCounter % writeEveryNFrames == 0))
//            WriteCsv(WorldJoints24, LocalRotations24);
//#endif
//    }

//    // ----------------- 辅助：姿态适配（仅 position） -----------------
//    private Vector3 AdaptPoint(Vector3 pWorld)
//    {
//        Transform T = trackingOrigin != null ? trackingOrigin : null;
//        Vector3 local = T ? T.InverseTransformPoint(pWorld) : pWorld;

//        if (swapXZ) local = new Vector3(local.z, local.y, local.x);

//        local = new Vector3(
//            flipX ? -local.x : local.x,
//            flipY ? -local.y : local.y,
//            flipZ ? -local.z : local.z
//        );

//        Vector3 w = T ? T.TransformPoint(local) : local;

//        if (Mathf.Abs(yawOffsetDeg) > 0.001f)
//        {
//            Quaternion yaw = Quaternion.AngleAxis(yawOffsetDeg, Vector3.up);
//            if (T)
//            {
//                Vector3 rel = w - T.position;
//                w = T.position + yaw * rel;
//            }
//            else
//            {
//                w = yaw * w;
//            }
//        }
//        return w;
//    }

//    private void WriteCsv(Vector3[] pts, Quaternion[] quatsLocal)
//    {
//        if (_writer == null) return;
//        try
//        {
//            _writer.Write($"{Time.frameCount},{Time.time:F6}");
//            for (int i = 0; i < 24; i++)
//            {
//                Vector3 p = pts[i];
//                Quaternion q = (quatsLocal != null && i < quatsLocal.Length) ? quatsLocal[i] : Quaternion.identity;

//                // position: world space after适配
//                // quaternion: 原始 local PICO Quaternion，只按 wxyz 顺序输出，不做额外处理
//                _writer.Write(
//                    $",{p.x:F6},{p.y:F6},{p.z:F6}," +
//                    $"{q.w:F6},{q.x:F6},{q.y:F6},{q.z:F6}"
//                );
//            }
//            _writer.WriteLine();
//            _writer.Flush();
//        }
//        catch (Exception e)
//        {
//            Debug.LogError($"[24PointMapper] WriteCsv failed: {e}");
//            recordCsv = false;
//            try { _writer?.Flush(); _writer?.Close(); } catch { }
//        }
//    }

//    // ----------------- Gizmos 可视化 -----------------
//    void OnDrawGizmos()
//    {
//        if (!drawGizmos || WorldJoints24 == null || WorldJoints24.Length < 24) return;

//        Gizmos.color = gizmoPointColor;
//        foreach (var p in WorldJoints24)
//            Gizmos.DrawSphere(p, gizmoPointSize);

//        Gizmos.color = gizmoBoneColor;
//        DrawBone(PositionIndex.hip, PositionIndex.spine1);
//        DrawBone(PositionIndex.spine1, PositionIndex.spine2);
//        DrawBone(PositionIndex.spine2, PositionIndex.spine3);
//        DrawBone(PositionIndex.spine3, PositionIndex.neck);
//        DrawBone(PositionIndex.neck, PositionIndex.head);

//        DrawBone(PositionIndex.left_hip, PositionIndex.left_knee);
//        DrawBone(PositionIndex.left_knee, PositionIndex.left_ankle);
//        DrawBone(PositionIndex.left_ankle, PositionIndex.left_foot_index);

//        DrawBone(PositionIndex.right_hip, PositionIndex.right_knee);
//        DrawBone(PositionIndex.right_knee, PositionIndex.right_ankle);
//        DrawBone(PositionIndex.right_ankle, PositionIndex.right_foot_index);

//        DrawBone(PositionIndex.left_collar, PositionIndex.left_shoulder);
//        DrawBone(PositionIndex.left_shoulder, PositionIndex.left_elbow);
//        DrawBone(PositionIndex.left_elbow, PositionIndex.left_wrist);
//        DrawBone(PositionIndex.left_wrist, PositionIndex.left_hand);

//        DrawBone(PositionIndex.right_collar, PositionIndex.right_shoulder);
//        DrawBone(PositionIndex.right_shoulder, PositionIndex.right_elbow);
//        DrawBone(PositionIndex.right_elbow, PositionIndex.right_wrist);
//        DrawBone(PositionIndex.right_wrist, PositionIndex.right_hand);
//    }

//    private void DrawBone(PositionIndex a, PositionIndex b)
//    {
//        int ia = (int)a, ib = (int)b;
//        if (WorldJoints24 == null || ia >= WorldJoints24.Length || ib >= WorldJoints24.Length) return;
//        Vector3 pa = WorldJoints24[ia], pb = WorldJoints24[ib];
//        Gizmos.DrawLine(pa, pb);
//    }
//}

