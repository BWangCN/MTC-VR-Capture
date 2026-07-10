// ========================= PXR_24PointCsvPlayer.cs (FULL) =========================
// Method-3 strict replay:
// - Reads META lines starting with '#':
//   # record_origin_pos=x,y,z
//   # record_origin_rot_xyzw=x,y,z,w
// - During playback, maps each point:
//     p_replay = T(replayOrigin) * inv(T(recordOrigin)) * p_record
// - If META is missing or replayOrigin is null, falls back to raw points.
//
// Usage:
// 1) Drag this script onto any GameObject in your replay scene (Windows/Editor).
// 2) Set csvFileName to an absolute path (as you do) or a file name under persistentDataPath.
// 3) Assign replayOrigin:
//    - Best: your current scene XR Rig TrackingOrigin (or any Transform you want as "new origin").
// 4) Press Play.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public class PXR_24PointCsvPlayer : MonoBehaviour
{
    [Header("CSV 设置")]
    [Tooltip("CSV 文件名（不含路径），默认从 Application.persistentDataPath 下读取。也可直接填绝对路径。")]
    public string csvFileName = "C:\\Users\\Wayne\\Desktop\\GMU\\PhD\\Humanoid\\smpl_24pts_test_20251114_133818.csv";

    [Tooltip("是否在 Start 时自动加载 CSV")]
    public bool loadOnStart = true;

    [Header("Bone Thickness (Gizmos)")]
    public bool thickBones = true;
    public float boneThickness = 0.01f;   // 米为单位，0.005~0.02 之间调
    public int boneThickRays = 6;         // 4/6/8 都行


    [Header("播放设置")]
    [Tooltip("回放时使用的帧率（逻辑播放用，不一定等于录制时的帧率）")]
    public float playbackFps = 60f;

    [Tooltip("播到最后一帧是否循环回到开头")]
    public bool loop = true;

    [Tooltip("开始时是否自动播放")]
    public bool autoPlay = true;

    [Header("Origin Mapping (Method 3)")]
    [Tooltip("将录制时的 recordOrigin 映射到回放场景的 replayOrigin：p_replay = T(replayOrigin)*inv(T(recordOrigin))*p_record")]
    public bool applyOriginMapping = true;

    [Tooltip("回放目标原点（强烈建议指定为你回放场景的 TrackingOrigin / XR Rig 原点）。")]
    public Transform replayOrigin;

    [Header("Gizmos 可视化")]
    public bool drawGizmos = true;
    public float gizmoPointSize = 0.025f;
    public Color gizmoPointColor = new Color(0.1f, 0.8f, 1f, 1f);
    public Color gizmoBoneColor = new Color(1f, 0.9f, 0.2f, 1f);

    private readonly List<Vector3[]> _frames = new List<Vector3[]>();
    private int _totalFrames = 0;

    private readonly Vector3[] _currentJoints24 = new Vector3[24];

    private float _timeCursor = 0f;
    private bool _isPlaying = false;

    // record origin meta
    private bool _hasRecordOrigin = false;
    private Vector3 _recordOriginPos = Vector3.zero;
    private Quaternion _recordOriginRot = Quaternion.identity;

    // mapping matrix
    private Matrix4x4 _M = Matrix4x4.identity;
    private bool _MReady = false;

    void Start()
    {
        if (loadOnStart) LoadCsv();
        _isPlaying = autoPlay;
    }

    void Update()
    {
        if (!_isPlaying || _totalFrames == 0) return;

        _timeCursor += Time.deltaTime;
        float frameFloat = _timeCursor * playbackFps;
        int frameIndex = Mathf.FloorToInt(frameFloat);

        if (frameIndex >= _totalFrames)
        {
            if (loop)
            {
                frameIndex = frameIndex % _totalFrames;
                _timeCursor = frameIndex / playbackFps;
            }
            else
            {
                frameIndex = _totalFrames - 1;
                _isPlaying = false;
            }
        }

        EnsureMappingMatrix();

        var src = _frames[frameIndex];
        for (int i = 0; i < 24; i++)
        {
            Vector3 p = src[i];
            if (applyOriginMapping && _MReady)
                p = _M.MultiplyPoint3x4(p);
            _currentJoints24[i] = p;
        }
    }

    public void LoadCsv()
    {
        _frames.Clear();
        _totalFrames = 0;
        _timeCursor = 0f;
        _isPlaying = false;
        _MReady = false;
        _M = Matrix4x4.identity;

        // reset meta
        _hasRecordOrigin = false;
        _recordOriginPos = Vector3.zero;
        _recordOriginRot = Quaternion.identity;

        string path = csvFileName;

        if (!Path.IsPathRooted(csvFileName))
            path = Path.Combine(Application.persistentDataPath, csvFileName);

        if (!File.Exists(path))
        {
            Debug.LogError($"[PXR_24PointCsvPlayer] CSV not found: {path}");
            return;
        }

        Debug.Log($"[PXR_24PointCsvPlayer] Loading CSV: {path}");

        try
        {
            var lines = File.ReadAllLines(path);
            if (lines.Length <= 1)
            {
                Debug.LogError("[PXR_24PointCsvPlayer] CSV has no data lines.");
                return;
            }

            // 1) Parse META lines, find header index
            int headerIndex = -1;
            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;

                if (line.StartsWith("#"))
                {
                    ParseMetaLine(line);
                    continue;
                }

                headerIndex = i;
                break;
            }

            if (headerIndex < 0 || headerIndex >= lines.Length - 1)
            {
                Debug.LogError("[PXR_24PointCsvPlayer] Cannot find CSV header/data.");
                return;
            }

            // 2) Parse data lines (after header)
            for (int lineIndex = headerIndex + 1; lineIndex < lines.Length; lineIndex++)
            {
                string line = lines[lineIndex].Trim();
                if (string.IsNullOrEmpty(line))
                    continue;

                // skip accidental meta lines inside file
                if (line.StartsWith("#")) continue;

                string[] cols = line.Split(',');

                // expected: frame,time + cam(7) + 24*(7) = 2+7+168 = 177 columns
                // also support old format without cam columns (170 columns)
                if (cols.Length < 170)
                {
                    Debug.LogWarning($"[PXR_24PointCsvPlayer] Line {lineIndex + 1} has too few columns ({cols.Length}). Skipped.");
                    continue;
                }

                Vector3[] joints = new Vector3[24];

                // new format has 7 camera columns (cam_px..cam_qz) before joints
                int baseCol = (cols.Length >= 177) ? 9 : 2;
                for (int j = 0; j < 24; j++)
                {
                    int idx = baseCol + j * 7;

                    float px = ParseFloat(cols[idx + 0]);
                    float py = ParseFloat(cols[idx + 1]);
                    float pz = ParseFloat(cols[idx + 2]);

                    joints[j] = new Vector3(px, py, pz);
                }

                _frames.Add(joints);
            }

            _totalFrames = _frames.Count;
            if (_totalFrames > 0)
            {
                Array.Copy(_frames[0], _currentJoints24, 24);
                _isPlaying = autoPlay;

                Debug.Log($"[PXR_24PointCsvPlayer] Loaded {_totalFrames} frames. hasRecordOrigin={_hasRecordOrigin}");
            }
            else
            {
                Debug.LogWarning("[PXR_24PointCsvPlayer] No valid frames parsed from CSV.");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[PXR_24PointCsvPlayer] LoadCsv failed: {e}");
        }
    }

    private void ParseMetaLine(string rawLine)
    {
        // rawLine like "# key=value"
        string line = rawLine.Substring(1).Trim();

        int eq = line.IndexOf('=');
        if (eq < 0) return;

        string key = line.Substring(0, eq).Trim();
        string val = line.Substring(eq + 1).Trim();

        if (key == "record_origin_pos")
        {
            var v = val.Split(',');
            if (v.Length == 3)
            {
                _recordOriginPos = new Vector3(ParseFloat(v[0]), ParseFloat(v[1]), ParseFloat(v[2]));
                _hasRecordOrigin = true;
            }
        }
        else if (key == "record_origin_rot_xyzw")
        {
            var q = val.Split(',');
            if (q.Length == 4)
            {
                float x = ParseFloat(q[0]);
                float y = ParseFloat(q[1]);
                float z = ParseFloat(q[2]);
                float w = ParseFloat(q[3]);
                _recordOriginRot = new Quaternion(x, y, z, w);
                _hasRecordOrigin = true;
            }
        }
    }

    private void EnsureMappingMatrix()
    {
        if (_MReady) return;

        if (!applyOriginMapping)
        {
            _M = Matrix4x4.identity;
            _MReady = true;
            return;
        }

        if (!_hasRecordOrigin)
        {
            Debug.LogWarning("[PXR_24PointCsvPlayer] No record origin meta found. Mapping disabled.");
            _M = Matrix4x4.identity;
            _MReady = true;
            return;
        }

        if (replayOrigin == null)
        {
            Debug.LogWarning("[PXR_24PointCsvPlayer] replayOrigin is null. Mapping disabled.");
            _M = Matrix4x4.identity;
            _MReady = true;
            return;
        }

        Matrix4x4 T_record = Matrix4x4.TRS(_recordOriginPos, _recordOriginRot, Vector3.one);
        Matrix4x4 T_replay = replayOrigin.localToWorldMatrix;

        _M = T_replay * T_record.inverse;
        _MReady = true;

        Debug.Log($"[PXR_24PointCsvPlayer] Origin mapping ready. recordPos={_recordOriginPos}, replayOrigin={replayOrigin.name}");
    }

    private float ParseFloat(string s)
    {
        if (float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
            return v;
        return 0f;
    }

    // ----------------- Playback controls -----------------
    public void Play()
    {
        if (_totalFrames == 0) return;
        _isPlaying = true;
    }

    public void Pause()
    {
        _isPlaying = false;
    }

    public void Stop()
    {
        _isPlaying = false;
        _timeCursor = 0f;
        if (_totalFrames > 0)
            Array.Copy(_frames[0], _currentJoints24, 24);
    }

    // ----------------- Gizmos -----------------
    void OnDrawGizmos()
    {
        if (!drawGizmos || _currentJoints24 == null || _currentJoints24.Length < 24)
            return;

        Gizmos.color = gizmoPointColor;
        foreach (var p in _currentJoints24)
            Gizmos.DrawSphere(p, gizmoPointSize);

        Gizmos.color = gizmoBoneColor;
        // 脊柱
        DrawBone(PositionIndex.hip, PositionIndex.spine1);
        DrawBone(PositionIndex.spine1, PositionIndex.spine2);
        DrawBone(PositionIndex.spine2, PositionIndex.spine3);
        DrawBone(PositionIndex.spine3, PositionIndex.neck);
        DrawBone(PositionIndex.neck, PositionIndex.head);

        // 左腿
        DrawBone(PositionIndex.hip, PositionIndex.left_hip);
        DrawBone(PositionIndex.left_hip, PositionIndex.left_knee);
        DrawBone(PositionIndex.left_knee, PositionIndex.left_ankle);
        DrawBone(PositionIndex.left_ankle, PositionIndex.left_foot_index);

        // 右腿
        DrawBone(PositionIndex.hip, PositionIndex.right_hip);
        DrawBone(PositionIndex.right_hip, PositionIndex.right_knee);
        DrawBone(PositionIndex.right_knee, PositionIndex.right_ankle);
        DrawBone(PositionIndex.right_ankle, PositionIndex.right_foot_index);

        // 左臂
        DrawBone(PositionIndex.spine3, PositionIndex.left_collar);
        DrawBone(PositionIndex.left_collar, PositionIndex.left_shoulder);
        DrawBone(PositionIndex.left_shoulder, PositionIndex.left_elbow);
        DrawBone(PositionIndex.left_elbow, PositionIndex.left_wrist);
        DrawBone(PositionIndex.left_wrist, PositionIndex.left_hand);

        // 右臂
        DrawBone(PositionIndex.spine3, PositionIndex.right_collar);
        DrawBone(PositionIndex.right_collar, PositionIndex.right_shoulder);
        DrawBone(PositionIndex.right_shoulder, PositionIndex.right_elbow);
        DrawBone(PositionIndex.right_elbow, PositionIndex.right_wrist);
        DrawBone(PositionIndex.right_wrist, PositionIndex.right_hand);
    }

    //private void DrawBone(PositionIndex a, PositionIndex b)
    //{
    //    int ia = (int)a, ib = (int)b;
    //    if (_currentJoints24 == null || ia >= _currentJoints24.Length || ib >= _currentJoints24.Length)
    //        return;

    //    Vector3 pa = _currentJoints24[ia];
    //    Vector3 pb = _currentJoints24[ib];
    //    Gizmos.DrawLine(pa, pb);
    //}
    private void DrawBone(PositionIndex a, PositionIndex b)
    {
        int ia = (int)a, ib = (int)b;
        if (_currentJoints24 == null || ia >= _currentJoints24.Length || ib >= _currentJoints24.Length)
            return;

        Vector3 pa = _currentJoints24[ia];
        Vector3 pb = _currentJoints24[ib];

        if (!thickBones || boneThickness <= 0f || boneThickRays <= 1)
        {
            Gizmos.DrawLine(pa, pb);
            return;
        }

        Vector3 dir = (pb - pa);
        float len = dir.magnitude;
        if (len < 1e-6f)
            return;

        dir /= len;

        // 找到一个与 dir 不共线的参考向量
        Vector3 refUp = (Mathf.Abs(Vector3.Dot(dir, Vector3.up)) > 0.95f) ? Vector3.right : Vector3.up;

        // 两个互相垂直的法向基
        Vector3 n1 = Vector3.Cross(dir, refUp).normalized;
        Vector3 n2 = Vector3.Cross(dir, n1).normalized;

        // 中心线
        Gizmos.DrawLine(pa, pb);

        // 多条偏移线模拟粗线
        // 在 n1 / n2 上均匀取一些方向
        int rays = Mathf.Max(2, boneThickRays);
        for (int k = 0; k < rays; k++)
        {
            float t = (float)k / rays * Mathf.PI * 2f;
            Vector3 off = (Mathf.Cos(t) * n1 + Mathf.Sin(t) * n2) * boneThickness;

            Gizmos.DrawLine(pa + off, pb + off);
        }
    }

}


//using System;
//using System.Collections.Generic;
//using System.Globalization;
//using System.IO;
//using UnityEngine;

///// <summary>
///// 从 PXR_24PointMapper 生成的 CSV 中读取 24 关节的 world position，
///// 在同一场景中以火柴人形式回放。
/////
///// CSV 格式假定为：
///// frame,time,
/////   p0x,p0y,p0z,q0w,q0x,q0y,q0z,
/////   ...
/////   p23x,p23y,p23z,q23w,q23x,q23y,q23z
/////
///// 这里只使用 p{i}x/p{i}y/p{i}z 来可视化，忽略四元数。
///// </summary>
//public class PXR_24PointCsvPlayer : MonoBehaviour
//{
//    [Header("CSV 设置")]
//    [Tooltip("CSV 文件名（不含路径），默认从 Application.persistentDataPath 下读取。")]
//    public string csvFileName = "C:\\Users\\Wayne\\Desktop\\GMU\\PhD\\Humanoid\\smpl_24pts_test_20251114_133818.csv";

//    [Tooltip("是否在 Start 时自动加载 CSV")]
//    public bool loadOnStart = true;

//    [Header("播放设置")]
//    [Tooltip("回放时使用的帧率（逻辑播放用，不一定等于录制时的帧率）")]
//    public float playbackFps = 60f;

//    [Tooltip("播到最后一帧是否循环回到开头")]
//    public bool loop = true;

//    [Tooltip("开始时是否自动播放")]
//    public bool autoPlay = true;

//    [Header("Gizmos 可视化")]
//    public bool drawGizmos = true;
//    public float gizmoPointSize = 0.025f;
//    public Color gizmoPointColor = new Color(0.1f, 0.8f, 1f, 1f);
//    public Color gizmoBoneColor = new Color(1f, 0.9f, 0.2f, 1f);

//    // 存储从 CSV 读取的所有帧，每帧是 24 个点的数组
//    private List<Vector3[]> _frames = new List<Vector3[]>();
//    private int _totalFrames = 0;

//    // 当前播放用的一帧 24 点（用于 Gizmos）
//    private Vector3[] _currentJoints24 = new Vector3[24];

//    // 播放时间累积（秒）
//    private float _timeCursor = 0f;
//    private bool _isPlaying = false;

//    void Start()
//    {
//        if (loadOnStart)
//        {
//            LoadCsv();
//        }

//        _isPlaying = autoPlay;
//    }

//    void Update()
//    {
//        if (!_isPlaying || _totalFrames == 0) return;

//        // 根据 playbackFps 推进时间，计算当前应显示哪一帧
//        _timeCursor += Time.deltaTime;
//        float frameFloat = _timeCursor * playbackFps;
//        int frameIndex = Mathf.FloorToInt(frameFloat);

//        if (frameIndex >= _totalFrames)
//        {
//            if (loop)
//            {
//                frameIndex = frameIndex % _totalFrames;
//                _timeCursor = frameIndex / playbackFps;
//            }
//            else
//            {
//                frameIndex = _totalFrames - 1;
//                _isPlaying = false;  // 播到最后一帧停住
//            }
//        }

//        // 更新当前 24 点
//        Array.Copy(_frames[frameIndex], _currentJoints24, 24);
//    }

//    /// <summary>
//    /// 从 CSV 文件读取所有帧。
//    /// 默认路径：Application.persistentDataPath/csvFileName
//    /// </summary>
//    public void LoadCsv()
//    {
//        _frames.Clear();
//        _totalFrames = 0;
//        _timeCursor = 0f;

//        //string path = Path.Combine(Application.persistentDataPath, csvFileName);
//        string path = csvFileName;

//        // 如果不是绝对路径，就自动拼到 persistentDataPath
//        if (!Path.IsPathRooted(csvFileName))
//        {
//            path = Path.Combine(Application.persistentDataPath, csvFileName);
//        }

//        if (!File.Exists(path))
//        {
//            Debug.LogError($"[PXR_24PointCsvPlayer] CSV not found: {path}");
//            return;
//        }


//        if (!File.Exists(path))
//        {
//            Debug.LogError($"[PXR_24PointCsvPlayer] CSV not found: {path}");
//            return;
//        }

//        Debug.Log($"[PXR_24PointCsvPlayer] Loading CSV: {path}");

//        try
//        {
//            var lines = File.ReadAllLines(path);
//            if (lines.Length <= 1)
//            {
//                Debug.LogError("[PXR_24PointCsvPlayer] CSV has no data lines.");
//                return;
//            }

//            // 跳过第一行 header，从第二行开始解析
//            for (int lineIndex = 1; lineIndex < lines.Length; lineIndex++)
//            {
//                string line = lines[lineIndex].Trim();
//                if (string.IsNullOrEmpty(line))
//                    continue;

//                string[] cols = line.Split(',');
//                // 最少应有：frame,time + 24*(7) = 2 + 168 = 170 列
//                if (cols.Length < 170)
//                {
//                    Debug.LogWarning($"[PXR_24PointCsvPlayer] Line {lineIndex + 1} has too few columns ({cols.Length}). Skipped.");
//                    continue;
//                }

//                Vector3[] joints = new Vector3[24];

//                // 从第 2 列开始（index 0=frame, 1=time）
//                // 每个关节占 7 列：px,py,pz,qw,qx,qy,qz
//                // 我们只读取前 3 列位置
//                int baseCol = 2;
//                for (int j = 0; j < 24; j++)
//                {
//                    int idx = baseCol + j * 7;

//                    float px = ParseFloat(cols[idx + 0]);
//                    float py = ParseFloat(cols[idx + 1]);
//                    float pz = ParseFloat(cols[idx + 2]);

//                    joints[j] = new Vector3(px, py, pz);
//                }

//                _frames.Add(joints);
//            }

//            _totalFrames = _frames.Count;
//            if (_totalFrames > 0)
//            {
//                // 初始化当前一帧为第一帧
//                Array.Copy(_frames[0], _currentJoints24, 24);
//                Debug.Log($"[PXR_24PointCsvPlayer] Loaded {_totalFrames} frames.");
//            }
//            else
//            {
//                Debug.LogWarning("[PXR_24PointCsvPlayer] No valid frames parsed from CSV.");
//            }
//        }
//        catch (Exception e)
//        {
//            Debug.LogError($"[PXR_24PointCsvPlayer] LoadCsv failed: {e}");
//        }
//    }

//    private float ParseFloat(string s)
//    {
//        if (float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out float v))
//            return v;
//        return 0f;
//    }

//    // ----------------- 播放控制（可在 Inspector 或其他脚本调用） -----------------
//    public void Play()
//    {
//        if (_totalFrames == 0) return;
//        _isPlaying = true;
//    }

//    public void Pause()
//    {
//        _isPlaying = false;
//    }

//    public void Stop()
//    {
//        _isPlaying = false;
//        _timeCursor = 0f;
//        if (_totalFrames > 0)
//            Array.Copy(_frames[0], _currentJoints24, 24);
//    }

//    // ----------------- Gizmos 可视化 -----------------
//    void OnDrawGizmos()
//    {
//        if (!drawGizmos || _currentJoints24 == null || _currentJoints24.Length < 24)
//            return;

//        // 画点
//        Gizmos.color = gizmoPointColor;
//        foreach (var p in _currentJoints24)
//            Gizmos.DrawSphere(p, gizmoPointSize);

//        // 画骨骼
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
//        if (_currentJoints24 == null || ia >= _currentJoints24.Length || ib >= _currentJoints24.Length)
//            return;

//        Vector3 pa = _currentJoints24[ia];
//        Vector3 pb = _currentJoints24[ib];
//        Gizmos.DrawLine(pa, pb);
//    }
//}
