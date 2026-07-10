using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using UnityEngine;
using Unity.XR.PXR;

public class PXR_BodyTrackingFeed : MonoBehaviour
{
    [Header("Output / Coordinates")]
    public bool outputRotation = true;          // 只要位置可关掉
    public bool useLocalCoords = true;          // true: 使用 SDK 的 localPose；false: 转到世界坐标（以 cubesRoot 为参考）

    [Header("Cubes Visualization")]
    public bool showCubes = true;
    public Transform cubesRoot;                 // 可选：作为可视化父节点
    public GameObject cubePrefab;               // 可选：自定义方块
    public Vector3 cubeScale = new Vector3(0.04f, 0.04f, 0.04f);

    [Header("Tracking")]
    public bool autoStartTracking = true;
    public BodyJointSet jointSetToStart = BodyJointSet.BODY_JOINT_SET_BODY_FULL_START;

    [Header("TXT Dump (Tree with Types)")]
    public bool autoDumpFirstValidFrame = true;   // 第一次拿到有效数据就自动Dump
    public KeyCode dumpHotkey = KeyCode.F9;       // 手动Dump热键（可改为按钮/手柄）
    public string dumpFileName = "bd_dump";       // 基础文件名（自动加时间戳）
    public int dumpMaxDepth = 8;                  // 反射最大深度，避免意外递归
    private bool hasDumpedOnce = false;

    [Header("Stickman (Lines)")]
    public bool showStickman = true;
    public float lineWidth = 0.01f;
    public Material lineMaterial;          // 可选；为空时用 Sprites/Default
    public Color lineColor = Color.green;  // 线的颜色
    private Transform stickmanRoot;        // 线段父节点
    private readonly System.Collections.Generic.Dictionary<(int, int), LineRenderer> boneLines
        = new System.Collections.Generic.Dictionary<(int, int), LineRenderer>();


    private bool supportedBT = false;
    private bool updateBT = false;
    private bool isTracking = false;
    private BodyTrackingStatus bs = new BodyTrackingStatus();
    private BodyTrackingGetDataInfo bdi = new BodyTrackingGetDataInfo();
    private BodyTrackingData bd = new BodyTrackingData();

    private static readonly (int a, int b)[] StickmanEdges = new (int, int)[]
    {
        // 躯干：Pelvis → Spine_01 → Spine_02 → Spine_03 → Neck → Head
        (0,3), (3,6), (6,9), (9,12), (12,15),

        // 骨盆到左右髋
        (0,1), (0,2),

        // 左腿：L_Hip → L_Knee → L_Ankle → L_Foot
        (1,4), (4,7), (7,10),

        // 右腿：R_Hip → R_Knee → R_Ankle → R_Foot
        (2,5), (5,8), (8,11),

        // 锁骨到颈
        (13,12), (14,12),

        // 左臂：L_Collar → L_Shoulder → L_Elbow → L_Wrist → L_Hand
        (13,16), (16,18), (18,20), (20,22),

        // 右臂：R_Collar → R_Shoulder → R_Elbow → R_Wrist → R_Hand
        (14,17), (17,19), (19,21), (21,23),
    };

    private void UpdateStickmanLines()
    {
        if (!showStickman || LatestRolePoses == null) return;

        // 1) 创建父节点（一次）
        if (stickmanRoot == null)
        {
            var go = new GameObject("StickmanLines");
            if (cubesRoot != null) go.transform.SetParent(cubesRoot, false);
            else go.transform.SetParent(transform, false);
            stickmanRoot = go.transform;
        }

        // 2) 遍历边，生成/更新 LineRenderer
        foreach (var (a, b) in StickmanEdges)
        {
            // 索引与有效性检查
            if (a < 0 || b < 0 || a >= LatestRolePoses.Length || b >= LatestRolePoses.Length)
                continue;

            var pa = LatestRolePoses[a];
            var pb = LatestRolePoses[b];
            if (!pa.valid || !pb.valid) continue;

            // 获取或创建该边的 LineRenderer
            if (!boneLines.TryGetValue((a, b), out var lr) || lr == null)
            {
                var name = $"Bone_{a}_{b}";
                var lineGO = new GameObject(name);
                lineGO.transform.SetParent(stickmanRoot, false);

                lr = lineGO.AddComponent<LineRenderer>();
                lr.positionCount = 2;
                lr.numCapVertices = 4;
                lr.numCornerVertices = 4;
                lr.startWidth = lineWidth;
                lr.endWidth = lineWidth;

                // 材质
                if (lineMaterial != null) lr.material = lineMaterial;
                else lr.material = new Material(Shader.Find("Sprites/Default"));

                lr.startColor = lineColor;
                lr.endColor = lineColor;

                // 与小方块坐标策略保持一致：
                // useLocalCoords = true  -> 我们设置 local 点，LineRenderer 用局部坐标（useWorldSpace=false）
                // useLocalCoords = false -> 我们设置世界坐标，LineRenderer 用世界坐标（useWorldSpace=true）
                lr.useWorldSpace = !useLocalCoords;

                boneLines[(a, b)] = lr;
            }

            // 计算两端点
            Vector3 A, B;
            if (useLocalCoords)
            {
                // 局部坐标：localPose 直接用
                A = pa.pos;
                B = pb.pos;
            }
            else
            {
                // 世界坐标：与小方块逻辑一致用 TransformPoint
                if (cubesRoot != null)
                {
                    A = cubesRoot.TransformPoint(pa.pos);
                    B = cubesRoot.TransformPoint(pb.pos);
                }
                else
                {
                    A = pa.pos;
                    B = pb.pos;
                }
            }

            lr.SetPosition(0, A);
            lr.SetPosition(1, B);
        }
    }



    // 公开给外部消费（例如你的 SMPL-X 管线）
    public struct RolePose
    {
        public Vector3 pos;        // meters, PICO tracking local space
        public Quaternion rot;     // local space
        public bool valid;
    }
    public RolePose[] LatestRolePoses { get; private set; }

    // 关节小方块（仅用于可视化）
    private readonly Dictionary<int, Transform> cubeByIndex = new Dictionary<int, Transform>();

    void Start()
    {
        LatestRolePoses = new RolePose[(int)BodyTrackerRole.ROLE_NUM];
        if (autoStartTracking) StartBodyTracking();
    }

    void OnDestroy()
    {
        StopBodyTracking();
    }

#if UNITY_ANDROID
    void Update()
    {
        if (!updateBT) return;

        // 热键手动Dump（真机可改为UI按钮或手柄映射）
        if (Input.GetKeyDown(dumpHotkey))
        {
            TryDumpBDTxt("manual");
        }

        PXR_MotionTracking.GetBodyTrackingState(ref isTracking, ref bs);
        if (bs.stateCode != BodyTrackingStatusCode.BT_VALID)
            return; // 未校准/未就绪，不更新

        int ret = PXR_MotionTracking.GetBodyTrackingData(ref bdi, ref bd);
        if (ret != 0) return;

        int n = Math.Min(LatestRolePoses.Length, bd.roleDatas.Length);
        for (int i = 0; i < n; i++)
        {
            if (showStickman) UpdateStickmanLines();

            var pose = bd.roleDatas[i].localPose; // SDK给的局部位姿
            var p = new Vector3((float)pose.PosX, (float)pose.PosY, (float)pose.PosZ);
            var q = new Quaternion((float)pose.RotQx, (float)pose.RotQy, (float)pose.RotQz, (float)pose.RotQw);

            LatestRolePoses[i] = new RolePose { pos = p, rot = q, valid = true };

            // 可视化（可选）
            if (showCubes)
            {
                var cube = GetOrCreateCube(i);
                if (useLocalCoords)
                {
                    cube.localPosition = p;
                    if (outputRotation) cube.localRotation = q;
                }
                else
                {
                    if (cubesRoot != null)
                    {
                        cube.position = cubesRoot.TransformPoint(p);
                        if (outputRotation) cube.rotation = cubesRoot.rotation * q;
                    }
                    else
                    {
                        cube.position = p; // 仅调试
                        if (outputRotation) cube.rotation = q;
                    }
                }
            }
        }

        // 首帧有效数据自动Dump
        if (autoDumpFirstValidFrame && !hasDumpedOnce)
        {
            TryDumpBDTxt("first_valid_frame");
            hasDumpedOnce = true;
        }

        // TODO: 这里可把 LatestRolePoses 传给你的 SMPL-X 管线
        // e.g., MySmplxConsumer.UpdateFromJoints(LatestRolePoses);
    }
#endif

    public void StartBodyTracking()
    {
        PXR_MotionTracking.GetBodyTrackingSupported(ref supportedBT);
        if (!supportedBT)
        {
            Debug.LogWarning("[PXR_BodyTrackingFeed] Body tracking not supported on this device.");
            return;
        }

        BodyTrackingBoneLength bones = new BodyTrackingBoneLength(); // 不用骨长就默认
        PXR_MotionTracking.StartBodyTracking(jointSetToStart, bones);

        PXR_MotionTracking.GetBodyTrackingState(ref isTracking, ref bs);
        if (bs.stateCode != BodyTrackingStatusCode.BT_VALID &&
            (bs.message == BodyTrackingMessage.BT_MESSAGE_TRACKER_NOT_CALIBRATED ||
             bs.message == BodyTrackingMessage.BT_MESSAGE_UNKNOWN))
        {
            Debug.Log("[PXR_BodyTrackingFeed] Launching calibration app...");
            PXR_MotionTracking.StartMotionTrackerCalibApp();
        }

        updateBT = true;
    }

    public void StopBodyTracking()
    {
        updateBT = false;
        PXR_MotionTracking.StopBodyTracking();
    }

    private Transform GetOrCreateCube(int index)
    {
        if (cubeByIndex.TryGetValue(index, out var t) && t != null) return t;

        GameObject go;
        if (cubePrefab != null)
            go = Instantiate(cubePrefab, cubesRoot != null ? cubesRoot : transform);
        else
            go = GameObject.CreatePrimitive(PrimitiveType.Cube);

        go.name = ((BodyTrackerRole)index).ToString();
        t = go.transform;
        if (cubesRoot != null) t.SetParent(cubesRoot, worldPositionStays: false);
        t.localScale = cubeScale;

        cubeByIndex[index] = t;
        return t;
    }

    // ------------------- TXT Dump（树状 + 类型信息） -------------------

    private void TryDumpBDTxt(string tag)
    {
        try
        {
            string time = DateTime.Now.ToString("yyyyMMdd_HHmmss");
            string file = $"{dumpFileName}_{tag}_{time}.txt";
            string path = Path.Combine(Application.persistentDataPath, file);

            using (var sw = new StreamWriter(path, false))
            {
                var visited = new HashSet<object>(new RefComparer());
                WriteObjectTree(sw, "bd", bd, 0, dumpMaxDepth, visited);
            }

            Debug.Log($"[PXR_BodyTrackingFeed] BodyTrackingData dumped to: {path}");
        }
        catch (Exception ex)
        {
            Debug.LogError($"[PXR_BodyTrackingFeed] Dump failed: {ex}");
        }
    }

    /// <summary>
    /// 把任意对象以树状结构写出到文本，包含类型信息；递归字段与属性（含私有）。
    /// </summary>
    private void WriteObjectTree(TextWriter sw, string name, object obj, int indent, int depth, HashSet<object> visited)
    {
        string ind = new string(' ', indent * 4);

        if (obj == null)
        {
            sw.WriteLine($"{ind}{name} (null)");
            return;
        }

        Type t = obj.GetType();

        // 简单类型：值类型、字符串、枚举、decimal/double/float/bool
        if (IsSimpleLeaf(t))
        {
            sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}): {SafeToString(obj)}");
            return;
        }

        // 避免循环引用（仅对引用类型）
        if (!t.IsValueType)
        {
            if (visited.Contains(obj))
            {
                sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}): <CyclicRef>");
                return;
            }
            visited.Add(obj);
        }

        // 达到最大深度
        if (depth <= 0)
        {
            sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}): <MaxDepthReached>");
            return;
        }

        // 集合/数组
        if (typeof(System.Collections.IEnumerable).IsAssignableFrom(t) && t != typeof(string))
        {
            sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}):");
            int idx = 0;
            foreach (var item in (System.Collections.IEnumerable)obj)
            {
                string childName = $"[{idx}] ({TypeNameFriendly(item?.GetType())})";
                WriteObjectTree(sw, childName, item, indent + 1, depth - 1, visited);
                idx++;
            }
            if (idx == 0)
                sw.WriteLine($"{ind}    <empty>");
            return;
        }

        // 一般对象：打印类型名，并展开字段/属性
        sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)})");

        // 附加一个 __type 行，展示完整限定名（可选）
        sw.WriteLine($"{ind}    __type: {t.FullName}");

        // Fields（包含 private/public，实例字段）
        var fields = t.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        foreach (var f in fields)
        {
            object val = null;
            try { val = f.GetValue(obj); }
            catch (Exception e) { val = $"<ReadFieldError:{e.GetType().Name}>"; }

            string fieldName = $"{f.Name} ({TypeNameFriendly(f.FieldType)})";
            WriteObjectTree(sw, fieldName, val, indent + 1, depth - 1, visited);
        }

        // Properties（只读 get，跳过索引器）
        var props = t.GetProperties(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
        foreach (var p in props)
        {
            if (p.GetIndexParameters().Length > 0) continue;
            if (!p.CanRead) continue;

            object val = null;
            try { val = p.GetValue(obj, null); }
            catch (Exception e) { val = $"<ReadPropError:{e.GetType().Name}>"; }

            string propName = $"{p.Name} ({TypeNameFriendly(p.PropertyType)})";
            WriteObjectTree(sw, propName, val, indent + 1, depth - 1, visited);
        }
    }

    private static bool IsSimpleLeaf(Type t)
    {
        return t.IsPrimitive
            || t.IsEnum
            || t == typeof(string)
            || t == typeof(decimal)
            || t == typeof(double)
            || t == typeof(float)
            || t == typeof(bool);
    }

    private static string SafeToString(object obj)
    {
        if (obj == null) return "<null>";
        if (obj is IFormattable f)
        {
            // 保持数值输出友好
            try { return f.ToString("G6", System.Globalization.CultureInfo.InvariantCulture); }
            catch { /* ignore */ }
        }
        return obj.ToString();
    }

    private static string TypeNameFriendly(Type t)
    {
        if (t == null) return "null";
        if (t.IsArray) return $"{TypeNameFriendly(t.GetElementType())}[]";
        return t.Name;
    }

    private class RefComparer : IEqualityComparer<object>
    {
        bool IEqualityComparer<object>.Equals(object x, object y) => ReferenceEquals(x, y);
        int IEqualityComparer<object>.GetHashCode(object obj) => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(obj);
    }
}

//using System;
//using System.Collections.Generic;
//using System.IO;
//using System.Reflection;
//using UnityEngine;
//using Unity.XR.PXR;

//public class PXR_BodyTrackingFeed : MonoBehaviour
//{
//    [Header("Output / Coordinates")]
//    public bool outputRotation = true;          // 只要位置可关掉
//    public bool useLocalCoords = true;          // true: 使用 SDK 的 localPose；false: 转到世界坐标（以 cubesRoot 为参考）

//    [Header("Cubes Visualization")]
//    public bool showCubes = true;
//    public Transform cubesRoot;                 // 可选：作为可视化父节点
//    public GameObject cubePrefab;               // 可选：自定义方块
//    public Vector3 cubeScale = new Vector3(0.04f, 0.04f, 0.04f);

//    [Header("Tracking")]
//    public bool autoStartTracking = true;
//    public BodyJointSet jointSetToStart = BodyJointSet.BODY_JOINT_SET_BODY_FULL_START;

//    [Header("TXT Dump (Tree with Types)")]
//    public bool autoDumpFirstValidFrame = true;   // 第一次拿到有效数据就自动Dump
//    public KeyCode dumpHotkey = KeyCode.F9;       // 手动Dump热键（可改为按钮/手柄）
//    public string dumpFileName = "bd_dump";       // 基础文件名（自动加时间戳）
//    public int dumpMaxDepth = 8;                  // 反射最大深度，避免意外递归
//    private bool hasDumpedOnce = false;

//    private bool supportedBT = false;
//    private bool updateBT = false;
//    private bool isTracking = false;
//    private BodyTrackingStatus bs = new BodyTrackingStatus();
//    private BodyTrackingGetDataInfo bdi = new BodyTrackingGetDataInfo();
//    private BodyTrackingData bd = new BodyTrackingData();

//    // 公开给外部消费（例如你的 SMPL-X 管线）
//    public struct RolePose
//    {
//        public Vector3 pos;        // meters, PICO tracking local space
//        public Quaternion rot;     // local space
//        public bool valid;
//    }
//    public RolePose[] LatestRolePoses { get; private set; }

//    // 关节小方块（仅用于可视化）
//    private readonly Dictionary<int, Transform> cubeByIndex = new Dictionary<int, Transform>();

//    void Start()
//    {
//        LatestRolePoses = new RolePose[(int)BodyTrackerRole.ROLE_NUM];
//        if (autoStartTracking) StartBodyTracking();
//    }

//    void OnDestroy()
//    {
//        StopBodyTracking();
//    }

//#if UNITY_ANDROID
//    void Update()
//    {
//        if (!updateBT) return;

//        // 热键手动Dump（真机可改为UI按钮或手柄映射）
//        if (Input.GetKeyDown(dumpHotkey))
//        {
//            TryDumpBDTxt("manual");
//        }

//        PXR_MotionTracking.GetBodyTrackingState(ref isTracking, ref bs);
//        if (bs.stateCode != BodyTrackingStatusCode.BT_VALID)
//            return; // 未校准/未就绪，不更新

//        int ret = PXR_MotionTracking.GetBodyTrackingData(ref bdi, ref bd);
//        if (ret != 0) return;

//        int n = Math.Min(LatestRolePoses.Length, bd.roleDatas.Length);
//        for (int i = 0; i < n; i++)
//        {
//            var pose = bd.roleDatas[i].localPose; // SDK给的局部位姿
//            var p = new Vector3((float)pose.PosX, (float)pose.PosY, (float)pose.PosZ);
//            var q = new Quaternion((float)pose.RotQx, (float)pose.RotQy, (float)pose.RotQz, (float)pose.RotQw);

//            LatestRolePoses[i] = new RolePose { pos = p, rot = q, valid = true };

//            // 可视化（可选）
//            if (showCubes)
//            {
//                var cube = GetOrCreateCube(i);
//                if (useLocalCoords)
//                {
//                    cube.localPosition = p;
//                    if (outputRotation) cube.localRotation = q;
//                }
//                else
//                {
//                    if (cubesRoot != null)
//                    {
//                        cube.position = cubesRoot.TransformPoint(p);
//                        if (outputRotation) cube.rotation = cubesRoot.rotation * q;
//                    }
//                    else
//                    {
//                        cube.position = p; // 仅调试
//                        if (outputRotation) cube.rotation = q;
//                    }
//                }
//            }
//        }

//        // 首帧有效数据自动Dump
//        if (autoDumpFirstValidFrame && !hasDumpedOnce)
//        {
//            TryDumpBDTxt("first_valid_frame");
//            hasDumpedOnce = true;
//        }

//        // TODO: 这里可把 LatestRolePoses 传给你的 SMPL-X 管线
//        // e.g., MySmplxConsumer.UpdateFromJoints(LatestRolePoses);
//    }
//#endif

//    public void StartBodyTracking()
//    {
//        PXR_MotionTracking.GetBodyTrackingSupported(ref supportedBT);
//        if (!supportedBT)
//        {
//            Debug.LogWarning("[PXR_BodyTrackingFeed] Body tracking not supported on this device.");
//            return;
//        }

//        BodyTrackingBoneLength bones = new BodyTrackingBoneLength(); // 不用骨长就默认
//        PXR_MotionTracking.StartBodyTracking(jointSetToStart, bones);

//        PXR_MotionTracking.GetBodyTrackingState(ref isTracking, ref bs);
//        if (bs.stateCode != BodyTrackingStatusCode.BT_VALID &&
//            (bs.message == BodyTrackingMessage.BT_MESSAGE_TRACKER_NOT_CALIBRATED ||
//             bs.message == BodyTrackingMessage.BT_MESSAGE_UNKNOWN))
//        {
//            Debug.Log("[PXR_BodyTrackingFeed] Launching calibration app...");
//            PXR_MotionTracking.StartMotionTrackerCalibApp();
//        }

//        updateBT = true;
//    }

//    public void StopBodyTracking()
//    {
//        updateBT = false;
//        PXR_MotionTracking.StopBodyTracking();
//    }

//    private Transform GetOrCreateCube(int index)
//    {
//        if (cubeByIndex.TryGetValue(index, out var t) && t != null) return t;

//        GameObject go;
//        if (cubePrefab != null)
//            go = Instantiate(cubePrefab, cubesRoot != null ? cubesRoot : transform);
//        else
//            go = GameObject.CreatePrimitive(PrimitiveType.Cube);

//        go.name = ((BodyTrackerRole)index).ToString();
//        t = go.transform;
//        if (cubesRoot != null) t.SetParent(cubesRoot, worldPositionStays: false);
//        t.localScale = cubeScale;

//        cubeByIndex[index] = t;
//        return t;
//    }

//    // ------------------- TXT Dump（树状 + 类型信息） -------------------

//    private void TryDumpBDTxt(string tag)
//    {
//        try
//        {
//            string time = DateTime.Now.ToString("yyyyMMdd_HHmmss");
//            string file = $"{dumpFileName}_{tag}_{time}.txt";
//            string path = Path.Combine(Application.persistentDataPath, file);

//            using (var sw = new StreamWriter(path, false))
//            {
//                var visited = new HashSet<object>(new RefComparer());
//                WriteObjectTree(sw, "bd", bd, 0, dumpMaxDepth, visited);
//            }

//            Debug.Log($"[PXR_BodyTrackingFeed] BodyTrackingData dumped to: {path}");
//        }
//        catch (Exception ex)
//        {
//            Debug.LogError($"[PXR_BodyTrackingFeed] Dump failed: {ex}");
//        }
//    }

//    /// <summary>
//    /// 把任意对象以树状结构写出到文本，包含类型信息；递归字段与属性（含私有）。
//    /// </summary>
//    private void WriteObjectTree(TextWriter sw, string name, object obj, int indent, int depth, HashSet<object> visited)
//    {
//        string ind = new string(' ', indent * 4);

//        if (obj == null)
//        {
//            sw.WriteLine($"{ind}{name} (null)");
//            return;
//        }

//        Type t = obj.GetType();

//        // 简单类型：值类型、字符串、枚举、decimal/double/float/bool
//        if (IsSimpleLeaf(t))
//        {
//            sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}): {SafeToString(obj)}");
//            return;
//        }

//        // 避免循环引用（仅对引用类型）
//        if (!t.IsValueType)
//        {
//            if (visited.Contains(obj))
//            {
//                sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}): <CyclicRef>");
//                return;
//            }
//            visited.Add(obj);
//        }

//        // 达到最大深度
//        if (depth <= 0)
//        {
//            sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}): <MaxDepthReached>");
//            return;
//        }

//        // 集合/数组
//        if (typeof(System.Collections.IEnumerable).IsAssignableFrom(t) && t != typeof(string))
//        {
//            sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)}):");
//            int idx = 0;
//            foreach (var item in (System.Collections.IEnumerable)obj)
//            {
//                string childName = $"[{idx}] ({TypeNameFriendly(item?.GetType())})";
//                WriteObjectTree(sw, childName, item, indent + 1, depth - 1, visited);
//                idx++;
//            }
//            if (idx == 0)
//                sw.WriteLine($"{ind}    <empty>");
//            return;
//        }

//        // 一般对象：打印类型名，并展开字段/属性
//        sw.WriteLine($"{ind}{name} ({TypeNameFriendly(t)})");

//        // 附加一个 __type 行，展示完整限定名（可选）
//        sw.WriteLine($"{ind}    __type: {t.FullName}");

//        // Fields（包含 private/public，实例字段）
//        var fields = t.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
//        foreach (var f in fields)
//        {
//            object val = null;
//            try { val = f.GetValue(obj); }
//            catch (Exception e) { val = $"<ReadFieldError:{e.GetType().Name}>"; }

//            string fieldName = $"{f.Name} ({TypeNameFriendly(f.FieldType)})";
//            WriteObjectTree(sw, fieldName, val, indent + 1, depth - 1, visited);
//        }

//        // Properties（只读 get，跳过索引器）
//        var props = t.GetProperties(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance);
//        foreach (var p in props)
//        {
//            if (p.GetIndexParameters().Length > 0) continue;
//            if (!p.CanRead) continue;

//            object val = null;
//            try { val = p.GetValue(obj, null); }
//            catch (Exception e) { val = $"<ReadPropError:{e.GetType().Name}>"; }

//            string propName = $"{p.Name} ({TypeNameFriendly(p.PropertyType)})";
//            WriteObjectTree(sw, propName, val, indent + 1, depth - 1, visited);
//        }
//    }

//    private static bool IsSimpleLeaf(Type t)
//    {
//        return t.IsPrimitive
//            || t.IsEnum
//            || t == typeof(string)
//            || t == typeof(decimal)
//            || t == typeof(double)
//            || t == typeof(float)
//            || t == typeof(bool);
//    }

//    private static string SafeToString(object obj)
//    {
//        if (obj == null) return "<null>";
//        if (obj is IFormattable f)
//        {
//            // 保持数值输出友好
//            try { return f.ToString("G6", System.Globalization.CultureInfo.InvariantCulture); }
//            catch { /* ignore */ }
//        }
//        return obj.ToString();
//    }

//    private static string TypeNameFriendly(Type t)
//    {
//        if (t == null) return "null";
//        if (t.IsArray) return $"{TypeNameFriendly(t.GetElementType())}[]";
//        return t.Name;
//    }

//    private class RefComparer : IEqualityComparer<object>
//    {
//        bool IEqualityComparer<object>.Equals(object x, object y) => ReferenceEquals(x, y);
//        int IEqualityComparer<object>.GetHashCode(object obj) => System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(obj);
//    }
//}
