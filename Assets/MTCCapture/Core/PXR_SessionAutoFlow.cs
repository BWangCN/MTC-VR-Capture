using System.Collections;
using UnityEngine;

public class PXR_SessionAutoFlow : MonoBehaviour
{
    public enum Embodiment
    {
        None = 0,
        UnitreeG1 = 1,
        // 以后可以加 H1 / H2 ...
    }

    public enum SpawnCorner
    {
        Corner0 = 0,  // 房间原点角 (minX, minZ)
        Corner1 = 1,  // (maxX, minZ)
        Corner2 = 2,  // (maxX, maxZ) — 对角
        Corner3 = 3,  // (minX, maxZ)
        Manual = -1,  // 使用 manualSpawnPos 手动指定
    }

    [Header("Build-time Toggles (set in Inspector before Build)")]
    public bool autoRun = true;                 // 启动后是否自动跑流程
    public bool autoCalibrate = true;           // 启动后是否自动标定
    public bool autoEnterWorld = true;          // 标定后是否自动进入世界(应用缩放并开启世界)
    public bool autoRecord = false;             // 进入世界后是否自动开始录制
    public Embodiment embodiment = Embodiment.UnitreeG1;

    [Header("References")]
    public PXR_24PointMapper mapper;
    public Transform hmd;                       // XR Camera (CenterEye)
    public Transform worldRoot;                 // 环境根：所有物体挂这下面
    public Transform avatarRoot;                // 可选：骨架/可视化根
    public float trackingWarmupSeconds = 1.0f;  // 等追踪稳定的时间
    public bool disableWorldUntilReady = true;  // 启动先隐藏世界，等 EnterWorld 再开

    [Header("Spawn Position")]
    [Tooltip("Manual = 不移动世界，worldRoot 保持 Inspector 中的位置/旋转。\n" +
             "Corner0~3 = 自动检测地板并将玩家出生在对应角落。")]
    public SpawnCorner spawnCorner = SpawnCorner.Manual;

    [Tooltip("离墙壁的安全距离（米），仅 Corner 模式下使用")]
    public float spawnMargin = 0.35f;

    [Header("Calibration")]
    public float calibrationSeconds = 5f;
    [Tooltip("眼睛到头顶的距离（米），用于从 XR Camera 眼高推算总身高")]
    public float eyeToTopOfHeadOffset = 0.12f;

    [Header("Embodiment Params (meters)")]
    public float g1HeightMeters = 1.27f;

    [Header("Scale Pivot")]
    public bool pivotAtHMD = true;
    public Transform customPivot;

    [Header("Auto-align World Forward to HMD")]
    [Tooltip("启用：calibration 后把 worldRoot 绕玩家脚下点旋转，让其局部 worldForwardLocal 轴对齐 HMD 当前水平朝向。\n" +
             "用于修复 Manual 模式下 PICO 启动朝向 ≠ Unity 世界 +Z 导致 corridor 出现在侧面的问题。\n" +
             "Corner 模式下也会再叠加一次旋转 -- 如果你想保留 ApplySpawnCorner 算出的 corner→center 朝向，把 worldForwardLocal 设成 (0,0,1)。")]
    public bool alignWorldForwardToHmd = true;

    [Tooltip("worldRoot 局部坐标系中 '玩家应看到的正前方' 方向。\n" +
             "新版 MTC corridor FBX 已把 travel 方向烤成 +Z (env_to_blender_scene 里 -90°Z 旋转),\n" +
             "默认 (0,0,1) 即可。旧版 FBX (travel 方向是 +X) 请改回 (1,0,0)。")]
    public Vector3 worldForwardLocal = new Vector3(0f, 0f, 1f);

    private bool _ran = false;

    void Start()
    {
        if (!autoRun) return;
        if (_ran) return;
        _ran = true;

        if (disableWorldUntilReady)
        {
            if (worldRoot) worldRoot.gameObject.SetActive(false);
            if (avatarRoot) avatarRoot.gameObject.SetActive(false);
        }

        StartCoroutine(RunFlow());
    }

    private IEnumerator RunFlow()
    {
        // 0) 等追踪稳定
        yield return new WaitForSeconds(trackingWarmupSeconds);

        if (mapper == null)
        {
            Debug.LogWarning("[AutoFlow] mapper not assigned, searching scene...");
            mapper = FindObjectOfType<PXR_24PointMapper>();
        }
        if (mapper == null)
        {
            Debug.LogError("[AutoFlow] mapper is null. No PXR_24PointMapper found in scene.");
            yield break;
        }
        if (pivotAtHMD && hmd == null)
        {
            Debug.LogError("[AutoFlow] hmd is null but pivotAtHMD=true. Drag XR Camera here.");
            yield break;
        }

        // 0.5) 将世界移动/旋转，使玩家出生在选定的房间角落，面朝房间内部
        ApplySpawnCorner();

        // 1) 标定（内联为协程，每帧 yield 一次，让 Time.time 推进、mapper.Update 执行）
        float scale = 1f;
        float measuredHeight = 0f;

        if (autoCalibrate)
        {
            float targetHeight = GetTargetHeightMeters();
            if (targetHeight <= 0f)
            {
                Debug.LogError("[AutoFlow] Target height invalid.");
                yield break;
            }

            float t0 = Time.time;
            int count = 0;
            float sum = 0f;

            while (Time.time - t0 < calibrationSeconds)
            {
                // 使用 XR Camera 高度（相对地面）作为身高测量
                float h = hmd.position.y;
                if (h > 0.5f && h < 3.0f)
                {
                    sum += h;
                    count++;
                }
                yield return null;
            }

            if (count == 0)
            {
                Debug.LogError("[AutoFlow] Calibration failed: no valid samples.");
                yield break;
            }

            float avgEyeHeight = sum / count;
            measuredHeight = avgEyeHeight + eyeToTopOfHeadOffset;
            scale = targetHeight / measuredHeight;
            Debug.Log($"[AutoFlow] EyeHeight={avgEyeHeight:F3}m + offset={eyeToTopOfHeadOffset:F3}m = TotalHeight={measuredHeight:F3}m, Scale={scale:F3} for {embodiment}");
        }

        // 1.5) 把 worldRoot 旋转到 HMD 朝向（修复 corridor 总是出现在侧面的问题）
        AlignWorldForwardToHmd();

        // 2) 设置 mapper：实时关节保持真人大小，仅 CSV 导出时缩放到 G1
        Vector3 pivot = GetScalePivotWorld();
        mapper.SetEmbodimentScale(scale);
        mapper.applyEmbodimentScale = false;            // 不缩放实时 WorldJoints24
        mapper.exportEmbodimentScaledPositions = false; // roomSpaceRef.InverseTransformPoint 已通过世界缩放自动处理 G1 比例

        // 自动关联 HMD Camera 用于录制视角
        if (mapper.hmdCamera == null && hmd != null)
        {
            mapper.hmdCamera = hmd;
            Debug.Log($"[AutoFlow] Auto-wired mapper.hmdCamera = {hmd.name}");
        }

        // 3) 进入世界
        // 实时关节保持真人大小，世界放大 1/scale，
        // 视觉上产生"我变矮了"的效果。
        if (autoEnterWorld)
        {
            if (worldRoot)
            {
                float worldScale = 1f / scale;
                ApplyUniformScaleAroundPivot(worldRoot, worldScale, pivot);
            }

            if (worldRoot) worldRoot.gameObject.SetActive(true);
            if (avatarRoot) avatarRoot.gameObject.SetActive(true);
        }

        // 4) 设置 room space reference —— 让 CSV 记录在房间坐标系中
        //    用 worldRoot 而非 FBX 子物体，因为 FBX 子物体有旋转 (270,180,0)，
        //    InverseTransformPoint 会把旋转也算进去导致坐标轴错乱。
        //    worldRoot 只有平移+缩放（无旋转或旋转为 identity），坐标系干净。
        if (worldRoot != null)
        {
            mapper.roomSpaceRef = worldRoot;
            Debug.Log($"[AutoFlow] Room space ref set to worldRoot: {worldRoot.name}");
        }

        // 5) 设置录制文件名前缀（基于当前启用的环境名称 + corner）
        string sceneName = DetectActiveSceneName();
        string cornerTag = spawnCorner == SpawnCorner.Manual
            ? "manual"
            : $"corner{(int)spawnCorner}";
        string prefix = string.IsNullOrEmpty(sceneName)
            ? cornerTag
            : $"{sceneName}_{cornerTag}";
        mapper.SetRecordingPrefix(prefix);
        Debug.Log($"[AutoFlow] Recording prefix: {prefix}");

        // 6) 自动录制
        if (autoRecord)
        {
            mapper.BeginRecording();
            Debug.Log("[AutoFlow] Recording started.");
        }
    }

    private float GetTargetHeightMeters()
    {
        switch (embodiment)
        {
            case Embodiment.UnitreeG1: return g1HeightMeters;
            default: return 0f;
        }
    }

    private Vector3 GetScalePivotWorld()
    {
        // 取 HMD 正下方地面点（y=0），避免围绕眼高缩放导致地板下沉/"飘起来"
        if (pivotAtHMD && hmd != null)
            return new Vector3(hmd.position.x, 0f, hmd.position.z);
        if (customPivot != null) return customPivot.position;
        return Vector3.zero;
    }

    // 围绕 pivot 做均匀缩放，保持 pivot 不动
    private void ApplyUniformScaleAroundPivot(Transform root, float scale, Vector3 pivotWorld)
    {
        if (!root) return;
        Vector3 dir = root.position - pivotWorld;
        root.localScale = root.localScale * scale;
        root.position = pivotWorld + dir * scale;
    }

    // 把 worldRoot 绕玩家脚下点旋转，使其 worldForwardLocal 方向对齐 HMD 当前 yaw。
    // 这样无论 PICO 启动时 HMD 朝哪边，corridor 都会"自动转过来"摆在玩家正前方。
    private void AlignWorldForwardToHmd()
    {
        if (!alignWorldForwardToHmd) return;
        if (worldRoot == null || hmd == null) return;

        // HMD 朝向投影到水平面 (yaw only, 去掉 pitch/roll 影响)
        Vector3 hmdFwd = hmd.forward;
        hmdFwd.y = 0f;
        if (hmdFwd.sqrMagnitude < 1e-4f)
        {
            Debug.LogWarning("[AutoFlow] HMD forward is near-vertical; skipping AlignWorldForwardToHmd.");
            return;
        }
        hmdFwd.Normalize();
        float hmdYawDeg = Mathf.Atan2(hmdFwd.x, hmdFwd.z) * Mathf.Rad2Deg;

        // worldRoot 当前"正前方"在世界坐标系中的方向
        Vector3 localFwd = worldForwardLocal;
        if (localFwd.sqrMagnitude < 1e-6f)
        {
            Debug.LogWarning("[AutoFlow] worldForwardLocal is zero; skipping AlignWorldForwardToHmd.");
            return;
        }
        Vector3 worldFwd = worldRoot.rotation * localFwd.normalized;
        worldFwd.y = 0f;
        if (worldFwd.sqrMagnitude < 1e-4f)
        {
            Debug.LogWarning("[AutoFlow] worldForwardLocal becomes vertical after rotation; skipping AlignWorldForwardToHmd.");
            return;
        }
        worldFwd.Normalize();
        float worldYawDeg = Mathf.Atan2(worldFwd.x, worldFwd.z) * Mathf.Rad2Deg;

        float deltaYawDeg = Mathf.DeltaAngle(worldYawDeg, hmdYawDeg);
        if (Mathf.Abs(deltaYawDeg) < 0.1f)
        {
            Debug.Log($"[AutoFlow] AlignWorldForwardToHmd: already aligned (delta={deltaYawDeg:F2}°), no rotation needed.");
            return;
        }

        // 玩家脚下点作为旋转 pivot
        Vector3 pivot = new Vector3(hmd.position.x, 0f, hmd.position.z);
        Quaternion deltaRot = Quaternion.Euler(0f, deltaYawDeg, 0f);
        Vector3 dirFromPivot = worldRoot.position - pivot;
        worldRoot.rotation = deltaRot * worldRoot.rotation;
        worldRoot.position = pivot + deltaRot * dirFromPivot;

        Debug.Log($"[AutoFlow] AlignWorldForwardToHmd: hmdYaw={hmdYawDeg:F1}°, " +
                  $"worldYawBefore={worldYawDeg:F1}°, delta={deltaYawDeg:F1}° " +
                  $"around pivot=({pivot.x:F2},{pivot.z:F2}). " +
                  $"localForward={localFwd}");
    }

    /// <summary>
    /// 检测 worldRoot 下当前启用的子物体名称作为场景名。
    /// 例如 worldRoot 下有多个场景 prefab (disaster_easy_3x4_000, disaster_hard_4x5_003, ...)，
    /// 只有一个处于 active 状态，返回其名称。
    /// </summary>
    private string DetectActiveSceneName()
    {
        Transform child = FindActiveSceneChild();
        if (child != null)
        {
            Debug.Log($"[AutoFlow] Detected active scene: {child.name}");
            return child.name;
        }

        Debug.LogWarning("[AutoFlow] No active scene child under worldRoot.");
        return "";
    }

    /// <summary>
    /// 在 worldRoot 子物体中找到第一个 active 的"场景"子物体。
    /// 优先选名字以 "corridor_" 开头的(MTC corridor 场景),其次任意 active
    /// 子物体。跳过 avatarRoot 引用以及名字含 "AvatarRoot"/"Avatar" 的非场景
    /// 子物体(它们是骨架可视化,不是采集要标记的环境)。
    /// </summary>
    private Transform FindActiveSceneChild()
    {
        if (worldRoot == null) return null;

        Transform fallback = null;
        for (int i = 0; i < worldRoot.childCount; i++)
        {
            Transform child = worldRoot.GetChild(i);
            if (!child.gameObject.activeSelf) continue;
            // 跳过 avatarRoot 引用 + 任何名字像骨架/可视化的子物体
            if (avatarRoot != null && child == avatarRoot) continue;
            string n = child.name;
            if (n.StartsWith("AvatarRoot", System.StringComparison.OrdinalIgnoreCase) ||
                n.StartsWith("Avatar",     System.StringComparison.OrdinalIgnoreCase) ||
                n.StartsWith("_Trimmer",   System.StringComparison.OrdinalIgnoreCase))
                continue;
            // 优先返回 corridor_* (MTC 走廊场景)
            if (n.StartsWith("corridor_", System.StringComparison.OrdinalIgnoreCase))
                return child;
            // 否则记为 fallback,继续找看有没有 corridor_*
            if (fallback == null) fallback = child;
        }
        return fallback;
    }

    /// <summary>
    /// 自动检测地板尺寸，将 worldRoot 平移+旋转，使玩家出生在选定角落并面朝房间中心。
    ///
    /// 检测方式：在 worldRoot 子树中搜索名字含 "Floor" 的 Renderer，
    /// 用其 AABB bounds 的 XZ 范围作为房间宽(W)和长(L)。
    ///
    /// Room coordinate system (Unity Y-up):
    ///   Corner0 (minX,minZ)───── Corner1 (maxX,minZ)
    ///      │                          │
    ///      │        room center       │
    ///      │                          │
    ///   Corner3 (minX,maxZ)───── Corner2 (maxX,maxZ)
    /// </summary>
    private void ApplySpawnCorner()
    {
        if (spawnCorner == SpawnCorner.Manual)
        {
            Debug.Log("[AutoFlow] SpawnCorner=Manual, worldRoot stays at Inspector position.");
            return;
        }

        if (worldRoot == null)
        {
            Debug.LogWarning("[AutoFlow] worldRoot is null, skipping spawn corner.");
            return;
        }

        // Temporarily enable worldRoot so Renderer.bounds are valid
        // (disabled objects return zero bounds)
        bool wasActive = worldRoot.gameObject.activeSelf;
        if (!wasActive)
            worldRoot.gameObject.SetActive(true);

        // ---- 自动检测地板尺寸 ----
        Bounds floorBounds;
        if (!TryDetectFloorBounds(out floorBounds))
        {
            if (!wasActive) worldRoot.gameObject.SetActive(false);
            Debug.LogError("[AutoFlow] Cannot detect floor bounds. " +
                           "Make sure there is a Renderer with 'Floor' in its name under worldRoot.");
            return;
        }

        // Restore disabled state (autoEnterWorld will re-enable later)
        if (!wasActive)
            worldRoot.gameObject.SetActive(false);

        // 从 bounds 提取地板在 XZ 平面的范围
        float minX = floorBounds.min.x;
        float maxX = floorBounds.max.x;
        float minZ = floorBounds.min.z;
        float maxZ = floorBounds.max.z;
        float W = maxX - minX;
        float L = maxZ - minZ;

        Debug.Log($"[AutoFlow] Detected floor: W={W:F2}m, L={L:F2}m, " +
                  $"bounds X=[{minX:F2},{maxX:F2}], Z=[{minZ:F2},{maxZ:F2}]");

        float m = spawnMargin;

        // 4个角落在世界坐标系中的位置（基于实际地板 bounds）
        Vector3 cornerPos;
        switch (spawnCorner)
        {
            case SpawnCorner.Corner0: cornerPos = new Vector3(minX + m, 0, minZ + m); break;
            case SpawnCorner.Corner1: cornerPos = new Vector3(maxX - m, 0, minZ + m); break;
            case SpawnCorner.Corner2: cornerPos = new Vector3(maxX - m, 0, maxZ - m); break;
            case SpawnCorner.Corner3: cornerPos = new Vector3(minX + m, 0, maxZ - m); break;
            default:                 cornerPos = new Vector3(minX + m, 0, minZ + m); break;
        }

        // 房间中心（bounds 中心的 XZ 投影）
        Vector3 roomCenter = new Vector3(floorBounds.center.x, 0f, floorBounds.center.z);

        // 角落朝向房间中心的方向
        Vector3 dirToCenter = (roomCenter - cornerPos).normalized;

        // Atan2(x, z) 给出该方向在 XZ 平面的 yaw 角
        float yawDeg = Mathf.Atan2(dirToCenter.x, dirToCenter.z) * Mathf.Rad2Deg;

        // 旋转世界（取反，因为是移动世界而非移动玩家）
        Quaternion worldRot = Quaternion.Euler(0f, -yawDeg, 0f);
        worldRoot.rotation = worldRot;

        // 平移世界，使角落对齐到原点（玩家位置）
        worldRoot.position = -(worldRot * cornerPos);

        Debug.Log($"[AutoFlow] Spawn at {spawnCorner}: " +
                  $"room={W:F1}x{L:F1}m, " +
                  $"cornerPos=({cornerPos.x:F2}, {cornerPos.z:F2}), " +
                  $"facing yaw={yawDeg:F1}°, " +
                  $"worldRoot.pos={worldRoot.position}, rot={worldRoot.rotation.eulerAngles}");
    }

    /// <summary>
    /// 在 worldRoot 子树中搜索名字含 "Floor" (不区分大小写) 的 Renderer，
    /// 返回其世界空间 AABB bounds。如果找到多个，取最大面积的那个。
    /// </summary>
    private bool TryDetectFloorBounds(out Bounds bounds)
    {
        bounds = new Bounds();
        if (worldRoot == null) return false;

        Renderer bestRenderer = null;
        float bestArea = 0f;

        // 搜索所有子 Renderer（包括 inactive 的父物体下的）
        var renderers = worldRoot.GetComponentsInChildren<Renderer>(true);
        foreach (var r in renderers)
        {
            // 名字匹配：Floor, floor, FLOOR 等
            if (r.gameObject.name.IndexOf("Floor", System.StringComparison.OrdinalIgnoreCase) < 0 &&
                r.gameObject.name.IndexOf("floor", System.StringComparison.OrdinalIgnoreCase) < 0)
                continue;

            Bounds b = r.bounds;
            float area = (b.max.x - b.min.x) * (b.max.z - b.min.z);
            if (area > bestArea)
            {
                bestArea = area;
                bestRenderer = r;
                bounds = b;
            }
        }

        if (bestRenderer != null)
        {
            Debug.Log($"[AutoFlow] Found floor: '{bestRenderer.gameObject.name}', " +
                      $"area={bestArea:F2}m²");
            return true;
        }

        // 如果没找到 "Floor"，退而搜索最大水平面积的 Renderer（可能是未命名的地板）
        Debug.LogWarning("[AutoFlow] No 'Floor' named renderer found, searching largest horizontal surface...");
        foreach (var r in renderers)
        {
            Bounds b = r.bounds;
            float height = b.max.y - b.min.y;
            // 地板特征：很薄（Y方向 < 0.2m）且水平面积大
            if (height > 0.2f) continue;

            float area = (b.max.x - b.min.x) * (b.max.z - b.min.z);
            if (area > bestArea)
            {
                bestArea = area;
                bestRenderer = r;
                bounds = b;
            }
        }

        if (bestRenderer != null)
        {
            Debug.Log($"[AutoFlow] Fallback floor: '{bestRenderer.gameObject.name}', " +
                      $"area={bestArea:F2}m²");
            return true;
        }

        return false;
    }
}
