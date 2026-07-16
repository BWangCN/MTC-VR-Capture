using UnityEngine;

/// <summary>
/// G1机器人控制器：将retarget结果应用到ArticulationBody关节
///
/// 使用方法:
/// 1. 拖到G1根节点GameObject上
/// 2. 在Inspector中设置retargetBridge引用
/// 3. 运行时会自动查找并控制29个关节
/// </summary>
public class G1Controller : MonoBehaviour
{
    [Header("References")]
    [Tooltip("拖入PythonRetargetBridge组件")]
    public PythonRetargetBridge retargetBridge;

    [Header("Settings")]
    [Tooltip("关节控制力度（N·m）- 人形机器人需要强驱动")]
    public float jointStiffness = 10000f;  // 提升10倍

    [Tooltip("关节阻尼 - 通常为stiffness的10%")]
    public float jointDamping = 1000f;  // 提升10倍

    [Tooltip("关节力矩限制（N·m）- 足够移动大腿等重部件")]
    public float jointForceLimit = 5000f;  // 提升到5000

    [Header("Per-Joint Settings")]
    [Tooltip("下肢关节使用更低的stiffness（避免过度僵硬）")]
    public float lowerBodyStiffnessMultiplier = 0.3f;  // 下肢用30%的stiffness

    [Tooltip("上肢关节stiffness倍率")]
    public float upperBodyStiffnessMultiplier = 1.0f;  // 上肢保持100%

    [Header("Status - READ ONLY")]
    public bool isReady = false;
    public int jointsFound = 0;

    [Header("Debug")]
    public bool showDebugValues = true;
    public int debugJointIndex = 3; // 左膝盖，观察这个关节
    public bool invertAllJoints = false; // 测试：反转所有关节角度
    public bool debugAllJoints = false; // 打印所有关节的值（每60帧）

    [Header("Coordinate Adjustment")]
    [Tooltip("手动旋转偏移（Euler角度）- 用于调整MuJoCo→Unity坐标系差异")]
    public Vector3 rootRotationOffset = Vector3.zero;  // 在Inspector中调整这个！

    [Tooltip("手动位置偏移 - 用于调整高度等")]
    public Vector3 rootPositionOffset = Vector3.zero;

    // G1的29个关节，按照GMR的顺序
    private readonly string[] jointNames = new string[]
    {
        // 左腿 (0-5)
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",

        // 右腿 (6-11)
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",

        // 腰部 (12-14)
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",

        // 左臂 (15-21)
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",

        // 右臂 (22-28)
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint"
    };

    private ArticulationBody[] joints = new ArticulationBody[29];
    private int[] jointAxisIndex = new int[29]; // 0=X, 1=Y, 2=Z
    private ArticulationBody rootBody; // Pelvis root body

    // Joint angle inversion flags (1.0 = normal, -1.0 = inverted)
    // FINAL CONFIG: Left leg hip inverted, right leg normal (URDF asymmetry)
    private readonly float[] jointInversion = new float[29]
    {
        // 左腿 (0-5): hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
        -1f, -1f, -1f, 1f, 1f, 1f,  // Hip inverted (required for correct motion)
        // 右腿 (6-11): hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
        1f, 1f, 1f, 1f, 1f, 1f,  // All normal (correct as-is)
        // 腰部 (12-14)
        1f, 1f, 1f,
        // 左臂 (15-21)
        1f, 1f, 1f, 1f, 1f, 1f, 1f,
        // 右臂 (22-28)
        1f, 1f, 1f, 1f, 1f, 1f, 1f
    };

    void Start()
    {
        // 自动查找retargetBridge（如果未手动设置）
        if (retargetBridge == null)
        {
            retargetBridge = FindObjectOfType<PythonRetargetBridge>();
            if (retargetBridge == null)
            {
                Debug.LogError("[G1Controller] PythonRetargetBridge not found!");
                enabled = false;
                return;
            }
        }

        // 查找root body (pelvis) - 通过名字或作为第一个ArticulationBody
        ArticulationBody[] allBodies = GetComponentsInChildren<ArticulationBody>();
        foreach (var body in allBodies)
        {
            if (body.isRoot || body.name.ToLower().Contains("pelvis"))
            {
                rootBody = body;
                break;
            }
        }

        if (rootBody == null)
        {
            Debug.LogError("[G1Controller] Root ArticulationBody (pelvis) not found!");
            enabled = false;
            return;
        }

        // CRITICAL: Root ArticulationBody MUST be immovable (Unity requirement)
        // Making it movable will crash Unity physics engine!
        rootBody.immovable = true;
        Debug.Log($"[G1Controller] Found root body: {rootBody.name}, keeping IMMOVABLE (required for ArticulationBody root)");

        // 查找所有关节的ArticulationBody
        FindJoints();

        if (jointsFound != 29)
        {
            Debug.LogWarning($"[G1Controller] Only found {jointsFound}/29 joints. Some joints may not be controlled.");
        }
        else
        {
            Debug.Log("[G1Controller] All 29 joints found successfully!");
        }

        // 配置关节驱动参数
        ConfigureJointDrives();

        isReady = true;
    }

    void FindJoints()
    {
        // 获取G1下的所有ArticulationBody
        ArticulationBody[] allBodies = GetComponentsInChildren<ArticulationBody>();

        Debug.Log($"[G1Controller] Found {allBodies.Length} ArticulationBody components in total");

        // 关节名到link名的映射（特殊情况）
        var jointToLink = new System.Collections.Generic.Dictionary<string, string>
        {
            { "waist_pitch_joint", "torso_link" }
            // 其他大部分是标准的 _joint -> _link 替换
        };

        // 为每个关节名称查找对应的ArticulationBody
        for (int i = 0; i < jointNames.Length; i++)
        {
            string jointName = jointNames[i];
            string linkName;

            // 检查是否有特殊映射
            if (jointToLink.ContainsKey(jointName))
            {
                linkName = jointToLink[jointName];
            }
            else
            {
                // 默认：_joint -> _link
                linkName = jointName.Replace("_joint", "_link");
            }

            // 在所有ArticulationBody中查找名称匹配的
            foreach (var body in allBodies)
            {
                if (body.name == linkName)
                {
                    joints[i] = body;
                    jointsFound++;
                    break;
                }
            }

            if (joints[i] == null)
            {
                Debug.LogWarning($"[G1Controller] Joint {i} ({jointName}) -> link '{linkName}' not found!");
            }
            else
            {
                // 检测这个关节使用的旋转轴
                jointAxisIndex[i] = DetectJointAxis(joints[i]);

                // DEBUG: 对比左右腿关节轴（检查对称性）
                if (i >= 0 && i <= 5)  // Left leg
                {
                    Debug.Log($"[LEFT LEG] Joint {i} ({jointName}): Axis={jointAxisIndex[i]} (0=X, 1=Y, 2=Z)");
                }
                else if (i >= 6 && i <= 11)  // Right leg
                {
                    Debug.Log($"[RIGHT LEG] Joint {i} ({jointName}): Axis={jointAxisIndex[i]} (0=X, 1=Y, 2=Z)");
                }
            }
        }

        // DIAGNOSTIC: 对比左右腿关节轴配置
        Debug.Log("=== LEFT vs RIGHT LEG AXIS COMPARISON ===");
        for (int i = 0; i < 6; i++)
        {
            string leftName = jointNames[i];
            string rightName = jointNames[i + 6];
            int leftAxis = jointAxisIndex[i];
            int rightAxis = jointAxisIndex[i + 6];
            string symmetric = (leftAxis == rightAxis) ? "SYMMETRIC" : "ASYMMETRIC!";
            Debug.Log($"  {leftName} (axis {leftAxis}) vs {rightName} (axis {rightAxis}) → {symmetric}");
        }
    }

    int DetectJointAxis(ArticulationBody body)
    {
        // 检查哪个轴的motion是Free（可旋转）
        if (body.xDrive.lowerLimit != 0 || body.xDrive.upperLimit != 0 ||
            body.twistLock == ArticulationDofLock.FreeMotion)
            return 0; // X轴

        if (body.yDrive.lowerLimit != 0 || body.yDrive.upperLimit != 0 ||
            body.swingYLock == ArticulationDofLock.FreeMotion)
            return 1; // Y轴

        if (body.zDrive.lowerLimit != 0 || body.zDrive.upperLimit != 0 ||
            body.swingZLock == ArticulationDofLock.FreeMotion)
            return 2; // Z轴

        return 0; // 默认X轴
    }

    void ConfigureJointDrives()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null) continue;

            // 区分下肢（0-11）和上肢（12-28）使用不同的stiffness
            // Lower body (legs): indices 0-11
            // Upper body (waist, arms): indices 12-28
            bool isLowerBody = (i < 12);
            float stiffnessMultiplier = isLowerBody ? lowerBodyStiffnessMultiplier : upperBodyStiffnessMultiplier;
            float effectiveStiffness = jointStiffness * stiffnessMultiplier;
            float effectiveDamping = jointDamping * stiffnessMultiplier;

            // 根据关节轴配置对应的drive
            ArticulationDrive drive;

            switch (jointAxisIndex[i])
            {
                case 0: // X轴
                    drive = joints[i].xDrive;
                    drive.stiffness = effectiveStiffness;
                    drive.damping = effectiveDamping;
                    drive.forceLimit = jointForceLimit;
                    joints[i].xDrive = drive;
                    break;
                case 1: // Y轴
                    drive = joints[i].yDrive;
                    drive.stiffness = effectiveStiffness;
                    drive.damping = effectiveDamping;
                    drive.forceLimit = jointForceLimit;
                    joints[i].yDrive = drive;
                    break;
                case 2: // Z轴
                    drive = joints[i].zDrive;
                    drive.stiffness = effectiveStiffness;
                    drive.damping = effectiveDamping;
                    drive.forceLimit = jointForceLimit;
                    joints[i].zDrive = drive;
                    break;
            }
        }

        Debug.Log($"[G1Controller] Configured drives for {jointsFound} joints (base stiffness={jointStiffness}, damping={jointDamping}, lower body multiplier={lowerBodyStiffnessMultiplier})");

        // 输出每个关节的轴配置（调试用）
        if (showDebugValues)
        {
            System.Text.StringBuilder sb = new System.Text.StringBuilder();
            sb.AppendLine("[G1Controller] Joint axis configuration:");
            for (int i = 0; i < joints.Length; i++)
            {
                if (joints[i] != null)
                {
                    string axisName = jointAxisIndex[i] == 0 ? "X" : (jointAxisIndex[i] == 1 ? "Y" : "Z");
                    sb.AppendLine($"  [{i}] {jointNames[i]} → {axisName} axis");
                }
            }
            Debug.Log(sb.ToString());
        }
    }

    void Update()
    {
        if (!isReady) return;
        if (retargetBridge == null) return;

        // 从retargetBridge获取root pose和29个关节角度
        Vector3 rootPos = retargetBridge.rootPos;
        Vector4 rootRotWxyz = retargetBridge.rootRotWxyz;
        float[] dofPos = retargetBridge.dofPos;

        if (dofPos == null || dofPos.Length < 29)
        {
            return;
        }

        // 应用root position和rotation到pelvis
        if (rootBody != null)
        {
            // MuJoCo wxyz quaternion → Unity xyzw quaternion
            Quaternion rootRot = new Quaternion(rootRotWxyz.y, rootRotWxyz.z, rootRotWxyz.w, rootRotWxyz.x);

            // 应用手动旋转偏移（用于调整坐标系差异）
            if (rootRotationOffset != Vector3.zero)
            {
                Quaternion offsetRot = Quaternion.Euler(rootRotationOffset);
                rootRot = offsetRot * rootRot;
            }

            // 应用手动位置偏移
            Vector3 finalPos = rootPos + rootPositionOffset;

            // IMPORTANT: For immovable root, use direct transform manipulation
            // TeleportRoot is only for movable ArticulationBodies and will crash on immovable!
            rootBody.transform.position = finalPos;
            rootBody.transform.rotation = rootRot;
        }

        // 调试：每60帧打印一次某个关节的值（目标 vs 当前）
        if (showDebugValues && Time.frameCount % 60 == 0)
        {
            int idx = debugJointIndex;
            if (joints[idx] != null)
            {
                string axisName = jointAxisIndex[idx] == 0 ? "X" : (jointAxisIndex[idx] == 1 ? "Y" : "Z");
                float targetDeg = dofPos[idx] * Mathf.Rad2Deg;

                // 获取当前关节角度
                float currentDeg = 0f;
                switch (jointAxisIndex[idx])
                {
                    case 0: currentDeg = joints[idx].xDrive.target; break;
                    case 1: currentDeg = joints[idx].yDrive.target; break;
                    case 2: currentDeg = joints[idx].zDrive.target; break;
                }

                // 获取实际物理状态（关节位置）
                float actualPos = 0f;
                if (joints[idx].jointType == ArticulationJointType.RevoluteJoint)
                {
                    actualPos = joints[idx].jointPosition[0] * Mathf.Rad2Deg;
                }

                Debug.Log($"[G1Controller DEBUG] Joint {idx} ({jointNames[idx]}): " +
                          $"Axis={axisName}, Target={targetDeg:F1}°, DriveTarget={currentDeg:F1}°, ActualPos={actualPos:F1}°");
            }
        }

        // 应用到每个关节
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null) continue;

            // 将弧度转换为度数，并应用per-joint反转
            float targetDegrees = dofPos[i] * Mathf.Rad2Deg;

            // 应用关节特定的反转（修复左右腿不对称问题）
            targetDegrees *= jointInversion[i];

            // 全局反转开关（调试用）
            if (invertAllJoints) targetDegrees = -targetDegrees;

            // 根据关节轴设置对应的drive target
            ArticulationDrive drive;
            switch (jointAxisIndex[i])
            {
                case 0: // X轴
                    drive = joints[i].xDrive;
                    drive.target = targetDegrees;
                    joints[i].xDrive = drive;
                    break;
                case 1: // Y轴
                    drive = joints[i].yDrive;
                    drive.target = targetDegrees;
                    joints[i].yDrive = drive;
                    break;
                case 2: // Z轴
                    drive = joints[i].zDrive;
                    drive.target = targetDegrees;
                    joints[i].zDrive = drive;
                    break;
            }
        }
    }

    void OnGUI()
    {
        if (!enabled) return;

        GUI.color = isReady ? Color.green : Color.yellow;
        GUI.Label(new Rect(10, 160, 400, 30), $"[G1Controller] Joints: {jointsFound}/29, Ready: {isReady}");

        if (isReady && retargetBridge != null)
        {
            GUI.color = Color.cyan;
            GUI.Label(new Rect(10, 190, 400, 30), $"Applying retarget @ {retargetBridge.retargetFPS:F1} FPS");
        }
    }
}
