using UnityEngine;

/// <summary>
/// PC端：直接将BodyDataReceiver接收的24点骨骼数据映射到G1模型
/// 实现100%几何贴合，用于碰撞感知
///
/// 数据源：BodyDataReceiver（接收PICO发来的UDP数据）
/// </summary>
public class G1DirectMapper_PC : MonoBehaviour
{
    [Header("Data Source")]
    [Tooltip("拖入场景中的BodyDataReceiver组件")]
    public BodyDataReceiver bodyReceiver;

    [Header("G1 Model References")]
    [Tooltip("拖入G1模型的根节点（通常是pelvis）")]
    public Transform g1Root;

    [Header("Body Scaling")]
    [Tooltip("自动缩放VR数据以匹配G1身高（127cm）")]
    public bool autoScale = true;

    [Tooltip("G1的标准身高（米）")]
    public float g1Height = 1.27f;

    [Tooltip("手动缩放因子（如果autoScale=false）")]
    public float manualScaleFactor = 1.0f;

    [Header("Foot Control")]
    [Tooltip("脚部控制模式")]
    public FootMode footMode = FootMode.NaturalOrientation;

    [Tooltip("地面高度（用于平行地面模式）")]
    public float groundHeight = 0f;

    [Header("Debug")]
    public bool showDebugInfo = false;

    public enum FootMode
    {
        AlwaysFlat,           // 始终平行地面（只保留Y轴旋转）
        NaturalOrientation,   // 根据脚踝-脚尖方向自然旋转（推荐）
        FollowVRRotation      // 完全跟随VR旋转数据
    }

    // G1骨骼引用（自动查找）
    private Transform g1_pelvis;
    private Transform g1_spine1, g1_spine2, g1_spine3;
    private Transform g1_neck, g1_head;

    private Transform g1_left_hip, g1_left_knee, g1_left_ankle, g1_left_foot;
    private Transform g1_right_hip, g1_right_knee, g1_right_ankle, g1_right_foot;

    private Transform g1_left_collar, g1_left_shoulder, g1_left_elbow, g1_left_wrist, g1_left_hand;
    private Transform g1_right_collar, g1_right_shoulder, g1_right_elbow, g1_right_wrist, g1_right_hand;

    private bool isInitialized = false;
    private float scaleFactor = 1.0f;
    private bool scaleCalculated = false;
    private float groundLevel = 0f;  // 脚底高度（作为缩放基准）

    void Start()
    {
        if (bodyReceiver == null)
        {
            Debug.LogError("[G1DirectMapper_PC] BodyDataReceiver not assigned!");
            enabled = false;
            return;
        }

        if (g1Root == null)
        {
            Debug.LogError("[G1DirectMapper_PC] G1 Root not assigned!");
            enabled = false;
            return;
        }

        InitializeG1Skeleton();
    }

    void InitializeG1Skeleton()
    {
        // 自动查找G1骨骼（基于Unity URDF导入的命名）
        g1_pelvis = FindChildRecursive(g1Root, "pelvis") ?? g1Root;

        // Spine chain
        g1_spine1 = FindChildRecursive(g1Root, "torso_link");
        g1_spine2 = g1_spine1; // G1可能只有一个torso
        g1_spine3 = g1_spine1;
        g1_neck = FindChildRecursive(g1Root, "head_link");
        g1_head = g1_neck; // 头和脖子可能是同一个

        // Left leg
        g1_left_hip = FindChildRecursive(g1Root, "left_hip_pitch_link");
        g1_left_knee = FindChildRecursive(g1Root, "left_knee_link");
        g1_left_ankle = FindChildRecursive(g1Root, "left_ankle_pitch_link");
        g1_left_foot = g1_left_ankle; // 脚和踝关节可能是同一个

        // Right leg
        g1_right_hip = FindChildRecursive(g1Root, "right_hip_pitch_link");
        g1_right_knee = FindChildRecursive(g1Root, "right_knee_link");
        g1_right_ankle = FindChildRecursive(g1Root, "right_ankle_pitch_link");
        g1_right_foot = g1_right_ankle;

        // Left arm
        g1_left_collar = g1_spine1; // collar通常连在spine上
        g1_left_shoulder = FindChildRecursive(g1Root, "left_shoulder_pitch_link");
        g1_left_elbow = FindChildRecursive(g1Root, "left_elbow_link");
        g1_left_wrist = FindChildRecursive(g1Root, "left_wrist_roll_link");
        g1_left_hand = g1_left_wrist;

        // Right arm
        g1_right_collar = g1_spine1;
        g1_right_shoulder = FindChildRecursive(g1Root, "right_shoulder_pitch_link");
        g1_right_elbow = FindChildRecursive(g1Root, "right_elbow_link");
        g1_right_wrist = FindChildRecursive(g1Root, "right_wrist_roll_link");
        g1_right_hand = g1_right_wrist;

        isInitialized = true;
        Debug.Log("[G1DirectMapper_PC] G1 skeleton initialized successfully!");
    }

    void LateUpdate()
    {
        if (!isInitialized || bodyReceiver == null || !bodyReceiver.isReceiving)
            return;

        // 检查是否有数据
        if (bodyReceiver.framesReceived == 0)
            return;

        Vector3[] vrJoints = bodyReceiver.receivedJoints;
        Quaternion[] vrRotations = bodyReceiver.receivedRotations;

        // VR关节索引（和PICO端的PositionIndex一致）
        const int VR_PELVIS = 0;
        const int VR_LEFT_HIP = 1, VR_RIGHT_HIP = 2;
        const int VR_SPINE1 = 3;
        const int VR_LEFT_KNEE = 4, VR_RIGHT_KNEE = 5;
        const int VR_SPINE2 = 6;
        const int VR_LEFT_ANKLE = 7, VR_RIGHT_ANKLE = 8;
        const int VR_SPINE3 = 9;
        const int VR_LEFT_FOOT = 10, VR_RIGHT_FOOT = 11;
        const int VR_NECK = 12;
        const int VR_LEFT_COLLAR = 13, VR_RIGHT_COLLAR = 14;
        const int VR_HEAD = 15;
        const int VR_LEFT_SHOULDER = 16, VR_RIGHT_SHOULDER = 17;
        const int VR_LEFT_ELBOW = 18, VR_RIGHT_ELBOW = 19;
        const int VR_LEFT_WRIST = 20, VR_RIGHT_WRIST = 21;
        const int VR_LEFT_HAND = 22, VR_RIGHT_HAND = 23;

        // 自动计算缩放因子（只在第一帧计算）
        if (autoScale && !scaleCalculated)
        {
            CalculateScaleFactor(vrJoints);
        }
        else if (!autoScale)
        {
            scaleFactor = manualScaleFactor;
        }

        // 找到当前帧脚底的最低点（作为缩放基准）
        float leftFootY = vrJoints[VR_LEFT_FOOT].y;
        float rightFootY = vrJoints[VR_RIGHT_FOOT].y;
        groundLevel = Mathf.Min(leftFootY, rightFootY);

        // ===== 核心部分：直接位置匹配 =====

        // Pelvis（根节点）
        SetTransform(g1_pelvis, vrJoints[VR_PELVIS], vrRotations[VR_PELVIS]);

        // Spine chain
        SetTransform(g1_spine1, vrJoints[VR_SPINE1], vrRotations[VR_SPINE1]);
        SetTransform(g1_neck, vrJoints[VR_NECK], vrRotations[VR_NECK]);
        SetTransform(g1_head, vrJoints[VR_HEAD], vrRotations[VR_HEAD]);

        // Left leg
        SetTransform(g1_left_hip, vrJoints[VR_LEFT_HIP], vrRotations[VR_LEFT_HIP]);
        SetTransform(g1_left_knee, vrJoints[VR_LEFT_KNEE], vrRotations[VR_LEFT_KNEE]);
        SetTransform(g1_left_ankle, vrJoints[VR_LEFT_ANKLE], vrRotations[VR_LEFT_ANKLE]);

        // Left foot（特殊处理）
        SetFootTransform(g1_left_foot, vrJoints[VR_LEFT_ANKLE], vrJoints[VR_LEFT_FOOT], vrRotations[VR_LEFT_FOOT]);

        // Right leg
        SetTransform(g1_right_hip, vrJoints[VR_RIGHT_HIP], vrRotations[VR_RIGHT_HIP]);
        SetTransform(g1_right_knee, vrJoints[VR_RIGHT_KNEE], vrRotations[VR_RIGHT_KNEE]);
        SetTransform(g1_right_ankle, vrJoints[VR_RIGHT_ANKLE], vrRotations[VR_RIGHT_ANKLE]);

        // Right foot（特殊处理）
        SetFootTransform(g1_right_foot, vrJoints[VR_RIGHT_ANKLE], vrJoints[VR_RIGHT_FOOT], vrRotations[VR_RIGHT_FOOT]);

        // Left arm
        SetTransform(g1_left_shoulder, vrJoints[VR_LEFT_SHOULDER], vrRotations[VR_LEFT_SHOULDER]);
        SetTransform(g1_left_elbow, vrJoints[VR_LEFT_ELBOW], vrRotations[VR_LEFT_ELBOW]);
        SetTransform(g1_left_wrist, vrJoints[VR_LEFT_WRIST], vrRotations[VR_LEFT_WRIST]);
        SetTransform(g1_left_hand, vrJoints[VR_LEFT_HAND], vrRotations[VR_LEFT_HAND]);

        // Right arm
        SetTransform(g1_right_shoulder, vrJoints[VR_RIGHT_SHOULDER], vrRotations[VR_RIGHT_SHOULDER]);
        SetTransform(g1_right_elbow, vrJoints[VR_RIGHT_ELBOW], vrRotations[VR_RIGHT_ELBOW]);
        SetTransform(g1_right_wrist, vrJoints[VR_RIGHT_WRIST], vrRotations[VR_RIGHT_WRIST]);
        SetTransform(g1_right_hand, vrJoints[VR_RIGHT_HAND], vrRotations[VR_RIGHT_HAND]);
    }

    void CalculateScaleFactor(Vector3[] vrJoints)
    {
        // 计算VR骨骼的身高（脚底到头顶的距离）
        const int VR_HEAD = 15;
        const int VR_LEFT_FOOT = 10;
        const int VR_RIGHT_FOOT = 11;

        Vector3 headPos = vrJoints[VR_HEAD];
        float leftFootY = vrJoints[VR_LEFT_FOOT].y;
        float rightFootY = vrJoints[VR_RIGHT_FOOT].y;
        float footGroundY = Mathf.Min(leftFootY, rightFootY);

        float vrHeight = headPos.y - footGroundY;  // 从脚底到头顶的高度

        if (vrHeight > 0.5f)  // 合理的身高数据（至少50cm）
        {
            scaleFactor = g1Height / vrHeight;
            scaleCalculated = true;

            Debug.Log($"[G1DirectMapper_PC] Auto-scale calculated (ground at y=0):");
            Debug.Log($"  VR Height (ground→head): {vrHeight:F3}m");
            Debug.Log($"  G1 Target Height: {g1Height:F3}m");
            Debug.Log($"  Scale Factor: {scaleFactor:F3}x");
            Debug.Log($"  Foot ground level: {footGroundY:F3}m → scaled to y=0");
        }
        else
        {
            Debug.LogWarning($"[G1DirectMapper_PC] Invalid VR height: {vrHeight:F3}m, using scale=1.0");
            scaleFactor = 1.0f;
        }
    }

    void SetTransform(Transform bone, Vector3 position, Quaternion rotation)
    {
        if (bone == null) return;

        // 以脚底为基准缩放：保持XZ位置，缩放Y高度
        // 脚底(groundLevel) → y=0, 其他关节高度按比例缩放
        float heightAboveGround = position.y - groundLevel;
        float scaledHeight = heightAboveGround * scaleFactor;

        Vector3 scaledPosition = new Vector3(
            position.x,           // X位置不变
            scaledHeight,         // Y高度缩放，脚底对齐y=0
            position.z            // Z位置不变
        );

        bone.position = scaledPosition;
        bone.rotation = rotation;
    }

    void SetFootTransform(Transform footBone, Vector3 anklePos, Vector3 toePos, Quaternion vrRotation)
    {
        if (footBone == null) return;

        // 以脚底为基准缩放脚部位置
        float ankleHeight = anklePos.y - groundLevel;
        float toeHeight = toePos.y - groundLevel;

        Vector3 scaledAnklePos = new Vector3(
            anklePos.x,
            ankleHeight * scaleFactor,  // 脚底对齐y=0
            anklePos.z
        );

        Vector3 scaledToePos = new Vector3(
            toePos.x,
            toeHeight * scaleFactor,
            toePos.z
        );

        footBone.position = scaledAnklePos; // 脚部位置在踝关节

        // 根据模式计算脚部旋转
        Quaternion footRotation;

        switch (footMode)
        {
            case FootMode.AlwaysFlat:
                // 保持平行地面，只保留Y轴旋转
                Vector3 forward = (scaledToePos - scaledAnklePos);
                forward.y = 0; // 压平到水平面
                if (forward.sqrMagnitude > 0.001f)
                {
                    footRotation = Quaternion.LookRotation(forward.normalized, Vector3.up);
                }
                else
                {
                    footRotation = Quaternion.identity;
                }
                break;

            case FootMode.NaturalOrientation:
                // 根据踝-趾方向自然旋转（推荐）
                Vector3 footForward = (scaledToePos - scaledAnklePos);
                if (footForward.sqrMagnitude > 0.001f)
                {
                    // 使用脚踝到脚尖的方向作为forward
                    // up保持接近世界up，但允许pitch变化
                    footRotation = Quaternion.LookRotation(footForward.normalized, Vector3.up);
                }
                else
                {
                    footRotation = vrRotation;
                }
                break;

            case FootMode.FollowVRRotation:
                // 完全跟随VR数据
                footRotation = vrRotation;
                break;

            default:
                footRotation = vrRotation;
                break;
        }

        footBone.rotation = footRotation;
    }

    Transform FindChildRecursive(Transform parent, string name)
    {
        if (parent == null) return null;

        // 检查当前节点
        if (parent.name.Contains(name))
            return parent;

        // 递归检查所有子节点
        foreach (Transform child in parent)
        {
            Transform result = FindChildRecursive(child, name);
            if (result != null)
                return result;
        }

        return null;
    }

    void OnGUI()
    {
        if (!showDebugInfo || !isInitialized) return;

        GUI.color = Color.green;
        GUI.Label(new Rect(10, 100, 400, 30), $"[G1DirectMapper_PC] Active - Foot Mode: {footMode}");

        if (bodyReceiver != null && bodyReceiver.isReceiving)
        {
            GUI.Label(new Rect(10, 130, 400, 30), $"Mapping @ {bodyReceiver.receivedFPS:F1} FPS");

            if (scaleCalculated)
            {
                GUI.Label(new Rect(10, 160, 400, 30), $"Scale Factor: {scaleFactor:F3} (VR→G1 {g1Height:F2}m)");
            }
        }
        else
        {
            GUI.color = Color.yellow;
            GUI.Label(new Rect(10, 130, 400, 30), "Waiting for PICO data...");
        }
    }
}
