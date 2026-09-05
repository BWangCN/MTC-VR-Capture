using System;
using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class PXR_SessionControlPanel : MonoBehaviour
{
    public enum SessionPhase
    {
        UIOnly,
        Calibrating,
        ReadyToEnterWorld,
        InWorld
    }

    [Header("References")]
    public PXR_24PointMapper mapper;         // 你的 24 点 mapper
    public Transform hmd;                   // 一般拖 XR Camera（CenterEye）或 Camera.main.transform
    public Transform worldRoot;             // 环境根（要缩放）
    public Transform avatarRoot;            // 骨架/火柴人根（要缩放）
    public GameObject uiRoot;               // UI 根（不要缩放）
    public UIFollowHMD uiFollow;            // 可选：UI 跟随脚本（UIOnly 阶段打开）

    [Header("UI Panels (optional)")]
    public GameObject panelMain;            // 主面板（Calibrate/Embodiment）
    public GameObject panelInWorld;         // 世界内小面板（Start/Stop Recording）

    [Header("UI Widgets (optional)")]
    public Text statusText;                 // 状态提示
    public Dropdown embodimentDropdown;     // 目前只有 G1，但留接口
    public Button btnCalibrate;
    public Button btnEnterWorld;
    public Button btnStartRec;
    public Button btnStopRec;
    public Button btnBackToUI;

    [Header("Calibration")]
    [Tooltip("站直采样秒数")]
    public float calibrationSeconds = 5f;

    [Tooltip("机器人目标身高（米，地板到头顶、腿伸直）。G1=1.32m")]
    public float targetRobotHeightMeters = 1.32f;

    [Tooltip("从 24 点中选 head / left_ankle(or left_foot_index) / right_ankle(or right_foot_index) 的索引")]
    public int headIndex = (int)PositionIndex.head;
    public int leftFootIndex = (int)PositionIndex.left_ankle;   // 你可改成 left_foot_index
    public int rightFootIndex = (int)PositionIndex.right_ankle; // 你可改成 right_foot_index

    [Header("World Scale Settings")]
    [Tooltip("缩放枢轴使用 HMD 位置（推荐）。这样缩放后眼睛仍与 XR Origin 高度齐平（相机不动）。")]
    public bool pivotAtHMD = true;

    [Tooltip("如果不使用 HMD pivot，则用这个 pivot（世界坐标）")]
    public Transform customPivot;

    [Header("Recording Control")]
    public bool autoHideMainPanelWhenEnterWorld = true;

    private SessionPhase _phase = SessionPhase.UIOnly;
    private float _calibratedScale = 1f;
    private float _measuredHeightAvg = 0f;

    void Start()
    {
        EnterPhase(SessionPhase.UIOnly);
        HookUiIfProvided();
        RefreshUi();
    }

    private void HookUiIfProvided()
    {
        if (btnCalibrate) btnCalibrate.onClick.AddListener(OnClickCalibrate);
        if (btnEnterWorld) btnEnterWorld.onClick.AddListener(OnClickEnterWorld);
        if (btnStartRec) btnStartRec.onClick.AddListener(OnClickStartRecording);
        if (btnStopRec) btnStopRec.onClick.AddListener(OnClickStopRecording);
        if (btnBackToUI) btnBackToUI.onClick.AddListener(OnClickBackToUI);
    }

    private void EnterPhase(SessionPhase phase)
    {
        _phase = phase;

        // 先把世界关掉，避免你说的“先渲染世界再 UI”
        if (worldRoot) worldRoot.gameObject.SetActive(phase == SessionPhase.InWorld);
        if (avatarRoot) avatarRoot.gameObject.SetActive(phase == SessionPhase.InWorld);

        if (panelMain) panelMain.SetActive(phase != SessionPhase.InWorld);
        if (panelInWorld) panelInWorld.SetActive(phase == SessionPhase.InWorld);

        // UI 跟随只在 UI 阶段开启，进世界后可以关掉（你也可以保持开启做悬浮菜单）
        if (uiFollow) uiFollow.enabled = (phase != SessionPhase.InWorld);

        // 录制默认关
        if (phase == SessionPhase.UIOnly)
        {
            if (mapper) mapper.EndRecording();
        }

        RefreshUi();
    }

    private void RefreshUi()
    {
        if (statusText)
        {
            statusText.text =
                $"Phase: {_phase}\n" +
                $"Measured Height Avg (m): {_measuredHeightAvg:F3}\n" +
                $"Scale (robot/human): {_calibratedScale:F3}\n" +
                $"Target Robot Height (m): {targetRobotHeightMeters:F3}\n";
        }

        if (btnEnterWorld) btnEnterWorld.interactable = (_phase == SessionPhase.ReadyToEnterWorld);
        if (btnStartRec) btnStartRec.interactable = (_phase == SessionPhase.InWorld);
        if (btnStopRec) btnStopRec.interactable = (_phase == SessionPhase.InWorld);
    }

    public void OnClickCalibrate()
    {
        if (_phase == SessionPhase.Calibrating) return;
        if (mapper == null)
        {
            Debug.LogError("[Session] mapper is null.");
            return;
        }
        if (hmd == null && pivotAtHMD)
        {
            Debug.LogError("[Session] hmd is null but pivotAtHMD=true. Drag XR Camera (CenterEye) here.");
            return;
        }
        StartCoroutine(CalibrationCoroutine());
    }

    private IEnumerator CalibrationCoroutine()
    {
        EnterPhase(SessionPhase.Calibrating);
        if (statusText) statusText.text = "Calibrating... Stand straight for 5 seconds.";

        float t0 = Time.time;
        int count = 0;
        float sum = 0f;

        while (Time.time - t0 < calibrationSeconds)
        {
            // mapper 的 WorldJoints24 会在 Update 内刷新
            var pts = mapper.WorldJoints24;
            if (pts != null && pts.Length >= 24)
            {
                Vector3 head = pts[headIndex];
                Vector3 footL = pts[leftFootIndex];
                Vector3 footR = pts[rightFootIndex];
                Vector3 footMid = 0.5f * (footL + footR);

                // 更鲁棒：用“head 到 feetMid”在 up 方向的投影当身高（比 head.y 更稳）
                float h = Mathf.Abs(Vector3.Dot(head - footMid, Vector3.up));
                if (h > 0.5f && h < 3.0f) // 简单范围过滤（避免抖动/无效帧）
                {
                    sum += h;
                    count++;
                }
            }

            if (statusText)
            {
                float remain = Mathf.Max(0f, calibrationSeconds - (Time.time - t0));
                statusText.text = $"Calibrating... {remain:F1}s\nSamples: {count}";
            }

            yield return null;
        }

        if (count == 0)
        {
            Debug.LogError("[Session] Calibration failed: no valid samples.");
            _measuredHeightAvg = 0f;
            _calibratedScale = 1f;
            EnterPhase(SessionPhase.UIOnly);
            yield break;
        }

        _measuredHeightAvg = sum / count;
        _calibratedScale = targetRobotHeightMeters / _measuredHeightAvg;

        // 告诉 mapper：你后续保存要用这个 scale，并且使用同一个 pivot（保证“眼睛不动”的一致性）
        Vector3 pivot = GetScalePivotWorld();
        mapper.SetEmbodimentScale(_calibratedScale);
        mapper.SetEmbodimentPivotWorld(pivot);

        EnterPhase(SessionPhase.ReadyToEnterWorld);
        RefreshUi();
    }

    public void OnClickEnterWorld()
    {
        if (_phase != SessionPhase.ReadyToEnterWorld) return;

        // 以 pivot 为中心缩放世界与骨架可视化：保持 HMD 位置不变（你要的“眼睛与 XR Origin 齐平”效果）
        Vector3 pivot = GetScalePivotWorld();

        if (worldRoot) ApplyUniformScaleAroundPivot(worldRoot, _calibratedScale, pivot);
        if (avatarRoot) ApplyUniformScaleAroundPivot(avatarRoot, _calibratedScale, pivot);

        EnterPhase(SessionPhase.InWorld);

        if (autoHideMainPanelWhenEnterWorld && panelMain) panelMain.SetActive(false);
    }

    public void OnClickStartRecording()
    {
        if (_phase != SessionPhase.InWorld) return;
        if (mapper == null) return;
        mapper.BeginRecording();
        if (statusText) statusText.text += "\nRecording: ON";
    }

    public void OnClickStopRecording()
    {
        if (_phase != SessionPhase.InWorld) return;
        if (mapper == null) return;
        mapper.EndRecording();
        if (statusText) statusText.text += "\nRecording: OFF";
    }

    public void OnClickBackToUI()
    {
        // 回 UI 时不要试图“反缩放世界”（会越来越乱）。推荐：重新加载 scene 或者重置 root 的 scale/position。
        // 这里提供最保守做法：提示你手动重置或重载场景。
        Debug.LogWarning("[Session] BackToUI requested. Recommended: Reload scene to reset scale/positions.");
        EnterPhase(SessionPhase.UIOnly);
    }

    private Vector3 GetScalePivotWorld()
    {
        if (pivotAtHMD && hmd != null) return hmd.position;
        if (customPivot != null) return customPivot.position;
        return Vector3.zero;
    }

    // 核心：以 pivot 为中心进行均匀缩放，同时调整 root.position 保证 pivot 不动
    private void ApplyUniformScaleAroundPivot(Transform root, float scale, Vector3 pivotWorld)
    {
        if (root == null) return;

        // 重要：root 的父节点不要被缩放，否则这里会复杂很多。推荐 root.parent = null 或父节点 scale=1
        Vector3 oldPos = root.position;
        Vector3 dir = oldPos - pivotWorld;

        root.localScale = root.localScale * scale;
        root.position = pivotWorld + dir * scale;
    }
}


//using System;
//using System.Collections.Generic;
//using UnityEngine;
//using UnityEngine.UI;

//public class PXR_SessionControlPanel : MonoBehaviour
//{
//    [Serializable]
//    public class EmbodimentProfile
//    {
//        public string name = "G1";
//        public float targetHeightMeters = 1.27f;
//    }

//    public enum State { Idle, Calibrating, Calibrated, Ready, Recording }

//    [Header("References")]
//    public PXR_24PointMapper mapper;          // 你的 recorder
//    public Transform trackingOrigin;          // 同 mapper.trackingOrigin
//    public Transform environmentRoot;         // 环境根
//    public GameObject worldRootToEnable;      // “进入世界”时要启用的根（可选）

//    [Header("UI")]
//    public Dropdown embodimentDropdown;
//    public Button calibrateButton;
//    public Button enterWorldButton;
//    public Button startRecButton;
//    public Button stopRecButton;
//    public Text statusText;

//    [Header("Calibration")]
//    public float calibrationSeconds = 5f;
//    public float heightMin = 0.8f;
//    public float heightMax = 2.2f;

//    [Header("Embodiments")]
//    public List<EmbodimentProfile> embodiments = new List<EmbodimentProfile>()
//    {
//        new EmbodimentProfile(){ name="Unitree G1 (1.27m)", targetHeightMeters=1.27f }
//    };

//    private State _state = State.Idle;
//    private float _t0;
//    private float _sumH;
//    private int _cntH;

//    private float _scale = 1f;
//    private EmbodimentProfile _selected;

//    void Start()
//    {
//        if (mapper == null) Debug.LogError("[SessionPanel] mapper not set.");
//        if (trackingOrigin == null && mapper != null) trackingOrigin = mapper.trackingOrigin;
//        if (environmentRoot == null) environmentRoot = mapper != null ? mapper.trackingOrigin : null;

//        // init dropdown
//        if (embodimentDropdown != null)
//        {
//            embodimentDropdown.ClearOptions();
//            var opts = new List<string>();
//            foreach (var e in embodiments) opts.Add(e.name);
//            embodimentDropdown.AddOptions(opts);
//            embodimentDropdown.onValueChanged.AddListener(OnEmbodimentChanged);
//            OnEmbodimentChanged(0);
//        }

//        calibrateButton.onClick.AddListener(BeginCalibration);
//        enterWorldButton.onClick.AddListener(EnterWorld);
//        startRecButton.onClick.AddListener(StartRecording);
//        stopRecButton.onClick.AddListener(StopRecording);

//        if (worldRootToEnable != null) worldRootToEnable.SetActive(false);

//        SetState(State.Idle);
//    }

//    void Update()
//    {
//        if (_state != State.Calibrating) return;

//        float elapsed = Time.time - _t0;
//        float remain = Mathf.Max(0f, calibrationSeconds - elapsed);

//        // 取当前一帧身高
//        if (mapper != null && mapper.WorldJoints24 != null && mapper.WorldJoints24.Length >= 24 && trackingOrigin != null)
//        {
//            var pts = mapper.WorldJoints24;
//            Vector3 headW = pts[(int)PositionIndex.head];
//            Vector3 lfW = pts[(int)PositionIndex.left_foot_index];
//            Vector3 rfW = pts[(int)PositionIndex.right_foot_index];

//            // trackingOrigin local height
//            float headY = trackingOrigin.InverseTransformPoint(headW).y;
//            float footY = Mathf.Min(
//                trackingOrigin.InverseTransformPoint(lfW).y,
//                trackingOrigin.InverseTransformPoint(rfW).y
//            );

//            float H = headY - footY;
//            if (H >= heightMin && H <= heightMax)
//            {
//                _sumH += H;
//                _cntH += 1;
//            }
//        }

//        statusText.text = $"Calibrating... {remain:F1}s\nSamples: {_cntH}";

//        if (elapsed >= calibrationSeconds)
//        {
//            FinishCalibration();
//        }
//    }

//    void OnEmbodimentChanged(int idx)
//    {
//        if (idx < 0 || idx >= embodiments.Count) return;
//        _selected = embodiments[idx];
//        UpdateUI();
//    }

//    void BeginCalibration()
//    {
//        if (mapper == null || trackingOrigin == null)
//        {
//            statusText.text = "ERROR: mapper/trackingOrigin not set.";
//            return;
//        }

//        _t0 = Time.time;
//        _sumH = 0f;
//        _cntH = 0;
//        SetState(State.Calibrating);
//    }

//    void FinishCalibration()
//    {
//        if (_cntH < 10)
//        {
//            statusText.text = "Calibration failed: too few valid samples.\nTry again standing straight.";
//            SetState(State.Idle);
//            return;
//        }

//        float avgH = _sumH / _cntH;
//        _scale = _selected.targetHeightMeters / avgH;

//        // 应用到 mapper（让它写 CSV 时用机器人尺度）
//        mapper.SetEmbodimentScale(_scale);

//        // 环境缩放（可选：如果你希望“我和机器人在同一尺度的世界里”）
//        if (environmentRoot != null)
//        {
//            environmentRoot.localScale = environmentRoot.localScale * _scale;
//        }

//        statusText.text =
//            $"Calibrated.\nAvgHumanHeight={avgH:F3}m\nTarget={_selected.targetHeightMeters:F3}m\nScale={_scale:F4}";

//        SetState(State.Calibrated);
//    }

//    void EnterWorld()
//    {
//        if (_state != State.Calibrated) return;

//        if (worldRootToEnable != null) worldRootToEnable.SetActive(true);
//        SetState(State.Ready);
//    }

//    void StartRecording()
//    {
//        if (_state != State.Ready) return;
//        mapper.BeginRecording();
//        SetState(State.Recording);
//    }

//    void StopRecording()
//    {
//        if (_state != State.Recording) return;
//        mapper.EndRecording();
//        SetState(State.Ready);
//    }

//    void SetState(State s)
//    {
//        _state = s;
//        UpdateUI();
//    }

//    void UpdateUI()
//    {
//        if (calibrateButton != null) calibrateButton.interactable = (_state == State.Idle || _state == State.Calibrated);
//        if (enterWorldButton != null) enterWorldButton.interactable = (_state == State.Calibrated);
//        if (startRecButton != null) startRecButton.interactable = (_state == State.Ready);
//        if (stopRecButton != null) stopRecButton.interactable = (_state == State.Recording);

//        if (statusText != null && _state == State.Idle)
//            statusText.text = "Select embodiment -> Calibrate (stand straight 5s)";
//    }
//}
