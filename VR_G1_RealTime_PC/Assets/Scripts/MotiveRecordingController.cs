using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

// Sends acknowledged, idempotent commands to the native recorder. Nothing is
// recorded from Unity's filtered/rendered point cloud.
[DisallowMultipleComponent]
public sealed class MotiveRecordingController : MonoBehaviour
{
    [SerializeField] private bool showHeadsetStatus = true;
    private const int ControlPort = 5011;
    private readonly string clientId = Guid.NewGuid().ToString("N");
    private UdpClient socket;
    private InputDevice rightController;
    private readonly List<InputDevice> rightDevices = new List<InputDevice>();
    private bool primaryButtonAvailable;
    private bool secondaryButtonAvailable;
    private bool previousBPressed;
    private bool bButtonArmed;
    private OptiTrackPointReceiver pointReceiver;
    private float nextControllerScan;
    private string lastInputStatus;
    private bool previousPressed;
    private bool buttonArmed;
    private long sequence;
    private string pendingOperation;
    private long pendingSequence;
    private float commandStarted;
    private float nextSend;
    private float lastReply = float.NegativeInfinity;
    private bool recording;
    private bool recorderError;
    private long frames;
    private string takePath;
    private Transform viewCamera;
    private TextMesh label;
    private string lastLabel;
    private long filterSequence;
    private long lastFilterAcknowledged;
    private float nextFilterSend;
    private float lastFilterReply = float.NegativeInfinity;

    public bool IsRecording => recording && Time.unscaledTime - lastReply < 2f;

    public void Initialize(Camera camera)
    {
        viewCamera = camera != null ? camera.transform : null;
        if (viewCamera == null || label != null || !showHeadsetStatus) return;
        var status = new GameObject("Mocap Recording Status", typeof(TextMesh));
        status.layer = camera.gameObject.layer;
        status.transform.SetParent(viewCamera, false);
        status.transform.localPosition = new Vector3(0.12f, 0.12f, 0.65f);
        label = status.GetComponent<TextMesh>();
        label.font = Resources.GetBuiltinResource<Font>("LegacyRuntime.ttf");
        if (label.font != null) label.GetComponent<MeshRenderer>().sharedMaterial = label.font.material;
        label.fontSize = 48;
        label.characterSize = 0.002f;
        label.anchor = TextAnchor.UpperLeft;
        label.alignment = TextAlignment.Left;
        label.GetComponent<MeshRenderer>().shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        UpdateLabel();
    }

    private void OnEnable()
    {
        lastReply = float.NegativeInfinity;
        lastFilterReply = float.NegativeInfinity;
        nextFilterSend = 0f;
        pointReceiver = GetComponent<OptiTrackPointReceiver>();
        previousBPressed = false;
        bButtonArmed = false;
        secondaryButtonAvailable = false;
        previousPressed = false;
        buttonArmed = false; // A held across a reload/reconnect must not toggle recording.
        rightController = default;
        primaryButtonAvailable = false;
        nextControllerScan = 0f;
        lastInputStatus = null;
        nextSend = 0f;
        try
        {
            socket = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
            socket.Connect(IPAddress.Loopback, ControlPort);
            socket.Client.Blocking = false;
        }
        catch (Exception e) { Debug.LogWarning($"Mocap recording control unavailable: {e.Message}", this); }
    }

    private void Update()
    {
        ReceiveReplies();
        if (TryReadAButton(out bool pressed))
        {
            if (!pressed) buttonArmed = true;
            if (pressed && !previousPressed && buttonArmed && pendingOperation == null &&
                Time.unscaledTime - lastReply < 2f)
            {
                ToggleRecording();
            }
            previousPressed = pressed;
            bool bAvailable = rightController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool bPressed);
            UpdateMarkerFilterButton(bAvailable, bPressed);
        }
        else
        {
            previousPressed = false; buttonArmed = false;
            UpdateMarkerFilterButton(false, false);
        }

        if (pendingOperation != null && Time.unscaledTime - commandStarted > 5f)
        {
            Debug.LogWarning("Mocap recording command was not acknowledged. Check motive_client; recording state is unknown.", this);
            pendingOperation = null;
            lastReply = float.NegativeInfinity;
        }
        if (Time.unscaledTime >= nextSend)
        {
            Send(pendingOperation ?? "STATUS", pendingOperation != null ? pendingSequence : ++sequence);
            nextSend = Time.unscaledTime + (pendingOperation != null ? 0.25f : 0.5f);
        }
        UpdateLabel();
    }

    private void UpdateMarkerFilterButton(bool available, bool pressed)
    {
        secondaryButtonAvailable = available;
        if (!available) { previousBPressed = false; bButtonArmed = false; return; }
        if (!pressed) bButtonArmed = true;
        if (pressed && !previousBPressed && bButtonArmed && pointReceiver != null)
        {
            pointReceiver.ToggleMarkerFilter();
            nextFilterSend = 0f;
            if (rightController.isValid && rightController.TryGetHapticCapabilities(out var capabilities) && capabilities.supportsImpulse)
                rightController.SendHapticImpulse(0, 0.25f, 0.1f);
        }
        previousBPressed = pressed;
    }

    private void LateUpdate()
    {
        if (socket == null || pointReceiver == null || Time.unscaledTime < nextFilterSend) return;
        nextFilterSend = Time.unscaledTime + 0.05f;
        bool valid = pointReceiver.TryGetMarkerFilterTransform(out Matrix4x4 matrix);
        var message = new StringBuilder($"MTCFILTER1 {clientId} {++filterSequence} " +
            $"{(pointReceiver.IsMarkerFilterEnabled ? 1 : 0)} {(valid ? 1 : 0)}");
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                message.Append(' ').Append((valid ? matrix[r, c] : Matrix4x4.identity[r, c]).ToString("R", CultureInfo.InvariantCulture));
        try
        {
            byte[] bytes = Encoding.UTF8.GetBytes(message.ToString());
            socket.Send(bytes, bytes.Length);
        }
        catch (SocketException) { }
        catch (ObjectDisposedException) { }
    }

    private bool TryReadAButton(out bool pressed)
    {
        // OpenXR can expose a physical controller, D-pad, palm pose and hand pose
        // at the same XRNode. Selecting by node alone may return a pose-only device.
        if (rightController.isValid &&
            rightController.TryGetFeatureValue(CommonUsages.primaryButton, out pressed))
        {
            primaryButtonAvailable = true;
            return true;
        }
        pressed = false;
        primaryButtonAvailable = false;
        if (Time.unscaledTime < nextControllerScan) return false;
        nextControllerScan = Time.unscaledTime + 0.5f;
        InputDevices.GetDevicesWithCharacteristics(
            InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, rightDevices);
        foreach (InputDevice candidate in rightDevices)
        {
            if (!IsPhysicalRightController(candidate.characteristics) ||
                !candidate.TryGetFeatureValue(CommonUsages.primaryButton, out bool value)) continue;
            rightController = candidate;
            buttonArmed = false;
            previousPressed = false;
            bButtonArmed = false;
            previousBPressed = false;
            primaryButtonAvailable = true;
            pressed = value;
            ReportInputStatus($"A button available on {candidate.name}");
            return true;
        }
        ReportInputStatus("A button unavailable. Enable the Oculus Touch interaction profile for ALVR Quest2Touch, then restart Unity Play mode.");
        return false;
    }

    internal static bool IsPhysicalRightController(InputDeviceCharacteristics characteristics)
    {
        const InputDeviceCharacteristics required = InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right;
        return (characteristics & required) == required &&
            (characteristics & InputDeviceCharacteristics.HandTracking) == 0;
    }

    private void ReportInputStatus(string status)
    {
        if (status == lastInputStatus) return;
        lastInputStatus = status;
        Debug.Log($"Mocap recording input: {status}", this);
    }

    [ContextMenu("Toggle Mocap Recording")]
    public void ToggleRecording()
    {
        if (!Application.isPlaying || pendingOperation != null) return;
        if (Time.unscaledTime - lastReply >= 2f)
        {
            Debug.LogWarning("Cannot toggle mocap recording: no current recorder acknowledgment.", this);
            return;
        }
        pendingOperation = recording ? "STOP" : "START";
        pendingSequence = ++sequence;
        commandStarted = Time.unscaledTime;
        nextSend = 0f;
        Debug.Log($"Mocap recording requested: {pendingOperation}", this);
    }

    private void Send(string operation, long id)
    {
        if (socket == null) return;
        try
        {
            var bytes = Encoding.UTF8.GetBytes($"MTCREC1 {clientId} {id} {operation}");
            socket.Send(bytes, bytes.Length);
        }
        catch (SocketException) { } // Reconnect/status is retried without per-frame log spam.
        catch (ObjectDisposedException) { }
    }

    private void ReceiveReplies()
    {
        if (socket == null) return;
        try
        {
            for (int i = 0; i < 16 && socket.Available > 0; ++i)
            {
                IPEndPoint remote = null;
                string reply = Encoding.UTF8.GetString(socket.Receive(ref remote));
                string[] fields = reply.Split(new[] { ' ' }, 7);
                if (fields.Length == 3 && fields[0] == "MTCFILTER1" && fields[1] == clientId &&
                    long.TryParse(fields[2], out long filterId) && filterId > lastFilterAcknowledged && filterId <= filterSequence)
                {
                    lastFilterAcknowledged = filterId;
                    lastFilterReply = Time.unscaledTime;
                    continue;
                }
                if (fields.Length != 7 || fields[0] != "MTCREC1" || fields[1] != clientId ||
                    !long.TryParse(fields[2], out long id) || id != sequence ||
                    (fields[3] != "ON" && fields[3] != "OFF" && fields[3] != "ERROR") ||
                    !long.TryParse(fields[4], NumberStyles.None, CultureInfo.InvariantCulture, out long count)) continue;
                bool changed = recording != (fields[3] == "ON");
                recording = fields[3] == "ON";
                recorderError = fields[3] == "ERROR";
                lastReply = Time.unscaledTime;
                frames = count;
                takePath = fields[6];
                bool acknowledged = pendingOperation != null && id == pendingSequence;
                if (acknowledged) pendingOperation = null;
                if (changed || acknowledged)
                {
                    Debug.Log($"Mocap recording: {fields[3]} | {frames} frames | {takePath}", this);
                    if (rightController.isValid && rightController.TryGetHapticCapabilities(out var capabilities) && capabilities.supportsImpulse)
                        rightController.SendHapticImpulse(0, 0.35f, recording ? 0.15f : 0.3f);
                }
            }
        }
        catch (SocketException) { }
        catch (ObjectDisposedException) { }
    }

    private void UpdateLabel()
    {
        if (label == null) return;
        bool connected = Time.unscaledTime - lastReply < 2f;
        string status = pendingOperation != null ? "Mocap: waiting..." : !connected ? "Mocap: no recorder" :
            recorderError ? "Mocap: recording ERROR\nCheck saved take" :
            recording ? $"REC  {frames} frames\n" + (primaryButtonAvailable ? "A: stop" : "A unavailable") :
            !primaryButtonAvailable ? "Mocap: A unavailable\nRestart Unity Play" : "Mocap: ready\nA: record";
        if (pointReceiver != null)
            status += "\n" + (secondaryButtonAvailable ?
                $"B: filter {(pointReceiver.IsMarkerFilterEnabled ? "ON" : "OFF")}" : "B unavailable");
        if (pointReceiver != null && pointReceiver.IsMarkerFilterEnabled && connected && Time.unscaledTime - lastFilterReply >= 1f)
            status += "\nFilter recording not synced";
        if (pointReceiver != null && !string.IsNullOrEmpty(pointReceiver.HeightScalingStatus))
            status += "\n" + pointReceiver.HeightScalingStatus;
        if (status == lastLabel) return;
        lastLabel = status;
        label.text = status;
        label.color = !connected || recorderError || !primaryButtonAvailable ? Color.yellow : recording ? Color.red : Color.green;
    }

    private void OnDisable()
    {
        socket?.Close(); socket = null;
        pendingOperation = null;
        // The native take deliberately survives Unity reloads/disconnects. A status
        // handshake on reconnect adopts the recorder's actual state.
        if (label != null) { Destroy(label.gameObject); label = null; lastLabel = null; }
    }
}
