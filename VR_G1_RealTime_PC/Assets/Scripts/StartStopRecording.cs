using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using UnityEngine.XR;

public sealed class VrRecordingTrigger : MonoBehaviour
{
	[Header("Python recorder")]
	[SerializeField]
	private string recorderIp = "127.0.0.1";

	[SerializeField]
	private int recorderPort = 5010;

	[Header("Controller")]
	[SerializeField]
	private XRNode controllerNode = XRNode.RightHand;

	[Tooltip("Press once to start, press again to stop.")]
	[SerializeField]
	private bool toggleRecording = true;

	private UdpClient udpClient;
	private InputDevice controller;

	private bool previousTriggerPressed;
	private bool recording;

	private void Start()
	{
		udpClient = new UdpClient();
		RefreshController();
	}

	private void Update()
	{
		if (!controller.isValid)
		{
			RefreshController();
		}

		if (!controller.TryGetFeatureValue(
				CommonUsages.triggerButton,
				out bool triggerPressed))
		{
			return;
		}

		bool triggerJustPressed =
			triggerPressed &&
			!previousTriggerPressed;

		bool triggerJustReleased =
			!triggerPressed &&
			previousTriggerPressed;

		if (toggleRecording)
		{
			if (triggerJustPressed)
			{
				recording = !recording;

				SendCommand(
					recording ? "START" : "STOP"
				);
			}
		}
		else
		{
			if (triggerJustPressed)
			{
				recording = true;
				SendCommand("START");
			}

			if (triggerJustReleased)
			{
				recording = false;
				SendCommand("STOP");
			}
		}

		previousTriggerPressed =
			triggerPressed;
	}

	private void RefreshController()
	{
		controller =
			InputDevices.GetDeviceAtXRNode(
				controllerNode
			);
	}

	private void SendCommand(string command)
	{
		try
		{
			byte[] bytes =
				Encoding.UTF8.GetBytes(command);

			udpClient.Send(
				bytes,
				bytes.Length,
				recorderIp,
				recorderPort
			);

			Debug.Log(
				$"C3D recorder command: {command}"
			);
		}
		catch (System.Exception exception)
		{
			Debug.LogWarning(
				$"Could not send recorder command: {exception.Message}"
			);
		}
	}

	private void OnDestroy()
	{
		udpClient?.Close();
		udpClient = null;
	}
}