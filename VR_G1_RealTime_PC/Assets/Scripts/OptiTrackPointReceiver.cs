using System;
using System.Collections.Generic;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.XR;

public sealed class OptiTrackPointReceiver : MonoBehaviour
{
	[Header("Network")]
	[SerializeField]
	private int listenPort = 5005;

	[Header("Point Visualization")]
	[SerializeField]
	private GameObject pointPrefab;

	[SerializeField]
	private float pointScale = 0.05f;

	[Tooltip("Hide point spheres whose centers are within this distance of the VR camera, in meters.")]
	[Min(0.0f)]
	[SerializeField]
	private float minimumCameraDistanceMeters = 0.15f;

	[SerializeField]
	private Transform pointParent;

	[Tooltip("How quickly points follow their latest mocap targets. Higher is snappier; 0 disables smoothing.")]
	[Min(0.0f)]
	[SerializeField]
	private float pointSmoothingRate = 35.0f;

	[Tooltip("Keep a missing point visible for this long to bridge brief occlusions.")]
	[Min(0.0f)]
	[SerializeField]
	private float pointVisibilityGraceSeconds = 0.2f;

	[Header("Headset Marker Filter (Right B)")]
	[Tooltip("Show only markers inside the headset-following box. Toggle with right-controller B.")]
	[SerializeField]
	private bool filterMarkersAroundHeadset;

	private const float MarkerFilterBelowMeters = 2f;
	private const float MarkerFilterAboveMeters = 0.5f;
	private const float MarkerFilterHorizontalMeters = 0.3f;
	private const float MarkerFilterBoundaryTolerance = 0.00001f;

	public bool IsMarkerFilterEnabled => filterMarkersAroundHeadset;

	[ContextMenu("Toggle Headset Marker Filter")]
	public void ToggleMarkerFilter()
	{
		if (!filterMarkersAroundHeadset && vrCamera == null)
		{
			Debug.LogWarning("Cannot enable the marker filter without the tracked VR camera.", this);
			return;
		}
		filterMarkersAroundHeadset = !filterMarkersAroundHeadset;
		Debug.Log($"Mocap marker filter: {(filterMarkersAroundHeadset ? "ON" : "OFF")} " +
			"(2 m below, 0.5 m above, 0.3 m in each horizontal direction from headset).", this);
		SmoothAndCullPointVisuals();
	}

	private bool MarkerPassesHeadsetFilter(Vector3 worldPosition)
	{
		if (!filterMarkersAroundHeadset || vrCamera == null) return true;
		// World-up box follows the headset's position, without tilting/rotating when
		// the wearer looks around. Test aligned world metres, never parent-local units.
		Vector3 delta = worldPosition - vrCamera.position;
		return Mathf.Abs(delta.x) <= MarkerFilterHorizontalMeters + MarkerFilterBoundaryTolerance &&
			Mathf.Abs(delta.z) <= MarkerFilterHorizontalMeters + MarkerFilterBoundaryTolerance &&
			delta.y >= -MarkerFilterBelowMeters - MarkerFilterBoundaryTolerance &&
			delta.y <= MarkerFilterAboveMeters + MarkerFilterBoundaryTolerance;
	}

	// Native recordings keep Motive coordinates. Supply the same conversion and
	// alignment used by live markers so both crop against the same headset box.
	public bool TryGetMarkerFilterTransform(out Matrix4x4 rawToHeadsetDelta)
	{
		rawToHeadsetDelta = Matrix4x4.identity;
		if (vrCamera == null) return false;
		Vector3 zero = MotivePointToWorld(Vector3.zero);
		Vector3 x = MotivePointToWorld(Vector3.right) - zero;
		Vector3 y = MotivePointToWorld(Vector3.up) - zero;
		Vector3 z = MotivePointToWorld(Vector3.forward) - zero;
		Vector3 translation = zero - vrCamera.position;
		rawToHeadsetDelta.SetColumn(0, new Vector4(x.x, x.y, x.z, 0));
		rawToHeadsetDelta.SetColumn(1, new Vector4(y.x, y.y, y.z, 0));
		rawToHeadsetDelta.SetColumn(2, new Vector4(z.x, z.y, z.z, 0));
		rawToHeadsetDelta.SetColumn(3, new Vector4(translation.x, translation.y, translation.z, 1));
		for (int r = 0; r < 3; r++)
			for (int c = 0; c < 4; c++)
				if (float.IsNaN(rawToHeadsetDelta[r, c]) || float.IsInfinity(rawToHeadsetDelta[r, c])) return false;
		return true;
	}

	private Vector3 MotivePointToWorld(Vector3 motivePosition)
	{
		Vector3 converted = ConvertPosition(motivePosition);
		return hasPointAlignment ? mocapToUnityRotation * converted + mocapToUnityTranslation : converted;
	}

	[Header("XR References")]
	[Tooltip("Assign the top-level XR Origin object.")]
	[SerializeField]
	private Transform xrOrigin;

	[Tooltip("Assign XR Origin/Camera Offset/Main Camera.")]
	[SerializeField]
	private Transform vrCamera;

	[Header("Motive Rigid-Body Calibration")]
	[Tooltip(
		"Offset from the Motive Pico rigid-body origin to the actual " +
		"headset camera position, expressed in the converted rigid-body frame."
	)]
	[SerializeField]
	private Vector3 cameraOffsetFromRigidBody = Vector3.zero;

	[Tooltip(
		"Fixed rotation needed because the Motive rigid-body axes " +
		"may not exactly match the headset camera axes."
	)]
	[SerializeField]
	private Vector3 rigidBodyRotationOffsetEuler = Vector3.zero;

	[Tooltip(
		"Local Motive rigid-body axis that points out of the front " +
		"of the headset."
	)]
	[SerializeField]
	private Vector3 motiveLocalForwardAxis = Vector3.right;

	[Tooltip(
		"Local Motive rigid-body axis that points upward."
	)]
	[SerializeField]
	private Vector3 motiveLocalUpAxis = Vector3.forward;

	[Header("Mocap Point Alignment to PICO")]
	[Tooltip(
		"Align mocap points using yaw only. The PICO camera remains fully controlled by headset tracking."
	)]
	[SerializeField]
	private bool useYawOnly = true;

	[Tooltip(
		"How quickly the mocap-to-Unity position alignment follows the latest good Motive pose."
	)]
	[Min(0.0f)]
	[SerializeField]
	private float positionCorrectionRate = 2.0f;

	[Tooltip(
		"How quickly the mocap-to-Unity rotation alignment follows the latest good Motive pose."
	)]
	[Min(0.0f)]
	[SerializeField]
	private float rotationCorrectionRate = 1.5f;

	[Tooltip("Limit translation catch-up speed. Leave off so markers can converge from any distance.")]
	[SerializeField]
	private bool limitPositionCorrectionSpeed = false;

	[Tooltip(
		"Maximum translation speed when Limit Position Correction Speed is enabled. Zero means unlimited."
	)]
	[Min(0.0f)]
	[SerializeField]
	private float maximumCorrectionSpeedMetersPerSecond = 1.0f;

	[Tooltip(
		"Maximum speed at which the mocap point alignment rotation may change."
	)]
	[Min(0.0f)]
	[SerializeField]
	private float maximumRotationCorrectionDegreesPerSecond = 90.0f;

	[Header("Pose Quality Rejection")]
	[Tooltip(
		"Reject Motive poses whose MeanError exceeds this value. " +
		"0.005 meters equals 5 mm."
	)]
	[Min(0.0f)]
	[SerializeField]
	private float maximumMeanErrorMeters = 0.005f;

	[Tooltip(
		"Stop applying Motive corrections when no new good pose has " +
		"arrived within this many seconds."
	)]
	[Min(0.0f)]
	[SerializeField]
	private float motivePoseTimeoutSeconds = 0.25f;

	[Tooltip(
		"Keep the previous rotation if yaw jumps farther than this amount. " +
		"Position still updates; rotation is reacquired after a tracking timeout. Zero disables this check."
	)]
	[Min(0.0f)]
	[SerializeField]
	private float maximumAcceptedYawJumpDegrees = 90.0f;

	[Header("Coordinate Conversion")]
	[Tooltip(
		"Enable this when Motive uses Z as its vertical axis."
	)]
	[SerializeField]
	private bool convertFromMotiveZUp = true;

	[SerializeField]
	private Vector3 positionOffset = Vector3.zero;

	[SerializeField]
	private float positionScale = 1.0f;

	[Header("Environment Collision")]
	[Tooltip("Assign the root GameObject named apartment_1.")]
	[SerializeField]
	private GameObject apartmentRoot;

	[Tooltip(
		"Approximate distance from a mocap marker center to the body surface. " +
		"A point is touching when it is within this distance of apartment_1."
	)]
	[Min(0.001f)]
	[SerializeField]
	private float contactRadiusMeters = 0.05f;

	[Tooltip(
		"How many visible mocap points must touch before contact is reported."
	)]
	[Min(1)]
	[SerializeField]
	private int requiredTouchingPoints = 1;

	[Tooltip(
		"How many consecutive Unity frames contact must exist before it is accepted."
	)]
	[Min(1)]
	[SerializeField]
	private int requiredContactFrames = 2;

	[Tooltip("Print when apartment contact begins or ends.")]
	[SerializeField]
	private bool logContactChanges = true;

	[Header("Controller Haptics")]
	[Range(0.0f, 1.0f)]
	[SerializeField]
	private float collisionHapticAmplitude = 0.7f;

	[Min(0.01f)]
	[SerializeField]
	private float collisionHapticDurationSeconds = 0.15f;

	[Tooltip("Send repeated vibration pulses while apartment contact continues.")]
	[SerializeField]
	private bool repeatHapticsWhileTouching = true;

	[Min(0.05f)]
	[SerializeField]
	private float hapticRepeatIntervalSeconds = 0.25f;

	[Header("Debug")]
	[SerializeField]
	private bool logFusionStatus = false;

	[Min(0.05f)]
	[SerializeField]
	private float logIntervalSeconds = 0.5f;

	public bool IsTouchingApartment { get; private set; }
	public int TouchingPointCount { get; private set; }

	private UdpClient udpClient;
	private Thread receiveThread;
	private volatile bool running;

	private readonly object latestFrameLock = new object();
	private PointFrame latestFrame;

	private readonly Dictionary<int, PointVisual> pointVisuals =
		new Dictionary<int, PointVisual>();

	private long packetsReceived;
	private long parsedFrames;
	private long parseFailures;
	private int displayedFrames;
	private int lastDisplayedFrameNumber = -1;
	private long lastDisplayedReceiveTimestamp;
	private string latestMotivePoseStatus = "no frame";
	private float nextDiagnosticsTime;

	private bool hasAcceptedMotivePose;
	private Vector3 acceptedMotiveCameraPosition;
	private Quaternion acceptedMotiveCameraRotation = Quaternion.identity;
	private float acceptedMeanErrorMeters = -1.0f;
	private float acceptedPoseArrivalTime = float.NegativeInfinity;
	private float nextLogTime;

	private bool hasPointAlignment;
	private Quaternion mocapToUnityRotation = Quaternion.identity;
	private Vector3 mocapToUnityTranslation = Vector3.zero;

	private MeshCollider[] apartmentMeshColliders =
		Array.Empty<MeshCollider>();

	private readonly HashSet<Collider> apartmentColliderSet =
		new HashSet<Collider>();

	private readonly Collider[] overlapResults =
		new Collider[32];

	private int consecutiveContactFrames;
	private bool previousTouchingState;

	private InputDevice leftController;
	private InputDevice rightController;
	private float nextHapticTime;

	private struct TrackedPoint
	{
		public int id;
		public Vector3 position;
	}

	private sealed class PointVisual
	{
		public GameObject gameObject;
		public Renderer[] renderers;
		public Vector3 targetLocalPosition;
		public float lastSeenTime;
	}

	private sealed class PointFrame
	{
		public int frameNumber;
		public long receivedTimestamp;
		public TrackedPoint[] points;

		public bool picoAvailable;
		public bool picoGoodQuality;
		public float picoMeanError;

		public Vector3 picoPosition;
		public Quaternion picoRotation;
	}

	private void OnEnable()
	{
		// Unity calls OnEnable again after a Play-mode script reload; Start is not
		// called again, so receiver sockets/threads must be restored here.
		hasAcceptedMotivePose = false;
		acceptedPoseArrivalTime = float.NegativeInfinity;
		QualitySettings.vSyncCount = 0;
		Application.targetFrameRate = 90;
		nextDiagnosticsTime = Time.unscaledTime + 1.0f;

		if (pointParent == null)
		{
			pointParent = transform;
		}

		if (xrOrigin == null)
		{
			Debug.LogError(
				"OptiTrackPointReceiver: XR Origin is not assigned."
			);
		}

		if (vrCamera == null)
		{
			Debug.LogError(
				"OptiTrackPointReceiver: VR Camera is not assigned."
			);
		}
		else
		{
			CollisionScreenBorder screenBorder = GetComponent<CollisionScreenBorder>();
			if (screenBorder == null)
			{
				screenBorder = gameObject.AddComponent<CollisionScreenBorder>();
			}

			screenBorder.Initialize(this, vrCamera.GetComponent<Camera>());
		}

		InitializeApartmentColliders();
		InitializeHapticControllers();
		MotiveRecordingController recordingController = GetComponent<MotiveRecordingController>();
		if (recordingController == null) recordingController = gameObject.AddComponent<MotiveRecordingController>();
		recordingController.Initialize(vrCamera != null ? vrCamera.GetComponent<Camera>() : null);
		StartReceiver();
	}

	private void InitializeApartmentColliders()
	{
		if (apartmentRoot == null)
		{
			apartmentRoot = GameObject.Find("apartment_1");
		}

		if (apartmentRoot == null)
		{
			Debug.LogError(
				"OptiTrackPointReceiver: Could not find apartment_1. " +
				"Assign Apartment Root in the Inspector."
			);

			apartmentMeshColliders = Array.Empty<MeshCollider>();
			return;
		}

		apartmentMeshColliders =
			apartmentRoot.GetComponentsInChildren<MeshCollider>(true);

		apartmentColliderSet.Clear();

		foreach (MeshCollider meshCollider in apartmentMeshColliders)
		{
			if (meshCollider != null)
			{
				apartmentColliderSet.Add(meshCollider);
			}
		}

		if (apartmentMeshColliders.Length == 0)
		{
			Debug.LogError(
				"OptiTrackPointReceiver: apartment_1 contains no MeshCollider. " +
				"Add a MeshCollider to the apartment mesh or one of its children."
			);
			return;
		}

		Debug.Log(
			"OptiTrackPointReceiver: Found " +
			$"{apartmentMeshColliders.Length} MeshCollider(s) under " +
			$"{apartmentRoot.name}. Using OverlapSphereNonAlloc for contact."
		);
	}

	private void InitializeHapticControllers()
	{
		leftController =
			InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);

		rightController =
			InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
	}

	private void StartReceiver()
	{
		try
		{
			udpClient = new UdpClient(listenPort);
			running = true;

			receiveThread = new Thread(ReceiveLoop)
			{
				IsBackground = true,
				Name = "OptiTrack UDP Receiver"
			};

			receiveThread.Start();

			Debug.Log(
				$"OptiTrack receiver listening on UDP {listenPort}"
			);
		}
		catch (Exception exception)
		{
			Debug.LogError(
				$"Could not start UDP receiver: {exception}"
			);
		}
	}

	private void ReceiveLoop()
	{
		var remoteEndpoint =
			new IPEndPoint(
				IPAddress.Any,
				0
			);

		while (running)
		{
			try
			{
				byte[] bytes = udpClient.Receive(ref remoteEndpoint);
				long receivedTimestamp = System.Diagnostics.Stopwatch.GetTimestamp();
				Interlocked.Increment(ref packetsReceived);

				string message = Encoding.UTF8.GetString(bytes);
				PointFrame parsed = ParseFrame(message);

				if (parsed != null)
				{
					parsed.receivedTimestamp = receivedTimestamp;
					Interlocked.Increment(ref parsedFrames);
					lock (latestFrameLock)
					{
						latestFrame = parsed;
					}
				}
				else
				{
					Interlocked.Increment(ref parseFailures);
				}
			}
			catch (SocketException)
			{
				if (running)
				{
					Debug.LogWarning(
						"OptiTrack UDP socket stopped unexpectedly."
					);
				}
			}
			catch (ObjectDisposedException)
			{
				// Expected during shutdown.
			}
			catch (Exception exception)
			{
				Debug.LogWarning(
					$"OptiTrack packet error: {exception.Message}"
				);
			}
		}
	}

	private static PointFrame ParseFrame(string message)
	{
		string[] fields = message.Split(';');

		if (fields.Length < 2)
		{
			return null;
		}

		if (!int.TryParse(
				fields[0],
				NumberStyles.Integer,
				CultureInfo.InvariantCulture,
				out int frameNumber))
		{
			return null;
		}

		if (!int.TryParse(
				fields[1],
				NumberStyles.Integer,
				CultureInfo.InvariantCulture,
				out int pointCount))
		{
			return null;
		}

		if (pointCount < 0 ||
			fields.Length < pointCount + 2)
		{
			return null;
		}

		var points = new TrackedPoint[pointCount];

		for (int i = 0; i < pointCount; ++i)
		{
			if (!TryParseTrackedPoint(fields[i + 2], i, out points[i]))
			{
				return null;
			}
		}

		var frame =
			new PointFrame
			{
				frameNumber = frameNumber,
				points = points,
				picoAvailable = false,
				picoGoodQuality = false,
				picoMeanError = -1.0f,
				picoPosition = Vector3.zero,
				picoRotation = Quaternion.identity
			};

		int rigidBodyStart = pointCount + 2;

		/*
		 * Expected suffix:
		 *
		 * PICO;available;goodQuality;meanError;x,y,z;qx,qy,qz,qw
		 */
		if (fields.Length >= rigidBodyStart + 6 &&
			fields[rigidBodyStart] == "PICO")
		{
			frame.picoAvailable =
				fields[rigidBodyStart + 1] == "1";

			frame.picoGoodQuality =
				fields[rigidBodyStart + 2] == "1";

			if (!TryParseFloat(
					fields[rigidBodyStart + 3],
					out frame.picoMeanError))
			{
				return null;
			}

			if (!TryParseVector3(
					fields[rigidBodyStart + 4],
					out frame.picoPosition))
			{
				return null;
			}

			if (!TryParseQuaternion(
					fields[rigidBodyStart + 5],
					out frame.picoRotation))
			{
				return null;
			}
		}

		return frame;
	}

	private static bool TryParseTrackedPoint(
		string text,
		int legacyIndex,
		out TrackedPoint result)
	{
		result = default;
		string[] values = text.Split(',');

		if (values.Length == 4)
		{
			if (!int.TryParse(
					values[0],
					NumberStyles.Integer,
					CultureInfo.InvariantCulture,
					out result.id) ||
				!TryParseFloat(values[1], out float x) ||
				!TryParseFloat(values[2], out float y) ||
				!TryParseFloat(values[3], out float z))
			{
				return false;
			}

			result.position = new Vector3(x, y, z);
			return true;
		}

		if (values.Length == 3 &&
			TryParseFloat(values[0], out float legacyX) &&
			TryParseFloat(values[1], out float legacyY) &&
			TryParseFloat(values[2], out float legacyZ))
		{
			result.id = legacyIndex;
			result.position =
				new Vector3(legacyX, legacyY, legacyZ);
			return true;
		}

		return false;
	}

	private static bool TryParseVector3(
		string text,
		out Vector3 result)
	{
		result = Vector3.zero;

		string[] values = text.Split(',');

		if (values.Length != 3)
		{
			return false;
		}

		if (!TryParseFloat(values[0], out float x) ||
			!TryParseFloat(values[1], out float y) ||
			!TryParseFloat(values[2], out float z))
		{
			return false;
		}

		result = new Vector3(x, y, z);
		return true;
	}

	private static bool TryParseQuaternion(
		string text,
		out Quaternion result)
	{
		result = Quaternion.identity;

		string[] values = text.Split(',');

		if (values.Length != 4)
		{
			return false;
		}

		if (!TryParseFloat(values[0], out float x) ||
			!TryParseFloat(values[1], out float y) ||
			!TryParseFloat(values[2], out float z) ||
			!TryParseFloat(values[3], out float w))
		{
			return false;
		}

		result = new Quaternion(x, y, z, w);
		return true;
	}

	private static bool TryParseFloat(
		string text,
		out float value)
	{
		return float.TryParse(
			text,
			NumberStyles.Float,
			CultureInfo.InvariantCulture,
			out value
		);
	}

	private void Update()
	{
		PointFrame frameToDisplay = null;

		lock (latestFrameLock)
		{
			if (latestFrame != null)
			{
				frameToDisplay = latestFrame;
				latestFrame = null;
			}
		}

		if (frameToDisplay != null)
		{
			DisplayPoints(frameToDisplay);
			displayedFrames++;
			lastDisplayedFrameNumber =
				frameToDisplay.frameNumber;
			lastDisplayedReceiveTimestamp = frameToDisplay.receivedTimestamp;
			latestMotivePoseStatus = $"available={frameToDisplay.picoAvailable}, " +
				$"good={frameToDisplay.picoGoodQuality}, error={frameToDisplay.picoMeanError * 1000f:F2} mm";

			if (PosePassesQualityChecks(frameToDisplay))
			{
				TryAcceptMotivePose(frameToDisplay);
			}
		}

		SmoothAndCullPointVisuals();
		DetectApartmentContact();
		LogRateDiagnostics();
	}

	private void LateUpdate()
	{
		if (!hasAcceptedMotivePose ||
			xrOrigin == null ||
			vrCamera == null)
		{
			return;
		}

		float poseAge =
			Time.unscaledTime -
			acceptedPoseArrivalTime;

		if (poseAge > motivePoseTimeoutSeconds)
		{
			return;
		}

		UpdatePointAlignment(Time.unscaledDeltaTime);
	}

	private bool PosePassesQualityChecks(PointFrame frame)
	{
		if (!frame.picoAvailable ||
			!frame.picoGoodQuality)
		{
			return false;
		}

		if (float.IsNaN(frame.picoMeanError) ||
			float.IsInfinity(frame.picoMeanError) ||
			frame.picoMeanError < 0.0f ||
			frame.picoMeanError > maximumMeanErrorMeters)
		{
			return false;
		}

		return true;
	}

	private void TryAcceptMotivePose(PointFrame frame)
	{
		Vector3 rigidBodyPosition =
			ConvertPosition(frame.picoPosition);

		Quaternion rigidBodyRotation =
			ConvertMotiveRotation(frame.picoRotation);

		rigidBodyRotation *=
			Quaternion.Euler(
				rigidBodyRotationOffsetEuler
			);

		Vector3 candidateCameraPosition =
			rigidBodyPosition +
			rigidBodyRotation *
			cameraOffsetFromRigidBody;

		Quaternion candidateCameraRotation =
			rigidBodyRotation;

		// A valid new position is always a new alignment target, regardless of distance.
		// Rotation outliers must not starve translation or latch the whole pose forever.
		bool previousPoseIsFresh = hasAcceptedMotivePose &&
			Time.unscaledTime - acceptedPoseArrivalTime <= motivePoseTimeoutSeconds;
		if (previousPoseIsFresh)
		{
			if (maximumAcceptedYawJumpDegrees > 0.0f)
			{
				float yawJump =
					Mathf.Abs(
						GetHorizontalYawError(
							acceptedMotiveCameraRotation,
							candidateCameraRotation
						)
					);

				if (yawJump >
					maximumAcceptedYawJumpDegrees)
				{
					if (logFusionStatus)
					{
						Debug.LogWarning(
							"Kept previous Motive rotation: " +
							$"yaw jumped {yawJump:F1} degrees; position still updates."
						);
					}

					candidateCameraRotation = acceptedMotiveCameraRotation;
				}
			}
		}

		acceptedMotiveCameraPosition =
			candidateCameraPosition;

		acceptedMotiveCameraRotation =
			candidateCameraRotation;

		acceptedMeanErrorMeters =
			frame.picoMeanError;

		acceptedPoseArrivalTime =
			Time.unscaledTime;

		hasAcceptedMotivePose = true;
	}

	private void UpdatePointAlignment(float deltaTime)
	{
		if (deltaTime <= 0.0f)
		{
			return;
		}

		/*
		 * Find the transform that maps the converted Motive camera pose
		 * onto the current PICO camera pose:
		 *
		 * unityPoint = alignmentRotation * motivePoint + alignmentTranslation
		 *
		 * The XR Origin and VR camera are never modified here.
		 */
		Quaternion desiredAlignmentRotation;

		if (useYawOnly)
		{
			float yawError =
				GetHorizontalYawError(
					acceptedMotiveCameraRotation,
					vrCamera.rotation
				);

			desiredAlignmentRotation =
				Quaternion.AngleAxis(
					yawError,
					Vector3.up
				);
		}
		else
		{
			desiredAlignmentRotation =
				vrCamera.rotation *
				Quaternion.Inverse(
					acceptedMotiveCameraRotation
				);
		}

		Vector3 desiredAlignmentTranslation =
			vrCamera.position -
			desiredAlignmentRotation *
			acceptedMotiveCameraPosition;

		if (!hasPointAlignment)
		{
			mocapToUnityRotation =
				desiredAlignmentRotation;

			mocapToUnityTranslation =
				desiredAlignmentTranslation;

			hasPointAlignment = true;
			LogFusionStatus();
			return;
		}

		float rotationAlpha =
			1.0f -
			Mathf.Exp(
				-rotationCorrectionRate *
					deltaTime
			);

		Quaternion smoothedRotation =
			Quaternion.Slerp(
				mocapToUnityRotation,
				desiredAlignmentRotation,
				rotationAlpha
			);

		float maximumRotationThisFrame =
			maximumRotationCorrectionDegreesPerSecond *
			deltaTime;

		mocapToUnityRotation =
			Quaternion.RotateTowards(
				mocapToUnityRotation,
				smoothedRotation,
				maximumRotationThisFrame
			);

		float positionAlpha =
			1.0f -
			Mathf.Exp(
				-positionCorrectionRate *
					deltaTime
			);

		Vector3 smoothedTranslation =
			Vector3.Lerp(
				mocapToUnityTranslation,
				desiredAlignmentTranslation,
				positionAlpha
			);

		float maximumDistanceThisFrame =
			maximumCorrectionSpeedMetersPerSecond *
			deltaTime;

		mocapToUnityTranslation =
			limitPositionCorrectionSpeed && maximumCorrectionSpeedMetersPerSecond > 0.0f
				? Vector3.MoveTowards(mocapToUnityTranslation, smoothedTranslation, maximumDistanceThisFrame)
				: smoothedTranslation;

		LogFusionStatus();
	}

	private static Quaternion CalculateYawOnlyOriginRotation(
		Quaternion cameraLocalRotation,
		Quaternion targetCameraRotation)
	{
		Vector3 localCameraForward =
			cameraLocalRotation *
			Vector3.forward;

		Vector3 targetCameraForward =
			targetCameraRotation *
			Vector3.forward;

		localCameraForward =
			Vector3.ProjectOnPlane(
				localCameraForward,
				Vector3.up
			);

		targetCameraForward =
			Vector3.ProjectOnPlane(
				targetCameraForward,
				Vector3.up
			);

		if (localCameraForward.sqrMagnitude < 0.000001f ||
			targetCameraForward.sqrMagnitude < 0.000001f)
		{
			return Quaternion.identity;
		}

		localCameraForward.Normalize();
		targetCameraForward.Normalize();

		float localCameraYaw =
			Vector3.SignedAngle(
				Vector3.forward,
				localCameraForward,
				Vector3.up
			);

		float targetCameraYaw =
			Vector3.SignedAngle(
				Vector3.forward,
				targetCameraForward,
				Vector3.up
			);

		float requiredOriginYaw =
			targetCameraYaw -
			localCameraYaw;

		return Quaternion.AngleAxis(
			requiredOriginYaw,
			Vector3.up
		);
	}

	private static float GetHorizontalYawError(
		Quaternion currentRotation,
		Quaternion targetRotation)
	{
		Vector3 currentForward =
			Vector3.ProjectOnPlane(
				currentRotation *
				Vector3.forward,
				Vector3.up
			);

		Vector3 targetForward =
			Vector3.ProjectOnPlane(
				targetRotation *
				Vector3.forward,
				Vector3.up
			);

		if (currentForward.sqrMagnitude < 0.000001f ||
			targetForward.sqrMagnitude < 0.000001f)
		{
			return 0.0f;
		}

		currentForward.Normalize();
		targetForward.Normalize();

		return Vector3.SignedAngle(
			currentForward,
			targetForward,
			Vector3.up
		);
	}

	private void LogFusionStatus()
	{
		if (!logFusionStatus ||
			Time.unscaledTime < nextLogTime)
		{
			return;
		}

		Vector3 alignedMotiveCameraPosition =
			mocapToUnityRotation *
			acceptedMotiveCameraPosition +
			mocapToUnityTranslation;

		Quaternion alignedMotiveCameraRotation =
			mocapToUnityRotation *
			acceptedMotiveCameraRotation;

		Vector3 positionError =
			vrCamera.position -
			alignedMotiveCameraPosition;

		float yawError =
			GetHorizontalYawError(
				alignedMotiveCameraRotation,
				vrCamera.rotation
			);

		Debug.Log(
			"PICO + Motive point alignment" +
			$"\nAligned Motive camera position: {alignedMotiveCameraPosition}" +
			$"\nPICO camera position: {vrCamera.position}" +
			$"\nRemaining position error: {positionError.magnitude:F4} m" +
			$"\nRemaining yaw error: {yawError:F2} degrees" +
			$"\nMean error: {acceptedMeanErrorMeters * 1000.0f:F2} mm"
		);

		nextLogTime =
			Time.unscaledTime +
			logIntervalSeconds;
	}

	private void DisplayPoints(PointFrame frame)
	{
		float now = Time.unscaledTime;

		for (int i = 0; i < frame.points.Length; ++i)
		{
			TrackedPoint trackedPoint = frame.points[i];

			bool isNewPoint = !pointVisuals.TryGetValue(
					trackedPoint.id,
					out PointVisual visual);
			if (isNewPoint)
			{
				visual = CreatePointVisual(trackedPoint.id);
				pointVisuals.Add(trackedPoint.id, visual);
			}

			Vector3 convertedMotivePosition =
				ConvertPosition(trackedPoint.position);

			Vector3 alignedWorldPosition =
				hasPointAlignment
					? mocapToUnityRotation * convertedMotivePosition +
						mocapToUnityTranslation
					: convertedMotivePosition;

			visual.targetLocalPosition =
				pointParent.InverseTransformPoint(
					alignedWorldPosition
				);

			visual.lastSeenTime = now;

			if (isNewPoint || !visual.gameObject.activeSelf)
			{
				// The first observation has no previous measured position to smooth
				// from. Place it directly instead of animating from the parent's origin.
				visual.gameObject.transform.localPosition =
					visual.targetLocalPosition;
				visual.gameObject.SetActive(true);
			}
		}
	}

	private PointVisual CreatePointVisual(int id)
	{
		GameObject point;

		if (pointPrefab != null)
		{
			point = Instantiate(pointPrefab, pointParent);
		}
		else
		{
			point =
				GameObject.CreatePrimitive(
					PrimitiveType.Sphere
				);

			point.transform.SetParent(pointParent, false);

			Collider collider =
				point.GetComponent<Collider>();

			if (collider != null)
			{
				Destroy(collider);
			}
		}

		point.name = $"OptiTrack Point {id}";
		point.transform.localScale =
			Vector3.one * pointScale;

		return new PointVisual
		{
			gameObject = point,
			renderers = point.GetComponentsInChildren<Renderer>(true),
			targetLocalPosition =
				point.transform.localPosition,
			lastSeenTime = Time.unscaledTime
		};
	}

	private void SmoothAndCullPointVisuals()
	{
		float now = Time.unscaledTime;

		float alpha =
			pointSmoothingRate <= 0.0f
				? 1.0f
				: 1.0f -
					Mathf.Exp(
						-pointSmoothingRate *
						Time.unscaledDeltaTime
					);

		foreach (PointVisual visual in pointVisuals.Values)
		{
			bool recentlySeen =
				now - visual.lastSeenTime <=
				pointVisibilityGraceSeconds;

			bool visible = recentlySeen && MarkerPassesHeadsetFilter(
				pointParent.TransformPoint(visual.targetLocalPosition));
			if (visual.gameObject.activeSelf != visible)
			{
				// Inactive markers are also excluded from apartment contact/haptics.
				visual.gameObject.SetActive(visible);
			}

			if (!recentlySeen)
			{
				continue;
			}
			// Keep updating hidden markers so filtering cannot interrupt alignment
			// drift or prevent them reappearing when they re-enter the box.

			visual.gameObject.transform.localPosition =
				Vector3.Lerp(
					visual.gameObject.transform.localPosition,
					visual.targetLocalPosition,
					alpha
				);

			// Hide only the rendering; keep tracking and apartment contact active.
			bool tooCloseToCamera =
				vrCamera != null &&
				(visual.gameObject.transform.position - vrCamera.position).sqrMagnitude <=
				minimumCameraDistanceMeters * minimumCameraDistanceMeters;

			foreach (Renderer pointRenderer in visual.renderers)
			{
				if (pointRenderer != null)
				{
					pointRenderer.forceRenderingOff = tooCloseToCamera;
				}
			}
		}
	}

	private float nextContactCheckTime;

	[Tooltip("Seconds between collision tests. 0.04 means 25 Hz.")]
	[Min(0.0f)]
	[SerializeField]
	private float contactCheckIntervalSeconds = 0.04f;

	[Tooltip("Log the point and apartment collider responsible for a hit.")]
	[SerializeField]
	private bool logIndividualContacts = false;

	private void DetectApartmentContact()
	{
		if (Time.unscaledTime < nextContactCheckTime)
		{
			return;
		}

		nextContactCheckTime =
			Time.unscaledTime +
			contactCheckIntervalSeconds;

		if (apartmentColliderSet.Count == 0)
		{
			TouchingPointCount = 0;
			consecutiveContactFrames = 0;
			SetApartmentContactState(false);
			return;
		}

		int touchingPoints = 0;

		foreach (KeyValuePair<int, PointVisual> entry in pointVisuals)
		{
			int pointId = entry.Key;
			PointVisual visual = entry.Value;

			if (visual == null ||
				visual.gameObject == null ||
				!visual.gameObject.activeInHierarchy)
			{
				continue;
			}

			if (!IsPointSphereIntersectingApartment(
					visual.gameObject.transform.position,
					out Collider hitCollider))
			{
				continue;
			}

			touchingPoints++;

			if (logIndividualContacts)
			{
				Debug.Log(
					$"Mocap point {pointId} intersects " +
					$"{hitCollider.name} at " +
					$"{visual.gameObject.transform.position:F3}"
				);
			}

			if (touchingPoints >= requiredTouchingPoints)
			{
				break;
			}
		}

		TouchingPointCount = touchingPoints;

		bool contactThisFrame =
			touchingPoints >= requiredTouchingPoints;

		if (contactThisFrame)
		{
			consecutiveContactFrames++;
		}
		else
		{
			consecutiveContactFrames = 0;
		}

		bool confirmedContact =
			consecutiveContactFrames >=
			requiredContactFrames;

		SetApartmentContactState(confirmedContact);
	}

	private bool IsPointSphereIntersectingApartment(
		Vector3 pointWorldPosition,
		out Collider hitCollider)
	{
		hitCollider = null;

		int hitCount =
			Physics.OverlapSphereNonAlloc(
				pointWorldPosition,
				contactRadiusMeters,
				overlapResults,
				~0,
				QueryTriggerInteraction.Ignore
			);

		int usableHitCount =
			Mathf.Min(
				hitCount,
				overlapResults.Length
			);

		for (int i = 0; i < usableHitCount; ++i)
		{
			Collider candidate =
				overlapResults[i];

			overlapResults[i] = null;

			if (candidate == null ||
				!candidate.enabled ||
				!candidate.gameObject.activeInHierarchy)
			{
				continue;
			}

			if (!apartmentColliderSet.Contains(candidate))
			{
				continue;
			}

			hitCollider = candidate;
			ClearOverlapResultBuffer(usableHitCount);
			return true;
		}

		ClearOverlapResultBuffer(usableHitCount);
		return false;
	}

	private void ClearOverlapResultBuffer(int usedCount)
	{
		for (int i = 0;
			i < usedCount &&
			i < overlapResults.Length;
			++i)
		{
			overlapResults[i] = null;
		}
	}

	private void SetApartmentContactState(bool touching)
	{
		IsTouchingApartment = touching;

		bool contactJustBegan =
			IsTouchingApartment &&
			!previousTouchingState;

		bool contactJustEnded =
			!IsTouchingApartment &&
			previousTouchingState;

		if (contactJustBegan)
		{
			SendCollisionHaptics();

			nextHapticTime =
				Time.unscaledTime +
				hapticRepeatIntervalSeconds;

			if (logContactChanges)
			{
				Debug.Log(
					"Mocap contact began with apartment_1."
				);
			}
		}
		else if (
			IsTouchingApartment &&
			repeatHapticsWhileTouching &&
			Time.unscaledTime >= nextHapticTime)
		{
			SendCollisionHaptics();

			nextHapticTime =
				Time.unscaledTime +
				hapticRepeatIntervalSeconds;
		}

		if (contactJustEnded &&
			logContactChanges)
		{
			Debug.Log(
				"Mocap contact ended with apartment_1."
			);
		}

		previousTouchingState =
			IsTouchingApartment;
	}

	private void SendCollisionHaptics()
	{
		RefreshControllerIfNeeded(
			ref leftController,
			XRNode.LeftHand
		);

		RefreshControllerIfNeeded(
			ref rightController,
			XRNode.RightHand
		);

		SendHapticImpulse(leftController);
		SendHapticImpulse(rightController);
	}

	private static void RefreshControllerIfNeeded(
		ref InputDevice device,
		XRNode node)
	{
		if (!device.isValid)
		{
			device =
				InputDevices.GetDeviceAtXRNode(node);
		}
	}

	private void SendHapticImpulse(InputDevice device)
	{
		if (!device.isValid)
		{
			return;
		}

		if (!device.TryGetHapticCapabilities(
				out HapticCapabilities capabilities))
		{
			return;
		}

		if (!capabilities.supportsImpulse)
		{
			return;
		}

		device.SendHapticImpulse(
			0,
			collisionHapticAmplitude,
			collisionHapticDurationSeconds
		);
	}

	private void LogRateDiagnostics()
	{
		if (Time.unscaledTime < nextDiagnosticsTime)
		{
			return;
		}

		long received =
			Interlocked.Exchange(
				ref packetsReceived,
				0
			);

		long parsed =
			Interlocked.Exchange(
				ref parsedFrames,
				0
			);

		long failures =
			Interlocked.Exchange(
				ref parseFailures,
				0
			);

		Debug.Log(
			$"OptiTrack | UDP {received}/s | parsed {parsed}/s | " +
			$"displayed {displayedFrames}/s | last frame {lastDisplayedFrameNumber} | " +
			$"parse failures {failures} | touching apartment {IsTouchingApartment}" +
			$" | local frame age {LocalFrameAgeMilliseconds():F1} ms" +
			$" | alignment error {AlignmentErrorMeters():F3} m" +
			$" | pose age {(Time.unscaledTime - acceptedPoseArrivalTime):F3} s" +
			$" | smoothing point={pointSmoothingRate:F2}, position={positionCorrectionRate:F2}" +
			$" | PICO {latestMotivePoseStatus}"
		);

		displayedFrames = 0;
		nextDiagnosticsTime =
			Time.unscaledTime + 1.0f;
	}

	// Local age starts when Unity receives UDP, not at camera capture on the Motive PC.
	private double LocalFrameAgeMilliseconds()
	{
		return lastDisplayedReceiveTimestamp == 0 ? -1.0 :
			(System.Diagnostics.Stopwatch.GetTimestamp() - lastDisplayedReceiveTimestamp) *
			1000.0 / System.Diagnostics.Stopwatch.Frequency;
	}

	private float AlignmentErrorMeters()
	{
		if (!hasPointAlignment || vrCamera == null) return -1f;
		return Vector3.Distance(vrCamera.position,
			mocapToUnityRotation * acceptedMotiveCameraPosition + mocapToUnityTranslation);
	}

	private Vector3 ConvertPosition(
		Vector3 motivePosition)
	{
		Vector3 unityPosition;

		if (convertFromMotiveZUp)
		{
			unityPosition =
				new Vector3(
					-motivePosition.y,
					motivePosition.z,
					motivePosition.x
				);
		}
		else
		{
			unityPosition = motivePosition;
		}

		return unityPosition *
			positionScale +
			positionOffset;
	}

	private Vector3 ConvertDirection(
		Vector3 motiveDirection)
	{
		if (!convertFromMotiveZUp)
		{
			return motiveDirection.normalized;
		}

		return new Vector3(
			-motiveDirection.y,
			motiveDirection.z,
			motiveDirection.x
		).normalized;
	}

	private Quaternion ConvertMotiveRotation(
		Quaternion motiveRotation)
	{
		float magnitudeSquared =
			motiveRotation.x * motiveRotation.x +
			motiveRotation.y * motiveRotation.y +
			motiveRotation.z * motiveRotation.z +
			motiveRotation.w * motiveRotation.w;

		if (magnitudeSquared < 0.000001f)
		{
			return Quaternion.identity;
		}

		float inverseMagnitude =
			1.0f /
			Mathf.Sqrt(magnitudeSquared);

		motiveRotation =
			new Quaternion(
				motiveRotation.x * inverseMagnitude,
				motiveRotation.y * inverseMagnitude,
				motiveRotation.z * inverseMagnitude,
				motiveRotation.w * inverseMagnitude
			);

		Vector3 motiveForward =
			RotateMotiveVector(
				motiveRotation,
				motiveLocalForwardAxis.normalized
			);

		Vector3 motiveUp =
			RotateMotiveVector(
				motiveRotation,
				motiveLocalUpAxis.normalized
			);

		Vector3 unityForward =
			ConvertDirection(motiveForward);

		Vector3 unityUp =
			ConvertDirection(motiveUp);

		if (unityForward.sqrMagnitude < 0.000001f ||
			unityUp.sqrMagnitude < 0.000001f)
		{
			return Quaternion.identity;
		}

		if (Mathf.Abs(
				Vector3.Dot(
					unityForward,
					unityUp
				)) > 0.999f)
		{
			Debug.LogWarning(
				"Pico forward and up axes are nearly parallel. " +
				"Check the rigid-body axis settings."
			);

			return Quaternion.identity;
		}

		return Quaternion.LookRotation(
			unityForward,
			unityUp
		);
	}

	private static Vector3 RotateMotiveVector(
		Quaternion rotation,
		Vector3 vector)
	{
		return rotation * vector;
	}

	private void OnDrawGizmosSelected()
	{
		foreach (PointVisual visual in pointVisuals.Values)
		{
			if (visual == null ||
				visual.gameObject == null ||
				!visual.gameObject.activeInHierarchy)
			{
				continue;
			}

			Gizmos.DrawWireSphere(
				visual.gameObject.transform.position,
				contactRadiusMeters
			);
		}
	}

	private void OnDisable()
	{
		running = false;

		if (udpClient != null)
		{
			udpClient.Close();
			udpClient = null;
		}

		if (receiveThread != null &&
			receiveThread.IsAlive)
		{
			receiveThread.Join(500);
		}

		receiveThread = null;
		lock (latestFrameLock) { latestFrame = null; }
		// Generated markers are not serialized with this dictionary across a domain
		// reload. Remove them so a reload cannot leave a frozen duplicate cloud.
		foreach (PointVisual visual in pointVisuals.Values)
		{
			if (visual.gameObject != null) Destroy(visual.gameObject);
		}
		pointVisuals.Clear();
	}
}
