using UnityEngine;
using UnityEngine.Rendering;

// The shader draws directly in each eye's clip space, independent of camera projection.
[DisallowMultipleComponent]
public sealed class CollisionScreenBorder : MonoBehaviour
{
    [Tooltip("Border tint and maximum opacity while a mocap point touches the apartment.")]
    [SerializeField]
    private Color borderColor = new Color(1f, 0f, 0f, 0.35f);

    [Tooltip("Fraction of the screen occupied by the soft border on each edge.")]
    [Range(0.01f, 0.5f)]
    [SerializeField]
    private float borderWidth = 0.1f;

    [Tooltip("Opacity change per second when contact begins or ends.")]
    [Min(0.01f)]
    [SerializeField]
    private float fadeSpeed = 3f;

    private OptiTrackPointReceiver receiver;
    private Camera viewCamera;
    private GameObject borderObject;
    private Mesh borderMesh;
    private MeshRenderer borderRenderer;
    private Material borderMaterial;
    private float opacity;

    private static readonly int ColorId = Shader.PropertyToID("_Color");
    private static readonly int WidthId = Shader.PropertyToID("_BorderWidth");

    public void Initialize(OptiTrackPointReceiver source, Camera camera)
    {
        receiver = source;
        viewCamera = camera;
        if (viewCamera == null || borderObject != null) return;

        // Resources keeps the runtime-created material's shader included in player builds.
        Shader shader = Resources.Load<Shader>("MocapCollisionBorder");
        if (shader == null)
        {
            Debug.LogError("CollisionScreenBorder: MocapCollisionBorder shader is missing.", this);
            return;
        }

        borderMaterial = new Material(shader) { enableInstancing = true };
        borderMaterial.SetColor(ColorId, new Color(borderColor.r, borderColor.g, borderColor.b, 0f));
        borderMaterial.SetFloat(WidthId, Mathf.Clamp(borderWidth, 0.01f, 0.5f));
        borderMesh = new Mesh
        {
            name = "Mocap Collision Fullscreen Quad",
            vertices = new[]
            {
                new Vector3(-1f, -1f, 0f), new Vector3(1f, -1f, 0f),
                new Vector3(-1f, 1f, 0f), new Vector3(1f, 1f, 0f)
            },
            uv = new[] { Vector2.zero, Vector2.right, Vector2.up, Vector2.one },
            triangles = new[] { 0, 2, 1, 2, 3, 1 },
            // Vertex positions are clip coordinates. Keep the camera inside the culling bounds.
            bounds = new Bounds(Vector3.zero, Vector3.one * 1000f)
        };

        borderObject = new GameObject("Mocap Collision Screen Border", typeof(MeshFilter), typeof(MeshRenderer));
        borderObject.layer = viewCamera.gameObject.layer;
        borderObject.transform.SetParent(viewCamera.transform, false);
        borderObject.GetComponent<MeshFilter>().sharedMesh = borderMesh;
        borderRenderer = borderObject.GetComponent<MeshRenderer>();
        borderRenderer.sharedMaterial = borderMaterial;
        borderRenderer.shadowCastingMode = ShadowCastingMode.Off;
        borderRenderer.receiveShadows = false;
        borderRenderer.lightProbeUsage = LightProbeUsage.Off;
        borderRenderer.reflectionProbeUsage = ReflectionProbeUsage.Off;
        borderRenderer.allowOcclusionWhenDynamic = false;
        borderRenderer.sortingOrder = short.MaxValue;
        borderRenderer.enabled = false;
    }

    private void LateUpdate()
    {
        if (borderMaterial == null || viewCamera == null) return;

        bool touching = receiver != null && receiver.isActiveAndEnabled && receiver.IsTouchingApartment;
        float targetOpacity = touching ? Mathf.Clamp01(borderColor.a) : 0f;
        opacity = Mathf.MoveTowards(opacity, targetOpacity, Mathf.Max(0.01f, fadeSpeed) * Time.unscaledDeltaTime);

        borderMaterial.SetColor(ColorId, new Color(borderColor.r, borderColor.g, borderColor.b, opacity));
        borderMaterial.SetFloat(WidthId, Mathf.Clamp(borderWidth, 0.01f, 0.5f));
    }

    private void OnEnable()
    {
        Camera.onPreCull += OnCameraPreCull;
    }

    private void OnCameraPreCull(Camera camera)
    {
        // Do not draw the fullscreen warning into unrelated cameras or the Scene view.
        if (borderRenderer != null)
            borderRenderer.enabled = camera == viewCamera && opacity > 0f && isActiveAndEnabled;
    }

    private void OnDisable()
    {
        opacity = 0f;
        Camera.onPreCull -= OnCameraPreCull;
        if (borderRenderer != null) borderRenderer.enabled = false;
    }

    private void OnDestroy()
    {
        if (borderObject != null) Destroy(borderObject);
        if (borderMesh != null) Destroy(borderMesh);
        if (borderMaterial != null) Destroy(borderMaterial);
    }
}
