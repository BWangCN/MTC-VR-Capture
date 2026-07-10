using UnityEngine;

public class UIFollowHMD : MonoBehaviour
{
    public Transform hmd;
    public float distance = 1.2f;
    public float yOffset = -0.05f;
    public bool followRotation = true;

    void LateUpdate()
    {
        if (!hmd) return;

        Vector3 forward = new Vector3(hmd.forward.x, 0f, hmd.forward.z).normalized;
        if (forward.sqrMagnitude < 1e-6f) forward = hmd.forward;

        Vector3 targetPos = hmd.position + forward * distance;
        targetPos.y = hmd.position.y + yOffset;

        transform.position = targetPos;

        if (followRotation)
        {
            Vector3 lookDir = (transform.position - hmd.position);
            lookDir.y = 0f;
            if (lookDir.sqrMagnitude > 1e-6f)
                transform.rotation = Quaternion.LookRotation(lookDir.normalized, Vector3.up);
        }
    }
}

