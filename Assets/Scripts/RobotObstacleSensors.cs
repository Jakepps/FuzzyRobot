using UnityEngine;

namespace FuzzyRobot
{
    public sealed class RobotObstacleSensors : MonoBehaviour
    {
        [Header("Raycast")]
        [SerializeField] private float originHeight = 0.25f;
        [SerializeField] private float maxDistance = 10f;
        [SerializeField] private float sideAngleDeg = 35f;
        [SerializeField] private LayerMask obstacleMask = ~0;

        [Header("Debug")]
        [SerializeField] private bool debugDraw = true;

        public void ReadDistances(Vector3 worldPosition, Vector3 forward, out float left, out float center, out float right)
        {
            Vector3 position = worldPosition + Vector3.up * originHeight;

            forward.y = 0f;
            if (forward.sqrMagnitude < 1e-6f)
            {
                forward = Vector3.forward;
            }
            forward.Normalize();

            center = Cast(position, forward, Color.green);

            Vector3 leftDir = Quaternion.AngleAxis(-sideAngleDeg, Vector3.up) * forward;
            left = Cast(position, leftDir, Color.cyan);

            Vector3 rightDir = Quaternion.AngleAxis(sideAngleDeg, Vector3.up) * forward;
            right = Cast(position, rightDir, Color.magenta);

            left = Mathf.Clamp(left, 0f, maxDistance);
            center = Mathf.Clamp(center, 0f, maxDistance);
            right = Mathf.Clamp(right, 0f, maxDistance);
        }

        private float Cast(Vector3 originPos, Vector3 dir, Color c)
        {
            dir.Normalize();

            if (Physics.Raycast(originPos, dir, out RaycastHit hit, maxDistance, obstacleMask, QueryTriggerInteraction.Ignore))
            {
                if (debugDraw)
                {
                    Debug.DrawLine(originPos, hit.point, c);
                }
                
                return hit.distance;
            }

            if (debugDraw)
            {
                Debug.DrawLine(originPos, originPos + dir * maxDistance, c);
            }
            
            return maxDistance;
        }
    }
}
