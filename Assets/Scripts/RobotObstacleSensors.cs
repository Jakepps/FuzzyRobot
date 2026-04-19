using UnityEngine;

namespace FuzzyRobot
{
    public class RobotObstacleSensors : MonoBehaviour
    {
        [Header("Raycast")]
        [SerializeField] private float originHeight = 0.25f;
        [SerializeField] private float maxDistance = 10f;
        [SerializeField] private float sideAngleDeg = 35f;
        [SerializeField] private LayerMask obstacleMask = ~0;

        [Header("Debug")]
        [SerializeField] private bool debugDraw = true;

        private readonly RaycastHit[] _hitsBuffer = new RaycastHit[16];

        public float MaxDistance => maxDistance;

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

        public bool HasLineOfSight(Vector3 worldPosition, Vector3 targetPosition)
        {
            Vector3 origin = worldPosition + Vector3.up * originHeight;
            Vector3 toTarget = targetPosition - origin;
            toTarget.y = 0f;

            float distance = toTarget.magnitude;
            if (distance < 1e-6f)
            {
                return true;
            }

            Vector3 dir = toTarget / distance;

            if (TryGetClosestHit(origin, dir, distance, out RaycastHit hit))
            {
                if (debugDraw)
                {
                    Debug.DrawLine(origin, hit.point, Color.red);
                }
                
                return false;
            }

            if (debugDraw)
            {
                Debug.DrawLine(origin, origin + dir * distance, Color.blue);
            }

            return true;
        }

        private float Cast(Vector3 originPos, Vector3 dir, Color color)
        {
            dir.Normalize();

            if (TryGetClosestHit(originPos, dir, maxDistance, out RaycastHit hit))
            {
                if (debugDraw)
                {
                    Debug.DrawLine(originPos, hit.point, color);
                }
                
                return hit.distance;
            }

            if (debugDraw)
            {
                Debug.DrawLine(originPos, originPos + dir * maxDistance, color);
            }

            return maxDistance;
        }

        private bool TryGetClosestHit(Vector3 originPos, Vector3 dir, float distance, out RaycastHit closestHit)
        {
            int hitCount = Physics.RaycastNonAlloc(
                originPos,
                dir,
                _hitsBuffer,
                distance,
                obstacleMask,
                QueryTriggerInteraction.Ignore);

            float bestDistance = float.PositiveInfinity;
            closestHit = default;
            bool found = false;

            for (int i = 0; i < hitCount; i++)
            {
                RaycastHit hit = _hitsBuffer[i];
                if (hit.collider == null)
                {
                    continue;
                }

                Transform hitTransform = hit.collider.transform;
                
                if (hitTransform == transform || 
                    hitTransform.IsChildOf(transform))
                {
                    continue;
                }

                if (hit.distance < bestDistance)
                {
                    bestDistance = hit.distance;
                    closestHit = hit;
                    found = true;
                }
            }

            return found;
        }
    }
}
