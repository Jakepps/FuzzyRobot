using UnityEngine;

namespace FuzzyRobot
{
    public sealed class RobotObstacleSensors : MonoBehaviour
    {
        [Header("Raycast")]
        [SerializeField] private Transform origin; // если null — берём transform
        [SerializeField] private float originHeight = 0.25f;
        [SerializeField] private float maxDistance = 10f;
        [SerializeField] private float sideAngleDeg = 35f;
        [SerializeField] private LayerMask obstacleMask = ~0;
        [SerializeField] private QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

        [Header("Debug")]
        [SerializeField] private bool debugDraw = true;

        public float MaxDistance => maxDistance;

        public void ReadDistances(out float left, out float center, out float right)
        {
            var t = origin != null ? origin : transform;
            Vector3 p = t.position + Vector3.up * originHeight;

            center = Cast(p, t.forward, Color.green);

            Vector3 leftDir = Quaternion.AngleAxis(-sideAngleDeg, Vector3.up) * t.forward;
            left = Cast(p, leftDir, Color.cyan);

            Vector3 rightDir = Quaternion.AngleAxis(sideAngleDeg, Vector3.up) * t.forward;
            right = Cast(p, rightDir, Color.magenta);

            // В fuzzy-коде расстояния ожидаются в диапазоне [0..10]
            // Здесь maxDistance по умолчанию 10м, поэтому дополнительно нормировать не нужно.
            left = Mathf.Clamp(left, 0f, maxDistance);
            center = Mathf.Clamp(center, 0f, maxDistance);
            right = Mathf.Clamp(right, 0f, maxDistance);
        }

        private float Cast(Vector3 originPos, Vector3 dir, Color c)
        {
            if (Physics.Raycast(originPos, dir, out var hit, maxDistance, obstacleMask, triggerInteraction))
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
            
            return maxDistance; // “далеко” / “ничего не вижу”
        }
    }
}
