using UnityEngine;

namespace FuzzyRobot
{
    [ExecuteAlways]
    public class RobotIlluminationSource : MonoBehaviour
    {
        [SerializeField] private float radius = 8f;
        [SerializeField, Range(0f, 1000f)] private float intensity = 400f;
        [SerializeField] private bool useAttachedLightRange = true;
        [SerializeField] private Light attachedLight;

        [Header("Gizmos")]
        [SerializeField] private bool drawSolidGizmo = true;
        [SerializeField] private Color gizmoColor = new Color(1f, 0.85f, 0.2f, 0.8f);
        [SerializeField] private Color gizmoFillColor = new Color(1f, 0.85f, 0.2f, 0.15f);

        public float Radius => radius;
        public float Intensity => intensity;
        public Vector3 Direction => transform.forward;

        public LightType SourceType =>
            attachedLight != null ? attachedLight.type : LightType.Point;

        public float SpotAngleDeg =>
            attachedLight != null && attachedLight.type == LightType.Spot
                ? attachedLight.spotAngle
                : 360f;

        private void OnValidate()
        {
            if (attachedLight == null)
            {
                attachedLight = GetComponent<Light>();
            }

            if (attachedLight != null)
            {
                intensity = attachedLight.intensity;
            }

            if (useAttachedLightRange && attachedLight != null)
            {
                radius = attachedLight.range;
            }
        }

        private void OnDrawGizmosSelected()
        {
            if (attachedLight == null)
            {
                attachedLight = GetComponent<Light>();
            }

            if (attachedLight != null && attachedLight.type == LightType.Spot)
            {
                DrawSpotLightGizmo();
            }
            else
            {
                DrawSphereGizmo();
            }
        }

        private void DrawSphereGizmo()
        {
            Gizmos.color = gizmoFillColor;
            if (drawSolidGizmo)
            {
                Gizmos.DrawSphere(transform.position, radius);
            }

            Gizmos.color = gizmoColor;
            Gizmos.DrawWireSphere(transform.position, radius);
        }

        private void DrawSpotLightGizmo()
        {
            float range = radius;
            float halfAngle = SpotAngleDeg * 0.5f;

            Vector3 origin = transform.position;
            Vector3 forward = transform.forward;

            float coneRadius = Mathf.Tan(halfAngle * Mathf.Deg2Rad) * range;
            Vector3 endCenter = origin + forward * range;

            Vector3 up = transform.up * coneRadius;
            Vector3 right = transform.right * coneRadius;

            Vector3 p1 = endCenter + up;
            Vector3 p2 = endCenter - up;
            Vector3 p3 = endCenter + right;
            Vector3 p4 = endCenter - right;

            Gizmos.color = gizmoColor;

            Gizmos.DrawLine(origin, p1);
            Gizmos.DrawLine(origin, p2);
            Gizmos.DrawLine(origin, p3);
            Gizmos.DrawLine(origin, p4);
            Gizmos.DrawLine(origin, endCenter);

            DrawWireCircle(endCenter, forward, coneRadius, 28);

            if (drawSolidGizmo)
            {
                Gizmos.color = gizmoFillColor;
                DrawConeFillApprox(origin, endCenter, forward, coneRadius, 20);
            }
        }

        private void DrawWireCircle(Vector3 center, Vector3 normal, float circleRadius, int segments)
        {
            Vector3 tangent = Vector3.Cross(normal, Vector3.up);
            if (tangent.sqrMagnitude < 0.0001f)
            {
                tangent = Vector3.Cross(normal, Vector3.right);
            }

            tangent.Normalize();
            Vector3 bitangent = Vector3.Cross(normal, tangent).normalized;

            Vector3 prevPoint = center + tangent * circleRadius;

            for (int i = 1; i <= segments; i++)
            {
                float t = (float)i / segments * Mathf.PI * 2f;
                Vector3 nextPoint =
                    center + (tangent * Mathf.Cos(t) + bitangent * Mathf.Sin(t)) * circleRadius;

                Gizmos.DrawLine(prevPoint, nextPoint);
                prevPoint = nextPoint;
            }
        }

        private void DrawConeFillApprox(Vector3 origin, Vector3 endCenter, Vector3 normal, float endRadius, int segments)
        {
            Vector3 tangent = Vector3.Cross(normal, Vector3.up);
            if (tangent.sqrMagnitude < 0.0001f)
            {
                tangent = Vector3.Cross(normal, Vector3.right);
            }

            tangent.Normalize();
            Vector3 bitangent = Vector3.Cross(normal, tangent).normalized;

            Vector3 prev = endCenter + tangent * endRadius;

            for (int i = 1; i <= segments; i++)
            {
                float t = (float)i / segments * Mathf.PI * 2f;
                Vector3 next =
                    endCenter + (tangent * Mathf.Cos(t) + bitangent * Mathf.Sin(t)) * endRadius;

                Gizmos.DrawLine(origin, prev);
                Gizmos.DrawLine(prev, next);

                prev = next;
            }
        }
    }
}
