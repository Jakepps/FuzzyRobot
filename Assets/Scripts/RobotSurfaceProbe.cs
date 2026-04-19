using UnityEngine;

namespace FuzzyRobot
{
    public class RobotSurfaceProbe : MonoBehaviour
    {
        [Header("Probe")]
        [SerializeField] private float rayHeight = 0.5f;
        [SerializeField] private float rayDistance = 2f;
        [SerializeField] private LayerMask groundMask = ~0;

        [Header("Surface layers")]
        [SerializeField] private LayerMask smoothLayer;
        [SerializeField] private LayerMask roughLayer;
        [SerializeField] private LayerMask slipperyLayer;

        [Header("Debug")]
        [SerializeField] private bool debug = true;

        public float CurrentSurfaceValue { get; private set; }

        public void TryReadSurface(Vector3 worldPosition, out float surfaceValue, out RaycastHit hitInfo)
        {
            Vector3 origin = worldPosition + Vector3.up * rayHeight;

            if (Physics.Raycast(origin, Vector3.down, out hitInfo, rayDistance, groundMask, QueryTriggerInteraction.Ignore))
            {
                int layer = hitInfo.collider.gameObject.layer;

                if (IsLayerInMask(layer, smoothLayer))
                {
                    surfaceValue = 0f; // smooth
                }
                else if (IsLayerInMask(layer, roughLayer))
                {
                    surfaceValue = 1f; // rough
                }
                else if (IsLayerInMask(layer, slipperyLayer))
                {
                    surfaceValue = 2f; // slippery
                }
                else
                {
                    surfaceValue = 0f;
                }

                CurrentSurfaceValue = surfaceValue;

                if (debug)
                {
                    Debug.DrawLine(origin, hitInfo.point, Color.yellow);
                }

                return;
            }

            surfaceValue = CurrentSurfaceValue;
        }

        private static bool IsLayerInMask(int layer, LayerMask mask)
        {
            return (mask.value & (1 << layer)) != 0;
        }
    }
}
