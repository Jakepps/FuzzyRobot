using UnityEngine;

namespace FuzzyRobot
{
    public class RobotIlluminationProbe : MonoBehaviour
    {
        [SerializeField] private float sampleHeight = 0.5f;
        [SerializeField] private LayerMask occluderMask = ~0;
        [SerializeField] private bool requireLineOfSight = true;

        [Header("Normalization")]
        [SerializeField] private float maxIllumination = 1000f;
        
        [Header("Illumination sources")]
        [SerializeField] private RobotIlluminationSource[] sources;
        [SerializeField] private bool autoFindSources = true;
        [SerializeField] private float sourceRefreshInterval = 1f;

        [Header("Spot response")]
        [SerializeField] private bool useSpotConeCheck = true;
        [SerializeField] private float edgeSoftness = 0.15f;

        [Header("Debug")]
        [SerializeField] private bool debug;

        public float CurrentIllumination { get; private set; }

        private readonly RaycastHit[] _hitsBuffer = new RaycastHit[16];
        private float _nextSourceRefreshTime;

        private void Awake()
        {
            RefreshSourcesIfNeeded(true);
        }

        private void OnEnable()
        {
            RefreshSourcesIfNeeded(true);
        }

        public float SampleIllumination(Vector3 worldPosition)
        {
            RefreshSourcesIfNeeded(false);

            Vector3 samplePos = worldPosition + Vector3.up * sampleHeight;
            float total = 0f;

            if (sources == null || sources.Length == 0)
            {
                CurrentIllumination = 0f;
                return CurrentIllumination;
            }

            foreach (var src in sources)
            {
                if (src == null || !src.IsActive)
                {
                    continue;
                }

                Vector3 fromSourceToSample = samplePos - src.transform.position;
                float dist = fromSourceToSample.magnitude;

                if (dist > src.Radius || dist < 1e-4f)
                {
                    continue;
                }

                Vector3 dirToSample = fromSourceToSample / dist;

                if (requireLineOfSight)
                {
                    if (!HasLineOfSightToSource(samplePos, src, dist))
                    {
                        continue;
                    }
                }

                float distanceFactor = 1f - dist / src.Radius;
                distanceFactor *= distanceFactor;

                float coneFactor = 1f;

                if (src.SourceType == LightType.Point)
                {
                    coneFactor = 1f;
                }
                else if (useSpotConeCheck && src.SourceType == LightType.Spot)
                {
                    coneFactor = EvaluateSpotConeFactor(src, dirToSample);

                    if (coneFactor <= 0f)
                    {
                        continue;
                    }
                }

                float contribution = src.Intensity * distanceFactor * coneFactor;
                total += contribution;

                if (debug)
                {
                    Color c = src.SourceType switch
                    {
                        LightType.Point => Color.cyan,
                        LightType.Spot => Color.Lerp(Color.red, Color.yellow, coneFactor),
                        _ => Color.yellow
                    };
                    Debug.DrawLine(src.transform.position, samplePos, c);
                }
            }

            CurrentIllumination = Mathf.Clamp(total, 0f, maxIllumination);

            if (debug)
            {
                Debug.Log($"Current Illumination: {CurrentIllumination}");
            }
            
            return CurrentIllumination;
        }

        private void RefreshSourcesIfNeeded(bool force)
        {
            if (!autoFindSources)
            {
                return;
            }

            if (!force && Time.time < _nextSourceRefreshTime && sources is { Length: > 0 })
            {
                return;
            }

            sources = FindObjectsByType<RobotIlluminationSource>(FindObjectsSortMode.None);
            _nextSourceRefreshTime = Time.time + Mathf.Max(0.1f, sourceRefreshInterval);
        }

        private bool HasLineOfSightToSource(Vector3 samplePos, RobotIlluminationSource src, float distance)
        {
            Vector3 direction = src.transform.position - samplePos;
            if (direction.sqrMagnitude < 1e-6f)
            {
                return true;
            }

            direction /= distance;

            int hitCount = Physics.RaycastNonAlloc(
                samplePos,
                direction,
                _hitsBuffer,
                distance,
                occluderMask,
                QueryTriggerInteraction.Ignore);

            for (int i = 0; i < hitCount; i++)
            {
                Collider hitCollider = _hitsBuffer[i].collider;
                if (hitCollider == null)
                {
                    continue;
                }

                Transform hitTransform = hitCollider.transform;
                if (hitTransform == transform ||
                    hitTransform.IsChildOf(transform) ||
                    hitTransform == src.transform ||
                    hitTransform.IsChildOf(src.transform))
                {
                    continue;
                }

                return false;
            }

            return true;
        }

        private float EvaluateSpotConeFactor(RobotIlluminationSource src, Vector3 dirToSample)
        {
            float halfAngle = src.SpotAngleDeg * 0.5f;
            float angleToSample = Vector3.Angle(src.Direction, dirToSample);

            if (angleToSample > halfAngle)
            {
                return 0f;
            }

            float softZone = Mathf.Clamp01(edgeSoftness) * halfAngle;
            float hardInnerAngle = Mathf.Max(0f, halfAngle - softZone);

            if (softZone <= 1e-4f || angleToSample <= hardInnerAngle)
            {
                return 1f;
            }

            float t = Mathf.InverseLerp(halfAngle, hardInnerAngle, angleToSample);
            return Mathf.Clamp01(t * t * (3f - 2f * t));
        }
    }
}
