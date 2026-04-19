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

        [Header("Spot response")]
        [SerializeField] private bool useSpotConeCheck = true;
        [SerializeField] private float edgeSoftness = 0.15f;

        [Header("Debug")]
        [SerializeField] private bool debug;

        public float CurrentIllumination { get; private set; }

        public float SampleIllumination(Vector3 worldPosition)
        {
            Vector3 samplePos = worldPosition + Vector3.up * sampleHeight;
            float total = 0f;

            foreach (var src in sources)
            {
                Vector3 fromSourceToSample = samplePos - src.transform.position;
                float dist = fromSourceToSample.magnitude;

                if (dist > src.Radius || dist < 1e-4f)
                {
                    continue;
                }

                Vector3 dirToSample = fromSourceToSample / dist;

                if (requireLineOfSight)
                {
                    if (Physics.Linecast(
                            samplePos,
                            src.transform.position,
                            occluderMask,
                            QueryTriggerInteraction.Ignore))
                    {
                        continue;
                    }
                }

                float distanceFactor = 1f - dist / src.Radius;
                distanceFactor *= distanceFactor;

                float coneFactor = 1f;

                if (useSpotConeCheck && src.SourceType == LightType.Spot)
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
                    Color c = src.SourceType == LightType.Spot ? Color.Lerp(Color.red, Color.yellow, coneFactor) : Color.yellow;
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
