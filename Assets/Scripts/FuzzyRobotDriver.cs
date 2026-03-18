using UnityEngine;

namespace FuzzyRobot
{
    [RequireComponent(typeof(Rigidbody))]
    public sealed class FuzzyRobotDriver : MonoBehaviour
    {
        private const float Epsilon = 1e-6f;
        
        [Header("Refs")]
        [SerializeField] private RobotObstacleSensors sensors;
        [SerializeField] private Transform target;
        [SerializeField] private Rigidbody rigidbodyDriver;

        [Tooltip("Опционально: объект для визуализации логического направления (стрелка, маркер и т.п.).")]
        [SerializeField] private Transform headingVisual;

        [Header("Speed scaling (Unity m/s <-> fuzzy 0..100)")]
        [Tooltip("Какая Unity-скорость (м/с) соответствует 100 fuzzy units.")]
        [SerializeField] private float speedAt100 = 10f;

        [Header("Motion limits")]
        [SerializeField] private float maxSpeedMs = 6f;
        [SerializeField] private float maxAccelMs2 = 8f;
        [SerializeField] private float maxDecelMs2 = 10f;
        [SerializeField] private float lateralDampingAccelMs2 = 12f;

        [Header("Turn control")]
        [Tooltip("Максимальная логическая скорость поворота (град/с).")]
        [SerializeField] private float maxYawRateDeg = 180f;

        [Tooltip("Максимальное изменение угловой скорости (град/с²).")]
        [SerializeField] private float maxYawAccelDeg = 720f;

        [Header("Goal")]
        [SerializeField] private float stopDistance = 0.5f;
        [SerializeField] private float stopBrakeAccelMs2 = 15f;
        [SerializeField] private float minGoalSpeedMs = 0.75f;
        [SerializeField] private bool requireLineOfSightToTarget = true;

        [Header("Obstacle exploration")]
        [SerializeField] private float frontBlockedDistance = 2.0f;
        [SerializeField] private float sidePreferenceThreshold = 0.25f;
        [SerializeField] private float avoidCruiseSpeedMs = 2.5f;
        [SerializeField] private float avoidCreepSpeedMs = 1.0f;

        [Header("Future extreme factors (пока можно не трогать)")]
        [SerializeField] private bool useExtremeFactors;
        [SerializeField, Range(0f, 100f)] private float battery = 50f;
        [SerializeField, Range(0f, 2f)] private float surface;
        [SerializeField, Range(0f, 1000f)] private float illumination = 200f;

        [Header("Debug")]
        [SerializeField] private bool debugLog;

        private readonly FuzzyMamdaniController _controller = new();

        /// <summary>
        /// Логическое направление движения.
        /// Оно намеренно отделено от transform.forward,
        /// потому что для катящегося шара transform.forward ненадёжен.
        /// </summary>
        private Vector3 _planarForward = Vector3.forward;

        private float _currentYawRateDeg;
        private int _avoidTurnSign = 1; // +1 = вправо, -1 = влево

        private enum NavigationMode
        {
            GoalSeek,
            AvoidObstacle
        }

        private void Awake()
        {
            if (rigidbodyDriver == null)
            {
                rigidbodyDriver = GetComponent<Rigidbody>();
            }

            InitializeForward();
        }

        private void InitializeForward()
        {
            if (target != null)
            {
                Vector3 toTarget = Flatten(target.position - transform.position);
                if (toTarget.sqrMagnitude > Epsilon)
                {
                    _planarForward = toTarget.normalized;
                }
            }

            if (_planarForward.sqrMagnitude < Epsilon)
            {
                Vector3 f = Flatten(transform.forward);
                _planarForward = f.sqrMagnitude > Epsilon ? f.normalized : Vector3.forward;
            }

            UpdateHeadingVisual();
        }

        private void FixedUpdate()
        {
            if (sensors == null || 
                rigidbodyDriver == null)
            {
                return;
            }

            Vector3 position = rigidbodyDriver.position;
            Vector3 planarVelocity = Flatten(GetLinearVelocity(rigidbodyDriver));
            float planarSpeed = planarVelocity.magnitude;

            // Если достигли цели — плавно тормозим и больше не рулём
            if (target != null)
            {
                Vector3 toTarget = Flatten(target.position - position);
                if (toTarget.sqrMagnitude <= stopDistance * stopDistance)
                {
                    BrakeToStop(planarVelocity);
                    UpdateHeadingVisual();
                    return;
                }
            }

            // 1) Сенсоры
            sensors.ReadDistances(position, _planarForward, out float dLeft, out float dCenter, out float dRight);

            // 2) Данные о цели
            bool hasTarget = target != null;
            bool targetVisible = false;
            float angleErrDeg = 0f;
            float distanceToTarget = float.PositiveInfinity;
            Vector3 toTargetDir = _planarForward;

            if (hasTarget)
            {
                Vector3 toTarget = Flatten(target.position - position);
                distanceToTarget = toTarget.magnitude;

                if (toTarget.sqrMagnitude > Epsilon)
                {
                    toTargetDir = toTarget.normalized;
                    angleErrDeg = Vector3.SignedAngle(_planarForward, toTargetDir, Vector3.up);
                    angleErrDeg = Mathf.Clamp(angleErrDeg, -90f, 90f);

                    targetVisible = !requireLineOfSightToTarget || 
                                    sensors.HasLineOfSight(position, target.position);
                }
            }

            bool frontBlocked = dCenter < frontBlockedDistance;

            NavigationMode mode =
                hasTarget && targetVisible && !frontBlocked
                    ? NavigationMode.GoalSeek
                    : NavigationMode.AvoidObstacle;

            float dt = Time.fixedDeltaTime;
            float currentForwardSpeed = Vector3.Dot(planarVelocity, _planarForward);

            Vector3 desiredForward;
            float desiredSpeedMs;

            if (mode == NavigationMode.GoalSeek)
            {
                float speedFuzzy = Mathf.Clamp(planarSpeed / Mathf.Max(0.001f, speedAt100) * 100f, 0f, 100f);

                float battInput = useExtremeFactors ? battery : 50f;
                float surfInput = useExtremeFactors ? surface : 0f;
                float illumInput = useExtremeFactors ? illumination : 200f;

                var outCmd = _controller.Compute(
                    dLeft,
                    dCenter,
                    dRight,
                    angleErrDeg,
                    speedFuzzy,
                    battInput,
                    surfInput,
                    illumInput
                );

                desiredForward = Quaternion.AngleAxis(outCmd.Phi, Vector3.up) * _planarForward;
                desiredForward = Flatten(desiredForward);

                if (desiredForward.sqrMagnitude < Epsilon)
                {
                    desiredForward = toTargetDir;
                }
                else
                {
                    desiredForward.Normalize();
                }

                float deltaSpeedMs = outCmd.DeltaV / 100f * speedAt100;
                desiredSpeedMs = Mathf.Clamp(currentForwardSpeed + deltaSpeedMs, 0f, maxSpeedMs);

                // Не даём роботу залипнуть в нулевой скорости, если цель видна и она ещё не достигнута
                if (distanceToTarget > stopDistance * 2f)
                {
                    desiredSpeedMs = Mathf.Max(desiredSpeedMs, minGoalSpeedMs);
                }

                if (debugLog)
                {
                    Debug.Log(
                        $"[FuzzyRobot][GoalSeek] dL={dLeft:F2} dC={dCenter:F2} dR={dRight:F2} " +
                        $"visible={targetVisible} ang={angleErrDeg:F1} desiredSpeed={desiredSpeedMs:F2}"
                    );
                }
            }
            else
            {
                desiredForward = ComputeAvoidanceForward(dLeft, dCenter, dRight);
                desiredSpeedMs = ComputeAvoidanceSpeed(dCenter);

                if (debugLog)
                {
                    Debug.Log(
                        $"[FuzzyRobot][Avoid] dL={dLeft:F2} dC={dCenter:F2} dR={dRight:F2} " +
                        $"turnSign={_avoidTurnSign} desiredSpeed={desiredSpeedMs:F2}"
                    );
                }
            }

            UpdateForwardTowards(desiredForward, dt);
            ApplyTranslation(planarVelocity, desiredSpeedMs, dt);
            UpdateHeadingVisual();
        }

        private Vector3 ComputeAvoidanceForward(float dLeft, float dCenter, float dRight)
        {
            // Выбираем сторону с большим просветом
            float delta = dRight - dLeft;

            if (Mathf.Abs(delta) > sidePreferenceThreshold)
            {
                _avoidTurnSign = delta > 0f ? 1 : -1;
            }

            Vector3 right = Vector3.Cross(Vector3.up, _planarForward);
            if (right.sqrMagnitude < Epsilon)
            {
                right = Vector3.right;
            }
            else
            {
                right.Normalize();
            }

            // Чем меньше пространства впереди, тем сильнее уводим вбок
            float forwardWeight = Mathf.Clamp01(dCenter / Mathf.Max(frontBlockedDistance, 0.001f));
            float sideWeight = 1.15f - 0.85f * forwardWeight;

            Vector3 desired = _planarForward * Mathf.Max(0.1f, forwardWeight)
                            + right * (_avoidTurnSign * sideWeight);

            // Если впереди почти упёрлись, делаем более резкий поворот
            if (dCenter < frontBlockedDistance * 0.35f)
            {
                desired = Quaternion.AngleAxis(85f * _avoidTurnSign, Vector3.up) * _planarForward;
            }

            desired = Flatten(desired);
            return desired.sqrMagnitude > Epsilon ? desired.normalized : _planarForward;
        }

        private float ComputeAvoidanceSpeed(float dCenter)
        {
            if (dCenter < frontBlockedDistance * 0.35f)
            {
                return avoidCreepSpeedMs;
            }

            if (dCenter < frontBlockedDistance)
            {
                return Mathf.Min(avoidCruiseSpeedMs, 1.5f);
            }

            return avoidCruiseSpeedMs;
        }

        private void UpdateForwardTowards(Vector3 desiredForward, float dt)
        {
            desiredForward = Flatten(desiredForward);
            if (desiredForward.sqrMagnitude < Epsilon)
            {
                desiredForward = _planarForward;
            }
            else
            {
                desiredForward.Normalize();
            }

            float headingErrorDeg = Vector3.SignedAngle(_planarForward, desiredForward, Vector3.up);
            float desiredYawRateDeg = Mathf.Clamp(headingErrorDeg * 4f, -maxYawRateDeg, maxYawRateDeg);

            _currentYawRateDeg = Mathf.MoveTowards(_currentYawRateDeg, desiredYawRateDeg, maxYawAccelDeg * dt);

            _planarForward = Quaternion.AngleAxis(_currentYawRateDeg * dt, Vector3.up) * _planarForward;
            _planarForward = Flatten(_planarForward);

            if (_planarForward.sqrMagnitude < Epsilon)
            {
                _planarForward = desiredForward;
            }
            else
            {
                _planarForward.Normalize();
            }
        }

        private void ApplyTranslation(Vector3 planarVelocity, float desiredSpeedMs, float dt)
        {
            float currentForwardSpeed = Vector3.Dot(planarVelocity, _planarForward);

            float forwardAccel = (desiredSpeedMs - currentForwardSpeed) / Mathf.Max(0.001f, dt);
            forwardAccel = Mathf.Clamp(forwardAccel, -maxDecelMs2, maxAccelMs2);

            Vector3 lateralVelocity = planarVelocity - _planarForward * currentForwardSpeed;
            Vector3 lateralAccel = Vector3.zero;

            if (lateralVelocity.sqrMagnitude > Epsilon)
            {
                lateralAccel = Vector3.ClampMagnitude(
                    -lateralVelocity / Mathf.Max(0.001f, dt),
                    lateralDampingAccelMs2);
            }

            rigidbodyDriver.AddForce(_planarForward * forwardAccel + lateralAccel, ForceMode.Acceleration);
        }

        private void BrakeToStop(Vector3 planarVelocity)
        {
            float dt = Time.fixedDeltaTime;

            Vector3 brakeAccel = Vector3.zero;
            if (planarVelocity.sqrMagnitude > Epsilon)
            {
                brakeAccel = Vector3.ClampMagnitude(-planarVelocity / Mathf.Max(0.001f, dt), stopBrakeAccelMs2);
            }

            rigidbodyDriver.AddForce(brakeAccel, ForceMode.Acceleration);
            _currentYawRateDeg = Mathf.MoveTowards(_currentYawRateDeg, 0f, maxYawAccelDeg * dt);
        }

        private void UpdateHeadingVisual()
        {
            if (headingVisual == null)
            {
                return;
            }

            Vector3 pos = rigidbodyDriver != null ? rigidbodyDriver.position : transform.position;
            headingVisual.position = pos;

            if (_planarForward.sqrMagnitude > Epsilon)
            {
                headingVisual.rotation = Quaternion.LookRotation(_planarForward, Vector3.up);
            }
        }

        private static Vector3 Flatten(Vector3 v)
        {
            v.y = 0f;
            return v;
        }

        private static Vector3 GetLinearVelocity(Rigidbody rb)
        {
            return rb.linearVelocity;
        }
    }
}
