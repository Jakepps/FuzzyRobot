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
        [Tooltip("Максимальная логическая скорость поворота (град/с), соответствующая |phi| = 45.")]
        [SerializeField] private float maxYawRateDeg = 180f;

        [Tooltip("Максимальное изменение угловой скорости (град/с²).")]
        [SerializeField] private float maxYawAccelDeg = 720f;

        [Header("Goal")]
        [SerializeField] private float stopDistance = 0.5f;
        [SerializeField] private float stopBrakeAccelMs2 = 15f;

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

            // 1) Считываем сенсоры по логическому forward
            sensors.ReadDistances(position, _planarForward, out float dLeft, out float dCenter, out float dRight);

            // 2) Угол на цель
            float angleErrDeg = 0f;
            if (target != null)
            {
                Vector3 toTarget = Flatten(target.position - position);
                if (toTarget.sqrMagnitude > Epsilon)
                {
                    angleErrDeg = Vector3.SignedAngle(_planarForward, toTarget.normalized, Vector3.up);
                    angleErrDeg = Mathf.Clamp(angleErrDeg, -90f, 90f);
                }
            }

            // 3) Текущая скорость в fuzzy units
            float speedFuzzy = Mathf.Clamp(planarSpeed / Mathf.Max(0.001f, speedAt100) * 100f, 0f, 100f);

            // 4) Нечёткий шаг
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

            float dt = Time.fixedDeltaTime;

            // 5) Обновляем логическое направление
            float desiredYawRateDeg = Mathf.Clamp(outCmd.Phi / 45f, -1f, 1f) * maxYawRateDeg;
            _currentYawRateDeg = Mathf.MoveTowards(_currentYawRateDeg, desiredYawRateDeg, maxYawAccelDeg * dt);

            _planarForward = Quaternion.AngleAxis(_currentYawRateDeg * dt, Vector3.up) * _planarForward;
            _planarForward = Flatten(_planarForward).normalized;

            // 6) Перевод ΔV обратно в м/с
            float deltaSpeedMs = outCmd.DeltaV / 100f * speedAt100;

            float currentForwardSpeed = Vector3.Dot(planarVelocity, _planarForward);
            float desiredSpeedMs = Mathf.Clamp(currentForwardSpeed + deltaSpeedMs, 0f, maxSpeedMs);

            // Разгон/торможение вдоль логического направления
            float forwardAccel = (desiredSpeedMs - currentForwardSpeed) / Mathf.Max(0.001f, dt);
            forwardAccel = Mathf.Clamp(forwardAccel, -maxDecelMs2, maxAccelMs2);

            // Подавление бокового сноса
            Vector3 lateralVelocity = planarVelocity - _planarForward * currentForwardSpeed;
            Vector3 lateralAccel = Vector3.zero;
            if (lateralVelocity.sqrMagnitude > Epsilon)
            {
                lateralAccel = Vector3.ClampMagnitude(-lateralVelocity / Mathf.Max(0.001f, dt), lateralDampingAccelMs2);
            }

            rigidbodyDriver.AddForce(_planarForward * forwardAccel + lateralAccel, ForceMode.Acceleration);

            UpdateHeadingVisual();

            if (debugLog)
            {
                Debug.LogError(
                    $"[FuzzyRobot] dL={dLeft:F2} dC={dCenter:F2} dR={dRight:F2} " +
                    $"ang={angleErrDeg:F1} speed={speedFuzzy:F1} dv={outCmd.DeltaV:F2} phi={outCmd.Phi:F2}"
                );
            }
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
