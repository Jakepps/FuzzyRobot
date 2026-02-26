using UnityEngine;

namespace FuzzyRobot
{
    [RequireComponent(typeof(Rigidbody))]
    public sealed class FuzzyRobotDriver : MonoBehaviour
    {
        [Header("Refs")]
        [SerializeField] private RobotObstacleSensors sensors;
        [SerializeField] private Transform target; // опционально (для angle_err)

        [Header("Speed scaling (Unity m/s <-> fuzzy 0..100)")]
        [Tooltip("Какая Unity-скорость (м/с) соответствует 100 fuzzy units.")]
        [SerializeField] private float speedAt100 = 10f;

        [Header("Motion limits")]
        [SerializeField] private float maxSpeedMs = 6f;
        [SerializeField] private float maxAccelMs2 = 8f;
        [SerializeField] private float maxDecelMs2 = 10f;

        [Header("Yaw control")]
        [Tooltip("Максимальная угловая скорость поворота (град/с), соответствующая |phi|=45.")]
        [SerializeField] private float maxYawRateDeg = 180f;

        [Tooltip("Максимальное угловое ускорение (рад/с^2).")]
        [SerializeField] private float maxYawAccelRad = 12f;

        private Rigidbody _rb;

        private void Awake()
        {
            _rb = GetComponent<Rigidbody>();
            if (sensors == null)
            {
                sensors = GetComponent<RobotObstacleSensors>();
            }
        }

        private void FixedUpdate()
        {
            if (sensors == null) return;

            // 1) Sensors
            sensors.ReadDistances(out float dLeft, out float dCenter, out float dRight);

            // 2) Angle error (optional)
            float angleErrDeg = 0f;
            if (target != null)
            {
                Vector3 to = target.position - transform.position;
                to.y = 0f;
                if (to.sqrMagnitude > 1e-6f)
                {
                    to.Normalize();
                    angleErrDeg = Vector3.SignedAngle(transform.forward, to, Vector3.up);
                    angleErrDeg = Mathf.Clamp(angleErrDeg, -90f, 90f);
                }
            }

            // 3) Speed in fuzzy units
            float speedMs = _rb.linearVelocity.magnitude;
            float speedFuzzy = Mathf.Clamp(speedMs / Mathf.Max(0.001f, speedAt100) * 100f, 0f, 100f);

            // 4) Fuzzy step (экстремальные факторы сейчас выключены)
            var outCmd = FuzzyMamdaniController.Compute(
                dLeft, dCenter, dRight,
                angleErrDeg,
                speedFuzzy,
                useExtremeFactors: false
            );

            // 5) Convert ΔV back to m/s delta
            float deltaSpeedMs = outCmd.DeltaV / 100f * speedAt100;

            // Desired speed
            float desiredSpeedMs = Mathf.Clamp(speedMs + deltaSpeedMs, 0f, maxSpeedMs);

            // 6) Linear acceleration control (вдоль forward)
            float dt = Time.fixedDeltaTime;
            Vector3 fwd = transform.forward;

            float currentForwardSpeed = Vector3.Dot(_rb.linearVelocity, fwd);
            float speedErr = desiredSpeedMs - currentForwardSpeed;

            float accel = speedErr / Mathf.Max(0.001f, dt);
            accel = Mathf.Clamp(accel, -maxDecelMs2, maxAccelMs2);

            _rb.AddForce(fwd * accel, ForceMode.Acceleration);

            // 7) Yaw control: phi[-45..45] -> yawRate[-max..max]
            float maxYawRateRad = maxYawRateDeg * Mathf.Deg2Rad;
            float desiredYawRate = Mathf.Clamp(outCmd.Phi / 45f, -1f, 1f) * maxYawRateRad;

            float currentYawRate = _rb.angularVelocity.y;
            float yawErr = desiredYawRate - currentYawRate;

            float yawAccel = yawErr / Mathf.Max(0.001f, dt);
            yawAccel = Mathf.Clamp(yawAccel, -maxYawAccelRad, maxYawAccelRad);

            _rb.AddTorque(Vector3.up * yawAccel, ForceMode.Acceleration);
        }
    }
}
