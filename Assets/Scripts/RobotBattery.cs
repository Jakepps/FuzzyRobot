using UnityEngine;

namespace FuzzyRobot
{
    public class RobotBattery : MonoBehaviour
    {
        private const float Epsilon = 1e-5f;

        [Header("Battery")]
        [SerializeField, Range(0f, 1f)] private float currentCharge01 = 1f;
        [SerializeField] private float idleDrainPerSecond = 0.002f;
        [SerializeField] private float moveDrainPerMeter = 0.003f;
        [SerializeField] private float turnDrainPerDegree = 0.002f;
        [SerializeField] private float rechargePerSecond = 0.02f;

        [Header("Recharge behavior")]
        [Tooltip("До какого минимального заряда робот должен подзарядиться, прежде чем снова сможет ехать.")]
        [SerializeField, Range(0f, 1f)] private float resumeMoveCharge01 = 0.3f;

        [Header("Debug")]
        [SerializeField] private bool debug;

        private Vector3 _lastPosition;
        private bool _initialized;

        public float ChargePercent => currentCharge01 * 100f;
        public bool IsRecharging { get; private set; }

        public bool CanMove => !IsRecharging;

        private void Awake()
        {
            _lastPosition = transform.position;
            _initialized = true;

            if (currentCharge01 <= Epsilon)
            {
                currentCharge01 = 0f;
                IsRecharging = true;
            }
        }

        public void Tick(float dt, Vector3 currentPosition, float yawRateDegAbs)
        {
            if (!_initialized)
            {
                _lastPosition = currentPosition;
                _initialized = true;
            }

            if (IsRecharging)
            {
                currentCharge01 = Mathf.Clamp01(currentCharge01 + rechargePerSecond * dt);

                if (currentCharge01 >= resumeMoveCharge01)
                {
                    IsRecharging = false;
                }

                _lastPosition = currentPosition;

                if (debug)
                {
                    Debug.Log($"<color=yellow>[Battery] Recharging: {currentCharge01:F3} ({ChargePercent:F1}%)</color>");
                }

                return;
            }

            float movedDistance = Vector3.Distance(_lastPosition, currentPosition);

            float drain =
                idleDrainPerSecond * dt +
                moveDrainPerMeter * movedDistance +
                turnDrainPerDegree * yawRateDegAbs * dt;

            currentCharge01 = Mathf.Clamp01(currentCharge01 - drain);

            if (currentCharge01 <= Epsilon)
            {
                currentCharge01 = 0f;
                IsRecharging = true;
            }

            _lastPosition = currentPosition;

            if (debug)
            {
                Debug.Log($"<color=green>[Battery] Charge: {currentCharge01:F3} ({ChargePercent:F1}%)</color>");
            }
        }
    }
}
