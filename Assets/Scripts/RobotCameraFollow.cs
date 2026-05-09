using UnityEngine;

namespace FuzzyRobot
{
    [DefaultExecutionOrder(10000)]
    public class RobotCameraFollow : MonoBehaviour
    {
        [Header("Refs")]
        [SerializeField] private Transform target;
        [SerializeField] private bool autoFindRobot = true;

        [Header("Follow")]
        [SerializeField] private Vector3 offset = new(0f, 7f, -9f);
        [SerializeField] private float lookAtHeight = 0.65f;
        [SerializeField] private float positionSmoothTime = 0.18f;
        [SerializeField] private float rotationSharpness = 10f;
        [SerializeField] private bool snapOnStart = true;

        private Vector3 _positionVelocity;
        private bool _snapped;

        private void LateUpdate()
        {
            if (target == null)
            {
                return;
            }

            Vector3 lookPoint = target.position + Vector3.up * lookAtHeight;
            Vector3 desiredPosition = target.position + offset;

            if (snapOnStart && 
                !_snapped)
            {
                transform.position = desiredPosition;
                _snapped = true;
            }
            else
            {
                transform.position = Vector3.SmoothDamp(
                    transform.position,
                    desiredPosition,
                    ref _positionVelocity,
                    Mathf.Max(0.01f, positionSmoothTime));
            }

            Vector3 lookDirection = lookPoint - transform.position;
            if (lookDirection.sqrMagnitude < 0.0001f)
            {
                return;
            }

            Quaternion desiredRotation = Quaternion.LookRotation(lookDirection.normalized, Vector3.up);
            transform.rotation = Quaternion.Slerp(
                transform.rotation,
                desiredRotation,
                1f - Mathf.Exp(-rotationSharpness * Time.deltaTime));
        }
    }
}
