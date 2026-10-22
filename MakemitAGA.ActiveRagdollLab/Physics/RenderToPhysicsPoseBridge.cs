using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.1.10 Render -> Physics pose bridge for the kinematic Pelvis proxy.
    ///
    /// The source Animator updates on the render clock, while PhysX consumes commands on the fixed clock.
    /// v0.1.9 already proved that source VELOCITY must be sampled on the render clock and held across
    /// intervening FixedUpdates. This bridge applies the same principle to POSE without differentiating the
    /// render-updated Transform again inside FixedUpdate.
    ///
    /// Each physics step:
    /// 1) predict forward from the previous commanded pose using the held render-sampled velocity;
    /// 2) estimate the source pose "now" from latest render pose + bounded sample age extrapolation;
    /// 3) apply a bounded proportional correction toward that estimate;
    /// 4) emit one continuous MovePosition / MoveRotation command for this physics step.
    ///
    /// The correction is deliberately bounded so a late/new render frame cannot collapse several physics
    /// steps of displacement into one command. This class never applies forces and never touches source bones.
    /// </summary>
    internal sealed class RenderToPhysicsPoseBridge
    {
        public const string ModeName = "RENDER_POSE_VELOCITY_PREDICT_CORRECT";

        private bool _initialized;
        private Vector3 _commandPosition;
        private Quaternion _commandRotation = Quaternion.identity;
        private Vector3 _commandLinearVelocity;
        private Vector3 _commandAngularVelocity;
        private Vector3 _positionCorrectionVelocity;
        private Vector3 _rotationCorrectionAngularVelocity;
        private float _positionErrorMeters;
        private float _rotationErrorDegrees;
        private float _predictionAgeSeconds;
        private bool _positionCorrectionClamped;
        private bool _rotationCorrectionClamped;
        private int _positionCorrectionClampCount;
        private int _rotationCorrectionClampCount;

        public bool Initialized => _initialized;
        public Vector3 CommandPosition => _commandPosition;
        public Quaternion CommandRotation => _commandRotation;
        public Vector3 CommandLinearVelocity => _commandLinearVelocity;
        public Vector3 CommandAngularVelocity => _commandAngularVelocity;
        public Vector3 PositionCorrectionVelocity => _positionCorrectionVelocity;
        public Vector3 RotationCorrectionAngularVelocity => _rotationCorrectionAngularVelocity;
        public float PositionErrorMeters => _positionErrorMeters;
        public float RotationErrorDegrees => _rotationErrorDegrees;
        public float PredictionAgeSeconds => _predictionAgeSeconds;
        public bool PositionCorrectionClamped => _positionCorrectionClamped;
        public bool RotationCorrectionClamped => _rotationCorrectionClamped;
        public int PositionCorrectionClampCount => _positionCorrectionClampCount;
        public int RotationCorrectionClampCount => _rotationCorrectionClampCount;

        public void Reset()
        {
            _initialized = false;
            _commandPosition = Vector3.zero;
            _commandRotation = Quaternion.identity;
            _commandLinearVelocity = Vector3.zero;
            _commandAngularVelocity = Vector3.zero;
            _positionCorrectionVelocity = Vector3.zero;
            _rotationCorrectionAngularVelocity = Vector3.zero;
            _positionErrorMeters = 0f;
            _rotationErrorDegrees = 0f;
            _predictionAgeSeconds = 0f;
            _positionCorrectionClamped = false;
            _rotationCorrectionClamped = false;
            _positionCorrectionClampCount = 0;
            _rotationCorrectionClampCount = 0;
        }

        public void Initialize(Vector3 position, Quaternion rotation)
        {
            _initialized = true;
            _commandPosition = position;
            _commandRotation = NormalizeSafe(rotation);
            _commandLinearVelocity = Vector3.zero;
            _commandAngularVelocity = Vector3.zero;
            _positionCorrectionVelocity = Vector3.zero;
            _rotationCorrectionAngularVelocity = Vector3.zero;
            _positionErrorMeters = 0f;
            _rotationErrorDegrees = 0f;
            _predictionAgeSeconds = 0f;
            _positionCorrectionClamped = false;
            _rotationCorrectionClamped = false;
        }

        public bool Step(
            Vector3 latestSourcePosition,
            Quaternion latestSourceRotation,
            Vector3 heldSourceLinearVelocity,
            Vector3 heldSourceAngularVelocity,
            float sourceSampleAgeSeconds,
            float fixedDeltaTime,
            float maxExtrapolationSeconds,
            float positionCorrectionGain,
            float maxPositionCorrectionSpeed,
            float rotationCorrectionGain,
            float maxRotationCorrectionAngularSpeed,
            float maxLinearVelocity,
            float maxAngularVelocity)
        {
            if (!IsFinite(latestSourcePosition) || !IsFinite(latestSourceRotation) ||
                !IsFinite(heldSourceLinearVelocity) || !IsFinite(heldSourceAngularVelocity) ||
                !IsFinite(sourceSampleAgeSeconds) || !IsFinite(fixedDeltaTime) || fixedDeltaTime <= 0.0001f)
                return false;

            latestSourceRotation = NormalizeSafe(latestSourceRotation);
            if (!_initialized)
                Initialize(latestSourcePosition, latestSourceRotation);

            float age = Mathf.Clamp(sourceSampleAgeSeconds, 0f, Mathf.Max(0f, maxExtrapolationSeconds));
            _predictionAgeSeconds = age;

            Vector3 baseLinearVelocity = ClampMagnitude(heldSourceLinearVelocity, maxLinearVelocity);
            Vector3 baseAngularVelocity = ClampMagnitude(heldSourceAngularVelocity, maxAngularVelocity);

            // Estimate where the latest render pose should be at the current physics time.
            Vector3 predictedSourcePosition = latestSourcePosition + baseLinearVelocity * age;
            Quaternion predictedSourceRotation = IntegrateWorldAngularVelocity(latestSourceRotation, baseAngularVelocity, age);

            // First advance from the previous physics command using the held render-sampled velocity.
            Vector3 velocityPredictedPosition = _commandPosition + baseLinearVelocity * fixedDeltaTime;
            Quaternion velocityPredictedRotation = IntegrateWorldAngularVelocity(_commandRotation, baseAngularVelocity, fixedDeltaTime);

            // Bounded positional correction. The clamp prevents a newly arrived render frame from being
            // consumed as one large kinematic jump.
            Vector3 positionError = predictedSourcePosition - velocityPredictedPosition;
            Vector3 rawCorrectionVelocity = positionError * Mathf.Max(0f, positionCorrectionGain);
            _positionCorrectionClamped = rawCorrectionVelocity.magnitude > maxPositionCorrectionSpeed + 0.0001f;
            if (_positionCorrectionClamped)
                _positionCorrectionClampCount++;
            _positionCorrectionVelocity = ClampMagnitude(rawCorrectionVelocity, maxPositionCorrectionSpeed);

            Vector3 combinedLinearVelocity = ClampMagnitude(
                baseLinearVelocity + _positionCorrectionVelocity,
                maxLinearVelocity);
            Vector3 nextPosition = _commandPosition + combinedLinearVelocity * fixedDeltaTime;

            // Bounded rotational correction in world space.
            Vector3 rotationErrorVector = RotationErrorVectorWorld(velocityPredictedRotation, predictedSourceRotation);
            Vector3 rawRotationCorrection = rotationErrorVector * Mathf.Max(0f, rotationCorrectionGain);
            _rotationCorrectionClamped = rawRotationCorrection.magnitude > maxRotationCorrectionAngularSpeed + 0.0001f;
            if (_rotationCorrectionClamped)
                _rotationCorrectionClampCount++;
            _rotationCorrectionAngularVelocity = ClampMagnitude(rawRotationCorrection, maxRotationCorrectionAngularSpeed);

            Vector3 combinedAngularVelocity = ClampMagnitude(
                baseAngularVelocity + _rotationCorrectionAngularVelocity,
                maxAngularVelocity);
            Quaternion nextRotation = IntegrateWorldAngularVelocity(
                _commandRotation,
                combinedAngularVelocity,
                fixedDeltaTime);

            if (!IsFinite(nextPosition) || !IsFinite(nextRotation) ||
                !IsFinite(combinedLinearVelocity) || !IsFinite(combinedAngularVelocity))
                return false;

            _commandPosition = nextPosition;
            _commandRotation = NormalizeSafe(nextRotation);
            _commandLinearVelocity = combinedLinearVelocity;
            _commandAngularVelocity = combinedAngularVelocity;
            _positionErrorMeters = Vector3.Distance(_commandPosition, predictedSourcePosition);
            _rotationErrorDegrees = Quaternion.Angle(_commandRotation, predictedSourceRotation);
            return IsFinite(_positionErrorMeters) && IsFinite(_rotationErrorDegrees);
        }

        private static Quaternion IntegrateWorldAngularVelocity(Quaternion rotation, Vector3 angularVelocity, float dt)
        {
            float speed = angularVelocity.magnitude;
            if (!IsFinite(speed) || speed <= 0.000001f || dt <= 0f)
                return NormalizeSafe(rotation);

            float angleDegrees = speed * dt * Mathf.Rad2Deg;
            Quaternion delta = Quaternion.AngleAxis(angleDegrees, angularVelocity / speed);
            return NormalizeSafe(delta * rotation);
        }

        private static Vector3 RotationErrorVectorWorld(Quaternion current, Quaternion target)
        {
            Quaternion delta = target * Quaternion.Inverse(current);
            if (delta.w < 0f)
            {
                delta.x = -delta.x;
                delta.y = -delta.y;
                delta.z = -delta.z;
                delta.w = -delta.w;
            }

            float w = Mathf.Clamp(delta.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (sinHalf <= 0.0001f || angle <= 0.000001f)
                return Vector3.zero;

            Vector3 axis = new Vector3(delta.x / sinHalf, delta.y / sinHalf, delta.z / sinHalf);
            return axis * angle;
        }

        private static Quaternion NormalizeSafe(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!IsFinite(mag) || mag <= 0.000001f)
                return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        private static Vector3 ClampMagnitude(Vector3 value, float maxMagnitude)
        {
            if (!IsFinite(value))
                return Vector3.zero;
            float safeMax = Mathf.Max(0f, maxMagnitude);
            float sqr = value.sqrMagnitude;
            if (sqr <= safeMax * safeMax || sqr <= 0.0000001f)
                return value;
            return value.normalized * safeMax;
        }

        private static bool IsFinite(float value)
        {
            return !(float.IsNaN(value) || float.IsInfinity(value));
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        }
    }
}
