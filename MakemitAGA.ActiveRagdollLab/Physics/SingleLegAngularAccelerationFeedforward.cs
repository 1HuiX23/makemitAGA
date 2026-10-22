using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.8 validated core — v0.2.37 real-inertia A/B re-confirmed healthy-normal A-FF as production-required.
    ///
    /// This is deliberately different from v0.2.7:
    /// - v0.2.7 used target A only to predict a D-term reference W.
    /// - The validated core keeps raw relative pose and raw sample-held target W unchanged, then adds a bounded
    ///   target-angular-acceleration contribution directly to the existing acceleration demand.
    ///
    /// Safety / isolation:
    /// - this module never changes actuator authority; Hip phase-aware 40/80 authority (v0.2.29) and Knee limits are selected elsewhere; v0.2.30 may additionally suppress FF while the virtual Hip parent frame is catching up.
    /// - Source Turn, torso recovery, physical Hip/Knee recovery, and active Hip control-frame inertialization are suppressed
    ///   so MiSide single-sample turn discontinuities and unresolved body-recovery transients are never injected as feedforward acceleration.
    /// - very stale Render acceleration samples are suppressed instead of being trusted indefinitely.
    /// </summary>
    internal sealed class SingleLegAngularAccelerationFeedforward
    {
        public const string ModeName = "TARGET_RELATIVE_ANGULAR_ACCELERATION_ACTUATION_FEEDFORWARD";
        public const string GateModeName = "PRODUCTION_NORMAL_ON_PLUS_SUPPRESS_SOURCE_TURN_TORSO_OR_PHYSICAL_LEG_RECOVERY_CONTROL_FRAME_GOAL_BRIDGE_STALE_SAMPLE";

        private bool _enabled;
        private bool _activeThisStep;
        private string _suppressionReason = "DISABLED";

        private Vector3 _hipRawWorld;
        private Vector3 _hipAppliedWorld;
        private float _kneeRawSigned;
        private float _kneeAppliedSigned;
        private bool _hipClamped;
        private bool _kneeClamped;

        private int _activeFixedSteps;
        private int _suppressedSourceTurnFixedSteps;
        private int _suppressedRecoveryFixedSteps;
        private int _suppressedStaleSampleFixedSteps;
        private int _hipClampCount;
        private int _kneeClampCount;
        private float _hipAppliedSum;
        private float _hipAppliedMax;
        private float _kneeAppliedAbsSum;
        private float _kneeAppliedAbsMax;

        public bool Enabled => _enabled;
        public bool ActiveThisStep => _activeThisStep;
        public string SuppressionReason => _suppressionReason;
        public Vector3 HipRawWorld => _hipRawWorld;
        public Vector3 HipAppliedWorld => _hipAppliedWorld;
        public float KneeRawSigned => _kneeRawSigned;
        public float KneeAppliedSigned => _kneeAppliedSigned;
        public bool HipClamped => _hipClamped;
        public bool KneeClamped => _kneeClamped;
        public int ActiveFixedSteps => _activeFixedSteps;
        public int SuppressedSourceTurnFixedSteps => _suppressedSourceTurnFixedSteps;
        public int SuppressedRecoveryFixedSteps => _suppressedRecoveryFixedSteps;
        public int SuppressedStaleSampleFixedSteps => _suppressedStaleSampleFixedSteps;
        public int HipClampCount => _hipClampCount;
        public int KneeClampCount => _kneeClampCount;
        public float HipAppliedMean => _activeFixedSteps > 0 ? _hipAppliedSum / _activeFixedSteps : 0f;
        public float HipAppliedMax => _hipAppliedMax;
        public float KneeAppliedAbsMean => _activeFixedSteps > 0 ? _kneeAppliedAbsSum / _activeFixedSteps : 0f;
        public float KneeAppliedAbsMax => _kneeAppliedAbsMax;

        public void SetEnabled(bool enabled)
        {
            _enabled = enabled;
            ResetStepState(enabled ? "WAITING_FOR_NORMAL_WINDOW" : "DISABLED");
        }

        public void Reset()
        {
            ResetDiagnostics();
            ResetStepState(_enabled ? "WAITING_FOR_NORMAL_WINDOW" : "DISABLED");
        }

        public void ResetDiagnostics()
        {
            _activeFixedSteps = 0;
            _suppressedSourceTurnFixedSteps = 0;
            _suppressedRecoveryFixedSteps = 0;
            _suppressedStaleSampleFixedSteps = 0;
            _hipClampCount = 0;
            _kneeClampCount = 0;
            _hipAppliedSum = 0f;
            _hipAppliedMax = 0f;
            _kneeAppliedAbsSum = 0f;
            _kneeAppliedAbsMax = 0f;
        }

        public bool Step(
            Vector3 hipTargetRelativeAngularAccelerationWorld,
            float kneeTargetAxisAngularAccelerationSigned,
            TwoBodyProxyReport torso,
            float sourceSampleAgeSeconds,
            float gain,
            float maxFeedforwardAcceleration,
            float maxSourceSampleAgeSeconds,
            bool physicalHipRecoveryPending,
            bool physicalKneeRecoveryPending,
            bool suppressForHipReferenceBridge,
            bool suppressForHipReferenceGoal = false)
        {
            if (!IsFinite(hipTargetRelativeAngularAccelerationWorld) ||
                !IsFinite(kneeTargetAxisAngularAccelerationSigned) ||
                !IsFinite(sourceSampleAgeSeconds) ||
                !IsFinite(gain) ||
                !IsFinite(maxFeedforwardAcceleration) ||
                !IsFinite(maxSourceSampleAgeSeconds))
                return false;

            ResetStepState(_enabled ? "WAITING_FOR_NORMAL_WINDOW" : "DISABLED");
            if (!_enabled)
                return true;

            if (torso == null)
            {
                _suppressionReason = "NO_TORSO_REPORT";
                return true;
            }

            if (torso.SourceTurnEventActive)
            {
                _suppressionReason = "SOURCE_TURN";
                _suppressedSourceTurnFixedSteps++;
                return true;
            }

            if (torso.ControlRecoveryActive)
            {
                _suppressionReason = "RECOVERY";
                _suppressedRecoveryFixedSteps++;
                return true;
            }

            if (physicalHipRecoveryPending || physicalKneeRecoveryPending)
            {
                _suppressionReason = physicalHipRecoveryPending && physicalKneeRecoveryPending
                    ? "PHYSICAL_HIP_AND_KNEE_RECOVERY"
                    : (physicalHipRecoveryPending ? "PHYSICAL_HIP_RECOVERY" : "PHYSICAL_KNEE_RECOVERY");
                _suppressedRecoveryFixedSteps++;
                return true;
            }

            if (suppressForHipReferenceBridge)
            {
                _suppressionReason = "HIP_CONTROL_FRAME_RECOVERY";
                _suppressedRecoveryFixedSteps++;
                return true;
            }

            // Scoped B policy: relative Render A is not the derivative of an absorbed/governed world goal.
            if (suppressForHipReferenceGoal)
            {
                _suppressionReason = "HIP_REFERENCE_GOAL_UNSTABLE";
                _suppressedRecoveryFixedSteps++;
                return true;
            }

            float maxAge = Mathf.Max(0f, maxSourceSampleAgeSeconds);
            if (sourceSampleAgeSeconds > maxAge + 0.0001f)
            {
                _suppressionReason = "STALE_SOURCE_SAMPLE";
                _suppressedStaleSampleFixedSteps++;
                return true;
            }

            float safeGain = Mathf.Max(0f, gain);
            float maxFf = Mathf.Max(0f, maxFeedforwardAcceleration);
            _hipRawWorld = hipTargetRelativeAngularAccelerationWorld * safeGain;
            _kneeRawSigned = kneeTargetAxisAngularAccelerationSigned * safeGain;

            _hipAppliedWorld = ClampMagnitude(_hipRawWorld, maxFf);
            _kneeAppliedSigned = Mathf.Clamp(_kneeRawSigned, -maxFf, maxFf);
            _hipClamped = _hipRawWorld.magnitude > maxFf + 0.0001f;
            _kneeClamped = Mathf.Abs(_kneeRawSigned) > maxFf + 0.0001f;
            if (_hipClamped) _hipClampCount++;
            if (_kneeClamped) _kneeClampCount++;

            _activeThisStep = true;
            _suppressionReason = "ACTIVE_NORMAL_WINDOW";
            _activeFixedSteps++;
            float hipMag = _hipAppliedWorld.magnitude;
            float kneeAbs = Mathf.Abs(_kneeAppliedSigned);
            _hipAppliedSum += hipMag;
            _hipAppliedMax = Mathf.Max(_hipAppliedMax, hipMag);
            _kneeAppliedAbsSum += kneeAbs;
            _kneeAppliedAbsMax = Mathf.Max(_kneeAppliedAbsMax, kneeAbs);

            return IsFinite(_hipAppliedWorld) && IsFinite(_kneeAppliedSigned);
        }

        private void ResetStepState(string reason)
        {
            _activeThisStep = false;
            _suppressionReason = reason;
            _hipRawWorld = Vector3.zero;
            _hipAppliedWorld = Vector3.zero;
            _kneeRawSigned = 0f;
            _kneeAppliedSigned = 0f;
            _hipClamped = false;
            _kneeClamped = false;
        }

        private static Vector3 ClampMagnitude(Vector3 value, float maxMagnitude)
        {
            float max = Mathf.Max(0f, maxMagnitude);
            float sq = value.sqrMagnitude;
            if (sq <= max * max || sq <= 1e-12f)
                return value;
            return value.normalized * max;
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }
    }
}
