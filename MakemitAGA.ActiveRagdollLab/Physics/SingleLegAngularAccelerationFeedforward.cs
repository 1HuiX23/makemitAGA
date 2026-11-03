using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.8 validated core — v0.2.37 real-inertia A/B re-confirmed healthy-normal A-FF as production-required.
    ///
    /// This is deliberately different from v0.2.7:
    /// - v0.2.7 used target A only to predict a D-term reference W.
    /// - This owner does not alter pose/W. It gates and bounds the upstream-selected
    ///   acceleration derivative before the existing PD/FF allocator and actuator.
    ///   Legacy inputs are held source A; coherent-reference inputs are same-step planned A.
    ///
    /// Safety / isolation:
    /// - this module never changes actuator authority; Hip phase-aware 40/80 authority (v0.2.29) and Knee limits are selected elsewhere; v0.2.30 may additionally suppress FF while the virtual Hip parent frame is catching up.
    /// - Raw A remains suppressed during source/torso recovery. The V041180 planned policy may consume only the caller-validated,
    ///   same-Fixed bounded Knee reference A in those two phases. Hip, physical recovery, frame and age gates remain.
    /// - very stale Render acceleration samples are suppressed instead of being trusted indefinitely.
    /// </summary>
    internal sealed class SingleLegAngularAccelerationFeedforward
    {
        public const string ModeName = "TARGET_RELATIVE_ANGULAR_ACCELERATION_ACTUATION_FEEDFORWARD";
        public const string GateModeName = "V041180_PER_LANE_RAW_SOURCE_RECOVERY_OFF_PLANNED_KNEE_AB_PHYSICAL_FRAME_GOAL_AGE_RETAINED";

        private bool _enabled;
        private bool _activeThisStep;
        private string _suppressionReason = "DISABLED";
        private bool _kneeActiveThisStep;
        private string _kneeSuppressionReason = "DISABLED";

        private Vector3 _hipRawWorld;
        private Vector3 _hipAppliedWorld;
        private float _kneeRawSigned;
        private float _kneeAppliedSigned;
        private bool _hipClamped;
        private bool _kneeClamped;

        private int _activeFixedSteps;
        private int _kneeActiveFixedSteps, _kneeGoalEligibleSteps, _kneeGoalNonzeroSelectedSteps;
        private int _hipClampCount;
        private int _kneeClampCount;
        private float _hipAppliedSum;
        private float _hipAppliedMax;
        private float _kneeAppliedAbsSum;
        private float _kneeAppliedAbsMax;
        private bool _kneePlannedSourceRecoveryActive, _kneeAllocationRecorded;
        private int _kneeSourceRecoveryRequestedSteps, _kneeSourceRecoveryEligibleSteps;
        private int _kneeSourceRecoveryNonzeroSelectedSteps, _kneeSourceRecoveryNonzeroSubmittedDeltaSteps;
        private int _kneeSourceRecoveryPhysicalBlockedSteps, _kneeSourceRecoveryFrameBlockedSteps;
        private int _kneeSourceRecoveryGoalBlockedSteps, _kneeSourceRecoveryStaleBlockedSteps;

        public bool Enabled => _enabled;
        public bool ActiveThisStep => _activeThisStep || _kneeActiveThisStep;
        public bool HipActiveThisStep => _activeThisStep;
        public bool KneeActiveThisStep => _kneeActiveThisStep;
        // Compatibility reason is the Hip first-match reason; Knee has its own reason below.
        public string SuppressionReason => _suppressionReason;
        public string KneeSuppressionReason => _kneeSuppressionReason;
        public Vector3 HipAppliedWorld => _hipAppliedWorld;
        public float KneeAppliedSigned => _kneeAppliedSigned;
        public bool HipClamped => _hipClamped;
        public bool KneeClamped => _kneeClamped;
        public int ActiveFixedSteps => _activeFixedSteps;
        public int KneeActiveFixedSteps => _kneeActiveFixedSteps;
        public int KneeGoalEligibleSteps => _kneeGoalEligibleSteps;
        public int KneeGoalNonzeroSelectedSteps => _kneeGoalNonzeroSelectedSteps;
        public int HipClampCount => _hipClampCount;
        public int KneeClampCount => _kneeClampCount;
        public float HipAppliedMean => _activeFixedSteps > 0 ? _hipAppliedSum / _activeFixedSteps : 0f;
        public float HipAppliedMax => _hipAppliedMax;
        public float KneeAppliedAbsMean => _kneeActiveFixedSteps > 0 ? _kneeAppliedAbsSum / _kneeActiveFixedSteps : 0f;
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
            _kneeActiveFixedSteps = _kneeGoalEligibleSteps = _kneeGoalNonzeroSelectedSteps = 0;
            _hipClampCount = 0;
            _kneeClampCount = 0;
            _hipAppliedSum = 0f;
            _hipAppliedMax = 0f;
            _kneeAppliedAbsSum = 0f;
            _kneeAppliedAbsMax = 0f;
            _kneeSourceRecoveryRequestedSteps = _kneeSourceRecoveryEligibleSteps = 0;
            _kneeSourceRecoveryNonzeroSelectedSteps = _kneeSourceRecoveryNonzeroSubmittedDeltaSteps = 0;
            _kneeSourceRecoveryPhysicalBlockedSteps = _kneeSourceRecoveryFrameBlockedSteps = 0;
            _kneeSourceRecoveryGoalBlockedSteps = _kneeSourceRecoveryStaleBlockedSteps = 0;
        }

        public bool Step(
            Vector3 hipTargetRelativeAngularAccelerationWorld,
            float kneeTargetAxisAngularAccelerationSigned,
            TorsoControlPhase? torso,
            bool controlRecoveryActive,
            float sourceSampleAgeSeconds,
            float gain,
            float maxFeedforwardAcceleration,
            float maxSourceSampleAgeSeconds,
            bool physicalHipRecoveryPending,
            bool physicalKneeRecoveryPending,
            bool suppressForHipReferenceBridge,
            bool suppressForHipReferenceGoal = false,
            bool hipGoalGateOnlyHip = false,
            bool allowPlannedKneeSourceRecovery = false)
        {
            ResetStepState(_enabled ? "WAITING_FOR_NORMAL_WINDOW" : "DISABLED");
            if (!IsFinite(hipTargetRelativeAngularAccelerationWorld) ||
                !IsFinite(kneeTargetAxisAngularAccelerationSigned) ||
                !IsFinite(sourceSampleAgeSeconds) ||
                !IsFinite(gain) ||
                !IsFinite(maxFeedforwardAcceleration) ||
                !IsFinite(maxSourceSampleAgeSeconds))
            {
                ResetStepState("INVALID_INPUT");
                return false;
            }

            if (!_enabled)
                return true;

            if (torso == null)
            {
                _suppressionReason = _kneeSuppressionReason = "NO_TORSO_REPORT";
                return true;
            }

            float maxAge = Mathf.Max(0f, maxSourceSampleAgeSeconds);
            bool stale = sourceSampleAgeSeconds > maxAge + 0.0001f;
            bool sourceRecovery = torso.Value.SourceTurnEventActive || controlRecoveryActive;
            // Resolve both complete gate chains. A source/torso exemption must never
            // hide a concurrent physical recovery, virtual frame, goal or stale input.
            _suppressionReason = ResolveGate(torso.Value.SourceTurnEventActive, controlRecoveryActive,
                physicalHipRecoveryPending, physicalKneeRecoveryPending, suppressForHipReferenceBridge,
                suppressForHipReferenceGoal, stale, false);
            _kneeSuppressionReason = ResolveGate(torso.Value.SourceTurnEventActive, controlRecoveryActive,
                physicalHipRecoveryPending, physicalKneeRecoveryPending, suppressForHipReferenceBridge,
                suppressForHipReferenceGoal && !hipGoalGateOnlyHip, stale, allowPlannedKneeSourceRecovery);

            _activeThisStep = _suppressionReason == null;
            _kneeActiveThisStep = _kneeSuppressionReason == null;
            _kneePlannedSourceRecoveryActive = allowPlannedKneeSourceRecovery && sourceRecovery && _kneeActiveThisStep;
            if (allowPlannedKneeSourceRecovery && sourceRecovery)
            {
                _kneeSourceRecoveryRequestedSteps++;
                if (_kneeActiveThisStep) _kneeSourceRecoveryEligibleSteps++;
                else if (physicalHipRecoveryPending || physicalKneeRecoveryPending) _kneeSourceRecoveryPhysicalBlockedSteps++;
                else if (suppressForHipReferenceBridge) _kneeSourceRecoveryFrameBlockedSteps++;
                else if (suppressForHipReferenceGoal && !hipGoalGateOnlyHip) _kneeSourceRecoveryGoalBlockedSteps++;
                else if (stale) _kneeSourceRecoveryStaleBlockedSteps++;
            }

            float safeGain = Mathf.Max(0f, gain);
            float maxFf = Mathf.Max(0f, maxFeedforwardAcceleration);
            if (_activeThisStep) _hipRawWorld = hipTargetRelativeAngularAccelerationWorld * safeGain;
            if (_kneeActiveThisStep) _kneeRawSigned = kneeTargetAxisAngularAccelerationSigned * safeGain;

            _hipAppliedWorld = ClampMagnitude(_hipRawWorld, maxFf);
            _kneeAppliedSigned = Mathf.Clamp(_kneeRawSigned, -maxFf, maxFf);
            _hipClamped = _hipRawWorld.magnitude > maxFf + 0.0001f;
            _kneeClamped = Mathf.Abs(_kneeRawSigned) > maxFf + 0.0001f;
            if (_hipClamped) _hipClampCount++;
            if (_kneeClamped) _kneeClampCount++;

            if (_activeThisStep) { _suppressionReason = "ACTIVE_NORMAL_WINDOW"; _activeFixedSteps++; }
            if (_kneeActiveThisStep)
            {
                _kneeSuppressionReason = _kneePlannedSourceRecoveryActive ? "ACTIVE_PLANNED_SOURCE_RECOVERY" : "ACTIVE_NORMAL_WINDOW";
                _kneeActiveFixedSteps++;
            }
            if (_kneePlannedSourceRecoveryActive && Mathf.Abs(_kneeAppliedSigned) > 0.0001f)
                _kneeSourceRecoveryNonzeroSelectedSteps++;
            if (_kneeActiveThisStep && suppressForHipReferenceGoal && !sourceRecovery)
            {
                _kneeGoalEligibleSteps++;
                if (Mathf.Abs(_kneeAppliedSigned) > 0.0001f) _kneeGoalNonzeroSelectedSteps++;
            }
            float hipMag = _hipAppliedWorld.magnitude;
            float kneeAbs = Mathf.Abs(_kneeAppliedSigned);
            _hipAppliedSum += hipMag;
            _hipAppliedMax = Mathf.Max(_hipAppliedMax, hipMag);
            _kneeAppliedAbsSum += kneeAbs;
            _kneeAppliedAbsMax = Mathf.Max(_kneeAppliedAbsMax, kneeAbs);

            return IsFinite(_hipAppliedWorld) && IsFinite(_kneeAppliedSigned);
        }

        private static string ResolveGate(bool sourceTurn, bool torsoRecovery, bool physicalHip, bool physicalKnee,
            bool frame, bool goal, bool stale, bool plannedSourceRecovery)
        {
            if (!plannedSourceRecovery && sourceTurn) return "SOURCE_TURN";
            if (!plannedSourceRecovery && torsoRecovery) return "RECOVERY";
            if (physicalHip || physicalKnee)
                return physicalHip && physicalKnee ? "PHYSICAL_HIP_AND_KNEE_RECOVERY" :
                    (physicalHip ? "PHYSICAL_HIP_RECOVERY" : "PHYSICAL_KNEE_RECOVERY");
            if (frame) return "HIP_CONTROL_FRAME_RECOVERY";
            if (goal) return "HIP_REFERENCE_GOAL_UNSTABLE";
            if (stale) return "STALE_SOURCE_SAMPLE";
            return null;
        }

        // Called after the existing Knee AddTorque succeeds. Count a change in the
        // submitted flexion command versus same-state capped PD, not native response.
        // Both opposing-FF suppression and final saturation can remove the change.
        internal void RecordKneeSubmittedCommand(float feedback, float finalFlexion, float maxAcceleration)
        {
            if (!_kneePlannedSourceRecoveryActive || _kneeAllocationRecorded) return;
            _kneeAllocationRecorded = true;
            if (IsFinite(feedback) && IsFinite(finalFlexion) && IsFinite(maxAcceleration) && maxAcceleration > 0f &&
                Mathf.Abs(finalFlexion - Mathf.Clamp(feedback, -maxAcceleration, maxAcceleration)) > 0.0001f)
                _kneeSourceRecoveryNonzeroSubmittedDeltaSteps++;
        }

        internal string GetKneeSourceRecoverySummary() =>
            "KNEE_PLANNED_FF_V041180/requested-eligible-selectedNonzero-submittedDeltaNonzero=" +
            _kneeSourceRecoveryRequestedSteps + "|" + _kneeSourceRecoveryEligibleSteps + "|" +
            _kneeSourceRecoveryNonzeroSelectedSteps + "|" + _kneeSourceRecoveryNonzeroSubmittedDeltaSteps +
            "/blockedFirst(physical-frame-goal-stale)=" + _kneeSourceRecoveryPhysicalBlockedSteps + "|" +
            _kneeSourceRecoveryFrameBlockedSteps + "|" + _kneeSourceRecoveryGoalBlockedSteps + "|" +
            _kneeSourceRecoveryStaleBlockedSteps + "/delta=SAME_STATE_FINAL_MINUS_CAPPED_PD_AFTER_SUBMIT_NOT_RESPONSE";

        private void ResetStepState(string reason)
        {
            _activeThisStep = false;
            _kneeActiveThisStep = false;
            _suppressionReason = reason;
            _kneeSuppressionReason = reason;
            _hipRawWorld = Vector3.zero;
            _hipAppliedWorld = Vector3.zero;
            _kneeRawSigned = 0f;
            _kneeAppliedSigned = 0f;
            _hipClamped = false;
            _kneeClamped = false;
            _kneePlannedSourceRecoveryActive = _kneeAllocationRecorded = false;
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
