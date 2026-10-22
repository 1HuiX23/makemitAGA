using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Frozen parent angular transport: strict frame and post-frame recovery share the
    // existing acceleration clamp. Healthy Normal is always OFF; the old v0.2.88 B
    // selector had no remaining caller. Historical report fields remain zero-compatible.
    internal sealed class HipMechanicalParentTransportFeedforward
    {
        public const string ModeName = "EXPLICIT_PHASE_OWNED_MECHANICAL_PARENT_ANGULAR_ACCELERATION_TRANSPORT_V088";

        // Historical v0.2.48 master strict/recovery transport baseline. Production stack keeps this ON.
        private bool _enabled = true;

        private bool _activeThisStep;
        private bool _recoveryActiveThisStep;
        private string _activeBranch = "NONE";
        private string _suppressionReason = "RESET";
        private Vector3 _transportWorld;
        private Vector3 _relativeCorrectionWorld;
        private Vector3 _combinedRawWorld;
        private Vector3 _combinedAppliedWorld;
        private float _maxA;
        private bool _combinedClamped;

        private int _steps;
        private int _activeSteps;
        private int _strictActiveSteps;
        private int _recoveryActiveSteps;
        private int _healthyNormalEligibleSteps;
        private int _normalBypassSteps;
        private int _normalScopeBlockedSteps;
        private int _normalScopeBlockedSourceSteps;
        private int _normalScopeBlockedRecoverySteps;
        private int _normalScopeBlockedUnhealthySteps;
        private int _disabledSteps;
        private int _realParentSteps;
        private int _inactiveFrameSteps;
        private int _invalidSteps;
        private int _clampSteps;

        private float _peakTransport;
        private float _peakRelativeCorrection;
        private float _peakCombinedRaw;
        private float _peakCombinedApplied;
        private float _peakTransportToMaxA;
        private float _peakCombinedRawToMaxA;
        private string _lastEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal bool NormalTransportEnabled => false;
        internal string NormalTransportArm => "A_V088_TRUE_HEALTHY_NORMAL_PARENT_TRANSPORT_OFF";

        // v0.2.86 scope switch is retired. Ownership is fixed explicitly by phase in v0.2.88.
        internal string NormalTransportScopeArm => "V088_FIXED_EXPLICIT_PHASE_OWNERSHIP";
        internal bool RecoveryTransportEnabled => _enabled;
        internal bool ActiveThisStep => _activeThisStep;
        internal Vector3 CombinedRawWorld => _combinedRawWorld;

        internal bool TrySetEnabled(bool enabled)
        {
            if (_enabled == enabled)
                return false;
            _enabled = enabled;
            return true;
        }

        internal void ResetDiagnostics(SingleLegChainReport report)
        {
            _activeThisStep = false;
            _recoveryActiveThisStep = false;
            _activeBranch = "NONE";
            _suppressionReason = "RESET";
            _transportWorld = Vector3.zero;
            _relativeCorrectionWorld = Vector3.zero;
            _combinedRawWorld = Vector3.zero;
            _combinedAppliedWorld = Vector3.zero;
            _maxA = 0f;
            _combinedClamped = false;

            _steps = _activeSteps = _strictActiveSteps = _recoveryActiveSteps = _healthyNormalEligibleSteps = 0;
            _normalBypassSteps = _normalScopeBlockedSteps = 0;
            _normalScopeBlockedSourceSteps = _normalScopeBlockedRecoverySteps = _normalScopeBlockedUnhealthySteps = 0;
            _disabledSteps = _realParentSteps = _inactiveFrameSteps = _invalidSteps = _clampSteps = 0;
            _peakTransport = _peakRelativeCorrection = _peakCombinedRaw = _peakCombinedApplied = 0f;
            _peakTransportToMaxA = _peakCombinedRawToMaxA = 0f;
            _lastEvidence = "<none>";
            CopyToReport(report);
        }

        internal Vector3 ComposeAndClamp(
            bool useInertializedMechanicalParent,
            bool strictControlFrameActive,
            bool sourceTurnActive,
            bool physicalRecoveryPending,
            bool trackingUnhealthy,
            Vector3 commandedMechanicalParentAccelerationWorld,
            Vector3 relativeCorrectionWorld,
            float maxAngularAcceleration,
            out bool clamped)
        {
            _steps++;
            _activeThisStep = false;
            _recoveryActiveThisStep = false;
            _transportWorld = Vector3.zero;
            _relativeCorrectionWorld = relativeCorrectionWorld;
            _combinedRawWorld = relativeCorrectionWorld;
            _combinedAppliedWorld = Vector3.zero;
            _maxA = Mathf.Max(0f, maxAngularAcceleration);
            _combinedClamped = false;
            _activeBranch = "NONE";

            if (!useInertializedMechanicalParent)
            {
                _realParentSteps++;
                _suppressionReason = "REAL_PELVIS_ARM_RAW_A_REJECTED";
            }
            else if (!Finite(commandedMechanicalParentAccelerationWorld) || !Finite(relativeCorrectionWorld) || !Finite(_maxA))
            {
                _invalidSteps++;
                _suppressionReason = "INVALID_SIGNAL";
            }
            else if (strictControlFrameActive)
            {
                // Frozen v0.2.48 strict Source/frame-recovery branch.
                if (!_enabled)
                {
                    _disabledSteps++;
                    _suppressionReason = "STRICT_TRANSPORT_DISABLED_MASTER";
                }
                else
                {
                    Activate("STRICT_SOURCE_FRAME_RECOVERY_V048", "ACTIVE_STRICT_INERTIALIZED_PARENT_TRANSPORT",
                        commandedMechanicalParentAccelerationWorld, recovery: false);
                    _strictActiveSteps++;
                }
            }
            else
            {
                _inactiveFrameSteps++;

                // A rare inactive-frame Source sample is NOT reclassified as post-frame recovery or Healthy Normal.
                if (sourceTurnActive)
                {
                    _normalScopeBlockedSteps++;
                    _normalScopeBlockedSourceSteps++;
                    _suppressionReason = "V088_INACTIVE_FRAME_SOURCE_BLOCKED";
                }
                else if (physicalRecoveryPending)
                {
                    // v0.2.87 STRONG PASS within POST_SOURCE + tracking-healthy exposure. v0.2.88 keeps the exact
                    // historical support vector/common clamp in BOTH A/B arms and gives it explicit recovery ownership.
                    if (!_enabled)
                    {
                        _disabledSteps++;
                        _suppressionReason = "POST_FRAME_RECOVERY_TRANSPORT_DISABLED_MASTER";
                    }
                    else
                    {
                        Activate("POST_FRAME_PHYSICAL_RECOVERY_V088", "ACTIVE_V088_DEDICATED_POST_FRAME_RECOVERY_PARENT_SUPPORT",
                            commandedMechanicalParentAccelerationWorld, recovery: true);
                        _recoveryActiveSteps++;
                    }
                }
                else if (trackingUnhealthy)
                {
                    // Not semantic Healthy Normal, and no authoritative physicalRecoveryPending ownership is present.
                    _normalScopeBlockedSteps++;
                    _normalScopeBlockedUnhealthySteps++;
                    _suppressionReason = "V088_TRACKING_UNHEALTHY_WITHOUT_RECOVERY_BLOCKED";
                }
                else
                {
                    _healthyNormalEligibleSteps++;
                    _normalBypassSteps++;
                    _suppressionReason = NormalTransportArm;
                }
            }

            _combinedAppliedWorld = ClampMagnitude(_combinedRawWorld, _maxA, out _combinedClamped);
            if (_combinedClamped)
                _clampSteps++;
            clamped = _combinedClamped;

            float transport = _transportWorld.magnitude;
            float correction = _relativeCorrectionWorld.magnitude;
            float raw = _combinedRawWorld.magnitude;
            float applied = _combinedAppliedWorld.magnitude;
            _peakTransport = Mathf.Max(_peakTransport, transport);
            _peakRelativeCorrection = Mathf.Max(_peakRelativeCorrection, correction);
            _peakCombinedRaw = Mathf.Max(_peakCombinedRaw, raw);
            _peakCombinedApplied = Mathf.Max(_peakCombinedApplied, applied);
            if (_maxA > 0.0001f)
            {
                _peakTransportToMaxA = Mathf.Max(_peakTransportToMaxA, transport / _maxA);
                _peakCombinedRawToMaxA = Mathf.Max(_peakCombinedRawToMaxA, raw / _maxA);
            }

            _lastEvidence =
                "active=" + _activeThisStep +
                "/branch=" + _activeBranch +
                "/reason=" + _suppressionReason +
                "/transport=" + F(transport) +
                "/relative=" + F(correction) +
                "/combinedRaw=" + F(raw) +
                "/applied=" + F(applied) +
                "/maxA=" + F(_maxA) +
                "/clamp=" + _combinedClamped;

            return _combinedAppliedWorld;
        }

        private void Activate(string branch, string reason, Vector3 transportWorld, bool recovery)
        {
            _activeThisStep = true;
            _activeSteps++;
            _activeBranch = branch;
            _suppressionReason = reason;
            _transportWorld = transportWorld;
            _combinedRawWorld = _relativeCorrectionWorld + _transportWorld;
            _recoveryActiveThisStep = recovery;
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null)
                return;
            report.HipMechanicalParentTransportMode = ModeName;
            report.HipMechanicalParentTransportEnabled = _enabled;
            report.HipMechanicalParentNormalTransportArm = NormalTransportArm;
            report.HipMechanicalParentNormalTransportScopeArm = NormalTransportScopeArm;
            report.HipMechanicalParentRecoveryTransportEnabled = RecoveryTransportEnabled;
            report.HipMechanicalParentRecoveryTransportActiveThisStep = _recoveryActiveThisStep;
            report.HipMechanicalParentHealthyNormalTransportActiveThisStep = false;
            report.HipMechanicalParentTransportActiveThisStep = _activeThisStep;
            report.HipMechanicalParentTransportActiveBranch = _activeBranch;
            report.HipMechanicalParentTransportSuppressionReason = _suppressionReason;
            report.HipMechanicalParentTransportCurrentMagnitude = _transportWorld.magnitude;
            report.HipMechanicalParentTransportCurrentRelativeCorrectionMagnitude = _relativeCorrectionWorld.magnitude;
            report.HipMechanicalParentTransportCurrentCombinedRawMagnitude = _combinedRawWorld.magnitude;
            report.HipMechanicalParentTransportCurrentCombinedAppliedMagnitude = _combinedAppliedWorld.magnitude;
            report.HipMechanicalParentTransportCurrentMaxAngularAcceleration = _maxA;
            report.HipMechanicalParentTransportCombinedClamped = _combinedClamped;
            report.HipMechanicalParentTransportSteps = _steps;
            report.HipMechanicalParentTransportActiveSteps = _activeSteps;
            report.HipMechanicalParentTransportStrictActiveSteps = _strictActiveSteps;
            report.HipMechanicalParentRecoveryTransportActiveSteps = _recoveryActiveSteps;
            report.HipMechanicalParentHealthyNormalEligibleSteps = _healthyNormalEligibleSteps;
            report.HipMechanicalParentTransportNormalActiveSteps = 0;
            report.HipMechanicalParentTransportNormalBypassSteps = _normalBypassSteps;
            report.HipMechanicalParentTransportScopeBlockedSteps = _normalScopeBlockedSteps;
            report.HipMechanicalParentTransportScopeBlockedSourceSteps = _normalScopeBlockedSourceSteps;
            report.HipMechanicalParentTransportScopeBlockedRecoverySteps = _normalScopeBlockedRecoverySteps;
            report.HipMechanicalParentTransportScopeBlockedUnhealthySteps = _normalScopeBlockedUnhealthySteps;
            report.HipMechanicalParentTransportDisabledSteps = _disabledSteps;
            report.HipMechanicalParentTransportRealParentSteps = _realParentSteps;
            report.HipMechanicalParentTransportInactiveFrameSteps = _inactiveFrameSteps;
            report.HipMechanicalParentTransportInvalidSteps = _invalidSteps;
            report.HipMechanicalParentTransportClampSteps = _clampSteps;
            report.HipMechanicalParentTransportPeakMagnitude = _peakTransport;
            report.HipMechanicalParentTransportPeakRelativeCorrectionMagnitude = _peakRelativeCorrection;
            report.HipMechanicalParentTransportPeakCombinedRawMagnitude = _peakCombinedRaw;
            report.HipMechanicalParentTransportPeakCombinedAppliedMagnitude = _peakCombinedApplied;
            report.HipMechanicalParentTransportPeakToMaxARatio = _peakTransportToMaxA;
            report.HipMechanicalParentTransportPeakCombinedRawToMaxARatio = _peakCombinedRawToMaxA;
            report.HipMechanicalParentTransportLastEvidence = _lastEvidence;
            report.HipMechanicalParentTransportKey = GetKey();
        }

        internal string GetKey()
        {
            return
                "mode=" + ModeName +
                " | strictEnabled=" + _enabled +
                " | recoverySupport=DEDICATED_V088_" + (_enabled ? "ON" : "OFF_MASTER") +
                " | healthyNormalArm=" + NormalTransportArm +
                " | phaseOwnership=" + NormalTransportScopeArm +
                " | steps(total/active/strict/recovery/healthyEligible/healthyActive/healthyBypass/nonHealthyBlocked/disabled/real/inactiveFrame/invalid/clamp)=" +
                _steps + "/" + _activeSteps + "/" + _strictActiveSteps + "/" + _recoveryActiveSteps + "/" + _healthyNormalEligibleSteps + "/" + 0 + "/" + _normalBypassSteps + "/" + _normalScopeBlockedSteps + "/" + _disabledSteps + "/" + _realParentSteps + "/" + _inactiveFrameSteps + "/" + _invalidSteps + "/" + _clampSteps +
                " | nonHealthyBlocked(source/recovery/unhealthyNoRecovery)=" + _normalScopeBlockedSourceSteps + "/" + _normalScopeBlockedRecoverySteps + "/" + _normalScopeBlockedUnhealthySteps +
                " | current(active/recovery/healthyNormal/branch/reason/transport/relative/raw/applied/maxA/clamp)=" +
                _activeThisStep + "/" + _recoveryActiveThisStep + "/" + false + "/" + _activeBranch + "/" + _suppressionReason + "/" + F(_transportWorld.magnitude) + "/" + F(_relativeCorrectionWorld.magnitude) + "/" + F(_combinedRawWorld.magnitude) + "/" + F(_combinedAppliedWorld.magnitude) + "/" + F(_maxA) + "/" + _combinedClamped +
                " | peak(transport/relative/raw/applied/transportToMaxA/rawToMaxA)=" +
                F(_peakTransport) + "/" + F(_peakRelativeCorrection) + "/" + F(_peakCombinedRaw) + "/" + F(_peakCombinedApplied) + "/" + F(_peakTransportToMaxA) + "/" + F(_peakCombinedRawToMaxA) +
                " | totalAuthority=UNCHANGED_HIP_MAXA_40_80_80" +
                " | recoveryVector=SAME_UPCOMING_PARENT_A_AS_V087_VALIDATED_BASELINE" +
                " | sourceA=STRICT_CONTROL_FRAME_BRANCH_ONLY" +
                " | guardPoseRebase=NEVER_USED_BY_TRANSPORT_FF" +
                " | rawPelvisA=NEVER_USED_BY_TRANSPORT_FF";
        }

        private static Vector3 ClampMagnitude(Vector3 value, float maxMagnitude, out bool clamped)
        {
            clamped = false;
            if (!Finite(value) || !Finite(maxMagnitude) || maxMagnitude <= 0f)
                return Vector3.zero;
            float magnitude = value.magnitude;
            if (magnitude <= maxMagnitude || magnitude <= 0.000001f)
                return value;
            clamped = true;
            return value * (maxMagnitude / magnitude);
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool Finite(Vector3 value) => Finite(value.x) && Finite(value.y) && Finite(value.z);
        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
