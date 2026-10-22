using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.64 narrow causal A/B for the v0.2.63 runaway-recovery result.
    ///
    /// A / production (default OFF): byte-for-byte v0.2.63 Hip control path.
    /// B / experimental (ON): only while authoritative PhysicalRecoveryPending is true, Hip error is meaningful,
    /// and the physical body is still moving AWAY from the raw Hip target, add one extra copy of the existing
    /// axial Hip D-damper as braking effort. The supplemental term is bounded to the existing recovery authority
    /// headroom (recoveryMaxA - baselineMaxA), and the final Hip command remains clamped by the already-existing
    /// recovery ceiling (default 80 rad/s^2). No new profile knob, no governor-lead change, no P-gain change.
    ///
    /// The latch enters only after a meaningful away rate and releases as soon as the axial relative motion reaches
    /// neutral/toward-target, recovery ends, or Hip error returns to the normal recovery-exit band. This prevents a
    /// permanent high-damping mode and directly tests the causal question raised by v0.2.63:
    /// can extra braking of already-existing away momentum prevent runaway without reproducing the rejected fixed
    /// 30-degree governor-lead behavior from v0.2.52?
    /// </summary>
    internal sealed class HipMomentumAwareRecoveryBrakingIsolation
    {
        internal const string Mode = "HIP_MOMENTUM_AWARE_RECOVERY_BRAKING_CAUSAL_AB";
        internal const string ProductionArm = "A_PRODUCTION_V063_BRAKING_OFF";
        internal const string ExperimentalArm = "B_MOMENTUM_AWARE_BRAKING_ON";

        // Diagnostic/causal gating constants only; intentionally not profile knobs.
        private const float EnterHipErrorDegrees = 20f;
        private const float EnterAwaySpeedDegPerSec = 20f;
        private const float Epsilon = 0.000001f;

        private bool _enabled;
        private bool _active;
        private int _steps;
        private int _recoveryEligibleSteps;
        private int _activeSteps;
        private int _entries;
        private int _exitsToward;
        private int _exitsRecoveryEnded;
        private int _exitsHipRecovered;
        private int _supplementalClampSteps;
        private float _sumSupplementalA;
        private float _peakSupplementalA;
        private float _peakAwaySpeedDegPerSec;
        private float _peakEffectiveMaxA;
        private float _currentHipDegrees;
        private float _currentAwaySpeedDegPerSec;
        private float _currentSupplementalA;
        private float _currentProductionMaxA;
        private float _currentEffectiveMaxA;
        private string _currentOrigin = "NONE";
        private string _lastExit = "<none>";
        private string _abortEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal bool Active => _active;
        internal string Arm => _enabled ? ExperimentalArm : ProductionArm;

        internal void SetEnabled(bool enabled, SingleLegChainReport report)
        {
            _enabled = enabled;
            ResetDiagnostics(report);
        }

        internal void ResetDiagnostics(SingleLegChainReport report)
        {
            _active = false;
            _steps = 0;
            _recoveryEligibleSteps = 0;
            _activeSteps = 0;
            _entries = 0;
            _exitsToward = 0;
            _exitsRecoveryEnded = 0;
            _exitsHipRecovered = 0;
            _supplementalClampSteps = 0;
            _sumSupplementalA = 0f;
            _peakSupplementalA = 0f;
            _peakAwaySpeedDegPerSec = 0f;
            _peakEffectiveMaxA = 0f;
            _currentHipDegrees = 0f;
            _currentAwaySpeedDegPerSec = 0f;
            _currentSupplementalA = 0f;
            _currentProductionMaxA = 0f;
            _currentEffectiveMaxA = 0f;
            _currentOrigin = "NONE";
            _lastExit = "<none>";
            _abortEvidence = "<none>";
            CopyToReport(report);
        }

        /// <summary>
        /// Returns the supplemental relative-Hip acceleration to add BEFORE the existing parent-transport compose/clamp.
        /// No Rigidbody/Joint mutation occurs here.
        /// </summary>
        internal Vector3 ComposeSupplementalBraking(
            ActiveRagdollLabProfile profile,
            SingleLegChainReport report,
            Vector3 errorWorld,
            Vector3 velocityErrorWorld,
            float productionMaxA,
            out float effectiveMaxA)
        {
            _steps++;
            _currentProductionMaxA = Mathf.Max(0f, productionMaxA);
            _currentEffectiveMaxA = _currentProductionMaxA;
            _currentSupplementalA = 0f;
            _currentHipDegrees = Mathf.Max(0f, errorWorld.magnitude * Mathf.Rad2Deg);
            _currentOrigin = report != null ? (report.HipPhysicalRecoveryOrigin ?? "NONE") : "NONE";
            effectiveMaxA = _currentProductionMaxA;

            bool recoveryPending = report != null && report.HipPhysicalRecoveryPending;
            float errorMagnitude = errorWorld.magnitude;
            Vector3 axisWorld = errorMagnitude > Epsilon ? errorWorld / errorMagnitude : Vector3.zero;
            float awaySpeedRadPerSec = axisWorld.sqrMagnitude > Epsilon
                ? Vector3.Dot(velocityErrorWorld, axisWorld)
                : 0f;
            _currentAwaySpeedDegPerSec = awaySpeedRadPerSec * Mathf.Rad2Deg;
            _peakAwaySpeedDegPerSec = Mathf.Max(_peakAwaySpeedDegPerSec, Mathf.Max(0f, _currentAwaySpeedDegPerSec));

            if (!_enabled)
            {
                if (_active) Exit("ARM_DISABLED");
                CopyToReport(report);
                return Vector3.zero;
            }

            if (!recoveryPending)
            {
                if (_active)
                {
                    _exitsRecoveryEnded++;
                    Exit("RECOVERY_ENDED");
                }
                CopyToReport(report);
                return Vector3.zero;
            }

            _recoveryEligibleSteps++;

            float recoveryExit = profile != null ? profile.LegHipRecoveryExitErrorDegrees : 10f;
            if (_currentHipDegrees <= recoveryExit)
            {
                if (_active)
                {
                    _exitsHipRecovered++;
                    Exit("HIP_RECOVERED");
                }
                CopyToReport(report);
                return Vector3.zero;
            }

            // Once latched, stay active only while physical motion is still AWAY. Reaching neutral/toward is the causal exit.
            if (_active && awaySpeedRadPerSec <= 0f)
            {
                _exitsToward++;
                Exit("MOTION_REVERSED_TOWARD_TARGET");
            }

            if (!_active &&
                _currentHipDegrees >= EnterHipErrorDegrees &&
                _currentAwaySpeedDegPerSec >= EnterAwaySpeedDegPerSec)
            {
                _active = true;
                _entries++;
            }

            if (!_active || awaySpeedRadPerSec <= 0f || axisWorld.sqrMagnitude <= Epsilon)
            {
                CopyToReport(report);
                return Vector3.zero;
            }

            float baselineMaxA = profile != null ? Mathf.Max(0f, profile.LegHipMaxAngularAcceleration) : _currentProductionMaxA;
            float recoveryMaxA = profile != null ? Mathf.Max(baselineMaxA, profile.LegHipRecoveryMaxAngularAcceleration) : _currentProductionMaxA;
            float supplementalBound = Mathf.Max(0f, recoveryMaxA - baselineMaxA);
            float damper = profile != null ? Mathf.Max(0f, profile.LegHipAngularDamper) : 0f;
            float requestedSupplementalA = damper * awaySpeedRadPerSec;
            float supplementalA = Mathf.Min(requestedSupplementalA, supplementalBound);
            if (requestedSupplementalA > supplementalA + 0.0001f)
                _supplementalClampSteps++;

            _currentSupplementalA = supplementalA;
            _currentEffectiveMaxA = Mathf.Max(_currentProductionMaxA, recoveryMaxA);
            effectiveMaxA = _currentEffectiveMaxA;
            _activeSteps++;
            _sumSupplementalA += supplementalA;
            _peakSupplementalA = Mathf.Max(_peakSupplementalA, supplementalA);
            _peakEffectiveMaxA = Mathf.Max(_peakEffectiveMaxA, _currentEffectiveMaxA);
            CopyToReport(report);
            return axisWorld * supplementalA;
        }

        internal void MarkAbort(string reason, SingleLegChainReport report)
        {
            _abortEvidence =
                "reason=" + (reason ?? "<none>") +
                "/arm=" + Arm +
                "/active=" + _active +
                "/hip=" + F(_currentHipDegrees) +
                "/awayDegS=" + F(_currentAwaySpeedDegPerSec) +
                "/supplementalA=" + F(_currentSupplementalA) +
                "/maxA=" + F(_currentProductionMaxA) + "->" + F(_currentEffectiveMaxA) +
                "/origin=" + _currentOrigin;
            CopyToReport(report);
            Plugin.Logger?.LogWarning("[ARL][HIP_MOMENTUM_RECOVERY_BRAKING_ABORT_EVIDENCE] " + _abortEvidence);
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.HipMomentumRecoveryBrakingMode = Mode;
            report.HipMomentumRecoveryBrakingArm = Arm;
            report.HipMomentumRecoveryBrakingEnabled = _enabled;
            report.HipMomentumRecoveryBrakingActiveThisStep = _active;
            report.HipMomentumRecoveryBrakingSteps = _steps;
            report.HipMomentumRecoveryBrakingRecoveryEligibleSteps = _recoveryEligibleSteps;
            report.HipMomentumRecoveryBrakingActiveSteps = _activeSteps;
            report.HipMomentumRecoveryBrakingEntries = _entries;
            report.HipMomentumRecoveryBrakingExitTowardCount = _exitsToward;
            report.HipMomentumRecoveryBrakingExitRecoveryEndedCount = _exitsRecoveryEnded;
            report.HipMomentumRecoveryBrakingExitHipRecoveredCount = _exitsHipRecovered;
            report.HipMomentumRecoveryBrakingSupplementalClampSteps = _supplementalClampSteps;
            report.HipMomentumRecoveryBrakingCurrentHipDegrees = _currentHipDegrees;
            report.HipMomentumRecoveryBrakingCurrentAwaySpeedDegPerSec = _currentAwaySpeedDegPerSec;
            report.HipMomentumRecoveryBrakingCurrentSupplementalAcceleration = _currentSupplementalA;
            report.HipMomentumRecoveryBrakingCurrentProductionMaxA = _currentProductionMaxA;
            report.HipMomentumRecoveryBrakingEffectiveMaxA = _currentEffectiveMaxA;
            report.HipMomentumRecoveryBrakingMeanSupplementalAcceleration = Mean(_sumSupplementalA, _activeSteps);
            report.HipMomentumRecoveryBrakingPeakSupplementalAcceleration = _peakSupplementalA;
            report.HipMomentumRecoveryBrakingPeakAwaySpeedDegPerSec = _peakAwaySpeedDegPerSec;
            report.HipMomentumRecoveryBrakingPeakEffectiveMaxA = _peakEffectiveMaxA;
            report.HipMomentumRecoveryBrakingCurrentRecoveryOrigin = _currentOrigin;
            report.HipMomentumRecoveryBrakingLastExitReason = _lastExit;
            report.HipMomentumRecoveryBrakingAbortEvidence = _abortEvidence;
            report.HipMomentumRecoveryBrakingKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            return "ARL Hip momentum-aware recovery braking A/B | mode=" + Mode +
                   " | arm=" + Arm +
                   " | gate=PhysicalRecoveryPending + hip>=" + F(EnterHipErrorDegrees) + "deg + away>=" + F(EnterAwaySpeedDegPerSec) + "deg/s" +
                   " | extraD=existing Hip damper x axial-away-W" +
                   " | supplementalBound=recoveryMaxA-baselineMaxA" +
                   " | finalCeiling=existing recoveryMaxA" +
                   " | active=" + _active +
                   " | current(hip/awayDegS/extraA/maxA)=" + F(_currentHipDegrees) + "/" + F(_currentAwaySpeedDegPerSec) + "/" + F(_currentSupplementalA) + "/" + F(_currentProductionMaxA) + "->" + F(_currentEffectiveMaxA) +
                   " | productionGovernorLead=UNCHANGED_10DEG";
        }

        internal string GetKey(SingleLegChainReport report)
        {
            return "mode=" + Mode +
                   " | arm=" + Arm +
                   " | gate(hip/awayDegS)=" + F(EnterHipErrorDegrees) + "/" + F(EnterAwaySpeedDegPerSec) +
                   " | steps(total/eligible/active)=" + _steps + "/" + _recoveryEligibleSteps + "/" + _activeSteps +
                   " | latch(entries/exitToward/exitRecoveryEnd/exitHipRecovered)=" + _entries + "/" + _exitsToward + "/" + _exitsRecoveryEnded + "/" + _exitsHipRecovered +
                   " | supplemental(mean/peak/clampSteps)=" + F(Mean(_sumSupplementalA, _activeSteps)) + "/" + F(_peakSupplementalA) + "/" + _supplementalClampSteps +
                   " | peakAwayDegS=" + F(_peakAwaySpeedDegPerSec) +
                   " | peakEffectiveMaxA=" + F(_peakEffectiveMaxA) +
                   " | response(hipPeak,abort)=" +
                   F(report != null ? report.PeakHipRotationErrorDegrees : 0f) + "," +
                   (report != null && report.DriveAborted) + "/" + (report != null ? (report.DriveAbortReason ?? "<none>") : "<none>") +
                   " | abortEvidence=" + _abortEvidence;
        }

        private void Exit(string reason)
        {
            _active = false;
            _currentSupplementalA = 0f;
            _lastExit = reason ?? "<none>";
        }

        private static float Mean(float sum, int count) => count > 0 ? sum / count : 0f;
        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
