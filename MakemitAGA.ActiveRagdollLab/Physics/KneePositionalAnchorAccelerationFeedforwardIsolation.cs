using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.77 Knee positional-anchor acceleration feedforward causal A/B.
    ///
    /// v0.2.76 B proved that removing the remaining Knee CharacterJoint substantially improves normal Knee/Lower
    /// tracking, but its Lower-COM translation-only tether eventually crossed the frozen 3 cm causal-validity gate
    /// without ever reaching the existing 60 m/s^2 total tether ceiling. That pattern is consistent with a moving
    /// parent-anchor tracking gap: a pure position/velocity PD must first accumulate separation before it can react.
    ///
    /// A preserves the exact v0.2.76 tether input (zero acceleration feedforward). B estimates the ACTUAL Upper Knee
    /// anchor world acceleration from consecutive fixed-step point velocities and adds that vector before the same
    /// unchanged positional correction and unchanged final 60 m/s^2 total clamp. The first B step is warm-up only.
    /// No Render future pose, source target, new gain, direct force, torque, Move* call, or authority increase exists
    /// in this module; it computes one causal world-space feedforward vector only.
    /// </summary>
    internal sealed class KneePositionalAnchorAccelerationFeedforwardIsolation
    {
        internal const string Mode = "KNEE_POSITIONAL_ANCHOR_ACCELERATION_FEEDFORWARD_CAUSAL_AB";
        internal const string ArmA = "A_V076_PD_ONLY_KNEE_ANCHOR_TETHER";
        internal const string ArmB = "B_V077_MEASURED_PARENT_ANCHOR_ACCELERATION_FF";

        private bool _enabled;
        private bool _hasPreviousParentAnchorVelocity;
        private Vector3 _previousParentAnchorVelocity;
        private int _steps;
        private int _warmupSteps;
        private int _activeSteps;
        private int _invalidSteps;
        private float _currentParentAnchorSpeed;
        private float _currentFeedforwardAcceleration;
        private float _peakParentAnchorSpeed;
        private float _peakFeedforwardAcceleration;
        private Vector3 _currentFeedforwardWorld;
        private string _lastEvidence = "<none>";
        private string _abortEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal string Arm => _enabled ? ArmB : ArmA;
        internal bool CurrentValid { get; private set; } = true;
        internal Vector3 CurrentFeedforwardWorld => _currentFeedforwardWorld;

        internal void SetEnabled(bool enabled, SingleLegChainReport report)
        {
            _enabled = enabled;
            ResetDiagnostics(report);
        }

        internal void ResetDiagnostics(SingleLegChainReport report)
        {
            _hasPreviousParentAnchorVelocity = false;
            _previousParentAnchorVelocity = Vector3.zero;
            _steps = 0;
            _warmupSteps = 0;
            _activeSteps = 0;
            _invalidSteps = 0;
            _currentParentAnchorSpeed = 0f;
            _currentFeedforwardAcceleration = 0f;
            _peakParentAnchorSpeed = 0f;
            _peakFeedforwardAcceleration = 0f;
            _currentFeedforwardWorld = Vector3.zero;
            _lastEvidence = "<none>";
            _abortEvidence = "<none>";
            CurrentValid = true;
            CopyToReport(report);
        }

        internal Vector3 Step(
            Rigidbody upper,
            Vector3 parentAnchorUpperLocal,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            _currentFeedforwardWorld = Vector3.zero;
            _currentParentAnchorSpeed = 0f;
            _currentFeedforwardAcceleration = 0f;
            CurrentValid = true;

            if (!_enabled)
            {
                CopyToReport(report);
                return Vector3.zero;
            }

            _steps++;
            if (upper == null || fixedDeltaTime <= 0.0001f)
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=missingBodyOrDt";
                CopyToReport(report);
                return Vector3.zero;
            }

            Vector3 parentAnchorWorld = upper.transform.TransformPoint(parentAnchorUpperLocal);
            Vector3 parentLever = parentAnchorWorld - upper.worldCenterOfMass;
            Vector3 parentAnchorVelocity = upper.velocity + Vector3.Cross(upper.angularVelocity, parentLever);
            if (!IsFinite(parentAnchorVelocity))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteParentAnchorVelocity";
                CopyToReport(report);
                return Vector3.zero;
            }

            _currentParentAnchorSpeed = parentAnchorVelocity.magnitude;
            _peakParentAnchorSpeed = Mathf.Max(_peakParentAnchorSpeed, _currentParentAnchorSpeed);

            if (!_hasPreviousParentAnchorVelocity)
            {
                _hasPreviousParentAnchorVelocity = true;
                _previousParentAnchorVelocity = parentAnchorVelocity;
                _warmupSteps++;
                _lastEvidence = "step=" + _steps + "/warmup=True/parentV=" + F(_currentParentAnchorSpeed);
                CopyToReport(report);
                return Vector3.zero;
            }

            Vector3 measuredParentAnchorAcceleration =
                (parentAnchorVelocity - _previousParentAnchorVelocity) / fixedDeltaTime;
            _previousParentAnchorVelocity = parentAnchorVelocity;

            if (!IsFinite(measuredParentAnchorAcceleration))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteParentAnchorAcceleration";
                CopyToReport(report);
                return Vector3.zero;
            }

            _currentFeedforwardWorld = measuredParentAnchorAcceleration;
            _currentFeedforwardAcceleration = measuredParentAnchorAcceleration.magnitude;
            _peakFeedforwardAcceleration = Mathf.Max(_peakFeedforwardAcceleration, _currentFeedforwardAcceleration);
            if (_currentFeedforwardAcceleration > 0.0001f)
                _activeSteps++;

            _lastEvidence =
                "step=" + _steps +
                "/warmup=False" +
                "/parentV=" + F(_currentParentAnchorSpeed) +
                "/ffA=" + F(_currentFeedforwardAcceleration);
            CopyToReport(report);
            return _currentFeedforwardWorld;
        }

        internal void CaptureAbort(SingleLegChainReport report, string reason)
        {
            if (!_enabled) return;
            _abortEvidence =
                "reason=" + (reason ?? "<none>") +
                "/ffA=" + F(_currentFeedforwardAcceleration) +
                "/peakFfA=" + F(_peakFeedforwardAcceleration) +
                "/parentV=" + F(_currentParentAnchorSpeed);
            CopyToReport(report);
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.KneeAnchorAccelerationFeedforwardMode = Mode;
            report.KneeAnchorAccelerationFeedforwardArm = Arm;
            report.KneeAnchorAccelerationFeedforwardEnabled = _enabled;
            report.KneeAnchorAccelerationFeedforwardSteps = _steps;
            report.KneeAnchorAccelerationFeedforwardWarmupSteps = _warmupSteps;
            report.KneeAnchorAccelerationFeedforwardActiveSteps = _activeSteps;
            report.KneeAnchorAccelerationFeedforwardInvalidSteps = _invalidSteps;
            report.KneeAnchorAccelerationFeedforwardCurrentParentAnchorSpeed = _currentParentAnchorSpeed;
            report.KneeAnchorAccelerationFeedforwardCurrentAcceleration = _currentFeedforwardAcceleration;
            report.KneeAnchorAccelerationFeedforwardPeakParentAnchorSpeed = _peakParentAnchorSpeed;
            report.KneeAnchorAccelerationFeedforwardPeakAcceleration = _peakFeedforwardAcceleration;
            report.KneeAnchorAccelerationFeedforwardCurrentValid = CurrentValid;
            report.KneeAnchorAccelerationFeedforwardLastEvidence = _lastEvidence;
            report.KneeAnchorAccelerationFeedforwardAbortEvidence = _abortEvidence;
            report.KneeAnchorAccelerationFeedforwardKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return
                "ARL Knee anchor acceleration FF A/B | mode=" + Mode +
                " | arm=" + Arm +
                " | enabled=" + _enabled +
                " | steps(active/warmup/total/invalid)=" + _activeSteps + "/" + _warmupSteps + "/" + _steps + "/" + _invalidSteps +
                " | current(parentV/ffA)=" + F(_currentParentAnchorSpeed) + "/" + F(_currentFeedforwardAcceleration) +
                " | peak(parentV/ffA)=" + F(_peakParentAnchorSpeed) + "/" + F(_peakFeedforwardAcceleration) +
                " | key=" + GetKey(report);
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            if (!_enabled)
                primary = "A_V076_PD_ONLY_KNEE_ANCHOR_TETHER";
            else if (report != null && report.KneeCharacterJointPresent)
                primary = "B_INVALID_KNEE_CHARACTERJOINT_PRESENT";
            else if (report != null && !report.KneeCharacterJointResponseIsolationEnabled)
                primary = "B_INVALID_V076_TETHER_NOT_ACTIVE";
            else if (_invalidSteps > 0)
                primary = "B_INVALID_NON_FINITE_PARENT_ANCHOR_ACCELERATION";
            else if (_activeSteps <= 0)
                primary = "B_NO_FEEDFORWARD_EXPOSURE";
            else if (report != null && report.KneeCharacterJointResponseIsolationInvalidAnchorSteps <= 0 &&
                     report.KneeCharacterJointResponseIsolationPeakAnchorErrorMeters <= KneeCharacterJointResponseIsolation.TetherAnchorValidityMeters)
                primary = "B_PARENT_ANCHOR_ACCELERATION_FF_VALID_CLOSURE_CANDIDATE";
            else
                primary = "B_PARENT_ANCHOR_ACCELERATION_FF_ACTIVE_POSITIONAL_CLOSURE_STILL_INVALID";

            return
                "primary=" + primary +
                " | mode=" + Mode +
                " | arm=" + Arm +
                " | steps(active/warmup/total/invalid)=" + _activeSteps + "/" + _warmupSteps + "/" + _steps + "/" + _invalidSteps +
                " | peak(parentV/ffA)=" + F(_peakParentAnchorSpeed) + "/" + F(_peakFeedforwardAcceleration) +
                " | kneeTether(peakAnchor/invalid/clamped/peakAppliedA)=" +
                F(report != null ? report.KneeCharacterJointResponseIsolationPeakAnchorErrorMeters : 0f) + "/" +
                (report != null ? report.KneeCharacterJointResponseIsolationInvalidAnchorSteps : 0) + "/" +
                (report != null ? report.KneeCharacterJointResponseIsolationTetherClampedSteps : 0) + "/" +
                F(report != null ? report.KneeCharacterJointResponseIsolationPeakAppliedAcceleration : 0f) +
                " | knee(clamp/mean/max)=" +
                (report != null ? report.TraceKneeClampSamples : 0) + "/" +
                F(report != null ? report.TraceKneeRotationMeanDegrees : 0f) + "/" +
                F(report != null ? report.TraceKneeRotationMaxDegrees : 0f) +
                " | last=" + _lastEvidence +
                " | abort=" + _abortEvidence;
        }

        private static bool IsFinite(Vector3 value)
        {
            return !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                   !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                   !float.IsNaN(value.z) && !float.IsInfinity(value.z);
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
