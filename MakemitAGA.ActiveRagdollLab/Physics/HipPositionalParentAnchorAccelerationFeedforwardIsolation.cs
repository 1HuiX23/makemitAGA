using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.79 Hip positional parent-anchor acceleration feedforward causal A/B.
    ///
    /// v0.2.78 recovered ~1 FixedUpdate Knee angular timing, but normal walking still showed a small common-mode
    /// Upper/Lower world-position excursion when the leg reversed from forward extension into backward swing.
    /// Code audit matters here: the inertialized Hip mechanical parent is a kinematic Rigidbody commanded with
    /// MovePosition(sourceHipWorldPosition) every FixedUpdate, while the old v0.2.50 linear transport intentionally
    /// computes/uses commanded linear acceleration only during the strict Source/Recovery control-frame window.
    /// Therefore Healthy Normal walking can present an upcoming kinematic parent translation without any matching
    /// normal-window Hip positional acceleration feedforward.
    ///
    /// A preserves v0.2.78 exactly and returns zero feedforward. B derives the UPCOMING parent-anchor command velocity
    /// from (sourceHipCommand - settledMechanicalParentAnchor)/dt, compares it against the settled parent-anchor
    /// velocity measured from consecutive fixed steps, and returns only that command acceleration. The vector is added
    /// before the existing Hip tether's unchanged 400/40 correction and unchanged final 60 m/s^2 total clamp.
    /// It is deliberately suppressed whenever the frozen v0.2.50 strict Source/Recovery linear transport is active,
    /// so v0.2.79 changes only the previously unserved Healthy-Normal positional path.
    ///
    /// This module never calls Rigidbody force/torque/Move* APIs. It also records signed parent-local position/velocity
    /// evidence so a future visual report such as "pulled forward" can be checked directly: local X=lateral,
    /// Y=vertical, Z=forward in the active Hip mechanical-parent frame.
    /// </summary>
    internal sealed class HipPositionalParentAnchorAccelerationFeedforwardIsolation
    {
        internal const string Mode = "HIP_POSITIONAL_PARENT_ANCHOR_ACCELERATION_FEEDFORWARD_CAUSAL_AB";
        internal const string ArmA = "A_V078_PD_ONLY_HIP_TETHER";
        internal const string ArmB = "B_V079_UPCOMING_PARENT_COMMAND_ACCELERATION_FF";

        private bool _enabled;
        private bool _hasPreviousParentAnchorPosition;
        private Vector3 _previousParentAnchorPosition;
        private int _steps;
        private int _warmupSteps;
        private int _activeSteps;
        private int _strictLinearTransportSuppressedSteps;
        private int _invalidSteps;
        private Vector3 _currentFeedforwardWorld;
        private float _currentSettledParentSpeed;
        private float _currentCommandedParentSpeed;
        private float _currentCandidateAcceleration;
        private float _currentAppliedAcceleration;
        private float _peakSettledParentSpeed;
        private float _peakCommandedParentSpeed;
        private float _peakCandidateAcceleration;
        private float _peakAppliedAcceleration;
        private Vector3 _currentSourceToChildLocal;
        private Vector3 _currentSourceToParentLocal;
        private Vector3 _currentParentToChildLocal;
        private float _currentForwardVelocityError;
        private float _peakAbsSourceToChildForward;
        private float _peakAbsSourceToChildLateral;
        private float _peakAbsSourceToChildVertical;
        private float _peakAbsSourceToParentForward;
        private float _peakAbsParentToChildForward;
        private float _peakAbsForwardVelocityError;
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
            _hasPreviousParentAnchorPosition = false;
            _previousParentAnchorPosition = Vector3.zero;
            _steps = 0;
            _warmupSteps = 0;
            _activeSteps = 0;
            _strictLinearTransportSuppressedSteps = 0;
            _invalidSteps = 0;
            _currentFeedforwardWorld = Vector3.zero;
            _currentSettledParentSpeed = 0f;
            _currentCommandedParentSpeed = 0f;
            _currentCandidateAcceleration = 0f;
            _currentAppliedAcceleration = 0f;
            _peakSettledParentSpeed = 0f;
            _peakCommandedParentSpeed = 0f;
            _peakCandidateAcceleration = 0f;
            _peakAppliedAcceleration = 0f;
            _currentSourceToChildLocal = Vector3.zero;
            _currentSourceToParentLocal = Vector3.zero;
            _currentParentToChildLocal = Vector3.zero;
            _currentForwardVelocityError = 0f;
            _peakAbsSourceToChildForward = 0f;
            _peakAbsSourceToChildLateral = 0f;
            _peakAbsSourceToChildVertical = 0f;
            _peakAbsSourceToParentForward = 0f;
            _peakAbsParentToChildForward = 0f;
            _peakAbsForwardVelocityError = 0f;
            _lastEvidence = "<none>";
            _abortEvidence = "<none>";
            CurrentValid = true;
            CopyToReport(report);
        }

        internal Vector3 Step(
            Rigidbody mechanicalParent,
            Vector3 upcomingSourceHipAnchorWorld,
            Rigidbody upper,
            Vector3 childAnchorWorld,
            bool existingStrictLinearTransportActive,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            _currentFeedforwardWorld = Vector3.zero;
            _currentSettledParentSpeed = 0f;
            _currentCommandedParentSpeed = 0f;
            _currentCandidateAcceleration = 0f;
            _currentAppliedAcceleration = 0f;
            _currentSourceToChildLocal = Vector3.zero;
            _currentSourceToParentLocal = Vector3.zero;
            _currentParentToChildLocal = Vector3.zero;
            _currentForwardVelocityError = 0f;
            CurrentValid = true;
            _steps++;

            if (mechanicalParent == null || upper == null || fixedDeltaTime <= 0.0001f ||
                !IsFinite(upcomingSourceHipAnchorWorld) || !IsFinite(childAnchorWorld))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=missingBodyDtOrPosition";
                CopyToReport(report);
                return Vector3.zero;
            }

            Vector3 parentAnchorWorld = mechanicalParent.position;
            if (!IsFinite(parentAnchorWorld) || !IsFinite(mechanicalParent.rotation))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteMechanicalParent";
                CopyToReport(report);
                return Vector3.zero;
            }

            Quaternion inverseParent = Quaternion.Inverse(mechanicalParent.rotation);
            _currentSourceToChildLocal = inverseParent * (upcomingSourceHipAnchorWorld - childAnchorWorld);
            _currentSourceToParentLocal = inverseParent * (upcomingSourceHipAnchorWorld - parentAnchorWorld);
            _currentParentToChildLocal = inverseParent * (parentAnchorWorld - childAnchorWorld);
            UpdateSignedPeaks();

            Vector3 commandedParentVelocity = (upcomingSourceHipAnchorWorld - parentAnchorWorld) / fixedDeltaTime;
            if (!IsFinite(commandedParentVelocity))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteCommandedParentVelocity";
                CopyToReport(report);
                return Vector3.zero;
            }
            _currentCommandedParentSpeed = commandedParentVelocity.magnitude;
            _peakCommandedParentSpeed = Mathf.Max(_peakCommandedParentSpeed, _currentCommandedParentSpeed);

            if (!_hasPreviousParentAnchorPosition)
            {
                _hasPreviousParentAnchorPosition = true;
                _previousParentAnchorPosition = parentAnchorWorld;
                _warmupSteps++;
                _lastEvidence = "step=" + _steps + "/warmup=True/cmdV=" + F(_currentCommandedParentSpeed) + SignedEvidence();
                CopyToReport(report);
                return Vector3.zero;
            }

            Vector3 settledParentVelocity = (parentAnchorWorld - _previousParentAnchorPosition) / fixedDeltaTime;
            _previousParentAnchorPosition = parentAnchorWorld;
            if (!IsFinite(settledParentVelocity))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteSettledParentVelocity";
                CopyToReport(report);
                return Vector3.zero;
            }
            _currentSettledParentSpeed = settledParentVelocity.magnitude;
            _peakSettledParentSpeed = Mathf.Max(_peakSettledParentSpeed, _currentSettledParentSpeed);

            Vector3 candidateAcceleration = (commandedParentVelocity - settledParentVelocity) / fixedDeltaTime;
            if (!IsFinite(candidateAcceleration))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteCommandAcceleration";
                CopyToReport(report);
                return Vector3.zero;
            }

            Vector3 childLever = childAnchorWorld - upper.worldCenterOfMass;
            Vector3 childAnchorVelocity = upper.velocity + Vector3.Cross(upper.angularVelocity, childLever);
            if (!IsFinite(childAnchorVelocity))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteChildAnchorVelocity";
                CopyToReport(report);
                return Vector3.zero;
            }
            _currentForwardVelocityError = (inverseParent * (commandedParentVelocity - childAnchorVelocity)).z;
            _peakAbsForwardVelocityError = Mathf.Max(_peakAbsForwardVelocityError, Mathf.Abs(_currentForwardVelocityError));

            _currentCandidateAcceleration = candidateAcceleration.magnitude;
            _peakCandidateAcceleration = Mathf.Max(_peakCandidateAcceleration, _currentCandidateAcceleration);
            if (_enabled && existingStrictLinearTransportActive)
            {
                // v0.2.79 isolates the Healthy-Normal positional gap. During strict Source/Recovery the frozen
                // v0.2.50 common-mode linear transport is already active on Upper+Lower, so do not double-feed Upper.
                _strictLinearTransportSuppressedSteps++;
            }
            else if (_enabled)
            {
                _currentFeedforwardWorld = candidateAcceleration;
                _currentAppliedAcceleration = _currentCandidateAcceleration;
                _peakAppliedAcceleration = Mathf.Max(_peakAppliedAcceleration, _currentAppliedAcceleration);
                if (_currentAppliedAcceleration > 0.0001f)
                    _activeSteps++;
            }

            _lastEvidence =
                "step=" + _steps +
                "/warmup=False" +
                "/settledV=" + F(_currentSettledParentSpeed) +
                "/cmdV=" + F(_currentCommandedParentSpeed) +
                "/candidateA=" + F(_currentCandidateAcceleration) +
                "/appliedA=" + F(_currentAppliedAcceleration) +
                "/strictLinearSuppressed=" + existingStrictLinearTransportActive +
                SignedEvidence();
            CopyToReport(report);
            return _currentFeedforwardWorld;
        }

        internal void CaptureAbort(SingleLegChainReport report, string reason)
        {
            _abortEvidence =
                "reason=" + (reason ?? "<none>") +
                "/candidateA=" + F(_currentCandidateAcceleration) +
                "/appliedA=" + F(_currentAppliedAcceleration) +
                "/sourceToChildF=" + F(_currentSourceToChildLocal.z) +
                "/sourceToParentF=" + F(_currentSourceToParentLocal.z) +
                "/parentToChildF=" + F(_currentParentToChildLocal.z);
            CopyToReport(report);
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.HipAnchorAccelerationFeedforwardMode = Mode;
            report.HipAnchorAccelerationFeedforwardArm = Arm;
            report.HipAnchorAccelerationFeedforwardEnabled = _enabled;
            report.HipAnchorAccelerationFeedforwardSteps = _steps;
            report.HipAnchorAccelerationFeedforwardWarmupSteps = _warmupSteps;
            report.HipAnchorAccelerationFeedforwardActiveSteps = _activeSteps;
            report.HipAnchorAccelerationFeedforwardStrictLinearTransportSuppressedSteps = _strictLinearTransportSuppressedSteps;
            report.HipAnchorAccelerationFeedforwardInvalidSteps = _invalidSteps;
            report.HipAnchorAccelerationFeedforwardCurrentSettledParentSpeed = _currentSettledParentSpeed;
            report.HipAnchorAccelerationFeedforwardCurrentCommandedParentSpeed = _currentCommandedParentSpeed;
            report.HipAnchorAccelerationFeedforwardCurrentCandidateAcceleration = _currentCandidateAcceleration;
            report.HipAnchorAccelerationFeedforwardCurrentAppliedAcceleration = _currentAppliedAcceleration;
            report.HipAnchorAccelerationFeedforwardCurrentSourceToChildLateral = _currentSourceToChildLocal.x;
            report.HipAnchorAccelerationFeedforwardCurrentSourceToChildVertical = _currentSourceToChildLocal.y;
            report.HipAnchorAccelerationFeedforwardCurrentSourceToChildForward = _currentSourceToChildLocal.z;
            report.HipAnchorAccelerationFeedforwardCurrentSourceToParentForward = _currentSourceToParentLocal.z;
            report.HipAnchorAccelerationFeedforwardCurrentParentToChildForward = _currentParentToChildLocal.z;
            report.HipAnchorAccelerationFeedforwardCurrentForwardVelocityError = _currentForwardVelocityError;
            report.HipAnchorAccelerationFeedforwardPeakAbsSourceToChildForward = _peakAbsSourceToChildForward;
            report.HipAnchorAccelerationFeedforwardPeakAbsSourceToChildLateral = _peakAbsSourceToChildLateral;
            report.HipAnchorAccelerationFeedforwardPeakAbsSourceToChildVertical = _peakAbsSourceToChildVertical;
            report.HipAnchorAccelerationFeedforwardPeakAbsSourceToParentForward = _peakAbsSourceToParentForward;
            report.HipAnchorAccelerationFeedforwardPeakAbsParentToChildForward = _peakAbsParentToChildForward;
            report.HipAnchorAccelerationFeedforwardPeakAbsForwardVelocityError = _peakAbsForwardVelocityError;
            report.HipAnchorAccelerationFeedforwardCurrentValid = CurrentValid;
            report.HipAnchorAccelerationFeedforwardLastEvidence = _lastEvidence;
            report.HipAnchorAccelerationFeedforwardAbortEvidence = _abortEvidence;
            report.HipAnchorAccelerationFeedforwardKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return
                "ARL Hip positional parent-anchor acceleration FF A/B | mode=" + Mode +
                " | arm=" + Arm +
                " | enabled=" + _enabled +
                " | steps(active/warmup/strictSuppressed/total/invalid)=" + _activeSteps + "/" + _warmupSteps + "/" + _strictLinearTransportSuppressedSteps + "/" + _steps + "/" + _invalidSteps +
                " | current(settledV/cmdV/candidateA/appliedA)=" + F(_currentSettledParentSpeed) + "/" + F(_currentCommandedParentSpeed) + "/" + F(_currentCandidateAcceleration) + "/" + F(_currentAppliedAcceleration) +
                " | signedLocal(sourceToChild[x/y/z]/sourceToParentZ/parentToChildZ/vErrZ)=" + V(_currentSourceToChildLocal) + "/" + F(_currentSourceToParentLocal.z) + "/" + F(_currentParentToChildLocal.z) + "/" + F(_currentForwardVelocityError) +
                " | key=" + GetKey(report);
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            if (!_enabled)
                primary = "A_V078_PD_ONLY_HIP_TETHER";
            else if (report != null && report.HipCharacterJointPresent)
                primary = "B_INVALID_HIP_CHARACTERJOINT_PRESENT";
            else if (report != null && !report.HipTranslationTetherUpperOnlyEnabled)
                primary = "B_INVALID_REQUIRES_UPPER_ONLY_HIP_TETHER";
            else if (_invalidSteps > 0)
                primary = "B_INVALID_NON_FINITE_PARENT_COMMAND_ACCELERATION";
            else if (_activeSteps <= 0)
                primary = "B_NO_FEEDFORWARD_EXPOSURE";
            else if (report != null && report.HipCharacterJointResponseIsolationInvalidAnchorSteps <= 0)
                primary = "B_UPCOMING_PARENT_COMMAND_ACCELERATION_FF_ACTIVE_VALID";
            else
                primary = "B_FF_ACTIVE_HIP_TETHER_CLOSURE_INVALID";

            return
                "primary=" + primary +
                " | mode=" + Mode +
                " | arm=" + Arm +
                " | steps(active/warmup/strictSuppressed/total/invalid)=" + _activeSteps + "/" + _warmupSteps + "/" + _strictLinearTransportSuppressedSteps + "/" + _steps + "/" + _invalidSteps +
                " | peak(settledV/cmdV/candidateA/appliedA)=" + F(_peakSettledParentSpeed) + "/" + F(_peakCommandedParentSpeed) + "/" + F(_peakCandidateAcceleration) + "/" + F(_peakAppliedAcceleration) +
                " | signedPeakAbs(sourceToChildF/L/V,sourceToParentF,parentToChildF,vErrF)=" +
                F(_peakAbsSourceToChildForward) + "/" + F(_peakAbsSourceToChildLateral) + "/" + F(_peakAbsSourceToChildVertical) + "," +
                F(_peakAbsSourceToParentForward) + "/" + F(_peakAbsParentToChildForward) + "/" + F(_peakAbsForwardVelocityError) +
                " | hipTether(peakAnchor/clamped/invalid/peakAppliedA)=" +
                F(report != null ? report.HipCharacterJointResponseIsolationPeakAnchorErrorMeters : 0f) + "/" +
                (report != null ? report.HipCharacterJointResponseIsolationTetherClampedSteps : 0) + "/" +
                (report != null ? report.HipCharacterJointResponseIsolationInvalidAnchorSteps : 0) + "/" +
                F(report != null ? report.HipCharacterJointResponseIsolationPeakAppliedAcceleration : 0f) +
                " | driveAborted=" + (report != null ? report.DriveAborted : false) +
                " | last=" + _lastEvidence +
                " | localAbortEvidence=" + _abortEvidence;
        }

        private void UpdateSignedPeaks()
        {
            _peakAbsSourceToChildLateral = Mathf.Max(_peakAbsSourceToChildLateral, Mathf.Abs(_currentSourceToChildLocal.x));
            _peakAbsSourceToChildVertical = Mathf.Max(_peakAbsSourceToChildVertical, Mathf.Abs(_currentSourceToChildLocal.y));
            _peakAbsSourceToChildForward = Mathf.Max(_peakAbsSourceToChildForward, Mathf.Abs(_currentSourceToChildLocal.z));
            _peakAbsSourceToParentForward = Mathf.Max(_peakAbsSourceToParentForward, Mathf.Abs(_currentSourceToParentLocal.z));
            _peakAbsParentToChildForward = Mathf.Max(_peakAbsParentToChildForward, Mathf.Abs(_currentParentToChildLocal.z));
        }

        private string SignedEvidence()
        {
            return
                "/sourceToChildLocal=" + V(_currentSourceToChildLocal) +
                "/sourceToParentF=" + F(_currentSourceToParentLocal.z) +
                "/parentToChildF=" + F(_currentParentToChildLocal.z) +
                "/forwardVelErr=" + F(_currentForwardVelocityError);
        }

        private static bool IsFinite(Vector3 value)
        {
            return !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                   !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                   !float.IsNaN(value.z) && !float.IsInfinity(value.z);
        }

        private static bool IsFinite(Quaternion value)
        {
            return !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                   !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                   !float.IsNaN(value.z) && !float.IsInfinity(value.z) &&
                   !float.IsNaN(value.w) && !float.IsInfinity(value.w);
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
