using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.73-v0.2.74 Hip backend causal isolation.
    ///
    /// v0.2.73 A/B:
    /// A keeps the complete v0.2.72 Hip CharacterJoint backend.
    /// B removes the Hip CharacterJoint component while drive/trace are OFF, then preserves Hip-anchor translation
    /// with an ARL-owned linear acceleration tether applied at the Upper COM. v0.2.73 originally applied the same
    /// acceleration to Upper+Lower COMs to avoid introducing a differential Knee acceleration.
    ///
    /// v0.2.74 serial-chain A/B keeps Hip CharacterJoint REMOVED in both arms and changes one variable only:
    /// A preserves the v0.2.73 common-mode Upper+Lower tether; B applies the Hip tether to Upper ONLY so Lower is
    /// transported through the existing Upper -> Knee CharacterJoint -> Lower serial chain instead of being directly
    /// dragged by the Hip tether. The tether still applies at COM and contributes no deliberate torque.
    ///
    /// The unchanged Hip AddTorque controller remains the only deliberate Hip angular actuator.
    /// </summary>
    internal sealed class HipCharacterJointResponseIsolation
    {
        internal const string Mode = "HIP_CHARACTER_JOINT_RESPONSE_ISOLATION_CAUSAL_AB";
        internal const string ProductionArm = "A_V072_CHARACTER_JOINT_BACKEND";
        internal const string ExperimentalArm = "B_V073_NO_HIP_CHARACTERJOINT_TRANSLATION_ONLY_TETHER";
        internal const string SerialChainMode = "HIP_TRANSLATION_TETHER_SERIAL_CHAIN_RESTORATION_CAUSAL_AB";
        internal const string SerialChainArmA = "A_V073_COMMON_MODE_UPPER_LOWER_TETHER";
        internal const string SerialChainArmB = "B_V074_UPPER_ONLY_HIP_TETHER";

        // Fixed experimental constants: no profile knobs. 400/40 is critically damped for wn=20 rad/s.
        internal const float TetherPositionSpring = 400f;
        internal const float TetherVelocityDamper = 40f;
        internal const float TetherMaxAcceleration = 60f;
        internal const float TetherAnchorValidityMeters = 0.03f;

        private bool _enabled;
        private bool _upperOnlyTether;
        private int _steps;
        private int _upperForceApplications;
        private int _lowerForceApplications;
        private int _clampedSteps;
        private int _invalidAnchorSteps;
        private float _currentAnchorError;
        private float _currentVelocityError;
        private float _currentPdAcceleration;
        private float _currentFeedforwardAcceleration;
        private float _currentRawAcceleration;
        private float _currentAppliedAcceleration;
        private float _peakAnchorError;
        private float _peakVelocityError;
        private float _peakPdAcceleration;
        private float _peakFeedforwardAcceleration;
        private float _peakRawAcceleration;
        private float _peakAppliedAcceleration;
        private bool _currentClamped;
        private Vector3 _currentAppliedWorld;
        private string _lastEvidence = "<none>";
        private string _abortEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal Vector3 CurrentAppliedAccelerationWorld => _currentAppliedWorld;
        internal bool CurrentAnchorValid => !_enabled || _currentAnchorError <= TetherAnchorValidityMeters;
        internal bool UpperOnlyTetherEnabled => _upperOnlyTether;
        internal string Arm => _enabled ? ExperimentalArm : ProductionArm;
        internal string SerialChainArm => _upperOnlyTether ? SerialChainArmB : SerialChainArmA;

        internal void SetEnabled(bool enabled, SingleLegChainReport report)
        {
            _enabled = enabled;
            ResetStats(report);
        }

        internal void SetUpperOnlyTetherEnabled(bool enabled, SingleLegChainReport report)
        {
            _upperOnlyTether = enabled;
            ResetStats(report);
        }

        internal void ResetStats(SingleLegChainReport report)
        {
            _steps = 0;
            _upperForceApplications = 0;
            _lowerForceApplications = 0;
            _clampedSteps = 0;
            _invalidAnchorSteps = 0;
            _currentAnchorError = 0f;
            _currentVelocityError = 0f;
            _currentPdAcceleration = 0f;
            _currentFeedforwardAcceleration = 0f;
            _currentRawAcceleration = 0f;
            _currentAppliedAcceleration = 0f;
            _peakAnchorError = 0f;
            _peakVelocityError = 0f;
            _peakPdAcceleration = 0f;
            _peakFeedforwardAcceleration = 0f;
            _peakRawAcceleration = 0f;
            _peakAppliedAcceleration = 0f;
            _currentClamped = false;
            _currentAppliedWorld = Vector3.zero;
            _lastEvidence = "<none>";
            _abortEvidence = "<none>";
            CopyToReport(report, null);
        }

        internal Vector3 ApplyTranslationOnlyTether(
            Rigidbody upper,
            Rigidbody lower,
            Rigidbody parent,
            Vector3 childAnchorWorld,
            Vector3 parentAnchorWorld,
            Vector3 accelerationFeedforwardWorld,
            float fixedDeltaTime,
            SingleLegChainReport report, Vector3? settledParentVelocityWorld = null)
        {
            if (!_enabled || upper == null || lower == null || parent == null || fixedDeltaTime <= 0.0001f)
            {
                _currentAnchorError = 0f;
                _currentVelocityError = 0f;
                _currentPdAcceleration = 0f;
                _currentFeedforwardAcceleration = 0f;
                _currentRawAcceleration = 0f;
                _currentAppliedAcceleration = 0f;
                _currentClamped = false;
                _currentAppliedWorld = Vector3.zero;
                CopyToReport(report, null);
                return Vector3.zero;
            }

            Vector3 childLever = childAnchorWorld - upper.worldCenterOfMass;
            Vector3 parentLever = parentAnchorWorld - parent.worldCenterOfMass;
            Vector3 childVelocity = upper.velocity + Vector3.Cross(upper.angularVelocity, childLever);
            Vector3 parentVelocity = settledParentVelocityWorld ?? (parent.velocity + Vector3.Cross(parent.angularVelocity, parentLever));
            Vector3 positionError = parentAnchorWorld - childAnchorWorld;
            Vector3 velocityError = parentVelocity - childVelocity;

            Vector3 pdRaw = positionError * TetherPositionSpring + velocityError * TetherVelocityDamper;
            Vector3 raw = pdRaw + accelerationFeedforwardWorld;
            Vector3 applied = raw;
            bool clamped = false;
            float rawMag = raw.magnitude;
            if (rawMag > TetherMaxAcceleration && rawMag > 0.0001f)
            {
                applied = raw * (TetherMaxAcceleration / rawMag);
                clamped = true;
            }

            _currentAppliedWorld = applied;
            upper.AddForce(applied, ForceMode.Acceleration);
            _upperForceApplications++;
            if (!_upperOnlyTether)
            {
                lower.AddForce(applied, ForceMode.Acceleration);
                _lowerForceApplications++;
            }

            _steps++;
            if (clamped) _clampedSteps++;
            _currentAnchorError = positionError.magnitude;
            _currentVelocityError = velocityError.magnitude;
            _currentPdAcceleration = pdRaw.magnitude;
            _currentFeedforwardAcceleration = accelerationFeedforwardWorld.magnitude;
            _currentRawAcceleration = rawMag;
            _currentAppliedAcceleration = applied.magnitude;
            _currentClamped = clamped;
            _peakAnchorError = Mathf.Max(_peakAnchorError, _currentAnchorError);
            _peakVelocityError = Mathf.Max(_peakVelocityError, _currentVelocityError);
            _peakPdAcceleration = Mathf.Max(_peakPdAcceleration, _currentPdAcceleration);
            _peakFeedforwardAcceleration = Mathf.Max(_peakFeedforwardAcceleration, _currentFeedforwardAcceleration);
            _peakRawAcceleration = Mathf.Max(_peakRawAcceleration, _currentRawAcceleration);
            _peakAppliedAcceleration = Mathf.Max(_peakAppliedAcceleration, _currentAppliedAcceleration);
            if (_currentAnchorError > TetherAnchorValidityMeters)
                _invalidAnchorSteps++;

            _lastEvidence =
                "step=" + _steps +
                "/anchorErr=" + F(_currentAnchorError) +
                "/velErr=" + F(_currentVelocityError) +
                "/pdA=" + F(_currentPdAcceleration) +
                "/ffA=" + F(_currentFeedforwardAcceleration) +
                "/rawA=" + F(_currentRawAcceleration) +
                "/appliedA=" + F(_currentAppliedAcceleration) +
                "/scope=" + (_upperOnlyTether ? "UPPER_ONLY" : "UPPER_PLUS_LOWER") +
                "/clamp=" + clamped;

            CopyToReport(report, applied);
            return applied;
        }

        internal void CaptureAbort(SingleLegChainReport report, string reason)
        {
            if (!_enabled) return;
            _abortEvidence =
                "reason=" + (reason ?? "<none>") +
                "/anchorErr=" + F(_currentAnchorError) +
                "/peakAnchorErr=" + F(_peakAnchorError) +
                "/appliedA=" + F(_currentAppliedAcceleration) +
                "/peakAppliedA=" + F(_peakAppliedAcceleration);
            CopyToReport(report, null);
        }

        internal void CopyToReport(SingleLegChainReport report, Vector3? appliedWorld)
        {
            if (report == null) return;
            report.HipCharacterJointResponseIsolationMode = Mode;
            report.HipCharacterJointResponseIsolationArm = Arm;
            report.HipCharacterJointResponseIsolationEnabled = _enabled;
            report.HipTranslationTetherSerialChainMode = SerialChainMode;
            report.HipTranslationTetherSerialChainArm = SerialChainArm;
            report.HipTranslationTetherUpperOnlyEnabled = _upperOnlyTether;
            report.HipTranslationTetherUpperForceApplications = _upperForceApplications;
            report.HipTranslationTetherLowerForceApplications = _lowerForceApplications;
            report.HipCharacterJointResponseIsolationTetherSteps = _steps;
            report.HipCharacterJointResponseIsolationTetherClampedSteps = _clampedSteps;
            report.HipCharacterJointResponseIsolationInvalidAnchorSteps = _invalidAnchorSteps;
            report.HipCharacterJointResponseIsolationCurrentAnchorErrorMeters = _currentAnchorError;
            report.HipCharacterJointResponseIsolationCurrentVelocityError = _currentVelocityError;
            report.HipCharacterJointResponseIsolationCurrentPdAcceleration = _currentPdAcceleration;
            report.HipCharacterJointResponseIsolationCurrentFeedforwardAcceleration = _currentFeedforwardAcceleration;
            report.HipCharacterJointResponseIsolationCurrentRawAcceleration = _currentRawAcceleration;
            report.HipCharacterJointResponseIsolationCurrentAppliedAcceleration = _currentAppliedAcceleration;
            report.HipCharacterJointResponseIsolationCurrentClamped = _currentClamped;
            report.HipCharacterJointResponseIsolationPeakAnchorErrorMeters = _peakAnchorError;
            report.HipCharacterJointResponseIsolationPeakVelocityError = _peakVelocityError;
            report.HipCharacterJointResponseIsolationPeakPdAcceleration = _peakPdAcceleration;
            report.HipCharacterJointResponseIsolationPeakFeedforwardAcceleration = _peakFeedforwardAcceleration;
            report.HipCharacterJointResponseIsolationPeakRawAcceleration = _peakRawAcceleration;
            report.HipCharacterJointResponseIsolationPeakAppliedAcceleration = _peakAppliedAcceleration;
            report.HipCharacterJointResponseIsolationTetherSpring = TetherPositionSpring;
            report.HipCharacterJointResponseIsolationTetherDamper = TetherVelocityDamper;
            report.HipCharacterJointResponseIsolationTetherMaxAcceleration = TetherMaxAcceleration;
            report.HipCharacterJointResponseIsolationAnchorValidityMeters = TetherAnchorValidityMeters;
            report.HipCharacterJointResponseIsolationLastEvidence = _lastEvidence;
            report.HipCharacterJointResponseIsolationAbortEvidence = _abortEvidence;
            report.HipCharacterJointResponseIsolationKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report, null);
            return "ARL Hip CharacterJoint response isolation | mode=" + Mode +
                   " | arm=" + Arm +
                   " | jointPresent=" + (report != null && report.HipCharacterJointPresent) +
                   " | serialArm=" + SerialChainArm +
                   " | tetherScope=" + (_upperOnlyTether ? "UPPER_ONLY" : "UPPER_PLUS_LOWER") +
                   " | forceApplications(upper/lower)=" + _upperForceApplications + "/" + _lowerForceApplications +
                   " | tether(kp/kd/maxA)=" + F(TetherPositionSpring) + "/" + F(TetherVelocityDamper) + "/" + F(TetherMaxAcceleration) +
                   " | current(anchorErr/velErr/pdA/ffA/rawA/appliedA/clamp)=" + F(_currentAnchorError) + "/" + F(_currentVelocityError) + "/" + F(_currentPdAcceleration) + "/" + F(_currentFeedforwardAcceleration) + "/" + F(_currentRawAcceleration) + "/" + F(_currentAppliedAcceleration) + "/" + _currentClamped +
                   " | peak(anchorErr/appliedA)=" + F(_peakAnchorError) + "/" + F(_peakAppliedAcceleration);
        }

        internal string GetSerialChainStatus(SingleLegChainReport report)
        {
            CopyToReport(report, null);
            return "ARL Hip translation tether serial-chain A/B | mode=" + SerialChainMode +
                   " | arm=" + SerialChainArm +
                   " | jointIsolation=" + _enabled +
                   " | jointPresent=" + (report != null && report.HipCharacterJointPresent) +
                   " | tetherScope=" + (_upperOnlyTether ? "UPPER_ONLY" : "UPPER_PLUS_LOWER") +
                   " | forceApplications(upper/lower)=" + _upperForceApplications + "/" + _lowerForceApplications +
                   " | current(anchorErr/appliedA)=" + F(_currentAnchorError) + "/" + F(_currentAppliedAcceleration) +
                   " | peak(anchorErr/appliedA)=" + F(_peakAnchorError) + "/" + F(_peakAppliedAcceleration) +
                   " | key=" + GetSerialChainKey(report);
        }

        internal string GetSerialChainKey(SingleLegChainReport report)
        {
            string primary;
            if (!_enabled)
                primary = "INVALID_REQUIRES_V073_NO_HIP_CHARACTERJOINT_BASELINE";
            else if (!_upperOnlyTether)
                primary = "A_V073_COMMON_MODE_TETHER_ACTIVE";
            else if (_lowerForceApplications != 0)
                primary = "B_INVALID_LOWER_DIRECT_HIP_TETHER_FORCE_OBSERVED";
            else if (_invalidAnchorSteps > 0)
                primary = "B_UPPER_ONLY_TETHER_ANCHOR_VALIDITY_EXCEEDED";
            else
                primary = "B_UPPER_ONLY_SERIAL_CHAIN_VALID";

            return "primary=" + primary +
                   " | mode=" + SerialChainMode +
                   " | arm=" + SerialChainArm +
                   " | scope=" + (_upperOnlyTether ? "UPPER_ONLY" : "UPPER_PLUS_LOWER") +
                   " | forceApplications(upper/lower)=" + _upperForceApplications + "/" + _lowerForceApplications +
                   " | tether(steps/clamped/invalidAnchor)=" + _steps + "/" + _clampedSteps + "/" + _invalidAnchorSteps +
                   " | peak(anchorErr/appliedA)=" + F(_peakAnchorError) + "/" + F(_peakAppliedAcceleration) +
                   " | response(hipPeak/kneePeak)=" +
                   F(report != null ? report.PeakHipRotationErrorDegrees : 0f) + "/" +
                   F(report != null ? report.PeakKneeRotationErrorDegrees : 0f);
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            if (!_enabled)
                primary = "A_V072_CHARACTER_JOINT_BACKEND_ACTIVE";
            else if (_invalidAnchorSteps > 0)
                primary = "B_TETHER_ANCHOR_VALIDITY_EXCEEDED_CAUSAL_RUN_REQUIRES_REVIEW";
            else
                primary = "B_NO_HIP_CHARACTERJOINT_TRANSLATION_TETHER_VALID";

            return "primary=" + primary +
                   " | mode=" + Mode +
                   " | arm=" + Arm +
                   " | jointPresent=" + (report != null && report.HipCharacterJointPresent) +
                   " | serialArm=" + SerialChainArm +
                   " | tetherScope=" + (_upperOnlyTether ? "UPPER_ONLY" : "UPPER_PLUS_LOWER") +
                   " | forceApplications(upper/lower)=" + _upperForceApplications + "/" + _lowerForceApplications +
                   " | tether(steps/clamped/invalidAnchor)=" + _steps + "/" + _clampedSteps + "/" + _invalidAnchorSteps +
                   " | current(anchorErr/velErr/rawA/appliedA)=" + F(_currentAnchorError) + "/" + F(_currentVelocityError) + "/" + F(_currentRawAcceleration) + "/" + F(_currentAppliedAcceleration) +
                   " | peak(anchorErr/velErr/rawA/appliedA)=" + F(_peakAnchorError) + "/" + F(_peakVelocityError) + "/" + F(_peakRawAcceleration) + "/" + F(_peakAppliedAcceleration) +
                   " | response(hipPeak)=" + F(report != null ? report.PeakHipRotationErrorDegrees : 0f) +
                   " | abort=" + (report != null && report.DriveAborted) + "/" + (report != null ? (report.DriveAbortReason ?? "<none>") : "<none>");
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
