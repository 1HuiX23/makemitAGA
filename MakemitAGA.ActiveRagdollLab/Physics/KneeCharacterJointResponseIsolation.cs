using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.76 Knee CharacterJoint response causal A/B.
    ///
    /// A keeps the complete v0.2.75 Knee CharacterJoint backend.
    /// B removes the Knee CharacterJoint while drive/trace/calibration are OFF and preserves only Knee-anchor
    /// translation with an ARL-owned COM force on Lower. The tether deliberately applies no torque and no equal/
    /// opposite force to Upper: this is a causal isolation arm for downstream CharacterJoint solver reaction, not
    /// yet a momentum-conserving production constraint.
    ///
    /// Lower angular control remains exactly v0.2.75: same Knee relative PD/PCA/A-FF plus the same Hip-command
    /// common-mode parent angular transport. Therefore the intended changed physical variable is the presence of
    /// the Knee CharacterJoint constraint solver.
    /// </summary>
    internal sealed class KneeCharacterJointResponseIsolation
    {
        internal const string Mode = "KNEE_CHARACTER_JOINT_RESPONSE_ISOLATION_CAUSAL_AB";
        internal const string ProductionArm = "A_V075_KNEE_CHARACTERJOINT_BACKEND";
        internal const string ExperimentalArm = "B_V076_NO_KNEE_CHARACTERJOINT_TRANSLATION_ONLY_TETHER";

        // Fixed causal-isolation constants; intentionally not profile knobs.
        internal const float TetherPositionSpring = 400f;
        internal const float TetherVelocityDamper = 40f;
        internal const float TetherMaxAcceleration = 60f;
        internal const float TetherAnchorValidityMeters = 0.03f;

        private bool _enabled;
        private bool _deferredForce;
        internal bool CurrentSupportNormalConstraint { get; private set; }
        internal bool DeferredForcePending => _deferredForce;
        internal void CancelDeferredForce()
        {
            if (_deferredForce)
            {
                // An early safety gate may cancel B before its single native write. Do not
                // present the provisional baseline vector as an actually applied command.
                _currentAppliedAcceleration = 0f;
                _currentAppliedAccelerationWorld = Vector3.zero;
                _currentClamped = false;
            }
            _deferredForce = false;
            CurrentSupportNormalConstraint = false;
        }
        private int _steps;
        private int _forceApplications;
        private int _clampedSteps;
        private int _invalidAnchorSteps;
        private float _currentAnchorError;
        private float _currentVelocityError;
        private float _currentRawAcceleration;
        private float _currentAppliedAcceleration;
        private float _peakAnchorError;
        private float _peakVelocityError;
        private float _peakRawAcceleration;
        private float _peakAppliedAcceleration;
        private bool _currentClamped;
        // Original v0.3.3 safety taps store the computed tether vectors. V04430 B also consumes
        // the prepared error/raw snapshots once, after current lifecycle and before its native write.
        private Vector3 _currentPositionErrorWorld;
        private Vector3 _currentVelocityErrorWorld;
        private Vector3 _currentParentAnchorFeedforwardWorld;
        private Vector3 _currentRawAccelerationWorld;
        private Vector3 _currentAppliedAccelerationWorld;
        private string _lastEvidence = "<none>";
        private string _abortEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal string Arm => _enabled ? ExperimentalArm : ProductionArm;
        internal bool CurrentAnchorValid => !_enabled || _currentAnchorError <= TetherAnchorValidityMeters;
        internal Vector3 CurrentPositionErrorWorld => _currentPositionErrorWorld;
        internal Vector3 CurrentVelocityErrorWorld => _currentVelocityErrorWorld;
        internal Vector3 CurrentParentAnchorFeedforwardWorld => _currentParentAnchorFeedforwardWorld;
        internal Vector3 CurrentRawAccelerationWorld => _currentRawAccelerationWorld;
        internal Vector3 CurrentAppliedAccelerationWorld => _currentAppliedAccelerationWorld;
        internal bool CurrentClamped => _currentClamped;

        internal void SetEnabled(bool enabled, SingleLegChainReport report)
        {
            _enabled = enabled;
            ResetStats(report);
        }

        internal void ResetStats(SingleLegChainReport report)
        {
            CancelDeferredForce();
            _steps = 0;
            _forceApplications = 0;
            _clampedSteps = 0;
            _invalidAnchorSteps = 0;
            _currentAnchorError = 0f;
            _currentVelocityError = 0f;
            _currentRawAcceleration = 0f;
            _currentAppliedAcceleration = 0f;
            _peakAnchorError = 0f;
            _peakVelocityError = 0f;
            _peakRawAcceleration = 0f;
            _peakAppliedAcceleration = 0f;
            _currentClamped = false;
            _currentPositionErrorWorld = Vector3.zero;
            _currentVelocityErrorWorld = Vector3.zero;
            _currentParentAnchorFeedforwardWorld = Vector3.zero;
            _currentRawAccelerationWorld = Vector3.zero;
            _currentAppliedAccelerationWorld = Vector3.zero;
            _lastEvidence = "<none>";
            _abortEvidence = "<none>";
            CopyToReport(report);
        }

        internal Vector3 ApplyTranslationOnlyTether(
            Rigidbody upper,
            Rigidbody lower,
            Vector3 parentAnchorUpperLocal,
            Vector3 childAnchorLowerLocal,
            Vector3 parentAnchorAccelerationFeedforwardWorld,
            float fixedDeltaTime,
            SingleLegChainReport report, bool deferForce = false)
        {
            if (_deferredForce) throw new System.InvalidOperationException("KNEE_UNCOMMITTED_PREVIOUS_COMMAND");
            CurrentSupportNormalConstraint = false;
            if (!_enabled || upper == null || lower == null || fixedDeltaTime <= 0.0001f)
            {
                _currentAnchorError = 0f;
                _currentVelocityError = 0f;
                _currentRawAcceleration = 0f;
                _currentAppliedAcceleration = 0f;
                _currentClamped = false;
                _currentPositionErrorWorld = Vector3.zero;
                _currentVelocityErrorWorld = Vector3.zero;
                _currentParentAnchorFeedforwardWorld = Vector3.zero;
                _currentRawAccelerationWorld = Vector3.zero;
                _currentAppliedAccelerationWorld = Vector3.zero;
                CopyToReport(report);
                return Vector3.zero;
            }

            Vector3 parentAnchorWorld = upper.transform.TransformPoint(parentAnchorUpperLocal);
            Vector3 childAnchorWorld = lower.transform.TransformPoint(childAnchorLowerLocal);
            Vector3 parentLever = parentAnchorWorld - upper.worldCenterOfMass;
            Vector3 childLever = childAnchorWorld - lower.worldCenterOfMass;
            Vector3 parentVelocity = upper.velocity + Vector3.Cross(upper.angularVelocity, parentLever);
            Vector3 childVelocity = lower.velocity + Vector3.Cross(lower.angularVelocity, childLever);
            Vector3 positionError = parentAnchorWorld - childAnchorWorld;
            Vector3 velocityError = parentVelocity - childVelocity;

            // v0.2.77 A passes Vector3.zero and is exactly the v0.2.76 PD-only tether. B adds measured
            // parent-anchor world acceleration before the same unchanged final 60 m/s^2 total clamp.
            Vector3 raw = parentAnchorAccelerationFeedforwardWorld +
                          positionError * TetherPositionSpring +
                          velocityError * TetherVelocityDamper;
            Vector3 applied = raw;
            bool clamped = false;
            float rawMag = raw.magnitude;
            if (rawMag > TetherMaxAcceleration && rawMag > 0.0001f)
            {
                applied = raw * (TetherMaxAcceleration / rawMag);
                clamped = true;
            }

            // Causal invariant: COM force only. No AddTorque, AddForceAtPosition, MovePosition/MoveRotation,
            // angularVelocity write, and no equal/opposite force on Upper.
            // A retains the original call and arithmetic. B waits for this solve's Foot lifecycle,
            // so a release cannot accidentally receive last step's support-normal constraint.
            _deferredForce = deferForce;
            if (!deferForce)
            {
                lower.AddForce(applied, ForceMode.Acceleration);
                _forceApplications++;
                if (clamped) _clampedSteps++;
            }
            _steps++;

            _currentAnchorError = positionError.magnitude;
            _currentVelocityError = velocityError.magnitude;
            _currentRawAcceleration = rawMag;
            _currentAppliedAcceleration = applied.magnitude;
            _currentClamped = clamped;
            _currentPositionErrorWorld = positionError;
            _currentVelocityErrorWorld = velocityError;
            _currentParentAnchorFeedforwardWorld = parentAnchorAccelerationFeedforwardWorld;
            _currentRawAccelerationWorld = raw;
            _currentAppliedAccelerationWorld = applied;
            _peakAnchorError = Mathf.Max(_peakAnchorError, _currentAnchorError);
            _peakVelocityError = Mathf.Max(_peakVelocityError, _currentVelocityError);
            if (!deferForce)
            {
                _peakRawAcceleration = Mathf.Max(_peakRawAcceleration, _currentRawAcceleration);
                _peakAppliedAcceleration = Mathf.Max(_peakAppliedAcceleration, _currentAppliedAcceleration);
            }
            if (_currentAnchorError > TetherAnchorValidityMeters)
                _invalidAnchorSteps++;

            _lastEvidence =
                "step=" + _steps +
                "/anchorErr=" + F(_currentAnchorError) +
                "/velErr=" + F(_currentVelocityError) +
                "/ffA=" + F(parentAnchorAccelerationFeedforwardWorld.magnitude) +
                "/rawA=" + F(_currentRawAcceleration) +
                "/candidateOrAppliedA=" + F(_currentAppliedAcceleration) +
                "/forceWritten=" + !deferForce + "/clamp=" + clamped;

            CopyToReport(report);
            return applied;
        }

        // V04430 B: normal constraint acts on both finite-mass COMs. The normal relative
        // acceleration is (1/mLower + 1/mUpper)*F, hence reduced mass, not mLower.
        // Measured parent acceleration is NOT an external acceleration once this actuator
        // moves Upper. Remove that delayed normal FF from the reciprocal constraint entirely.
        // Keep the original tangential raw command and the same total 60 m/s^2 vector cap.
        internal static Vector3 SupportNormalRaw(Vector3 baselineRaw, Vector3 positionError,
            Vector3 velocityError, Vector3 unitNormal, float lowerMass, float upperMass)
        {
            float lowerShare = 1f / (1f + lowerMass / upperMass);
            float relativePd = Vector3.Dot(positionError * TetherPositionSpring +
                velocityError * TetherVelocityDamper, unitNormal);
            return baselineRaw - Vector3.Dot(baselineRaw, unitNormal) * unitNormal +
                (lowerShare * relativePd) * unitNormal;
        }

        internal bool ApplyDeferredTranslationOnlyTether(Rigidbody upper, Rigidbody lower,
            bool exactAuthorizedStance, bool zeroFriction, Vector3 supportNormal, SingleLegChainReport report)
        {
            if (!_deferredForce || !_enabled || upper == null || lower == null || upper == lower ||
                upper.isKinematic || lower.isKinematic || !CurrentAnchorValid)
                return false;
            Vector3 raw = _currentRawAccelerationWorld;
            if (exactAuthorizedStance)
            {
                if (!zeroFriction || upper.useGravity || !Finite(upper.mass) || !Finite(lower.mass) ||
                    upper.mass <= 0.0000001f || lower.mass <= 0.0000001f || !Finite(supportNormal) ||
                    !Finite(supportNormal.sqrMagnitude) || supportNormal.sqrMagnitude <= 0.0000001f)
                    return false;
                Vector3 n = supportNormal.normalized;
                raw = SupportNormalRaw(raw, _currentPositionErrorWorld, _currentVelocityErrorWorld, n, lower.mass, upper.mass);
            }
            float magnitude = raw.magnitude;
            if (!Finite(raw) || !Finite(magnitude)) return false;
            bool clamped = magnitude > TetherMaxAcceleration;
            Vector3 applied = clamped ? raw * (TetherMaxAcceleration / magnitude) : raw;
            if (!Finite(applied)) return false;
            // Consume before native call; exceptions go through the chain's fail-closed path.
            _deferredForce = false;
            lower.AddForce(applied, ForceMode.Acceleration);
            CurrentSupportNormalConstraint = exactAuthorizedStance;
            _forceApplications++;
            if (clamped) _clampedSteps++;
            _currentRawAccelerationWorld = raw;
            _currentAppliedAccelerationWorld = applied;
            _currentRawAcceleration = magnitude;
            _currentAppliedAcceleration = applied.magnitude;
            _currentClamped = clamped;
            _peakRawAcceleration = Mathf.Max(_peakRawAcceleration, magnitude);
            _peakAppliedAcceleration = Mathf.Max(_peakAppliedAcceleration, _currentAppliedAcceleration);
            _lastEvidence = "step=" + _steps + "/normalReducedMassPd=" + CurrentSupportNormalConstraint +
                "/anchorErr=" + F(_currentAnchorError) + "/velErr=" + F(_currentVelocityError) +
                "/rawA=" + F(magnitude) + "/appliedA=" + F(_currentAppliedAcceleration) + "/forceWritten=True/clamp=" + clamped;
            CopyToReport(report);
            return true;
        }

        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool Finite(Vector3 value) => Finite(value.x) && Finite(value.y) && Finite(value.z);

        internal void CaptureAbort(SingleLegChainReport report, string reason)
        {
            if (!_enabled) return;
            _abortEvidence =
                "reason=" + (reason ?? "<none>") +
                "/anchorErr=" + F(_currentAnchorError) +
                "/peakAnchorErr=" + F(_peakAnchorError) +
                "/appliedA=" + F(_currentAppliedAcceleration) +
                "/peakAppliedA=" + F(_peakAppliedAcceleration);
            CopyToReport(report);
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.KneeCharacterJointResponseIsolationMode = Mode;
            report.KneeCharacterJointResponseIsolationArm = Arm;
            report.KneeCharacterJointResponseIsolationEnabled = _enabled;
            report.KneeCharacterJointResponseIsolationTetherSteps = _steps;
            report.KneeCharacterJointResponseIsolationForceApplications = _forceApplications;
            report.KneeCharacterJointResponseIsolationTetherClampedSteps = _clampedSteps;
            report.KneeCharacterJointResponseIsolationInvalidAnchorSteps = _invalidAnchorSteps;
            report.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters = _currentAnchorError;
            report.KneeCharacterJointResponseIsolationCurrentVelocityError = _currentVelocityError;
            report.KneeCharacterJointResponseIsolationCurrentRawAcceleration = _currentRawAcceleration;
            report.KneeCharacterJointResponseIsolationCurrentAppliedAcceleration = _currentAppliedAcceleration;
            report.KneeCharacterJointResponseIsolationCurrentClamped = _currentClamped;
            report.KneeCharacterJointResponseIsolationPeakAnchorErrorMeters = _peakAnchorError;
            report.KneeCharacterJointResponseIsolationPeakVelocityError = _peakVelocityError;
            report.KneeCharacterJointResponseIsolationPeakRawAcceleration = _peakRawAcceleration;
            report.KneeCharacterJointResponseIsolationPeakAppliedAcceleration = _peakAppliedAcceleration;
            report.KneeCharacterJointResponseIsolationTetherSpring = TetherPositionSpring;
            report.KneeCharacterJointResponseIsolationTetherDamper = TetherVelocityDamper;
            report.KneeCharacterJointResponseIsolationTetherMaxAcceleration = TetherMaxAcceleration;
            report.KneeCharacterJointResponseIsolationAnchorValidityMeters = TetherAnchorValidityMeters;
            report.KneeCharacterJointResponseIsolationLastEvidence = _lastEvidence;
            report.KneeCharacterJointResponseIsolationAbortEvidence = _abortEvidence;
            report.KneeCharacterJointResponseIsolationKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return
                "ARL Knee CharacterJoint response isolation | mode=" + Mode +
                " | arm=" + Arm +
                " | enabled=" + _enabled +
                " | jointPresent=" + (report != null && report.KneeCharacterJointPresent) +
                " | tetherSteps=" + _steps +
                " | forceApplications=" + _forceApplications +
                " | current(anchorErr/appliedA)=" + F(_currentAnchorError) + "/" + F(_currentAppliedAcceleration) +
                " | peak(anchorErr/appliedA)=" + F(_peakAnchorError) + "/" + F(_peakAppliedAcceleration) +
                " | key=" + GetKey(report);
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            if (!_enabled)
                primary = "A_KNEE_CHARACTERJOINT_BACKEND";
            else if (report != null && report.KneeCharacterJointPresent)
                primary = "B_INVALID_KNEE_CHARACTERJOINT_STILL_PRESENT";
            else if (_steps <= 0)
                primary = "B_NO_TETHER_EXPOSURE";
            else if (_invalidAnchorSteps > 0 || _peakAnchorError > TetherAnchorValidityMeters)
                primary = "B_INVALID_KNEE_TETHER_POSITIONAL_CLOSURE";
            else
                primary = "B_NO_KNEE_CHARACTERJOINT_TRANSLATION_TETHER_VALID";

            return
                "primary=" + primary +
                " | mode=" + Mode +
                " | arm=" + Arm +
                " | jointPresent=" + (report != null && report.KneeCharacterJointPresent) +
                " | tether(steps/force/clamped/invalidAnchor)=" + _steps + "/" + _forceApplications + "/" + _clampedSteps + "/" + _invalidAnchorSteps +
                " | peak(anchorErr/velErr/rawA/appliedA)=" + F(_peakAnchorError) + "/" + F(_peakVelocityError) + "/" + F(_peakRawAcceleration) + "/" + F(_peakAppliedAcceleration) +
                " | response(hipPeak/kneePeak)=" +
                F(report != null ? report.PeakHipRotationErrorDegrees : 0f) + "/" +
                F(report != null ? report.PeakKneeRotationErrorDegrees : 0f) +
                " | last=" + _lastEvidence +
                " | abort=" + _abortEvidence;
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
