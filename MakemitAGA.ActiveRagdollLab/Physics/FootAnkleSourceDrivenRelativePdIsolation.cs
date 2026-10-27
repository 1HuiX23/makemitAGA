using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.3.58 source-bone-driven relative Ankle controller, frozen since v0.3.69 with the v0.3.68 persistence-conditioned target-A path promoted.
    ///
    /// Historical A = kinematic source-following baseline.
    /// Historical B = Dynamic Foot + one bounded internal Lower<->Foot torque-pair controller whose target is the source
    ///     Lower->sole relative pose sampled from the real ankle/toe animation. v0.3.63 allows the positional backend
    ///     to be either the historical CharacterJoint baseline or the new ARL-owned translation-only tether.  The controller is explicitly relative: it does not command world-space Foot
    ///     orientation and does not reuse the rejected v0.3.54 direct-Lower correction route.
    ///
    /// Control law in source/physical Lower local space:
    ///   alphaRel = Kp * log(qSourceRel * inverse(qPhysicalRel))
    ///            + Kd * (sourceRelativeW - physicalRelativeW)
    ///            + bounded persistence-conditioned source-relative target-A feedforward (v0.3.69+ frozen production path)
    /// The desired relative angular acceleration is bounded, mapped through the two-body effective inertia, and
    /// realized as an equal/opposite physical torque pair (+tau Foot, -tau Lower) using ForceMode.Force.
    /// Gravity, Foot Collider/contact and GSP remain OFF. Angular tracking authority remains this controller only;
    /// v0.3.63's tether arm has no native angular constraint.
    /// </summary>
    internal sealed class FootAnkleSourceDrivenRelativePdIsolation
    {
        internal const string Mode = "SOURCE_BONE_DRIVEN_RELATIVE_ANKLE_PD_CAUSAL_AB_V0358";
        internal const string ArmA = "A_KINEMATIC_FROZEN_BASELINE";
        internal const string ArmB = "B_DYNAMIC_SOURCE_BONE_RELATIVE_ANKLE_PD";

        internal const float AnkleAnchorValidityMeters = 0.030f;
        internal const float FlexionLimitDegrees = 30f;
        internal const float AxialLimitDegrees = 25f;
        internal const float InversionLimitDegrees = 15f;

        // First physical-controller qualification values.  These are intentionally code-frozen for one causal run;
        // they are not Profile sweep knobs.  Kd is near the critical-damping scale for Kp while MaxA/MaxTorque keep
        // the first source-driven experiment well below the authority used by the major leg joints.
        internal const float Kp = 24f;
        internal const float Kd = 8f;
        internal const float MaxRelativeAngularAcceleration = 30f;
        internal const float MaxInternalTorqueNm = 0.35f;
        internal const float MaxFreshTargetAgeSeconds = 0.12f;

        // v0.3.69 promoted the v0.3.68 conditioned target-A lane into the frozen free-space Foot baseline; v0.3.70 keeps it unchanged.
        // It still reallocates only the existing MaxRelativeAngularAcceleration budget and composes BEFORE the unchanged
        // final MaxRelA30 and MaxTau0.35 clamps, so no new final authority is introduced.
        internal const string ProductionTargetAccelerationMode = "PRODUCTION_V0368_PERSISTENCE_CONDITIONED_TARGET_A_FF";
        internal const float TargetAccelerationFeedforwardGain = 1f;
        internal const float MaxTargetAccelerationFeedforward = 20f;

        private const float InertiaEpsilon = 0.000001f;

        private bool _armB;
        private bool _finite = true;
        private string _invalidReason;
        private bool _driveActive;
        private bool _currentKinematic = true;
        private float _currentAnchorError;
        private float _currentTrackingError;
        private Vector3 _currentPhysicalBindDeltaDegrees;
        private Vector3 _currentLimitRatio;
        private float _currentLinearSpeed;
        private float _currentAngularSpeed;
        private Vector3 _currentPoseErrorLocal;
        private Vector3 _currentTargetRelativeWLocal;
        private Vector3 _currentTargetRelativeALocal;
        private Vector3 _currentConditionedTargetRelativeALocal;
        private Vector3 _currentTargetAccelerationFeedforwardLocal;
        private Vector3 _currentBodyRelativeWLocal;
        private Vector3 _currentRawRelativeAccelerationLocal;
        private Vector3 _currentAppliedRelativeAccelerationLocal;
        private Vector3 _currentTorqueWorld;
        private bool _currentAccelerationClamp;
        private bool _currentTorqueClamp;
        private bool _currentTargetAccelerationFeedforwardClamp;
        private bool _currentStaleTargetW;

        private int _traceSamples;
        private int _traceDynamicSamples;
        private int _traceAppliedSamples;
        private int _traceInvalid;
        private int _traceAccelerationClamp;
        private int _traceTorqueClamp;
        private int _traceTargetAccelerationFeedforwardClamp;
        private int _traceStaleTargetW;
        private float _traceAnchorSum;
        private float _traceAnchorMax;
        private float _traceTrackingErrorSum;
        private float _traceTrackingErrorMax;
        private Vector3 _tracePhysicalAxisAbsMax;
        private Vector3 _traceLimitRatioAbsMax;
        private float _traceLinearSpeedSum;
        private float _traceLinearSpeedMax;
        private float _traceAngularSpeedSum;
        private float _traceAngularSpeedMax;
        private float _tracePoseErrorSum;
        private float _tracePoseErrorMax;
        private float _traceTargetAccelerationSum;
        private float _traceTargetAccelerationMax;
        private float _traceConditionedTargetAccelerationSum;
        private float _traceConditionedTargetAccelerationMax;
        private float _traceTargetAccelerationFeedforwardSum;
        private float _traceTargetAccelerationFeedforwardMax;
        private float _traceAppliedAccelerationSum;
        private float _traceAppliedAccelerationMax;
        private float _traceTorqueSum;
        private float _traceTorqueMax;

        internal string Arm => _armB ? ArmB : ArmA;
        internal bool IsProductionArm => _armB;
        internal bool SelectedDynamicArm => _armB;
        internal bool DynamicActive => _armB && _driveActive;
        internal string TargetAccelerationArm => ProductionTargetAccelerationMode;
        internal bool TargetAccelerationFeedforwardEnabled => true;

        // v0.4.4.1 read-only diagnostic taps. These expose the already-computed controller state to the
        // contact-induced axial-debt observer; they do not add or modify any physical authority.
        internal Vector3 CurrentPoseErrorLocal => _currentPoseErrorLocal;
        internal Vector3 CurrentTargetAccelerationFeedforwardLocal => _currentTargetAccelerationFeedforwardLocal;
        internal bool CurrentStaleTargetW => _currentStaleTargetW;
        internal Vector3 CurrentTargetRelativeWLocal => _currentTargetRelativeWLocal;
        internal Vector3 CurrentBodyRelativeWLocal => _currentBodyRelativeWLocal;
        internal Vector3 CurrentAppliedRelativeAccelerationLocal => _currentAppliedRelativeAccelerationLocal;
        internal Vector3 CurrentTorqueWorld => _currentTorqueWorld;
        internal bool CurrentAccelerationClamp => _currentAccelerationClamp;
        internal bool CurrentTorqueClamp => _currentTorqueClamp;

        internal void FreezeProductionArm()
        {
            _armB = true;
            ClearCurrentCommand();
        }

        internal void SetDriveActive(bool enabled)
        {
            _driveActive = enabled;
            // Foundation semantics are explicit: A is always kinematic; B is dynamic only while leg drive is active.
            // Keep status readback coherent even before the next FixedUpdate observer sample.
            _currentKinematic = !(_armB && enabled);
            if (!enabled)
                ClearCurrentCommand();
        }

        internal bool Step(
            bool traceEnabled,
            Rigidbody footBody,
            CharacterJoint ankleJoint,
            bool characterJointBackend,
            Rigidbody lowerBody,
            Vector3 footAnchorLocal,
            Vector3 lowerAnchorLocal,
            Quaternion bindSourceRelative,
            Quaternion sourceRelative,
            Vector3 sourceRelativeAngularVelocityLocal,
            Vector3 sourceRelativeAngularAccelerationLocal,
            Vector3 conditionedSourceRelativeAngularAccelerationLocal,
            float sourceSampleAgeSeconds,
            FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            ClearCurrentCommand();

            if (footBody == null || lowerBody == null ||
                !FiniteVector(footAnchorLocal) || !FiniteVector(lowerAnchorLocal) ||
                !FiniteQuaternion(bindSourceRelative) || !FiniteQuaternion(sourceRelative) ||
                !FiniteVector(sourceRelativeAngularVelocityLocal) || !FiniteVector(sourceRelativeAngularAccelerationLocal) ||
                !FiniteFloat(sourceSampleAgeSeconds))
                return Invalidate("MISSING_OR_NON_FINITE_ANKLE_DRIVE_INPUT", traceEnabled, report);

            Vector3 footAnchorWorld = footBody.transform.TransformPoint(footAnchorLocal);
            Vector3 lowerAnchorWorld = lowerBody.transform.TransformPoint(lowerAnchorLocal);
            Quaternion physicalRelative = Quaternion.Inverse(lowerBody.rotation) * footBody.rotation;
            Quaternion physicalBindDelta = Quaternion.Inverse(bindSourceRelative) * physicalRelative;
            Vector3 physicalBindDeltaDegrees = QuaternionToAngularVector(physicalBindDelta) * Mathf.Rad2Deg;
            float anchorError = Vector3.Distance(footAnchorWorld, lowerAnchorWorld);
            float trackingError = Quaternion.Angle(sourceRelative, physicalRelative);
            float linearSpeed = footBody.velocity.magnitude;
            float angularSpeed = footBody.angularVelocity.magnitude;
            Vector3 limitRatio = new Vector3(
                Mathf.Abs(physicalBindDeltaDegrees.x) / FlexionLimitDegrees,
                Mathf.Abs(physicalBindDeltaDegrees.y) / AxialLimitDegrees,
                Mathf.Abs(physicalBindDeltaDegrees.z) / InversionLimitDegrees);

            if (!FiniteVector(footAnchorWorld) || !FiniteVector(lowerAnchorWorld) ||
                !FiniteQuaternion(physicalRelative) || !FiniteQuaternion(physicalBindDelta) ||
                !FiniteVector(physicalBindDeltaDegrees) || !FiniteVector(limitRatio) ||
                !FiniteFloat(anchorError) || !FiniteFloat(trackingError) ||
                !FiniteFloat(linearSpeed) || !FiniteFloat(angularSpeed))
                return Invalidate("NON_FINITE_ANKLE_DRIVE_STATE", traceEnabled, report);

            bool expectedDynamic = DynamicActive;
            bool jointExists = ankleJoint != null;
            bool connected = jointExists && ankleJoint.connectedBody == lowerBody;
            bool kinematic = footBody.isKinematic;
            bool dynamicBackendContract = characterJointBackend
                ? (jointExists && connected && JointMappingMatches(ankleJoint))
                : !jointExists;
            bool structuralContract = !footBody.useGravity &&
                                      (_armB ? (dynamicBackendContract && (expectedDynamic ? !kinematic : kinematic))
                                             : (!jointExists && kinematic));
            if (!structuralContract)
                return Invalidate("ANKLE_DRIVE_STRUCTURAL_CONTRACT_MISMATCH", traceEnabled, report);

            if (expectedDynamic && anchorError >= AnkleAnchorValidityMeters)
                return Invalidate("ANKLE_ANCHOR_OVER_30MM", traceEnabled, report);

            _currentKinematic = kinematic;
            _currentAnchorError = anchorError;
            _currentTrackingError = trackingError;
            _currentPhysicalBindDeltaDegrees = physicalBindDeltaDegrees;
            _currentLimitRatio = limitRatio;
            _currentLinearSpeed = linearSpeed;
            _currentAngularSpeed = angularSpeed;

            if (expectedDynamic)
            {
                // qError rotates current physical relative pose toward the source-bone target in Lower local space.
                Quaternion qError = sourceRelative * Quaternion.Inverse(physicalRelative);
                Vector3 poseErrorLocal = QuaternionToAngularVector(qError);
                Vector3 bodyRelativeWWorld = footBody.angularVelocity - lowerBody.angularVelocity;
                Vector3 bodyRelativeWLocal = Quaternion.Inverse(lowerBody.rotation) * bodyRelativeWWorld;

                bool staleTargetW = sourceSampleAgeSeconds > MaxFreshTargetAgeSeconds;
                Vector3 targetRelativeWLocal = staleTargetW ? Vector3.zero : sourceRelativeAngularVelocityLocal;
                Vector3 targetRelativeALocal = staleTargetW ? Vector3.zero : sourceRelativeAngularAccelerationLocal;
                Vector3 conditionedTargetRelativeALocal = staleTargetW ? Vector3.zero : conditionedSourceRelativeAngularAccelerationLocal;
                Vector3 rawTargetAccelerationFeedforwardLocal = conditionedTargetRelativeALocal * TargetAccelerationFeedforwardGain;
                bool targetAccelerationFeedforwardClamp =
                    rawTargetAccelerationFeedforwardLocal.sqrMagnitude > MaxTargetAccelerationFeedforward * MaxTargetAccelerationFeedforward * 1.000001f;
                // Diagnostic tolerance only: a coherent 20 rad/s² derivative may round a few ulps high.
                // The actual ClampMagnitude below remains exact and is independent of this count.
                Vector3 targetAccelerationFeedforwardLocal =
                    Vector3.ClampMagnitude(rawTargetAccelerationFeedforwardLocal, MaxTargetAccelerationFeedforward);
                Vector3 feedbackAccelerationLocal = poseErrorLocal * Kp + (targetRelativeWLocal - bodyRelativeWLocal) * Kd;
                Vector3 rawAccelerationLocal = feedbackAccelerationLocal + targetAccelerationFeedforwardLocal;
                Vector3 appliedAccelerationLocal = Vector3.ClampMagnitude(rawAccelerationLocal, MaxRelativeAngularAcceleration);
                bool accelerationClamp = rawAccelerationLocal.sqrMagnitude > MaxRelativeAngularAcceleration * MaxRelativeAngularAcceleration + 0.000001f;
                Vector3 desiredRelativeAccelerationWorld = lowerBody.rotation * appliedAccelerationLocal;

                if (!FiniteQuaternion(qError) || !FiniteVector(poseErrorLocal) ||
                    !FiniteVector(bodyRelativeWWorld) || !FiniteVector(bodyRelativeWLocal) ||
                    !FiniteVector(targetRelativeWLocal) || !FiniteVector(targetRelativeALocal) ||
                    !FiniteVector(conditionedTargetRelativeALocal) || !FiniteVector(rawTargetAccelerationFeedforwardLocal) || !FiniteVector(targetAccelerationFeedforwardLocal) ||
                    !FiniteVector(feedbackAccelerationLocal) || !FiniteVector(rawAccelerationLocal) ||
                    !FiniteVector(appliedAccelerationLocal) || !FiniteVector(desiredRelativeAccelerationWorld))
                    return Invalidate("NON_FINITE_ANKLE_PD_COMMAND", traceEnabled, report);

                Vector3 torqueWorld;
                if (!TrySolveEqualOppositeTorquePair(footBody, lowerBody, desiredRelativeAccelerationWorld, out torqueWorld))
                    return Invalidate("ANKLE_PD_EFFECTIVE_INERTIA_SOLVE_FAILED", traceEnabled, report);

                bool torqueClamp = torqueWorld.sqrMagnitude > MaxInternalTorqueNm * MaxInternalTorqueNm + 0.000001f;
                torqueWorld = Vector3.ClampMagnitude(torqueWorld, MaxInternalTorqueNm);
                // v0.4.40 yaw removal was insufficient; retain the unfiltered equal/opposite production pair.
                if (!FiniteVector(torqueWorld))
                    return Invalidate("NON_FINITE_ANKLE_PD_TORQUE", traceEnabled, report);

                Vector3 predictedRelativeAccelerationWorld;
                if (!TryPredictEqualOppositeRelativeAcceleration(footBody, lowerBody, torqueWorld, out predictedRelativeAccelerationWorld))
                    return Invalidate("ANKLE_PD_REALIZED_ACCELERATION_PREDICTION_FAILED", traceEnabled, report);
                Vector3 predictedRelativeAccelerationLocal = Quaternion.Inverse(lowerBody.rotation) * predictedRelativeAccelerationWorld;
                if (!FiniteVector(predictedRelativeAccelerationLocal))
                    return Invalidate("NON_FINITE_ANKLE_PD_REALIZED_ACCELERATION", traceEnabled, report);

                // True internal ankle actuator: equal and opposite physical torque, no world-space direct-Lower correction.
                footBody.AddTorque(torqueWorld, ForceMode.Force);
                lowerBody.AddTorque(-torqueWorld, ForceMode.Force);

                _currentPoseErrorLocal = poseErrorLocal;
                _currentTargetRelativeWLocal = targetRelativeWLocal;
                _currentTargetRelativeALocal = targetRelativeALocal;
                _currentConditionedTargetRelativeALocal = conditionedTargetRelativeALocal;
                _currentTargetAccelerationFeedforwardLocal = targetAccelerationFeedforwardLocal;
                _currentBodyRelativeWLocal = bodyRelativeWLocal;
                _currentRawRelativeAccelerationLocal = rawAccelerationLocal;
                // Report the predicted acceleration after the physical torque clamp, not the unclamped desired command.
                _currentAppliedRelativeAccelerationLocal = predictedRelativeAccelerationLocal;
                _currentTorqueWorld = torqueWorld;
                _currentAccelerationClamp = accelerationClamp;
                _currentTorqueClamp = torqueClamp;
                _currentTargetAccelerationFeedforwardClamp = targetAccelerationFeedforwardClamp;
                _currentStaleTargetW = staleTargetW;
            }

            if (traceEnabled)
            {
                _traceSamples++;
                if (expectedDynamic) _traceDynamicSamples++;
                _traceAnchorSum += anchorError;
                _traceAnchorMax = Mathf.Max(_traceAnchorMax, anchorError);
                _traceTrackingErrorSum += trackingError;
                _traceTrackingErrorMax = Mathf.Max(_traceTrackingErrorMax, trackingError);
                _tracePhysicalAxisAbsMax = ComponentMax(_tracePhysicalAxisAbsMax, ComponentAbs(physicalBindDeltaDegrees));
                _traceLimitRatioAbsMax = ComponentMax(_traceLimitRatioAbsMax, limitRatio);
                _traceLinearSpeedSum += linearSpeed;
                _traceLinearSpeedMax = Mathf.Max(_traceLinearSpeedMax, linearSpeed);
                _traceAngularSpeedSum += angularSpeed;
                _traceAngularSpeedMax = Mathf.Max(_traceAngularSpeedMax, angularSpeed);

                if (expectedDynamic)
                {
                    _traceAppliedSamples++;
                    if (_currentAccelerationClamp) _traceAccelerationClamp++;
                    if (_currentTorqueClamp) _traceTorqueClamp++;
                    if (_currentTargetAccelerationFeedforwardClamp) _traceTargetAccelerationFeedforwardClamp++;
                    if (_currentStaleTargetW) _traceStaleTargetW++;
                    float poseError = _currentPoseErrorLocal.magnitude * Mathf.Rad2Deg;
                    float targetAcceleration = _currentTargetRelativeALocal.magnitude;
                    float conditionedTargetAcceleration = _currentConditionedTargetRelativeALocal.magnitude;
                    float targetAccelerationFeedforward = _currentTargetAccelerationFeedforwardLocal.magnitude;
                    float appliedAcceleration = _currentAppliedRelativeAccelerationLocal.magnitude;
                    float torque = _currentTorqueWorld.magnitude;
                    _tracePoseErrorSum += poseError;
                    _tracePoseErrorMax = Mathf.Max(_tracePoseErrorMax, poseError);
                    _traceTargetAccelerationSum += targetAcceleration;
                    _traceTargetAccelerationMax = Mathf.Max(_traceTargetAccelerationMax, targetAcceleration);
                    _traceConditionedTargetAccelerationSum += conditionedTargetAcceleration;
                    _traceConditionedTargetAccelerationMax = Mathf.Max(_traceConditionedTargetAccelerationMax, conditionedTargetAcceleration);
                    _traceTargetAccelerationFeedforwardSum += targetAccelerationFeedforward;
                    _traceTargetAccelerationFeedforwardMax = Mathf.Max(_traceTargetAccelerationFeedforwardMax, targetAccelerationFeedforward);
                    _traceAppliedAccelerationSum += appliedAcceleration;
                    _traceAppliedAccelerationMax = Mathf.Max(_traceAppliedAccelerationMax, appliedAcceleration);
                    _traceTorqueSum += torque;
                    _traceTorqueMax = Mathf.Max(_traceTorqueMax, torque);
                }
            }

            CopyToReport(report);
            return true;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceSamples = 0;
            _traceDynamicSamples = 0;
            _traceAppliedSamples = 0;
            _traceInvalid = 0;
            _traceAccelerationClamp = 0;
            _traceTorqueClamp = 0;
            _traceTargetAccelerationFeedforwardClamp = 0;
            _traceStaleTargetW = 0;
            _traceAnchorSum = 0f;
            _traceAnchorMax = 0f;
            _traceTrackingErrorSum = 0f;
            _traceTrackingErrorMax = 0f;
            _tracePhysicalAxisAbsMax = Vector3.zero;
            _traceLimitRatioAbsMax = Vector3.zero;
            _traceLinearSpeedSum = 0f;
            _traceLinearSpeedMax = 0f;
            _traceAngularSpeedSum = 0f;
            _traceAngularSpeedMax = 0f;
            _tracePoseErrorSum = 0f;
            _tracePoseErrorMax = 0f;
            _traceTargetAccelerationSum = 0f;
            _traceTargetAccelerationMax = 0f;
            _traceConditionedTargetAccelerationSum = 0f;
            _traceConditionedTargetAccelerationMax = 0f;
            _traceTargetAccelerationFeedforwardSum = 0f;
            _traceTargetAccelerationFeedforwardMax = 0f;
            _traceAppliedAccelerationSum = 0f;
            _traceAppliedAccelerationMax = 0f;
            _traceTorqueSum = 0f;
            _traceTorqueMax = 0f;
            ClearCurrentCommand();
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            report.AnkleDriveMode = Mode;
            report.AnkleDriveArm = Arm;
            report.AnkleDriveFinite = _finite;
            report.AnkleDriveInvalidReason = _invalidReason;
            report.AnkleDriveCurrentSummary =
                "arm=" + Arm +
                "/drive=" + _driveActive +
                "/dynamic=" + DynamicActive +
                "/joint=" + (report.AnkleJointExists ? "PRESENT" : "ABSENT") +
                "/kinematic=" + _currentKinematic +
                "/anchorMm=" + F(_currentAnchorError * 1000f) +
                "/trackDeg=" + F(_currentTrackingError) +
                "/physDeg(F-A-I)=" + V(_currentPhysicalBindDeltaDegrees) +
                "/referenceRatio=" + V(_currentLimitRatio) +
                "/poseErrDeg=" + F(_currentPoseErrorLocal.magnitude * Mathf.Rad2Deg) +
                "/targetW=" + V(_currentTargetRelativeWLocal) +
                "/targetA=" + V(_currentTargetRelativeALocal) +
                "/conditionedTargetA=" + V(_currentConditionedTargetRelativeALocal) +
                "/targetAFFMode=" + TargetAccelerationArm +
                "/targetAFF=" + V(_currentTargetAccelerationFeedforwardLocal) +
                "/bodyRelW=" + V(_currentBodyRelativeWLocal) +
                "/rawA=" + V(_currentRawRelativeAccelerationLocal) +
                "/appA=" + V(_currentAppliedRelativeAccelerationLocal) +
                "/tau=" + V(_currentTorqueWorld) +
                "/clampA-T-FF=" + _currentAccelerationClamp + "-" + _currentTorqueClamp + "-" + _currentTargetAccelerationFeedforwardClamp +
                "/staleW=" + _currentStaleTargetW +
                "/v=" + F(_currentLinearSpeed) +
                "/w=" + F(_currentAngularSpeed);

            float denom = Mathf.Max(1, _traceSamples);
            float appliedDenom = Mathf.Max(1, _traceAppliedSamples);
            report.AnkleDriveTraceSummary =
                "n=" + _traceSamples +
                "/dynamic=" + _traceDynamicSamples +
                "/applied=" + _traceAppliedSamples +
                "/invalid=" + _traceInvalid +
                "/anchorMeanMaxMm=" + F((_traceAnchorSum / denom) * 1000f) + "-" + F(_traceAnchorMax * 1000f) +
                "/trackMeanMaxDeg=" + F(_traceTrackingErrorSum / denom) + "-" + F(_traceTrackingErrorMax) +
                "/poseErrMeanMaxDeg=" + F(_tracePoseErrorSum / appliedDenom) + "-" + F(_tracePoseErrorMax) +
                "/targetAMeanMax=" + F(_traceTargetAccelerationSum / appliedDenom) + "-" + F(_traceTargetAccelerationMax) +
                "/conditionedTargetAMeanMax=" + F(_traceConditionedTargetAccelerationSum / appliedDenom) + "-" + F(_traceConditionedTargetAccelerationMax) +
                "/targetAFFMeanMax=" + F(_traceTargetAccelerationFeedforwardSum / appliedDenom) + "-" + F(_traceTargetAccelerationFeedforwardMax) +
                "/targetAFFMode=" + TargetAccelerationArm +
                "/physAxisAbsMaxDeg=" + V(_tracePhysicalAxisAbsMax) +
                "/referenceRatioAbsMax=" + V(_traceLimitRatioAbsMax) +
                "/appAMeanMax=" + F(_traceAppliedAccelerationSum / appliedDenom) + "-" + F(_traceAppliedAccelerationMax) +
                "/tauMeanMax=" + F(_traceTorqueSum / appliedDenom) + "-" + F(_traceTorqueMax) +
                "/clampA-T-FF-staleW=" + _traceAccelerationClamp + "-" + _traceTorqueClamp + "-" + _traceTargetAccelerationFeedforwardClamp + "-" + _traceStaleTargetW +
                "/vMeanMax=" + F(_traceLinearSpeedSum / denom) + "-" + F(_traceLinearSpeedMax) +
                "/wMeanMax=" + F(_traceAngularSpeedSum / denom) + "-" + F(_traceAngularSpeedMax) +
                "/constants(Kp-Kd-MaxA-MaxTau-MaxAge)=" + F(Kp) + "-" + F(Kd) + "-" + F(MaxRelativeAngularAcceleration) + "-" + F(MaxInternalTorqueNm) + "-" + F(MaxFreshTargetAgeSeconds) +
                "/targetAFF(gain-bound)=" + F(TargetAccelerationFeedforwardGain) + "-" + F(MaxTargetAccelerationFeedforward) +
                "/referenceEnvelopeDeg(F-A-I)=" + F(FlexionLimitDegrees) + "-" + F(AxialLimitDegrees) + "-" + F(InversionLimitDegrees) +
                "/anchorGateMm=" + F(AnkleAnchorValidityMeters * 1000f);
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/arm=" + Arm +
                   "/targetAFFMode=" + TargetAccelerationArm +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.AnkleDriveCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.AnkleDriveTraceSummary : "<none>");
        }

        private bool JointMappingMatches(CharacterJoint joint)
        {
            if (joint == null) return false;
            Vector3 axis = joint.axis;
            Vector3 swingAxis = joint.swingAxis;
            if (!FiniteVector(axis) || !FiniteVector(swingAxis) || axis.sqrMagnitude < 0.5f || swingAxis.sqrMagnitude < 0.5f)
                return false;
            axis.Normalize();
            swingAxis.Normalize();
            return Vector3.Dot(axis, Vector3.right) >= 0.999f &&
                   Vector3.Dot(swingAxis, Vector3.up) >= 0.999f &&
                   Mathf.Abs(joint.lowTwistLimit.limit + FlexionLimitDegrees) <= 0.05f &&
                   Mathf.Abs(joint.highTwistLimit.limit - FlexionLimitDegrees) <= 0.05f &&
                   Mathf.Abs(joint.swing1Limit.limit - AxialLimitDegrees) <= 0.05f &&
                   Mathf.Abs(joint.swing2Limit.limit - InversionLimitDegrees) <= 0.05f;
        }

        private bool Invalidate(string reason, bool traceEnabled, FootAnkleFoundationReport report)
        {
            _finite = false;
            _invalidReason = reason;
            if (traceEnabled) _traceInvalid++;
            CopyToReport(report);
            return false;
        }

        private void ClearCurrentCommand()
        {
            _currentPoseErrorLocal = Vector3.zero;
            _currentTargetRelativeWLocal = Vector3.zero;
            _currentTargetRelativeALocal = Vector3.zero;
            _currentConditionedTargetRelativeALocal = Vector3.zero;
            _currentTargetAccelerationFeedforwardLocal = Vector3.zero;
            _currentBodyRelativeWLocal = Vector3.zero;
            _currentRawRelativeAccelerationLocal = Vector3.zero;
            _currentAppliedRelativeAccelerationLocal = Vector3.zero;
            _currentTorqueWorld = Vector3.zero;
            _currentAccelerationClamp = false;
            _currentTorqueClamp = false;
            _currentTargetAccelerationFeedforwardClamp = false;
            _currentStaleTargetW = false;
        }

        private static bool TrySolveEqualOppositeTorquePair(
            Rigidbody footBody,
            Rigidbody lowerBody,
            Vector3 desiredRelativeAccelerationWorld,
            out Vector3 torqueWorld)
        {
            torqueWorld = Vector3.zero;
            if (!ValidInertiaBody(footBody) || !ValidInertiaBody(lowerBody) || !FiniteVector(desiredRelativeAccelerationWorld))
                return false;

            Quaternion footPrincipal = Normalize(footBody.rotation * footBody.inertiaTensorRotation);
            Quaternion lowerPrincipal = Normalize(lowerBody.rotation * lowerBody.inertiaTensorRotation);

            Vector3 footGyro = GyroTorque(footPrincipal, footBody.inertiaTensor, footBody.angularVelocity);
            Vector3 lowerGyro = GyroTorque(lowerPrincipal, lowerBody.inertiaTensor, lowerBody.angularVelocity);
            if (!FiniteVector(footGyro) || !FiniteVector(lowerGyro))
                return false;

            Vector3 c0 = ApplyEffectiveInverseInertia(footPrincipal, footBody.inertiaTensor, lowerPrincipal, lowerBody.inertiaTensor, Vector3.right);
            Vector3 c1 = ApplyEffectiveInverseInertia(footPrincipal, footBody.inertiaTensor, lowerPrincipal, lowerBody.inertiaTensor, Vector3.up);
            Vector3 c2 = ApplyEffectiveInverseInertia(footPrincipal, footBody.inertiaTensor, lowerPrincipal, lowerBody.inertiaTensor, Vector3.forward);
            if (!FiniteVector(c0) || !FiniteVector(c1) || !FiniteVector(c2))
                return false;

            Matrix4x4 effectiveInverse = Matrix4x4.identity;
            effectiveInverse.SetColumn(0, new Vector4(c0.x, c0.y, c0.z, 0f));
            effectiveInverse.SetColumn(1, new Vector4(c1.x, c1.y, c1.z, 0f));
            effectiveInverse.SetColumn(2, new Vector4(c2.x, c2.y, c2.z, 0f));

            float determinant = effectiveInverse.determinant;
            if (!FiniteFloat(determinant) || Mathf.Abs(determinant) <= 0.000000001f)
                return false;

            Vector3 gyroBias = InverseWorldInertia(footPrincipal, footBody.inertiaTensor, footGyro) -
                               InverseWorldInertia(lowerPrincipal, lowerBody.inertiaTensor, lowerGyro);
            Vector3 rhs = desiredRelativeAccelerationWorld + gyroBias;
            Matrix4x4 effectiveInertia = effectiveInverse.inverse;
            torqueWorld = effectiveInertia.MultiplyVector(rhs);
            return FiniteVector(torqueWorld);
        }

        private static bool TryPredictEqualOppositeRelativeAcceleration(
            Rigidbody footBody,
            Rigidbody lowerBody,
            Vector3 torqueWorld,
            out Vector3 relativeAccelerationWorld)
        {
            relativeAccelerationWorld = Vector3.zero;
            if (!ValidInertiaBody(footBody) || !ValidInertiaBody(lowerBody) || !FiniteVector(torqueWorld))
                return false;

            Quaternion footPrincipal = Normalize(footBody.rotation * footBody.inertiaTensorRotation);
            Quaternion lowerPrincipal = Normalize(lowerBody.rotation * lowerBody.inertiaTensorRotation);
            Vector3 footGyro = GyroTorque(footPrincipal, footBody.inertiaTensor, footBody.angularVelocity);
            Vector3 lowerGyro = GyroTorque(lowerPrincipal, lowerBody.inertiaTensor, lowerBody.angularVelocity);
            if (!FiniteVector(footGyro) || !FiniteVector(lowerGyro))
                return false;

            Vector3 footAlpha = InverseWorldInertia(footPrincipal, footBody.inertiaTensor, torqueWorld - footGyro);
            Vector3 lowerAlpha = InverseWorldInertia(lowerPrincipal, lowerBody.inertiaTensor, -torqueWorld - lowerGyro);
            relativeAccelerationWorld = footAlpha - lowerAlpha;
            return FiniteVector(relativeAccelerationWorld);
        }

        private static Vector3 ApplyEffectiveInverseInertia(
            Quaternion footPrincipal,
            Vector3 footInertia,
            Quaternion lowerPrincipal,
            Vector3 lowerInertia,
            Vector3 torqueWorld)
        {
            return InverseWorldInertia(footPrincipal, footInertia, torqueWorld) +
                   InverseWorldInertia(lowerPrincipal, lowerInertia, torqueWorld);
        }

        private static Vector3 GyroTorque(Quaternion principalWorld, Vector3 inertia, Vector3 angularVelocityWorld)
        {
            Vector3 angularMomentum = MultiplyWorldInertia(principalWorld, inertia, angularVelocityWorld);
            return Vector3.Cross(angularVelocityWorld, angularMomentum);
        }

        private static Vector3 MultiplyWorldInertia(Quaternion principalWorld, Vector3 inertia, Vector3 vectorWorld)
        {
            Vector3 local = Quaternion.Inverse(principalWorld) * vectorWorld;
            Vector3 mapped = new Vector3(local.x * inertia.x, local.y * inertia.y, local.z * inertia.z);
            return principalWorld * mapped;
        }

        private static Vector3 InverseWorldInertia(Quaternion principalWorld, Vector3 inertia, Vector3 torqueWorld)
        {
            Vector3 local = Quaternion.Inverse(principalWorld) * torqueWorld;
            Vector3 mapped = new Vector3(local.x / inertia.x, local.y / inertia.y, local.z / inertia.z);
            return principalWorld * mapped;
        }

        private static bool ValidInertiaBody(Rigidbody body)
        {
            return body != null && FiniteQuaternion(body.rotation) && FiniteVector(body.angularVelocity) &&
                   FiniteVector(body.inertiaTensor) && FiniteQuaternion(body.inertiaTensorRotation) &&
                   body.inertiaTensor.x > InertiaEpsilon && body.inertiaTensor.y > InertiaEpsilon && body.inertiaTensor.z > InertiaEpsilon;
        }

        private static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag <= 0.000001f || !FiniteFloat(mag)) return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        private static Vector3 QuaternionToAngularVector(Quaternion q)
        {
            if (q.w < 0f)
            {
                q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
            }
            float w = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            if (angle > Mathf.PI) angle -= 2f * Mathf.PI;
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (sinHalf <= 0.0001f || Mathf.Abs(angle) <= 0.000001f)
                return Vector3.zero;
            return new Vector3(q.x / sinHalf, q.y / sinHalf, q.z / sinHalf) * angle;
        }

        private static Vector3 ComponentAbs(Vector3 value)
        {
            return new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));
        }

        private static Vector3 ComponentMax(Vector3 a, Vector3 b)
        {
            return new Vector3(Mathf.Max(a.x, b.x), Mathf.Max(a.y, b.y), Mathf.Max(a.z, b.z));
        }

        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
        private static bool FiniteFloat(float value) => !(float.IsNaN(value) || float.IsInfinity(value));
        private static bool FiniteVector(Vector3 value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z);
        private static bool FiniteQuaternion(Quaternion value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z) && FiniteFloat(value.w);
    }
}
