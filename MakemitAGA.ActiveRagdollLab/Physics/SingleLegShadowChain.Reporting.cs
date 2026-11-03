using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Numeric report projection and trace formatting. Some readbacks are readiness inputs; preserve all original call sites even with trace disabled.
    internal sealed partial class SingleLegShadowChain
    {

        private void CopyRigidbodyMassPropertiesToReport(bool refreshText = false)
        {
            _report.RigidbodyInertiaFoundationMode = SingleLegRigidbodyMassProperties.FoundationMode;
            _report.RigidbodyInertiaAuditCompleted = _rigidbodyMassProperties.AuditCompleted;
            _report.RigidbodyInertiaBuildFoundationValidated = _rigidbodyMassProperties.BuildFoundationValidated;

            // Mass properties are immutable during active drive in v0.2.17. Avoid rebuilding the long summary/detail
            // string every FixedUpdate; refresh only after Build or an explicit read-only status/audit request.
            if (refreshText || string.IsNullOrEmpty(_report.RigidbodyInertiaAuditSummary))
            {
                _report.RigidbodyInertiaAuditSummary = _rigidbodyMassProperties.GetConsoleSummary();
                _report.RigidbodyInertiaAuditDetail = _rigidbodyMassProperties.GetDetailedReport();
            }
        }

        private void CopyKneeConstraintBackendToReport()
        {
            _report.KneeConstraintBackendMode = _kneeCharacterJointResponseIsolation.Enabled
                ? "NO_KNEE_CHARACTERJOINT_PLUS_TRANSLATION_ONLY_TETHER_PLUS_SOFTWARE_PD"
                : "WIDE_CHARACTER_JOINT_PLUS_SOFTWARE_OFFAXIS_PD";
            _report.KneeSoftOffAxisControlEnabled = true;
            _report.KneeSoftOffAxisAngularSpring = _profile.LegKneeSoftOffAxisAngularSpring;
            _report.KneeSoftOffAxisAngularDamper = _profile.LegKneeSoftOffAxisAngularDamper;
            _report.KneeSoftOffAxisMaxAngularAcceleration = _profile.LegKneeSoftOffAxisMaxAngularAcceleration;
            _report.KneeSoftOffAxisSourceTurnMaxAngularAcceleration = _profile.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration;
            _report.KneeSoftOffAxisRecoveryMaxAngularAcceleration = _profile.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration;
            if (_kneeJoint != null)
            {
                _report.KneeLowTwistLimitReadbackDegrees = _kneeJoint.lowTwistLimit.limit;
                _report.KneeHighTwistLimitReadbackDegrees = _kneeJoint.highTwistLimit.limit;
                _report.KneeSwing1LimitReadbackDegrees = _kneeJoint.swing1Limit.limit;
                _report.KneeSwing2LimitReadbackDegrees = _kneeJoint.swing2Limit.limit;
            }
            else
            {
                _report.KneeLowTwistLimitReadbackDegrees = 0f;
                _report.KneeHighTwistLimitReadbackDegrees = 0f;
                _report.KneeSwing1LimitReadbackDegrees = 0f;
                _report.KneeSwing2LimitReadbackDegrees = 0f;
            }
        }

        private void CopyLegAngularAccelerationFeedforwardToReport()
        {
            _report.LegAngularAccelerationFeedforwardEnabled = _legAngularAccelerationFeedforward.Enabled;
            _report.LegAngularAccelerationFeedforwardMode = SingleLegAngularAccelerationFeedforward.ModeName;
            _report.LegAngularAccelerationFeedforwardGateMode = SingleLegAngularAccelerationFeedforward.GateModeName;
            _report.LegAngularAccelerationFeedforwardGain = _profile.LegAngularAccelerationFeedforwardGain;
            _report.LegAngularAccelerationFeedforwardMaxAcceleration = _profile.LegAngularAccelerationFeedforwardMaxAcceleration;
            _report.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds = _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds;
            _report.LegAngularAccelerationFeedforwardActiveThisStep = _legAngularAccelerationFeedforward.ActiveThisStep;
            _report.LegAngularAccelerationFeedforwardSuppressionReason = _legAngularAccelerationFeedforward.SuppressionReason;
            _report.HipAngularAccelerationFeedforwardActiveThisStep = _legAngularAccelerationFeedforward.HipActiveThisStep;
            _report.KneeAngularAccelerationFeedforwardActiveThisStep = _legAngularAccelerationFeedforward.KneeActiveThisStep;
            _report.KneeAngularAccelerationFeedforwardSuppressionReason = _legAngularAccelerationFeedforward.KneeSuppressionReason;
            _report.TraceHipFeedforwardActiveSteps = _legAngularAccelerationFeedforward.ActiveFixedSteps;
            _report.TraceKneeFeedforwardActiveSteps = _legAngularAccelerationFeedforward.KneeActiveFixedSteps;
            _report.TraceKneeHipGoalEligibleSteps = _legAngularAccelerationFeedforward.KneeGoalEligibleSteps;
            _report.TraceKneeHipGoalNonzeroSelectedSteps = _legAngularAccelerationFeedforward.KneeGoalNonzeroSelectedSteps;
            _report.HipAngularAccelerationFeedforwardAppliedMagnitude = _legAngularAccelerationFeedforward.HipAppliedWorld.magnitude;
            _report.KneeAngularAccelerationFeedforwardAppliedSigned = _legAngularAccelerationFeedforward.KneeAppliedSigned;
            _report.HipAngularAccelerationFeedforwardClamped = _legAngularAccelerationFeedforward.HipClamped;
            _report.KneeAngularAccelerationFeedforwardClamped = _legAngularAccelerationFeedforward.KneeClamped;
            _report.HipAngularAccelerationFeedforwardClampCount = _legAngularAccelerationFeedforward.HipClampCount;
            _report.KneeAngularAccelerationFeedforwardClampCount = _legAngularAccelerationFeedforward.KneeClampCount;
            _report.TraceHipAngularAccelerationFeedforwardAppliedMean = _legAngularAccelerationFeedforward.HipAppliedMean;
            _report.TraceHipAngularAccelerationFeedforwardAppliedMax = _legAngularAccelerationFeedforward.HipAppliedMax;
            _report.TraceKneeAngularAccelerationFeedforwardAppliedAbsMean = _legAngularAccelerationFeedforward.KneeAppliedAbsMean;
            _report.TraceKneeAngularAccelerationFeedforwardAppliedAbsMax = _legAngularAccelerationFeedforward.KneeAppliedAbsMax;
        }

        private void UpdateReport()
        {
            if (!IsBuilt)
                return;

            _report.Built = true;
            _report.DriveEnabled = _driveEnabled;
            _report.GravityEnabled = _gravityEnabled;
            _report.CollidersEnabled = _collidersEnabled;
            UpdateJointPresence();

            Quaternion currentHip = Quaternion.Inverse(_pelvisBody.rotation) * _upperBody.rotation;
            Quaternion currentKnee = Quaternion.Inverse(_upperBody.rotation) * _lowerBody.rotation;
            _report.HipRotationErrorDegrees = Quaternion.Angle(_sampledTargetHipRelative, currentHip);
            _report.KneeRotationErrorDegrees = Quaternion.Angle(_sampledTargetKneeRelative, currentKnee);
            _report.HipControlRotationErrorDegrees = Quaternion.Angle(_controlTargetHipRelative, currentHip);

            // v0.2.25: full quaternion error 与 PCA flexion axis / software off-axis error 分开记录。
            Quaternion kneeErrorRelative = _sampledTargetKneeRelative * Quaternion.Inverse(currentKnee);
            Vector3 kneeErrorLocal = QuaternionToAngularVector(kneeErrorRelative);
            Vector3 kneeErrorWorld = _upperBody.rotation * kneeErrorLocal;
            _report.KneeAxisMode = _kneeAxisMode;
            _report.KneeFlexionAxisLowerLocal = _kneeFlexionAxisLowerLocal.normalized;
            _report.KneeFlexionAxisWorld = _lowerBody.transform.TransformDirection(_kneeFlexionAxisLowerLocal).normalized;
            _kneeAxisCalibrator.CopyToReport(_report);
            CopyKneeConstraintBackendToReport();
            CopyLegAngularAccelerationFeedforwardToReport();
            _hipSourceReferenceShockAbsorber.CopyToReport(_report);
            _kneeAnchorAccelerationFeedforward.CopyToReport(_report);
            _kneeFlexionAllocation.CopyToReport(_report);
            _report.SingleLegFreezeCertificationMode = SingleLegFreezeReadinessCertification.Mode;
            _report.SingleLegFreezeCertificationPolicy = SingleLegFreezeReadinessCertification.Policy;
            _hipAngularActuator.CopyToReport(_report, _upperBody);
            _legAngularDragContract.CopyToReport(_report, _upperBody, _lowerBody, _profile.AngularDrag);
            CopyRigidbodyMassPropertiesToReport();

            Vector3 currentFlexionAxisWorld = _report.KneeFlexionAxisWorld;
            float kneeAxisErrorRad = Vector3.Dot(kneeErrorWorld, currentFlexionAxisWorld);
            Vector3 kneeOffAxisError = kneeErrorWorld - currentFlexionAxisWorld * kneeAxisErrorRad;
            _report.KneeAxisErrorDegrees = Mathf.Abs(kneeAxisErrorRad) * Mathf.Rad2Deg;
            _report.KneeOffAxisErrorDegrees = kneeOffAxisError.magnitude * Mathf.Rad2Deg;

            Quaternion kneeControlErrorRelative = _controlTargetKneeRelative * Quaternion.Inverse(currentKnee);
            Vector3 kneeControlErrorWorld = _upperBody.rotation * QuaternionToAngularVector(kneeControlErrorRelative);
            _report.KneeControlAxisErrorDegrees = Mathf.Abs(Vector3.Dot(kneeControlErrorWorld, currentFlexionAxisWorld)) * Mathf.Rad2Deg;

            // v0.2.17 cleanup: the rejected relative-pose predictor is retired, so aligned-source metrics
            // deliberately collapse to the raw held-source metrics. Keeping the fields preserves report compatibility
            // without carrying the old experimental control/diagnostic branch back into the active code path.

            Vector3 targetRelativeWWorld = _sampledUpperRotation * _sampledTargetKneeRelativeAngularVelocityLocal;
            Vector3 targetFlexionAxisWorld = _sampledLowerRotation * _kneeFlexionAxisLowerLocal;
            if (targetFlexionAxisWorld.sqrMagnitude < 0.0001f) targetFlexionAxisWorld = currentFlexionAxisWorld;
            targetFlexionAxisWorld.Normalize();
            float targetWTotal = targetRelativeWWorld.magnitude;
            float targetWAlongFlexion = Mathf.Abs(Vector3.Dot(targetRelativeWWorld, targetFlexionAxisWorld));
            Vector3 targetWOffVector = targetRelativeWWorld - targetFlexionAxisWorld * Vector3.Dot(targetRelativeWWorld, targetFlexionAxisWorld);
            _report.KneeTargetRelativeAngularSpeedTotal = targetWTotal;
            _report.KneeTargetRelativeAngularSpeedAlongFlexionAxis = targetWAlongFlexion;
            _report.KneeTargetRelativeAngularSpeedOffAxis = targetWOffVector.magnitude;
            _report.KneeAxisExplainedRatio = targetWTotal > 0.0001f ? Mathf.Clamp01(targetWAlongFlexion / targetWTotal) : 0f;

            Vector3 hipA = GetHipChildAnchorWorld();
            Vector3 hipB = GetHipParentAnchorWorld();
            Vector3 kneeA = _lowerObject.transform.TransformPoint(_kneeChildAnchorLowerLocal);
            Vector3 kneeB = _upperBody.transform.TransformPoint(_kneeParentAnchorUpperLocal);
            _report.HipAnchorErrorMeters = Vector3.Distance(hipA, hipB);
            _report.KneeAnchorErrorMeters = Vector3.Distance(kneeA, kneeB);
            // v0.2.8 diagnostic fix: only accumulate active-drive peaks. After a fail-fast abort the kinematic preview
            // may teleport back to source and must not contaminate the mechanical-drive peak anchor statistic.
            if (_driveEnabled)
            {
                _report.PeakHipRotationErrorDegrees = Mathf.Max(_report.PeakHipRotationErrorDegrees, _report.HipRotationErrorDegrees);
                _report.PeakKneeRotationErrorDegrees = Mathf.Max(_report.PeakKneeRotationErrorDegrees, _report.KneeRotationErrorDegrees);
                _report.PeakHipAnchorErrorMeters = Mathf.Max(_report.PeakHipAnchorErrorMeters, _report.HipAnchorErrorMeters);
                _report.PeakKneeAnchorErrorMeters = Mathf.Max(_report.PeakKneeAnchorErrorMeters, _report.KneeAnchorErrorMeters);
            }
            _report.Finite = HasFiniteState();
            _report.Detail = _driveEnabled ? "LEG_DRIVE_ACTIVE" : "KINEMATIC_PREVIEW";
        }

        private void UpdateAttributionMetrics()
        {
            if (!IsBuilt) return;
            _report.TransportAttributionValid = _measuredTransportValidCurrent && _hasPreviousSourceMotionSample;
            if (!_report.TransportAttributionValid)
            {
                _report.HipRelativeComTransportErrorMagnitude = 0f;
                _report.KneeRelativeComTransportErrorMagnitude = 0f;
                return;
            }

            // 以 segment collider center 作为近似 COM。RigidBody 默认 COM 由 collider 自动计算，
            // 对当前单 BoxCollider segment 这一近似与真实 worldCenterOfMass 基本一致。
            Vector3 sourceUpperCom = _target.UpperLeg.TransformPoint(_upperCollider.center);
            Vector3 sourceLowerCom = _target.LowerLeg.TransformPoint(_lowerCollider.center);
            Vector3 physicsUpperCom = _upperBody.worldCenterOfMass;
            Vector3 physicsLowerCom = _lowerBody.worldCenterOfMass;

            Vector3 sourceUpperComVelocity = _sampledUpperLinearVelocity +
                Vector3.Cross(_sampledUpperAngularVelocity, sourceUpperCom - _sampledUpperPosition);
            Vector3 sourceLowerComVelocity = _sampledLowerLinearVelocity +
                Vector3.Cross(_sampledLowerAngularVelocity, sourceLowerCom - _sampledLowerPosition);
            Vector3 physicsUpperComVelocity = _measuredUpperLinearVelocity +
                Vector3.Cross(_measuredUpperAngularVelocity, physicsUpperCom - _upperBody.position);
            Vector3 physicsLowerComVelocity = _measuredLowerLinearVelocity +
                Vector3.Cross(_measuredLowerAngularVelocity, physicsLowerCom - _lowerBody.position);

            Vector3 sourceHipParentTransport = _sampledSourcePelvisLinearVelocity +
                Vector3.Cross(_sampledSourcePelvisAngularVelocity, sourceUpperCom - _sampledSourcePelvisPosition);
            Vector3 physicsHipParentTransport = _measuredPelvisLinearVelocity +
                Vector3.Cross(_measuredPelvisAngularVelocity, physicsUpperCom - _pelvisBody.position);
            Vector3 sourceHipRelative = sourceUpperComVelocity - sourceHipParentTransport;
            Vector3 physicsHipRelative = physicsUpperComVelocity - physicsHipParentTransport;
            Vector3 hipRelativeError = sourceHipRelative - physicsHipRelative;

            Vector3 sourceKneeParentTransport = _sampledUpperLinearVelocity +
                Vector3.Cross(_sampledUpperAngularVelocity, sourceLowerCom - _sampledUpperPosition);
            Vector3 physicsKneeParentTransport = _measuredUpperLinearVelocity +
                Vector3.Cross(_measuredUpperAngularVelocity, physicsLowerCom - _upperBody.position);
            Vector3 sourceKneeRelative = sourceLowerComVelocity - sourceKneeParentTransport;
            Vector3 physicsKneeRelative = physicsLowerComVelocity - physicsKneeParentTransport;
            Vector3 kneeRelativeError = sourceKneeRelative - physicsKneeRelative;

            _report.HipRelativeComTransportErrorMagnitude = hipRelativeError.magnitude;

            _report.KneeRelativeComTransportErrorMagnitude = kneeRelativeError.magnitude;
        }

        private void UpdateJointPresence()
        {
            _report.HipCharacterJointPresent = _hipJoint != null;
            _report.HipConnectedBodyMatch = _hipJoint != null && _hipJoint.connectedBody == GetActiveHipMechanicalParentBody();
            _hipMechanicalParent.CopyToReport(_report, _hipJoint);
            _hipMechanicalParentTransport.CopyToReport(_report);
            _hipAnchorLinearTransport.CopyToReport(_report);
            _hipCharacterJointResponseIsolation.CopyToReport(_report, null);
            _report.KneeCharacterJointPresent = _kneeJoint != null;
            _report.KneeConnectedBodyMatch = _kneeJoint != null && _kneeJoint.connectedBody == _upperBody;
            _kneeCharacterJointResponseIsolation.CopyToReport(_report);
        }

        private void UpdateTraceStats()
        {
            if (!_traceEnabled || !_driveEnabled)
                return;
            _traceSamples++;
            _traceHipErrorSum += _report.HipRotationErrorDegrees;
            _traceKneeErrorSum += _report.KneeRotationErrorDegrees;
            _traceHipErrorMax = Mathf.Max(_traceHipErrorMax, _report.HipRotationErrorDegrees);
            _traceKneeErrorMax = Mathf.Max(_traceKneeErrorMax, _report.KneeRotationErrorDegrees);
            if (_report.HipAccelerationClamped) _traceHipClampSamples++;
            if (_report.KneeAccelerationClamped) _traceKneeClampSamples++;
            _traceKneeOffAxisErrorSum += _report.KneeOffAxisErrorDegrees;
            _traceKneeOffAxisErrorMax = Mathf.Max(_traceKneeOffAxisErrorMax, _report.KneeOffAxisErrorDegrees);
            _traceKneeSoftAppliedSum += _report.KneeSoftOffAxisAppliedAccelerationMagnitude;
            _traceKneeSoftAppliedMax = Mathf.Max(_traceKneeSoftAppliedMax, _report.KneeSoftOffAxisAppliedAccelerationMagnitude);
            if (_report.KneeSoftOffAxisAccelerationClamped) _traceKneeSoftClampSamples++;
            CopyTraceStatsToReport();
        }

        // Read-only current-step phase snapshot for observers that run before the retained trace classifier.
        // This intentionally mirrors UpdateRetainedPhaseClassification() and never mutates controller state.
        private string GetCurrentAttributionPhaseSnapshot()
        {
            TwoBodyProxyReport torso = _torso.Report;
            if (torso == null) return "UNAVAILABLE";
            if (_torso.ControlPhase.SourceTurnEventActive) return "SOURCE_TURN";
            if (_torso.ControlRecoveryActive || _report.HipControlFrameActive ||
                _report.HipPhysicalRecoveryPending || _report.KneeSoftOffAxisPhysicalRecoveryPending)
                return "RECOVERY";
            if (!torso.MotionTransition && string.Equals(torso.MotionPhase, "IDLE", StringComparison.OrdinalIgnoreCase))
                return "IDLE";
            if (!torso.MotionTransition && string.Equals(torso.MotionPhase, "STEADY", StringComparison.OrdinalIgnoreCase))
                return "NORMAL_STEADY";
            return "NORMAL_TRANSITION";
        }

        // v0.3.17 maintenance: retain only the phase classification still consumed by the
        // bilateral read-only snapshot / Knee safety context. Historical per-phase accumulators
        // and axis-quality statistics were observer-only and are intentionally retired.
        private void UpdateRetainedPhaseClassification()
        {
            _report.AttributionMode = "SINGLE_LEG_PHASE_CLASSIFIER_RETAINED_V0317";
            TwoBodyProxyReport torso = _torso.Report;
            if (torso == null)
            {
                _report.AttributionPhase = "UNAVAILABLE";
                return;
            }

            if (_torso.ControlPhase.SourceTurnEventActive)
                _report.AttributionPhase = "SOURCE_TURN";
            else if (_torso.ControlRecoveryActive || _report.HipControlFrameActive ||
                     _report.HipPhysicalRecoveryPending || _report.KneeSoftOffAxisPhysicalRecoveryPending)
                _report.AttributionPhase = "RECOVERY";
            else if (!torso.MotionTransition && string.Equals(torso.MotionPhase, "IDLE", StringComparison.OrdinalIgnoreCase))
                _report.AttributionPhase = "IDLE";
            else if (!torso.MotionTransition && string.Equals(torso.MotionPhase, "STEADY", StringComparison.OrdinalIgnoreCase))
                _report.AttributionPhase = "NORMAL_STEADY";
            else
                _report.AttributionPhase = "NORMAL_TRANSITION";
        }

        private void EmitTraceIfDue()
        {
            if (!_traceEnabled || !_driveEnabled || CurrentUnscaledTime < _nextTraceTime)
                return;
            _nextTraceTime = CurrentUnscaledTime + TraceLogInterval;
            Plugin.Logger?.LogInfo(
                "[ARL][LEG_TRACE] side=" + (_report.Side ?? "<none>") + " | phase=" + (_report.AttributionPhase ?? "<none>") +
                " | errDeg(H/K/Koff)=" + F(_report.HipRotationErrorDegrees) + "/" + F(_report.KneeRotationErrorDegrees) + "/" + F(_report.KneeOffAxisErrorDegrees) +
                " | strictLag=" + F(_report.HipControlFrameStrictMechanicalRotationLagDegrees) +
                " | governor(active/gap/lead)=" + _report.HipReferenceGovernorActiveThisStep + "/" + F(_report.HipReferenceGovernorCurrentRawGapDegrees) + "/" + F(_report.HipReferenceGovernorCurrentPhysicalLeadDegrees) +
                " | authority(H/K)=" + F(_report.HipAuthorityCurrentMaxAngularAcceleration) + "/" + F(_report.KneeSoftOffAxisAuthorityCurrentMaxAngularAcceleration) +
                " | anchor(H/K/tetherK)=" + F(_report.HipAnchorErrorMeters) + "/" + F(_report.KneeAnchorErrorMeters) + "/" + F(_report.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters) +
                " | clamp(H/K/Ksoft)=" + _report.HipAccelerationClamped + "/" + _report.KneeAccelerationClamped + "/" + _report.KneeSoftOffAxisAccelerationClamped +
                " | render(fresh/held/age)=" + _report.TargetRenderSampleFreshAtFixedStep + "/" + _report.FixedStepsSinceTargetRenderSample + "/" + F(_report.TargetRenderSampleAgeSeconds) +
                " | frozenBaseline=" + SingleLegFreezeReadinessCertification.Mode);
        }

        private string GetTraceSummary()
        {
            return
                "samples=" + _report.TraceSamples +
                " | hipRot(mean/max)=" + F(_report.TraceHipRotationMeanDegrees) + "/" + F(_report.TraceHipRotationMaxDegrees) +
                " | kneeRot(mean/max)=" + F(_report.TraceKneeRotationMeanDegrees) + "/" + F(_report.TraceKneeRotationMaxDegrees) +
                " | kneeOff(mean/max)=" + F(_report.TraceKneeOffAxisErrorMeanDegrees) + "/" + F(_report.TraceKneeOffAxisErrorMaxDegrees) +
                " | kneeSoftA(mean/max/clampSamples)=" + F(_report.TraceKneeSoftOffAxisAppliedMean) + "/" + F(_report.TraceKneeSoftOffAxisAppliedMax) + "/" + _report.TraceKneeSoftOffAxisClampSamples +
                " | clampSamples(hip/knee)=" + _report.TraceHipClampSamples + "/" + _report.TraceKneeClampSamples +
                " | kneeFlexAlloc=" + (_report.KneeFlexionAllocationArm ?? "<none>") +
                "/opposing/suppressed=" + _report.KneeFlexionAllocationOpposingSteps + "/" + _report.KneeFlexionAllocationSuppressedOpposingFfSteps +
                "/clampOriginal/final=" + _report.KneeFlexionAllocationOriginalWouldClampSteps + "/" + _report.KneeFlexionAllocationFinalClampSteps +
                " | legAngularAccelFF=" + (_report.LegAngularAccelerationFeedforwardEnabled ? "ON" : "OFF") +
                "/bound=" + GetAngularAccelerationFeedforwardBoundLabel(_report.LegAngularAccelerationFeedforwardMaxAcceleration) +
                "/gain/maxA/maxAge=" + F(_report.LegAngularAccelerationFeedforwardGain) + "/" + F(_report.LegAngularAccelerationFeedforwardMaxAcceleration) + "/" + F(_report.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds) +
                "/affMeanMax(hipApplied-kneeApplied)=" + F(_report.TraceHipAngularAccelerationFeedforwardAppliedMean) + "/" + F(_report.TraceHipAngularAccelerationFeedforwardAppliedMax) + "," + F(_report.TraceKneeAngularAccelerationFeedforwardAppliedAbsMean) + "/" + F(_report.TraceKneeAngularAccelerationFeedforwardAppliedAbsMax) +
                "/ffClamp=" + _report.HipAngularAccelerationFeedforwardClampCount + "/" + _report.KneeAngularAccelerationFeedforwardClampCount +
                " | inertiaFoundation=" + (_report.RigidbodyInertiaFoundationMode ?? "<none>") +
                "/validated=" + _report.RigidbodyInertiaBuildFoundationValidated +
                " | kneeTorque=CHILD_ONLY" +
                " | anchorNow(hip/knee)=" + F(_report.HipAnchorErrorMeters) + "/" + F(_report.KneeAnchorErrorMeters) +
                " | peakAnchor(hip/knee)=" + F(_report.PeakHipAnchorErrorMeters) + "/" + F(_report.PeakKneeAnchorErrorMeters) +
                " | kneeAxisMode=" + _kneeAxisMode +
                " | kneeAxisCalibration=" + (_report.KneeAxisCalibrationReason ?? "<none>") + "/samples=" + _report.KneeAxisCalibrationSampleCount + "/energy=" + F(_report.KneeAxisCalibrationPrincipalEnergyRatio) +
                " | aborted=" + _report.DriveAborted + "/" + (_report.DriveAbortReason ?? "<none>");
        }

        private void ResetTraceStats()
        {
            _stanceLowerNativeGravity.ResetTrace(_report);
            _kneeSupportReaction.ResetTrace();
            _upperSelfWeightTransfer.ResetTrace();
            _stanceFootLowerReciprocal.ResetTrace(_report);
            _stanceLowerSelfWeightTransfer.ResetTrace(_report);
            _report.AttributionMode = "SINGLE_LEG_PHASE_CLASSIFIER_RETAINED_V0317";
            _report.AttributionPhase = "UNSAMPLED";
            _traceHipErrorSum = 0f;
            _traceKneeErrorSum = 0f;
            _traceHipErrorMax = 0f;
            _traceKneeErrorMax = 0f;
            _traceSamples = 0;
            _traceHipClampSamples = 0;
            _traceKneeClampSamples = 0;
            _traceKneeOffAxisErrorSum = 0f;
            _traceKneeOffAxisErrorMax = 0f;
            _traceKneeSoftAppliedSum = 0f;
            _traceKneeSoftAppliedMax = 0f;
            _traceKneeSoftClampSamples = 0;
            _report.HipClampCount = 0;
            _report.KneeClampCount = 0;
            _report.KneeSoftOffAxisClampCount = 0;
            _report.PeakHipRotationErrorDegrees = 0f;
            _report.PeakKneeRotationErrorDegrees = 0f;
            _report.PeakHipAnchorErrorMeters = 0f;
            _report.PeakKneeAnchorErrorMeters = 0f;
            _hipMomentumRecoveryBraking.ResetDiagnostics(_report);
            _hipVectorCaptureDamping.ResetDiagnostics(_report);
            _hipAngularActuator.ResetDiagnostics(_report);
            _legAngularDragContract.ResetRunDiagnostics(_report, _upperBody, _lowerBody, _profile.AngularDrag);
            _hipMechanicalParent.ResetStatistics();
            _hipMechanicalParentTransport.ResetDiagnostics(_report);
            _hipAnchorLinearTransport.ResetDiagnostics(_report);
            _hipCharacterJointResponseIsolation.ResetStats(_report);
            _hipAnchorAccelerationFeedforward.ResetDiagnostics(_report);
            _kneeCharacterJointResponseIsolation.ResetStats(_report);
            _kneeAnchorAccelerationFeedforward.ResetDiagnostics(_report);
            _kneeFlexionAllocation.ResetDiagnostics(_report);
            _singleLegFreezeCertification.Reset(_report);
            _kneeParentAngularTransport.ResetDiagnostics(_report);
            _hipReferenceGovernor.ResetDiagnostics();
            _hipSourceReferenceShockAbsorber.ResetDiagnostics(_report);
            _hipCompositeReferenceShockAbsorber.ResetDiagnostics(_report);
            _hipControlFrame.ResetDiagnostics();
            _legAngularAccelerationFeedforward.ResetDiagnostics();
            _hipNormalReference.ResetTrace();
            _kneeReference.ResetTrace();
            _report.HipNormalReferenceSummary = null;
            _report.KneeReferenceSummary = null;
            _report.KneePlannedFeedforwardSummary = null;
            CopyLegAngularAccelerationFeedforwardToReport();
            CopyTraceStatsToReport();
        }

        private void CopyTraceStatsToReport()
        {
            _report.TraceSamples = _traceSamples;
            _report.TraceHipRotationMeanDegrees = _traceSamples > 0 ? _traceHipErrorSum / _traceSamples : 0f;
            _report.TraceHipRotationMaxDegrees = _traceHipErrorMax;
            _report.TraceKneeRotationMeanDegrees = _traceSamples > 0 ? _traceKneeErrorSum / _traceSamples : 0f;
            _report.TraceKneeRotationMaxDegrees = _traceKneeErrorMax;
            _report.TraceHipClampSamples = _traceHipClampSamples;
            _report.TraceKneeClampSamples = _traceKneeClampSamples;
            _report.TraceKneeOffAxisErrorMeanDegrees = _traceSamples > 0 ? _traceKneeOffAxisErrorSum / _traceSamples : 0f;
            _report.TraceKneeOffAxisErrorMaxDegrees = _traceKneeOffAxisErrorMax;
            _report.TraceKneeSoftOffAxisAppliedMean = _traceSamples > 0 ? _traceKneeSoftAppliedSum / _traceSamples : 0f;
            _report.TraceKneeSoftOffAxisAppliedMax = _traceKneeSoftAppliedMax;
            _report.TraceKneeSoftOffAxisClampSamples = _traceKneeSoftClampSamples;
        }

        private string GetDriveSummary()
        {
            return
                "ARL leg drive | enabled=" + _driveEnabled +
                " | hip=" + (_hipCharacterJointResponseIsolation.Enabled ? "NO_CHARACTERJOINT+TRANSLATION_ONLY_TETHER+RelativePD" : "CharacterJoint+RelativePD") + "/jointIsolation=" + _hipCharacterJointResponseIsolation.Arm + "/previewPreprocessing=ON_IF_JOINT_PRESENT" + "/preprocess=" + (_hipJoint != null && _hipJoint.enablePreprocessing) + "/MaxA(base/source/recovery)=" + F(_profile.LegHipMaxAngularAcceleration) + "/" + F(_profile.LegHipSourceTurnMaxAngularAcceleration) + "/" + F(_profile.LegHipRecoveryMaxAngularAcceleration) +
                "/controlFrame=" + (_profile.LegHipControlFrameInertialization ? "STRICT_WA_BRAKING_AWARE_BOUNDED_LAG" : "RAW_PARENT_FRAME") +
                "/maxLag=" + F(_profile.LegHipControlFrameMaxLagDegrees) + "deg" +
                "/hipReference=" + HipFeasibilityReferenceGovernor.ModeName +
                "/governorLead=10deg" +
                "/momentumRecoveryBraking=" + _hipMomentumRecoveryBraking.Arm + "/vectorCapture=" + _hipVectorCaptureDamping.Arm + "/lagGuardRebase=" + _hipControlFrame.ReferenceLagGuardRebaseArm + "/sourceShockAbsorber=" + _hipSourceReferenceShockAbsorber.Arm + "/compositeShockAbsorber=" + _hipCompositeReferenceShockAbsorber.Arm +
                "/mechanicalParent=" + _hipMechanicalParent.Mode + "/parentTransportFF=" + (_hipMechanicalParentTransport.Enabled ? "ON" : "OFF") + "/anchorLinearTransport=" + (_hipAnchorLinearTransport.Enabled ? "ON" : "OFF") +
                " | knee=" + (_kneeCharacterJointResponseIsolation.Enabled ? "NO_CHARACTERJOINT+TRANSLATION_ONLY_TETHER+FlexionPD+SoftOffAxisPD/CHILD_ONLY" : "WideCharacterJoint+FlexionPD+SoftOffAxisPD/CHILD_ONLY") +
                "/parentAngularTransport=" + _kneeParentAngularTransport.Arm +
                "/jointIsolation=" + _kneeCharacterJointResponseIsolation.Arm +
                "/supportNormalReaction=" + _kneeSupportReaction.Arm +
                "/anchorAccelFF=" + _kneeAnchorAccelerationFeedforward.Arm +
                " | kneeAxis=" + _kneeAxisMode + "/" + V(_kneeFlexionAxisLowerLocal) +
                " | kneeSoftPD=" + F(_profile.LegKneeSoftOffAxisAngularSpring) + "/" + F(_profile.LegKneeSoftOffAxisAngularDamper) + "/baseline=" + F(_profile.LegKneeSoftOffAxisMaxAngularAcceleration) + "/source=" + F(_profile.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration) + "/recovery=" + F(_profile.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration) +
                " | legAngularAccelFF=" + (_legAngularAccelerationFeedforward.Enabled ? "ON" : "OFF") + "/" + SingleLegAngularAccelerationFeedforward.ModeName + "/normal=PRODUCTION_ON" +
                "/bound=" + GetAngularAccelerationFeedforwardBoundLabel(_profile.LegAngularAccelerationFeedforwardMaxAcceleration) +
                " | inertiaFoundation=" + SingleLegRigidbodyMassProperties.FoundationMode +
                "/buildValidated=" + _rigidbodyMassProperties.BuildFoundationValidated +
                " | hipActuator=" + _hipAngularActuator.Arm + "/upperDrag=" + LegAngularDragContract.Mode + "/readback=" + F(_upperBody != null ? _upperBody.angularDrag : 0f) +
                " | footFoundation=" + (_footFoundation != null ? "V0358_SOURCE_BONE_RELATIVE_ANKLE_PD_FROZEN+V0363_NO_CHARACTERJOINT_TETHER_FROZEN+V0364_LINEAR_PARENT_TRANSPORT_FROZEN+V0365_ANGULAR_PARENT_TRANSPORT_FROZEN+V0368_PERSISTENCE_CONDITIONED_TARGET_A_FF_FROZEN+V0373_CONDITIONAL_CAPTURE_AUTHORITY_PRODUCTION+V0378_BOUNDED_COMMON_MODE_DRAG_COMPENSATION_PRODUCTION+V03561_SUPPORT_PLANE" : "OFF") +
                "/ankleTargetAFF=" + (_footFoundation != null ? _footFoundation.AnkleTargetAccelerationMode : FootAnkleSourceDrivenRelativePdIsolation.ProductionTargetAccelerationMode) +
                " | upperHipObserver=RETIRED_V035" +
                " | gravity=" + _gravityEnabled +
                " | colliders=" + _collidersEnabled +
                " | diagnostics=" + SingleLegFreezeReadinessCertification.Mode +
                " | realInertia=True";
        }
    }
}
