using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        private LegTurnStepEvidence _legTurnCurrent;
        private HipCommandWrench _hipWrenchCurrent;
        internal HipCommandWrench DynamicHipCommand => _hipWrenchCurrent;

        // Called only after the original force and physical AddTorque calls have returned.
        // Optional diagnostic failure cannot alter the controller or its run safety.
        private void CaptureDynamicHipCommand(Vector3 committedTorque)
        {
            if (!_traceEnabled || !_dynamicParentActive || !PelvisRunScopes.HasUpperWeight(_dynamicScope)) return;
            // Hip scope stages the real torque independently of trace; its audit is
            // captured after the bilateral force commit at the end of this same step.
            if (PelvisRunScopes.HasHipCoupling(_dynamicScope)) return;
            try
            {
                if (!_hipCharacterJointResponseIsolation.Enabled || !_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled ||
                    !_hipAngularActuator.Enabled || !_hipAngularActuator.CurrentValid || _hipJoint != null ||
                    !_physicalParent.Matches(_runtimeStep) || _physicalParent.BodyId != _pelvisBody.GetInstanceID()) return;
                Vector3 upperCom = _upperBody.worldCenterOfMass;
                Vector3 childAnchor = GetHipChildAnchorWorld(), parentAnchor = GetHipParentAnchorWorld();
                _hipWrenchCurrent = new HipCommandWrench
                {
                    Valid = true, Step = _runtimeStep, Fixed = _fixedUpdateSequence,
                    UpperBodyId = _upperBody.GetInstanceID(), ParentBodyId = _pelvisBody.GetInstanceID(), Mass = _upperBody.mass,
                    UpperCom = upperCom, ParentCom = _physicalParent.SettledPoint(_physicalParent.ComLocal),
                    ChildAnchor = childAnchor, ParentAnchor = parentAnchor,
                    UpperV = _upperBody.velocity, UpperW = _upperBody.angularVelocity,
                    ParentV = _physicalParent.ComVelocity, ParentW = _physicalParent.W,
                    ChildPointV = PointVelocity(_upperBody, childAnchor),
                    ParentPointV = _physicalParent.SettledPointVelocity(_rawHipConnectedAnchorLocal),
                    Force = _upperBody.mass * _hipCharacterJointResponseIsolation.CurrentAppliedAccelerationWorld,
                    Torque = committedTorque
                };
            }
            catch { _hipWrenchCurrent = default; }
        }
        private SourceStepContext _legTurnRawSource;
        private bool _legTurnRawSourceValid;
        private float _legTurnRawSourceDt;
        private Vector3 _legTurnRawUpperW, _legTurnRawLowerW;

        // Tap the raw derivatives ALREADY computed by real Render sampling, before its clamp/blend.
        // No new estimator, native getter, clock, observer loop or physical-history reset.
        private void RecordLegTurnRawSource(Vector3 upperW, Vector3 lowerW, float dt)
        {
            _legTurnRawSourceValid = _traceEnabled && _dynamicParentActive && _runtimeSource.Valid &&
                IsFinite(upperW) && IsFinite(lowerW) && IsFinite(dt) && dt >= 0.001f && dt <= 0.20f;
            if (!_legTurnRawSourceValid) return;
            _legTurnRawSource = _runtimeSource;
            _legTurnRawSourceDt = dt;
            _legTurnRawUpperW = upperW; _legTurnRawLowerW = lowerW;
        }

        private void ResetLegTurnEvidence()
        {
            _legTurnCurrent = new LegTurnStepEvidence();
            _hipWrenchCurrent = default;
            _legTurnRawSourceValid = false;
        }

        private void BeginLegTurnEvidence()
        {
            _legTurnCurrent = new LegTurnStepEvidence();
            _hipWrenchCurrent = default;
            if (!_traceEnabled || !_dynamicParentActive) return;
            _legTurnCurrent.Step = _runtimeStep; _legTurnCurrent.Fixed = _fixedUpdateSequence;
            _legTurnCurrent.Source = _runtimeSource;
            _legTurnCurrent.SourcePhase = _torso.Report != null ? _torso.Report.MotionPhase : null;
            _legTurnCurrent.RawWorldWValid = _legTurnRawSourceValid && _legTurnRawSource.Matches(_runtimeSource);
            _legTurnCurrent.RawSourceDt = _legTurnRawSourceDt;
            _legTurnCurrent.RawUpperW = _legTurnRawUpperW; _legTurnCurrent.RawLowerW = _legTurnRawLowerW;
            _legTurnCurrent.RawUpperR = _sampledUpperRotation; _legTurnCurrent.RawLowerR = _sampledLowerRotation;
            _legTurnCurrent.RawHipRelativeR = _sampledTargetHipRelative;
            // UpdateMeasuredTransportKinematics has just refreshed these cached native snapshots.
            _legTurnCurrent.UpperR = _previousMeasuredUpperRotation;
            _legTurnCurrent.UpperW = _measuredUpperAngularVelocity;
        }

        private void CaptureLegTurnHipFeedforward(bool stepValid, Vector3 inputA, float kneeInputA)
        {
            if (!_traceEnabled || !_dynamicParentActive) return;
            try
            {
                // Called immediately after FF.Step, before any of its gate owners advance again.
                // Read the actual first-match reason; do not create a second decision tree from the bits.
                TwoBodyProxyReport torso = _torso.Report;
                _legTurnCurrent.HipNormalReference = _hipNormalReference.Current;
                _legTurnCurrent.HipNormalRecoveryHandoffStarted = _hipCompositeReferenceShockAbsorber.NormalRecoveryStartedThisStep;
                _legTurnCurrent.HipNormalRecoveryHandoffActive = _hipCompositeReferenceShockAbsorber.NormalRecoveryHandoffActive;
                _legTurnCurrent.HipFeedforwardEnabled = _legAngularAccelerationFeedforward.Enabled;
                _legTurnCurrent.HipFeedforwardActive = _legAngularAccelerationFeedforward.HipActiveThisStep;
                _legTurnCurrent.HipFeedforwardClamped = _legAngularAccelerationFeedforward.HipClamped;
                _legTurnCurrent.HipFeedforwardReason = _legAngularAccelerationFeedforward.SuppressionReason;
                _legTurnCurrent.HipFeedforwardStateBits = (torso == null ? 1 : 0) |
                    (torso != null && torso.SourceTurnEventActive ? 2 : 0) |
                    (torso != null && torso.ControlRecoveryActive ? 4 : 0) |
                    (_hipAuthority.PhysicalRecoveryPending ? 8 : 0) |
                    (_softKneeAuthority.PhysicalRecoveryPending ? 16 : 0) |
                    (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization ? 32 : 0) |
                    (_hipAuthority.TrackingHealthUnhealthy ? 64 : 0) |
                    (_softKneeAuthority.TrackingHealthUnhealthy ? 128 : 0) |
                    (HipFeedforwardGoalBridgeActive ? 256 : 0);
                _legTurnCurrent.TorsoRecoveryDynamicSelected = torso != null && torso.UseDynamicTorsoRecovery;
                _legTurnCurrent.TorsoRecoveryLegacyActive = torso != null && torso.FreezeRecoveryActive;
                if (torso != null) _legTurnCurrent.TorsoRecovery = torso.DynamicTorsoRecovery;
                _legTurnCurrent.HipGoalGateKneeIndependent = _dynamicParentActive && torso != null && torso.HipGoalGateOnlyHip;
                _legTurnCurrent.KneeFeedforwardActive = _legAngularAccelerationFeedforward.KneeActiveThisStep;
                _legTurnCurrent.KneeFeedforwardClamped = _legAngularAccelerationFeedforward.KneeClamped;
                _legTurnCurrent.KneeFeedforwardReason = _legAngularAccelerationFeedforward.KneeSuppressionReason;
                _legTurnCurrent.KneeFeedforwardInputSigned = kneeInputA;
                _legTurnCurrent.KneeFeedforwardSelectedSigned = _legAngularAccelerationFeedforward.KneeAppliedSigned;
                _legTurnCurrent.KneeFeedforwardValid = stepValid && _legTurnCurrent.KneeFeedforwardReason != null &&
                    IsFinite(kneeInputA) && IsFinite(_legTurnCurrent.KneeFeedforwardSelectedSigned);
                _legTurnCurrent.HipFeedforwardAge = _report.TargetRenderSampleAgeSeconds;
                _legTurnCurrent.HipFeedforwardMaxAge = _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds;
                _legTurnCurrent.HipFeedforwardGain = _profile.LegAngularAccelerationFeedforwardGain;
                _legTurnCurrent.HipFeedforwardBound = _profile.LegAngularAccelerationFeedforwardMaxAcceleration;
                // Input is the existing Render-derived relative A mapped to the controller parent basis,
                // not d(final world goal W)/dt. Selected A is before the original final Hip sum/cap.
                _legTurnCurrent.HipFeedforwardInputA = inputA;
                _legTurnCurrent.HipFeedforwardSelectedA = _legAngularAccelerationFeedforward.HipAppliedWorld;
                _legTurnCurrent.HipFeedforwardValid = stepValid && _legTurnCurrent.HipFeedforwardReason != null &&
                    IsFinite(inputA) && IsFinite(_legTurnCurrent.HipFeedforwardSelectedA) &&
                    IsFinite(_legTurnCurrent.HipFeedforwardAge) && IsFinite(_legTurnCurrent.HipFeedforwardMaxAge) &&
                    IsFinite(_legTurnCurrent.HipFeedforwardGain) && IsFinite(_legTurnCurrent.HipFeedforwardBound);
            }
            catch { _legTurnCurrent.HipFeedforwardValid = _legTurnCurrent.KneeFeedforwardValid = false; } // Optional evidence cannot interrupt the controller.
        }

        // After composite conversion, before the ordinary world-reference stage. Use cached
        // settled parent/Upper and the exact already-computed targets, never native rereads.
        private void CaptureHipGoalPath(Quaternion parentR, Quaternion afterGovernor)
        {
            if (!_traceEnabled || !_dynamicParentActive) return;
            _legTurnCurrent.HipGoalPath = new HipGoalPathEvidence
            {
                Prepared = true, Step = _runtimeStep, Source = _runtimeSource, Fixed = _fixedUpdateSequence,
                RawUpper = _sampledUpperRotation, PhysicalParent = _previousMeasuredPelvisRotation,
                ControlParent = parentR, RawInControl = parentR * _sampledTargetHipRelative,
                AfterSource = parentR * _sourceControlTargetHipRelative,
                AfterGovernor = afterGovernor, AfterComposite = parentR * _controlTargetHipRelative
            };
        }

        private void CaptureLegTurnHip(bool frameActive, Quaternion parentR, Vector3 targetRelativeW,
            Vector3 relativeRawA, Vector3 combinedRawA, Vector3 finalA, float maxA, bool clamped)
        {
            if (!_traceEnabled || !_dynamicParentActive) return;
            _legTurnCurrent.HipParentW = frameActive ? _hipControlFrame.AngularVelocityWorld : _measuredPelvisAngularVelocity;
            _legTurnCurrent.HipTargetR = parentR * _controlTargetHipRelative;
            _legTurnCurrent.HipTargetW = _legTurnCurrent.HipParentW + targetRelativeW;
            _legTurnCurrent.HipFinalA = finalA;
            // This is the PRE-cap parent contribution; never label it an applied/native torque.
            _legTurnCurrent.HipParentPreCapA = combinedRawA - relativeRawA;
            _legTurnCurrent.HipMaxA = maxA;
            _legTurnCurrent.Modes = (frameActive ? 1 : 0) |
                (_hipSourceReferenceShockAbsorber.Active ? 2 : 0) | (_hipReferenceGovernor.Active ? 4 : 0) |
                (_hipCompositeReferenceShockAbsorber.OutputAppliedThisStep ? 8 : 0) |
                (_hipAuthority.PhysicalRecoveryPending ? 16 : 0) | (_legAngularAccelerationFeedforward.HipActiveThisStep ? 32 : 0) |
                (_hipMechanicalParentTransport.ActiveThisStep ? 64 : 0) | (clamped ? 128 : 0);
            _legTurnCurrent.HipCommitted = true;
            HipGoalPathEvidence path = _legTurnCurrent.HipGoalPath;
            path.Final = _legTurnCurrent.HipTargetR; path.PhysicalUpper = _legTurnCurrent.UpperR;
            path.Modes = _legTurnCurrent.Modes; path.NormalApplied = _hipNormalReference.Current.Applied;
            path.Committed = true; _legTurnCurrent.HipGoalPath = path;
        }

        private void CaptureLandingReferenceInput()
        {
            if (!_traceEnabled || !_dynamicParentActive || _footFoundation == null ||
                !_footFoundation.LandingReferenceCaptureRequested) return;
            // Capture only a committed dynamic, joint-free chain. No diagnostic state can
            // become a dependency of native writes. Zero is the actual Hip child anchor here.
            try
            {
                if (!_legTurnCurrent.HipCommitted || !_legTurnCurrent.KneeCommitted ||
                    _hipJoint != null || _kneeJoint != null || !_physicalParent.Matches(_runtimeStep)) return;
                Vector3 upperScale = _upperBody.transform.lossyScale, lowerScale = _lowerBody.transform.lossyScale;
                if (!IsFinite(upperScale) || !IsFinite(lowerScale) ||
                    upperScale.x <= 0f || upperScale.y <= 0f || upperScale.z <= 0f ||
                    lowerScale.x <= 0f || lowerScale.y <= 0f || lowerScale.z <= 0f) return;
                _legTurnCurrent.LandingInput = new LegLandingReferenceInput
                {
                    Prepared = true, Step = _runtimeStep, Source = _runtimeSource, Fixed = _fixedUpdateSequence,
                    UpperGoal = _legTurnCurrent.HipTargetR, KneeRelativeGoal = _controlTargetKneeRelative,
                    Upper = _legTurnCurrent.UpperR, ParentHip = _physicalParent.SettledPoint(_rawHipConnectedAnchorLocal),
                    UpperHip = _upperBody.position,
                    UpperHipToKnee = Vector3.Scale(_kneeParentAnchorUpperLocal, upperScale),
                    LowerKneeAnchor = Vector3.Scale(_kneeChildAnchorLowerLocal, lowerScale),
                    RawHip = _sampledUpperPosition, RawLower = _sampledLowerRotation,
                    RawKneeRelative = _sampledTargetKneeRelative,
                    HipPath = _legTurnCurrent.HipGoalPath, Normal = _hipNormalReference.Current,
                    Parent = CaptureLandingParentInput(), Knee = CaptureLandingKneeInput()
                };
            }
            catch { _legTurnCurrent.LandingInput = default; }
        }

        private LandingParentPlacementInput CaptureLandingParentInput()
        {
            // Called only inside the existing inactive-entry window quota. Failure
            // leaves the original ankle path and the independent Knee receipt intact.
            try
            {
                if (_torso == null || !_physicalParent.Matches(_runtimeStep) ||
                    !_torso.TryGetHeldPelvisTargetPose(_runtimeSource, out Vector3 rawP, out Quaternion rawR)) return default;
                return new LandingParentPlacementInput
                {
                    Prepared = true, Step = _physicalParent.Step, Source = _runtimeSource,
                    RawP = rawP, RawR = rawR, LocalAnchor = _rawHipConnectedAnchorLocal,
                    ReferenceP = _physicalParent.CommandPosition, ReferenceR = _physicalParent.CommandRotation,
                    ActualP = _physicalParent.Position, ActualR = _physicalParent.Rotation
                };
            }
            catch { return default; }
        }

        private LandingKneeControlInput CaptureLandingKneeInput()
        {
            try
            {
                if (!_legTurnCurrent.KneeCommitted || !_legTurnCurrent.Step.Matches(_runtimeStep) ||
                    !_legTurnCurrent.Source.Matches(_runtimeSource) || _report == null) return default;
                return new LandingKneeControlInput
                {
                    Prepared = true, Step = _legTurnCurrent.Step, Source = _legTurnCurrent.Source, Fixed = _fixedUpdateSequence,
                    ErrorAbsDegrees = _report.KneeControlAxisErrorDegrees,
                    TargetW = _report.KneeTargetAxisAngularVelocitySigned, BodyW = _report.KneeBodyAxisAngularVelocitySigned,
                    VelocityError = _report.KneeAxisAngularVelocityErrorSigned,
                    P = _report.KneePdPContributionSigned, D = _report.KneePdDContributionSigned,
                    Feedback = _legTurnCurrent.KneeFeedbackSigned,
                    FeedforwardValid = _legTurnCurrent.KneeFeedforwardValid,
                    FeedforwardActive = _legTurnCurrent.KneeFeedforwardActive,
                    FeedforwardClamped = _legTurnCurrent.KneeFeedforwardClamped,
                    FeedforwardReason = _legTurnCurrent.KneeFeedforwardReason,
                    IndependentGate = _legTurnCurrent.HipGoalGateKneeIndependent,
                    InputFF = _legTurnCurrent.KneeFeedforwardInputSigned, SelectedFF = _legTurnCurrent.KneeFeedforwardSelectedSigned,
                    AllocatedFF = _legTurnCurrent.KneeFeedforwardAllocatedSigned, FinalFlexion = _legTurnCurrent.KneeFlexionFinalSigned,
                    MaxFlexionA = _profile.LegKneeMaxAngularAcceleration, FlexionClamped = _report.KneeAccelerationClamped,
                    SoftRawA = _report.KneeSoftOffAxisRawAccelerationMagnitude,
                    SoftAppliedA = _report.KneeSoftOffAxisAppliedAccelerationMagnitude,
                    SoftMaxA = _softKneeAuthority.CurrentMaxAngularAcceleration, SoftPhase = _softKneeAuthority.Phase,
                    SoftClamped = _report.KneeSoftOffAxisAccelerationClamped,
                    ParentTransportEnabled = _kneeParentAngularTransport.Enabled,
                    RelativeA = _legTurnCurrent.KneeRelativeA,
                    ParentA = _kneeParentAngularTransport.Enabled ? _lastHipAppliedAngularAccelerationWorld : Vector3.zero,
                    TotalA = _legTurnCurrent.KneeTotalA,
                    ReferenceSelected = _kneeReference.Selected, Reference = _kneeReference.Current,
                    PlannedSourceRecoverySelected = _kneePlannedFeedforwardSelected,
                    FeedforwardStateBits = _legTurnCurrent.HipFeedforwardStateBits,
                    RawSourceA = _sampledTargetKneeRelativeAngularAccelerationLocal
                };
            }
            catch { return default; }
        }

        private void CaptureLegTurnKnee(Vector3 targetRelativeW, Vector3 relativeA, Vector3 totalA,
            float feedbackSigned, float allocatedFeedforwardSigned, float finalFlexionSigned)
        {
            if (!_traceEnabled || !_dynamicParentActive) return;
            _legTurnCurrent.KneeTargetRelW = targetRelativeW;
            _legTurnCurrent.KneeRelativeA = relativeA; _legTurnCurrent.KneeTotalA = totalA;
            // Values from the successful original allocator/native command, not a second allocation.
            _legTurnCurrent.KneeFeedbackSigned = feedbackSigned;
            _legTurnCurrent.KneeFeedforwardAllocatedSigned = allocatedFeedforwardSigned;
            _legTurnCurrent.KneeFlexionFinalSigned = finalFlexionSigned;
            _legTurnCurrent.KneeCommitted = true;
            CaptureLandingReferenceInput();
        }
    }
}
