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
        }
    }
}
