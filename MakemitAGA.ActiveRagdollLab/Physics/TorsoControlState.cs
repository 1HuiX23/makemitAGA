using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // A value captured at the existing source/motion publication boundary. Helpers
    // receive this small input, never a mutable diagnostic report or native owner.
    internal readonly struct TorsoControlPhase
    {
        internal bool SourceTurnEventActive { get; }
        internal string MotionPhase { get; }
        internal bool MotionTransition { get; }

        internal TorsoControlPhase(bool sourceTurnEventActive, string motionPhase, bool motionTransition)
        {
            SourceTurnEventActive = sourceTurnEventActive;
            MotionPhase = motionPhase;
            MotionTransition = motionTransition;
        }
    }

    // Owns recovery selection/publication and the observations used by that control
    // contract. Observe methods consume values already sampled by TwoBody, at the
    // original assignment points; none samples Unity or reads a report. Independent
    // publications deliberately retain old values when their original branch is
    // skipped, including partial native-read failures and calls outside FixedUpdate.
    internal sealed class TorsoControlState
    {
        private readonly DynamicTorsoRecoveryGate _dynamicRecovery = new DynamicTorsoRecoveryGate();
        private bool _legacyRecoveryActive;

        internal TorsoControlPolicy Policy { get; private set; }
        internal TorsoRecoveryState PublishedRecovery { get; private set; }
        internal bool RecoveryActive => Policy.UseDynamicTorsoRecovery
            ? !PublishedRecovery.Valid || PublishedRecovery.Active : _legacyRecoveryActive;
        internal string RecoverySummary => _dynamicRecovery.GetSummary(Policy.UseDynamicTorsoRecovery);
        internal TorsoControlPhase Phase => new TorsoControlPhase(SourceTurnEventActive, MotionPhase, MotionTransition);

        internal bool Built { get; private set; }
        internal bool SourceAlive { get; private set; }
        internal bool FiniteState { get; private set; }
        internal bool CharacterJointPresent { get; private set; }
        internal bool CharacterJointConnectedBodyMatch { get; private set; }
        internal bool CharacterJointAnchorModeValidated { get; private set; }
        internal float RotationErrorDegrees { get; private set; }
        internal float ChestPositionErrorMeters { get; private set; }
        internal float CharacterJointConstraintAnchorErrorMeters { get; private set; }
        internal float AnchorErrorMeters { get; private set; }
        internal Vector3 MeasuredAnchorRelativeVelocity { get; private set; }
        internal Vector3 TargetRelativeAngularVelocity { get; private set; }
        internal bool AngularAccelerationClamped { get; private set; }
        internal bool ParentFeedforwardAccelerationClamped { get; private set; }
        internal string MotionPhase { get; private set; }
        internal bool MotionTransition { get; private set; }
        internal float MotionTargetSpeed { get; private set; }
        internal bool SourceTurnEventActive { get; private set; }

        internal void SelectPolicy(bool success, string arm) => Policy = TorsoControlPolicy.FromReceiverSelection(success, arm);

        internal TorsoRecoveryState BeginFixedStep()
        {
            // The internal gate may still hold the previous step. Consumers must
            // keep seeing the original unpublished sentinel until ObserveRecovery.
            PublishedRecovery = new TorsoRecoveryState { Active = true, Reason = "STEP_NOT_PUBLISHED" };
            return PublishedRecovery;
        }

        internal TorsoRecoveryState ResetRecovery()
        {
            _dynamicRecovery.ResetState();
            PublishedRecovery = _dynamicRecovery.Current;
            return PublishedRecovery;
        }

        internal void ResetRecoveryDiagnostics() => _dynamicRecovery.ResetDiagnostics();

        internal TorsoRecoveryState ObserveRecovery(PhysicsStepContext step, bool insideRuntimeStep,
            bool receiverReady, bool receiverErrorsValid, float receiverComError, float receiverPelvisError)
        {
            _dynamicRecovery.Step(step, insideRuntimeStep && receiverReady && receiverErrorsValid &&
                FiniteState && CharacterJointConnectedBodyMatch && CharacterJointAnchorModeValidated,
                SourceTurnEventActive, receiverComError, receiverPelvisError,
                RotationErrorDegrees, CharacterJointConstraintAnchorErrorMeters, AnchorErrorMeters);
            PublishedRecovery = _dynamicRecovery.Current;
            return PublishedRecovery;
        }

        internal void RecordRecovery(bool tracing)
        {
            if (tracing) _dynamicRecovery.Record(_legacyRecoveryActive);
        }

        // Legacy recovery is published only where the validator previously copied
        // its live flag to the report; reading its private current state early would
        // change the handoff at trace start/stop and failed finalization boundaries.
        internal bool PublishLegacyRecovery(bool active) => _legacyRecoveryActive = active;

        // Return the observed value so the report projection can be assigned at the
        // same source location, without reading mutable report storage back here.
        internal bool ObserveBuilt(bool value) => Built = value;
        internal bool ObserveSourceAlive(bool value) => SourceAlive = value;
        internal bool ObserveFiniteState(bool value) => FiniteState = value;
        internal bool ObserveJointPresent(bool value) => CharacterJointPresent = value;
        internal bool ObserveJointConnection(bool value) => CharacterJointConnectedBodyMatch = value;
        internal bool ObserveJointAnchorValidation(bool value) => CharacterJointAnchorModeValidated = value;
        internal float ObserveRotationError(float value) => RotationErrorDegrees = value;
        internal float ObserveChestPositionError(float value) => ChestPositionErrorMeters = value;
        internal float ObserveConstraintAnchorError(float value) => CharacterJointConstraintAnchorErrorMeters = value;
        internal float ObserveVirtualAnchorError(float value) => AnchorErrorMeters = value;
        internal Vector3 ObserveAnchorRelativeVelocity(Vector3 value) => MeasuredAnchorRelativeVelocity = value;
        internal Vector3 ObserveTargetRelativeAngularVelocity(Vector3 value) => TargetRelativeAngularVelocity = value;
        internal bool ObserveAngularClamp(bool value) => AngularAccelerationClamped = value;
        internal bool ObserveParentFeedforwardClamp(bool value) => ParentFeedforwardAccelerationClamped = value;
        internal string PublishMotionPhase(string value) => MotionPhase = value;
        internal bool PublishMotionTransition(bool value) => MotionTransition = value;
        internal float PublishMotionTargetSpeed(float value) => MotionTargetSpeed = value;
        internal bool PublishSourceTurn(bool value) => SourceTurnEventActive = value;
    }
}
