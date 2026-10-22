using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed class TwoBodyProxyReport
    {
        internal PelvisReceiverReport PelvisReceiver;
        internal bool UseDynamicTorsoRecovery;
        internal TorsoRecoveryState DynamicTorsoRecovery;
        // Legacy and standalone scopes retain their original qualification. Missing B state suppresses FF.
        internal bool ControlRecoveryActive => UseDynamicTorsoRecovery
            ? !DynamicTorsoRecovery.Valid || DynamicTorsoRecovery.Active : FreezeRecoveryActive;
        public bool Built;
        public bool DriveEnabled;
        public bool GravityEnabled;
        public bool CollidersEnabled;
        public bool VisualEnabled;
        public bool SourceAlive;
        public bool FiniteState;
        public string TargetName;
        public string TargetPath;
        public string RootName;
        public Vector3 PelvisSize;
        public Vector3 ChestSize;
        public float TorsoLength;
        public float RotationErrorDegrees;
        public float AnchorErrorMeters;
        public float PelvisPositionErrorMeters;
        public float ChestPositionErrorMeters;

        public Vector3 TargetChestLinearVelocity;
        public Vector3 TargetChestAngularVelocity;
        public Vector3 TargetPelvisLinearVelocity;
        public Vector3 TargetPelvisAngularVelocity;
        public Vector3 TargetChestLinearAcceleration;
        public Vector3 TargetChestAngularAcceleration;
        public Vector3 TargetPelvisLinearAcceleration;
        public Vector3 TargetPelvisAngularAcceleration;

        public Vector3 PelvisVelocity;
        public Vector3 PelvisAngularVelocity;
        public Vector3 EstimatedPelvisLinearVelocity;
        public Vector3 EstimatedPelvisAngularVelocity;
        // Source/render parent transport remains the animation-side reference.
        public Vector3 ParentTransportLinearVelocity;
        // v0.1.11: what the CharacterJoint actually sees from the Pose Bridge command at the target Chest COM lever arm.
        public Vector3 PhysicsParentTransportLinearVelocity;
        public Vector3 ParentTransportConsistencyError;
        public float ParentTransportConsistencyErrorMagnitude;
        public string ParentFeedforwardReferenceMode;
        public Vector3 ParentFeedforwardReferenceVelocity;
        public Vector3 SourceRelativeLinearVelocity;
        public Vector3 TargetRelativeAngularVelocity;
        public Vector3 BodyRelativeAngularVelocity;
        public Vector3 RelativeAngularVelocityError;
        public Vector3 ChestVelocity;
        public Vector3 ChestAngularVelocity;
        public Vector3 FixedStepPelvisTransportLinearVelocity;
        public Vector3 FixedStepPelvisTransportAngularVelocity;
        public Vector3 FixedStepTargetChestComVelocity;
        public string AngularVelocityMode;
        public string DrivePhaseSyncMode;
        public string SourceVelocityBridgeMode;
        public int SourceVelocitySampleSequence;
        public bool SourceVelocitySampleFreshThisFixedStep;
        public float SourceVelocitySampleAgeSeconds;
        public Vector3 SampledTargetRelativeAngularVelocity;

        // v0.1.10: Render -> Physics Pelvis Pose Bridge diagnostics.
        public string PoseBridgeMode;
        public Vector3 PoseBridgeCommandLinearVelocity;
        public Vector3 PoseBridgeCommandAngularVelocity;
        public Vector3 PoseBridgePositionCorrectionVelocity;
        public Vector3 PoseBridgeRotationCorrectionAngularVelocity;
        public float PoseBridgePositionErrorMeters;
        public float PoseBridgeRotationErrorDegrees;
        public float PoseBridgePredictionAgeSeconds;
        public bool PoseBridgePositionCorrectionClamped;
        public bool PoseBridgeRotationCorrectionClamped;
        public int PoseBridgePositionCorrectionClampCount;
        public int PoseBridgeRotationCorrectionClampCount;

        // v0.1.8: settled fixed-step motion measured at the beginning of FixedUpdate, before the next
        // kinematic parent command. This removes LateUpdate/FixedUpdate phase aliasing from v0.1.7.
        public Vector3 MeasuredPelvisLinearVelocity;
        public Vector3 MeasuredChestLinearVelocity;
        public Vector3 MeasuredPelvisAngularVelocity;
        public Vector3 MeasuredChestAngularVelocity;
        public Vector3 MeasuredPelvisAnchorVelocity;
        public Vector3 MeasuredChestAnchorVelocity;
        public Vector3 MeasuredAnchorRelativeVelocity;
        public Vector3 MeasuredTransportReferenceVelocity;
        public Vector3 MeasuredChestTransportVelocityError;

        // v0.1.8: locomotion-phase classification. Classification is based on source Pelvis motion only.
        public string MotionPhase;
        public bool MotionTransition;
        public float MotionTargetSpeed;
        public float MotionSignedSpeedAcceleration;

        // v0.1.12: raw Render-clock source turn attribution. DIAGNOSTIC ONLY.
        public string SourceTurnDiagnosticsMode;
        public float SourceTurnSampleDeltaSeconds;
        public float SourceTurnSampleRotationDeltaDegrees;
        public float SourceTurnSampleYawDegrees;
        public float SourceTurnSampleYawDeltaDegrees;
        public Vector3 SourceTurnRawAngularVelocity;
        public float SourceTurnRawAngularSpeed;
        public Vector3 SourceTurnRawAngularAcceleration;
        public float SourceTurnRawAngularAccelerationMagnitude;
        public float SourceTurnChestRelativeRotationDeltaDegrees;
        public bool SourceTurnEventActive;
        public int SourceTurnCurrentEventId;
        public int SourceTurnEventCount;
        public int SourceTurnDiscontinuityEventCount;
        public int SourceTurnRapidContinuousEventCount;
        public float SourceTurnMaxSingleSampleRotationDeltaDegrees;
        public float SourceTurnMaxSingleSampleAbsYawDeltaDegrees;
        public float SourceTurnMaxRawAngularSpeed;
        public float SourceTurnMaxRawAngularAcceleration;
        public float SourceTurnMaxChestRelativeRotationDeltaDegrees;
        public float SourceTurnMaxResponseRotationErrorDegrees;
        public float SourceTurnMaxPoseBridgeRotationErrorDegrees;
        public float SourceTurnMaxPoseBridgeCommandAngularSpeed;
        public float SourceTurnMaxAnchorErrorMeters;
        public int SourceTurnAngularClampFixedStepCount;
        public string SourceTurnLastEventClassification;
        public float SourceTurnLastEventSignedYawDeltaDegrees;
        public float SourceTurnLastEventAbsoluteYawDeltaDegrees;
        public float SourceTurnLastEventMaxSingleSampleRotationDeltaDegrees;
        public float SourceTurnLastEventMaxRawAngularSpeed;
        public float SourceTurnLastEventMaxResponseRotationErrorDegrees;

        // v0.1.16: source-aware freeze validation + drive-loss fail-fast. DIAGNOSTIC/ACCEPTANCE ONLY.
        public string FreezeValidationMode;
        public string FreezeValidationPolicy;
        public bool FreezeValidationComplete;
        public bool NormalTrackingPass;
        public bool SourceDiscontinuitySurvivalPass;
        public bool RecoveryPass;
        public bool TwoBodyFreezeReady;
        public string FreezeValidationReason;
        public string FreezeValidationThresholds;
        public bool FreezeValidationTraceAborted;
        public string FreezeValidationAbortReason;
        public string FreezeValidationAbortStage;
        public string FreezeValidationAbortDetail;
        public int FreezeNormalSteadySampleCount;
        public float FreezeNormalSteadyRotationMeanDegrees;
        public float FreezeNormalSteadyRotationMaxDegrees;
        public float FreezeNormalSteadyChestPositionMeanMeters;
        public float FreezeNormalSteadyChestPositionMaxMeters;
        public float FreezeNormalSteadyAnchorRelativeSpeedMean;
        public float FreezeNormalSteadyAnchorRelativeSpeedMax;
        public int FreezeNormalTurnSampleCount;
        public float FreezeNormalTurnRotationMeanDegrees;
        public float FreezeNormalTurnRotationMaxDegrees;
        public float FreezeNormalTurnAnchorRelativeSpeedMean;
        public float FreezeNormalTurnAnchorRelativeSpeedMax;
        public int FreezeNormalAngularClampStepCount;
        public int FreezeNormalParentFeedforwardClampStepCount;
        public int FreezeSourceEventFixedStepCount;
        public int FreezeSourceInvalidStateStepCount;
        public int FreezeSourceParentFeedforwardClampStepCount;
        public float FreezeSourceMaxAnchorErrorMeters;
        public bool FreezeRecoveryActive;
        public int FreezeRecoveryStartedCount;
        public int FreezeRecoveryCompletedCount;
        public int FreezeRecoveryPassedCount;
        public int FreezeRecoveryFailedCount;
        public int FreezeRecoveryCensoredCount;
        public int FreezeRecoveryCensoredByNewSourceEventCount;
        public int FreezeRecoveryCensoredByTraceStopCount;
        public int FreezeSourceEventOpenAtTraceStopCount;
        public int FreezeRecoveryMinCompletedRequired;
        public int FreezeRecoveryInvalidStateStepCount;
        public int FreezeRecoveryParentFeedforwardClampStepCount;
        public int FreezeCensoredRecoveryInvalidStateStepCount;
        public int FreezeCensoredRecoveryParentFeedforwardClampStepCount;
        public float FreezeCensoredRecoveryMaxAnchorErrorMeters;
        // Canonical recovery timing is measured from SOURCE CLOSE.
        public float FreezeRecoveryMaxUnder5DegreesSecondsFromSourceClose;
        public float FreezeRecoveryMaxUnder2DegreesSecondsFromSourceClose;
        public float FreezeRecoveryMaxAnchorErrorMeters;
        public string FreezeLastRecoveryResult;
        // v0.1.15: diagnostic-only attribution for completed Recovery failures.
        public int FreezeRecoveryFailureAttributionCount;
        public string FreezeLastRecoveryAttribution;

        public string DriveBackend;
        public string ConstraintBackend;
        public bool CharacterJointPresent;
        public bool CharacterJointConnectedBodyMatch;
        public float ChestLowTwistLimitDegrees;
        public float ChestHighTwistLimitDegrees;
        public float ChestSwing1LimitDegrees;
        public float ChestSwing2LimitDegrees;
        public string JointLimitMode;
        public float JointCurrentTwistDegrees;
        public float JointCurrentSwing1Degrees;
        public float JointCurrentSwing2Degrees;
        public float JointTargetTwistDegrees;
        public float JointTargetSwing1Degrees;
        public float JointTargetSwing2Degrees;
        public float JointLimitProximity;
        public float TargetJointLimitProximity;
        public float PeakJointLimitProximity;
        public bool AngularAccelerationClamped;
        public int AngularAccelerationClampCount;
        public float RawAngularAccelerationMagnitude;
        // v0.3.11 read-only controller/CharacterJoint solver reaction taps.
        // v0.3.15 read-only: exact same previous-settled reaction sample as currentForce.
        // v0.3.14: native CharacterJoint constraint-anchor geometry witness.
        // This is distinct from AnchorErrorMeters, which retains the frozen source/control anchor semantics.
        public string CharacterJointAnchorMode;
        public bool CharacterJointAnchorModeValidated;
        public float CharacterJointConstraintAnchorErrorMeters;
        public float CharacterJointChestComLeverMeters;
        public bool ParentMotionFeedforwardEnabled;
        public bool ParentFeedforwardAccelerationClamped;
        public int ParentFeedforwardAccelerationClampCount;
        public float RawParentFeedforwardAccelerationMagnitude;
        public Vector3 ParentFeedforwardAcceleration;
        public Vector3 TargetConnectedAnchorVelocity;
        public Vector3 ActualConnectedAnchorVelocity;
        public Vector3 ActualChestAnchorVelocity;
        public Vector3 AnchorVelocityError;
        public Vector3 LastLinearAcceleration;
        public Vector3 LastAngularAcceleration;
        public float PeakChestPositionErrorMeters;
        public float PeakRotationErrorDegrees;
        public float PeakAnchorErrorMeters;
        public float PeakLinearAcceleration;
        public float PeakAngularAcceleration;

        public bool TraceEnabled;
        public int TraceSteadySampleCount;
        public int TraceTransitionSampleCount;
        public int TraceIdleSampleCount;
        public float TraceSteadyRotationErrorMeanDegrees;
        public float TraceSteadyRotationErrorMaxDegrees;
        public float TraceTransitionRotationErrorMeanDegrees;
        public float TraceTransitionRotationErrorMaxDegrees;
        public float TraceSteadyChestPositionErrorMeanMeters;
        public float TraceSteadyChestPositionErrorMaxMeters;
        public float TraceTransitionChestPositionErrorMeanMeters;
        public float TraceTransitionChestPositionErrorMaxMeters;
        public float TraceSteadyMeasuredTransportErrorMean;
        public float TraceSteadyMeasuredTransportErrorMax;
        public float TraceTransitionMeasuredTransportErrorMean;
        public float TraceTransitionMeasuredTransportErrorMax;
        public int TraceTurnSampleCount;
        public float TraceTurnRotationErrorMeanDegrees;
        public float TraceTurnRotationErrorMaxDegrees;
        public int TraceSteadyAngularClampSampleCount;
        public int TraceTransitionAngularClampSampleCount;
        public int TraceTurnAngularClampSampleCount;
        public float TraceSteadyAnchorRelativeSpeedMean;
        public float TraceSteadyAnchorRelativeSpeedMax;
        public float TraceTransitionAnchorRelativeSpeedMean;
        public float TraceTransitionAnchorRelativeSpeedMax;
        public float TraceTurnAnchorRelativeSpeedMean;
        public float TraceTurnAnchorRelativeSpeedMax;
        public int TraceFreshSourceVelocityFixedStepCount;
        public int TraceHeldSourceVelocityFixedStepCount;
        public float TraceSteadyParentTransportConsistencyMean;
        public float TraceSteadyParentTransportConsistencyMax;
        public float TraceTransitionParentTransportConsistencyMean;
        public float TraceTransitionParentTransportConsistencyMax;
        public float TraceTurnParentTransportConsistencyMean;
        public float TraceTurnParentTransportConsistencyMax;

        public int ContactEnterCount;
        public int ContactStayCount;
        public int ContactExitCount;
        public int IgnorePairCount;
        public int IgnoreFailureCount;
        public string LastContactBody;
        public string LastContactOther;
        public string BuildPhase;
        public string LastDetail;

        public string GetConsoleSummary()
        {
            return
                "ARL two-body proxy | built=" + Built +
                " | receiver=" + (PelvisReceiver != null && PelvisReceiver.Selected ? PelvisReceiver.GetSummary() : "LEGACY_IDLE") +
                " | drive=" + DriveEnabled +
                " | gravity=" + GravityEnabled +
                " | colliders=" + CollidersEnabled +
                " | visual=" + VisualEnabled +
                " | sourceAlive=" + SourceAlive +
                " | finite=" + FiniteState +
                " | target=" + (TargetName ?? "<none>") +
                " | torso=" + F(TorsoLength) +
                " | motion=" + (MotionPhase ?? "<none>") + "/transition=" + MotionTransition + "/speed=" + F(MotionTargetSpeed) + "/dSpeed=" + F(MotionSignedSpeedAcceleration) +
                " | sourceTurn=" + (SourceTurnDiagnosticsMode ?? "<none>") +
                "/sample=" + F(SourceTurnSampleDeltaSeconds) + "/rotDelta=" + F(SourceTurnSampleRotationDeltaDegrees) + "/yaw=" + F(SourceTurnSampleYawDegrees) + "/yawDelta=" + F(SourceTurnSampleYawDeltaDegrees) +
                "/rawW=" + F(SourceTurnRawAngularSpeed) + "/rawA=" + F(SourceTurnRawAngularAccelerationMagnitude) + "/relDelta=" + F(SourceTurnChestRelativeRotationDeltaDegrees) +
                "/event=" + SourceTurnEventActive + "/id=" + SourceTurnCurrentEventId +
                " | freeze=" + (FreezeValidationMode ?? "<none>") + "/complete=" + FreezeValidationComplete + "/normal=" + NormalTrackingPass + "/source=" + SourceDiscontinuitySurvivalPass + "/recovery=" + RecoveryPass + "/ready=" + TwoBodyFreezeReady + "/reason=" + (FreezeValidationReason ?? "<none>") +
                "/abort=" + FreezeValidationTraceAborted + "/" + (FreezeValidationAbortReason ?? "<none>") + "/stage=" + (FreezeValidationAbortStage ?? "<none>") +
                " | rotationErrorDeg=" + F(RotationErrorDegrees) +
                " | anchorError=" + F(AnchorErrorMeters) +
                " | pelvisPosError=" + F(PelvisPositionErrorMeters) +
                " | chestPosError=" + F(ChestPositionErrorMeters) +
                " | targetLinearVel=" + V(TargetChestLinearVelocity) +
                " | pelvisTargetVel=" + V(TargetPelvisLinearVelocity) +
                " | pelvisTargetAcc=" + V(TargetPelvisLinearAcceleration) +
                " | pelvisTargetAngularVel=" + V(TargetPelvisAngularVelocity) +
                " | pelvisTargetAngularAcc=" + V(TargetPelvisAngularAcceleration) +
                " | parentTransportVel=" + V(ParentTransportLinearVelocity) +
                " | physicsParentTransportVel=" + V(PhysicsParentTransportLinearVelocity) +
                " | transportConsistency=" + V(ParentTransportConsistencyError) + "/mag=" + F(ParentTransportConsistencyErrorMagnitude) +
                " | parentFFRef=" + (ParentFeedforwardReferenceMode ?? "<none>") + "/vel=" + V(ParentFeedforwardReferenceVelocity) +
                " | fixedPelvisTransportVel=" + V(FixedStepPelvisTransportLinearVelocity) +
                " | fixedPelvisTransportAngularVel=" + V(FixedStepPelvisTransportAngularVelocity) +
                " | fixedTargetChestComVel=" + V(FixedStepTargetChestComVelocity) +
                " | angularVelocityMode=" + (AngularVelocityMode ?? "<none>") +
                " | drivePhaseSync=" + (DrivePhaseSyncMode ?? "<none>") +
                " | sourceVelocityBridge=" + (SourceVelocityBridgeMode ?? "<none>") +
                " | sourceSample=" + SourceVelocitySampleSequence + "/fresh=" + SourceVelocitySampleFreshThisFixedStep + "/age=" + F(SourceVelocitySampleAgeSeconds) +
                " | sampledTargetRelAngularVel=" + V(SampledTargetRelativeAngularVelocity) +
                " | poseBridge=" + (PoseBridgeMode ?? "<none>") +
                "/cmdV=" + V(PoseBridgeCommandLinearVelocity) +
                "/cmdW=" + V(PoseBridgeCommandAngularVelocity) +
                "/posCorrV=" + V(PoseBridgePositionCorrectionVelocity) +
                "/rotCorrW=" + V(PoseBridgeRotationCorrectionAngularVelocity) +
                "/err=" + F(PoseBridgePositionErrorMeters) + "/" + F(PoseBridgeRotationErrorDegrees) +
                "/age=" + F(PoseBridgePredictionAgeSeconds) +
                "/clamp=" + PoseBridgePositionCorrectionClamped + "/" + PoseBridgeRotationCorrectionClamped +
                "/count=" + PoseBridgePositionCorrectionClampCount + "/" + PoseBridgeRotationCorrectionClampCount +
                " | rigidbodyReportedChestVel=" + V(ChestVelocity) +
                " | measuredPelvisVel=" + V(MeasuredPelvisLinearVelocity) +
                " | measuredChestVel=" + V(MeasuredChestLinearVelocity) +
                " | measuredTransportRef=" + V(MeasuredTransportReferenceVelocity) +
                " | measuredTransportError=" + V(MeasuredChestTransportVelocityError) +
                " | measuredPelvisAnchorVel=" + V(MeasuredPelvisAnchorVelocity) +
                " | measuredChestAnchorVel=" + V(MeasuredChestAnchorVelocity) +
                " | measuredAnchorRelVel=" + V(MeasuredAnchorRelativeVelocity) +
                " | targetRelAngularVel=" + V(TargetRelativeAngularVelocity) +
                " | bodyRelAngularVel=" + V(BodyRelativeAngularVelocity) +
                " | relAngularVelError=" + V(RelativeAngularVelocityError) +
                " | driveBackend=" + (DriveBackend ?? "<none>") +
                " | constraintBackend=" + (ConstraintBackend ?? "<none>") +
                " | characterJoint=" + CharacterJointPresent +
                " | connectedBodyMatch=" + CharacterJointConnectedBodyMatch +
                " | jointLimitMode=" + (JointLimitMode ?? "<none>") +
                " | chestLimits=" + F(ChestLowTwistLimitDegrees) + ".." + F(ChestHighTwistLimitDegrees) + "/" + F(ChestSwing1LimitDegrees) + "/" + F(ChestSwing2LimitDegrees) +
                " | jointAngles(cur)=" + F(JointCurrentTwistDegrees) + "/" + F(JointCurrentSwing1Degrees) + "/" + F(JointCurrentSwing2Degrees) +
                " | jointAngles(target)=" + F(JointTargetTwistDegrees) + "/" + F(JointTargetSwing1Degrees) + "/" + F(JointTargetSwing2Degrees) +
                " | limitProximity=" + F(JointLimitProximity) + "/" + F(TargetJointLimitProximity) + "/peak=" + F(PeakJointLimitProximity) +
                " | angularClamp=" + AngularAccelerationClamped + "/count=" + AngularAccelerationClampCount + "/rawMag=" + F(RawAngularAccelerationMagnitude) +
                " | parentFF=" + ParentMotionFeedforwardEnabled + "/clamp=" + ParentFeedforwardAccelerationClamped + "/count=" + ParentFeedforwardAccelerationClampCount + "/raw=" + F(RawParentFeedforwardAccelerationMagnitude) +
                " | parentFFAccel=" + V(ParentFeedforwardAcceleration) +
                " | legacyAnchorVelError=" + V(AnchorVelocityError) +
                " | linearAccel=" + V(LastLinearAcceleration) +
                " | angularAccel=" + V(LastAngularAcceleration) +
                " | peaks(pos/rot/anchor/linA/angA)=" + F(PeakChestPositionErrorMeters) + "/" + F(PeakRotationErrorDegrees) + "/" + F(PeakAnchorErrorMeters) + "/" + F(PeakLinearAcceleration) + "/" + F(PeakAngularAcceleration) +
                " | trace=" + TraceEnabled +
                " | traceStats(S/T/I)=" + TraceSteadySampleCount + "/" + TraceTransitionSampleCount + "/" + TraceIdleSampleCount +
                " | contacts=" + ContactEnterCount + "/" + ContactStayCount + "/" + ContactExitCount +
                " | ignored=" + IgnorePairCount +
                " | ignoreFailures=" + IgnoreFailureCount +
                " | buildPhase=" + (BuildPhase ?? "<none>") +
                " | detail=" + (LastDetail ?? "<none>");
        }

        public string GetTraceSummary()
        {
            return
                "built=" + Built +
                " | drive=" + DriveEnabled +
                " | finite=" + FiniteState +
                " | phase=" + (MotionPhase ?? "<none>") +
                " | transition=" + MotionTransition +
                " | sourceTurnSample=dt" + F(SourceTurnSampleDeltaSeconds) + "/rot" + F(SourceTurnSampleRotationDeltaDegrees) + "/yaw" + F(SourceTurnSampleYawDegrees) + "/dYaw" + F(SourceTurnSampleYawDeltaDegrees) + "/rawW" + F(SourceTurnRawAngularSpeed) + "/rawA" + F(SourceTurnRawAngularAccelerationMagnitude) + "/rel" + F(SourceTurnChestRelativeRotationDeltaDegrees) + "/event" + SourceTurnEventActive + "/id" + SourceTurnCurrentEventId +
                " | targetSpeed=" + F(MotionTargetSpeed) +
                " | signedSpeedAccel=" + F(MotionSignedSpeedAcceleration) +
                " | pelvisTargetAcc=" + V(TargetPelvisLinearAcceleration) +
                " | pelvisTargetAngularAcc=" + V(TargetPelvisAngularAcceleration) +
                " | chestPosError=" + F(ChestPositionErrorMeters) +
                " | rotationErrorDeg=" + F(RotationErrorDegrees) +
                " | anchorError=" + F(AnchorErrorMeters) +
                " | parentTransportVel=" + V(ParentTransportLinearVelocity) +
                " | physicsParentTransportVel=" + V(PhysicsParentTransportLinearVelocity) +
                " | transportConsistency=" + V(ParentTransportConsistencyError) + "/mag=" + F(ParentTransportConsistencyErrorMagnitude) +
                " | parentFFRef=" + (ParentFeedforwardReferenceMode ?? "<none>") + "/vel=" + V(ParentFeedforwardReferenceVelocity) +
                " | fixedPelvisTransportVel=" + V(FixedStepPelvisTransportLinearVelocity) +
                " | fixedPelvisTransportAngularVel=" + V(FixedStepPelvisTransportAngularVelocity) +
                " | fixedTargetChestComVel=" + V(FixedStepTargetChestComVelocity) +
                " | angularVelocityMode=" + (AngularVelocityMode ?? "<none>") +
                " | drivePhaseSync=" + (DrivePhaseSyncMode ?? "<none>") +
                " | sourceVelocityBridge=" + (SourceVelocityBridgeMode ?? "<none>") +
                " | sourceSample=" + SourceVelocitySampleSequence + "/fresh=" + SourceVelocitySampleFreshThisFixedStep + "/age=" + F(SourceVelocitySampleAgeSeconds) +
                " | sampledTargetRelAngularVel=" + V(SampledTargetRelativeAngularVelocity) +
                " | poseBridge=" + (PoseBridgeMode ?? "<none>") +
                "/cmdV=" + V(PoseBridgeCommandLinearVelocity) +
                "/cmdW=" + V(PoseBridgeCommandAngularVelocity) +
                "/posCorrV=" + V(PoseBridgePositionCorrectionVelocity) +
                "/rotCorrW=" + V(PoseBridgeRotationCorrectionAngularVelocity) +
                "/err=" + F(PoseBridgePositionErrorMeters) + "/" + F(PoseBridgeRotationErrorDegrees) +
                "/age=" + F(PoseBridgePredictionAgeSeconds) +
                "/clamp=" + PoseBridgePositionCorrectionClamped + "/" + PoseBridgeRotationCorrectionClamped +
                "/count=" + PoseBridgePositionCorrectionClampCount + "/" + PoseBridgeRotationCorrectionClampCount +
                " | rigidbodyReportedChestVel=" + V(ChestVelocity) +
                " | measuredPelvisVel=" + V(MeasuredPelvisLinearVelocity) +
                " | measuredChestVel=" + V(MeasuredChestLinearVelocity) +
                " | measuredTransportRef=" + V(MeasuredTransportReferenceVelocity) +
                " | measuredTransportError=" + V(MeasuredChestTransportVelocityError) +
                " | measuredPelvisAnchorVel=" + V(MeasuredPelvisAnchorVelocity) +
                " | measuredChestAnchorVel=" + V(MeasuredChestAnchorVelocity) +
                " | measuredAnchorRelVel=" + V(MeasuredAnchorRelativeVelocity) +
                " | targetRelAngularVel=" + V(TargetRelativeAngularVelocity) +
                " | bodyRelAngularVel=" + V(BodyRelativeAngularVelocity) +
                " | relAngularVelError=" + V(RelativeAngularVelocityError) +
                " | parentFF=" + ParentMotionFeedforwardEnabled + "/" + V(ParentFeedforwardAcceleration) +
                "/clamp=" + ParentFeedforwardAccelerationClamped + "/count=" + ParentFeedforwardAccelerationClampCount + "/raw=" + F(RawParentFeedforwardAccelerationMagnitude) +
                " | angularAccel=" + V(LastAngularAcceleration) +
                " | jointAngles=" + F(JointCurrentTwistDegrees) + "/" + F(JointCurrentSwing1Degrees) + "/" + F(JointCurrentSwing2Degrees) +
                "->" + F(JointTargetTwistDegrees) + "/" + F(JointTargetSwing1Degrees) + "/" + F(JointTargetSwing2Degrees) +
                " | limitProximity=" + F(JointLimitProximity) + "/" + F(TargetJointLimitProximity) +
                " | angularClamp=" + AngularAccelerationClamped + "/" + AngularAccelerationClampCount + "/raw=" + F(RawAngularAccelerationMagnitude) +
                " | constraint=" + (ConstraintBackend ?? "<none>") +
                " | peaks=" + F(PeakChestPositionErrorMeters) + "/" + F(PeakRotationErrorDegrees) + "/" + F(PeakAnchorErrorMeters);
        }


        public string GetTraceSummaryCompact()
        {
            return
                "finite=" + FiniteState +
                " | phase=" + (MotionPhase ?? "<none>") + "/transition=" + MotionTransition +
                " | targetSpeed=" + F(MotionTargetSpeed) +
                " | rotErr=" + F(RotationErrorDegrees) +
                " | chestPosErr=" + F(ChestPositionErrorMeters) +
                " | " + (PelvisReceiver != null && PelvisReceiver.Selected ? "legacyVirtualAnchorErr=" : "anchorErr=") + F(AnchorErrorMeters) +
                " | sourceTurn=" + SourceTurnEventActive + "/id=" + SourceTurnCurrentEventId +
                    "/sampleRot=" + F(SourceTurnSampleRotationDeltaDegrees) + "/rawW=" + F(SourceTurnRawAngularSpeed) +
                " | legacyFreezeRecoveryActive=" + FreezeRecoveryActive +
                " | controlRecoveryActive=" + ControlRecoveryActive + "/dynamicPolicy=" + UseDynamicTorsoRecovery +
                " | poseBridgeErr=" + F(PoseBridgeRotationErrorDegrees) + "/cmdW=" + F(PoseBridgeCommandAngularVelocity.magnitude) +
                " | transportConsistency=" + F(ParentTransportConsistencyErrorMagnitude) +
                " | angularClamp=" + AngularAccelerationClamped + "/count=" + AngularAccelerationClampCount +
                " | parentFFClamp=" + ParentFeedforwardAccelerationClamped + "/count=" + ParentFeedforwardAccelerationClampCount +
                (PelvisReceiver != null && PelvisReceiver.Selected ? " | receiverTracking=" + PelvisReceiver.GetTrackingCompact() : "");
        }

        public string GetTraceSessionSummary()
        {
            return
                "parentFFReference=" + (ParentFeedforwardReferenceMode ?? "<none>") +
                " | steadySamples=" + TraceSteadySampleCount +
                " | transitionSamples=" + TraceTransitionSampleCount +
                " | idleSamples=" + TraceIdleSampleCount +
                " | steadyRot(mean/max)=" + F(TraceSteadyRotationErrorMeanDegrees) + "/" + F(TraceSteadyRotationErrorMaxDegrees) +
                " | transitionRot(mean/max)=" + F(TraceTransitionRotationErrorMeanDegrees) + "/" + F(TraceTransitionRotationErrorMaxDegrees) +
                " | steadyPos(mean/max)=" + F(TraceSteadyChestPositionErrorMeanMeters) + "/" + F(TraceSteadyChestPositionErrorMaxMeters) +
                " | transitionPos(mean/max)=" + F(TraceTransitionChestPositionErrorMeanMeters) + "/" + F(TraceTransitionChestPositionErrorMaxMeters) +
                " | steadyMeasuredTransportErr(mean/max)=" + F(TraceSteadyMeasuredTransportErrorMean) + "/" + F(TraceSteadyMeasuredTransportErrorMax) +
                " | transitionMeasuredTransportErr(mean/max)=" + F(TraceTransitionMeasuredTransportErrorMean) + "/" + F(TraceTransitionMeasuredTransportErrorMax) +
                " | turnSamples=" + TraceTurnSampleCount +
                " | turnRot(mean/max)=" + F(TraceTurnRotationErrorMeanDegrees) + "/" + F(TraceTurnRotationErrorMaxDegrees) +
                " | clampSamples(S/T/TURN)=" + TraceSteadyAngularClampSampleCount + "/" + TraceTransitionAngularClampSampleCount + "/" + TraceTurnAngularClampSampleCount +
                " | anchorRelSpeed(S mean/max)=" + F(TraceSteadyAnchorRelativeSpeedMean) + "/" + F(TraceSteadyAnchorRelativeSpeedMax) +
                " | anchorRelSpeed(T mean/max)=" + F(TraceTransitionAnchorRelativeSpeedMean) + "/" + F(TraceTransitionAnchorRelativeSpeedMax) +
                " | anchorRelSpeed(TURN mean/max)=" + F(TraceTurnAnchorRelativeSpeedMean) + "/" + F(TraceTurnAnchorRelativeSpeedMax) +
                " | sourceBridgeFixedSteps(fresh/held)=" + TraceFreshSourceVelocityFixedStepCount + "/" + TraceHeldSourceVelocityFixedStepCount +
                " | parentTransportConsistency(S mean/max)=" + F(TraceSteadyParentTransportConsistencyMean) + "/" + F(TraceSteadyParentTransportConsistencyMax) +
                " | parentTransportConsistency(T mean/max)=" + F(TraceTransitionParentTransportConsistencyMean) + "/" + F(TraceTransitionParentTransportConsistencyMax) +
                " | parentTransportConsistency(TURN mean/max)=" + F(TraceTurnParentTransportConsistencyMean) + "/" + F(TraceTurnParentTransportConsistencyMax) +
                " | sourceTurnEvents(total/discontinuity/rapidContinuous)=" + SourceTurnEventCount + "/" + SourceTurnDiscontinuityEventCount + "/" + SourceTurnRapidContinuousEventCount +
                " | sourceTurnMax(sampleRot/yaw/rawW/rawA/relDelta)=" + F(SourceTurnMaxSingleSampleRotationDeltaDegrees) + "/" + F(SourceTurnMaxSingleSampleAbsYawDeltaDegrees) + "/" + F(SourceTurnMaxRawAngularSpeed) + "/" + F(SourceTurnMaxRawAngularAcceleration) + "/" + F(SourceTurnMaxChestRelativeRotationDeltaDegrees) +
                " | sourceTurnResponseMax(rot/poseRot/cmdW/anchor)=" + F(SourceTurnMaxResponseRotationErrorDegrees) + "/" + F(SourceTurnMaxPoseBridgeRotationErrorDegrees) + "/" + F(SourceTurnMaxPoseBridgeCommandAngularSpeed) + "/" + F(SourceTurnMaxAnchorErrorMeters) +
                " | sourceTurnClampFixedSteps=" + SourceTurnAngularClampFixedStepCount +
                " | sourceTurnLast=" + (SourceTurnLastEventClassification ?? "<none>") + "/yaw=" + F(SourceTurnLastEventSignedYawDeltaDegrees) + "/absYaw=" + F(SourceTurnLastEventAbsoluteYawDeltaDegrees) + "/maxSample=" + F(SourceTurnLastEventMaxSingleSampleRotationDeltaDegrees) + "/rawW=" + F(SourceTurnLastEventMaxRawAngularSpeed) + "/responseRot=" + F(SourceTurnLastEventMaxResponseRotationErrorDegrees) +
                " | freezeValidation=" + (FreezeValidationPolicy ?? "<none>") + "/complete=" + FreezeValidationComplete + "/normal=" + NormalTrackingPass + "/source=" + SourceDiscontinuitySurvivalPass + "/recovery=" + RecoveryPass + "/ready=" + TwoBodyFreezeReady + "/reason=" + (FreezeValidationReason ?? "<none>") +
                " | freezeNormalSteady(samples/rotMean/rotMax/posMean/anchorMean/anchorMax)=" + FreezeNormalSteadySampleCount + "/" + F(FreezeNormalSteadyRotationMeanDegrees) + "/" + F(FreezeNormalSteadyRotationMaxDegrees) + "/" + F(FreezeNormalSteadyChestPositionMeanMeters) + "/" + F(FreezeNormalSteadyAnchorRelativeSpeedMean) + "/" + F(FreezeNormalSteadyAnchorRelativeSpeedMax) +
                " | freezeNormalTurn(samples/rotMean/rotMax/anchorMean/anchorMax)=" + FreezeNormalTurnSampleCount + "/" + F(FreezeNormalTurnRotationMeanDegrees) + "/" + F(FreezeNormalTurnRotationMaxDegrees) + "/" + F(FreezeNormalTurnAnchorRelativeSpeedMean) + "/" + F(FreezeNormalTurnAnchorRelativeSpeedMax) +
                " | freezeNormalClamp(angular/parentFF)=" + FreezeNormalAngularClampStepCount + "/" + FreezeNormalParentFeedforwardClampStepCount +
                " | freezeSource(fixed/invalid/parentFFClamp/maxAnchor)=" + FreezeSourceEventFixedStepCount + "/" + FreezeSourceInvalidStateStepCount + "/" + FreezeSourceParentFeedforwardClampStepCount + "/" + F(FreezeSourceMaxAnchorErrorMeters) +
                " | freezeRecovery(start/completed/pass/fail/censored/newEvent/traceStop/max5FromClose/max2FromClose/maxAnchor)=" +
                    FreezeRecoveryStartedCount + "/" + FreezeRecoveryCompletedCount + "/" + FreezeRecoveryPassedCount + "/" + FreezeRecoveryFailedCount + "/" +
                    FreezeRecoveryCensoredCount + "/" + FreezeRecoveryCensoredByNewSourceEventCount + "/" + FreezeRecoveryCensoredByTraceStopCount + "/" +
                    F(FreezeRecoveryMaxUnder5DegreesSecondsFromSourceClose) + "/" + F(FreezeRecoveryMaxUnder2DegreesSecondsFromSourceClose) + "/" + F(FreezeRecoveryMaxAnchorErrorMeters) +
                " | angularClampCount=" + AngularAccelerationClampCount +
                " | parentFFClampCount=" + ParentFeedforwardAccelerationClampCount +
                " | poseBridgeClampCount(pos/rot)=" + PoseBridgePositionCorrectionClampCount + "/" + PoseBridgeRotationCorrectionClampCount +
                " | peakRotation=" + F(PeakRotationErrorDegrees) +
                " | peakChestPos=" + F(PeakChestPositionErrorMeters) +
                " | peakAnchor=" + F(PeakAnchorErrorMeters) +
                " | recoveryFailureAttributionCount=" + FreezeRecoveryFailureAttributionCount +
                " | traceAbort=" + FreezeValidationTraceAborted + "/" + (FreezeValidationAbortReason ?? "<none>") + "/stage=" + (FreezeValidationAbortStage ?? "<none>");
        }

        public string GetFullText()
        {
            StringBuilder sb = new StringBuilder(4096);
            sb.AppendLine("MakemitAGA Active Ragdoll Lab 0.4.2.1 - Frozen Two-Body Foundation Report (validated dynamics baseline retained)");
            sb.AppendLine("built=" + Built);
            sb.AppendLine("driveEnabled=" + DriveEnabled);
            sb.AppendLine("gravityEnabled=" + GravityEnabled);
            sb.AppendLine("collidersEnabled=" + CollidersEnabled);
            sb.AppendLine("visualEnabled=" + VisualEnabled);
            sb.AppendLine("sourceAlive=" + SourceAlive);
            sb.AppendLine("finiteState=" + FiniteState);
            sb.AppendLine("targetName=" + (TargetName ?? "<none>"));
            sb.AppendLine("targetPath=" + (TargetPath ?? "<none>"));
            sb.AppendLine("rootName=" + (RootName ?? "<none>"));
            sb.AppendLine("torsoLength=" + F(TorsoLength));
            sb.AppendLine("pelvisSize=" + V(PelvisSize));
            sb.AppendLine("chestSize=" + V(ChestSize));
            sb.AppendLine("motionPhase=" + (MotionPhase ?? "<none>"));
            sb.AppendLine("motionTransition=" + MotionTransition);
            sb.AppendLine("motionTargetSpeed=" + F(MotionTargetSpeed));
            sb.AppendLine("motionSignedSpeedAcceleration=" + F(MotionSignedSpeedAcceleration));
            sb.AppendLine("sourceTurnDiagnosticsMode=" + (SourceTurnDiagnosticsMode ?? "<none>"));
            sb.AppendLine("sourceTurnSampleDeltaSeconds=" + F(SourceTurnSampleDeltaSeconds));
            sb.AppendLine("sourceTurnSampleRotationDeltaDegrees=" + F(SourceTurnSampleRotationDeltaDegrees));
            sb.AppendLine("sourceTurnSampleYawDegrees=" + F(SourceTurnSampleYawDegrees));
            sb.AppendLine("sourceTurnSampleYawDeltaDegrees=" + F(SourceTurnSampleYawDeltaDegrees));
            sb.AppendLine("sourceTurnRawAngularVelocity=" + V(SourceTurnRawAngularVelocity));
            sb.AppendLine("sourceTurnRawAngularSpeed=" + F(SourceTurnRawAngularSpeed));
            sb.AppendLine("sourceTurnRawAngularAcceleration=" + V(SourceTurnRawAngularAcceleration));
            sb.AppendLine("sourceTurnRawAngularAccelerationMagnitude=" + F(SourceTurnRawAngularAccelerationMagnitude));
            sb.AppendLine("sourceTurnChestRelativeRotationDeltaDegrees=" + F(SourceTurnChestRelativeRotationDeltaDegrees));
            sb.AppendLine("sourceTurnEventActive=" + SourceTurnEventActive);
            sb.AppendLine("sourceTurnCurrentEventId=" + SourceTurnCurrentEventId);
            sb.AppendLine("sourceTurnEventCount=" + SourceTurnEventCount);
            sb.AppendLine("sourceTurnDiscontinuityEventCount=" + SourceTurnDiscontinuityEventCount);
            sb.AppendLine("sourceTurnRapidContinuousEventCount=" + SourceTurnRapidContinuousEventCount);
            sb.AppendLine("sourceTurnMaxSingleSampleRotationDeltaDegrees=" + F(SourceTurnMaxSingleSampleRotationDeltaDegrees));
            sb.AppendLine("sourceTurnMaxSingleSampleAbsYawDeltaDegrees=" + F(SourceTurnMaxSingleSampleAbsYawDeltaDegrees));
            sb.AppendLine("sourceTurnMaxRawAngularSpeed=" + F(SourceTurnMaxRawAngularSpeed));
            sb.AppendLine("sourceTurnMaxRawAngularAcceleration=" + F(SourceTurnMaxRawAngularAcceleration));
            sb.AppendLine("sourceTurnMaxChestRelativeRotationDeltaDegrees=" + F(SourceTurnMaxChestRelativeRotationDeltaDegrees));
            sb.AppendLine("sourceTurnMaxResponseRotationErrorDegrees=" + F(SourceTurnMaxResponseRotationErrorDegrees));
            sb.AppendLine("sourceTurnMaxPoseBridgeRotationErrorDegrees=" + F(SourceTurnMaxPoseBridgeRotationErrorDegrees));
            sb.AppendLine("sourceTurnMaxPoseBridgeCommandAngularSpeed=" + F(SourceTurnMaxPoseBridgeCommandAngularSpeed));
            sb.AppendLine("sourceTurnMaxAnchorErrorMeters=" + F(SourceTurnMaxAnchorErrorMeters));
            sb.AppendLine("sourceTurnAngularClampFixedStepCount=" + SourceTurnAngularClampFixedStepCount);
            sb.AppendLine("sourceTurnLastEventClassification=" + (SourceTurnLastEventClassification ?? "<none>"));
            sb.AppendLine("sourceTurnLastEventSignedYawDeltaDegrees=" + F(SourceTurnLastEventSignedYawDeltaDegrees));
            sb.AppendLine("sourceTurnLastEventAbsoluteYawDeltaDegrees=" + F(SourceTurnLastEventAbsoluteYawDeltaDegrees));
            sb.AppendLine("sourceTurnLastEventMaxSingleSampleRotationDeltaDegrees=" + F(SourceTurnLastEventMaxSingleSampleRotationDeltaDegrees));
            sb.AppendLine("sourceTurnLastEventMaxRawAngularSpeed=" + F(SourceTurnLastEventMaxRawAngularSpeed));
            sb.AppendLine("sourceTurnLastEventMaxResponseRotationErrorDegrees=" + F(SourceTurnLastEventMaxResponseRotationErrorDegrees));
            sb.AppendLine("freezeValidationMode=" + (FreezeValidationMode ?? "<none>"));
            sb.AppendLine("freezeValidationPolicy=" + (FreezeValidationPolicy ?? "<none>"));
            sb.AppendLine("freezeValidationComplete=" + FreezeValidationComplete);
            sb.AppendLine("normalTrackingPass=" + NormalTrackingPass);
            sb.AppendLine("sourceDiscontinuitySurvivalPass=" + SourceDiscontinuitySurvivalPass);
            sb.AppendLine("recoveryPass=" + RecoveryPass);
            sb.AppendLine("twoBodyFreezeReady=" + TwoBodyFreezeReady);
            sb.AppendLine("freezeValidationReason=" + (FreezeValidationReason ?? "<none>"));
            sb.AppendLine("freezeValidationThresholds=" + (FreezeValidationThresholds ?? "<none>"));
            sb.AppendLine("freezeValidationTraceAborted=" + FreezeValidationTraceAborted);
            sb.AppendLine("freezeValidationAbortReason=" + (FreezeValidationAbortReason ?? "<none>"));
            sb.AppendLine("freezeValidationAbortStage=" + (FreezeValidationAbortStage ?? "<none>"));
            sb.AppendLine("freezeValidationAbortDetail=" + (FreezeValidationAbortDetail ?? "<none>"));
            sb.AppendLine("freezeNormalSteadySampleCount=" + FreezeNormalSteadySampleCount);
            sb.AppendLine("freezeNormalSteadyRotationMeanDegrees=" + F(FreezeNormalSteadyRotationMeanDegrees));
            sb.AppendLine("freezeNormalSteadyRotationMaxDegrees=" + F(FreezeNormalSteadyRotationMaxDegrees));
            sb.AppendLine("freezeNormalSteadyChestPositionMeanMeters=" + F(FreezeNormalSteadyChestPositionMeanMeters));
            sb.AppendLine("freezeNormalSteadyChestPositionMaxMeters=" + F(FreezeNormalSteadyChestPositionMaxMeters));
            sb.AppendLine("freezeNormalSteadyAnchorRelativeSpeedMean=" + F(FreezeNormalSteadyAnchorRelativeSpeedMean));
            sb.AppendLine("freezeNormalSteadyAnchorRelativeSpeedMax=" + F(FreezeNormalSteadyAnchorRelativeSpeedMax));
            sb.AppendLine("freezeNormalTurnSampleCount=" + FreezeNormalTurnSampleCount);
            sb.AppendLine("freezeNormalTurnRotationMeanDegrees=" + F(FreezeNormalTurnRotationMeanDegrees));
            sb.AppendLine("freezeNormalTurnRotationMaxDegrees=" + F(FreezeNormalTurnRotationMaxDegrees));
            sb.AppendLine("freezeNormalTurnAnchorRelativeSpeedMean=" + F(FreezeNormalTurnAnchorRelativeSpeedMean));
            sb.AppendLine("freezeNormalTurnAnchorRelativeSpeedMax=" + F(FreezeNormalTurnAnchorRelativeSpeedMax));
            sb.AppendLine("freezeNormalAngularClampStepCount=" + FreezeNormalAngularClampStepCount);
            sb.AppendLine("freezeNormalParentFeedforwardClampStepCount=" + FreezeNormalParentFeedforwardClampStepCount);
            sb.AppendLine("freezeSourceEventFixedStepCount=" + FreezeSourceEventFixedStepCount);
            sb.AppendLine("freezeSourceInvalidStateStepCount=" + FreezeSourceInvalidStateStepCount);
            sb.AppendLine("freezeSourceParentFeedforwardClampStepCount=" + FreezeSourceParentFeedforwardClampStepCount);
            sb.AppendLine("freezeSourceMaxAnchorErrorMeters=" + F(FreezeSourceMaxAnchorErrorMeters));
            sb.AppendLine("freezeRecoveryActive=" + FreezeRecoveryActive);
            sb.AppendLine("controlTorsoRecovery(dynamicPolicy-active)=" + UseDynamicTorsoRecovery + "/" + ControlRecoveryActive);
            sb.AppendLine("freezeRecoveryStartedCount=" + FreezeRecoveryStartedCount);
            sb.AppendLine("freezeRecoveryCompletedCount=" + FreezeRecoveryCompletedCount);
            sb.AppendLine("freezeRecoveryPassedCount=" + FreezeRecoveryPassedCount);
            sb.AppendLine("freezeRecoveryFailedCount=" + FreezeRecoveryFailedCount);
            sb.AppendLine("freezeRecoveryCensoredCount=" + FreezeRecoveryCensoredCount);
            sb.AppendLine("freezeRecoveryCensoredByNewSourceEventCount=" + FreezeRecoveryCensoredByNewSourceEventCount);
            sb.AppendLine("freezeRecoveryCensoredByTraceStopCount=" + FreezeRecoveryCensoredByTraceStopCount);
            sb.AppendLine("freezeSourceEventOpenAtTraceStopCount=" + FreezeSourceEventOpenAtTraceStopCount);
            sb.AppendLine("freezeRecoveryMinCompletedRequired=" + FreezeRecoveryMinCompletedRequired);
            sb.AppendLine("freezeRecoveryInvalidStateStepCount=" + FreezeRecoveryInvalidStateStepCount);
            sb.AppendLine("freezeRecoveryParentFeedforwardClampStepCount=" + FreezeRecoveryParentFeedforwardClampStepCount);
            sb.AppendLine("freezeCensoredRecoveryInvalidStateStepCount=" + FreezeCensoredRecoveryInvalidStateStepCount);
            sb.AppendLine("freezeCensoredRecoveryParentFeedforwardClampStepCount=" + FreezeCensoredRecoveryParentFeedforwardClampStepCount);
            sb.AppendLine("freezeCensoredRecoveryMaxAnchorErrorMeters=" + F(FreezeCensoredRecoveryMaxAnchorErrorMeters));
            sb.AppendLine("freezeRecoveryMaxUnder5DegreesSecondsFromSourceClose=" + F(FreezeRecoveryMaxUnder5DegreesSecondsFromSourceClose));
            sb.AppendLine("freezeRecoveryMaxUnder2DegreesSecondsFromSourceClose=" + F(FreezeRecoveryMaxUnder2DegreesSecondsFromSourceClose));
            sb.AppendLine("freezeRecoveryMaxAnchorErrorMeters=" + F(FreezeRecoveryMaxAnchorErrorMeters));
            sb.AppendLine("freezeLastRecoveryResult=" + (FreezeLastRecoveryResult ?? "<none>"));
            sb.AppendLine("freezeRecoveryFailureAttributionCount=" + FreezeRecoveryFailureAttributionCount);
            sb.AppendLine("freezeLastRecoveryAttribution=" + (FreezeLastRecoveryAttribution ?? "<none>"));
            sb.AppendLine("rotationErrorDegrees=" + F(RotationErrorDegrees));
            sb.AppendLine("anchorErrorMeters=" + F(AnchorErrorMeters));
            sb.AppendLine("pelvisPositionErrorMeters=" + F(PelvisPositionErrorMeters));
            sb.AppendLine("chestPositionErrorMeters=" + F(ChestPositionErrorMeters));
            sb.AppendLine("targetChestLinearVelocity=" + V(TargetChestLinearVelocity));
            sb.AppendLine("targetChestAngularVelocity=" + V(TargetChestAngularVelocity));
            sb.AppendLine("targetPelvisLinearVelocity=" + V(TargetPelvisLinearVelocity));
            sb.AppendLine("targetPelvisAngularVelocity=" + V(TargetPelvisAngularVelocity));
            sb.AppendLine("targetChestLinearAcceleration=" + V(TargetChestLinearAcceleration));
            sb.AppendLine("targetChestAngularAcceleration=" + V(TargetChestAngularAcceleration));
            sb.AppendLine("targetPelvisLinearAcceleration=" + V(TargetPelvisLinearAcceleration));
            sb.AppendLine("targetPelvisAngularAcceleration=" + V(TargetPelvisAngularAcceleration));
            sb.AppendLine("pelvisVelocity=" + V(PelvisVelocity));
            sb.AppendLine("pelvisAngularVelocity=" + V(PelvisAngularVelocity));
            sb.AppendLine("estimatedPelvisLinearVelocity=" + V(EstimatedPelvisLinearVelocity));
            sb.AppendLine("estimatedPelvisAngularVelocity=" + V(EstimatedPelvisAngularVelocity));
            sb.AppendLine("parentTransportLinearVelocity=" + V(ParentTransportLinearVelocity));
            sb.AppendLine("physicsParentTransportLinearVelocity=" + V(PhysicsParentTransportLinearVelocity));
            sb.AppendLine("parentTransportConsistencyError=" + V(ParentTransportConsistencyError));
            sb.AppendLine("parentTransportConsistencyErrorMagnitude=" + F(ParentTransportConsistencyErrorMagnitude));
            sb.AppendLine("parentFeedforwardReferenceMode=" + (ParentFeedforwardReferenceMode ?? "<none>"));
            sb.AppendLine("parentFeedforwardReferenceVelocity=" + V(ParentFeedforwardReferenceVelocity));
            sb.AppendLine("sourceRelativeLinearVelocity=" + V(SourceRelativeLinearVelocity));
            sb.AppendLine("targetRelativeAngularVelocity=" + V(TargetRelativeAngularVelocity));
            sb.AppendLine("bodyRelativeAngularVelocity=" + V(BodyRelativeAngularVelocity));
            sb.AppendLine("relativeAngularVelocityError=" + V(RelativeAngularVelocityError));
            sb.AppendLine("chestVelocity=" + V(ChestVelocity));
            sb.AppendLine("chestAngularVelocity=" + V(ChestAngularVelocity));
            sb.AppendLine("fixedStepPelvisTransportLinearVelocity=" + V(FixedStepPelvisTransportLinearVelocity));
            sb.AppendLine("fixedStepPelvisTransportAngularVelocity=" + V(FixedStepPelvisTransportAngularVelocity));
            sb.AppendLine("fixedStepTargetChestComVelocity=" + V(FixedStepTargetChestComVelocity));
            sb.AppendLine("angularVelocityMode=" + (AngularVelocityMode ?? "<none>"));
            sb.AppendLine("drivePhaseSyncMode=" + (DrivePhaseSyncMode ?? "<none>"));
            sb.AppendLine("sourceVelocityBridgeMode=" + (SourceVelocityBridgeMode ?? "<none>"));
            sb.AppendLine("sourceVelocitySampleSequence=" + SourceVelocitySampleSequence);
            sb.AppendLine("sourceVelocitySampleFreshThisFixedStep=" + SourceVelocitySampleFreshThisFixedStep);
            sb.AppendLine("sourceVelocitySampleAgeSeconds=" + F(SourceVelocitySampleAgeSeconds));
            sb.AppendLine("poseBridgeMode=" + (PoseBridgeMode ?? "<none>"));
            sb.AppendLine("poseBridgeCommandLinearVelocity=" + V(PoseBridgeCommandLinearVelocity));
            sb.AppendLine("poseBridgeCommandAngularVelocity=" + V(PoseBridgeCommandAngularVelocity));
            sb.AppendLine("poseBridgePositionCorrectionVelocity=" + V(PoseBridgePositionCorrectionVelocity));
            sb.AppendLine("poseBridgeRotationCorrectionAngularVelocity=" + V(PoseBridgeRotationCorrectionAngularVelocity));
            sb.AppendLine("poseBridgePositionErrorMeters=" + F(PoseBridgePositionErrorMeters));
            sb.AppendLine("poseBridgeRotationErrorDegrees=" + F(PoseBridgeRotationErrorDegrees));
            sb.AppendLine("poseBridgePredictionAgeSeconds=" + F(PoseBridgePredictionAgeSeconds));
            sb.AppendLine("poseBridgePositionCorrectionClamped=" + PoseBridgePositionCorrectionClamped);
            sb.AppendLine("poseBridgeRotationCorrectionClamped=" + PoseBridgeRotationCorrectionClamped);
            sb.AppendLine("poseBridgePositionCorrectionClampCount=" + PoseBridgePositionCorrectionClampCount);
            sb.AppendLine("poseBridgeRotationCorrectionClampCount=" + PoseBridgeRotationCorrectionClampCount);
            sb.AppendLine("sampledTargetRelativeAngularVelocity=" + V(SampledTargetRelativeAngularVelocity));
            sb.AppendLine("measuredPelvisLinearVelocity=" + V(MeasuredPelvisLinearVelocity));
            sb.AppendLine("measuredChestLinearVelocity=" + V(MeasuredChestLinearVelocity));
            sb.AppendLine("measuredPelvisAngularVelocity=" + V(MeasuredPelvisAngularVelocity));
            sb.AppendLine("measuredChestAngularVelocity=" + V(MeasuredChestAngularVelocity));
            sb.AppendLine("measuredPelvisAnchorVelocity=" + V(MeasuredPelvisAnchorVelocity));
            sb.AppendLine("measuredChestAnchorVelocity=" + V(MeasuredChestAnchorVelocity));
            sb.AppendLine("measuredAnchorRelativeVelocity=" + V(MeasuredAnchorRelativeVelocity));
            sb.AppendLine("measuredTransportReferenceVelocity=" + V(MeasuredTransportReferenceVelocity));
            sb.AppendLine("measuredChestTransportVelocityError=" + V(MeasuredChestTransportVelocityError));
            sb.AppendLine("driveBackend=" + (DriveBackend ?? "<none>"));
            sb.AppendLine("constraintBackend=" + (ConstraintBackend ?? "<none>"));
            sb.AppendLine("characterJointPresent=" + CharacterJointPresent);
            sb.AppendLine("characterJointConnectedBodyMatch=" + CharacterJointConnectedBodyMatch);
            sb.AppendLine("chestLowTwistLimitDegrees=" + F(ChestLowTwistLimitDegrees));
            sb.AppendLine("chestHighTwistLimitDegrees=" + F(ChestHighTwistLimitDegrees));
            sb.AppendLine("chestSwing1LimitDegrees=" + F(ChestSwing1LimitDegrees));
            sb.AppendLine("chestSwing2LimitDegrees=" + F(ChestSwing2LimitDegrees));
            sb.AppendLine("jointLimitMode=" + (JointLimitMode ?? "<none>"));
            sb.AppendLine("characterJointAnchorMode=" + (CharacterJointAnchorMode ?? "<none>") + " | validated=" + CharacterJointAnchorModeValidated + " | constraintAnchorErrorMeters=" + F(CharacterJointConstraintAnchorErrorMeters) + " | chestComLeverMeters=" + F(CharacterJointChestComLeverMeters));
            sb.AppendLine("jointCurrentTwistDegrees=" + F(JointCurrentTwistDegrees));
            sb.AppendLine("jointCurrentSwing1Degrees=" + F(JointCurrentSwing1Degrees));
            sb.AppendLine("jointCurrentSwing2Degrees=" + F(JointCurrentSwing2Degrees));
            sb.AppendLine("jointTargetTwistDegrees=" + F(JointTargetTwistDegrees));
            sb.AppendLine("jointTargetSwing1Degrees=" + F(JointTargetSwing1Degrees));
            sb.AppendLine("jointTargetSwing2Degrees=" + F(JointTargetSwing2Degrees));
            sb.AppendLine("jointLimitProximity=" + F(JointLimitProximity));
            sb.AppendLine("targetJointLimitProximity=" + F(TargetJointLimitProximity));
            sb.AppendLine("peakJointLimitProximity=" + F(PeakJointLimitProximity));
            sb.AppendLine("angularAccelerationClamped=" + AngularAccelerationClamped);
            sb.AppendLine("angularAccelerationClampCount=" + AngularAccelerationClampCount);
            sb.AppendLine("rawAngularAccelerationMagnitude=" + F(RawAngularAccelerationMagnitude));
            sb.AppendLine("parentMotionFeedforwardEnabled=" + ParentMotionFeedforwardEnabled);
            sb.AppendLine("parentFeedforwardAccelerationClamped=" + ParentFeedforwardAccelerationClamped);
            sb.AppendLine("parentFeedforwardAccelerationClampCount=" + ParentFeedforwardAccelerationClampCount);
            sb.AppendLine("rawParentFeedforwardAccelerationMagnitude=" + F(RawParentFeedforwardAccelerationMagnitude));
            sb.AppendLine("parentFeedforwardAcceleration=" + V(ParentFeedforwardAcceleration));
            sb.AppendLine("targetConnectedAnchorVelocity=" + V(TargetConnectedAnchorVelocity));
            sb.AppendLine("actualConnectedAnchorVelocity=" + V(ActualConnectedAnchorVelocity));
            sb.AppendLine("actualChestAnchorVelocity=" + V(ActualChestAnchorVelocity));
            sb.AppendLine("legacyAnchorVelocityError=" + V(AnchorVelocityError));
            sb.AppendLine("lastLinearAcceleration=" + V(LastLinearAcceleration));
            sb.AppendLine("lastAngularAcceleration=" + V(LastAngularAcceleration));
            sb.AppendLine("peakChestPositionErrorMeters=" + F(PeakChestPositionErrorMeters));
            sb.AppendLine("peakRotationErrorDegrees=" + F(PeakRotationErrorDegrees));
            sb.AppendLine("peakAnchorErrorMeters=" + F(PeakAnchorErrorMeters));
            sb.AppendLine("peakLinearAcceleration=" + F(PeakLinearAcceleration));
            sb.AppendLine("peakAngularAcceleration=" + F(PeakAngularAcceleration));
            sb.AppendLine("traceEnabled=" + TraceEnabled);
            sb.AppendLine("traceSummary=" + GetTraceSessionSummary());
            sb.AppendLine("contactEnterCount=" + ContactEnterCount);
            sb.AppendLine("contactStayCount=" + ContactStayCount);
            sb.AppendLine("contactExitCount=" + ContactExitCount);
            sb.AppendLine("ignorePairCount=" + IgnorePairCount);
            sb.AppendLine("ignoreFailureCount=" + IgnoreFailureCount);
            sb.AppendLine("lastContactBody=" + (LastContactBody ?? "<none>"));
            sb.AppendLine("lastContactOther=" + (LastContactOther ?? "<none>"));
            if (PelvisReceiver != null) sb.AppendLine("pelvisReceiver=" + PelvisReceiver.GetDetails());
            sb.AppendLine("buildPhase=" + (BuildPhase ?? "<none>"));
            sb.AppendLine("detail=" + (LastDetail ?? "<none>"));
            return sb.ToString();
        }

        private static string F(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + ", " + F(value.y) + ", " + F(value.z) + ")";
    }
}
