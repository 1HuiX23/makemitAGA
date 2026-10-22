using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    /// <summary>
    /// v0.2.51 单腿链紧凑报告；保留 v0.2.50 linear transport，并新增 RAW→STRICT mechanical activation-sync 诊断。
    /// 只保留当前 real-inertia 主线仍会产生或消费的状态：Knee PCA、transport/axis attribution、A-FF、
    /// angular/phase timing、Hip frame/coupling read-only attribution、validated uniform-box mass properties、anchor/clamp/trace 与 fail-fast。
    /// 历史只读实验若已无生产者/消费者，只保留必要 closure 常量；详细字段留在旧 ZIP，不进入活动报告模型。
    /// </summary>
    internal sealed class SingleLegChainReport
    {
        public bool Built;
        public bool DriveEnabled;
        public bool GravityEnabled;
        public bool CollidersEnabled;
        public bool VisualEnabled;
        public bool Finite = true;
        public string Stage = "UNBUILT";
        public string Detail;
        public string Side;
        public string ResolutionMode;
        public string UpperLegPath;
        public string LowerLegPath;
        public string FootPath;
        public float UpperLegLength;
        public float LowerLegLength;
        public bool HipCharacterJointPresent;
        public bool HipConnectedBodyMatch;

        // v0.2.48 mechanical-parent + bounded transport A/B. Default experimental arm uses an ARL-owned kinematic Hip parent located
        // directly at the source Hip world anchor; rotation follows the validated HipControlFrame while active.
        public string HipMechanicalParentExperiment;
        public string HipMechanicalParentMode;
        public string HipMechanicalParentKey;
        public bool HipMechanicalParentProxyPresent;
        public bool HipMechanicalParentUsesInertializedProxy;
        public bool HipMechanicalParentConnectedToProxy;
        public int HipMechanicalParentObservedFixedSteps;
        public int HipMechanicalParentCommandedFixedSteps;
        public int HipMechanicalParentControlFrameCommandedFixedSteps;
        public float HipMechanicalParentCurrentPositionErrorMeters;
        public float HipMechanicalParentCurrentRawPelvisRotationLagDegrees;
        public float HipMechanicalParentCurrentControlFrameRotationErrorDegrees;
        public float HipMechanicalParentCurrentAngularSpeed;
        public float HipMechanicalParentCurrentAngularAcceleration;
        public float HipMechanicalParentCurrentRawPelvisAngularAcceleration;
        public float HipMechanicalParentPeakPositionErrorMeters;
        public float HipMechanicalParentPeakRawPelvisRotationLagDegrees;
        public float HipMechanicalParentPeakControlFrameRotationErrorDegrees;
        public float HipMechanicalParentPeakAngularSpeed;
        public float HipMechanicalParentPeakAngularAcceleration;
        public float HipMechanicalParentPeakRawPelvisAngularAcceleration;
        public float HipMechanicalParentCurrentCommandedAngularSpeed;
        public float HipMechanicalParentCurrentCommandedAngularAcceleration;
        public float HipMechanicalParentPeakCommandedAngularSpeed;
        public float HipMechanicalParentPeakCommandedAngularAcceleration;
        // v0.2.50 same-FixedUpdate kinematic Hip-anchor command derivative used only by bounded whole-leg linear transport.
        // v0.2.56 strict->proxy command closure audit.
        public float HipMechanicalParentCommandClosureAngularVelocityError;
        public float HipMechanicalParentCommandClosureAngularAccelerationError;
        public float HipMechanicalParentCommandClosureMaxAccelerationRatio;
        public bool HipMechanicalParentCommandClosureAccelerationContractSatisfied;
        // v0.2.60: pose-derived kinematic W/A is an audit channel, not the authoritative strict MaxA contract.
        public bool HipMechanicalParentPoseDerivedAuditWithinTolerance;
        public float HipMechanicalParentPoseDerivedAuditTolerance;
        public float HipMechanicalParentPoseDerivedCommandOverMaxA;
        public float HipMechanicalParentPeakCommandClosureAngularVelocityError;
        public float HipMechanicalParentPeakCommandClosureAngularAccelerationError;
        public float HipMechanicalParentPeakCommandClosureMaxAccelerationRatio;

        // v0.2.48 guard-decoupled strict mechanical-parent angular-acceleration transport FF. No authority increase: final SUM still
        // uses the historical Hip MaxA 40/80/80. REAL_PELVIS raw acceleration is intentionally never consumed.
        public string HipMechanicalParentTransportMode;
        public string HipMechanicalParentTransportKey;
        public bool HipMechanicalParentTransportEnabled;
        // v0.2.88: post-frame physical recovery support is explicit and common to both arms;
        // NormalTransportEnabled now means TRUE Healthy Normal transport only (A=OFF/B=ON).
        public string HipMechanicalParentNormalTransportArm;
        // Current fixed phase-ownership label for Healthy Normal vs dedicated Recovery transport.
        public string HipMechanicalParentNormalTransportScopeArm;
        public bool HipMechanicalParentRecoveryTransportEnabled;
        public bool HipMechanicalParentRecoveryTransportActiveThisStep;
        public bool HipMechanicalParentHealthyNormalTransportActiveThisStep;
        public bool HipMechanicalParentTransportActiveThisStep;
        public string HipMechanicalParentTransportActiveBranch;
        public string HipMechanicalParentTransportSuppressionReason;
        public float HipMechanicalParentTransportCurrentMagnitude;
        public float HipMechanicalParentTransportCurrentRelativeCorrectionMagnitude;
        public float HipMechanicalParentTransportCurrentCombinedRawMagnitude;
        public float HipMechanicalParentTransportCurrentCombinedAppliedMagnitude;
        public float HipMechanicalParentTransportCurrentMaxAngularAcceleration;
        public bool HipMechanicalParentTransportCombinedClamped;
        public int HipMechanicalParentTransportSteps;
        public int HipMechanicalParentTransportActiveSteps;
        public int HipMechanicalParentTransportStrictActiveSteps;
        public int HipMechanicalParentRecoveryTransportActiveSteps;
        public int HipMechanicalParentHealthyNormalEligibleSteps;
        public int HipMechanicalParentTransportNormalActiveSteps;
        public int HipMechanicalParentTransportNormalBypassSteps;
        public int HipMechanicalParentTransportScopeBlockedSteps;
        public int HipMechanicalParentTransportScopeBlockedSourceSteps;
        public int HipMechanicalParentTransportScopeBlockedRecoverySteps;
        public int HipMechanicalParentTransportScopeBlockedUnhealthySteps;
        public int HipMechanicalParentTransportDisabledSteps;
        public int HipMechanicalParentTransportRealParentSteps;
        public int HipMechanicalParentTransportInactiveFrameSteps;
        public int HipMechanicalParentTransportInvalidSteps;
        public int HipMechanicalParentTransportClampSteps;
        public float HipMechanicalParentTransportPeakMagnitude;
        public float HipMechanicalParentTransportPeakRelativeCorrectionMagnitude;
        public float HipMechanicalParentTransportPeakCombinedRawMagnitude;
        public float HipMechanicalParentTransportPeakCombinedAppliedMagnitude;
        public float HipMechanicalParentTransportPeakToMaxARatio;
        public float HipMechanicalParentTransportPeakCombinedRawToMaxARatio;
        public string HipMechanicalParentTransportLastEvidence;

        // v0.2.50 bounded whole-leg common-mode Hip-anchor linear transport. Same acceleration is applied at both
        // Upper/Lower Rigidbody COMs with ForceMode.Acceleration; no direct torque and no new profile knob.
        public string HipAnchorLinearTransportMode;
        public string HipAnchorLinearTransportKey;
        public string HipAnchorLinearTransportBoundSource;
        public bool HipAnchorLinearTransportEnabled;
        public bool HipAnchorLinearTransportActiveThisStep;
        // V04350 R2: cached writer output, not a reconstructed direction from the magnitude below.
        public int HipAnchorLinearTransportCurrentFixed;
        public Vector3 HipAnchorLinearTransportCurrentAppliedAccelerationWorld;
        public string HipAnchorLinearTransportSuppressionReason;
        public float HipAnchorLinearTransportCurrentRawAcceleration;
        public float HipAnchorLinearTransportCurrentAppliedAcceleration;
        public float HipAnchorLinearTransportCurrentMaxAcceleration;
        public bool HipAnchorLinearTransportClampedThisStep;
        public int HipAnchorLinearTransportSteps;
        public int HipAnchorLinearTransportActiveSteps;
        public int HipAnchorLinearTransportDisabledSteps;
        public int HipAnchorLinearTransportRealParentSteps;
        public int HipAnchorLinearTransportInactiveFrameSteps;
        public int HipAnchorLinearTransportInvalidSteps;
        public int HipAnchorLinearTransportClampSteps;
        public float HipAnchorLinearTransportPeakRawAcceleration;
        public float HipAnchorLinearTransportPeakAppliedAcceleration;
        public float HipAnchorLinearTransportPeakRawToBoundRatio;
        public string HipAnchorLinearTransportLastEvidence;

        public bool KneeCharacterJointPresent;
        public bool KneeConnectedBodyMatch;
        public Vector3 KneeFlexionAxisWorld;
        public Vector3 KneeFlexionAxisLowerLocal;
        public string KneeAxisMode;
        public string KneeConstraintBackendMode;
        public float KneeLowTwistLimitReadbackDegrees;
        public float KneeHighTwistLimitReadbackDegrees;
        public float KneeSwing1LimitReadbackDegrees;
        public float KneeSwing2LimitReadbackDegrees;

        // v0.2.2 source-only PCA calibration. Only explicit apply can change the ARL-owned Knee CharacterJoint flexion axis.
        public bool KneeAxisCalibrationCollecting;
        public bool KneeAxisCalibrationReady;
        public string KneeAxisCalibrationReason;
        public int KneeAxisCalibrationSampleCount;
        public float KneeAxisCalibrationPrincipalEnergyRatio;
        public float KneeAxisCalibrationAngleFromInitialDegrees;

        public float HipRotationErrorDegrees;

        // v0.2.29 phase-aware Hip Source-Turn / recovery MaxA scheduling. Baseline LegHipMaxAngularAcceleration stays 40.
        public string HipAuthorityMode;
        public float HipBaselineMaxAngularAcceleration;
        public float HipSourceTurnMaxAngularAcceleration;
        public float HipRecoveryMaxAngularAcceleration;
        public string HipAuthorityPhase;
        public float HipAuthorityCurrentMaxAngularAcceleration;
        public float HipAuthorityRecoveryErrorDegrees;
        public bool HipAuthorityBoostActive;
        public bool HipPhysicalRecoveryPending;
        public bool HipRecoveryBoostActive;
        public string HipPhysicalRecoveryOrigin;
        public bool HipTrackingHealthUnhealthy;
        public float HipTrackingHealthCandidateSeconds;
        public int HipTrackingHealthReentryCount;
        public int HipTrackingHealthImmediateReentryCount;
        public float HipTrackingHealthPeakEntryErrorDegrees;
        public int HipAuthorityBaselineFixedSteps;
        public int HipAuthoritySourceBoostFixedSteps;
        public int HipAuthorityRecoveryBoostFixedSteps;
        public int HipAuthorityPhysicalRecoveryPendingBaselineFixedSteps;
        public int HipAuthorityTrackingUnhealthyBaselineFixedSteps;
        public int HipAuthorityBaselineClampSamples;
        public int HipAuthoritySourceClampSamples;
        public int HipAuthorityRecoveryClampSamples;
        public int HipAuthorityRecoveryStarted;
        public int HipAuthorityRecoveryCompleted;
        public int HipAuthorityRecoveryTimedOut;
        public int HipAuthorityRecoveryCensoredByNewSource;

        // v0.2.31 bounded-lag virtual parent frame used only by Hip PD during Source Turn / frame catch-up.
        public string HipControlFrameMode;
        public bool HipControlFrameInertializationEnabled;
        public string HipControlFramePhase;
        public bool HipControlFrameActive;
        public float HipControlFrameRotationLagDegrees;
        // v0.2.48 guard-decoupled mechanical branch. Reference Rotation above may receive the 25deg rebase; these never do.
        public Vector3 HipControlFrameStrictMechanicalAngularVelocityWorld;
        public Vector3 HipControlFrameStrictMechanicalAngularAccelerationWorld;
        public float HipControlFrameStrictMechanicalRotationLagDegrees;
        public float HipControlFramePeakStrictMechanicalRotationLagDegrees;
        public bool HipControlFrameLagGuardDecoupledFromMechanical;
        public float HipControlFrameVirtualAngularSpeed;
        public float HipControlFrameRealPelvisAngularSpeed;
        public float HipControlFrameAngularVelocityError;
        // v0.2.41 strict parent control-frame integrator contract.
        public string HipControlFrameIntegratorMode;
        public bool HipControlFrameStrictIntegratorActiveThisStep;
        public bool HipControlFrameSpeedContractSatisfiedThisStep;
        public bool HipControlFrameAccelerationContractSatisfiedThisStep;
        public bool HipControlFrameInheritedSpeedOverLimitThisStep;
        public float HipControlFrameIntegratorCurrentMaxAngularSpeed;
        public float HipControlFrameIntegratorCurrentMaxAngularAcceleration;
        public float HipControlFrameVirtualAngularAcceleration;
        public float HipControlFramePeakVirtualAngularAcceleration;
        public float HipControlFramePeakVirtualSpeedToLimitRatio;
        public float HipControlFramePeakVirtualAccelerationToLimitRatio;
        public int HipControlFrameBrakingLimitedFixedSteps;
        public int HipControlFrameInheritedSpeedOverLimitFixedSteps;
        // v0.2.51 RAW_BASELINE -> SOURCE_TURN_INERTIALIZED continuity diagnostics.
        public bool HipControlFrameActivationSyncAppliedThisStep;
        public int HipControlFrameActivationSyncCount;
        public float HipControlFrameActivationSyncPoseCorrectionDegrees;
        public float HipControlFrameActivationSyncAngularVelocityCorrection;
        public float HipControlFramePeakActivationSyncPoseCorrectionDegrees;
        public float HipControlFramePeakActivationSyncAngularVelocityCorrection;
        // v0.2.56 active strict mechanical settled-origin command closure.
        public bool HipControlFrameMechanicalClosureSyncAppliedThisStep;
        public int HipControlFrameMechanicalClosureSyncCount;
        public float HipControlFrameMechanicalClosureSyncPoseCorrectionDegrees;
        public float HipControlFrameMechanicalClosureSyncAngularVelocityCorrection;
        public float HipControlFramePeakMechanicalClosureSyncPoseCorrectionDegrees;
        public float HipControlFramePeakMechanicalClosureSyncAngularVelocityCorrection;
        public float HipControlFrameMaxLagDegrees;
        // v0.2.70 reference-only lag-guard rebase causal A/B. B suppresses only pose rebase while measuring the
        // counterfactual correction that A would have applied; strict mechanical branch remains unchanged.
        public string HipControlFrameReferenceLagGuardRebaseMode;
        public string HipControlFrameReferenceLagGuardRebaseArm;
        public bool HipControlFrameReferenceLagGuardRebaseEnabled;
        public int HipControlFrameLagGuardSuppressedFixedSteps;
        public float HipControlFrameCurrentSuppressedLagBoundCorrectionDegrees;
        public float HipControlFramePeakSuppressedLagBoundCorrectionDegrees;
        public float HipControlFrameTotalSuppressedLagBoundCorrectionDegrees;
        public float HipControlFramePreGuardRotationLagDegrees;
        public bool HipControlFrameLagBoundAppliedThisStep;
        public float HipControlFrameLagBoundCorrectionDegrees;
        public int HipControlFrameLagBoundCorrectionFixedSteps;
        public float HipControlFramePeakPreGuardRotationLagDegrees;
        public float HipControlFramePeakLagBoundCorrectionDegrees;
        public float HipControlFrameTotalLagBoundCorrectionDegrees;
        public int HipControlFrameBaselineFixedSteps;
        public int HipControlFrameSourceFixedSteps;
        public int HipControlFrameRecoveryFixedSteps;
        public int HipControlFrameSpeedLimitedFixedSteps;
        public int HipControlFrameAccelerationLimitedFixedSteps;
        public int HipControlFrameRecoveryStarted;
        public int HipControlFrameRecoveryCompleted;
        public int HipControlFrameRecoveryTimedOut;
        public int HipControlFrameRecoveryCensoredByNewSource;
        public float HipControlFramePeakRotationLagDegrees;
        public float HipControlFramePeakVirtualAngularSpeed;
        public float HipControlFramePeakRealPelvisAngularSpeed;
        public float HipControlFramePeakAngularVelocityError;
        // 保留 v0.2.0 字段名：这是 full quaternion error，不等同于 PCA flexion 轴向误差。
        public float KneeRotationErrorDegrees;
        public float KneeAxisErrorDegrees;
        public float KneeOffAxisErrorDegrees;
        public float KneeTargetRelativeAngularSpeedTotal;
        public float KneeTargetRelativeAngularSpeedAlongFlexionAxis;
        public float KneeTargetRelativeAngularSpeedOffAxis;
        public float KneeAxisExplainedRatio;

        // v0.2.1 parent-transport attribution：纯诊断，不进入任何控制。
        public bool TransportAttributionValid;
        public float HipRelativeComTransportErrorMagnitude;

        public float KneeRelativeComTransportErrorMagnitude;

        // Current raw-pose control diagnostics. v0.2.17 removed the optional Single-Leg pose bridge;
        // these fields now compare the active raw sampled target against the physics body.
        public float HipControlRotationErrorDegrees;
        public float KneeControlAxisErrorDegrees;

        // Active D-target diagnostics. The optional v0.2.7 velocity bridge is retired. Legacy field name retained:
        // KneeRawSampledAxisAngularVelocitySigned actually mirrors the TargetVelocityBlend-filtered Render sample-held W,
        // not the v0.2.114 pre-blend raw Render W tap.
        public float KneeRawSampledAxisAngularVelocitySigned;

        // v0.2.8 validated Single-Leg target angular-acceleration ACTUATION feedforward.
        // The feedforward contribution is added to the existing PD acceleration demand, but the final actuator output
        // remains clamped by the original Hip/Knee MaxAngularAcceleration. Source Turn/Recovery/stale samples suppress it.
        public bool LegAngularAccelerationFeedforwardEnabled;
        public string LegAngularAccelerationFeedforwardMode;
        public string LegAngularAccelerationFeedforwardGateMode;
        public float LegAngularAccelerationFeedforwardGain;
        public float LegAngularAccelerationFeedforwardMaxAcceleration;
        public float LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds;
        public bool LegAngularAccelerationFeedforwardActiveThisStep;
        public string LegAngularAccelerationFeedforwardSuppressionReason;
        public float HipAngularAccelerationFeedforwardAppliedMagnitude;
        public float KneeAngularAccelerationFeedforwardAppliedSigned;
        public bool HipAngularAccelerationFeedforwardClamped;
        public bool KneeAngularAccelerationFeedforwardClamped;
        public int HipAngularAccelerationFeedforwardClampCount;
        public int KneeAngularAccelerationFeedforwardClampCount;
        public float TraceHipAngularAccelerationFeedforwardAppliedMean;
        public float TraceHipAngularAccelerationFeedforwardAppliedMax;
        public float TraceKneeAngularAccelerationFeedforwardAppliedAbsMean;
        public float TraceKneeAngularAccelerationFeedforwardAppliedAbsMax;

        // v0.2.17 production Knee actuator topology: CHILD_ONLY lower-body ForceMode.Acceleration.
        // Pairwise/reaction attribution fields were removed with their archived experimental module.
        public float KneeLowerAppliedAngularAccelerationMagnitude;
        public float KneeFlexionAppliedAngularAccelerationMagnitude;

        // v0.2.25 production software anatomical constraint. The physical CharacterJoint remains wide.
        // v0.2.26 added read-only recovery attribution; v0.2.27 keeps the same P/D law but phase-schedules only the soft MaxA ceiling.
        public bool KneeSoftOffAxisControlEnabled;
        public float KneeSoftOffAxisAngularSpring;
        public float KneeSoftOffAxisAngularDamper;
        public float KneeSoftOffAxisMaxAngularAcceleration;
        // v0.2.27: phase-aware MaxA scheduling. Baseline value above remains the healthy-motion ceiling.
        public float KneeSoftOffAxisSourceTurnMaxAngularAcceleration;
        public float KneeSoftOffAxisRecoveryMaxAngularAcceleration;
        public string KneeSoftOffAxisAuthorityMode;
        public string KneeSoftOffAxisAuthorityPhase;
        public float KneeSoftOffAxisAuthorityCurrentMaxAngularAcceleration;
        public bool KneeSoftOffAxisAuthorityBoostActive;
        public bool KneeSoftOffAxisPhysicalRecoveryPending;
        public bool KneeSoftOffAxisRecoveryBoostActive;
        public float KneeSoftOffAxisRecoveryErrorDegrees;
        public string KneeSoftOffAxisPhysicalRecoveryOrigin;
        public bool KneeSoftOffAxisTrackingHealthUnhealthy;
        public float KneeSoftOffAxisTrackingHealthCandidateSeconds;
        public int KneeSoftOffAxisTrackingHealthReentryCount;
        public int KneeSoftOffAxisTrackingHealthImmediateReentryCount;
        public float KneeSoftOffAxisTrackingHealthPeakEntryErrorDegrees;
        public int KneeSoftOffAxisSourceBoostFixedSteps;
        public int KneeSoftOffAxisRecoveryBoostFixedSteps;
        public int KneeSoftOffAxisPhysicalRecoveryPendingBaselineFixedSteps;
        public int KneeSoftOffAxisTrackingUnhealthyBaselineFixedSteps;
        public int KneeSoftOffAxisRecoveryStarted;
        public int KneeSoftOffAxisRecoveryCompleted;
        public int KneeSoftOffAxisRecoveryTimedOut;
        public int KneeSoftOffAxisRecoveryCensoredByNewSource;
        public float KneeSoftOffAxisAngularVelocityErrorMagnitude;
        public float KneeSoftOffAxisPContributionMagnitude;
        public float KneeSoftOffAxisDContributionMagnitude;
        public float KneeSoftOffAxisRawAccelerationMagnitude;
        public float KneeSoftOffAxisAppliedAccelerationMagnitude;
        public bool KneeSoftOffAxisAccelerationClamped;
        public int KneeSoftOffAxisClampCount;


        // v0.2.64 momentum-aware Hip recovery braking causal A/B.
        public string HipMomentumRecoveryBrakingMode;
        public string HipMomentumRecoveryBrakingArm;
        public bool HipMomentumRecoveryBrakingEnabled;
        public bool HipMomentumRecoveryBrakingActiveThisStep;
        public int HipMomentumRecoveryBrakingSteps;
        public int HipMomentumRecoveryBrakingRecoveryEligibleSteps;
        public int HipMomentumRecoveryBrakingActiveSteps;
        public int HipMomentumRecoveryBrakingEntries;
        public int HipMomentumRecoveryBrakingExitTowardCount;
        public int HipMomentumRecoveryBrakingExitRecoveryEndedCount;
        public int HipMomentumRecoveryBrakingExitHipRecoveredCount;
        public int HipMomentumRecoveryBrakingSupplementalClampSteps;
        public float HipMomentumRecoveryBrakingCurrentHipDegrees;
        public float HipMomentumRecoveryBrakingCurrentAwaySpeedDegPerSec;
        public float HipMomentumRecoveryBrakingCurrentSupplementalAcceleration;
        public float HipMomentumRecoveryBrakingCurrentProductionMaxA;
        public float HipMomentumRecoveryBrakingEffectiveMaxA;
        public float HipMomentumRecoveryBrakingMeanSupplementalAcceleration;
        public float HipMomentumRecoveryBrakingPeakSupplementalAcceleration;
        public float HipMomentumRecoveryBrakingPeakAwaySpeedDegPerSec;
        public float HipMomentumRecoveryBrakingPeakEffectiveMaxA;
        public string HipMomentumRecoveryBrakingCurrentRecoveryOrigin;
        public string HipMomentumRecoveryBrakingLastExitReason;
        public string HipMomentumRecoveryBrakingAbortEvidence;
        public string HipMomentumRecoveryBrakingKey;

        // v0.2.67 3D vector capture damping causal A/B. B replaces only the v0.2.64 supplemental braking term.
        public string HipVectorCaptureMode;
        public string HipVectorCaptureArm;
        public bool HipVectorCaptureEnabled;
        public bool HipVectorCaptureActive;
        public bool HipVectorCaptureSourceCarryActive;
        public int HipVectorCaptureSteps;
        public int HipVectorCaptureEligibleSteps;
        public int HipVectorCaptureActiveSteps;
        public int HipVectorCaptureEntries;
        public int HipVectorCapturePhysicalExitCount;
        public int HipVectorCaptureSourceCarryEntries;
        public int HipVectorCaptureSourceCarrySteps;
        public int HipVectorCaptureRadialAwayBrakeSteps;
        public int HipVectorCaptureRadialTowardCaptureBrakeSteps;
        public int HipVectorCaptureTangentialDampingSteps;
        public int HipVectorCaptureSupplementalClampSteps;
        // v0.2.69 high-error radial-away final-command floor causal A/B diagnostics.
        public bool HipVectorCaptureHighErrorAwayBrakeFloorEnabled;
        public int HipVectorCaptureHighErrorAwayBrakeFloorGateSteps;
        public int HipVectorCaptureHighErrorAwayBrakeFloorRaisedSteps;
        public int HipVectorCaptureHighErrorAwayBrakeFloorSatisfiedSteps;
        public int HipVectorCaptureHighErrorAwayBrakeFloorUnsatisfiedSteps;
        public int HipVectorCaptureHighErrorAwayBrakeFloorTangentialTrimSteps;
        public bool HipVectorCaptureCurrentHighErrorAwayBrakeFloorGate;
        public bool HipVectorCaptureCurrentHighErrorAwayBrakeFloorSatisfied;
        public float HipVectorCaptureCurrentPreFloorRadialCommandA;
        public float HipVectorCaptureCurrentPostFloorRadialCommandA;
        public float HipVectorCaptureCurrentHighErrorAwayBrakeFloorDeltaA;
        public float HipVectorCaptureMeanHighErrorAwayBrakeFloorDeltaA;
        public float HipVectorCapturePeakHighErrorAwayBrakeFloorDeltaA;
        public float HipVectorCaptureHighErrorAwayBrakeFloorA;
        public string HipVectorCaptureHighErrorAwayBrakeFloorKey;
        public float HipVectorCaptureCurrentErrorDegrees;
        public float HipVectorCaptureCurrentRadialWSignedDegPerSec;
        public float HipVectorCaptureCurrentTangentialWDegPerSec;
        public float HipVectorCaptureCurrentTotalWDegPerSec;
        public float HipVectorCaptureCurrentStoppingAngleDegrees;
        public float HipVectorCaptureCurrentStoppingRatio;
        public float HipVectorCaptureCurrentRadialSupplementalA;
        public float HipVectorCaptureCurrentTangentialSupplementalA;
        public float HipVectorCaptureCurrentSupplementalA;
        public float HipVectorCaptureCurrentProductionMaxA;
        public float HipVectorCaptureCurrentEffectiveMaxA;
        public float HipVectorCaptureMeanSupplementalA;
        public float HipVectorCaptureMeanRadialSupplementalA;
        public float HipVectorCaptureMeanTangentialSupplementalA;
        public float HipVectorCapturePeakSupplementalA;
        public float HipVectorCapturePeakRadialSupplementalA;
        public float HipVectorCapturePeakTangentialSupplementalA;
        public float HipVectorCapturePeakAbsRadialWDegPerSec;
        public float HipVectorCapturePeakTangentialWDegPerSec;
        public float HipVectorCapturePeakStoppingAngleDegrees;
        public float HipVectorCapturePeakStoppingRatio;
        public string HipVectorCaptureCurrentRecoveryOrigin;
        public string HipVectorCaptureLastEntryEvidence;
        public string HipVectorCaptureLastExitReason;
        public string HipVectorCaptureLastSourceCarryEvidence;
        public string HipVectorCaptureAbortEvidence;
        public string HipVectorCaptureKey;

        // v0.2.71 pre-governor Hip Source-reference shock absorber causal A/B. Raw animation remains health truth.
        public string HipSourceReferenceShockAbsorberMode;
        public string HipSourceReferenceShockAbsorberArm;
        public bool HipSourceReferenceShockAbsorberEnabled;
        public bool HipSourceReferenceShockAbsorberActive;
        public int HipSourceReferenceShockAbsorberFixedSteps;
        public int HipSourceReferenceShockAbsorberSourceSteps;
        public int HipSourceReferenceShockAbsorberRecoverySteps;
        public int HipSourceReferenceShockAbsorberActivations;
        public int HipSourceReferenceShockAbsorberReleases;
        public int HipSourceReferenceShockAbsorberSpeedLimitedSteps;
        public int HipSourceReferenceShockAbsorberAccelerationLimitedSteps;
        public int HipSourceReferenceShockAbsorberRawPacketShockCount;
        public float HipSourceReferenceShockAbsorberCurrentRawGapDegrees;
        public float HipSourceReferenceShockAbsorberCurrentOutputStepDegrees;
        public float HipSourceReferenceShockAbsorberCurrentAngularSpeed;
        public float HipSourceReferenceShockAbsorberCurrentAngularAcceleration;
        public float HipSourceReferenceShockAbsorberCurrentMaxAngularSpeed;
        public float HipSourceReferenceShockAbsorberCurrentMaxAngularAcceleration;
        public float HipSourceReferenceShockAbsorberPeakRawPacketDeltaDegrees;
        public float HipSourceReferenceShockAbsorberPeakRawGapDegrees;
        public float HipSourceReferenceShockAbsorberPeakOutputStepDegrees;
        public float HipSourceReferenceShockAbsorberPeakAngularSpeed;
        public float HipSourceReferenceShockAbsorberPeakAngularAcceleration;
        public float HipSourceReferenceShockAbsorberPeakAccelerationRatio;
        public bool HipSourceReferenceShockAbsorberSpeedContractSatisfied;
        public bool HipSourceReferenceShockAbsorberAccelerationContractSatisfied;
        public string HipSourceReferenceShockAbsorberLastEvidence;
        public string HipSourceReferenceShockAbsorberAbortEvidence;
        public string HipSourceReferenceShockAbsorberKey;

        // v0.2.72 final parent*HipLocal composite world-reference feasibility isolation.
        public string HipCompositeReferenceShockAbsorberMode;
        public string HipCompositeReferenceShockAbsorberArm;
        public bool HipCompositeReferenceShockAbsorberEnabled;
        public bool HipCompositeReferenceShockAbsorberActive;
        public int HipCompositeReferenceShockAbsorberFixedSteps;
        public int HipCompositeReferenceShockAbsorberSourceSteps;
        public int HipCompositeReferenceShockAbsorberRecoverySteps;
        public int HipCompositeReferenceShockAbsorberActivations;
        public int HipCompositeReferenceShockAbsorberReleases;
        public int HipCompositeReferenceShockAbsorberSpeedLimitedSteps;
        public int HipCompositeReferenceShockAbsorberAccelerationLimitedSteps;
        public int HipCompositeReferenceShockAbsorberInheritedOverSpeedSteps;
        public int HipCompositeReferenceShockAbsorberInputSpikeSteps;
        public float HipCompositeReferenceShockAbsorberCurrentInputGapDegrees;
        public float HipCompositeReferenceShockAbsorberCurrentInputStepDegrees;
        public float HipCompositeReferenceShockAbsorberCurrentInputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberCurrentInputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberCurrentOutputStepDegrees;
        public float HipCompositeReferenceShockAbsorberCurrentOutputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberCurrentOutputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberCurrentMaxAngularSpeed;
        public float HipCompositeReferenceShockAbsorberCurrentMaxAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberCurrentLocalCompensationDegrees;
        public bool HipCompositeReferenceShockAbsorberCurrentUpstreamReferenceActive;
        public float HipCompositeReferenceShockAbsorberPeakInputGapDegrees;
        public float HipCompositeReferenceShockAbsorberPeakInputStepDegrees;
        public float HipCompositeReferenceShockAbsorberPeakInputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberPeakInputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberPeakOutputStepDegrees;
        public float HipCompositeReferenceShockAbsorberPeakOutputAngularSpeed;
        public float HipCompositeReferenceShockAbsorberPeakOutputAngularAcceleration;
        public float HipCompositeReferenceShockAbsorberPeakOutputAccelerationRatio;
        public float HipCompositeReferenceShockAbsorberPeakLocalCompensationDegrees;
        public bool HipCompositeReferenceShockAbsorberSpeedContractSatisfied;
        public bool HipCompositeReferenceShockAbsorberAccelerationContractSatisfied;
        public string HipCompositeReferenceShockAbsorberLastEvidence;
        public string HipCompositeReferenceShockAbsorberAbortEvidence;
        public string HipCompositeReferenceShockAbsorberKey;

        // v0.2.73 Hip CharacterJoint backend isolation. B removes the Hip CharacterJoint and replaces only
        // translational anchor holding with a common-mode COM acceleration tether; the unchanged Hip AddTorque path
        // remains the only deliberate angular actuator.
        public string HipCharacterJointResponseIsolationMode;
        public string HipCharacterJointResponseIsolationArm;
        public bool HipCharacterJointResponseIsolationEnabled;
        public string HipTranslationTetherSerialChainMode;
        public string HipTranslationTetherSerialChainArm;
        public bool HipTranslationTetherUpperOnlyEnabled;
        public int HipTranslationTetherUpperForceApplications;
        public int HipTranslationTetherLowerForceApplications;
        public int HipCharacterJointResponseIsolationTetherSteps;
        public int HipCharacterJointResponseIsolationTetherClampedSteps;
        public int HipCharacterJointResponseIsolationInvalidAnchorSteps;
        public float HipCharacterJointResponseIsolationCurrentAnchorErrorMeters;
        public float HipCharacterJointResponseIsolationCurrentVelocityError;
        public float HipCharacterJointResponseIsolationCurrentPdAcceleration;
        public float HipCharacterJointResponseIsolationCurrentFeedforwardAcceleration;
        public float HipCharacterJointResponseIsolationCurrentRawAcceleration;
        public float HipCharacterJointResponseIsolationCurrentAppliedAcceleration;
        public bool HipCharacterJointResponseIsolationCurrentClamped;
        public float HipCharacterJointResponseIsolationPeakAnchorErrorMeters;
        public float HipCharacterJointResponseIsolationPeakVelocityError;
        public float HipCharacterJointResponseIsolationPeakPdAcceleration;
        public float HipCharacterJointResponseIsolationPeakFeedforwardAcceleration;
        public float HipCharacterJointResponseIsolationPeakRawAcceleration;
        public float HipCharacterJointResponseIsolationPeakAppliedAcceleration;
        public float HipCharacterJointResponseIsolationTetherSpring;
        public float HipCharacterJointResponseIsolationTetherDamper;
        public float HipCharacterJointResponseIsolationTetherMaxAcceleration;
        public float HipCharacterJointResponseIsolationAnchorValidityMeters;
        public string HipCharacterJointResponseIsolationLastEvidence;
        public string HipCharacterJointResponseIsolationAbortEvidence;
        public string HipCharacterJointResponseIsolationKey;

        // v0.2.75 Knee parent-angular-transport causal A/B. B reuses the exact same-step final Hip angular
        // acceleration command as a common-mode world-space transport term on Lower, while leaving Knee relative
        // controller gains/limits/PCA/CharacterJoint unchanged.
        public string KneeParentAngularTransportMode;
        public string KneeParentAngularTransportArm;
        public bool KneeParentAngularTransportEnabled;
        public int KneeParentAngularTransportSteps;
        public int KneeParentAngularTransportActiveSteps;
        public int KneeParentAngularTransportInvalidSteps;
        public float KneeParentAngularTransportCurrentTransportAcceleration;
        public float KneeParentAngularTransportCurrentRelativeAcceleration;
        public float KneeParentAngularTransportCurrentTotalLowerAcceleration;
        public float KneeParentAngularTransportPeakTransportAcceleration;
        public float KneeParentAngularTransportPeakRelativeAcceleration;
        public float KneeParentAngularTransportPeakTotalLowerAcceleration;
        public string KneeParentAngularTransportLastEvidence;
        public string KneeParentAngularTransportKey;

        // v0.2.76 Knee CharacterJoint response causal A/B. B removes the remaining Knee CharacterJoint and
        // uses an ARL-owned Lower-COM translation-only Knee-anchor tether; angular control remains v0.2.75.
        public string KneeCharacterJointResponseIsolationMode;
        public string KneeCharacterJointResponseIsolationArm;
        public bool KneeCharacterJointResponseIsolationEnabled;
        public int KneeCharacterJointResponseIsolationTetherSteps;
        public int KneeCharacterJointResponseIsolationForceApplications;
        public int KneeCharacterJointResponseIsolationTetherClampedSteps;
        public int KneeCharacterJointResponseIsolationInvalidAnchorSteps;
        public float KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters;
        public float KneeCharacterJointResponseIsolationCurrentVelocityError;
        public float KneeCharacterJointResponseIsolationCurrentRawAcceleration;
        public float KneeCharacterJointResponseIsolationCurrentAppliedAcceleration;
        public bool KneeCharacterJointResponseIsolationCurrentClamped;
        public float KneeCharacterJointResponseIsolationPeakAnchorErrorMeters;
        public float KneeCharacterJointResponseIsolationPeakVelocityError;
        public float KneeCharacterJointResponseIsolationPeakRawAcceleration;
        public float KneeCharacterJointResponseIsolationPeakAppliedAcceleration;
        public float KneeCharacterJointResponseIsolationTetherSpring;
        public float KneeCharacterJointResponseIsolationTetherDamper;
        public float KneeCharacterJointResponseIsolationTetherMaxAcceleration;
        public float KneeCharacterJointResponseIsolationAnchorValidityMeters;
        public string KneeCharacterJointResponseIsolationLastEvidence;
        public string KneeCharacterJointResponseIsolationAbortEvidence;
        public string KneeCharacterJointResponseIsolationKey;

        // v0.2.77 Knee positional-anchor acceleration feedforward causal A/B. B estimates the actual Upper Knee
        // anchor world acceleration from consecutive fixed-step point velocities and adds it before the unchanged
        // v0.2.76 positional PD + unchanged final 60 m/s^2 total tether clamp.
        public string KneeAnchorAccelerationFeedforwardMode;
        public string KneeAnchorAccelerationFeedforwardArm;
        public bool KneeAnchorAccelerationFeedforwardEnabled;
        public int KneeAnchorAccelerationFeedforwardSteps;
        public int KneeAnchorAccelerationFeedforwardWarmupSteps;
        public int KneeAnchorAccelerationFeedforwardActiveSteps;
        public int KneeAnchorAccelerationFeedforwardInvalidSteps;
        public float KneeAnchorAccelerationFeedforwardCurrentParentAnchorSpeed;
        public float KneeAnchorAccelerationFeedforwardCurrentAcceleration;
        public float KneeAnchorAccelerationFeedforwardPeakParentAnchorSpeed;
        public float KneeAnchorAccelerationFeedforwardPeakAcceleration;
        public bool KneeAnchorAccelerationFeedforwardCurrentValid;
        public string KneeAnchorAccelerationFeedforwardLastEvidence;
        public string KneeAnchorAccelerationFeedforwardAbortEvidence;
        public string KneeAnchorAccelerationFeedforwardKey;

        // Historical cleanup (v0.4.15): the non-promoted v0.4.6 Lower-reference reconciler and its
        // report payload were removed. Production is permanently the raw native-gait reference; source-turn
        // diagnostics and unchanged hard safety remain the only retained discontinuity boundary.
        // v0.2.78 Knee flexion feedback-headroom allocation causal A/B. Parent angular transport is NOT part of
        // this ceiling; the tested shared scalar ceiling is only (P+D feedback) + Knee A-FF -> existing +/-40 clamp.
        public string KneeFlexionAllocationMode;
        public string KneeFlexionAllocationArm;
        public bool KneeFlexionAllocationEnabled;
        public int KneeFlexionAllocationSteps;
        public int KneeFlexionAllocationActiveSteps;
        public int KneeFlexionAllocationInvalidSteps;
        public int KneeFlexionAllocationOpposingSteps;
        public int KneeFlexionAllocationSuppressedOpposingFfSteps;
        public int KneeFlexionAllocationSameDirectionSteps;
        public int KneeFlexionAllocationOriginalWouldClampSteps;
        public int KneeFlexionAllocationFinalClampSteps;
        public float KneeFlexionAllocationCurrentPd;
        public float KneeFlexionAllocationCurrentInputFf;
        public float KneeFlexionAllocationCurrentEffectiveFf;
        public float KneeFlexionAllocationCurrentOriginalRaw;
        public float KneeFlexionAllocationCurrentAllocatedRaw;
        public float KneeFlexionAllocationCurrentApplied;
        public float KneeFlexionAllocationCurrentMaxA;
        public bool KneeFlexionAllocationCurrentOpposing;
        public bool KneeFlexionAllocationCurrentOriginalWouldClamp;
        public bool KneeFlexionAllocationCurrentFinalClamped;
        public float KneeFlexionAllocationPeakPdAbs;
        public float KneeFlexionAllocationPeakInputFfAbs;
        public float KneeFlexionAllocationPeakOriginalRawAbs;
        public float KneeFlexionAllocationPeakAllocatedRawAbs;
        public float KneeFlexionAllocationPeakAppliedAbs;
        public float KneeFlexionAllocationSuppressedFfAbsMean;
        public string KneeFlexionAllocationLastEvidence;
        public string KneeFlexionAllocationKey;

        // v0.2.79 Hip positional parent-anchor acceleration feedforward causal A/B. B consumes the upcoming
        // source-Hip anchor command acceleration for the already-kinematic mechanical parent before the unchanged
        // Hip positional tether 400/40 + final 60m/s^2 clamp. Signed parent-local diagnostics use X/Y/Z=L/V/F.
        public string HipAnchorAccelerationFeedforwardMode;
        public string HipAnchorAccelerationFeedforwardArm;
        public bool HipAnchorAccelerationFeedforwardEnabled;
        public int HipAnchorAccelerationFeedforwardSteps;
        public int HipAnchorAccelerationFeedforwardWarmupSteps;
        public int HipAnchorAccelerationFeedforwardActiveSteps;
        public int HipAnchorAccelerationFeedforwardStrictLinearTransportSuppressedSteps;
        public int HipAnchorAccelerationFeedforwardInvalidSteps;
        public float HipAnchorAccelerationFeedforwardCurrentSettledParentSpeed;
        public float HipAnchorAccelerationFeedforwardCurrentCommandedParentSpeed;
        public float HipAnchorAccelerationFeedforwardCurrentCandidateAcceleration;
        public float HipAnchorAccelerationFeedforwardCurrentAppliedAcceleration;
        public float HipAnchorAccelerationFeedforwardCurrentSourceToChildLateral;
        public float HipAnchorAccelerationFeedforwardCurrentSourceToChildVertical;
        public float HipAnchorAccelerationFeedforwardCurrentSourceToChildForward;
        public float HipAnchorAccelerationFeedforwardCurrentSourceToParentForward;
        public float HipAnchorAccelerationFeedforwardCurrentParentToChildForward;
        public float HipAnchorAccelerationFeedforwardCurrentForwardVelocityError;
        public float HipAnchorAccelerationFeedforwardPeakAbsSourceToChildForward;
        public float HipAnchorAccelerationFeedforwardPeakAbsSourceToChildLateral;
        public float HipAnchorAccelerationFeedforwardPeakAbsSourceToChildVertical;
        public float HipAnchorAccelerationFeedforwardPeakAbsSourceToParentForward;
        public float HipAnchorAccelerationFeedforwardPeakAbsParentToChildForward;
        public float HipAnchorAccelerationFeedforwardPeakAbsForwardVelocityError;
        public bool HipAnchorAccelerationFeedforwardCurrentValid;
        public string HipAnchorAccelerationFeedforwardLastEvidence;
        public string HipAnchorAccelerationFeedforwardAbortEvidence;
        public string HipAnchorAccelerationFeedforwardKey;

        // v0.2.80 read-only settled-target Hip physical catch-up attribution. Starts only in Normal when the
        // raw Source Hip target is already low-W/low-A but physical Hip error remains >=5deg; no controller writes.

        // v0.2.81 Hip angular actuator inertia-space response-closure causal A/B. A keeps historical
        // ForceMode.Acceleration; B maps the exact same desired world alpha through validated inertia + gyro
        // into physical torque and uses ForceMode.Force. angularDrag remains identical/uncompensated in both arms.
        public string HipAngularActuatorMode;
        public string HipAngularActuatorArm;
        public bool HipAngularActuatorEnabled;
        public int HipAngularActuatorSteps;
        public int HipAngularActuatorActiveSteps;
        public int HipAngularActuatorInvalidSteps;
        public float HipAngularActuatorCurrentDesiredAcceleration;
        public float HipAngularActuatorCurrentAngularSpeed;
        public float HipAngularActuatorCurrentInertiaAlphaTorque;
        public float HipAngularActuatorCurrentGyroTorque;
        public float HipAngularActuatorCurrentTotalTorque;
        public float HipAngularActuatorCurrentPredictedAcceleration;
        public float HipAngularActuatorCurrentPredictionError;
        public float HipAngularActuatorCurrentPredictionAlignment;
        public float HipAngularActuatorCurrentGyroToInertiaRatio;
        public float HipAngularActuatorPeakDesiredAcceleration;
        public float HipAngularActuatorPeakAngularSpeed;
        public float HipAngularActuatorPeakInertiaAlphaTorque;
        public float HipAngularActuatorPeakGyroTorque;
        public float HipAngularActuatorPeakTotalTorque;
        public float HipAngularActuatorPeakPredictionError;
        public float HipAngularActuatorPeakGyroToInertiaRatio;
        public float HipAngularActuatorBodyMass;
        public float HipAngularActuatorBodyAngularDrag;
        public Vector3 HipAngularActuatorBodyInertiaTensor;
        public string HipAngularActuatorLastEvidence;
        public string HipAngularActuatorKey;

        // v0.2.82 Upper angular-drag causal A/B. A keeps the complete v0.2.81 B backend with the profile
        // Upper angularDrag; B sets Upper angularDrag=0 only. Lower drag and every controller/actuator input remain frozen.
        // v0.2.44 production physical-state-aware, recovery-continuous Hip reference governor.
        public string HipReferenceGovernorMode;
        public string HipReferenceGovernorKey;
        public bool HipReferenceGovernorActiveThisStep;
        public bool HipReferenceGovernorAccelerationContractSatisfiedThisStep;
        public int HipReferenceGovernorActiveFixedSteps;
        public int HipReferenceGovernorNormalBypassFixedSteps;
        public int HipReferenceGovernorActivationCount;
        public int HipReferenceGovernorReleaseCount;
        public int HipReferenceGovernorLeadCappedFixedSteps;
        // v0.2.98 exact diagnostic mirrors of existing governor branches; read-only attribution only.
        public int HipReferenceGovernorAccelerationLimitedFixedSteps;
        public int HipReferenceGovernorBrakingSpeedLimitedFixedSteps;
        public bool HipReferenceGovernorLeadCappedThisStep;
        public bool HipReferenceGovernorAccelerationLimitedThisStep;
        public bool HipReferenceGovernorBrakingSpeedLimitedThisStep;
        public bool HipReferenceGovernorReleaseEligibleThisStep;
        public float HipReferenceGovernorCurrentPredictedRawAdvanceDegrees;
        public int HipReferenceGovernorPredictedRawFixedSteps;
        public int HipReferenceGovernorSourceActiveFixedSteps;
        public int HipReferenceGovernorPhysicalRecoveryFixedSteps;
        public int HipReferenceGovernorControlFrameActiveFixedSteps;
        public float HipReferenceGovernorPeakRawPacketDeltaDegrees;
        public float HipReferenceGovernorPeakGovernedFixedStepDegrees;
        public float HipReferenceGovernorPeakRawToGovernedGapDegrees;
        public float HipReferenceGovernorPeakPhysicalLeadDegrees;
        public float HipReferenceGovernorPeakRawPhysicalErrorDegrees;
        public float HipReferenceGovernorPeakAngularSpeed;
        public float HipReferenceGovernorPeakAngularAcceleration;
        public float HipReferenceGovernorPeakAllowedAngularAcceleration;
        public float HipReferenceGovernorPeakAccelerationRatio;
        public float HipReferenceGovernorPeakPredictedRawAdvanceDegrees;
        public float HipReferenceGovernorPeakReleasePoseJumpDegrees;
        public float HipReferenceGovernorPeakReleaseAngularVelocityJump;
        public float HipReferenceGovernorCurrentRawGapDegrees;
        public float HipReferenceGovernorCurrentPhysicalLeadDegrees;
        public float HipReferenceGovernorCurrentAngularAcceleration;
        public float HipReferenceGovernorCurrentAllowedAngularAcceleration;
        public float HipReferenceGovernorReleaseCandidateSeconds;
        public string HipReferenceGovernorActivationReason;
        public string HipReferenceGovernorLastEvidence;

        // v0.2.17 production Rigidbody mass-property foundation. Read-only after Build.
        public string RigidbodyInertiaFoundationMode;
        public bool RigidbodyInertiaAuditCompleted;
        public bool RigidbodyInertiaBuildFoundationValidated;
        public string RigidbodyInertiaAuditSummary;
        public string RigidbodyInertiaAuditDetail;
        // v0.2.122 read-only Single-Leg frozen-baseline certification. Existing safety contracts only.
        public string SingleLegFreezeCertificationMode;
        public string SingleLegFreezeCertificationPolicy;
        public bool SingleLegFreezeCertificationComplete;
        public bool SingleLegFreezeProductionContractPass;
        public bool SingleLegFreezeCalibrationPass;
        public bool SingleLegFreezeInertiaPass;
        public bool SingleLegFreezeTwoBodyPass;
        public bool SingleLegFreezeRunSafetyPass;
        public bool SingleLegFreezeReady;
        public string SingleLegFreezeReason;
        public string SingleLegFreezeProductionContractKey;
        public string SingleLegFreezeKey;

        // Current compact angular kinematics retained for controller/report audit; historical lag-correlation accumulators retired in v0.3.17.
        public string AngularDynamicsMode;
        public bool AngularDynamicsValid;
        public float HipTargetRelativeAngularSpeed;
        public float HipBodyRelativeAngularSpeed;
        public float HipAngularVelocityErrorMagnitude;
        public float HipTargetRelativeAngularAccelerationMagnitude;
        public float HipPdPContributionMagnitude;
        public float HipPdDContributionMagnitude;
        // Controller-local signed 3D velocities retained for actuator/reference audit; active D target is raw Render sample-held W.
        public float KneeTargetAxisAngularVelocitySigned;
        public float KneeBodyAxisAngularVelocitySigned;
        public float KneeAxisAngularVelocityErrorSigned;
        public float KneeTargetAxisAngularAccelerationSigned;
        public float KneePdPContributionSigned;
        public float KneePdDContributionSigned;
        // Explicit Render->FixedUpdate/body-measurement timing semantics retained for safety/clock diagnostics.
        public PhysicsStepContext RuntimeContext;
        public SourceStepContext RuntimeSourceContext;
        public bool RuntimeParentMatched;
        public int FixedUpdateSequence;
        public int TargetRenderSampleSequence;
        public bool TargetRenderSampleFreshAtFixedStep;
        public float TargetRenderSampleAgeSeconds;
        public int FixedStepsSinceTargetRenderSample;

        // v0.3.42 physical causal A/B under fixed Hip v0.3.39 B. A keeps Knee 45/9 exactly; B scales only
        // NORMAL_TRANSITION P x1.20 and D x sqrt(1.20) before the unchanged v0.2.78 allocation / +/-40 clamp.

        public float HipAnchorErrorMeters;
        public float KneeAnchorErrorMeters;
        // Historical PD-only raw demand (before v0.2.8 A-FF).
        // v0.2.8 total raw actuator demand after adding bounded A-FF but before the unchanged final MaxA clamp.
        public bool HipAccelerationClamped;
        public bool KneeAccelerationClamped;
        public int HipClampCount;
        public int KneeClampCount;
        public float PeakHipRotationErrorDegrees;
        public float PeakKneeRotationErrorDegrees;
        public float PeakHipAnchorErrorMeters;
        public float PeakKneeAnchorErrorMeters;

        public int TraceSamples;
        public float TraceHipRotationMeanDegrees;
        public float TraceHipRotationMaxDegrees;
        public float TraceKneeRotationMeanDegrees;
        public float TraceKneeRotationMaxDegrees;
        public int TraceHipClampSamples;
        public int TraceKneeClampSamples;
        public float TraceKneeOffAxisErrorMeanDegrees;
        public float TraceKneeOffAxisErrorMaxDegrees;
        public float TraceKneeSoftOffAxisAppliedMean;
        public float TraceKneeSoftOffAxisAppliedMax;
        public int TraceKneeSoftOffAxisClampSamples;

        // v0.2.1 source-aware / phase-aware attribution summary.
        public string AttributionMode;
        public string AttributionPhase;

        // v0.4.19 Physical Lower native-gravity support-load causal A/B. Both arms keep the v0.4.18
        // Foot-gravity baseline; B additionally enables native gravity on Lower only for exact authorized STANCE.
        // Diagnostics attribute theoretical Lower self-weight and the existing child-only Knee tether support-normal acceleration.
        public string StanceLowerNativeGravityMode;
        public string StanceLowerNativeGravityArm;
        public bool StanceLowerNativeGravityFinite = true;
        public string StanceLowerNativeGravityInvalidReason;
        public bool StanceLowerNativeGravityEnabled;
        public int StanceLowerNativeGravityTraceSamples;
        public int StanceLowerNativeGravityEligibleSamples;
        public int StanceLowerNativeGravityAppliedSamples;
        public int StanceLowerNativeGravityNonStanceAppliedSamples;
        public int StanceLowerNativeGravityReadbackFailures;
        public float StanceLowerNativeGravityExpectedNormalForceMean;
        public float StanceLowerNativeGravityExpectedNormalForceMax;
        public float StanceLowerNativeGravityExpectedNormalImpulseMean;
        public float StanceLowerNativeGravityExpectedNormalImpulseMax;
        public float StanceLowerNativeGravitySupportNormalSpeedAbsMean;
        public float StanceLowerNativeGravitySupportNormalSpeedAbsMax;
        public float StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMean;
        public float StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMin;
        public float StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMax;
        public Vector3 StanceLowerNativeGravityGravityWorld;
        public float StanceLowerNativeGravityGravityMagnitude;
        public float StanceLowerNativeGravityLowerMass;
        public bool StanceLowerNativeGravityCurrentApplied;
        public int StanceLowerNativeGravityCurrentScheduledFixed;
        public int StanceLowerNativeGravityFirstAppliedFixed;
        public int StanceLowerNativeGravityLastAppliedFixed;
        public string StanceLowerNativeGravitySummary;

        // v0.4.20 support-normal reciprocal Foot<->Lower load-transfer telemetry. v0.4.21 freezes this B/ON
        // in both arms; it remains useful attribution for the independent self-weight transfer experiment.
        public KneeSupportNormalReactionReport KneeSupportReaction;
        public UpperSelfWeightSupportReport UpperSelfWeightSupport;
        public string StanceFootLowerReciprocalMode;
        public string StanceFootLowerReciprocalArm;
        public bool StanceFootLowerReciprocalFinite = true;
        public string StanceFootLowerReciprocalInvalidReason;
        public bool StanceFootLowerReciprocalEnabled;
        public int StanceFootLowerReciprocalTraceSamples;
        public int StanceFootLowerReciprocalEligibleSamples;
        public int StanceFootLowerReciprocalAppliedSamples;
        public int StanceFootLowerReciprocalNonStanceAppliedSamples;
        public int StanceFootLowerReciprocalZeroReactionSamples;
        public int StanceFootLowerReciprocalUpwardLowerReactionSamples;
        public int StanceFootLowerReciprocalDownwardLowerReactionSamples;
        public float StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMean;
        public float StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMin;
        public float StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMax;
        public float StanceFootLowerReciprocalForceAbsMean;
        public float StanceFootLowerReciprocalForceAbsMax;
        public float StanceFootLowerReciprocalLowerSupportNormalAccelerationMean;
        public float StanceFootLowerReciprocalLowerSupportNormalAccelerationMin;
        public float StanceFootLowerReciprocalLowerSupportNormalAccelerationMax;
        public bool StanceFootLowerReciprocalCurrentApplied;
        public int StanceFootLowerReciprocalCurrentFixed;
        public float StanceFootLowerReciprocalCurrentFootMass;
        public float StanceFootLowerReciprocalCurrentLowerMass;
        public float StanceFootLowerReciprocalCurrentFootTetherSupportNormalAcceleration;
        public float StanceFootLowerReciprocalCurrentFootTetherSupportNormalForce;
        public float StanceFootLowerReciprocalCurrentLowerSupportNormalAcceleration;
        public Vector3 StanceFootLowerReciprocalCurrentReactionForceWorld;
        public int StanceFootLowerReciprocalFirstAppliedFixed;
        public int StanceFootLowerReciprocalLastAppliedFixed;
        public string StanceFootLowerReciprocalSummary;

        // v0.4.21 independent Lower self-weight support-normal load-transfer causal A/B.
        public string StanceLowerSelfWeightTransferMode;
        public string StanceLowerSelfWeightTransferArm;
        public bool StanceLowerSelfWeightTransferFinite = true;
        public string StanceLowerSelfWeightTransferInvalidReason;
        public bool StanceLowerSelfWeightTransferEnabled;
        public int StanceLowerSelfWeightTransferTraceSamples;
        public int StanceLowerSelfWeightTransferEligibleSamples;
        public int StanceLowerSelfWeightTransferAppliedSamples;
        public int StanceLowerSelfWeightTransferNonStanceAppliedSamples;
        public float StanceLowerSelfWeightTransferExpectedForceMean;
        public float StanceLowerSelfWeightTransferExpectedForceMax;
        public float StanceLowerSelfWeightTransferExpectedImpulseMean;
        public float StanceLowerSelfWeightTransferExpectedImpulseMax;
        public float StanceLowerSelfWeightTransferLowerSupportAccelerationMean;
        public float StanceLowerSelfWeightTransferLowerSupportAccelerationMax;
        public float StanceLowerSelfWeightTransferFootLoadAccelerationAbsMean;
        public float StanceLowerSelfWeightTransferFootLoadAccelerationAbsMax;
        public bool StanceLowerSelfWeightTransferCurrentApplied;
        public int StanceLowerSelfWeightTransferCurrentFixed;
        public float StanceLowerSelfWeightTransferCurrentLowerMass;
        public float StanceLowerSelfWeightTransferCurrentFootMass;
        public float StanceLowerSelfWeightTransferCurrentExpectedForce;
        public float StanceLowerSelfWeightTransferCurrentExpectedImpulse;
        public float StanceLowerSelfWeightTransferCurrentLowerSupportAcceleration;
        public float StanceLowerSelfWeightTransferCurrentFootLoadAcceleration;
        public Vector3 StanceLowerSelfWeightTransferCurrentForceWorld;
        public int StanceLowerSelfWeightTransferFirstAppliedFixed;
        public int StanceLowerSelfWeightTransferLastAppliedFixed;
        public string StanceLowerSelfWeightTransferSummary;

        public bool DriveAborted;
        public string DriveAbortReason;

        // Profile angular drag remains a live safety contract; the rejected zero-drag experiment is retired.
        public int LegAngularDragSteps;
        public int LegAngularDragInvalidSteps;
        public float LegAngularDragExpected;
        public float LegAngularDragUpper;
        public float LegAngularDragLower;
        public float LegAngularDragPeakUpperError;
        public float LegAngularDragPeakLowerError;
        public string LegAngularDragEvidence;

        public string GetConsoleSummary()
        {
            return
                "ARL single-leg | built=" + Built +
                " | drive=" + DriveEnabled +
                " | finite=" + Finite +
                " | side=" + (Side ?? "<none>") +
                " | resolve=" + (ResolutionMode ?? "<none>") +
                " | joints(hip/kneeCharacter)=" + HipCharacterJointPresent + "/" + KneeCharacterJointPresent +
                " | connected=" + HipConnectedBodyMatch + "/" + KneeConnectedBodyMatch +
                " | hipMechanicalParent=" + (HipMechanicalParentMode ?? "<none>") + "/proxy=" + HipMechanicalParentConnectedToProxy + "/A=" + F(HipMechanicalParentCurrentAngularAcceleration) + "/rawA=" + F(HipMechanicalParentCurrentRawPelvisAngularAcceleration) + "/cmdA=" + F(HipMechanicalParentCurrentCommandedAngularAcceleration) + "/closureAErr=" + F(HipMechanicalParentCommandClosureAngularAccelerationError) + "/closureARatio=" + F(HipMechanicalParentCommandClosureMaxAccelerationRatio) + "/strictClosureOK=" + HipMechanicalParentCommandClosureAccelerationContractSatisfied + "/poseAuditOK=" + HipMechanicalParentPoseDerivedAuditWithinTolerance + "/poseOverMaxA=" + F(HipMechanicalParentPoseDerivedCommandOverMaxA) +
                " | hipParentTransport=" + HipMechanicalParentTransportEnabled + "/healthyArm=" + (HipMechanicalParentNormalTransportArm ?? "<none>") + "/recoverySupport=" + HipMechanicalParentRecoveryTransportEnabled + "/active=" + HipMechanicalParentTransportActiveThisStep + "/recoveryActive=" + HipMechanicalParentRecoveryTransportActiveThisStep + "/healthyActive=" + HipMechanicalParentHealthyNormalTransportActiveThisStep + "/branch=" + (HipMechanicalParentTransportActiveBranch ?? "<none>") + "/transport=" + F(HipMechanicalParentTransportCurrentMagnitude) + "/relative=" + F(HipMechanicalParentTransportCurrentRelativeCorrectionMagnitude) + "/applied=" + F(HipMechanicalParentTransportCurrentCombinedAppliedMagnitude) + "/maxA=" + F(HipMechanicalParentTransportCurrentMaxAngularAcceleration) + "/clamp=" + HipMechanicalParentTransportCombinedClamped +
                " | hipLinearTransport=" + HipAnchorLinearTransportEnabled + "/active=" + HipAnchorLinearTransportActiveThisStep + "/raw=" + F(HipAnchorLinearTransportCurrentRawAcceleration) + "/applied=" + F(HipAnchorLinearTransportCurrentAppliedAcceleration) + "/maxA=" + F(HipAnchorLinearTransportCurrentMaxAcceleration) + "/clamp=" + HipAnchorLinearTransportClampedThisStep +
                " | phase=" + (AttributionPhase ?? "<none>") +
                " | kneeBackend=" + (KneeConstraintBackendMode ?? "<none>") +
                " | kneeAxis=" + (KneeAxisMode ?? "<none>") + "/" + V(KneeFlexionAxisLowerLocal) +
                " | calibration=" + KneeAxisCalibrationCollecting + "/" + KneeAxisCalibrationReady + "/" + (KneeAxisCalibrationReason ?? "<none>") + "/n=" + KneeAxisCalibrationSampleCount +
                " | errDeg(hip/kneeFull/kneeAxis/kneeOff)=" + F(HipRotationErrorDegrees) + "/" + F(KneeRotationErrorDegrees) + "/" + F(KneeAxisErrorDegrees) + "/" + F(KneeOffAxisErrorDegrees) +
                " | hipAuthority=" + (HipAuthorityPhase ?? "<none>") + ":" + F(HipAuthorityCurrentMaxAngularAcceleration) + "/rawRecoveryErr=" + F(HipAuthorityRecoveryErrorDegrees) + "/source=" + F(HipSourceTurnMaxAngularAcceleration) + "/recovery=" + F(HipRecoveryMaxAngularAcceleration) +
                " | hipControlFrame=" + (HipControlFramePhase ?? "<none>") + "/active=" + HipControlFrameActive + "/lag=" + F(HipControlFrameRotationLagDegrees) + "/w=" + F(HipControlFrameVirtualAngularSpeed) + "/a=" + F(HipControlFrameVirtualAngularAcceleration) + "/contract=" + HipControlFrameSpeedContractSatisfiedThisStep + "/" + HipControlFrameAccelerationContractSatisfiedThisStep + "/activationSync=" + HipControlFrameActivationSyncAppliedThisStep + "/syncPose=" + F(HipControlFrameActivationSyncPoseCorrectionDegrees) + "/syncW=" + F(HipControlFrameActivationSyncAngularVelocityCorrection) + "/closureSync=" + HipControlFrameMechanicalClosureSyncAppliedThisStep + "/closurePose=" + F(HipControlFrameMechanicalClosureSyncPoseCorrectionDegrees) + "/closureW=" + F(HipControlFrameMechanicalClosureSyncAngularVelocityCorrection) +
                " | sourceShockAbsorber=" + (HipSourceReferenceShockAbsorberArm ?? "<none>") + "/active=" + HipSourceReferenceShockAbsorberActive + "/rawGap=" + F(HipSourceReferenceShockAbsorberCurrentRawGapDegrees) + "/step=" + F(HipSourceReferenceShockAbsorberCurrentOutputStepDegrees) + "/W-A=" + F(HipSourceReferenceShockAbsorberCurrentAngularSpeed) + "/" + F(HipSourceReferenceShockAbsorberCurrentAngularAcceleration) +
                " | compositeShockAbsorber=" + (HipCompositeReferenceShockAbsorberArm ?? "<none>") + "/active=" + HipCompositeReferenceShockAbsorberActive + "/inputStep-A=" + F(HipCompositeReferenceShockAbsorberCurrentInputStepDegrees) + "/" + F(HipCompositeReferenceShockAbsorberCurrentInputAngularAcceleration) + "/outputStep-A=" + F(HipCompositeReferenceShockAbsorberCurrentOutputStepDegrees) + "/" + F(HipCompositeReferenceShockAbsorberCurrentOutputAngularAcceleration) +
                " | kneeAxisRatio=" + F(KneeAxisExplainedRatio) +
                " | transportErr(hip/knee)=" + F(HipRelativeComTransportErrorMagnitude) + "/" + F(KneeRelativeComTransportErrorMagnitude) +
                " | legAngularDrag=" + F(LegAngularDragUpper) + "/" + F(LegAngularDragLower) + "/expected=" + F(LegAngularDragExpected) + "/invalid=" + LegAngularDragInvalidSteps +
                " | hipActuator=" + (HipAngularActuatorArm ?? "<none>") + "/A=" + F(HipAngularActuatorCurrentDesiredAcceleration) + "/w=" + F(HipAngularActuatorCurrentAngularSpeed) + "/tauI=" + F(HipAngularActuatorCurrentInertiaAlphaTorque) + "/gyro=" + F(HipAngularActuatorCurrentGyroTorque) + "/tau=" + F(HipAngularActuatorCurrentTotalTorque) + "/predA=" + F(HipAngularActuatorCurrentPredictedAcceleration) + "/predErr=" + F(HipAngularActuatorCurrentPredictionError) + "/align=" + F(HipAngularActuatorCurrentPredictionAlignment) +
                " | legAFF=" + LegAngularAccelerationFeedforwardEnabled + "/" + (LegAngularAccelerationFeedforwardMode ?? "<none>") + "/normal=PRODUCTION_ON" +
                "/gain/maxA=" + F(LegAngularAccelerationFeedforwardGain) + "/" + F(LegAngularAccelerationFeedforwardMaxAcceleration) +
                "/active=" + LegAngularAccelerationFeedforwardActiveThisStep + "/suppress=" + (LegAngularAccelerationFeedforwardSuppressionReason ?? "<none>") +
                "/applied=" + F(HipAngularAccelerationFeedforwardAppliedMagnitude) + "/" + F(KneeAngularAccelerationFeedforwardAppliedSigned) +
                "/clamp=" + HipAngularAccelerationFeedforwardClamped + "/" + KneeAngularAccelerationFeedforwardClamped +
                " | kneeAnchorFF=" + (KneeAnchorAccelerationFeedforwardArm ?? "<none>") + ":" + F(KneeAnchorAccelerationFeedforwardCurrentAcceleration) + "/valid=" + KneeAnchorAccelerationFeedforwardCurrentValid +
                " | kneeTorque=CHILD_ONLY/lowerA=" + F(KneeLowerAppliedAngularAccelerationMagnitude) +
                "/parentTransport=" + (KneeParentAngularTransportArm ?? "<none>") + ":" + F(KneeParentAngularTransportCurrentTransportAcceleration) + "/relative=" + F(KneeParentAngularTransportCurrentRelativeAcceleration) + "/total=" + F(KneeParentAngularTransportCurrentTotalLowerAcceleration) +
                "/flexA=" + F(KneeFlexionAppliedAngularAccelerationMagnitude) +
                "/softA=" + F(KneeSoftOffAxisAppliedAccelerationMagnitude) +
                "/softClamp=" + KneeSoftOffAxisAccelerationClamped +
                "/authority=" + (KneeSoftOffAxisAuthorityPhase ?? "<none>") + ":" + F(KneeSoftOffAxisAuthorityCurrentMaxAngularAcceleration) +
                " | lowerGravity=" + (StanceLowerNativeGravityArm ?? "<none>") + "/on=" + StanceLowerNativeGravityEnabled + "/applied=" + StanceLowerNativeGravityAppliedSamples + "/kneeSupportN=" + F(StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMean) +
                " | footLowerReciprocal=" + (StanceFootLowerReciprocalArm ?? "<none>") + "/on=" + StanceFootLowerReciprocalEnabled + "/applied=" + StanceFootLowerReciprocalAppliedSamples + "/forceN=" + F(StanceFootLowerReciprocalForceAbsMean) + "/lowerSupportA=" + F(StanceFootLowerReciprocalLowerSupportNormalAccelerationMean) +
                " | lowerSelfWeightTransfer=" + (StanceLowerSelfWeightTransferArm ?? "<none>") + "/on=" + StanceLowerSelfWeightTransferEnabled + "/applied=" + StanceLowerSelfWeightTransferAppliedSamples + "/forceN=" + F(StanceLowerSelfWeightTransferExpectedForceMean) + "/lowerSupportA=" + F(StanceLowerSelfWeightTransferLowerSupportAccelerationMean) +
                " | kneeSoftPD=" + F(KneeSoftOffAxisAngularSpring) + "/" + F(KneeSoftOffAxisAngularDamper) + "/base=" + F(KneeSoftOffAxisMaxAngularAcceleration) + "/source=" + F(KneeSoftOffAxisSourceTurnMaxAngularAcceleration) + "/recovery=" + F(KneeSoftOffAxisRecoveryMaxAngularAcceleration) +
                " | rigidbodyInertia=" + (RigidbodyInertiaFoundationMode ?? "<none>") +
                "/audit=" + RigidbodyInertiaAuditCompleted +
                "/validated=" + RigidbodyInertiaBuildFoundationValidated +
                " | angDyn=" + (AngularDynamicsMode ?? "<none>") + "/valid=" + AngularDynamicsValid +
                "/hip(aT/vErr/P/D)=" + F(HipTargetRelativeAngularAccelerationMagnitude) + "/" + F(HipAngularVelocityErrorMagnitude) + "/" + F(HipPdPContributionMagnitude) + "/" + F(HipPdDContributionMagnitude) +
                "/knee(aT/vErr/P/D)=" + F(KneeTargetAxisAngularAccelerationSigned) + "/" + F(KneeAxisAngularVelocityErrorSigned) + "/" + F(KneePdPContributionSigned) + "/" + F(KneePdDContributionSigned) +
                " | " + RuntimeContext.Status() + "/parentMatched=" + RuntimeParentMatched +
                " | timing=localFixed#" + FixedUpdateSequence + "/render#" + TargetRenderSampleSequence + "/fresh=" + TargetRenderSampleFreshAtFixedStep + "/age=" + F(TargetRenderSampleAgeSeconds) + "/held=" + FixedStepsSinceTargetRenderSample +
                " | anchor(hip/knee)=" + F(HipAnchorErrorMeters) + "/" + F(KneeAnchorErrorMeters) +
                " | clamps(hip/knee)=" + HipClampCount + "/" + KneeClampCount +
                " | trace(samples/hipMean/hipMax/kneeMean/kneeMax)=" + TraceSamples + "/" + F(TraceHipRotationMeanDegrees) + "/" + F(TraceHipRotationMaxDegrees) + "/" + F(TraceKneeRotationMeanDegrees) + "/" + F(TraceKneeRotationMaxDegrees) +
                " | traceKneeOff(mean/max/softMean/softMax/softClamp)=" + F(TraceKneeOffAxisErrorMeanDegrees) + "/" + F(TraceKneeOffAxisErrorMaxDegrees) + "/" + F(TraceKneeSoftOffAxisAppliedMean) + "/" + F(TraceKneeSoftOffAxisAppliedMax) + "/" + TraceKneeSoftOffAxisClampSamples +
                " | freeze=" + (SingleLegFreezeCertificationMode ?? "<none>") + "/complete=" + SingleLegFreezeCertificationComplete + "/production=" + SingleLegFreezeProductionContractPass + "/twoBody=" + SingleLegFreezeTwoBodyPass + "/runSafety=" + SingleLegFreezeRunSafetyPass + "/ready=" + SingleLegFreezeReady + "/reason=" + (SingleLegFreezeReason ?? "<none>") +
                " | abort=" + DriveAborted + "/" + (DriveAbortReason ?? "<none>") +
                " | stage=" + (Stage ?? "<none>");
        }


        public string GetFullText()
        {
            StringBuilder sb = new StringBuilder(8192);
            sb.AppendLine(RuntimeContext.Status() + "/parentMatched=" + RuntimeParentMatched);
            sb.AppendLine("MakemitAGA Active Ragdoll Lab 0.4.2.1 - Per-Side Frozen Single-Leg / Foot-Ankle Requalification Foundation Report");
            sb.AppendLine("stage=" + (Stage ?? "<none>"));
            sb.AppendLine("detail=" + (Detail ?? "<none>"));
            sb.AppendLine("built=" + Built);
            sb.AppendLine("driveEnabled=" + DriveEnabled);
            sb.AppendLine("finite=" + Finite);
            sb.AppendLine("side=" + (Side ?? "<none>"));
            sb.AppendLine("resolutionMode=" + (ResolutionMode ?? "<none>"));
            sb.AppendLine("upperLegPath=" + (UpperLegPath ?? "<none>"));
            sb.AppendLine("lowerLegPath=" + (LowerLegPath ?? "<none>"));
            sb.AppendLine("footPath=" + (FootPath ?? "<none>"));
            sb.AppendLine("hipCharacterJointPresent=" + HipCharacterJointPresent);
            sb.AppendLine("kneeCharacterJointPresent=" + KneeCharacterJointPresent);
            sb.AppendLine("kneeConstraintBackendMode=" + (KneeConstraintBackendMode ?? "<none>"));
            sb.AppendLine("kneeJointLimitReadbackDegrees=" + F(KneeLowTwistLimitReadbackDegrees) + "/" + F(KneeHighTwistLimitReadbackDegrees) + "/" + F(KneeSwing1LimitReadbackDegrees) + "/" + F(KneeSwing2LimitReadbackDegrees));
            sb.AppendLine("hipConnectedBodyMatch=" + HipConnectedBodyMatch);
            sb.AppendLine("hipMechanicalParentExperiment=" + (HipMechanicalParentExperiment ?? "<none>"));
            sb.AppendLine("hipMechanicalParentMode=" + (HipMechanicalParentMode ?? "<none>"));
            sb.AppendLine("hipMechanicalParentProxyPresent=" + HipMechanicalParentProxyPresent);
            sb.AppendLine("hipMechanicalParentUsesInertializedProxy=" + HipMechanicalParentUsesInertializedProxy);
            sb.AppendLine("hipMechanicalParentConnectedToProxy=" + HipMechanicalParentConnectedToProxy);
            sb.AppendLine("hipMechanicalParentSteps(observed/commanded/controlFrame)=" + HipMechanicalParentObservedFixedSteps + "/" + HipMechanicalParentCommandedFixedSteps + "/" + HipMechanicalParentControlFrameCommandedFixedSteps);
            sb.AppendLine("hipMechanicalParentCurrent(posErr/rawLag/controlErr/W/A/rawA/cmdW/cmdA)=" + F(HipMechanicalParentCurrentPositionErrorMeters) + "/" + F(HipMechanicalParentCurrentRawPelvisRotationLagDegrees) + "/" + F(HipMechanicalParentCurrentControlFrameRotationErrorDegrees) + "/" + F(HipMechanicalParentCurrentAngularSpeed) + "/" + F(HipMechanicalParentCurrentAngularAcceleration) + "/" + F(HipMechanicalParentCurrentRawPelvisAngularAcceleration) + "/" + F(HipMechanicalParentCurrentCommandedAngularSpeed) + "/" + F(HipMechanicalParentCurrentCommandedAngularAcceleration));
            sb.AppendLine("hipMechanicalParentPeak(posErr/rawLag/controlErr/W/A/rawA/cmdW/cmdA)=" + F(HipMechanicalParentPeakPositionErrorMeters) + "/" + F(HipMechanicalParentPeakRawPelvisRotationLagDegrees) + "/" + F(HipMechanicalParentPeakControlFrameRotationErrorDegrees) + "/" + F(HipMechanicalParentPeakAngularSpeed) + "/" + F(HipMechanicalParentPeakAngularAcceleration) + "/" + F(HipMechanicalParentPeakRawPelvisAngularAcceleration) + "/" + F(HipMechanicalParentPeakCommandedAngularSpeed) + "/" + F(HipMechanicalParentPeakCommandedAngularAcceleration));
            sb.AppendLine("hipMechanicalParentCommandClosure(currentWErr/currentAErr/maxARatio/strictContract/peakWErr/peakAErr/peakMaxARatio,poseAuditOK/tolerance/overMaxA)=" + F(HipMechanicalParentCommandClosureAngularVelocityError) + "/" + F(HipMechanicalParentCommandClosureAngularAccelerationError) + "/" + F(HipMechanicalParentCommandClosureMaxAccelerationRatio) + "/" + HipMechanicalParentCommandClosureAccelerationContractSatisfied + "/" + F(HipMechanicalParentPeakCommandClosureAngularVelocityError) + "/" + F(HipMechanicalParentPeakCommandClosureAngularAccelerationError) + "/" + F(HipMechanicalParentPeakCommandClosureMaxAccelerationRatio) + "," + HipMechanicalParentPoseDerivedAuditWithinTolerance + "/" + F(HipMechanicalParentPoseDerivedAuditTolerance) + "/" + F(HipMechanicalParentPoseDerivedCommandOverMaxA));
            sb.AppendLine("hipMechanicalParentKey=" + (HipMechanicalParentKey ?? "<none>"));
            sb.AppendLine("hipMechanicalParentTransportMode=" + (HipMechanicalParentTransportMode ?? "<none>"));
            sb.AppendLine("hipMechanicalParentTransportCurrent(strictEnabled/healthyArm/phaseOwnership/recoveryEnabled/active/recoveryActive/healthyActive/branch/reason/transport/relative/raw/applied/maxA/clamp)=" + HipMechanicalParentTransportEnabled + "/" + (HipMechanicalParentNormalTransportArm ?? "<none>") + "/" + (HipMechanicalParentNormalTransportScopeArm ?? "<none>") + "/" + HipMechanicalParentRecoveryTransportEnabled + "/" + HipMechanicalParentTransportActiveThisStep + "/" + HipMechanicalParentRecoveryTransportActiveThisStep + "/" + HipMechanicalParentHealthyNormalTransportActiveThisStep + "/" + (HipMechanicalParentTransportActiveBranch ?? "<none>") + "/" + (HipMechanicalParentTransportSuppressionReason ?? "<none>") + "/" + F(HipMechanicalParentTransportCurrentMagnitude) + "/" + F(HipMechanicalParentTransportCurrentRelativeCorrectionMagnitude) + "/" + F(HipMechanicalParentTransportCurrentCombinedRawMagnitude) + "/" + F(HipMechanicalParentTransportCurrentCombinedAppliedMagnitude) + "/" + F(HipMechanicalParentTransportCurrentMaxAngularAcceleration) + "/" + HipMechanicalParentTransportCombinedClamped);
            sb.AppendLine("hipMechanicalParentTransportSteps(total/active/strict/recovery/healthyEligible/healthyActive/healthyBypass/nonHealthyBlocked/disabled/real/inactiveFrame/invalid/clamp)=" + HipMechanicalParentTransportSteps + "/" + HipMechanicalParentTransportActiveSteps + "/" + HipMechanicalParentTransportStrictActiveSteps + "/" + HipMechanicalParentRecoveryTransportActiveSteps + "/" + HipMechanicalParentHealthyNormalEligibleSteps + "/" + HipMechanicalParentTransportNormalActiveSteps + "/" + HipMechanicalParentTransportNormalBypassSteps + "/" + HipMechanicalParentTransportScopeBlockedSteps + "/" + HipMechanicalParentTransportDisabledSteps + "/" + HipMechanicalParentTransportRealParentSteps + "/" + HipMechanicalParentTransportInactiveFrameSteps + "/" + HipMechanicalParentTransportInvalidSteps + "/" + HipMechanicalParentTransportClampSteps);
            sb.AppendLine("hipMechanicalParentTransportScopeBlocked(source/recovery/unhealthy)=" + HipMechanicalParentTransportScopeBlockedSourceSteps + "/" + HipMechanicalParentTransportScopeBlockedRecoverySteps + "/" + HipMechanicalParentTransportScopeBlockedUnhealthySteps);
            sb.AppendLine("hipMechanicalParentTransportPeak(transport/relative/raw/applied/transportToMaxA/rawToMaxA)=" + F(HipMechanicalParentTransportPeakMagnitude) + "/" + F(HipMechanicalParentTransportPeakRelativeCorrectionMagnitude) + "/" + F(HipMechanicalParentTransportPeakCombinedRawMagnitude) + "/" + F(HipMechanicalParentTransportPeakCombinedAppliedMagnitude) + "/" + F(HipMechanicalParentTransportPeakToMaxARatio) + "/" + F(HipMechanicalParentTransportPeakCombinedRawToMaxARatio));
            sb.AppendLine("hipMechanicalParentTransportLast=" + (HipMechanicalParentTransportLastEvidence ?? "<none>"));
            sb.AppendLine("hipMechanicalParentTransportKey=" + (HipMechanicalParentTransportKey ?? "<none>"));
            sb.AppendLine("hipAnchorLinearTransportMode=" + (HipAnchorLinearTransportMode ?? "<none>") + "/boundSource=" + (HipAnchorLinearTransportBoundSource ?? "<none>"));
            sb.AppendLine("hipAnchorLinearTransportCurrent(enabled/active/reason/raw/applied/maxA/clamp)=" + HipAnchorLinearTransportEnabled + "/" + HipAnchorLinearTransportActiveThisStep + "/" + (HipAnchorLinearTransportSuppressionReason ?? "<none>") + "/" + F(HipAnchorLinearTransportCurrentRawAcceleration) + "/" + F(HipAnchorLinearTransportCurrentAppliedAcceleration) + "/" + F(HipAnchorLinearTransportCurrentMaxAcceleration) + "/" + HipAnchorLinearTransportClampedThisStep);
            sb.AppendLine("hipAnchorLinearTransportInput(fixed/worldAppliedA)=" + HipAnchorLinearTransportCurrentFixed + "/" + V(HipAnchorLinearTransportCurrentAppliedAccelerationWorld));
            sb.AppendLine("hipAnchorLinearTransportSteps(total/active/disabled/real/inactiveFrame/invalid/clamp)=" + HipAnchorLinearTransportSteps + "/" + HipAnchorLinearTransportActiveSteps + "/" + HipAnchorLinearTransportDisabledSteps + "/" + HipAnchorLinearTransportRealParentSteps + "/" + HipAnchorLinearTransportInactiveFrameSteps + "/" + HipAnchorLinearTransportInvalidSteps + "/" + HipAnchorLinearTransportClampSteps);
            sb.AppendLine("hipAnchorLinearTransportPeak(raw/applied/rawToBound)=" + F(HipAnchorLinearTransportPeakRawAcceleration) + "/" + F(HipAnchorLinearTransportPeakAppliedAcceleration) + "/" + F(HipAnchorLinearTransportPeakRawToBoundRatio));
            sb.AppendLine("hipAnchorLinearTransportLast=" + (HipAnchorLinearTransportLastEvidence ?? "<none>"));
            sb.AppendLine("hipAnchorLinearTransportKey=" + (HipAnchorLinearTransportKey ?? "<none>"));
            sb.AppendLine("kneeConnectedBodyMatch=" + KneeConnectedBodyMatch);
            sb.AppendLine("kneeAxisMode=" + (KneeAxisMode ?? "<none>"));
            sb.AppendLine("kneeFlexionAxisLowerLocal=" + V(KneeFlexionAxisLowerLocal));
            sb.AppendLine("kneeFlexionAxisWorld=" + V(KneeFlexionAxisWorld));
            sb.AppendLine("kneeAxisCalibrationCollecting=" + KneeAxisCalibrationCollecting);
            sb.AppendLine("kneeAxisCalibrationReady=" + KneeAxisCalibrationReady);
            sb.AppendLine("kneeAxisCalibrationReason=" + (KneeAxisCalibrationReason ?? "<none>"));
            sb.AppendLine("kneeAxisCalibrationSampleCount=" + KneeAxisCalibrationSampleCount);
            sb.AppendLine("kneeAxisCalibrationPrincipalEnergyRatio=" + F(KneeAxisCalibrationPrincipalEnergyRatio));
            sb.AppendLine("kneeAxisCalibrationAngleFromInitialDegrees=" + F(KneeAxisCalibrationAngleFromInitialDegrees));
            sb.AppendLine("hipRotationErrorDegrees=" + F(HipRotationErrorDegrees));
            sb.AppendLine("kneeRotationErrorDegrees=" + F(KneeRotationErrorDegrees));
            sb.AppendLine("kneeAxisErrorDegrees=" + F(KneeAxisErrorDegrees));
            sb.AppendLine("kneeOffAxisErrorDegrees=" + F(KneeOffAxisErrorDegrees));
            sb.AppendLine("hipControlRotationErrorDegrees=" + F(HipControlRotationErrorDegrees));
            sb.AppendLine("kneeControlAxisErrorDegrees=" + F(KneeControlAxisErrorDegrees));
            sb.AppendLine("kneeTargetRelativeAngularSpeedTotal=" + F(KneeTargetRelativeAngularSpeedTotal));
            sb.AppendLine("kneeTargetRelativeAngularSpeedAlongFlexionAxis=" + F(KneeTargetRelativeAngularSpeedAlongFlexionAxis));
            sb.AppendLine("kneeTargetRelativeAngularSpeedOffAxis=" + F(KneeTargetRelativeAngularSpeedOffAxis));
            sb.AppendLine("kneeAxisExplainedRatio=" + F(KneeAxisExplainedRatio));
            sb.AppendLine("transportAttributionValid=" + TransportAttributionValid);
            sb.AppendLine("hipRelativeComTransportErrorMagnitude=" + F(HipRelativeComTransportErrorMagnitude));
            sb.AppendLine("kneeRelativeComTransportErrorMagnitude=" + F(KneeRelativeComTransportErrorMagnitude));
            sb.AppendLine("legAngularAccelerationFeedforwardEnabled=" + LegAngularAccelerationFeedforwardEnabled);
            sb.AppendLine("legAngularAccelerationFeedforwardMode=" + (LegAngularAccelerationFeedforwardMode ?? "<none>"));
            sb.AppendLine("legAngularAccelerationFeedforwardGateMode=" + (LegAngularAccelerationFeedforwardGateMode ?? "<none>"));
            sb.AppendLine("legAngularAccelerationFeedforwardGain=" + F(LegAngularAccelerationFeedforwardGain));
            sb.AppendLine("legAngularAccelerationFeedforwardMaxAcceleration=" + F(LegAngularAccelerationFeedforwardMaxAcceleration));
            sb.AppendLine("legAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds=" + F(LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds));
            sb.AppendLine("hipAngularAccelerationFeedforwardAppliedMagnitude=" + F(HipAngularAccelerationFeedforwardAppliedMagnitude));
            sb.AppendLine("kneeAngularAccelerationFeedforwardAppliedSigned=" + F(KneeAngularAccelerationFeedforwardAppliedSigned));
            sb.AppendLine("hipAngularAccelerationFeedforwardClampCount=" + HipAngularAccelerationFeedforwardClampCount);
            sb.AppendLine("kneeAngularAccelerationFeedforwardClampCount=" + KneeAngularAccelerationFeedforwardClampCount);
            sb.AppendLine("kneeTorqueTopologyMode=CHILD_ONLY");
            sb.AppendLine("kneeLowerAppliedAngularAccelerationMagnitude=" + F(KneeLowerAppliedAngularAccelerationMagnitude));
            sb.AppendLine("kneeFlexionAppliedAngularAccelerationMagnitude=" + F(KneeFlexionAppliedAngularAccelerationMagnitude));
            sb.AppendLine("kneeSoftOffAxisControlEnabled=" + KneeSoftOffAxisControlEnabled);
            sb.AppendLine("kneeSoftOffAxisPD=" + F(KneeSoftOffAxisAngularSpring) + "/" + F(KneeSoftOffAxisAngularDamper) + "/baseline=" + F(KneeSoftOffAxisMaxAngularAcceleration) + "/source=" + F(KneeSoftOffAxisSourceTurnMaxAngularAcceleration) + "/recovery=" + F(KneeSoftOffAxisRecoveryMaxAngularAcceleration));
            sb.AppendLine("kneeSoftOffAxisAuthorityMode=" + (KneeSoftOffAxisAuthorityMode ?? "<none>"));
            sb.AppendLine("kneeSoftOffAxisAuthorityPhase=" + (KneeSoftOffAxisAuthorityPhase ?? "<none>"));
            sb.AppendLine("kneeSoftOffAxisAuthorityCurrentMaxAngularAcceleration=" + F(KneeSoftOffAxisAuthorityCurrentMaxAngularAcceleration));
            sb.AppendLine("kneeSoftOffAxisAuthorityBoostActive=" + KneeSoftOffAxisAuthorityBoostActive);
            sb.AppendLine("kneeSoftOffAxisAuthorityFixedSteps(source/recovery)=" + KneeSoftOffAxisSourceBoostFixedSteps + "/" + KneeSoftOffAxisRecoveryBoostFixedSteps);
            sb.AppendLine("kneeSoftPhysicalRecovery(pending/origin/recoveryBoost/rawErr/pendingBaselineSteps/healthBaselineSteps)=" + KneeSoftOffAxisPhysicalRecoveryPending + "/" + (KneeSoftOffAxisPhysicalRecoveryOrigin ?? "<none>") + "/" + KneeSoftOffAxisRecoveryBoostActive + "/" + F(KneeSoftOffAxisRecoveryErrorDegrees) + "/" + KneeSoftOffAxisPhysicalRecoveryPendingBaselineFixedSteps + "/" + KneeSoftOffAxisTrackingUnhealthyBaselineFixedSteps);
            sb.AppendLine("kneeTrackingHealth(unhealthy/candidate/reentry/immediate/peakEntryDeg)=" + KneeSoftOffAxisTrackingHealthUnhealthy + "/" + F(KneeSoftOffAxisTrackingHealthCandidateSeconds) + "/" + KneeSoftOffAxisTrackingHealthReentryCount + "/" + KneeSoftOffAxisTrackingHealthImmediateReentryCount + "/" + F(KneeSoftOffAxisTrackingHealthPeakEntryErrorDegrees));
            sb.AppendLine("kneeSoftOffAxisPhysicalRecovery(start/complete/boostTimeout/censored)=" + KneeSoftOffAxisRecoveryStarted + "/" + KneeSoftOffAxisRecoveryCompleted + "/" + KneeSoftOffAxisRecoveryTimedOut + "/" + KneeSoftOffAxisRecoveryCensoredByNewSource);
            sb.AppendLine("kneeSoftOffAxisAngularVelocityErrorMagnitude=" + F(KneeSoftOffAxisAngularVelocityErrorMagnitude));
            sb.AppendLine("kneeSoftOffAxisPContributionMagnitude=" + F(KneeSoftOffAxisPContributionMagnitude));
            sb.AppendLine("kneeSoftOffAxisDContributionMagnitude=" + F(KneeSoftOffAxisDContributionMagnitude));
            sb.AppendLine("kneeSoftOffAxisRawAccelerationMagnitude=" + F(KneeSoftOffAxisRawAccelerationMagnitude));
            sb.AppendLine("kneeSoftOffAxisAppliedAccelerationMagnitude=" + F(KneeSoftOffAxisAppliedAccelerationMagnitude));
            sb.AppendLine("kneeSoftOffAxisAccelerationClamped=" + KneeSoftOffAxisAccelerationClamped);
            sb.AppendLine("kneeSoftOffAxisClampCount=" + KneeSoftOffAxisClampCount);
            sb.AppendLine("hipAuthorityMode=" + (HipAuthorityMode ?? "<none>"));
            sb.AppendLine("hipAuthorityPhase/currentMaxA/rawRecoveryErr/boost=" + (HipAuthorityPhase ?? "<none>") + "/" + F(HipAuthorityCurrentMaxAngularAcceleration) + "/" + F(HipAuthorityRecoveryErrorDegrees) + "/" + HipAuthorityBoostActive);
            sb.AppendLine("hipPhysicalRecovery(pending/origin/recoveryBoost/pendingBaselineSteps/healthBaselineSteps)=" + HipPhysicalRecoveryPending + "/" + (HipPhysicalRecoveryOrigin ?? "<none>") + "/" + HipRecoveryBoostActive + "/" + HipAuthorityPhysicalRecoveryPendingBaselineFixedSteps + "/" + HipAuthorityTrackingUnhealthyBaselineFixedSteps);
            sb.AppendLine("hipTrackingHealth(unhealthy/candidate/reentry/immediate/peakEntryDeg)=" + HipTrackingHealthUnhealthy + "/" + F(HipTrackingHealthCandidateSeconds) + "/" + HipTrackingHealthReentryCount + "/" + HipTrackingHealthImmediateReentryCount + "/" + F(HipTrackingHealthPeakEntryErrorDegrees));
            sb.AppendLine("hipAuthorityMaxA(base/source/recovery)=" + F(HipBaselineMaxAngularAcceleration) + "/" + F(HipSourceTurnMaxAngularAcceleration) + "/" + F(HipRecoveryMaxAngularAcceleration));
            sb.AppendLine("hipAuthoritySteps(base/source/recovery)=" + HipAuthorityBaselineFixedSteps + "/" + HipAuthoritySourceBoostFixedSteps + "/" + HipAuthorityRecoveryBoostFixedSteps);
            sb.AppendLine("hipAuthorityClamps(base/source/recovery)=" + HipAuthorityBaselineClampSamples + "/" + HipAuthoritySourceClampSamples + "/" + HipAuthorityRecoveryClampSamples);
            sb.AppendLine("hipPhysicalRecovery(start/completed/boostTimedOut/censoredByNewSource)=" + HipAuthorityRecoveryStarted + "/" + HipAuthorityRecoveryCompleted + "/" + HipAuthorityRecoveryTimedOut + "/" + HipAuthorityRecoveryCensoredByNewSource);
            sb.AppendLine("hipControlFrameMode=" + (HipControlFrameMode ?? "<none>") + "/enabled=" + HipControlFrameInertializationEnabled + "/phase=" + (HipControlFramePhase ?? "<none>") + "/active=" + HipControlFrameActive);
            sb.AppendLine("hipControlFrameCurrent(lag/preGuardLag/strictMechanicalLag/virtualW/realW/wErr)=" + F(HipControlFrameRotationLagDegrees) + "/" + F(HipControlFramePreGuardRotationLagDegrees) + "/" + F(HipControlFrameStrictMechanicalRotationLagDegrees) + "/" + F(HipControlFrameVirtualAngularSpeed) + "/" + F(HipControlFrameRealPelvisAngularSpeed) + "/" + F(HipControlFrameAngularVelocityError));
            sb.AppendLine("hipControlFrameStrictMechanical(decoupled/W/A/peakLag)=" + HipControlFrameLagGuardDecoupledFromMechanical + "/" + F(HipControlFrameStrictMechanicalAngularVelocityWorld.magnitude) + "/" + F(HipControlFrameStrictMechanicalAngularAccelerationWorld.magnitude) + "/" + F(HipControlFramePeakStrictMechanicalRotationLagDegrees));
            sb.AppendLine("hipControlFrameIntegrator=" + (HipControlFrameIntegratorMode ?? "<none>") + "/active=" + HipControlFrameStrictIntegratorActiveThisStep + "/contract(speed/accel/inherited)=" + HipControlFrameSpeedContractSatisfiedThisStep + "/" + HipControlFrameAccelerationContractSatisfiedThisStep + "/" + HipControlFrameInheritedSpeedOverLimitThisStep + "/limitW-A=" + F(HipControlFrameIntegratorCurrentMaxAngularSpeed) + "/" + F(HipControlFrameIntegratorCurrentMaxAngularAcceleration) + "/virtualA=" + F(HipControlFrameVirtualAngularAcceleration));
            sb.AppendLine("hipControlFrameIntegratorPeak(virtualA/Wratio/Aratio/brakingSteps/inheritedOverSpeedSteps)=" + F(HipControlFramePeakVirtualAngularAcceleration) + "/" + F(HipControlFramePeakVirtualSpeedToLimitRatio) + "/" + F(HipControlFramePeakVirtualAccelerationToLimitRatio) + "/" + HipControlFrameBrakingLimitedFixedSteps + "/" + HipControlFrameInheritedSpeedOverLimitFixedSteps);
            sb.AppendLine("hipControlFrameActivationSync(applied/count/poseDeg/wDelta/peakPose/peakW)=" + HipControlFrameActivationSyncAppliedThisStep + "/" + HipControlFrameActivationSyncCount + "/" + F(HipControlFrameActivationSyncPoseCorrectionDegrees) + "/" + F(HipControlFrameActivationSyncAngularVelocityCorrection) + "/" + F(HipControlFramePeakActivationSyncPoseCorrectionDegrees) + "/" + F(HipControlFramePeakActivationSyncAngularVelocityCorrection));
            sb.AppendLine("hipControlFrameMechanicalClosureSync(applied/count/poseDeg/wDelta/peakPose/peakW)=" + HipControlFrameMechanicalClosureSyncAppliedThisStep + "/" + HipControlFrameMechanicalClosureSyncCount + "/" + F(HipControlFrameMechanicalClosureSyncPoseCorrectionDegrees) + "/" + F(HipControlFrameMechanicalClosureSyncAngularVelocityCorrection) + "/" + F(HipControlFramePeakMechanicalClosureSyncPoseCorrectionDegrees) + "/" + F(HipControlFramePeakMechanicalClosureSyncAngularVelocityCorrection));
            sb.AppendLine("hipControlFrameLagGuardRebase(mode/arm/enabled)=" + (HipControlFrameReferenceLagGuardRebaseMode ?? "<none>") + "/" + (HipControlFrameReferenceLagGuardRebaseArm ?? "<none>") + "/" + HipControlFrameReferenceLagGuardRebaseEnabled);
            sb.AppendLine("hipControlFrameLagBound(max/currentApplied/currentCorrection/steps/peakPre/peakCorrection/totalCorrection)=" + F(HipControlFrameMaxLagDegrees) + "/" + HipControlFrameLagBoundAppliedThisStep + "/" + F(HipControlFrameLagBoundCorrectionDegrees) + "/" + HipControlFrameLagBoundCorrectionFixedSteps + "/" + F(HipControlFramePeakPreGuardRotationLagDegrees) + "/" + F(HipControlFramePeakLagBoundCorrectionDegrees) + "/" + F(HipControlFrameTotalLagBoundCorrectionDegrees));
            sb.AppendLine("hipControlFrameLagGuardSuppressed(steps/current/peak/total)=" + HipControlFrameLagGuardSuppressedFixedSteps + "/" + F(HipControlFrameCurrentSuppressedLagBoundCorrectionDegrees) + "/" + F(HipControlFramePeakSuppressedLagBoundCorrectionDegrees) + "/" + F(HipControlFrameTotalSuppressedLagBoundCorrectionDegrees));
            sb.AppendLine("hipControlFrameSteps(base/source/recovery)=" + HipControlFrameBaselineFixedSteps + "/" + HipControlFrameSourceFixedSteps + "/" + HipControlFrameRecoveryFixedSteps);
            sb.AppendLine("hipControlFrameLimited(speed/accel)=" + HipControlFrameSpeedLimitedFixedSteps + "/" + HipControlFrameAccelerationLimitedFixedSteps);
            sb.AppendLine("hipControlFrameRecovery(start/complete/timeout/censored)=" + HipControlFrameRecoveryStarted + "/" + HipControlFrameRecoveryCompleted + "/" + HipControlFrameRecoveryTimedOut + "/" + HipControlFrameRecoveryCensoredByNewSource);
            sb.AppendLine("hipControlFramePeak(lag/virtualW/realW/wErr)=" + F(HipControlFramePeakRotationLagDegrees) + "/" + F(HipControlFramePeakVirtualAngularSpeed) + "/" + F(HipControlFramePeakRealPelvisAngularSpeed) + "/" + F(HipControlFramePeakAngularVelocityError));

            sb.AppendLine("hipMomentumRecoveryBrakingMode/arm=" + (HipMomentumRecoveryBrakingMode ?? "<none>") + "/" + (HipMomentumRecoveryBrakingArm ?? "<none>"));
            sb.AppendLine("hipMomentumRecoveryBraking(enabled/active,steps/eligible/active,entries/exitsToward/exitsRecoveryEnd/exitsHipRecovered)=" + HipMomentumRecoveryBrakingEnabled + "/" + HipMomentumRecoveryBrakingActiveThisStep + "," + HipMomentumRecoveryBrakingSteps + "/" + HipMomentumRecoveryBrakingRecoveryEligibleSteps + "/" + HipMomentumRecoveryBrakingActiveSteps + "," + HipMomentumRecoveryBrakingEntries + "/" + HipMomentumRecoveryBrakingExitTowardCount + "/" + HipMomentumRecoveryBrakingExitRecoveryEndedCount + "/" + HipMomentumRecoveryBrakingExitHipRecoveredCount);
            sb.AppendLine("hipMomentumRecoveryBrakingCurrent(hip/awayDegS/extraA/baseMaxA/effectiveMaxA/origin)=" + F(HipMomentumRecoveryBrakingCurrentHipDegrees) + "/" + F(HipMomentumRecoveryBrakingCurrentAwaySpeedDegPerSec) + "/" + F(HipMomentumRecoveryBrakingCurrentSupplementalAcceleration) + "/" + F(HipMomentumRecoveryBrakingCurrentProductionMaxA) + "/" + F(HipMomentumRecoveryBrakingEffectiveMaxA) + "/" + (HipMomentumRecoveryBrakingCurrentRecoveryOrigin ?? "<none>"));
            sb.AppendLine("hipMomentumRecoveryBrakingStats(meanExtra/peakExtra/peakAway/peakEffectiveMaxA/suppClamp,lastExit)=" + F(HipMomentumRecoveryBrakingMeanSupplementalAcceleration) + "/" + F(HipMomentumRecoveryBrakingPeakSupplementalAcceleration) + "/" + F(HipMomentumRecoveryBrakingPeakAwaySpeedDegPerSec) + "/" + F(HipMomentumRecoveryBrakingPeakEffectiveMaxA) + "/" + HipMomentumRecoveryBrakingSupplementalClampSteps + "," + (HipMomentumRecoveryBrakingLastExitReason ?? "<none>"));
            sb.AppendLine("hipMomentumRecoveryBrakingAbort=" + (HipMomentumRecoveryBrakingAbortEvidence ?? "<none>"));
            sb.AppendLine("hipMomentumRecoveryBrakingKey=" + (HipMomentumRecoveryBrakingKey ?? "<none>"));
            sb.AppendLine("hipVectorCaptureMode/arm=" + (HipVectorCaptureMode ?? "<none>") + "/" + (HipVectorCaptureArm ?? "<none>"));
            sb.AppendLine("hipVectorCapture(enabled/capture/sourceCarry,steps/eligible/active,entries/exits/carryEntries/carrySteps)=" + HipVectorCaptureEnabled + "/" + HipVectorCaptureActive + "/" + HipVectorCaptureSourceCarryActive + "," + HipVectorCaptureSteps + "/" + HipVectorCaptureEligibleSteps + "/" + HipVectorCaptureActiveSteps + "," + HipVectorCaptureEntries + "/" + HipVectorCapturePhysicalExitCount + "/" + HipVectorCaptureSourceCarryEntries + "/" + HipVectorCaptureSourceCarrySteps);
            sb.AppendLine("hipVectorCaptureActions(radialAway/radialTowardCapture/tangential/clamp)=" + HipVectorCaptureRadialAwayBrakeSteps + "/" + HipVectorCaptureRadialTowardCaptureBrakeSteps + "/" + HipVectorCaptureTangentialDampingSteps + "/" + HipVectorCaptureSupplementalClampSteps);
            sb.AppendLine("hipVectorCaptureHighErrorAwayFloor(enabled/floorA,gate/raised/satisfied/unsatisfied/tanTrim)=" + HipVectorCaptureHighErrorAwayBrakeFloorEnabled + "/" + F(HipVectorCaptureHighErrorAwayBrakeFloorA) + "," + HipVectorCaptureHighErrorAwayBrakeFloorGateSteps + "/" + HipVectorCaptureHighErrorAwayBrakeFloorRaisedSteps + "/" + HipVectorCaptureHighErrorAwayBrakeFloorSatisfiedSteps + "/" + HipVectorCaptureHighErrorAwayBrakeFloorUnsatisfiedSteps + "/" + HipVectorCaptureHighErrorAwayBrakeFloorTangentialTrimSteps);
            sb.AppendLine("hipVectorCaptureHighErrorAwayFloorCurrent(gate/satisfied,preRadial/postRadial/delta,meanDelta/peakDelta)=" + HipVectorCaptureCurrentHighErrorAwayBrakeFloorGate + "/" + HipVectorCaptureCurrentHighErrorAwayBrakeFloorSatisfied + "," + F(HipVectorCaptureCurrentPreFloorRadialCommandA) + "/" + F(HipVectorCaptureCurrentPostFloorRadialCommandA) + "/" + F(HipVectorCaptureCurrentHighErrorAwayBrakeFloorDeltaA) + "," + F(HipVectorCaptureMeanHighErrorAwayBrakeFloorDeltaA) + "/" + F(HipVectorCapturePeakHighErrorAwayBrakeFloorDeltaA));
            sb.AppendLine("hipVectorCaptureHighErrorAwayFloorKey=" + (HipVectorCaptureHighErrorAwayBrakeFloorKey ?? "<none>"));
            sb.AppendLine("hipVectorCaptureCurrent(error/rW/tW/totalW/stopDeg/stopRatio/rA/tA/suppA/maxA)=" + F(HipVectorCaptureCurrentErrorDegrees) + "/" + F(HipVectorCaptureCurrentRadialWSignedDegPerSec) + "/" + F(HipVectorCaptureCurrentTangentialWDegPerSec) + "/" + F(HipVectorCaptureCurrentTotalWDegPerSec) + "/" + F(HipVectorCaptureCurrentStoppingAngleDegrees) + "/" + F(HipVectorCaptureCurrentStoppingRatio) + "/" + F(HipVectorCaptureCurrentRadialSupplementalA) + "/" + F(HipVectorCaptureCurrentTangentialSupplementalA) + "/" + F(HipVectorCaptureCurrentSupplementalA) + "/" + F(HipVectorCaptureCurrentProductionMaxA) + "->" + F(HipVectorCaptureCurrentEffectiveMaxA));
            sb.AppendLine("hipVectorCaptureStats(mean/peak,radialMean/peak,tanMean/peak,peakWrad/peakWtan,stopDeg/ratio)=" + F(HipVectorCaptureMeanSupplementalA) + "/" + F(HipVectorCapturePeakSupplementalA) + "," + F(HipVectorCaptureMeanRadialSupplementalA) + "/" + F(HipVectorCapturePeakRadialSupplementalA) + "," + F(HipVectorCaptureMeanTangentialSupplementalA) + "/" + F(HipVectorCapturePeakTangentialSupplementalA) + "," + F(HipVectorCapturePeakAbsRadialWDegPerSec) + "/" + F(HipVectorCapturePeakTangentialWDegPerSec) + "," + F(HipVectorCapturePeakStoppingAngleDegrees) + "/" + F(HipVectorCapturePeakStoppingRatio));
            sb.AppendLine("hipVectorCaptureEvidence(origin/entry/exit/carry/abort)=" + (HipVectorCaptureCurrentRecoveryOrigin ?? "<none>") + "/" + (HipVectorCaptureLastEntryEvidence ?? "<none>") + "/" + (HipVectorCaptureLastExitReason ?? "<none>") + "/" + (HipVectorCaptureLastSourceCarryEvidence ?? "<none>") + "/" + (HipVectorCaptureAbortEvidence ?? "<none>"));
            sb.AppendLine("hipVectorCaptureKey=" + (HipVectorCaptureKey ?? "<none>"));
            sb.AppendLine("hipSourceShockAbsorber(mode/arm/enabled/active)=" + (HipSourceReferenceShockAbsorberMode ?? "<none>") + "/" + (HipSourceReferenceShockAbsorberArm ?? "<none>") + "/" + HipSourceReferenceShockAbsorberEnabled + "/" + HipSourceReferenceShockAbsorberActive);
            sb.AppendLine("hipSourceShockAbsorberSteps(total/source/recovery,activate/release,limitW/limitA/rawShock)=" + HipSourceReferenceShockAbsorberFixedSteps + "/" + HipSourceReferenceShockAbsorberSourceSteps + "/" + HipSourceReferenceShockAbsorberRecoverySteps + "," + HipSourceReferenceShockAbsorberActivations + "/" + HipSourceReferenceShockAbsorberReleases + "," + HipSourceReferenceShockAbsorberSpeedLimitedSteps + "/" + HipSourceReferenceShockAbsorberAccelerationLimitedSteps + "/" + HipSourceReferenceShockAbsorberRawPacketShockCount);
            sb.AppendLine("hipSourceShockAbsorberCurrent(rawGap/step/W/maxW/A/maxA)=" + F(HipSourceReferenceShockAbsorberCurrentRawGapDegrees) + "/" + F(HipSourceReferenceShockAbsorberCurrentOutputStepDegrees) + "/" + F(HipSourceReferenceShockAbsorberCurrentAngularSpeed) + "/" + F(HipSourceReferenceShockAbsorberCurrentMaxAngularSpeed) + "/" + F(HipSourceReferenceShockAbsorberCurrentAngularAcceleration) + "/" + F(HipSourceReferenceShockAbsorberCurrentMaxAngularAcceleration));
            sb.AppendLine("hipSourceShockAbsorberPeak(rawPacket/rawGap/step/W/A/Aratio,contractW/A)=" + F(HipSourceReferenceShockAbsorberPeakRawPacketDeltaDegrees) + "/" + F(HipSourceReferenceShockAbsorberPeakRawGapDegrees) + "/" + F(HipSourceReferenceShockAbsorberPeakOutputStepDegrees) + "/" + F(HipSourceReferenceShockAbsorberPeakAngularSpeed) + "/" + F(HipSourceReferenceShockAbsorberPeakAngularAcceleration) + "/" + F(HipSourceReferenceShockAbsorberPeakAccelerationRatio) + "," + HipSourceReferenceShockAbsorberSpeedContractSatisfied + "/" + HipSourceReferenceShockAbsorberAccelerationContractSatisfied);
            sb.AppendLine("hipSourceShockAbsorberEvidence(last/abort)=" + (HipSourceReferenceShockAbsorberLastEvidence ?? "<none>") + "/" + (HipSourceReferenceShockAbsorberAbortEvidence ?? "<none>"));
            sb.AppendLine("hipSourceShockAbsorberKey=" + (HipSourceReferenceShockAbsorberKey ?? "<none>"));
            sb.AppendLine("hipCompositeShockAbsorber(mode/arm/enabled/active)=" + (HipCompositeReferenceShockAbsorberMode ?? "<none>") + "/" + (HipCompositeReferenceShockAbsorberArm ?? "<none>") + "/" + HipCompositeReferenceShockAbsorberEnabled + "/" + HipCompositeReferenceShockAbsorberActive);
            sb.AppendLine("hipCompositeShockAbsorberSteps(total/source/recovery,activate/release,limitW/limitA/inherited/inputSpike)=" + HipCompositeReferenceShockAbsorberFixedSteps + "/" + HipCompositeReferenceShockAbsorberSourceSteps + "/" + HipCompositeReferenceShockAbsorberRecoverySteps + "," + HipCompositeReferenceShockAbsorberActivations + "/" + HipCompositeReferenceShockAbsorberReleases + "," + HipCompositeReferenceShockAbsorberSpeedLimitedSteps + "/" + HipCompositeReferenceShockAbsorberAccelerationLimitedSteps + "/" + HipCompositeReferenceShockAbsorberInheritedOverSpeedSteps + "/" + HipCompositeReferenceShockAbsorberInputSpikeSteps);
            sb.AppendLine("hipCompositeShockAbsorberCurrent(inputGap/inputStep/inputW/inputA/outputStep/outputW/outputA/maxW/maxA/localComp)=" + F(HipCompositeReferenceShockAbsorberCurrentInputGapDegrees) + "/" + F(HipCompositeReferenceShockAbsorberCurrentInputStepDegrees) + "/" + F(HipCompositeReferenceShockAbsorberCurrentInputAngularSpeed) + "/" + F(HipCompositeReferenceShockAbsorberCurrentInputAngularAcceleration) + "/" + F(HipCompositeReferenceShockAbsorberCurrentOutputStepDegrees) + "/" + F(HipCompositeReferenceShockAbsorberCurrentOutputAngularSpeed) + "/" + F(HipCompositeReferenceShockAbsorberCurrentOutputAngularAcceleration) + "/" + F(HipCompositeReferenceShockAbsorberCurrentMaxAngularSpeed) + "/" + F(HipCompositeReferenceShockAbsorberCurrentMaxAngularAcceleration) + "/" + F(HipCompositeReferenceShockAbsorberCurrentLocalCompensationDegrees) + "/" + HipCompositeReferenceShockAbsorberCurrentUpstreamReferenceActive);
            sb.AppendLine("hipCompositeShockAbsorberPeak(inputGap/inputStep/inputW/inputA,outputStep/outputW/outputA/Aratio/localComp,contractW/A)=" + F(HipCompositeReferenceShockAbsorberPeakInputGapDegrees) + "/" + F(HipCompositeReferenceShockAbsorberPeakInputStepDegrees) + "/" + F(HipCompositeReferenceShockAbsorberPeakInputAngularSpeed) + "/" + F(HipCompositeReferenceShockAbsorberPeakInputAngularAcceleration) + "," + F(HipCompositeReferenceShockAbsorberPeakOutputStepDegrees) + "/" + F(HipCompositeReferenceShockAbsorberPeakOutputAngularSpeed) + "/" + F(HipCompositeReferenceShockAbsorberPeakOutputAngularAcceleration) + "/" + F(HipCompositeReferenceShockAbsorberPeakOutputAccelerationRatio) + "/" + F(HipCompositeReferenceShockAbsorberPeakLocalCompensationDegrees) + "," + HipCompositeReferenceShockAbsorberSpeedContractSatisfied + "/" + HipCompositeReferenceShockAbsorberAccelerationContractSatisfied);
            sb.AppendLine("hipCompositeShockAbsorberEvidence(last/abort)=" + (HipCompositeReferenceShockAbsorberLastEvidence ?? "<none>") + "/" + (HipCompositeReferenceShockAbsorberAbortEvidence ?? "<none>"));
            sb.AppendLine("hipCompositeShockAbsorberKey=" + (HipCompositeReferenceShockAbsorberKey ?? "<none>"));
            sb.AppendLine("hipCharacterJointResponseIsolation(mode/arm/enabled,jointPresent)=" + (HipCharacterJointResponseIsolationMode ?? "<none>") + "/" + (HipCharacterJointResponseIsolationArm ?? "<none>") + "/" + HipCharacterJointResponseIsolationEnabled + "," + HipCharacterJointPresent);
            sb.AppendLine("hipTranslationTetherSerialChain(mode/arm/upperOnly,forceAppsUpperLower)=" + (HipTranslationTetherSerialChainMode ?? "<none>") + "/" + (HipTranslationTetherSerialChainArm ?? "<none>") + "/" + HipTranslationTetherUpperOnlyEnabled + "," + HipTranslationTetherUpperForceApplications + "/" + HipTranslationTetherLowerForceApplications);
            sb.AppendLine("hipCharacterJointResponseIsolationTether(steps/clamped/invalidAnchor,kp/kd/maxA/validity)=" + HipCharacterJointResponseIsolationTetherSteps + "/" + HipCharacterJointResponseIsolationTetherClampedSteps + "/" + HipCharacterJointResponseIsolationInvalidAnchorSteps + "," + F(HipCharacterJointResponseIsolationTetherSpring) + "/" + F(HipCharacterJointResponseIsolationTetherDamper) + "/" + F(HipCharacterJointResponseIsolationTetherMaxAcceleration) + "/" + F(HipCharacterJointResponseIsolationAnchorValidityMeters));
            sb.AppendLine("hipCharacterJointResponseIsolationCurrent(anchorErr/velErr/pdA/ffA/rawA/appliedA/clamp)=" + F(HipCharacterJointResponseIsolationCurrentAnchorErrorMeters) + "/" + F(HipCharacterJointResponseIsolationCurrentVelocityError) + "/" + F(HipCharacterJointResponseIsolationCurrentPdAcceleration) + "/" + F(HipCharacterJointResponseIsolationCurrentFeedforwardAcceleration) + "/" + F(HipCharacterJointResponseIsolationCurrentRawAcceleration) + "/" + F(HipCharacterJointResponseIsolationCurrentAppliedAcceleration) + "/" + HipCharacterJointResponseIsolationCurrentClamped);
            sb.AppendLine("hipCharacterJointResponseIsolationPeak(anchorErr/velErr/pdA/ffA/rawA/appliedA)=" + F(HipCharacterJointResponseIsolationPeakAnchorErrorMeters) + "/" + F(HipCharacterJointResponseIsolationPeakVelocityError) + "/" + F(HipCharacterJointResponseIsolationPeakPdAcceleration) + "/" + F(HipCharacterJointResponseIsolationPeakFeedforwardAcceleration) + "/" + F(HipCharacterJointResponseIsolationPeakRawAcceleration) + "/" + F(HipCharacterJointResponseIsolationPeakAppliedAcceleration));
            sb.AppendLine("hipCharacterJointResponseIsolationEvidence(last/abort)=" + (HipCharacterJointResponseIsolationLastEvidence ?? "<none>") + "/" + (HipCharacterJointResponseIsolationAbortEvidence ?? "<none>"));
            sb.AppendLine("hipCharacterJointResponseIsolationKey=" + (HipCharacterJointResponseIsolationKey ?? "<none>"));
            sb.AppendLine("kneeParentAngularTransport(mode/arm/enabled)=" + (KneeParentAngularTransportMode ?? "<none>") + "/" + (KneeParentAngularTransportArm ?? "<none>") + "/" + KneeParentAngularTransportEnabled);
            sb.AppendLine("kneeParentAngularTransportSteps(active/total/invalid)=" + KneeParentAngularTransportActiveSteps + "/" + KneeParentAngularTransportSteps + "/" + KneeParentAngularTransportInvalidSteps);
            sb.AppendLine("kneeParentAngularTransportCurrent(transport/relative/totalA)=" + F(KneeParentAngularTransportCurrentTransportAcceleration) + "/" + F(KneeParentAngularTransportCurrentRelativeAcceleration) + "/" + F(KneeParentAngularTransportCurrentTotalLowerAcceleration));
            sb.AppendLine("kneeParentAngularTransportPeak(transport/relative/totalA)=" + F(KneeParentAngularTransportPeakTransportAcceleration) + "/" + F(KneeParentAngularTransportPeakRelativeAcceleration) + "/" + F(KneeParentAngularTransportPeakTotalLowerAcceleration));
            sb.AppendLine("kneeParentAngularTransportLast=" + (KneeParentAngularTransportLastEvidence ?? "<none>"));
            sb.AppendLine("kneeParentAngularTransportKey=" + (KneeParentAngularTransportKey ?? "<none>"));
            sb.AppendLine("kneeJointIsolation=" + (KneeCharacterJointResponseIsolationMode ?? "<none>") + "/" + (KneeCharacterJointResponseIsolationArm ?? "<none>") + "/enabled=" + KneeCharacterJointResponseIsolationEnabled);
            sb.AppendLine("kneeJointIsolationTether(steps/force/clamped/invalid)=" + KneeCharacterJointResponseIsolationTetherSteps + "/" + KneeCharacterJointResponseIsolationForceApplications + "/" + KneeCharacterJointResponseIsolationTetherClampedSteps + "/" + KneeCharacterJointResponseIsolationInvalidAnchorSteps);
            sb.AppendLine("kneeJointIsolationCurrent(anchorErr/velErr/rawA/appliedA/clamped)=" + F(KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters) + "/" + F(KneeCharacterJointResponseIsolationCurrentVelocityError) + "/" + F(KneeCharacterJointResponseIsolationCurrentRawAcceleration) + "/" + F(KneeCharacterJointResponseIsolationCurrentAppliedAcceleration) + "/" + KneeCharacterJointResponseIsolationCurrentClamped);
            sb.AppendLine("kneeJointIsolationPeak(anchorErr/velErr/rawA/appliedA)=" + F(KneeCharacterJointResponseIsolationPeakAnchorErrorMeters) + "/" + F(KneeCharacterJointResponseIsolationPeakVelocityError) + "/" + F(KneeCharacterJointResponseIsolationPeakRawAcceleration) + "/" + F(KneeCharacterJointResponseIsolationPeakAppliedAcceleration));
            sb.AppendLine("kneeJointIsolationConfig(kp/kd/maxA/validM)=" + F(KneeCharacterJointResponseIsolationTetherSpring) + "/" + F(KneeCharacterJointResponseIsolationTetherDamper) + "/" + F(KneeCharacterJointResponseIsolationTetherMaxAcceleration) + "/" + F(KneeCharacterJointResponseIsolationAnchorValidityMeters));
            sb.AppendLine("kneeJointIsolationLast=" + (KneeCharacterJointResponseIsolationLastEvidence ?? "<none>"));
            sb.AppendLine("kneeJointIsolationAbort=" + (KneeCharacterJointResponseIsolationAbortEvidence ?? "<none>"));
            sb.AppendLine("kneeJointIsolationKey=" + (KneeCharacterJointResponseIsolationKey ?? "<none>"));
            sb.AppendLine("kneeAnchorAccelFF=" + (KneeAnchorAccelerationFeedforwardMode ?? "<none>") + "/" + (KneeAnchorAccelerationFeedforwardArm ?? "<none>") + "/enabled=" + KneeAnchorAccelerationFeedforwardEnabled + "/valid=" + KneeAnchorAccelerationFeedforwardCurrentValid);
            sb.AppendLine("kneeAnchorAccelFFSteps(active/warmup/total/invalid)=" + KneeAnchorAccelerationFeedforwardActiveSteps + "/" + KneeAnchorAccelerationFeedforwardWarmupSteps + "/" + KneeAnchorAccelerationFeedforwardSteps + "/" + KneeAnchorAccelerationFeedforwardInvalidSteps);
            sb.AppendLine("kneeAnchorAccelFFCurrent(parentV/ffA)=" + F(KneeAnchorAccelerationFeedforwardCurrentParentAnchorSpeed) + "/" + F(KneeAnchorAccelerationFeedforwardCurrentAcceleration));
            sb.AppendLine("kneeAnchorAccelFFPeak(parentV/ffA)=" + F(KneeAnchorAccelerationFeedforwardPeakParentAnchorSpeed) + "/" + F(KneeAnchorAccelerationFeedforwardPeakAcceleration));
            sb.AppendLine("kneeAnchorAccelFFLast=" + (KneeAnchorAccelerationFeedforwardLastEvidence ?? "<none>"));
            sb.AppendLine("kneeAnchorAccelFFAbort=" + (KneeAnchorAccelerationFeedforwardAbortEvidence ?? "<none>"));
            sb.AppendLine("kneeAnchorAccelFFKey=" + (KneeAnchorAccelerationFeedforwardKey ?? "<none>"));
            sb.AppendLine("kneeFlexionAllocation=" + (KneeFlexionAllocationMode ?? "<none>") + "/" + (KneeFlexionAllocationArm ?? "<none>") + "/enabled=" + KneeFlexionAllocationEnabled);
            sb.AppendLine("kneeFlexionAllocationSteps(active/total/invalid,opposing/suppressed/same,clampOriginal/final)=" + KneeFlexionAllocationActiveSteps + "/" + KneeFlexionAllocationSteps + "/" + KneeFlexionAllocationInvalidSteps + "," + KneeFlexionAllocationOpposingSteps + "/" + KneeFlexionAllocationSuppressedOpposingFfSteps + "/" + KneeFlexionAllocationSameDirectionSteps + "," + KneeFlexionAllocationOriginalWouldClampSteps + "/" + KneeFlexionAllocationFinalClampSteps);
            sb.AppendLine("kneeFlexionAllocationCurrent(PD/ffIn/ffEff/raw/allocated/applied/maxA,opposing/originalClamp/finalClamp)=" + F(KneeFlexionAllocationCurrentPd) + "/" + F(KneeFlexionAllocationCurrentInputFf) + "/" + F(KneeFlexionAllocationCurrentEffectiveFf) + "/" + F(KneeFlexionAllocationCurrentOriginalRaw) + "/" + F(KneeFlexionAllocationCurrentAllocatedRaw) + "/" + F(KneeFlexionAllocationCurrentApplied) + "/" + F(KneeFlexionAllocationCurrentMaxA) + "," + KneeFlexionAllocationCurrentOpposing + "/" + KneeFlexionAllocationCurrentOriginalWouldClamp + "/" + KneeFlexionAllocationCurrentFinalClamped);
            sb.AppendLine("kneeFlexionAllocationPeak(PD/ffIn/raw/allocated/applied,suppressedFfMean)=" + F(KneeFlexionAllocationPeakPdAbs) + "/" + F(KneeFlexionAllocationPeakInputFfAbs) + "/" + F(KneeFlexionAllocationPeakOriginalRawAbs) + "/" + F(KneeFlexionAllocationPeakAllocatedRawAbs) + "/" + F(KneeFlexionAllocationPeakAppliedAbs) + "," + F(KneeFlexionAllocationSuppressedFfAbsMean));
            sb.AppendLine("kneeFlexionAllocationLast=" + (KneeFlexionAllocationLastEvidence ?? "<none>"));
            sb.AppendLine("kneeFlexionAllocationKey=" + (KneeFlexionAllocationKey ?? "<none>"));
            sb.AppendLine("hipAnchorAccelFF=" + (HipAnchorAccelerationFeedforwardMode ?? "<none>") + "/" + (HipAnchorAccelerationFeedforwardArm ?? "<none>") + "/enabled=" + HipAnchorAccelerationFeedforwardEnabled + "/valid=" + HipAnchorAccelerationFeedforwardCurrentValid);
            sb.AppendLine("hipAnchorAccelFFSteps(active/warmup/strictSuppressed/total/invalid)=" + HipAnchorAccelerationFeedforwardActiveSteps + "/" + HipAnchorAccelerationFeedforwardWarmupSteps + "/" + HipAnchorAccelerationFeedforwardStrictLinearTransportSuppressedSteps + "/" + HipAnchorAccelerationFeedforwardSteps + "/" + HipAnchorAccelerationFeedforwardInvalidSteps);
            sb.AppendLine("hipAnchorAccelFFCurrent(settledV/cmdV/candidateA/appliedA)=" + F(HipAnchorAccelerationFeedforwardCurrentSettledParentSpeed) + "/" + F(HipAnchorAccelerationFeedforwardCurrentCommandedParentSpeed) + "/" + F(HipAnchorAccelerationFeedforwardCurrentCandidateAcceleration) + "/" + F(HipAnchorAccelerationFeedforwardCurrentAppliedAcceleration));
            sb.AppendLine("hipAnchorAccelFFSignedCurrent(sourceToChildL/V/F,sourceToParentF,parentToChildF,vErrF)=" + F(HipAnchorAccelerationFeedforwardCurrentSourceToChildLateral) + "/" + F(HipAnchorAccelerationFeedforwardCurrentSourceToChildVertical) + "/" + F(HipAnchorAccelerationFeedforwardCurrentSourceToChildForward) + "," + F(HipAnchorAccelerationFeedforwardCurrentSourceToParentForward) + "/" + F(HipAnchorAccelerationFeedforwardCurrentParentToChildForward) + "/" + F(HipAnchorAccelerationFeedforwardCurrentForwardVelocityError));
            sb.AppendLine("hipAnchorAccelFFSignedPeakAbs(sourceToChildF/L/V,sourceToParentF,parentToChildF,vErrF)=" + F(HipAnchorAccelerationFeedforwardPeakAbsSourceToChildForward) + "/" + F(HipAnchorAccelerationFeedforwardPeakAbsSourceToChildLateral) + "/" + F(HipAnchorAccelerationFeedforwardPeakAbsSourceToChildVertical) + "," + F(HipAnchorAccelerationFeedforwardPeakAbsSourceToParentForward) + "/" + F(HipAnchorAccelerationFeedforwardPeakAbsParentToChildForward) + "/" + F(HipAnchorAccelerationFeedforwardPeakAbsForwardVelocityError));
            sb.AppendLine("legAngularDrag=PROFILE_CONTRACT | steps/invalid=" + LegAngularDragSteps + "/" + LegAngularDragInvalidSteps + "/upper/lower/expected=" + F(LegAngularDragUpper) + "/" + F(LegAngularDragLower) + "/" + F(LegAngularDragExpected) + "/peakErrorU/L=" + F(LegAngularDragPeakUpperError) + "/" + F(LegAngularDragPeakLowerError) + "/evidence=" + (LegAngularDragEvidence ?? "<none>"));
            sb.AppendLine("hipAngularActuator=" + (HipAngularActuatorMode ?? "<none>") + " | arm=" + (HipAngularActuatorArm ?? "<none>") + " | enabled=" + HipAngularActuatorEnabled + " | steps(active/total/invalid)=" + HipAngularActuatorActiveSteps + "/" + HipAngularActuatorSteps + "/" + HipAngularActuatorInvalidSteps);
            sb.AppendLine("hipAngularActuatorBody(mass/drag/I)=" + F(HipAngularActuatorBodyMass) + "/" + F(HipAngularActuatorBodyAngularDrag) + "/" + V(HipAngularActuatorBodyInertiaTensor));
            sb.AppendLine("hipAngularActuatorCurrent(A/w/tauI/gyro/tau/predA/predErr/align/gyroRatio)=" + F(HipAngularActuatorCurrentDesiredAcceleration) + "/" + F(HipAngularActuatorCurrentAngularSpeed) + "/" + F(HipAngularActuatorCurrentInertiaAlphaTorque) + "/" + F(HipAngularActuatorCurrentGyroTorque) + "/" + F(HipAngularActuatorCurrentTotalTorque) + "/" + F(HipAngularActuatorCurrentPredictedAcceleration) + "/" + F(HipAngularActuatorCurrentPredictionError) + "/" + F(HipAngularActuatorCurrentPredictionAlignment) + "/" + F(HipAngularActuatorCurrentGyroToInertiaRatio));
            sb.AppendLine("hipAngularActuatorPeak(A/w/tauI/gyro/tau/predErr/gyroRatio)=" + F(HipAngularActuatorPeakDesiredAcceleration) + "/" + F(HipAngularActuatorPeakAngularSpeed) + "/" + F(HipAngularActuatorPeakInertiaAlphaTorque) + "/" + F(HipAngularActuatorPeakGyroTorque) + "/" + F(HipAngularActuatorPeakTotalTorque) + "/" + F(HipAngularActuatorPeakPredictionError) + "/" + F(HipAngularActuatorPeakGyroToInertiaRatio));
            sb.AppendLine("hipAngularActuatorKey=" + (HipAngularActuatorKey ?? "<none>") + " | evidence=" + (HipAngularActuatorLastEvidence ?? "<none>"));
            sb.AppendLine("hipAnchorAccelFFLast=" + (HipAnchorAccelerationFeedforwardLastEvidence ?? "<none>"));
            sb.AppendLine("hipAnchorAccelFFAbort=" + (HipAnchorAccelerationFeedforwardAbortEvidence ?? "<none>"));
            sb.AppendLine("hipAnchorAccelFFKey=" + (HipAnchorAccelerationFeedforwardKey ?? "<none>"));
            sb.AppendLine("hipReferenceGovernorMode=" + (HipReferenceGovernorMode ?? "<none>"));
            sb.AppendLine("hipReferenceGovernorActive=" + HipReferenceGovernorActiveThisStep + "/contract=" + HipReferenceGovernorAccelerationContractSatisfiedThisStep + " | counts(active/bypass/activate/release/leadCap/accelLimit/brakeLimit/predict/source/recovery/frame)=" + HipReferenceGovernorActiveFixedSteps + "/" + HipReferenceGovernorNormalBypassFixedSteps + "/" + HipReferenceGovernorActivationCount + "/" + HipReferenceGovernorReleaseCount + "/" + HipReferenceGovernorLeadCappedFixedSteps + "/" + HipReferenceGovernorAccelerationLimitedFixedSteps + "/" + HipReferenceGovernorBrakingSpeedLimitedFixedSteps + "/" + HipReferenceGovernorPredictedRawFixedSteps + "/" + HipReferenceGovernorSourceActiveFixedSteps + "/" + HipReferenceGovernorPhysicalRecoveryFixedSteps + "/" + HipReferenceGovernorControlFrameActiveFixedSteps);
            sb.AppendLine("hipReferenceGovernorPeak(rawPacket/governedStep/rawGap/physicalLead/rawPhysicalErr/W/A/allowedA/Aratio/predAdvance/releasePose/releaseW)=" + F(HipReferenceGovernorPeakRawPacketDeltaDegrees) + "/" + F(HipReferenceGovernorPeakGovernedFixedStepDegrees) + "/" + F(HipReferenceGovernorPeakRawToGovernedGapDegrees) + "/" + F(HipReferenceGovernorPeakPhysicalLeadDegrees) + "/" + F(HipReferenceGovernorPeakRawPhysicalErrorDegrees) + "/" + F(HipReferenceGovernorPeakAngularSpeed) + "/" + F(HipReferenceGovernorPeakAngularAcceleration) + "/" + F(HipReferenceGovernorPeakAllowedAngularAcceleration) + "/" + F(HipReferenceGovernorPeakAccelerationRatio) + "/" + F(HipReferenceGovernorPeakPredictedRawAdvanceDegrees) + "/" + F(HipReferenceGovernorPeakReleasePoseJumpDegrees) + "/" + F(HipReferenceGovernorPeakReleaseAngularVelocityJump));
            sb.AppendLine("hipReferenceGovernorCurrent(rawGap/physicalLead/A/allowedA/releaseCandidate/predAdvance,constraints lead/accel/brake/releaseEligible)=" + F(HipReferenceGovernorCurrentRawGapDegrees) + "/" + F(HipReferenceGovernorCurrentPhysicalLeadDegrees) + "/" + F(HipReferenceGovernorCurrentAngularAcceleration) + "/" + F(HipReferenceGovernorCurrentAllowedAngularAcceleration) + "/" + F(HipReferenceGovernorReleaseCandidateSeconds) + "/" + F(HipReferenceGovernorCurrentPredictedRawAdvanceDegrees) + "," + HipReferenceGovernorLeadCappedThisStep + "/" + HipReferenceGovernorAccelerationLimitedThisStep + "/" + HipReferenceGovernorBrakingSpeedLimitedThisStep + "/" + HipReferenceGovernorReleaseEligibleThisStep + " | activation=" + (HipReferenceGovernorActivationReason ?? "<none>"));
            sb.AppendLine("hipReferenceGovernorLast=" + (HipReferenceGovernorLastEvidence ?? "<none>"));
            sb.AppendLine("hipReferenceGovernorKey=" + (HipReferenceGovernorKey ?? "<none>"));
            sb.AppendLine("rigidbodyInertiaFoundationMode=" + (RigidbodyInertiaFoundationMode ?? "<none>"));
            sb.AppendLine("rigidbodyInertiaAuditCompleted=" + RigidbodyInertiaAuditCompleted);
            sb.AppendLine("rigidbodyInertiaBuildFoundationValidated=" + RigidbodyInertiaBuildFoundationValidated);
            sb.AppendLine("rigidbodyInertiaAuditSummary=" + (RigidbodyInertiaAuditSummary ?? "<none>"));
            sb.AppendLine("rigidbodyInertiaAuditDetailBegin");
            sb.AppendLine(RigidbodyInertiaAuditDetail ?? "<none>");
            sb.AppendLine("rigidbodyInertiaAuditDetailEnd");
            sb.AppendLine("singleLegFreezeCertification=" + (SingleLegFreezeCertificationMode ?? "<none>") + " | policy=" + (SingleLegFreezeCertificationPolicy ?? "<none>") + " | complete=" + SingleLegFreezeCertificationComplete + " | production=" + SingleLegFreezeProductionContractPass + " | pca=" + SingleLegFreezeCalibrationPass + " | inertia=" + SingleLegFreezeInertiaPass + " | twoBody=" + SingleLegFreezeTwoBodyPass + " | runSafety=" + SingleLegFreezeRunSafetyPass + " | ready=" + SingleLegFreezeReady + " | reason=" + (SingleLegFreezeReason ?? "<none>"));
            sb.AppendLine("singleLegFreezeProductionContractKey=" + (SingleLegFreezeProductionContractKey ?? "<none>"));
            sb.AppendLine("singleLegFreezeKey=" + (SingleLegFreezeKey ?? "<none>"));
            sb.AppendLine("angularDynamicsMode=" + (AngularDynamicsMode ?? "<none>"));
            sb.AppendLine("angularDynamicsValid=" + AngularDynamicsValid);
            sb.AppendLine("hipTargetRelativeAngularSpeed=" + F(HipTargetRelativeAngularSpeed));
            sb.AppendLine("hipBodyRelativeAngularSpeed=" + F(HipBodyRelativeAngularSpeed));
            sb.AppendLine("hipAngularVelocityErrorMagnitude=" + F(HipAngularVelocityErrorMagnitude));
            sb.AppendLine("hipTargetRelativeAngularAccelerationMagnitude=" + F(HipTargetRelativeAngularAccelerationMagnitude));
            sb.AppendLine("hipPdPContributionMagnitude=" + F(HipPdPContributionMagnitude));
            sb.AppendLine("hipPdDContributionMagnitude=" + F(HipPdDContributionMagnitude));
            sb.AppendLine("kneeTargetAxisAngularVelocitySigned=" + F(KneeTargetAxisAngularVelocitySigned));
            sb.AppendLine("kneeBodyAxisAngularVelocitySigned=" + F(KneeBodyAxisAngularVelocitySigned));
            sb.AppendLine("kneeAxisAngularVelocityErrorSigned=" + F(KneeAxisAngularVelocityErrorSigned));
            sb.AppendLine("kneeTargetAxisAngularAccelerationSigned=" + F(KneeTargetAxisAngularAccelerationSigned));
            sb.AppendLine("kneePdPContributionSigned=" + F(KneePdPContributionSigned));
            sb.AppendLine("kneePdDContributionSigned=" + F(KneePdDContributionSigned));
            sb.AppendLine("targetRenderSampleSequence=" + TargetRenderSampleSequence);
            sb.AppendLine("fixedUpdateSequence=" + FixedUpdateSequence);
            sb.AppendLine("targetRenderSampleFreshAtFixedStep=" + TargetRenderSampleFreshAtFixedStep);
            sb.AppendLine("targetRenderSampleAgeSeconds=" + F(TargetRenderSampleAgeSeconds));
            sb.AppendLine("fixedStepsSinceTargetRenderSample=" + FixedStepsSinceTargetRenderSample);
            sb.AppendLine("hipAnchorErrorMeters=" + F(HipAnchorErrorMeters));
            sb.AppendLine("kneeAnchorErrorMeters=" + F(KneeAnchorErrorMeters));
            sb.AppendLine("peakHipAnchorErrorMeters=" + F(PeakHipAnchorErrorMeters));
            sb.AppendLine("peakKneeAnchorErrorMeters=" + F(PeakKneeAnchorErrorMeters));
            sb.AppendLine("hipClampCount=" + HipClampCount);
            sb.AppendLine("kneeClampCount=" + KneeClampCount);
            sb.AppendLine("traceSamples=" + TraceSamples);
            sb.AppendLine("traceHipRotationMeanDegrees=" + F(TraceHipRotationMeanDegrees));
            sb.AppendLine("traceHipRotationMaxDegrees=" + F(TraceHipRotationMaxDegrees));
            sb.AppendLine("traceKneeRotationMeanDegrees=" + F(TraceKneeRotationMeanDegrees));
            sb.AppendLine("traceKneeRotationMaxDegrees=" + F(TraceKneeRotationMaxDegrees));
            sb.AppendLine("traceHipClampSamples=" + TraceHipClampSamples);
            sb.AppendLine("traceKneeClampSamples=" + TraceKneeClampSamples);
            sb.AppendLine("traceKneeOffAxisErrorMeanDegrees=" + F(TraceKneeOffAxisErrorMeanDegrees));
            sb.AppendLine("traceKneeOffAxisErrorMaxDegrees=" + F(TraceKneeOffAxisErrorMaxDegrees));
            sb.AppendLine("traceKneeSoftOffAxisAppliedMean=" + F(TraceKneeSoftOffAxisAppliedMean));
            sb.AppendLine("traceKneeSoftOffAxisAppliedMax=" + F(TraceKneeSoftOffAxisAppliedMax));
            sb.AppendLine("traceKneeSoftOffAxisClampSamples=" + TraceKneeSoftOffAxisClampSamples);
            sb.AppendLine("attributionMode=" + (AttributionMode ?? "<none>"));
            sb.AppendLine("attributionPhase=" + (AttributionPhase ?? "<none>"));
            sb.AppendLine("stanceLowerNativeGravitySupportLoad=" + (StanceLowerNativeGravitySummary ?? "<none>"));
            sb.AppendLine("kneeSupportNormalReaction=" + (KneeSupportReaction?.GetSummary() ?? "<none>"));
            sb.AppendLine("upperSelfWeightSupport=" + (UpperSelfWeightSupport?.GetDetails() ?? "<none>"));
            sb.AppendLine("stanceFootLowerSupportNormalReciprocalLoadTransfer=" + (StanceFootLowerReciprocalSummary ?? "<none>"));
            sb.AppendLine("stanceLowerSelfWeightSupportNormalLoadTransfer=" + (StanceLowerSelfWeightTransferSummary ?? "<none>"));
        sb.AppendLine("driveAborted=" + DriveAborted);
            sb.AppendLine("driveAbortReason=" + (DriveAbortReason ?? "<none>"));
            return sb.ToString();
        }


        private static string F(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + ", " + F(value.y) + ", " + F(value.z) + ")";
    }
}
