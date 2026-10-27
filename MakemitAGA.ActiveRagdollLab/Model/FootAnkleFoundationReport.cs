using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    /// <summary>
    /// ARL v0.4.0 Stage-E contact identity qualification shell over the unchanged v0.3.78 Foot/Ankle free-space production report.
    /// The v0.3.76 bounded common-mode angular-drag compensation passed v0.3.77 stronger-motion confirmation and is now frozen production.
    /// The v0.3.69 free-space Dynamic Foot production physics remains the baseline. v0.3.70 signed-radial precursor
    /// attribution proved parent transport and the final command were correctly radial-closing during the tail, while
    /// MaxA60 saturated against abrupt COM-velocity debt. v0.3.71 qualified a narrow conditional-80 candidate; v0.3.72
    /// confirmed the same policy again while translation stayed below ~15 mm before an independent Left axial hard-envelope abort. v0.3.73 promotes conditional80 into production and adds only read-only axial temporal attribution. The 30 mm translation gate and 45/40/30/total60 angular hard gate remain unchanged.
    /// </summary>
    internal sealed class FootAnkleFoundationReport
    {
        public const string ModeName = "ANKLE_FREE_SPACE_PRODUCTION_V0378";

        public bool Built;
        public bool Finite = true;
        public string Side;
        public string AnklePath;
        public string ToePath;
        public string ToeResolution;
        public string Stage = "UNBUILT";
        public string Detail;

        public float FootLength;
        public float Mass;
        public string GeometryMode;
        public bool GeometryMeshCalibrated;
        public bool ContactGeometryAuthorized;
        public float ProxyForwardLength;
        public float ProxyHeelLength;
        public float ProxyTotalLength;
        public float ProxyWidth;
        public float ProxyThickness;
        public float ProxyAnkleHeight;
        public bool SupportPlaneCalibrationAttempted;
        public bool SupportPlaneCalibrationApplied;
        public string SupportPlaneCalibrationReason;
        public float SupportPlaneCalibrationInitialAnkleHeightMeters;
        public float SupportPlaneCalibrationCorrectionMeters;
        public float SupportPlaneCalibrationFinalAnkleHeightMeters;
        public int SupportPlaneCalibrationHitCount;
        public float SupportPlaneCalibrationSpreadMeters;
        public float SupportPlaneCalibrationNormalDot;
        public string SupportPlaneCalibrationSurfacePath;
        public bool FootwearStateResolved;
        public string FootwearGeometryPolicy;
        public float ProjectedAnkleToeLength;
        public float BoneToSoleCorrectionDegrees;
        public Quaternion BoneToSoleBindRotation = Quaternion.identity;
        public float SoleToeForwardDot;
        public float SoleUpReferenceUpDot;
        public Vector3 SourceBindSoleDeltaDegrees;
        public Vector3 ColliderCenterLocal;
        public Vector3 ColliderSizeLocal;
        public Vector3 RuntimeCenterOfMassLocal;
        public Vector3 RuntimeInertiaTensor;
        public bool MassPropertiesValidated;

        public bool Kinematic;
        public bool GravityEnabled;
        public bool ColliderEnabled;
        public bool ColliderFrozenOff;
        public bool VisualEnabled;
        // V04390 frozen Foot Speculative CCD / idle Discrete owner and actual-mode readback.
        public string FootCollisionDetectionModeExperiment;
        public string FootCollisionDetectionModeArm;
        public bool FootCollisionDetectionModeFinite = true;
        public string FootCollisionDetectionModeInvalidReason;
        public string FootCollisionDetectionModeReadback;
        public string FootCollisionDetectionModeSummary;

        public string FootContactOffsetMode, FootContactOffsetArm, FootContactOffsetInvalidReason, FootContactOffsetSummary;
        public bool FootContactOffsetInitialized, FootContactOffsetFinite = true, FootContactOffsetReadbackValid, FootContactOffsetBaselineRestored;
        public int FootContactOffsetOwnerId, FootContactOffsetReadbackFailures;
        public float FootContactOffsetBaselineMeters, FootContactOffsetSelectedMeters, FootContactOffsetActualMeters;

        public bool AnkleJointExists;
        public bool AnkleConnectedBodyMatch;
        public Vector3 AnkleJointAnchorLocal;
        public Vector3 AnkleConnectedAnchorLocal;
        public Vector3 AnkleAxisLocal;
        public Vector3 AnkleSwingAxisLocal;
        public float AnkleLowTwistLimit;
        public float AnkleHighTwistLimit;
        public float AnkleSwing1Limit;
        public float AnkleSwing2Limit;

        public int TargetRenderSampleSequence;
        // Materialized only when tracing ends; no per-Fixed formatting or readiness dependency.
        public string TurnEventDetails;
        public PhysicsStepContext RuntimeContext;
        public SourceStepContext RuntimeSourceContext;
        public int FixedUpdateSequence;
        public bool TargetRenderSampleFreshAtFixedStep;
        public int FixedStepsSinceTargetRenderSample;
        public float TargetRenderSampleAgeSeconds;

        public Quaternion BindAnkleRelativeRotation = Quaternion.identity;
        public Quaternion SourceAnkleRelativeRotation = Quaternion.identity;
        // ROM from bind is not a per-packet jump. Render step is measured only between real source samples.
        public bool SourceAnkleRenderStepValid;
        public float SourceAnkleRenderStepSeconds;
        public Vector3 SourceAnkleRenderStepDegrees;
        public int TraceSourceRenderStepSamples;
        public float TraceSourceRenderStepAngleMaxDegrees;
        public FootFixedStepEvidence PreviousObservedStep;
        public FootFixedStepEvidence CurrentObservedStep;
        public int AnkleConstraintLastEvaluatedFixedSequence;
        public int AnkleDriveLastEvaluatedFixedSequence;
        public int FootControllerLastCompletedFixedSequence;
        public int TraceObservedFixedSamples;
        public float TraceObservedAnchorErrorMaxMeters;
        public float TraceObservedTrackingErrorMaxDegrees;
        public Vector3 SourceAnkleFromBindRotationVectorDegrees;
        public float SourceAnkleFromBindAngleDegrees;
        public Vector3 SourceAnkleRelativeAngularVelocityLocal;
        public Vector3 SourceAnkleRelativeAngularAccelerationLocal;
        public float SourceAnkleRelativeAngularSpeed;
        public float AnkleRelativeTrackingErrorDegrees;
        public float AnkleAnchorErrorMeters;
        public Vector3 SampledFootLinearVelocityWorld;

        public int TraceSamples;
        public float TraceAnkleRelativeErrorMeanDegrees;
        public float TraceAnkleRelativeErrorMaxDegrees;
        public float TraceAnkleAnchorErrorMeanMeters;
        public float TraceAnkleAnchorErrorMaxMeters;
        public float TraceSourceFromBindAngleMeanDegrees;
        public float TraceSourceFromBindAngleMaxDegrees;
        public float TraceSourceAngularSpeedMean;
        public float TraceSourceAngularSpeedMax;
        public Vector3 TraceSourceFromBindAxisMinDegrees;
        public Vector3 TraceSourceFromBindAxisMaxDegrees;
        public Vector3 TraceSourceFromBindAxisAbsMaxDegrees;
        public Vector3 TraceSourceAngularVelocityAxisMin;
        public Vector3 TraceSourceAngularVelocityAxisMax;
        public Vector3 TraceSourceAngularVelocityAxisAbsMax;

        // v0.3.56.1 support-plane calibration / sole-frame / ankle-ROM / ground-readiness requalification.
        public string SoleRequalificationMode;
        public bool SoleRequalificationFinite = true;
        public string SoleRequalificationInvalidReason;
        public string SoleRequalificationCurrentSummary;
        public string SoleRequalificationTraceSummary;
        public bool GroundGeometryReady;
        public bool PhysicalGroundContactPossible;
        public int GroundProbeHitCount;
        public float GroundCenterClearanceMeters;
        public float GroundClearanceMinMeters;
        public float GroundClearanceMaxMeters;
        public float GroundClearanceSpreadMeters;
        public float GroundNormalDotReferenceUp;
        public float GroundFootprintMinNormalDotReferenceUp;
        public Vector3 GroundAverageNormalWorld;
        public string GroundColliderPath;
        public bool GroundFootprintSameSurface;
        public int GroundColliderInstanceId;
        public bool GroundColliderHasAttachedRigidbody;

        // v0.4.0 Stage-E read-only exact support/contact identity qualification.
        public string ContactIdentityMode;
        public bool ContactIdentityFinite = true;
        public string ContactIdentityInvalidReason;
        public bool ContactIdentityCurrentQualified;
        public bool ContactIdentityCurrentStaticCollider;
        public int ContactIdentityCurrentColliderInstanceId;
        public string ContactIdentityCurrentColliderPath;
        public float ContactIdentityCurrentStableSeconds;
        public int ContactIdentityTraceQuietEligibleSamples;
        public int ContactIdentityTraceQuietQualifiedSamples;
        public float ContactIdentityTraceQuietQualifiedRatio;
        public int ContactIdentityTraceNoHitSamples;
        public int ContactIdentityTraceDynamicRejectedSamples;
        public int ContactIdentityTraceSwitches;
        public int ContactIdentityTraceStableSegments;
        public float ContactIdentityTraceLongestStableSeconds;
        public int ContactIdentityLastStableColliderInstanceId;
        public string ContactIdentityLastStableColliderPath;
        public bool ContactIdentityTraceEndedStable;
        public int ContactIdentityEndingStableColliderInstanceId;
        public string ContactIdentityEndingStableColliderPath;
        public bool ContactIdentityTraceQualified;
        public string ContactIdentityCurrentSummary;
        public string ContactIdentityTraceSummary;

        // v0.4.1.1 qualified-static-support Foot Collider/contact callback shadow.
        public string ContactShadowMode;
        public string ContactShadowArm;
        public bool ContactShadowFinite = true;
        public string ContactShadowInvalidReason;
        public bool ContactShadowPrepared;
        public bool ContactShadowActive;
        public int ContactShadowAuthorizedColliderInstanceId;
        public string ContactShadowAuthorizedColliderPath;
        public int ContactShadowSceneColliderCount;
        public int ContactShadowOwnedIsolationPairs;
        public int ContactShadowAlreadyIgnoredPairs;
        public int ContactShadowIsolationFailures;
        public int ContactShadowRestoreFailures;
        // v0.4.4.2 persistent source/self-collider isolation lifecycle diagnostics. These remain separate
        // from authorized-support callback counts so a quarantined self callback can never masquerade as GRF evidence.
        public int ContactShadowSourceRegistryCount;
        public int ContactShadowSourceAuxiliaryRegistryCount;
        public int ContactShadowSourceLateDiscoveries;
        public int ContactShadowSourceIsolationRepairs;
        public int ContactShadowSourceIsolationReadbackFailures;
        public int ContactShadowSourceCallbackEnter;
        public int ContactShadowSourceCallbackStay;
        public int ContactShadowSourceCallbackExit;
        public int ContactShadowSourceCallbacksQuarantined;
        public int ContactShadowLastSourceColliderInstanceId;
        public string ContactShadowLastSourceColliderPath;
        public int ContactShadowTraceFixedSamples;
        public int ContactShadowTraceRaycastMatchSamples;
        public int ContactShadowTraceRaycastNoHitSamples;
        public int ContactShadowTraceRaycastMismatchSamples;
        public int ContactShadowTraceEnter;
        public int ContactShadowTraceStay;
        public int ContactShadowTraceExit;
        public int ContactShadowTraceMatchedCallbacks;
        public int ContactShadowTraceUnexpectedCallbacks;
        public bool ContactShadowCurrentContactActive;
        public string ContactShadowLastCallbackPhase;
        public int ContactShadowLastCallbackColliderInstanceId;
        public string ContactShadowLastCallbackColliderPath;
        public bool ContactShadowTraceQualified;
        public string ContactShadowCurrentSummary;
        public string ContactShadowTraceSummary;

        // v0.4.7 per-foot stance-only authorized contact lifecycle. This layer reuses the frozen v0.4.5 gait
        // classifier state and the closed v0.4.1-v0.4.4 exact-support contact primitive. It does not add any
        // force/torque/raycast or relax existing hard gates.
        public string StanceContactLifecycleMode;
        public bool StanceContactLifecycleEnabled;
        public bool StanceContactLifecycleFinite = true;
        public string StanceContactLifecycleInvalidReason;
        public string StanceContactLifecycleState;
        public string StanceContactLifecycleLastReason;
        public float StanceContactLifecycleQualifiedStanceSeconds;
        public int StanceContactLifecycleQualifiedColliderInstanceId;
        public string StanceContactLifecycleQualifiedColliderPath;
        public int StanceContactLifecycleAuthorizedColliderInstanceId;
        public string StanceContactLifecycleAuthorizedColliderPath;
        public int StanceContactLifecycleFixedSamples;
        public int StanceContactLifecycleActiveSamples;
        public int StanceContactLifecycleContactActivations;
        public int StanceContactLifecyclePrepareCount;
        public int StanceContactLifecyclePreparedReuseCount;
        public int StanceContactLifecycleSurfaceReprepareCount;
        public int StanceContactLifecycleReleaseCount;
        public int StanceContactLifecycleTransitionReleaseCount;
        public int StanceContactLifecycleSwingReleaseCount;
        public int StanceContactLifecycleIdentityLossReleaseCount;
        public int StanceContactLifecycleSurfaceChangeReleaseCount;
        public int StanceContactLifecycleIdentityRejectSamples;
        // v0.4.16.1 activation-readiness hotfix diagnostics.  These count only OFF->ON gating;
        // they do not relax the downstream support-normal reaction contract.
        public int StanceContactLifecycleActivationGeometryRejectSamples;
        public int StanceContactLifecycleActivationGeometryReadySamples;
        public int StanceContactLifecycleNonStanceActiveSamples;
        public int StanceContactLifecycleEnableFailures;
        public int StanceContactLifecycleDisableFailures;
        public int StanceContactLifecycleFirstContactFixed;
        public int StanceContactLifecycleLastContactFixed;
        public int StanceContactLifecycleLastReleaseFixed;
        public bool StanceContactLifecycleTraceQualified;
        public string StanceContactLifecycleCurrentSummary;
        public string StanceContactLifecycleTraceSummary;

        // v0.4.8 causal isolation of stance-contact friction/yaw constraint. Both arms keep the same
        // v0.4.7 stance lifecycle; only the Foot Collider PhysicMaterial changes.
        public string StanceContactFrictionIsolationMode;
        public string StanceContactFrictionIsolationArm;
        public bool StanceContactFrictionIsolationFinite = true;
        public string StanceContactFrictionIsolationInvalidReason;
        public bool StanceContactFrictionIsolationZeroFriction;
        public bool StanceContactFrictionIsolationMaterialAssigned;
        public float StanceContactFrictionIsolationStaticFriction;
        public float StanceContactFrictionIsolationDynamicFriction;
        public string StanceContactFrictionIsolationFrictionCombine;
        public bool StanceContactFrictionIsolationGroundMaterialAssigned;
        public float StanceContactFrictionIsolationGroundStaticFriction;
        public float StanceContactFrictionIsolationGroundDynamicFriction;
        public string StanceContactFrictionIsolationGroundFrictionCombine;
        public bool StanceContactFrictionIsolationEffectiveZeroGuaranteed;
        public string StanceContactFrictionIsolationSummary;

        // v0.4.16 consolidation: the v0.4.10 Foot world-ground anti-slip actuator and its A/B report
        // fields were retired after v0.4.15 proved Lower-ground ownership + the frozen Foot↔Lower
        // tether preserves local closure without that duplicate world-ground write.
        // v0.4.11 observer-only separation between source-relative planar tracking and true ground-relative
        // stance drift. The v0.4.10 B physical plant is frozen; these fields are diagnostic-only.

        // v0.4.16 consolidation: the large v0.4.12 anti-slip-vs-tether competition observer/report
        // was retired after v0.4.15 closed the ownership question. Tether load remains available
        // from the frozen v0.3.63 report; ground-relative common/differential motion remains in V04130.
        // v0.4.13 observer-only stance ground-anchor decomposition. Foot/Lower ground-relative motion is split
        // exactly into equal-weight common and differential contributions, and two Lower-ground counterfactual
        // acceleration demands are recorded without applying them.

        public string StanceLowerGroundAuthorityMode;
        public string StanceLowerGroundAuthorityArm;
        public bool StanceLowerGroundAuthorityFinite = true;
        public string StanceLowerGroundAuthorityInvalidReason;
        public bool StanceLowerGroundAuthorityEnabled;
        public float StanceLowerGroundAuthorityKp;
        public float StanceLowerGroundAuthorityKd;
        public float StanceLowerGroundAuthorityMaxAcceleration;
        public int StanceLowerGroundAuthorityTraceSamples;
        public int StanceLowerGroundAuthorityEligibleSamples;
        public int StanceLowerGroundAuthoritySupportNormalGapSamples;
        public int StanceLowerGroundAuthorityEpisodesStarted;
        public int StanceLowerGroundAuthorityAppliedSamples;
        public int StanceLowerGroundAuthorityClampSamples;
        public int StanceLowerGroundAuthorityNonStanceAppliedSamples;
        public float StanceLowerGroundAuthorityPositionErrorMeanMeters;
        public float StanceLowerGroundAuthorityPositionErrorMaxMeters;
        public float StanceLowerGroundAuthorityVelocityErrorMean;
        public float StanceLowerGroundAuthorityVelocityErrorMax;
        public float StanceLowerGroundAuthorityRawAccelerationMean;
        public float StanceLowerGroundAuthorityRawAccelerationMax;
        public float StanceLowerGroundAuthorityAppliedAccelerationMean;
        public float StanceLowerGroundAuthorityAppliedAccelerationMax;
        public Vector3 StanceLowerGroundAuthorityPositionErrorPlanarWorld;
        public Vector3 StanceLowerGroundAuthorityVelocityErrorPlanarWorld;
        public Vector3 StanceLowerGroundAuthorityRawAccelerationWorld;
        public Vector3 StanceLowerGroundAuthorityAppliedAccelerationWorld;
        public bool StanceLowerGroundAuthorityCurrentClamped;
        public bool StanceLowerGroundAuthorityCurrentApplied;
        public float StanceLowerGroundAuthorityDirectYawTorqueMax;
        public int StanceLowerGroundAuthorityFirstAppliedFixed;
        public int StanceLowerGroundAuthorityLastAppliedFixed;
        public string StanceLowerGroundAuthoritySummary;

        // v0.4.17 narrow native-gravity support-load causal A/B. B enables only the Dynamic Foot
        // Rigidbody's native gravity for exact authorized STANCE solves; torso/Upper/Lower/GSP remain OFF.
        public string StanceFootNativeGravityMode;
        public string StanceFootNativeGravityArm;
        public bool StanceFootNativeGravityFinite = true;
        public string StanceFootNativeGravityInvalidReason;
        public bool StanceFootNativeGravityEnabled;
        public int StanceFootNativeGravityTraceSamples;
        public int StanceFootNativeGravityEligibleSamples;
        public int StanceFootNativeGravityAppliedSamples;
        public int StanceFootNativeGravityNonStanceAppliedSamples;
        public int StanceFootNativeGravityReadbackFailures;
        public int StanceFootNativeGravityCallbackSamples;
        public int StanceFootNativeGravityCallbackGravityOnSamples;
        public int StanceFootNativeGravityCallbackGravityOffSamples;
        public float StanceFootNativeGravityExpectedNormalForceMean;
        public float StanceFootNativeGravityExpectedNormalForceMax;
        public float StanceFootNativeGravityExpectedNormalImpulseMean;
        public float StanceFootNativeGravityExpectedNormalImpulseMax;
        public float StanceFootNativeGravitySupportNormalSpeedAbsMean;
        public float StanceFootNativeGravitySupportNormalSpeedAbsMax;
        public float StanceFootNativeGravityTetherSupportNormalAccelerationMean;
        public float StanceFootNativeGravityTetherSupportNormalAccelerationMin;
        public float StanceFootNativeGravityTetherSupportNormalAccelerationMax;
        public float StanceFootNativeGravityGravityOnSignedNormalImpulseMean;
        public float StanceFootNativeGravityGravityOnSignedNormalImpulseMax;
        public float StanceFootNativeGravityGravityOffSignedNormalImpulseMean;
        public float StanceFootNativeGravityGravityOffSignedNormalImpulseMax;
        public Vector3 StanceFootNativeGravityGravityWorld;
        public float StanceFootNativeGravityGravityMagnitude;
        public float StanceFootNativeGravityFootMass;
        public bool StanceFootNativeGravityCurrentApplied;
        public int StanceFootNativeGravityCurrentScheduledFixed;
        public int StanceFootNativeGravityFirstAppliedFixed;
        public int StanceFootNativeGravityLastAppliedFixed;
        public string StanceFootNativeGravitySummary;

        // v0.4.3 read-only static-support-normal mechanical reaction attribution.
        public string MechanicalReactionMode;
        public bool MechanicalReactionFinite = true;
        public string MechanicalReactionInvalidReason;
        public bool MechanicalReactionReadbackClean = true;
        public string MechanicalReactionLastReadFailureStage;
        public string MechanicalReactionLastReadFailureType;
        public string MechanicalReactionLastReadFailureMessage;
        public bool MechanicalReactionSolverWitnessQualified;
        public bool MechanicalReactionSupportNormalWitnessQualified;
        public int MechanicalReactionTraceCallbacks;
        public int MechanicalReactionTraceSupportNormalSamples;
        public int MechanicalReactionTraceSupportNormalInvalidSamples;
        public float MechanicalReactionTraceSupportNormalValidRatio;
        public int MechanicalReactionTraceSupportColliderMismatchSamples;

        // v0.4.3.1 observer-only decomposition of transient support-normal gaps. These fields do not
        // alter the v0.4.3 qualification contract or any physical authority.
        public int MechanicalReactionTraceSupportInvalidAuthorizedIdUnavailable;
        public int MechanicalReactionTraceSupportInvalidSupportIdUnavailable;
        public int MechanicalReactionTraceSupportInvalidIdMismatch;
        public int MechanicalReactionTraceSupportInvalidGeometryNotReady;
        public int MechanicalReactionTraceSupportInvalidFootprintNotSameSurface;
        public int MechanicalReactionTraceSupportInvalidNormalNonFiniteOrZero;
        public int MechanicalReactionTraceSupportPredicateGeometryNotReady;
        public int MechanicalReactionTraceSupportPredicateFootprintNotSameSurface;
        public int MechanicalReactionTraceSupportPredicateNormalNonFiniteOrZero;
        public int MechanicalReactionTraceSupportPredicateHitCountLt5;
        public int MechanicalReactionTraceSupportPredicateHitCountLt3;
        public int MechanicalReactionTraceSupportPredicatePrimarySurfaceNotSame;
        public int MechanicalReactionTraceSupportPredicateClearanceGateFail;
        public int MechanicalReactionTraceSupportPredicateSpreadGateFail;
        public int MechanicalReactionTraceSupportPredicateNormalDotGateFail;
        public int MechanicalReactionTraceSupportInvalidEnter;
        public int MechanicalReactionTraceSupportInvalidStay;
        public int MechanicalReactionTraceSupportInvalidOtherPhase;
        public int MechanicalReactionTraceSupportInvalidEpisodes;
        public int MechanicalReactionTraceSupportInvalidMaxStreak;
        public int MechanicalReactionTraceSupportInvalidFirstCallbackIndex;
        public int MechanicalReactionTraceSupportInvalidLastCallbackIndex;
        public int MechanicalReactionTraceSupportInvalidFirstPreFixedSequence;
        public int MechanicalReactionTraceSupportInvalidLastPreFixedSequence;

        // v0.4.3.2 observer-only lateral-probe alternate-collider attribution.
        // These counters/summaries explain WHICH already-executed sole probe diverged from the
        // authorized center surface during an invalid episode. They do not alter any readiness gate.
        public int MechanicalReactionTraceInvalidLateralLeftOnlySamples;
        public int MechanicalReactionTraceInvalidLateralRightOnlySamples;
        public int MechanicalReactionTraceInvalidLateralBothSamples;
        public int MechanicalReactionTraceInvalidNoLateralDivergenceSamples;
        public int MechanicalReactionTraceInvalidPrimaryProbeDivergenceSamples;
        public int MechanicalReactionTraceAlternateColliderUniqueCount;
        public int MechanicalReactionTraceAlternateColliderOverflowCount;
        public string MechanicalReactionTraceAlternateColliderCatalog;
        public string MechanicalReactionTraceInvalidEpisodeAttribution;

        public int MechanicalReactionTraceProjectionSamples;
        public int MechanicalReactionTraceReadFailures;
        public int MechanicalReactionTracePrePhysicsSamples;
        public int MechanicalReactionTraceResponseSamples;
        public int MechanicalReactionTraceImpulseReadSamples;
        public int MechanicalReactionTraceImpulseReadFailures;
        public int MechanicalReactionTraceRelativeVelocityReadSamples;
        public int MechanicalReactionTraceRelativeVelocityReadFailures;
        public int MechanicalReactionTracePostBodyReadSamples;
        public int MechanicalReactionTracePostBodyReadFailures;
        public int MechanicalReactionTraceNonZeroImpulseSamples;
        public int MechanicalReactionTraceAlignedImpulseSamples;
        public int MechanicalReactionTraceOpposedImpulseSamples;
        public int MechanicalReactionTraceNearZeroImpulseSamples;
        public float MechanicalReactionTraceImpulseMagnitudeMean;
        public float MechanicalReactionTraceImpulseMagnitudeMax;
        public float MechanicalReactionTraceSignedNormalImpulseMean;
        public float MechanicalReactionTraceSignedNormalImpulseMin;
        public float MechanicalReactionTraceSignedNormalImpulseMax;
        public float MechanicalReactionTraceAbsNormalImpulseMean;
        public float MechanicalReactionTraceAbsNormalImpulseMax;
        public float MechanicalReactionTraceTangentImpulseMagnitudeMean;
        public float MechanicalReactionTraceTangentImpulseMagnitudeMax;
        public float MechanicalReactionTraceNormalShareAbsMean;
        public float MechanicalReactionTraceNormalShareAbsMin;
        public float MechanicalReactionTraceTangentShareMean;
        public float MechanicalReactionTraceTangentShareMax;
        public float MechanicalReactionTraceAlignedImpulseRatio;
        public float MechanicalReactionTraceOpposedImpulseRatio;
        public float MechanicalReactionTraceRelativeSpeedMean;
        public float MechanicalReactionTraceRelativeSpeedMax;
        public float MechanicalReactionTraceSignedRelativeNormalSpeedMean;
        public float MechanicalReactionTraceRelativeNormalSpeedAbsMean;
        public float MechanicalReactionTraceRelativeNormalSpeedAbsMax;
        public float MechanicalReactionTraceObservedDeltaVMean;
        public float MechanicalReactionTraceObservedDeltaVMax;
        public float MechanicalReactionTraceImpulseDeltaVMean;
        public float MechanicalReactionTraceImpulseDeltaVMax;
        public float MechanicalReactionTraceDeltaVCosAbsMean;
        public float MechanicalReactionTraceDeltaVCosAbsMin;
        public float MechanicalReactionTraceObservedDeltaWMean;
        public float MechanicalReactionTraceObservedDeltaWMax;
        public float MechanicalReactionTracePositionResidualMean;
        public float MechanicalReactionTracePositionResidualMax;
        public bool MechanicalReactionTraceQualified;
        public string MechanicalReactionCurrentSummary;
        public string MechanicalReactionTraceSummary;

        // v0.4.23 read-only Fixed N -> PhysX callback -> Fixed N+1 solve-continuity attribution.
        // Diagnostic-only: never participates in Foot fail-close or controller inputs.

        // V04280 diagnostic-only fields. Never part of production finite/safety checks.
        public string StancePlanarDampingMode;
        public string StancePlanarDampingArm;
        public bool StancePlanarDampingEnabled;
        public bool StancePlanarDampingQuiesced;
        public bool StancePlanarDampingFinite = true;
        public string StancePlanarDampingInvalidReason;
        public bool StancePlanarDampingCurrentActive;
        public int StancePlanarDampingLoadedSolves;
        public int StancePlanarDampingNonzeroWrites;
        public string StancePlanarDampingSummary;

        public string SupportEntityBoundaryMode;
        public bool SupportEntityBoundaryFinite = true;
        public string SupportEntityBoundaryInvalidReason;
        public bool SupportEntityBoundaryEvidenceReady;
        public int SupportEntityBoundaryResolvedLoadedSolves;
        public string SupportEntityBoundaryTraceSummary;
        public string SupportEntityBoundaryRecentFrames;
        // V04350 retains raw ledgers and compares damping/actual release steps in the same bounded windows.
        public string SupportEntityBoundaryMotionMode;
        public bool SupportEntityBoundaryMotionFinite = true;
        public string SupportEntityBoundaryMotionInvalidReason;
        public string SupportEntityBoundaryMotionTraceSummary;
        public string SupportNormalWindowMode;
        public bool SupportNormalWindowFinite = true;
        public string SupportNormalWindowInvalidReason;
        public int SupportNormalWindowJoinedSolves;
        public bool SupportNormalWindowEvidenceReady;
        public string SupportNormalWindowTraceSummary;
        // V04360 reuses the same callback snapshots and cached angular/linear ledgers.
        // These fields never qualify the plant, contact authority, controller or safety gates.
        public string SupportCallbackMomentumMode;
        public bool SupportCallbackMomentumFinite = true;
        public string SupportCallbackMomentumInvalidReason;
        public string SupportCallbackMomentumTraceSummary;

        // V04330 independent diagnostic domain; no physical qualification/gate consumes these fields.
        public string DynamicNormalLoadMode;
        public bool DynamicNormalLoadFinite = true;
        public string DynamicNormalLoadInvalidReason;
        public int DynamicNormalLoadResolvedSolves;
        public bool DynamicNormalLoadEvidenceReady;
        public string DynamicNormalLoadTraceSummary;
        public string DynamicNormalLoadPeakFrames;

        public string RockingTetherCouplingMode;
        public bool RockingTetherCouplingFinite = true;
        public string RockingTetherCouplingInvalidReason;
        public bool RockingTetherCouplingEvidenceReady;
        public int RockingTetherCouplingResolvedSolves;
        public string RockingTetherCouplingTraceSummary;
        public string RockingTetherCouplingRecentFrames;

        // V04270 selector readbacks and run provenance; current values may be restored before audit.
        public string FootVelocitySolverMode;
        public string FootVelocitySolverArm;
        public bool FootVelocitySolverInitialized;
        public bool FootVelocitySolverFinite;
        public string FootVelocitySolverInvalidReason;
        public int FootVelocitySolverBaselineVelocity;
        public int FootVelocitySolverBaselinePosition;
        public int FootVelocitySolverSelectedVelocity;
        public int FootVelocitySolverActualVelocity;
        public int FootVelocitySolverActualPosition;
        public bool FootVelocitySolverReadbackValid;
        public bool FootVelocitySolverQuiesced;
        public bool FootVelocitySolverBaselineRestored;
        public int FootVelocitySolverEligibleSolves;
        public int FootVelocitySolverReadbackFailures;
        public string FootVelocitySolverSummary;

        // V04310 position-only experiment; captured velocity and stop originals share one owner.
        public string FootPositionSolverMode;
        public string FootPositionSolverArm;
        public int FootPositionSolverSelectedPosition;
        public bool FootPositionSolverEnabled;
        public int FootPositionSolverInterventionSolves;
        public string FootPositionSolverSummary;
        public int SupportEntityBoundaryPositionSolverResolvedSolves;
        public int SupportEntityBoundaryCcdResolvedSolves;
        public int SupportEntityBoundaryCcdMismatchSolves;
        public int SupportEntityBoundaryContactOffsetResolvedSolves, SupportEntityBoundaryContactOffsetMismatchSolves;
        public int SupportEntityBoundaryContactOffsetBoundaryChangedSolves, SupportEntityBoundaryContactOffsetReadUnavailableSolves;
        public int SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves, SupportEntityBoundaryContactOffsetSupportLostSolves;
        public int SupportEntityBoundaryContactOffsetGroundBaselines, SupportEntityBoundaryContactOffsetGroundSwitches;

        // V04260 isolated physical damping channel; run evidence survives quiesce until selector reset.
        public string StanceSupportAngularArm;
        public bool StanceSupportAngularEnabled;
        public bool StanceSupportAngularFinite = true;
        public string StanceSupportAngularInvalidReason;
        public int StanceSupportAngularEligibleSamples;
        public int StanceSupportAngularAppliedSamples;
        public int StanceSupportAngularClampedSamples;
        public Vector3 StanceSupportAngularCurrentTorqueWorld;
        public float StanceSupportAngularMaxNormalTorqueNm;
        public float StanceSupportAngularMaxPowerWatts;
        public string StanceSupportAngularSummary;

        // v0.4.25 observer retained; v0.4.26 also subtracts support damping torque impulse. ContactPoint APIs remain unavailable on this IL2CPP runtime.
        public string FootAngularImpulseAttributionMode;
        public bool FootAngularImpulseAttributionFinite = true;
        public string FootAngularImpulseAttributionInvalidReason;
        public bool FootAngularImpulseAttributionEvidenceReady;
        public string FootAngularImpulseAttributionCurrentSummary;
        public string FootAngularImpulseAttributionTraceSummary;

        // Historical cleanup (v0.4.15): v0.4.4 mild-motion recovery and v0.4.4.1 axial-debt observers
        // were removed; v0.4.7 lifecycle + v0.4.8 zero-friction supersede their online diagnostic role.
        // v0.4.5 read-only game-native locomotion stance/swing phase qualification. This observer is
        // intentionally contact-free (formal test uses arm A / Foot Collider OFF) and reuses the existing
        // five-point sole probe plus render-sampled source Foot velocity. It does not control contact.
        public string GameNativeGaitPhaseMode;
        public bool GameNativeGaitPhaseFinite = true;
        public string GameNativeGaitPhaseInvalidReason;
        public string GameNativeGaitPhaseState;
        public int GameNativeGaitTraceSamples;
        public int GameNativeGaitEligibleSamples;
        public int GameNativeGaitStanceSamples;
        public int GameNativeGaitSwingSamples;
        public int GameNativeGaitTransitionSamples;
        public int GameNativeGaitUnknownSamples;
        public int GameNativeGaitStanceEntries;
        public int GameNativeGaitSwingEntries;
        public int GameNativeGaitStanceToSwingTransitions;
        public int GameNativeGaitSwingToStanceTransitions;
        public int GameNativeGaitAmbiguousSamples;
        public int GameNativeGaitNoHitSamples;
        public int GameNativeGaitFullFootprintSamples;
        public int GameNativeGaitSameSurfaceSamples;
        public int GameNativeGaitFirstStanceFixed;
        public int GameNativeGaitFirstSwingFixed;
        public int GameNativeGaitLastTransitionFixed;
        public float GameNativeGaitStanceTangentialSpeedMean;
        public float GameNativeGaitStanceTangentialSpeedMax;
        public float GameNativeGaitSwingTangentialSpeedMean;
        public float GameNativeGaitSwingTangentialSpeedMax;
        public float GameNativeGaitStanceNormalSpeedAbsMean;
        public float GameNativeGaitStanceNormalSpeedAbsMax;
        public float GameNativeGaitSwingNormalSpeedAbsMean;
        public float GameNativeGaitSwingNormalSpeedAbsMax;
        public float GameNativeGaitStanceClearanceAbsMeanMeters;
        public float GameNativeGaitStanceClearanceAbsMaxMeters;
        public float GameNativeGaitSwingClearanceAbsMeanMeters;
        public float GameNativeGaitSwingClearanceAbsMaxMeters;
        public int GameNativeGaitStanceSurfaceSwitches;
        public bool GameNativeGaitTraceQualified;
        public string GameNativeGaitLastReason;
        public string GameNativeGaitTraceSummary;

        // v0.3.58 Dynamic Foot + data-bounded CharacterJoint + source-bone relative Ankle PD causal A/B.
        public string AnkleDriveMode;
        public string AnkleDriveArm;
        public bool AnkleDriveFinite = true;
        public string AnkleDriveInvalidReason;
        public string AnkleDriveCurrentSummary;
        public string AnkleDriveTraceSummary;

        // v0.3.63 Dynamic-Foot positional-backend causal A/B: native CharacterJoint versus ARL-owned COM-only tether.
        public string AnkleConstraintMode;
        public string AnkleConstraintArm;
        public bool AnkleConstraintFinite = true;
        public string AnkleConstraintInvalidReason;
        public string AnkleConstraintCurrentSummary;
        // v0.4.12 exposes the already-computed v0.3.63 tether command numerically so the stance-only
        // competition observer can consume it without parsing formatted status strings. These are readback only.
        public bool AnkleConstraintCurrentDynamic;
        public Vector3 AnkleConstraintCurrentCorrectionAccelerationWorld;
        public Vector3 AnkleConstraintCurrentTransportAccelerationWorld;
        public Vector3 AnkleConstraintCurrentRawAccelerationWorld;
        public Vector3 AnkleConstraintCurrentAppliedAccelerationWorld;
        public bool AnkleConstraintCurrentClamped;
        public string AnkleConstraintTraceSummary;

        // v0.3.64 translation-tether parent-transport causal A/B. The physical topology remains no-Joint in the
        // official comparison; only measured Lower ankle-point acceleration feedforward changes.
        public string AnkleTransportMode;
        public string AnkleTransportArm;
        public bool AnkleTransportFinite = true;
        public string AnkleTransportInvalidReason;
        public string AnkleTransportCurrentSummary;
        public string AnkleTransportTraceSummary;

        // v0.3.65 angular common-mode parent-transport causal A/B. Official comparison fixes the no-Joint topology
        // and v0.3.64 translation transport B; only measured Lower angular-acceleration transport changes.
        public string AnkleAngularTransportMode;
        public string AnkleAngularTransportPolicy;
        public bool AnkleAngularTransportFinite = true;
        public string AnkleAngularTransportInvalidReason;
        public string AnkleAngularTransportCurrentSummary;
        public string AnkleAngularTransportTraceSummary;

        // v0.3.68 source-side persistence conditioner, promoted in v0.3.69 as the frozen free-space target-A path.
        // It is updated only by real Render samples; FixedUpdate consumes only its held conditioned result.
        public string TargetAccelerationConditionerMode;
        public bool TargetAccelerationConditionerFinite = true;
        public string TargetAccelerationConditionerInvalidReason;
        public string TargetAccelerationConditionerCurrentSummary;
        public string TargetAccelerationConditionerTraceSummary;

        // v0.3.73 production: exact v0.3.71/v0.3.72 conditional MaxA80 capture law promoted into the frozen free-space contract.
        public string TranslationCaptureAuthorityMode;
        public string TranslationCaptureAuthorityArm;
        public bool TranslationCaptureAuthorityFinite = true;
        public string TranslationCaptureAuthorityInvalidReason;
        public string TranslationCaptureAuthorityCurrentSummary;
        public string TranslationCaptureAuthorityTraceSummary;

        // v0.4.17.1 predictive de-clamp remains frozen as a rare tail-protection layer in v0.4.21. The hard 30 mm gate remains unchanged.
        public string TranslationPredictiveTailCaptureMode;
        public string TranslationPredictiveTailCaptureArm;
        public bool TranslationPredictiveTailCaptureFinite = true;
        public string TranslationPredictiveTailCaptureInvalidReason;
        public bool TranslationPredictiveTailCaptureEnabled;
        public int TranslationPredictiveTailCaptureTraceSamples;
        public int TranslationPredictiveTailCaptureDynamicSamples;
        public int TranslationPredictiveTailCaptureBaseEligibleSamples;
        public int TranslationPredictiveTailCapturePredictedBreachSamples;
        public int TranslationPredictiveTailCaptureReserveActiveSamples;
        public int TranslationPredictiveTailCapturePredictedFinalBreachSamples;
        public int TranslationPredictiveTailCaptureUnexpectedElevationSamples;
        public float TranslationPredictiveTailCapturePredictedBaselineAnchorMaxMeters;
        public float TranslationPredictiveTailCapturePredictedFinalAnchorMaxMeters;
        public float TranslationPredictiveTailCaptureFinalSelectedMaxAccelerationMax;
        public int TranslationPredictiveTailCaptureFirstReserveFixed;
        public int TranslationPredictiveTailCaptureLastReserveFixed;
        public string TranslationPredictiveTailCaptureCurrentSummary;
        public string TranslationPredictiveTailCaptureTraceSummary;

        private string GetSourceRenderStepSummary()
        {
            return "valid=" + SourceAnkleRenderStepValid + "/dt=" + F(SourceAnkleRenderStepSeconds) +
                   "/deg=" + V(SourceAnkleRenderStepDegrees) + "/angle=" + F(SourceAnkleRenderStepDegrees.magnitude) +
                   "/traceN=" + TraceSourceRenderStepSamples + "/tracePeakDeg=" + F(TraceSourceRenderStepAngleMaxDegrees);
        }

        private string GetStepEvidenceSummary()
        {
            return "readback=BEFORE_LIFECYCLE_AND_FOOT_CONTROL" +
                   "/previous={" + PreviousObservedStep.GetSummary() + "}/current={" + CurrentObservedStep.GetSummary() + "}" +
                   "/lastEvaluated(tether/drive)=" + AnkleConstraintLastEvaluatedFixedSequence + "/" + AnkleDriveLastEvaluatedFixedSequence +
                   "/lastCompletedFootControl=" + FootControllerLastCompletedFixedSequence +
                   "/observedTrace(n/anchorPeakMm/trackingPeakDeg)=" + TraceObservedFixedSamples + "/" + F(TraceObservedAnchorErrorMaxMeters * 1000f) + "/" + F(TraceObservedTrackingErrorMaxDegrees) +
                   "/completedTrace(n/anchorPeakMm)=" + TraceSamples + "/" + F(TraceAnkleAnchorErrorMaxMeters * 1000f);
        }

        // Only explicit free-space callers opt out. Any exposure/failure restores the full legacy evidence.
        private bool SupportOutputNeeded => ColliderEnabled || GravityEnabled || ContactShadowActive ||
            StanceContactLifecycleEnabled || StanceFootNativeGravityEnabled || ContactShadowTraceMatchedCallbacks > 0 ||
            MechanicalReactionTraceCallbacks > 0 || StanceContactLifecycleActiveSamples > 0 || !Finite ||
            !SoleRequalificationFinite || !ContactIdentityFinite || !ContactShadowFinite || !StanceContactLifecycleFinite ||
            !MechanicalReactionFinite || !SupportEntityBoundaryFinite || !SupportEntityBoundaryMotionFinite ||
            !SupportNormalWindowFinite || !SupportCallbackMomentumFinite || !DynamicNormalLoadFinite ||
            !RockingTetherCouplingFinite || !FootAngularImpulseAttributionFinite;

        public string GetConsoleSummary(bool includeSupport = true)
        {
            return "ARL foot/ankle foundation" +
                " | mode=" + ModeName +
                " | built=" + Built +
                " | side=" + (Side ?? "<none>") +
                " | finite=" + Finite +
                " | ankle=" + (AnklePath ?? "<none>") +
                " | toe=" + (ToePath ?? "<none>") +
                " | toeResolve=" + (ToeResolution ?? "<none>") +
                " | boneLength=" + F(FootLength) +
                " | sole=" + (GeometryMode ?? "<none>") +
                "/meshCalibrated=" + GeometryMeshCalibrated +
                "/contactAuthorized=" + ContactGeometryAuthorized +
                "/dims(L-W-T)=" + F(ProxyTotalLength) + "-" + F(ProxyWidth) + "-" + F(ProxyThickness) +
                "/heel-forward=" + F(ProxyHeelLength) + "-" + F(ProxyForwardLength) +
                "/ankleH=" + F(ProxyAnkleHeight) +
                "/supportCal=" + SupportPlaneCalibrationApplied +
                "/supportCorr=" + F(SupportPlaneCalibrationCorrectionMeters) +
                "/supportReason=" + (SupportPlaneCalibrationReason ?? "<none>") +
                "/footwearResolved=" + FootwearStateResolved +
                "/projectedBoneLen=" + F(ProjectedAnkleToeLength) +
                "/boneToSoleDeg=" + F(BoneToSoleCorrectionDegrees) +
                "/toeDot=" + F(SoleToeForwardDot) +
                "/upDot=" + F(SoleUpReferenceUpDot) +
                " | mass=" + F(Mass) +
                " | box=" + V(ColliderSizeLocal) +
                " | massProps=" + MassPropertiesValidated +
                " | kinematic=" + Kinematic +
                " | gravity=" + GravityEnabled +
                " | collider=" + ColliderEnabled +
                " | colliderFrozenOff=" + ColliderFrozenOff +
                " | ankleJoint=" + AnkleJointExists + "/connected=" + AnkleConnectedBodyMatch +
                "/axis=" + V(AnkleAxisLocal) + "/swingAxis=" + V(AnkleSwingAxisLocal) +
                " | visual=" + VisualEnabled +
                " | " + RuntimeContext.Status() +
                " | timing=localFixed#" + FixedUpdateSequence + "/render#" + TargetRenderSampleSequence + "/fresh=" + TargetRenderSampleFreshAtFixedStep + "/held=" + FixedStepsSinceTargetRenderSample + "/age=" + F(TargetRenderSampleAgeSeconds) +
                " | stepEvidence=" + GetStepEvidenceSummary() +
                " | sourceRenderStep=" + GetSourceRenderStepSummary() +
                " | sourceFromBindDeg=" + V(SourceAnkleFromBindRotationVectorDegrees) + "/angle=" + F(SourceAnkleFromBindAngleDegrees) +
                " | sourceW=" + V(SourceAnkleRelativeAngularVelocityLocal) + "/mag=" + F(SourceAnkleRelativeAngularSpeed) +
                " | sourceA=" + V(SourceAnkleRelativeAngularAccelerationLocal) +
                " | continuity(errDeg/anchor)=" + F(AnkleRelativeTrackingErrorDegrees) + "/" + F(AnkleAnchorErrorMeters) +
                (includeSupport || SupportOutputNeeded ?
                " | v0561=" + (SoleRequalificationMode ?? "<none>") + "/finite=" + SoleRequalificationFinite +
                " | v0561Now=" + (SoleRequalificationCurrentSummary ?? "<none>") +
                " | v0561Trace=" + (SoleRequalificationTraceSummary ?? "<none>") +
                " | v0400ContactIdentity=" + (ContactIdentityMode ?? "<none>") + "/finite=" + ContactIdentityFinite +
                " | v0400Now=" + (ContactIdentityCurrentSummary ?? "<none>") +
                " | v0400Trace=" + (ContactIdentityTraceSummary ?? "<none>") +
                " | v0410ContactShadow=" + (ContactShadowMode ?? "<none>") + "/arm=" + (ContactShadowArm ?? "<none>") + "/finite=" + ContactShadowFinite +
                " | v0410Now=" + (ContactShadowCurrentSummary ?? "<none>") +
                " | v0410Trace=" + (ContactShadowTraceSummary ?? "<none>") +
                " | v0430MechanicalReaction=" + (MechanicalReactionMode ?? "<none>") + "/finite=" + MechanicalReactionFinite +
                " | v0430Now=" + (MechanicalReactionCurrentSummary ?? "<none>") +
                " | v0430Trace=" + (MechanicalReactionTraceSummary ?? "<none>") +
                " | v04270VelocitySolver=" + (FootVelocitySolverSummary ?? "<none>") +
                " | v04310PositionSolver=" + (FootPositionSolverSummary ?? "<none>") +
                " | v04260SupportAngular=" + (StanceSupportAngularSummary ?? "<none>") +
                " | v04240FootCollision=" + (FootCollisionDetectionModeSummary ?? "<none>") +
                " | v04390ContactOffset=" + (FootContactOffsetSummary ?? "<none>") +
                " | v04250AngularImpulse=" + (FootAngularImpulseAttributionMode ?? "<none>") + "/finite=" + FootAngularImpulseAttributionFinite + "/ready=" + FootAngularImpulseAttributionEvidenceReady +
                " | v04250Now=" + (FootAngularImpulseAttributionCurrentSummary ?? "<none>") +
                " | v04250Trace=" + (FootAngularImpulseAttributionTraceSummary ?? "<none>")  : " | inactiveSupport=OMITTED_FREE_SPACE_NO_EXPOSURE") +
                " | v0450GameNativeGait=" + (GameNativeGaitPhaseMode ?? "<none>") + "/finite=" + GameNativeGaitPhaseFinite +
                " | v0450Trace=" + (GameNativeGaitTraceSummary ?? "<none>") +
                " | v058=" + (AnkleDriveMode ?? "<none>") + "/arm=" + (AnkleDriveArm ?? "<none>") + "/finite=" + AnkleDriveFinite +
                " | v058Now=" + (AnkleDriveCurrentSummary ?? "<none>") +
                " | v058Trace=" + (AnkleDriveTraceSummary ?? "<none>") +
                " | v063=" + (AnkleConstraintMode ?? "<none>") + "/arm=" + (AnkleConstraintArm ?? "<none>") + "/finite=" + AnkleConstraintFinite +
                " | v063Now=" + (AnkleConstraintCurrentSummary ?? "<none>") +
                " | v063Trace=" + (AnkleConstraintTraceSummary ?? "<none>") +
                " | v064=" + (AnkleTransportMode ?? "<none>") + "/arm=" + (AnkleTransportArm ?? "<none>") + "/finite=" + AnkleTransportFinite +
                " | v064Now=" + (AnkleTransportCurrentSummary ?? "<none>") +
                " | v064Trace=" + (AnkleTransportTraceSummary ?? "<none>") +
                " | v078AngularTransport=" + (AnkleAngularTransportMode ?? "<none>") + "/policy=" + (AnkleAngularTransportPolicy ?? "<none>") + "/finite=" + AnkleAngularTransportFinite +
                " | v078AngularTransportNow=" + (AnkleAngularTransportCurrentSummary ?? "<none>") +
                " | v078AngularTransportTrace=" + (AnkleAngularTransportTraceSummary ?? "<none>") +
                " | v068=" + (TargetAccelerationConditionerMode ?? "<none>") + "/finite=" + TargetAccelerationConditionerFinite +
                " | v068Now=" + (TargetAccelerationConditionerCurrentSummary ?? "<none>") +
                " | v068Trace=" + (TargetAccelerationConditionerTraceSummary ?? "<none>") +
                " | v073Capture=" + (TranslationCaptureAuthorityMode ?? "<none>") + "/arm=" + (TranslationCaptureAuthorityArm ?? "<none>") + "/finite=" + TranslationCaptureAuthorityFinite +
                " | v073CaptureNow=" + (TranslationCaptureAuthorityCurrentSummary ?? "<none>") +
                " | v073CaptureTrace=" + (TranslationCaptureAuthorityTraceSummary ?? "<none>") +
                " | v04171PredictiveCapture=" + (TranslationPredictiveTailCaptureMode ?? "<none>") + "/arm=" + (TranslationPredictiveTailCaptureArm ?? "<none>") + "/finite=" + TranslationPredictiveTailCaptureFinite +
                " | v04171PredictiveCaptureNow=" + (TranslationPredictiveTailCaptureCurrentSummary ?? "<none>") +
                " | v04171PredictiveCaptureTrace=" + (TranslationPredictiveTailCaptureTraceSummary ?? "<none>") +
                " | trace(n/errMeanMax/anchorMeanMax/wMeanMax)=" + TraceSamples + "/" + F(TraceAnkleRelativeErrorMeanDegrees) + "/" + F(TraceAnkleRelativeErrorMaxDegrees) + "/" + F(TraceAnkleAnchorErrorMeanMeters) + "/" + F(TraceAnkleAnchorErrorMaxMeters) + "/" + F(TraceSourceAngularSpeedMean) + "/" + F(TraceSourceAngularSpeedMax);
        }

        public string GetFullText(bool includeSupport = true)
        {
            StringBuilder sb = new StringBuilder(4096);
            sb.AppendLine("MakemitAGA Active Ragdoll Lab - Foot/Ankle | sourceRevision=" + Plugin.SourceRevision);
            sb.AppendLine("mode=" + ModeName);
            sb.AppendLine("built=" + Built + "/finite=" + Finite + "/stage=" + (Stage ?? "<none>"));
            sb.AppendLine("detail=" + (Detail ?? "<none>"));
            sb.AppendLine("side=" + (Side ?? "<none>"));
            sb.AppendLine("ankle=" + (AnklePath ?? "<none>"));
            sb.AppendLine("toe=" + (ToePath ?? "<none>") + "/resolve=" + (ToeResolution ?? "<none>"));
            sb.AppendLine("boneFootLength=" + F(FootLength) + "/mass=" + F(Mass));
            sb.AppendLine("geometryMode=" + (GeometryMode ?? "<none>") + "/meshCalibrated=" + GeometryMeshCalibrated + "/contactAuthorized=" + ContactGeometryAuthorized);
            sb.AppendLine("proxyForwardHeelTotal=" + F(ProxyForwardLength) + "/" + F(ProxyHeelLength) + "/" + F(ProxyTotalLength));
            sb.AppendLine("proxyWidthThicknessAnkleHeight=" + F(ProxyWidth) + "/" + F(ProxyThickness) + "/" + F(ProxyAnkleHeight));
            sb.AppendLine("supportPlaneCalibration=attempted:" + SupportPlaneCalibrationAttempted + "/applied:" + SupportPlaneCalibrationApplied + "/reason:" + (SupportPlaneCalibrationReason ?? "<none>"));
            sb.AppendLine("supportPlaneHeight(initial/correction/final)=" + F(SupportPlaneCalibrationInitialAnkleHeightMeters) + "/" + F(SupportPlaneCalibrationCorrectionMeters) + "/" + F(SupportPlaneCalibrationFinalAnkleHeightMeters));
            sb.AppendLine("supportPlaneProbe(hits/spread/normal/surface)=" + SupportPlaneCalibrationHitCount + "/" + F(SupportPlaneCalibrationSpreadMeters) + "/" + F(SupportPlaneCalibrationNormalDot) + "/" + (SupportPlaneCalibrationSurfacePath ?? "<none>"));
            sb.AppendLine("footwearStateResolved=" + FootwearStateResolved + "/policy=" + (FootwearGeometryPolicy ?? "<none>"));
            sb.AppendLine("projectedAnkleToeLength=" + F(ProjectedAnkleToeLength) + "/boneToSoleCorrectionDeg=" + F(BoneToSoleCorrectionDegrees));
            sb.AppendLine("boneToSoleBindRotation=" + Q(BoneToSoleBindRotation));
            sb.AppendLine("soleToeForwardDot=" + F(SoleToeForwardDot) + "/soleUpReferenceUpDot=" + F(SoleUpReferenceUpDot));
            sb.AppendLine("sourceBindSoleDeltaDeg(FlexX/AxialY/InvZ)=" + V(SourceBindSoleDeltaDegrees));
            sb.AppendLine("colliderCenter=" + V(ColliderCenterLocal) + "/size=" + V(ColliderSizeLocal));
            sb.AppendLine("massPropertiesValidated=" + MassPropertiesValidated + "/com=" + V(RuntimeCenterOfMassLocal) + "/inertia=" + V(RuntimeInertiaTensor));
            sb.AppendLine("kinematic=" + Kinematic + "/gravity=" + GravityEnabled + "/collider=" + ColliderEnabled + "/colliderFrozenOff=" + ColliderFrozenOff + "/visual=" + VisualEnabled);
            sb.AppendLine("ankleJoint=" + AnkleJointExists + "/connected=" + AnkleConnectedBodyMatch + "/anchor=" + V(AnkleJointAnchorLocal) + "/connectedAnchor=" + V(AnkleConnectedAnchorLocal));
            sb.AppendLine("ankleAxis/swingAxis=" + V(AnkleAxisLocal) + "/" + V(AnkleSwingAxisLocal));
            sb.AppendLine("ankleLimits(low/high/s1/s2)=" + F(AnkleLowTwistLimit) + "/" + F(AnkleHighTwistLimit) + "/" + F(AnkleSwing1Limit) + "/" + F(AnkleSwing2Limit));
            sb.AppendLine(RuntimeContext.Status());
            sb.AppendLine(TurnEventDetails ?? "FOOT_TURN_V04650=NOT_FINALIZED");
            sb.AppendLine("timing=fixed#" + FixedUpdateSequence + "/render#" + TargetRenderSampleSequence + "/fresh=" + TargetRenderSampleFreshAtFixedStep + "/held=" + FixedStepsSinceTargetRenderSample + "/age=" + F(TargetRenderSampleAgeSeconds));
            sb.AppendLine("bindAnkleRelative=" + Q(BindAnkleRelativeRotation));
            sb.AppendLine("sourceAnkleRelative=" + Q(SourceAnkleRelativeRotation));
            sb.AppendLine("stepEvidence=" + GetStepEvidenceSummary());
            sb.AppendLine("sourceRenderStep=" + GetSourceRenderStepSummary());
            sb.AppendLine("sourceFromBindDeg=" + V(SourceAnkleFromBindRotationVectorDegrees) + "/angle=" + F(SourceAnkleFromBindAngleDegrees));
            sb.AppendLine("sourceW=" + V(SourceAnkleRelativeAngularVelocityLocal) + "/speed=" + F(SourceAnkleRelativeAngularSpeed));
            sb.AppendLine("sourceA=" + V(SourceAnkleRelativeAngularAccelerationLocal));
            sb.AppendLine("sampledFootLinearVelocity=" + V(SampledFootLinearVelocityWorld));
            sb.AppendLine("continuityErrorDeg=" + F(AnkleRelativeTrackingErrorDegrees) + "/anchor=" + F(AnkleAnchorErrorMeters));
            sb.AppendLine("traceSamples=" + TraceSamples);
            sb.AppendLine("traceAnkleErrorMeanMax=" + F(TraceAnkleRelativeErrorMeanDegrees) + "/" + F(TraceAnkleRelativeErrorMaxDegrees));
            sb.AppendLine("traceAnchorMeanMax=" + F(TraceAnkleAnchorErrorMeanMeters) + "/" + F(TraceAnkleAnchorErrorMaxMeters));
            sb.AppendLine("traceSourceFromBindMeanMax=" + F(TraceSourceFromBindAngleMeanDegrees) + "/" + F(TraceSourceFromBindAngleMaxDegrees));
            sb.AppendLine("traceSourceWMeanMax=" + F(TraceSourceAngularSpeedMean) + "/" + F(TraceSourceAngularSpeedMax));
            sb.AppendLine("traceSourceFromBindAxisMinMaxAbs=" + V(TraceSourceFromBindAxisMinDegrees) + "/" + V(TraceSourceFromBindAxisMaxDegrees) + "/" + V(TraceSourceFromBindAxisAbsMaxDegrees));
            sb.AppendLine("traceSourceWAxisMinMaxAbs=" + V(TraceSourceAngularVelocityAxisMin) + "/" + V(TraceSourceAngularVelocityAxisMax) + "/" + V(TraceSourceAngularVelocityAxisAbsMax));
            sb.AppendLine("footCollisionDetectionMode=" + (FootCollisionDetectionModeSummary ?? "<none>"));
            sb.AppendLine("footContactOffset=" + (FootContactOffsetSummary ?? "<none>"));
            if (includeSupport || SupportOutputNeeded)
            {
                sb.AppendLine("soleRequalificationMode=" + (SoleRequalificationMode ?? "<none>"));
                sb.AppendLine("soleRequalificationFinite=" + SoleRequalificationFinite + "/reason=" + (SoleRequalificationInvalidReason ?? "<none>"));
                sb.AppendLine("soleRequalificationCurrent=" + (SoleRequalificationCurrentSummary ?? "<none>"));
                sb.AppendLine("soleRequalificationTrace=" + (SoleRequalificationTraceSummary ?? "<none>"));
                sb.AppendLine("contactIdentityMode=" + (ContactIdentityMode ?? "<none>"));
                sb.AppendLine("contactIdentityFinite=" + ContactIdentityFinite + "/reason=" + (ContactIdentityInvalidReason ?? "<none>"));
                sb.AppendLine("contactIdentityCurrent=" + (ContactIdentityCurrentSummary ?? "<none>"));
                sb.AppendLine("contactIdentityTrace=" + (ContactIdentityTraceSummary ?? "<none>"));
                sb.AppendLine("contactShadowMode=" + (ContactShadowMode ?? "<none>") + "/arm=" + (ContactShadowArm ?? "<none>"));
                sb.AppendLine("contactShadowFinite=" + ContactShadowFinite + "/reason=" + (ContactShadowInvalidReason ?? "<none>"));
                sb.AppendLine("contactShadowCurrent=" + (ContactShadowCurrentSummary ?? "<none>"));
                sb.AppendLine("contactShadowTrace=" + (ContactShadowTraceSummary ?? "<none>"));
                sb.AppendLine("stanceContactLifecycleMode=" + (StanceContactLifecycleMode ?? "<none>"));
                sb.AppendLine("stanceContactLifecycleFinite=" + StanceContactLifecycleFinite + "/reason=" + (StanceContactLifecycleInvalidReason ?? "<none>"));
                sb.AppendLine("stanceContactLifecycleCurrent=" + (StanceContactLifecycleCurrentSummary ?? "<none>"));
                sb.AppendLine("stanceContactLifecycleTrace=" + (StanceContactLifecycleTraceSummary ?? "<none>"));
                sb.AppendLine("stanceContactFrictionIsolation=" + (StanceContactFrictionIsolationSummary ?? "<none>"));
                sb.AppendLine("stanceLowerGroundAnchorAuthority=" + (StanceLowerGroundAuthoritySummary ?? "<none>"));
                sb.AppendLine("stanceFootNativeGravitySupportLoad=" + (StanceFootNativeGravitySummary ?? "<none>"));
                sb.AppendLine("mechanicalReactionMode=" + (MechanicalReactionMode ?? "<none>"));
                sb.AppendLine("mechanicalReactionFinite=" + MechanicalReactionFinite + "/reason=" + (MechanicalReactionInvalidReason ?? "<none>"));
                sb.AppendLine("mechanicalReactionCurrent=" + (MechanicalReactionCurrentSummary ?? "<none>"));
                sb.AppendLine("mechanicalReactionTrace=" + (MechanicalReactionTraceSummary ?? "<none>"));
                sb.AppendLine("stancePlanarDamping=" + (StancePlanarDampingSummary ?? "<none>"));
                sb.AppendLine("supportEntityBoundaryMode=" + (SupportEntityBoundaryMode ?? "<none>"));
                sb.AppendLine("supportEntityBoundaryFinite=" + SupportEntityBoundaryFinite + "/ready=" + SupportEntityBoundaryEvidenceReady + "/reason=" + (SupportEntityBoundaryInvalidReason ?? "<none>"));
                sb.AppendLine("supportEntityBoundaryTrace=" + (SupportEntityBoundaryTraceSummary ?? "<none>"));
                sb.AppendLine("supportMotionWindowMode=" + (SupportEntityBoundaryMotionMode ?? "<none>"));
                sb.AppendLine("supportMotionWindowFinite=" + SupportEntityBoundaryMotionFinite + "/reason=" + (SupportEntityBoundaryMotionInvalidReason ?? "<none>"));
                sb.AppendLine("supportMotionWindowTrace=" + (SupportEntityBoundaryMotionTraceSummary ?? "<none>"));
                sb.AppendLine("supportNormalWindowMode=" + (SupportNormalWindowMode ?? "<none>"));
                sb.AppendLine("supportNormalWindowFinite=" + SupportNormalWindowFinite + "/ready=" + SupportNormalWindowEvidenceReady + "/reason=" + (SupportNormalWindowInvalidReason ?? "<none>"));
                sb.AppendLine("supportNormalWindowTrace=" + (SupportNormalWindowTraceSummary ?? "<none>"));
                sb.AppendLine("supportCallbackMomentumMode=" + (SupportCallbackMomentumMode ?? "<none>"));
                sb.AppendLine("supportCallbackMomentumFinite=" + SupportCallbackMomentumFinite + "/reason=" + (SupportCallbackMomentumInvalidReason ?? "<none>"));
                sb.AppendLine("supportCallbackMomentumTrace=" + (SupportCallbackMomentumTraceSummary ?? "<none>"));
                sb.AppendLine("supportEntityBoundaryRecent12Frames(SI_units;before_gravity_clear_and_lifecycle):");
                sb.AppendLine(SupportEntityBoundaryRecentFrames ?? "<none>");
                sb.AppendLine("dynamicNormalLoadMode=" + (DynamicNormalLoadMode ?? "<none>"));
                sb.AppendLine("dynamicNormalLoadFinite=" + DynamicNormalLoadFinite + "/ready=" + DynamicNormalLoadEvidenceReady + "/reason=" + (DynamicNormalLoadInvalidReason ?? "<none>"));
                sb.AppendLine("dynamicNormalLoadTrace=" + (DynamicNormalLoadTraceSummary ?? "<none>"));
                sb.AppendLine("dynamicNormalLoadPeakFrames_Ns:").AppendLine(DynamicNormalLoadPeakFrames ?? "<none>");
                sb.AppendLine("rockingTetherCouplingMode=" + (RockingTetherCouplingMode ?? "<none>"));
                sb.AppendLine("rockingTetherCouplingFinite=" + RockingTetherCouplingFinite + "/ready=" + RockingTetherCouplingEvidenceReady + "/reason=" + (RockingTetherCouplingInvalidReason ?? "<none>"));
                sb.AppendLine("rockingTetherCouplingTrace=" + (RockingTetherCouplingTraceSummary ?? "<none>"));
                sb.AppendLine("rockingTetherCouplingRecentFrames_SI_WORLD_VECTORS_NORMAL_AXIS_PRE_SOLVE:");
                sb.AppendLine(RockingTetherCouplingRecentFrames ?? "<none>");
                sb.AppendLine("footAngularImpulseAttributionMode=" + (FootAngularImpulseAttributionMode ?? "<none>"));
                sb.AppendLine("footAngularImpulseAttributionFinite=" + FootAngularImpulseAttributionFinite + "/ready=" + FootAngularImpulseAttributionEvidenceReady + "/reason=" + (FootAngularImpulseAttributionInvalidReason ?? "<none>"));
                sb.AppendLine("footAngularImpulseAttributionCurrent=" + (FootAngularImpulseAttributionCurrentSummary ?? "<none>"));
                sb.AppendLine("footAngularImpulseAttributionTrace=" + (FootAngularImpulseAttributionTraceSummary ?? "<none>"));
            }
            else sb.AppendLine("inactiveSupport=OMITTED_FREE_SPACE_NO_EXPOSURE");
            sb.AppendLine("gameNativeGaitPhaseMode=" + (GameNativeGaitPhaseMode ?? "<none>"));
            sb.AppendLine("gameNativeGaitPhaseFinite=" + GameNativeGaitPhaseFinite + "/reason=" + (GameNativeGaitPhaseInvalidReason ?? "<none>"));
            sb.AppendLine("gameNativeGaitPhaseTrace=" + (GameNativeGaitTraceSummary ?? "<none>"));
            sb.AppendLine("ankleDriveMode=" + (AnkleDriveMode ?? "<none>"));
            sb.AppendLine("ankleDriveArm=" + (AnkleDriveArm ?? "<none>"));
            sb.AppendLine("ankleDriveFinite=" + AnkleDriveFinite + "/reason=" + (AnkleDriveInvalidReason ?? "<none>"));
            sb.AppendLine("ankleDriveCurrent=" + (AnkleDriveCurrentSummary ?? "<none>"));
            sb.AppendLine("ankleDriveTrace=" + (AnkleDriveTraceSummary ?? "<none>"));
            sb.AppendLine("ankleConstraintMode=" + (AnkleConstraintMode ?? "<none>"));
            sb.AppendLine("ankleConstraintArm=" + (AnkleConstraintArm ?? "<none>"));
            sb.AppendLine("ankleConstraintFinite=" + AnkleConstraintFinite + "/reason=" + (AnkleConstraintInvalidReason ?? "<none>"));
            sb.AppendLine("ankleConstraintCurrent=" + (AnkleConstraintCurrentSummary ?? "<none>"));
            sb.AppendLine("ankleConstraintTrace=" + (AnkleConstraintTraceSummary ?? "<none>"));
            sb.AppendLine("ankleTransportMode=" + (AnkleTransportMode ?? "<none>") + "/arm=" + (AnkleTransportArm ?? "<none>"));
            sb.AppendLine("ankleTransportFinite=" + AnkleTransportFinite + "/reason=" + (AnkleTransportInvalidReason ?? "<none>"));
            sb.AppendLine("ankleTransportCurrent=" + (AnkleTransportCurrentSummary ?? "<none>"));
            sb.AppendLine("ankleTransportTrace=" + (AnkleTransportTraceSummary ?? "<none>"));
            sb.AppendLine("ankleAngularTransportMode=" + (AnkleAngularTransportMode ?? "<none>") + "/policy=" + (AnkleAngularTransportPolicy ?? "<none>"));
            sb.AppendLine("ankleAngularTransportFinite=" + AnkleAngularTransportFinite + "/reason=" + (AnkleAngularTransportInvalidReason ?? "<none>"));
            sb.AppendLine("ankleAngularTransportCurrent=" + (AnkleAngularTransportCurrentSummary ?? "<none>"));
            sb.AppendLine("ankleAngularTransportTrace=" + (AnkleAngularTransportTraceSummary ?? "<none>"));
            sb.AppendLine("targetAccelerationConditionerMode=" + (TargetAccelerationConditionerMode ?? "<none>"));
            sb.AppendLine("targetAccelerationConditionerFinite=" + TargetAccelerationConditionerFinite + "/reason=" + (TargetAccelerationConditionerInvalidReason ?? "<none>"));
            sb.AppendLine("targetAccelerationConditionerCurrent=" + (TargetAccelerationConditionerCurrentSummary ?? "<none>"));
            sb.AppendLine("targetAccelerationConditionerTrace=" + (TargetAccelerationConditionerTraceSummary ?? "<none>"));
            sb.AppendLine("translationCaptureAuthorityMode=" + (TranslationCaptureAuthorityMode ?? "<none>") + "/arm=" + (TranslationCaptureAuthorityArm ?? "<none>"));
            sb.AppendLine("translationCaptureAuthorityFinite=" + TranslationCaptureAuthorityFinite + "/reason=" + (TranslationCaptureAuthorityInvalidReason ?? "<none>"));
            sb.AppendLine("translationCaptureAuthorityCurrent=" + (TranslationCaptureAuthorityCurrentSummary ?? "<none>"));
            sb.AppendLine("translationCaptureAuthorityTrace=" + (TranslationCaptureAuthorityTraceSummary ?? "<none>"));
            sb.AppendLine("translationPredictiveTailCaptureMode=" + (TranslationPredictiveTailCaptureMode ?? "<none>") + "/arm=" + (TranslationPredictiveTailCaptureArm ?? "<none>"));
            sb.AppendLine("translationPredictiveTailCaptureFinite=" + TranslationPredictiveTailCaptureFinite + "/reason=" + (TranslationPredictiveTailCaptureInvalidReason ?? "<none>"));
            sb.AppendLine("translationPredictiveTailCaptureCurrent=" + (TranslationPredictiveTailCaptureCurrentSummary ?? "<none>"));
            sb.AppendLine("translationPredictiveTailCaptureTrace=" + (TranslationPredictiveTailCaptureTraceSummary ?? "<none>"));
            sb.AppendLine("groundGeometryReady=" + GroundGeometryReady + "/physicalGroundContactPossible=" + PhysicalGroundContactPossible + "/hits=" + GroundProbeHitCount);
            sb.AppendLine("groundClearance(center/min/max/spread)=" + F(GroundCenterClearanceMeters) + "/" + F(GroundClearanceMinMeters) + "/" + F(GroundClearanceMaxMeters) + "/" + F(GroundClearanceSpreadMeters));
            sb.AppendLine("groundNormalDotReferenceUp=" + F(GroundNormalDotReferenceUp) + "/footprintMin=" + F(GroundFootprintMinNormalDotReferenceUp) + "/avgNormalWorld=" + V(GroundAverageNormalWorld) + "/collider=" + (GroundColliderPath ?? "<none>") + "/id=" + GroundColliderInstanceId + "/sameFootprint=" + GroundFootprintSameSurface + "/static=" + !GroundColliderHasAttachedRigidbody);
            return sb.ToString();
        }

        private static string F(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + ", " + F(value.y) + ", " + F(value.z) + ")";
        private static string Q(Quaternion value) => "(" + F(value.x) + ", " + F(value.y) + ", " + F(value.z) + ", " + F(value.w) + ")";
    }
}
