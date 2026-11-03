using System;
using System.Globalization;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class FootAnkleFoundationReport
    {
        // Owners update typed receipts each physics step. Formatting belongs to readers,
        // so fixed-step copies/collision callbacks do not allocate these long summaries.
        // Preserve the historical field names, labels and numeric precision for consumers.
        // Cross-owner and separately refreshed inputs are captured at the original format
        // boundary, so a later update cannot relabel an already copied summary snapshot.
        internal string StanceContactLifecycleSummaryGait;
        internal bool StanceContactLifecycleSummaryShadowPrepared, StanceContactLifecycleSummaryShadowActive;
        internal bool FrictionSummaryGroundAssigned, FrictionSummaryEffectiveZero;
        internal float FrictionSummaryGroundStatic, FrictionSummaryGroundDynamic;
        internal string FrictionSummaryGroundCombine;

        public string ContactShadowCurrentSummary => ContactShadowMode == null ? null :
            "arm=" + ContactShadowArm +
            "/prepared=" + ContactShadowPrepared +
            "/active=" + ContactShadowActive +
            "/authorizedId=" + ContactShadowAuthorizedColliderInstanceId +
            "/authorizedSurface=" + SummaryText(ContactShadowAuthorizedColliderPath) +
            "/footCollider=" + ContactShadowFootColliderEnabled +
            "/contactActive=" + ContactShadowCurrentContactActive +
            "/sourceRegistry=" + ContactShadowSourceRegistryCount +
            "/sourceAux=" + ContactShadowSourceAuxiliaryRegistryCount +
            "/sourceReadbackFail=" + ContactShadowSourceIsolationReadbackFailures;

        public string ContactShadowTraceSummary => ContactShadowMode == null ? null :
            "fixed=" + ContactShadowTraceFixedSamples +
            "/rayMatch=" + ContactShadowTraceRaycastMatchSamples +
            "/rayNoHit=" + ContactShadowTraceRaycastNoHitSamples +
            "/rayMismatch=" + ContactShadowTraceRaycastMismatchSamples +
            "/callback(E-S-X)=" + ContactShadowTraceEnter + "-" + ContactShadowTraceStay + "-" + ContactShadowTraceExit +
            "/matched=" + ContactShadowTraceMatchedCallbacks +
            "/unexpected=" + ContactShadowTraceUnexpectedCallbacks +
            "/sceneColliders=" + ContactShadowSceneColliderCount +
            "/ownedIgnore=" + ContactShadowOwnedIsolationPairs +
            "/alreadyIgnored=" + ContactShadowAlreadyIgnoredPairs +
            "/sourceRegistry(aux)=" + ContactShadowSourceRegistryCount + "(" + ContactShadowSourceAuxiliaryRegistryCount + ")" +
            "/sourceLate=" + ContactShadowSourceLateDiscoveries +
            "/sourceRepair=" + ContactShadowSourceIsolationRepairs +
            "/sourceReadbackFail=" + ContactShadowSourceIsolationReadbackFailures +
            "/sourceCallback(E-S-X)=" + ContactShadowSourceCallbackEnter + "-" + ContactShadowSourceCallbackStay + "-" + ContactShadowSourceCallbackExit +
            "/sourceQuarantined=" + ContactShadowSourceCallbacksQuarantined +
            "/lastSource=" + ContactShadowLastSourceColliderInstanceId + ":" + SummaryText(ContactShadowLastSourceColliderPath) +
            "/isolationFailures=" + ContactShadowIsolationFailures +
            "/restoreFailures=" + ContactShadowRestoreFailures +
            "/last=" + SummaryText(ContactShadowLastCallbackPhase) + ":" + ContactShadowLastCallbackColliderInstanceId + ":" + SummaryText(ContactShadowLastCallbackColliderPath) +
            "/TRACE_CONTACT_SHADOW_QUALIFIED=" + ContactShadowTraceQualified +
            "/thresholds(rayMatch-callback)=" + ContactShadowMinRaycastMatchSamples + "-" + ContactShadowMinMatchedCallbacks +
            "/gravityReadback=STANCE_NATIVE_GRAVITY_OWNER/mechanicalReaction=READ_DOWNSTREAM_V0430_SUPPORT_NORMAL_ONLY";

        public string StanceFootNativeGravitySummary => StanceFootNativeGravityMode == null ? null :
            "mode=" + StanceFootNativeGravityMode + "/arm=" + StanceFootNativeGravityArm + "/finite=" + StanceFootNativeGravityFinite + "/enabled=" + StanceFootNativeGravityEnabled +
            "/n-eligible-applied-nonStance-readbackFail=" + StanceFootNativeGravityTraceSamples + "-" + StanceFootNativeGravityEligibleSamples + "-" + StanceFootNativeGravityAppliedSamples + "-" + StanceFootNativeGravityNonStanceAppliedSamples + "-" + StanceFootNativeGravityReadbackFailures +
            "/callbacks(total-gravityOn-gravityOff)=" + StanceFootNativeGravityCallbackSamples + "-" + StanceFootNativeGravityCallbackGravityOnSamples + "-" + StanceFootNativeGravityCallbackGravityOffSamples +
            "/expectedNormalForceMeanMaxN=" + SummaryNumber(StanceFootNativeGravityExpectedNormalForceMean) + "-" + SummaryNumber(StanceFootNativeGravityExpectedNormalForceMax) +
            "/expectedNormalImpulseMeanMaxNs=" + SummaryNumber(StanceFootNativeGravityExpectedNormalImpulseMean) + "-" + SummaryNumber(StanceFootNativeGravityExpectedNormalImpulseMax) +
            "/supportNormalSpeedAbsMeanMax=" + SummaryNumber(StanceFootNativeGravitySupportNormalSpeedAbsMean) + "-" + SummaryNumber(StanceFootNativeGravitySupportNormalSpeedAbsMax) +
            "/tetherSupportNormalA(mean-min-max)=" + SummaryNumber(StanceFootNativeGravityTetherSupportNormalAccelerationMean) + "-" + SummaryNumber(StanceFootNativeGravityTetherSupportNormalAccelerationMin) + "-" + SummaryNumber(StanceFootNativeGravityTetherSupportNormalAccelerationMax) +
            "/reactionSignedJMeanMax(gravityOn)=" + SummaryNumber(StanceFootNativeGravityGravityOnSignedNormalImpulseMean) + "-" + SummaryNumber(StanceFootNativeGravityGravityOnSignedNormalImpulseMax) +
            "/reactionSignedJMeanMax(gravityOff)=" + SummaryNumber(StanceFootNativeGravityGravityOffSignedNormalImpulseMean) + "-" + SummaryNumber(StanceFootNativeGravityGravityOffSignedNormalImpulseMax) +
            "/gravity=" + SummaryVector(StanceFootNativeGravityGravityWorld) + "/gMag=" + SummaryNumber(StanceFootNativeGravityGravityWorld.magnitude) + "/mass=" + SummaryNumber(StanceFootNativeGravityFootMass) +
            "/currentApplied=" + StanceFootNativeGravityCurrentApplied + "/currentFixed=" + StanceFootNativeGravityCurrentScheduledFixed +
            "/firstLastApplied=" + StanceFootNativeGravityFirstAppliedFixed + "-" + StanceFootNativeGravityLastAppliedFixed +
            "/reason=" + (string.IsNullOrEmpty(StanceFootNativeGravityInvalidReason) ? "<none>" : StanceFootNativeGravityInvalidReason);

        public string StanceContactFrictionIsolationSummary => StanceContactFrictionIsolationMode == null ? null :
            "mode=" + StanceContactFrictionIsolationMode + "/arm=" + StanceContactFrictionIsolationArm + "/finite=" + StanceContactFrictionIsolationFinite +
            "/zero=" + StanceContactFrictionIsolationZeroFriction + "/assigned=" + StanceContactFrictionIsolationMaterialAssigned +
            "/static=" + SummaryNumber(StanceContactFrictionIsolationStaticFriction) +
            "/dynamic=" + SummaryNumber(StanceContactFrictionIsolationDynamicFriction) +
            "/combine=" + StanceContactFrictionIsolationFrictionCombine +
            "/groundAssigned=" + FrictionSummaryGroundAssigned +
            "/groundStatic=" + SummaryNumber(FrictionSummaryGroundStatic) +
            "/groundDynamic=" + SummaryNumber(FrictionSummaryGroundDynamic) +
            "/groundCombine=" + (FrictionSummaryGroundCombine ?? "<none>") +
            "/effectiveZeroGuaranteed=" + FrictionSummaryEffectiveZero +
            "/reason=" + (string.IsNullOrEmpty(StanceContactFrictionIsolationInvalidReason) ? "<none>" : StanceContactFrictionIsolationInvalidReason);

        public string FootCollisionDetectionModeSummary => FootCollisionDetectionModeExperiment == null ? null :
            "mode=" + FootCollisionDetectionModeExperiment +
            "/arm=" + FootCollisionDetectionModeArm +
            "/finite=" + FootCollisionDetectionModeFinite +
            "/continuousSpeculative=" + FootCollisionDetectionModeSpeculative +
            "/readback=" + FootCollisionDetectionModeReadback +
            "/reason=" + (FootCollisionDetectionModeInvalidReason ?? "<none>");

        public string StanceContactLifecycleCurrentSummary => StanceContactLifecycleMode == null ? null :
            "enabled=" + StanceContactLifecycleEnabled + "/finite=" + StanceContactLifecycleFinite + "/state=" + StanceContactLifecycleState +
            "/gait=" + SummaryText(StanceContactLifecycleSummaryGait) +
            "/shadowPreparedActive=" + StanceContactLifecycleSummaryShadowPrepared + "-" + StanceContactLifecycleSummaryShadowActive +
            "/qualifiedHoldSec=" + SummaryNumber(StanceContactLifecycleQualifiedStanceSeconds) +
            "/candidateId=" + StanceContactLifecycleQualifiedColliderInstanceId +
            "/activationReadyHold=" + StanceContactLifecycleActivationGeometryReadySamples + "/" + StanceContactLifecycleMinGeometryReadySamples +
            "/authorizedId=" + StanceContactLifecycleAuthorizedColliderInstanceId +
            "/last=" + SummaryText(StanceContactLifecycleLastReason);

        public string StanceContactLifecycleTraceSummary => StanceContactLifecycleMode == null ? null :
            "fixed=" + StanceContactLifecycleFixedSamples + "/active=" + StanceContactLifecycleActiveSamples +
            "/activation=" + StanceContactLifecycleContactActivations +
            "/prepare-reuse-reprepare=" + StanceContactLifecyclePrepareCount + "-" + StanceContactLifecyclePreparedReuseCount + "-" + StanceContactLifecycleSurfaceReprepareCount +
            "/release(total-transition-swing-idLoss-surfaceChange)=" + StanceContactLifecycleReleaseCount + "-" + StanceContactLifecycleTransitionReleaseCount + "-" + StanceContactLifecycleSwingReleaseCount + "-" + StanceContactLifecycleIdentityLossReleaseCount + "-" + StanceContactLifecycleSurfaceChangeReleaseCount +
            "/identityReject=" + StanceContactLifecycleIdentityRejectSamples +
            "/activationGeometryReject=" + StanceContactLifecycleActivationGeometryRejectSamples +
            "/activationReadyHold=" + StanceContactLifecycleActivationGeometryReadySamples + "/" + StanceContactLifecycleMinGeometryReadySamples +
            "/nonStanceActive=" + StanceContactLifecycleNonStanceActiveSamples +
            "/fail(enable-disable)=" + StanceContactLifecycleEnableFailures + "-" + StanceContactLifecycleDisableFailures +
            "/firstContact-lastContact-lastRelease=" + StanceContactLifecycleFirstContactFixed + "-" + StanceContactLifecycleLastContactFixed + "-" + StanceContactLifecycleLastReleaseFixed +
            "/TRACE_STANCE_CONTACT_LIFECYCLE_QUALIFIED=" + StanceContactLifecycleTraceQualified +
            "/thresholds(identityHoldSec-geometryReadySamples-activation-release-activeSamples)=" + SummaryNumber(StanceContactLifecycleTraceIdentityHoldSeconds) + "-" + StanceContactLifecycleMinGeometryReadySamples + "-" + StanceContactLifecycleMinActivations + "-" + StanceContactLifecycleMinReleases + "-" + StanceContactLifecycleMinActiveSamples;

        private static string SummaryNumber(float value) => value.ToString("0.######", CultureInfo.InvariantCulture);
        private static string SummaryVector(Vector3 value) => "(" + SummaryNumber(value.x) + "," + SummaryNumber(value.y) + "," + SummaryNumber(value.z) + ")";
        private static string SummaryText(string value) => string.IsNullOrEmpty(value) ? "<none>" : value.Replace('\r', ' ').Replace('\n', ' ');
    }
}
