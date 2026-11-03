using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Read-only scope qualifications and retained regression evidence. Keeping these
    // reports separate from lifecycle dispatch does not create another state/physics owner.
    internal static partial class ActiveRagdollLabRuntime
    {
        public static string GetStageEFootCollisionSupportLoadContinuityAudit()
        {
            if (!AnkleTestActive && _completedAnkleTestAudit != null)
                return _completedAnkleTestAudit;
            return BuildFootCollisionSupportLoadContinuityAudit();
        }

        private static string BuildFootCollisionSupportLoadContinuityAudit(bool completed = false)
        {
            const string mode = UpperSelfWeightSupportNormalLoadTransferIsolation.Mode;
            bool primaryIsLeft = _legChain != null && string.Equals(_legChain.PrimarySide, "LEFT", StringComparison.OrdinalIgnoreCase);
            FootAnkleFoundationReport leftFoot = _legChain != null ? (primaryIsLeft ? _legChain.PrimaryFootReport : _legChain.SecondaryFootReport) : null;
            FootAnkleFoundationReport rightFoot = _legChain != null ? (primaryIsLeft ? _legChain.SecondaryFootReport : _legChain.PrimaryFootReport) : null;
            SingleLegChainReport leftLeg = _legChain != null ? (primaryIsLeft ? _legChain.PrimaryReport : _legChain.SecondaryReport) : null;
            SingleLegChainReport rightLeg = _legChain != null ? (primaryIsLeft ? _legChain.SecondaryReport : _legChain.PrimaryReport) : null;
            BilateralLegReport bilateral = _legChain != null ? _legChain.BilateralReport : null;
            TwoBodyProxyReport torso = _proxy != null ? _proxy.Report : null;
            bool built = _legChain != null && _legChain.IsBuilt && leftFoot != null && rightFoot != null && leftLeg != null && rightLeg != null;
            bool testArmKnown = UpperSelfWeightSupportNormalLoadTransferIsolation.IsExperimentArm(_ankleTestArm);
            string expectedCollisionArm = FootCollisionDetectionModeIsolation.ActiveArm;
            bool contactOffsetContract = testArmKnown && built &&
                FootContactOffsetSelectorContract(leftFoot, FootContactOffsetIsolation.ArmB) && FootContactOffsetSelectorContract(rightFoot, FootContactOffsetIsolation.ArmB) &&
                leftFoot.FootContactOffsetReadbackFailures == 0 && rightFoot.FootContactOffsetReadbackFailures == 0;
            bool contactOffsetExposure = contactOffsetContract &&
                FootContactOffsetObservedExposure(leftFoot) && FootContactOffsetObservedExposure(rightFoot);

            FootSupportSurfaceIsolation surface = _legChain != null ? _legChain.SupportSurface : null;
            bool surfaceContract = testArmKnown && built && surface != null && surface.Active && surface.Finite &&
                string.Equals(surface.Arm, FootSupportSurfaceIsolation.ArmB, StringComparison.Ordinal);
            bool surfaceExposure = surfaceContract && surface.ExposureReady;
            bool kneeReactionContract = testArmKnown && built && _legChain.KneeSupportReactionArmsMatch &&
                KneeSupportNormalReactionIsolation.SelectorContract(leftLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB) &&
                KneeSupportNormalReactionIsolation.SelectorContract(rightLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB);
            bool kneeReactionExposure = kneeReactionContract &&
                KneeSupportNormalReactionIsolation.ExposureReady(leftLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB) &&
                KneeSupportNormalReactionIsolation.ExposureReady(rightLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB);
            bool upperLoadContract = testArmKnown && built && _legChain.UpperSelfWeightArmsMatch &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.SelectorContract(leftLeg.UpperSelfWeightSupport, _ankleTestArm, completed) &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.SelectorContract(rightLeg.UpperSelfWeightSupport, _ankleTestArm, completed);
            bool upperLoadExposure = upperLoadContract &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.ExposureReady(leftLeg.UpperSelfWeightSupport, _ankleTestArm) &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.ExposureReady(rightLeg.UpperSelfWeightSupport, _ankleTestArm);


            bool frozenWorldGravityBoundary = built && torso != null && !torso.GravityEnabled && !leftLeg.GravityEnabled && !rightLeg.GravityEnabled;
            bool zeroFrictionBaseline = built && leftFoot.StanceContactFrictionIsolationZeroFriction && rightFoot.StanceContactFrictionIsolationZeroFriction &&
                                        leftFoot.StanceContactFrictionIsolationFinite && rightFoot.StanceContactFrictionIsolationFinite;
            bool frozenTetherTopology = built && leftFoot.AnkleConstraintFinite && rightFoot.AnkleConstraintFinite &&
                                string.Equals(leftFoot.AnkleConstraintArm, FootAnkleConstraintTopologyIsolation.ArmB, StringComparison.Ordinal) &&
                                string.Equals(rightFoot.AnkleConstraintArm, FootAnkleConstraintTopologyIsolation.ArmB, StringComparison.Ordinal);
            bool lowerGroundFrozen = built && leftFoot.StanceLowerGroundAuthorityFinite && rightFoot.StanceLowerGroundAuthorityFinite &&
                                     string.Equals(leftFoot.StanceLowerGroundAuthorityArm, FootStanceLowerGroundAnchorAuthority.ArmB, StringComparison.Ordinal) &&
                                     string.Equals(rightFoot.StanceLowerGroundAuthorityArm, FootStanceLowerGroundAnchorAuthority.ArmB, StringComparison.Ordinal) &&
                                     leftFoot.StanceLowerGroundAuthorityEnabled && rightFoot.StanceLowerGroundAuthorityEnabled;
            bool predictiveTailFrozen = built && leftFoot.TranslationPredictiveTailCaptureFinite && rightFoot.TranslationPredictiveTailCaptureFinite &&
                                        string.Equals(leftFoot.TranslationPredictiveTailCaptureArm, FootAnkleTranslationPredictiveTailCaptureIsolation.ArmB, StringComparison.Ordinal) &&
                                        string.Equals(rightFoot.TranslationPredictiveTailCaptureArm, FootAnkleTranslationPredictiveTailCaptureIsolation.ArmB, StringComparison.Ordinal);
            bool footGravityFrozenOn = built && leftFoot.StanceFootNativeGravityFinite && rightFoot.StanceFootNativeGravityFinite &&
                                       string.Equals(leftFoot.StanceFootNativeGravityArm, FootStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                       string.Equals(rightFoot.StanceFootNativeGravityArm, FootStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                       leftFoot.StanceFootNativeGravityEnabled && rightFoot.StanceFootNativeGravityEnabled;
            bool lowerGravityFrozenOn = built && leftLeg.StanceLowerNativeGravityFinite && rightLeg.StanceLowerNativeGravityFinite &&
                                        string.Equals(leftLeg.StanceLowerNativeGravityArm, LowerStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                        string.Equals(rightLeg.StanceLowerNativeGravityArm, LowerStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                        leftLeg.StanceLowerNativeGravityEnabled && rightLeg.StanceLowerNativeGravityEnabled;
            bool reciprocalFrozenOn = built && leftLeg.StanceFootLowerReciprocalFinite && rightLeg.StanceFootLowerReciprocalFinite &&
                                      string.Equals(leftLeg.StanceFootLowerReciprocalArm, FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                      string.Equals(rightLeg.StanceFootLowerReciprocalArm, FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                      leftLeg.StanceFootLowerReciprocalEnabled && rightLeg.StanceFootLowerReciprocalEnabled;
            bool transferFrozenOn = built && leftLeg.StanceLowerSelfWeightTransferFinite && rightLeg.StanceLowerSelfWeightTransferFinite &&
                                    string.Equals(leftLeg.StanceLowerSelfWeightTransferArm, LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                    string.Equals(rightLeg.StanceLowerSelfWeightTransferArm, LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                    leftLeg.StanceLowerSelfWeightTransferEnabled && rightLeg.StanceLowerSelfWeightTransferEnabled;
            bool collisionSelectorContract = testArmKnown && built && _legChain.FootCollisionDetectionModeFinite &&
                                             _legChain.FootCollisionDetectionModeArmsMatch &&
                                             string.Equals(_legChain.FootCollisionDetectionModeArm, expectedCollisionArm, StringComparison.Ordinal) &&
                                             FootCcdSelectorContract(leftFoot, expectedCollisionArm) &&
                                             FootCcdSelectorContract(rightFoot, expectedCollisionArm);
            bool collisionObservedExposure = collisionSelectorContract &&
                leftFoot.SupportEntityBoundaryCcdResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.SupportEntityBoundaryCcdResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                leftFoot.SupportEntityBoundaryCcdMismatchSolves == 0 && rightFoot.SupportEntityBoundaryCcdMismatchSolves == 0;
            bool supportAngularContract = testArmKnown && built && _legChain.StanceSupportAngularFinite &&
                _legChain.StanceSupportAngularArmsMatch && string.Equals(_legChain.StanceSupportAngularArm, FootStanceSupportNormalAngularStabilization.ArmB, StringComparison.Ordinal) &&
                leftFoot.StanceSupportAngularFinite && rightFoot.StanceSupportAngularFinite &&
                string.Equals(leftFoot.StanceSupportAngularArm, FootStanceSupportNormalAngularStabilization.ArmB, StringComparison.Ordinal) &&
                string.Equals(rightFoot.StanceSupportAngularArm, FootStanceSupportNormalAngularStabilization.ArmB, StringComparison.Ordinal) &&
                leftFoot.StanceSupportAngularEnabled && rightFoot.StanceSupportAngularEnabled &&
                leftFoot.StanceSupportAngularMaxNormalTorqueNm <= 0.00001f && rightFoot.StanceSupportAngularMaxNormalTorqueNm <= 0.00001f &&
                leftFoot.StanceSupportAngularMaxPowerWatts <= 0.00001f && rightFoot.StanceSupportAngularMaxPowerWatts <= 0.00001f;
            bool supportAngularExposure = built && leftFoot.StanceSupportAngularEligibleSamples >= FootStanceSupportNormalAngularStabilization.MinimumEligibleSamples &&
                rightFoot.StanceSupportAngularEligibleSamples >= FootStanceSupportNormalAngularStabilization.MinimumEligibleSamples &&
                (leftFoot.StanceSupportAngularAppliedSamples > 0 && rightFoot.StanceSupportAngularAppliedSamples > 0);
            bool velocitySolverContract = testArmKnown && built && _legChain.FootVelocitySolverFinite && _legChain.FootVelocitySolverArmsMatch &&
                string.Equals(_legChain.FootVelocitySolverArm, FootSolverIterationBaselineContract.ArmA, StringComparison.Ordinal) &&
                FootVelocitySolverContract(leftFoot) && FootVelocitySolverContract(rightFoot);
            bool velocitySolverExposure = built &&
                leftFoot.FootVelocitySolverEligibleSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.FootVelocitySolverEligibleSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves;
            bool positionSolverContract = testArmKnown && built && _legChain.FootPositionSolverArmsMatch &&
                string.Equals(_legChain.FootPositionSolverArm, FootSolverIterationBaselineContract.PositionArmB, StringComparison.Ordinal) &&
                FootPositionSolverContract(leftFoot) && FootPositionSolverContract(rightFoot);
            bool positionSolverExposure = built &&
                leftFoot.FootPositionSolverInterventionSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.FootPositionSolverInterventionSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                leftFoot.SupportEntityBoundaryPositionSolverResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.SupportEntityBoundaryPositionSolverResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves;
            bool nativeGravityEnvironment = built && leftLeg.StanceLowerNativeGravityGravityMagnitude > 0.0001f && rightLeg.StanceLowerNativeGravityGravityMagnitude > 0.0001f &&
                                            leftLeg.StanceLowerNativeGravityLowerMass > 0f && rightLeg.StanceLowerNativeGravityLowerMass > 0f &&
                                            leftLeg.StanceLowerNativeGravityExpectedNormalImpulseMean > 0f && rightLeg.StanceLowerNativeGravityExpectedNormalImpulseMean > 0f &&
                                            leftFoot.StanceFootNativeGravityExpectedNormalImpulseMean > 0f && rightFoot.StanceFootNativeGravityExpectedNormalImpulseMean > 0f;
            bool bilateralExposure = built && leftLeg.StanceLowerSelfWeightTransferEligibleSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinEligibleSamples &&
                                     rightLeg.StanceLowerSelfWeightTransferEligibleSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinEligibleSamples &&
                                     leftLeg.StanceLowerSelfWeightTransferAppliedSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinAppliedSamples &&
                                     rightLeg.StanceLowerSelfWeightTransferAppliedSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinAppliedSamples &&
                                     leftLeg.StanceLowerNativeGravityAppliedSamples >= LowerStanceNativeGravitySupportLoadIsolation.MinAppliedSamples &&
                                     rightLeg.StanceLowerNativeGravityAppliedSamples >= LowerStanceNativeGravitySupportLoadIsolation.MinAppliedSamples &&
                                     leftLeg.StanceFootLowerReciprocalAppliedSamples >= FootLowerSupportNormalReciprocalLoadTransferIsolation.MinAppliedSamples &&
                                     rightLeg.StanceFootLowerReciprocalAppliedSamples >= FootLowerSupportNormalReciprocalLoadTransferIsolation.MinAppliedSamples;
            bool noTransferLeak = built && leftLeg.StanceLowerSelfWeightTransferNonStanceAppliedSamples == 0 && rightLeg.StanceLowerSelfWeightTransferNonStanceAppliedSamples == 0;
            bool noReciprocalLeak = built && leftLeg.StanceFootLowerReciprocalNonStanceAppliedSamples == 0 && rightLeg.StanceFootLowerReciprocalNonStanceAppliedSamples == 0;
            bool noLowerGravityLeak = built && leftLeg.StanceLowerNativeGravityNonStanceAppliedSamples == 0 && rightLeg.StanceLowerNativeGravityNonStanceAppliedSamples == 0 &&
                                      leftLeg.StanceLowerNativeGravityReadbackFailures == 0 && rightLeg.StanceLowerNativeGravityReadbackFailures == 0 &&
                                      LowerGravityCurrentStateValid(leftLeg, leftFoot, completed) && LowerGravityCurrentStateValid(rightLeg, rightFoot, completed);
            bool lifecycleEvidence = built && leftFoot.StanceContactLifecycleTraceQualified && rightFoot.StanceContactLifecycleTraceQualified &&
                                     leftFoot.StanceContactLifecycleNonStanceActiveSamples == 0 && rightFoot.StanceContactLifecycleNonStanceActiveSamples == 0;
            bool supportLoadReactionDataset = built && SupportLoadReactionDatasetUsable(leftFoot) && SupportLoadReactionDatasetUsable(rightFoot);
            bool angularImpulseObserverFinite = built && leftFoot.FootAngularImpulseAttributionFinite && rightFoot.FootAngularImpulseAttributionFinite;
            bool angularImpulseEvidenceReady = angularImpulseObserverFinite && leftFoot.FootAngularImpulseAttributionEvidenceReady && rightFoot.FootAngularImpulseAttributionEvidenceReady;
            bool couplingObserverFinite = built && leftFoot.RockingTetherCouplingFinite && rightFoot.RockingTetherCouplingFinite;
            bool couplingEvidenceReady = couplingObserverFinite && leftFoot.RockingTetherCouplingEvidenceReady && rightFoot.RockingTetherCouplingEvidenceReady;
            bool supportEntityFinite = built && leftFoot.SupportEntityBoundaryFinite && rightFoot.SupportEntityBoundaryFinite;
            bool supportEntityReady = supportEntityFinite && leftFoot.SupportEntityBoundaryEvidenceReady && rightFoot.SupportEntityBoundaryEvidenceReady;
            bool planarDampingContract = testArmKnown && built &&
                leftFoot.StancePlanarDampingFinite && rightFoot.StancePlanarDampingFinite &&
                leftFoot.StancePlanarDampingEnabled && rightFoot.StancePlanarDampingEnabled &&
                string.Equals(leftFoot.StancePlanarDampingArm, FootStancePlanarRotationalDampingIsolation.ArmB, StringComparison.Ordinal) &&
                string.Equals(rightFoot.StancePlanarDampingArm, FootStancePlanarRotationalDampingIsolation.ArmB, StringComparison.Ordinal);
            bool planarDampingExposure = built &&
                leftFoot.StancePlanarDampingLoadedSolves >= FootStancePlanarRotationalDampingIsolation.MinimumLoadedSolves &&
                rightFoot.StancePlanarDampingLoadedSolves >= FootStancePlanarRotationalDampingIsolation.MinimumLoadedSolves &&
                leftFoot.StancePlanarDampingNonzeroWrites > 0 && rightFoot.StancePlanarDampingNonzeroWrites > 0;
            bool localRunSafety = built && FootRunSafety(leftFoot) && FootRunSafety(rightFoot) && bilateral != null && bilateral.ClockParityPass &&
                                  bilateral.FailClosedEvents == 0 && !bilateral.PrimaryDriveAborted && !bilateral.SecondaryDriveAborted;

            bool qualified = testArmKnown && upperLoadContract && upperLoadExposure && kneeReactionContract && kneeReactionExposure && surfaceContract && surfaceExposure && frozenWorldGravityBoundary && zeroFrictionBaseline && frozenTetherTopology && lowerGroundFrozen && predictiveTailFrozen &&
                             footGravityFrozenOn && lowerGravityFrozenOn && reciprocalFrozenOn && transferFrozenOn && collisionSelectorContract && collisionObservedExposure && contactOffsetContract && contactOffsetExposure && supportAngularContract && supportAngularExposure && velocitySolverContract && velocitySolverExposure && positionSolverContract && positionSolverExposure &&
                             nativeGravityEnvironment && bilateralExposure && noTransferLeak && noReciprocalLeak && noLowerGravityLeak &&
                             lifecycleEvidence && supportLoadReactionDataset && angularImpulseEvidenceReady && couplingEvidenceReady && supportEntityReady && planarDampingContract && planarDampingExposure && localRunSafety;

            bool normalLoadReady = qualified && leftFoot.DynamicNormalLoadFinite && rightFoot.DynamicNormalLoadFinite &&
                leftFoot.DynamicNormalLoadEvidenceReady && rightFoot.DynamicNormalLoadEvidenceReady;
            bool normalWindowReady = normalLoadReady && leftFoot.SupportNormalWindowFinite && rightFoot.SupportNormalWindowFinite &&
                leftFoot.SupportNormalWindowEvidenceReady && rightFoot.SupportNormalWindowEvidenceReady;
            HipPelvisLoadPathReport hipPelvis = _legChain?.HipPelvisLoadPath;
            bool hipPelvisReady = normalWindowReady && _ankleTestArm == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB &&
                hipPelvis != null && hipPelvis.EvidenceReady;
            string reason;
            if (!built) reason = "FOUNDATION_NOT_BUILT";
            else if (!testArmKnown) reason = "NO_ACTIVE_V04450_UPPER_SELF_WEIGHT_TEST_PROVENANCE";
            else if (surface == null || !surface.Finite) reason = "V04410_SUPPORT_SCOPE_INVALID:" + (surface?.InvalidReason ?? "<none>");
            else if (bilateral != null && (bilateral.FailClosedEvents > 0 || bilateral.PrimaryDriveAborted || bilateral.SecondaryDriveAborted))
                reason = "PHYSICAL_RUN_SAFETY_NOT_CLEAN";
            else if (!frozenWorldGravityBoundary) reason = "PUBLIC_TORSO_OR_LEG_GRAVITY_BOUNDARY_NOT_FROZEN_OFF";
            else if (!zeroFrictionBaseline) reason = "V0480_ZERO_FRICTION_BASELINE_NOT_CLEAN";
            else if (!frozenTetherTopology) reason = "V0363_FOOT_TETHER_TOPOLOGY_CONTRACT_NOT_CLEAN";
            else if (!lowerGroundFrozen) reason = "V04140_B_LOWER_PLANAR_GROUND_BASELINE_NOT_CLEAN";
            else if (!predictiveTailFrozen) reason = "V04171_B_PREDICTIVE_TAIL_PROTECTION_NOT_FROZEN_CLEAN";
            else if (!footGravityFrozenOn) reason = "V04180_FOOT_NATIVE_GRAVITY_BASELINE_NOT_FROZEN_ON";
            else if (!lowerGravityFrozenOn) reason = "V04190_B_LOWER_NATIVE_GRAVITY_BASELINE_NOT_FROZEN_ON";
            else if (!reciprocalFrozenOn) reason = "V04200_B_RECIPROCAL_BASELINE_NOT_FROZEN_ON";
            else if (!transferFrozenOn) reason = "V04210_B_SELF_WEIGHT_TRANSFER_BASELINE_NOT_FROZEN_ON";
            else if (!upperLoadContract) reason = "V04450_UPPER_LOAD_SELECTOR_GRAVITY_OR_COMMAND_INVALID";
            else if (!upperLoadExposure) reason = "V04450_BILATERAL_LOAD_NONSTANCE_OR_UPPER_LEDGER_EXPOSURE_INSUFFICIENT";
            else if (!kneeReactionContract) reason = "V04430_KNEE_REACTION_SELECTOR_OR_COMMAND_INVALID";
            else if (!kneeReactionExposure) reason = "V04430_BILATERAL_REACTION_OR_NONSTANCE_EXPOSURE_INSUFFICIENT";
            else if (!surfaceContract) reason = "V04410_SUPPORT_SURFACE_CONTRACT_FAILED";
            else if (!surfaceExposure) reason = "V04410_BILATERAL_SUPPORT_EXPOSURE_INSUFFICIENT";
            else if (!contactOffsetContract) reason = "V04390_FOOT_CONTACT_OFFSET_SELECTOR_OR_READBACK_FAILED";
            else if ((leftFoot != null && leftFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves > 0) ||
                     (rightFoot != null && rightFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves > 0))
                reason = "V04390_SUPPORT_ENVIRONMENT_EXCLUDED_SOLVES";
            else if (!contactOffsetExposure) reason = "V04390_BILATERAL_CONTACT_OFFSET_EXPOSURE_INCOMPLETE_OR_INVALID";
            else if (!collisionSelectorContract) reason = "V04390_FROZEN_SPECULATIVE_CCD_CONTRACT_FAILED";
            else if (!collisionObservedExposure) reason = "V04390_FROZEN_SPECULATIVE_CCD_EXPOSURE_INSUFFICIENT_OR_MISMATCH";
            else if (!supportAngularContract) reason = "V04260_SELECTOR_OR_PASSIVITY_CONTRACT_FAILED";
            else if (!supportAngularExposure) reason = "V04260_BILATERAL_EXPOSURE_INSUFFICIENT";
            else if (!velocitySolverContract) reason = "V04270_CAPTURED_VELOCITY_OR_SOLVER_READBACK_FAILED";
            else if (!velocitySolverExposure) reason = "V04270_BILATERAL_LOADED_SOLVE_EXPOSURE_INSUFFICIENT";
            else if (!positionSolverContract) reason = "V04310_POSITION_SELECTOR_OR_READBACK_CONTRACT_FAILED";
            else if (!positionSolverExposure) reason = "V04310_BILATERAL_OBSERVED_POSITION_INTERVENTION_INSUFFICIENT";
            else if (!nativeGravityEnvironment) reason = "NATIVE_GRAVITY_VECTOR_OR_EXPECTED_LOAD_INVALID";
            else if (!bilateralExposure) reason = "BILATERAL_SUPPORT_LOAD_EXPOSURE_INSUFFICIENT";
            else if (!noTransferLeak) reason = "SELF_WEIGHT_TRANSFER_NONSTANCE_LEAK";
            else if (!noReciprocalLeak) reason = "V04200_RECIPROCAL_NONSTANCE_LEAK";
            else if (!noLowerGravityLeak) reason = "NATIVE_LOWER_GRAVITY_LEAK_OR_READBACK_FAILURE";
            else if (!lifecycleEvidence) reason = "STANCE_LIFECYCLE_EVIDENCE_NOT_CLEAN";
            else if (!supportLoadReactionDataset) reason = "SUPPORT_LOAD_REACTION_DATASET_NOT_USABLE";
            else if (!angularImpulseEvidenceReady) reason = "V04250_ANGULAR_IMPULSE_ATTRIBUTION_EVIDENCE_NOT_READY";
            else if (!couplingEvidenceReady) reason = "V04280_COUPLING_OBSERVER_EVIDENCE_NOT_READY";
            else if (!supportEntityReady) reason = "V04290_SUPPORT_ENTITY_BOUNDARY_EVIDENCE_NOT_READY";
            else if (!planarDampingContract) reason = "V04300_PLANAR_DAMPING_SELECTOR_CONTRACT_FAILED";
            else if (!planarDampingExposure) reason = "V04300_BILATERAL_LOADED_INTERVENTION_EXPOSURE_INSUFFICIENT";
            else if (!localRunSafety) reason = "PHYSICAL_RUN_SAFETY_NOT_CLEAN";
            else if (!normalLoadReady) reason = "V04330_DOWNSTREAM_NORMAL_LOAD_LEDGER_NOT_READY";
            else if (!normalWindowReady) reason = "V04350_DOWNSTREAM_NORMAL_WINDOW_LEDGER_NOT_READY";
            else if (_ankleTestArm == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB && !hipPelvisReady)
                reason = "V04460_HIP_PELVIS_INPUT_OR_RECEIVER_AUDIT_NOT_READY:" + (hipPelvis?.Issue ?? "EXPOSURE_INSUFFICIENT");
            else reason = hipPelvisReady ? "V04460_HIP_PELVIS_LOAD_PATH_DATASET_READY" : "V04450_UPPER_SELF_WEIGHT_DATASET_READY";

            float leftFootExpectedJ = leftFoot != null ? leftFoot.StanceFootNativeGravityExpectedNormalImpulseMean : 0f;
            float rightFootExpectedJ = rightFoot != null ? rightFoot.StanceFootNativeGravityExpectedNormalImpulseMean : 0f;
            float leftLowerExpectedJ = leftLeg != null ? leftLeg.StanceLowerNativeGravityExpectedNormalImpulseMean : 0f;
            float rightLowerExpectedJ = rightLeg != null ? rightLeg.StanceLowerNativeGravityExpectedNormalImpulseMean : 0f;
            float leftUpperExpectedJ = leftLeg?.UpperSelfWeightSupport?.AppliedImpulseMean ?? 0f;
            float rightUpperExpectedJ = rightLeg?.UpperSelfWeightSupport?.AppliedImpulseMean ?? 0f;
            float leftExpectedCombinedJ = leftFootExpectedJ + leftLowerExpectedJ + leftUpperExpectedJ;
            float rightExpectedCombinedJ = rightFootExpectedJ + rightLowerExpectedJ + rightUpperExpectedJ;
            float leftObservedJ = leftFoot != null ? leftFoot.StanceFootNativeGravityGravityOnSignedNormalImpulseMean : 0f;
            float rightObservedJ = rightFoot != null ? rightFoot.StanceFootNativeGravityGravityOnSignedNormalImpulseMean : 0f;
            float leftLoadRatio = leftExpectedCombinedJ > 0.0000001f ? leftObservedJ / leftExpectedCombinedJ : 0f;
            float rightLoadRatio = rightExpectedCombinedJ > 0.0000001f ? rightObservedJ / rightExpectedCombinedJ : 0f;

            string result =
                "ARL Stage-E joined normal load motion windows" +
                " | physicalMode=" + mode +
                " | testArm=" + _ankleTestArm +
                " | collisionArm=" + expectedCollisionArm +
                " | positionSolverBackground=B12_FROZEN" +
                " | velocitySolverContract=" + velocitySolverContract + "/exposure=" + velocitySolverExposure +
                " | V04310PositionSolverContract=" + positionSolverContract + "/observedExposure=" + positionSolverExposure +
                " | supportAngularContract=" + supportAngularContract + "/exposure=" + supportAngularExposure +
                " | V04210SelfWeightTransfer=FROZEN_B_ON" +
                " | V04200Reciprocal=FROZEN_B_ON" +
                " | footGravity=V04180_B_FROZEN_ON" +
                " | lowerGravity=V04190_B_FROZEN_ON" +
                " | publicTorsoLegGravityOff=" + frozenWorldGravityBoundary +
                " | zeroFrictionBaseline=" + zeroFrictionBaseline +
                " | V0363FootTetherTopologyContract=" + frozenTetherTopology +
                " | frozenV04140B=" + lowerGroundFrozen +
                " | predictiveTailFrozen=" + predictiveTailFrozen +
                " | footGravityFrozenOn=" + footGravityFrozenOn +
                " | lowerGravityFrozenOn=" + lowerGravityFrozenOn +
                " | reciprocalFrozenOn=" + reciprocalFrozenOn +
                " | transferFrozenOn=" + transferFrozenOn +
                " | supportSurface=" + (surface != null ? surface.Summary() : "<none>") +
                " | V04410SupportSurfaceContract=" + surfaceContract + "/observedExposure=" + surfaceExposure +
                " | contactOffsetBackground=FROZEN_20MM_BOTH_ARMS" +
                " | contactOffsetContract=" + contactOffsetContract +
                " | V04390ContactOffsetObservedExposure=" + contactOffsetExposure +
                "/resolved(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetResolvedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetResolvedSolves : 0) +
                "/mismatch(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetMismatchSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetMismatchSolves : 0) +
                "/entityChangedWithinSolve(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetBoundaryChangedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetBoundaryChangedSolves : 0) +
                "/readUnavailable(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetReadUnavailableSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetReadUnavailableSolves : 0) +
                "/environmentExcluded(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves : 0) +
                "/supportLostWithinSolve(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetSupportLostSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetSupportLostSolves : 0) +
                "/groundBaselines(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetGroundBaselines : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetGroundBaselines : 0) +
                "/groundSwitchesAcrossLoadedSolves(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetGroundSwitches : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetGroundSwitches : 0) +
                " | gravityReadbackScope=" + (completed ? "COMPLETED_REQUIRES_QUIESCED" : "LIVE_AUTHORIZED_STANCE_ALLOWED") +
                " | collisionSelectorContract=" + collisionSelectorContract +
                " | frozenSpeculativeCcdObservedExposure=" + collisionObservedExposure +
                "/resolved(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryCcdResolvedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryCcdResolvedSolves : 0) +
                "/mismatch(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryCcdMismatchSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryCcdMismatchSolves : 0) +
                " | nativeGravityEnvironment=" + nativeGravityEnvironment +
                " | bilateralExposure=" + bilateralExposure +
                " | noTransferLeak=" + noTransferLeak +
                " | noReciprocalLeak=" + noReciprocalLeak +
                " | noLowerGravityLeak=" + noLowerGravityLeak +
                " | lifecycleEvidence=" + lifecycleEvidence +
                " | supportLoadReactionDataset=" + supportLoadReactionDataset +
                " | V04250ObserverFinite=" + angularImpulseObserverFinite +
                " | FOOT_ANGULAR_IMPULSE_ATTRIBUTION_EVIDENCE_READY=" + angularImpulseEvidenceReady +
                " | localRunSafety=" + localRunSafety +
                " | V04280ObserverFinite=" + couplingObserverFinite + "/ready=" + couplingEvidenceReady +
                " | V04290ObserverFinite=" + supportEntityFinite + "/ready=" + supportEntityReady +
                " | V04300Contract=" + planarDampingContract + "/loadedExposure=" + planarDampingExposure +
                " | V04330AuditReady(left-right)=" + (leftFoot != null && leftFoot.DynamicNormalLoadEvidenceReady) + "-" + (rightFoot != null && rightFoot.DynamicNormalLoadEvidenceReady) +
                " | DYNAMIC_NORMAL_LOAD_DATASET_QUALIFIED=" + normalLoadReady +
                " | V04350WindowAuditReady(left-right)=" + (leftFoot != null && leftFoot.SupportNormalWindowEvidenceReady) + "-" + (rightFoot != null && rightFoot.SupportNormalWindowEvidenceReady) +
                " | NORMAL_LOAD_WINDOW_DATASET_QUALIFIED=" + normalWindowReady +
                " | dynamicLoadVerdict=REQUIRES_NORMAL_IMPULSE_LEDGER_REVIEW" +
                " | UPPER_SELF_WEIGHT_TRANSFER_DATASET_QUALIFIED=" + normalWindowReady +
                " | V04450UpperLoadContract=" + upperLoadContract + "/exposure=" + upperLoadExposure +
                " | HIP_PELVIS_LOAD_PATH_DATASET_QUALIFIED=" + hipPelvisReady +
                " | KNEE_SUPPORT_NORMAL_REACTION_DATASET_QUALIFIED=" + qualified +
                " | V04430ReactionContract=" + kneeReactionContract + "/exposure=" + kneeReactionExposure +
                " | MECHANISM_VERDICT=REQUIRES_UPPER_HIP_MOMENTUM_AND_SERIAL_LOAD_GROUND_LEDGER_REVIEW_NOT_WHOLE_BODY_SUPPORT_PASS" +
                " | reason=" + reason +
                "\n  left: stancePlanarDamping=" + Clean(leftFoot != null ? leftFoot.StancePlanarDampingSummary : null) + "/positionSolver=" + Clean(leftFoot != null ? leftFoot.FootPositionSolverSummary : null) + "/velocitySolver=" + Clean(leftFoot != null ? leftFoot.FootVelocitySolverSummary : null) + "/supportAngular=" + Clean(leftFoot != null ? leftFoot.StanceSupportAngularSummary : null) + "/collision=" + Clean(leftFoot != null ? leftFoot.FootCollisionDetectionModeSummary : null) +
                    "/contactOffset=" + Clean(leftFoot != null ? leftFoot.FootContactOffsetSummary : null) +
                    "/selfWeightTransfer=" + Clean(leftLeg != null ? leftLeg.StanceLowerSelfWeightTransferSummary : null) +
                    "/loadJ(observed-combinedExpected-ratio)=" + F(leftObservedJ) + "-" + F(leftExpectedCombinedJ) + "-" + F(leftLoadRatio) +
                    "/anchorsMm(knee-footPeak)=" + Fmm(leftLeg != null ? leftLeg.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters : 0f) + "-" + Fmm(leftFoot != null ? leftFoot.TraceObservedAnchorErrorMaxMeters : 0f) +
                    "/reaction=" + SupportLoadReactionDatasetSummary(leftFoot) +
                    "/angularImpulse=" + Clean(leftFoot != null ? leftFoot.FootAngularImpulseAttributionTraceSummary : null) +
                    "/coupling=" + Clean(leftFoot != null ? leftFoot.RockingTetherCouplingTraceSummary : null) +
                    "/supportEntity=" + Clean(leftFoot != null ? leftFoot.SupportEntityBoundaryTraceSummary : null) +
                "\n  right: stancePlanarDamping=" + Clean(rightFoot != null ? rightFoot.StancePlanarDampingSummary : null) + "/positionSolver=" + Clean(rightFoot != null ? rightFoot.FootPositionSolverSummary : null) + "/velocitySolver=" + Clean(rightFoot != null ? rightFoot.FootVelocitySolverSummary : null) + "/supportAngular=" + Clean(rightFoot != null ? rightFoot.StanceSupportAngularSummary : null) + "/collision=" + Clean(rightFoot != null ? rightFoot.FootCollisionDetectionModeSummary : null) +
                    "/contactOffset=" + Clean(rightFoot != null ? rightFoot.FootContactOffsetSummary : null) +
                    "/selfWeightTransfer=" + Clean(rightLeg != null ? rightLeg.StanceLowerSelfWeightTransferSummary : null) +
                    "/loadJ(observed-combinedExpected-ratio)=" + F(rightObservedJ) + "-" + F(rightExpectedCombinedJ) + "-" + F(rightLoadRatio) +
                    "/anchorsMm(knee-footPeak)=" + Fmm(rightLeg != null ? rightLeg.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters : 0f) + "-" + Fmm(rightFoot != null ? rightFoot.TraceObservedAnchorErrorMaxMeters : 0f) +
                    "/reaction=" + SupportLoadReactionDatasetSummary(rightFoot) +
                    "/angularImpulse=" + Clean(rightFoot != null ? rightFoot.FootAngularImpulseAttributionTraceSummary : null) +
                    "/coupling=" + Clean(rightFoot != null ? rightFoot.RockingTetherCouplingTraceSummary : null) +
                    "/supportEntity=" + Clean(rightFoot != null ? rightFoot.SupportEntityBoundaryTraceSummary : null) +
                "\n  V04430 leftKneeReaction=" + (leftLeg?.KneeSupportReaction?.GetSummary() ?? "<none>") +
                "\n  V04430 rightKneeReaction=" + (rightLeg?.KneeSupportReaction?.GetSummary() ?? "<none>") +
                "\n  V04450 leftUpperLoad=" + (leftLeg?.UpperSelfWeightSupport?.GetDetails() ?? "<none>") +
                "\n  V04450 rightUpperLoad=" + (rightLeg?.UpperSelfWeightSupport?.GetDetails() ?? "<none>") +
                "\n  V04460 hipPelvisLoadPath=" + (hipPelvis?.GetDetails() ?? "<none>") +
                "\n  V04460 canonical run is B on the unchanged V04450 loaded-leg plant. A remains the explicit legacy Upper-gravity-OFF comparison and does not qualify the new receiver audit. Hip/common inputs are read-only; no Pelvis force, native tensor, pose owner or gravity is changed. Input exposure requires >=50 loaded and >=30 released samples per side, >=50 IDLE and >=30 motion/source-turn samples; readiness also requires the bilateral downstream load/window datasets." +
                "\n  V04450 A=Upper weight OFF; B=Upper exact-STANCE native gravity plus explicit static Upper/Lower and Lower/Foot COM force pairs. Both use V04430 B reduced-mass normal PD and V04410 static 30cm Box. Torso gravity OFF. Original scene objects and the global collision matrix are not modified." +
                "\n  Both arms keep production torque, Foot contactOffset20mm, Speculative CCD, position12/captured velocity and frozen loads/gates. Exposure needs >=50 verified Foot-pair samples on each side plus actual callback and joined loaded-solve evidence. V04430 common B additionally needs >=50 eligible, >=30 nonzero candidate and >=30 nonstance samples per side. V04450 needs >=50 eligible and resolved loaded Upper ledger samples, >=30 nonstance, with A load writes=0/B load writes=eligible. Stop captures evidence before selector reset." +
                "\n  V04290 evidence: same authorized Ground object and cached ancestor path at pre-solve, first/last callback and N+1 before gravity clear/lifecycle; current probe is appended afterwards. Stable snapshots cannot exclude a transient change between samples, transient layer collision-matrix changes or an internal solver/CCD interruption. No automatic mechanism verdict.";
            return result;
        }

        private static bool FootContactOffsetObservedExposure(FootAnkleFoundationReport foot) => foot != null &&
            foot.SupportEntityBoundaryContactOffsetResolvedSolves >= FootSupportEntitySolveBoundaryObserver.MinimumResolvedSolves &&
            foot.SupportEntityBoundaryContactOffsetMismatchSolves == 0 &&
            foot.SupportEntityBoundaryContactOffsetBoundaryChangedSolves == 0 &&
            foot.SupportEntityBoundaryContactOffsetReadUnavailableSolves == 0 &&
            foot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves == 0;

        private static bool FootContactOffsetSelectorContract(FootAnkleFoundationReport foot, string arm)
        {
            if (foot == null || !foot.FootContactOffsetInitialized || !foot.FootContactOffsetFinite ||
                !foot.FootContactOffsetReadbackValid || foot.FootContactOffsetOwnerId == 0 ||
                !string.Equals(foot.FootContactOffsetArm, arm, StringComparison.Ordinal)) return false;
            bool widened = string.Equals(arm, FootContactOffsetIsolation.ArmB, StringComparison.Ordinal);
            bool baseline = string.Equals(arm, FootContactOffsetIsolation.ArmA, StringComparison.Ordinal) ||
                string.Equals(arm, FootContactOffsetIsolation.IdleArm, StringComparison.Ordinal);
            float expected = widened ? FootContactOffsetIsolation.BOffsetMeters : foot.FootContactOffsetBaselineMeters;
            return (widened || baseline) &&
                (!widened || foot.FootContactOffsetBaselineMeters < expected - FootContactOffsetIsolation.ReadbackToleranceMeters) &&
                FootContactOffsetIsolation.Matches(foot.FootContactOffsetSelectedMeters, expected) &&
                FootContactOffsetIsolation.Matches(foot.FootContactOffsetActualMeters, expected);
        }

        private static bool LowerGravityCurrentStateValid(SingleLegChainReport leg, FootAnkleFoundationReport foot, bool completed)
        {
            if (leg == null) return false;
            if (!leg.StanceLowerNativeGravityCurrentApplied) return true;
            // A completed audit has been quiesced. A live snapshot may legally show the current stance load.
            return !completed && foot != null && leg.StanceLowerNativeGravityEnabled && leg.StanceLowerNativeGravityFinite &&
                leg.FixedUpdateSequence > 0 && leg.StanceLowerNativeGravityCurrentScheduledFixed == leg.FixedUpdateSequence &&
                foot.FixedUpdateSequence == leg.FixedUpdateSequence && foot.ContactShadowCurrentContactActive &&
                foot.StanceContactLifecycleEnabled && foot.StanceContactLifecycleFinite &&
                foot.StanceContactLifecycleAuthorizedColliderInstanceId != 0 &&
                string.Equals(foot.GameNativeGaitPhaseState, "STANCE", StringComparison.Ordinal) &&
                foot.StanceFootNativeGravityCurrentApplied &&
                foot.StanceFootNativeGravityCurrentScheduledFixed == leg.FixedUpdateSequence;
        }

        private static bool FootCcdSelectorContract(FootAnkleFoundationReport foot, string arm)
        {
            return foot != null && foot.FootCollisionDetectionModeFinite &&
                FootCollisionDetectionModeIsolation.TryGetMode(arm, out CollisionDetectionMode expected) &&
                string.Equals(foot.FootCollisionDetectionModeArm, arm, StringComparison.Ordinal) &&
                string.Equals(foot.FootCollisionDetectionModeReadback, expected.ToString(), StringComparison.Ordinal);
        }

        private static bool FootVelocitySolverContract(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.FootVelocitySolverInitialized && foot.FootVelocitySolverFinite &&
                foot.FootVelocitySolverReadbackValid && foot.FootVelocitySolverReadbackFailures == 0 &&
                foot.FootVelocitySolverBaselineVelocity > 0 && foot.FootVelocitySolverBaselinePosition > 0 &&
                string.Equals(foot.FootVelocitySolverArm, FootSolverIterationBaselineContract.ArmA, StringComparison.Ordinal) &&
                foot.FootVelocitySolverSelectedVelocity == foot.FootVelocitySolverBaselineVelocity &&
                foot.FootVelocitySolverActualVelocity == foot.FootVelocitySolverBaselineVelocity &&
                foot.FootPositionSolverSelectedPosition > 0 &&
                foot.FootVelocitySolverActualPosition == (foot.FootVelocitySolverQuiesced
                    ? foot.FootVelocitySolverBaselinePosition : foot.FootPositionSolverSelectedPosition) &&
                (!foot.FootVelocitySolverQuiesced || foot.FootVelocitySolverBaselineRestored);
        }

        private static bool FootPositionSolverContract(FootAnkleFoundationReport foot)
        {
            return FootVelocitySolverContract(foot) &&
                string.Equals(foot.FootPositionSolverArm, FootSolverIterationBaselineContract.PositionArmB, StringComparison.Ordinal) &&
                foot.FootPositionSolverSelectedPosition == FootSolverIterationBaselineContract.BPositionIterations &&
                foot.FootPositionSolverSelectedPosition > foot.FootVelocitySolverBaselinePosition &&
                (foot.FootVelocitySolverQuiesced ? !foot.FootPositionSolverEnabled : foot.FootPositionSolverEnabled);
        }

        public static string GetFreeSpaceFoundationReadinessAudit()
        {
            const string mode = "FULL_FREE_SPACE_FOUNDATION_READINESS_AUDIT_V0379";
            TwoBodyProxyReport torso = _proxy != null ? _proxy.Report : null;
            BilateralLegReport bilateral = _legChain != null ? _legChain.BilateralReport : null;
            SingleLegChainReport primary = _legChain != null ? _legChain.PrimaryReport : null;
            SingleLegChainReport secondary = _legChain != null ? _legChain.SecondaryReport : null;
            FootAnkleFoundationReport leftFoot = _legChain != null ? _legChain.PrimaryFootReport : null;
            FootAnkleFoundationReport rightFoot = _legChain != null ? _legChain.SecondaryFootReport : null;

            bool built = torso != null && torso.Built && _legChain != null && _legChain.IsBuilt && bilateral != null && bilateral.Built;
            bool frozenStack = built && _legChain.FrozenProductionStackReady;
            bool footProduction = built && _legChain.FootProductionBaselineReady && _legChain.FootRuntimeContractReady &&
                                  string.Equals(_legChain.AnkleAngularTransportPolicy, FootAnkleAngularTransport.Production, StringComparison.Ordinal) &&
                                  string.Equals(_legChain.TranslationCaptureAuthorityArm, FootAnkleTranslationCaptureAuthorityIsolation.Production, StringComparison.Ordinal) &&
                                  _legChain.TranslationCaptureAuthorityArmsMatch;
            bool environmentFreeSpace = torso != null && !torso.GravityEnabled && !torso.CollidersEnabled &&
                                        FootFreeSpaceContract(leftFoot) && FootFreeSpaceContract(rightFoot);
            bool productionContract = built && frozenStack && footProduction && environmentFreeSpace;

            bool runEvidence = bilateral != null && bilateral.ClockSamples > 0 &&
                               bilateral.PrimaryTraceSamples > 0 && bilateral.SecondaryTraceSamples > 0 &&
                               primary != null && primary.TraceSamples > 0 && secondary != null && secondary.TraceSamples > 0 &&
                               leftFoot != null && leftFoot.TraceSamples > 0 && rightFoot != null && rightFoot.TraceSamples > 0;

            // v0.4.1 contact-shadow B intentionally changes the tested plant. Once either Foot has accumulated
            // active contact-shadow evidence, that trace must not later be re-labelled as a v0.3.79 free-space
            // certification merely because StopAnkleTest has already switched the colliders back OFF.
            bool runWasFreeSpace = runEvidence &&
                                   leftFoot.ContactShadowTraceFixedSamples == 0 && rightFoot.ContactShadowTraceFixedSamples == 0 &&
                                   leftFoot.ContactShadowTraceMatchedCallbacks == 0 && rightFoot.ContactShadowTraceMatchedCallbacks == 0 &&
                                   leftFoot.ContactShadowTraceUnexpectedCallbacks == 0 && rightFoot.ContactShadowTraceUnexpectedCallbacks == 0;

            bool legLocalSafety = runEvidence && primary.Finite && secondary.Finite &&
                                  primary.SingleLegFreezeProductionContractPass && secondary.SingleLegFreezeProductionContractPass &&
                                  primary.SingleLegFreezeCalibrationPass && secondary.SingleLegFreezeCalibrationPass &&
                                  primary.SingleLegFreezeInertiaPass && secondary.SingleLegFreezeInertiaPass &&
                                  primary.SingleLegFreezeRunSafetyPass && secondary.SingleLegFreezeRunSafetyPass &&
                                  !primary.DriveAborted && !secondary.DriveAborted;
            bool bilateralLocalSafety = runEvidence && bilateral.ClockParityPass && bilateral.FailClosedEvents == 0 &&
                                        !bilateral.PrimaryDriveAborted && !bilateral.SecondaryDriveAborted &&
                                        bilateral.FixedSequenceMismatchSamples == 0 && bilateral.RenderSequenceMismatchSamples == 0 &&
                                        bilateral.RenderFreshnessMismatchSamples == 0 && bilateral.RenderHeldDepthMismatchSamples == 0 &&
                                        bilateral.DriveStateMismatchSamples == 0;
            bool footLocalSafety = runEvidence && FootRunSafety(leftFoot) && FootRunSafety(rightFoot);
            bool torsoNoAbort = torso != null && torso.FiniteState && torso.FreezeValidationComplete && !torso.FreezeValidationTraceAborted;

            bool recoveryEvidenceHealthy = torsoNoAbort &&
                                           torso.FreezeRecoveryCompletedCount >= SourceAwareFreezeValidation.MinCompletedRecoverySessions &&
                                           torso.FreezeRecoveryPassedCount == torso.FreezeRecoveryCompletedCount &&
                                           torso.FreezeRecoveryFailedCount == 0 &&
                                           torso.FreezeRecoveryInvalidStateStepCount == 0 &&
                                           torso.FreezeRecoveryParentFeedforwardClampStepCount == 0 &&
                                           torso.FreezeRecoveryMaxUnder5DegreesSecondsFromSourceClose <= SourceAwareFreezeValidation.RecoveryLooseMaxSecondsFromSourceClose &&
                                           torso.FreezeRecoveryMaxUnder2DegreesSecondsFromSourceClose <= SourceAwareFreezeValidation.RecoveryStrictMaxSecondsFromSourceClose &&
                                           torso.FreezeRecoveryMaxAnchorErrorMeters <= SourceAwareFreezeValidation.RecoveryAnchorErrorMaxMeters;
            bool rapidTurnSurvivalHealthy = torsoNoAbort &&
                                            torso.FreezeSourceInvalidStateStepCount == 0 &&
                                            torso.FreezeSourceParentFeedforwardClampStepCount == 0 &&
                                            torso.FreezeSourceMaxAnchorErrorMeters <= SourceAwareFreezeValidation.SourceTurnMaxAnchorErrorMeters;
            bool legacyCoverageOnly = torsoNoAbort && !torso.TwoBodyFreezeReady &&
                                      string.Equals(torso.FreezeValidationReason, "INSUFFICIENT_SOURCE_TURN_DATA", StringComparison.Ordinal) &&
                                      torso.NormalTrackingPass && recoveryEvidenceHealthy && rapidTurnSurvivalHealthy &&
                                      torso.SourceTurnEventCount >= SourceAwareFreezeValidation.MinSourceTurnEvents &&
                                      torso.SourceTurnDiscontinuityEventCount < SourceAwareFreezeValidation.MinSourceDiscontinuityEvents &&
                                      torso.SourceTurnRapidContinuousEventCount > 0;
            bool torsoCurrentSafety = torsoNoAbort && (torso.TwoBodyFreezeReady || legacyCoverageOnly);
            bool currentRunSafety = runWasFreeSpace && torsoCurrentSafety && legLocalSafety && bilateralLocalSafety && footLocalSafety;

            bool supportPlaneReady = FootSupportPlaneReady(leftFoot) && FootSupportPlaneReady(rightFoot);
            bool contactProductionReady = leftFoot != null && rightFoot != null &&
                                          leftFoot.ContactGeometryAuthorized && rightFoot.ContactGeometryAuthorized &&
                                          leftFoot.ColliderEnabled && rightFoot.ColliderEnabled &&
                                          leftFoot.PhysicalGroundContactPossible && rightFoot.PhysicalGroundContactPossible &&
                                          torso != null && torso.GravityEnabled;

            // Stage-E entry means the free-space plant is sufficiently frozen to BEGIN a new contact/gravity
            // requalification. It deliberately does not mean current contact production is ready.
            bool stageEEntryReady = productionContract && currentRunSafety && supportPlaneReady && !contactProductionReady;

            string coverageClass;
            if (torso == null || !torso.FreezeValidationComplete) coverageClass = "NOT_FINALIZED";
            else if (torso.TwoBodyFreezeReady) coverageClass = "CURRENT_RUN_ALL_LEGACY_GATES_PASS";
            else if (legacyCoverageOnly) coverageClass = "LEGACY_SOURCE_DISCONTINUITY_COVERAGE_GAP_ONLY";
            else coverageClass = "LEGACY_GATE_PHYSICAL_OR_VALIDATION_FAIL:" + (torso.FreezeValidationReason ?? "<none>");

            string reason;
            if (!built) reason = "FOUNDATION_NOT_BUILT";
            else if (!productionContract) reason = "FROZEN_PRODUCTION_CONTRACT_MISMATCH";
            else if (!runEvidence) reason = "CURRENT_RUN_EVIDENCE_NOT_AVAILABLE";
            else if (!runWasFreeSpace) reason = "CURRENT_TRACE_IS_V0410_CONTACT_SHADOW_NOT_FREE_SPACE";
            else if (!currentRunSafety) reason = "CURRENT_RUN_SAFETY_OR_INTEGRATION_FAIL";
            else if (!supportPlaneReady) reason = "FOOT_SUPPORT_PLANE_NOT_READY_FOR_STAGE_E_REQUALIFICATION";
            else if (contactProductionReady) reason = "CONTACT_ALREADY_ACTIVE_OUTSIDE_V0379_FREE_SPACE_SCOPE";
            else reason = "READY_TO_BEGIN_STAGE_E_CONTACT_REQUALIFICATION";

            string result =
                "ARL full free-space foundation audit" +
                " | mode=" + mode +
                " | built=" + built +
                " | productionContract=" + productionContract +
                " | frozenStack=" + frozenStack +
                " | footProduction=" + footProduction +
                " | freeSpaceEnv=" + environmentFreeSpace +
                " | runEvidence=" + runEvidence +
                " | runWasFreeSpace=" + runWasFreeSpace +
                " | currentRunSafety=" + currentRunSafety +
                " | torsoSafety=" + torsoCurrentSafety +
                " | legLocalSafety=" + legLocalSafety +
                " | bilateralLocalSafety=" + bilateralLocalSafety +
                " | footLocalSafety=" + footLocalSafety +
                " | legacyCoverage=" + coverageClass +
                " | supportPlaneReady=" + supportPlaneReady +
                " | contactProductionReady=" + contactProductionReady +
                " | STAGE_E_ENTRY_READY=" + stageEEntryReady +
                " | reason=" + reason +
                "\n  torso: freezeReady=" + (torso != null && torso.TwoBodyFreezeReady) +
                "/normal=" + (torso != null && torso.NormalTrackingPass) +
                "/sourceLegacyGate=" + (torso != null && torso.SourceDiscontinuitySurvivalPass) +
                "/recoveryLegacyGate=" + (torso != null && torso.RecoveryPass) +
                "/rapidTurnSurvivalHealthy=" + rapidTurnSurvivalHealthy +
                "/recoveryEvidenceHealthy=" + recoveryEvidenceHealthy +
                "/events(total/discontinuity/rapid)=" + (torso != null ? torso.SourceTurnEventCount : 0) + "/" +
                    (torso != null ? torso.SourceTurnDiscontinuityEventCount : 0) + "/" + (torso != null ? torso.SourceTurnRapidContinuousEventCount : 0) +
                "\n  bilateral: n=" + (bilateral != null ? bilateral.ClockSamples : 0) +
                "/clockParity=" + (bilateral != null && bilateral.ClockParityPass) +
                "/failClosed=" + (bilateral != null ? bilateral.FailClosedEvents : 0) +
                "/aborted=" + (bilateral != null && bilateral.PrimaryDriveAborted) + "/" + (bilateral != null && bilateral.SecondaryDriveAborted) +
                "/kneeAnchorPeakMm=" + Fmm(bilateral != null ? bilateral.PrimaryPeakKneeAnchorMeters : 0f) + "/" + Fmm(bilateral != null ? bilateral.SecondaryPeakKneeAnchorMeters : 0f) +
                "\n  foot: traceN=" + (leftFoot != null ? leftFoot.TraceSamples : 0) + "/" + (rightFoot != null ? rightFoot.TraceSamples : 0) +
                "/trackMeanDeg=" + F(leftFoot != null ? leftFoot.TraceAnkleRelativeErrorMeanDegrees : 0f) + "/" + F(rightFoot != null ? rightFoot.TraceAnkleRelativeErrorMeanDegrees : 0f) +
                "/trackMaxDeg=" + F(leftFoot != null ? leftFoot.TraceAnkleRelativeErrorMaxDegrees : 0f) + "/" + F(rightFoot != null ? rightFoot.TraceAnkleRelativeErrorMaxDegrees : 0f) +
                "/anchorPeakMm=" + Fmm(leftFoot != null ? leftFoot.TraceObservedAnchorErrorMaxMeters : 0f) + "/" + Fmm(rightFoot != null ? rightFoot.TraceObservedAnchorErrorMaxMeters : 0f) +
                "\n  boundary: Gravity/Collider/contact/GSP remain outside current production; Stage-E entry authorizes only a new causal contact requalification, not direct contact enablement.";
            _logger?.LogWarning("[ARL][FREE_SPACE_FOUNDATION_AUDIT_V079] " + result);
            return result;
        }

        private static bool FootFreeSpaceContract(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.Built && foot.Finite && foot.MassPropertiesValidated &&
                   !foot.GravityEnabled && !foot.ColliderEnabled && foot.ColliderFrozenOff &&
                   !foot.AnkleJointExists && foot.AnkleDriveFinite && foot.AnkleConstraintFinite &&
                   foot.AnkleTransportFinite && foot.AnkleAngularTransportFinite &&
                   foot.TargetAccelerationConditionerFinite && foot.TranslationCaptureAuthorityFinite &&
                   foot.StanceContactFrictionIsolationFinite;
        }

        private static bool SupportLoadReactionDatasetUsable(FootAnkleFoundationReport report)
        {
            if (report == null || !report.MechanicalReactionFinite || !report.MechanicalReactionReadbackClean ||
                !report.MechanicalReactionSolverWitnessQualified)
                return false;

            if (report.MechanicalReactionTraceSupportNormalSamples < 5 ||
                report.MechanicalReactionTraceProjectionSamples < 5 ||
                report.MechanicalReactionTraceNonZeroImpulseSamples < 1 ||
                report.MechanicalReactionTraceSupportNormalValidRatio < 0.995f)
                return false;

            // v0.4.22 keeps strict support-normal/lifecycle fields visible; the support-load consumer still tolerates only the previously defined rare geometry-only animation transient
            // does not invalidate the load dataset when every accepted reaction is identity-clean and finite.
            // This support-load consumer rule is reused by frozen V04190 telemetry and V04200; it does not relax contact lifecycle or hard safety gates.
            int invalid = report.MechanicalReactionTraceSupportNormalInvalidSamples;
            bool invalidsAreGeometryOnly = invalid == report.MechanicalReactionTraceSupportInvalidGeometryNotReady &&
                                           report.MechanicalReactionTraceSupportInvalidAuthorizedIdUnavailable == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidSupportIdUnavailable == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidIdMismatch == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidFootprintNotSameSurface == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidNormalNonFiniteOrZero == 0 &&
                                           report.MechanicalReactionTraceSupportColliderMismatchSamples == 0 &&
                                           report.MechanicalReactionTraceReadFailures == 0 &&
                                           report.MechanicalReactionTraceImpulseReadFailures == 0 &&
                                           report.MechanicalReactionTraceRelativeVelocityReadFailures == 0 &&
                                           report.MechanicalReactionTracePostBodyReadFailures == 0;
            return invalidsAreGeometryOnly;
        }

        private static string SupportLoadReactionDatasetSummary(FootAnkleFoundationReport report)
        {
            if (report == null) return "<none>";
            return "usable=" + SupportLoadReactionDatasetUsable(report) +
                   "/strictDynamic=" + DynamicStanceReactionEvidence(report) +
                   "/strictSupportN=" + report.MechanicalReactionSupportNormalWitnessQualified +
                   "/validRatio=" + F(report.MechanicalReactionTraceSupportNormalValidRatio) +
                   "/invalid=" + report.MechanicalReactionTraceSupportNormalInvalidSamples +
                   "/geometryInvalid=" + report.MechanicalReactionTraceSupportInvalidGeometryNotReady +
                   "/maxStreak=" + report.MechanicalReactionTraceSupportInvalidMaxStreak +
                   "/idMismatch=" + report.MechanicalReactionTraceSupportInvalidIdMismatch +
                   "/readFail=" + report.MechanicalReactionTraceReadFailures;
        }

        private static bool DynamicStanceReactionEvidence(FootAnkleFoundationReport report)
        {
            if (report == null || !report.MechanicalReactionFinite || !report.MechanicalReactionReadbackClean ||
                !report.MechanicalReactionSolverWitnessQualified)
                return false;

            // Preserve the historical v0.4.3 strict zero-invalid verdict exactly. Dynamic stance consumers
            // may additionally accept isolated one-FixedUpdate geometry-not-ready boundary samples from
            // repeated contact enter/exit, but never ID mismatches, missing authorization, non-finite normals
            // or multi-frame invalid streaks. This is a consumer contract, not a rewrite of v0.4.3 history.
            if (report.MechanicalReactionSupportNormalWitnessQualified)
                return true;

            int invalid = report.MechanicalReactionTraceSupportNormalInvalidSamples;
            bool onlyIsolatedGeometryBoundary =
                invalid > 0 &&
                invalid == report.MechanicalReactionTraceSupportInvalidGeometryNotReady &&
                report.MechanicalReactionTraceSupportInvalidAuthorizedIdUnavailable == 0 &&
                report.MechanicalReactionTraceSupportInvalidSupportIdUnavailable == 0 &&
                report.MechanicalReactionTraceSupportInvalidIdMismatch == 0 &&
                report.MechanicalReactionTraceSupportInvalidFootprintNotSameSurface == 0 &&
                report.MechanicalReactionTraceSupportInvalidNormalNonFiniteOrZero == 0 &&
                report.MechanicalReactionTraceSupportColliderMismatchSamples == 0 &&
                report.MechanicalReactionTraceSupportInvalidMaxStreak <= 1;

            return onlyIsolatedGeometryBoundary &&
                   report.MechanicalReactionTraceSupportNormalSamples >= 5 &&
                   report.MechanicalReactionTraceProjectionSamples >= 5 &&
                   report.MechanicalReactionTraceNonZeroImpulseSamples >= 1 &&
                   report.MechanicalReactionTraceSupportNormalValidRatio >= 0.995f;
        }

        private static bool FootRunSafety(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.Built && foot.Finite && foot.TraceSamples > 0 &&
                   foot.TraceAnkleAnchorErrorMaxMeters <= 0.030001f &&
                   foot.TraceObservedFixedSamples > 0 && foot.TraceObservedAnchorErrorMaxMeters <= 0.030001f &&
                   foot.FootCollisionDetectionModeFinite && foot.AnkleDriveFinite && foot.AnkleConstraintFinite && foot.AnkleTransportFinite &&
                   foot.AnkleAngularTransportFinite && foot.TargetAccelerationConditionerFinite &&
                   foot.TranslationCaptureAuthorityFinite && foot.TranslationPredictiveTailCaptureFinite && foot.StanceContactLifecycleFinite &&
                   foot.StanceContactFrictionIsolationFinite && foot.StanceLowerGroundAuthorityFinite &&
                   foot.StanceFootNativeGravityFinite;
        }

        private static bool FootSupportPlaneReady(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.Built && foot.SupportPlaneCalibrationApplied &&
                   foot.SupportPlaneCalibrationHitCount >= 3 && foot.SupportPlaneCalibrationNormalDot >= 0.65f;
        }

    }
}
