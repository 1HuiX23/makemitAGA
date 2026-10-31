using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralLegCoordinator
    {
        internal bool DynamicContactSelected => (_primary != null && _primary.DynamicContactSelected) ||
            (_secondary != null && _secondary.DynamicContactSelected);
        internal DynamicContactBoundaryIssue DynamicContactIssue
        {
            get
            {
                if (!IsBuilt || !_bilateralEnabled || _primary == null || _secondary == null)
                    return DynamicContactBoundaryIssue.MissingFoundation;
                if (!_primary.DynamicContactSelected || !_secondary.DynamicContactSelected || _primary.DynamicScope != _secondary.DynamicScope)
                    return DynamicContactBoundaryIssue.Scope;
                if (!_supportSurface.Active || !_supportSurface.Finite || _supportSurface.Arm != FootSupportSurfaceIsolation.ArmB)
                    return DynamicContactBoundaryIssue.SupportSurface;
                if (!_contactShadowSelfPairReadback || _contactShadowSelfPairPrimary != _primary.FootContactShadowCollider ||
                    _contactShadowSelfPairSecondary != _secondary.FootContactShadowCollider)
                    return DynamicContactBoundaryIssue.FootSelfPair;
                DynamicContactBoundaryIssue primary = _primary.DynamicContactIssue;
                return primary != DynamicContactBoundaryIssue.None ? primary : _secondary.DynamicContactIssue;
            }
        }
        internal bool DynamicContactReady => DynamicContactIssue == DynamicContactBoundaryIssue.None;
        internal bool DynamicContactMatchesScope(PelvisRunScope scope) => PelvisRunScopes.IsContact(scope) &&
            DynamicContactReady && _primary.DynamicScope == scope && _secondary.DynamicScope == scope;

        internal string PrepareDynamicContact(PelvisRunScope scope)
        {
            if (!PelvisRunScopes.IsContact(scope) || !DynamicParentsActive || !FreeSpaceReady || TraceEnabled)
                throw new InvalidOperationException("DYNAMIC_CONTACT_REQUIRES_JOINED_FREE_SPACE_AND_TRACE_OFF");
            // Synchronous, after bilateral continuous join and before formal trace/next solve.
            // No collider is enabled by this setup; stance owns later OFF->ON transitions.
            if (PelvisRunScopes.HasKneeCoupling(scope))
            {
                _primary.PrepareDynamicKneeCoupling(scope);
                _secondary.PrepareDynamicKneeCoupling(scope);
            }
            SetTranslationPredictiveTailCapture(false);
            SetStanceContactFrictionIsolation(true);
            SetFootCollisionDetectionModeArm(FootCollisionDetectionModeIsolation.ActiveArm);
            SetFootContactOffsetArm(FootContactOffsetIsolation.ArmB);
            RestoreFootSolverIterationBaseline();
            SetFootPositionSolverArm(FootSolverIterationBaselineContract.PositionArmB);
            string support = ConfigureFootSupportSurface(FootSupportSurfaceIsolation.ArmB);
            if (!_supportSurface.Active || !_supportSurface.Finite || _supportSurface.Arm != FootSupportSurfaceIsolation.ArmB)
                throw new InvalidOperationException("DYNAMIC_CONTACT_SUPPORT_SETUP_FAILED:" + support);
            SetStanceOnlyContactQualification(true);
            // Select the existing gravity owner while native gravity is still OFF; only its post-controller
            // scheduler may enable Foot gravity for a subsequent exact-authorized stance solve.
            SetStanceFootNativeGravityLoad(PelvisRunScopes.HasFootWeight(scope));
            bool lowerWeight = PelvisRunScopes.HasLowerWeight(scope);
            SetStanceLowerSelfWeightSupportTransfer(lowerWeight);
            SetStanceLowerNativeGravityLoad(lowerWeight);
            // Each child checks all native/selector prerequisites before accepting the new scope.
            _primary.SelectDynamicContactScope(scope);
            _secondary.SelectDynamicContactScope(scope);
            if (!DynamicContactMatchesScope(scope)) throw new InvalidOperationException("DYNAMIC_CONTACT_SETUP_READBACK_FAILED:" + DynamicContactIssue);
            return DynamicContactBoundarySummary();
        }

        internal string DynamicContactBoundarySummary() => "dynamicContactBoundary=V04870/selected=" + DynamicContactSelected +
            "/scope(primary-secondary)=" + (_primary?.DynamicScope.ToString() ?? "MISSING") + ":" + (_secondary?.DynamicScope.ToString() ?? "MISSING") +
            "/issue=" + DynamicContactIssue + "/primary=" + (_primary?.DynamicContactIssue ?? DynamicContactBoundaryIssue.MissingFoundation) +
            "/secondary=" + (_secondary?.DynamicContactIssue ?? DynamicContactBoundaryIssue.MissingFoundation) +
            "/support=" + _supportSurface.Arm + "/supportId=" + _supportSurface.SelectedId +
            "/footPairIgnored=" + _contactShadowSelfPairReadback +
            "/predictiveCapture=" + TranslationPredictiveTailCaptureArm +
            "/requiredContact=EXACT_STANCE/requiredZeroFriction=True/requiredCCD=SPECULATIVE/requiredContactOffsetMm=20/requiredPositionIterations=12/requiredVelocityIterations=CAPTURED" +
            "/requiredFootGravity=" + (_primary != null && PelvisRunScopes.HasFootWeight(_primary.DynamicScope) ? "EXACT_STANCE_NATIVE" : "OFF") +
            "/requiredLowerWeight=" + (_primary != null && PelvisRunScopes.HasLowerWeight(_primary.DynamicScope) ? "NATIVE_GRAVITY_PLUS_NORMAL_FORCE_PAIR" : "OFF") +
            "/requiredKneeReaction=" + (_primary != null && PelvisRunScopes.HasKneeCoupling(_primary.DynamicScope) ? "REDUCED_MASS_NORMAL_PD_PAIR" : "OFF") +
            "/requiredUpperGravity=" + (_primary != null && PelvisRunScopes.HasUpperWeight(_primary.DynamicScope) ? "EXACT_STANCE_SERIAL_SELF_WEIGHT" : "OFF") +
            "/requiredTorsoGravity=OFF/requiredReciprocal=OFF/hipReaction=" + (_primary != null && PelvisRunScopes.HasHipCoupling(_primary.DynamicScope) ? "TRANSLATION_AT_HIP_ANCHORS" : "NOT_ROUTED") + "/hipAngularControllerReaction=OFF/loadedSupportCertified=False";

        // Read-only qualification of THIS scope. Historical kinematic Pelvis load-path readiness is not reused.
        internal string DynamicContactEvidence(bool runReady, PelvisRunScope scope)
        {
            bool surface = _supportSurface.ExposureReady;
            bool left = ContactEvidenceReady(PrimaryFootReport), right = ContactEvidenceReady(SecondaryFootReport);
            bool footSelfWeight = PelvisRunScopes.HasFootWeight(scope);
            bool lowerWeight = PelvisRunScopes.HasLowerWeight(scope);
            bool kneeCoupling = PelvisRunScopes.HasKneeCoupling(scope);
            bool gravity = footSelfWeight ? FootSelfWeightEvidenceReady(PrimaryFootReport) && FootSelfWeightEvidenceReady(SecondaryFootReport)
                : FootUnloadedEvidenceReady(PrimaryFootReport) && FootUnloadedEvidenceReady(SecondaryFootReport);
            bool lower = !lowerWeight || (LowerSelfWeightEvidenceReady(PrimaryReport, PrimaryFootReport) &&
                LowerSelfWeightEvidenceReady(SecondaryReport, SecondaryFootReport));
            bool knee = !kneeCoupling || (KneeCouplingEvidenceReady(PrimaryReport, PrimaryFootReport) &&
                KneeCouplingEvidenceReady(SecondaryReport, SecondaryFootReport));
            bool upperWeight = PelvisRunScopes.HasUpperWeight(scope);
            bool upper = !upperWeight || (UpperWeightEvidenceReady(PrimaryReport, PrimaryFootReport) && UpperWeightEvidenceReady(SecondaryReport, SecondaryFootReport));
            bool hip = !PelvisRunScopes.HasHipCoupling(scope) || (_primary.HipAnchorPair.EvidenceReady && _secondary.HipAnchorPair.EvidenceReady &&
                _primary.HipAnchorPair.Samples == PrimaryReport.TraceSamples && _secondary.HipAnchorPair.Samples == SecondaryReport.TraceSamples);
            bool ready = hip && upper && runReady && DynamicContactMatchesScope(scope) && surface && left && right && gravity && lower && knee;
            return PelvisRunScopes.Dataset(scope) + "=" + ready +
                " | runtimeAndSafetyReady=" + runReady + " | supportExposureReady=" + surface +
                " | bilateralContactEvidence(primary-secondary)=" + left + ":" + right +
                " | gravityScopeEvidenceReady=" + gravity + " | footSelfWeightEvidenceReady=" + (footSelfWeight && ready) +
                " | lowerSelfWeightEvidenceReady=" + (lowerWeight && ready) +
                " | hipAnchorTranslationPairEvidenceReady=" + (PelvisRunScopes.HasHipCoupling(scope) && ready) +
                " | dynamicHipWrenchInputEvidenceReady=" + (upperWeight && ready && _hipPelvisLoadPath.Report.DynamicMode && _hipPelvisLoadPath.Report.EvidenceReady) +
                " | upperSelfWeightEvidenceReady=" + (upperWeight && ready) +
                " | upperLoadLedgerEvidenceReady=" + (upperWeight && ready && PrimaryReport.UpperSelfWeightSupport.LedgerFinite && SecondaryReport.UpperSelfWeightSupport.LedgerFinite) +
                " | kneeNormalCouplingEvidenceReady=" + (kneeCoupling && ready) +
                " | lowerLoadLedgerEvidenceReady=" + (lowerWeight && ready && LowerLedgerEvidenceReady(PrimaryFootReport) && LowerLedgerEvidenceReady(SecondaryFootReport)) +
                " | loadedSupportCertified=False | wholeBodyBalanceCertified=False\n" + DynamicContactBoundarySummary() +
                "\n" + _supportSurface.Summary() + "\n" + ContactEvidenceSummary("PRIMARY", PrimaryFootReport) +
                "\n" + ContactEvidenceSummary("SECONDARY", SecondaryFootReport);
        }

        internal string HipAnchorPairDetails() => "[PRIMARY_HIP_PAIR]\n" + (_primary?.HipAnchorPair.GetDetails() ?? "<missing>") +
            "\n" + (_primary?.HipPointResponseDetails ?? "<missing>") +
            "\n[SECONDARY_HIP_PAIR]\n" + (_secondary?.HipAnchorPair.GetDetails() ?? "<missing>") +
            "\n" + (_secondary?.HipPointResponseDetails ?? "<missing>");

        private static bool UpperWeightEvidenceReady(SingleLegChainReport leg, FootAnkleFoundationReport foot)
        {
            UpperSelfWeightSupportReport r = leg?.UpperSelfWeightSupport;
            return r != null && foot != null && UpperSelfWeightSupportNormalLoadTransferIsolation.ExposureReady(r, UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB) &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.DynamicCommandMatches(r, leg, foot, leg.FixedUpdateSequence, leg.StanceLowerNativeGravityCurrentApplied) &&
                r.Samples == foot.StanceFootNativeGravityTraceSamples && r.Eligible == leg.StanceLowerNativeGravityEligibleSamples &&
                r.NonStance == r.Samples - r.Eligible && r.LedgerCaptured == r.Samples &&
                r.LedgerResolved + r.LedgerCensored == r.LedgerCaptured;
        }

        private static bool LowerLedgerEvidenceReady(FootAnkleFoundationReport foot) => foot != null &&
            foot.DynamicNormalLoadFinite && foot.DynamicNormalLoadEvidenceReady &&
            foot.SupportNormalWindowFinite && foot.SupportNormalWindowEvidenceReady;

        private static bool KneeCouplingEvidenceReady(SingleLegChainReport leg, FootAnkleFoundationReport foot)
        {
            KneeSupportNormalReactionReport knee = leg?.KneeSupportReaction;
            return knee != null && foot != null && KneeSupportNormalReactionIsolation.ExposureReady(knee, KneeSupportNormalReactionIsolation.ArmB) &&
                knee.Samples == foot.StanceFootNativeGravityTraceSamples &&
                knee.Eligible == leg.StanceLowerNativeGravityEligibleSamples && knee.NonStance == knee.Samples - knee.Eligible &&
                knee.CurrentFixed == leg.FixedUpdateSequence && knee.CurrentSourceFixed == knee.CurrentFixed &&
                knee.CurrentApplied == knee.CurrentEligible && knee.CurrentNormalConstraint == knee.CurrentEligible &&
                !float.IsNaN(knee.PeakPairClosureNewtons) && !float.IsInfinity(knee.PeakPairClosureNewtons) &&
                knee.PeakPairClosureNewtons >= 0f && knee.PeakPairClosureNewtons <= 0.01f;
        }

        private static bool LowerSelfWeightEvidenceReady(SingleLegChainReport r, FootAnkleFoundationReport foot) => r != null && foot != null &&
            r.StanceLowerNativeGravityArm == LowerStanceNativeGravitySupportLoadIsolation.ArmB && r.StanceLowerNativeGravityFinite &&
            r.StanceLowerSelfWeightTransferArm == LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB && r.StanceLowerSelfWeightTransferFinite &&
            r.StanceLowerNativeGravityTraceSamples == foot.StanceFootNativeGravityTraceSamples &&
            r.StanceLowerSelfWeightTransferTraceSamples == r.StanceLowerNativeGravityTraceSamples &&
            r.StanceLowerNativeGravityEligibleSamples >= LowerStanceNativeGravitySupportLoadIsolation.MinEligibleSamples &&
            r.StanceLowerNativeGravityAppliedSamples >= LowerStanceNativeGravitySupportLoadIsolation.MinAppliedSamples &&
            r.StanceLowerNativeGravityAppliedSamples == r.StanceLowerNativeGravityEligibleSamples &&
            r.StanceLowerNativeGravityAppliedSamples == foot.StanceFootNativeGravityAppliedSamples &&
            r.StanceLowerSelfWeightTransferEligibleSamples == r.StanceLowerNativeGravityEligibleSamples &&
            r.StanceLowerSelfWeightTransferAppliedSamples == r.StanceLowerNativeGravityAppliedSamples &&
            r.StanceLowerNativeGravityNonStanceAppliedSamples == 0 && r.StanceLowerNativeGravityReadbackFailures == 0 &&
            r.StanceLowerSelfWeightTransferNonStanceAppliedSamples == 0 &&
            NormalLoadMatches(r.StanceLowerNativeGravityExpectedNormalForceMean, r.StanceLowerSelfWeightTransferExpectedForceMean) &&
            NormalLoadMatches(r.StanceLowerNativeGravityExpectedNormalImpulseMean, r.StanceLowerSelfWeightTransferExpectedImpulseMean);
        private static bool NormalLoadMatches(float nativeWeight, float transferredWeight) => nativeWeight > 0f && transferredWeight > 0f &&
            !float.IsInfinity(nativeWeight) && !float.IsInfinity(transferredWeight) &&
            UnityEngine.Mathf.Abs(nativeWeight - transferredWeight) <= UnityEngine.Mathf.Max(0.0001f, nativeWeight * 0.0001f);
        // Foot-only load evidence is independent of the retired kinematic-Pelvis load-path certificate.
        // Callback counts are events, not unique Fixed steps or a whole-body force-closure measurement.
        private static bool FootSelfWeightEvidenceReady(FootAnkleFoundationReport r) => r != null &&
            r.StanceFootNativeGravityArm == FootStanceNativeGravitySupportLoadIsolation.ArmB && r.StanceFootNativeGravityFinite &&
            r.StanceFootNativeGravityEligibleSamples >= FootStanceNativeGravitySupportLoadIsolation.MinEligibleSamples &&
            r.StanceFootNativeGravityAppliedSamples >= FootStanceNativeGravitySupportLoadIsolation.MinAppliedSamples &&
            r.StanceFootNativeGravityAppliedSamples == r.StanceFootNativeGravityEligibleSamples &&
            r.StanceFootNativeGravityNonStanceAppliedSamples == 0 && r.StanceFootNativeGravityReadbackFailures == 0 &&
            r.StanceFootNativeGravityCallbackGravityOnSamples >= FootStanceNativeGravitySupportLoadIsolation.MinReactionCallbackSamples &&
            r.StanceFootNativeGravityGravityOnSignedNormalImpulseMean > 0f && !float.IsInfinity(r.StanceFootNativeGravityGravityOnSignedNormalImpulseMean) &&
            r.StanceFootNativeGravityExpectedNormalForceMean > 0f && !float.IsInfinity(r.StanceFootNativeGravityExpectedNormalForceMean) &&
            r.StanceFootNativeGravityExpectedNormalImpulseMean > 0f && !float.IsInfinity(r.StanceFootNativeGravityExpectedNormalImpulseMean);
        private static bool FootUnloadedEvidenceReady(FootAnkleFoundationReport r) => r != null &&
            r.StanceFootNativeGravityArm == FootStanceNativeGravitySupportLoadIsolation.ArmA && r.StanceFootNativeGravityFinite &&
            r.StanceFootNativeGravityAppliedSamples == 0 && r.StanceFootNativeGravityCallbackGravityOnSamples == 0 &&
            r.StanceFootNativeGravityNonStanceAppliedSamples == 0 && r.StanceFootNativeGravityReadbackFailures == 0;
        private static bool ContactEvidenceReady(FootAnkleFoundationReport r) => r != null && r.Finite &&
            r.ContactShadowFinite && r.ContactShadowTraceQualified && r.StanceContactLifecycleFinite &&
            r.StanceContactLifecycleTraceQualified && r.MechanicalReactionFinite && r.MechanicalReactionTraceQualified &&
            r.ContactShadowIsolationFailures == 0 && r.ContactShadowRestoreFailures == 0 &&
            r.ContactShadowSourceIsolationReadbackFailures == 0 && r.ContactShadowTraceUnexpectedCallbacks == 0;
        private static string ContactEvidenceSummary(string side, FootAnkleFoundationReport r) => r == null ? side + "/contact=MISSING" :
            side + "/contactEvidence=V04830/stance(active-activate-release)=" + r.StanceContactLifecycleActiveSamples + "-" +
            r.StanceContactLifecycleContactActivations + "-" + r.StanceContactLifecycleReleaseCount +
            "/callback(matched-unexpected)=" + r.ContactShadowTraceMatchedCallbacks + "-" + r.ContactShadowTraceUnexpectedCallbacks +
            "/mechanical(impulseRead-response-nonzero)=" + r.MechanicalReactionTraceImpulseReadSamples + "-" +
            r.MechanicalReactionTraceResponseSamples + "-" + r.MechanicalReactionTraceNonZeroImpulseSamples +
            "/qualified(identity-lifecycle-mechanical)=" + r.ContactShadowTraceQualified + "-" +
            r.StanceContactLifecycleTraceQualified + "-" + r.MechanicalReactionTraceQualified;

        internal bool RestoreDynamicContact(StringBuilder evidence)
        {
            // Partial setup and exceptional half-open states use the same independent cleanup sequence.
            // No solve can occur between these calls. A failure remains visible and invokes the runtime full rollback.
            bool ok = ContactCleanup("CONTACT_LEG_DRIVE_OFF", () => { SetDrive(false); return NativeBodiesSafeOff; }, evidence);
            ok &= ContactCleanup("CONTACT_PRIMARY_SELECTORS", () => _primary != null && _primary.RestoreDynamicContactSelectors(evidence), evidence);
            ok &= ContactCleanup("CONTACT_SECONDARY_SELECTORS", () => _secondary != null && _secondary.RestoreDynamicContactSelectors(evidence), evidence);
            ok &= ContactCleanup("CONTACT_FOOT_PAIR_RESTORE", () => RestoreBilateralFootSelfCollisionIsolation() == 0, evidence);
            ok &= ContactCleanup("CONTACT_SUPPORT_RELEASE", () => { StopFootSupportSurface(); return _supportSurface.StoppedSafe; }, evidence);
            bool ready = ok && !DynamicContactSelected && NativeBodiesSafeOff && FreeSpaceReady;
            evidence.AppendLine("dynamicContactBaselineRestored=" + ready + " | " + FreeSpaceBoundarySummary());
            return ready;
        }
        private static bool ContactCleanup(string name, Func<bool> operation, StringBuilder evidence)
        {
            try { bool ok = operation(); evidence.AppendLine(name + "=" + ok); return ok; }
            catch (Exception e) { evidence.AppendLine(name + "=False/exception=" + e.GetType().Name); return false; }
        }
    }
}
