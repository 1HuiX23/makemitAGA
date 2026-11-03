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
                bool continuousEntry = PelvisRunScopes.HasHipCoupling(_primary.DynamicScope);
                bool recoveryEntry = continuousEntry;
                bool partialGround = continuousEntry;
                bool coherentKnee = continuousEntry;
                bool plannedKneeFeedforward = continuousEntry;
                bool loadAdmission = continuousEntry && _torso != null &&
                    _torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmB;
                if (_primary.ContactEntryContinuous != continuousEntry || _secondary.ContactEntryContinuous != continuousEntry ||
                    _primary.ContactEntryRecovery != recoveryEntry || _secondary.ContactEntryRecovery != recoveryEntry ||
                    _primary.StanceLoadAdmissionSelected != loadAdmission || _secondary.StanceLoadAdmissionSelected != loadAdmission ||
                    _primary.GroundReferenceEnabled != continuousEntry || _secondary.GroundReferenceEnabled != continuousEntry ||
                    _primary.GroundReferenceApplied != continuousEntry || _secondary.GroundReferenceApplied != continuousEntry ||
                    _primary.GroundReferencePartial != partialGround || _secondary.GroundReferencePartial != partialGround ||
                    _primary.KneeReferenceSelected != coherentKnee || _secondary.KneeReferenceSelected != coherentKnee ||
                    _primary.KneePlannedFeedforwardSelected != plannedKneeFeedforward ||
                    _secondary.KneePlannedFeedforwardSelected != plannedKneeFeedforward)
                    return DynamicContactBoundaryIssue.ContactLifecycle;
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
            bool continuousEntry = PelvisRunScopes.HasHipCoupling(scope);
            bool recoveryEntry = continuousEntry;
            bool partialGround = continuousEntry;
            bool loadAdmission = continuousEntry && _torso != null &&
                _torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmB;
            _primary.SelectContactEntryContinuous(continuousEntry, recoveryEntry, continuousEntry, partialGround, loadAdmission);
            _secondary.SelectContactEntryContinuous(continuousEntry, recoveryEntry, continuousEntry, partialGround, loadAdmission);
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
            "/kneeReference(primary-secondary)=" + (_primary?.KneeReferenceSelected ?? false) + ":" + (_secondary?.KneeReferenceSelected ?? false) +
            "/kneePlannedFF(primary-secondary)=" + (_primary?.KneePlannedFeedforwardSelected ?? false) + ":" + (_secondary?.KneePlannedFeedforwardSelected ?? false) +
            "/loadAdmission(primary-secondary)=" + (_primary?.StanceLoadAdmissionSelected ?? false) + ":" + (_secondary?.StanceLoadAdmissionSelected ?? false) +
            "/scope(primary-secondary)=" + (_primary?.DynamicScope.ToString() ?? "MISSING") + ":" + (_secondary?.DynamicScope.ToString() ?? "MISSING") +
            "/issue=" + DynamicContactIssue + "/primary=" + (_primary?.DynamicContactIssue ?? DynamicContactBoundaryIssue.MissingFoundation) +
            "/secondary=" + (_secondary?.DynamicContactIssue ?? DynamicContactBoundaryIssue.MissingFoundation) +
            "/support=" + _supportSurface.Arm + "/supportId=" + _supportSurface.SelectedId +
            "/footPairIgnored=" + _contactShadowSelfPairReadback +
            "/predictiveCapture=" + TranslationPredictiveTailCaptureArm +
            "/entryPolicy=" + (_primary != null && _primary.ContactEntryContinuous ? (_primary.ContactEntryRecovery ? "SHALLOW_RECOVERY" : "STRICT_BOX") : "LEGACY_SERIAL") +
            "/groundReference=" + (_primary != null && _primary.GroundReferenceEnabled ? (_primary.GroundReferenceApplied ? (_primary.GroundReferencePartial ? "FEASIBLE_OR_PARTIAL_RENDER_GOAL" : "FEASIBLE_RENDER_GOAL") : "OBSERVE_ONLY") : "OFF") +
            "/requiredContact=EXACT_STANCE/requiredZeroFriction=True/requiredCCD=SPECULATIVE/requiredContactOffsetMm=20/requiredPositionIterations=12/requiredVelocityIterations=CAPTURED" +
            "/requiredFootGravity=" + (_primary != null && PelvisRunScopes.HasFootWeight(_primary.DynamicScope) ? "EXACT_STANCE_NATIVE" : "OFF") +
            "/requiredLowerWeight=" + (_primary != null && PelvisRunScopes.HasLowerWeight(_primary.DynamicScope) ? "NATIVE_GRAVITY_PLUS_NORMAL_FORCE_PAIR" : "OFF") +
            "/requiredKneeReaction=" + (_primary != null && PelvisRunScopes.HasKneeCoupling(_primary.DynamicScope) ? "REDUCED_MASS_NORMAL_PD_PAIR" : "OFF") +
            "/requiredUpperGravity=" + (_primary != null && PelvisRunScopes.HasUpperWeight(_primary.DynamicScope) ? "EXACT_STANCE_SERIAL_SELF_WEIGHT" : "OFF") +
            "/requiredTorsoGravity=OFF/requiredReciprocal=OFF/hipReaction=" + (_primary != null && PelvisRunScopes.HasHipCoupling(_primary.DynamicScope) ? "TRANSLATION_AT_HIP_ANCHORS" : "NOT_ROUTED") + "/hipAngularControllerReaction=" + PelvisDynamicReceiverIsolation.HipReactionName(_torso == null ? null : _torso.PelvisReceiverControl.Arm) + "/loadedSupportCertified=False";

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
            bool boundary = DynamicContactMatchesScope(scope);
            bool lowerLedger = lowerWeight && LowerLedgerEvidenceReady(PrimaryFootReport) && LowerLedgerEvidenceReady(SecondaryFootReport);
            bool upperLedger = upperWeight && UpperLedgerEvidenceReady(PrimaryReport?.UpperSelfWeightSupport) &&
                UpperLedgerEvidenceReady(SecondaryReport?.UpperSelfWeightSupport);
            bool wrench = upperWeight && _hipPelvisLoadPath.Report.DynamicMode && _hipPelvisLoadPath.Report.EvidenceReady;
            bool ready = hip && upper && runReady && boundary && surface && left && right && gravity && lower && knee;
            string missing = "";
            if (!runReady) missing += "RUNTIME_SAFETY;";
            if (!boundary) missing += "DYNAMIC_BOUNDARY;";
            if (!surface) missing += "SUPPORT_EXPOSURE;";
            if (!left) missing += "LEFT_CONTACT;";
            if (!right) missing += "RIGHT_CONTACT;";
            if (!gravity) missing += "FOOT_GRAVITY;";
            if (!lower) missing += "LOWER_WEIGHT;";
            if (!knee) missing += "KNEE_COUPLING;";
            if (!upper) missing += "UPPER_WEIGHT;";
            if (!hip) missing += "HIP_PAIR;";
            return PelvisRunScopes.Dataset(scope) + "=" + ready +
                " | runtimeAndSafetyReady=" + runReady + " | supportExposureReady=" + surface +
                " | bilateralContactEvidence(primary-secondary)=" + left + ":" + right +
                " | gravityScopeEvidenceReady=" + gravity + " | footSelfWeightEvidenceReady=" + (footSelfWeight && gravity) +
                " | lowerSelfWeightEvidenceReady=" + (lowerWeight && lower) +
                " | hipAnchorTranslationPairEvidenceReady=" + (PelvisRunScopes.HasHipCoupling(scope) && hip) +
                " | dynamicHipWrenchInputEvidenceReady=" + wrench +
                " | upperSelfWeightEvidenceReady=" + (upperWeight && upper) +
                " | upperLoadLedgerEvidenceReady=" + upperLedger +
                " | kneeNormalCouplingEvidenceReady=" + (kneeCoupling && knee) +
                " | lowerLoadLedgerEvidenceReady=" + lowerLedger +
                " | evidenceSemantics=LOCAL_PREREQUISITES_NOT_AGGREGATE_PASS_V041190 | missingPrerequisites=" + (missing.Length == 0 ? "<none>" : missing) +
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

        // Finite defaults alone are not collected evidence. Keep the existing load-exposure
        // minimum and require the ledger's complete resolved/censored accounting independently.
        private static bool UpperLedgerEvidenceReady(UpperSelfWeightSupportReport r) => r != null &&
            r.LedgerFinite && r.Samples > 0 && r.LedgerCaptured == r.Samples &&
            r.LedgerLoaded >= UpperSelfWeightSupportNormalLoadTransferIsolation.MinimumEligibleSamples &&
            r.LedgerResolved >= r.LedgerLoaded && r.LedgerCensored >= 0 &&
            r.LedgerResolved + r.LedgerCensored == r.LedgerCaptured;

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
        private static bool ContactEvidenceReady(FootAnkleFoundationReport r) => r != null && r.Finite && r.ContactEntryEvidenceReady &&
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
            r.StanceContactLifecycleTraceQualified + "-" + r.MechanicalReactionTraceQualified +
            "/contactEntry=" + (r.ContactEntrySummary ?? "<none>");

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
