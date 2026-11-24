using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralLegCoordinator
    {
        // Shared by setup readback and live scope validation; tested with the real Knee selection.
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
                // v127 keeps the v126 B joint input in BOTH allocation arms.
                // Match KneeReferenceArmSelected; unknown/absent receivers still reject.
                bool kneeGroundTarget = continuousEntry && _torso != null &&
                    (_torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmA ||
                     _torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmB);
                bool kneeExtension = kneeGroundTarget;
                bool hipLanding = kneeGroundTarget && PelvisDynamicReceiverIsolation.UsesHipLandingPreparation(_torso.PelvisReceiverControl.Arm);
                bool kneeLanding = kneeGroundTarget && PelvisDynamicReceiverIsolation.UsesKneeLandingPreparation(_torso.PelvisReceiverControl.Arm);
                bool footTrajectory = kneeGroundTarget;
                bool footKneeProgress = kneeGroundTarget;
                bool loadAdmission = continuousEntry; // V041200: both arms retain v119 B admission.
                var precontact = footTrajectory ? RightFootPrecontact.ForArm(_torso.PelvisReceiverControl.Arm) : RightFootPrecontact.Mode.Off;
                if (!PrecontactMatches(precontact) ||
                    _primary.ContactEntryContinuous != continuousEntry || _secondary.ContactEntryContinuous != continuousEntry ||
                    _primary.ContactEntryRecovery != recoveryEntry || _secondary.ContactEntryRecovery != recoveryEntry ||
                    _primary.StanceLoadAdmissionSelected != loadAdmission || _secondary.StanceLoadAdmissionSelected != loadAdmission ||
                    _primary.GroundReferenceEnabled != continuousEntry || _secondary.GroundReferenceEnabled != continuousEntry ||
                    _primary.GroundReferenceApplied != continuousEntry || _secondary.GroundReferenceApplied != continuousEntry ||
                    _primary.GroundReferencePartial != partialGround || _secondary.GroundReferencePartial != partialGround ||
                    _primary.KneeGroundTargetSelected != kneeGroundTarget || _secondary.KneeGroundTargetSelected != kneeGroundTarget ||
                    _primary.FootGroundTrajectorySelected != footTrajectory || _secondary.FootGroundTrajectorySelected != footTrajectory ||
                    _primary.FootKneeProgressSelected != footKneeProgress || _secondary.FootKneeProgressSelected != footKneeProgress ||
                    _primary.KneeGroundExtensionSelected != kneeExtension || _secondary.KneeGroundExtensionSelected != kneeExtension ||
                    _primary.HipLandingPreparationSelected != hipLanding || _secondary.HipLandingPreparationSelected != hipLanding ||
                    _primary.KneeLandingPreparationSelected != kneeLanding || _secondary.KneeLandingPreparationSelected != kneeLanding ||
                    _primary.KneeReferenceSelected != coherentKnee || _secondary.KneeReferenceSelected != coherentKnee ||
                    _primary.KneePlannedFeedforwardSelected != plannedKneeFeedforward ||
                    _secondary.KneePlannedFeedforwardSelected != plannedKneeFeedforward)
                    return DynamicContactBoundaryIssue.ContactLifecycle;
                DynamicContactBoundaryIssue primary = _primary.DynamicContactIssue;
                return primary != DynamicContactBoundaryIssue.None ? primary : _secondary.DynamicContactIssue;
            }
        }
        private bool PrecontactMatches(RightFootPrecontact.Mode expected)
        {
            var primary = _primary.PrecontactMode;
            var secondary = _secondary.PrecontactMode;
            const RightFootPrecontact.Mode off = RightFootPrecontact.Mode.Off;
            if (expected == off) return primary == off && secondary == off;
            if (string.Equals(PrimarySide, "RIGHT", StringComparison.OrdinalIgnoreCase))
                return primary == expected && secondary == off && string.Equals(SecondarySide, "LEFT", StringComparison.OrdinalIgnoreCase);
            return primary == off && secondary == expected && string.Equals(PrimarySide, "LEFT", StringComparison.OrdinalIgnoreCase) &&
                string.Equals(SecondarySide, "RIGHT", StringComparison.OrdinalIgnoreCase);
        }

        internal bool DynamicContactReady => DynamicContactIssue == DynamicContactBoundaryIssue.None;
        internal bool DynamicContactMatchesScope(PelvisRunScope scope) => PelvisRunScopes.IsContact(scope) &&
            DynamicContactReady && _primary.DynamicScope == scope && _secondary.DynamicScope == scope;
    }
}
