using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class PelvisReceiverReport
    {
        private int _recoveryCommitted, _recoverySelected, _recoveryQualified, _recoveryEligible, _recoveryRaised, _recoveryLimited;
        private float _recoveryExtraPeak, _recoveryRemainingPeak;
        private SupportNormalRecovery.Result _recoveryPeak;

        internal void ResetSupportRecovery()
        {
            _recoveryCommitted = _recoverySelected = _recoveryQualified = _recoveryEligible = _recoveryRaised = _recoveryLimited = 0;
            _recoveryExtraPeak = _recoveryRemainingPeak = 0f; _recoveryPeak = default;
        }
        // Call only after the original receiver's three native writes returned.
        // This is still before bilateral allocation / the next solved response.
        internal void RecordSupportRecovery(SupportNormalRecovery.Result r, PhysicsStepContext step)
        {
            if (!r.Step.Matches(step)) return;
            _recoveryCommitted++;
            if (r.Selected) _recoverySelected++;
            if (!r.Qualified || !r.Support.Matches(step)) return;
            _recoveryQualified++;
            if (r.Eligible) _recoveryEligible++;
            // Evidence threshold on the uncoupled one-step displacement equivalent;
            // not a control deadband and not a claim about realized body displacement.
            if (r.ExtraY * step.Dt * step.Dt > .000001f) _recoveryRaised++;
            if (r.RemainingY * step.Dt * step.Dt > .000001f) _recoveryLimited++;
            _recoveryRemainingPeak = System.Math.Max(_recoveryRemainingPeak, r.RemainingY);
            if (r.ExtraY > _recoveryExtraPeak) { _recoveryExtraPeak = r.ExtraY; _recoveryPeak = r; }
        }
        private string SupportRecoverySummary()
        {
            var sb = new StringBuilder("supportNormalRecovery=V041370/receiverCommitted-selected-qualified-eligible-raised-limited=");
            sb.Append(_recoveryCommitted).Append('|').Append(_recoverySelected).Append('|').Append(_recoveryQualified)
                .Append('|').Append(_recoveryEligible).Append('|').Append(_recoveryRaised).Append('|').Append(_recoveryLimited)
                .Append("/peak(extraY-remainingY)mps2=").Append(F(_recoveryExtraPeak)).Append('|').Append(F(_recoveryRemainingPeak))
                .Append("/raised=EXTRA_A_TIMES_DT_SQUARED_GT_0.001MM")
                .Append("/domain=RECEIVER_COMMITTED_REQUEST_BEFORE_BILATERAL_ALLOCATION_NOT_SOLVED")
                .Append("/floor=HELD_REFERENCE_COM_HEIGHT_NOT_GROUND/noReferenceOffset=True/tangentRequestPreserved=True/originalCapMps2=12");
            if (_recoveryPeak.Step.Valid) AppendSupportRecovery(sb, _recoveryPeak);
            return sb.ToString();
        }
        private static void AppendSupportRecovery(StringBuilder sb, SupportNormalRecovery.Result r)
        {
            if (!r.Step.Valid) { sb.Append("/supportRecoveryV041370=NA_NO_CURRENT_COMMIT"); return; }
            sb.Append("/supportRecoveryV041370(selected-qualified-eligible)=").Append(r.Selected).Append('|').Append(r.Qualified).Append('|').Append(r.Eligible);
            if (!r.Qualified) return;
            sb.Append("/recoveryStep(epoch-fixed-source-dt)=").Append(r.Step.Epoch).Append('|').Append(r.Step.Sequence).Append('|')
                .Append(r.Step.SourceSequence).Append('|').Append(F(r.Step.Dt))
                .Append("/recoverySupport(id-mask-approachMask-observedFixed)=").Append(r.Support.SupportId).Append('|')
                .Append(r.Support.StanceMask).Append('|').Append(r.Support.ApproachMask).Append('|').Append(r.Support.ObservedStep.Sequence)
                .Append("/recoveryHeight(floor-body-projected)m=").Append(F(r.FloorY)).Append('|').Append(F(r.BodyY)).Append('|').Append(F(r.ProjectedY))
                .Append("/recoveryVelocityY=").Append(F(r.BodyVY))
                .Append("/recoveryA(required-baseline-request-extra-remaining)=").Append(F(r.RequiredY)).Append('|').Append(F(r.Baseline.y)).Append('|')
                .Append(F(r.Request.y)).Append('|').Append(F(r.ExtraY)).Append('|').Append(F(r.RemainingY));
        }
    }
}
