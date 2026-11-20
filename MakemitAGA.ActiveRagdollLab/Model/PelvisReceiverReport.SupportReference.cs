using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class PelvisReceiverReport
    {
        private int _supportPlans, _supportEligible, _supportSelected, _supportRaised, _supportBelow;
        private int _supportPrimary, _supportSecondary;
        private int _supportApproachOnly, _supportMixed, _supportMeaningful, _supportApproachMeaningful, _supportApproachBelow;
        private int _supportPrioritySelected, _supportPriorityBraking, _supportPriorityRaised, _supportTangentChanged, _supportCandidateBelow;
        private float _supportPriorityExtraPeak, _supportTangentDeltaPeak, _supportCandidateUnderPeak;
        private float _supportDeltaVPeak, _supportUnderPeak;
        private float _supportApproachUnderPeak;
        private PelvisComReferencePlanner.ComPlan _supportPeakPlan;
        private PelvisComReferencePlanner.ComPlan _supportApproachPeakPlan;
        private PhysicsStepContext _supportPeakStep;
        private PhysicsStepContext _supportApproachPeakStep;
        private int _supportBrakeSelected, _supportBrakeAdvanced, _supportBrakeResidual;
        private float _supportBrakeAdvancePeak, _supportBrakeResidualPeak;
        private PelvisComReferencePlanner.ComPlan _supportBrakePeakPlan;
        private PhysicsStepContext _supportBrakePeakStep;

        internal void ResetSupportReference()
        {
            _supportPlans = _supportEligible = _supportSelected = _supportRaised = _supportBelow = 0;
            _supportPrimary = _supportSecondary = 0;
            _supportApproachOnly = _supportMixed = _supportMeaningful = _supportApproachMeaningful = _supportApproachBelow = 0;
            _supportPrioritySelected = _supportPriorityBraking = _supportPriorityRaised = _supportTangentChanged = _supportCandidateBelow = 0;
            _supportPriorityExtraPeak = _supportTangentDeltaPeak = _supportCandidateUnderPeak = 0f;
            _supportDeltaVPeak = _supportUnderPeak = 0f;
            _supportApproachUnderPeak = 0f;
            _supportPeakPlan = default; _supportPeakStep = default;
            _supportApproachPeakPlan = default; _supportApproachPeakStep = default;
            _supportBrakeSelected = _supportBrakeAdvanced = _supportBrakeResidual = 0;
            _supportBrakeAdvancePeak = _supportBrakeResidualPeak = 0f;
            _supportBrakePeakPlan = default; _supportBrakePeakStep = default;
        }
        internal void RecordSupportReference(PelvisComReferencePlanner.ComPlan plan, PhysicsStepContext step)
        {
            _supportPlans++;
            if (!plan.SupportInput.Matches(step)) return;
            _supportEligible++;
            if ((plan.SupportInput.StanceMask & 1) != 0) _supportPrimary++;
            if ((plan.SupportInput.StanceMask & 2) != 0) _supportSecondary++;
            if (plan.SupportFloorSelected) _supportSelected++;
            if (plan.SupportBrakingSelected) _supportBrakeSelected++;
            float advance = plan.SupportBrakingExtraY * step.Dt;
            if (advance > .000001f) _supportBrakeAdvanced++;
            if (plan.SupportBrakingFinalResidual > .000001f) _supportBrakeResidual++;
            _supportBrakeResidualPeak = System.Math.Max(_supportBrakeResidualPeak, plan.SupportBrakingFinalResidual);
            if (advance > _supportBrakeAdvancePeak)
            {
                _supportBrakeAdvancePeak = advance; _supportBrakePeakPlan = plan; _supportBrakePeakStep = step;
            }
            if (plan.SupportCandidateDeltaY > 0f) _supportRaised++;
            // A nonzero float is not necessarily a useful preparation. This is an
            // evidence threshold on candidate displacement, never a control gate.
            bool meaningful = plan.SupportCandidateDeltaY * step.Dt > 0.000001f;
            if (meaningful) _supportMeaningful++;
            if (plan.SupportNormalPrioritySelected) _supportPrioritySelected++;
            if (plan.SupportTangentBrakingPreserved) _supportPriorityBraking++;
            if (plan.SupportPriorityExtraY * step.Dt > 0.000001f) _supportPriorityRaised++;
            if (plan.SupportTangentDeltaV * step.Dt > 0.000001f) _supportTangentChanged++;
            if (plan.SupportCandidateUnderY > 0.000001f) _supportCandidateBelow++;
            _supportPriorityExtraPeak = System.Math.Max(_supportPriorityExtraPeak, plan.SupportPriorityExtraY);
            _supportTangentDeltaPeak = System.Math.Max(_supportTangentDeltaPeak, plan.SupportTangentDeltaV);
            _supportCandidateUnderPeak = System.Math.Max(_supportCandidateUnderPeak, plan.SupportCandidateUnderY);
            if (plan.SupportInput.ApproachOnly)
            {
                _supportApproachOnly++;
                if (meaningful) _supportApproachMeaningful++;
                if (plan.SupportRemainingUnderY > 0.000001f) _supportApproachBelow++;
                if (!_supportApproachPeakStep.Valid || plan.SupportRemainingUnderY > _supportApproachUnderPeak)
                {
                    _supportApproachUnderPeak = plan.SupportRemainingUnderY;
                    _supportApproachPeakPlan = plan; _supportApproachPeakStep = step;
                }
            }
            else if (plan.SupportInput.ApproachMask != 0) _supportMixed++;
            _supportDeltaVPeak = System.Math.Max(_supportDeltaVPeak, plan.SupportCandidateDeltaY);
            if (plan.SupportRemainingUnderY > 0.000001f) _supportBelow++;
            if (!_supportPeakStep.Valid || plan.SupportRemainingUnderY > _supportUnderPeak)
            {
                _supportUnderPeak = plan.SupportRemainingUnderY;
                _supportPeakPlan = plan; _supportPeakStep = step;
            }
        }
        private string SupportReferenceSummary()
        {
            var sb = new StringBuilder("parentSupportReference=V041220/planned-eligible-selected-raised-below=");
            sb.Append(_supportPlans).Append('|').Append(_supportEligible).Append('|').Append(_supportSelected)
                .Append('|').Append(_supportRaised).Append('|').Append(_supportBelow)
                .Append("/stancePrimary-secondary=").Append(_supportPrimary).Append('|').Append(_supportSecondary)
                .Append("/peak(candidateDeltaVmps-remainingUnderMm)=").Append(F(_supportDeltaVPeak)).Append('|').Append(F(_supportUnderPeak * 1000f))
                .Append("/witnessPlans(confirmedOnly-approachOnly-mixed)=").Append(_supportEligible - _supportApproachOnly - _supportMixed)
                .Append('|').Append(_supportApproachOnly).Append('|').Append(_supportMixed)
                .Append("/meaningfulRaised(total-approachOnly)=").Append(_supportMeaningful).Append('|').Append(_supportApproachMeaningful)
                .Append("/approachOnly(below-underPeakMm)=").Append(_supportApproachBelow).Append('|').Append(F(_supportApproachUnderPeak * 1000f))
                .Append("/meaningfulCandidate=DELTA_V_TIMES_DT_GT_0.001MM")
                .Append("/normalBudget(prioritySelected-brakingKept-extraRaised-tangentChanged)=").Append(_supportPrioritySelected)
                .Append('|').Append(_supportPriorityBraking).Append('|').Append(_supportPriorityRaised).Append('|').Append(_supportTangentChanged)
                .Append("/priorityPeak(extraDeltaY-tangentDeltaVmps)=").Append(F(_supportPriorityExtraPeak)).Append('|').Append(F(_supportTangentDeltaPeak))
                .Append("/candidateBeforeFeedback(below-underPeakMm)=").Append(_supportCandidateBelow).Append('|').Append(F(_supportCandidateUnderPeak * 1000f))
                .Append("/domain=PLANNED_BEFORE_ACTUATION_NOT_SOLVED/support=PREVIOUS_COMPLETED_FIXED_STANCE_OR_APPROACH_NOT_CONTACT_OR_LOAD")
                .Append("/floor=HELD_CALIBRATED_ANIMATION_PELVIS_ORIGIN_PLUS_CURRENT_REFERENCE_COM_OFFSET")
                .Append("/bounds=EXISTING_VECTOR_V3_A6/feedback=ORIGINAL_DEFICIT_AND_PHYSICAL_BRAKE_AFTER_CANDIDATE")
                .Append("/scope=HORIZONTAL_OWNED_BOX/groundFeasibilityCertified=False");
            sb.Append("/supportReferenceBraking=V041360/selected-advanced-finalResidual=").Append(_supportBrakeSelected)
                .Append('|').Append(_supportBrakeAdvanced).Append('|').Append(_supportBrakeResidual)
                .Append("/brakingPeak(candidateAdvanceMm-finalResidualMm)=").Append(F(_supportBrakeAdvancePeak * 1000f))
                .Append('|').Append(F(_supportBrakeResidualPeak * 1000f))
                .Append("/brakingModel=HELD_FLOOR_REFERENCE_ONLY_FULL_A6_RESERVE_NOT_BODY_CAPABILITY")
                .Append("/advanced=PRE_FEEDBACK_CANDIDATE_DELTA_Y_TIMES_DT_GT_0.001MM");
            if (_supportPeakStep.Valid) AppendSupportReference(sb, _supportPeakPlan, _supportPeakStep);
            if (_supportApproachPeakStep.Valid)
            {
                sb.Append("/approachOnlyPeak");
                AppendSupportReference(sb, _supportApproachPeakPlan, _supportApproachPeakStep);
            }
            if (_supportBrakePeakStep.Valid)
            {
                sb.Append("/brakingAdvancePeak");
                AppendSupportReference(sb, _supportBrakePeakPlan, _supportBrakePeakStep);
            }
            return sb.ToString();
        }
        private static void AppendSupportReference(StringBuilder sb, PelvisComReferencePlanner.ComPlan plan, PhysicsStepContext step)
        {
            if (!plan.SupportInput.Matches(step)) { sb.Append("/parentSupportV041220=NA_NO_SELECTED_PREVIOUS_WITNESS"); return; }
            sb.Append("/parentSupportV041220(selected-mask-approachMask-support-observed-for)=").Append(plan.SupportFloorSelected).Append('|')
                .Append(plan.SupportInput.StanceMask).Append('|').Append(plan.SupportInput.ApproachMask).Append('|').Append(plan.SupportInput.SupportId).Append('|')
                .Append(plan.SupportInput.ObservedStep.Epoch).Append(':').Append(plan.SupportInput.ObservedStep.Sequence).Append('|')
                .Append(step.Epoch).Append(':').Append(step.Sequence)
                .Append("/parentSupportY(floor-final-nominalCandidateV-candidateDeltaV-underMm)=")
                .Append(F(plan.SupportFloorY)).Append('|').Append(F(plan.Position.y)).Append('|').Append(F(plan.NominalCandidateY))
                .Append('|').Append(F(plan.SupportCandidateDeltaY)).Append('|').Append(F(plan.SupportRemainingUnderY * 1000f))
                .Append("/normalBudgetV041220(priority-brakingKept-extraDeltaY-tangentDeltaV-candidateUnderMm)=")
                .Append(plan.SupportNormalPrioritySelected).Append('|').Append(plan.SupportTangentBrakingPreserved).Append('|')
                .Append(F(plan.SupportPriorityExtraY)).Append('|').Append(F(plan.SupportTangentDeltaV)).Append('|').Append(F(plan.SupportCandidateUnderY * 1000f))
                .Append("/supportBrakingV041360(selected-minV-extraV-baselineCandidateResidualMm-finalResidualMm)=")
                .Append(plan.SupportBrakingSelected).Append('|').Append(F(plan.SupportBrakingMinimumY)).Append('|')
                .Append(F(plan.SupportBrakingExtraY)).Append('|').Append(F(plan.SupportBrakingBaselineResidual * 1000f)).Append('|')
                .Append(F(plan.SupportBrakingFinalResidual * 1000f))
                .Append("/brakingReference(pY-vY-aY-candidateAy)=").Append(F(plan.Position.y)).Append('|')
                .Append(F(plan.Velocity.y)).Append('|').Append(F(plan.Acceleration.y)).Append('|').Append(F(plan.CandidateA.y));
        }
    }
}
