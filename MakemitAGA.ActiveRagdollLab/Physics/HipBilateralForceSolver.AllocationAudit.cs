using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static partial class HipBilateralForceSolver
    {
        // Read-only same-state candidates. Never compose a physical Command from the
        // angular-tradeoff result: its two scales are outside the live receipt contract.
        internal static bool TryAudit(HipAnchorPairInput l, HipAnchorPairInput r,
            HipAnchorPairReport.Command actualL, HipAnchorPairReport.Command actualR, out HipAllocationAuditSample sample)
        {
            sample = default;
            var p = l.Seed.ParentInput; var context = p.AllocationAudit;
            if (!l.Ready || !r.Ready || !p.SameAs(r.Seed.ParentInput) || !context.Matches(p) ||
                !r.Seed.ParentInput.AllocationAudit.Matches(p) ||
                context.Eligible != r.Seed.ParentInput.AllocationAudit.Eligible || context.ErrorY != r.Seed.ParentInput.AllocationAudit.ErrorY ||
                !actualL.Step.Matches(p.Step) || !actualR.Step.Matches(p.Step) || !p.JointTorqueReaction ||
                !actualL.JointSolved || !actualR.JointSolved || !l.Seed.Step.Matches(p.Step) || !r.Seed.Step.Matches(p.Step) ||
                !p.SameAs(actualL.ParentInput) || !p.SameAs(actualR.ParentInput) ||
                actualL.ChildId != l.Seed.ChildId || actualR.ChildId != r.Seed.ChildId || actualL.ChildId == actualR.ChildId ||
                actualL.ParentId != p.BodyId || actualR.ParentId != p.BodyId ||
                actualL.Fixed != l.Seed.Fixed || actualR.Fixed != r.Seed.Fixed || l.Seed.Fixed != r.Seed.Fixed ||
                actualL.MotionScale != actualR.MotionScale || actualL.FullFeedbackFeasible != actualR.FullFeedbackFeasible ||
                !PelvisChestPointResponse.Same(actualL.CommonAcceleration, actualR.CommonAcceleration) ||
                !PelvisChestPointResponse.Same(actualL.PeerForce, actualR.Force) || !PelvisChestPointResponse.Same(actualR.PeerForce, actualL.Force) ||
                !PelvisChestPointResponse.Same(actualL.NetHipTorque, actualL.MotionScale * l.PureTorque) ||
                !PelvisChestPointResponse.Same(actualR.NetHipTorque, actualR.MotionScale * r.PureTorque) ||
                p.UpwardRetentionSelected || p.UpwardRetentionEligible || actualL.UpwardExtraAcceleration != 0f || actualR.UpwardExtraAcceleration != 0f ||
                !HipSystemAllocation.Matches(p, actualL.MotionScale, 0f, actualL.CommonAcceleration, -actualL.Force - actualR.Force)) return false;
            float q = actualL.MotionScale;
            sample = new HipAllocationAuditSample { Valid = true, Step = p.Step, LeftFixed = actualL.Fixed, RightFixed = actualR.Fixed,
                Eligible = context.Eligible, Limited = q < 1f, FullFeedback = actualL.FullFeedbackFeasible,
                ErrorY = context.ErrorY, Q = q, RequestY = p.Acceleration.y, BaselineY = q * p.Acceleration.y };
            // The committed q=1 baseline already admits the full request. Avoid all
            // extra interval solves on this common path. No extra per-step records.
            if (q == 1f) return sample.FullFeedback;
            if (!MotionRequest(l, r, 0f, out var l0, out var r0, out var u0) ||
                !MotionRequest(l, r, 1f, out var l1, out var r1, out var u1) ||
                !TryBudgetLine(l, r, 0f, 1f, l0, r0, l1, r1, u0, u1, out var uniform)) return false;
            bool feasible = AuditCuts(uniform, out sample.FullRequestOver, out int uniformCuts, out float low, out float high);
            // Independent ball clipping and the sequential live intersection can
            // differ at inward-rounding scale. Attribute cuts only to feasible q;
            // a feedback fallback has no such upper-bound interpretation.
            if (sample.FullFeedback)
            {
                if (!feasible || Mathf.Abs(high - q) > .00001f) return false;
                sample.UniformCuts = uniformCuts;
            }
            if (!sample.Eligible || !sample.Limited || !sample.FullFeedback) return true;
            sample.Compared = true;
            float lost = (1f - q) * p.Acceleration.y;
            if (!F(lost) || lost <= 0f) return false;

            // Candidate 1 is the exact v127 B path, evaluated on value copies only.
            var probeL = l; var probeR = r; var probeParent = p;
            probeParent.UpwardRetentionSelected = probeParent.UpwardRetentionEligible = true;
            probeL.Seed.ParentInput = probeR.Seed.ParentInput = probeParent;
            Vector3 lf = actualL.Force, rf = actualR.Force, common = actualL.CommonAcceleration;
            RetainUpward(probeL, probeR, q, ref lf, ref rf, ref common, out float extra);
            sample.SameQGain = extra;
            sample.SameQValid = HipSystemAllocation.Matches(probeParent, q, extra, common, -lf - rf);
            if (MotionRequest(probeL, probeR, q, out l1, out r1, out u1, lost) &&
                TryBudgetLine(l, r, q, q, actualL.Force, actualR.Force, l1, r1, actualL.CommonAcceleration, u1, out var sameQ))
                AuditCuts(sameQ, out _, out sample.SameQCuts, out _, out _);

            // Candidate 2 follows ONE affine path: keep actual XZ and full Hip
            // point feedback, exchange a fraction of allocated Upper net torque
            // (including its relative reaction) for the missing positive Y.
            // This is not a global optimizer or a prediction of future leg tracking.
            Vector3 baseTarget = q * p.Acceleration;
            Vector3 endTarget = baseTarget; endTarget.y = p.Acceleration.y;
            if (!AuditMotion(l, r, 0f, endTarget, out l1, out r1, out u1) ||
                !TryBudgetLine(l, r, q, 0f, actualL.Force, actualR.Force, l1, r1, actualL.CommonAcceleration, u1, out var trade))
            { sample.TradeIssue = HipAllocationCandidateIssue.EndpointPlan; return true; }
            bool tradeFeasible = AuditCuts(trade, out _, out sample.TradeCuts, out low, out high);
            if (!tradeFeasible) { sample.TradeIssue = HipAllocationCandidateIssue.NoInterval; return true; }
            float t = high;
            for (int retry = 0; retry < 4; retry++, t *= .99999f)
            {
                float upperScale = q * (1f - t);
                Vector3 target = baseTarget; target.y += lost * t;
                if (!AuditMotion(l, r, upperScale, target, out lf, out rf, out common))
                { sample.TradeIssue = HipAllocationCandidateIssue.EndpointPlan; return true; }
                float a = 0f, b = 1f;
                if (!BudgetInterval(l, r, upperScale, upperScale, lf, rf, lf, rf, common, common, ref a, ref b) ||
                    !AuditClosure(l, r, upperScale, target, lf, rf, common)) continue;
                sample.TradeValid = true; sample.TradeT = t; sample.TradeUpperScale = upperScale;
                sample.TradeGain = target.y - baseTarget.y; sample.TradeTarget = target; sample.TradeCommon = common;
                sample.TradeLeftForce = lf; sample.TradeRightForce = rf;
                sample.UpperTorqueCost = new Vector3((q - upperScale) * l.PureTorque.magnitude,
                    (q - upperScale) * r.PureTorque.magnitude, 0f);
                sample.ReactionDelta = Reaction(l, r, upperScale) - Reaction(l, r, q);
                return true;
            }
            sample.TradeIssue = HipAllocationCandidateIssue.EndpointRecheck;
            return true; // Optional candidate rejection never rejects the live baseline.
        }
        private static bool AuditMotion(HipAnchorPairInput l, HipAnchorPairInput r, float upperScale, Vector3 target,
            out Vector3 lf, out Vector3 rf, out Vector3 common)
        {
            Vector3 reaction = Reaction(l, r, upperScale);
            lf = l.Seed.ChildMass * (l.Seed.FeedbackA - PointBias(l, upperScale, reaction) + target);
            rf = r.Seed.ChildMass * (r.Seed.FeedbackA - PointBias(r, upperScale, reaction) + target);
            common = target + (lf + rf) / l.Seed.ParentInput.Response.TotalMass;
            return V(lf) && V(rf) && V(common);
        }
        private static bool AuditClosure(HipAnchorPairInput l, HipAnchorPairInput r, float q, Vector3 target,
            Vector3 lf, Vector3 rf, Vector3 common)
        {
            Vector3 reaction = Reaction(l, r, q);
            return (common - (lf + rf) / l.Seed.ParentInput.Response.TotalMass - target).magnitude <= .0001f * (1f + target.magnitude) &&
                (ForceResponse(l, lf, rf) + PointBias(l, q, reaction) - common - l.Seed.FeedbackA).magnitude <= .001f * (1f + l.Seed.FeedbackA.magnitude) &&
                (ForceResponse(r, rf, lf) + PointBias(r, q, reaction) - common - r.Seed.FeedbackA).magnitude <= .001f * (1f + r.Seed.FeedbackA.magnitude);
        }
    }
}
