using System.Diagnostics;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static partial class HipBilateralForceSolver
    {
        // The exact v129 CAP10 affine path, now an optional real allocation.
        // It starts at the UNIFORM q solution, not the retained-Y solution:
        // Upper scale=q*(1-t); system XZ=q*request.XZ; system Y=q*request.Y+lostY*t.
        // Compare against the separately computed same-q result; never stack both gains.
        private static void SelectBoundedTrade(HipAnchorPairInput l, HipAnchorPairInput r, float q,
            Vector3 uniformL, Vector3 uniformR, Vector3 uniformCommon,
            ref Vector3 lf, ref Vector3 rf, ref Vector3 common, ref float angularScale, ref float extra,
            ref HipAllocationDecision decision)
        {
            decision.Attempted = true;
            long begin = Stopwatch.GetTimestamp();
            var c = PlanBoundedTrade(l, r, q, uniformL, uniformR, uniformCommon);
            decision.ComputeMilliseconds = (Stopwatch.GetTimestamp() - begin) * (1000d / Stopwatch.Frequency);
            decision.CandidateValid = c.Valid; decision.Issue = c.Issue;
            if (!c.Valid) return; // Keep the exact same-q values, including fallback arithmetic.
            decision.TradeExtra = c.Extra; decision.TradeFraction = c.T;
            decision.LeftTorqueCost = (q - c.AngularScale) * l.PureTorque.magnitude;
            decision.RightTorqueCost = (q - c.AngularScale) * r.PureTorque.magnitude;
            // Numerical tie guard only, not a demonstrated physical-usefulness threshold.
            // In particular the retired audit's 0.01 m/s² histogram is NOT a control gate.
            float tolerance = .000001f * (1f + Mathf.Abs(q * l.Seed.ParentInput.Acceleration.y) + Mathf.Abs(extra));
            if (c.Extra - extra <= tolerance || c.AngularScale >= q) return;
            lf = c.Left; rf = c.Right; common = c.Common;
            angularScale = c.AngularScale; extra = c.Extra; decision.Applied = true;
        }

        private struct TradePlan
        {
            internal bool Valid;
            internal float T, AngularScale, Extra;
            internal Vector3 Left, Right, Common;
            internal HipAllocationCandidateIssue Issue;
        }

        private static TradePlan PlanBoundedTrade(HipAnchorPairInput l, HipAnchorPairInput r, float q,
            Vector3 uniformL, Vector3 uniformR, Vector3 uniformCommon)
        {
            var result = new TradePlan();
            Vector3 baseTarget = q * l.Seed.ParentInput.Acceleration;
            float lost = (1f - q) * l.Seed.ParentInput.Acceleration.y;
            Vector3 endTarget = baseTarget; endTarget.y = l.Seed.ParentInput.Acceleration.y;
            if (!TradeMotion(l, r, 0f, endTarget, out var le, out var re, out var ue) ||
                !TryBudgetLine(l, r, q, 0f, uniformL, uniformR, le, re, uniformCommon, ue, out var line))
            { result.Issue = HipAllocationCandidateIssue.EndpointPlan; return result; }
            if (!IndependentBudgetInterval(line, out float low, out float high))
            { result.Issue = HipAllocationCandidateIssue.NoInterval; return result; }
            if (!TryBoundedTradeRange(low, high, HipSystemAllocation.MaxAngularTradeFraction, out float t))
            { result.Issue = HipAllocationCandidateIssue.CapExcludesInterval; return result; }
            for (int retry = 0; retry < 4; retry++)
            {
                float angularScale = q * (1f - t);
                Vector3 target = baseTarget; target.y += lost * t;
                if (!TradeMotion(l, r, angularScale, target, out var lf, out var rf, out var common))
                { result.Issue = HipAllocationCandidateIssue.EndpointPlan; return result; }
                float a = 0f, b = 1f;
                float extra = target.y - baseTarget.y;
                // Recompute and independently validate the expression actually committed.
                // Target() must reconstruct the same Y after the extra receipt round-trip.
                if (BudgetInterval(l, r, angularScale, angularScale, lf, rf, lf, rf, common, common, ref a, ref b) &&
                    HipSystemAllocation.AngularValid(l.Seed.ParentInput, q, angularScale, extra) &&
                    HipSystemAllocation.Matches(l.Seed.ParentInput, q, extra, common, -lf - rf) &&
                    TradeClosure(l, r, angularScale, target, lf, rf, common))
                {
                    result.Valid = true; result.T = t; result.AngularScale = angularScale; result.Extra = extra;
                    result.Left = lf; result.Right = rf; result.Common = common;
                    return result;
                }
                float next = low + (t - low) * .99999f;
                if (next >= t || next < low) break;
                t = next;
            }
            result.Issue = HipAllocationCandidateIssue.EndpointRecheck;
            return result;
        }

        // The feasible interval can have a positive lower bound. Clipping only hi
        // would admit an infeasible command when the cap lies below that lower bound.
        internal static bool TryBoundedTradeRange(float low, float high, float cap, out float t)
        {
            t = 0f;
            if (!F(low) || !F(high) || !F(cap) || low < 0f || high > 1f || low > high || cap < 0f || cap > 1f) return false;
            t = Mathf.Min(high, cap);
            return t >= low;
        }
        private static bool TradeMotion(HipAnchorPairInput l, HipAnchorPairInput r, float angularScale, Vector3 target,
            out Vector3 lf, out Vector3 rf, out Vector3 common)
        {
            Vector3 reaction = Reaction(l, r, angularScale);
            lf = l.Seed.ChildMass * (l.Seed.FeedbackA - PointBias(l, angularScale, reaction) + target);
            rf = r.Seed.ChildMass * (r.Seed.FeedbackA - PointBias(r, angularScale, reaction) + target);
            common = target + (lf + rf) / l.Seed.ParentInput.Response.TotalMass;
            return V(lf) && V(rf) && V(common);
        }
        private static bool TradeClosure(HipAnchorPairInput l, HipAnchorPairInput r, float angularScale, Vector3 target,
            Vector3 lf, Vector3 rf, Vector3 common)
        {
            Vector3 reaction = Reaction(l, r, angularScale);
            return (common - (lf + rf) / l.Seed.ParentInput.Response.TotalMass - target).magnitude <= .0001f * (1f + target.magnitude) &&
                (ForceResponse(l, lf, rf) + PointBias(l, angularScale, reaction) - common - l.Seed.FeedbackA).magnitude <= .001f * (1f + l.Seed.FeedbackA.magnitude) &&
                (ForceResponse(r, rf, lf) + PointBias(r, angularScale, reaction) - common - r.Seed.FeedbackA).magnitude <= .001f * (1f + r.Seed.FeedbackA.magnitude);
        }
    }
}
