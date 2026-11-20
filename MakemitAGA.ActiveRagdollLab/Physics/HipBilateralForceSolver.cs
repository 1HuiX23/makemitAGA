using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Same-solve algebra only. Cancel Upper point moments and the combined effective
    // Pelvis point-force moment, then solve the P+C COM response together with both attachment
    // forces. Let aS=q*baselineA + upwardExtra*Y, F act on Upper, and u drive BOTH P/C COMs:
    //   aSystem=u-(FL+FR)/M; Fi=mi*(feedback_i-pointBias_i+aS).
    // Thus u=aS+(FL+FR)/M. This is known same-step input, not measured-A feedback
    // or an inverse-q gain. With joint reaction enabled, -angularScale*relativeTorque enters parent pointBias and
    // is NOT cancelled by the external point-force compensation. q scales baseline
    // SYSTEM motion; angularScale separately scales Upper net torque/reaction.
    // Both current arms retain Y and may select the v129 CAP10 path only
    // if it improves on that same-q result. All point/common/torque budgets remain.
    // The final actuator u has its own unchanged 12m/s² bound.
    // If no full-feedback q is feasible, q=0 and scale forces AND their common
    // compensation together. Flag the lost feedback; do not reset source/reference.
    internal static partial class HipBilateralForceSolver
    {
        internal const float UpperCorrectionTorqueLimit = 8f;
        internal const float UpperActuatorTorqueLimit = 8f;
        // Preserve the v99 envelope for a baseline request already above 8Nm.
        // Allocation may reduce this request; it grants no extra outward authority.
        internal static float ActuatorLimit(Vector3 baseline) => Mathf.Max(UpperActuatorTorqueLimit, baseline.magnitude);
        internal static bool TryPlan(HipAnchorPairInput left, HipAnchorPairInput right,
            out HipAnchorPairReport.Command lc, out HipAnchorPairReport.Command rc)
        {
            lc = rc = new HipAnchorPairReport.Command();
            if (!left.Ready || !right.Ready || left.Seed.ChildId == right.Seed.ChildId ||
                left.Seed.ParentId != right.Seed.ParentId || left.Seed.Fixed != right.Seed.Fixed ||
                !left.Seed.Step.Matches(right.Seed.Step) || !left.Seed.ParentInput.SameAs(right.Seed.ParentInput)) return false;
            PelvisHipCommandFrame baseline = left.Seed.ParentInput;
            if (!MotionRequest(left, right, 1f, out Vector3 lf, out Vector3 rf, out Vector3 common1) ||
                !MotionRequest(left, right, 0f, out Vector3 lf0, out Vector3 rf0, out Vector3 common0)) return false;
            float dt = left.Seed.Step.Dt, kp = HipCharacterJointResponseIsolation.TetherPositionSpring,
                kd = HipCharacterJointResponseIsolation.TetherVelocityDamper;
            float dampingGain = (kd + kp * dt) / (1f + kd * dt + kp * dt * dt);
            Vector3 ld = dampingGain * left.Seed.VelocityError, rd = dampingGain * right.Seed.VelocityError;
            Vector3 ldf = left.Seed.ChildMass * ld, rdf = right.Seed.ChildMass * rd;
            Vector3 lu = -Vector3.Cross(left.Seed.ChildPoint - left.Seed.ChildCom, lf);
            Vector3 ru = -Vector3.Cross(right.Seed.ChildPoint - right.Seed.ChildCom, rf);
            Vector3 la = ForceResponse(left, lf, rf), ra = ForceResponse(right, rf, lf);
            if (!V(ldf) || !V(rdf) || !V(lu) || !V(ru) || !V(la) || !V(ra)) return false;
            float pointScale = Bound(HipCharacterJointResponseIsolation.TetherMaxAcceleration, la.magnitude, ra.magnitude);
            float commonScale = Bound(PelvisHipCommandFrame.MaxMagnitude, common1.magnitude, 0f);
            float correctionScale = Bound(UpperCorrectionTorqueLimit, lu.magnitude, ru.magnitude);
            // q=1 budget ratios are diagnostics only. Retire the old three extra
            // force-only bisections: the actual allocation is the interval below.
            Vector3 leftActuator = left.PureTorque + lu, rightActuator = right.PureTorque + ru;
            if (!V(leftActuator) || !V(rightActuator)) return false;
            float actuatorScale = Mathf.Min(Bound(ActuatorLimit(left.PureTorque), leftActuator.magnitude, 0f),
                Bound(ActuatorLimit(right.PureTorque), rightActuator.magnitude, 0f));
            Vector3 moment = Vector3.Cross(left.Seed.ParentPoint - left.Seed.ParentCom, -lf) +
                Vector3.Cross(right.Seed.ParentPoint - right.Seed.ParentCom, -rf);
            if (!PelvisHipAttitudeCommand.TryPlanWithReaction(baseline, -lf - rf, moment, Reaction(left, right, 1f), out PelvisHipAttitudeCommand unbounded)) return false;
            float receiverScale = Bound(PelvisHipAttitudeCommand.TotalTorqueLimit, unbounded.RequestedTorque.magnitude, unbounded.ExternalRequestedTorque.magnitude);
            // Full-feedback force is affine in the common motion factor q. Intersect
            // all balls on that line; q=0 need not be feasible (do not binary-search
            // the whole allocation under a false monotonicity assumption).
            float lo = 0f, hi = 1f;
            bool feasible = BudgetInterval(left, right, 0f, 1f, lf0, rf0, lf, rf, common0, common1, ref lo, ref hi);
            float motion = feasible ? hi : 0f;
            if (!MotionRequest(left, right, motion, out lf, out rf, out Vector3 common)) return false;
            Vector3 uniformL = lf, uniformR = rf, uniformCommon = common;
            float upwardExtra = 0f, angularScale = motion;
            if (feasible && baseline.UpwardRetentionEligible)
                RetainUpward(left, right, motion, ref lf, ref rf, ref common, out upwardExtra);
            var decision = new HipAllocationDecision { SameQExtra = upwardExtra };
            if (feasible && baseline.BoundedTradeSelected && baseline.UpwardRetentionEligible && motion > 0f && motion < 1f)
                SelectBoundedTrade(left, right, motion, uniformL, uniformR, uniformCommon,
                    ref lf, ref rf, ref common, ref angularScale, ref upwardExtra, ref decision);
            float scale = 1f;
            if (!feasible)
            {
                lu = -Vector3.Cross(left.Seed.ChildPoint - left.Seed.ChildCom, lf);
                ru = -Vector3.Cross(right.Seed.ChildPoint - right.Seed.ChildCom, rf);
                moment = Vector3.Cross(left.Seed.ParentPoint - left.Seed.ParentCom, -lf) +
                    Vector3.Cross(right.Seed.ParentPoint - right.Seed.ParentCom, -rf);
                if (!PelvisHipAttitudeCommand.TryPlan(baseline, -lf - rf, moment, out PelvisHipAttitudeCommand fallback)) return false;
                float receiver = TorqueScale(baseline.ReceiverTorque, -fallback.EffectiveHipTorque, PelvisHipAttitudeCommand.TotalTorqueLimit);
                if (receiver < 0f) return false;
                // At q=0 net Upper torque is zero, so its actuator and correction
                // norms coincide. The unchanged 8Nm correction budget is tighter.
                scale = Mathf.Min(Bound(PelvisHipCommandFrame.MaxMagnitude, common.magnitude, 0f),
                    Mathf.Min(Bound(HipCharacterJointResponseIsolation.TetherMaxAcceleration,
                    ForceResponse(left, lf, rf).magnitude, ForceResponse(right, rf, lf).magnitude),
                    Mathf.Min(Bound(UpperCorrectionTorqueLimit, lu.magnitude, ru.magnitude), receiver)));
                if (scale < 1f) scale *= 0.999998f;
            }
            float dampingPower = scale * (Vector3.Dot(ldf, -left.Seed.VelocityError) + Vector3.Dot(rdf, -right.Seed.VelocityError));
            if (!F(scale) || scale < 0f || scale > 1f || !F(dampingPower) || dampingPower > 0.0001f) return false;
            Vector3 actualL = lf * scale, actualR = rf * scale;
            common *= scale;
            moment = Vector3.Cross(left.Seed.ParentPoint - left.Seed.ParentCom, -actualL) +
                Vector3.Cross(right.Seed.ParentPoint - right.Seed.ParentCom, -actualR);
            if (!PelvisHipAttitudeCommand.TryPlanWithReaction(baseline, -actualL - actualR, moment, Reaction(left, right, angularScale), out PelvisHipAttitudeCommand final) || final.Clamped) return false;
            final.MotionScale = motion; final.UpperAngularScale = angularScale; final.CommonAcceleration = common; final.UpwardExtraAcceleration = upwardExtra;
            if (!final.AllocationMatches()) return false;
            lc = Compose(left, lf, rf, actualL, actualR, ldf * scale, common, scale, pointScale, correctionScale, actuatorScale, receiverScale, commonScale, dampingPower, motion, angularScale, upwardExtra, feasible, Reaction(left, right, angularScale));
            rc = Compose(right, rf, lf, actualR, actualL, rdf * scale, common, scale, pointScale, correctionScale, actuatorScale, receiverScale, commonScale, dampingPower, motion, angularScale, upwardExtra, feasible, Reaction(left, right, angularScale));
            lc.AllocationDecision = rc.AllocationDecision = decision;
            float le = lc.FeedbackResidualA.magnitude, re = rc.FeedbackResidualA.magnitude;
            return V(lc.PredictedNetA) && V(rc.PredictedNetA) && V(lc.UpperActuatorTorque) && V(rc.UpperActuatorTorque) &&
                V(lc.AllocatedHipAlpha) && V(rc.AllocatedHipAlpha) && F(le) && F(re) &&
                (!feasible || (le <= 0.001f * (1f + left.Seed.FeedbackA.magnitude) && re <= 0.001f * (1f + right.Seed.FeedbackA.magnitude)));
        }
        // Second one-dimensional interval at the ORIGINAL feasible q. XZ, relative
        // reaction and allocated net Upper torque stay exact. No authority tradeoff,
        // q inverse, extra force owner or post-hoc clamp. If numerical validation of
        // the new endpoint fails, retain the already feasible uniform allocation.
        private static void RetainUpward(HipAnchorPairInput l, HipAnchorPairInput r, float q,
            ref Vector3 lf, ref Vector3 rf, ref Vector3 common, out float extra)
        {
            extra = 0f;
            float available = (1f - q) * l.Seed.ParentInput.Acceleration.y;
            if (available <= .000001f || !HipSystemAllocation.Valid(l.Seed.ParentInput, q, available) ||
                !MotionRequest(l, r, q, out Vector3 lf1, out Vector3 rf1, out Vector3 common1, available)) return;
            float lo = 0f, hi = 1f;
            if (!BudgetInterval(l, r, q, q, lf, rf, lf1, rf1, common, common1, ref lo, ref hi)) return;
            float candidate = available * hi;
            // Recompute from the actual command expression, not the interpolated
            // interval endpoints. A boundary rounding difference must not stop a run.
            for (int i = 0; i < 4 && candidate > .000001f; i++, candidate *= .99999f)
            {
                if (!MotionRequest(l, r, q, out Vector3 a, out Vector3 b, out Vector3 u, candidate)) return;
                lo = 0f; hi = 1f;
                if (!BudgetInterval(l, r, q, q, a, b, a, b, u, u, ref lo, ref hi) ||
                    !HipSystemAllocation.Matches(l.Seed.ParentInput, q, candidate, u, -a - b)) continue;
                lf = a; rf = b; common = u; extra = candidate;
                return;
            }
        }
        private static Vector3 Actuator(Vector3 pure, float q, Vector3 correction) =>
            q == 0f ? correction : q == 1f ? pure + correction : q * pure + correction;

        // A convex ball cuts a closed interval, possibly [lo>0,hi]. Find its minimum
        // first, then bisect each crossing from the feasible side. Fixed bounded work;
        // no allocations, iterative physics, trace dependence or arbitrary search grid.
        private static bool BallInterval(Vector3 start, Vector3 end, float cap, ref float lo, ref float hi)
        {
            if (!V(start) || !V(end) || !F(cap) || cap <= 0f) return false;
            Vector3 delta = end - start;
            if (!V(delta) || !F(delta.sqrMagnitude)) return false;
            if ((start + lo * delta).magnitude <= cap && (start + hi * delta).magnitude <= cap) return true;
            cap *= 0.999998f; // inward roundoff room only when clipping is necessary
            float nearest = delta.sqrMagnitude > 0f ? Mathf.Clamp(-Vector3.Dot(start, delta) / delta.sqrMagnitude, lo, hi) : lo;
            if (!F(nearest) || (start + nearest * delta).magnitude > cap) return false;
            if ((start + lo * delta).magnitude > cap)
            {
                float outside = lo, inside = nearest;
                for (int i = 0; i < 24; i++)
                {
                    float mid = (outside + inside) * 0.5f;
                    if ((start + mid * delta).magnitude <= cap) inside = mid; else outside = mid;
                }
                lo = inside;
            }
            if ((start + hi * delta).magnitude > cap)
            {
                float inside = nearest, outside = hi;
                for (int i = 0; i < 24; i++)
                {
                    float mid = (outside + inside) * 0.5f;
                    if ((start + mid * delta).magnitude <= cap) inside = mid; else outside = mid;
                }
                hi = inside;
            }
            return lo <= hi;
        }
        private static bool MotionRequest(HipAnchorPairInput left, HipAnchorPairInput right, float motion,
            out Vector3 lf, out Vector3 rf, out Vector3 common, float upwardExtra = 0f)
        {
            Vector3 systemA = HipSystemAllocation.Target(left.Seed.ParentInput, motion, upwardExtra);
            Vector3 reaction = Reaction(left, right, motion);
            lf = left.Seed.ChildMass * (left.Seed.FeedbackA - PointBias(left, motion, reaction) + systemA);
            rf = right.Seed.ChildMass * (right.Seed.FeedbackA - PointBias(right, motion, reaction) + systemA);
            common = systemA + (lf + rf) / left.Seed.ParentInput.Response.TotalMass;
            return V(lf) && V(rf) && V(common);
        }
        // Upper pure angular point A minus parent point A relative to the P+C COM.
        // Receiver attitude compensation keeps this part independent of the Hip forces.
        private static Vector3 PointBias(HipAnchorPairInput input, float motion, Vector3 reaction)
        {
            PelvisHipCommandFrame c = input.Seed.ParentInput;
            Vector3 parentTorque = c.JointTorqueReaction ? c.ReceiverTorque + reaction : c.ReceiverTorque;
            Vector3 alpha = c.Response.PredictAngular(Vector3.zero, parentTorque, c.ParentW);
            Vector3 parentA = c.Response.PredictCom(Vector3.zero, alpha, c.ParentW) +
                Vector3.Cross(alpha, input.Seed.ParentPoint - input.Seed.ParentCom);
            Vector3 pureAlpha = HipPointPairMobility.InverseTensor(input.ChildInertia, input.ChildPrincipal, motion * input.PureTorque);
            return Vector3.Cross(pureAlpha, input.Seed.ChildPoint - input.Seed.ChildCom) - parentA;
        }
        private static HipAnchorPairReport.Command Compose(HipAnchorPairInput input, Vector3 raw, Vector3 rawPeer, Vector3 force, Vector3 peer,
            Vector3 damping, Vector3 common, float scale, float pointScale, float correctionScale, float actuatorScale, float receiverScale, float commonScale, float totalPower, float motion, float angularScale, float upwardExtra, bool feasible, Vector3 reaction)
        {
            HipAnchorPairReport.Command c = input.Seed;
            c.Angular = input.Angular;
            c.ParentAngularReaction = c.ParentInput.JointTorqueReaction ? -angularScale * input.Angular.RelativeTorque : Vector3.zero;
            c.BilateralAngularReaction = reaction;
            c.JointSolved = true; c.Force = force; c.UnboundedForce = raw; c.PeerForce = peer; c.PureHipTorque = input.PureTorque;
            c.Scale = scale; c.PointBudgetRatio = pointScale; c.UpperCorrectionBudgetRatio = correctionScale; c.UpperActuatorBudgetRatio = actuatorScale; c.ReceiverBudgetRatio = receiverScale;
            c.CommonBudgetRatio = commonScale;
            c.ChildTorque = Vector3.Cross(c.ChildPoint - c.ChildCom, force); c.ParentTorque = Vector3.Cross(c.ParentPoint - c.ParentCom, -force);
            c.UpperCorrectionTorque = -c.ChildTorque; c.NetHipTorque = angularScale * input.PureTorque; c.CommonAcceleration = common;
            c.MotionScale = motion; c.UpperAngularScale = angularScale; c.UpwardExtraAcceleration = upwardExtra; c.FullFeedbackFeasible = feasible;
            c.AllocatedHipAlpha = input.DesiredAlpha + HipPointPairMobility.InverseTensor(input.ChildInertia, input.ChildPrincipal, c.NetHipTorque - input.PureTorque);
            c.UpperActuatorTorque = c.NetHipTorque + c.UpperCorrectionTorque;
            c.UpperCorrectionAlpha = HipPointPairMobility.InverseTensor(input.ChildInertia, input.ChildPrincipal, c.UpperCorrectionTorque);
            c.UpperActuatorLimitNm = ActuatorLimit(input.PureTorque);
            c.GeometryMoment = Vector3.Cross(c.ChildPoint - c.ParentPoint, force); c.ParentResponseAlpha = ParentAlpha(input, force);
            c.DesiredA = ForceResponse(input, raw, rawPeer); c.AppliedA = ForceResponse(input, force, peer);
            c.KnownBiasA = PointBias(input, angularScale, reaction) - common;
            c.PredictedNetA = c.AppliedA + c.KnownBiasA;
            c.FeedbackResidualA = c.PredictedNetA - c.FeedbackA;
            c.DampingPower = Vector3.Dot(damping, -c.VelocityError); c.BilateralDampingPower = totalPower;
            c.PairPower = Vector3.Dot(force, -c.VelocityError);
            return c;
        }
        private static Vector3 Reaction(HipAnchorPairInput left, HipAnchorPairInput right, float motion) =>
            left.Seed.ParentInput.JointTorqueReaction ?
                (-motion * left.Angular.RelativeTorque) + (-motion * right.Angular.RelativeTorque) : Vector3.zero;
        private static Vector3 ForceResponse(HipAnchorPairInput input, Vector3 force, Vector3 peer) =>
            force / input.Seed.ChildMass + (force + peer) / input.Seed.ParentInput.Response.TotalMass;
        // Feasible scales on a line from a feasible baseline form [0,s]. The torque
        // magnitude need not increase monotonically (correction may oppose baseline).
        // Fixed bisection avoids a cancellation-prone quadratic root, including cap contact.
        private static float TorqueScale(Vector3 baseline, Vector3 delta, float cap)
        {
            if (!V(baseline) || !V(delta) || !F(cap) || cap <= 0f || baseline.magnitude > cap || !V(baseline + delta)) return -1f;
            if ((baseline + delta).magnitude <= cap) return 1f;
            float lo = 0f, hi = 1f;
            for (int i = 0; i < 20; i++)
            {
                float mid = (lo + hi) * 0.5f;
                if ((baseline + mid * delta).magnitude <= cap) lo = mid;
                else hi = mid;
            }
            return lo * 0.999999f; // inward roundoff headroom, never a larger cap
        }
        private static Vector3 ParentAlpha(HipAnchorPairInput input, Vector3 force) =>
            input.Seed.ParentInput.Response.AngularResponse(-force, Vector3.Cross(input.Seed.ParentPoint - input.Seed.ParentCom, -force));
        private static float Bound(float cap, float a, float b) => Mathf.Min(1f, cap / Mathf.Max(0.000001f, Mathf.Max(a, b)));
        private static bool V(Vector3 v) => HipPointPairMobility.V(v);
        private static bool F(float x) => HipPointPairMobility.F(x);
    }
}
