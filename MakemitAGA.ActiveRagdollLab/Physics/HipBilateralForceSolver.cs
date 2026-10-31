using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Same-solve algebra only. Both Upper point moments and the combined effective
    // Pelvis moment are cancelled, leaving translational diagonal (1/mU+1/M)I
    // and off-diagonal I/M. Receiver attitude and other loads retain their existing owners.
    // First retain full attachment feedback while jointly reducing common COM and
    // pure Upper torque demand. If no such request is feasible, zero those motion
    // requests and bound the remaining attachment force. Never claim feedback was
    // achieved in that fallback. No source pose, reference history or safety gate is reset.
    internal static class HipBilateralForceSolver
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
            float b = 1f / baseline.Response.TotalMass;
            float il = 1f / left.Seed.ChildMass, ir = 1f / right.Seed.ChildMass;
            float a = il + b, d = ir + b, determinant = il * ir + b * (il + ir);
            if (!F(a) || !F(d) || !F(determinant) || determinant <= 0f) return false;
            Vector3 lb = Bias(left, 1f), rb = Bias(right, 1f);
            Vector3 lr = left.Seed.FeedbackA - lb, rr = right.Seed.FeedbackA - rb;
            Vector3 lf = (d * lr - b * rr) / determinant;
            Vector3 rf = (a * rr - b * lr) / determinant;
            if (!V(lb) || !V(rb) || !V(lf) || !V(rf)) return false;
            float dt = left.Seed.Step.Dt, kp = HipCharacterJointResponseIsolation.TetherPositionSpring,
                kd = HipCharacterJointResponseIsolation.TetherVelocityDamper;
            float dampingGain = (kd + kp * dt) / (1f + kd * dt + kp * dt * dt);
            Vector3 ld = dampingGain * left.Seed.VelocityError, rd = dampingGain * right.Seed.VelocityError;
            Vector3 ldf = (d * ld - b * rd) / determinant, rdf = (a * rd - b * ld) / determinant;
            Vector3 lu = -Vector3.Cross(left.Seed.ChildPoint - left.Seed.ChildCom, lf);
            Vector3 ru = -Vector3.Cross(right.Seed.ChildPoint - right.Seed.ChildCom, rf);
            Vector3 la = ForceResponse(left, lf, rf), ra = ForceResponse(right, rf, lf);
            if (!V(ldf) || !V(rdf) || !V(lu) || !V(ru) || !V(la) || !V(ra)) return false;
            float pointScale = Bound(HipCharacterJointResponseIsolation.TetherMaxAcceleration, la.magnitude, ra.magnitude);
            float correctionScale = Bound(UpperCorrectionTorqueLimit, lu.magnitude, ru.magnitude);
            float leftScale = TorqueScale(left.PureTorque, lu, ActuatorLimit(left.PureTorque));
            float rightScale = TorqueScale(right.PureTorque, ru, ActuatorLimit(right.PureTorque));
            if (leftScale < 0f || rightScale < 0f) return false;
            float actuatorScale = Mathf.Min(leftScale, rightScale);
            Vector3 moment = Vector3.Cross(left.Seed.ParentPoint - left.Seed.ParentCom, -lf) +
                Vector3.Cross(right.Seed.ParentPoint - right.Seed.ParentCom, -rf);
            if (!PelvisHipAttitudeCommand.TryPlan(baseline, -lf - rf, moment, out PelvisHipAttitudeCommand unbounded)) return false;
            float receiverScale = TorqueScale(baseline.ReceiverTorque, -unbounded.EffectiveHipTorque, PelvisHipAttitudeCommand.TotalTorqueLimit);
            if (receiverScale < 0f) return false;
            // Full-feedback force is affine in the common motion factor q. Intersect
            // all balls on that line; q=0 need not be feasible (do not binary-search
            // the whole allocation under a false monotonicity assumption).
            Vector3 lb0 = Bias(left, 0f), rb0 = Bias(right, 0f);
            Vector3 lf0 = (d * (left.Seed.FeedbackA - lb0) - b * (right.Seed.FeedbackA - rb0)) / determinant;
            Vector3 rf0 = (a * (right.Seed.FeedbackA - rb0) - b * (left.Seed.FeedbackA - lb0)) / determinant;
            if (!V(lb0) || !V(rb0) || !V(lf0) || !V(rf0)) return false;
            float lo = 0f, hi = 1f;
            bool feasible = MotionInterval(left, right, lf0, rf0, lf, rf, ref lo, ref hi);
            float motion = feasible ? hi : 0f;
            lb = Bias(left, motion); rb = Bias(right, motion);
            lr = left.Seed.FeedbackA - lb; rr = right.Seed.FeedbackA - rb;
            lf = (d * lr - b * rr) / determinant; rf = (a * rr - b * lr) / determinant;
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
                scale = Mathf.Min(Bound(HipCharacterJointResponseIsolation.TetherMaxAcceleration,
                    ForceResponse(left, lf, rf).magnitude, ForceResponse(right, rf, lf).magnitude),
                    Mathf.Min(Bound(UpperCorrectionTorqueLimit, lu.magnitude, ru.magnitude), receiver));
                if (scale < 1f) scale *= 0.999998f;
            }
            float dampingPower = scale * (Vector3.Dot(ldf, -left.Seed.VelocityError) + Vector3.Dot(rdf, -right.Seed.VelocityError));
            if (!F(scale) || scale < 0f || scale > 1f || !F(dampingPower) || dampingPower > 0.0001f) return false;
            Vector3 actualL = lf * scale, actualR = rf * scale;
            moment = Vector3.Cross(left.Seed.ParentPoint - left.Seed.ParentCom, -actualL) +
                Vector3.Cross(right.Seed.ParentPoint - right.Seed.ParentCom, -actualR);
            if (!PelvisHipAttitudeCommand.TryPlan(baseline, -actualL - actualR, moment, out PelvisHipAttitudeCommand final) || final.Clamped) return false;
            lc = Compose(left, lf, rf, actualL, actualR, ldf * scale, lb, scale, pointScale, correctionScale, actuatorScale, receiverScale, dampingPower, motion, feasible);
            rc = Compose(right, rf, lf, actualR, actualL, rdf * scale, rb, scale, pointScale, correctionScale, actuatorScale, receiverScale, dampingPower, motion, feasible);
            float le = (lc.PredictedNetA - (lb + scale * lr)).magnitude,
                re = (rc.PredictedNetA - (rb + scale * rr)).magnitude;
            return V(lc.PredictedNetA) && V(rc.PredictedNetA) && V(lc.UpperActuatorTorque) && V(rc.UpperActuatorTorque) &&
                V(lc.AllocatedHipAlpha) && V(rc.AllocatedHipAlpha) && F(le) && F(re) &&
                le <= 0.001f * (1f + lr.magnitude) && re <= 0.001f * (1f + rr.magnitude);
        }
        private static bool MotionInterval(HipAnchorPairInput l, HipAnchorPairInput r, Vector3 lf0, Vector3 rf0,
            Vector3 lf1, Vector3 rf1, ref float lo, ref float hi)
        {
            Vector3 lu0 = -Vector3.Cross(l.Seed.ChildPoint - l.Seed.ChildCom, lf0),
                ru0 = -Vector3.Cross(r.Seed.ChildPoint - r.Seed.ChildCom, rf0),
                lu1 = -Vector3.Cross(l.Seed.ChildPoint - l.Seed.ChildCom, lf1),
                ru1 = -Vector3.Cross(r.Seed.ChildPoint - r.Seed.ChildCom, rf1);
            Vector3 m0 = Vector3.Cross(l.Seed.ParentPoint - l.Seed.ParentCom, -lf0) + Vector3.Cross(r.Seed.ParentPoint - r.Seed.ParentCom, -rf0),
                m1 = Vector3.Cross(l.Seed.ParentPoint - l.Seed.ParentCom, -lf1) + Vector3.Cross(r.Seed.ParentPoint - r.Seed.ParentCom, -rf1);
            if (!PelvisHipAttitudeCommand.TryPlan(l.Seed.ParentInput, -lf0 - rf0, m0, out PelvisHipAttitudeCommand p0) ||
                !PelvisHipAttitudeCommand.TryPlan(l.Seed.ParentInput, -lf1 - rf1, m1, out PelvisHipAttitudeCommand p1)) return false;
            return BallInterval(ForceResponse(l, lf0, rf0), ForceResponse(l, lf1, rf1), HipCharacterJointResponseIsolation.TetherMaxAcceleration, ref lo, ref hi) &&
                BallInterval(ForceResponse(r, rf0, lf0), ForceResponse(r, rf1, lf1), HipCharacterJointResponseIsolation.TetherMaxAcceleration, ref lo, ref hi) &&
                BallInterval(lu0, lu1, UpperCorrectionTorqueLimit, ref lo, ref hi) &&
                BallInterval(ru0, ru1, UpperCorrectionTorqueLimit, ref lo, ref hi) &&
                BallInterval(lu0, l.PureTorque + lu1, ActuatorLimit(l.PureTorque), ref lo, ref hi) &&
                BallInterval(ru0, r.PureTorque + ru1, ActuatorLimit(r.PureTorque), ref lo, ref hi) &&
                BallInterval(p0.RequestedTorque, p1.RequestedTorque, PelvisHipAttitudeCommand.TotalTorqueLimit, ref lo, ref hi);
        }
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
        private static Vector3 Bias(HipAnchorPairInput input, float motion)
        {
            PelvisHipCommandFrame c = input.Seed.ParentInput;
            Vector3 alpha = c.Response.PredictAngular(Vector3.zero, c.ReceiverTorque, c.ParentW);
            Vector3 parentA = motion * c.Acceleration + c.Response.PredictCom(Vector3.zero, alpha, c.ParentW) +
                Vector3.Cross(alpha, input.Seed.ParentPoint - input.Seed.ParentCom);
            Vector3 pureAlpha = HipPointPairMobility.InverseTensor(input.ChildInertia, input.ChildPrincipal, motion * input.PureTorque);
            return Vector3.Cross(pureAlpha, input.Seed.ChildPoint - input.Seed.ChildCom) - parentA;
        }
        private static HipAnchorPairReport.Command Compose(HipAnchorPairInput input, Vector3 raw, Vector3 rawPeer, Vector3 force, Vector3 peer,
            Vector3 damping, Vector3 bias, float scale, float pointScale, float correctionScale, float actuatorScale, float receiverScale, float totalPower, float motion, bool feasible)
        {
            HipAnchorPairReport.Command c = input.Seed;
            c.JointSolved = true; c.Force = force; c.UnboundedForce = raw; c.PeerForce = peer; c.PureHipTorque = input.PureTorque;
            c.Scale = scale; c.PointScale = pointScale; c.UpperCorrectionScale = correctionScale; c.UpperActuatorScale = actuatorScale; c.ReceiverScale = receiverScale;
            c.ChildTorque = Vector3.Cross(c.ChildPoint - c.ChildCom, force); c.ParentTorque = Vector3.Cross(c.ParentPoint - c.ParentCom, -force);
            c.UpperCorrectionTorque = -c.ChildTorque; c.NetHipTorque = motion * input.PureTorque; c.CommonAcceleration = motion * c.ParentInput.Acceleration;
            c.MotionScale = motion; c.FullFeedbackFeasible = feasible;
            c.AllocatedHipAlpha = input.DesiredAlpha + HipPointPairMobility.InverseTensor(input.ChildInertia, input.ChildPrincipal, c.NetHipTorque - input.PureTorque);
            c.UpperActuatorTorque = c.NetHipTorque + c.UpperCorrectionTorque;
            c.UpperCorrectionAlpha = HipPointPairMobility.InverseTensor(input.ChildInertia, input.ChildPrincipal, c.UpperCorrectionTorque);
            c.UpperActuatorLimitNm = ActuatorLimit(input.PureTorque);
            c.GeometryMoment = Vector3.Cross(c.ChildPoint - c.ParentPoint, force); c.ParentResponseAlpha = ParentAlpha(input, force);
            c.DesiredA = ForceResponse(input, raw, rawPeer); c.AppliedA = ForceResponse(input, force, peer);
            c.KnownBiasA = bias;
            c.PredictedNetA = c.AppliedA + bias;
            c.FeedbackResidualA = c.PredictedNetA - c.FeedbackA;
            c.DampingPower = Vector3.Dot(damping, -c.VelocityError); c.BilateralDampingPower = totalPower;
            c.PairPower = Vector3.Dot(force, -c.VelocityError);
            return c;
        }
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
