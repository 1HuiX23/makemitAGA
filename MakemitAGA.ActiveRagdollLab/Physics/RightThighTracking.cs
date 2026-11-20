using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Same PRE_N pose and staged bilateral inputs for both arms of the comparison.
    // A projection covers one Fixed interval. Only the original command reaches PhysX.
    internal static class RightThighTracking
    {
        internal enum Status : byte { Missing, Protected, BaselineMismatch, Geometry, NoRoom, Budget, BelowGain, Tradeoff, Improved, Fault }
        internal struct Motion
        {
            internal float HipMaxA, ReferenceMaxA, ReferenceMaxW, UpperMaxW, LowerMaxW, FootMaxW;
            internal Vector3 LowerW, FootW, KneeRelativeA, KneeGapV, AnkleGapV;
        }
        internal struct Forecast
        {
            internal Vector3 Heights; // world knee Y, world lower-side ankle Y, signed lowest corner above the plane
            internal float Angle;     // radians to the held upper goal
            internal bool Inside;
        }
        internal struct Result
        {
            internal Status State;
            internal SwingKneeCandidate.HipPlacement Path;
            internal bool HasBase, HasTrial;
            internal Forecast Baseline, Trial;
            internal Vector3 Gain;
            internal float Fraction, AddedA, BaseTorque, TrialTorque, TorqueLimit, LeftForceDelta, ReceiverReactionDelta, CommonDelta;
            internal float BaseRequestA, TrialRequestA, BaseAllocatedA, TrialAllocatedA, BaseNextW, TrialNextW, MaxCommandA, MaxSpeed;
            internal int Tried, BudgetRejected, GeometryRejected;
        }
        internal static bool Same(Vector3 a, Vector3 b) => PelvisParentFrame.V(a) && PelvisParentFrame.V(b) &&
            (a - b).magnitude <= .0001f * (1f + Mathf.Max(a.magnitude, b.magnitude));

        private static bool Matches(in HipAnchorPairReport.Command plan, in HipAnchorPairReport.Command applied) =>
            plan.JointSolved && applied.JointSolved && plan.Step.Matches(applied.Step) && plan.Fixed == applied.Fixed &&
            plan.ChildId == applied.ChildId && plan.ParentId == applied.ParentId && plan.ParentInput.SameAs(applied.ParentInput) &&
            plan.Angular.SameAs(applied.Angular) && Same(plan.PureHipTorque, applied.PureHipTorque) &&
            Same(plan.Force, applied.Force) && Same(plan.UpperActuatorTorque, applied.UpperActuatorTorque) &&
            Same(plan.AllocatedHipAlpha, applied.AllocatedHipAlpha) && Same(plan.CommonAcceleration, applied.CommonAcceleration) &&
            Same(plan.ParentAngularReaction, applied.ParentAngularReaction) && Same(plan.BilateralAngularReaction, applied.BilateralAngularReaction) &&
            Mathf.Abs(plan.MotionScale - applied.MotionScale) <= .00001f && Mathf.Abs(plan.UpperAngularScale - applied.UpperAngularScale) <= .00001f &&
            Mathf.Abs(plan.UpwardExtraAcceleration - applied.UpwardExtraAcceleration) <= .0001f &&
            Mathf.Abs(plan.UpperActuatorLimitNm - applied.UpperActuatorLimitNm) <= .0001f;

        private static bool Positive(float f) => PelvisParentFrame.F(f) && f > 0f;
        private static bool Valid(in Motion m) => Positive(m.HipMaxA) && Positive(m.ReferenceMaxA) && Positive(m.ReferenceMaxW) &&
            Positive(m.UpperMaxW) && Positive(m.LowerMaxW) && Positive(m.FootMaxW) && PelvisParentFrame.V(m.LowerW) &&
            PelvisParentFrame.V(m.FootW) && PelvisParentFrame.V(m.KneeRelativeA) && PelvisParentFrame.V(m.KneeGapV) && PelvisParentFrame.V(m.AnkleGapV);

        internal static void ProjectAnchors(in RightKneePivot.Frame p, in Motion m, in HipAnchorPairInput right, in HipAnchorPairReport.Command c,
            out Vector3 knee, out Vector3 ankle, out Vector3 foot, out Quaternion upperTurn)
        {
            float dt = p.Dt;
            upperTurn = FootRelativeReference.Advance(Quaternion.identity, Vector3.ClampMagnitude(right.ChildW + c.AllocatedHipAlpha * dt, m.UpperMaxW), dt);
            var lowerTurn = FootRelativeReference.Advance(Quaternion.identity, Vector3.ClampMagnitude(m.LowerW + (m.KneeRelativeA + c.AllocatedHipAlpha) * dt, m.LowerMaxW), dt);
            var com = c.ChildCom + (right.ChildComV + c.Force * (dt / c.ChildMass)) * dt;
            knee = com + upperTurn * (p.UpperKnee - c.ChildCom) + (p.LowerKnee - p.UpperKnee) + m.KneeGapV * dt;
            ankle = knee + lowerTurn * (p.LowerAnkle - p.LowerKnee);
            foot = ankle + (p.FootAnkle - p.LowerAnkle) + m.AnkleGapV * dt;
        }

        private static Forecast Project(in RightKneePivot.Frame p, in Motion m, in HipAnchorPairInput right, in HipAnchorPairReport.Command c)
        {
            ProjectAnchors(p, m, right, c, out var knee, out var ankle, out var foot, out var upperTurn);
            float dt = p.Dt;
            // Foot orientation coasts identically in both forecasts. Gap velocities are native anchor differences at PRE_N.
            var footR = FootRelativeReference.Advance(p.FootR, Vector3.ClampMagnitude(m.FootW, m.FootMaxW), dt);
            return new Forecast {
                Heights = new Vector3(knee.y, ankle.y, FootBoxGeometry.Minimum(foot, footR, p.Center, p.Half) - (p.FloorCenter.y + p.FloorHalf.y)),
                Angle = FootRelativeReference.Log(p.GoalR * Quaternion.Inverse(upperTurn * p.UpperR)).magnitude,
                Inside = FootBoxGeometry.Inside(foot, footR, p.Center, p.Half, p.FloorCenter, p.FloorHalf)
            };
        }
        // Distance along +axis before an existing acceleration/speed ball would be left.
        private static float Room(Vector3 start, Vector3 axis, float limit)
        {
            float slack = limit * limit - start.sqrMagnitude, along = Vector3.Dot(start, axis);
            if (slack < -.000001f * (1f + limit * limit)) return 0f;
            return Mathf.Max(0f, -along + Mathf.Sqrt(along * along + Mathf.Max(0f, slack)));
        }
        internal static bool TryStageRelative(in HipAnchorPairInput right, Vector3 delta, float limit, float originalTorqueLimit, out HipAnchorPairInput trial)
        {
            trial = right;
            trial.DesiredAlpha += delta;
            trial.PureTorque += HipAngularRequest.Tensor(right.ChildInertia, HipAngularRequest.UnitPrincipal(right.ChildPrincipal), delta);
            if (trial.DesiredAlpha.magnitude > limit + .0001f || HipBilateralForceSolver.ActuatorLimit(trial.PureTorque) > originalTorqueLimit + .0001f) return false;
            // Only relative control owns the supplement. The existing already-bounded transport and gyro survive intact.
            return HipAngularRequest.TryCreate(right.ChildInertia, right.ChildPrincipal, right.ChildW, trial.PureTorque, trial.DesiredAlpha,
                right.Angular.RelativeAlpha + delta, right.Angular.TransportAlpha, trial.DesiredAlpha, limit, out trial.Angular) &&
                Same(trial.Angular.TransportTorque, right.Angular.TransportTorque) && Same(trial.Angular.GyroTorque, right.Angular.GyroTorque);
        }
        private static float Depth(float clearance) => Mathf.Max(0f, -clearance);
        internal static Result Compare(in RightKneePivot.Frame p, in Motion m, in HipAnchorPairInput left, in HipAnchorPairInput right,
            in HipAnchorPairReport.Command actualLeft, in HipAnchorPairReport.Command actualRight)
        {
            Result result = default;
            if (!left.Ready || !right.Ready || !Valid(m) || !PelvisParentFrame.V(right.ChildW) || !PelvisParentFrame.V(right.ChildComV)) return result;
            if (RightKneePivot.Decompose(p, out _) != RightKneePivot.Capture.Ready || Mathf.Abs(right.Seed.Step.Dt - p.Dt) > .000001f ||
                !Same(right.Seed.ChildPoint, p.UpperHip)) { result.State = Status.Geometry; return result; }
            if (!HipBilateralForceSolver.TryPlan(left, right, out var lc, out var rc) || !Matches(lc, actualLeft) || !Matches(rc, actualRight))
            { result.State = Status.BaselineMismatch; return result; }

            var baseline = Project(p, m, right, actualRight);
            if (!PelvisParentFrame.V(baseline.Heights) || !PelvisParentFrame.F(baseline.Angle)) { result.State = Status.Geometry; return result; }
            result.HasBase = true; result.Baseline = baseline;
            result.BaseTorque = actualRight.UpperActuatorTorque.magnitude; result.TorqueLimit = actualRight.UpperActuatorLimitNm;
            result.BaseRequestA = right.DesiredAlpha.magnitude; result.BaseAllocatedA = actualRight.AllocatedHipAlpha.magnitude;
            result.MaxCommandA = m.HipMaxA; result.MaxSpeed = Mathf.Min(m.ReferenceMaxW, m.UpperMaxW);
            var currentNextW = right.ChildW + actualRight.AllocatedHipAlpha * p.Dt;
            result.BaseNextW = currentNextW.magnitude;
            if (!baseline.Inside) { result.State = Status.Geometry; return result; }
            var arc = FootRelativeReference.Log(p.GoalR * Quaternion.Inverse(p.UpperR));
            float angle = arc.magnitude, dt = p.Dt;
            result.State = Status.NoRoom;
            if (angle <= .000001f) return result;
            var axis = arc / angle;
            float maxW = result.MaxSpeed, maxA = Mathf.Min(m.ReferenceMaxA, m.HipMaxA);
            float targetSpeed = Mathf.Min(angle / dt, Mathf.Min(maxW, Mathf.Sqrt(2f * maxA * angle)));
            float chase = (targetSpeed - Vector3.Dot(currentNextW, axis)) / dt;
            float add = Mathf.Min(chase, Mathf.Min(maxA, Room(right.DesiredAlpha, axis, m.HipMaxA)));
            if (!(add > .00001f)) return result;
            if (currentNextW.magnitude <= maxW)
                add = Mathf.Min(add, Room(currentNextW, axis, maxW) / dt);
            // The shortest-turn torque is affine in the supplement. Bound the interval before sampling fractions,
            // so a small but usable remainder under the original torque ceiling is not skipped.
            var perAlpha = HipAngularRequest.Tensor(right.ChildInertia, HipAngularRequest.UnitPrincipal(right.ChildPrincipal), axis);
            float torquePerAlpha = perAlpha.magnitude;
            if (!Positive(torquePerAlpha)) { result.State = Status.Budget; return result; }
            add = Mathf.Min(add, Room(right.PureTorque, perAlpha / torquePerAlpha, result.TorqueLimit) / torquePerAlpha);
            if (!(add > .00001f)) { result.State = Status.Budget; return result; }
            result.State = Status.Budget;
            float best = float.NegativeInfinity;
            for (int i = 0; i < 3; i++)
            {
                float fraction = i == 0 ? .25f : i == 1 ? .5f : 1f;
                result.Tried++;
                if (!TryStageRelative(right, axis * (add * fraction), m.HipMaxA, result.TorqueLimit, out var trial) ||
                    !HipBilateralForceSolver.TryPlan(left, trial, out var tl, out var tr) ||
                    tr.UpperActuatorTorque.magnitude > result.TorqueLimit + .0001f || tl.UpperActuatorTorque.magnitude > actualLeft.UpperActuatorLimitNm + .0001f ||
                    tr.AllocatedHipAlpha.magnitude > m.HipMaxA + .0001f || (right.ChildW + tr.AllocatedHipAlpha * dt).magnitude > maxW + .0001f)
                { result.BudgetRejected++; continue; }
                var forecast = Project(p, m, right, tr);
                if (!PelvisParentFrame.V(forecast.Heights) || !PelvisParentFrame.F(forecast.Angle) || !forecast.Inside ||
                    forecast.Heights.z > FootContactGeometryLimits.RecoveryMaxGapMeters)
                { result.GeometryRejected++; continue; }
                var gain = new Vector3(
                    Mathf.Abs(baseline.Heights.x - p.GoalKneeY) - Mathf.Abs(forecast.Heights.x - p.GoalKneeY),
                    Mathf.Abs(baseline.Heights.y - p.RawAnkleY) - Mathf.Abs(forecast.Heights.y - p.RawAnkleY),
                    Depth(baseline.Heights.z) - Depth(forecast.Heights.z));
                float worst = Mathf.Min(gain.x, Mathf.Min(gain.y, gain.z));
                bool closes = forecast.Angle < baseline.Angle;
                bool improved = closes && worst >= KneeGroundTarget.MinGain;
                if (!improved && result.HasTrial && worst <= best) continue;
                best = worst;
                result.HasTrial = true; result.Fraction = fraction; result.AddedA = add * fraction;
                result.Trial = forecast; result.Gain = gain; result.TrialTorque = tr.UpperActuatorTorque.magnitude;
                result.TrialRequestA = trial.DesiredAlpha.magnitude; result.TrialAllocatedA = tr.AllocatedHipAlpha.magnitude;
                result.TrialNextW = (right.ChildW + tr.AllocatedHipAlpha * dt).magnitude;
                result.LeftForceDelta = (tl.Force - lc.Force).magnitude;
                result.ReceiverReactionDelta = (tr.BilateralAngularReaction - rc.BilateralAngularReaction).magnitude;
                result.CommonDelta = (tr.CommonAcceleration - rc.CommonAcceleration).magnitude;
                result.State = improved ? Status.Improved : worst < -0.000001f || !closes ? Status.Tradeoff : Status.BelowGain;
                if (improved) break;
            }
            if (!result.HasTrial && result.GeometryRejected > 0) result.State = Status.Geometry;
            return result;
        }
    }
}
