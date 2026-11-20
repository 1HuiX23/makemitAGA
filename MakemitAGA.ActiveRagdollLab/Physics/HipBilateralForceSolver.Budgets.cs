using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static partial class HipBilateralForceSolver
    {
        // Value-only constraint vectors shared by the uniform, same-q and bounded-trade intervals.
        // Original order, arithmetic and limits are retained; no per-step arrays.
        private struct BudgetBall
        {
            internal Vector3 Start, End;
            internal float Cap;
            internal BudgetBall(Vector3 start, Vector3 end, float cap) { Start = start; End = end; Cap = cap; }
        }
        private struct BudgetLine
        {
            internal BudgetBall Common, LeftPoint, RightPoint, LeftCorrection, RightCorrection;
            internal BudgetBall LeftActuator, RightActuator, Receiver, External;
            internal int Count;
            internal BudgetBall At(int index) => index switch {
                0 => Common, 1 => LeftPoint, 2 => RightPoint, 3 => LeftCorrection, 4 => RightCorrection,
                5 => LeftActuator, 6 => RightActuator, 7 => Receiver, _ => External
            };
        }
        private static bool TryBudgetLine(HipAnchorPairInput l, HipAnchorPairInput r, float q0, float q1,
            Vector3 lf0, Vector3 rf0, Vector3 lf1, Vector3 rf1, Vector3 common0, Vector3 common1, out BudgetLine line)
        {
            line = default;
            Vector3 lu0 = -Vector3.Cross(l.Seed.ChildPoint - l.Seed.ChildCom, lf0),
                ru0 = -Vector3.Cross(r.Seed.ChildPoint - r.Seed.ChildCom, rf0),
                lu1 = -Vector3.Cross(l.Seed.ChildPoint - l.Seed.ChildCom, lf1),
                ru1 = -Vector3.Cross(r.Seed.ChildPoint - r.Seed.ChildCom, rf1);
            Vector3 m0 = Vector3.Cross(l.Seed.ParentPoint - l.Seed.ParentCom, -lf0) + Vector3.Cross(r.Seed.ParentPoint - r.Seed.ParentCom, -rf0),
                m1 = Vector3.Cross(l.Seed.ParentPoint - l.Seed.ParentCom, -lf1) + Vector3.Cross(r.Seed.ParentPoint - r.Seed.ParentCom, -rf1);
            if (!PelvisHipAttitudeCommand.TryPlanWithReaction(l.Seed.ParentInput, -lf0 - rf0, m0, Reaction(l, r, q0), out var p0) ||
                !PelvisHipAttitudeCommand.TryPlanWithReaction(l.Seed.ParentInput, -lf1 - rf1, m1, Reaction(l, r, q1), out var p1)) return false;
            line = new BudgetLine {
                Common = new BudgetBall(common0, common1, PelvisHipCommandFrame.MaxMagnitude),
                LeftPoint = new BudgetBall(ForceResponse(l, lf0, rf0), ForceResponse(l, lf1, rf1), HipCharacterJointResponseIsolation.TetherMaxAcceleration),
                RightPoint = new BudgetBall(ForceResponse(r, rf0, lf0), ForceResponse(r, rf1, lf1), HipCharacterJointResponseIsolation.TetherMaxAcceleration),
                LeftCorrection = new BudgetBall(lu0, lu1, UpperCorrectionTorqueLimit),
                RightCorrection = new BudgetBall(ru0, ru1, UpperCorrectionTorqueLimit),
                LeftActuator = new BudgetBall(Actuator(l.PureTorque, q0, lu0), Actuator(l.PureTorque, q1, lu1), ActuatorLimit(l.PureTorque)),
                RightActuator = new BudgetBall(Actuator(r.PureTorque, q0, ru0), Actuator(r.PureTorque, q1, ru1), ActuatorLimit(r.PureTorque)),
                Receiver = new BudgetBall(p0.RequestedTorque, p1.RequestedTorque, PelvisHipAttitudeCommand.TotalTorqueLimit),
                External = new BudgetBall(p0.ExternalRequestedTorque, p1.ExternalRequestedTorque, PelvisHipAttitudeCommand.TotalTorqueLimit),
                Count = l.Seed.ParentInput.JointTorqueReaction ? 9 : 8
            };
            return true;
        }
        private static bool BudgetInterval(HipAnchorPairInput l, HipAnchorPairInput r, float q0, float q1,
            Vector3 lf0, Vector3 rf0, Vector3 lf1, Vector3 rf1, Vector3 common0, Vector3 common1, ref float lo, ref float hi)
        {
            if (!TryBudgetLine(l, r, q0, q1, lf0, rf0, lf1, rf1, common0, common1, out var line)) return false;
            for (int i = 0; i < line.Count; i++)
            {
                BudgetBall ball = line.At(i);
                if (!BallInterval(ball.Start, ball.End, ball.Cap, ref lo, ref hi)) return false;
            }
            return true;
        }

        // Preserve the v129 candidate's independent intersections (including positive
        // lower bounds), then validate the final expression with BudgetInterval.
        private static bool IndependentBudgetInterval(BudgetLine line, out float lo, out float hi)
        {
            lo = 0f; hi = 1f; bool feasible = true;
            for (int i = 0; i < line.Count; i++)
            {
                BudgetBall ball = line.At(i); float a = 0f, b = 1f;
                if (!BallInterval(ball.Start, ball.End, ball.Cap, ref a, ref b)) { feasible = false; continue; }
                lo = Mathf.Max(lo, a); hi = Mathf.Min(hi, b);
            }
            return feasible && lo <= hi;
        }
    }
}
