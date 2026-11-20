using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // The two original leg inputs already contain their measured points, velocities, inertia and
    // independent torque requests. Substitute only the candidate common translation into value copies.
    internal static class ParentHeightAllocation
    {
        internal enum Status { Missing, BaselineMismatch, PositionGate, SolveRejected, Measured, Fault }
        internal struct Result
        {
            internal Status State;
            internal float BaseRequestY, TrialRequestY, BaseAllocatedY, TrialAllocatedY;
            internal float RightForceDeltaY, RightForceDeltaNorm;
        }

        internal static Result Compare(in ParentHeightProbe.Input input, in PelvisComReferencePlanner.ComPlan plan,
            HipAnchorPairInput left, HipAnchorPairInput right, in HipAnchorPairReport.Command actualLeft,
            in HipAnchorPairReport.Command actualRight)
        {
            var result = new Result();
            var step = input.Step;
            var original = left.Seed.ParentInput;
            if (!input.ReceiverReady || !left.Ready || !right.Ready || !left.Seed.Step.Matches(step) ||
                !right.Seed.Step.Matches(step) || !actualLeft.Step.Matches(step) || !actualRight.Step.Matches(step) ||
                actualLeft.ChildId != left.Seed.ChildId || actualRight.ChildId != right.Seed.ChildId ||
                !original.SameAs(right.Seed.ParentInput) || !original.SameAs(actualLeft.ParentInput) ||
                !original.SameAs(actualRight.ParentInput) || !PelvisParentFrame.V(input.PositionError) ||
                !PelvisParentFrame.V(input.VelocityError) || !PelvisParentFrame.V(input.ActualCom) ||
                !PelvisParentFrame.F(input.PositionLimit) || input.PositionLimit <= 0f) return result;
            Vector3 baseRaw = PelvisComFeedback.Evaluate(input.PositionError, input.VelocityError, true) +
                (input.CommittedV - input.Velocity) / step.Dt;
            // Both plans have the same support floor and support selection, the only plan fields
            // consumed by the v137 recovery envelope. Check against the request actually committed.
            var recovery = PelvisDynamicReceiverIsolation.BuildCommonAcceleration(input.Arm, baseRaw,
                input.ActualCom, input.SettledV, step, plan);
            if ((recovery.Request - original.Acceleration).magnitude > .0001f)
            { result.State = Status.BaselineMismatch; return result; }
            // Require the untouched local copies to reproduce both actual point forces and
            // the original system target. A mixed or asymmetric receipt cannot serve as baseline.
            if (!HipBilateralForceSolver.TryPlan(left, right, out var baseLeft, out var baseRight) ||
                !Same(baseLeft.Force, actualLeft.Force) || !Same(baseRight.Force, actualRight.Force) ||
                !Same(HipSystemAllocation.Target(original, baseLeft.MotionScale, baseLeft.UpwardExtraAcceleration),
                    HipSystemAllocation.Target(original, actualLeft.MotionScale, actualLeft.UpwardExtraAcceleration)))
            { result.State = Status.BaselineMismatch; return result; }
            Vector3 error = input.PositionError + (plan.Position - input.CommittedPosition);
            if (error.magnitude > input.PositionLimit) { result.State = Status.PositionGate; return result; }
            Vector3 raw = PelvisComFeedback.Evaluate(error, input.VelocityError + (plan.Velocity - input.CommittedV), true) + plan.Acceleration;
            var trial = original;
            trial.Acceleration = PelvisDynamicReceiverIsolation.BuildCommonAcceleration(input.Arm, raw,
                input.ActualCom, input.SettledV, step, plan).Request;
            trial.UpwardRetentionEligible = HipSystemAllocation.Eligible(trial.UpwardRetentionSelected, step, step, plan, error.y, trial.Acceleration);
            left.Seed.ParentInput = right.Seed.ParentInput = trial;
            if (!HipBilateralForceSolver.TryPlan(left, right, out var lc, out var rc))
            { result.State = Status.SolveRejected; return result; }
            result.State = Status.Measured;
            result.BaseRequestY = original.Acceleration.y; result.TrialRequestY = trial.Acceleration.y;
            result.BaseAllocatedY = HipSystemAllocation.Target(original, actualLeft.MotionScale, actualLeft.UpwardExtraAcceleration).y;
            result.TrialAllocatedY = HipSystemAllocation.Target(trial, lc.MotionScale, lc.UpwardExtraAcceleration).y;
            Vector3 rightDelta = rc.Force - actualRight.Force;
            result.RightForceDeltaY = rightDelta.y; result.RightForceDeltaNorm = rightDelta.magnitude;
            return result;
        }

        private static bool Same(Vector3 a, Vector3 b) => PelvisParentFrame.V(a) && PelvisParentFrame.V(b) &&
            (a - b).magnitude <= .0001f * (1f + Mathf.Max(a.magnitude, b.magnitude));
    }
}
