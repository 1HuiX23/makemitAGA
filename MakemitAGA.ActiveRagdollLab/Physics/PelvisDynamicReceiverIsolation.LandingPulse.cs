using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        // Called after both Foot decisions but before PhysX. The Knee and Foot commands
        // already issued this step are the same ones used in the 172 single-solve comparison.
        internal bool ApplyLandingPulse(in PhysicsStepContext step, int floor, HipAnchorTranslationPair left,
            HipAnchorTranslationPair right, bool primaryLeft, in LandingRiseAllocation.Allocation plan)
        {
            if (left == null || right == null || !LandingRiseAllocation.TryTorso(plan.Left,plan.Right,out var target)) return RejectHipCompletion("RISE_PULSE_TORSO_BUDGET");
            var frame = target.Baseline;
            if (!IsDynamicHipArm(_control.Arm) || !ReadyForRun || !_expectHipCompletion || !BodyStateMatch() ||
                !LandingRiseInput.Step.Matches(step) || !LandingRiseInput.Valid || LandingRiseInput.Support.Matches(step) ||
                !LandingRiseInput.WitnessKnown(floor) || !_hipCommand.Matches(step,_pelvis.GetInstanceID()) ||
                _hipCommand.ChestBodyId != _chest.GetInstanceID() || !_hipAttitudeCommand.Matches(_hipCommand) ||
                !left.Report.Current.ParentInput.SameAs(_hipCommand) || !right.Report.Current.ParentInput.SameAs(_hipCommand) ||
                left == right || left.Report.ChildId == right.Report.ChildId ||
                !frame.SameAs(plan.Torso.Baseline) ||
                !frame.SameAs(plan.Right.ParentInput) || plan.Left.Fixed != plan.Right.Fixed ||
                (plan.Kind != LandingRiseAllocation.Choice.Request && plan.Kind != LandingRiseAllocation.Choice.Retention) ||
                frame.UpwardRetentionEligible != (plan.Kind == LandingRiseAllocation.Choice.Retention) ||
                !PelvisChestPointResponse.Same(plan.Left.PeerForce,plan.Right.Force) || !PelvisChestPointResponse.Same(plan.Right.PeerForce,plan.Left.Force) ||
                !PelvisChestPointResponse.Same(target.AngularReaction,plan.Left.ParentAngularReaction+plan.Right.ParentAngularReaction) ||
                !PelvisChestPointResponse.Same(target.CommonAcceleration,plan.Left.CommonAcceleration) ||
                !PelvisChestPointResponse.Same(target.CommonAcceleration,plan.Right.CommonAcceleration) ||
                target.MotionScale != plan.Right.MotionScale ||
                target.UpperAngularScale != plan.Right.UpperAngularScale ||
                target.UpwardExtraAcceleration != plan.Right.UpwardExtraAcceleration ||
                !left.AcceptsPulse(plan.Left,_pelvis,out var l) || !right.AcceptsPulse(plan.Right,_pelvis,out var r))
                return RejectHipCompletion("RISE_PULSE_INPUT_OR_PAIR_CONTRACT");

            var writes = LandingPulseForces.Create(l,r,_pelvis,_chest,left.Report.Current,right.Report.Current,_hipAttitudeCommand,plan.Left,plan.Right,target);
            if (!writes.TryApply(out int rollbackFailures))
            {
                string failure = "RISE_PULSE_WRITE_FAILED_UNDO_FAILURES_" + rollbackFailures;
                RejectHipCompletion(failure);
                if (rollbackFailures != 0)
                {
                    Report.FaultEvents += rollbackFailures;
                    return Invalid(failure);
                }
                return false;
            }
            target.Committed = true;
            left.PublishPulse(plan.Left);
            right.PublishPulse(plan.Right);
            _hipCommand = frame;
            _hipAttitudeCommand = target;
            Report.CommonA = target.CommonAcceleration;
            Report.PelvisTorque = target.AppliedTorque;
            if (Report.TraceRunning && _pending >= 0)
            {
                var f = Report.Ring[_pending];
                // The receiver's historical left/right fields follow primary/secondary order.
                f.HipLeftJ = -(primaryLeft ? plan.Left.Force : plan.Right.Force) * step.Dt;
                f.HipRightJ = -(primaryLeft ? plan.Right.Force : plan.Left.Force) * step.Dt;
                f.HipMomentJ = (plan.Left.ParentTorque + plan.Right.ParentTorque) * step.Dt;
                f.PelvisJ = f.PelvisMass * target.CommonAcceleration * step.Dt;
                f.ChestJ = f.ChestMass * target.CommonAcceleration * step.Dt;
                f.TorqueJ = target.AppliedTorque * step.Dt;
                f.HipAttitude = target;
                Report.Ring[_pending] = f;
            }
            return true;
        }
    }
}
