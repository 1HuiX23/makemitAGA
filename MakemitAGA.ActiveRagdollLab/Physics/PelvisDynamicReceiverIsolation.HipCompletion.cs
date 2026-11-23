using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        internal string HipCompletionIssue { get; private set; }
        internal bool RejectHipCompletion(string reason)
        {
            if (HipCompletionIssue == null) HipCompletionIssue = reason;
            return false;
        }

        // Provenance is captured at the real receiver write, before optional
        // motion evidence and before Hip completion. Never infer it from trace age.
        private void CaptureReceiverTraceInput(PhysicsStepContext step, float dt,
            Vector3 preP, Vector3 preC, Vector3 pForce, Vector3 cForce, Vector3 torque)
        {
            var f = new PelvisReceiverReport.Frame { InputStep = step, Fixed = Report.CurrentTraceFixed, Dt = dt,
                PelvisMass = _pelvis.mass, ChestMass = _chest.mass, PrePelvisV = preP, PreChestV = preC,
                PelvisJ = pForce * dt, ChestJ = cForce * dt, TorqueJ = torque * dt };
            Report.Ring[Report.Next] = f; _pending = Report.Next;
            Report.Next = (Report.Next + 1) % Report.Ring.Length;
            Report.Count = Math.Min(Report.Count + 1, Report.Ring.Length);
        }

        private void CountStep(string phase)
        { Report.Samples++; Report.CurrentTraceFixed++; if (phase == "IDLE") Report.IdleSamples++; else Report.MotionSamples++; }

        // Both real force pairs have returned. The sole receiver owner now adds
        // (bounded total - already committed baseline) torque, once before solve.
        internal bool CompleteHipTranslationReaction(MakemitAGA.ActiveRagdollLab.Runtime.PhysicsStepContext step,
            HipAnchorPairReport left, HipAnchorPairReport right, bool awaitingFootDecision = false)
        {
            try
            {
                if (!IsDynamicHipArm(_control.Arm) || !ReadyForRun || left == null || right == null || !left.Selected || !right.Selected || !left.Finite || !right.Finite ||
                    !left.CurrentApplied || !right.CurrentApplied || !left.Current.Step.Matches(step) || !right.Current.Step.Matches(step) ||
                    left.ParentId != _pelvis.GetInstanceID() || right.ParentId != left.ParentId || left.ChildId == right.ChildId ||
                    left.Current.ChildId != left.ChildId || right.Current.ChildId != right.ChildId ||
                    left.Current.ParentId != left.ParentId || right.Current.ParentId != right.ParentId ||
                    left.Current.Fixed <= 0 || left.Current.Fixed != right.Current.Fixed || !V(left.Current.Force) || !V(right.Current.Force) ||
                    !_hipCommand.Matches(step, left.ParentId) ||
                    !left.Current.ParentInput.Matches(step, left.ParentId) || !right.Current.ParentInput.Matches(step, right.ParentId) ||
                    !left.Current.ParentInput.SameAs(_hipCommand) || !right.Current.ParentInput.SameAs(_hipCommand) ||
                    _hipAttitudeCommand.Committed || !BodyStateMatch() ||
                    !left.Current.JointSolved || !right.Current.JointSolved ||
                    !F(left.Current.MotionScale) || left.Current.MotionScale < 0f || left.Current.MotionScale > 1f ||
                    left.Current.MotionScale != right.Current.MotionScale ||
                    left.Current.UpperAngularScale != right.Current.UpperAngularScale ||
                    !HipSystemAllocation.AngularValid(_hipCommand, left.Current.MotionScale, left.Current.UpperAngularScale, left.Current.UpwardExtraAcceleration) ||
                    left.Current.UpwardExtraAcceleration != right.Current.UpwardExtraAcceleration ||
                    (!left.Current.FullFeedbackFeasible && (left.Current.UpwardExtraAcceleration != 0f || left.Current.UpperAngularScale != left.Current.MotionScale)) ||
                    left.Current.Scale != right.Current.Scale || left.Current.FullFeedbackFeasible != right.Current.FullFeedbackFeasible ||
                    !PelvisChestPointResponse.Same(left.Current.PeerForce, right.Current.Force) ||
                    !PelvisChestPointResponse.Same(right.Current.PeerForce, left.Current.Force) ||
                    !PelvisChestPointResponse.Same(left.Current.CommonAcceleration, right.Current.CommonAcceleration) ||
                    !PelvisChestPointResponse.Same(left.Current.BilateralAngularReaction, right.Current.BilateralAngularReaction) ||
                    !PelvisChestPointResponse.Same(left.Current.BilateralAngularReaction, left.Current.ParentAngularReaction + right.Current.ParentAngularReaction) ||
                    !ReactionMatches(left.Current) || !ReactionMatches(right.Current)) return RejectHipCompletion("INPUT_OR_PAIR_CONTRACT");
                if (!PelvisHipAttitudeCommand.TryPlanWithReaction(_hipCommand, -left.Current.Force - right.Current.Force,
                    left.Current.ParentTorque + right.Current.ParentTorque, left.Current.BilateralAngularReaction, out PelvisHipAttitudeCommand command) || command.Clamped) return RejectHipCompletion("ATTITUDE_PLAN_OR_TORQUE_BUDGET");
                command.MotionScale = left.Current.MotionScale; command.UpperAngularScale = left.Current.UpperAngularScale; command.CommonAcceleration = left.Current.CommonAcceleration;
                command.UpwardExtraAcceleration = left.Current.UpwardExtraAcceleration;
                if (!command.AllocationMatches()) return RejectHipCompletion("SYSTEM_COM_ALLOCATION");
                var f = new PelvisReceiverReport.Frame();
                if (Report.TraceRunning)
                {
                    if (_pending < 0 || _pending >= Report.Ring.Length || !Report.TraceHipTranslation) return RejectHipCompletion("TRACE_PENDING_MISSING");
                    f = Report.Ring[_pending];
                    // Receiver Fixed starts at trace; leg Fixed starts at drive and
                    // includes unloaded preparation. Only the shared context joins them.
                    if (f.Fixed != Report.CurrentTraceFixed || !f.InputStep.Matches(step) || f.Dt != step.Dt)
                        return RejectHipCompletion("TRACE_INPUT_STEP_MISMATCH:receiver=" + f.Fixed +
                            "/leg=" + left.Current.Fixed + "/input=" + f.InputStep.Status() + "/current=" + step.Status());
                    if (f.HipReactionRecorded || f.Resolved || f.Censored)
                        return RejectHipCompletion("TRACE_INPUT_ALREADY_CLOSED");
                }
                f.HipLeftJ = -left.Current.Force * step.Dt; f.HipRightJ = -right.Current.Force * step.Dt;
                f.HipMomentJ = (left.Current.ParentTorque + right.Current.ParentTorque) * step.Dt;
                f.TorqueJ = command.AppliedTorque * step.Dt;
                if (!V(f.HipLeftJ) || !V(f.HipRightJ) || !V(f.HipMomentJ) || !V(f.TorqueJ)) return RejectHipCompletion("IMPULSE_NONFINITE");
                Vector3 deltaA = command.CommonAcceleration - _hipCommand.Acceleration;
                Vector3 pDelta = _pelvis.mass * deltaA, cDelta = _chest.mass * deltaA;
                if (!V(pDelta) || !V(cDelta)) return RejectHipCompletion("DELTA_FORCE_NONFINITE");
                int written = 0;
                try
                {
                    _pelvis.AddForce(pDelta, ForceMode.Force); written = 1;
                    _chest.AddForce(cDelta, ForceMode.Force); written = 2;
                    _pelvis.AddTorque(command.CorrectionTorque, ForceMode.Force);
                }
                catch (Exception e)
                {
                    // Only returned writes are owned. Continue rollback if either native call fails.
                    if (written >= 2) { try { _chest.AddForce(-cDelta, ForceMode.Force); } catch { Report.FaultEvents++; } }
                    if (written >= 1) { try { _pelvis.AddForce(-pDelta, ForceMode.Force); } catch { Report.FaultEvents++; } }
                    RejectHipCompletion("ALLOCATION_WRITE_EXCEPTION:" + e.GetType().Name);
                    return Invalid("RECEIVER_HIP_ALLOCATION_WRITE_EXCEPTION:" + e.GetType().Name);
                }
                f.PelvisJ = _pelvis.mass * command.CommonAcceleration * step.Dt;
                f.ChestJ = _chest.mass * command.CommonAcceleration * step.Dt;
                Report.CommonA = command.CommonAcceleration;
                command.Committed = true; _hipAttitudeCommand = command; _expectHipCompletion = true;
                Report.PelvisTorque = command.AppliedTorque;
                if (!Report.TraceRunning) return true;
                f.HipAttitude = command;
                f.HipReactionRecorded = true;
                Report.Ring[_pending] = f;
                if (!awaitingFootDecision) FinishFootDecision(step);
                return true;
            }
            catch (Exception e)
            {
                RejectHipCompletion("COMPLETION_EXCEPTION:" + e.GetType().Name);
                return Invalid("RECEIVER_HIP_ATTITUDE_COMPLETION_EXCEPTION:" + e.GetType().Name);
            }
        }
        internal void FinishFootDecision(PhysicsStepContext step)
        {
            if (!Report.TraceRunning || _pending < 0) return;
            var f = Report.Ring[_pending];
            if (!f.InputStep.Matches(step) || !f.HipReactionRecorded || f.HipEvidenceFinalized) return;
            var command = f.HipAttitude;
            f.HipEvidenceFinalized = true;
            Report.CaptureMotionInput(ref f, step);
            Report.Ring[_pending] = f; Report.HipReactionRecords++;
            Report.HipAttitudeRequestedPeak = Mathf.Max(Report.HipAttitudeRequestedPeak, command.RequestedTorque.magnitude);
            Report.HipAttitudeAppliedPeak = Mathf.Max(Report.HipAttitudeAppliedPeak, command.AppliedTorque.magnitude);
            Report.HipAttitudeResidualPeak = Mathf.Max(Report.HipAttitudeResidualPeak, command.ResidualTorque.magnitude);
        }

        private void ResolvePending(PhysicsStepContext step)
        {
            if (_pending < 0) return;
            PelvisReceiverReport.Frame f = Report.Ring[_pending];
            if (f.Fixed != Report.CurrentTraceFixed || !f.InputStep.Valid || !step.Valid ||
                f.InputStep.Epoch != step.Epoch || f.InputStep.Sequence + 1 != step.Sequence || f.Dt != f.InputStep.Dt ||
                !BodyStateMatch() || !V(_pelvis.velocity) || !V(_chest.velocity))
            { Invalid("RECEIVER_SETTLED_CLOCK_OR_BODY_INVALID"); EndTrace(); return; }
            f.PostPelvisV = _pelvis.velocity; f.PostChestV = _chest.velocity; f.Settled = f.Fixed + 1;
            f.SystemDeltaP = f.PelvisMass * (f.PostPelvisV - f.PrePelvisV) + f.ChestMass * (f.PostChestV - f.PreChestV);
            if (Report.TraceHipTranslation && !f.HipReactionRecorded) { Invalid("RECEIVER_HIP_INPUT_MISSING"); EndTrace(); return; }
            f.SystemResidual = f.SystemDeltaP - f.PelvisJ - f.ChestJ - f.HipLeftJ - f.HipRightJ;
            if (!V(f.SystemDeltaP) || !V(f.SystemResidual)) { Invalid("RECEIVER_SETTLED_NONFINITE"); EndTrace(); return; }
            f.Resolved = true;
            Report.ResolveMotionInput(ref f, step); // Read-only N -> N+1, before ring replacement.
            Report.Ring[_pending] = f; _pending = -1; Report.Resolved++;
            Report.SystemResidualPeak = Mathf.Max(Report.SystemResidualPeak, f.SystemResidual.magnitude);
        }
        private static bool ReactionMatches(HipAnchorPairReport.Command c) => c.Angular.Valid &&
            PelvisChestPointResponse.Same(c.NetHipTorque, c.UpperAngularScale * c.PureHipTorque) &&
            PelvisChestPointResponse.Same(c.ParentAngularReaction,
                c.ParentInput.JointTorqueReaction ? -c.UpperAngularScale * c.Angular.RelativeTorque : Vector3.zero);
    }
}
