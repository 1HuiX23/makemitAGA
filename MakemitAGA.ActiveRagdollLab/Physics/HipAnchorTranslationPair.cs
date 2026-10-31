using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Only the new typed Hip scope may prepare this owner. No generic console selector.
    internal sealed class HipAnchorTranslationPair
    {
        internal readonly HipAnchorPairReport Report = new HipAnchorPairReport();
        internal readonly HipPointResponseAudit Response = new HipPointResponseAudit();
        internal void ResetTrace() { Report.ResetTrace(); Response.Reset(); }
        private Rigidbody _child, _parent;
        private PhysicsStepContext _lastStep;
        private bool _queued, _staged, _trace;
        private HipAnchorPairInput _input;
        internal bool Off => !Report.Selected && !Report.CurrentApplied && !_staged;
        internal bool Prepare(Rigidbody child, Rigidbody parent)
        {
            if (!Off || child == null || parent == null || child == parent || child.isKinematic || parent.isKinematic || child.useGravity || parent.useGravity) return false;
            if (!V(child.worldCenterOfMass) || !V(parent.worldCenterOfMass) ||
                !HipPointPairMobility.TryCreate(child.mass, child.inertiaTensor, child.rotation * child.inertiaTensorRotation, Vector3.zero,
                    parent.mass, parent.inertiaTensor, parent.rotation * parent.inertiaTensorRotation, Vector3.zero, out HipPointPairMobility seed) || !V(seed.X)) return false;
            Report.ChildId = child.GetInstanceID(); Report.ParentId = parent.GetInstanceID();
            Report.Selected = Report.Finite = true; Report.Issue = null; Report.RollbackFailures = 0; Report.Current = new HipAnchorPairReport.Command();
            _child = child; _parent = parent; _lastStep = new PhysicsStepContext(); _queued = _staged = false; _input = new HipAnchorPairInput(); ResetTrace();
            return true;
        }
        internal bool Stage(bool trace, PhysicsStepContext step, int fixedSequence, Rigidbody child, Rigidbody parent,
            Vector3 childPoint, Vector3 parentPoint, Vector3 parentPointVelocity, PelvisHipCommandFrame commonCommand)
        {
            // Arrival at a new physical step retires the previous queued solve; no derivative/history reset on trace.
            if (_lastStep.Valid && step.Valid && step.Epoch == _lastStep.Epoch && step.Sequence > _lastStep.Sequence)
                { _queued = false; Report.CurrentApplied = false; }
            if (trace) Response.Observe(step, fixedSequence, child, parent, childPoint, parentPoint);
            else Response.Stop();
            try
            {
                if (_staged || !Report.Selected || !Report.Finite || !step.Valid || fixedSequence <= 0 || child != _child || parent != _parent ||
                    child == null || parent == null || child.GetInstanceID() != Report.ChildId || parent.GetInstanceID() != Report.ParentId ||
                    child.isKinematic || parent.isKinematic || parent.useGravity ||
                    (_lastStep.Valid && (step.Epoch != _lastStep.Epoch || step.Sequence != _lastStep.Sequence + 1 || fixedSequence != Report.Current.Fixed + 1)))
                    return Invalid("HIP_PAIR_BODY_OR_STEP_INVALID");
                if (!commonCommand.Matches(step, Report.ParentId)) return Invalid("HIP_PAIR_PARENT_COMMAND_INVALID");
                Vector3 cc = child.worldCenterOfMass, pc = parent.worldCenterOfMass, cr = childPoint - cc, pr = parentPoint - pc;
                Vector3 cv = child.velocity + Vector3.Cross(child.angularVelocity, cr), pv = parentPointVelocity;
                Vector3 nativePV = parent.velocity + Vector3.Cross(parent.angularVelocity, pr);
                Vector3 e = parentPoint - childPoint, v = pv - cv;
                if (!V(e) || !V(v) || !V(nativePV) || !V(cc) || !V(pc) || !V(cr) || !V(pr)) return Invalid("HIP_PAIR_POINT_NONFINITE");
                if ((pv - nativePV).magnitude > 0.0001f) return Invalid("HIP_PAIR_PARENT_POINT_VELOCITY_MISMATCH");
                if (e.magnitude > HipCharacterJointResponseIsolation.TetherAnchorValidityMeters)
                { Report.RejectedGapM = e.magnitude; return Invalid("HIP_PAIR_30MM_GATE"); }
                Vector3 ci = child.inertiaTensor, pi = parent.inertiaTensor;
                Quaternion cq = child.rotation * child.inertiaTensorRotation, pq = parent.rotation * parent.inertiaTensorRotation;
                if (!commonCommand.Response.MatchesBody(parent.mass, pi, pq)) return Invalid("HIP_PAIR_PARENT_MODEL_NOT_CURRENT");
                if (!HipPointPairMobility.ValidBody(child.mass, ci, cq))
                    return Invalid("HIP_PAIR_POINT_MOBILITY_INVALID");
                float dt = step.Dt, kp = HipCharacterJointResponseIsolation.TetherPositionSpring, kd = HipCharacterJointResponseIsolation.TetherVelocityDamper;
                Vector3 feedbackA = (kp * e + (kd + kp * dt) * v) / (1f + kd * dt + kp * dt * dt);
                if (!V(feedbackA) || !V(child.velocity) || !V(child.angularVelocity)) return Invalid("HIP_PAIR_STAGE_NONFINITE");
                var c = new HipAnchorPairReport.Command
                {
                    Step = step, Fixed = fixedSequence, ChildId = Report.ChildId, ParentId = Report.ParentId,
                    ParentInput = commonCommand, FeedbackA = feedbackA,
                    ChildMass = child.mass, ParentMass = parent.mass, ChildCom = cc, ParentCom = pc, ChildPoint = childPoint, ParentPoint = parentPoint,
                    ChildV = cv, ParentV = pv, Error = e, VelocityError = v
                };
                _input = new HipAnchorPairInput { Valid = true, Seed = c, ChildInertia = ci, ChildPrincipal = cq,
                    ChildComV = child.velocity, ChildW = child.angularVelocity };
                _staged = true; _trace = trace;
                return true;
            }
            catch (Exception e) { return Invalid("HIP_PAIR_STAGE_EXCEPTION:" + e.GetType().Name); }
        }
        internal bool StageHipTorque(PhysicsStepContext step, int localFixed, Vector3 torque, Vector3 desiredAlpha)
        {
            if (!Report.Selected || !Report.Finite || !_staged || !_input.Valid || _input.TorqueRequested || !_input.Seed.Step.Matches(step) ||
                _input.Seed.Fixed != localFixed || !V(torque) || !V(desiredAlpha)) return Invalid("HIP_PAIR_TORQUE_REQUEST_INVALID");
            _input.PureTorque = torque; _input.DesiredAlpha = desiredAlpha; _input.TorqueRequested = true;
            return true;
        }
        internal static bool CommitBilateral(PhysicsStepContext step, HipAnchorTranslationPair left, HipAnchorTranslationPair right)
        {
            if (left == null || right == null || left == right) return false;
            try
            {
                if (!left._staged || !right._staged || !left._input.Seed.Step.Matches(step) || !right._input.Seed.Step.Matches(step)) return false;
                if (!HipBilateralForceSolver.TryPlan(left._input, right._input, out HipAnchorPairReport.Command lc, out HipAnchorPairReport.Command rc))
                    return left.Invalid("HIP_BILATERAL_PLAN_INVALID");
                // Validate BOTH snapshots before any force/actuator write. CurrentApplied
                // means all three writes returned; bilateral fail-close owns their cancellation.
                if (!left.CanCommit(lc) || !right.CanCommit(rc)) return left.Invalid("HIP_BILATERAL_PRECOMMIT_INVALID");
                return left.Commit(lc) && right.Commit(rc);
            }
            catch (Exception e) { return left.Invalid("HIP_BILATERAL_COMMIT_EXCEPTION:" + e.GetType().Name); }
        }
        private bool CanCommit(HipAnchorPairReport.Command c)
        {
            if (!_staged || !Report.Selected || !Report.Finite || !_input.Ready || !c.JointSolved || Report.CurrentApplied ||
                !c.Step.Matches(_input.Seed.Step) || c.Fixed != _input.Seed.Fixed || c.ChildId != Report.ChildId || c.ParentId != Report.ParentId ||
                !c.ParentInput.SameAs(_input.Seed.ParentInput) || !Same(c.PureHipTorque, _input.PureTorque) ||
                !Same(c.ChildCom, _input.Seed.ChildCom) || !Same(c.ParentCom, _input.Seed.ParentCom) ||
                !Same(c.ChildPoint, _input.Seed.ChildPoint) || !Same(c.ParentPoint, _input.Seed.ParentPoint) ||
                _child == null || _parent == null || _child.isKinematic || _parent.isKinematic || _parent.useGravity ||
                _child.GetInstanceID() != Report.ChildId || _parent.GetInstanceID() != Report.ParentId ||
                _child.mass != c.ChildMass || _parent.mass != c.ParentMass ||
                !Same(_child.worldCenterOfMass, c.ChildCom) || !Same(_parent.worldCenterOfMass, c.ParentCom) ||
                !Same(_child.velocity, _input.ChildComV) || !Same(_child.angularVelocity, _input.ChildW) ||
                !Same(_child.inertiaTensor, _input.ChildInertia) ||
                !HipPointPairMobility.ValidBody(_child.mass, _child.inertiaTensor, _child.rotation * _child.inertiaTensorRotation) ||
                Quaternion.Angle(_child.rotation * _child.inertiaTensorRotation, _input.ChildPrincipal) > 0.001f ||
                !c.ParentInput.Response.MatchesBody(_parent.mass, _parent.inertiaTensor, _parent.rotation * _parent.inertiaTensorRotation) ||
                !Same(_parent.angularVelocity, c.ParentInput.ParentW) || !V(_parent.velocity) ||
                (_parent.velocity + Vector3.Cross(_parent.angularVelocity, c.ParentPoint - c.ParentCom) - c.ParentV).magnitude > 0.0001f ||
                !V(c.Force) || !V(c.ChildTorque) || !V(c.ParentTorque) || !V(c.PredictedNetA) || !V(c.FeedbackResidualA) ||
                !V(c.UpperCorrectionTorque) || !V(c.UpperActuatorTorque) || !V(c.UpperCorrectionAlpha) ||
                !Same(c.ChildTorque, Vector3.Cross(c.ChildPoint - c.ChildCom, c.Force)) ||
                !Same(c.UpperCorrectionTorque, -c.ChildTorque) || !Same(c.UpperActuatorTorque, c.NetHipTorque + c.UpperCorrectionTorque) ||
                !F(c.MotionScale) || c.MotionScale < 0f || c.MotionScale > 1f ||
                !Same(c.NetHipTorque, c.PureHipTorque * c.MotionScale) ||
                !Same(c.CommonAcceleration, c.ParentInput.Acceleration * c.MotionScale) || !V(c.AllocatedHipAlpha) ||
                !Same(c.AllocatedHipAlpha, _input.DesiredAlpha + HipPointPairMobility.InverseTensor(_input.ChildInertia, _input.ChildPrincipal, c.NetHipTorque - c.PureHipTorque)) ||
                !Same(c.UpperCorrectionAlpha, HipPointPairMobility.InverseTensor(_input.ChildInertia, _input.ChildPrincipal, c.UpperCorrectionTorque)) ||
                !F(c.UpperActuatorLimitNm) || c.UpperActuatorLimitNm != HipBilateralForceSolver.ActuatorLimit(c.PureHipTorque) ||
                c.UpperCorrectionTorque.magnitude > HipBilateralForceSolver.UpperCorrectionTorqueLimit + 0.0001f ||
                c.UpperActuatorTorque.magnitude > c.UpperActuatorLimitNm + 0.0001f ||
                !V(c.PeerForce) || !V(c.AppliedA) ||
                !Same(c.AppliedA, c.Force / c.ChildMass + (c.Force + c.PeerForce) / c.ParentInput.Response.TotalMass) ||
                c.AppliedA.magnitude > HipCharacterJointResponseIsolation.TetherMaxAcceleration + 0.0001f ||
                !F(c.Scale) || c.Scale < 0f || c.Scale > 1f || !F(c.PairPower) || !F(c.DampingPower) ||
                !F(c.BilateralDampingPower) || c.BilateralDampingPower > 0.0001f) return false;
            return true;
        }
        private bool Commit(HipAnchorPairReport.Command c)
        {
            if (!CanCommit(c)) return Invalid("HIP_PAIR_COMMIT_INVALID");
            int written = 0;
            try
            {
                _child.AddForceAtPosition(c.Force, c.ChildPoint, ForceMode.Force); written = 1;
                _parent.AddForceAtPosition(-c.Force, c.ParentPoint, ForceMode.Force); written = 2;
                // The staged pure request was not written. This is the sole full Hip actuator write.
                // Last write: if it throws, only the two returned force writes are owned.
                // Once it returns, publish all three together before optional diagnostics.
                _child.AddTorque(c.UpperActuatorTorque, ForceMode.Force);
            }
            catch (Exception ex)
            {
                if (written >= 2) Undo(_parent, c.Force, c.ParentPoint);
                if (written >= 1) Undo(_child, -c.Force, c.ChildPoint);
                return Invalid("HIP_PAIR_WRITE_EXCEPTION:" + ex.GetType().Name);
            }
            Report.Current = c; Report.CurrentApplied = true; _lastStep = c.Step; _queued = true; _staged = false;
            if (_trace)
            {
                Report.Record(c, (c.AppliedA - c.DesiredA * c.Scale).magnitude);
                Response.Capture(c, _child, _parent);
                Response.CaptureHipTorque(c.Step, c.Fixed, c.UpperActuatorTorque, c.AllocatedHipAlpha);
            }
            return true;
        }
        private static bool Same(Vector3 a, Vector3 b) => PelvisChestPointResponse.Same(a, b);
        internal bool CancelQueued(PhysicsStepContext step)
        {
            if (_staged && step.Matches(_input.Seed.Step)) { _staged = false; _input = new HipAnchorPairInput(); }
            if (!_queued || !step.Matches(_lastStep)) return true;
            Response.Cancel(step);
            _queued = false; int failures = Report.RollbackFailures; HipAnchorPairReport.Command c = Report.Current;
            CancelUpperTorque(_child, -c.UpperActuatorTorque);
            CancelBody(_parent, c.Force, c.ParentPoint);
            CancelBody(_child, -c.Force, c.ChildPoint);
            Report.CurrentApplied = false; Report.CancelledCommands++;
            return failures == Report.RollbackFailures;
        }
        internal bool Quiesce(PhysicsStepContext queuedStep)
        {
            bool ok = CancelQueued(queuedStep); Response.Stop(); _queued = _staged = false; _input = new HipAnchorPairInput(); Report.Selected = false; Report.CurrentApplied = false;
            return ok && Report.RollbackFailures == 0;
        }
        private void CancelBody(Rigidbody body, Vector3 force, Vector3 point)
        {
            try { if (body != null && !body.isKinematic) Undo(body, force, point); }
            catch { Report.RollbackFailures++; Report.Finite = false; Report.Issue = Report.Issue ?? "HIP_PAIR_CANCEL_NATIVE_READ_FAILED"; }
        }
        private void Undo(Rigidbody body, Vector3 force, Vector3 point)
        { try { body.AddForceAtPosition(force, point, ForceMode.Force); } catch { Report.RollbackFailures++; Report.Finite = false; Report.Issue = Report.Issue ?? "HIP_PAIR_ROLLBACK_FAILED"; } }
        private void CancelUpperTorque(Rigidbody body, Vector3 torque)
        {
            try { if (body != null && !body.isKinematic) body.AddTorque(torque, ForceMode.Force); }
            catch { Report.RollbackFailures++; Report.Finite = false; Report.Issue = Report.Issue ?? "HIP_PAIR_UPPER_TORQUE_ROLLBACK_FAILED"; }
        }
        private bool Invalid(string reason) { Report.Finite = false; Report.Issue = Report.Issue ?? reason; return false; }
        private static bool V(Vector3 v) => HipPointPairMobility.V(v);
        private static bool F(float v) => HipPointPairMobility.F(v);
    }
}
