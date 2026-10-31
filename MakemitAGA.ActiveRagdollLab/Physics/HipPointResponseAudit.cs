using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Optional, bounded pre/next-settled evidence. Never a controller or a drive eligibility gate.
    // Planned point response includes compensated bilateral forces and baseline bias.
    // Independent joined prediction includes both Hip pairs, final
    // Upper actuator torque and final receiver input; residuals still include Knee/load/drag
    // and integration. Neither prediction represents the complete native solve.
    internal sealed class HipPointResponseAudit
    {
        internal struct ResponseFrame
        {
            internal HipAnchorPairReport.Command Command;
            internal HipAttitudeBudgetSample Budget;
            internal PelvisHipAttitudeCommand Attitude;
            internal HipGoalPathEvidence GoalPath;
            internal bool Resolved, Censored, Cancelled, HipTorqueCaptured;
            internal string Issue;
            internal PhysicsStepContext NextStep;
            internal Vector3 ChildV, ParentV, ChildW, ParentW, ChildLever, ParentLever;
            internal Vector3 ChildInertia, ParentInertia;
            internal Quaternion ChildPrincipal, ParentPrincipal;
            internal float ChildDrag, ParentDrag, ChildAngularDrag, ParentAngularDrag;
            internal Vector3 HipTorque, HipDesiredAlpha, HipTorqueAlpha, PairChildAlpha, PairParentAlpha;
            internal Vector3 NextChildPoint, NextParentPoint, NextChildCom, NextParentCom, NextChildV, NextParentV, NextChildW, NextParentW;
            internal Vector3 ObservedPointA, ComPartA, AngularPartA, LeverPartA, PointResidualA;
            internal Vector3 FreeParentModelPointA;
            internal bool CoupledInputsJoined;
            internal Vector3 PeerForce, PeerParentTorque, PredictedParentComA, PredictedParentAlpha, PredictedKnownFixedLeverA;
            internal Vector3 ParentComResidualA, ParentAlphaResidual, KnownFixedLeverResidualA;
            internal Vector3 ChildObservedAlpha, ParentObservedAlpha, ChildTorqueResidualAlpha;
            internal float PostGap, PointClosureError;
        }
        private readonly ResponseFrame[] _ring = new ResponseFrame[16];
        private int _next, _count;
        private ResponseFrame _pending, _peakResidual, _peakGap;
        private bool _hasPending;
        private ResponseFrame _lastBudgetFrame, _beforeFirstTorqueCap, _firstTorqueCap, _firstCompensatedOver;
        internal int BudgetSamples, BudgetMissing, GoalPathMissing, PreserveRawOverSteps, PreserveAppliedOverSteps;
        internal float PeakPreserveRawTorque, PeakPreserveAppliedTorque;
        internal int Captured, Resolved, Censored, Cancelled, Invalid, MissingHipTorque;
        internal float PeakResidual, PeakPostGap, PeakPointClosure;
        internal int CoupledInputJoins, MissingCoupledInputs;
        internal float PeakParentComResidual, PeakParentAlphaResidual, PeakKnownFixedLeverResidual;
        internal string LastIssue;

        internal void Reset()
        {
            _hasPending = false; _pending = _peakResidual = _peakGap = new ResponseFrame();
            _next = _count = Captured = Resolved = Censored = Cancelled = Invalid = MissingHipTorque = 0;
            PeakResidual = PeakPostGap = PeakPointClosure = 0f; LastIssue = null;
            _lastBudgetFrame = _beforeFirstTorqueCap = _firstTorqueCap = _firstCompensatedOver = new ResponseFrame();
            BudgetSamples = BudgetMissing = GoalPathMissing = PreserveRawOverSteps = PreserveAppliedOverSteps = 0;
            PeakPreserveRawTorque = PeakPreserveAppliedTorque = 0f;
            CoupledInputJoins = MissingCoupledInputs = 0;
            PeakParentComResidual = PeakParentAlphaResidual = PeakKnownFixedLeverResidual = 0f;
        }
        internal void Capture(HipAnchorPairReport.Command c, Rigidbody child, Rigidbody parent)
        {
            try
            {
                if (_hasPending) ClosePending("OVERWRITTEN_PENDING", false, true);
                var f = new ResponseFrame { Command = c };
                if (!BodiesMatch(c, child, parent)) { Fault("CAPTURE_BODY_INVALID"); return; }
                f.ChildV = child.velocity; f.ParentV = parent.velocity;
                f.ChildW = child.angularVelocity; f.ParentW = parent.angularVelocity;
                f.ChildLever = c.ChildPoint - c.ChildCom; f.ParentLever = c.ParentPoint - c.ParentCom;
                f.ChildInertia = child.inertiaTensor; f.ParentInertia = parent.inertiaTensor;
                f.ChildPrincipal = child.rotation * child.inertiaTensorRotation;
                f.ParentPrincipal = parent.rotation * parent.inertiaTensorRotation;
                f.ChildDrag = child.drag; f.ParentDrag = parent.drag;
                f.ChildAngularDrag = child.angularDrag; f.ParentAngularDrag = parent.angularDrag;
                f.PairChildAlpha = HipPointPairMobility.InverseTensor(f.ChildInertia, f.ChildPrincipal, c.ChildTorque);
                f.PairParentAlpha = HipPointPairMobility.InverseTensor(f.ParentInertia, f.ParentPrincipal, c.ParentTorque);
                f.FreeParentModelPointA = c.Force * (1f / c.ChildMass + 1f / c.ParentMass) +
                    Vector3.Cross(f.PairChildAlpha, f.ChildLever) - Vector3.Cross(f.PairParentAlpha, f.ParentLever);
                if (!V(f.ChildV) || !V(f.ParentV) || !V(f.ChildW) || !V(f.ParentW) ||
                    !V(f.PairChildAlpha) || !V(f.PairParentAlpha) || !F(f.ChildDrag) || !F(f.ParentDrag) ||
                    !F(f.ChildAngularDrag) || !F(f.ParentAngularDrag) ||
                    (f.ChildV + Vector3.Cross(f.ChildW, f.ChildLever) - c.ChildV).magnitude > 0.0001f ||
                    (f.ParentV + Vector3.Cross(f.ParentW, f.ParentLever) - c.ParentV).magnitude > 0.0001f)
                { Fault("CAPTURE_NATIVE_STATE_INVALID"); return; }
                _pending = f; _hasPending = true; Captured++;
            }
            catch { Fault("CAPTURE_NATIVE_READ_EXCEPTION"); }
        }
        // Run after both real force pairs and both baseline + correction torque writes returned.
        // This predicts only the specified inputs. Child Knee/weight/ground and child native
        // drag remain outside it; changing point levers are separately observed, not guessed.
        internal void CompleteInputs(PhysicsStepContext step, int localFixed, HipAnchorPairReport peer, PelvisHipAttitudeCommand attitude)
        {
            if (!_hasPending) return;
            try
            {
                ResponseFrame f = _pending; HipAnchorPairReport.Command c = f.Command;
                if (!c.Step.Matches(step) || c.Fixed != localFixed || f.CoupledInputsJoined || !f.HipTorqueCaptured ||
                    peer == null || !peer.CurrentApplied || !peer.Finite || !peer.Current.Step.Matches(step) ||
                    peer.Current.Fixed != localFixed || peer.Current.ChildId == c.ChildId || peer.Current.ParentId != c.ParentId ||
                    !c.ParentInput.SameAs(peer.Current.ParentInput) || !attitude.Matches(c.ParentInput) || attitude.MotionScale != c.MotionScale ||
                    !PelvisChestPointResponse.Same(attitude.CommonAcceleration, c.CommonAcceleration))
                { Fault("COUPLED_INPUT_JOIN_INVALID"); return; }
                f.PeerForce = peer.Current.Force; f.PeerParentTorque = peer.Current.ParentTorque;
                PelvisHipCommandFrame input = c.ParentInput;
                Vector3 totalForce = -c.Force - f.PeerForce;
                f.Attitude = attitude;
                Vector3 totalTorque = c.ParentTorque + f.PeerParentTorque + attitude.AppliedTorque;
                f.PredictedParentAlpha = input.Response.PredictAngular(totalForce, totalTorque, input.ParentW);
                f.PredictedParentComA = attitude.CommonAcceleration + input.Response.PredictCom(totalForce, f.PredictedParentAlpha, input.ParentW);
                f.PredictedKnownFixedLeverA = c.Force / c.ChildMass + Vector3.Cross(f.PairChildAlpha + f.HipTorqueAlpha, f.ChildLever) -
                    f.PredictedParentComA - Vector3.Cross(f.PredictedParentAlpha, f.ParentLever);
                if (!V(f.PeerForce) || !V(f.PeerParentTorque) || !V(f.PredictedParentAlpha) || !V(f.PredictedParentComA) || !V(f.PredictedKnownFixedLeverA))
                { Fault("COUPLED_INPUT_MODEL_NONFINITE"); return; }
                f.CoupledInputsJoined = true; _pending = f; CoupledInputJoins++;
            }
            catch { Fault("COUPLED_INPUT_JOIN_EXCEPTION"); }
        }
        // Optional provenance is attached only after the real Hip torque call returned.
        internal void CaptureGoalPath(HipGoalPathEvidence path)
        {
            if (_hasPending && path.Valid && path.Step.Matches(_pending.Command.Step) && path.Fixed == _pending.Command.Fixed)
                _pending.GoalPath = path;
        }
        internal void CompleteAttitudeBudget(PhysicsStepContext step, int localFixed, PelvisReceiverReport receiver)
        {
            if (!_hasPending) return;
            try
            {
                ResponseFrame f = _pending;
                if (!f.Command.Step.Matches(step) || f.Command.Fixed != localFixed || !f.CoupledInputsJoined || f.Budget.Valid ||
                    receiver == null || !receiver.TraceRunning || !receiver.TraceHipTranslation || receiver.CurrentTraceFixed != localFixed ||
                    receiver.AngularActuation.Fixed != localFixed) return;
                f.Budget = HipAttitudeBudgetSample.Evaluate(f.Command.ParentInput, receiver.AngularActuation, f.Attitude,
                    -f.Command.Force - f.PeerForce, f.Command.ParentTorque + f.PeerParentTorque);
                _pending = f;
            }
            catch { } // Missing/invalid evidence is counted at resolution; never changes run eligibility.
        }
        // Called only AFTER both actual Upper AddTorque calls returned. Torque is their
        // final actuator sum; baseline and correction remain separately available in Command.
        internal void CaptureHipTorque(PhysicsStepContext step, int localFixed, Vector3 torque, Vector3 desiredAlpha)
        {
            try
            {
                if (!_hasPending || !_pending.Command.Step.Matches(step) || _pending.Command.Fixed != localFixed)
                { Fault("HIP_TORQUE_JOIN_MISSING"); return; }
                if (_pending.HipTorqueCaptured || !V(torque) || !V(desiredAlpha))
                { Fault("HIP_TORQUE_DUPLICATE_OR_INVALID"); return; }
                Vector3 a = HipPointPairMobility.InverseTensor(_pending.ChildInertia, _pending.ChildPrincipal, torque);
                if (!V(a)) { Fault("HIP_TORQUE_ALPHA_INVALID"); return; }
                _pending.HipTorque = torque; _pending.HipDesiredAlpha = desiredAlpha;
                _pending.HipTorqueAlpha = a; _pending.HipTorqueCaptured = true;
            }
            catch { Fault("HIP_TORQUE_CAPTURE_EXCEPTION"); }
        }
        // Run before this step's 30 mm gate and before any safe-off zeros native velocities.
        internal void Observe(PhysicsStepContext step, int localFixed, Rigidbody child, Rigidbody parent,
            Vector3 childPoint, Vector3 parentPoint)
        {
            if (!_hasPending) return;
            try
            {
                ResponseFrame f = _pending; HipAnchorPairReport.Command c = f.Command;
                if (c.Step.Matches(step)) return; // duplicate does not consume the original command
                if (!step.Valid || step.Epoch != c.Step.Epoch || step.Sequence != c.Step.Sequence + 1 || localFixed != c.Fixed + 1)
                { ClosePending("NEXT_STEP_GAP", false, true); return; }
                if (!BodiesMatch(c, child, parent) || !V(childPoint) || !V(parentPoint))
                { ClosePending("NEXT_BODY_OR_POINT_INVALID", false, true); return; }
                Vector3 cv = child.velocity, pv = parent.velocity, cw = child.angularVelocity, pw = parent.angularVelocity;
                Vector3 cc = child.worldCenterOfMass, pc = parent.worldCenterOfMass;
                Vector3 cr = childPoint - cc, pr = parentPoint - pc;
                if (!V(cv) || !V(pv) || !V(cw) || !V(pw) || !V(cr) || !V(pr))
                { ClosePending("NEXT_NATIVE_STATE_INVALID", false, true); return; }
                float dt = c.Step.Dt; // the completed command's duration, never the new command dt
                f.NextStep = step; f.NextChildPoint = childPoint; f.NextParentPoint = parentPoint;
                f.NextChildCom = cc; f.NextParentCom = pc;
                f.NextChildV = cv; f.NextParentV = pv; f.NextChildW = cw; f.NextParentW = pw;
                f.ObservedPointA = (cv + Vector3.Cross(cw, cr) - pv - Vector3.Cross(pw, pr) - c.ChildV + c.ParentV) / dt;
                f.ComPartA = (cv - f.ChildV - pv + f.ParentV) / dt;
                f.AngularPartA = (Vector3.Cross(cw - f.ChildW, f.ChildLever) - Vector3.Cross(pw - f.ParentW, f.ParentLever)) / dt;
                f.LeverPartA = (Vector3.Cross(cw, cr - f.ChildLever) - Vector3.Cross(pw, pr - f.ParentLever)) / dt;
                f.PointClosureError = (f.ObservedPointA - f.ComPartA - f.AngularPartA - f.LeverPartA).magnitude;
                f.PointResidualA = f.ObservedPointA - c.PredictedNetA;
                f.ChildObservedAlpha = (cw - f.ChildW) / dt; f.ParentObservedAlpha = (pw - f.ParentW) / dt;
                f.ChildTorqueResidualAlpha = f.ChildObservedAlpha - f.PairChildAlpha - f.HipTorqueAlpha;
                f.PostGap = (parentPoint - childPoint).magnitude;
                if (!V(f.ObservedPointA) || !V(f.PointResidualA) || !V(f.ChildTorqueResidualAlpha) || !F(f.PointClosureError) || !F(f.PostGap))
                { ClosePending("NEXT_RESPONSE_NONFINITE", false, true); return; }
                if (!f.HipTorqueCaptured) MissingHipTorque++;
                if (f.CoupledInputsJoined)
                {
                    f.ParentComResidualA = (pv - f.ParentV) / dt - f.PredictedParentComA;
                    f.ParentAlphaResidual = f.ParentObservedAlpha - f.PredictedParentAlpha;
                    f.KnownFixedLeverResidualA = f.ComPartA + f.AngularPartA - f.PredictedKnownFixedLeverA;
                    if (!V(f.ParentComResidualA) || !V(f.ParentAlphaResidual) || !V(f.KnownFixedLeverResidualA))
                    { f.CoupledInputsJoined = false; Fault("COUPLED_RESPONSE_NONFINITE"); }
                    else
                    {
                        PeakParentComResidual = Mathf.Max(PeakParentComResidual, f.ParentComResidualA.magnitude);
                        PeakParentAlphaResidual = Mathf.Max(PeakParentAlphaResidual, f.ParentAlphaResidual.magnitude);
                        PeakKnownFixedLeverResidual = Mathf.Max(PeakKnownFixedLeverResidual, f.KnownFixedLeverResidualA.magnitude);
                    }
                }
                if (!f.CoupledInputsJoined) MissingCoupledInputs++;
                f.Resolved = true; _hasPending = false; Resolved++;
                if (_peakResidual.Command.Fixed == 0 || f.PointResidualA.magnitude > PeakResidual) _peakResidual = f;
                if (_peakGap.Command.Fixed == 0 || f.PostGap > PeakPostGap) _peakGap = f;
                PeakResidual = Mathf.Max(PeakResidual, f.PointResidualA.magnitude);
                PeakPostGap = Mathf.Max(PeakPostGap, f.PostGap); PeakPointClosure = Mathf.Max(PeakPointClosure, f.PointClosureError);
                RecordBudgetResolution(f);
                Store(f);
            }
            catch { ClosePending("NEXT_NATIVE_READ_EXCEPTION", false, true); }
        }
        private void RecordBudgetResolution(ResponseFrame f)
        {
            // Only completed solves enter coverage/counts and first-onset witnesses.
            // Keep the preceding resolved frame only if it is truly adjacent in the same epoch.
            if (!f.GoalPath.Valid) GoalPathMissing++;
            if (!f.Budget.Valid) { BudgetMissing++; _lastBudgetFrame = new ResponseFrame(); return; }
            BudgetSamples++;
            if (f.Budget.PreserveRawExceeds) PreserveRawOverSteps++;
            if (f.Budget.PreserveAppliedExceeds) PreserveAppliedOverSteps++;
            PeakPreserveRawTorque = Mathf.Max(PeakPreserveRawTorque, f.Budget.PreserveRawTorque.magnitude);
            PeakPreserveAppliedTorque = Mathf.Max(PeakPreserveAppliedTorque, f.Budget.PreserveAppliedTorque.magnitude);
            if (_firstTorqueCap.Command.Fixed == 0 && f.Budget.ReceiverClamped)
            {
                _firstTorqueCap = f;
                if (_lastBudgetFrame.Budget.Valid && _lastBudgetFrame.Command.Step.Epoch == f.Command.Step.Epoch &&
                    _lastBudgetFrame.Command.Step.Sequence + 1 == f.Command.Step.Sequence && _lastBudgetFrame.Command.Fixed + 1 == f.Command.Fixed)
                    _beforeFirstTorqueCap = _lastBudgetFrame;
            }
            if (_firstCompensatedOver.Command.Fixed == 0 && f.Budget.PreserveAppliedExceeds) _firstCompensatedOver = f;
            _lastBudgetFrame = f;
        }
        internal void Cancel(PhysicsStepContext step)
        {
            if (_hasPending && _pending.Command.Step.Matches(step)) ClosePending("CANCELLED_BEFORE_SOLVE", true, false);
        }
        internal void Stop() { if (_hasPending) ClosePending("STOP_WITHOUT_NEXT_SETTLED", false, false); }
        private void ClosePending(string issue, bool cancelled, bool invalid)
        {
            if (!_hasPending) return;
            ResponseFrame f = _pending; _hasPending = false; f.Censored = true; f.Cancelled = cancelled; f.Issue = issue;
            Censored++; if (cancelled) Cancelled++; if (invalid) Fault(issue); Store(f);
        }
        private void Store(ResponseFrame f) { _ring[_next] = f; _next = (_next + 1) % _ring.Length; _count = Math.Min(_count + 1, _ring.Length); }
        private void Fault(string issue) { Invalid++; LastIssue = issue; }
        private static bool BodiesMatch(HipAnchorPairReport.Command c, Rigidbody child, Rigidbody parent) => c.Step.Valid && c.Fixed > 0 &&
            child != null && parent != null && !child.isKinematic && !parent.isKinematic &&
            child.GetInstanceID() == c.ChildId && parent.GetInstanceID() == c.ParentId;
        private static bool V(Vector3 v) => HipPointPairMobility.V(v);
        private static bool F(float v) => HipPointPairMobility.F(v);
        private static string N(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
        private static string S(Vector3 v) => "(" + N(v.x) + "," + N(v.y) + "," + N(v.z) + ")";
        private static string Q(Quaternion q) => "(" + N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w) + ")";
        internal string GetDetails()
        {
            var sb = new StringBuilder("hipPointResponse=V041000_MOTION_FEASIBILITY_READ_ONLY/physicalArm=B_V041000_HIP_MOTION_FEASIBILITY_ALLOCATION/captured-resolved-censored-cancelled-invalid-missingTorque=" +
                Captured + "-" + Resolved + "-" + Censored + "-" + Cancelled + "-" + Invalid + "-" + MissingHipTorque +
                "/pending=" + _hasPending + "/lastIssue=" + (LastIssue ?? "<none>") + "/peak(plannedKnownResidualA-postGapMm-pointClosureA)=" +
                N(PeakResidual) + "|" + N(PeakPostGap * 1000f) + "|" + N(PeakPointClosure) +
                "/coupledInputs(joins-missing)=" + CoupledInputJoins + "-" + MissingCoupledInputs +
                "/coupledResidualPeak(parentComA-parentAlpha-knownFixedLeverA)=" + N(PeakParentComResidual) + "|" + N(PeakParentAlphaResidual) + "|" + N(PeakKnownFixedLeverResidual) +
                "/coupledPrediction=READ_ONLY_BOTH_HIPS_FINAL_UPPER_AND_RECEIVER_TORQUE_AND_CHEST_COM_JOINT/childOtherLoadsAndDragExcluded=True/compensatedForcePointA=BILATERAL_FORCE_ONLY_BASELINE_BIAS_EXCLUDED" +
                "/residualMeaning=OBSERVED_MINUS_KNOWN_PLAN_INCLUDES_LEVER_CHANGE_OTHER_LOADS_DRAG_INTEGRATION/noFeedback=True");
            sb.Append("/attitudeBudget(resolved-missing-goalMissing-rawOver-appliedOver)=").Append(BudgetSamples).Append('-').Append(BudgetMissing)
                .Append('-').Append(GoalPathMissing).Append('-').Append(PreserveRawOverSteps).Append('-').Append(PreserveAppliedOverSteps)
                .Append("/requiredTorquePeak(preserveRaw-preserveApplied)=").Append(N(PeakPreserveRawTorque)).Append('|').Append(N(PeakPreserveAppliedTorque))
                .Append("/budgetMeaning=SAME_STATE_BASELINE_RAW_AND_TORQUE2_RESPONSE/overThresholdNm=2/notAFutureTrajectory=True");
            Append(sb, "BEFORE_FIRST_RECEIVER_TORQUE_CAP", _beforeFirstTorqueCap);
            Append(sb, "FIRST_RECEIVER_TORQUE_CAP", _firstTorqueCap);
            Append(sb, "FIRST_PRESERVE_APPLIED_OVER_BUDGET", _firstCompensatedOver);
            Append(sb, "PEAK_RESIDUAL", _peakResidual); Append(sb, "PEAK_POST_GAP", _peakGap);
            for (int i = 0; i < _count; i++) Append(sb, "TAIL", _ring[(_next - _count + i + _ring.Length) % _ring.Length]);
            if (_hasPending) Append(sb, "PENDING", _pending);
            return sb.ToString();
        }
        private static void Append(StringBuilder sb, string tag, ResponseFrame f)
        {
            HipAnchorPairReport.Command c = f.Command; if (c.Fixed <= 0) return;
            sb.Append("\nhipPointResponseFrame=").Append(c.Fixed).Append("/selection=").Append(tag).Append('/').Append(c.Step.Status())
                .Append("/next=").Append(f.NextStep.Status()).Append("/resolved-censored-cancelled-torque=").Append(f.Resolved).Append('-').Append(f.Censored).Append('-').Append(f.Cancelled).Append('-').Append(f.HipTorqueCaptured)
                .Append("/issue=").Append(f.Issue ?? "<none>").Append("/ids(child-parent)=").Append(c.ChildId).Append('|').Append(c.ParentId)
                .Append("/mass=").Append(N(c.ChildMass)).Append('|').Append(N(c.ParentMass))
                .Append("/inertia=").Append(S(f.ChildInertia)).Append('|').Append(S(f.ParentInertia)).Append("/principal=").Append(Q(f.ChildPrincipal)).Append('|').Append(Q(f.ParentPrincipal))
                .Append("/drag(linear-angular)=").Append(N(f.ChildDrag)).Append('|').Append(N(f.ParentDrag)).Append('|').Append(N(f.ChildAngularDrag)).Append('|').Append(N(f.ParentAngularDrag))
                .Append("/preCOM=").Append(S(c.ChildCom)).Append('|').Append(S(c.ParentCom)).Append("/prePoint=").Append(S(c.ChildPoint)).Append('|').Append(S(c.ParentPoint))
                .Append("/preV=").Append(S(f.ChildV)).Append('|').Append(S(f.ParentV)).Append("/preW=").Append(S(f.ChildW)).Append('|').Append(S(f.ParentW))
                .Append("/preE-Verr=").Append(S(c.Error)).Append('|').Append(S(c.VelocityError))
                .Append("/feedbackA=").Append(S(c.FeedbackA)).Append("/commonCommand=").Append(c.ParentInput.Step.Status())
                .Append("/commonBodyId=").Append(c.ParentInput.BodyId).Append("/baselineCommonA-unscaled=").Append(S(c.ParentInput.Acceleration))
                .Append("/chestId-mass=").Append(c.ParentInput.ChestBodyId).Append('|').Append(N(c.ParentInput.Response.ChestMass))
                .Append("/chestLever-parentW-baselineReceiverTorque=").Append(S(c.ParentInput.Response.ChestLever)).Append('|').Append(S(c.ParentInput.ParentW)).Append('|').Append(S(c.ParentInput.ReceiverTorque))
                .Append("/force-scale=").Append(S(c.Force)).Append('|').Append(N(c.Scale)).Append("/geometryTorque=").Append(S(c.ChildTorque)).Append('|').Append(S(c.ParentTorque))
                .Append("/hipActuatorTorque-desiredAlpha=").Append(f.HipTorqueCaptured ? S(f.HipTorque) + "|" + S(f.HipDesiredAlpha) : "NA_NOT_CAPTURED")
                .Append("/upperTorque(originalRequest-pointCorrection-actuator-netResidual)=").Append(S(c.PureHipTorque)).Append('|').Append(S(c.UpperCorrectionTorque))
                .Append('|').Append(S(c.UpperActuatorTorque)).Append('|').Append(S(c.UpperActuatorTorque + c.ChildTorque - c.NetHipTorque))
                .Append("/upperActuatorLimitNm=").Append(N(c.UpperActuatorLimitNm))
                .Append("/allocation(motionScale-fullFeedbackFeasible-forceScale)=").Append(N(c.MotionScale)).Append('|').Append(c.FullFeedbackFeasible).Append('|').Append(N(c.Scale))
                .Append("/allocatedCommonA-netHipTorque-alpha=").Append(S(c.CommonAcceleration)).Append('|').Append(S(c.NetHipTorque)).Append('|').Append(S(c.AllocatedHipAlpha))
                .Append("/upperCorrectionAlpha=").Append(S(c.UpperCorrectionAlpha))
                .Append("/pairAlpha(freeUpper-freePelvis-constrainedPelvis)=").Append(S(f.PairChildAlpha)).Append('|').Append(S(f.PairParentAlpha)).Append('|').Append(S(c.ParentResponseAlpha))
                .Append("/coupledInputsJoined=").Append(f.CoupledInputsJoined);
            if (f.CoupledInputsJoined)
                sb.Append("/peerForce-parentTorque=").Append(S(f.PeerForce)).Append('|').Append(S(f.PeerParentTorque))
                    .Append("/hipAttitude(request-total-delta-residual)=").Append(S(f.Attitude.RequestedTorque)).Append('|').Append(S(f.Attitude.AppliedTorque))
                    .Append('|').Append(S(f.Attitude.CorrectionTorque)).Append('|').Append(S(f.Attitude.ResidualTorque))
                    .Append("/hipAttitudeTotalCapNm=").Append(N(PelvisHipAttitudeCommand.TotalTorqueLimit)).Append("/hipAttitudeClamped=").Append(f.Attitude.Clamped)
                    .Append("/predictedParent(comA-alpha)=").Append(S(f.PredictedParentComA)).Append('|').Append(S(f.PredictedParentAlpha))
                    .Append("/predictedKnownFixedLeverA=").Append(S(f.PredictedKnownFixedLeverA));
            sb.Append("/jointSolve=").Append(c.JointSolved)
                .Append("/fullRequestForceOnlyScales(point-upperCorrection-upperActuator-receiver)=").Append(N(c.PointScale)).Append('|').Append(N(c.UpperCorrectionScale)).Append('|').Append(N(c.UpperActuatorScale)).Append('|').Append(N(c.ReceiverScale))
                .Append("/jointUnboundedForce=").Append(S(c.UnboundedForce))
                .Append("/jointA(bias-planned-feedbackResidual)=").Append(S(c.KnownBiasA)).Append('|').Append(S(c.PredictedNetA)).Append('|').Append(S(c.FeedbackResidualA))
                .Append("/jointPlanVsJoinedA=").Append(f.CoupledInputsJoined ? S(c.PredictedNetA - f.PredictedKnownFixedLeverA) : "NA");
            sb.Append("/hipGoalPath={").Append(f.GoalPath.Details()).Append('}');
            if (f.Budget.Valid)
                sb.Append("/attitudeBudget(rawReceiver-effectiveHip-preserveRaw-preserveApplied)=").Append(S(f.Budget.RawReceiverTorque))
                    .Append('|').Append(S(f.Budget.EffectiveHipTorque)).Append('|').Append(S(f.Budget.PreserveRawTorque)).Append('|').Append(S(f.Budget.PreserveAppliedTorque))
                    .Append("/budgetAlpha(hipResponse-rawResponseDeficit)=").Append(S(f.Budget.HipResponseAlpha)).Append('|').Append(S(f.Budget.RawResponseDeficitAlpha))
                    .Append("/budgetFlags(receiverCap-rawOver-appliedOver)=").Append(f.Budget.ReceiverClamped).Append('|').Append(f.Budget.PreserveRawExceeds).Append('|').Append(f.Budget.PreserveAppliedExceeds);
            else sb.Append("/attitudeBudget=NA_INPUTS_NOT_JOINED");
            if (!f.Resolved) { sb.Append("/response=NA_CENSORED_OR_PENDING"); return; }
            sb.Append("/postCOM=").Append(S(f.NextChildCom)).Append('|').Append(S(f.NextParentCom))
                .Append("/postPoint=").Append(S(f.NextChildPoint)).Append('|').Append(S(f.NextParentPoint)).Append("/postV=").Append(S(f.NextChildV)).Append('|').Append(S(f.NextParentV)).Append("/postW=").Append(S(f.NextChildW)).Append('|').Append(S(f.NextParentW))
                .Append("/postGapMm=").Append(N(f.PostGap * 1000f))
                .Append("/compensatedForcePointA=").Append(S(c.AppliedA)).Append("/freeParentModelPointA=").Append(S(f.FreeParentModelPointA)).Append("/observedPointA=").Append(S(f.ObservedPointA)).Append("/plannedKnownPointResidualA=").Append(S(f.PointResidualA))
                .Append("/pointParts(COM-angular-lever)=").Append(S(f.ComPartA)).Append('|').Append(S(f.AngularPartA)).Append('|').Append(S(f.LeverPartA)).Append("/pointClosureA=").Append(N(f.PointClosureError))
                .Append("/hipTorqueAlphaNoGyroDrag=").Append(f.HipTorqueCaptured ? S(f.HipTorqueAlpha) : "NA_NOT_CAPTURED")
                .Append("/observedAlpha=").Append(S(f.ChildObservedAlpha)).Append('|').Append(S(f.ParentObservedAlpha)).Append("/childTorqueResidualAlpha=").Append(f.HipTorqueCaptured ? S(f.ChildTorqueResidualAlpha) : "NA_NOT_CAPTURED");
            sb.Append("/coupledResidual(parentComA-parentAlpha-knownFixedLeverA)=").Append(f.CoupledInputsJoined ?
                S(f.ParentComResidualA) + "|" + S(f.ParentAlphaResidual) + "|" + S(f.KnownFixedLeverResidualA) : "NA_INPUTS_NOT_JOINED");
        }
    }
}
