using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // All candidates start at the same measured PRE_N. Planning uses private Hip inputs.
    // The receiver can commit the single selected B result after native contact decisions.
    internal static class LandingRiseAllocation
    {
        internal struct Input
        {
            internal PhysicsStepContext Step;
            internal SupportReferenceContext Support;
            internal float RefV, RefA, PreV, Error;
            internal float VelocityError => RefV - PreV;
            internal bool Valid => Step.Valid && Step.Dt <= .1f && float.IsFinite(Error) && float.IsFinite(RefV) &&
                float.IsFinite(PreV) && float.IsFinite(RefA) && float.IsFinite(VelocityError);
            private static bool Unset(in PhysicsStepContext s) => s.Epoch == 0 && s.Sequence == 0 && s.SourceSequence == 0 &&
                s.FixedTime == 0f && s.Dt == 0f && s.UnscaledTime == 0f;
            internal bool WitnessKnown(int floor) => floor != 0 && (Support.Matches(Step) ? Support.SupportId == floor :
                Unset(Support.ForStep) && Unset(Support.ObservedStep) &&
                Support.StanceMask == 0 && Support.ApproachMask == 0 && Support.SupportId == 0);
        }

        internal enum State { Missing, WrongSupport, ContactOrGeometry, NoDeficit, NoChange, BaselineMismatch, SolveRejected, Compared, PulseApplied }

        internal struct Plan
        {
            internal bool Valid;
            internal float Request, Q, AngularScale, Extra, SocketA;
            internal Vector3 System, RightForce;
        }

        internal struct Result
        {
            internal State Status;
            internal Plan Native, RequestOnly, EarlyRetention;
        }

        internal static float LeadRequest(in Input i, Vector3 native, out bool ahead)
        {
            float dt = i.Step.Dt, dv = i.VelocityError;
            // A rising target or a braking descent can qualify as soon as the next position would trail.
            ahead = dv > 0f && (i.RefV > 0f || i.RefA > 0f) && i.Error + dv * dt > 0f;
            if (!ahead) return native.y;
            float risingA = Mathf.Max(0f, i.RefA);
            float lead = dt * (PelvisComFeedback.NormalPositionGain * dv + PelvisComFeedback.NormalVelocityGain * risingA);
            // Preserve native X/Z, the vector ceiling, and the one-step relative velocity budget.
            float velocityRoom = Mathf.Max(0f, dv / dt + risingA - native.y);
            float vectorRoom = (float)GroundReferenceStepBudget.PositiveBallExit(native, Vector3.up, PelvisHipCommandFrame.MaxMagnitude);
            return native.y + Mathf.Min(lead, Mathf.Min(velocityRoom, vectorRoom));
        }

        internal static Result Compare(in PhysicsStepContext step, int floor, bool free, in Input input,
            HipAnchorPairInput left, HipAnchorPairInput right, in Command actualL, in Command actualR,
            in PelvisHipAttitudeCommand actualTorso, bool leftFirst = true)
        {
            var result = new Result();
            var baseFrame = actualL.ParentInput;
            if (!input.Step.Matches(step) || !input.Valid ||
                !left.Ready || !right.Ready || !actualL.JointSolved || !actualR.JointSolved ||
                !actualL.Step.Matches(step) || !actualR.Step.Matches(step) ||
                !left.Seed.Step.Matches(step) || !right.Seed.Step.Matches(step) || left.Seed.ChildId == right.Seed.ChildId ||
                actualL.ChildId != left.Seed.ChildId || actualR.ChildId != right.Seed.ChildId ||
                actualL.Fixed != left.Seed.Fixed || actualR.Fixed != right.Seed.Fixed || actualL.Fixed != actualR.Fixed ||
                actualR.MotionScale != actualL.MotionScale || actualR.UpperAngularScale != actualL.UpperAngularScale ||
                actualR.UpwardExtraAcceleration != actualL.UpwardExtraAcceleration ||
                !baseFrame.SameAs(left.Seed.ParentInput) || !baseFrame.SameAs(right.Seed.ParentInput) ||
                !baseFrame.SameAs(actualR.ParentInput) || !actualTorso.Matches(baseFrame) ||
                !Same(actualTorso.HipForce, -actualL.Force - actualR.Force) ||
                !Same(actualTorso.AngularReaction, actualL.ParentAngularReaction + actualR.ParentAngularReaction) ||
                !Same(actualTorso.CommonAcceleration, actualL.CommonAcceleration) ||
                actualTorso.MotionScale != actualL.MotionScale || actualTorso.UpperAngularScale != actualL.UpperAngularScale ||
                actualTorso.UpwardExtraAcceleration != actualL.UpwardExtraAcceleration) return result;

            result.Native = Model(actualL, actualR, actualTorso);
            if (!result.Native.Valid) return result;
            bool witness = input.Support.Matches(step);
            if (!input.WitnessKnown(floor))
            { result.Status = State.WrongSupport; return result; }
            if (baseFrame.UpwardRetentionEligible != (baseFrame.UpwardRetentionSelected && witness && input.Error > 0f && baseFrame.Acceleration.y > 0f))
            { result.Status = State.BaselineMismatch; return result; }
            if (!free) { result.Status = State.ContactOrGeometry; return result; }
            float wanted = LeadRequest(input, baseFrame.Acceleration, out bool ahead);
            if (!ahead) { result.Status = State.NoDeficit; return result; }
            var request = baseFrame;
            request.Acceleration.y = wanted;
            // Keep the ordinary witness/position rule in the request-only control.
            request.UpwardRetentionEligible = request.UpwardRetentionSelected && witness && input.Error > 0f && wanted > 0f;
            bool earlier = !witness && request.UpwardRetentionSelected && wanted > 0f;
            if (wanted == baseFrame.Acceleration.y && request.UpwardRetentionEligible == baseFrame.UpwardRetentionEligible && !earlier)
            { result.Status = State.NoChange; return result; }
            if (!PlanPair(left,right,leftFirst,out var baseL,out var baseR) ||
                !Reproduces(baseL,actualL) || !Reproduces(baseR,actualR))
            { result.Status = State.BaselineMismatch; return result; }
            result.Status = State.SolveRejected;
            if (wanted == baseFrame.Acceleration.y && request.UpwardRetentionEligible == baseFrame.UpwardRetentionEligible)
                result.RequestOnly = result.Native;
            else if (!Solve(request,left,right,leftFirst,out result.RequestOnly)) return result;
            if (!earlier) result.EarlyRetention = result.RequestOnly;
            else
            {
                request.UpwardRetentionEligible = true;
                if (!Solve(request,left,right,leftFirst,out result.EarlyRetention)) return result;
            }
            result.Status = State.Compared;
            return result;
        }

        // Preserve primary/secondary order even when the anatomical left leg is secondary.
        private static bool PlanPair(HipAnchorPairInput l, HipAnchorPairInput r, bool leftFirst, out Command lc, out Command rc) =>
            leftFirst ? HipBilateralForceSolver.TryPlan(l,r,out lc,out rc) : HipBilateralForceSolver.TryPlan(r,l,out rc,out lc);

        private static bool Solve(PelvisHipCommandFrame frame, HipAnchorPairInput l, HipAnchorPairInput r, bool leftFirst, out Plan plan)
            => Solve(frame, l, r, leftFirst, out plan, out _);

        internal enum Choice { None, Request, Retention }
        internal struct Allocation
        {
            internal Command Left, Right;
            internal PelvisHipAttitudeCommand Torso;
            internal Choice Kind;
            internal Plan Native, Selected;
        }

        // Only predictions available at command issue may choose the one-step intervention.
        internal static Choice Choose(float preV, float nativeV, float requestV, float retentionV)
        {
            if (!float.IsFinite(preV) || !float.IsFinite(nativeV) || preV > 0f || nativeV > 0f) return Choice.None;
            if (!float.IsFinite(requestV)) return Choice.None;
            if (requestV > 0f) return Choice.Request;
            return float.IsFinite(retentionV) && retentionV > 0f ? Choice.Retention : Choice.None;
        }

        internal static bool TryChoose(in PhysicsStepContext step, int floor, in Input i,
            in HipAnchorPairInput l, in HipAnchorPairInput r, in Command lc, in Command rc,
            in PelvisHipAttitudeCommand torso, bool leftFirst, out Allocation issued)
        {
            issued = default;
            if (!i.Step.Matches(step) || !i.Valid || !i.WitnessKnown(floor) || i.Support.Matches(step) ||
                !lc.Step.Matches(step) || !rc.Step.Matches(step) || lc.ParentV.y > 0f ||
                !torso.Matches(lc.ParentInput) || lc.ParentInput.UpwardRetentionEligible) return false;
            var native = Model(lc, rc, torso);
            float pre = lc.ParentV.y, dt = step.Dt;
            if (!native.Valid || pre + dt * native.SocketA > 0f) return false;
            var frame = lc.ParentInput;
            float wanted = LeadRequest(i, frame.Acceleration, out bool ahead);
            if (!ahead || !l.Seed.ParentInput.SameAs(frame) || !r.Seed.ParentInput.SameAs(frame) ||
                !PlanPair(l,r,leftFirst,out var checkL,out var checkR) || !Reproduces(checkL,lc) || !Reproduces(checkR,rc)) return false;
            frame.Acceleration.y = wanted;
            if (!Solve(frame,l,r,leftFirst,out var request,out var requestAllocation)) return false;
            float nativeV = pre + dt * native.SocketA, requestV = pre + dt * request.SocketA;
            var choice = Choose(pre,nativeV,requestV,float.NaN);
            if (choice == Choice.Request) issued = requestAllocation;
            else
            {
                if (!frame.UpwardRetentionSelected || wanted <= 0f) return false;
                frame.UpwardRetentionEligible = true;
                if (!Solve(frame,l,r,leftFirst,out var retained,out issued)) return false;
                choice = Choose(pre,nativeV,requestV,pre + dt * retained.SocketA);
                if (choice != Choice.Retention || retained.Extra <= 0f) { issued = default; return false; }
            }
            issued.Kind = choice;
            issued.Native = native;
            return true;
        }

        private static bool Solve(PelvisHipCommandFrame frame, HipAnchorPairInput l, HipAnchorPairInput r, bool leftFirst,
            out Plan plan, out Allocation allocation)
        {
            plan = default;
            allocation = default;
            l.Seed.ParentInput = r.Seed.ParentInput = frame;
            if (!PlanPair(l,r,leftFirst,out var lc,out var rc) || !TryTorso(lc,rc,out var torso)) return false;
            plan = Model(lc,rc,torso);
            allocation = new Allocation { Left=lc, Right=rc, Torso=torso, Selected=plan };
            return plan.Valid;
        }

        internal static bool TryTorso(in Command l, in Command r, out PelvisHipAttitudeCommand torso)
        {
            if (!PelvisHipAttitudeCommand.TryPlanWithReaction(l.ParentInput,-l.Force-r.Force,l.ParentTorque+r.ParentTorque,
                l.BilateralAngularReaction,out torso) || torso.Clamped) return false;
            torso.MotionScale=l.MotionScale; torso.UpperAngularScale=l.UpperAngularScale;
            torso.UpwardExtraAcceleration=l.UpwardExtraAcceleration; torso.CommonAcceleration=l.CommonAcceleration;
            return torso.AllocationMatches();
        }

        internal static Plan Model(in Command l, in Command r, in PelvisHipAttitudeCommand torso)
        {
            var frame = l.ParentInput;
            var lever = l.ParentPoint - l.ParentCom;
            var totalTorque = l.ParentTorque + r.ParentTorque + torso.AppliedTorque;
            var alpha = frame.Response.PredictAngular(torso.HipForce,totalTorque,frame.ParentW);
            var pelvisA = torso.CommonAcceleration + frame.Response.PredictCom(torso.HipForce,alpha,frame.ParentW);
            var socketA = pelvisA + Vector3.Cross(alpha,lever) + Vector3.Cross(frame.ParentW,Vector3.Cross(frame.ParentW,lever));
            return new Plan { Valid = HipPointPairMobility.V(socketA) && HipPointPairMobility.V(torso.TargetSystemAcceleration),
                Request = frame.Acceleration.y, Q = l.MotionScale, AngularScale = l.UpperAngularScale,
                Extra = l.UpwardExtraAcceleration, SocketA = socketA.y, System = torso.TargetSystemAcceleration, RightForce = r.Force };
        }

        private static bool Reproduces(in Command a, in Command b) =>
            Same(a.Force,b.Force) && Same(a.UpperActuatorTorque,b.UpperActuatorTorque) && Same(a.CommonAcceleration,b.CommonAcceleration) &&
            Same(a.BilateralAngularReaction,b.BilateralAngularReaction) && Near(a.MotionScale,b.MotionScale) &&
            Near(a.UpperAngularScale,b.UpperAngularScale) && Near(a.UpwardExtraAcceleration,b.UpwardExtraAcceleration);

        internal static bool Near(float a,float b) => Mathf.Abs(a-b) <= .0001f * (1f+Mathf.Max(Mathf.Abs(a),Mathf.Abs(b)));
        private static bool Same(Vector3 a, Vector3 b) => (a-b).magnitude <= .0001f * (1f+Mathf.Max(a.magnitude,b.magnitude));
    }
}
