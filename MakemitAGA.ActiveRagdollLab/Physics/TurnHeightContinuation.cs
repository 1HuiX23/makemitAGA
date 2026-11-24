using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Tick = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace.Tick;
using ComPlan = MakemitAGA.ActiveRagdollLab.Physics.PelvisComReferencePlanner.ComPlan;
using Rise = MakemitAGA.ActiveRagdollLab.Physics.LandingRiseAllocation;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Owned only by the recorded release. Integrate the alternative reference, while
    // both allocations see the SAME measured body, preceding actual feedback and local leg commands.
    internal struct TurnHeightContinuation
    {
        internal const float Duration = .20f;
        internal enum State { None, Seed, Compared, Pulse, WitnessReturned, TimeLimit, Geometry, MotionChanged, Clock, Missing, BaselineMismatch, PositionGate, Allocation }
        internal struct Result
        {
            internal State State;
            internal float Floor, InstantY, ReferenceY, VelocityY, ReferenceXZ;
            internal Rise.Plan Native, Trial;
            internal bool HasPlan => State == State.Compared || State == State.Pulse || State == State.PositionGate || State == State.Allocation;
        }

        private PhysicsStepContext _last;
        private SourceStepContext _source;
        private int _floor, _root, _left, _right, _leftContinuity, _rightContinuity, _local;
        private float _elapsed, _startTime, _seedPelvisY;
        private bool _closed;
        private Vector3 _position, _velocity;

        internal Result End(State reason)
        {
            _closed = true;
            return new Result { State = reason };
        }

        private static ComPlan Plan(in ParentHeightProbe.Input i, Vector3 position, Vector3 velocity, float? floor)
            => PelvisComReferencePlanner.Build(position, velocity, i.Goal, i.GoalV, i.Step.Dt,
                true, i.Feedback, i.PreviousRequest, i.PreviousAllocated, i.SettledV, true,
                enhancedNormalFeedback: true, transitionFloorY: floor);

        internal Result Compare(int origin, in Tick l, in Tick r, in ParentHeightProbe.Input i, in Rise.Input rise,
            in HipAnchorPairInput left, in HipAnchorPairInput right, in Command lc, in Command rc,
            bool leftFirst, bool pulse, in Rise.Plan actual)
        {
            if (_closed) return default;
            var s = l.Step;
            if (!_last.Valid && l.Local != origin) return End(State.Missing);
            if (!l.Usable || !r.Usable || !s.Matches(r.Step) || !l.Source.Clock.Matches(r.Source.Clock) ||
                l.Local != r.Local || l.Body == r.Body || l.Floor != r.Floor || l.Source.Root != r.Source.Root)
                return End(State.Clock);
            if (!PelvisDynamicReceiverIsolation.IsDynamicHipArm(i.Arm) || !i.ReceiverReady ||
                !s.Matches(i.Step) || !s.Matches(rise.Step) || !rise.Valid || !rise.WitnessKnown(l.Floor) ||
                !float.IsFinite(i.SampledPelvisY) || !float.IsFinite(i.FloorY) ||
                !PelvisParentFrame.V(i.CommittedPosition) || !PelvisParentFrame.V(i.CommittedV) ||
                !PelvisParentFrame.V(i.PositionError) || !PelvisParentFrame.V(i.VelocityError) ||
                !PelvisParentFrame.V(i.ActualCom) || !float.IsFinite(i.PositionLimit) || i.PositionLimit <= 0f ||
                !actual.Valid) return End(State.Missing);
            if (!l.Full || !r.Full || l.Active || r.Active || l.Precontact || r.Precontact ||
                l.Miss || r.Miss || l.Ordinary || r.Ordinary || l.Recovery || r.Recovery) return End(State.Geometry);
            if (!_last.Valid)
            {
                if (!l.Was || !r.Was || !i.Support.Matches(s) || i.Support.SupportId != l.Floor ||
                    i.Support.StanceMask != 3 || i.Support.ApproachMask != 0 ||
                    !rise.Support.Matches(s) || rise.Support.SupportId != i.Support.SupportId ||
                    rise.Support.StanceMask != i.Support.StanceMask || rise.Support.ApproachMask != i.Support.ApproachMask ||
                    !rise.Support.ObservedStep.Matches(i.Support.ObservedStep)) return End(State.Missing);
                _last=s; _source=l.Source.Clock; _local=l.Local; _floor=l.Floor; _root=l.Source.Root; _left=l.Body; _right=r.Body;
                _leftContinuity=l.Source.Continuity; _rightContinuity=r.Source.Continuity;
                _startTime=s.UnscaledTime; _seedPelvisY=i.SampledPelvisY;
                _position=i.CommittedPosition; _velocity=i.CommittedV;
                return new Result { State=State.Seed, Floor=i.FloorY };
            }
            if (s.Epoch != _last.Epoch || s.Sequence != _last.Sequence+1 || l.Local != _local+1 ||
                (!l.Source.Clock.Matches(_source) && (s.SourceSequence <= _last.SourceSequence || l.Source.Clock.Time <= _source.Time ||
                    l.Source.Clock.Time-_source.Time > FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds)) ||
                s.SourceSequence < _last.SourceSequence || s.FixedTime <= _last.FixedTime || s.UnscaledTime < _last.UnscaledTime ||
                l.Floor != _floor || l.Source.Root != _root || l.Body != _left || r.Body != _right ||
                l.Source.Continuity != _leftContinuity || r.Source.Continuity != _rightContinuity)
                return End(State.Clock);
            if ((l.Motion != "TURN" && l.Motion != "ACCEL") || (r.Motion != "TURN" && r.Motion != "ACCEL"))
                return End(State.MotionChanged);
            if (i.Support.Matches(s) || rise.Support.Matches(s)) return End(State.WitnessReturned);
            if (!new Rise.Input { Support = i.Support, Step = s }.WitnessKnown(l.Floor)) return End(State.Missing);
            _elapsed += s.Dt;
            if (_elapsed > Duration + .000001f || s.UnscaledTime-_startTime > Duration + .0001f) return End(State.TimeLimit);
            _last=s; _source=l.Source.Clock; _local=l.Local;
            // Preserve a deliberate lowering of the animation. Recompute the mass offset with
            // this step's governed rotation, even when its source pose is held across solves.
            float floor = Mathf.Min(_seedPelvisY,i.SampledPelvisY) + (i.FloorY-i.SampledPelvisY);
            var baseline = Plan(i,i.Position,i.Velocity,null);
            if ((baseline.Position-i.CommittedPosition).magnitude > .00001f ||
                (baseline.Velocity-i.CommittedV).magnitude > .00001f) return End(State.BaselineMismatch);
            var instant = Plan(i,i.Position,i.Velocity,floor);
            var trial = Plan(i,_position,_velocity,floor);
            _position=trial.Position; _velocity=trial.Velocity;
            var result = new Result { Floor=floor, InstantY=instant.Position.y-baseline.Position.y,
                ReferenceY=trial.Position.y-baseline.Position.y, VelocityY=trial.Velocity.y-baseline.Velocity.y,
                ReferenceXZ=FootTransitionSource.Planar(trial.Position-baseline.Position).magnitude, Native=actual };
            // This native pulse changes the issued allocation AFTER planning. Keep the reference
            // comparison, never claim an allocation comparison against the pre-pulse request.
            if (pulse) { result.State=State.Pulse; return result; }
            var frame=lc.ParentInput;
            if (!left.Ready || !right.Ready || !lc.Step.Matches(s) || !rc.Step.Matches(s) ||
                !frame.SameAs(left.Seed.ParentInput) || !frame.SameAs(right.Seed.ParentInput) ||
                !frame.SameAs(rc.ParentInput) || frame.UpwardRetentionEligible ||
                !Rise.PlanPair(left,right,leftFirst,out var baseL,out var baseR) ||
                !Rise.Reproduces(baseL,lc) || !Rise.Reproduces(baseR,rc)) return End(State.BaselineMismatch);
            var baseA=PelvisComFeedback.Evaluate(i.PositionError,i.VelocityError,true)+baseline.Acceleration;
            var baseRequest=PelvisDynamicReceiverIsolation.BuildCommonAcceleration(i.Arm,baseA,i.ActualCom,i.SettledV,s,baseline);
            if ((baseRequest.Request-frame.Acceleration).magnitude > .0001f) return End(State.BaselineMismatch);
            var error=i.PositionError+(trial.Position-i.CommittedPosition);
            if (error.magnitude > i.PositionLimit) { result.State=State.PositionGate; _closed=true; return result; }
            var raw=PelvisComFeedback.Evaluate(error,i.VelocityError+(trial.Velocity-i.CommittedV),true)+trial.Acceleration;
            frame.Acceleration=PelvisDynamicReceiverIsolation.BuildCommonAcceleration(i.Arm,raw,i.ActualCom,i.SettledV,s,trial).Request;
            if (!Rise.Solve(frame,left,right,leftFirst,out result.Trial)) { result.State=State.Allocation; _closed=true; return result; }
            result.State=State.Compared;
            return result;
        }
    }
}
