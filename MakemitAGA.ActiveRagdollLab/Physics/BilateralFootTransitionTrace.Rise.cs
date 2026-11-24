using System.Collections.Generic;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Rise = MakemitAGA.ActiveRagdollLab.Physics.LandingRiseAllocation;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralFootTransitionTrace
    {
        // A window may have closed on an ordinary entry before the coordinator arrives.
        // Keep its incoming command until that exact next endpoint is read.
        private Window _risePending;
        private PhysicsStepContext _lastRiseStep;
        internal bool WantsRiseStep(in PhysicsStepContext step) => !_sealed && !_lastRiseStep.Matches(step) && _previous.Left.Step.Matches(step) &&
            (_risePending != null || WantsLeftHeight);

        internal void ObserveRise(in PhysicsStepContext step, in Rise.Input input, in HipAnchorPairInput left,
            in HipAnchorPairInput right, in Command actualL, in Command actualR, in PelvisHipAttitudeCommand torso,
            in PelvisNormalResponseEvidence resolved, bool leftFirst = true, bool pulseApplied = false, ParentHeightProbe.Input height = default)
        {
            if (!WantsRiseStep(step)) return;
            _lastRiseStep = step;
            var t = _previous.Left;
            var pending = _risePending;
            if (pending != null && pending.Last.Foot.Step.Matches(step))
                pending.NativeLandingPath.SettleRise(t,input,actualL,actualR,resolved);
            _risePending = null;
            if (!WantsLeftHeight) return;
            var w = _release[0];
            if (!w.Last.Foot.Step.Matches(step) || !w.NativeLandingPath.BeginRise(t,_previous.Right,input,
                left,right,actualL,actualR,torso,leftFirst,pulseApplied)) return;
            if (w.Early != null && PelvisDynamicReceiverIsolation.IsDynamicHipArm(height.Arm))
                w.NativeLandingPath.CompareTurnHeight(w.Origin.Foot.Local,t,_previous.Right,height,left,right,actualL,actualR,leftFirst,pulseApplied);
            _risePending = w;
        }

        internal void NoteRiseFault(in PhysicsStepContext step)
        {
            if (_previous.Left.Step.Matches(step))
            {
                var w = _risePending ?? _release[0];
                if (w?.NativeLandingPath != null) w.NativeLandingPath.RiseFaults++;
            }
            _risePending = null;
        }

        private sealed partial class LeftLandingPath
        {
            private enum Read { Pending, IdentityMismatch, ResponseMissing, BaselineMismatch, Matched }
            private sealed class RiseRow
            {
                internal PhysicsStepContext Step, ReadClock;
                internal int From, To, Upper, OtherUpper, Pelvis, Chest, LegFixed, Floor, Foot;
                internal float PreHipV, PostHipV, PostComV, ObservedA, HipError;
                internal bool HasHipError, InputValid, WitnessKnown;
                internal Read State;
                internal Rise.Input Input;
                internal Rise.Result Plans;
                internal TurnHeightContinuation.Result TurnHeight;
                internal ParentHeightProbe.Input HeightInput;
                internal float Predicted(in Rise.Plan p) => PreHipV + Step.Dt * p.SocketA;
                internal bool Earlier(in Rise.Plan p)
                {
                    if (State != Read.Matched || !p.Valid || PreHipV > 0f || PostHipV > 0f) return false;
                    float original = Predicted(Plans.Native), alternative = Predicted(p), gain = alternative - original;
                    return original <= 0f && alternative > 0f && PostHipV + gain > 0f && gain > Mathf.Abs(original - PostHipV);
                }
            }

            private readonly List<RiseRow> _rise = new List<RiseRow>(16);
            internal int RiseFaults;
            private int _riseOmitted;

            internal bool BeginRise(in Tick t, in Tick other, in Rise.Input i, in HipAnchorPairInput l, in HipAnchorPairInput r,
                in Command lc, in Command rc, in PelvisHipAttitudeCommand torso, bool leftFirst, bool pulseApplied)
            {
                if (_rise.Count != 0 && _rise[_rise.Count-1].Step.Matches(t.Step)) return false;
                if (_rise.Count >= LandingEventHistory.RingCapacity) { _riseOmitted++; return false; }
                bool free = t.Full && other.Full && !(t.Active || t.Precontact || other.Active || other.Precontact ||
                    t.Miss || t.Ordinary || t.Recovery || other.Miss || other.Ordinary || other.Recovery);
                // The pulse receipt carries the original and the selected prediction. This row
                // joins the actual selected command to the next receiver and socket readback.
                var plans = pulseApplied ? new Rise.Result { Status=Rise.State.PulseApplied, Native=Rise.Model(lc,rc,torso) } :
                    Rise.Compare(t.Step,t.Floor,free,i,l,r,lc,rc,torso,leftFirst);
                bool known = i.Valid && i.Step.Matches(t.Step);
                _rise.Add(new RiseRow { Step=t.Step, From=t.Local, Floor=t.Floor, Foot=t.Body, Upper=lc.ChildId,
                    OtherUpper=rc.ChildId, LegFixed=lc.Fixed, Pelvis=lc.ParentId, Chest=lc.ParentInput.ChestBodyId, PreHipV=lc.ParentV.y,
                    InputValid=known, WitnessKnown=known && i.WitnessKnown(t.Floor), Input=i, Plans=plans });
                return true;
            }

            internal void SettleRise(in Tick t, in Rise.Input now, in Command nextL, in Command nextR, in PelvisNormalResponseEvidence response)
            {
                if (_rise.Count == 0) return;
                var row = _rise[_rise.Count-1];
                if (row.State != Read.Pending) return;
                row.State = Read.IdentityMismatch;
                if (t.Local != row.From + 1 || t.Floor != row.Floor || t.Body != row.Foot ||
                    t.Step.Epoch != row.Step.Epoch || t.Step.Sequence != row.Step.Sequence+1 ||
                    t.Step.SourceSequence < row.Step.SourceSequence || !nextL.Step.Matches(t.Step) || !nextR.Step.Matches(t.Step) ||
                    nextL.Fixed != row.LegFixed+1 || nextR.Fixed != nextL.Fixed || nextL.ChildId != row.Upper ||
                    nextR.ChildId != row.OtherUpper || nextL.ParentId != row.Pelvis || nextR.ParentId != row.Pelvis ||
                    nextL.ParentInput.ChestBodyId != row.Chest || !nextL.JointSolved || !nextR.JointSolved ||
                    !float.IsFinite(nextL.ParentV.y)) return;
                row.ReadClock = t.Step; row.To = t.Local; row.PostHipV = nextL.ParentV.y;
                row.HasHipError = t.HipValid; row.HipError = t.HipHeight;
                row.State = Read.ResponseMissing;
                if (!response.Matches(t.Step) || !response.Input.Matches(row.Step) ||
                    response.PelvisId != row.Pelvis || response.ChestId != row.Chest) return;
                row.ObservedA = response.Observed; row.PostComV = response.PostV;
                var native = row.Plans.Native;
                row.State = native.Valid && Rise.Near(response.Request,native.Request) && Rise.Near(response.Q,native.Q) &&
                    Rise.Near(response.Allocated,native.System.y) && Rise.Near(response.PreV,row.Input.PreV) &&
                    Rise.Near(response.VelocityErrorY,row.Input.VelocityError) && Rise.Near(response.ErrorY,row.Input.Error) &&
                    Rise.Near(response.Feedforward,row.Input.RefA) && Rise.Near(response.UpwardExtra,native.Extra) &&
                    now.Step.Matches(t.Step) && Rise.Near(response.PostV,now.PreV) &&
                    (!response.BrakingCaptured || (row.WitnessKnown && row.Input.Support.StanceMask == response.WitnessMask &&
                        row.Input.Support.ApproachMask == response.ApproachMask && row.Input.Support.SupportId == response.WitnessSupportId)) ?
                    Read.Matched : Read.BaselineMismatch;
            }

            internal void AppendRise(StringBuilder b, int origin, int first, in Tick entry, TurnHandoffWindow legs)
            {
                bool applied = false;
                int matched = 0, compared = 0, firstActual = 0, lead = 0, early = 0, earliestFree = 0, earliestWitness = 0;
                foreach (var r in _rise)
                {
                    applied |= r.Plans.Status == Rise.State.PulseApplied;
                    if (r.State != Read.Matched) continue;
                    matched++;
                    if (firstActual == 0 && r.PreHipV <= 0f && r.PostHipV > 0f) firstActual = r.To;
                    if (r.Plans.Status != Rise.State.Compared) continue;
                    compared++;
                    if (r.Earlier(r.Plans.RequestOnly) && lead == 0) lead = r.To;
                    if (!r.Earlier(r.Plans.EarlyRetention)) continue;
                    if (early == 0) early = r.To;
                    if (r.Input.Support.StanceMask == 0 && earliestFree == 0) earliestFree = r.To;
                    if (r.Input.Support.StanceMask != 0 && earliestWitness == 0) earliestWitness = r.To;
                }
                b.Append("\nTURN_RISE_LINK_V041720/start-first=").Append(origin).Append('|').Append(first)
                    .Append("/issues-matched-compared-omitted-faults=").Append(_rise.Count).Append('|').Append(matched).Append('|')
                    .Append(compared).Append('|').Append(_riseOmitted).Append('|').Append(RiseFaults)
                    .Append(applied ? "/firstIssuedCross-requestOnlyEarlier-earlyRetentionEarlier-noWitness-withWitness=" :
                        "/firstNativeCross-requestOnlyEarlier-earlyRetentionEarlier-noWitness-withWitness=")
                    .Append(firstActual).Append('|').Append(lead).Append('|').Append(early).Append('|').Append(earliestFree).Append('|').Append(earliestWitness);
                b.Append("/firstNativeCornerVyMps=").Append(first > 0 && entry.Local == first ? F(entry.CornerVelocity.y) : "NA");
                AppendFirstFoot(b,legs,first,false); AppendFirstFoot(b,legs,first,true);
                b.Append("/model=INDEPENDENT_ONE_SOLVE_LEFT_SOCKET/foot=INDEPENDENT_TWO_SOLVE");
                b.Append("\nTURN_RISE_V041720/clock=from,to,shared,source/status=issue,read,witness,approach")
                    .Append("/input=errorMm,refV,refA,preComV,preHipV/read=observedComA,postComV,postHipV,hipMinusReferenceMm")
                    .Append("/each(native,requestOnly,earlyRetention)=requestY,allocatedY,q,angularScale,extraY,predHipV,deltaSystemXZ,deltaRightForceN");
                foreach (var r in _rise)
                {
                    var i=r.Input;
                    b.Append("\nR172,").Append(r.From).Append(',').Append(r.To).Append(',').Append(r.Step.Sequence).Append(',').Append(r.Step.SourceSequence)
                        .Append(',').Append(r.Plans.Status).Append(',').Append(r.State).Append(',').Append(Integer(r.WitnessKnown,i.Support.StanceMask))
                        .Append(',').Append(Integer(r.WitnessKnown,i.Support.ApproachMask));
                    RiseValue(b,r.InputValid,i.Error*1000f); RiseValue(b,r.InputValid,i.RefV); RiseValue(b,r.InputValid,i.RefA);
                    RiseValue(b,r.InputValid,i.PreV); RiseValue(b,r.Plans.Native.Valid,r.PreHipV);
                    bool resolved = r.State == Read.Matched || r.State == Read.BaselineMismatch;
                    RiseValue(b,resolved,r.ObservedA); RiseValue(b,resolved,r.PostComV);
                    RiseValue(b,r.To!=0,r.PostHipV); RiseValue(b,r.HasHipError,r.HipError*1000f);
                    WritePlan(b,r,r.Plans.Native); WritePlan(b,r,r.Plans.RequestOnly); WritePlan(b,r,r.Plans.EarlyRetention);
                }
            }

            private static void AppendFirstFoot(StringBuilder b, TurnHandoffWindow legs, int first, bool buffered)
            {
                b.Append(buffered ? "/bufferedTwo=" : "/originalTwo=");
                if (legs == null || !legs.TryTwoStepMargin(first,buffered,out var m)) { b.Append("Unavailable"); return; }
                b.Append(m.Pass ? "Pass" : "Fail").Append('|').Append(m.MarginPass ? "MarginPass" : "MarginFail")
                    .Append('|').Append(F(m.WorstMargin*1000f)).Append('|').Append(F(m.Body.Velocity.y));
            }
            private static void RiseValue(StringBuilder b,bool valid,float x) => b.Append(',').Append(valid ? F(x) : "NA");
            private static void WritePlan(StringBuilder b,RiseRow r,in Rise.Plan p)
            {
                if (!p.Valid) { b.Append(",NA,NA,NA,NA,NA,NA,NA,NA"); return; }
                b.Append(',').Append(F(p.Request)).Append(',').Append(F(p.System.y)).Append(',').Append(F(p.Q))
                    .Append(',').Append(F(p.AngularScale)).Append(',').Append(F(p.Extra)).Append(',').Append(F(r.Predicted(p)))
                    .Append(',').Append(F(FootTransitionSource.Planar(p.System-r.Plans.Native.System).magnitude))
                    .Append(',').Append(F((p.RightForce-r.Plans.Native.RightForce).magnitude));
            }
        }
    }
}
