using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using Command = MakemitAGA.ActiveRagdollLab.Model.HipAnchorPairReport.Command;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // The two native lifecycles own their receipts. This window owns one choice from
    // a simultaneous formal release until the left foot's first landing opportunity.
    internal sealed class LandingPulseWindow
    {
        internal struct Foot
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Local, Body, Floor, Root, SourceContinuity;
            internal bool Was, Active, Precontact, LiftRelease, Miss, Ordinary, Recovery, PendingMiss;
            internal float Age, Sole, Gap;
            internal bool HasEntryGeometry;
            internal bool Valid => Step.Valid && Source.Valid && Source.Epoch == Step.Epoch && Source.Sequence == Step.SourceSequence &&
                SourceContinuity > 0 && Local > 0 && Body != 0 && Floor != 0 && Root != 0 && float.IsFinite(Age) &&
                Age >= 0f && Age <= FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds && !(Active && Precontact);
            internal bool Free => !(Active || Precontact || Miss || Ordinary || Recovery);
            internal bool Follows(in Foot prev) => prev.Valid && Step.Epoch == prev.Step.Epoch && Step.Sequence == prev.Step.Sequence + 1 &&
                Local == prev.Local + 1 && Body == prev.Body && Floor == prev.Floor && Root == prev.Root &&
                Was == prev.Active && SourceContinuity == prev.SourceContinuity && Step.FixedTime >= prev.Step.FixedTime && Step.UnscaledTime >= prev.Step.UnscaledTime &&
                (Source.Matches(prev.Source) || (Source.Sequence > prev.Source.Sequence && Source.Time > prev.Source.Time && Source.Dt > 0f &&
                    Source.Dt < .25f && Source.Time - prev.Source.Time < .25f));
        }

        private readonly Action<string> _write;
        private Foot _left, _right;
        private string _motion;
        private int _origin, _first, _issue, _upper, _otherUpper, _pelvis, _chest;
        private float _startTime;
        private bool _pending;
        internal LandingPulseWindow(Action<string> write) { _write=write; }

        // Reserve the ordinary evidence slot only on steps on which a decision is possible.
        internal bool MayChoose(in PhysicsStepContext step, string motion) =>
            _left.Valid && _left.Step.Epoch == step.Epoch && _left.Step.Sequence + 1 == step.Sequence &&
            ((_origin != 0 && _issue == 0 && _first == 0) ||
             (_motion == "IDLE" && motion == "TURN" && _left.Active && _right.Active));

        internal bool Advance(in PhysicsStepContext step, string motion, in Foot l, in Foot r,
            in Command lc, in Command rc)
        {
            if (_left.Step.Matches(step)) return false;
            if (!l.Valid || !r.Valid || !l.Step.Matches(step) || !r.Step.Matches(step) ||
                !l.Source.Matches(r.Source) || l.Floor != r.Floor || l.Root != r.Root || l.Body == r.Body ||
                l.Local != r.Local || !lc.Step.Matches(step) || !rc.Step.Matches(step) || lc.Fixed != l.Local || rc.Fixed != r.Local ||
                !lc.JointSolved || !rc.JointSolved || lc.ParentId == 0 || lc.ParentId != rc.ParentId ||
                lc.ChildId == 0 || rc.ChildId == 0 || lc.ChildId == rc.ChildId ||
                lc.ParentInput.ChestBodyId == 0 || lc.ParentInput.ChestBodyId == lc.ParentId ||
                rc.ParentInput.ChestBodyId != lc.ParentInput.ChestBodyId || !float.IsFinite(lc.ParentV.y))
            { Reset("ClockOrBody"); return false; }
            bool continuous = l.Follows(_left) && r.Follows(_right);
            if (!continuous) Reset("Discontinuity");
            bool opens = continuous && _motion == "IDLE" && motion == "TURN" && _left.Active && _right.Active &&
                l.Was && r.Was && !l.Active && !r.Active && l.LiftRelease && r.LiftRelease;
            if (_origin != 0)
            {
                if (lc.ChildId != _upper || rc.ChildId != _otherUpper || lc.ParentId != _pelvis ||
                    lc.ParentInput.ChestBodyId != _chest || rc.ParentInput.ChestBodyId != _chest) Close("BodyChanged");
                else if (_pending)
                {
                    _write?.Invoke("READ/start-from-to=" + _origin + "|" + _issue + "|" + l.Local +
                        "/actualLeftHipVy=" + F(lc.ParentV.y) + "/positive=" + (lc.ParentV.y > 0f));
                    _pending=false;
                }
                if (_origin != 0 && (l.Was && !l.Active || step.FixedTime - _startTime > 1.5f))
                    Close(l.Was && !l.Active ? "NextRelease" : "TimeLimit");
            }
            if (opens)
            {
                _origin=l.Local; _startTime=step.FixedTime;
                _upper=lc.ChildId; _otherUpper=rc.ChildId; _pelvis=lc.ParentId; _chest=lc.ParentInput.ChestBodyId;
            }
            _left=l; _right=r; _motion=motion;
            if (_origin == 0) return false;
            if (_first != 0)
            {
                if (l.Recovery && l.Active) Close("Recovered",l.Local);
                else if (!l.PendingMiss || l.Miss) Close("HoldEnded",l.Local);
            }
            else if (l.Miss || l.Ordinary || l.Recovery)
            {
                _first=l.Local;
                _write?.Invoke("FIRST/start-issue-first=" + _origin + "|" + _issue + "|" + _first +
                    "/entry=" + (l.Miss ? "Miss" : l.Ordinary ? "Ordinary" : "Recovery") +
                    "/sole-gapMm=" + (l.HasEntryGeometry ? F(l.Sole*1000f) + "|" + F(l.Gap*1000f) : "NA|NA"));
                if (l.Ordinary && l.Active) Close("Ordinary",l.Local);
                else if (l.Recovery && l.Active) Close("RecoveryOfEarlierMiss",l.Local);
            }
            return _origin != 0 && _first == 0 && _issue == 0 && l.Free && r.Free;
        }

        internal void RecordIssue(in PhysicsStepContext step, in LandingRiseAllocation.Allocation choice,
            float preV, bool applied)
        {
            if (_origin == 0 || _first != 0 || _issue != 0 || !_left.Step.Matches(step) || choice.Kind == LandingRiseAllocation.Choice.None) return;
            _issue=_left.Local; _pending=true;
            _write?.Invoke("ISSUE/start-local-shared-source=" + _origin + "|" + _issue + "|" + step.Sequence + "|" + step.SourceSequence +
                "/choice=" + choice.Kind + "/applied=" + applied + "/leftHipVy(pre-native-selected)=" + F(preV) + "|" +
                F(preV + step.Dt * choice.Native.SocketA) + "|" + F(preV + step.Dt * choice.Selected.SocketA) +
                "/allocatedY(native-selected)=" + F(choice.Native.System.y) + "|" + F(choice.Selected.System.y) +
                "/angularScale(native-selected)=" + F(choice.Native.AngularScale) + "|" + F(choice.Selected.AngularScale) +
                "/rightForceDeltaN=" + F((choice.Selected.RightForce-choice.Native.RightForce).magnitude));
        }

        internal void Reset(string reason)
        {
            Close(reason);
            _left=_right=default; _motion=null;
        }

        private void Close(string reason, int local = 0)
        {
            if (_origin != 0)
                _write?.Invoke("END/start-issue-first-end=" + _origin + "|" + _issue + "|" + _first + "|" + local +
                    "/result=" + reason + "/nextReadPending=" + _pending);
            _origin=_first=_issue=0; _pending=false;
        }
        private static string F(float value) => value.ToString("G9",CultureInfo.InvariantCulture);
    }
}
