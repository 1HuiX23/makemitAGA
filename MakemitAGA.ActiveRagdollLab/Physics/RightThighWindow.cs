using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed class RightThighWindow
    {
        internal enum Signal : byte { None, First, Onset, Held }
        private enum Readback : byte { Pending, Measured, Censored }
        internal const float Low = .002f;
        internal const int Capacity = 128;
        internal struct Request
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Local, Support;
            internal RightKneePivot.Frame Pose;
            internal float Gap;
        }
        private struct Row
        {
            internal Request Request;
            internal RightThighTracking.Result Result;
            internal Readback Next;
            internal Vector3 Measured;
            internal int NextSource;
            internal float NextKneeGoal, NextAnkleGoal;
        }
        private readonly Row[] _rows = new Row[Capacity];
        private readonly int[] _outcomes = new int[(int)RightThighTracking.Status.Fault + 1];
        private int _total, _saved, _read, _censored, _pairs, _held, _overGap, _calibrated, _streak;
        private double _kneeError, _ankleError, _boxError;
        private bool _waitingCommand, _waitingNext;
        private Request _stamp;
        private RightThighTracking.Result _last;
        private int _slot;

        internal void Cut()
        {
            if (_waitingCommand) Complete(default);
            if (_waitingNext)
            {
                _censored++;
                if (_slot >= 0) _rows[_slot].Next = Readback.Censored;
            }
            _waitingNext = false; _streak = 0; _stamp = default;
        }
        internal bool Requested(PhysicsStepContext step) => _waitingCommand && _stamp.Step.Matches(step);
        internal Request Current => _stamp;
        internal void Complete(in RightThighTracking.Result result)
        {
            if (!_waitingCommand) return;
            _waitingCommand = false; _last = result; _outcomes[(int)result.State]++;
            if (_slot >= 0) _rows[_slot].Result = result;
        }
        private static bool Adjacent(in Request last, in Sample s) =>
            RightAnkleWindow.Adjacent(last.Local, last.Step, last.Source, last.Support, last.Pose.FloorCenter, last.Pose.FloorHalf, s);

        private void ReadNext(in Sample s, bool adjacent)
        {
            if (_waitingCommand) Complete(default);
            if (!_waitingNext) return;
            if (!adjacent || !s.ClockValid || s.EarlyFoot != EarlyLandingWindow.Foot.Right || s.WasActive || !s.HeightValid ||
                !s.FullFootprint || !s.FivePoint || !s.Identity || s.SelectedBoxFallback || !s.KneeHeightsValid || !s.AnkleHeightsValid ||
                !(s.SourceAge >= 0f && s.SourceAge <= FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds) ||
                !PelvisParentFrame.V(s.KneeHeights) || !PelvisParentFrame.V(s.AnkleHeights)) { Cut(); return; }
            _waitingNext = false; _read++;
            var measured = new Vector3(s.KneeHeights.z, s.AnkleHeights.y, s.Min);
            if (_slot >= 0)
            {
                _rows[_slot].Next = Readback.Measured; _rows[_slot].Measured = measured; _rows[_slot].NextSource = s.Source.Sequence;
                _rows[_slot].NextKneeGoal = s.KneeHeights.y; _rows[_slot].NextAnkleGoal = s.AnkleHeights.x;
            }
            if (_last.HasBase)
            {
                var e = measured - _last.Baseline.Heights;
                _kneeError += Mathf.Abs(e.x); _ankleError += Mathf.Abs(e.y); _boxError += Mathf.Abs(e.z); _calibrated++;
            }
        }
        internal Signal Observe(in Sample s, in RightKneePivot.Frame p)
        {
            bool adjacent = Adjacent(_stamp, s);
            bool stale = _stamp.Local > 0 && s.Step.Epoch == _stamp.Step.Epoch &&
                (s.Fixed <= _stamp.Local || s.Step.Sequence <= _stamp.Step.Sequence || s.Step.FixedTime < _stamp.Step.FixedTime ||
                s.Step.UnscaledTime < _stamp.Step.UnscaledTime || s.Source.Sequence < _stamp.Source.Sequence || s.Source.Time < _stamp.Source.Time ||
                (s.Source.Sequence == _stamp.Source.Sequence && !s.Source.Matches(_stamp.Source)));
            ReadNext(s, adjacent);
            if (stale || !RightAnkleWindow.InSwingContext(s) || !s.KneeHeightsValid || !s.AnkleHeightsValid ||
                !(s.KneeHeights.y - s.KneeHeights.z >= Low) || !(s.AnkleHeights.x - s.AnkleHeights.y >= Low) ||
                !RightKneePivot.Near(p.GoalKneeY,s.KneeHeights.y) || !RightKneePivot.Near(p.LowerKnee.y,s.KneeHeights.z) ||
                !RightKneePivot.Near(p.RawAnkleY,s.AnkleHeights.x) || !RightKneePivot.Near(p.LowerAnkle.y,s.AnkleHeights.y) ||
                !RightKneePivot.Near(p.Box,s.Min) || p.Dt != s.Step.Dt ||
                (p.FloorCenter-s.FloorCenter).sqrMagnitude > 1e-12f || (p.FloorHalf-s.FloorHalf).sqrMagnitude > 1e-12f ||
                RightKneePivot.Decompose(p, out var split) != RightKneePivot.Capture.Ready || split.Dominant != RightKneePivot.Part.Thigh)
            { Cut(); return Signal.None; }
            if (!adjacent) _streak = 0;
            _stamp = new Request { Step = s.Step, Source = s.Source, Local = s.Fixed, Support = s.SupportId, Pose = p, Gap = s.AnchorGap };
            if (_streak == 0) { _streak = 1; return Signal.First; }
            _pairs++;
            if (_streak == 2) { _held++; return Signal.Held; }
            _streak = 2; _total++; _last = default;
            if (s.AnchorGap > FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) _overGap++;
            _slot = _saved < Capacity ? _saved++ : -1;
            if (_slot >= 0) _rows[_slot] = new Row { Request = _stamp };
            _waitingCommand = _waitingNext = true;
            return Signal.Onset;
        }
        private static string Num(double v) => v.ToString("0.######", CultureInfo.InvariantCulture);
        private static string Mm(float v) => Num(1000d * v);
        private static string Triple(Vector3 v) => Mm(v.x) + "|" + Mm(v.y) + "|" + Mm(v.z);
        private static string Relative(Vector3 v, float floor) => Triple(new Vector3(v.x - floor, v.y - floor, v.z));
        internal string Summary()
        {
            Cut(); // Called only on stop. Seal an unresolved origin once, including on repeated reporting.
            var b = new StringBuilder("\nRIGHT_THIGH_TRACKING_V041500/side=RIGHT/observerOnly=True/feedback=False")
                .Append("/thresholdMm(knee-ankle)=2|2/pairs-onsets-held-over5mm=").Append(_pairs).Append('|').Append(_total).Append('|').Append(_held).Append('|').Append(_overGap)
                .Append("/saved-dropped=").Append(_saved).Append('|').Append(_total - _saved)
                .Append("/next(measured-censored)=").Append(_read).Append('|').Append(_censored)
                .Append("/baselineCalibrationN=").Append(_calibrated).Append("/baselineMAEMm(knee-ankle-box)=")
                .Append(_calibrated > 0 ? Num(1000d * _kneeError / _calibrated) + "|" + Num(1000d * _ankleError / _calibrated) + "|" + Num(1000d * _boxError / _calibrated) : "NA")
                .Append("/model=UPPER_POINT_FORCE_AND_NET_ALPHA_LOWER_RELATIVE_PLUS_HIP_NATIVE_GAP_VELOCITY_FOOT_ANGULAR_COAST");
            for (int i = 0; i < _outcomes.Length; i++)
                b.Append('/').Append((RightThighTracking.Status)i).Append('=').Append(_outcomes[i]);
            for (int i = 0; i < _saved; i++)
            {
                ref var row = ref _rows[i]; ref var req = ref row.Request; ref var r = ref row.Result;
                float floor = req.Pose.FloorCenter.y + req.Pose.FloorHalf.y;
                b.Append("\nRIGHT_THIGH_TRIAL_V041500/epoch-shared-local-source-support=").Append(req.Step.Epoch).Append('|').Append(req.Step.Sequence).Append('|')
                    .Append(req.Local).Append('|').Append(req.Source.Sequence).Append('|').Append(req.Support)
                    .Append("/dtMs=").Append(Num(1000d * req.Step.Dt)).Append("/gapMm=").Append(Mm(req.Gap))
                    .Append("/heldGoalMm(knee-ankle)=").Append(Mm(req.Pose.GoalKneeY-floor)).Append('|').Append(Mm(req.Pose.RawAnkleY-floor))
                    .Append("/preMm(knee-ankle-box)=").Append(Relative(new Vector3(req.Pose.LowerKnee.y,req.Pose.LowerAnkle.y,req.Pose.Box),floor))
                    .Append("/path=").Append(r.Path).Append("/outcome=").Append(r.State)
                    .Append("/tried-budgetRejected-geometryRejected=").Append(r.Tried).Append('|').Append(r.BudgetRejected).Append('|').Append(r.GeometryRejected);
                if (r.HasBase) b.Append("/originalMm(knee-ankle-box)=").Append(Relative(r.Baseline.Heights,floor))
                    .Append("/baseRequestA-allocatedA-maxA=").Append(Num(r.BaseRequestA)).Append('|').Append(Num(r.BaseAllocatedA)).Append('|').Append(Num(r.MaxCommandA))
                    .Append("/baseNextW-maxW=").Append(Num(r.BaseNextW)).Append('|').Append(Num(r.MaxSpeed))
                    .Append("/baseTorque-limitNm=").Append(Num(r.BaseTorque)).Append('|').Append(Num(r.TorqueLimit));
                if (r.HasTrial) b.Append("/candidateMm(knee-ankle-box)=").Append(Relative(r.Trial.Heights,floor))
                    .Append("/gainMm(kneeAbs-ankleAbs-depth)=").Append(Triple(r.Gain))
                    .Append("/fraction-addedAlpha=").Append(Num(r.Fraction)).Append('|').Append(Num(r.AddedA))
                    .Append("/candidateRequestA-allocatedA-nextW=").Append(Num(r.TrialRequestA)).Append('|').Append(Num(r.TrialAllocatedA)).Append('|').Append(Num(r.TrialNextW))
                    .Append("/angleToGoalDeg(original-candidate)=").Append(Num(r.Baseline.Angle*Mathf.Rad2Deg)).Append('|').Append(Num(r.Trial.Angle*Mathf.Rad2Deg))
                    .Append("/candidateTorqueNm-leftForceDeltaN-receiverReactionDeltaNm-commonDeltaMps2=").Append(Num(r.TrialTorque)).Append('|')
                    .Append(Num(r.LeftForceDelta)).Append('|').Append(Num(r.ReceiverReactionDelta)).Append('|').Append(Num(r.CommonDelta));
                b.Append("/next=").Append(row.Next);
                if (row.Next == Readback.Measured)
                {
                    b.Append("/nextLocal-source=").Append(req.Local+1).Append('|').Append(row.NextSource)
                        .Append("/actualOriginalMm(knee-ankle-box)=").Append(Relative(row.Measured,floor))
                        .Append("/nextGoalMm(knee-ankle)=").Append(Mm(row.NextKneeGoal-floor)).Append('|').Append(Mm(row.NextAnkleGoal-floor));
                    if (r.HasTrial) b.Append("/gainAgainstNextGoalMm(kneeAbs-ankleAbs)=")
                        .Append(Mm(Mathf.Abs(r.Baseline.Heights.x-row.NextKneeGoal)-Mathf.Abs(r.Trial.Heights.x-row.NextKneeGoal))).Append('|')
                        .Append(Mm(Mathf.Abs(r.Baseline.Heights.y-row.NextAnkleGoal)-Mathf.Abs(r.Trial.Heights.y-row.NextAnkleGoal)));
                }
            }
            return b.ToString();
        }
    }
}
