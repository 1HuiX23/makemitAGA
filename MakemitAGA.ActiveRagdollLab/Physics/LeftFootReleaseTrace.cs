using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Motion = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast.Motion;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One real formal release starts one record. Every tested command is a fresh
    // one-step comparison in 164/165. The 166 foot trial carries its own translation across at most three solves.
    internal sealed partial class LeftFootReleaseTrace
    {
        internal struct Tick
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Local, Support, ReleasedSupport, EventSupport, FormalFoot, FormalLower, FormalUpper;
            internal float Age;
            internal Phase Gait;
            internal string Motion, ReleaseReason;
            internal bool Released, Active, Ordinary, Miss, Recovery, MissInProgress;
            internal bool Clock => Local > 0 && Step.Valid && Source.Valid && Source.Epoch == Step.Epoch &&
                Source.Sequence == Step.SourceSequence;
        }

        internal sealed class Row
        {
            internal Tick From, To;
            internal Forecast.Frame Before, After;
            internal Forecast.Result Command, Corrected;
            internal Motion Motion;
            internal LeftFootLeadSequence.Step Lead;
            internal LeftShinForecast.Result Shin;
            internal bool ShinCompleted, ShinPaired, CorrectedCross;
            internal float CorrectedSole = float.NaN, CorrectedGap = float.NaN;
            internal bool Settled, Cross;
            internal float ConditionalSole = float.NaN, ConditionalGap = float.NaN;
        }

        private sealed class Release
        {
            internal Tick Origin;
            internal LeadEvent Lead;
            internal readonly List<Row> Rows = new List<Row>();
            internal string End, Landing;
            internal int LandingLocal, ResumedLocal;
            internal bool WaitingForLift, SawLift;
        }

        private const int DetailLimit = 64, MaxIntervals = 16;
        // A natural ordinary step in 163 took up to 63 fixed intervals after release.
        // Between capture segments only the clock, gait and first-event link stay live.
        private const double LinkHorizonSeconds = 2d;
        private readonly List<Release> _details = new List<Release>();
        private Release _open;
        private Row _pending;
        private Tick _last;
        private double _age, _windowStartAge;
        private int _windowStartRow;
        private int _releases, _issued, _settled, _models, _trials, _pairedTrials, _crossings, _faults;
        private int _kneeExamined, _kneeFavorable;
        internal bool Tracking => _open != null;
        internal bool Sampling => _open != null && _open.End == null;
        internal bool NeedsFrame(Phase gait) => Sampling || (_open?.WaitingForLift == true && gait == Phase.Lift);
        private static bool Adjacent(in Tick a, in Tick b) => a.Clock && b.Clock && a.Step.Epoch == b.Step.Epoch &&
            a.Step.Sequence + 1 == b.Step.Sequence && a.Local + 1 == b.Local && b.Step.FixedTime >= a.Step.FixedTime &&
            b.Step.UnscaledTime >= a.Step.UnscaledTime && (a.Source.Sequence == b.Source.Sequence ? a.Source.Matches(b.Source) :
                b.Source.Sequence > a.Source.Sequence && b.Source.Time >= a.Source.Time);

        private static bool SameBodiesAndBox(in Forecast.Frame a, in Forecast.Frame b)
        {
            var x = a.Pose.Box; var y = b.Pose.Box;
            return a.FootId == b.FootId && a.LowerId == b.LowerId &&
                (a.UpperId == 0 || b.UpperId == 0 || a.UpperId == b.UpperId) &&
                (x.FloorCenter - y.FloorCenter).sqrMagnitude <= 1e-12f &&
                (x.FloorHalf - y.FloorHalf).sqrMagnitude <= 1e-12f && (x.Half - y.Half).sqrMagnitude <= 1e-12f &&
                (x.Center - y.Center).sqrMagnitude <= 1e-12f;
        }

        internal void Observe(in Tick t, in Forecast.Frame frame, bool unloaded, in Motion motion)
        {
            if (t.Released)
            {
                Close("NextRelease");
                _open = new Release { Origin = t };
                _releases++;
                if (_details.Count < DetailLimit) _details.Add(_open);
                _windowStartRow = 0;
                _windowStartAge = _age = 0d;
            }
            if (_open == null) return;
            if (!t.Clock || (!t.Released && !Adjacent(_last, t))) { Close("Clock"); return; }
            if (t.Support == 0 || t.Support != _open.Origin.ReleasedSupport) { Close("Floor"); return; }
            if (!float.IsFinite(t.Age) || t.Age < 0f || t.Age > FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds)
            { Close("Stale"); return; }
            if (t.Gait == Phase.Unknown) { Close("Gait"); return; }
            if (!t.Released)
            {
                _age += _last.Step.Dt;
                if (_age > LinkHorizonSeconds + 1e-6d) { Close("Horizon"); return; }
                bool newLift = t.Gait == Phase.Lift && (_last.Gait == Phase.Swing || _last.Gait == Phase.Approach);
                if (newLift || (WatchingLeadFormal && LandingWindowOutcomes.StartsNewSwing(t.Gait,_last.Gait)))
                { Close(t.Gait == Phase.Lift ? "NewLift" : "NewSwing"); return; }
            }
            _last = t;
            if (_open.WaitingForLift && t.Gait != Phase.Stance)
            {
                _open.WaitingForLift = false;
                if (t.Gait == Phase.Lift && !t.Active && !t.Miss)
                {
                    // A late normal lift gets one short segment. No row spans the unsampled gap.
                    if (_open.Rows.Count > 0 && frame.Valid && !SameBodiesAndBox(_open.Rows[0].Before, frame))
                    { Close("ChangedBodyOrBox"); return; }
                    _open.End = null;
                    _open.ResumedLocal = t.Local;
                    _windowStartAge = _age;
                    _windowStartRow = _open.Rows.Count;
                }
            }
            if (t.Gait == Phase.Lift) _open.SawLift = true;
            if (Sampling)
            {
                if (!frame.Valid) { _open.End = "MissingPose"; EndLead("MissingPose"); _pending = null; }
                else
                {
                    if (_pending != null)
                    {
                        if (!SameBodiesAndBox(_pending.Before, frame)) { Close("ChangedBodyOrBox"); return; }
                        Settle(t, frame);
                    }
                    if (t.Active) _open.End = "Formal";
                    else if (t.Miss) _open.End = "FirstMiss";
                    else if (!FollowingLead && !t.Released && t.Gait == Phase.Swing) _open.End = "FirstSwing";
                    else if (!FollowingLead && !t.Released && t.Gait == Phase.Approach) _open.End = "Approach";
                    else if (!t.Released && (_open.Origin.Gait == Phase.Swing || _open.Origin.Gait == Phase.Approach))
                        _open.End = "ReleaseSuccessor";
                    else if (!FollowingLead && (_age - _windowStartAge + t.Step.Dt > FootContactGeometryLimits.RecoveryHoldSeconds + 1e-6d ||
                        _open.Rows.Count - _windowStartRow >= MaxIntervals))
                    {
                        _open.End = "WindowLimit";
                        _open.WaitingForLift = _open.Origin.Gait == Phase.Stance && t.Gait == Phase.Stance && !_open.SawLift;
                    }
                    else Issue(t, frame, unloaded, motion);
                }
            }
            LeadOutcome(t);
            if (t.Ordinary || t.Miss || t.Recovery)
            {
                if (_open.Landing == null)
                {
                    _open.LandingLocal = t.Local;
                    _open.Landing = t.EventSupport != t.Support ? "UnverifiedFloor" :
                        t.Miss ? "FirstMiss" : t.Recovery ? "Recovery" : "Ordinary";
                }
                if (WatchingLeadFormal && t.Miss && !t.Active && _open.Landing == "FirstMiss" && t.EventSupport == t.Support)
                {
                    // Keep only the real first formal enable and original miss outcome until the cycle changes.
                    _open.End ??= "FirstMiss";
                    _open.WaitingForLift = false;
                    EndLead("FirstMiss");
                }
                else Close("Landing");
            }
            else if (t.Active) Close("FormalWithoutEvent");
        }

        private void Issue(in Tick t, in Forecast.Frame frame, bool unloaded, in Motion motion)
        {
            var cmd = Forecast.Compare(frame, t.Gait, unloaded, motion);
            if (!cmd.Predicted && FollowingLead) EndLead(cmd.State.ToString());
            _pending = new Row { From = t, Before = frame, Command = cmd, Motion = motion,
                Shin = new LeftShinForecast.Result { State = cmd.Predicted ? LeftShinForecast.Status.MissingReceipt : LeftShinForecast.Status.NotSampled } };
            _open.Rows.Add(_pending);
            _issued++;
            if (frame.KneeExamined)
            {
                _kneeExamined++;
                if (frame.Knee.Reason == SwingKneeCandidate.Status.Favorable) _kneeFavorable++;
            }
            if (cmd.Predicted) _models++;
            if (cmd.State == Forecast.Decision.Trial) _trials++;
        }
        private void Settle(in Tick t, in Forecast.Frame frame)
        {
            var row = _pending;
            row.To = t; row.After = frame; row.Settled = true;
            row.Cross = Forecast.ReadTrial(row.Command, frame, out row.ConditionalSole, out row.ConditionalGap);
            SettleShin(row, frame);
            ReadLead(row, frame);
            _settled++;
            if (row.Command.State == Forecast.Decision.Trial) _pairedTrials++;
            if (row.Cross) _crossings++;
            _pending = null;
        }
        private void Close(string reason)
        {
            if (_open == null) return;
            EndLead(reason);
            if (_open.Lead != null)
            {
                _open.Lead.Formal ??= reason;
                if (_open.Landing == "FirstMiss") _open.Lead.MissOutcome ??= reason;
            }
            _open.End ??= reason;
            _open.Landing ??= reason;
            _pending = null; _open = null;
        }
        internal void Fault() { _faults++; Close("Fault"); }
        internal void Seal() => Close("TraceEnded");

        private static string N(float n) => float.IsFinite(n) ? n.ToString("G8", CultureInfo.InvariantCulture) : "NA";
        private static string Mm(float n) => N(n * 1000f);
        internal string Summary()
        {
            var b = new StringBuilder("\nLEFT_RELEASE_V041640/releases-saved-omitted=").Append(_releases).Append('|')
                .Append(_details.Count).Append('|').Append(_releases - _details.Count)
                .Append("/issued-settled-modeled-trials-pairedTrials-conditionedCross=").Append(_issued).Append('|').Append(_settled)
                .Append('|').Append(_models).Append('|').Append(_trials).Append('|').Append(_pairedTrials).Append('|').Append(_crossings)
                .Append("/kneeExamined-favorable=").Append(_kneeExamined).Append('|').Append(_kneeFavorable)
                .Append("/faults=").Append(_faults).Append("/lowerForecast=PRE_POINT_VELOCITY/plantWrites=0");
            AppendShinSummary(b);
            AppendLeadSummary(b);
            foreach (var e in _details)
            {
                var o = e.Origin;
                b.Append("\nLEFT_RELEASE_EVENT_V041640/epoch-shared-origin-source-support=").Append(o.Step.Epoch).Append('|')
                    .Append(o.Step.Sequence).Append('|').Append(o.Local).Append('|').Append(o.Source.Sequence).Append('|').Append(o.ReleasedSupport)
                    .Append("/gait=").Append(o.Gait).Append("/reason=").Append(o.ReleaseReason)
                    .Append("/exit=").Append(e.End).Append("/firstLanding-local=").Append(e.Landing).Append('|').Append(e.LandingLocal);
                if (e.ResumedLocal > 0) b.Append("/resumedAtLift=").Append(e.ResumedLocal);
                AppendLeadEvent(b,e);
                for (int i = 0; i < e.Rows.Count; i++)
                {
                    var row = e.Rows[i];
                    AppendRow(b, row);
                    AppendShinRow(b, row, i + 1 < e.Rows.Count ? e.Rows[i + 1] : null);
                    AppendLeadRow(b,row);
                }
            }
            return b.ToString();
        }

        private static void AppendRow(StringBuilder b, Row row)
        {
            var t = row.From; var f = row.Before; var p = f.Pose.Box; var cmd = row.Command;
            b.Append("\nLEFT_RELEASE_STEP_V041640/from-to=").Append(t.Local).Append('|').Append(row.Settled ? row.To.Local : 0)
                .Append("/gait=").Append(t.Gait).Append('|').Append(row.Settled ? row.To.Gait.ToString() : "NA")
                .Append("/source=").Append(t.Source.Sequence).Append('|').Append(row.Settled ? row.To.Source.Sequence : 0)
                .Append("/motion=").Append(t.Motion ?? "NA").Append("/dt-ageMs=").Append(Mm(t.Step.Dt)).Append('|').Append(Mm(t.Age))
                .Append("/fivePoint=").Append(f.FivePoint).Append("/decision=").Append(cmd.State)
                .Append("/preSole-gapMm=").Append(Mm(p.Box)).Append('|').Append(Mm(f.Pose.Gap));
            if (f.KneeExamined)
                b.Append("/kneePose=").Append(f.Knee.Detail()).Append("/kneeRoomRadPerSec=")
                    .Append(f.Knee.LaneMeasured ? N(f.Knee.Room) : "NA").Append("/kneeBlock=").Append(f.Knee.Gate ?? "None");
            if (cmd.Predicted)
                b.Append("/originalSole-gapMm=").Append(Mm(cmd.Sole)).Append('|').Append(Mm(cmd.Gap))
                    .Append("/original-trialY-capMps2=").Append(N(cmd.AppliedY)).Append('|').Append(N(cmd.TrialY)).Append('|').Append(N(cmd.Cap))
                    .Append("/availableRise-rise-trialSole-trialGapMm=").Append(Mm(cmd.AvailableRise)).Append('|').Append(Mm(cmd.Rise))
                    .Append('|').Append(Mm(cmd.Sole + cmd.Rise)).Append('|').Append(Mm(cmd.TrialGap));
            if (!row.Settled) return;
            var n = row.After; var q = n.Pose.Box;
            float lower = q.LowerAnkle.y - p.LowerAnkle.y, foot = q.FootAnkle.y - p.FootAnkle.y;
            b.Append("/nextSole-gapMm=").Append(Mm(q.Box)).Append('|').Append(Mm(n.Pose.Gap))
                .Append("/deltaSourceAnkle-hip-knee-lowerAnkle-footAnkle-footMinusLower-soleMm=").Append(Mm(n.SourceAnkleY - f.SourceAnkleY))
                .Append('|').Append(Mm(n.HipY - f.HipY)).Append('|').Append(Mm(n.KneeY - f.KneeY)).Append('|').Append(Mm(lower))
                .Append('|').Append(Mm(foot)).Append('|').Append(Mm(foot - lower)).Append('|').Append(Mm(q.Box - p.Box));
            if (t.Gait == Phase.Lift && row.To.Gait == Phase.Swing &&
                p.Box >= -FootContactGeometryLimits.OrdinaryPenetrationMeters && q.Box < -FootContactGeometryLimits.OrdinaryPenetrationMeters)
            {
                bool fits = Forecast.MeasuredRoom(f.Pose.Gap, q.Box, lower, foot, n.Pose.Gap, q.FootAnkle.y - q.LowerAnkle.y,
                    out float needed, out float excess, out float gapAtDepth);
                b.Append("/measuredNeed-excess-gapAtDepthMm=").Append(Mm(needed)).Append('|').Append(Mm(excess)).Append('|').Append(Mm(gapAtDepth))
                    .Append("/measuredRoom=").Append(fits);
            }
            if (cmd.Predicted)
                b.Append("/measuredMinusForecastLower-foot-soleMm=").Append(Mm(q.LowerAnkle.y - cmd.LowerY))
                    .Append('|').Append(Mm(q.FootAnkle.y - cmd.FootY)).Append('|').Append(Mm(q.Box - cmd.Sole));
            if (cmd.State == Forecast.Decision.Trial)
                b.Append("/conditionedSole-gapMm=").Append(Mm(row.ConditionalSole)).Append('|').Append(Mm(row.ConditionalGap))
                    .Append("/conditionedCross=").Append(row.Cross);
        }
    }
}
