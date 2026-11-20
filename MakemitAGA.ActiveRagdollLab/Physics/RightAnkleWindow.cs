using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;
using Reject = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Rejection;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Last four contiguous right Swing samples, including samples rejected only for ankle gap.
    // The existing live warning exclusively owns trial eligibility; history never feeds that decision.
    internal sealed class RightAnkleWindow
    {
        private const int HistorySize = 4, Capacity = 128;
        private struct Point
        {
            internal int Local, Source;
            internal float Gap, VerticalGap, RawBox, Box, HipRaw, HipRef, Hip, KneeRaw, Knee, AnkleRaw, Ankle;
            internal bool HasHip, HasKnee, HasAnkle;
            internal Point(in Sample s)
            {
                Local = s.Fixed; Source = s.Source.Sequence; Gap = s.AnchorGap; RawBox = s.RawMin; Box = s.Min;
                float floor = s.FloorCenter.y + s.FloorHalf.y;
                HasHip = s.ParentReady; HipRaw = s.RawHipY - floor; HipRef = s.ReferenceHipY - floor; Hip = s.ActualHipY - floor;
                HasKnee = s.KneeHeightsValid && PelvisParentFrame.V(s.KneeHeights);
                KneeRaw = s.KneeHeights.x - floor; Knee = s.KneeHeights.z - floor;
                HasAnkle = s.AnkleHeightsValid && PelvisParentFrame.V(s.AnkleHeights);
                AnkleRaw = s.AnkleHeights.x - floor; Ankle = s.AnkleHeights.y - floor; VerticalGap = s.AnkleHeights.z - s.AnkleHeights.y;
            }
            internal void Append(StringBuilder b)
            {
                b.Append("/local-source=").Append(Local).Append('|').Append(Source)
                    .Append("/mm(gap-verticalGap-rawBox-box)=").Append(M(Gap)).Append('|').Append(HasAnkle ? M(VerticalGap) : "NA")
                    .Append('|').Append(M(RawBox)).Append('|').Append(M(Box))
                    .Append("/hipMm(raw-reference-actual)=").Append(HasHip ? M(HipRaw) + "|" + M(HipRef) + "|" + M(Hip) : "NA")
                    .Append("/kneeMm(raw-actual)=").Append(HasKnee ? M(KneeRaw) + "|" + M(Knee) : "NA")
                    .Append("/ankleMm(raw-lower)=").Append(HasAnkle ? M(AnkleRaw) + "|" + M(Ankle) : "NA");
            }
        }
        private struct Row
        {
            internal int Epoch, Shared, Support;
            internal Point Onset;
            internal bool NextMeasured, NextKneeMeasured;
            internal float NextKnee;
            internal float NextBox, NextAnkle;
            internal int NextSource;
            internal Point[] Previous;
            internal RightAnkleProbe.Result Result;
        }
        private readonly Point[] _history = new Point[HistorySize];
        private readonly Row[] _rows = new Row[Capacity];
        private readonly int[] _outcomes = new int[(int)RightAnkleProbe.Status.Fault + 1];
        private int _count, _next, _total, _saved, _local, _support;
        private PhysicsStepContext _step;
        private SourceStepContext _source;
        private Vector3 _center, _half;
        private bool _pending;
        private int _pendingRow, _resolved, _censored;
        internal void Cut()
        {
            if (_pending) { _censored++; _pending = false; }
            _count = _next = _local = 0;
        }

        private bool Adjacent(in Sample s) => Adjacent(_local, _step, _source, _support, _center, _half, s);
        internal static bool Adjacent(int local, in PhysicsStepContext step, in SourceStepContext source, int support,
            Vector3 center, Vector3 half, in Sample s) => local > 0 && s.Fixed == local + 1 && s.Step.Sequence == step.Sequence + 1 &&
            step.Epoch == s.Step.Epoch && s.Step.FixedTime >= step.FixedTime && s.Step.UnscaledTime >= step.UnscaledTime &&
            s.Source.Sequence >= source.Sequence && s.Source.Time >= source.Time &&
            (s.Source.Sequence != source.Sequence || s.Source.Matches(source)) && support == s.SupportId &&
            (center - s.FloorCenter).sqrMagnitude <= 1e-12f && (half - s.FloorHalf).sqrMagnitude <= 1e-12f;

        private void ReadNext(in Sample s)
        {
            if (!_pending) return;
            _pending = false;
            if (!Adjacent(s) || !s.ClockValid || s.EarlyFoot != EarlyLandingWindow.Foot.Right || s.WasActive ||
                !s.HeightValid || !s.Identity || !s.FivePoint || !s.FullFootprint || s.SelectedBoxFallback ||
                !s.AnkleHeightsValid || !PelvisParentFrame.V(s.AnkleHeights))
            { _censored++; return; }
            _resolved++;
            if (_pendingRow < 0) return;
            ref Row row = ref _rows[_pendingRow];
            row.NextMeasured = true; row.NextSource = s.Source.Sequence;
            row.NextAnkle = s.AnkleHeights.y; row.NextBox = s.Min;
            row.NextKneeMeasured = s.KneeHeightsValid && PelvisParentFrame.V(s.KneeHeights);
            if (row.NextKneeMeasured) row.NextKnee = s.KneeHeights.z - (s.FloorCenter.y + s.FloorHalf.y);
        }

        internal static bool InSwingContext(in Sample s)
        {
            const Reject required = Reject.Geometry | Reject.Footprint | Reject.Motion | Reject.Age | Reject.FivePoint | Reject.Identity;
            return s.EarlyFoot == EarlyLandingWindow.Foot.Right && s.ClockValid && !s.WasActive && !s.NowActive &&
                !s.FirstMiss && !s.MissInProgress && s.Gait == FootLandingOpportunityAudit.Phase.Swing &&
                s.SourcePhase == "STEADY" && s.SourceMotion == FootLandingOpportunityAudit.SourceKind.Motion &&
                s.BoxReferencesValid && !s.SelectedBoxFallback && PelvisParentFrame.F(s.RawMin) &&
                PelvisParentFrame.F(s.AnchorGap) && s.AnchorGap >= 0f && (FootLandingOpportunityAudit.Screen(s) & required) == 0;
        }
        internal static bool Opens(in Sample s) => s.EarlyFoot == EarlyLandingWindow.Foot.Right && s.Fixed > 0 &&
            s.EarlySignal == EarlyLandingWindow.Signal.RightNearLow && s.RightWarningOrigin == s.Fixed;

        internal void Observe(in Sample s, in RightAnkleProbe.Result result)
        {
            ReadNext(s);
            if (!InSwingContext(s)) { Cut(); return; }
            if (_local != 0 && s.Step.Epoch == _step.Epoch && (s.Fixed <= _local || s.Step.Sequence <= _step.Sequence ||
                s.Step.FixedTime < _step.FixedTime || s.Step.UnscaledTime < _step.UnscaledTime || s.Source.Sequence < _source.Sequence ||
                s.Source.Time < _source.Time || (s.Source.Sequence == _source.Sequence && !s.Source.Matches(_source))))
            { Cut(); return; }
            if (!Adjacent(s)) Cut();
            _local = s.Fixed; _step = s.Step; _source = s.Source; _support = s.SupportId; _center = s.FloorCenter; _half = s.FloorHalf;
            var point = new Point(s);
            if (Opens(s))
            {
                _total++; _outcomes[(int)result.State]++;
                _pending = true; _pendingRow = _saved < Capacity ? _saved : -1;
                if (_saved < Capacity)
                {
                    var previous = new Point[_count];
                    for (int i = 0; i < _count; i++) previous[i] = _history[(_next + HistorySize - _count + i) % HistorySize];
                    _rows[_saved++] = new Row
                    {
                        Epoch = s.Step.Epoch,
                        Shared = s.Step.Sequence,
                        Support = s.SupportId,
                        Onset = point,
                        Previous = previous,
                        Result = result
                    };
                }
            }
            _history[_next] = point; _next = (_next + 1) % HistorySize; if (_count < HistorySize) _count++;
        }

        private static string N(float v) => v.ToString("G8", CultureInfo.InvariantCulture);
        private static string M(float v) => N(v * 1000f);
        internal string Summary()
        {
            var b = new StringBuilder("\nRIGHT_ANKLE_V041480/starts-saved-omitted=").Append(_total).Append('|').Append(_saved)
                .Append('|').Append(_total - _saved).Append("/outcomes(")
                .Append(string.Join("-", Enum.GetNames(typeof(RightAnkleProbe.Status)))).Append(")=").Append(string.Join("|", _outcomes))
                .Append("/next(resolved-censored-pending)=").Append(_resolved).Append('|').Append(_censored).Append('|').Append(_pending ? 1 : 0)
                .Append("/model=ONE_FIXED_FREE_ANGULAR_STEP_FIXED_KNEE_GAP_AND_FOOT_RELATIVE/port=KNEE_ACCELERATION/feedbackToControl=False");
            for (int i = 0; i < _saved; i++)
            {
                var row = _rows[i]; var r = row.Result;
                foreach (var p in row.Previous)
                {
                    b.Append("\nRIGHT_ANKLE_PRE_V041480/onset=").Append(row.Onset.Local);
                    p.Append(b);
                }
                b.Append("\nRIGHT_ANKLE_STEP_V041480/epoch-shared-support=").Append(row.Epoch).Append('|').Append(row.Shared)
                    .Append('|').Append(row.Support);
                row.Onset.Append(b);
                b.Append("/previous=").Append(row.Previous.Length).Append("/result=").Append(r.State)
                    .Append("/lanes-legal=").Append(r.Lanes).Append('|').Append(r.Legal)
                    .Append("/baseMeasured=").Append(r.BaseMeasured).Append("/candidateMeasured=").Append(r.CandidateMeasured);
                b.Append("/nextMeasured=").Append(row.NextMeasured);
                if (row.NextMeasured)
                {
                    b.Append("/nextSource=").Append(row.NextSource).Append("/nextBoxMm=").Append(M(row.NextBox));
                    if (r.BaseMeasured)
                        b.Append("/actualMinusBaseNextMm(ankle-box)=").Append(M(row.NextAnkle - r.Baseline.Ankle)).Append('|').Append(M(row.NextBox - r.Baseline.Box));
                }
                if (r.BaseMeasured)
                    b.Append("/baseProjectedMm(ankleMinusRaw-box)=").Append(M(r.Baseline.Ankle - r.TargetAnkle)).Append('|').Append(M(r.Baseline.Box))
                        .Append("/basePD-allocatedFF-flex=").Append(N(r.BasePd)).Append('|').Append(N(r.BaseFf)).Append('|').Append(N(r.BaseFlex))
                        .Append("/limits(refW-refA-flexA-bodyW)=").Append(N(r.MaxW)).Append('|').Append(N(r.MaxA)).Append('|').Append(N(r.FlexLimit)).Append('|').Append(N(r.BodyMaxW));
                if (r.CandidateMeasured)
                    b.Append("/laneSign-index=").Append(r.Sign).Append('|').Append(r.Index)
                        .Append("/referenceGainMm(ankle-box-coastBox)=").Append(M(r.ReferenceAnkleGain)).Append('|').Append(M(r.ReferenceBoxGain)).Append('|').Append(M(r.ReferenceCoastBoxGain))
                        .Append("/trialProjectedMm(ankleMinusRaw-box)=").Append(M(r.Candidate.Ankle - r.TargetAnkle)).Append('|').Append(M(r.Candidate.Box))
                        .Append("/commandGainMm(ankle-box)=").Append(M(r.Candidate.Ankle - r.Baseline.Ankle)).Append('|').Append(M(r.Candidate.Box - r.Baseline.Box))
                        .Append("/trialPD-allocatedFF-flex-clamped=").Append(N(r.TrialPd)).Append('|').Append(N(r.TrialFf)).Append('|').Append(N(r.TrialFlex)).Append('|').Append(r.CandidateClamped)
                        .Append("/refW-refA-baseBodyW-trialBodyW=").Append(N(r.RefW)).Append('|').Append(N(r.RefA)).Append('|').Append(N(r.BaseBodyW)).Append('|').Append(N(r.TrialBodyW));
                if (r.BaseMeasured)
                {
                    b.Append("/pointVelocityForecastMm=").Append(r.PivotForecast ? M(r.PredictedKneeShift) : "NA");
                    if (r.PivotForecast && row.NextMeasured)
                    {
                        b.Append("/actualMinusMovingBaseMm(ankle-box)=")
                            .Append(M(row.NextAnkle - r.Baseline.Ankle - r.PredictedKneeShift)).Append('|')
                            .Append(M(row.NextBox - r.Baseline.Box - r.PredictedKneeShift));
                    }
                }
                if (row.NextKneeMeasured && row.Onset.HasKnee)
                    b.Append("/actualKneeShiftMm=").Append(M(row.NextKnee - row.Onset.Knee));
            }
            return b.ToString();
        }
    }
}
