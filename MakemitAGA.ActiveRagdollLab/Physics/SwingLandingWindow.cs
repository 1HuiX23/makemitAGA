using System;
using System.Text;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Same-foot observations in metres. The horizon is accumulated physics time, independent of Render FPS.
    // A source can be held inside the window, but its two endpoints cannot be the same animation sample.
    internal sealed class SwingLandingWindow
    {
        internal enum Signal : byte { None, Low, Worsening, Diverging }
        internal const double Window = .05d;
        private const double Epsilon = 1e-6d;
        private const float Margin = FootContactGeometryLimits.OrdinaryPenetrationMeters;
        private struct Point
        {
            internal double Time;
            internal int Source;
            internal float Box, RawBox, Knee, Ankle;
        }
        private readonly Point[] _points = new Point[32];
        private int _head, _count, _local, _support;
        private Vector3 _center, _half;
        private PhysicsStepContext _step;
        private SourceStepContext _source;
        private double _time;
        private int _seen, _eligible, _compared, _low, _worsening, _diverging, _overwritten, _faults;
        private readonly int[] _outcomes = new int[(int)SwingKneeCandidate.Status.Fault + 1];

        private struct Witness
        {
            internal int Local, Support;
            internal SwingKneeCandidate.Pose Pose;
            internal KneeReferenceState Knee;
            internal SwingKneeCandidate.Result Result;
        }
        private readonly Witness[] _witnesses = new Witness[(int)SwingKneeCandidate.Status.Fault + 1];
        internal void KeepPose(int local, int support, in SwingKneeCandidate.Pose pose,
            in KneeReferenceState knee, in SwingKneeCandidate.Result result)
        {
            int index = (int)result.Reason;
            if (_witnesses[index].Local == 0)
                _witnesses[index] = new Witness { Local=local, Support=support, Pose=pose, Knee=knee, Result=result };
        }
        internal void Cut() { _head = _count = _local = 0; _time = 0d; }
        internal void Reset()
        {
            Cut(); _seen = _eligible = _compared = _low = _worsening = _diverging = _overwritten = _faults = 0;
            Array.Clear(_outcomes, 0, _outcomes.Length); Array.Clear(_witnesses, 0, _witnesses.Length);
        }
        internal void Fault() { _faults++; Cut(); }
        internal void Record(SwingKneeCandidate.Status status) { _outcomes[(int)status]++; }
        private bool Follows(in Sample s) => _local != 0 && s.Fixed == _local + 1 && s.Step.Sequence == _step.Sequence + 1 &&
            s.Step.Epoch == _step.Epoch && s.Step.FixedTime >= _step.FixedTime && s.Step.UnscaledTime >= _step.UnscaledTime &&
            s.Source.Sequence >= _source.Sequence && s.Source.Time >= _source.Time &&
            (s.Source.Sequence != _source.Sequence || s.Source.Matches(_source)) && s.SupportId == _support &&
            (s.FloorCenter - _center).sqrMagnitude <= 1e-12f && (s.FloorHalf - _half).sqrMagnitude <= 1e-12f;

        internal Signal Observe(in Sample s)
        {
            _seen++;
            if (s.WasActive || s.NowActive || !s.ClockValid || s.Gait != FootLandingOpportunityAudit.Phase.Swing ||
                s.SourceMotion != FootLandingOpportunityAudit.SourceKind.Motion || !s.Identity || !s.FivePoint ||
                s.SelectedBoxFallback || !s.FullFootprint || !s.VelocityValid || !s.BoxReferencesValid ||
                !s.KneeHeightsValid || !s.AnkleHeightsValid || !PelvisParentFrame.V(s.KneeHeights) || !PelvisParentFrame.V(s.AnkleHeights) ||
                !PelvisParentFrame.F(s.RawMin) || !PelvisParentFrame.F(s.SourceAge) || s.SourceAge < 0f ||
                s.SourceAge > FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds || !PelvisParentFrame.F(s.AnchorGap) ||
                s.AnchorGap < 0f || s.AnchorGap > FootContactGeometryLimits.RecoveryMaxAnchorGapMeters)
            { Cut(); return Signal.None; }
            if (_local != 0 && s.Step.Epoch == _step.Epoch && (s.Fixed <= _local || s.Step.Sequence <= _step.Sequence ||
                s.Step.FixedTime < _step.FixedTime || s.Step.UnscaledTime < _step.UnscaledTime || s.Source.Sequence < _source.Sequence ||
                s.Source.Time < _source.Time || (s.Source.Sequence == _source.Sequence && !s.Source.Matches(_source))))
            { _faults++; Cut(); return Signal.None; }
            _eligible++;
            if (!Follows(s)) Cut();
            _step = s.Step; _source = s.Source; _support = s.SupportId; _center = s.FloorCenter; _half = s.FloorHalf; _local = s.Fixed;
            _time += s.Step.Dt;
            var now = new Point { Time = _time, Source = s.Source.Sequence, Box = s.Min, RawBox = s.RawMin,
                Knee = s.KneeHeights.z - s.KneeHeights.x, Ankle = s.AnkleHeights.y - s.AnkleHeights.x };
            // Keep the youngest witness at least 50 ms old. Do not interpolate a held animation frame.
            while (_count >= 2 && now.Time - _points[(_head + 1) % _points.Length].Time >= Window - Epsilon)
            { _head = (_head + 1) % _points.Length; _count--; }
            var prior = _points[_head];
            double age = now.Time - prior.Time;
            bool comparable = _count > 0 && age >= Window - Epsilon && age <= .1d + Epsilon && prior.Source < now.Source;
            if (_count == _points.Length) { _head = (_head + 1) % _points.Length; _count--; _overwritten++; }
            _points[(_head + _count) % _points.Length] = now; _count++;
            if (comparable) _compared++;
            if (now.Knee >= -Margin || now.Ankle >= -Margin || now.RawBox > FootContactGeometryLimits.RecoveryMaxGapMeters) return Signal.None;
            _low++;
            if (!comparable || now.Ankle > prior.Ankle - Margin || now.Knee > prior.Knee) return Signal.Low;
            _worsening++;
            if (now.Box >= -Margin || prior.RawBox >= 0f || now.RawBox < prior.RawBox + Margin || now.Box > prior.Box - Margin)
                return Signal.Worsening;
            _diverging++; return Signal.Diverging;
        }

        internal string Summary(bool early = false)
        {
            var b = new StringBuilder();
            if (!early) b.Append("\nSWING_WINDOW_V041450/inactive-eligible-compared=" + _seen + "|" + _eligible + "|" + _compared +
            "/inclusive(low-worsening-diverging)=" + _low + "|" + _worsening + "|" + _diverging +
            "/overwritten-faults=" + _overwritten + "|" + _faults +
            "/probe(none-unavailable-protected-geometry-budget-noRoom-noLegal-favorable-fault)=" + string.Join("|", _outcomes) +
            "/windowMs=50-100/shadowOnly=True");
            foreach (var w in _witnesses)
            {
                if (w.Local == 0) continue;
                var p=w.Pose; var g=p.Reference; var k=w.Knee;
                b.Append(early ? "\nEARLY_POSE_V041460/firstOfReason=" : "\nSWING_POSE_V041450/firstOfReason=").Append(w.Result.Reason).Append("/local=").Append(w.Local)
                    .Append("/support=").Append(w.Support).Append("/epoch-fixed-source=").Append(k.Step.Epoch).Append('|').Append(k.Step.Sequence).Append('|').Append(k.Source.Sequence)
                    .Append("/dt=").Append(N(k.Step.Dt)).Append("/refHip-upper=").Append(V(g.Hip)).Append('|').Append(Q(g.Upper))
                    .Append("/spans=").Append(V(g.UpperSpan)).Append('|').Append(V(g.LowerSpan)).Append("/box=").Append(V(g.Center)).Append('|').Append(V(g.Half))
                    .Append("/floor=").Append(V(g.FloorCenter)).Append('|').Append(V(g.FloorHalf)).Append("/foot(raw-bind-selected)=").Append(Q(g.FootRelative)).Append('|').Append(Q(g.FootBindRelative)).Append('|').Append(Q(p.SelectedFoot))
                    .Append("/physicalKnee-gap=").Append(V(p.PhysicalKnee)).Append('|').Append(V(p.AnkleGap)).Append("/physical(upper-lower-footRel)=").Append(Q(p.PhysicalUpper)).Append('|').Append(Q(p.PhysicalLower)).Append('|').Append(Q(p.PhysicalFootRelative))
                    .Append("/observed(boxMeters-worldLowerY)=").Append(N(p.ObservedBox)).Append('|').Append(N(p.LowerAnkleY)).Append("/hipW-axis-extension=").Append(V(p.HipW)).Append('|').Append(V(p.Axis)).Append('|').Append(p.AllowExtension)
                    .Append("/kneeR(prev-goal-base)=").Append(Q(k.PreviousR)).Append('|').Append(Q(k.InputR)).Append('|').Append(Q(k.R))
                    .Append("/kneeW(prev-goal-base-A)=").Append(V(k.PreviousW)).Append('|').Append(V(k.InputW)).Append('|').Append(V(k.W)).Append('|').Append(V(k.A))
                    .Append("/limits(W-A)=").Append(N(k.MaxW)).Append('|').Append(N(k.MaxA)).Append("/stale-groundApplied-landingApplied=").Append(k.Stale).Append('|').Append(k.GroundTrajectory.Applied).Append('|').Append(k.LandingPreparation.Applied)
                    .Append("/result=").Append(early ? w.Result.EarlyDetail() : w.Result.Detail());
            }
            return b.ToString();
        }
        private static string N(float x) => x.ToString("G9",CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => "("+N(v.x)+","+N(v.y)+","+N(v.z)+")";
        private static string Q(Quaternion q) => "("+N(q.x)+","+N(q.y)+","+N(q.z)+","+N(q.w)+")";
    }
}
