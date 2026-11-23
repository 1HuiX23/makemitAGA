using System.Collections.Generic;
using System.Text;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralFootTransitionTrace
    {
        private sealed partial class Window
        {
            internal LeftLandingPath NativeLandingPath;
        }

        // Keep the native path even after one or both simulated trajectories stop.
        // No formatting in the physics tick and no repeated floor query.
        private sealed partial class LeftLandingPath
        {
            private struct Point
            {
                internal int Local, Source;
                internal FootLandingOpportunityAudit.Phase Phase;
                internal float Min, Gap, SourceMin, ControlMin;
                internal bool References;
                internal LandingHeightBreakdown Height;
            }
            private readonly List<Point> _points = new List<Point>(16);
            internal int FirstGapOver, PeakGapAt, Omitted;
            internal float PeakGap;

            internal void Observe(in Tick t)
            {
                float gap = (t.Foot - t.Lower).magnitude;
                if (FirstGapOver == 0 && gap > TurnHandoffTrajectory.GapLimit) FirstGapOver = t.Local;
                if (PeakGapAt == 0 || gap > PeakGap) { PeakGapAt = t.Local; PeakGap = gap; }
                if (_points.Count == LandingEventHistory.RingCapacity) { Omitted++; return; }
                _points.Add(new Point { Local = t.Local, Source = t.Source.Clock.Sequence, Phase = t.Phase,
                    Min = t.BoxMin, Gap = gap, Height = t.Height, References = t.EntryReferenceValid, SourceMin = t.SourceMin, ControlMin = t.ControlMin });
            }
            internal void Append(StringBuilder b, int start)
            {
                b.Append("\nTURN_LANDING_NATIVE_V041700/start=").Append(start).Append("/omitted=").Append(Omitted)
                    .Append("/columns=local,source,gait,actualMm,sourceMm,controlMm,ankleGapMm");
                foreach (var p in _points)
                    b.Append("\nN170,").Append(p.Local).Append(',').Append(p.Source).Append(',').Append(p.Phase).Append(',')
                        .Append(F(p.Min * 1000f)).Append(',').Append(p.References ? F(p.SourceMin * 1000f) : "NA").Append(',')
                        .Append(p.References ? F(p.ControlMin * 1000f) : "NA").Append(',').Append(F(p.Gap * 1000f));
            }
        }

        internal bool WantsLeftHeight => !_sealed && _release[0]?.Type == Kind.PairRelease && _release[0].First.Foot.Local == 0;

        private enum Endpoint { Unavailable, Fail, Pass }

        private static Endpoint EndpointAt(TurnHandoffWindow lane, int local) =>
            local > 0 && lane.TryReadPass(local, out bool pass) ? (pass ? Endpoint.Pass : Endpoint.Fail) : Endpoint.Unavailable;

        private static int FirstOpportunity(Window w)
        {
            var t = w.First.Foot;
            return t.EventFloor == t.Floor && ((t.Miss && !t.Active) || (t.Ordinary && t.Active)) ? t.Local : 0;
        }

        private static void AppendLeftLanding(StringBuilder b, Window w)
        {
            int start = w.Handoff.Start, third = start + TurnHandoffTrajectory.ComparisonDepth, first = FirstOpportunity(w);
            b.Append("\nTURN_LANDING_LINK_V041700/start-third-first=").Append(start).Append('|').Append(third).Append('|').Append(first)
                .Append("/third(reactive-early)=").Append(EndpointAt(w.Handoff, third)).Append('|').Append(EndpointAt(w.Early, third))
                .Append("/first(reactive-early)=").Append(EndpointAt(w.Handoff, first)).Append('|').Append(EndpointAt(w.Early, first))
                .Append("/native-resolved=").Append(w.Landing).Append('|').Append(w.Resolved.Foot.Local)
                .Append("/reactiveEnd-earlyEnd=").Append(w.Handoff.End ?? "Open").Append('|').Append(w.Early.End ?? "Open")
                .Append("/actualGap(firstOver5At-peakAt-peakMm)=").Append(w.NativeLandingPath.FirstGapOver).Append('|').Append(w.NativeLandingPath.PeakGapAt)
                .Append('|').Append(w.NativeLandingPath.PeakGapAt > 0 ? F(1000f * w.NativeLandingPath.PeakGap) : "NA");
            if (first > 0)
            {
                var t = w.First.Foot;
                b.Append("/firstActual(soleMm-gapMm)=").Append(F(t.BoxMin * 1000f)).Append('|').Append(F((t.Foot - t.Lower).magnitude * 1000f));
            }
            w.NativeLandingPath.Append(b, start);
        }

        private void AppendLeftLandingCounts(StringBuilder b)
        {
            int started = 0, native = 0, paired = 0, original = 0, early = 0, newAtFirst = 0, lost = 0;
            int newThird = 0, retained = 0, failed = 0, unavailable = 0;
            foreach (var w in _windows)
            {
                if (w.Early == null) continue;
                started++;
                int third = w.Handoff.Start + TurnHandoffTrajectory.ComparisonDepth, first = FirstOpportunity(w);
                if (first > 0) native++;
                var reactiveFirst = EndpointAt(w.Handoff, first);
                var earlyFirst = EndpointAt(w.Early, first);
                if (reactiveFirst != Endpoint.Unavailable && earlyFirst != Endpoint.Unavailable)
                {
                    paired++;
                    if (reactiveFirst == Endpoint.Pass) original++;
                    if (earlyFirst == Endpoint.Pass) early++;
                    if (reactiveFirst == Endpoint.Fail && earlyFirst == Endpoint.Pass) newAtFirst++;
                    if (reactiveFirst == Endpoint.Pass && earlyFirst == Endpoint.Fail) lost++;
                }
                if (EndpointAt(w.Handoff, third) != Endpoint.Fail || EndpointAt(w.Early, third) != Endpoint.Pass) continue;
                newThird++;
                if (earlyFirst == Endpoint.Unavailable) unavailable++;
                else if (earlyFirst == Endpoint.Pass) retained++;
                else failed++;
            }
            AppendTractionCounts(b);
            b.Append("\nTURN_LANDING_COUNTS_V041700/started-nativeFirst-pairedFirst=").Append(started).Append('|').Append(native).Append('|').Append(paired)
                .Append("/pairedFirst(reactivePass-earlyPass-new-lost)=").Append(original).Append('|').Append(early).Append('|').Append(newAtFirst).Append('|').Append(lost)
                .Append("/newAtThird-earlyFirst(pass-fail-unavailable)=").Append(newThird).Append('|').Append(retained).Append('|').Append(failed).Append('|').Append(unavailable);
        }
    }
}
