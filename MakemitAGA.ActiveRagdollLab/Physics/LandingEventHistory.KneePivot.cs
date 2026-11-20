using System.Text;
using Pivot = MakemitAGA.ActiveRagdollLab.Physics.RightKneePivot;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class LandingEventHistory
    {
        // The future landing selects a report window only. A trial reads the pose at N and,
        // for its recent-motion scale, the adjacent N-1. Ordinary entries use the same window.
        private static void AppendPivotWindow(StringBuilder b, Record r, int cycleStart, int at, int edge, string mode)
        {
            var points = r.Points;
            var frames = r.Pivot;
            int end = points[points.Length - 1].Local;
            b.Append("\nRIGHT_PIVOT_WINDOW_V041490/epoch-support=").Append(r.Epoch).Append('|').Append(r.Support)
                .Append("/eventLocal=").Append(end).Append("/kind=").Append(r.Kind)
                .Append("/outcome=").Append(r.Kind == Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open")
                .Append("/align=").Append(mode).Append("/edgeLocal=").Append(edge < 0 ? 0 : points[edge].Local);
            if (at < cycleStart)
            {
                b.Append("/coverage=").Append(edge < 0 ? "EdgeUnavailable" : "NoPriorStep");
                return;
            }
            b.Append("/centerLocal=").Append(points[at].Local).Append("/leadToEdgeMs=").Append(Lead(points, edge, at));
            if (!PivotContext(frames[at]))
            {
                b.Append("/coverage=").Append(frames[at].State).Append("/stored=0");
                return;
            }
            int first = at;
            while (first > cycleStart && at - first < Pivot.PriorSteps && points[first - 1].Local + 1 == points[first].Local &&
                PivotContext(frames[first - 1])) first--;
            b.Append("/coverage=").Append(at - first == Pivot.PriorSteps ? "Full" : "Partial")
                .Append("/stored=").Append(at - first + 1);
            for (int i = first; i <= at; i++)
            {
                ref readonly var p = ref frames[i];
                var state = Pivot.Decompose(p, out var split);
                b.Append("\nRIGHT_PIVOT_STEP_V041490/center-offset=").Append(points[at].Local).Append('|').Append(i - at)
                    .Append("/local-source=").Append(points[i].Local).Append('|').Append(points[i].Source)
                    .Append("/gapMm=").Append(Number(1000d * points[i].Gap))
                    .Append("/gapWithin5mm=").Append((points[i].Rejections & (int)FootLandingOpportunityAudit.Rejection.Gap) == 0)
                    .Append("/state=").Append(state);
                if (state != Pivot.Capture.Ready) continue;
                b.Append("/goalMinusKneeMm=").Append(Number(1000d * split.Total))
                    .Append("/hip-thigh-kneeGapMm=").Append(Number(1000d * split.Hip)).Append('|')
                    .Append(Number(1000d * split.Thigh)).Append('|').Append(Number(1000d * split.KneeGap))
                    .Append("/hipParts(receiver-joint)Mm=").Append(Number(1000d * split.Receiver)).Append('|')
                    .Append(Number(1000d * split.HipJoint)).Append("/closureMm=").Append(Number(1000d * split.Closure))
                    .Append("/originalMm(ankleMinusRaw-box)=").Append(Number(1000d * (p.LowerAnkle.y - p.RawAnkleY)))
                    .Append('|').Append(Number(1000d * split.Box)).Append("/dominant=").Append(split.Dominant);
                bool previous = i > cycleStart && points[i - 1].Local + 1 == points[i].Local &&
                    Pivot.Decompose(frames[i - 1], out _) == Pivot.Capture.Ready;
                if (previous)
                {
                    ref readonly var prior = ref frames[i - 1];
                    float hipMotion = p.UpperHip.y - prior.UpperHip.y;
                    float thighMotion = (p.UpperKnee.y - p.UpperHip.y) - (prior.UpperKnee.y - prior.UpperHip.y);
                    float gapMotion = (p.LowerKnee.y - p.UpperKnee.y) - (prior.LowerKnee.y - prior.UpperKnee.y);
                    b.Append("/incomingMm(hip-thigh-gap-knee)=").Append(Number(1000d * hipMotion)).Append('|')
                        .Append(Number(1000d * thighMotion)).Append('|').Append(Number(1000d * gapMotion)).Append('|')
                        .Append(Number(1000d * (p.LowerKnee.y - prior.LowerKnee.y)));
                }
                if (split.Dominant == Pivot.Part.NoDeficit || split.Dominant == Pivot.Part.Tied) continue;
                AppendPivotTrial(b, "full", Pivot.Compare(p, split, default, false));
                AppendPivotTrial(b, "recent", previous ? Pivot.Compare(p, split, frames[i - 1], true) : default);
            }
        }
        private static bool PivotContext(in Pivot.Frame p) => p.State != Pivot.Capture.Disabled && p.State != Pivot.Capture.OutsideSwing;

        private static void AppendPivotTrial(StringBuilder b, string label, in Pivot.Trial t)
        {
            b.Append('/').Append(label).Append('=');
            if (!t.Measured)
            {
                b.Append("NA");
                return;
            }
            b.Append(Number(t.Angle)).Append('|').Append(Number(1000d * t.KneeY)).Append('|')
                .Append(Number(1000d * t.AnkleY)).Append('|').Append(Number(1000d * t.Box)).Append('|')
                .Append(Number(1000d * t.KneeGain)).Append('|').Append(Number(1000d * t.AnkleGain)).Append('|')
                .Append(Number(1000d * t.DepthGain)).Append('|').Append(Number(1000d * t.AddedClearance)).Append('|')
                .Append(t.Inside).Append('|').Append(t.WasInBand).Append('|').Append(t.InBand);
        }
    }
}
