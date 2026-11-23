using System.Collections.Generic;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;
using Prep = MakemitAGA.ActiveRagdollLab.Physics.LeftLegHeightPreparation;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class LeftFootReleaseTrace
    {
        private sealed class PreparationWindow
        {
            internal int Origin, Count;
            internal Prep.Group Group;
            internal readonly Prep.Step[] Steps = new Prep.Step[Prep.Length];
            internal string End;
        }

        private sealed class PreparationWindows
        {
            internal readonly List<PreparationWindow> All = new List<PreparationWindow>();
            internal int Live;
            internal bool HasRising;
        }

        private readonly int[] _prepRead = new int[Prep.Length], _controlRead = new int[Prep.Length];
        private int _descentStarts, _seedPulses, _risingStarts, _missingPreparationInputs;
        private bool FollowingPrediction => FollowingLead || (_open?.Preparation?.Live ?? 0) > 0;

        private void EndPredictions(string reason)
        {
            EndLead(reason);
            StopPreparations(reason);
        }

        private static void FinishWindow(PreparationWindows windows, PreparationWindow window, string reason)
        {
            if (window.End != null) return;
            window.End = reason;
            windows.Live--;
        }

        private void StopPreparations(string reason)
        {
            var windows = _open?.Preparation;
            if (windows == null || windows.Live == 0) return;
            foreach (var w in windows.All) FinishWindow(windows,w,reason);
        }

        private void IssuePreparation(Row row)
        {
            var windows = _open.Preparation;
            bool mayStart = _age-_windowStartAge+row.From.Step.Dt <= FootContactGeometryLimits.RecoveryHoldSeconds+1e-6d &&
                _open.Rows.Count-_windowStartRow <= MaxIntervals;
            var group = mayStart ? Prep.Classify(row.From.Gait,row.Before,row.Corrected,row.Shin) : Prep.Group.None;
            // Capture one naturally rising comparison per release. Descent origins
            // may overlap: a weak early request must not hide the next stronger one.
            if (group == Prep.Group.RisingControl && windows?.HasRising == true) group = Prep.Group.None;
            if (group != Prep.Group.None && (row.Shin.Knee.Cap <= 0f || !row.Motion.MeasuredLowerTransport))
            {
                _missingPreparationInputs++;
                group = Prep.Group.None;
            }
            if (group == Prep.Group.None && (windows?.Live ?? 0) == 0) return;
            row.PreparationCommand = true;
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LeftLegPreparation);
            if (group != Prep.Group.None)
            {
                windows ??= _open.Preparation = new PreparationWindows();
                WatchContact(row.Before);
                windows.All.Add(new PreparationWindow { Origin = row.From.Local, Group = group });
                windows.Live++;
                if (group == Prep.Group.Descent) _descentStarts++;
                else { windows.HasRising = true; _risingStarts++; }
            }
            foreach (var w in windows.All)
            {
                if (w.End != null) continue;
                var previous = w.Count == 0 ? null : w.Steps[w.Count-1];
                var s = Prep.Predict(row.From.Local,w.Group,row.From.Gait,row.Before,row.Corrected,row.Shin,row.Motion,previous);
                w.Steps[w.Count++] = s;
                if (!s.Issued) FinishWindow(windows,w,s.Decision.ToString());
                else if (s.Decision == Prep.Action.Prepare) _seedPulses++;
            }
        }

        private void ReadPreparation(Row r, in Foot.Frame next)
        {
            var windows = _open.Preparation;
            if (windows == null || windows.Live == 0) return;
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LeftLegPreparation);
            if (!r.ShinPaired || next.UpperId != _open.Contact.Upper || !next.FivePoint)
            { StopPreparations(!next.FivePoint ? "ReadFloorWitness" : "MissingAdjacentReceipt"); return; }
            foreach (var w in windows.All)
            {
                if (w.End != null) continue;
                var s = w.Steps[w.Count-1];
                if (s.Local != r.From.Local || s.Local+1 != r.To.Local)
                { FinishWindow(windows,w,"MissingAdjacentReceipt"); continue; }
                Prep.Readback(s,next,r.Shin.Knee.HardGap,r.Motion.Tether.HardGap);
                if (!s.Read) { FinishWindow(windows,w,"MissingPose"); continue; }
                var counts = w.Group == Prep.Group.Descent ? _prepRead : _controlRead;
                counts[s.Depth-1]++;
                if (!s.Inside) FinishWindow(windows,w,"ReadFootprint");
                else if (s.ReadHard) FinishWindow(windows,w,"ReadHardGap");
                else if (w.Count == Prep.Length) FinishWindow(windows,w,"ThreeSteps");
            }
        }

        private void AppendPreparationSummary(StringBuilder b)
        {
            b.Append("\nLEFT_LEG_PREPARATION_V041670/descent-pulses-risingControls=")
                .Append(_descentStarts).Append('|').Append(_seedPulses).Append('|').Append(_risingStarts)
                .Append("/descentRead1-2-3=").Append(_prepRead[0]).Append('|').Append(_prepRead[1]).Append('|').Append(_prepRead[2])
                .Append("/risingRead1-2-3=").Append(_controlRead[0]).Append('|').Append(_controlRead[1]).Append('|').Append(_controlRead[2])
                .Append("/missingInputAtOrigin=").Append(_missingPreparationInputs).Append("/plantWrites=0");
        }

        private static void AppendPreparationCommand(StringBuilder b, Row row)
        {
            if (!row.PreparationCommand) return;
            var k = row.Shin.Knee;
            var m = row.Motion;
            b.Append("\nLEFT_LEG_PREPARATION_COMMAND_V041670/from=").Append(row.From.Local)
                .Append("/kneeRaw-actualA=").Append(Vec(k.Raw,1)).Append('|').Append(Vec(k.Applied,1))
                .Append("/footRaw-actualA=").Append(Vec(m.Tether.Raw,1)).Append('|').Append(Vec(m.AppliedLinearA,1))
                .Append("/dt-lowerDrag-footDrag=").Append(N(row.From.Step.Dt)).Append('|').Append(N(k.Drag)).Append('|').Append(N(m.LinearDrag))
                .Append("/kneeKp-Kd-cap=").Append(N(k.Kp)).Append('|').Append(N(k.Kd)).Append('|').Append(N(k.Cap))
                .Append("/footKp-Kd-baseCap-selectedCap=").Append(N(m.Tether.Kp)).Append('|').Append(N(m.Tether.Kd))
                .Append('|').Append(N(m.Tether.BaseCap)).Append('|').Append(N(m.Tether.SelectedCap))
                .Append("/parentTransport=").Append(m.MeasuredLowerTransport);
        }

        private static void AppendPreparations(StringBuilder b, Release release)
        {
            if (release.Preparation == null) return;
            foreach (var w in release.Preparation.All)
            {
                b.Append("\nLEFT_LEG_PREPARATION_EVENT_V041670/epoch-support-release-origin=").Append(release.Origin.Step.Epoch)
                    .Append('|').Append(release.Origin.ReleasedSupport).Append('|').Append(release.Origin.Local).Append('|').Append(w.Origin)
                    .Append("/group=").Append(w.Group).Append("/end=").Append(w.End ?? "Pending");
                AppendActualContact(b,release);
                for (int i=0; i<w.Count; i++)
                {
                    var s = w.Steps[i];
                    b.Append("\nLEFT_LEG_PREPARATION_STEP_V041670/origin-depth-from-to=").Append(w.Origin).Append('|').Append(s.Depth)
                        .Append('|').Append(s.Local).Append('|').Append(s.Read ? s.Local+1 : 0)
                        .Append("/action=").Append(s.Decision).Append("/available-usedRiseMm=").Append(Mm(s.AvailableRise)).Append('|').Append(Mm(s.SeedRise));
                    if (!s.Issued) continue;
                    b.Append("/kneeA-footA-parentDeltaA=").Append(Vec(s.KneeA,1)).Append('|').Append(Vec(s.FootA,1)).Append('|').Append(Vec(s.ParentDeltaA,1))
                        .Append("/lower-footOffsetMm=").Append(Vec(s.Lower.Position,1000)).Append('|').Append(Vec(s.FootBody.Position,1000))
                        .Append("/predSole-ankle-knee-baseKneeMm=").Append(Mm(s.PredSole)).Append('|').Append(Mm(s.PredAnkle))
                        .Append('|').Append(Mm(s.PredKnee)).Append('|').Append(Mm(s.BaseKnee));
                    if (s.Combined != null)
                    {
                        var c = s.Combined;
                        b.Append("/combinedHorizontal-vertical=").Append(c.Horizontal?.ToString() ?? "NA").Append('|').Append(c.Vertical?.ToString() ?? "NA")
                            .Append("/combinedAddedXZ-riseMm=").Append(Mm(c.AddedXZ)).Append('|').Append(Mm(c.Rise))
                            .Append("/combinedA=").Append(Vec(c.Applied,1)).Append("/combinedPredSole-ankleMm=")
                            .Append(Mm(c.PredSole)).Append('|').Append(Mm(c.PredAnkle)).Append("/combinedEnd=").Append(c.End ?? "NA");
                        if (c.Read) b.Append("/combinedConditionedSole-ankleMm=").Append(Mm(c.Sole)).Append('|')
                            .Append(Mm(c.Ankle)).Append("/combinedPass=").Append(c.Pass);
                    }
                    if (!s.Read) continue;
                    b.Append("/actualSole-ankle-kneeMm=").Append(Mm(s.ActualSole)).Append('|').Append(Mm(s.ActualAnkle)).Append('|').Append(Mm(s.ActualKnee))
                        .Append("/conditionedSole-ankle-kneeMm=").Append(Mm(s.TrialSole)).Append('|').Append(Mm(s.TrialAnkle)).Append('|').Append(Mm(s.TrialKnee))
                        .Append("/bestFixedPoseMm=").Append(Mm(s.BestFixedPose)).Append("/inside-actualPass-trialPass=")
                        .Append(s.Inside).Append('|').Append(s.ActualPass).Append('|').Append(s.TrialPass);
                }
            }
        }
    }
}
