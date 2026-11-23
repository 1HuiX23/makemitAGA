using System.Text;
using UnityEngine;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class TurnHandoffWindow
    {
        private readonly CombinedLandingSearch _combined;

        internal void AppendCombined(StringBuilder b, in Trace.Tick first, string nativeOutcome, int resolved,
            int firstEligible, TurnHandoffWindow braked)
        {
            var path = _combined.LastRead;
            var outcome = _combined.Outcome ?? End ?? "Open";
            bool scoredFirst = first.Local > 0 && _combined.LastReadLocal == first.Local &&
                _combined.EndLocal == first.Local && (outcome == "WouldEnter" || outcome == "DepthLatched" || outcome == "NotReadyAtFirst");
            bool bufferedFirst = outcome == "WouldEnter" && path.Entry.StrongSamples > 0 &&
                path.Entry.StrongStart == _nativeEntry.StrongStart && path.Entry.StrongSamples == path.MarginSamples;
            int shoe = 0, pre = 0, lower = 0, both = 0;
            if (scoredFirst)
            {
                var step = _rows[first.Local - Start - 1].Combined;
                for (int i = 0; i < step.Count; i++)
                {
                    ref var p = ref step.Paths[i];
                    if (!p.SafeRead || p.Entry.DepthLatch != 0 || p.Entry.StrongStart != _nativeEntry.StrongStart ||
                        p.Entry.StrongSamples == 0 || p.Entry.StrongSamples != p.MarginSamples ||
                        !first.Entry.CanEnter(p.Entry.ReadySamples)) continue;
                    shoe++;
                    if (p.FirstPreExcess == 0) pre++;
                    if (p.FirstLowerExcess == 0) lower++;
                    if (p.FirstPreExcess == 0 && p.FirstLowerExcess == 0) both++;
                }
            }
            b.Append("\nTURN_COMBINED_V041780/start-first-resolved=").Append(Start).Append('|').Append(first.Local).Append('|').Append(resolved)
                .Append("/native=").Append(nativeOutcome ?? "Open").Append("/nativeFirstEligible-depthLatch=").Append(firstEligible).Append('|').Append(first.NativeDepthLatch)
                .Append("/sequenceEnd=").Append(outcome).Append("/stopLocal=").Append(_combined.Outcome == null ? _lastVisit : _combined.EndLocal)
                .Append("/inputEnd=").Append(End ?? "Open").Append("/inputs=").Append(_rows.Count)
                .Append("/lastSafeRead=").Append(_combined.LastReadLocal).Append("/allMarginsAtNativeFirst=").Append(scoredFirst ? bufferedFirst.ToString() : "Unavailable")
                .Append("/width=").Append(CombinedLandingSearch.Width).Append("/path-protected=").Append(path.Id).Append('|').Append(path.Protected)
                .Append("/strongSteps-marginPass-readySamples=").Append(path.Entry.StrongSamples).Append('|').Append(path.MarginSamples).Append('|').Append(path.Entry.ReadySamples)
                .Append("/strongStart=").Append(path.Entry.StrongStart).Append("/trialDepthLatch=").Append(path.Entry.DepthLatch).Append("/worstMarginMm=").Append(path.Entry.StrongSamples > 0 ? Mm(path.LeastMargin) : "NA")
                .Append("/retainedBufferedShoe-preAlso-lowerAlso-bothAlso=");
            if (scoredFirst) b.Append(shoe).Append('|').Append(pre).Append('|').Append(lower).Append('|').Append(both);
            else b.Append("Unavailable");
            b.Append("/originHipY=").Append(Exact(_hipY)).Append("/axis=").Append(V(_travel))
                .Append("/shoeLossOrder=ankleXZ|soleXZ|absAnkleAlong|absSoleAlong/model=CONDITIONED_ON_NATIVE_GAIT_AND_COMMANDS/horizon=HELD_AIM_BUDGET_AND_RATES/plantWrites=0");
            int paired = 0, predictedBuffered = 0, measuredGeometry = 0, measuredTracking = 0, measuredBoth = 0;
            int feasible = 0;
            var allErrors = new AnkleGapErrors();
            var bufferedErrors = new AnkleGapErrors();
            for (int rowIndex = 0; rowIndex < _rows.Count; rowIndex++)
            {
                var r = _rows[rowIndex];
                var step = r.Combined;
                var prior = rowIndex > 0 ? _rows[rowIndex - 1].Combined : null;
                var next = rowIndex + 1 < _rows.Count ? _rows[rowIndex + 1] : null;
                b.Append("\nTURN_COMBINED_INPUT_V041780/start-issue-read=").Append(Start).Append('|').Append(r.From).Append('|').Append(r.To)
                    .Append("/dt=").Append(Exact(r.Dt)).Append("/source-next=").Append(r.Source).Append('|').Append(r.NextSource)
                    .Append("/parent-legOptions-predicted-kept-safeReads=").Append(step.Parents).Append('|').Append(step.LegOptions).Append('|')
                    .Append(step.Forecasts).Append('|').Append(step.Count).Append('|').Append(step.SafeReads)
                    .Append("/horizonReady=").Append(step.HaveHorizon)
                    .Append("/extraLegs-jointCandidates-shortlist-nextFootCommands=").Append(step.ExtraLegs).Append('|').Append(step.JointChecked)
                    .Append('|').Append(step.Shortlisted).Append('|').Append(step.NextCommands)
                    .Append("/startFootMinusLowerM=").Append(V(step.StartGap));
                if (braked?.HasInputs(r.From) == true) b.Append("/legInputs=LeftBraked");
                else AppendForecastInputs(b, r);
                var cmd = step.Command;
                var tether = cmd.Tether;
                b.Append("/footCommand=").Append(cmd.Valid).Append("/rawA-appliedA=").Append(V(tether.Raw)).Append('|').Append(V(cmd.Applied))
                    .Append("/baseCap-selectedCap-Kp-Kd-drag-hardM=").Append(Exact(tether.BaseCap)).Append('|')
                    .Append(Exact(tether.SelectedCap)).Append('|').Append(Exact(tether.Kp))
                    .Append('|').Append(Exact(tether.Kd)).Append('|').Append(Exact(cmd.Drag))
                    .Append('|').Append(Exact(tether.HardGap));
                if (r.Read)
                {
                    b.Append("/baselineAgrees=").Append(r.BaselineAgrees).Append("/maxNominalErrorMm=").Append(Mm(r.Residual))
                        .Append("/endpointAimM=").Append(V(r.MeasuredAim.Target)).Append('|').Append(V(r.MeasuredAim.TargetSole))
                        .Append("/entryLocal-floor-confirmed-ground-held-inside=").Append(r.Entry.Local).Append('|').Append(r.Entry.Floor).Append('|')
                        .Append(r.Entry.Confirmed).Append('|').Append(r.Entry.LegacyReady).Append('|').Append(r.Entry.IdentityHeld).Append('|').Append(r.Entry.NativeInside)
                        .Append("/nativeEntrySoleMm=").Append(r.Entry.Floor == 0 ? "NA" : Mm(r.Entry.NativeSole));
                    if (step.HasPhysicalPose) AppendReadPose(b, step.PhysicalPose);
                }
                for (int i = 0; i < step.Count; i++)
                {
                    ref var p = ref step.Paths[i];
                    var a = p.Leg.Offset;
                    b.Append("\nJ178/issue-path-parent=").Append(r.From).Append('|').Append(p.Id).Append('|').Append(p.Parent)
                        .Append("/protected=").Append(p.Protected).Append("/hip-kneeRad-rate=").Append(Exact(a.Hip)).Append('|').Append(Exact(a.Knee))
                        .Append('|').Append(Exact(a.HipRate)).Append('|').Append(Exact(a.KneeRate))
                        .Append("/footOffsetM=").Append(V(p.Move.Body.Position)).Append("/footDeltaVMps=").Append(V(p.Move.Body.Velocity))
                        .Append("/forecastSole-gapMm=").Append(Mm(p.Move.Forecast.Sole)).Append('|').Append(Mm(p.Move.Forecast.Gap))
                        .Append("/safeRead=").Append(p.SafeRead).Append("/readGate=").Append(p.Gate);
                    if (p.Move.Status == LandingFootTraction.State.Read)
                        b.Append("/readSole-gapMm=").Append(Mm(p.Move.Settled.Sole)).Append('|').Append(Mm(p.Move.Settled.Gap));
                    b.Append("/lowerIssueLossXZ-alongMm=").Append(Mm(p.IssueLower.XZ)).Append('|').Append(Mm(p.IssueLower.Along));
                    if (p.HasHorizontalRead)
                    {
                        b.Append("/preShoeLossMm="); AppendShoeLoss(b, p.PreTraction);
                        b.Append("/postShoeLossMm="); AppendShoeLoss(b, p.PostTraction);
                        b.Append("/lowerReadLossXZ-alongMm=").Append(Mm(p.ReadLower.XZ)).Append('|').Append(Mm(p.ReadLower.Along));
                    }
                    b.Append("/firstPreExcess-firstLowerExcess-peakLowerLossMm=").Append(p.FirstPreExcess).Append('|')
                        .Append(p.FirstLowerExcess).Append('|').Append(Mm(p.PeakLowerLoss));
                    b.Append("/previewSlot=").Append(p.PreviewSlot).Append("/planarAdjustment=").Append(p.Move.PlanarAdjusted);
                    var h = p.Horizon;
                    b.Append("/nextForecast=").Append(!h.Evaluated ? "Unexamined" : !h.Feasible ? "NoCommand" : h.Buffered ? "Buffered" : "Safe");
                    if (h.Feasible)
                    {
                        feasible++;
                        b.Append("/nextSole-gap-lowerLossMm=").Append(Mm(h.Sole)).Append('|').Append(Mm(h.Gap)).Append('|').Append(Mm(h.Lower))
                            .Append("/nextHip-kneeRad=").Append(Exact(h.NextOffset.Hip)).Append('|').Append(Exact(h.NextOffset.Knee))
                            .Append("/nextShiftM=").Append(V(h.NextShift));
                        if (next != null && CombinedLandingSearch.TryReadWitness(p,step,next.Combined,next.MeasuredAim,next.Actual,out var measured))
                        {
                            paired++;
                            if (h.Buffered)
                            {
                                predictedBuffered++;
                                if (measured.Geometry) measuredGeometry++;
                                if (measured.Tracking) measuredTracking++;
                                if (measured.Geometry && measured.Tracking) measuredBoth++;
                            }
                            b.Append("/nextReadSole-gap-lowerLossMm=").Append(Mm(measured.Sole)).Append('|').Append(Mm(measured.Gap)).Append('|').Append(Mm(measured.Lower))
                                .Append("/nextReadGeometry-tracking=").Append(measured.Geometry).Append('|').Append(measured.Tracking);
                            if (prior != null && prior.HasPhysicalPose && prior.VerifiedReadClock.Matches(step.IssueClock))
                            {
                                var held = HoldAnkleGap(prior.PhysicalPose, r.Nominal, h);
                                allErrors.Add(h, held, measured);
                                if (h.Buffered) bufferedErrors.Add(h, held, measured);
                                b.Append("/nextHeldSole-gapMm=").Append(Mm(held.Sole)).Append('|').Append(Mm(held.Gap));
                            }
                            else b.Append("/nextHeldSole-gapMm=Unavailable");
                        }
                        else b.Append("/nextRead=Unavailable");
                    }
                    if (p.SafeRead)
                        b.Append("/strongSteps-marginPass-latch=").Append(p.Entry.StrongSamples).Append('|').Append(p.MarginSamples).Append('|').Append(p.Entry.DepthLatch);
                }
            }
            b.Append("\nTURN_LANDING_PREVIEW_V041780/start=").Append(Start)
                .Append("/paired-predictedBuffered-actualGeometry-actualTracking-actualBoth=").Append(paired).Append('|')
                .Append(predictedBuffered).Append('|').Append(measuredGeometry).Append('|').Append(measuredTracking).Append('|').Append(measuredBoth);
            b.Append("\nTURN_ANKLE_GAP_V041780/start=").Append(Start)
                .Append("/feasible-paired-unpaired-bufferedPairs=").Append(feasible).Append('|').Append(allErrors.Count)
                .Append('|').Append(feasible - allErrors.Count).Append('|').Append(bufferedErrors.Count)
                .Append("/maeOrder=linearSole|linearGap|heldSole|heldGap/allMaeMm=");
            allErrors.AppendMeans(b);
            b.Append("/bufferedMaeMm=");
            bufferedErrors.AppendMeans(b);
            b.Append("/allBoundsDisagreementsLinear-held=").Append(allErrors.LinearDisagreements).Append('|').Append(allErrors.HeldDisagreements)
                .Append("/bufferedBoundsDisagreementsLinear-held=").Append(bufferedErrors.LinearDisagreements).Append('|').Append(bufferedErrors.HeldDisagreements);
        }

        // Loss relative to the same native point and target in metres; output in millimetres.
        private static void AppendShoeLoss(StringBuilder b, in CombinedLandingSearch.ShoeLoss d) =>
            b.Append(Mm(d.AnkleXZ)).Append('|').Append(Mm(d.SoleXZ)).Append('|').Append(Mm(d.AnkleAlong)).Append('|').Append(Mm(d.SoleAlong));

        private static void AppendReadPose(StringBuilder b, in TurnHandoffTrajectory.Pose p)
        {
            b.Append("/readBonesM=").Append(V(p.Hip)).Append('|').Append(V(p.UpperKnee)).Append('|').Append(V(p.LowerKnee)).Append('|').Append(V(p.Lower)).Append('|').Append(V(p.Foot))
                .Append("/readAxis=").Append(V(p.Axis)).Append("/readFootR=").Append(V(new Vector3(p.Rotation.x,p.Rotation.y,p.Rotation.z)))
                .Append(',').Append(Exact(p.Rotation.w));
        }
    }
}
