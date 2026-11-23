using System;
using System.Globalization;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralFootTransitionTrace
    {
        // Each curve uses a single material landmark, the ankle or the center of the sole's
        // bottom face. A rotating box can lead with its sole even if its ankle tracks well.
        private sealed class Curve
        {
            internal float Baseline, AtQuiet, Peak, PeakActual, PeakTarget, ReturnActual, ReturnTarget;
            internal int PeakAt, ReturnAt;
            internal bool BaselineKnown;
            internal void Update(int clock, float actual, float target, bool firstQuiet)
            {
                float lead = actual - target;
                if (firstQuiet) AtQuiet = lead;
                if (PeakAt == 0 || lead > Peak)
                {
                    Peak = lead; PeakAt = clock; PeakActual = actual; PeakTarget = target;
                    ReturnAt = 0; ReturnActual = ReturnTarget = 0f;
                }
                else if (ReturnAt == 0 && clock > PeakAt && Peak > .002f && lead <= (Ahead ? Math.Max(0f,Baseline) : 0f) + .001f && Peak-lead >= .001f)
                {
                    ReturnAt = clock;
                    ReturnActual = actual-PeakActual; ReturnTarget = target-PeakTarget;
                }
            }
            // The 2/1 mm display bands are diagnostic only. Retain the signed peak even below them.
            internal bool Ahead => BaselineKnown && PeakAt > 0 && Peak > .002f && Peak-Baseline > .002f;
            internal string Result => PeakAt == 0 ? "NoQuiet" : Peak <= .002f ? "NoForwardLead" : !BaselineKnown ? "UnknownBaseline" : Ahead ? "NewForwardLead" : "AlreadyAhead";
            internal string Closure => ReturnAt == 0 ? "Unobserved" :
                ReturnActual < -.001f ? (ReturnTarget > .001f ? "BothMoved" : "ProxyBack") :
                ReturnTarget > .001f ? "TargetCaughtUp" : "SmallClosure";
            internal string Text() => (BaselineKnown ? F(Baseline*1000f) : "NA") + "|" + (PeakAt == 0 ? "NA|NA|0|0|NA|NA" :
                F(AtQuiet*1000f)+"|"+F(Peak*1000f)+"|"+PeakAt+"|"+ReturnAt+"|"+
                (ReturnAt == 0 ? "NA|NA" : F(ReturnActual*1000f)+"|"+F(ReturnTarget*1000f)));
        }

        private sealed partial class Window
        {
            internal readonly int Side, TriggerSource;
            internal readonly Kind Type;
            internal readonly string Trigger;
            internal readonly View Origin, Before;
            internal readonly FootTransitionSource.Heading Heading;
            internal View Last, Next, Swing, Quiet, PeakAnkle, PeakSole, First, Resolved, HipBelow;
            internal string EndReason, TorsoEdge, Landing = "NoNewEntry";
            internal int FootBrakeSource, Steps, SlidIntervals, FormalIntervals, OtherFormalSteps, OtherPreSteps, OtherFormalBeforeFirst, OtherPreBeforeFirst;
            internal float NearPath, FormalPath, AlongFloor, CallbackFormalPath, CallbackPrePath;
            internal int CallbackFormalIntervals, CallbackPreIntervals;
            internal int QuietStreak, LongestQuiet;
            private int _lastQuietSource;
            internal bool LandingFinal, Slowed;
            internal readonly Curve Ankle = new Curve(), Sole = new Curve();
            internal Window(int side, Kind kind, in View point, in View prev, FootTransitionSource.Heading heading,
                string trigger, int source, FootTransitionSource renders)
            {
                Side = side; Type = kind; Origin = point; Before = prev; Trigger = trigger; TriggerSource = source;
                Heading = heading.Recent(point.Foot.Source.Clock.Time) ? heading : default;
                if (side == 0 && kind == Kind.PairRelease) NativeLandingPath = new LeftLandingPath();
                var reference = prev.Foot.Local != 0 ? prev.Foot : point.Foot;
                Ankle.BaselineKnown = Sole.BaselineKnown = prev.Foot.Local != 0;
                Ankle.Baseline = Along(reference.Foot-reference.Target);
                Sole.Baseline = Along(reference.Sole-reference.SourceSole);
                _nextRender = source;
                Observe(point,renders);
            }
            internal float Along(Vector3 delta) => Vector3.Dot(delta,Heading.Axis);
            internal void Observe(in View point, FootTransitionSource source)
            {
                var t = point.Foot;
                Steps++;
                if (point.OtherActive) OtherFormalSteps++;
                if (point.OtherPrecontact) OtherPreSteps++;
                if (Steps == 2) Next = point;
                if (Swing.Foot.Local == 0 && t.Phase == Phase.Swing) Swing = point;
                if (First.Foot.Local == 0)
                {
                    if (point.OtherActive) OtherFormalBeforeFirst++;
                    if (point.OtherPrecontact) OtherPreBeforeFirst++;
                    if (HipBelow.Foot.Local == 0 && Last.Foot.HipValid && Last.Foot.HipHeight >= 0f && t.HipValid && t.HipHeight < 0f)
                        HipBelow = point;
                }
                if (First.Foot.Local == 0)
                {
                    NativeLandingPath?.Observe(t);
                    if (Braked != null && FirstEligibleLocal == 0 && t.Floor != 0 && t.Entry.Local == t.Local && t.Entry.Floor == t.Floor)
                        FirstEligibleLocal = t.Local;
                }
                ObserveLanding(point);
                if (Type == Kind.Brake)
                {
                    ReadRenders(source,t.Source,t.Source.Clock.Sequence);
                    float speed = FootTransitionSource.Planar(t.Source.World).magnitude;
                    Slowed |= speed <= FootGaitEvidence.StanceTangent;
                    bool firstQuiet = Quiet.Foot.Local == 0 && speed <= FootContactGeometryLimits.RecoveryMaxPlanarSpeed;
                    if (firstQuiet) Quiet = point;
                    if (speed <= FootContactGeometryLimits.RecoveryMaxPlanarSpeed)
                    {
                        if (t.Source.Clock.Sequence != _lastQuietSource)
                            QuietStreak = t.Source.Clock.Sequence == _lastQuietSource+1 ? QuietStreak+1 : 1;
                        _lastQuietSource = t.Source.Clock.Sequence;
                        LongestQuiet = Math.Max(LongestQuiet,QuietStreak);
                    }
                    else QuietStreak = 0;
                    if (Quiet.Foot.Local != 0)
                    {
                        // Anchor positions at the origin before taking dot products to preserve mm precision.
                        Ankle.Update(t.Local, Along(t.Foot-Origin.Foot.Target), Along(t.Target-Origin.Foot.Target), firstQuiet);
                        Sole.Update(t.Local, Along(t.Sole-Origin.Foot.SourceSole), Along(t.SourceSole-Origin.Foot.SourceSole), firstQuiet);
                        if (Ankle.PeakAt == t.Local) PeakAnkle = point;
                        if (Sole.PeakAt == t.Local) PeakSole = point;
                    }
                }
                var old = Last.Foot.Local != 0 ? Last : Before;
                if (old.Foot.Local != 0 && old.Foot.NearFloor && t.NearFloor)
                {
                    Vector3 delta = FootTransitionSource.Planar(t.Sole-old.Foot.Sole);
                    float d = delta.magnitude;
                    NearPath += d; AlongFloor += Along(delta); SlidIntervals++;
                    // Contact selected at PRE_N applies to the interval ending at PRE_N+1.
                    if (old.Foot.Active)
                    {
                        FormalPath += d; FormalIntervals++;
                        if (t.PreviousHit) { CallbackFormalPath += d; CallbackFormalIntervals++; }
                    }
                    else if (old.Foot.Precontact && t.PreviousHit) { CallbackPrePath += d; CallbackPreIntervals++; }
                }
                Last = point;
            }
            private void ObserveLanding(in View point)
            {
                if (LandingFinal) return;
                var t = point.Foot;
                if (First.Foot.Local != 0)
                {
                    if (t.Recovery && t.Active) Landing = "Recovered";
                    else if (!t.PendingMiss || t.Miss) Landing = "HoldEnded";
                    else return;
                }
                else
                {
                    if (t.Miss && !t.Active) { First = point; Landing = "FirstMissPending"; return; }
                    if (t.Ordinary && t.Active) Landing = "Ordinary";
                    else if (t.Recovery && t.Active) Landing = "RecoveryOfEarlierMiss";
                    else return;
                    First = point;
                }
                LandingFinal = true; Resolved = point;
            }
        }

        private static string F(float x) => x.ToString("0.#####", CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => F(v.x)+","+F(v.y)+","+F(v.z);
        private static string Bit(bool b) => b ? "1" : "0";
        internal string Summary()
        {
            var b = new StringBuilder("\nFOOT_TRANSITION_V041680/pairs-cuts-rejected-missing-faults-sealedPartial=")
                .Append(_pairs).Append('|').Append(_cuts).Append('|').Append(_rejected).Append('|').Append(_missingFoot)
                .Append('|').Append(_faults).Append('|').Append(_sealedPartial)
                .Append("/left-right-unseenRenderBrakes=").Append(_sources[0].UnseenBrakes).Append('|').Append(_sources[1].UnseenBrakes)
                .Append("/unmatchedCallbacks=").Append(_unmatchedCallbacks)
                .Append("/capacityPerSideKind=").Append(MaxPerSideKind);
            for (int side=0; side<2; side++)
            {
                b.Append("\nFOOT_TRANSITION_COUNTS_V041680/side=").Append(side==0 ? "LEFT" : "RIGHT")
                    .Append("/seen-skipped(pair,single,brake)=");
                for (int k=0;k<3;k++) b.Append(k==0 ? "" : ",").Append(_found[side,k]).Append('|').Append(_skipped[side,k]);
                b.Append("/pairedLift-worldTanOnly-relNoSwing-relStance-nearQuiet=");
                for (int k=0;k<5;k++) b.Append(k==0 ? "" : "|").Append(_lift[side,k]);
            }
            AppendHandoffCounts(b);
            foreach (var w in _windows) AppendWindow(b,w);
            return b.ToString();
        }

        private static void AppendWindow(StringBuilder b, Window w)
        {
            b.Append("\nFOOT_TRANSITION_EVENT_V041680/side=").Append(w.Side==0 ? "LEFT" : "RIGHT")
                .Append("/kind=").Append(w.Type).Append("/epoch-shared-local-floor-body=")
                .Append(w.Origin.Foot.Step.Epoch).Append('|').Append(w.Origin.Foot.Step.Sequence).Append('|')
                .Append(w.Origin.Foot.Local).Append('|').Append(w.Origin.Foot.Floor).Append('|').Append(w.Origin.Foot.Body)
                .Append("/trigger-source-footBrakeSource-torsoEdge=").Append(w.Trigger).Append('|').Append(w.TriggerSource)
                .Append('|').Append(w.FootBrakeSource).Append('|').Append(w.TorsoEdge ?? "NA")
                .Append("/axis-kind-xyz=").Append(w.Heading.Axis.sqrMagnitude < .9f ? "NA" : w.Heading.FromRoot ? "RootTravel" : "FootTravel")
                .Append('|').Append(V(w.Heading.Axis)).Append("/first-outcome-resolved=").Append(w.First.Foot.Local)
                .Append('|').Append(w.Landing).Append('|').Append(w.Resolved.Foot.Local)
                .Append("/end-local=").Append(w.EndReason ?? "Open").Append('|').Append(w.Last.Foot.Local)
                .Append("/steps-otherFormal-otherPre=").Append(w.Steps).Append('|').Append(w.OtherFormalSteps).Append('|').Append(w.OtherPreSteps)
                .Append("/otherFormal-otherPreThroughFirst=").Append(w.OtherFormalBeforeFirst).Append('|').Append(w.OtherPreBeforeFirst)
                .Append("/nearIntervals-pathMm-netAlongMm-formalIntervals-formalPathMm=").Append(w.SlidIntervals).Append('|')
                .Append(F(w.NearPath*1000f)).Append('|').Append(w.Heading.Axis.sqrMagnitude < .9f ? "NA" : F(w.AlongFloor*1000f))
                .Append('|').Append(w.FormalIntervals).Append('|').Append(F(w.FormalPath*1000f))
                .Append("/callbackFormalIntervals-pathMm-preIntervals-pathMm=").Append(w.CallbackFormalIntervals)
                .Append('|').Append(F(w.CallbackFormalPath*1000f)).Append('|').Append(w.CallbackPreIntervals).Append('|').Append(F(w.CallbackPrePath*1000f));
            if (w.Type == Kind.Brake)
                b.Append("/quiet=").Append(w.Quiet.Foot.Local).Append("/ankle-base-quiet-peakMm-peakAt-returnAt-proxyDelta-targetDeltaMm=")
                    .Append(w.Ankle.Text()).Append("/sole-same=").Append(w.Sole.Text())
                    .Append("/ankleLead-closure-soleLead-closure=").Append(w.Ankle.Result).Append('|').Append(w.Ankle.Closure)
                    .Append('|').Append(w.Sole.Result).Append('|').Append(w.Sole.Closure).Append("/maxQuietSources=").Append(w.LongestQuiet);
            if (w.Type == Kind.Brake)
                b.Append("/renderSamples-quietSource-end=").Append(w.RenderCount).Append('|').Append(w.RenderQuiet)
                    .Append('|').Append(w.RenderEnd ?? "WindowEnd")
                    .Append("/renderAnkle-base-quiet-peakMm-sourceAt-returnSource-proxyDelta-targetDeltaMm=").Append(w.RenderAnkle.Text())
                    .Append("/renderSole-same=").Append(w.RenderSole.Text())
                    .Append("/renderAnkleLead-closure-soleLead-closure=").Append(w.RenderAnkle.Result).Append('|').Append(w.RenderAnkle.Closure)
                    .Append('|').Append(w.RenderSole.Result).Append('|').Append(w.RenderSole.Closure);
            b.Append("/locals(before-origin-next-swing-hipBelow-quiet-anklePeak-solePeak-first-end)=")
                .Append(w.Before.Foot.Local).Append('|').Append(w.Origin.Foot.Local).Append('|').Append(w.Next.Foot.Local)
                .Append('|').Append(w.Swing.Foot.Local).Append('|').Append(w.HipBelow.Foot.Local).Append('|').Append(w.Quiet.Foot.Local)
                .Append('|').Append(w.PeakAnkle.Foot.Local).Append('|').Append(w.PeakSole.Foot.Local).Append('|').Append(w.First.Foot.Local)
                .Append('|').Append(w.Last.Foot.Local);
            var written = new HashSet<int>();
            AppendView(b,"before",w.Before,written); AppendView(b,"origin",w.Origin,written);
            AppendView(b,"next",w.Next,written); AppendView(b,"swing",w.Swing,written);
            AppendView(b,"quiet",w.Quiet,written); AppendView(b,"anklePeak",w.PeakAnkle,written);
            if (w.PeakSole.Foot.Local != w.PeakAnkle.Foot.Local) AppendView(b,"solePeak",w.PeakSole,written);
            AppendView(b,"hipCrossBelow",w.HipBelow,written); AppendView(b,"first",w.First,written);
            AppendView(b,"end",w.Last,written);
            if (w.RightControlEligible)
                b.Append("\nTURN_HANDOFF_CONTROL_V041690/quiet-precontactSolves-start=").Append(w.Quiet.Foot.Local).Append('|')
                    .Append(w.WaitingPrecontact).Append('|').Append(w.Handoff?.Start ?? 0).Append("/result=")
                    .Append(w.Handoff!=null ? "Selected" : w.ControlExit ?? w.EndReason ?? "Open");
            w.Handoff?.Append(b,w.Side,w.Origin.Foot.Local,w.Landing,w.First.Foot.Local,w.Resolved.Foot.Local,
                w.Type==Kind.Brake ? w.Ankle.Result : null,w.Ankle.PeakAt,w.Ankle.Peak);
            if (w.Early != null)
            {
                w.Early.Append(b, w.Side, w.Origin.Foot.Local, w.Landing, w.First.Foot.Local, w.Resolved.Foot.Local,
                    null, 0, 0f, w.Handoff);
                w.Braked.Append(b, w.Side, w.Origin.Foot.Local, w.Landing, w.First.Foot.Local, w.Resolved.Foot.Local,
                    null, 0, 0f, w.Handoff, w.Early);
                AppendLeftLanding(b, w);
                w.Braked.AppendTraction(b, FirstOpportunity(w), w.Early);
                w.Braked.AppendContinuous(b, w.First.Foot, w.Landing, w.Resolved.Foot.Local, w.FirstEligibleLocal);
                w.Combined.AppendCombined(b, w.First.Foot, w.Landing, w.Resolved.Foot.Local, w.FirstEligibleLocal, w.Braked);
                w.NativeLandingPath.AppendHeightBudget(b, w.Combined, w.FirstEligibleLocal);
            }
            w.NativeLandingPath?.AppendRise(b, w.Origin.Foot.Local, FirstOpportunity(w), w.First.Foot, w.Braked);
            w.NativeLandingPath?.AppendHeights(b, w.Origin.Foot.Local, FirstOpportunity(w), w.Braked != null);
        }
        private static void AppendView(StringBuilder b, string label, in View v, HashSet<int> written)
        {
            var t = v.Foot; if (t.Local == 0 || !written.Add(t.Local)) return;
            var g = t.Gait; var rel = g.WithVelocity(t.Source.Relative);
            b.Append("\nFOOT_TRANSITION_STEP_V041680/at=").Append(label).Append("/local-shared-source=")
                .Append(t.Local).Append('|').Append(t.Step.Sequence).Append('|').Append(t.Source.Clock.Sequence)
                .Append("/fresh-held-ageMs=").Append(Bit(t.Fresh)).Append('|').Append(t.Held).Append('|').Append(F(t.Age*1000f))
                .Append("/motion-gait=").Append(t.Motion ?? "NA").Append('|').Append(t.Phase)
                .Append("/was-formal-pre-otherFormal-otherPre-otherRelease-otherGait-otherLocal=")
                .Append(Bit(t.Was)).Append('|').Append(Bit(t.Active)).Append('|').Append(Bit(t.Precontact)).Append('|')
                .Append(Bit(v.OtherActive)).Append('|').Append(Bit(v.OtherPrecontact)).Append('|').Append(Bit(v.OtherReleased)).Append('|')
                .Append(v.OtherPhase).Append('|').Append(v.OtherLocal)
                .Append("/previousSolveHit-otherHit=").Append(Bit(t.PreviousHit)).Append('|').Append(Bit(v.OtherPreviousHit))
                .Append("/releaseReason=").Append(t.ReleaseReason ?? "NA")
                .Append("/hits-same-probeId-maxAbsMm-vt-vn-swingBits-stance=").Append(g.Hits).Append('|').Append(Bit(g.Same))
                .Append('|').Append(g.Surface).Append('|').Append(F(g.Clearance*1000f)).Append('|').Append(F(g.Tangent))
                .Append('|').Append(F(g.SignedNormal)).Append('|').Append((int)g.Swing).Append('|').Append(Bit(g.Stance))
                .Append("/classifierNormal=").Append(V(g.Normal))
                .Append("/world-rootTranslation-rootTurn-footRelativeMps=").Append(V(t.Source.World)).Append('|')
                .Append(V(t.Source.Translation)).Append('|').Append(V(t.Source.Turn)).Append('|').Append(V(t.Source.Relative))
                .Append("/relativeOnly-vt-vn-swingBits-stance=").Append(F(rel.Tangent)).Append('|').Append(F(rel.SignedNormal))
                .Append('|').Append((int)rel.Swing).Append('|').Append(Bit(rel.Stance))
                .Append("/inside-minMm-lowestPointPlanarMps=").Append(Bit(t.Inside)).Append('|').Append(F(t.BoxMin*1000f))
                .Append('|').Append(F(FootTransitionSource.Planar(t.CornerVelocity).magnitude))
                .Append("/footMinusSourceMm=").Append(V((t.Foot-t.Target)*1000f))
                .Append("/footMinusLowerMm=").Append(V((t.Foot-t.Lower)*1000f))
                .Append("/soleMinusSourceMm=").Append(V((t.Sole-t.SourceSole)*1000f))
                .Append("/lateUpdateAnkle-soleDeltaMm=").Append(t.Source.Visual.Valid ?
                    V((t.Source.Visual.Foot-t.Source.Visual.Target)*1000f)+"|"+V((t.Source.Visual.Sole-t.Source.Visual.TargetSole)*1000f) : "NA")
                .Append("/proxyAnkleMps=").Append(V(t.Velocity))
                .Append("/hipActualMinusReferenceMm=").Append(t.HipValid ? F(t.HipHeight*1000f) : "NA");
        }
    }
}
