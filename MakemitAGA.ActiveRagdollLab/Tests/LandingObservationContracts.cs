using System;
using System.Linq;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;

namespace ARL.Contracts
{
    internal static class LandingObservationContracts
    {
        internal static void Run()
        {
            Check.Run("v143 multiple windows and same-step receipts retain distinct event denominators", Ordinary);
            Check.Run("v143 missed hold, downstream recovery and windows starting after miss", MissRecovery);
            Check.Run("v143 owner reset, stop, changed support, clock loss and new swing", Censoring);
            Check.Run("v143 unique missed stances retain their own outcome even without a prior window", MissDenominators);
            Check.Run("v143 half-second match horizon is inclusive and held sources are not refreshes", Horizon);
            Check.Run("v143 limited detail output cannot drop windows or actual event totals", LongRun);
            Check.Run("v143 compact active samples preserve next edge and complete common evidence", ActiveParity);
            Check.Run("v143 compact parent-only input preserves valid heights and refuses stale evidence", Parent);
            Check.Run("v143 measured source intervals, nested callback costs and run boundaries", Timing);
            Check.Run("v143 deferred entry text has exact snapshot values and no per-publish formatting", DeferredReport);
        }

        private static Audit.Sample Sample(int seq, bool pass = true)
        {
            var s = LandingOpportunityContracts.S(seq);
            if (!pass) s.Min = -.001f;
            return s;
        }
        private static string Row(string report, string prefix) => report.Split('\n').Single(s => s.StartsWith(prefix, StringComparison.Ordinal));
        private static void Ordinary()
        {
            var a = new Audit();
            a.Observe(Sample(1)); a.Observe(Sample(2, false)); a.Observe(Sample(3));
            var s = Sample(4, false); s.OrdinaryEnabled = s.NowActive = true; a.Observe(s);
            s = Sample(5); s.OrdinaryEnabled = s.NowActive = true; a.Observe(s);
            s = Sample(6, false); s.OrdinaryEnabled = s.NowActive = true; a.Observe(s);
            a.Seal(); var t = a.Summary();
            Check.That(t.Contains("/ordinary=3|1|1|1"), "multiple windows still match only one ordinary event");
            a.Reset(); s = Sample(1); s.NowActive = true; a.Observe(s); a.Seal();
            Check.That(a.Summary().Contains("/ordinary=0|0|0|0"), "an active flag alone cannot counterfeit a committed ordinary enable");
        }

        private static void MissRecovery()
        {
            var a = new Audit(); a.Observe(Sample(1)); a.Observe(Sample(2, false));
            var s = Sample(3); s.FirstMiss = s.MissInProgress = true; a.Observe(s);
            for (int i = 4; i <= 7; i++)
            {
                s = Sample(i, i == 4 || i == 6);
                s.Step.SourceSequence = s.Source.Sequence = 2;
                s.Source.Time = .05f;
                s.MissInProgress = true;
                s.RecoveryEnabled = s.NowActive = i == 7;
                a.Observe(s);
            }
            a.Seal(); string t = a.Summary();
            Check.That(t.Contains("/firstMiss=1|1|0|0/shallowRecovery=1|1|0|0"), "one miss and one recovery despite three linked windows");
            Check.That(t.Contains("/Recovered=1|1|0|0"), "later candidates cannot move the first-miss receipt");
            a.Reset(); s = Sample(1); s.MissInProgress = s.FirstMiss = s.RecoveryEnabled = s.NowActive = true; a.Observe(s); a.Seal();
            Check.That(a.Summary().Contains("/firstMiss=1|0|1|0/shallowRecovery=1|0|1|0"), "simultaneous miss and recovery are both real events");
            a.Reset(); s = Sample(1); s.MissInProgress = true; a.Observe(s);
            s = Sample(2, false); s.NowActive = s.RecoveryEnabled = true; a.Observe(s); a.Seal();
            Check.That(a.Summary().Contains("recoveryWithoutObservedOrigin=1"),
                "left-censored miss never invents an origin");
        }

        private static void Censoring()
        {
            var a = new Audit(); a.Observe(Sample(1));
            var s = Sample(2, false); s.MissInProgress = s.FirstMiss = true; a.Observe(s);
            s = Sample(3, false); a.Observe(s); a.Seal();
            Check.That(a.Summary().Contains("/HoldEnded=1|1|0|0"), "an actual hold reset is observed");
            a.Reset(); a.Observe(Sample(1)); s = Sample(2, false); s.FirstMiss = s.MissInProgress = true; a.Observe(s); a.Seal();
            Check.That(a.Summary().Contains("/TraceEnded=1|1|0|0"), "stop before recovery is right-censored");
            for (int fault = 0; fault < 5; fault++)
            {
                a.Reset(); a.Observe(Sample(1)); s = Sample(2, false);
                if (fault == 0) s.FloorCenter.y += .001f;
                if (fault == 1) s.SupportId++;
                if (fault == 2) s = Sample(3, false);
                if (fault == 3) s.Source.Time += .0001f;
                if (fault == 4) a.Fault();
                s.NowActive = s.OrdinaryEnabled = true; a.Observe(s); a.Seal();
                Check.That(a.Summary().Contains("/ordinary=1|0|0|1"),
                    "a subsequent event cannot borrow a broken observation");
            }
            a.Reset(); s = Sample(1); s.Gait = Audit.Phase.Approach; a.Observe(s);
            s = Sample(2, false); s.Gait = Audit.Phase.Swing; a.Observe(s);
            s = Sample(3, false); s.NowActive = s.OrdinaryEnabled = true; a.Observe(s); a.Seal();
            Check.That(a.Summary().Contains("/ordinary=1|0|0|1"), "a different swing cannot complete a prior window");
        }

        private static void MissDenominators()
        {
            var a = new Audit();
            var s = Sample(1, false); s.FirstMiss = s.MissInProgress = true; a.Observe(s);
            s = Sample(2); s.MissInProgress = true; a.Observe(s);
            s = Sample(3, false); s.MissInProgress = s.NowActive = s.RecoveryEnabled = true; a.Observe(s);
            s = Sample(4); s.FirstMiss = s.MissInProgress = true; a.Observe(s);
            a.Observe(Sample(5, false));
            a.Observe(Sample(6)); a.Observe(Sample(7, false)); a.Observe(Sample(8));
            s = Sample(9, false); s.FirstMiss = s.MissInProgress = true; a.Observe(s);
            s = Sample(10, false); s.FloorCenter.y += .01f; s.MissInProgress = true; a.Observe(s);
            s = Sample(11, false); s.FirstMiss = s.MissInProgress = true; a.Observe(s);
            a.Seal();
            string row = Row(a.Summary(), "LANDING_EVENT_MISS_V041440/");
            Check.That(row.Contains("/Recovered=1|0|0|1") && row.Contains("/HoldEnded=1|0|1|0") &&
                row.Contains("/Discontinuity=1|1|0|0") && row.Contains("/TraceEnded=1|0|0|1"),
                "every real miss has exactly one terminal outcome and advance relation; post-miss windows cannot rewrite it");
            Check.That(a.Summary().Contains("/firstMiss=4|1|1|2"), "miss denominator counts events rather than windows");
        }

        private static void Horizon()
        {
            for (int end = 8; end <= 9; end++)
            {
                var a = new Audit();
                for (int i = 1; i <= end; i++)
                {
                    var s = Sample(i, i <= 3); s.Step.Dt = .1f;
                    if (i >= 3) { s.Source.Sequence = s.Step.SourceSequence = 7; s.Source.Time = .02f; }
                    if (i == end) s.NowActive = s.OrdinaryEnabled = true;
                    a.Observe(s);
                }
                a.Seal(); string t = a.Summary();
                Check.That(end == 8 ? t.Contains("/ordinary=1|1|0|0") : t.Contains("/ordinary=1|0|0|1"),
                    "500ms since last qualifying point, independent of rendered frames");
            }
        }

        private static void LongRun()
        {
            var a = new Audit();
            for (int i = 1; i <= 300; i++)
            {
                var s = Sample(i, i % 2 == 1);
                if (i % 2 == 0) s.NowActive = s.OrdinaryEnabled = true;
                a.Observe(s);
            }
            a.Seal(); string t = a.Summary();
            Check.That(t.Contains("/ordinary=150|150|0|0"), "event counts outlive any display quota");
        }

        private static void ActiveParity()
        {
            var full = new Audit(); var compact = new Audit();
            for (int i = 1; i <= 80; i++)
            {
                var s = Sample(i, i % 3 != 0); s.WasActive = i % 8 >= 3 && i % 8 <= 6;
                s.NowActive = i % 8 >= 2 && i % 8 <= 5; s.OrdinaryEnabled = !s.WasActive && s.NowActive;
                s.Strong = i % 4 != 0; s.Gait = i % 5 < 2 ? Audit.Phase.Stance : Audit.Phase.Swing;
                full.Observe(s);
                if (s.WasActive) compact.ObserveActive(s.Step, s.Source, s.Fixed, s.Gait, s.Strong, s.NowActive);
                else compact.Observe(s);
            }
            full.Seal(); compact.Seal();
            Check.That(full.Summary() == compact.Summary(), "entire shared result identical across active hold and release");
        }

        private static void Parent()
        {
            for (int mask = 0; mask < 8; mask++)
            {
                bool compact = (mask & 1) != 0, history = (mask & 2) != 0, adopted = (mask & 4) != 0;
                Check.That(LandingReferenceCapturePolicy.NeedsFull(compact, history, adopted) == (!compact || history || adopted),
                    "full history or adopted evidence survives compact mode");
            }
            var input = LandingEvidenceContracts.Input(); var full = LandingEvidenceContracts.Build(input);
            var partial = new LegLandingReferenceInput { Step = input.Step, Source = input.Source, Fixed = input.Fixed,
                RawHip = input.RawHip, Parent = input.Parent };
            Check.That(!partial.Matches(input.Step, input.Source, input.Fixed) && full.ParentPlacementValid, "partial is never a full-leg receipt");
            Check.That(LandingReferenceCapturePolicy.TryParentHeights(partial, input.Step, input.Source, input.Fixed, out float raw, out float r, out float a) &&
                raw == full.ParentRaw.y && r == full.ParentReference.y && a == full.ParentActual.y, "three independently rotated hip points match full decomposition");
            for (int n = 0; n < 6; n++)
            {
                var p = partial;
                if (n == 0) p.Fixed++;
                if (n == 1) p.Parent.Step.Sequence++;
                if (n == 2) p.Source.Time++;
                if (n == 3) p.Parent.ActualR = default;
                if (n == 4) p.RawHip.y = float.NaN;
                if (n == 5) p.Prepared = true;
                Check.That(!LandingReferenceCapturePolicy.TryParentHeights(p, input.Step, input.Source, input.Fixed, out _, out _, out _),
                    "foreign partial data or a failed full request cannot impersonate a valid projection");
            }
        }

        private static void Timing()
        {
            var t = new LandingObservationTiming();
            t.Begin(PelvisDynamicReceiverIsolation.DynamicHipArmA);
            var old = t.Measure(LandingObservationTiming.Stage.FixedLoop, true);
            t.Seal(); t.Begin(PelvisDynamicReceiverIsolation.DynamicHipArmB); old.Dispose();
            Check.That(t.Compact && t.Running, "new scope resets counters and mode");
            using (t.Measure(LandingObservationTiming.Stage.FixedLoop, true)) { }
            using (t.Measure(LandingObservationTiming.Stage.FixedLoop, false)) { }
            var s = new SourceStepContext { Epoch = 5, Sequence = 1, Dt = .01f, Time = 1f };
            t.Source(s, true); s.Sequence++; s.Time += .125f; t.Source(s, true);
            t.FixedBoundary(false); t.FixedBoundary(true); t.Source(s,true); // duplicate cannot restore both-throughout
            s.Sequence++; s.Time += .25f; t.Source(s, true);
            s.Sequence += 2; s.Time += 1f; t.Source(s, true);
            s.Sequence++; s.Time += .5f; t.Source(s, false);
            Check.That(t.Seal() && !t.Seal() && !t.Compact, "stop has no live diagnostic selector");
            using (t.Measure(LandingObservationTiming.Stage.FixedLoop)) { }
            var report = t.Summary(); var loop = Row(report, "LANDING_COST_STAGE_V041430/stage=FixedLoop/");
            Check.That(loop.Contains("maxMs=2|") && loop.Contains("bothActiveAtCallbackEntry=1|"), "partial previous run and after-stop spans are excluded");
            var intervals = Row(report, "LANDING_SOURCE_INTERVAL_V041430/");
            Check.That(intervals.Contains("sourceBreaks=1") && intervals.Contains("/all=3|0.875|") &&
                intervals.Contains("/bothActiveThroughout=1|0.125|8|125|125|1|1|1|0") &&
                intervals.Contains("/other=2|0.75|"), "an intervening release, skip and idle endpoints are distinguished");
            t.Begin("RETIRED"); Check.That(!t.Running && !t.Compact, "legacy and rejected scopes do not measure");
        }

        private static void DeferredReport()
        {
            var a = new FootAnkleFoundationReport(); var b = new FootAnkleFoundationReport();
            var snap = new FootContactEntrySnapshot
            {
                TracePolicy="SHALLOW", LivePolicy="LIVE", Ready=true, AtEnable=true, HistoryMotion=true, Fixed=81,
                Missed=5, Recovered=2, RecoveryReject=6, HistoryCount=2, GeometryReject=4, GeometryInvalid=3, Evaluated=8,
                HoldSeconds=.12f, Peak=.004f, EnabledPeak=.0005f, IdentityHold=.05f,
                Geometry=new FootContactEntrySnapshot.Box { Valid=true, FullFootprint=true, MotionValid=true, ReferencesValid=true,
                    MinClearance=-.003f, SourceMinClearance=.012f, ControlMinClearance=.009f, PredictedMinClearance=-.002f,
                    MaxCornerNormalSpeed=.03f, PlanarSpeed=.04f, AngularSpeed=.15f, AnchorGap=.003f, SourceAge=.015f, Issue="a\nb" }
            };
            Check.That(b.ContactEntrySummary == null, "never-published view is absent");
            var timing = LandingObservationTiming.Run;
            timing.Begin(PelvisDynamicReceiverIsolation.DynamicHipArmA);
            a.PublishEntrySummary(snap, false); b.PublishEntrySummary(snap, true);
            var expected = a.ContactEntrySummary;
            Check.That(expected.Contains("/geometrySampleFixed=81/geometrySampleRole=ENABLE_PRE_CONTROL") &&
                expected.Contains("/recovered-unrecovered=2-3/") && expected.Contains("/source-controlMinMm=12|9/") &&
                expected.Contains("/geometryIssue=a b/"), "logged scalar units, availability and sanitization");
            for (int i=0; i<200; i++) b.PublishEntrySummary(snap, true);
            long mark = GC.GetAllocatedBytesForCurrentThread();
            for (int i=0; i<1000; i++) b.PublishEntrySummary(snap, true);
            Check.That(GC.GetAllocatedBytesForCurrentThread() == mark, "repeated compact publication allocates zero managed bytes");
            snap.Fixed++; snap.Geometry.MinClearance = float.NaN;
            Check.That(b.ContactEntrySummary == expected && ReferenceEquals(b.ContactEntrySummary, b.ContactEntrySummary), "deferred view freezes values and caches first explicit read");
            b.PublishEntrySummary(snap, true); Check.That(b.ContactEntrySummary.Contains("/geometrySampleFixed=82/"), "publishing invalidates old text");
            timing.Seal();
            Check.That(Row(timing.Summary(), "LANDING_COST_STAGE_V041430/stage=EntryText/").Contains("maxMs=3|"), "one eager format plus two actual reads even after 1200 deferred publications");
        }
    }
}
