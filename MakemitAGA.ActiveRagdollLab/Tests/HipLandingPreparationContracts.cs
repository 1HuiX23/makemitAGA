using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
namespace ARL.Contracts
{
    internal static class HipLandingPreparationContracts
    {
        internal static void Run()
        {
            Check.Run("v140 recorded same-state Hip positives and deep approach counterexamples", Recorded);
            Check.Run("v140 synthetic current improvement cannot bypass worsening aligned coast", CoastCounterexample);
            Check.Run("v140 protected, stale, malformed and foreign snapshots cannot prepare Hip", Reject);
            Check.Run("v140 independent corners, unchanged endpoint, W/A and brake debt", Synthetic);
            Check.Run("v140 original owner finalizes once and trace cannot change RWA", Owner);
            Check.Run("v140 preparation and bounded trace have zero hot-path allocation", Allocation);
            SingleLegShadowChain.RunHipLandingEntryContracts();
        }
        internal static HipLandingPreparation.Result Eval(KneeLandingWitnessFixtures.Fixture f) =>
            HipLandingPreparation.Evaluate(f.Packet, f.Hip, f.HipR, f.HipW, f.Knee);
        private static void Validate(KneeLandingWitnessFixtures.Fixture f, HipLandingPreparation.Result r)
        {
            if (!r.Better) return;
            var h = f.Hip; var k = f.Knee; var g = f.Packet.Geometry; var foot = f.Packet.Sample.PlannedFoot;
            var e = g.Upper * Quaternion.Inverse(f.HipR); var hc = FootRelativeReference.Advance(r.R, r.W, k.Step.Dt);
            var kc = FootRelativeReference.Advance(k.R, k.W, k.Step.Dt);
            Check.Near(r.Best.ReferenceNow, KneeLandingPreparationContracts.Oracle(g, r.R, k.R, foot), .000002f, "reference eight corners");
            Check.Near(r.Best.ReferenceCoast, KneeLandingPreparationContracts.Oracle(g, hc, kc, foot), .000002f, "reference coast corners");
            Check.Near(r.Best.AlignedNow, KneeLandingPreparationContracts.Oracle(g, e * r.R, k.R, foot), .000002f, "error-aligned eight corners");
            Check.Near(r.Best.AlignedCoast, KneeLandingPreparationContracts.Oracle(g, e * hc, kc, foot), .000002f, "same error transforms coast");
            Check.That(r.Best.ReferenceNow >= r.Baseline.ReferenceNow && r.Best.ReferenceCoast >= r.Baseline.ReferenceCoast &&
                r.Best.AlignedNow >= r.Baseline.AlignedNow && r.Best.AlignedCoast >= r.Baseline.AlignedCoast && r.Best.Risk - r.Baseline.Risk >= .0001f, "all endpoints nonregressing");
            Check.That(r.W.magnitude <= MathF.Max(h.MaxW, h.PreviousW.magnitude) + .00001f &&
                ((r.W - h.PreviousW) / k.Step.Dt).magnitude <= h.MaxA + .0002f, "original W/A bounds");
            Check.That(KneeLandingPreparationContracts.Angle(r.R, FootRelativeReference.Advance(h.PreviousR, r.W, k.Step.Dt)) < .0005f, "integrates once from previous R");
            var before = FootRelativeReference.Log(h.InputR * Quaternion.Inverse(f.HipR)); var after = FootRelativeReference.Log(h.InputR * Quaternion.Inverse(r.R));
            var direction = (r.W - f.HipW).normalized;
            float Debt(Vector3 w, Vector3 gap) => MathF.Max(0f, MathF.Max(0f, Vector3.Dot(w - h.InputW, direction)) *
                ((w - h.InputW).magnitude / (2f * h.MaxA) + k.Step.Dt) - Vector3.Dot(gap, direction));
            Check.That(after.magnitude <= before.magnitude + .000002f && Vector3.Dot(after, direction) >= -.000002f &&
                Debt(r.W, after) <= Debt(f.HipW, before) + .000003f, "unchanged goal, no overshoot or added stopping debt");
        }
        private static void Recorded()
        {
            int count = 0;
            foreach (var f in HipLandingWitnessFixtures.All())
            {
                var r = Eval(f); Validate(f, r);
                bool expected = f.Knee.Step.Sequence == 8997 || f.Knee.Step.Sequence == 7332;
                Check.That(r.Better == expected && !r.Applied, "recorded choice " + f.Name);
                if (r.Better) count++;
                Console.WriteLine("  v140 shared=" + f.Knee.Step.Sequence + " reason=" + r.Reason + " deltaDeg=" + r.DeltaDegrees + " riskGainMm=" + (r.Best.Risk - r.Baseline.Risk) * 1000f);
                if (f.Knee.Step.Sequence == 8997) Check.Near(r.DeltaDegrees, .16776086f, .0001f, "left Hip progress");
                if (f.Knee.Step.Sequence == 7332) Check.Near(r.DeltaDegrees, .18097225f, .0001f, "right Hip progress");
                if (f.Knee.Step.Sequence == 7489) Check.That(!r.Better && r.Baseline.Risk < -.04f, "deep approach is not falsely repaired");
            }
            Check.That(count == 2, "two distinct same-state Hip witnesses");
        }
        private static void CoastCounterexample()
        {
            // Synthetic orientation perturbation of the recorded left input. This
            // Upper pose is NOT a game observation: it isolates the coast guard.
            var f = HipLandingWitnessFixtures.All()[0];
            f.Packet.Geometry.Upper = new Quaternion(-.75971633f, .061194345f, .63577807f, -.1219541f);
            var r = Eval(f); Validate(f, r);
            Check.That(r.AlignedRejected > 0 && r.Rejected.AlignedNow >= r.Baseline.AlignedNow &&
                r.Rejected.AlignedCoast < r.Baseline.AlignedCoast, "a good aligned current endpoint can have a harmful coast endpoint");
        }
        private static void Reject()
        {
            var original = HipLandingWitnessFixtures.All()[0]; Check.That(Eval(original).Better, "positive guard control");
            for (int i = 0; i < 19; i++)
            {
                var f = original;
                if (i == 0) f.Packet.Published = false;
                if (i == 1) f.Packet.Input.Apply = false;
                if (i == 2) f.Packet.PreparationSupport.SupportId++;
                if (i == 3) f.Packet.Sample.Source.Sequence++;
                if (i == 4) f.Packet.PreparationSupport.SourceAge = .09f;
                if (i == 5) f.Packet.PreparationSupport.StrongStance = false;
                if (i == 6) f.Hip.Step.Sequence++;
                if (i == 7) f.Hip.Applied = false;
                if (i == 8) f.Hip.SampleAgePrediction = true;
                if (i == 9) f.Hip.ReceiverParentReference = false;
                if (i == 10) f.Hip.PredictionSeconds = .001f;
                if (i == 11) f.Hip.A.x += .1f;
                if (i == 12) f.HipR = Quaternion.AngleAxis(1f, Vector3.up) * f.HipR;
                if (i == 13) f.Knee.Stale = true;
                if (i == 14) f.Knee.A.x += .1f;
                if (i == 15) f.Packet.Geometry.Hip.y = float.NaN;
                if (i == 16) f.Packet.Geometry.FloorHalf.x = .001f;
                if (i == 17) f.HipW.y = float.NaN;
                if (i == 18) f.Packet.Sample.PlannedFoot = default;
                var r = Eval(f); Check.That(!r.Better && !r.Applied, "Hip guard " + i);
            }
        }
        private static void Synthetic()
        {
            var random = new Random(41400); int accepted = 0, guarded = 0;
            for (int i = 0; i < 2500; i++)
            {
                var x = LandingPreparationFixtures.Make(random); var f = new KneeLandingWitnessFixtures.Fixture {
                    Packet = x.Packet, Hip = x.Hip.Current, HipR = x.Hip.Rotation, HipW = x.Hip.W, Knee = x.Knee };
                // Include large native/reference mismatches to challenge the second basis.
                f.Packet.Geometry.Upper = Quaternion.AngleAxis((float)random.NextDouble() * 100f - 50f, Vector3.right) * f.HipR;
                var r = Eval(f); Validate(f, r); if (r.Better) accepted++; if (r.AlignedRejected > 0) guarded++;
                if (r.Better)
                {
                    var y = f; y.Packet.Geometry.Hip += new Vector3(.25f, 1f, -.5f); y.Packet.Geometry.FloorCenter += new Vector3(.25f, 1f, -.5f);
                    var moved = Eval(y); Check.That(moved.Better, "translated support plane preserves improvement");
                    Check.Near(moved.Best.Risk, r.Best.Risk, .000002f, "translation preserves clearance");
                }
            }
            Check.That(accepted > 50 && guarded > 10, "positive and error-aligned rejection coverage");
        }
        internal static HipNormalReference MakeOwner(KneeLandingWitnessFixtures.Fixture f, bool trace)
        {
            var h = new HipNormalReference(); h.Reset(true, f.Hip.PreviousR, f.Hip.PreviousW, false);
            Check.That(h.Step(trace, f.Knee.Step, f.Knee.Source, true, f.Hip.InputR, f.Hip.InputW, Quaternion.identity, Vector3.zero,
                f.Knee.Step.UnscaledTime - f.Knee.Source.Time, .1f, f.Hip.MaxW, f.Hip.MaxA, 55f / 12f, true), "real original owner step");
            return h;
        }
        private static void Owner()
        {
            foreach (var f in HipLandingWitnessFixtures.All())
            {
                var a = MakeOwner(f, false); var b = MakeOwner(f, true); var oldR = a.Rotation; var oldW = a.W;
                var expected = HipLandingPreparation.Evaluate(f.Packet, a.Current, oldR, oldW, f.Knee);
                Check.That(a.PrepareLanding(true, false, f.Packet, f.Knee) && b.PrepareLanding(true, true, f.Packet, f.Knee), "finalize owner");
                Check.That(a.Rotation.Equals(b.Rotation) && a.W.Equals(b.W) && a.Current.A.Equals(b.Current.A), "trace independence");
                Check.That(a.LandingPreparation.Applied == expected.Better, "owner only applies qualified candidate");
                if (!expected.Better) Check.That(a.Rotation.Equals(oldR) && a.W.Equals(oldW), "exact baseline fallback");
                else Check.That(a.Rotation.Equals(expected.R) && a.W.Equals(expected.W), "selected R/W published by owner");
                Check.That(!a.PrepareLanding(true, false, f.Packet, f.Knee), "duplicate finalization fails before second advance");
                string report = b.GetSummary(); b.DisablePreserveTrace(); Check.That(b.GetSummary().Contains("HIP_LANDING_PREPARATION_V041400"), "stop preserves trace");
                b.ResetTrace(); Check.That(b.GetSummary().Contains("applied(total-approach-stance)=0|0|0"), "trace reset clears observations");
            }
        }
        private static void Allocation()
        {
            var f = HipLandingWitnessFixtures.All()[0]; var trace = new HipLandingPreparation.Trace(); var r = Eval(f); r.Applied = r.Better;
            for (int i = 0; i < 100; i++) trace.Observe(true, f.Packet, f.Hip, f.HipR, f.HipW, f.Knee, Eval(f));
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 1000; i++) trace.Observe(true, f.Packet, f.Hip, f.HipR, f.HipW, f.Knee, Eval(f));
            Check.That(GC.GetAllocatedBytesForCurrentThread() == before, "allocation-free evaluation/trace");
        }
    }
}
