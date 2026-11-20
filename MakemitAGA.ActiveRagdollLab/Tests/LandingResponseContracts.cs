using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
namespace ARL.Contracts
{
    internal static class LandingResponseContracts
    {
        internal static void Run()
        {
            Check.Run("v140 response joins actual Box corners with command plane, not reference gain", Geometry);
            Check.Run("v140 response rejects missing, switched-support and interrupted endpoints", Rejection);
            Check.Run("v140 response censored/unavailable counts, reset and zero allocation", Lifecycle);
        }
        private static LandingPreparationCommand Command()
        {
            var f = HipLandingWitnessFixtures.All()[0]; var g = f.Packet.Geometry;
            return new LandingPreparationCommand { Step = f.Knee.Step, Source = f.Knee.Source, HipApplied = true, Approach = true,
                SupportId = 21, HipGain = .001f, Center = g.Center, Half = g.Half, FloorCenter = g.FloorCenter, FloorHalf = g.FloorHalf };
        }
        private static FootFixedStepEvidence Pose(int n, float y) => new FootFixedStepEvidence {
            FixedSequence = n, Valid = true, PreparationPoseValid = true, PreparationSupportId = 21,
            FootAnchorWorld = new Vector3(0f, y, 0f), AnchorErrorMeters = .005f, ContactActiveBeforeLifecycle = true };
        private static PhysicsStepContext Next(LandingPreparationCommand c) { var s = c.Step; s.Sequence++; s.FixedTime += s.Dt; s.UnscaledTime += s.Dt; return s; }
        private static void Geometry()
        {
            var c = Command(); var q = Quaternion.AngleAxis(13f, Vector3.forward); var before = Pose(501, .1f); var after = Pose(502, .098f);
            var s = LandingPreparationResponse.Measure(c, before, q, Next(c), after, q);
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                min = MathF.Min(min, (after.FootAnchorWorld + q * (c.Center + new Vector3(x * c.Half.x, y * c.Half.y, z * c.Half.z))).y - c.FloorCenter.y - c.FloorHalf.y);
            Check.That(s.Valid && s.AfterMin < s.BeforeMin, "positive reference gain can coexist with worse actual height");
            Check.Near(s.AfterMin, min, .000002f, "minimum is actual rotated box corner");
            Check.Near(s.AfterMin - s.BeforeMin, -.002f, .000001f, "actual vertical motion is observed without crediting reference gain");
        }
        private static void Rejection()
        {
            for (int i = 0; i < 13; i++)
            {
                var c = Command(); var next = Next(c); var a = Pose(501, .1f); var b = Pose(502, .1f); var q = Quaternion.identity;
                if (i == 0) next.Epoch++;
                if (i == 1) next.Sequence++;
                if (i == 2) b.FixedSequence++;
                if (i == 3) b.Valid = false;
                if (i == 4) b.PreparationPoseValid = false;
                if (i == 5) b.PreparationSupportId++;
                if (i == 6) a.PreparationSupportId++;
                if (i == 7) c.Source.Sequence++;
                if (i == 8) q = default;
                if (i == 9) b.FootAnchorWorld.y = float.NaN;
                if (i == 10) b.FootAnchorWorld.x = 100f;
                if (i == 11) c.Half.x = -1f;
                if (i == 12) b.AnchorErrorMeters = float.NaN;
                Check.That(!LandingPreparationResponse.Measure(c, a, Quaternion.identity, next, b, q).Valid, "response guard " + i);
            }
        }
        private static void Lifecycle()
        {
            var c = Command(); var r = new LandingPreparationResponse();
            var s = LandingPreparationResponse.Measure(c, Pose(501,.1f), Quaternion.identity, Next(c), Pose(502,.09f), Quaternion.identity);
            r.Observe(c, true, s); r.Observe(c, false, default); r.Observe(c, true, default);
            Check.That(r.Summary().Contains("commands-resolved-censored-unavailable=3|1|1|1"), "denominator closure");
            Check.That(r.Summary().Contains("APPROACH_LOWEST_AFTER") && r.Summary().Contains("causalIsolation=False"), "bounded approach witness, no causal certification");
            r.Reset(); Check.That(r.Summary().Contains("commands-resolved-censored-unavailable=0|0|0|0"), "reset drops preceding run");
            for (int i = 0; i < 100; i++) r.Observe(c,true,s);
            long start = GC.GetAllocatedBytesForCurrentThread(); for (int i = 0; i < 2000; i++) r.Observe(c,true,s);
            Check.That(GC.GetAllocatedBytesForCurrentThread() == start, "bounded observation allocates no bytes");
        }
    }
}
