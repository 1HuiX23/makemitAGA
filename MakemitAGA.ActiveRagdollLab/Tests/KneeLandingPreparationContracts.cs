using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class KneeLandingPreparationContracts
    {
        internal static void Run()
        {
            Check.Run("v139 recorded right approach accepts bounded extension; left current/coast counterexamples reject", Recorded);
            Check.Run("v139 foreign, stale, protected, partial and invalid inputs preserve baseline", Rejections);
            Check.Run("v139 independent eight-corner oracle verifies both bases, reference budgets and stopping debt", Synthetic);
            Check.Run("v139 original clear-endpoint path keeps priority; selected RWA commits through original planner", Planner);
            Check.Run("v139 target, axis and global coordinate choices stay consistent", Coordinates);
            Check.Run("v139 trace lifecycle has phase-specific counts, preserved stop and no hot-path allocation", TraceAllocation);
            SingleLegShadowChain.RunKneeLandingEntryContracts();
        }
        internal static KneeLandingPreparation.Result Eval(KneeLandingWitnessFixtures.Fixture f) =>
            KneeLandingPreparation.Evaluate(f.Packet, f.Hip, f.HipR, f.HipW, f.Knee);
        internal static float Angle(Quaternion a, Quaternion b) => FootRelativeReference.Log(a * Quaternion.Inverse(b)).magnitude * Mathf.Rad2Deg;
        internal static float Oracle(KneeGroundTarget.Geometry g, Quaternion upper, Quaternion knee, Quaternion foot)
        {
            var lo = upper * knee; var ankle = g.Hip + upper * g.UpperSpan + lo * g.LowerSpan;
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                min = MathF.Min(min, (ankle + lo * foot * (g.Center + new Vector3(x * g.Half.x, y * g.Half.y, z * g.Half.z))).y - g.FloorCenter.y - g.FloorHalf.y);
            return min;
        }
        private static void Validate(KneeLandingWitnessFixtures.Fixture f, KneeLandingPreparation.Result r)
        {
            if (!r.Better) return;
            var k = f.Knee; var g = f.Packet.Geometry; var foot = f.Packet.Sample.PlannedFoot; float dt = k.Step.Dt;
            var hc = FootRelativeReference.Advance(f.HipR, f.HipW, dt); var kc = FootRelativeReference.Advance(r.R, r.W, dt);
            Check.Near(r.Best.ReferenceNow, Oracle(g, f.HipR, r.R, foot), .000002f, "reference current eight corners");
            Check.Near(r.Best.ReferenceCoast, Oracle(g, hc, kc, foot), .000002f, "reference coast eight corners");
            Check.Near(r.Best.NativeNow, Oracle(g, g.Upper, r.R, foot), .000002f, "native current eight corners");
            Check.Near(r.Best.NativeCoast, Oracle(g, g.Upper, kc, foot), .000002f, "frozen native Upper coast eight corners");
            Check.That(r.Best.ReferenceNow >= r.Baseline.ReferenceNow && r.Best.ReferenceCoast >= r.Baseline.ReferenceCoast &&
                r.Best.NativeNow >= r.Baseline.NativeNow && r.Best.NativeCoast >= r.Baseline.NativeCoast &&
                r.Best.Risk - r.Baseline.Risk >= .0001f, "all four heights nonregressing, significant reference risk gain");
            Check.That(r.W.magnitude <= MathF.Max(k.MaxW, k.PreviousW.magnitude) + .00001f &&
                ((r.W - k.PreviousW) / dt).magnitude <= k.MaxA + .0001f, "original W and A balls");
            Check.That(Angle(r.R, FootRelativeReference.Advance(k.PreviousR, r.W, dt)) < .0005f, "same previous R integration");
            var before = FootRelativeReference.Log(k.InputR * Quaternion.Inverse(k.R)); var after = FootRelativeReference.Log(k.InputR * Quaternion.Inverse(r.R));
            var direction = (r.W - k.W).normalized;
            float Debt(Vector3 w, Vector3 e) => MathF.Max(0f, MathF.Max(0f, Vector3.Dot(w - k.InputW, direction)) *
                ((w - k.InputW).magnitude / (2f * k.MaxA) + dt) - Vector3.Dot(e, direction));
            Check.That(after.magnitude <= before.magnitude + .000002f && Vector3.Dot(after, direction) >= -.000002f &&
                Debt(r.W, after) <= Debt(k.W, before) + .000003f, "unchanged endpoint and no added stopping debt");
            float Bend(Quaternion q) => MathF.Acos(Math.Clamp(Vector3.Dot(g.UpperSpan.normalized, (q * g.LowerSpan).normalized), -1f, 1f)) * Mathf.Rad2Deg;
            Check.That(Bend(r.R) >= 5f && Bend(r.R) <= 150f && Bend(r.R) >= MathF.Min(Bend(k.R), Bend(k.InputR)) - .0002f &&
                Bend(r.R) <= MathF.Max(Bend(k.R), Bend(k.InputR)) + .0002f, "original bend branch with straight-knee margin");
            foreach (var upper in new[] { f.HipR, g.Upper })
                Check.That(FootGroundReference.GoalWithinBounds(upper * r.R, g.FootRelative, foot, g.FootBindRelative), "unchanged full Foot ROM");
        }
        private static void Recorded()
        {
            var all = KneeLandingWitnessFixtures.All(); int accepted = 0;
            foreach (var f in all)
            {
                var r = Eval(f); Validate(f, r);
                Console.WriteLine("  v139 recorded " + f.Name + " shared=" + f.Knee.Step.Sequence + " reason=" + r.Reason +
                    " nativeRejected=" + r.NativeRejected + " deltaDeg=" + r.DeltaDegrees);
                bool right = f.Name == "RIGHT_PEAK_SAMPLED_GAIN";
                Check.That(r.Better == right && !r.Applied, "recorded expected outcome " + f.Name);
                if (right)
                {
                    accepted++; Check.Near(r.DeltaDegrees, .5779733f, .00005f, "recorded right bounded extension");
                    Check.Near(r.Best.ReferenceNow * 1000f, -2.734904f, .005f, "recorded right current partial gain");
                    Check.That(r.Best.Risk < 0f && r.Approach, "partial approach is not certified clearance");
                }
                if (f.Name.StartsWith("LEFT_") && !f.Name.Contains("APPROACH")) Check.That(r.NativeRejected == 4, "all four left candidates rejected by native guard");
                if (f.Knee.Step.Sequence == 5616) Check.That(r.Reason == KneeLandingPreparation.Status.NoHeadroom && r.Tested == 0, "no progress room is not failed geometry search");
            }
            Check.That(accepted == 1, "one distinct retained state passes");
        }
        private static void Rejections()
        {
            var original = KneeLandingWitnessFixtures.All()[3];
            for (int i = 0; i < 26; i++)
            {
                var f = original;
                if (i == 0) f.Packet.Published = false;
                if (i == 1) f.Packet.Input.Apply = false;
                if (i == 2) f.Packet.Sample.Source.Sequence++;
                if (i == 3) f.Packet.PreparationSupport.Step.Sequence++;
                if (i == 4) f.Packet.PreparationSupport.SupportId++;
                if (i == 5) f.Packet.PreparationSupport.StrongStance = true;
                if (i == 6) f.Packet.PreparationSupport.StrongLandingApproach = false;
                if (i == 7) f.Packet.PreparationSupport.SourceAge = float.NaN;
                if (i == 8) f.Hip.Step.Sequence++;
                if (i == 9) f.Hip.Applied = false;
                if (i == 10) f.Hip.ReceiverParentReference = false;
                if (i == 11) f.Hip.SampleAgePrediction = true;
                if (i == 12) f.Knee.Stale = true;
                if (i == 13) f.Hip.A.x += .1f;
                if (i == 14) f.HipR = Quaternion.AngleAxis(1f, Vector3.up) * f.HipR;
                if (i == 15) f.Knee.W.x = float.NaN;
                if (i == 16) f.Packet.Geometry.Hip.y = float.NaN;
                if (i == 17) f.Packet.Geometry.FloorHalf.x = .001f;
                if (i == 18) f.Packet.Input.Axis = Vector3.zero;
                if (i == 19) f.Packet.Sample.PlannedFoot = Quaternion.AngleAxis(180f, Vector3.forward);
                if (i == 20) f.Knee.GroundTrajectory.Applied = true;
                if (i == 21) { f.Knee.Step.Dt = 0f; f.Hip.Step = f.Knee.Step; }
                if (i == 22) f.Packet.Input.CompletedStep.Sequence = f.Knee.Step.Sequence;
                if (i == 23)
                {
                    // Individually fresh Render packet and .07 s support become
                    // too old together at a later held Fixed consumer.
                    f.Packet.PreparationSupport.SourceAge = .07f;
                    Check.That(f.Packet.PreparationSupport.EligibleForSource(f.Knee.Source, f.Packet.Input.CompletedStep, f.Packet.SupportId), "support was eligible at Render");
                    f.Knee.Step.FixedTime += .03f; f.Knee.Step.UnscaledTime += .03f; f.Hip.Step = f.Knee.Step;
                }
                if (i == 24) f.Packet.Input.AllowExtension = f.Packet.Sample.AllowExtension = false;
                if (i == 25)
                {
                    // Equal/quantized timestamps cannot erase the elapsed Fixed dt.
                    var completed = f.Packet.Input.CompletedStep; completed.FixedTime = f.Knee.Step.FixedTime; completed.UnscaledTime = f.Knee.Step.UnscaledTime;
                    f.Packet.Input.CompletedStep = f.Packet.Sample.GeometryStep = f.Packet.PreparationSupport.Step = completed;
                    var source = f.Knee.Source; source.Time = f.Knee.Step.UnscaledTime;
                    f.Knee.Source = f.Packet.Input.Source = f.Packet.Sample.Source = source;
                    f.Packet.PreparationSupport.SourceAge = .09f;
                    Check.That(f.Packet.PreparationSupport.EligibleForSource(source, completed, f.Packet.SupportId), "quantized Render witness itself is fresh");
                }
                var r = Eval(f);
                Check.That(!r.Better && r.R.Equals(f.Knee.R) && r.W.Equals(f.Knee.W), "complete baseline fallback " + i);
            }
        }
        private static KneeLandingWitnessFixtures.Fixture From(LandingPreparationFixtures.Fixture f)
        {
            f.Packet.Input.AllowExtension = f.Packet.Sample.AllowExtension = true;
            return new KneeLandingWitnessFixtures.Fixture { Packet = f.Packet, Hip = f.Hip.Current, HipR = f.Hip.Rotation, HipW = f.Hip.W, Knee = f.Knee };
        }
        private static void Synthetic()
        {
            var random = new Random(41390); int accepted = 0, rejected = 0, extension = 0;
            for (int i = 0; i < 1800; i++)
            {
                var f = From(LandingPreparationFixtures.Make(random)); var snapshot = f; var r = Eval(f); Validate(f, r);
                Check.That(f.Equals(snapshot) && r.Tested <= 4 && r.NativeRejected <= r.ReferenceGains, "value-only bounded search");
                if (r.NativeRejected > 0) rejected++;
                if (!r.Better) continue;
                accepted++; if (LandingPreparationMath.Bend(f.Packet.Geometry, r.R) < LandingPreparationMath.Bend(f.Packet.Geometry, f.Knee.R)) extension++;
            }
            Console.WriteLine("  v139 synthetic accepted=" + accepted + " nativeRejectedSamples=" + rejected + " extension=" + extension);
            Check.That(accepted > 20 && rejected > 0 && extension > 0, "exercise positive and negative geometry branches");
        }
        internal static KneeRelativeReference Run(KneeLandingWitnessFixtures.Fixture f, bool selected, bool trace = false)
        {
            var k = f.Knee; var p = new KneeRelativeReference(); p.Reset(true, k.PreviousR, k.PreviousW);
            Check.That(p.Step(trace, k.Step, k.Source, k.InputR, k.InputW, k.Step.UnscaledTime - k.Source.Time, .1f,
                k.MaxW, k.MaxA, 5f, f.Packet, true, selected, f.Hip, f.HipR, f.HipW), "production planner takes recorded input");
            return p;
        }
        private static void Planner()
        {
            foreach (var f in KneeLandingWitnessFixtures.All())
            {
                var a = Run(f, false); var b = Run(f, true); var x = a.Current; var y = b.Current;
                Check.That(Angle(x.R, f.Knee.R) < .0001f && (x.W - f.Knee.W).magnitude < .00002f, "baseline replays retained Knee state");
                bool right = f.Name == "RIGHT_PEAK_SAMPLED_GAIN";
                Check.That(y.LandingPreparation.Applied == right, "real planner application matches pure selector");
                if (!right) Check.That(x.R.Equals(y.R) && x.W.Equals(y.W) && x.A.Equals(y.A), "rejected B preserves complete A state");
                Check.That(y.InputR.Equals(x.InputR) && y.InputW.Equals(x.InputW), "no changed goal or nominal derivative");
                Check.Near((y.A - (y.W - y.PreviousW) / y.Step.Dt).magnitude, 0f, .00001f, "committed A matches selected W");
            }
            var old = KneeGroundTrajectoryContracts.Example();
            var packet = KneeGroundTrajectoryContracts.Packet(old, old.Baseline.Step, old.Baseline.Source);
            var p = new KneeRelativeReference(); p.Reset(true, old.Previous, old.W);
            var b0 = old.Baseline;
            Check.That(p.Step(true, b0.Step, b0.Source, old.Goal, old.SourceW, 0f, .1f, b0.MaxW, b0.MaxA, 5f, packet, true, true), "existing ground path remains usable without Hip optional input");
            Check.That(p.Current.GroundTrajectory.Applied && !p.Current.LandingPreparation.Applied, "old applied choice never stacks with new choice");
        }
        private static void Coordinates()
        {
            var f = KneeLandingWitnessFixtures.All()[3]; var r = Eval(f);
            var other = f; other.Packet.Input.Axis = -other.Packet.Input.Axis;
            Check.That(Eval(other).R.Equals(r.R) && Eval(other).W.Equals(r.W), "axis sign invariant");
            other = f; var t = new Vector3(.3f, 1.5f, -.5f); other.Packet.Geometry.Hip += t; other.Packet.Geometry.FloorCenter += t;
            var changed = Eval(other); Check.That(changed.Better && (changed.W - r.W).magnitude < .000001f, "common translation invariant");
            other = f; var yaw = Quaternion.AngleAxis(90f, Vector3.up);
            other.Packet.Geometry.Hip = yaw * f.Packet.Geometry.Hip; other.Packet.Geometry.FloorCenter = yaw * f.Packet.Geometry.FloorCenter;
            var half = f.Packet.Geometry.FloorHalf; other.Packet.Geometry.FloorHalf = new Vector3(half.z, half.y, half.x);
            other.Packet.Geometry.Upper = yaw * f.Packet.Geometry.Upper;
            other.Hip.PreviousR = yaw * f.Hip.PreviousR; other.Hip.InputR = yaw * f.Hip.InputR; other.HipR = yaw * f.HipR;
            other.Hip.PreviousW = yaw * f.Hip.PreviousW; other.Hip.InputW = yaw * f.Hip.InputW; other.Hip.A = yaw * f.Hip.A; other.HipW = yaw * f.HipW;
            changed = Eval(other); Check.That(changed.Better && (changed.W - r.W).magnitude < .00001f, "global yaw preserves local Knee choice");
            other = f; other.Packet.Sample.PlannedFoot = f.Packet.Geometry.FootRelative;
            Check.That(!Eval(other).R.Equals(r.R) || MathF.Abs(Eval(other).Baseline.Risk - r.Baseline.Risk) > .000001f,
                "published Foot rather than raw Foot changes geometry as expected");
        }
        private static void TraceAllocation()
        {
            var f = KneeLandingWitnessFixtures.All()[3]; var p = Run(f, true, true); var current = p.Current;
            string saved = p.GetSummary(); Check.That(saved.Contains("applied(total-approach-stance)=1|1|0"), "true approach application counted");
            p.ResetTrace(); Check.That(p.Current.Equals(current), "trace reset cannot reseed or clear control receipt");
            Check.That(p.GetSummary().Contains("applied(total-approach-stance)=0|0|0"), "only statistics reset");
            var trace = new KneeLandingPreparation.Trace(); trace.Reset(true); var r = Eval(f); r.Applied = true;
            for (int n = 0; n < 200; n++) trace.Observe(true, f.Packet, f.Hip, f.HipR, f.HipW, f.Knee, r);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int n = 0; n < 2000; n++) { r = Eval(f); trace.Observe(true, f.Packet, f.Hip, f.HipR, f.HipW, f.Knee, r); }
            Check.That(GC.GetAllocatedBytesForCurrentThread() == before, "selector and bounded trace allocate zero managed bytes");
            var b = Run(f, true, true); saved = b.GetSummary(); b.DisablePreserveTrace();
            Check.That(b.GetSummary().Contains("applied(total-approach-stance)=1|1|0"), "disable preserves trace witness");
            b.Reset(true, f.Knee.PreviousR, f.Knee.PreviousW); Check.That(b.GetSummary().Contains("applied(total-approach-stance)=0|0|0"), "restart clears prior trace");
        }
    }
}
