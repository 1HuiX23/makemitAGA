using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class FootGroundTrajectoryContracts
    {
        internal struct Fixture
        {
            internal FootGroundTrajectory.Input Input;
            internal Quaternion Previous, Goal;
            internal Vector3 W, NominalW;
            internal float Dt;
        }
        internal static void Run()
        {
            Check.Run("Foot ground choice preserves full Box, native geometry, W/A and braking", GeometryAndBudgets);
            Check.Run("Foot ground choice respects goal, support, ROM and invalid fallback", Fallbacks);
            Check.Run("Foot ground geometry is invariant to world yaw and horizontal translation", Frames);
            Check.Run("Foot selection, held source, recovery and trace reset preserve one Render clock", Planner);
            Check.Run("recursive Foot reference remains continuous through moving inputs at 20/40/60 Hz", Sequences);
            Check.Run("Foot ground choice allocates no managed heap on hot path", Allocation);
            Check.Run("joint feasibility separates subnoise from unresolved without clearing physical geometry", NoiseClassification);
            FootAnkleProxyFoundation.RunFootTrajectoryPublicationContracts();
        }
        private static float Rand(Random r, float lo, float hi) => lo + (hi - lo) * (float)r.NextDouble();
        internal static Fixture RandomFixture(Random random, int hz = 60)
        {
            float dt = 1f / hz;
            var previous = Quaternion.AngleAxis(Rand(random, 1f, 12f), Vector3.right);
            var g = new KneeGroundTarget.Geometry { Hip = new Vector3(0f, 1.025f + Rand(random, 0f, .003f), 0f),
                Upper = Quaternion.identity, UpperSpan = new Vector3(0f, -.45f, 0f), LowerSpan = new Vector3(0f, -.45f, 0f),
                FootRelative = previous, FootBindRelative = Quaternion.identity,
                Center = new Vector3(0f, -.1f, .04f), Half = new Vector3(.05f, .025f, .1f),
                FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f) };
            var source = new SourceStepContext { Epoch = 7, Sequence = 201, Time = 1f + dt, Dt = dt };
            return new Fixture { Previous = previous, Goal = Quaternion.identity, Dt = dt,
                W = new Vector3(Rand(random, -2f, 2f), Rand(random, -.3f, .3f), Rand(random, -.3f, .3f)),
                NominalW = new Vector3(Rand(random, -3f, 3f), Rand(random, -.3f, .3f), Rand(random, -.3f, .3f)),
                Input = new FootGroundTrajectory.Input { Source = source, GeometryStep = new PhysicsStepContext {
                    Epoch = 7, Sequence = 100, SourceSequence = 200, FixedTime = 1f, UnscaledTime = 1f, Dt = 1f / 60f },
                    ParentId = 11, UpperId = 12, LowerId = 13, SupportId = 21, Geometry = g, KneeGoal = Quaternion.identity,
                    NativeLower = Quaternion.identity, NativeAnkle = g.Hip + g.UpperSpan + g.LowerSpan } };
        }
        internal static Fixture Example()
        {
            var rng = new Random(4134);
            for (int n = 0; n < 2000; n++) { var f = RandomFixture(rng); if (Evaluate(f).Better) return f; }
            throw new InvalidOperationException("no improving Foot fixture");
        }
        internal static FootRelativeReference.Sample Baseline(Fixture f) =>
            FootRelativeReference.Build(f.Previous, f.W, f.Goal, f.NominalW, f.Dt);
        internal static FootGroundTrajectory.Result Evaluate(Fixture f)
        {
            var b = Baseline(f);
            return FootGroundTrajectory.Evaluate(f.Input, f.Previous, f.W, f.Goal, f.NominalW, b.Rotation, b.W, f.Dt);
        }
        // Independent corners and stopping-distance oracle, not the selector's predicates.
        private static float Minimum(Vector3 ankle, Quaternion lower, Quaternion foot, KneeGroundTarget.Geometry g)
        {
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                min = MathF.Min(min, (ankle + lower * foot * (g.Center + new Vector3(x * g.Half.x, y * g.Half.y, z * g.Half.z))).y - g.FloorCenter.y - g.FloorHalf.y);
            return min;
        }
        private static float Debt(Vector3 w, Vector3 axis, float gap, float dt) =>
            MathF.Max(0f, MathF.Max(0f, Vector3.Dot(w, axis)) * (w.magnitude / 40f + dt) - gap);
        private static void GeometryAndBudgets()
        {
            var rng = new Random(13401); int improved = 0, noRoom = 0;
            foreach (int hz in new[] { 20, 40, 60 }) for (int n = 0; n < 1800; n++)
            {
                var f = RandomFixture(rng, hz); var b = Baseline(f); var s = Evaluate(f);
                if (s.Reason == FootGroundTrajectory.Status.NoHeadroom) noRoom++;
                if (!s.Better) { Check.That(s.R.Equals(b.Rotation) && s.W.Equals(b.W), "failure keeps exact baseline"); continue; }
                improved++; var g = f.Input.Geometry; var ankle = KneeGroundTarget.Ankle(g, f.Input.KneeGoal); var lower = g.Upper * f.Input.KneeGoal;
                Check.That(!s.Applied && s.GoalMin >= 0f && s.CandidateMin - s.BaselineMin >= .0001f, "pure candidate never self-authorizes");
                Check.Near(Minimum(ankle, lower, s.R, g), s.CandidateMin, 3e-7f, "full eight-corner clearance");
                Check.That(s.NativeCandidateMin >= s.NativeBaselineMin, "native Lower clearance cannot regress");
                Check.That(s.W.magnitude <= MathF.Max(3.75f, f.W.magnitude) + .0001f && (s.W - f.W).magnitude / f.Dt <= 20.0002f, "unchanged budget spheres");
                Check.That(FootRelativeReference.Log(s.R * Quaternion.Inverse(FootRelativeReference.Advance(f.Previous, s.W, f.Dt))).magnitude < 1e-6f, "pose integrates selected W");
                var old = FootRelativeReference.Log(f.Goal * Quaternion.Inverse(b.Rotation)); var after = FootRelativeReference.Log(f.Goal * Quaternion.Inverse(s.R));
                var axis = old.normalized;
                Check.That(after.magnitude <= old.magnitude + 1e-6f && Vector3.Dot(after, axis) >= -1e-6f, "no target overshoot or larger gap");
                Check.That(Debt(s.W - f.NominalW, axis, Vector3.Dot(after, axis), f.Dt) <= Debt(b.W - f.NominalW, axis, old.magnitude, f.Dt) + 1.1e-6f, "no extra relative-W braking debt");
                Check.That(FootGroundReference.GoalWithinBounds(lower, g.FootRelative, s.R, g.FootBindRelative), "original ROM, correction and upright cone");
                Check.That(FootBoxGeometry.Inside(ankle, lower * s.R, g.Center, g.Half, g.FloorCenter, g.FloorHalf), "complete support footprint");
            }
            Console.WriteLine("Foot synthetic improved=" + improved + " noHeadroom=" + noRoom);
            Check.That(improved > 50 && noRoom > 50, "both improvement and budget limitation exercised");
        }
        private static void Fallbacks()
        {
            var f = Example();
            for (int i = 0; i < 9; i++)
            {
                var bad = f;
                if (i == 0) bad.Input.NativeLower = default;
                if (i == 1) bad.Input.Source.Dt = .1f;
                if (i == 2) bad.Input.GeometryStep.Epoch++;
                if (i == 3) bad.Input.ParentId = bad.Input.UpperId;
                if (i == 4) bad.Input.Geometry.FloorHalf.x = .001f;
                if (i == 5) bad.Input.NativeAnkle.x = 10f;
                if (i == 6) bad.Input.Geometry.FootBindRelative = Quaternion.AngleAxis(90f, Vector3.right);
                if (i == 7) { bad.Input.Geometry.Hip.y -= .1f; bad.Input.NativeAnkle.y -= .1f; }
                if (i == 8) bad.Input.NativeLower = Quaternion.AngleAxis(179f, Vector3.right);
                Check.That(!Evaluate(bad).Better, "invalid or geometrically unsafe candidate " + i);
            }
            var nativeConflict = f; nativeConflict.Input.NativeLower = Quaternion.AngleAxis(-20f, Vector3.right);
            Check.That(!Evaluate(nativeConflict).Better, "joint clearance cannot buy worse native-Lower clearance");
            var clear = f; clear.Input.Geometry.Hip.y += .3f; clear.Input.NativeAnkle.y += .3f;
            Check.That(Evaluate(clear).Reason == FootGroundTrajectory.Status.BaselineClear, "already-clear baseline unchanged");
            var mismatch = f; mismatch.Input.GeometryStep.UnscaledTime -= .2f;
            Check.That(!Evaluate(mismatch).Evaluated, "expired native snapshot is unavailable");
        }
        private static void Frames()
        {
            var f = Example(); var a = Evaluate(f); var yaw = Quaternion.AngleAxis(113f, Vector3.up); var shift = new Vector3(2f, 0f, -1f);
            var g = f.Input.Geometry;
            g.Hip = yaw * g.Hip + shift; g.Upper = yaw * g.Upper; g.FloorCenter += shift;
            f.Input.Geometry = g; f.Input.NativeAnkle = yaw * f.Input.NativeAnkle + shift; f.Input.NativeLower = yaw * f.Input.NativeLower;
            var b = Evaluate(f);
            Check.That(b.Better, "world yaw retains improving local step");
            Check.Near((a.W - b.W).magnitude, 0f, 1e-5f, "same Lower-local angular velocity");
            Check.Near(a.CandidateMin, b.CandidateMin, 1e-6f, "same ground clearance");
        }
        private static FootRelativeReference Planner(Fixture f, bool selected)
        {
            var p = new FootRelativeReference(); p.Select(true);
            var source = f.Input.Source; source.Sequence--; source.Time -= f.Dt;
            Check.That(p.Seed(source, f.Previous, f.W), "seed"); p.SelectGroundTrajectory(selected); return p;
        }
        private static void Planner()
        {
            var f = Example(); f.Dt = f.Input.Source.Time - 1f;
            var a = Planner(f, false); var b = Planner(f, true); var silent = Planner(f, true);
            var baseline = FootRelativeReference.Build(a.Current.Rotation, a.Current.W, f.Goal, f.NominalW, f.Dt);
            a.ObserveRender(true, f.Input.Source, f.Goal, f.NominalW, f.Dt, f.Input);
            b.ObserveRender(true, f.Input.Source, f.Goal, f.NominalW, f.Dt, f.Input);
            silent.ObserveRender(false, f.Input.Source, f.Goal, f.NominalW, f.Dt, f.Input);
            Check.That(a.Current.Rotation.Equals(baseline.Rotation) && a.Current.W.Equals(baseline.W) && a.Current.A.Equals(baseline.A), "A exactly preserves original step");
            Check.That(a.Current.GroundTrajectory.Better && !a.Current.GroundTrajectory.Applied && b.Current.GroundTrajectory.Applied, "only selected B applies common same-input candidate");
            Check.That(b.Current.Rotation.Equals(silent.Current.Rotation) && b.Current.W.Equals(silent.Current.W) && b.Current.A.Equals(silent.Current.A), "trace cannot change execution");
            Check.Near((b.Current.A - (b.Current.W - f.W) / f.Dt).magnitude, 0f, 1e-6f, "published A from selected W");
            string details = b.GetDetails(); var before = b.Current;
            b.ObserveRender(true, f.Input.Source, f.Goal, f.NominalW, f.Dt, default);
            Check.That(b.Current.Equals(before) && b.GetDetails().Substring(b.GetDetails().IndexOf("FOOT_GROUND_TRAJECTORY_V041340", StringComparison.Ordinal)) == details.Substring(details.IndexOf("FOOT_GROUND_TRAJECTORY_V041340", StringComparison.Ordinal)), "held source integrates and counts nothing");
            b.ResetTrace(); Check.That(b.Current.Equals(before) && b.GroundTrajectorySelected && !b.GetDetails().Contains("FOOT_TRAJECTORY_WITNESS_V041340"), "trace reset changes no live state");
            b.DisableGroundTrajectoryPreserveTrace(); Check.That(b.Current.Equals(before) && !b.GroundTrajectorySelected, "quiescing choice does not rebase reference");
            for (int fault = 0; fault < 4; fault++)
            {
                var p = Planner(f, true); var input = f.Input;
                if (fault == 0) input.Source.Sequence++;
                if (fault == 1) input.GeometryStep.UnscaledTime -= .2f;
                if (fault == 2) input.Source.Epoch++;
                if (fault == 3) input.SupportId = 0;
                p.ObserveRender(true, f.Input.Source, f.Goal, f.NominalW, f.Dt, input);
                Check.That(p.Finite && p.Current.Rotation.Equals(baseline.Rotation) && !p.Current.GroundTrajectory.Applied, "foreign optional geometry falls back " + fault);
            }
            var gap = Planner(f, true); var gsource = f.Input.Source; gsource.Time += .3f; gsource.Dt += .3f;
            gap.ObserveRender(true, gsource, f.Goal, f.NominalW, gsource.Time - 1f, f.Input);
            Check.That(gap.Finite && gap.Current.GapRecovery && !gap.Current.GroundTrajectory.Evaluated && gap.Current.Dt <= 1f / 60f, "long-gap recovery never runs ground catch-up");
            b.DisablePreserveTrace(); b.Select(true); Check.That(!b.GroundTrajectorySelected && !b.Active, "restart discards previous selected arm");
        }
        private static void Allocation()
        {
            var f = Example(); for (int i = 0; i < 100; i++) Evaluate(f);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 1000; i++) Evaluate(f);
            Check.That(before == GC.GetAllocatedBytesForCurrentThread(), "bounded helper allocates zero");
        }
        private static void Sequences()
        {
            int applied = 0;
            foreach (int hz in new[] { 20, 40, 60 }) foreach (bool selected in new[] { false, true })
            {
                var f = RandomFixture(new Random(13403), hz); var p = Planner(f, selected);
                Quaternion previousGoal = f.Goal;
                for (int n = 1; n <= 240; n++)
                {
                    var before = p.Current; var source = before.Source;
                    source.Sequence++; source.Time = 1f + n / (float)hz; source.Dt = source.Time - before.Source.Time;
                    var input = f.Input; input.Source = source; input.GeometryStep.Sequence = 100 + n;
                    input.GeometryStep.SourceSequence = before.Source.Sequence; input.GeometryStep.UnscaledTime = before.Source.Time;
                    Quaternion goal = Quaternion.AngleAxis(3f * MathF.Sin(n * .12f), Vector3.right);
                    var g = input.Geometry; g.FootRelative = FootRelativeReference.Advance(goal, Vector3.right, .08f);
                    float min = KneeGroundTarget.Minimum(g, input.KneeGoal, goal);
                    g.Hip.y += .001f - min; input.Geometry = g;
                    input.NativeAnkle = KneeGroundTarget.Ankle(g, input.KneeGoal);
                    Vector3 nominal = FootRelativeReference.Log(goal * Quaternion.Inverse(previousGoal)) / source.Dt;
                    var baseline = FootRelativeReference.Build(before.Rotation, before.W, goal, nominal, source.Dt);
                    p.ObserveRender(true, source, goal, nominal, source.Dt, input);
                    var next = p.Current;
                    Check.That(p.Finite && next.Source.Matches(source), "current Render consumed exactly once");
                    Check.Near((next.A - (next.W - before.W) / source.Dt).magnitude, 0f, 1e-5f, "recursive published derivative");
                    Check.That(FootRelativeReference.Log(next.Rotation * Quaternion.Inverse(FootRelativeReference.Advance(before.Rotation, next.W, source.Dt))).magnitude < 1e-6f,
                        "recursive selected pose integral");
                    Check.That(next.A.magnitude <= 20.002f && next.W.magnitude <= MathF.Max(3.75f, before.W.magnitude) + .0001f, "recursive original bounds");
                    if (!selected) Check.That(next.Rotation.Equals(baseline.Rotation) && next.W.Equals(baseline.W), "A exact original recursive output");
                    if (next.GroundTrajectory.Applied) applied++;
                    if (n % 11 == 0) { p.ObserveRender(true, source, goal, nominal, source.Dt, default); Check.That(p.Current.Equals(next), "duplicate never integrates"); }
                    if (n == 120) { p.ResetTrace(); Check.That(p.Current.Equals(next) && p.GroundTrajectorySelected == selected, "midrun trace reset is statistics only"); }
                    previousGoal = goal;
                }
            }
            Check.That(applied > 0, "recursive moving-source sequence includes executed choices");
        }
        private static void NoiseClassification()
        {
            var g = KneeGroundFeasibilityContracts.Geometry(); var s = KneeGroundFeasibilityContracts.Sample(g);
            var trace = new KneeGroundFeasibility.Trace(); var p = new KneeFootGroundPacket { Geometry = g, Sample = s };
            foreach (float min in new[] { -.00000001f, -.000099999f, -.0001f })
            {
                s.Source.Sequence++; s.Step.SourceSequence = s.Source.Sequence; s.GoalMin = min;
                KneeGroundFeasibility.AssessFixed(ref s, g);
                Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalWithinNoise && s.GoalMin == min && s.PlanMin < 0f,
                    "noise category preserves signed target and final reference penetration");
                trace.Observe(s, p, new KneeReferenceState { Step = s.Step, Source = s.Source, R = s.Goal });
            }
            Check.That(trace.Summary().Contains("sourceGoalBelow(total-heightDeficit-searchUnresolved-boundUnavailable-subNoise)=3|0|0|0|3") &&
                trace.Summary().Contains("unresolvedWitness=NONE"), "subnoise cannot flood unresolved counters or witness");
            s.GoalMin = -.00010001f; s.Feasibility.CeilingMin = .1f; KneeGroundFeasibility.AssessFixed(ref s, g);
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalBelowSearchUnresolved, "material search miss remains unresolved");
        }
    }
}
