using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class SupportBrakingContracts
    {
        internal static void Run()
        {
            Check.Run("support braking bound agrees with independent distance bisection", DistanceBound);
            Check.Run("braking kernel contrast and current common receiver selection require exact support context", Selection);
            Check.Run("support braking intervenes above the floor without a height offset", EarlyStep);
            Check.Run("recursive support braking retains speed and acceleration limits", Recursive);
            Check.Run("support braking shares vector budgets and protects nominal tangent braking", VectorBudgets);
            Check.Run("support braking retains actual-completion deficit and physical brake protection", Feedback);
            Check.Run("support braking reports pre-feedback intervention and unresolved final reserve separately", Reports);
            Check.Run("support braking hot path allocates no managed memory", Allocation);
        }
        private static PhysicsStepContext Step(int n = 11, float dt = .02f) => new PhysicsStepContext
        { Epoch = 7, Sequence = n, SourceSequence = 20 + n / 2, FixedTime = n * dt, UnscaledTime = n * dt, Dt = dt };
        private static SupportReferenceContext Support(PhysicsStepContext step)
        {
            var old = step; old.Sequence--; old.FixedTime -= step.Dt; old.UnscaledTime -= step.Dt;
            var witness = new PreviousFootSupport { Step = old, SupportId = 33, StrongLandingApproach = true, SourceAge = .001f };
            return SupportReferenceContext.Create(step, witness, witness, 33, true);
        }
        private static PelvisComReferencePlanner.ComPlan Plan(bool b, Vector3 p, Vector3 v, Vector3 goal, Vector3 goalV,
            PhysicsStepContext step, SupportReferenceContext support, float floor = 0f, bool feedback = false,
            Vector3 request = default, Vector3 allocated = default, Vector3 bodyV = default) =>
            PelvisComReferencePlanner.Build(p, v, goal, goalV, step.Dt, true, feedback, request, allocated, bodyV, true,
                support.Matches(step) ? support : default, support.Matches(step), floor, true, true, b);
        private static void SameControl(PelvisComReferencePlanner.ComPlan a, PelvisComReferencePlanner.ComPlan b, string label)
        {
            Check.That(a.Position.Equals(b.Position) && a.Velocity.Equals(b.Velocity) && a.Acceleration.Equals(b.Acceleration) &&
                a.CandidateA.Equals(b.CandidateA) && a.RemovedAcceleration == b.RemovedAcceleration &&
                a.PhysicalBrakeProtected == b.PhysicalBrakeProtected, label);
        }
        private static void DistanceBound()
        {
            foreach (float floor in new[] { -3f, 0f, 1.14f, 100f })
            foreach (float dt in new[] { 1f / 240f, 1f / 60f, .02f, .05f, .1f })
            foreach (float distance in new[] { .000001f, .0001f, .001f, .02f, .1f, 1f })
            {
                float p = floor + distance;
                float actual = SupportNormalBraking.MinimumVelocity(p, floor, dt, 6f);
                double clearance = (double)p - floor, lo = 0d, hi = 100d;
                for (int i = 0; i < 90; i++)
                {
                    double mid = (lo + hi) / 2d;
                    if (mid * dt + mid * mid / 12d <= clearance) lo = mid; else hi = mid;
                }
                Check.Near(actual, (float)-lo, .000001f, "distance root independent oracle");
                Check.That(actual >= (floor - p) / dt && actual <= 0f, "same floor, earlier downward-speed restriction only");
            }
            Check.Near(SupportNormalBraking.MinimumVelocity(-.02f, 0f, .02f, 6f), 1f, 0f, "below-floor recovery exactly original");
            Check.Near(SupportNormalBraking.MinimumVelocity(0f, 0f, .02f, 6f), 0f, 0f, "no upward standstill bias");
            foreach (float bad in new[] { float.NaN, float.PositiveInfinity, float.NegativeInfinity })
            {
                Check.Throws<InvalidOperationException>(() => SupportNormalBraking.MinimumVelocity(bad, 0f, .02f, 6f), "invalid position");
                Check.Throws<InvalidOperationException>(() => SupportNormalBraking.MinimumVelocity(0f, bad, .02f, 6f), "invalid floor");
                Check.Throws<InvalidOperationException>(() => SupportNormalBraking.MinimumVelocity(0f, 0f, bad, 6f), "invalid clock");
                Check.Throws<InvalidOperationException>(() => SupportNormalBraking.MinimumVelocity(0f, 0f, .02f, bad), "invalid reference bound");
            }
            Check.Throws<InvalidOperationException>(() => SupportNormalBraking.MinimumVelocity(0f, 0f, 0f, 6f), "zero dt");
            Check.Throws<InvalidOperationException>(() => SupportNormalBraking.MinimumVelocity(0f, 0f, .02f, 0f), "zero acceleration bound");
        }
        private static void Selection()
        {
            var step = Step(); var support = Support(step); var p = new Vector3(0f, .02f, 0f); var v = new Vector3(0f, -.4f, 0f);
            var a = Plan(false, p, v, p + v * step.Dt, v, step, support);
            var b = Plan(true, p, v, p + v * step.Dt, v, step, support);
            Check.That(!a.SupportBrakingSelected && b.SupportBrakingSelected && b.SupportInput.ApproachOnly, "historical kernel contrast; approach unchanged");
            foreach (string arm in new[] { PelvisDynamicReceiverIsolation.DynamicHipArmA, PelvisDynamicReceiverIsolation.DynamicHipArmB })
            {
                var current = PelvisDynamicReceiverIsolation.BuildComReferencePlan(arm, p, v, p + v * step.Dt, v,
                    step.Dt, step, false, default, default, default, support, 0f);
                Check.That(current.SupportBrakingSelected, "both 137 arms retain actual 136 B reference braking");
                SameControl(current, b, "current reference control equals old B kernel");
            }
            foreach (string old in new[] { null, "OFF", "B_V041350_KNEE_PROGRESS_FOOT_TARGET", "B_V04830_COHERENT_HIP_NORMAL_NO_AGE_PREDICTION" })
                Check.That(!PelvisDynamicReceiverIsolation.UsesSupportBraking(old), "retired and unrelated scope cannot select new behavior");
            var baseline = Plan(false, p, v, p, v, step, default);
            foreach (var invalid in new[] { default(SupportReferenceContext), Mutate(support, 0), Mutate(support, 1), Mutate(support, 2), Mutate(support, 3) })
            {
                var absent = Plan(true, p, v, p, v, step, invalid);
                SameControl(absent, baseline, "absent/wrong step/dt/source/support returns exact ungated plan");
                Check.That(!absent.SupportBrakingSelected && !absent.SupportInput.Matches(step), "no stale activation or current relabeling");
            }
            var wrong = step; wrong.Dt *= 2f;
            Check.Throws<InvalidOperationException>(() => PelvisDynamicReceiverIsolation.BuildComReferencePlan(PelvisDynamicReceiverIsolation.DynamicHipArmB,
                p, v, p, v, step.Dt, wrong, false, default, default, default, Support(wrong), 0f), "current support dt must match integration dt");
        }
        private static SupportReferenceContext Mutate(SupportReferenceContext s, int mode)
        {
            if (mode == 0) s.ForStep.Sequence++; else if (mode == 1) s.ForStep.Dt *= 2f;
            else if (mode == 2) s.ForStep.SourceSequence++; else s.SupportId = 0;
            return s;
        }
        private static void EarlyStep()
        {
            var step = Step(); var support = Support(step); var p = new Vector3(0f, .02f, 0f); var v = new Vector3(0f, -.4f, 0f);
            var a = Plan(false, p, v, p + v * step.Dt, v, step, support);
            var b = Plan(true, p, v, p + v * step.Dt, v, step, support);
            Check.That(a.Position.y > 0f && a.SupportBrakingBaselineResidual > .001f, "one-step floor clear while reference lacks stopping distance");
            Check.That(b.SupportBrakingExtraY > .001f && b.SupportBrakingFinalResidual < .000001f, "earlier braking inside original budget");
            Check.That(b.Position.y < p.y && b.Velocity.y < 0f && b.SupportFloorY == a.SupportFloorY, "continues downward without lifting or moving the floor");
            Check.That(b.Acceleration.magnitude <= 6.001f && b.Velocity.magnitude <= 3.00001f, "bounded actual reference output");
            foreach (float y in new[] { -.02f, 0f, .2f })
            {
                p.y = y; v = Vector3.zero;
                SameControl(Plan(false, p, v, p, v, step, support), Plan(true, p, v, p, v, step, support), "under-floor recovery, floor standstill and clear standstill unchanged");
            }
        }
        private static void Recursive()
        {
            foreach (float dt in new[] { 1f / 240f, 1f / 60f, .02f, .05f })
            {
                var pa = new Vector3(0f, .08f, 0f); var pb = pa;
                var va = new Vector3(0f, -.6f, 0f); var vb = va;
                float minA = pa.y, minB = pb.y; int advanced = 0;
                for (int i = 1; i <= 400; i++)
                {
                    var step = Step(i + 10, dt); var support = Support(step);
                    // Deliberately conflicting downward input: tests reference planning,
                    // not Animator, body integration, contacts or a real motion rollout.
                    var a = Plan(false, pa, va, new Vector3(0f, -.1f, 0f), new Vector3(0f, -.6f, 0f), step, support);
                    var b = Plan(true, pb, vb, new Vector3(0f, -.1f, 0f), new Vector3(0f, -.6f, 0f), step, support);
                    Check.That(b.Velocity.magnitude <= 3.00001f && b.Acceleration.magnitude <= 6.001f, "recursive caps");
                    Check.That((pb + b.Velocity * dt).Equals(b.Position), "same semi-implicit integration, no pose snap");
                    pa = a.Position; va = a.Velocity; pb = b.Position; vb = b.Velocity;
                    minA = Math.Min(minA, pa.y); minB = Math.Min(minB, pb.y);
                    if (b.SupportBrakingExtraY > .000001f) advanced++;
                }
                Check.That(advanced > 0 && minA < -.0001f && minB > -.000002f, "static one-dimensional recursive stopping witness");
                Console.WriteLine("  reference-only braking fixture: dt=" + dt.ToString("G9", CultureInfo.InvariantCulture) +
                    ", minMm(A|B)=" + (1000f * minA).ToString("G9", CultureInfo.InvariantCulture) + "|" +
                    (1000f * minB).ToString("G9", CultureInfo.InvariantCulture) + ", advanced=" + advanced);
            }
        }
        private static void VectorBudgets()
        {
            var rng = new Random(41360); int advanced = 0, tangentProtected = 0, residuals = 0;
            for (int i = 0; i < 2500; i++)
            {
                float N() => (float)(2d * rng.NextDouble() - 1d);
                float dt = .005f + .045f * (float)rng.NextDouble(); var step = Step(11, dt); var support = Support(step);
                var p = new Vector3(0f, .05f * N(), 0f); var v = Vector3.ClampMagnitude(new Vector3(N(), N(), N()) * 2f, 3f);
                var goal = new Vector3(N(), N(), N()); var goalV = new Vector3(N(), N(), N());
                var req = new Vector3(N(), N(), N()) * 5f; var alloc = req * .5f; var body = new Vector3(N(), N(), N());
                var a = Plan(false, p, v, goal, goalV, step, support, 0f, true, req, alloc, body);
                var b = Plan(true, p, v, goal, goalV, step, support, 0f, true, req, alloc, body);
                Check.That(b.Velocity.magnitude <= 3.00001f && b.Acceleration.magnitude <= 6.001f, "shared spheres after original feedback");
                Check.That(b.SupportBrakingExtraY >= -.000001f, "candidate normal never reduced");
                Check.That(b.Acceleration.Equals((b.Velocity - v) / dt) && b.Position.Equals(p + b.Velocity * dt), "published p/v/a consistent");
                if (b.SupportTangentBrakingPreserved)
                {
                    tangentProtected++;
                    Check.That(b.CandidateA.x == a.CandidateA.x && b.CandidateA.z == a.CandidateA.z, "nominal tangential braking kept before feedback");
                }
                if (b.SupportBrakingExtraY > .000001f) advanced++;
                if (b.SupportBrakingFinalResidual > .000001f) residuals++;
                if (p.y <= 0f) SameControl(a, b, "new policy does not alter already-below recovery");
            }
            Check.That(advanced > 0 && tangentProtected > 0 && residuals > 0, "exercise useful, protected and unresolved cases");
        }
        private static void Feedback()
        {
            var step = Step(); var support = Support(step);
            var p = new Vector3(0f, -.02f, 0f); var v = new Vector3(0f, .2f, 0f);
            var b = Plan(true, p, v, new Vector3(0f, 1f, 0f), default, step, support, 0f, true,
                new Vector3(0f, 2f, 0f), default, new Vector3(0f, -.2f, 0f));
            Check.That(b.PhysicalBrakeProtected && b.RemovedAcceleration == 0f, "existing real downward velocity protects upward brake");
            b = Plan(true, p, v, new Vector3(0f, 1f, 0f), default, step, support, 0f, true,
                new Vector3(0f, 2f, 0f), default, new Vector3(0f, .2f, 0f));
            Check.That(b.RemovedAcceleration > 0f && b.SupportBrakingFinalResidual > 0f, "braking policy does not bypass original deficit or hide unresolved floor");
        }
        private static void Reports()
        {
            var step = Step(); var support = Support(step); var p = new Vector3(0f, .02f, 0f); var v = new Vector3(0f, -.4f, 0f);
            var b = Plan(true, p, v, p + v * step.Dt, v, step, support);
            var report = new PelvisReceiverReport(); report.ResetSupportReference(); report.RecordSupportReference(b, step);
            string text = report.SupportSummaryForTest();
            Check.That(text.Contains("selected-advanced-finalResidual=1|1|0") && text.Contains("brakingAdvancePeak") &&
                text.Contains("NOT_BODY_CAPABILITY"), "real report distinguishes input selection, bounded intervention and domain");
            report.ResetSupportReference();
            Check.That(report.SupportSummaryForTest().Contains("selected-advanced-finalResidual=0|0|0"), "trace statistics reset independently");
            SameControl(b, Plan(true, p, v, p + v * step.Dt, v, step, support), "trace reset has no controller effect");
            report.RecordSupportReference(b, Step(12));
            Check.That(report.SupportSummaryForTest().Contains("selected-advanced-finalResidual=0|0|0"), "report rejects mismatched support step");
        }
        private static void Allocation()
        {
            var step = Step(); var support = Support(step); var p = new Vector3(0f, .02f, 0f); var v = new Vector3(0f, -.4f, 0f);
            var report = new PelvisReceiverReport();
            for (int i = 0; i < 100; i++) report.RecordSupportReference(Plan(true, p, v, p + v * step.Dt, v, step, support), step);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 1000; i++) report.RecordSupportReference(Plan(true, p, v, p + v * step.Dt, v, step, support), step);
            Check.That(GC.GetAllocatedBytesForCurrentThread() == before, "planner and aggregate hot path allocation-free");
        }
    }
}

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class PelvisReceiverReport
    {
        private static string F(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        internal string SupportSummaryForTest() => SupportReferenceSummary();
    }
}
