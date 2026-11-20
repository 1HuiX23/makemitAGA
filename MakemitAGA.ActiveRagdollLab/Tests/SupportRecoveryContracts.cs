using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class SupportRecoveryContracts
    {
        internal static void Run()
        {
            Check.Run("support recovery reproduces the 136 logged PD AFF cancellation witness", LoggedWitness);
            Check.Run("both current arms retain 137 B recovery with exact qualified support", Selection);
            Check.Run("recovery anticipates one-step descent and retains upward-velocity damping", Direction);
            Check.Run("recovery preserves original tangent request and shared sphere including saturation", Budgets);
            Check.Run("recovery rejects invalid qualified inputs without borrowing a stale floor", Invalid);
            Check.Run("receiver reference is common 136 B and actual deficit feedback remains active", ReferenceAndFeedback);
            Check.Run("recursive point-mass request fixture checks recovery without pose or reference snaps", Recursive);
            Check.Run("recovery evidence is step-specific and trace reset never selects control", Reports);
            Check.Run("recovery request and aggregate hot path allocate no managed memory", Allocation);
        }
        private static PhysicsStepContext Step(int n = 11, float dt = 1f / 60f) => new PhysicsStepContext
        { Epoch = 7, Sequence = n, SourceSequence = 30 + n / 2, FixedTime = n * dt, UnscaledTime = n * dt, Dt = dt };
        private static PelvisComReferencePlanner.ComPlan Plan(PhysicsStepContext step, float floor = 0f)
        {
            var old = step; old.Sequence--; old.FixedTime -= step.Dt; old.UnscaledTime -= step.Dt;
            var foot = new PreviousFootSupport { Step = old, SupportId = 31, StrongLandingApproach = true, SourceAge = .001f };
            return new PelvisComReferencePlanner.ComPlan { SupportFloorSelected = true, SupportFloorY = floor,
                SupportInput = SupportReferenceContext.Create(step, foot, foot, 31, true) };
        }
        private static SupportNormalRecovery.Result Build(bool b, Vector3 raw, Vector3 body, Vector3 velocity,
            PhysicsStepContext step, PelvisComReferencePlanner.ComPlan plan) =>
            PelvisDynamicReceiverIsolation.BuildCommonAcceleration(b ? PelvisDynamicReceiverIsolation.DynamicHipArmB :
                "KERNEL_RECOVERY_OFF", raw, body, velocity, step, plan);

        private static void LoggedWitness()
        {
            // B(20261115-114944), shared 7:5514/source 5343. Rounded logged inputs;
            // this replays the actual request policy, not the game's future response.
            var step = Step(5514); step.SourceSequence = 5343;
            var reference = new Vector3(0f, 1.13813877f, 0f);
            var body = reference - new Vector3(0f, .0132414103f, 0f);
            var bodyV = new Vector3(0f, .0137700858f, 0f);
            var pd = PelvisComFeedback.Evaluate(reference - body, new Vector3(0f, -.000775055028f, 0f) - bodyV, true);
            var raw = new Vector3(1.77090114f, pd.y - .401502341f, .078835288f);
            var plan = Plan(step, 1.13759828f);
            var a = Build(false, raw, body, bodyV, step, plan);
            var b = Build(true, raw, body, bodyV, step, plan);
            Check.Near(a.Request.y, -.056975335f, .000006f, "logged original downward request");
            Check.Near(b.Request.y, .33277223f, .000006f, "held-floor restoring request using existing gains");
            Check.That(b.Selected && b.Qualified && b.Eligible && b.ExtraY > .38f, "real B policy intervenes despite reference above floor");
            Check.That(b.Request.x == a.Request.x && b.Request.z == a.Request.z && b.RemainingY == 0f,
                "same original tangent request and no unresolved local normal budget");
            Check.That(plan.SupportFloorY == 1.13759828f && reference.y == 1.13813877f, "no reference offset or pose write");
            Console.WriteLine("  logged request-only witness: originalY=" + a.Request.y + ", recoveryY=" + b.Request.y + ", extraY=" + b.ExtraY);
        }
        private static void Selection()
        {
            var step = Step(); var plan = Plan(step); var raw = new Vector3(1f, -1f, 2f); var body = new Vector3(0f, -.01f, 0f);
            foreach (string arm in new[] { null, "OFF", "B_V041360_SUPPORT_REFERENCE_BRAKING", "B_V04830_COHERENT_HIP_NORMAL_NO_AGE_PREDICTION", "B_V041370_SUPPORT_NORMAL_RECOVERY" })
            {
                var r = PelvisDynamicReceiverIsolation.BuildCommonAcceleration(arm, raw, body, default, step, plan);
                Check.That(!r.Selected && r.Request.Equals(raw), "old and unrelated arms cannot select recovery");
            }
            var currentA = PelvisDynamicReceiverIsolation.BuildCommonAcceleration(PelvisDynamicReceiverIsolation.DynamicHipArmA, raw, body, default, step, plan);
            var currentB = PelvisDynamicReceiverIsolation.BuildCommonAcceleration(PelvisDynamicReceiverIsolation.DynamicHipArmB, raw, body, default, step, plan);
            Check.That(currentA.Selected && currentB.Selected && currentA.Request.Equals(currentB.Request), "138 A/B share the real 137 B physical request");
            Check.That(PelvisDynamicReceiverIsolation.UsesKneeLandingPreparation(PelvisDynamicReceiverIsolation.DynamicHipArmA) &&
                PelvisDynamicReceiverIsolation.UsesKneeLandingPreparation(PelvisDynamicReceiverIsolation.DynamicHipArmB) &&
                !PelvisDynamicReceiverIsolation.UsesKneeLandingPreparation(null), "v140 both arms retain v139 B Knee preparation");
            Check.That(!PelvisDynamicReceiverIsolation.UsesHipLandingPreparation(PelvisDynamicReceiverIsolation.DynamicHipArmA) &&
                !PelvisDynamicReceiverIsolation.UsesHipLandingPreparation(PelvisDynamicReceiverIsolation.DynamicHipArmB), "both arms use v142 A Hip selection");
            for (int mode = 0; mode < 8; mode++)
            {
                var bad = plan;
                if (mode == 0) bad.SupportFloorSelected = false;
                if (mode == 1) bad.SupportInput.ForStep.Sequence++;
                if (mode == 2) bad.SupportInput.ForStep.SourceSequence++;
                if (mode == 3) bad.SupportInput.ForStep.Dt *= 2f;
                if (mode == 4) bad.SupportInput.SupportId = 0;
                if (mode == 5) bad.SupportInput.ObservedStep.Epoch++;
                if (mode == 6) bad.SupportInput.StanceMask = 0;
                if (mode == 7) bad.SupportInput.ApproachMask = 4;
                var r = Build(true, raw, body, default, step, bad);
                Check.That(r.Selected && !r.Qualified && r.Request.Equals(raw), "invalid optional support cannot alter command");
            }
            Check.That(Build(true, raw, body, default, step, plan).Support.ApproachOnly, "original strong approach witness retained");
        }
        private static void Direction()
        {
            var step = Step(11, .02f); var plan = Plan(step); var raw = new Vector3(0f, -4f, 0f);
            var anticipated = Build(true, raw, new Vector3(0f, .002f, 0f), new Vector3(0f, -.3f, 0f), step, plan);
            Check.That(anticipated.Eligible && anticipated.BodyY > 0f && anticipated.ProjectedY < 0f && anticipated.Request.y > 0f,
                "qualified one-step descent can brake before actual height crosses");
            var clear = Build(true, raw, new Vector3(0f, .02f, 0f), new Vector3(0f, -.3f, 0f), step, plan);
            Check.That(!clear.Eligible && clear.Request.Equals(raw), "clear body with clear projection stays unchanged");
            var upward = Build(true, raw, new Vector3(0f, -.01f, 0f), new Vector3(0f, .2f, 0f), step, plan);
            Check.That(upward.Request.y < 0f && upward.Request.y > raw.y, "upward velocity retains a negative damping request");
            Check.Near(upward.Request.y, .4f - .2f * PelvisComFeedback.NormalVelocityGain, 1e-6f, "same normal gains, not force-up logic");
            var adequate = Build(true, new Vector3(0f, 4f, 0f), new Vector3(0f, -.01f, 0f), default, step, plan);
            Check.That(adequate.ExtraY == 0f, "already sufficient restoring request is unchanged");
        }
        private static void Budgets()
        {
            var step = Step(); var plan = Plan(step);
            var saturated = Build(true, new Vector3(12f, 0f, 0f), new Vector3(0f, -1f, 0f), default, step, plan);
            Check.That(saturated.ExtraY == 0f && saturated.RemainingY == 40f, "no tangent room is invented to satisfy normal demand");
            var rng = new Random(41370); int raised = 0, limited = 0;
            for (int i = 0; i < 12000; i++)
            {
                float N() => (float)(2d * rng.NextDouble() - 1d);
                var raw = new Vector3(N(), N(), N()) * 30f;
                var body = new Vector3(0f, N() * .2f, 0f); var v = new Vector3(N(), N(), N());
                var a = Build(false, raw, body, v, step, plan); var b = Build(true, raw, body, v, step, plan);
                Check.That(a.Request.Equals(Vector3.ClampMagnitude(raw, 12f)), "A exact original request cap");
                Check.That(b.Request.x == a.Request.x && b.Request.z == a.Request.z && b.Request.y >= a.Request.y,
                    "all tangent bits retained, only upward normal adjustment");
                Check.That(b.Request.magnitude <= 12.00001f, "original total request cap");
                if (b.ExtraY > 1e-5f) raised++;
                if (b.RemainingY > 1e-5f) limited++;
                if (!b.Eligible) Check.That(a.Request.Equals(b.Request), "ineligible control remains identical");
            }
            Check.That(raised > 0 && limited > 0, "exercise both useful and unresolved cases");
        }
        private static void Invalid()
        {
            var step = Step(); var plan = Plan(step); var body = new Vector3(0f, -.01f, 0f);
            foreach (float bad in new[] { float.NaN, float.PositiveInfinity, float.NegativeInfinity })
            {
                Check.Throws<InvalidOperationException>(() => Build(true, new Vector3(bad, 0f, 0f), body, default, step, plan), "invalid baseline");
                Check.Throws<InvalidOperationException>(() => Build(true, default, new Vector3(0f, bad, 0f), default, step, plan), "invalid physical position");
                Check.Throws<InvalidOperationException>(() => Build(true, default, body, new Vector3(0f, bad, 0f), step, plan), "invalid physical velocity");
                var invalid = plan; invalid.SupportFloorY = bad;
                Check.Throws<InvalidOperationException>(() => Build(true, default, body, default, step, invalid), "invalid qualified floor");
            }
        }
        private static void ReferenceAndFeedback()
        {
            var step = Step(); var support = Plan(step).SupportInput;
            var p = new Vector3(0f, -.02f, 0f); var v = new Vector3(0f, .2f, 0f);
            var previous = Build(true, new Vector3(0f, -1f, 0f), new Vector3(0f, -.02f, 0f), default, step, Plan(step));
            var a = PelvisDynamicReceiverIsolation.BuildComReferencePlan(PelvisDynamicReceiverIsolation.DynamicHipArmA, p, v,
                new Vector3(0f, 1f, 0f), default, step.Dt, step, true, previous.Request, previous.Request * .5f,
                v, support, 0f);
            var b = PelvisDynamicReceiverIsolation.BuildComReferencePlan(PelvisDynamicReceiverIsolation.DynamicHipArmB, p, v,
                new Vector3(0f, 1f, 0f), default, step.Dt, step, true, previous.Request, previous.Request * .5f,
                v, support, 0f);
            Check.That(a.Position.Equals(b.Position) && a.Velocity.Equals(b.Velocity) && a.Acceleration.Equals(b.Acceleration) &&
                a.SupportBrakingSelected && b.SupportBrakingSelected, "same references from identical actual receipt in both arms");
            Check.That(a.FeedbackAvailable && a.RemovedAcceleration > 0f && a.PreviousRequestA.Equals(previous.Request),
                "actual recovery request deficit is retained, not discarded baseline or inverse q");
        }
        private static void Recursive()
        {
            foreach (float dt in new[] { 1f / 240f, 1f / 60f, .02f, .05f })
            {
                // Deliberately reduced one-axis plant: same fixed goal and downward
                // AFF disturbance in both arms, unity input response, no contact.
                // Tests request behavior and continuity, not reference or PhysX fidelity.
                float aY = -.02f, bY = aY, aV = 0f, bV = 0f;
                for (int i = 1; i <= 400; i++)
                {
                    var step = Step(i + 10, dt); var plan = Plan(step);
                    var bodyA = new Vector3(0f, aY, 0f); var bodyB = new Vector3(0f, bY, 0f);
                    var va = new Vector3(0f, aV, 0f); var vb = new Vector3(0f, bV, 0f);
                    var disturbance = new Vector3(0f, -.4f, 0f);
                    var a = Build(false, PelvisComFeedback.Evaluate(-bodyA, -va, true) + disturbance, bodyA, va, step, plan);
                    var b = Build(true, PelvisComFeedback.Evaluate(-bodyB, -vb, true) + disturbance, bodyB, vb, step, plan);
                    aV += a.Request.y * dt; bV += b.Request.y * dt;
                    aY += aV * dt; bY += bV * dt;
                    Check.That(Math.Abs(b.Request.y) <= 12.00001f && bY < .01f && bY >= -.020001f, "bounded recursive response without lift jump");
                }
                Check.That(bY > aY + .006f, "recovery removes much of the fixture's persistent downward bias");
                Console.WriteLine("  simplified point-mass fixture: dt=" + dt + ", finalYmm(A|B)=" + aY * 1000f + "|" + bY * 1000f);
            }
        }
        private static void Reports()
        {
            var step = Step(); var plan = Plan(step); var body = new Vector3(0f, -.02f, 0f);
            var r = Build(true, default, body, default, step, plan);
            var report = new PelvisReceiverReport(); report.RecordSupportRecovery(r, step);
            string summary = report.RecoverySummaryForTest();
            Check.That(summary.Contains("receiverCommitted-selected-qualified-eligible-raised-limited=1|1|1|1|1|0") &&
                summary.Contains("BEFORE_BILATERAL_ALLOCATION_NOT_SOLVED"), "actual request domain and intervention are explicit");
            report.ResetSupportRecovery(); report.RecordSupportRecovery(r, Step(12));
            Check.That(report.RecoverySummaryForTest().Contains("=0|0|0|0|0|0"), "reset and wrong-step records cannot invent a new request");
            Check.That(r.Request.Equals(Build(true, default, body, default, step, plan).Request), "trace state does not select control");
        }
        private static void Allocation()
        {
            var step = Step(); var plan = Plan(step); var body = new Vector3(0f, -.02f, 0f); var report = new PelvisReceiverReport();
            for (int i = 0; i < 100; i++) report.RecordSupportRecovery(Build(true, default, body, default, step, plan), step);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 1000; i++) report.RecordSupportRecovery(Build(true, default, body, default, step, plan), step);
            Check.That(GC.GetAllocatedBytesForCurrentThread() == before, "recovery math and aggregation allocate no managed memory");
        }
    }
}
namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class PelvisReceiverReport
    {
        internal string RecoverySummaryForTest() => SupportRecoverySummary();
    }
}
