using System;
using System.Reflection;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class HipBoundedTradeContracts
    {
        internal static void Run()
        {
            Check.Run("conditional CAP10 preserves same-q fallback and all nine budgets", Sweep);
            Check.Run("bounded trade scale contract rejects over-cap and unselected allocation", ScaleContract);
            Check.Run("bounded trade intersects both interval ends", Interval);
            Check.Run("actual trade precommit rejects old angular and alpha receipts", Precommit);
            Check.Run("CAP10 keeps gyro and transport ownership separate from joint reaction", MixedAngularOwnership);
            Check.Run("allocation report counts completed commands without re-solving", Report);
            Check.Run("bounded allocation and reporting have no per-step heap allocations", NoHeap);
            PelvisDynamicReceiverIsolation.RunTradeCompletionContracts();
        }
        private static float R(Random rng, float span) => ((float)rng.NextDouble() * 2f - 1f) * span;
        private static Vector3 V(Random rng, float span) => new Vector3(R(rng, span), R(rng, span), R(rng, span));
        private static void Generate(Random rng, int i, out HipAnchorPairInput l, out HipAnchorPairInput r)
        {
            var p = PelvisDynamicReceiverIsolation.Baseline(PelvisDynamicReceiverIsolation.LoggedStep(i));
            p.Acceleration = new Vector3(R(rng, 5f), .1f + (float)rng.NextDouble() * 8f, R(rng, 5f));
            p.ReceiverTorque = V(rng, .5f);
            p.UpwardRetentionSelected = p.UpwardRetentionEligible = true;
            bool simple = (i & 1) == 0;
            if (!simple) Check.That(PelvisChestPointResponse.TryCreate(2f + (float)rng.NextDouble() * 15f,
                3f + (float)rng.NextDouble() * 20f, new Vector3(.2f, .4f, .3f),
                Quaternion.AngleAxis(R(rng, 180f), new Vector3(1f, 2f, 3f)), new Vector3(.03f, .3f, -.05f), out p.Response), "coupled fixture");
            float span = i % 10 == 0 ? 0f : 8f, feedback = i % 10 == 0 ? 0f : 25f;
            l = HipAllocationContracts.Input(p, true, V(rng, span), V(rng, feedback),
                new Vector3(R(rng, .15f), .2f, R(rng, .1f)), new Vector3(-.1f, -.05f, .03f),
                simple ? 1f : 1f + (float)rng.NextDouble() * 24f,
                simple ? new Vector3(1f, 1f, 1f) : new Vector3(.15f, .08f, .2f), Quaternion.AngleAxis(R(rng, 180f), Vector3.forward));
            r = HipAllocationContracts.Input(p, false, V(rng, span), V(rng, feedback),
                new Vector3(R(rng, .15f), .2f, R(rng, .1f)), new Vector3(.1f, -.05f, .03f),
                simple ? 1f : 1f + (float)rng.NextDouble() * 24f,
                simple ? new Vector3(1f, 1f, 1f) : new Vector3(.2f, .12f, .15f), Quaternion.AngleAxis(R(rng, 180f), Vector3.right));
        }
        internal static void AppliedFixture(out HipAnchorPairInput l, out HipAnchorPairInput r,
            out HipAnchorPairReport.Command a, out HipAnchorPairReport.Command b)
        {
            var rng = new Random(128);
            for (int i = 0; i < 1200; i++)
            {
                Generate(rng, i, out l, out r);
                // Completion port has the original 4+6 kg model; choose a simple case.
                if ((i & 1) != 0) continue;
                l.Seed.ParentInput.BoundedTradeSelected = r.Seed.ParentInput.BoundedTradeSelected = true;
                if (HipBilateralForceSolver.TryPlan(l, r, out a, out b) && a.AllocationDecision.Applied) return;
            }
            throw new InvalidOperationException("missing applied trade fixture");
        }
        internal static PelvisHipAttitudeCommand Completed(HipAnchorPairReport.Command l, HipAnchorPairReport.Command r)
        {
            Check.That(PelvisHipAttitudeCommand.TryPlanWithReaction(l.ParentInput, -l.Force - r.Force,
                l.ParentTorque + r.ParentTorque, l.BilateralAngularReaction, out var c), "receiver plan");
            c.MotionScale = l.MotionScale; c.UpperAngularScale = l.UpperAngularScale;
            c.UpwardExtraAcceleration = l.UpwardExtraAcceleration; c.CommonAcceleration = l.CommonAcceleration; c.Committed = true;
            return c;
        }
        private static void SamePhysical(HipAnchorPairReport.Command a, HipAnchorPairReport.Command b)
        {
            foreach (var field in typeof(HipAnchorPairReport.Command).GetFields(BindingFlags.Instance | BindingFlags.NonPublic))
                if (field.Name != nameof(a.ParentInput) && field.Name != nameof(a.AllocationDecision))
                    Check.That(object.Equals(field.GetValue(a), field.GetValue(b)), "fallback exact physical field: " + field.Name);
        }
        private static void Sweep()
        {
            var rng = new Random(128); int accepted = 0, applied = 0, unchanged = 0, fallback = 0, full = 0, rejected = 0;
            for (int i = 0; i < 1200; i++)
            {
                Generate(rng, i, out var l, out var r);
                bool aok = HipBilateralForceSolver.TryPlan(l, r, out var a, out var ar);
                l.Seed.ParentInput.BoundedTradeSelected = r.Seed.ParentInput.BoundedTradeSelected = true;
                bool bok = HipBilateralForceSolver.TryPlan(l, r, out var b, out var br);
                Check.That(aok == bok, "optional exchange never breaks an accepted same-q solve: " + i);
                if (!bok) continue;
                accepted++;
                Check.That(a.MotionScale == b.MotionScale && a.Scale == b.Scale && a.FullFeedbackFeasible == b.FullFeedbackFeasible &&
                    b.UpwardExtraAcceleration >= a.UpwardExtraAcceleration, "unchanged baseline allocation and no lost Y");
                Check.That(b.UpperAngularScale >= b.MotionScale * .9f && b.UpperAngularScale <= b.MotionScale, "ten percent of allocated angle request");
                if (b.MotionScale == 1f) full++;
                if (!b.FullFeedbackFeasible) fallback++;
                if (b.AllocationDecision.Attempted && !b.AllocationDecision.CandidateValid) rejected++;
                if (!b.AllocationDecision.Applied) { unchanged++; SamePhysical(a, b); SamePhysical(ar, br); }
                else
                {
                    applied++;
                    Check.That(b.FullFeedbackFeasible && b.AllocationDecision.CandidateValid &&
                        b.UpwardExtraAcceleration > a.UpwardExtraAcceleration && b.UpperAngularScale < a.UpperAngularScale,
                        "only valid improvements are selected");
                    Check.That(b.AllocationDecision.SameQExtra == a.UpwardExtraAcceleration, "counterfactual is the exact A input");
                    IndependentClosure(l, r, b, br);
                }
                HipAllocationContracts.AssertBudget(b, br); HipAllocationContracts.AssertBudget(br, b);
                Check.That(HipAnchorTranslationPair.CheckActualPrecommit(l, b) && HipAnchorTranslationPair.CheckActualPrecommit(r, br), "actual precommit accepts both allocations");
                Check.That(b.AllocatedHipAlpha.Equals(l.DesiredAlpha + HipPointPairMobility.InverseTensor(l.ChildInertia, l.ChildPrincipal, b.NetHipTorque - l.PureTorque)),
                    "Knee receives actual allocated Hip alpha");
                var c = Completed(b, br);
                Check.That(HipActuationOwnership.TryMeasure(b, b.UpperActuatorTorque, Vector3.up, Vector3.right, c, out var ownership) &&
                    ownership.JointPairResidual.magnitude < .00001f && ownership.SystemForceResidual.magnitude < .002f,
                    "stop-time ledger uses the actual angular scale");
                var report = new HipAllocationReport(); report.Observe(true, b, br, c);
                Check.That(report.Samples == 1 && report.Invalid == 0 && report.Applied == (b.AllocationDecision.Applied ? 1 : 0), "successful allocations have valid evidence");
                l.Seed.ParentInput.UpwardRetentionEligible = r.Seed.ParentInput.UpwardRetentionEligible = false;
                Check.That(HipBilateralForceSolver.TryPlan(l, r, out var inactive, out _) && !inactive.AllocationDecision.Attempted &&
                    inactive.UpwardExtraAcceleration == 0f && inactive.UpperAngularScale == inactive.MotionScale, "ineligible B grants no trade");
            }
            Console.WriteLine("  conditional synthetic cases: accepted=" + accepted + ", executed=" + applied + ", exactSameQ=" + unchanged + ", q1=" + full + ", fallback=" + fallback + ", rejected=" + rejected);
            Check.That(accepted > 1000 && applied > 10 && unchanged > 100 && fallback > 100 && full > 10, "coverage of selected and fallback paths");
        }
        private static void IndependentClosure(HipAnchorPairInput l, HipAnchorPairInput r, HipAnchorPairReport.Command c, HipAnchorPairReport.Command d)
        {
            var p = l.Seed.ParentInput; float angular = c.UpperAngularScale;
            var reaction = -angular * l.Angular.RelativeTorque - angular * r.Angular.RelativeTorque;
            var parentAlpha = p.Response.PredictAngular(Vector3.zero, p.ReceiverTorque + reaction, p.ParentW);
            var parentA = p.Response.PredictCom(Vector3.zero, parentAlpha, p.ParentW);
            var target = p.Acceleration * c.MotionScale; target.y += c.UpwardExtraAcceleration;
            Check.That((c.CommonAcceleration - (c.Force + d.Force) / p.Response.TotalMass - target).magnitude < .001f, "system COM closure");
            foreach (bool left in new[] { true, false })
            {
                var input = left ? l : r; var cmd = left ? c : d;
                var pointA = cmd.Force / input.Seed.ChildMass + (c.Force + d.Force) / p.Response.TotalMass;
                var pureAlpha = HipPointPairMobility.InverseTensor(input.ChildInertia, input.ChildPrincipal, angular * input.PureTorque);
                var residual = pointA + Vector3.Cross(pureAlpha, input.Seed.ChildPoint - input.Seed.ChildCom) - parentA -
                    Vector3.Cross(parentAlpha, input.Seed.ParentPoint - input.Seed.ParentCom) - cmd.CommonAcceleration - input.Seed.FeedbackA;
                Check.That(residual.magnitude < .001f && cmd.ParentAngularReaction.Equals(-angular * input.Angular.RelativeTorque), "point closure and joint reaction");
            }
        }
        private static void ScaleContract()
        {
            AppliedFixture(out var l, out _, out var c, out _); var p = l.Seed.ParentInput;
            Check.That(HipSystemAllocation.AngularValid(p, c.MotionScale, c.UpperAngularScale, c.UpwardExtraAcceleration), "actual scale valid");
            foreach (float angular in new[] { float.NaN, float.PositiveInfinity, -.1f, c.MotionScale + .01f, c.MotionScale * .899f })
                Check.That(!HipSystemAllocation.AngularValid(p, c.MotionScale, angular, c.UpwardExtraAcceleration), "invalid angular scale rejected");
            p.BoundedTradeSelected = false;
            Check.That(!HipSystemAllocation.AngularValid(p, c.MotionScale, c.UpperAngularScale, c.UpwardExtraAcceleration), "A cannot exchange torque");
            p.BoundedTradeSelected = true; p.UpwardRetentionEligible = false;
            Check.That(!HipSystemAllocation.AngularValid(p, c.MotionScale, c.UpperAngularScale, c.UpwardExtraAcceleration), "ineligible B cannot exchange");
            p.UpwardRetentionSelected = false;
            Check.That(!p.Matches(p.Step, p.BodyId), "inconsistent selector receipt rejected");
        }
        private static void Interval()
        {
            Check.That(!HipBilateralForceSolver.TryBoundedTradeRange(.11f, .8f, .1f, out _), "cap below feasible lower endpoint rejected");
            Check.That(HipBilateralForceSolver.TryBoundedTradeRange(.03f, .08f, .1f, out float t) && t == .08f, "budget clips upper endpoint");
            Check.That(HipBilateralForceSolver.TryBoundedTradeRange(.03f, .8f, .1f, out t) && t == .1f, "cap clips upper endpoint");
            Check.That(!HipBilateralForceSolver.TryBoundedTradeRange(float.NaN, .8f, .1f, out _), "invalid range rejected");
        }
        private static void Precommit()
        {
            AppliedFixture(out var l, out _, out var c, out _);
            for (int fault = 0; fault < 8; fault++)
            {
                var bad = c;
                switch (fault)
                {
                    case 0: bad.UpperAngularScale = bad.MotionScale; break;
                    case 1: bad.UpperAngularScale = bad.MotionScale * .8f; break;
                    case 2: bad.NetHipTorque = bad.MotionScale * bad.PureHipTorque; break;
                    case 3: bad.ParentAngularReaction = -bad.MotionScale * bad.Angular.RelativeTorque; break;
                    case 4: bad.AllocatedHipAlpha = l.DesiredAlpha; break;
                    case 5: bad.ParentInput.BoundedTradeSelected = false; break;
                    case 6: bad.FullFeedbackFeasible = false; break;
                    case 7: bad.UpperAngularScale = float.NaN; break;
                }
                Check.That(!HipAnchorTranslationPair.CheckActualPrecommit(l, bad), "precommit rejects angular corruption " + fault);
            }
        }
        private static Vector3 Tensor(Vector3 inertia, Quaternion principal, Vector3 v)
        {
            var local = Quaternion.Inverse(principal) * v;
            return principal * new Vector3(inertia.x * local.x, inertia.y * local.y, inertia.z * local.z);
        }
        private static void WithGyroAndTransport(ref HipAnchorPairInput input)
        {
            if (input.ChildInertia.x == 1f && input.ChildInertia.y == 1f && input.ChildInertia.z == 1f)
                input.ChildInertia = new Vector3(.8f, 1.1f, .9f); // A spherical tensor has zero gyro torque.
            input.ChildW = new Vector3(.4f, -.7f, .9f);
            var gyro = Vector3.Cross(input.ChildW, Tensor(input.ChildInertia, input.ChildPrincipal, input.ChildW));
            input.PureTorque = Tensor(input.ChildInertia, input.ChildPrincipal, input.DesiredAlpha) + gyro;
            Check.That(HipAngularRequest.TryCreate(input.ChildInertia, input.ChildPrincipal, input.ChildW, input.PureTorque,
                input.DesiredAlpha, .6f * input.DesiredAlpha, .4f * input.DesiredAlpha, input.DesiredAlpha, 40f, out input.Angular),
                "mixed gyro and transport request");
        }
        private static void MixedAngularOwnership()
        {
            var rng = new Random(130); int executed = 0, nontrivialAlpha = 0;
            for (int i = 0; i < 600; i++)
            {
                Generate(rng, i, out var l, out var r);
                WithGyroAndTransport(ref l); WithGyroAndTransport(ref r);
                l.Seed.ParentInput.BoundedTradeSelected = r.Seed.ParentInput.BoundedTradeSelected = true;
                if (!HipBilateralForceSolver.TryPlan(l, r, out var a, out var b) || !a.AllocationDecision.Applied) continue;
                executed++; IndependentClosure(l, r, a, b);
                HipAllocationContracts.AssertBudget(a, b); HipAllocationContracts.AssertBudget(b, a);
                Check.That(HipAnchorTranslationPair.CheckActualPrecommit(l, a) && HipAnchorTranslationPair.CheckActualPrecommit(r, b),
                    "mixed angular command passes real precommit");
                Check.That(a.ParentAngularReaction.Equals(-a.UpperAngularScale * l.Angular.RelativeTorque),
                    "only relative control is mirrored, not transport/gyro");
                var expected = l.DesiredAlpha + HipPointPairMobility.InverseTensor(l.ChildInertia, l.ChildPrincipal,
                    a.UpperAngularScale * l.PureTorque - l.PureTorque);
                Check.That(a.AllocatedHipAlpha.Equals(expected), "downstream alpha includes actual torque change");
                if ((a.AllocatedHipAlpha - a.UpperAngularScale * l.DesiredAlpha).magnitude > .0001f) nontrivialAlpha++;
            }
            Console.WriteLine("  mixed angular cases: executed=" + executed + ", nontrivialAlpha=" + nontrivialAlpha);
            Check.That(executed > 10 && nontrivialAlpha > 0, "coverage includes trades where simply scaling alpha is wrong");
        }
        private static void Report()
        {
            AppliedFixture(out _, out _, out var l, out var r); var c = Completed(l, r);
            var report = new HipAllocationReport();
            report.Observe(false, l, r, c); Check.That(report.Samples == 0, "trace off");
            var incomplete = c; incomplete.Committed = false;
            report.Observe(true, l, r, incomplete); Check.That(report.Invalid == 1 && report.Applied == 0, "uncommitted receiver cannot claim execution");
            report.Observe(true, l, r, c); report.Observe(true, l, r, c);
            Check.That(report.Samples == 1 && report.Applied == 1 && report.Duplicate == 1, "no double counting");
            var wrong = r; wrong.UpperAngularScale = wrong.MotionScale;
            report.Observe(true, l, wrong, c); Check.That(report.Invalid == 2, "peer scale mismatch visible");
            var step = l.Step; step.Sequence += 2; step.FixedTime += 2f * step.Dt;
            l.Step = r.Step = l.ParentInput.Step = r.ParentInput.Step = c.Baseline.Step = step;
            report.Observe(true, l, r, c); Check.That(report.Gaps == 1, "clock gap counted");
            step.Sequence--;
            l.Step = r.Step = l.ParentInput.Step = r.ParentInput.Step = c.Baseline.Step = step;
            report.Observe(true, l, r, c); Check.That(report.Invalid == 3 && report.Samples == 2, "backward delivery rejected, not counted twice");
            step.Epoch++;
            l.Step = r.Step = l.ParentInput.Step = r.ParentInput.Step = c.Baseline.Step = step;
            report.Observe(true, l, r, c); Check.That(report.Invalid == 4 && report.Samples == 2, "cross-epoch delivery needs trace reset");
            report.NoteMissing(); report.Reset();
            Check.That(report.Samples == 0 && report.Applied == 0 && report.Missing == 0 &&
                !report.Details(true).Contains("hipAllocationFrame="), "reset clears counters and bounded snapshots");
        }
        private static void NoHeap()
        {
            AppliedFixture(out var l, out var r, out var a, out var b);
            var report = new HipAllocationReport(); var complete = Completed(a, b);
            for (int i = 0; i < 20; i++) { HipBilateralForceSolver.TryPlan(l, r, out _, out _); report.Observe(true, a, b, complete); }
            long before = GC.GetAllocatedBytesForCurrentThread(); bool ok = true;
            for (int i = 0; i < 200; i++)
            {
                ok &= HipBilateralForceSolver.TryPlan(l, r, out a, out b);
                report.Observe(true, a, b, complete);
            }
            long bytes = GC.GetAllocatedBytesForCurrentThread() - before;
            Check.That(ok && bytes == 0, "hot path allocates zero managed bytes; bytes=" + bytes);
        }
    }
}
