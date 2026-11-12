using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class HipAllocationContracts
    {
        internal static void Run()
        {
            Check.Run("upward allocation requires current reference and completed support", Eligibility);
            Check.Run("bilateral allocator retains upward input without changing q or other axes", Sweep);
            Check.Run("bilateral fallback and ineligible commands retain the original result", Ineligible);
        }
        private static void Eligibility()
        {
            var step = PelvisDynamicReceiverIsolation.LoggedStep();
            var reference = new PelvisComReferencePlanner.ComPlan { ArrivalEnabled = true,
                SupportInput = new SupportReferenceContext { ForStep = step,
                    ObservedStep = PelvisDynamicReceiverIsolation.LoggedStep(-1), SupportId = 301, StanceMask = 3 } };
            Check.That(HipSystemAllocation.Eligible(true, step, step, reference, .01f, Vector3.up), "qualified current support/error/request");
            for (int scenario = 0; scenario < 11; scenario++)
            {
                var r = reference; var rs = step; float e = .01f; var a = Vector3.up; bool selected = true;
                switch (scenario)
                {
                    case 0: selected = false; break;
                    case 1: rs.Sequence--; break;
                    case 2: r.ArrivalEnabled = false; break;
                    case 3: r.SupportInput = default; break;
                    case 4: r.SupportInput.ObservedStep.Epoch++; break;
                    case 5: r.SupportInput.ObservedStep.Sequence--; break;
                    case 6: r.SupportInput.ForStep.SourceSequence--; break;
                    case 7: e = 0f; break;
                    case 8: e = float.NaN; break;
                    case 9: a.y = -1f; break;
                    case 10: a.x = float.PositiveInfinity; break;
                }
                Check.That(!HipSystemAllocation.Eligible(selected, step, rs, r, e, a), "eligibility fails closed " + scenario);
            }
            reference.SupportInput.ApproachMask = 3;
            Check.That(HipSystemAllocation.Eligible(true, step, step, reference, .001f, Vector3.up), "confirmed approach remains witness, not contact authority");
        }

        internal static HipAnchorPairInput Input(PelvisHipCommandFrame parent, bool left, Vector3 pure,
            Vector3 feedback, Vector3 childLever, Vector3 parentLever, float mass = 1f,
            Vector3? inertiaOverride = null, Quaternion? principalOverride = null)
        {
            var inertia = inertiaOverride ?? (Vector3.right + Vector3.up + Vector3.forward);
            var principal = principalOverride ?? Quaternion.identity;
            Vector3 alpha = HipPointPairMobility.InverseTensor(inertia, principal, pure);
            if (alpha.magnitude > 39f) { pure *= 39f / alpha.magnitude; alpha = HipPointPairMobility.InverseTensor(inertia, principal, pure); }
            Check.That(HipAngularRequest.TryCreate(inertia, principal, Vector3.zero, pure, alpha,
                alpha, Vector3.zero, alpha, 40f, out var angular), "physical angular fixture");
            return new HipAnchorPairInput { Valid = true, TorqueRequested = true, ChildInertia = inertia,
                ChildPrincipal = principal, ChildComV = Vector3.zero, ChildW = Vector3.zero,
                PureTorque = pure, DesiredAlpha = alpha, Angular = angular,
                Seed = new HipAnchorPairReport.Command { Step = parent.Step, Fixed = 11, ChildId = left ? 201 : 202,
                    ParentId = parent.BodyId, ParentInput = parent, ChildMass = mass, ParentMass = parent.Response.PelvisMass,
                    ChildPoint = childLever, ParentPoint = parentLever, FeedbackA = feedback,
                    VelocityError = new Vector3(.03f, -.01f, .02f) } };
        }
        private static void Sweep()
        {
            var random = new Random(127); int raised = 0, feasible = 0, limited = 0, fallback = 0;
            float peak = 0f;
            for (int i = 0; i < 1200; i++)
            {
                var parent = PelvisDynamicReceiverIsolation.Baseline(PelvisDynamicReceiverIsolation.LoggedStep());
                parent.Acceleration = new Vector3(R(random, 5f), .1f + (float)random.NextDouble() * 8f, R(random, 5f));
                parent.ReceiverTorque = new Vector3(R(random, .5f), R(random, .5f), R(random, .5f));
                var simpleResponse = parent.Response;
                Check.That(PelvisChestPointResponse.TryCreate(2f + (float)random.NextDouble() * 15f,
                    3f + (float)random.NextDouble() * 20f, new Vector3(.2f, .4f, .3f),
                    Quaternion.AngleAxis(R(random, 180f), new Vector3(1f, 2f, 3f)),
                    new Vector3(.03f, .3f, -.05f), out parent.Response), "varied coupled parent fixture");
                if ((i & 1) == 0) parent.Response = simpleResponse;
                Vector3 leverL = new Vector3(R(random, .15f), .2f, R(random, .1f));
                Vector3 leverR = new Vector3(R(random, .15f), .2f, R(random, .1f));
                var l = Input(parent, true, RandomV(random, 8f), RandomV(random, 25f), leverL, new Vector3(-.1f, -.05f, .03f),
                    (i & 1) == 0 ? 1f : 1f + (float)random.NextDouble() * 24f,
                    (i & 1) == 0 ? new Vector3(1f, 1f, 1f) : new Vector3(.15f, .08f, .2f), Quaternion.AngleAxis(R(random, 180f), Vector3.forward));
                var r = Input(parent, false, RandomV(random, 8f), RandomV(random, 25f), leverR, new Vector3(.1f, -.05f, .03f),
                    (i & 1) == 0 ? 1f : 1f + (float)random.NextDouble() * 24f,
                    (i & 1) == 0 ? new Vector3(1f, 1f, 1f) : new Vector3(.2f, .12f, .15f), Quaternion.AngleAxis(R(random, 180f), Vector3.right));
                bool okA = HipBilateralForceSolver.TryPlan(l, r, out var a, out var ar);
                parent.UpwardRetentionSelected = parent.UpwardRetentionEligible = true;
                l.Seed.ParentInput = r.Seed.ParentInput = parent;
                bool okB = HipBilateralForceSolver.TryPlan(l, r, out var b, out var br);
                Check.That(okB == okA, "retention never turns a uniform success into a plan failure");
                if (!okA) continue;
                feasible++;
                Check.That(a.MotionScale == b.MotionScale && a.FullFeedbackFeasible == b.FullFeedbackFeasible && a.Scale == b.Scale,
                    "original motion and feedback allocation remain exact");
                var ta = HipSystemAllocation.Target(a.ParentInput, a.MotionScale, 0f);
                var tb = HipSystemAllocation.Target(b.ParentInput, b.MotionScale, b.UpwardExtraAcceleration);
                Check.That(ta.x == tb.x && ta.z == tb.z && tb.y >= ta.y && tb.y <= parent.Acceleration.y + 1e-6f,
                    "only retain the upward request, never exceed it");
                Check.That(PelvisChestPointResponse.Same(a.NetHipTorque, b.NetHipTorque) &&
                    PelvisChestPointResponse.Same(a.BilateralAngularReaction, b.BilateralAngularReaction), "net Upper and relative reaction unchanged");
                AssertBudget(b, br); AssertBudget(br, b);
                Check.That(HipAnchorTranslationPair.CheckActualPrecommit(l, b) && HipAnchorTranslationPair.CheckActualPrecommit(r, br),
                    "actual bilateral precommit accepts the new receipt");
                if (b.MotionScale < .999999f) limited++;
                if (!b.FullFeedbackFeasible) { fallback++; Check.That(b.UpwardExtraAcceleration == 0f, "fallback grants no retention"); }
                if (b.UpwardExtraAcceleration > 0f) { raised++; peak = MathF.Max(peak, b.UpwardExtraAcceleration); }
                else Check.That(PelvisChestPointResponse.Same(a.Force, b.Force) &&
                    PelvisChestPointResponse.Same(a.CommonAcceleration, b.CommonAcceleration), "unused branch keeps exact original force arithmetic");
            }
            Check.That(feasible > 500 && limited > 100 && raised > 30 && fallback > 0 && peak > .1f,
                "sweep coverage: accepted=" + feasible + ", limited=" + limited + ", raised=" + raised + ", fallback=" + fallback + ", peak=" + peak);
            Console.WriteLine("  allocator synthetic cases: accepted=" + feasible + ", qLimited=" + limited +
                ", retained=" + raised + ", fallback=" + fallback + ", peakExtraY=" + peak);
        }
        internal static void AssertBudget(HipAnchorPairReport.Command c, HipAnchorPairReport.Command peer)
        {
            Check.That(c.UpwardExtraAcceleration == peer.UpwardExtraAcceleration &&
                HipSystemAllocation.Matches(c.ParentInput, c.MotionScale, c.UpwardExtraAcceleration,
                    c.CommonAcceleration, -c.Force - peer.Force), "same bilateral allocation and system COM closure");
            Check.That(c.CommonAcceleration.magnitude <= 12.0001f && c.AppliedA.magnitude <= 60.0001f &&
                c.UpperCorrectionTorque.magnitude <= 8.0001f && c.UpperActuatorTorque.magnitude <= c.UpperActuatorLimitNm + .0001f,
                "common, point, correction and actuator budgets unchanged");
            Check.That(PelvisHipAttitudeCommand.TryPlanWithReaction(c.ParentInput, -c.Force - peer.Force,
                c.ParentTorque + peer.ParentTorque, c.BilateralAngularReaction, out var receipt) && !receipt.Clamped &&
                receipt.RequestedTorque.magnitude <= 8f && receipt.ExternalRequestedTorque.magnitude <= 8f,
                "external and total receiver torque independently bounded");
            Check.That(!c.FullFeedbackFeasible || c.FeedbackResidualA.magnitude <= .001f * (1f + c.FeedbackA.magnitude),
                "full Hip feedback preserved");
            Check.That(c.BilateralDampingPower <= .0001f, "damping cannot inject positive energy");
        }
        private static void Ineligible()
        {
            foreach (float y in new[] { -3f, 0f, 4f })
            {
                var p = PelvisDynamicReceiverIsolation.Baseline(PelvisDynamicReceiverIsolation.LoggedStep());
                p.Acceleration = new Vector3(2f, y, 1f);
                var l = Input(p, true, 6f * Vector3.up, Vector3.zero, .2f * Vector3.up, -.1f * Vector3.right);
                var r = Input(p, false, 6f * Vector3.up, Vector3.zero, .2f * Vector3.up, .1f * Vector3.right);
                Check.That(HipBilateralForceSolver.TryPlan(l, r, out var a, out _), "baseline fixture");
                p.UpwardRetentionSelected = true; // no qualified support/error
                l.Seed.ParentInput = r.Seed.ParentInput = p;
                Check.That(HipBilateralForceSolver.TryPlan(l, r, out var b, out _) && b.UpwardExtraAcceleration == 0f &&
                    a.MotionScale == b.MotionScale && PelvisChestPointResponse.Same(a.Force, b.Force) &&
                    PelvisChestPointResponse.Same(a.CommonAcceleration, b.CommonAcceleration), "selection alone changes no force");
            }
        }
        private static float R(Random r, float span) => ((float)r.NextDouble() * 2f - 1f) * span;
        private static Vector3 RandomV(Random r, float span) => new Vector3(R(r, span), R(r, span), R(r, span));
    }
}
