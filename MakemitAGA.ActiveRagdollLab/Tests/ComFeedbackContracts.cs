using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class ComFeedbackContracts
    {
        internal static void Run()
        {
            Check.Run("COM feedback normal-only change and damping ratio", NormalOnly);
            Check.Run("COM reference sensitivity matches actual semi-implicit feedback", Sensitivity);
            Check.Run("COM planner retains spheres and physical braking with anisotropic gains", Planner);
            Check.Run("normal response joins solved input to exact next context", ResponseIdentity);
        }
        private static void Near(Vector3 a, Vector3 b, float tolerance, string label)
        { Check.Near(a.x, b.x, tolerance, label + " X"); Check.Near(a.y, b.y, tolerance, label + " Y"); Check.Near(a.z, b.z, tolerance, label + " Z"); }
        private static void NormalOnly()
        {
            var error = new Vector3(0.02f, -0.03f, 0.04f);
            var velocity = new Vector3(-0.2f, 0.1f, 0.3f);
            Vector3 a = PelvisComFeedback.Evaluate(error, velocity, false);
            Vector3 b = PelvisComFeedback.Evaluate(error, velocity, true);
            Near(a, new Vector3(-1.4f, 0.3f, 3.5f), 1e-6f, "frozen baseline fixture");
            Check.That(a.x == b.x && a.z == b.z && a.y != b.y, "only the normal feedback changes");
            Check.Near(b.y, 0.0727922f, 1e-6f, "normal P and damping act in both directions");
            Check.Near(PelvisComFeedback.NormalVelocityGain / MathF.Sqrt(PelvisComFeedback.NormalPositionGain),
                PelvisComFeedback.VelocityGain / MathF.Sqrt(PelvisComFeedback.PositionGain), 1e-6f, "nominal damping ratio retained");
            Near(PelvisComFeedback.Evaluate(Vector3.zero, Vector3.zero, true), Vector3.zero, 0f, "no unconditional lift");
        }
        private static void Sensitivity()
        {
            foreach (float dt in new[] { 1f / 240f, 1f / 60f, 0.02f, 1f / 30f, 0.1f })
                foreach (bool enhanced in new[] { false, true })
                {
                    var deficit = new Vector3(2f, -3f, 4f);
                    Vector3 referenceA = PelvisComFeedback.ReferenceDeficit(deficit, dt, enhanced);
                    // Independent finite change: integrate reference velocity then position,
                    // reevaluate the production feedback and include reference AFF once.
                    Vector3 response = referenceA + PelvisComFeedback.Evaluate(referenceA * (dt * dt), referenceA * dt, enhanced);
                    Near(response, deficit, 2e-6f, "deficit back-map predicts the actual control change");
                }
        }
        private static PelvisComReferencePlanner.ComPlan Plan(Vector3 velocity, Vector3 goal, Vector3 deficit, Vector3 bodyV, bool enhanced, float dt = 0.02f) =>
            PelvisComReferencePlanner.Build(Vector3.zero, velocity, goal, Vector3.zero, dt, true, true,
                deficit, Vector3.zero, bodyV, true, default, false, 0f, true, enhanced);
        private static void Planner()
        {
            foreach (bool enhanced in new[] { false, true })
            {
                var brake = Plan(new Vector3(0f, 0.2f, 0f), new Vector3(0f, 1f, 0f),
                    new Vector3(0f, 2f, 0f), new Vector3(0f, -0.2f, 0f), enhanced);
                Check.That(brake.PhysicalBrakeProtected && brake.RemovedAcceleration == 0f,
                    "opposing real COM motion protects the reference braking request");
                var random = new Random(124);
                for (int i = 0; i < 300; i++)
                {
                    float N() => (float)(random.NextDouble() * 2d - 1d);
                    Vector3 velocity = Vector3.ClampMagnitude(new Vector3(N(), N(), N()) * 3f, 3f);
                    var p = Plan(velocity, new Vector3(N(), N(), N()), new Vector3(N(), N(), N()) * 8f,
                        new Vector3(N(), N(), N()), enhanced);
                    Check.That(p.Velocity.magnitude <= 3.00001f && p.Acceleration.magnitude <= 6.001f,
                        "original vector speed and acceleration bounds remain");
                    Check.That(p.EnhancedNormalFeedback == enhanced && p.RemovedAcceleration >= 0f,
                        "gain identity and one-sided removal retained");
                }
            }
            Check.Throws<InvalidOperationException>(() => Plan(Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero, true, 0f), "zero solve dt rejected");
            Check.Throws<InvalidOperationException>(() => Plan(Vector3.zero, Vector3.zero, new Vector3(float.NaN, 0f, 0f), Vector3.zero, true), "invalid prior input rejected");
        }
        private static PhysicsStepContext Step(int sequence) => new PhysicsStepContext
            { Epoch = 3, Sequence = sequence, SourceSequence = 7, FixedTime = sequence * 0.02f, Dt = 0.02f, UnscaledTime = sequence * 0.02f };
        private static void ResponseIdentity()
        {
            var s = new PelvisNormalResponseEvidence { Valid = true, Input = Step(10), Settled = Step(11), Fixed = 5,
                PelvisId = 11, ChestId = 12, Q = 0.5f, SourcePhase = "ACCEL", Observed = 1f, Request = 2f, Allocated = 1f,
                Enhanced = true, SourceFresh = true, LinearClamped = true, SourceAge = .01f,
                ErrorY = .002f, VelocityErrorY = .03f, P = .2f, D = .3f, Feedforward = 1.5f,
                Actuator = 2f, PreV = .1f, PostV = .12f };
            Check.That(s.Matches(Step(11)), "previous input -> current settled sample");
            Check.That(!s.Matches(Step(10)) && !s.Matches(Step(12)), "not current unresolved or later reused input");
            var next = Step(11); next.SourceSequence++;
            Check.That(!s.Matches(next), "full context includes the held source identity");
            next = Step(11); next.Epoch++;
            Check.That(!s.Matches(next), "new binding cannot reuse evidence");
            next = Step(11); next.Dt = 0.01f;
            Check.That(!s.Matches(next), "new solve duration cannot relabel old response");
            var invalid = s; invalid.Settled = Step(12);
            Check.That(!invalid.Matches(Step(12)), "response gap is missing evidence");
            invalid = s; invalid.Q = float.NaN;
            Check.That(!invalid.Matches(Step(11)), "nonfinite response rejected");
            invalid = s; invalid.ChestId = invalid.PelvisId;
            Check.That(!invalid.Matches(Step(11)), "distinct physical bodies required");
            Check.That(default(PelvisNormalResponseEvidence).Text() == "receiverNormalV041240=NA", "missing receipt explicitly labelled");
            Check.That(s.Text().Contains("/normalEnhanced=True") && s.Text().Contains("/normalQ-linearClamped=0.5|True"),
                "populated optional response evidence reaches the report");
            invalid = s; invalid.SourceAge = -.01f;
            Check.That(!invalid.Matches(Step(11)), "negative observation age rejected");
            var retained = s;
            retained.UpwardSelected = retained.UpwardEligible = true;
            retained.UpwardExtra = .4f; retained.Allocated = 1.4f;
            Check.That(retained.Matches(Step(11)) && retained.Text().Contains("/normalUpwardRetention(selected-eligible-extraA)=True|True|"),
                "retained evidence reports the actual target, not q alone");
            invalid = retained; invalid.UpwardExtra = 0f;
            Check.That(!invalid.Matches(Step(11)), "lost extra cannot silently label a q-only target");
            invalid = retained; invalid.UpwardExtra = float.NaN;
            Check.That(!invalid.Matches(Step(11)), "nonfinite extra rejected");
            invalid = retained; invalid.UpwardExtra = -1f;
            Check.That(!invalid.Matches(Step(11)), "negative retention rejected");
            invalid = retained; invalid.UpwardSelected = false;
            Check.That(!invalid.Matches(Step(11)), "unselected extra rejected");
            invalid = retained; invalid.UpwardEligible = false;
            Check.That(!invalid.Matches(Step(11)), "ineligible extra rejected");
            invalid = retained; invalid.UpwardExtra = 1.1f; invalid.Allocated = 2.1f;
            Check.That(!invalid.Matches(Step(11)), "cannot exceed original upward request");
            invalid = s; invalid.PreV = float.PositiveInfinity;
            Check.That(!invalid.Matches(Step(11)), "nonfinite pre-solve velocity rejected");
        }
    }
}
