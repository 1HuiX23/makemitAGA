using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class SupportReferenceContracts
    {
        internal static void Run()
        {
            Check.Run("support reference previous-completed identity and age", Identity);
            Check.Run("landing preparation timing keeps confirmed baseline and source guards", LandingPreparation);
            Check.Run("support normal reference bounded one-sided candidate", Bounds);
            Check.Run("normal priority shares original spheres and retains tangential braking", NormalPriority);
        }
        private static PhysicsStepContext Step(int sequence) => new PhysicsStepContext
            { Epoch = 3, Sequence = sequence, SourceSequence = 7, FixedTime = sequence * 0.02f, Dt = 0.02f, UnscaledTime = sequence * 0.02f };
        private static void Identity()
        {
            var left = new PreviousFootSupport { Step = Step(10), SupportId = 24, StrongStance = true, SourceAge = 0.01f };
            var right = left;
            Check.That(SupportReferenceContext.Create(Step(11), left, right, 24).StanceMask == 3, "both preceding stance witnesses");
            right.StrongStance = false;
            Check.That(SupportReferenceContext.Create(Step(11), left, right, 24).StanceMask == 1, "single stance is sufficient");
            var mismatched = right; mismatched.Step.SourceSequence--;
            Check.That(!SupportReferenceContext.Create(Step(11), left, mismatched, 24).Matches(Step(11)), "two sides must share the full previous identity");
            var unpublished = left; unpublished.Step.SourceSequence = 0;
            Check.That(!unpublished.Eligible(Step(11), 24), "no stance before a source identity is published");
            Check.That(!SupportReferenceContext.Create(Step(12), left, right, 24).Matches(Step(12)), "skipped step cannot reuse stance");
            Check.That(!SupportReferenceContext.Create(Step(10), left, right, 24).Matches(Step(10)), "same-step witness is not previous");
            Check.That(!SupportReferenceContext.Create(Step(11), left, right, 25).Matches(Step(11)), "support identity change removes eligibility");
            var changed = Step(11); changed.Epoch++;
            Check.That(!left.Eligible(changed, 24), "rebind removes witness");
            changed = Step(11); changed.UnscaledTime += 0.2f;
            Check.That(!left.Eligible(changed, 24), "wall-time hitch cannot retain old stance");
            left.SourceAge = 0.09f;
            Check.That(!left.Eligible(Step(11), 24), "age includes following physics interval");
            left.SourceAge = float.NaN;
            Check.That(!left.Eligible(Step(11), 24), "unknown age rejected");
            left.SourceAge = 0f; changed = Step(11); changed.FixedTime = left.Step.FixedTime; changed.UnscaledTime = left.Step.UnscaledTime;
            Check.That(left.Eligible(changed, 24), "equal rounded timestamps with advancing sequence allowed");
        }
        private static void LandingPreparation()
        {
            var left = new PreviousFootSupport { Step = Step(10), SupportId = 24, StrongLandingApproach = true, SourceAge = 0.01f };
            var right = new PreviousFootSupport { Step = Step(10), SupportId = 24, SourceAge = 0.01f };
            var next = Step(11);
            Check.That(!SupportReferenceContext.Create(next, left, right, 24).Matches(next), "confirmed baseline does not use an approach witness");
            var early = SupportReferenceContext.Create(next, left, right, 24, true);
            Check.That(early.Matches(next) && early.StanceMask == 1 && early.ApproachMask == 1 && early.ApproachOnly,
                "completed strong approach prepares the following reference without waiting for confirmation");
            Check.That(!early.Matches(Step(10)) && !early.Matches(Step(12)), "approach never becomes same-step or persistent authority");
            right.StrongStance = true;
            var baseline = SupportReferenceContext.Create(next, left, right, 24);
            early = SupportReferenceContext.Create(next, left, right, 24, true);
            Check.That(baseline.StanceMask == 2 && baseline.ApproachMask == 0, "confirmed other side already enables baseline");
            Check.That(early.StanceMask == 3 && early.ApproachMask == 1 && !early.ApproachOnly, "mixed witness is distinct from new approach-only exposure");
            left.StrongLandingApproach = false; left.StrongStance = true;
            early = SupportReferenceContext.Create(next, left, right, 24, true);
            Check.That(early.StanceMask == 3 && early.ApproachMask == 0 && !early.ApproachOnly, "confirmed phase converges to the existing policy");
            left.StrongLandingApproach = true; right.StrongStance = false;
            Check.That(!left.Eligible(next, 24, true), "contradictory phase evidence is rejected");
            left.StrongStance = false; left.SourceAge = 0.09f;
            Check.That(!left.Eligible(next, 24, true), "early preparation retains source-age bound");
            left.SourceAge = 0.01f;
            Check.That(!left.Eligible(next, 25, true), "early preparation cannot cross support identity");
            next.UnscaledTime += 0.2f;
            Check.That(!left.Eligible(next, 24, true), "early preparation cannot bridge a wall-time hitch");
            next = Step(11); right.Step = Step(9);
            Check.That(!SupportReferenceContext.Create(next, left, right, 24, true).Matches(next), "both controllers still must have completed the same step");
            early.ApproachMask = 4;
            Check.That(!early.Matches(next), "invalid side mask rejected");
            early.StanceMask = 1; early.ApproachMask = 2;
            Check.That(!early.Matches(next), "approach must belong to a selected side");
        }
        private static void NormalPriority()
        {
            var position = new Vector3(0f, 1f, 0f);
            var lateral = new Vector3(0.12f, 0f, 0f);
            var result = SupportNormalReference.AllocateNormalPriority(position, Vector3.zero, lateral,
                1.01f, 0.02f, 3f, 6f, out bool braking);
            Check.Near(result.y, 0.12f, 1e-6f, "normal can use a formerly saturated tangential acceleration budget");
            Check.Near(result.x, 0f, 1e-5f, "no simultaneous full allocation to both axes");
            Check.That(!braking && position.y + result.y * 0.02f < 1.01f, "unreachable height remains a deficit");
            result = SupportNormalReference.AllocateNormalPriority(position, Vector3.zero, lateral,
                1.001f, 0.02f, 3f, 6f, out braking);
            Check.Near(position.y + result.y * 0.02f, 1.001f, 1e-6f, "reachable height need does not consume extra normal budget");
            Check.That(result.x > 0.1f && result.x < lateral.x, "retain remaining horizontal acceleration");

            var previous = new Vector3(0.5f, 0f, 0f);
            var decelerating = new Vector3(0.38f, 0f, 0f);
            var baseline = SupportNormalReference.RaiseCandidate(position, previous, decelerating, 1.1f, 0.02f, 3f, 6f);
            result = SupportNormalReference.AllocateNormalPriority(position, previous, decelerating, 1.1f, 0.02f, 3f, 6f, out braking);
            Check.That(braking && result.x == baseline.x && result.y == baseline.y && result.z == baseline.z, "do not trade away nominal tangential braking");
            result = SupportNormalReference.AllocateNormalPriority(position, previous, previous, 1.1f, 0.02f, 3f, 6f, out braking);
            Check.That(result.x == previous.x && !braking, "existing horizontal velocity is not scaled");
            result = SupportNormalReference.AllocateNormalPriority(position, Vector3.zero, lateral, 0.9f, 0.02f, 3f, 6f, out braking);
            Check.That(result.x == lateral.x && result.y == lateral.y && !braking, "no intervention without a downward reference deficit");

            previous = new Vector3(2.99f, 0.2f, 0f);
            result = SupportNormalReference.AllocateNormalPriority(position, previous, new Vector3(2.993f, 0.2f, 0f),
                1.1f, 0.02f, 3f, 6f, out braking);
            Check.That(result.magnitude <= 3.00001f && (result - previous).magnitude <= 0.12001f, "speed boundary and acceleration boundary both apply");
            Check.That(result.x >= previous.x - 1e-6f && result.y > 0.2f, "priority does not invent horizontal reversal to make room");
            Check.Throws<InvalidOperationException>(() => SupportNormalReference.AllocateNormalPriority(position, previous, lateral,
                float.NaN, 0.02f, 3f, 6f, out _), "unknown height rejected");
            Check.Throws<InvalidOperationException>(() => SupportNormalReference.AllocateNormalPriority(position, previous, lateral,
                1f, 0f, 3f, 6f, out _), "invalid timestep rejected");

            var rng = new Random(4122);
            for (int i = 0; i < 512; i++)
            {
                previous = Vector3.ClampMagnitude(new Vector3(6f * ((float)rng.NextDouble() - 0.5f),
                    6f * ((float)rng.NextDouble() - 0.5f), 6f * ((float)rng.NextDouble() - 0.5f)), 3f);
                var increment = Vector3.ClampMagnitude(new Vector3((float)rng.NextDouble() - 0.5f,
                    (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f), 0.12f);
                var candidate = Vector3.ClampMagnitude(previous + increment, 3f);
                float floor = 1f + 0.1f * ((float)rng.NextDouble() - 0.2f);
                baseline = SupportNormalReference.RaiseCandidate(position, previous, candidate, floor, 0.02f, 3f, 6f);
                result = SupportNormalReference.AllocateNormalPriority(position, previous, candidate, floor, 0.02f, 3f, 6f, out braking);
                Check.That(result.magnitude <= 3.00001f && (result - previous).magnitude <= 0.12001f, "same two vector spheres");
                Check.That(result.y >= baseline.y, "normal priority never reduces the remainder candidate Y");
                var nominalXZ = new Vector3(candidate.x - previous.x, 0f, candidate.z - previous.z);
                var actualXZ = new Vector3(result.x - previous.x, 0f, result.z - previous.z);
                Check.That(actualXZ.magnitude <= nominalXZ.magnitude + 1e-6f &&
                    Vector3.Cross(actualXZ, nominalXZ).magnitude < 1e-6f && Vector3.Dot(actualXZ, nominalXZ) >= -1e-7f,
                    "only reduce the nominal horizontal increment without redirecting it");
                if (Vector3.Dot(new Vector3(previous.x, 0f, previous.z), nominalXZ) < 0f)
                    Check.That(result.x == baseline.x && result.y == baseline.y && result.z == baseline.z, "braking remains baseline for every sampled orientation");
            }
        }
        private static void Bounds()
        {
            var p = new Vector3(0f, 1f, 0f);
            Vector3 v = Vector3.zero;
            var raised = SupportNormalReference.RaiseCandidate(p, v, v, 1.01f, 0.02f, 3f, 6f);
            Check.Near(raised.y, 0.12f, 1e-6f, "large target step spends existing acceleration budget only");
            Check.That(p.y + raised.y * 0.02f < 1.01f, "unreachable floor retains deficit instead of snapping");
            Check.Near(SupportNormalReference.RaiseCandidate(p, v, v, 0.9f, 0.02f, 3f, 6f).y, 0f, 0f, "no downward correction");
            var lateral = new Vector3(0.12f, 0f, 0f);
            Check.Near(SupportNormalReference.RaiseCandidate(p, v, lateral, 1.1f, 0.02f, 3f, 6f).y, 0f, 1e-6f, "saturated tangent consumes all acceleration budget");
            var moving = new Vector3(3f, 0f, 0f);
            Check.Near(SupportNormalReference.RaiseCandidate(p, moving, moving, 2f, 0.02f, 3f, 6f).y, 0f, 1e-6f, "total speed cap preserved");
            Check.Throws<InvalidOperationException>(() => SupportNormalReference.RaiseCandidate(p, v, v, float.NaN, 0.02f, 3f, 6f), "invalid floor rejected");
            Check.Throws<InvalidOperationException>(() => SupportNormalReference.RaiseCandidate(p, v, v, 1f, 0f, 3f, 6f), "invalid dt rejected");
            var rng = new Random(4120);
            for (int i = 0; i < 256; i++)
            {
                var previous = new Vector3((float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f, (float)rng.NextDouble() - 0.5f);
                var candidate = new Vector3(previous.x + 0.08f * ((float)rng.NextDouble() - 0.5f),
                    previous.y + 0.08f * ((float)rng.NextDouble() - 0.5f), previous.z + 0.08f * ((float)rng.NextDouble() - 0.5f));
                var result = SupportNormalReference.RaiseCandidate(p, previous, candidate, 1f + (float)rng.NextDouble() * 0.1f, 0.02f, 3f, 6f);
                float dx = result.x - previous.x, dy = result.y - previous.y, dz = result.z - previous.z;
                Check.That(result.x == candidate.x && result.z == candidate.z && result.y >= candidate.y, "only raises normal candidate");
                Check.That(result.magnitude <= 3.00001f && MathF.Sqrt(dx * dx + dy * dy + dz * dz) <= 0.12001f, "vector speed and acceleration limits");
            }
        }
    }
}
