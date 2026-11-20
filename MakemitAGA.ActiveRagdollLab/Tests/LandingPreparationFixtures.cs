using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class LandingPreparationFixtures
    {
        internal struct Fixture
        {
            internal KneeFootGroundPacket Packet;
            internal HipNormalReference Hip;
            internal KneeReferenceState Knee;
        }
        internal static Fixture Make(Random random, int hz = 60)
        {
            float N() => (float)random.NextDouble();
            float hipAngle = 20f + N() * 30f, kneeAngle = -35f - N() * 45f;
            var step = new PhysicsStepContext { Epoch = 8, Sequence = 101, SourceSequence = 30, FixedTime = 2f, UnscaledTime = 4f, Dt = 1f / 60f };
            var source = new SourceStepContext { Epoch = 8, Sequence = 30, Time = 4f, Dt = 1f / hz };
            var hip = new HipNormalReference();
            hip.Reset(true, Quaternion.AngleAxis(hipAngle, Vector3.right), new Vector3(N() * 2f - 1f, 0f, 0f), false);
            Check.That(hip.Step(false, step, source, true, Quaternion.AngleAxis(hipAngle + N() * 16f - 8f, Vector3.right),
                new Vector3(N() * 2f - 1f, 0f, 0f), Quaternion.identity, Vector3.zero, 0f, .1f, 40f / 12f, 20f, 5f, true), "real Hip planner fixture");
            var k = new KneeRelativeReference();
            k.Reset(true, Quaternion.AngleAxis(kneeAngle, Vector3.right), new Vector3(N() * 2f - 1f, 0f, 0f));
            var goal = Quaternion.AngleAxis(kneeAngle + N() * 16f - 8f, Vector3.right);
            Check.That(k.Step(false, step, source, goal, new Vector3(N() * 2f - 1f, 0f, 0f), 0f, .1f, 40f / 9f, 20f, 5f), "real Knee planner fixture");
            var foot = Quaternion.AngleAxis(-hipAngle - kneeAngle, Vector3.right);
            var publishedFoot = foot * Quaternion.AngleAxis(N() * 8f - 4f, Vector3.right);
            var g = new KneeGroundTarget.Geometry { Hip = new Vector3(0f, .9f, 0f), Upper = Quaternion.AngleAxis(hipAngle, Vector3.right),
                UpperSpan = new Vector3(0f, -.45f, 0f), LowerSpan = new Vector3(0f, -.45f, 0f),
                FootRelative = foot, FootBindRelative = foot, Center = new Vector3(0f, -.1f, .025f), Half = new Vector3(.05f, .025f, .08f),
                FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f) };
            g.Hip.y -= Oracle(g, hip.Rotation, k.Current.R, publishedFoot) + .002f + N() * .020f;
            var completed = step; completed.Sequence--; completed.SourceSequence--; completed.FixedTime -= step.Dt; completed.UnscaledTime -= .01f;
            var input = new KneeFootGroundInput { Enabled = true, Apply = true, Source = source, CompletedStep = completed,
                ParentId = 11, UpperId = 12, LowerId = 13, Axis = Vector3.right, RawKnee = goal,
                Hip = g.Hip, Upper = g.Upper, UpperSpan = g.UpperSpan };
            var packet = new KneeFootGroundPacket { Published = true, SupportId = 21, Input = input, Geometry = g,
                PreparationSupport = new PreviousFootSupport { Step = completed, SupportId = 21, StrongLandingApproach = true, SourceAge = .001f },
                Sample = new KneeGroundTarget.Sample { Valid = true, Source = source, GeometryStep = completed, Raw = goal, Goal = goal,
                    PlannedFoot = publishedFoot, Foot = new FootGroundReference.GroundSample { Valid = true, Feasible = true, Goal = foot } } };
            return new Fixture { Packet = packet, Hip = hip, Knee = k.Current };
        }
        private static float Oracle(KneeGroundTarget.Geometry g, Quaternion hip, Quaternion knee, Quaternion foot)
        {
            Vector3 ankle = g.Hip + hip * g.UpperSpan + hip * knee * g.LowerSpan;
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                min = MathF.Min(min, (ankle + (hip * knee * foot) * (g.Center + new Vector3(x * g.Half.x, y * g.Half.y, z * g.Half.z))).y - g.FloorCenter.y - g.FloorHalf.y);
            return min;
        }
    }
}
