using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class KneeGroundTrajectoryContracts
    {
        internal struct Fixture
        {
            internal KneeGroundTarget.Geometry Geometry;
            internal Quaternion Previous, Goal, Foot;
            internal Vector3 W, SourceW;
            internal KneeReferenceState Baseline;
        }
        internal static readonly PhysicsStepContext Step = new PhysicsStepContext
            { Epoch = 7, Sequence = 101, SourceSequence = 50, FixedTime = 2f, UnscaledTime = 4f, Dt = 1f / 60f };
        internal static readonly SourceStepContext Source = new SourceStepContext
            { Epoch = 7, Sequence = 50, Time = 4f, Dt = 1f / 40f };
        internal static void Run()
        {
            Check.Run("ground trajectory chooses only original-budget continuous improving steps", GeometryAndBudgets);
            Check.Run("ground trajectory fallback, target and calibration branch boundaries", Fallbacks);
            Check.Run("ground trajectory respects oblique local axes and world yaw", CoordinateFrames);
            Check.Run("ground trajectory preserves braking margin under mixed angular velocity", Braking);
            Check.Run("Knee planner read-only mode and selected mode retain coherent R/W/A and trace reset", Planner);
            Check.Run("ground packet stale, held and foreign identities cannot shape Knee", PacketClocks);
            Check.Run("ground shaping at 20/40/60 Hz retains Fixed closure and bounded state", SourceRates);
            Check.Run("ground trajectory does not allocate on the managed hot path", Allocation);
            SingleLegShadowChain.RunGroundTrajectoryEntryContracts();
        }
        internal static KneeFootGroundPacket Packet(Fixture f, PhysicsStepContext step, SourceStepContext source)
        {
            var completed = step; completed.Sequence--; completed.SourceSequence = source.Sequence - 1;
            completed.FixedTime -= step.Dt; completed.UnscaledTime = source.Time - .01f;
            var input = new KneeFootGroundInput { Enabled = true, Apply = true, Source = source, CompletedStep = completed,
                ParentId = 11, UpperId = 12, LowerId = 13, Axis = Vector3.right, RawKnee = f.Goal,
                Hip = f.Geometry.Hip, Upper = f.Geometry.Upper, UpperSpan = f.Geometry.UpperSpan };
            return new KneeFootGroundPacket { Published = true, SupportId = 21, Input = input, Geometry = f.Geometry,
                Sample = new KneeGroundTarget.Sample { Valid = true, Source = source, GeometryStep = completed,
                    Raw = f.Goal, Goal = f.Goal, PlannedFoot = f.Foot } };
        }
        internal static KneeReferenceState Run(Fixture f, KneeFootGroundPacket packet, bool selected, bool trace = false)
        {
            var p = new KneeRelativeReference(); p.Reset(true, f.Previous, f.W);
            Check.That(p.Step(trace, Step, Source, f.Goal, f.SourceW, 0f, .1f, 40f / 9f, 20f, 5f, packet, selected), "planner step");
            return p.Current;
        }
        internal static Fixture Example()
        {
            var random = new Random(4131);
            for (int i = 0; i < 1000; i++)
            {
                var f = RandomFixture(random);
                if (Evaluate(f).Better) return f;
            }
            throw new InvalidOperationException("no improving reachable fixture");
        }
        private static Fixture RandomFixture(Random random)
        {
            float angle = -35f - (float)random.NextDouble() * 40f;
            float goal = angle - 2f - (float)random.NextDouble() * 18f;
            var f = new Fixture { Previous = Quaternion.AngleAxis(angle, Vector3.right), Goal = Quaternion.AngleAxis(goal, Vector3.right),
                Foot = Quaternion.AngleAxis(-30f - goal, Vector3.right),
                W = new Vector3((float)random.NextDouble() * 3f - 1.5f, 0f, 0f),
                SourceW = new Vector3((float)random.NextDouble() * 6f - 3f, 0f, 0f) };
            f.Geometry = new KneeGroundTarget.Geometry { Hip = new Vector3(0f, .9f, 0f), Upper = Quaternion.AngleAxis(30f, Vector3.right),
                UpperSpan = new Vector3(0f, -.45f, 0f), LowerSpan = new Vector3(0f, -.45f, 0f), FootRelative = f.Foot,
                FootBindRelative = f.Foot, Center = new Vector3(0f, -.1f, 0f), Half = new Vector3(.05f, .025f, .08f),
                FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f) };
            f.Baseline = Run(f, default, false);
            float min = KneeGroundTarget.Minimum(f.Geometry, f.Baseline.R, f.Foot);
            f.Geometry.Hip.y -= min + .0002f + (float)random.NextDouble() * .006f;
            return f;
        }
        private static KneeGroundTrajectory.Result Evaluate(Fixture f, Vector3? axis = null) => KneeGroundTrajectory.Evaluate(f.Geometry,
            f.Foot, axis ?? Vector3.right, f.Baseline.PreviousR, f.Baseline.PreviousW, f.Baseline.InputR, f.SourceW, f.Baseline.R, f.Baseline.W, Step.Dt, 40f / 9f, 20f);
        private static float CornerOracle(KneeGroundTarget.Geometry g, Quaternion knee, Quaternion foot)
        {
            Vector3 ankle = g.Hip + g.Upper * g.UpperSpan + (g.Upper * knee) * g.LowerSpan;
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                min = MathF.Min(min, (ankle + (g.Upper * knee * foot) * (g.Center + new Vector3(x * g.Half.x, y * g.Half.y, z * g.Half.z))).y - g.FloorCenter.y - g.FloorHalf.y);
            return min;
        }
        private static void Closure(KneeReferenceState s)
        {
            var dq = Quaternion.AngleAxis(s.W.magnitude * Step.Dt * Mathf.Rad2Deg, s.W);
            Check.That(FootRelativeReference.Log(s.R * Quaternion.Inverse(dq * s.PreviousR)).magnitude < .000001f, "R integrates the published W");
            Check.Near((s.A - (s.W - s.PreviousW) / Step.Dt).magnitude, 0f, .00001f, "A differentiates published W");
            Check.That(s.A.magnitude <= 20.0002f && s.W.magnitude <= MathF.Max(40f / 9f, s.PreviousW.magnitude) + .0001f, "unchanged W/A budgets");
        }
        private static void GeometryAndBudgets()
        {
            var random = new Random(4131); int improved = 0, noRoom = 0;
            for (int i = 0; i < 1800; i++)
            {
                var f = RandomFixture(random); var r = Evaluate(f);
                if (r.Reason == KneeGroundTrajectory.Status.NoHeadroom) noRoom++;
                if (!r.Better) continue;
                improved++;
                Check.That(r.GoalMin >= 0f && r.CandidateMin - r.BaselineMin >= KneeGroundTarget.MinGain, "clear unchanged endpoint and meaningful improvement");
                Check.Near(CornerOracle(f.Geometry, r.R, f.Foot), r.CandidateMin, .000001f, "independent all-corner geometry");
                var reverse = Evaluate(f, -Vector3.right);
                Check.That(reverse.Better && reverse.R.Equals(r.R) && reverse.W.Equals(r.W), "calibration sign invariant");
                var actual = Run(f, Packet(f, Step, Source), true); Closure(actual);
                Check.That(actual.GroundTrajectory.Applied && actual.R.Equals(r.R) && actual.W.Equals(r.W), "real planner commits evaluated choice at fixture " + i);
                Check.That(FootRelativeReference.Log(f.Goal * Quaternion.Inverse(actual.R)).magnitude <=
                    FootRelativeReference.Log(f.Goal * Quaternion.Inverse(f.Baseline.R)).magnitude + .000001f, "no added goal error");
            }
            Console.WriteLine("ground synthetic improved=" + improved + " noHeadroom=" + noRoom);
            Check.That(improved > 50 && noRoom > 50, "both successful choice and unavailable authority are exercised");
        }
        private static void Fallbacks()
        {
            var f = Example(); var baseline = f.Baseline;
            var invalid = f; invalid.Geometry.Hip.y = float.NaN;
            var r = Evaluate(invalid); Check.That(!r.Better && r.R.Equals(baseline.R) && r.W.Equals(baseline.W), "invalid geometry exact fallback");
            var low = f; low.Geometry.Hip.y -= .5f; r = Evaluate(low);
            Check.That(r.Reason == KneeGroundTrajectory.Status.EndpointBelow && !r.Better, "unclear ideal endpoint has no authorized detour");
            var clear = f; clear.Geometry.Hip.y += .5f; r = Evaluate(clear);
            Check.That(r.Reason == KneeGroundTrajectory.Status.BaselineClear && !r.Better, "clear baseline not changed");
            var outside = f; outside.Geometry.FloorHalf = new Vector3(.001f, .05f, .001f);
            Check.That(!Evaluate(outside).Evaluated && !Evaluate(f, Vector3.zero).Evaluated, "footprint and axis rejected");
            var noBend = f; noBend.Geometry.UpperSpan = -(f.Baseline.R * f.Geometry.LowerSpan);
            Check.That(!Evaluate(noBend).Better, "cannot cross the bend bound");
            // Foot GOAL clearance cannot stand in for the actual published Foot reference.
            var badFoot = f; badFoot.Foot = Quaternion.AngleAxis(180f, Vector3.forward) * f.Foot;
            var check = Evaluate(badFoot);
            if (check.Evaluated) Check.Near(check.GoalMin, CornerOracle(badFoot.Geometry, badFoot.Goal, badFoot.Foot), .000001f, "held Foot owns the endpoint test");
        }
        private static void Planner()
        {
            var f = Example(); var packet = Packet(f, Step, Source);
            var a = Run(f, packet, false, true); var b = Run(f, packet, true, true);
            Check.That(a.R.Equals(f.Baseline.R) && a.W.Equals(f.Baseline.W) && a.A.Equals(f.Baseline.A), "read-only physical reference exactly unchanged");
            Check.That(a.GroundTrajectory.Better && !a.GroundTrajectory.Applied && b.GroundTrajectory.Applied, "read-only and selected modes separate");
            Check.That(a.InputR.Equals(b.InputR) && a.InputW.Equals(b.InputW), "no new target or nominal source W");
            var readOnlyPacket = packet; readOnlyPacket.Input.Apply = false;
            var readOnly = Run(f, readOnlyPacket, true);
            Check.That(readOnly.GroundTrajectory.Better && !readOnly.GroundTrajectory.Applied &&
                readOnly.R.Equals(f.Baseline.R) && readOnly.W.Equals(f.Baseline.W) && readOnly.A.Equals(f.Baseline.A),
                "packet with no apply authority remains exact baseline even when selected");
            var p = new KneeRelativeReference(); p.Reset(true, f.Previous, f.W);
            p.Step(true, Step, Source, f.Goal, f.SourceW, 0f, .1f, 40f / 9f, 20f, 5f, packet, true);
            var current = p.Current; p.ResetTrace();
            Check.That(p.Current.R.Equals(current.R) && p.Current.W.Equals(current.W), "trace reset does not reseed reference");
            Check.That(p.GetSummary().Contains("steps-evaluated-below-endpointBelow-noHeadroom-better-applied=0|0|0|0|0|0|0"), "all trace counts reset");
            Check.That(p.GetSummary().Contains("/noBend-noLegal=0|0"), "rejection reasons reset");
            p.DisablePreserveTrace(); Check.That(!p.Selected && !p.Current.Valid, "disable clears reference authority");
        }
        private static void CoordinateFrames()
        {
            var f = Example(); var original = Evaluate(f);
            for (int i = 1; i <= 24; i++)
            {
                Quaternion upperBasis = Quaternion.AngleAxis(i * 11f, new Vector3(.2f, .8f, -.3f));
                Quaternion lowerBasis = Quaternion.AngleAxis(i * 7f, new Vector3(-.7f, .1f, .4f));
                Quaternion world = Quaternion.AngleAxis(i * 15f, Vector3.up);
                var g = f.Geometry;
                g.Hip = world * g.Hip; g.Upper = world * g.Upper * Quaternion.Inverse(upperBasis);
                g.UpperSpan = upperBasis * g.UpperSpan; g.LowerSpan = lowerBasis * g.LowerSpan;
                g.FootRelative = lowerBasis * g.FootRelative; g.FootBindRelative = lowerBasis * g.FootBindRelative;
                Quaternion previous = upperBasis * f.Baseline.PreviousR * Quaternion.Inverse(lowerBasis);
                Quaternion goal = upperBasis * f.Baseline.InputR * Quaternion.Inverse(lowerBasis);
                Quaternion baseline = upperBasis * f.Baseline.R * Quaternion.Inverse(lowerBasis);
                var r = KneeGroundTrajectory.Evaluate(g, lowerBasis * f.Foot, lowerBasis * Vector3.right,
                    previous, upperBasis * f.W, goal, upperBasis * f.SourceW, baseline, upperBasis * f.Baseline.W, Step.Dt, 40f / 9f, 20f);
                Check.That(r.Better, "oblique coordinate system still finds reachable improvement");
                Check.Near(r.CandidateMin, original.CandidateMin, .000003f, "world clearance is basis independent");
                Check.Near((r.W - upperBasis * original.W).magnitude, 0f, .00002f, "W belongs to the Upper-local basis");
            }
        }
        private static void Braking()
        {
            var random = new Random(13117); int improved = 0, rejected = 0;
            for (int i = 0; i < 1000; i++)
            {
                var f = RandomFixture(random);
                f.W.y = (float)random.NextDouble() - .5f; f.W.z = (float)random.NextDouble() - .5f;
                f.Baseline = Run(f, default, false);
                var r = Evaluate(f);
                if (!r.Better) { rejected++; continue; }
                improved++;
                Vector3 axis = f.Baseline.PreviousR * Vector3.right;
                Vector3 e0 = FootRelativeReference.Log(f.Baseline.InputR * Quaternion.Inverse(f.Baseline.R));
                axis *= Vector3.Dot(e0, axis) >= 0f ? 1f : -1f;
                Vector3 e1 = FootRelativeReference.Log(f.Baseline.InputR * Quaternion.Inverse(r.R));
                Vector3 v0 = f.Baseline.W - f.SourceW, v1 = r.W - f.SourceW;
                float d0 = MathF.Max(0f, MathF.Max(0f, Vector3.Dot(v0, axis)) * (v0.magnitude / 40f + Step.Dt) - Vector3.Dot(e0, axis));
                float d1 = MathF.Max(0f, MathF.Max(0f, Vector3.Dot(v1, axis)) * (v1.magnitude / 40f + Step.Dt) - Vector3.Dot(e1, axis));
                Check.That(d1 <= d0 + .000002f, "no added relative stopping debt");
                Vector3 delta = r.W - f.Baseline.W;
                Check.Near((delta - axis * Vector3.Dot(delta, axis)).magnitude, 0f, .000001f, "off-axis baseline velocity preserved");
                Check.That(((r.W - f.Baseline.PreviousW) / Step.Dt).magnitude <= 20.0002f, "total A bound includes off-axis share");
            }
            Check.That(improved > 10 && rejected > 10, "mixed velocity acceptance and rejection exercised");
        }
        private static void PacketClocks()
        {
            var f = Example(); var packet = Packet(f, Step, Source);
            for (int fault = 0; fault < 6; fault++)
            {
                var p = packet;
                if (fault == 0) p.Published = false;
                if (fault == 1) p.Sample.Source.Sequence--;
                if (fault == 2) p.Sample.GeometryStep.Sequence--;
                if (fault == 3) p.Input.CompletedStep.UnscaledTime -= .2f;
                if (fault == 4) p.Sample.PlannedFoot = default;
                if (fault == 5) p.SupportId = 0;
                var a = Run(f, p, true);
                Check.That(!a.GroundTrajectory.Applied && a.R.Equals(f.Baseline.R) && a.W.Equals(f.Baseline.W) && a.A.Equals(f.Baseline.A), "packet fault exact fallback " + fault);
            }
            var stale = new KneeRelativeReference(); stale.Reset(true, f.Previous, f.W);
            stale.Step(false, Step, Source, f.Goal, f.SourceW, .2f, .1f, 40f / 9f, 20f, 5f, packet, true);
            Check.That(stale.Current.Stale && !stale.Current.GroundTrajectory.Applied, "stale still uses original braking");
        }
        private static void SourceRates()
        {
            var f = Example();
            foreach (int hz in new[] { 20, 40, 60 })
            {
                var p = new KneeRelativeReference(); p.Reset(true, f.Previous, f.W);
                var packet = default(KneeFootGroundPacket); var source = default(SourceStepContext); int last = -1;
                for (int i = 0; i < 240; i++)
                {
                    int sample = i * hz / 60;
                    var step = Step; step.Sequence += i; step.SourceSequence += sample; step.FixedTime += i / 60f; step.UnscaledTime += i / 60f;
                    if (sample != last)
                    {
                        source = Source; source.Sequence += sample; source.Time += sample / (float)hz; source.Dt = 1f / hz;
                        packet = Packet(f, step, source); last = sample;
                    }
                    Check.That(p.Step(true, step, source, f.Goal, Vector3.zero, step.UnscaledTime - source.Time, .1f,
                        40f / 9f, 20f, 5f, packet, true), "source rate " + hz);
                    Closure(p.Current);
                    Check.That(p.Current.Source.Matches(source) && packet.Sample.PlannedFoot.Equals(f.Foot), "held packets do not advance Foot");
                }
                Check.That(p.Current.W.magnitude < .01f, "bounded reference settles without accumulating speed");
            }
        }
        private static void Allocation()
        {
            var f = Example(); for (int i = 0; i < 100; i++) Evaluate(f);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 500; i++) Evaluate(f);
            Check.That(GC.GetAllocatedBytesForCurrentThread() == before, "pure chooser allocates no managed heap");
        }
    }
}

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        internal static void RunGroundTrajectoryEntryContracts()
        {
            ARL.Contracts.Check.Run("real leg entry selects common ground trajectory in both arms and rechecks native owner", () =>
            {
                var f = ARL.Contracts.KneeGroundTrajectoryContracts.Example();
                foreach (bool selected in new[] { false, true })
                {
                    var leg = Joined(selected); leg._traceEnabled = true; leg._dynamicScope = PelvisRunScope.DynamicLegHipAnchorCouplingContact;
                    leg._kneeReference.Reset(true, f.Previous, f.W);
                    leg._sampledTargetKneeRelative = f.Goal; leg._sampledTargetKneeRelativeAngularVelocityLocal = f.SourceW;
                    leg._footFoundation.Available = true;
                    leg._footFoundation.Packet = ARL.Contracts.KneeGroundTrajectoryContracts.Packet(f, leg._runtimeStep, leg._runtimeSource);
                    leg._footFoundation.Packet.Input.AllowExtension = true;
                    leg._footFoundation.Packet.Sample.AllowExtension = true;
                    leg.ExpectStep("real entry with reachable goal");
                    ARL.Contracts.Check.That(leg._kneeReference.Current.GroundTrajectory.Applied, "both arms retain v132 B intermediate policy");
                    ARL.Contracts.Check.That(leg._controlTargetKneeRelative.Equals(leg._kneeReference.Current.R) &&
                        leg.KneeTargetRelativeA.Equals(leg._kneeReference.Current.A), "controller/FF consume one final reference");
                }
                var bad = Joined(true); bad._kneeReference.Reset(true, f.Previous, f.W);
                bad._dynamicScope = PelvisRunScope.DynamicLegHipAnchorCouplingContact;
                bad._sampledTargetKneeRelative = f.Goal; bad._sampledTargetKneeRelativeAngularVelocityLocal = f.SourceW;
                bad._footFoundation.Available = true;
                bad._footFoundation.Packet = ARL.Contracts.KneeGroundTrajectoryContracts.Packet(f, bad._runtimeStep, bad._runtimeSource);
                bad._footFoundation.Packet.Input.UpperId = 1234;
                bad.ExpectStep("foreign owner optional packet");
                ARL.Contracts.Check.That(!bad._kneeReference.Current.GroundTrajectory.Applied, "real caller rejects native identity mismatch");
            });
        }
    }
}
