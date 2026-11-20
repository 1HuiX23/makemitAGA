using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class KneeGroundExtensionContracts
    {
        internal struct Fixture { internal KneeGroundTarget.Geometry Geometry; internal Quaternion Raw; internal Vector3 Axis; }
        // Saved Render witnesses from the 2026-11-14 v132 B run. No native response
        // is reconstructed: these pin a reproducible INPUT geometry counterexample.
        internal static readonly Fixture[] Logged = {
            new Fixture { Raw = new Quaternion(-0.127170563f,0.00701689068f,-0.0384978838f,0.991109192f), Axis = new Vector3(-0.955108941f,-0.000230244346f,-0.296254665f), Geometry = new KneeGroundTarget.Geometry { Hip = new Vector3(-2.33408713f,0.99420023f,-2.22406483f), Upper = new Quaternion(0.985085011f,-0.0442449935f,-0.0130595323f,-0.16576916f), UpperSpan = new Vector3(-1.99303017E-07f,0.441748381f,8.19563937E-08f), LowerSpan = new Vector3(-1.21071935E-07f,0.475477487f,1.06543291E-06f), Center = new Vector3(0f,-0.0813970342f,0.0491553061f), Half = new Vector3(0.0474999994f,0.0275000017f,0.117216498f), FloorCenter = new Vector3(0.5f,-0.149999663f,0.398396015f), FloorHalf = new Vector3(4.49999952f,0.150000006f,4.39839554f), FootRelative = new Quaternion(-0.00773696369f,0.0180742312f,0.999790788f,0.00575513113f), FootBindRelative = new Quaternion(-0.0554540902f,-0.0957112536f,0.993861794f,-0.0018375311f) } },
            new Fixture { Raw = new Quaternion(-0.131297708f,-0.00708845258f,0.0397568792f,0.990520239f), Axis = new Vector3(-0.955107331f,0.000494042994f,0.296259642f), Geometry = new KneeGroundTarget.Geometry { Hip = new Vector3(1.66112363f,0.98360467f,1.96762466f), Upper = new Quaternion(-0.464918375f,0.114051871f,0.863077402f,0.161060423f), UpperSpan = new Vector3(-7.45058131E-08f,0.441749036f,-6.33299351E-08f), LowerSpan = new Vector3(-1.26659899E-07f,0.475474775f,2.83122063E-07f), Center = new Vector3(0f,-0.0813742504f,0.0491556413f), Half = new Vector3(0.047499992f,0.027499998f,0.117217302f), FloorCenter = new Vector3(0.5f,-0.149999663f,0.398396015f), FloorHalf = new Vector3(4.49999952f,0.150000006f,4.39839554f), FootRelative = new Quaternion(-0.0786686838f,-0.0279198829f,-0.995999992f,0.0318851732f), FootBindRelative = new Quaternion(-0.0315593332f,0.0910935774f,-0.995328009f,-0.00537794456f) } }
        };
        // v133 A deepest unresolved snapshots, 2026-11-14; left remains subnoise, not strictly clear.
        private static readonly Fixture[] Logged133 = {
            new Fixture { Raw = new Quaternion(-0.107292503f,0.00705513358f,-0.0324810669f,0.993672132f), Axis = new Vector3(-0.954987407f,-9.86595751E-07f,-0.296646476f), Geometry = new KneeGroundTarget.Geometry { Hip = new Vector3(-2.31004405f,1.0118506f,-2.14744139f), UpperSpan = new Vector3(-9.49949168E-08f,0.441748798f,2.75671567E-07f), LowerSpan = new Vector3(-1.28522515E-07f,0.475477427f,9.31322575E-07f), Upper = new Quaternion(0.856808007f,0.0149073172f,0.502333522f,-0.11540699f), Center = new Vector3(0f,-0.0813728422f,0.0491552874f), Half = new Vector3(0.047499992f,0.0275000017f,0.117216453f), FloorCenter = new Vector3(0.5f,-0.149999663f,0.398396015f), FloorHalf = new Vector3(4.49999952f,0.150000006f,4.39839554f), FootRelative = new Quaternion(-0.0175945461f,-0.044642318f,0.998846591f,0.00201890781f), FootBindRelative = new Quaternion(-0.0590053648f,-0.0936490595f,0.993855119f,-0.000740402378f) } },
            new Fixture { Raw = new Quaternion(-0.137419105f,-0.00706097484f,0.0416078418f,0.98961401f), Axis = new Vector3(-0.954987288f,7.41215274E-07f,0.296646774f), Geometry = new KneeGroundTarget.Geometry { Hip = new Vector3(-1.27989006f,0.992758155f,1.82519209f), UpperSpan = new Vector3(-4.84287909E-07f,0.441749215f,1.4528635E-07f), LowerSpan = new Vector3(6.70552254E-08f,0.475474924f,2.6822093E-07f), Upper = new Quaternion(-0.683208466f,0.0909468308f,0.708459377f,0.151789755f), Center = new Vector3(0f,-0.0813793391f,0.0491557159f), Half = new Vector3(0.0474999957f,0.0275000017f,0.117217466f), FloorCenter = new Vector3(0.5f,-0.149999663f,0.398396015f), FloorHalf = new Vector3(4.49999952f,0.150000006f,4.39839554f), FootRelative = new Quaternion(-0.00619259477f,-0.0258178767f,-0.999645591f,0.00227039214f), FootBindRelative = new Quaternion(-0.0268794447f,0.0941019207f,-0.995182335f,-0.00592955202f) } }
        };
        internal static void Run()
        {
            Check.Run("logged left/right goals improve only after bounded extension is allowed", LoggedWitnesses);
            Check.Run("v133 A saved geometry distinguishes subnoise partial from strict clearance", Logged133Witnesses);
            Check.Run("extension never crosses straight knee and preserves full Foot constraints", SearchBounds);
            Check.Run("extension candidate is invariant to calibration sign and oblique local bases", Coordinates);
            Check.Run("extension retains invalid, outside, Foot-only and ambiguous-axis fallback", Fallbacks);
            Check.Run("extension packet rechecks policy and pair apply authority", PacketPolicy);
            Check.Run("extension search allocates no managed heap on repeated witnesses", Allocation);
            FootAnkleProxyFoundation.RunExtensionSourceRates();
            SingleLegShadowChain.RunExtensionEntryContracts();
        }
        private static KneeGroundTarget.Sample Evaluate(Fixture f, bool allow) => KneeGroundTarget.Evaluate(f.Geometry, f.Raw, f.Axis, allow);
        private static float Bend(KneeGroundTarget.Geometry g, Quaternion knee) => MathF.Acos(Math.Clamp(
            Vector3.Dot(g.UpperSpan.normalized, (knee * g.LowerSpan).normalized), -1f, 1f)) * Mathf.Rad2Deg;
        private static float Corners(KneeGroundTarget.Geometry g, Quaternion knee, Quaternion foot)
        {
            var ankle = g.Hip + g.Upper * g.UpperSpan + (g.Upper * knee) * g.LowerSpan;
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                min = MathF.Min(min, (ankle + (g.Upper * knee * foot) *
                    (g.Center + new Vector3(x * g.Half.x, y * g.Half.y, z * g.Half.z))).y - g.FloorCenter.y - g.FloorHalf.y);
            return min;
        }
        private static void Legal(Fixture f, KneeGroundTarget.Sample s)
        {
            Check.That(s.Valid && s.Candidate && MathF.Abs(s.Correction) <= 5f && s.GoalBend <= 150f, "original Knee limits");
            Check.That(s.GoalAnkleY - s.RawAnkleY >= .0001f && s.GoalMin - s.FootOnlyMin >= .0001f, "both independent improvements remain meaningful");
            var foot = KneeGroundTarget.SelectedFoot(f.Geometry, s.Foot);
            Check.That(FootGroundReference.GoalWithinBounds(f.Geometry.Upper * s.Goal, f.Geometry.FootRelative, foot,
                f.Geometry.FootBindRelative), "original Foot correction, ROM and upright constraints");
            Check.That(FootBoxGeometry.Inside(KneeGroundTarget.Ankle(f.Geometry, s.Goal), f.Geometry.Upper * s.Goal * foot,
                f.Geometry.Center, f.Geometry.Half, f.Geometry.FloorCenter, f.Geometry.FloorHalf), "all-corner footprint");
            Check.Near(Corners(f.Geometry, s.Goal, foot), s.GoalMin, .000002f, "independent eight-corner minimum");
            float previous = s.RawBend;
            for (int i = 1; i <= 64; i++)
            {
                var knee = f.Raw * Quaternion.AngleAxis(s.Correction * i / 64f, f.Axis);
                float bend = Bend(f.Geometry, knee);
                Check.That(s.Extension ? bend <= previous + .0002f && bend >= 4.9998f : bend >= previous - .0002f,
                    "whole selected arc stays on its monotone bend branch with extension margin");
                previous = bend;
            }
        }
        private static void LoggedWitnesses()
        {
            float[] originalMm = { -4.42614222f, -3.79267597f }, corrections = { -4.6875f, -3.4375f };
            for (int i = 0; i < Logged.Length; i++)
            {
                var f = Logged[i]; var a = Evaluate(f, false); var b = Evaluate(f, true);
                Check.That(a.Valid && !a.Candidate && a.Goal.Equals(f.Raw), "unchanged flexion-only path remains unresolved");
                Check.Near(a.GoalMin * 1000f, originalMm[i], .001f, "reproduce saved goal deficit");
                Check.That(b.Extension && b.AllowExtension && !b.Applied && b.GoalMin >= 0f, "extension clears INPUT, never self-grants application");
                Check.Near(b.Correction, corrections[i], .00001f, "original 16-point grid supplies witness");
                Check.That(b.GoalBend > 10f, "logged cases remain comfortably bent"); Legal(f, b);
                Console.WriteLine("extension witness side=" + i + " correctionDeg=" + b.Correction + " bendDeg=" + b.GoalBend + " soleMm=" + b.GoalMin * 1000f);
            }
        }
        private static void Logged133Witnesses()
        {
            float[] beforeMm = { -.592557907f, -4.58350658f };
            for (int i = 0; i < Logged133.Length; i++)
            {
                var f = Logged133[i]; var a = Evaluate(f, false); var b = Evaluate(f, true);
                Check.Near(a.GoalMin * 1000f, beforeMm[i], .0002f, "saved v133 A signed geometry reproduced");
                Check.That(b.Extension && b.Correction == -5f && b.GoalMin > a.GoalMin && b.GoalBend >= 5f, "same-input bounded extension helps");
                Check.That(i == 0 ? b.GoalMin < 0f && b.GoalMin >= -.0001f : b.GoalMin >= 0f, "subnoise is not mislabeled strictly clear");
                Legal(f, b);
            }
        }
        private static Fixture Synthetic(float bend, float upper = 30f)
        {
            var f = new Fixture { Raw = Quaternion.AngleAxis(-bend, Vector3.right), Axis = Vector3.right };
            f.Geometry = new KneeGroundTarget.Geometry { Hip = new Vector3(0f, .9f, 0f), Upper = Quaternion.AngleAxis(upper, Vector3.right),
                UpperSpan = new Vector3(0f, -.45f, 0f), LowerSpan = new Vector3(0f, -.45f, 0f),
                FootRelative = Quaternion.AngleAxis(bend - upper, Vector3.right), FootBindRelative = Quaternion.AngleAxis(bend - upper, Vector3.right),
                Center = new Vector3(0f, -.1f, 0f), Half = new Vector3(.05f, .025f, .08f),
                FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f) };
            f.Geometry.Hip.y -= KneeGroundTarget.Minimum(f.Geometry, f.Raw) + .008f;
            return f;
        }
        private static void SearchBounds()
        {
            var random = new Random(4133); int extension = 0, flexion = 0, margin = 0, partial = 0;
            for (int n = 0; n < 1200; n++)
            {
                float bend = n < 120 ? n * .05f : 5f + (float)random.NextDouble() * 144f;
                var f = Synthetic(bend, -50f + (float)random.NextDouble() * 130f);
                f.Geometry.Hip.y += .006f - (float)random.NextDouble() * .025f;
                var a = Evaluate(f, false); var b = Evaluate(f, true);
                Check.That(!a.Extension && !a.AllowExtension, "A never gains extension permission");
                if (b.Candidate) { Legal(f, b); if (b.Extension) extension++; else flexion++; if (b.GoalMin < 0f) partial++; }
                if (bend < 5f) { Check.That(!b.Extension, "input inside straight margin cannot extend"); margin++; }
            }
            Check.That(extension > 30 && flexion > 30 && margin > 30 && partial > 30, "direction, margin and partial cases exercised");
            // The extension option must not consume the entire remaining bend near straight.
            var near = Synthetic(6f); var limited = Evaluate(near, true);
            Check.That(limited.Extension && limited.GoalBend >= 5f && limited.GoalMin < 0f && MathF.Abs(limited.Correction) < 1.1f,
                "margin binds before a below-ground target could be cleared");
            var over = Synthetic(150.1f); Check.That(!Evaluate(over, true).Candidate, "old maximum raw-bend guard retained");
        }
        private static void Coordinates()
        {
            foreach (var f in Logged)
            {
                var original = Evaluate(f, true); var reversed = f; reversed.Axis = -f.Axis;
                var sign = Evaluate(reversed, true);
                Check.That(sign.Goal.Equals(original.Goal) && sign.Foot.Goal.Equals(original.Foot.Goal), "axis sign has no physical effect");
                for (int n = 1; n <= 12; n++)
                {
                    var a = Quaternion.AngleAxis(n * 11f, new Vector3(.2f, .8f, -.3f));
                    var b = Quaternion.AngleAxis(n * 7f, new Vector3(-.7f, .1f, .4f));
                    var moved = f; var g = f.Geometry;
                    g.Upper = g.Upper * Quaternion.Inverse(a); g.UpperSpan = a * g.UpperSpan; g.LowerSpan = b * g.LowerSpan;
                    g.FootRelative = b * g.FootRelative; g.FootBindRelative = b * g.FootBindRelative;
                    moved.Geometry = g; moved.Raw = a * f.Raw * Quaternion.Inverse(b); moved.Axis = b * f.Axis;
                    var s = Evaluate(moved, true);
                    Check.That(s.Extension && s.GoalMin >= 0f, "oblique body-local bases retain clear extension");
                    Check.Near(s.Correction, original.Correction, .0001f, "grid direction is coordinate invariant");
                    Check.Near(s.GoalMin, original.GoalMin, .000003f, "same physical clearance");
                }
            }
        }
        private static void Fallbacks()
        {
            var f = Logged[0]; var bad = f; bad.Geometry.Hip.y = float.NaN;
            Check.That(!Evaluate(bad, true).Valid, "NaN has no candidate");
            bad = f; bad.Geometry.FloorHalf.x = .001f; Check.That(!Evaluate(bad, true).Candidate, "outside support cannot extend");
            bad = f; bad.Geometry.Hip.y += .2f; Check.That(!Evaluate(bad, true).Candidate, "Foot-only clear retains Knee");
            bad = f; bad.Axis = Vector3.zero; Check.That(!Evaluate(bad, true).Valid, "missing PCA axis unavailable");
            bad = Synthetic(0f); Check.That(!Evaluate(bad, true).Candidate, "ambiguous straight branch unchanged");
        }
        private static void PacketPolicy()
        {
            var f = KneeGroundTrajectoryContracts.Example(); var step = KneeGroundTrajectoryContracts.Step; var source = KneeGroundTrajectoryContracts.Source;
            var valid = KneeGroundTrajectoryContracts.Packet(f, step, source);
            valid.Input.AllowExtension = valid.Sample.AllowExtension = true;
            Check.That(valid.Matches(step, source, 11, 12, 13, 21), "consistent B packet matches");
            for (int fault = 0; fault < 5; fault++)
            {
                var p = valid;
                if (fault == 0) p.Input.AllowExtension = false;
                if (fault == 1) p.Sample.AllowExtension = false;
                if (fault == 2) p.Input.Apply = false;
                if (fault == 3) p.Sample.Applied = true;
                if (fault == 4) { p.Input.AllowExtension = p.Sample.AllowExtension = false; p.Sample.Candidate = p.Sample.Applied = true; p.Sample.RawBend = 15f; p.Sample.GoalBend = 12f; }
                Check.That(!p.Matches(step, source, 11, 12, 13, 21), "inconsistent immutable policy/apply pair rejected: " + fault);
            }
        }
        private static void Allocation()
        {
            var f = Logged[0]; for (int i = 0; i < 100; i++) Evaluate(f, true);
            long before = GC.GetAllocatedBytesForCurrentThread(); bool all = true;
            for (int i = 0; i < 500; i++) all &= Evaluate(f, true).Extension;
            long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
            Check.That(all && allocated == 0, "real extension search creates no managed garbage; not an FPS claim");
        }
    }
}

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private FootAnkleProxyFoundation() { }
        // Test-only native observation setup; all planning/publication remains linked production.
        private FootAnkleProxyFoundation(KneeGroundTarget.Geometry g, KneeFootGroundInput input)
        {
            _bindSourceRelative = g.FootBindRelative; _lowerAnkleAnchorLocal = g.LowerSpan; _footAnkleAnchorLocal = Vector3.zero;
            _collider.center = g.Center; _collider.size = g.Half * 2f;
            _supportSurface.Center = g.FloorCenter; _supportSurface.Half = g.FloorHalf;
            _report.RuntimeContext = input.CompletedStep;
            _previousReferenceSupport = new PreviousFootSupport { Step = input.CompletedStep, SupportId = 21, StrongStance = true, SourceAge = 0f };
            _lowerBody.rotation = input.Upper * input.RawKnee; _lowerBody.position = input.Hip + input.Upper * input.UpperSpan;
            var previous = input.Source; previous.Sequence = input.CompletedStep.SourceSequence; previous.Time = input.CompletedStep.UnscaledTime;
            _groundReference.Select(true, true, true); _groundReference.SeedInput(previous, g.FootRelative, Vector3.zero);
            _relativeReference.Select(true); _relativeReference.Seed(previous, g.FootRelative, Vector3.zero);
            _relativeReference.SelectGroundTrajectory(true);
            _groundReference.SelectKneeProgress(input.PreferKneeProgress);
        }
        internal static KneeFootGroundPacket PublishExtensionFixture(KneeGroundTarget.Geometry g, KneeFootGroundInput input)
        {
            var foot = new FootAnkleProxyFoundation(g, input);
            foot.ObserveFootReference(input.Source, g.FootRelative, Vector3.zero, input.Source.Dt, input);
            var step = input.CompletedStep; step.Sequence++; step.SourceSequence = input.Source.Sequence;
            step.FixedTime += step.Dt; step.UnscaledTime = input.Source.Time;
            ARL.Contracts.Check.That(foot.Read(step, input.Source, out var p), "actual Foot planner publishes extension-policy packet");
            return p;
        }
        internal static void RunExtensionSourceRates() => ARL.Contracts.Check.Run("real extension Foot and Knee planners stay continuous at 20/40/60 Hz", () =>
        {
            var f = ARL.Contracts.KneeGroundExtensionContracts.Logged[0];
            foreach (int hz in new[] { 20, 40, 60 })
            {
                var input = new KneeFootGroundInput { Enabled = true, Apply = true, AllowExtension = true,
                    Source = Source(201, 1f + 1f / hz, 1f / hz), CompletedStep = Step(100, 200, 1f),
                    ParentId = 11, UpperId = 12, LowerId = 13, Hip = f.Geometry.Hip, Upper = f.Geometry.Upper,
                    UpperSpan = f.Geometry.UpperSpan, RawKnee = f.Raw, Axis = f.Axis };
                var foot = new FootAnkleProxyFoundation(f.Geometry, input); var knee = new KneeRelativeReference();
                knee.Reset(true, f.Raw, Vector3.zero); var source = Source(200, 1f); int lastRender = 0, used = 0;
                for (int n = 1; n <= 180; n++)
                {
                    int render = n * hz / 60; float time = 1f + n / 60f;
                    if (render > lastRender)
                    {
                        var completed = Step(99 + n, source.Sequence, time - 1f / 60f);
                        source = Source(200 + render, time, time - source.Time);
                        foot._report.RuntimeContext = foot._previousReferenceSupport.Step = completed;
                        input.Source = source; input.CompletedStep = completed;
                        foot.ObserveFootReference(source, f.Geometry.FootRelative, Vector3.zero, source.Dt, input);
                        lastRender = render;
                    }
                    var step = Step(100 + n, source.Sequence, time); bool read = foot.Read(step, source, out var packet);
                    Quaternion goal = read && packet.Sample.Applied ? packet.Sample.Goal : f.Raw;
                    var previous = knee.Current;
                    ARL.Contracts.Check.That(knee.Step(true, step, source, goal, Vector3.zero, time - source.Time,
                        .1f, 40f / 9f, 20f, 5f, packet, true), "extension shared Fixed plan at source Hz=" + hz);
                    if (previous.Valid) ARL.Contracts.Check.That(knee.Current.PreviousR.Equals(previous.R) &&
                        knee.Current.PreviousW.Equals(previous.W), "extension does not reseed on a new source or held step");
                    ARL.Contracts.Check.That(knee.Current.A.magnitude <= 20.002f && foot._relativeReference.Current.A.magnitude <= 20.002f,
                        "both original A budgets retained");
                    if (render > 0)
                    {
                        ARL.Contracts.Check.That(read && packet.Sample.Extension && packet.Sample.Applied &&
                            packet.Sample.PlannedFoot.Equals(foot._relativeReference.Current.Rotation), "extension packet and actual Foot reference stay coherent");
                        used++;
                    }
                }
                ARL.Contracts.Check.That(used > 170, "real extension inputs exercised throughout each rate sequence");
            }
        });
    }
    internal sealed partial class SingleLegShadowChain
    {
        internal static void RunExtensionEntryContracts() => ARL.Contracts.Check.Run("actual capture Render publication Knee consumption and stop preserve arm policy", () =>
        {
            foreach (bool b in new[] { false, true })
            {
                var f = ARL.Contracts.KneeGroundExtensionContracts.Logged[0]; var g = f.Geometry;
                var leg = Joined(b); leg._traceEnabled = true; leg._dynamicScope = PelvisRunScope.DynamicLegHipAnchorCouplingContact;
                // Capture after completed Fixed, with a genuinely newer Render source.
                leg._runtimeSource.Sequence = leg._runtimeStep.SourceSequence + 1;
                leg._runtimeSource.Time = leg._runtimeStep.UnscaledTime + 1f / 60f; leg._runtimeSource.Dt = 1f / 60f;
                leg._pelvisBody.position = g.Hip; leg._pelvisBody.rotation = Quaternion.identity;
                leg._upperBody.rotation = g.Upper; leg._kneeParentAnchorUpperLocal = g.UpperSpan;
                leg._sampledTargetKneeRelative = f.Raw; leg._kneeFlexionAxisLowerLocal = f.Axis;
                var captured = leg.CaptureKneeFootGroundInput();
                ARL.Contracts.Check.That(captured.Matches(leg._runtimeSource) && captured.AllowExtension, "native capture carries selected policy");
                var packet = FootAnkleProxyFoundation.PublishExtensionFixture(g, captured);
                ARL.Contracts.Check.That(packet.Sample.Extension && packet.Sample.Applied, "both arms retain the bounded-extension input");
                leg._runtimeStep.Sequence++; leg._runtimeStep.SourceSequence = leg._runtimeSource.Sequence;
                leg._runtimeStep.FixedTime += leg._runtimeStep.Dt; leg._runtimeStep.UnscaledTime = leg._runtimeSource.Time;
                leg._physicalParent = Parent(leg._runtimeStep);
                leg._kneeReference.Reset(true, f.Raw, Vector3.zero); leg._footFoundation.Available = true; leg._footFoundation.Packet = packet;
                leg.ExpectStep("extension actual controller entry");
                ARL.Contracts.Check.That(FootRelativeReference.Log(leg._kneeReference.Current.InputR * Quaternion.Inverse(packet.Sample.Goal)).magnitude < .000001f, "actual Knee input owns the selected target");
                ARL.Contracts.Check.That(leg._kneeReference.Current.A.magnitude <= 20.002f && leg._controlTargetKneeRelative.Equals(leg._kneeReference.Current.R), "controller remains bounded and consumes continuous reference");
                ARL.Contracts.Check.That(leg.KneeGroundSummary().Contains("/extensionCandidate-applied=1|1"), "cold summary distinguishes extension");
                var before = leg._kneeReference.Current; leg.ResetKneeGroundTrace();
                ARL.Contracts.Check.That(leg._kneeReference.Current.Equals(before) && leg._footFoundation.Packet.Published, "trace reset leaves live ownership");
                leg._runtimeStep.Sequence++; leg._runtimeStep.FixedTime += leg._runtimeStep.Dt;
                leg._runtimeStep.UnscaledTime += leg._runtimeStep.Dt; leg._physicalParent = Parent(leg._runtimeStep);
                leg.ExpectStep("held source consumes same immutable direction policy");
                ARL.Contracts.Check.That(leg._kneeGroundSample.Extension && leg._footFoundation.Packet.Equals(packet),
                    "held Fixed cannot rewrite Foot packet or search policy");
                // A correctly formed historical flexion-only packet must fail at
                // the real current-owner boundary, independently of packet validity.
                captured.AllowExtension = false;
                leg._footFoundation.Packet = FootAnkleProxyFoundation.PublishExtensionFixture(g, captured);
                leg._runtimeStep.Sequence++; leg._runtimeStep.FixedTime += leg._runtimeStep.Dt;
                leg._runtimeStep.UnscaledTime += leg._runtimeStep.Dt; leg._physicalParent = Parent(leg._runtimeStep);
                leg.ExpectStep("foreign policy packet falls back without reseeding");
                ARL.Contracts.Check.That(!leg._kneeGroundSample.Valid && !leg._kneeGroundSample.Applied &&
                    leg._kneeGroundSample.Reason == "INPUT_DIRECTION_POLICY_MISMATCH" && leg._kneeGroundSample.AllowExtension &&
                    !leg._kneeReference.Current.GroundTrajectory.Applied,
                    "current owner rejects another arm while keeping its own diagnostic policy");
                leg.DisableKneeReferencePreserveTrace();
                ARL.Contracts.Check.That(!leg.KneeGroundExtensionSelected && !leg._footFoundation.Packet.Published, "stop revokes policy and live packet");
                leg._torso.PelvisReceiverControl.Arm = b ? PelvisDynamicReceiverIsolation.DynamicHipArmA : PelvisDynamicReceiverIsolation.DynamicHipArmB;
                leg.ResetKneeReferenceForDrive();
                ARL.Contracts.Check.That(leg.KneeGroundExtensionSelected && !leg._footFoundation.Packet.Published, "restart chooses new arm without inheriting old packet");
            }
        });
    }
}
