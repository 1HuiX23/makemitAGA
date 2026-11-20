using System;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class KneeGroundFeasibilityContracts
    {
        internal static void Run()
        {
            Check.Run("relaxed height ceiling bounds independent full Box samples across oblique Knee arcs", EnvelopeOracle);
            Check.Run("height ceiling includes interior maxima, axis sign and world translation", InteriorAndFrames);
            Check.Run("unavailable and optimistic bounds never certify a legal joint target", BoundLimits);
            Check.Run("joint target, published Foot and final Knee are distinct evidence stages", Stages);
            Check.Run("feasibility trace counts held Fixed separately and retains one unresolved witness", TraceLifecycle);
            Check.Run("feasibility hot path has no managed heap allocation", Allocation);
            FootAnkleProxyFoundation.RunFeasibilityPublicationContracts();
            SingleLegShadowChain.RunFeasibilityEntryContracts();
        }
        internal static KneeGroundTarget.Geometry Geometry() => new KneeGroundTarget.Geometry
        {
            Hip = new Vector3(0f, .86f, 0f), Upper = Quaternion.AngleAxis(30f, Vector3.right),
            UpperSpan = new Vector3(0f, -.45f, 0f), LowerSpan = new Vector3(0f, -.45f, 0f),
            FootRelative = Quaternion.AngleAxis(30f, Vector3.right), FootBindRelative = Quaternion.AngleAxis(30f, Vector3.right),
            Center = new Vector3(0f, -.1f, 0f), Half = new Vector3(.05f, .025f, .08f),
            FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f)
        };
        private static float Rand(Random r, float a, float b) => a + (b - a) * (float)r.NextDouble();
        private static Vector3 Axis(Random r) => new Vector3(Rand(r, -1f, 1f), Rand(r, -1f, 1f), Rand(r, -1f, 1f)).normalized;
        private static void EnvelopeOracle()
        {
            var random = new Random(4132); int deficits = 0;
            for (int n = 0; n < 200; n++)
            {
                var g = Geometry(); g.Hip.y = Rand(random, -.2f, 1.1f);
                g.Upper = Quaternion.AngleAxis(Rand(random, -180f, 180f), Axis(random));
                g.LowerSpan = Axis(random) * Rand(random, .15f, .7f);
                g.Center.x = Rand(random, -.04f, .04f); g.Center.z = Rand(random, -.07f, .07f);
                var raw = Quaternion.AngleAxis(Rand(random, -160f, 160f), Axis(random)); var axis = Axis(random);
                var e = KneeGroundFeasibility.AtRender(g, raw, axis);
                Check.That(e.Valid && e.RoundoffGuard > 0f, "unit input has guarded relaxed bound");
                if (e.HeightDeficit) deficits++;
                for (int k = 0; k <= 50; k++)
                {
                    var knee = raw * Quaternion.AngleAxis(-5f + k / 5f, axis);
                    var ankle = KneeGroundTarget.Ankle(g, knee);
                    Check.That(ankle.y - (g.FloorCenter.y + g.FloorHalf.y) <= e.MaxAnkleY + e.RoundoffGuard,
                        "analytic maximum bounds all sampled Knee heights");
                    for (int j = 0; j < 24; j++)
                    {
                        float azimuth = Rand(random, -180f, 180f), tilt = j == 0 ? 0f : (j == 1 ? 30f : Rand(random, 0f, 30f));
                        var world = Quaternion.AngleAxis(azimuth, Vector3.up) * Quaternion.AngleAxis(tilt, Vector3.right) *
                            Quaternion.AngleAxis(Rand(random, -180f, 180f), Vector3.up);
                        float min = float.PositiveInfinity;
                        // Independent eight-corner oracle; no production support function.
                        for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                            min = MathF.Min(min, (ankle + world * (g.Center + new Vector3(x * g.Half.x, y * g.Half.y, z * g.Half.z))).y -
                                g.FloorCenter.y - g.FloorHalf.y);
                        Check.That(min <= e.CeilingMin, "full Box cannot clear above the relaxed upper bound");
                    }
                }
            }
            Check.That(deficits > 20, "oracle includes genuinely negative upper bounds");
        }
        private static void InteriorAndFrames()
        {
            var g = Geometry(); g.Upper = Quaternion.identity; g.LowerSpan = new Vector3(0f, .45f, 0f);
            var raw = Quaternion.AngleAxis(2f, Vector3.right);
            var e = KneeGroundFeasibility.AtRender(g, raw, Vector3.right);
            Check.Near(e.AtDegrees, -2f, .0001f, "maximum can lie between both interval endpoints");
            Check.Near(e.MaxAnkleY, g.Hip.y, .000001f, "interior straight-up Lower maximum");
            var flipped = KneeGroundFeasibility.AtRender(g, raw, -Vector3.right);
            Check.Near(flipped.CeilingMin, e.CeilingMin, 1e-6f, "axis sign cannot alter relaxed reachable set");
            Check.Near(flipped.AtDegrees, -e.AtDegrees, 1e-4f, "witness angle changes sign");
            var yaw = Quaternion.AngleAxis(71f, Vector3.up); g.Upper = yaw * g.Upper;
            g.Hip += new Vector3(7f, 20f, -3f); g.FloorCenter += new Vector3(7f, 20f, -3f);
            var moved = KneeGroundFeasibility.AtRender(g, raw, Vector3.right);
            Check.Near(moved.CeilingMin - moved.RoundoffGuard, e.CeilingMin - e.RoundoffGuard, 3e-6f,
                "ceiling is relative to support height, with separately reported scale guard");
        }
        private static void BoundLimits()
        {
            var g = Geometry(); var raw = Quaternion.AngleAxis(-60f, Vector3.right);
            Check.That(KneeGroundFeasibility.AtRender(g, raw, Vector3.right).HeightDeficit, "low fixed Hip proves relaxed height deficit");
            g.Hip.y = 1.2f; g.FloorHalf.x = .001f;
            var bound = KneeGroundFeasibility.AtRender(g, raw, Vector3.right);
            var target = KneeGroundTarget.Evaluate(g, raw, Vector3.right);
            Check.That(bound.Valid && bound.CeilingMin > 0f && !target.Candidate && !target.Foot.Valid,
                "positive relaxed ceiling does not bypass footprint or certify a legal pose");
            g = Geometry(); g.Center.x = .06f;
            Check.That(KneeGroundFeasibility.AtRender(g, raw, Vector3.right).Status == KneeGroundFeasibility.BoundStatus.UnsupportedBox,
                "ankle outside box footprint has no proof from this formula");
            g = Geometry(); g.Center.y = 0f;
            Check.That(!KneeGroundFeasibility.AtRender(g, raw, Vector3.right).Valid, "box above ankle violates bound assumptions");
            g = Geometry(); g.Upper.w *= 1.0005f;
            Check.That(g.Valid && !KneeGroundFeasibility.AtRender(g, raw, Vector3.right).Valid,
                "diagnostic rejects a loose-unit quaternion without changing control finite gate");
            Check.That(!KneeGroundFeasibility.AtRender(Geometry(), raw, Vector3.zero).Valid &&
                !KneeGroundFeasibility.AtRender(default, raw, Vector3.right).Valid, "invalid inputs stay unavailable");
        }
        internal static KneeGroundTarget.Sample Sample(KneeGroundTarget.Geometry g)
        {
            var raw = Quaternion.AngleAxis(-60f, Vector3.right);
            var s = KneeGroundTarget.Evaluate(g, raw, Vector3.right);
            s.Step = KneeGroundTrajectoryContracts.Step; s.Source = KneeGroundTrajectoryContracts.Source;
            s.Feasibility = KneeGroundFeasibility.AtRender(g, raw, Vector3.right);
            s.PlannedFoot = KneeGroundTarget.SelectedFoot(g, s.Foot);
            s.Planned = true; s.PlanMin = KneeGroundTarget.Minimum(g, s.Goal, s.PlannedFoot);
            KneeGroundFeasibility.AssessFixed(ref s, g); return s;
        }
        private static void Stages()
        {
            var g = Geometry(); var s = Sample(g);
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalBelowHeightCeiling, "selected pair cannot clear at relaxed height");
            s.Feasibility.CeilingMin = .001f; KneeGroundFeasibility.AssessFixed(ref s, g);
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalBelowSearchUnresolved,
                "optimistic ceiling is not a feasible target certificate");
            s.Feasibility = default; KneeGroundFeasibility.AssessFixed(ref s, g);
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalBelowBoundUnavailable, "missing optional proof is explicit");
            g.Hip.y = .9f; s = Sample(g);
            Check.That(s.GoalMin >= 0f && s.PlanMin >= 0f, "fixture has actual bounded legal goal");
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalAndReferencesClear, "goal and references clear in saved geometry");
            s.PlanMin = -.003f; KneeGroundFeasibility.AssessFixed(ref s, g);
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalClearKneeReferenceBelow, "Knee reference clearance gap");
            s.PlannedFoot = Quaternion.AngleAxis(80f, Vector3.right); KneeGroundFeasibility.AssessFixed(ref s, g);
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.GoalClearFootReferenceBelow,
                "published Foot counterfactual is checked before final Knee, not conflated with ideal pair");
            s = Sample(g);
            string mixed = KneeGroundFeasibility.ActualComparison(s, s.Step, s.Source, true, -.004f);
            Check.That(mixed.Contains("PLAN_CLEAR_ACTUAL_BELOW") && mixed.Contains("NOT_RESPONSE_PROOF"), "different-time geometry is labeled");
            var foreign = s.Source; foreign.Sequence++;
            Check.That(KneeGroundFeasibility.ActualComparison(s, s.Step, foreign, true, -.004f).EndsWith("UNAVAILABLE"), "foreign source cannot pair with actual sample");
            s.Planned = false; KneeGroundFeasibility.AssessFixed(ref s, g);
            Check.That(s.FeasibilityStage == KneeGroundFeasibility.Stage.Unavailable, "missing plan cannot masquerade as clear zero");
        }
        private static void TraceLifecycle()
        {
            var trace = new KneeGroundFeasibility.Trace(); var s = Sample(Geometry());
            var p = new KneeFootGroundPacket { Geometry = Geometry(), Sample = s };
            var r = new KneeReferenceState { Step = s.Step, Source = s.Source, R = s.Goal };
            trace.Observe(s, p, r); s.Step.Sequence++; r.Step = s.Step; trace.Observe(s, p, r);
            Check.That(trace.Summary().Contains("steps-valid-held-sources=2|2|1|1") &&
                trace.Summary().Contains("sourceGoalBelow(total-heightDeficit-searchUnresolved-boundUnavailable-subNoise)=1|1|0|0|0"), "one Render geometry, two Fixed consumptions");
            s.Source.Sequence++; s.Step.SourceSequence = s.Source.Sequence; s.Feasibility.CeilingMin = 0f;
            KneeGroundFeasibility.AssessFixed(ref s, p.Geometry); r.Source = s.Source; r.Step = s.Step;
            trace.Observe(s, p, r);
            Check.That(trace.Summary().Contains("JOINT_FEASIBILITY_WITNESS_V041320") &&
                trace.Summary().Contains("footR(raw-bind-goal-planned)="), "unresolved witness preserves reproducible original geometry");
            trace.Fault(); string saved = trace.Summary();
            Check.That(saved == trace.Summary() && saved.Contains("diagnosticFaults=1"), "cold output is idempotent");
            trace.Reset();
            Check.That(trace.Summary().Contains("steps-valid-held-sources=0|0|0|0") &&
                trace.Summary().Contains("unresolvedWitness=NONE"), "trace reset removes only its own evidence");
        }
        private static void Allocation()
        {
            var g = Geometry(); var raw = Quaternion.AngleAxis(-60f, Vector3.right); var s = Sample(g);
            var t = new KneeGroundFeasibility.Trace(); var p = new KneeFootGroundPacket { Geometry = g };
            var r = new KneeReferenceState { Step = s.Step, Source = s.Source, R = s.Goal };
            for (int i = 0; i < 100; i++) { s.Feasibility = KneeGroundFeasibility.AtRender(g, raw, Vector3.right); KneeGroundFeasibility.AssessFixed(ref s, g); t.Observe(s, p, r); }
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 1000; i++) { s.Feasibility = KneeGroundFeasibility.AtRender(g, raw, Vector3.right); KneeGroundFeasibility.AssessFixed(ref s, g); t.Observe(s, p, r); }
            Check.That(GC.GetAllocatedBytesForCurrentThread() == before, "no heap allocation outside cold text formatting");
        }
    }
}

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        internal static void RunFeasibilityPublicationContracts() => Check.Run("actual Render publication holds optional height evidence without changing packet authority", () =>
        {
            var f = Create(); var source = Source(201, 1f + 1f / 60f); f.Render(source, f.Input(source));
            var step = Step(101, 201, source.Time);
            Check.That(f.Read(step, source, out var p) && p.Sample.Feasibility.Valid, "real producer computes envelope");
            float ceiling = p.Sample.Feasibility.CeilingMin;
            var changed = f.Input(source); changed.Hip.y += .1f; f.Render(source, changed);
            Check.That(f.Read(step, source, out p) && p.Sample.Feasibility.CeilingMin == ceiling, "held source never recomputes envelope");
            f._kneeFootGroundPacket.Sample.Feasibility.CeilingMin = float.NaN;
            Check.That(f.Read(step, source, out p), "optional invalid evidence cannot reject physical packet");
        });
    }
    internal sealed partial class SingleLegShadowChain
    {
        internal static void RunFeasibilityEntryContracts() => Check.Run("real Knee entry ignores missing diagnostic proof and retains trace through stop", () =>
        {
            foreach (bool b in new[] { false, true })
            {
                var f = KneeGroundTrajectoryContracts.Example(); var a = Joined(b); var missing = Joined(b);
                foreach (var leg in new[] { a, missing })
                {
                    leg._traceEnabled = true; leg._dynamicScope = PelvisRunScope.DynamicLegHipAnchorCouplingContact;
                    leg._kneeReference.Reset(true, f.Previous, f.W); leg._sampledTargetKneeRelative = f.Goal;
                    leg._sampledTargetKneeRelativeAngularVelocityLocal = f.SourceW; leg._footFoundation.Available = true;
                    leg._footFoundation.Packet = KneeGroundTrajectoryContracts.Packet(f, leg._runtimeStep, leg._runtimeSource);
                    leg._footFoundation.Packet.Input.AllowExtension = true;
                    var s = leg._footFoundation.Packet.Sample; s.AllowExtension = true;
                    s.Foot.Valid = true; s.GoalMin = KneeGroundTarget.Minimum(f.Geometry, s.Goal, f.Foot);
                    s.Feasibility = KneeGroundFeasibility.AtRender(f.Geometry, s.Raw, Vector3.right);
                    leg._footFoundation.Packet.Sample = s;
                }
                missing._footFoundation.Packet.Sample.Feasibility = default;
                a.ExpectStep("normal diagnostic"); missing.ExpectStep("missing optional proof");
                Check.That(a._controlTargetKneeRelative.Equals(missing._controlTargetKneeRelative) &&
                    a._kneeReference.Current.W.Equals(missing._kneeReference.Current.W) && a.KneeTargetRelativeA.Equals(missing.KneeTargetRelativeA),
                    "both arms preserve exact physical R/W/A when diagnostic proof is absent");
                Check.That(a._kneeGroundSample.FeasibilityStage != KneeGroundFeasibility.Stage.Unavailable, "actual entry assesses final plan");
                string saved = a._kneeGroundFeasibilityTrace.Summary(); a.DisableKneeReferencePreserveTrace();
                Check.That(saved == a._kneeGroundFeasibilityTrace.Summary(), "stop preserves evidence after clearing live packet");
                missing.ResetKneeGroundTrace();
                Check.That(missing._kneeReference.Current.Valid && missing._footFoundation.Packet.Published, "trace reset does not erase live reference or packet");
            }
        });
    }
}
