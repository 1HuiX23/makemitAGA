// Only native snapshot reads surrounding the linked production input entry are
// substituted. No pose integration, contacts, forces or duplicate planner here.
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private readonly FootGroundReference _groundReference = new FootGroundReference();
        private readonly FootRelativeReference _relativeReference = new FootRelativeReference();
        private readonly DrivePort _ankleDrive = new DrivePort();
        private readonly ReportPort _report = new ReportPort();
        private readonly SurfacePort _supportSurface = new SurfacePort();
        private readonly BodyPort _lowerBody = new BodyPort();
        private readonly BoxPort _collider = new BoxPort();
        private readonly Vector3 _lowerAnkleAnchorLocal = new Vector3(0f, -.45f, 0f);
        private readonly Vector3 _footAnkleAnchorLocal = Vector3.zero;
        private readonly Quaternion _bindSourceRelative = Quaternion.AngleAxis(30f, Vector3.right);
        private PreviousFootSupport _previousReferenceSupport;
        private bool _traceEnabled = true;
        private static bool IsFinite(Vector3 v) => PelvisParentFrame.V(v);
        private sealed class DrivePort { internal bool DynamicActive = true; }
        private sealed class ReportPort { internal PhysicsStepContext RuntimeContext; }
        private sealed class PosePort { internal Vector3 lossyScale = new Vector3(1f, 1f, 1f); }
        private sealed class BodyPort
        {
            internal readonly PosePort transform = new PosePort();
            internal Vector3 position;
            internal Quaternion rotation = Quaternion.AngleAxis(-30f, Vector3.right);
            internal int GetInstanceID() => 13;
        }
        private sealed class BoxPort
        {
            internal readonly PosePort transform = new PosePort();
            internal Vector3 center = new Vector3(0f, -.1f, 0f), size = new Vector3(.1f, .05f, .16f);
        }
        private sealed class SurfacePort
        {
            internal int SelectedId = 21;
            internal bool Available = true;
            internal bool TryGetEntryBox(int id, out Vector3 center, out Vector3 half)
            { center = new Vector3(0f, -.05f, 0f); half = new Vector3(3f, .05f, 3f); return Available && id == SelectedId; }
        }

        internal static void RunReferenceInputContracts()
        {
            Check.Run("production Render publishes one bounded Knee Foot packet after Foot RWA", JointPublication);
            Check.Run("A Foot reference is identical with joint observation enabled or absent", BaselineUnchanged);
            Check.Run("held Render and held Fixed consumers do not rewrite a joint packet", HeldPacket);
            Check.Run("joint packet rejects unpublished stale owner epoch support and time mismatches", PacketRejection);
            Check.Run("optional geometry loss falls back and invalid Foot reference cannot publish", FallbackAndFailure);
            Check.Run("joint footprint outside support preserves the independent Foot fallback", OutsideFootprintFallback);
            Check.Run("20 40 60 Hz Render keeps both existing planner clocks and continuous handoff", MultipleRates);
        }
        private static PhysicsStepContext Step(int fixedId, int sourceId, float time) =>
            new PhysicsStepContext { Epoch = 7, Sequence = fixedId, SourceSequence = sourceId, FixedTime = time, UnscaledTime = time, Dt = 1f / 60f };
        private static SourceStepContext Source(int id, float time, float dt = 1f / 60f) =>
            new SourceStepContext { Epoch = 7, Sequence = id, Time = time, Dt = dt };
        private static FootAnkleProxyFoundation Create()
        {
            var f = new FootAnkleProxyFoundation();
            f._report.RuntimeContext = Step(100, 200, 1f);
            f._previousReferenceSupport = new PreviousFootSupport
                { Step = f._report.RuntimeContext, SupportId = 21, StrongStance = true, SourceAge = 0f };
            f._lowerBody.position = new Vector3(0f, .9f, 0f) + Quaternion.AngleAxis(30f, Vector3.right) * new Vector3(0f, -.45f, 0f);
            f._groundReference.Select(true, true, true);
            f._groundReference.SeedInput(Source(200, 1f), f._bindSourceRelative, Vector3.zero);
            f._relativeReference.Select(true); f._relativeReference.Seed(Source(200, 1f), f._bindSourceRelative, Vector3.zero);
            return f;
        }
        private KneeFootGroundInput Input(SourceStepContext source, bool apply = true) => new KneeFootGroundInput
        {
            Enabled = true, Apply = apply, Source = source, CompletedStep = _report.RuntimeContext,
            ParentId = 11, UpperId = 12, LowerId = 13,
            Hip = new Vector3(0f, .9f, 0f), Upper = Quaternion.AngleAxis(30f, Vector3.right),
            UpperSpan = new Vector3(0f, -.45f, 0f), LowerKneeAnchor = Vector3.zero,
            RawKnee = Quaternion.AngleAxis(-60f, Vector3.right), Axis = Vector3.right
        };
        private void Render(SourceStepContext source, KneeFootGroundInput input) =>
            ObserveFootReference(source, _bindSourceRelative, Vector3.zero, source.Time - _relativeReference.Current.Source.Time, input);
        private bool Read(PhysicsStepContext step, SourceStepContext source, out KneeFootGroundPacket packet) =>
            TryReadKneeFootGroundPacket(step, source, 11, 12, 13, out packet, out _);

        private static void JointPublication()
        {
            var f = Create(); var source = Source(201, 1f + 1f / 60f);
            f.Render(source, f.Input(source));
            Check.That(f.Read(Step(101, 201, source.Time), source, out var p), "actual producer packet is readable on next Fixed");
            Check.That(p.Sample.Candidate && p.Sample.Applied && p.Sample.GoalMin >= 0f, "bounded coupled candidate selected");
            Check.Near(Quaternion.Angle(p.Sample.PlannedFoot, f._relativeReference.Current.Rotation), 0f, .001f, "records consumed planner output");
            Check.That(f._relativeReference.Current.A.magnitude <= 20.002f && f._relativeReference.Current.W.magnitude <= 3.7501f,
                "joint geometry does not bypass the existing Foot acceleration or speed budget");
            Check.That(p.Sample.GeometryStep.Matches(f._report.RuntimeContext) && p.Sample.Source.Matches(source),
                "completed control identity and new Render source remain separate");
        }
        private static void BaselineUnchanged()
        {
            var observed = Create(); var original = Create();
            for (int n = 1; n <= 60; n++)
            {
                var source = Source(200 + n, 1f + n / 60f);
                var step = Step(99 + n, 199 + n, 1f + (n - 1) / 60f);
                observed._report.RuntimeContext = original._report.RuntimeContext = step;
                observed._previousReferenceSupport.Step = original._previousReferenceSupport.Step = step;
                observed.Render(source, observed.Input(source, false)); original.Render(source, default);
                var a = observed._relativeReference.Current; var b = original._relativeReference.Current;
                Check.That(a.Rotation.x == b.Rotation.x && a.Rotation.y == b.Rotation.y && a.Rotation.z == b.Rotation.z && a.Rotation.w == b.Rotation.w &&
                    (a.W - b.W).sqrMagnitude == 0f && (a.A - b.A).sqrMagnitude == 0f, "A observer changes no Foot R W A bits");
                Check.That(!observed._kneeFootGroundPacket.Sample.Applied, "A never applies a Knee candidate");
            }
        }
        private static void HeldPacket()
        {
            var f = Create(); var source = Source(201, 1f + 1f / 60f);
            f.Render(source, f.Input(source)); var before = f._kneeFootGroundPacket;
            var changed = f.Input(source); changed.Hip = new Vector3(0f, 4f, 0f);
            f.Render(source, changed); f._groundReference.ResetTrace(); f._relativeReference.ResetTrace();
            Check.That(f._kneeFootGroundPacket.Published && f._kneeFootGroundPacket.Geometry.Hip.y == before.Geometry.Hip.y,
                "same source and trace reset preserve the exact held geometry");
            for (int n = 1; n <= 3; n++)
            {
                Check.That(f.Read(Step(100 + n, 201, 1f + n / 60f), source, out var held), "held Fixed packet remains usable inside lease");
                Check.Near(Quaternion.Angle(held.Sample.PlannedFoot, before.Sample.PlannedFoot), 0f, .001f, "Fixed read never advances Foot R");
            }
            f.ClearKneeFootGroundPacket();
            Check.That(!f.Read(Step(104, 201, 1f + 4f / 60f), source, out _), "disable cannot resurrect an old goal");
        }
        private static void PacketRejection()
        {
            var f = Create(); var source = Source(201, 1f + 1f / 60f); f.Render(source, f.Input(source));
            var p = f._kneeFootGroundPacket; var step = Step(101, 201, source.Time);
            Check.That(!p.Matches(step, source, 99, 12, 13, 21) && !p.Matches(step, source, 11, 99, 13, 21) &&
                !p.Matches(step, source, 11, 12, 99, 21) && !p.Matches(step, source, 11, 12, 13, 99), "all owners and support must match");
            p.Published = false; Check.That(!p.Matches(step, source, 11, 12, 13, 21), "unpublished pair refused"); p.Published = true;
            step.Epoch++; Check.That(!p.Matches(step, source, 11, 12, 13, 21), "cross epoch refused"); step.Epoch--;
            step.Sequence = 100; Check.That(!p.Matches(step, source, 11, 12, 13, 21), "pre-publication Fixed refused"); step.Sequence++;
            var different = source; different.Time += .001f;
            Check.That(!p.Matches(step, different, 11, 12, 13, 21), "same number is not same source payload identity");
            step.UnscaledTime = 1.2f; Check.That(!p.Matches(step, source, 11, 12, 13, 21), "old native geometry expires even with held source");
        }
        private static void FallbackAndFailure()
        {
            foreach (int reason in new[] { 0, 1, 2, 3, 4, 5 })
            {
                var f = Create(); var source = Source(201, 1f + 1f / 60f); var input = f.Input(source);
                if (reason == 0) f._previousReferenceSupport.StrongStance = false;
                if (reason == 1) f._previousReferenceSupport.SupportId++;
                if (reason == 2) f._previousReferenceSupport.SourceAge = .2f;
                if (reason == 3) input.Hip = new Vector3(float.NaN, .9f, 0f);
                if (reason == 4) input.LowerId++;
                if (reason == 5) f._supportSurface.Available = false;
                f.Render(source, input);
                Check.That(!f.Read(Step(101, 201, source.Time), source, out _), "unqualified geometry never supplies Knee input");
                Check.That(f._relativeReference.Finite && f._relativeReference.Current.Source.Matches(source), "ordinary Foot input remains continuous");
            }
            var bad = Create(); var s = Source(201, 1f + 1f / 60f);
            bad.ObserveFootReference(s, default, Vector3.zero, 1f / 60f, bad.Input(s));
            Check.That(!bad._kneeFootGroundPacket.Published && !bad._relativeReference.Finite,
                "invalid Foot source cannot publish a valid-looking Knee half");
        }
        private static void OutsideFootprintFallback()
        {
            var joint = Create(); var independent = Create(); var source = Source(201, 1f + 1f / 60f);
            var outside = joint.Input(source); outside.Hip.x = 10f;
            joint.Render(source, outside); independent.Render(source, default);
            Check.That(!joint.Read(Step(101, 201, source.Time), source, out _), "outside chain cannot publish a pair");
            Check.That(joint._groundReference.Current.Valid && !joint._groundReference.Current.JointChain,
                "actual Lower still supplies its eligible ordinary Foot geometry");
            var a = joint._relativeReference.Current; var b = independent._relativeReference.Current;
            Check.That(a.Rotation.x == b.Rotation.x && a.Rotation.y == b.Rotation.y && a.Rotation.z == b.Rotation.z && a.Rotation.w == b.Rotation.w &&
                (a.W - b.W).sqrMagnitude == 0f && (a.A - b.A).sqrMagnitude == 0f,
                "unavailable joint geometry preserves exactly the original Foot fallback");
        }
        private static void MultipleRates()
        {
            foreach (int hz in new[] { 20, 40, 60 })
            {
                var f = Create(); var knee = new KneeRelativeReference(); var raw = Quaternion.AngleAxis(-60f, Vector3.right);
                knee.Reset(true, raw, Vector3.zero); var source = Source(200, 1f); int lastRender = 0;
                for (int n = 1; n <= 120; n++)
                {
                    int render = (n * hz) / 60;
                    if (render > lastRender)
                    {
                        float time = 1f + n / 60f;
                        source = Source(200 + render, time, time - source.Time);
                        f._report.RuntimeContext = Step(99 + n, f._relativeReference.Current.Source.Sequence, time - 1f / 60f);
                        f._previousReferenceSupport.Step = f._report.RuntimeContext;
                        f.Render(source, f.Input(source)); lastRender = render;
                    }
                    var step = Step(100 + n, source.Sequence, 1f + n / 60f);
                    bool read = f.Read(step, source, out var p);
                    Quaternion goal = read && p.Sample.Applied ? p.Sample.Goal : raw;
                    var previous = knee.Current;
                    Check.That(knee.Step(true, step, source, goal, Vector3.zero, step.UnscaledTime - source.Time, .1f, 40f / 9f, 20f, 5f), "Knee shared Fixed plan succeeds");
                    if (previous.Valid) Check.Near(Quaternion.Angle(previous.R, knee.Current.PreviousR), 0f, .001f, "joint source update does not reseed Knee");
                    Check.That(knee.Current.A.magnitude <= 20.002f && f._relativeReference.Current.A.magnitude <= 20.002f, "both original acceleration bounds retained");
                    if (render > 0) Check.That(read, "real published packet survives expected held Fixed steps at source Hz=" + hz);
                }
            }
        }
    }
}
