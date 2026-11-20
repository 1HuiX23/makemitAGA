using System;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        internal static void RunKneeReferenceContracts()
        {
            Check.Run("real Knee/FF entry accepts first held-source preparation step for both Hip arms", FirstPreparationStep);
            Check.Run("real Knee/FF path stays continuous from preparation to traced Hip run", PreparationHandoff);
            Check.Run("Knee preparation rejects selection, native and join boundary faults", PreparationFaults);
            Check.Run("Knee clocks and current FF receipt reject stale identities", ReferenceClockFaults);
            Check.Run("Knee planned FF retains source, physical, frame and stale gates", FeedforwardProtection);
            Check.Run("both allocation arms consume joint ground input; legacy raw bypass remains", GroundAndLegacy);
            Check.Run("parent frame fixture validates point velocity and complete identity", ParentFrameContract);
            Check.Run("joint Render capture uses current native geometry and explicit completed identity", RenderInputSnapshot);
            Check.Run("v135 capture preserves completed controller Knee reference identity without relabeling it as Render", CompletedKneeSnapshot);
        }

        private static void CompletedKneeSnapshot()
        {
            foreach(bool b in new[]{false,true}) {
                var leg=Joined(b);leg._dynamicScope=PelvisRunScope.DynamicLegHipAnchorCouplingContact;
                leg.ExpectStep("complete real Knee step before Render");var completed=leg._kneeReference.Current;
                leg._runtimeSource.Sequence++;
                leg._runtimeSource.Time=leg._runtimeStep.UnscaledTime+1f/60f;
                var input=leg.CaptureKneeFootGroundInput();
                Check.That(input.HasCurrentKneeReference && input.PreferKneeProgress,"both arms capture the matching old reference and retain v135 B policy");
                Check.That(input.CompletedKneeReference.Equals(completed) && input.CompletedKneeReference.Source.Sequence < input.Source.Sequence,
                    "RWA and old source identity remain exact values, not a current-render receipt");
                leg._kneeReference.ResetTrace();var after=leg.CaptureKneeFootGroundInput();
                Check.That(after.CompletedKneeReference.Equals(completed),"trace reset cannot change captured controller state");
            }
        }

        private static PhysicsStepContext Context(int offset, int hz = 60) => new PhysicsStepContext
        {
            Epoch = 7, Sequence = 5280 + offset, SourceSequence = 5049 + Math.Max(0, offset - 1) * hz / 60,
            FixedTime = 88f + offset / 60f, Dt = 1f / 60f, UnscaledTime = 200f + offset / 60f
        };
        private static PelvisParentFrame Parent(PhysicsStepContext step)
        {
            var p = new PelvisParentFrame
            {
                Valid = true, Step = step, Epoch = step.Epoch, PhysicsSequence = step.Sequence, TraceFixed = 0,
                BodyId = 11, SourceCapture = step.SourceSequence, SourceSequence = step.SourceSequence,
                FixedTime = step.FixedTime, Dt = step.Dt, Position = new Vector3(0f, .9f, 0f),
                ComLocal = new Vector3(0f, .1f, 0f), ComVelocity = new Vector3(1f, 2f, 3f), W = new Vector3(0f, 0f, 2f),
                Rotation = Quaternion.identity, CommandPosition = new Vector3(0f, .91f, 0f),
                CommandOriginVelocity = new Vector3(2f, 0f, 0f), CommandW = new Vector3(0f, 0f, 1f),
                CommandRotation = Quaternion.identity
            };
            p.Left = p.MapHip(new Vector3(-.1f, 0f, 0f), Vector3.zero, Quaternion.identity);
            p.Right = p.MapHip(new Vector3(.1f, 0f, 0f), Vector3.zero, Quaternion.identity);
            return p;
        }
        private static SingleLegShadowChain Joined(bool b)
        {
            var leg = new SingleLegShadowChain { _dynamicParentActive = true, _driveEnabled = true, _joinStep = Context(0) };
            leg._torso.PelvisReceiverControl.Arm = b ? PelvisDynamicReceiverIsolation.DynamicHipArmB : PelvisDynamicReceiverIsolation.DynamicHipArmA;
            leg._legAngularAccelerationFeedforward.SetEnabled(true);
            leg.ResetKneeReferenceForDrive();
            leg._sampledTargetKneeRelative = Quaternion.AngleAxis(30f, Vector3.right);
            leg.SetContext(1);
            return leg;
        }
        private void SetContext(int offset, int hz = 60)
        {
            _runtimeStep = Context(offset, hz);
            _runtimeSource = new SourceStepContext { Epoch = _runtimeStep.Epoch, Sequence = _runtimeStep.SourceSequence,
                Time = 200f + (_runtimeStep.SourceSequence - 5049) / (float)hz, Dt = 1f / hz };
            _physicalParent = Parent(_runtimeStep);
            _targetSampleTiming.AgeSeconds = _runtimeStep.UnscaledTime - _runtimeSource.Time;
        }
        private void ExpectStep(string label)
        {
            Check.That(UpdateKneeReferenceAndFeedforward(out string issue), label + ": " + issue);
            Check.That(LegFeedforwardGatesSatisfied(), label + " recovery invariant");
        }
        private static void FirstPreparationStep()
        {
            foreach (bool b in new[] { false, true })
            {
                var leg = Joined(b); leg.ExpectStep("first post-join Fixed, arm B=" + b);
                Check.That(leg._runtimeStep.Sequence == 5281 && leg._runtimeStep.SourceSequence == 5049,
                    "logged first solve uses the unchanged render packet");
                Check.That(leg._kneeReference.Current.Matches(leg._runtimeStep, leg._runtimeSource) && leg._testFeedforwardCalls == 1,
                    "real reference update reaches real FF evaluation in this solve");
                Check.That(leg._kneeReference.Current.A.magnitude <= 20.002f && leg._legAngularAccelerationFeedforward.KneeActiveThisStep,
                    "preparation uses the bounded reference acceleration");
                Check.That(leg._dynamicScope == PelvisRunScope.DynamicLegFreeSpace && !leg._traceEnabled &&
                    leg.FreeSpaceSelectorsOff && leg.FreeSpaceNativeReady, "reference execution changes no contact/load scope");
            }
        }
        private static HipContactPreparationSample QuietSample() => new HipContactPreparationSample
        {
            NativeOff = true, IdentityQualified = true,
            Geometry = new InitialGeometryPreflight { Valid = true, Calibrated = true, FootprintsInside = true,
                RawMin = -.0026f, ControlMin = -.0027f, ActualMin = -.0027f },
            Motion = new FootContactMotion { Valid = true, FullFootprint = true, Min = -.0027f, PredictedMin = -.0027f }
        };
        private static void PreparationHandoff()
        {
            foreach (bool b in new[] { false, true }) foreach (int hz in new[] { 20, 40, 60 })
            {
                var leg = Joined(b); var window = new HipContactPreparationWindow(); window.Begin(leg._joinStep);
                var previous = default(KneeReferenceState); int n = 1;
                for (; n < 40 && !window.Qualified; n++)
                {
                    leg.SetContext(n, hz); leg.ExpectStep("preparation at source Hz=" + hz);
                    var current = leg._kneeReference.Current;
                    if (previous.Valid) AssertContinuous(previous, current);
                    previous = current;
                    window.Observe(leg._runtimeStep, leg._runtimeSource, QuietSample(), QuietSample(), null);
                }
                Check.That(window.QualifiedFor(leg._runtimeStep) && n > 9, "actual preparation window owns the current receipt");
                // A qualified native sample is injected; this suite does not simulate a
                // solve or claim that scope selection/PhysX passed in the real game.
                leg._dynamicScope = PelvisRunScope.DynamicLegHipAnchorCouplingContact;
                leg._testSelectorsOff = leg._testNativeOff = false;
                leg._traceEnabled = true;
                leg._kneeReference.ResetTrace(); leg.ResetKneeGroundTrace();
                leg._legAngularAccelerationFeedforward.ResetDiagnostics();
                for (int end = n + 3; n < end; n++)
                {
                    leg.SetContext(n, hz); leg.ExpectStep("traced Hip handoff");
                    AssertContinuous(previous, leg._kneeReference.Current); previous = leg._kneeReference.Current;
                }
                string trace = leg._kneeReference.GetSummary();
                leg.DisableKneeReferencePreserveTrace();
                Check.That(!leg.KneeReferenceSelected && !leg.KneePlannedFeedforwardSelected && !leg.KneeGroundTargetSelected && !leg.KneeGroundExtensionSelected &&
                    !leg._kneeReference.Current.Valid, "stop removes live selection and current receipt");
                Check.That(leg._kneeReference.GetSummary().Contains("/steps-held-stale-alphaLimited-speedLimited=3|") &&
                    trace.Contains("/steps-held-stale-alphaLimited-speedLimited=3|"), "stop retains only formal trace statistics");
                // A new drive explicitly seeds anew; a trace transition above did not.
                leg._dynamicScope = PelvisRunScope.DynamicLegFreeSpace; leg._traceEnabled = false;
                leg._testSelectorsOff = leg._testNativeOff = true; leg._joinStep = leg._runtimeStep;
                leg.ResetKneeReferenceForDrive(); leg.SetContext(n, hz); leg.ExpectStep("restart after stop");
            }
        }
        private static void AssertContinuous(KneeReferenceState before, KneeReferenceState after)
        {
            Check.Near(Quaternion.Angle(before.R, after.PreviousR), 0f, .001f, "handoff retains the consumed R");
            Check.Near((before.W - after.PreviousW).magnitude, 0f, 1e-6f, "handoff retains the consumed W");
            Check.That(after.Step.Sequence == before.Step.Sequence + 1 && after.A.magnitude <= 20.002f,
                "same shared clock and acceleration budget through trace start");
        }
        private static void PreparationFaults()
        {
            Action<SingleLegShadowChain>[] faults = {
                l => l._traceEnabled = true, l => l._testSelectorsOff = false, l => l._testNativeOff = false,
                l => l._dynamicScope = PelvisRunScope.DynamicLegUnloadedContact,
                l => l._joinStep = default, l => l._joinStep.Epoch++, l => l._runtimeStep = l._joinStep,
                l => l._driveEnabled = false, l => l._dynamicJoinPending = true,
                l => l._kneeGroundExtensionSelected = !l._kneeGroundExtensionSelected,
                l => l._kneePlannedFeedforwardSelected = false, l => l._kneeGroundTargetSelected = !l._kneeGroundTargetSelected,
                l => l._torso.PelvisReceiverControl.Arm = "LEGACY", l => l._runtimeStep.SourceSequence--
            };
            foreach (bool b in new[] { false, true }) for (int i = 0; i < faults.Length; i++)
            {
                var leg = Joined(b); faults[i](leg);
                Check.That(!leg.UpdateKneeReferenceAndFeedforward(out string issue) &&
                    issue.StartsWith("KNEE_REFERENCE_INVALID:") && leg._testFeedforwardCalls == 0,
                    "boundary fault rejected before FF, arm/fault=" + b + "/" + i);
            }
        }
        private static void ReferenceClockFaults()
        {
            for (int fault = 0; fault < 5; fault++)
            {
                var leg = Joined(true); leg.ExpectStep("before clock fault");
                if (fault != 0) leg.SetContext(fault == 1 ? 3 : 2);
                if (fault == 2) { leg._runtimeSource.Sequence = 5049; leg._runtimeStep.SourceSequence = 5049; leg._runtimeSource.Time += .01f; }
                if (fault == 3) leg._runtimeSource.Sequence++;
                if (fault == 4) leg._sampledTargetKneeRelative = new Quaternion(float.NaN, 0f, 0f, 1f);
                Check.That(!leg.UpdateKneeReferenceAndFeedforward(out _) && leg._testFeedforwardCalls == 1,
                    "duplicate/skipped/changed source/mismatched source/nonfinite rejected before new FF: " + fault);
            }
            var missing = Joined(true);
            Check.That(!missing.UpdateLegAngularAccelerationFeedforward(), "no seed-only reference may act as a same-step FF receipt");
            missing.ExpectStep("before obsolete FF receipt"); missing.SetContext(2);
            Check.That(!missing.UpdateLegAngularAccelerationFeedforward(), "prior reference may not feed a new solve");
        }
        private static void FeedforwardProtection()
        {
            for (int gate = 0; gate < 7; gate++)
            {
                var leg = Joined(true); leg._torso.ControlPhase = new TorsoControlPhase(true);
                if (gate == 1) { leg._torso.ControlPhase = new TorsoControlPhase(false); leg._torso.ControlRecoveryActive = true; }
                if (gate == 2) leg._hipAuthority.PhysicalRecoveryPending = true;
                if (gate == 3) leg._softKneeAuthority.PhysicalRecoveryPending = true;
                if (gate == 4) leg._hipControlFrame.Active = true;
                if (gate == 5) leg._targetSampleTiming.AgeSeconds = .2f;
                if (gate == 6) { leg._testGoalBridge = true; leg._torso.ControlPolicy.HipGoalGateOnlyHip = false; }
                leg.ExpectStep("protected preparation " + gate);
                Check.That(!leg._legAngularAccelerationFeedforward.HipActiveThisStep &&
                    leg._legAngularAccelerationFeedforward.KneeActiveThisStep == (gate < 2),
                    "only the original planned-source/torso exemption remains: " + gate);
            }
        }
        private static void GroundAndLegacy()
        {
            foreach (bool b in new[] { false, true })
            {
                var leg = Joined(b); leg._footFoundation.Available = true;
                leg._sampledTargetKneeRelative = Quaternion.AngleAxis(-60f, Vector3.right);
                leg._controlTargetHipRelative = Quaternion.AngleAxis(30f, Vector3.right);
                var g = new KneeGroundTarget.Geometry {
                    Hip = new Vector3(0f, .9f, 0f), Upper = Quaternion.AngleAxis(30f, Vector3.right),
                    UpperSpan = new Vector3(0f, -.45f, 0f), FootBindRelative = Quaternion.AngleAxis(30f, Vector3.right),
                    LowerSpan = new Vector3(0f, -.45f, 0f), FootRelative = Quaternion.AngleAxis(30f, Vector3.right),
                    Center = new Vector3(0f, -.1f, 0f), Half = new Vector3(.05f, .025f, .08f),
                    FloorCenter = new Vector3(0f, -.05f, 0f), FloorHalf = new Vector3(3f, .05f, 3f)
                };
                var observed = leg._runtimeStep; observed.Sequence--; observed.SourceSequence--;
                observed.UnscaledTime = leg._runtimeSource.Time - 1f / 60f;
                observed.FixedTime -= 1f / 60f;
                var input = new KneeFootGroundInput { Enabled = true, Apply = true, AllowExtension = true, Source = leg._runtimeSource,
                    CompletedStep = observed, ParentId = 11, UpperId = 12, LowerId = 13,
                    Hip = g.Hip, Upper = g.Upper, UpperSpan = g.UpperSpan,
                    RawKnee = leg._sampledTargetKneeRelative, Axis = Vector3.right, LowerKneeAnchor = Vector3.zero };
                var sample = KneeGroundTarget.Evaluate(g, input.RawKnee, input.Axis, input.AllowExtension);
                sample.Source = input.Source; sample.GeometryStep = observed;
                sample.Applied = sample.Candidate; sample.PlannedFoot = KneeGroundTarget.SelectedFoot(g, sample.Foot);
                leg._footFoundation.Packet = new KneeFootGroundPacket
                    { Input = input, Geometry = g, Sample = sample, SupportId = 21, Published = true };
                leg.ExpectStep("real ground input consumption");
                Check.That(leg._kneeGroundSample.Valid && leg._kneeGroundSample.Candidate && leg._kneeGroundSample.Planned &&
                    leg._kneeGroundSample.Applied && float.IsFinite(leg._kneeGroundSample.PlanMin),
                    "candidate, applied arm and consumed planned geometry remain distinct");
            }
            foreach (PelvisRunScope scope in Enum.GetValues<PelvisRunScope>())
            {
                var leg = Joined(false); leg._torso.PelvisReceiverControl.Arm = "LEGACY";
                leg._dynamicScope = scope; leg._traceEnabled = true; leg.ResetKneeReferenceForDrive();
                leg._sampledTargetKneeRelativeAngularAccelerationLocal = new Vector3(7f, 0f, 0f);
                leg.ExpectStep("legacy unselected path");
                Check.That(!leg.KneeReferenceSelected && !leg.KneePlannedFeedforwardSelected && !leg.KneeGroundTargetSelected,
                    "unselected arm never receives the preparation exemption");
                Check.Near(leg._legAngularAccelerationFeedforward.KneeAppliedSigned, 7f, 1e-5f, "legacy held raw A remains the input");
            }
        }
        private static void ParentFrameContract()
        {
            var p = Parent(Context(1));
            Check.That(p.Finite() && p.Matches(Context(1)) && p.Matches(7, 11, 5281, p.FixedTime, p.Dt), "fully populated physical parent frame");
            Check.Near((p.SettledPointVelocity(Vector3.zero) - new Vector3(1.2f, 2f, 3f)).magnitude, 0f, 1e-6f,
                "point velocity uses COM offset, not origin velocity");
            Check.Near((p.Left.CommandPoint - p.TargetPoint(p.Left.LocalAnchor)).magnitude, 0f, 1e-6f, "mapped Hip command shares the same parent");
            p.Rotation = default; Check.That(!p.Finite(), "default all-zero rotation is not a native sample");
        }

        private static void RenderInputSnapshot()
        {
            foreach (bool b in new[] { false, true })
            {
                var leg = Joined(b); leg._dynamicScope = PelvisRunScope.DynamicLegHipAnchorCouplingContact;
                leg._runtimeSource = new SourceStepContext { Epoch = 7, Sequence = leg._runtimeStep.SourceSequence + 1,
                    Time = leg._runtimeStep.UnscaledTime + 1f / 60f, Dt = 1f / 60f };
                leg._pelvisBody.position = new Vector3(0f, .9f, 0f);
                leg._pelvisBody.rotation = Quaternion.AngleAxis(90f, Vector3.up);
                leg._rawHipConnectedAnchorLocal = new Vector3(.1f, .01f, 0f);
                leg._upperBody.rotation = Quaternion.AngleAxis(30f, Vector3.right);
                leg._controlTargetHipRelative = Quaternion.AngleAxis(70f, Vector3.right);
                var input = leg.CaptureKneeFootGroundInput();
                Check.That(input.Matches(leg._runtimeSource) && input.Apply, "selection and identity captured independent of trace");
                Check.Near((input.Hip - new Vector3(0f, .91f, -.1f)).magnitude, 0f, .00001f, "native parent position/rotation form the Hip point");
                Check.Near(Quaternion.Angle(input.Upper, leg._upperBody.rotation), 0f, .001f, "uses current Upper native pose, not a cached diagnostic/control target");
                leg._dynamicJoinPending = true;
                Check.That(!leg.CaptureKneeFootGroundInput().Enabled, "pending join cannot publish optional input");
            }
        }
    }
}
