using System;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        internal static void RunKneeLandingEntryContracts()
        {
            Check.Run("v139 actual leg entry selects common Knee background and publishes chosen RWA to original FF", LandingActualEntry);
            Check.Run("v139 actual entry remains trace-independent at 20/40/60 Hz, held/stale, stop and restart", LandingActualTiming);
            Check.Run("v139 actual entry refuses foreign owner, missing policy and obsolete Hip state", LandingActualGuards);
        }
        private static SingleLegShadowChain LandingLeg(bool b, KneeLandingWitnessFixtures.Fixture f)
        {
            var l = Joined(b); l._dynamicScope = PelvisRunScope.DynamicLegHipAnchorCouplingContact;
            l._joinStep = f.Packet.Input.CompletedStep; l._runtimeStep = f.Knee.Step; l._runtimeSource = f.Knee.Source;
            l._physicalParent = Parent(l._runtimeStep); l._traceEnabled = true;
            l._targetSampleTiming.AgeSeconds = f.Knee.Step.UnscaledTime - f.Knee.Source.Time;
            l._sampledTargetKneeRelative = f.Packet.Input.RawKnee; l._sampledTargetKneeRelativeAngularVelocityLocal = f.Knee.InputW;
            l._upperBody.rotation = f.Packet.Geometry.Upper; l._lowerBody.rotation = f.Packet.Geometry.Upper * f.Knee.R;
            l._kneeFlexionAxisLowerLocal = f.Packet.Input.Axis;
            l.ResetKneeReferenceForDrive(); l._kneeReference.Reset(true, f.Knee.PreviousR, f.Knee.PreviousW);
            l._hipNormalReference.Reset(true, f.Hip.PreviousR, f.Hip.PreviousW, false);
            var p = f.Packet; p.Input.ParentId = 11; p.Input.UpperId = 12; p.Input.LowerId = 13;
            p.SupportId = p.PreparationSupport.SupportId = 21;
            l._footFoundation.Packet = p; l._footFoundation.Available = true;
            return l;
        }
        private void LandingHipStep(KneeLandingWitnessFixtures.Fixture f)
        {
            _hipPublicationStep = _runtimeStep; _hipPublicationParentR = _physicalParent.Rotation; _hipPublicationParentW = _physicalParent.W;
            _hipCompositeReferenceShockAbsorber.SetEnabled(true, _hipNormalReference.Rotation, _hipNormalReference.W, null);
            bool ordinary = _targetSampleTiming.AgeSeconds <= .1001f;
            Check.That(_hipNormalReference.Step(_traceEnabled, _runtimeStep, _runtimeSource, ordinary, f.Hip.InputR, f.Hip.InputW,
                Quaternion.identity, Vector3.zero, _targetSampleTiming.AgeSeconds, .1f, f.Hip.MaxW, f.Hip.MaxA, 55f / 12f, ordinary), "real Hip step before Knee");
        }
        private static void LandingActualEntry()
        {
            foreach (var f in KneeLandingWitnessFixtures.All())
            {
                var a = LandingLeg(false, f); var b = LandingLeg(true, f);
                a.LandingHipStep(f); b.LandingHipStep(f); a.ExpectStep("v139 A actual entry"); b.ExpectStep("v139 B actual entry");
                bool right = f.Name == "RIGHT_PEAK_SAMPLED_GAIN";
                var x = a._kneeReference.Current; var y = b._kneeReference.Current;
                Check.That(x.LandingPreparation.Applied == right && y.LandingPreparation.Applied == right, "actual branch per arm and witness");
                Check.That(a.KneeLandingPreparationSelected && b.KneeLandingPreparationSelected, "latched per-arm policy");
                if (!b._hipNormalReference.LandingPreparation.Applied) Check.That(a._hipNormalReference.Rotation.Equals(b._hipNormalReference.Rotation) &&
                    a._hipNormalReference.W.Equals(b._hipNormalReference.W), "no Hip change");
                Check.That(b._controlTargetKneeRelative.Equals(y.R) && b._controlTargetKneeRelativeAngularVelocityLocal.Equals(y.W), "actual selected control publication");
                float ff = Vector3.Dot(b._upperBody.rotation * y.A, b._lowerBody.transform.TransformDirection(b._kneeFlexionAxisLowerLocal).normalized);
                Check.Near(b._legAngularAccelerationFeedforward.KneeAppliedSigned, Mathf.Clamp(ff, -20f, 20f), .00001f, "original FF consumes committed Knee derivative");
                if (!right) Check.That(x.R.Equals(y.R) && x.W.Equals(y.W) && x.A.Equals(y.A), "actual entry exact fallback");
            }
        }
        private static void LandingActualTiming()
        {
            var f = KneeLandingWitnessFixtures.All()[3];
            foreach (int hz in new[] { 20, 40, 60 })
            {
                var a = LandingLeg(true, f); var b = LandingLeg(true, f); int lastRender = 0, applied = 0, stale = 0;
                for (int n = 0; n < 150; n++)
                {
                    int render = n * hz / 60;
                    foreach (var l in new[] { a, b })
                    {
                        l._traceEnabled = l == b;
                        var step = f.Knee.Step; step.Sequence += n; step.FixedTime += n / 60f; step.UnscaledTime += n / 60f;
                        if (render > lastRender && !(n >= 40 && n <= 50))
                        {
                            var source = f.Knee.Source; source.Sequence += render; source.Time = step.UnscaledTime; source.Dt = 1f / hz;
                            var completed = step; completed.Sequence--; completed.SourceSequence = l._runtimeSource.Sequence;
                            completed.FixedTime -= step.Dt; completed.UnscaledTime -= step.Dt;
                            var p = l._footFoundation.Packet; p.Input.Source = p.Sample.Source = source;
                            p.Input.CompletedStep = p.Sample.GeometryStep = p.PreparationSupport.Step = completed;
                            p.PreparationSupport.StrongLandingApproach = n < 80; p.PreparationSupport.StrongStance = n >= 80;
                            l._runtimeSource = source; l._footFoundation.Packet = p;
                        }
                        step.SourceSequence = l._runtimeSource.Sequence; l._runtimeStep = step; l._physicalParent = Parent(step);
                        l._targetSampleTiming.AgeSeconds = step.UnscaledTime - l._runtimeSource.Time;
                        l.LandingHipStep(f); l.ExpectStep("selected timed entry Hz=" + hz);
                    }
                    lastRender = render;
                    var x = a._kneeReference.Current; var y = b._kneeReference.Current;
                    Check.That(x.R.Equals(y.R) && x.W.Equals(y.W) && x.A.Equals(y.A) && x.InputR.Equals(y.InputR) &&
                        a._controlTargetKneeRelative.Equals(b._controlTargetKneeRelative) &&
                        a._legAngularAccelerationFeedforward.KneeAppliedSigned.Equals(b._legAngularAccelerationFeedforward.KneeAppliedSigned), "trace cannot change selected trajectory or FF");
                    Check.That(y.A.magnitude < 20.002f && y.W.magnitude <= MathF.Max(y.MaxW, y.PreviousW.magnitude) + .0001f, "recursive budget");
                    Check.That(KneeLandingPreparationContracts.Angle(y.R, FootRelativeReference.Advance(y.PreviousR, y.W, y.Step.Dt)) < .0005f, "recursive integral closure");
                    if (y.LandingPreparation.Applied) applied++; if (y.Stale) { stale++; Check.That(!y.LandingPreparation.Applied, "stale is never prepared"); }
                    if (n == 25) { b._kneeReference.ResetTrace(); b.ResetKneeGroundTrace(); }
                }
                Check.That(applied > 0 && stale > 0, "actual application plus stale coverage");
                string trace = b._kneeReference.GetSummary().Split("KNEE_LANDING_PREPARATION_V041390")[1];
                b.DisableKneeReferencePreserveTrace();
                Check.That(!b.KneeLandingPreparationSelected && b._kneeReference.GetSummary().EndsWith(trace), "stop clears live and preserves diagnostic evidence");
                b.ResetKneeReferenceForDrive(); Check.That(b._kneeReference.GetSummary().Contains("applied(total-approach-stance)=0|0|0"), "restart clears previous run");
            }
        }
        private static void LandingActualGuards()
        {
            var f = KneeLandingWitnessFixtures.All()[3];
            for (int i = 0; i < 5; i++)
            {
                var l = LandingLeg(true, f); l.LandingHipStep(f);
                if (i == 0) { var p = l._footFoundation.Packet; p.Input.UpperId = 999; l._footFoundation.Packet = p; }
                if (i == 1) l._kneeLandingPreparationSelected = false;
                if (i == 2) l._hipNormalReference.DisablePreserveTrace();
                if (i == 3 || i == 4) l._dynamicScope = PelvisRunScope.DynamicLegFreeSpace;
                if (i == 4) l._traceEnabled = false;
                if (i == 1 || i == 3)
                    Check.That(!l.UpdateKneeReferenceAndFeedforward(out _), "unexpected policy/free-space tracing rejected before control");
                else { l.ExpectStep("optional unavailable preparation"); Check.That(!l._kneeReference.Current.LandingPreparation.Applied, "foreign/missing input or unloaded scope never selected"); }
            }
        }
    }
}
