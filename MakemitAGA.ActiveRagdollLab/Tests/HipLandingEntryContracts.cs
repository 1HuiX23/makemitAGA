using ARL.Contracts;
using UnityEngine;
namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        internal static void RunHipLandingEntryContracts()
        {
            Check.Run("v143 both actual entries retain v142 A Hip and Knee over recorded Hip-positive witnesses", HipEntry);
            Check.Run("v143 recursive Hip baseline remains identical between observation arms and across stale bypass", HipTiming);
        }
        private static void HipTiming()
        {
            var f = HipLandingWitnessFixtures.All()[0]; int total = 0, stale = 0;
            foreach (int hz in new[] { 20, 40, 60 })
            {
                var a = LandingLeg(false, f); var b = LandingLeg(true, f); int lastRender = 0;
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
                        l.LandingHipStep(f); l.ExpectStep("recursive actual Hip entry");
                    }
                    lastRender = render; var x = a._hipNormalReference; var y = b._hipNormalReference;
                    Check.That(x.Rotation.Equals(y.Rotation) && x.W.Equals(y.W) && x.Current.A.Equals(y.Current.A) &&
                        a._controlTargetHipRelative.Equals(b._controlTargetHipRelative) &&
                        a._legAngularAccelerationFeedforward.HipAppliedWorld.Equals(b._legAngularAccelerationFeedforward.HipAppliedWorld), "trace does not alter final Hip or FF");
                    if (y.LandingPreparation.Applied) total++;
                    if (!y.Current.Applied) { stale++; Check.That(!y.LandingPreparation.Applied, "stale Hip bypass never prepares"); }
                    else
                    {
                        Check.That(y.Current.A.magnitude <= y.Current.MaxA + .002f && y.W.magnitude <= System.MathF.Max(y.Current.MaxW, y.Current.PreviousW.magnitude) + .0001f, "recursive Hip budget");
                        Check.That(KneeLandingPreparationContracts.Angle(y.Rotation, FootRelativeReference.Advance(y.Current.PreviousR, y.W, y.Current.Step.Dt)) < .0005f, "recursive Hip one-step integral");
                    }
                    if (n == 25) b._hipNormalReference.ResetTrace();
                }
            }
            Check.That(total == 0 && stale > 0, "neither formal arm adopts Hip on held/refreshed sources or stale bypass");
        }
        private static void HipEntry()
        {
            int positiveWitnesses = 0;
            foreach (var f in HipLandingWitnessFixtures.All())
            {
                if (HipLandingPreparationContracts.Eval(f).Better) positiveWitnesses++;
                var a = LandingLeg(false, f); var b = LandingLeg(true, f);
                a.LandingHipStep(f); b.LandingHipStep(f);
                a.ExpectStep("143 A full observation"); b.ExpectStep("143 B compact observation");
                Check.That(!a.HipLandingPreparationSelected && !b.HipLandingPreparationSelected &&
                    !a._hipNormalReference.LandingPreparation.Applied && !b._hipNormalReference.LandingPreparation.Applied,
                    "formal arms both keep baseline even where the old Hip candidate is positive");
                Check.That(a._kneeReference.Current.R.Equals(b._kneeReference.Current.R) &&
                    a._kneeReference.Current.W.Equals(b._kneeReference.Current.W) &&
                    a._kneeReference.Current.A.Equals(b._kneeReference.Current.A) &&
                    a._hipNormalReference.Rotation.Equals(b._hipNormalReference.Rotation) &&
                    a._hipNormalReference.W.Equals(b._hipNormalReference.W) &&
                    a._hipNormalReference.Current.A.Equals(b._hipNormalReference.Current.A),
                    "the actual committed joint commands agree");
                Check.That(a._legAngularAccelerationFeedforward.HipAppliedWorld.Equals(b._legAngularAccelerationFeedforward.HipAppliedWorld) &&
                    a._legAngularAccelerationFeedforward.KneeAppliedSigned == b._legAngularAccelerationFeedforward.KneeAppliedSigned &&
                    !a._landingPreparationCommand.HipApplied && !b._landingPreparationCommand.HipApplied,
                    "published feedforward and adopted receipt agree");
            }
            Check.That(positiveWitnesses > 0, "uses actual historical positive counterexamples");
        }
    }
}
