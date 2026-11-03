using MakemitAGA.ActiveRagdollLab.Physics;

namespace ARL.Contracts
{
    internal static class SampleTimingContracts
    {
        internal static void Run()
        {
            Check.Run("render sample observation and consumption boundary", Observation);
            Check.Run("drive reset preserves published timing until observation", Reset);
        }
        private static void PublishObservation(TargetRenderSampleTiming timing, int fixedSequence, int sourceSequence, float observedAge)
        {
            bool fresh = timing.ObserveSourceSequence(sourceSequence);
            // Age is supplied by the real owner's original conditional clock read.
            timing.PublishBeforeBodyMeasurement(fixedSequence, sourceSequence, fresh, observedAge);
        }
        private static void Observation()
        {
            var timing = new TargetRenderSampleTiming();
            PublishObservation(timing, 7, 3, .02f);
            Check.That(timing.FixedSequence == 7 && timing.SourceSequence == 3 && timing.Fresh && timing.HeldDepth == 0, "observation publishes both identities without consuming the source");
            Check.Near(timing.AgeSeconds, .02f, .000001f, "published age stays attached to the observed identity");
            // Simulate an early return before the original consumption commit point.
            // The production owner retains its later post-drive finite check after this commit.
            PublishObservation(timing, 8, 3, .04f);
            Check.That(timing.Fresh && timing.HeldDepth == 0, "exit before consumption leaves the render sample fresh");
            timing.CompleteControlStep(3);
            Check.That(timing.Fresh && timing.FixedSequence == 8, "completion does not retroactively change the observation projection");
            PublishObservation(timing, 9, 3, .06f);
            Check.That(!timing.Fresh && timing.HeldDepth == 1, "next solve observes a held sample after the consumption commit");
            PublishObservation(timing, 10, 3, .08f);
            Check.That(!timing.Fresh && timing.HeldDepth == 2, "held depth advances with observed solves");
            timing.ObserveSourceSequence(3); // The caller's following clock read fails; no publication occurs.
            Check.That(timing.HeldDepth == 3 && timing.FixedSequence == 10 && timing.AgeSeconds == .08f, "clock failure retains the old published packet after the original depth update");
            PublishObservation(timing, 11, 4, .01f);
            Check.That(timing.Fresh && timing.HeldDepth == 0, "new render packet resets held depth");
        }
        private static void Reset()
        {
            var timing = new TargetRenderSampleTiming();
            PublishObservation(timing, 10, 4, .25f);
            timing.ResetForDrive(4);
            Check.That(timing.FixedSequence == 10 && timing.SourceSequence == 4 && timing.Fresh && timing.AgeSeconds == .25f, "drive reset retains the previous published observation");
            Check.That(timing.HeldDepth == 0, "drive reset clears only owned hold/consume state");
            PublishObservation(timing, 11, 4, .5f);
            Check.That(!timing.Fresh && timing.HeldDepth == 1, "drive reset aligns the consumption cursor");
            timing.ResetForDrive(0);
            PublishObservation(timing, 12, 0, 0f);
            Check.That(timing.SourceSequence == 0 && timing.AgeSeconds == 0f && !timing.Fresh, "no-source observation preserves the supplied zero-age packet");
        }
    }
}
