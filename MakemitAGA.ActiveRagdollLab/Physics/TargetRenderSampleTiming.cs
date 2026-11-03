namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // The sample identity is produced by the render owner; this state owns only
    // its Fixed consumer cursor and the age/freshness published before measurement.
    // It does not advance source/Fixed clocks or sample Time itself.
    internal sealed class TargetRenderSampleTiming
    {
        private int _lastConsumedSequence;
        private int _heldDepth;

        internal int FixedSequence { get; private set; }
        internal int SourceSequence { get; private set; }
        internal bool Fresh { get; private set; }
        internal float AgeSeconds { get; private set; }
        internal int HeldDepth => _heldDepth;

        internal void ResetForDrive(int sourceSequence)
        {
            _heldDepth = 0;
            _lastConsumedSequence = sourceSequence;
            // The previous published age/freshness stays visible until Observe,
            // matching the original drive reset (which did not rewrite the report).
        }

        // Freshness/depth changes precede the caller's conditional clock read.
        // The published packet remains unchanged if that original read fails.
        internal bool ObserveSourceSequence(int sourceSequence)
        {
            bool fresh = sourceSequence != _lastConsumedSequence;
            if (fresh) _heldDepth = 0;
            else _heldDepth++;
            return fresh;
        }

        internal void PublishBeforeBodyMeasurement(int fixedSequence, int sourceSequence,
            bool fresh, float ageSeconds)
        {
            FixedSequence = fixedSequence;
            SourceSequence = sourceSequence;
            Fresh = fresh;
            AgeSeconds = ageSeconds;
        }

        // Early returns must not consume the source packet. Preserve the original
        // commit point, which precedes the final post-drive finite-state check.
        internal void CompleteControlStep(int sourceSequence) => _lastConsumedSequence = sourceSequence;
    }
}
