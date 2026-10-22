using System;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Validates delivery, not controller time integration. A new binding may join at
    // any sequence; thereafter every source/physics entry is consumed exactly once.
    // Drive/trace changes never reset this cursor or the Runtime clock.
    internal sealed class StepContextCursor
    {
        internal SourceStepContext Source { get; private set; }
        internal PhysicsStepContext Fixed { get; private set; }

        internal void AcceptSource(SourceStepContext source)
        {
            if (!source.Valid || (Fixed.Valid && source.Epoch != Fixed.Epoch) ||
                (Source.Valid && (source.Epoch != Source.Epoch || source.Sequence != Source.Sequence + 1 || source.Time < Source.Time)))
                throw new InvalidOperationException("SOURCE_CONTEXT_INVALID_REUSED_OR_SKIPPED");
            Source = source;
        }

        internal void AcceptFixed(PhysicsStepContext step)
        {
            if (!step.Valid || (Fixed.Valid && (step.Epoch != Fixed.Epoch || step.Sequence != Fixed.Sequence + 1 || step.FixedTime < Fixed.FixedTime)) ||
                (Source.Valid && !SourceMatches(step)))
                throw new InvalidOperationException("PHYSICS_CONTEXT_INVALID_REUSED_SKIPPED_OR_SOURCE_MISMATCH");
            Fixed = step;
        }

        internal bool SourceMatches(PhysicsStepContext step) => Source.Valid && step.Valid &&
            Source.Epoch == step.Epoch && Source.Sequence == step.SourceSequence;
    }
}
