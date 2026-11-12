using System;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Published only after the Foot controller's completion boundary. This is a
    // stance/approach witness, not contact, admitted load or solved support.
    internal struct PreviousFootSupport
    {
        internal PhysicsStepContext Step;
        internal int SupportId;
        internal bool StrongStance;
        internal bool StrongLandingApproach;
        internal float SourceAge;

        internal bool Precedes(PhysicsStepContext next) => Step.Valid && next.Valid && Step.SourceSequence > 0 &&
            Step.Epoch == next.Epoch && next.Sequence == Step.Sequence + 1 &&
            next.SourceSequence >= Step.SourceSequence && next.FixedTime >= Step.FixedTime &&
            next.UnscaledTime >= Step.UnscaledTime;

        internal bool Eligible(PhysicsStepContext next, int supportId, bool allowApproach = false) => Precedes(next) &&
            !(StrongStance && StrongLandingApproach) && (StrongStance || (allowApproach && StrongLandingApproach)) &&
            supportId != 0 && SupportId == supportId &&
            !float.IsNaN(SourceAge) && !float.IsInfinity(SourceAge) && SourceAge >= 0f &&
            SourceAge + Math.Max(next.Dt, Math.Max(next.FixedTime - Step.FixedTime,
                next.UnscaledTime - Step.UnscaledTime)) <= 0.1f;

        // Render has no future Fixed identity. Qualify only the explicit completed
        // witness against the new source time; do not fabricate an N+1 context.
        internal bool EligibleForSource(SourceStepContext source, PhysicsStepContext completed, int supportId) =>
            Step.Matches(completed) && source.Valid && source.Epoch == Step.Epoch &&
            source.Sequence > Step.SourceSequence && source.Time + .0001f >= Step.UnscaledTime &&
            !(StrongStance && StrongLandingApproach) && (StrongStance || StrongLandingApproach) &&
            supportId != 0 && SupportId == supportId && source.Dt < .1f &&
            !float.IsNaN(SourceAge) && !float.IsInfinity(SourceAge) && SourceAge >= 0f &&
            SourceAge + Math.Max(0f, source.Time - Step.UnscaledTime) <= .1f;
    }

    // Torso executes before legs. Carry N-1 explicitly; never label it current N.
    // Current support identity/geometry is independently validated by its owner.
    internal struct SupportReferenceContext
    {
        internal PhysicsStepContext ForStep, ObservedStep;
        internal int SupportId, StanceMask; // selected witnesses: bit 0 primary; bit 1 secondary
        internal int ApproachMask; // selected subset still transitioning to stance; never contact authorization
        internal bool ApproachOnly => StanceMask != 0 && ApproachMask == StanceMask;
        internal bool Matches(PhysicsStepContext step) => ForStep.Matches(step) &&
            ObservedStep.Valid && ObservedStep.SourceSequence > 0 && ObservedStep.Epoch == step.Epoch &&
            ObservedStep.Sequence + 1 == step.Sequence && ObservedStep.SourceSequence <= step.SourceSequence &&
            ObservedStep.FixedTime <= step.FixedTime && ObservedStep.UnscaledTime <= step.UnscaledTime &&
            SupportId != 0 && StanceMask >= 1 && StanceMask <= 3 &&
            ApproachMask >= 0 && ApproachMask <= 3 && (ApproachMask & StanceMask) == ApproachMask;

        internal static SupportReferenceContext Create(PhysicsStepContext step,
            PreviousFootSupport left, PreviousFootSupport right, int supportId, bool allowApproach = false)
        {
            if (!left.Precedes(step) || !right.Precedes(step) || !left.Step.Matches(right.Step)) return default;
            int mask = (left.Eligible(step, supportId, allowApproach) ? 1 : 0) | (right.Eligible(step, supportId, allowApproach) ? 2 : 0);
            if (mask == 0) return default;
            int approach = mask & ((left.StrongLandingApproach ? 1 : 0) | (right.StrongLandingApproach ? 2 : 0));
            return new SupportReferenceContext
                { ForStep = step, ObservedStep = left.Step, SupportId = supportId, StanceMask = mask, ApproachMask = approach };
        }
    }
}
