using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Controller-owned values captured once by the torso, before either leg consumes this step.
    // Error is PRE_N against the current governed command FOR_N, not a post-solve measurement.
    internal struct TorsoRecoveryState
    {
        internal PhysicsStepContext Step;
        internal bool Valid, Active;
        internal string Reason;
        internal float HoldSeconds, ComErrorMeters, PelvisErrorDegrees, ChestRelativeErrorDegrees;
        internal float NativeAnchorGapMeters, LegacyAnchorGapMeters;
        internal bool Matches(PhysicsStepContext step) => Valid && Step.Matches(step);
    }
}
