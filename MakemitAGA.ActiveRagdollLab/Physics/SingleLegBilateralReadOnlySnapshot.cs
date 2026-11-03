using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.3.12 minimal read-only bilateral tap. Hip reaction fields retired with v0.3.5;
    /// lifecycle plus the consumed Knee tether vectors feed the hard-safety context ring.
    /// </summary>
    internal readonly struct SingleLegBilateralReadOnlySnapshot
    {
        public readonly int FixedSequence;
        public readonly bool DriveEnabled;
        public readonly Vector3 KneePositionErrorWorld;
        public readonly Vector3 KneeParentAnchorFeedforwardWorld;
        public readonly Vector3 KneeRawAccelerationWorld;
        public readonly Vector3 KneeAppliedAccelerationWorld;
        public readonly bool KneeTetherClamped;

        public SingleLegBilateralReadOnlySnapshot(
            int fixedSequence,
            bool driveEnabled,
            Vector3 kneePositionErrorWorld,
            Vector3 kneeParentAnchorFeedforwardWorld,
            Vector3 kneeRawAccelerationWorld,
            Vector3 kneeAppliedAccelerationWorld,
            bool kneeTetherClamped)
        {
            FixedSequence = fixedSequence;
            DriveEnabled = driveEnabled;
            KneePositionErrorWorld = kneePositionErrorWorld;
            KneeParentAnchorFeedforwardWorld = kneeParentAnchorFeedforwardWorld;
            KneeRawAccelerationWorld = kneeRawAccelerationWorld;
            KneeAppliedAccelerationWorld = kneeAppliedAccelerationWorld;
            KneeTetherClamped = kneeTetherClamped;
        }
    }
}
