using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.3.12 minimal read-only bilateral tap. Hip reaction fields retired with v0.3.5;
    /// lifecycle/phase plus Knee tether vectors now feed only the small hard-safety context ring.
    /// </summary>
    internal readonly struct SingleLegBilateralReadOnlySnapshot
    {
        public readonly int FixedSequence;
        public readonly bool DriveEnabled;
        public readonly bool RecoveryOrUnhealthy;
        public readonly bool Idle;

        public readonly bool KneeTranslationTetherEnabled;
        public readonly float KneeAnchorValidityMeters;
        public readonly Vector3 KneePositionErrorWorld;
        public readonly Vector3 KneeVelocityErrorWorld;
        public readonly Vector3 KneeParentAnchorFeedforwardWorld;
        public readonly Vector3 KneeRawAccelerationWorld;
        public readonly Vector3 KneeAppliedAccelerationWorld;
        public readonly bool KneeTetherClamped;

        public SingleLegBilateralReadOnlySnapshot(
            int fixedSequence,
            bool driveEnabled,
            bool recoveryOrUnhealthy,
            bool idle,
            bool kneeTranslationTetherEnabled,
            float kneeAnchorValidityMeters,
            Vector3 kneePositionErrorWorld,
            Vector3 kneeVelocityErrorWorld,
            Vector3 kneeParentAnchorFeedforwardWorld,
            Vector3 kneeRawAccelerationWorld,
            Vector3 kneeAppliedAccelerationWorld,
            bool kneeTetherClamped)
        {
            FixedSequence = fixedSequence;
            DriveEnabled = driveEnabled;
            RecoveryOrUnhealthy = recoveryOrUnhealthy;
            Idle = idle;
            KneeTranslationTetherEnabled = kneeTranslationTetherEnabled;
            KneeAnchorValidityMeters = kneeAnchorValidityMeters;
            KneePositionErrorWorld = kneePositionErrorWorld;
            KneeVelocityErrorWorld = kneeVelocityErrorWorld;
            KneeParentAnchorFeedforwardWorld = kneeParentAnchorFeedforwardWorld;
            KneeRawAccelerationWorld = kneeRawAccelerationWorld;
            KneeAppliedAccelerationWorld = kneeAppliedAccelerationWorld;
            KneeTetherClamped = kneeTetherClamped;
        }
    }
}
