using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One gain definition for the force request AND reference-deficit back-calculation.
    // The normal is world Y in the current horizontal owned-Box experiment only.
    // Current Hip arms retain v124 B normal gains and the nominal damping ratio;
    // this is an experiment, not a human-motion or loaded-balance certification.
    internal static class PelvisComFeedback
    {
        internal const float PositionGain = 20f, VelocityGain = 9f;
        internal const float NormalPositionGain = 40f, NormalVelocityGain = 12.727922f;

        internal static float PositionY(bool enhanced) => enhanced ? NormalPositionGain : PositionGain;
        internal static float VelocityY(bool enhanced) => enhanced ? NormalVelocityGain : VelocityGain;

        internal static Vector3 Evaluate(Vector3 error, Vector3 velocityError, bool enhanced)
        {
            // Keep legacy arithmetic exactly, including both tangent components.
            Vector3 original = PositionGain * error + VelocityGain * velocityError;
            return enhanced ? new Vector3(original.x,
                NormalPositionGain * error.y + NormalVelocityGain * velocityError.y, original.z) : original;
        }

        internal static float Sensitivity(float dt, bool normalEnhanced) =>
            1f + VelocityY(normalEnhanced) * dt + PositionY(normalEnhanced) * dt * dt;

        // Semi-implicit reference p/v make d(commandA)/d(referenceA) diagonal.
        // Map the known previous INPUT deficit into reference-acceleration space.
        // It is not an observed-force estimate and never divides by motion scale q.
        internal static Vector3 ReferenceDeficit(Vector3 deficit, float dt, bool enhanced)
        {
            float tangent = Sensitivity(dt, false), normal = Sensitivity(dt, enhanced);
            return new Vector3(deficit.x / tangent, deficit.y / normal, deficit.z / tangent);
        }
    }
}
