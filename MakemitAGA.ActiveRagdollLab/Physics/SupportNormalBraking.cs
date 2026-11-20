using System;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Reference-only look-ahead on the already qualified, held horizontal floor.
    // If next normal velocity is -u, reserve u*dt for this step and u^2/(2*a)
    // for later braking. a is the EXISTING reference acceleration bound, not an
    // estimate of body/allocator capability. Shared X/Z budget, later feedback,
    // moving floors or witness loss can leave a residual: no feasibility promise.
    internal static class SupportNormalBraking
    {
        internal static float MinimumVelocity(float positionY, float floorY, float dt, float maxA)
        {
            Validate(positionY, floorY, dt, maxA);
            float original = (floorY - positionY) / dt;
            if (positionY <= floorY) return original; // Original recovery, no added height offset.
            double distance = (double)positionY - floorY;
            // Positive root of u*dt + u^2/(2*a) = distance, rationalized to avoid
            // cancellation near the floor. No retained state or future sample.
            double speed = 2d * distance / (dt + Math.Sqrt((double)dt * dt + 2d * distance / maxA));
            return Math.Max(original, (float)-speed);
        }

        internal static float Residual(float nextPositionY, float nextVelocityY, float floorY, float maxA)
        {
            Validate(nextPositionY, floorY, 1f, maxA);
            if (!Finite(nextVelocityY)) throw new InvalidOperationException("SUPPORT_BRAKING_VELOCITY_INVALID");
            double down = Math.Max(0d, -(double)nextVelocityY);
            return (float)Math.Max(0d, floorY - (double)nextPositionY + down * down / (2d * maxA));
        }
        private static void Validate(float p, float floor, float dt, float maxA)
        {
            if (!Finite(p) || !Finite(floor) || !Finite(dt) || dt <= 0f || !Finite(maxA) || maxA <= 0f ||
                !Finite((floor - p) / dt))
                throw new InvalidOperationException("SUPPORT_BRAKING_INPUT_INVALID");
        }
        private static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
    }
}
