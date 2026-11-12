using System;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Horizontal controlled Box only: reduce parent-position lag BELOW the held
    // calibrated animation origin, within the original vector V/A budget. The
    // V041200 remainder candidate is retained as A; V041220 B may reallocate the
    // nominal X/Z increment to normal correction while preserving tangent braking.
    // No pose clamp, body write, leg IK or guarantee of a ground-feasible foot.
    internal static class SupportNormalReference
    {
        internal static Vector3 RaiseCandidate(Vector3 position, Vector3 velocity, Vector3 candidate,
            float floorY, float dt, float maxV, float maxA)
        {
            if (!V(position) || !V(velocity) || !V(candidate) || !F(floorY) ||
                !F(dt) || dt <= 0f || !F(maxV) || maxV <= 0f || !F(maxA) || maxA <= 0f)
                throw new InvalidOperationException("SUPPORT_REFERENCE_INPUT_INVALID");
            float requiredY = (floorY - position.y) / dt;
            if (!F(requiredY)) throw new InvalidOperationException("SUPPORT_REFERENCE_REQUIRED_V_INVALID");
            if (requiredY <= candidate.y) return candidate;
            float dx = candidate.x - velocity.x, dz = candidate.z - velocity.z;
            float dv = maxA * dt;
            float aRoom = dv * dv - dx * dx - dz * dz;
            float vRoom = maxV * maxV - candidate.x * candidate.x - candidate.z * candidate.z;
            // Roundoff at a saturated vector bound must not create extra authority.
            float highestY = Mathf.Min(velocity.y + Mathf.Sqrt(Mathf.Max(0f, aRoom)),
                Mathf.Sqrt(Mathf.Max(0f, vRoom)));
            if (highestY <= candidate.y) return candidate;
            return new Vector3(candidate.x, Mathf.Min(requiredY, highestY), candidate.z);
        }

        // V041220 keeps the SAME speed sphere and acceleration sphere. Only an
        // additional upward correction may reduce the nominal X/Z velocity INCREMENT;
        // existing X/Z velocity is not scaled. Retain nominal tangential braking.
        // The receiver still applies its original deficit/braking feedback afterwards.
        internal static Vector3 AllocateNormalPriority(Vector3 position, Vector3 velocity, Vector3 candidate,
            float floorY, float dt, float maxV, float maxA, out bool tangentBrakingPreserved)
        {
            tangentBrakingPreserved = false;
            Vector3 remainder = RaiseCandidate(position, velocity, candidate, floorY, dt, maxV, maxA);
            float requiredY = (floorY - position.y) / dt;
            if (requiredY <= remainder.y) return remainder;

            double dx = (double)candidate.x - velocity.x, dz = (double)candidate.z - velocity.z;
            double deltaSquared = dx * dx + dz * dz;
            double tangentDot = velocity.x * dx + velocity.z * dz;
            if (tangentDot < 0d)
            {
                tangentBrakingPreserved = true;
                return remainder;
            }
            if (deltaSquared == 0d) return remainder;

            double speedSquared = (double)maxV * maxV;
            double tangentSquared = (double)velocity.x * velocity.x + (double)velocity.z * velocity.z;
            if (tangentSquared > speedSquared) return remainder; // Rounded input at the speed boundary.
            double stepBudget = (double)maxA * dt;
            float normal = (float)Math.Min(requiredY, Math.Min(velocity.y + stepBudget,
                Math.Sqrt(Math.Max(0d, speedSquared - tangentSquared))));
            if (normal <= remainder.y) return remainder;

            // With tangentDot >= 0, both spheres permit an interval 0 <= scale <= k.
            // Choose its greatest scale: retain as much nominal X/Z increment as
            // possible after the bounded normal choice. Use double intermediates
            // and the rounded final Y when solving the two scalar constraints.
            double dy = (double)normal - velocity.y;
            double accelerationScale = Math.Sqrt(Math.Max(0d, stepBudget * stepBudget - dy * dy) / deltaSquared);
            double speedRoom = Math.Max(0d, speedSquared - tangentSquared - (double)normal * normal);
            double denominator = tangentDot + Math.Sqrt(tangentDot * tangentDot + deltaSquared * speedRoom);
            double speedScale = denominator > 0d ? speedRoom / denominator : 0d;
            double scale = Math.Min(1d, Math.Min(accelerationScale, speedScale));
            return new Vector3((float)(velocity.x + scale * dx), normal, (float)(velocity.z + scale * dz));
        }
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
    }
}
