using System;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Shared by the optional Knee/Foot reference choices. These are the original
    // budget balls and relative-speed stopping offset, not extra actuator authority.
    internal static class GroundReferenceStepBudget
    {
        internal static float BrakeDebt(Vector3 relativeW, Vector3 direction, float remaining, float maxA, float dt) =>
            Mathf.Max(0f, Mathf.Max(0f, Vector3.Dot(relativeW, direction)) *
                (relativeW.magnitude / (2f * maxA) + dt) - remaining);

        internal static double PositiveBallExit(Vector3 value, Vector3 direction, float radius)
        {
            double a = (double)direction.x * direction.x + (double)direction.y * direction.y + (double)direction.z * direction.z;
            double b = (double)value.x * direction.x + (double)value.y * direction.y + (double)value.z * direction.z;
            double c = (double)value.x * value.x + (double)value.y * value.y + (double)value.z * value.z - (double)radius * radius;
            double d = b * b - a * c;
            if (a <= 0 || d < 0 || double.IsNaN(d) || double.IsInfinity(d)) return 0;
            return Math.Max(0, (-b + Math.Sqrt(d)) / a);
        }
    }
}
