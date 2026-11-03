using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// Pure trajectory math shared by the local and final-world Hip shock absorbers.
    /// The callers retain phase budgets, state, release policy and contract diagnostics.
    /// Other Hip integrators intentionally keep their different normalization/limiting rules.
    /// </summary>
    internal static class HipReferenceTrajectoryMath
    {
        internal static Vector3 ComputeBrakingDesiredAngularVelocity(Quaternion from, Quaternion to, float maxA, float maxW, float dt)
        {
            Quaternion delta = Normalize(to * Quaternion.Inverse(from));
            delta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
            if (!Finite(angleDegrees) || !Finite(axis) || axis.sqrMagnitude < 1e-12f)
                return Vector3.zero;
            if (angleDegrees > 180f) angleDegrees -= 360f;
            float angleRad = angleDegrees * Mathf.Deg2Rad;
            if (Mathf.Abs(angleRad) < 1e-6f)
                return Vector3.zero;
            axis.Normalize();
            float stoppingLimitedW = Mathf.Sqrt(Mathf.Max(0f, 2f * maxA * Mathf.Abs(angleRad)));
            float oneStepSpeed = Mathf.Abs(angleRad) / Mathf.Max(0.0001f, dt);
            float desiredSpeed = Mathf.Min(maxW, Mathf.Min(stoppingLimitedW, oneStepSpeed));
            return axis * Mathf.Sign(angleRad) * desiredSpeed;
        }

        internal static Quaternion IntegrateRotation(Quaternion pose, Vector3 angularVelocity, float dt)
        {
            float speed = angularVelocity.magnitude;
            if (!Finite(speed) || speed < 1e-7f)
                return Normalize(pose);
            Vector3 axis = angularVelocity / speed;
            Quaternion step = Quaternion.AngleAxis(speed * dt * Mathf.Rad2Deg, axis);
            return Normalize(step * pose);
        }

        internal static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!Finite(mag) || mag < 1e-8f) return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        private static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
        private static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    }
}
