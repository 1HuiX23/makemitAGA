using System;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class TrajectoryContracts
    {
        internal static void Run()
        {
            Check.Run("trajectory finite fallback and analytic rotations", KnownRotations);
            Check.Run("trajectory bounded speed and progress", BoundedProgress);
        }
        private static float Norm(Quaternion q) => MathF.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        private static float DistanceDegrees(Quaternion a, Quaternion b)
        {
            a = HipReferenceTrajectoryMath.Normalize(a); b = HipReferenceTrajectoryMath.Normalize(b);
            float dot = MathF.Abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w);
            return 2f * MathF.Acos(Math.Clamp(dot, 0f, 1f)) * 180f / MathF.PI;
        }
        private static void KnownRotations()
        {
            foreach (float bad in new[] { 0f, 1e-12f, float.NaN, float.PositiveInfinity, float.NegativeInfinity })
            {
                Quaternion value = HipReferenceTrajectoryMath.Normalize(new Quaternion(bad, 0f, 0f, 0f));
                Check.That(value.x == 0f && value.y == 0f && value.z == 0f && value.w == 1f, "invalid/degenerate pose falls back to identity");
            }
            Check.Near(Norm(HipReferenceTrajectoryMath.Normalize(new Quaternion(2f, 3f, 4f, 5f))), 1f, 1e-6f, "valid pose normalized");
            Vector3 stopped = HipReferenceTrajectoryMath.ComputeBrakingDesiredAngularVelocity(Quaternion.identity, Quaternion.identity, 60f, 6f, .02f);
            Check.That(stopped.sqrMagnitude == 0f, "same rotation commands zero speed");
            Quaternion quarterTurn = HipReferenceTrajectoryMath.IntegrateRotation(Quaternion.identity, new Vector3(0f, MathF.PI / 2f, 0f), 1f);
            Check.Near(quarterTurn.y, MathF.Sqrt(.5f), 1e-6f, "one-second 90-degree rotation has analytic sine component");
            Check.Near(quarterTurn.w, MathF.Sqrt(.5f), 1e-6f, "one-second 90-degree rotation has analytic cosine component");
            Quaternion unchanged = HipReferenceTrajectoryMath.IntegrateRotation(quarterTurn, new Vector3(float.NaN, 0f, 0f), .02f);
            Check.Near(DistanceDegrees(unchanged, quarterTurn), 0f, .06f, "nonfinite angular speed holds normalized pose");
            var around = Quaternion.AngleAxis(270f, new Vector3(0f, 1f, 0f));
            Vector3 shortest = HipReferenceTrajectoryMath.ComputeBrakingDesiredAngularVelocity(Quaternion.identity, around, 60f, 6f, .02f);
            Check.That(shortest.y < 0f && shortest.magnitude <= 6.00001f, "270 degrees chooses the short negative route");
            var near = Quaternion.AngleAxis(1f, new Vector3(0f, 0f, 1f));
            Vector3 oneStep = HipReferenceTrajectoryMath.ComputeBrakingDesiredAngularVelocity(Quaternion.identity, near, 60f, 6f, .1f);
            Check.Near(oneStep.magnitude, MathF.PI / 18f, .00003f, "one-step cap avoids overshooting a small target");
            var far = Quaternion.AngleAxis(90f, new Vector3(0f, 1f, 0f));
            Vector3 brake = HipReferenceTrajectoryMath.ComputeBrakingDesiredAngularVelocity(Quaternion.identity, far, .1f, 100f, .02f);
            Check.Near(brake.magnitude, MathF.Sqrt(.1f * MathF.PI), .00001f, "stopping-distance cap applies before the speed ceiling");
        }
        private static void BoundedProgress()
        {
            var random = new Random(4119);
            for (int i = 0; i < 256; i++)
            {
                Quaternion from = HipReferenceTrajectoryMath.Normalize(new Quaternion((float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble(), (float)random.NextDouble()));
                Vector3 axis = new Vector3((float)random.NextDouble() - .5f, (float)random.NextDouble() - .5f, (float)random.NextDouble() - .5f);
                float degrees = .5f + 169f * (float)random.NextDouble();
                Quaternion to = Quaternion.AngleAxis(degrees, axis) * from;
                float dt = .005f + .04f * (float)random.NextDouble();
                float limit = .25f + 6f * (float)random.NextDouble();
                Vector3 velocity = HipReferenceTrajectoryMath.ComputeBrakingDesiredAngularVelocity(from, to, 60f, limit, dt);
                Check.That(float.IsFinite(velocity.magnitude) && velocity.magnitude <= limit + .00001f, "finite input respects speed ceiling");
                Check.That(velocity.magnitude * dt <= degrees * Mathf.Deg2Rad + .00005f, "one step cannot exceed the remaining angle");
                Quaternion next = HipReferenceTrajectoryMath.IntegrateRotation(from, velocity, dt);
                Check.Near(Norm(next), 1f, .000002f, "integration preserves unit quaternion");
                Check.That(DistanceDegrees(next, to) <= DistanceDegrees(from, to) + .06f, "bounded step approaches the target");
                Quaternion negative = new Quaternion(-to.x, -to.y, -to.z, -to.w);
                Vector3 sameRotation = HipReferenceTrajectoryMath.ComputeBrakingDesiredAngularVelocity(from, negative, 60f, limit, dt);
                Check.Near(sameRotation.x, velocity.x, .0005f, "quaternion sign does not change x command");
                Check.Near(sameRotation.y, velocity.y, .0005f, "quaternion sign does not change y command");
                Check.Near(sameRotation.z, velocity.z, .0005f, "quaternion sign does not change z command");
            }
        }
    }
}
