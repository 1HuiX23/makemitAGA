// Native-call boundary only; no rigid-body integration or collision simulation.
namespace UnityEngine
{
    internal sealed class Rigidbody
    {
        internal bool isKinematic = false, useGravity = false, FailTorque = false;
        internal float mass = 1f;
        internal int Id, ForceCalls, TorqueCalls, PointCalls;
        internal Vector3 velocity, angularVelocity, worldCenterOfMass, ForceSum, TorqueSum;
        internal Vector3 inertiaTensor = new Vector3(1f,1f,1f);
        internal Quaternion rotation = Quaternion.identity, inertiaTensorRotation = Quaternion.identity;
        internal readonly System.Collections.Generic.HashSet<int> FailForceCalls = new System.Collections.Generic.HashSet<int>();
        internal System.Action BeforeWrite;
        internal int GetInstanceID() => Id;
        internal void AddForce(Vector3 force, ForceMode mode)
        {
            BeforeWrite?.Invoke();
            ForceCalls++;
            if (FailForceCalls.Contains(ForceCalls)) throw new System.InvalidOperationException("force");
            ForceSum += force;
        }
        internal void AddTorque(Vector3 torque, ForceMode mode)
        {
            BeforeWrite?.Invoke();
            TorqueCalls++;
            if (FailTorque) throw new System.InvalidOperationException("torque");
            TorqueSum += torque;
        }
        internal void AddForceAtPosition(Vector3 force, Vector3 point, ForceMode mode)
        {
            BeforeWrite?.Invoke();
            PointCalls++;
            ForceSum += force;
            TorqueSum += Vector3.Cross(point-worldCenterOfMass,force);
        }
    }
}
namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static class KneeCharacterJointResponseIsolation
    {
        // Same frozen limit; used only by the linked reaction's diagnostic near-cap counter.
        internal const float TetherMaxAcceleration = 60f;
    }
}
