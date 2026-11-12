// Native-call boundary only; no rigid-body integration or collision simulation.
namespace UnityEngine
{
    internal sealed class Rigidbody
    {
        internal bool isKinematic = false, useGravity = false;
        internal float mass = 1f;
        internal int ForceCalls = 0;
        internal Vector3 ForceSum = Vector3.zero;
        internal void AddForce(Vector3 force, ForceMode mode) { ForceCalls++; ForceSum += force; }
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
