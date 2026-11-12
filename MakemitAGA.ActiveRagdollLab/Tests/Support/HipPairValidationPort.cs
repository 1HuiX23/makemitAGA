// Native snapshot boundary only. Production CanCommit is linked unchanged;
// no force integration or replacement validation algorithm lives in this port.
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class HipAnchorTranslationPair
    {
        private bool _staged = true;
        private readonly HipAnchorPairReport Report = new HipAnchorPairReport { Selected = true, Finite = true };
        private HipAnchorPairInput _input;
        private BodySnapshot _child, _parent;
        private sealed class BodySnapshot
        {
            internal int Id;
            internal bool isKinematic = false, useGravity = false;
            internal float mass;
            internal Vector3 worldCenterOfMass, velocity, angularVelocity, inertiaTensor;
            internal Quaternion rotation = Quaternion.identity, inertiaTensorRotation = Quaternion.identity;
            internal int GetInstanceID() => Id;
        }
        internal static bool CheckActualPrecommit(HipAnchorPairInput input, HipAnchorPairReport.Command c)
        {
            var p = new HipAnchorTranslationPair { _input = input };
            p.Report.ChildId = c.ChildId; p.Report.ParentId = c.ParentId;
            p._child = new BodySnapshot { Id = c.ChildId, mass = input.Seed.ChildMass,
                worldCenterOfMass = input.Seed.ChildCom, velocity = input.ChildComV,
                angularVelocity = input.ChildW, inertiaTensor = input.ChildInertia, rotation = input.ChildPrincipal };
            p._parent = new BodySnapshot { Id = c.ParentId, mass = c.ParentInput.Response.PelvisMass,
                worldCenterOfMass = input.Seed.ParentCom, velocity = input.Seed.ParentV,
                angularVelocity = c.ParentInput.ParentW, inertiaTensor = c.ParentInput.Response.PelvisInertia,
                rotation = c.ParentInput.Response.PelvisPrincipal };
            return p.CanCommit(c);
        }
        private static bool F(float v) => HipPointPairMobility.F(v);
        private static bool V(Vector3 v) => HipPointPairMobility.V(v);
        private static bool Same(Vector3 a, Vector3 b) => PelvisChestPointResponse.Same(a, b);
    }
}
