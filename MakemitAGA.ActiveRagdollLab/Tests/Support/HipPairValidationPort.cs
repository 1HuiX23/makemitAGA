// Native snapshot boundary only. Production CanCommit is linked unchanged;
// no force integration or replacement validation algorithm lives in this port.
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class HipAnchorTranslationPair
    {
        internal static bool CheckActualPrecommit(HipAnchorPairInput input, HipAnchorPairReport.Command c)
        {
            var p = Staged(input, c, null, false);
            return p.CanCommit(c);
        }

        internal static HipAnchorTranslationPair Staged(HipAnchorPairInput input, HipAnchorPairReport.Command c, Rigidbody pelvis, bool trace)
        {
            var pair = new HipAnchorTranslationPair { _input=input, _staged=true, _trace=trace };
            pair.Report.Selected=pair.Report.Finite=true; pair.Report.ChildId=c.ChildId; pair.Report.ParentId=c.ParentId;
            pair._child = new Rigidbody { Id=c.ChildId, mass=input.Seed.ChildMass, worldCenterOfMass=input.Seed.ChildCom,
                velocity=input.ChildComV, angularVelocity=input.ChildW, inertiaTensor=input.ChildInertia, rotation=input.ChildPrincipal };
            pair._parent = pelvis ?? new Rigidbody { Id=c.ParentId, mass=c.ParentInput.Response.PelvisMass,
                worldCenterOfMass=input.Seed.ParentCom, velocity=input.Seed.ParentV-Vector3.Cross(c.ParentInput.ParentW,input.Seed.ParentPoint-input.Seed.ParentCom),
                angularVelocity=c.ParentInput.ParentW, inertiaTensor=c.ParentInput.Response.PelvisInertia, rotation=c.ParentInput.Response.PelvisPrincipal };
            return pair;
        }

        internal Rigidbody TestUpper => _child;

    }
}
