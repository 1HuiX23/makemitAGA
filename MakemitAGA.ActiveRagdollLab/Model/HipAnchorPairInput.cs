using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Physical staging receipt, independent of optional trace. No force is issued until
    // both sides have the same baseline and their pure Upper torque requests have been staged.
    internal struct HipAnchorPairInput
    {
        internal bool Valid, TorqueRequested;
        internal HipAnchorPairReport.Command Seed;
        internal Vector3 ChildInertia, ChildComV, ChildW, PureTorque, DesiredAlpha;
        internal Quaternion ChildPrincipal;

        internal bool Ready => Valid && TorqueRequested && Seed.Fixed > 0 &&
            Seed.ChildId != Seed.ParentId && Seed.ChildId != 0 &&
            Seed.ParentInput.Matches(Seed.Step, Seed.ParentId) &&
            HipPointPairMobility.ValidBody(Seed.ChildMass, ChildInertia, ChildPrincipal) &&
            HipPointPairMobility.V(PureTorque) && HipPointPairMobility.V(DesiredAlpha);
    }
}
