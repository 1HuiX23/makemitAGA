using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Committed Hip commands and their pre-solve native geometry. Value-only, no actuator.
    // The parent wrenches below are unexecuted algebraic counterparts, NOT approved internal
    // joint commands. The total Hip torque includes controller/transport/gyro contributions.
    internal struct HipCommandWrench
    {
        internal bool Valid;
        internal PhysicsStepContext Step;
        internal int Fixed, UpperBodyId, ParentBodyId;
        internal float Mass;
        internal Vector3 UpperCom, ParentCom, ChildAnchor, ParentAnchor;
        internal Vector3 UpperV, ParentV, UpperW, ParentW, ChildPointV, ParentPointV;
        internal Vector3 Force, Torque;

        internal Vector3 ParentForce => -Force;
        internal Vector3 ComForcePairMoment => Vector3.Cross(UpperCom - ParentCom, Force);
        internal Vector3 AnchorForcePairMoment => Vector3.Cross(UpperCom - ParentAnchor, Force);
        internal Vector3 MoveUpperForceToAnchorTorque => Vector3.Cross(ChildAnchor - UpperCom, Force);
        internal Vector3 BothAnchorForcePairMoment => Vector3.Cross(ChildAnchor - ParentAnchor, Force);
        // Transport an exact negative Upper wrench to either point on the parent.
        internal Vector3 EquivalentParentComTorque => -Torque - ComForcePairMoment;
        internal Vector3 EquivalentParentAnchorTorque => -Torque - AnchorForcePairMoment;
        internal float ActualUpperPower => Vector3.Dot(Force, UpperV) + Vector3.Dot(Torque, UpperW);
        internal float CandidateParentComPower => Vector3.Dot(ParentForce, ParentV) + Vector3.Dot(EquivalentParentComTorque, ParentW);
        internal float CandidateParentAnchorPower => Vector3.Dot(ParentForce, ParentPointV) + Vector3.Dot(EquivalentParentAnchorTorque, ParentW);
        internal float BothAnchorForcePairPower => Vector3.Dot(Force, ChildPointV - ParentPointV);
        internal Vector3 WrenchClosure => Torque + Vector3.Cross(UpperCom - ParentCom, Force) +
            Vector3.Cross(ParentAnchor - ParentCom, ParentForce) + EquivalentParentAnchorTorque;

        internal bool Finite => Valid && Step.Valid && Fixed > 0 && UpperBodyId != 0 && ParentBodyId != 0 &&
            UpperBodyId != ParentBodyId && F(Mass) && Mass > 0.000001f &&
            V(UpperCom) && V(ParentCom) && V(ChildAnchor) && V(ParentAnchor) &&
            V(UpperV) && V(ParentV) && V(UpperW) && V(ParentW) && V(ChildPointV) && V(ParentPointV) &&
            V(Force) && V(Torque) && V(ComForcePairMoment) && V(AnchorForcePairMoment) &&
            V(MoveUpperForceToAnchorTorque) && V(BothAnchorForcePairMoment) &&
            V(EquivalentParentComTorque) && V(EquivalentParentAnchorTorque) && V(WrenchClosure) &&
            F(ActualUpperPower) && F(CandidateParentComPower) && F(CandidateParentAnchorPower) && F(BothAnchorForcePairPower);
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 x) => F(x.x) && F(x.y) && F(x.z) && F(x.sqrMagnitude);
    }
}
