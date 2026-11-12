using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class HipAnchorTranslationPair
    {
        private bool CanCommit(HipAnchorPairReport.Command c)
        {
            if (!_staged || !Report.Selected || !Report.Finite || !_input.Ready || !c.JointSolved || Report.CurrentApplied ||
                !c.Step.Matches(_input.Seed.Step) || c.Fixed != _input.Seed.Fixed || c.ChildId != Report.ChildId || c.ParentId != Report.ParentId ||
                !c.ParentInput.SameAs(_input.Seed.ParentInput) || !Same(c.PureHipTorque, _input.PureTorque) ||
                !Same(c.ChildCom, _input.Seed.ChildCom) || !Same(c.ParentCom, _input.Seed.ParentCom) ||
                !Same(c.ChildPoint, _input.Seed.ChildPoint) || !Same(c.ParentPoint, _input.Seed.ParentPoint) ||
                _child == null || _parent == null || _child.isKinematic || _parent.isKinematic || _parent.useGravity ||
                _child.GetInstanceID() != Report.ChildId || _parent.GetInstanceID() != Report.ParentId ||
                _child.mass != c.ChildMass || _parent.mass != c.ParentMass ||
                !Same(_child.worldCenterOfMass, c.ChildCom) || !Same(_parent.worldCenterOfMass, c.ParentCom) ||
                !Same(_child.velocity, _input.ChildComV) || !Same(_child.angularVelocity, _input.ChildW) ||
                !Same(_child.inertiaTensor, _input.ChildInertia) ||
                !HipPointPairMobility.ValidBody(_child.mass, _child.inertiaTensor, _child.rotation * _child.inertiaTensorRotation) ||
                Quaternion.Angle(_child.rotation * _child.inertiaTensorRotation, _input.ChildPrincipal) > 0.001f ||
                !c.ParentInput.Response.MatchesBody(_parent.mass, _parent.inertiaTensor, _parent.rotation * _parent.inertiaTensorRotation) ||
                !Same(_parent.angularVelocity, c.ParentInput.ParentW) || !V(_parent.velocity) ||
                (_parent.velocity + Vector3.Cross(_parent.angularVelocity, c.ParentPoint - c.ParentCom) - c.ParentV).magnitude > 0.0001f ||
                !V(c.Force) || !V(c.ChildTorque) || !V(c.ParentTorque) || !V(c.PredictedNetA) || !V(c.FeedbackResidualA) ||
                !V(c.UpperCorrectionTorque) || !V(c.UpperActuatorTorque) || !V(c.UpperCorrectionAlpha) ||
                !Same(c.ChildTorque, Vector3.Cross(c.ChildPoint - c.ChildCom, c.Force)) ||
                !Same(c.ParentTorque, Vector3.Cross(c.ParentPoint - c.ParentCom, -c.Force)) ||
                !Same(c.UpperCorrectionTorque, -c.ChildTorque) || !Same(c.UpperActuatorTorque, c.NetHipTorque + c.UpperCorrectionTorque) ||
                !HipSystemAllocation.Valid(c.ParentInput, c.MotionScale, c.UpwardExtraAcceleration) ||
                (!c.FullFeedbackFeasible && c.UpwardExtraAcceleration != 0f) ||
                !Same(c.NetHipTorque, c.PureHipTorque * c.MotionScale) || !c.Angular.SameAs(_input.Angular) ||
                !Same(c.ParentAngularReaction, c.ParentInput.JointTorqueReaction ? -c.MotionScale * c.Angular.RelativeTorque : Vector3.zero) ||
                !V(c.BilateralAngularReaction) || (!c.ParentInput.JointTorqueReaction && !Same(c.BilateralAngularReaction, Vector3.zero)) ||
                !V(c.CommonAcceleration) || c.CommonAcceleration.magnitude > PelvisHipCommandFrame.MaxMagnitude + 0.0001f ||
                !V(c.PeerForce) || !V(c.AllocatedHipAlpha) ||
                !HipSystemAllocation.Matches(c.ParentInput, c.MotionScale, c.UpwardExtraAcceleration,
                    c.CommonAcceleration, -c.Force - c.PeerForce) ||
                !Same(c.AllocatedHipAlpha, _input.DesiredAlpha + HipPointPairMobility.InverseTensor(_input.ChildInertia, _input.ChildPrincipal, c.NetHipTorque - c.PureHipTorque)) ||
                !Same(c.UpperCorrectionAlpha, HipPointPairMobility.InverseTensor(_input.ChildInertia, _input.ChildPrincipal, c.UpperCorrectionTorque)) ||
                !F(c.UpperActuatorLimitNm) || c.UpperActuatorLimitNm != HipBilateralForceSolver.ActuatorLimit(c.PureHipTorque) ||
                c.UpperCorrectionTorque.magnitude > HipBilateralForceSolver.UpperCorrectionTorqueLimit + 0.0001f ||
                c.UpperActuatorTorque.magnitude > c.UpperActuatorLimitNm + 0.0001f ||
                !V(c.AppliedA) ||
                !Same(c.AppliedA, c.Force / c.ChildMass + (c.Force + c.PeerForce) / c.ParentInput.Response.TotalMass) ||
                c.AppliedA.magnitude > HipCharacterJointResponseIsolation.TetherMaxAcceleration + 0.0001f ||
                !F(c.Scale) || c.Scale < 0f || c.Scale > 1f || !F(c.PairPower) || !F(c.DampingPower) ||
                !F(c.BilateralDampingPower) || c.BilateralDampingPower > 0.0001f) return false;
            return true;
        }
    }
}
