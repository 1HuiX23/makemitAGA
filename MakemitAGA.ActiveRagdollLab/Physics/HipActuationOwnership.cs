using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Stop-time command audit, reusing actual committed receipts and cached pre-solve
    // velocities. No native reads, new sampler, force, state feedback or safety gate.
    // These are selected Hip inputs, not a complete whole-body momentum/energy ledger.
    internal static class HipActuationOwnership
    {
        internal struct OwnershipSample
        {
            internal Vector3 PointPairMoment, PointMomentClosure, UpperCorrection, UpperNetResidual;
            internal Vector3 JointPairResidual;
            internal float JointPairPower, CompensationResidual;
            internal Vector3 CommonForce, CommonDeltaForce, SystemForceResidual;
            internal float PointPairPower, UpperPower, ReceiverPower;
        }

        internal static bool TryMeasure(HipAnchorPairReport.Command c, Vector3 actualUpperTorque,
            Vector3 upperW, Vector3 parentW, PelvisHipAttitudeCommand receiver, out OwnershipSample sample)
        {
            sample = new OwnershipSample();
            if (!c.Step.Valid || c.Fixed <= 0 || !c.JointSolved || !c.Angular.Valid || !receiver.Matches(c.ParentInput) ||
                !PelvisChestPointResponse.Same(c.BilateralAngularReaction, receiver.AngularReaction) ||
                receiver.MotionScale != c.MotionScale || !V(c.Force) || !V(c.ChildCom) || !V(c.ParentCom) ||
                !V(c.ChildPoint) || !V(c.ParentPoint) || !V(c.ChildV) || !V(c.ParentV) ||
                !V(c.ChildTorque) || !V(c.ParentTorque) || !V(c.NetHipTorque) ||
                !V(actualUpperTorque) || !V(upperW) || !V(parentW)) return false;
            // Translate the two COM moments to ONE origin (parent COM). Adding
            // moments about different COMs directly is not angular closure.
            Vector3 moment = Vector3.Cross(c.ChildCom - c.ParentCom, c.Force) + c.ChildTorque + c.ParentTorque;
            Vector3 direct = Vector3.Cross(c.ChildPoint - c.ParentPoint, c.Force);
            float mass = c.ParentInput.Response.TotalMass;
            OwnershipSample s = new OwnershipSample
            {
                PointPairMoment = moment, PointMomentClosure = moment - direct,
                JointPairResidual = c.ParentAngularReaction + (c.ParentInput.JointTorqueReaction ? c.MotionScale * c.Angular.RelativeTorque : Vector3.zero),
                JointPairPower = c.ParentInput.JointTorqueReaction ? Vector3.Dot(c.MotionScale * c.Angular.RelativeTorque, upperW - parentW) : 0f,
                CompensationResidual = (receiver.ResidualTorque - receiver.AngularReaction).magnitude,
                UpperCorrection = actualUpperTorque - c.NetHipTorque,
                UpperNetResidual = actualUpperTorque + c.ChildTorque - c.NetHipTorque,
                CommonForce = receiver.CommonAcceleration * mass,
                CommonDeltaForce = (receiver.CommonAcceleration - c.ParentInput.Acceleration) * mass,
                SystemForceResidual = (receiver.CommonAcceleration + receiver.HipForce / mass - receiver.TargetSystemAcceleration) * mass,
                PointPairPower = Vector3.Dot(c.Force, c.ChildV - c.ParentV),
                UpperPower = Vector3.Dot(actualUpperTorque, upperW),
                ReceiverPower = Vector3.Dot(receiver.AppliedTorque, parentW)
            };
            if (!V(s.PointPairMoment) || !V(s.PointMomentClosure) || !V(s.UpperCorrection) || !V(s.UpperNetResidual) ||
                !V(s.JointPairResidual) || !F(s.JointPairPower) || !F(s.CompensationResidual) || !V(s.CommonForce) || !V(s.CommonDeltaForce) || !V(s.SystemForceResidual) ||
                !F(s.PointPairPower) || !F(s.UpperPower) || !F(s.ReceiverPower)) return false;
            sample = s; return true;
        }
        private static bool V(Vector3 v) => HipPointPairMobility.V(v) && F(v.sqrMagnitude);
        private static bool F(float v) => HipPointPairMobility.F(v);
    }
}
