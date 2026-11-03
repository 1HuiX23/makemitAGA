using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Same-step native parent model plus the actual committed receiver inputs.
    // Published only after BOTH native force writes and the receiver torque return.
    // Acceleration is the bounded nominal PD+AFF request. In the typed Hip scope
    // the bilateral solver targets q*Acceleration for the P+C SYSTEM COM and adds
    // same-step Hip-force compensation to the final actuator receipt.
    // This is the immutable BASELINE receipt (torque <= 2 Nm), never the final
    // compensated torque. Bilateral completion publishes a separate receipt.
    internal struct PelvisHipCommandFrame
    {
        internal const float MaxMagnitude = 12f;
        // Shared with reference back-calculation; these are the unchanged common PD gains.
        internal const float PositionGain = 20f, VelocityGain = 9f;
        internal bool Committed, JointTorqueReaction;
        internal PhysicsStepContext Step;
        internal int BodyId;
        internal int ChestBodyId;
        internal Vector3 Acceleration;
        internal Vector3 ReceiverTorque, ParentW;
        // V041080: the governed target consumed by THIS receiver step, not the
        // settled parent state or the response to its torque. B normal Hip goals
        // use this separate reference domain; physical feedback still uses ParentW.
        internal Quaternion ReferenceRotation;
        internal Vector3 ReferenceW;
        internal PelvisChestPointResponse Response;
        internal bool Matches(PhysicsStepContext step, int bodyId) => Committed && BodyId != 0 && BodyId == bodyId &&
            ChestBodyId != 0 && ChestBodyId != BodyId && Step.Matches(step) && Response.Valid &&
            PelvisParentFrame.V(Acceleration) && Acceleration.magnitude <= MaxMagnitude + 0.001f &&
            PelvisParentFrame.V(ReceiverTorque) && ReceiverTorque.magnitude <= 2.001f && PelvisParentFrame.V(ParentW) &&
            (!JointTorqueReaction || (PelvisParentFrame.Q(ReferenceRotation) && PelvisParentFrame.V(ReferenceW)));
        internal bool SameAs(PelvisHipCommandFrame other) => Matches(other.Step, other.BodyId) && other.Committed &&
            ChestBodyId == other.ChestBodyId && JointTorqueReaction == other.JointTorqueReaction && Response.SameAs(other.Response) &&
            PelvisChestPointResponse.Same(Acceleration, other.Acceleration) && PelvisChestPointResponse.Same(ReceiverTorque, other.ReceiverTorque) &&
            PelvisChestPointResponse.Same(ParentW, other.ParentW) &&
            (!JointTorqueReaction || (ReferenceRotation.x == other.ReferenceRotation.x && ReferenceRotation.y == other.ReferenceRotation.y &&
                ReferenceRotation.z == other.ReferenceRotation.z && ReferenceRotation.w == other.ReferenceRotation.w &&
                PelvisChestPointResponse.Same(ReferenceW, other.ReferenceW)));
    }
}
