using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Same-step native parent model plus the actual committed receiver inputs.
    // Published only after BOTH native force writes and the receiver torque return.
    // Acceleration is the actual bounded receiver request: PD+AFF plus the selected
    // v137 held-floor recovery envelope. It is the request that was written, never
    // a discarded baseline. In the typed Hip scope
    // the bilateral solver targets q*Acceleration + explicit upward extra for SYSTEM COM,
    // uses a separate Upper angular scale, and adds
    // same-step Hip-force compensation to the final actuator receipt.
    // This is the immutable BASELINE receipt (torque <= 2 Nm), never the final
    // compensated torque. Bilateral completion publishes a separate receipt.
    internal struct PelvisHipCommandFrame
    {
        internal const float MaxMagnitude = 12f;
        // Gain ownership is PelvisComFeedback; this receipt carries actual bounded inputs.
        internal bool Committed, JointTorqueReaction;
        // Both current Hip arms retain Y at fixed q and can trade at most 10%
        // of the allocated Upper angular request for a better Y allocation.
        internal bool UpwardRetentionSelected, UpwardRetentionEligible, BoundedTradeSelected;
        internal PhysicsStepContext Step;
        internal int BodyId;
        internal int ChestBodyId;
        internal Vector3 Acceleration;
        internal Vector3 ReceiverTorque, ParentW;
        // V041080: the governed target consumed by THIS receiver step, not the
        // settled parent state or the response to its torque. Current normal Hip goals
        // use this separate reference domain; physical feedback still uses ParentW.
        internal Quaternion ReferenceRotation;
        internal Vector3 ReferenceW;
        internal PelvisChestPointResponse Response;
        internal bool Matches(PhysicsStepContext step, int bodyId) => Committed && BodyId != 0 && BodyId == bodyId &&
            ChestBodyId != 0 && ChestBodyId != BodyId && Step.Matches(step) && Response.Valid &&
            (!UpwardRetentionSelected || JointTorqueReaction) &&
            (!BoundedTradeSelected || (UpwardRetentionSelected && JointTorqueReaction)) &&
            (!UpwardRetentionEligible || (UpwardRetentionSelected && Acceleration.y > 0f)) &&
            PelvisParentFrame.V(Acceleration) && Acceleration.magnitude <= MaxMagnitude + 0.001f &&
            PelvisParentFrame.V(ReceiverTorque) && ReceiverTorque.magnitude <= 2.001f && PelvisParentFrame.V(ParentW) &&
            (!JointTorqueReaction || (PelvisParentFrame.Q(ReferenceRotation) && PelvisParentFrame.V(ReferenceW)));
        internal bool SameAs(PelvisHipCommandFrame other) => Matches(other.Step, other.BodyId) && other.Committed &&
            ChestBodyId == other.ChestBodyId && JointTorqueReaction == other.JointTorqueReaction && Response.SameAs(other.Response) &&
            UpwardRetentionSelected == other.UpwardRetentionSelected && UpwardRetentionEligible == other.UpwardRetentionEligible &&
            BoundedTradeSelected == other.BoundedTradeSelected &&
            PelvisChestPointResponse.Same(Acceleration, other.Acceleration) && PelvisChestPointResponse.Same(ReceiverTorque, other.ReceiverTorque) &&
            PelvisChestPointResponse.Same(ParentW, other.ParentW) &&
            (!JointTorqueReaction || (ReferenceRotation.x == other.ReferenceRotation.x && ReferenceRotation.y == other.ReferenceRotation.y &&
                ReferenceRotation.z == other.ReferenceRotation.z && ReferenceRotation.w == other.ReferenceRotation.w &&
                PelvisChestPointResponse.Same(ReferenceW, other.ReferenceW)));
    }
}
