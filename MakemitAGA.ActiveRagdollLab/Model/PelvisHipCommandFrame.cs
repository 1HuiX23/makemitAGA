using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Same-step native parent model plus the actual committed receiver inputs.
    // Published only after BOTH native force writes and the receiver torque return.
    // Actual common A still drives Hip FF; Response supplies the Hip point mobility.
    // This is the immutable BASELINE receipt (torque <= 2 Nm), never the final
    // compensated torque. Bilateral completion publishes a separate receipt.
    internal struct PelvisHipCommandFrame
    {
        internal const float MaxMagnitude = 12f;
        internal bool Committed;
        internal PhysicsStepContext Step;
        internal int BodyId;
        internal int ChestBodyId;
        internal Vector3 Acceleration;
        internal Vector3 ReceiverTorque, ParentW;
        internal PelvisChestPointResponse Response;
        internal bool Matches(PhysicsStepContext step, int bodyId) => Committed && BodyId != 0 && BodyId == bodyId &&
            ChestBodyId != 0 && ChestBodyId != BodyId && Step.Matches(step) && Response.Valid &&
            PelvisParentFrame.V(Acceleration) && Acceleration.magnitude <= MaxMagnitude + 0.001f &&
            PelvisParentFrame.V(ReceiverTorque) && ReceiverTorque.magnitude <= 2.001f && PelvisParentFrame.V(ParentW);
        internal bool SameAs(PelvisHipCommandFrame other) => Matches(other.Step, other.BodyId) && other.Committed &&
            ChestBodyId == other.ChestBodyId && Response.SameAs(other.Response) &&
            PelvisChestPointResponse.Same(Acceleration, other.Acceleration) && PelvisChestPointResponse.Same(ReceiverTorque, other.ReceiverTorque) &&
            PelvisChestPointResponse.Same(ParentW, other.ParentW);
    }
}
