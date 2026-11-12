using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Optional same-input evidence, deliberately excluded from physical receipt validity.
    // A damaged/missing audit context must never reject an otherwise valid force command.
    internal struct HipAllocationAuditContext
    {
        internal bool Selected, Eligible;
        internal PhysicsStepContext Step;
        internal float ErrorY;
        internal static HipAllocationAuditContext Capture(bool selected, PhysicsStepContext step,
            PhysicsStepContext referenceStep, PelvisComReferencePlanner.ComPlan reference, float errorY, Vector3 request) =>
            !selected ? default : new HipAllocationAuditContext { Selected = true, Step = step, ErrorY = errorY,
                Eligible = HipSystemAllocation.Eligible(true, step, referenceStep, reference, errorY, request) };
        internal bool Matches(PelvisHipCommandFrame frame) => Selected && Step.Matches(frame.Step) &&
            HipPointPairMobility.F(ErrorY) && (!Eligible || (ErrorY > 0f && frame.Acceleration.y > 0f));
    }
}
