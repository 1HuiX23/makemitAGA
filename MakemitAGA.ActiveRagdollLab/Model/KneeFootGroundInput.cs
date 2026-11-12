using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Render capture inputs and the last COMPLETED control identity are distinct.
    // The native Hip point/Upper rotation are read together in Render, never
    // extrapolated from a diagnostic frame. No collision/load authority lives here.
    internal struct KneeFootGroundInput
    {
        internal SourceStepContext Source;
        internal PhysicsStepContext CompletedStep;
        internal int ParentId, UpperId, LowerId;
        internal bool Enabled, Apply;
        internal Vector3 Hip, UpperSpan, LowerKneeAnchor, Axis;
        internal Quaternion Upper, RawKnee;
        internal bool Matches(SourceStepContext source) => Enabled && Source.Matches(source) &&
            CompletedStep.Valid && CompletedStep.Epoch == source.Epoch &&
            CompletedStep.SourceSequence > 0 && CompletedStep.SourceSequence < source.Sequence &&
            source.Time + .0001f >= CompletedStep.UnscaledTime &&
            source.Time - CompletedStep.UnscaledTime <= .1f && source.Dt < .1f &&
            ParentId != 0 && UpperId != 0 && LowerId != 0 && ParentId != UpperId &&
            ParentId != LowerId && UpperId != LowerId &&
            PelvisParentFrame.V(Hip) && PelvisParentFrame.V(UpperSpan) &&
            PelvisParentFrame.V(LowerKneeAnchor) && PelvisParentFrame.V(Axis) &&
            PelvisParentFrame.Q(Upper) && PelvisParentFrame.Q(RawKnee);
    }

    // A source packet is immutable after Foot's existing Render planner consumes
    // it. Fixed Knee consumers may hold it; they never advance the Foot planner.
    internal struct KneeFootGroundPacket
    {
        internal KneeFootGroundInput Input;
        internal KneeGroundTarget.Geometry Geometry;
        internal KneeGroundTarget.Sample Sample;
        internal int SupportId;
        internal bool Published;

        internal bool Matches(PhysicsStepContext step, SourceStepContext source,
            int parentId, int upperId, int lowerId, int supportId) =>
            Published && Sample.Valid && Geometry.Valid && Input.Matches(source) && Sample.Source.Matches(source) &&
            Sample.GeometryStep.Matches(Input.CompletedStep) && PelvisParentFrame.Q(Sample.PlannedFoot) &&
            PelvisParentFrame.Q(Sample.Goal) &&
            step.Valid && step.Epoch == source.Epoch && step.SourceSequence == source.Sequence &&
            step.Sequence > Input.CompletedStep.Sequence && step.FixedTime >= Input.CompletedStep.FixedTime &&
            step.UnscaledTime + .0001f >= Input.CompletedStep.UnscaledTime &&
            step.UnscaledTime - Input.CompletedStep.UnscaledTime <= .1f &&
            SupportId != 0 && SupportId == supportId && Input.ParentId == parentId &&
            Input.UpperId == upperId && Input.LowerId == lowerId;
    }
}
