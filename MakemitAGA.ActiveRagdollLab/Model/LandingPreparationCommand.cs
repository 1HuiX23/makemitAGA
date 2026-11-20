using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Optional immutable diagnostic; no controller reads this receipt.
    internal struct LandingPreparationCommand
    {
        internal PhysicsStepContext Step;
        internal SourceStepContext Source;
        internal bool HipApplied, KneeApplied, Approach;
        internal float HipGain, KneeGain;
        internal int SupportId;
        internal Vector3 Center, Half, FloorCenter, FloorHalf;
        // Optional V141 geometry, separate from the actual-response qualification.
        internal bool GeometryPrepared;
        // RenderPhysicalHip already contains physical parent displacement.
        internal Vector3 RenderPhysicalHip, UpperSpan, LowerSpan;
        internal Quaternion HipBaseline, KneeBaseline, HipSelected, KneeSelected, PublishedFoot;
        internal bool Applied => HipApplied || KneeApplied;
    }
}
