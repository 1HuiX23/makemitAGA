using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // These native points are available in compact capture even when the landing
    // reference was not requested. They belong only to the completed leg command.
    internal readonly struct LegMeasuredAnchors
    {
        internal readonly bool Valid;
        internal readonly Vector3 UpperHip, UpperKnee, LowerKnee;
        private LegMeasuredAnchors(Vector3 hip, Vector3 upperKnee, Vector3 lowerKnee)
        { Valid = true; UpperHip = hip; UpperKnee = upperKnee; LowerKnee = lowerKnee; }

        internal static LegMeasuredAnchors Capture(bool committed, in PhysicsStepContext owner, int ownerLocal,
            in PhysicsStepContext step, int local, Vector3 hip, Vector3 upperKnee, Vector3 lowerKnee) =>
            committed && local > 0 && ownerLocal == local && owner.Matches(step) &&
            PelvisParentFrame.V(hip) && PelvisParentFrame.V(upperKnee) && PelvisParentFrame.V(lowerKnee)
                ? new LegMeasuredAnchors(hip, upperKnee, lowerKnee) : default;
    }
}
