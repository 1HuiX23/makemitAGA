using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Read-only projection of the EXISTING initializer, never a live-body measurement.
    // Geometry.ActualMin here means projected initialized Foot, not an observed solve.
    internal struct HipJoinPose
    {
        internal InitialGeometryPreflight Geometry;
        internal Vector3 Hip, Knee, Ankle, RawHip, RawFoot;
        internal Quaternion Upper, Lower, Foot, RawUpper, RawLower, RawFootRotation;
        internal bool Finite => PelvisParentFrame.V(Hip) && PelvisParentFrame.V(Knee) &&
            PelvisParentFrame.V(Ankle) && PelvisParentFrame.V(RawHip) && PelvisParentFrame.V(RawFoot) &&
            PelvisParentFrame.Q(Upper) && PelvisParentFrame.Q(Lower) && PelvisParentFrame.Q(Foot) &&
            PelvisParentFrame.Q(RawUpper) && PelvisParentFrame.Q(RawLower) && PelvisParentFrame.Q(RawFootRotation);
        internal bool StableAgainst(HipJoinPose p) => NearPoint(Hip, p.Hip) && NearPoint(Knee, p.Knee) && NearPoint(Ankle, p.Ankle) &&
            NearPoint(RawHip, p.RawHip) && NearPoint(RawFoot, p.RawFoot) && NearRotation(Upper, p.Upper) && NearRotation(Lower, p.Lower) &&
            NearRotation(Foot, p.Foot) && NearRotation(RawUpper, p.RawUpper) && NearRotation(RawLower, p.RawLower) && NearRotation(RawFootRotation, p.RawFootRotation);
        private static bool NearPoint(Vector3 a, Vector3 b) => (a-b).magnitude <= FoundationPoseWindow.MaxPositionDrift;
        private static bool NearRotation(Quaternion a, Quaternion b) => Quaternion.Angle(a,b) <= FoundationPoseWindow.MaxRotationDrift;
    }
}
