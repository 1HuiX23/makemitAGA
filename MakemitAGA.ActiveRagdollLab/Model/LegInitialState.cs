using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Pure geometry: preserve the fixed anatomical anchors and the held local source pose.
    // Velocities belong to COMs. Point continuity is reconstructed at Hip and Knee.
    internal struct LegInitialState
    {
        internal Vector3 UpperPosition, LowerPosition, UpperVelocity, LowerVelocity, UpperW, LowerW;
        internal Quaternion UpperRotation, LowerRotation;
        internal bool Finite => PelvisParentFrame.V(UpperPosition) && PelvisParentFrame.V(LowerPosition) &&
            PelvisParentFrame.V(UpperVelocity) && PelvisParentFrame.V(LowerVelocity) &&
            PelvisParentFrame.V(UpperW) && PelvisParentFrame.V(LowerW) &&
            PelvisParentFrame.Q(UpperRotation) && PelvisParentFrame.Q(LowerRotation);

        internal static LegInitialState Create(PelvisParentFrame parent, Vector3 hipLocal,
            Quaternion hipRelative, Vector3 hipRelativeW, Quaternion kneeRelative, Vector3 kneeRelativeW,
            Vector3 kneeUpperLocal, Vector3 kneeLowerLocal, Vector3 upperComLocal, Vector3 lowerComLocal)
        {
            var s = new LegInitialState();
            s.UpperPosition = parent.SettledPoint(hipLocal); // Hip child anchor is the Upper origin.
            s.UpperRotation = parent.Rotation * hipRelative;
            s.UpperW = parent.W + parent.Rotation * hipRelativeW;
            s.UpperVelocity = parent.SettledPointVelocity(hipLocal) + Vector3.Cross(s.UpperW, s.UpperRotation * upperComLocal);
            Vector3 knee = s.UpperPosition + s.UpperRotation * kneeUpperLocal;
            Vector3 kneeVelocity = s.UpperVelocity + Vector3.Cross(s.UpperW, s.UpperRotation * (kneeUpperLocal - upperComLocal));
            s.LowerRotation = s.UpperRotation * kneeRelative;
            s.LowerPosition = knee - s.LowerRotation * kneeLowerLocal;
            s.LowerW = s.UpperW + s.UpperRotation * kneeRelativeW;
            s.LowerVelocity = kneeVelocity + Vector3.Cross(s.LowerW, s.LowerRotation * (lowerComLocal - kneeLowerLocal));
            return s;
        }
    }
}
