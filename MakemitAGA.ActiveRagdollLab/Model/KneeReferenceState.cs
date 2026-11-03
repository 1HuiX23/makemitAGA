using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Same-Fixed, Upper-local relative reference. A is a reference derivative,
    // not a native response or an extra actuator outside the original FF gate.
    internal struct KneeReferenceState
    {
        internal PhysicsStepContext Step;
        internal SourceStepContext Source;
        internal bool Valid, Selected, Stale, AlphaLimited, SpeedLimited;
        internal Quaternion PreviousR, InputR, R;
        internal Vector3 PreviousW, InputW, W, A;
        internal float MaxW, MaxA;
        internal bool Matches(PhysicsStepContext step, SourceStepContext source) =>
            Valid && Selected && Step.Matches(step) && Source.Matches(source) &&
            PelvisParentFrame.Q(PreviousR) && PelvisParentFrame.Q(InputR) && PelvisParentFrame.Q(R) &&
            PelvisParentFrame.V(PreviousW) && PelvisParentFrame.V(InputW) &&
            PelvisParentFrame.V(W) && PelvisParentFrame.V(A) &&
            PelvisParentFrame.F(MaxW) && MaxW > 0f && PelvisParentFrame.F(MaxA) && MaxA > 0f;
    }
}
