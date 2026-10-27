using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Same-step planner state; control consumes the owner's Applied/A. Leg evidence stores a value copy.
    // V04790 may use that diagnostic copy's gap to select an existing window, never to change control.
    internal struct HipNormalReferenceState
    {
        internal PhysicsStepContext Step;
        internal bool Valid, Selected, Applied, AlphaLimited, SpeedLimited, SampleAgePrediction;
        internal string Reason;
        internal Quaternion PreviousR, InputR;
        internal Vector3 PreviousW, InputW, A;
        internal float PredictionSeconds, MaxW, MaxA;
        internal bool Matches(PhysicsStepContext step) => Valid && Step.Matches(step);
    }
}
