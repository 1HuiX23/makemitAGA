using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Exact same command previously read through the torso getters. Captured once
    // AFTER torso command N and shared by both legs BEFORE either writes forces.
    // This stamp proves command provenance, not native-solver completion. Each
    // typed scope separately requires its physical parent and committed inputs.
    internal struct LegParentCommandFrame
    {
        internal PhysicsStepContext Step;
        internal Vector3 Position;
        internal Quaternion Rotation;
        // Dynamic handoff keeps the settled native state separate from the governed target.
        internal PelvisParentFrame PhysicalParent;
        // Hip-coupled scope only: actual committed receiver input, distinct from pose/reference data.
        internal PelvisHipCommandFrame HipCommand;
        internal bool Valid => Step.Valid && PelvisParentFrame.V(Position) && PelvisParentFrame.Q(Rotation);
        internal bool Matches(PhysicsStepContext step) => Valid && Step.Matches(step);
    }
}
