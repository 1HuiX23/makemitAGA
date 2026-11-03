using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Value view for callers selecting scopes and checking receiver readiness.
    internal readonly struct PelvisReceiverControlStatus
    {
        internal string Arm { get; }
        internal bool Selected { get; }
        internal bool Active { get; }
        internal bool Ready { get; }
        internal bool Finite { get; }
        internal bool BaselineCaptured { get; }

        internal PelvisReceiverControlStatus(string arm, bool selected, bool active,
            bool ready, bool finite, bool baselineCaptured)
        {
            Arm = arm; Selected = selected; Active = active;
            Ready = ready; Finite = finite; BaselineCaptured = baselineCaptured;
        }
    }

    // The receiver owns selection/readiness and its sampled native configuration.
    // Observe/Publish calls stay at the original individual assignments: partial
    // native-read failures must not pretend that a complete new snapshot exists.
    // No native operation, trace ring, controller gain, or source clock lives here.
    internal sealed class PelvisReceiverControlState
    {
        internal string Arm { get; private set; } = PelvisDynamicReceiverIsolation.IdleArm;
        internal bool Selected { get; private set; }
        internal bool Dynamic { get; private set; }
        internal bool Active { get; private set; }
        internal bool Ready { get; private set; }
        internal bool Finite { get; private set; } = true;
        internal bool BaselineCaptured { get; private set; }
        internal bool BaselineRestored { get; private set; } = true;
        internal float PelvisMass { get; private set; }
        internal float ChestMass { get; private set; }
        internal Vector3 PelvisComLocal { get; private set; }
        internal Vector3 ChestComLocal { get; private set; }
        internal Vector3 PelvisInertia { get; private set; }
        internal Vector3 ChestInertia { get; private set; }
        internal Vector3 NativeChestAnchor { get; private set; }
        internal Vector3 NativePelvisAnchor { get; private set; }
        internal float ReferenceAlphaCap { get; private set; }
        internal int PassiveIssued { get; private set; }
        internal int PassiveResolved { get; private set; }
        internal int RestoreFailures { get; private set; }

        internal PelvisReceiverControlStatus Status => new PelvisReceiverControlStatus(
            Arm, Selected, Active, Ready, Finite, BaselineCaptured);
        internal bool DynamicSelected => Selected && Dynamic;
        internal bool ReadyForRun => Selected && Finite && Active && Ready;

        // Return values feed the report projection at the existing publication point.
        internal string ObserveArm(string value) => Arm = value;
        internal bool ObserveSelected(bool value) => Selected = value;
        internal bool ObserveDynamic(bool value) => Dynamic = value;
        internal bool ObserveActive(bool value) => Active = value;
        internal bool ObserveReady(bool value) => Ready = value;
        internal bool ObserveFinite(bool value) => Finite = value;
        internal bool ObserveBaselineCaptured(bool value) => BaselineCaptured = value;
        internal bool ObserveBaselineRestored(bool value) => BaselineRestored = value;
        internal float ObservePelvisMass(float value) => PelvisMass = value;
        internal float ObserveChestMass(float value) => ChestMass = value;
        internal Vector3 ObservePelvisComLocal(Vector3 value) => PelvisComLocal = value;
        internal Vector3 ObserveChestComLocal(Vector3 value) => ChestComLocal = value;
        internal Vector3 ObservePelvisInertia(Vector3 value) => PelvisInertia = value;
        internal Vector3 ObserveChestInertia(Vector3 value) => ChestInertia = value;
        internal Vector3 ObserveNativeChestAnchor(Vector3 value) => NativeChestAnchor = value;
        internal Vector3 ObserveNativePelvisAnchor(Vector3 value) => NativePelvisAnchor = value;
        internal float ObserveReferenceAlphaCap(float value) => ReferenceAlphaCap = value;

        // These are control receipts: only a completed passive native check may
        // advance readiness. Trace resets never clear this qualification.
        internal int ResetPassiveQualification() { PassiveIssued = PassiveResolved = 0; return 0; }
        internal int IssuePassiveStep() => ++PassiveIssued;
        internal int ResolvePassiveStep() => ++PassiveResolved;
        // Restoration compares the operation-owned count before/after independent
        // attempts; the report counter cannot decide whether cleanup succeeded.
        internal int RecordRestoreFailure() => ++RestoreFailures;
    }
}
