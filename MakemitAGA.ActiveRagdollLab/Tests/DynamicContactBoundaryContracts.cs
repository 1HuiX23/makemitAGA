using System;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Observation ports only. The coordinator's final readback and the leg's Knee
    // selection/reset are linked production code. Native setup/solves are not simulated.
    internal static class FootSupportSurfaceIsolation
    {
        internal const string ArmB = "B_V04410_STATIC_BOX_SUPPORT";
    }

    internal sealed partial class SingleLegShadowChain
    {
        internal PelvisRunScope DynamicScope => _dynamicScope;
        internal bool DynamicContactSelected => PelvisRunScopes.IsContact(_dynamicScope);
        internal DynamicContactBoundaryIssue DynamicContactIssue = DynamicContactBoundaryIssue.None;
        internal bool ContactEntryContinuous, ContactEntryRecovery, StanceLoadAdmissionSelected;
        internal bool GroundReferenceEnabled, GroundReferenceApplied, GroundReferencePartial;
        internal object FootContactShadowCollider = new object();

        internal static SingleLegShadowChain ContactBoundaryFixture(bool b, PelvisRunScope scope)
        {
            var leg = Joined(b); // Real ResetKneeReferenceForDrive, not a preset Knee flag.
            leg.SetBoundaryScope(scope);
            bool hip = PelvisRunScopes.HasHipCoupling(scope);
            if (!hip) leg.DisableKneeReferencePreserveTrace();
            leg.ContactEntryContinuous = leg.ContactEntryRecovery = leg.StanceLoadAdmissionSelected = hip;
            leg.GroundReferenceEnabled = leg.GroundReferenceApplied = leg.GroundReferencePartial = hip;
            return leg;
        }

        internal void SetBoundaryScope(PelvisRunScope scope) { _dynamicScope = scope; }

        internal static readonly Action<SingleLegShadowChain>[] ContactBoundarySelectionFaults = {
            l => l.ContactEntryContinuous = false,
            l => l.ContactEntryRecovery = false,
            l => l.StanceLoadAdmissionSelected = false,
            l => l.GroundReferenceEnabled = false,
            l => l.GroundReferenceApplied = false,
            l => l.GroundReferencePartial = false,
            l => l._kneeGroundTargetSelected = false,
            l => l._kneeReference.DisablePreserveTrace(),
            l => l._kneePlannedFeedforwardSelected = false
        };

        internal void DisableBoundaryKnee() { DisableKneeReferencePreserveTrace(); }
        internal void InjectUnexpectedJointInput() { _kneeGroundTargetSelected = true; }
    }

    internal sealed partial class BilateralLegCoordinator
    {
        private SingleLegShadowChain _primary, _secondary;
        private bool IsBuilt = true, _bilateralEnabled = true, _contactShadowSelfPairReadback = true;
        private object _contactShadowSelfPairPrimary, _contactShadowSelfPairSecondary;
        private readonly SupportObservation _supportSurface = new SupportObservation();
        private TorsoObservation _torso = new TorsoObservation();
        private sealed class SupportObservation
        {
            internal bool Active = true, Finite = true;
            internal string Arm = FootSupportSurfaceIsolation.ArmB;
        }
        private sealed class TorsoObservation
        {
            internal readonly ArmObservation PelvisReceiverControl = new ArmObservation();
        }
        private sealed class ArmObservation { internal string Arm; }

        private static BilateralLegCoordinator BoundaryFixture(bool b,
            PelvisRunScope scope = PelvisRunScope.DynamicLegHipAnchorCouplingContact)
        {
            var pair = new BilateralLegCoordinator
            {
                _primary = SingleLegShadowChain.ContactBoundaryFixture(b, scope),
                _secondary = SingleLegShadowChain.ContactBoundaryFixture(b, scope)
            };
            pair._torso.PelvisReceiverControl.Arm = b
                ? PelvisDynamicReceiverIsolation.DynamicHipArmB : PelvisDynamicReceiverIsolation.DynamicHipArmA;
            pair._contactShadowSelfPairPrimary = pair._primary.FootContactShadowCollider;
            pair._contactShadowSelfPairSecondary = pair._secondary.FootContactShadowCollider;
            return pair;
        }

        internal static void RunDynamicContactBoundaryContracts()
        {
            Check.Run("A final contact readback accepts real shared Knee selection", () => SharedJointInput(false));
            Check.Run("B final contact readback accepts real shared Knee selection", () => SharedJointInput(true));
            Check.Run("both legs retain every contact selection guard in both arms", SelectionFaults);
            Check.Run("bilateral readback retains foundation, scope, support and identity guards", BoundaryFaults);
            Check.Run("bilateral readback propagates child failure and preserves first cause", ChildFaults);
            Check.Run("legacy contact scopes retain joint-input-off boundary", LegacyContactScopes);
            Check.Run("disabled Knee selection cannot reuse an earlier accepted boundary", StoppedSelection);
        }

        private static void SharedJointInput(bool b)
        {
            var pair = BoundaryFixture(b);
            Check.That(pair._primary.KneeGroundTargetSelected && pair._secondary.KneeGroundTargetSelected,
                "both Knee owners actually select the common v126 joint input");
            Check.That(pair._primary.DynamicContactIssue == DynamicContactBoundaryIssue.None &&
                pair._secondary.DynamicContactIssue == DynamicContactBoundaryIssue.None,
                "logged fixture: no child boundary failure");
            Check.That(pair.DynamicContactIssue == DynamicContactBoundaryIssue.None &&
                pair.DynamicContactMatchesScope(PelvisRunScope.DynamicLegHipAnchorCouplingContact),
                "same final readback used by PrepareDynamicContact must accept arm B=" + b);
            Check.That(!pair.DynamicContactMatchesScope(PelvisRunScope.DynamicLegUpperSelfWeightContact) &&
                !pair.DynamicContactMatchesScope(PelvisRunScope.DynamicLegFreeSpace),
                "a valid Hip receipt does not authorize another scope");
        }

        private static void SelectionFaults()
        {
            foreach (bool b in new[] { false, true }) foreach (bool secondary in new[] { false, true })
            for (int i = 0; i < SingleLegShadowChain.ContactBoundarySelectionFaults.Length; i++)
            {
                var pair = BoundaryFixture(b);
                SingleLegShadowChain.ContactBoundarySelectionFaults[i](secondary ? pair._secondary : pair._primary);
                Check.That(pair.DynamicContactIssue == DynamicContactBoundaryIssue.ContactLifecycle &&
                    !pair.DynamicContactMatchesScope(PelvisRunScope.DynamicLegHipAnchorCouplingContact),
                    "missing selector rejected: B=" + b + ", secondary=" + secondary + ", fault=" + i);
            }
        }

        private static void BoundaryFaults()
        {
            var faults = new (Action<BilateralLegCoordinator> Apply, DynamicContactBoundaryIssue Issue)[] {
                (p => p.IsBuilt = false, DynamicContactBoundaryIssue.MissingFoundation),
                (p => p._bilateralEnabled = false, DynamicContactBoundaryIssue.MissingFoundation),
                (p => p._primary = null, DynamicContactBoundaryIssue.MissingFoundation),
                (p => p._secondary = null, DynamicContactBoundaryIssue.MissingFoundation),
                (p => p._primary.SetBoundaryScope(PelvisRunScope.DynamicLegFreeSpace), DynamicContactBoundaryIssue.Scope),
                (p => p._secondary.SetBoundaryScope(PelvisRunScope.DynamicLegUnloadedContact), DynamicContactBoundaryIssue.Scope),
                (p => p._supportSurface.Active = false, DynamicContactBoundaryIssue.SupportSurface),
                (p => p._supportSurface.Finite = false, DynamicContactBoundaryIssue.SupportSurface),
                (p => p._supportSurface.Arm = "OTHER", DynamicContactBoundaryIssue.SupportSurface),
                (p => p._contactShadowSelfPairReadback = false, DynamicContactBoundaryIssue.FootSelfPair),
                (p => p._primary.FootContactShadowCollider = new object(), DynamicContactBoundaryIssue.FootSelfPair),
                (p => p._secondary.FootContactShadowCollider = new object(), DynamicContactBoundaryIssue.FootSelfPair),
                (p => p._torso = null, DynamicContactBoundaryIssue.ContactLifecycle),
                (p => p._torso.PelvisReceiverControl.Arm = "OTHER", DynamicContactBoundaryIssue.ContactLifecycle)
            };
            foreach (bool b in new[] { false, true }) foreach (var fault in faults)
            {
                var pair = BoundaryFixture(b); fault.Apply(pair);
                Check.That(pair.DynamicContactIssue == fault.Issue &&
                    !pair.DynamicContactMatchesScope(PelvisRunScope.DynamicLegHipAnchorCouplingContact),
                    "original guard rejects " + fault.Issue + ", B=" + b);
            }
        }

        private static void ChildFaults()
        {
            foreach (bool b in new[] { false, true }) foreach (bool secondary in new[] { false, true })
            foreach (DynamicContactBoundaryIssue issue in Enum.GetValues(typeof(DynamicContactBoundaryIssue)))
            {
                if (issue == DynamicContactBoundaryIssue.None) continue;
                var pair = BoundaryFixture(b);
                (secondary ? pair._secondary : pair._primary).DynamicContactIssue = issue;
                Check.That(pair.DynamicContactIssue == issue && !pair.DynamicContactReady,
                    "child issue retained: " + issue + ", secondary=" + secondary + ", B=" + b);
            }
            var both = BoundaryFixture(false);
            both._primary.DynamicContactIssue = DynamicContactBoundaryIssue.NativeState;
            both._secondary.DynamicContactIssue = DynamicContactBoundaryIssue.FootGravity;
            Check.That(both.DynamicContactIssue == DynamicContactBoundaryIssue.NativeState,
                "primary child remains the first failed owner");
        }

        private static void LegacyContactScopes()
        {
            foreach (PelvisRunScope scope in Enum.GetValues(typeof(PelvisRunScope)))
            {
                if (!PelvisRunScopes.IsContact(scope) || PelvisRunScopes.HasHipCoupling(scope)) continue;
                var pair = BoundaryFixture(false, scope);
                pair._torso.PelvisReceiverControl.Arm = "LEGACY";
                Check.That(pair.DynamicContactMatchesScope(scope), "legacy contact boundary unchanged: " + scope);
                pair._secondary.InjectUnexpectedJointInput();
                Check.That(pair.DynamicContactIssue == DynamicContactBoundaryIssue.ContactLifecycle,
                    "joint input cannot leak into " + scope);
            }
        }

        private static void StoppedSelection()
        {
            foreach (bool b in new[] { false, true })
            {
                var pair = BoundaryFixture(b);
                Check.That(pair.DynamicContactReady, "initial selection accepted");
                pair._primary.DisableBoundaryKnee(); pair._secondary.DisableBoundaryKnee();
                Check.That(!pair.DynamicContactReady, "disabled live selection cannot reuse accepted readback");
                Check.That(BoundaryFixture(b).DynamicContactReady, "a fresh prepared pair can qualify again");
            }
        }
    }
}
