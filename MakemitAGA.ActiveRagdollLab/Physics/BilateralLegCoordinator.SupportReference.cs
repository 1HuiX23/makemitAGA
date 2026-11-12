using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralLegCoordinator
    {
        internal SupportReferenceContext CaptureSupportReference(PhysicsStepContext next)
        {
            if (!IsBuilt || !DriveEnabled || _primary == null || _secondary == null ||
                !PelvisRunScopes.HasHipCoupling(_primary.DynamicScope) || _primary.DynamicScope != _secondary.DynamicScope ||
                !_supportSurface.Active || !_supportSurface.Finite || _supportSurface.Arm != FootSupportSurfaceIsolation.ArmB)
                return default;
            var input = SupportReferenceContext.Create(next, _primary.PreviousReferenceSupport,
                _secondary.PreviousReferenceSupport, _supportSurface.SelectedId,
                _torso != null && PelvisDynamicReceiverIsolation.IsDynamicHipArm(_torso.PelvisReceiverControl.Arm));
            if (!input.Matches(next)) return default;
            // Existing owned-Box contract: current identity, static horizontal pose,
            // enabled state and dimensions. No new raycast or contact authorization.
            return _supportSurface.TryGetEntryBox(input.SupportId, out _, out _) ? input : default;
        }
    }
}
