using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        // This is a separate plant contract. FreeSpaceSelectorIssue/FreeSpaceNativeState stay strict.
        // Reuse the existing owners and their cached validated readbacks; no observer/scene scan is added.
        internal DynamicContactBoundaryIssue GetDynamicContactIssue(bool footSelfWeight)
        {
            if (!IsBuilt) return DynamicContactBoundaryIssue.MissingFoundation;
            if (_body == null || _body.isKinematic || _collider == null || _ankleJoint != null ||
                !IsFinite(_body.position) || !IsFinite(_body.rotation) || !IsFinite(_body.velocity) || !IsFinite(_body.angularVelocity))
                return DynamicContactBoundaryIssue.NativeState;
            if (_supportSurface == null || !_supportSurface.Active || !_supportSurface.Finite ||
                _supportSurface.Arm != FootSupportSurfaceIsolation.ArmB || _supportSurface.SelectedId == 0)
                return DynamicContactBoundaryIssue.SupportSurface;
            if (!_stanceContactLifecycle.Enabled || !_stanceContactLifecycle.Finite)
                return DynamicContactBoundaryIssue.ContactLifecycle;
            // Prepared-but-released contact is valid during swing. Active contact must name the exact owned Box.
            if (!_contactShadow.Finite || !_contactShadow.RuntimeContractValid(_collider) ||
                (_contactShadow.Prepared && (_contactShadow.AuthorizedColliderId != _supportSurface.SelectedId ||
                _contactShadow.AuthorizedCollider != _supportSurface.SelectedCollider)) ||
                (_contactShadow.Active && (!_contactShadow.Prepared || _contactShadow.Arm != FootQualifiedContactShadow.ArmB)))
                return DynamicContactBoundaryIssue.ContactAuthorization;
            if (!_stanceContactFrictionIsolation.ZeroFriction || !_stanceContactFrictionIsolation.Finite)
                return DynamicContactBoundaryIssue.Friction;
            if (_stanceLowerGroundAuthority.Arm != FootStanceLowerGroundAnchorAuthority.ArmA || !_stanceLowerGroundAuthority.Finite)
                return DynamicContactBoundaryIssue.GroundAuthority;
            if (_ankleConstraint.PredictiveTailCaptureArm != FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA ||
                !_ankleConstraint.PredictiveTailCaptureFinite) return DynamicContactBoundaryIssue.PredictiveCapture;
            if (!DynamicContactGravityMatches(footSelfWeight))
                return DynamicContactBoundaryIssue.FootGravity;
            if (_footCollisionDetectionMode.Arm != FootCollisionDetectionModeIsolation.ActiveArm || !_footCollisionDetectionMode.Finite ||
                _body.collisionDetectionMode != CollisionDetectionMode.ContinuousSpeculative)
                return DynamicContactBoundaryIssue.CollisionMode;
            if (_report.FootContactOffsetArm != FootContactOffsetIsolation.ArmB || !_report.FootContactOffsetInitialized ||
                !_report.FootContactOffsetFinite || !_report.FootContactOffsetReadbackValid ||
                !FootContactOffsetIsolation.Matches(_report.FootContactOffsetActualMeters, FootContactOffsetIsolation.BOffsetMeters))
                return DynamicContactBoundaryIssue.ContactOffset;
            if (!_footVelocitySolver.Finite || !_footVelocitySolver.PositionEnabled ||
                _footVelocitySolver.PositionArm != FootSolverIterationBaselineContract.PositionArmB || !_report.FootVelocitySolverReadbackValid ||
                _report.FootVelocitySolverActualVelocity != _report.FootVelocitySolverBaselineVelocity ||
                _report.FootVelocitySolverActualPosition != FootSolverIterationBaselineContract.BPositionIterations)
                return DynamicContactBoundaryIssue.Solver;
            if (_stanceSupportAngular.Arm != FootStanceSupportNormalAngularStabilization.ArmA || !_stanceSupportAngular.Finite)
                return DynamicContactBoundaryIssue.SupportAngular;
            if (_ankleConstraint.StancePlanarDampingEnabled) return DynamicContactBoundaryIssue.PlanarDamping;
            return DynamicContactBoundaryIssue.None;
        }

        private bool DynamicContactGravityMatches(bool footSelfWeight)
        {
            string required = footSelfWeight ? FootStanceNativeGravitySupportLoadIsolation.ArmB : FootStanceNativeGravitySupportLoadIsolation.ArmA;
            if (_stanceFootNativeGravity.Arm != required || !_stanceFootNativeGravity.Finite ||
                _body.useGravity != _stanceFootNativeGravity.CurrentApplied) return false;
            if (!_body.useGravity) return true;
            // Before the next Foot controller clears gravity, the previous completed stance solve may own it.
            // ENTER/EXIT callbacks describe the solved contact, not the eligibility used to issue
            // that command. Reject unowned/stale schedules here; current authorization is checked
            // above, and the next lifecycle releases gravity before any further physics advance.
            return footSelfWeight &&
                _report.StanceFootNativeGravityCurrentScheduledFixed > 0 &&
                _report.StanceFootNativeGravityCurrentScheduledFixed == _report.FootControllerLastCompletedFixedSequence;
        }
    }
}
