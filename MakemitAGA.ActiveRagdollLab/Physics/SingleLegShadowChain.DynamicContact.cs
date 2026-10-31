using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        private PelvisRunScope _dynamicScope = PelvisRunScope.DynamicLegFreeSpace;
        internal PelvisRunScope DynamicScope => _dynamicScope;
        internal bool DynamicContactSelected => PelvisRunScopes.IsContact(_dynamicScope);
        internal DynamicContactBoundaryIssue DynamicContactIssue => GetDynamicContactIssue(_dynamicScope);
        private DynamicContactBoundaryIssue GetDynamicContactIssue(PelvisRunScope scope)
        {
            if (!IsBuilt || _footFoundation == null) return DynamicContactBoundaryIssue.MissingFoundation;
            if (!DynamicParentActive || _gravityEnabled || _collidersEnabled || _upperBody.isKinematic || _lowerBody.isKinematic ||
                (!PelvisRunScopes.HasUpperWeight(scope) && _upperBody.useGravity) || _upperCollider.enabled || _lowerCollider.enabled)
                return DynamicContactBoundaryIssue.NativeState;
            DynamicContactBoundaryIssue foot = _footFoundation.GetDynamicContactIssue(PelvisRunScopes.HasFootWeight(scope));
            if (foot != DynamicContactBoundaryIssue.None) return foot;
            bool lowerWeight = PelvisRunScopes.HasLowerWeight(scope);
            if (!DynamicLowerGravityMatches(lowerWeight))
                return DynamicContactBoundaryIssue.LowerGravity;
            if (_stanceFootLowerReciprocal.Arm != FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmA || !_stanceFootLowerReciprocal.Finite)
                return DynamicContactBoundaryIssue.ReciprocalLoad;
            if (_stanceLowerSelfWeightTransfer.Arm != (lowerWeight ? LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB : LowerSelfWeightSupportNormalLoadTransferIsolation.ArmA) ||
                !_stanceLowerSelfWeightTransfer.Finite || (lowerWeight && !DynamicLowerWeightPairMatches()))
                return DynamicContactBoundaryIssue.LowerSelfWeight;
            if (PelvisRunScopes.HasUpperWeight(scope) ? !DynamicUpperWeightMatches() : (!UpperSelfWeightOff || !UpperSelfWeightFinite)) return DynamicContactBoundaryIssue.UpperSelfWeight;
            if (PelvisRunScopes.HasKneeCoupling(scope) ? !DynamicKneeCouplingMatches() :
                (!KneeSupportReactionOff || !KneeSupportReactionFinite)) return DynamicContactBoundaryIssue.KneeReaction;
            if (!DynamicHipPairMatches(scope)) return DynamicContactBoundaryIssue.HipTranslationPair;
            return DynamicContactBoundaryIssue.None;
        }
        private bool DynamicHipPairMatches(PelvisRunScope scope)
        {
            if (!PelvisRunScopes.HasHipCoupling(scope)) return _hipAnchorPair.Off;
            HipAnchorPairReport r = _hipAnchorPair.Report;
            if (!r.Selected || !r.Finite || r.RollbackFailures != 0 || r.ChildId != _upperBody.GetInstanceID() || r.ParentId != _pelvisBody.GetInstanceID()) return false;
            if (r.Current.Fixed == 0) return _dynamicParentSteps == 0 && !r.CurrentApplied;
            return r.CurrentApplied && r.Current.Fixed == _fixedUpdateSequence && r.Current.Step.Matches(_runtimeStep) &&
                r.Current.ChildId == r.ChildId && r.Current.ParentId == r.ParentId;
        }

        // A narrow preparation boundary, called synchronously just after continuous join, before
        // any child control/trace/contact step. The general drive-OFF selector remains unchanged.
        internal void PrepareDynamicKneeCoupling(PelvisRunScope scope)
        {
            if (!PelvisRunScopes.HasKneeCoupling(scope) || !DynamicParentActive || _traceEnabled ||
                _dynamicScope != PelvisRunScope.DynamicLegFreeSpace || _dynamicParentSteps != 0 || _dynamicJoinPending ||
                !FreeSpaceSelectorsOff || !FreeSpaceNativeReady || _hipJoint != null || _kneeJoint != null ||
                !_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled || !_kneeCharacterJointResponseIsolation.Enabled ||
                _kneeCharacterJointResponseIsolation.DeferredForcePending)
                throw new InvalidOperationException("DYNAMIC_KNEE_COUPLING_REQUIRES_NEW_JOIN_AND_FREE_SPACE_BASELINE");
            _kneeSupportReaction.Select(true);
            if (!KneeSupportNormalReactionIsolation.SelectorContract(_kneeSupportReaction.Report, KneeSupportNormalReactionIsolation.ArmB))
                throw new InvalidOperationException("DYNAMIC_KNEE_COUPLING_SELECTOR_READBACK_FAILED");
            if (PelvisRunScopes.HasUpperWeight(scope) && !_upperSelfWeightTransfer.Select(UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB, _upperBody))
                throw new InvalidOperationException("DYNAMIC_UPPER_LOAD_SELECTOR_READBACK_FAILED");
            if (PelvisRunScopes.HasHipCoupling(scope) && !_hipAnchorPair.Prepare(_upperBody, _pelvisBody))
                throw new InvalidOperationException("DYNAMIC_HIP_PAIR_PREPARATION_FAILED");
        }

        private bool DynamicUpperWeightMatches()
        {
            UpperSelfWeightSupportReport upper = _upperSelfWeightTransfer.Report;
            if (_upperBody.useGravity != upper.CurrentApplied) return false;
            if (upper.CurrentFixed == 0)
                return _dynamicParentSteps == 0 && UpperSelfWeightSupportNormalLoadTransferIsolation.SelectorContract(upper,
                    UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB, true);
            return UpperSelfWeightSupportNormalLoadTransferIsolation.DynamicCommandMatches(upper, _report,
                _footFoundation.Report, _fixedUpdateSequence, _stanceLowerNativeGravity.CurrentApplied);
        }

        private bool DynamicKneeCouplingMatches()
        {
            KneeSupportNormalReactionReport knee = _kneeSupportReaction.Report;
            if (!KneeSupportNormalReactionIsolation.SelectorContract(knee, KneeSupportNormalReactionIsolation.ArmB) ||
                !_kneeCharacterJointResponseIsolation.Enabled || _kneeJoint != null ||
                _kneeCharacterJointResponseIsolation.DeferredForcePending || !IsFinite(knee.CurrentUpperForceWorld)) return false;
            if (knee.CurrentFixed == 0)
                return _dynamicParentSteps == 0 && knee.CurrentSourceFixed == 0 && !knee.CurrentEligible &&
                    !knee.CurrentApplied && !knee.CurrentNormalConstraint &&
                    !_kneeCharacterJointResponseIsolation.CurrentSupportNormalConstraint && knee.CurrentUpperForceWorld == UnityEngine.Vector3.zero;
            // Validate the LAST completed pre-solve command, not contact eligibility changed by
            // a subsequent ENTER/EXIT callback. The Lower gravity owner was scheduled after this
            // Knee command with the same eligibility; the boundary above checks its native state,
            // Foot schedule and self-weight pair. The next lifecycle decides the next command.
            bool loaded = _stanceLowerNativeGravity.CurrentApplied;
            if (knee.CurrentFixed != _fixedUpdateSequence || knee.CurrentSourceFixed != _fixedUpdateSequence ||
                _footFoundation.Report.FootControllerLastCompletedFixedSequence != _fixedUpdateSequence ||
                knee.CurrentEligible != loaded || knee.CurrentApplied != loaded || knee.CurrentNormalConstraint != loaded ||
                _kneeCharacterJointResponseIsolation.CurrentSupportNormalConstraint != loaded) return false;
            if (!loaded) return knee.CurrentUpperForceWorld == UnityEngine.Vector3.zero;
            UnityEngine.Vector3 n = knee.CurrentNormalWorld, acceleration = _kneeCharacterJointResponseIsolation.CurrentAppliedAccelerationWorld;
            return IsFinite(n) && IsFinite(acceleration) && IsFinite(knee.CurrentLowerMass) && knee.CurrentLowerMass > 0f &&
                IsFinite(knee.CurrentUpperMass) && knee.CurrentUpperMass > 0f &&
                UnityEngine.Mathf.Abs(n.sqrMagnitude - 1f) <= 0.0001f &&
                (n - _footFoundation.CurrentAuthorizedSupportNormalWorld).magnitude <= 0.0001f &&
                UnityEngine.Mathf.Abs(knee.CurrentLowerMass - _report.StanceLowerNativeGravityLowerMass) <= 0.0001f &&
                (knee.CurrentUpperForceWorld + knee.CurrentLowerMass * UnityEngine.Vector3.Dot(acceleration, n) * n).magnitude <= 0.0001f;
        }

        private bool DynamicLowerGravityMatches(bool lowerWeight)
        {
            string required = lowerWeight ? LowerStanceNativeGravitySupportLoadIsolation.ArmB : LowerStanceNativeGravitySupportLoadIsolation.ArmA;
            if (_stanceLowerNativeGravity.Arm != required || !_stanceLowerNativeGravity.Finite ||
                _lowerBody.useGravity != _stanceLowerNativeGravity.CurrentApplied) return false;
            if (lowerWeight)
            {
                // This experiment transfers the full self-weight only on the existing horizontal Box.
                // Do not silently discard tangential gravity or claim a slope/general gravity contract.
                UnityEngine.Vector3 gravity = UnityEngine.Physics.gravity;
                if (!IsFinite(gravity) || gravity.y >= 0f || gravity.x * gravity.x + gravity.z * gravity.z > 0.000001f) return false;
                if (_stanceLowerNativeGravity.CurrentApplied != _footFoundation.Report.StanceFootNativeGravityCurrentApplied) return false;
            }
            if (!_lowerBody.useGravity) return true;
            // A callback may have ended the contact AFTER the owned solve. Let the controller
            // clear that previous gravity schedule before evaluating the next lifecycle.
            return lowerWeight &&
                _footFoundation.Report.StanceFootNativeGravityCurrentApplied &&
                _report.StanceLowerNativeGravityCurrentScheduledFixed > 0 &&
                _report.StanceLowerNativeGravityCurrentScheduledFixed == _fixedUpdateSequence &&
                _footFoundation.Report.FootControllerLastCompletedFixedSequence == _fixedUpdateSequence;
        }

        private bool DynamicLowerWeightPairMatches() =>
            _report.StanceLowerSelfWeightTransferCurrentApplied == _stanceLowerNativeGravity.CurrentApplied &&
            (!_report.StanceLowerSelfWeightTransferCurrentApplied ||
             _report.StanceLowerSelfWeightTransferCurrentFixed == _report.StanceLowerNativeGravityCurrentScheduledFixed);

        internal void SelectDynamicContactScope(PelvisRunScope scope)
        {
            bool enabled = PelvisRunScopes.IsContact(scope);
            if ((!enabled && scope != PelvisRunScope.DynamicLegFreeSpace) || _traceEnabled ||
                (enabled ? GetDynamicContactIssue(scope) != DynamicContactBoundaryIssue.None : _driveEnabled))
                throw new InvalidOperationException("DYNAMIC_CONTACT_SCOPE_TRANSITION_NOT_READY");
            _dynamicScope = scope;
        }

        private bool DynamicEnvironmentNativeReady => DynamicContactSelected
            ? DynamicContactIssue == DynamicContactBoundaryIssue.None : FreeSpaceNativeReady;

        internal bool RestoreDynamicContactSelectors(StringBuilder evidence)
        {
            bool ok = RestoreContactSelector("HIP_TRANSLATION_PAIR_OFF", () => _hipAnchorPair.Quiesce(new PhysicsStepContext()) && _hipAnchorPair.Off, evidence);
            ok &= RestoreContactSelector("UPPER_WEIGHT_OFF", () => SetUpperSelfWeightTransfer(UpperSelfWeightSupportNormalLoadTransferIsolation.IdleArm, out _) && UpperSelfWeightOff, evidence);
            ok &= RestoreContactSelector("KNEE_NORMAL_COUPLING_OFF", () => SetKneeSupportNormalReaction(false, out _) && KneeSupportReactionOff, evidence);
            ok &= RestoreContactSelector("LOWER_GRAVITY_OFF", () => SetStanceLowerNativeGravityArm(LowerStanceNativeGravitySupportLoadIsolation.ArmA, out _), evidence);
            ok &= RestoreContactSelector("LOWER_WEIGHT_TRANSFER_OFF", () => SetStanceLowerSelfWeightTransferArm(LowerSelfWeightSupportNormalLoadTransferIsolation.ArmA, out _), evidence);
            ok &= RestoreContactSelector("FOOT_GRAVITY_OFF", () => SetStanceFootNativeGravityArm(FootStanceNativeGravitySupportLoadIsolation.ArmA, out _), evidence);
            ok &= RestoreContactSelector("STANCE_OFF", () => { DisableStanceContactLifecycle(); return !StanceContactLifecycleEnabled; }, evidence);
            ok &= RestoreContactSelector("SOLVER_BASELINE", () => RestoreFootSolverIterationBaseline(out _), evidence);
            ok &= RestoreContactSelector("OFFSET_BASELINE", () => SetFootContactOffsetArm(FootContactOffsetIsolation.IdleArm, out _), evidence);
            ok &= RestoreContactSelector("CCD_IDLE", () => SetFootCollisionDetectionModeArm(FootCollisionDetectionModeIsolation.IdleArm, out _), evidence);
            ok &= RestoreContactSelector("FRICTION_BASELINE", () => SetStanceContactFrictionIsolationArm(FootStanceContactFrictionIsolation.ArmA, out _), evidence);
            ok &= RestoreContactSelector("PREDICTIVE_A", () => SetTranslationPredictiveTailCaptureArm(FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA, out _), evidence);
            ok &= RestoreContactSelector("SCOPE_FREE_SPACE", () => { SelectDynamicContactScope(PelvisRunScope.DynamicLegFreeSpace); return !DynamicContactSelected; }, evidence);
            return ok && FreeSpaceSelectorsOff && NativeBodiesSafeOff;
        }
        private bool RestoreContactSelector(string name, Func<bool> operation, StringBuilder evidence)
        {
            try { bool ok = operation(); evidence.AppendLine(_report.Side + "/" + name + "=" + ok); return ok; }
            catch (Exception e) { evidence.AppendLine(_report.Side + "/" + name + "=False/exception=" + e.GetType().Name); return false; }
        }
    }
}
