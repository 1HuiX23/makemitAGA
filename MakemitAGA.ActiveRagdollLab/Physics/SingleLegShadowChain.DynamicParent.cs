using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        private bool _dynamicParentActive, _dynamicJoinPending;
        private PelvisParentFrame _physicalParent;
        private LegInitialState _initialState;
        private PhysicsStepContext _joinStep;
        private int _dynamicParentSteps;
        private float _joinHipGap, _joinKneeGap, _joinHipVelocityGap, _joinKneeVelocityGap;

        internal bool NativeBodiesSafeOff => IsBuilt && !_driveEnabled &&
            _upperBody.isKinematic && !_upperBody.useGravity && !_upperCollider.enabled &&
            _lowerBody.isKinematic && !_lowerBody.useGravity && !_lowerCollider.enabled &&
            _footFoundation != null && _footFoundation.FreeSpaceNativeState(false);
        internal bool DynamicParentActive => _dynamicParentActive && _driveEnabled;
        internal bool FootRelativeReferenceSelected => _footFoundation != null && _footFoundation.RelativeReferenceSelected;
        internal bool FootRelativeReferenceMatches(bool enabled) => _footFoundation != null &&
            _footFoundation.RelativeReferenceSelected == enabled &&
            _footFoundation.RelativeReferenceActive == (enabled && DynamicParentActive) &&
            (!enabled || _footFoundation.RelativeReferenceFinite);
        internal bool SelectFootRelativeReference(bool enabled) => !_driveEnabled && !_traceEnabled &&
            _footFoundation != null && _footFoundation.SelectRelativeReference(enabled);
        internal string FootRelativeReferenceStatus => _footFoundation?.RelativeReferenceStatus ?? "FOOT_REFERENCE_V04730=MISSING";
        internal FreeSpaceBoundaryIssue FreeSpaceSelectorIssue
        {
            get
            {
                if (!IsBuilt || _footFoundation == null) return FreeSpaceBoundaryIssue.MissingFoundation;
                FreeSpaceBoundaryIssue foot = _footFoundation.FreeSpaceSelectorIssue;
                if (foot != FreeSpaceBoundaryIssue.None) return foot;
                if (_stanceLowerNativeGravity.Arm != LowerStanceNativeGravitySupportLoadIsolation.ArmA || !_stanceLowerNativeGravity.Finite)
                    return FreeSpaceBoundaryIssue.LowerGravity;
                if (_stanceFootLowerReciprocal.Arm != FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmA || !_stanceFootLowerReciprocal.Finite)
                    return FreeSpaceBoundaryIssue.ReciprocalLoad;
                if (_stanceLowerSelfWeightTransfer.Arm != LowerSelfWeightSupportNormalLoadTransferIsolation.ArmA || !_stanceLowerSelfWeightTransfer.Finite)
                    return FreeSpaceBoundaryIssue.LowerSelfWeight;
                if (!UpperSelfWeightOff || !UpperSelfWeightFinite) return FreeSpaceBoundaryIssue.UpperSelfWeight;
                if (!KneeSupportReactionOff || !KneeSupportReactionFinite) return FreeSpaceBoundaryIssue.KneeReaction;
                if (!_hipAnchorPair.Off) return FreeSpaceBoundaryIssue.HipTranslationPair;
                return FreeSpaceBoundaryIssue.None;
            }
        }
        internal bool FreeSpaceSelectorsOff => FreeSpaceSelectorIssue == FreeSpaceBoundaryIssue.None;
        internal bool FreeSpaceNativeReady => IsBuilt && !_gravityEnabled && !_collidersEnabled &&
            !_upperBody.useGravity && !_lowerBody.useGravity && !_upperCollider.enabled && !_lowerCollider.enabled &&
            _upperBody.isKinematic == !_driveEnabled && _lowerBody.isKinematic == !_driveEnabled &&
            _footFoundation != null && _footFoundation.FreeSpaceNativeState(_driveEnabled);
        internal bool DynamicJoinFoundationReady => !DynamicContactSelected && IsBuilt && !_driveEnabled && !_traceEnabled &&
            FrozenProductionStackReady && FootProductionBaselineReady && !KneeAxisCalibrationCollecting &&
            NativeBodiesSafeOff && FreeSpaceSelectorsOff && _hipJoint == null && _kneeJoint == null &&
            _upperObject.GetComponent<CharacterJoint>() == null && _lowerObject.GetComponent<CharacterJoint>() == null &&
            !AnkleJointDestroyPending;

        internal LegInitialState PrepareDynamicJoin(PhysicsStepContext step, PelvisParentFrame parent)
        {
            if (!DynamicJoinFoundationReady || !parent.Matches(step) || !parent.Finite() ||
                parent.BodyId != _pelvisBody.GetInstanceID() || _pelvisBody.isKinematic ||
                !_runtimeSource.Valid || _runtimeSource.Epoch != step.Epoch || _runtimeSource.Sequence != step.SourceSequence)
                throw new InvalidOperationException("DYNAMIC_LEG_JOIN_CONTRACT");
            LegInitialState state = LegInitialState.Create(parent, _rawHipConnectedAnchorLocal,
                _sampledTargetHipRelative, _sampledTargetHipRelativeAngularVelocityLocal,
                _sampledTargetKneeRelative, _sampledTargetKneeRelativeAngularVelocityLocal,
                _kneeParentAnchorUpperLocal, _kneeChildAnchorLowerLocal, _upperBody.centerOfMass, _lowerBody.centerOfMass);
            if (!state.Finite) throw new InvalidOperationException("DYNAMIC_LEG_INITIAL_STATE_NONFINITE");
            return state;
        }

        internal InitialGeometryPreflight CaptureInitialGeometry(PhysicsStepContext step, Vector3 floorCenter, Vector3 floorHalf)
        {
            if (_footFoundation == null || !_runtimeSource.Valid || _runtimeSource.Epoch != step.Epoch ||
                _runtimeSource.Sequence != step.SourceSequence ||
                !_torso.TryGetHeldPelvisTargetPose(_runtimeSource, out Vector3 rawP, out Quaternion rawR)) return default;
            InitialGeometryPreflight result = _footFoundation.CaptureInitialGeometry(_runtimeSource, step.UnscaledTime, floorCenter, floorHalf);
            // Reconstruct the frozen Hip anchor in the held raw proxy frame, rather than
            // conflating its binding offset with current physical tracking response.
            Vector3 offset = rawP + rawR * _rawHipConnectedAnchorLocal - _sampledUpperPosition;
            result.HipBindingGap = offset.magnitude; result.HipBindingY = offset.y;
            return result;
        }

        internal HipJoinPose ProjectDynamicJoin(PhysicsStepContext step, PelvisParentFrame parent,
            Vector3 floorCenter, Vector3 floorHalf)
        {
            LegInitialState state = PrepareDynamicJoin(step, parent);
            HipJoinPose p = new HipJoinPose { Hip = state.UpperPosition, Knee = state.LowerPosition,
                Upper = state.UpperRotation, Lower = state.LowerRotation, RawHip = _sampledUpperPosition,
                RawUpper = _sampledUpperRotation, RawLower = _sampledLowerRotation };
            p.Geometry = _footFoundation.ProjectInitialGeometry(_runtimeSource, step.UnscaledTime, floorCenter, floorHalf,
                state.LowerPosition, state.LowerRotation, out p.Ankle, out p.Foot, out p.RawFoot, out p.RawFootRotation);
            if (!_torso.TryGetHeldPelvisTargetPose(_runtimeSource, out Vector3 rawP, out Quaternion rawR))
                p.Geometry.Valid = false;
            else
            {
                Vector3 offset = rawP + rawR * _rawHipConnectedAnchorLocal - _sampledUpperPosition;
                p.Geometry.HipBindingGap = offset.magnitude; p.Geometry.HipBindingY = offset.y;
            }
            return p;
        }

        internal HipContactPreparationSample CaptureContactPreparation(PhysicsStepContext step, int supportId,
            Vector3 floorCenter, Vector3 floorHalf) => _footFoundation == null ? default :
            _footFoundation.CaptureContactPreparation(step, _runtimeSource, supportId, floorCenter, floorHalf,
                CaptureInitialGeometry(step, floorCenter, floorHalf));

        internal void ActivateDynamicJoin(PhysicsStepContext step, PelvisParentFrame parent, LegInitialState state)
        {
            _physicalParent = parent; _initialState = state; _joinStep = step;
            _dynamicContactPreparationStep = default;
            _dynamicParentSteps = 0; _dynamicJoinPending = true;
            try
            {
                SetDrive(true);
                if (!DynamicParentActive || !FreeSpaceNativeReady)
                    throw new InvalidOperationException("DYNAMIC_LEG_ACTIVATION_READBACK_FAILED");
                Vector3 hip = _upperBody.position;
                Vector3 kneeParent = _upperBody.position + _upperBody.rotation * _kneeParentAnchorUpperLocal;
                Vector3 kneeChild = _lowerBody.position + _lowerBody.rotation * _kneeChildAnchorLowerLocal;
                _joinHipGap = (hip - parent.SettledPoint(_rawHipConnectedAnchorLocal)).magnitude;
                _joinKneeGap = (kneeParent - kneeChild).magnitude;
                _joinHipVelocityGap = (PointVelocity(_upperBody, hip) - parent.SettledPointVelocity(_rawHipConnectedAnchorLocal)).magnitude;
                _joinKneeVelocityGap = (PointVelocity(_upperBody, kneeParent) - PointVelocity(_lowerBody, kneeChild)).magnitude;
                if (!PelvisParentFrame.F(_joinHipGap + _joinKneeGap + _joinHipVelocityGap + _joinKneeVelocityGap) ||
                    _joinHipGap > 0.0001f || _joinKneeGap > 0.0001f || _joinHipVelocityGap > 0.001f || _joinKneeVelocityGap > 0.001f)
                    throw new InvalidOperationException("DYNAMIC_LEG_INITIAL_CONTINUITY_FAILED");
            }
            catch { ForceDriveOffSafe(); throw; }
            finally { _dynamicJoinPending = false; }
        }

        private static Vector3 PointVelocity(Rigidbody body, Vector3 point) =>
            body.velocity + Vector3.Cross(body.angularVelocity, point - body.worldCenterOfMass);

        // Source derivatives stay filtered/held in parent-local space. Re-express them in
        // the same parent basis as feedback only for the dynamic join; the loaded baseline
        // retains its original source-world mapping. This is not an added transport term.
        private Quaternion GetHipDerivativeParentRotation() => !_dynamicParentActive
            ? _torso.SampledPelvisTargetRotation
            : (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization
                ? _hipControlFrame.Rotation : _physicalParent.Rotation);

        private Quaternion GetKneeDerivativeParentRotation() => _dynamicParentActive
            ? _upperBody.rotation : _sampledUpperRotation;

        internal string DynamicParentSummary() => "side=" + _report.Side + "/dynamicParent=" + DynamicParentActive +
            "/parentBody=" + _physicalParent.BodyId + "/join=" + _joinStep.Status() + "/steps=" + _dynamicParentSteps +
            "/initialGapHipKnee=" + F(_joinHipGap) + ":" + F(_joinKneeGap) +
            "/initialVelocityGapHipKnee=" + F(_joinHipVelocityGap) + ":" + F(_joinKneeVelocityGap) +
            "/hipTranslation=" + (PelvisRunScopes.HasHipCoupling(_dynamicScope) ? "POINT_MOBILITY_PELVIS_CHEST_CONSTRAINED_COMMON_ACCEL_FF" : "SETTLED_NATIVE_POINT_PD") +
            "/kinematicPredictionFF=OFF/HipReaction=" + (PelvisRunScopes.HasHipCoupling(_dynamicScope) ? "TRANSLATION_AT_HIP_ANCHORS" : "OFF") +
            "/relativeAngularFrame=" + (_dynamicParentActive ? "FEEDBACK_PARENT_BASIS" : "LEGACY_SOURCE_BASIS");
    }
}
