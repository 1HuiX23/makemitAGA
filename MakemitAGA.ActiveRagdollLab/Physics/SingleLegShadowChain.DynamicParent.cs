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
        internal bool FreeSpaceSelectorsOff => !StanceContactLifecycleEnabled &&
            FootContactShadowArm == FootQualifiedContactShadow.ArmA && !StanceLowerGroundAuthorityEnabled &&
            !StanceFootNativeGravityEnabled && !StanceLowerNativeGravityEnabled &&
            !StanceFootLowerReciprocalEnabled && !StanceLowerSelfWeightTransferEnabled && UpperSelfWeightOff &&
            KneeSupportReactionOff && !StanceSupportAngularEnabled && !FootPositionSolverEnabled && !StancePlanarDampingEnabled;
        internal bool FreeSpaceNativeReady => IsBuilt && !_gravityEnabled && !_collidersEnabled &&
            !_upperBody.useGravity && !_lowerBody.useGravity && !_upperCollider.enabled && !_lowerCollider.enabled &&
            _upperBody.isKinematic == !_driveEnabled && _lowerBody.isKinematic == !_driveEnabled &&
            _footFoundation != null && _footFoundation.FreeSpaceNativeState(_driveEnabled);
        internal bool DynamicJoinFoundationReady => IsBuilt && !_driveEnabled && !_traceEnabled &&
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

        internal void ActivateDynamicJoin(PhysicsStepContext step, PelvisParentFrame parent, LegInitialState state)
        {
            _physicalParent = parent; _initialState = state; _joinStep = step;
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
            "/hipTranslation=SETTLED_NATIVE_POINT_PD/kinematicPredictionFF=OFF/HipReaction=OFF" +
            "/relativeAngularFrame=" + (_dynamicParentActive ? "FEEDBACK_PARENT_BASIS" : "LEGACY_SOURCE_BASIS");
    }
}
