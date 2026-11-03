using System;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Owned native resources and the build / activate / safe / rollback transitions.
    // Fixed control and held Render sampling remain in the main partial.
    internal sealed partial class FootAnkleProxyFoundation
    {
        // A freshly created preview is not root-owned until SetParent succeeds. Keep
        // its cleanup receipt across any intermediate native setter failure.
        private readonly List<GameObject> _unparentedObjects = new List<GameObject>();

        public string Build()
        {
            if (_destroyed)
                return "ARL foot/ankle build 失败：FOUNDATION_ROLLBACK_STARTED; create a new foundation after cleanup.";
            if (IsBuilt)
                return _report.GetConsoleSummary();
            if (!_profile.FootAnkleFoundationEnabled)
                return "ARL foot/ankle foundation | enabled=False | action=SKIPPED_BY_PROFILE";
            if (_target.Foot == null || _target.Toe == null || !_target.FootFoundationReady)
                return "ARL foot/ankle build 失败：Toe/foot geometry 未解析完成。 | toeResolve=" + (_target.ToeResolution ?? "<none>");

            try
            {
                _report.Stage = "CREATE_FOOT_OBJECT";
                _footObject = new GameObject("ARL_Foot_" + _target.RequestedSide);
                _footObject.hideFlags = HideFlags.DontSave;
                _footObject.layer = _layer;
                _footObject.transform.SetParent(_ownerRoot, true);

                Vector3 ankleWorld = _target.Foot.position;
                Vector3 toeWorld = _target.Toe.position;
                Vector3 ankleToToe = toeWorld - ankleWorld;
                float boneFootLength = ankleToToe.magnitude;
                if (!IsFinite(ankleToToe) || !IsFinite(boneFootLength) || boneFootLength < 0.03f)
                    throw new InvalidOperationException("FOOT_INVALID_ANKLE_TO_TOE_GEOMETRY");

                Vector3 referenceUp = _targetRoot != null ? _targetRoot.up : Vector3.up;
                if (!IsFinite(referenceUp) || referenceUp.sqrMagnitude < 0.0001f)
                    referenceUp = Vector3.up;
                referenceUp.Normalize();

                Vector3 soleForward = Vector3.ProjectOnPlane(ankleToToe, referenceUp);
                float projectedAnkleToeLength = soleForward.magnitude;
                if (!IsFinite(soleForward) || soleForward.sqrMagnitude < 0.0001f)
                    soleForward = Vector3.ProjectOnPlane(_target.Foot.forward, referenceUp);
                if (!IsFinite(soleForward) || soleForward.sqrMagnitude < 0.0001f)
                    soleForward = ankleToToe;
                soleForward.Normalize();

                Vector3 soleRight = Vector3.Cross(referenceUp, soleForward);
                if (!IsFinite(soleRight) || soleRight.sqrMagnitude < 0.0001f)
                    soleRight = _target.Foot.right;
                soleRight.Normalize();
                Vector3 soleUp = Vector3.Cross(soleForward, soleRight);
                if (!IsFinite(soleUp) || soleUp.sqrMagnitude < 0.0001f)
                    soleUp = referenceUp;
                soleUp.Normalize();
                if (Vector3.Dot(soleUp, referenceUp) < 0f)
                {
                    soleRight = -soleRight;
                    soleUp = -soleUp;
                }

                Quaternion soleRotation = Quaternion.LookRotation(soleForward, soleUp);
                if (!IsFinite(soleRotation))
                    throw new InvalidOperationException("FOOT_INVALID_SOLE_FRAME");
                _boneToSoleBindRotation = Quaternion.Inverse(_target.Foot.rotation) * soleRotation;
                if (!IsFinite(_boneToSoleBindRotation))
                    throw new InvalidOperationException("FOOT_INVALID_BONE_TO_SOLE_BIND");

                _footObject.transform.position = ankleWorld;
                _footObject.transform.rotation = soleRotation;

                // The hierarchy only gives ankle->toe centerline; it cannot provide real shoe width/heel/sole height.
                // v0.3.56.1 keeps the same conservative L/W/T estimate, but replaces the guessed vertical ankle->sole
                // offset with a one-shot support-plane calibration when five filtered probes agree on one flat
                // scene surface.  This is NOT footwear detection and never ground-snaps during runtime.
                // Creation starts with contact OFF; the exact-support stance owner authorizes later contact.
                float thickness = Mathf.Clamp(Mathf.Max(_profile.LegSegmentThickness * 0.55f, 0.05f), 0.05f, 0.08f);
                float forwardLength = Mathf.Clamp(Mathf.Max(boneFootLength * 1.10f, 0.14f), 0.14f, 0.26f);
                float heelLength = Mathf.Clamp(Mathf.Max(boneFootLength * 0.45f, 0.055f), 0.055f, 0.12f);
                float totalLength = forwardLength + heelLength;
                float width = Mathf.Clamp(Mathf.Max(boneFootLength * 0.58f, _profile.LegSegmentThickness * 0.95f), 0.085f, 0.14f);
                float estimatedAnkleHeight = Mathf.Clamp(Mathf.Max(boneFootLength * 0.40f, thickness * 1.05f), 0.055f, 0.10f);
                float ankleHeight = estimatedAnkleHeight;
                Vector3 size = new Vector3(width, thickness, totalLength);
                Vector3 center = new Vector3(0f, -ankleHeight + thickness * 0.5f, (forwardLength - heelLength) * 0.5f);
                if (!IsFinite(size) || !IsFinite(center) || size.x <= 0f || size.y <= 0f || size.z <= 0f)
                    throw new InvalidOperationException("FOOT_INVALID_SOLE_PROXY_GEOMETRY");

                // One-shot support-plane calibration.  The initial v0.3.56 run showed a bilateral ~48.4 mm
                // positive clearance while all five probes agreed on one horizontal room floor.  Instead of
                // hard-coding that shoe-specific number, infer the correction from the current support plane once
                // at build time.  After this point the calibrated offset is frozen; walking/lifting the foot can
                // never re-snap the proxy to the floor.
                _report.SupportPlaneCalibrationAttempted = true;
                _report.SupportPlaneCalibrationInitialAnkleHeightMeters = estimatedAnkleHeight;
                FootAnkleSoleRequalificationObserver.SupportPlaneCalibrationCandidate supportCandidate =
                    _soleRequalification.GetSupportPlaneCalibrationCandidate(
                        _footObject.transform,
                        center,
                        size,
                        _target.Foot,
                        _targetRoot,
                        _ownerRoot);
                _report.SupportPlaneCalibrationHitCount = supportCandidate.HitCount;
                _report.SupportPlaneCalibrationSpreadMeters = supportCandidate.SpreadMeters;
                _report.SupportPlaneCalibrationNormalDot = supportCandidate.NormalDotUp;
                _report.SupportPlaneCalibrationSurfacePath = supportCandidate.SurfacePath;
                _report.SupportPlaneCalibrationReason = supportCandidate.Reason;

                if (supportCandidate.Valid)
                {
                    float calibratedAnkleHeight = estimatedAnkleHeight + supportCandidate.CenterClearanceMeters;
                    // Keep a broad but explicit sanity envelope.  This is support-plane geometry, not a final
                    // anatomical/shoe dimension, so future footwear-aware calibration can replace it cleanly.
                    if (IsFinite(calibratedAnkleHeight) && calibratedAnkleHeight >= 0.045f && calibratedAnkleHeight <= 0.14f)
                    {
                        ankleHeight = calibratedAnkleHeight;
                        _report.SupportPlaneCalibrationApplied = true;
                        _report.SupportPlaneCalibrationCorrectionMeters = supportCandidate.CenterClearanceMeters;
                        _report.SupportPlaneCalibrationReason = "PASS_APPLIED";
                        center = new Vector3(0f, -ankleHeight + thickness * 0.5f, (forwardLength - heelLength) * 0.5f);
                    }
                    else
                    {
                        _report.SupportPlaneCalibrationReason = "FINAL_ANKLE_HEIGHT_OUT_OF_RANGE";
                    }
                }
                _report.SupportPlaneCalibrationFinalAnkleHeightMeters = ankleHeight;
                _report.FootwearStateResolved = false;
                _report.FootwearGeometryPolicy = "SUPPORT_PLANE_CALIBRATED_CURRENT_POSE;FOOTWEAR_STATE_UNRESOLVED";

                _report.Stage = "ADD_FOOT_BOX_COLLIDER";
                _collider = Il2CppComponentFactory.Add<BoxCollider>(_footObject, "FOOT_ADD_BOX_COLLIDER");
                _collider.center = center;
                _collider.size = size;
                _collider.enabled = false;
                if (!_footContactOffset.Initialize(_collider, _report, out string offsetInitError))
                    throw new InvalidOperationException("FOOT_CONTACT_OFFSET_INIT_FAILED:" + offsetInitError);
                if (!_stanceContactFrictionIsolation.SetArm(FootStanceContactFrictionIsolation.ArmA, _collider, _report, out string frictionInitError))
                    throw new InvalidOperationException("FOOT_FRICTION_ISOLATION_INIT_FAILED:" + (frictionInitError ?? "<none>"));

                _report.Stage = "ADD_FOOT_CONTACT_SHADOW_TRACKER";
                FootContactShadowTracker contactTracker = Il2CppComponentFactory.Add<FootContactShadowTracker>(_footObject, "FOOT_ADD_CONTACT_SHADOW_TRACKER");
                contactTracker.Initialize(_target.RequestedSide);
                _contactShadow.SetArmA(_collider, _report);

                _report.Stage = "ADD_FOOT_RIGIDBODY";
                _body = Il2CppComponentFactory.Add<Rigidbody>(_footObject, "FOOT_ADD_RIGIDBODY");
                if (!_footVelocitySolver.Initialize(_body, _report, out string solverInitError))
                    throw new InvalidOperationException("FOOT_VELOCITY_SOLVER_INIT_FAILED:" + solverInitError);
                _body.mass = _profile.FootMass;
                _body.isKinematic = true;
                _body.useGravity = false;
                _body.drag = 0.25f;
                _body.angularDrag = _profile.AngularDrag;
                _body.maxAngularVelocity = _profile.MaxAngularVelocity;
                _body.interpolation = RigidbodyInterpolation.None;
                _body.collisionDetectionMode = CollisionDetectionMode.Discrete;
                if (!_footCollisionDetectionMode.SetArm(FootCollisionDetectionModeIsolation.IdleArm, _body, _report, out string footCollisionInitError))
                    throw new InvalidOperationException("FOOT_COLLISION_MODE_INIT_FAILED:" + (footCollisionInitError ?? "<none>"));
                if (!_stanceLowerGroundAuthority.SetArm(FootStanceLowerGroundAnchorAuthority.ArmA, _report, out string lowerGroundInitError))
                    throw new InvalidOperationException("FOOT_LOWER_GROUND_AUTHORITY_INIT_FAILED:" + (lowerGroundInitError ?? "<none>"));

                _lowerAnkleAnchorLocal = _lowerBody.transform.InverseTransformPoint(_target.Foot.position);
                if (!IsFinite(_lowerAnkleAnchorLocal))
                    throw new InvalidOperationException("FOOT_INVALID_LOWER_ANKLE_ANCHOR");

                _report.Stage = "APPLY_FOOT_EXPLICIT_BOX_MASS_PROPERTIES";
                if (!ApplyAndValidateMassProperties())
                    throw new InvalidOperationException("FOOT_EXPLICIT_BOX_MASS_PROPERTY_VALIDATION_FAILED");

                _report.Stage = "CREATE_FOOT_VISUAL";
                CreateVisual(center, size);
                CreateReferenceMarkers(center, size);

                _lowerAnkleAnchorLocal = _lowerBody.transform.InverseTransformPoint(_target.Foot.position);
                _bindSourceRelative = Quaternion.Inverse(_target.LowerLeg.rotation) * soleRotation;
                if (!IsFinite(_bindSourceRelative))
                    throw new InvalidOperationException("FOOT_BIND_RELATIVE_ROTATION_INVALID");

                // v0.3.70 retains the v0.3.69 production freeze: these historical A/B selectors are no longer runtime experiment knobs.
                // Build locks the already-qualified B configuration while the Foot remains kinematic; SetDriveState(true)
                // is the only transition that makes the Foot dynamic.
                _ankleDrive.FreezeProductionArm();
                _ankleConstraint.FreezeProductionArm();
                _ankleTransport.FreezeProductionArm();
                _ankleAngularTransport.RefreshFootAngularDragReadback(_body.angularDrag, _report);

                _report.Built = true;
                _report.Side = _target.RequestedSide;
                _report.AnklePath = _target.FootPath;
                _report.ToePath = _target.ToePath;
                _report.ToeResolution = _target.ToeResolution;
                _report.FootLength = _target.FootLength;
                _report.GeometryMode = "SOLE_ALIGNED_SUPPORT_PLANE_CALIBRATED_V03561_REUSED_BY_V0378";
                _report.GeometryMeshCalibrated = false;
                _report.ContactGeometryAuthorized = false;
                _report.ProxyForwardLength = forwardLength;
                _report.ProxyHeelLength = heelLength;
                _report.ProxyTotalLength = totalLength;
                _report.ProxyWidth = width;
                _report.ProxyThickness = thickness;
                _report.ProxyAnkleHeight = ankleHeight;
                _report.ProjectedAnkleToeLength = projectedAnkleToeLength;
                _report.BoneToSoleCorrectionDegrees = Quaternion.Angle(_target.Foot.rotation, soleRotation);
                _report.BoneToSoleBindRotation = _boneToSoleBindRotation;
                _report.Mass = _body.mass;
                _report.ColliderCenterLocal = _collider.center;
                _report.ColliderSizeLocal = _collider.size;
                _report.BindAnkleRelativeRotation = _bindSourceRelative;
                _report.ColliderFrozenOff = true;
                _report.VisualEnabled = true;
                ClearAnkleJointReadback();
                _soleRequalification.CopyToReport(_report);
                _ankleDrive.CopyToReport(_report);
                _ankleConstraint.CopyToReport(_report);
                _ankleTransport.CopyToReport(_report);
                _report.Stage = "BUILD_COMPLETE";
                _report.Detail = "V0378_FREE_SPACE_PRODUCTION;FOOT_DRIVE_SOURCE_RELATIVE_ANKLE_PD_FROZEN;CONSTRAINT_NO_CHARACTERJOINT+FOOT_COM_TRANSLATION_TETHER_FROZEN;LINEAR_TRANSPORT_MEASURED_LOWER_ANKLE_POINT_ACCELERATION_FF_FROZEN;ANGULAR_TRANSPORT_MEASURED_LOWER_A_PLUS_BOUNDED_COMMON_MODE_DRAG_COMPENSATION_PRODUCTION;FOOT_ANGULAR_DRAG_PROFILE_FROZEN;TARGET_A_FF_V0368_PERSISTENCE_CONDITIONED_FROZEN;V0373_CONDITIONAL_CAPTURE_AUTHORITY_PRODUCTION;V0377_CONFIRMATION_OBSERVER_RETIRED;HISTORICAL_FOOT_OBSERVERS_RETIRED;ONE_SHOT_SUPPORT_PLANE_HEIGHT_CALIBRATION;FOOTWEAR_STATE_UNRESOLVED;CONTACT_SCOPE_SELECTED_BY_SESSION";

                SampleSourcePose(new SourceStepContext(), 0, Time.unscaledTime, false);
                FollowSourceKinematic();
                UpdateStaticState();
                return _report.GetConsoleSummary();
            }
            catch (Exception e)
            {
                _report.Stage = "BUILD_FAILED";
                _report.Detail = e.GetType().Name + "/" + e.Message;
                try { Plugin.Logger?.LogError("[ARL][FOOT_ANKLE][BUILD_FAILED] side=" + (_target.RequestedSide ?? "<none>") + " | " + e); }
                catch { } // Logging must not prevent owned native resources from being released.
                int failures = RollbackInternal().FailureCount.Value;
                return "ARL foot/ankle build 失败：" + e.GetType().Name + " / " + e.Message +
                       (failures > 0 ? " | cleanupFailures=" + failures : string.Empty);
            }
        }

        public void SetVisual(bool enabled)
        {
            SetRendererEnabled(_visual, enabled);
            SetRendererEnabled(_toeMarker, enabled);
            SetRendererEnabled(_ankleMarker, enabled);
            _report.VisualEnabled = enabled;
        }

        public int ForceSafeState()
        {
            _groundReference.DisablePreserveTrace();
            _loadAdmission.DisablePreserveTrace();
            int failures = 0;
            // Stop-time failures must not skip independent native shutdown attempts. Keep
            // the original owner order; diagnostic formatting never owns collision/gravity.
            TrySafeStateStep("SEAL_EVIDENCE", () =>
            {
                _turnEvents.Seal();
                _ankleConstraint.QuiesceStancePlanarDamping(_report);
                _rockingTetherCoupling.ClearPendingPreserveTrace(_report);
                _supportEntityBoundary.ClearPendingPreserveTrace(_report);
                _rockingTetherCoupling.CopyToReport(_report, false);
                _supportEntityBoundary.CopyToReport(_report, false);
                _ankleConstraint.GetStancePlanarDampingSummary(_report);
            }, ref failures);
            TrySafeStateStep("VELOCITY_SOLVER", () =>
            {
                // A successful owned-root destruction releases the body as well. A later
                // cleanup retry may still have external pair/material receipts to restore.
                if (_body != null && !_footVelocitySolver.QuiescePreserveTrace(_body, _report, out string error))
                    throw new InvalidOperationException(error ?? "SOLVER_QUIESCE_FAILED");
            }, ref failures);
            TrySafeStateStep("NATIVE_GRAVITY_OWNER", () =>
            {
                if (!_stanceFootNativeGravity.QuiescePreserveTrace(_body, _report, out string error))
                    throw new InvalidOperationException(error ?? "NATIVE_GRAVITY_QUIESCE_FAILED");
            }, ref failures);
            TrySafeStateStep("CONTROL_STATE", () =>
            {
                _stanceSupportAngular.ClearCurrentState(_report);
                _relativeReference.DisablePreserveTrace();
                _ankleDrive.SetDriveActive(false);
                _ankleConstraint.ClearCurrentState(_report);
                _ankleTransport.ClearCurrentState(_report);
                _ankleAngularTransport.ClearCurrentState(_report);
            }, ref failures);
            TrySafeStateStep("CONTACT_LIFECYCLE", () =>
                _stanceContactLifecycle.DisableAndReset(_contactShadow, _collider, _report), ref failures);
            int shadowFailures = 0;
            TrySafeStateStep("CONTACT_SHADOW", () => shadowFailures = _contactShadow.DisablePreserveTrace(_report), ref failures);
            failures += shadowFailures;
            TrySafeStateStep("COLLIDER_OFF", () => { if (_collider != null) _collider.enabled = false; }, ref failures);
            TrySafeStateStep("GRAVITY_OFF", () => { if (_body != null) _body.useGravity = false; }, ref failures);
            TrySafeStateStep("VELOCITY_ZERO", () => { if (_body != null) _body.velocity = Vector3.zero; }, ref failures);
            TrySafeStateStep("ANGULAR_VELOCITY_ZERO", () => { if (_body != null) _body.angularVelocity = Vector3.zero; }, ref failures);
            TrySafeStateStep("KINEMATIC_ON", () => { if (_body != null) _body.isKinematic = true; }, ref failures);
            TrySafeStateStep("FINAL_READBACK", UpdateStaticState, ref failures);
            failures += RefreshAnkleDriveReport();
            return failures;
        }

        private int RefreshAnkleDriveReport()
        {
            // Cold-path projection only: no native read, control reset, or trace mutation.
            // A diagnostic failure must not interrupt shutdown or change a completed OFF result.
            int failures = 0;
            TrySafeStateStep("ANKLE_DRIVE_REPORT", () => _ankleDrive.CopyToReport(_report), ref failures);
            return failures;
        }

        private void TrySafeStateStep(string stage, Action action, ref int failures)
        {
            try { action(); }
            catch (Exception e)
            {
                failures++;
                try
                {
                    Plugin.Logger?.LogWarning("[ARL][FOOT_SAFE_STATE_ERROR] side=" + (_report.Side ?? "<none>") +
                        " | " + stage + " | " + e.GetType().Name + " / " + e.Message);
                }
                catch { } // A failed diagnostic sink must not interrupt the remaining shutdown steps.
            }
        }

        public bool SetDriveState(bool enabled, bool physicalParent = false)
        {
            if (!enabled) { _groundReference.DisablePreserveTrace(); _loadAdmission.DisablePreserveTrace(); }
            if (!IsBuilt) return false;
            try
            {
                if (!enabled)
                {
                    _relativeReference.DisablePreserveTrace();
                    _ankleConstraint.QuiesceStancePlanarDamping(_report);
                    _rockingTetherCoupling.ClearPendingPreserveTrace(_report);
                    _supportEntityBoundary.ClearPendingPreserveTrace(_report);
                    _rockingTetherCoupling.CopyToReport(_report, false);
                    _supportEntityBoundary.CopyToReport(_report, false);
                    _ankleConstraint.GetStancePlanarDampingSummary(_report);
                    bool solverRestored = _footVelocitySolver.QuiescePreserveTrace(_body, _report, out string solverStopError);
                    if (!solverRestored)
                        Plugin.Logger?.LogWarning("[ARL][FOOT_DRIVE_STOP_SOLVER_RESTORE_FAILED] side=" + _report.Side + " | " + solverStopError);
                    _stanceSupportAngular.ClearCurrentState(_report);
                    _ankleDrive.SetDriveActive(false);
                    _ankleConstraint.ClearCurrentState(_report);
                    _ankleTransport.ClearCurrentState(_report);
                    _ankleAngularTransport.ClearCurrentState(_report);
                    FollowSourceKinematic();
                    RefreshAnkleDriveReport();
                    return solverRestored;
                }

                if (!_ankleDrive.SelectedDynamicArm)
                {
                    if (_relativeReference.Selected) return false;
                    _ankleDrive.SetDriveActive(true);
                    _ankleConstraint.ClearCurrentState(_report);
                    _ankleTransport.ClearCurrentState(_report);
                    _ankleAngularTransport.ClearCurrentState(_report);
                            FollowSourceKinematic();
                    return true;
                }

                if (!_report.SupportPlaneCalibrationApplied)
                {
                    Plugin.Logger?.LogError("[ARL][FOOT_ANKLE_DRIVE_ACTIVATION_REJECTED] side=" + (_report.Side ?? "<none>") + " | reason=SUPPORT_PLANE_CALIBRATION_NOT_APPLIED");
                    return false;
                }
                Vector3 activationSourceDeltaDegrees = QuaternionToAngularVector(Quaternion.Inverse(_bindSourceRelative) * _sampledSourceRelative) * Mathf.Rad2Deg;
                if (!IsFinite(activationSourceDeltaDegrees) ||
                    Mathf.Abs(activationSourceDeltaDegrees.x) > FootAnkleSourceDrivenRelativePdIsolation.FlexionLimitDegrees ||
                    Mathf.Abs(activationSourceDeltaDegrees.y) > FootAnkleSourceDrivenRelativePdIsolation.AxialLimitDegrees ||
                    Mathf.Abs(activationSourceDeltaDegrees.z) > FootAnkleSourceDrivenRelativePdIsolation.InversionLimitDegrees)
                {
                    Plugin.Logger?.LogError("[ARL][FOOT_ANKLE_DRIVE_ACTIVATION_REJECTED] side=" + (_report.Side ?? "<none>") +
                                            " | reason=SOURCE_POSE_OUTSIDE_REFERENCE_ENVELOPE" +
                                            " | sourceDeg(F-A-I)=" + V(activationSourceDeltaDegrees));
                    return false;
                }

                // All shipped scopes use the frozen no-Joint tether. The retired v0.3.62
                // creator is intentionally absent; never rebuild its native angular/anchor solver.
                if (_ankleConstraint.CharacterJointArm)
                    throw new InvalidOperationException("FOOT_CONSTRAINT_NOT_FROZEN_TO_TETHER");
                DestroyAnkleJoint();
                if (RefreshPendingAnkleJointDestroy())
                {
                    Plugin.Logger?.LogWarning("[ARL][FOOT_ANKLE_DRIVE_ACTIVATION_REJECTED] side=" + (_report.Side ?? "<none>") +
                                              " | reason=ANKLE_JOINT_DESTROY_PENDING; retry after one frame");
                    return false;
                }

                // Enter Dynamic Foot with zero positional ankle debt. Source orientation initializes the sole pose, but the
                // Foot origin is placed at the physical Lower ankle endpoint; otherwise the pre-existing source-vs-
                // physical distal gap would be converted into an artificial first-frame Joint impulse.
                Vector3 ankleWorld = physicalParent ? _lowerBody.position + _lowerBody.rotation * _lowerAnkleAnchorLocal :
                    _lowerBody.transform.TransformPoint(_lowerAnkleAnchorLocal);
                _body.position = ankleWorld;
                _body.rotation = physicalParent ? _lowerBody.rotation * _sampledSourceRelative : _sampledFootRotation;
                _body.useGravity = false;
                _contactShadow.DisablePreserveTrace(_report);
                if (_collider != null) _collider.enabled = false;
                Vector3 ankleVelocity = _lowerBody.velocity + Vector3.Cross(_lowerBody.angularVelocity, ankleWorld - _lowerBody.worldCenterOfMass);
                _body.isKinematic = false;
                _body.velocity = IsFinite(ankleVelocity) ? ankleVelocity : Vector3.zero;
                Vector3 sourceRelativeAngularVelocityWorld = _lowerBody.rotation * _sampledSourceRelativeAngularVelocityLocal;
                Vector3 initialFootAngularVelocity = _lowerBody.angularVelocity + sourceRelativeAngularVelocityWorld;
                _body.angularVelocity = IsFinite(initialFootAngularVelocity)
                    ? initialFootAngularVelocity
                    : (IsFinite(_lowerBody.angularVelocity) ? _lowerBody.angularVelocity : Vector3.zero);
                if (physicalParent)
                {
                    // The ankle point and the Foot COM have different velocities when rotating.
                    _body.velocity = ankleVelocity + Vector3.Cross(_body.angularVelocity, _body.rotation * _body.centerOfMass);
                    Vector3 readback = _body.velocity + Vector3.Cross(_body.angularVelocity, ankleWorld - _body.worldCenterOfMass);
                    if (!IsFinite(_body.velocity) || !IsFinite(_body.angularVelocity) ||
                        !IsFinite(readback) || (readback - ankleVelocity).magnitude > 0.001f ||
                        Quaternion.Angle(_body.rotation, _lowerBody.rotation * _sampledSourceRelative) > 0.01f)
                        throw new InvalidOperationException("FOOT_INITIAL_POINT_CONTINUITY_FAILED");
                }
                _ankleDrive.SetDriveActive(true);
                if (_relativeReference.Selected && (!physicalParent ||
                    !_relativeReference.Seed(_report.RuntimeSourceContext, _sampledSourceRelative, _sampledSourceRelativeAngularVelocityLocal)))
                    throw new InvalidOperationException("FOOT_REFERENCE_V04730_ACTIVATION_FAILED");
                UpdateStaticState();
                bool constraintReady = _ankleConstraint.CharacterJointArm
                    ? (_ankleJoint != null && _ankleJoint.connectedBody == _lowerBody)
                    : _ankleJoint == null;
                return constraintReady && !_body.isKinematic && !_body.useGravity && (_collider == null || !_collider.enabled);
            }
            catch (Exception e)
            {
                try { Plugin.Logger?.LogError("[ARL][FOOT_ANKLE_DRIVE_ACTIVATION_FAILED] side=" + (_report.Side ?? "<none>") + " | " + e); }
                catch { }
                ForceSafeState();
                return false;
            }
        }

        public int Rollback()
            => RollbackWithReceipt().FailureCount.Value;

        internal CleanupReceipt RollbackWithReceipt()
        {
            CleanupReceipt result = RollbackInternal();
            _report.Built = false;
            _report.Stage = result.Succeeded ? "ROLLED_BACK" : "ROLLBACK_WITH_ERRORS";
            return result;
        }

        private CleanupReceipt RollbackInternal()
        {
            // Retire control immediately, including failed builds. Retained native handles
            // remain cleanup-owned and must never make IsBuilt true again during a retry.
            _destroyed = true;
            int failures = ForceSafeState();
            int destroyAttempts = 0, destroyReturned = 0;
            try { failures += _stanceContactFrictionIsolation.RestoreAndDispose(_collider, _report); }
            catch { failures++; }
            for (int i = _unparentedObjects.Count - 1; i >= 0; i--)
            {
                try
                {
                    if (_unparentedObjects[i] != null)
                    {
                        destroyAttempts++;
                        Object.Destroy(_unparentedObjects[i]);
                        destroyReturned++;
                    }
                    _unparentedObjects.RemoveAt(i);
                }
                catch { failures++; } // Retain failed orphan destruction for the next rollback.
            }
            bool rootReleased = false;
            try
            {
                if (_footObject != null)
                {
                    destroyAttempts++;
                    Object.Destroy(_footObject);
                    destroyReturned++;
                }
                rootReleased = true;
            }
            catch (Exception e)
            {
                failures++;
                try { Plugin.Logger?.LogWarning("[ARL][FOOT_ROLLBACK_STEP_FAILED] side=" + (_report.Side ?? "<none>") + " | DESTROY_ROOT | " + e.GetType().Name + " / " + e.Message); }
                catch { }
            }
            if (rootReleased)
            {
                _footObject = null;
                _visual = null;
                _toeMarker = null;
                _ankleMarker = null;
                _body = null;
                _collider = null;
                _ankleJoint = null;
                _ankleJointDestroyPending = null;
            }
            _report.Built = false;
            // Managed ownership bookkeeping only: do not add a native fake-null/readback
            // query merely to populate a receipt after the original cleanup sequence.
            int retained = _unparentedObjects.Count + (ReferenceEquals(_footObject, null) ? 0 : 1);
            return new CleanupReceipt(true, failures == 0, failures, retained, null,
                destroyAttempts, destroyReturned, null);
        }

        private bool RefreshPendingAnkleJointDestroy()
        {
            if (_ankleJointDestroyPending == null)
            {
                _ankleJointDestroyPending = null;
                return false;
            }
            return true;
        }

        private void DestroyAnkleJoint()
        {
            if (RefreshPendingAnkleJointDestroy())
            {
                CopyAnkleJointReadback();
                return;
            }
            // The old creation path is retired. Still detect a surviving component on our
            // owned object at activation and retain the deferred-destruction interlock.
            if (_ankleJoint == null && _footObject != null)
                _ankleJoint = _footObject.GetComponent<CharacterJoint>();
            if (_ankleJoint == null)
            {
                CopyAnkleJointReadback();
                return;
            }
            // Object.Destroy is deferred to end-of-frame. Do not set connectedBody=null first: a live Joint with
            // connectedBody=null is a world-connected constraint in Unity, which can create a transient world anchor.
            // Arm switches are allowed only while the leg/Foot are kinematic, so keeping the old connection until the
            // deferred destruction completes is the safer zero-impulse transition.
            CharacterJoint oldJoint = _ankleJoint;
            Object.Destroy(oldJoint);
            _ankleJointDestroyPending = oldJoint;
            _ankleJoint = null;
            CopyAnkleJointReadback();
        }

        private void CopyAnkleJointReadback()
        {
            _report.AnkleJointExists = _ankleJoint != null;
            _report.AnkleConnectedBodyMatch = _ankleJoint != null && _ankleJoint.connectedBody == _lowerBody;
            if (_ankleJoint == null)
            {
                _report.AnkleJointAnchorLocal = Vector3.zero;
                _report.AnkleConnectedAnchorLocal = Vector3.zero;
                _report.AnkleAxisLocal = Vector3.zero;
                _report.AnkleSwingAxisLocal = Vector3.zero;
                _report.AnkleLowTwistLimit = 0f;
                _report.AnkleHighTwistLimit = 0f;
                _report.AnkleSwing1Limit = 0f;
                _report.AnkleSwing2Limit = 0f;
                return;
            }
            _report.AnkleJointAnchorLocal = _ankleJoint.anchor;
            _report.AnkleConnectedAnchorLocal = _ankleJoint.connectedAnchor;
            _report.AnkleAxisLocal = _ankleJoint.axis;
            _report.AnkleSwingAxisLocal = _ankleJoint.swingAxis;
            _report.AnkleLowTwistLimit = _ankleJoint.lowTwistLimit.limit;
            _report.AnkleHighTwistLimit = _ankleJoint.highTwistLimit.limit;
            _report.AnkleSwing1Limit = _ankleJoint.swing1Limit.limit;
            _report.AnkleSwing2Limit = _ankleJoint.swing2Limit.limit;
        }

        private void ClearAnkleJointReadback()
        {
            _report.AnkleJointExists = false;
            _report.AnkleConnectedBodyMatch = false;
            _report.AnkleJointAnchorLocal = Vector3.zero;
            _report.AnkleConnectedAnchorLocal = Vector3.zero;
            _report.AnkleAxisLocal = Vector3.zero;
            _report.AnkleSwingAxisLocal = Vector3.zero;
            _report.AnkleLowTwistLimit = 0f;
            _report.AnkleHighTwistLimit = 0f;
            _report.AnkleSwing1Limit = 0f;
            _report.AnkleSwing2Limit = 0f;
        }

        private bool ApplyAndValidateMassProperties()
        {
            Vector3 scale = _collider.transform.lossyScale;
            Vector3 size = new Vector3(
                Mathf.Abs(_collider.size.x * scale.x),
                Mathf.Abs(_collider.size.y * scale.y),
                Mathf.Abs(_collider.size.z * scale.z));
            if (!IsFinite(size) || size.x <= 0f || size.y <= 0f || size.z <= 0f || !IsFinite(_body.mass) || _body.mass <= 0f)
                return false;

            float k = _body.mass / 12f;
            Vector3 tensor = new Vector3(
                k * (size.y * size.y + size.z * size.z),
                k * (size.x * size.x + size.z * size.z),
                k * (size.x * size.x + size.y * size.y));
            if (!IsFinite(tensor) || tensor.x <= TensorEpsilon || tensor.y <= TensorEpsilon || tensor.z <= TensorEpsilon)
                return false;

            _body.centerOfMass = _collider.center;
            _body.inertiaTensorRotation = Quaternion.identity;
            _body.inertiaTensor = tensor;

            Vector3 runtimeCom = _body.centerOfMass;
            Vector3 runtimeTensor = _body.inertiaTensor;
            bool valid =
                ApproximatelyMassProperty(runtimeCom, _collider.center) &&
                ApproximatelyMassProperty(runtimeTensor, tensor) &&
                Quaternion.Angle(_body.inertiaTensorRotation, Quaternion.identity) <= 0.05f;
            _report.RuntimeCenterOfMassLocal = runtimeCom;
            _report.RuntimeInertiaTensor = runtimeTensor;
            _report.MassPropertiesValidated = valid;
            return valid;
        }

        private void CreateVisual(Vector3 center, Vector3 size)
        {
            _visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _unparentedObjects.Add(_visual);
            _visual.name = "ARL_Foot_" + _target.RequestedSide + "_Visual";
            _visual.hideFlags = HideFlags.DontSave;
            _visual.layer = _layer;
            _visual.transform.SetParent(_footObject.transform, false);
            _unparentedObjects.Remove(_visual);
            _visual.transform.localPosition = center;
            _visual.transform.localRotation = Quaternion.identity;
            // V04450: avoid coplanar display flashing against the floor; physics geometry is unchanged.
            _visual.transform.localScale = size * 0.98f;
            try
            {
                Collider primitiveCollider = _visual.GetComponent<Collider>();
                if (primitiveCollider == null)
                    throw new InvalidOperationException("FOOT_PREVIEW_COLLIDER_MISSING");
                primitiveCollider.enabled = false;
                Object.Destroy(primitiveCollider);
            }
            catch (Exception e)
            {
                throw new InvalidOperationException("FOOT_PREVIEW_COLLIDER_DISABLE_FAILED", e);
            }
            try
            {
                Renderer renderer = _visual.GetComponent<Renderer>();
                if (renderer != null && _previewMaterial != null)
                    renderer.sharedMaterial = _previewMaterial;
            }
            catch { }
        }

        private void CreateReferenceMarkers(Vector3 center, Vector3 size)
        {
            // Geometry-only visual witnesses.  The small forward cap makes sole +Z (toe direction) visible in-game;
            // the ankle cube marks the Rigidbody/ankle pivot.  Both have their primitive colliders removed.
            _toeMarker = CreateMarker(
                "ARL_Foot_" + _target.RequestedSide + "_ToeDirectionMarker",
                new Vector3(0f, center.y, center.z + size.z * 0.5f + 0.0125f),
                new Vector3(Mathf.Max(0.03f, size.x * 0.55f), Mathf.Max(0.012f, size.y * 0.45f), 0.025f));
            _ankleMarker = CreateMarker(
                "ARL_Foot_" + _target.RequestedSide + "_AnklePivotMarker",
                Vector3.zero,
                new Vector3(0.025f, 0.025f, 0.025f));
        }

        private GameObject CreateMarker(string name, Vector3 localPosition, Vector3 localScale)
        {
            GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _unparentedObjects.Add(marker);
            marker.name = name;
            marker.hideFlags = HideFlags.DontSave;
            marker.layer = _layer;
            marker.transform.SetParent(_footObject.transform, false);
            _unparentedObjects.Remove(marker);
            marker.transform.localPosition = localPosition;
            marker.transform.localRotation = Quaternion.identity;
            marker.transform.localScale = localScale;
            Collider markerCollider = marker.GetComponent<Collider>();
            if (markerCollider != null)
            {
                markerCollider.enabled = false;
                Object.Destroy(markerCollider);
            }
            try
            {
                Renderer renderer = marker.GetComponent<Renderer>();
                if (renderer != null && _previewMaterial != null)
                    renderer.sharedMaterial = _previewMaterial;
            }
            catch { }
            return marker;
        }

        private static void SetRendererEnabled(GameObject obj, bool enabled)
        {
            if (obj == null) return;
            try
            {
                Renderer renderer = obj.GetComponent<Renderer>();
                if (renderer != null) renderer.enabled = enabled;
            }
            catch { }
        }

        private static bool ApproximatelyMassProperty(Vector3 a, Vector3 b)
        {
            return ApproximatelyComponent(a.x, b.x) && ApproximatelyComponent(a.y, b.y) && ApproximatelyComponent(a.z, b.z);
        }

        private static bool ApproximatelyComponent(float a, float b)
        {
            float tolerance = Mathf.Max(
                MassReadbackAbsoluteTolerance,
                Mathf.Max(Mathf.Abs(a), Mathf.Abs(b)) * MassReadbackRelativeTolerance);
            return Mathf.Abs(a - b) <= tolerance;
        }
    }
}
