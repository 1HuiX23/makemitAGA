using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.3.78 frozen Foot/Ankle free-space production foundation.
    ///
    /// v0.3.68 promoted persistence-conditioned target-A after a complete causal A/B showed bilateral NORMAL_TRANSITION
    /// improvement without safety failure. v0.3.69 froze Dynamic Foot + no-CharacterJoint translation tether +
    /// v0.3.64 linear transport + v0.3.65 angular transport + v0.3.68 conditioned target-A as one production configuration.
    /// v0.3.70 then proved the rare tail is COM-velocity/capture-authority limited while parent transport is correctly closing.
    /// v0.3.71 qualified a narrowly eligible MaxA60->80 capture reserve and v0.3.72 confirmed it again while translation stayed below ~15 mm. v0.3.73 promotes that unchanged law into production and adds only a read-only axial hard-envelope temporal-debt observer; the 30 mm translation gate and 45/40/30/total60 angular gate remain unchanged.
    /// </summary>
    internal sealed class FootAnkleProxyFoundation
    {
        private const float TensorEpsilon = 0.000001f;
        private const float MassReadbackAbsoluteTolerance = 0.00005f;
        private const float MassReadbackRelativeTolerance = 0.0025f;

        private readonly ActiveRagdollLabProfile _profile;
        private readonly SingleLegTargetReport _target;
        private readonly Transform _ownerRoot;
        private readonly Transform _targetRoot;
        private readonly Rigidbody _lowerBody;
        private readonly int _layer;
        private readonly Material _previewMaterial;
        private readonly FootAnkleFoundationReport _report = new FootAnkleFoundationReport();
        private readonly FootAnkleSoleRequalificationObserver _soleRequalification = new FootAnkleSoleRequalificationObserver();
        private readonly FootGroundContactIdentityQualificationObserver _contactIdentity = new FootGroundContactIdentityQualificationObserver();
        private readonly FootQualifiedContactShadow _contactShadow = new FootQualifiedContactShadow();
        private readonly FootSupportNormalMechanicalReactionObserver _mechanicalReaction = new FootSupportNormalMechanicalReactionObserver();

        private readonly FootContactAngularImpulseAttributionObserver _footAngularImpulseAttribution = new FootContactAngularImpulseAttributionObserver();
        private readonly FootStanceSupportNormalAngularStabilization _stanceSupportAngular = new FootStanceSupportNormalAngularStabilization();
        private readonly FootContactRockingTetherCouplingObserver _rockingTetherCoupling = new FootContactRockingTetherCouplingObserver();
        private readonly FootSupportEntitySolveBoundaryObserver _supportEntityBoundary = new FootSupportEntitySolveBoundaryObserver();
        private readonly FootSolverIterationBaselineContract _footVelocitySolver = new FootSolverIterationBaselineContract();
        private readonly FootCollisionDetectionModeIsolation _footCollisionDetectionMode = new FootCollisionDetectionModeIsolation();
        private readonly FootContactOffsetIsolation _footContactOffset = new FootContactOffsetIsolation();

        private readonly FootGameNativeLocomotionPhaseObserver _gameNativeGaitPhase = new FootGameNativeLocomotionPhaseObserver();
        private readonly FootStanceOnlyAuthorizedContactLifecycle _stanceContactLifecycle = new FootStanceOnlyAuthorizedContactLifecycle();
        private readonly FootStanceContactFrictionIsolation _stanceContactFrictionIsolation = new FootStanceContactFrictionIsolation();

        private readonly FootStanceLowerGroundAnchorAuthority _stanceLowerGroundAuthority = new FootStanceLowerGroundAnchorAuthority();
        private readonly FootStanceNativeGravitySupportLoadIsolation _stanceFootNativeGravity = new FootStanceNativeGravitySupportLoadIsolation();
        private readonly FootAnkleSourceDrivenRelativePdIsolation _ankleDrive = new FootAnkleSourceDrivenRelativePdIsolation();
        private readonly FootAnkleConstraintTopologyIsolation _ankleConstraint = new FootAnkleConstraintTopologyIsolation();
        private readonly FootAnkleTranslationTransportIsolation _ankleTransport = new FootAnkleTranslationTransportIsolation();
        private readonly FootAnkleAngularTransport _ankleAngularTransport = new FootAnkleAngularTransport();
        private readonly FootAnkleTargetAccelerationPersistenceConditioner _targetAccelerationPersistenceConditioner = new FootAnkleTargetAccelerationPersistenceConditioner();
        private readonly FootTurnEventRecorder _turnEvents = new FootTurnEventRecorder();
        private readonly FootRelativeReference _relativeReference = new FootRelativeReference();

        internal bool RelativeReferenceSelected => _relativeReference.Selected;
        internal bool RelativeReferenceActive => _relativeReference.Active;
        internal bool RelativeReferenceFinite => _relativeReference.Finite;
        internal string RelativeReferenceStatus => _relativeReference.GetDetails();
        internal bool SelectRelativeReference(bool enabled)
        {
            if (!IsBuilt || _ankleDrive.DynamicActive || _traceEnabled) return false;
            _relativeReference.Select(enabled);
            return _relativeReference.Selected == enabled;
        }
        private Quaternion ControlRelativeRotation => _relativeReference.Active ? _relativeReference.Current.Rotation : _sampledSourceRelative;
        private Vector3 ControlRelativeW => _relativeReference.Active ? _relativeReference.Current.W : _sampledSourceRelativeAngularVelocityLocal;
        private Vector3 ControlRelativeA => _relativeReference.Active ? _relativeReference.Current.A : _sampledSourceRelativeAngularAccelerationLocal;
        private Vector3 ControlRelativeFF => _relativeReference.Active ? _relativeReference.Current.A : _sampledConditionedSourceRelativeAngularAccelerationLocal;

        private GameObject _footObject;
        private GameObject _visual;
        private GameObject _toeMarker;
        private GameObject _ankleMarker;
        private Rigidbody _body;
        private BoxCollider _collider;
        private CharacterJoint _ankleJoint;
        // Object.Destroy is end-of-frame. Retain the old wrapper separately so the tether arm cannot unlock a
        // Dynamic Foot while a native CharacterJoint may still survive for the remainder of the current frame.
        private CharacterJoint _ankleJointDestroyPending;
        private Vector3 _lowerAnkleAnchorLocal;
        private readonly Vector3 _footAnkleAnchorLocal = Vector3.zero;
        private Quaternion _boneToSoleBindRotation = Quaternion.identity;
        private Quaternion _bindSourceRelative = Quaternion.identity;
        private Quaternion _sampledSourceRelative = Quaternion.identity;
        private Vector3 _sampledFootPosition;
        private Quaternion _sampledFootRotation = Quaternion.identity;
        private Vector3 _sampledFootLinearVelocityWorld;
        private Vector3 _sampledSourceRelativeAngularVelocityLocal;
        private Vector3 _sampledSourceRelativeAngularAccelerationLocal;
        private Vector3 _sampledConditionedSourceRelativeAngularAccelerationLocal;
        private Vector3 _previousSourceRelativeAngularVelocityLocal;
        private bool _hasPreviousSourceRelativeAngularVelocitySample;
        private Vector3 _previousFootPosition;
        private float _previousFootPositionTime;
        private bool _hasPreviousFootPositionSample;
        private Quaternion _previousSourceRelative = Quaternion.identity;
        private float _previousSampleTime;
        private bool _hasPreviousSourceSample;
        private bool _traceEnabled;
        private float _traceErrorSum;
        private float _traceAnchorSum;
        private float _traceSourceAngularSpeedSum;
        private float _traceSourceFromBindAngleSum;
        private bool _destroyed;
        private FootSupportSurfaceIsolation _supportSurface;

        public FootAnkleProxyFoundation(
            ActiveRagdollLabProfile profile,
            SingleLegTargetReport target,
            Transform ownerRoot,
            Transform targetRoot,
            Rigidbody lowerBody,
            int layer,
            Material previewMaterial)
        {
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            _target = target ?? throw new ArgumentNullException(nameof(target));
            _ownerRoot = ownerRoot ?? throw new ArgumentNullException(nameof(ownerRoot));
            _targetRoot = targetRoot;
            _lowerBody = lowerBody ?? throw new ArgumentNullException(nameof(lowerBody));
            _layer = layer;
            _previewMaterial = previewMaterial;
        }

        public FootAnkleFoundationReport Report => _report;
        public bool IsBuilt => !_destroyed && _footObject != null && _body != null && _collider != null;

        public string Build()
        {
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
                // Collider/contact remains hard-OFF until a later mesh/GSP/contact calibration stage.
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
                _report.Detail = "V0378_FREE_SPACE_PRODUCTION;FOOT_DRIVE_SOURCE_RELATIVE_ANKLE_PD_FROZEN;CONSTRAINT_NO_CHARACTERJOINT+FOOT_COM_TRANSLATION_TETHER_FROZEN;LINEAR_TRANSPORT_MEASURED_LOWER_ANKLE_POINT_ACCELERATION_FF_FROZEN;ANGULAR_TRANSPORT_MEASURED_LOWER_A_PLUS_BOUNDED_COMMON_MODE_DRAG_COMPENSATION_PRODUCTION;FOOT_ANGULAR_DRAG_PROFILE_FROZEN;TARGET_A_FF_V0368_PERSISTENCE_CONDITIONED_FROZEN;V0373_CONDITIONAL_CAPTURE_AUTHORITY_PRODUCTION;V0377_CONFIRMATION_OBSERVER_RETIRED;HISTORICAL_FOOT_OBSERVERS_RETIRED;ONE_SHOT_SUPPORT_PLANE_HEIGHT_CALIBRATION;FOOTWEAR_STATE_UNRESOLVED;NO_CONTACT";

                SampleSourcePose(new SourceStepContext(), 0, Time.unscaledTime, false);
                FollowSourceKinematic();
                UpdateStaticState();
                return _report.GetConsoleSummary();
            }
            catch (Exception e)
            {
                _report.Stage = "BUILD_FAILED";
                _report.Detail = e.GetType().Name + "/" + e.Message;
                Plugin.Logger?.LogError("[ARL][FOOT_ANKLE][BUILD_FAILED] side=" + (_target.RequestedSide ?? "<none>") + " | " + e);
                int failures = RollbackInternal();
                return "ARL foot/ankle build 失败：" + e.GetType().Name + " / " + e.Message +
                       (failures > 0 ? " | cleanupFailures=" + failures : string.Empty);
            }
        }

        public void SampleSourcePose(SourceStepContext source, int renderSequence, float sampleTime, bool advance)
        {
            if (advance)
            {
                if (!source.Valid || source.Time != sampleTime) throw new InvalidOperationException("FOOT_SOURCE_CONTEXT_INVALID");
                _report.RuntimeSourceContext = source;
            }
            if (!IsBuilt || _target.LowerLeg == null || _target.Foot == null)
                return;

            Quaternion sampledSoleRotation = _target.Foot.rotation * _boneToSoleBindRotation;
            if (!IsFinite(sampledSoleRotation))
                return;
            Quaternion relative = Quaternion.Inverse(_target.LowerLeg.rotation) * sampledSoleRotation;
            Vector3 footPosition = _target.Foot.position;
            if (!advance)
            {
                // Preview/reset snapshots are not new Render packets. Preserve all W/A,
                // sample timestamps, persistence confirmation and diagnostic source identities.
                // An active controller must keep its entire held packet, including pose.
                if (_ankleDrive.DynamicActive) return;
                _sampledFootPosition = footPosition; _sampledFootRotation = sampledSoleRotation;
                _sampledSourceRelative = relative;
                _report.SourceAnkleRelativeRotation = relative;
                Vector3 previewDelta = QuaternionToAngularVector(relative * Quaternion.Inverse(_bindSourceRelative)) * Mathf.Rad2Deg;
                _report.SourceAnkleFromBindRotationVectorDegrees = previewDelta;
                _report.SourceAnkleFromBindAngleDegrees = previewDelta.magnitude;
                if (!_ankleDrive.DynamicActive) FollowSourceKinematic();
                return;
            }
            Vector3 footLinearVelocity = Vector3.zero;
            if (_hasPreviousFootPositionSample)
            {
                float footDt = sampleTime - _previousFootPositionTime;
                if (IsFinite(footDt) && footDt > 0.0001f && footDt < 0.25f)
                    footLinearVelocity = (footPosition - _previousFootPosition) / footDt;
            }
            _sampledFootPosition = footPosition;
            _sampledFootRotation = sampledSoleRotation;
            _sampledFootLinearVelocityWorld = IsFinite(footLinearVelocity) ? footLinearVelocity : Vector3.zero;
            _sampledSourceRelative = relative;
            if (advance) _turnEvents.ObserveSource(source, sampledSoleRotation);

            Vector3 renderStepRadians = Vector3.zero;
            Vector3 angularVelocity = Vector3.zero;
            Vector3 angularAcceleration = Vector3.zero;
            bool angularVelocityValid = false;
            float sourceDt = 0f;
            if (_hasPreviousSourceSample)
            {
                sourceDt = sampleTime - _previousSampleTime;
                if (IsFinite(sourceDt) && sourceDt > 0.0001f && sourceDt < 0.25f)
                {
                    renderStepRadians = QuaternionToAngularVector(relative * Quaternion.Inverse(_previousSourceRelative));
                    angularVelocity = renderStepRadians / sourceDt;
                    angularVelocityValid = IsFinite(angularVelocity);
                }
            }

            // v0.3.66 target-A is differentiated only on real Render samples. FixedUpdate never differentiates a held W,
            // which avoids injecting sample-and-hold impulses into the physical controller. Two valid W samples are required
            // before A becomes available; invalid/long Render gaps reset the derivative history fail-safe to zero.
            bool angularAccelerationValid = angularVelocityValid && _hasPreviousSourceRelativeAngularVelocitySample;
            if (angularAccelerationValid)
            {
                angularAcceleration = (angularVelocity - _previousSourceRelativeAngularVelocityLocal) / sourceDt;
                if (!IsFinite(angularAcceleration))
                {
                    angularAcceleration = Vector3.zero;
                    angularAccelerationValid = false;
                }
            }

            // v0.3.68 source-side conditioner is now the frozen v0.3.70 production target-A path. It still updates
            // only on real Render samples; FixedUpdate consumes only the held conditioned result.
            _targetAccelerationPersistenceConditioner.ObserveRenderSample(
                _traceEnabled,
                angularAccelerationValid,
                angularAcceleration,
                _report);
            _sampledConditionedSourceRelativeAngularAccelerationLocal =
                _targetAccelerationPersistenceConditioner.CurrentConditionedTargetAccelerationLocal;

            // Bind-relative ROM and adjacent Render-packet motion are different evidence. In particular,
            // a held FixedUpdate must never turn a 22-degree bind offset into a 22-degree target jump.
            _report.SourceAnkleRenderStepValid = angularVelocityValid;
            _report.SourceAnkleRenderStepSeconds = sourceDt;
            _report.SourceAnkleRenderStepDegrees = angularVelocityValid ? renderStepRadians * Mathf.Rad2Deg : Vector3.zero;
            if (_traceEnabled && angularVelocityValid)
            {
                _report.TraceSourceRenderStepSamples++;
                _report.TraceSourceRenderStepAngleMaxDegrees = Mathf.Max(_report.TraceSourceRenderStepAngleMaxDegrees, _report.SourceAnkleRenderStepDegrees.magnitude);
            }

            Quaternion sourceDeltaFromBind = relative * Quaternion.Inverse(_bindSourceRelative);
            Vector3 deltaDegrees = QuaternionToAngularVector(sourceDeltaFromBind) * Mathf.Rad2Deg;

            _report.TargetRenderSampleSequence = renderSequence;
            _report.SourceAnkleRelativeRotation = relative;
            _report.SourceAnkleFromBindRotationVectorDegrees = deltaDegrees;
            _report.SourceAnkleFromBindAngleDegrees = deltaDegrees.magnitude;
            _sampledSourceRelativeAngularVelocityLocal = angularVelocity;
            _sampledSourceRelativeAngularAccelerationLocal = angularAcceleration;
            _relativeReference.ObserveRender(_traceEnabled, source, relative, angularVelocity, sourceDt);
            _report.SourceAnkleRelativeAngularVelocityLocal = angularVelocity;
            _report.SourceAnkleRelativeAngularAccelerationLocal = angularAcceleration;
            _report.SourceAnkleRelativeAngularSpeed = angularVelocity.magnitude;
            _report.SampledFootLinearVelocityWorld = _sampledFootLinearVelocityWorld;

            _previousFootPosition = footPosition;
            _previousFootPositionTime = sampleTime;
            _hasPreviousFootPositionSample = true;
            _previousSourceRelative = relative;
            _previousSampleTime = sampleTime;
            _hasPreviousSourceSample = true;
            if (angularVelocityValid)
            {
                _previousSourceRelativeAngularVelocityLocal = angularVelocity;
                _hasPreviousSourceRelativeAngularVelocitySample = true;
            }
            else
            {
                _previousSourceRelativeAngularVelocityLocal = Vector3.zero;
                _hasPreviousSourceRelativeAngularVelocitySample = false;
            }

            // A (and B while leg drive is OFF) follows the source kinematically. During active B the Dynamic Foot
            // must never be teleported by render sampling; source samples remain target/read-only witnesses only.
            if (!_ankleDrive.DynamicActive)
                FollowSourceKinematic();
        }

        public bool ObserveFixed(
            PhysicsStepContext step, int fixedSequence,
            int renderSequence,
            bool fresh,
            int heldDepth,
            float sampleAge,
            bool traceEnabled,
            string phase,
            float kneeAnchorErrorMeters,
            float kneeOffAxisErrorDegrees,
            Vector3 kneeFlexionAxisWorld,
            Vector3 kneeRelativeCommandWorld,
            bool loadedStanceEnvironmentEnabled, LegTurnStepEvidence legTurnEvidence)
        {
            if (!IsBuilt)
                return false;

            if (!step.Valid || !_report.RuntimeSourceContext.Valid ||
                _report.RuntimeSourceContext.Epoch != step.Epoch || _report.RuntimeSourceContext.Sequence != step.SourceSequence)
                throw new InvalidOperationException("FOOT_PHYSICS_SOURCE_CONTEXT_INVALID");
            _report.RuntimeContext = step;

            // Read N+1 before the existing gravity clear, structural readbacks, probe and lifecycle.
            _supportEntityBoundary.ObserveSettledBeforeControllerPreparation(_traceEnabled && traceEnabled,
                fixedSequence, _contactShadow.Active, _contactShadow.AuthorizedColliderId, _collider, _body, _report);

            // V04170 may historically have left native Foot gravity ON for a just-completed stance solve.
            // v0.4.19 freezes V04170 B/ON as the Foot self-weight primitive; this defensive clear still guarantees every frozen controller step starts gravity-OFF.
            // Clear it before any frozen controller/structural contract is evaluated for the next step.
            if (!_stanceFootNativeGravity.PrepareControllerStep(_body, _report))
            {
                _report.Finite = false;
                return false;
            }
            UpdateStaticState();
            _report.FixedUpdateSequence = fixedSequence;
            _report.TargetRenderSampleSequence = renderSequence;
            _report.TargetRenderSampleFreshAtFixedStep = fresh;
            _report.FixedStepsSinceTargetRenderSample = heldDepth;
            _report.TargetRenderSampleAgeSeconds = Mathf.Max(0f, sampleAge);

            if (traceEnabled && !_traceEnabled)
                BeginTrace();
            else if (!traceEnabled && _traceEnabled)
                EndTrace();

            Quaternion currentRelative = Quaternion.Inverse(_lowerBody.rotation) * _body.rotation;
            Vector3 lowerAnchorWorld = _lowerBody.transform.TransformPoint(_lowerAnkleAnchorLocal);
            Vector3 footAnchorWorld = _body.transform.TransformPoint(_footAnkleAnchorLocal);
            float relativeError = Quaternion.Angle(_sampledSourceRelative, currentRelative);
            float anchorError = Vector3.Distance(lowerAnchorWorld, footAnchorWorld);
            bool expectedDynamic = _ankleDrive.DynamicActive;
            bool dynamicConstraintContract = _ankleConstraint.CharacterJointArm
                ? (_ankleJoint != null && _ankleJoint.connectedBody == _lowerBody)
                : _ankleJoint == null;
            bool contactColliderContract = _contactShadow.RuntimeContractValid(_collider);
            bool structuralContract =
                _body != null && !_body.useGravity &&
                _collider != null && contactColliderContract &&
                (_ankleDrive.SelectedDynamicArm
                    ? (dynamicConstraintContract && (expectedDynamic ? !_body.isKinematic : _body.isKinematic))
                    : (_ankleJoint == null && _body.isKinematic));
            bool solverContract = _footVelocitySolver.ValidateReadback(_body, _report);
            bool referenceContract = !_relativeReference.Selected || (_relativeReference.Finite &&
                (expectedDynamic ? (_relativeReference.Active && _relativeReference.Current.Source.Matches(_report.RuntimeSourceContext))
                    : !_relativeReference.Active));
            bool finite = structuralContract && solverContract && referenceContract && _report.FootContactOffsetFinite &&
                          IsFinite(currentRelative) && IsFinite(relativeError) && IsFinite(anchorError) &&
                          IsFinite(_body.position) && IsFinite(_body.rotation) && IsFinite(_sampledSourceRelative);

            if (!structuralContract)
            {
                _report.Detail = "STRUCTURAL_CONTRACT_BROKEN:V0378;FOOT_PRODUCTION=DYNAMIC_SOURCE_RELATIVE_PD_WHEN_DRIVE_ON;CONSTRAINT_PRODUCTION=NO_JOINT+ARL_TRANSLATION_TETHER;LINEAR_B+ANGULAR_B+CONDITIONED_TARGET_A_FROZEN;GRAVITY_OFF;FOOT_COLLIDER=OFF_OR_V0410_AUTHORIZED_CONTACT_SHADOW";
                Plugin.Logger?.LogError("[ARL][FOOT_ANKLE_CONTRACT_BROKEN] side=" + (_report.Side ?? "<none>") +
                                        " | arm=" + _ankleDrive.Arm +
                                        " | dynamicExpected=" + expectedDynamic +
                                        " | kinematic=" + (_body != null && _body.isKinematic) +
                                        " | constraintArm=" + _ankleConstraint.Arm +
                                        " | jointExists=" + (_ankleJoint != null) +
                                        " | connected=" + (_ankleJoint != null && _ankleJoint.connectedBody == _lowerBody) +
                                        " | gravity=" + (_body != null && _body.useGravity) +
                                        " | collider=" + (_collider != null && _collider.enabled));
            }

            _report.Finite = finite;
            if (!referenceContract)
                _report.Detail = "FOOT_REFERENCE_V04730_CONTRACT_FAILED:" + (_relativeReference.Issue ?? "SOURCE_IDENTITY_OR_ACTIVE_STATE");
            _report.AnkleRelativeTrackingErrorDegrees = finite ? relativeError : float.PositiveInfinity;
            _report.AnkleAnchorErrorMeters = finite ? anchorError : float.PositiveInfinity;

            // This readback already exists before contact enable/release and this step's tether/PD writes.
            // Keep a predecessor plus the current sample, including any finite sample that aborts below.
            Vector3 lowerPointVelocity = _lowerBody.GetPointVelocity(lowerAnchorWorld);
            Vector3 footPointVelocity = _body.GetPointVelocity(footAnchorWorld);
            _report.PreviousObservedStep = _report.CurrentObservedStep;
            _report.CurrentObservedStep = new FootFixedStepEvidence
            {
                FixedSequence = fixedSequence,
                Valid = finite && IsFinite(lowerPointVelocity) && IsFinite(footPointVelocity) &&
                        IsFinite(_lowerBody.angularVelocity) && IsFinite(_body.angularVelocity),
                ContactActiveBeforeLifecycle = _contactShadow.Active,
                AnchorErrorMeters = anchorError,
                TrackingErrorDegrees = relativeError,
                LowerAnkleVelocityWorld = lowerPointVelocity,
                FootAnkleVelocityWorld = footPointVelocity,
                LowerAngularVelocityWorld = _lowerBody.angularVelocity,
                FootAngularVelocityWorld = _body.angularVelocity
            };
            if (_traceEnabled && _report.CurrentObservedStep.Valid)
            {
                _report.TraceObservedFixedSamples++;
                _report.TraceObservedAnchorErrorMaxMeters = Mathf.Max(_report.TraceObservedAnchorErrorMaxMeters, anchorError);
                _report.TraceObservedTrackingErrorMaxDegrees = Mathf.Max(_report.TraceObservedTrackingErrorMaxDegrees, relativeError);
            }

            // Settle the prior diagnostic command before lifecycle/current controllers, including a finite gate endpoint.
            _turnEvents.ObserveSettled(_traceEnabled, step, _report.CurrentObservedStep, _lowerBody, _body,
                _report.RuntimeSourceContext, ControlRelativeRotation, sampleAge, legTurnEvidence);

            // V04230 duplicate continuity statistics retired in v0.4.41. The joined
            // normal-load/angular/four-boundary ledger below retains the pre-lifecycle endpoint.

            // v0.4.25 observer-only angular-impulse join. This reads the settled N+1 Foot state before
            // lifecycle release or any current-step controller write. It never participates in fail-close.
            _footAngularImpulseAttribution.ObserveSettledPreLifecycle(
                _traceEnabled,
                fixedSequence,
                anchorError,
                _body,
                _report);

            _supportEntityBoundary.RecordSettledAngular(_traceEnabled, fixedSequence,
                _footAngularImpulseAttribution.GetResolvedSolve(fixedSequence), _report);

            _rockingTetherCoupling.ObserveSettledPreLifecycle(_traceEnabled, fixedSequence,
                _body, _lowerBody, _footAnkleAnchorLocal, _lowerAnkleAnchorLocal,
                _report);
            _supportEntityBoundary.RecordSettledNormalLoad(_traceEnabled, fixedSequence,
                _rockingTetherCoupling.GetSettledNormalLoad(fixedSequence), _report);

            if (!solverContract)
                _report.Detail = "V04270_FOOT_VELOCITY_SOLVER_CONTRACT_FAILED:" + _report.FootVelocitySolverInvalidReason;

            if (finite)
            {
                Vector3 soleGroundEligibilityVelocity = _ankleDrive.DynamicActive && _body != null
                    ? _body.velocity
                    : _sampledFootLinearVelocityWorld;
                if (!_soleRequalification.Observe(
                    _traceEnabled,
                    phase,
                    _body,
                    _collider,
                    _target.Foot,
                    _target.Toe,
                    _targetRoot,
                    _ownerRoot,
                    _bindSourceRelative,
                    _sampledSourceRelative,
                    soleGroundEligibilityVelocity,
                    _report))
                {
                    _report.Finite = false;
                    return false;
                }
                _supportEntityBoundary.RecordSettledProbe(_traceEnabled, fixedSequence, _report);

                // v0.4.0 Stage-E identity qualification consumes the already-computed five-point sole witness only.
                // It performs no additional raycast and cannot fail/modify the physical controller.
                _contactIdentity.Observe(_traceEnabled, phase, soleGroundEligibilityVelocity, step.Dt, _report);

                // v0.4.5 observer-only game-native locomotion phase qualification. The formal v0.4.5 run uses
                // arm A (Foot Collider OFF), so this consumes only source Foot velocity + the already-executed
                // five-point sole probe. No extra raycast, Collider toggle, Rigidbody write or contact authority.
                Vector3 gaitReferenceUp = _targetRoot != null ? _targetRoot.up : Vector3.up;
                bool gaitQualificationSampleEligible = !_contactShadow.Active && _collider != null && !_collider.enabled;
                _gameNativeGaitPhase.ObserveFixed(
                    _traceEnabled,
                    gaitQualificationSampleEligible,
                    _collider != null,
                    fixedSequence,
                    step.Dt,
                    _sampledFootLinearVelocityWorld,
                    gaitReferenceUp,
                    _soleRequalification.CurrentGroundHitCount,
                    _report.GroundFootprintSameSurface,
                    _report.GroundColliderInstanceId,
                    _soleRequalification.CurrentGroundMaxAbsClearanceMeters,
                    _soleRequalification.CurrentGroundAverageNormalWorld,
                    _report);

                // v0.4.8 B must guarantee that the Foot zero-friction material cannot be overridden by an
                // authorized ground material using non-zero Maximum combine. Validate only when a real stance
                // contact candidate exists; swing/no-hit samples are not contact candidates.
                if (_stanceContactFrictionIsolation.ZeroFriction &&
                    _gameNativeGaitPhase.CurrentStrongStanceSample && _report.ContactIdentityCurrentQualified &&
                    !_stanceContactFrictionIsolation.ValidateGroundCompatibility(_soleRequalification.CurrentGroundCollider, _report, out string frictionGroundError))
                {
                    _report.Finite = false;
                    _report.Detail = frictionGroundError;
                    return false;
                }

                // v0.4.7: per-foot stance-only contact lifecycle. The classifier state above is available in the
                // same FixedUpdate. On the first TRANSITION_TO_SWING sample this disables contact before any
                // remaining Foot controller work / Unity physics advance. Landing is gated by confirmed STANCE,
                // current v0.4.0 exact static identity, a short same-Collider hold, and (v0.4.16.1) two consecutive
                // current physical-sole geometry-ready samples before the collider may transition OFF->ON.
                if (!_stanceContactLifecycle.Step(
                    _traceEnabled,
                    fixedSequence,
                    step.Dt,
                    _gameNativeGaitPhase.State,
                    _gameNativeGaitPhase.CurrentStrongStanceSample,
                    _report.ContactIdentityCurrentQualified,
                    _report.ContactIdentityCurrentColliderInstanceId,
                    _report.ContactIdentityCurrentColliderPath,
                    _soleRequalification.CurrentGroundCollider,
                    _contactShadow,
                    _collider,
                    _targetRoot,
                    _report))
                {
                    _report.Finite = false;
                    return false;
                }

                // v0.4.1: exact-support contact shadow is the only low-level stage allowed to enable the Foot Collider.
                // v0.4.7 only orchestrates this closed primitive by per-foot gait phase; the shadow still validates
                // the authorized Collider object lifecycle and raycast identity every active FixedUpdate.
                if (!_contactShadow.ObserveFixed(_traceEnabled, _report))
                {
                    _report.Finite = false;
                    return false;
                }
                // Validate the newly activated Foot pair before this step can reach a native solve.
                if (_supportSurface != null && !_supportSurface.ValidateFootPair(_collider, _traceEnabled, out string surfaceError))
                {
                    _report.Finite = false;
                    _report.Detail = "SUPPORT_SURFACE_CONTRACT:" + surfaceError;
                    return false;
                }
                _report.PhysicalGroundContactPossible = _contactShadow.Active && _collider != null && _collider.enabled;

                // V04110/V04130 research statistics retired after Lower ground ownership closed.
                // Foot world-ground anti-slip stays retired; the Foot-Lower tether owns local closure.

                // v0.3.64: measured physical Lower ankle-point acceleration feedforward is a separate causal selector.
                // It is meaningful only in the no-CharacterJoint tether arm and never changes the final tether MaxA.
                Vector3 ankleTransportFeedforwardWorld = Vector3.zero;
                bool tetherDynamicActive = _ankleDrive.DynamicActive && !_ankleConstraint.CharacterJointArm;
                if (!_ankleTransport.Step(
                    _traceEnabled,
                    tetherDynamicActive,
                    _lowerBody,
                    _lowerAnkleAnchorLocal,
                    step.Dt,
                    _report,
                    out ankleTransportFeedforwardWorld))
                {
                    _report.Finite = false;
                    return false;
                }

                // v0.3.63 positional-backend provenance retained; v0.3.73 keeps the frozen no-CharacterJoint tether topology.
                // The promoted production law changes only the final translation cap inside the unchanged narrow eligibility
                // window. In the positional B arm the native Joint is absent and this applies only a bounded Foot-COM
                // acceleration. v0.3.64 composes optional measured parent transport with the exact
                // same relative PD correction before the v0.3.73 production selected clamp (normally 60, conditionally 80).
                // v0.4.18 freezes the v0.4.17.1 predictive B guard: it may only de-clamp that already-computed command above 80 when the
                // one-step radial model forecasts crossing the unchanged 30 mm gate; it never changes command direction.
                _report.AnkleConstraintLastEvaluatedFixedSequence = fixedSequence;
                if (!_ankleConstraint.Step(
                    _traceEnabled,
                    _ankleDrive.DynamicActive,
                    _body,
                    _lowerBody,
                    _footAnkleAnchorLocal,
                    _lowerAnkleAnchorLocal,
                    _bindSourceRelative,
                    ankleTransportFeedforwardWorld,
                    fixedSequence,
                    step.Dt,
                    _report,
                    _rockingTetherCoupling,
                    CurrentAuthorizedSupportNormalValid(),
                    _soleRequalification.CurrentGroundAverageNormalWorld,
                    loadedStanceEnvironmentEnabled && ExactAuthorizedStanceSupportLoadEligible &&
                    _stanceFootNativeGravity.Enabled && _footCollisionDetectionMode.EnabledSpeculative &&
                    _stanceSupportAngular.Enabled))
                {
                    _report.Finite = false;
                    return false;
                }

                // Frozen production: the Lower-COM controller is the single stance planar owner.
                if (!_stanceLowerGroundAuthority.Step(
                    _traceEnabled,
                    fixedSequence,
                    _contactShadow.Active && _report.ContactShadowCurrentContactActive,
                    CurrentAuthorizedSupportNormalValid(),
                    _stanceContactFrictionIsolation.ZeroFriction,
                    _contactShadow.AuthorizedColliderId,
                    _lowerBody,
                    _lowerAnkleAnchorLocal,
                    _soleRequalification.CurrentGroundAverageNormalWorld,
                    _report))
                {
                    _report.Finite = false;
                    return false;
                }

                // v0.3.78 production: the v0.3.76 bounded common-mode drag compensation, confirmed by v0.3.77 under
                // stronger Lower-W coverage, is now permanently part of the single Foot-only angular transport actuator.
                if (!_ankleAngularTransport.Step(
                    _traceEnabled,
                    _ankleDrive.DynamicActive && !_ankleConstraint.CharacterJointArm,
                    _body,
                    _lowerBody,
                    step.Dt,
                    _report))
                {
                    _report.Finite = false;
                    return false;
                }

                _report.AnkleDriveLastEvaluatedFixedSequence = fixedSequence;
                if (!_ankleDrive.Step(
                    _traceEnabled,
                    _body,
                    _ankleJoint,
                    _ankleConstraint.CharacterJointArm,
                    _lowerBody,
                    _footAnkleAnchorLocal,
                    _lowerAnkleAnchorLocal,
                    _bindSourceRelative,
                    ControlRelativeRotation,
                    ControlRelativeW,
                    ControlRelativeA,
                    ControlRelativeFF,
                    _report.TargetRenderSampleAgeSeconds,
                    _report))
                {
                    _report.Finite = false;
                    return false;
                }

                // V04260 single-variable damping: exact authorized static stance, zero friction, CCD,
                // no Joint, dynamic Foot only. It adds no linear support or attitude/yaw target.
                if (!_stanceSupportAngular.Step(_traceEnabled, fixedSequence, step.Dt,
                    ExactAuthorizedStanceSupportLoadEligible, _footCollisionDetectionMode.EnabledSpeculative,
                    _stanceContactFrictionIsolation.ZeroFriction,
                    _ankleDrive.DynamicActive && !_ankleConstraint.CharacterJointArm,
                    _body, _soleRequalification.CurrentGroundAverageNormalWorld, _report))
                {
                    _report.Finite = false;
                    return false;
                }

                // v0.4.25 observer-only angular-impulse attribution. The three direct Foot AddTorque owners have
                // already issued their requests, so their exact commanded torque impulse can be separated from the
                // later PhysX/contact/drag angular response without changing any physical write. ContactPoint APIs
                // remain deliberately unused because v0.4.2.1 proved them unavailable on this IL2CPP runtime.
                _footAngularImpulseAttribution.CapturePreSolve(
                    _traceEnabled,
                    fixedSequence,
                    step.Dt,
                    _contactShadow.Active && _report.ContactShadowCurrentContactActive && CurrentAuthorizedSupportNormalValid(),
                    _body,
                    _collider,
                    _ankleAngularTransport.CurrentTorqueWorld,
                    _ankleDrive.CurrentTorqueWorld,
                    _stanceSupportAngular.CurrentTorqueWorld,
                    _soleRequalification.GetCurrentProbeSnapshot(),
                    _report);

                // v0.4.15 cleanup: the closed v0.4.4.1 axial-debt ring observer was retired; the unchanged
                // angular hard gate remains authoritative and zero-friction is the frozen stance contact material.
                // v0.4.3 observer-only pre-physics witness. This is sampled after the frozen controller has issued
                // the current FixedUpdate's force/torque requests but before Unity advances the physics step.
                // Callback-time state can therefore be compared with this settled pre-step state without writing
                // anything back into the plant. Observer validity is intentionally not part of Foot fail-close.
                _mechanicalReaction.CapturePrePhysics(
                    _traceEnabled,
                    _contactShadow.Active,
                    _body,
                    step.Dt,
                    fixedSequence,
                    _report);

                // V04170 Foot self-weight selector is frozen B/ON by v0.4.19 at this existing post-controller scheduling point; the
                // gravity boundary is explicitly read back every FixedUpdate after every frozen Foot
                // controller has completed, so the controller contract itself remains byte-intent equivalent.
                // B enables only the Foot Rigidbody for the upcoming exact authorized stance solve.
                if (!_stanceFootNativeGravity.SchedulePhysicsStep(
                    _traceEnabled,
                    fixedSequence,
                    _contactShadow.Active && _report.ContactShadowCurrentContactActive,
                    CurrentAuthorizedSupportNormalValid(),
                    _stanceContactFrictionIsolation.ZeroFriction,
                    _body,
                    _soleRequalification.CurrentGroundAverageNormalWorld,
                    _report.AnkleConstraintCurrentAppliedAccelerationWorld,
                    step.Dt,
                    _report))
                {
                    _report.Finite = false;
                    return false;
                }

                _report.FootControllerLastCompletedFixedSequence = fixedSequence;

            }

            if (_traceEnabled && finite)
            {
                _report.TraceSamples++;
                _traceErrorSum += relativeError;
                _traceAnchorSum += anchorError;
                _traceSourceAngularSpeedSum += _report.SourceAnkleRelativeAngularSpeed;
                _traceSourceFromBindAngleSum += _report.SourceAnkleFromBindAngleDegrees;
                _report.TraceAnkleRelativeErrorMeanDegrees = _traceErrorSum / _report.TraceSamples;
                _report.TraceAnkleRelativeErrorMaxDegrees = Mathf.Max(_report.TraceAnkleRelativeErrorMaxDegrees, relativeError);
                _report.TraceAnkleAnchorErrorMeanMeters = _traceAnchorSum / _report.TraceSamples;
                _report.TraceAnkleAnchorErrorMaxMeters = Mathf.Max(_report.TraceAnkleAnchorErrorMaxMeters, anchorError);
                _report.TraceSourceFromBindAngleMeanDegrees = _traceSourceFromBindAngleSum / _report.TraceSamples;
                _report.TraceSourceFromBindAngleMaxDegrees = Mathf.Max(_report.TraceSourceFromBindAngleMaxDegrees, _report.SourceAnkleFromBindAngleDegrees);
                _report.TraceSourceAngularSpeedMean = _traceSourceAngularSpeedSum / _report.TraceSamples;
                _report.TraceSourceAngularSpeedMax = Mathf.Max(_report.TraceSourceAngularSpeedMax, _report.SourceAnkleRelativeAngularSpeed);
                if (_report.TraceSamples == 1)
                {
                    _report.TraceSourceFromBindAxisMinDegrees = _report.SourceAnkleFromBindRotationVectorDegrees;
                    _report.TraceSourceFromBindAxisMaxDegrees = _report.SourceAnkleFromBindRotationVectorDegrees;
                    _report.TraceSourceAngularVelocityAxisMin = _report.SourceAnkleRelativeAngularVelocityLocal;
                    _report.TraceSourceAngularVelocityAxisMax = _report.SourceAnkleRelativeAngularVelocityLocal;
                }
                else
                {
                    _report.TraceSourceFromBindAxisMinDegrees = ComponentMin(_report.TraceSourceFromBindAxisMinDegrees, _report.SourceAnkleFromBindRotationVectorDegrees);
                    _report.TraceSourceFromBindAxisMaxDegrees = ComponentMax(_report.TraceSourceFromBindAxisMaxDegrees, _report.SourceAnkleFromBindRotationVectorDegrees);
                    _report.TraceSourceAngularVelocityAxisMin = ComponentMin(_report.TraceSourceAngularVelocityAxisMin, _report.SourceAnkleRelativeAngularVelocityLocal);
                    _report.TraceSourceAngularVelocityAxisMax = ComponentMax(_report.TraceSourceAngularVelocityAxisMax, _report.SourceAnkleRelativeAngularVelocityLocal);
                }
                _report.TraceSourceFromBindAxisAbsMaxDegrees = ComponentMaxAbs(_report.TraceSourceFromBindAxisAbsMaxDegrees, _report.SourceAnkleFromBindRotationVectorDegrees);
                _report.TraceSourceAngularVelocityAxisAbsMax = ComponentMaxAbs(_report.TraceSourceAngularVelocityAxisAbsMax, _report.SourceAnkleRelativeAngularVelocityLocal);
            }

            return finite;
        }

        public void BeginTrace()
        {
            _traceEnabled = true;
            _turnEvents.Reset();
            _relativeReference.ResetTrace();
            _report.TurnEventDetails = null;
            _footContactOffset.ResetTrace(_report);

            // Trace is diagnostic-only. Do NOT reset source pose/W/A or foot-velocity derivative history here:
            // those states feed the physical controller/parent-transport paths, so arl_trace(on) must not create
            // an artificial one/two-sample warm-up. Arm/drive/rollback transitions own physical-history resets.
            _traceErrorSum = 0f;
            _traceAnchorSum = 0f;
            _traceSourceAngularSpeedSum = 0f;
            _traceSourceFromBindAngleSum = 0f;
            _report.TraceSamples = 0;
            _report.PreviousObservedStep = default;
            _report.CurrentObservedStep = default;
            _report.AnkleConstraintLastEvaluatedFixedSequence = 0;
            _report.AnkleDriveLastEvaluatedFixedSequence = 0;
            _report.FootControllerLastCompletedFixedSequence = 0;
            _report.TraceObservedFixedSamples = 0;
            _report.TraceObservedAnchorErrorMaxMeters = 0f;
            _report.TraceObservedTrackingErrorMaxDegrees = 0f;
            _report.TraceSourceRenderStepSamples = 0;
            _report.TraceSourceRenderStepAngleMaxDegrees = 0f;
            _report.TraceAnkleRelativeErrorMeanDegrees = 0f;
            _report.TraceAnkleRelativeErrorMaxDegrees = 0f;
            _report.TraceAnkleAnchorErrorMeanMeters = 0f;
            _report.TraceAnkleAnchorErrorMaxMeters = 0f;
            _report.TraceSourceFromBindAngleMeanDegrees = 0f;
            _report.TraceSourceFromBindAngleMaxDegrees = 0f;
            _report.TraceSourceAngularSpeedMean = 0f;
            _report.TraceSourceAngularSpeedMax = 0f;
            _report.TraceSourceFromBindAxisMinDegrees = Vector3.zero;
            _report.TraceSourceFromBindAxisMaxDegrees = Vector3.zero;
            _report.TraceSourceFromBindAxisAbsMaxDegrees = Vector3.zero;
            _report.TraceSourceAngularVelocityAxisMin = Vector3.zero;
            _report.TraceSourceAngularVelocityAxisMax = Vector3.zero;
            _report.TraceSourceAngularVelocityAxisAbsMax = Vector3.zero;
            _soleRequalification.ResetTrace(_report);
            _contactIdentity.ResetTrace(_report);
            _contactShadow.ResetTrace(_report);
            _mechanicalReaction.ResetTrace(_report);

            _footAngularImpulseAttribution.ResetTrace(_report);
            _rockingTetherCoupling.ResetTrace(_report);
            _supportEntityBoundary.ResetTrace(_report);
            _stanceSupportAngular.ResetTrace(_report);
            _footVelocitySolver.ResetTrace(_report);
            _gameNativeGaitPhase.ResetTrace(_report);
            _stanceContactLifecycle.ResetTrace(_report);

            _stanceLowerGroundAuthority.ResetTrace(_report);
            _stanceFootNativeGravity.ResetTrace(_report);
            _ankleDrive.ResetTrace(_report);
            _ankleConstraint.ResetTrace(_report);
            _ankleTransport.ResetTrace(_report);
            _ankleAngularTransport.ResetTrace(_report);
            _targetAccelerationPersistenceConditioner.ResetTrace(_report);
        }

        public void EndTrace()
        {
            _turnEvents.Seal();
            _report.TurnEventDetails = _turnEvents.GetDetails() + "\n" + _relativeReference.GetDetails();

            _contactIdentity.EndTrace(_report);
            _traceEnabled = false;
            _mechanicalReaction.EndTrace(_report);

            _footAngularImpulseAttribution.EndTrace(_report);
            _rockingTetherCoupling.EndTrace(_report);
            _supportEntityBoundary.EndTrace(_report);
            _stanceSupportAngular.CopyToReport(_report);
            _footVelocitySolver.CopyToReport(_report);
            _gameNativeGaitPhase.EndTrace(_report);
            _stanceContactLifecycle.EndTrace(_report);

            _stanceLowerGroundAuthority.EndTrace(_report);
            _stanceFootNativeGravity.EndTrace(_report);
            _targetAccelerationPersistenceConditioner.CopyToReport(_report);
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
            // Never join a pending command to teleported/zeroed cleanup state. Formatting remains stop-time only.
            _turnEvents.Seal();
            _ankleConstraint.QuiesceStancePlanarDamping(_report);
            _rockingTetherCoupling.ClearPendingPreserveTrace(_report);
            _supportEntityBoundary.ClearPendingPreserveTrace(_report);
            _rockingTetherCoupling.CopyToReport(_report, false);
            _supportEntityBoundary.CopyToReport(_report, false);
            _ankleConstraint.GetStancePlanarDampingSummary(_report);
            int failures = 0;
            if (!_footVelocitySolver.QuiescePreserveTrace(_body, _report, out string solverSafeError))
            {
                failures++;
                Plugin.Logger?.LogWarning("[ARL][FOOT_SAFE_STATE_ERROR] side=" + (_report.Side ?? "<none>") + " | velocitySolver | " + solverSafeError);
            }
            if (!_stanceFootNativeGravity.QuiescePreserveTrace(_body, _report, out string gravitySafeError))
            {
                failures++;
                Plugin.Logger?.LogWarning("[ARL][FOOT_SAFE_STATE_ERROR] side=" + (_report.Side ?? "<none>") + " | nativeGravity | " + (gravitySafeError ?? "UNKNOWN"));
            }
            _stanceSupportAngular.ClearCurrentState(_report);

            _relativeReference.DisablePreserveTrace();
            _ankleDrive.SetDriveActive(false);
            _ankleConstraint.ClearCurrentState(_report);
            _ankleTransport.ClearCurrentState(_report);
            _ankleAngularTransport.ClearCurrentState(_report);
            _stanceContactLifecycle.DisableAndReset(_contactShadow, _collider, _report);
            failures += _contactShadow.DisablePreserveTrace(_report);
            try
            {
                if (_collider != null) _collider.enabled = false;
            }
            catch (Exception e)
            {
                failures++;
                Plugin.Logger?.LogWarning("[ARL][FOOT_SAFE_STATE_ERROR] side=" + (_report.Side ?? "<none>") + " | collider | " + e.GetType().Name + " / " + e.Message);
            }
            try
            {
                if (_body != null)
                {
                    _body.useGravity = false;
                    _body.velocity = Vector3.zero;
                    _body.angularVelocity = Vector3.zero;
                    _body.isKinematic = true;
                }
            }
            catch (Exception e)
            {
                failures++;
                Plugin.Logger?.LogWarning("[ARL][FOOT_SAFE_STATE_ERROR] side=" + (_report.Side ?? "<none>") + " | rigidbody | " + e.GetType().Name + " / " + e.Message);
            }
            UpdateStaticState();
            return failures;
        }

        public void FollowSourceKinematic()
        {
            if (!IsBuilt || _body == null)
                return;
            _contactShadow.DisablePreserveTrace(_report);
            _body.isKinematic = true;
            _body.useGravity = false;
            if (_collider != null) _collider.enabled = false;
            _body.position = _sampledFootPosition;
            _body.rotation = _sampledFootRotation;
            _body.velocity = Vector3.zero;
            _body.angularVelocity = Vector3.zero;
            UpdateStaticState();
        }

        public string GetStatus()
        {

            RefreshSoleRequalificationCurrent();
            UpdateStaticState();
            // Formatting stays status-time only; FixedUpdate observers accumulate numeric state only.
            _soleRequalification.CopyToReport(_report);
            _contactIdentity.CopyToReport(_report);
            _contactShadow.CopyToReport(_report);
            _mechanicalReaction.CopyToReport(_report);

            _footAngularImpulseAttribution.CopyToReport(_report);
            _rockingTetherCoupling.CopyToReport(_report);
            _supportEntityBoundary.CopyToReport(_report);
            _ankleConstraint.GetStancePlanarDampingSummary(_report);
            _stanceSupportAngular.CopyToReport(_report);
            _footVelocitySolver.CopyToReport(_report);

            _ankleDrive.CopyToReport(_report);
            _ankleConstraint.CopyToReport(_report);
            _ankleTransport.CopyToReport(_report);
            _ankleAngularTransport.CopyToReport(_report);
            _targetAccelerationPersistenceConditioner.CopyToReport(_report);
            CopyAnkleJointReadback();
            return _report.GetConsoleSummary();
        }

        internal string GetSupportEntityBoundaryEvidence()
        {
            _supportEntityBoundary.CopyToReport(_report);
            _ankleConstraint.GetStancePlanarDampingSummary(_report);
            return "mode=" + _report.SupportEntityBoundaryMode + "/finite=" + _report.SupportEntityBoundaryFinite +
                   "/ready=" + _report.SupportEntityBoundaryEvidenceReady + "/reason=" + (_report.SupportEntityBoundaryInvalidReason ?? "<none>") +
                   " | " + _report.SupportEntityBoundaryTraceSummary + "\n" + _report.SupportEntityBoundaryRecentFrames;
        }

        internal string GetRockingTetherCouplingEvidence()
        {
            _rockingTetherCoupling.CopyToReport(_report);
            return "mode=" + _report.RockingTetherCouplingMode + "/finite=" + _report.RockingTetherCouplingFinite +
                   "/ready=" + _report.RockingTetherCouplingEvidenceReady + "/reason=" + (_report.RockingTetherCouplingInvalidReason ?? "<none>") +
                   " | " + _report.RockingTetherCouplingTraceSummary + "\n" + _report.RockingTetherCouplingRecentFrames;
        }

        public string GetSoleRequalificationStatus()
        {
            RefreshSoleRequalificationCurrent();
            UpdateStaticState();
            _soleRequalification.CopyToReport(_report);
            return "supportCal=" + _report.SupportPlaneCalibrationApplied +
                   "/reason=" + (_report.SupportPlaneCalibrationReason ?? "<none>") +
                   "/heightM(initial-corr-final)=" + F(_report.SupportPlaneCalibrationInitialAnkleHeightMeters) + "-" +
                       F(_report.SupportPlaneCalibrationCorrectionMeters) + "-" + F(_report.SupportPlaneCalibrationFinalAnkleHeightMeters) +
                   "/probe(hits-spreadMm-normal)=" + _report.SupportPlaneCalibrationHitCount + "-" +
                       F(_report.SupportPlaneCalibrationSpreadMeters * 1000f) + "-" + F(_report.SupportPlaneCalibrationNormalDot) +
                   "/surface=" + (_report.SupportPlaneCalibrationSurfacePath ?? "<none>") +
                   "/footwearResolved=" + _report.FootwearStateResolved +
                   "/footwearPolicy=" + (_report.FootwearGeometryPolicy ?? "<none>") +
                   " | " + _soleRequalification.GetStatus(_report);
        }

        private void RefreshSoleRequalificationCurrent()
        {
            if (!IsBuilt) return;
            // Status is allowed before leg-drive/trace. In A (or inactive B) refresh the kinematic pose.
            // Critical v0.3.60 rule retained from v0.3.57: querying status during active B must NEVER teleport the Dynamic Foot.
            if (!_ankleDrive.DynamicActive)
                FollowSourceKinematic();
            Vector3 statusGroundEligibilityVelocity = _ankleDrive.DynamicActive && _body != null
                ? _body.velocity
                : _sampledFootLinearVelocityWorld;
            _soleRequalification.Observe(
                false,
                "STATUS",
                _body,
                _collider,
                _target.Foot,
                _target.Toe,
                _targetRoot,
                _ownerRoot,
                _bindSourceRelative,
                _sampledSourceRelative,
                statusGroundEligibilityVelocity,
                _report);
            _contactIdentity.Observe(false, "STATUS", statusGroundEligibilityVelocity, Time.fixedDeltaTime, _report);
        }

        public string GetContactIdentityStatus()
        {
            RefreshSoleRequalificationCurrent();
            _contactIdentity.CopyToReport(_report);
            return _contactIdentity.GetStatus(_report);
        }

        public string GetContactShadowStatusText()
        {
            UpdateStaticState();
            _mechanicalReaction.CopyToReport(_report);

            return _contactShadow.GetStatus(_report);
        }

        public string GetMechanicalReactionStatusText()
        {
            UpdateStaticState();
            return GetMechanicalReactionStatus();
        }

        public string GetAnkleDriveStatus()
        {
            return GetAnkleDrivePhysicalStatus() + " | v063=" + GetAnkleConstraintStatus() + " | v064=" + GetAnkleTransportStatus() + " | v078AngularTransport=" + GetAnkleAngularTransportStatus();
        }

        public string GetAnkleDrivePhysicalStatus()
        {
            UpdateStaticState();
            _ankleDrive.CopyToReport(_report);
            CopyAnkleJointReadback();
            return _ankleDrive.GetStatus(_report);
        }

        public string GetAnkleConstraintStatus()
        {
            _ankleConstraint.CopyToReport(_report);
            bool destroyPending = RefreshPendingAnkleJointDestroy();
            return _ankleConstraint.GetStatus(_report) + "/jointDestroyPending=" + destroyPending;
        }

        public string GetAnkleTransportStatus()
        {
            _ankleTransport.CopyToReport(_report);
            return _ankleTransport.GetStatus(_report);
        }

        public string GetAnkleAngularTransportStatus()
        {
            _ankleAngularTransport.CopyToReport(_report);
            return _ankleAngularTransport.GetStatus(_report);
        }

        public string GetAnkleTargetAccelerationStatus()
        {
            _ankleDrive.CopyToReport(_report);
            return "ARL Ankle target-A FF | mode=" + _ankleDrive.TargetAccelerationArm +
                   " | enabled=" + _ankleDrive.TargetAccelerationFeedforwardEnabled +
                   " | v0.3.78PhysicalPolicy=FROZEN_PERSISTENCE_CONDITIONED_TARGET_A" +
                   " | gain/bound/finalMaxA/finalMaxTau=" +
                   FootAnkleSourceDrivenRelativePdIsolation.TargetAccelerationFeedforwardGain.ToString("G9", CultureInfo.InvariantCulture) + "/" +
                   FootAnkleSourceDrivenRelativePdIsolation.MaxTargetAccelerationFeedforward.ToString("G9", CultureInfo.InvariantCulture) + "/" +
                   FootAnkleSourceDrivenRelativePdIsolation.MaxRelativeAngularAcceleration.ToString("G9", CultureInfo.InvariantCulture) + "/" +
                   FootAnkleSourceDrivenRelativePdIsolation.MaxInternalTorqueNm.ToString("G9", CultureInfo.InvariantCulture);
        }

        public string GetTargetAccelerationPersistenceConditionerStatus()
        {
            return _targetAccelerationPersistenceConditioner.GetStatus(_report);
        }

        public string GetTranslationCaptureAuthorityStatus()
        {
            return _ankleConstraint.GetCaptureAuthorityStatus(_report);
        }

        public string TranslationCaptureAuthorityArm => _ankleConstraint.CaptureAuthorityArm;
        public bool TranslationCaptureAuthorityProduction => _ankleConstraint.CaptureAuthorityProduction;

        public string TranslationPredictiveTailCaptureArm => _ankleConstraint.PredictiveTailCaptureArm;
        public bool TranslationPredictiveTailCaptureEnabled => _ankleConstraint.PredictiveTailCaptureEnabled;
        public bool TranslationPredictiveTailCaptureFinite => _ankleConstraint.PredictiveTailCaptureFinite;

        internal bool SetTranslationPredictiveTailCaptureArm(string arm, out string error)
        {
            bool ok = _ankleConstraint.SetPredictiveTailCaptureArm(arm, _report, out error);
            UpdateStaticState();
            return ok;
        }

        public string AnkleDriveArm => _ankleDrive.Arm;
        public bool AnkleDriveProductionArm => _ankleDrive.IsProductionArm;
        public string AnkleConstraintArm => _ankleConstraint.Arm;
        public bool AnkleConstraintProductionArm => _ankleConstraint.IsProductionSelectorArm;
        public bool AnkleConstraintCharacterJointArm => _ankleConstraint.CharacterJointArm;
        public string AnkleTransportArm => _ankleTransport.Arm;
        public bool AnkleTransportFeedforwardEnabled => _ankleTransport.Enabled;
        public string AnkleAngularTransportPolicy => _ankleAngularTransport.Policy;
        public bool AnkleAngularTransportFeedforwardEnabled => _ankleAngularTransport.Enabled;
        public bool AnkleAngularTransportProductionReady => _ankleAngularTransport.Enabled &&
                                                     _body != null &&
                                                     Mathf.Abs(_body.angularDrag - _profile.AngularDrag) <= 0.0001f;
        public float FootAngularDrag => _body != null ? _body.angularDrag : _profile.AngularDrag;
        internal Rigidbody DynamicBody => _body;
        internal bool FreeSpaceNativeState(bool dynamicExpected) => IsBuilt && _body.isKinematic == !dynamicExpected &&
            !_body.useGravity && _collider != null && !_collider.enabled && _ankleJoint == null &&
            IsFinite(_body.position) && IsFinite(_body.rotation) && IsFinite(_body.velocity) && IsFinite(_body.angularVelocity);
        public string AnkleTargetAccelerationMode => _ankleDrive.TargetAccelerationArm;
        public bool AnkleTargetAccelerationFeedforwardEnabled => _ankleDrive.TargetAccelerationFeedforwardEnabled;
        public bool AnkleJointDestroyPending => RefreshPendingAnkleJointDestroy();
        public string CurrentInvalidReason
        {
            get
            {
                if (_supportSurface != null && !_supportSurface.Finite) return "SUPPORT_SURFACE_CONTRACT:" + _supportSurface.InvalidReason;
                if (!_stanceSupportAngular.Finite) return _report.StanceSupportAngularInvalidReason ?? "STANCE_SUPPORT_ANGULAR_INVALID";

                if (!_report.FootContactOffsetFinite) return _report.FootContactOffsetInvalidReason ?? "FOOT_CONTACT_OFFSET_INVALID";
                if (!_report.StanceContactLifecycleFinite) return _report.StanceContactLifecycleInvalidReason ?? "STANCE_CONTACT_LIFECYCLE_INVALID";
                if (!_report.StanceContactFrictionIsolationFinite) return _report.StanceContactFrictionIsolationInvalidReason ?? "STANCE_CONTACT_FRICTION_ISOLATION_INVALID";
                if (!_report.StanceLowerGroundAuthorityFinite) return _report.StanceLowerGroundAuthorityInvalidReason ?? "STANCE_LOWER_GROUND_AUTHORITY_INVALID";
                if (!_report.StanceFootNativeGravityFinite) return _report.StanceFootNativeGravityInvalidReason ?? "STANCE_FOOT_NATIVE_GRAVITY_INVALID";
                if (!_report.ContactShadowFinite) return _report.ContactShadowInvalidReason ?? "FOOT_CONTACT_SHADOW_INVALID";
                if (!_report.AnkleAngularTransportFinite) return _report.AnkleAngularTransportInvalidReason ?? "ANKLE_ANGULAR_TRANSPORT_INVALID";
                if (!_report.AnkleTransportFinite) return _report.AnkleTransportInvalidReason ?? "ANKLE_TRANSPORT_INVALID";
                if (!_report.TranslationPredictiveTailCaptureFinite) return _report.TranslationPredictiveTailCaptureInvalidReason ?? "ANKLE_TRANSLATION_PREDICTIVE_TAIL_CAPTURE_INVALID";
                if (!_report.AnkleConstraintFinite) return _report.AnkleConstraintInvalidReason ?? "ANKLE_CONSTRAINT_INVALID";
                if (!_report.AnkleDriveFinite) return _report.AnkleDriveInvalidReason ?? "ANKLE_DRIVE_INVALID";
                if (!_report.SoleRequalificationFinite) return _report.SoleRequalificationInvalidReason ?? "SOLE_REQUAL_INVALID";
                if (!_report.Finite) return _report.Detail ?? "FOOT_FOUNDATION_INVALID";
                return null;
            }
        }

        internal void RefreshSupportProbe() => RefreshSoleRequalificationCurrent();
        internal void BindSupportSurface(FootSupportSurfaceIsolation surface)
        {
            _supportSurface = surface;
            _soleRequalification.BindSupportSurface(surface);
            _contactShadow.BindSupportSurface(surface);
        }

        internal Collider CurrentQualifiedGroundCollider => _soleRequalification.CurrentGroundCollider;
        // v0.4.1.1: bilateral coordinator needs the exact proxy Collider object so Foot↔Foot self-collision
        // can be isolated as an explicit structural pair before either Foot collider is enabled.
        internal Collider ContactShadowFootCollider => _collider;
        internal string ContactShadowArm => _contactShadow.Arm;
        internal bool ContactShadowActive => _contactShadow.Active;
        internal bool ContactShadowFinite => _contactShadow.Finite;
        internal bool StanceContactLifecycleEnabled => _stanceContactLifecycle.Enabled;
        internal bool StanceContactLifecycleFinite => _stanceContactLifecycle.Finite;
        internal string StanceContactFrictionIsolationArm => _stanceContactFrictionIsolation.Arm;
        internal bool StanceContactFrictionIsolationFinite => _stanceContactFrictionIsolation.Finite;
        internal bool StanceContactZeroFriction => _stanceContactFrictionIsolation.ZeroFriction;
        // v0.4.19: narrow read-only bridge for the SingleLeg-owned Lower gravity selector.
        // It reuses the exact same current Foot contact/support-normal witness as V04170; no new raycast or authority.
        internal bool ExactAuthorizedStanceSupportLoadEligible =>
            _contactShadow.Active && _report.ContactShadowCurrentContactActive && CurrentAuthorizedSupportNormalValid() && _stanceContactFrictionIsolation.ZeroFriction;
        internal Vector3 CurrentAuthorizedSupportNormalWorld => _soleRequalification.CurrentGroundAverageNormalWorld;
        internal string StanceLowerGroundAuthorityArm => _stanceLowerGroundAuthority.Arm;
        internal bool StanceLowerGroundAuthorityEnabled => _stanceLowerGroundAuthority.Enabled;
        internal bool StanceLowerGroundAuthorityFinite => _stanceLowerGroundAuthority.Finite;
        internal string StanceFootNativeGravityArm => _stanceFootNativeGravity.Arm;
        internal bool StanceFootNativeGravityEnabled => _stanceFootNativeGravity.Enabled;
        internal bool StanceFootNativeGravityFinite => _stanceFootNativeGravity.Finite;

        internal string FootPositionSolverArm => _footVelocitySolver.PositionArm;
        internal bool FootPositionSolverEnabled => _footVelocitySolver.PositionEnabled;
        internal bool SetFootPositionSolverArm(string arm, out string error)
        {
            bool ok = _footVelocitySolver.SelectPositionArm(arm, _body, _report, out error);
            UpdateStaticState();
            return ok;
        }

        internal string FootVelocitySolverArm => _footVelocitySolver.Arm;
        internal bool FootVelocitySolverFinite => _footVelocitySolver.Finite;
        internal bool FootVelocitySolverBaselineRestored => _footVelocitySolver.BaselineRestored;
        internal bool StancePlanarDampingEnabled => _ankleConstraint.StancePlanarDampingEnabled;
        internal void SetStancePlanarDamping(bool enabled)
        {
            _ankleConstraint.SetStancePlanarDamping(enabled, _report);
        }
        internal bool RestoreFootSolverIterationBaseline(out string error)
        {
            bool ok = _footVelocitySolver.SelectCapturedBaseline(_body, _report, out error);
            UpdateStaticState();
            return ok;
        }

        internal string FootCollisionDetectionModeArm => _footCollisionDetectionMode.Arm;
        internal string StanceSupportAngularArm => _stanceSupportAngular.Arm;

        internal bool StanceSupportAngularEnabled => _stanceSupportAngular.Enabled;
        internal bool StanceSupportAngularFinite => _stanceSupportAngular.Finite;
        internal bool SetStanceSupportAngularArm(string arm, out string error)
        {
            return _stanceSupportAngular.SetArm(arm, _report, out error);
        }

        internal bool FootCollisionDetectionModeFinite => _footCollisionDetectionMode.Finite;

        internal bool SetStanceContactFrictionIsolationArm(string arm, out string error)
        {
            bool ok = _stanceContactFrictionIsolation.SetArm(arm, _collider, _report, out error);
            UpdateStaticState();
            return ok;
        }

        internal bool SetStanceLowerGroundAuthorityArm(string arm, out string error)
        {
            bool ok = _stanceLowerGroundAuthority.SetArm(arm, _report, out error);
            UpdateStaticState();
            return ok;
        }

        internal bool SetStanceFootNativeGravityArm(string arm, out string error)
        {
            bool ok = _stanceFootNativeGravity.SetArm(arm, _body, _report, out error);
            UpdateStaticState();
            return ok;
        }

        internal bool SetFootCollisionDetectionModeArm(string arm, out string error)
        {
            bool ok = _footCollisionDetectionMode.SetArm(arm, _body, _report, out error);
            UpdateStaticState();
            return ok;
        }

        internal bool SetFootContactOffsetArm(string arm, out string error)
        {
            bool ok = _footContactOffset.SetArm(arm, _collider, _body, _report, out error);
            UpdateStaticState();
            // UpdateStaticState performs another contract read. Do not return success if that later read failed.
            bool ready = ok && _report.FootContactOffsetFinite && _report.FootContactOffsetReadbackValid &&
                string.Equals(_report.FootContactOffsetArm, arm, StringComparison.Ordinal);
            if (!ready && string.IsNullOrEmpty(error)) error = _report.FootContactOffsetInvalidReason ?? "FOOT_CONTACT_OFFSET_POST_SET_CONTRACT_FAILED";
            return ready;
        }

        internal void SetStanceContactLifecycleEnabled(bool enabled)
        {
            if (!enabled)
                _stanceFootNativeGravity.QuiescePreserveTrace(_body, _report, out _);
            _stanceContactLifecycle.SetEnabled(enabled, _contactShadow, _collider, _report);
            UpdateStaticState();
        }

        internal void DisableStanceContactLifecycle()
        {
            _stanceFootNativeGravity.QuiescePreserveTrace(_body, _report, out _);
            _stanceContactLifecycle.DisableAndReset(_contactShadow, _collider, _report);
            UpdateStaticState();
        }

        internal bool QuiesceStanceContactLifecyclePreserveTrace(out string error)
        {
            _ankleConstraint.QuiesceStancePlanarDamping(_report);
            _stanceSupportAngular.ClearCurrentState(_report);
            _stanceSupportAngular.CopyToReport(_report);
            _footVelocitySolver.CopyToReport(_report);
            _rockingTetherCoupling.ClearPendingPreserveTrace(_report);
            _supportEntityBoundary.ClearPendingPreserveTrace(_report);
            _rockingTetherCoupling.CopyToReport(_report, false);
            _supportEntityBoundary.CopyToReport(_report, false);
            _ankleConstraint.GetStancePlanarDampingSummary(_report);
            bool solverOk = _footVelocitySolver.QuiescePreserveTrace(_body, _report, out string solverError);
            bool gravityOk = _stanceFootNativeGravity.QuiescePreserveTrace(_body, _report, out string gravityError);
            bool contactOk = _stanceContactLifecycle.QuiescePreserveTrace(_contactShadow, _report, out string contactError);
            error = !solverOk ? solverError : !gravityOk ? (gravityError ?? "FOOT_NATIVE_GRAVITY_QUIESCE_FAILED") : contactError;
            UpdateStaticState();
            return solverOk && gravityOk && contactOk;
        }

        internal void SetContactShadowArmA()
        {
            _contactShadow.SetArmA(_collider, _report);
            UpdateStaticState();
        }

        internal bool PrepareContactShadowArmB(Collider authorizedCollider, out string error)
        {
            bool ok = _contactShadow.PrepareArmB(_collider, authorizedCollider, _targetRoot, _report, out error);
            UpdateStaticState();
            return ok;
        }

        internal bool EnablePreparedContactShadowArmB(out string error)
        {
            bool ok = _contactShadow.EnablePreparedArmB(_report, out error);
            UpdateStaticState();
            return ok;
        }

        internal int DisableContactShadowPreserveTrace()
        {
            int failures = _contactShadow.DisablePreserveTrace(_report);
            UpdateStaticState();
            return failures;
        }

        internal bool NotifyContactShadowCollision(string phase, Collision collision)
        {
            bool ok = _contactShadow.NotifyCollision(phase, collision, _report);
            _supportEntityBoundary.ObserveCallbackCensus(_traceEnabled, _report.FixedUpdateSequence,
                _contactShadow.LastCallbackWasAuthorized, ok);
            if (ok && _contactShadow.LastCallbackWasAuthorized)
            {
                // v0.4.3.2: v0.4.2.1 already proved the direct solver witnesses are readable while
                // Collision.GetContact(int) is not available on this IL2CPP runtime. Reuse the independently
                // qualified five-point sole support normal instead of the dead ContactPoint path.
                _mechanicalReaction.ObserveCollision(
                    _traceEnabled,
                    _contactShadow.Active,
                    _body,
                    _sampledFootPosition,
                    _sampledFootRotation,
                    _contactShadow.AuthorizedCollider,
                    _soleRequalification.CurrentGroundAverageNormalWorld,
                    _report.GroundGeometryReady,
                    _report.GroundFootprintSameSurface,
                    _report.GroundColliderInstanceId,
                    _soleRequalification.CurrentGroundHitCount,
                    _soleRequalification.CurrentPrimarySameSurface,
                    _soleRequalification.CurrentGroundMaxAbsClearanceMeters,
                    _soleRequalification.CurrentGroundSpreadMeters,
                    _soleRequalification.CurrentGroundNormalDotUp,
                    _soleRequalification.GroundReadyAbsClearanceThreshold,
                    _soleRequalification.GroundReadySpreadThreshold,
                    _soleRequalification.GroundNormalDotThreshold,
                    _soleRequalification.GetCurrentProbeSnapshot(),
                    phase,
                    collision,
                    _report);

                _footAngularImpulseAttribution.ObserveCollision(
                    _traceEnabled,
                    _mechanicalReaction.LastCollisionPreFixedSequence,
                    _mechanicalReaction.LastPhase,
                    _mechanicalReaction.LastProjectionValidThisCallback,
                    _mechanicalReaction.LastImpulseWorld,
                    _mechanicalReaction.LastSupportNormalWorld,
                    _mechanicalReaction.LastSignedNormalImpulse,
                    _mechanicalReaction.LastTangentImpulse,
                    _report);
                _rockingTetherCoupling.ObserveCollision(_traceEnabled,
                    _mechanicalReaction.LastCollisionPreFixedSequence, phase,
                    _mechanicalReaction.LastProjectionValidThisCallback,
                    _mechanicalReaction.LastImpulseWorld, _report);
                _supportEntityBoundary.ObserveAuthorizedCallback(_traceEnabled, _report.FixedUpdateSequence, phase, _contactShadow.AuthorizedColliderId,
                    _mechanicalReaction.LastProjectionValidThisCallback, _mechanicalReaction.LastImpulseWorld,
                    _collider, _body, _report);
                if (_mechanicalReaction.LastProjectionValidThisCallback && _mechanicalReaction.LastSupportNormalWorld.sqrMagnitude > 0.0001f)
                {
                    _stanceFootNativeGravity.ObserveAuthorizedReaction(
                        _traceEnabled,
                        _body,
                        _mechanicalReaction.LastSupportNormalWorld,
                        _mechanicalReaction.LastSignedNormalImpulse,
                        _report);
                }
            }
            UpdateStaticState();
            return ok;
        }

        internal string GetMechanicalReactionStatus()
        {
            UpdateStaticState();
            _mechanicalReaction.CopyToReport(_report);

            _footAngularImpulseAttribution.CopyToReport(_report);
            _rockingTetherCoupling.CopyToReport(_report);
            _supportEntityBoundary.CopyToReport(_report);
            _ankleConstraint.GetStancePlanarDampingSummary(_report);
            _stanceSupportAngular.CopyToReport(_report);
            _footVelocitySolver.CopyToReport(_report);
            return "mode=" + (_report.MechanicalReactionMode ?? "<none>") +
                   "/finite=" + _report.MechanicalReactionFinite +
                   "/reason=" + (_report.MechanicalReactionInvalidReason ?? "<none>") +
                   " | current=" + (_report.MechanicalReactionCurrentSummary ?? "<none>") +
                   " | trace=" + (_report.MechanicalReactionTraceSummary ?? "<none>") +
                   " | v04250=" + (_report.FootAngularImpulseAttributionTraceSummary ?? "<none>");
        }

        internal bool CaptureSupportStepEvidencePrePhysics(
            PhysicsStepContext step, int fixedSequence,
            bool selfWeightTransferApplied,
            float selfWeightFootLoadAcceleration,
            SingleLegChainReport legReport, Vector3 kneeAppliedAcceleration, string sourceMotionPhase,
            PelvisParentFrame dynamicParent, LegTurnStepEvidence legTurnEvidence)
        {
            if (!_report.RuntimeContext.Matches(step) || _report.FixedUpdateSequence != fixedSequence)
                throw new InvalidOperationException("FOOT_PRE_SOLVE_CONTEXT_NOT_CURRENT");
            // Called by SingleLeg only after V04200, V04210 and Lower native-gravity scheduling have completed
            // for this Fixed step. Solver readback is a production contract; the joined ledger remains diagnostic-only.
            bool exactAuthorizedStance = ExactAuthorizedStanceSupportLoadEligible;
            bool footGravityScheduled = _report.StanceFootNativeGravityCurrentApplied &&
                                        _report.StanceFootNativeGravityCurrentScheduledFixed == fixedSequence;
            _ankleConstraint.RecordStancePlanarDampingLoadedSolve(_traceEnabled, fixedSequence,
                exactAuthorizedStance && footGravityScheduled && selfWeightTransferApplied, _report);
            if (!_footVelocitySolver.RecordPreSolveExposure(_traceEnabled, fixedSequence,
                exactAuthorizedStance && footGravityScheduled && selfWeightTransferApplied &&
                _body != null && !_body.isKinematic && _footCollisionDetectionMode.EnabledSpeculative && _stanceSupportAngular.Enabled,
                _body, _report))
            {
                _report.Finite = false;
                _report.Detail = "V04270_PRE_SOLVE_READBACK_FAILED:" + _report.FootVelocitySolverInvalidReason;
                return false;
            }

            // Same shared step as Pelvis and leg; invalid/default parent leaves legacy loaded runs untouched.
            _turnEvents.Capture(_traceEnabled, dynamicParent, _report, _lowerBody, _body,
                _sampledFootRotation, ControlRelativeRotation, ControlRelativeW,
                ControlRelativeA, _ankleDrive, _ankleAngularTransport, legTurnEvidence, _targetAccelerationPersistenceConditioner,
                _relativeReference, _sampledSourceRelative, _sampledSourceRelativeAngularVelocityLocal, _sampledSourceRelativeAngularAccelerationLocal);

            _rockingTetherCoupling.CapturePreSolve(_traceEnabled, fixedSequence, step.Dt,
                exactAuthorizedStance && CurrentAuthorizedSupportNormalValid(),
                _stanceContactFrictionIsolation.ZeroFriction,
                _soleRequalification.CurrentGroundAverageNormalWorld,
                _body, _lowerBody, _footAnkleAnchorLocal, _lowerAnkleAnchorLocal,
                footGravityScheduled, selfWeightTransferApplied, selfWeightFootLoadAcceleration, _report);
            _rockingTetherCoupling.CaptureNormalLoadInputs(_traceEnabled, fixedSequence,
                legReport, _lowerBody, kneeAppliedAcceleration, sourceMotionPhase, _report);
            _supportEntityBoundary.CapturePreSolve(_traceEnabled, fixedSequence,
                exactAuthorizedStance && footGravityScheduled && selfWeightTransferApplied &&
                _body != null && !_body.isKinematic && _stanceContactFrictionIsolation.ZeroFriction,
                _contactShadow.Active, _contactShadow.AuthorizedCollider, _collider, _body, _report);
            return true;
        }

        private bool CurrentAuthorizedSupportNormalValid()
        {
            int authorizedId = _contactShadow != null ? _contactShadow.AuthorizedColliderId : 0;
            Vector3 normal = _soleRequalification.CurrentGroundAverageNormalWorld;
            return _report.GroundGeometryReady &&
                   _report.GroundFootprintSameSurface &&
                   authorizedId != 0 &&
                   _report.GroundColliderInstanceId == authorizedId &&
                   IsFinite(normal) && normal.sqrMagnitude > 0.00000001f;
        }

        public bool SetDriveState(bool enabled, bool physicalParent = false)
        {
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

                if (_ankleConstraint.CharacterJointArm)
                    EnsureAnkleJoint();
                else
                {
                    DestroyAnkleJoint();
                    if (RefreshPendingAnkleJointDestroy())
                    {
                        Plugin.Logger?.LogWarning("[ARL][FOOT_ANKLE_DRIVE_ACTIVATION_REJECTED] side=" + (_report.Side ?? "<none>") +
                                                  " | reason=ANKLE_JOINT_DESTROY_PENDING; retry after one frame");
                        return false;
                    }
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
                Plugin.Logger?.LogError("[ARL][FOOT_ANKLE_DRIVE_ACTIVATION_FAILED] side=" + (_report.Side ?? "<none>") + " | " + e);
                ForceSafeState();
                return false;
            }
        }

        public int Rollback()
        {
            int failures = RollbackInternal();
            _destroyed = true;
            _report.Built = false;
            _report.Stage = failures == 0 ? "ROLLED_BACK" : "ROLLBACK_WITH_ERRORS";
            return failures;
        }

        private int RollbackInternal()
        {
            int failures = ForceSafeState();
            try { _stanceContactFrictionIsolation.RestoreAndDispose(_collider, _report); }
            catch { failures++; }
            try { if (_footObject != null) Object.Destroy(_footObject); }
            catch (Exception e)
            {
                failures++;
                Plugin.Logger?.LogWarning("[ARL][FOOT_ROLLBACK_STEP_FAILED] side=" + (_report.Side ?? "<none>") + " | DESTROY_ROOT | " + e.GetType().Name + " / " + e.Message);
            }
            _footObject = null;
            _visual = null;
            _toeMarker = null;
            _ankleMarker = null;
            _body = null;
            _collider = null;
            _ankleJoint = null;
            _ankleJointDestroyPending = null;
            _report.Built = false;
            return failures;
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

        private void EnsureAnkleJoint()
        {
            if (RefreshPendingAnkleJointDestroy())
                throw new InvalidOperationException("ANKLE_JOINT_DESTROY_PENDING");
            if (_ankleJoint != null)
            {
                if (_ankleJoint.connectedBody != _lowerBody)
                    throw new InvalidOperationException("ANKLE_JOINT_CONNECTED_BODY_MISMATCH");
                ConfigureAnkleJoint();
                return;
            }
            if (_footObject == null || _body == null || _lowerBody == null)
                throw new InvalidOperationException("ANKLE_JOINT_BUILD_PREREQUISITE_MISSING");
            _ankleJoint = Il2CppComponentFactory.Add<CharacterJoint>(_footObject, "FOOT_ADD_ANKLE_CHARACTER_JOINT_V0363_BASELINE_A");
            ConfigureAnkleJoint();
            if (_ankleJoint.connectedBody != _lowerBody ||
                Vector3.Distance(_ankleJoint.anchor, _footAnkleAnchorLocal) > 0.0001f ||
                Vector3.Distance(_ankleJoint.connectedAnchor, _lowerAnkleAnchorLocal) > 0.0001f)
                throw new InvalidOperationException("ANKLE_JOINT_READBACK_VALIDATION_FAILED");
        }

        private void DestroyAnkleJoint()
        {
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
            try
            {
                Object.Destroy(oldJoint);
                _ankleJointDestroyPending = oldJoint;
            }
            finally { _ankleJoint = null; }
            CopyAnkleJointReadback();
        }

        private void ConfigureAnkleJoint()
        {
            if (_ankleJoint == null) return;
            _ankleJoint.connectedBody = _lowerBody;
            _ankleJoint.autoConfigureConnectedAnchor = false;
            _ankleJoint.anchor = _footAnkleAnchorLocal;
            _ankleJoint.connectedAnchor = _lowerAnkleAnchorLocal;
            _ankleJoint.enableCollision = false;
            _ankleJoint.enablePreprocessing = true;

            // v0.3.63 A baseline retains the exact v0.3.57-v0.3.62 CharacterJoint mapping. The B topology destroys
            // this component completely; no native angular or anchor solver authority is retained in the tether arm.
            _ankleJoint.axis = Vector3.right;
            _ankleJoint.swingAxis = Vector3.up;
            SoftJointLimit low = _ankleJoint.lowTwistLimit;
            low.limit = -FootAnkleSourceDrivenRelativePdIsolation.FlexionLimitDegrees;
            _ankleJoint.lowTwistLimit = low;
            SoftJointLimit high = _ankleJoint.highTwistLimit;
            high.limit = FootAnkleSourceDrivenRelativePdIsolation.FlexionLimitDegrees;
            _ankleJoint.highTwistLimit = high;
            SoftJointLimit swing1 = _ankleJoint.swing1Limit;
            swing1.limit = FootAnkleSourceDrivenRelativePdIsolation.AxialLimitDegrees;
            _ankleJoint.swing1Limit = swing1;
            SoftJointLimit swing2 = _ankleJoint.swing2Limit;
            swing2.limit = FootAnkleSourceDrivenRelativePdIsolation.InversionLimitDegrees;
            _ankleJoint.swing2Limit = swing2;
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
            _visual.name = "ARL_Foot_" + _target.RequestedSide + "_Visual";
            _visual.hideFlags = HideFlags.DontSave;
            _visual.layer = _layer;
            _visual.transform.SetParent(_footObject.transform, false);
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
            marker.name = name;
            marker.hideFlags = HideFlags.DontSave;
            marker.layer = _layer;
            marker.transform.SetParent(_footObject.transform, false);
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

        private void UpdateStaticState()
        {
            _report.Built = IsBuilt;
            _report.Kinematic = _body != null && _body.isKinematic;
            _report.GravityEnabled = _body != null && _body.useGravity;
            _report.ColliderEnabled = _collider != null && _collider.enabled;
            _report.ColliderFrozenOff = !_report.ColliderEnabled;
            _report.PhysicalGroundContactPossible = _contactShadow.Active && _report.ColliderEnabled;
            _report.Mass = _body != null ? _body.mass : _report.Mass;
            _report.SampledFootLinearVelocityWorld = _sampledFootLinearVelocityWorld;
            _stanceContactFrictionIsolation.CopyToReport(_collider, _report);
            // UpdateStaticState is reached from every FixedUpdate. Keep the v0.4.12/v0.4.13 observer hot paths numeric-only;
            // formatted summaries are built only by status/finalize calls.

            _stanceLowerGroundAuthority.CopyToReport(
                _report,
                _report.StanceLowerGroundAuthorityPositionErrorPlanarWorld,
                _report.StanceLowerGroundAuthorityVelocityErrorPlanarWorld,
                _report.StanceLowerGroundAuthorityRawAccelerationWorld,
                _report.StanceLowerGroundAuthorityAppliedAccelerationWorld,
                _report.StanceLowerGroundAuthorityCurrentClamped,
                _report.StanceLowerGroundAuthorityCurrentApplied);
            _stanceFootNativeGravity.CopyToReport(_report);
            _footCollisionDetectionMode.CopyToReport(_body, _report);
            _footContactOffset.CopyToReport(_report, false);

            _stanceSupportAngular.CopyToReport(_report, false);
            _footVelocitySolver.CopyToReport(_report, false);
            _rockingTetherCoupling.CopyToReport(_report, false);
            CopyAnkleJointReadback();
            bool expectedDynamic = _ankleDrive.DynamicActive;
            bool gravityContract = _body == null || !_body.useGravity || (_stanceFootNativeGravity.Enabled && _stanceFootNativeGravity.CurrentApplied);
            bool structuralContract = _body == null || (gravityContract && (expectedDynamic ? !_body.isKinematic : _body.isKinematic));
            structuralContract = structuralContract && _contactShadow.RuntimeContractValid(_collider);
            structuralContract = structuralContract && (_ankleDrive.SelectedDynamicArm
                ? (_ankleConstraint.CharacterJointArm
                    ? (_ankleJoint != null && _ankleJoint.connectedBody == _lowerBody)
                    : _ankleJoint == null)
                : _ankleJoint == null);
            bool collisionModeContract = _footCollisionDetectionMode.ValidateReadback(_body, _report);
            bool contactOffsetContract = _footContactOffset.ValidateReadback(_collider, _report, out _);
            bool solverContract = _footVelocitySolver.ValidateReadback(_body, _report);
            _report.Finite = structuralContract && collisionModeContract && contactOffsetContract && solverContract && (_supportSurface == null || _supportSurface.Finite) && _stanceSupportAngular.Finite && _stanceContactFrictionIsolation.Finite && _stanceLowerGroundAuthority.Finite && _stanceFootNativeGravity.Finite &&
                             (_body == null || (IsFinite(_body.position) && IsFinite(_body.rotation) && IsFinite(_body.inertiaTensor))) &&
                             (_collider == null || (IsFinite(_collider.center) && IsFinite(_collider.size)));
        }

        private static string F(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";

        private static Vector3 ComponentMin(Vector3 a, Vector3 b)
        {
            return new Vector3(Mathf.Min(a.x, b.x), Mathf.Min(a.y, b.y), Mathf.Min(a.z, b.z));
        }

        private static Vector3 ComponentMax(Vector3 a, Vector3 b)
        {
            return new Vector3(Mathf.Max(a.x, b.x), Mathf.Max(a.y, b.y), Mathf.Max(a.z, b.z));
        }

        private static Vector3 ComponentMaxAbs(Vector3 current, Vector3 value)
        {
            return new Vector3(
                Mathf.Max(current.x, Mathf.Abs(value.x)),
                Mathf.Max(current.y, Mathf.Abs(value.y)),
                Mathf.Max(current.z, Mathf.Abs(value.z)));
        }

        private static Vector3 QuaternionToAngularVector(Quaternion q)
        {
            if (q.w < 0f)
            {
                q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
            }
            float w = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            if (angle > Mathf.PI) angle -= 2f * Mathf.PI;
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (sinHalf <= 0.0001f || Mathf.Abs(angle) <= 0.000001f)
                return Vector3.zero;
            return new Vector3(q.x / sinHalf, q.y / sinHalf, q.z / sinHalf) * angle;
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

        private static bool IsFinite(float value) => !(float.IsNaN(value) || float.IsInfinity(value));
        private static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        private static bool IsFinite(Quaternion value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
    }
}
