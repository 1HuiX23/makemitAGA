using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// Foot/Ankle orchestration: held Render source, ordered Fixed control, authorized contact and evidence.
    /// The frozen baseline uses the no-Joint tether, measured Lower transport and bounded relative ankle PD.
    /// Current run scopes own contact/load selection; physical thresholds and controller authority stay fixed.
    /// Native creation and shutdown live in Lifecycle; contact/reference/admission extensions keep their partials.
    /// </summary>
    internal sealed partial class FootAnkleProxyFoundation
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
        private readonly FootControlEvidenceState _controlEvidence = new FootControlEvidenceState();
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
        internal string RelativeReferenceStatus => _relativeReference.GetDetails() + "\n" + _groundReference.GetDetails();
        internal bool SelectRelativeReference(bool enabled)
        {
            if (!IsBuilt || _ankleDrive.DynamicActive || _traceEnabled) return false;
            if (!enabled) { _groundReference.DisablePreserveTrace(); _loadAdmission.DisablePreserveTrace(); }
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
        internal int LastCompletedControlFixedSequence => _controlEvidence.LastCompletedControlFixedSequence;
        public bool IsBuilt => !_destroyed && _footObject != null && _body != null && _collider != null;

        public void SampleSourcePose(SourceStepContext source, int renderSequence, float sampleTime, bool advance)
        {
            if (advance)
            {
                if (!source.Valid || source.Time != sampleTime) throw new InvalidOperationException("FOOT_SOURCE_CONTEXT_INVALID");
            }
            if (!IsBuilt) return;
            if (_target.LowerLeg == null || _target.Foot == null)
            {
                if (advance) throw new InvalidOperationException("FOOT_REQUIRED_SOURCE_TARGET_MISSING");
                return;
            }

            Quaternion sampledSoleRotation = _target.Foot.rotation * _boneToSoleBindRotation;
            Quaternion lowerRotation = _target.LowerLeg.rotation;
            Vector3 footPosition = _target.Foot.position;
            if (!ValidSourceRotation(sampledSoleRotation) || !ValidSourceRotation(lowerRotation) || !IsFinite(footPosition))
            {
                if (advance) throw new InvalidOperationException("FOOT_REQUIRED_SOURCE_POSE_INVALID");
                return;
            }
            Quaternion relative = Quaternion.Inverse(lowerRotation) * sampledSoleRotation;
            if (!ValidSourceRotation(relative))
            {
                if (advance) throw new InvalidOperationException("FOOT_REQUIRED_RELATIVE_SOURCE_POSE_INVALID");
                return;
            }
            // Publish the new identity only after its required payload is valid. A bad
            // source frame must not relabel the previous held Foot pose as a fresh sample.
            if (advance) _report.RuntimeSourceContext = source;
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
                _controlEvidence.ObserveSource(_report.SourceAnkleRenderStepDegrees);
                _controlEvidence.Publish(_report);
            }

            Quaternion sourceDeltaFromBind = relative * Quaternion.Inverse(_bindSourceRelative);
            Vector3 deltaDegrees = QuaternionToAngularVector(sourceDeltaFromBind) * Mathf.Rad2Deg;

            _report.TargetRenderSampleSequence = renderSequence;
            _report.SourceAnkleRelativeRotation = relative;
            _report.SourceAnkleFromBindRotationVectorDegrees = deltaDegrees;
            _report.SourceAnkleFromBindAngleDegrees = deltaDegrees.magnitude;
            _sampledSourceRelativeAngularVelocityLocal = angularVelocity;
            _sampledSourceRelativeAngularAccelerationLocal = angularAcceleration;
            PrepareGroundReferenceInput(source, relative, angularVelocity, sourceDt, out Quaternion referenceGoal, out Vector3 referenceGoalW);
            _relativeReference.ObserveRender(_traceEnabled, source, referenceGoal, referenceGoalW, sourceDt);
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
            float targetSampleAgeSeconds = Mathf.Max(0f, sampleAge);
            _report.TargetRenderSampleAgeSeconds = targetSampleAgeSeconds;

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
            bool referenceContract = (!_groundReference.Enabled || _groundReference.Finite) && (!_relativeReference.Selected || (_relativeReference.Finite &&
                (expectedDynamic ? (_relativeReference.Active && _relativeReference.Current.Source.Matches(_report.RuntimeSourceContext))
                    : !_relativeReference.Active)));
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
                _report.Detail = "FOOT_REFERENCE_CONTRACT_FAILED:" +
                    ((_groundReference.Enabled && !_groundReference.Finite ? _groundReference.Issue : _relativeReference.Issue) ?? "SOURCE_IDENTITY_OR_ACTIVE_STATE");
            _report.AnkleRelativeTrackingErrorDegrees = finite ? relativeError : float.PositiveInfinity;
            _report.AnkleAnchorErrorMeters = finite ? anchorError : float.PositiveInfinity;

            // This readback already exists before contact enable/release and this step's tether/PD writes.
            // Keep a predecessor plus the current sample, including any finite sample that aborts below.
            Vector3 lowerPointVelocity = _lowerBody.GetPointVelocity(lowerAnchorWorld);
            Vector3 footPointVelocity = _body.GetPointVelocity(footAnchorWorld);
            _controlEvidence.BeginObservation();
            _controlEvidence.Publish(_report);
            _controlEvidence.Observe(_traceEnabled, new FootFixedStepEvidence
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
            });
            _controlEvidence.Publish(_report);

            // Settle the prior diagnostic command before lifecycle/current controllers, including a finite gate endpoint.
            _turnEvents.ObserveSettled(_traceEnabled, step, _report.CurrentObservedStep, _lowerBody, _body,
                _report.RuntimeSourceContext, ControlRelativeRotation, sampleAge, legTurnEvidence);
            _report.PublishTurnResponse(_turnEvents.ResponseReceipt);

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
                FootStanceOnlyAuthorizedContactLifecycle.EntryContext entryContext = default;
                if (_stanceContactLifecycle.ContinuousEntry && !_contactShadow.Active)
                {
                    // Both rotations use this physical Lower read and the held Render packet above.
                    // Reference substitutions are diagnostic only; no live source Transform is sampled here.
                    Quaternion entryLowerRotation = _lowerBody.rotation;
                    entryContext = FootStanceOnlyAuthorizedContactLifecycle.CaptureEntryContext(fixedSequence, _body,
                        lowerAnchorWorld, _footAnkleAnchorLocal, entryLowerRotation * ControlRelativeRotation,
                        _sampledFootPosition, _sampledFootRotation, anchorError, sampleAge, _sampledSourceRelative, entryLowerRotation);
                    entryContext.LandingGoal = _groundReference.Current;
                    // Reuse this Fixed's existing source receipt/classifier. These fields only select/describe a log window.
                    entryContext.SourcePhase = legTurnEvidence.Matches(step, fixedSequence) &&
                        legTurnEvidence.Source.Matches(_report.RuntimeSourceContext) ? legTurnEvidence.SourcePhase : null;
                    entryContext.SourceFresh = fresh;
                    entryContext.SourceHeldDepth = heldDepth;
                    entryContext.SourceDt = _report.RuntimeSourceContext.Dt;
                    if (LandingReferenceCaptureRequested)
                        entryContext.LandingPath = CaptureLandingReference(step, fixedSequence, legTurnEvidence, entryLowerRotation);
                }
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
                    _report,
                    _supportSurface,
                    entryContext))
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
                if (!UpdateStanceLoadAdmission(step, fixedSequence, anchorError, sampleAge)) return false;

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
                _controlEvidence.EvaluateConstraint(fixedSequence);
                _controlEvidence.Publish(_report);
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

                _controlEvidence.EvaluateDrive(fixedSequence);
                _controlEvidence.Publish(_report);
                if (!_ankleDrive.Step(
                    _traceEnabled,
                    _body,
                    _ankleJoint,
                    _lowerBody,
                    _footAnkleAnchorLocal,
                    _lowerAnkleAnchorLocal,
                    _bindSourceRelative,
                    ControlRelativeRotation,
                    ControlRelativeW,
                    ControlRelativeA,
                    ControlRelativeFF,
                    targetSampleAgeSeconds,
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
                    ExactAuthorizedStanceSupportLoadEligible,
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

                _controlEvidence.CompleteController(fixedSequence);
                _controlEvidence.Publish(_report);

            }

            if (_traceEnabled && finite)
            {
                _controlEvidence.CompleteObservation(relativeError, anchorError,
                    _report.SourceAnkleRelativeAngularSpeed, _report.SourceAnkleFromBindAngleDegrees,
                    _report.SourceAnkleFromBindRotationVectorDegrees, _report.SourceAnkleRelativeAngularVelocityLocal);
                _controlEvidence.Publish(_report);
            }

            return finite;
        }

        public void BeginTrace(PelvisRunScope scope = PelvisRunScope.DynamicLegFreeSpace)
        {
            _traceEnabled = true;
            _turnEvents.Reset(scope);
            _relativeReference.ResetTrace();
            _groundReference.ResetTrace();
            _loadAdmission.ResetTrace();
            _report.StanceLoadAdmissionSummary = null;
            _report.TurnEventDetails = null;
            _footContactOffset.ResetTrace(_report);

            // Trace is diagnostic-only. Do NOT reset source pose/W/A or foot-velocity derivative history here:
            // those states feed the physical controller/parent-transport paths, so arl_trace(on) must not create
            // an artificial one/two-sample warm-up. Arm/drive/rollback transitions own physical-history resets.
            _controlEvidence.Reset();
            _controlEvidence.Publish(_report);
            _report.PublishTurnResponse(_turnEvents.ResponseReceipt);
            _soleRequalification.ResetTrace(_report);
            _contactIdentity.ResetTrace(_report);
            _contactShadow.ResetTrace(_report);
            _mechanicalReaction.ResetTrace(_report);

            _footAngularImpulseAttribution.ResetTrace(_report);
            _rockingTetherCoupling.ResetTrace(_report, scope);
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
            _report.TurnEventDetails = _turnEvents.GetDetails() + "\n" + _relativeReference.GetDetails() + "\n" + _groundReference.GetDetails();

            _report.StanceLoadAdmissionSummary = _loadAdmission.Summary();
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
            // Publish the owner state before trace consumers read the cached v058Now projection.
            RefreshAnkleDriveReport();
            _report.PublishTurnResponse(_turnEvents.ResponseReceipt);
            _controlEvidence.Seal(_report.RuntimeContext, _report.RuntimeSourceContext, _turnEvents.ResponseReceipt,
                _ankleDrive.DynamicActive, _report.Kinematic, _report.GravityEnabled, _report.ColliderEnabled,
                _report.AnkleDriveCurrentSummary);
            _controlEvidence.Publish(_report);
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
            // Active control owns the sole/identity witness in ObserveFixed. A status query
            // must not resample geometry or reset identity continuity between that command
            // and the next solve; read the last completed physical-step result instead.
            if (!IsBuilt || _ankleDrive.DynamicActive) return;
            FollowSourceKinematic();
            RefreshSoleRequalificationWitness(_sampledFootLinearVelocityWorld);
        }

        private bool RefreshSoleRequalificationWitness(Vector3 groundEligibilityVelocity)
        {
            bool valid = _soleRequalification.Observe(
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
                groundEligibilityVelocity,
                _report);
            _contactIdentity.Observe(false, "STATUS", groundEligibilityVelocity, Time.fixedDeltaTime, _report);
            return valid && _report.SoleRequalificationFinite && _report.ContactIdentityFinite;
        }

        public string GetContactIdentityStatus()
        {
            RefreshSoleRequalificationCurrent();
            _contactIdentity.CopyToReport(_report);
            return _contactIdentity.GetStatus(_report);
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
        public bool AnkleConstraintProductionArm => _ankleConstraint.IsProductionSelectorArm;
        public bool AnkleConstraintCharacterJointArm => _ankleConstraint.CharacterJointArm;
        public string AnkleTransportArm => _ankleTransport.Arm;
        public bool AnkleTransportFeedforwardEnabled => _ankleTransport.Enabled;
        public string AnkleAngularTransportPolicy => _ankleAngularTransport.Policy;
        public bool AnkleAngularTransportFeedforwardEnabled => _ankleAngularTransport.Enabled;
        public bool AnkleAngularTransportProductionReady => _ankleAngularTransport.Enabled &&
                                                     _body != null &&
                                                     Mathf.Abs(_body.angularDrag - _profile.AngularDrag) <= 0.0001f;
        internal Rigidbody DynamicBody => _body;
        internal FreeSpaceBoundaryIssue FreeSpaceSelectorIssue
        {
            get
            {
                if (!IsBuilt) return FreeSpaceBoundaryIssue.MissingFoundation;
                if (_loadAdmission.Selected || _groundReference.Enabled || _stanceContactLifecycle.Enabled || _stanceContactLifecycle.ContinuousEntry || _stanceContactLifecycle.EntryRecovery || !_stanceContactLifecycle.Finite ||
                    _contactShadow.Arm != FootQualifiedContactShadow.ArmA || _contactShadow.Active || !_contactShadow.Finite)
                    return FreeSpaceBoundaryIssue.ContactAuthorization;
                if (_stanceContactFrictionIsolation.Arm != FootStanceContactFrictionIsolation.ArmA || !_stanceContactFrictionIsolation.Finite)
                    return FreeSpaceBoundaryIssue.Friction;
                if (_stanceLowerGroundAuthority.Arm != FootStanceLowerGroundAnchorAuthority.ArmA || !_stanceLowerGroundAuthority.Finite)
                    return FreeSpaceBoundaryIssue.GroundAuthority;
                // V04171 B can alter the tether even with contact OFF. No-load scope must explicitly retain A.
                if (_ankleConstraint.PredictiveTailCaptureArm != FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA ||
                    !_ankleConstraint.PredictiveTailCaptureFinite) return FreeSpaceBoundaryIssue.PredictiveCapture;
                if (_stanceFootNativeGravity.Arm != FootStanceNativeGravitySupportLoadIsolation.ArmA || !_stanceFootNativeGravity.Finite)
                    return FreeSpaceBoundaryIssue.FootGravity;
                if (_footCollisionDetectionMode.Arm != FootCollisionDetectionModeIsolation.IdleArm || !_footCollisionDetectionMode.Finite)
                    return FreeSpaceBoundaryIssue.CollisionMode;
                if (_report.FootContactOffsetArm != FootContactOffsetIsolation.IdleArm || !_report.FootContactOffsetInitialized ||
                    !_report.FootContactOffsetFinite || !_report.FootContactOffsetReadbackValid || !_report.FootContactOffsetBaselineRestored)
                    return FreeSpaceBoundaryIssue.ContactOffset;
                if (!_footVelocitySolver.Finite || !_footVelocitySolver.BaselineRestored ||
                    _footVelocitySolver.PositionArm != FootSolverIterationBaselineContract.PositionArmA)
                    return FreeSpaceBoundaryIssue.Solver;
                if (_stanceSupportAngular.Arm != FootStanceSupportNormalAngularStabilization.ArmA || !_stanceSupportAngular.Finite)
                    return FreeSpaceBoundaryIssue.SupportAngular;
                if (_ankleConstraint.StancePlanarDampingEnabled) return FreeSpaceBoundaryIssue.PlanarDamping;
                return FreeSpaceBoundaryIssue.None;
            }
        }
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

        // Explicit preparation, used before pinning the original floor AND after binding the
        // owned Box. Legs may already have joined their dynamic parent, but no traced/contact
        // solve may own these witnesses. This must not go through the read-only status path.
        internal void RefreshSupportProbe()
        {
            if (!IsBuilt) return;
            if (_traceEnabled || _body.useGravity || _collider.enabled || _contactShadow.Active)
                throw new InvalidOperationException("SUPPORT_PROBE_REQUIRES_TRACE_GRAVITY_AND_CONTACT_OFF");
            if (!_ankleDrive.DynamicActive) FollowSourceKinematic();
            if (!RefreshSoleRequalificationWitness(_ankleDrive.DynamicActive ? _body.velocity : _sampledFootLinearVelocityWorld))
                throw new InvalidOperationException("SUPPORT_PROBE_CURRENT_WITNESS_INVALID");
        }
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
        internal bool ContactShadowFinite => _contactShadow.Finite;
        internal bool StanceContactLifecycleEnabled => _stanceContactLifecycle.Enabled;
        internal bool StanceContactLifecycleFinite => _stanceContactLifecycle.Finite;
        internal string StanceContactFrictionIsolationArm => _stanceContactFrictionIsolation.Arm;
        internal bool StanceContactFrictionIsolationFinite => _stanceContactFrictionIsolation.Finite;
        internal bool StanceContactZeroFriction => _stanceContactFrictionIsolation.ZeroFriction;
        // v0.4.19: narrow read-only bridge for the SingleLeg-owned Lower gravity selector.
        // It reuses the exact same current Foot contact/support-normal witness as V04170; no new raycast or authority.
        internal bool ExactAuthorizedStanceSupportLoadEligible => BaseAuthorizedStanceLoadEligible &&
            (!_loadAdmission.Selected || (_loadAdmission.Matches(_report.RuntimeContext, _report.FixedUpdateSequence) && _loadAdmission.Eligible));
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

        internal bool ContactEntryContinuous => _stanceContactLifecycle.ContinuousEntry;
        internal bool ContactEntryRecovery => _stanceContactLifecycle.EntryRecovery;
        internal void SelectContactEntryContinuous(bool enabled, bool recovery = false, bool groundReference = false, bool partialGround = false, bool loadAdmission = false)
        {
            if (_traceEnabled || _collider == null || _collider.enabled || _contactShadow.Active)
                throw new InvalidOperationException("CONTACT_ENTRY_SELECTION_REQUIRES_QUIESCED_TRACE_OFF");
            if (loadAdmission && !enabled) throw new InvalidOperationException("LOAD_ADMISSION_REQUIRES_CONTINUOUS_CONTACT");
            _stanceContactLifecycle.SelectContinuousEntry(enabled, _report, recovery);
            _loadAdmission.Select(loadAdmission);
            _groundReference.Select(enabled, groundReference, partialGround);
            if (enabled) _groundReference.SeedInput(_report.RuntimeSourceContext, _sampledSourceRelative, _sampledSourceRelativeAngularVelocityLocal);
        }

        internal void SetStanceContactLifecycleEnabled(bool enabled)
        {
            if (!enabled) { _groundReference.DisablePreserveTrace(); _loadAdmission.DisablePreserveTrace(); }
            if (!enabled)
                _stanceFootNativeGravity.QuiescePreserveTrace(_body, _report, out _);
            _stanceContactLifecycle.SetEnabled(enabled, _contactShadow, _collider, _report);
            UpdateStaticState();
        }

        internal void DisableStanceContactLifecycle()
        {
            _groundReference.DisablePreserveTrace();
            _loadAdmission.DisablePreserveTrace();
            _stanceFootNativeGravity.QuiescePreserveTrace(_body, _report, out _);
            _stanceContactLifecycle.DisableAndReset(_contactShadow, _collider, _report);
            UpdateStaticState();
        }

        internal bool QuiesceStanceContactLifecyclePreserveTrace(out string error)
        {
            _groundReference.DisablePreserveTrace();
            _loadAdmission.DisablePreserveTrace();
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

        internal bool CaptureSupportStepEvidencePrePhysics(
            PhysicsStepContext step, int fixedSequence,
            bool selfWeightTransferApplied,
            float selfWeightFootLoadAcceleration,
            SingleLegChainReport legReport, Vector3 kneeAppliedAcceleration, string sourceMotionPhase,
            PelvisParentFrame dynamicParent, LegTurnStepEvidence legTurnEvidence, PelvisRunScope scope = PelvisRunScope.TorsoPairOnly)
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
            _report.PublishTurnResponse(_turnEvents.ResponseReceipt);

            _rockingTetherCoupling.CapturePreSolve(_traceEnabled, fixedSequence, step.Dt,
                exactAuthorizedStance && CurrentAuthorizedSupportNormalValid(),
                _stanceContactFrictionIsolation.ZeroFriction,
                _soleRequalification.CurrentGroundAverageNormalWorld,
                _body, _lowerBody, _footAnkleAnchorLocal, _lowerAnkleAnchorLocal,
                footGravityScheduled, selfWeightTransferApplied, selfWeightFootLoadAcceleration, _report);
            _rockingTetherCoupling.CaptureNormalLoadInputs(_traceEnabled, fixedSequence,
                legReport, _lowerBody, kneeAppliedAcceleration, sourceMotionPhase, _report, scope, step, dynamicParent);
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

        private static bool IsFinite(float value) => !(float.IsNaN(value) || float.IsInfinity(value));
        private static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        private static bool IsFinite(Quaternion value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        private static bool ValidSourceRotation(Quaternion value)
        {
            float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w;
            return IsFinite(value) && IsFinite(lengthSquared) && lengthSquared > 0.00000001f;
        }
    }
}
