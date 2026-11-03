using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// Frozen per-side leg controller. v0.2.122 certified this exact production path as the Single-Leg baseline.
    /// v0.3.x intentionally reuses two instances of this same class for LEFT/RIGHT bilateral foundation work;
    /// there is no side-specific controller fork or new tuning here. Production remains exact v0.2.105 filtered/held
    /// target-W at alpha=0.50, with the existing v0.2.122 safety/freeze certification retained per side.
    /// </summary>
    internal sealed partial class SingleLegShadowChain
    {
        private const string CurrentAngularKinematicsMode = "CURRENT_COMPACT_ANGULAR_KINEMATICS_V0317";
        private readonly ActiveRagdollLabProfile _profile;
        private readonly SingleLegTargetReport _target;
        private readonly TwoBodyShadowProxy _torso;
        private readonly Rigidbody _pelvisBody;
        private readonly Collider[] _torsoOwnedColliders;
        private readonly Transform _targetRoot;
        private readonly Transform _proxyRoot;
        private readonly OwnedCollisionIgnorePairs _ignoredPairs = new OwnedCollisionIgnorePairs();
        // Construction owns each object before any native property/parenting can throw.
        // Successful parenting transfers lifetime to _root; only unfinished transfers remain here.
        private readonly List<GameObject> _unparentedObjects = new List<GameObject>();
        private readonly SingleLegChainReport _report = new SingleLegChainReport();
        private readonly KneeFlexionAxisCalibrator _kneeAxisCalibrator = new KneeFlexionAxisCalibrator();
        private readonly SingleLegAngularAccelerationFeedforward _legAngularAccelerationFeedforward = new SingleLegAngularAccelerationFeedforward();
        private readonly SingleLegRigidbodyMassProperties _rigidbodyMassProperties = new SingleLegRigidbodyMassProperties();
        private readonly SingleLegFreezeReadinessCertification _singleLegFreezeCertification = new SingleLegFreezeReadinessCertification();
        private readonly KneeParentAngularTransportFeedforwardIsolation _kneeParentAngularTransport = new KneeParentAngularTransportFeedforwardIsolation();
        private readonly KneeCharacterJointResponseIsolation _kneeCharacterJointResponseIsolation = new KneeCharacterJointResponseIsolation();
        private readonly KneePositionalAnchorAccelerationFeedforwardIsolation _kneeAnchorAccelerationFeedforward = new KneePositionalAnchorAccelerationFeedforwardIsolation();
        private readonly KneeFlexionFeedbackHeadroomAllocationIsolation _kneeFlexionAllocation = new KneeFlexionFeedbackHeadroomAllocationIsolation();
        private readonly PhaseAwareSoftKneeAuthority _softKneeAuthority = new PhaseAwareSoftKneeAuthority();
        private readonly PhaseAwareHipAuthority _hipAuthority = new PhaseAwareHipAuthority();
        private readonly HipControlFrameInertializer _hipControlFrame = new HipControlFrameInertializer();
        private readonly HipMomentumAwareRecoveryBrakingIsolation _hipMomentumRecoveryBraking = new HipMomentumAwareRecoveryBrakingIsolation();
        private readonly HipVectorCaptureDampingIsolation _hipVectorCaptureDamping = new HipVectorCaptureDampingIsolation();
        private readonly HipFeasibilityReferenceGovernor _hipReferenceGovernor = new HipFeasibilityReferenceGovernor();
        private readonly HipSourceReferenceShockAbsorberIsolation _hipSourceReferenceShockAbsorber = new HipSourceReferenceShockAbsorberIsolation();
        private readonly HipCompositeReferenceShockAbsorberIsolation _hipCompositeReferenceShockAbsorber = new HipCompositeReferenceShockAbsorberIsolation();
        private readonly HipCharacterJointResponseIsolation _hipCharacterJointResponseIsolation = new HipCharacterJointResponseIsolation();
        private readonly HipPositionalParentAnchorAccelerationFeedforwardIsolation _hipAnchorAccelerationFeedforward = new HipPositionalParentAnchorAccelerationFeedforwardIsolation();
        private readonly HipAnchorTranslationPair _hipAnchorPair = new HipAnchorTranslationPair();
        internal HipAnchorPairReport HipAnchorPair => _hipAnchorPair.Report;
        internal string HipPointResponseDetails => _hipAnchorPair.Response.GetDetails(_torso == null ? null : _torso.PelvisReceiver);
        internal bool CancelQueuedHipPair(PhysicsStepContext step)
        { _hipContinuationPending = false; return _hipAnchorPair.CancelQueued(step); }
        internal bool CommitBilateralHipPairs(PhysicsStepContext step, SingleLegShadowChain peer) =>
            peer != null && HipAnchorTranslationPair.CommitBilateral(step, _hipAnchorPair, peer._hipAnchorPair);
        internal void CompleteHipPointResponse(PhysicsStepContext step, HipAnchorPairReport peer)
        {
            if (!_traceEnabled) return;
            _upperSelfWeightTransfer.CompleteHipLedger(_upperBody, _fixedUpdateSequence,
                _footFoundation.ExactAuthorizedStanceSupportLoadEligible, PhysicsDt, _report, HipLedgerAcceleration);
            _hipAnchorPair.Response.CaptureGoalPath(_legTurnCurrent.HipGoalPath, _legTurnCurrent.HipNormalReference);
            _hipAnchorPair.Response.CompleteInputs(step, _fixedUpdateSequence, peer,
                _torso == null ? new PelvisHipAttitudeCommand() : _torso.HipAttitudeCommand);
            _hipAnchorPair.Response.CompleteAttitudeBudget(step, _fixedUpdateSequence, _torso == null ? null : _torso.PelvisReceiver);
        }
        private Vector3 HipLedgerAcceleration => _dynamicParentActive && PelvisRunScopes.HasHipCoupling(_dynamicScope)
            ? (_hipAnchorPair.Report.CurrentApplied ? _hipAnchorPair.Report.Current.Force / _hipAnchorPair.Report.Current.ChildMass : Vector3.zero)
            : _hipCharacterJointResponseIsolation.CurrentAppliedAccelerationWorld;
        private readonly HipAngularActuatorInertiaSpaceResponseIsolation _hipAngularActuator = new HipAngularActuatorInertiaSpaceResponseIsolation();
        private readonly LegAngularDragContract _legAngularDragContract = new LegAngularDragContract();
        private readonly HipMechanicalParentProxy _hipMechanicalParent = new HipMechanicalParentProxy();
        private readonly HipMechanicalParentTransportFeedforward _hipMechanicalParentTransport = new HipMechanicalParentTransportFeedforward();
        private readonly HipAnchorCommonModeLinearTransport _hipAnchorLinearTransport = new HipAnchorCommonModeLinearTransport();
        private readonly LowerStanceNativeGravitySupportLoadIsolation _stanceLowerNativeGravity = new LowerStanceNativeGravitySupportLoadIsolation();
        private readonly FootLowerSupportNormalReciprocalLoadTransferIsolation _stanceFootLowerReciprocal = new FootLowerSupportNormalReciprocalLoadTransferIsolation();
        private readonly LowerSelfWeightSupportNormalLoadTransferIsolation _stanceLowerSelfWeightTransfer = new LowerSelfWeightSupportNormalLoadTransferIsolation();
        private readonly KneeSupportNormalReactionIsolation _kneeSupportReaction = new KneeSupportNormalReactionIsolation();
        private readonly UpperSelfWeightSupportNormalLoadTransferIsolation _upperSelfWeightTransfer = new UpperSelfWeightSupportNormalLoadTransferIsolation();
        private FootAnkleProxyFoundation _footFoundation;

        private GameObject _root;
        private GameObject _upperObject;
        private GameObject _lowerObject;
        private GameObject _upperVisual;
        private GameObject _lowerVisual;
        private Rigidbody _upperBody;
        private Rigidbody _lowerBody;
        private BoxCollider _upperCollider;
        private BoxCollider _lowerCollider;
        private CharacterJoint _hipJoint;
        private CharacterJoint _kneeJoint;
        private Vector3 _kneeFlexionAxisLowerLocal;
        private Vector3 _kneeChildAnchorLowerLocal;
        private Vector3 _kneeParentAnchorUpperLocal;
        private Material _previewMaterial;

        private bool _driveEnabled;
        private bool _gravityEnabled;
        private bool _collidersEnabled;
        private bool _visualEnabled = true;
        private bool _destroyed;
        private bool _traceEnabled;
        private float _nextTraceTime;
        private Vector3 _initialKneeFlexionAxisLowerLocal = Vector3.right;
        private string _kneeAxisMode = "INITIAL_CHARACTER_RIGHT";
        private Vector3 _rawHipConnectedAnchorLocal;
        private Vector3 _lastHipAppliedAngularAccelerationWorld;
        private readonly HipNormalReference _hipNormalReference = new HipNormalReference();

        // Actual relative Knee command remains a live Foot attribution input.
        // The v0.3.34 frame/response observer closed in v0.3.75; its unused counterfactual storage is retired.
        private Vector3 _kneeResponseRelativeCommandWorld;

        private Vector3 _sampledUpperPosition;
        private Quaternion _sampledUpperRotation = Quaternion.identity;
        private Vector3 _sampledLowerPosition;
        private Quaternion _sampledLowerRotation = Quaternion.identity;
        private Quaternion _sampledPelvisRotation = Quaternion.identity;
        private Quaternion _sampledTargetHipRelative = Quaternion.identity;
        private Quaternion _sampledTargetKneeRelative = Quaternion.identity;
        // v0.2.71 pre-governor physical-control Source target. A mirrors raw sampled target exactly; B is the
        // feasibility-limited Source accumulator. Raw sampled target remains authoritative for health/diagnostics.
        private Quaternion _sourceControlTargetHipRelative = Quaternion.identity;
        private Vector3 _sourceControlTargetHipRelativeAngularVelocityLocal;
        // Active P targets. Baseline Normal mirrors Render; V04780 B overrides ordinary Hip from one Fixed world trajectory.
        // v0.2.44 replaces Hip while the
        // physical-state-aware feasibility governor is active across Source Turn and physical recovery.
        private Quaternion _controlTargetHipRelative = Quaternion.identity;
        private Quaternion _controlTargetKneeRelative = Quaternion.identity;
        private Vector3 _sampledTargetHipRelativeAngularVelocityLocal;
        private Vector3 _sampledTargetKneeRelativeAngularVelocityLocal;
        // Active D targets. Healthy Normal Knee mirrors TargetVelocityBlend-filtered Render sample-held W; v0.2.44 derives Hip D target from the governed
        // Fixed-step pose trajectory so P/W remain dynamically consistent throughout Source/recovery handoff.
        private Vector3 _controlTargetHipRelativeAngularVelocityLocal;
        private Vector3 _controlTargetKneeRelativeAngularVelocityLocal;
        // Render-only relative target-A for the original FF path. V04780 coherent Normal Hip instead uses its private world-plan A.
        private Vector3 _sampledTargetHipRelativeAngularAccelerationLocal;
        private Vector3 _sampledTargetKneeRelativeAngularAccelerationLocal;
        private Vector3 _previousFilteredTargetHipRelativeAngularVelocityLocal;
        private Vector3 _previousFilteredTargetKneeRelativeAngularVelocityLocal;
        private bool _hasPreviousFilteredTargetVelocitySample;

        // Retained timing diagnostics: explicit Render->FixedUpdate sample age and settled-body measurement sequence.
        private int _targetRenderSampleSequence;
        private float _targetRenderSampleUnscaledTime;
        private readonly TargetRenderSampleTiming _targetSampleTiming = new TargetRenderSampleTiming();
        private int _fixedUpdateSequence;
        private SourceStepContext _runtimeSource;
        private PhysicsStepContext _runtimeStep;
        private LegParentCommandFrame _parentCommand;
        private bool _insideSourceStep, _insidePhysicsStep;
        private float PhysicsDt => _runtimeStep.Dt;
        private float CurrentUnscaledTime => _insidePhysicsStep ? _runtimeStep.UnscaledTime :
            _insideSourceStep ? _runtimeSource.Time : Time.unscaledTime;

        // v0.2.1 DIAGNOSTIC ONLY: render-clock source motion for parent-transport attribution.
        private Transform _sourcePelvisTransform;
        private Vector3 _sampledSourcePelvisPosition;
        private Quaternion _sampledSourcePelvisRotation = Quaternion.identity;
        private Vector3 _sampledSourcePelvisLinearVelocity;
        private Vector3 _sampledSourcePelvisAngularVelocity;
        private Vector3 _sampledUpperLinearVelocity;
        private Vector3 _sampledUpperAngularVelocity;
        private Vector3 _sampledLowerLinearVelocity;
        private Vector3 _sampledLowerAngularVelocity;
        private Vector3 _previousSourcePelvisPosition;
        private Quaternion _previousSourcePelvisRotation = Quaternion.identity;
        private Vector3 _previousSourceUpperPosition;
        private Quaternion _previousSourceUpperRotation = Quaternion.identity;
        private Vector3 _previousSourceLowerPosition;
        private Quaternion _previousSourceLowerRotation = Quaternion.identity;
        private bool _hasPreviousSourceMotionSample;
        // v0.2.2 source-only calibration keeps its own raw pelvis angular-acceleration trigger so calibration
        // can reject MiSide rapid-turn windows even when arl_trace is intentionally OFF.
        private Vector3 _previousCalibrationPelvisAngularVelocityRaw;
        private bool _hasPreviousCalibrationPelvisAngularVelocityRaw;

        // v0.2.1 DIAGNOSTIC ONLY: settled FixedUpdate body motion, sampled before applying the next PD torque.
        private Vector3 _measuredPelvisLinearVelocity;
        private Vector3 _measuredPelvisAngularVelocity;
        private Vector3 _measuredUpperLinearVelocity;
        private Vector3 _measuredUpperAngularVelocity;
        private Vector3 _measuredLowerLinearVelocity;
        private Vector3 _measuredLowerAngularVelocity;
        private Vector3 _previousMeasuredPelvisPosition;
        private Quaternion _previousMeasuredPelvisRotation = Quaternion.identity;
        private Vector3 _previousMeasuredUpperPosition;
        private Quaternion _previousMeasuredUpperRotation = Quaternion.identity;
        private Vector3 _previousMeasuredLowerPosition;
        private Quaternion _previousMeasuredLowerRotation = Quaternion.identity;
        private bool _hasPreviousMeasuredMotionSample;
        private bool _measuredTransportValidCurrent;
        private Quaternion _previousTargetHipRelative = Quaternion.identity;
        private Quaternion _previousTargetKneeRelative = Quaternion.identity;
        private bool _hasPreviousTargetSample;

        // 与冻结 torso 一样，body relative angular velocity 从 FixedUpdate settled relative pose delta 得到，
        // 不依赖 kinematic Rigidbody.angularVelocity 的平台语义。
        private Quaternion _previousBodyHipRelative = Quaternion.identity;
        private Quaternion _previousBodyKneeRelative = Quaternion.identity;
        private Vector3 _fixedBodyHipRelativeAngularVelocityLocal;
        private Vector3 _fixedBodyKneeRelativeAngularVelocityLocal;
        private bool _hasPreviousBodyRelativeSample;

        private float _traceHipErrorSum;
        private float _traceKneeErrorSum;
        private float _traceHipErrorMax;
        private float _traceKneeErrorMax;
        private int _traceSamples;
        private int _traceHipClampSamples;
        private int _traceKneeClampSamples;
        private float _traceKneeOffAxisErrorSum;
        private float _traceKneeOffAxisErrorMax;
        private float _traceKneeSoftAppliedSum;
        private float _traceKneeSoftAppliedMax;
        private int _traceKneeSoftClampSamples;

        public SingleLegShadowChain(
            ActiveRagdollLabProfile profile,
            SingleLegTargetReport target,
            Transform targetRoot,
            TwoBodyShadowProxy torso)
        {
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            _target = target ?? throw new ArgumentNullException(nameof(target));
            _targetRoot = targetRoot;
            _torso = torso ?? throw new ArgumentNullException(nameof(torso));
            _report.Side = target.RequestedSide;
            _report.KneeSupportReaction = _kneeSupportReaction.Report;
            _report.UpperSelfWeightSupport = _upperSelfWeightTransfer.Report;
            _proxyRoot = torso.ProxyRoot;
            _pelvisBody = torso.PelvisBody;
            _torsoOwnedColliders = torso.OwnedColliders ?? Array.Empty<Collider>();
            _sourcePelvisTransform = target.UpperLeg != null ? target.UpperLeg.parent : null;
            _initialKneeFlexionAxisLowerLocal = target.KneeFlexionAxisLowerLocal.sqrMagnitude > 0.0001f
                ? target.KneeFlexionAxisLowerLocal.normalized
                : Vector3.right;
            _kneeFlexionAxisLowerLocal = _initialKneeFlexionAxisLowerLocal;
            _report.AttributionMode = "SINGLE_LEG_PHASE_CLASSIFIER_RETAINED_V0317";
            _report.AttributionPhase = "UNSAMPLED";
            _kneeAxisCalibrator.Reset(_initialKneeFlexionAxisLowerLocal);
            _legAngularAccelerationFeedforward.SetEnabled(_profile.LegAngularAccelerationFeedforward);
        }

        public SingleLegChainReport Report => _report;
        public bool IsBuilt => !_destroyed && _root != null && _pelvisBody != null &&
                               _upperBody != null && _lowerBody != null && _upperCollider != null && _lowerCollider != null &&
                               (_hipJoint != null || _hipCharacterJointResponseIsolation.Enabled) &&
                               (_kneeJoint != null || _kneeCharacterJointResponseIsolation.Enabled) && _hipMechanicalParent.Built;
        public bool DriveEnabled => _driveEnabled;
        public bool LastRollbackSucceeded { get; private set; } = true;
        internal int LastSafeStateFailures { get; private set; }
        public bool TraceEnabled => _traceEnabled;
        public bool KneeAxisCalibrationCollecting => _kneeAxisCalibrator.Collecting;
        public bool KneeAxisCalibrationReady => _kneeAxisCalibrator.Ready;
        public bool KneeAxisCalibrationApplied => _kneeAxisCalibrator.Applied;
        public bool FrozenProductionStackReady => IsCompletePostPcaStackReady();
        public FootAnkleFoundationReport FootReport => _footFoundation != null ? _footFoundation.Report : null;
        public string TranslationCaptureAuthorityArm => _footFoundation != null ? _footFoundation.TranslationCaptureAuthorityArm : FootAnkleTranslationCaptureAuthorityIsolation.Production;
        public string TranslationPredictiveTailCaptureArm => _footFoundation != null ? _footFoundation.TranslationPredictiveTailCaptureArm : FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA;
        public bool TranslationPredictiveTailCaptureEnabled => _footFoundation != null && _footFoundation.TranslationPredictiveTailCaptureEnabled;
        public bool TranslationPredictiveTailCaptureFinite => _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.TranslationPredictiveTailCaptureFinite;

        // v0.3.8 minimal READ-ONLY bilateral tap. Retired Hip-reaction fields are no longer exported;
        // the unresolved Knee tether safety context remains allocation-free and control-neutral.
        internal SingleLegBilateralReadOnlySnapshot GetBilateralReadOnlySnapshot()
        {
            return new SingleLegBilateralReadOnlySnapshot(
                _fixedUpdateSequence,
                _driveEnabled,
                _kneeCharacterJointResponseIsolation.CurrentPositionErrorWorld,
                _kneeCharacterJointResponseIsolation.CurrentParentAnchorFeedforwardWorld,
                _kneeCharacterJointResponseIsolation.CurrentRawAccelerationWorld,
                _kneeCharacterJointResponseIsolation.CurrentAppliedAccelerationWorld,
                _kneeCharacterJointResponseIsolation.CurrentClamped);
        }

        public void SampleSourcePose(SourceStepContext source)
        {
            if (!IsBuilt)
            {
                if (_driveEnabled) AbortDrive("LEG_FOUNDATION_LOST");
                return;
            }
            if (!source.Valid || !_torso.CompletedSource.Matches(source))
                throw new InvalidOperationException("SINGLE_LEG_SOURCE_CONTEXT_INVALID");
            _runtimeSource = source; _insideSourceStep = true;
            try { SampleSourcePoseCore(); }
            finally { _insideSourceStep = false; }
        }

        private void SampleSourcePoseCore()
        {
            SampleSourcePoseInternal(true);
            // v0.2.102.1 hotfix: kinematic preview follow is production-independent lifecycle behavior.
            // It must run whenever leg drive is OFF, regardless of trace state. v0.2.102 accidentally left
            // the retired timing probe's trace guard in place, nesting this call under _traceEnabled.
            if (!_driveEnabled)
            {
                FollowSourceKinematicPreview();
            }
        }

        public void FixedUpdate(PhysicsStepContext step, LegParentCommandFrame parent)
        {
            if (!IsBuilt)
            {
                if (_driveEnabled) AbortDrive("LEG_FOUNDATION_LOST");
                return;
            }
            if (!step.Valid || (_driveEnabled && (!parent.Matches(step) || !_runtimeSource.Valid ||
                _runtimeSource.Epoch != step.Epoch || _runtimeSource.Sequence != step.SourceSequence)))
                throw new InvalidOperationException("SINGLE_LEG_PHYSICS_OR_PARENT_CONTEXT_INVALID");
            if (_driveEnabled && (_dynamicParentActive != parent.PhysicalParent.Valid ||
                (_dynamicParentActive && (!parent.PhysicalParent.Matches(step) ||
                parent.PhysicalParent.BodyId != _pelvisBody.GetInstanceID() || parent.PhysicalParent.Epoch != _physicalParent.Epoch ||
                _pelvisBody.isKinematic || !DynamicEnvironmentNativeReady))))
                throw new InvalidOperationException("LEG_PHYSICAL_PARENT_OR_ENVIRONMENT_CONTRACT_LOST" +
                    (DynamicContactSelected ? ":" + DynamicContactIssue : ""));
            if (_driveEnabled && _dynamicParentActive && !DynamicContactSelected)
            {
                FreeSpaceBoundaryIssue boundary = FreeSpaceSelectorIssue;
                if (boundary != FreeSpaceBoundaryIssue.None)
                    throw new InvalidOperationException("LEG_FREE_SPACE_SELECTOR_CONTRACT_LOST:" + boundary);
            }
            if (_driveEnabled && _dynamicParentActive && PelvisRunScopes.HasHipCoupling(_dynamicScope) &&
                !parent.HipCommand.Matches(step, parent.PhysicalParent.BodyId))
                throw new InvalidOperationException("LEG_HIP_PARENT_COMMAND_NOT_CURRENT");
            if (_driveEnabled && _torso.ControlPolicy.UseDynamicTorsoRecovery &&
                (!_dynamicParentActive || !_torso.PublishedTorsoRecovery.Matches(step)))
                throw new InvalidOperationException("DYNAMIC_TORSO_RECOVERY_SHARED_CLOCK_MISMATCH");
            if (_dynamicParentActive) { _physicalParent = parent.PhysicalParent; _dynamicParentSteps++; }
            _runtimeStep = step; _parentCommand = parent;
            _report.RuntimeContext = step; _report.RuntimeParentMatched = parent.Matches(step);
            _insidePhysicsStep = true;
            try { FixedUpdateCore(); }
            finally { _insidePhysicsStep = false; }
        }

        private bool _hipContinuationPending;
        private int _hipContinuationKneeFixed;

        internal bool CompleteHipControllerStep(PhysicsStepContext step)
        {
            if (!_driveEnabled || !_hipContinuationPending || !_runtimeStep.Matches(step) ||
                !_hipAnchorPair.Report.CurrentApplied || !_hipAnchorPair.Report.Current.Step.Matches(step) ||
                !_torso.HipAttitudeCommand.Matches(_parentCommand.HipCommand)) return false;
            _hipContinuationPending = false;
            _lastHipAppliedAngularAccelerationWorld = _hipAnchorPair.Report.Current.AllocatedHipAlpha;
            if (_traceEnabled)
            {
                _legTurnCurrent.HipFinalA = _lastHipAppliedAngularAccelerationWorld;
                _legTurnCurrent.HipCommitted = true;
                HipGoalPathEvidence path = _legTurnCurrent.HipGoalPath;
                path.Committed = true; _legTurnCurrent.HipGoalPath = path;
            }
            _insidePhysicsStep = true;
            try { CompleteControllerStep(_hipContinuationKneeFixed); }
            finally { _insidePhysicsStep = false; }
            return _driveEnabled;
        }

        private void FixedUpdateCore()
        {
            if (!IsBuilt || !_driveEnabled)
                return;

            try
            {
                if (_hipContinuationPending) { AbortDrive("HIP_CONTROLLER_PREVIOUS_STEP_INCOMPLETE"); return; }
                // V04450 resolves Upper momentum before clearing last-solve native gravity.
                if (!_upperSelfWeightTransfer.PrepareControllerStep(_upperBody, _fixedUpdateSequence + 1))
                {
                    AbortDrive("UPPER_SELF_WEIGHT_PRESTEP_INVALID:" + _upperSelfWeightTransfer.Report.InvalidReason);
                    return;
                }
                // v0.4.19: B may have left native Lower gravity ON for the just-completed exact STANCE solve.
                // Clear it before every frozen Hip/Knee/Foot controller and structural contract is evaluated.
                if (!_stanceLowerNativeGravity.PrepareControllerStep(_lowerBody, _report))
                {
                    AbortDrive("LOWER_NATIVE_GRAVITY_PRESTEP_INVALID:" + (_report.StanceLowerNativeGravityInvalidReason ?? "UNKNOWN"));
                    return;
                }

                if (!_torso.DriveEnabled)
                {
                    AbortDrive("PARENT_TORSO_DRIVE_LOST");
                    return;
                }

                // v0.4.1.1: contact-shadow semantic invalidity is a hard fail-close condition, but it is not
                // a numerical NaN/Infinity state. Classify it before the legacy aggregate finite check so logs
                // preserve the actual owner instead of collapsing the failure into NON_FINITE_STATE.
                if (_footFoundation != null && _footFoundation.IsBuilt && !_footFoundation.ContactShadowFinite)
                {
                    AbortDrive("FOOT_CONTACT_SHADOW_INVALID:" + (_footFoundation.CurrentInvalidReason ?? "UNKNOWN"));
                    return;
                }

                if (!HasFiniteState())
                {
                    AbortDrive("NON_FINITE_STATE");
                    return;
                }

                // Upper/Lower angularDrag must both match the frozen profile. The rejected zero-drag
                // experiment is retired; this readback contract applies no force or torque.
                if (!_legAngularDragContract.ValidateStep(_upperBody, _lowerBody, _profile.AngularDrag, _report))
                {
                    AbortDrive("HIP_UPPER_ANGULAR_DRAG_ISOLATION_INVALID");
                    return;
                }

                _fixedUpdateSequence++;
                UpdatePhaseTimingBeforeBodyMeasurement();
                // v0.2.102.1 hotfix: this estimator feeds production Governor/D-term paths and therefore
                // must never be gated by tracing. v0.2.102 accidentally inherited the retired timing
                // observer's _traceEnabled guard after that observer call was removed.
                UpdateBodyRelativeAngularVelocities();
                UpdateMeasuredTransportKinematics();
                BeginLegTurnEvidence();

                Quaternion settledDesiredMechanicalRotation = (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization)
                    ? _hipControlFrame.StrictMechanicalRotation
                    : _pelvisBody.rotation;
                _hipMechanicalParent.ObserveSettled(
                    _measuredPelvisAngularVelocity,
                    _pelvisBody.rotation,
                    settledDesiredMechanicalRotation,
                    GetRawPelvisHipAnchorWorld(),
                    PhysicsDt,
                    _report);

                float upperDistance = Vector3.Distance(_sampledUpperPosition, _upperBody.position);
                float lowerDistance = Vector3.Distance(_sampledLowerPosition, _lowerBody.position);

                // v0.2.28 read-only observation happens BEFORE fail-fast so the exact fling/abort state is preserved.
                Quaternion wholeLegCurrentHip = Quaternion.Inverse(_pelvisBody.rotation) * _upperBody.rotation;
                float wholeLegHipErrorDeg = Quaternion.Angle(_sampledTargetHipRelative, wholeLegCurrentHip);
                float wholeLegKneeOffDeg = GetCurrentKneeOffAxisErrorDegrees();

                // v0.2.32 exact world-position decomposition, sampled before fail-fast.
                // UpperError = source Hip - physics Upper anchor.
                // LowerError = UpperError + Upper lever/orientation error + Knee joint anchor error.
                Vector3 physicsHipChildAnchor = GetHipChildAnchorWorld();
                Vector3 physicsHipParentAnchor = GetHipParentAnchorWorld();
                Vector3 physicsKneeParentAnchor = _upperBody.transform.TransformPoint(_kneeParentAnchorUpperLocal);
                Vector3 physicsKneeChildAnchor = _lowerBody.transform.TransformPoint(_kneeChildAnchorLowerLocal);
                Vector3 hipJointAnchorErrorVector = physicsHipParentAnchor - physicsHipChildAnchor;
                Vector3 kneeJointAnchorErrorVector = physicsKneeParentAnchor - physicsKneeChildAnchor;

                // v0.2.36: update authoritative physical-recovery + continuous-health state BEFORE A-FF/controller.
                // A health re-entry can happen even after an earlier recovery completed; it suppresses A-FF but keeps baseline authority.
                _hipAuthority.Step(
                    _profile,
                    _torso.ControlPhase,
                    _torso.ControlRecoveryActive,
                    wholeLegHipErrorDeg,
                    CurrentUnscaledTime,
                    PhysicsDt,
                    _report);
                _softKneeAuthority.Step(
                    _profile,
                    _torso.ControlPhase,
                    _torso.ControlRecoveryActive,
                    wholeLegKneeOffDeg,
                    CurrentUnscaledTime,
                    PhysicsDt,
                    _report);

                // v0.2.36 fail-closed health invariants. Severe tracking divergence must re-enter pending in the
                // same FixedUpdate, and continuous-health re-entry must never silently enable the post-Source boost.
                bool sourceActiveForHealth = _torso.Report != null && _torso.ControlPhase.SourceTurnEventActive;
                if (!sourceActiveForHealth &&
                    ((wholeLegHipErrorDeg >= _profile.LegHipTrackingHealthImmediateErrorDegrees && !_hipAuthority.PhysicalRecoveryPending) ||
                     (wholeLegKneeOffDeg >= _profile.LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees && !_softKneeAuthority.PhysicalRecoveryPending) ||
                     (_hipAuthority.TrackingHealthUnhealthy && _hipAuthority.RecoveryBoostActive) ||
                     (_softKneeAuthority.TrackingHealthUnhealthy && _softKneeAuthority.RecoveryBoostActive)))
                {
                    AbortDrive("TRACKING_HEALTH_REENTRY_INVARIANT_BROKEN");
                    return;
                }

                float hipAnchorDistance = hipJointAnchorErrorVector.magnitude;
                float kneeAnchorDistance = kneeJointAnchorErrorVector.magnitude;
                bool positionSafetyFailed =
                    !IsFinite(upperDistance) || !IsFinite(lowerDistance) ||
                    !IsFinite(hipAnchorDistance) || !IsFinite(kneeAnchorDistance) ||
                    upperDistance > _profile.LegDriveDisableDistance || lowerDistance > _profile.LegDriveDisableDistance ||
                    hipAnchorDistance > _profile.JointAnchorSafetyDistance || kneeAnchorDistance > _profile.JointAnchorSafetyDistance;
                string safetyReason =
                    "POSITION_SAFETY_GATE:upper=" + F(upperDistance) +
                    "/lower=" + F(lowerDistance) +
                    "/hipAnchor=" + F(hipAnchorDistance) +
                    "/kneeAnchor=" + F(kneeAnchorDistance);

                if (positionSafetyFailed)
                {
                    AbortDrive(safetyReason);
                    return;
                }

                // Raw sampled Knee pose/W remain the baseline and the independent health truth.
                // V041170 B replaces only consumed R/W/A below with one continuous reference;
                // the rejected v0.2.104 phase-lead branch is not restored.
                _controlTargetKneeRelative = _sampledTargetKneeRelative;
                _controlTargetKneeRelativeAngularVelocityLocal = _sampledTargetKneeRelativeAngularVelocityLocal;

                _hipControlFrame.Step(
                    _profile,
                    _torso.ControlPhase,
                    _pelvisBody.rotation,
                    _measuredPelvisAngularVelocity,
                    _hipMechanicalParent.Built && _hipMechanicalParent.UseInertialized,
                    _hipMechanicalParent.SettledRotation,
                    _hipMechanicalParent.MeasuredProxyAngularVelocity,
                    CurrentUnscaledTime,
                    PhysicsDt,
                    _report);

                // v0.2.41 fail-closed strict integrator contract. During SOURCE/FRAME_RECOVERY the stored virtual W
                // may change only through the configured MaxA limiter. A temporary speed above MaxW is permitted only
                // when inherited from the deliberately raw healthy baseline and monotonically decelerating under MaxA.
                if (_report.HipControlFrameStrictIntegratorActiveThisStep &&
                    !_report.HipControlFrameAccelerationContractSatisfiedThisStep)
                {
                    Plugin.Logger?.LogError("[ARL][CONTROL_FRAME_INTEGRATOR_CONTRACT_BROKEN] " + _hipControlFrame.GetContractKey());
                    AbortDrive("CONTROL_FRAME_ACCELERATION_CONTRACT_BROKEN");
                    return;
                }
                if (_report.HipControlFrameStrictIntegratorActiveThisStep &&
                    !_report.HipControlFrameSpeedContractSatisfiedThisStep)
                {
                    Plugin.Logger?.LogError("[ARL][CONTROL_FRAME_INTEGRATOR_CONTRACT_BROKEN] " + _hipControlFrame.GetContractKey());
                    AbortDrive("CONTROL_FRAME_SPEED_CONTRACT_BROKEN");
                    return;
                }

                // v0.2.48 production: reference-side 25deg lag rebase is deliberately NOT a mechanical command.
                // The Hip CharacterJoint proxy consumes only the guard-decoupled strict 6/60 -> 8/80 branch; position
                // remains the exact raw Pelvis-connected Hip anchor path from v0.2.46.
                bool strictMechanicalFrameActive = _hipControlFrame.Active && _profile.LegHipControlFrameInertialization;
                _hipMechanicalParent.CommandForNextPhysics(
                    GetCommandedRawPelvisHipAnchorWorld(),
                    _dynamicParentActive ? _physicalParent.Rotation : _parentCommand.Rotation,
                    _hipControlFrame.StrictMechanicalRotation,
                    _hipControlFrame.StrictMechanicalAngularVelocityWorld,
                    _hipControlFrame.StrictMechanicalAngularAccelerationWorld,
                    _hipControlFrame.StrictMechanicalCurrentMaxAngularAcceleration,
                    strictMechanicalFrameActive,
                    PhysicsDt,
                    _report);

                // v0.2.60: fail closed only on the authoritative strict mathematical MaxA contract. The pose-derived
                // MoveRotation W/A reconstruction remains visible as an audit, but small finite-step mismatch must not
                // abort an otherwise valid 60/80 rad/s^2 strict trajectory.
                if (_hipMechanicalParent.UseInertialized && strictMechanicalFrameActive)
                {
                    float strictMechanicalMaxA = _hipControlFrame.StrictMechanicalCurrentMaxAngularAcceleration;
                    float proxyCommandA = _hipMechanicalParent.CommandedProxyAngularAcceleration.magnitude;
                    if (strictMechanicalMaxA > 0.0001f && !_hipMechanicalParent.CommandClosureAccelerationContractSatisfied)
                    {
                        Plugin.Logger?.LogError("[ARL][HIP_MECHANICAL_PARENT_STRICT_CONTRACT_BROKEN] cmdA=" + F(proxyCommandA) +
                            "/strictMaxA=" + F(strictMechanicalMaxA) +
                            "/strictA=" + F(_hipControlFrame.StrictMechanicalAngularAccelerationWorld.magnitude) +
                            "/closureWErr=" + F(_hipMechanicalParent.CommandClosureAngularVelocityError) +
                            "/closureAErr=" + F(_hipMechanicalParent.CommandClosureAngularAccelerationError) +
                            "/closureARatio=" + F(_hipMechanicalParent.CommandClosureMaxAccelerationRatio) +
                            "/poseAuditOK=" + _hipMechanicalParent.PoseDerivedAuditWithinTolerance +
                            "/poseAuditTolerance=" + F(_hipMechanicalParent.PoseDerivedAuditTolerance) +
                            "/poseOverMaxA=" + F(_hipMechanicalParent.PoseDerivedCommandOverMaxA) +
                            "/guardApplied=" + _report.HipControlFrameLagBoundAppliedThisStep +
                            "/activationSync=" + _report.HipControlFrameActivationSyncAppliedThisStep +
                            "/commandClosureSync=" + _report.HipControlFrameMechanicalClosureSyncAppliedThisStep +
                            "/syncPoseDeg=" + F(_report.HipControlFrameMechanicalClosureSyncPoseCorrectionDegrees) +
                            "/syncWDelta=" + F(_report.HipControlFrameMechanicalClosureSyncAngularVelocityCorrection));
                        AbortDrive("HIP_MECHANICAL_PARENT_STRICT_ACCELERATION_CONTRACT_BROKEN");
                        return;
                    }
                }

                // v0.2.44 production fix: the Hip-local reference lifecycle is driven by PHYSICAL feasibility, not by
                // SourceTurn close. The governor stays active through parent-frame and physical-Hip recovery, limits the
                // reference lead relative to the current physical Hip, predicts the latest Render pose only within the
                // existing source-sample validity window, and releases only after raw pose/W + physical health converge.
                bool sourceActiveForGovernor = _torso.Report != null && _torso.ControlPhase.SourceTurnEventActive;

                // v0.2.71 causal B: bound the animation Hip-local Source target BEFORE the existing governor/vector
                // control stack. Raw animation pose/W remain untouched for health and all raw Source diagnostics.
                _hipSourceReferenceShockAbsorber.Step(
                    _profile,
                    sourceActiveForGovernor,
                    _sampledTargetHipRelative,
                    _sampledTargetHipRelativeAngularVelocityLocal,
                    PhysicsDt,
                    _report);
                if (!_hipSourceReferenceShockAbsorber.OutputValid ||
                    !_hipSourceReferenceShockAbsorber.SpeedContractSatisfied ||
                    !_hipSourceReferenceShockAbsorber.AccelerationContractSatisfied)
                {
                    Plugin.Logger?.LogError("[ARL][HIP_SOURCE_REFERENCE_SHOCK_ABSORBER_CONTRACT_BROKEN] " + _hipSourceReferenceShockAbsorber.GetKey());
                    AbortDrive("HIP_SOURCE_REFERENCE_SHOCK_ABSORBER_CONTRACT_BROKEN");
                    return;
                }
                _sourceControlTargetHipRelative = _hipSourceReferenceShockAbsorber.Enabled
                    ? _hipSourceReferenceShockAbsorber.OutputPose
                    : _sampledTargetHipRelative;
                _sourceControlTargetHipRelativeAngularVelocityLocal = _hipSourceReferenceShockAbsorber.Enabled
                    ? _hipSourceReferenceShockAbsorber.OutputAngularVelocityLocal
                    : _sampledTargetHipRelativeAngularVelocityLocal;

                bool controlFrameActiveForGovernor = _hipControlFrame.Active && _profile.LegHipControlFrameInertialization;
                Quaternion governorParentRotation = controlFrameActiveForGovernor ? _hipControlFrame.Rotation : _pelvisBody.rotation;
                Quaternion physicalHipRelativeForGovernor = Quaternion.Inverse(governorParentRotation) * _upperBody.rotation;
                Vector3 physicalHipRelativeWForGovernor = controlFrameActiveForGovernor
                    ? Quaternion.Inverse(governorParentRotation) * (_measuredUpperAngularVelocity - _hipControlFrame.AngularVelocityWorld)
                    : _fixedBodyHipRelativeAngularVelocityLocal;
                _hipReferenceGovernor.Step(
                    _profile,
                    sourceActiveForGovernor,
                    controlFrameActiveForGovernor,
                    _hipAuthority.PhysicalRecoveryPending,
                    _sourceControlTargetHipRelative,
                    _sourceControlTargetHipRelativeAngularVelocityLocal,
                    physicalHipRelativeForGovernor,
                    physicalHipRelativeWForGovernor,
                    wholeLegHipErrorDeg,
                    _targetRenderSampleSequence,
                    _targetRenderSampleUnscaledTime,
                    CurrentUnscaledTime,
                    PhysicsDt,
                    _hipAuthority.CurrentMaxAngularAcceleration,
                    _report);
                if (!_hipReferenceGovernor.OutputValid)
                {
                    AbortDrive("HIP_REFERENCE_GOVERNOR_NON_FINITE_OUTPUT");
                    return;
                }
                if (!_hipReferenceGovernor.AccelerationContractSatisfied)
                {
                    Plugin.Logger?.LogError("[ARL][HIP_REFERENCE_GOVERNOR_CONTRACT_BROKEN] " + _hipReferenceGovernor.GetKey());
                    AbortDrive("HIP_REFERENCE_GOVERNOR_ACCELERATION_CONTRACT_BROKEN");
                    return;
                }
                _controlTargetHipRelative = _hipReferenceGovernor.Active
                    ? _hipReferenceGovernor.OutputPose
                    : _sourceControlTargetHipRelative;
                _controlTargetHipRelativeAngularVelocityLocal = _hipReferenceGovernor.Active
                    ? _hipReferenceGovernor.OutputAngularVelocityLocal
                    : _sourceControlTargetHipRelativeAngularVelocityLocal;

                // v0.2.72 causal B: v0.2.71 proved that bounding Hip-local alone is insufficient because parent and
                // local dynamics can recombine into a >200rad/s^2 Upper world reference. Compose the ACTUAL upstream
                // parent*local control target, bound that FINAL world pose/W with the same frozen 6/60 -> 8/80 budget,
                // then convert the bounded world trajectory back to an equivalent local target in the UNCHANGED parent
                // frame. Real Pelvis, strict mechanical parent, CharacterJoint and all actuator authority are untouched.
                Quaternion compositeControlParentRotation = controlFrameActiveForGovernor ? _hipControlFrame.Rotation : _pelvisBody.rotation;
                Vector3 compositeControlParentW = controlFrameActiveForGovernor ? _hipControlFrame.AngularVelocityWorld : _measuredPelvisAngularVelocity;
                Quaternion preCompositeAbsorberLocalTarget = _controlTargetHipRelative;
                Quaternion preCompositeAbsorberWorldTarget = compositeControlParentRotation * _controlTargetHipRelative;
                Vector3 preCompositeAbsorberWorldW = compositeControlParentW + compositeControlParentRotation * _controlTargetHipRelativeAngularVelocityLocal;
                bool upstreamSourceReferenceActive = _hipSourceReferenceShockAbsorber.Active ||
                    _hipReferenceGovernor.Active ||
                    _hipControlFrame.Active ||
                    _hipAuthority.PhysicalRecoveryPending;
                if (!TryPrepareHipNormalRecovery(sourceActiveForGovernor, out bool normalRecoveryRequested))
                {
                    AbortDrive("HIP_NORMAL_RECOVERY_HANDOFF_CONTRACT_BROKEN");
                    return;
                }
                _hipCompositeReferenceShockAbsorber.Step(
                    _profile,
                    sourceActiveForGovernor,
                    upstreamSourceReferenceActive,
                    preCompositeAbsorberWorldTarget,
                    preCompositeAbsorberWorldW,
                    PhysicsDt,
                    _report,
                    normalRecoveryRequested);
                if (!_hipCompositeReferenceShockAbsorber.OutputValid ||
                    !_hipCompositeReferenceShockAbsorber.SpeedContractSatisfied ||
                    !_hipCompositeReferenceShockAbsorber.AccelerationContractSatisfied)
                {
                    Plugin.Logger?.LogError("[ARL][HIP_COMPOSITE_REFERENCE_SHOCK_ABSORBER_CONTRACT_BROKEN] " + _hipCompositeReferenceShockAbsorber.GetKey());
                    AbortDrive("HIP_COMPOSITE_REFERENCE_SHOCK_ABSORBER_CONTRACT_BROKEN");
                    return;
                }
                if (_hipCompositeReferenceShockAbsorber.OutputAppliedThisStep)
                {
                    _controlTargetHipRelative = Quaternion.Inverse(compositeControlParentRotation) * _hipCompositeReferenceShockAbsorber.OutputWorldPose;
                    Vector3 boundedRelativeWWorld = _hipCompositeReferenceShockAbsorber.OutputWorldAngularVelocity - compositeControlParentW;
                    _controlTargetHipRelativeAngularVelocityLocal = Quaternion.Inverse(compositeControlParentRotation) * boundedRelativeWWorld;
                }
                _hipCompositeReferenceShockAbsorber.CaptureLocalCompensation(
                    preCompositeAbsorberLocalTarget,
                    _controlTargetHipRelative,
                    _report);

                CaptureHipGoalPath(compositeControlParentRotation, preCompositeAbsorberWorldTarget);
                if (!UpdateHipNormalReference(compositeControlParentRotation, compositeControlParentW))
                {
                    AbortDrive("HIP_NORMAL_REFERENCE_INVALID:" + _hipNormalReference.Issue);
                    return;
                }

                if (!UpdateKneeReference())
                {
                    AbortDrive("KNEE_REFERENCE_INVALID:" + (_kneeReference.Issue ?? "SELECTION_OR_STEP_MISMATCH"));
                    return;
                }

                // V041180 both arms use coherent Knee R/W/A; only B can pass planned
                // Knee A through source/torso recovery. All other gates still apply.
                if (!UpdateLegAngularAccelerationFeedforward())
                {
                    AbortDrive("LEG_ANGULAR_ACCELERATION_FEEDFORWARD_INVALID");
                    return;
                }

                // Retain fail-closed physical/frame protection for both lanes and
                // raw source/torso protection for Hip and the non-exempt Knee arm.
                if (!LegFeedforwardGatesSatisfied())
                {
                    AbortDrive("A_FF_RECOVERY_GATE_INVARIANT_BROKEN");
                    return;
                }

                if (HipFeedforwardGoalBridgeActive && (_legAngularAccelerationFeedforward.HipActiveThisStep ||
                    (!_torso.ControlPolicy.HipGoalGateOnlyHip && _legAngularAccelerationFeedforward.KneeActiveThisStep)))
                {
                    AbortDrive("A_FF_HIP_GOAL_GATE_SCOPE_INVARIANT_BROKEN");
                    return;
                }

                // v0.2.50 production A/B: apply the same bounded Hip-anchor linear acceleration at both leg COMs.
                // This is common-mode translation only; it does not add torque and it never consumes REAL_PELVIS raw acceleration.
                if (!_hipAnchorLinearTransport.StepAndApply(
                    _fixedUpdateSequence,
                    !_dynamicParentActive && _hipMechanicalParent.UseInertialized,
                    strictMechanicalFrameActive,
                    _hipMechanicalParent.CommandedProxyLinearAcceleration,
                    _profile.MaxParentMotionFeedforwardAcceleration,
                    _upperBody,
                    _lowerBody,
                    _report))
                {
                    AbortDrive("HIP_ANCHOR_LINEAR_TRANSPORT_INVALID");
                    return;
                }

                // v0.2.73-v0.2.74 B: with the Hip CharacterJoint removed, preserve only translational Hip-anchor attachment.
                // v0.2.73 A applies the tether at Upper+Lower COMs; v0.2.74 B applies it at Upper COM only so Lower
                // returns to the physical Upper -> Knee CharacterJoint -> Lower serial chain. No direct tether torque.
                if (_dynamicParentActive && PelvisRunScopes.HasHipCoupling(_dynamicScope))
                {
                    if (!_hipAnchorPair.Stage(_traceEnabled, _runtimeStep, _fixedUpdateSequence, _upperBody, _pelvisBody,
                        GetHipChildAnchorWorld(), GetHipParentAnchorWorld(), _physicalParent.SettledPointVelocity(_rawHipConnectedAnchorLocal),
                        _parentCommand.HipCommand))
                    {
                        // Include the rejected physical sample, rather than only the last accepted command.
                        float rejectedGap = _hipAnchorPair.Report.RejectedGapM;
                        if (IsFinite(rejectedGap)) _report.PeakHipAnchorErrorMeters = Mathf.Max(_report.PeakHipAnchorErrorMeters, rejectedGap);
                        AbortDrive("HIP_ANCHOR_PAIR_INVALID:" + _hipAnchorPair.Report.Issue); return;
                    }
                }
                else if (_hipCharacterJointResponseIsolation.Enabled)
                {
                    Rigidbody tetherParent = GetActiveHipMechanicalParentBody();
                    if (tetherParent == null)
                    {
                        AbortDrive("HIP_TRANSLATION_ONLY_TETHER_PARENT_MISSING");
                        return;
                    }
                    Vector3 hipChildAnchorWorld = GetHipChildAnchorWorld();
                    // A dynamic parent has no imposed next-pose jump. Its native point velocity is
                    // authoritative; the old kinematic prediction FF must not command against it.
                    Vector3 hipAnchorAccelerationFeedforward = _dynamicParentActive ? Vector3.zero : _hipAnchorAccelerationFeedforward.Step(
                        tetherParent,
                        GetCommandedRawPelvisHipAnchorWorld(),
                        _upperBody,
                        hipChildAnchorWorld,
                        _hipAnchorLinearTransport.ActiveThisStep,
                        PhysicsDt,
                        _report);
                    if (!_hipAnchorAccelerationFeedforward.CurrentValid)
                    {
                        AbortDrive("HIP_ANCHOR_ACCELERATION_FF_INVALID");
                        return;
                    }
                    _hipCharacterJointResponseIsolation.ApplyTranslationOnlyTether(
                        _upperBody, _lowerBody, tetherParent, hipChildAnchorWorld, GetHipParentAnchorWorld(),
                        hipAnchorAccelerationFeedforward, PhysicsDt, _report,
                        _dynamicParentActive ? _physicalParent.SettledPointVelocity(_rawHipConnectedAnchorLocal) : (Vector3?)null);
                    if (!_hipCharacterJointResponseIsolation.CurrentAnchorValid)
                    {
                        // Hip and Knee translation-only tethers share the same frozen 30 mm fail-closed contract.
                        // Capture the aborting Hip sample before UpdateReport() refreshes the generic peak so the
                        // trace summary, certification and fail-fast reason all describe the same physical step.
                        float currentHipAnchorError = _report.HipCharacterJointResponseIsolationCurrentAnchorErrorMeters;
                        if (IsFinite(currentHipAnchorError))
                            _report.PeakHipAnchorErrorMeters = Mathf.Max(_report.PeakHipAnchorErrorMeters, currentHipAnchorError);
                        AbortDrive(
                            "HIP_TRANSLATION_ONLY_TETHER_INVALID_ANCHOR:current=" + F(currentHipAnchorError) +
                            "/limit=" + F(HipCharacterJointResponseIsolation.TetherAnchorValidityMeters));
                        return;
                    }
                }
                else
                {
                    _hipCharacterJointResponseIsolation.CopyToReport(_report, null);
                    _hipAnchorAccelerationFeedforward.CopyToReport(_report);
                }

                // v0.2.76 B: remove the Knee CharacterJoint and preserve only Knee-anchor translation with
                // an ARL-owned Lower-COM force. v0.2.77 A keeps the exact PD-only tether; B adds only measured
                // Upper Knee-anchor world acceleration before the SAME 400/40 correction and SAME final 60 m/s^2 cap.
                int kneeCommandFixed = 0;
                if (_kneeCharacterJointResponseIsolation.Enabled)
                {
                    Vector3 kneeAnchorAccelerationFeedforward = _kneeAnchorAccelerationFeedforward.Step(
                        _upperBody,
                        _kneeParentAnchorUpperLocal,
                        PhysicsDt,
                        _report);
                    if (!_kneeAnchorAccelerationFeedforward.CurrentValid)
                    {
                        AbortDrive("KNEE_ANCHOR_ACCELERATION_FF_INVALID");
                        return;
                    }

                    _kneeCharacterJointResponseIsolation.ApplyTranslationOnlyTether(
                        _upperBody,
                        _lowerBody,
                        _kneeParentAnchorUpperLocal,
                        _kneeChildAnchorLowerLocal,
                        kneeAnchorAccelerationFeedforward,
                        PhysicsDt,
                        _report, _kneeSupportReaction.Enabled);

                    kneeCommandFixed = _fixedUpdateSequence;

                    if (!_kneeCharacterJointResponseIsolation.CurrentAnchorValid)
                    {
                        // The tether validates immediately after applying its force, before UpdateReport() normally refreshes
                        // the generic peak. Capture the aborting sample first so LEG_TRACE_SUMMARY and the fail-fast reason
                        // describe the same metric. The frozen 30 mm gate itself is unchanged.
                        float currentKneeAnchorError = _report.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters;
                        if (IsFinite(currentKneeAnchorError))
                            _report.PeakKneeAnchorErrorMeters = Mathf.Max(_report.PeakKneeAnchorErrorMeters, currentKneeAnchorError);
                        AbortDrive(
                            "KNEE_TRANSLATION_ONLY_TETHER_INVALID_ANCHOR:current=" + F(currentKneeAnchorError) +
                            "/limit=" + F(KneeCharacterJointResponseIsolation.TetherAnchorValidityMeters));
                        return;
                    }
                }
                else
                {
                    if (_kneeAnchorAccelerationFeedforward.Enabled)
                    {
                        AbortDrive("KNEE_ANCHOR_ACCELERATION_FF_BASELINE_MISSING");
                        return;
                    }
                    _kneeAnchorAccelerationFeedforward.CopyToReport(_report);
                    _kneeCharacterJointResponseIsolation.CopyToReport(_report);
                }

                if (_kneeFlexionAllocation.Enabled && (!_kneeAnchorAccelerationFeedforward.Enabled || !_kneeCharacterJointResponseIsolation.Enabled || _kneeJoint != null))
                {
                    AbortDrive("KNEE_FLEXION_ALLOCATION_BASELINE_MISSING");
                    return;
                }

                ApplyHipPd();
                if (!_driveEnabled) return;
                if (_dynamicParentActive && PelvisRunScopes.HasHipCoupling(_dynamicScope))
                {
                    _hipContinuationPending = true; _hipContinuationKneeFixed = kneeCommandFixed;
                    return; // Both requests and receiver allocation must commit before Knee consumes Hip alpha.
                }
                CompleteControllerStep(kneeCommandFixed);
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError(
                    "[ARL][LEG_FIXED_UPDATE_EXCEPTION] type=" + e.GetType().Name +
                    " | message=" + e.Message + " | " + e);
                AbortDrive("FIXED_UPDATE_EXCEPTION:" + e.GetType().Name);
            }
        }

        private void CompleteControllerStep(int kneeCommandFixed)
        {
            try
            {
                ApplyKneePd();
                if (!_driveEnabled) return;
                UpdateReport();
                UpdateAttributionMetrics();

                if (_footFoundation != null && _footFoundation.IsBuilt)
                {
                    // v0.3.52 maintenance: v0.3.34 reconciliation still needs the current FixedUpdate phase
                    // before the retained trace classifier runs later in this method. Keep this as a read-only
                    // snapshot rather than consuming _report.AttributionPhase from the previous FixedUpdate.
                    string attributionPhase = GetCurrentAttributionPhaseSnapshot();
                    Vector3 footAttributionKneeAxisWorld = GetCurrentKneeFlexionAxisWorld();
                    bool footFinite = _footFoundation.ObserveFixed(
                        _runtimeStep, _fixedUpdateSequence,
                        _targetRenderSampleSequence,
                        _targetSampleTiming.Fresh,
                        _targetSampleTiming.HeldDepth,
                        _targetSampleTiming.AgeSeconds,
                        _traceEnabled,
                        attributionPhase,
                        _report.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters,
                        _report.KneeOffAxisErrorDegrees,
                        footAttributionKneeAxisWorld,
                        _kneeResponseRelativeCommandWorld,
                        _stanceLowerNativeGravity.Enabled && _stanceFootLowerReciprocal.Enabled &&
                        _stanceLowerSelfWeightTransfer.Enabled, _legTurnCurrent);
                    if (!footFinite)
                    {
                        if (!_footFoundation.ContactShadowFinite)
                            AbortDrive("FOOT_CONTACT_SHADOW_INVALID:" + (_footFoundation.CurrentInvalidReason ?? "UNKNOWN"));
                        else
                            AbortDrive("FOOT_ANKLE_FOUNDATION_INVALID:" + (_footFoundation.CurrentInvalidReason ?? "UNKNOWN"));
                        return;
                    }

                    // Decide B's normal algorithm only AFTER the current contact lifecycle. The Lower
                    // Knee command is issued once, before its matching Upper force and the common ledger.
                    if (_kneeSupportReaction.Enabled && !_kneeCharacterJointResponseIsolation.ApplyDeferredTranslationOnlyTether(
                        _upperBody, _lowerBody, _footFoundation.ExactAuthorizedStanceSupportLoadEligible,
                        _footFoundation.StanceContactZeroFriction, _footFoundation.CurrentAuthorizedSupportNormalWorld, _report))
                    {
                        AbortDrive("KNEE_DEFERRED_NORMAL_CONSTRAINT_INVALID");
                        return;
                    }

                    // v0.4.21 baseline: v0.4.20 B reciprocity is frozen ON in both arms. It still applies only the
                    // exact equal-and-opposite support-normal component of the already-issued Foot tether force at Lower COM.
                    // The new experiment is the separate self-weight transfer layer below, not a change to this tether path.
                    if (!_stanceFootLowerReciprocal.Step(
                        _traceEnabled,
                        _fixedUpdateSequence,
                        _footFoundation.ExactAuthorizedStanceSupportLoadEligible,
                        _footFoundation.StanceContactZeroFriction,
                        _lowerBody,
                        _footFoundation.Report,
                        _footFoundation.CurrentAuthorizedSupportNormalWorld,
                        _report))
                    {
                        AbortDrive("FOOT_LOWER_RECIPROCAL_LOAD_TRANSFER_INVALID:" + (_report.StanceFootLowerReciprocalInvalidReason ?? "UNKNOWN"));
                        return;
                    }

                    // v0.4.21 single-variable intervention: V04200 B reciprocity is frozen ON in both arms.
                    // B adds an independent equal/opposite support-normal force pair representing only Lower native
                    // self-weight transfer through an idealized axial ankle/support constraint. It does not reuse or
                    // raise the Foot tracking tether authority.
                    if (!_stanceLowerSelfWeightTransfer.Step(
                        _traceEnabled,
                        _fixedUpdateSequence,
                        _footFoundation.ExactAuthorizedStanceSupportLoadEligible,
                        _footFoundation.StanceContactZeroFriction,
                        _lowerBody,
                        _footFoundation.DynamicBody,
                        _footFoundation.CurrentAuthorizedSupportNormalWorld,
                        PhysicsDt,
                        _report))
                    {
                        AbortDrive("LOWER_SELF_WEIGHT_SUPPORT_TRANSFER_INVALID:" + (_report.StanceLowerSelfWeightTransferInvalidReason ?? "UNKNOWN"));
                        return;
                    }

                    // V04430 mirrors the committed normal PD command only. ObserveFixed owns same-step
                    // eligibility; release keeps the original free-space command and suppresses the reaction.
                    if (!_kneeSupportReaction.Step(_traceEnabled, _fixedUpdateSequence, kneeCommandFixed,
                        _kneeCharacterJointResponseIsolation.Enabled && _kneeJoint == null,
                        _footFoundation.ExactAuthorizedStanceSupportLoadEligible,
                        _footFoundation.StanceContactZeroFriction, _upperBody, _lowerBody,
                        _kneeCharacterJointResponseIsolation.CurrentAppliedAccelerationWorld,
                        _footFoundation.CurrentAuthorizedSupportNormalWorld, PhysicsDt,
                        _kneeCharacterJointResponseIsolation.CurrentSupportNormalConstraint))
                    {
                        AbortDrive("KNEE_SUPPORT_NORMAL_REACTION_INVALID:" + _kneeSupportReaction.Report.InvalidReason);
                        return;
                    }

                    // v0.4.21 freezes V04190 B/ON and V04200 B/ON in BOTH arms. Lower gravity is still scheduled only
                    // after every frozen controller and both load-transfer selectors have issued their current-solve forces;
                    // controller math therefore starts gravity-OFF. From v0.4.24 onward V04210 is frozen B/ON;
                    // v0.4.29 freezes CCD/damping B and captured solver originals; V04280 is diagnostic-only.
                    if (!_stanceLowerNativeGravity.SchedulePhysicsStep(
                        _traceEnabled,
                        _fixedUpdateSequence,
                        _footFoundation.ExactAuthorizedStanceSupportLoadEligible,
                        _lowerBody,
                        _footFoundation.CurrentAuthorizedSupportNormalWorld,
                        _kneeCharacterJointResponseIsolation.CurrentAppliedAccelerationWorld,
                        PhysicsDt,
                        _report))
                    {
                        AbortDrive("LOWER_NATIVE_GRAVITY_SCHEDULE_INVALID:" + (_report.StanceLowerNativeGravityInvalidReason ?? "UNKNOWN"));
                        return;
                    }

                    if (!_upperSelfWeightTransfer.Schedule(_traceEnabled, _fixedUpdateSequence,
                        _footFoundation.ExactAuthorizedStanceSupportLoadEligible,
                        _footFoundation.StanceContactZeroFriction, _upperBody, _lowerBody, _footFoundation.DynamicBody,
                        _footFoundation.CurrentAuthorizedSupportNormalWorld, PhysicsDt,
                        _report, _footFoundation.Report, HipLedgerAcceleration,
                        _dynamicParentActive ? _dynamicScope : PelvisRunScope.TorsoPairOnly))
                    {
                        AbortDrive("UPPER_SELF_WEIGHT_SCHEDULE_INVALID:" + _upperSelfWeightTransfer.Report.InvalidReason);
                        return;
                    }

                    // v0.4.23 read-only solve continuity observer: every current-solve Foot/Lower load selector
                    // has now issued its command. Capture the pre-physics state and scheduled Foot-normal load so
                    // authorized Ground callback(s) and next Fixed settled state can be joined by the same sequence.
                    if (!_footFoundation.CaptureSupportStepEvidencePrePhysics(
                        _runtimeStep, _fixedUpdateSequence,
                        _report.StanceLowerSelfWeightTransferCurrentApplied,
                        _report.StanceLowerSelfWeightTransferCurrentFootLoadAcceleration,
                        _report, _kneeCharacterJointResponseIsolation.CurrentAppliedAccelerationWorld,
                        _torso.Report.MotionPhase, _dynamicParentActive ? _physicalParent : default, _legTurnCurrent, _dynamicScope))
                    {
                        AbortDrive("FOOT_VELOCITY_SOLVER_PRE_SOLVE_INVALID:" + (_footFoundation.CurrentInvalidReason ?? "UNKNOWN"));
                        return;
                    }

                }
                else if (_kneeSupportReaction.Enabled)
                {
                    // A support experiment cannot silently run without its lifecycle owner.
                    AbortDrive("KNEE_NORMAL_CONSTRAINT_FOOT_LIFECYCLE_MISSING");
                    return;
                }
                if (_traceEnabled)
                    UpdateRetainedPhaseClassification();
                UpdateTraceStats();
                EmitTraceIfDue();
                _targetSampleTiming.CompleteControlStep(_targetRenderSampleSequence);

                if (!HasFiniteState())
                    AbortDrive("NON_FINITE_AFTER_DRIVE");
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError(
                    "[ARL][LEG_FIXED_UPDATE_EXCEPTION] type=" + e.GetType().Name +
                    " | message=" + e.Message + " | " + e);
                AbortDrive("FIXED_UPDATE_EXCEPTION:" + e.GetType().Name);
            }
        }

        public string SetKneeAxisCalibration(bool enabled)
        {
            if (!IsBuilt)
                return "ARL knee axis calibration：单腿链尚未创建。";

            if (enabled)
            {
                if (_kneeAxisCalibrator.Collecting)
                    return "ARL knee axis calibration：已经在收集。| " + GetKneeAxisCalibrationStatus();
                if (_driveEnabled)
                    return "ARL knee axis calibration：拒绝启动。请先 arl_leg_drive(off)，标定阶段必须保持 kinematic preview。";
                if (_traceEnabled)
                    return "ARL knee axis calibration：拒绝启动。请先 arl_trace(off)，避免把标定窗口与动态 Trace 混在一起。";
                if (!_torso.DriveEnabled)
                    return "ARL knee axis calibration：拒绝启动。内部 frozen-stack 依赖未满足。";
                _previousCalibrationPelvisAngularVelocityRaw = Vector3.zero;
                _hasPreviousCalibrationPelvisAngularVelocityRaw = false;
                _kneeAxisCalibrator.Begin(_initialKneeFlexionAxisLowerLocal);
                _kneeAxisCalibrator.CopyToReport(_report);
                string started = GetKneeAxisCalibrationStatus();
                Plugin.Logger?.LogWarning("[ARL][KNEE_AXIS_CALIBRATION] side=" + (_report.Side ?? "<none>") + " | action=START | " + started);
                return "ARL knee axis calibration | action=START | " + started;
            }

            if (!_kneeAxisCalibrator.Collecting)
                return "ARL knee axis calibration：当前未在收集。| " + GetKneeAxisCalibrationStatus();

            _kneeAxisCalibrator.Stop();
            _kneeAxisCalibrator.CopyToReport(_report);
            string stopped = GetKneeAxisCalibrationStatus();
            Plugin.Logger?.LogWarning("[ARL][KNEE_AXIS_CALIBRATION] side=" + (_report.Side ?? "<none>") + " | action=STOP | " + stopped);
            return "ARL knee axis calibration | action=STOP | " + stopped;
        }

        public string ApplyCalibratedKneeAxis()
        {
            if (!IsBuilt)
                return "ARL knee axis apply：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL knee axis apply：拒绝。必须先 arl_leg_drive(off)，禁止运行中修改 Knee CharacterJoint axis。";
            if (_traceEnabled)
                return "ARL knee axis apply：拒绝。请先 arl_trace(off)，应用新轴后再开启独立验证 Trace。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL knee axis apply：拒绝。请先 arl_knee_axis_calibrate(off)。";

            _kneeAxisCalibrator.CopyToReport(_report);
            if (!_kneeAxisCalibrator.Ready)
                return "ARL knee axis apply：候选轴尚未达到应用门槛。| " + GetKneeAxisCalibrationStatus();

            Vector3 candidate = _kneeAxisCalibrator.CandidateAxisLowerLocal;
            if (!IsFinite(candidate) || candidate.sqrMagnitude < 0.5f)
                return "ARL knee axis apply：候选轴无效。";

            ApplyKneeCharacterJointAxes(candidate.normalized);
            _kneeAxisMode = "DATA_DRIVEN_CALIBRATED";
            _kneeAxisCalibrator.MarkApplied();
            ResetBodyRelativeAngularVelocityEstimator();
            ResetMeasuredTransportKinematics();
            SampleSourcePoseInternal(false);
            FollowSourceKinematicPreview();
            UpdateReport();
            _kneeAxisCalibrator.CopyToReport(_report);

            string result =
                "ARL knee axis apply | success=True" +
                " | mode=" + _kneeAxisMode +
                " | axisLowerLocal=" + V(_kneeFlexionAxisLowerLocal) +
                " | angleFromInitialDeg=" + F(_kneeAxisCalibrator.AngleFromInitialDegrees) +
                " | principalEnergy=" + F(_kneeAxisCalibrator.PrincipalEnergyRatio) +
                " | samples=" + _kneeAxisCalibrator.AcceptedSamples;
            Plugin.Logger?.LogWarning("[ARL][KNEE_AXIS_APPLY] side=" + (_report.Side ?? "<none>") + " | " + result);
            return result;
        }

        public string ResetKneeAxisToInitial()
        {
            if (!IsBuilt)
                return "ARL knee axis reset：单腿链尚未创建。";
            if (_driveEnabled || _traceEnabled || _kneeAxisCalibrator.Collecting)
                return "ARL knee axis reset：拒绝。请保持 leg drive/trace/calibration 全部 off。";

            ApplyKneeCharacterJointAxes(_initialKneeFlexionAxisLowerLocal);
            _kneeAxisMode = "INITIAL_CHARACTER_RIGHT";
            _kneeAxisCalibrator.Reset(_initialKneeFlexionAxisLowerLocal);
            ResetBodyRelativeAngularVelocityEstimator();
            ResetMeasuredTransportKinematics();
            SampleSourcePoseInternal(false);
            FollowSourceKinematicPreview();
            UpdateReport();
            string result = "ARL knee axis reset | success=True | mode=" + _kneeAxisMode + " | axisLowerLocal=" + V(_kneeFlexionAxisLowerLocal);
            Plugin.Logger?.LogWarning("[ARL][KNEE_AXIS_RESET] side=" + (_report.Side ?? "<none>") + " | " + result);
            return result;
        }

        public string GetKneeAxisCalibrationStatus()
        {
            string calibration = _kneeAxisCalibrator.GetStatusText();
            _kneeAxisCalibrator.CopyToReport(_report);
            string current = V(_kneeFlexionAxisLowerLocal);
            return "axisMode=" + _kneeAxisMode + " | currentLowerLocal=" + current + " | " + calibration;
        }

        private Vector3 GetCurrentKneeFlexionAxisWorld()
        {
            if (_lowerBody == null)
                return Vector3.zero;
            Vector3 axis = _lowerBody.transform.TransformDirection(_kneeFlexionAxisLowerLocal);
            return axis.sqrMagnitude > 0.0001f ? axis.normalized : Vector3.zero;
        }

        public string SetTrace(bool enabled)
        {
            if (enabled && !_driveEnabled)
                return "ARL leg trace：拒绝启动。请先 arl_leg_drive(on)。";
            if (enabled && _kneeAxisCalibrator.Collecting)
                return "ARL leg trace：拒绝启动。Knee axis calibration 正在收集；请先 arl_knee_axis_calibrate(off)。";
            if (_traceEnabled == enabled)
                return "ARL leg trace | enabled=" + enabled + " | alreadyInRequestedState=True | action=NO_OP";

            bool wasEnabled = _traceEnabled;
            if (!enabled && wasEnabled)
            {
                try
                {
                    // Finalize Foot-local diagnostic episodes/windows before emitting trace-stop summaries. v0.3.73
                    // translation-tail episodes may still be active at trace stop. EndTrace is diagnostic-only and writes no physics.
                    _upperSelfWeightTransfer.EndTrace();
                    _footFoundation?.EndTrace();
                    CopyTraceStatsToReport();
                    CopyLegAngularAccelerationFeedforwardToReport();
                    _report.HipNormalReferenceSummary = _hipNormalReference.GetSummary();
                    _report.KneeReferenceSummary = _kneeReference.GetSummary();
                    _report.KneePlannedFeedforwardSummary = _legAngularAccelerationFeedforward.GetKneeSourceRecoverySummary();

                    // v0.3.52 maintenance: completed historical observers are retired from the hot path. Keep only diagnostics that
                    // still support the frozen runtime contract, current safety/recovery interpretation, or the active Foot/Ankle distal-boundary foundation.
                    if (!_dynamicParentActive) Plugin.Logger?.LogWarning("[ARL][UPPER_SELF_WEIGHT_V04450] side=" + (_report.Side ?? "<none>") + " | " + _upperSelfWeightTransfer.Report.GetDetails());
                    Plugin.Logger?.LogWarning("[ARL][LEG_TRACE_SUMMARY] side=" + (_report.Side ?? "<none>") + " | realInertia=True | " + GetTraceSummary());
                    Plugin.Logger?.LogWarning("[ARL][HIP_REFERENCE_GOVERNOR_KEY] side=" + (_report.Side ?? "<none>") + " | " + _hipReferenceGovernor.GetKey());
                    Plugin.Logger?.LogWarning("[ARL][KNEE_TETHER_SAFETY_KEY] side=" + (_report.Side ?? "<none>") + " | " + _kneeCharacterJointResponseIsolation.GetKey(_report));
                    if (_footFoundation != null && _footFoundation.IsBuilt)
                    {
                        Plugin.Logger?.LogWarning("[ARL][FOOT_TRACE_SUMMARY] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.Report.GetConsoleSummary(!_dynamicParentActive));
                        if (!_dynamicParentActive) Plugin.Logger?.LogWarning("[ARL][FOOT_SOLE_REQUAL_V0561] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.GetSoleRequalificationStatus());
                        if (!_dynamicParentActive) Plugin.Logger?.LogWarning("[ARL][FOOT_CONTACT_IDENTITY_V0400] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.GetContactIdentityStatus());
                        Plugin.Logger?.LogWarning("[ARL][FOOT_ANKLE_DRIVE_V058] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.GetAnkleDrivePhysicalStatus());
                        Plugin.Logger?.LogWarning("[ARL][ANKLE_TARGET_A_FF_V068] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.GetAnkleTargetAccelerationStatus());
                        Plugin.Logger?.LogWarning("[ARL][ANKLE_TARGET_A_CONDITIONER_V068] side=" + (_report.Side ?? "<none>") + " | frozenProduction=True | " + _footFoundation.GetTargetAccelerationPersistenceConditionerStatus());
                        Plugin.Logger?.LogWarning("[ARL][ANKLE_CAPTURE_AUTHORITY_V073] side=" + (_report.Side ?? "<none>") + " | production=True | " + _footFoundation.GetTranslationCaptureAuthorityStatus());
                        Plugin.Logger?.LogWarning("[ARL][ANKLE_CONSTRAINT_V063] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.GetAnkleConstraintStatus());
                        Plugin.Logger?.LogWarning("[ARL][ANKLE_TETHER_TRANSPORT_V064] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.GetAnkleTransportStatus());
                        Plugin.Logger?.LogWarning("[ARL][ANKLE_COMMON_MODE_ANGULAR_TRANSPORT_V078] side=" + (_report.Side ?? "<none>") + " | " + _footFoundation.GetAnkleAngularTransportStatus());
                        if (!_dynamicParentActive) Plugin.Logger?.LogWarning("[ARL][FOOT_LOWER_RECIPROCAL_V04200] side=" + (_report.Side ?? "<none>") + " | " + (_report.StanceFootLowerReciprocalSummary ?? "<none>"));
                        if (!_dynamicParentActive) Plugin.Logger?.LogWarning("[ARL][LOWER_SELF_WEIGHT_TRANSFER_V04210] side=" + (_report.Side ?? "<none>") + " | " + (_report.StanceLowerSelfWeightTransferSummary ?? "<none>"));
                    }
                }
                finally
                {
                    // Diagnostic/native-read/logging failure must not leave trace ownership ON.
                    _traceEnabled = false;
                    _hipAnchorPair.Response.Stop();
                }
            }

            ResetLegTurnEvidence();
            _traceEnabled = enabled;
            if (enabled)
            {
                _footFoundation?.BeginTrace(_dynamicScope);
                _hipAnchorPair.ResetTrace();
                ResetTraceStats();
                _nextTraceTime = CurrentUnscaledTime + TraceLogInterval;
            }
            UpdateReport();
            if (!enabled && wasEnabled && !_dynamicParentActive)
            {
                string freeze = EvaluateSingleLegFreezeReadiness(true);
                Plugin.Logger?.LogWarning("[ARL][SINGLE_LEG_FREEZE_CERTIFICATION] side=" + (_report.Side ?? "<none>") + " | " + freeze);
                Plugin.Logger?.LogWarning("[ARL][SINGLE_LEG_FREEZE_KEY] side=" + (_report.Side ?? "<none>") + " | " + (_report.SingleLegFreezeKey ?? "<none>"));
            }
            return "ARL leg trace | enabled=" + enabled + " | compact=True | singleLegFreeze=" + SingleLegFreezeReadinessCertification.Mode +
                   (!enabled && wasEnabled ? " | " + GetTraceSummary() : string.Empty);
        }

        public bool FootProductionBaselineReady
        {
            get
            {
                return IsBuilt && _footFoundation != null && _footFoundation.IsBuilt &&
                       _footFoundation.AnkleDriveProductionArm &&
                       _footFoundation.AnkleConstraintProductionArm &&
                       string.Equals(_footFoundation.AnkleDriveArm, FootAnkleSourceDrivenRelativePdIsolation.ArmB, StringComparison.Ordinal) &&
                       !_footFoundation.AnkleConstraintCharacterJointArm &&
                       string.Equals(_footFoundation.AnkleTransportArm, FootAnkleTranslationTransportIsolation.ArmB, StringComparison.Ordinal) &&
                       _footFoundation.AnkleTransportFeedforwardEnabled &&
                       _footFoundation.AnkleAngularTransportFeedforwardEnabled &&
                       _footFoundation.AnkleAngularTransportProductionReady &&
                       _footFoundation.AnkleTargetAccelerationFeedforwardEnabled &&
                       _footFoundation.TranslationCaptureAuthorityProduction &&
                       _footFoundation.FootCollisionDetectionModeFinite &&
                       _footFoundation.Report.FootContactOffsetInitialized && _footFoundation.Report.FootContactOffsetFinite &&
                       _footFoundation.FootVelocitySolverFinite &&
                       !_footFoundation.AnkleJointDestroyPending;
            }
        }

        public bool FootRuntimeContractReady => FootProductionBaselineReady;

        public bool AnkleJointDestroyPending => _footFoundation != null && _footFoundation.IsBuilt && _footFoundation.AnkleJointDestroyPending;
        public string AnkleAngularTransportPolicy => _footFoundation != null && _footFoundation.IsBuilt ? _footFoundation.AnkleAngularTransportPolicy : FootAnkleAngularTransport.Production;
        internal Collider FootContactShadowCollider => _footFoundation != null && _footFoundation.IsBuilt ? _footFoundation.ContactShadowFootCollider : null;
        internal bool FootContactShadowFinite => _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.ContactShadowFinite;
        // v0.4.7 compile hotfix: expose the Foot foundation stance-contact lifecycle through
        // the single-leg boundary so the bilateral coordinator never reaches into the Foot
        // implementation directly. Keep the null semantics aligned with ContactShadowFinite.
        internal bool StanceContactLifecycleEnabled => _footFoundation != null && _footFoundation.IsBuilt && _footFoundation.StanceContactLifecycleEnabled;
        internal bool StanceContactLifecycleFinite => _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.StanceContactLifecycleFinite;
        internal string StanceContactFrictionIsolationArm => _footFoundation != null && _footFoundation.IsBuilt ? _footFoundation.StanceContactFrictionIsolationArm : FootStanceContactFrictionIsolation.ArmA;
        internal bool StanceContactFrictionIsolationFinite => _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.StanceContactFrictionIsolationFinite;
        internal bool StanceContactZeroFriction => _footFoundation != null && _footFoundation.IsBuilt && _footFoundation.StanceContactZeroFriction;
        internal string StanceLowerGroundAuthorityArm => _footFoundation != null && _footFoundation.IsBuilt ? _footFoundation.StanceLowerGroundAuthorityArm : FootStanceLowerGroundAnchorAuthority.ArmA;
        internal bool StanceLowerGroundAuthorityEnabled => _footFoundation != null && _footFoundation.IsBuilt && _footFoundation.StanceLowerGroundAuthorityEnabled;
        internal bool StanceLowerGroundAuthorityFinite => _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.StanceLowerGroundAuthorityFinite;
        internal string StanceFootNativeGravityArm => _footFoundation != null && _footFoundation.IsBuilt ? _footFoundation.StanceFootNativeGravityArm : FootStanceNativeGravitySupportLoadIsolation.ArmA;
        internal bool StanceFootNativeGravityEnabled => _footFoundation != null && _footFoundation.IsBuilt && _footFoundation.StanceFootNativeGravityEnabled;
        internal bool StanceFootNativeGravityFinite => _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.StanceFootNativeGravityFinite;
        internal string StanceLowerNativeGravityArm => _stanceLowerNativeGravity.Arm;
        internal bool StanceLowerNativeGravityEnabled => _stanceLowerNativeGravity.Enabled;
        internal bool StanceLowerNativeGravityFinite => _stanceLowerNativeGravity.Finite;
        internal string UpperSelfWeightArm => _upperSelfWeightTransfer.Report.Arm;
        internal bool UpperSelfWeightFinite => _upperSelfWeightTransfer.Report.Finite;
        internal bool UpperSelfWeightOff => _upperSelfWeightTransfer.Off && (_upperBody == null || !_upperBody.useGravity);
        internal bool SetUpperSelfWeightTransfer(string arm, out string error)
        {
            error = null;
            if (UpperSelfWeightSupportNormalLoadTransferIsolation.IsExperimentArm(arm) &&
                (!IsBuilt || _upperBody == null || _lowerBody == null || _footFoundation == null || !_footFoundation.IsBuilt ||
                 _driveEnabled || _traceEnabled || !_kneeSupportReaction.Enabled || _hipJoint != null || _kneeJoint != null ||
                 !_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled))
            {
                error = "UPPER_LOAD_REQUIRES_BUILT_COMMON_KNEE_B_UPPER_ONLY_NO_JOINT_STACK_DRIVE_TRACE_OFF";
                return false;
            }
            bool ok = _upperSelfWeightTransfer.Select(arm, _upperBody);
            if (!ok) error = _upperSelfWeightTransfer.Report.InvalidReason ?? "UPPER_LOAD_ARM_OR_GRAVITY_OFF_INVALID";
            return ok;
        }
        internal bool QuiesceUpperSelfWeightPreserveTrace(out string error)
        {
            bool ok = _upperSelfWeightTransfer.Quiesce(_upperBody);
            error = ok ? null : _upperSelfWeightTransfer.Report.InvalidReason;
            return ok;
        }

        internal string KneeSupportReactionArm => _kneeSupportReaction.Arm;
        internal bool KneeSupportReactionFinite => _kneeSupportReaction.Finite;
        internal bool KneeSupportReactionOff => _kneeSupportReaction.Off;
        internal bool SetKneeSupportNormalReaction(bool enabled, out string error)
        {
            error = null;
            if (enabled && (!IsBuilt || _upperBody == null || _lowerBody == null ||
                _footFoundation == null || !_footFoundation.IsBuilt || _driveEnabled || _traceEnabled ||
                !_kneeCharacterJointResponseIsolation.Enabled || _kneeJoint != null))
            {
                error = "KNEE_REACTION_SETUP_REQUIRES_BUILT_NO_JOINT_STACK_AND_DRIVE_TRACE_OFF";
                return false;
            }
            _kneeSupportReaction.Select(enabled);
            return true;
        }

        internal string StanceFootLowerReciprocalArm => _stanceFootLowerReciprocal.Arm;
        internal bool StanceFootLowerReciprocalEnabled => _stanceFootLowerReciprocal.Enabled;
        internal bool StanceFootLowerReciprocalFinite => _stanceFootLowerReciprocal.Finite;
        internal string StanceLowerSelfWeightTransferArm => _stanceLowerSelfWeightTransfer.Arm;
        internal bool StanceLowerSelfWeightTransferEnabled => _stanceLowerSelfWeightTransfer.Enabled;
        internal bool StanceLowerSelfWeightTransferFinite => _stanceLowerSelfWeightTransfer.Finite;
        internal string FootPositionSolverArm => _footFoundation != null ? _footFoundation.FootPositionSolverArm : FootSolverIterationBaselineContract.PositionArmA;
        internal bool FootPositionSolverEnabled => _footFoundation != null && _footFoundation.FootPositionSolverEnabled;
        internal bool SetFootPositionSolverArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetFootPositionSolverArm(arm, out error);
        }

        internal string FootVelocitySolverArm => _footFoundation != null ? _footFoundation.FootVelocitySolverArm : FootSolverIterationBaselineContract.ArmA;
        internal bool FootVelocitySolverFinite => _footFoundation != null && _footFoundation.FootVelocitySolverFinite;
        internal bool FootVelocitySolverBaselineRestored => _footFoundation != null && _footFoundation.FootVelocitySolverBaselineRestored;
        internal bool StancePlanarDampingEnabled => _footFoundation != null && _footFoundation.StancePlanarDampingEnabled;
        internal bool SetStancePlanarDamping(bool enabled)
        {
            if (_footFoundation == null || !_footFoundation.IsBuilt) return false;
            _footFoundation.SetStancePlanarDamping(enabled);
            return _footFoundation.StancePlanarDampingEnabled == enabled;
        }

        internal bool RestoreFootSolverIterationBaseline(out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.RestoreFootSolverIterationBaseline(out error);
        }

        internal string StanceSupportAngularArm => _footFoundation != null ? _footFoundation.StanceSupportAngularArm : FootStanceSupportNormalAngularStabilization.ArmA;
        internal bool StanceSupportAngularEnabled => _footFoundation != null && _footFoundation.StanceSupportAngularEnabled;
        internal bool StanceSupportAngularFinite => _footFoundation != null && _footFoundation.StanceSupportAngularFinite;
        internal bool SetStanceSupportAngularArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetStanceSupportAngularArm(arm, out error);
        }

        internal void RefreshFootSupportProbe() => _footFoundation?.RefreshSupportProbe();
        internal void BindFootSupportSurface(FootSupportSurfaceIsolation surface) => _footFoundation?.BindSupportSurface(surface);
        internal Collider CurrentFootGround => _footFoundation?.CurrentQualifiedGroundCollider;

        internal string FootCollisionDetectionModeArm => _footFoundation != null && _footFoundation.IsBuilt ? _footFoundation.FootCollisionDetectionModeArm : FootCollisionDetectionModeIsolation.IdleArm;
        internal bool SetFootContactOffsetArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetFootContactOffsetArm(arm, out error);
        }

        internal bool FootCollisionDetectionModeFinite => _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.FootCollisionDetectionModeFinite;

        internal bool SetFootCollisionDetectionModeArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetFootCollisionDetectionModeArm(arm, out error);
        }

        internal bool SetTranslationPredictiveTailCaptureArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetTranslationPredictiveTailCaptureArm(arm, out error);
        }

        internal bool SetStanceContactFrictionIsolationArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetStanceContactFrictionIsolationArm(arm, out error);
        }

        internal bool SetStanceLowerGroundAuthorityArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetStanceLowerGroundAuthorityArm(arm, out error);
        }

        internal bool SetStanceFootNativeGravityArm(string arm, out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.SetStanceFootNativeGravityArm(arm, out error);
        }

        internal bool SetStanceLowerNativeGravityArm(string arm, out string error)
        {
            error = null;
            if (!IsBuilt || _lowerBody == null)
            {
                error = "LOWER_BODY_NOT_BUILT";
                return false;
            }
            return _stanceLowerNativeGravity.SetArm(arm, _lowerBody, _report, out error);
        }

        internal bool SetStanceFootLowerReciprocalArm(string arm, out string error)
        {
            error = null;
            if (!IsBuilt || _lowerBody == null || _footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "LOWER_OR_FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _stanceFootLowerReciprocal.SetArm(arm, _report, out error);
        }

        internal bool SetStanceLowerSelfWeightTransferArm(string arm, out string error)
        {
            error = null;
            if (!IsBuilt || _lowerBody == null || _footFoundation == null || !_footFoundation.IsBuilt || _footFoundation.DynamicBody == null)
            {
                error = "LOWER_OR_FOOT_BODY_NOT_BUILT";
                return false;
            }
            return _stanceLowerSelfWeightTransfer.SetArm(arm, _report, out error);
        }

        internal bool QuiesceStanceLowerNativeGravityPreserveTrace(out string error)
        {
            return _stanceLowerNativeGravity.QuiescePreserveTrace(_lowerBody, _report, out error);
        }

        internal void SetStanceContactLifecycleEnabled(bool enabled)
        {
            if (_footFoundation != null && _footFoundation.IsBuilt)
                _footFoundation.SetStanceContactLifecycleEnabled(enabled);
        }

        internal void DisableStanceContactLifecycle()
        {
            if (_footFoundation != null && _footFoundation.IsBuilt)
                _footFoundation.DisableStanceContactLifecycle();
        }

        internal bool QuiesceStanceContactLifecyclePreserveTrace(out string error)
        {
            error = null;
            if (_footFoundation == null || !_footFoundation.IsBuilt)
            {
                error = "FOOT_FOUNDATION_NOT_BUILT";
                return false;
            }
            return _footFoundation.QuiesceStanceContactLifecyclePreserveTrace(out error);
        }

        public string GetFootSoleRequalificationStatus()
        {
            return _footFoundation != null
                ? _footFoundation.GetSoleRequalificationStatus()
                : "ARL Foot sole requalification | built=False";
        }

        internal void SetFootContactShadowArmA()
        {
            _footFoundation?.SetContactShadowArmA();
        }

        internal int DisableFootContactShadowPreserveTrace()
        {
            return _footFoundation != null ? _footFoundation.DisableContactShadowPreserveTrace() : 0;
        }

        internal bool NotifyFootContactShadowCollision(string phase, Collision collision)
        {
            return _footFoundation == null || _footFoundation.NotifyContactShadowCollision(phase, collision);
        }

        public string GetFootAnkleStatus()
        {
            if (!_profile.FootAnkleFoundationEnabled)
                return "ARL foot/ankle foundation | enabled=False";
            return _footFoundation != null ? _footFoundation.GetStatus() : "ARL foot/ankle foundation | built=False";
        }

        private Vector3 GetHipChildAnchorWorld()
        {
            if (_upperBody == null) return Vector3.zero;
            Vector3 local = _hipJoint != null ? _hipJoint.anchor : Vector3.zero;
            return _upperBody.transform.TransformPoint(local);
        }

        private Vector3 GetHipParentAnchorWorld()
        {
            if (_dynamicParentActive) return GetRawPelvisHipAnchorWorld();
            Rigidbody parent = GetActiveHipMechanicalParentBody();
            if (parent == null) return GetHipChildAnchorWorld();
            Vector3 local = _hipJoint != null
                ? _hipJoint.connectedAnchor
                : (_hipMechanicalParent.UseInertialized ? Vector3.zero : _rawHipConnectedAnchorLocal);
            return parent.transform.TransformPoint(local);
        }

        private Vector3 GetRawPelvisHipAnchorWorld()
        {
            if (_dynamicParentActive && _insidePhysicsStep) return _physicalParent.SettledPoint(_rawHipConnectedAnchorLocal);
            return _pelvisBody != null
                ? _pelvisBody.transform.TransformPoint(_rawHipConnectedAnchorLocal)
                : _sampledUpperPosition;
        }

        private Vector3 GetCommandedRawPelvisHipAnchorWorld()
        {
            // Reconstruct the exact Hip-anchor world target that the v0.2.45 raw connected Pelvis will present to
            // PhysX in this same physics step. This keeps translation identical across A/B; only parent rotation changes.
            if (!_insidePhysicsStep || !_parentCommand.Matches(_runtimeStep))
                throw new InvalidOperationException("LEG_PARENT_COMMAND_OUTSIDE_OWNING_STEP");
            return _dynamicParentActive ? _physicalParent.SettledPoint(_rawHipConnectedAnchorLocal) :
                _parentCommand.Position + _parentCommand.Rotation * _rawHipConnectedAnchorLocal;
        }

        private Rigidbody GetActiveHipMechanicalParentBody()
        {
            return _dynamicParentActive ? _pelvisBody : _hipMechanicalParent.GetConnectedBody(_pelvisBody);
        }

        private void ApplyHipMechanicalParentConnection()
        {
            if (_hipJoint == null)
                return;
            Rigidbody parent = GetActiveHipMechanicalParentBody();
            if (parent == null)
                throw new InvalidOperationException("HIP_MECHANICAL_PARENT_BODY_MISSING");

            _hipJoint.connectedBody = parent;
            if (_hipMechanicalParent.UseInertialized)
            {
                // Proxy origin is the Hip anchor itself, so rotation cannot orbit the anchor through a pelvis->hip lever.
                _hipJoint.connectedAnchor = Vector3.zero;
            }
            else
            {
                // Exact v0.2.45 baseline geometry.
                _hipJoint.connectedAnchor = _rawHipConnectedAnchorLocal;
            }
            _hipMechanicalParent.CopyToReport(_report, _hipJoint);
        }

        private void ConfigureHipJoint()
        {
            _hipJoint.autoConfigureConnectedAnchor = false;
            _hipJoint.anchor = Vector3.zero;
            ApplyHipMechanicalParentConnection();
            _hipJoint.enableCollision = false;
            // v0.2.58 single-variable causal A/B. Default arm is byte-for-behavior production ON.
            // Preview-only CharacterJoint; production removes this joint. Closed v0.2.58 OFF probe retired.
            _hipJoint.enablePreprocessing = true;

            Vector3 twistWorld = _target.UpperLeg.up;
            Vector3 swingWorld = _target.UpperLeg.right;
            _hipJoint.axis = _upperObject.transform.InverseTransformDirection(twistWorld).normalized;
            _hipJoint.swingAxis = _upperObject.transform.InverseTransformDirection(swingWorld).normalized;

            float wide = Mathf.Clamp(_profile.WideJointLimitDegrees, 120f, 175f);
            SoftJointLimit low = _hipJoint.lowTwistLimit;
            low.limit = -wide;
            _hipJoint.lowTwistLimit = low;
            SoftJointLimit high = _hipJoint.highTwistLimit;
            high.limit = wide;
            _hipJoint.highTwistLimit = high;
            SoftJointLimit swing1 = _hipJoint.swing1Limit;
            swing1.limit = wide;
            _hipJoint.swing1Limit = swing1;
            SoftJointLimit swing2 = _hipJoint.swing2Limit;
            swing2.limit = wide;
            _hipJoint.swing2Limit = swing2;
        }

        private void ConfigureKneeJoint()
        {
            _kneeJoint.connectedBody = _upperBody;
            _kneeJoint.autoConfigureConnectedAnchor = false;
            _kneeJoint.anchor = _kneeChildAnchorLowerLocal;
            _kneeJoint.connectedAnchor = _kneeParentAnchorUpperLocal;
            _kneeJoint.enableCollision = false;
            _kneeJoint.enablePreprocessing = true;
            ApplyKneeCharacterJointAxes(_kneeFlexionAxisLowerLocal);

            // v0.2.23 proved that engaged hard 1-DOF angular constraints recreate the severe Hip JS2->JT coupling.
            // Production therefore keeps PhysX limits wide and lets ARL software torque shape the anatomical off-axis DOFs.
            float wide = Mathf.Clamp(_profile.WideJointLimitDegrees, 120f, 175f);
            SetCharacterJointLimits(_kneeJoint, wide, wide, wide);
            CopyKneeConstraintBackendToReport();
        }

        private void ApplyKneeCharacterJointAxes(Vector3 flexionAxisLowerLocal)
        {
            Vector3 twist = flexionAxisLowerLocal.sqrMagnitude > 0.0001f ? flexionAxisLowerLocal.normalized : Vector3.right;
            _kneeFlexionAxisLowerLocal = twist;
            if (_kneeJoint == null) return;
            Vector3 swing = Vector3.Cross(twist, Vector3.up);
            if (swing.sqrMagnitude < 0.0001f) swing = Vector3.Cross(twist, Vector3.forward);
            if (swing.sqrMagnitude < 0.0001f) swing = Vector3.up;
            swing.Normalize();
            _kneeJoint.axis = twist;
            _kneeJoint.swingAxis = swing;
        }

        private static void SetCharacterJointLimits(CharacterJoint joint, float twistDegrees, float swing1Degrees, float swing2Degrees)
        {
            SoftJointLimit low = joint.lowTwistLimit; low.limit = -Mathf.Abs(twistDegrees); joint.lowTwistLimit = low;
            SoftJointLimit high = joint.highTwistLimit; high.limit = Mathf.Abs(twistDegrees); joint.highTwistLimit = high;
            SoftJointLimit swing1 = joint.swing1Limit; swing1.limit = Mathf.Abs(swing1Degrees); joint.swing1Limit = swing1;
            SoftJointLimit swing2 = joint.swing2Limit; swing2.limit = Mathf.Abs(swing2Degrees); joint.swing2Limit = swing2;
        }

        private bool HipFeedforwardGoalBridgeActive => _dynamicParentActive && _torso.Report != null && _torso.ControlPolicy.UseDynamicTorsoRecovery &&
            (_hipSourceReferenceShockAbsorber.Active || _hipReferenceGovernor.Active || _hipCompositeReferenceShockAbsorber.OutputAppliedThisStep);

        private bool UpdateLegAngularAccelerationFeedforward()
        {
            // The exemption is never a raw-input fallback. Reject a missing, stale
            // identity or wrong arm before selecting any feedforward contribution.
            if (_kneeReference.Selected != KneeReferenceArmSelected ||
                _kneePlannedFeedforwardSelected != KneePlannedFeedforwardArmSelected ||
                (_kneePlannedFeedforwardSelected && !_kneeReference.Selected) ||
                (_kneeReference.Selected && (!PelvisRunScopes.HasHipCoupling(_dynamicScope) ||
                    !_kneeReference.Current.Matches(_runtimeStep, _runtimeSource)))) return false;
            Vector3 hipTargetRelativeAWorld = _hipNormalReference.Current.Applied ? _hipNormalReference.Current.A :
                GetHipDerivativeParentRotation() * _sampledTargetHipRelativeAngularAccelerationLocal;
            Vector3 flexionAxisWorld = _lowerBody.transform.TransformDirection(_kneeFlexionAxisLowerLocal);
            if (flexionAxisWorld.sqrMagnitude < 0.0001f)
                return false;
            flexionAxisWorld.Normalize();
            Vector3 kneeTargetRelativeAWorld = GetKneeDerivativeParentRotation() * KneeTargetRelativeA;
            float kneeTargetAxisA = Vector3.Dot(kneeTargetRelativeAWorld, flexionAxisWorld);

            bool ok = _legAngularAccelerationFeedforward.Step(
                hipTargetRelativeAWorld,
                kneeTargetAxisA,
                _torso.ControlPhase,
                _torso.ControlRecoveryActive,
                _targetSampleTiming.AgeSeconds,
                _profile.LegAngularAccelerationFeedforwardGain,
                _profile.LegAngularAccelerationFeedforwardMaxAcceleration,
                _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds,
                _hipAuthority.PhysicalRecoveryPending,
                _softKneeAuthority.PhysicalRecoveryPending,
                (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization),
                HipFeedforwardGoalBridgeActive,
                _dynamicParentActive && _torso.Report != null && _torso.ControlPolicy.HipGoalGateOnlyHip,
                _kneePlannedFeedforwardSelected);
            CaptureLegTurnHipFeedforward(ok, hipTargetRelativeAWorld, kneeTargetAxisA);
            CopyLegAngularAccelerationFeedforwardToReport();
            return ok;
        }

        private void ApplyHipPd()
        {
            _lastHipAppliedAngularAccelerationWorld = Vector3.zero;
            bool inertialized = _hipControlFrame.Active && _profile.LegHipControlFrameInertialization;
            Quaternion controlParentRotation = inertialized ? _hipControlFrame.Rotation : _pelvisBody.rotation;
            Quaternion currentRelative = Quaternion.Inverse(controlParentRotation) * _upperBody.rotation;
            Quaternion errorRelative = _controlTargetHipRelative * Quaternion.Inverse(currentRelative);
            Vector3 errorLocal = QuaternionToAngularVector(errorRelative);
            _report.HipControlRotationErrorDegrees = errorLocal.magnitude * Mathf.Rad2Deg;
            Vector3 errorWorld = controlParentRotation * errorLocal;

            Vector3 targetRelativeW = controlParentRotation * _controlTargetHipRelativeAngularVelocityLocal;
            Vector3 bodyRelativeW;
            if (inertialized)
            {
                // Relative angular velocity in the same virtual parent frame used by P. This avoids mixing a smoothed P frame
                // with the real Pelvis' instantaneous relative-W estimator during the exact window we are testing.
                bodyRelativeW = _measuredUpperAngularVelocity - _hipControlFrame.AngularVelocityWorld;
            }
            else
            {
                // Exact v0.2.29 baseline path outside Source Turn / frame catch-up.
                bodyRelativeW = _pelvisBody.rotation * _fixedBodyHipRelativeAngularVelocityLocal;
            }
            Vector3 velocityError = targetRelativeW - bodyRelativeW;
            Vector3 targetRelativeA = _hipNormalReference.Current.Applied ? _hipNormalReference.Current.A :
                controlParentRotation * _sampledTargetHipRelativeAngularAccelerationLocal;

            // v0.3.52 retirement: the v0.3.50-v0.3.51 Hip bandwidth/selective-swing route is closed.
            // Production now directly uses the frozen Hip55/12 contributions again; there is no runtime gain selector.
            Vector3 pContribution = errorWorld * _profile.LegHipAngularSpring;
            Vector3 effectiveDContribution = velocityError * _profile.LegHipAngularDamper;

            // v0.2.8 production target-A feedforward remains directly composed into Hip PD. v0.3.36 causally
            // re-verified that suppressing this term in NORMAL_TRANSITION worsens phase response; it remains frozen ON.
            Vector3 effectiveHipAff = _legAngularAccelerationFeedforward.HipAppliedWorld;
            Vector3 relativeControlRaw = pContribution + effectiveDContribution + effectiveHipAff;
            _report.AngularDynamicsMode = CurrentAngularKinematicsMode;
            _report.AngularDynamicsValid = IsFinite(targetRelativeW) && IsFinite(bodyRelativeW) && IsFinite(targetRelativeA);
            _report.HipTargetRelativeAngularSpeed = targetRelativeW.magnitude;
            _report.HipBodyRelativeAngularSpeed = bodyRelativeW.magnitude;
            _report.HipAngularVelocityErrorMagnitude = velocityError.magnitude;
            _report.HipTargetRelativeAngularAccelerationMagnitude = targetRelativeA.magnitude;
            _report.HipPdPContributionMagnitude = pContribution.magnitude;
            _report.HipPdDContributionMagnitude = effectiveDContribution.magnitude;
            float hipMaxA = _hipAuthority.CurrentMaxAngularAcceleration;

            // v0.2.64 causal B arm: v0.2.63 proved controller direction was correct during runaway, but the
            // physical body could remain strongly AWAY for many consecutive recovery samples. Add ONLY an axial
            // D-like braking term while that condition is present. A arm returns Vector3.zero and preserves the
            // exact v0.2.63 path. Governor lead/P/A-FF/transport/Joint are untouched.
            Quaternion rawPhysicalHipRelative = Quaternion.Inverse(_pelvisBody.rotation) * _upperBody.rotation;
            Quaternion rawPhysicalHipErrorRelative = _sourceControlTargetHipRelative * Quaternion.Inverse(rawPhysicalHipRelative);
            Vector3 rawPhysicalHipErrorWorld = _pelvisBody.rotation * QuaternionToAngularVector(rawPhysicalHipErrorRelative);
            Vector3 rawTargetHipRelativeWWorld = _pelvisBody.rotation * _sourceControlTargetHipRelativeAngularVelocityLocal;
            Vector3 rawBodyHipRelativeWWorld = _pelvisBody.rotation * _fixedBodyHipRelativeAngularVelocityLocal;
            Vector3 rawPhysicalVelocityErrorWorld = rawTargetHipRelativeWWorld - rawBodyHipRelativeWWorld;
            Vector3 rawPhysicalBodyMinusTargetWWorld = -rawPhysicalVelocityErrorWorld;
            float effectiveHipMaxA;
            Vector3 supplementalHipCapture;
            if (_hipVectorCaptureDamping.Enabled)
            {
                // v0.2.67-v0.2.69 vector capture: replace ONLY the v0.2.64 supplemental term. The v0.2.64 module remains the
                // required fixed baseline but is deliberately not composed on top, so each A/B changes one control concept.
                bool sourceActive = _torso.Report != null && _torso.ControlPhase.SourceTurnEventActive;
                int sourceId = _torso.Report != null ? _torso.Report.SourceTurnCurrentEventId : 0;
                // Step has already published this authority state before ApplyHipPd;
                // consume the owner value while the report remains diagnostic output.
                supplementalHipCapture = _hipVectorCaptureDamping.ComposeSupplementalCapture(
                    _profile, _report, _hipAuthority.PhysicalRecoveryPending, sourceActive, sourceId, rawPhysicalHipErrorWorld,
                    rawPhysicalBodyMinusTargetWWorld, hipMaxA, out effectiveHipMaxA);
                _hipMomentumRecoveryBraking.CopyToReport(_report);
            }
            else
            {
                supplementalHipCapture = _hipMomentumRecoveryBraking.ComposeSupplementalBraking(
                    _profile, _report, rawPhysicalHipErrorWorld, rawPhysicalVelocityErrorWorld,
                    hipMaxA, out effectiveHipMaxA);
                _hipVectorCaptureDamping.CopyToReport(_report);
            }
            relativeControlRaw += supplementalHipCapture;

            // v0.2.88 explicit phase-owned parent transport.
            // - Strict Source/frame recovery keeps frozen v0.2.48 transport.
            // - Post-frame physical recovery keeps the v0.2.87-validated common-mode support in BOTH causal arms.
            // - TRUE Healthy Normal is the only A/B variable: A=OFF, B=upcoming mechanical-parent A ON.
            // All branches remain mutually exclusive and the final SUM still uses the unchanged Hip 40/80/80 MaxA.
            Vector3 parentTransportCommandWorld = inertialized
                ? _hipControlFrame.StrictMechanicalAngularAccelerationWorld
                : _hipMechanicalParent.CommandedProxyAngularAcceleration;
            bool sourceTurnActiveForParentTransport = _torso.Report != null && _torso.ControlPhase.SourceTurnEventActive;
            Vector3 acceleration = _hipMechanicalParentTransport.ComposeAndClamp(
                _hipMechanicalParent.UseInertialized,
                inertialized,
                sourceTurnActiveForParentTransport,
                _hipAuthority.PhysicalRecoveryPending,
                _hipAuthority.TrackingHealthUnhealthy,
                parentTransportCommandWorld,
                relativeControlRaw,
                effectiveHipMaxA,
                out bool clamped);

            // v0.2.69 B single-variable authority test: after the unchanged v0.2.68 composition/clamp, guarantee a
            // 40rad/s^2 corrective radial component only in high-error AWAY capture. The vector norm remains within the
            // same effectiveHipMaxA (normally recovery/source ceiling 80). No extra Rigidbody actuator is introduced.
            Vector3 beforeHipBrakeFloor = acceleration;
            if (_hipVectorCaptureDamping.Enabled)
                acceleration = _hipVectorCaptureDamping.ApplyFinalHighErrorAwayBrakeFloor(acceleration, effectiveHipMaxA, _report);

            Vector3 controlRaw = _hipMechanicalParentTransport.CombinedRawWorld;

            // Hip bandwidth experiments are retired; the downstream clamp/transport/actuator path remains the frozen production path.
            _hipMechanicalParentTransport.CopyToReport(_report);
            _hipAuthority.RecordResult(_profile, clamped, _report);
            _report.HipAccelerationClamped = clamped;
            if (clamped) _report.HipClampCount++;

            // v0.2.81 causal A/B: upstream desired world alpha is identical. A keeps historical Acceleration mode;
            // B maps that same alpha through validated inertia + omega x (I*omega) and uses physical Force torque.
            // angularDrag is intentionally left identical/uncompensated in both arms.
            Vector3 hipActuatorVector = _hipAngularActuator.Compose(_upperBody, acceleration, _report);
            if (!_hipAngularActuator.CurrentValid)
            {
                AbortDrive("HIP_ANGULAR_ACTUATOR_INERTIA_SPACE_INVALID");
                return;
            }

            if (_dynamicParentActive && PelvisRunScopes.HasHipCoupling(_dynamicScope))
            {
                if (!_hipAngularActuator.Enabled || !_hipAnchorPair.StageHipTorque(_runtimeStep, _fixedUpdateSequence, hipActuatorVector, acceleration,
                    relativeControlRaw, _hipMechanicalParentTransport.SelectedTransportWorld, beforeHipBrakeFloor, effectiveHipMaxA))
                { AbortDrive("HIP_PAIR_TORQUE_REQUEST_INVALID"); return; }
                CaptureLegTurnHip(inertialized, controlParentRotation, targetRelativeW,
                    relativeControlRaw, controlRaw, acceleration, effectiveHipMaxA, clamped);
                // Request provenance is retained, but not labelled committed before the native writes.
                _legTurnCurrent.HipCommitted = false;
                HipGoalPathEvidence path = _legTurnCurrent.HipGoalPath;
                path.Committed = false; _legTurnCurrent.HipGoalPath = path;
                return;
            }
            _lastHipAppliedAngularAccelerationWorld = acceleration;
            if (_hipAngularActuator.Enabled)
                _upperBody.AddTorque(hipActuatorVector, ForceMode.Force);
            else
                _upperBody.AddTorque(acceleration, ForceMode.Acceleration);
            CaptureDynamicHipCommand(hipActuatorVector);
            CaptureLegTurnHip(inertialized, controlParentRotation, targetRelativeW,
                relativeControlRaw, controlRaw, acceleration, effectiveHipMaxA, clamped);
        }

        private void ApplyKneePd()
        {
            Quaternion currentRelative = Quaternion.Inverse(_upperBody.rotation) * _lowerBody.rotation;
            Quaternion errorRelative = _controlTargetKneeRelative * Quaternion.Inverse(currentRelative);
            Vector3 errorLocal = QuaternionToAngularVector(errorRelative);
            Vector3 errorWorld = _upperBody.rotation * errorLocal;

            Vector3 flexionAxisWorld = GetCurrentKneeFlexionAxisWorld();
            if (flexionAxisWorld.sqrMagnitude < 0.0001f)
            {
                AbortDrive("INVALID_KNEE_FLEXION_AXIS_WORLD");
                return;
            }
            flexionAxisWorld.Normalize();
            float angleError = Vector3.Dot(errorWorld, flexionAxisWorld);
            Vector3 offAxisErrorWorld = errorWorld - flexionAxisWorld * angleError;
            _report.KneeControlAxisErrorDegrees = Mathf.Abs(angleError) * Mathf.Rad2Deg;

            Quaternion derivativeParentRotation = GetKneeDerivativeParentRotation();
            Vector3 targetRelativeW = derivativeParentRotation * _controlTargetKneeRelativeAngularVelocityLocal;
            Vector3 bodyRelativeW = _upperBody.rotation * _fixedBodyKneeRelativeAngularVelocityLocal;
            Vector3 velocityErrorWorld = targetRelativeW - bodyRelativeW;
            float targetAxisW = Vector3.Dot(targetRelativeW, flexionAxisWorld);
            float bodyAxisW = Vector3.Dot(bodyRelativeW, flexionAxisWorld);
            float velocityError = Vector3.Dot(velocityErrorWorld, flexionAxisWorld);
            Vector3 offAxisVelocityErrorWorld = velocityErrorWorld - flexionAxisWorld * velocityError;
            Vector3 targetRelativeA = derivativeParentRotation * KneeTargetRelativeA;
            float targetAxisA = Vector3.Dot(targetRelativeA, flexionAxisWorld);

            // V041180 both Hip arms consume the same continuous relative R/W/A;
            // legacy scopes retain raw filtered/held input. Raw W remains reported.
            float baselinePContribution = angleError * _profile.LegKneeAngularSpring;
            float baselineDContribution = velocityError * _profile.LegKneeAngularDamper;

            // v0.3.50 maintenance: v0.3.42-v0.3.44 Knee gain experiment remains closed/rejected. Production Knee feedback is
            // restored to the frozen 45/9 path directly; no runtime Knee bandwidth selector remains active.
            float pContribution = baselinePContribution;
            float dContribution = baselineDContribution;
            float raw = pContribution + dContribution;

            // Frozen v0.2.78 allocation remains unchanged: production P/D + existing A-FF share the same +/-40 clamp.
            float flexionAccel = _kneeFlexionAllocation.Compose(
                raw,
                _legAngularAccelerationFeedforward.KneeAppliedSigned,
                _profile.LegKneeMaxAngularAcceleration,
                _report,
                out float effectiveFeedforwardSigned,
                out _,
                out bool flexionClamped);
            _report.KneeAccelerationClamped = flexionClamped;
            if (flexionClamped) _report.KneeClampCount++;

            // Same target-current quaternion and velocity error, but only the off-flexion component enters this software PD.
            Vector3 softPWorld = offAxisErrorWorld * _profile.LegKneeSoftOffAxisAngularSpring;
            Vector3 softDWorld = offAxisVelocityErrorWorld * _profile.LegKneeSoftOffAxisAngularDamper;
            Vector3 softFeedbackRawWorld = softPWorld + softDWorld;

            // v0.3.60 predictive reaction-FF experiment is retired. Later v0.3.61-v0.3.65 work localized the
            // important distal solver/topology issue and removed the Ankle CharacterJoint, so the frozen Knee soft lane
            // is again simply its validated feedback command under the unchanged phase-aware authority ceiling.
            float softAuthorityMaxA = _softKneeAuthority.CurrentMaxAngularAcceleration;
            Vector3 softRawWorld = softFeedbackRawWorld;
            Vector3 softAppliedWorld = ClampMagnitude(softRawWorld, softAuthorityMaxA, out bool softClamped);
            if (!IsFinite(softAppliedWorld))
            {
                AbortDrive("NON_FINITE_KNEE_SOFT_OFFAXIS_COMMAND");
                return;
            }
            if (softClamped) _report.KneeSoftOffAxisClampCount++;

            Vector3 flexionAppliedWorld = flexionAxisWorld * flexionAccel;
            Vector3 relativeKneeAppliedWorld = flexionAppliedWorld + softAppliedWorld;
            // Frozen v0.2.75 transport: compose the relative Knee command with the actual same-step
            // final Hip angular acceleration; common-mode transport is outside the relative Knee cap.
            Vector3 combinedAppliedWorld = _kneeParentAngularTransport.Compose(
                _lastHipAppliedAngularAccelerationWorld,
                relativeKneeAppliedWorld,
                _report);

            // Retain the actual relative command consumed by Foot angular attribution.
            _kneeResponseRelativeCommandWorld = relativeKneeAppliedWorld;

            _report.AngularDynamicsMode = CurrentAngularKinematicsMode;
            _report.AngularDynamicsValid = _report.AngularDynamicsValid && IsFinite(targetRelativeW) && IsFinite(bodyRelativeW) && IsFinite(targetAxisA);
            _report.KneeTargetAxisAngularVelocitySigned = targetAxisW;
            _report.KneeBodyAxisAngularVelocitySigned = bodyAxisW;
            _report.KneeAxisAngularVelocityErrorSigned = velocityError;
            _report.KneeTargetAxisAngularAccelerationSigned = targetAxisA;
            _report.KneePdPContributionSigned = pContribution;
            _report.KneePdDContributionSigned = dContribution;
            _report.KneeFlexionAppliedAngularAccelerationMagnitude = Mathf.Abs(flexionAccel);
            _report.KneeSoftOffAxisAngularVelocityErrorMagnitude = offAxisVelocityErrorWorld.magnitude;
            _report.KneeSoftOffAxisPContributionMagnitude = softPWorld.magnitude;
            _report.KneeSoftOffAxisDContributionMagnitude = softDWorld.magnitude;
            _report.KneeSoftOffAxisRawAccelerationMagnitude = softRawWorld.magnitude;
            _report.KneeSoftOffAxisAppliedAccelerationMagnitude = softAppliedWorld.magnitude;
            _report.KneeSoftOffAxisAccelerationClamped = softClamped;
            _report.KneeLowerAppliedAngularAccelerationMagnitude = combinedAppliedWorld.magnitude;

            // v0.2.10-v0.2.11 pairwise reaction experiments remain retired: production actuator is CHILD_ONLY.
            _lowerBody.AddTorque(combinedAppliedWorld, ForceMode.Acceleration);
            _legAngularAccelerationFeedforward.RecordKneeSubmittedCommand(raw, flexionAccel, _profile.LegKneeMaxAngularAcceleration);
            CaptureLegTurnKnee(targetRelativeW, relativeKneeAppliedWorld, combinedAppliedWorld, raw, effectiveFeedforwardSigned, flexionAccel);
        }

        private float GetCurrentKneeOffAxisErrorDegrees()
        {
            if (_upperBody == null || _lowerBody == null) return 0f;
            Quaternion currentKnee = Quaternion.Inverse(_upperBody.rotation) * _lowerBody.rotation;
            Quaternion errorRelative = _sampledTargetKneeRelative * Quaternion.Inverse(currentKnee);
            Vector3 errorWorld = _upperBody.rotation * QuaternionToAngularVector(errorRelative);
            Vector3 flexionAxisWorld = GetCurrentKneeFlexionAxisWorld();
            if (flexionAxisWorld.sqrMagnitude < 0.0001f) return errorWorld.magnitude * Mathf.Rad2Deg;
            flexionAxisWorld.Normalize();
            Vector3 offAxis = errorWorld - flexionAxisWorld * Vector3.Dot(errorWorld, flexionAxisWorld);
            return offAxis.magnitude * Mathf.Rad2Deg;
        }

        private void UpdateBodyRelativeAngularVelocities()
        {
            float dt = PhysicsDt;
            Quaternion hipRelative = Quaternion.Inverse(_pelvisBody.rotation) * _upperBody.rotation;
            Quaternion kneeRelative = Quaternion.Inverse(_upperBody.rotation) * _lowerBody.rotation;
            if (_hasPreviousBodyRelativeSample && IsFinite(dt) && dt > 0.0001f)
            {
                _fixedBodyHipRelativeAngularVelocityLocal = ComputeAngularVelocity(_previousBodyHipRelative, hipRelative, dt);
                _fixedBodyKneeRelativeAngularVelocityLocal = ComputeAngularVelocity(_previousBodyKneeRelative, kneeRelative, dt);
            }
            else
            {
                _fixedBodyHipRelativeAngularVelocityLocal = Vector3.zero;
                _fixedBodyKneeRelativeAngularVelocityLocal = Vector3.zero;
            }
            if (_dynamicParentActive)
            {
                _fixedBodyHipRelativeAngularVelocityLocal = Quaternion.Inverse(_physicalParent.Rotation) * (_upperBody.angularVelocity - _physicalParent.W);
                _fixedBodyKneeRelativeAngularVelocityLocal = Quaternion.Inverse(_upperBody.rotation) * (_lowerBody.angularVelocity - _upperBody.angularVelocity);
            }
            _previousBodyHipRelative = hipRelative;
            _previousBodyKneeRelative = kneeRelative;
            _hasPreviousBodyRelativeSample = true;
        }

        private void ResetBodyRelativeAngularVelocityEstimator()
        {
            _fixedBodyHipRelativeAngularVelocityLocal = Vector3.zero;
            _fixedBodyKneeRelativeAngularVelocityLocal = Vector3.zero;
            if (_pelvisBody != null && _upperBody != null && _lowerBody != null)
            {
                _previousBodyHipRelative = Quaternion.Inverse(_pelvisBody.rotation) * _upperBody.rotation;
                _previousBodyKneeRelative = Quaternion.Inverse(_upperBody.rotation) * _lowerBody.rotation;
            }
            else
            {
                _previousBodyHipRelative = Quaternion.identity;
                _previousBodyKneeRelative = Quaternion.identity;
            }
            _hasPreviousBodyRelativeSample = false;
        }

        private void SampleSourcePoseInternal(bool advanceVelocityEstimator)
        {
            if (_target.UpperLeg == null || _target.LowerLeg == null || _target.Foot == null)
                throw new InvalidOperationException("LEG_SOURCE_TRANSFORM_LOST");

            Vector3 upperPosition = _target.UpperLeg.position;
            Quaternion upperRotation = _target.UpperLeg.rotation;
            Vector3 lowerPosition = _target.LowerLeg.position;
            Quaternion lowerRotation = _target.LowerLeg.rotation;
            if (!IsFinite(upperPosition) || !PelvisParentFrame.Q(upperRotation) ||
                !IsFinite(lowerPosition) || !PelvisParentFrame.Q(lowerRotation))
                throw new InvalidOperationException("LEG_SOURCE_POSE_INVALID");

            if (advanceVelocityEstimator)
            {
                _targetRenderSampleSequence++;
                _targetRenderSampleUnscaledTime = CurrentUnscaledTime;
            }

            _sampledUpperPosition = upperPosition;
            _sampledUpperRotation = upperRotation;
            _sampledLowerPosition = lowerPosition;
            _sampledLowerRotation = lowerRotation;
            _sampledPelvisRotation = _pelvisBody != null ? _pelvisBody.rotation : Quaternion.identity;
            if (_sourcePelvisTransform == null)
                _sourcePelvisTransform = _target.UpperLeg.parent;
            _sampledSourcePelvisPosition = _sourcePelvisTransform != null ? _sourcePelvisTransform.position : _sampledUpperPosition;
            _sampledSourcePelvisRotation = _sourcePelvisTransform != null ? _sourcePelvisTransform.rotation : _torso.SampledPelvisTargetRotation;

            // Torso proxy 的世界姿态包含骨骼->代理框架校准；Hip target 必须使用同一目标父体框架，
            // 不能直接拿源 Hips bone rotation 与代理 UpperLeg 混算。
            Quaternion sourcePelvisRotation = _torso.SampledPelvisTargetRotation;
            if (!IsFinite(sourcePelvisRotation))
                sourcePelvisRotation = _sampledPelvisRotation;
            Quaternion hipRelative = Quaternion.Inverse(sourcePelvisRotation) * _sampledUpperRotation;
            Quaternion kneeRelative = Quaternion.Inverse(_sampledUpperRotation) * _sampledLowerRotation;
            if (advanceVelocityEstimator)
            {
                float dt = _runtimeSource.Dt;
                if (dt >= 0.001f && dt <= 0.20f && _hasPreviousTargetSample && _hasPreviousSourceMotionSample)
                {
                    // v0.2.1: 与现有 Render sample-hold estimator 共用真实新样本 dt；只为诊断 parent transport。
                    Vector3 pelvisLinearRaw = (_sampledSourcePelvisPosition - _previousSourcePelvisPosition) / dt;
                    Vector3 pelvisAngularRaw = ComputeAngularVelocity(_previousSourcePelvisRotation, _sampledSourcePelvisRotation, dt);
                    Vector3 upperLinearRaw = (_sampledUpperPosition - _previousSourceUpperPosition) / dt;
                    Vector3 upperAngularRaw = ComputeAngularVelocity(_previousSourceUpperRotation, _sampledUpperRotation, dt);
                    Vector3 lowerLinearRaw = (_sampledLowerPosition - _previousSourceLowerPosition) / dt;
                    Vector3 lowerAngularRaw = ComputeAngularVelocity(_previousSourceLowerRotation, _sampledLowerRotation, dt);

                    RecordLegTurnRawSource(upperAngularRaw, lowerAngularRaw, dt);

                    float pelvisRotationDeltaRawDeg = Quaternion.Angle(_previousSourcePelvisRotation, _sampledSourcePelvisRotation);
                    Vector3 pelvisAngularAccelerationRaw = _hasPreviousCalibrationPelvisAngularVelocityRaw
                        ? (pelvisAngularRaw - _previousCalibrationPelvisAngularVelocityRaw) / dt
                        : Vector3.zero;

                    _previousCalibrationPelvisAngularVelocityRaw = pelvisAngularRaw;
                    _hasPreviousCalibrationPelvisAngularVelocityRaw = true;
                    bool calibrationSourceTurnLike =
                        pelvisRotationDeltaRawDeg >= _profile.SourceTurnEventRotationDeltaDegrees ||
                        pelvisAngularRaw.magnitude >= _profile.SourceTurnEventAngularSpeedThreshold ||
                        (pelvisAngularAccelerationRaw.magnitude >= _profile.SourceTurnEventAngularAccelerationThreshold &&
                         (pelvisAngularRaw.magnitude >= 0.75f || pelvisRotationDeltaRawDeg >= 2.0f));

                    pelvisLinearRaw = ClampMagnitudeSimple(pelvisLinearRaw, _profile.MaxLinearVelocity);
                    upperLinearRaw = ClampMagnitudeSimple(upperLinearRaw, _profile.MaxLinearVelocity);
                    lowerLinearRaw = ClampMagnitudeSimple(lowerLinearRaw, _profile.MaxLinearVelocity);
                    pelvisAngularRaw = ClampMagnitudeSimple(pelvisAngularRaw, _profile.MaxAngularVelocity);
                    upperAngularRaw = ClampMagnitudeSimple(upperAngularRaw, _profile.MaxAngularVelocity);
                    lowerAngularRaw = ClampMagnitudeSimple(lowerAngularRaw, _profile.MaxAngularVelocity);

                    float motionBlend = _profile.TargetVelocityBlend;
                    _sampledSourcePelvisLinearVelocity = Vector3.Lerp(_sampledSourcePelvisLinearVelocity, pelvisLinearRaw, motionBlend);
                    _sampledSourcePelvisAngularVelocity = Vector3.Lerp(_sampledSourcePelvisAngularVelocity, pelvisAngularRaw, motionBlend);
                    _sampledUpperLinearVelocity = Vector3.Lerp(_sampledUpperLinearVelocity, upperLinearRaw, motionBlend);
                    _sampledUpperAngularVelocity = Vector3.Lerp(_sampledUpperAngularVelocity, upperAngularRaw, motionBlend);
                    _sampledLowerLinearVelocity = Vector3.Lerp(_sampledLowerLinearVelocity, lowerLinearRaw, motionBlend);
                    _sampledLowerAngularVelocity = Vector3.Lerp(_sampledLowerAngularVelocity, lowerAngularRaw, motionBlend);

                    Vector3 hipRaw = ComputeAngularVelocity(_previousTargetHipRelative, hipRelative, dt);
                    Vector3 kneeRaw = ComputeAngularVelocity(_previousTargetKneeRelative, kneeRelative, dt);
                    hipRaw = ClampMagnitudeSimple(hipRaw, _profile.MaxAngularVelocity);
                    kneeRaw = ClampMagnitudeSimple(kneeRaw, _profile.MaxAngularVelocity);

                    // v0.2.2 SOURCE-ONLY calibration: kneeRaw is expressed in source UpperLeg local frame.
                    // Convert through world into the instantaneous source LowerLeg local frame. A true hinge axis
                    // remains invariant under flex/extend, while Σ(w w^T) is sign-insensitive across both directions.
                    Vector3 kneeRawWorld = _sampledUpperRotation * kneeRaw;
                    Vector3 kneeRawLowerLocal = Quaternion.Inverse(_sampledLowerRotation) * kneeRawWorld;
                    _kneeAxisCalibrator.Observe(
                        kneeRawLowerLocal,
                        _torso.ControlPhase,
                        _torso.ControlRecoveryActive,
                        calibrationSourceTurnLike,
                        _profile.SourceTurnContextPostSeconds);

                    float blend = _profile.TargetVelocityBlend;
                    _sampledTargetHipRelativeAngularVelocityLocal = Vector3.Lerp(
                        _sampledTargetHipRelativeAngularVelocityLocal, hipRaw, blend);
                    _sampledTargetKneeRelativeAngularVelocityLocal = Vector3.Lerp(
                        _sampledTargetKneeRelativeAngularVelocityLocal, kneeRaw, blend);

                    // v0.2.4: acceleration is derived only when a real new Render/LateUpdate sample arrives.
                    // Never finite-difference a held render value in FixedUpdate; that would recreate the v0.1.8 aliasing failure mode.
                    if (_hasPreviousFilteredTargetVelocitySample)
                    {
                        _sampledTargetHipRelativeAngularAccelerationLocal =
                            (_sampledTargetHipRelativeAngularVelocityLocal - _previousFilteredTargetHipRelativeAngularVelocityLocal) / dt;
                        _sampledTargetKneeRelativeAngularAccelerationLocal =
                            (_sampledTargetKneeRelativeAngularVelocityLocal - _previousFilteredTargetKneeRelativeAngularVelocityLocal) / dt;
                    }
                    else
                    {
                        _sampledTargetHipRelativeAngularAccelerationLocal = Vector3.zero;
                        _sampledTargetKneeRelativeAngularAccelerationLocal = Vector3.zero;
                    }
                    _previousFilteredTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
                    _previousFilteredTargetKneeRelativeAngularVelocityLocal = _sampledTargetKneeRelativeAngularVelocityLocal;
                    _hasPreviousFilteredTargetVelocitySample = true;
                }
                else if (!_hasPreviousTargetSample)
                {
                    _sampledTargetHipRelativeAngularVelocityLocal = Vector3.zero;
                    _sampledTargetKneeRelativeAngularVelocityLocal = Vector3.zero;
                    _sampledTargetHipRelativeAngularAccelerationLocal = Vector3.zero;
                    _sampledTargetKneeRelativeAngularAccelerationLocal = Vector3.zero;
                    _previousFilteredTargetHipRelativeAngularVelocityLocal = Vector3.zero;
                    _previousFilteredTargetKneeRelativeAngularVelocityLocal = Vector3.zero;
                    _hasPreviousFilteredTargetVelocitySample = false;
                    _sampledSourcePelvisLinearVelocity = Vector3.zero;
                    _sampledSourcePelvisAngularVelocity = Vector3.zero;
                    _sampledUpperLinearVelocity = Vector3.zero;
                    _sampledUpperAngularVelocity = Vector3.zero;
                    _sampledLowerLinearVelocity = Vector3.zero;
                    _sampledLowerAngularVelocity = Vector3.zero;
                }
                else
                {
                    // Invalid/very long Render dt: do not carry a stale angular-acceleration sample into later FixedUpdates.
                    _sampledTargetHipRelativeAngularAccelerationLocal = Vector3.zero;
                    _sampledTargetKneeRelativeAngularAccelerationLocal = Vector3.zero;
                }

                _previousSourcePelvisPosition = _sampledSourcePelvisPosition;
                _previousSourcePelvisRotation = _sampledSourcePelvisRotation;
                _previousSourceUpperPosition = _sampledUpperPosition;
                _previousSourceUpperRotation = _sampledUpperRotation;
                _previousSourceLowerPosition = _sampledLowerPosition;
                _previousSourceLowerRotation = _sampledLowerRotation;
                _hasPreviousSourceMotionSample = true;

                _previousTargetHipRelative = hipRelative;
                _previousTargetKneeRelative = kneeRelative;
                _hasPreviousTargetSample = true;
            }

            _sampledTargetHipRelative = hipRelative;
            _sampledTargetKneeRelative = kneeRelative;
            if (advanceVelocityEstimator)
            {
                _hipReferenceGovernor.ObserveRenderPacket(hipRelative, _targetRenderSampleSequence, _targetRenderSampleUnscaledTime);
                _hipSourceReferenceShockAbsorber.ObserveRenderPacket(hipRelative, _targetRenderSampleSequence, _profile.SourceTurnEventRotationDeltaDegrees);
            }
            if (!_hipSourceReferenceShockAbsorber.Enabled)
            {
                _sourceControlTargetHipRelative = hipRelative;
                _sourceControlTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
            }
            _controlTargetHipRelative = hipRelative;
            _controlTargetKneeRelative = kneeRelative;
            _controlTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
            _controlTargetKneeRelativeAngularVelocityLocal = _sampledTargetKneeRelativeAngularVelocityLocal;

            if (_footFoundation != null && _footFoundation.IsBuilt)
                _footFoundation.SampleSourcePose(_runtimeSource, _targetRenderSampleSequence, _targetRenderSampleUnscaledTime, advanceVelocityEstimator);
        }

        private void UpdatePhaseTimingBeforeBodyMeasurement()
        {
            bool fresh = _targetSampleTiming.ObserveSourceSequence(_targetRenderSampleSequence);
            float age = _targetRenderSampleSequence > 0
                ? Mathf.Max(0f, CurrentUnscaledTime - _targetRenderSampleUnscaledTime)
                : 0f;
            _targetSampleTiming.PublishBeforeBodyMeasurement(_fixedUpdateSequence, _targetRenderSampleSequence, fresh, age);
            _report.FixedUpdateSequence = _targetSampleTiming.FixedSequence;
            _report.TargetRenderSampleSequence = _targetSampleTiming.SourceSequence;
            _report.TargetRenderSampleFreshAtFixedStep = _targetSampleTiming.Fresh;
            _report.TargetRenderSampleAgeSeconds = _targetSampleTiming.AgeSeconds;
            _report.FixedStepsSinceTargetRenderSample = _targetSampleTiming.HeldDepth;
            // Body pair fields are finalized by UpdateBodyRelativeAngularVelocities later in this FixedUpdate.
        }

        private void FollowSourceKinematicPreview()
        {
            if (_upperBody == null || _lowerBody == null)
                return;
            _upperBody.isKinematic = true;
            _lowerBody.isKinematic = true;
            _upperBody.useGravity = false;
            _lowerBody.useGravity = false;
            _upperBody.position = _sampledUpperPosition;
            _upperBody.rotation = _sampledUpperRotation;
            _lowerBody.position = _sampledLowerPosition;
            _lowerBody.rotation = _sampledLowerRotation;
            _upperBody.velocity = Vector3.zero;
            _upperBody.angularVelocity = Vector3.zero;
            _lowerBody.velocity = Vector3.zero;
            _lowerBody.angularVelocity = Vector3.zero;
            _footFoundation?.FollowSourceKinematic();
            if (_hipMechanicalParent.Built)
                _hipMechanicalParent.ResetPose(GetRawPelvisHipAnchorWorld(), _pelvisBody.rotation);
            ResetBodyRelativeAngularVelocityEstimator();
            ResetMeasuredTransportKinematics();
            _report.AngularDynamicsValid = false;
            UpdateReport();
            UpdateAttributionMetrics();
        }

        private void UpdateMeasuredTransportKinematics()
        {
            float dt = PhysicsDt;
            Vector3 pelvisPosition = _pelvisBody.position;
            Quaternion pelvisRotation = _pelvisBody.rotation;
            Vector3 upperPosition = _upperBody.position;
            Quaternion upperRotation = _upperBody.rotation;
            Vector3 lowerPosition = _lowerBody.position;
            Quaternion lowerRotation = _lowerBody.rotation;

            _measuredTransportValidCurrent = _hasPreviousMeasuredMotionSample && IsFinite(dt) && dt > 0.0001f;
            if (_measuredTransportValidCurrent)
            {
                _measuredPelvisLinearVelocity = (pelvisPosition - _previousMeasuredPelvisPosition) / dt;
                _measuredPelvisAngularVelocity = ComputeAngularVelocity(_previousMeasuredPelvisRotation, pelvisRotation, dt);
                _measuredUpperLinearVelocity = (upperPosition - _previousMeasuredUpperPosition) / dt;
                _measuredUpperAngularVelocity = ComputeAngularVelocity(_previousMeasuredUpperRotation, upperRotation, dt);
                _measuredLowerLinearVelocity = (lowerPosition - _previousMeasuredLowerPosition) / dt;
                _measuredLowerAngularVelocity = ComputeAngularVelocity(_previousMeasuredLowerRotation, lowerRotation, dt);
            }
            else
            {
                _measuredPelvisLinearVelocity = Vector3.zero;
                _measuredPelvisAngularVelocity = Vector3.zero;
                _measuredUpperLinearVelocity = Vector3.zero;
                _measuredUpperAngularVelocity = Vector3.zero;
                _measuredLowerLinearVelocity = Vector3.zero;
                _measuredLowerAngularVelocity = Vector3.zero;
            }

            if (_dynamicParentActive)
            {
                // Rigidbody velocity is at COM; keep this historical origin-velocity field honest.
                _measuredPelvisLinearVelocity = _physicalParent.SettledPointVelocity(Vector3.zero);
                _measuredPelvisAngularVelocity = _physicalParent.W;
                _measuredUpperAngularVelocity = _upperBody.angularVelocity;
                _measuredLowerAngularVelocity = _lowerBody.angularVelocity;
            }
            _previousMeasuredPelvisPosition = pelvisPosition;
            _previousMeasuredPelvisRotation = pelvisRotation;
            _previousMeasuredUpperPosition = upperPosition;
            _previousMeasuredUpperRotation = upperRotation;
            _previousMeasuredLowerPosition = lowerPosition;
            _previousMeasuredLowerRotation = lowerRotation;
            _hasPreviousMeasuredMotionSample = true;
        }

        private void ResetMeasuredTransportKinematics()
        {
            _measuredPelvisLinearVelocity = Vector3.zero;
            _measuredPelvisAngularVelocity = Vector3.zero;
            _measuredUpperLinearVelocity = Vector3.zero;
            _measuredUpperAngularVelocity = Vector3.zero;
            _measuredLowerLinearVelocity = Vector3.zero;
            _measuredLowerAngularVelocity = Vector3.zero;
            if (_pelvisBody != null)
            {
                _previousMeasuredPelvisPosition = _pelvisBody.position;
                _previousMeasuredPelvisRotation = _pelvisBody.rotation;
            }
            if (_upperBody != null)
            {
                _previousMeasuredUpperPosition = _upperBody.position;
                _previousMeasuredUpperRotation = _upperBody.rotation;
            }
            if (_lowerBody != null)
            {
                _previousMeasuredLowerPosition = _lowerBody.position;
                _previousMeasuredLowerRotation = _lowerBody.rotation;
            }
            _hasPreviousMeasuredMotionSample = false;
            _measuredTransportValidCurrent = false;
        }

        private float TraceLogInterval => RunLogPolicy.PeriodicInterval(_profile.TraceIntervalSeconds, RunLogPolicy.CompactLegWindows(_dynamicScope));

        private bool HasFiniteState()
        {
            if (!IsBuilt)
                return false;
            return
                IsFinite(_upperBody.position) && IsFinite(_upperBody.rotation) &&
                IsFinite(_lowerBody.position) && IsFinite(_lowerBody.rotation) &&
                (!_hipMechanicalParent.UseInertialized || (_hipMechanicalParent.Body != null && IsFinite(_hipMechanicalParent.Body.position) && IsFinite(_hipMechanicalParent.Body.rotation))) &&
                IsFinite(_upperBody.velocity) && IsFinite(_upperBody.angularVelocity) &&
                IsFinite(_lowerBody.velocity) && IsFinite(_lowerBody.angularVelocity) &&
                (!_profile.FootAnkleFoundationEnabled || (_footFoundation != null && _footFoundation.IsBuilt && _footFoundation.Report.Finite));
        }

        private static Vector3 QuaternionToAngularVector(Quaternion q)
        {
            if (!IsFinite(q)) return new Vector3(float.NaN, float.NaN, float.NaN);
            if (q.w < 0f)
            {
                q.x = -q.x;
                q.y = -q.y;
                q.z = -q.z;
                q.w = -q.w;
            }
            // atan2 preserves small rotations when round-off moves w to/above 1.
            // The ratio is scale-invariant, so near-unit products need no acos/sqrt subtraction.
            Vector3 v = new Vector3(q.x, q.y, q.z);
            float length = v.magnitude;
            return length > 0.00000001f ? v * (2f * Mathf.Atan2(length, q.w) / length) : Vector3.zero;
        }

        private static Vector3 ComputeAngularVelocity(Quaternion previous, Quaternion current, float dt)
        {
            if (dt <= 0.000001f)
                return Vector3.zero;
            return QuaternionToAngularVector(current * Quaternion.Inverse(previous)) / dt;
        }

        private static Vector3 ClampMagnitude(Vector3 value, float max, out bool clamped)
        {
            float magnitude = value.magnitude;
            clamped = magnitude > max && magnitude > 0.000001f;
            return clamped ? value * (max / magnitude) : value;
        }

        private static Vector3 ClampMagnitudeSimple(Vector3 value, float max)
        {
            float magnitude = value.magnitude;
            return magnitude > max && magnitude > 0.000001f ? value * (max / magnitude) : value;
        }

        private static bool IsFinite(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        private static bool IsFinite(Quaternion q) => IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
        private static bool IsFinite(float v) => !(float.IsNaN(v) || float.IsInfinity(v));
        private static string GetAngularAccelerationFeedforwardBoundLabel(float value)
        {
            if (Mathf.Abs(value - 20f) <= 0.001f) return "FROZEN_20";
            return "NON_FROZEN_" + F(value);
        }

        private static string F(float value) => value.ToString("G6", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
