using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Acquisition;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal enum TwoBodyMassPropertyMode
    {
        LegacyUnityDefault = 0,
        ExplicitCenterOfMassOnly = 1,
        ExplicitInertiaOnly = 2,
        ExplicitUniformBox = 3
    }

    internal enum TwoBodyCharacterJointAnchorMode
    {
        CalibratedSourceJoint = 0,
        ChestCenterOfMass = 1
    }

    internal sealed class TwoBodyShadowProxy
    {
        private sealed class IgnoredPair
        {
            public Collider A;
            public Collider B;
        }

        private readonly ActiveRagdollLabProfile _profile;
        private readonly BoneAuditReport _audit;
        private readonly List<IgnoredPair> _ignoredPairs = new List<IgnoredPair>();
        private readonly RenderToPhysicsPoseBridge _poseBridge = new RenderToPhysicsPoseBridge();
        private readonly SourceTurnDiscontinuityDiagnostics _sourceTurnDiagnostics = new SourceTurnDiscontinuityDiagnostics();
        private readonly PelvisDynamicReceiverIsolation _pelvisReceiver = new PelvisDynamicReceiverIsolation();
        private readonly PelvisLegReference _pelvisLegReference = new PelvisLegReference();
        private readonly DynamicTorsoRecoveryGate _dynamicTorsoRecovery = new DynamicTorsoRecoveryGate();
        internal string TorsoRecoverySummary => _dynamicTorsoRecovery.GetSummary(_report.UseDynamicTorsoRecovery);
        private void ResetTorsoRecoveryState()
        {
            _dynamicTorsoRecovery.ResetState();
            _report.DynamicTorsoRecovery = _dynamicTorsoRecovery.Current;
        }
        private void PublishTorsoRecoveryState()
        {
            if (!DynamicPelvisReceiverSelected || !_driveEnabled) return;
            _dynamicTorsoRecovery.Step(_runtimeStep, _insideRuntimeStep && _pelvisReceiver.ReadyForRun &&
                _pelvisReceiver.ControlErrorsValid && _report.FiniteState && _report.CharacterJointConnectedBodyMatch &&
                _report.CharacterJointAnchorModeValidated, _report.SourceTurnEventActive,
                _pelvisReceiver.ControlComErrorMeters, _pelvisReceiver.ControlPelvisErrorDegrees,
                _report.RotationErrorDegrees, _report.CharacterJointConstraintAnchorErrorMeters, _report.AnchorErrorMeters);
            _report.DynamicTorsoRecovery = _dynamicTorsoRecovery.Current;
            if (_traceEnabled) _dynamicTorsoRecovery.Record(_report.FreezeRecoveryActive);
            if (_report.UseDynamicTorsoRecovery && _pelvisReceiver.ReadyForRun &&
                !_report.DynamicTorsoRecovery.Matches(_runtimeStep))
                throw new InvalidOperationException("DYNAMIC_TORSO_RECOVERY_CURRENT_INPUT_INVALID");
        }
        private readonly StepContextCursor _contextCursor = new StepContextCursor();
        internal SourceStepContext CompletedSource { get; private set; }
        private LegParentCommandFrame _legParentCommand;
        private PhysicsStepContext _runtimeStep;
        private SourceStepContext _runtimeSource;
        private bool _insideRuntimeStep, _insideRuntimeSource;
        private float PhysicsDt => _insideRuntimeStep ? _runtimeStep.Dt : Time.fixedDeltaTime;
        private float PhysicsTime => _insideRuntimeStep ? _runtimeStep.FixedTime : Time.fixedTime;
        private float CurrentUnscaledTime => _insideRuntimeStep ? _runtimeStep.UnscaledTime :
            _insideRuntimeSource ? _runtimeSource.Time : Time.unscaledTime;
        private readonly SourceAwareFreezeValidation _freezeValidation = new SourceAwareFreezeValidation();
        private readonly TwoBodyMassPropertyMode _massPropertyMode;
        private readonly string _instanceTag;
        private readonly bool _emitLogs;
        private readonly TwoBodyCharacterJointAnchorMode _characterJointAnchorMode;
        private bool _massPropertyModeValidated;
        private bool _characterJointAnchorModeValidated;

        private GameObject _root;
        private GameObject _pelvisObject;
        private GameObject _chestObject;
        private GameObject _pelvisVisual;
        private GameObject _chestVisual;
        private Rigidbody _pelvisBody;
        private Rigidbody _chestBody;
        private BoxCollider _pelvisCollider;
        private BoxCollider _chestCollider;
        private CharacterJoint _characterJoint;
        private Material _previewMaterial;

        // Source/control anchor remains the frozen calibrated spine-point semantics.
        private Vector3 _virtualPelvisAnchorLocal;
        private Vector3 _virtualChestAnchorLocal;
        // Native CharacterJoint constraint anchors may differ only in isolated v0.3.14 replicas.
        private Vector3 _characterJointPelvisAnchorLocal;
        private Vector3 _characterJointChestAnchorLocal;
        private Vector3 _lastLinearAcceleration;
        private Vector3 _lastAngularAcceleration;
        private Vector3 _sampledPelvisLinearVelocity;
        private Vector3 _sampledChestLinearVelocity;
        private Vector3 _sampledPelvisAngularVelocity;
        private Vector3 _sampledChestAngularVelocity;
        private Vector3 _sampledPelvisLinearAcceleration;
        private Vector3 _sampledChestLinearAcceleration;
        private Vector3 _sampledPelvisAngularAcceleration;
        private Vector3 _sampledChestAngularAcceleration;

        // v0.1.10: render-sampled velocity bridge retained unchanged from v0.1.9. Animator/source bones update on the render clock,
        // while the proxy is consumed on the physics clock. These values are refreshed only by the
        // real LateUpdate source sample and are then held constant across any intervening FixedUpdates.
        // This deliberately avoids differentiating an unchanged render pose again inside FixedUpdate.
        private Vector3 _sampledTargetRelativeAngularVelocityLocal;
        private Quaternion _previousSampledTargetRelativeRotation = Quaternion.identity;
        private bool _hasPreviousTargetRelativeSample;
        private int _sourceVelocitySampleSequence;
        private int _lastFixedConsumedSourceVelocitySampleSequence = -1;
        private float _sourceVelocitySampleTime;
        private float _sourceVelocitySampleAgeSeconds;
        private bool _sourceVelocitySampleFreshThisFixedStep;
        private int _traceFreshSourceVelocityFixedStepCount;
        private int _traceHeldSourceVelocityFixedStepCount;
        private Vector3 _previousSampledPelvisLinearVelocityForAcceleration;
        private Vector3 _previousSampledChestLinearVelocityForAcceleration;
        private Vector3 _previousSampledPelvisAngularVelocityForAcceleration;
        private Vector3 _previousSampledChestAngularVelocityForAcceleration;
        private bool _hasPreviousSourceAccelerationSample;
        private Vector3 _estimatedPelvisBodyLinearVelocity;
        private Vector3 _estimatedPelvisBodyAngularVelocity;
        private Vector3 _previousPelvisBodyPosition;
        private Quaternion _previousPelvisBodyRotation;
        private bool _hasPreviousPelvisBodySample;
        // Animation/render-side parent transport reference retained for comparison.
        private Vector3 _parentTransportLinearVelocity;
        // v0.1.11: physics-side parent transport implied by the Pose Bridge command at the target Chest COM lever arm.
        private Vector3 _physicsParentTransportLinearVelocity;
        private Vector3 _parentTransportConsistencyError;
        private float _parentTransportConsistencyErrorMagnitude;
        private Vector3 _parentFeedforwardReferenceVelocity;
        private Vector3 _sourceRelativeLinearVelocity;
        private Vector3 _targetRelativeAngularVelocity;
        private Vector3 _bodyRelativeAngularVelocity;
        private Vector3 _relativeAngularVelocityError;
        private Vector3 _parentFeedforwardAcceleration;
        private bool _parentFeedforwardAccelerationClamped;
        private int _parentFeedforwardAccelerationClampCount;
        private float _rawParentFeedforwardAccelerationMagnitude;
        private Vector3 _targetConnectedAnchorVelocity;
        private Vector3 _actualConnectedAnchorVelocity;
        private Vector3 _actualChestAnchorVelocity;
        private Vector3 _anchorVelocityError;
        private bool _parentMotionFeedforwardEnabled;

        // v0.1.10: physics-step command-delta diagnostics now measure the Pose Bridge command. Pelvis is still commanded only from FixedUpdate,
        // but these finite-difference values are DIAGNOSTIC ONLY. v0.1.8 proved that differentiating the
        // render-updated source pose again on the physics clock aliases into 0/spike/0/spike transport.
        // Control now uses the render-sampled sample-and-hold bridge above instead.
        private Vector3 _fixedStepPelvisTransportLinearVelocity;
        private Vector3 _fixedStepPelvisTransportAngularVelocity;
        private Vector3 _fixedStepTargetChestComVelocity;
        private Vector3 _fixedStepTargetRelativeAngularVelocity;
        private Vector3 _fixedStepBodyRelativeAngularVelocity;
        private Vector3 _previousCommandedPelvisPosition;
        private Quaternion _previousCommandedPelvisRotation;
        private Vector3 _previousCommandedTargetChestComPosition;
        private Quaternion _previousCommandedTargetRelativeRotation;
        private Quaternion _previousSettledBodyRelativeRotation;
        private Quaternion _settledBodyRelativeRotation = Quaternion.identity;
        private Quaternion _settledPelvisRotation = Quaternion.identity;
        private bool _hasPreviousFixedStepCommand;
        private Vector3 _lastAppliedTargetChestComVelocity;

        // v0.1.8: settled fixed-step finite-difference diagnostics. Samples are taken at the beginning
        // of FixedUpdate, before issuing the next kinematic Pelvis command, so Pelvis/Chest/anchors share
        // one physics phase. These values remain diagnostic only.
        // actual Rigidbody/anchor displacement between physics steps and are NOT used by the controller.
        private Vector3 _measuredPelvisLinearVelocity;
        private Vector3 _measuredChestLinearVelocity;
        private Vector3 _measuredPelvisAngularVelocity;
        private Vector3 _measuredChestAngularVelocity;
        private Vector3 _measuredPelvisAnchorVelocity;
        private Vector3 _measuredChestAnchorVelocity;
        private Vector3 _measuredAnchorRelativeVelocity;
        private Vector3 _measuredTransportReferenceVelocity;
        private Vector3 _measuredChestTransportVelocityError;
        private Vector3 _previousMeasuredPelvisPosition;
        private Vector3 _previousMeasuredChestPosition;
        private Quaternion _previousMeasuredPelvisRotation;
        private Quaternion _previousMeasuredChestRotation;
        private Vector3 _previousMeasuredPelvisAnchorPosition;
        private Vector3 _previousMeasuredChestAnchorPosition;
        private bool _hasPreviousMeasuredMotionSample;

        // v0.1.8: source-motion phase classifier and per-trace-session statistics.
        private string _motionPhase = "IDLE";
        private bool _motionTransition;
        private float _motionTargetSpeed;
        private float _motionSignedSpeedAcceleration;
        private float _previousMotionSpeed;
        private bool _hasPreviousMotionSpeed;
        private float _transitionHoldUntil;
        private string _heldTransitionPhase = "TRANSITION";
        private int _traceSteadySampleCount;
        private int _traceTransitionSampleCount;
        private int _traceIdleSampleCount;
        private float _traceSteadyRotationErrorSum;
        private float _traceSteadyRotationErrorMax;
        private float _traceTransitionRotationErrorSum;
        private float _traceTransitionRotationErrorMax;
        private float _traceSteadyChestPositionErrorSum;
        private float _traceSteadyChestPositionErrorMax;
        private float _traceTransitionChestPositionErrorSum;
        private float _traceTransitionChestPositionErrorMax;
        private float _traceSteadyMeasuredTransportErrorSum;
        private float _traceSteadyMeasuredTransportErrorMax;
        private float _traceTransitionMeasuredTransportErrorSum;
        private float _traceTransitionMeasuredTransportErrorMax;
        private int _traceTurnSampleCount;
        private float _traceTurnRotationErrorSum;
        private float _traceTurnRotationErrorMax;
        private int _traceSteadyAngularClampSampleCount;
        private int _traceTransitionAngularClampSampleCount;
        private int _traceTurnAngularClampSampleCount;
        private float _traceSteadyAnchorRelativeSpeedSum;
        private float _traceSteadyAnchorRelativeSpeedMax;
        private float _traceTransitionAnchorRelativeSpeedSum;
        private float _traceTransitionAnchorRelativeSpeedMax;
        private float _traceTurnAnchorRelativeSpeedSum;
        private float _traceTurnAnchorRelativeSpeedMax;
        private float _traceSteadyParentTransportConsistencySum;
        private float _traceSteadyParentTransportConsistencyMax;
        private float _traceTransitionParentTransportConsistencySum;
        private float _traceTransitionParentTransportConsistencyMax;
        private float _traceTurnParentTransportConsistencySum;
        private float _traceTurnParentTransportConsistencyMax;
        private Vector3 _previousPelvisPosition;
        private Vector3 _previousChestPosition;
        private Quaternion _previousPelvisRotation;
        private Quaternion _previousChestRotation;
        private bool _hasPreviousSourceSample;
        private bool _traceEnabled;
        private float _nextTraceTime;
        // v0.2.0: diagnostic-only FixedUpdate stage + fail-fast trace lifecycle.
        // Never feeds PD / Parent FF / Pose Bridge; it only explains and quarantines an unexpected drive drop.
        private string _fixedUpdateStage = "IDLE";
        private int _traceDriveAbortCount;
        private float _peakChestPositionError;
        private float _peakRotationError;
        private float _peakAnchorError;
        private float _peakLinearAcceleration;
        private float _peakAngularAcceleration;
        private float _peakJointLimitProximity;
        private bool _lastAngularAccelerationClamped;
        private int _angularAccelerationClampCount;
        private float _rawAngularAccelerationMagnitude;

        // v0.3.11 read-only torso controller/solver taps. These fields never feed back into control.
        private Vector3 _angularPAcceleration;
        private Vector3 _angularDAcceleration;
        private float _angularAccelerationHeadroom;
        private Vector3 _jointCurrentForceWorld;
        private bool _jointReactionReadbackValid;
        private Vector3 _jointCurrentTorqueWorld;
        private Vector3 _jointChestComLeverWorld;
        // v0.3.15 read-only same-phase parent-side reaction geometry snapshot.
        private Vector3 _jointPelvisComLeverWorld;
        private float _jointReactionConstraintAnchorErrorMeters;
        private Vector3 _jointForceMomentWorld;
        private Vector3 _jointCombinedMomentWorld;
        private Vector3 _jointTorqueAngularAccelerationProxyWorld;
        private Vector3 _jointForceMomentAngularAccelerationProxyWorld;
        private Vector3 _jointCombinedAngularAccelerationProxyWorld;
        private Quaternion _jointReferenceRelativeRotation = Quaternion.identity;
        private Vector3 _jointTwistAxis = Vector3.up;
        private Vector3 _jointSwing1Axis = Vector3.right;
        private Vector3 _jointSwing2Axis = Vector3.forward;
        private float _effectiveLowTwistLimit;
        private float _effectiveHighTwistLimit;
        private float _effectiveSwing1Limit;
        private float _effectiveSwing2Limit;
        private const string DriveBackend = "RELATIVE_JOINT_PD_PLUS_PARENT_FEEDFORWARD";
        private const string ConstraintBackend = "CHARACTER_JOINT";
        private const string ParentFeedforwardReferencePoseBridge = "POSE_BRIDGE_COMMAND";

        private Quaternion _pelvisBoneToProxy;
        private Quaternion _chestBoneToProxy;
        private Vector3 _sampledPelvisPosition;
        private Quaternion _sampledPelvisRotation;
        private Vector3 _sampledChestPosition;
        private Quaternion _sampledChestRotation;
        private bool _sampleValid;
        private bool _driveEnabled;
        private bool _gravityEnabled;
        private bool _collidersEnabled;
        private bool _visualEnabled = true;
        private bool _destroyed;
        private string _buildPhase = "NOT_STARTED";

        private readonly TwoBodyProxyReport _report = new TwoBodyProxyReport();

        public TwoBodyShadowProxy(ActiveRagdollLabProfile profile, BoneAuditReport audit)
            : this(profile, audit, TwoBodyMassPropertyMode.LegacyUnityDefault, "PRODUCTION", true, TwoBodyCharacterJointAnchorMode.CalibratedSourceJoint)
        {
        }

        internal TwoBodyShadowProxy(ActiveRagdollLabProfile profile, BoneAuditReport audit, TwoBodyMassPropertyMode massPropertyMode, string instanceTag, bool emitLogs)
            : this(profile, audit, massPropertyMode, instanceTag, emitLogs, TwoBodyCharacterJointAnchorMode.CalibratedSourceJoint)
        {
        }

        // v0.3.14 experiment-only CharacterJoint constraint-anchor override.
        // Source/control anchors and Parent-FF semantics remain frozen and are not redirected by this mode.
        internal TwoBodyShadowProxy(ActiveRagdollLabProfile profile, BoneAuditReport audit, TwoBodyMassPropertyMode massPropertyMode, string instanceTag, bool emitLogs, TwoBodyCharacterJointAnchorMode characterJointAnchorMode)
        {
            _profile = profile;
            _audit = audit;
            _massPropertyMode = massPropertyMode;
            _instanceTag = string.IsNullOrWhiteSpace(instanceTag) ? "SHADOW" : instanceTag.Trim();
            _emitLogs = emitLogs;
            _characterJointAnchorMode = characterJointAnchorMode;
            _sourceTurnDiagnostics.Enabled = emitLogs;
            _report.PelvisReceiver = _pelvisReceiver.Report;
        }

        public TwoBodyProxyReport Report => _report;
        public bool IsBuilt => !_destroyed && _root != null && _pelvisBody != null && _chestBody != null && _characterJoint != null;

        // v0.2.0 只读暴露冻结 torso 的挂接点，让新链路通过组合而不是复制/改写 Two-Body Dynamics。
        internal Transform ProxyRoot => _root != null ? _root.transform : null;
        internal Rigidbody PelvisBody => _pelvisBody;
        internal Rigidbody ChestBody => _chestBody; // V04460 read-only receiver audit; no new controller authority.
        internal PelvisReceiverReport PelvisReceiver => _pelvisReceiver.Report;
        internal bool DynamicPelvisReceiverSelected => _pelvisReceiver.DynamicSelected;
        private string CurrentParentTranslationReference => DynamicPelvisReceiverSelected ? "TWO_BODY_COMMON_COM_PD" : ParentFeedforwardReferencePoseBridge;
        internal bool PelvisReceiverReady => _pelvisReceiver.ReadyForRun;
        internal bool PelvisReceiverSafeOff
        {
            get
            {
                try { return !_driveEnabled && _pelvisReceiver.SafeOff && _pelvisBody != null && _chestBody != null &&
                    _pelvisBody.isKinematic && _chestBody.isKinematic && !_pelvisBody.useGravity && !_chestBody.useGravity &&
                    _pelvisCollider != null && _chestCollider != null && !_pelvisCollider.enabled && !_chestCollider.enabled; }
                catch { return false; }
            }
        }
        internal string GetPelvisReceiverNativeState() => _pelvisReceiver.GetNativeState();
        internal bool TryGetPelvisReceiverNativeState(out string state) => _pelvisReceiver.TryGetNativeState(out state);
        internal string SetPelvisReceiver(string arm)
        {
            if (!IsBuilt || _driveEnabled) return "ARL pelvis receiver | success=False | reason=STOP_TORSO_DRIVE_FIRST";
            if (arm != PelvisDynamicReceiverIsolation.IdleArm && !PelvisDynamicReceiverIsolation.IsExperimentArm(arm))
                return "ARL pelvis receiver | success=False | reason=RETIRED_OR_INVALID_ARM";
            _pelvisLegReference.Clear();
            ResetTorsoRecoveryState();
            _dynamicTorsoRecovery.ResetDiagnostics();
            SampleSourcePoseSnapshotOnly();
            if (!_sampleValid && arm != PelvisDynamicReceiverIsolation.IdleArm) return "ARL pelvis receiver | success=False | reason=SOURCE_POSE_INVALID";
            if (arm != PelvisDynamicReceiverIsolation.IdleArm)
            {
                PelvisDynamicReceiverIsolation.SynchronizeKinematicPose(_pelvisBody, _sampledPelvisPosition, _sampledPelvisRotation);
                PelvisDynamicReceiverIsolation.SynchronizeKinematicPose(_chestBody, _sampledChestPosition, _sampledChestRotation);
            }
            bool ok = _pelvisReceiver.Select(arm, _pelvisBody, _chestBody, _pelvisCollider, _chestCollider, _characterJoint);
            if (ok && arm == PelvisDynamicReceiverIsolation.IdleArm && _sampleValid)
            {
                PelvisDynamicReceiverIsolation.SynchronizeKinematicPose(_pelvisBody, _sampledPelvisPosition, _sampledPelvisRotation);
                PelvisDynamicReceiverIsolation.SynchronizeKinematicPose(_chestBody, _sampledChestPosition, _sampledChestRotation);
            }
            if (_characterJoint != null)
            {
                _characterJointChestAnchorLocal = _characterJoint.anchor;
                _characterJointPelvisAnchorLocal = _characterJoint.connectedAnchor;
            }
            _characterJointAnchorModeValidated = ok && ValidateConfiguredCharacterJointAnchors();
            _report.UseDynamicTorsoRecovery = ok && PelvisDynamicReceiverIsolation.IsDynamicLegArm(arm);
            _report.HipGoalGateOnlyHip = _report.UseDynamicTorsoRecovery;
            _report.UseCoherentHipNormalReference = _report.UseDynamicTorsoRecovery;
            _report.HipNormalSampleAgePrediction = _report.UseCoherentHipNormalReference && arm == PelvisDynamicReceiverIsolation.DynamicLegArmA;
            UpdateReport("receiver-selected");
            return "ARL pelvis receiver | success=" + ok + " | " + _pelvisReceiver.Report.GetSummary();
        }
        internal string QuiescePelvisReceiverPreserveTrace()
        {
            ResetTorsoRecoveryState();
            _pelvisLegReference.Clear();
            _driveEnabled = false;
            bool ok = _pelvisReceiver.QuiesceOwnedPair(_pelvisBody, _chestBody);
            ok &= DisableReceiverCollider(_pelvisCollider); ok &= DisableReceiverCollider(_chestCollider);
            _collidersEnabled = false;
            if (!ok) _pelvisReceiver.MarkFault("RECEIVER_QUIESCE_NATIVE_PAIR_FAILED");
            UpdateReport("receiver-quiesced");
            return "ARL pelvis receiver quiesce | success=" + ok;
        }
        private static bool DisableReceiverCollider(Collider collider)
        {
            try { if (collider == null) return false; collider.enabled = false; return !collider.enabled; }
            catch { return false; }
        }
        internal Collider[] OwnedColliders => new Collider[] { _pelvisCollider, _chestCollider };
        internal bool DriveEnabled => _driveEnabled;
        internal bool LastRollbackSucceeded { get; private set; } = true;
        internal Quaternion SampledPelvisTargetRotation => _sampledPelvisRotation;
        // Legacy consumers retain their kinematic pose. Dynamic consumers must receive
        // the FINAL governed command, with no fallback to the faster raw pose bridge.
        internal Vector3 CurrentPelvisCommandPosition => DynamicPelvisReceiverSelected ? RequireDynamicParentFrame().CommandPosition :
            _driveEnabled && _poseBridge.Initialized ? _poseBridge.CommandPosition : _sampledPelvisPosition;
        internal Quaternion CurrentPelvisCommandRotation => DynamicPelvisReceiverSelected ? RequireDynamicParentFrame().CommandRotation :
            _driveEnabled && _poseBridge.Initialized ? _poseBridge.CommandRotation : _sampledPelvisRotation;
        internal bool TryGetLegParentCommand(PhysicsStepContext step, out LegParentCommandFrame command)
        {
            command = _legParentCommand;
            return _driveEnabled && command.Matches(step);
        }
        internal bool LegParentReferenceBound => _pelvisLegReference.Bound;
        internal string LegParentReferenceIssue => _pelvisLegReference.Issue;
        internal bool BindPelvisLegReference(SingleLegTargetReport primary, SingleLegTargetReport secondary)
        {
            if (_driveEnabled || !DynamicPelvisReceiverSelected || primary == null || secondary == null || !primary.Success || !secondary.Success)
                return false;
            bool primaryLeft = string.Equals(primary.RequestedSide, "LEFT", StringComparison.OrdinalIgnoreCase);
            bool primaryRight = string.Equals(primary.RequestedSide, "RIGHT", StringComparison.OrdinalIgnoreCase);
            if ((!primaryLeft && !primaryRight) || !string.Equals(secondary.RequestedSide, primaryLeft ? "RIGHT" : "LEFT", StringComparison.OrdinalIgnoreCase)) return false;
            return _pelvisLegReference.Bind(_pelvisBody, _audit.Pelvis, primaryLeft ? primary.UpperLeg : secondary.UpperLeg,
                primaryLeft ? secondary.UpperLeg : primary.UpperLeg, _sampledPelvisPosition, _sampledPelvisRotation, _sourceVelocitySampleSequence);
        }
        internal bool TryGetDynamicParentFrame(out PelvisParentFrame frame)
        {
            frame = new PelvisParentFrame();
            return _driveEnabled && DynamicPelvisReceiverSelected && _pelvisReceiver.ReadyForRun &&
                _pelvisLegReference.TryGet(PhysicsTime, PhysicsDt, out frame);
        }
        private PelvisParentFrame RequireDynamicParentFrame()
        {
            if (!TryGetDynamicParentFrame(out PelvisParentFrame frame)) throw new InvalidOperationException("DYNAMIC_PARENT_FRAME_NOT_CURRENT");
            return frame;
        }
        // v0.3.8 read-only geometry exposure for isolated Two-Body COM/inertia factorization.
        internal bool CharacterJointAnchorModeValidated => _characterJointAnchorModeValidated;
        internal TwoBodyCharacterJointAnchorMode CharacterJointAnchorMode => DynamicPelvisReceiverSelected ? TwoBodyCharacterJointAnchorMode.ChestCenterOfMass : _characterJointAnchorMode;

        public string Build()
        {
            if (_audit == null || !_audit.Success || _audit.Pelvis == null || _audit.Chest == null)
                return "ARL build failed：没有可用的成功骨骼审计。";

            if (IsBuilt)
                return "ARL 双刚体代理已经存在。请先执行 arl_rollback。";

            try
            {
                EnterBuildPhase("CALCULATE_BODY_FRAME");
                float torso = _audit.PelvisToChestDistance;
                Vector3 up = _audit.CharacterUp.normalized;
                Vector3 forward = _audit.CharacterForward.normalized;
                Vector3 right = Vector3.Cross(up, forward).normalized;
                if (right.sqrMagnitude < 0.0001f)
                    right = Vector3.right;
                forward = Vector3.Cross(right, up).normalized;
                Quaternion frameRotation = Quaternion.LookRotation(forward, up);

                Vector3 pelvisSize = ClampSize(Vector3.Scale(_profile.PelvisSizeScale, Vector3.one * torso));
                Vector3 chestSize = ClampSize(Vector3.Scale(_profile.ChestSizeScale, Vector3.one * torso));

                EnterBuildPhase("CREATE_ROOT");
                _root = new GameObject("ARL_TwoBodyShadowProxy_" + Sanitize(_audit.TargetName) + (_instanceTag == "PRODUCTION" ? string.Empty : "_" + Sanitize(_instanceTag)));
                _root.hideFlags = HideFlags.DontSave;
                _root.layer = _audit.TargetRoot != null ? _audit.TargetRoot.gameObject.layer : 0;

                EnterBuildPhase("CREATE_PREVIEW_MATERIAL");
                _previewMaterial = CreatePreviewMaterial(_profile.PreviewAlpha);

                EnterBuildPhase("CREATE_PELVIS_OBJECT");
                _pelvisObject = CreateBody(
                    "PELVIS",
                    "ARL_PelvisBody",
                    _audit.Pelvis.position,
                    frameRotation,
                    pelvisSize,
                    new Vector3(0f, torso * 0.08f, 0f),
                    _profile.PelvisMass,
                    true,
                    out _pelvisBody,
                    out _pelvisCollider,
                    out _pelvisVisual);

                EnterBuildPhase("CREATE_CHEST_OBJECT");
                _chestObject = CreateBody(
                    "CHEST",
                    "ARL_ChestBody",
                    _audit.Chest.position,
                    frameRotation,
                    chestSize,
                    new Vector3(0f, -torso * 0.16f, 0f),
                    _profile.ChestMass,
                    true,
                    out _chestBody,
                    out _chestCollider,
                    out _chestVisual);

                _massPropertyModeValidated = _massPropertyMode == TwoBodyMassPropertyMode.LegacyUnityDefault ||
                    ValidateBodyMassPropertyMode(_pelvisBody, _pelvisCollider, _profile.PelvisMass, _massPropertyMode) &&
                    ValidateBodyMassPropertyMode(_chestBody, _chestCollider, _profile.ChestMass, _massPropertyMode);
                if (_massPropertyMode != TwoBodyMassPropertyMode.LegacyUnityDefault && !_massPropertyModeValidated)
                    throw new InvalidOperationException("TWOBODY_MASS_PROPERTY_MODE_READBACK_MISMATCH:" + _massPropertyMode);

                EnterBuildPhase("PARENT_PROXY_OBJECTS");
                _pelvisObject.transform.SetParent(_root.transform, true);
                _chestObject.transform.SetParent(_root.transform, true);

                EnterBuildPhase("CALIBRATE_PROXY_ROTATION");
                _pelvisBoneToProxy = Quaternion.Inverse(_audit.Pelvis.rotation) * frameRotation;
                _chestBoneToProxy = Quaternion.Inverse(_audit.Chest.rotation) * frameRotation;
                Vector3 jointWorld = _audit.Spine != null
                    ? _audit.Spine.position
                    : Vector3.Lerp(_audit.Pelvis.position, _audit.Chest.position, 0.42f);

                EnterBuildPhase("CALIBRATE_CHARACTER_JOINT_ANCHORS");
                _virtualPelvisAnchorLocal = _pelvisObject.transform.InverseTransformPoint(jointWorld);
                _virtualChestAnchorLocal = _chestObject.transform.InverseTransformPoint(jointWorld);

                // v0.3.14: keep the frozen source/control anchor semantics above untouched.
                // Only the native CharacterJoint constraint point may move in an isolated experiment lane.
                if (CharacterJointAnchorMode == TwoBodyCharacterJointAnchorMode.ChestCenterOfMass)
                {
                    Vector3 constraintWorld = _chestBody.worldCenterOfMass;
                    _characterJointChestAnchorLocal = _chestBody.centerOfMass;
                    _characterJointPelvisAnchorLocal = _pelvisObject.transform.InverseTransformPoint(constraintWorld);
                }
                else
                {
                    _characterJointChestAnchorLocal = _virtualChestAnchorLocal;
                    _characterJointPelvisAnchorLocal = _virtualPelvisAnchorLocal;
                }
                _characterJointAnchorModeValidated = ValidateCharacterJointAnchorMode();
                if (!_characterJointAnchorModeValidated)
                    throw new InvalidOperationException("TWOBODY_CHARACTERJOINT_ANCHOR_MODE_READBACK_MISMATCH:" + _characterJointAnchorMode);

                _lastLinearAcceleration = Vector3.zero;
                _lastAngularAcceleration = Vector3.zero;
                _parentMotionFeedforwardEnabled = _profile.ParentMotionFeedforward;
                ResetSourceVelocityEstimator();
                ResetPelvisBodyVelocityEstimator();
                ResetDrivePeaks();

                EnterBuildPhase("ADD_CHARACTER_JOINT");
                _characterJoint = Il2CppComponentFactory.Add<CharacterJoint>(
                    _chestObject,
                    _buildPhase);

                EnterBuildPhase("CONFIGURE_CHARACTER_JOINT");
                ConfigureCharacterJoint();
                _characterJointAnchorModeValidated = ValidateConfiguredCharacterJointAnchors();
                if (!_characterJointAnchorModeValidated)
                    throw new InvalidOperationException("TWOBODY_CHARACTERJOINT_CONFIGURED_ANCHOR_READBACK_MISMATCH:" + _characterJointAnchorMode);

                EnterBuildPhase("ADD_PELVIS_CONTACT_TRACKER");
                RagdollProxyContactTracker pelvisTracker =
                    Il2CppComponentFactory.Add<RagdollProxyContactTracker>(
                        _pelvisObject,
                        _buildPhase);
                pelvisTracker.Initialize("PELVIS");

                EnterBuildPhase("ADD_CHEST_CONTACT_TRACKER");
                RagdollProxyContactTracker chestTracker =
                    Il2CppComponentFactory.Add<RagdollProxyContactTracker>(
                        _chestObject,
                        _buildPhase);
                chestTracker.Initialize("CHEST");

                EnterBuildPhase("REGISTER_SELF_COLLISION_FILTER");
                RegisterIgnorePair(_pelvisCollider, _chestCollider);

                EnterBuildPhase("REGISTER_SOURCE_COLLISION_FILTER");
                IgnoreSourceCharacterColliders(_audit.TargetRoot);
                if (_report.IgnoreFailureCount > 0)
                    throw new InvalidOperationException("COLLISION_ISOLATION_FAILED count=" + _report.IgnoreFailureCount);

                EnterBuildPhase("APPLY_SAFE_INITIAL_STATE");
                _gravityEnabled = _profile.DefaultGravity;
                _collidersEnabled = _profile.DefaultColliders;
                _pelvisCollider.enabled = _collidersEnabled;
                _chestCollider.enabled = _collidersEnabled;
                _pelvisBody.useGravity = false;
                _chestBody.useGravity = false;
                _pelvisBody.isKinematic = true;
                _chestBody.isKinematic = true;

                EnterBuildPhase("SAMPLE_AND_RESET_POSE");
                SampleSourcePoseSnapshotOnly();
                ResetPoseInternal();
                ResetMeasuredMotionDiagnostics();
                ResetFixedStepTransportKinematics();
                ResetMotionPhaseClassifier();

                EnterBuildPhase("BUILD_COMPLETE");
                UpdateReport("build-complete-safe-preview");

                return
                    "ARL two-body build | success=True" +
                    " | phase=" + _buildPhase +
                    " | target=" + _audit.TargetName +
                    " | pelvisSize=" + Format(pelvisSize) +
                    " | chestSize=" + Format(chestSize) +
                    " | torso=" + torso.ToString("G7") +
                    " | pelvisKinematic=True | chestKinematic=True" +
                    " | drive=False | gravity=" + _gravityEnabled +
                    " | colliders=" + _collidersEnabled +
                    " | ignoredPairs=" + _ignoredPairs.Count +
                    " | componentFactory=non-generic-Il2CppType" +
                    " | driveBackend=" + DriveBackend +
                    " | diagnostics=SOURCE_AWARE_FREEZE_VALIDATION+SOURCE_TURN_DISCONTINUITY_ATTRIBUTION" +
                    " | angularVelocityMode=RENDER_SAMPLED_TARGET_RELATIVE_PLUS_FIXED_SETTLED_BODY_DELTA" +
                    " | drivePhaseSync=FIXEDUPDATE_PARENT_PLUS_RENDER_SAMPLE_HOLD_VELOCITY_AND_POSE_BRIDGE_CONSISTENT_PARENT_FF" +
                    " | parentFeedforward=" + _parentMotionFeedforwardEnabled +
                    " | parentFFReference=" + CurrentParentTranslationReference +
                    " | poseBridge=" + RenderToPhysicsPoseBridge.ModeName +
                    " | constraintBackend=" + ConstraintBackend +
                    " | characterJoint=True" +
                    " | characterJointAnchorMode=" + _characterJointAnchorMode +
                    " | characterJointAnchorValidated=" + _characterJointAnchorModeValidated +
                    " | connectedBodyMatch=" + (_characterJoint != null && _characterJoint.connectedBody == _pelvisBody) +
                    " | jointLimitMode=" + (_profile.WideJointLimitIsolation ? "WIDE_ISOLATION" : "ANATOMICAL") +
                    " | effectiveLimits=" + _effectiveLowTwistLimit.ToString("G6") + ".." + _effectiveHighTwistLimit.ToString("G6") + "/" + _effectiveSwing1Limit.ToString("G6") + "/" + _effectiveSwing2Limit.ToString("G6") +
                    " | standardJointRequired=CharacterJoint" +
                    " | sourceObjectUnchanged=True";
            }
            catch (Exception e)
            {
                string failedPhase = _buildPhase;
                string exceptionText = Il2CppComponentFactory.DescribeException(e);
                if (_emitLogs) Plugin.Logger?.LogWarning(
                    "[ARL][TWO_BODY][BUILD_FAILURE] phase=" + failedPhase +
                    " | exception=" + exceptionText);
                Rollback("build-exception:" + failedPhase);
                _report.BuildPhase = failedPhase;
                _report.LastDetail = "build-failed:" + failedPhase + ":" + exceptionText;
                return
                    "ARL build failed | phase=" + failedPhase +
                    " | exception=" + exceptionText +
                    " | rollback=True | sourceObjectUnchanged=True";
            }
        }

        public void SampleSourcePose(SourceStepContext source)
        {
            _contextCursor.AcceptSource(source);
            CompletedSource = new SourceStepContext(); _legParentCommand = new LegParentCommandFrame();
            _runtimeSource = source; _insideRuntimeSource = true;
            try
            {
                SampleSourcePoseInternal(true);
                if (_sampleValid) CompletedSource = source;
            }
            finally { _insideRuntimeSource = false; }
        }

        private void SampleSourcePoseSnapshotOnly()
        {
            SampleSourcePoseInternal(false);
        }

        private void SampleSourcePoseInternal(bool updateVelocityBridge)
        {
            _pelvisLegReference.Invalidate();
            if (_destroyed || _audit?.Pelvis == null || _audit.Chest == null)
            {
                _sampleValid = false;
                return;
            }

            try
            {
                Vector3 pelvisPosition = _audit.Pelvis.position;
                Quaternion pelvisRotation = _audit.Pelvis.rotation * _pelvisBoneToProxy;
                Vector3 chestPosition = _audit.Chest.position;
                Quaternion chestRotation = _audit.Chest.rotation * _chestBoneToProxy;

                float dt = _insideRuntimeSource ? _runtimeSource.Dt : Time.unscaledDeltaTime;
                Quaternion targetRelativeRotation = Quaternion.Inverse(pelvisRotation) * chestRotation;

                if (updateVelocityBridge)
                {
                    if (_hasPreviousSourceSample && IsFinite(dt) && dt >= 0.001f && dt <= 0.20f)
                    {
                        float blend = _profile.TargetVelocityBlend;
                        Vector3 pelvisLinearRaw = (pelvisPosition - _previousPelvisPosition) / dt;
                        Vector3 chestLinearRaw = (chestPosition - _previousChestPosition) / dt;
                        Vector3 pelvisAngularRaw = ComputeAngularVelocity(_previousPelvisRotation, pelvisRotation, dt);
                        Vector3 chestAngularRaw = ComputeAngularVelocity(_previousChestRotation, chestRotation, dt);
                        Vector3 targetRelativeAngularRawLocal = _hasPreviousTargetRelativeSample
                            ? ComputeAngularVelocity(_previousSampledTargetRelativeRotation, targetRelativeRotation, dt)
                            : Vector3.zero;

                        // v0.1.12 PURE DIAGNOSTIC: observe the raw Render-clock turn before any velocity clamp/filter.
                        // This path never modifies the values later consumed by PD / Parent FF / Pose Bridge.
                        if (_hasPreviousTargetRelativeSample)
                        {
                            _sourceTurnDiagnostics.ObserveRenderSample(
                                _sourceVelocitySampleSequence + 1,
                                CurrentUnscaledTime,
                                dt,
                                _previousPelvisRotation,
                                pelvisRotation,
                                _previousSampledTargetRelativeRotation,
                                targetRelativeRotation,
                                pelvisAngularRaw,
                                _profile.SourceTurnEventRotationDeltaDegrees,
                                _profile.SourceTurnEventAngularSpeedThreshold,
                                _profile.SourceTurnEventAngularAccelerationThreshold,
                                _profile.SourceTurnContextPreSeconds,
                                _profile.SourceTurnContextPostSeconds,
                                _profile.SourceTurnVerboseContextLogging);
                        }

                        pelvisLinearRaw = ClampMagnitude(pelvisLinearRaw, _profile.MaxLinearVelocity);
                        chestLinearRaw = ClampMagnitude(chestLinearRaw, _profile.MaxLinearVelocity);
                        pelvisAngularRaw = ClampMagnitude(pelvisAngularRaw, _profile.MaxAngularVelocity);
                        chestAngularRaw = ClampMagnitude(chestAngularRaw, _profile.MaxAngularVelocity);
                        targetRelativeAngularRawLocal = ClampMagnitude(targetRelativeAngularRawLocal, _profile.MaxAngularVelocity);

                        _sampledPelvisLinearVelocity = Vector3.Lerp(_sampledPelvisLinearVelocity, pelvisLinearRaw, blend);
                        _sampledChestLinearVelocity = Vector3.Lerp(_sampledChestLinearVelocity, chestLinearRaw, blend);
                        _sampledPelvisAngularVelocity = Vector3.Lerp(_sampledPelvisAngularVelocity, pelvisAngularRaw, blend);
                        _sampledChestAngularVelocity = Vector3.Lerp(_sampledChestAngularVelocity, chestAngularRaw, blend);
                        _sampledTargetRelativeAngularVelocityLocal = Vector3.Lerp(
                            _sampledTargetRelativeAngularVelocityLocal,
                            targetRelativeAngularRawLocal,
                            blend);

                        if (_hasPreviousSourceAccelerationSample)
                        {
                            _sampledPelvisLinearAcceleration = (_sampledPelvisLinearVelocity - _previousSampledPelvisLinearVelocityForAcceleration) / dt;
                            _sampledChestLinearAcceleration = (_sampledChestLinearVelocity - _previousSampledChestLinearVelocityForAcceleration) / dt;
                            _sampledPelvisAngularAcceleration = (_sampledPelvisAngularVelocity - _previousSampledPelvisAngularVelocityForAcceleration) / dt;
                            _sampledChestAngularAcceleration = (_sampledChestAngularVelocity - _previousSampledChestAngularVelocityForAcceleration) / dt;
                        }
                        else
                        {
                            _sampledPelvisLinearAcceleration = Vector3.zero;
                            _sampledChestLinearAcceleration = Vector3.zero;
                            _sampledPelvisAngularAcceleration = Vector3.zero;
                            _sampledChestAngularAcceleration = Vector3.zero;
                        }
                        _previousSampledPelvisLinearVelocityForAcceleration = _sampledPelvisLinearVelocity;
                        _previousSampledChestLinearVelocityForAcceleration = _sampledChestLinearVelocity;
                        _previousSampledPelvisAngularVelocityForAcceleration = _sampledPelvisAngularVelocity;
                        _previousSampledChestAngularVelocityForAcceleration = _sampledChestAngularVelocity;
                        _hasPreviousSourceAccelerationSample = true;
                    }
                    else
                    {
                        _sampledPelvisLinearVelocity = Vector3.zero;
                        _sampledChestLinearVelocity = Vector3.zero;
                        _sampledPelvisAngularVelocity = Vector3.zero;
                        _sampledChestAngularVelocity = Vector3.zero;
                        _sampledTargetRelativeAngularVelocityLocal = Vector3.zero;
                        _sampledPelvisLinearAcceleration = Vector3.zero;
                        _sampledChestLinearAcceleration = Vector3.zero;
                        _sampledPelvisAngularAcceleration = Vector3.zero;
                        _sampledChestAngularAcceleration = Vector3.zero;
                        _previousSampledPelvisLinearVelocityForAcceleration = Vector3.zero;
                        _previousSampledChestLinearVelocityForAcceleration = Vector3.zero;
                        _previousSampledPelvisAngularVelocityForAcceleration = Vector3.zero;
                        _previousSampledChestAngularVelocityForAcceleration = Vector3.zero;
                        _hasPreviousSourceAccelerationSample = false;
                    }

                    _previousPelvisPosition = pelvisPosition;
                    _previousChestPosition = chestPosition;
                    _previousPelvisRotation = pelvisRotation;
                    _previousChestRotation = chestRotation;
                    _previousSampledTargetRelativeRotation = targetRelativeRotation;
                    _hasPreviousSourceSample = true;
                    _hasPreviousTargetRelativeSample = true;
                    _sourceVelocitySampleSequence++;
                    _sourceVelocitySampleTime = CurrentUnscaledTime;
                }

                _sampledPelvisPosition = pelvisPosition;
                _sampledPelvisRotation = pelvisRotation;
                _sampledChestPosition = chestPosition;
                _sampledChestRotation = chestRotation;
                _sampleValid = IsFinite(_sampledPelvisPosition) && IsFinite(_sampledChestPosition) &&
                               IsFinite(_sampledPelvisRotation) && IsFinite(_sampledChestRotation) &&
                               IsFinite(_sampledPelvisLinearVelocity) && IsFinite(_sampledChestLinearVelocity) &&
                               IsFinite(_sampledPelvisAngularVelocity) && IsFinite(_sampledChestAngularVelocity) &&
                               IsFinite(_sampledTargetRelativeAngularVelocityLocal) &&
                               IsFinite(_sampledPelvisLinearAcceleration) && IsFinite(_sampledChestLinearAcceleration) &&
                               IsFinite(_sampledPelvisAngularAcceleration) && IsFinite(_sampledChestAngularAcceleration);
                if (_sampleValid) _pelvisLegReference.Sample(pelvisPosition, pelvisRotation, _sourceVelocitySampleSequence);

                // v0.1.8：LateUpdate 直接贴姿势只允许用于 drive=False 的安全预览。
                // drive=True 时即使 Collider 关闭，也绝不在渲染相位直接改 kinematic Pelvis；
                // Pelvis/CharacterJoint 的所有物理父体运动统一交给 FixedUpdate -> MovePosition/MoveRotation。
                // 这避免快速 TURN 时在两个物理步之间“瞬移”父关节坐标系，造成 Chest 被 CharacterJoint 突然扭动。
                if (_sampleValid && IsBuilt && !_driveEnabled && !_collidersEnabled)
                {
                    if (_pelvisBody.isKinematic)
                    {
                        _pelvisBody.position = _sampledPelvisPosition;
                        _pelvisBody.rotation = _sampledPelvisRotation;
                    }
                    if (_chestBody.isKinematic)
                    {
                        _chestBody.position = _sampledChestPosition;
                        _chestBody.rotation = _sampledChestRotation;
                    }
                }
            }
            catch
            {
                _sampleValid = false;
            }
        }

        public void FixedUpdate(PhysicsStepContext step)
        {
            _contextCursor.AcceptFixed(step);
            _legParentCommand = new LegParentCommandFrame();
            _report.DynamicTorsoRecovery = new TorsoRecoveryState { Active = true, Reason = "STEP_NOT_PUBLISHED" };
            _runtimeStep = step; _insideRuntimeStep = true;
            try
            {
                _fixedUpdateStage = "CONTEXT_ACCEPTED";
                FixedUpdateCore();
                if (_fixedUpdateStage == "COMPLETE" && IsBuilt && _sampleValid && _report.FiniteState && _driveEnabled &&
                    (!DynamicPelvisReceiverSelected || _pelvisReceiver.ReadyForRun))
                {
                    _legParentCommand = new LegParentCommandFrame { Step = step,
                        Position = CurrentPelvisCommandPosition, Rotation = CurrentPelvisCommandRotation };
                    if (DynamicPelvisReceiverSelected)
                    {
                        PelvisParentFrame physical = RequireDynamicParentFrame();
                        if (!physical.Matches(step)) throw new InvalidOperationException("DYNAMIC_PARENT_SHARED_CLOCK_MISMATCH");
                        _legParentCommand.PhysicalParent = physical;
                    }
                }
            }
            finally { _insideRuntimeStep = false; }
        }

        private void FixedUpdateCore()
        {
            _pelvisLegReference.Invalidate();
            if (!IsBuilt)
                return;

            if (!_sampleValid)
            {
                if (_pelvisReceiver.Report.Selected && _driveEnabled) DisableDriveForSafety("RECEIVER_SOURCE_POSE_INVALID", "ENTRY", null);
                UpdateReport("source-pose-invalid");
                return;
            }

            _fixedUpdateStage = "ENTRY";
            try
            {
                // v0.1.10 固定相位顺序不变；v0.2.0 只在每一步前记录诊断 stage。
                _fixedUpdateStage = "MEASURED_MOTION";
                UpdateMeasuredMotionDiagnostics();
                // v0.3.11: Joint.currentForce/currentTorque belong to the physics step that has just settled.
                // Capture them before issuing the next Pelvis/Chest command so they align with the measured motion above.
                _fixedUpdateStage = "JOINT_REACTION_WITNESS";
                CaptureSolvedCharacterJointReaction();
                _fixedUpdateStage = "SOURCE_VELOCITY_BRIDGE";
                UpdateSourceVelocityBridgeDiagnostics();
                _fixedUpdateStage = "MOTION_PHASE";
                UpdateMotionPhaseClassification();
                _fixedUpdateStage = "PELVIS_BODY_VELOCITY_ESTIMATE";
                UpdatePelvisBodyVelocityEstimate();
                _fixedUpdateStage = "RENDER_SAMPLE_HOLD_KINEMATICS";
                UpdateRenderSampleHoldControlKinematics();

                Vector3 pelvisCommandPosition = _sampledPelvisPosition;
                Quaternion pelvisCommandRotation = _sampledPelvisRotation;
                if (_driveEnabled)
                {
                    _fixedUpdateStage = "POSE_BRIDGE";
                    UpdateRenderToPhysicsPoseBridge();
                    pelvisCommandPosition = _poseBridge.CommandPosition;
                    pelvisCommandRotation = _poseBridge.CommandRotation;
                }

                _fixedUpdateStage = "FIXED_TRANSPORT_KINEMATICS";
                UpdateFixedStepTransportKinematics(pelvisCommandPosition, pelvisCommandRotation);
                Quaternion receiverRelative = Quaternion.Inverse(_sampledPelvisRotation) * _sampledChestRotation;
                Vector3 receiverRelativePosition = Quaternion.Inverse(_sampledPelvisRotation) * (_sampledChestPosition - _sampledPelvisPosition);
                Vector3 receiverTargetChestPosition = pelvisCommandPosition + pelvisCommandRotation * receiverRelativePosition;
                Quaternion receiverTargetChestRotation = pelvisCommandRotation * receiverRelative;
                var rawReceiverReference = new PelvisDynamicReceiverIsolation.Reference
                {
                    PelvisPosition = pelvisCommandPosition, PelvisRotation = pelvisCommandRotation,
                    PelvisVelocity = _poseBridge.CommandLinearVelocity, PelvisW = _poseBridge.CommandAngularVelocity,
                    ChestPosition = receiverTargetChestPosition, ChestRotation = receiverTargetChestRotation,
                    ChestComVelocity = _fixedStepTargetChestComVelocity
                };
                _fixedUpdateStage = "RECEIVER_REFERENCE";
                if (DynamicPelvisReceiverSelected && _driveEnabled) _pelvisLegReference.BeginStep(_runtimeStep);
                var controlReceiverReference = _pelvisReceiver.BuildControlReference(rawReceiverReference, PhysicsDt);
                PelvisReceiverReport.TrackingFrame receiverTracking = _pelvisReceiver.CaptureTracking(_traceEnabled,
                    controlReceiverReference.PelvisPosition, controlReceiverReference.PelvisRotation,
                    controlReceiverReference.PelvisVelocity, controlReceiverReference.PelvisW,
                    controlReceiverReference.ChestPosition, controlReceiverReference.ChestRotation, controlReceiverReference.ChestComVelocity,
                    _virtualChestAnchorLocal, _virtualPelvisAnchorLocal, PhysicsDt);
                receiverTracking = _pelvisReceiver.CaptureAnimationComparison(receiverTracking, controlReceiverReference, rawReceiverReference,
                    _sampledPelvisPosition, _sampledPelvisRotation, _sampledChestPosition, _sampledChestRotation);
                _fixedUpdateStage = "MOVE_PELVIS";
                if (DynamicPelvisReceiverSelected && _driveEnabled)
                {
                    _fixedUpdateStage = "PELVIS_RECEIVER";
                    if (!_pelvisReceiver.Step(_traceEnabled, _motionPhase, controlReceiverReference.PelvisPosition, controlReceiverReference.PelvisRotation,
                        controlReceiverReference.PelvisVelocity, controlReceiverReference.PelvisW,
                        controlReceiverReference.ChestPosition, controlReceiverReference.ChestRotation, controlReceiverReference.ChestComVelocity,
                        PhysicsDt, _profile.DriveDisableDistance))
                        throw new InvalidOperationException("PELVIS_RECEIVER_INVALID:" + _pelvisReceiver.Report.Issue);
                }
                else
                {
                    _pelvisBody.MovePosition(pelvisCommandPosition);
                    _pelvisBody.MoveRotation(pelvisCommandRotation);
                }

                if (!_driveEnabled)
                {
                    _fixedUpdateStage = "PREVIEW_FOLLOW_CHEST";
                    _chestBody.MovePosition(_sampledChestPosition);
                    _chestBody.MoveRotation(_sampledChestRotation);
                }
                else
                {
                    _fixedUpdateStage = "APPLY_CHARACTER_JOINT_PD";
                    // Both formal arms require two passive solves without controller force or torque.
                    if (!DynamicPelvisReceiverSelected || _pelvisReceiver.Report.Ready) ApplyCharacterJointPdTorqueDrive();
                }

                _fixedUpdateStage = "FINITE_STATE_CHECK";
                if (!HasFinitePhysicsState())
                {
                    DisableDriveForSafety("NON_FINITE_STATE", _fixedUpdateStage, null);
                    UpdateReport("non-finite-state-drive-disabled");
                    return;
                }

                _fixedUpdateStage = "UPDATE_REPORT";
                UpdateReport(_driveEnabled ? "drive-active" : "preview-follow");
                _fixedUpdateStage = "SOURCE_TURN_PHYSICS_RESPONSE";
                _sourceTurnDiagnostics.ObservePhysicsResponse(
                    _report.RotationErrorDegrees,
                    _report.AnchorErrorMeters,
                    _report.PoseBridgeRotationErrorDegrees,
                    _report.PoseBridgeCommandAngularVelocity,
                    _report.AngularAccelerationClamped);
                CopySourceTurnDiagnosticsToReport();
                receiverTracking = _pelvisReceiver.AttachAngularContext(receiverTracking,
                    _angularPAcceleration, _angularDAcceleration, _lastAngularAcceleration,
                    _targetRelativeAngularVelocity, _bodyRelativeAngularVelocity, _jointReactionReadbackValid,
                    _jointCurrentForceWorld, _jointCurrentTorqueWorld, _jointPelvisComLeverWorld);
                if (DynamicPelvisReceiverSelected && _driveEnabled)
                {
                    _pelvisLegReference.Publish(controlReceiverReference, _sourceVelocitySampleSequence, receiverTracking.Fixed,
                        _pelvisReceiver.ReadyForRun, PhysicsTime, PhysicsDt);
                    receiverTracking.ParentReferenceExpected = true; receiverTracking.ParentReferenceTime = PhysicsTime;
                    TryGetDynamicParentFrame(out receiverTracking.ParentFrame);
                    receiverTracking.ParentReferenceIssue = _pelvisLegReference.Issue;
                    if (receiverTracking.ParentFrame.Valid)
                    {
                        receiverTracking.ParentBridgeLeftShift = receiverTracking.ParentFrame.Left.CommandPoint -
                            (rawReceiverReference.PelvisPosition + rawReceiverReference.PelvisRotation * receiverTracking.ParentFrame.Left.LocalAnchor);
                        receiverTracking.ParentBridgeRightShift = receiverTracking.ParentFrame.Right.CommandPoint -
                            (rawReceiverReference.PelvisPosition + rawReceiverReference.PelvisRotation * receiverTracking.ParentFrame.Right.LocalAnchor);
                    }
                }
                receiverTracking.RuntimeContextExpected = _insideRuntimeStep;
                receiverTracking.RuntimeContext = _runtimeStep;
                _pelvisReceiver.CommitTracking(receiverTracking, _report);
                _fixedUpdateStage = "FREEZE_VALIDATION";
                _freezeValidation.ObserveFixedStep(_report, _sourceTurnDiagnostics, CurrentUnscaledTime);
                _freezeValidation.CopyToReport(_report);
                _fixedUpdateStage = "DYNAMIC_TORSO_RECOVERY";
                PublishTorsoRecoveryState();
                _fixedUpdateStage = "TRACE_STATISTICS";
                UpdateTraceSessionStatistics();
                _fixedUpdateStage = "TRACE_EMIT";
                EmitTraceIfDue();
                _fixedUpdateStage = "COMPLETE";
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError(
                    "[ARL][FIXED_UPDATE_EXCEPTION] stage=" + _fixedUpdateStage +
                    " | type=" + e.GetType().Name +
                    " | message=" + SafeLogText(e.Message));
                DisableDriveForSafety("FIXED_UPDATE_EXCEPTION", _fixedUpdateStage, e);
                UpdateReport("fixed-update-exception:" + _fixedUpdateStage + ":" + e.GetType().Name + "/" + e.Message);
            }
        }

        public string SetDrive(bool enabled)
        {
            if (!IsBuilt)
                return "ARL drive：代理尚未创建。";
            if (enabled && _driveEnabled && _pelvisReceiver.Report.Selected)
                return "ARL drive | enabled=True | alreadyInRequestedState=True | action=NO_OP";
            ResetTorsoRecoveryState();
            _pelvisLegReference.Invalidate();
            if (!enabled) _pelvisLegReference.Clear();

            // v0.2.0: trace is only a validation of active dynamics. A manual drive-off before
            // arl_trace(off) aborts the validation instead of letting preview-follow samples contaminate it.
            if (!enabled && _driveEnabled && _traceEnabled)
                AbortActiveTraceForDriveLoss("MANUAL_DRIVE_OFF", "COMMAND", "arl_drive(off) while trace was active");

            SampleSourcePoseSnapshotOnly();
            if (enabled && !_sampleValid)
                return "ARL drive：源骨骼姿势无效。";

            if (enabled)
            {
                ResetPoseInternal();
                if (!DynamicPelvisReceiverSelected)
                {
                    _chestBody.isKinematic = false;
                    _chestBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                    _chestBody.useGravity = _gravityEnabled;
                    _chestBody.drag = 0f;
                    _chestBody.angularDrag = 0f;
                    _chestBody.velocity = ClampMagnitude(_sampledChestLinearVelocity, _profile.MaxLinearVelocity);
                    _chestBody.angularVelocity = ClampMagnitude(_sampledChestAngularVelocity, _profile.MaxAngularVelocity);
                }
                if (!_pelvisReceiver.BeginDrive())
                {
                    ForceDriveOffSafeKinematic(); UpdateReport("receiver-drive-rejected");
                    return "ARL drive | enabled=False | reason=" + _pelvisReceiver.Report.Issue;
                }
                if (DynamicPelvisReceiverSelected)
                {
                    _characterJointChestAnchorLocal = _characterJoint.anchor;
                    _characterJointPelvisAnchorLocal = _characterJoint.connectedAnchor;
                    _characterJointAnchorModeValidated = ValidateConfiguredCharacterJointAnchors();
                    if (!_characterJointAnchorModeValidated) throw new InvalidOperationException("RECEIVER_NATIVE_ANCHOR_START_READBACK_FAILED");
                }
                _lastLinearAcceleration = Vector3.zero;
                _lastAngularAcceleration = Vector3.zero;
                _parentFeedforwardAcceleration = Vector3.zero;
                ResetPelvisBodyVelocityEstimator();
                ResetMeasuredMotionDiagnostics();
                _poseBridge.Reset();
                _poseBridge.Initialize(_sampledPelvisPosition, _sampledPelvisRotation);
                ResetFixedStepTransportKinematics();
                ResetMotionPhaseClassifier();
                ResetDrivePeaks();
                _driveEnabled = true;
            }
            else
            {
                _pelvisReceiver.QuiesceOwnedPair(_pelvisBody, _chestBody);
                _driveEnabled = false;
                _lastLinearAcceleration = Vector3.zero;
                _lastAngularAcceleration = Vector3.zero;
                _parentFeedforwardAcceleration = Vector3.zero;
                _chestBody.useGravity = false;
                _chestBody.isKinematic = true;
                if (!DynamicPelvisReceiverSelected)
                {
                    _chestBody.collisionDetectionMode = CollisionDetectionMode.Discrete;
                    _chestBody.drag = 0.25f;
                    _chestBody.angularDrag = _profile.AngularDrag;
                }
                _chestBody.velocity = Vector3.zero;
                _chestBody.angularVelocity = Vector3.zero;
                ResetPoseInternal();
                _poseBridge.Reset();
                ResetMeasuredMotionDiagnostics();
                ResetMotionPhaseClassifier();
            }

            UpdateReport(enabled ? "drive-enabled" : "drive-disabled-safe-kinematic");
            return
                "ARL drive | enabled=" + _driveEnabled +
                " | pelvisKinematic=" + _pelvisBody.isKinematic + " | receiverArm=" + _pelvisReceiver.Report.Arm +
                " | chestKinematic=" + _chestBody.isKinematic +
                " | gravityApplied=" + (_driveEnabled && _gravityEnabled) +
                " | colliders=" + _collidersEnabled +
                " | driveBackend=" + DriveBackend +
                " | parentFeedforward=" + _parentMotionFeedforwardEnabled +
                " | parentFFReference=" + CurrentParentTranslationReference +
                " | poseBridge=" + RenderToPhysicsPoseBridge.ModeName +
                " | constraintBackend=" + ConstraintBackend +
                " | characterJoint=" + (_characterJoint != null) +
                " | connectedBodyMatch=" + (_characterJoint != null && _characterJoint.connectedBody == _pelvisBody);
        }

        public string SetGravity(bool enabled)
        {
            if (!IsBuilt)
                return "ARL gravity：代理尚未创建。";
            if (enabled && _driveEnabled)
                return "ARL gravity：拒绝开启。v0.3.28.1 frozen torso drive 固定 Gravity=OFF；请先停止 drive。";
            if (enabled && _pelvisReceiver.Report.Selected)
                return "ARL gravity：接收体参考预检固定躯干重力OFF；请先停止并清理本轮实验。";
            _gravityEnabled = enabled;
            _chestBody.useGravity = _driveEnabled && enabled;
            UpdateReport("gravity-changed");
            return "ARL gravity | configured=" + enabled + " | applied=" + _chestBody.useGravity;
        }

        public string SetColliders(bool enabled)
        {
            if (!IsBuilt)
                return "ARL colliders：代理尚未创建。";
            if (enabled && _pelvisReceiver.Report.Selected)
                return "ARL colliders：接收体参考预检固定躯干碰撞OFF。";
            if (enabled && _driveEnabled)
                return "ARL colliders：拒绝开启。v0.3.28.1 frozen torso drive 固定 Colliders=OFF；请先停止 drive。";
            _collidersEnabled = enabled;
            _pelvisCollider.enabled = enabled;
            _chestCollider.enabled = enabled;
            UpdateReport("colliders-changed");
            return "ARL colliders | enabled=" + enabled + " | sourceCharacterPairsIgnored=" + _ignoredPairs.Count;
        }

        public string SetVisual(bool enabled)
        {
            if (!IsBuilt)
                return "ARL visual：代理尚未创建。";
            _visualEnabled = enabled;
            SetRendererEnabled(_pelvisVisual, enabled);
            SetRendererEnabled(_chestVisual, enabled);
            UpdateReport("visual-changed");
            return "ARL visual | enabled=" + enabled;
        }

        public string SetTrace(bool enabled)
        {
            if (!IsBuilt)
                return "ARL trace：代理尚未创建。";

            if (enabled)
            {
                if (!_driveEnabled)
                    return "ARL trace：冻结验收要求 drive=True。请先执行 arl_drive(on)。";
                if (_pelvisReceiver.Report.Selected && !_pelvisReceiver.ReadyForRun)
                    return "ARL trace：骨盆接收体尚未通过两拍被动求解或驱动资格。请查看arl_test_status。";

                _traceDriveAbortCount = 0;
                ResetTraceSessionStatistics();
                _sourceTurnDiagnostics.BeginTraceSession();
                _freezeValidation.BeginTraceSession();
                _freezeValidation.CopyToReport(_report);
                _traceEnabled = true;
                _pelvisReceiver.BeginTrace();
                _dynamicTorsoRecovery.ResetDiagnostics();
                _nextTraceTime = 0f;
                UpdateReport("trace-enabled");
                if (_emitLogs) Plugin.Logger?.LogInfo(
                    "[ARL][TRACE] trace-start | intervalSeconds=" + _profile.TraceIntervalSeconds.ToString("G6") +
                    " | logMode=" + (_profile.TraceVerboseSnapshots ? "VERBOSE" : "COMPACT") +
                    " | sourceTurnContext=" + (_profile.SourceTurnVerboseContextLogging ? "VERBOSE" : "SUMMARY_ONLY") +
                    " | diagnostics=render-sample-hold-velocity+pose-bridge+pose-consistent-parent-ff+source-turn-discontinuity-attribution+recovery-failure-attribution+drive-fail-fast");
                return
                    "ARL trace | enabled=True" +
                    " | intervalSeconds=" + _profile.TraceIntervalSeconds.ToString("G6") +
                    " | statsReset=True" +
                    " | phaseSafeFixedStep=True | renderVelocityBridge=sample-and-hold | poseBridge=predict-correct" +
                    " | parentFFReference=" + CurrentParentTranslationReference +
                    " | traceLogMode=" + (_profile.TraceVerboseSnapshots ? "VERBOSE" : "COMPACT") +
                    " | sourceTurnContext=" + (_profile.SourceTurnVerboseContextLogging ? "VERBOSE" : "SUMMARY_ONLY") +
                    " | freezeValidation=" + SourceAwareFreezeValidation.PolicyName;
            }

            _sourceTurnDiagnostics.EndTraceSession();
            _pelvisReceiver.EndTrace();
            _traceEnabled = false;
            UpdateReport("trace-disabled");
            CopySourceTurnDiagnosticsToReport();
            _freezeValidation.EndTraceSession(_report, _sourceTurnDiagnostics, CurrentUnscaledTime);
            _freezeValidation.CopyToReport(_report);
            string summary = _report.GetTraceSessionSummary();
            string freezeSummary = _freezeValidation.GetSummary(_report, _sourceTurnDiagnostics);
            if (_emitLogs) Plugin.Logger?.LogInfo("[ARL][TRACE_SUMMARY] " + summary);
            if (_emitLogs) Plugin.Logger?.LogWarning("[ARL][FREEZE_VALIDATION] " + freezeSummary +
                " | verdictRole=" + (DynamicPelvisReceiverSelected ? "LEGACY_REFERENCE_ONLY_FOR_DYNAMIC_RECEIVER" : "LEGACY_KINEMATIC_BASELINE"));
            if (_emitLogs) Plugin.Logger?.LogInfo("[ARL][TRACE] trace-stop");
            return
                "ARL trace | enabled=False" +
                " | intervalSeconds=" + _profile.TraceIntervalSeconds.ToString("G6") +
                " | summary=" + summary +
                " | freeze=" + freezeSummary;
        }

        private void DisableDriveForSafety(string reason, string stage, Exception exception)
        {
            if (_pelvisReceiver.Report.Selected) _pelvisReceiver.MarkFault(reason);
            ActiveRagdollLabRuntime.RecordAutomaticStop("TORSO/" + stage, reason + (exception != null ? ":" + exception.GetType().Name : ""));
            string detail = exception == null
                ? "<none>"
                : exception.GetType().Name + ":" + SafeLogText(exception.Message);

            if (_traceEnabled && _driveEnabled)
            {
                try
                {
                    AbortActiveTraceForDriveLoss(reason, stage, detail);
                }
                catch (Exception abortException)
                {
                    // Fail closed even if the diagnostic/validation abort path itself is the throwing stage.
                    _traceEnabled = false;
                    Plugin.Logger?.LogError(
                        "[ARL][DRIVE_ABORT_HANDLER_EXCEPTION] type=" + abortException.GetType().Name +
                        " | message=" + SafeLogText(abortException.Message));
                }
            }

            // Keep the existing v0.1.11 safe-kinematic shutdown semantics, but do not depend on
            // a fresh source snapshot while already handling a safety failure/exception.
            ForceDriveOffSafeKinematic();
        }

        private void ForceDriveOffSafeKinematic()
        {
            ResetTorsoRecoveryState();
            _pelvisLegReference.Clear();
            _driveEnabled = false;
            _lastLinearAcceleration = Vector3.zero;
            _lastAngularAcceleration = Vector3.zero;
            _parentFeedforwardAcceleration = Vector3.zero;
            _pelvisReceiver.QuiesceOwnedPair(_pelvisBody, _chestBody);

            try
            {
                if (_chestBody != null)
                {
                    _chestBody.useGravity = false;
                    _chestBody.isKinematic = true;
                    if (!DynamicPelvisReceiverSelected)
                    {
                        _chestBody.collisionDetectionMode = CollisionDetectionMode.Discrete;
                        _chestBody.drag = 0.25f;
                        _chestBody.angularDrag = _profile.AngularDrag;
                    }
                    _chestBody.velocity = Vector3.zero;
                    _chestBody.angularVelocity = Vector3.zero;
                }
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError(
                    "[ARL][DRIVE_ABORT_SAFE_STATE_ERROR] type=" + e.GetType().Name +
                    " | message=" + SafeLogText(e.Message));
            }

            try
            {
                if (_sampleValid)
                    ResetPoseInternal();
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogWarning("[ARL][DRIVE_ABORT_SAFE_STATE_ERROR] step=RESET_POSE | type=" + e.GetType().Name + " | message=" + SafeLogText(e.Message));
            }

            _poseBridge.Reset();
            ResetMeasuredMotionDiagnostics();
            ResetMotionPhaseClassifier();
        }

        private void AbortActiveTraceForDriveLoss(string reason, string stage, string detail)
        {
            if (!_traceEnabled)
                return;

            _traceDriveAbortCount++;
            Plugin.Logger?.LogError(
                "[ARL][DRIVE_ABORT] abortCount=" + _traceDriveAbortCount +
                " | reason=" + reason +
                " | stage=" + stage +
                " | detail=" + SafeLogText(detail) +
                " | drive=" + _driveEnabled +
                " | finite=" + HasFinitePhysicsState() +
                " | rotErr=" + _report.RotationErrorDegrees.ToString("G7") +
                " | chestPosErr=" + _report.ChestPositionErrorMeters.ToString("G7") +
                " | anchorErr=" + _report.AnchorErrorMeters.ToString("G7"));

            // Stop both diagnostic clocks immediately. No preview-follow sample after this point may enter
            // NORMAL/SOURCE/RECOVERY freeze statistics. The user can still call arl_trace(off) later to
            // print the preserved aborted-session summary.
            _sourceTurnDiagnostics.EndTraceSession();
            CopySourceTurnDiagnosticsToReport();
            _freezeValidation.AbortTraceSession(
                _report,
                _sourceTurnDiagnostics,
                CurrentUnscaledTime,
                reason,
                stage,
                detail);
            _freezeValidation.CopyToReport(_report);
            _traceEnabled = false;

            Plugin.Logger?.LogWarning(
                "[ARL][FREEZE_VALIDATION_ABORT] " + _freezeValidation.GetSummary(_report, _sourceTurnDiagnostics));
        }

        private static string SafeLogText(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }

        public string ResetPose()
        {
            if (!IsBuilt)
                return "ARL reset：代理尚未创建。";
            SampleSourcePoseSnapshotOnly();
            if (!_sampleValid)
                return "ARL reset：源骨骼姿势无效。";
            bool driveWasEnabled = _driveEnabled;
            if (driveWasEnabled)
                SetDrive(false);
            ResetPoseInternal();
            _poseBridge.Reset();
            ResetMeasuredMotionDiagnostics();
            ResetFixedStepTransportKinematics();
            ResetMotionPhaseClassifier();
            UpdateReport("pose-reset");
            return "ARL reset | success=True | driveNow=False | chestKinematic=True";
        }

        public void NotifyCollision(string bodyLabel, string phase, Collision collision)
        {
            if (!IsBuilt || collision == null)
                return;

            string other = "<unknown>";
            try
            {
                Collider collider = collision.collider;
                if (collider != null)
                    other = CharacterTargetFinder.GetPath(collider.transform);
                else if (collision.gameObject != null)
                    other = CharacterTargetFinder.GetPath(collision.gameObject.transform);
            }
            catch { }

            if (phase == "ENTER") _report.ContactEnterCount++;
            else if (phase == "STAY") _report.ContactStayCount++;
            else if (phase == "EXIT") _report.ContactExitCount++;
            _report.LastContactBody = bodyLabel;
            _report.LastContactOther = other;
            _report.LastDetail = "collision-" + phase.ToLowerInvariant();
        }

        public string Rollback(string reason)
        {
            ResetTorsoRecoveryState();
            _pelvisLegReference.Clear();
            if (_destroyed)
                return "ARL rollback | success=" + LastRollbackSucceeded + " | already-cleared=" + LastRollbackSucceeded + " | cleanupPending=" + !LastRollbackSucceeded + " | failedLegacyTeardownRequiresRestart=" + !LastRollbackSucceeded;

            int cleanupFailures = 0;
            if (!_pelvisReceiver.QuiesceOwnedPair(_pelvisBody, _chestBody)) cleanupFailures++;
            if (_pelvisReceiver.Report.BaselineCaptured && !_pelvisReceiver.Select(PelvisDynamicReceiverIsolation.IdleArm, _pelvisBody, _chestBody, _pelvisCollider, _chestCollider, _characterJoint)) cleanupFailures++;
            _sourceTurnDiagnostics.EndTraceSession();
            _destroyed = true;
            _driveEnabled = false;
            _traceEnabled = false;
            _poseBridge.Reset();

            int ignoreRestoreFailures = 0;
            for (int i = _ignoredPairs.Count - 1; i >= 0; i--)
            {
                IgnoredPair pair = _ignoredPairs[i];
                try
                {
                    if (pair?.A != null && pair.B != null)
                        UnityEngine.Physics.IgnoreCollision(pair.A, pair.B, false);
                }
                catch
                {
                    ignoreRestoreFailures++;
                }
            }
            _ignoredPairs.Clear();
            if (ignoreRestoreFailures > 0)
            {
                cleanupFailures += ignoreRestoreFailures;
                Plugin.Logger?.LogWarning("[ARL][ROLLBACK_ISOLATION_RESTORE] owner=TWOBODY | failures=" + ignoreRestoreFailures);
            }

            try
            {
                if (_pelvisCollider != null) _pelvisCollider.enabled = false;
                if (_chestCollider != null) _chestCollider.enabled = false;
                if (_chestBody != null)
                {
                    _chestBody.useGravity = false;
                    _chestBody.isKinematic = true;
                    _chestBody.velocity = Vector3.zero;
                    _chestBody.angularVelocity = Vector3.zero;
                }
            }
            catch (Exception e)
            {
                cleanupFailures++;
                Plugin.Logger?.LogWarning("[ARL][TWOBODY_ROLLBACK_STEP_FAILED] step=SAFE_STATE | " + e.GetType().Name + " / " + SafeLogText(e.Message));
            }

            try { if (_root != null) Object.Destroy(_root); }
            catch (Exception e)
            {
                cleanupFailures++;
                Plugin.Logger?.LogWarning("[ARL][TWOBODY_ROLLBACK_STEP_FAILED] step=DESTROY_ROOT | " + e.GetType().Name + " / " + SafeLogText(e.Message));
            }
            try { if (_previewMaterial != null) Object.Destroy(_previewMaterial); }
            catch (Exception e)
            {
                cleanupFailures++;
                Plugin.Logger?.LogWarning("[ARL][TWOBODY_ROLLBACK_STEP_FAILED] step=DESTROY_PREVIEW_MATERIAL | " + e.GetType().Name + " / " + SafeLogText(e.Message));
            }

            _root = null;
            _pelvisObject = null;
            _chestObject = null;
            _pelvisBody = null;
            _chestBody = null;
            _pelvisCollider = null;
            _chestCollider = null;
            _characterJoint = null;
            _pelvisVisual = null;
            _chestVisual = null;
            _previewMaterial = null;

            _report.Built = false;
            _report.DriveEnabled = false;
            _report.CollidersEnabled = false;
            LastRollbackSucceeded = cleanupFailures == 0;
            _report.LastDetail = (LastRollbackSucceeded ? "rollback:" : "rollback-with-errors:") + reason;
            return "ARL rollback | success=" + LastRollbackSucceeded + " | reason=" + reason + " | cleanupFailures=" + cleanupFailures + " | sourceObjectUnchanged=True";
        }

        private GameObject CreateBody(
            string bodyLabel,
            string name,
            Vector3 position,
            Quaternion rotation,
            Vector3 size,
            Vector3 colliderCenter,
            float mass,
            bool kinematic,
            out Rigidbody body,
            out BoxCollider collider,
            out GameObject visual)
        {
            GameObject gameObject = new GameObject(name + (_instanceTag == "PRODUCTION" ? string.Empty : "_" + Sanitize(_instanceTag)));
            gameObject.hideFlags = HideFlags.DontSave;
            gameObject.layer = _root != null ? _root.layer : 0;
            // Establish ownership before AddComponent/readback/preview work can throw.  Older builds parented
            // Pelvis/Chest only after both CreateBody calls returned, so a mid-build exception could leave an
            // unparented ARL body behind even though rollback destroyed _root.
            if (_root != null)
                gameObject.transform.SetParent(_root.transform, true);
            gameObject.transform.position = position;
            gameObject.transform.rotation = rotation;

            EnterBuildPhase("ADD_" + bodyLabel + "_BOX_COLLIDER");
            collider = Il2CppComponentFactory.Add<BoxCollider>(gameObject, _buildPhase);
            collider.center = colliderCenter;
            collider.size = size;
            collider.enabled = false;

            EnterBuildPhase("ADD_" + bodyLabel + "_RIGIDBODY");
            body = Il2CppComponentFactory.Add<Rigidbody>(gameObject, _buildPhase);
            body.mass = mass;
            body.isKinematic = kinematic;
            body.useGravity = false;
            body.drag = 0.25f;
            body.angularDrag = _profile.AngularDrag;
            body.maxAngularVelocity = _profile.MaxAngularVelocity;
            body.interpolation = RigidbodyInterpolation.None;
            body.collisionDetectionMode = CollisionDetectionMode.Discrete;

            Vector3 legacyCom = body.centerOfMass;
            Vector3 legacyInertia = body.inertiaTensor;
            Quaternion legacyInertiaRotation = body.inertiaTensorRotation;
            Vector3 expectedInertia = AnalyticalUniformBoxInertia(mass, size);

            if (_massPropertyMode == TwoBodyMassPropertyMode.ExplicitCenterOfMassOnly ||
                _massPropertyMode == TwoBodyMassPropertyMode.ExplicitUniformBox)
                body.centerOfMass = colliderCenter;

            if (_massPropertyMode == TwoBodyMassPropertyMode.ExplicitInertiaOnly ||
                _massPropertyMode == TwoBodyMassPropertyMode.ExplicitUniformBox)
            {
                body.inertiaTensorRotation = Quaternion.identity;
                body.inertiaTensor = expectedInertia;
            }

            bool comOk = (_massPropertyMode == TwoBodyMassPropertyMode.ExplicitCenterOfMassOnly || _massPropertyMode == TwoBodyMassPropertyMode.ExplicitUniformBox)
                ? Approximately(body.centerOfMass, colliderCenter)
                : Approximately(body.centerOfMass, legacyCom);
            bool inertiaOk = (_massPropertyMode == TwoBodyMassPropertyMode.ExplicitInertiaOnly || _massPropertyMode == TwoBodyMassPropertyMode.ExplicitUniformBox)
                ? Approximately(body.inertiaTensor, expectedInertia) && Approximately(body.inertiaTensorRotation, Quaternion.identity)
                : Approximately(body.inertiaTensor, legacyInertia) && Approximately(body.inertiaTensorRotation, legacyInertiaRotation);
            if (!comOk || !inertiaOk)
                throw new InvalidOperationException(bodyLabel + " mass-property mode readback mismatch: " + _massPropertyMode);

            EnterBuildPhase("CREATE_" + bodyLabel + "_VISUAL");
            visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            visual.name = gameObject.name + "_Visual";
            visual.hideFlags = HideFlags.DontSave;
            visual.layer = gameObject.layer;
            visual.transform.SetParent(gameObject.transform, false);
            visual.transform.localPosition = colliderCenter;
            visual.transform.localRotation = Quaternion.identity;
            visual.transform.localScale = size;

            try
            {
                Collider primitiveCollider = visual.GetComponent<Collider>();
                if (primitiveCollider == null)
                    throw new InvalidOperationException("TWOBODY_PREVIEW_COLLIDER_MISSING");
                // CreatePrimitive(Cube) includes a Collider. It must be disabled synchronously before the
                // deferred Destroy call; otherwise a preview-only object could become an unintended physics body.
                primitiveCollider.enabled = false;
                Object.Destroy(primitiveCollider);
            }
            catch (Exception e)
            {
                throw new InvalidOperationException("TWOBODY_PREVIEW_COLLIDER_DISABLE_FAILED", e);
            }

            try
            {
                Renderer renderer = visual.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.sharedMaterial = _previewMaterial;
            }
            catch { }

            return gameObject;
        }

        private void ApplyCharacterJointPdTorqueDrive()
        {
            if (_characterJoint == null || _characterJoint.connectedBody != _pelvisBody)
                throw new InvalidOperationException("CHARACTER_JOINT missing or connectedBody mismatch.");

            Vector3 positionError = _sampledChestPosition - _chestBody.position;
            float errorDistance = positionError.magnitude;
            if (!IsFinite(errorDistance) || errorDistance > _profile.DriveDisableDistance)
                throw new InvalidOperationException(
                    "CHARACTER_JOINT chest position error exceeded safety gate: " +
                    errorDistance.ToString("G7"));

            // Legacy virtual/native anchors coincide. B's native pivot is Chest COM;
            // a rotated source virtual lever is no longer a native constraint separation.
            Vector3 chestAnchorWorld = ChestPoint(DynamicPelvisReceiverSelected ? _characterJointChestAnchorLocal : _virtualChestAnchorLocal);
            Vector3 pelvisAnchorWorld = PelvisPoint(DynamicPelvisReceiverSelected ? _characterJointPelvisAnchorLocal : _virtualPelvisAnchorLocal);
            float jointAnchorError = Vector3.Distance(chestAnchorWorld, pelvisAnchorWorld);
            if (!IsFinite(jointAnchorError) || jointAnchorError > _profile.JointAnchorSafetyDistance)
                throw new InvalidOperationException(
                    "CHARACTER_JOINT anchor error exceeded safety gate: " +
                    jointAnchorError.ToString("G7"));

            // v0.2.0 清理冻结前的 A/B 分支：v0.1.11-v0.2.0 已经验证 POSE_BRIDGE_COMMAND 为正式主线。
            // source/render parentTransportVel 仍保留作诊断，但不再允许运行时切换控制参考，避免后续链路误入历史实验分支。
            _parentFeedforwardReferenceVelocity = _physicsParentTransportLinearVelocity;
            Vector3 rawParentFeedforwardAcceleration = Vector3.zero;
            _parentFeedforwardAccelerationClamped = false;
            if (_parentMotionFeedforwardEnabled && !DynamicPelvisReceiverSelected)
            {
                rawParentFeedforwardAcceleration =
                    (_parentFeedforwardReferenceVelocity - _chestBody.velocity) *
                    _profile.ParentMotionFeedforwardGain;
                _rawParentFeedforwardAccelerationMagnitude = rawParentFeedforwardAcceleration.magnitude;
                _parentFeedforwardAccelerationClamped =
                    IsFinite(_rawParentFeedforwardAccelerationMagnitude) &&
                    _rawParentFeedforwardAccelerationMagnitude > _profile.MaxParentMotionFeedforwardAcceleration + 0.0001f;
                if (_parentFeedforwardAccelerationClamped)
                    _parentFeedforwardAccelerationClampCount++;

                _parentFeedforwardAcceleration = ClampMagnitude(
                    rawParentFeedforwardAcceleration,
                    _profile.MaxParentMotionFeedforwardAcceleration);
                if (!IsFinite(_parentFeedforwardAcceleration))
                    throw new InvalidOperationException("PARENT_MOTION_FEEDFORWARD produced non-finite acceleration.");
                _chestBody.AddForce(_parentFeedforwardAcceleration, ForceMode.Acceleration);
                _lastLinearAcceleration = _parentFeedforwardAcceleration;
            }
            else
            {
                // Common translation was already issued once to BOTH bodies by the receiver.
                _rawParentFeedforwardAccelerationMagnitude = 0f;
                _parentFeedforwardAcceleration = Vector3.zero;
                _lastLinearAcceleration = DynamicPelvisReceiverSelected ? _pelvisReceiver.Report.CommonA : Vector3.zero;
            }

            // v0.1.10：目标相对角速度继续来自 Render/LateUpdate 中直接对 targetRelative pose 求导并低通，
            // 然后 sample-and-hold 到物理时钟；物理 bodyRelative 角速度仍来自 FixedUpdate 开头 settled
            // relative pose 的同相位差分。这样 target clock 与 physics clock 各自在自己的时间基准上求导。
            Quaternion currentRelative = _settledBodyRelativeRotation;
            Quaternion targetRelative = Quaternion.Inverse(_sampledPelvisRotation) * _sampledChestRotation;
            Quaternion relativeError = targetRelative * Quaternion.Inverse(currentRelative);
            if (relativeError.w < 0f)
            {
                relativeError.x = -relativeError.x;
                relativeError.y = -relativeError.y;
                relativeError.z = -relativeError.z;
                relativeError.w = -relativeError.w;
            }

            float w = Mathf.Clamp(relativeError.w, -1f, 1f);
            float angleRadians = 2f * Mathf.Acos(w);
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            Vector3 relativeAxisLocal;
            if (sinHalf > 0.0001f)
            {
                relativeAxisLocal = new Vector3(
                    relativeError.x / sinHalf,
                    relativeError.y / sinHalf,
                    relativeError.z / sinHalf);
            }
            else
            {
                relativeAxisLocal = Vector3.zero;
                angleRadians = 0f;
            }
            Vector3 errorAxisWorld = _settledPelvisRotation * relativeAxisLocal;

            // B's source-local relative velocity belongs to the same governed parent as its Chest reference.
            Quaternion relativeVelocityFrame = DynamicPelvisReceiverSelected ? _pelvisReceiver.ControlPelvisRotation : _sampledPelvisRotation;
            _targetRelativeAngularVelocity = relativeVelocityFrame * _sampledTargetRelativeAngularVelocityLocal;
            _bodyRelativeAngularVelocity = _fixedStepBodyRelativeAngularVelocity;
            _relativeAngularVelocityError = _targetRelativeAngularVelocity - _bodyRelativeAngularVelocity;

            // v0.3.11 taps preserve the exact frozen P+D command while exposing its components read-only.
            _angularPAcceleration = errorAxisWorld * (angleRadians * _profile.AngularSpring);
            _angularDAcceleration = _relativeAngularVelocityError * _profile.AngularDamper;
            Vector3 rawAngularAcceleration = _angularPAcceleration + _angularDAcceleration;
            _rawAngularAccelerationMagnitude = rawAngularAcceleration.magnitude;
            _lastAngularAccelerationClamped =
                IsFinite(_rawAngularAccelerationMagnitude) &&
                _rawAngularAccelerationMagnitude > _profile.MaxAngularAcceleration + 0.0001f;
            if (_lastAngularAccelerationClamped)
                _angularAccelerationClampCount++;

            Vector3 angularAcceleration = ClampMagnitude(
                rawAngularAcceleration,
                _profile.MaxAngularAcceleration);
            _angularAccelerationHeadroom = Mathf.Max(0f, _profile.MaxAngularAcceleration - angularAcceleration.magnitude);

            if (!IsFinite(angularAcceleration))
                throw new InvalidOperationException(
                    "RENDER_BRIDGED_RELATIVE_JOINT_PD produced non-finite angular acceleration.");

            _lastAngularAcceleration = angularAcceleration;
            _chestBody.AddTorque(angularAcceleration, ForceMode.Acceleration);

            // 同相位锚点速度诊断。旧字段继续保留，便于和 v0.1.6/v0.1.7 日志对照；
            // v0.1.8 正式门槛优先看 measuredAnchorRelVel，而不是 Rigidbody.velocity。
            Vector3 targetPelvisAnchorWorld = _sampledPelvisPosition + _sampledPelvisRotation * _virtualPelvisAnchorLocal;
            _targetConnectedAnchorVelocity =
                _sampledPelvisLinearVelocity +
                Vector3.Cross(
                    _sampledPelvisAngularVelocity,
                    targetPelvisAnchorWorld - _sampledPelvisPosition);
            _actualConnectedAnchorVelocity = _measuredPelvisAnchorVelocity;
            _actualChestAnchorVelocity = _measuredChestAnchorVelocity;
            _anchorVelocityError = _targetConnectedAnchorVelocity - _actualChestAnchorVelocity;

            Vector3 velocity = _chestBody.velocity;
            // B must keep its linear ledger force-based: a direct child velocity clamp
            // would be an unreported external impulse. Native maxAngularVelocity remains set.
            if (!DynamicPelvisReceiverSelected && velocity.sqrMagnitude > _profile.MaxLinearVelocity * _profile.MaxLinearVelocity)
                _chestBody.velocity = velocity.normalized * _profile.MaxLinearVelocity;

            Vector3 angularVelocity = _chestBody.angularVelocity;
            if (!DynamicPelvisReceiverSelected && angularVelocity.sqrMagnitude > _profile.MaxAngularVelocity * _profile.MaxAngularVelocity)
                _chestBody.angularVelocity = angularVelocity.normalized * _profile.MaxAngularVelocity;
        }

        private void CaptureSolvedCharacterJointReaction()
        {
            _jointCurrentForceWorld = Vector3.zero; _jointReactionReadbackValid = false;
            _jointCurrentTorqueWorld = Vector3.zero;
            _jointChestComLeverWorld = Vector3.zero;
            _jointPelvisComLeverWorld = Vector3.zero;
            _jointReactionConstraintAnchorErrorMeters = 0f;
            _jointForceMomentWorld = Vector3.zero;
            _jointCombinedMomentWorld = Vector3.zero;
            _jointTorqueAngularAccelerationProxyWorld = Vector3.zero;
            _jointForceMomentAngularAccelerationProxyWorld = Vector3.zero;
            _jointCombinedAngularAccelerationProxyWorld = Vector3.zero;

            if (_characterJoint == null ||
                _chestBody == null || _chestObject == null ||
                _pelvisBody == null || _pelvisObject == null)
                return;

            try
            {
                // Unity exposes the solver's settled reaction from the previous physics step.
                // v0.3.15 captures BOTH body-side levers at this exact readback instant.
                Vector3 force = _characterJoint.currentForce;
                Vector3 torque = _characterJoint.currentTorque;
                Vector3 chestAnchorWorld = ChestPoint(_characterJointChestAnchorLocal);
                Vector3 pelvisAnchorWorld = PelvisPoint(_characterJointPelvisAnchorLocal);
                Vector3 chestLever = chestAnchorWorld - _chestBody.worldCenterOfMass;
                Vector3 pelvisLever = pelvisAnchorWorld - _pelvisBody.worldCenterOfMass;
                float constraintError = Vector3.Distance(chestAnchorWorld, pelvisAnchorWorld);
                Vector3 forceMoment = Vector3.Cross(chestLever, force);
                Vector3 combinedMoment = torque + forceMoment;

                if (!IsFinite(force) || !IsFinite(torque) ||
                    !IsFinite(chestLever) || !IsFinite(pelvisLever) || !IsFinite(constraintError) ||
                    !IsFinite(forceMoment) || !IsFinite(combinedMoment))
                    return;

                Vector3 torqueA = TorqueToAngularAccelerationProxy(_chestBody, torque);
                Vector3 forceMomentA = TorqueToAngularAccelerationProxy(_chestBody, forceMoment);
                Vector3 combinedA = TorqueToAngularAccelerationProxy(_chestBody, combinedMoment);
                if (!IsFinite(torqueA) || !IsFinite(forceMomentA) || !IsFinite(combinedA))
                    return;

                _jointCurrentForceWorld = force;
                _jointCurrentTorqueWorld = torque;
                _jointChestComLeverWorld = chestLever;
                _jointPelvisComLeverWorld = pelvisLever;
                _jointReactionConstraintAnchorErrorMeters = constraintError;
                _jointForceMomentWorld = forceMoment;
                _jointCombinedMomentWorld = combinedMoment;
                _jointTorqueAngularAccelerationProxyWorld = torqueA;
                _jointForceMomentAngularAccelerationProxyWorld = forceMomentA;
                _jointCombinedAngularAccelerationProxyWorld = combinedA;
                _jointReactionReadbackValid = true;
            }
            catch
            {
                // Diagnostic capability must never gate the frozen production torso.
            }
        }

        private static Vector3 TorqueToAngularAccelerationProxy(Rigidbody body, Vector3 torqueWorld)
        {
            if (body == null || !IsFinite(torqueWorld))
                return Vector3.zero;

            Vector3 tensor = body.inertiaTensor;
            if (!IsFinite(tensor) || tensor.x <= 1e-7f || tensor.y <= 1e-7f || tensor.z <= 1e-7f)
                return Vector3.zero;

            Quaternion inertiaWorld = body.rotation * body.inertiaTensorRotation;
            Vector3 localTorque = Quaternion.Inverse(inertiaWorld) * torqueWorld;
            Vector3 localA = new Vector3(localTorque.x / tensor.x, localTorque.y / tensor.y, localTorque.z / tensor.z);
            return inertiaWorld * localA;
        }

        private Vector3 ChestPoint(Vector3 local) => DynamicPelvisReceiverSelected ? _chestBody.position + _chestBody.rotation * local : _chestObject.transform.TransformPoint(local);
        private Vector3 PelvisPoint(Vector3 local) => DynamicPelvisReceiverSelected ? _pelvisBody.position + _pelvisBody.rotation * local : _pelvisObject.transform.TransformPoint(local);

        private bool ValidateCharacterJointAnchorMode()
        {
            if (_pelvisObject == null || _chestObject == null || _chestBody == null)
                return false;

            Vector3 chestWorld = ChestPoint(_characterJointChestAnchorLocal);
            Vector3 pelvisWorld = PelvisPoint(_characterJointPelvisAnchorLocal);
            if (!IsFinite(chestWorld) || !IsFinite(pelvisWorld) || Vector3.Distance(chestWorld, pelvisWorld) > 0.00005f)
                return false;

            if (CharacterJointAnchorMode == TwoBodyCharacterJointAnchorMode.ChestCenterOfMass)
                return Vector3.Distance(chestWorld, _chestBody.worldCenterOfMass) <= 0.00005f;

            Vector3 sourceChestWorld = ChestPoint(_virtualChestAnchorLocal);
            Vector3 sourcePelvisWorld = PelvisPoint(_virtualPelvisAnchorLocal);
            return Vector3.Distance(chestWorld, sourceChestWorld) <= 0.00005f &&
                   Vector3.Distance(pelvisWorld, sourcePelvisWorld) <= 0.00005f;
        }

        private bool ValidateConfiguredCharacterJointAnchors()
        {
            if (_characterJoint == null || _pelvisObject == null || _chestObject == null || _chestBody == null)
                return false;

            Vector3 chestLocal = _characterJoint.anchor;
            Vector3 pelvisLocal = _characterJoint.connectedAnchor;
            if (!IsFinite(chestLocal) || !IsFinite(pelvisLocal))
                return false;
            if (Vector3.Distance(chestLocal, _characterJointChestAnchorLocal) > 0.00005f ||
                Vector3.Distance(pelvisLocal, _characterJointPelvisAnchorLocal) > 0.00005f)
                return false;

            Vector3 chestWorld = ChestPoint(chestLocal);
            Vector3 pelvisWorld = PelvisPoint(pelvisLocal);
            if (Vector3.Distance(chestWorld, pelvisWorld) > 0.00005f)
                return false;

            if (CharacterJointAnchorMode == TwoBodyCharacterJointAnchorMode.ChestCenterOfMass)
                return Vector3.Distance(chestWorld, _chestBody.worldCenterOfMass) <= 0.00005f;

            return Vector3.Distance(chestLocal, _virtualChestAnchorLocal) <= 0.00005f &&
                   Vector3.Distance(pelvisLocal, _virtualPelvisAnchorLocal) <= 0.00005f;
        }

        private void ConfigureCharacterJoint()
        {
            if (_characterJoint == null)
                throw new InvalidOperationException("CharacterJoint creation returned null.");

            _characterJoint.connectedBody = _pelvisBody;
            _characterJoint.autoConfigureConnectedAnchor = false;
            _characterJoint.anchor = _characterJointChestAnchorLocal;
            _characterJoint.connectedAnchor = _characterJointPelvisAnchorLocal;

            // Proxy 局部坐标由 frameRotation 建立：Y=角色 up，X=角色 right，Z=角色 forward。
            // CharacterJoint.axis 为 twist 轴。v0.1.8 仍保持 v0.1.5 已验证的宽限位/轴定义，避免把 joint-frame 变量重新混入收尾诊断。
            _characterJoint.axis = Vector3.up;
            _characterJoint.swingAxis = Vector3.right;
            _characterJoint.enableCollision = false;
            _characterJoint.enablePreprocessing = true;

            _jointTwistAxis = SafeNormalize(_characterJoint.axis, Vector3.up);
            _jointSwing1Axis = SafeNormalize(
                _characterJoint.swingAxis - Vector3.Project(_characterJoint.swingAxis, _jointTwistAxis),
                Vector3.right);
            _jointSwing2Axis = SafeNormalize(Vector3.Cross(_jointTwistAxis, _jointSwing1Axis), Vector3.forward);
            _jointReferenceRelativeRotation = Quaternion.Inverse(_pelvisBody.rotation) * _chestBody.rotation;

            float wide = _profile.WideJointLimitDegrees;
            _effectiveLowTwistLimit = _profile.WideJointLimitIsolation ? -wide : _profile.ChestLowTwistLimitDegrees;
            _effectiveHighTwistLimit = _profile.WideJointLimitIsolation ? wide : _profile.ChestHighTwistLimitDegrees;
            _effectiveSwing1Limit = _profile.WideJointLimitIsolation ? wide : _profile.ChestSwing1LimitDegrees;
            _effectiveSwing2Limit = _profile.WideJointLimitIsolation ? wide : _profile.ChestSwing2LimitDegrees;

            SoftJointLimit lowTwist = _characterJoint.lowTwistLimit;
            lowTwist.limit = _effectiveLowTwistLimit;
            lowTwist.bounciness = 0f;
            _characterJoint.lowTwistLimit = lowTwist;

            SoftJointLimit highTwist = _characterJoint.highTwistLimit;
            highTwist.limit = _effectiveHighTwistLimit;
            highTwist.bounciness = 0f;
            _characterJoint.highTwistLimit = highTwist;

            SoftJointLimit swing1 = _characterJoint.swing1Limit;
            swing1.limit = _effectiveSwing1Limit;
            swing1.bounciness = 0f;
            _characterJoint.swing1Limit = swing1;

            SoftJointLimit swing2 = _characterJoint.swing2Limit;
            swing2.limit = _effectiveSwing2Limit;
            swing2.bounciness = 0f;
            _characterJoint.swing2Limit = swing2;
        }

        private void MeasureJointAngles(Quaternion relativeRotation, out float twist, out float swing1, out float swing2)
        {
            Quaternion delta = Quaternion.Inverse(_jointReferenceRelativeRotation) * relativeRotation;
            Vector3 rotationVectorDegrees = QuaternionToRotationVectorDegrees(delta);
            twist = Vector3.Dot(rotationVectorDegrees, _jointTwistAxis);
            swing1 = Vector3.Dot(rotationVectorDegrees, _jointSwing1Axis);
            swing2 = Vector3.Dot(rotationVectorDegrees, _jointSwing2Axis);
        }

        private float ComputeLimitProximity(float twist, float swing1, float swing2)
        {
            float twistLimit = twist >= 0f ? Mathf.Max(1f, _effectiveHighTwistLimit) : Mathf.Max(1f, -_effectiveLowTwistLimit);
            float t = Mathf.Abs(twist) / twistLimit;
            float s1 = Mathf.Abs(swing1) / Mathf.Max(1f, _effectiveSwing1Limit);
            float s2 = Mathf.Abs(swing2) / Mathf.Max(1f, _effectiveSwing2Limit);
            return Mathf.Max(t, Mathf.Max(s1, s2));
        }

        private static Vector3 QuaternionToRotationVectorDegrees(Quaternion q)
        {
            if (q.w < 0f)
            {
                q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
            }
            float w = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (sinHalf < 0.0001f || angle < 0.0001f)
                return Vector3.zero;
            Vector3 axis = new Vector3(q.x / sinHalf, q.y / sinHalf, q.z / sinHalf);
            return axis * (angle * Mathf.Rad2Deg);
        }

        private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
        {
            if (!IsFinite(value) || value.sqrMagnitude < 0.000001f)
                return fallback.normalized;
            return value.normalized;
        }

        private static Vector3 ClampMagnitude(Vector3 value, float maxMagnitude)
        {
            if (maxMagnitude <= 0f)
                return Vector3.zero;
            float sqr = value.sqrMagnitude;
            float maxSqr = maxMagnitude * maxMagnitude;
            if (sqr <= maxSqr)
                return value;
            float magnitude = Mathf.Sqrt(sqr);
            if (magnitude <= 0.000001f)
                return Vector3.zero;
            return value * (maxMagnitude / magnitude);
        }

        private void IgnoreSourceCharacterColliders(Transform sourceRoot)
        {
            if (sourceRoot == null)
                throw new InvalidOperationException("SOURCE_COLLIDER_ROOT_MISSING");

            Collider[] sourceColliders;
            try
            {
                sourceColliders = sourceRoot.gameObject.GetComponentsInChildren<Collider>(true);
            }
            catch (Exception e)
            {
                throw new InvalidOperationException("SOURCE_COLLIDER_ENUMERATION_FAILED", e);
            }
            if (sourceColliders == null)
                throw new InvalidOperationException("SOURCE_COLLIDER_ENUMERATION_RETURNED_NULL");

            for (int i = 0; i < sourceColliders.Length; i++)
            {
                Collider source = sourceColliders[i];
                if (source == null || source == _pelvisCollider || source == _chestCollider)
                    continue;
                RegisterIgnorePair(_pelvisCollider, source);
                RegisterIgnorePair(_chestCollider, source);
            }
        }

        private void RegisterIgnorePair(Collider a, Collider b)
        {
            if (a == null || b == null || a == b)
                return;
            try
            {
                UnityEngine.Physics.IgnoreCollision(a, b, true);
                _ignoredPairs.Add(new IgnoredPair { A = a, B = b });
            }
            catch
            {
                _report.IgnoreFailureCount++;
            }
        }

        private void ResetPoseInternal()
        {
            _pelvisLegReference.Invalidate();
            if (!_sampleValid || _pelvisBody == null || _chestBody == null)
                return;
            if (_pelvisReceiver.Report.Selected)
            {
                PelvisDynamicReceiverIsolation.SynchronizeKinematicPose(_pelvisBody, _sampledPelvisPosition, _sampledPelvisRotation);
                PelvisDynamicReceiverIsolation.SynchronizeKinematicPose(_chestBody, _sampledChestPosition, _sampledChestRotation);
                return;
            }
            _pelvisBody.position = _sampledPelvisPosition;
            _pelvisBody.rotation = _sampledPelvisRotation;
            _chestBody.position = _sampledChestPosition;
            _chestBody.rotation = _sampledChestRotation;
            _pelvisBody.velocity = Vector3.zero;
            _pelvisBody.angularVelocity = Vector3.zero;
            _chestBody.velocity = Vector3.zero;
            _chestBody.angularVelocity = Vector3.zero;
        }

        private void UpdateReport(string detail)
        {
            _report.Built = IsBuilt;
            _report.DriveEnabled = _driveEnabled;
            _report.GravityEnabled = _gravityEnabled;
            _report.CollidersEnabled = _collidersEnabled;
            _report.VisualEnabled = _visualEnabled;
            _report.SourceAlive = _audit?.Pelvis != null && _audit.Chest != null;
            _report.FiniteState = HasFinitePhysicsState();
            _report.TargetName = _audit?.TargetName;
            _report.TargetPath = _audit?.TargetPath;
            _report.RootName = _root != null ? _root.name : null;
            _report.TorsoLength = _audit?.PelvisToChestDistance ?? 0f;
            _report.PelvisSize = _pelvisCollider != null ? _pelvisCollider.size : Vector3.zero;
            _report.ChestSize = _chestCollider != null ? _chestCollider.size : Vector3.zero;
            _report.IgnorePairCount = _ignoredPairs.Count;
            _report.LastDetail = detail;

            if (_pelvisBody != null && _chestBody != null && _sampleValid)
            {
                Quaternion currentRelative = _driveEnabled
                    ? _settledBodyRelativeRotation
                    : Quaternion.Inverse(_pelvisBody.rotation) * _chestBody.rotation;
                Quaternion targetRelative = Quaternion.Inverse(_sampledPelvisRotation) * _sampledChestRotation;
                _report.RotationErrorDegrees = Quaternion.Angle(currentRelative, targetRelative);
                _report.PelvisPositionErrorMeters = Vector3.Distance(_pelvisBody.position, _sampledPelvisPosition);
                _report.ChestPositionErrorMeters = Vector3.Distance(_chestBody.position, _sampledChestPosition);
                _report.TargetChestLinearVelocity = _sampledChestLinearVelocity;
                _report.TargetChestAngularVelocity = _sampledChestAngularVelocity;
                _report.TargetPelvisLinearVelocity = _sampledPelvisLinearVelocity;
                _report.TargetPelvisAngularVelocity = _sampledPelvisAngularVelocity;
                _report.TargetChestLinearAcceleration = _sampledChestLinearAcceleration;
                _report.TargetChestAngularAcceleration = _sampledChestAngularAcceleration;
                _report.TargetPelvisLinearAcceleration = _sampledPelvisLinearAcceleration;
                _report.TargetPelvisAngularAcceleration = _sampledPelvisAngularAcceleration;
                _report.PelvisVelocity = _pelvisBody.velocity;
                _report.PelvisAngularVelocity = _pelvisBody.angularVelocity;
                _report.EstimatedPelvisLinearVelocity = _estimatedPelvisBodyLinearVelocity;
                _report.EstimatedPelvisAngularVelocity = _estimatedPelvisBodyAngularVelocity;
                _report.ParentTransportLinearVelocity = _parentTransportLinearVelocity;
                _report.PhysicsParentTransportLinearVelocity = _physicsParentTransportLinearVelocity;
                _report.ParentTransportConsistencyError = _parentTransportConsistencyError;
                _report.ParentTransportConsistencyErrorMagnitude = _parentTransportConsistencyErrorMagnitude;
                _report.ParentFeedforwardReferenceMode = CurrentParentTranslationReference;
                _report.ParentFeedforwardReferenceVelocity = _parentFeedforwardReferenceVelocity;
                _report.SourceRelativeLinearVelocity = _sourceRelativeLinearVelocity;
                _report.TargetRelativeAngularVelocity = _targetRelativeAngularVelocity;
                _report.BodyRelativeAngularVelocity = _bodyRelativeAngularVelocity;
                _report.RelativeAngularVelocityError = _relativeAngularVelocityError;
                _report.ChestVelocity = _chestBody.velocity;
                _report.ChestAngularVelocity = _chestBody.angularVelocity;
                _report.FixedStepPelvisTransportLinearVelocity = _fixedStepPelvisTransportLinearVelocity;
                _report.FixedStepPelvisTransportAngularVelocity = _fixedStepPelvisTransportAngularVelocity;
                _report.FixedStepTargetChestComVelocity = _fixedStepTargetChestComVelocity;
                _report.AngularVelocityMode = "RENDER_SAMPLED_TARGET_RELATIVE_PLUS_FIXED_SETTLED_BODY_DELTA";
                _report.DrivePhaseSyncMode = "FIXEDUPDATE_PARENT_PLUS_RENDER_SAMPLE_HOLD_VELOCITY_AND_POSE_BRIDGE_CONSISTENT_PARENT_FF";
                _report.SourceVelocityBridgeMode = "RENDER_SAMPLE_HOLD";
                _report.SourceVelocitySampleSequence = _sourceVelocitySampleSequence;
                _report.SourceVelocitySampleFreshThisFixedStep = _sourceVelocitySampleFreshThisFixedStep;
                _report.SourceVelocitySampleAgeSeconds = _sourceVelocitySampleAgeSeconds;
                _report.SampledTargetRelativeAngularVelocity = _sampledPelvisRotation * _sampledTargetRelativeAngularVelocityLocal;
                _report.PoseBridgeMode = RenderToPhysicsPoseBridge.ModeName;
                _report.PoseBridgeCommandLinearVelocity = _poseBridge.CommandLinearVelocity;
                _report.PoseBridgeCommandAngularVelocity = _poseBridge.CommandAngularVelocity;
                _report.PoseBridgePositionCorrectionVelocity = _poseBridge.PositionCorrectionVelocity;
                _report.PoseBridgeRotationCorrectionAngularVelocity = _poseBridge.RotationCorrectionAngularVelocity;
                _report.PoseBridgePositionErrorMeters = _poseBridge.PositionErrorMeters;
                _report.PoseBridgeRotationErrorDegrees = _poseBridge.RotationErrorDegrees;
                _report.PoseBridgePredictionAgeSeconds = _poseBridge.PredictionAgeSeconds;
                _report.PoseBridgePositionCorrectionClamped = _poseBridge.PositionCorrectionClamped;
                _report.PoseBridgeRotationCorrectionClamped = _poseBridge.RotationCorrectionClamped;
                _report.PoseBridgePositionCorrectionClampCount = _poseBridge.PositionCorrectionClampCount;
                _report.PoseBridgeRotationCorrectionClampCount = _poseBridge.RotationCorrectionClampCount;
            }

            _report.MeasuredPelvisLinearVelocity = _measuredPelvisLinearVelocity;
            _report.MeasuredChestLinearVelocity = _measuredChestLinearVelocity;
            _report.MeasuredPelvisAngularVelocity = _measuredPelvisAngularVelocity;
            _report.MeasuredChestAngularVelocity = _measuredChestAngularVelocity;
            _report.MeasuredPelvisAnchorVelocity = _measuredPelvisAnchorVelocity;
            _report.MeasuredChestAnchorVelocity = _measuredChestAnchorVelocity;
            _report.MeasuredAnchorRelativeVelocity = _measuredAnchorRelativeVelocity;
            _report.MeasuredTransportReferenceVelocity = _measuredTransportReferenceVelocity;
            _report.MeasuredChestTransportVelocityError = _measuredChestTransportVelocityError;
            _report.MotionPhase = _motionPhase;
            _report.MotionTransition = _motionTransition;
            _report.MotionTargetSpeed = _motionTargetSpeed;
            _report.MotionSignedSpeedAcceleration = _motionSignedSpeedAcceleration;
            CopySourceTurnDiagnosticsToReport();

            _report.DriveBackend = DriveBackend;
            _report.ConstraintBackend = ConstraintBackend;
            _report.CharacterJointPresent = _characterJoint != null;
            _report.CharacterJointConnectedBodyMatch =
                _characterJoint != null && _characterJoint.connectedBody == _pelvisBody;
            _report.ChestLowTwistLimitDegrees = _effectiveLowTwistLimit;
            _report.ChestHighTwistLimitDegrees = _effectiveHighTwistLimit;
            _report.ChestSwing1LimitDegrees = _effectiveSwing1Limit;
            _report.ChestSwing2LimitDegrees = _effectiveSwing2Limit;
            _report.JointLimitMode = _profile.WideJointLimitIsolation ? "WIDE_ISOLATION" : "ANATOMICAL";

            if (_pelvisBody != null && _chestBody != null && _sampleValid)
            {
                Quaternion currentRelative = _driveEnabled
                    ? _settledBodyRelativeRotation
                    : Quaternion.Inverse(_pelvisBody.rotation) * _chestBody.rotation;
                Quaternion targetRelative = Quaternion.Inverse(_sampledPelvisRotation) * _sampledChestRotation;
                MeasureJointAngles(currentRelative, out _report.JointCurrentTwistDegrees, out _report.JointCurrentSwing1Degrees, out _report.JointCurrentSwing2Degrees);
                MeasureJointAngles(targetRelative, out _report.JointTargetTwistDegrees, out _report.JointTargetSwing1Degrees, out _report.JointTargetSwing2Degrees);
                _report.JointLimitProximity = ComputeLimitProximity(_report.JointCurrentTwistDegrees, _report.JointCurrentSwing1Degrees, _report.JointCurrentSwing2Degrees);
                _report.TargetJointLimitProximity = ComputeLimitProximity(_report.JointTargetTwistDegrees, _report.JointTargetSwing1Degrees, _report.JointTargetSwing2Degrees);
            }

            _report.AngularAccelerationClamped = _lastAngularAccelerationClamped;
            _report.AngularAccelerationClampCount = _angularAccelerationClampCount;
            _report.RawAngularAccelerationMagnitude = _rawAngularAccelerationMagnitude;
            _report.ParentMotionFeedforwardEnabled = _parentMotionFeedforwardEnabled && !DynamicPelvisReceiverSelected;
            _report.CharacterJointAnchorMode = CharacterJointAnchorMode.ToString();
            _report.CharacterJointAnchorModeValidated = _characterJointAnchorModeValidated;
            if (_pelvisObject != null && _chestObject != null && _chestBody != null)
            {
                Vector3 constraintChestWorld = ChestPoint(_characterJointChestAnchorLocal);
                Vector3 constraintPelvisWorld = PelvisPoint(_characterJointPelvisAnchorLocal);
                _report.CharacterJointConstraintAnchorErrorMeters = Vector3.Distance(constraintChestWorld, constraintPelvisWorld);
                _report.CharacterJointChestComLeverMeters = Vector3.Distance(constraintChestWorld, _chestBody.worldCenterOfMass);
            }
            _report.ParentFeedforwardAccelerationClamped = _parentFeedforwardAccelerationClamped;
            _report.ParentFeedforwardAccelerationClampCount = _parentFeedforwardAccelerationClampCount;
            _report.RawParentFeedforwardAccelerationMagnitude = _rawParentFeedforwardAccelerationMagnitude;
            _report.ParentFeedforwardAcceleration = _parentFeedforwardAcceleration;
            _report.TargetConnectedAnchorVelocity = _targetConnectedAnchorVelocity;
            _report.ActualConnectedAnchorVelocity = _actualConnectedAnchorVelocity;
            _report.ActualChestAnchorVelocity = _actualChestAnchorVelocity;
            _report.AnchorVelocityError = _anchorVelocityError;
            _report.LastLinearAcceleration = _lastLinearAcceleration;
            _report.LastAngularAcceleration = _lastAngularAcceleration;

            if (_pelvisObject != null && _chestObject != null)
            {
                Vector3 a = ChestPoint(_virtualChestAnchorLocal);
                Vector3 b = PelvisPoint(_virtualPelvisAnchorLocal);
                _report.AnchorErrorMeters = Vector3.Distance(a, b);
            }

            if (_driveEnabled)
            {
                _peakChestPositionError = Mathf.Max(_peakChestPositionError, _report.ChestPositionErrorMeters);
                _peakRotationError = Mathf.Max(_peakRotationError, _report.RotationErrorDegrees);
                _peakAnchorError = Mathf.Max(_peakAnchorError, _report.AnchorErrorMeters);
                _peakLinearAcceleration = Mathf.Max(_peakLinearAcceleration, _lastLinearAcceleration.magnitude);
                _peakAngularAcceleration = Mathf.Max(_peakAngularAcceleration, _lastAngularAcceleration.magnitude);
                _peakJointLimitProximity = Mathf.Max(_peakJointLimitProximity, _report.JointLimitProximity);
            }
            _report.PeakChestPositionErrorMeters = _peakChestPositionError;
            _report.PeakRotationErrorDegrees = _peakRotationError;
            _report.PeakAnchorErrorMeters = _peakAnchorError;
            _report.PeakLinearAcceleration = _peakLinearAcceleration;
            _report.PeakAngularAcceleration = _peakAngularAcceleration;
            _report.PeakJointLimitProximity = _peakJointLimitProximity;
            CopyTraceSessionStatisticsToReport();
            _freezeValidation.CopyToReport(_report);
            _report.TraceEnabled = _traceEnabled;
        }

        private void UpdateMeasuredMotionDiagnostics()
        {
            if (_pelvisBody == null || _chestBody == null || _pelvisObject == null || _chestObject == null)
                return;

            // v0.1.8：本方法只在 FixedUpdate 开头调用。drive=True 时 LateUpdate 不再改 Pelvis，
            // 因此这里读取的是上一物理步已经由 PhysX 求解完成后的 settled state。
            float dt = PhysicsDt;
            Vector3 pelvisPosition = _pelvisBody.position;
            Vector3 chestPosition = _chestBody.position;
            Quaternion pelvisRotation = _pelvisBody.rotation;
            Quaternion chestRotation = _chestBody.rotation;
            Quaternion bodyRelativeRotation = Quaternion.Inverse(pelvisRotation) * chestRotation;
            Vector3 pelvisAnchorPosition = PelvisPoint(_virtualPelvisAnchorLocal);
            Vector3 chestAnchorPosition = ChestPoint(_virtualChestAnchorLocal);

            _settledPelvisRotation = pelvisRotation;
            _settledBodyRelativeRotation = bodyRelativeRotation;

            if (_hasPreviousMeasuredMotionSample && IsFinite(dt) && dt > 0.0001f)
            {
                _measuredPelvisLinearVelocity = ClampMagnitude(
                    (pelvisPosition - _previousMeasuredPelvisPosition) / dt,
                    _profile.MaxLinearVelocity * 2f);
                _measuredChestLinearVelocity = ClampMagnitude(
                    (chestPosition - _previousMeasuredChestPosition) / dt,
                    _profile.MaxLinearVelocity * 2f);
                _measuredPelvisAngularVelocity = ClampMagnitude(
                    ComputeAngularVelocity(_previousMeasuredPelvisRotation, pelvisRotation, dt),
                    _profile.MaxAngularVelocity * 2f);
                _measuredChestAngularVelocity = ClampMagnitude(
                    ComputeAngularVelocity(_previousMeasuredChestRotation, chestRotation, dt),
                    _profile.MaxAngularVelocity * 2f);
                _measuredPelvisAnchorVelocity = ClampMagnitude(
                    (pelvisAnchorPosition - _previousMeasuredPelvisAnchorPosition) / dt,
                    _profile.MaxLinearVelocity * 2f);
                _measuredChestAnchorVelocity = ClampMagnitude(
                    (chestAnchorPosition - _previousMeasuredChestAnchorPosition) / dt,
                    _profile.MaxLinearVelocity * 2f);
                _measuredAnchorRelativeVelocity = _measuredChestAnchorVelocity - _measuredPelvisAnchorVelocity;

                Vector3 bodyRelativeAngularVelocityLocal =
                    ComputeAngularVelocity(_previousSettledBodyRelativeRotation, bodyRelativeRotation, dt);
                _fixedStepBodyRelativeAngularVelocity = ClampMagnitude(
                    pelvisRotation * bodyRelativeAngularVelocityLocal,
                    _profile.MaxAngularVelocity * 2f);

                // 与本次 settled displacement 对齐的是“上一物理步已经提交的 target Chest 速度”，
                // 而不是本 FixedUpdate 即将提交的新目标。这样不会再出现 v0.1.7 的一拍错位比较。
                _measuredTransportReferenceVelocity = _lastAppliedTargetChestComVelocity;
                _measuredChestTransportVelocityError =
                    _measuredTransportReferenceVelocity - _measuredChestLinearVelocity;
            }
            else
            {
                _measuredPelvisLinearVelocity = Vector3.zero;
                _measuredChestLinearVelocity = Vector3.zero;
                _measuredPelvisAngularVelocity = Vector3.zero;
                _measuredChestAngularVelocity = Vector3.zero;
                _measuredPelvisAnchorVelocity = Vector3.zero;
                _measuredChestAnchorVelocity = Vector3.zero;
                _measuredAnchorRelativeVelocity = Vector3.zero;
                _fixedStepBodyRelativeAngularVelocity = Vector3.zero;
                _measuredTransportReferenceVelocity = Vector3.zero;
                _measuredChestTransportVelocityError = Vector3.zero;
            }

            _previousMeasuredPelvisPosition = pelvisPosition;
            _previousMeasuredChestPosition = chestPosition;
            _previousMeasuredPelvisRotation = pelvisRotation;
            _previousMeasuredChestRotation = chestRotation;
            _previousMeasuredPelvisAnchorPosition = pelvisAnchorPosition;
            _previousMeasuredChestAnchorPosition = chestAnchorPosition;
            _previousSettledBodyRelativeRotation = bodyRelativeRotation;
            _hasPreviousMeasuredMotionSample = true;
        }

        private void ResetMeasuredMotionDiagnostics()
        {
            _measuredPelvisLinearVelocity = Vector3.zero;
            _measuredChestLinearVelocity = Vector3.zero;
            _measuredPelvisAngularVelocity = Vector3.zero;
            _measuredChestAngularVelocity = Vector3.zero;
            _measuredPelvisAnchorVelocity = Vector3.zero;
            _measuredChestAnchorVelocity = Vector3.zero;
            _measuredAnchorRelativeVelocity = Vector3.zero;
            _measuredTransportReferenceVelocity = Vector3.zero;
            _measuredChestTransportVelocityError = Vector3.zero;
            _fixedStepBodyRelativeAngularVelocity = Vector3.zero;
            if (_pelvisBody != null && _chestBody != null && _pelvisObject != null && _chestObject != null)
            {
                _previousMeasuredPelvisPosition = _pelvisBody.position;
                _previousMeasuredChestPosition = _chestBody.position;
                _previousMeasuredPelvisRotation = _pelvisBody.rotation;
                _previousMeasuredChestRotation = _chestBody.rotation;
                _previousMeasuredPelvisAnchorPosition = PelvisPoint(_virtualPelvisAnchorLocal);
                _previousMeasuredChestAnchorPosition = ChestPoint(_virtualChestAnchorLocal);
                _settledPelvisRotation = _pelvisBody.rotation;
                _settledBodyRelativeRotation = Quaternion.Inverse(_pelvisBody.rotation) * _chestBody.rotation;
                _previousSettledBodyRelativeRotation = _settledBodyRelativeRotation;
                _hasPreviousMeasuredMotionSample = true;
                }
            else
            {
                _settledPelvisRotation = Quaternion.identity;
                _settledBodyRelativeRotation = Quaternion.identity;
                _previousSettledBodyRelativeRotation = Quaternion.identity;
                _hasPreviousMeasuredMotionSample = false;
            }
        }

        private void UpdateRenderToPhysicsPoseBridge()
        {
            float dt = PhysicsDt;
            bool ok = _poseBridge.Step(
                _sampledPelvisPosition,
                _sampledPelvisRotation,
                _sampledPelvisLinearVelocity,
                _sampledPelvisAngularVelocity,
                _sourceVelocitySampleAgeSeconds,
                dt,
                _profile.PoseBridgeMaxExtrapolationSeconds,
                _profile.PoseBridgePositionCorrectionGain,
                _profile.PoseBridgeMaxPositionCorrectionSpeed,
                _profile.PoseBridgeRotationCorrectionGain,
                _profile.PoseBridgeMaxRotationCorrectionAngularSpeed,
                _profile.MaxLinearVelocity,
                _profile.MaxAngularVelocity);

            if (!ok || !_poseBridge.Initialized)
                throw new InvalidOperationException("RENDER_TO_PHYSICS_POSE_BRIDGE produced invalid command.");
        }

        private void UpdateFixedStepTransportKinematics(Vector3 pelvisCommandPosition, Quaternion pelvisCommandRotation)
        {
            float dt = PhysicsDt;
            Quaternion targetRelative = Quaternion.Inverse(_sampledPelvisRotation) * _sampledChestRotation;
            Vector3 targetChestLocalOffset = Quaternion.Inverse(_sampledPelvisRotation) * (_sampledChestPosition - _sampledPelvisPosition);
            Vector3 bridgedTargetChestPosition = pelvisCommandPosition + pelvisCommandRotation * targetChestLocalOffset;
            Quaternion bridgedTargetChestRotation = pelvisCommandRotation * targetRelative;
            Vector3 targetChestComOffset = bridgedTargetChestRotation * _chestBody.centerOfMass;
            Vector3 targetChestComPosition = bridgedTargetChestPosition + targetChestComOffset;

            // v0.1.11: this is the parent transport velocity the CharacterJoint will actually experience
            // from the Pose Bridge command at the target Chest COM lever arm. Do not include source-relative
            // Chest motion here; that remains the job of the joint geometry + relative angular controller.
            Vector3 physicsParentLeverArm = targetChestComPosition - pelvisCommandPosition;
            Vector3 commandLinearVelocity = _driveEnabled && _poseBridge.Initialized
                ? _poseBridge.CommandLinearVelocity
                : Vector3.zero;
            Vector3 commandAngularVelocity = _driveEnabled && _poseBridge.Initialized
                ? _poseBridge.CommandAngularVelocity
                : Vector3.zero;
            _physicsParentTransportLinearVelocity =
                commandLinearVelocity +
                Vector3.Cross(commandAngularVelocity, physicsParentLeverArm);
            _physicsParentTransportLinearVelocity = ClampMagnitude(
                _physicsParentTransportLinearVelocity,
                _profile.MaxLinearVelocity * 2f);
            _parentTransportConsistencyError =
                _physicsParentTransportLinearVelocity - _parentTransportLinearVelocity;
            _parentTransportConsistencyErrorMagnitude = _parentTransportConsistencyError.magnitude;
            if (!IsFinite(_physicsParentTransportLinearVelocity) ||
                !IsFinite(_parentTransportConsistencyError) ||
                !IsFinite(_parentTransportConsistencyErrorMagnitude))
                throw new InvalidOperationException("PARENT_TRANSPORT_CONSISTENCY produced non-finite diagnostics.");

            if (_hasPreviousFixedStepCommand && IsFinite(dt) && dt > 0.0001f)
            {
                _fixedStepPelvisTransportLinearVelocity = ClampMagnitude(
                    (pelvisCommandPosition - _previousCommandedPelvisPosition) / dt,
                    _profile.MaxLinearVelocity * 2f);
                _fixedStepPelvisTransportAngularVelocity = ClampMagnitude(
                    ComputeAngularVelocity(_previousCommandedPelvisRotation, pelvisCommandRotation, dt),
                    _profile.MaxAngularVelocity * 2f);
                _fixedStepTargetChestComVelocity = ClampMagnitude(
                    (targetChestComPosition - _previousCommandedTargetChestComPosition) / dt,
                    _profile.MaxLinearVelocity * 2f);

                Vector3 targetRelativeAngularVelocityLocal =
                    ComputeAngularVelocity(_previousCommandedTargetRelativeRotation, targetRelative, dt);
                _fixedStepTargetRelativeAngularVelocity = ClampMagnitude(
                    _settledPelvisRotation * targetRelativeAngularVelocityLocal,
                    _profile.MaxAngularVelocity * 2f);
            }
            else
            {
                _fixedStepPelvisTransportLinearVelocity = Vector3.zero;
                _fixedStepPelvisTransportAngularVelocity = Vector3.zero;
                _fixedStepTargetChestComVelocity = Vector3.zero;
                _fixedStepTargetRelativeAngularVelocity = Vector3.zero;
            }

            _previousCommandedPelvisPosition = pelvisCommandPosition;
            _previousCommandedPelvisRotation = pelvisCommandRotation;
            _previousCommandedTargetChestComPosition = targetChestComPosition;
            _previousCommandedTargetRelativeRotation = targetRelative;
            _hasPreviousFixedStepCommand = true;

            // 下一次 FixedUpdate 开头的 settled displacement 与“本次 command-delta 诊断速度”对齐比较。
            // Legacy只用作phase-safe diagnostics；原V04470 B使用此raw Chest command-delta V。当前动态基线从受限共同COM及native锚点反解同钟速度，此字段保留raw对照，旧Chest-only FF仍跳过。
            _lastAppliedTargetChestComVelocity = _fixedStepTargetChestComVelocity;
        }

        private void UpdateSourceVelocityBridgeDiagnostics()
        {
            _sourceVelocitySampleFreshThisFixedStep =
                _sourceVelocitySampleSequence != _lastFixedConsumedSourceVelocitySampleSequence;
            _lastFixedConsumedSourceVelocitySampleSequence = _sourceVelocitySampleSequence;

            float now = CurrentUnscaledTime;
            _sourceVelocitySampleAgeSeconds = IsFinite(now) && IsFinite(_sourceVelocitySampleTime)
                ? Mathf.Max(0f, now - _sourceVelocitySampleTime)
                : 0f;
        }

        private void UpdateRenderSampleHoldControlKinematics()
        {
            // Render-clock source velocities were already clamped and low-pass filtered in SampleSourcePoseInternal(true).
            // Holding them across extra FixedUpdates is intentional: no new render pose means "same latest velocity", not "velocity=0".
            Vector3 targetChestComOffset = _sampledChestRotation * _chestBody.centerOfMass;
            Vector3 targetChestComPosition = _sampledChestPosition + targetChestComOffset;
            Vector3 targetChestComVelocity =
                _sampledChestLinearVelocity +
                Vector3.Cross(_sampledChestAngularVelocity, targetChestComOffset);
            Vector3 targetChestComOffsetFromPelvis = targetChestComPosition - _sampledPelvisPosition;

            _parentTransportLinearVelocity =
                _sampledPelvisLinearVelocity +
                Vector3.Cross(_sampledPelvisAngularVelocity, targetChestComOffsetFromPelvis);
            _sourceRelativeLinearVelocity = targetChestComVelocity - _parentTransportLinearVelocity;
        }

        private void ResetFixedStepTransportKinematics()
        {
            _sourceVelocitySampleFreshThisFixedStep = false;
            _sourceVelocitySampleAgeSeconds = 0f;
            _lastFixedConsumedSourceVelocitySampleSequence = _sourceVelocitySampleSequence;
            _fixedStepPelvisTransportLinearVelocity = Vector3.zero;
            _fixedStepPelvisTransportAngularVelocity = Vector3.zero;
            _fixedStepTargetChestComVelocity = Vector3.zero;
            _fixedStepTargetRelativeAngularVelocity = Vector3.zero;
            _fixedStepBodyRelativeAngularVelocity = Vector3.zero;
            _physicsParentTransportLinearVelocity = Vector3.zero;
            _parentTransportConsistencyError = Vector3.zero;
            _parentTransportConsistencyErrorMagnitude = 0f;
            _parentFeedforwardReferenceVelocity = Vector3.zero;
            _lastAppliedTargetChestComVelocity = Vector3.zero;
            _hasPreviousFixedStepCommand = false;

            if (_sampleValid && _chestBody != null)
            {
                Vector3 targetChestComOffset = _sampledChestRotation * _chestBody.centerOfMass;
                _previousCommandedPelvisPosition = _poseBridge.Initialized ? _poseBridge.CommandPosition : _sampledPelvisPosition;
                _previousCommandedPelvisRotation = _poseBridge.Initialized ? _poseBridge.CommandRotation : _sampledPelvisRotation;
                Quaternion targetRelative = Quaternion.Inverse(_sampledPelvisRotation) * _sampledChestRotation;
                Vector3 targetChestLocalOffset = Quaternion.Inverse(_sampledPelvisRotation) * (_sampledChestPosition - _sampledPelvisPosition);
                Vector3 bridgedTargetChestPosition = _previousCommandedPelvisPosition + _previousCommandedPelvisRotation * targetChestLocalOffset;
                Quaternion bridgedTargetChestRotation = _previousCommandedPelvisRotation * targetRelative;
                Vector3 bridgedChestComOffset = bridgedTargetChestRotation * _chestBody.centerOfMass;
                _previousCommandedTargetChestComPosition = bridgedTargetChestPosition + bridgedChestComOffset;
                _previousCommandedTargetRelativeRotation = targetRelative;
                _hasPreviousFixedStepCommand = true;
            }
            else
            {
                _previousCommandedPelvisPosition = Vector3.zero;
                _previousCommandedPelvisRotation = Quaternion.identity;
                _previousCommandedTargetChestComPosition = Vector3.zero;
                _previousCommandedTargetRelativeRotation = Quaternion.identity;
            }
        }

        private void UpdateMotionPhaseClassification()
        {
            float dt = PhysicsDt;
            float speed = _sampledPelvisLinearVelocity.magnitude;
            float previousSpeed = _hasPreviousMotionSpeed ? _previousMotionSpeed : speed;
            float signedSpeedAcceleration = (IsFinite(dt) && dt > 0.0001f)
                ? (speed - previousSpeed) / dt
                : 0f;
            _previousMotionSpeed = speed;
            _hasPreviousMotionSpeed = true;

            float angularSpeed = _sampledPelvisAngularVelocity.magnitude;
            float angularAcceleration = _sampledPelvisAngularAcceleration.magnitude;
            bool turnEvent =
                angularSpeed >= _profile.MotionTurnAngularVelocityThreshold ||
                (angularSpeed >= 0.25f && angularAcceleration >= _profile.MotionTurnAngularAccelerationThreshold);
            bool linearEvent =
                Mathf.Max(speed, previousSpeed) >= _profile.MotionIdleSpeedThreshold &&
                Mathf.Abs(signedSpeedAcceleration) >= _profile.MotionLinearAccelerationThreshold;

            if (turnEvent)
            {
                _heldTransitionPhase = "TURN";
                _transitionHoldUntil = CurrentUnscaledTime + _profile.MotionTransitionHoldSeconds;
            }
            else if (linearEvent)
            {
                if (signedSpeedAcceleration > 0f && previousSpeed <= _profile.MotionIdleSpeedThreshold)
                    _heldTransitionPhase = "START";
                else if (signedSpeedAcceleration < 0f && speed <= _profile.MotionSteadyMinSpeed)
                    _heldTransitionPhase = "STOP";
                else
                    _heldTransitionPhase = signedSpeedAcceleration >= 0f ? "ACCEL" : "DECEL";
                _transitionHoldUntil = CurrentUnscaledTime + _profile.MotionTransitionHoldSeconds;
            }

            _motionTransition = CurrentUnscaledTime <= _transitionHoldUntil;
            if (_motionTransition)
                _motionPhase = _heldTransitionPhase;
            else if (speed <= _profile.MotionIdleSpeedThreshold)
                _motionPhase = "IDLE";
            else if (speed >= _profile.MotionSteadyMinSpeed)
                _motionPhase = "STEADY";
            else
                _motionPhase = "LOW_SPEED";

            _motionTargetSpeed = speed;
            _motionSignedSpeedAcceleration = signedSpeedAcceleration;
        }

        private void ResetMotionPhaseClassifier()
        {
            _motionPhase = "IDLE";
            _motionTransition = false;
            _motionTargetSpeed = 0f;
            _motionSignedSpeedAcceleration = 0f;
            _previousMotionSpeed = 0f;
            _hasPreviousMotionSpeed = false;
            _transitionHoldUntil = 0f;
            _heldTransitionPhase = "TRANSITION";
        }

        private void ResetTraceSessionStatistics()
        {
            _traceSteadySampleCount = 0;
            _traceTransitionSampleCount = 0;
            _traceIdleSampleCount = 0;
            _traceSteadyRotationErrorSum = 0f;
            _traceSteadyRotationErrorMax = 0f;
            _traceTransitionRotationErrorSum = 0f;
            _traceTransitionRotationErrorMax = 0f;
            _traceSteadyChestPositionErrorSum = 0f;
            _traceSteadyChestPositionErrorMax = 0f;
            _traceTransitionChestPositionErrorSum = 0f;
            _traceTransitionChestPositionErrorMax = 0f;
            _traceSteadyMeasuredTransportErrorSum = 0f;
            _traceSteadyMeasuredTransportErrorMax = 0f;
            _traceTransitionMeasuredTransportErrorSum = 0f;
            _traceTransitionMeasuredTransportErrorMax = 0f;
            _traceTurnSampleCount = 0;
            _traceTurnRotationErrorSum = 0f;
            _traceTurnRotationErrorMax = 0f;
            _traceSteadyAngularClampSampleCount = 0;
            _traceTransitionAngularClampSampleCount = 0;
            _traceTurnAngularClampSampleCount = 0;
            _traceSteadyAnchorRelativeSpeedSum = 0f;
            _traceSteadyAnchorRelativeSpeedMax = 0f;
            _traceTransitionAnchorRelativeSpeedSum = 0f;
            _traceTransitionAnchorRelativeSpeedMax = 0f;
            _traceTurnAnchorRelativeSpeedSum = 0f;
            _traceTurnAnchorRelativeSpeedMax = 0f;
            _traceSteadyParentTransportConsistencySum = 0f;
            _traceSteadyParentTransportConsistencyMax = 0f;
            _traceTransitionParentTransportConsistencySum = 0f;
            _traceTransitionParentTransportConsistencyMax = 0f;
            _traceTurnParentTransportConsistencySum = 0f;
            _traceTurnParentTransportConsistencyMax = 0f;
            _traceFreshSourceVelocityFixedStepCount = 0;
            _traceHeldSourceVelocityFixedStepCount = 0;
            CopyTraceSessionStatisticsToReport();
        }

        private void UpdateTraceSessionStatistics()
        {
            if (!_traceEnabled || !_driveEnabled)
                return;

            float measuredTransportError = _measuredChestTransportVelocityError.magnitude;
            float anchorRelativeSpeed = _measuredAnchorRelativeVelocity.magnitude;
            float transportConsistency = _parentTransportConsistencyErrorMagnitude;
            if (_sourceVelocitySampleFreshThisFixedStep)
                _traceFreshSourceVelocityFixedStepCount++;
            else
                _traceHeldSourceVelocityFixedStepCount++;
            bool angularClampedThisStep = _lastAngularAccelerationClamped;

            if (_motionTransition)
            {
                _traceTransitionSampleCount++;
                _traceTransitionRotationErrorSum += _report.RotationErrorDegrees;
                _traceTransitionRotationErrorMax = Mathf.Max(_traceTransitionRotationErrorMax, _report.RotationErrorDegrees);
                _traceTransitionChestPositionErrorSum += _report.ChestPositionErrorMeters;
                _traceTransitionChestPositionErrorMax = Mathf.Max(_traceTransitionChestPositionErrorMax, _report.ChestPositionErrorMeters);
                _traceTransitionMeasuredTransportErrorSum += measuredTransportError;
                _traceTransitionMeasuredTransportErrorMax = Mathf.Max(_traceTransitionMeasuredTransportErrorMax, measuredTransportError);
                _traceTransitionAnchorRelativeSpeedSum += anchorRelativeSpeed;
                _traceTransitionAnchorRelativeSpeedMax = Mathf.Max(_traceTransitionAnchorRelativeSpeedMax, anchorRelativeSpeed);
                _traceTransitionParentTransportConsistencySum += transportConsistency;
                _traceTransitionParentTransportConsistencyMax = Mathf.Max(_traceTransitionParentTransportConsistencyMax, transportConsistency);
                if (angularClampedThisStep)
                    _traceTransitionAngularClampSampleCount++;

                if (_motionPhase == "TURN")
                {
                    _traceTurnSampleCount++;
                    _traceTurnRotationErrorSum += _report.RotationErrorDegrees;
                    _traceTurnRotationErrorMax = Mathf.Max(_traceTurnRotationErrorMax, _report.RotationErrorDegrees);
                    _traceTurnAnchorRelativeSpeedSum += anchorRelativeSpeed;
                    _traceTurnAnchorRelativeSpeedMax = Mathf.Max(_traceTurnAnchorRelativeSpeedMax, anchorRelativeSpeed);
                    _traceTurnParentTransportConsistencySum += transportConsistency;
                    _traceTurnParentTransportConsistencyMax = Mathf.Max(_traceTurnParentTransportConsistencyMax, transportConsistency);
                    if (angularClampedThisStep)
                        _traceTurnAngularClampSampleCount++;
                }
            }
            else if (_motionPhase == "STEADY")
            {
                _traceSteadySampleCount++;
                _traceSteadyRotationErrorSum += _report.RotationErrorDegrees;
                _traceSteadyRotationErrorMax = Mathf.Max(_traceSteadyRotationErrorMax, _report.RotationErrorDegrees);
                _traceSteadyChestPositionErrorSum += _report.ChestPositionErrorMeters;
                _traceSteadyChestPositionErrorMax = Mathf.Max(_traceSteadyChestPositionErrorMax, _report.ChestPositionErrorMeters);
                _traceSteadyMeasuredTransportErrorSum += measuredTransportError;
                _traceSteadyMeasuredTransportErrorMax = Mathf.Max(_traceSteadyMeasuredTransportErrorMax, measuredTransportError);
                _traceSteadyAnchorRelativeSpeedSum += anchorRelativeSpeed;
                _traceSteadyAnchorRelativeSpeedMax = Mathf.Max(_traceSteadyAnchorRelativeSpeedMax, anchorRelativeSpeed);
                _traceSteadyParentTransportConsistencySum += transportConsistency;
                _traceSteadyParentTransportConsistencyMax = Mathf.Max(_traceSteadyParentTransportConsistencyMax, transportConsistency);
                if (angularClampedThisStep)
                    _traceSteadyAngularClampSampleCount++;
            }
            else
            {
                _traceIdleSampleCount++;
            }
            CopyTraceSessionStatisticsToReport();
        }

        private void CopyTraceSessionStatisticsToReport()
        {
            _report.TraceSteadySampleCount = _traceSteadySampleCount;
            _report.TraceTransitionSampleCount = _traceTransitionSampleCount;
            _report.TraceIdleSampleCount = _traceIdleSampleCount;
            _report.TraceSteadyRotationErrorMeanDegrees = _traceSteadySampleCount > 0 ? _traceSteadyRotationErrorSum / _traceSteadySampleCount : 0f;
            _report.TraceSteadyRotationErrorMaxDegrees = _traceSteadyRotationErrorMax;
            _report.TraceTransitionRotationErrorMeanDegrees = _traceTransitionSampleCount > 0 ? _traceTransitionRotationErrorSum / _traceTransitionSampleCount : 0f;
            _report.TraceTransitionRotationErrorMaxDegrees = _traceTransitionRotationErrorMax;
            _report.TraceSteadyChestPositionErrorMeanMeters = _traceSteadySampleCount > 0 ? _traceSteadyChestPositionErrorSum / _traceSteadySampleCount : 0f;
            _report.TraceSteadyChestPositionErrorMaxMeters = _traceSteadyChestPositionErrorMax;
            _report.TraceTransitionChestPositionErrorMeanMeters = _traceTransitionSampleCount > 0 ? _traceTransitionChestPositionErrorSum / _traceTransitionSampleCount : 0f;
            _report.TraceTransitionChestPositionErrorMaxMeters = _traceTransitionChestPositionErrorMax;
            _report.TraceSteadyMeasuredTransportErrorMean = _traceSteadySampleCount > 0 ? _traceSteadyMeasuredTransportErrorSum / _traceSteadySampleCount : 0f;
            _report.TraceSteadyMeasuredTransportErrorMax = _traceSteadyMeasuredTransportErrorMax;
            _report.TraceTransitionMeasuredTransportErrorMean = _traceTransitionSampleCount > 0 ? _traceTransitionMeasuredTransportErrorSum / _traceTransitionSampleCount : 0f;
            _report.TraceTransitionMeasuredTransportErrorMax = _traceTransitionMeasuredTransportErrorMax;
            _report.TraceTurnSampleCount = _traceTurnSampleCount;
            _report.TraceTurnRotationErrorMeanDegrees = _traceTurnSampleCount > 0 ? _traceTurnRotationErrorSum / _traceTurnSampleCount : 0f;
            _report.TraceTurnRotationErrorMaxDegrees = _traceTurnRotationErrorMax;
            _report.TraceSteadyAngularClampSampleCount = _traceSteadyAngularClampSampleCount;
            _report.TraceTransitionAngularClampSampleCount = _traceTransitionAngularClampSampleCount;
            _report.TraceTurnAngularClampSampleCount = _traceTurnAngularClampSampleCount;
            _report.TraceSteadyAnchorRelativeSpeedMean = _traceSteadySampleCount > 0 ? _traceSteadyAnchorRelativeSpeedSum / _traceSteadySampleCount : 0f;
            _report.TraceSteadyAnchorRelativeSpeedMax = _traceSteadyAnchorRelativeSpeedMax;
            _report.TraceTransitionAnchorRelativeSpeedMean = _traceTransitionSampleCount > 0 ? _traceTransitionAnchorRelativeSpeedSum / _traceTransitionSampleCount : 0f;
            _report.TraceTransitionAnchorRelativeSpeedMax = _traceTransitionAnchorRelativeSpeedMax;
            _report.TraceTurnAnchorRelativeSpeedMean = _traceTurnSampleCount > 0 ? _traceTurnAnchorRelativeSpeedSum / _traceTurnSampleCount : 0f;
            _report.TraceTurnAnchorRelativeSpeedMax = _traceTurnAnchorRelativeSpeedMax;
            _report.TraceFreshSourceVelocityFixedStepCount = _traceFreshSourceVelocityFixedStepCount;
            _report.TraceHeldSourceVelocityFixedStepCount = _traceHeldSourceVelocityFixedStepCount;
            _report.TraceSteadyParentTransportConsistencyMean = _traceSteadySampleCount > 0 ? _traceSteadyParentTransportConsistencySum / _traceSteadySampleCount : 0f;
            _report.TraceSteadyParentTransportConsistencyMax = _traceSteadyParentTransportConsistencyMax;
            _report.TraceTransitionParentTransportConsistencyMean = _traceTransitionSampleCount > 0 ? _traceTransitionParentTransportConsistencySum / _traceTransitionSampleCount : 0f;
            _report.TraceTransitionParentTransportConsistencyMax = _traceTransitionParentTransportConsistencyMax;
            _report.TraceTurnParentTransportConsistencyMean = _traceTurnSampleCount > 0 ? _traceTurnParentTransportConsistencySum / _traceTurnSampleCount : 0f;
            _report.TraceTurnParentTransportConsistencyMax = _traceTurnParentTransportConsistencyMax;
        }

        private void CopySourceTurnDiagnosticsToReport()
        {
            _report.SourceTurnDiagnosticsMode = SourceTurnDiscontinuityDiagnostics.ModeName;
            _report.SourceTurnSampleDeltaSeconds = _sourceTurnDiagnostics.CurrentSampleDeltaSeconds;
            _report.SourceTurnSampleRotationDeltaDegrees = _sourceTurnDiagnostics.CurrentRotationDeltaDegrees;
            _report.SourceTurnSampleYawDegrees = _sourceTurnDiagnostics.CurrentYawDegrees;
            _report.SourceTurnSampleYawDeltaDegrees = _sourceTurnDiagnostics.CurrentYawDeltaDegrees;
            _report.SourceTurnRawAngularVelocity = _sourceTurnDiagnostics.CurrentRawAngularVelocity;
            _report.SourceTurnRawAngularSpeed = _sourceTurnDiagnostics.CurrentRawAngularSpeed;
            _report.SourceTurnRawAngularAcceleration = _sourceTurnDiagnostics.CurrentRawAngularAcceleration;
            _report.SourceTurnRawAngularAccelerationMagnitude = _sourceTurnDiagnostics.CurrentRawAngularAccelerationMagnitude;
            _report.SourceTurnChestRelativeRotationDeltaDegrees = _sourceTurnDiagnostics.CurrentChestRelativeRotationDeltaDegrees;
            _report.SourceTurnEventActive = _sourceTurnDiagnostics.EventActive;
            _report.SourceTurnCurrentEventId = _sourceTurnDiagnostics.CurrentEventId;
            _report.SourceTurnEventCount = _sourceTurnDiagnostics.EventCount;
            _report.SourceTurnDiscontinuityEventCount = _sourceTurnDiagnostics.DiscontinuityEventCount;
            _report.SourceTurnRapidContinuousEventCount = _sourceTurnDiagnostics.RapidContinuousEventCount;
            _report.SourceTurnMaxSingleSampleRotationDeltaDegrees = _sourceTurnDiagnostics.MaxSingleSampleRotationDeltaDegrees;
            _report.SourceTurnMaxSingleSampleAbsYawDeltaDegrees = _sourceTurnDiagnostics.MaxSingleSampleAbsYawDeltaDegrees;
            _report.SourceTurnMaxRawAngularSpeed = _sourceTurnDiagnostics.MaxRawAngularSpeed;
            _report.SourceTurnMaxRawAngularAcceleration = _sourceTurnDiagnostics.MaxRawAngularAcceleration;
            _report.SourceTurnMaxChestRelativeRotationDeltaDegrees = _sourceTurnDiagnostics.MaxChestRelativeRotationDeltaDegrees;
            _report.SourceTurnMaxResponseRotationErrorDegrees = _sourceTurnDiagnostics.MaxResponseRotationErrorDegrees;
            _report.SourceTurnMaxPoseBridgeRotationErrorDegrees = _sourceTurnDiagnostics.MaxPoseBridgeRotationErrorDegrees;
            _report.SourceTurnMaxPoseBridgeCommandAngularSpeed = _sourceTurnDiagnostics.MaxPoseBridgeCommandAngularSpeed;
            _report.SourceTurnMaxAnchorErrorMeters = _sourceTurnDiagnostics.MaxAnchorErrorMeters;
            _report.SourceTurnAngularClampFixedStepCount = _sourceTurnDiagnostics.AngularClampFixedStepCount;
            _report.SourceTurnLastEventClassification = _sourceTurnDiagnostics.LastEventClassification;
            _report.SourceTurnLastEventSignedYawDeltaDegrees = _sourceTurnDiagnostics.LastEventSignedYawDeltaDegrees;
            _report.SourceTurnLastEventAbsoluteYawDeltaDegrees = _sourceTurnDiagnostics.LastEventAbsoluteYawDeltaDegrees;
            _report.SourceTurnLastEventMaxSingleSampleRotationDeltaDegrees = _sourceTurnDiagnostics.LastEventMaxSingleSampleRotationDeltaDegrees;
            _report.SourceTurnLastEventMaxRawAngularSpeed = _sourceTurnDiagnostics.LastEventMaxRawAngularSpeed;
            _report.SourceTurnLastEventMaxResponseRotationErrorDegrees = _sourceTurnDiagnostics.LastEventMaxResponseRotationErrorDegrees;
        }

        private void EmitTraceIfDue()
        {
            if (!_traceEnabled || !IsBuilt || !_driveEnabled)
                return;
            float now = CurrentUnscaledTime;
            if (now < _nextTraceTime)
                return;
            _nextTraceTime = now + _profile.TraceIntervalSeconds;
            if (_emitLogs) Plugin.Logger?.LogInfo("[ARL][TRACE] " + (_profile.TraceVerboseSnapshots ? _report.GetTraceSummary() : _report.GetTraceSummaryCompact()));
        }

        private void ResetDrivePeaks()
        {
            _peakChestPositionError = 0f;
            _peakRotationError = 0f;
            _peakAnchorError = 0f;
            _peakLinearAcceleration = 0f;
            _peakAngularAcceleration = 0f;
            _peakJointLimitProximity = 0f;
            _lastAngularAccelerationClamped = false;
            _angularAccelerationClampCount = 0;
            _rawAngularAccelerationMagnitude = 0f;
            _angularPAcceleration = Vector3.zero;
            _angularDAcceleration = Vector3.zero;
            _angularAccelerationHeadroom = 0f;
            _jointCurrentForceWorld = Vector3.zero; _jointReactionReadbackValid = false;
            _jointCurrentTorqueWorld = Vector3.zero;
            _jointChestComLeverWorld = Vector3.zero;
            _jointForceMomentWorld = Vector3.zero;
            _jointCombinedMomentWorld = Vector3.zero;
            _jointTorqueAngularAccelerationProxyWorld = Vector3.zero;
            _jointForceMomentAngularAccelerationProxyWorld = Vector3.zero;
            _jointCombinedAngularAccelerationProxyWorld = Vector3.zero;
            _parentFeedforwardAcceleration = Vector3.zero;
            _parentFeedforwardAccelerationClamped = false;
            _parentFeedforwardAccelerationClampCount = 0;
            _rawParentFeedforwardAccelerationMagnitude = 0f;
            _parentFeedforwardReferenceVelocity = Vector3.zero;
            _targetConnectedAnchorVelocity = Vector3.zero;
            _actualConnectedAnchorVelocity = Vector3.zero;
            _actualChestAnchorVelocity = Vector3.zero;
            _anchorVelocityError = Vector3.zero;
        }

        private void UpdatePelvisBodyVelocityEstimate()
        {
            if (_pelvisBody == null)
                return;
            float dt = PhysicsDt;
            Vector3 position = _pelvisBody.position;
            Quaternion rotation = _pelvisBody.rotation;
            if (_hasPreviousPelvisBodySample && IsFinite(dt) && dt > 0.0001f)
            {
                _estimatedPelvisBodyLinearVelocity = ClampMagnitude(
                    (position - _previousPelvisBodyPosition) / dt,
                    _profile.MaxLinearVelocity);
                _estimatedPelvisBodyAngularVelocity = ClampMagnitude(
                    ComputeAngularVelocity(_previousPelvisBodyRotation, rotation, dt),
                    _profile.MaxAngularVelocity);
            }
            else
            {
                _estimatedPelvisBodyLinearVelocity = _sampledPelvisLinearVelocity;
                _estimatedPelvisBodyAngularVelocity = _sampledPelvisAngularVelocity;
            }
            _previousPelvisBodyPosition = position;
            _previousPelvisBodyRotation = rotation;
            _hasPreviousPelvisBodySample = true;
        }

        private void ResetPelvisBodyVelocityEstimator()
        {
            _estimatedPelvisBodyLinearVelocity = _sampledPelvisLinearVelocity;
            _estimatedPelvisBodyAngularVelocity = _sampledPelvisAngularVelocity;
            if (_pelvisBody != null)
            {
                _previousPelvisBodyPosition = _pelvisBody.position;
                _previousPelvisBodyRotation = _pelvisBody.rotation;
                _hasPreviousPelvisBodySample = true;
            }
            else
            {
                _previousPelvisBodyPosition = Vector3.zero;
                _previousPelvisBodyRotation = Quaternion.identity;
                _hasPreviousPelvisBodySample = false;
            }
        }

        private void ResetSourceVelocityEstimator()
        {
            _sourceTurnDiagnostics.ResetAll();
            _sampledPelvisLinearVelocity = Vector3.zero;
            _sampledChestLinearVelocity = Vector3.zero;
            _sampledPelvisAngularVelocity = Vector3.zero;
            _sampledChestAngularVelocity = Vector3.zero;
            _sampledPelvisLinearAcceleration = Vector3.zero;
            _sampledChestLinearAcceleration = Vector3.zero;
            _sampledPelvisAngularAcceleration = Vector3.zero;
            _sampledChestAngularAcceleration = Vector3.zero;
            _previousSampledPelvisLinearVelocityForAcceleration = Vector3.zero;
            _previousSampledChestLinearVelocityForAcceleration = Vector3.zero;
            _previousSampledPelvisAngularVelocityForAcceleration = Vector3.zero;
            _previousSampledChestAngularVelocityForAcceleration = Vector3.zero;
            _hasPreviousSourceAccelerationSample = false;
            _parentTransportLinearVelocity = Vector3.zero;
            _physicsParentTransportLinearVelocity = Vector3.zero;
            _parentTransportConsistencyError = Vector3.zero;
            _parentTransportConsistencyErrorMagnitude = 0f;
            _parentFeedforwardReferenceVelocity = Vector3.zero;
            _sourceRelativeLinearVelocity = Vector3.zero;
            _targetRelativeAngularVelocity = Vector3.zero;
            _sampledTargetRelativeAngularVelocityLocal = Vector3.zero;
            _sourceVelocitySampleAgeSeconds = 0f;
            _sourceVelocitySampleFreshThisFixedStep = false;
            _sourceVelocitySampleSequence = 0;
            _lastFixedConsumedSourceVelocitySampleSequence = -1;
            _sourceVelocitySampleTime = 0f;
            _previousSampledTargetRelativeRotation = Quaternion.identity;
            _hasPreviousTargetRelativeSample = false;
            _bodyRelativeAngularVelocity = Vector3.zero;
            _relativeAngularVelocityError = Vector3.zero;
            _hasPreviousSourceSample = false;
        }

        private static Vector3 ComputeAngularVelocity(Quaternion previous, Quaternion current, float dt)
        {
            if (dt <= 0.000001f)
                return Vector3.zero;
            Quaternion delta = current * Quaternion.Inverse(previous);
            if (delta.w < 0f)
            {
                delta.x = -delta.x;
                delta.y = -delta.y;
                delta.z = -delta.z;
                delta.w = -delta.w;
            }
            float w = Mathf.Clamp(delta.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (sinHalf <= 0.0001f || angle <= 0.000001f)
                return Vector3.zero;
            Vector3 axis = new Vector3(delta.x / sinHalf, delta.y / sinHalf, delta.z / sinHalf);
            return axis * (angle / dt);
        }

        private bool HasFinitePhysicsState()
        {
            if (!IsBuilt)
                return false;
            return IsFinite(_pelvisBody.position) && IsFinite(_pelvisBody.rotation) &&
                   IsFinite(_chestBody.position) && IsFinite(_chestBody.rotation) &&
                   IsFinite(_chestBody.velocity) && IsFinite(_chestBody.angularVelocity) &&
                   (!DynamicPelvisReceiverSelected || (IsFinite(_pelvisBody.velocity) && IsFinite(_pelvisBody.angularVelocity)));
        }

        private void EnterBuildPhase(string phase)
        {
            _buildPhase = string.IsNullOrWhiteSpace(phase) ? "<unknown>" : phase;
            _report.BuildPhase = _buildPhase;
            if (_emitLogs) Plugin.Logger?.LogInfo("[ARL][TWO_BODY][BUILD_PHASE] phase=" + _buildPhase);
        }

        private static Vector3 AnalyticalUniformBoxInertia(float mass, Vector3 size)
        {
            float x = Mathf.Max(0.0001f, Mathf.Abs(size.x));
            float y = Mathf.Max(0.0001f, Mathf.Abs(size.y));
            float z = Mathf.Max(0.0001f, Mathf.Abs(size.z));
            float k = Mathf.Max(0.0001f, mass) / 12f;
            return new Vector3(k * (y * y + z * z), k * (x * x + z * z), k * (x * x + y * y));
        }

        private static bool ValidateBodyMassPropertyMode(Rigidbody body, BoxCollider collider, float mass, TwoBodyMassPropertyMode mode)
        {
            if (body == null || collider == null) return false;
            Vector3 expectedInertia = AnalyticalUniformBoxInertia(mass, collider.size);
            Vector3 expectedCom = (mode == TwoBodyMassPropertyMode.ExplicitCenterOfMassOnly || mode == TwoBodyMassPropertyMode.ExplicitUniformBox)
                ? collider.center
                : Vector3.zero;
            Vector3 expectedTensor = (mode == TwoBodyMassPropertyMode.ExplicitInertiaOnly || mode == TwoBodyMassPropertyMode.ExplicitUniformBox)
                ? expectedInertia
                : Vector3.one;
            return Approximately(body.centerOfMass, expectedCom) &&
                   Approximately(body.inertiaTensor, expectedTensor) &&
                   Approximately(body.inertiaTensorRotation, Quaternion.identity);
        }

        private static bool Approximately(Vector3 a, Vector3 b, float tolerance = 0.0005f)
        {
            return (a - b).sqrMagnitude <= tolerance * tolerance;
        }

        private static bool Approximately(Quaternion a, Quaternion b, float toleranceDegrees = 0.05f)
        {
            return Quaternion.Angle(a, b) <= toleranceDegrees;
        }

        private Vector3 ClampSize(Vector3 value)
        {
            return new Vector3(
                Mathf.Clamp(value.x, _profile.MinBoxDimension, _profile.MaxBoxDimension),
                Mathf.Clamp(value.y, _profile.MinBoxDimension, _profile.MaxBoxDimension),
                Mathf.Clamp(value.z, _profile.MinBoxDimension, _profile.MaxBoxDimension));
        }

        private static Material CreatePreviewMaterial(float alpha)
        {
            Shader shader = Shader.Find("Legacy Shaders/Transparent/Diffuse");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                throw new InvalidOperationException("没有找到可用的预览 Shader。" );

            Material material = new Material(shader);
            material.name = "ARL_TwoBodyPreviewMaterial";
            material.hideFlags = HideFlags.DontSave;
            material.color = new Color(0.15f, 0.75f, 1.0f, alpha);
            return material;
        }

        private static void SetRendererEnabled(GameObject visual, bool enabled)
        {
            if (visual == null)
                return;
            try
            {
                Renderer renderer = visual.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.enabled = enabled;
            }
            catch { }
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        }

        private static bool IsFinite(float value)
        {
            return !(float.IsNaN(value) || float.IsInfinity(value));
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
                return "Target";
            return value.Trim().Replace(' ', '_').Replace('/', '_').Replace('\\', '_');
        }

        private static string Format(Vector3 value)
        {
            return "(" + value.x.ToString("G6") + "," + value.y.ToString("G6") + "," + value.z.ToString("G6") + ")";
        }
    }
}
