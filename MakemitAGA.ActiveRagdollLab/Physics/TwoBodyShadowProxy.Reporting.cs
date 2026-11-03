using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Report projection, retained reaction evidence, and trace aggregates.
    // This is the same owner: the core keeps every capture/copy call in its original
    // phase. Control observations are published at these same sampling points;
    // this work must not be skipped merely because trace is OFF.
    internal sealed partial class TwoBodyShadowProxy
    {
        private void CaptureSolvedCharacterJointReaction()
        {
            _jointCurrentForceWorld = Vector3.zero; _jointReactionReadbackValid = false;
            _jointCurrentTorqueWorld = Vector3.zero;
            _jointPelvisComLeverWorld = Vector3.zero;

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

                // Preserve the old derived-response eligibility check after retiring
                // its unused taps. Invalid inertia is unavailable evidence, not zero A.
                if (!TryTorqueToAngularAccelerationProxy(_chestBody, torque, out _) ||
                    !TryTorqueToAngularAccelerationProxy(_chestBody, forceMoment, out _) ||
                    !TryTorqueToAngularAccelerationProxy(_chestBody, combinedMoment, out _))
                    return;

                _jointCurrentForceWorld = force;
                _jointCurrentTorqueWorld = torque;
                _jointPelvisComLeverWorld = pelvisLever;
                _jointReactionReadbackValid = true;
            }
            catch
            {
                // Diagnostic capability must never gate the frozen production torso.
            }
        }

        private static bool TryTorqueToAngularAccelerationProxy(Rigidbody body, Vector3 torqueWorld, out Vector3 acceleration)
        {
            acceleration = Vector3.zero;
            if (body == null || !IsFinite(torqueWorld)) return false;
            Vector3 tensor = body.inertiaTensor;
            if (!IsFinite(tensor) || tensor.x <= 1e-7f || tensor.y <= 1e-7f || tensor.z <= 1e-7f) return false;
            Quaternion inertiaWorld = body.rotation * body.inertiaTensorRotation;
            Vector3 localTorque = Quaternion.Inverse(inertiaWorld) * torqueWorld;
            Vector3 localA = new Vector3(localTorque.x / tensor.x, localTorque.y / tensor.y, localTorque.z / tensor.z);
            acceleration = inertiaWorld * localA;
            return IsFinite(acceleration);
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

        private void UpdateReport(string detail)
        {
            _report.UseDynamicTorsoRecovery = _controlState.Policy.UseDynamicTorsoRecovery;
            _report.HipGoalGateOnlyHip = _controlState.Policy.HipGoalGateOnlyHip;
            _report.Built = _controlState.ObserveBuilt(IsBuilt);
            _report.DriveEnabled = _driveEnabled;
            _report.GravityEnabled = _gravityEnabled;
            _report.CollidersEnabled = _collidersEnabled;
            _report.VisualEnabled = _visualEnabled;
            _report.SourceAlive = _controlState.ObserveSourceAlive(_audit?.Pelvis != null && _audit.Chest != null);
            _report.FiniteState = _controlState.ObserveFiniteState(HasFinitePhysicsState());
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
                _report.RotationErrorDegrees = _controlState.ObserveRotationError(Quaternion.Angle(currentRelative, targetRelative));
                _report.PelvisPositionErrorMeters = Vector3.Distance(_pelvisBody.position, _sampledPelvisPosition);
                _report.ChestPositionErrorMeters = _controlState.ObserveChestPositionError(Vector3.Distance(_chestBody.position, _sampledChestPosition));
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
                _report.TargetRelativeAngularVelocity = _controlState.ObserveTargetRelativeAngularVelocity(_targetRelativeAngularVelocity);
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
            _report.MeasuredAnchorRelativeVelocity = _controlState.ObserveAnchorRelativeVelocity(_measuredAnchorRelativeVelocity);
            _report.MeasuredTransportReferenceVelocity = _measuredTransportReferenceVelocity;
            _report.MeasuredChestTransportVelocityError = _measuredChestTransportVelocityError;
            _report.MotionPhase = _controlState.PublishMotionPhase(_motionPhase);
            _report.MotionTransition = _controlState.PublishMotionTransition(_motionTransition);
            _report.MotionTargetSpeed = _controlState.PublishMotionTargetSpeed(_motionTargetSpeed);
            _report.MotionSignedSpeedAcceleration = _motionSignedSpeedAcceleration;
            CopySourceTurnDiagnosticsToReport();

            _report.DriveBackend = DriveBackend;
            _report.ConstraintBackend = ConstraintBackend;
            _report.CharacterJointPresent = _controlState.ObserveJointPresent(_characterJoint != null);
            _report.CharacterJointConnectedBodyMatch =
                _controlState.ObserveJointConnection(_characterJoint != null && _characterJoint.connectedBody == _pelvisBody);
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

            _report.AngularAccelerationClamped = _controlState.ObserveAngularClamp(_lastAngularAccelerationClamped);
            _report.AngularAccelerationClampCount = _angularAccelerationClampCount;
            _report.RawAngularAccelerationMagnitude = _rawAngularAccelerationMagnitude;
            _report.ParentMotionFeedforwardEnabled = _parentMotionFeedforwardEnabled && !DynamicPelvisReceiverSelected;
            _report.CharacterJointAnchorMode = CharacterJointAnchorMode;
            _report.CharacterJointAnchorModeValidated = _controlState.ObserveJointAnchorValidation(_characterJointAnchorModeValidated);
            if (_pelvisObject != null && _chestObject != null && _chestBody != null)
            {
                Vector3 constraintChestWorld = ChestPoint(_characterJointChestAnchorLocal);
                Vector3 constraintPelvisWorld = PelvisPoint(_characterJointPelvisAnchorLocal);
                _report.CharacterJointConstraintAnchorErrorMeters = _controlState.ObserveConstraintAnchorError(Vector3.Distance(constraintChestWorld, constraintPelvisWorld));
                _report.CharacterJointChestComLeverMeters = Vector3.Distance(constraintChestWorld, _chestBody.worldCenterOfMass);
            }
            _report.ParentFeedforwardAccelerationClamped = _controlState.ObserveParentFeedforwardClamp(_parentFeedforwardAccelerationClamped);
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
                _report.AnchorErrorMeters = _controlState.ObserveVirtualAnchorError(Vector3.Distance(a, b));
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
            _freezeValidation.CopyToReport(_report, _controlState);
            _report.TraceEnabled = _traceEnabled;
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
            _report.SourceTurnEventActive = _controlState.PublishSourceTurn(_sourceTurnDiagnostics.EventActive);
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

        private float TraceLogInterval => RunLogPolicy.PeriodicInterval(_profile.TraceIntervalSeconds, _pelvisReceiver.Report.TraceHipTranslation);

        private void EmitTraceIfDue()
        {
            if (!_traceEnabled || !IsBuilt || !_driveEnabled)
                return;
            float now = CurrentUnscaledTime;
            if (now < _nextTraceTime)
                return;
            _nextTraceTime = now + TraceLogInterval;
            Plugin.Logger?.LogInfo("[ARL][TRACE] " + (_profile.TraceVerboseSnapshots ? _report.GetTraceSummary() : _report.GetTraceSummaryCompact()));
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
            _jointCurrentForceWorld = Vector3.zero; _jointReactionReadbackValid = false;
            _jointCurrentTorqueWorld = Vector3.zero;
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

    }
}
