using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Native construction, activation and cleanup. Per-side ownership remains in this instance.
    internal sealed partial class SingleLegShadowChain
    {

        public string Build()
        {
            if (_destroyed) return "ARL single-leg build | success=False | reason=OWNER_TERMINATED_RECREATE_REQUIRED";
            if (IsBuilt)
                return "ARL single-leg：已经创建。";
            if (!_target.Success || _target.UpperLeg == null || _target.LowerLeg == null || _target.Foot == null)
                return "ARL single-leg build 失败：目标骨解析无效。";
            if (_pelvisBody == null || _proxyRoot == null)
                return "ARL single-leg build 失败：冻结 torso Pelvis Rigidbody/Root 不可用。";

            try
            {
                _report.Stage = "CREATE_ROOT";
                _root = new GameObject("ARL_SingleLeg_" + _target.RequestedSide);
                _root.hideFlags = HideFlags.DontSave;
                _root.transform.SetParent(_proxyRoot, true);
                _root.transform.position = Vector3.zero;
                _root.transform.rotation = Quaternion.identity;

                CreatePreviewMaterial(_profile.PreviewAlpha);

                SampleSourcePoseInternal(false);

                _report.Stage = "CREATE_UPPER_LEG";
                CreateSegmentBody(
                    "ARL_UpperLeg_" + _target.RequestedSide,
                    _target.UpperLeg,
                    _target.LowerLeg.position,
                    _profile.UpperLegMass,
                    out _upperObject,
                    out _upperBody,
                    out _upperCollider,
                    out _upperVisual);

                _report.Stage = "CREATE_LOWER_LEG";
                CreateSegmentBody(
                    "ARL_LowerLeg_" + _target.RequestedSide,
                    _target.LowerLeg,
                    _target.Foot.position,
                    _profile.LowerLegMass,
                    out _lowerObject,
                    out _lowerBody,
                    out _lowerCollider,
                    out _lowerVisual);

                if (_profile.FootAnkleFoundationEnabled)
                {
                    _report.Stage = "CREATE_FOOT_ANKLE_FOUNDATION";
                    _footFoundation = new FootAnkleProxyFoundation(
                        _profile, _target, _root.transform, _targetRoot, _lowerBody, _upperObject.layer, _previewMaterial);
                    string footBuild = _footFoundation.Build();
                    if (!_footFoundation.IsBuilt)
                        throw new InvalidOperationException("FOOT_ANKLE_FOUNDATION_BUILD_FAILED:" + footBuild);
                    Plugin.Logger?.LogWarning("[ARL][FOOT_ANKLE_BUILD] side=" + _target.RequestedSide + " | " + footBuild);
                }

                _upperObject.transform.SetParent(_root.transform, true);
                _lowerObject.transform.SetParent(_root.transform, true);

                // v0.2.17 production foundation: the v0.2.12-v0.2.16 attribution matrix is complete.
                // Always establish analytical uniform-box COM/inertia before adding constraints; fail closed on readback mismatch.
                _report.Stage = "APPLY_EXPLICIT_BOX_MASS_PROPERTY_FOUNDATION";
                Vector3 buildKneeAxisWorld = _target.KneeFlexionAxisWorld;
                bool inertiaFoundationOk = _rigidbodyMassProperties.ApplyValidatedBuildFoundation(
                    _upperBody, _upperCollider, _lowerBody, _lowerCollider, buildKneeAxisWorld);
                CopyRigidbodyMassPropertiesToReport(true);
                if (!inertiaFoundationOk)
                {
                    throw new InvalidOperationException(
                        "LEG_EXPLICIT_BOX_MASS_PROPERTY_FOUNDATION_FAILED:" +
                        (_rigidbodyMassProperties.LastError ?? "UNKNOWN") +
                        " | " + _rigidbodyMassProperties.GetConsoleSummary());
                }
                Plugin.Logger?.LogWarning(
                    "[ARL][LEG_MASS_PROPERTIES] buildValidated=True | " +
                    _rigidbodyMassProperties.GetConsoleSummary());

                _rawHipConnectedAnchorLocal = _pelvisBody.transform.InverseTransformPoint(_target.UpperLeg.position);
                _report.Stage = "CREATE_HIP_MECHANICAL_PARENT";
                _hipMechanicalParent.Build(
                    _root.transform,
                    _upperObject.layer,
                    GetRawPelvisHipAnchorWorld(),
                    _pelvisBody.rotation,
                    _profile.MaxAngularVelocity);

                _report.Stage = "ADD_HIP_CHARACTER_JOINT";
                _hipJoint = Il2CppComponentFactory.Add<CharacterJoint>(_upperObject, "LEG_ADD_HIP_CHARACTER_JOINT");
                ConfigureHipJoint();

                _kneeChildAnchorLowerLocal = Vector3.zero;
                _kneeParentAnchorUpperLocal = _upperBody.transform.InverseTransformPoint(_target.LowerLeg.position);
                _kneeFlexionAxisLowerLocal = _initialKneeFlexionAxisLowerLocal;
                _report.Stage = "ADD_KNEE_WIDE_CHARACTER_JOINT";
                _kneeJoint = Il2CppComponentFactory.Add<CharacterJoint>(_lowerObject, "LEG_ADD_KNEE_WIDE_CHARACTER_JOINT");
                ConfigureKneeJoint();

                _report.Stage = "COLLISION_ISOLATION";
                ConfigureCollisionIsolation();

                _gravityEnabled = _profile.DefaultGravity;
                _collidersEnabled = _profile.DefaultColliders;
                _visualEnabled = true;
                ApplyGravityState();
                ApplyColliderState();
                ApplyVisualState();

                _report.Built = true;
                _report.Side = _target.RequestedSide;
                _report.ResolutionMode = _target.ResolutionMode;
                _report.UpperLegPath = _target.UpperLegPath;
                _report.LowerLegPath = _target.LowerLegPath;
                _report.FootPath = _target.FootPath;
                _report.KneeFlexionAxisWorld = _target.KneeFlexionAxisWorld;
                _report.KneeFlexionAxisLowerLocal = _kneeFlexionAxisLowerLocal;
                CopyKneeConstraintBackendToReport();
                _report.KneeAxisMode = _kneeAxisMode;
                _kneeAxisCalibrator.CopyToReport(_report);
                UpdateJointPresence();
                _report.Stage = "BUILD_COMPLETE";
                _report.Detail = "KINEMATIC_PREVIEW_READY";
                UpdateReport();

                return
                    "ARL single-leg build | success=True" +
                    " | side=" + _target.RequestedSide +
                    " | resolve=" + _target.ResolutionMode +
                    " | upper=" + _target.UpperLegPath +
                    " | lower=" + _target.LowerLegPath +
                    " | foot=" + _target.FootPath +
                    " | lengths=" + F(_target.UpperLegLength) + "/" + F(_target.LowerLegLength) +
                    " | initialBuildHip=UnityCharacterJoint+RelativePD/frozenProductionAfterStack=NO_HIP_CHARACTERJOINT+TRANSLATION_ONLY_TETHER+RelativePD/currentArm=" + _hipCharacterJointResponseIsolation.Arm + "/mechanicalParent=" + _hipMechanicalParent.Mode + "(default-B)/parentTransportFF=ON(default)/anchorLinearTransport=ON(default)/previewPreprocessing=ON_IF_JOINT_PRESENT" +
                    " | initialBuildKnee=UnityCharacterJoint(WIDE)+FlexionPD+SoftOffAxisPD/frozenProductionAfterStack=NO_KNEE_CHARACTERJOINT+TRANSLATION_ONLY_TETHER+FlexionPD+SoftOffAxisPD/CHILD_ONLY/currentArm=" + _kneeCharacterJointResponseIsolation.Arm + "/axis=" + _kneeAxisMode + "/local=" + V(_kneeFlexionAxisLowerLocal) + "/calibration=v0.2.2" +
                    " | legAngularAccelFF=" + _legAngularAccelerationFeedforward.Enabled + "/" + SingleLegAngularAccelerationFeedforward.ModeName + "/normal=PRODUCTION_ON" +
                    " | kneeTorque=CHILD_ONLY/LOWER_ONLY_FORCEMODE_ACCELERATION" +
                    " | inertiaFoundation=" + SingleLegRigidbodyMassProperties.FoundationMode +
                    "/buildValidated=" + _rigidbodyMassProperties.BuildFoundationValidated +
                    " | footFoundation=" + (_profile.FootAnkleFoundationEnabled ? (_footFoundation != null ? _footFoundation.GetStatus() : "missing") : "disabled") +
                    " | drive=False | gravity=" + _gravityEnabled + " | colliders=" + _collidersEnabled;
            }
            catch (Exception e)
            {
                _report.Stage = "BUILD_FAILED";
                _report.Detail = e.GetType().Name + "/" + e.Message;
                try { Plugin.Logger?.LogError("[ARL][SINGLE_LEG][BUILD_FAILED] side=" + (_report.Side ?? "<none>") + " | stage=" + _report.Stage + " | " + e); }
                catch (Exception) { }
                int cleanupFailures = RollbackInternal().FailureCount.Value;
                LastRollbackSucceeded = cleanupFailures == 0;
                if (cleanupFailures > 0)
                    Plugin.Logger?.LogWarning("[ARL][SINGLE_LEG][BUILD_ROLLBACK_INCOMPLETE] failures=" + cleanupFailures);
                return "ARL single-leg build 失败：" + e.GetType().Name + " / " + e.Message +
                       (cleanupFailures > 0 ? " | cleanupFailures=" + cleanupFailures : string.Empty);
            }
        }

        public string SetDrive(bool enabled)
        {
            if (!IsBuilt)
            {
                // OFF must still reach surviving native owners after a partial loss.
                // A foundation guard is an ON precondition, never a cleanup guard.
                if (!enabled)
                {
                    int failures = ForceDriveOffSafe();
                    return "ARL leg drive | enabled=False | foundationMissing=True | success=" + (failures == 0) +
                        " | safeStateFailures=" + failures;
                }
                return "ARL leg drive：单腿链尚未创建。";
            }
            if (_driveEnabled == enabled && (enabled || (NativeBodiesSafeOff && LastSafeStateFailures == 0)))
                return GetDriveSummary();

            if (enabled)
            {
                try
                {
                    if (_torso.DynamicPelvisReceiverSelected && !_dynamicJoinPending)
                        return "ARL leg drive | enabled=False | reason=DYNAMIC_PARENT_REQUIRES_SESSION_JOIN";
                    if (!_rigidbodyMassProperties.BuildFoundationValidated)
                        return "ARL leg drive：拒绝启动。Explicit Box mass-property foundation 未通过 readback 验证；请 rollback/rebuild 并查看 [LEG_MASS_PROPERTIES]。";
                    if (_kneeAxisCalibrator.Collecting)
                        return "ARL leg drive：拒绝启动。请先执行 arl_knee_axis_calibrate(off)，完成或停止 Knee axis 标定。";
                    if (!_legAngularAccelerationFeedforward.Enabled || Mathf.Abs(_profile.LegAngularAccelerationFeedforwardMaxAcceleration - 20f) > 0.01f)
                        return "ARL leg drive：拒绝启动。v0.2.17 frozen real-inertia baseline 固定 Angular Accel FF=ON 且 bound=20。";
                    if (!_kneeAxisCalibrator.Ready || _kneeAxisMode != "DATA_DRIVEN_CALIBRATED")
                        return "ARL leg drive：拒绝启动。production Single-Leg 要求 v0.2.2 Data-Driven Knee axis 已 apply。";
                    if (_gravityEnabled || _collidersEnabled)
                        return "ARL leg drive：拒绝启动。v0.2.17 frozen real-inertia baseline 固定 Gravity=OFF、Colliders=OFF。";
                    if (_profile.FootAnkleFoundationEnabled && !FootRuntimeContractReady)
                        return "ARL leg drive：拒绝启动。v0.3.78 Foot production contract/readback 未满足；common-mode drag compensation 必须处于冻结production状态。";
                    if (_dynamicJoinPending)
                    {
                        // Use the held source packet validated against the parent step; never resample animation here.
                        _upperBody.position = _initialState.UpperPosition; _upperBody.rotation = _initialState.UpperRotation;
                        _lowerBody.position = _initialState.LowerPosition; _lowerBody.rotation = _initialState.LowerRotation;
                    }
                    else
                    {
                        SampleSourcePoseInternal(false);
                        FollowSourceKinematicPreview();
                        _upperBody.velocity = Vector3.zero;
                        _upperBody.angularVelocity = Vector3.zero;
                        _lowerBody.velocity = Vector3.zero;
                        _lowerBody.angularVelocity = Vector3.zero;
                    }
                    _dynamicParentActive = _dynamicJoinPending;
                    ResetBodyRelativeAngularVelocityEstimator();
                    ResetMeasuredTransportKinematics();
                    if (_dynamicParentActive) _measuredPelvisAngularVelocity = _physicalParent.W;
                    _sampledTargetHipRelativeAngularAccelerationLocal = Vector3.zero;
                    _sampledTargetKneeRelativeAngularAccelerationLocal = Vector3.zero;
                    _previousFilteredTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
                    _previousFilteredTargetKneeRelativeAngularVelocityLocal = _sampledTargetKneeRelativeAngularVelocityLocal;
                    _hasPreviousFilteredTargetVelocitySample = true;
                    _hipContinuationPending = false;
                    _fixedUpdateSequence = 0;
                    _targetSampleTiming.ResetForDrive(_targetRenderSampleSequence);
                    _sourceControlTargetHipRelative = _sampledTargetHipRelative;
                    _sourceControlTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
                    _controlTargetHipRelative = _sampledTargetHipRelative;
                    _controlTargetKneeRelative = _sampledTargetKneeRelative;
                    _controlTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
                    _controlTargetKneeRelativeAngularVelocityLocal = _sampledTargetKneeRelativeAngularVelocityLocal;
                    _hipSourceReferenceShockAbsorber.Reset(_sampledTargetHipRelative, _sampledTargetHipRelativeAngularVelocityLocal, _report);
                    _hipReferenceGovernor.Reset(_sampledTargetHipRelative, _sampledTargetHipRelativeAngularVelocityLocal, _report);
                    _hipReferenceGovernor.ObserveRenderPacket(_sampledTargetHipRelative, _targetRenderSampleSequence, _targetRenderSampleUnscaledTime);
                    _legAngularAccelerationFeedforward.Reset();
                    _softKneeAuthority.Reset(_report);
                    _hipAuthority.Reset(_profile, _report);
                    _hipControlFrame.Reset(_pelvisBody.rotation, _measuredPelvisAngularVelocity, _profile, _report);
                    _hipCompositeReferenceShockAbsorber.Reset(
                        _pelvisBody.rotation * _sampledTargetHipRelative,
                        _measuredPelvisAngularVelocity + _pelvisBody.rotation * _sampledTargetHipRelativeAngularVelocityLocal,
                        _report);
                    _hipNormalReference.Reset(_dynamicParentActive && _torso.ControlPolicy.UseCoherentHipNormalReference,
                        _pelvisBody.rotation * _controlTargetHipRelative,
                        _measuredPelvisAngularVelocity + _pelvisBody.rotation * _controlTargetHipRelativeAngularVelocityLocal,
                        _torso.ControlPolicy.HipNormalSampleAgePrediction);
                    _kneePlannedFeedforwardSelected = KneePlannedFeedforwardArmSelected;
                    _kneeReference.Reset(KneeReferenceArmSelected, _controlTargetKneeRelative,
                        _controlTargetKneeRelativeAngularVelocityLocal);
                    _hipCharacterJointResponseIsolation.ResetStats(_report);
                    _kneeParentAngularTransport.ResetDiagnostics(_report);
                    _lastHipAppliedAngularAccelerationWorld = Vector3.zero;
                    _hipMechanicalParent.ResetPose(GetRawPelvisHipAnchorWorld(), _pelvisBody.rotation);
                    ApplyHipMechanicalParentConnection();
                    _upperBody.isKinematic = false;
                    _lowerBody.isKinematic = false;
                    ApplyGravityState();
                    if (_dynamicParentActive)
                    {
                        _upperBody.velocity = _initialState.UpperVelocity; _upperBody.angularVelocity = _initialState.UpperW;
                        _lowerBody.velocity = _initialState.LowerVelocity; _lowerBody.angularVelocity = _initialState.LowerW;
                    }
                    if (_footFoundation != null && !_footFoundation.SetDriveState(true, _dynamicParentActive))
                    {
                        ForceDriveOffSafe();
                        return "ARL leg drive：拒绝启动。v0.3.78 Foot production contract/readback 失败；已 fail-closed。";
                    }
                    _driveEnabled = true;
                    _report.DriveAborted = false;
                    _report.DriveAbortReason = null;
                    // Fresh drive clears physical state. Starting/restarting a trace
                    // below resets statistics without changing capture or derivative history.
                    _hipMomentumRecoveryBraking.ResetForDrive(_report);
                    _hipVectorCaptureDamping.ResetForDrive(_report);
                    _hipAnchorAccelerationFeedforward.ResetForDrive(_report);
                    _kneeAnchorAccelerationFeedforward.ResetForDrive(_report);
                    ResetTraceStats();
                }
                catch
                {
                    // Console callers only report exceptions. Roll back partial native
                    // activation here even when the logical drive latch was not set yet.
                    ForceDriveOffSafe();
                    throw;
                }
            }
            else
            {
                int safeFailures = ForceDriveOffSafe();
                if (safeFailures > 0)
                {
                    UpdateReport();
                    return "ARL leg drive | enabled=False | success=False | safeStateFailures=" + safeFailures + " | action=FAIL_CLOSED_KEEP_DRIVE_OFF";
                }
                SampleSourcePoseInternal(false);
                FollowSourceKinematicPreview();
            }

            UpdateReport();
            return GetDriveSummary();
        }

        public string SetGravity(bool enabled)
        {
            if (!IsBuilt) return "ARL leg gravity：单腿链尚未创建。";
            if (enabled && _driveEnabled) return "ARL leg gravity：拒绝开启。frozen leg drive 固定 Gravity=OFF；请先停止 drive。";
            _gravityEnabled = enabled;
            ApplyGravityState();
            UpdateReport();
            return "ARL leg gravity | enabled=" + enabled + " | drive=" + _driveEnabled +
                   " | footGravity=False(frozen-v0.3.78)";
        }

        public string SetColliders(bool enabled)
        {
            if (!IsBuilt) return "ARL leg colliders：单腿链尚未创建。";
            if (enabled && _driveEnabled) return "ARL leg colliders：拒绝开启。frozen leg drive 固定 Colliders=OFF；请先停止 drive。";
            _collidersEnabled = enabled;
            ApplyColliderState();
            UpdateReport();
            return "ARL leg colliders | enabled=" + enabled + " | isolatedPairs=" + _ignoredPairs.Count +
                   " | footCollider=False(frozen-v0.3.78)";
        }

        public string SetVisual(bool enabled)
        {
            _visualEnabled = enabled;
            ApplyVisualState();
            _footFoundation?.SetVisual(enabled);
            UpdateReport();
            return "ARL leg visual | enabled=" + enabled;
        }

        public string ResetPose()
        {
            if (!IsBuilt)
                return "ARL leg reset：单腿链尚未创建。";
            bool wasDrive = _driveEnabled;
            int safeFailures = ForceDriveOffSafe();
            if (safeFailures > 0)
            {
                UpdateReport();
                return "ARL leg reset | success=False | safeStateFailures=" + safeFailures + " | restoredDrive=False | action=FAIL_CLOSED_KEEP_DRIVE_OFF";
            }
            SampleSourcePoseInternal(false);
            FollowSourceKinematicPreview();
            if (wasDrive)
            {
                _upperBody.isKinematic = false;
                _lowerBody.isKinematic = false;
                _driveEnabled = true;
                ApplyGravityState();
            }
            UpdateReport();
            return "ARL leg reset | success=True | restoredDrive=" + wasDrive;
        }

        public string Rollback(string reason)
            => RollbackWithReceipt(reason).Summary;

        internal CleanupReceipt RollbackWithReceipt(string reason)
        {
            // Terminal for control immediately; failed resource owners remain available
            // to subsequent cleanup attempts until their actual release succeeds.
            _destroyed = true;
            _traceEnabled = false;
            LastRollbackSucceeded = false;
            int failures = 0;
            try { failures += ForceDriveOffSafe(); }
            catch (Exception e)
            {
                failures++;
                LogCleanupFailure("ROLLBACK_SAFE_STATE", e);
            }
            CleanupReceipt result = RollbackInternal();
            failures += result.FailureCount.Value;
            result = result.WithFailureCount(failures);
            LastRollbackSucceeded = result.Succeeded;
            _destroyed = true;
            _report.Built = false;
            _report.Stage = LastRollbackSucceeded ? "ROLLED_BACK" : "ROLLBACK_WITH_ERRORS";
            _report.Detail = reason;
            return result.WithSummary("ARL single-leg rollback | success=" + LastRollbackSucceeded + " | reason=" + reason + " | cleanupFailures=" + failures);
        }

        private void CreateSegmentBody(
            string name,
            Transform sourceBone,
            Vector3 childWorldPosition,
            float mass,
            out GameObject gameObject,
            out Rigidbody body,
            out BoxCollider collider,
            out GameObject visual)
        {
            gameObject = new GameObject(name);
            _unparentedObjects.Add(gameObject);
            gameObject.hideFlags = HideFlags.DontSave;
            gameObject.layer = _proxyRoot.gameObject.layer;
            // Attach immediately so any later component/material/readback failure remains owned by _root and
            // is removed by rollback instead of leaving a transient scene-root leg body behind.
            if (_root != null)
            {
                gameObject.transform.SetParent(_root.transform, true);
                _unparentedObjects.Remove(gameObject);
            }
            gameObject.transform.position = sourceBone.position;
            gameObject.transform.rotation = sourceBone.rotation;

            Vector3 localDelta = sourceBone.InverseTransformDirection(childWorldPosition - sourceBone.position);
            float thickness = _profile.LegSegmentThickness;
            Vector3 size = new Vector3(
                Mathf.Max(Mathf.Abs(localDelta.x), thickness),
                Mathf.Max(Mathf.Abs(localDelta.y), thickness),
                Mathf.Max(Mathf.Abs(localDelta.z), thickness));
            Vector3 center = localDelta * 0.5f;

            collider = Il2CppComponentFactory.Add<BoxCollider>(gameObject, "LEG_ADD_BOX_COLLIDER");
            collider.center = center;
            collider.size = size;
            collider.enabled = false;

            body = Il2CppComponentFactory.Add<Rigidbody>(gameObject, "LEG_ADD_RIGIDBODY");
            body.mass = mass;
            body.isKinematic = true;
            body.useGravity = false;
            body.drag = 0.25f;
            body.angularDrag = _profile.AngularDrag;
            body.maxAngularVelocity = _profile.MaxAngularVelocity;
            body.interpolation = RigidbodyInterpolation.None;
            body.collisionDetectionMode = CollisionDetectionMode.Discrete;

            visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
            _unparentedObjects.Add(visual);
            visual.name = name + "_Visual";
            visual.hideFlags = HideFlags.DontSave;
            visual.layer = gameObject.layer;
            visual.transform.SetParent(gameObject.transform, false);
            // A segment without _root remains independently owned until rollback.
            if (!_unparentedObjects.Contains(gameObject)) _unparentedObjects.Remove(visual);
            visual.transform.localPosition = center;
            visual.transform.localRotation = Quaternion.identity;
            visual.transform.localScale = size;
            try
            {
                Collider primitiveCollider = visual.GetComponent<Collider>();
                if (primitiveCollider == null)
                    throw new InvalidOperationException("LEG_PREVIEW_COLLIDER_MISSING");
                // CreatePrimitive(Cube) includes a Collider. It must be disabled synchronously before the
                // deferred Destroy call; otherwise a preview-only object could become an unintended physics body.
                primitiveCollider.enabled = false;
                Object.Destroy(primitiveCollider);
            }
            catch (Exception e)
            {
                throw new InvalidOperationException("LEG_PREVIEW_COLLIDER_DISABLE_FAILED", e);
            }
            try
            {
                Renderer renderer = visual.GetComponent<Renderer>();
                if (renderer != null)
                    renderer.sharedMaterial = _previewMaterial;
            }
            catch { }
        }

        private void ConfigureCollisionIsolation()
        {
            IgnorePair(_upperCollider, _lowerCollider);
            for (int i = 0; i < _torsoOwnedColliders.Length; i++)
            {
                IgnorePair(_upperCollider, _torsoOwnedColliders[i]);
                IgnorePair(_lowerCollider, _torsoOwnedColliders[i]);
            }

            Collider[] sourceColliders = null;
            try
            {
                sourceColliders = _targetRoot != null ? _targetRoot.gameObject.GetComponentsInChildren<Collider>(true) : null;
            }
            catch (Exception e)
            {
                throw new InvalidOperationException("LEG_SOURCE_COLLIDER_ENUMERATION_FAILED", e);
            }
            if (_targetRoot != null && sourceColliders == null)
                throw new InvalidOperationException("LEG_SOURCE_COLLIDER_ENUMERATION_RETURNED_NULL");
            if (sourceColliders == null)
                return;
            for (int i = 0; i < sourceColliders.Length; i++)
            {
                Collider source = sourceColliders[i];
                if (source == null || source == _upperCollider || source == _lowerCollider)
                    continue;
                IgnorePair(_upperCollider, source);
                IgnorePair(_lowerCollider, source);
            }
        }

        private void IgnorePair(Collider a, Collider b)
        {
            if (a == null || b == null || a == b)
                return;
            try
            {
                // Preserve pre-existing isolation and retain ownership before a native
                // write that can fail after partially changing the pair.
                if (UnityEngine.Physics.GetIgnoreCollision(a, b)) return;
                _ignoredPairs.Remember(a, b);
                UnityEngine.Physics.IgnoreCollision(a, b, true);
            }
            catch (Exception e)
            {
                throw new InvalidOperationException("LEG_COLLISION_ISOLATION_FAILED", e);
            }
        }

        private int RestoreIgnoredPairs()
        {
            int failures = _ignoredPairs.RestoreAll();
            if (failures > 0)
                LogCleanupFailure("RESTORE_IGNORE_PAIRS", new InvalidOperationException("failures=" + failures));
            return failures;
        }
        private void AbortDrive(string reason)
        {
            // Stop notice is recorded before diagnostic formatting. Detailed Foot windows are already
            // retained by existing bounded observers and included in the completed report after stop.
            _report.DriveAborted = true;
            _report.DriveAbortReason = reason;
            ActiveRagdollLabRuntime.RecordAutomaticStop("LEG/" + (_report.Side ?? "UNKNOWN") + "/fixed=" + _fixedUpdateSequence, reason);
            int safeFailures = ForceDriveOffSafe();
            Plugin.Logger?.LogError("[ARL][LEG_DRIVE_ABORT] side=" + (_report.Side ?? "<none>") +
                " | fixed=" + _fixedUpdateSequence + " | reason=" + reason + " | driveOff=True");

            _hipCharacterJointResponseIsolation.CaptureAbort(_report, reason);
            _hipAnchorAccelerationFeedforward.CaptureAbort(_report, reason);
            _kneeCharacterJointResponseIsolation.CaptureAbort(_report, reason);
            _kneeAnchorAccelerationFeedforward.CaptureAbort(_report, reason);
            _report.DriveAborted = true;
            _report.DriveAbortReason = reason;
            _hipMomentumRecoveryBraking.MarkAbort(reason, _report);
            _hipVectorCaptureDamping.MarkAbort(reason, _report);
            _hipSourceReferenceShockAbsorber.MarkAbort(reason, _report);
            _hipCompositeReferenceShockAbsorber.MarkAbort(reason, _report);
            if (_traceEnabled)
                Plugin.Logger?.LogWarning("[ARL][KNEE_TETHER_SAFETY_ABORT_EVIDENCE] " + _kneeCharacterJointResponseIsolation.GetKey(_report));
            if (safeFailures > 0)
            {
                _report.DriveAbortReason = reason + "|SAFE_STATE_FAILURES=" + safeFailures;
                Plugin.Logger?.LogError("[ARL][LEG_ABORT_SAFE_STATE_FAILED] side=" + (_report.Side ?? "<none>") + " | failures=" + safeFailures + " | drive remains logically OFF; bilateral/runtime cleanup must retry safe-state/rollback.");
            }
            UpdateReport();
        }

        private int ForceDriveOffSafe()
        {
            _hipContinuationPending = false;
            int failures = 0;
            _driveEnabled = false; _dynamicParentActive = false;
            _report.DriveEnabled = false;
            failures += TryCleanupStep("HIP_PAIR", () =>
            {
                if (!_hipAnchorPair.Quiesce(_insidePhysicsStep ? _runtimeStep : new PhysicsStepContext()))
                    throw new InvalidOperationException("HIP_PAIR_QUIESCE_FAILED");
            });
            _hipNormalReference.DisablePreserveTrace();
            _kneeReference.DisablePreserveTrace();
            _kneePlannedFeedforwardSelected = false;
            _collidersEnabled = false;
            _report.CollidersEnabled = false;
            failures += TryCleanupStep("UPPER_COLLIDER", () => { if (_upperCollider != null) _upperCollider.enabled = false; });
            failures += TryCleanupStep("LOWER_COLLIDER", () => { if (_lowerCollider != null) _lowerCollider.enabled = false; });
            failures += TryCleanupStep("UPPER_SELF_WEIGHT", () =>
            {
                if (!_upperSelfWeightTransfer.Quiesce(_upperBody))
                    throw new InvalidOperationException("UPPER_SELF_WEIGHT_QUIESCE_FAILED");
            });
            _kneeCharacterJointResponseIsolation.CancelDeferredForce();
            _kneeSupportReaction.Quiesce();
            _report.AngularDynamicsValid = false;
            // Clear only the current A-FF output when drive stops/aborts, but preserve trace counters for the just-finished run.
            _legAngularAccelerationFeedforward.SetEnabled(_legAngularAccelerationFeedforward.Enabled);
            _controlTargetHipRelative = _sampledTargetHipRelative;
            _controlTargetKneeRelative = _sampledTargetKneeRelative;
            _controlTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
            _controlTargetKneeRelativeAngularVelocityLocal = _sampledTargetKneeRelativeAngularVelocityLocal;
            failures += QuiesceNativeBody(_upperBody, "UPPER");
            failures += QuiesceNativeBody(_lowerBody, "LOWER");
            failures += TryCleanupStep("LOWER_NATIVE_GRAVITY", () =>
            {
                if (!_stanceLowerNativeGravity.QuiescePreserveTrace(_lowerBody, _report, out string error))
                    throw new InvalidOperationException(error ?? "LOWER_GRAVITY_QUIESCE_FAILED");
            });
            int footFailures = 0;
            failures += TryCleanupStep("FOOT_FOUNDATION", () =>
            {
                if (_footFoundation != null) footFailures = _footFoundation.ForceSafeState();
            });
            failures += footFailures;
            failures += TryCleanupStep("COLLIDER_READBACK", () =>
            {
                if ((_upperCollider != null && _upperCollider.enabled) || (_lowerCollider != null && _lowerCollider.enabled))
                    throw new InvalidOperationException("LEG_COLLIDER_SAFE_STATE_READBACK_FAILED");
            });
            LastSafeStateFailures = failures;
            return failures;
        }

        // Cold lifecycle path only. Each native write is independent so a failed
        // gravity/velocity setter cannot prevent the final kinematic shutdown.
        private int QuiesceNativeBody(Rigidbody body, string name)
        {
            if (body == null) return 0;
            int failures = TryCleanupStep(name + "_GRAVITY", () => body.useGravity = false);
            failures += TryCleanupStep(name + "_VELOCITY", () => body.velocity = Vector3.zero);
            failures += TryCleanupStep(name + "_ANGULAR_VELOCITY", () => body.angularVelocity = Vector3.zero);
            failures += TryCleanupStep(name + "_KINEMATIC", () => body.isKinematic = true);
            failures += TryCleanupStep(name + "_READBACK", () =>
            {
                if (body.useGravity || !body.isKinematic)
                    throw new InvalidOperationException("LEG_BODY_SAFE_STATE_READBACK_FAILED");
            });
            return failures;
        }

        private int TryCleanupStep(string step, Action action)
        {
            try { action(); return 0; }
            catch (Exception e) { LogCleanupFailure(step, e); return 1; }
        }

        private void LogCleanupFailure(string step, Exception error)
        {
            try { Plugin.Logger?.LogWarning("[ARL][LEG_CLEANUP_FAILED] side=" + (_report.Side ?? "<none>") +
                " | step=" + step + " | " + error.GetType().Name + " / " + error.Message); }
            catch (Exception) { } // Logging cannot interrupt another native owner's cleanup.
        }

        private void ApplyGravityState()
        {
            if (_upperBody != null) _upperBody.useGravity = _driveEnabled && _gravityEnabled;
            if (_lowerBody != null) _lowerBody.useGravity = _driveEnabled && _gravityEnabled;
        }

        private void ApplyColliderState()
        {
            if (_upperCollider != null) _upperCollider.enabled = _collidersEnabled;
            if (_lowerCollider != null) _lowerCollider.enabled = _collidersEnabled;
        }

        private void ApplyVisualState()
        {
            SetRendererEnabled(_upperVisual, _visualEnabled);
            SetRendererEnabled(_lowerVisual, _visualEnabled);
        }

        private CleanupReceipt RollbackInternal()
        {
            _destroyed = true;
            _traceEnabled = false;
            LastRollbackSucceeded = false;
            int failures = TryCleanupStep("STOP_HIP_RESPONSE", () => _hipAnchorPair.Response.Stop());
            int pairFailures = 0;
            failures += TryCleanupStep("RESTORE_IGNORE_PAIRS", () => pairFailures = RestoreIgnoredPairs());
            failures += pairFailures;
            failures += TryCleanupStep("DESTROY_MECHANICAL_PARENT", () => _hipMechanicalParent.Destroy());
            if (_footFoundation != null)
            {
                int footFailures = 0;
                CleanupReceipt footResult = default;
                int footException = TryCleanupStep("DESTROY_FOOT_FOUNDATION", () =>
                {
                    footResult = _footFoundation.RollbackWithReceipt();
                    footFailures = footResult.FailureCount.Value;
                });
                failures += footException + footFailures;
                if (footException == 0 && footResult.Succeeded) _footFoundation = null;
            }
            int destroyAttempts = 0, destroyReturned = 0;
            for (int i = _unparentedObjects.Count - 1; i >= 0; i--)
            {
                GameObject orphan = _unparentedObjects[i];
                int orphanFailures = TryCleanupStep("DESTROY_UNPARENTED_OBJECT", () =>
                {
                    if (orphan != null)
                    {
                        destroyAttempts++;
                        Object.Destroy(orphan);
                        destroyReturned++;
                    }
                });
                failures += orphanFailures;
                if (orphanFailures == 0) _unparentedObjects.RemoveAt(i);
            }
            int rootFailures = TryCleanupStep("DESTROY_ROOT", () =>
            {
                if (_root != null)
                {
                    destroyAttempts++;
                    Object.Destroy(_root);
                    destroyReturned++;
                }
            });
            failures += rootFailures;
            if (rootFailures == 0)
            {
                _root = null;
                _upperObject = _lowerObject = null;
                _upperBody = _lowerBody = null;
                _upperCollider = _lowerCollider = null;
                _hipJoint = _kneeJoint = null;
                _upperVisual = _lowerVisual = null;
            }
            int materialFailures = TryCleanupStep("DESTROY_PREVIEW_MATERIAL", () =>
            {
                if (_previewMaterial != null)
                {
                    destroyAttempts++;
                    Object.Destroy(_previewMaterial);
                    destroyReturned++;
                }
            });
            failures += materialFailures;
            if (materialFailures == 0) _previewMaterial = null;
            _hipAnchorAccelerationFeedforward.SetEnabled(false, _report);
            _hipCharacterJointResponseIsolation.SetEnabled(false, _report);
            _kneeCharacterJointResponseIsolation.SetEnabled(false, _report);
            _report.Built = false;
            LastRollbackSucceeded = failures == 0;
            int retained = _unparentedObjects.Count + (ReferenceEquals(_root, null) ? 0 : 1) +
                (ReferenceEquals(_previewMaterial, null) ? 0 : 1) + (_footFoundation == null ? 0 : 1);
            return new CleanupReceipt(true, failures == 0, failures, retained, _ignoredPairs.Count,
                destroyAttempts, destroyReturned, null);
        }

        private void CreatePreviewMaterial(float alpha)
        {
            Shader shader = Shader.Find("Legacy Shaders/Transparent/Diffuse");
            if (shader == null) shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("Standard");
            if (shader == null) throw new InvalidOperationException("没有找到可用的腿部预览 Shader。");
            _previewMaterial = new Material(shader);
            _previewMaterial.name = "ARL_SingleLegPreviewMaterial";
            _previewMaterial.hideFlags = HideFlags.DontSave;
            _previewMaterial.color = new Color(1.0f, 0.55f, 0.15f, alpha);
        }

        private static void SetRendererEnabled(GameObject visual, bool enabled)
        {
            if (visual == null) return;
            try
            {
                Renderer renderer = visual.GetComponent<Renderer>();
                if (renderer != null) renderer.enabled = enabled;
            }
            catch { }
        }
    }
}
