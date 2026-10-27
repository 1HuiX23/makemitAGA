using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // The legacy leg/foot stack's native selectors and readbacks are kept together.
    // Partial only splits the host adapter; it introduces no second object owner or controller.
    internal static partial class ActiveRagdollLabRuntime
    {
        private sealed class AnkleRunPort : IAnkleRunPort
        {
            public string Scene => _sceneName + "#" + _sceneHandle;
            public bool CleanupPending => _cleanupPending;
            private bool SharedTraceReady => _proxy != null && _proxy.Report.TraceEnabled && _legChain != null && _legChain.TraceFullyEnabled;
            public string ResolveArm(string requested)
            {
                if (string.Equals(requested, "a", StringComparison.OrdinalIgnoreCase) || requested == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmA)
                    return UpperSelfWeightSupportNormalLoadTransferIsolation.ArmA;
                if (string.Equals(requested, "b", StringComparison.OrdinalIgnoreCase) || requested == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB)
                    return UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB;
                return null;
            }
            public string Preflight()
            {
                if (_cleanupPending) return "CLEANUP_PENDING";
                if (PelvisTestActive) return "PELVIS_TEST_ACTIVE";
                if (_legChain == null || !_legChain.IsBuilt)
                    return "ARL ankle v0.4.89：bilateral leg foundation尚未创建。";
                if (_proxy == null || !_proxy.IsBuilt || !_proxy.DriveEnabled)
                    return "ARL ankle v0.4.89：拒绝启动。请先保持 arl_drive(on)。";
                if (!_legChain.SourceContextReady) return "LEG_SOURCE_CONTEXT_NOT_READY | 等待下一动画帧后再启动。";
                if (!_legChain.FrozenProductionStackReady)
                    return "ARL ankle v0.4.89：拒绝启动。bilateral frozen production stack尚未READY；请先完成Knee PCA apply并执行 arl_hip_stack(on)。";
                if (!_legChain.FootProductionBaselineReady)
                    return "ARL ankle v0.4.89：拒绝启动。Foot frozen production contract/readback不完整。";
                if (_proxy.Report.GravityEnabled || (_legChain.PrimaryReport != null && _legChain.PrimaryReport.GravityEnabled) ||
                    (_legChain.SecondaryReport != null && _legChain.SecondaryReport.GravityEnabled))
                    return "ARL ankle v0.4.89：拒绝启动。通用torso/leg native gravity必须保持OFF；Foot/Lower gravity由冻结的V04180/V04190窄授权，Upper gravity仅由V04450 B exact-STANCE selector窄授权。";

                string expectedCapturePolicy = FootAnkleTranslationCaptureAuthorityIsolation.Production;
                if (!string.Equals(_legChain.TranslationCaptureAuthorityArm, expectedCapturePolicy, StringComparison.Ordinal) || !_legChain.TranslationCaptureAuthorityArmsMatch)
                    return "ARL ankle v0.4.89：frozen V0373 production capture policy bilateral readback失败。";
                string expectedAngularPolicy = FootAnkleAngularTransport.Production;
                if (!string.Equals(_legChain.AnkleAngularTransportPolicy, expectedAngularPolicy, StringComparison.Ordinal))
                    return "ARL ankle v0.4.89：common-mode angular transport policy readback失败。";

                if (_legChain.KneeAxisCalibrationCollecting) return "KNEE_PCA_STILL_COLLECTING";
                return null;
            }
            public string Prepare(string arm)
            {
                string upperLoadArm = arm;
                string kneeReactionArm = KneeSupportNormalReactionIsolation.ArmB;
                string surfaceArm = FootSupportSurfaceIsolation.ArmB; // Same controlled Box in both Upper self-weight arms.
                string contactOffsetArm = FootContactOffsetIsolation.ArmB; // Fixed background for BOTH Upper self-weight arms.
                string collisionArm = FootCollisionDetectionModeIsolation.ActiveArm;
                string supportAngularArm = FootStanceSupportNormalAngularStabilization.ArmB;
                string solverArm = FootSolverIterationBaselineContract.ArmA;

                string priorTrace = TraceActive ? SetTraceCore(false) : "trace=already-off";
                string priorLeg = _legChain.AnyDriveEnabled ? SetLegDriveCore(false) : "leg=already-off";
                string priorStance = _legChain.SetStanceOnlyContactQualification(false);
                _legChain.SetFootContactShadowArmA();

                string friction = _legChain.SetStanceContactFrictionIsolation(true);
                if (!_legChain.StanceContactZeroFriction || !_legChain.StanceContactFrictionIsolationFinite || !_legChain.StanceContactFrictionIsolationArmsMatch)
                    throw new InvalidOperationException("ANKLE_SETUP_FRICTION_FAILED | " + friction);

                string lowerGround = _legChain.SetStanceLowerGroundAnchorAuthority(true);
                if (!_legChain.StanceLowerGroundAuthorityFinite || !_legChain.StanceLowerGroundAuthorityArmsMatch ||
                    !string.Equals(_legChain.StanceLowerGroundAuthorityArm, FootStanceLowerGroundAnchorAuthority.ArmB, StringComparison.Ordinal) ||
                    !_legChain.StanceLowerGroundAuthorityEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_LOWERGROUND_FAILED | " + lowerGround);

                string predictive = _legChain.SetTranslationPredictiveTailCapture(true);
                if (!_legChain.TranslationPredictiveTailCaptureFinite || !_legChain.TranslationPredictiveTailCaptureArmsMatch ||
                    !string.Equals(_legChain.TranslationPredictiveTailCaptureArm, FootAnkleTranslationPredictiveTailCaptureIsolation.ArmB, StringComparison.Ordinal) ||
                    !_legChain.TranslationPredictiveTailCaptureEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_PREDICTIVE_FAILED | " + predictive);

                string footGravity = _legChain.SetStanceFootNativeGravityLoad(true);
                if (!_legChain.StanceFootNativeGravityFinite || !_legChain.StanceFootNativeGravityArmsMatch ||
                    !string.Equals(_legChain.StanceFootNativeGravityArm, FootStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) ||
                    !_legChain.StanceFootNativeGravityEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_FOOTGRAVITY_FAILED | " + footGravity);

                string lowerGravity = _legChain.SetStanceLowerNativeGravityLoad(true);
                if (!_legChain.StanceLowerNativeGravityFinite || !_legChain.StanceLowerNativeGravityArmsMatch ||
                    !string.Equals(_legChain.StanceLowerNativeGravityArm, LowerStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) ||
                    !_legChain.StanceLowerNativeGravityEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_LOWERGRAVITY_FAILED | " + lowerGravity);

                string reciprocal = _legChain.SetStanceFootLowerReciprocalLoadTransfer(true);
                if (!_legChain.StanceFootLowerReciprocalFinite || !_legChain.StanceFootLowerReciprocalArmsMatch ||
                    !string.Equals(_legChain.StanceFootLowerReciprocalArm, FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmB, StringComparison.Ordinal) ||
                    !_legChain.StanceFootLowerReciprocalEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_RECIPROCAL_FAILED | " + reciprocal);

                // Both V04450 arms retain the same V04210 static load path and V04430 B Knee PD.
                string transfer = _legChain.SetStanceLowerSelfWeightSupportTransfer(true);
                if (!_legChain.StanceLowerSelfWeightTransferFinite || !_legChain.StanceLowerSelfWeightTransferArmsMatch ||
                    !string.Equals(_legChain.StanceLowerSelfWeightTransferArm, LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB, StringComparison.Ordinal) ||
                    !_legChain.StanceLowerSelfWeightTransferEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_TRANSFER_FAILED | " + transfer);

                string kneeReaction = _legChain.SetKneeSupportNormalReaction(true);
                if (!_legChain.KneeSupportReactionArmsMatch || !_legChain.KneeSupportReactionFinite ||
                    _legChain.KneeSupportReactionArm != kneeReactionArm)
                    throw new InvalidOperationException("ANKLE_SETUP_KNEE_REACTION_FAILED | " + kneeReaction);

                string upperLoad = _legChain.SetUpperSelfWeightTransfer(upperLoadArm);
                if (!_legChain.UpperSelfWeightArmsMatch || !_legChain.UpperSelfWeightFinite || _legChain.UpperSelfWeightArm != upperLoadArm)
                    throw new InvalidOperationException("ANKLE_SETUP_UPPER_SELF_WEIGHT_FAILED | " + upperLoad);

                string stance = _legChain.SetStanceOnlyContactQualification(true);
                if (!_legChain.StanceOnlyContactLifecycleEnabled || !_legChain.StanceOnlyContactLifecycleFinite)
                    throw new InvalidOperationException("ANKLE_SETUP_STANCE_FAILED | " + stance);

                string legStart = SetLegDriveCore(true);
                if (!_legChain.DriveEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_LEGSTART_FAILED | " + legStart);

                // Apply the frozen Speculative CCD mode after the Foot becomes dynamic, before the next physics solve.
                string collisionMode = _legChain.SetFootCollisionDetectionModeArm(collisionArm);
                if (!_legChain.FootCollisionDetectionModeFinite || !_legChain.FootCollisionDetectionModeArmsMatch ||
                    !string.Equals(_legChain.FootCollisionDetectionModeArm, collisionArm, StringComparison.Ordinal) ||
                    !FootCcdSelectorContract(_legChain.PrimaryFootReport, collisionArm) ||
                    !FootCcdSelectorContract(_legChain.SecondaryFootReport, collisionArm))
                    throw new InvalidOperationException("ANKLE_SETUP_COLLISIONMODE_FAILED | " + collisionMode);

                string contactOffset = _legChain.SetFootContactOffsetArm(contactOffsetArm);
                if (!FootContactOffsetSelectorContract(_legChain.PrimaryFootReport, contactOffsetArm) ||
                    !FootContactOffsetSelectorContract(_legChain.SecondaryFootReport, contactOffsetArm))
                    throw new InvalidOperationException("ANKLE_SETUP_CONTACTOFFSET_FAILED | " + contactOffset);

                string supportAngular = _legChain.SetStanceSupportAngularStabilization(true);
                if (!_legChain.StanceSupportAngularFinite || !_legChain.StanceSupportAngularArmsMatch ||
                    !string.Equals(_legChain.StanceSupportAngularArm, supportAngularArm, StringComparison.Ordinal) ||
                    !_legChain.StanceSupportAngularEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_SUPPORTANGULAR_FAILED | " + supportAngular);

                string velocitySolver = _legChain.RestoreFootSolverIterationBaseline();
                if (!_legChain.FootVelocitySolverFinite || !_legChain.FootVelocitySolverArmsMatch ||
                    !string.Equals(_legChain.FootVelocitySolverArm, solverArm, StringComparison.Ordinal))
                    throw new InvalidOperationException("ANKLE_SETUP_VELOCITYSOLVER_FAILED | " + velocitySolver);

                string planarDamping = _legChain.SetStancePlanarDamping(true);
                if (!_legChain.StancePlanarDampingEnabled)
                    throw new InvalidOperationException("ANKLE_SETUP_PLANARDAMPING_FAILED | " + planarDamping);

                string positionSolver = _legChain.SetFootPositionSolverArm(FootSolverIterationBaselineContract.PositionArmB);
                if (!_legChain.FootPositionSolverEnabled || !_legChain.FootPositionSolverArmsMatch || !_legChain.FootVelocitySolverFinite)
                    throw new InvalidOperationException("ANKLE_SETUP_POSITIONSOLVER_FAILED | " + positionSolver);

                string supportSurface = _legChain.ConfigureFootSupportSurface(surfaceArm);
                if (!_legChain.SupportSurface.Active || !_legChain.SupportSurface.Finite || _legChain.SupportSurface.Arm != surfaceArm)
                    throw new InvalidOperationException("ANKLE_SETUP_SUPPORTSURFACE_FAILED | " + supportSurface);

                string trace = SetTraceCore(true);
                if (!SharedTraceReady)
                    throw new InvalidOperationException("ANKLE_SETUP_TRACE_FAILED | " + trace);

                string result = "frozenLegFootStack=V04460_HIP_PELVIS_READ_ONLY" +
                                " | arm=" + upperLoadArm +
                                " | V04460HipPelvisAudit=" + (_legChain.HipPelvisLoadPath.Selected ? "READ_ONLY_ACTIVE" : "INACTIVE_LEGACY_A_COMPARISON") +
                                " | V04460ReceiverCaptured=" + _legChain.HipPelvisLoadPath.ReceiverCaptured +
                                "/auditFinite=" + _legChain.HipPelvisLoadPath.Finite + "/issue=" + (_legChain.HipPelvisLoadPath.Issue ?? "<none>") +
                                " | upperSelfWeight={" + upperLoad + "}" +
                                " | kneeSupportReaction={" + kneeReaction + "}" +
                                " | supportSurface={" + supportSurface + "}" +
                                " | contactOffsetBackground=FROZEN_20MM_BOTH_ARMS" +
                                " | contactOffset={" + contactOffset + "}" +
                                " | V04370AngularModel=READ_ONLY_ORDINARY_PRE_SOLVER_DAMPING_CAP_AND_INERTIA_BASIS_COMPARISON | V04330NormalLoadAudit=READ_ONLY_REUSED_SAMPLES | V04350Windows=READ_ONLY_RAW_DRAG_MODEL_UNLOADED_INPUTS_PRE2_POST2_MAX8_PER_SIDE_FIXED_NORMAL_SUMS | V04310={" + positionSolver + "}" +
                                " | V04300={" + planarDamping + "}" +
                                " | capturedBaselineRestoreBeforeV04310={" + velocitySolver.Replace('\n', ' ') + "}" +
                                " | FootCollisionMode=" + _legChain.PrimaryFootReport.FootCollisionDetectionModeReadback +
                                " | CCDObservedExposure=REQUIRES_PRE_FIRST_LAST_SETTLED_MATCHED_LOADED_SOLVES" +
                                " | V04260 arm=B_FROZEN" + "/enabled=" + _legChain.StanceSupportAngularEnabled + "/ratePerSec=30/maxTorqueNm=1" +
                                " | V04210SelfWeightTransfer=FROZEN_B_ON | FootNativeGravity=FROZEN_EXACT_STANCE_ON | LowerNativeGravity=FROZEN_EXACT_STANCE_ON | V04200Reciprocal=FROZEN_B_ON" +
                                " | UpperGravity=V04450_EXACT_STANCE_B_ONLY | TorsoGravity=OFF | GSP=OFF | predictiveTail=V04171_B_FROZEN_ON_MAXA140_RARE_GUARD | zeroFriction=True" +
                                " | stanceLifecycle=V0470_FROZEN | lowerPlanarGround=FROZEN_B_20_6_12 | V0373=FROZEN_60_80 | Foot/KneeGateMm=30_UNCHANGED" +
                                " | prior={" + priorTrace + " ; " + priorLeg + " ; " + priorStance.Replace('\n', ' ') + "}" +
                                " | friction={" + friction.Replace('\n', ' ') + "} | lowerGround={" + lowerGround.Replace('\n', ' ') +
                                "} | predictive={" + predictive.Replace('\n', ' ') + "} | footGravity={" + footGravity.Replace('\n', ' ') +
                                "} | lowerGravity={" + lowerGravity.Replace('\n', ' ') + "} | reciprocal={" + reciprocal.Replace('\n', ' ') +
                                "} | supportAngular={" + supportAngular.Replace('\n', ' ') + "} | transfer={" + transfer.Replace('\n', ' ') + "} | collisionMode={" + collisionMode.Replace('\n', ' ') +
                                "} | stance={" + stance.Replace('\n', ' ') + "} | trace=" + trace;
                return result;
            }
            public string HealthIssue
            {
                get
                {
                    if (_proxy == null || !_proxy.IsBuilt || !_proxy.DriveEnabled || !_proxy.Report.FiniteState)
                        return "AUTO_TORSO_SCOPE_LOST";
                    if (_legChain == null || !_legChain.IsBuilt) return "AUTO_LEG_SCOPE_LOST";
                    if (!_legChain.SupportSurface.Finite || !_legChain.SupportSurface.Active) return "AUTO_SUPPORT_SCOPE_ABORT";
                    if (_legChain.AnyDriveAborted || !_legChain.DriveEnabled || (_legChain.BilateralReport?.FailClosedEvents ?? 0) > 0)
                        return "AUTO_LEG_DRIVE_ABORT";
                    if (!SharedTraceReady) return "AUTO_SHARED_TRACE_LOST";
                    if (_legChain.KneeAxisCalibrationCollecting || _legChain.UpperSelfWeightArm != _ankleTest.Session.Arm)
                        return "AUTO_LEG_CONFIGURATION_CHANGED";
                    return null;
                }
            }
            public bool Ready => HealthIssue == null && (_ankleTest.Session.Arm != UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB ||
                (_legChain.HipPelvisLoadPath.Selected && _legChain.HipPelvisLoadPath.ReceiverCaptured && _legChain.HipPelvisLoadPath.Finite));
            public string ReadinessSummary() => "sharedTraceReady=" + SharedTraceReady + " | legStackReady=" + (_legChain != null && _legChain.FrozenProductionStackReady) +
                " | V04460ReceiverCaptured=" + (_legChain != null && _legChain.HipPelvisLoadPath.ReceiverCaptured) +
                " | auditFinite=" + (_legChain != null && _legChain.HipPelvisLoadPath.Finite) +
                " | sourceContextReady=" + (_legChain != null && _legChain.SourceContextReady) + " | healthIssue=" + (HealthIssue ?? "<none>");
            public AnkleRunEvidence QuiesceAndCapture(bool started, string header, StringBuilder cleanup)
            {
                if (TraceActive) SetTraceCore(false);
                if (_legChain != null)
                {
                    cleanup.AppendLine(_legChain.QuiesceStanceOnlyContactPreserveTrace());
                    cleanup.AppendLine(_legChain.QuiesceStanceLowerNativeGravityPreserveTrace());
                    cleanup.AppendLine(_legChain.QuiesceUpperSelfWeightPreserveTrace());
                }
                if (!started) return new AnkleRunEvidence { Audit = "resultScope=REJECTED_SETUP | " + header,
                    Snapshot = "[FAILED_BEFORE_RESTORE]\n" + BuildAnkleLiveDetails() };
                string audit = "resultScope=COMPLETED_RUN | " + header + "\n" + BuildFootCollisionSupportLoadContinuityAudit(true);
                return new AnkleRunEvidence { Audit = audit, Snapshot = BuildMaintenanceSnapshot(null, true) };
            }
            public bool Restore(bool started, StringBuilder cleanup)
            {
                try
                {
                    bool restored = RestoreAnkleTestSafeState(cleanup);
                    if (!started) restored &= TryAnkleCleanup("TORSO_DRIVE_OFF", () => SetDriveCore(false), cleanup);
                    bool safe = restored && (started ? AnkleLegStopReadbackSafe() : RejectedAnklePreviewSafe());
                    if (!safe) cleanup.AppendLine("UNSAFE_STOP_FULL_ROLLBACK | " + RollbackInternal("ANKLE_STOP_RESTORE_NOT_SAFE"));
                    return safe;
                }
                catch (Exception e)
                {
                    _cleanupPending = true;
                    cleanup.AppendLine("RESTORE_EXCEPTION=" + e.GetType().Name);
                    cleanup.AppendLine(RollbackInternal("ANKLE_RESTORE_EXCEPTION"));
                    return false;
                }
            }
            public string LiveDetails() => BuildAnkleLiveDetails();
            public string Notice() => _automaticStop.Status();
            public void BeginNoticeSession() { _automaticStop.BeginDriveSession(AnkleTestCoordinator.StopCommand); }
            public void AutomaticStop(string reason)
            {
                string cause = reason;
                if (_legChain != null && reason == "AUTO_SUPPORT_SCOPE_ABORT" && !string.IsNullOrEmpty(_legChain.SupportSurface.InvalidReason))
                    cause = _legChain.SupportSurface.InvalidReason;
                else if (_legChain != null && reason == "AUTO_LEG_DRIVE_ABORT")
                {
                    if (_legChain.PrimaryReport != null && !string.IsNullOrEmpty(_legChain.PrimaryReport.DriveAbortReason))
                        cause = _legChain.PrimaryReport.DriveAbortReason;
                    else if (_legChain.SecondaryReport != null && !string.IsNullOrEmpty(_legChain.SecondaryReport.DriveAbortReason))
                        cause = _legChain.SecondaryReport.DriveAbortReason;
                    else if (_legChain.BilateralReport != null && !string.IsNullOrEmpty(_legChain.BilateralReport.LastFailClosedReason))
                        cause = _legChain.BilateralReport.LastFailClosedReason;
                }
                RecordAutomaticStop("ANKLE_BUNDLE", cause);
            }
            public void Emit(string text, bool warning, bool deferred)
            {
                if (deferred) _deferredCompletedOutput = text;
                else if (warning) _logger?.LogWarning(text);
                else _logger?.LogInfo(text);
            }
            public void WriteReports() { WriteAllReports(); }
        }

        private static bool RejectedAnklePreviewSafe()
        {
            if (_proxy == null || !_proxy.IsBuilt || _proxy.Report == null || _legChain == null || !_legChain.IsBuilt ||
                _legChain.PrimaryReport == null || _legChain.SecondaryReport == null ||
                _legChain.PrimaryFootReport == null || _legChain.SecondaryFootReport == null)
                return false;
            return !AnkleTestActive && !_proxy.DriveEnabled && _proxy.Report.FiniteState &&
                !_proxy.Report.GravityEnabled && !_proxy.Report.CollidersEnabled && AnkleLegStopReadbackSafe() &&
                FootFreeSpaceContract(_legChain.PrimaryFootReport) && FootFreeSpaceContract(_legChain.SecondaryFootReport) &&
                _legChain.PrimaryFootReport.Kinematic && _legChain.SecondaryFootReport.Kinematic &&
                _legChain.PrimaryReport.Finite && _legChain.SecondaryReport.Finite &&
                !_legChain.PrimaryReport.DriveEnabled && !_legChain.SecondaryReport.DriveEnabled &&
                !_legChain.PrimaryReport.GravityEnabled && !_legChain.SecondaryReport.GravityEnabled &&
                !_legChain.PrimaryReport.CollidersEnabled && !_legChain.SecondaryReport.CollidersEnabled &&
                !_legChain.PrimaryReport.StanceLowerNativeGravityCurrentApplied &&
                !_legChain.SecondaryReport.StanceLowerNativeGravityCurrentApplied &&
                !_legChain.PrimaryFootReport.StanceFootNativeGravityCurrentApplied &&
                !_legChain.SecondaryFootReport.StanceFootNativeGravityCurrentApplied &&
                !_legChain.StanceLowerGroundAuthorityEnabled && !_legChain.TranslationPredictiveTailCaptureEnabled &&
                !_legChain.StanceContactZeroFriction;
        }

        private static bool RestoreAnkleTestSafeState(StringBuilder cleanup)
        {
            bool success = true;
            // A failed cleanup step must not prevent the remaining independent safe-state steps.
            success &= TryAnkleCleanup("TRACE_OFF", () => SetTraceCore(false), cleanup);
            if (_legChain != null)
            {
                success &= TryAnkleCleanup("UPPER_SELF_WEIGHT_OFF", () => _legChain.SetUpperSelfWeightTransfer(UpperSelfWeightSupportNormalLoadTransferIsolation.IdleArm), cleanup);
                success &= TryAnkleCleanup("KNEE_SUPPORT_REACTION_OFF", () => _legChain.SetKneeSupportNormalReaction(false), cleanup);
                success &= TryAnkleCleanup("STANCE_PLANAR_DAMPING_OFF", () => _legChain.SetStancePlanarDamping(false), cleanup);
                success &= TryAnkleCleanup("FOOT_SOLVER_CAPTURED_BASELINE_RESTORE", () => _legChain.RestoreFootSolverIterationBaseline(), cleanup);
                success &= TryAnkleCleanup("SUPPORT_ANGULAR_OFF", () => _legChain.SetStanceSupportAngularStabilization(false), cleanup);
                success &= TryAnkleCleanup("STANCE_OFF", () => _legChain.SetStanceOnlyContactQualification(false), cleanup);
                success &= TryAnkleCleanup("SUPPORT_SURFACE_OFF", () => _legChain.StopFootSupportSurface(), cleanup);
                success &= TryAnkleCleanup("FOOT_CONTACT_OFFSET_CAPTURED_BASELINE_RESTORE", () => _legChain.SetFootContactOffsetArm(FootContactOffsetIsolation.IdleArm), cleanup);
                success &= TryAnkleCleanup("FOOT_COLLISION_MODE_IDLE_DISCRETE", () => _legChain.SetFootCollisionDetectionModeArm(FootCollisionDetectionModeIsolation.IdleArm), cleanup);
                success &= TryAnkleCleanup("SELF_WEIGHT_OFF", () => _legChain.SetStanceLowerSelfWeightSupportTransfer(false), cleanup);
                success &= TryAnkleCleanup("RECIPROCAL_OFF", () => _legChain.SetStanceFootLowerReciprocalLoadTransfer(false), cleanup);
                success &= TryAnkleCleanup("LOWER_GRAVITY_OFF", () => _legChain.SetStanceLowerNativeGravityLoad(false), cleanup);
                success &= TryAnkleCleanup("FOOT_GRAVITY_OFF", () => _legChain.SetStanceFootNativeGravityLoad(false), cleanup);
                success &= TryAnkleCleanup("PREDICTIVE_OFF", () => _legChain.SetTranslationPredictiveTailCapture(false), cleanup);
                success &= TryAnkleCleanup("LOWER_GROUND_OFF", () => _legChain.SetStanceLowerGroundAnchorAuthority(false), cleanup);
                success &= TryAnkleCleanup("FRICTION_OFF", () => _legChain.SetStanceContactFrictionIsolation(false), cleanup);
                success &= TryAnkleCleanup("LEG_DRIVE_OFF", () => SetLegDriveCore(false), cleanup);
            }
            return success;
        }

        private static bool AnkleLegStopReadbackSafe()
        {
            return !TraceActive && (_legChain == null ||
                (!_legChain.AnyDriveEnabled && _legChain.UpperSelfWeightOff && _legChain.KneeSupportReactionOff && _legChain.SupportSurface.StoppedSafe && _legChain.FootContactShadowFinite && _legChain.FootPositionSolverOff && _legChain.StancePlanarDampingOff && !_legChain.StanceOnlyContactLifecycleEnabled && !_legChain.StanceSupportAngularEnabled &&
                 _legChain.FootVelocitySolverFinite && _legChain.FootVelocitySolverArmsMatch && _legChain.FootVelocitySolverBaselineRestored &&
                 string.Equals(_legChain.FootVelocitySolverArm, FootSolverIterationBaselineContract.ArmA, StringComparison.Ordinal) &&
                 _legChain.StanceSupportAngularFinite && _legChain.StanceSupportAngularArmsMatch &&
                 string.Equals(_legChain.StanceSupportAngularArm, FootStanceSupportNormalAngularStabilization.ArmA, StringComparison.Ordinal) &&
                 !_legChain.StanceLowerSelfWeightTransferEnabled && !_legChain.StanceFootLowerReciprocalEnabled &&
                 !_legChain.StanceLowerNativeGravityEnabled && !_legChain.StanceFootNativeGravityEnabled &&
                 _legChain.FootCollisionDetectionModeArmsMatch && _legChain.FootCollisionDetectionModeFinite &&
                 string.Equals(_legChain.FootCollisionDetectionModeArm, FootCollisionDetectionModeIsolation.IdleArm, StringComparison.Ordinal) &&
                 FootCcdSelectorContract(_legChain.PrimaryFootReport, FootCollisionDetectionModeIsolation.IdleArm) &&
                 FootCcdSelectorContract(_legChain.SecondaryFootReport, FootCollisionDetectionModeIsolation.IdleArm) &&
                 FootContactOffsetSelectorContract(_legChain.PrimaryFootReport, FootContactOffsetIsolation.IdleArm) &&
                 FootContactOffsetSelectorContract(_legChain.SecondaryFootReport, FootContactOffsetIsolation.IdleArm) &&
                 _legChain.PrimaryFootReport.FootContactOffsetBaselineRestored && _legChain.SecondaryFootReport.FootContactOffsetBaselineRestored &&
                 (_legChain.PrimaryFootReport == null || (!_legChain.PrimaryFootReport.ColliderEnabled && !_legChain.PrimaryFootReport.GravityEnabled)) &&
                 (_legChain.SecondaryFootReport == null || (!_legChain.SecondaryFootReport.ColliderEnabled && !_legChain.SecondaryFootReport.GravityEnabled))));
        }

        private static string BuildAnkleLiveDetails()
        {
            BilateralLegReport bilateral = _legChain != null ? _legChain.BilateralReport : null;
            return "ARL ankle legacy-stack | stateScope=LIVE_STATE" +
                   " | testArm=" + _ankleTestArm +
                   " | runId=" + _ankleTest.Session.RunId +
                   " | ankleTestActive=" + AnkleTestActive +
                   " | completed=" + (_completedAnkleTestSummary ?? "<none>") +
                   " | upperSelfWeightArm=" + (_legChain?.UpperSelfWeightArm ?? "<none>") +
                   " | upperSelfWeightOff=" + (_legChain == null || _legChain.UpperSelfWeightOff) +
                   " | kneeReactionArm=" + (_legChain?.KneeSupportReactionArm ?? "<none>") +
                   " | kneeReactionOff=" + (_legChain == null || _legChain.KneeSupportReactionOff) +
                   " | positionSolverArm=" + (_legChain != null ? _legChain.FootPositionSolverArm : "<none>") +
                   " | positionSolverEnabled=" + (_legChain != null && _legChain.FootPositionSolverEnabled) +
                   " | positionSolverOff=" + (_legChain == null || _legChain.FootPositionSolverOff) +
                   " | velocitySolverArm=" + (_legChain != null ? _legChain.FootVelocitySolverArm : "<none>") +
                   " | velocitySolverFinite=" + (_legChain != null && _legChain.FootVelocitySolverFinite) +
                   " | velocitySolverBaselineRestored=" + (_legChain != null && _legChain.FootVelocitySolverBaselineRestored) +
                   " | stancePlanarDampingEnabled=" + (_legChain != null && _legChain.StancePlanarDampingEnabled) +
                   " | stancePlanarDampingOff=" + (_legChain == null || _legChain.StancePlanarDampingOff) +
                   " | supportAngularArm=" + (_legChain != null ? _legChain.StanceSupportAngularArm : "<none>") +
                   " | supportAngularEnabled=" + (_legChain != null && _legChain.StanceSupportAngularEnabled) +
                   " | footCollisionArm=" + (_legChain != null ? _legChain.FootCollisionDetectionModeArm : "<none>") +
                   " | supportSurface=" + (_legChain != null ? _legChain.SupportSurface.Summary() : "<none>") +
                   " | footContactOffset(left-right)=" + Clean(_legChain?.PrimaryFootReport?.FootContactOffsetSummary) + ":" + Clean(_legChain?.SecondaryFootReport?.FootContactOffsetSummary) +
                   " | footCollisionReadback(left-right)=" + (_legChain?.PrimaryFootReport?.FootCollisionDetectionModeReadback ?? "<none>") + "-" +
                   (_legChain?.SecondaryFootReport?.FootCollisionDetectionModeReadback ?? "<none>") +
                   " | selfWeightTransferArm=" + (_legChain != null ? _legChain.StanceLowerSelfWeightTransferArm : "<none>") +
                   " | selfWeightTransferEnabled=" + (_legChain != null && _legChain.StanceLowerSelfWeightTransferEnabled) +
                   " | reciprocalArm=" + (_legChain != null ? _legChain.StanceFootLowerReciprocalArm : "<none>") +
                   " | reciprocalEnabled=" + (_legChain != null && _legChain.StanceFootLowerReciprocalEnabled) +
                   " | footGravity=" + (_legChain != null ? _legChain.StanceFootNativeGravityArm : "<none>") +
                   " | lowerGravity=" + (_legChain != null ? _legChain.StanceLowerNativeGravityArm : "<none>") +
                   " | predictiveArm=" + (_legChain != null ? _legChain.TranslationPredictiveTailCaptureArm : "<none>") +
                   " | zeroFriction=" + (_legChain != null && _legChain.StanceContactZeroFriction) +
                   " | stanceLifecycleEnabled=" + (_legChain != null && _legChain.StanceOnlyContactLifecycleEnabled) +
                   " | lowerPlanarGroundArm=" + (_legChain != null ? _legChain.StanceLowerGroundAuthorityArm : "<none>") +
                   " | bilateral(clock/failClosed/abort)=" + (bilateral != null && bilateral.ClockParityPass) + "/" + (bilateral != null ? bilateral.FailClosedEvents : 0) + "/" + (bilateral != null && bilateral.PrimaryDriveAborted) + "-" + (bilateral != null && bilateral.SecondaryDriveAborted) +
                   " | publicTorsoLegGravity=OFF | UpperGravity=V04450_EXACT_STANCE_B_ONLY | globalPhysicsGravity=" + UnityEngine.Physics.gravity + " | drive=" + (_legChain != null && _legChain.DriveEnabled) + " | trace=" + TraceActive;
        }

    }
}
