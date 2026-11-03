using System;
using System.IO;
using System.Text;
using BepInEx;
using BepInEx.Logging;
using MakemitAGA.ActiveRagdollLab.Acquisition;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    internal static partial class ActiveRagdollLabRuntime
    {
        internal static bool IsBuilt => !_cleanupPending && _proxy != null && _proxy.IsBuilt;
        internal static bool AuditReady => !_cleanupPending && _audit != null && _audit.Success;
        private static bool HasActiveFoundation => _proxy != null || _legChain != null;
        internal static bool TraceActive =>
            (_proxy != null && _proxy.TraceEnabled) ||
            (_legChain != null && _legChain.TraceEnabled);
        internal static bool LastRollbackSucceeded { get; private set; } = true;
        private static ManualLogSource _logger;
        private static ActiveRagdollLabProfile _profile;
        private static BoneAuditReport _audit;
        private static TwoBodyShadowProxy _proxy;
        private static BilateralLegCoordinator _legChain;
        private static SingleLegTargetReport _legTarget;
        private static SingleLegTargetReport _secondaryLegTarget;
        private static string _sceneName;
        private static int _sceneHandle;
        private static bool _shutdown;
        private static PelvisTestCoordinator _pelvisTest;
        private static PelvisRunScope _pelvisScope;
        private static bool _pelvisWithLegs => _pelvisScope != PelvisRunScope.TorsoPairOnly;
        private static bool _pelvisWithContact => PelvisRunScopes.IsContact(_pelvisScope);
        private static RuntimeStepClock _stepClock = new RuntimeStepClock();
        private static bool _cleanupPending;
        private static string _runtimeStage = "IDLE";
        internal static bool PelvisTestActive => _pelvisTest != null && _pelvisTest.Session.Active;
        private static string _completedPelvisTestSnapshot => _pelvisTest?.Session.CompletedSnapshot;
        private static string _lastRejectedPelvisSetup => _pelvisTest?.Session.RejectedSnapshot;
        internal static bool PelvisStatusIsCurrentScope => PelvisTestActive ||
            (_lastTestScope == "PELVIS" && !AnkleTestActive && !TraceActive && (_proxy == null || !_proxy.DriveEnabled) &&
             (_completedPelvisTestSnapshot != null || _lastRejectedPelvisSetup != null));
        private static AnkleTestCoordinator _ankleTest;
        private static string _lastTestScope = "PELVIS"; // Status routing only; session state stays in RunSession.
        internal static bool AnkleTestActive => _ankleTest != null && _ankleTest.Session.Active;
        private static string _ankleTestArm => _ankleTest != null && _ankleTest.Session.OwnsScope ? _ankleTest.Session.Arm : "<none>";
        private static string _completedAnkleTestAudit => _ankleTest?.CompletedAudit;
        private static string _completedAnkleTestSnapshot => _ankleTest?.Session.CompletedSnapshot;
        private static string _completedAnkleTestSummary => _ankleTest?.Session.CompletedSummary;
        private static string _lastRejectedAnkleSetup => _ankleTest?.Session.RejectedSnapshot;
        private static string _deferredCompletedOutput;
        internal static void FlushDeferredDiagnostics()
        {
            string output = _deferredCompletedOutput;
            if (output == null) return;
            _deferredCompletedOutput = null;
            _logger?.LogInfo(output);
        }
        private static AutomaticStopNotice _automaticStop = new AutomaticStopNotice();
        private static string BoneAuditPath => Path.Combine(Paths.ConfigPath, "MakemitAGA.ActiveRagdollLab.BoneAudit.txt");
        private static string ProxyReportPath => Path.Combine(Paths.ConfigPath, "MakemitAGA.ActiveRagdollLab.TwoBodyReport.txt");
        private static string SingleLegReportPath => Path.Combine(Paths.ConfigPath, "MakemitAGA.ActiveRagdollLab.SingleLegReport.txt");
        private static string SecondaryLegReportPath => Path.Combine(Paths.ConfigPath, "MakemitAGA.ActiveRagdollLab.SecondaryLegReport.txt");
        private static string BilateralLegReportPath => Path.Combine(Paths.ConfigPath, "MakemitAGA.ActiveRagdollLab.BilateralLegReport.txt");
        private static string KneeSafetyContextReportPath => Path.Combine(Paths.ConfigPath, "MakemitAGA.ActiveRagdollLab.KneeSafetyContextReport.txt");
        private static string FootFoundationReportPath => Path.Combine(Paths.ConfigPath, "MakemitAGA.ActiveRagdollLab.FootFoundationReport.txt");

        public static void Initialize(ManualLogSource logger)
        {
            _logger = logger;
            _profile = ActiveRagdollLabProfile.Load(logger);
            _shutdown = false;
            _pelvisTest = new PelvisTestCoordinator(new PelvisRunPort());
            _stepClock = new RuntimeStepClock(); _cleanupPending = false; _runtimeStage = "IDLE";
            _ankleTest = new AnkleTestCoordinator(new AnkleRunPort());
            _lastTestScope = "PELVIS"; _pelvisScope = PelvisRunScope.TorsoPairOnly;
            _deferredCompletedOutput = null;
            _automaticStop = new AutomaticStopNotice();
            LastRollbackSucceeded = true;
        }

        public static void OnSceneChanged(string sceneName, int sceneHandle)
        {
            if (PelvisTestActive || AnkleTestActive || (_proxy != null && _proxy.DriveEnabled) || (_legChain != null && _legChain.AnyDriveEnabled))
                RecordAutomaticStop("SCENE", "SCENE_CHANGE:" + sceneName + "#" + sceneHandle);
            RollbackInternal("scene-change:" + sceneName + "#" + sceneHandle);
            _audit = null;
            _legTarget = null;
            _secondaryLegTarget = null;
            _sceneName = sceneName;
            _sceneHandle = sceneHandle;
            _logger?.LogInfo("[ARL] Cleared | reason=scene-change:" + sceneName + "#" + sceneHandle);
        }

        public static void SampleSourcePoseLateUpdate()
        {
            if (_shutdown || _cleanupPending)
                return;

            // Production torso sampling remains first; bilateral legs consume the same source frame afterwards.
            try
            {
                _runtimeStage = "SOURCE_TORSO";
                SourceStepContext source = _stepClock.CaptureSource(Time.unscaledTime, Time.unscaledDeltaTime);
                _proxy?.SampleSourcePose(source);
            }
            catch (Exception e)
            {
                FailCloseRuntimeLoop("TORSO_LATEUPDATE_SAMPLE", e);
                return;
            }

            try
            {
                _runtimeStage = "SOURCE_LEGS";
                _legChain?.SampleSourcePose(_stepClock.LastSource);
                _runtimeStage = "SOURCE_COMPLETE";
            }
            catch (Exception e)
            {
                FailCloseRuntimeLoop("BILATERAL_LEG_LATEUPDATE_SAMPLE", e);
            }
        }

        public static void FixedUpdate()
        {
            if (_shutdown || _cleanupPending)
                return;

            try
            {
                _runtimeStage = "FIXED_TORSO";
                PhysicsStepContext step = _stepClock.CaptureFixed(Time.fixedTime, Time.fixedDeltaTime, Time.unscaledTime);
                _proxy?.FixedUpdate(step);
            }
            catch (Exception e)
            {
                FailCloseRuntimeLoop("TORSO_FIXEDUPDATE", e);
                return;
            }

            try
            {
                _runtimeStage = "FIXED_LEGS";
                _legChain?.FixedUpdate(_stepClock.LastFixed);
            }
            catch (Exception e)
            {
                FailCloseRuntimeLoop("BILATERAL_LEG_FIXEDUPDATE", e);
                return;
            }

            if (PelvisTestActive)
            {
                _runtimeStage = "SESSION_READINESS";
                _pelvisTest.OnFixed(_stepClock.LastFixed);
                _runtimeStage = "FIXED_COMPLETE";
                return; // One typed session owns torso-only, dynamic free-space, or unloaded stance contact.
            }

            if (AnkleTestActive)
            {
                _runtimeStage = "SESSION_READINESS";
                _ankleTest.OnFixed(_stepClock.LastFixed);
            }
            else if (_legChain != null && _legChain.AnyDriveAborted && TraceActive)
                _logger?.LogWarning("[ARL][TRACE_AUTO_STOP] reason=LEG_DRIVE_ABORT | " + SetTrace(false));
            _runtimeStage = "FIXED_COMPLETE";

        }

        private static void FailCloseRuntimeLoop(string stage, Exception exception)
        {
            string type = exception != null ? exception.GetType().Name : "<unknown>";
            string message = exception != null ? exception.Message : "<none>";
            RecordAutomaticStop("RUNTIME/" + stage, "RUNTIME_EXCEPTION:" + type + ":" + message);
            try { _logger?.LogError("[ARL][RUNTIME_FAIL_CLOSED] stage=" + stage + " | " + type + " / " + message); }
            catch (Exception) { } // never skip rollback because the error logger itself failed
            string rollback = RollbackInternal("runtime-fail-closed:" + stage + ":" + type);
            try { _logger?.LogError("[ARL][RUNTIME_FAIL_CLOSED_ROLLBACK] stage=" + stage + " | " + rollback); }
            catch (Exception) { }
        }

        public static string EnsureProfileTemplate()
        {
            return ActiveRagdollLabProfile.EnsureTemplate(_logger);
        }

        public static string ReloadProfile()
        {
            if (HasActiveFoundation)
                return "ARL profile reload 被拒绝：请先执行 arl_rollback，避免运行中改变刚体、关节或 production soft-Knee 参数。";
            _profile = ActiveRagdollLabProfile.Load(_logger);
            _audit = null;
            _legTarget = null;
            _secondaryLegTarget = null;
            return _profile.GetStatusText() + " | auditInvalidated=True";
        }

        public static string GetProfileStatus()
        {
            if (_profile == null)
                _profile = ActiveRagdollLabProfile.Load(_logger);
            return _profile.GetStatusText();
        }

        public static string Audit(string optionalTargetName)
        {
            if (_profile == null)
                _profile = ActiveRagdollLabProfile.Load(_logger);

            string targetName = string.IsNullOrWhiteSpace(optionalTargetName)
                ? _profile.TargetName
                : optionalTargetName.Trim();

            if (HasActiveFoundation || _cleanupPending)
            {
                CleanupReceipt cleanup = RollbackWithReceipt("new-audit");
                if (!cleanup.Succeeded) return "ARL audit：清理尚未完成，请先arl_test_cleanup；保留旧对象以便重试。";
            }

            _audit = BoneHierarchyAuditor.Audit(targetName, _profile);
            _legTarget = null;
            _secondaryLegTarget = null;
            if (_audit.Success && _profile.SingleLegEnabled)
            {
                _legTarget = SingleLegTargetResolver.Resolve(_audit.TargetRoot, _profile.SingleLegSide, _profile.SingleLegUpperBonePath, _profile.SingleLegLowerBonePath, _profile.SingleLegFootBonePath);
                AppendLegTargetAuditLine(_audit, "PRIMARY", _legTarget);

                if (_profile.BilateralLegEnabled)
                {
                    string secondarySide = OppositeLegSide(_legTarget.RequestedSide);
                    // The legacy explicit path triplet belongs only to the configured primary side. The opposite side
                    // deliberately uses the already validated Humanoid/MiSide-canonical/structural resolver.
                    _secondaryLegTarget = SingleLegTargetResolver.Resolve(_audit.TargetRoot, secondarySide, string.Empty, string.Empty, string.Empty);
                    AppendLegTargetAuditLine(_audit, "SECONDARY", _secondaryLegTarget);
                }
            }
            WriteTextFile(BoneAuditPath, _audit.GetFullText());

            string summary = _audit.GetConsoleSummary() + " | fullReport=" + BoneAuditPath;
            if (_profile.SingleLegEnabled)
            {
                summary += " | primaryLeg=" + (_legTarget != null && _legTarget.Success) +
                           "/" + (_legTarget != null ? _legTarget.RequestedSide : "<none>") +
                           "/" + (_legTarget != null ? _legTarget.ResolutionMode : "<none>") +
                           "/chain=" + (_legTarget != null ? (_legTarget.ChainValidation ?? "<none>") : "<none>") +
                           "/footReady=" + (_legTarget != null && _legTarget.FootFoundationReady) +
                           "/toe=" + (_legTarget != null ? (_legTarget.ToePath ?? "<missing>") : "<none>") +
                           " | bilateral=" + _profile.BilateralLegEnabled +
                           "/secondary=" + (!_profile.BilateralLegEnabled || (_secondaryLegTarget != null && _secondaryLegTarget.Success)) +
                           "/" + (_secondaryLegTarget != null ? _secondaryLegTarget.RequestedSide : "<disabled>") +
                           "/footReady=" + (!_profile.BilateralLegEnabled || (_secondaryLegTarget != null && _secondaryLegTarget.FootFoundationReady));
            }
            if (_audit.Success)
                _logger?.LogWarning("[ARL][BONE_AUDIT] " + summary);
            else
                _logger?.LogWarning("[ARL][BONE_AUDIT] failed | " + summary);
            return summary;
        }

        public static string WriteAuditReportAndGetSummary()
        {
            if (_audit == null)
                return "ARL bone audit：尚无报告。先执行 arl_audit。";
            WriteTextFile(BoneAuditPath, _audit.GetFullText());
            return _audit.GetConsoleSummary() + " | fullReport=" + BoneAuditPath;
        }

        public static string Build(string optionalTargetName) => BuildCore(optionalTargetName, false);
        public static string BuildPelvisTestFoundation() => BuildCore(null, true);
        private static string BuildCore(string optionalTargetName, bool torsoOnly)
        {
            if (_profile == null)
                _profile = ActiveRagdollLabProfile.Load(_logger);

            if (HasActiveFoundation || _cleanupPending)
                return "ARL build：代理或待清理对象仍然存在。请先执行 arl_test_cleanup。";

            if (!string.IsNullOrWhiteSpace(optionalTargetName) || _audit == null || !_audit.Success)
            {
                string auditResult = Audit(optionalTargetName);
                if (_audit == null || !_audit.Success)
                    return auditResult + "\nARL build：骨骼审计未通过，未创建任何物理对象。";
            }

            if (!torsoOnly && _profile.SingleLegEnabled && (_legTarget == null || !_legTarget.Success))
            {
                _legTarget = SingleLegTargetResolver.Resolve(_audit.TargetRoot, _profile.SingleLegSide, _profile.SingleLegUpperBonePath, _profile.SingleLegLowerBonePath, _profile.SingleLegFootBonePath);
                if (!_legTarget.Success)
                    return "ARL build：primary leg目标解析失败，未创建物理对象。" + (_legTarget.Error ?? string.Empty);
            }
            if (torsoOnly)
            {
                // Resolve source references only: never invoke the preview leg builder,
                // whose native Hip joints can constrain a dynamic parent even with drive OFF.
                _legTarget = SingleLegTargetResolver.Resolve(_audit.TargetRoot, _profile.SingleLegSide,
                    _profile.SingleLegUpperBonePath, _profile.SingleLegLowerBonePath, _profile.SingleLegFootBonePath);
                if (_legTarget == null || !_legTarget.Success)
                    return "ARL pelvis setup：primary腿源骨解析失败，未创建物理对象。";
                _secondaryLegTarget = SingleLegTargetResolver.Resolve(_audit.TargetRoot, OppositeLegSide(_legTarget.RequestedSide), string.Empty, string.Empty, string.Empty);
                if (_secondaryLegTarget == null || !_secondaryLegTarget.Success)
                    return "ARL pelvis setup：secondary腿源骨解析失败，未创建物理对象。";
            }
            if (!torsoOnly && _profile.SingleLegEnabled && _profile.BilateralLegEnabled && (_secondaryLegTarget == null || !_secondaryLegTarget.Success))
            {
                _secondaryLegTarget = SingleLegTargetResolver.Resolve(_audit.TargetRoot, OppositeLegSide(_legTarget.RequestedSide), string.Empty, string.Empty, string.Empty);
                if (!_secondaryLegTarget.Success)
                    return "ARL build：secondary leg目标解析失败，未创建物理对象。" + (_secondaryLegTarget.Error ?? string.Empty);
            }
            if (!torsoOnly && _profile.SingleLegEnabled && _profile.FootAnkleFoundationEnabled)
            {
                if (_legTarget == null || !_legTarget.FootFoundationReady)
                    return "ARL build：primary Foot/Toe endpoint 未通过 v0.3.76 foundation 解析；未创建任何物理对象。 | toe=" + (_legTarget != null ? (_legTarget.ToeResolution ?? "<none>") : "<missing-target>");
                if (_profile.BilateralLegEnabled && (_secondaryLegTarget == null || !_secondaryLegTarget.FootFoundationReady))
                    return "ARL build：secondary Foot/Toe endpoint 未通过 v0.3.76 foundation 解析；未创建任何物理对象。 | toe=" + (_secondaryLegTarget != null ? (_secondaryLegTarget.ToeResolution ?? "<none>") : "<missing-target>");
            }

            _stepClock.InvalidateBinding();
            _proxy = new TwoBodyShadowProxy(_profile, _audit);
            string torsoResult = _proxy.Build();
            if (!_proxy.IsBuilt)
            {
                // Torso 与 bilateral/per-leg 报告统一由聚合写入器维护；失败路径也走同一入口。
                WriteAllReports();
                _logger?.LogWarning("[ARL][TWO_BODY] build-failed | " + torsoResult);
                RollbackInternal("torso-build-failed");
                return torsoResult;
            }
            _logger?.LogWarning("[ARL][TWO_BODY_FROZEN_BASELINE] " + torsoResult);

            string legResult = torsoOnly ? "scope=TORSO_PAIR_ONLY | legFootObjectsAndJoints=NOT_CREATED" : "singleLegEnabled=False";
            if (!torsoOnly && _profile.SingleLegEnabled)
            {
                _legChain = new BilateralLegCoordinator(
                    _profile,
                    _legTarget,
                    _secondaryLegTarget,
                    _audit.TargetRoot,
                    _proxy,
                    _profile.BilateralLegEnabled);
                legResult = _legChain.Build();
                if (!_legChain.IsBuilt)
                {
                    _logger?.LogError("[ARL][BILATERAL_LEG] build-failed | " + legResult);
                    RollbackInternal("bilateral-leg-build-failed");
                    return torsoResult + "\n" + legResult + "\nARL build：腿部foundation失败，已整体回滚。";
                }
                _logger?.LogWarning("[ARL][BILATERAL_LEG] " + legResult);
            }

            WriteAllReports();
            return torsoResult + "\n" + legResult;
        }

        public static string SetDrive(bool enabled)
        {
            if (AnkleTestActive) return enabled ? "ARL drive | action=NO_OP | owner=ANKLE_TEST" : StopAnkleTest("MANUAL_TORSO_DRIVE_OFF") + "\n" + SetDriveCore(false);
            return SetDriveCore(enabled);
        }
        private static string SetDriveCore(bool enabled)
        {
            if (enabled && _cleanupPending) return "ARL drive | enabled=False | reason=CLEANUP_PENDING";
            if (PelvisTestActive) return enabled ? "ARL drive | action=NO_OP | owner=PELVIS_TEST" : StopPelvisTest("MANUAL_TORSO_DRIVE_OFF");
            if (_proxy == null)
                return "ARL drive：代理尚未创建。";

            // Keep torso/leg trace lifecycle synchronized. Historically arl_drive(off) could abort only the torso trace
            // while bilateral leg trace remained active. Finalize the shared trace first, then shut down drives.
            if (!enabled && TraceActive)
            {
                string traceStop = SetTrace(false);
                _logger?.LogWarning("[ARL][TRACE_AUTO_STOP] reason=TORSO_DRIVE_OFF | " + traceStop);
            }
            if (!enabled && _legChain != null && _legChain.KneeAxisCalibrationCollecting)
                _legChain.SetKneeAxisCalibration(false);
            if (!enabled && _legChain != null && _legChain.AnyDriveEnabled)
                _legChain.SetDrive(false);

            bool wasEnabled = _proxy.DriveEnabled;
            string torso = _proxy.SetDrive(enabled);
            if (enabled && !wasEnabled && _proxy.DriveEnabled) _automaticStop.BeginDriveSession("arl_drive(off)");
            WriteAllReports();
            string result = torso;
            _logger?.LogWarning("[ARL][DRIVE] " + result);
            return result;
        }

        public static string SetLegDrive(bool enabled)
        {
            if (AnkleTestActive) return enabled ? "ARL leg drive | action=NO_OP | owner=ANKLE_TEST" : StopAnkleTest("MANUAL_LEG_DRIVE_OFF");
            return SetLegDriveCore(enabled);
        }
        private static string SetLegDriveCore(bool enabled)
        {
            if (enabled && _cleanupPending) return "ARL leg drive | enabled=False | reason=CLEANUP_PENDING";
            if (PelvisTestActive) return enabled ? "ARL leg drive | action=NO_OP | owner=PELVIS_TEST" : StopPelvisTest("MANUAL_LEG_DRIVE_OFF");
            if (_legChain == null)
                return "ARL leg drive：bilateral leg foundation尚未创建。";
            if (enabled && !_legChain.IsBuilt)
                return "ARL leg drive：bilateral leg foundation尚未完整创建。";
            if (enabled && (_proxy == null || !_proxy.IsBuilt || !_proxy.DriveEnabled))
                return "ARL leg drive：拒绝启动。请先执行 arl_drive(on)，让冻结 Pelvis Pose Bridge 成为活动父体。";
            if (!enabled && TraceActive)
            {
                string traceStop = SetTrace(false);
                _logger?.LogWarning("[ARL][TRACE_AUTO_STOP] reason=LEG_DRIVE_OFF | " + traceStop);
            }

            bool wasEnabled = _legChain.AnyDriveEnabled;
            string result = _legChain.SetDrive(enabled);
            if (enabled && !wasEnabled && _legChain.DriveEnabled && !AnkleTestActive) _automaticStop.BeginDriveSession("arl_ankle_test(stop)");
            WriteAllReports();
            _logger?.LogWarning("[ARL][LEG_DRIVE] " + result);
            return result;
        }

        public static string SetHipExperimentStack(bool enabled)
        {
            if (AnkleTestActive) return "ARL | action=REJECTED | owner=ANKLE_TEST | 请先arl_ankle_test(stop)。";
            if (enabled && _cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING | 请先清理，旧析构失败时需重启。";
            if (PelvisTestActive) return "ARL Hip stack：骨盆会话拥有当前配置，请先停止本轮测试。";
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL Hip experiment stack：bilateral leg foundation尚未创建。";
            string result = _legChain.SetHipExperimentStack(enabled);
            WriteAllReports();
            _logger?.LogWarning("[ARL][HIP_EXPERIMENT_STACK] " + result);
            return result;
        }

        public static string GetHipExperimentStackStatus()
        {
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL Hip experiment stack | built=False";
            string result = _legChain.GetHipExperimentStackStatus();
            WriteAllReports();
            _logger?.LogWarning("[ARL][HIP_EXPERIMENT_STACK_STATUS] " + result);
            return result;
        }

        public static string GetSingleLegFreezeStatus()
        {
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL v0.2.122 Single-Leg freeze readiness | built=False";
            string result = _legChain.GetSingleLegFreezeStatus();
            WriteAllReports();
            _logger?.LogWarning("[ARL][SINGLE_LEG_FREEZE_STATUS] " + result);
            return result;
        }

        public static string SetKneeAxisCalibration(bool enabled)
        {
            if (PelvisTestActive) return "ARL | action=REJECTED | owner=PELVIS_TEST | 请先停止本轮测试。";
            if (AnkleTestActive) return "ARL | action=REJECTED | owner=ANKLE_TEST | 请先arl_ankle_test(stop)。";
            if (enabled && _cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING | 请先清理，旧析构失败时需重启。";
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL knee axis calibration：bilateral leg foundation尚未创建。";
            string result = _legChain.SetKneeAxisCalibration(enabled);
            WriteAllReports();
            return result;
        }

        public static string GetKneeAxisCalibrationStatus()
        {
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL knee axis calibration | built=False";
            string result = _legChain.GetKneeAxisCalibrationStatus();
            _logger?.LogWarning("[ARL][KNEE_AXIS_CALIBRATION_STATUS] " + result);
            WriteAllReports();
            return result;
        }

        public static string ApplyCalibratedKneeAxis()
        {
            if (PelvisTestActive) return "ARL | action=REJECTED | owner=PELVIS_TEST | 请先停止本轮测试。";
            if (AnkleTestActive) return "ARL | action=REJECTED | owner=ANKLE_TEST | 请先arl_ankle_test(stop)。";
            if (_cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING";
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL knee axis apply：bilateral leg foundation尚未创建。";
            string result = _legChain.ApplyCalibratedKneeAxis();
            WriteAllReports();
            return result;
        }

        public static string ResetKneeAxisToInitial()
        {
            if (PelvisTestActive) return "ARL | action=REJECTED | owner=PELVIS_TEST | 请先停止本轮测试。";
            if (AnkleTestActive) return "ARL | action=REJECTED | owner=ANKLE_TEST | 请先arl_ankle_test(stop)。";
            if (_cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING";
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL knee axis reset：bilateral leg foundation尚未创建。";
            string result = _legChain.ResetKneeAxisToInitial();
            WriteAllReports();
            return result;
        }

        public static string GetLegStatus()
        {
            if (_legChain == null)
                return "ARL bilateral foundation | built=False";
            string result = _legChain.GetStatus();
            _logger?.LogWarning("[ARL][LEG_STATUS] " + result);
            return result;
        }

        public static string GetBilateralStatus()
        {
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL bilateral foundation | built=False";
            string result = _legChain.GetBilateralStatus();
            WriteAllReports();
            _logger?.LogWarning("[ARL][BILATERAL_STATUS] " + result);
            return result;
        }

        public static string GetKneeSafetyContextStatus()
        {
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL Knee hard-safety context | built=False";
            string result = _legChain.GetKneeSafetyContextStatus();
            WriteAllReports();
            _logger?.LogWarning("[ARL][KNEE_SAFETY_CONTEXT_STATUS] " + result);
            return result;
        }

        public static string GetFootSoleRequalificationStatus()
        {
            if (_legChain == null)
                return "ARL bilateral Foot sole requalification | built=False";
            string result = _legChain.GetFootSoleRequalificationStatus();
            _logger?.LogWarning("[ARL][FOOT_SOLE_REQUAL_STATUS] " + result);
            return result;
        }

        public static string GetFootAnkleStatus()
        {
            if (_legChain == null || !_legChain.IsBuilt)
                return "ARL bilateral foot/ankle foundation | built=False";
            string result = _legChain.GetFootAnkleStatus();
            WriteAllReports();
            _logger?.LogWarning("[ARL][FOOT_ANKLE_STATUS] " + result);
            return result + " | fullReport=" + FootFoundationReportPath;
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace('\r', ' ').Replace('\n', ' ');
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string Fmm(float meters) => (meters * 1000f).ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);

        public static string StartPelvisTest() => PelvisTestCoordinator.TestInstructions;
        public static string StartPelvisTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.TorsoPairOnly);
        public static string StartPelvisLegTest() => "ARL v0.4.119 无接触回归：完成arl_test_setup、arl_drive(on)、双侧Knee PCA采集/停止ready/apply、arl_hip_stack(on) READY后等待1秒，执行arl_pelvis_leg_test(a|b)。A保留采样年龄外推，B关闭；两者均无接触/重力/载荷/Hip反力。无载荷接触对照用arl_pelvis_contact_test；足部自重对照用arl_pelvis_foot_load_test，小腿自重回归用arl_pelvis_lower_load_test，膝部耦合回归用arl_pelvis_knee_test，Upper审计回归用arl_pelvis_upper_load_test，当前Hip平移耦合用arl_pelvis_hip_test。自动[STARTED]后计时，结束本入口stop、arl_test_cleanup、arl_test_status各一次。";
        public static string StartPelvisLegTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.DynamicLegFreeSpace);
        public static string StartPelvisContactTest() => "ARL v0.4.119 动态无载荷接触：arl_test_setup → arl_drive(on) → 双侧Knee PCA采集/停止确认ready/apply → arl_hip_stack(on)，双侧READY后静止等待1秒 → arl_pelvis_contact_test(b)。本入口固定B普通参考、厚静态Box、exact-STANCE、零摩擦、Speculative/20mm/position12，全部重力、自重、reciprocal、Knee反力与Hip反力OFF。看到[STARTED]且scope=DYNAMIC_PELVIS_LEGS_UNLOADED_CONTACT后，静止40秒→同房自然行走转身40秒→静止20秒。结束arl_pelvis_contact_test(stop)、arl_test_cleanup、arl_test_status各一次；自动停止或启动拒绝后只cleanup/status，不继续开drive。";
        public static string StartPelvisContactTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.DynamicLegUnloadedContact);
        public static string StartPelvisFootLoadTest() => "ARL v0.4.119 足部自重接触：arl_test_setup → arl_drive(on) → 双侧Knee PCA采集/停止确认ready/apply → arl_hip_stack(on)，双侧READY后静止等待1秒 → arl_pelvis_foot_load_test(b)。固定B普通参考、厚静态Box、exact-STANCE、零摩擦、Speculative/20mm/position12；仅Foot由既有模块在授权支撑物理步开启native gravity，沿用游戏Physics.gravity，Lower/Upper/Torso重力、上游自重传递、reciprocal、Knee/Hip反力OFF。看到[STARTED]且scope=DYNAMIC_PELVIS_LEGS_FOOT_SELF_WEIGHT_CONTACT后，静止40秒→同房自然行走转身40秒→静止20秒。结束arl_pelvis_foot_load_test(stop)、arl_test_cleanup、arl_test_status各一次；自动停止或启动拒绝后只cleanup/status，不继续开drive。无载荷接触对照仍用arl_pelvis_contact_test(b)。";
        public static string StartPelvisFootLoadTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.DynamicLegFootSelfWeightContact);
        public static string StartPelvisLowerLoadTest() => "ARL v0.4.119 小腿自重接触：arl_test_setup → arl_drive(on) → 双侧Knee PCA采集/停止确认ready/apply → arl_hip_stack(on)，双侧READY后静止等待1秒 → arl_pelvis_lower_load_test(b)。本范围保留86的Foot/Lower重力、自重法向力对、B参考及同拍只读收支审计，reciprocal OFF按实际零记账，限定同房水平Box；Upper/Torso重力、Foot-tether reciprocal、Knee/Hip反力、Lower-ground authority与支撑角阻尼仍OFF，V04171 A。看到[STARTED]、testArm=B_V04860_DYNAMIC_LOWER_SELF_WEIGHT_LEDGER_AUDIT且scope=DYNAMIC_PELVIS_LEGS_LOWER_SELF_WEIGHT_CONTACT后，静止40秒→同房自然行走转身40秒→静止20秒。结束arl_pelvis_lower_load_test(stop)、arl_test_cleanup、arl_test_status各一次；自动停止或启动拒绝后只cleanup/status，不继续开drive。Foot-only对照仍用arl_pelvis_foot_load_test(b)。";
        public static string StartPelvisLowerLoadTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.DynamicLegLowerSelfWeightContact);
        public static string StartPelvisHipTest() => "ARL v0.4.119 完整承重准入。本轮A/B各自重启，均采用118 B的连续Knee参考及受限规划前馈背景。A接触合格后直接完整承重；B等待实际及一步恒速预计脚底最低点均在支持面-3至+2mm、完整足印、踝间隙不超过5mm、源龄不超过100ms，连续2个共享物理步后同步接入Foot/Lower/Upper整链载荷。无超时放行；接触资格丢失后重新准入。等待不等于安全停机，载荷曝光不足不能判为改善。原接触入口、安全门及力/加速度上限不变；ENTRY_MISSED继续，自动停止则收尾保存日志。每轮arl_test_setup → arl_drive(on) → arl_knee_axis_calibrate(on)；走动采样，以arl_knee_axis_status确认双侧COLLECTING_QUALITY_READY_STOP_TO_FINALIZE，再arl_knee_axis_calibrate(off)，双侧ready=True后arl_knee_axis_apply → arl_hip_stack(on)。双侧READY后静止1秒，分别执行arl_pelvis_hip_test(a)或(b)，等[STARTED]后静止40秒→走动转身40秒含多次停启→静止20秒。正常结束arl_pelvis_hip_test(stop) → arl_test_cleanup → arl_test_status各一次；拒绝/自动停机则只cleanup和status并保存日志。";
        public static string StartPelvisHipTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.DynamicLegHipAnchorCouplingContact);
        public static string StartPelvisUpperLoadTest() => "ARL v0.4.119 大腿自重与只读Hip wrench审计：arl_test_setup → arl_drive(on) → 双侧Knee PCA采集/停止ready/apply → arl_hip_stack(on)，双侧READY后静止1秒 → arl_pelvis_upper_load_test(b)。[STARTED]且B_V04880_DYNAMIC_UPPER_SELF_WEIGHT后静止40秒→同房水平地面自然行走转身40秒→静止20秒。沿用88 Upper支撑重力和两级COM自重传递，本入口保留同拍Hip命令/作用点/功率审计；骨盆候选反力不执行，Hip反力、Torso重力、Foot-tether reciprocal仍OFF。正常结束本入口stop、arl_test_cleanup、arl_test_status各一次；拒绝/自动停止后只cleanup/status。";
        public static string StartPelvisUpperLoadTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.DynamicLegUpperSelfWeightContact);
        public static string StartPelvisKneeTest() => "ARL v0.4.119 膝部法向耦合：arl_test_setup → arl_drive(on) → 双侧Knee PCA采集/停止确认ready/apply → arl_hip_stack(on)，双侧READY后静止等待1秒 → arl_pelvis_knee_test(b)。保留86 Foot/Lower自重组合，仅新增既有V04430 B膝部约化质量法向PD和对应Upper反力；Upper/Torso重力、Foot-tether reciprocal、Hip反力、Lower-ground authority、支撑角阻尼及planar damping OFF，V04171 A，控制上限与硬门不变。看到[STARTED]、testArm=B_V04870_DYNAMIC_KNEE_NORMAL_COUPLING且scope=DYNAMIC_PELVIS_LEGS_KNEE_NORMAL_COUPLING_CONTACT后，静止40秒→同房水平地面自然行走转身40秒→静止20秒。结束arl_pelvis_knee_test(stop)、arl_test_cleanup、arl_test_status各一次；自动停止或启动拒绝后只cleanup/status，不继续开drive。小腿自重回归仍用arl_pelvis_lower_load_test(b)，不要手动切换重力或反力选择器。";
        public static string StartPelvisKneeTest(string requestedArm) => StartPelvisRun(requestedArm, PelvisRunScope.DynamicLegKneeNormalCouplingContact);
        private static string StartPelvisRun(string requestedArm, PelvisRunScope scope)
        {
            if (string.Equals(requestedArm, "stop", StringComparison.OrdinalIgnoreCase)) return StopPelvisTest();
            if (PelvisTestActive && _pelvisScope != scope) return "ARL pelvis | started=False | reason=OTHER_PELVIS_SCOPE_ACTIVE";
            if (AnkleTestActive) return "ARL pelvis | started=False | reason=ANKLE_TEST_ACTIVE";
            if (PelvisRunPort.ResolveRequestedArm(requestedArm, scope) == null)
                return "ARL pelvis | started=False | reason=INVALID_ARM | " + (PelvisRunScopes.HasHipCoupling(scope) ? StartPelvisHipTest() : PelvisRunScopes.HasUpperWeight(scope) ? StartPelvisUpperLoadTest() : PelvisRunScopes.HasKneeCoupling(scope) ? StartPelvisKneeTest() : scope == PelvisRunScope.DynamicLegLowerSelfWeightContact ? StartPelvisLowerLoadTest() : scope == PelvisRunScope.DynamicLegFootSelfWeightContact ? StartPelvisFootLoadTest() : scope == PelvisRunScope.DynamicLegUnloadedContact ? StartPelvisContactTest() : scope == PelvisRunScope.DynamicLegFreeSpace ? StartPelvisLegTest() : StartPelvisTest());
            _pelvisScope = scope; _lastTestScope = "PELVIS";
            return _pelvisTest.Start(requestedArm);
        }
        public static string StopPelvisTest() => StopPelvisTest("USER_STOP");
        private static string StopPelvisTest(string reason) => _pelvisTest.Stop(reason);

        private static bool RestorePelvisTestSafeState(StringBuilder evidence)
        {
            bool contactRestored = true;
            if (_legChain != null)
            {
                if (_legChain.TraceEnabled) TryAnkleCleanup("DEFENSIVE_LEG_TRACE_OFF", () => _legChain.SetTrace(false), evidence);
                if (!_legChain.NativeBodiesSafeOff) TryAnkleCleanup("DEFENSIVE_LEG_DRIVE_OFF", () => _legChain.SetDrive(false), evidence);
                if (_pelvisWithContact || _legChain.DynamicContactSelected) contactRestored = _legChain.RestoreDynamicContact(evidence);
                if (_legChain.KneeAxisCalibrationCollecting) TryAnkleCleanup("DEFENSIVE_PCA_OFF", () => _legChain.SetKneeAxisCalibration(false), evidence);
                TryAnkleCleanup("FOOT_REFERENCE_BASELINE_RESTORE", () => _legChain.RestoreFootRelativeReferenceBaseline(), evidence);
            }
            if (_proxy != null)
            {
                if (_proxy.TraceEnabled) TryAnkleCleanup("TORSO_TRACE_OFF", () => _proxy.SetTrace(false), evidence);
                TryAnkleCleanup("RECEIVER_QUIESCE", () => _proxy.QuiescePelvisReceiverPreserveTrace(), evidence);
                TryAnkleCleanup("RECEIVER_BASELINE_RESTORE", () => _proxy.SetPelvisReceiver(PelvisDynamicReceiverIsolation.IdleArm), evidence);
            }
            bool safe = contactRestored && _proxy != null && _proxy.PelvisReceiverSafeOff && (_legChain == null || (_legChain.NativeBodiesSafeOff &&
                !_legChain.TraceEnabled && _legChain.FreeSpaceReady && _legChain.FootRelativeReferenceMatches(false)));
            evidence.AppendLine("torsoSafeOffAndBaselineRestored=" + safe + " | legsNativeSafeOff=" + (_legChain == null || _legChain.NativeBodiesSafeOff));
            if (!safe) evidence.AppendLine("UNSAFE_RECEIVER_FULL_ROLLBACK | " + RollbackInternal("RECEIVER_RESTORE_NOT_SAFE"));
            return safe;
        }

        public static string GetPelvisTestStatus(bool detailed = true) => _pelvisTest.Status(detailed) + "\nruntimeStage=" + _runtimeStage +
            "/cleanupPending=" + _cleanupPending + "/clockScope=TORSO_LEG_FOOT_SHARED_CONTEXT" +
            (_pelvisWithLegs && _legChain != null ? "\n" + (_pelvisWithContact && PelvisTestActive ? _legChain.DynamicContactBoundarySummary() : _legChain.FreeSpaceBoundarySummary()) : "");

        public static string StartAnkleTest() => AnkleTestCoordinator.TestInstructions;
        public static string StartAnkleTest(string requestedArm)
        {
            if (!PelvisTestActive && !string.Equals(requestedArm, "stop", StringComparison.OrdinalIgnoreCase)) _lastTestScope = "ANKLE";
            return _ankleTest.Start(requestedArm);
        }
        public static string StopAnkleTest() => StopAnkleTest("USER_STOP");
        private static string StopAnkleTest(string reason) => _ankleTest.Stop(reason);
        internal static string GetAnkleTestStatus(bool detailed = true) => _ankleTest.Status(detailed);

        private static bool TryAnkleCleanup(string step, Func<string> action, StringBuilder evidence)
        {
            try { evidence.AppendLine(step + " | " + action()); return true; }
            catch (Exception e)
            {
                string failure = step + " | EXCEPTION=" + e.GetType().Name + ":" + e.Message;
                evidence.AppendLine(failure);
                _logger?.LogError("[ARL][ANKLE_CLEANUP_FAILED] " + failure);
                return false;
            }
        }

        // Notification failures never interrupt a safety owner's native shutdown or rollback.
        internal static void RecordAutomaticStop(string owner, string reason)
        {
            try
            {
                if (_automaticStop.Record(owner, reason, Clean(_sceneName) + "#" + _sceneHandle,
                    _pelvisTest.Session.OwnsScope ? _pelvisTest.Session.RunId : _ankleTest.Session.OwnsScope ? _ankleTest.Session.RunId : 0, _pelvisTest.Session.OwnsScope ? _pelvisTest.Session.Arm : _ankleTest.Session.OwnsScope ? _ankleTest.Session.Arm : "PREPARATION_OR_FREE_SPACE"))
                    _logger?.LogWarning("[ARL][AUTO_STOP_NOTICE] " + _automaticStop.Status());
            }
            catch { }
        }

        internal static void FlushAutomaticStopNotice()
        {
            if (_shutdown || !_automaticStop.Pending) return;
            try
            {
                if (_automaticStop.TryDeliver(Time.unscaledTime, ActiveRagdollConsolePatch.TryPrintGame))
                    _logger?.LogInfo("[ARL][AUTO_STOP_CONSOLE_ATTEMPT] " + _automaticStop.Status());
            }
            catch { }
        }

        internal static string GetAutomaticStopGameStatus() => _automaticStop.GameText();

        public static string GetMaintenanceStatusSnapshot(bool detailed = false)
        {
            // Ordinary status is read-only and bounded. Do not rebuild/rewrite megabytes
            // of evidence on each query; the explicit report command retains that path.
            if (detailed) WriteAllReports();
            if (PelvisStatusIsCurrentScope)
                return _automaticStop.Status() + "\n" + GetPelvisTestStatus(detailed);
            string live = _automaticStop.Status() + "\n[LIVE_STATE] built=" + IsBuilt + " | ankleTestActive=" + AnkleTestActive +
                          " | legDrive=" + (_legChain != null && _legChain.AnyDriveEnabled) + " | trace=" + TraceActive +
                          " | cleanupPending=" + _cleanupPending + " | runtimeStage=" + _runtimeStage + " | " + _ankleTest.Session.Status();
            if (!detailed)
                return live + "\n" + (_legChain != null ? _legChain.GetStepContextStatus() : "legStepContext=<not-built>") +
                    _ankleTest.Session.EvidenceSummary() + "\nfullEvidence=STOP_RECORD_AND_COMPONENT_REPORTS | command=arl_test_report";
            if (!AnkleTestActive && _lastRejectedAnkleSetup != null)
                live += "\n[LAST_REJECTED_SETUP]\n" + _lastRejectedAnkleSetup;
            if (!AnkleTestActive && _completedAnkleTestSnapshot != null)
                return live + "\n" + _completedAnkleTestSnapshot;
            string audit = AnkleTestActive ? "resultScope=ACTIVE_RUN | " + GetStageEFootCollisionSupportLoadContinuityAudit()
                                          : GetFreeSpaceFoundationReadinessAudit();
            return live + "\n" + BuildMaintenanceSnapshot(audit);
        }

        private static string BuildMaintenanceSnapshot(string audit, bool completed = false)
        {
            // One raw snapshot per component. Historical standalone qualifications live in the long-term
            // record; their observers, hard gates and strict verdict fields remain in the current reports.
            var sb = new StringBuilder(16384);
            sb.AppendLine("[1/6] ANKLE_TEST_STATE").AppendLine(completed
                ? GetAnkleTestStatus().Replace("stateScope=LIVE_STATE", "stateScope=BEFORE_SELECTOR_RESET")
                : GetAnkleTestStatus());
            sb.AppendLine("[2/6] CURRENT_SCOPE_AUDIT").AppendLine(audit ?? "Included once above in the completed run audit.");
            sb.AppendLine("[3/6] BILATERAL_AND_LEGS");
            sb.AppendLine(_legChain != null ? _legChain.GetBilateralStatus() : "<not-built>");
            sb.AppendLine(_legChain?.PrimaryReport?.GetConsoleSummary() ?? "<no-primary>");
            sb.AppendLine(_legChain?.SecondaryReport?.GetConsoleSummary() ?? "<no-secondary>");
            sb.AppendLine("[4/6] KNEE_SAFETY").AppendLine(_legChain != null ? _legChain.GetKneeSafetyContextStatus() : "<not-built>");
            sb.AppendLine("[5/6] FOOT_RAW_EVIDENCE");
            sb.AppendLine(_legChain?.PrimaryFootReport?.GetFullText() ?? "<no-primary>");
            sb.AppendLine(_legChain?.SecondaryFootReport?.GetFullText() ?? "<no-secondary>");
            sb.AppendLine("[6/6] TORSO").AppendLine(_proxy?.Report?.GetConsoleSummary() ?? "<not-built>");
            sb.AppendLine("Full component reports: " + Paths.ConfigPath);
            return sb.ToString();
        }

        public static string SetGravity(bool enabled)
        {
            if (PelvisTestActive) return "ARL | action=REJECTED | owner=PELVIS_TEST | 请先停止本轮测试。";
            if (AnkleTestActive) return "ARL | action=REJECTED | owner=ANKLE_TEST | 请先arl_ankle_test(stop)。";
            if (enabled && _cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING | 请先清理，旧析构失败时需重启。";
            if (_proxy == null)
                return "ARL gravity：代理尚未创建。";
            // The frozen bilateral production chain is certified only with leg Gravity=OFF.  The old console
            // surface allowed gravity to be switched on after arl_leg_drive(on), bypassing the start-time gate.
            // Keep the low-level switch for future feet/contact work, but fail closed while the active leg drive
            // depends on the frozen v0.2.122 contract.
            if (enabled && ((_proxy != null && _proxy.DriveEnabled) || (_legChain != null && _legChain.AnyDriveEnabled)))
                return "ARL gravity：拒绝开启。活动驱动的通用torso/leg gravity必须保持OFF；旧ankle承重仅由其内部exact-STANCE owner授权。请先停止当前测试或torso/leg drive。";
            string torso = _proxy.SetGravity(enabled);
            string leg = _legChain != null ? _legChain.SetGravity(enabled) : "leg=not-built";
            WriteAllReports();
            string result = torso + " | " + leg;
            _logger?.LogInfo("[ARL][GRAVITY] " + result);
            return result;
        }

        public static string SetColliders(bool enabled)
        {
            if (PelvisTestActive) return "ARL | action=REJECTED | owner=PELVIS_TEST | 请先停止本轮测试。";
            if (AnkleTestActive) return "ARL | action=REJECTED | owner=ANKLE_TEST | 请先arl_ankle_test(stop)。";
            if (enabled && _cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING | 请先清理，旧析构失败时需重启。";
            if (_proxy == null)
                return "ARL colliders：代理尚未创建。";
            // Same contract as Gravity above: leg colliders must not be enabled behind the running frozen stack.
            // Future full-body/GSP contact integration can establish a new explicit contact contract instead of
            // silently mutating the already-certified no-collider bilateral baseline.
            if (enabled && ((_proxy != null && _proxy.DriveEnabled) || (_legChain != null && _legChain.AnyDriveEnabled)))
                return "ARL colliders：拒绝开启。活动驱动的通用torso/leg collider必须保持OFF；Foot contact仅由独立接触会话的exact-STANCE owner授权。请先停止当前测试或torso/leg drive。";
            string contactShadow = "footContactShadow=not-built";
            if (!enabled && _legChain != null)
                contactShadow = _legChain.DisableFootContactShadowPreserveTrace();

            string torso = _proxy.SetColliders(enabled);
            string leg = _legChain != null ? _legChain.SetColliders(enabled) : "leg=not-built";
            WriteAllReports();
            string result = torso + " | " + leg + " | " + contactShadow;
            _logger?.LogWarning("[ARL][COLLIDERS] " + result);
            return result;
        }

        public static string SetVisual(bool enabled)
        {
            if (_proxy == null)
                return "ARL visual：代理尚未创建。";
            string torso = _proxy.SetVisual(enabled);
            string leg = _legChain != null ? _legChain.SetVisual(enabled) : "leg=not-built";
            WriteAllReports();
            return torso + " | " + leg;
        }

        public static string ResetPose()
        {
            if (_cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING";
            if (PelvisTestActive) StopPelvisTest("MANUAL_RESET_POSE");
            if (AnkleTestActive) StopAnkleTest("MANUAL_RESET_POSE");
            if (_proxy == null)
                return "ARL reset：代理尚未创建。";
            if (TraceActive)
            {
                string traceStop = SetTrace(false);
                _logger?.LogWarning("[ARL][TRACE_AUTO_STOP] reason=RESET_POSE | " + traceStop);
            }
            if (_legChain != null && _legChain.AnyDriveEnabled)
                _legChain.SetDrive(false);
            string torso = _proxy.ResetPose();
            string leg = _legChain != null ? _legChain.ResetPose() : "leg=not-built";
            WriteAllReports();
            string result = torso + " | " + leg;
            _logger?.LogWarning("[ARL][RESET] " + result);
            return result;
        }

        public static string SetTrace(bool enabled)
        {
            if (AnkleTestActive) return enabled ? "ARL trace | action=NO_OP | owner=ANKLE_TEST" : StopAnkleTest("MANUAL_TRACE_OFF");
            return SetTraceCore(enabled);
        }
        private static string SetTraceCore(bool enabled)
        {
            if (enabled && _cleanupPending) return "ARL | action=REJECTED | reason=CLEANUP_PENDING | 请先清理，旧析构失败时需重启。";
            if (PelvisTestActive) return enabled ? "ARL trace | action=NO_OP | owner=PELVIS_TEST | 请等待STARTED。" : StopPelvisTest("MANUAL_TRACE_OFF");
            if (_proxy == null)
                return "ARL trace：代理尚未创建。";

            bool torsoTrace = _proxy.TraceEnabled;
            // Starting needs a complete foundation; stopping must also reach a stale owner
            // whose native body disappeared while its retained trace is still active.
            bool legPresent = _legChain != null;
            bool legRequired = legPresent && _legChain.IsBuilt;
            bool legAnyTrace = legPresent && _legChain.TraceEnabled;
            bool legFullyTrace = legRequired && _legChain.TraceFullyEnabled;
            bool fullyActive = torsoTrace && (!legPresent || legFullyTrace);
            bool fullyInactive = !torsoTrace && !legAnyTrace;

            if (enabled && fullyActive)
                return "ARL trace | enabled=True | alreadyActive=True | action=NO_OP";
            if (!enabled && fullyInactive)
                return "ARL trace | enabled=False | alreadyInactive=True | action=NO_OP";

            // A half-open torso/leg trace is never a valid steady state. It can be left behind by an exception or by
            // an old build. Normalize it before either a restart or an explicit stop so stale statistics cannot leak
            // into the next certification window. Stop only sides that are actually active; TwoBody.SetTrace(false)
            // is intentionally not used as an idempotent no-op because it finalizes its current statistics.
            string repaired = null;
            if (legPresent && (!fullyActive && (torsoTrace || legAnyTrace)))
            {
                repaired = "inconsistentTraceRepaired=True | " + StopTraceOwners(torsoTrace, legAnyTrace, "RUNTIME_TRACE_REPAIR_FAILED");
                _logger?.LogWarning("[ARL][TRACE_STATE_REPAIR] " + repaired);
                torsoTrace = false;
                legAnyTrace = false;
                legFullyTrace = false;
                if (!enabled)
                {
                    WriteAllReports();
                    return "ARL trace | enabled=False | " + repaired;
                }
            }

            if (enabled && legPresent && !legRequired)
                return "ARL trace：拒绝启动。保留的腿部 foundation 已失效，请先 arl_test_cleanup。";
            if (enabled && !_proxy.DriveEnabled)
                return "ARL trace：拒绝启动。请先 arl_drive(on)。";
            if (enabled && legRequired && !_legChain.DriveEnabled)
                return "ARL trace：拒绝启动。bilateral leg foundation 已创建时必须先 arl_leg_drive(on)，避免 torso/leg trace 半开状态。";
            if (enabled && legRequired && _legChain.KneeAxisCalibrationCollecting)
                return "ARL trace：拒绝启动。Knee axis calibration 正在收集，请先 arl_knee_axis_calibrate(off)。";

            if (enabled)
            {
                bool torsoStartAttempted = false, legStartAttempted = false;
                try
                {
                    // An owner can acquire its live latch before report projection/logging
                    // throws. Track attempts as well as published report state for rollback.
                    torsoStartAttempted = true;
                    string torso = _proxy.SetTrace(true);
                    if (!_proxy.TraceEnabled)
                    {
                        torsoStartAttempted = false;
                        WriteAllReports();
                        string fail = "ARL trace：torso trace 启动失败；未启动 bilateral leg trace。 | " + torso;
                        _logger?.LogWarning("[ARL][TRACE_CONTROL] " + fail);
                        return fail;
                    }

                    legStartAttempted = legRequired;
                    string leg = legRequired ? _legChain.SetTrace(true) : "leg=not-built";
                    if (legRequired && !_legChain.TraceFullyEnabled)
                    {
                        string rollback = StopTraceOwners(true, true, "RUNTIME_TRACE_REJECTED_START_CLEANUP_FAILED");
                        torsoStartAttempted = legStartAttempted = false;
                        WriteAllReports();
                        string fail = "ARL trace：bilateral leg trace 启动失败；已关闭本次 trace。 | leg=" + leg + " | rollback=" + rollback;
                        _logger?.LogWarning("[ARL][TRACE_CONTROL] " + fail);
                        return fail;
                    }

                    WriteAllReports();
                    string result = torso + " | " + leg + (repaired != null ? " | " + repaired : string.Empty);
                    _logger?.LogInfo("[ARL][TRACE_CONTROL] " + result);
                    return result;
                }
                catch (Exception startFailure)
                {
                    try { StopTraceOwners(torsoStartAttempted, legStartAttempted, "RUNTIME_TRACE_START_CLEANUP_FAILED"); }
                    catch (Exception cleanupFailure)
                    {
                        throw new AggregateException("RUNTIME_TRACE_START_AND_CLEANUP_FAILED", startFailure, cleanupFailure);
                    }
                    throw new InvalidOperationException("RUNTIME_TRACE_START_FAILED", startFailure);
                }
            }

            // Finalize the torso first because per-leg freeze certification consumes the finalized TwoBody freeze verdict.
            // The old leg-first order could leave the immediately emitted bilateral snapshot stale until a later
            // arl_leg_freeze_status call refreshed the child certifications. No physics step occurs between these calls.
            string stopped = StopTraceOwners(torsoTrace, legAnyTrace, "RUNTIME_TRACE_STOP_FAILED") +
                (repaired != null ? " | " + repaired : string.Empty);
            WriteAllReports();
            _logger?.LogInfo("[ARL][TRACE_CONTROL] " + stopped);
            return stopped;
        }

        private static string StopTraceOwners(bool stopTorso, bool stopLeg, string failureReason)
        {
            string torso = "torso=already-off", leg = "leg=already-off";
            Exception torsoFailure = null, legFailure = null;
            // Torso finalization must precede leg freeze certification. A failed first
            // owner must still allow the second to close its trace and retain evidence.
            try { if (stopTorso && _proxy != null) torso = _proxy.SetTrace(false); }
            catch (Exception e) { torsoFailure = e; }
            try { if (stopLeg && _legChain != null) leg = _legChain.SetTrace(false); }
            catch (Exception e) { legFailure = e; }
            if (torsoFailure != null && legFailure != null)
                throw new AggregateException(failureReason, torsoFailure, legFailure);
            if (torsoFailure != null || legFailure != null)
                throw new InvalidOperationException(failureReason, torsoFailure ?? legFailure);
            return torso + " | " + leg;
        }

        public static string Rollback(string reason)
        {
            return RollbackInternal(reason);
        }

        public static void NotifyProxyCollision(string bodyLabel, string phase, Collision collision)
        {
            if (_shutdown || _cleanupPending)
                return;
            try
            {
                if (_proxy == null || !_proxy.IsBuilt)
                    return;
                _proxy.NotifyCollision(bodyLabel, phase, collision);
                if (phase == "ENTER")
                {
                    string other = _proxy.Report.LastContactOther ?? "<unknown>";
                    _logger?.LogInfo(
                        "[ARL][CONTACT] phase=ENTER | body=" + bodyLabel +
                        " | other=" + other +
                        " | enterCount=" + _proxy.Report.ContactEnterCount);
                }
            }
            catch (Exception e)
            {
                _logger?.LogWarning("[ARL][CONTACT] callback-failed | " + e.GetType().Name + " / " + e.Message);
            }
        }

        public static void NotifyFootContactShadow(string side, string phase, Collision collision)
        {
            if (_shutdown || _cleanupPending)
                return;
            try
            {
                if (_legChain == null || !_legChain.IsBuilt)
                    return;
                bool ok = _legChain.NotifyFootContactShadowCollision(side, phase, collision);
                if (!ok)
                    _logger?.LogError("[ARL][FOOT_CONTACT_SHADOW_CONTRACT_INVALID] side=" + side + " | phase=" + phase + " | action=COLLIDER_FAIL_CLOSED_NEXT_FIXED_ABORT | class=SEMANTIC_CONTACT_CONTRACT_NOT_NUMERIC_NONFINITE");
            }
            catch (Exception e)
            {
                _logger?.LogError("[ARL][FOOT_CONTACT_SHADOW_CALLBACK_EXCEPTION] side=" + side + " | phase=" + phase + " | " + e.GetType().Name + " / " + e.Message);
            }
        }

        public static void Shutdown(string reason)
        {
            // Pending cleanup is a reason to retry teardown, never a reason to skip it.
            // Both plugin unload and application quit may arrive after an earlier rollback failed.
            if (_shutdown && !_cleanupPending)
                return;
            _shutdown = true;
            _cleanupPending = true;
            CleanupReceipt cleanup = RollbackWithReceipt("shutdown:" + reason);
            _logger?.LogInfo("[ARL] Shutdown | cleanupSucceeded=" + cleanup.Succeeded + " | reason=" + reason);
        }

        private static string RollbackInternal(string reason)
            => RollbackWithReceipt(reason).Summary;

        internal static CleanupReceipt RollbackWithReceipt(string reason)
        {
            if (PelvisTestActive)
            {
                try { StopPelvisTest("ROLLBACK:" + reason); }
                catch (Exception e)
                {
                    // A failed evidence logger must not prevent native owner cleanup.
                    try { _logger?.LogError("[ARL][PELVIS_SNAPSHOT_BEFORE_ROLLBACK_FAILED] " + e.GetType().Name); } catch { }
                }

            }
            if (AnkleTestActive)
            {
                try { StopAnkleTest("ROLLBACK:" + reason); }
                catch (Exception e)
                {
                    try { _logger?.LogError("[ARL][ANKLE_SNAPSHOT_BEFORE_ROLLBACK_FAILED] " + e.GetType().Name); } catch { }
                }
            }
            _stepClock.InvalidateBinding();
            if (_proxy == null && _legChain == null)
            {
                _cleanupPending = false; _pelvisTest?.Session.NoteCleanup(false); _ankleTest?.Session.NoteCleanup(false);
                LastRollbackSucceeded = true;
                return new CleanupReceipt(false, true, 0, 0, 0, 0, 0,
                    "ARL foundation rollback | success=True | cleared=False | reason=" + reason + " | detail=no-active-object");
            }

            bool success = true;
            string legResult = null;
            if (_legChain != null)
            {
                CleanupReceipt legCleanup = default;
                try
                {
                    legCleanup = _legChain.RollbackWithReceipt(reason);
                    legResult = legCleanup.Summary;
                    if (!legCleanup.Succeeded)
                        success = false;
                }
                catch (Exception e)
                {
                    success = false;
                    legResult = "ARL bilateral-leg rollback failed：" + e.GetType().Name + " / " + e.Message;
                }
                if (legCleanup.Succeeded && success) _legChain = null;
            }

            string torsoResult = "torso=not-built";
            if (_proxy != null && _legChain != null)
            {
                success = false;
                try { _proxy.SetDrive(false); } catch { }
                torsoResult = "torso=retained-until-leg-cleanup";
            }
            else if (_proxy != null)
            {
                CleanupReceipt torsoCleanup = default;
                try
                {
                    torsoCleanup = _proxy.RollbackWithReceipt(reason);
                    torsoResult = torsoCleanup.Summary;
                    if (!torsoCleanup.Succeeded)
                        success = false;
                }
                catch (Exception e)
                {
                    success = false;
                    torsoResult = "ARL rollback failed：" + e.GetType().Name + " / " + e.Message;
                }
                if (torsoCleanup.Succeeded && success) _proxy = null;
            }
            _legTarget = null;
            _secondaryLegTarget = null;
            _cleanupPending = !success;
            _pelvisTest?.Session.NoteCleanup(_cleanupPending); _ankleTest?.Session.NoteCleanup(_cleanupPending);
            LastRollbackSucceeded = success;
            string result = "ARL foundation rollback | success=" + LastRollbackSucceeded + " | reason=" + reason + " | " + torsoResult +
                            (legResult != null ? " | " + legResult : string.Empty);
            if (LastRollbackSucceeded)
                _logger?.LogInfo("[ARL] " + result);
            else
                _logger?.LogError("[ARL] " + result);
            return new CleanupReceipt(true, success, null, (_legChain == null ? 0 : 1) + (_proxy == null ? 0 : 1),
                null, 0, 0, result);
        }

        private static void WriteAllReports()
        {
            if (_proxy != null)
                WriteTextFile(ProxyReportPath, _proxy.Report.GetFullText() + (_completedPelvisTestSnapshot != null ? "\n\n" + _completedPelvisTestSnapshot : string.Empty));
            if (_legChain != null)
            {
                if (_legChain.PrimaryReport != null)
                    WriteTextFile(SingleLegReportPath, _legChain.PrimaryReport.GetFullText());
                if (_legChain.BilateralEnabled && _legChain.SecondaryReport != null)
                    WriteTextFile(SecondaryLegReportPath, _legChain.SecondaryReport.GetFullText());
                if (_legChain.BilateralEnabled && _legChain.BilateralReport != null)
                    WriteTextFile(BilateralLegReportPath, _legChain.BilateralReport.GetFullText() +
                        (_completedAnkleTestSnapshot != null ? "\n\n" + _completedAnkleTestSnapshot : string.Empty));
                if (_legChain.KneeSafetyReport != null)
                    WriteTextFile(KneeSafetyContextReportPath, _legChain.KneeSafetyReport.GetFullText());
                if (_legChain.PrimaryFootReport != null || _legChain.SecondaryFootReport != null)
                    WriteTextFile(FootFoundationReportPath, _legChain.GetFootAnkleFullReport());
            }
        }

        private static string OppositeLegSide(string side)
        {
            return string.Equals(side, "RIGHT", StringComparison.OrdinalIgnoreCase) ? "Left" : "Right";
        }

        private static void AppendLegTargetAuditLine(BoneAuditReport audit, string role, SingleLegTargetReport target)
        {
            if (audit == null || target == null) return;
            audit.Lines.Add(
                "[ARL Leg Target] role=" + role +
                " | success=" + target.Success +
                " | side=" + target.RequestedSide +
                " | mode=" + target.ResolutionMode +
                " | candidateRoot=" + (target.CandidateRootPath ?? "<none>") +
                " | chainValidation=" + (target.ChainValidation ?? "<none>") +
                " | upper=" + (target.UpperLegPath ?? "<missing>") +
                " | lower=" + (target.LowerLegPath ?? "<missing>") +
                " | foot=" + (target.FootPath ?? "<missing>") +
                " | toe=" + (target.ToePath ?? "<missing>") +
                " | footReady=" + target.FootFoundationReady +
                " | footLength=" + target.FootLength +
                " | toeResolve=" + (target.ToeResolution ?? "<none>") +
                " | detail=" + (target.ResolutionDetail ?? "<none>") +
                " | error=" + (target.Error ?? "<none>"));
        }

        private static void WriteTextFile(string path, string content)
        {
            try
            {
                Directory.CreateDirectory(Path.GetDirectoryName(path));
                File.WriteAllText(path, content ?? string.Empty);
            }
            catch (Exception e)
            {
                _logger?.LogWarning("[ARL] report-write-failed | path=" + path + " | " + e.Message);
            }
        }
    }
}
