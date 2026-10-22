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
            (_proxy != null && _proxy.Report != null && _proxy.Report.TraceEnabled) ||
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
        private static bool _pelvisWithLegs;
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
            _lastTestScope = "PELVIS"; _pelvisWithLegs = false;
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
                return; // One session owns either the torso-only baseline or the staged free-space leg join.
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
            _logger?.LogError("[ARL][RUNTIME_FAIL_CLOSED] stage=" + stage + " | " + type + " / " + message);
            string rollback = RollbackInternal("runtime-fail-closed:" + stage + ":" + type);
            _logger?.LogError("[ARL][RUNTIME_FAIL_CLOSED_ROLLBACK] stage=" + stage + " | " + rollback);
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
                RollbackInternal("new-audit");
                if (!LastRollbackSucceeded) return "ARL audit：清理尚未完成，请先arl_test_cleanup；保留旧对象以便重试。";
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

        public static string GetStageEFootCollisionSupportLoadContinuityAudit()
        {
            if (!AnkleTestActive && _completedAnkleTestAudit != null)
                return _completedAnkleTestAudit;
            return BuildFootCollisionSupportLoadContinuityAudit();
        }

        private static string BuildFootCollisionSupportLoadContinuityAudit(bool completed = false)
        {
            const string mode = UpperSelfWeightSupportNormalLoadTransferIsolation.Mode;
            bool primaryIsLeft = _legChain != null && string.Equals(_legChain.PrimarySide, "LEFT", StringComparison.OrdinalIgnoreCase);
            FootAnkleFoundationReport leftFoot = _legChain != null ? (primaryIsLeft ? _legChain.PrimaryFootReport : _legChain.SecondaryFootReport) : null;
            FootAnkleFoundationReport rightFoot = _legChain != null ? (primaryIsLeft ? _legChain.SecondaryFootReport : _legChain.PrimaryFootReport) : null;
            SingleLegChainReport leftLeg = _legChain != null ? (primaryIsLeft ? _legChain.PrimaryReport : _legChain.SecondaryReport) : null;
            SingleLegChainReport rightLeg = _legChain != null ? (primaryIsLeft ? _legChain.SecondaryReport : _legChain.PrimaryReport) : null;
            BilateralLegReport bilateral = _legChain != null ? _legChain.BilateralReport : null;
            TwoBodyProxyReport torso = _proxy != null ? _proxy.Report : null;
            bool built = _legChain != null && _legChain.IsBuilt && leftFoot != null && rightFoot != null && leftLeg != null && rightLeg != null;
            bool testArmKnown = UpperSelfWeightSupportNormalLoadTransferIsolation.IsExperimentArm(_ankleTestArm);
            string expectedCollisionArm = FootCollisionDetectionModeIsolation.ActiveArm;
            bool contactOffsetContract = testArmKnown && built &&
                FootContactOffsetSelectorContract(leftFoot, FootContactOffsetIsolation.ArmB) && FootContactOffsetSelectorContract(rightFoot, FootContactOffsetIsolation.ArmB) &&
                leftFoot.FootContactOffsetReadbackFailures == 0 && rightFoot.FootContactOffsetReadbackFailures == 0;
            bool contactOffsetExposure = contactOffsetContract &&
                FootContactOffsetObservedExposure(leftFoot) && FootContactOffsetObservedExposure(rightFoot);

            FootSupportSurfaceIsolation surface = _legChain != null ? _legChain.SupportSurface : null;
            bool surfaceContract = testArmKnown && built && surface != null && surface.Active && surface.Finite &&
                string.Equals(surface.Arm, FootSupportSurfaceIsolation.ArmB, StringComparison.Ordinal);
            bool surfaceExposure = surfaceContract && surface.ExposureReady;
            bool kneeReactionContract = testArmKnown && built && _legChain.KneeSupportReactionArmsMatch &&
                KneeSupportNormalReactionIsolation.SelectorContract(leftLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB) &&
                KneeSupportNormalReactionIsolation.SelectorContract(rightLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB);
            bool kneeReactionExposure = kneeReactionContract &&
                KneeSupportNormalReactionIsolation.ExposureReady(leftLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB) &&
                KneeSupportNormalReactionIsolation.ExposureReady(rightLeg.KneeSupportReaction, KneeSupportNormalReactionIsolation.ArmB);
            bool upperLoadContract = testArmKnown && built && _legChain.UpperSelfWeightArmsMatch &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.SelectorContract(leftLeg.UpperSelfWeightSupport, _ankleTestArm, completed) &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.SelectorContract(rightLeg.UpperSelfWeightSupport, _ankleTestArm, completed);
            bool upperLoadExposure = upperLoadContract &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.ExposureReady(leftLeg.UpperSelfWeightSupport, _ankleTestArm) &&
                UpperSelfWeightSupportNormalLoadTransferIsolation.ExposureReady(rightLeg.UpperSelfWeightSupport, _ankleTestArm);


            bool frozenWorldGravityBoundary = built && torso != null && !torso.GravityEnabled && !leftLeg.GravityEnabled && !rightLeg.GravityEnabled;
            bool zeroFrictionBaseline = built && leftFoot.StanceContactFrictionIsolationZeroFriction && rightFoot.StanceContactFrictionIsolationZeroFriction &&
                                        leftFoot.StanceContactFrictionIsolationFinite && rightFoot.StanceContactFrictionIsolationFinite;
            bool frozenTetherTopology = built && leftFoot.AnkleConstraintFinite && rightFoot.AnkleConstraintFinite &&
                                string.Equals(leftFoot.AnkleConstraintArm, FootAnkleConstraintTopologyIsolation.ArmB, StringComparison.Ordinal) &&
                                string.Equals(rightFoot.AnkleConstraintArm, FootAnkleConstraintTopologyIsolation.ArmB, StringComparison.Ordinal);
            bool lowerGroundFrozen = built && leftFoot.StanceLowerGroundAuthorityFinite && rightFoot.StanceLowerGroundAuthorityFinite &&
                                     string.Equals(leftFoot.StanceLowerGroundAuthorityArm, FootStanceLowerGroundAnchorAuthority.ArmB, StringComparison.Ordinal) &&
                                     string.Equals(rightFoot.StanceLowerGroundAuthorityArm, FootStanceLowerGroundAnchorAuthority.ArmB, StringComparison.Ordinal) &&
                                     leftFoot.StanceLowerGroundAuthorityEnabled && rightFoot.StanceLowerGroundAuthorityEnabled;
            bool predictiveTailFrozen = built && leftFoot.TranslationPredictiveTailCaptureFinite && rightFoot.TranslationPredictiveTailCaptureFinite &&
                                        string.Equals(leftFoot.TranslationPredictiveTailCaptureArm, FootAnkleTranslationPredictiveTailCaptureIsolation.ArmB, StringComparison.Ordinal) &&
                                        string.Equals(rightFoot.TranslationPredictiveTailCaptureArm, FootAnkleTranslationPredictiveTailCaptureIsolation.ArmB, StringComparison.Ordinal);
            bool footGravityFrozenOn = built && leftFoot.StanceFootNativeGravityFinite && rightFoot.StanceFootNativeGravityFinite &&
                                       string.Equals(leftFoot.StanceFootNativeGravityArm, FootStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                       string.Equals(rightFoot.StanceFootNativeGravityArm, FootStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                       leftFoot.StanceFootNativeGravityEnabled && rightFoot.StanceFootNativeGravityEnabled;
            bool lowerGravityFrozenOn = built && leftLeg.StanceLowerNativeGravityFinite && rightLeg.StanceLowerNativeGravityFinite &&
                                        string.Equals(leftLeg.StanceLowerNativeGravityArm, LowerStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                        string.Equals(rightLeg.StanceLowerNativeGravityArm, LowerStanceNativeGravitySupportLoadIsolation.ArmB, StringComparison.Ordinal) &&
                                        leftLeg.StanceLowerNativeGravityEnabled && rightLeg.StanceLowerNativeGravityEnabled;
            bool reciprocalFrozenOn = built && leftLeg.StanceFootLowerReciprocalFinite && rightLeg.StanceFootLowerReciprocalFinite &&
                                      string.Equals(leftLeg.StanceFootLowerReciprocalArm, FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                      string.Equals(rightLeg.StanceFootLowerReciprocalArm, FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                      leftLeg.StanceFootLowerReciprocalEnabled && rightLeg.StanceFootLowerReciprocalEnabled;
            bool transferFrozenOn = built && leftLeg.StanceLowerSelfWeightTransferFinite && rightLeg.StanceLowerSelfWeightTransferFinite &&
                                    string.Equals(leftLeg.StanceLowerSelfWeightTransferArm, LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                    string.Equals(rightLeg.StanceLowerSelfWeightTransferArm, LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB, StringComparison.Ordinal) &&
                                    leftLeg.StanceLowerSelfWeightTransferEnabled && rightLeg.StanceLowerSelfWeightTransferEnabled;
            bool collisionSelectorContract = testArmKnown && built && _legChain.FootCollisionDetectionModeFinite &&
                                             _legChain.FootCollisionDetectionModeArmsMatch &&
                                             string.Equals(_legChain.FootCollisionDetectionModeArm, expectedCollisionArm, StringComparison.Ordinal) &&
                                             FootCcdSelectorContract(leftFoot, expectedCollisionArm) &&
                                             FootCcdSelectorContract(rightFoot, expectedCollisionArm);
            bool collisionObservedExposure = collisionSelectorContract &&
                leftFoot.SupportEntityBoundaryCcdResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.SupportEntityBoundaryCcdResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                leftFoot.SupportEntityBoundaryCcdMismatchSolves == 0 && rightFoot.SupportEntityBoundaryCcdMismatchSolves == 0;
            bool supportAngularContract = testArmKnown && built && _legChain.StanceSupportAngularFinite &&
                _legChain.StanceSupportAngularArmsMatch && string.Equals(_legChain.StanceSupportAngularArm, FootStanceSupportNormalAngularStabilization.ArmB, StringComparison.Ordinal) &&
                leftFoot.StanceSupportAngularFinite && rightFoot.StanceSupportAngularFinite &&
                string.Equals(leftFoot.StanceSupportAngularArm, FootStanceSupportNormalAngularStabilization.ArmB, StringComparison.Ordinal) &&
                string.Equals(rightFoot.StanceSupportAngularArm, FootStanceSupportNormalAngularStabilization.ArmB, StringComparison.Ordinal) &&
                leftFoot.StanceSupportAngularEnabled && rightFoot.StanceSupportAngularEnabled &&
                leftFoot.StanceSupportAngularMaxNormalTorqueNm <= 0.00001f && rightFoot.StanceSupportAngularMaxNormalTorqueNm <= 0.00001f &&
                leftFoot.StanceSupportAngularMaxPowerWatts <= 0.00001f && rightFoot.StanceSupportAngularMaxPowerWatts <= 0.00001f;
            bool supportAngularExposure = built && leftFoot.StanceSupportAngularEligibleSamples >= FootStanceSupportNormalAngularStabilization.MinimumEligibleSamples &&
                rightFoot.StanceSupportAngularEligibleSamples >= FootStanceSupportNormalAngularStabilization.MinimumEligibleSamples &&
                (leftFoot.StanceSupportAngularAppliedSamples > 0 && rightFoot.StanceSupportAngularAppliedSamples > 0);
            bool velocitySolverContract = testArmKnown && built && _legChain.FootVelocitySolverFinite && _legChain.FootVelocitySolverArmsMatch &&
                string.Equals(_legChain.FootVelocitySolverArm, FootSolverIterationBaselineContract.ArmA, StringComparison.Ordinal) &&
                FootVelocitySolverContract(leftFoot) && FootVelocitySolverContract(rightFoot);
            bool velocitySolverExposure = built &&
                leftFoot.FootVelocitySolverEligibleSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.FootVelocitySolverEligibleSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves;
            bool positionSolverContract = testArmKnown && built && _legChain.FootPositionSolverArmsMatch &&
                string.Equals(_legChain.FootPositionSolverArm, FootSolverIterationBaselineContract.PositionArmB, StringComparison.Ordinal) &&
                FootPositionSolverContract(leftFoot) && FootPositionSolverContract(rightFoot);
            bool positionSolverExposure = built &&
                leftFoot.FootPositionSolverInterventionSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.FootPositionSolverInterventionSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                leftFoot.SupportEntityBoundaryPositionSolverResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves &&
                rightFoot.SupportEntityBoundaryPositionSolverResolvedSolves >= FootSolverIterationBaselineContract.MinimumEligibleSolves;
            bool nativeGravityEnvironment = built && leftLeg.StanceLowerNativeGravityGravityMagnitude > 0.0001f && rightLeg.StanceLowerNativeGravityGravityMagnitude > 0.0001f &&
                                            leftLeg.StanceLowerNativeGravityLowerMass > 0f && rightLeg.StanceLowerNativeGravityLowerMass > 0f &&
                                            leftLeg.StanceLowerNativeGravityExpectedNormalImpulseMean > 0f && rightLeg.StanceLowerNativeGravityExpectedNormalImpulseMean > 0f &&
                                            leftFoot.StanceFootNativeGravityExpectedNormalImpulseMean > 0f && rightFoot.StanceFootNativeGravityExpectedNormalImpulseMean > 0f;
            bool bilateralExposure = built && leftLeg.StanceLowerSelfWeightTransferEligibleSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinEligibleSamples &&
                                     rightLeg.StanceLowerSelfWeightTransferEligibleSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinEligibleSamples &&
                                     leftLeg.StanceLowerSelfWeightTransferAppliedSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinAppliedSamples &&
                                     rightLeg.StanceLowerSelfWeightTransferAppliedSamples >= LowerSelfWeightSupportNormalLoadTransferIsolation.MinAppliedSamples &&
                                     leftLeg.StanceLowerNativeGravityAppliedSamples >= LowerStanceNativeGravitySupportLoadIsolation.MinAppliedSamples &&
                                     rightLeg.StanceLowerNativeGravityAppliedSamples >= LowerStanceNativeGravitySupportLoadIsolation.MinAppliedSamples &&
                                     leftLeg.StanceFootLowerReciprocalAppliedSamples >= FootLowerSupportNormalReciprocalLoadTransferIsolation.MinAppliedSamples &&
                                     rightLeg.StanceFootLowerReciprocalAppliedSamples >= FootLowerSupportNormalReciprocalLoadTransferIsolation.MinAppliedSamples;
            bool noTransferLeak = built && leftLeg.StanceLowerSelfWeightTransferNonStanceAppliedSamples == 0 && rightLeg.StanceLowerSelfWeightTransferNonStanceAppliedSamples == 0;
            bool noReciprocalLeak = built && leftLeg.StanceFootLowerReciprocalNonStanceAppliedSamples == 0 && rightLeg.StanceFootLowerReciprocalNonStanceAppliedSamples == 0;
            bool noLowerGravityLeak = built && leftLeg.StanceLowerNativeGravityNonStanceAppliedSamples == 0 && rightLeg.StanceLowerNativeGravityNonStanceAppliedSamples == 0 &&
                                      leftLeg.StanceLowerNativeGravityReadbackFailures == 0 && rightLeg.StanceLowerNativeGravityReadbackFailures == 0 &&
                                      LowerGravityCurrentStateValid(leftLeg, leftFoot, completed) && LowerGravityCurrentStateValid(rightLeg, rightFoot, completed);
            bool lifecycleEvidence = built && leftFoot.StanceContactLifecycleTraceQualified && rightFoot.StanceContactLifecycleTraceQualified &&
                                     leftFoot.StanceContactLifecycleNonStanceActiveSamples == 0 && rightFoot.StanceContactLifecycleNonStanceActiveSamples == 0;
            bool supportLoadReactionDataset = built && SupportLoadReactionDatasetUsable(leftFoot) && SupportLoadReactionDatasetUsable(rightFoot);
            bool angularImpulseObserverFinite = built && leftFoot.FootAngularImpulseAttributionFinite && rightFoot.FootAngularImpulseAttributionFinite;
            bool angularImpulseEvidenceReady = angularImpulseObserverFinite && leftFoot.FootAngularImpulseAttributionEvidenceReady && rightFoot.FootAngularImpulseAttributionEvidenceReady;
            bool couplingObserverFinite = built && leftFoot.RockingTetherCouplingFinite && rightFoot.RockingTetherCouplingFinite;
            bool couplingEvidenceReady = couplingObserverFinite && leftFoot.RockingTetherCouplingEvidenceReady && rightFoot.RockingTetherCouplingEvidenceReady;
            bool supportEntityFinite = built && leftFoot.SupportEntityBoundaryFinite && rightFoot.SupportEntityBoundaryFinite;
            bool supportEntityReady = supportEntityFinite && leftFoot.SupportEntityBoundaryEvidenceReady && rightFoot.SupportEntityBoundaryEvidenceReady;
            bool planarDampingContract = testArmKnown && built &&
                leftFoot.StancePlanarDampingFinite && rightFoot.StancePlanarDampingFinite &&
                leftFoot.StancePlanarDampingEnabled && rightFoot.StancePlanarDampingEnabled &&
                string.Equals(leftFoot.StancePlanarDampingArm, FootStancePlanarRotationalDampingIsolation.ArmB, StringComparison.Ordinal) &&
                string.Equals(rightFoot.StancePlanarDampingArm, FootStancePlanarRotationalDampingIsolation.ArmB, StringComparison.Ordinal);
            bool planarDampingExposure = built &&
                leftFoot.StancePlanarDampingLoadedSolves >= FootStancePlanarRotationalDampingIsolation.MinimumLoadedSolves &&
                rightFoot.StancePlanarDampingLoadedSolves >= FootStancePlanarRotationalDampingIsolation.MinimumLoadedSolves &&
                leftFoot.StancePlanarDampingNonzeroWrites > 0 && rightFoot.StancePlanarDampingNonzeroWrites > 0;
            bool localRunSafety = built && FootRunSafety(leftFoot) && FootRunSafety(rightFoot) && bilateral != null && bilateral.ClockParityPass &&
                                  bilateral.FailClosedEvents == 0 && !bilateral.PrimaryDriveAborted && !bilateral.SecondaryDriveAborted;

            bool qualified = testArmKnown && upperLoadContract && upperLoadExposure && kneeReactionContract && kneeReactionExposure && surfaceContract && surfaceExposure && frozenWorldGravityBoundary && zeroFrictionBaseline && frozenTetherTopology && lowerGroundFrozen && predictiveTailFrozen &&
                             footGravityFrozenOn && lowerGravityFrozenOn && reciprocalFrozenOn && transferFrozenOn && collisionSelectorContract && collisionObservedExposure && contactOffsetContract && contactOffsetExposure && supportAngularContract && supportAngularExposure && velocitySolverContract && velocitySolverExposure && positionSolverContract && positionSolverExposure &&
                             nativeGravityEnvironment && bilateralExposure && noTransferLeak && noReciprocalLeak && noLowerGravityLeak &&
                             lifecycleEvidence && supportLoadReactionDataset && angularImpulseEvidenceReady && couplingEvidenceReady && supportEntityReady && planarDampingContract && planarDampingExposure && localRunSafety;

            bool normalLoadReady = qualified && leftFoot.DynamicNormalLoadFinite && rightFoot.DynamicNormalLoadFinite &&
                leftFoot.DynamicNormalLoadEvidenceReady && rightFoot.DynamicNormalLoadEvidenceReady;
            bool normalWindowReady = normalLoadReady && leftFoot.SupportNormalWindowFinite && rightFoot.SupportNormalWindowFinite &&
                leftFoot.SupportNormalWindowEvidenceReady && rightFoot.SupportNormalWindowEvidenceReady;
            HipPelvisLoadPathReport hipPelvis = _legChain?.HipPelvisLoadPath;
            bool hipPelvisReady = normalWindowReady && _ankleTestArm == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB &&
                hipPelvis != null && hipPelvis.EvidenceReady;
            string reason;
            if (!built) reason = "FOUNDATION_NOT_BUILT";
            else if (!testArmKnown) reason = "NO_ACTIVE_V04450_UPPER_SELF_WEIGHT_TEST_PROVENANCE";
            else if (surface == null || !surface.Finite) reason = "V04410_SUPPORT_SCOPE_INVALID:" + (surface?.InvalidReason ?? "<none>");
            else if (bilateral != null && (bilateral.FailClosedEvents > 0 || bilateral.PrimaryDriveAborted || bilateral.SecondaryDriveAborted))
                reason = "PHYSICAL_RUN_SAFETY_NOT_CLEAN";
            else if (!frozenWorldGravityBoundary) reason = "PUBLIC_TORSO_OR_LEG_GRAVITY_BOUNDARY_NOT_FROZEN_OFF";
            else if (!zeroFrictionBaseline) reason = "V0480_ZERO_FRICTION_BASELINE_NOT_CLEAN";
            else if (!frozenTetherTopology) reason = "V0363_FOOT_TETHER_TOPOLOGY_CONTRACT_NOT_CLEAN";
            else if (!lowerGroundFrozen) reason = "V04140_B_LOWER_PLANAR_GROUND_BASELINE_NOT_CLEAN";
            else if (!predictiveTailFrozen) reason = "V04171_B_PREDICTIVE_TAIL_PROTECTION_NOT_FROZEN_CLEAN";
            else if (!footGravityFrozenOn) reason = "V04180_FOOT_NATIVE_GRAVITY_BASELINE_NOT_FROZEN_ON";
            else if (!lowerGravityFrozenOn) reason = "V04190_B_LOWER_NATIVE_GRAVITY_BASELINE_NOT_FROZEN_ON";
            else if (!reciprocalFrozenOn) reason = "V04200_B_RECIPROCAL_BASELINE_NOT_FROZEN_ON";
            else if (!transferFrozenOn) reason = "V04210_B_SELF_WEIGHT_TRANSFER_BASELINE_NOT_FROZEN_ON";
            else if (!upperLoadContract) reason = "V04450_UPPER_LOAD_SELECTOR_GRAVITY_OR_COMMAND_INVALID";
            else if (!upperLoadExposure) reason = "V04450_BILATERAL_LOAD_NONSTANCE_OR_UPPER_LEDGER_EXPOSURE_INSUFFICIENT";
            else if (!kneeReactionContract) reason = "V04430_KNEE_REACTION_SELECTOR_OR_COMMAND_INVALID";
            else if (!kneeReactionExposure) reason = "V04430_BILATERAL_REACTION_OR_NONSTANCE_EXPOSURE_INSUFFICIENT";
            else if (!surfaceContract) reason = "V04410_SUPPORT_SURFACE_CONTRACT_FAILED";
            else if (!surfaceExposure) reason = "V04410_BILATERAL_SUPPORT_EXPOSURE_INSUFFICIENT";
            else if (!contactOffsetContract) reason = "V04390_FOOT_CONTACT_OFFSET_SELECTOR_OR_READBACK_FAILED";
            else if ((leftFoot != null && leftFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves > 0) ||
                     (rightFoot != null && rightFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves > 0))
                reason = "V04390_SUPPORT_ENVIRONMENT_EXCLUDED_SOLVES";
            else if (!contactOffsetExposure) reason = "V04390_BILATERAL_CONTACT_OFFSET_EXPOSURE_INCOMPLETE_OR_INVALID";
            else if (!collisionSelectorContract) reason = "V04390_FROZEN_SPECULATIVE_CCD_CONTRACT_FAILED";
            else if (!collisionObservedExposure) reason = "V04390_FROZEN_SPECULATIVE_CCD_EXPOSURE_INSUFFICIENT_OR_MISMATCH";
            else if (!supportAngularContract) reason = "V04260_SELECTOR_OR_PASSIVITY_CONTRACT_FAILED";
            else if (!supportAngularExposure) reason = "V04260_BILATERAL_EXPOSURE_INSUFFICIENT";
            else if (!velocitySolverContract) reason = "V04270_CAPTURED_VELOCITY_OR_SOLVER_READBACK_FAILED";
            else if (!velocitySolverExposure) reason = "V04270_BILATERAL_LOADED_SOLVE_EXPOSURE_INSUFFICIENT";
            else if (!positionSolverContract) reason = "V04310_POSITION_SELECTOR_OR_READBACK_CONTRACT_FAILED";
            else if (!positionSolverExposure) reason = "V04310_BILATERAL_OBSERVED_POSITION_INTERVENTION_INSUFFICIENT";
            else if (!nativeGravityEnvironment) reason = "NATIVE_GRAVITY_VECTOR_OR_EXPECTED_LOAD_INVALID";
            else if (!bilateralExposure) reason = "BILATERAL_SUPPORT_LOAD_EXPOSURE_INSUFFICIENT";
            else if (!noTransferLeak) reason = "SELF_WEIGHT_TRANSFER_NONSTANCE_LEAK";
            else if (!noReciprocalLeak) reason = "V04200_RECIPROCAL_NONSTANCE_LEAK";
            else if (!noLowerGravityLeak) reason = "NATIVE_LOWER_GRAVITY_LEAK_OR_READBACK_FAILURE";
            else if (!lifecycleEvidence) reason = "STANCE_LIFECYCLE_EVIDENCE_NOT_CLEAN";
            else if (!supportLoadReactionDataset) reason = "SUPPORT_LOAD_REACTION_DATASET_NOT_USABLE";
            else if (!angularImpulseEvidenceReady) reason = "V04250_ANGULAR_IMPULSE_ATTRIBUTION_EVIDENCE_NOT_READY";
            else if (!couplingEvidenceReady) reason = "V04280_COUPLING_OBSERVER_EVIDENCE_NOT_READY";
            else if (!supportEntityReady) reason = "V04290_SUPPORT_ENTITY_BOUNDARY_EVIDENCE_NOT_READY";
            else if (!planarDampingContract) reason = "V04300_PLANAR_DAMPING_SELECTOR_CONTRACT_FAILED";
            else if (!planarDampingExposure) reason = "V04300_BILATERAL_LOADED_INTERVENTION_EXPOSURE_INSUFFICIENT";
            else if (!localRunSafety) reason = "PHYSICAL_RUN_SAFETY_NOT_CLEAN";
            else if (!normalLoadReady) reason = "V04330_DOWNSTREAM_NORMAL_LOAD_LEDGER_NOT_READY";
            else if (!normalWindowReady) reason = "V04350_DOWNSTREAM_NORMAL_WINDOW_LEDGER_NOT_READY";
            else if (_ankleTestArm == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB && !hipPelvisReady)
                reason = "V04460_HIP_PELVIS_INPUT_OR_RECEIVER_AUDIT_NOT_READY:" + (hipPelvis?.Issue ?? "EXPOSURE_INSUFFICIENT");
            else reason = hipPelvisReady ? "V04460_HIP_PELVIS_LOAD_PATH_DATASET_READY" : "V04450_UPPER_SELF_WEIGHT_DATASET_READY";

            float leftFootExpectedJ = leftFoot != null ? leftFoot.StanceFootNativeGravityExpectedNormalImpulseMean : 0f;
            float rightFootExpectedJ = rightFoot != null ? rightFoot.StanceFootNativeGravityExpectedNormalImpulseMean : 0f;
            float leftLowerExpectedJ = leftLeg != null ? leftLeg.StanceLowerNativeGravityExpectedNormalImpulseMean : 0f;
            float rightLowerExpectedJ = rightLeg != null ? rightLeg.StanceLowerNativeGravityExpectedNormalImpulseMean : 0f;
            float leftUpperExpectedJ = leftLeg?.UpperSelfWeightSupport?.AppliedImpulseMean ?? 0f;
            float rightUpperExpectedJ = rightLeg?.UpperSelfWeightSupport?.AppliedImpulseMean ?? 0f;
            float leftExpectedCombinedJ = leftFootExpectedJ + leftLowerExpectedJ + leftUpperExpectedJ;
            float rightExpectedCombinedJ = rightFootExpectedJ + rightLowerExpectedJ + rightUpperExpectedJ;
            float leftObservedJ = leftFoot != null ? leftFoot.StanceFootNativeGravityGravityOnSignedNormalImpulseMean : 0f;
            float rightObservedJ = rightFoot != null ? rightFoot.StanceFootNativeGravityGravityOnSignedNormalImpulseMean : 0f;
            float leftLoadRatio = leftExpectedCombinedJ > 0.0000001f ? leftObservedJ / leftExpectedCombinedJ : 0f;
            float rightLoadRatio = rightExpectedCombinedJ > 0.0000001f ? rightObservedJ / rightExpectedCombinedJ : 0f;

            string result =
                "ARL Stage-E joined normal load motion windows" +
                " | physicalMode=" + mode +
                " | testArm=" + _ankleTestArm +
                " | collisionArm=" + expectedCollisionArm +
                " | positionSolverBackground=B12_FROZEN" +
                " | velocitySolverContract=" + velocitySolverContract + "/exposure=" + velocitySolverExposure +
                " | V04310PositionSolverContract=" + positionSolverContract + "/observedExposure=" + positionSolverExposure +
                " | supportAngularContract=" + supportAngularContract + "/exposure=" + supportAngularExposure +
                " | V04210SelfWeightTransfer=FROZEN_B_ON" +
                " | V04200Reciprocal=FROZEN_B_ON" +
                " | footGravity=V04180_B_FROZEN_ON" +
                " | lowerGravity=V04190_B_FROZEN_ON" +
                " | publicTorsoLegGravityOff=" + frozenWorldGravityBoundary +
                " | zeroFrictionBaseline=" + zeroFrictionBaseline +
                " | V0363FootTetherTopologyContract=" + frozenTetherTopology +
                " | frozenV04140B=" + lowerGroundFrozen +
                " | predictiveTailFrozen=" + predictiveTailFrozen +
                " | footGravityFrozenOn=" + footGravityFrozenOn +
                " | lowerGravityFrozenOn=" + lowerGravityFrozenOn +
                " | reciprocalFrozenOn=" + reciprocalFrozenOn +
                " | transferFrozenOn=" + transferFrozenOn +
                " | supportSurface=" + (surface != null ? surface.Summary() : "<none>") +
                " | V04410SupportSurfaceContract=" + surfaceContract + "/observedExposure=" + surfaceExposure +
                " | contactOffsetBackground=FROZEN_20MM_BOTH_ARMS" +
                " | contactOffsetContract=" + contactOffsetContract +
                " | V04390ContactOffsetObservedExposure=" + contactOffsetExposure +
                "/resolved(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetResolvedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetResolvedSolves : 0) +
                "/mismatch(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetMismatchSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetMismatchSolves : 0) +
                "/entityChangedWithinSolve(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetBoundaryChangedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetBoundaryChangedSolves : 0) +
                "/readUnavailable(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetReadUnavailableSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetReadUnavailableSolves : 0) +
                "/environmentExcluded(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves : 0) +
                "/supportLostWithinSolve(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetSupportLostSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetSupportLostSolves : 0) +
                "/groundBaselines(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetGroundBaselines : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetGroundBaselines : 0) +
                "/groundSwitchesAcrossLoadedSolves(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryContactOffsetGroundSwitches : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryContactOffsetGroundSwitches : 0) +
                " | gravityReadbackScope=" + (completed ? "COMPLETED_REQUIRES_QUIESCED" : "LIVE_AUTHORIZED_STANCE_ALLOWED") +
                " | collisionSelectorContract=" + collisionSelectorContract +
                " | frozenSpeculativeCcdObservedExposure=" + collisionObservedExposure +
                "/resolved(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryCcdResolvedSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryCcdResolvedSolves : 0) +
                "/mismatch(left-right)=" + (leftFoot != null ? leftFoot.SupportEntityBoundaryCcdMismatchSolves : 0) + "-" +
                    (rightFoot != null ? rightFoot.SupportEntityBoundaryCcdMismatchSolves : 0) +
                " | nativeGravityEnvironment=" + nativeGravityEnvironment +
                " | bilateralExposure=" + bilateralExposure +
                " | noTransferLeak=" + noTransferLeak +
                " | noReciprocalLeak=" + noReciprocalLeak +
                " | noLowerGravityLeak=" + noLowerGravityLeak +
                " | lifecycleEvidence=" + lifecycleEvidence +
                " | supportLoadReactionDataset=" + supportLoadReactionDataset +
                " | V04250ObserverFinite=" + angularImpulseObserverFinite +
                " | FOOT_ANGULAR_IMPULSE_ATTRIBUTION_EVIDENCE_READY=" + angularImpulseEvidenceReady +
                " | localRunSafety=" + localRunSafety +
                " | V04280ObserverFinite=" + couplingObserverFinite + "/ready=" + couplingEvidenceReady +
                " | V04290ObserverFinite=" + supportEntityFinite + "/ready=" + supportEntityReady +
                " | V04300Contract=" + planarDampingContract + "/loadedExposure=" + planarDampingExposure +
                " | V04330AuditReady(left-right)=" + (leftFoot != null && leftFoot.DynamicNormalLoadEvidenceReady) + "-" + (rightFoot != null && rightFoot.DynamicNormalLoadEvidenceReady) +
                " | DYNAMIC_NORMAL_LOAD_DATASET_QUALIFIED=" + normalLoadReady +
                " | V04350WindowAuditReady(left-right)=" + (leftFoot != null && leftFoot.SupportNormalWindowEvidenceReady) + "-" + (rightFoot != null && rightFoot.SupportNormalWindowEvidenceReady) +
                " | NORMAL_LOAD_WINDOW_DATASET_QUALIFIED=" + normalWindowReady +
                " | dynamicLoadVerdict=REQUIRES_NORMAL_IMPULSE_LEDGER_REVIEW" +
                " | UPPER_SELF_WEIGHT_TRANSFER_DATASET_QUALIFIED=" + normalWindowReady +
                " | V04450UpperLoadContract=" + upperLoadContract + "/exposure=" + upperLoadExposure +
                " | HIP_PELVIS_LOAD_PATH_DATASET_QUALIFIED=" + hipPelvisReady +
                " | KNEE_SUPPORT_NORMAL_REACTION_DATASET_QUALIFIED=" + qualified +
                " | V04430ReactionContract=" + kneeReactionContract + "/exposure=" + kneeReactionExposure +
                " | MECHANISM_VERDICT=REQUIRES_UPPER_HIP_MOMENTUM_AND_SERIAL_LOAD_GROUND_LEDGER_REVIEW_NOT_WHOLE_BODY_SUPPORT_PASS" +
                " | reason=" + reason +
                "\n  left: stancePlanarDamping=" + Clean(leftFoot != null ? leftFoot.StancePlanarDampingSummary : null) + "/positionSolver=" + Clean(leftFoot != null ? leftFoot.FootPositionSolverSummary : null) + "/velocitySolver=" + Clean(leftFoot != null ? leftFoot.FootVelocitySolverSummary : null) + "/supportAngular=" + Clean(leftFoot != null ? leftFoot.StanceSupportAngularSummary : null) + "/collision=" + Clean(leftFoot != null ? leftFoot.FootCollisionDetectionModeSummary : null) +
                    "/contactOffset=" + Clean(leftFoot != null ? leftFoot.FootContactOffsetSummary : null) +
                    "/selfWeightTransfer=" + Clean(leftLeg != null ? leftLeg.StanceLowerSelfWeightTransferSummary : null) +
                    "/loadJ(observed-combinedExpected-ratio)=" + F(leftObservedJ) + "-" + F(leftExpectedCombinedJ) + "-" + F(leftLoadRatio) +
                    "/anchorsMm(knee-footPeak)=" + Fmm(leftLeg != null ? leftLeg.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters : 0f) + "-" + Fmm(leftFoot != null ? leftFoot.TraceObservedAnchorErrorMaxMeters : 0f) +
                    "/reaction=" + SupportLoadReactionDatasetSummary(leftFoot) +
                    "/angularImpulse=" + Clean(leftFoot != null ? leftFoot.FootAngularImpulseAttributionTraceSummary : null) +
                    "/coupling=" + Clean(leftFoot != null ? leftFoot.RockingTetherCouplingTraceSummary : null) +
                    "/supportEntity=" + Clean(leftFoot != null ? leftFoot.SupportEntityBoundaryTraceSummary : null) +
                "\n  right: stancePlanarDamping=" + Clean(rightFoot != null ? rightFoot.StancePlanarDampingSummary : null) + "/positionSolver=" + Clean(rightFoot != null ? rightFoot.FootPositionSolverSummary : null) + "/velocitySolver=" + Clean(rightFoot != null ? rightFoot.FootVelocitySolverSummary : null) + "/supportAngular=" + Clean(rightFoot != null ? rightFoot.StanceSupportAngularSummary : null) + "/collision=" + Clean(rightFoot != null ? rightFoot.FootCollisionDetectionModeSummary : null) +
                    "/contactOffset=" + Clean(rightFoot != null ? rightFoot.FootContactOffsetSummary : null) +
                    "/selfWeightTransfer=" + Clean(rightLeg != null ? rightLeg.StanceLowerSelfWeightTransferSummary : null) +
                    "/loadJ(observed-combinedExpected-ratio)=" + F(rightObservedJ) + "-" + F(rightExpectedCombinedJ) + "-" + F(rightLoadRatio) +
                    "/anchorsMm(knee-footPeak)=" + Fmm(rightLeg != null ? rightLeg.KneeCharacterJointResponseIsolationCurrentAnchorErrorMeters : 0f) + "-" + Fmm(rightFoot != null ? rightFoot.TraceObservedAnchorErrorMaxMeters : 0f) +
                    "/reaction=" + SupportLoadReactionDatasetSummary(rightFoot) +
                    "/angularImpulse=" + Clean(rightFoot != null ? rightFoot.FootAngularImpulseAttributionTraceSummary : null) +
                    "/coupling=" + Clean(rightFoot != null ? rightFoot.RockingTetherCouplingTraceSummary : null) +
                    "/supportEntity=" + Clean(rightFoot != null ? rightFoot.SupportEntityBoundaryTraceSummary : null) +
                "\n  V04430 leftKneeReaction=" + (leftLeg?.KneeSupportReaction?.GetSummary() ?? "<none>") +
                "\n  V04430 rightKneeReaction=" + (rightLeg?.KneeSupportReaction?.GetSummary() ?? "<none>") +
                "\n  V04450 leftUpperLoad=" + (leftLeg?.UpperSelfWeightSupport?.GetDetails() ?? "<none>") +
                "\n  V04450 rightUpperLoad=" + (rightLeg?.UpperSelfWeightSupport?.GetDetails() ?? "<none>") +
                "\n  V04460 hipPelvisLoadPath=" + (hipPelvis?.GetDetails() ?? "<none>") +
                "\n  V04460 canonical run is B on the unchanged V04450 loaded-leg plant. A remains the explicit legacy Upper-gravity-OFF comparison and does not qualify the new receiver audit. Hip/common inputs are read-only; no Pelvis force, native tensor, pose owner or gravity is changed. Input exposure requires >=50 loaded and >=30 released samples per side, >=50 IDLE and >=30 motion/source-turn samples; readiness also requires the bilateral downstream load/window datasets." +
                "\n  V04450 A=Upper weight OFF; B=Upper exact-STANCE native gravity plus explicit static Upper/Lower and Lower/Foot COM force pairs. Both use V04430 B reduced-mass normal PD and V04410 static 30cm Box. Torso gravity OFF. Original scene objects and the global collision matrix are not modified." +
                "\n  Both arms keep production torque, Foot contactOffset20mm, Speculative CCD, position12/captured velocity and frozen loads/gates. Exposure needs >=50 verified Foot-pair samples on each side plus actual callback and joined loaded-solve evidence. V04430 common B additionally needs >=50 eligible, >=30 nonzero candidate and >=30 nonstance samples per side. V04450 needs >=50 eligible and resolved loaded Upper ledger samples, >=30 nonstance, with A load writes=0/B load writes=eligible. Stop captures evidence before selector reset." +
                "\n  V04290 evidence: same authorized Ground object and cached ancestor path at pre-solve, first/last callback and N+1 before gravity clear/lifecycle; current probe is appended afterwards. Stable snapshots cannot exclude a transient change between samples, transient layer collision-matrix changes or an internal solver/CCD interruption. No automatic mechanism verdict.";
            return result;
        }

        private static bool FootContactOffsetObservedExposure(FootAnkleFoundationReport foot) => foot != null &&
            foot.SupportEntityBoundaryContactOffsetResolvedSolves >= FootSupportEntitySolveBoundaryObserver.MinimumResolvedSolves &&
            foot.SupportEntityBoundaryContactOffsetMismatchSolves == 0 &&
            foot.SupportEntityBoundaryContactOffsetBoundaryChangedSolves == 0 &&
            foot.SupportEntityBoundaryContactOffsetReadUnavailableSolves == 0 &&
            foot.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves == 0;

        private static bool FootContactOffsetSelectorContract(FootAnkleFoundationReport foot, string arm)
        {
            if (foot == null || !foot.FootContactOffsetInitialized || !foot.FootContactOffsetFinite ||
                !foot.FootContactOffsetReadbackValid || foot.FootContactOffsetOwnerId == 0 ||
                !string.Equals(foot.FootContactOffsetArm, arm, StringComparison.Ordinal)) return false;
            bool widened = string.Equals(arm, FootContactOffsetIsolation.ArmB, StringComparison.Ordinal);
            bool baseline = string.Equals(arm, FootContactOffsetIsolation.ArmA, StringComparison.Ordinal) ||
                string.Equals(arm, FootContactOffsetIsolation.IdleArm, StringComparison.Ordinal);
            float expected = widened ? FootContactOffsetIsolation.BOffsetMeters : foot.FootContactOffsetBaselineMeters;
            return (widened || baseline) &&
                (!widened || foot.FootContactOffsetBaselineMeters < expected - FootContactOffsetIsolation.ReadbackToleranceMeters) &&
                FootContactOffsetIsolation.Matches(foot.FootContactOffsetSelectedMeters, expected) &&
                FootContactOffsetIsolation.Matches(foot.FootContactOffsetActualMeters, expected);
        }

        private static bool LowerGravityCurrentStateValid(SingleLegChainReport leg, FootAnkleFoundationReport foot, bool completed)
        {
            if (leg == null) return false;
            if (!leg.StanceLowerNativeGravityCurrentApplied) return true;
            // A completed audit has been quiesced. A live snapshot may legally show the current stance load.
            return !completed && foot != null && leg.StanceLowerNativeGravityEnabled && leg.StanceLowerNativeGravityFinite &&
                leg.FixedUpdateSequence > 0 && leg.StanceLowerNativeGravityCurrentScheduledFixed == leg.FixedUpdateSequence &&
                foot.FixedUpdateSequence == leg.FixedUpdateSequence && foot.ContactShadowCurrentContactActive &&
                foot.StanceContactLifecycleEnabled && foot.StanceContactLifecycleFinite &&
                foot.StanceContactLifecycleAuthorizedColliderInstanceId != 0 &&
                string.Equals(foot.GameNativeGaitPhaseState, "STANCE", StringComparison.Ordinal) &&
                foot.StanceFootNativeGravityCurrentApplied &&
                foot.StanceFootNativeGravityCurrentScheduledFixed == leg.FixedUpdateSequence;
        }

        private static bool FootCcdSelectorContract(FootAnkleFoundationReport foot, string arm)
        {
            return foot != null && foot.FootCollisionDetectionModeFinite &&
                FootCollisionDetectionModeIsolation.TryGetMode(arm, out CollisionDetectionMode expected) &&
                string.Equals(foot.FootCollisionDetectionModeArm, arm, StringComparison.Ordinal) &&
                string.Equals(foot.FootCollisionDetectionModeReadback, expected.ToString(), StringComparison.Ordinal);
        }

        private static bool FootVelocitySolverContract(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.FootVelocitySolverInitialized && foot.FootVelocitySolverFinite &&
                foot.FootVelocitySolverReadbackValid && foot.FootVelocitySolverReadbackFailures == 0 &&
                foot.FootVelocitySolverBaselineVelocity > 0 && foot.FootVelocitySolverBaselinePosition > 0 &&
                string.Equals(foot.FootVelocitySolverArm, FootSolverIterationBaselineContract.ArmA, StringComparison.Ordinal) &&
                foot.FootVelocitySolverSelectedVelocity == foot.FootVelocitySolverBaselineVelocity &&
                foot.FootVelocitySolverActualVelocity == foot.FootVelocitySolverBaselineVelocity &&
                foot.FootPositionSolverSelectedPosition > 0 &&
                foot.FootVelocitySolverActualPosition == (foot.FootVelocitySolverQuiesced
                    ? foot.FootVelocitySolverBaselinePosition : foot.FootPositionSolverSelectedPosition) &&
                (!foot.FootVelocitySolverQuiesced || foot.FootVelocitySolverBaselineRestored);
        }

        private static bool FootPositionSolverContract(FootAnkleFoundationReport foot)
        {
            return FootVelocitySolverContract(foot) &&
                string.Equals(foot.FootPositionSolverArm, FootSolverIterationBaselineContract.PositionArmB, StringComparison.Ordinal) &&
                foot.FootPositionSolverSelectedPosition == FootSolverIterationBaselineContract.BPositionIterations &&
                foot.FootPositionSolverSelectedPosition > foot.FootVelocitySolverBaselinePosition &&
                (foot.FootVelocitySolverQuiesced ? !foot.FootPositionSolverEnabled : foot.FootPositionSolverEnabled);
        }

        public static string GetFreeSpaceFoundationReadinessAudit()
        {
            const string mode = "FULL_FREE_SPACE_FOUNDATION_READINESS_AUDIT_V0379";
            TwoBodyProxyReport torso = _proxy != null ? _proxy.Report : null;
            BilateralLegReport bilateral = _legChain != null ? _legChain.BilateralReport : null;
            SingleLegChainReport primary = _legChain != null ? _legChain.PrimaryReport : null;
            SingleLegChainReport secondary = _legChain != null ? _legChain.SecondaryReport : null;
            FootAnkleFoundationReport leftFoot = _legChain != null ? _legChain.PrimaryFootReport : null;
            FootAnkleFoundationReport rightFoot = _legChain != null ? _legChain.SecondaryFootReport : null;

            bool built = torso != null && torso.Built && _legChain != null && _legChain.IsBuilt && bilateral != null && bilateral.Built;
            bool frozenStack = built && _legChain.FrozenProductionStackReady;
            bool footProduction = built && _legChain.FootProductionBaselineReady && _legChain.FootRuntimeContractReady &&
                                  string.Equals(_legChain.AnkleAngularTransportPolicy, FootAnkleAngularTransport.Production, StringComparison.Ordinal) &&
                                  string.Equals(_legChain.TranslationCaptureAuthorityArm, FootAnkleTranslationCaptureAuthorityIsolation.Production, StringComparison.Ordinal) &&
                                  _legChain.TranslationCaptureAuthorityArmsMatch;
            bool environmentFreeSpace = torso != null && !torso.GravityEnabled && !torso.CollidersEnabled &&
                                        FootFreeSpaceContract(leftFoot) && FootFreeSpaceContract(rightFoot);
            bool productionContract = built && frozenStack && footProduction && environmentFreeSpace;

            bool runEvidence = bilateral != null && bilateral.ClockSamples > 0 &&
                               bilateral.PrimaryTraceSamples > 0 && bilateral.SecondaryTraceSamples > 0 &&
                               primary != null && primary.TraceSamples > 0 && secondary != null && secondary.TraceSamples > 0 &&
                               leftFoot != null && leftFoot.TraceSamples > 0 && rightFoot != null && rightFoot.TraceSamples > 0;

            // v0.4.1 contact-shadow B intentionally changes the tested plant. Once either Foot has accumulated
            // active contact-shadow evidence, that trace must not later be re-labelled as a v0.3.79 free-space
            // certification merely because StopAnkleTest has already switched the colliders back OFF.
            bool runWasFreeSpace = runEvidence &&
                                   leftFoot.ContactShadowTraceFixedSamples == 0 && rightFoot.ContactShadowTraceFixedSamples == 0 &&
                                   leftFoot.ContactShadowTraceMatchedCallbacks == 0 && rightFoot.ContactShadowTraceMatchedCallbacks == 0 &&
                                   leftFoot.ContactShadowTraceUnexpectedCallbacks == 0 && rightFoot.ContactShadowTraceUnexpectedCallbacks == 0;

            bool legLocalSafety = runEvidence && primary.Finite && secondary.Finite &&
                                  primary.SingleLegFreezeProductionContractPass && secondary.SingleLegFreezeProductionContractPass &&
                                  primary.SingleLegFreezeCalibrationPass && secondary.SingleLegFreezeCalibrationPass &&
                                  primary.SingleLegFreezeInertiaPass && secondary.SingleLegFreezeInertiaPass &&
                                  primary.SingleLegFreezeRunSafetyPass && secondary.SingleLegFreezeRunSafetyPass &&
                                  !primary.DriveAborted && !secondary.DriveAborted;
            bool bilateralLocalSafety = runEvidence && bilateral.ClockParityPass && bilateral.FailClosedEvents == 0 &&
                                        !bilateral.PrimaryDriveAborted && !bilateral.SecondaryDriveAborted &&
                                        bilateral.FixedSequenceMismatchSamples == 0 && bilateral.RenderSequenceMismatchSamples == 0 &&
                                        bilateral.RenderFreshnessMismatchSamples == 0 && bilateral.RenderHeldDepthMismatchSamples == 0 &&
                                        bilateral.DriveStateMismatchSamples == 0;
            bool footLocalSafety = runEvidence && FootRunSafety(leftFoot) && FootRunSafety(rightFoot);
            bool torsoNoAbort = torso != null && torso.FiniteState && torso.FreezeValidationComplete && !torso.FreezeValidationTraceAborted;

            bool recoveryEvidenceHealthy = torsoNoAbort &&
                                           torso.FreezeRecoveryCompletedCount >= SourceAwareFreezeValidation.MinCompletedRecoverySessions &&
                                           torso.FreezeRecoveryPassedCount == torso.FreezeRecoveryCompletedCount &&
                                           torso.FreezeRecoveryFailedCount == 0 &&
                                           torso.FreezeRecoveryInvalidStateStepCount == 0 &&
                                           torso.FreezeRecoveryParentFeedforwardClampStepCount == 0 &&
                                           torso.FreezeRecoveryMaxUnder5DegreesSecondsFromSourceClose <= SourceAwareFreezeValidation.RecoveryLooseMaxSecondsFromSourceClose &&
                                           torso.FreezeRecoveryMaxUnder2DegreesSecondsFromSourceClose <= SourceAwareFreezeValidation.RecoveryStrictMaxSecondsFromSourceClose &&
                                           torso.FreezeRecoveryMaxAnchorErrorMeters <= SourceAwareFreezeValidation.RecoveryAnchorErrorMaxMeters;
            bool rapidTurnSurvivalHealthy = torsoNoAbort &&
                                            torso.FreezeSourceInvalidStateStepCount == 0 &&
                                            torso.FreezeSourceParentFeedforwardClampStepCount == 0 &&
                                            torso.FreezeSourceMaxAnchorErrorMeters <= SourceAwareFreezeValidation.SourceTurnMaxAnchorErrorMeters;
            bool legacyCoverageOnly = torsoNoAbort && !torso.TwoBodyFreezeReady &&
                                      string.Equals(torso.FreezeValidationReason, "INSUFFICIENT_SOURCE_TURN_DATA", StringComparison.Ordinal) &&
                                      torso.NormalTrackingPass && recoveryEvidenceHealthy && rapidTurnSurvivalHealthy &&
                                      torso.SourceTurnEventCount >= SourceAwareFreezeValidation.MinSourceTurnEvents &&
                                      torso.SourceTurnDiscontinuityEventCount < SourceAwareFreezeValidation.MinSourceDiscontinuityEvents &&
                                      torso.SourceTurnRapidContinuousEventCount > 0;
            bool torsoCurrentSafety = torsoNoAbort && (torso.TwoBodyFreezeReady || legacyCoverageOnly);
            bool currentRunSafety = runWasFreeSpace && torsoCurrentSafety && legLocalSafety && bilateralLocalSafety && footLocalSafety;

            bool supportPlaneReady = FootSupportPlaneReady(leftFoot) && FootSupportPlaneReady(rightFoot);
            bool contactProductionReady = leftFoot != null && rightFoot != null &&
                                          leftFoot.ContactGeometryAuthorized && rightFoot.ContactGeometryAuthorized &&
                                          leftFoot.ColliderEnabled && rightFoot.ColliderEnabled &&
                                          leftFoot.PhysicalGroundContactPossible && rightFoot.PhysicalGroundContactPossible &&
                                          torso != null && torso.GravityEnabled;

            // Stage-E entry means the free-space plant is sufficiently frozen to BEGIN a new contact/gravity
            // requalification. It deliberately does not mean current contact production is ready.
            bool stageEEntryReady = productionContract && currentRunSafety && supportPlaneReady && !contactProductionReady;

            string coverageClass;
            if (torso == null || !torso.FreezeValidationComplete) coverageClass = "NOT_FINALIZED";
            else if (torso.TwoBodyFreezeReady) coverageClass = "CURRENT_RUN_ALL_LEGACY_GATES_PASS";
            else if (legacyCoverageOnly) coverageClass = "LEGACY_SOURCE_DISCONTINUITY_COVERAGE_GAP_ONLY";
            else coverageClass = "LEGACY_GATE_PHYSICAL_OR_VALIDATION_FAIL:" + (torso.FreezeValidationReason ?? "<none>");

            string reason;
            if (!built) reason = "FOUNDATION_NOT_BUILT";
            else if (!productionContract) reason = "FROZEN_PRODUCTION_CONTRACT_MISMATCH";
            else if (!runEvidence) reason = "CURRENT_RUN_EVIDENCE_NOT_AVAILABLE";
            else if (!runWasFreeSpace) reason = "CURRENT_TRACE_IS_V0410_CONTACT_SHADOW_NOT_FREE_SPACE";
            else if (!currentRunSafety) reason = "CURRENT_RUN_SAFETY_OR_INTEGRATION_FAIL";
            else if (!supportPlaneReady) reason = "FOOT_SUPPORT_PLANE_NOT_READY_FOR_STAGE_E_REQUALIFICATION";
            else if (contactProductionReady) reason = "CONTACT_ALREADY_ACTIVE_OUTSIDE_V0379_FREE_SPACE_SCOPE";
            else reason = "READY_TO_BEGIN_STAGE_E_CONTACT_REQUALIFICATION";

            string result =
                "ARL full free-space foundation audit" +
                " | mode=" + mode +
                " | built=" + built +
                " | productionContract=" + productionContract +
                " | frozenStack=" + frozenStack +
                " | footProduction=" + footProduction +
                " | freeSpaceEnv=" + environmentFreeSpace +
                " | runEvidence=" + runEvidence +
                " | runWasFreeSpace=" + runWasFreeSpace +
                " | currentRunSafety=" + currentRunSafety +
                " | torsoSafety=" + torsoCurrentSafety +
                " | legLocalSafety=" + legLocalSafety +
                " | bilateralLocalSafety=" + bilateralLocalSafety +
                " | footLocalSafety=" + footLocalSafety +
                " | legacyCoverage=" + coverageClass +
                " | supportPlaneReady=" + supportPlaneReady +
                " | contactProductionReady=" + contactProductionReady +
                " | STAGE_E_ENTRY_READY=" + stageEEntryReady +
                " | reason=" + reason +
                "\n  torso: freezeReady=" + (torso != null && torso.TwoBodyFreezeReady) +
                "/normal=" + (torso != null && torso.NormalTrackingPass) +
                "/sourceLegacyGate=" + (torso != null && torso.SourceDiscontinuitySurvivalPass) +
                "/recoveryLegacyGate=" + (torso != null && torso.RecoveryPass) +
                "/rapidTurnSurvivalHealthy=" + rapidTurnSurvivalHealthy +
                "/recoveryEvidenceHealthy=" + recoveryEvidenceHealthy +
                "/events(total/discontinuity/rapid)=" + (torso != null ? torso.SourceTurnEventCount : 0) + "/" +
                    (torso != null ? torso.SourceTurnDiscontinuityEventCount : 0) + "/" + (torso != null ? torso.SourceTurnRapidContinuousEventCount : 0) +
                "\n  bilateral: n=" + (bilateral != null ? bilateral.ClockSamples : 0) +
                "/clockParity=" + (bilateral != null && bilateral.ClockParityPass) +
                "/failClosed=" + (bilateral != null ? bilateral.FailClosedEvents : 0) +
                "/aborted=" + (bilateral != null && bilateral.PrimaryDriveAborted) + "/" + (bilateral != null && bilateral.SecondaryDriveAborted) +
                "/kneeAnchorPeakMm=" + Fmm(bilateral != null ? bilateral.PrimaryPeakKneeAnchorMeters : 0f) + "/" + Fmm(bilateral != null ? bilateral.SecondaryPeakKneeAnchorMeters : 0f) +
                "\n  foot: traceN=" + (leftFoot != null ? leftFoot.TraceSamples : 0) + "/" + (rightFoot != null ? rightFoot.TraceSamples : 0) +
                "/trackMeanDeg=" + F(leftFoot != null ? leftFoot.TraceAnkleRelativeErrorMeanDegrees : 0f) + "/" + F(rightFoot != null ? rightFoot.TraceAnkleRelativeErrorMeanDegrees : 0f) +
                "/trackMaxDeg=" + F(leftFoot != null ? leftFoot.TraceAnkleRelativeErrorMaxDegrees : 0f) + "/" + F(rightFoot != null ? rightFoot.TraceAnkleRelativeErrorMaxDegrees : 0f) +
                "/anchorPeakMm=" + Fmm(leftFoot != null ? leftFoot.TraceObservedAnchorErrorMaxMeters : 0f) + "/" + Fmm(rightFoot != null ? rightFoot.TraceObservedAnchorErrorMaxMeters : 0f) +
                "\n  boundary: Gravity/Collider/contact/GSP remain outside current production; Stage-E entry authorizes only a new causal contact requalification, not direct contact enablement.";
            _logger?.LogWarning("[ARL][FREE_SPACE_FOUNDATION_AUDIT_V079] " + result);
            return result;
        }

        private static bool FootFreeSpaceContract(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.Built && foot.Finite && foot.MassPropertiesValidated &&
                   !foot.GravityEnabled && !foot.ColliderEnabled && foot.ColliderFrozenOff &&
                   !foot.AnkleJointExists && foot.AnkleDriveFinite && foot.AnkleConstraintFinite &&
                   foot.AnkleTransportFinite && foot.AnkleAngularTransportFinite &&
                   foot.TargetAccelerationConditionerFinite && foot.TranslationCaptureAuthorityFinite &&
                   foot.StanceContactFrictionIsolationFinite;
        }

        private static bool SupportLoadReactionDatasetUsable(FootAnkleFoundationReport report)
        {
            if (report == null || !report.MechanicalReactionFinite || !report.MechanicalReactionReadbackClean ||
                !report.MechanicalReactionSolverWitnessQualified)
                return false;

            if (report.MechanicalReactionTraceSupportNormalSamples < 5 ||
                report.MechanicalReactionTraceProjectionSamples < 5 ||
                report.MechanicalReactionTraceNonZeroImpulseSamples < 1 ||
                report.MechanicalReactionTraceSupportNormalValidRatio < 0.995f)
                return false;

            // v0.4.22 keeps strict support-normal/lifecycle fields visible; the support-load consumer still tolerates only the previously defined rare geometry-only animation transient
            // does not invalidate the load dataset when every accepted reaction is identity-clean and finite.
            // This support-load consumer rule is reused by frozen V04190 telemetry and V04200; it does not relax contact lifecycle or hard safety gates.
            int invalid = report.MechanicalReactionTraceSupportNormalInvalidSamples;
            bool invalidsAreGeometryOnly = invalid == report.MechanicalReactionTraceSupportInvalidGeometryNotReady &&
                                           report.MechanicalReactionTraceSupportInvalidAuthorizedIdUnavailable == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidSupportIdUnavailable == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidIdMismatch == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidFootprintNotSameSurface == 0 &&
                                           report.MechanicalReactionTraceSupportInvalidNormalNonFiniteOrZero == 0 &&
                                           report.MechanicalReactionTraceSupportColliderMismatchSamples == 0 &&
                                           report.MechanicalReactionTraceReadFailures == 0 &&
                                           report.MechanicalReactionTraceImpulseReadFailures == 0 &&
                                           report.MechanicalReactionTraceRelativeVelocityReadFailures == 0 &&
                                           report.MechanicalReactionTracePostBodyReadFailures == 0;
            return invalidsAreGeometryOnly;
        }

        private static string SupportLoadReactionDatasetSummary(FootAnkleFoundationReport report)
        {
            if (report == null) return "<none>";
            return "usable=" + SupportLoadReactionDatasetUsable(report) +
                   "/strictDynamic=" + DynamicStanceReactionEvidence(report) +
                   "/strictSupportN=" + report.MechanicalReactionSupportNormalWitnessQualified +
                   "/validRatio=" + F(report.MechanicalReactionTraceSupportNormalValidRatio) +
                   "/invalid=" + report.MechanicalReactionTraceSupportNormalInvalidSamples +
                   "/geometryInvalid=" + report.MechanicalReactionTraceSupportInvalidGeometryNotReady +
                   "/maxStreak=" + report.MechanicalReactionTraceSupportInvalidMaxStreak +
                   "/idMismatch=" + report.MechanicalReactionTraceSupportInvalidIdMismatch +
                   "/readFail=" + report.MechanicalReactionTraceReadFailures;
        }

        private static bool DynamicStanceReactionEvidence(FootAnkleFoundationReport report)
        {
            if (report == null || !report.MechanicalReactionFinite || !report.MechanicalReactionReadbackClean ||
                !report.MechanicalReactionSolverWitnessQualified)
                return false;

            // Preserve the historical v0.4.3 strict zero-invalid verdict exactly. Dynamic stance consumers
            // may additionally accept isolated one-FixedUpdate geometry-not-ready boundary samples from
            // repeated contact enter/exit, but never ID mismatches, missing authorization, non-finite normals
            // or multi-frame invalid streaks. This is a consumer contract, not a rewrite of v0.4.3 history.
            if (report.MechanicalReactionSupportNormalWitnessQualified)
                return true;

            int invalid = report.MechanicalReactionTraceSupportNormalInvalidSamples;
            bool onlyIsolatedGeometryBoundary =
                invalid > 0 &&
                invalid == report.MechanicalReactionTraceSupportInvalidGeometryNotReady &&
                report.MechanicalReactionTraceSupportInvalidAuthorizedIdUnavailable == 0 &&
                report.MechanicalReactionTraceSupportInvalidSupportIdUnavailable == 0 &&
                report.MechanicalReactionTraceSupportInvalidIdMismatch == 0 &&
                report.MechanicalReactionTraceSupportInvalidFootprintNotSameSurface == 0 &&
                report.MechanicalReactionTraceSupportInvalidNormalNonFiniteOrZero == 0 &&
                report.MechanicalReactionTraceSupportColliderMismatchSamples == 0 &&
                report.MechanicalReactionTraceSupportInvalidMaxStreak <= 1;

            return onlyIsolatedGeometryBoundary &&
                   report.MechanicalReactionTraceSupportNormalSamples >= 5 &&
                   report.MechanicalReactionTraceProjectionSamples >= 5 &&
                   report.MechanicalReactionTraceNonZeroImpulseSamples >= 1 &&
                   report.MechanicalReactionTraceSupportNormalValidRatio >= 0.995f;
        }

        private static bool FootRunSafety(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.Built && foot.Finite && foot.TraceSamples > 0 &&
                   foot.TraceAnkleAnchorErrorMaxMeters <= 0.030001f &&
                   foot.TraceObservedFixedSamples > 0 && foot.TraceObservedAnchorErrorMaxMeters <= 0.030001f &&
                   foot.FootCollisionDetectionModeFinite && foot.AnkleDriveFinite && foot.AnkleConstraintFinite && foot.AnkleTransportFinite &&
                   foot.AnkleAngularTransportFinite && foot.TargetAccelerationConditionerFinite &&
                   foot.TranslationCaptureAuthorityFinite && foot.TranslationPredictiveTailCaptureFinite && foot.StanceContactLifecycleFinite &&
                   foot.StanceContactFrictionIsolationFinite && foot.StanceLowerGroundAuthorityFinite &&
                   foot.StanceFootNativeGravityFinite;
        }

        private static bool FootSupportPlaneReady(FootAnkleFoundationReport foot)
        {
            return foot != null && foot.Built && foot.SupportPlaneCalibrationApplied &&
                   foot.SupportPlaneCalibrationHitCount >= 3 && foot.SupportPlaneCalibrationNormalDot >= 0.65f;
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace('\r', ' ').Replace('\n', ' ');
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string Fmm(float meters) => (meters * 1000f).ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);

        public static string StartPelvisTest() => PelvisTestCoordinator.TestInstructions;
        public static string StartPelvisTest(string requestedArm) => StartPelvisRun(requestedArm, false);
        public static string StartPelvisLegTest() => "ARL v0.4.76 动态腿足：完成arl_test_setup、PCA/apply、hip_stack后执行arl_pelvis_leg_test(a或b)。A沿用旧躯干恢复资格，B使用当前受控骨盆/胸腔误差及原生连接点恢复资格，并在髋部目标缓冲时关闭原始动画加速度前馈；两臂足部均使用一致受限参考，骨盆均保持制动B，受力上限不变。自动[STARTED]后计时；结束stop、arl_test_cleanup、arl_test_status各一次。";
        public static string StartPelvisLegTest(string requestedArm) => StartPelvisRun(requestedArm, true);
        private static string StartPelvisRun(string requestedArm, bool withLegs)
        {
            if (string.Equals(requestedArm, "stop", StringComparison.OrdinalIgnoreCase)) return StopPelvisTest();
            if (PelvisTestActive && _pelvisWithLegs != withLegs) return "ARL pelvis | started=False | reason=OTHER_PELVIS_SCOPE_ACTIVE";
            if (AnkleTestActive) return "ARL pelvis | started=False | reason=ANKLE_TEST_ACTIVE";
            if (PelvisRunPort.ResolveRequestedArm(requestedArm, withLegs) == null)
                return "ARL pelvis | started=False | reason=INVALID_ARM | " + (withLegs ? StartPelvisLegTest() : StartPelvisTest());
            _pelvisWithLegs = withLegs; _lastTestScope = "PELVIS";
            return _pelvisTest.Start(requestedArm);
        }
        public static string StopPelvisTest() => StopPelvisTest("USER_STOP");
        private static string StopPelvisTest(string reason) => _pelvisTest.Stop(reason);

        private static bool RestorePelvisTestSafeState(StringBuilder evidence)
        {
            if (_legChain != null)
            {
                if (_legChain.TraceEnabled) TryAnkleCleanup("DEFENSIVE_LEG_TRACE_OFF", () => _legChain.SetTrace(false), evidence);
                if (!_legChain.NativeBodiesSafeOff) TryAnkleCleanup("DEFENSIVE_LEG_DRIVE_OFF", () => _legChain.SetDrive(false), evidence);
                if (_legChain.KneeAxisCalibrationCollecting) TryAnkleCleanup("DEFENSIVE_PCA_OFF", () => _legChain.SetKneeAxisCalibration(false), evidence);
                TryAnkleCleanup("FOOT_REFERENCE_BASELINE_RESTORE", () => _legChain.RestoreFootRelativeReferenceBaseline(), evidence);
            }
            if (_proxy != null)
            {
                if (_proxy.Report.TraceEnabled) TryAnkleCleanup("TORSO_TRACE_OFF", () => _proxy.SetTrace(false), evidence);
                TryAnkleCleanup("RECEIVER_QUIESCE", () => _proxy.QuiescePelvisReceiverPreserveTrace(), evidence);
                TryAnkleCleanup("RECEIVER_BASELINE_RESTORE", () => _proxy.SetPelvisReceiver(PelvisDynamicReceiverIsolation.IdleArm), evidence);
            }
            bool safe = _proxy != null && _proxy.PelvisReceiverSafeOff && (_legChain == null || (_legChain.NativeBodiesSafeOff &&
                !_legChain.TraceEnabled && _legChain.FootRelativeReferenceMatches(false)));
            evidence.AppendLine("torsoSafeOffAndBaselineRestored=" + safe + " | legsNativeSafeOff=" + (_legChain == null || _legChain.NativeBodiesSafeOff));
            if (!safe) evidence.AppendLine("UNSAFE_RECEIVER_FULL_ROLLBACK | " + RollbackInternal("RECEIVER_RESTORE_NOT_SAFE"));
            return safe;
        }

        public static string GetPelvisTestStatus(bool detailed = true) => _pelvisTest.Status(detailed) + "\nruntimeStage=" + _runtimeStage +
            "/cleanupPending=" + _cleanupPending + "/clockScope=TORSO_LEG_FOOT_SHARED_CONTEXT";

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
                return "ARL gravity：拒绝开启。v0.4.46只允许内部冻结的V04180/V04190在exact STANCE开启Foot/Lower gravity，V04200与V04210都固定B/ON；CCD固定ContinuousSpeculative，Foot接触裕量固定20mm，支撑面固定V04410静态Box，V04430 B膝部双体法向PD固定，两组仅对照V04450大腿自重OFF/逐级传递ON，V04260角阻尼B保持，V04290只读观测，V04310位置12、速度保持原值；通用arl_gravity(on)仍会同时影响torso/leg边界，因此drive活动时禁止。请先停止torso/leg drive。";
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
                return "ARL colliders：拒绝开启。v0.4.22仍冻结通用torso/leg collider开关；Foot contact只由V0470 exact STANCE lifecycle窄授权。请先停止torso/leg drive。";
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

            bool torsoTrace = _proxy.Report != null && _proxy.Report.TraceEnabled;
            bool legRequired = _legChain != null && _legChain.IsBuilt;
            bool legAnyTrace = legRequired && _legChain.TraceEnabled;
            bool legFullyTrace = legRequired && _legChain.TraceFullyEnabled;
            bool fullyActive = torsoTrace && (!legRequired || legFullyTrace);
            bool fullyInactive = !torsoTrace && (!legRequired || !legAnyTrace);

            if (enabled && fullyActive)
                return "ARL trace | enabled=True | alreadyActive=True | action=NO_OP";
            if (!enabled && fullyInactive)
                return "ARL trace | enabled=False | alreadyInactive=True | action=NO_OP";

            // A half-open torso/leg trace is never a valid steady state. It can be left behind by an exception or by
            // an old build. Normalize it before either a restart or an explicit stop so stale statistics cannot leak
            // into the next certification window. Stop only sides that are actually active; TwoBody.SetTrace(false)
            // is intentionally not used as an idempotent no-op because it finalizes its current statistics.
            string repaired = null;
            if (legRequired && (!fullyActive && (torsoTrace || legAnyTrace)))
            {
                string torsoRepair = torsoTrace ? _proxy.SetTrace(false) : "torso=already-off";
                string legRepair = legAnyTrace ? _legChain.SetTrace(false) : "leg=already-off";
                repaired = "inconsistentTraceRepaired=True | " + torsoRepair + " | " + legRepair;
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

            if (enabled && !_proxy.DriveEnabled)
                return "ARL trace：拒绝启动。请先 arl_drive(on)。";
            if (enabled && legRequired && !_legChain.DriveEnabled)
                return "ARL trace：拒绝启动。bilateral leg foundation 已创建时必须先 arl_leg_drive(on)，避免 torso/leg trace 半开状态。";
            if (enabled && legRequired && _legChain.KneeAxisCalibrationCollecting)
                return "ARL trace：拒绝启动。Knee axis calibration 正在收集，请先 arl_knee_axis_calibrate(off)。";

            if (enabled)
            {
                string torso = _proxy.SetTrace(true);
                if (_proxy.Report == null || !_proxy.Report.TraceEnabled)
                {
                    WriteAllReports();
                    string fail = "ARL trace：torso trace 启动失败；未启动 bilateral leg trace。 | " + torso;
                    _logger?.LogWarning("[ARL][TRACE_CONTROL] " + fail);
                    return fail;
                }

                string leg = legRequired ? _legChain.SetTrace(true) : "leg=not-built";
                if (legRequired && !_legChain.TraceFullyEnabled)
                {
                    string torsoRollback = _proxy.SetTrace(false);
                    WriteAllReports();
                    string fail = "ARL trace：bilateral leg trace 启动失败；torso trace 已回滚。 | leg=" + leg + " | torsoRollback=" + torsoRollback;
                    _logger?.LogWarning("[ARL][TRACE_CONTROL] " + fail);
                    return fail;
                }

                WriteAllReports();
                string result = torso + " | " + leg + (repaired != null ? " | " + repaired : string.Empty);
                _logger?.LogInfo("[ARL][TRACE_CONTROL] " + result);
                return result;
            }

            // Finalize the torso first because per-leg freeze certification consumes the finalized TwoBody freeze verdict.
            // The old leg-first order could leave the immediately emitted bilateral snapshot stale until a later
            // arl_leg_freeze_status call refreshed the child certifications. No physics step occurs between these calls.
            string torsoStop = torsoTrace ? _proxy.SetTrace(false) : "torso=already-off";
            string legStop = legAnyTrace && legRequired ? _legChain.SetTrace(false) : "leg=already-off";
            WriteAllReports();
            string stopped = torsoStop + " | " + legStop + (repaired != null ? " | " + repaired : string.Empty);
            _logger?.LogInfo("[ARL][TRACE_CONTROL] " + stopped);
            return stopped;
        }

        public static string WriteProxyReportAndGetSummary()
        {
            if (_proxy == null)
                return "ARL proxy：尚无报告。";
            WriteAllReports();
            string result = _proxy.Report.GetConsoleSummary() + " | fullReport=" + ProxyReportPath;
            if (_legChain != null)
                result += "\n" + _legChain.GetStatus() +
                          " | primaryReport=" + SingleLegReportPath +
                          (_legChain.BilateralEnabled ? " | secondaryReport=" + SecondaryLegReportPath + " | bilateralReport=" + BilateralLegReportPath : string.Empty) +
                          (_legChain.KneeSafetyReport != null ? " | kneeSafetyContextReport=" + KneeSafetyContextReportPath : string.Empty) +
                          " | footFoundationReport=" + FootFoundationReportPath;
            return result;
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
            if (_shutdown || _cleanupPending)
                return;
            _shutdown = true;
            RollbackInternal("shutdown:" + reason);
            _logger?.LogInfo("[ARL] Shutdown complete | reason=" + reason);
        }

        private static string RollbackInternal(string reason)
        {
            if (PelvisTestActive)
            {
                try { StopPelvisTest("ROLLBACK:" + reason); }
                catch (Exception e) { _logger?.LogError("[ARL][PELVIS_SNAPSHOT_BEFORE_ROLLBACK_FAILED] " + e.GetType().Name); }

            }
            if (AnkleTestActive)
            {
                try { StopAnkleTest("ROLLBACK:" + reason); }
                catch (Exception e) { _logger?.LogError("[ARL][ANKLE_SNAPSHOT_BEFORE_ROLLBACK_FAILED] " + e.GetType().Name); }
            }
            _stepClock.InvalidateBinding();
            if (_proxy == null && _legChain == null)
            {
                _cleanupPending = false; _pelvisTest?.Session.NoteCleanup(false); _ankleTest?.Session.NoteCleanup(false);
                LastRollbackSucceeded = true;
                return "ARL foundation rollback | success=True | cleared=False | reason=" + reason + " | detail=no-active-object";
            }

            bool success = true;
            string legResult = null;
            if (_legChain != null)
            {
                try
                {
                    legResult = _legChain.Rollback(reason);
                    if (!_legChain.LastRollbackSucceeded)
                        success = false;
                }
                catch (Exception e)
                {
                    success = false;
                    legResult = "ARL bilateral-leg rollback failed：" + e.GetType().Name + " / " + e.Message;
                }
                if (_legChain.LastRollbackSucceeded && success) _legChain = null;
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
                try
                {
                    torsoResult = _proxy.Rollback(reason);
                    if (!_proxy.LastRollbackSucceeded)
                        success = false;
                }
                catch (Exception e)
                {
                    success = false;
                    torsoResult = "ARL rollback failed：" + e.GetType().Name + " / " + e.Message;
                }
                if (_proxy.LastRollbackSucceeded && success) _proxy = null;
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
            return result;
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
