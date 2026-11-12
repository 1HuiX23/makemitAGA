using System;
using System.Text;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    internal sealed class RunPreparationRejectedException : InvalidOperationException
    {
        internal RunPreparationRejectedException(string reason) : base(reason) { }
    }
    // A typed boundary around the existing torso implementation. The coordinator owns
    // sequencing; the port owns module-specific readiness, native operations and evidence.
    internal interface IPelvisRunPort
    {
        string Scene { get; }
        string ResolveArm(string requestedArm);
        string Scope { get; }
        string StopInstruction { get; }
        bool CleanupPending { get; }
        string Preflight();
        void Prepare(string arm);
        void AdvancePreparation(PhysicsStepContext step);
        bool Healthy { get; }
        string Issue { get; }
        bool Ready { get; }
        // A non-null dependency owns a bounded preparation deadline in its native port.
        string PreparationDependency { get; }
        bool Tracing { get; }
        string ReadinessSummary();
        void BeginTrace();
        string QuiesceAndCapture(bool started, string header, StringBuilder cleanup);
        bool Restore(StringBuilder cleanup);
        string LiveDetails();
        string Notice();
        void BeginNoticeSession(string stopCommand);
        void AutomaticStop(string reason);
        void Emit(string text, bool warning, bool deferred);
        void ShowStatus(string text);
        void WriteReports();
    }

    // The torso consumer of RunSession, also used by the legacy ankle coordinator.
    // Native ports remain separate. No forces, controller gains or report gates live here.
    internal sealed class PelvisTestCoordinator
    {
        internal const string Plan = "TORSO_SAMPLE> TORSO_CONTROL_AND_PARENT_PUBLISH> CLOCKED_LEGS_IF_PRESENT> SESSION_READINESS";
        internal const string TestInstructions = "ARL pelvis v0.4.128：骨盆独立基线使用arl_pelvis_test_setup → arl_pelvis_test(b)。自动[STARTED]后才计时；结束arl_pelvis_test(stop)、arl_test_cleanup、arl_test_status各一次。动态腿足无接触用arl_pelvis_leg_test，无载荷接触用arl_pelvis_contact_test，足部自重接触用arl_pelvis_foot_load_test，小腿自重用arl_pelvis_lower_load_test，膝部耦合回归用arl_pelvis_knee_test，大腿自重回归用arl_pelvis_upper_load_test，当前Hip同输入只读对照先测arl_pelvis_hip_test(b)，重启后从建模到收尾全程客厅Main；两臂固定126 B膝足联合输入并实际执行127 A统一q，B只在实际命令完成后计算原预算内的同q向上与角请求交换候选；candidateExecuted=False为预期，A保留为关闭审计的回归入口；arl_test_setup先等待[SETUP_READY]再drive/PCA，正式Hip run前分别等待预计初态与接入后的实际刚体检查，WAIT_INITIAL_GEOMETRY/WAIT_SETTLED_GEOMETRY期间继续站立；[NOT_STARTED]时清理后重新建模，勿继续正式测试。原接触/承重准入、缺口反馈方向/制动保护、安全线和控制预算保持。";
        private readonly IPelvisRunPort _port;
        internal readonly RunSession Session = new RunSession();
        private PhysicsStepContext _lastStep;
        internal PelvisTestCoordinator(IPelvisRunPort port) { _port = port; }

        internal string Start(string requestedArm)
        {
            if (string.Equals(requestedArm, "stop", StringComparison.OrdinalIgnoreCase)) return Stop("USER_STOP");
            string arm = _port.ResolveArm(requestedArm);
            if (arm == null) return Reject("RETIRED_OR_INVALID_ARM | 当前Hip用arl_pelvis_hip_test(a|b)，A=原Knee输入并观察候选，B=有界地面相容Knee输入；两臂共同采用124 B法向反馈、匹配缺口回算及原参考/承重/足部背景，保留原保护、接触入口与安全门，旧完整arm标识已退休。大腿自重用arl_pelvis_upper_load_test(b)，膝部法向耦合用arl_pelvis_knee_test(b)，小腿自重用arl_pelvis_lower_load_test(b)，足部自重用arl_pelvis_foot_load_test(b)，无载荷接触用arl_pelvis_contact_test(b)，无接触用arl_pelvis_leg_test(a|b)，独立骨盆用arl_pelvis_test(b)。");
            string failure = _port.Preflight();
            if (failure != null) return Reject(failure);
            if (Session.Active)
            {
                Stop("RESTART_REQUEST");
                failure = _port.Preflight();
                if (failure != null) return Reject(failure);
            }
            if (_port.CleanupPending || Session.CleanupPending) return Reject("CLEANUP_PENDING | 请先arl_test_cleanup。");
            Session.Begin(arm, _port.Scene); _lastStep = new PhysicsStepContext();
            try
            {
                _port.BeginNoticeSession(_port.StopInstruction);
                _port.Prepare(arm);
                Session.AwaitReadiness();
                if (_port.Ready) BeginTrace();
                string result = "ARL pelvis v0.4.128 | started=" + Session.Started + " | pending=" + Session.Pending +
                    " | scope=" + _port.Scope + " | runId=" + Session.RunId + " | testArm=" + Session.Arm + " | " + _port.ReadinessSummary() + " | " + Session.Status();
                _port.Emit("[ARL][PELVIS_TEST_BUNDLE][" + (Session.Started ? "START_ACK" : "PENDING_NATIVE_SOLVES") + "] " + result, false, false);
                return result;
            }
            catch (RunPreparationRejectedException e) { return RejectPreparation(e.Message); }
            catch (Exception e)
            {
                string reason = "RECEIVER_SETUP_EXCEPTION:" + e.GetType().Name + ":" + Clean(e.Message);
                Stop(reason);
                return "ARL pelvis | started=False | " + reason + " | 保存本次日志。";
            }
        }
        private string Reject(string reason)
        {
            string result = "ARL pelvis | started=False | reason=" + reason;
            _port.Emit("[ARL][PELVIS_TEST_BUNDLE][NOT_STARTED] " + result, true, false); return result;
        }
        private void BeginTrace()
        {
            _port.BeginTrace();
            if (!_port.Tracing) throw new InvalidOperationException("RECEIVER_TRACE_START_FAILED");
            Session.MarkRunning();
            string text = "ARL pelvis [STARTED] | started=True | pending=False | runId=" + Session.RunId +
                " | scope=" + _port.Scope + " | testArm=" + Session.Arm + " | " + _port.ReadinessSummary() + " | " + Session.Status();
            _port.Emit("[ARL][PELVIS_TEST_BUNDLE][STARTED] " + text, false, false);
            _port.ShowStatus(text);
        }
        private string RejectPreparation(string reason)
        {
            Stop("SETUP_REJECTED:" + Clean(reason));
            string text = "ARL pelvis [NOT_STARTED] | started=False | reason=" + Clean(reason) +
                " | 初始化未通过，已停止准备；先arl_test_cleanup，再站稳后重新arl_test_setup及PCA。";
            _port.Emit("[ARL][PELVIS_TEST_BUNDLE][NOT_STARTED] " + text, true, false);
            _port.ShowStatus(text);
            return text;
        }
        internal void OnFixed(PhysicsStepContext step)
        {
            if (!Session.Active) return;
            try
            {
                if (!step.Valid || (_lastStep.Valid && (step.Epoch != _lastStep.Epoch || step.Sequence != _lastStep.Sequence + 1)))
                { Stop("AUTO_SESSION_CLOCK_DISCONTINUITY"); return; }
                _lastStep = step;
                if (!_port.Healthy) { Stop("AUTO_RECEIVER_OR_SCOPE_ABORT:" + (_port.Issue ?? "DRIVE_OR_LEG_SCOPE_LOST")); return; }
                if (!Session.Started)
                {
                    Session.CountPendingStep();
                    _port.AdvancePreparation(step);
                    string preparation = _port.PreparationDependency;
                    Session.UpdateReadinessDependency(preparation ?? "TWO_PASSIVE_NATIVE_SOLVES");
                    if (_port.Ready) BeginTrace();
                    else if (preparation == null && Session.PendingSteps > 8) Stop("AUTO_PASSIVE_SOLVE_TIMEOUT");
                }
                else if (!_port.Tracing) Stop("AUTO_TORSO_TRACE_LOST");
            }
            catch (RunPreparationRejectedException e) { RejectPreparation(e.Message); }
            catch (Exception e) { Stop("AUTO_RECEIVER_LIFECYCLE_EXCEPTION:" + e.GetType().Name + ":" + Clean(e.Message)); }
        }
        internal string Stop(string reason)
        {
            if (!Session.BeginStop(reason)) return "ARL pelvis stop | alreadyStopped=True | " + (Session.CompletedSummary ?? "completedRun=<none>");
            string header = "resultId=PELVIS/" + Plugin.SourceRevision + "/" + Session.RunId + " | sourceRevision=" + Plugin.SourceRevision + " | runId=" + Session.RunId + " | finishingArm=" + Session.Arm +
                " | scope=" + _port.Scope + " | started=" + Session.Started + " | scene=" + Clean(Session.Scene) + " | stopReason=" + Session.StopReason + Session.StopTriggerDetails;
            bool automatic = reason.StartsWith("AUTO_", StringComparison.Ordinal);
            var cleanup = new StringBuilder();
            if (automatic)
            {
                try { _port.AutomaticStop(reason); }
                catch (Exception e) { cleanup.AppendLine("NOTICE_EXCEPTION=" + e.GetType().Name); }
            }
            string snapshot;
            try { snapshot = _port.QuiesceAndCapture(Session.Started, header, cleanup); }
            catch (Exception e) { snapshot = (Session.Started ? "resultScope=COMPLETED_RUN" : "resultScope=REJECTED_SETUP") + " | " + header + " | RECEIVER_EXPOSURE_DATASET_READY=False | FINALIZATION_EXCEPTION=" + e.GetType().Name; }
            snapshot += "\n[RUNTIME_AT_FINALIZE]\n" + Session.Status() + "/plan=" + Plan + "\n" + _lastStep.Status();
            bool safe = false, restoreReturned = false;
            try { safe = _port.Restore(cleanup); restoreReturned = true; }
            catch (Exception e) { cleanup.AppendLine("RESTORE_EXCEPTION=" + e.GetType().Name); }
            string summary = header + " | torsoSafeOffAndBaselineRestored=" + safe;
            string complete = (Session.Started ? "[COMPLETED_PELVIS_TEST]\n" : "[REJECTED_PELVIS_SETUP]\n") + snapshot +
                "\n[NOTICE_AT_FINALIZE]\n" + _port.Notice() + "\n[STOP_CLEANUP]\n" + cleanup;
            Session.Finish(complete, summary, _port.CleanupPending || !safe);
            // A successful defensive full rollback resolves pending ownership, while preserving
            // the baseline-restore failure in immutable evidence.
            if (restoreReturned && !_port.CleanupPending) Session.NoteCleanup(false);
            _port.Emit("[ARL][PELVIS_TEST_BUNDLE][STOP] " + complete + "\n[SESSION_AFTER_STOP]\n" + Session.Status(), false, automatic);
            try { _port.WriteReports(); }
            catch (Exception e) { _port.Emit("[ARL][REPORT_WRITE_FAILED] " + e.GetType().Name, true, false); }
            return "ARL pelvis v0.4.128 | stopped=True | " + summary;
        }
        internal string Status(bool detailed = true)
        {
            string live = "ARL pelvis v0.4.128 | stateScope=LIVE_STATE | scope=" + _port.Scope + " | pelvisTestActive=" + Session.Active +
                " | started=" + Session.Started + " | pending=" + Session.Pending + " | testArm=" + Session.Arm + " | " + Session.Status() +
                " | plan=" + Plan + " | " + _lastStep.Status();
            if (!detailed) return live + Session.EvidenceSummary();
            if (Session.Active) return live + "\n" + _port.LiveDetails();
            return live + (Session.RejectedSnapshot != null ? "\n[LAST_REJECTED_PELVIS_SETUP]\n" + Session.RejectedSnapshot : "") +
                (Session.CompletedSnapshot != null ? "\n" + Session.CompletedSnapshot : "");
        }
        private static string Clean(string value) => (value ?? "<none>").Replace('\r', ' ').Replace('\n', ' ');
    }
}
