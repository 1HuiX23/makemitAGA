using System;
using MakemitAGA.ActiveRagdollLab.Physics;
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
        internal const string TestInstructions = "ARL v0.4.166。本轮测 B，在 Main 平地从 arl_test_setup 开始。SETUP_READY 后 arl_drive(on) → arl_knee_axis_calibrate(on)，自然行走和转身采样。arl_knee_axis_status 双侧 COLLECTING_QUALITY_READY_STOP_TO_FINALIZE 后 arl_knee_axis_calibrate(off)，双侧 ready=True 后 arl_knee_axis_apply → arl_hip_stack(on)。arl_hip_stack_status 双侧 READY 后静止约 1 秒，执行 arl_pelvis_hip_test(b)。等待几何时静止，见 STARTED 与对应实验臂后计时。静止 40 秒、自然走停转身 40 秒（直线优先）、静止 20 秒。arl_pelvis_hip_test(stop) → arl_test_cleanup → arl_test_status。NOT_STARTED 或自动停止后直接 cleanup、status。revision=V041660_LEFT_FOOT_HORIZONTAL_LEAD_20261120。";
        private readonly IPelvisRunPort _port;
        internal readonly RunSession Session = new RunSession();
        private PhysicsStepContext _lastStep;
        internal PelvisTestCoordinator(IPelvisRunPort port) { _port = port; }

        internal string Start(string requestedArm)
        {
            if (string.Equals(requestedArm, "stop", StringComparison.OrdinalIgnoreCase)) return Stop("USER_STOP");
            string arm = _port.ResolveArm(requestedArm);
            if (arm == null) return Reject("RETIRED_OR_INVALID_ARM | 参数与当前测试入口不匹配。请用 arl_help 核对该入口支持的实验臂。");
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
                string result = "ARL pelvis v0.4.166 | started=" + Session.Started + " | pending=" + Session.Pending +
                    " | scope=" + _port.Scope + " | runId=" + Session.RunId + " | testArm=" + Session.Arm + " | " + _port.ReadinessSummary() + " | " + Session.Status();
                Emit("[ARL][PELVIS_TEST_BUNDLE][" + (Session.Started ? "START_ACK" : "PENDING_NATIVE_SOLVES") + "] " + result, false, false);
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
            Emit("[ARL][PELVIS_TEST_BUNDLE][NOT_STARTED] " + result, true, false); return result;
        }
        private void BeginTrace()
        {
            _port.BeginTrace();
            if (!_port.Tracing) throw new InvalidOperationException("RECEIVER_TRACE_START_FAILED");
            Session.MarkRunning();
            LandingObservationTiming.Run.Begin(Session.Arm);
            string observation = LandingObservationTiming.Run.Running ? " | landingObservation=" + LandingObservationTiming.ModeName(LandingObservationTiming.Run.Compact) +
                " | eventHistory=" + LandingObservationTiming.Run.History + " | rightPrecontactArm=" +
                RightFootPrecontact.ForArm(Session.Arm) + " | leftReleaseProbe=" + LandingObservationTiming.Run.LeftReleaseProbe : "";
            string text = "ARL pelvis [STARTED] | started=True | pending=False | runId=" + Session.RunId + observation +
                " | scope=" + _port.Scope + " | testArm=" + Session.Arm + " | " + _port.ReadinessSummary() + " | " + Session.Status();
            Emit("[ARL][PELVIS_TEST_BUNDLE][STARTED] " + text, false, false);
            _port.ShowStatus(text);
        }
        private string RejectPreparation(string reason)
        {
            Stop("SETUP_REJECTED:" + Clean(reason));
            string text = "ARL pelvis [NOT_STARTED] | started=False | reason=" + Clean(reason) +
                " | 初始化未通过，已停止准备；先arl_test_cleanup，再站稳后重新arl_test_setup及PCA。";
            Emit("[ARL][PELVIS_TEST_BUNDLE][NOT_STARTED] " + text, true, false);
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
            bool measured = Session.Started && LandingObservationTiming.Run.Seal();
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
            if (measured)
            {
                try { snapshot += LandingObservationTiming.Run.Summary(); }
                catch (Exception e) { cleanup.AppendLine("LANDING_COST_SUMMARY_EXCEPTION=" + e.GetType().Name); }
            }
            string notice;
            try { notice = _port.Notice(); }
            catch (Exception e) { notice = "NOTICE_SNAPSHOT_EXCEPTION=" + e.GetType().Name; }
            string summary = header + " | torsoSafeOffAndBaselineRestored=" + safe;
            string complete = (Session.Started ? "[COMPLETED_PELVIS_TEST]\n" : "[REJECTED_PELVIS_SETUP]\n") + snapshot +
                "\n[NOTICE_AT_FINALIZE]\n" + notice + "\n[STOP_CLEANUP]\n" + cleanup;
            Session.Finish(complete, summary, _port.CleanupPending || !safe);
            // A successful defensive full rollback resolves pending ownership, while preserving
            // the baseline-restore failure in immutable evidence.
            if (restoreReturned && !_port.CleanupPending) Session.NoteCleanup(false);
            Emit("[ARL][PELVIS_TEST_BUNDLE][STOP] " + complete + "\n[SESSION_AFTER_STOP]\n" + Session.Status(), false, automatic);
            try { _port.WriteReports(); }
            catch (Exception e) { Emit("[ARL][REPORT_WRITE_FAILED] " + e.GetType().Name, true, false); }
            return "ARL pelvis v0.4.166 | stopped=True | " + summary;
        }
        internal string Status(bool detailed = true)
        {
            string live = "ARL pelvis v0.4.166 | stateScope=LIVE_STATE | scope=" + _port.Scope + " | pelvisTestActive=" + Session.Active +
                " | started=" + Session.Started + " | pending=" + Session.Pending + " | testArm=" + Session.Arm + " | " + Session.Status() +
                " | plan=" + Plan + " | " + _lastStep.Status();
            if (!detailed) return live + Session.EvidenceSummary();
            if (Session.Active) return live + "\n" + _port.LiveDetails();
            return live + (Session.RejectedSnapshot != null ? "\n[LAST_REJECTED_PELVIS_SETUP]\n" + Session.RejectedSnapshot : "") +
                (Session.CompletedSnapshot != null ? "\n" + Session.CompletedSnapshot : "");
        }
        // Presentation is optional. A failed logger must not be caught as a
        // preparation/control failure, or escape after native restoration completed.
        private void Emit(string text, bool warning, bool deferred)
        {
            try { _port.Emit(text, warning, deferred); }
            catch { } // Session snapshots remain available through explicit status.
        }
        private static string Clean(string value) => (value ?? "<none>").Replace('\r', ' ').Replace('\n', ' ');
    }
}
