using System;
using System.Text;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
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
        void ShowStarted(string text);
        void WriteReports();
    }

    // The torso consumer of RunSession, also used by the legacy ankle coordinator.
    // Native ports remain separate. No forces, controller gains or report gates live here.
    internal sealed class PelvisTestCoordinator
    {
        internal const string Plan = "TORSO_SAMPLE> TORSO_CONTROL_AND_PARENT_PUBLISH> CLOCKED_LEGS_IF_PRESENT> SESSION_READINESS";
        internal const string TestInstructions = "ARL pelvis v0.4.89：骨盆独立基线使用arl_pelvis_test_setup → arl_pelvis_test(b)。自动[STARTED]后才计时；结束arl_pelvis_test(stop)、arl_test_cleanup、arl_test_status各一次。动态腿足无接触用arl_pelvis_leg_test，无载荷接触用arl_pelvis_contact_test，足部自重接触用arl_pelvis_foot_load_test，小腿自重用arl_pelvis_lower_load_test，膝部耦合回归用arl_pelvis_knee_test，当前大腿自重用arl_pelvis_upper_load_test。";
        private readonly IPelvisRunPort _port;
        internal readonly RunSession Session = new RunSession();
        private PhysicsStepContext _lastStep;
        internal PelvisTestCoordinator(IPelvisRunPort port) { _port = port; }

        internal string Start(string requestedArm)
        {
            if (string.Equals(requestedArm, "stop", StringComparison.OrdinalIgnoreCase)) return Stop("USER_STOP");
            string arm = _port.ResolveArm(requestedArm);
            if (arm == null) return Reject("RETIRED_OR_INVALID_ARM | 当前大腿自重用arl_pelvis_upper_load_test(b)，膝部法向耦合用arl_pelvis_knee_test(b)，小腿自重用arl_pelvis_lower_load_test(b)，足部自重用arl_pelvis_foot_load_test(b)，无载荷接触用arl_pelvis_contact_test(b)，无接触用arl_pelvis_leg_test(a|b)，独立骨盆用arl_pelvis_test(b)。");
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
                string result = "ARL pelvis v0.4.89 | started=" + Session.Started + " | pending=" + Session.Pending +
                    " | scope=" + _port.Scope + " | runId=" + Session.RunId + " | testArm=" + Session.Arm + " | " + _port.ReadinessSummary() + " | " + Session.Status();
                _port.Emit("[ARL][PELVIS_TEST_BUNDLE][" + (Session.Started ? "START_ACK" : "PENDING_NATIVE_SOLVES") + "] " + result, false, false);
                return result;
            }
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
            _port.ShowStarted(text);
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
                    if (_port.Ready) BeginTrace();
                    else if (Session.PendingSteps > 8) Stop("AUTO_PASSIVE_SOLVE_TIMEOUT");
                }
                else if (!_port.Tracing) Stop("AUTO_TORSO_TRACE_LOST");
            }
            catch (Exception e) { Stop("AUTO_RECEIVER_LIFECYCLE_EXCEPTION:" + e.GetType().Name + ":" + Clean(e.Message)); }
        }
        internal string Stop(string reason)
        {
            if (!Session.BeginStop(reason)) return "ARL pelvis stop | alreadyStopped=True | " + (Session.CompletedSummary ?? "completedRun=<none>");
            string header = "resultId=PELVIS/" + Plugin.SourceRevision + "/" + Session.RunId + " | sourceRevision=" + Plugin.SourceRevision + " | runId=" + Session.RunId + " | finishingArm=" + Session.Arm +
                " | scope=" + _port.Scope + " | started=" + Session.Started + " | scene=" + Clean(Session.Scene) + " | stopReason=" + Session.StopReason;
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
            return "ARL pelvis v0.4.89 | stopped=True | " + summary;
        }
        internal string Status(bool detailed = true)
        {
            string live = "ARL pelvis v0.4.89 | stateScope=LIVE_STATE | scope=" + _port.Scope + " | pelvisTestActive=" + Session.Active +
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
