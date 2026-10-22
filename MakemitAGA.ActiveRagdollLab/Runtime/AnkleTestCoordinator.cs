using System;
using System.Text;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    internal struct AnkleRunEvidence
    {
        internal string Audit, Snapshot;
    }

    // Operations and readbacks stay in the legacy stack adapter. This coordinator owns
    // only the transaction, its evidence and the shared RunSession state machine.
    internal interface IAnkleRunPort
    {
        string Scene { get; }
        bool CleanupPending { get; }
        string ResolveArm(string requested);
        string Preflight();
        string Prepare(string arm);
        bool Ready { get; }
        string HealthIssue { get; }
        string ReadinessSummary();
        AnkleRunEvidence QuiesceAndCapture(bool started, string header, StringBuilder cleanup);
        bool Restore(bool started, StringBuilder cleanup);
        string LiveDetails();
        string Notice();
        void BeginNoticeSession();
        void AutomaticStop(string reason);
        void Emit(string text, bool warning, bool deferred);
        void WriteReports();
    }

    internal sealed class AnkleTestCoordinator
    {
        internal const string StopCommand = "arl_ankle_test(stop)";
        internal const string Plan = "PREFLIGHT>FROZEN_SELECTORS>LEG_DRIVE>CCD_OFFSET_SOLVERS>SUPPORT>SHARED_TRACE>RUNNING";
        internal const string TestInstructions = "ARL ankle v0.4.76：旧腿足组合的共享时钟与父级命令回归。重启游戏、同房同起点；arl_test_setup → arl_drive(on) → arl_knee_axis_calibrate(on)。同房走动采样，以arl_knee_axis_status确认双侧COLLECTING_QUALITY_READY_STOP_TO_FINALIZE后arl_knee_axis_calibrate(off)，确认双侧ready=True → arl_knee_axis_apply → arl_hip_stack(on)，确认READY；arl_ankle_test(b)，须started=True、phase=Running、sharedTraceReady=True、V04460ReceiverCaptured=True、auditFinite=True、sourceContextReady=True后arl_test_status；站立40秒、正常走动30秒、停下站立10秒；arl_ankle_test(stop) → arl_test_cleanup → arl_test_status各一次。拒绝启动或自动停止则直接收尾并保存完整日志。";
        private readonly IAnkleRunPort _port;
        internal readonly RunSession Session = new RunSession();
        internal string CompletedAudit { get; private set; }
        private PhysicsStepContext _lastStep;
        internal AnkleTestCoordinator(IAnkleRunPort port) { _port = port; }

        internal string Start(string requested)
        {
            if (string.Equals(requested, "stop", StringComparison.OrdinalIgnoreCase)) return Stop("USER_STOP");
            string arm = _port.ResolveArm(requested);
            if (arm == null) return Reject("INVALID_ARM | 当前回归使用arl_ankle_test(b)。");
            string failure = _port.Preflight();
            if (failure != null) return Reject(failure);
            if (Session.Active)
            {
                Stop("RESTART_REQUEST");
                failure = _port.Preflight();
                if (failure != null) return Reject(failure);
            }
            if (_port.CleanupPending || Session.CleanupPending) return Reject("CLEANUP_PENDING | 请先arl_test_cleanup。");
            Session.Begin(arm, _port.Scene, "FROZEN_LEG_FOOT_SELECTORS_AND_SHARED_TRACE");
            _lastStep = new PhysicsStepContext();
            try
            {
                _port.BeginNoticeSession();
                string details = _port.Prepare(arm);
                Session.AwaitReadiness("BOTH_LEG_DRIVES_AND_SHARED_TRACE");
                if (!_port.Ready) throw new InvalidOperationException("ANKLE_READY_READBACK_FAILED:" + _port.ReadinessSummary());
                Session.MarkRunning(); CompletedAudit = null;
                string result = "ARL ankle v0.4.76 | started=True | pending=False | runId=" + Session.RunId +
                    " | arm=" + Session.Arm + " | " + Session.Status() + " | " + _port.ReadinessSummary();
                _port.Emit("[ARL][ANKLE_TEST_BUNDLE][STARTED] " + result + " | " + details, true, false);
                try { _port.WriteReports(); }
                catch (Exception e) { _port.Emit("[ARL][REPORT_WRITE_FAILED] " + e.GetType().Name, true, false); }
                return result;
            }
            catch (Exception e)
            {
                string reason = "ANKLE_SETUP_EXCEPTION:" + e.GetType().Name + ":" + Clean(e.Message);
                string restored = Stop(reason);
                return "ARL ankle | started=False | reason=" + reason + " | " + restored;
            }
        }

        private string Reject(string reason)
        {
            string text = "ARL ankle | started=False | reason=" + reason;
            _port.Emit("[ARL][ANKLE_TEST_BUNDLE][NOT_STARTED] " + text, true, false);
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
                string issue = _port.HealthIssue;
                if (issue != null) Stop(issue);
            }
            catch (Exception e) { Stop("AUTO_ANKLE_LIFECYCLE_EXCEPTION:" + e.GetType().Name); }
        }

        internal string Stop(string reason)
        {
            if (!Session.BeginStop(reason))
            {
                string noOp = "ARL ankle stop | alreadyStopped=True | " + (Session.CompletedSummary ?? "completedRun=<none>");
                _port.Emit("[ARL][ANKLE_STOP_NO_OP] " + noOp, false, false); return noOp;
            }
            string header = "resultId=ANKLE/" + Plugin.SourceRevision + "/" + Session.RunId + " | sourceRevision=" + Plugin.SourceRevision + " | runId=" + Session.RunId +
                " | finishingArm=" + Session.Arm + " | scene=" + Clean(Session.Scene) + " | stopReason=" + Session.StopReason;
            bool automatic = reason.StartsWith("AUTO_", StringComparison.Ordinal);
            var cleanup = new StringBuilder();
            if (automatic)
            {
                try { _port.AutomaticStop(reason); }
                catch (Exception e) { cleanup.AppendLine("NOTICE_EXCEPTION=" + e.GetType().Name); }
            }
            AnkleRunEvidence evidence;
            try { evidence = _port.QuiesceAndCapture(Session.Started, header, cleanup); }
            catch (Exception e)
            {
                evidence = new AnkleRunEvidence { Audit = "resultScope=" + (Session.Started ? "COMPLETED_RUN" : "REJECTED_SETUP") +
                    " | " + header + " | UPPER_SELF_WEIGHT_TRANSFER_DATASET_QUALIFIED=False | KNEE_SUPPORT_NORMAL_REACTION_DATASET_QUALIFIED=False | reason=FINALIZATION_EXCEPTION:" + e.GetType().Name,
                    Snapshot = "FINALIZATION_EXCEPTION=" + e.GetType().Name + ":" + Clean(e.Message) };
            }
            bool safe = false, restoreReturned = false;
            try { safe = _port.Restore(Session.Started, cleanup); restoreReturned = true; }
            catch (Exception e) { cleanup.AppendLine("RESTORE_EXCEPTION=" + e.GetType().Name); }
            cleanup.AppendLine("legStopReadbackSafe=" + safe + " | torsoDriveRequiresTestCleanup=" + Session.Started);
            string scope = Session.Started ? "COMPLETED_RUN" : "REJECTED_SETUP";
            string summary = header + " | resultScope=" + scope + " | legStopReadbackSafe=" + safe;
            string snapshot = (Session.Started ? "[COMPLETED_ANKLE_TEST] " : "[REJECTED_ANKLE_SETUP] ") + header +
                "\n" + evidence.Audit + "\n" + evidence.Snapshot + "\n[RUNTIME_AT_FINALIZE]\n" + Session.Status() +
                "/plan=" + Plan + "\n" + _lastStep.Status() + "\n[NOTICE_AT_FINALIZE]\n" + _port.Notice() + "\n[STOP_CLEANUP]\n" + cleanup;
            if (Session.Started) CompletedAudit = evidence.Audit;
            Session.Finish(snapshot, summary, _port.CleanupPending || !safe);
            if (restoreReturned && !_port.CleanupPending) Session.NoteCleanup(false);
            _port.Emit("[ARL][ANKLE_TEST_BUNDLE][" + (Session.Started ? "STOP" : "REJECTED_SETUP") + "] " + snapshot +
                "\n[SESSION_AFTER_STOP]\n" + Session.Status(), !Session.Started, automatic);
            try { _port.WriteReports(); }
            catch (Exception e) { _port.Emit("[ARL][REPORT_WRITE_FAILED] " + e.GetType().Name, true, false); }
            return "ARL ankle v0.4.76 | stopped=True | started=" + Session.Started + " | " + summary;
        }

        internal string Status(bool detailed = true) => "ARL ankle v0.4.76 | stateScope=LIVE_STATE | ankleTestActive=" + Session.Active +
            " | " + Session.Status() + " | plan=" + Plan + " | sessionLastStep=" + _lastStep.Status() + (detailed ? "\n" + _port.LiveDetails() : "");
        private static string Clean(string text) => (text ?? "<none>").Replace('\r', ' ').Replace('\n', ' ');
    }
}
