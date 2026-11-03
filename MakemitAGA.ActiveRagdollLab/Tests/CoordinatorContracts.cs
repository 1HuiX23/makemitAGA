using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace ARL.Contracts
{
    internal static class CoordinatorContracts
    {
        internal static void Run()
        {
            Check.Run("pelvis readiness, capture order and idempotence", PelvisLifecycle);
            Check.Run("pelvis fault finalization and timeout", PelvisFaults);
            Check.Run("ankle completed/rejected evidence and idempotence", AnkleLifecycle);
            Check.Run("ankle fault finalization and clock", AnkleFaults);
        }

        private static bool Ordered(List<string> calls, string first, string second)
        {
            int firstIndex = calls.IndexOf(first), secondIndex = calls.IndexOf(second);
            return firstIndex >= 0 && secondIndex >= 0 && firstIndex < secondIndex;
        }

        private static void PelvisLifecycle()
        {
            var p = new PelvisPort(); var c = new PelvisTestCoordinator(p);
            c.Start("b");
            Check.That(c.Session.Pending && !c.Session.Started && p.TraceStarts == 0, "warmup cannot fabricate a started trace");
            c.OnFixed(RuntimeContracts.Step(1)); p.ReadyValue = true; c.OnFixed(RuntimeContracts.Step(2));
            Check.That(c.Session.Started && p.TraceStarts == 1, "native readiness commits one trace");
            c.Stop("USER_STOP"); string saved = c.Session.CompletedSnapshot;
            Check.That(Ordered(p.Calls, "capture", "restore"), "evidence capture occurs before baseline restoration");
            Check.That(saved.Contains("stopReason=USER_STOP") && c.Session.Phase == RunPhase.Completed, "formal completion retains stop identity");
            int restores = p.Restores; c.Stop("USER_STOP");
            Check.That(p.Restores == restores && c.Session.CompletedSnapshot == saved, "duplicate stop cannot repeat native cleanup or rewrite snapshot");
            p.PrepareFails = true; c.Start("b");
            Check.That(c.Session.RejectedSnapshot != null && c.Session.CompletedSnapshot == saved, "failed restart retains the completed result");
            Check.That(c.Status(false).Contains("COMPLETED_RESULT") && c.Status(false).Contains("REJECTED_RESULT"), "compact status exposes both result identities");
            var blocked = new PelvisPort { CleanupPendingValue = true };
            var rejected = new PelvisTestCoordinator(blocked); rejected.Start("b");
            Check.That(rejected.Session.RunId == 0 && !blocked.Calls.Contains("prepare"), "cleanup debt rejects before allocating new scope");
        }

        private static void PelvisFaults()
        {
            var p = new PelvisPort { ReadyValue = true, CaptureFails = true };
            var c = new PelvisTestCoordinator(p); c.Start("b"); c.Stop("USER_STOP");
            Check.That(p.Restores == 1 && c.Session.CompletedSnapshot.Contains("FINALIZATION_EXCEPTION"), "capture exception still restores and invalidates evidence explicitly");
            p = new PelvisPort { ReadyValue = true, RestoreFails = true }; c = new PelvisTestCoordinator(p); c.Start("b"); c.Stop("USER_STOP");
            Check.That(c.Session.CleanupPending && c.Session.Phase == RunPhase.CleanupFailed, "restore exception retains cleanup debt");
            int run = c.Session.RunId; c.Start("b");
            Check.That(c.Session.RunId == run, "failed restoration blocks the next run");
            p = new PelvisPort(); c = new PelvisTestCoordinator(p); c.Start("b");
            for (int i = 1; i <= 9; i++) c.OnFixed(RuntimeContracts.Step(i));
            Check.That(c.Session.StopReason == "AUTO_PASSIVE_SOLVE_TIMEOUT" && p.AutomaticNotices == 1, "warmup timeout finalizes exactly once");
            Check.That(c.Session.RejectedSnapshot != null && c.Session.CompletedSnapshot == null, "timeout before start is rejected setup, not completed physics");
            Check.That(Ordered(p.Calls, "notice", "capture"), "automatic first cause is recorded before capture");
            p = new PelvisPort { ReadyValue = true }; c = new PelvisTestCoordinator(p); c.Start("b");
            c.OnFixed(RuntimeContracts.Step(1)); c.OnFixed(RuntimeContracts.Step(3));
            Check.That(c.Session.StopReason == "AUTO_SESSION_CLOCK_DISCONTINUITY", "skipped shared Fixed identity stops the session");
            p = new PelvisPort { ReadyValue = true, WriteFails = true }; c = new PelvisTestCoordinator(p); c.Start("b"); c.Stop("USER_STOP");
            Check.That(c.Session.Phase == RunPhase.Completed && c.Session.CompletedSnapshot != null, "report write failure cannot erase completed in-memory evidence");
        }

        private static void AnkleLifecycle()
        {
            var p = new AnklePort(); var c = new AnkleTestCoordinator(p); c.Start("b");
            Check.That(c.Session.Started && !c.Session.Pending, "legacy ankle port preserves immediate readiness");
            c.Stop("USER_STOP"); string saved = c.Session.CompletedSnapshot;
            Check.That(Ordered(p.Calls, "capture", "restore") && c.CompletedAudit == "audit", "ankle audit captured before restoration");
            int restores = p.Restores; c.Stop("USER_STOP");
            Check.That(restores == p.Restores && c.Session.CompletedSnapshot == saved, "duplicate ankle stop is a no-op");
            p.PrepareFails = true; c.Start("b");
            Check.That(c.Session.CompletedSnapshot == saved && c.Session.RejectedSnapshot != null && c.CompletedAudit == "audit", "rejected ankle setup preserves the completed audit");
            var blocked = new AnklePort { CleanupPendingValue = true }; c = new AnkleTestCoordinator(blocked); c.Start("b");
            Check.That(c.Session.RunId == 0 && !blocked.Calls.Contains("prepare"), "ankle cleanup debt blocks preparation");
        }

        private static void AnkleFaults()
        {
            var p = new AnklePort { CaptureFails = true }; var c = new AnkleTestCoordinator(p); c.Start("b"); c.Stop("USER_STOP");
            Check.That(p.Restores == 1 && c.Session.CompletedSnapshot.Contains("FINALIZATION_EXCEPTION"), "ankle capture exception still restores");
            p = new AnklePort { RestoreFails = true }; c = new AnkleTestCoordinator(p); c.Start("b"); c.Stop("USER_STOP");
            Check.That(c.Session.CleanupPending && c.Session.Phase == RunPhase.CleanupFailed, "ankle restoration exception retains debt");
            p = new AnklePort(); c = new AnkleTestCoordinator(p); c.Start("b"); p.Health = "AUTO_LEG_DRIVE_ABORT"; c.OnFixed(RuntimeContracts.Step(1));
            Check.That(c.Session.StopReason == p.Health && c.Session.Phase == RunPhase.Completed && p.AutomaticNotices == 1, "health abort finalizes the formal run with its cause");
            p = new AnklePort(); c = new AnkleTestCoordinator(p); c.Start("b");
            var step = RuntimeContracts.Step(1); c.OnFixed(step); c.OnFixed(step);
            Check.That(c.Session.StopReason == "AUTO_SESSION_CLOCK_DISCONTINUITY", "reused Fixed packet cannot be credited twice");
            p = new AnklePort { WriteFails = true }; c = new AnkleTestCoordinator(p); c.Start("b"); c.Stop("USER_STOP");
            Check.That(c.Session.Phase == RunPhase.Completed && c.CompletedAudit == "audit", "ankle report IO failure preserves completion");
        }

        // Ports model only coordinator contracts. No native body, collision or solver is simulated.
        private sealed class PelvisPort : IPelvisRunPort
        {
            internal bool ReadyValue { get; set; }
            internal bool PrepareFails { get; set; }
            internal bool CaptureFails { get; set; }
            internal bool RestoreFails { get; set; }
            internal bool WriteFails { get; set; }
            internal bool CleanupPendingValue { get; set; }
            internal int TraceStarts, Restores, AutomaticNotices;
            internal readonly List<string> Calls = new List<string>();
            public string Scene => "contract-room";
            public string Scope => "CONTRACT";
            public string StopInstruction => "stop";
            public bool CleanupPending => CleanupPendingValue;
            public string ResolveArm(string arm) => arm == "b" ? "B" : null;
            public string Preflight() => null;
            public void Prepare(string arm) { Calls.Add("prepare"); if (PrepareFails) throw new InvalidOperationException("prepare"); }
            public void AdvancePreparation(PhysicsStepContext step) { }
            public bool Healthy => true;
            public string Issue => null;
            public bool Ready => ReadyValue;
            public bool Tracing { get; private set; }
            public string ReadinessSummary() => "ready=" + Ready;
            public void BeginTrace() { Tracing = true; TraceStarts++; }
            public string QuiesceAndCapture(bool started, string header, StringBuilder cleanup)
            { Calls.Add("capture"); Tracing = false; if (CaptureFails) throw new InvalidOperationException("capture"); return header; }
            public bool Restore(StringBuilder cleanup)
            { Calls.Add("restore"); Restores++; if (RestoreFails) throw new InvalidOperationException("restore"); return true; }
            public string LiveDetails() => "live";
            public string Notice() => "notice";
            public void BeginNoticeSession(string stopCommand) { }
            public void AutomaticStop(string reason) { Calls.Add("notice"); AutomaticNotices++; }
            public void Emit(string text, bool warning, bool deferred) { }
            public void ShowStarted(string text) { }
            public void WriteReports() { if (WriteFails) throw new IOException("report"); }
        }

        private sealed class AnklePort : IAnkleRunPort
        {
            internal bool PrepareFails { get; set; }
            internal bool CaptureFails { get; set; }
            internal bool RestoreFails { get; set; }
            internal bool WriteFails { get; set; }
            internal bool CleanupPendingValue { get; set; }
            internal string Health { get; set; }
            internal int Restores, AutomaticNotices;
            internal readonly List<string> Calls = new List<string>();
            public string Scene => "contract-room";
            public bool CleanupPending => CleanupPendingValue;
            public string ResolveArm(string arm) => arm == "b" ? "B" : null;
            public string Preflight() => null;
            public string Prepare(string arm)
            { Calls.Add("prepare"); if (PrepareFails) throw new InvalidOperationException("prepare"); return "prepared"; }
            public bool Ready => true;
            public string HealthIssue => Health;
            public string ReadinessSummary() => "ready";
            public AnkleRunEvidence QuiesceAndCapture(bool started, string header, StringBuilder cleanup)
            { Calls.Add("capture"); if (CaptureFails) throw new InvalidOperationException("capture"); return new AnkleRunEvidence { Audit = "audit", Snapshot = header }; }
            public bool Restore(bool started, StringBuilder cleanup)
            { Calls.Add("restore"); Restores++; if (RestoreFails) throw new InvalidOperationException("restore"); return true; }
            public string LiveDetails() => "live";
            public string Notice() => "notice";
            public void BeginNoticeSession() { }
            public void AutomaticStop(string reason) { AutomaticNotices++; }
            public void Emit(string text, bool warning, bool deferred) { }
            public void WriteReports() { if (WriteFails) throw new IOException("report"); }
        }
    }
}
