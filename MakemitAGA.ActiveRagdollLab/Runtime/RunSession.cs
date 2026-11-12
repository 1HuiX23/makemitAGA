using System;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    internal enum RunPhase { Idle, Preparing, Warming, Running, Stopping, Completed, Rejected, CleanupFailed }

    // Lifecycle truth, independent of diagnostics and Unity objects. Completed evidence is
    // immutable text; an unsuccessful preparation must not erase the previous formal run.
    internal sealed class RunSession
    {
        internal RunPhase Phase { get; private set; } = RunPhase.Idle;
        internal int RunId { get; private set; }
        internal int PendingSteps { get; private set; }
        internal string Arm { get; private set; } = "<none>";
        internal string Scene { get; private set; }
        internal string StopReason { get; private set; }
        internal string StopTrigger { get; private set; }
        private string _firstAutomaticCause;
        internal bool Started { get; private set; }
        internal bool CleanupPending { get; private set; }
        internal string CompletedSnapshot { get; private set; }
        internal string CompletedSummary { get; private set; }
        internal string RejectedSnapshot { get; private set; }
        internal string RejectedSummary { get; private set; }
        // Summaries retain the finishing run identity, including after a later rejected attempt.
        // Full immutable evidence is emitted once at stop and remains available on explicit request.
        internal string EvidenceSummary() =>
            (CompletedSummary != null ? "\n[COMPLETED_RESULT] " + CompletedSummary + " | snapshotChars=" + (CompletedSnapshot?.Length ?? 0) : "") +
            (RejectedSummary != null ? "\n[REJECTED_RESULT] " + RejectedSummary + " | snapshotChars=" + (RejectedSnapshot?.Length ?? 0) : "");
        private string _preparingFor, _warmingFor;
        internal bool Active => Phase == RunPhase.Preparing || Phase == RunPhase.Warming || Phase == RunPhase.Running;
        internal bool OwnsScope => Active || Phase == RunPhase.Stopping;
        internal bool Pending => Phase == RunPhase.Preparing || Phase == RunPhase.Warming;

        internal void Begin(string arm, string scene, string preparingFor = "RECEIVER_SELECT_SOURCE_BIND_DRIVE")
        {
            if (OwnsScope || CleanupPending) throw new InvalidOperationException("SESSION_BUSY_OR_CLEANUP_PENDING");
            RunId++; Arm = arm; Scene = scene; Started = false; PendingSteps = 0; StopReason = null;
            StopTrigger = _firstAutomaticCause = null;
            _preparingFor = preparingFor;
            Phase = RunPhase.Preparing;
        }
        internal void AwaitReadiness(string dependency = "TWO_PASSIVE_NATIVE_SOLVES")
        {
            if (Phase != RunPhase.Preparing) throw new InvalidOperationException("SESSION_NOT_PREPARING");
            _warmingFor = dependency;
            Phase = RunPhase.Warming;
        }
        internal void UpdateReadinessDependency(string dependency)
        {
            if (Phase != RunPhase.Warming) throw new InvalidOperationException("SESSION_NOT_WARMING");
            _warmingFor = dependency;
        }
        internal void MarkRunning()
        {
            if (Phase != RunPhase.Warming) throw new InvalidOperationException("SESSION_NOT_WARMING");
            Started = true; Phase = RunPhase.Running;
            CompletedSnapshot = CompletedSummary = RejectedSnapshot = RejectedSummary = null;
        }
        internal void CountPendingStep() { if (Phase == RunPhase.Warming) PendingSteps++; }
        // Capture before the native owner turns selectors/drive off. Later health
        // checks observe that shutdown, not necessarily the original fault.
        // This is result provenance only; it never changes eligibility or timing.
        internal void RecordAutomaticStop(string owner, string reason)
        {
            if (!Active || _firstAutomaticCause != null) return;
            _firstAutomaticCause = "AUTO_FIRST_CAUSE:" + OneLine(owner) + ":" + OneLine(reason);
        }
        private static string OneLine(string value) => (value ?? "<none>").Replace('\r', ' ').Replace('\n', ' ');
        internal bool BeginStop(string reason)
        {
            if (!Active) return false;
            StopTrigger = reason; StopReason = _firstAutomaticCause ?? reason;
            Phase = RunPhase.Stopping; return true;
        }
        internal void Finish(string snapshot, string summary, bool cleanupPending)
        {
            if (Phase != RunPhase.Stopping) throw new InvalidOperationException("SESSION_NOT_STOPPING");
            if (Started) { CompletedSnapshot = snapshot; CompletedSummary = summary; }
            else { RejectedSnapshot = snapshot; RejectedSummary = summary; }
            CleanupPending = cleanupPending;
            Phase = cleanupPending ? RunPhase.CleanupFailed : Started ? RunPhase.Completed : RunPhase.Rejected;
        }
        internal void NoteCleanup(bool pending)
        {
            CleanupPending = pending;
            if (Phase == RunPhase.CleanupFailed && !pending) Phase = Started ? RunPhase.Completed : RunPhase.Rejected;
        }
        internal string WaitingFor => CleanupPending ? "RESOURCE_RELEASE" : Phase == RunPhase.Preparing ?
            _preparingFor : Phase == RunPhase.Warming ? _warmingFor : "<none>";
        internal string Status() => "runtimeSession=V04620/phase=" + Phase + "/runId=" + RunId +
            "/started=" + Started + "/pending=" + Pending + "/pendingSteps=" + PendingSteps +
            "/cleanupPending=" + CleanupPending + "/waitingFor=" + WaitingFor + "/stopReason=" + (StopReason ?? "<none>") + StopTriggerDetails;
        internal string StopTriggerDetails => StopTrigger != StopReason ? "/stopTrigger=" + StopTrigger : "";
    }
}
