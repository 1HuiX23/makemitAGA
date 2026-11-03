using System;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace ARL.Contracts
{
    internal static class RuntimeContracts
    {
        internal static PhysicsStepContext Step(int sequence) => new PhysicsStepContext
        { Epoch = 1, Sequence = sequence, SourceSequence = 1, FixedTime = sequence / 60f, Dt = 1f / 60f, UnscaledTime = sequence / 60f };

        internal static void Run()
        {
            Check.Run("clock identity and invalid-input transaction", Clock);
            Check.Run("cursor delivery and binding boundaries", Cursor);
            Check.Run("session lifecycle and retained evidence", Session);
            Check.Run("session illegal transitions", IllegalTransitions);
            Check.Run("first-cause notice and bounded delivery", Notice);
        }

        private static void Clock()
        {
            var clock = new RuntimeStepClock();
            var source = clock.CaptureSource(1f, 0f);
            var step = clock.CaptureFixed(1f, .02f, 1f);
            Check.That(source.Valid && step.Valid && step.SourceSequence == source.Sequence, "render and Fixed share source identity");
            var next = clock.CaptureFixed(1f, .02f, 1f);
            Check.That(next.Sequence == 2 && !step.Matches(next), "equal rounded times remain distinct steps");
            Check.That(clock.LastSource.Matches(source), "Fixed capture does not fabricate a new source sample");
            foreach (float invalid in new[] { float.NaN, float.PositiveInfinity, float.NegativeInfinity })
            {
                Check.Throws<InvalidOperationException>(() => clock.CaptureSource(invalid, .02f), "nonfinite source time rejected");
                Check.Throws<InvalidOperationException>(() => clock.CaptureSource(2f, invalid), "nonfinite render dt rejected");
                Check.Throws<InvalidOperationException>(() => clock.CaptureFixed(2f, invalid, 2f), "nonfinite solve dt rejected");
                Check.Throws<InvalidOperationException>(() => clock.CaptureFixed(2f, .02f, invalid), "nonfinite unscaled time rejected");
            }
            Check.Throws<InvalidOperationException>(() => clock.CaptureSource(.9f, .02f), "backward source rejected");
            Check.Throws<InvalidOperationException>(() => clock.CaptureSource(2f, -.02f), "negative render dt rejected");
            Check.Throws<InvalidOperationException>(() => clock.CaptureFixed(2f, 0f, 2f), "zero solve dt rejected");
            Check.Throws<InvalidOperationException>(() => clock.CaptureFixed(.9f, .02f, 2f), "backward Fixed time rejected");
            Check.That(clock.LastSource.Matches(source) && clock.LastFixed.Matches(next), "rejections do not consume or publish identities");
            clock.InvalidateBinding();
            Check.That(!clock.LastSource.Valid && !clock.LastFixed.Valid, "binding invalidation clears held packets");
            var rebound = clock.CaptureSource(.5f, .02f);
            Check.That(rebound.Epoch == source.Epoch + 1 && rebound.Sequence == 1, "rebind restarts sequence within a new epoch");
            var beforeSource = new RuntimeStepClock().CaptureFixed(0f, .02f, 0f);
            Check.That(beforeSource.Valid && beforeSource.SourceSequence == 0, "pre-source Fixed packet remains representable");
        }

        private static void Cursor()
        {
            var clock = new RuntimeStepClock();
            var cursor = new StepContextCursor();
            var source = clock.CaptureSource(1f, .01f);
            var step = clock.CaptureFixed(1f, .02f, 1f);
            cursor.AcceptSource(source); cursor.AcceptFixed(step);
            Check.Throws<InvalidOperationException>(() => cursor.AcceptSource(source), "source cannot be consumed twice");
            Check.Throws<InvalidOperationException>(() => cursor.AcceptFixed(step), "Fixed packet cannot be consumed twice");
            var skipped = step; skipped.Sequence += 2;
            Check.Throws<InvalidOperationException>(() => cursor.AcceptFixed(skipped), "unobserved Fixed interval cannot be skipped silently");
            var mismatch = step; mismatch.Sequence++; mismatch.SourceSequence++;
            Check.Throws<InvalidOperationException>(() => cursor.AcceptFixed(mismatch), "Fixed packet must match held source identity");
            Check.That(cursor.Fixed.Matches(step) && cursor.Source.Matches(source), "rejected deliveries leave cursor unchanged");
            cursor.AcceptSource(clock.CaptureSource(1f, .01f));
            cursor.AcceptFixed(clock.CaptureFixed(1f, .02f, 1f));
            Check.That(cursor.SourceMatches(cursor.Fixed), "consecutive packets accept equal rounded timestamps");
            clock.InvalidateBinding();
            Check.Throws<InvalidOperationException>(() => cursor.AcceptSource(clock.CaptureSource(2f, .01f)), "old owner cannot silently consume a new binding");
            var lateJoin = new StepContextCursor();
            lateJoin.AcceptFixed(Step(77));
            Check.That(lateJoin.Fixed.Sequence == 77, "a new owner may join an existing sequence");
        }

        private static void Session()
        {
            var s = new RunSession();
            Check.That(!s.Active && !s.OwnsScope && !s.BeginStop("IDLE"), "idle stop is a no-op");
            s.Begin("A", "room");
            Check.That(s.Active && s.Pending && !s.Started, "preparation owns scope without formal-run evidence");
            Check.Throws<InvalidOperationException>(() => s.Begin("B", "room"), "parallel run rejected");
            s.CountPendingStep(); Check.That(s.PendingSteps == 0, "preparing does not count native warmup steps");
            s.AwaitReadiness(); s.CountPendingStep(); s.MarkRunning();
            Check.That(s.Started && !s.Pending && s.PendingSteps == 1, "readiness commits the run once");
            Check.That(s.BeginStop("FIRST") && !s.BeginStop("SECOND") && s.StopReason == "FIRST", "first finalizer owns stop reason");
            Check.That(s.OwnsScope && !s.Active, "stopping still owns the scope");
            s.Finish("complete", "complete summary", false);
            s.Begin("B", "room"); s.BeginStop("SETUP_FAILED"); s.Finish("rejected", "rejected summary", true);
            Check.That(s.CompletedSnapshot == "complete" && s.RejectedSnapshot == "rejected", "failed preparation preserves the previous completed run");
            Check.That(s.WaitingFor == "RESOURCE_RELEASE" && s.Phase == RunPhase.CleanupFailed, "cleanup failure keeps explicit ownership debt");
            Check.Throws<InvalidOperationException>(() => s.Begin("C", "room"), "cleanup debt blocks next run");
            s.NoteCleanup(false);
            Check.That(s.Phase == RunPhase.Rejected && s.CompletedSnapshot == "complete", "retry resolves phase without rewriting evidence");
            s.Begin("C", "room"); s.AwaitReadiness();
            Check.That(s.CompletedSnapshot == "complete", "pending preparation alone does not erase history");
            s.MarkRunning();
            Check.That(s.RunId == 3 && s.CompletedSnapshot == null && s.RejectedSnapshot == null, "new formal run owns fresh evidence");
            s.BeginStop("DONE"); s.Finish("new complete", "new summary", true); s.NoteCleanup(false);
            Check.That(s.Phase == RunPhase.Completed && s.CompletedSnapshot == "new complete", "completed run survives cleanup retry");
        }

        private static void IllegalTransitions()
        {
            var s = new RunSession();
            Check.Throws<InvalidOperationException>(() => s.AwaitReadiness(), "idle cannot become warm");
            Check.Throws<InvalidOperationException>(() => s.MarkRunning(), "idle cannot start a formal run");
            Check.Throws<InvalidOperationException>(() => s.Finish("x", "y", false), "idle cannot finalize");
            s.Begin("A", "room");
            Check.Throws<InvalidOperationException>(() => s.MarkRunning(), "preparation cannot bypass readiness");
            s.AwaitReadiness();
            Check.Throws<InvalidOperationException>(() => s.AwaitReadiness(), "readiness cannot be entered twice");
            s.MarkRunning();
            Check.Throws<InvalidOperationException>(() => s.MarkRunning(), "running cannot be committed twice");
            s.BeginStop("DONE"); s.Finish("x", "y", false);
            Check.Throws<InvalidOperationException>(() => s.Finish("overwrite", "overwrite", false), "completed evidence cannot be finalized twice");
        }

        private static void Notice()
        {
            var n = new AutomaticStopNotice(); n.BeginDriveSession("stop");
            Check.That(n.Record("owner", "FIRST", "room", 1, "A"), "first cause recorded");
            Check.That(!n.Record("other", "SECOND", "room", 1, "A") && n.Status().Contains("reason=FIRST"), "secondary shutdown does not replace first cause");
            int calls = 0; Func<string, bool> fail = _ => { calls++; return false; };
            n.TryDeliver(0f, fail); n.TryDeliver(.1f, fail); n.TryDeliver(.5f, fail); n.TryDeliver(1f, fail); n.TryDeliver(2f, fail);
            Check.That(calls == 3 && !n.Pending, "delivery attempts are bounded and spaced");
            n.BeginDriveSession("next");
            Check.That(n.HasNotice && !n.Pending && n.Status().Contains("PREVIOUS_DRIVE_SESSION"), "restart retains history without replay");
            n.Record("owner", "NEW", "room", 2, "B");
            n.TryDeliver(3f, _ => throw new InvalidOperationException("console"));
            Check.That(n.Pending, "console exception does not discard pending notice");
            n.TryDeliver(3.5f, _ => true);
            Check.That(!n.Pending && n.Status().Contains("consoleCallReturned=True"), "returned console call stops retries without claiming visibility");
        }
    }
}
