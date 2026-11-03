namespace MakemitAGA.ActiveRagdollLab.Model
{
    /// <summary>
    /// Immutable facts returned by one owner teardown invocation. Succeeded preserves
    /// that owner's existing release/retry contract; it is not a native safe-state or
    /// physical-destruction certificate. In particular, Destroy returning only commits
    /// a request to Unity. Opaque child/helper work is never invented as local evidence.
    /// </summary>
    internal readonly struct CleanupReceipt
    {
        internal bool Attempted { get; }
        internal bool Succeeded { get; }
        // Null means this owner only had a boolean aggregate, or did no fresh attempt.
        internal int? FailureCount { get; }
        internal bool RetryRequired => !Succeeded;
        // Managed root/material/child/orphan slots retained by THIS owner. This is not
        // a count of live Unity objects, descendants, or opaque helper resources.
        internal int RetainedOwnerSlots { get; }
        // Null means collision restoration is owned by an opaque child/helper.
        internal int? RemainingCollisionPairs { get; }
        // Only direct Object.Destroy calls made in this invocation are counted.
        // Neither counter establishes completion of Unity's deferred destruction.
        internal int DestroyRequestsAttempted { get; }
        internal int DestroyRequestsReturned { get; }
        internal string Summary { get; }

        internal CleanupReceipt(bool attempted, bool succeeded, int? failureCount,
            int retainedOwnerSlots, int? remainingCollisionPairs,
            int destroyRequestsAttempted, int destroyRequestsReturned, string summary)
        {
            Attempted = attempted;
            Succeeded = succeeded;
            FailureCount = failureCount;
            RetainedOwnerSlots = retainedOwnerSlots;
            RemainingCollisionPairs = remainingCollisionPairs;
            DestroyRequestsAttempted = destroyRequestsAttempted;
            DestroyRequestsReturned = destroyRequestsReturned;
            Summary = summary;
        }

        internal CleanupReceipt WithSummary(string summary) => new CleanupReceipt(
            Attempted, Succeeded, FailureCount, RetainedOwnerSlots, RemainingCollisionPairs,
            DestroyRequestsAttempted, DestroyRequestsReturned, summary);

        internal CleanupReceipt WithFailureCount(int failureCount) => new CleanupReceipt(
            Attempted, failureCount == 0, failureCount, RetainedOwnerSlots, RemainingCollisionPairs,
            DestroyRequestsAttempted, DestroyRequestsReturned, Summary);
    }
}
