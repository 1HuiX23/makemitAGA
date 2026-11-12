using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class FoundationContracts
    {
        internal static void Run()
        {
            Check.Run("foundation genuine stable window and accumulated drift", StableWindow);
            Check.Run("foundation source discontinuity, timeout and cancellation", WindowLifecycle);
            Check.Run("initial geometry recorded A/B and independent failure paths", Geometry);
        }
        private static SourceStepContext Source(int sequence, float time, int epoch = 1) =>
            new SourceStepContext { Epoch = epoch, Sequence = sequence, Time = time, Dt = .02f };
        private static void Pose(out Vector3[] p, out Quaternion[] q)
        {
            p = new Vector3[FoundationPoseWindow.BoneCount]; q = new Quaternion[p.Length];
            for (int i = 0; i < q.Length; i++) q[i] = Quaternion.identity;
            p[3] = new Vector3(-.1f, .111f, 0f); p[6] = new Vector3(.1f, .111f, 0f);
        }
        private static void StableWindow()
        {
            Pose(out var p, out var q); var w = new FoundationPoseWindow(); w.Begin(0f);
            for (int i = 1; i <= 50; i++) w.Observe(Source(i, i * .02f), p, q);
            Check.That(w.Pending && !w.Qualified, "less than one elapsed second cannot qualify");
            w.Observe(Source(51, 1.021f), p, q);
            Check.That(w.Qualified && !w.Pending, "a completed stable source interval qualifies once");
            p[0] = new Vector3(10f, 0f, 0f); w.Observe(Source(52, 1.04f), p, q);
            Check.That(w.Qualified, "finished acquisition does not turn into a runtime observer");
            Pose(out p, out q); w.Begin(0f);
            for (int i = 1; i <= 100; i++)
            { p[0] = new Vector3(i * .0005f, 0f, 0f); w.Observe(Source(i, i * .02f), p, q); }
            Check.That(w.Pending && !w.Qualified, "small per-frame motion cannot hide accumulated drift");
            Pose(out p, out q); w.Begin(0f);
            for (int i = 1; i <= 40; i++) w.Observe(Source(i, i * .02f), p, q);
            q[3] = Quaternion.AngleAxis(3f, new Vector3(0f, 1f, 0f)); w.Observe(Source(41, .82f), p, q);
            Check.Near(w.StableSeconds, 0f, 0f, "foot-only rotation starts a new pose window");
        }
        private static void WindowLifecycle()
        {
            Pose(out var p, out var q); var w = new FoundationPoseWindow(); w.Begin(0f);
            for (int i = 1; i <= 40; i++) w.Observe(Source(i, i * .02f), p, q);
            w.Observe(Source(41, 1.1f), p, q);
            Check.That(w.Pending && w.StableSeconds == 0f, "a stalled render interval cannot complete acquisition");
            w.Observe(Source(42, 1.12f), p, q);
            Check.That(w.StableSeconds == 0f, "stable timing restarts only with a genuine subsequent packet");
            w.Observe(Source(44, 1.14f), p, q);
            Check.That(!w.Pending && !w.Qualified && w.Reason == "SOURCE_DISCONTINUITY", "skipped identity rejects this request");
            w.Begin(0f); w.Observe(Source(1, .02f), p, q); w.Observe(Source(2, .04f, 2), p, q);
            Check.That(!w.Pending && !w.Qualified, "binding changes cannot inherit an old stable window");
            w.Begin(0f); p[6] = new Vector3(.1f, .122207f, 0f);
            for (int i = 1; i <= 749; i++) w.Observe(Source(i, i * .02f), p, q);
            Check.That(w.Pending && w.Reason == "WAIT_LEVEL_ANKLES", "recorded 11.2 mm mismatch never freezes a neutral fixture");
            w.Observe(Source(750, 15f), p, q);
            Check.That(!w.Pending && !w.Qualified && w.Reason.StartsWith("TIMEOUT:"), "resets cannot extend the absolute timeout");
            w.Begin(20f); w.Cancel("CLEANUP"); w.Observe(Source(1, 21f), p, q);
            Check.That(!w.Pending && w.Reason == "CLEANUP", "cleanup cancels delayed construction");
            Pose(out p, out q); w.Begin(0f); q[0] = new Quaternion(); w.Observe(Source(1, .02f), p, q);
            Check.That(!w.Pending && w.Reason == "SOURCE_POSE_INVALID", "zero quaternion cannot qualify by being still");
            w.Begin(0f); q[0] = Quaternion.identity; w.Observe(Source(1, .02f), p, q);
            w.Observe(Source(2, .02f), p, q);
            Check.That(w.Pending && w.StableSeconds == 0f, "equal rounded source time earns no stable duration");
        }
        private static InitialGeometryPreflight Good() => new InitialGeometryPreflight
        {
            Valid = true, Calibrated = true, FootprintsInside = true, SourceAge = .02f,
            HipBindingGap = .004f, HipBindingY = -.003886819f,
            RawMin = -.0026f, ControlMin = -.0014f, ActualMin = -.0014f
        };
        private static void Geometry()
        {
            var b = Good(); Check.That(b.Issue == null, "raw animation at -2.6 mm is separate from contact geometry");
            b.ActualMin = -.002884194f;
            Check.That(b.Issue == "ACTUAL_SOLE_BELOW_START_BAND", "historical recovery-depth start is no longer ordinary admission");
            b = Good(); b.ControlMin = -.002472526f;
            Check.That(b.Issue == "CONTROL_SOLE_BELOW_START_BAND", "v124 A start must not begin with an already missed entry");
            b = Good(); b.ControlMin = b.ActualMin = -FootContactGeometryLimits.OrdinaryPenetrationMeters;
            Check.That(b.Issue == null, "ordinary penetration boundary is shared and inclusive");
            var a = Good(); a.RawMin = -.01396893f; a.ControlMin = -.01662678f; a.ActualMin = -.01660346f;
            Check.That(a.Issue == "RAW_SOLE_BELOW_START_BAND", "recorded A persistent initial undercut rejects before trace");
            a = Good(); a.ControlMin = -.0166f;
            Check.That(a.Issue == "CONTROL_SOLE_BELOW_START_BAND", "a correct raw pose cannot mask the joined control geometry");
            a = Good(); a.ActualMin = -.0166f;
            Check.That(a.Issue == "ACTUAL_SOLE_BELOW_START_BAND", "source and control cannot mask the actual foot");
            a = Good(); a.HipBindingGap = .006f; Check.That(a.Issue == "HIP_BINDING_OFFSET", "frozen Hip mismatch is independently rejected");
            a = Good(); a.RawMin = .02f; Check.That(a.Issue == "SOLE_ABOVE_START_BAND", "hovering start is not accepted as grounded");
            a = Good(); a.SourceAge = .101f; Check.That(a.Issue == "SOURCE_NOT_FRESH", "stale held source does not certify current geometry");
            a = Good(); a.FootprintsInside = false; Check.That(a.Issue == "FOOTPRINT_OUTSIDE_SUPPORT", "all poses must lie inside the actual owned support");
            a = Good(); a.Calibrated = false; Check.That(a.Issue == "SUPPORT_PLANE_NOT_CALIBRATED", "fallback dimensions are not silently accepted");
            a = Good(); a.ActualMin = float.NaN; Check.That(a.Issue == "GEOMETRY_OR_SOURCE_INVALID", "nonfinite actual geometry fails closed");
            a = Good(); a.RawMin = -.003f; a.ControlMin = .015f; a.HipBindingGap = .005f; a.SourceAge = .1f;
            Check.That(a.Issue == null, "documented inclusive bounds stay inclusive");
            a = default; Check.That(a.Issue != null, "missing native readback cannot pass with default zeros");
        }
    }
}
