using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using MakemitAGA.ActiveRagdollLab.Physics;
using Window = MakemitAGA.ActiveRagdollLab.Physics.RightFootPrecontact;

namespace ARL.Contracts
{
    internal static partial class RightFootPrecontactContracts
    {
        private static readonly Window.Mode First25 = Window.ForArm(PelvisDynamicReceiverIsolation.DynamicHipArmA);
        private static readonly Window.Mode First35 = Window.ForArm(PelvisDynamicReceiverIsolation.DynamicHipArmB);

        private static void RunPhysicalEntryContracts()
        {
            Check.Run("182 right native ceilings vary only on the first sample", PhysicalEntryBands);
            Check.Run("182 entry priority, occupied window, lock and trace carry retain real ownership", PhysicalEntryLifecycle);
            Check.Run("182 replays all 40 actionable pairs in 181 B and three subsequent rejections at 4031", CapturedPhysicalPairs);
        }

        private static void PhysicalEntryBands()
        {
            foreach (var mode in new[] { First25, First35 })
            foreach (var first in new[] { .025f, MathF.BitIncrement(.025f), .035f, MathF.BitIncrement(.035f), .045f })
            {
                var w = Audited(mode);
                var a = S(1, first); a.Gap = .012f;
                var b = S(2, .001f); b.Gap = .005f; b.Predicted = -.002f;
                EntryStep(w, a); EntryStep(w, b);
                bool allowed = first <= (mode == First25 ? .025f : .035f);
                Check.That(w.Physical == allowed && w.LinkedOrigin == (allowed ? 2 : 0), "inclusive physical first sample " + first);
                Check.That(w.Summary().Contains("nativeRequest-open-commitMismatch=" + (allowed ? "1|1|0" : "0|0|0")), "native request reconciles with commit");
            }

            foreach (string field in new[] { "second", "secondLow", "gap", "prediction", "firstFloor", "secondFloor",
                "support", "stale", "duration", "source", "newSwing", "lift", "phase", "speed", "ankle", "miss" })
            {
                var w = Audited(First35);
                var a = S(1, .030f);
                var b = S(2, .012f);
                switch (field)
                {
                    case "second": b.Min = MathF.BitIncrement(.025f); break;
                    case "secondLow": b.Min = MathF.BitDecrement(.001f); break;
                    case "gap": b.Gap = MathF.BitIncrement(.005f); break;
                    case "prediction": b.Predicted = MathF.BitDecrement(-.002f); break;
                    case "firstFloor": a.ExactFloor = false; break;
                    case "secondFloor": b.ExactFloor = false; break;
                    case "support": b.Support++; break;
                    case "stale": b.Age = .101f; break;
                    case "duration": b.Step.Dt = MathF.BitIncrement(Window.MaxSeconds); break;
                    case "source": b.Step.SourceSequence++; break;
                    case "newSwing": a.Swing = false; break;
                    case "lift": a.Lift = true; break;
                    case "phase": b.PhaseKnown = false; break;
                    case "speed": a.CornerV = 0; break;
                    case "ankle": b.AnkleY = a.AnkleY; break;
                }
                EntryStep(w, a); EntryStep(w, b, issueMiss: field == "miss");
                Check.That(!w.Tracking && w.Summary().Contains("nativeRequest-open-commitMismatch=0|0|0"), field);
            }
        }

        private static void PhysicalEntryLifecycle()
        {
            foreach (string reason in new[] { "Ordinary", "Recovery", "FirstMiss", "MissHeld", "Fault" })
            {
                var w = Audited(First35); EntryStep(w, S(1,.031f));
                EntryStep(w, S(2,.015f), ordinary: reason == "Ordinary", recovery: reason == "Recovery",
                    miss: reason == "FirstMiss", depth: 2, held: reason == "MissHeld", ok: reason != "Fault");
                Check.That(!w.Tracking && w.Summary().Contains("/nativeRequest-open-commitMismatch=1|0|0/"),
                    "same-step first refusal suppresses a requested open: " + reason);
            }
            var b = Audited(First35);
            EntryStep(b, S(1,.032f)); EntryStep(b, S(2,.020f)); EntryStep(b, S(3,.016f));
            Check.That(b.Physical && b.LinkedOrigin == 2, "an active common pair cannot stack a second window");
            EntryStep(b, S(4,.026f));
            Check.That(!b.Tracking && b.Summary().Contains("/end-local=Depth|4"), "active upper exit still 25 mm");
            EntryStep(b, S(5,.019f));
            Check.That(!b.Tracking, "close did not bypass the rearm lock");
            EntryStep(b, S(6,.031f)); EntryStep(b, S(7,.020f));
            Check.That(b.Physical && b.LinkedOrigin == 7, "rearm still uses 25 mm in B");
            EntryStep(b, S(8,.010f), ordinary:true);
            string log = b.Summary();
            Check.That(log.Contains("openedCommon25-added35-unknown=0|2|0") &&
                log.Contains("added35Starts-first-last=2|2|7") &&
                log.Contains("formalLinkedOrigin=7/") && log.Contains("windowSteps-nativeVerifiedUnloaded-violations=3|3|0"),
                "two expanded native windows belong to a single formal event");
            var carried = Audited(First35);
            EntryStep(carried, S(1,.030f), trace:false); EntryStep(carried, S(2,.018f), trace:false);
            carried.ResetTrace(); EntryStep(carried, S(3,.010f), ordinary:true);
            string summary = carried.Summary();
            Check.That(summary.Contains("firstMinMm=30/entryBand=Added35") && summary.Contains("carriedIn=1") &&
                summary.Contains("carriedOrigin=2/nativeStarts-first-last=0|0|0") && summary.Contains("added35Starts-first-last=0|0|0"),
                "carry preserves the actual band without inventing a new start at trace begin");

            var repeating = Audited(First35);
            float[] cycle = { .032f, .020f, .014f, .026f };
            for (int n = 1; n <= 200; n++) EntryStep(repeating, S(n, cycle[(n-1) % cycle.Length]));
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int n = 201; n <= 2000; n++) EntryStep(repeating, S(n, cycle[(n-1) % cycle.Length]));
            long bytes = GC.GetAllocatedBytesForCurrentThread() - before;
            Check.That(bytes == 0 && repeating.Summary().Contains("openedCommon25-added35-unknown=0|500|0"),
                "bounded expanded-window evidence counts all 500 starts without per-step allocations");
        }

        // E181 stored the exact gate inputs and identities below. Absolute ankle position,
        // absolute times, and floor bounds are reconstructed solely to supply a coherent pair.
        // Every row starts unlocked with no occupied window, as recorded at issue.
        private static void CapturedPhysicalPairs()
        {
            using var data = typeof(RightFootPrecontactContracts).Assembly.GetManifestResourceStream("ARL.Contracts.RightPrecontactPhysical181");
            using var reader = new StreamReader(data);
            var earlier = new HashSet<int>();
            var steadyOrdinary = new HashSet<int>();
            var earlierMiss = new HashSet<int>();
            var noOldWindow = new HashSet<int>();
            int common = 0, expanded = 0, rejected = 0, rejectedAfter4008 = 0;
            string line;
            while ((line = reader.ReadLine()) != null)
            {
                if (line.StartsWith("#")) continue;
                var col = line.Split('\t');
                int I(int i) => int.Parse(col[i], CultureInfo.InvariantCulture);
                float MetresOrSeconds(int i) => EntryFloat(col[i]);
                float dt = MetresOrSeconds(12);
                var first = S(I(6), MetresOrSeconds(8), dt);
                var second = S(I(2), MetresOrSeconds(9), dt);
                first.Step.Epoch = first.Source.Epoch = second.Step.Epoch = second.Source.Epoch = I(0);
                first.Step.Sequence = I(5); second.Step.Sequence = I(1);
                first.Step.SourceSequence = first.Source.Sequence = I(7);
                second.Step.SourceSequence = second.Source.Sequence = I(3);
                first.Source.Time = first.Source.Sequence * dt; second.Source.Time = second.Source.Sequence * dt;
                first.Support = second.Support = I(4);
                first.ExactFloor = (I(18) & 1) != 0; second.ExactFloor = (I(18) & 2) != 0;
                first.Age = MetresOrSeconds(13); second.Age = MetresOrSeconds(14);
                first.CornerV = MetresOrSeconds(15); second.CornerV = MetresOrSeconds(16);
                second.Gap = MetresOrSeconds(10); second.Predicted = MetresOrSeconds(11);
                second.AnkleY = first.AnkleY + MetresOrSeconds(17);
                int mask = I(19), landing = I(21);
                var a = Audited(First25); var b = Audited(First35);
                EntryStep(a, first); EntryStep(a, second); EntryStep(b, first); EntryStep(b, second);
                Check.That(a.Physical == bool.Parse(col[20]) && a.Physical == ((mask & 1) != 0) &&
                    b.Physical == ((mask & 2) != 0), "physical decisions on the same pair at " + second.Local);
                if (a.Physical) common++;
                else if (b.Physical) expanded++;
                else rejected++;
                if (second.Local == 4008)
                    Check.That(!a.Physical && b.Physical && landing == 4031 && I(25) == 0 &&
                        b.Summary().Contains("/entryBand=Added35"), "previously unopened 4031 approach");
                if (second.Local >= 4009 && second.Local <= 4011 && !a.Physical && !b.Physical) rejectedAfter4008++;
                if (a.Physical || !b.Physical || I(24) != second.Local) continue;
                earlier.Add(landing);
                if (col[22] == "Enabled" && col[23] == "STEADY") steadyOrdinary.Add(landing);
                if (col[22] == "HoldEnded") earlierMiss.Add(landing);
                if (I(25) == 0) noOldWindow.Add(landing);
                else Check.That(second.Local < I(25), "extra first opportunity precedes the actual 181 window");
            }
            Check.That(common == 23 && expanded == 17 && rejected == 3 && rejectedAfter4008 == 3, "23 common and 17 wider pairs");
            Check.That(earlier.Count == 14 && steadyOrdinary.Count == 8 && earlierMiss.SetEquals(new[] {3990,4031}) &&
                noOldWindow.SetEquals(new[] {4031}), "separate earlier windows, already successful controls, and independent missed events");
        }
    }
}
