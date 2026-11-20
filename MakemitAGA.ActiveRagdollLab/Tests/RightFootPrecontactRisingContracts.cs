using System.Collections.Generic;
using Window = MakemitAGA.ActiveRagdollLab.Physics.RightFootPrecontact;

namespace ARL.Contracts
{
    internal static partial class RightFootPrecontactContracts
    {
        private static void RunRisingContracts()
        {
            Check.Run("B retains each verified solve inside the inclusive near-ground band", NearGroundRange);
            Check.Run("B requires continuous native receipts, current gait and the original time budget", NearGroundExits);
            Check.Run("Observation restart preserves released-window origins and live formal impacts", TraceLinksAndImpacts);
            Check.Run("Recorded v155 and v156 narrow-band rises preserve the signed-speed decision", RecordedNarrowRises);
            Check.Run("Only the falling eight-corner forecasts qualify at recorded rises above 3 mm", RecordedWideRises);
            Check.Run("The v158 B prediction-only window reaches the measured first rise", PredictedWindowFirstRise);
            Check.Run("The one-solve review requires an extended unloaded window in the entry band", RisingEntry);
            Check.Run("Zero or positive speed releases, a finite strictly negative speed resumes", RisingSign);
            Check.Run("A held solve cannot bypass lift, floor, clock, miss, or native load checks", RisingExits);
            Check.Run("The review and subsequent continuation spend the same 250 ms budget", RisingBudget);
            Check.Run("Trace reset preserves the one-shot and terminal events retain their origin", RisingLifetime);
        }

        // Inputs are eight-corner clearance and physical Foot ankle position in mm, and signed velocity in mm/s.
        private static Window.Sample Pose(int n, float height, float ankle, float goal, float velocity, float? predicted = null)
        {
            var s = Boundary(n, height/1000f, goal/1000f);
            s.AnkleY = ankle/1000f;
            s.CornerV = velocity/1000f;
            if (predicted.HasValue) s.Predicted = predicted.Value/1000f;
            return s;
        }

        private static void Schedule(Window w, int local)
        {
            w.CountSolve();
            Check.That(w.CheckUnloaded(local, false), "native load readback on scheduled solve");
        }

        private enum RecordedReview { Shared, FallingWide, Release }

        private static void RecordedNarrowRises()
        {
            var rows = new[] {
                (2809, Pose(6, .000238419f,108.88056f,-14.48153f,-.0599032f),
                       Pose(7, .42629987f,109.81326f,-13.5618f,25.303081f),
                       Pose(8,-.26171654f,109.71712f,-12.761116f,-41.280571f), RecordedReview.Shared, true),
                (3347, Pose(6, .005327165f,111.42962f,-3.633708f,-.7028952f),
                       Pose(7,2.816409f,112.93336f,-3.491335f,168.97012f),
                       Pose(8,3.558293f,113.30836f,-5.177587f,-207.3483f), RecordedReview.Shared, true),
                (4883, Pose(6, .23829192f,109.24406f,-8.762427f,-56.787796f),
                       Pose(7,1.1423603f,111.26739f,-7.384643f,53.948492f),
                       Pose(8,1.3394728f,112.08814f,-6.670468f,11.840841f), RecordedReview.Shared, false),
                (2709, Pose(6, .0014305115f,108.92452f,-10.256872f,-.48819184f),
                       Pose(7, .027380884f,109.69230f,-9.5542744f,-6.8711974f),
                       Pose(8, .0047683716f,110.56872f,-8.8664517f,-1.3986714f), RecordedReview.Shared, true),
                (2774, Pose(6, .00054389238f,108.93250f,-7.8022629f,-.18355064f),
                       Pose(7, .65616518f,109.71542f,-7.2072521f,29.022994f),
                       Pose(8, .38536638f,109.58432f,-6.8261176f,-16.244311f), RecordedReview.Shared, true)
            };
            CheckRecordedRises(rows);
        }

        private static void RecordedWideRises()
        {
            // At these v156 and v160 rises the real collider was released. The next speed was measured on that trajectory.
            var rows = new[] {
                (3502, Pose(6, 0.013634562f, 112.82834f, -3.5301819f, 0.023752451f),
                       Pose(7, 3.4176186f, 114.33955f, -5.0434768f, 205.00837f, 6.834425f),
                       Pose(8, 4.109107f, 113.25091f, -3.2137111f, 42.336553f), RecordedReview.Release, false),
                (3567, Pose(6, 0.97528845f, 114.11533f, -8.2880929f, 55.683099f),
                       Pose(7, 3.8038343f, 115.67314f, -4.9769059f, 169.85177f, 6.6346973f),
                       Pose(8, 3.3104494f, 114.33156f, -5.9413165f, -29.622279f), RecordedReview.Release, true),
                (5138, Pose(6, 0.0079721212f, 110.87651f, -7.8142509f, -0.060528517f),
                       Pose(7, 4.530862f, 113.92845f, -8.5116699f, -121.55788f, 2.5048971f),
                       Pose(8, -0.050321221f, 112.90491f, -5.2113011f, -274.60021f), RecordedReview.FallingWide, true),
                (5202, Pose(6, 0.79634786f, 114.2759f, -9.7298771f, 47.70337f),
                       Pose(7, 4.9235299f, 116.34389f, -11.23859f, 248.40117f, 9.0635493f),
                       Pose(8, 5.4119751f, 114.89578f, -7.8854859f, 29.832214f), RecordedReview.Release, false),
                (4031, Pose(6, 0.8219704f, 111.7648f, -3.7770122f, 28.430112f),
                       Pose(7, 4.7631487f, 114.23891f, -3.3240765f, 236.67896f, 3.4641288f),
                       Pose(8, 0.9053871f, 114.52585f, -5.0775185f, -268.03637f), RecordedReview.FallingWide, true),
                (2814, Pose(6, 0.0059083104f, 117.71058f, -15.17652f, -1.7062649f),
                       Pose(7, 4.0461272f, 121.72724f, -7.0947483f, 241.49781f, 8.071091f),
                       Pose(8, 10.779671f, 128.46875f, 0.94907731f, 403.96386f), RecordedReview.Release, false),
                (4989, Pose(6, 1.0700822f, 111.24993f, -4.9663782f, 51.143408f),
                       Pose(7, 3.1524748f, 112.23266f, -4.9573332f, 125.04238f, 1.5777574f),
                       Pose(8, 0.65807998f, 111.98676f, -4.7034919f, -171.51518f), RecordedReview.FallingWide, true)
            };
            CheckRecordedRises(rows);
        }

        private static void CheckRecordedRises(
            (int local, Window.Sample previous, Window.Sample rise, Window.Sample following, RecordedReview review, bool nextDescends)[] rows)
        {
            foreach (var (original, prior, rising, next, review, nextDescends) in rows)
            {
                var log = new List<string>();
                // Seed the extended window without manufacturing an earlier positional rise.
                var w = StartExtension(log, Window.Mode.FallingForecastRise5mm, cutoffAnkleMeters: prior.AnkleY);
                Check.That(w.Evaluate(prior, false, out _) == Window.End.None, "settled before " + original);
                Schedule(w, 6);
                var end = w.Evaluate(rising, false, out _);
                Check.That(end == Window.End.Rising, "positional release in original " + original);
                bool held = w.TryKeepAtBoundary(end);
                bool fallingWide = review == RecordedReview.FallingWide;
                bool expectHold = review == RecordedReview.Shared || fallingWide;
                Check.That(held == expectHold, "recorded rise decision " + original);
                const string limit = "/reviewLimitMm=5";
                string flag = "/wideForecastAtRise=" + fallingWide;
                if (!held)
                {
                    w.Close(end, 7);
                    w.Evaluate(next, false, out _);
                    w.BreakContinuity();
                    Check.That(log.Exists(x => x.Contains("/action=ControlRelease/") && x.Contains(flag) && x.EndsWith(limit)) &&
                        log.Exists(x => x.Contains("/firstLocal-minMm-cornerMmSec=8|")), "release and following off-step");
                    continue;
                }
                Check.That(!w.TryKeepAtBoundary(end), "single physical review");
                Check.That(log.Exists(x => x.Contains("/action=HoldOneSolve/") && x.Contains(flag) && x.EndsWith(limit)), "request evidence");
                Schedule(w, 7);
                end = w.Evaluate(next, false, out _);
                Check.That(end == (nextDescends ? Window.End.None : Window.End.Rising), "next velocity " + original);
                if (nextDescends) Schedule(w, 8);
                else
                {
                    Check.That(!w.TryKeepAtBoundary(end), "no retry after positive confirmation");
                    w.Close(end, 8);
                }
                Check.That(log.Exists(x => x.Contains("/action=" + (nextDescends ? "Resume" : "Release") + "/") &&
                    x.Contains(flag) && x.Contains("/rising-decision-shared-source=7|8|")), "one joined decision for " + original);
            }
        }

        private static void PredictedWindowFirstRise()
        {
            // Consecutive v158 B samples from approach to the first step after release.
            // Heights, gap and ankle are in mm, corner speed is in mm/s, source age is in ms.
            var rows = new (int source, float min, float goal, float predicted, float speed, float ankle, float gap, float age)[] {
                (8332, 17.796412f, 22.795513f, 9.9735986f, -469.36879f, 191.73409f, 3.8798894f, 7.6141357f),
                (8333, 10.950804f, 17.294005f, 4.1093114f, -410.48953f, 183.77632f, 2.6217769f, 7.2784424f),
                (8334, 5.9540123f, 11.935547f, 0.9723613f, -298.89905f, 176.68046f, 1.3731137f, 7.0343018f),
                (8334, 1.2669414f, 8.1150085f, -3.3934102f, -279.62109f, 169.09653f, 0.88968879f, 23.696899f),
                (8335, 0.42104721f, 2.5245845f, -0.99430443f, -41.013479f, 160.28237f, 5.5986312f, 9.6435547f),
                (8336, 1.4569759f, -0.22025406f, 2.4214115f, 57.866126f, 152.91174f, 7.6564448f, 9.3231201f),
                (8337, 3.3768117f, -1.4127493f, 5.4276688f, 123.0514f, 145.20106f, 8.8236732f, 9.0332031f),
                (8338, 6.8995059f, -3.03334f, 10.531375f, 217.91214f, 138.34164f, 3.9389371f, 9.0332031f),
                (8339, 8.303687f, -2.9049367f, 7.9204105f, 89.461833f, 128.76306f, 1.5026318f, 9.0484619f),
                (8340, 9.325549f, -3.0573085f, -4.7471644f, -844.36274f, 120.24075f, 3.2573247f, 8.9874268f),
                (8341, 1.8876046f, -1.9393936f, -7.0612496f, -444.81051f, 114.78376f, 3.1792873f, 8.9874268f),
                (8342, 0.37920475f, -0.95089525f, -1.441329f, -90.398289f, 114.05088f, 5.3467262f, 8.9874268f),
                (8343, 0.075645745f, -0.87447464f, -0.29562649f, -18.227544f, 113.70479f, 8.8018449f, 8.0718994f),
                (8344, 0.017851591f, -3.545709f, -0.056847311f, -3.3063442f, 113.04125f, 9.7760549f, 7.6141357f),
                (8345, 0.0064969063f, -4.5495629f, -0.0050036861f, -0.69003552f, 112.12093f, 10.563607f, 7.2784424f),
                (8346, 0.88065118f, -3.8617924f, 1.7595378f, 52.733198f, 111.60956f, 11.961935f, 6.8054199f),
                (8347, 3.1038746f, -3.1829402f, 2.2488823f, 133.58346f, 112.54483f, 9.7881686f, 6.8206787f),
                (8348, 1.3910532f, -3.0150786f, -1.9403927f, -199.88674f, 112.24882f, 7.0703733f, 6.8206787f)
            };
            var log = new List<string>();
            var window = new Window(log.Add);
            window.Select(Window.Mode.FallingForecastRise5mm);
            float sourceTime = 0f;
            for (int i = 0; i < rows.Length; i++)
            {
                int local = 5672 + i;
                var r = rows[i];
                var s = S(local, r.min / 1000f, 1f / 60f);
                s.Step.Sequence = local + 2885;
                s.Step.SourceSequence = r.source;
                s.Source.Sequence = r.source;
                s.Age = r.age / 1000f;
                if (i == 0 || r.source != rows[i - 1].source)
                    sourceTime = s.Step.FixedTime - s.Age;
                s.Source.Time = sourceTime;
                s.ControlMin = r.goal / 1000f;
                s.Predicted = r.predicted / 1000f;
                s.CornerV = r.speed / 1000f;
                s.AnkleY = r.ankle / 1000f;
                s.Gap = r.gap / 1000f;
                var end = window.Evaluate(s, false, out bool open);
                Check.That(open == (local == 5673), "recorded opening");
                if (open) window.Open();
                if (local == 5682)
                {
                    Check.That(end == Window.End.Timeout && window.TryKeepAtBoundary(end), "forecast grants extension");
                    Check.That(log.Exists(x => x.Contains("/action=Extend/atBaseMatch=Eligible/") &&
                        x.Contains("/predictionOnlyAtBase=True/scheduled-unloaded-violations=9|9|0")), "same cutoff evidence in both arms");
                }
                else if (local == 5688)
                {
                    bool hold = window.TryKeepAtBoundary(end);
                    Check.That(end == Window.End.Rising && hold,
                        "forecast falls from +3.104 to +2.249 mm");
                }
                else Check.That(end == Window.End.None, "continuous replay at " + local);
                if (window.Physical)
                {
                    Schedule(window, local);
                    if (local == 5675) window.Callback(local, s.Support, .09912605f, 0f);
                }
            }
            Check.That(window.Physical, "physical state on the step following the rise");
            Check.That(log.Exists(x => x.Contains("/action=HoldOneSolve/") &&
                x.Contains("/rising-decision-shared-source=5688|5688|") && x.Contains("/wideForecastAtRise=True/") &&
                x.Contains("/scheduled-unloaded-violations=15|15|0/")), "decision at the original rise");
            Check.That(log.Exists(x => x.Contains("/action=Resume/") && x.Contains("/rising-decision-shared-source=5688|5689|")),
                "recorded next velocity on the released trajectory");
        }

        private static Window Waiting(List<string> log = null, bool checkedLoad = true, bool wide = false)
        {
            var w = StartExtension(log, Window.Mode.FallingForecastRise5mm);
            var rising = Pose(6, wide ? 4.5f : 2.5f, 104, -4, 20, wide ? 1f : 3f);
            Check.That(w.TryKeepAtBoundary(w.Evaluate(rising, false, out _)), "granted one solve");
            w.CountSolve();
            if (checkedLoad) w.CheckUnloaded(6, false);
            return w;
        }

        private static void RisingEntry()
        {
            var cases = new (float height, float goal, float forecast, bool narrow, bool observedWide, bool fallingWide)[] {
                (0f, -3f, 4f, true, false, false), (3f, -2.001f, 3f, true, false, false),
                (3f, -3f, 5f, true, false, false), (-.001f, -3f, -7f, false, false, false),
                (3.001f, -4f, 3f, false, true, true), (5f, -4f, 4.999f, false, true, true),
                (5f, -4f, 5f, false, true, false), (3.001f, -4f, 4f, false, true, false),
                (5.001f, -4f, -7f, false, false, false), (1f, -2f, -7f, false, false, false),
                (4.5f, -2f, 1f, false, false, false), (4.5f, float.NaN, 1f, false, false, false),
                (4.5f, -4f, float.NaN, false, false, false) };
            foreach (var row in cases)
            {
                var log = new List<string>();
                var w = StartExtension(log, Window.Mode.FallingForecastRise5mm);
                var low = Pose(6, -.5f, 100, -3, -20);
                Check.That(w.Evaluate(low, false, out _) == Window.End.None, "settling before rise");
                Schedule(w, 6);
                var rising = Pose(7, row.height, 101, row.goal, 5, row.forecast);
                var end = w.Evaluate(rising, false, out _);
                bool held = w.TryKeepAtBoundary(end);
                bool expected = row.narrow || row.fallingWide;
                Check.That(end == (float.IsFinite(row.forecast) ? Window.End.Rising : Window.End.Floor) && held == expected,
                    "height, held goal and strictly falling physical forecast");
                if (!held) w.Close(end, 7);
                Check.That(log.Exists(x => x.Contains("/action=ControlRelease/") &&
                    x.Contains("/wideForecastAtRise=" + row.fallingWide + "/")) == (row.observedWide && !held),
                    "paired opportunity at the actual release");
            }
            var original = Open(Window.Mode.FallingForecastRise5mm);
            original.CheckUnloaded(2, false);
            original.Evaluate(S(3, .001f), false, out _);
            Schedule(original, 3);
            var baseRise = Pose(4, 2, 104, -4, 20);
            var reason = original.Evaluate(baseRise, false, out _);
            Check.That(reason == Window.End.Rising && !original.TryKeepAtBoundary(reason), "no review in original window");
            var missing = StartExtension(mode: Window.Mode.FallingForecastRise5mm);
            missing.Evaluate(Pose(6, .1f, 100, -4, -5), false, out _);
            missing.CountSolve();
            reason = missing.Evaluate(Pose(7, 2, 104, -4, 20), false, out _);
            Check.That(reason == Window.End.Rising && !missing.TryKeepAtBoundary(reason), "no hold after a missed readback");
        }

        private static void RisingSign()
        {
            foreach (var (velocity, expected) in new[] {
                (-.0001f, Window.End.None), (0f, Window.End.Rising),
                (.0001f, Window.End.Rising), (float.NaN, Window.End.Floor) })
            foreach (bool wide in new[] { false, true })
            {
                var log = new List<string>();
                var w = Waiting(log, wide: wide);
                w.ResetTrace();
                var next = Pose(7, 6, 105, 1, velocity, 9f);
                Check.That(w.Evaluate(next, false, out _) == expected, "signed speed on next same-floor step");
                if (expected == Window.End.None) Schedule(w, 7);
                else
                {
                    Check.That(!w.TryKeepAtBoundary(expected), "no second chance");
                    w.Close(expected, 7);
                }
                string action = expected == Window.End.None ? "Resume" : expected == Window.End.Rising ? "Release" : "Floor";
                Check.That(log.Exists(x => x.Contains("/action=" + action + "/") &&
                    x.Contains("/wideForecastAtRise=" + wide + "/")), "origin forecast survives reset and opposite next forecast");
            }
        }

        private static void RisingExits()
        {
            foreach (var why in new[] { Window.End.Clock, Window.End.Floor, Window.End.Stale, Window.End.Gap,
                Window.End.Depth, Window.End.Lift, Window.End.Miss, Window.End.GoalLost, Window.End.Fault })
            {
                var log = new List<string>();
                var w = Waiting(log, why != Window.End.Fault);
                var s = Pose(7, 3.5f, 105, -4, -45);
                switch (why)
                {
                    case Window.End.Clock: s.Step.Sequence++; break;
                    case Window.End.Floor: s.Support++; break;
                    case Window.End.Stale: s.Age = .101f; break;
                    case Window.End.Gap: s.Gap = .030f; break;
                    case Window.End.Depth: s.Min = -.00301f; break;
                    case Window.End.Lift: s.Lift = true; break;
                    case Window.End.GoalLost: s.ControlMin = float.NaN; break;
                }
                var end = w.Evaluate(s, why == Window.End.Miss, out _);
                Check.That(end == why && !w.TryKeepAtBoundary(end), "pending review exits on " + why);
                w.Close(end, 7);
                Check.That(!w.Physical && log.Exists(x => x.Contains("RISING_V041560]") &&
                    x.Contains("/action=" + why + "/")), "censored reason preserved");
            }
            var firstLift = StartExtension(mode: Window.Mode.FallingForecastRise5mm);
            var lift = Pose(6, 5.22f, 110, .765f, 307.9f); lift.Lift = true;
            var reason = firstLift.Evaluate(lift, false, out _);
            Check.That(reason == Window.End.Lift && !firstLift.TryKeepAtBoundary(reason), "155 B step 4243 is an explicit lift");
        }

        private static void RisingBudget()
        {
            foreach (bool fits in new[] { true, false })
            foreach (bool wide in new[] { false, true })
            {
                var w = StartExtension(mode: Window.Mode.FallingForecastRise5mm);
                var settle = Pose(6, 1, 101, -4, 0);
                settle.Step.Dt = fits ? .15f : .151f;
                Check.That(w.Evaluate(settle, false, out _) == Window.End.None, "extension settling");
                Schedule(w, 6);
                var rise = Pose(7, wide ? 4f : 2f, 104, -4, 10, 1f);
                var end = w.Evaluate(rise, false, out _);
                Check.That(end == Window.End.Rising && w.TryKeepAtBoundary(end) == fits, "review solve fits remaining time");
                if (!fits) continue;
                Schedule(w, 7);
                end = w.Evaluate(Pose(8, 2, 104, -4, -20), false, out _);
                Check.That(end == Window.End.ExtensionLimit && !w.TryKeepAtBoundary(end), "negative speed cannot borrow extra time");
                w.Close(end, 8);
                Check.That(w.Summary().Contains("/baseExpiryLocal-extraSolves-extraMs=5|3|250"), "review solve billed exactly once");
            }
        }

        private static void NearGroundRange()
        {
            var log = new List<string>();
            var w = StartExtension(log, Window.Mode.NearGroundHold8mm);
            float[] heights = { 0f, 4f, 8f, 8f, .4f, 4.1f, 7.9f };
            for (int i = 0; i < heights.Length; i++)
            {
                int local = 6 + i;
                var s = Pose(local, heights[i], 102 + i, i % 2 == 0 ? 5 : -4, i % 2 == 0 ? 120 : -110, 12f);
                s.Step.Dt = .02f;
                Check.That(w.Evaluate(s, false, out _) == Window.End.None, "hold the measured in-band step " + local);
                Schedule(w, local);
                w.ResetTrace();
            }
            var leave = Pose(13, 8.001f, 103, -9f, -100f, -5f);
            Check.That(w.Evaluate(leave, false, out _) == Window.End.NearGroundExit, "outside band releases even on a falling forecast");
            w.Close(Window.End.NearGroundExit, 13);
            Check.That(log.Exists(x => x.Contains("/action=NearGroundExit/") && x.Contains("/nearHoldFirstRiseLocal=7/")),
                "first retained positional rise recorded once after the scheduled solve");
            w.Evaluate(Pose(14, 4f, 101, -4, -80), false, out bool reopen);
            Check.That(!reopen, "one approach cannot repeatedly re-open");
            foreach (float h in new[] { -.000335f, -.001f })
            {
                var settled = StartExtension(mode: Window.Mode.NearGroundHold8mm);
                var atZero = Pose(6, h, 100, -4, -30);
                if (h == -.001f) atZero.Min = -.000001f;
                Check.That(settled.Evaluate(atZero, false, out _) == Window.End.None,
                    "sub-micrometre zero readback at " + h + " mm");
            }
            foreach (float h in new[] { -.001001f, -.01f, 8.001f })
            {
                var other = StartExtension(mode: Window.Mode.NearGroundHold8mm);
                Check.That(other.Evaluate(Pose(6, h, 100, -4, -30), false, out _) == Window.End.NearGroundExit,
                    "use raw signed clearance at " + h + " mm");
            }
        }

        private static void NearGroundExits()
        {
            foreach (var why in new[] { Window.End.Lift, Window.End.Floor, Window.End.Clock, Window.End.Gap,
                Window.End.Depth, Window.End.Stale, Window.End.Miss, Window.End.GoalLost, Window.End.Fault })
            {
                var w = StartExtension(mode: Window.Mode.NearGroundHold8mm);
                var prior = Pose(6, 2, 103, 1, 10);
                Check.That(w.Evaluate(prior, false, out _) == Window.End.None, "in-band previous solve");
                w.CountSolve();
                if (why != Window.End.Fault) Check.That(w.CheckUnloaded(6, false), "native readback");
                var s = Pose(7, 2.5f, 104, 1, 12);
                switch (why)
                {
                    case Window.End.Clock: s.Step.UnscaledTime = 0f; break;
                    case Window.End.Floor: s.Support++; break;
                    case Window.End.Gap: s.Gap = .030f; break;
                    case Window.End.Depth: s.Min = -.00301f; break;
                    case Window.End.Stale: s.Age = .101f; break;
                    case Window.End.Lift: s.Lift = true; break;
                    case Window.End.GoalLost: s.ControlMin = float.NaN; break;
                }
                var end = w.Evaluate(s, why == Window.End.Miss, out _);
                Check.That(end == why && !w.TryKeepAtBoundary(end), "in-band hard exit " + why);
            }
            var limit = StartExtension(mode: Window.Mode.NearGroundHold8mm);
            var first = Pose(6, 6, 110, 2, 100);
            first.Step.Dt = .11f;
            Check.That(limit.Evaluate(first, false, out _) == Window.End.None, "first rising solve");
            Schedule(limit, 6);
            var second = Pose(7, 7, 112, 2, 90);
            second.Step.Dt = .09f;
            Check.That(limit.Evaluate(second, false, out _) == Window.End.None, "exactly 250 ms total extension");
            Schedule(limit, 7);
            Check.That(limit.Evaluate(Pose(8, 8, 114, 2, 80), false, out _) == Window.End.ExtensionLimit,
                "subsequent rise cannot renew time");
        }

        private static void TraceLinksAndImpacts()
        {
            var w = Open(Window.Mode.NearGroundHold8mm);
            w.Evaluate(S(3, .012f), false, out _);
            w.Close(Window.End.Rising, 3);
            w.ResetTrace();
            Check.That(w.LinkedOrigin == 2, "a recent released window stays linked");
            w.BeginFormal(false);
            w.AfterFormalEvent();
            w.ResetTrace();
            Check.That(w.NeedsFormalImpact(42), "trace started before the fresh formal callback");
            w.FormalCallback(4, 42, 0f, .4f);
            w.ResetTrace();
            w.FormalCallback(5, 42, .12f, .6f);
            Check.That(w.Summary().Contains("starts-saved-firstCallback-firstNonzero=1|1|1|1") &&
                w.Summary().Contains("/priorWindowLocal=2/firstCallbackLocal-signedNormalNs-deltaWRadSec=4|0|0.4/firstNonzeroLocal-signedNormalNs-deltaWRadSec=5|0.12|0.6"),
                "first callback and first nonzero stay attached to the formal start");
            w.EndFormal();
            w.ResetTrace();
            Check.That(!w.NeedsFormalImpact(42) && w.Summary().Contains("starts-saved-firstCallback-firstNonzero=0|0|0|0"),
                "completed formal stance is not carried into another trace");
            var log = new List<string>();
            var follow = StartExtension(log, Window.Mode.NearGroundHold8mm);
            var exit = Pose(6, 9, 105, -4, 100);
            follow.Close(follow.Evaluate(exit, false, out _), 6);
            for (int n = 7; n <= 10; n++)
            {
                if (n == 9) follow.ResetTrace();
                follow.Evaluate(Pose(n, 6, 103, -4, -50), false, out _);
            }
            Check.That(log.Exists(x => x.Contains("/end=FourSteps/samples-last-firstBelowMinus2=4|10|0")),
                "a trace restart preserves the four-step observation of the old trajectory");
        }

        private static void RisingLifetime()
        {
            var log = new List<string>();
            var w = Waiting(log);
            w.ResetTrace();
            Check.That(w.Physical && w.Summary().Contains("/risingReviewLocal-pending=6|True"), "trace reset preserves pending state");
            var down = Pose(7, 1, 103, -4, -20);
            Check.That(w.Evaluate(down, false, out _) == Window.End.None, "continue normally after reset");
            Schedule(w, 7);
            w.ResetTrace();
            var reason = w.Evaluate(Pose(8, 2, 104, -4, 20), false, out _);
            Check.That(reason == Window.End.Rising && !w.TryKeepAtBoundary(reason), "second rise closes the same window");
            w.Close(reason, 8);
            Check.That(log.FindAll(x => x.Contains("/action=HoldOneSolve/")).Count == 1, "one request per origin");
            foreach (var terminal in new[] { Window.End.Stop, Window.End.Handoff })
            {
                log.Clear(); w = Waiting(log);
                if (terminal == Window.End.Handoff)
                {
                    w.Evaluate(Pose(7, 2.5f, 105, -4, 12), false, out _);
                    w.BeginFormal(false);
                }
                w.Close(terminal, terminal == Window.End.Stop ? 0 : 7);
                Check.That(log.Exists(x => x.Contains("/action=" + terminal + "/")) &&
                    !log.Exists(x => x.Contains("/action=Resume/")), "terminal owner wins");
            }
        }
    }
}
