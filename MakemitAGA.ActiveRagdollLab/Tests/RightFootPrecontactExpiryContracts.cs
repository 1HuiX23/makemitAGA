using System;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Physics;
using Window = MakemitAGA.ActiveRagdollLab.Physics.RightFootPrecontact;

namespace ARL.Contracts
{
    internal static partial class RightFootPrecontactContracts
    {
        private static void RunExpiryContracts()
        {
            Check.Run("Current hip arms select physical precontact on the right only", ExpiryArms);
            Check.Run("Eight v154 B expiry snapshots score from information available at cutoff", RecordedCutoffs);
            Check.Run("Both arms extend on the measured prediction-only cutoffs unless formal stance takes over", RecordedPredictedCutoffs);
            Check.Run("Prediction alone at base expiry keeps the common safety and timing gates", PredictedCutoffEdges);
            Check.Run("The wider first-expiry band requires a descending physical forecast on the measured floor", WideForecastCutoffs);
            Check.Run("A wide cutoff preserves formal priority, trace attribution and one shared time budget", WideForecastLifetime);
            Check.Run("Expiry requires a verified unloaded window, measured impulse, and strict directional thresholds", ExpiryEdges);
            Check.Run("Extension consumes at most a quarter second of fixed solves, once", ExtensionBudget);
            Check.Run("Extended windows release on the existing safety and motion boundaries", ExtensionExits);
            Check.Run("Every expiry and its four subsequent off solves survive the detail cap", ExpiryCoverage);
        }

        private static void ExpiryArms()
        {
            Check.That(Window.ForArm(PelvisDynamicReceiverIsolation.DynamicHipArmA) == Window.Mode.PreviousStepExpiryReview &&
                Window.ForArm(PelvisDynamicReceiverIsolation.DynamicHipArmB) == Window.Mode.PreviousStepExpiryReviewFirst35mm &&
                Window.ForArm(null) == Window.Mode.Off && Window.ForArm("UNKNOWN") == Window.Mode.Off, "production selector");
            foreach (var mode in new[] { Window.Mode.NearGroundHold8mm, Window.Mode.PreviousStepExpiryReview, Window.Mode.PreviousStepExpiryReviewFirst35mm })
                Check.That(Open(mode).Physical && Window.ForFoot(mode, "LEFT") == Window.Mode.Off, "right physical arm");
        }

        private static Window AtDeadline(Window.Mode mode, float impulse, Action<string> log = null, bool checkedLoad = true)
        {
            var w = Open(mode, .05f, log);
            if (checkedLoad) w.CheckUnloaded(2, false);
            w.Callback(2, 42, impulse, 0f);
            for (int i = 3; i <= 4; i++)
            {
                Check.That(w.Evaluate(S(i, .010f, .05f), false, out _) == Window.End.None, "inside 150 ms");
                w.CountSolve();
                w.CheckUnloaded(i, false);
            }
            return w;
        }

        private static Window.Sample Boundary(int n = 5, float min = .001f, float control = -.010f)
        {
            var s = S(n, min, .05f);
            s.AnkleY = .101f; s.ControlMin = control;
            return s;
        }

        private static void RecordedCutoffs()
        {
            // Measured v154 B inputs in mm, mm/s and N·s. At 4047 the held goal was clear but the physical forecast was deep.
            var rows = new (int local, float impulse, float min, float goal, float predicted, float speed, bool goalEligible)[] {
                (2764, .1690859f, .461f, -19.906f, -1.404f, -110.575f, true),
                (3382, .001019191f, .371f, -7.826f, -1.116f, -88.239f, true),
                (3445, .1552993f, 2.387f, -5.979f, -8.942f, -550.029f, true),
                (3511, 0, 13.618f, 14.192f, 13.299f, -19.148f, false),
                (3984, .01513517f, .109f, .002f, -.556f, -26.812f, false),
                (4047, .1340616f, 1.130f, .146f, -3.430f, -260.173f, false),
                (4759, .02517055f, 11.940f, .969f, 15.378f, 460.954f, false),
                (4890, 0, 18.263f, 13.433f, 14.583f, -220.828f, false)
            };
            foreach (var r in rows)
            foreach (var mode in new[] { Window.Mode.FallingForecastRise5mm, Window.Mode.NearGroundHold8mm })
            {
                var log = new List<string>();
                var w = AtDeadline(mode, r.impulse, log.Add);
                var s = Boundary(min: r.min/1000f, control: r.goal/1000f);
                s.Predicted = r.predicted/1000f; s.CornerV = r.speed/1000f;
                var end = w.Evaluate(s, false, out _);
                Check.That(end == Window.End.Timeout, "same physical cutoff " + r.local);
                bool expected = r.goalEligible || r.local == 4047;
                bool extended = w.TryKeepAtBoundary(end);
                Check.That(extended == expected, "selection " + r.local);
                if (!extended) w.Close(end, s.Local);
                Check.That(log.Count == 1 && log[0].Contains("/atBaseMatch=Eligible/") == expected &&
                    log[0].Contains("/predictionOnlyAtBase=True/") == (r.local == 4047), "same-state measurement");
            }
        }

        private static void RecordedPredictedCutoffs()
        {
            // Original cutoff, height, held goal, next-step physical minimum, signed corner speed and prior impulse.
            // Include the ordinary v157 A handoff to check that a prediction can be logged without granting an extension.
            var rows = new (int local, float height, float goal, float predicted, float velocity, float impulse)[] {
                (2840, 1.051478f, 7.087424f, -3.33153f, -228.5672f, .1125567f),
                (2738, 2.473228f, -1.762606f, -7.239652f, -582.7727f, .02828927f),
                (5603, .8196607f, -.8513555f, -4.29262f, -189.3042f, .01424326f),
                (2831, 2.692267f, 4.288659f, -7.684852f, -620.3016f, .04043663f),
                (3359, 1.313143f, 1.563966f, -5.363561f, -313.3809f, .05316359f),
                (5891, .3595203f, -1.622468f, -2.166429f, -87.40187f, .004153267f),
                (3435, .7296801f, 2.434544f, -2.382344f, -172.7741f, .01061017f),
                (3499, 1.001731f, -.1076981f, -3.039599f, -234.0485f, .05889545f),
                (5075, 1.293823f, -1.993686f, -3.750247f, -302.5952f, .005693369f),
                (4339, 1.988545f, 3.71179f, -5.947484f, -463.924f, .008147256f),
                (5682, 1.887605f, -1.939394f, -7.06125f, -444.8105f, .09912605f),
                (4688, 2.440617f, -.5517602f, -4.180131f, -397.2448f, .02923636f)
            };
            foreach (var r in rows)
            foreach (var mode in new[] { Window.Mode.FallingForecastRise5mm, Window.Mode.NearGroundHold8mm })
            {
                var log = new List<string>();
                var w = AtDeadline(mode, r.impulse, log.Add);
                var s = Boundary(min: r.height/1000f, control: r.goal/1000f);
                s.Predicted = r.predicted/1000f; s.CornerV = r.velocity/1000f;
                var reason = w.Evaluate(s, false, out _);
                Check.That(reason == Window.End.Timeout, "first deadline " + r.local);
                if (r.local == 4688)
                {
                    w.BeginFormal(false); w.Close(Window.End.Handoff, s.Local);
                    Check.That(!w.TryKeepAtBoundary(reason) && log[0].Contains("/action=Handoff/"), "formal owner at cutoff");
                }
                else
                {
                    Check.That(w.TryKeepAtBoundary(reason) && log[0].Contains("/action=Extend/"), "shared expiry " + r.local);
                    Schedule(w, s.Local);
                    w.ResetTrace();
                    var next = Boundary(6, min: .0029f, control: 0f);
                    next.AnkleY = s.AnkleY + .001f; next.Predicted = -.005f;
                    var rise = w.Evaluate(next, false, out _);
                    bool nearGround = mode == Window.Mode.NearGroundHold8mm;
                    Check.That(rise == (nearGround ? Window.End.None : Window.End.Rising), "renewal after a forecast-only cutoff");
                    if (nearGround) Schedule(w, next.Local);
                    else Check.That(!w.TryKeepAtBoundary(rise), "control needs a downward goal on its first rise");
                    w.Close(nearGround ? Window.End.Stop : rise, next.Local);
                    Check.That(log[1].Contains("phase=END/") && log[1].Contains("/predictionOnlyAtBase=True/") &&
                        log[1].Contains(nearGround ? "/extraSolves=2/" : "/extraSolves=1/"), "trace retains the actual extra solves");
                }
                Check.That(log[0].Contains("/predictionOnlyAtBase=True/") && log[0].Contains("/scheduled-unloaded-violations=3|3|0"),
                    "paired pre-intervention evidence " + r.local);
            }
        }

        private static void PredictedCutoffEdges()
        {
            var cases = new (float height, float control, float predicted, float speed, float impulse, bool verified, float dt, bool eligible)[] {
                (0f, 0f, -.002001f, -.01f, .1f, true, .05f, true),
                (.003f, -.002f, -.002001f, -.01f, .1f, true, .25f, true),
                (.001f, 0f, -.002f, -.01f, .1f, true, .05f, false),
                (.001f, 0f, -.001999f, -.01f, .1f, true, .05f, false),
                (.005001f, 0f, -.01f, -.01f, .1f, true, .05f, false),
                (-.000001f, 0f, -.01f, -.01f, .1f, true, .05f, false),
                (.001f, 0f, -.01f, 0f, .1f, true, .05f, false),
                (.001f, 0f, -.01f, .01f, .1f, true, .05f, false),
                (.001f, float.NaN, -.01f, -.01f, .1f, true, .05f, false),
                (.001f, float.PositiveInfinity, -.01f, -.01f, .1f, true, .05f, false),
                (.001f, 0f, -.01f, -.01f, 0f, true, .05f, false),
                (.001f, 0f, -.01f, -.01f, .1f, false, .05f, false),
                (.001f, 0f, -.01f, -.01f, .1f, true, .251f, false)
            };
            foreach (var r in cases)
            foreach (var mode in new[] { Window.Mode.FallingForecastRise5mm, Window.Mode.NearGroundHold8mm })
            {
                var log = new List<string>(); var w = AtDeadline(mode, r.impulse, log.Add, r.verified);
                var s = Boundary(min: r.height, control: r.control);
                s.Predicted = r.predicted; s.CornerV = r.speed; s.Step.Dt = r.dt;
                var reason = w.Evaluate(s, false, out _);
                Check.That(reason == Window.End.Timeout, "base expiry on a continuous physical step");
                bool granted = w.TryKeepAtBoundary(reason);
                Check.That(granted == r.eligible, "forecast boundaries and evidence");
                if (!granted) w.Close(reason, s.Local);
                Check.That(log[0].Contains("/predictionOnlyAtBase=True/") == r.eligible, "common marker requires full same-step eligibility");
            }
            foreach (float invalid in new[] { float.NaN, float.PositiveInfinity, float.NegativeInfinity })
            {
                var w = AtDeadline(Window.Mode.NearGroundHold8mm, .1f);
                var s = Boundary(); s.Predicted = invalid;
                var reason = w.Evaluate(s, false, out _);
                Check.That(reason == Window.End.Floor && !w.TryKeepAtBoundary(reason), "unusable physical forecast");
            }
        }

        private static void WideForecastCutoffs()
        {
            // Height, held target and eight-corner forecast in mm. Velocity in mm/s.
            var cases = new (float height, float goal, float predicted, float speed, bool match)[] {
                (3.450639f, -8.882091f, -9.564167f, -780.8883f, true),  // v159 B 2765
                (3.622122f, 13.05547f, -4.09138f, -462.8101f, true),    // v159 A 2854
                (3.150761f, 10.67407f, -4.49143f, -458.5315f, true),    // v159 A 5907
                (4.943393f, -8.990556f, -6.507518f, 158.6022f, false), // v159 A 5716
                (3.001f, -2f, -2.001f, -.001f, true),
                (5f, 2f, -2.001f, -1f, true), (5.001f, -8f, -9f, -1f, false),
                (4f, -8f, -2f, -50f, false), (4f, -8f, -1.999f, -50f, false),
                (4f, -8f, -9f, 0f, false), (4f, float.NaN, -9f, -50f, false)
            };
            foreach (var r in cases)
            foreach (var mode in new[] { Window.Mode.FallingForecastRise5mm, Window.Mode.NearGroundHold8mm })
            {
                var log = new List<string>();
                var w = AtDeadline(mode, .07f, log.Add);
                var s = Boundary(min: r.height / 1000f, control: r.goal / 1000f);
                s.Predicted = r.predicted / 1000f;
                s.CornerV = r.speed / 1000f;
                var end = w.Evaluate(s, false, out _);
                Check.That(end == Window.End.Timeout && w.TryKeepAtBoundary(end) == r.match, "both arms inherit the same cutoff");
                if (!r.match) w.Close(end, s.Local);
                Check.That(log.Count == 1 && log[0].Contains("/wideForecastAtBase=" + r.match) &&
                    log[0].Contains("/predictionOnlyAtBase=" + (r.match && r.goal >= -2f) + "/") &&
                    log[0].Contains(r.match ? "/action=Extend/atBaseMatch=Eligible/" : "/action=Timeout/atBaseMatch=OutsideBand/"),
                    "paired opportunity and actual action");
            }
            foreach (var reason in new[] { "zero impulse", "missing readback", "over budget" })
            {
                var log = new List<string>();
                var w = AtDeadline(Window.Mode.NearGroundHold8mm, reason == "zero impulse" ? 0f : .07f,
                    log.Add, checkedLoad: reason != "missing readback");
                var s = Boundary(min: .004f);
                s.Predicted = -.009f;
                if (reason == "over budget") s.Step.Dt = .251f;
                var end = w.Evaluate(s, false, out _);
                Check.That(end == Window.End.Timeout && !w.TryKeepAtBoundary(end), reason);
                w.Close(end, s.Local);
                Check.That(log[0].Contains("/wideForecastAtBase=False"), reason + " excluded in both arms");
            }
        }

        private static void WideForecastLifetime()
        {
            foreach (bool formal in new[] { false, true })
            foreach (var mode in new[] { Window.Mode.FallingForecastRise5mm, Window.Mode.NearGroundHold8mm })
            {
                var log = new List<string>();
                var w = AtDeadline(mode, .07f, log.Add);
                var s = Boundary(min: .004f, control: .013f);
                s.Predicted = -.009f;
                var end = w.Evaluate(s, false, out _);
                Check.That(end == Window.End.Timeout, "original 150 ms deadline");
                if (formal)
                {
                    w.BeginFormal(false);
                    w.Close(Window.End.Handoff, s.Local);
                    Check.That(!w.TryKeepAtBoundary(end) && log[0].Contains("/action=Handoff/atBaseMatch=Eligible/") &&
                        log[0].Contains("/extraSolves=0/") && log[0].Contains("/wideForecastAtBase=True"),
                        "formal stance preempts even a qualifying wide forecast");
                    continue;
                }
                Check.That(w.TryKeepAtBoundary(end) && !w.TryKeepAtBoundary(end), "one extension at first expiry");
                Schedule(w, s.Local);
                w.ResetTrace();
                for (int n = 6; n <= 9; n++)
                {
                    Check.That(w.Evaluate(Boundary(n), false, out _) == Window.End.None, "shared budget");
                    Schedule(w, n);
                }
                end = w.Evaluate(Boundary(10), false, out _);
                Check.That(end == Window.End.ExtensionLimit && !w.TryKeepAtBoundary(end), "250 ms exhausted");
                w.Close(end, 10);
                Check.That(log.Count == 2 && log[1].Contains("phase=END/") &&
                    log[1].Contains("/extraSolves=5/") && log[1].Contains("/wideForecastAtBase=True"),
                    "cutoff evidence follows the window through trace reset");
            }
        }

        private static void ExpiryEdges()
        {
            var edges = new (float min, float goal, float speed, float impulse, bool verified, bool eligible)[] {
                (0f, -.003f, -.1f, .2f, true, true), (.003f, -.003f, -.1f, .2f, true, true),
                (-.000001f, -.003f, -.1f, .2f, true, false), (.003001f, -.003f, -.1f, .2f, true, false),
                (.004f, -.003f, -.1f, .2f, true, false), (.005f, -.003f, -.1f, .2f, true, false),
                (.001f, -.002f, -.1f, .2f, true, false), (.001f, float.NaN, -.1f, .2f, true, false),
                (.001f, -.003f, 0f, .2f, true, false), (.001f, -.003f, -.1f, 0f, true, false),
                (.001f, -.003f, -.1f, float.NaN, true, false), (.001f, -.003f, -.1f, .2f, false, false) };
            foreach (var edge in edges)
            foreach (var mode in new[] { Window.Mode.FallingForecastRise5mm, Window.Mode.NearGroundHold8mm })
            {
                var w = AtDeadline(mode, edge.impulse, checkedLoad: edge.verified);
                var s = Boundary(min: edge.min, control: edge.goal); s.CornerV = edge.speed;
                Check.That(w.TryExtendAtExpiry(w.Evaluate(s, false, out _)) == edge.eligible, "cutoff boundary " + edge);
            }
            var log = new List<string>();
            var handoff = AtDeadline(Window.Mode.FallingForecastRise5mm, .1f, log.Add);
            var due = handoff.Evaluate(Boundary(), false, out _);
            handoff.BeginFormal(false); handoff.Close(Window.End.Handoff, 5);
            Check.That(!handoff.TryExtendAtExpiry(due) && log[0].Contains("/action=Handoff/atBaseMatch=Eligible"), "formal first refusal");
            var largeStep = AtDeadline(Window.Mode.FallingForecastRise5mm, .1f);
            var longDt = Boundary(); longDt.Step.Dt = .251f;
            Check.That(!largeStep.TryExtendAtExpiry(largeStep.Evaluate(longDt, false, out _)), "first extra solve cannot overrun");
        }

        private static Window StartExtension(List<string> log = null, Window.Mode mode = Window.Mode.FallingForecastRise5mm,
            float cutoffAnkleMeters = .101f)
        {
            var w = AtDeadline(mode, .12f, log == null ? null : log.Add);
            var cutoff = Boundary();
            cutoff.AnkleY = cutoffAnkleMeters;
            Check.That(w.TryExtendAtExpiry(w.Evaluate(cutoff, false, out _)), "granted once at the original cutoff");
            Check.That(!w.TryExtendAtExpiry(Window.End.Timeout), "cannot grant twice on one step");
            w.CountSolve(); w.CheckUnloaded(5, false);
            return w;
        }

        private static void ExtensionBudget()
        {
            var log = new List<string>(); var w = StartExtension(log);
            w.ResetTrace();
            float[] dts = { .04f, .06f, .10f };
            for (int i = 0; i < dts.Length; i++)
            {
                var s = Boundary(6+i, control: .001f); s.Step.Dt = dts[i]; s.CornerV = 0;
                Check.That(w.Evaluate(s, false, out _) == Window.End.None, "quiet supported foot can await stance");
                w.CountSolve(); w.CheckUnloaded(s.Local, false);
            }
            var end = w.Evaluate(Boundary(9), false, out _);
            Check.That(end == Window.End.ExtensionLimit && !w.TryExtendAtExpiry(end), "next solve would exceed 250 ms");
            w.Close(end, 9);
            Check.That(log.Count == 2 && log[1].Contains("phase=END") && log[1].Contains("/extraSolves=4") &&
                w.Summary().Contains("/baseExpiryLocal-extraSolves-extraMs=5|4|250"), "close records actual extra exposure");
            w.Evaluate(Boundary(10), false, out bool reopened);
            Check.That(!reopened, "same descent stays locked");
        }

        private static void ExtensionExits()
        {
            foreach (var why in new[] { Window.End.Lift, Window.End.Floor, Window.End.Clock, Window.End.Gap, Window.End.Depth,
                Window.End.Rising, Window.End.Stale, Window.End.Miss, Window.End.GoalLost })
            {
                var w = StartExtension(); var s = Boundary(6);
                switch (why)
                {
                    case Window.End.Lift: s.Lift = true; break;
                    case Window.End.Floor: s.Support++; break;
                    case Window.End.Clock: s.Step.Sequence++; break;
                    case Window.End.Gap: s.Gap = .030f; break;
                    case Window.End.Depth: s.Min = -.00301f; break;
                    case Window.End.Rising: s.Min += .001f; s.AnkleY += .001f; break;
                    case Window.End.Stale: s.Age = .101f; break;
                    case Window.End.GoalLost: s.ControlMin = float.NaN; break;
                }
                var end = w.Evaluate(s, why == Window.End.Miss, out _);
                Check.That(end == why && !w.TryExtendAtExpiry(end), "early exit " + why);
                w.Close(end, s.Local);
                Check.That(!w.Physical, "off after " + why);
            }
        }

        private static void ExpiryCoverage()
        {
            var log = new List<string>(); var w = new Window(log.Add); w.Select(Window.Mode.NearGroundHold8mm);
            for (int i = 0; i < 70; i++)
            {
                int n = 10*i;
                w.Evaluate(S(n+1, .024f, .05f), false, out _);
                w.Evaluate(S(n+2, .020f, .05f), false, out bool open);
                Check.That(open, "fresh descent " + i); w.Open(); w.CountSolve(); w.CheckUnloaded(n+2, false);
                w.Callback(n+2, 42, 0, 0);
                for (int step = n+3; step <= n+4; step++)
                {
                    w.Evaluate(S(step, .010f, .05f), false, out _); w.CountSolve(); w.CheckUnloaded(step, false);
                }
                var end = w.Evaluate(Boundary(n+5), false, out _); w.Close(end, n+5);
                for (int j = 0; j < 4; j++) w.Evaluate(Boundary(n+6+j, j == 1 ? -.004f : .001f), false, out _);
                w.Evaluate(S(n+10, .029f, .05f), false, out _);
            }
            Check.That(log.Count == 140 && w.Summary().Contains("/started-saved-omitted=70|64|6"), "all 70 cutoffs and follow-ups emitted");
            Check.That(log[139].Contains("/samples-last-firstBelowMinus2=4|699|697/"), "same-floor post-release crossing");
            var stopped = AtDeadline(Window.Mode.NearGroundHold8mm, 0f, log.Add);
            stopped.Close(stopped.Evaluate(Boundary(), false, out _), 5);
            stopped.Evaluate(Boundary(6), false, out _); stopped.BreakContinuity();
            Check.That(log[log.Count-1].Contains("/end=ContinuityEnded/samples-last-firstBelowMinus2=1|6|0"), "interrupted observation stays partial");
        }
    }
}
