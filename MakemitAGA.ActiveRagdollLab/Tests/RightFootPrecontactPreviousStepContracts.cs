using System;
using System.Collections.Generic;
using Window = MakemitAGA.ActiveRagdollLab.Physics.RightFootPrecontact;

namespace ARL.Contracts
{
    internal static partial class RightFootPrecontactContracts
    {
        private const float ReviewDt = 1f / 60f;

        private static void RunPreviousStepExpiryContracts()
        {
            Check.Run("At the three recorded v162 B cutoffs only the new arm schedules the extra solve", RecordedPreviousStepCutoffs);
            Check.Run("The previous-step cutoff uses only adjacent qualifying evidence and current hard limits", PreviousStepEdges);
            Check.Run("Formal entry, miss and native readback own the first review and its next solve", PreviousStepOwnership);
            Check.Run("A single first-expiry review shares the original near-ground range and time budget", PreviousStepBudget);
        }

        // Measured signed clearances and ankle position in mm, signed lowest-corner speed in mm/s.
        private static Window.Sample ReviewPose(int local, float h, float goal, float prediction, float speed, float ankle = 110f)
        {
            var s = S(local, h / 1000f, ReviewDt);
            s.ControlMin = goal / 1000f;
            s.Predicted = prediction / 1000f;
            s.CornerV = speed / 1000f;
            s.AnkleY = ankle / 1000f;
            return s;
        }

        private static Window BeforeFirstExpiry(Window.Mode mode, Window.Sample prior, Action<string> log = null,
            float impulse = .01f, int impulseStep = 0, int skipReceipt = 0)
        {
            int origin = prior.Local - 8;
            var w = new Window(log);
            w.Select(mode);
            w.Evaluate(S(origin - 1, .016f, ReviewDt), false, out _);
            for (int i = origin; i <= prior.Local; i++)
            {
                var s = i == prior.Local ? prior : S(i, i == origin ? .012f : .010f, ReviewDt);
                Check.That(w.Evaluate(s, false, out bool open) == Window.End.None, "original 150 ms window");
                if (i == origin)
                {
                    Check.That(open, "real two-step candidate");
                    w.Open();
                }
                w.CountSolve();
                if (i != skipReceipt) Check.That(w.CheckUnloaded(i, false), "receipt belongs to this solve");
                if (i == (impulseStep == 0 ? origin : impulseStep)) w.Callback(i, 42, impulse, 0f);
            }
            return w;
        }

        private static void RecordedPreviousStepCutoffs()
        {
            var cases = new[] {
                (ReviewPose(3491, 1.279086f, -.8111075f, -4.14224f, -302.1809f, 112.1948f),
                 ReviewPose(3492, .2559274f, .6117076f, -.8462814f, -61.38194f, 111.8827f),
                 ReviewPose(3493, -.8685365f, -2.030425f, -2.024177f, -69.33843f, 111.3058f), 3486, .00539f, "GoalClear"),
                (ReviewPose(4297, .07467717f, -6.319322f, -.5041376f, -4.877895f, 115.7595f),
                 ReviewPose(4298, .05254149f, -9.070165f, -.005667913f, .798285f, 112.8529f),
                 ReviewPose(4299, -.8385032f, -12.07048f, -8.832801f, -51.9602f, 109.0827f), 4294, .01019f, "NotDescending"),
                (ReviewPose(4958, 1.241021f, 1.198821f, -4.381429f, -293.6441f, 111.4566f),
                 ReviewPose(4959, .2484843f, -1.127504f, -.8927338f, -59.53336f, 111.0032f),
                 ReviewPose(4960, -.6906092f, -4.948929f, -1.682442f, -59.50994f, 110.1686f), 4954, .003166f, "GoalClear")
            };
            foreach (var (prior, due, releasedNext, impulseStep, impulse, rejection) in cases)
            foreach (var mode in new[] { Window.Mode.NearGroundHold8mm, Window.Mode.PreviousStepExpiryReview })
            {
                var log = new List<string>();
                var w = BeforeFirstExpiry(mode, prior, log.Add, impulse, impulseStep);
                var end = w.Evaluate(due, false, out _);
                bool grant = mode == Window.Mode.PreviousStepExpiryReview;
                Check.That(end == Window.End.Timeout && w.TryKeepAtBoundary(end) == grant, "cutoff " + due.Local);
                if (!grant) w.Close(end, due.Local);
                Check.That(log.Count == 1 && log[0].Contains("/action=" + (grant ? "Extend" : "Timeout") + "/atBaseMatch=" + rejection + "/") &&
                    log[0].Contains("/previousStepCandidate=True/") && log[0].Contains("cornerMmSec=" + prior.Local + "|"),
                    "candidate, previous measurements and actual action");
                if (!grant) continue;
                Check.That(!w.TryKeepAtBoundary(end), "grant once");
                Schedule(w, due.Local);
                w.ResetTrace();
                // This is the recorded contact-OFF trajectory. A real held solve supplies its own next measurements.
                end = w.Evaluate(releasedNext, false, out _);
                Check.That(end == Window.End.NearGroundExit && !w.TryKeepAtBoundary(end), "old trajectory outside lower edge");
                w.Close(end, releasedNext.Local);
                Check.That(log.Count == 3 && log[1].Contains("BASE_REVIEW_V041630] phase=NEXT/") &&
                    log[1].Contains("/action=NearGroundExit/") && log[1].Contains("/extraSolves=1/") &&
                    log[2].Contains("phase=END/"), "measured next solve emitted once across trace reset");
            }
        }

        private static void PreviousStepEdges()
        {
            var prior = ReviewPose(18, 1.5f, -4f, 0f, -20f);
            var due = ReviewPose(19, .5f, 0f, 0f, -5f);
            bool Grants(Window.Sample p, Window.Sample s, float impulse = .01f, int skip = 0)
            {
                var w = BeforeFirstExpiry(Window.Mode.PreviousStepExpiryReview, p, impulse: impulse, skipReceipt: skip);
                var end = w.Evaluate(s, false, out _);
                return w.TryKeepAtBoundary(end);
            }

            Check.That(Grants(prior, ReviewPose(19, 1f, 0f, 0f, -1f)) && Grants(prior, ReviewPose(19, 0f, 0f, 0f, 0f)), "inclusive [0, +1] mm");
            Check.That(!Grants(prior, ReviewPose(19, 1.001f, 0f, 0f, -5f)) && !Grants(prior, ReviewPose(19, -.0005f, 0f, 0f, -5f)),
                "zero tolerance is only for continuations after the cutoff");
            Check.That(!Grants(ReviewPose(18, .5f, -4f, 0f, -20f), due) && !Grants(ReviewPose(18, .4f, -4f, 0f, -20f), due),
                "height must strictly decrease across the two observations");
            Check.That(Grants(ReviewPose(18, 5f, 0f, -3f, -1f), due) && !Grants(ReviewPose(18, 5f, -5f, -2f, -1f), due) &&
                !Grants(ReviewPose(18, 5.01f, -5f, -3f, -1f), due), "original wide forecast rule is reused on the previous solve");
            Check.That(!Grants(ReviewPose(18, 1.5f, -2f, -2f, -1f), due) && !Grants(ReviewPose(18, 1.5f, -4f, -3f, 0f), due) &&
                !Grants(ReviewPose(18, 1.5f, float.NaN, -3f, -5f), due), "strict threshold, signed speed and readable prior goal");
            Check.That(!Grants(prior, ReviewPose(19, .5f, float.NaN, 0f, 1f)), "current goal must still be readable");
            var held = due;
            held.Source = prior.Source;
            held.Step.SourceSequence = held.Source.Sequence;
            Check.That(Grants(prior, held), "two physics solves may use the same fresh render sample");
            held.Source.Time += .001f;
            Check.That(!Grants(prior, held), "changed stamp on a held render sample");
            Check.That(!Grants(prior, due, 0f) && !Grants(prior, due, float.NaN) &&
                !Grants(prior, due, skip: 10) && !Grants(prior, due, skip: prior.Local), "historical impulse and every unload receipt required");
            // At 25 ms the penultimate eligible sample is two solves old when the base window ends.
            var w = Open(Window.Mode.PreviousStepExpiryReview, .025f);
            w.CheckUnloaded(2, false); w.Callback(2, 42, .01f, 0f);
            for (int i = 3; i <= 7; i++)
            {
                var s = S(i, i == 6 ? .0015f : i == 7 ? .0007f : .005f, .025f);
                s.ControlMin = i == 6 ? -.004f : 0f;
                s.Predicted = 0f;
                Check.That(w.Evaluate(s, false, out _) == Window.End.None, "six base solves");
                Schedule(w, i);
            }
            var clear = S(8, .0005f, .025f); clear.ControlMin = clear.Predicted = 0f;
            Check.That(!w.TryKeepAtBoundary(w.Evaluate(clear, false, out _)), "two-solve-old evidence cannot grant");
            var loaded = BeforeFirstExpiry(Window.Mode.PreviousStepExpiryReview, prior);
            Check.That(!loaded.CheckUnloaded(prior.Local, true) && !loaded.TryKeepAtBoundary(loaded.Evaluate(due, false, out _)), "positive native load");
            var longStep = due; longStep.Step.Dt = .251f;
            Check.That(!Grants(prior, longStep), "first solve must fit current budget");
            foreach (string failure in new[] { "clock", "floor", "age", "lift", "gap" })
            {
                var s = due;
                if (failure == "clock") s.Step.Sequence++;
                if (failure == "floor") s.Support++;
                if (failure == "age") s.Age = .101f;
                if (failure == "lift") s.Lift = true;
                if (failure == "gap") s.Gap = .030f;
                Check.That(!Grants(prior, s), failure);
            }
        }

        private static void PreviousStepOwnership()
        {
            var p = ReviewPose(18, 1.5f, -4f, 0f, -20f);
            var due = ReviewPose(19, .5f, 0f, 0f, -.2f);
            foreach (var close in new[] { Window.End.Handoff, Window.End.Miss })
            {
                var log = new List<string>();
                var w = BeforeFirstExpiry(Window.Mode.PreviousStepExpiryReview, p, log.Add);
                var end = w.Evaluate(due, false, out _);
                if (close == Window.End.Handoff) w.BeginFormal(false);
                w.Close(close, due.Local);
                Check.That(!w.TryKeepAtBoundary(end) && log.Count == 1 && log[0].Contains("/previousStepCandidate=True/") &&
                    log[0].Contains("/extraSolves=0/") && log[0].Contains("/action=" + close + "/"), "formal owner decides before first grant");
            }
            foreach (var decision in new[] { Window.End.Handoff, Window.End.Fault, Window.End.Miss })
            {
                var log = new List<string>();
                var w = BeforeFirstExpiry(Window.Mode.PreviousStepExpiryReview, p, log.Add, impulseStep: p.Local);
                Check.That(w.TryKeepAtBoundary(w.Evaluate(due, false, out _)), "completed prior solve may supply first impulse");
                w.CountSolve();
                if (decision != Window.End.Fault) w.CheckUnloaded(due.Local, false);
                var next = ReviewPose(20, -.8f, -2f, -2.5f, -10f);
                var end = w.Evaluate(next, decision == Window.End.Miss, out _);
                if (decision == Window.End.Handoff)
                {
                    Check.That(end == Window.End.NearGroundExit, "range exit is deferred for formal first refusal");
                    w.BeginFormal(true);
                    end = Window.End.Handoff;
                }
                Check.That(end == decision, "early exit wins");
                w.Close(end, next.Local);
                Check.That(log.Count == 3 && log[1].Contains("phase=NEXT/") && log[1].Contains("/action=" + decision + "/"), "actual next decision");
            }
        }

        private static void PreviousStepBudget()
        {
            var log = new List<string>();
            var w = BeforeFirstExpiry(Window.Mode.PreviousStepExpiryReview, ReviewPose(18, 1.5f, -4f, 0f, -20f), log.Add);
            var end = w.Evaluate(ReviewPose(19, .5f, 0f, 0f, 0f), false, out _);
            Check.That(w.TryKeepAtBoundary(end), "candidate granted");
            Schedule(w, 19);
            for (int i = 20; i <= 33; i++)
            {
                var s = ReviewPose(i, i == 20 ? 8f : -.0005f, 0f, 0f, 5f);
                Check.That(w.Evaluate(s, false, out _) == Window.End.None, "ordinary extension range after the held solve");
                Schedule(w, i);
            }
            end = w.Evaluate(ReviewPose(34, .5f, 0f, 0f, 0f), false, out _);
            Check.That(end == Window.End.ExtensionLimit && !w.TryKeepAtBoundary(end), "15 total extra solves");
            w.Close(end, 34);
            Check.That(log.Count == 3 && log[1].Contains("phase=NEXT/") && log[1].Contains("/action=Continue/") &&
                log[1].Contains("/elapsed-extra-nextDtMs=166.6667|16.66667|16.66667/") &&
                log[1].Contains("/scheduled-unloaded-violations=10|10|0/") &&
                log[2].Contains("/extraSolves=15/"), "one next-step event, one budget");

            w = BeforeFirstExpiry(Window.Mode.PreviousStepExpiryReview, ReviewPose(18, 1.5f, -4f, 0f, -20f));
            var due = ReviewPose(19, .5f, 0f, 0f, 0f);
            due.Step.Dt = .125f;
            Check.That(w.TryKeepAtBoundary(w.Evaluate(due, false, out _)), "charge the actual first step size");
            Schedule(w, 19);
            var next = ReviewPose(20, .3f, 0f, 0f, 0f);
            next.Step.Dt = .125f;
            Check.That(w.Evaluate(next, false, out _) == Window.End.None, "two 125 ms solves fit");
            Schedule(w, 20);
            Check.That(w.Evaluate(ReviewPose(21, .3f, 0f, 0f, 0f), false, out _) == Window.End.ExtensionLimit,
                "a shorter step still exceeds the shared budget");
        }
    }
}
