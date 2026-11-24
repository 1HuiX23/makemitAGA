using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Entry = MakemitAGA.ActiveRagdollLab.Physics.OrdinaryFootEntry;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;

namespace ARL.Contracts
{
    internal static class CombinedLandingContracts
    {
        internal static void Run()
        {
            Check.Run("175 distinguish a fully observed failed first from unavailable geometry", ScoredAndCensored);
            Check.Run("175 paired callbacks retain raw inputs after older or combined paths stop", IndependentInput);
            Check.Run("175 incomplete and mismatched reads never authorize a combined path", Receipts);
            Check.Run("175 four strict 174 B releases retain causal joint and foot histories", () => Recorded(true));
            Check.Run("175 ten releases in 173 A/B keep the five complete buffered controls", () => Recorded(false));
        }

        private static Foot.Input NeutralCommand() => new Foot.Input {
            Tether = new RightAnkleLandingSequence.Tether { BaseCap = 60f, SelectedCap = 60f, Kp = 100f, Kd = 10f, HardGap = .03f } };

        private static void IndependentInput()
        {
            foreach (bool reverse in new[] { false, true })
            foreach (bool loseFoot in new[] { false, true })
            {
                const int start = 3, strong = 6, first = 9;
                var sim = new FootTransitionContracts.Sim { Reverse = reverse };
                sim.Frame(0, 0, 0);
                sim.Frame(0, 0, 0, (ref Trace.Tick l, ref Trace.Tick r) => l.Motion = r.Motion = "IDLE");
                var p = TurnHandoffContracts.Pose(-.0024f);
                var budget = TurnHandoffContracts.Budget();
                for (int n = start; n <= first; n++)
                {
                    PhysicsStepContext step = default;
                    sim.Frame(.02f, .03f, -.02f, (ref Trace.Tick l, ref Trace.Tick r) =>
                    {
                        l.Motion = r.Motion = "TURN";
                        TurnHandoffContracts.SetFree(ref l);
                        TurnHandoffContracts.SetFree(ref r);
                        if (n == start) { TurnHandoffContracts.SetRelease(ref l); TurnHandoffContracts.SetRelease(ref r); }
                        TurnHandoffContracts.SetPose(ref l, p, 0);
                        l.Source.World = Vector3.forward;
                        l.Entry = new Entry.Sample { Local = n, Floor = n >= strong ? l.Floor : 0, NativeSole = l.BoxMin,
                            NativeInside = l.Inside, LegacyReady = n >= strong, Confirmed = n == first, IdentityHeld = n == first };
                        if (n == first)
                        {
                            l.Phase = FootLandingOpportunityAudit.Phase.Stance;
                            l.Miss = l.PendingMiss = true;
                            l.EventFloor = l.Floor;
                            l.NativeDepthLatch = strong;
                        }
                        step = l.Step;
                    }, held: n == first - 1);
                    Check.That(sim.Trace.TryHandoff(0, step, out var tick, out var reactive, out var why, out var early, out var braked, out var combined),
                        "raw left stream remains reachable in either callback order");
                    if (n == start)
                    {
                        reactive.Stop("NoContinuation");
                        early.Stop("NoContinuation");
                        braked.Stop("NoContinuation");
                    }
                    else Check.That(reactive == null && early == null && braked == null, "strict window is sole remaining consumer");
                    bool issue = combined.Visit(tick, p, 22, 23, why);
                    Check.That(issue == (n < first), "native first consumes prior unloaded solve");
                    if (issue) combined.Issue(tick, p, budget, n == start + 1 && loseFoot ? default : NeutralCommand());
                    Check.That(!combined.Visit(tick, p, 22, 23, why), "duplicate callback cannot advance any trail");
                }
                string report = sim.Report();
                Check.That(report.Contains("/inputEnd=FirstOpportunity/inputs=6") &&
                    report.Split("TURN_COMBINED_INPUT_V041790/").Length == 7 && report.Split("/readBonesM=").Length == 7 &&
                    report.Contains("start-issue-read=3|8|9") && report.Contains("/entryLocal-floor-confirmed-ground-held-inside=9|6|True|True|True|True"),
                    "all native commands and surveyed endpoints retained through first");
                Check.That(!report.Contains("/legInputs=LeftBraked") && report.Contains("/rawA-appliedA=") && report.Contains("/nominalM(hip-upperKnee-lowerKnee-lowerAnkle-foot)="),
                    "raw inputs independently printable after braked stops");
                if (loseFoot)
                    Check.That(report.Contains("/sequenceEnd=MissingFootCommand/stopLocal=4") &&
                        report.Contains("/lastSafeRead=4/allMarginsAtNativeFirst=Unavailable") &&
                        report.Contains("/retainedBufferedShoe-preAlso-lowerAlso-bothAlso=Unavailable") && report.Split("J179/").Length == 2,
                        "raw reporting cannot reseed a terminated trial");
                else Check.That(report.Contains("/sequenceEnd=WouldEnter/stopLocal=9") &&
                    report.Contains("/lastSafeRead=9/allMarginsAtNativeFirst=True") && report.Contains("/strongSteps-marginPass-readySamples=4|4|4"),
                    "four buffered reads on one continuous path alongside native miss");
            }
        }

        private static void ScoredAndCensored()
        {
            const int start = 3, strong = 6, first = 9;
            // The native shoe is just below the ordinary floor. Tiny traction preserves
            // safe tracking but cannot clear it; an invalid starting gap prevents any forecast.
            foreach (bool noForecast in new[] { false, true })
            {
                var p = TurnHandoffContracts.Pose(-.0024f);
                var command = NeutralCommand();
                if (noForecast)
                {
                    p.Lower = p.Foot - Vector3.right * .008f;
                    command.Tether.HardGap = .006f;
                }
                else command.Tether.BaseCap = command.Tether.SelectedCap = .0001f;
                var w = new TurnHandoffWindow(start, Vector3.forward, TurnHandoffWindow.Policy.LeftCombined);
                Trace.Tick t = default;
                for (int n = start; n <= first; n++)
                {
                    t = ContinuousLandingContracts.Tick(n, p, strong, first, strong);
                    if (w.Visit(t, p, 22, 23, null)) w.Issue(t, p, TurnHandoffContracts.Budget(), command);
                }
                var b = new StringBuilder();
                w.AppendCombined(b, t, "HoldEnded", 0, strong, null);
                string log = b.ToString();
                Check.That(log.Contains("/inputEnd=FirstOpportunity/inputs=6") && log.Contains("start-issue-read=3|8|9"),
                    "physical history outlives geometric stop");
                if (noForecast)
                    Check.That(log.Contains("/sequenceEnd=NoSafeFootForecast/stopLocal=3") && log.Contains("/allMarginsAtNativeFirst=Unavailable") &&
                        !log.Contains("J179/"), "zero legal foot commands cannot be graded or reseeded");
                else Check.That(log.Contains("/sequenceEnd=DepthLatched/stopLocal=9") && log.Contains("/allMarginsAtNativeFirst=False") &&
                    log.Contains("/strongStart=6/trialDepthLatch=6"), "complete but failed ordinary opportunity receives an explicit negative grade");
            }
        }

        private static void Receipts()
        {
            foreach (string expected in new[] { "ClockMismatch", "BaselineMismatch", "IssueIdentity", "EntryReceipt", "EntryGeometryMismatch",
                "NativeDisagreement", "BodyOrBoxChanged", "MissingCompletion", "RunEnd" })
            {
                var p = TurnHandoffContracts.Pose(-.0024f);
                var w = new TurnHandoffWindow(1, Vector3.forward, TurnHandoffWindow.Policy.LeftCombined);
                var t = ContinuousLandingContracts.Tick(1, p, 2, 2, 2);
                Check.That(w.Visit(t, p, 22, 23, null), "valid release");
                if (expected == "IssueIdentity") t.Source.Clock.Sequence++;
                w.Issue(t, p, TurnHandoffContracts.Budget(), NeutralCommand());
                var readPose = p;
                if (expected == "BaselineMismatch") readPose.Hip.y += .003f;
                if (expected == "BodyOrBoxChanged") readPose.Half.x += .001f;
                t = ContinuousLandingContracts.Tick(expected == "MissingCompletion" ? 3 : 2, readPose, 2, 2, 2);
                if (expected == "ClockMismatch") t.Step.SourceSequence++;
                if (expected == "EntryReceipt") t.Entry.Floor++;
                if (expected == "EntryGeometryMismatch") t.Entry.NativeSole += .001f;
                if (expected == "NativeDisagreement") t.NativeDepthLatch++;
                if (expected == "RunEnd") w.Stop(expected);
                else Check.That(!w.Visit(t, readPose, 22, 23, null), "mismatched read cannot authorize next solve");
                var b = new StringBuilder();
                w.AppendCombined(b, t, "HoldEnded", 0, 2, null);
                string log = b.ToString();
                Check.That(w.End == expected && log.Contains("/sequenceEnd=" + expected) && log.Contains("/allMarginsAtNativeFirst=Unavailable") &&
                    !log.Contains("readGate=Accepted"), "reject " + expected);
                if (expected == "RunEnd") Check.That(log.Contains("start-issue-read=1|1|0") && log.Contains("/readGate=Unread"),
                    "no retroactive score for unfinished read");
            }
        }

        private static float Number(string text) => float.Parse(text, CultureInfo.InvariantCulture);
        private static Vector3 Vector(string text)
        {
            var xyz = text.Split(',');
            return new Vector3(Number(xyz[0]), Number(xyz[1]), Number(xyz[2]));
        }

        internal static Foot.Input Command(string[] row, bool hardGapInMillimetres = true)
        {
            if (row[15].Length == 0) return default;
            var force = row[15].Split('|');
            var terms = row[16].Split('|').Select(Number).ToArray();
            return new Foot.Input { Applied = Vector(force[1]), Drag = terms[4],
                Tether = new RightAnkleLandingSequence.Tether { Raw = Vector(force[0]), BaseCap = terms[0],
                    SelectedCap = terms[1], Kp = terms[2], Kd = terms[3], HardGap = hardGapInMillimetres ? terms[5] * .001f : terms[5] } };
        }

        private static void Recorded(bool recent)
        {
            var name = recent ? "CombinedLanding174" : "ContinuousLanding173";
            using var reader = new StreamReader(typeof(CombinedLandingContracts).Assembly.GetManifestResourceStream("ARL.Contracts." + name));
            string line = reader.ReadLine();
            var rows = new List<string[]>();
            while ((line = reader.ReadLine()) != null) rows.Add(line.Split('\t'));
            int complete = 0, groups = 0;
            foreach (var release in rows.GroupBy(c => (c[0], recent ? "B" : c[23])))
            {
                if (Replay(release.ToArray(), recent)) complete++;
                groups++;
            }
            Check.That(rows.Count == (recent ? 46 : 103) && groups == (recent ? 4 : 10) &&
                complete == (recent ? 2 : 5), "all input rows, complete paths and still-censored releases");
        }

        private static bool Replay(string[][] rows, bool recent)
        {
            var search = new CombinedLandingSearch(usePreview: false);
            CombinedLandingSearch.Step previous = null;
            int start = int.Parse(rows[0][0]), strong = int.Parse(rows[0][20]), first = int.Parse(rows[0][21]);
            var issuedIds = new HashSet<int>();
            var initial = TurnLandingContracts.DecodeIssue(rows[0]);
            var pose = initial.Pose;
            pose.Hip.y = initial.HipY;
            pose.Lower = pose.Foot - Vector(rows[0][17]);
            var window = new TurnHandoffWindow(start, initial.Issue.Travel, TurnHandoffWindow.Policy.LeftCombined, usePreview: false);
            var tick = ContinuousLandingContracts.Tick(start, pose, strong, first);
            tick.Step.Dt = initial.Dt;
            tick.Target = initial.Issue.Target;
            tick.SourceSole = initial.Issue.TargetSole;
            Check.That(window.Visit(tick, pose, 22, 23, null), "recorded release reaches the native reader");
            int surface = 0;
            foreach (var row in rows)
            {
                var s = TurnLandingContracts.DecodeIssue(row);
                int to = start + int.Parse(row[1]), from = to - 1;
                var command = Command(row);
                var gap = Vector(row[17]);
                Check.That((tick.Target - s.Issue.Target).magnitude < .000002f &&
                    (tick.SourceSole - s.Issue.TargetSole).magnitude < .000002f, "issue aim agrees with this native tick");
                window.Issue(tick, s.Pose, s.Budget, command);
                var step = search.Issue(from, s.Dt, s.HipY, s.Pose, s.Issue, s.Budget, command, gap, previous);
                Check.That(step.Count <= CombinedLandingSearch.Width && step.Count <= step.Forecasts, "bounded unique freeze");
                for (int i = 0; i < step.Count; i++)
                {
                    var path = step.Paths[i];
                    var parent = previous == null ? default : previous.Paths.Take(previous.Count).Single(p => p.Id == path.Parent);
                    Check.That((previous == null ? path.Parent == 0 : parent.SafeRead) && issuedIds.Add(path.Id), "unique issued child of its own safe read");
                    Check.That(!path.SafeRead && path.Move.Status == Foot.State.Predicted, "future endpoint still unknown");
                    var check = Foot.Predict(s.Pose, path.Leg, s.Issue, command, s.Dt, gap, parent.Move, previous != null, true);
                    Check.Near((path.Move.Body.Position - check.Body.Position).magnitude, 0f, 1e-8f, "own carried displacement");
                    Check.Near((path.Move.Body.Velocity - check.Body.Velocity).magnitude, 0f, 1e-8f, "own carried velocity");
                    Check.Near(path.Leg.Offset.Hip, parent.Leg.Offset.Hip + path.Leg.Offset.HipRate * s.Dt, 1e-6f, "own hip posture and rate");
                    Check.Near(path.Leg.Offset.Knee, parent.Leg.Offset.Knee + path.Leg.Offset.KneeRate * s.Dt, 1e-6f, "own knee posture and rate");
                }
                if (row[9].Length == 0)
                {
                    int expectedOptions = start == 2589 ? 4 : 8;
                    Check.That(search.LastReadLocal == from && search.EndLocal == from && step.Parents == 4 &&
                        step.LegOptions == expectedOptions && step.Forecasts == 0, "separate next leg choices from missing foot at " + from);
                    Check.That(!command.Valid && step.Count == 0 && step.LegOptions > 0 && search.Outcome == "MissingFootCommand",
                        "documented angular continuation, missing ankle command at " + from);
                    break;
                }
                var read = TurnLandingContracts.ReadAim(row[9], s);
                var native = Leg.Evaluate(s.Pose, default, read);
                Check.Near(native.Sole * 1000f, Number(row[22]), .002f, "recorded physical sole reproduced at " + to);
                var receipt = new Entry.Sample { Local = to, NativeSole = Number(row[22]) * .001f, NativeInside = native.Inside,
                    Floor = to >= strong ? 3 : 0, Confirmed = to == first, IdentityHeld = to == first, LegacyReady = to >= strong };
                if (recent)
                {
                    receipt.NativeInside = (int.Parse(row[27]) & 8) != 0;
                    Check.That(receipt.NativeInside == native.Inside, "native H144 footprint matches the measured geometry");
                    var flags = row[23].Split('|');
                    if (flags.Length == 4)
                    {
                        receipt.Floor = int.Parse(flags[0]);
                        if (receipt.Floor != 0)
                        {
                            Check.That(surface == 0 || surface == receipt.Floor, "same native ground identity throughout the release");
                            surface = receipt.Floor;
                        }
                        receipt.Confirmed = bool.Parse(flags[1]);
                        receipt.LegacyReady = bool.Parse(flags[2]);
                        receipt.IdentityHeld = bool.Parse(flags[3]);
                    }
                    else
                    {
                        // H144 supplies strong stance, footprint and shoe height at this endpoint.
                        // The legacy ground veto was not printed. Do not grant an extra ready sample.
                        Check.That(to == 2600 && (int.Parse(row[27]) & 15) == 15, "partial native receipt");
                        Check.That(surface != 0, "last directly captured ground identity known");
                        receipt.Floor = surface;
                        receipt.LegacyReady = false;
                    }
                }
                search.Read(to, step, s.Pose, read, native, receipt, to == first);
                tick = ContinuousLandingContracts.Tick(to, s.Pose, strong, first, strong);
                tick.Step.Dt = s.Dt;
                tick.Target = read.Target;
                tick.SourceSole = read.TargetSole;
                tick.BoxMin = receipt.NativeSole;
                tick.Entry = receipt;
                // The test tick maps this release's verified native collider to its fixture-local ID.
                tick.Entry.Floor = receipt.Floor == 0 ? 0 : tick.Floor;
                Check.That(window.Visit(tick, s.Pose, 22, 23, null) == (to < first), "native clock, geometry and ordinary oracle agree at " + to);
                if (recent && row[24].Length != 0)
                {
                    var control = step.Paths.Take(step.Count).Single(p => p.Protected);
                    Check.That(control.SafeRead, "174 control retained through its last known foot read");
                    Check.Near((control.Move.Body.Position - Vector(row[24])).magnitude, 0f, .000002f, "logged control position");
                    Check.Near((control.Move.Body.Velocity - Vector(row[25])).magnitude, 0f, .00002f, "logged control velocity");
                }
                if (to == first)
                {
                    var control = step.Paths.Take(step.Count).Single(p => p.Protected);
                    Check.That(control.SafeRead && control.Entry.StrongSamples == 4 && control.MarginSamples == 4 &&
                        control.Entry.ReadySamples >= 2 && control.Entry.DepthLatch == 0, "same control covers four buffered endpoints");
                    Check.That(search.Outcome == "WouldEnter" && search.EndLocal == first, "hypothetical entry on native decision, no later free motion");
                }
                if (recent && to == 2600)
                    Check.That(step.SafeReads == 4 && step.Paths.Take(step.Count).Count(p => p.SafeRead && p.FirstPreExcess == 0) == 1 &&
                        !search.LastRead.Protected && search.LastRead.Entry.StrongSamples == 3 &&
                        search.LastRead.MarginSamples == 3 && search.LastRead.LeastMargin >= Foot.LandingMargin &&
                        search.LastRead.Entry.ReadySamples == 0, "four final shoes and one strict pre-traction lineage; third geometric margin lacks ground evidence");
                if (recent && to == 3983)
                    Check.That(step.SafeReads == 4 && search.LastRead.Entry.StrongSamples == 0,
                        "four alternatives read safely before the first strong stance");
                previous = step;
            }
            bool complete = search.Outcome == "WouldEnter";
            if (window.End == null) window.Stop("EndOfRecordedInputs");
            var firstTick = ContinuousLandingContracts.Tick(first, initial.Pose, strong, first, strong);
            var log = new StringBuilder();
            window.AppendCombined(log, firstTick, "HoldEnded", 0, strong, null);
            var summary = log.ToString().TrimStart().Split('\n')[0];
            Check.That(summary.Contains("/allMarginsAtNativeFirst=" + (complete ? "True" : "Unavailable")) &&
                summary.Contains("/sequenceEnd=" + (search.Outcome ?? "EndOfRecordedInputs")), "window reports independently reproduced endpoint");
            return complete;
        }
    }
}
