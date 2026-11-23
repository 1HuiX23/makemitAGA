using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class TurnLandingContracts
    {
        internal static void Run()
        {
            Check.Run("170 original 169 B command receipts reproduce both rules on all 15 endpoints", Recorded);
            Check.Run("170 first native miss or ordinary entry settles the last free solve in both callback orders", Links);
            Check.Run("170 censored trajectory remains unavailable while native gaps and the other path continue", Censored);
            Check.Run("170 a calibrated endpoint straddling the lower threshold agrees between row and totals", ThresholdAgreement);
            Check.Run("170 wrong issuance identity, body, horizon and measured vertical bounds", Boundaries);
        }

        private static float F(string x) => float.Parse(x, CultureInfo.InvariantCulture);
        private static Vector3 V(string x)
        {
            var v = x.Split(',').Select(F).ToArray();
            return new Vector3(v[0], v[1], v[2]);
        }
        private static Vector3[] Vectors(string x) => x.Split('|').Select(V).ToArray();

        // World positions, nominal commands and caps come from 169 B. The final
        // measured animation targets are supplied only after the offset has been chosen.
        private static void Recorded()
        {
            using var file = typeof(TurnLandingContracts).Assembly.GetManifestResourceStream("ARL.Contracts.TurnLanding169");
            using var reader = new StreamReader(file);
            reader.ReadLine();
            string row;
            int rows = 0, start = 0;
            int[,] passed = new int[2, 3];
            var prior = new Trial.Offset[2];
            var example = new RecordedSolve[3];
            while ((row = reader.ReadLine()) != null)
            {
                var cells = row.Split('\t');
                int origin = int.Parse(cells[0]), depth = int.Parse(cells[1]);
                if (origin != start) { Check.That(depth == 1, "new release"); Array.Clear(prior, 0, 2); start = origin; }
                var sample = DecodeIssue(cells);
                var pose = sample.Pose; var issued = sample.Issue; var budget = sample.Budget;
                for (int lane = 0; lane < 2; lane++)
                {
                    var plan = Trial.Select(pose, issued, budget, F(cells[2]), F(cells[3]), prior[lane], lane == 1);
                    Check.That(plan.Issued, "complete trajectory"); prior[lane] = plan.Offset;
                    var read = ReadAim(cells[9], sample);
                    if (origin == 3998 && lane == 1) example[depth - 1] = new RecordedSolve
                    { Pose = pose, Budget = budget, Issue = issued, Read = read, HipY = F(cells[3]), Dt = F(cells[2]) };
                    var actual = Trial.Evaluate(pose, default, read);
                    var result = Trial.Evaluate(pose, plan.Offset, read);
                    Check.That(result.Tracks(actual) && result.Safe && result.MaintainsHeight(actual), "native constraints at readback");
                    Check.Near(1000f * result.Sole, F(cells[15 + lane]), .002f, "measured shoe corner " + origin + "/" + depth);
                    if (result.GeometricPass && plan.Trial.GeometricPass) passed[lane, depth-1]++;
                }
                rows++;
            }
            Check.That(rows == 15 && passed[0,0] == 5 && passed[1,0] == 5 && passed[0,1] == 4 && passed[1,1] == 4 &&
                passed[0,2] == 1 && passed[1,2] == 3, "certified third-solves, genuine 5 mm gaps preserved");
            foreach (string outcome in new[] { "retained", "failed", "unavailable" }) TransferToFirst(example, outcome);
        }

        internal static RecordedSolve DecodeIssue(string[] cells)
        {
            var bones = Vectors(cells[5]); var rotation = cells[6].Split(',').Select(F).ToArray();
            var box = Vectors(cells[7]); var axes = Vectors(cells[11]);
            var pose = new Trial.Pose { Hip = bones[0], UpperKnee = bones[1], LowerKnee = bones[2], Lower = bones[3], Foot = bones[4],
                Rotation = new Quaternion(rotation[0],rotation[1],rotation[2],rotation[3]), Axis = axes[1],
                Center = box[0], Half = box[1], FloorCenter = box[2], FloorHalf = box[3], AllowExtension = bool.Parse(cells[10]) };
            var aims = Vectors(cells[8]); var issued = new Trial.Aim { Target = aims[0], TargetSole = aims[1], Travel = V(cells[4]) };
            var a = cells[12].Split('|'); var rates = Vectors(cells[13]); var caps = cells[14].Split('|').Select(F).ToArray();
            var budget = new Trial.Budget { Axis = axes[0], HipA = V(a[0]), KneeA = F(a[1]), HipCap = F(a[2]), KneeCap = F(a[3]),
                UpperW = rates[0], LowerW = rates[1], FootW = rates[2], UpperMaxW = caps[0], LowerMaxW = caps[1], FootMaxW = caps[2] };
            return new RecordedSolve { Pose = pose, Budget = budget, Issue = issued, HipY = F(cells[3]), Dt = F(cells[2]) };
        }

        internal static Trial.Aim ReadAim(string field, RecordedSolve sample)
        {
            var pose = sample.Pose; var issued = sample.Issue;
            var source = field.Split(':'); var values = Vectors(source[1]); var read = issued;
            if (source[0] == "target") { read.Target = values[0]; read.TargetSole = values[1]; }
            else
            {
                Check.That(source[0] == "errorMm", "measured error");
                read.Target = pose.Foot - values[0] * .001f;
                read.TargetSole = pose.Foot + pose.Rotation * (pose.Center - Vector3.up * pose.Half.y) - values[1] * .001f;
            }
            return read;
        }

        internal struct RecordedSolve
        {
            internal Trial.Pose Pose;
            internal Trial.Budget Budget;
            internal Trial.Aim Issue, Read;
            internal float HipY, Dt;
        }

        private static void TransferToFirst(RecordedSolve[] solves, string outcome)
        {
            // Reuse 3998's genuine third-step gain. The native entry and any fourth
            // solve below are synthetic, to exercise each transfer denominator.
            var sim = new FootTransitionContracts.Sim();
            sim.Frame(0, 0, 0); sim.Frame(0, 0, 0, (ref Trace.Tick l, ref Trace.Tick r) => l.Motion = r.Motion = "IDLE");
            var fourth = solves[2].Pose;
            fourth.Lower = fourth.Foot + Vector3.right * .012f;
            int first = outcome == "retained" ? 6 : 7;
            for (int local = 3; local <= first; local++)
            {
                int i = local - 3;
                var pose = i == 0 ? solves[0].Pose : i == 4 ? fourth : solves[i - 1].Pose;
                if (i == 0) pose.Hip.y = solves[0].HipY;
                var aim = i == 0 ? solves[0].Issue : solves[Math.Min(i - 1, 2)].Read;
                PhysicsStepContext step = default;
                sim.Frame(.02f, .03f, -.02f, (ref Trace.Tick l, ref Trace.Tick r) =>
                {
                    TurnHandoffContracts.SetFree(ref l); TurnHandoffContracts.SetFree(ref r);
                    if (i == 0) { TurnHandoffContracts.SetRelease(ref l); TurnHandoffContracts.SetRelease(ref r); }
                    l.Motion = r.Motion = "TURN"; l.Step.Dt = r.Step.Dt = solves[0].Dt;
                    l.Source.World = aim.Travel; l.Target = aim.Target; l.SourceSole = aim.TargetSole;
                    l.Foot = pose.Foot; l.Lower = pose.Lower; l.BoxMin = Trial.Evaluate(pose, default, aim).Sole;
                    l.Sole = pose.Foot + pose.Rotation * (pose.Center - Vector3.up * pose.Half.y);
                    if (local == first) { l.Miss = l.PendingMiss = true; l.EventFloor = l.Floor; l.Phase = Phase.Stance; }
                    step = l.Step;
                });
                Check.That(sim.Trace.TryHandoff(0, step, out var tick, out var standard, out string why, out var early, out _, out _), "recorded trace work");
                bool issue = standard.Visit(tick, pose, 30, 31, why);
                bool issueEarly = early?.Visit(tick, pose, 30, 31, why) == true;
                if (local == first) { Check.That(!issue && !issueEarly, "entry consumes previous solve"); continue; }
                var forecast = i < 3 ? solves[i].Pose : fourth;
                var budget = solves[Math.Min(i, 2)].Budget;
                if (issue) standard.Issue(tick, forecast, budget);
                if (issueEarly)
                {
                    if (i == 3 && outcome == "unavailable") early.Stop("CaptureFault");
                    else early.Issue(tick, forecast, budget);
                }
            }
            string log = sim.Report();
            string totals = outcome == "retained" ? "1|1|0|0" : outcome == "failed" ? "1|0|1|0" : "1|0|0|1";
            Check.That(log.Contains("third(reactive-early)=Fail|Pass") &&
                log.Contains("newAtThird-earlyFirst(pass-fail-unavailable)=" + totals), "third gain transfers to " + outcome);
        }

        private static void Links()
        {
            foreach (bool reverse in new[] { false, true }) foreach (bool miss in new[] { false, true }) RunPath(reverse, miss, false);
        }
        private static void Censored()
        {
            RunPath(false, true, true);
            RunPath(true, true, true, true);
        }

        private static Trial.Pose PathPose(int n)
        {
            var p = TurnHandoffContracts.Pose();
            p.Lower = p.Foot - Vector3.right * (n == 8 ? .006f : .001f);
            return p;
        }
        private static void RunPath(bool reverse, bool miss, bool stopEarly, bool stopBoth = false)
        {
            var sim = new FootTransitionContracts.Sim { Reverse = reverse };
            sim.Frame(0, 0, 0);
            sim.Frame(0, 0, 0, (ref Trace.Tick l, ref Trace.Tick r) => { l.Motion = r.Motion = "IDLE"; });
            const int start = 3, first = 16;
            var budget = TurnHandoffContracts.Budget();
            for (int local = start; local <= first + 1; local++)
            {
                PhysicsStepContext step = default;
                var p = PathPose(local);
                sim.Frame(.02f, .03f, -.02f, (ref Trace.Tick l, ref Trace.Tick r) =>
                {
                    l.Motion = r.Motion = "TURN";
                    TurnHandoffContracts.SetFree(ref l); TurnHandoffContracts.SetFree(ref r);
                    if (local == start) { TurnHandoffContracts.SetRelease(ref l); TurnHandoffContracts.SetRelease(ref r); }
                    if (local >= first-4) l.Phase = Phase.Approach;
                    if (local >= first-2) l.Phase = Phase.Stance;
                    if (local == first)
                    {
                        l.Miss = l.PendingMiss = miss; l.Ordinary = l.Active = !miss; l.EventFloor = l.Floor;
                    }
                    if (local > first) { l.Was = !miss; l.Recovery = miss; l.Active = true; l.EventFloor = l.Floor; }
                    l.EntryReferenceValid = local > start; l.SourceMin = .001f; l.ControlMin = -.003f;
                    TurnHandoffContracts.SetPose(ref l, p, 0); step = l.Step;
                }, held: local == first-1);
                bool pending = sim.Trace.TryHandoff(0, step, out var tick, out var reactive, out string prohibit, out var early, out _, out _);
                Check.That(pending == (local <= first && (!stopBoth || local <= 5)), "no issue beyond native opportunity");
                if (!pending) continue;
                Check.That(reactive != null && (early != null) == (!stopEarly || local <= 5), "per-rule continuity");
                bool issue = reactive.Visit(tick, p, 30, 31, prohibit);
                bool issueEarly = early?.Visit(tick, p, 30, 31, prohibit) == true;
                Check.That(issue == (local < first), "free approaches and pre-decision stances remain observable");
                if (issue)
                {
                    if (stopBoth && local == 5) reactive.Stop("MissingBudget");
                    else reactive.Issue(tick, PathPose(local + 1), budget);
                }
                if (issueEarly)
                {
                    if (stopEarly && local == 5) early.Stop("MissingBudget");
                    else early.Issue(tick, PathPose(local + 1), budget);
                }
            }
            string log = sim.Report();
            string expected = stopEarly ? "Unavailable" : "Pass";
            string standard = stopBoth ? "Unavailable" : "Pass";
            Check.That(log.Contains("/start-third-first=3|6|16/third(reactive-early)=" + standard + "|" + expected +
                "/first(reactive-early)=" + standard + "|" + expected), "same first identity");
            Check.That(log.Contains("native-resolved=" + (miss ? "Recovered|17" : "Ordinary|16")), "eventual native outcome");
            Check.That(log.Contains("actualGap(firstOver5At-peakAt-peakMm)=8|8|6"), "first crossing before both simulated stops");
            Check.That(log.Contains("started-nativeFirst-pairedFirst=1|1|" + (stopEarly ? 0 : 1)), "censoring excluded from paired totals");
            Check.That(log.Contains("N170,3,3,Lift,") && log.Contains("N170,16,15,Stance,") && !log.Contains("N170,17,"), "held source and bounded native path");
            Check.That(log.Contains(",NA,NA,") && log.Contains(",1,-3,"), "explicitly missing or measured source and control heights");
        }

        private static void ThresholdAgreement()
        {
            var start = TurnHandoffContracts.Pose();
            var forecast = TurnHandoffContracts.Pose(-.00205f);
            var measured = TurnHandoffContracts.Pose(-.00195f);
            var t = TurnHandoffContracts.Tick(3, start);
            var w = new TurnHandoffWindow(3, Vector3.forward, TurnHandoffWindow.Policy.LeftReactive);
            Check.That(w.Visit(t, start, 30, 31, null), "origin");
            w.Issue(t, forecast, TurnHandoffContracts.Budget());
            t = TurnHandoffContracts.Tick(4, measured);
            w.Visit(t, measured, 30, 31, null);
            var counts = new TurnHandoffWindow.Count[1];
            w.Accumulate(counts);
            var s = new StringBuilder(); w.Append(s, 0, 3, "NoNewEntry", 0, 0, null, 0, 0f);
            Check.That(counts[0].Calibrated == 1 && counts[0].OriginalPass == 0 &&
                s.ToString().Contains("nominalAgrees-horizontal-basePass-trialPass=True|True|False|"), "both issue and read geometries required");
        }

        private static void Boundaries()
        {
            var p = TurnHandoffContracts.Pose(); var b = TurnHandoffContracts.Budget();
            var t = TurnHandoffContracts.Tick(3, p);
            var w = new TurnHandoffWindow(3, Vector3.forward, TurnHandoffWindow.Policy.LeftEarly);
            Check.That(w.Visit(t, p, 30, 31, null), "visited origin");
            var alien = t; alien.Source.Clock.Sequence++; w.Issue(alien, p, b);
            Check.That(w.End == "IssueIdentity" && !w.TryReadPass(4, out _), "mixed source cannot issue");
            w = new TurnHandoffWindow(3, Vector3.forward, TurnHandoffWindow.Policy.LeftEarly);
            Check.That(w.Visit(t, p, 30, 31, null), "second origin"); w.Issue(t, p, b);
            t = TurnHandoffContracts.Tick(4, p); t.Body++;
            Check.That(!w.Visit(t, p, 30, 31, null) && w.End == "BodyOrBoxChanged", "foot body changed");
            w = new TurnHandoffWindow(3, Vector3.forward, TurnHandoffWindow.Policy.LeftEarly);
            t = TurnHandoffContracts.Tick(3, p);
            Check.That(w.Visit(t, p, 30, 31, null), "unclosed parent"); w.Issue(t, p, b);
            t = TurnHandoffContracts.Tick(4, p); t.Ordinary = t.Active = true; t.EventFloor = t.Floor;
            Check.That(!w.Visit(t, p, 30, 31, "WindowLimit") && w.End == "WindowLimit", "later entry outside the native parent is not attributed to it");
            w = new TurnHandoffWindow(3, Vector3.forward, TurnHandoffWindow.Policy.LeftEarly);
            for (int n = 3; n <= 3 + LandingEventHistory.RingCapacity; n++)
            {
                t = TurnHandoffContracts.Tick(n, p);
                bool issue = w.Visit(t, p, 30, 31, null);
                Check.That(issue == (n < 3 + LandingEventHistory.RingCapacity), "bounded rows");
                if (issue) w.Issue(t, p, b);
            }
            Check.That(w.End == "Capacity" && !w.TryReadPass(4 + LandingEventHistory.RingCapacity, out _), "never extrapolate beyond cap");
            var actual = new Trial.Measure { Sole = 0f };
            var bad = new Trial.Measure { Sole = -.00011f };
            Check.That(!bad.MaintainsHeight(actual), "readback cannot carry newly adverse lowest corner");
            bad.Sole = .026f; Check.That(!bad.MaintainsHeight(actual), "upper bound");
            bad.Sole = 0f; bad.LowerRise = -.00011f; Check.That(!bad.MaintainsHeight(actual), "ankle regression");
        }
    }
}
