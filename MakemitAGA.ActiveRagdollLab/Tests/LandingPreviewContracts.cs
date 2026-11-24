using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using UnityEngine;
using Entry = MakemitAGA.ActiveRagdollLab.Physics.OrdinaryFootEntry;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Search = MakemitAGA.ActiveRagdollLab.Physics.CombinedLandingSearch;

namespace ARL.Contracts
{
    internal static class LandingPreviewContracts
    {
        private static float F(string s) => float.Parse(s, CultureInfo.InvariantCulture);
        private static int I(string s) => int.Parse(s, CultureInfo.InvariantCulture);
        private static Vector3 V(string s)
        {
            var a = s.Split(',');
            return new Vector3(F(a[0]), F(a[1]), F(a[2]));
        }

        internal static void Run()
        {
            Check.Run("177 next pose carries measured pelvis transport and rigid segment rates", HorizonGeometry);
            Check.Run("177 forecast receipts require consecutive physical clocks and a measured first command", WitnessClocks);
            Check.Run("178 ankle model comparisons share the exact buffered geometry boundaries", BufferedBounds);
            Check.Run("179 seventeen strict releases, preview rank and paired second-step forecasts", Recorded);
        }

        private static Leg.Pose ReadPose(string[] row, Leg.Pose p)
        {
            var bones = row[18].Split('|').Select(V).ToArray();
            var q = row[20].Split(',').Select(F).ToArray();
            p.Hip = bones[0]; p.UpperKnee = bones[1]; p.LowerKnee = bones[2];
            p.Lower = bones[3]; p.Foot = bones[4]; p.Axis = V(row[19]);
            p.Rotation = new Quaternion(q[0], q[1], q[2], q[3]);
            return p;
        }

        private static void CheckPlanarContinuation(TurnLandingContracts.RecordedSolve s, Foot.Input cmd, Vector3 gap,
            Search.Step previous, Leg.Pose real, Leg.Aim aim, Leg.Measure native)
        {
            var parent = previous.Paths.Take(previous.Count).Single(p => p.SafeRead);
            var strict = new Leg.Options();
            var extra = new Leg.Options();
            Leg.Select(s.Pose, s.Issue, s.Budget, s.Dt, s.HipY, parent.Leg.Offset, true, true, strict, extra);
            int eligible = 0, rays = 0, balanced = 0;
            foreach (var leg in extra.Items.Take(extra.Count))
            {
                if (new Search.LowerLoss(leg.Trial, leg.Base, s.Issue).Worst > Leg.Precision) continue;
                eligible++;
                var ray = Foot.Predict(s.Pose, leg, s.Issue, cmd, s.Dt, gap, parent.Move, true, true);
                if (ray.Status == Foot.State.Predicted) rays++;
                var move = Foot.Balance(s.Pose, leg, s.Issue, cmd, s.Dt, parent.Move.Body, ray);
                if (!move.PlanarAdjusted) continue;
                var issued = move;
                var actualLeg = Leg.Evaluate(real, leg.Offset, aim);
                Foot.SettleOnLeg(ref move, real, actualLeg, aim, native, cmd.Tether.HardGap);
                var body = parent.Move.Body.Advance(issued.Applied - cmd.Applied, s.Dt, Math.Max(0, 1 - cmd.Drag * s.Dt));
                Check.That(move.MarginPass && move.Settled.Tracks(native) && issued.Forecast.Tracks(leg.Base) &&
                    new Search.LowerLoss(actualLeg, native, aim).Worst <= Leg.Precision && issued.Applied.magnitude < cmd.Cap &&
                    (body.Position - issued.Body.Position).sqrMagnitude == 0 && (body.Velocity - issued.Body.Velocity).sqrMagnitude == 0,
                    "single unchanged vector cap, two body anchors and fresh shoe receipt for the off-ray command");
                balanced++;
            }
            Check.That(strict.Count == 0 && eligible == 2 && rays == 0 && balanced == 1,
                "3920 has one buffered continuation in the horizontal slice after the scalar ray runs out");
        }

        private static void CheckRefresh(Search.Step step, Search.Step previous, Leg.Pose real,
            Leg.Aim refreshed, Leg.Measure native, Foot.Input command)
        {
            var path = step.Paths.Take(step.Count).Single();
            var parent = previous.Paths.Take(previous.Count).Single(p => p.Id == path.Parent);
            Check.That(path.Id == 53 && parent.Id == 50 && parent.SafeRead && parent.MarginSamples == 3 &&
                path.Move.PlanarAdjusted && path.SafeRead && path.MarginSamples == 4 && path.FirstLowerExcess == 0,
                "the sole surviving 4903 to 4904 continuation has four measured margins on the original parent");
            Check.Near(path.Leg.Offset.Hip, -.03425825f, 1e-7f, "same issued hip angle as the 177 witness");
            Check.Near(path.Leg.Offset.Knee, .05625775f, 1e-7f, "same issued knee angle as the 177 witness");
            var oldShift = new Vector3(-.0019335678f, .004411312f, -.003675004f); // Recorded J177 path 53.
            var oldRead = Leg.Evaluate(real, path.Leg.Offset, refreshed, oldShift);
            var oldLoss = new Search.ShoeLoss(oldRead, native);
            var issueLoss = new Search.ShoeLoss(path.Move.Forecast, path.Leg.Base);
            Check.Near(oldLoss.AnkleXZ * 1000f, .10249f, .00012f, "historical command evaluated on the refreshed target in mm");
            Check.Near(issueLoss.AnkleXZ * 1000f, .08996f, .00014f, "horizontal distance loss at issue in mm");
            Check.Near(path.PostTraction.AnkleXZ * 1000f, .09503f, .00014f, "horizontal distance loss at read in mm");
            Check.Near((path.Move.Body.Position - oldShift).magnitude * 1000f, .00904f, .00014f, "change to the frozen command position in mm");
            Check.Near(path.Move.Settled.Sole * 1000f, -1.7480f, .0002f, "shoe lowest corner in mm");
            Check.Near(path.Move.Settled.Gap * 1000f, 4.74823f, .0002f, "three-dimensional ankle gap in mm");
            Check.Near(path.Move.Applied.magnitude, 12.1762f, .002f, "issued acceleration in m/s squared");
            Check.That(Leg.Precision == .0001f && oldLoss.AnkleXZ > Leg.Precision && path.Move.Settled.Tracks(native) &&
                path.ReadLower.Worst <= Leg.Precision && path.Move.MarginPass && command.Cap == 60f &&
                path.Move.Applied.magnitude < command.Cap, "both commands are judged under the same physical limits");
        }

        private static void HorizonGeometry()
        {
            var current = TurnHandoffContracts.Pose(-.001f);
            var next = current;
            var translation = new Vector3(.012f, -.003f, .008f);
            next.Hip += translation; next.UpperKnee += translation; next.LowerKnee += translation;
            next.Lower += translation; next.Foot += translation + Vector3.right * .0005f;
            var budget = TurnHandoffContracts.Budget();
            budget.UpperW = new Vector3(.6f, -.2f, .1f); budget.LowerW = new Vector3(.1f, .8f, -.2f);
            budget.FootW = new Vector3(.4f, .3f, -.5f);
            foreach (float dt in new[] { .01f, .02f, 1f / 30f })
            {
                Check.That(Search.TryHorizon(current, next, budget, dt, out var forecast), "valid held issue-time inputs");
                Check.Near((forecast.Hip - next.Hip - translation).magnitude, 0, 2e-7f, "full pelvis displacement");
                Check.Near((forecast.UpperKnee - forecast.Hip).magnitude, (next.UpperKnee - next.Hip).magnitude, 2e-7f, "rigid thigh length");
                Check.Near((forecast.Lower - forecast.LowerKnee).magnitude, (next.Lower - next.LowerKnee).magnitude, 2e-7f, "rigid shin length");
                Check.Near((forecast.Foot - forecast.Lower - (next.Foot - next.Lower) - Vector3.right * .0005f).magnitude, 0, 2e-7f, "distinct native ankle separation");
                var shoeRate = FootRelativeReference.Log(forecast.Rotation * Quaternion.Inverse(next.Rotation)) / dt;
                Check.Near((shoeRate - budget.FootW).magnitude, 0, .00002f, "shoe uses the corresponding body angular velocity");
            }
            var cut = next; cut.Half.x += .001f;
            Check.That(!Search.TryHorizon(current, cut, budget, .02f, out _) && !Search.TryHorizon(default, next, budget, .02f, out _) &&
                !Search.TryHorizon(current, next, budget, float.NaN, out _) && !Search.TryHorizon(current, next, budget, .1001f, out _),
                "missing pose, changed material box or invalid step has no second prediction");
            budget.FootW = Vector3.up * 19f;
            Check.That(!Search.TryHorizon(current, next, budget, .02f, out _), "over-budget speed has no witness");
        }

        private static void BufferedBounds()
        {
            float bottom = -FootContactGeometryLimits.OrdinaryPenetrationMeters + Foot.LandingMargin;
            float top = FootContactGeometryLimits.RecoveryMaxGapMeters;
            float gap = Leg.GapLimit - Foot.LandingMargin;
            Check.That(Search.Evidence.WithinMargins(bottom, gap) && Search.Evidence.WithinMargins(top, gap) &&
                !Search.Evidence.WithinMargins(MathF.BitDecrement(bottom), gap) &&
                !Search.Evidence.WithinMargins(MathF.BitIncrement(top), gap) &&
                !Search.Evidence.WithinMargins(bottom, MathF.BitIncrement(gap)), "inclusive sole band and three-dimensional ankle limit");
            var e = new Search.Evidence { Sole = bottom, Gap = gap };
            Check.That(!e.Buffered, "a numerical endpoint without a legal command cannot enter the buffered cohort");
            e.Feasible = true;
            Check.That(e.Buffered, "a feasible command on both boundaries enters exactly once");
        }

        private static void WitnessClocks()
        {
            var pose = TurnHandoffContracts.Pose(-.001f);
            var aim = new Leg.Aim { Target = pose.Foot, TargetSole = pose.Foot + pose.Rotation * (pose.Center - Vector3.up * pose.Half.y), Travel = Vector3.forward };
            var native = Leg.Evaluate(pose, default, aim);
            var clock = TurnHandoffContracts.Tick(21, pose).Step;
            var issued = new Search.Step { IssueClock = clock };
            issued.Command.Tether.HardGap = .03f;
            clock.Sequence++;
            issued.VerifiedReadClock = clock;
            var next = new Search.Step { IssueClock = clock, HasPhysicalPose = true, PhysicalPose = pose };
            clock.Sequence++;
            next.VerifiedReadClock = clock;
            var p = new Search.Path { Gate = Search.ReadGate.Accepted, Horizon = new Search.Evidence { Feasible = true } };
            Check.That(Search.TryReadWitness(p, issued, next, aim, native, out var read) && read.Geometry && read.Tracking,
                "the exact same frozen witness can be checked once two native solves are paired");
            p.Horizon.NextShift = Vector3.forward * .004f;
            Check.That(Search.TryReadWitness(p, issued, next, aim, native, out read) && read.Geometry && !read.Tracking,
                "geometry and horizontal follow at the refreshed target are independent");
            next.HasPhysicalPose = false;
            Check.That(!Search.TryReadWitness(p, issued, next, aim, native, out _), "both models require the actual second pose");
            next.HasPhysicalPose = true;
            next.IssueClock.SourceSequence++;
            Check.That(!Search.TryReadWitness(p, issued, next, aim, native, out _), "a different render source at the shared handoff cannot be paired");
            next.IssueClock = issued.VerifiedReadClock;
            next.IssueClock.Epoch++;
            Check.That(!Search.TryReadWitness(p, issued, next, aim, native, out _), "foreign native epoch");
            next.IssueClock = issued.VerifiedReadClock;
            next.VerifiedReadClock.Sequence++;
            Check.That(!Search.TryReadWitness(p, issued, next, aim, native, out _), "skipped second read");
            next.VerifiedReadClock.Sequence--;
            issued.IssueClock.Dt *= .5f;
            Check.That(!Search.TryReadWitness(p, issued, next, aim, native, out _), "held horizon duration differs from actual second solve");
            issued.IssueClock.Dt = next.IssueClock.Dt;
            p.Gate = Search.ReadGate.Unread;
            Check.That(!Search.TryReadWitness(p, issued, next, aim, native, out _), "unmeasured first leg cannot certify a later witness");
            p.Gate = Search.ReadGate.Accepted; p.Horizon.Feasible = false;
            Check.That(!Search.TryReadWitness(p, issued, next, aim, native, out _), "missing second command cannot become a successful read");
        }

        private static void Recorded()
        {
            int oldShoes = 0, oldLower = 0, newShoes = 0, newLower = 0, releases = 0;
            foreach (int version in new[] { 175, 176, 177, 178 })
            {
                using var file = new StreamReader(typeof(LandingPreviewContracts).Assembly.GetManifestResourceStream("ARL.Contracts.LandingTracking" + version));
                file.ReadLine();
                var rows = new List<string[]>();
                string line;
                while ((line = file.ReadLine()) != null) rows.Add(line.Split('\t'));
                Check.That(rows.Count == (version switch { 175 => 50, 176 => 51, 177 => 55, _ => 64 }), "every logged command has a measured endpoint");
                foreach (var release in rows.GroupBy(row => I(row[0])))
                {
                    var steps = release.ToArray();
                    if (version <= 177)
                    {
                        var old = Replay(steps, false);
                        oldShoes += old.shoe ? 1 : 0; oldLower += old.lower ? 1 : 0;
                    }
                    var current = Replay(steps, true);
                    newShoes += current.shoe ? 1 : 0; newLower += current.lower ? 1 : 0;
                    releases++;
                }
            }
            Check.That(releases == 17 && oldShoes == 7 && oldLower == 5 && newShoes == 12 && newLower == 12,
                $"historical no-preview results on 12 releases: {oldShoes}/{oldLower}; current results on 17: {newShoes}/{newLower}");
        }

        private static void SetClock(ref Trace.Tick tick, int shared, int source, float dt)
        {
            tick.Step.Sequence = shared; tick.Step.SourceSequence = source; tick.Step.Dt = dt;
            tick.Step.FixedTime = tick.Step.UnscaledTime = shared * dt;
            tick.Source.Clock.Sequence = source; tick.Source.Clock.Time = source * dt;
        }

        private static (bool shoe, bool lower) Replay(string[][] rows, bool preview)
        {
            var search = new Search(preview);
            var ankleForecasts = new AnkleGapForecastChecks();
            var nativeTrail = new Entry.Trail();
            Search.Step previous = null;
            int start = I(rows[0][0]), first = I(rows[0][23]), strong = I(rows[0][27]);
            int shoes = 0, lower = 0, nominations = 0, feasible = 0, buffered = 0, maxCommands = 0, checkedJoint = 0;
            int paired = 0, correctBuffered = 0, forecastBuffered = 0, adjusted = 0, actualGeometry = 0, actualTracking = 0;
            var initial = TurnLandingContracts.DecodeIssue(rows[0]);
            var origin = initial.Pose;
            origin.Hip.y = initial.HipY; origin.Lower = origin.Foot - V(rows[0][17]);
            var tick = ContinuousLandingContracts.Tick(start, origin, strong, first);
            SetClock(ref tick, I(rows[0][28]), I(rows[0][24].Split('|')[0]), initial.Dt);
            tick.Target = initial.Issue.Target; tick.SourceSole = initial.Issue.TargetSole;
            var window = new TurnHandoffWindow(start, initial.Issue.Travel, TurnHandoffWindow.Policy.LeftCombined, usePreview: preview);
            Check.That(window.Visit(tick, origin, 22, 23, null), "normal release visit");
            bool sampledLowerRejection = false;
            int previewParent = 0;
            foreach (var row in rows)
            {
                var s = TurnLandingContracts.DecodeIssue(row);
                int to = start + I(row[1]), from = to - 1;
                var clocks = row[24].Split('|').Select(I).ToArray();
                Check.That(tick.Step.Sequence == I(row[28]) && tick.Step.SourceSequence == clocks[0] &&
                    (tick.Target - s.Issue.Target).magnitude < 2e-6f && (tick.SourceSole - s.Issue.TargetSole).magnitude < 2e-6f,
                    "same issued clock and aim, including repeated render sources");
                var cmd = CombinedLandingContracts.Command(row, false);
                window.Issue(tick, s.Pose, s.Budget, cmd);
                var step = search.Issue(from, s.Dt, s.HipY, s.Pose, s.Issue, s.Budget, cmd, V(row[17]), previous);
                step.IssueClock = tick.Step;
                ankleForecasts.Issue(from, s, step, previous);
                var real = ReadPose(row, s.Pose);
                var aim = TurnLandingContracts.ReadAim(row[9], s);
                var native = Leg.Evaluate(real, default, aim);
                var e = row[21].Split('|');
                var receipt = new Entry.Sample
                {
                    Local = I(e[0]),
                    Floor = I(e[1]),
                    Confirmed = bool.Parse(e[2]),
                    LegacyReady = bool.Parse(e[3]),
                    IdentityHeld = bool.Parse(e[4]),
                    NativeInside = bool.Parse(e[5]),
                    NativeSole = row[22] == "NA" ? 0 : F(row[22]) * .001f
                };
                Check.That(receipt.Local == to && step.Count <= Search.Width && step.Shortlisted <= Search.NominationLimit, "finite search on the issued tick");
                Check.That(step.Paths.Take(step.Count).Count(p => p.PreviewSlot) <= 1 &&
                    step.Count <= step.Forecasts && step.NextCommands <= Search.NominationLimit * Leg.Options.Capacity,
                    "one reserved slot and fixed upper bound on speculative commands");
                if (previous == null) Check.That(!step.HaveHorizon, "no invented incoming transport at the release");
                maxCommands = Math.Max(maxCommands, step.NextCommands); checkedJoint += step.JointChecked;
                foreach (var p in step.Paths.Take(step.Count))
                {
                    if (p.Horizon.Evaluated) nominations++;
                    if (p.Horizon.Feasible) feasible++;
                    if (p.Horizon.Buffered) buffered++;
                    if (p.Move.PlanarAdjusted) adjusted++;
                    Check.That(p.Move.Applied.magnitude <= cmd.Cap + .00001f && p.Move.Forecast.Safe &&
                        p.Move.Forecast.MaintainsHeight(p.Leg.Base) && p.Move.Forecast.Tracks(p.Leg.Base) &&
                        p.Move.Forecast.Gap < cmd.Tether.HardGap, "bounded actual first command on every path");
                    var parent = previous == null ? default : previous.Paths.Take(previous.Count).Single(q => q.Id == p.Parent);
                    if (p.Protected)
                    {
                        var old = Foot.Predict(s.Pose, p.Leg, s.Issue, cmd, s.Dt, V(row[17]), parent.Move, previous != null, true);
                        Check.That(p.Leg.Trial.Tracks(p.Leg.Base) && !p.PreviewSlot && !p.Move.PlanarAdjusted &&
                            (old.Body.Position - p.Move.Body.Position).sqrMagnitude == 0f &&
                            (old.Body.Velocity - p.Move.Body.Velocity).sqrMagnitude == 0f, "original control stores its own unmodified traction");
                    }
                    if (!preview && to == 4169 && !sampledLowerRejection && p.FirstLowerExcess == 0 && p.IssueLower.Worst <= Leg.Precision)
                    {
                        var copy = p; copy.PreviewSlot = true;
                        var independent = new Search();
                        var sample = new Search.Step { Command = cmd, Count = 1, Paths = new[] { copy } };
                        independent.Read(to, sample, real, aim, native, receipt, false);
                        Check.That(sample.Paths[0].Move.SafeRead && sample.Paths[0].Gate == Search.ReadGate.LowerHorizontal &&
                            sample.Paths[0].FirstLowerExcess == to && sample.SafeReads == 0 && independent.Outcome == "NoSafeRead",
                            "a safe shoe cannot carry a preview child past a failed shank read");
                        sampledLowerRejection = true;
                    }
                    Check.That(!p.SafeRead && p.Move.Status == Foot.State.Predicted && (!p.PreviewSlot ||
                        (p.FirstLowerExcess == 0 && p.IssueLower.Worst <= Leg.Precision && p.Move.Forecast.Tracks(p.Leg.Base))),
                        "unmeasured candidates cannot claim readback");
                    if (previous != null) Check.That(previous.Paths.Take(previous.Count).Any(parent => parent.Id == p.Parent && parent.SafeRead), "measured parent identity");
                }
                if (!preview && from == 3920) CheckPlanarContinuation(s, cmd, V(row[17]), previous, real, aim, native);
                search.Read(to, step, real, aim, native, receipt, to == first);
                if (preview && start == 4121 && to == first - 1)
                {
                    var slot = step.Paths.Take(step.Count).Single(p => p.PreviewSlot);
                    Check.That(step.PreviewPeakShift.HasValue && slot.SafeRead && slot.MarginSamples == 3 &&
                        slot.FirstLowerExcess == 0, "the issued preview survives the refreshed target before 4132");
                    previewParent = slot.Id;
                }
                if (preview && start == 4121 && to == first)
                {
                    var child = step.Paths.Take(step.Count).Single(p => p.SafeRead && p.MarginSamples == 4 && p.FirstLowerExcess == 0);
                    Check.That(previewParent != 0 && child.Parent == previewParent && child.Entry.DepthLatch == 0 &&
                        receipt.CanEnter(child.Entry.ReadySamples), "4132 qualifies through the retained parent on the preceding receipt");
                    Check.Near(child.Move.Settled.Sole * 1000f, -1.749f, .002f, "4132 measured-pose shoe height in mm");
                    Check.Near(child.Move.Settled.Gap * 1000f, 4.749f, .002f, "4132 measured-pose ankle gap in mm");
                }
                if (preview && from == 4903) CheckRefresh(step, previous, real, aim, native, cmd);
                var nativeDecision = nativeTrail.Observe(receipt, receipt.NativeInside, receipt.NativeSole);
                tick = ContinuousLandingContracts.Tick(to, real, strong, first, nativeTrail.DepthLatch);
                Check.That(tick.Ordinary == (nativeDecision == Entry.Decision.Ordinary) &&
                    tick.Miss == (nativeDecision == Entry.Decision.Miss), "owner decision reconstructed from the recorded floor, gait and shoe");
                SetClock(ref tick, step.IssueClock.Sequence + 1, clocks[1], s.Dt);
                tick.Target = aim.Target; tick.SourceSole = aim.TargetSole; tick.Entry = receipt;
                tick.Entry.Floor = receipt.Floor == 0 ? 0 : tick.Floor;
                Check.That(window.Visit(tick, real, 22, 23, null) == (to < first), "native first is determined by the physical owner");
                step.VerifiedReadClock = tick.Step;
                if (previous != null)
                    foreach (var parent in previous.Paths.Take(previous.Count))
                        if (Search.TryReadWitness(parent, previous, step, aim, native, out var observed))
                        {
                            paired++;
                            ankleForecasts.Read(from - 1, parent, real, aim);
                            if (parent.Horizon.Buffered)
                            {
                                forecastBuffered++;
                                if (observed.Geometry) actualGeometry++;
                                if (observed.Tracking) actualTracking++;
                                if (observed.Geometry && observed.Tracking) correctBuffered++;
                            }
                        }
                if (to == first)
                {
                    foreach (var p in step.Paths.Take(step.Count))
                    {
                        if (!p.SafeRead || p.Entry.DepthLatch != 0 || p.Entry.StrongStart != strong || p.Entry.StrongSamples != 4 ||
                            p.MarginSamples != 4 || !receipt.CanEnter(p.Entry.ReadySamples)) continue;
                        shoes++;
                        if (p.FirstLowerExcess == 0) lower++;
                    }
                }
                previous = step;
            }
            var last = search.LastRead;
            var report = new StringBuilder();
            Check.That(tick.Ordinary == (start == 4100) && tick.Miss == (start != 4100),
                "only the physical release at 4100 has an ordinary entry on its first opportunity");
            string nativeOutcome = tick.Ordinary ? "Ordinary" : "FirstMiss";
            window.AppendCombined(report, tick, nativeOutcome, tick.Ordinary ? first : 0, strong, null);
            var log = report.ToString();
            ankleForecasts.Verify(log);
            var repeated = new StringBuilder();
            window.AppendCombined(repeated, tick, nativeOutcome, tick.Ordinary ? first : 0, strong, null);
            Check.That(log == repeated.ToString(), "report assembly leaves all issued paths, recipients and counters intact");
            Check.That(log.Contains("/sequenceEnd=" + search.Outcome + "/") && window.End == "FirstOpportunity" &&
                log.Contains("/retainedBufferedShoe-preAlso-lowerAlso-bothAlso=" + (search.LastReadLocal == first ? shoes + "|" : "Unavailable")) &&
                log.Split("/nextReadGeometry-tracking=").Length - 1 == paired &&
                log.Contains($"/paired-predictedBuffered-actualGeometry-actualTracking-actualBoth={paired}|{forecastBuffered}|{actualGeometry}|{actualTracking}|{correctBuffered}"), "normal owner reports exactly the verified N+2 pairs");
            if (!preview && start == 4159) Check.That(sampledLowerRejection, "fresh target genuinely invalidates a shank certificate");
            if (preview)
            {
                Check.That(search.LastReadLocal == first && search.EndLocal == first && last.Entry.StrongSamples == 4 &&
                    nominations > 0 && feasible > 0 && buffered > 0 && paired > 0 && forecastBuffered > 0 &&
                    log.Contains("/horizonReady=True") && log.Contains("J179/") &&
                    log.Split("/nextRead=Unavailable").Length - 1 == feasible - paired,
                    $"search reaches the physical first at {first} and reports exactly the unpaired second reads");
                int margins = start switch { 2471 => 0, 2540 => 1, 2668 => 1, 3267 => 3, 3910 => 2, _ => 4 };
                int latch = start switch { 2471 => 2483, 2540 => 2550, 2668 => 2679, 3267 => 3279, 3910 => 3920, _ => 0 };
                Check.That(last.MarginSamples == margins && last.Entry.DepthLatch == latch &&
                    search.Outcome == (latch == 0 ? "WouldEnter" : "DepthLatched") && (shoes > 0) == (latch == 0) && (lower > 0) == (latch == 0),
                    "measured continuous strong-stance margins and independent joint qualification at " + first);
            }
            if (preview)
                Check.That(checkedJoint >= nominations && nominations >= feasible && feasible >= forecastBuffered &&
                    forecastBuffered >= correctBuffered && correctBuffered <= actualTracking && correctBuffered <= actualGeometry &&
                    adjusted <= Search.Width * rows.Length && maxCommands <= Search.NominationLimit * Leg.Options.Capacity,
                    "subsets of real paired, forecast and bounded search populations");
            return (shoes > 0, lower > 0);
        }
    }
}
