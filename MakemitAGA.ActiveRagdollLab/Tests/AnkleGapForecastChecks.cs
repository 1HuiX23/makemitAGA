using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Search = MakemitAGA.ActiveRagdollLab.Physics.CombinedLandingSearch;

namespace ARL.Contracts
{
    // Rebuild both N+2 poses in full, then apply the recorded command. This checks the
    // cheaper translation used in the stop report, including the rotated shoe corners.
    internal sealed class AnkleGapForecastChecks
    {
        private sealed class Sample
        {
            internal Leg.Measure Linear, Held, Measured;
            internal bool Buffered, Paired;
        }

        private readonly Dictionary<(int Issue, int Path), Sample> _samples = new();

        internal void Issue(int issue, TurnLandingContracts.RecordedSolve s, Search.Step step, Search.Step prior)
        {
            if (!step.HaveHorizon) return;
            Check.That(Search.TryHorizon(prior.PhysicalPose, s.Pose, s.Budget, s.Dt, out var linear), "rebuild the issued horizon");
            var held = linear;
            held.Foot = held.Lower + (s.Pose.Foot - s.Pose.Lower);
            foreach (var p in step.Paths.Take(step.Count))
            {
                var h = p.Horizon;
                if (!h.Feasible) continue;
                _samples.Add((issue, p.Id), new Sample {
                    Linear = Leg.Evaluate(linear, h.NextOffset, s.Issue, h.NextShift),
                    Held = Leg.Evaluate(held, h.NextOffset, s.Issue, h.NextShift),
                    Buffered = h.Buffered
                });
            }
        }

        internal void Read(int issue, Search.Path p, Leg.Pose real, Leg.Aim aim)
        {
            var s = _samples[(issue, p.Id)];
            Check.That(!s.Paired, "each frozen command has at most one measured second endpoint");
            s.Measured = Leg.Evaluate(real, p.Horizon.NextOffset, aim, p.Horizon.NextShift);
            s.Paired = true;
        }

        private static string Field(string line, string name)
        {
            string prefix = name + "=";
            return line.Split('/').Single(x => x.StartsWith(prefix, StringComparison.Ordinal)).Substring(prefix.Length);
        }

        private static float[] Numbers(string line, string field) =>
            Field(line, field).Split('|').Select(n => float.Parse(n, CultureInfo.InvariantCulture)).ToArray();

        private static void CheckSet(string report, string cohort, Sample[] samples)
        {
            string name = cohort + "MaeMm";
            if (samples.Length == 0) Check.That(Field(report, name) == "NA", "no mean error without a pair");
            else
            {
                var expected = new[] {
                    samples.Average(x => Math.Abs(x.Linear.Sole - x.Measured.Sole)),
                    samples.Average(x => Math.Abs(x.Linear.Gap - x.Measured.Gap)),
                    samples.Average(x => Math.Abs(x.Held.Sole - x.Measured.Sole)),
                    samples.Average(x => Math.Abs(x.Held.Gap - x.Measured.Gap))
                };
                var actual = Numbers(report, name);
                Check.That(actual.Length == expected.Length, "four means share the same denominator");
                for (int i = 0; i < expected.Length; i++)
                    Check.Near(actual[i], expected[i] * 1000f, .0006f, "mean absolute error from independent full-pose reconstruction");
            }
            bool Fits(Leg.Measure m) => Search.Evidence.WithinMargins(m.Sole, m.Gap);
            int linearMisses = samples.Count(s => Fits(s.Linear) != Fits(s.Measured));
            int heldMisses = samples.Count(s => Fits(s.Held) != Fits(s.Measured));
            Check.That(Field(report, cohort + "BoundsDisagreementsLinear-held") == $"{linearMisses}|{heldMisses}",
                "separate shoe and ankle envelope classification from full contact geometry");
        }

        internal void Verify(string report)
        {
            var lines = report.Split('\n');
            var pairs = _samples.Values.Where(s => s.Paired).ToArray();
            var buffered = pairs.Where(s => s.Buffered).ToArray();
            var summary = lines.Single(l => l.StartsWith("TURN_ANKLE_GAP_V041780/", StringComparison.Ordinal));
            Check.That(Field(summary, "feasible-paired-unpaired-bufferedPairs") ==
                $"{_samples.Count}|{pairs.Length}|{_samples.Count - pairs.Length}|{buffered.Length}", "all unavailable endpoints stay in the coverage count");
            var pairedLines = lines.Where(l => l.StartsWith("J178/", StringComparison.Ordinal) && l.Contains("/nextHeldSole-gapMm=")).ToArray();
            Check.That(pairedLines.Length == pairs.Length, "only verified physical pairs produce a held forecast receipt");
            foreach (string line in pairedLines)
            {
                var id = Field(line, "issue-path-parent").Split('|');
                var s = _samples[(int.Parse(id[0], CultureInfo.InvariantCulture), int.Parse(id[1], CultureInfo.InvariantCulture))];
                var held = Numbers(line, "nextHeldSole-gapMm");
                var linear = Numbers(line, "nextSole-gap-lowerLossMm");
                var measured = Numbers(line, "nextReadSole-gap-lowerLossMm");
                Check.That(s.Paired, "one measured endpoint for these exact issue and path identifiers");
                Check.Near(held[0], 1000f * s.Held.Sole, .0006f, "held ankle vector, signed sole height in mm");
                Check.Near(held[1], 1000f * s.Held.Gap, .0006f, "held ankle vector plus the same three-dimensional shoe shift in mm");
                Check.Near(linear[0], 1000f * s.Linear.Sole, .0006f, "linear sole in mm");
                Check.Near(linear[1], 1000f * s.Linear.Gap, .0006f, "linear gap in mm");
                Check.Near(measured[0], 1000f * s.Measured.Sole, .0006f, "measured sole in mm");
                Check.Near(measured[1], 1000f * s.Measured.Gap, .0006f, "measured gap in mm");
            }
            CheckSet(summary, "all", pairs);
            CheckSet(summary, "buffered", buffered);
        }
    }
}
