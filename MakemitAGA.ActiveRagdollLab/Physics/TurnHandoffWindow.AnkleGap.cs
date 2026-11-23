using System;
using System.Text;
using Search = MakemitAGA.ActiveRagdollLab.Physics.CombinedLandingSearch;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class TurnHandoffWindow
    {
        // Called during report assembly. Only the second forecast's native ankle separation
        // changes. Keep the frozen leg angles, shoe rotation, floor and world-space traction.
        private static (float Sole, float Gap) HoldAnkleGap(in Leg.Pose atIssue, in Leg.Pose nominal, in Search.Evidence linear)
        {
            var currentGap = atIssue.Foot - atIssue.Lower;
            var nextGap = nominal.Foot - nominal.Lower;
            // The linear model used nextGap + (nextGap - currentGap). Removing that last
            // increment translates every shoe corner equally, including the lowest one.
            return (linear.Sole + (currentGap.y - nextGap.y), (nextGap + linear.NextShift).magnitude);
        }

        private struct AnkleGapErrors
        {
            internal int Count, LinearDisagreements, HeldDisagreements;
            private double _linearSole, _linearGap, _heldSole, _heldGap;

            internal void Add(in Search.Evidence linear, (float Sole, float Gap) held, in Search.WitnessRead read)
            {
                Count++;
                _linearSole += Math.Abs(linear.Sole - read.Sole);
                _linearGap += Math.Abs(linear.Gap - read.Gap);
                _heldSole += Math.Abs(held.Sole - read.Sole);
                _heldGap += Math.Abs(held.Gap - read.Gap);
                bool actual = Search.Evidence.WithinMargins(read.Sole, read.Gap);
                if (linear.Buffered != actual) LinearDisagreements++;
                if (Search.Evidence.WithinMargins(held.Sole, held.Gap) != actual) HeldDisagreements++;
            }

            internal void AppendMeans(StringBuilder b)
            {
                if (Count == 0) { b.Append("NA"); return; }
                b.Append(Mm((float)(_linearSole / Count))).Append('|').Append(Mm((float)(_linearGap / Count))).Append('|')
                    .Append(Mm((float)(_heldSole / Count))).Append('|').Append(Mm((float)(_heldGap / Count)));
            }
        }
    }
}
