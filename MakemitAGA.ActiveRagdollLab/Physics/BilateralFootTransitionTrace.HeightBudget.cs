using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using Height = MakemitAGA.ActiveRagdollLab.Physics.LandingHipHeightBudget;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Rise = MakemitAGA.ActiveRagdollLab.Physics.LandingRiseAllocation;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class TurnHandoffWindow
    {
        // Each observer already checked its own two clocks and body identities. Join their receipts
        // again before comparing a pelvis command with the geometry recorded by the leg observer.
        internal bool TryHeightPair(int from, int to, in PhysicsStepContext issue, in PhysicsStepContext read,
            int upper, int foot, int floor, out Leg.Pose nominal, out Leg.Pose measured)
        {
            nominal = measured = default;
            int index = from - Start;
            if (_combined == null || index < 0 || index >= _rows.Count || to != from + 1 ||
                upper != _upper || foot != _foot || floor != _floor) return false;
            var row = _rows[index];
            if (row.From != from || row.To != to || !row.Combined.IssueClock.Matches(issue) ||
                !row.Combined.VerifiedReadClock.Matches(read)) return false;
            nominal = row.Nominal; measured = row.Combined.PhysicalPose;
            return true;
        }
    }

    internal sealed partial class BilateralFootTransitionTrace
    {
        private sealed partial class LeftLandingPath
        {
            internal void AppendHeightBudget(StringBuilder b, TurnHandoffWindow legs, int strong)
            {
                int paired = 0, compared = 0;
                bool strongPaired = false, strongCompared = false;
                b.Append("\nTURN_HIP_HEIGHT_V041760/start-firstStrong=").Append(legs.Start).Append('|').Append(strong)
                    .Append("/model=INDEPENDENT_ONE_SOLVE_COMMON_Y/targetMm=").Append(HeightMm(Height.Target))
                    .Append("/H176=from,to,issue,read,geometry,issueCeilingMm,readCeilingMm,needMm,requestGainMm,retentionGainMm,best,remainingMm,nativeResidualMm,issueRefMinusRawMm,readRefMinusRawMm,readHipMinusRefMm");
                foreach (var row in _rise)
                {
                    if (row.From < legs.Start || (strong > 0 && row.From >= strong)) continue;
                    bool geometry = legs.TryHeightPair(row.From,row.To,row.Step,row.ReadClock,row.Upper,row.Foot,row.Floor,
                        out var nominal,out var measured);
                    double issueCeiling = geometry ? Height.Ceiling(nominal) : double.NaN;
                    double readCeiling = geometry ? Height.Ceiling(measured) : double.NaN;
                    geometry &= double.IsFinite(issueCeiling) && double.IsFinite(readCeiling);
                    var p = row.Plans;
                    var budget = geometry && row.State == Read.Matched && p.Status == Rise.State.Compared &&
                        p.Native.Valid && p.RequestOnly.Valid && p.EarlyRetention.Valid ?
                        Height.Compare(row.Step.Dt,row.Predicted(p.Native),row.Predicted(p.RequestOnly),row.Predicted(p.EarlyRetention),row.PostHipV) : default;
                    if (geometry) paired++;
                    if (budget.Valid) compared++;
                    if (row.To == strong && strong > 0) { strongPaired = geometry; strongCompared = budget.Valid; }
                    var atIssue = geometry ? ParentHeight(row.From,row.Step.SourceSequence) : default;
                    var atRead = geometry ? ParentHeight(row.To,row.ReadClock.SourceSequence) : default;
                    b.Append("\nH176,").Append(row.From).Append(',').Append(row.To).Append(',').Append(p.Status).Append(',')
                        .Append(row.State).Append(',').Append(geometry ? "Paired" : "Unavailable").Append(',')
                        .Append(HeightMm(issueCeiling)).Append(',').Append(HeightMm(readCeiling)).Append(',')
                        .Append(HeightMm(Math.Max(0.0,Height.Target-readCeiling))).Append(',');
                    if (budget.Valid)
                        b.Append(HeightMm(budget.RequestGain)).Append(',').Append(HeightMm(budget.RetentionGain)).Append(',')
                            .Append(budget.Best).Append(',').Append(HeightMm(budget.Remaining(readCeiling))).Append(',')
                            .Append(HeightMm(budget.NativeResidual));
                    else b.Append("NA,NA,NA,NA,NA");
                    b.Append(',').Append(atIssue.Parent ? HeightMm(atIssue.Reference) : "NA").Append(',')
                        .Append(atRead.Parent ? HeightMm(atRead.Reference) : "NA").Append(',')
                        .Append(atRead.Parent ? HeightMm(atRead.ResponseY) : "NA");
                }
                b.Append("\nTURN_HIP_HEIGHT_RECEIPTS_V041760/start=").Append(legs.Start)
                    .Append("/geometry-compared=").Append(paired).Append('|').Append(compared)
                    .Append("/firstStrongGeometry-budget=").Append(strongPaired).Append('|').Append(strongCompared);
            }

            private LandingHeightBreakdown ParentHeight(int local, int source)
            {
                if (_points.Count == 0) return default;
                int index = local - _points[0].Local;
                if (index < 0 || index >= _points.Count) return default;
                var point = _points[index];
                return point.Local == local && point.Source == source ? point.Height : default;
            }

            private static string HeightMm(double m) =>
                double.IsFinite(m) ? (m*1000.0).ToString("0.#####",CultureInfo.InvariantCulture) : "NA";
        }
    }
}
