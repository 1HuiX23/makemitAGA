using System;
using UnityEngine;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Stop-time geometry. The ankle sphere is allowed its full buffered vertical radius,
    // and command, rate, bend and horizontal tracking constraints can only reduce this ceiling.
    internal static class LandingHipHeightBudget
    {
        internal const double Target = -FootContactGeometryLimits.OrdinaryPenetrationMeters + LandingFootTraction.LandingMargin;
        private const double Limit = KneeGroundTarget.MaxCorrectionDegrees * (Math.PI / 180.0);

        // World height of one vector after rotation by t about the measured flexion axis.
        private readonly struct Wave
        {
            private readonly double _cos, _sin, _fixed;
            private Wave(double cosine, double sine, double constant) { _cos = cosine; _sin = sine; _fixed = constant; }

            internal Wave(Vector3 v, Vector3 axis)
            {
                double x = axis.x, y = axis.y, z = axis.z;
                double length = Math.Sqrt(x*x + y*y + z*z);
                x /= length; y /= length; z /= length;
                _fixed = y * (x*v.x + y*v.y + z*v.z);
                _cos = v.y - _fixed; _sin = z*v.x - x*v.z;
            }

            internal Wave Add(in Wave b, double sign = 1.0) =>
                new Wave(_cos + sign*b._cos, _sin + sign*b._sin, _fixed + sign*b._fixed);
            // This(t - shift)
            internal Wave Shift(double shift) =>
                new Wave(_cos*Math.Cos(shift) - _sin*Math.Sin(shift), _cos*Math.Sin(shift) + _sin*Math.Cos(shift), _fixed);
            internal double At(double t) => _cos*Math.Cos(t) + _sin*Math.Sin(t) + _fixed;
            internal double PeakAngle => Math.Atan2(_sin, _cos);
            internal double Max(double lo, double hi)
            {
                if (lo > hi) return double.NegativeInfinity;
                double peak = PeakAngle;
                return peak >= lo && peak <= hi ? _fixed + Math.Sqrt(_cos*_cos + _sin*_sin) : Math.Max(At(lo), At(hi));
            }

            internal void AddZeroes(Span<double> cuts, ref int count)
            {
                double radius = Math.Sqrt(_cos*_cos + _sin*_sin);
                if (radius == 0.0 || Math.Abs(_fixed) > radius) return;
                double arc = Math.Acos(-_fixed / radius), peak = PeakAngle;
                AddCut(cuts, ref count, peak - arc);
                AddCut(cuts, ref count, peak + arc);
            }
            private static void AddCut(Span<double> cuts, ref int count, double t)
            {
                t = Math.IEEERemainder(t, 2.0*Math.PI);
                if (t > -2.0*Limit && t < 2.0*Limit) cuts[count++] = t;
            }
        }

        internal static double Ceiling(in Leg.Pose p)
        {
            if (!p.Valid) return double.NaN;
            var thigh = p.UpperKnee - p.Hip;
            var shin = p.Lower - p.LowerKnee;
            var upper = new Wave(thigh, p.Axis);
            var center = new Wave(shin + p.Rotation * p.Center, p.Axis);
            var x = new Wave(p.Rotation * new Vector3(p.Half.x, 0f, 0f), p.Axis);
            var y = new Wave(p.Rotation * new Vector3(0f, p.Half.y, 0f), p.Axis);
            var z = new Wave(p.Rotation * new Vector3(0f, 0f, p.Half.z), p.Axis);
            // The bottom corner changes only when a rotated box axis has zero vertical projection.
            Span<double> cuts = stackalloc double[8];
            cuts[0] = -2.0*Limit; cuts[1] = 2.0*Limit;
            int count = 2;
            x.AddZeroes(cuts, ref count); y.AddZeroes(cuts, ref count); z.AddZeroes(cuts, ref count);
            cuts.Slice(0, count).Sort();
            double best = double.NegativeInfinity;
            for (int i = 1; i < count; i++)
            {
                double lo = cuts[i-1], hi = cuts[i];
                if (lo >= hi) continue;
                double mid = (lo + hi) * .5;
                var corner = center.Add(x, -Math.Sign(x.At(mid))).Add(y, -Math.Sign(y.At(mid))).Add(z, -Math.Sign(z.At(mid)));
                best = Math.Max(best, SegmentMaximum(upper, corner, lo, hi));
            }
            return (double)p.Lower.y - thigh.y - shin.y - p.FloorCenter.y - p.FloorHalf.y +
                Leg.GapLimit - LandingFootTraction.LandingMargin + best;
        }

        // h is the hip offset and s = h + knee. On this strip the clearance is A(h)+B(s).
        // A maximum lies on a fixed s, fixed h, fixed knee, or at an interior peak of A.
        private static double SegmentMaximum(in Wave a, in Wave b, double lo, double hi)
        {
            double top = Math.Max(FixedShank(a,b,lo), FixedShank(a,b,hi));
            top = Math.Max(top, FixedHip(a,b,-Limit,lo,hi));
            top = Math.Max(top, FixedHip(a,b, Limit,lo,hi));
            double peak = a.PeakAngle;
            if (peak > -Limit && peak < Limit) top = Math.Max(top, FixedHip(a,b,peak,lo,hi));
            top = Math.Max(top, FixedKnee(a,b,-Limit,lo,hi));
            return Math.Max(top, FixedKnee(a,b,Limit,lo,hi));
        }

        private static double FixedShank(in Wave a, in Wave b, double s) =>
            b.At(s) + a.Max(Math.Max(-Limit,s-Limit), Math.Min(Limit,s+Limit));
        private static double FixedHip(in Wave a, in Wave b, double h, double lo, double hi) =>
            a.At(h) + b.Max(Math.Max(lo,h-Limit), Math.Min(hi,h+Limit));
        private static double FixedKnee(in Wave a, in Wave b, double k, double lo, double hi) =>
            a.Shift(k).Add(b).Max(Math.Max(lo,k-Limit), Math.Min(hi,k+Limit));

        internal enum Choice { Native, Request, Retention }
        internal struct Budget
        {
            internal bool Valid;
            internal Choice Best;
            internal double RequestGain, RetentionGain, BestGain, NativeResidual;
            internal double Remaining(double ceiling) => Math.Max(0.0, Target - ceiling - BestGain);
        }

        // The choice uses only the three predictions made at issuance. The measured native
        // socket velocity provides a separate model residual and never selects the alternative.
        internal static Budget Compare(float dt, float nativeV, float requestV, float retentionV, float measuredV)
        {
            if (!float.IsFinite(dt) || dt <= 0f || dt > .1f || !float.IsFinite(nativeV) || !float.IsFinite(requestV) ||
                !float.IsFinite(retentionV) || !float.IsFinite(measuredV)) return default;
            var budget = new Budget { Valid = true, RequestGain = ((double)requestV-nativeV)*dt,
                RetentionGain = ((double)retentionV-nativeV)*dt, NativeResidual = Math.Abs((double)measuredV-nativeV)*dt };
            if (budget.RequestGain > 0.0) { budget.Best = Choice.Request; budget.BestGain = budget.RequestGain; }
            if (budget.RetentionGain > budget.BestGain) { budget.Best = Choice.Retention; budget.BestGain = budget.RetentionGain; }
            return budget;
        }
    }
}
