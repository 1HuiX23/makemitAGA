using System;
using UnityEngine;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Translation = MakemitAGA.ActiveRagdollLab.Physics.LeftLegHeightPreparation.Translation;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static partial class LandingFootTraction
    {
        // At fixed vertical displacement, the ankle sphere, acceleration ball and the two
        // tracking circles are disks in XZ. Both along-travel checks give one common strip.
        // The closest permissible point is the preferred point, a boundary projection or an
        // intersection of two boundaries. The final Unity geometry is checked independently.
        private struct Disk
        {
            internal double X, Z, R;
            internal Disk(double x, double z, double r) { X = x; Z = z; R = r; }
        }

        private struct Slice
        {
            internal double Tx, Tz, Lo, Hi, WantX, WantZ, BestD, X, Z;

            internal void Try(double x, double z, ReadOnlySpan<Disk> disks)
            {
                double along = Tx * x + Tz * z;
                if (along < Lo - 1e-12 || along > Hi + 1e-12) return;
                double dist = (x - WantX) * (x - WantX) + (z - WantZ) * (z - WantZ);
                if (dist >= BestD) return;
                foreach (var c in disks)
                    if ((x - c.X) * (x - c.X) + (z - c.Z) * (z - c.Z) > c.R * c.R + 1e-14) return;
                X = x; Z = z; BestD = dist;
            }

            internal void Find(ReadOnlySpan<Disk> disks)
            {
                BestD = double.PositiveInfinity;
                Try(WantX, WantZ, disks);
                if (BestD == 0.0) return;
                double along = Tx * WantX + Tz * WantZ;
                for (int edge = 0; edge < 2; edge++)
                {
                    double bound = edge == 0 ? Lo : Hi;
                    Try(WantX + Tx * (bound - along), WantZ + Tz * (bound - along), disks);
                    foreach (var c in disks)
                    {
                        double d = bound - (Tx * c.X + Tz * c.Z), h2 = c.R * c.R - d * d;
                        if (h2 < 0.0) continue;
                        double h = Math.Sqrt(h2), x = c.X + Tx * d, z = c.Z + Tz * d;
                        Try(x - Tz * h, z + Tx * h, disks);
                        Try(x + Tz * h, z - Tx * h, disks);
                    }
                }
                for (int i = 0; i < disks.Length; i++)
                {
                    var a = disks[i];
                    double dx = WantX - a.X, dz = WantZ - a.Z, length = Math.Sqrt(dx * dx + dz * dz);
                    if (length > 0.0) Try(a.X + a.R * dx / length, a.Z + a.R * dz / length, disks);
                    else Try(a.X, a.Z, disks);
                    for (int j = 0; j < i; j++)
                    {
                        var b = disks[j]; dx = b.X - a.X; dz = b.Z - a.Z;
                        double d = Math.Sqrt(dx * dx + dz * dz);
                        if (d == 0.0 || d > a.R + b.R || d < Math.Abs(a.R - b.R)) continue;
                        double side = (a.R * a.R - b.R * b.R + d * d) / (2.0 * d);
                        double h = Math.Sqrt(Math.Max(0.0, a.R * a.R - side * side));
                        double x = a.X + dx * side / d, z = a.Z + dz * side / d;
                        Try(x - dz * h / d, z + dx * h / d, disks);
                        Try(x + dz * h / d, z - dx * h / d, disks);
                    }
                }
            }
        }

        internal static Move Balance(in Leg.Pose pose, in Leg.Plan leg, in Leg.Aim aim, in Input input,
            float dt, in Translation old, in Move ray)
        {
            // Only a forecast that actually reached the original bounded search may be refined.
            if (ray.Status != State.UnsafeForecast && (ray.Status != State.Predicted ||
                (ray.Forecast.GeometricPass && Margin(ray.Forecast) >= LandingMargin))) return ray;
            // The next read uses a refreshed animation target. Leave room inside the two
            // horizontal tracking disks and their along-travel strip before it arrives.
            const float horizontalInset = .000010f;
            const float geometryInset = .000002f;
            float damping = Mathf.Max(0f, 1f - input.Drag * dt), response = dt * dt * damping;
            var center = old.Advance(-input.Applied, dt, damping).Position;
            var raw = input.Tether.Raw - old.Position * input.Tether.Kp - old.Velocity * input.Tether.Kd;
            var feedback = Vector3.ClampMagnitude(raw, input.Cap);
            var free = old.Advance(feedback - input.Applied, dt, damping).Position;
            var preferred = free + ray.RequestedShift;
            var gap = pose.Foot - pose.Lower;
            double radius = Leg.GapLimit - LandingMargin - geometryInset, force = response * input.Cap;
            double lo = Math.Max(Math.Max(-FootContactGeometryLimits.OrdinaryPenetrationMeters + LandingMargin + geometryInset - leg.Trial.Sole,
                leg.Base.Sole - Leg.Precision + geometryInset - leg.Trial.Sole), Math.Max(-radius - gap.y, center.y - force));
            double hi = Math.Min(FootContactGeometryLimits.RecoveryMaxGapMeters - geometryInset - leg.Trial.Sole,
                Math.Min(radius - gap.y, center.y + force));
            if (lo > hi || !(response > 0f)) return ray;
            var ankle = leg.Trial.Foot - aim.Target;
            var sole = leg.Trial.Foot + leg.Trial.ShoeRotation * (pose.Center - Vector3.up * pose.Half.y) - aim.TargetSole;
            double norm = Math.Sqrt((double)aim.Travel.x * aim.Travel.x + (double)aim.Travel.z * aim.Travel.z);
            double aLimit = Math.Abs(leg.Base.AnkleAlong) + Leg.Precision - horizontalInset, sLimit = Math.Abs(leg.Base.SoleAlong) + Leg.Precision - horizontalInset;
            var search = new Slice
            {
                Tx = aim.Travel.x / norm,
                Tz = aim.Travel.z / norm,
                WantX = preferred.x,
                WantZ = preferred.z,
                Lo = Math.Max(-aLimit - leg.Trial.AnkleAlong, -sLimit - leg.Trial.SoleAlong) / norm,
                Hi = Math.Min(aLimit - leg.Trial.AnkleAlong, sLimit - leg.Trial.SoleAlong) / norm
            };
            if (search.Lo > search.Hi) return ray;
            Span<Disk> disks = stackalloc Disk[4];
            disks[2] = new Disk(-ankle.x, -ankle.z, leg.Base.AnkleXZ + Leg.Precision - horizontalInset);
            disks[3] = new Disk(-sole.x, -sole.z, leg.Base.SoleXZ + Leg.Precision - horizontalInset);
            double preferredY = Math.Clamp((double)preferred.y, lo, hi);
            for (int i = 0; i < 3; i++)
            {
                double y = i == 0 ? preferredY : i == 1 ? lo : (lo + hi) * .5;
                if (i != 0 && y == preferredY) continue;
                double ankleY = gap.y + y, forceY = y - center.y;
                disks[0] = new Disk(-gap.x, -gap.z, Math.Sqrt(Math.Max(0.0, radius * radius - ankleY * ankleY)));
                disks[1] = new Disk(center.x, center.z, Math.Sqrt(Math.Max(0.0, force * force - forceY * forceY)));
                search.Find(disks);
                if (!double.IsFinite(search.BestD)) continue;
                var shift = new Vector3((float)search.X, (float)y, (float)search.Z);
                var accel = (shift - center) / response;
                var body = old.Advance(accel - input.Applied, dt, damping);
                var m = Leg.TranslateShoe(pose, leg.Trial, aim, body.Position);
                if (!Finite(accel) || accel.magnitude > input.Cap + .00001f || !Finite(body.Position) || !Finite(body.Velocity) ||
                    !m.Safe || m.Gap >= input.Tether.HardGap || !m.MaintainsHeight(leg.Base) || !m.Tracks(leg.Base) ||
                    !m.GeometricPass || Margin(m) < LandingMargin) continue;
                var result = ray;
                result.Status = State.Predicted; result.PlanarAdjusted = true;
                result.Body = body; result.Applied = accel; result.Forecast = m;
                result.RequestedShift = shift - free; result.Fraction = 1f;
                return result;
            }
            return ray;
        }
    }
}
