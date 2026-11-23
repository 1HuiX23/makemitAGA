using UnityEngine;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class CombinedLandingSearch
    {
        // Six distinct nominations at most; only one can take a place in the four issued paths.
        internal const int NominationLimit = 6;
        internal struct Evidence
        {
            internal bool Evaluated, Feasible;
            internal Leg.Offset NextOffset;
            internal Vector3 NextShift;
            internal float Sole, Gap, Lower, TrackingRoom, PlanarShift;
            internal bool Buffered => Feasible && WithinMargins(Sole, Gap);

            internal static bool WithinMargins(float sole, float gap) =>
                sole >= -FootContactGeometryLimits.OrdinaryPenetrationMeters + Foot.LandingMargin &&
                sole <= FootContactGeometryLimits.RecoveryMaxGapMeters && gap <= Leg.GapLimit - Foot.LandingMargin;
        }

        private struct Nomination
        {
            internal Path Path;
            internal float Score;
        }

        private sealed class PreviewScratch
        {
            internal readonly Nomination[] Nominations = new Nomination[NominationLimit];
            internal readonly Leg.Options Extra = new Leg.Options(), NextStrict = new Leg.Options(), NextExtra = new Leg.Options();
            internal bool HasNominee;
        }

        private PreviewScratch _preview;

        // Repeat the incoming pelvis displacement, body angular velocities and changes in the
        // two native joint separations. Rotate the rigid segments about their own anchors.
        // The animation aim and the command budget are held at issue N. No N+1 source is used.
        internal static bool TryHorizon(in Leg.Pose current, in Leg.Pose next, in Leg.Budget b, float dt, out Leg.Pose pose)
        {
            pose = default;
            if (!current.Valid || !next.Valid || !current.SameBox(next) || !b.Valid ||
                !float.IsFinite(dt) || dt <= 0f || dt > .1f) return false;
            var upper = FootRelativeReference.Advance(Quaternion.identity, b.UpperW, dt);
            var lower = FootRelativeReference.Advance(Quaternion.identity, b.LowerW, dt);
            var kneeGap = next.LowerKnee - next.UpperKnee;
            var ankleGap = next.Foot - next.Lower;
            pose = next;
            pose.Hip += next.Hip - current.Hip;
            pose.UpperKnee = pose.Hip + upper * (next.UpperKnee - next.Hip);
            pose.LowerKnee = pose.UpperKnee + kneeGap + (kneeGap - (current.LowerKnee - current.UpperKnee));
            pose.Lower = pose.LowerKnee + lower * (next.Lower - next.LowerKnee);
            pose.Foot = pose.Lower + ankleGap + (ankleGap - (current.Foot - current.Lower));
            pose.Axis = lower * next.Axis;
            pose.Rotation = FootRelativeReference.Advance(next.Rotation, b.FootW, dt);
            return pose.Valid;
        }

        // Cheap nominations cover position, momentum, available height and tracking room.
        // The free angular drift only ranks proposals. A bounded legal continuation must be
        // found below before the report can claim a two-step witness.
        private void Offer(in Path p, in Leg.Pose horizon, in Leg.Aim aim, float dt)
        {
            int geometry = !_preview.HasNominee ? -1 : CompareGeometry(p, _preview.Nominations[0].Path);
            if (geometry > 0) return;
            var a = p.Leg.Offset;
            var drift = a;
            drift.Hip += dt * a.HipRate;
            drift.Knee += dt * a.KneeRate;
            var m = Leg.Evaluate(horizon, drift, aim);
            float height = m.Sole - (horizon.Foot.y - horizon.Lower.y) + Leg.GapLimit - Foot.LandingMargin;
            float heightDebt = Mathf.Max(0f, -FootContactGeometryLimits.OrdinaryPenetrationMeters + Foot.LandingMargin - height);
            var shift = p.Move.Body.Position + dt * p.Move.Body.Velocity;
            var natural = Leg.TranslateShoe(horizon, m, aim, shift);
            float speed = dt * (Mathf.Abs(a.HipRate) + Mathf.Abs(a.KneeRate));
            float driftEffort = Mathf.Abs(drift.Hip) + Mathf.Abs(drift.Knee);
            for (int i = 0; i < NominationLimit; i++)
            {
                float score = i switch
                {
                    0 => driftEffort + speed,
                    1 => Shortfall(natural) + heightDebt,
                    2 => FootTransitionSource.Planar(p.Leg.Trial.Lower - p.Leg.Base.Lower).magnitude,
                    3 => -height,
                    4 => shift.magnitude + dt * p.Move.Body.Velocity.magnitude,
                    _ => speed + a.Effort * .25f
                };
                ref var n = ref _preview.Nominations[i];
                if (geometry < 0 || score < n.Score)
                    n = new Nomination { Path = p, Score = score };
            }
            _preview.HasNominee = true;
        }

        private static float TrackingRoom(in Leg.Measure m, in Leg.Measure native, float lower)
        {
            var d = new ShoeLoss(m, native);
            return Leg.Precision - Mathf.Max(lower, Mathf.Max(Mathf.Max(d.AnkleXZ, d.SoleXZ), Mathf.Max(d.AnkleAlong, d.SoleAlong)));
        }

        private static float PlanarShift(in Leg.Pose pose, in Leg.Measure m, in Leg.Measure native)
        {
            var solePoint = pose.Center - Vector3.up * pose.Half.y;
            var delta = m.Foot - native.Foot;
            var sole = delta + (m.ShoeRotation * solePoint - native.ShoeRotation * solePoint);
            return Mathf.Max(FootTransitionSource.Planar(m.Lower - native.Lower).magnitude,
                Mathf.Max(FootTransitionSource.Planar(delta).magnitude, FootTransitionSource.Planar(sole).magnitude));
        }

        private static float Debt(in Evidence e) => Mathf.Max(0f, Mathf.Max(
            Mathf.Max(-FootContactGeometryLimits.OrdinaryPenetrationMeters + Foot.LandingMargin - e.Sole,
                e.Sole - FootContactGeometryLimits.RecoveryMaxGapMeters), e.Gap - (Leg.GapLimit - Foot.LandingMargin)));

        private static int CompareWitness(in Evidence a, in Evidence b)
        {
            if (a.Feasible != b.Feasible) return a.Feasible ? -1 : 1;
            if (!a.Feasible) return 0;
            int order = Debt(a).CompareTo(Debt(b));
            if (order != 0) return order;
            // A refreshed animation target may turn. Minimise displacement from the native
            // material points once the two geometric margins are secured on the held target.
            order = a.PlanarShift.CompareTo(b.PlanarShift);
            return order != 0 ? order : b.TrackingRoom.CompareTo(a.TrackingRoom);
        }

        private Evidence LookAhead(in Path path, in Leg.Pose horizon, in Leg.Pose nominal, in Leg.Aim aim,
            Leg.Budget budget, float dt, float hipY, Step step)
        {
            var best = new Evidence { Evaluated = true };
            budget.Axis = nominal.Axis;
            var strict = _preview.NextStrict;
            var extra = _preview.NextExtra;
            Leg.Select(horizon, aim, budget, dt, hipY, path.Leg.Offset, true, true, strict, extra, distinctAccelerations: true);
            for (int i = 0; i < strict.Count + extra.Count; i++)
            {
                var leg = i < strict.Count ? strict.Items[i] : extra.Items[i - strict.Count];
                float lower = new LowerLoss(leg.Trial, leg.Base, aim).Worst;
                if (lower > Leg.Precision) continue;
                // The shoe cannot move the shank. Even perfect traction could not rank this option higher.
                if (best.Feasible && Debt(best) == 0f &&
                    FootTransitionSource.Planar(leg.Trial.Lower - leg.Base.Lower).magnitude > best.PlanarShift) continue;
                step.NextCommands++;
                var move = Foot.Preview(horizon, leg, aim, step.Command, dt, nominal.Foot - nominal.Lower, path.Move);
                move = Foot.Balance(horizon, leg, aim, step.Command, dt, path.Move.Body, move);
                if (move.Status != Foot.State.Predicted) continue;
                var e = new Evidence
                {
                    Evaluated = true,
                    Feasible = true,
                    Sole = move.Forecast.Sole,
                    Gap = move.Forecast.Gap,
                    NextOffset = leg.Offset,
                    NextShift = move.Body.Position,
                    PlanarShift = PlanarShift(horizon, move.Forecast, leg.Base),
                    Lower = lower,
                    TrackingRoom = TrackingRoom(move.Forecast, leg.Base, lower)
                };
                if (CompareWitness(e, best) < 0) best = e;
            }
            return best;
        }

        internal struct WitnessRead
        {
            internal float Sole, Gap, Lower;
            internal bool Geometry, Tracking;
        }

        // Stop-time comparison on two verified native solves. The second command is the
        // hypothetical command frozen at N. Its N+2 receipt cannot select any N+1 branch.
        internal static bool TryReadWitness(in Path path, Step issued, Step next, in Leg.Aim aim, in Leg.Measure native,
            out WitnessRead read)
        {
            read = default;
            if (!path.SafeRead || !path.Horizon.Feasible || issued == null || next == null || !next.HasPhysicalPose ||
                !issued.IssueClock.Valid || !issued.VerifiedReadClock.Matches(next.IssueClock) ||
                !next.VerifiedReadClock.Valid || issued.IssueClock.Epoch != next.VerifiedReadClock.Epoch ||
                issued.IssueClock.Epoch != next.IssueClock.Epoch ||
                next.IssueClock.Sequence != issued.IssueClock.Sequence + 1 ||
                next.VerifiedReadClock.Sequence != next.IssueClock.Sequence + 1 ||
                issued.IssueClock.Dt != next.IssueClock.Dt) return false;
            var e = path.Horizon;
            var pose = next.PhysicalPose;
            var leg = Leg.Evaluate(pose, e.NextOffset, aim);
            var shoe = Leg.TranslateShoe(pose, leg, aim, e.NextShift);
            var lower = new LowerLoss(leg, native, aim);
            read = new WitnessRead
            {
                Sole = shoe.Sole,
                Gap = shoe.Gap,
                Lower = lower.Worst,
                Geometry = leg.Safe && leg.MaintainsHeight(native) && shoe.GeometricPass &&
                    shoe.Gap < issued.Command.Tether.HardGap && shoe.MaintainsHeight(native) && Foot.Margin(shoe) >= Foot.LandingMargin,
                Tracking = lower.Worst <= Leg.Precision && shoe.Tracks(native)
            };
            return true;
        }

        private void PickPreview(Step step, in Leg.Pose horizon, in Leg.Pose nominal, in Leg.Aim aim, in Leg.Budget budget,
            float dt, float hipY)
        {
            Path winner = default;
            bool found = false;
            if (!_preview.HasNominee) return;
            for (int i = 0; i < NominationLimit; i++)
            {
                var p = _preview.Nominations[i].Path;
                bool seen = false;
                for (int j = 0; j < i; j++)
                    if (SameState(p, _preview.Nominations[j].Path)) { seen = true; break; }
                if (seen) continue;
                step.Shortlisted++;
                p.Horizon = LookAhead(p, horizon, nominal, aim, budget, dt, hipY, step);
                for (int j = 0; j < step.Count; j++)
                {
                    if (!SameState(p, step.Paths[j])) continue;
                    step.Paths[j].Horizon = p.Horizon;
                    seen = true;
                    break;
                }
                if (seen) continue;
                int geometry = found ? CompareGeometry(p, winner) : -1;
                if (geometry > 0 || (geometry == 0 && CompareWitness(p.Horizon, winner.Horizon) >= 0)) continue;
                found = true; winner = p;
            }
            if (!found || step.Count == Width) return;
            winner.Protected = false;
            winner.PreviewSlot = true;
            winner.Id = ++_nextId;
            step.Paths[step.Count++] = winner;
        }
    }
}
