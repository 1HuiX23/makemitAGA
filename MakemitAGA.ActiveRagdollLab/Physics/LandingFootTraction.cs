using UnityEngine;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Translation = MakemitAGA.ActiveRagdollLab.Physics.LeftLegHeightPreparation.Translation;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Conditional on the same guarded leg pose, shoe attitude and committed parent transport.
    // Only the additional foot translation changes. No motion is written to a Rigidbody.
    internal static partial class LandingFootTraction
    {
        internal struct Input
        {
            internal RightAnkleLandingSequence.Tether Tether;
            internal Vector3 Applied;
            internal float Drag;
            internal Input(in RightFootLandingForecast.Motion motion)
            { Tether = motion.Tether; Applied = motion.AppliedLinearA; Drag = motion.LinearDrag; }

            internal float Cap => Mathf.Min(Tether.BaseCap, Tether.SelectedCap);
            internal bool Valid => float.IsFinite(Tether.BaseCap) && float.IsFinite(Tether.SelectedCap) && Cap > 0f &&
                float.IsFinite(Tether.Kp) && Tether.Kp > 0f && float.IsFinite(Tether.Kd) && Tether.Kd >= 0f &&
                float.IsFinite(Tether.HardGap) && Tether.HardGap > Trial.GapLimit &&
                float.IsFinite(Drag) && Drag >= 0f && Finite(Tether.Raw) && Finite(Applied) &&
                Applied.magnitude <= Cap + .00001f &&
                (Vector3.ClampMagnitude(Tether.Raw, Tether.SelectedCap) - Applied).sqrMagnitude <= 1e-8f;
        }

        // Reserve a quarter millimetre on each of the two limiting ordinary boundaries.
        internal const float LandingMargin = .00025f;

        internal enum State { MissingInput, MissingPrevious, StartBounds, UnsafeForecast, Predicted, Read }
        internal struct Move
        {
            internal State Status;
            internal Translation Body;
            internal Vector3 Applied, RequestedShift;
            internal Trial.Measure Forecast, Settled;
            internal float ClearanceNeed, Fraction;
            internal bool HeightReachable, SafeRead, Pass, PlanarAdjusted;
            internal float WorstMargin => Mathf.Min(Margin(Forecast), Margin(Settled));
            internal bool MarginPass => Pass && WorstMargin >= LandingMargin;
        }
        internal sealed class Pair
        {
            internal Input Command;
            internal Move One, Two, BufferedOne, BufferedTwo, Continuous;
        }
        private static bool Finite(Vector3 v) => HipPointPairMobility.V(v) && float.IsFinite(v.sqrMagnitude);

        internal static float Margin(in Trial.Measure m) =>
            Mathf.Min(m.Sole + FootContactGeometryLimits.OrdinaryPenetrationMeters, Trial.GapLimit - m.Gap);

        // Minimum planar closure for this requested Y. Leave one micrometre for
        // roundoff in the forecast. Scoring below still uses the original thresholds.
        internal static Vector3 GeometryRequest(Vector3 gap, float sole, out float rise, out bool heightReachable, bool buffered = false)
        {
            const float inset = .000001f;
            float margin = buffered ? LandingMargin : 0f;
            float floor = -FootContactGeometryLimits.OrdinaryPenetrationMeters + margin;
            float limit = Trial.GapLimit - margin;
            rise = Mathf.Max(0f, floor - sole);
            if (rise == 0f && gap.magnitude <= limit) { heightReachable = true; return Vector3.zero; }
            float radius = limit - inset;
            // A shoe already above the lower bound can also lower slightly to enter the ankle sphere.
            heightReachable = gap.y + (floor - sole) <= limit;
            float wantedY = gap.y + (rise > 0f ? rise + inset : 0f);
            float y = Mathf.Clamp(wantedY, -radius, radius);
            float slice = Mathf.Sqrt(Mathf.Max(0f, (radius-y) * (radius+y)));
            var flat = FootTransitionSource.Planar(gap);
            float length = flat.magnitude;
            // When already above the positive pole, let Y descend only as far as the ordinary shoe floor permits.
            float dy = Mathf.Max(y - gap.y, Mathf.Min(0f, floor - sole));
            return Vector3.up * dy - (length > slice ? flat * ((length - slice) / length) : Vector3.zero);
        }

        internal static Move Predict(in Trial.Pose pose, in Trial.Plan leg, in Trial.Aim aim, in Input input,
            float dt, Vector3 startGap, in Move previous, bool second, bool buffered = false)
        {
            if (second && (previous.Status != State.Read || !previous.SafeRead))
                return new Move { Status = State.MissingPrevious };
            return Forecast(pose, leg, aim, input, dt, startGap, second ? previous.Body : default, second, buffered);
        }

        // A second prediction carries a forecast state. It has no physical receipt and never enters Read().
        internal static Move Preview(in Trial.Pose pose, in Trial.Plan leg, in Trial.Aim aim, in Input input,
            float dt, Vector3 startGap, in Move issued)
        {
            if (issued.Status != State.Predicted) return new Move { Status = State.MissingPrevious };
            return Forecast(pose, leg, aim, input, dt, startGap, issued.Body, true, true);
        }

        private static Move Forecast(in Trial.Pose pose, in Trial.Plan leg, in Trial.Aim aim, in Input input,
            float dt, Vector3 startGap, in Translation old, bool continued, bool buffered)
        {
            var result = new Move();
            if (!leg.Issued || !input.Valid || !float.IsFinite(dt) || dt <= 0f || dt > .1f) return result;
            float hard = Mathf.Min(input.Tether.HardGap, FootContactGeometryLimits.AnchorValidityMeters);
            if (!Finite(startGap) || (startGap + old.Position).magnitude >= hard)
            { result.Status = State.StartBounds; return result; }
            float damping = Mathf.Max(0f, 1f - input.Drag * dt), response = dt * dt * damping;
            if (response <= 0f || !float.IsFinite(response)) return result;
            var raw = input.Tether.Raw - old.Position * input.Tether.Kp - old.Velocity * input.Tether.Kd;
            if (!Finite(raw)) return result;
            var feedback = continued ? Vector3.ClampMagnitude(raw, input.Cap) : input.Applied;
            var free = old.Advance(feedback - input.Applied, dt, damping);
            // Both ankles inherit the same translation from the leg-angle substitution.
            var delta = pose.Foot - pose.Lower + free.Position;
            result.RequestedShift = GeometryRequest(delta, leg.Trial.Sole + free.Position.y,
                out result.ClearanceNeed, out result.HeightReachable, buffered);
            float distance = result.RequestedShift.magnitude;
            Vector3 direction = distance > 0f ? result.RequestedShift / distance : Vector3.zero;
            float extra = distance > 0f ? Mathf.Min(distance / response,
                (float)GroundReferenceStepBudget.PositiveBallExit(feedback, direction, input.Cap)) : 0f;
            // Choose the largest safe fraction on this ray, including the unmodified feedback.
            for (int i = 0; i < 4; i++)
            {
                float fraction = i == 0 ? 1f : i == 1 ? .5f : i == 2 ? .25f : 0f;
                var applied = feedback + direction * (extra * fraction);
                var body = old.Advance(applied - input.Applied, dt, damping);
                var measure = Trial.TranslateShoe(pose, leg.Trial, aim, body.Position);
                if (Finite(body.Velocity) && Finite(body.Position) && Finite(applied) &&
                    applied.magnitude <= input.Cap + .00001f && measure.Gap < hard && measure.Safe &&
                    measure.MaintainsHeight(leg.Base) && measure.Tracks(leg.Base))
                {
                    result.Status = State.Predicted; result.Body = body; result.Applied = applied; result.Forecast = measure;
                    result.Fraction = distance > 0f ? Mathf.Clamp(extra * fraction * response / distance, 0f, 1f) : 0f;
                    return result;
                }
                if (extra == 0f) break;
            }
            result.Status = State.UnsafeForecast;
            return result;
        }

        internal static void Settle(ref Move move, in Trial.Pose pose, in Trial.Offset chosen,
            in Trial.Aim aim, in Trial.Measure original, float hardGap)
        {
            if (move.Status != State.Predicted) return;
            var leg = Trial.Evaluate(pose, chosen, aim);
            SettleOnLeg(ref move, pose, leg, aim, original, hardGap);
        }

        internal static void SettleOnLeg(ref Move move, in Trial.Pose pose, in Trial.Measure leg,
            in Trial.Aim aim, in Trial.Measure original, float hardGap)
        {
            if (move.Status != State.Predicted) return;
            move.Settled = Trial.TranslateShoe(pose, leg, aim, move.Body.Position);
            move.SafeRead = move.Settled.Safe && move.Settled.Gap < hardGap && move.Settled.MaintainsHeight(original) && move.Settled.Tracks(original);
            move.Pass = move.SafeRead && move.Forecast.GeometricPass && move.Settled.GeometricPass;
            move.Status = State.Read;
        }
    }
}
