using System;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // A geometric reference on the real unloaded forecast. Angles and rates
    // are scalar offsets in the moving, calibrated flexion plane. The pelvis and both
    // measured joint gaps keep their nominal world positions/offsets. No wrench is issued.
    internal static class TurnHandoffTrajectory
    {
        internal const int ComparisonDepth = 3;
        internal const float Precision = KneeGroundTarget.MinGain;
        internal const float GapLimit = FootContactGeometryLimits.RecoveryMaxAnchorGapMeters;

        internal struct Pose
        {
            internal Vector3 Hip, UpperKnee, LowerKnee, Lower, Foot, Axis;
            internal Quaternion Rotation;
            internal bool AllowExtension;
            internal Vector3 Center, Half, FloorCenter, FloorHalf;

            internal bool Valid => Finite(Hip) && Finite(UpperKnee) && Finite(LowerKnee) && Finite(Lower) &&
                Finite(Foot) && Finite(Axis) && Mathf.Abs(Axis.sqrMagnitude - 1f) < .001f &&
                (UpperKnee - Hip).sqrMagnitude > .0001f && (Lower - LowerKnee).sqrMagnitude > .0001f &&
                RightFootTorqueForecast.ValidRotation(Rotation) && Finite(Center) && Positive(Half) &&
                Finite(FloorCenter) && Positive(FloorHalf);

            internal bool SameBox(in Pose other) => AllowExtension == other.AllowExtension && (Center - other.Center).sqrMagnitude <= 1e-12f &&
                (Half - other.Half).sqrMagnitude <= 1e-12f && (FloorCenter - other.FloorCenter).sqrMagnitude <= 1e-12f &&
                (FloorHalf - other.FloorHalf).sqrMagnitude <= 1e-12f;
        }

        internal struct Budget
        {
            internal Vector3 Axis, HipA, UpperW, LowerW, FootW;
            internal float HipCap, KneeA, KneeCap, UpperMaxW, LowerMaxW, FootMaxW;
            internal bool Valid => Finite(Axis) && Mathf.Abs(Axis.sqrMagnitude - 1f) < .001f &&
                Finite(HipA) && Finite(UpperW) && Finite(LowerW) && Finite(FootW) &&
                Positive(HipCap) && Positive(KneeCap) && Positive(UpperMaxW) && Positive(LowerMaxW) && Positive(FootMaxW) &&
                HipA.magnitude <= HipCap + .0001f && Mathf.Abs(KneeA) <= KneeCap + .0001f &&
                UpperW.magnitude <= UpperMaxW + .0001f && LowerW.magnitude <= LowerMaxW + .0001f && FootW.magnitude <= FootMaxW + .0001f;
        }

        internal struct Offset
        {
            internal float Hip, Knee, HipRate, KneeRate;
            internal bool Changed => Hip != 0f || Knee != 0f || HipRate != 0f || KneeRate != 0f;
            internal float Effort => Mathf.Abs(Hip) + Mathf.Abs(Knee);
        }

        internal struct Aim
        {
            internal Vector3 Target, TargetSole, Travel;
            internal bool Valid => Finite(Target) && Finite(TargetSole) && Finite(Travel) &&
                Mathf.Abs(Travel.y) < .0001f && Mathf.Abs(Travel.sqrMagnitude - 1f) < .001f;
        }

        internal struct Measure
        {
            internal Vector3 Lower, Foot;
            internal Quaternion ShoeRotation;
            internal float Sole, Gap, KneeGap, LowerRise, AnkleAlong, SoleAlong, AnkleXZ, SoleXZ;
            internal bool Inside, BendLegal;
            internal bool GeometricPass => Inside && BendLegal && RightFootTorqueForecast.Band(Sole) == RightFootTorqueForecast.HeightBand.InBand &&
                Gap <= GapLimit && KneeGap < FootContactGeometryLimits.AnchorValidityMeters;
            internal bool Safe => Inside && BendLegal && Gap < FootContactGeometryLimits.AnchorValidityMeters &&
                KneeGap < FootContactGeometryLimits.AnchorValidityMeters;
            internal bool MaintainsHeight(in Measure baseline) => LowerRise >= -Precision && Sole >= baseline.Sole - Precision &&
                Sole <= Mathf.Max(baseline.Sole, FootContactGeometryLimits.RecoveryMaxGapMeters);
            internal bool Tracks(in Measure baseline) => AnkleXZ <= baseline.AnkleXZ + Precision &&
                SoleXZ <= baseline.SoleXZ + Precision && Mathf.Abs(AnkleAlong) <= Mathf.Abs(baseline.AnkleAlong) + Precision &&
                Mathf.Abs(SoleAlong) <= Mathf.Abs(baseline.SoleAlong) + Precision;
        }

        internal enum Choice { MissingGeometry, MissingBudget, Outside, NoContinuation, Baseline, Prepared }
        internal struct Plan
        {
            internal Choice Result;
            internal Offset Offset;
            internal float HipDrop, Wanted, HipDeltaA, KneeDeltaA;
            internal int Tested, Safe, Tracking;
            internal Measure Base, Trial;
            internal bool Issued => Result == Choice.Baseline || Result == Choice.Prepared;
        }

        private static bool Finite(Vector3 v) => HipPointPairMobility.V(v) && float.IsFinite(v.sqrMagnitude);
        private static bool Positive(float f) => float.IsFinite(f) && f > 0f;
        private static bool Positive(Vector3 v) => Finite(v) && v.x > 0f && v.y > 0f && v.z > 0f;

        // Preserve the two distinct anchors at each joint. Keep the existing relative
        // ankle attitude, so a shank turn may lower the toe even if the ankle rises.
        internal static Measure Evaluate(in Pose p, in Offset offset, in Aim aim, Vector3 footShift = default)
        {
            var thigh = p.UpperKnee - p.Hip;
            var shin = p.Lower - p.LowerKnee;
            var hipTurn = Quaternion.AngleAxis(offset.Hip * Mathf.Rad2Deg, p.Axis);
            var shankTurn = Quaternion.AngleAxis((offset.Hip + offset.Knee) * Mathf.Rad2Deg, p.Axis);
            var thighNext = hipTurn * thigh;
            var shinNext = shankTurn * shin;
            var delta = (thighNext - thigh) + (shinNext - shin);
            bool bendLegal = true;
            if (offset.Knee != 0f)
            {
                float before = Bend(thigh, shin), after = Bend(thighNext, shinNext);
                float oldSide = Vector3.Dot(Vector3.Cross(thigh, shin), p.Axis);
                float newSide = Vector3.Dot(Vector3.Cross(thighNext, shinNext), p.Axis);
                bendLegal = oldSide * newSide >= 0f && (p.AllowExtension || after >= before - .001f) &&
                    after >= Mathf.Min(before, KneeGroundTarget.MinExtensionBendDegrees) - .001f &&
                    after <= Mathf.Max(before, KneeGroundTarget.MaxBendDegrees) + .001f;
            }
            var leg = new Measure {
                Foot = p.Foot + delta, Lower = p.Lower + delta, LowerRise = delta.y,
                KneeGap = (p.LowerKnee - p.UpperKnee).magnitude, BendLegal = bendLegal,
                ShoeRotation = shankTurn * p.Rotation
            };
            return TranslateShoe(p, leg, aim, footShift);
        }

        // A traction fraction changes only the shoe position. Reuse the leg posture
        // evaluated at this endpoint, including its bend and material shoe rotation.
        internal static Measure TranslateShoe(in Pose p, Measure leg, in Aim aim, Vector3 shift)
        {
            leg.Foot += shift;
            var rotation = leg.ShoeRotation;
            var ankleError = FootTransitionSource.Planar(leg.Foot - aim.Target);
            var sole = leg.Foot + rotation * (p.Center - Vector3.up * p.Half.y);
            var soleError = FootTransitionSource.Planar(sole - aim.TargetSole);
            leg.Sole = FootBoxGeometry.Minimum(leg.Foot, rotation, p.Center, p.Half) - (p.FloorCenter.y + p.FloorHalf.y);
            leg.Gap = (p.Foot - p.Lower + shift).magnitude;
            leg.Inside = FootBoxGeometry.Inside(leg.Foot, rotation, p.Center, p.Half, p.FloorCenter, p.FloorHalf);
            leg.AnkleAlong = Vector3.Dot(ankleError, aim.Travel);
            leg.SoleAlong = Vector3.Dot(soleError, aim.Travel);
            leg.AnkleXZ = ankleError.magnitude;
            leg.SoleXZ = soleError.magnitude;
            return leg;
        }

        private static float Bend(Vector3 thigh, Vector3 shin) =>
            Mathf.Atan2(Vector3.Cross(thigh, shin).magnitude, Vector3.Dot(thigh, shin)) * Mathf.Rad2Deg;

        // Both rules use only the upper hip descent forecast at issue time. The earlier
        // rule can spend it before penetration, up to the existing upper clearance bound.
        internal static float WantedRise(float originHip, float endpointHip, float baselineSole, bool anticipate = false)
        {
            float ceiling = anticipate ? FootContactGeometryLimits.RecoveryMaxGapMeters : 0f;
            return Mathf.Min(Mathf.Max(0f, originHip - endpointHip), Mathf.Max(0f, ceiling - baselineSole));
        }

        // Test holding acceleration, braking the carried rate, and ±half/full remaining
        // acceleration. A saturated nominal command can still be reduced. Both sides
        // of the existing vector/scalar cap are checked again on the resulting command.
        private static float Acceleration(int slot, float negative, float positive, float rate, float dt)
        {
            switch (slot)
            {
                case 0: return 0f;
                case 1: return Mathf.Clamp(-rate / dt, -negative, positive);
                case 2: return -negative * .5f;
                case 3: return positive * .5f;
                case 4: return -negative;
                default: return positive;
            }
        }

        private static int Accelerations(Span<float> values, float negative, float positive, float rate, float dt, bool distinct)
        {
            int count = 0;
            for (int i = 0; i < 6; i++)
            {
                float a = Acceleration(i, negative, positive, rate, dt);
                if (distinct && values.Slice(0, count).IndexOf(a) >= 0) continue;
                values[count++] = a;
            }
            return count;
        }

        // One more semi-implicit step at the current signed braking authority. No future command is read.
        private static bool CanBrakeNext(float angle, float rate, float negative, float positive, float dt, float limit)
        {
            float remaining = rate > 0f ? Mathf.Max(0f, rate - negative * dt) : Mathf.Min(0f, rate + positive * dt);
            return Mathf.Abs(angle + remaining * dt) <= limit;
        }

        // The original selector still ranks by leg height. The combined observer
        // receives the legal choices from the very same bounds and flexion axes.
        internal sealed class Options
        {
            internal const int Capacity = 6 * 6;
            internal readonly Plan[] Items = new Plan[Capacity];
            internal int Count;
            internal void Add(in Plan p)
            {
                for (int i = 0; i < Count; i++)
                    if (Items[i].Offset.HipRate == p.Offset.HipRate && Items[i].Offset.KneeRate == p.Offset.KneeRate) return;
                Items[Count++] = p;
            }
        }

        internal static Plan Select(in Pose nominal, in Aim aim, in Budget budget, float dt, float originHip, in Offset previous,
            bool anticipate = false, bool reserveBrake = false, Options options = null, Options afterTraction = null, bool distinctAccelerations = false)
        {
            if (options != null) options.Count = 0;
            if (afterTraction != null) afterTraction.Count = 0;
            var plan = new Plan();
            if (!nominal.Valid || !aim.Valid || !Positive(dt) || dt > .1f || !float.IsFinite(originHip)) return plan;
            plan.Base = Evaluate(nominal, default, aim);
            plan.HipDrop = originHip - nominal.Hip.y;
            plan.Wanted = WantedRise(originHip, nominal.Hip.y, plan.Base.Sole, anticipate);
            if (!plan.Base.Safe) { plan.Result = Choice.Outside; return plan; }
            if (!budget.Valid) { plan.Result = Choice.MissingBudget; return plan; }
            if (plan.Wanted < Precision && !previous.Changed)
            {
                plan.Result = Choice.Baseline; plan.Trial = plan.Base;
                options?.Add(plan);
                return plan;
            }

            float negative = Mathf.Min(budget.HipCap, (float)GroundReferenceStepBudget.PositiveBallExit(budget.HipA, -budget.Axis, budget.HipCap));
            float positive = Mathf.Min(budget.HipCap, (float)GroundReferenceStepBudget.PositiveBallExit(budget.HipA, budget.Axis, budget.HipCap));
            float kneeNeg = Mathf.Clamp(budget.KneeCap + budget.KneeA, 0f, budget.KneeCap);
            float kneePos = Mathf.Clamp(budget.KneeCap - budget.KneeA, 0f, budget.KneeCap);
            float best = float.PositiveInfinity;
            plan.Result = Choice.NoContinuation;
            const float angleLimit = KneeGroundTarget.MaxCorrectionDegrees * Mathf.Deg2Rad;
            Span<float> hipA = stackalloc float[6], kneeA = stackalloc float[6];
            int hipCount = Accelerations(hipA, negative, positive, previous.HipRate, dt, distinctAccelerations);
            int kneeCount = Accelerations(kneeA, kneeNeg, kneePos, previous.KneeRate, dt, distinctAccelerations);
            for (int h = 0; h < hipCount; h++)
            {
                float ha = hipA[h];
                float hw = previous.HipRate + ha * dt, hip = previous.Hip + hw * dt;
                if (Mathf.Abs(hip) > angleLimit || (reserveBrake && !CanBrakeNext(hip, hw, negative, positive, dt, angleLimit)) ||
                    (budget.HipA + budget.Axis * ha).magnitude > budget.HipCap + .0001f ||
                    (budget.UpperW + nominal.Axis * hw).magnitude > budget.UpperMaxW + .0001f) continue;
                for (int k = 0; k < kneeCount; k++)
                {
                    float ka = kneeA[k];
                    float kw = previous.KneeRate + ka * dt, knee = previous.Knee + kw * dt;
                    var downstreamRate = nominal.Axis * (hw + kw);
                    if (Mathf.Abs(knee) > angleLimit || (reserveBrake && !CanBrakeNext(knee, kw, kneeNeg, kneePos, dt, angleLimit)) ||
                        Mathf.Abs(budget.KneeA + ka) > budget.KneeCap + .0001f ||
                        (budget.LowerW + downstreamRate).magnitude > budget.LowerMaxW + .0001f ||
                        (budget.FootW + downstreamRate).magnitude > budget.FootMaxW + .0001f) continue;
                    var offset = new Offset { Hip = hip, Knee = knee, HipRate = hw, KneeRate = kw };
                    var trial = Evaluate(nominal, offset, aim);
                    plan.Tested++;
                    if (!trial.Safe || !trial.MaintainsHeight(plan.Base)) continue;
                    plan.Safe++;
                    bool tracks = trial.Tracks(plan.Base);
                    var sink = tracks ? options : afterTraction;
                    if (sink != null)
                    {
                        var legal = new Plan {
                            Result = offset.Changed ? Choice.Prepared : Choice.Baseline, Offset = offset,
                            Base = plan.Base, Trial = trial, Wanted = plan.Wanted, HipDrop = plan.HipDrop,
                            HipDeltaA = ha, KneeDeltaA = ka
                        };
                        sink.Add(legal);
                    }
                    if (!tracks) continue;
                    plan.Tracking++;
                    // Both the lower-side ankle and the actual lowest shoe corner must rise.
                    float error = Mathf.Max(Mathf.Abs(plan.Wanted - trial.LowerRise),
                        Mathf.Abs(plan.Wanted - (trial.Sole - plan.Base.Sole)));
                    if (error > best || (error == best && offset.Effort >= plan.Offset.Effort)) continue;
                    best = error; plan.Result = offset.Changed ? Choice.Prepared : Choice.Baseline;
                    plan.Offset = offset; plan.Trial = trial; plan.HipDeltaA = ha; plan.KneeDeltaA = ka;
                }
            }
            return plan;
        }
    }
}
