using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One optional choice inside Foot's existing Render R/W/A step. Primary geometry
    // uses the selected Knee INPUT; native Lower geometry is a same-Render non-regression
    // check. Neither is a prediction of the next Fixed Knee or the native contact solve.
    internal static class FootGroundTrajectory
    {
        internal enum Status { Unavailable, InvalidGeometry, BaselineClear, EndpointBelow,
            NoProgress, NoHeadroom, NoLegalImprovement, Improved }

        internal struct Input
        {
            internal SourceStepContext Source;
            internal PhysicsStepContext GeometryStep;
            internal int ParentId, UpperId, LowerId, SupportId;
            internal KneeGroundTarget.Geometry Geometry;
            internal Quaternion KneeGoal, NativeLower;
            internal bool CurrentKneeBasis; // KneeGoal is the completed reference only for an adopted v135 waypoint.
            internal Vector3 NativeAnkle;
            internal bool Matches(SourceStepContext source) => Source.Valid && Source.Matches(source) &&
                GeometryStep.Valid && GeometryStep.Epoch == source.Epoch &&
                GeometryStep.SourceSequence > 0 && GeometryStep.SourceSequence < source.Sequence &&
                source.Dt < .1f && source.Time + .0001f >= GeometryStep.UnscaledTime &&
                source.Time - GeometryStep.UnscaledTime <= .1f &&
                ParentId != 0 && UpperId != 0 && LowerId != 0 && SupportId != 0 &&
                ParentId != UpperId && ParentId != LowerId && UpperId != LowerId &&
                Geometry.Valid && Q(KneeGoal) && Q(NativeLower) && V(NativeAnkle);
        }

        internal struct Result
        {
            internal Status Reason;
            internal bool Evaluated, Better, Applied, CurrentKneeBasis;
            internal float BaselineMin, GoalMin, CandidateMin, NativeBaselineMin, NativeCandidateMin, DeltaDegrees;
            internal Quaternion R;
            internal Vector3 W;
            internal string Text() => "footGroundTrajectoryV041340=" + Reason +
                "/footTrajectory(evaluated-better-applied)=" + Evaluated + "|" + Better + "|" + Applied +
                "/footTrajectoryMinMm(base-goal-candidate)=" + (Evaluated ?
                    N(BaselineMin * 1000f) + "|" + N(GoalMin * 1000f) + "|" + N(CandidateMin * 1000f) : "NA|NA|NA") +
                "/footTrajectoryNativeMinMm(base-candidate)=" + (Evaluated ?
                    N(NativeBaselineMin * 1000f) + "|" + N(NativeCandidateMin * 1000f) : "NA|NA") +
                "/footTrajectoryDeltaDeg=" + N(DeltaDegrees) + "/footTrajectoryKneeBasis=" +
                (CurrentKneeBasis ? "COMPLETED_KNEE_REFERENCE" : "JOINT_INPUT_GOAL");
        }

        internal static Result Evaluate(Input input, Quaternion previousR, Vector3 previousW,
            Quaternion goal, Vector3 nominalW, Quaternion baselineR, Vector3 baselineW, float dt)
        {
            var result = new Result { Reason = Status.InvalidGeometry, R = baselineR, W = baselineW, CurrentKneeBasis = input.CurrentKneeBasis };
            if (!input.Matches(input.Source) || !Q(previousR) || !Q(goal) || !Q(baselineR) ||
                !V(previousW) || !V(nominalW) || !V(baselineW) || !F(dt) || dt <= .0001f || dt >= .25f)
                return result;
            var g = input.Geometry;
            Vector3 ankle = KneeGroundTarget.Ankle(g, input.KneeGoal);
            Quaternion lower = g.Upper * input.KneeGoal;
            float top = g.FloorCenter.y + g.FloorHalf.y;
            float baselineMin = FootBoxGeometry.Minimum(ankle, lower * baselineR, g.Center, g.Half) - top;
            float goalMin = FootBoxGeometry.Minimum(ankle, lower * goal, g.Center, g.Half) - top;
            float nativeMin = FootBoxGeometry.Minimum(input.NativeAnkle, input.NativeLower * baselineR, g.Center, g.Half) - top;
            if (!F(baselineMin) || !F(goalMin) || !F(nativeMin) || !Inside(g, ankle, lower, baselineR) ||
                !Inside(g, ankle, lower, goal) || !Inside(g, input.NativeAnkle, input.NativeLower, baselineR) ||
                !FootGroundReference.GoalWithinBounds(lower, g.FootRelative, goal, g.FootBindRelative)) return result;
            result.Evaluated = true; result.BaselineMin = result.CandidateMin = baselineMin;
            result.GoalMin = goalMin; result.NativeBaselineMin = result.NativeCandidateMin = nativeMin;
            if (baselineMin >= -KneeGroundTarget.MinGain) { result.Reason = Status.BaselineClear; return result; }
            if (goalMin < 0f) { result.Reason = Status.EndpointBelow; return result; }
            Vector3 remaining = FootRelativeReference.Log(goal * Quaternion.Inverse(baselineR));
            float oldGap = remaining.magnitude;
            if (oldGap < .000001f) { result.Reason = Status.NoProgress; return result; }
            Vector3 direction = remaining / oldGap; // Lower-local angular-velocity basis.
            const float maxW = FootRelativeReference.MaxW, maxA = FootRelativeReference.MaxA;
            float speedBound = Mathf.Max(maxW, previousW.magnitude);
            float room = (float)Math.Min(Math.Min(
                GroundReferenceStepBudget.PositiveBallExit(baselineW - previousW, direction, maxA * dt),
                GroundReferenceStepBudget.PositiveBallExit(baselineW, direction, speedBound)), oldGap / dt);
            if (!F(room) || room <= .000001f) { result.Reason = Status.NoHeadroom; return result; }
            room *= .99999f;
            float oldDebt = GroundReferenceStepBudget.BrakeDebt(baselineW - nominalW, direction, oldGap, maxA, dt);
            result.Reason = Status.NoLegalImprovement;
            // Bounded allocation-free search; A evaluates the same candidate but keeps
            // the baseline. Trace selection never decides which R/W/A is published.
            for (int i = 1; i <= 16; i++)
            {
                Vector3 w = baselineW + direction * (room * i / 16f);
                if (!V(w) || w.magnitude > speedBound + .00001f ||
                    ((w - previousW) / dt).magnitude > maxA + .0001f || w.magnitude * dt >= Mathf.PI - .0001f) continue;
                Quaternion r = FootRelativeReference.Advance(previousR, w, dt);
                Vector3 error = FootRelativeReference.Log(goal * Quaternion.Inverse(r));
                float along = Vector3.Dot(error, direction);
                if (!Q(r) || !V(error) || error.magnitude > oldGap + .000001f || along < -.000001f ||
                    GroundReferenceStepBudget.BrakeDebt(w - nominalW, direction, along, maxA, dt) > oldDebt + .000001f ||
                    !FootGroundReference.GoalWithinBounds(lower, g.FootRelative, r, g.FootBindRelative) ||
                    !FootGroundReference.GoalWithinBounds(input.NativeLower, g.FootRelative, r, g.FootBindRelative) ||
                    !Inside(g, ankle, lower, r) || !Inside(g, input.NativeAnkle, input.NativeLower, r)) continue;
                float min = FootBoxGeometry.Minimum(ankle, lower * r, g.Center, g.Half) - top;
                float native = FootBoxGeometry.Minimum(input.NativeAnkle, input.NativeLower * r, g.Center, g.Half) - top;
                if (!F(min) || !F(native) || min - baselineMin < KneeGroundTarget.MinGain ||
                    min <= result.CandidateMin || native < nativeMin) continue;
                result.Reason = Status.Improved; result.Better = true;
                result.R = r; result.W = w; result.CandidateMin = min; result.NativeCandidateMin = native;
                result.DeltaDegrees = FootRelativeReference.Log(r * Quaternion.Inverse(baselineR)).magnitude * Mathf.Rad2Deg;
                if (min >= 0f) break;
            }
            return result;
        }

        private static bool Inside(KneeGroundTarget.Geometry g, Vector3 ankle, Quaternion lower, Quaternion foot) =>
            FootBoxGeometry.Inside(ankle, lower * foot, g.Center, g.Half, g.FloorCenter, g.FloorHalf);
        private static bool F(float x) => PelvisParentFrame.F(x);
        private static bool V(Vector3 x) => PelvisParentFrame.V(x);
        private static bool Q(Quaternion x) => PelvisParentFrame.Q(x);
        internal static string N(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
    }
}
