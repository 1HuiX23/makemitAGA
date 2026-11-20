using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // A waypoint for the EXISTING Foot Render planner. The Knee endpoint stays
    // unchanged. Completed Knee R is a held controller reference, never a claim
    // that native Lower already reached it or a forecast of the next solve.
    internal static class FootKneeProgressTarget
    {
        internal enum Status { Unavailable, InvalidGeometry, BaselineClear, NoGoalGain,
            NativeGoalRegression, PreviewUnavailable, NoStepGain, NativeStepRegression, Improved }
        internal struct Result
        {
            internal SourceStepContext Source;
            internal Status Reason;
            internal bool Selected, Evaluated, Previewed, Better, Applied;
            internal float GoalBaselineMin, GoalCandidateMin, NativeGoalBaselineMin, NativeGoalCandidateMin;
            internal float StepBaselineMin, StepCandidateMin, NativeStepBaselineMin, NativeStepCandidateMin;
            internal float IdealAtCandidateMin, KneeGapDegrees;
            internal string Text() => "footKneeProgressV041350=" + Reason +
                "/kneeProgress(selected-evaluated-previewed-better-applied)=" + Selected + "|" + Evaluated + "|" + Previewed + "|" + Better + "|" + Applied +
                "/progressGoalMinMm(base-candidate-nativeBase-nativeCandidate)=" + (Evaluated ?
                    N(GoalBaselineMin * 1000f) + "|" + N(GoalCandidateMin * 1000f) + "|" +
                    N(NativeGoalBaselineMin * 1000f) + "|" + N(NativeGoalCandidateMin * 1000f) : "NA|NA|NA|NA") +
                "/progressStepMinMm(base-candidate-nativeBase-nativeCandidate)=" + (Previewed ?
                    N(StepBaselineMin * 1000f) + "|" + N(StepCandidateMin * 1000f) + "|" +
                    N(NativeStepBaselineMin * 1000f) + "|" + N(NativeStepCandidateMin * 1000f) : "NA|NA|NA|NA") +
                "/progressIdealAtCandidateMm-kneeGapDeg=" + (Evaluated ? N(IdealAtCandidateMin * 1000f) + "|" + N(KneeGapDegrees) : "NA|NA") +
                "/progressDomain=COMPLETED_KNEE_REFERENCE_AND_SAME_RENDER_NATIVE_LOWER_NOT_SOLVED";
        }

        internal static Result Evaluate(KneeFootGroundPacket packet, FootGroundTrajectory.Input geometry,
            FootGroundReference.GroundSample baseline, out FootGroundReference.GroundSample candidate)
        {
            candidate = baseline;
            var r = new Result { Source = geometry.Source, Reason = Status.Unavailable };
            var input = packet.Input;
            if (!input.HasCurrentKneeReference || !packet.Sample.Valid || !baseline.Valid || !packet.Geometry.Valid ||
                !geometry.Matches(input.Source) || !packet.Sample.Source.Matches(input.Source) ||
                !packet.Sample.GeometryStep.Matches(input.CompletedStep) ||
                !geometry.GeometryStep.Matches(input.CompletedStep) || packet.SupportId != geometry.SupportId ||
                input.ParentId != geometry.ParentId || input.UpperId != geometry.UpperId || input.LowerId != geometry.LowerId)
                return r;
            var g = packet.Geometry; var knee = input.CompletedKneeReference.R;
            Vector3 ankle = KneeGroundTarget.Ankle(g, knee); Quaternion lower = g.Upper * knee;
            Quaternion baselineGoal = KneeGroundTarget.SelectedFoot(g, baseline);
            r.Reason = Status.InvalidGeometry;
            if (!FootBoxGeometry.Inside(ankle, lower * baselineGoal, g.Center, g.Half, g.FloorCenter, g.FloorHalf) ||
                !FootBoxGeometry.Inside(geometry.NativeAnkle, geometry.NativeLower * baselineGoal, g.Center, g.Half, g.FloorCenter, g.FloorHalf)) return r;
            var next = FootGroundReference.Evaluate(ankle, lower, g.FootRelative, g.FootBindRelative,
                g.Center, g.Half, g.FloorCenter, g.FloorHalf, true);
            if (!next.Valid) return r;
            Quaternion nextGoal = KneeGroundTarget.SelectedFoot(g, next);
            r.GoalBaselineMin = Minimum(g, ankle, lower, baselineGoal);
            r.GoalCandidateMin = Minimum(g, ankle, lower, nextGoal);
            r.NativeGoalBaselineMin = Minimum(g, geometry.NativeAnkle, geometry.NativeLower, baselineGoal);
            r.NativeGoalCandidateMin = Minimum(g, geometry.NativeAnkle, geometry.NativeLower, nextGoal);
            r.IdealAtCandidateMin = KneeGroundTarget.Minimum(g, packet.Sample.Goal, nextGoal);
            r.KneeGapDegrees = FootRelativeReference.Log(knee * Quaternion.Inverse(packet.Sample.Goal)).magnitude * Mathf.Rad2Deg;
            if (!F(r.GoalBaselineMin) || !F(r.GoalCandidateMin) || !F(r.NativeGoalBaselineMin) ||
                !F(r.NativeGoalCandidateMin) || !F(r.IdealAtCandidateMin) || !F(r.KneeGapDegrees)) return r;
            r.Evaluated = true;
            if (r.GoalBaselineMin >= -KneeGroundTarget.MinGain) { r.Reason = Status.BaselineClear; return r; }
            if (r.GoalCandidateMin - r.GoalBaselineMin < KneeGroundTarget.MinGain ||
                !Legal(g, ankle, lower, nextGoal) || !Legal(g, geometry.NativeAnkle, geometry.NativeLower, nextGoal))
            { r.Reason = Status.NoGoalGain; return r; }
            if (r.NativeGoalCandidateMin < r.NativeGoalBaselineMin)
            { r.Reason = Status.NativeGoalRegression; return r; }
            next.JointChain = next.CurrentKneeBasis = true;
            candidate = next; r.Reason = Status.PreviewUnavailable;
            return r;
        }

        internal static void AssessPreview(ref Result r, KneeFootGroundPacket packet, FootGroundTrajectory.Input geometry,
            FootRelativeReference.Sample baseline, FootRelativeReference.Sample candidate)
        {
            if (r.Reason != Status.PreviewUnavailable || !baseline.Source.Matches(packet.Input.Source) ||
                !candidate.Source.Matches(packet.Input.Source) || !packet.Input.HasCurrentKneeReference) return;
            var g = packet.Geometry; var knee = packet.Input.CompletedKneeReference.R;
            Vector3 ankle = KneeGroundTarget.Ankle(g, knee); Quaternion lower = g.Upper * knee;
            r.StepBaselineMin = Minimum(g, ankle, lower, baseline.Rotation);
            r.StepCandidateMin = Minimum(g, ankle, lower, candidate.Rotation);
            r.NativeStepBaselineMin = Minimum(g, geometry.NativeAnkle, geometry.NativeLower, baseline.Rotation);
            r.NativeStepCandidateMin = Minimum(g, geometry.NativeAnkle, geometry.NativeLower, candidate.Rotation);
            if (!F(r.StepBaselineMin) || !F(r.StepCandidateMin) || !F(r.NativeStepBaselineMin) || !F(r.NativeStepCandidateMin)) return;
            r.Previewed = true;
            // Compare final bounded steps, including the common v134 ground choice.
            // New goals do not buy an unchecked jump or a native-Lower regression.
            if (r.StepCandidateMin - r.StepBaselineMin < KneeGroundTarget.MinGain ||
                !Legal(g, ankle, lower, candidate.Rotation) ||
                !Legal(g, geometry.NativeAnkle, geometry.NativeLower, candidate.Rotation))
            { r.Reason = Status.NoStepGain; return; }
            if (r.NativeStepCandidateMin < r.NativeStepBaselineMin)
            { r.Reason = Status.NativeStepRegression; return; }
            r.Better = true; r.Reason = Status.Improved;
        }
        private static bool Legal(KneeGroundTarget.Geometry g, Vector3 ankle, Quaternion lower, Quaternion foot) =>
            FootGroundReference.GoalWithinBounds(lower, g.FootRelative, foot, g.FootBindRelative) &&
            FootBoxGeometry.Inside(ankle, lower * foot, g.Center, g.Half, g.FloorCenter, g.FloorHalf);
        private static float Minimum(KneeGroundTarget.Geometry g, Vector3 ankle, Quaternion lower, Quaternion foot) =>
            FootBoxGeometry.Minimum(ankle, lower * foot, g.Center, g.Half) - g.FloorCenter.y - g.FloorHalf.y;
        private static bool F(float value) => PelvisParentFrame.F(value);
        private static string N(float value) => FootGroundTrajectory.N(value);
    }
}
