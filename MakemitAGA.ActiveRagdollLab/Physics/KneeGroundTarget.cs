using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One Render-snapshot search for a bounded Knee/Foot INPUT pair. Each existing
    // reference still owns its own continuous R/W/A; this is not a pose or contact command.
    internal static class KneeGroundTarget
    {
        internal const float MaxCorrectionDegrees = 5f;
        internal const float MaxBendDegrees = 150f; // Conservative experiment search bound, not a human limit.
        internal const float MinExtensionBendDegrees = 5f; // Search margin from straight, not an anatomical claim.
        internal const float MinGain = .0001f;
        private const int SearchSamples = 16;

        internal struct Geometry
        {
            internal Vector3 Hip, UpperSpan, LowerSpan, Center, Half, FloorCenter, FloorHalf;
            internal Quaternion Upper, FootRelative, FootBindRelative;
            internal bool Valid => V(Hip) && V(UpperSpan) && V(LowerSpan) &&
                UpperSpan.sqrMagnitude > .000001f && LowerSpan.sqrMagnitude > .000001f &&
                Q(Upper) && Q(FootRelative) && Q(FootBindRelative) && V(Center) && Positive(Half) && V(FloorCenter) && Positive(FloorHalf);
        }
        internal struct Sample
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal bool Valid, Candidate, Applied, Planned;
            internal bool AllowExtension;
            internal bool Extension => Candidate && GoalBend < RawBend;
            internal string Reason;
            internal Quaternion Raw, Goal;
            internal FootGroundReference.GroundSample Foot;
            internal Quaternion PlannedFoot;
            internal FootGroundTrajectory.Result FootTrajectory;
            internal FootKneeProgressTarget.Result KneeProgress;
            internal PhysicsStepContext GeometryStep;
            internal float FootOnlyMin;
            internal float RawMin, GoalMin, PlanMin, RawAnkleY, GoalAnkleY, Correction, RawBend, GoalBend;
            // Optional evidence. Never read by candidate selection or packet validity.
            internal KneeGroundFeasibility.Envelope Feasibility;
            internal KneeGroundFeasibility.Stage FeasibilityStage;
            internal float GoalWithPublishedFootMin;
            internal bool Matches(PhysicsStepContext step, SourceStepContext source) =>
                Step.Matches(step) && Source.Matches(source) && source.Valid &&
                source.Epoch == step.Epoch && source.Sequence == step.SourceSequence;
        }
        internal static Vector3 Ankle(Geometry g, Quaternion knee) =>
            g.Hip + g.Upper * g.UpperSpan + (g.Upper * knee) * g.LowerSpan;
        internal static float Minimum(Geometry g, Quaternion knee) =>
            Minimum(g, knee, g.FootRelative);
        internal static float Minimum(Geometry g, Quaternion knee, Quaternion foot) =>
            FootBoxGeometry.Minimum(Ankle(g, knee), g.Upper * knee * foot, g.Center, g.Half) -
            (g.FloorCenter.y + g.FloorHalf.y);
        private static bool Inside(Geometry g, Quaternion knee) =>
            FootBoxGeometry.Inside(Ankle(g, knee), g.Upper * knee * g.FootRelative,
                g.Center, g.Half, g.FloorCenter, g.FloorHalf);
        private static float Bend(Geometry g, Quaternion knee) => Mathf.Acos(Mathf.Clamp(
            Vector3.Dot(g.UpperSpan.normalized, (knee * g.LowerSpan).normalized), -1f, 1f)) * Mathf.Rad2Deg;

        // Derivative of cos(segment bend) along the signed calibrated-axis arc.
        // Checking both ends of an arc shorter than pi keeps the same bending branch;
        // endpoint bend alone could accept crossing a near-straight knee to the other side.
        private static float BendSlope(Geometry g, Quaternion knee, Vector3 axis) =>
            Vector3.Dot(g.UpperSpan.normalized, knee * Vector3.Cross(axis, g.LowerSpan.normalized));

        internal static Sample Evaluate(Geometry g, Quaternion raw, Vector3 lowerLocalAxis, bool allowExtension = false)
        {
            Sample s = new Sample { Raw = raw, Goal = raw, AllowExtension = allowExtension, Reason = "GEOMETRY_UNAVAILABLE" };
            if (!g.Valid || !Q(raw) || !V(lowerLocalAxis) || lowerLocalAxis.sqrMagnitude < .0001f) return s;
            s.RawMin = s.GoalMin = Minimum(g, raw);
            s.RawAnkleY = s.GoalAnkleY = Ankle(g, raw).y - (g.FloorCenter.y + g.FloorHalf.y);
            s.RawBend = s.GoalBend = Bend(g, raw);
            if (!F(s.RawMin) || !F(s.RawAnkleY) || !F(s.RawBend)) return s;
            s.Valid = true;
            if (!Inside(g, raw)) { s.Reason = "FOOTPRINT_OUTSIDE"; return s; }
            s.Foot = EvaluateFoot(g, raw);
            if (!s.Foot.Valid) { s.Valid = false; s.Reason = "FOOT_INPUT_INVALID"; return s; }
            s.GoalMin = s.FootOnlyMin = SelectedMinimum(s.Foot);
            // Give the existing bounded Foot correction the first opportunity. A Knee
            // change must improve on that same-state Foot-only candidate, not on raw.
            if (s.GoalMin >= -MinGain) { s.Reason = "FOOT_ONLY_CLEAR_OR_SUB_NOISE"; return s; }
            if (s.RawBend >= MaxBendDegrees) { s.Reason = "BEND_SEARCH_BOUND"; return s; }
            Vector3 axis = lowerLocalAxis.normalized;
            float rawSlope = BendSlope(g, raw, axis);
            // Axis sign is calibration-dependent. A retains added bend only. B may
            // also reduce bend on the SAME monotone branch, leaving a straight-knee
            // margin. The original 5-degree arc and independent Foot limits still apply.
            for (int i = 1; i <= SearchSamples; i++)
            {
                for (int sign = -1; sign <= 1; sign += 2)
                {
                    float angle = sign * MaxCorrectionDegrees * i / SearchSamples;
                    Quaternion q = raw * Quaternion.AngleAxis(angle, axis);
                    // Ambiguous straight/axis-degenerate input is left unchanged.
                    float endSlope = sign * BendSlope(g, q, axis);
                    bool flexion = sign * rawSlope < -0.000001f && endSlope < -0.000001f;
                    bool extension = allowExtension && sign * rawSlope > 0.000001f && endSlope > 0.000001f;
                    if (!flexion && !extension) continue;
                    float bend = Bend(g, q);
                    float ankleY = Ankle(g, q).y - (g.FloorCenter.y + g.FloorHalf.y);
                    if (!Q(q) || !F(bend) || !F(ankleY) || bend > MaxBendDegrees ||
                        (flexion ? bend <= s.RawBend : bend >= s.RawBend || bend < MinExtensionBendDegrees) ||
                        ankleY - s.RawAnkleY < MinGain) continue;
                    FootGroundReference.GroundSample foot = EvaluateFoot(g, q);
                    Quaternion footGoal = SelectedFoot(g, foot);
                    float min = Minimum(g, q, footGoal);
                    if (!foot.Valid || !F(min) || min <= s.GoalMin || min - s.FootOnlyMin < MinGain ||
                        !FootGroundReference.GoalWithinBounds(g.Upper * q, g.FootRelative, footGoal, g.FootBindRelative) ||
                        !FootBoxGeometry.Inside(Ankle(g, q), g.Upper * q * footGoal,
                            g.Center, g.Half, g.FloorCenter, g.FloorHalf)) continue;
                    s.Candidate = true; s.Goal = q; s.Correction = angle; s.GoalMin = min;
                    s.GoalAnkleY = ankleY; s.GoalBend = bend; s.Foot = foot;
                }
                // Least sampled correction that clears; otherwise retain the best
                // legal partial improvement. No global feasibility claim is made.
                if (s.Candidate && s.GoalMin >= 0f) break;
            }
            s.Reason = !s.Candidate ? "NO_JOINT_IMPROVEMENT_OVER_FOOT_ONLY" :
                (s.GoalMin >= 0f ? "JOINT_CLEAR_CANDIDATE" : "JOINT_PARTIAL_CANDIDATE");
            return s;
        }

        private static FootGroundReference.GroundSample EvaluateFoot(Geometry g, Quaternion knee) =>
            FootGroundReference.Evaluate(Ankle(g, knee), g.Upper * knee, g.FootRelative, g.FootBindRelative,
                g.Center, g.Half, g.FloorCenter, g.FloorHalf, true);
        internal static Quaternion SelectedFoot(Geometry g, FootGroundReference.GroundSample foot) =>
            foot.Valid && (foot.Feasible || foot.Partial) ? foot.Goal : g.FootRelative;
        private static float SelectedMinimum(FootGroundReference.GroundSample foot) =>
            foot.Feasible || foot.Partial ? foot.GoalMin : foot.RawMin;

        internal static string Text(Sample s) => "kneeFootGroundV041260=" + (s.Reason ?? "NOT_CAPTURED") +
            "/geometrySource(epoch-source-observedFixed)=" + s.Source.Epoch + "|" + s.Source.Sequence + "|" + s.GeometryStep.Sequence +
            "/valid-candidate-applied-planned=" + s.Valid + "|" + s.Candidate + "|" + s.Applied + "|" + s.Planned +
            "/soleMinMm(rawChain-candidate-planned)=" + (s.Valid ? N(s.RawMin * 1000f) + "|" + N(s.GoalMin * 1000f) : "NA|NA") + "|" + (s.Planned ? N(s.PlanMin * 1000f) : "NA") +
            "/ankleHeightMm(raw-candidate)=" + (s.Valid ? N(s.RawAnkleY * 1000f) + "|" + N(s.GoalAnkleY * 1000f) : "NA") +
            "/footOnlyMinMm=" + (s.Valid ? N(s.FootOnlyMin * 1000f) : "NA") +
            "/footCandidate(correctionDeg-feasible-partial)=" + N(s.Foot.CorrectionDegrees) + "|" + s.Foot.Feasible + "|" + s.Foot.Partial +
            "/correction-bendDeg=" + N(s.Correction) + "|" + N(s.RawBend) + "|" + N(s.GoalBend) +
            "/directionPolicy=" + (s.AllowExtension ? "BOUNDED_EXTENSION_ALLOWED" : "FLEXION_ONLY") +
            "/extensionCandidate=" + s.Extension +
            "/rawKneeR=" + N(s.Raw.x) + "," + N(s.Raw.y) + "," + N(s.Raw.z) + "," + N(s.Raw.w) +
            "/domain=ONE_RENDER_NATIVE_HIP_UPPER_RAW_KNEE_FOOT_PAIR_NOT_SOLVED_CONTACT/" + KneeGroundFeasibility.SampleText(s) + "/" + s.FootTrajectory.Text() + "/" + s.KneeProgress.Text();
        private static string N(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        private static bool Positive(Vector3 v) => V(v) && v.x > 0f && v.y > 0f && v.z > 0f;
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < .01f;
    }
}
