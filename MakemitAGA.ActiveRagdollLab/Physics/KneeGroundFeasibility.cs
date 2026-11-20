using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Optional evidence, never an input, acceptance test or actuator. Render computes
    // one relaxed height ceiling; Fixed only compares the already published rotations.
    internal static class KneeGroundFeasibility
    {
        internal enum BoundStatus { Unavailable, UnsupportedBox, NonUnitRotation, Available }
        internal enum Stage
        {
            Unavailable, GoalBelowHeightCeiling, GoalBelowSearchUnresolved, GoalBelowBoundUnavailable,
            GoalClearFootReferenceBelow, GoalClearKneeReferenceBelow, GoalAndReferencesClear, GoalWithinNoise
        }
        internal struct Envelope
        {
            internal BoundStatus Status;
            internal float CeilingMin, MaxAnkleY, AtDegrees, RoundoffGuard;
            internal bool Valid => Status == BoundStatus.Available && PelvisParentFrame.F(CeilingMin) &&
                PelvisParentFrame.F(MaxAnkleY) && PelvisParentFrame.F(AtDegrees) &&
                PelvisParentFrame.F(RoundoffGuard) && RoundoffGuard >= 0f;
            internal bool HeightDeficit => Valid && CeilingMin < -KneeGroundTarget.MinGain;
        }

        internal static Envelope AtRender(KneeGroundTarget.Geometry g, Quaternion raw, Vector3 axis)
        {
            if (!g.Valid || !PelvisParentFrame.Q(raw) || !PelvisParentFrame.V(axis) || axis.sqrMagnitude < .0001f)
                return default;
            // The analytic arc uses rotations. Wider controller finite tolerances do
            // not license a geometric proof with materially non-unit quaternions.
            if (!Unit(g.Upper) || !Unit(raw) || !Unit(g.FootRelative))
                return new Envelope { Status = BoundStatus.NonUnitRotation };
            Vector3 u = axis.normalized, parallel = u * Vector3.Dot(u, g.LowerSpan);
            Quaternion basis = g.Upper * raw;
            double a = (basis * (g.LowerSpan - parallel)).y;
            double b = (basis * Vector3.Cross(u, g.LowerSpan)).y;
            double c = (double)g.Hip.y - g.FloorCenter.y - g.FloorHalf.y +
                (g.Upper * g.UpperSpan).y + (basis * parallel).y;
            double limit = KneeGroundTarget.MaxCorrectionDegrees * Math.PI / 180.0;
            double angle = -limit, best = a * Math.Cos(angle) + b * Math.Sin(angle);
            double end = a * Math.Cos(limit) + b * Math.Sin(limit);
            if (end > best) { best = end; angle = limit; }
            double stationary = Math.Atan2(b, a);
            if (stationary >= -limit && stationary <= limit)
            {
                double interior = a * Math.Cos(stationary) + b * Math.Sin(stationary);
                if (interior > best) { best = interior; angle = stationary; }
            }
            float ankleY = (float)(c + best);
            if (!FootGroundReference.UprightBound(new Vector3(0f, ankleY, 0f), g.Center, g.Half, 0f, out float ceiling))
                return new Envelope { Status = BoundStatus.UnsupportedBox };
            float radius = g.Center.magnitude + g.Half.magnitude;
            float scale = 1f + Mathf.Abs(g.Hip.y) + Mathf.Abs(g.FloorCenter.y) + Mathf.Abs(g.FloorHalf.y) +
                g.UpperSpan.magnitude + g.LowerSpan.magnitude + radius;
            // Upward guard covers float arithmetic and the accepted near-unit input
            // distortion. A sub-noise negative ceiling is deliberately inconclusive.
            float guard = .000008f * scale + 2f * Mathf.Abs(Norm(g.Upper) - 1f) * g.UpperSpan.magnitude +
                2f * Mathf.Abs(Norm(g.Upper) * Norm(raw) - 1f) * g.LowerSpan.magnitude +
                2f * (Mathf.Abs(Norm(g.Upper) * Norm(raw) * Norm(g.FootRelative) - 1f) + .00001f) * radius;
            var result = new Envelope { Status = BoundStatus.Available, CeilingMin = ceiling + guard,
                MaxAnkleY = ankleY, AtDegrees = (float)(angle * 180.0 / Math.PI), RoundoffGuard = guard };
            return result.Valid ? result : default;
        }

        // Relaxation: BOTH signed Knee arcs within 5 degrees, all yaw/up directions
        // inside the Foot 30-degree upright cone. Ignore Knee bend branch/150-degree
        // cap, Foot 15-degree correction/ROM and footprint. Negative rules out this
        // larger set at the SAVED Hip/Upper geometry; nonnegative does NOT establish a legal pose.
        internal static void AssessFixed(ref KneeGroundTarget.Sample s, KneeGroundTarget.Geometry g)
        {
            s.FeasibilityStage = Stage.Unavailable;
            if (!s.Valid || !s.Planned || !s.Foot.Valid || !g.Valid || !PelvisParentFrame.Q(s.Goal) ||
                !PelvisParentFrame.Q(s.PlannedFoot) || !PelvisParentFrame.F(s.GoalMin) || !PelvisParentFrame.F(s.PlanMin)) return;
            s.GoalWithPublishedFootMin = KneeGroundTarget.Minimum(g, s.Goal, s.PlannedFoot);
            if (!PelvisParentFrame.F(s.GoalWithPublishedFootMin)) return;
            if (s.GoalMin < 0f && s.GoalMin >= -KneeGroundTarget.MinGain)
                s.FeasibilityStage = Stage.GoalWithinNoise; // Input noise allowance, never contact clearance.
            else if (s.GoalMin < 0f)
                s.FeasibilityStage = !s.Feasibility.Valid ? Stage.GoalBelowBoundUnavailable :
                    (s.Feasibility.HeightDeficit ? Stage.GoalBelowHeightCeiling : Stage.GoalBelowSearchUnresolved);
            else s.FeasibilityStage = s.GoalWithPublishedFootMin < 0f ? Stage.GoalClearFootReferenceBelow :
                (s.PlanMin < 0f ? Stage.GoalClearKneeReferenceBelow : Stage.GoalAndReferencesClear);
        }

        internal static string SampleText(KneeGroundTarget.Sample s) => "jointFeasibilityV041320=" + s.FeasibilityStage +
            "/bound=" + s.Feasibility.Status + "/ceilingMm=" + (s.Feasibility.Valid ? N(s.Feasibility.CeilingMin * 1000f) : "NA") +
            "/maxAnkleMm-atKneeDeg-guardMm=" + (s.Feasibility.Valid ? N(s.Feasibility.MaxAnkleY * 1000f) + "|" +
                N(s.Feasibility.AtDegrees) + "|" + N(s.Feasibility.RoundoffGuard * 1000f) : "NA") +
            "/goalWithPublishedFootMinMm=" + (s.FeasibilityStage != Stage.Unavailable ? N(s.GoalWithPublishedFootMin * 1000f) : "NA") +
            "/domain=SAVED_RENDER_PARENT_GEOMETRY/ceilingNonnegative=UNRESOLVED_NOT_FEASIBLE";

        internal static string ActualComparison(KneeGroundTarget.Sample s, PhysicsStepContext step,
            SourceStepContext source, bool actualValid, float actualMin)
        {
            if (!s.Matches(step, source) || !s.Valid || !s.Planned || s.FeasibilityStage == Stage.Unavailable ||
                !actualValid || !PelvisParentFrame.F(s.PlanMin) || !PelvisParentFrame.F(actualMin)) return "jointPlanActualV041320=UNAVAILABLE";
            return "jointPlanActualV041320=" + (s.PlanMin >= 0f ? "PLAN_CLEAR" : "PLAN_BELOW") +
                (actualMin >= 0f ? "_ACTUAL_CLEAR" : "_ACTUAL_BELOW") +
                "/domain=SAVED_RENDER_PARENT_VS_PRE_FIXED_BODY_NOT_RESPONSE_PROOF";
        }

        // Owned/reset by the existing Knee trace. No new sampling loop or ring.
        internal sealed class Trace
        {
            private readonly int[] _fixedStages = new int[8];
            private SourceStepContext _lastSource;
            private int _steps, _valid, _held, _sources, _goalBelow, _heightDeficit, _unresolved, _boundUnavailable, _subNoise, _faults;
            private float _peakDeficit;
            private bool _hasWitness;
            private KneeFootGroundPacket _witness;
            private Quaternion _witnessKnee;
            internal void Reset()
            {
                Array.Clear(_fixedStages, 0, _fixedStages.Length); _lastSource = default;
                _steps = _valid = _held = _sources = _goalBelow = _heightDeficit = _unresolved = _boundUnavailable = _subNoise = _faults = 0;
                _peakDeficit = 0f; _hasWitness = false; _witness = default; _witnessKnee = default;
            }
            internal void Fault() { _faults++; }
            internal void Observe(KneeGroundTarget.Sample s, KneeFootGroundPacket packet, KneeReferenceState reference)
            {
                _steps++;
                if (!s.Matches(reference.Step, reference.Source) || s.FeasibilityStage == Stage.Unavailable)
                { _fixedStages[0]++; return; }
                _fixedStages[(int)s.FeasibilityStage]++; _valid++;
                if (s.Source.Matches(_lastSource)) { _held++; return; }
                _lastSource = s.Source; _sources++;
                if (s.GoalMin >= 0f) return;
                _goalBelow++;
                if (s.GoalMin >= -KneeGroundTarget.MinGain) { _subNoise++; return; }
                if (s.Feasibility.HeightDeficit)
                {
                    _heightDeficit++;
                    _peakDeficit = Mathf.Max(_peakDeficit, -s.Feasibility.CeilingMin);
                }
                else
                {
                    if (s.Feasibility.Valid) _unresolved++; else _boundUnavailable++;
                    // One full reproducible geometry, only for the deepest unresolved
                    // selected goal. It is not a completed solve or a representative run.
                    if (!_hasWitness || s.GoalMin < _witness.Sample.GoalMin)
                    { _hasWitness = true; _witness = packet; _witness.Sample = s; _witnessKnee = reference.R; }
                }
            }
            internal string Summary() => "KNEE_GROUND_FEASIBILITY_V041340/steps-valid-held-sources=" +
                _steps + "|" + _valid + "|" + _held + "|" + _sources +
                "/fixedStages(unavailable-heightDeficit-searchUnresolved-boundUnavailable-footReferenceBelow-kneeReferenceBelow-clear-subNoise)=" +
                string.Join("|", _fixedStages) + "/sourceGoalBelow(total-heightDeficit-searchUnresolved-boundUnavailable-subNoise)=" +
                _goalBelow + "|" + _heightDeficit + "|" + _unresolved + "|" + _boundUnavailable + "|" + _subNoise +
                "/provenDeficitPeakMm=" + N(_peakDeficit * 1000f) + "/diagnosticFaults=" + _faults +
                "/subNoiseMm=0.1_NOT_CONTACT_TOLERANCE/sources=DISTINCT_PACKETS_FIRST_FIXED_CONSUMPTION/fixed=INCLUDING_HELD" +
                "/bound=RELAXED_KNEE_PLUS_MINUS5_ALL_UPRIGHT_FOOT30_NO_ROM_OR_FOOTPRINT/authority=NONE" + WitnessText();
            private string WitnessText()
            {
                if (!_hasWitness) return "/unresolvedWitness=NONE";
                var p = _witness; var s = p.Sample; var g = p.Geometry;
                return "\nJOINT_FEASIBILITY_WITNESS_V041320/selection=DEEPEST_UNRESOLVED_SELECTED_GOAL" +
                    "/epoch-fixed-source-geometryFixed-support=" + s.Step.Epoch + "|" + s.Step.Sequence + "|" +
                    s.Source.Sequence + "|" + s.GeometryStep.Sequence + "|" + p.SupportId + "/" + SampleText(s) +
                    "/directionPolicy=" + (s.AllowExtension ? "BOUNDED_EXTENSION_ALLOWED" : "FLEXION_ONLY") +
                    "/extensionCandidate=" + s.Extension +
                    "/soleMinMm(goal-publishedFoot-finalPlan)=" + N(s.GoalMin * 1000f) + "|" +
                    N(s.GoalWithPublishedFootMin * 1000f) + "|" + N(s.PlanMin * 1000f) +
                    "/hip=" + Vec(g.Hip) + "/upper=" + Quat(g.Upper) + "/upperSpan=" + Vec(g.UpperSpan) +
                    "/lowerSpan=" + Vec(g.LowerSpan) + "/boxCenter-half=" + Vec(g.Center) + "|" + Vec(g.Half) +
                    "/floorCenter-half=" + Vec(g.FloorCenter) + "|" + Vec(g.FloorHalf) + "/axis=" + Vec(p.Input.Axis) +
                    "/kneeR(raw-goal-planned)=" + Quat(s.Raw) + "|" + Quat(s.Goal) + "|" + Quat(_witnessKnee) +
                    "/footR(raw-bind-goal-planned)=" + Quat(g.FootRelative) + "|" + Quat(g.FootBindRelative) + "|" +
                    Quat(KneeGroundTarget.SelectedFoot(g, s.Foot)) + "|" + Quat(s.PlannedFoot) + "/vectors=METERS_LOCAL_WHERE_NAMED";
            }
        }
        private static float Norm(Quaternion q) => q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        private static bool Unit(Quaternion q) => Mathf.Abs(Norm(q) - 1f) <= .00001f;
        private static string N(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static string Vec(Vector3 v) => N(v.x) + "," + N(v.y) + "," + N(v.z);
        private static string Quat(Quaternion q) => N(q.x) + "," + N(q.y) + "," + N(q.z) + "," + N(q.w);
    }
}
