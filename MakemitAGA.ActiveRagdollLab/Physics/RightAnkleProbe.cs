using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using static MakemitAGA.ActiveRagdollLab.Physics.LandingPreparationMath;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One PRE_N state, two virtual lower-leg commands. The native Knee port uses
    // ForceMode.Acceleration. The pivot, ankle gap and foot-to-lower rotation are held
    // for this one-step comparison; contact, drag and tether response belong to PhysX.
    internal static class RightAnkleProbe
    {
        internal enum Status { Missing, ProtectedHip, StaleKnee, GroundKnee, LandingKnee, InvalidReference, Geometry, BaselineMismatch, NoRoom, NoLegal, BelowGain, Improved, Fault }
        internal struct Input
        {
            internal KneeGroundTarget.Geometry Geometry;
            internal Quaternion Upper, Lower, RawKnee, FootRelative, SelectedFoot;
            internal Vector3 Knee, Gap, LowerW, UpperW, Axis, HipTargetW;
            internal float ObservedBox, ObservedLowerY, SourceAnkleY, BodyMaxW;
            internal float Kp, Kd, SoftKp, SoftKd, SoftLimit, FlexLimit, FfGain, FfLimit;
            internal bool FfActive, FeedbackPriority, AllowExtension;
            internal float SubmittedPd, SubmittedFf, AllocatedFf, SubmittedFlex;
            internal Vector3 RelativeCommand, TotalCommand;
        }
        internal struct Heights { internal float Ankle, Box; }
        internal struct Result
        {
            internal Status State;
            internal int Lanes, Legal, Index, Sign;
            internal bool BaseMeasured, CandidateMeasured, CandidateClamped, PivotForecast;
            internal float PredictedKneeShift;
            internal float BasePd, BaseFf, BaseFlex, TrialPd, TrialFf, TrialFlex;
            internal float RefW, RefA, MaxW, MaxA, FlexLimit;
            internal float ReferenceBoxGain, ReferenceAnkleGain, ReferenceCoastBoxGain, TargetAnkle, BaseBodyW, TrialBodyW, BodyMaxW;
            internal Heights Baseline, Candidate;
        }
        private struct Command
        {
            internal float Pd, SelectedFf, AllocatedFf, Flex;
            internal bool Clamped;
            internal Vector3 Relative;
        }

        private static bool Unit(Quaternion q) => PelvisParentFrame.Q(q) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < .001f;
        private static bool Nonnegative(float v) => PelvisParentFrame.F(v) && v >= 0f;
        private static bool Near(float a, float b) => PelvisParentFrame.F(a) && PelvisParentFrame.F(b) && Mathf.Abs(a - b) <= .0002f * (1f + Mathf.Abs(b));
        private static bool Near(Vector3 a, Vector3 b) => PelvisParentFrame.V(a) && PelvisParentFrame.V(b) && (a - b).magnitude <= .0002f * (1f + b.magnitude);

        private static Command Compose(in Input p, Vector3 axis, Quaternion r, Vector3 w, Vector3 a)
        {
            var relative = Quaternion.Inverse(p.Upper) * p.Lower;
            var error = p.Upper * FootRelativeReference.Log(r * Quaternion.Inverse(relative));
            var velocityError = p.Upper * w - (p.LowerW - p.UpperW);
            float angle = Vector3.Dot(error, axis), speed = Vector3.Dot(velocityError, axis);
            var c = new Command { Pd = angle * p.Kp + speed * p.Kd };
            c.SelectedFf = p.FfActive ? Mathf.Clamp(Vector3.Dot(p.Upper * a, axis) * p.FfGain, -p.FfLimit, p.FfLimit) : 0f;
            c.Flex = KneeFlexionFeedbackHeadroomAllocationIsolation.Allocate(c.Pd, c.SelectedFf, p.FlexLimit,
                p.FeedbackPriority, out c.AllocatedFf, out float raw, out _);
            c.Clamped = Mathf.Abs(raw - c.Flex) > .0001f;
            var soft = (error - axis * angle) * p.SoftKp + (velocityError - axis * speed) * p.SoftKd;
            c.Relative = axis * c.Flex + Vector3.ClampMagnitude(soft, p.SoftLimit);
            return c;
        }

        private static bool Measure(in Input p, Quaternion lower, out Heights h)
        {
            var g = p.Geometry;
            Vector3 lowerAnkle = p.Knee + lower * g.LowerSpan;
            Vector3 footAnkle = lowerAnkle + p.Gap;
            Quaternion foot = lower * p.FootRelative;
            h = new Heights
            {
                Ankle = lowerAnkle.y,
                Box = FootBoxGeometry.Minimum(footAnkle, foot, g.Center, g.Half) -
                (g.FloorCenter.y + g.FloorHalf.y)
            };
            return PelvisParentFrame.F(h.Ankle) && PelvisParentFrame.F(h.Box) &&
                FootBoxGeometry.Inside(footAnkle, foot, g.Center, g.Half, g.FloorCenter, g.FloorHalf);
        }

        private static bool Project(in Input p, Vector3 total, float dt, out Heights h, out float speed)
        {
            Vector3 nextW = Vector3.ClampMagnitude(p.LowerW + total * dt, p.BodyMaxW);
            speed = nextW.magnitude;
            return Measure(p, FootRelativeReference.Advance(p.Lower, nextW, dt), out h);
        }

        internal static Result Compare(in Input p, in KneeReferenceState k, PhysicsStepContext step, SourceStepContext source)
        {
            var r = new Result { State = Status.Missing };
            if (!k.Matches(step, source) || !source.Valid || source.Epoch != step.Epoch || source.Sequence != step.SourceSequence) return r;
            if (k.Stale) { r.State = Status.StaleKnee; return r; }
            if (k.GroundTrajectory.Applied) { r.State = Status.GroundKnee; return r; }
            if (k.LandingPreparation.Applied) { r.State = Status.LandingKnee; return r; }
            r.State = Status.InvalidReference;
            if (!ValidLane(k.PreviousR, k.PreviousW, k.InputR, k.InputW, k.R, k.W, k.A, step.Dt, k.MaxW, k.MaxA) ||
                !Nonnegative(p.Kp) || !Nonnegative(p.Kd) || !Nonnegative(p.SoftKp) || !Nonnegative(p.SoftKd) ||
                !Nonnegative(p.SoftLimit) || !Nonnegative(p.FfGain) || !Nonnegative(p.FfLimit) ||
                !Nonnegative(p.FlexLimit) || p.FlexLimit <= 0f || !Nonnegative(p.BodyMaxW) || p.BodyMaxW <= 0f ||
                !PelvisParentFrame.V(p.LowerW) || !PelvisParentFrame.V(p.UpperW) || !PelvisParentFrame.V(p.RelativeCommand) ||
                !PelvisParentFrame.V(p.TotalCommand) || !PelvisParentFrame.V(p.HipTargetW)) return r;
            r.State = Status.Geometry;
            var g = p.Geometry;
            if (!g.Valid || !Unit(g.Upper) || !Unit(p.RawKnee) || !Unit(p.Upper) || !Unit(p.Lower) || !Unit(p.FootRelative) || !Unit(p.SelectedFoot) ||
                !PelvisParentFrame.V(p.Axis) || p.Axis.sqrMagnitude < .0001f || !PelvisParentFrame.V(p.Gap) ||
                !PelvisParentFrame.V(p.Knee) || !PelvisParentFrame.F(p.ObservedBox) || !PelvisParentFrame.F(p.ObservedLowerY) ||
                !PelvisParentFrame.F(p.SourceAnkleY) || !Measure(p, p.Lower, out var measured) ||
                Mathf.Abs(measured.Box - p.ObservedBox) > FootBoxGeometry.SupportPlaneToleranceMeters ||
                Mathf.Abs(measured.Ankle - p.ObservedLowerY) > FootBoxGeometry.SupportPlaneToleranceMeters) return r;
            var axis = (p.Lower * p.Axis).normalized;
            var original = Compose(p, axis, k.R, k.W, k.A);
            r.State = Status.BaselineMismatch;
            if (!Near(original.Pd, p.SubmittedPd) || !Near(original.SelectedFf, p.SubmittedFf) ||
                !Near(original.AllocatedFf, p.AllocatedFf) || !Near(original.Flex, p.SubmittedFlex) ||
                !Near(original.Relative, p.RelativeCommand)) return r;
            r.State = Status.Geometry;
            var coastUpper = FootRelativeReference.Advance(g.Upper, p.HipTargetW, step.Dt);
            var coastKnee = FootRelativeReference.Advance(k.R, k.W, step.Dt);
            if (!LegalPose(g, p.SelectedFoot, g.Upper, k.R) || !LegalPose(g, p.SelectedFoot, coastUpper, coastKnee) || !Project(p, p.TotalCommand, step.Dt, out r.Baseline, out r.BaseBodyW)) return r;
            r.BaseMeasured = true; r.BasePd = original.Pd; r.BaseFf = original.AllocatedFf; r.BaseFlex = original.Flex;
            r.TargetAnkle = p.SourceAnkleY; r.MaxW = Mathf.Max(k.PreviousW.magnitude, k.MaxW); r.MaxA = k.MaxA;
            r.BodyMaxW = p.BodyMaxW; r.FlexLimit = p.FlexLimit;
            float refBox = Minimum(g, g.Upper, k.R, p.SelectedFoot), refAnkle = KneeGroundTarget.Ankle(g, k.R).y;
            float coastBox = Minimum(g, coastUpper, coastKnee, p.SelectedFoot);
            bool haveFavorable = false;
            float best = float.NegativeInfinity;
            for (int sign = -1; sign <= 1; sign += 2)
            {
                // Change only the Knee endpoint, within the existing five-degree input envelope.
                var goal = p.RawKnee * Quaternion.AngleAxis(sign * KneeGroundTarget.MaxCorrectionDegrees, p.Axis.normalized);
                var lane = MakeLane(k.PreviousR, k.PreviousW, goal, k.InputW, k.R, k.W, step.Dt, k.MaxW, k.MaxA, p.Axis, true);
                if (!PelvisParentFrame.F(lane.Room) || lane.Room <= .000001f) continue;
                r.Lanes++;
                for (int i = 1; i <= SamplesPerLane; i++)
                {
                    if (!TryLane(lane, g, i, out var rotation, out var w) || (!p.AllowExtension && Bend(g, rotation) < Bend(g, k.R)) ||
                        !LegalPose(g, p.SelectedFoot, g.Upper, rotation)) continue;
                    var coast = FootRelativeReference.Advance(rotation, w, step.Dt);
                    if (!LegalPose(g, p.SelectedFoot, coastUpper, coast)) continue;
                    Vector3 a = (w - k.PreviousW) / step.Dt;
                    var trial = Compose(p, axis, rotation, w, a);
                    // Preserve the actual same-step transported Hip term outside the Knee flexion cap.
                    if (!PelvisParentFrame.V(trial.Relative) ||
                        !Project(p, p.TotalCommand + (trial.Relative - original.Relative), step.Dt, out var height, out float bodyW)) continue;
                    r.Legal++;
                    float tracking = Mathf.Abs(r.Baseline.Ankle - p.SourceAnkleY) - Mathf.Abs(height.Ankle - p.SourceAnkleY);
                    float depth = Mathf.Max(0f, -r.Baseline.Box) - Mathf.Max(0f, -height.Box);
                    float score = Mathf.Min(tracking, depth);
                    float refBoxGain = Minimum(g, g.Upper, rotation, p.SelectedFoot) - refBox;
                    float refAnkleGain = KneeGroundTarget.Ankle(g, rotation).y - refAnkle;
                    float coastGain = Minimum(g, coastUpper, coast, p.SelectedFoot) - coastBox;
                    bool favorable = score >= KneeGroundTarget.MinGain && refBoxGain >= 0f && refAnkleGain >= 0f && coastGain >= 0f;
                    if (height.Box > FootContactGeometryLimits.RecoveryMaxGapMeters || (haveFavorable && !favorable) ||
                        (favorable == haveFavorable && score <= best)) continue;
                    best = score; haveFavorable = favorable; r.CandidateMeasured = true; r.Candidate = height; r.TrialBodyW = bodyW;
                    r.ReferenceBoxGain = refBoxGain; r.ReferenceAnkleGain = refAnkleGain; r.ReferenceCoastBoxGain = coastGain;
                    r.Sign = sign; r.Index = i; r.RefW = w.magnitude; r.RefA = a.magnitude;
                    r.TrialPd = trial.Pd; r.TrialFf = trial.AllocatedFf; r.TrialFlex = trial.Flex; r.CandidateClamped = trial.Clamped;
                }
            }
            r.State = r.Lanes == 0 ? Status.NoRoom : !r.CandidateMeasured ? Status.NoLegal :
                haveFavorable ? Status.Improved : Status.BelowGain;
            return r;
        }
    }
}
