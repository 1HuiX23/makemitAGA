using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Optional choice INSIDE the existing Knee Fixed reference step. The goal,
    // Foot Render reference and both original W/A balls are unchanged. This is
    // a held-geometry endpoint comparison, not a prediction of the native solve.
    internal static class KneeGroundTrajectory
    {
        internal enum Status { Unavailable, Stale, InvalidGeometry, BaselineClear,
            EndpointBelow, NoBendProgress, NoHeadroom, NoLegalImprovement, Improved }
        internal struct Result
        {
            internal Status Reason;
            internal bool Evaluated, Better, Applied;
            internal float BaselineMin, GoalMin, CandidateMin, DeltaDegrees;
            internal Quaternion R;
            internal Vector3 W;
            internal string Text() => "kneeGroundTrajectoryV041310=" + Reason +
                "/trajectory(evaluated-better-applied)=" + Evaluated + "|" + Better + "|" + Applied +
                "/trajectoryMinMm(base-goal-candidate)=" + (Evaluated
                    ? N(BaselineMin * 1000f) + "|" + N(GoalMin * 1000f) + "|" + N(CandidateMin * 1000f) : "NA|NA|NA") +
                "/trajectoryDeltaDeg=" + N(DeltaDegrees) +
                "/trajectoryDomain=HELD_RENDER_GEOMETRY_CURRENT_FIXED_KNEE_AND_PUBLISHED_FOOT_NOT_SOLVED";
        }

        internal static Result Evaluate(KneeGroundTarget.Geometry g, Quaternion foot, Vector3 lowerAxis,
            Quaternion previousR, Vector3 previousW, Quaternion goal, Vector3 nominalW, Quaternion baselineR, Vector3 baselineW,
            float dt, float maxW, float maxA)
        {
            var result = new Result { Reason = Status.InvalidGeometry, R = baselineR, W = baselineW };
            if (!g.Valid || !Q(foot) || !V(lowerAxis) || lowerAxis.sqrMagnitude < .0001f ||
                !Q(previousR) || !Q(goal) || !Q(baselineR) || !V(previousW) || !V(nominalW) || !V(baselineW) ||
                !F(dt) || dt <= 0f || dt >= .25f || !F(maxW) || maxW <= 0f || !F(maxA) || maxA <= 0f)
                return result;
            float baselineMin = KneeGroundTarget.Minimum(g, baselineR, foot);
            float goalMin = KneeGroundTarget.Minimum(g, goal, foot);
            if (!F(baselineMin) || !F(goalMin) || !Inside(g, baselineR, foot) || !Inside(g, goal, foot)) return result;
            result.Evaluated = true; result.BaselineMin = result.CandidateMin = baselineMin; result.GoalMin = goalMin;
            if (baselineMin >= -KneeGroundTarget.MinGain) { result.Reason = Status.BaselineClear; return result; }
            // A partial/unclear endpoint has no proven clear target to approach.
            // Use the SAME published Foot R for both endpoint tests, not its ideal goal.
            if (goalMin < 0f) { result.Reason = Status.EndpointBelow; return result; }

            Vector3 localAxis = lowerAxis.normalized;
            Vector3 axis = (previousR * localAxis).normalized; // Upper-local W basis.
            Vector3 remaining = FootRelativeReference.Log(goal * Quaternion.Inverse(baselineR));
            float along = Vector3.Dot(remaining, axis);
            float sign = along >= 0f ? 1f : -1f;
            float baselineBend = Bend(g, baselineR);
            // Never straighten/cross the near-straight branch to obtain clearance.
            // The common baseline planner may approach raw or bounded-extension inputs;
            // this additional intermediate-step choice retains its original bend-only rule.
            if (Mathf.Abs(along) < .000001f || sign * Slope(g, baselineR, localAxis) >= -.000001f ||
                baselineBend >= KneeGroundTarget.MaxBendDegrees)
            { result.Reason = Status.NoBendProgress; return result; }
            Vector3 direction = axis * sign;
            float speedBound = Mathf.Max(maxW, previousW.magnitude);
            double roomA = GroundReferenceStepBudget.PositiveBallExit(baselineW - previousW, direction, maxA * dt);
            double roomW = GroundReferenceStepBudget.PositiveBallExit(baselineW, direction, speedBound);
            // The axis increment is also limited by the unchanged input endpoint.
            // Recheck its true quaternion error below; this scalar is not an IK solve.
            float room = (float)Math.Min(Math.Min(roomA, roomW), Math.Abs(along) / dt);
            if (!F(room) || room <= .000001f) { result.Reason = Status.NoHeadroom; return result; }
            room *= .99999f; // inward rounding; no enlargement of either budget
            result.Reason = Status.NoLegalImprovement;
            float oldGap = remaining.magnitude;
            float oldBrakeDebt = GroundReferenceStepBudget.BrakeDebt(baselineW - nominalW, direction, Mathf.Abs(along), maxA, dt);
            // Fixed, allocation-free search: first sampled clear step, else best
            // legal partial gain >= the existing 0.1 mm geometric noise threshold.
            for (int i = 1; i <= 16; i++)
            {
                Vector3 w = baselineW + direction * (room * i / 16f);
                if (!V(w) || w.magnitude > speedBound + .00001f ||
                    ((w - previousW) / dt).magnitude > maxA + .0001f || w.magnitude * dt >= Mathf.PI - .0001f) continue;
                Quaternion r = FootRelativeReference.Advance(previousR, w, dt);
                Vector3 error = FootRelativeReference.Log(goal * Quaternion.Inverse(r));
                float bend = Bend(g, r);
                if (!Q(r) || !V(error) || error.magnitude > oldGap + .000001f ||
                    sign * Vector3.Dot(error, axis) < -.000001f ||
                    GroundReferenceStepBudget.BrakeDebt(w - nominalW, direction, Vector3.Dot(error, direction), maxA, dt) > oldBrakeDebt + .000001f ||
                    !F(bend) || bend < baselineBend || bend > KneeGroundTarget.MaxBendDegrees ||
                    sign * Slope(g, r, localAxis) >= -.000001f || !Inside(g, r, foot)) continue;
                float min = KneeGroundTarget.Minimum(g, r, foot);
                if (!F(min) || min - baselineMin < KneeGroundTarget.MinGain || min <= result.CandidateMin) continue;
                result.Better = true; result.Reason = Status.Improved;
                result.R = r; result.W = w; result.CandidateMin = min;
                result.DeltaDegrees = FootRelativeReference.Log(r * Quaternion.Inverse(baselineR)).magnitude * Mathf.Rad2Deg;
                if (min >= 0f) break;
            }
            return result;
        }

        private static bool Inside(KneeGroundTarget.Geometry g, Quaternion knee, Quaternion foot) =>
            FootBoxGeometry.Inside(KneeGroundTarget.Ankle(g, knee), g.Upper * knee * foot,
                g.Center, g.Half, g.FloorCenter, g.FloorHalf);
        private static float Bend(KneeGroundTarget.Geometry g, Quaternion knee) => Mathf.Acos(Mathf.Clamp(
            Vector3.Dot(g.UpperSpan.normalized, (knee * g.LowerSpan).normalized), -1f, 1f)) * Mathf.Rad2Deg;
        private static float Slope(KneeGroundTarget.Geometry g, Quaternion knee, Vector3 axis) =>
            Vector3.Dot(g.UpperSpan.normalized, knee * Vector3.Cross(axis, g.LowerSpan.normalized));
        private static bool F(float x) => PelvisParentFrame.F(x);
        private static bool V(Vector3 x) => PelvisParentFrame.V(x);
        private static bool Q(Quaternion x) => PelvisParentFrame.Q(x);
        private static string N(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
    }
}
