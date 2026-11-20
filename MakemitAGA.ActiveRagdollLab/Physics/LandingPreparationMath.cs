using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Shared value-only geometry and budget checks for Hip/Knee preparation.
    // Each original planner alone owns its trajectory; neither selector owns native authority.
    internal static class LandingPreparationMath
    {
        internal const int SamplesPerLane = 4;
        internal struct Lane
        {
            internal Quaternion Previous, Goal, Baseline;
            internal Vector3 PreviousW, NominalW, W, Direction;
            internal float MaxW, MaxA, Dt, Gap, Debt, Room;
            internal bool Knee;
        }
        internal static bool ValidLane(Quaternion previous, Vector3 previousW, Quaternion goal, Vector3 nominalW,
            Quaternion rotation, Vector3 w, Vector3 a, float dt, float maxW, float maxA)
        {
            if (!Q(previous) || !Q(goal) || !Q(rotation) || !V(previousW) || !V(nominalW) || !V(w) || !V(a) ||
                !F(dt) || dt <= 0f || dt >= .25f || !F(maxW) || maxW <= 0f || !F(maxA) || maxA <= 0f) return false;
            return w.magnitude <= Mathf.Max(maxW, previousW.magnitude) + .0001f &&
                a.magnitude <= maxA + .002f && ((w - previousW) / dt - a).magnitude <= .0001f &&
                w.magnitude * dt < Mathf.PI - .0001f &&
                FootRelativeReference.Log(rotation * Quaternion.Inverse(FootRelativeReference.Advance(previous, w, dt))).magnitude <= .00001f;
        }
        internal static Lane MakeLane(Quaternion previous, Vector3 previousW, Quaternion goal, Vector3 nominalW,
            Quaternion baseline, Vector3 w, float dt, float maxW, float maxA, Vector3 axis, bool knee)
        {
            var l = new Lane { Previous = previous, PreviousW = previousW, Goal = goal, NominalW = nominalW,
                Baseline = baseline, W = w, Dt = dt, MaxW = maxW, MaxA = maxA, Knee = knee };
            Vector3 error = FootRelativeReference.Log(goal * Quaternion.Inverse(baseline));
            l.Gap = error.magnitude;
            if (l.Gap < .000001f || (knee && (!V(axis) || axis.sqrMagnitude < .0001f))) return l;
            l.Direction = knee ? (previous * axis.normalized).normalized : error / l.Gap;
            float along = Vector3.Dot(error, l.Direction);
            if (along < 0f) { l.Direction = -l.Direction; along = -along; }
            if (along < .000001f) return l;
            l.Room = (float)Math.Min(along / dt, Math.Min(
                GroundReferenceStepBudget.PositiveBallExit(w - previousW, l.Direction, maxA * dt),
                GroundReferenceStepBudget.PositiveBallExit(w, l.Direction, Mathf.Max(maxW, previousW.magnitude)))) * .99999f;
            l.Debt = GroundReferenceStepBudget.BrakeDebt(w - nominalW, l.Direction, along, maxA, dt);
            return l;
        }
        internal static bool TryLane(Lane l, KneeGroundTarget.Geometry g, int sample, out Quaternion rotation, out Vector3 w)
        {
            rotation = l.Baseline; w = l.W;
            if (sample == 0) return true;
            if (!F(l.Room) || l.Room <= .000001f) return false;
            w += l.Direction * (l.Room * sample / SamplesPerLane);
            Vector3 a = (w - l.PreviousW) / l.Dt;
            if (!V(w) || !V(a) || w.magnitude > Mathf.Max(l.MaxW, l.PreviousW.magnitude) + .00001f ||
                a.magnitude > l.MaxA + .0001f || w.magnitude * l.Dt >= Mathf.PI - .0001f) return false;
            rotation = FootRelativeReference.Advance(l.Previous, w, l.Dt);
            Vector3 error = FootRelativeReference.Log(l.Goal * Quaternion.Inverse(rotation));
            if (!Q(rotation) || error.magnitude > l.Gap + .000001f || Vector3.Dot(error, l.Direction) < -.000001f ||
                GroundReferenceStepBudget.BrakeDebt(w - l.NominalW, l.Direction, Vector3.Dot(error, l.Direction), l.MaxA, l.Dt) > l.Debt + .000001f) return false;
            if (l.Knee)
            {
                float bend = Bend(g, rotation), oldBend = Bend(g, l.Baseline), goalBend = Bend(g, l.Goal);
                // Stay between the unchanged baseline/goal bends on the same
                // calibrated monotone branch. No hyperextension or added endpoint.
                Vector3 localAxis = Quaternion.Inverse(l.Previous) * l.Direction;
                float start = Vector3.Dot(g.UpperSpan.normalized, l.Baseline * Vector3.Cross(localAxis, g.LowerSpan.normalized));
                float end = Vector3.Dot(g.UpperSpan.normalized, rotation * Vector3.Cross(localAxis, g.LowerSpan.normalized));
                if (!F(bend) || Mathf.Abs(start) < .000001f || start * end <= 0f ||
                    bend < KneeGroundTarget.MinExtensionBendDegrees || bend > KneeGroundTarget.MaxBendDegrees ||
                    bend < Mathf.Min(oldBend, goalBend) - .0001f || bend > Mathf.Max(oldBend, goalBend) + .0001f) return false;
            }
            return true;
        }
        internal static float Bend(KneeGroundTarget.Geometry g, Quaternion knee) => Mathf.Acos(Mathf.Clamp(
            Vector3.Dot(g.UpperSpan.normalized, (knee * g.LowerSpan).normalized), -1f, 1f)) * Mathf.Rad2Deg;
        internal static bool LegalPose(KneeGroundTarget.Geometry g, Quaternion foot, Quaternion upper, Quaternion knee)
        {
            var changed = g; changed.Upper = upper;
            return FootGroundReference.GoalWithinBounds(upper * knee, g.FootRelative, foot, g.FootBindRelative) &&
                FootBoxGeometry.Inside(KneeGroundTarget.Ankle(changed, knee), upper * knee * foot,
                    g.Center, g.Half, g.FloorCenter, g.FloorHalf);
        }
        internal static float Minimum(KneeGroundTarget.Geometry g, Quaternion upper, Quaternion knee, Quaternion foot)
        { g.Upper = upper; return KneeGroundTarget.Minimum(g, knee, foot); }
        private static bool F(float v) => PelvisParentFrame.F(v);
        private static bool V(Vector3 v) => PelvisParentFrame.V(v);
        private static bool Q(Quaternion q) => PelvisParentFrame.Q(q) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < .01f;
    }
}
