using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // PRE_N pose substitution. A positive part lowers the actual knee relative to its selected goal.
    // A substitution has no command owner. PhysX, controller budgets and future contacts are not simulated.
    internal static class RightKneePivot
    {
        internal enum Capture : byte { Disabled, OutsideSwing, MissingAnchors, GeometryMismatch, Ready, Fault }
        internal enum Part : byte { NoDeficit, Hip, Thigh, KneeGap, Tied }
        internal const int PriorSteps = 4;
        private const float ClosureTolerance = .00001f;

        // Kept only in the right B ring. All poses refer to the same pre-solve and support.
        internal struct Frame
        {
            internal Capture State;
            internal float Dt, ReferenceHipY, ParentHipY, GoalKneeY, RawAnkleY, Box;
            internal Vector3 UpperHip, UpperKnee, LowerKnee, LowerAnkle, FootAnkle;
            internal Quaternion UpperR, GoalR, FootR;
            internal Vector3 Center, Half, FloorCenter, FloorHalf;
        }
        internal struct Split
        {
            internal float Total, Receiver, HipJoint, Thigh, KneeGap, Closure, Box;
            internal float Hip => Receiver + HipJoint;
            internal Part Dominant;
        }
        internal struct Trial
        {
            internal bool Measured, Inside, WasInBand, InBand;
            internal float KneeY, AnkleY, Box, Angle, KneeGain, AnkleGain, DepthGain, AddedClearance;
        }

        private static bool Unit(Quaternion q) => PelvisParentFrame.Q(q) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) <= .001f;
        private static bool Positive(Vector3 v) => PelvisParentFrame.V(v) && v.x > 0f && v.y > 0f && v.z > 0f;
        internal static bool Near(float a, float b) => PelvisParentFrame.F(a) && PelvisParentFrame.F(b) && Mathf.Abs(a - b) <= ClosureTolerance;
        private static float Clearance(in Frame frame, Vector3 ankle, Quaternion rotation) =>
            FootBoxGeometry.Minimum(ankle, rotation, frame.Center, frame.Half) - (frame.FloorCenter.y + frame.FloorHalf.y);
        private static bool InBand(float box) => box >= -FootContactGeometryLimits.OrdinaryPenetrationMeters &&
            box <= FootContactGeometryLimits.RecoveryMaxGapMeters;

        internal static Capture Decompose(in Frame frame, out Split split)
        {
            split = default;
            if (frame.State != Capture.Ready) return frame.State;
            if (!Unit(frame.UpperR) || !Unit(frame.GoalR) || !Unit(frame.FootR) || !Positive(frame.Half) || !Positive(frame.FloorHalf) ||
                !PelvisParentFrame.V(frame.Center) || !PelvisParentFrame.V(frame.FloorCenter) || !PelvisParentFrame.V(frame.UpperHip) ||
                !PelvisParentFrame.V(frame.UpperKnee) || !PelvisParentFrame.V(frame.LowerKnee) || !PelvisParentFrame.V(frame.LowerAnkle) ||
                !PelvisParentFrame.V(frame.FootAnkle) || !PelvisParentFrame.F(frame.RawAnkleY) || !PelvisParentFrame.F(frame.Box) ||
                !PelvisParentFrame.F(frame.Dt) || frame.Dt <= 0f || frame.Dt > .1f) return Capture.GeometryMismatch;

            var upperSpan = frame.UpperKnee - frame.UpperHip;
            if (upperSpan.sqrMagnitude <= 1e-10f || (frame.LowerAnkle - frame.LowerKnee).sqrMagnitude <= 1e-10f)
                return Capture.GeometryMismatch;
            split.Total = frame.GoalKneeY - frame.LowerKnee.y;
            split.Receiver = frame.ReferenceHipY - frame.ParentHipY;
            split.HipJoint = frame.ParentHipY - frame.UpperHip.y;
            split.Thigh = ((frame.GoalR * Quaternion.Inverse(frame.UpperR)) * upperSpan).y - upperSpan.y;
            split.KneeGap = frame.UpperKnee.y - frame.LowerKnee.y;
            split.Closure = split.Total - (split.Hip + split.Thigh + split.KneeGap);
            split.Box = Clearance(frame, frame.FootAnkle, frame.FootR);
            if (!Near(split.Closure, 0f) || !PelvisParentFrame.F(split.Box) ||
                Mathf.Abs(split.Box - frame.Box) > FootBoxGeometry.SupportPlaneToleranceMeters ||
                !FootBoxGeometry.Inside(frame.FootAnkle, frame.FootR, frame.Center, frame.Half, frame.FloorCenter, frame.FloorHalf))
                return Capture.GeometryMismatch;

            float largest = Mathf.Max(split.Hip, Mathf.Max(split.Thigh, split.KneeGap));
            if (split.Total < KneeGroundTarget.MinGain || largest <= ClosureTolerance) return Capture.Ready;
            bool hip = largest - split.Hip <= ClosureTolerance, thigh = largest - split.Thigh <= ClosureTolerance;
            bool gap = largest - split.KneeGap <= ClosureTolerance;
            split.Dominant = (hip && (thigh || gap)) || (thigh && gap) ? Part.Tied :
                hip ? Part.Hip : thigh ? Part.Thigh : Part.KneeGap;
            return Capture.Ready;
        }

        // Full uses the selected endpoint. Recent uses only the magnitude traversed by this
        // same degree of freedom in the previous measured interval, scaled to the current dt.
        // Hip and gap are vertical translations. Thigh follows the shortest angular path;
        // relative joint rotations and both world anchor offsets stay fixed during its substitution.
        internal static Trial Compare(in Frame frame, in Split split, in Frame previous, bool recent)
        {
            if (split.Dominant != Part.Hip && split.Dominant != Part.Thigh && split.Dominant != Part.KneeGap) return default;
            if (recent && (previous.State != Capture.Ready || !(previous.Dt > 0f))) return default;

            float rate = recent ? frame.Dt / previous.Dt : 1f;
            var knee = frame.LowerKnee;
            var ankle = frame.LowerAnkle;
            var foot = frame.FootAnkle;
            var rotation = frame.FootR;
            float angle = 0f;
            if (split.Dominant == Part.Thigh)
            {
                var turn = frame.GoalR * Quaternion.Inverse(frame.UpperR);
                var arc = FootRelativeReference.Log(turn);
                if (recent)
                {
                    var incoming = FootRelativeReference.Log(frame.UpperR * Quaternion.Inverse(previous.UpperR));
                    float travel = incoming.magnitude;
                    arc = travel <= .000001f ? Vector3.zero : Vector3.ClampMagnitude(arc, travel * rate);
                    turn = FootRelativeReference.Advance(Quaternion.identity, arc, 1f);
                }
                angle = arc.magnitude * Mathf.Rad2Deg;
                knee = frame.UpperHip + turn * (frame.UpperKnee - frame.UpperHip) + (frame.LowerKnee - frame.UpperKnee);
                ankle = knee + turn * (frame.LowerAnkle - frame.LowerKnee);
                foot = ankle + (frame.FootAnkle - frame.LowerAnkle);
                rotation = turn * frame.FootR;
            }
            else
            {
                bool hip = split.Dominant == Part.Hip;
                float shift = hip ? split.Hip : split.KneeGap;
                if (recent)
                {
                    float travel = hip ? frame.UpperHip.y - previous.UpperHip.y :
                        split.KneeGap - (previous.UpperKnee.y - previous.LowerKnee.y);
                    shift = Mathf.Min(shift, Mathf.Abs(travel) * rate);
                }
                knee.y += shift;
                ankle.y += shift;
                foot.y += shift;
            }

            float box = Clearance(frame, foot, rotation);
            if (!PelvisParentFrame.V(knee) || !PelvisParentFrame.V(ankle) || !PelvisParentFrame.F(box)) return default;
            return new Trial
            {
                Measured = true,
                Inside = FootBoxGeometry.Inside(foot, rotation, frame.Center, frame.Half, frame.FloorCenter, frame.FloorHalf),
                WasInBand = InBand(split.Box),
                InBand = InBand(box),
                Angle = angle,
                KneeY = knee.y - frame.LowerKnee.y,
                AnkleY = ankle.y - frame.LowerAnkle.y,
                Box = box,
                KneeGain = Mathf.Abs(split.Total) - Mathf.Abs(frame.GoalKneeY - knee.y),
                AnkleGain = Mathf.Abs(frame.RawAnkleY - frame.LowerAnkle.y) - Mathf.Abs(frame.RawAnkleY - ankle.y),
                DepthGain = Mathf.Max(0f, -split.Box) - Mathf.Max(0f, -box),
                AddedClearance = Mathf.Max(0f, box) - Mathf.Max(0f, split.Box)
            };
        }
    }
}
