using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // A single PRE_N geometry and ankle forecast for all three orientation models.
    // Complete controller receipts own torque. Constraint impulses are observed only at PRE_N+1.
    internal static class RightFootTorqueForecast
    {
        internal enum Status : byte { MissingController, Predicted, MissingLeg, BodyMismatch, Geometry, InvalidMotion, Fault }
        internal enum HeightBand : byte { Missing, Below, InBand, Above }
        internal struct BodyMotion
        {
            internal Vector3 W, Inertia, Transport, Drive, Support;
            internal Quaternion WorldPrincipal;
            internal float Mass, Drag, MaxW;
        }
        internal struct Lane
        {
            internal Quaternion R;
            internal Vector3 W;
            internal float Box;
            internal bool Inside;
        }
        internal struct Result
        {
            internal Status State;
            internal Vector3 PredictedFoot;
            internal float LowerY;
            internal Lane Coast, Free, Issued;
            internal BodyMotion Motion;
        }
        internal static HeightBand Band(float height) => !PelvisParentFrame.F(height) ? HeightBand.Missing :
            height < -FootContactGeometryLimits.OrdinaryPenetrationMeters ? HeightBand.Below :
            height > FootContactGeometryLimits.RecoveryMaxGapMeters ? HeightBand.Above : HeightBand.InBand;

        internal static bool ValidRotation(Quaternion q) => PelvisParentFrame.Q(q) &&
            Mathf.Abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1f) < .001f;
        internal static float Clearance(in RightKneePivot.Frame p, Vector3 foot, Quaternion rotation) =>
            FootBoxGeometry.Minimum(foot, rotation, p.Center, p.Half) - (p.FloorCenter.y + p.FloorHalf.y);
        internal static bool ValidGeometry(in RightKneePivot.Frame p) =>
            ValidRotation(p.FootR) && PelvisParentFrame.V(p.Center) && PelvisParentFrame.V(p.Half) &&
            p.Half.x > 0f && p.Half.y > 0f && p.Half.z > 0f && PelvisParentFrame.V(p.FloorCenter) &&
            PelvisParentFrame.V(p.FloorHalf) && p.FloorHalf.x > 0f && p.FloorHalf.y > 0f && p.FloorHalf.z > 0f &&
            PelvisParentFrame.V(p.LowerAnkle) && PelvisParentFrame.V(p.FootAnkle) && RightKneePivot.Near(Clearance(p, p.FootAnkle, p.FootR), p.Box);
        private static Lane Project(in RightKneePivot.Frame p, Vector3 ankle, Vector3 w, float maxW)
        {
            var limited = Vector3.ClampMagnitude(w, maxW);
            var r = FootRelativeReference.Advance(p.FootR, limited, p.Dt);
            return new Lane { W = limited, R = r, Box = Clearance(p, ankle, r),
                Inside = FootBoxGeometry.Inside(ankle, r, p.Center, p.Half, p.FloorCenter, p.FloorHalf) };
        }
        // Returns the uncapped one-step velocity. Both observation models use the same
        // inertia and damping convention and apply the body's speed limit before rotating.
        internal static bool TryIssuedVelocity(in BodyMotion m, float dt, out float damping, out Vector3 issuedW)
        {
            damping = 0f; issuedW = default;
            if (!HipPointPairMobility.ValidBody(m.Mass, m.Inertia, m.WorldPrincipal) || !PelvisParentFrame.V(m.W) ||
                !PelvisParentFrame.V(m.Transport) || !PelvisParentFrame.V(m.Drive) || !PelvisParentFrame.V(m.Support) ||
                !PelvisParentFrame.F(m.Drag) || m.Drag < 0f || !PelvisParentFrame.F(m.MaxW) || m.MaxW <= 0f ||
                !PelvisParentFrame.F(dt) || dt <= 0f || dt > .1f) return false;
            var alpha = HipPointPairMobility.InverseTensor(m.Inertia, m.WorldPrincipal, m.Transport + m.Drive + m.Support);
            damping = Mathf.Max(0f, 1f - m.Drag * dt);
            issuedW = (m.W + alpha * dt) * damping;
            return PelvisParentFrame.V(alpha) && PelvisParentFrame.V(issuedW);
        }
        internal static Result Predict(in RightKneePivot.Frame p, Vector3 lower, Vector3 foot, in BodyMotion m)
        {
            if (!ValidGeometry(p) || !PelvisParentFrame.V(lower) || !PelvisParentFrame.V(foot))
                return new Result { State = Status.Geometry };
            if (!TryIssuedVelocity(m, p.Dt, out float damping, out var issuedW))
                return new Result { State = Status.InvalidMotion };
            var r = new Result { State = Status.Predicted, LowerY = lower.y, PredictedFoot = foot,
                Motion = m,
                Coast = Project(p, foot, m.W, m.MaxW), Free = Project(p, foot, m.W * damping, m.MaxW),
                Issued = Project(p, foot, issuedW, m.MaxW) };
            if (!PelvisParentFrame.F(r.Coast.Box) || !PelvisParentFrame.F(r.Free.Box) || !PelvisParentFrame.F(r.Issued.Box))
                return new Result { State = Status.Geometry };
            return r;
        }
    }
}
