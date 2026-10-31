using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // The baseline receiver inputs stay immutable: both Hip owners consumed them
    // before the bilateral completion. This second receipt exists only after the
    // receiver's common-force deltas and correction AddTorque return, before the SAME native solve.
    internal struct PelvisHipAttitudeCommand
    {
        internal const float TotalTorqueLimit = 8f;
        internal bool Valid, Committed, Clamped;
        internal PelvisHipCommandFrame Baseline;
        internal float MotionScale;
        internal Vector3 CommonAcceleration;
        internal Vector3 EffectiveHipTorque, RequestedTorque, AppliedTorque, CorrectionTorque, ResidualTorque;

        internal static bool TryPlan(PelvisHipCommandFrame baseline, Vector3 hipForce, Vector3 hipMoment,
            out PelvisHipAttitudeCommand command)
        {
            command = new PelvisHipAttitudeCommand();
            if (!baseline.Matches(baseline.Step, baseline.BodyId) || !V(hipForce) || !V(hipMoment)) return false;
            // Eliminate the native Chest-COM constraint using the current model.
            // F/M are the SUM of the actual parent-side point-force commands.
            // This does not mirror pure Upper torque or apply either force again.
            Vector3 h = hipMoment - baseline.Response.ChestFraction * Vector3.Cross(baseline.Response.ChestLever, hipForce);
            Vector3 requested = baseline.ReceiverTorque - h;
            if (!V(h) || !V(requested)) return false;
            Vector3 applied = Vector3.ClampMagnitude(requested, TotalTorqueLimit);
            Vector3 correction = applied - baseline.ReceiverTorque;
            Vector3 residual = applied + h - baseline.ReceiverTorque;
            if (!V(applied) || !V(correction) || !V(residual)) return false;
            command = new PelvisHipAttitudeCommand { Valid = true, Baseline = baseline, MotionScale = 1f, CommonAcceleration = baseline.Acceleration,
                EffectiveHipTorque = h, RequestedTorque = requested, AppliedTorque = applied,
                CorrectionTorque = correction, ResidualTorque = residual, Clamped = requested.magnitude > TotalTorqueLimit };
            return true;
        }
        internal bool Matches(PelvisHipCommandFrame baseline) => Valid && Committed && Baseline.SameAs(baseline) &&
            V(EffectiveHipTorque) && V(RequestedTorque) && V(AppliedTorque) && V(CorrectionTorque) && V(ResidualTorque) &&
            AppliedTorque.magnitude <= TotalTorqueLimit + 0.001f &&
            HipPointPairMobility.F(MotionScale) && MotionScale >= 0f && MotionScale <= 1f &&
            V(CommonAcceleration) && PelvisChestPointResponse.Same(CommonAcceleration, baseline.Acceleration * MotionScale);
        private static bool V(Vector3 v) => PelvisParentFrame.V(v) && HipPointPairMobility.F(v.sqrMagnitude);
    }
}
