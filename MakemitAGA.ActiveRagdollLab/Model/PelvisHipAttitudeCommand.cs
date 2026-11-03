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
        internal Vector3 CommonAcceleration, HipForce;
        internal Vector3 TargetSystemAcceleration => Baseline.Acceleration * MotionScale;
        internal Vector3 PredictedSystemAcceleration => CommonAcceleration + HipForce / Baseline.Response.TotalMass;
        internal Vector3 EffectiveHipTorque, RequestedTorque, AppliedTorque, CorrectionTorque, ResidualTorque;
        internal Vector3 AngularReaction, ExternalRequestedTorque;

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
            command = new PelvisHipAttitudeCommand { Valid = true, Baseline = baseline, MotionScale = 1f, CommonAcceleration = baseline.Acceleration, HipForce = hipForce,
                EffectiveHipTorque = h, ExternalRequestedTorque = requested, RequestedTorque = requested, AppliedTorque = applied,
                CorrectionTorque = correction, ResidualTorque = residual, Clamped = requested.magnitude > TotalTorqueLimit };
            return true;
        }
        // Joint-relative reaction is routed by the SAME receiver owner, once. The
        // external tracking/point compensation deliberately does not cancel it.
        // Both the external request and total direct input retain the 8 Nm bound.
        internal static bool TryPlanWithReaction(PelvisHipCommandFrame baseline, Vector3 hipForce, Vector3 hipMoment,
            Vector3 reaction, out PelvisHipAttitudeCommand command)
        {
            if (!TryPlan(baseline, hipForce, hipMoment, out command) || !V(reaction) ||
                (!baseline.JointTorqueReaction && !PelvisChestPointResponse.Same(reaction, Vector3.zero))) return false;
            if (!baseline.JointTorqueReaction) return true; // exact v106 A arithmetic
            command.AngularReaction = reaction;
            command.RequestedTorque = command.ExternalRequestedTorque + reaction;
            if (!V(command.RequestedTorque)) return false;
            command.AppliedTorque = Vector3.ClampMagnitude(command.RequestedTorque, TotalTorqueLimit);
            command.CorrectionTorque = command.AppliedTorque - baseline.ReceiverTorque;
            // Preserve this field's meaning: net change from the original baseline.
            // In B it includes the intended reaction. It is not a compensation fault.
            command.ResidualTorque = command.AppliedTorque + command.EffectiveHipTorque - baseline.ReceiverTorque;
            command.Clamped = command.Clamped || command.RequestedTorque.magnitude > TotalTorqueLimit;
            return V(command.AppliedTorque) && V(command.CorrectionTorque) && V(command.ResidualTorque);
        }
        // Arithmetic closure has a roundoff tolerance; identity/clock/receipt copies
        // remain exact. TryPlan above plans attitude only. Completion must supply the
        // allocated actuator input and pass this check BEFORE any receiver delta write.
        internal bool AllocationMatches() => Valid &&
            HipPointPairMobility.F(MotionScale) && MotionScale >= 0f && MotionScale <= 1f &&
            HipPointPairMobility.F(Baseline.Response.TotalMass) && Baseline.Response.TotalMass > 0f &&
            V(CommonAcceleration) && CommonAcceleration.magnitude <= PelvisHipCommandFrame.MaxMagnitude + 0.0001f &&
            V(HipForce) && V(TargetSystemAcceleration) && V(PredictedSystemAcceleration) &&
            (PredictedSystemAcceleration - TargetSystemAcceleration).magnitude <= 0.0001f * (1f + TargetSystemAcceleration.magnitude);
        internal bool Matches(PelvisHipCommandFrame baseline) => Valid && Committed && Baseline.SameAs(baseline) &&
            V(EffectiveHipTorque) && V(RequestedTorque) && V(AppliedTorque) && V(CorrectionTorque) && V(ResidualTorque) &&
            V(AngularReaction) && V(ExternalRequestedTorque) &&
            (Baseline.JointTorqueReaction || PelvisChestPointResponse.Same(AngularReaction, Vector3.zero)) &&
            ExternalRequestedTorque.magnitude <= TotalTorqueLimit + 0.001f &&
            AppliedTorque.magnitude <= TotalTorqueLimit + 0.001f &&
            AllocationMatches();
        private static bool V(Vector3 v) => PelvisParentFrame.V(v) && HipPointPairMobility.F(v.sqrMagnitude);
    }
}
