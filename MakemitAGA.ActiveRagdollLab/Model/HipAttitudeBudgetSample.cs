using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Read-only, same-state budget arithmetic AFTER both Hip point-force writes.
    // H includes Chest COM elimination, not just the moment about Pelvis COM.
    // No pure Upper Hip torque is mirrored to Pelvis. This remains diagnostic;
    // the separate committed attitude receipt supplies the actual FINAL torque.
    internal struct HipAttitudeBudgetSample
    {
        internal bool Valid, ReceiverClamped, PreserveRawExceeds, PreserveAppliedExceeds;
        internal Vector3 RawReceiverTorque, EffectiveHipTorque, PreserveRawTorque, PreserveAppliedTorque;
        internal Vector3 HipResponseAlpha, RawResponseDeficitAlpha;
        internal static HipAttitudeBudgetSample Evaluate(PelvisHipCommandFrame input,
            PelvisReceiverReport.AngularActuationSample actuation, PelvisHipAttitudeCommand attitude, Vector3 hipForce, Vector3 hipMoment)
        {
            var f = new HipAttitudeBudgetSample();
            if (!input.Matches(input.Step, input.BodyId) || !actuation.Valid || !actuation.Step.Matches(input.Step) ||
                !PelvisChestPointResponse.Same(actuation.Torque, input.ReceiverTorque) || !attitude.Matches(input) ||
                !V(actuation.RawTorque) || !V(hipForce) || !V(hipMoment)) return f;
            f.RawReceiverTorque = actuation.RawTorque; // after alpha40, before the existing torque2 cap
            f.EffectiveHipTorque = hipMoment - input.Response.ChestFraction * Vector3.Cross(input.Response.ChestLever, hipForce);
            f.PreserveRawTorque = f.RawReceiverTorque - f.EffectiveHipTorque;
            f.PreserveAppliedTorque = input.ReceiverTorque - f.EffectiveHipTorque;
            f.HipResponseAlpha = input.Response.AngularInertia.Solve(f.EffectiveHipTorque);
            f.RawResponseDeficitAlpha = input.Response.AngularInertia.Solve(attitude.AppliedTorque + f.EffectiveHipTorque - f.RawReceiverTorque);
            f.Valid = V(f.EffectiveHipTorque) && V(f.PreserveRawTorque) && V(f.PreserveAppliedTorque) &&
                V(f.HipResponseAlpha) && V(f.RawResponseDeficitAlpha);
            f.ReceiverClamped = f.RawReceiverTorque.magnitude > 2f;
            // 1 mNm read-only tolerance keeps float noise at the budget boundary out of counts.
            f.PreserveRawExceeds = f.PreserveRawTorque.magnitude > 2.001f;
            f.PreserveAppliedExceeds = f.PreserveAppliedTorque.magnitude > 2.001f;
            return f;
        }
        private static bool V(Vector3 v) => PelvisParentFrame.V(v) && HipPointPairMobility.F(v.sqrMagnitude);
    }
}
