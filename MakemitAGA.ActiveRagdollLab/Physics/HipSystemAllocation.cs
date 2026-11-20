using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One allocation contract shared by solve, pair precommit and receiver completion.
    // Eligibility uses CURRENT receiver control inputs. Both Hip arms retain Y;
    // both may reduce the separate Upper angular scale under the fixed cap.
    // Support is a previous completed stance/approach witness, never contact authority.
    internal static class HipSystemAllocation
    {
        internal const float MaxAngularTradeFraction = .10f;
        // q continues to own base SYSTEM XZ/Y; angularScale owns ONLY net Upper
        // torque and its relative reaction. Neither may be inferred from the other.
        internal static bool AngularValid(PelvisHipCommandFrame baseline, float q, float angularScale, float extraY) =>
            Valid(baseline, q, extraY) && HipPointPairMobility.F(angularScale) && angularScale >= 0f && angularScale <= q &&
            (angularScale == q || (baseline.BoundedTradeSelected && baseline.UpwardRetentionSelected &&
                baseline.UpwardRetentionEligible && baseline.JointTorqueReaction && q > 0f && q < 1f && extraY > 0f &&
                angularScale >= q * (1f - MaxAngularTradeFraction)));

        internal static bool Eligible(bool selected, PhysicsStepContext step, PhysicsStepContext referenceStep,
            PelvisComReferencePlanner.ComPlan reference, float errorY, Vector3 request) =>
            selected && referenceStep.Matches(step) && reference.ArrivalEnabled &&
            reference.SupportInput.Matches(step) && HipPointPairMobility.F(errorY) && errorY > 0f &&
            HipPointPairMobility.V(request) && request.y > 0f;

        internal static Vector3 Target(PelvisHipCommandFrame baseline, float q, float extraY)
        {
            Vector3 result = baseline.Acceleration * q;
            // Keep the original path's arithmetic exact when no retention is used.
            if (extraY != 0f) result.y += extraY;
            return result;
        }

        internal static bool Valid(PelvisHipCommandFrame baseline, float q, float extraY) =>
            HipPointPairMobility.F(q) && q >= 0f && q <= 1f && HipPointPairMobility.F(extraY) && extraY >= 0f &&
            (extraY == 0f || (baseline.UpwardRetentionSelected && baseline.UpwardRetentionEligible &&
                baseline.JointTorqueReaction && baseline.Acceleration.y > 0f &&
                extraY <= (1f - q) * baseline.Acceleration.y));

        internal static bool Matches(PelvisHipCommandFrame baseline, float q, float extraY,
            Vector3 common, Vector3 parentForce) => Valid(baseline, q, extraY) &&
            HipPointPairMobility.F(baseline.Response.TotalMass) && baseline.Response.TotalMass > 0f &&
            HipPointPairMobility.V(common) && common.magnitude <= PelvisHipCommandFrame.MaxMagnitude + .0001f &&
            HipPointPairMobility.V(parentForce) && HipPointPairMobility.V(Target(baseline, q, extraY)) &&
            (common + parentForce / baseline.Response.TotalMass - Target(baseline, q, extraY)).magnitude <=
                .0001f * (1f + Target(baseline, q, extraY).magnitude);
    }
}
