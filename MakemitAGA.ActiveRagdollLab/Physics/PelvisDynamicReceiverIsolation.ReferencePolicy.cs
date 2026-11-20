using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Both arms retain v163 B physical control. B replays a three-step foot tether alternative from the unloaded shin and foot receipts.
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        internal const string DynamicHipArmA = "A_V041660_PREVIOUS_STEP_BASELINE";
        internal const string DynamicHipArmB = "B_V041660_LEFT_FOOT_HORIZONTAL_LEAD";
        internal static bool UsesSupportBraking(string arm) => IsDynamicHipArm(arm);
        internal static bool UsesSupportNormalRecovery(string arm) => IsDynamicHipArm(arm);
        internal static bool UsesHipLandingPreparation(string arm) => false; // Both formal arms use the V142 A physical baseline.
        internal static bool UsesKneeLandingPreparation(string arm) => IsDynamicHipArm(arm);
        internal static bool IsDynamicHipArm(string arm) => arm == DynamicHipArmA || arm == DynamicHipArmB;

        // Called by the actual Step after its native state/safety checks and before
        // the existing force pair. Optional evidence never selects this policy.
        internal static SupportNormalRecovery.Result BuildCommonAcceleration(string arm, Vector3 rawA,
            Vector3 body, Vector3 velocity, PhysicsStepContext step, PelvisComReferencePlanner.ComPlan plan)
        {
            Vector3 baseline = Vector3.ClampMagnitude(rawA, PelvisHipCommandFrame.MaxMagnitude);
            SupportReferenceContext support = IsDynamicHipArm(arm) && plan.SupportFloorSelected &&
                plan.SupportInput.Matches(step) ? plan.SupportInput : default;
            return SupportNormalRecovery.Build(baseline, body, velocity, step,
                UsesSupportNormalRecovery(arm), support, plan.SupportFloorY);
        }

        // The actual receiver calls this with its private reference state and the
        // already validated previous completion. Keep current-step support matching
        // and arm selection in this testable production boundary, not a report flag.
        internal static PelvisComReferencePlanner.ComPlan BuildComReferencePlan(string arm,
            Vector3 position, Vector3 velocity, Vector3 goal, Vector3 goalV, float dt, PhysicsStepContext step,
            bool feedback, Vector3 previousRequest, Vector3 previousAllocated, Vector3 settledV,
            SupportReferenceContext support, float floorY)
        {
            bool hip = IsDynamicHipArm(arm);
            bool selected = hip && support.Matches(step);
            return PelvisComReferencePlanner.Build(position, velocity, goal, goalV, dt, hip, feedback,
                previousRequest, previousAllocated, settledV, hip, support.Matches(step) ? support : default,
                selected, floorY, hip, hip, UsesSupportBraking(arm));
        }
    }
}
