using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // A bounded request policy inside the existing receiver, not a new actuator.
    // The held reference floor is NOT a terrain surface or a load authorization.
    internal static class SupportNormalRecovery
    {
        internal struct Result
        {
            internal PhysicsStepContext Step;
            internal SupportReferenceContext Support;
            internal bool Selected, Qualified, Eligible;
            internal Vector3 Baseline, Request;
            internal float FloorY, BodyY, BodyVY, ProjectedY, RequiredY;
            internal float ExtraY => Request.y - Baseline.y;
            internal float RemainingY => Eligible ? Mathf.Max(0f, RequiredY - Request.y) : 0f;
        }

        internal static Result Build(Vector3 baseline, Vector3 body, Vector3 velocity,
            PhysicsStepContext step, bool selected, SupportReferenceContext support, float floorY)
        {
            var r = new Result { Step = step, Selected = selected, Baseline = baseline, Request = baseline };
            if (!selected || !support.Matches(step)) return r;
            if (!V(baseline) || baseline.magnitude > PelvisHipCommandFrame.MaxMagnitude + .001f ||
                !V(body) || !V(velocity) || !F(floorY))
                throw new InvalidOperationException("SUPPORT_NORMAL_RECOVERY_INPUT_INVALID");
            r.Qualified = true; r.Support = support; r.FloorY = floorY;
            r.BodyY = body.y; r.BodyVY = velocity.y;
            // Constant-velocity lookahead only decides whether to consider recovery.
            // It does not predict this solve, move the reference or authorize contact.
            r.ProjectedY = body.y + velocity.y * step.Dt;
            if (!F(r.ProjectedY)) throw new InvalidOperationException("SUPPORT_NORMAL_RECOVERY_PROJECTION_INVALID");
            r.Eligible = body.y < floorY || r.ProjectedY < floorY;
            if (!r.Eligible) return r;

            // Reuse v124's normal gains with this step's held floor and zero floor V.
            // Upward body V retains its damping: the bound may still be NEGATIVE.
            // No extra gain, fixed lift, integral, future animation or q inversion.
            r.RequiredY = PelvisComFeedback.NormalPositionGain * (floorY - body.y) -
                PelvisComFeedback.NormalVelocityGain * velocity.y;
            if (!F(r.RequiredY)) throw new InvalidOperationException("SUPPORT_NORMAL_RECOVERY_REQUEST_INVALID");
            if (r.RequiredY <= baseline.y) return r;

            // Preserve the already-capped X/Z request bit for bit. Use only normal
            // room in the original 12 m/s^2 sphere; subsequent bilateral allocation
            // and actual-completion deficit feedback keep their existing authority.
            double cap = PelvisHipCommandFrame.MaxMagnitude;
            double room = cap * cap - (double)baseline.x * baseline.x - (double)baseline.z * baseline.z;
            float highestY = (float)Math.Sqrt(Math.Max(0d, room));
            float y = Mathf.Min(r.RequiredY, highestY);
            if (y > baseline.y) r.Request = new Vector3(baseline.x, y, baseline.z);
            if (!V(r.Request) || r.Request.magnitude > PelvisHipCommandFrame.MaxMagnitude + .001f)
                throw new InvalidOperationException("SUPPORT_NORMAL_RECOVERY_FINAL_INVALID");
            return r;
        }
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
    }
}
