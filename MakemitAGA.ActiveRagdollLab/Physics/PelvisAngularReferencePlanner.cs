using System;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Pure angular planning shared by the applied reference and its read-only alternative.
    // No body access, force write, source prediction, time history or report dependency.
    internal static class PelvisAngularReferencePlanner
    {
        internal const float MaxW = 3f, MaxAlpha = 4f, MaxBrakeAlpha = 8f, TorqueBudget = 1f;
        internal struct Plan
        {
            internal Quaternion Rotation;
            internal Vector3 W, PreviousW, BridgeW, DesiredW, LimitedW, Alpha;
            internal Vector3 RelativeW, StoppingOffset, ArrivalError, InertiaTorque, RigidGyro;
            internal float ErrorAngle, BrakeCap, BaseStepCap, StepCap, BrakeInertia, StepInertia, ExtraBrakeAlpha;
            internal bool SpeedLimited, AlphaLimited;
        }

        internal static Plan Build(Quaternion rotation, Vector3 w, Quaternion goalRotation, Vector3 goalW,
            float dt, float scalarCap, bool directional, bool brakingReserve, PelvisReferenceInertiaBudget budget)
        {
            if (!Positive(dt) || !Positive(scalarCap) || !V(w) || !V(goalW))
                throw new InvalidOperationException("REFERENCE_PLANNER_INPUT_INVALID");
            Quaternion origin = Advance(rotation, goalW, dt);
            Quaternion error = Normalize(goalRotation * Quaternion.Inverse(origin));
            if (error.w < 0f) error = new Quaternion(-error.x, -error.y, -error.z, -error.w);
            // Preserve V04580/V04660 arithmetic and the deterministic 180-degree tie.
            Vector3 e = new Vector3(error.x, error.y, error.z);
            if (error.w == 0f && (e.x < 0f || (e.x == 0f && (e.y < 0f || (e.y == 0f && e.z < 0f))))) e = -e;
            float sinHalf = e.magnitude;
            float angle = 2f * Mathf.Atan2(sinHalf, Mathf.Abs(error.w));
            Vector3 errorVector = sinHalf > 0f ? e * (angle / sinHalf) : Vector3.zero;
            Vector3 relativeW = w - goalW;
            float brakeCap = scalarCap, stepCap = scalarCap, brakeInertia = 0f, stepInertia = 0f;
            if (directional && !budget.TryAlphaCap(relativeW, scalarCap, MaxAlpha, TorqueBudget, out brakeCap, out brakeInertia))
                throw new InvalidOperationException("REFERENCE_PLANNER_BRAKE_AXIS_INVALID");
            // BOTH arms retain the old conservative arrival distance. The reserve changes
            // only the subsequent removal of existing WORLD W, never this feasibility estimate.
            Vector3 stoppingOffset = relativeW * (relativeW.magnitude / (2f * brakeCap) + dt);
            Vector3 arrivalError = errorVector - stoppingOffset;
            Vector3 desiredW = goalW + 4f * arrivalError;
            if (!V(stoppingOffset) || !V(desiredW)) throw new InvalidOperationException("REFERENCE_PLANNER_ARRIVAL_NONFINITE");
            Vector3 limitedW = Vector3.ClampMagnitude(desiredW, MaxW);
            Vector3 deltaW = limitedW - w;
            if (directional && !budget.TryAlphaCap(deltaW, scalarCap, MaxAlpha, TorqueBudget, out stepCap, out stepInertia))
                throw new InvalidOperationException("REFERENCE_PLANNER_STEP_AXIS_INVALID");
            float baseCap = stepCap;
            Vector3 nextW = Vector3.ClampMagnitude(w + Vector3.ClampMagnitude(deltaW, stepCap * dt), MaxW);
            float extra = 0f;
            if (directional && brakingReserve)
            {
                nextW = RemoveExistingSpeed(w, nextW, limitedW, dt, budget, out extra);
                if (extra > 0f && !budget.TryAlphaCap(nextW - w, scalarCap, MaxBrakeAlpha, TorqueBudget, out stepCap, out stepInertia))
                    throw new InvalidOperationException("REFERENCE_PLANNER_RESERVE_AXIS_INVALID");
            }
            Vector3 alpha = (nextW - w) / dt;
            Vector3 torque = directional ? budget.Apply(alpha) : Vector3.zero;
            Vector3 gyro = directional ? budget.RigidGyroEstimate(w) : Vector3.zero;
            float ceiling = extra > 0f ? MaxBrakeAlpha : MaxAlpha;
            if (!V(nextW) || !V(alpha) || !V(torque) || !V(gyro) || nextW.magnitude > MaxW + 0.00001f ||
                alpha.magnitude > ceiling + 0.001f || alpha.magnitude > stepCap + 0.001f ||
                (directional && torque.magnitude > TorqueBudget + 0.0001f))
                throw new InvalidOperationException("REFERENCE_PLANNER_FINAL_BUDGET_EXCEEDED");
            Quaternion nextRotation = Advance(rotation, nextW, dt);
            return new Plan { Rotation = nextRotation, W = nextW, PreviousW = w, BridgeW = goalW,
                DesiredW = desiredW, LimitedW = limitedW, Alpha = alpha, ErrorAngle = angle,
                RelativeW = relativeW, StoppingOffset = stoppingOffset, ArrivalError = arrivalError,
                BrakeCap = brakeCap, BaseStepCap = baseCap, StepCap = stepCap,
                BrakeInertia = brakeInertia, StepInertia = stepInertia, InertiaTorque = torque, RigidGyro = gyro,
                ExtraBrakeAlpha = extra, SpeedLimited = desiredW.magnitude > MaxW,
                AlphaLimited = extra > 0f ? (limitedW - nextW).magnitude > 0.000001f : deltaW.magnitude > baseCap * dt };
        }

        private static Vector3 RemoveExistingSpeed(Vector3 previous, Vector3 baseline, Vector3 desired, float dt,
            PelvisReferenceInertiaBudget budget, out float extra)
        {
            extra = 0f;
            float speed = previous.magnitude;
            if (speed <= 0.000001f || baseline.sqrMagnitude > previous.sqrMagnitude) return baseline;
            Vector3 axis = previous / speed;
            float remaining = Vector3.Dot(baseline, axis);
            float removed = speed - remaining;
            float requestedRemaining = Vector3.Dot(baseline - desired, axis);
            if (removed <= 0.000001f || remaining <= 0f || requestedRemaining <= 0.000001f) return baseline;
            Vector3 a = (baseline - previous) / dt;
            Vector3 tau = budget.Apply(a), inertiaAxis = budget.Apply(axis);
            float inertiaSq = inertiaAxis.sqrMagnitude;
            if (!Positive(inertiaSq)) throw new InvalidOperationException("REFERENCE_PLANNER_RESERVE_INERTIA_INVALID");
            // Largest extra k with |a-k*axis|<=8 and |I*a-k*I*axis|<=1.
            // Starting from the already feasible baseline keeps both intervals connected to k=0.
            float ad = Vector3.Dot(a, axis), td = Vector3.Dot(tau, inertiaAxis);
            float alphaRoom = ad + Mathf.Sqrt(Mathf.Max(0f, ad * ad + MaxBrakeAlpha * MaxBrakeAlpha - a.sqrMagnitude));
            float torqueRoom = (td + Mathf.Sqrt(Mathf.Max(0f, td * td + inertiaSq * (TorqueBudget * TorqueBudget - tau.sqrMagnitude)))) / inertiaSq;
            float k = Mathf.Max(0f, Mathf.Min(Mathf.Min(remaining, requestedRemaining) / dt, Mathf.Min(alphaRoom, torqueRoom)));
            if (!F(k)) throw new InvalidOperationException("REFERENCE_PLANNER_RESERVE_NONFINITE");
            if (k <= 0.000001f) return baseline;
            // Pure opposition to previous world W: no extra turn-axis acceleration and
            // no spending the reserve on acceleration through zero into a reversal.
            // Do not brake past the ordinary desired-W component: a tiny requested
            // slowdown must not turn into a full-budget brake pulse near steady motion.
            Vector3 result = baseline - axis * (k * dt);
            if (!V(result) || result.sqrMagnitude > baseline.sqrMagnitude + 0.00001f || Vector3.Dot(result, axis) < -0.00001f)
                throw new InvalidOperationException("REFERENCE_PLANNER_RESERVE_DIRECTION_INVALID");
            extra = k;
            return result;
        }

        private static Quaternion Advance(Quaternion q, Vector3 w, float dt)
        {
            float speed = w.magnitude;
            return speed > 0f ? Normalize(Quaternion.AngleAxis(speed * dt * Mathf.Rad2Deg, w / speed) * q) : q;
        }
        private static Quaternion Normalize(Quaternion q)
        {
            float n = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!Positive(n)) throw new InvalidOperationException("REFERENCE_PLANNER_ROTATION_INVALID");
            return new Quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
        }
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        private static bool Positive(float x) => F(x) && x > 0f;
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
    }
}
