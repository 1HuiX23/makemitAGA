using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Pure reference math; the receiver owns history and validates the previous
    // actually committed Hip receipt. No body writes, measured-A feedback or q^-1.
    internal static class PelvisComReferencePlanner
    {
        internal const float MaxV = 3f, MaxA = 6f;
        internal struct ComPlan
        {
            internal bool ArrivalEnabled, FeedbackEnabled, FeedbackAvailable, SpeedLimited, AccelerationLimited, PhysicalBrakeProtected;
            internal Vector3 Position, Velocity, Acceleration, CandidateA;
            internal Vector3 RelativeV, StoppingOffset, ArrivalError, PreviousRequestA, PreviousAllocatedA;
            internal Vector3 SettledVelocity;
            internal float ProposedReduction, RemovedAcceleration;
        }

        internal static ComPlan Build(Vector3 position, Vector3 velocity, Vector3 goal, Vector3 goalV,
            float dt, bool arrival, bool feedback, Vector3 previousRequest, Vector3 previousAllocated, Vector3 settledV, bool feedbackEnabled)
        {
            if (!F(dt) || dt <= 0f || !V(position) || !V(velocity) || !V(goal) || !V(goalV) ||
                velocity.magnitude > MaxV + 0.001f || (arrival && !V(settledV)) ||
                (feedback && (!arrival || !V(previousRequest) || !V(previousAllocated))))
                throw new InvalidOperationException("COM_REFERENCE_PLANNER_INPUT_INVALID");
            // Incoming goal already denotes this Fixed step: keep the original
            // goal-V prediction, so a constant moving target does not gain v*dt lead.
            Vector3 predicted = position + goalV * dt;
            Vector3 error = goal - predicted;
            Vector3 relativeV = velocity - goalV;
            Vector3 stoppingOffset = arrival ? relativeV * (relativeV.magnitude / (2f * MaxA) + dt) : Vector3.zero;
            Vector3 arrivalError = error - stoppingOffset;
            Vector3 desiredV;
            if (arrival) desiredV = goalV + 4f * arrivalError;
            else
            {
                // Retained scopes keep their existing reference law exactly.
                float distance = error.magnitude;
                float correctionV = Mathf.Min(4f * distance, Mathf.Sqrt(2f * MaxA * distance));
                desiredV = goalV + (distance > 0.000001f ? error * (correctionV / distance) : Vector3.zero);
            }
            Vector3 limitedV = Vector3.ClampMagnitude(desiredV, MaxV);
            Vector3 deltaV = limitedV - velocity;
            Vector3 candidateV = Vector3.ClampMagnitude(velocity + Vector3.ClampMagnitude(deltaV, MaxA * dt), MaxV);
            Vector3 candidateA = (candidateV - velocity) / dt;
            Vector3 nextV = candidateV;
            float proposed = 0f, removed = 0f;
            bool physicalBrakeProtected = false;
            if (feedback && feedbackEnabled)
            {
                // The deficit is known input N-1, not a claimed current capability
                // bound. Remove only a SAME-DIRECTION part of candidate reference A;
                // never flip it, scale existing V by q, or carry a drive deficit into
                // an opposite braking request. Protect both existing reference V and
                // settled physical COM V: the reference can reverse before the body.
                // This preserves nominal braking; the same-step allocator still owns
                // feasibility and may limit the actually committed response.
                // New-state PD+AFF sensitivity is
                // d(commandA)/d(referenceA)=1+kd*dt+kp*dt^2 for semi-implicit p/v.
                Vector3 deficit = previousRequest - previousAllocated;
                float magnitude = deficit.magnitude;
                if (!V(deficit)) throw new InvalidOperationException("COM_REFERENCE_DEFICIT_INVALID");
                if (magnitude > 0.000001f)
                {
                    Vector3 axis = deficit / magnitude;
                    float sensitivity = 1f + PelvisHipCommandFrame.VelocityGain * dt + PelvisHipCommandFrame.PositionGain * dt * dt;
                    proposed = Vector3.Dot(velocity, axis) >= 0f ?
                        Mathf.Min(Mathf.Max(0f, Vector3.Dot(candidateA, axis)), magnitude / sensitivity) : 0f;
                    physicalBrakeProtected = proposed > 0f && Vector3.Dot(settledV, axis) < 0f;
                    removed = physicalBrakeProtected ? 0f : proposed;
                    if (removed > 0f) nextV = Vector3.ClampMagnitude(candidateV - axis * (removed * dt), MaxV);
                }
            }
            Vector3 acceleration = (nextV - velocity) / dt;
            Vector3 nextPosition = position + nextV * dt;
            if (!V(stoppingOffset) || !V(arrivalError) || !V(desiredV) || !V(candidateA) ||
                !V(nextPosition) || !V(nextV) || !V(acceleration) || !F(removed) ||
                nextV.magnitude > MaxV + 0.00001f || acceleration.magnitude > MaxA + 0.001f)
                throw new InvalidOperationException("COM_REFERENCE_PLANNER_FINAL_INVALID");
            // Receipt validity and policy are separate: B still validates and records
            // the real completion but does not use its deficit to slow this reference.
            return new ComPlan { ArrivalEnabled = arrival, FeedbackEnabled = arrival && feedbackEnabled, FeedbackAvailable = feedback,
                Position = nextPosition, Velocity = nextV, Acceleration = acceleration, CandidateA = candidateA,
                RelativeV = relativeV, StoppingOffset = stoppingOffset, ArrivalError = arrivalError,
                PreviousRequestA = feedback ? previousRequest : Vector3.zero,
                PreviousAllocatedA = feedback ? previousAllocated : Vector3.zero,
                SettledVelocity = arrival ? settledV : Vector3.zero, ProposedReduction = proposed,
                PhysicalBrakeProtected = physicalBrakeProtected,
                RemovedAcceleration = removed, SpeedLimited = desiredV.magnitude > MaxV,
                AccelerationLimited = deltaV.magnitude > MaxA * dt || removed > 0f };
        }
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    }
}
