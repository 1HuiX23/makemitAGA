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
            internal SupportReferenceContext SupportInput;
            internal bool SupportFloorSelected;
            internal bool SupportNormalPrioritySelected, SupportTangentBrakingPreserved;
            internal bool SupportBrakingSelected;
            internal float SupportBrakingMinimumY, SupportBrakingExtraY, SupportBrakingBaselineResidual, SupportBrakingFinalResidual;
            internal float SupportFloorY, NominalCandidateY, SupportCandidateDeltaY, SupportRemainingUnderY;
            internal float SupportPriorityExtraY, SupportTangentDeltaV, SupportCandidateUnderY;
            internal bool ArrivalEnabled, FeedbackEnabled, FeedbackAvailable, SpeedLimited, AccelerationLimited, PhysicalBrakeProtected;
            internal bool EnhancedNormalFeedback;
            internal Vector3 Position, Velocity, Acceleration, CandidateA;
            internal Vector3 RelativeV, StoppingOffset, ArrivalError, PreviousRequestA, PreviousAllocatedA;
            internal Vector3 SettledVelocity;
            internal float ProposedReduction, RemovedAcceleration;
        }

        internal static ComPlan Build(Vector3 position, Vector3 velocity, Vector3 goal, Vector3 goalV,
            float dt, bool arrival, bool feedback, Vector3 previousRequest, Vector3 previousAllocated, Vector3 settledV, bool feedbackEnabled, SupportReferenceContext support = default,
            bool supportFloor = false, float supportFloorY = 0f, bool supportNormalPriority = false, bool enhancedNormalFeedback = false,
            bool supportBraking = false, float? transitionFloorY = null)
        {
            if (!F(dt) || dt <= 0f || !V(position) || !V(velocity) || !V(goal) || !V(goalV) ||
                velocity.magnitude > MaxV + 0.001f || (arrival && !V(settledV)) ||
                (feedback && (!arrival || !V(previousRequest) || !V(previousAllocated))) || (enhancedNormalFeedback && !arrival))
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
            float nominalCandidateY = candidateV.y;
            float priorityExtraY = 0f, tangentDeltaV = 0f;
            bool tangentBrakingPreserved = false;
            float brakingMinimumY = 0f, brakingExtraY = 0f, brakingBaselineResidual = 0f;
            if (supportFloor)
            {
                if (!arrival || !support.Matches(support.ForStep) || support.ForStep.Dt != dt)
                    throw new InvalidOperationException("SUPPORT_REFERENCE_CONTEXT_INVALID");
                Vector3 unshaped = candidateV;
                Vector3 remainder = SupportNormalReference.RaiseCandidate(position, velocity, candidateV, supportFloorY, dt, MaxV, MaxA);
                candidateV = supportNormalPriority ? SupportNormalReference.AllocateNormalPriority(position, velocity, candidateV,
                    supportFloorY, dt, MaxV, MaxA, out tangentBrakingPreserved) : remainder;
                brakingMinimumY = SupportNormalBraking.MinimumVelocity(position.y, supportFloorY, dt, MaxA);
                brakingBaselineResidual = SupportNormalBraking.Residual(position.y + candidateV.y * dt, candidateV.y, supportFloorY, MaxA);
                if (supportBraking)
                {
                    // Compare the new candidate with the unchanged one-step floor
                    // candidate from this SAME input. Both still precede feedback.
                    Vector3 original = candidateV;
                    remainder = SupportNormalReference.RaiseCandidate(position, velocity, unshaped, supportFloorY, dt, MaxV, MaxA, brakingMinimumY);
                    candidateV = supportNormalPriority ? SupportNormalReference.AllocateNormalPriority(position, velocity, unshaped,
                        supportFloorY, dt, MaxV, MaxA, out tangentBrakingPreserved, brakingMinimumY) : remainder;
                    brakingExtraY = candidateV.y - original.y;
                }
                priorityExtraY = candidateV.y - remainder.y;
                tangentDeltaV = new Vector3(candidateV.x - remainder.x, 0f, candidateV.z - remainder.z).magnitude;
            }
            // The turn comparison consumes only the remaining vector budget. The real support
            // fields stay empty and the original deficit feedback still runs on the candidate.
            if (transitionFloorY.HasValue)
            {
                if (!arrival || supportFloor || support.StanceMask != 0)
                    throw new InvalidOperationException("TURN_HEIGHT_REFERENCE_CONTEXT_INVALID");
                float floor = transitionFloorY.Value;
                float minimum = SupportNormalBraking.MinimumVelocity(position.y, floor, dt, MaxA);
                candidateV = SupportNormalReference.RaiseCandidate(position, velocity, candidateV, floor, dt, MaxV, MaxA, minimum);
            }
            // Preserve the original deficit/braking policy AFTER candidate shaping.
            // Any remaining undershoot is evidence, never permission to snap or raise a cap.
            float supportCandidateDeltaY = candidateV.y - nominalCandidateY;
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
                    float sensitivity = PelvisComFeedback.Sensitivity(dt, false);
                    float referenceDeficitMagnitude = magnitude / sensitivity;
                    if (enhancedNormalFeedback)
                    {
                        // Current Hip arms share the v124 enhanced normal sensitivity.
                        // This is the nominal PD+AFF input back-calculation; request
                        // saturation and the v137 recovery envelope are nonlinear.
                        // Keep actual receipt deficits and the original bounded removal.
                        Vector3 referenceDeficit = PelvisComFeedback.ReferenceDeficit(deficit, dt, true);
                        referenceDeficitMagnitude = referenceDeficit.magnitude;
                        if (!V(referenceDeficit) || !F(referenceDeficitMagnitude) || referenceDeficitMagnitude <= 0f)
                            throw new InvalidOperationException("COM_REFERENCE_DEFICIT_MAP_INVALID");
                        axis = referenceDeficit / referenceDeficitMagnitude;
                    }
                    proposed = Vector3.Dot(velocity, axis) >= 0f ?
                        Mathf.Min(Mathf.Max(0f, Vector3.Dot(candidateA, axis)), referenceDeficitMagnitude) : 0f;
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
            // Receipt validity and feedback selection remain separate. Current Hip
            // arms both retain feedback; support shaping never bypasses that policy.
            return new ComPlan { EnhancedNormalFeedback = enhancedNormalFeedback, SupportInput = support, SupportFloorSelected = supportFloor,
                SupportBrakingSelected = supportFloor && supportBraking, SupportBrakingMinimumY = brakingMinimumY,
                SupportBrakingExtraY = brakingExtraY, SupportBrakingBaselineResidual = brakingBaselineResidual,
                SupportBrakingFinalResidual = supportFloor ? SupportNormalBraking.Residual(nextPosition.y, nextV.y, supportFloorY, MaxA) : 0f,
                SupportNormalPrioritySelected = supportFloor && supportNormalPriority, SupportTangentBrakingPreserved = tangentBrakingPreserved,
                SupportFloorY = supportFloorY, NominalCandidateY = nominalCandidateY,
                SupportCandidateDeltaY = supportCandidateDeltaY,
                SupportPriorityExtraY = priorityExtraY, SupportTangentDeltaV = tangentDeltaV,
                SupportCandidateUnderY = support.Matches(support.ForStep) ? Mathf.Max(0f, supportFloorY - (position.y + candidateV.y * dt)) : 0f,
                SupportRemainingUnderY = support.Matches(support.ForStep) ? Mathf.Max(0f, supportFloorY - nextPosition.y) : 0f,
                ArrivalEnabled = arrival, FeedbackEnabled = arrival && feedbackEnabled, FeedbackAvailable = feedback,
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
