using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04430: A keeps the child-only baseline. B first replaces the support-normal Knee
    /// command with reduced-mass relative PD, then this module applies its exact reciprocal.
    /// Parent-acceleration FF remains tangential only in supported B; free space is unchanged.
    /// The reaction always uses the ACTUAL final clamped command, never an unclamped estimate.
    /// COM normal force closure does not prove angular closure or whole-leg weight support.
    /// </summary>
    internal sealed class KneeSupportNormalReactionIsolation
    {
        internal const string Mode = "KNEE_REDUCED_MASS_NORMAL_PD_CAUSAL_AB_V04430";
        internal const string ArmA = "A_V04430_CHILD_ONLY_BASELINE";
        internal const string ArmB = "B_V04430_REDUCED_MASS_NORMAL_PD";
        internal const int MinimumEligibleSamples = 50;
        internal const int MinimumNonzeroSamples = 30;
        internal const int MinimumNonStanceSamples = 30;
        private const float Epsilon = 0.0000001f;
        internal readonly KneeSupportNormalReactionReport Report = new KneeSupportNormalReactionReport();

        internal KneeSupportNormalReactionIsolation() { Select(false); }
        internal bool Enabled => Report.Enabled;
        internal bool Finite => Report.Finite;
        internal string Arm => Report.Arm;
        internal bool Off => !Enabled && Arm == ArmA && !Report.CurrentApplied && Report.CurrentUpperForceWorld == Vector3.zero;
        internal static bool IsExperimentArm(string arm) => arm == ArmA || arm == ArmB;

        // The chain owns the drive/trace preconditions; disabling is always available to cleanup.
        internal void Select(bool enabled)
        {
            Report.Arm = enabled ? ArmB : ArmA;
            Report.Enabled = enabled;
            Report.Finite = true;
            Report.InvalidReason = null;
            Quiesce();
        }

        internal void ResetTrace()
        {
            Report.Finite = true;
            Report.InvalidReason = null;
            Report.NormalConstraintWrites = Report.NearCapWrites = Report.AdjacentSignFlips = 0;
            Report.Samples = Report.Eligible = Report.NonStance = Report.Writes = Report.NonzeroCandidates = 0;
            Report.UpwardWrites = Report.DownwardWrites = Report.FirstWrittenFixed = Report.LastWrittenFixed = 0;
            Report.PeakAbsForce = Report.PeakUpperAcceleration = Report.PeakPairClosureNewtons = 0f;
            Report.CandidateAbsForceSum = Report.AppliedNormalImpulseSum = 0d;
            Report.LastWrittenForceWorld = Vector3.zero;
            Quiesce();
        }

        // AddForce is a one-solve command, not a persistent force. Drive-off makes the bodies kinematic.
        // Clear live output only; immutable completed evidence is built before selector reset.
        internal void Quiesce()
        {
            Report.CurrentFixed = Report.CurrentSourceFixed = 0;
            Report.CurrentNormalConstraint = false;
            Report.CurrentEligible = Report.CurrentApplied = false;
            Report.CurrentLowerMass = Report.CurrentUpperMass = Report.CurrentLowerForceNormal = 0f;
            Report.CurrentNormalWorld = Report.CurrentCandidateUpperForceWorld = Report.CurrentUpperForceWorld = Vector3.zero;
        }

        internal bool Step(bool trace, int fixedSequence, int kneeCommandFixed, bool kneeTetherValid,
            bool exactAuthorizedStance, bool zeroFriction, Rigidbody upper, Rigidbody lower,
            Vector3 kneeAppliedAcceleration, Vector3 supportNormal, float dt, bool normalConstraint)
        {
            int previousFixed = Report.CurrentFixed;
            bool previousApplied = Report.CurrentApplied;
            float previousForce = Report.CurrentLowerForceNormal;
            Quiesce();
            if (!Report.Finite) return false;
            Report.CurrentFixed = fixedSequence;
            Report.CurrentSourceFixed = kneeCommandFixed;
            if (trace) Report.Samples++;
            if (fixedSequence <= 0 || fixedSequence <= previousFixed)
                return Invalid("DUPLICATE_OR_INVALID_FIXED_SEQUENCE");
            if (!exactAuthorizedStance)
            {
                if (trace) Report.NonStance++;
                return true; // No old normal, candidate or actual force survives release/swing.
            }
            if (!zeroFriction || !kneeTetherValid || kneeCommandFixed != fixedSequence || (Enabled && !normalConstraint))
                return Invalid("SUPPORT_OR_KNEE_COMMAND_CONTRACT_INVALID");
            if (upper == null || lower == null || upper == lower || upper.isKinematic || lower.isKinematic || upper.useGravity)
                return Invalid("UPPER_LOWER_BODY_OR_GRAVITY_BOUNDARY_INVALID");
            float lowerMass = lower.mass;
            float upperMass = upper.mass;
            if (!F(lowerMass) || !F(upperMass) || lowerMass <= Epsilon || upperMass <= Epsilon ||
                !F(dt) || dt <= 0f || !V(kneeAppliedAcceleration) || !V(supportNormal) ||
                !F(supportNormal.sqrMagnitude) || supportNormal.sqrMagnitude <= Epsilon)
                return Invalid("NONFINITE_OR_INVALID_REACTION_INPUT");

            Vector3 n = supportNormal.normalized;
            float lowerForceNormal = lowerMass * Vector3.Dot(kneeAppliedAcceleration, n);
            Vector3 upperForce = -lowerForceNormal * n;
            float forceAbs = upperForce.magnitude;
            float upperAcceleration = forceAbs / upperMass;
            float bound = lowerMass * kneeAppliedAcceleration.magnitude;
            float pairClosure = (lowerForceNormal * n + upperForce).magnitude;
            float signedImpulse = Vector3.Dot(upperForce, n) * dt;
            if (!V(n) || Mathf.Abs(n.sqrMagnitude - 1f) > 0.0001f || !F(lowerForceNormal) ||
                !V(upperForce) || !F(forceAbs) || !F(upperAcceleration) || !F(bound) || !F(pairClosure) || !F(signedImpulse))
                return Invalid("NONFINITE_REACTION_COMMAND");
            if (forceAbs > bound + 0.01f || pairClosure > 0.01f)
                return Invalid("REACTION_FORCE_BOUND_OR_PAIR_CLOSURE_INVALID");

            Report.CurrentEligible = true;
            Report.CurrentNormalConstraint = normalConstraint;
            Report.CurrentLowerMass = lowerMass;
            Report.CurrentUpperMass = upperMass;
            Report.CurrentNormalWorld = n;
            Report.CurrentLowerForceNormal = lowerForceNormal;
            Report.CurrentCandidateUpperForceWorld = upperForce;
            if (trace)
            {
                Report.Eligible++;
                if (forceAbs > 0.001f) Report.NonzeroCandidates++;
                Report.CandidateAbsForceSum += forceAbs;
                Report.PeakAbsForce = Mathf.Max(Report.PeakAbsForce, forceAbs);
                Report.PeakUpperAcceleration = Mathf.Max(Report.PeakUpperAcceleration, upperAcceleration);
                Report.PeakPairClosureNewtons = Mathf.Max(Report.PeakPairClosureNewtons, pairClosure);
            }
            if (!Enabled) return true;
            try { upper.AddForce(upperForce, ForceMode.Force); }
            catch (Exception e) { return Invalid("UPPER_ADDFORCE_EXCEPTION:" + e.GetType().Name); }
            Report.CurrentApplied = true;
            Report.CurrentUpperForceWorld = upperForce;
            if (trace)
            {
                Report.Writes++;
                if (normalConstraint) Report.NormalConstraintWrites++;
                if (forceAbs >= lowerMass * KneeCharacterJointResponseIsolation.TetherMaxAcceleration * 0.95f) Report.NearCapWrites++;
                if (previousApplied && previousFixed + 1 == fixedSequence && previousForce * lowerForceNormal < 0f &&
                    Mathf.Abs(previousForce) > 0.001f && forceAbs > 0.001f) Report.AdjacentSignFlips++;
                if (signedImpulse > Epsilon) Report.UpwardWrites++;
                else if (signedImpulse < -Epsilon) Report.DownwardWrites++;
                Report.AppliedNormalImpulseSum += signedImpulse;
                if (Report.FirstWrittenFixed == 0) Report.FirstWrittenFixed = fixedSequence;
                Report.LastWrittenFixed = fixedSequence;
                Report.LastWrittenForceWorld = upperForce;
            }
            return true;
        }

        internal static bool SelectorContract(KneeSupportNormalReactionReport r, string arm) =>
            r != null && r.Finite && IsExperimentArm(arm) && r.Arm == arm && r.Enabled == (arm == ArmB) &&
            (!r.CurrentApplied || (r.Enabled && r.CurrentEligible && r.CurrentFixed == r.CurrentSourceFixed));

        internal static bool ExposureReady(KneeSupportNormalReactionReport r, string arm) =>
            SelectorContract(r, arm) && r.Eligible >= MinimumEligibleSamples &&
            r.NonzeroCandidates >= MinimumNonzeroSamples && r.NonStance >= MinimumNonStanceSamples &&
            (arm == ArmA ? r.Writes == 0 : r.Writes == r.Eligible && r.NormalConstraintWrites == r.Writes);

        private bool Invalid(string reason)
        {
            Report.Finite = false;
            Report.InvalidReason = Report.InvalidReason ?? reason;
            return false;
        }
        private static bool F(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool V(Vector3 value) => F(value.x) && F(value.y) && F(value.z);
    }
}
