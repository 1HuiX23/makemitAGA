using System.Globalization;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // One bounded accumulator per leg. Formatting is on demand, never in the FixedUpdate writer.
    internal sealed class KneeSupportNormalReactionReport
    {
        public string Arm;
        public bool Enabled, Finite = true;
        public string InvalidReason;
        public int Samples, Eligible, NonStance, Writes, NonzeroCandidates, UpwardWrites, DownwardWrites;
        public int CurrentFixed, CurrentSourceFixed, FirstWrittenFixed, LastWrittenFixed;
        public bool CurrentEligible, CurrentApplied, CurrentNormalConstraint;
        public int NormalConstraintWrites, NearCapWrites, AdjacentSignFlips;
        public float CurrentLowerMass, CurrentUpperMass, CurrentLowerForceNormal;
        public Vector3 CurrentNormalWorld, CurrentCandidateUpperForceWorld, CurrentUpperForceWorld;
        public Vector3 LastWrittenForceWorld;
        public float PeakAbsForce, PeakUpperAcceleration, PeakPairClosureNewtons;
        public double CandidateAbsForceSum, AppliedNormalImpulseSum;

        public string GetSummary() =>
            "arm=" + (Arm ?? "<none>") + "/enabled=" + Enabled + "/finite=" + Finite +
            "/reason=" + (InvalidReason ?? "<none>") +
            "/samples-eligible-nonstance-writes-nonzero=" + Samples + "-" + Eligible + "-" + NonStance + "-" + Writes + "-" + NonzeroCandidates +
            "/normalConstraintWrites-nearCapWrites-adjacentSignFlips=" + NormalConstraintWrites + "-" + NearCapWrites + "-" + AdjacentSignFlips +
            "/currentNormalConstraint=" + CurrentNormalConstraint +
            "/upperWrites(up-down)=" + UpwardWrites + "-" + DownwardWrites +
            "/candidateForceAbsMeanPeakN=" + F(Eligible > 0 ? (float)(CandidateAbsForceSum / Eligible) : 0f) + "-" + F(PeakAbsForce) +
            "/candidateUpperPeakA=" + F(PeakUpperAcceleration) + "/candidatePairClosurePeakN=" + F(PeakPairClosureNewtons) +
            "/appliedNormalImpulseSumNs=" + AppliedNormalImpulseSum.ToString("G17", CultureInfo.InvariantCulture) +
            "/current(fixed-source-eligible-applied)=" + CurrentFixed + "-" + CurrentSourceFixed + "-" + CurrentEligible + "-" + CurrentApplied +
            "/currentMass(lower-upper)=" + F(CurrentLowerMass) + "-" + F(CurrentUpperMass) +
            "/currentN=" + V(CurrentNormalWorld) + "/lowerForceN=" + F(CurrentLowerForceNormal) +
            "/candidateUpperF=" + V(CurrentCandidateUpperForceWorld) + "/actualUpperF=" + V(CurrentUpperForceWorld) +
            "/first-lastWrittenFixed=" + FirstWrittenFixed + "-" + LastWrittenFixed + "/lastWrittenUpperF=" + V(LastWrittenForceWorld) +
            "/scope=KNEE_COM_SUPPORT_NORMAL_COMMAND_PAIR_ONLY_NOT_FULL_BODY_MOMENTUM_OR_GRF";

        private static string F(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
