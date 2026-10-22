using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // The load owner also records its own COM ledger. Bounded numeric storage; format only on demand.
    internal sealed class UpperSelfWeightSupportReport
    {
        public string Arm, InvalidReason, LedgerIssue;
        public bool Selected, Enabled, Finite = true, GravityReadback, LedgerFinite = true;
        public int Samples, Eligible, NonStance, Writes, ReadbackFailures, RollbackFailures;
        public int CurrentFixed, FirstWrittenFixed, LastWrittenFixed;
        public bool CurrentEligible, CurrentApplied;
        public float CurrentUpperMass, CurrentLowerMass, CurrentFootMass, CurrentExpectedImpulse, PeakForce;
        public Vector3 CurrentNormal, CurrentUpperForce, CurrentLowerKneeForce, CurrentLowerAnkleForce, CurrentFootForce, CurrentGravity;
        public double CandidateImpulseSum, AppliedImpulseSum;
        public int LedgerCaptured, LedgerResolved, LedgerLoaded, LedgerCensored, LedgerDragValid;
        public double HipNormalSum, CommonNormalSum, KneeNormalSum, WeightSupportNormalSum, GravityNormalSum, DeltaNormalSum, ResidualNormalSum;
        public float PeakAbsHipNormal, PeakAbsResidual;
        internal struct UpperFrame
        {
            internal int Fixed, Settled, BodyId;
            internal bool Loaded, Applied, Resolved, Censored, DragValid;
            internal float Mass, Dt;
            internal Vector3 Normal, PreCom, PreV, PostV, HipJ, CommonJ, KneeJ, WeightJ, GravityJ, KnownJ, DragJ, DeltaP, Residual;
        }
        internal readonly UpperFrame[] Ring = new UpperFrame[12];
        internal int Next, Count;

        public float AppliedImpulseMean => Writes > 0 ? (float)(AppliedImpulseSum / Writes) : 0f;
        public string GetSummary() =>
            "arm=" + (Arm ?? "<none>") + "/selected-enabled-finite=" + Selected + "-" + Enabled + "-" + Finite +
            "/reason=" + (InvalidReason ?? "<none>") + "/samples-eligible-nonstance-writes=" + Samples + "-" + Eligible + "-" + NonStance + "-" + Writes +
            "/readback-rollbackFailures=" + ReadbackFailures + "-" + RollbackFailures +
            "/candidateAppliedMeanNs=" + F(Eligible > 0 ? (float)(CandidateImpulseSum / Eligible) : 0f) + "-" + F(AppliedImpulseMean) +
            "/peakAppliedForceN=" + F(PeakForce) + "/first-lastWritten=" + FirstWrittenFixed + "-" + LastWrittenFixed +
            "/current(fixed-eligible-applied-gravityReadback)=" + CurrentFixed + "-" + CurrentEligible + "-" + CurrentApplied + "-" + GravityReadback +
            "/mass(upper-lower-foot)=" + F(CurrentUpperMass) + "-" + F(CurrentLowerMass) + "-" + F(CurrentFootMass) +
            "/currentCandidateJNs=" + F(CurrentExpectedImpulse) +
            "/currentN=" + V(CurrentNormal) + "/actualForce(upper-lowerKnee-lowerAnkle-foot)=" + V(CurrentUpperForce) + "|" + V(CurrentLowerKneeForce) + "|" + V(CurrentLowerAnkleForce) + "|" + V(CurrentFootForce) +
            "/upperLedgerFinite=" + LedgerFinite + "/ledgerIssue=" + (LedgerIssue ?? "<none>") +
            "/captured-resolved-loaded-censored-dragValid=" + LedgerCaptured + "-" + LedgerResolved + "-" + LedgerLoaded + "-" + LedgerCensored + "-" + LedgerDragValid +
            "/loadedNormalSumNs(hip-common-knee-weightSupport-gravity-deltaP-residual)=" + D(HipNormalSum) + "|" + D(CommonNormalSum) + "|" + D(KneeNormalSum) + "|" + D(WeightSupportNormalSum) + "|" + D(GravityNormalSum) + "|" + D(DeltaNormalSum) + "|" + D(ResidualNormalSum) +
            "/peakAbsHipNormal-residualNs=" + F(PeakAbsHipNormal) + "-" + F(PeakAbsResidual) +
            "/scope=EXPLICIT_STATIC_AXIAL_LOAD_PAIRS_AND_UPPER_COM_LEDGER_NOT_ANGULAR_OR_WHOLE_BODY_SUPPORT_VERDICT";

        public string GetDetails()
        {
            var sb = new StringBuilder(GetSummary());
            for (int k = 0; k < Count; k++)
            {
                UpperFrame f = Ring[(Next - Count + k + Ring.Length) % Ring.Length];
                sb.Append("\nupperRow=").Append(f.Fixed).Append('>').Append(f.Settled)
                    .Append("/body-loaded-applied=").Append(f.BodyId).Append('-').Append(f.Loaded).Append('-').Append(f.Applied)
                    .Append("/mass-dt=").Append(F(f.Mass)).Append('-').Append(F(f.Dt)).Append("/preN=").Append(V(f.Normal))
                    .Append("/preSolveCom=").Append(V(f.PreCom))
                    .Append("/v(pre-post)=").Append(V(f.PreV)).Append('|').Append(f.Resolved ? V(f.PostV) : "NA")
                    .Append("/worldJ(hip-common-knee-weightSupport-gravity-known)=").Append(V(f.HipJ)).Append('|').Append(V(f.CommonJ)).Append('|').Append(V(f.KneeJ)).Append('|').Append(V(f.WeightJ)).Append('|').Append(V(f.GravityJ)).Append('|').Append(V(f.KnownJ))
                    .Append("/worldDeltaP-residual=").Append(f.Resolved ? V(f.DeltaP) + "|" + V(f.Residual) : "NA")
                    .Append("/dragModelJ=").Append(f.DragValid ? V(f.DragJ) : "NA")
                    .Append("/end=").Append(f.Resolved ? "RESOLVED" : f.Censored ? "CENSORED_UNRESOLVED" : "PENDING");
            }
            return sb + "\nUpper drag is an ordinary pre-solver damping comparison, not a measured force. Hip/common/Knee inputs remain separate; residual includes unobserved solver/contact effects.";
        }
        private static string F(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static string D(double x) => x.ToString("G17", CultureInfo.InvariantCulture);
        private static string V(Vector3 x) => "(" + F(x.x) + "," + F(x.y) + "," + F(x.z) + ")";
    }
}
