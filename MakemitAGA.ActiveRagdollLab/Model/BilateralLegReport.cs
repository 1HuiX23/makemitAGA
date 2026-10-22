using System.Text;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    /// <summary>
    /// v0.3.0 bilateral foundation report. This is deliberately a coordinator-level report:
    /// it never feeds either leg controller and only verifies that two frozen Single-Leg instances
    /// stay lifecycle/clock-consistent while sharing the same frozen Two-Body parent.
    /// </summary>
    internal sealed class BilateralLegReport
    {
        public string Mode;
        public bool Built;
        public bool BilateralEnabled;
        public bool DistinctSourceChains;
        public string PrimarySide;
        public string SecondarySide;
        public bool PrimaryDriveEnabled;
        public bool SecondaryDriveEnabled;
        public bool TraceEnabled;

        public int ClockSamples;
        public int FixedSequenceMismatchSamples;
        public int RenderSequenceMismatchSamples;
        public int RenderFreshnessMismatchSamples;
        public int RenderHeldDepthMismatchSamples;
        public int DriveStateMismatchSamples;
        public int MaxFixedSequenceDelta;
        public int MaxRenderSequenceDelta;
        public int MaxHeldDepthDelta;
        public int SimultaneousHipClampSamples;
        public int SimultaneousKneeClampSamples;
        public int FailClosedEvents;
        public string LastFailClosedReason;

        public int PrimaryTraceSamples;
        public int SecondaryTraceSamples;
        public float PrimaryHipMeanDegrees;
        public float SecondaryHipMeanDegrees;
        public float PrimaryHipMaxDegrees;
        public float SecondaryHipMaxDegrees;
        public float PrimaryKneeMeanDegrees;
        public float SecondaryKneeMeanDegrees;
        public float PrimaryKneeMaxDegrees;
        public float SecondaryKneeMaxDegrees;
        public float PrimaryKneeOffAxisMeanDegrees;
        public float SecondaryKneeOffAxisMeanDegrees;
        public float PrimaryPeakHipAnchorMeters;
        public float SecondaryPeakHipAnchorMeters;
        public float PrimaryPeakKneeAnchorMeters;
        public float SecondaryPeakKneeAnchorMeters;
        public bool PrimarySingleLegFreezeReady;
        public bool SecondarySingleLegFreezeReady;
        public bool PrimaryDriveAborted;
        public bool SecondaryDriveAborted;
        public string PrimaryDriveAbortReason;
        public string SecondaryDriveAbortReason;

        public bool ClockParityPass;
        public bool FrozenReplicationPass;
        public bool FoundationReady;
        public string Reason;
        public string Key;

        public string GetConsoleSummary()
        {
            return
                "ARL bilateral foundation" +
                " | mode=" + (Mode ?? "<none>") +
                " | built=" + Built +
                " | enabled=" + BilateralEnabled +
                " | sides=" + (PrimarySide ?? "<none>") + "/" + (SecondarySide ?? "<none>") +
                " | distinct=" + DistinctSourceChains +
                " | drive=" + PrimaryDriveEnabled + "/" + SecondaryDriveEnabled +
                " | trace=" + TraceEnabled +
                " | clockSamples=" + ClockSamples +
                " | mismatch(fixed/render/fresh/held/drive)=" +
                FixedSequenceMismatchSamples + "/" + RenderSequenceMismatchSamples + "/" +
                RenderFreshnessMismatchSamples + "/" + RenderHeldDepthMismatchSamples + "/" + DriveStateMismatchSamples +
                " | maxDelta(fixed/render/held)=" + MaxFixedSequenceDelta + "/" + MaxRenderSequenceDelta + "/" + MaxHeldDepthDelta +
                " | simultaneousClamp(H/K)=" + SimultaneousHipClampSamples + "/" + SimultaneousKneeClampSamples +
                " | failClosed=" + FailClosedEvents + "/" + (LastFailClosedReason ?? "<none>") +
                " | traceSamples=" + PrimaryTraceSamples + "/" + SecondaryTraceSamples +
                " | hipMean=" + F(PrimaryHipMeanDegrees) + "/" + F(SecondaryHipMeanDegrees) +
                " | kneeMean=" + F(PrimaryKneeMeanDegrees) + "/" + F(SecondaryKneeMeanDegrees) +
                " | kneeOffMean=" + F(PrimaryKneeOffAxisMeanDegrees) + "/" + F(SecondaryKneeOffAxisMeanDegrees) +
                " | peakAnchorH=" + F(PrimaryPeakHipAnchorMeters) + "/" + F(SecondaryPeakHipAnchorMeters) +
                " | peakAnchorK=" + F(PrimaryPeakKneeAnchorMeters) + "/" + F(SecondaryPeakKneeAnchorMeters) +
                " | freeze=" + PrimarySingleLegFreezeReady + "/" + SecondarySingleLegFreezeReady +
                " | aborted=" + PrimaryDriveAborted + "/" + SecondaryDriveAborted +
                " | clockParity=" + ClockParityPass +
                " | replication=" + FrozenReplicationPass +
                " | ready=" + FoundationReady +
                " | reason=" + (Reason ?? "<none>");
        }

        public string GetFullText()
        {
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("MakemitAGA Active Ragdoll Lab 0.4.2.1 - Bilateral Frozen Foundation Report");
            sb.AppendLine(GetConsoleSummary());
            sb.AppendLine("key=" + (Key ?? "<none>"));
            sb.AppendLine("primaryAbortReason=" + (PrimaryDriveAbortReason ?? "<none>"));
            sb.AppendLine("secondaryAbortReason=" + (SecondaryDriveAbortReason ?? "<none>"));
            sb.AppendLine("hipMax(primary/secondary)=" + F(PrimaryHipMaxDegrees) + "/" + F(SecondaryHipMaxDegrees));
            sb.AppendLine("kneeMax(primary/secondary)=" + F(PrimaryKneeMaxDegrees) + "/" + F(SecondaryKneeMaxDegrees));
            return sb.ToString();
        }

        private static string F(float v) => v.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
