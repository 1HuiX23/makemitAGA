using System.Globalization;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed partial class SingleLegChainReport
    {
        public string KneeFlexionAllocationMode;
        public string KneeFlexionAllocationArm;
        public bool KneeFlexionAllocationEnabled;
        public int KneeFlexionAllocationSteps;
        public int KneeFlexionAllocationActiveSteps;
        public int KneeFlexionAllocationInvalidSteps;
        public int KneeFlexionAllocationOpposingSteps;
        public int KneeFlexionAllocationSuppressedOpposingFfSteps;
        public int KneeFlexionAllocationSameDirectionSteps;
        public int KneeFlexionAllocationOriginalWouldClampSteps;
        public int KneeFlexionAllocationFinalClampSteps;
        public float KneeFlexionAllocationCurrentPd;
        public float KneeFlexionAllocationCurrentInputFf;
        public float KneeFlexionAllocationCurrentEffectiveFf;
        public float KneeFlexionAllocationCurrentOriginalRaw;
        public float KneeFlexionAllocationCurrentAllocatedRaw;
        public float KneeFlexionAllocationCurrentApplied;
        public float KneeFlexionAllocationCurrentMaxA;
        public bool KneeFlexionAllocationCurrentOpposing;
        public bool KneeFlexionAllocationCurrentOriginalWouldClamp;
        public bool KneeFlexionAllocationCurrentFinalClamped;
        public float KneeFlexionAllocationPeakPdAbs;
        public float KneeFlexionAllocationPeakInputFfAbs;
        public float KneeFlexionAllocationPeakOriginalRawAbs;
        public float KneeFlexionAllocationPeakAllocatedRawAbs;
        public float KneeFlexionAllocationPeakAppliedAbs;
        public float KneeFlexionAllocationSuppressedFfAbsMean;

        private bool _allocationLastInvalid;
        private int _allocationTraceClamped;
        private float _allocationTraceMean, _allocationTraceMax;

        // Preserve the publication instant. Other trace counters can advance before status is read.
        internal void CaptureKneeAllocationTextInputs(bool invalid)
        {
            _allocationLastInvalid = invalid;
            _allocationTraceClamped = TraceKneeClampSamples;
            _allocationTraceMean = TraceKneeRotationMeanDegrees;
            _allocationTraceMax = TraceKneeRotationMaxDegrees;
        }
        public string KneeFlexionAllocationKey => KneeFlexionAllocationMode == null ? null : FormatKneeAllocationKey();
        public string KneeFlexionAllocationLastEvidence => KneeFlexionAllocationMode == null ? null :
            KneeFlexionAllocationSteps == 0 ? "<none>" : _allocationLastInvalid ? "invalid=True/step=" + KneeFlexionAllocationSteps :
                "step=" + KneeFlexionAllocationSteps +
                "/opposing=" + KneeFlexionAllocationCurrentOpposing +
                "/pd=" + AllocationValue(KneeFlexionAllocationCurrentPd) +
                "/ffIn=" + AllocationValue(KneeFlexionAllocationCurrentInputFf) +
                "/ffEff=" + AllocationValue(KneeFlexionAllocationCurrentEffectiveFf) +
                "/rawA=" + AllocationValue(KneeFlexionAllocationCurrentOriginalRaw) +
                "/allocatedA=" + AllocationValue(KneeFlexionAllocationCurrentAllocatedRaw) +
                "/appliedA=" + AllocationValue(KneeFlexionAllocationCurrentApplied) +
                "/maxA=" + AllocationValue(KneeFlexionAllocationCurrentMaxA) +
                "/clamp=" + KneeFlexionAllocationCurrentFinalClamped;

        private string FormatKneeAllocationKey()
        {
            string primary;
            if (!KneeFlexionAllocationEnabled)
                primary = "A_V077_SUM_THEN_CLAMP";
            else if (KneeFlexionAllocationInvalidSteps > 0)
                primary = "B_INVALID_NON_FINITE_ALLOCATION_INPUT";
            else if (KneeFlexionAllocationActiveSteps == 0)
                primary = "B_NO_EXPOSURE";
            else if (KneeFlexionAllocationOpposingSteps == 0)
                primary = "B_NO_OPPOSING_AFF_EXPOSURE";
            else
                primary = "B_FEEDBACK_PRIORITY_ACTIVE_FROZEN";

            return "primary=" + primary +
                   " | mode=" + KneeFlexionAllocationMode +
                   " | arm=" + KneeFlexionAllocationArm +
                   " | steps(active/total/invalid)=" + KneeFlexionAllocationActiveSteps + "/" + KneeFlexionAllocationSteps + "/" + KneeFlexionAllocationInvalidSteps +
                   " | direction(opposing/suppressed/same)=" + KneeFlexionAllocationOpposingSteps + "/" + KneeFlexionAllocationSuppressedOpposingFfSteps + "/" + KneeFlexionAllocationSameDirectionSteps +
                   " | clamp(original/final)=" + KneeFlexionAllocationOriginalWouldClampSteps + "/" + KneeFlexionAllocationFinalClampSteps +
                   " | peak(PD/ffIn/raw/allocated/applied)=" + AllocationValue(KneeFlexionAllocationPeakPdAbs) + "/" + AllocationValue(KneeFlexionAllocationPeakInputFfAbs) + "/" + AllocationValue(KneeFlexionAllocationPeakOriginalRawAbs) + "/" + AllocationValue(KneeFlexionAllocationPeakAllocatedRawAbs) + "/" + AllocationValue(KneeFlexionAllocationPeakAppliedAbs) +
                   " | suppressedFfMean=" + AllocationValue(KneeFlexionAllocationSuppressedFfAbsMean) +
                   " | knee(clamp/mean/max)=" +
                   _allocationTraceClamped + "/" +
                   AllocationValue(_allocationTraceMean) + "/" +
                   AllocationValue(_allocationTraceMax);
        }

        private static string AllocationValue(float v) => v.ToString("0.######", CultureInfo.InvariantCulture);
    }
}
