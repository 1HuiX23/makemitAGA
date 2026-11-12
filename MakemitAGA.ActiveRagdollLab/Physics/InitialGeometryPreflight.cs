using System;
using System.Globalization;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // R2 distinguishes collision-OFF preparation from ordinary contact geometry.
    // A shallow preparation receipt is never permission to enable contact or weight.
    internal struct InitialGeometryPreflight
    {
        internal const float MaxSourceAge = .1f, MaxHipBindingGap = .005f;
        // Raw animation/neutral Box is not a contact permission. The historical
        // ~-2.6 mm was a Bedroom-built -> Main test case, not a universal neutral offset.
        // One-shot foot calibration belongs to setup; never resnap it during a run.
        internal const float MinRawClearance = -.003f, MaxClearance = .015f;
        internal const float MinControlClearance = -FootContactGeometryLimits.OrdinaryPenetrationMeters;
        internal const float MinPreparationClearance = -FootContactGeometryLimits.RecoveryPenetrationMeters;
        internal bool Valid, Calibrated, FootprintsInside;
        internal float SourceAge, HipBindingGap, HipBindingY;
        internal float RawMin, ControlMin, ActualMin;
        internal string Issue => Check(MinControlClearance);
        internal string PreparationIssue => Check(MinPreparationClearance);
        internal string PreparationRoute => PreparationIssue != null ? "REJECTED" :
            Issue == null ? "ORDINARY_ENTRY_CANDIDATE" : "SHALLOW_RECOVERY_REQUIRED";
        private string Check(float minimum)
        {
            if (!Valid || !Finite(SourceAge) || !Finite(HipBindingGap) || !Finite(HipBindingY) ||
                !Finite(RawMin) || !Finite(ControlMin) || !Finite(ActualMin)) return "GEOMETRY_OR_SOURCE_INVALID";
            if (SourceAge < 0f || SourceAge > MaxSourceAge) return "SOURCE_NOT_FRESH";
            if (!Calibrated) return "SUPPORT_PLANE_NOT_CALIBRATED";
            if (HipBindingGap < 0f || HipBindingGap > MaxHipBindingGap) return "HIP_BINDING_OFFSET";
            if (!FootprintsInside) return "FOOTPRINT_OUTSIDE_SUPPORT";
            if (RawMin < MinRawClearance) return "RAW_SOLE_BELOW_START_BAND";
            if (ControlMin < minimum) return "CONTROL_SOLE_BELOW_START_BAND";
            if (ActualMin < minimum) return "ACTUAL_SOLE_BELOW_START_BAND";
            if (RawMin > MaxClearance || ControlMin > MaxClearance || ActualMin > MaxClearance) return "SOLE_ABOVE_START_BAND";
            return null;
        }
        internal string Summary(string side) => "side=" + side + " | preparationPass=" + (PreparationIssue == null) +
            " | issue=" + (PreparationIssue ?? "NONE") + " | route=" + PreparationRoute + " | ordinaryGeometryPass=" + (Issue == null) +
            " | calibrated=" + Calibrated + " | sourceAgeMs=" + F(SourceAge * 1000f) +
            " | hipBindingGap-yMm=" + F(HipBindingGap * 1000f) + "|" + F(HipBindingY * 1000f) +
            " | soleMinMm(raw-control-actual)=" + F(RawMin * 1000f) + "|" + F(ControlMin * 1000f) + "|" + F(ActualMin * 1000f) +
            " | footprintsInside=" + FootprintsInside;
        private static bool Finite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static string F(float value) => value.ToString("R", CultureInfo.InvariantCulture);
    }
}
