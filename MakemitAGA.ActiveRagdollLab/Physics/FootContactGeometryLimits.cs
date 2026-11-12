namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Shared geometry/quietness bounds. Preparation does not bypass the lifecycle's
    // confirmed strong stance, exact identity, hold or load-admission checks.
    internal static class FootContactGeometryLimits
    {
        internal const float OrdinaryPenetrationMeters = .002f;
        internal const float RecoveryPenetrationMeters = .003f;
        internal const float RecoveryHoldSeconds = .15f;
        internal const float RecoveryMaxGapMeters = .025f;
        internal const float RecoveryMaxCornerNormalSpeed = .03f;
        internal const float RecoveryMaxPlanarSpeed = .05f;
        internal const float RecoveryMaxAngularSpeed = .5f;
        internal const float RecoveryMaxAnchorGapMeters = .005f;
        internal const float RecoveryMaxSourceAgeSeconds = .1f;
    }

    // Physical motion only. Neither this value nor QuietIssue grants contact.
    internal struct FootContactMotion
    {
        internal bool Valid, FullFootprint;
        internal float Min, PredictedMin, CornerNormalSpeed, PlanarSpeed, AngularSpeed, AnchorGap, SourceAge;
        internal string QuietIssue
        {
            get
            {
                if (!Valid || !F(Min) || !F(PredictedMin) || !F(CornerNormalSpeed) || !F(PlanarSpeed) ||
                    !F(AngularSpeed) || !F(AnchorGap) || !F(SourceAge)) return "MOTION_INVALID";
                if (!FullFootprint) return "FOOTPRINT_OUTSIDE_SUPPORT";
                if (SourceAge < 0f || SourceAge > FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds) return "SOURCE_NOT_FRESH";
                if (AnchorGap < 0f || AnchorGap > FootContactGeometryLimits.RecoveryMaxAnchorGapMeters) return "ANKLE_GAP";
                if (Min < -FootContactGeometryLimits.RecoveryPenetrationMeters || Min > FootContactGeometryLimits.RecoveryMaxGapMeters) return "SOLE_OUTSIDE_PREPARATION_BAND";
                if (PredictedMin < -FootContactGeometryLimits.RecoveryPenetrationMeters) return "PREDICTED_SOLE_TOO_LOW";
                if (CornerNormalSpeed < 0f || CornerNormalSpeed > FootContactGeometryLimits.RecoveryMaxCornerNormalSpeed) return "CORNER_MOVING";
                if (PlanarSpeed < 0f || PlanarSpeed > FootContactGeometryLimits.RecoveryMaxPlanarSpeed) return "PLANAR_MOVING";
                if (AngularSpeed < 0f || AngularSpeed > FootContactGeometryLimits.RecoveryMaxAngularSpeed) return "ROTATING";
                return null;
            }
        }
        private static bool F(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
