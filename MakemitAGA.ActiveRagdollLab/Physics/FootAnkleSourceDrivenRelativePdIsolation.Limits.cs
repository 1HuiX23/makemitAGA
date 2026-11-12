namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleSourceDrivenRelativePdIsolation
    {
        // Frozen controller authority, shared directly by the input planners and
        // their managed tests. Moving these declarations changes no values.
        internal const float AnkleAnchorValidityMeters = 0.030f;
        internal const float FlexionLimitDegrees = 30f;
        internal const float AxialLimitDegrees = 25f;
        internal const float InversionLimitDegrees = 15f;
        internal const float Kp = 24f;
        internal const float Kd = 8f;
        internal const float MaxRelativeAngularAcceleration = 30f;
        internal const float MaxInternalTorqueNm = 0.35f;
        internal const float MaxFreshTargetAgeSeconds = 0.12f;
        internal const float TargetAccelerationFeedforwardGain = 1f;
        internal const float MaxTargetAccelerationFeedforward = 20f;
    }
}
