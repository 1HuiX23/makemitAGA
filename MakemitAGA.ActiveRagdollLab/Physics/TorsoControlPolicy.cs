namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Selection truth belongs to the receiver owner. Reports copy this value for
    // display/evidence; editing a report must not select a different controller policy.
    internal readonly struct TorsoControlPolicy
    {
        internal bool UseDynamicTorsoRecovery { get; }
        internal bool HipGoalGateOnlyHip { get; }
        internal bool UseCoherentHipNormalReference { get; }
        internal bool HipNormalSampleAgePrediction { get; }

        private TorsoControlPolicy(bool dynamicLeg, bool predictSampleAge)
        {
            UseDynamicTorsoRecovery = dynamicLeg;
            HipGoalGateOnlyHip = dynamicLeg;
            UseCoherentHipNormalReference = dynamicLeg;
            HipNormalSampleAgePrediction = predictSampleAge;
        }

        internal static TorsoControlPolicy FromReceiverSelection(bool success, string arm)
        {
            bool dynamicLeg = success && PelvisDynamicReceiverIsolation.IsDynamicLegArm(arm);
            return new TorsoControlPolicy(dynamicLeg,
                dynamicLeg && arm == PelvisDynamicReceiverIsolation.DynamicLegArmA);
        }
    }
}
