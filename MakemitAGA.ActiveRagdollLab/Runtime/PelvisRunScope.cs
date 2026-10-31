namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Session identity is independent of reference A/B. Contact is never inferred from a selector side effect.
    internal enum PelvisRunScope
    {
        TorsoPairOnly,
        DynamicLegFreeSpace,
        DynamicLegUnloadedContact,
        DynamicLegFootSelfWeightContact,
        DynamicLegLowerSelfWeightContact,
        DynamicLegKneeNormalCouplingContact,
        DynamicLegUpperSelfWeightContact,
        DynamicLegHipAnchorCouplingContact
    }

    // Shared identity only; each native owner still validates its own physical state.
    internal static class PelvisRunScopes
    {
        internal static bool IsContact(PelvisRunScope scope) => scope == PelvisRunScope.DynamicLegUnloadedContact ||
            scope == PelvisRunScope.DynamicLegFootSelfWeightContact || HasLowerWeight(scope);
        internal static bool HasFootWeight(PelvisRunScope scope) => scope == PelvisRunScope.DynamicLegFootSelfWeightContact ||
            HasLowerWeight(scope);
        internal static bool HasLowerWeight(PelvisRunScope scope) => scope == PelvisRunScope.DynamicLegLowerSelfWeightContact || HasKneeCoupling(scope);
        internal static bool HasKneeCoupling(PelvisRunScope scope) => scope == PelvisRunScope.DynamicLegKneeNormalCouplingContact || HasUpperWeight(scope);
        internal static bool HasUpperWeight(PelvisRunScope scope) => scope == PelvisRunScope.DynamicLegUpperSelfWeightContact || HasHipCoupling(scope);
        internal static bool HasHipCoupling(PelvisRunScope scope) => scope == PelvisRunScope.DynamicLegHipAnchorCouplingContact;
        internal static string Name(PelvisRunScope scope) => HasHipCoupling(scope) ? "DYNAMIC_PELVIS_LEGS_HIP_ANCHOR_PAIR_CONTACT" : HasUpperWeight(scope) ? "DYNAMIC_PELVIS_LEGS_UPPER_SELF_WEIGHT_CONTACT" : HasKneeCoupling(scope)
            ? "DYNAMIC_PELVIS_LEGS_KNEE_NORMAL_COUPLING_CONTACT" : scope == PelvisRunScope.DynamicLegLowerSelfWeightContact
            ? "DYNAMIC_PELVIS_LEGS_LOWER_SELF_WEIGHT_CONTACT" : scope == PelvisRunScope.DynamicLegFootSelfWeightContact
            ? "DYNAMIC_PELVIS_LEGS_FOOT_SELF_WEIGHT_CONTACT" : scope == PelvisRunScope.DynamicLegUnloadedContact
            ? "DYNAMIC_PELVIS_LEGS_UNLOADED_CONTACT" : scope == PelvisRunScope.DynamicLegFreeSpace
            ? "DYNAMIC_PELVIS_LEGS_FREE_SPACE" : "TORSO_PAIR_ONLY";
        internal static string Command(PelvisRunScope scope) => HasHipCoupling(scope) ? "arl_pelvis_hip_test" : HasUpperWeight(scope) ? "arl_pelvis_upper_load_test" : HasKneeCoupling(scope)
            ? "arl_pelvis_knee_test" : scope == PelvisRunScope.DynamicLegLowerSelfWeightContact
            ? "arl_pelvis_lower_load_test" : scope == PelvisRunScope.DynamicLegFootSelfWeightContact
            ? "arl_pelvis_foot_load_test" : scope == PelvisRunScope.DynamicLegUnloadedContact
            ? "arl_pelvis_contact_test" : scope == PelvisRunScope.DynamicLegFreeSpace ? "arl_pelvis_leg_test" : "arl_pelvis_test";
        internal static string Dataset(PelvisRunScope scope) => HasHipCoupling(scope) ? "DYNAMIC_HIP_ANCHOR_PAIR_CONTACT_DATASET_READY" : HasUpperWeight(scope) ? "DYNAMIC_UPPER_SELF_WEIGHT_CONTACT_DATASET_READY" : HasKneeCoupling(scope)
            ? "DYNAMIC_KNEE_NORMAL_COUPLING_CONTACT_DATASET_READY" : scope == PelvisRunScope.DynamicLegLowerSelfWeightContact
            ? "DYNAMIC_LOWER_SELF_WEIGHT_CONTACT_DATASET_READY" : scope == PelvisRunScope.DynamicLegFootSelfWeightContact
            ? "DYNAMIC_FOOT_SELF_WEIGHT_CONTACT_DATASET_READY" : scope == PelvisRunScope.DynamicLegUnloadedContact
            ? "DYNAMIC_UNLOADED_CONTACT_DATASET_READY" : "DYNAMIC_LEG_FREE_SPACE_DATASET_READY";
    }
}
