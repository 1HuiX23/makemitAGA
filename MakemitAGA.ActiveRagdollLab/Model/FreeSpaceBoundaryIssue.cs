namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Current no-contact/no-load authorization boundary, not a loaded-support readiness verdict.
    // Owners return typed first failures; status text is never parsed to make a control decision.
    internal enum FreeSpaceBoundaryIssue
    {
        None, MissingFoundation, ContactAuthorization, Friction, GroundAuthority,
        PredictiveCapture, FootGravity, CollisionMode, ContactOffset, Solver,
        SupportAngular, PlanarDamping, LowerGravity, ReciprocalLoad,
        LowerSelfWeight, UpperSelfWeight, KneeReaction, HipTranslationPair
    }
}
