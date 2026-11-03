using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Optional, same-Fixed receipt of the consumed Hip world goal and Knee relative goal.
    // These are rotation commands. A closed-chain ankle reconstructed from them is NOT
    // an independently commanded position, an IK solution or a native solver prediction.
    internal struct LegLandingReferenceInput
    {
        internal bool Prepared;
        internal PhysicsStepContext Step;
        internal SourceStepContext Source;
        internal int Fixed;
        internal Quaternion UpperGoal, KneeRelativeGoal, Upper;
        internal Vector3 ParentHip, UpperHip, UpperHipToKnee, LowerKneeAnchor;
        // Optional finer attribution. Keep its validity separate from the original endpoint path.
        // RawHip is the held source Upper origin, NOT the source Hips bone origin.
        internal Vector3 RawHip;
        internal Quaternion RawLower, RawKneeRelative;
        internal HipGoalPathEvidence HipPath;
        internal HipNormalReferenceState Normal;
        internal LandingParentPlacementInput Parent;
        internal LandingKneeControlInput Knee;

        internal bool Matches(PhysicsStepContext step, SourceStepContext source, int localFixed) =>
            Prepared && Step.Matches(step) && Source.Matches(source) && Fixed == localFixed &&
            source.Valid && source.Epoch == step.Epoch && source.Sequence == step.SourceSequence &&
            PelvisParentFrame.Q(UpperGoal) && PelvisParentFrame.Q(KneeRelativeGoal) && PelvisParentFrame.Q(Upper) &&
            PelvisParentFrame.V(ParentHip) && PelvisParentFrame.V(UpperHip) &&
            PelvisParentFrame.V(UpperHipToKnee) && PelvisParentFrame.V(LowerKneeAnchor);

        internal bool UpstreamMatches(PhysicsStepContext step, SourceStepContext source, int localFixed) =>
            Matches(step, source, localFixed) && HipPath.Valid && HipPath.Step.Matches(step) &&
            HipPath.Source.Matches(source) && HipPath.Fixed == localFixed && Normal.Matches(step) &&
            PelvisParentFrame.V(RawHip) && PelvisParentFrame.Q(RawLower) &&
            PelvisParentFrame.Q(RawKneeRelative) && PelvisParentFrame.Q(Normal.InputR);
    }

    // Optional evidence only. RawP/RawR are the calibrated proxy source frame,
    // never the uncalibrated animation Hips transform. Actual is settled PRE_N;
    // Reference is the final receiver target FOR_N, not a completed solve.
    internal struct LandingParentPlacementInput
    {
        internal bool Prepared;
        internal PhysicsStepContext Step;
        internal SourceStepContext Source;
        internal Vector3 RawP, ReferenceP, ActualP, LocalAnchor;
        internal Quaternion RawR, ReferenceR, ActualR;

        internal bool Matches(PhysicsStepContext step, SourceStepContext source) =>
            Prepared && Step.Matches(step) && Source.Matches(source) && source.Valid &&
            source.Epoch == step.Epoch && source.Sequence == step.SourceSequence &&
            PelvisParentFrame.V(RawP) && PelvisParentFrame.V(ReferenceP) &&
            PelvisParentFrame.V(ActualP) && PelvisParentFrame.V(LocalAnchor) &&
            PelvisParentFrame.Q(RawR) && PelvisParentFrame.Q(ReferenceR) && PelvisParentFrame.Q(ActualR);
    }

    // Copies the successful original Knee command and its already-written report.
    // No second PD/FF allocation or native read. Missing optional FF evidence is
    // explicit and does not invalidate the independent P/D/command receipt.
    internal struct LandingKneeControlInput
    {
        internal bool Prepared, FeedforwardValid, FeedforwardActive, FeedforwardClamped, IndependentGate;
        internal bool FlexionClamped, SoftClamped, ParentTransportEnabled;
        internal PhysicsStepContext Step;
        internal SourceStepContext Source;
        internal int Fixed;
        internal string FeedforwardReason, SoftPhase;
        internal float ErrorAbsDegrees, TargetW, BodyW, VelocityError;
        internal float P, D, Feedback, InputFF, SelectedFF, AllocatedFF, FinalFlexion, MaxFlexionA;
        internal float SoftRawA, SoftAppliedA, SoftMaxA;
        internal Vector3 RelativeA, ParentA, TotalA;
        internal bool ReferenceSelected;
        internal bool PlannedSourceRecoverySelected;
        internal int FeedforwardStateBits;
        internal KneeReferenceState Reference;
        internal Vector3 RawSourceA;

        internal bool Matches(PhysicsStepContext step, SourceStepContext source, int localFixed) =>
            Prepared && Step.Matches(step) && Source.Matches(source) && Fixed == localFixed &&
            source.Valid && source.Epoch == step.Epoch && source.Sequence == step.SourceSequence &&
            PelvisParentFrame.F(ErrorAbsDegrees) && ErrorAbsDegrees >= 0f &&
            PelvisParentFrame.F(TargetW) && PelvisParentFrame.F(BodyW) && PelvisParentFrame.F(VelocityError) &&
            PelvisParentFrame.F(P) && PelvisParentFrame.F(D) && PelvisParentFrame.F(Feedback) &&
            PelvisParentFrame.F(AllocatedFF) && PelvisParentFrame.F(FinalFlexion) &&
            PelvisParentFrame.F(MaxFlexionA) && MaxFlexionA > 0f &&
            PelvisParentFrame.F(SoftRawA) && SoftRawA >= 0f &&
            PelvisParentFrame.F(SoftAppliedA) && SoftAppliedA >= 0f &&
            PelvisParentFrame.F(SoftMaxA) && SoftMaxA >= 0f &&
            PelvisParentFrame.V(RelativeA) && PelvisParentFrame.V(ParentA) && PelvisParentFrame.V(TotalA) &&
            (!FeedforwardValid || (FeedforwardReason != null &&
                PelvisParentFrame.F(InputFF) && PelvisParentFrame.F(SelectedFF)));
    }
}
