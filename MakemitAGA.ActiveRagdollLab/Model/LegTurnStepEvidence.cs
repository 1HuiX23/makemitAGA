using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Read-only values for the existing Foot turn windows. No controller consumes this packet.
    // Body state is settled PRE_N; Hip/Knee accelerations are committed commands FOR_N.
    internal struct LegTurnStepEvidence
    {
        internal PhysicsStepContext Step;
        internal LandingPreparationCommand LandingPreparation;
        internal SourceStepContext Source;
        internal int Fixed, Modes, CommittedUpperId;
        internal LegLandingReferenceInput LandingInput; // Optional; never part of Matches or a control gate.
        internal bool KneeHeightsValid;
        internal Vector3 KneeHeights, UpperLocalSpan; // Held animation, selected pose on receiver reference, and settled lower-side knee anchor.
        // The Upper-side anchors were read by mechanical reporting in this same PRE_N.
        internal bool UpperAnchorsValid;
        internal LegMeasuredAnchors MeasuredAnchors;
        internal Vector3 PhysicalUpperHip => MeasuredAnchors.UpperHip;
        internal Vector3 PhysicalUpperKnee => MeasuredAnchors.UpperKnee;
        internal Vector3 PhysicalLowerKnee => MeasuredAnchors.LowerKnee;
        internal string SourcePhase; // Current existing torso source classifier; window selection only.
        internal HipGoalPathEvidence HipGoalPath; // Optional; deliberately excluded from Matches.
        internal PelvisNormalResponseEvidence ReceiverNormalResponse; // Solved N-1 -> PRE_N; optional, never a gate.
        internal bool HipCommitted, KneeCommitted, RawWorldWValid;
        internal float RawSourceDt, HipMaxA;
        internal Quaternion RawUpperR, RawLowerR, UpperR, HipTargetR;
        internal Quaternion RawHipRelativeR; // Optional witness: the exact input of legacy hipRot, not the final world control goal.
        internal Vector3 RawUpperW, RawLowerW, UpperW, HipParentW, HipTargetW, KneeTargetRelW;
        internal Vector3 HipFinalA, HipParentPreCapA, KneeRelativeA, KneeTotalA;

        // Optional same-Fixed snapshot of the existing FF owner. Not part of Matches or any control gate.
        internal bool HipFeedforwardValid, HipFeedforwardEnabled, HipFeedforwardActive, HipFeedforwardClamped;
        internal int HipFeedforwardStateBits;
        internal string HipFeedforwardReason;
        internal float HipFeedforwardAge, HipFeedforwardMaxAge, HipFeedforwardGain, HipFeedforwardBound;
        internal Vector3 HipFeedforwardInputA, HipFeedforwardSelectedA;
        internal bool TorsoRecoveryDynamicSelected, TorsoRecoveryLegacyActive;
        internal TorsoRecoveryState TorsoRecovery;
        internal bool HipGoalGateKneeIndependent, KneeFeedforwardValid, KneeFeedforwardActive, KneeFeedforwardClamped;
        internal HipNormalReferenceState HipNormalReference;
        internal bool HipNormalRecoveryHandoffStarted, HipNormalRecoveryHandoffActive;
        internal string KneeFeedforwardReason;
        internal float KneeFeedforwardInputSigned, KneeFeedforwardSelectedSigned;
        internal float KneeFeedbackSigned, KneeFeedforwardAllocatedSigned, KneeFlexionFinalSigned;

        internal void CaptureKneeHeights(PhysicsStepContext step, int localFixed,
            Vector3 physicalUpperHip, Vector3 physicalUpperKnee, Vector3 physicalLowerKnee)
        {
            UpperAnchorsValid = KneeHeightsValid = false;
            MeasuredAnchors = LegMeasuredAnchors.Capture(HipCommitted && KneeCommitted, Step, Fixed,
                step, localFixed, physicalUpperHip, physicalUpperKnee, physicalLowerKnee);
            if (!MeasuredAnchors.Valid || LandingInput.Fixed != localFixed || !LandingInput.Step.Matches(step) ||
                !LandingInput.Source.Matches(Source) || !LandingInput.Parent.Matches(step, Source) ||
                !PelvisParentFrame.Q(UpperR) || !PelvisParentFrame.Q(RawUpperR) || !PelvisParentFrame.Q(HipTargetR)) return;

            // The two upper-side points are from this pre-solve. Keep the actual lower-side joint gap.
            Vector3 span = Quaternion.Inverse(UpperR) * (physicalUpperKnee - physicalUpperHip);
            KneeHeights = new Vector3((LandingInput.RawHip + RawUpperR * span).y,
                (LandingInput.Parent.ReferencePoint + HipTargetR * span).y, physicalLowerKnee.y);
            UpperLocalSpan = span;
            KneeHeightsValid = PelvisParentFrame.V(KneeHeights);
            UpperAnchorsValid = KneeHeightsValid && PelvisParentFrame.V(physicalUpperHip) && PelvisParentFrame.V(physicalUpperKnee);
        }

        internal bool Matches(PhysicsStepContext step, int localFixed) =>
            HipCommitted && KneeCommitted && Step.Matches(step) && Fixed == localFixed &&
            Source.Valid && Source.Epoch == step.Epoch && Source.Sequence == step.SourceSequence &&
            Q(RawUpperR) && Q(RawLowerR) && Q(UpperR) && Q(HipTargetR) &&
            V(UpperW) && V(HipParentW) && V(HipTargetW) && V(KneeTargetRelW) &&
            V(HipFinalA) && V(HipParentPreCapA) && V(KneeRelativeA) && V(KneeTotalA) &&
            F(HipMaxA) && HipMaxA > 0f &&
            (!RawWorldWValid || (F(RawSourceDt) && RawSourceDt >= 0.001f && RawSourceDt <= 0.20f &&
                V(RawUpperW) && V(RawLowerW)));

        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) &&
            q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.000001f;
    }
}
