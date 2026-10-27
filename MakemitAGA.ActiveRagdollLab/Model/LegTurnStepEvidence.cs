using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Read-only values for the existing Foot turn windows. No controller consumes this packet.
    // Body state is settled PRE_N; Hip/Knee accelerations are committed commands FOR_N.
    internal struct LegTurnStepEvidence
    {
        internal PhysicsStepContext Step;
        internal SourceStepContext Source;
        internal int Fixed, Modes;
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
