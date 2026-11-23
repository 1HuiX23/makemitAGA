using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Optional immutable-by-copy evidence. Input N-1 -> settled PRE_N, never
    // the unresolved request FOR_N. No controller or contact gate consumes it.
    internal struct PelvisNormalResponseEvidence
    {
        internal bool Valid, Enhanced, SourceFresh, LinearClamped;
        internal bool UpwardSelected, UpwardEligible;
        internal float UpwardExtra;
        internal bool RecoveryCaptured, RecoverySelected, RecoveryEligible;
        internal float RecoveryExtraY;
        internal bool BrakingCaptured, BrakingSelected;
        internal int WitnessMask, ApproachMask, WitnessSupportId;
        internal float BrakingExtraV;
        internal PhysicsStepContext Input, Settled;
        internal int Fixed, PelvisId, ChestId;
        internal string SourcePhase;
        internal float SourceAge, ErrorY, VelocityErrorY, P, D, Feedforward;
        internal float Request, Actuator, Allocated, Q, Observed, PreV, PostV;

        internal bool Matches(PhysicsStepContext step) => Valid && Input.Valid && Settled.Matches(step) &&
            Input.Epoch == Settled.Epoch && Input.Sequence + 1 == Settled.Sequence && Fixed > 0 &&
            Input.SourceSequence > 0 && Settled.SourceSequence >= Input.SourceSequence &&
            Settled.FixedTime >= Input.FixedTime && Settled.UnscaledTime >= Input.UnscaledTime &&
            PelvisId != 0 && ChestId != 0 && PelvisId != ChestId &&
            Finite(SourceAge) && SourceAge >= 0f && Finite(ErrorY) && Finite(VelocityErrorY) &&
            Finite(P) && Finite(D) && Finite(Feedforward) && Finite(Request) && Finite(Actuator) &&
            Finite(UpwardExtra) && UpwardExtra >= 0f &&
            (UpwardExtra == 0f || (UpwardSelected && UpwardEligible && Request > 0f && UpwardExtra <= (1f - Q) * Request)) &&
            (!RecoveryCaptured || (Finite(RecoveryExtraY) && RecoveryExtraY >= 0f &&
                RecoveryExtraY <= 2f * PelvisHipCommandFrame.MaxMagnitude + .002f &&
                (!RecoveryEligible || RecoverySelected) && (RecoveryExtraY == 0f || RecoveryEligible))) &&
            Finite(Allocated) && Finite(Q) && Q >= 0f && Q <= 1f &&
            System.Math.Abs(Allocated - (Request * Q + UpwardExtra)) <= .0001f * (1f + System.Math.Abs(Allocated)) &&
            Finite(Observed) && Finite(PreV) && Finite(PostV);

        // Optional context from the same resolved N-1 input. Missing this context
        // leaves the existing normal-response receipt and all controllers independent.
        internal void CaptureSupportBraking(in Physics.PelvisComReferencePlanner.ComPlan plan, PhysicsStepContext planStep)
        {
            bool witness = plan.SupportInput.Matches(Input);
            BrakingCaptured = planStep.Matches(Input) && plan.ArrivalEnabled && Finite(plan.SupportBrakingExtraY) &&
                (witness || (!plan.SupportInput.ForStep.Valid && !plan.SupportBrakingSelected));
            if (!BrakingCaptured) return;
            WitnessMask = witness ? plan.SupportInput.StanceMask : 0;
            ApproachMask = witness ? plan.SupportInput.ApproachMask : 0;
            WitnessSupportId = witness ? plan.SupportInput.SupportId : 0;
            BrakingSelected = plan.SupportBrakingSelected;
            BrakingExtraV = plan.SupportBrakingExtraY;
        }

        internal string Text()
        {
            if (!Matches(Settled)) return "receiverNormalV041240=NA";
            return "receiverNormalV041240=PREVIOUS_INPUT_TO_CURRENT_SETTLED" +
                "/normalClock(inputEpoch-fixed-source-settledFixed-localInput-dt)=" + Input.Epoch + "|" + Input.Sequence + "|" +
                Input.SourceSequence + "|" + Settled.Sequence + "|" + Fixed + "|" + F(Input.Dt) +
                "/normalSource(phase-fresh-age)=" + (SourcePhase ?? "UNKNOWN") + "|" + SourceFresh + "|" + F(SourceAge) +
                "/normalEnhanced=" + Enhanced + "/normalPre(errorYmm-velocityErrorY)=" + F(ErrorY * 1000f) + "|" + F(VelocityErrorY) +
                "/normalA(P-D-FF-request-actuator-allocated-observed)=" + F(P) + "|" + F(D) + "|" + F(Feedforward) + "|" +
                F(Request) + "|" + F(Actuator) + "|" + F(Allocated) + "|" + F(Observed) +
                "/normalQ-linearClamped=" + F(Q) + "|" + LinearClamped +
                "/normalUpwardRetention(selected-eligible-extraA)=" + UpwardSelected + "|" + UpwardEligible + "|" + F(UpwardExtra) +
                "/normalV(pre-post)=" + F(PreV) + "|" + F(PostV) +
                "/normalRecoveryV041370(captured-selected-eligible-extraY)=" + RecoveryCaptured + "|" + RecoverySelected + "|" + RecoveryEligible + "|" + F(RecoveryExtraY);
        }
        private static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static string F(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
    }
}
