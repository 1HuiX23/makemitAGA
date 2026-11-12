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
            Finite(Allocated) && Finite(Q) && Q >= 0f && Q <= 1f &&
            System.Math.Abs(Allocated - (Request * Q + UpwardExtra)) <= .0001f * (1f + System.Math.Abs(Allocated)) &&
            Finite(Observed) && Finite(PreV) && Finite(PostV);

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
                "/normalV(pre-post)=" + F(PreV) + "|" + F(PostV);
        }
        private static bool Finite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static string F(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
    }
}
