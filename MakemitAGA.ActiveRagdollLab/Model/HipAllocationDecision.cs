namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal enum HipAllocationCandidateIssue { None, EndpointPlan, NoInterval, CapExcludesInterval, EndpointRecheck }

    // Value receipt for diagnostics only. Physical validation uses the explicit
    // MotionScale/UpperAngularScale/UpwardExtraAcceleration and real wrench fields.
    // Recorded only after both pairs, receiver completion and leg continuations return.
    internal struct HipAllocationDecision
    {
        internal bool Attempted, CandidateValid, Applied;
        internal float SameQExtra, TradeExtra, TradeFraction, LeftTorqueCost, RightTorqueCost;
        internal double ComputeMilliseconds;
        internal HipAllocationCandidateIssue Issue;
        internal bool SameAs(HipAllocationDecision other) => Attempted == other.Attempted &&
            CandidateValid == other.CandidateValid && Applied == other.Applied && SameQExtra == other.SameQExtra &&
            TradeExtra == other.TradeExtra && TradeFraction == other.TradeFraction &&
            LeftTorqueCost == other.LeftTorqueCost && RightTorqueCost == other.RightTorqueCost &&
            ComputeMilliseconds == other.ComputeMilliseconds && Issue == other.Issue;
    }
}
