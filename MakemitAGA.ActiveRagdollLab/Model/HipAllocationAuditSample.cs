using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal enum HipAllocationCandidateIssue { None, EndpointPlan, NoInterval, EndpointRecheck }
    internal struct HipAllocationAuditSample
    {
        internal bool Valid, Eligible, Limited, FullFeedback, Compared, SameQValid, TradeValid;
        internal PhysicsStepContext Step;
        internal int LeftFixed, RightFixed, FullRequestOver, UniformCuts, SameQCuts, TradeCuts;
        internal float ErrorY, Q, RequestY, BaselineY, SameQGain, TradeGain, TradeT, TradeUpperScale;
        internal Vector3 TradeTarget, TradeCommon, TradeLeftForce, TradeRightForce;
        internal Vector3 UpperTorqueCost, ReactionDelta;
        internal HipAllocationCandidateIssue TradeIssue;
    }
}
