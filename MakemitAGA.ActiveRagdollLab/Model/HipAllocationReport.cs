using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // One bilateral accumulator after successful receiver AND downstream completion.
    // No candidate re-solve, body getters, ring, per-step strings or control feedback.
    internal sealed class HipAllocationReport
    {
        internal int Samples, Invalid, Missing, Duplicate, Gaps, Eligible, Limited, FeedbackFallback;
        internal int SameQNonzero, Attempted, CandidateValid, Applied, NotBetter;
        internal int EndpointRejected, IntervalRejected, CapRejected, RecheckRejected;
        private double _sameQ, _actual, _extraOverSameQ, _costL, _costR, _t, _compute, _computePeak;
        private float _gainPeak;
        private PhysicsStepContext _last;
        private HipAnchorPairReport.Command _first, _peak;

        internal void Reset()
        {
            Samples = Invalid = Missing = Duplicate = Gaps = Eligible = Limited = FeedbackFallback = 0;
            SameQNonzero = Attempted = CandidateValid = Applied = NotBetter = 0;
            EndpointRejected = IntervalRejected = CapRejected = RecheckRejected = 0;
            _sameQ = _actual = _extraOverSameQ = _costL = _costR = _t = _compute = _computePeak = 0d;
            _gainPeak = 0f; _last = default; _first = _peak = default;
        }
        internal void NoteMissing() => Missing++;
        internal void Observe(bool traceRunning, HipAnchorPairReport.Command l, HipAnchorPairReport.Command r,
            PelvisHipAttitudeCommand completed)
        {
            if (!traceRunning) return;
            try
            {
                if (!l.Step.Valid || !r.Step.Matches(l.Step) || !l.ParentInput.UpwardRetentionSelected ||
                    !l.ParentInput.Matches(l.Step, l.ParentId) || !r.ParentInput.Matches(r.Step, r.ParentId) ||
                    !l.ParentInput.SameAs(r.ParentInput) || !l.JointSolved || !r.JointSolved ||
                    l.Fixed <= 0 || l.Fixed != r.Fixed || l.ChildId == r.ChildId ||
                    !completed.Matches(l.ParentInput) || completed.Clamped || completed.MotionScale != l.MotionScale ||
                    completed.UpperAngularScale != l.UpperAngularScale || completed.UpwardExtraAcceleration != l.UpwardExtraAcceleration ||
                    r.MotionScale != l.MotionScale || r.UpperAngularScale != l.UpperAngularScale ||
                    r.UpwardExtraAcceleration != l.UpwardExtraAcceleration ||
                    !PelvisChestPointResponse.Same(l.CommonAcceleration, completed.CommonAcceleration) ||
                    !PelvisChestPointResponse.Same(r.CommonAcceleration, l.CommonAcceleration) ||
                    !PelvisChestPointResponse.Same(l.PeerForce, r.Force) || !PelvisChestPointResponse.Same(r.PeerForce, l.Force) ||
                    !PelvisChestPointResponse.Same(completed.HipForce, -l.Force - r.Force) ||
                    !PelvisChestPointResponse.Same(l.BilateralAngularReaction, r.BilateralAngularReaction) ||
                    !PelvisChestPointResponse.Same(completed.AngularReaction, l.ParentAngularReaction + r.ParentAngularReaction) ||
                    !PelvisChestPointResponse.Same(completed.AngularReaction, l.BilateralAngularReaction) ||
                    !l.AllocationDecision.SameAs(r.AllocationDecision) || !DecisionMatches(l))
                { Invalid++; return; }
                if (_last.Matches(l.Step)) { Duplicate++; return; }
                if (_last.Valid && (_last.Epoch != l.Step.Epoch || l.Step.Sequence <= _last.Sequence)) { Invalid++; return; }
                if (_last.Valid && _last.Sequence + 1 != l.Step.Sequence) Gaps++;
                _last = l.Step; Samples++;
                if (l.ParentInput.UpwardRetentionEligible) Eligible++;
                if (l.MotionScale < 1f) Limited++;
                if (!l.FullFeedbackFeasible) FeedbackFallback++;
                var d = l.AllocationDecision;
                if (d.SameQExtra > 0f) SameQNonzero++;
                _sameQ += d.SameQExtra; _actual += l.UpwardExtraAcceleration;
                if (!d.Attempted) return;
                Attempted++; _compute += d.ComputeMilliseconds; _computePeak = Math.Max(_computePeak, d.ComputeMilliseconds);
                if (!d.CandidateValid)
                {
                    switch (d.Issue)
                    {
                        case HipAllocationCandidateIssue.EndpointPlan: EndpointRejected++; break;
                        case HipAllocationCandidateIssue.NoInterval: IntervalRejected++; break;
                        case HipAllocationCandidateIssue.CapExcludesInterval: CapRejected++; break;
                        case HipAllocationCandidateIssue.EndpointRecheck: RecheckRejected++; break;
                    }
                    return;
                }
                CandidateValid++;
                if (!d.Applied) { NotBetter++; return; }
                Applied++; float gain = l.UpwardExtraAcceleration - d.SameQExtra;
                _extraOverSameQ += gain; _costL += d.LeftTorqueCost; _costR += d.RightTorqueCost; _t += d.TradeFraction;
                if (_first.Fixed == 0) _first = l;
                if (_peak.Fixed == 0 || gain > _gainPeak) { _gainPeak = gain; _peak = l; }
            }
            catch { Invalid++; } // Optional evidence must never stop physics.
        }
        private static bool DecisionMatches(HipAnchorPairReport.Command c)
        {
            var d = c.AllocationDecision;
            if (!HipPointPairMobility.F(d.SameQExtra) || d.SameQExtra < 0f ||
                !HipPointPairMobility.F(d.TradeExtra) || d.TradeExtra < 0f ||
                !HipPointPairMobility.F(d.TradeFraction) || d.TradeFraction < 0f || d.TradeFraction > HipSystemAllocation.MaxAngularTradeFraction ||
                !HipPointPairMobility.F(d.LeftTorqueCost) || d.LeftTorqueCost < 0f ||
                !HipPointPairMobility.F(d.RightTorqueCost) || d.RightTorqueCost < 0f ||
                double.IsNaN(d.ComputeMilliseconds) || double.IsInfinity(d.ComputeMilliseconds) || d.ComputeMilliseconds < 0d ||
                d.Attempted != (c.ParentInput.BoundedTradeSelected && c.ParentInput.UpwardRetentionEligible &&
                    c.FullFeedbackFeasible && c.MotionScale > 0f && c.MotionScale < 1f)) return false;
            if (d.CandidateValid && (!d.Attempted || d.Issue != HipAllocationCandidateIssue.None)) return false;
            if (d.Attempted && !d.CandidateValid && d.Issue == HipAllocationCandidateIssue.None) return false;
            if (d.Applied) return d.CandidateValid && c.UpperAngularScale == c.MotionScale * (1f - d.TradeFraction) &&
                c.UpperAngularScale < c.MotionScale && c.UpwardExtraAcceleration == d.TradeExtra && d.TradeExtra > d.SameQExtra;
            return c.UpperAngularScale == c.MotionScale && c.UpwardExtraAcceleration == d.SameQExtra;
        }
        internal string Details(bool tradeSelected)
        {
            var sb = new StringBuilder("hipAllocation=V041300/tradeSelected=" + tradeSelected +
                "/samples-invalid-missing-duplicate-gaps=" + Samples + "|" + Invalid + "|" + Missing + "|" + Duplicate + "|" + Gaps +
                "/eligible-limited-feedbackFallback=" + Eligible + "|" + Limited + "|" + FeedbackFallback +
                "/sameQNonzero=" + SameQNonzero + "/meanExtraA_allSamples(sameQ-actual)=" + Mean(_sameQ, Samples) + "|" + Mean(_actual, Samples) +
                "/actual=SAME_Q_WITH_OPTIONAL_CAP10/capBasis=FRACTION_OF_ALLOCATED_NET_UPPER_TORQUE/maxTradeFraction=0.10" +
                "/domain=COMMITTED_INPUT_NOT_PHYSX_RESPONSE/contactCertification=False");
            sb.Append("\nhipAllocationTrade=CAP10/attempted-valid-executed-notBetter=").Append(Attempted).Append('|').Append(CandidateValid).Append('|').Append(Applied).Append('|').Append(NotBetter)
                .Append("/reject(endpoint-interval-cap-recheck)=").Append(EndpointRejected).Append('|').Append(IntervalRejected).Append('|').Append(CapRejected).Append('|').Append(RecheckRejected)
                .Append("/executedMean(gainOverSameQ-t-costNmL-costNmR)=").Append(Mean(_extraOverSameQ, Applied)).Append('|').Append(Mean(_t, Applied)).Append('|').Append(Mean(_costL, Applied)).Append('|').Append(Mean(_costR, Applied))
                .Append("/gainOverSameQPeak=").Append(F(_gainPeak)).Append("/attemptComputeMs(mean-peak)=").Append(Mean(_compute, Attempted)).Append('|').Append(F(_computePeak));
            Append(sb, "FIRST_EXECUTED", _first); Append(sb, "MAX_GAIN_OVER_SAME_Q", _peak);
            return sb.ToString();
        }
        private static void Append(StringBuilder sb, string tag, HipAnchorPairReport.Command c)
        {
            if (c.Fixed == 0) return;
            var d = c.AllocationDecision;
            sb.Append("\nhipAllocationFrame=").Append(tag).Append('/').Append(c.Step.Status()).Append("/localFixed=").Append(c.Fixed)
                .Append("/q-upperAngularScale-t=").Append(F(c.MotionScale)).Append('|').Append(F(c.UpperAngularScale)).Append('|').Append(F(d.TradeFraction))
                .Append("/requestY-baseY-sameQExtra-actualExtra=").Append(F(c.ParentInput.Acceleration.y)).Append('|').Append(F(c.MotionScale * c.ParentInput.Acceleration.y)).Append('|').Append(F(d.SameQExtra)).Append('|').Append(F(c.UpwardExtraAcceleration))
                .Append("/costNm(L-R)=").Append(F(d.LeftTorqueCost)).Append('|').Append(F(d.RightTorqueCost));
        }
        private static string Mean(double sum, int count) => count == 0 ? "NA" : F(sum / count);
        private static string F(double v) => v.ToString("G9", CultureInfo.InvariantCulture);
    }
}
