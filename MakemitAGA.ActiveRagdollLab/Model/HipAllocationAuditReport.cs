using System;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // One bilateral accumulator, called after all actual writes/continuations succeed.
    // Constant storage: three value snapshots, no per-step strings/ring/native reads.
    internal sealed class HipAllocationAuditReport
    {
        internal const float GainThreshold = .01f; // Reporting threshold only, m/s²; never an admission/control limit.
        internal int Samples, Invalid, Missing, Duplicate, Gaps, Eligible, Limited, Fallback, Compared;
        internal int SameQNonzero, SameQUseful, TradeUseful, TradeBeatsSameQ, SameQInvalid, TradeInvalid;
        internal readonly int[] FullOver = new int[9], UniformCuts = new int[9], SameQCuts = new int[9], TradeCuts = new int[9];
        private readonly int[] _tradeRejects = new int[4];
        private PhysicsStepContext _last;
        private HipAllocationAuditSample _lossPeak, _tradePeak, _firstNoGain;
        private double _sameQSum, _tradeSum, _costLeftSum, _costRightSum;
        private long _ticks, _peakTicks;
        private int _timed;
        internal void Reset()
        {
            Samples = Invalid = Missing = Duplicate = Gaps = Eligible = Limited = Fallback = Compared = 0;
            SameQNonzero = SameQUseful = TradeUseful = TradeBeatsSameQ = SameQInvalid = TradeInvalid = 0;
            Array.Clear(FullOver, 0, 9); Array.Clear(UniformCuts, 0, 9); Array.Clear(SameQCuts, 0, 9); Array.Clear(TradeCuts, 0, 9);
            Array.Clear(_tradeRejects, 0, 4);
            _last = default; _lossPeak = _tradePeak = _firstNoGain = default;
            _sameQSum = _tradeSum = _costLeftSum = _costRightSum = 0d; _ticks = _peakTicks = 0; _timed = 0;
        }
        internal void NoteMissing() { Missing++; }
        internal void Observe(bool traceRunning, HipAnchorPairInput l, HipAnchorPairInput r,
            HipAnchorPairReport.Command lc, HipAnchorPairReport.Command rc, PelvisHipAttitudeCommand completed)
        {
            if (!traceRunning || (!l.Seed.ParentInput.AllocationAudit.Selected && !r.Seed.ParentInput.AllocationAudit.Selected)) return;
            long begin = Stopwatch.GetTimestamp();
            try
            {
                var step = l.Seed.Step;
                if (_last.Matches(step)) { Duplicate++; return; }
                if ((_last.Valid && (step.Epoch != _last.Epoch || step.Sequence <= _last.Sequence)) ||
                    !completed.Matches(l.Seed.ParentInput) || completed.Clamped ||
                    completed.MotionScale != lc.MotionScale || completed.UpwardExtraAcceleration != 0f ||
                    !PelvisChestPointResponse.Same(completed.CommonAcceleration, lc.CommonAcceleration) ||
                    !PelvisChestPointResponse.Same(completed.HipForce, -lc.Force - rc.Force) ||
                    !PelvisChestPointResponse.Same(completed.AngularReaction, lc.BilateralAngularReaction) ||
                    !HipBilateralForceSolver.TryAudit(l, r, lc, rc, out var s)) { Invalid++; return; }
                if (_last.Valid && step.Sequence != _last.Sequence + 1) Gaps++;
                _last = step; Samples++;
                if (s.Eligible) Eligible++;
                if (s.Limited) Limited++;
                if (!s.FullFeedback) Fallback++;
                Accumulate(FullOver, s.FullRequestOver); Accumulate(UniformCuts, s.UniformCuts);
                if (!s.Compared) return;
                Compared++; Accumulate(SameQCuts, s.SameQCuts); Accumulate(TradeCuts, s.TradeCuts);
                if (!s.SameQValid) SameQInvalid++;
                else
                {
                    _sameQSum += s.SameQGain;
                    if (s.SameQGain > 0f) SameQNonzero++;
                    if (s.SameQGain >= GainThreshold) SameQUseful++;
                }
                if (!s.TradeValid) { TradeInvalid++; _tradeRejects[(int)s.TradeIssue]++; }
                else
                {
                    _tradeSum += s.TradeGain; _costLeftSum += s.UpperTorqueCost.x; _costRightSum += s.UpperTorqueCost.y;
                    if (s.TradeGain >= GainThreshold) TradeUseful++;
                    if (s.SameQValid && s.TradeGain - s.SameQGain >= GainThreshold) TradeBeatsSameQ++;
                }
                if (!_lossPeak.Valid || s.RequestY - s.BaselineY > _lossPeak.RequestY - _lossPeak.BaselineY) _lossPeak = s;
                if (s.TradeValid && (!_tradePeak.Valid || s.TradeGain > _tradePeak.TradeGain)) _tradePeak = s;
                if (!_firstNoGain.Valid && s.SameQValid && s.SameQGain < GainThreshold) _firstNoGain = s;
            }
            catch { Invalid++; } // Optional diagnostics cannot escape into the physical fail-close path.
            finally
            {
                long elapsed = Math.Max(0, Stopwatch.GetTimestamp() - begin);
                _timed++; _ticks += elapsed; _peakTicks = Math.Max(_peakTicks, elapsed);
            }
        }
        private static void Accumulate(int[] counts, int mask)
        { for (int i = 0; i < 9; i++) if ((mask & (1 << i)) != 0) counts[i]++; }
        internal string Details(bool selected)
        {
            var sb = new StringBuilder("hipAllocationAudit=V041280/selected=" + selected +
                "/samples-invalid-missing-duplicate-gaps=" + Samples + "|" + Invalid + "|" + Missing + "|" + Duplicate + "|" + Gaps +
                "/eligible-limited-fallback-compared=" + Eligible + "|" + Limited + "|" + Fallback + "|" + Compared +
                "/sameQ(nonzero-gainGE0.01-invalid)=" + SameQNonzero + "|" + SameQUseful + "|" + SameQInvalid +
                "/trade(gainGE0.01-beatsSameQGE0.01-rejected)=" + TradeUseful + "|" + TradeBeatsSameQ + "|" + TradeInvalid +
                "/tradeReject(endpoint-interval-recheck)=" + _tradeRejects[1] + "|" + _tradeRejects[2] + "|" + _tradeRejects[3] +
                "/meanGainA(sameQ-trade)=" + Mean(_sameQSum, Compared - SameQInvalid) + "|" + Mean(_tradeSum, Compared - TradeInvalid) +
                "/meanTradeUpperTorqueCostNm(left-right)=" + Mean(_costLeftSum, Compared - TradeInvalid) + "|" + Mean(_costRightSum, Compared - TradeInvalid) +
                "/computeMs(mean-peak)=" + Mean(_ticks * 1000d / Stopwatch.Frequency, _timed) + "|" + F((float)(_peakTicks * 1000d / Stopwatch.Frequency)) +
                "/actual=UNIFORM_Q_ONLY/candidateExecuted=False/domain=POST_COMMIT_SAME_INPUT_ALGEBRA_NOT_PHYSX_RESPONSE" +
                "/tradePath=KEEP_XZ_AND_POINT_FEEDBACK_REDUCE_NET_UPPER_AND_RELATIVE_REACTION_FOR_POSITIVE_Y" +
                "/globalOptimality=False/contactCertification=False");
            sb.Append("\nhipAllocationBudgetOrder=COMMON|POINT_L|POINT_R|CORRECTION_L|CORRECTION_R|ACTUATOR_L|ACTUATOR_R|RECEIVER_TOTAL|RECEIVER_EXTERNAL")
                .Append("/fullRequestOver=").Append(string.Join("|", FullOver)).Append("/uniformUpperCuts=").Append(string.Join("|", UniformCuts))
                .Append("/sameQUpperCuts=").Append(string.Join("|", SameQCuts)).Append("/tradeUpperCuts=").Append(string.Join("|", TradeCuts))
                .Append("/countsOverlap=True/upperCutTieTolerance=1e-6/notActualSaturation=True");
            Append(sb, "MAX_Y_LOSS", _lossPeak); Append(sb, "MAX_TRADE_GAIN", _tradePeak); Append(sb, "FIRST_SMALL_SAME_Q_GAIN", _firstNoGain);
            return sb.ToString();
        }
        private static void Append(StringBuilder sb, string name, HipAllocationAuditSample s)
        {
            if (!s.Valid) return;
            sb.Append("\nhipAllocationAuditFrame=").Append(name).Append('/').Append(s.Step.Status())
                .Append("/localFixed(left-right)=").Append(s.LeftFixed).Append('|').Append(s.RightFixed)
                .Append("/errorYmm-q-requestY-baselineY=").Append(F(s.ErrorY * 1000f)).Append('|').Append(F(s.Q)).Append('|').Append(F(s.RequestY)).Append('|').Append(F(s.BaselineY))
                .Append("/sameQ(valid-gainA-cutMask)=").Append(s.SameQValid).Append('|').Append(F(s.SameQGain)).Append('|').Append(s.SameQCuts)
                .Append("/trade(valid-t-upperScale-gainA-cutMask)=").Append(s.TradeValid).Append('|').Append(F(s.TradeT)).Append('|').Append(F(s.TradeUpperScale)).Append('|').Append(F(s.TradeGain)).Append('|').Append(s.TradeCuts)
                .Append("/tradeCostNm(left-right-relativeReactionDelta)=").Append(F(s.UpperTorqueCost.x)).Append('|').Append(F(s.UpperTorqueCost.y)).Append('|').Append(F(s.ReactionDelta.magnitude))
                .Append("/tradeIssue=").Append(s.TradeIssue)
                .Append("/fullOver-uniformCutMask=").Append(s.FullRequestOver).Append('|').Append(s.UniformCuts);
        }
        private static string Mean(double sum, int count) => count > 0 ? F((float)(sum / count)) : "NA";
        private static string F(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
    }
}
