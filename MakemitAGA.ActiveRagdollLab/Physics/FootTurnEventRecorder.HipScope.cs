using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V04790: read-only grouping of existing committed snapshots. No control consumer,
    // native getter, sampler or history buffer. Unknown evidence is never called protected.
    internal sealed partial class FootTurnEventRecorder
    {
        private const int ScopeUnavailable = 0, ScopeNormal = 1, ScopeProtected = 2, ScopeStale = 3;
        private const int ProtectedHipBits = 2 | 4 | 8 | 16 | 32 | 256;

        private sealed class ScopeStats
        {
            internal int Count, HipSamples, FootSamples, PlanSamples, MotionKnown, MotionMoving, HipClamped;
            internal int HipRawOver5Degrees, FootRawOver5Degrees;
            internal double Seconds, HipRawLocalSum, HipControlSum, HipRawWorldSum, HipGoalGapSum;
            internal double FootRawLocalSum, FootControlSum, FootRawWorldSum, FootGapSum, PlanGapSum;
            internal float HipRawLocalPeak, HipControlPeak, HipRawWorldPeak, HipGoalGapPeak;
            internal float FootRawLocalPeak, FootControlPeak, FootRawWorldPeak, FootGapPeak, PlanGapPeak;
        }

        private ScopeStats _normalScope = new ScopeStats(), _protectedScope = new ScopeStats();
        private ScopeStats _staleScope = new ScopeStats(), _unavailableScope = new ScopeStats(), _normalMovingScope = new ScopeStats();
        private int _hipScopeFailures;

        private void ResetHipScope()
        {
            _normalScope = new ScopeStats(); _protectedScope = new ScopeStats();
            _staleScope = new ScopeStats(); _unavailableScope = new ScopeStats(); _normalMovingScope = new ScopeStats();
            _hipScopeFailures = 0;
        }

        private static int ClassifyHipScope(Frame f)
        {
            // HipActiveThisStep is the actual FF owner's result, not a new health gate.
            // The cached bits/age only identify its inactive domain and detect contradictions.
            // Health bits 64/128 remain context, exactly as in the original owner.
            if (!f.LegValid || !f.Leg.HipFeedforwardValid || !f.Leg.HipFeedforwardEnabled ||
                (f.Leg.HipFeedforwardStateBits & ~511) != 0 || (f.Leg.HipFeedforwardStateBits & 1) != 0 ||
                !F(f.Leg.HipFeedforwardAge) || !F(f.Leg.HipFeedforwardMaxAge) || f.Leg.HipFeedforwardAge < -0.0001f)
                return ScopeUnavailable;
            bool active = f.Leg.HipFeedforwardActive;
            if ((f.Leg.HipFeedforwardStateBits & ProtectedHipBits) != 0)
                return active ? ScopeUnavailable : ScopeProtected;
            if (f.Leg.HipFeedforwardAge > Mathf.Max(0f, f.Leg.HipFeedforwardMaxAge) + 0.0001f)
                return active ? ScopeUnavailable : ScopeStale;
            return active ? ScopeNormal : ScopeUnavailable;
        }

        private static bool LegSourceMoving(Frame f) => f.LegValid && f.Leg.RawWorldWValid &&
            (f.Leg.RawUpperW.magnitude > QuietW || f.Leg.RawLowerW.magnitude > QuietW);

        private void CaptureHipScope(ref Frame f)
        {
            f.HipScope = ScopeUnavailable;
            try
            {
                f.HipScope = ClassifyHipScope(f);
                // A has no planner witness by design. Its absence does not remove A's normal samples.
                // This gap is to the planner's unpredicted input, not animation-world vs body error.
                var plan = f.Leg.HipNormalReference;
                f.NormalPlanGapValid = f.HipScope == ScopeNormal && plan.Matches(f.Step) && plan.Selected && plan.Applied && Q(plan.InputR);
                if (f.NormalPlanGapValid)
                {
                    f.NormalPlanGap = Log(plan.InputR * Quaternion.Inverse(f.Leg.HipTargetR)).magnitude;
                    f.NormalPlanGapValid = F(f.NormalPlanGap);
                }
            }
            catch { f.HipScope = ScopeUnavailable; f.NormalPlanGapValid = false; _hipScopeFailures++; }
            ScopeStats s = f.HipScope == ScopeNormal ? _normalScope : f.HipScope == ScopeProtected ? _protectedScope :
                f.HipScope == ScopeStale ? _staleScope : _unavailableScope;
            RecordHipScopeStats(s, f);
            if (f.HipScope == ScopeNormal && LegSourceMoving(f)) RecordHipScopeStats(_normalMovingScope, f);
        }

        private static void RecordHipScopeStats(ScopeStats s, Frame f)
        {
            s.Count++; s.Seconds += f.Step.Dt;
            if (f.LegValid && f.Leg.RawWorldWValid) s.MotionKnown++;
            if (LegSourceMoving(f)) s.MotionMoving++;
            if (f.LegValid && (f.Leg.Modes & 128) != 0) s.HipClamped++;
            if (f.HipWitnessValid)
            {
                s.HipSamples++;
                s.HipRawLocalSum += f.HipRawLocalError; s.HipRawLocalPeak = Mathf.Max(s.HipRawLocalPeak, f.HipRawLocalError);
                s.HipControlSum += f.HipControlError; s.HipControlPeak = Mathf.Max(s.HipControlPeak, f.HipControlError);
                s.HipRawWorldSum += f.HipRawWorldError; s.HipRawWorldPeak = Mathf.Max(s.HipRawWorldPeak, f.HipRawWorldError);
                s.HipGoalGapSum += f.HipTargetGap; s.HipGoalGapPeak = Mathf.Max(s.HipGoalGapPeak, f.HipTargetGap);
                if (f.HipRawLocalError > 5f * Mathf.Deg2Rad) s.HipRawOver5Degrees++;
            }
            if (f.ReferenceWitnessValid)
            {
                s.FootSamples++;
                s.FootRawLocalSum += f.RawLocalError; s.FootRawLocalPeak = Mathf.Max(s.FootRawLocalPeak, f.RawLocalError);
                s.FootControlSum += f.Error; s.FootControlPeak = Mathf.Max(s.FootControlPeak, f.Error);
                s.FootRawWorldSum += f.RawError; s.FootRawWorldPeak = Mathf.Max(s.FootRawWorldPeak, f.RawError);
                s.FootGapSum += f.ReferenceGap; s.FootGapPeak = Mathf.Max(s.FootGapPeak, f.ReferenceGap);
                if (f.RawLocalError > 5f * Mathf.Deg2Rad) s.FootRawOver5Degrees++;
            }
            if (f.NormalPlanGapValid)
            {
                s.PlanSamples++; s.PlanGapSum += f.NormalPlanGap;
                s.PlanGapPeak = Mathf.Max(s.PlanGapPeak, f.NormalPlanGap);
            }
        }

        private void AppendHipScopeSummary(StringBuilder sb)
        {
            sb.Append("HIP_SCOPE_V04790/readOnly=True/normal-protected-stale-unavailable=")
                .Append(_normalScope.Count).Append('/').Append(_protectedScope.Count).Append('/').Append(_staleScope.Count).Append('/').Append(_unavailableScope.Count)
                .Append("/unaggregated=").Append(_captured - _normalScope.Count - _protectedScope.Count - _staleScope.Count - _unavailableScope.Count)
                .Append("/failures=").Append(_hipScopeFailures)
                .AppendLine("/domain=ACTUAL_HIP_FF_OWNER_WITH_CACHED_BITS_AGE_CORROBORATION/priority=UNAVAILABLE_THEN_PROTECTION_THEN_STALE_THEN_NORMAL/healthBits=CONTEXT_NOT_GATES/aggregation=CAPTURED_FIXED_COMMANDS_INCLUDING_CENSORED_NOT_WINDOW_ROWS/means=FIXED_SAMPLE_NOT_TIME_WEIGHTED/normalMoving=SUBSET_RAW_UPPER_OR_LOWER_W_GT0.05/over5Deg=DIAGNOSTIC_EXPOSURE_NOT_SAFETY_OR_HUMAN_LIMIT/noNewRing=True");
            AppendHipScopeStats(sb, "NORMAL", _normalScope);
            AppendHipScopeStats(sb, "NORMAL_MOVING", _normalMovingScope);
            AppendHipScopeStats(sb, "PROTECTED", _protectedScope);
            AppendHipScopeStats(sb, "STALE", _staleScope);
            AppendHipScopeStats(sb, "UNAVAILABLE", _unavailableScope);
        }

        private static void AppendHipScopeStats(StringBuilder sb, string name, ScopeStats s)
        {
            sb.Append("HIP_SCOPE_STATS_V04790 scope=").Append(name).Append(" count=").Append(s.Count)
                .Append(" seconds=").Append(S((float)s.Seconds)).Append(" motionKnown-moving=").Append(s.MotionKnown).Append('/').Append(s.MotionMoving)
                .Append(" hipSamples-footSamples-planSamples=").Append(s.HipSamples).Append('/').Append(s.FootSamples).Append('/').Append(s.PlanSamples)
                .Append(" hipClamped-hipRawOver5Deg-footRawOver5Deg=").Append(s.HipClamped).Append('/').Append(s.HipRawOver5Degrees).Append('/').Append(s.FootRawOver5Degrees);
            sb.Append(" hipRawLocalMeanPeakDeg="); AppendScopeMeanPeak(sb, s.HipRawLocalSum, s.HipRawLocalPeak, s.HipSamples);
            sb.Append(" hipControlWorldMeanPeakDeg="); AppendScopeMeanPeak(sb, s.HipControlSum, s.HipControlPeak, s.HipSamples);
            sb.Append(" hipRawWorldMeanPeakDeg="); AppendScopeMeanPeak(sb, s.HipRawWorldSum, s.HipRawWorldPeak, s.HipSamples);
            sb.Append(" hipRawWorldToGoalMeanPeakDeg="); AppendScopeMeanPeak(sb, s.HipGoalGapSum, s.HipGoalGapPeak, s.HipSamples);
            sb.Append(" footRawLocalMeanPeakDeg="); AppendScopeMeanPeak(sb, s.FootRawLocalSum, s.FootRawLocalPeak, s.FootSamples);
            sb.Append(" footControlLocalMeanPeakDeg="); AppendScopeMeanPeak(sb, s.FootControlSum, s.FootControlPeak, s.FootSamples);
            sb.Append(" footRawWorldMeanPeakDeg="); AppendScopeMeanPeak(sb, s.FootRawWorldSum, s.FootRawWorldPeak, s.FootSamples);
            sb.Append(" footReferenceGapMeanPeakDeg="); AppendScopeMeanPeak(sb, s.FootGapSum, s.FootGapPeak, s.FootSamples);
            sb.Append(" normalInputToPlanMeanPeakDeg="); AppendScopeMeanPeak(sb, s.PlanGapSum, s.PlanGapPeak, s.PlanSamples);
            sb.AppendLine();
        }

        private static void AppendScopeMeanPeak(StringBuilder sb, double sum, float peak, int count)
        {
            if (count == 0) { sb.Append("NA"); return; }
            sb.Append(S((float)(sum / count) * Mathf.Rad2Deg)).Append('/').Append(S(peak * Mathf.Rad2Deg));
        }

        private static string HipScopeName(int scope) => scope == ScopeNormal ? "NORMAL" : scope == ScopeProtected ? "PROTECTED" :
            scope == ScopeStale ? "STALE" : "UNAVAILABLE";

        private static void AppendHipScopeWitness(StringBuilder sb, Frame f)
        {
            sb.Append(" hipScope_V04790=").Append(HipScopeName(f.HipScope)).Append(" normalInputToPlanRad=")
                .Append(f.NormalPlanGapValid ? S(f.NormalPlanGap) : "NA");
        }
    }
}
