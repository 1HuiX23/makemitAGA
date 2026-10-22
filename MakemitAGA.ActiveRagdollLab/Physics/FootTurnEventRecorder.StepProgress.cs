using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V04710: values from the existing command/pending endpoint, no extra sampler or ring.
    // This is a diagnostic ordering of error changes, NOT a second plant or controller.
    internal sealed partial class FootTurnEventRecorder
    {
        private struct StepProgress
        {
            internal bool ReferenceValid, FreshPair, MechanicalValid;
            internal SourceStepContext NextSource;
            internal float NextAge, LockedError, NextError, PhysicalDelta, SourceDelta, NetDelta, Closure;
            internal Vector3 PairRequestA, ActualRelativeA, RelativeResidualA, CommonPastMinusNextA, ResidualMinusLagA;
        }
        private int _progressResolved, _progressReference, _progressFresh, _progressMechanical, _progressUnclamped, _progressFailures;
        private int _conditionerSamples, _conditionerUnavailable, _conditionerRawValid, _conditionerHigh, _conditionerConfirmed, _conditionerStale;
        private double _physicalPositive, _sourcePositive, _netPositive, _lagSum, _residualSum, _minusLagSum;
        private float _closurePeak, _lagPeak, _residualPeak;
        private int _referenceSamples, _referenceMissing, _referenceSelectedSamples;
        private double _rawLocalSum, _controlLocalSum, _rawWorldSum, _referenceGapSum;
        private float _rawLocalPeak, _controlLocalPeak, _rawWorldPeak, _referenceGapPeak;
        private struct HipProgress
        {
            internal bool Valid;
            internal float LockedError, NextError, PhysicalDelta, TargetDelta, NetDelta, Closure;
            internal Quaternion NextUpperR, NextTargetR;
            internal Vector3 NextUpperW, ActualUpperA;
        }
        private int _hipSamples, _hipMissing, _hipResponseSamples, _hipResponseFailures;
        private double _hipRawLocalSum, _hipControlSum, _hipRawWorldSum, _hipTargetGapSum;
        private float _hipRawLocalPeak, _hipControlPeak, _hipRawWorldPeak, _hipTargetGapPeak, _hipClosurePeak;

        private void ResetHipWitness()
        {
            _hipSamples = _hipMissing = _hipResponseSamples = _hipResponseFailures = 0;
            _hipRawLocalSum = _hipControlSum = _hipRawWorldSum = _hipTargetGapSum = 0d;
            _hipRawLocalPeak = _hipControlPeak = _hipRawWorldPeak = _hipTargetGapPeak = _hipClosurePeak = 0f;
        }

        private void CaptureHipWitness(ref Frame f)
        {
            // Independent optional diagnostic: legacy hipRot compares RAW LOCAL to the actual
            // Pelvis-relative Upper, whereas the final controller may use an inertialized world frame.
            if (!f.LegValid || !Q(f.Leg.RawHipRelativeR)) { _hipMissing++; return; }
            f.HipRawLocalError = Log((f.PelvisR * f.Leg.RawHipRelativeR) * Quaternion.Inverse(f.Leg.UpperR)).magnitude;
            f.HipControlError = Log(f.Leg.HipTargetR * Quaternion.Inverse(f.Leg.UpperR)).magnitude;
            f.HipRawWorldError = Log(f.Leg.RawUpperR * Quaternion.Inverse(f.Leg.UpperR)).magnitude;
            f.HipTargetGap = Log(f.Leg.RawUpperR * Quaternion.Inverse(f.Leg.HipTargetR)).magnitude;
            f.HipWitnessValid = F(f.HipRawLocalError) && F(f.HipControlError) && F(f.HipRawWorldError) && F(f.HipTargetGap);
            if (!f.HipWitnessValid) { _hipMissing++; return; }
            _hipSamples++;
            _hipRawLocalSum += f.HipRawLocalError; _hipControlSum += f.HipControlError;
            _hipRawWorldSum += f.HipRawWorldError; _hipTargetGapSum += f.HipTargetGap;
            _hipRawLocalPeak = Mathf.Max(_hipRawLocalPeak, f.HipRawLocalError);
            _hipControlPeak = Mathf.Max(_hipControlPeak, f.HipControlError);
            _hipRawWorldPeak = Mathf.Max(_hipRawWorldPeak, f.HipRawWorldError);
            _hipTargetGapPeak = Mathf.Max(_hipTargetGapPeak, f.HipTargetGap);
        }

        private HipProgress ResolveHipProgress(Frame f, PhysicsStepContext endpoint, SourceStepContext nextSource,
            LegTurnStepEvidence nextLeg, int nextFixed)
        {
            HipProgress p = new HipProgress();
            if (!f.HipWitnessValid || !nextLeg.Matches(endpoint, nextFixed) || !nextLeg.Source.Matches(nextSource)) return p;
            p.NextUpperR = nextLeg.UpperR; p.NextTargetR = nextLeg.HipTargetR; p.NextUpperW = nextLeg.UpperW;
            p.ActualUpperA = (nextLeg.UpperW - f.Leg.UpperW) / f.Step.Dt;
            // Hold the OLD WORLD goal first; new source/frame/reference changes are the second term.
            // This scalar ordering is diagnostic, not a torque decomposition or causal share.
            p.LockedError = Log(f.Leg.HipTargetR * Quaternion.Inverse(nextLeg.UpperR)).magnitude;
            p.NextError = Log(nextLeg.HipTargetR * Quaternion.Inverse(nextLeg.UpperR)).magnitude;
            p.PhysicalDelta = p.LockedError - f.HipControlError;
            p.TargetDelta = p.NextError - p.LockedError;
            p.NetDelta = p.NextError - f.HipControlError;
            p.Closure = p.PhysicalDelta + p.TargetDelta - p.NetDelta;
            p.Valid = V(p.ActualUpperA) && F(p.ActualUpperA.magnitude) && F(p.LockedError) && F(p.NextError) && F(p.Closure);
            if (p.Valid)
            {
                _hipResponseSamples++;
                _hipClosurePeak = Mathf.Max(_hipClosurePeak, Mathf.Abs(p.Closure));
            }
            return p;
        }

        private void AppendHipSummary(StringBuilder sb)
        {
            sb.Append("HIP_TURN_V04740/readOnly=True/samples-missing-resolved-failures=")
                .Append(_hipSamples).Append('/').Append(_hipMissing).Append('/').Append(_hipResponseSamples).Append('/').Append(_hipResponseFailures)
                .Append("/rawLocalMeanPeakDeg="); AppendHipMeanPeak(sb, _hipRawLocalSum, _hipRawLocalPeak);
            sb.Append("/controlWorldMeanPeakDeg="); AppendHipMeanPeak(sb, _hipControlSum, _hipControlPeak);
            sb.Append("/rawWorldMeanPeakDeg="); AppendHipMeanPeak(sb, _hipRawWorldSum, _hipRawWorldPeak);
            sb.Append("/rawToControlGoalMeanPeakDeg="); AppendHipMeanPeak(sb, _hipTargetGapSum, _hipTargetGapPeak);
            sb.Append("/closurePeakRad=").Append(_hipResponseSamples > 0 ? S(_hipClosurePeak) : "NA")
                .AppendLine("/peakSelection=FINAL_WORLD_CONTROL_ERROR/endpoint=CACHED_SETTLED_UPPER_BEFORE_SOLVE/ordering=OLD_WORLD_GOAL_THEN_NEW_WORLD_GOAL/response=TOTAL_UPPER_NOT_FOOT_REACTION_CAUSAL_SHARE");
        }
        private void AppendHipMeanPeak(StringBuilder sb, double sum, float peak)
        {
            if (_hipSamples == 0) { sb.Append("NA"); return; }
            sb.Append(S((float)(sum / _hipSamples) * Mathf.Rad2Deg)).Append('/').Append(S(peak * Mathf.Rad2Deg));
        }
        private static void AppendHipWitness(StringBuilder sb, Frame f)
        {
            sb.Append(" hipErrorRad(rawLocal-controlWorld-rawWorld-goalGap)=");
            if (!f.HipWitnessValid) { sb.Append("NA"); return; }
            sb.Append(S(f.HipRawLocalError)).Append('/').Append(S(f.HipControlError)).Append('/')
                .Append(S(f.HipRawWorldError)).Append('/').Append(S(f.HipTargetGap))
                .Append(" rawHipLocalR=").Append(R(f.Leg.RawHipRelativeR))
                .Append(" hipStepRad(before-lockedNext-currentNext-physical-target-net-closure)=");
            HipProgress p = f.HipResponse;
            if (!f.Resolved || !p.Valid) { sb.Append("NA"); return; }
            sb.Append(S(f.HipControlError)).Append('/').Append(S(p.LockedError)).Append('/').Append(S(p.NextError))
                .Append('/').Append(S(p.PhysicalDelta)).Append('/').Append(S(p.TargetDelta)).Append('/').Append(S(p.NetDelta)).Append('/').Append(S(p.Closure))
                .Append(" hipNext(Rupper-Rtarget-Wupper-Aupper)=").Append(R(p.NextUpperR)).Append('/').Append(R(p.NextTargetR))
                .Append('/').Append(T(p.NextUpperW)).Append('/').Append(T(p.ActualUpperA));
        }

        private static void AppendHipFeedforwardWitness(StringBuilder sb, Frame f)
        {
            LegTurnStepEvidence l = f.Leg;
            if (!f.LegValid || !l.HipFeedforwardValid || l.HipFeedforwardReason == null ||
                !V(l.HipFeedforwardInputA) || !V(l.HipFeedforwardSelectedA) ||
                !F(l.HipFeedforwardAge) || !F(l.HipFeedforwardMaxAge) ||
                !F(l.HipFeedforwardGain) || !F(l.HipFeedforwardBound))
            { sb.Append(" hipFF_V04760=NA"); return; }
            sb.Append(" hipFF_V04760(enabled-active-clamped-stateBits)=").Append(l.HipFeedforwardEnabled)
                .Append('/').Append(l.HipFeedforwardActive).Append('/').Append(l.HipFeedforwardClamped).Append('/').Append(l.HipFeedforwardStateBits)
                .Append(" hipFFReason=").Append(l.HipFeedforwardReason)
                .Append(" hipFF(age-maxAge-gain-bound)=").Append(S(l.HipFeedforwardAge)).Append('/').Append(S(l.HipFeedforwardMaxAge))
                .Append('/').Append(S(l.HipFeedforwardGain)).Append('/').Append(S(l.HipFeedforwardBound))
                .Append(" hipFFWorld(inputA-selectedA)=").Append(T(l.HipFeedforwardInputA)).Append('/').Append(T(l.HipFeedforwardSelectedA))
                .Append(" torsoRecovery(selectedDynamic-legacyActive)=").Append(l.TorsoRecoveryDynamicSelected).Append('/').Append(l.TorsoRecoveryLegacyActive);
            TorsoRecoveryState r = l.TorsoRecovery;
            if (!r.Matches(l.Step)) { sb.Append(" torsoRecoveryDynamic=NA"); return; }
            sb.Append(" torsoRecoveryDynamic(active-reason-hold)=").Append(r.Active).Append('/').Append(r.Reason).Append('/').Append(S(r.HoldSeconds))
                .Append(" torsoRecoveryErrors(COMm-pelvisDeg-chestRelDeg-nativeM-legacyM)=").Append(S(r.ComErrorMeters))
                .Append('/').Append(S(r.PelvisErrorDegrees)).Append('/').Append(S(r.ChestRelativeErrorDegrees))
                .Append('/').Append(S(r.NativeAnchorGapMeters)).Append('/').Append(S(r.LegacyAnchorGapMeters));
        }

        private void ResetStepProgress()
        {
            _progressResolved = _progressReference = _progressFresh = _progressMechanical = _progressUnclamped = _progressFailures = 0;
            _conditionerSamples = _conditionerUnavailable = _conditionerRawValid = _conditionerHigh = _conditionerConfirmed = _conditionerStale = 0;
            _physicalPositive = _sourcePositive = _netPositive = _lagSum = _residualSum = _minusLagSum = 0d;
            _closurePeak = _lagPeak = _residualPeak = 0f;
            _referenceSamples = _referenceMissing = _referenceSelectedSamples = 0;
            _rawLocalSum = _controlLocalSum = _rawWorldSum = _referenceGapSum = 0d;
            _rawLocalPeak = _controlLocalPeak = _rawWorldPeak = _referenceGapPeak = 0f;
        }

        private void RecordConditionerWitness(Frame f)
        {
            if (f.ConditionerStage == null || f.ConditionerStage == "UNSAMPLED" || !V(f.ConditionedA))
            { _conditionerUnavailable++; return; }
            _conditionerSamples++;
            if (f.ConditionerRawValid) _conditionerRawValid++;
            if (f.ConditionerHigh) _conditionerHigh++;
            if (f.ConditionerConfirmed) _conditionerConfirmed++;
            if (f.Stale) _conditionerStale++;
        }

        private StepProgress ResolveStepProgress(Frame f, PhysicsStepContext endpoint, SourceStepContext nextSource,
            Quaternion nextSourceRelative, float nextAge, Quaternion nextLowerR, Quaternion nextFootR)
        {
            _progressResolved++;
            StepProgress p = new StepProgress();
            // Shared source identities are distinct from the local Render counters. Multiple
            // real Render packets between Fixed steps are legal; held packets must be exact.
            bool sourcePair = f.CommandSource.Valid && nextSource.Valid &&
                f.CommandSource.Epoch == f.Step.Epoch && f.CommandSource.Sequence == f.Step.SourceSequence &&
                nextSource.Epoch == endpoint.Epoch && nextSource.Sequence == endpoint.SourceSequence &&
                nextSource.Sequence >= f.CommandSource.Sequence && nextSource.Time >= f.CommandSource.Time &&
                (nextSource.Sequence != f.CommandSource.Sequence || nextSource.Matches(f.CommandSource));
            if (sourcePair && Q(nextSourceRelative) && F(nextAge) && nextAge >= 0f)
            {
                p.NextSource = nextSource; p.NextAge = nextAge;
                p.LockedError = f.LockedNextErrorWorld.magnitude;
                p.NextError = Log((nextLowerR * nextSourceRelative) * Quaternion.Inverse(nextFootR)).magnitude;
                // Exact scalar telescoping in this stated order. Do NOT add 3D log vectors
                // or interpret either signed term as a context-free causal percentage.
                p.PhysicalDelta = p.LockedError - f.Error;
                p.SourceDelta = p.NextError - p.LockedError;
                p.NetDelta = p.NextError - f.Error;
                p.Closure = p.PhysicalDelta + p.SourceDelta - p.NetDelta;
                p.ReferenceValid = F(p.NextError) && F(p.Closure);
                p.FreshPair = p.ReferenceValid && !f.Stale &&
                    f.Age <= FootAnkleSourceDrivenRelativePdIsolation.MaxFreshTargetAgeSeconds &&
                    nextAge <= FootAnkleSourceDrivenRelativePdIsolation.MaxFreshTargetAgeSeconds;
                if (p.ReferenceValid)
                {
                    _progressReference++; _closurePeak = Mathf.Max(_closurePeak, Mathf.Abs(p.Closure));
                    if (p.FreshPair)
                    {
                        _progressFresh++;
                        _physicalPositive += Mathf.Max(0f, p.PhysicalDelta);
                        _sourcePositive += Mathf.Max(0f, p.SourceDelta);
                        _netPositive += Mathf.Max(0f, p.NetDelta);
                    }
                }
            }
            // PairRequestA predicts ONLY the internal torque pair. Full response includes
            // common transport, native damping/gyro and all Lower inputs; it need not equal it.
            p.PairRequestA = f.LowerR * f.RelativePredictedA;
            p.ActualRelativeA = f.MeasuredFootA - f.MeasuredLowerA;
            p.RelativeResidualA = p.ActualRelativeA - p.PairRequestA;
            p.CommonPastMinusNextA = f.CommonMeasuredA - f.MeasuredLowerA;
            p.ResidualMinusLagA = p.RelativeResidualA - p.CommonPastMinusNextA;
            p.MechanicalValid = f.CommonActive && !f.CommonWarmup && V(p.PairRequestA) &&
                V(p.ActualRelativeA) && V(p.RelativeResidualA) && V(p.CommonPastMinusNextA) && V(p.ResidualMinusLagA) &&
                F(p.RelativeResidualA.magnitude) && F(p.CommonPastMinusNextA.magnitude) && F(p.ResidualMinusLagA.magnitude);
            if (p.MechanicalValid)
            {
                _progressMechanical++;
                if (!f.RelativeAClamp && !f.RelativeTauClamp && !f.CommonAClamp && !f.CommonTauClamp) _progressUnclamped++;
                float lag = p.CommonPastMinusNextA.magnitude, residual = p.RelativeResidualA.magnitude;
                _lagSum += lag; _residualSum += residual; _minusLagSum += p.ResidualMinusLagA.magnitude;
                _lagPeak = Mathf.Max(_lagPeak, lag); _residualPeak = Mathf.Max(_residualPeak, residual);
            }
            return p;
        }

        private void AppendStepProgressSummary(StringBuilder sb)
        {
            sb.Append("FOOT_STEP_ATTR_V04710/readOnly=True/resolved-reference-fresh-mechanical-footUnclamped-failures=")
                .Append(_progressResolved).Append('/').Append(_progressReference).Append('/').Append(_progressFresh)
                .Append('/').Append(_progressMechanical).Append('/').Append(_progressUnclamped).Append('/').Append(_progressFailures)
                .Append("/freshPositiveErrorGrowthRad(physical-source-net)=");
            if (_progressFresh > 0) sb.Append(S((float)_physicalPositive)).Append('/').Append(S((float)_sourcePositive)).Append('/').Append(S((float)_netPositive));
            else sb.Append("NA");
            sb.Append("/closurePeakRad=").Append(_progressReference > 0 ? S(_closurePeak) : "NA")
                .Append("/mechanicalMeanNormA(lag-residual-residualMinusLag)=");
            if (_progressMechanical > 0) sb.Append(S((float)(_lagSum / _progressMechanical))).Append('/')
                .Append(S((float)(_residualSum / _progressMechanical))).Append('/').Append(S((float)(_minusLagSum / _progressMechanical)));
            else sb.Append("NA");
            sb.Append("/mechanicalPeakNormA(lag-residual)=").Append(_progressMechanical > 0 ? S(_lagPeak) + "/" + S(_residualPeak) : "NA")
                .Append("/conditioner(captured-unavailable-rawValid-high-confirmed-stale)=").Append(_conditionerSamples).Append('/')
                .Append(_conditionerUnavailable).Append('/').Append(_conditionerRawValid).Append('/').Append(_conditionerHigh).Append('/')
                .Append(_conditionerConfirmed).Append('/').Append(_conditionerStale)
                .AppendLine("/ordering=BODY_RELATIVE_STEP_AT_OLD_TARGET_THEN_NEW_SOURCE_AT_SAME_ENDPOINT/growth=SUM_POSITIVE_INCREMENTS_NOT_CAUSAL_SHARE/A=COMMAND_N_VS_NEXT_SOLVE_COMMAND_DT/lag=PAST_MEASURED_LOWER_A_MINUS_NEXT_ACTUAL_LOWER_A/NA=INDEPENDENT_OPTIONAL_QUALIFICATION");
        }

        private void RecordReferenceWitness(Frame f)
        {
            if (!f.ReferenceWitnessValid) { _referenceMissing++; return; }
            _referenceSamples++;
            if (f.ReferenceSelected && f.ReferenceActive) _referenceSelectedSamples++;
            _rawLocalSum += f.RawLocalError; _controlLocalSum += f.Error;
            _rawWorldSum += f.RawError; _referenceGapSum += f.ReferenceGap;
            _rawLocalPeak = Mathf.Max(_rawLocalPeak, f.RawLocalError); _controlLocalPeak = Mathf.Max(_controlLocalPeak, f.Error);
            _rawWorldPeak = Mathf.Max(_rawWorldPeak, f.RawError); _referenceGapPeak = Mathf.Max(_referenceGapPeak, f.ReferenceGap);
        }

        private void AppendReferenceSummary(StringBuilder sb)
        {
            sb.Append("FOOT_REFERENCE_TRACKING_V04720/samples-missing-selected=").Append(_referenceSamples).Append('/')
                .Append(_referenceMissing).Append('/').Append(_referenceSelectedSamples)
                .Append("/rawLocalMeanPeakDeg=");
            AppendMeanPeak(sb, _rawLocalSum, _rawLocalPeak);
            sb.Append("/controlLocalMeanPeakDeg="); AppendMeanPeak(sb, _controlLocalSum, _controlLocalPeak);
            sb.Append("/rawWorldMeanPeakDeg="); AppendMeanPeak(sb, _rawWorldSum, _rawWorldPeak);
            sb.Append("/referenceGapMeanPeakDeg="); AppendMeanPeak(sb, _referenceGapSum, _referenceGapPeak);
            sb.AppendLine("/raw=UNMODIFIED_ANIMATION/control=SELECTED_REFERENCE/means=CAPTURED_FIXED_SAMPLES_NOT_CROSS_RUN_CAUSAL_EFFECT");
        }
        private void AppendMeanPeak(StringBuilder sb, double sum, float peak)
        {
            if (_referenceSamples == 0) { sb.Append("NA"); return; }
            sb.Append(S((float)(sum / _referenceSamples) * Mathf.Rad2Deg)).Append('/').Append(S(peak * Mathf.Rad2Deg));
        }

        private static void AppendReferenceWitness(StringBuilder sb, Frame f)
        {
            sb.Append(" footReference(selected-active-rawLocalErrRad-gapRad)=");
            if (!f.ReferenceWitnessValid) { sb.Append("NA"); return; }
            sb.Append(f.ReferenceSelected).Append('/').Append(f.ReferenceActive).Append('/').Append(S(f.RawLocalError)).Append('/').Append(S(f.ReferenceGap))
                .Append(" rawSourceLocal(R-W-A)=").Append(R(f.RawSourceRelR)).Append('/').Append(T(f.RawSourceW)).Append('/').Append(T(f.RawSourceA));
            if (!f.ReferenceActive) return;
            FootRelativeReference.Sample r = f.Reference;
            sb.Append(" referenceRender(sourceEpoch-seq-dt-W-A-alphaLimited-speedLimited-inheritedOverW)=")
                .Append(r.Source.Epoch).Append('/').Append(r.Source.Sequence).Append('/').Append(S(r.Dt)).Append('/')
                .Append(T(r.W)).Append('/').Append(T(r.A)).Append('/').Append(r.AlphaLimited).Append('/').Append(r.SpeedLimited).Append('/').Append(r.InheritedOverSpeed);
            sb.Append(" referenceGapRecovery(active-sourceElapsed-planDt-discardedElapsed)=").Append(r.GapRecovery)
                .Append('/').Append(S(r.SourceElapsed)).Append('/').Append(S(r.Dt)).Append('/').Append(S(r.DiscardedElapsed));
        }

        private static void AppendStepProgress(StringBuilder sb, Frame f)
        {
            sb.Append(" conditioner(stage-rawValid-high-confirmed-heldA)=");
            if (f.ConditionerStage == null || f.ConditionerStage == "UNSAMPLED" || !V(f.ConditionedA)) sb.Append("NA");
            else sb.Append(f.ConditionerStage).Append('/').Append(f.ConditionerRawValid).Append('/').Append(f.ConditionerHigh)
                .Append('/').Append(f.ConditionerConfirmed).Append('/').Append(T(f.ConditionedA));
            sb.Append(" stepErrorRad(fresh-before-lockedNext-currentNext-physical-source-net-closure)=");
            StepProgress p = f.Progress;
            if (!f.Resolved || !p.ReferenceValid) sb.Append("NA");
            else sb.Append(p.FreshPair).Append('/').Append(S(f.Error)).Append('/').Append(S(p.LockedError)).Append('/')
                .Append(S(p.NextError)).Append('/').Append(S(p.PhysicalDelta)).Append('/').Append(S(p.SourceDelta)).Append('/')
                .Append(S(p.NetDelta)).Append('/').Append(S(p.Closure))
                .Append(" nextSource(epoch-seq-time-dt-age)=").Append(p.NextSource.Epoch).Append('/').Append(p.NextSource.Sequence).Append('/')
                .Append(S(p.NextSource.Time)).Append('/').Append(S(p.NextSource.Dt)).Append('/').Append(S(p.NextAge));
            sb.Append(" stepAWorld(pairOnly-actualRelative-residual-pastMinusNextLower-residualMinusLag)=");
            if (!f.Resolved || !p.MechanicalValid) sb.Append("NA");
            else sb.Append(T(p.PairRequestA)).Append('/').Append(T(p.ActualRelativeA)).Append('/').Append(T(p.RelativeResidualA))
                .Append('/').Append(T(p.CommonPastMinusNextA)).Append('/').Append(T(p.ResidualMinusLagA));
        }
    }
}
