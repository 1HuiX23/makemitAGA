using System;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class RightFootPrecontact
    {
        internal const float MaxExtraSeconds = .250f;
        private const float ExpiryBandMeters = .003f;
        private const float ForecastExpiryBandMeters = .005f;
        private const float PreviousStepReviewHeightMeters = .001f;
        private const float TimeTolerance = .000001f;
        private const int FollowSteps = 4;
        private enum ExpiryMatch { Eligible, NoImpulse, UnverifiedLoad, OutsideBand, NotDescending, MissingGoal, GoalClear, StepOverBudget }

        private readonly Action<string> _log;
        private ExpiryMatch _expiryMatch;
        private Sample _previousAtBase;
        private int _cutoffs, _matches, _extensions;

        internal RightFootPrecontact(Action<string> log = null) { _log = log; }

        private void ResetExpiryCounters() { _cutoffs = _matches = _extensions = 0; }

        private bool FitsExtraStep(float dt) => _current.ExtraSeconds + dt <= MaxExtraSeconds + TimeTolerance;

        // The same held control pose and eight-corner minimum as the landing history.
        // Current step/floor, age, depth, lift and hard ankle gap were already checked by Evaluate.
        private ExpiryMatch MatchExpiry(in Sample s, out bool predictionOnly, out bool wideForecast)
        {
            predictionOnly = false;
            wideForecast = false;
            if (_current.FirstNonzero == 0 || _current.FirstNonzero >= s.Local) return ExpiryMatch.NoImpulse;
            if (_current.VerifiedUnloaded != _current.Solves || _current.LoadViolations != 0) return ExpiryMatch.UnverifiedLoad;
            return MatchExpiryGeometry(s, FitsExtraStep(s.Step.Dt), out predictionOnly, out wideForecast);
        }

        private static ExpiryMatch MatchExpiryGeometry(in Sample s, bool stepFits, out bool predictionOnly, out bool wideForecast)
        {
            predictionOnly = wideForecast = false;
            if (s.Min < 0f || s.Min > ForecastExpiryBandMeters) return ExpiryMatch.OutsideBand;
            float threshold = -FootContactGeometryLimits.OrdinaryPenetrationMeters;
            bool predictedBelow = s.Predicted < threshold;
            if (s.Min > ExpiryBandMeters)
            {
                // Mark the same physical opportunity in both arms, before formal stance gets first refusal.
                wideForecast = predictedBelow && s.CornerV < 0f && float.IsFinite(s.ControlMin) && stepFits;
                if (!wideForecast) return ExpiryMatch.OutsideBand;
                predictionOnly = s.ControlMin >= threshold;
                return ExpiryMatch.Eligible;
            }
            if (s.CornerV >= 0f) return ExpiryMatch.NotDescending;
            if (!float.IsFinite(s.ControlMin)) return ExpiryMatch.MissingGoal;
            bool goalBelow = s.ControlMin < threshold;
            predictionOnly = !goalBelow && predictedBelow && stepFits;
            if (!goalBelow && !predictedBelow) return ExpiryMatch.GoalClear;
            return stepFits ? ExpiryMatch.Eligible : ExpiryMatch.StepOverBudget;
        }

        private End CheckExtendedBudget(in Sample s)
        {
            if (!float.IsFinite(s.ControlMin)) return End.GoalLost;
            return FitsExtraStep(s.Step.Dt) ? End.None : End.ExtensionLimit;
        }

        private End CheckBaseExpiry(in Sample previous, in Sample s)
        {
            if (_current.Seconds + s.Step.Dt <= MaxSeconds + TimeTolerance) return End.None;
            _current.ExpiryLocal = s.Local;
            _expiryMatch = MatchExpiry(s, out _current.PredictionOnlyAtBase, out _current.WideForecastAtBase);
            _previousAtBase = previous;
            // Evaluate has already checked adjacent clocks, the same floor, gait and hard limits.
            // Both arms record the opportunity. Only B can spend the first extra solve on it.
            _current.PreviousStepCandidate = UsesNearGroundHold(Selected) &&
                (_expiryMatch is ExpiryMatch.GoalClear or ExpiryMatch.NotDescending) &&
                _current.LastUnloadedLocal == previous.Local && s.Min >= 0f && s.Min <= PreviousStepReviewHeightMeters &&
                s.Min < previous.Min && float.IsFinite(s.ControlMin) && FitsExtraStep(s.Step.Dt) &&
                MatchExpiryGeometry(previous, true, out _, out _) == ExpiryMatch.Eligible;
            _cutoffs++;
            if (CanExtendAtBase) _matches++;
            return End.Timeout;
        }

        private bool CanExtendAtBase => _expiryMatch == ExpiryMatch.Eligible ||
            (Selected == Mode.PreviousStepExpiryReview && _current.PreviousStepCandidate);

        // Called only after the ordinary/recovery/miss owner decides the current step.
        internal bool TryExtendAtExpiry(End reason)
        {
            if (reason != End.Timeout || !Physical || _current.Extended ||
                _current.ExpiryLocal != _last.Local || !CanExtendAtBase) return false;
            _current.Extended = true;
            _extensions++;
            WriteExpiry("BASE", "Extend", _last.Local);
            return true;
        }

        private void CloseExpiry(End reason, int local)
        {
            if (_current.ExpiryLocal == 0) return;
            WriteReviewReadback(reason, local);
            WriteExpiry(_current.Extended ? "END" : "BASE", reason.ToString(), local);
            // Compare the first four completed contact-off solves, even when no later stance event is recorded.
            if (reason == End.Handoff || reason == End.Stop || reason == End.Fault ||
                reason == End.Clock || reason == End.Floor || reason == End.Stale || !_last.Geometry || local != _last.Local) return;
            _follow = new ReleaseFollow { Epoch = _current.Epoch, Origin = _current.Origin, Support = _current.Support,
                Release = local, Low = float.NaN, LastMin = float.NaN, LastPredicted = float.NaN,
                FirstMin = float.NaN, FirstCornerV = float.NaN };
            if (reason == End.Lift) FinishFollow("Lift");
        }

        private bool OnWindowFloor(in Sample s) =>
            s.Clock && s.Geometry && s.Support == _current.Support && s.Step.Epoch == _current.Epoch &&
            s.FloorCenter.Equals(_windowFloorCenter) && s.FloorHalf.Equals(_windowFloorHalf);

        // CountSolve and Close are mutually exclusive for the next decision. Emit after formal
        // priority is resolved, including a first-step exit with no subsequent landing history.
        private void WriteReviewReadback(End reason, int local)
        {
            if (!_current.PreviousStepCandidate || !_current.Extended || _current.ExtraSolves != 1 ||
                local != _current.ExpiryLocal + 1 || local != _last.Local || _log == null) return;
            WriteExpiry("NEXT", reason == End.None ? "Continue" : reason.ToString(), local);
        }

        // Event-only output is independent of the first 64 full episode summaries.
        private void WriteExpiry(string phase, string action, int local)
        {
            if (_log == null) return;
            var s = _last;
            bool geometry = OnWindowFloor(s);
            bool nearGround = UsesNearGroundHold(Selected);
            string tag = phase == "NEXT" ? "[ARL][RIGHT_PRECONTACT_BASE_REVIEW_V041630]" : "[ARL][RIGHT_PRECONTACT_EXPIRY_V041550]";
            string previous = "";
            if (nearGround && phase == "BASE")
            {
                var p = _previousAtBase;
                previous = "/previousAtBase-local-min-control-predictedMm-cornerMmSec=" + p.Local + "|" +
                    N(p.Min * 1000f) + "|" + N(p.ControlMin * 1000f) + "|" + N(p.Predicted * 1000f) + "|" + N(p.CornerV * 1000f);
            }
            _log(tag + " phase=" + phase + "/mode=" + Selected +
                "/epoch-origin-support=" + _current.Epoch + "|" + _current.Origin + "|" + _current.Support +
                "/sampleSupport=" + s.Support +
                "/expiry-decision-sample-shared-source=" + _current.ExpiryLocal + "|" + local + "|" + s.Local + "|" + s.Step.Sequence + "|" + s.Source.Sequence +
                "/action=" + action + "/atBaseMatch=" + _expiryMatch + "/gait=" + (s.Gait ?? "NA") +
                "/clock-exactGeometry-fresh=" + s.Clock + "|" + geometry + "|" + s.Fresh +
                "/min-control-predictedMm=" + Mm(geometry, s.Min) + "|" + Mm(geometry, s.ControlMin) + "|" + Mm(geometry, s.Predicted) +
                "/cornerMmSec-ankleGapMm-sourceAgeMs=" + Mm(geometry, s.CornerV) + "|" + Mm(geometry, s.Gap) + "|" + N(s.Age*1000f) +
                "/firstNonzeroLocal-signedNs=" + _current.FirstNonzero + "|" + N(_current.NonzeroJ) +
                "/elapsed-extra-nextDtMs=" + N(_current.Seconds*1000f) + "|" + N(_current.ExtraSeconds*1000f) + "|" + N(s.Step.Dt*1000f) +
                "/extraSolves=" + _current.ExtraSolves + "/predictionOnlyAtBase=" + _current.PredictionOnlyAtBase +
                "/scheduled-unloaded-violations=" + _current.Solves + "|" + _current.VerifiedUnloaded + "|" + _current.LoadViolations +
                (nearGround ? "/nearHoldFirstRiseLocal=" + _current.NearHoldFirstRise + "/previousStepCandidate=" + _current.PreviousStepCandidate : "") +
                "/wideForecastAtBase=" + _current.WideForecastAtBase + previous);
        }

        private static string Mm(bool valid, float value) => valid ? N(value * 1000f) : "NA";

        private struct ReleaseFollow
        {
            internal int Epoch, Origin, Support, Release, Samples, FirstBelow, Last, FirstLocal;
            internal float Low, LastMin, LastPredicted, FirstMin, FirstCornerV;
        }
        private ReleaseFollow _follow;

        private void ObserveFollow(in Sample s, bool continuousFloor, bool newSwing)
        {
            if (!continuousFloor || !s.Geometry || !s.Fresh || !s.PhaseKnown || s.Lift || newSwing)
            {
                FinishFollow("Interrupted");
                return;
            }
            _follow.Samples++;
            if (_follow.Samples == 1)
            {
                _follow.FirstLocal = s.Local;
                _follow.FirstMin = s.Min;
                _follow.FirstCornerV = s.CornerV;
            }
            _follow.Last = s.Local;
            _follow.LastMin = s.Min;
            _follow.LastPredicted = s.Predicted;
            _follow.Low = _follow.Samples == 1 ? s.Min : Mathf.Min(_follow.Low, s.Min);
            if (_follow.FirstBelow == 0 && s.Min < -FootContactGeometryLimits.OrdinaryPenetrationMeters) _follow.FirstBelow = s.Local;
            if (_follow.Samples == FollowSteps) FinishFollow("FourSteps");
        }

        private void FinishFollow(string reason)
        {
            if (_follow.Origin == 0) return;
            var f = _follow;
            _follow = default;
            _log?.Invoke("[ARL][RIGHT_PRECONTACT_AFTER_RELEASE_V041550] mode=" + Selected +
                "/epoch-origin-support=" + f.Epoch + "|" + f.Origin + "|" + f.Support +
                "/release-local=" + f.Release + "/end=" + reason + "/samples-last-firstBelowMinus2=" + f.Samples + "|" + f.Last + "|" + f.FirstBelow +
                "/low-lastMin-lastPredictedMm=" + N(f.Low*1000f) + "|" + N(f.LastMin*1000f) + "|" + N(f.LastPredicted*1000f) +
                "/firstLocal-minMm-cornerMmSec=" + f.FirstLocal + "|" + N(f.FirstMin*1000f) + "|" + N(f.FirstCornerV*1000f));
        }
    }
}
