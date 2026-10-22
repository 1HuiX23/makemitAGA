using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.17.1 narrow predictive tail-capture causal A/B layered on top of the frozen v0.3.73 60/80 law.
    ///
    /// A is an exact no-op over the frozen V0373 selected MaxA. B may only de-clamp the already-computed tether command
    /// when the existing V0373 capture window is active AND a one-FixedUpdate radial predictor says the frozen 80 m/s^2
    /// cap is insufficient to stay below the unchanged 30 mm hard gate. B never invents a new acceleration direction or
    /// amplifies beyond the existing raw tether command; it only permits more of that same command, bounded at 140 m/s^2.
    ///
    /// The predictor explicitly subtracts measured parent-transport demand from the radial component of the baseline
    /// applied command before estimating relative closing acceleration. This reproduces the signature seen in the two
    /// repeated safety-censored tail events: the old 80 cap mostly followed the rapidly accelerating Lower and left too
    /// little relative closing acceleration to absorb >1 m/s radial opening within one FixedUpdate.
    ///
    /// The 30 mm hard gate remains the final fail-close and is intentionally evaluated before this selector on the next
    /// FixedUpdate. This class therefore has to act one step earlier or not at all.
    /// </summary>
    internal sealed class FootAnkleTranslationPredictiveTailCaptureIsolation
    {
        internal const string Mode = "ANKLE_TRANSLATION_PREDICTIVE_TAIL_CAPTURE_CAUSAL_AB_V04171";
        internal const string ArmA = "A_V04171_FROZEN_V0373_60_80";
        internal const string ArmB = "B_V04171_PREDICTIVE_NEXT_STEP_DECLAMP_MAXA140";

        // 140 is a deliberately experimental ceiling, not a promoted production parameter. It minimally envelopes
        // the two repeated pre-abort raw-demand tails observed before v0.4.17.1 (~121.9 and ~136.8 m/s^2) while
        // remaining bounded. The selector never exceeds rawMagnitude, so it cannot create extra command by itself.
        internal const float PredictiveReserveMaxAcceleration = 140f;
        internal const float Epsilon = 0.000001f;
        internal const int MinTraceSamples = 300;
        internal const int MinReserveActivationSamples = 1;

        private string _arm = ArmA;
        private bool _finite = true;
        private string _invalidReason;

        private bool _currentDynamic;
        private bool _currentBaseCaptureEligible;
        private bool _currentPredictedBreach;
        private bool _currentReserveActive;
        private bool _currentPredictedFinalBreach;
        private float _currentAnchorMeters;
        private float _currentRadialOpeningRate;
        private float _currentParentTransportRadialClosing;
        private float _currentRawRadialClosing;
        private float _currentRawMagnitude;
        private float _currentAlignment;
        private float _currentBaseSelectedMaxAcceleration = FootAnkleTranslationCaptureAuthorityIsolation.BaselineMaxAcceleration;
        private float _currentFinalSelectedMaxAcceleration = FootAnkleTranslationCaptureAuthorityIsolation.BaselineMaxAcceleration;
        private float _currentBaselineAppliedRadialClosing;
        private float _currentFinalAppliedRadialClosing;
        private float _currentEstimatedBaselineRelativeClosingAcceleration;
        private float _currentEstimatedFinalRelativeClosingAcceleration;
        private float _currentPredictedBaselineNextAnchorMeters;
        private float _currentPredictedFinalNextAnchorMeters;
        private int _currentFixedSequence;

        private int _traceSamples;
        private int _traceDynamicSamples;
        private int _traceBaseCaptureEligibleSamples;
        private int _tracePredictedBreachSamples;
        private int _traceReserveActiveSamples;
        private int _tracePredictedFinalBreachSamples;
        private int _traceUnexpectedElevationSamples;
        private int _traceFirstReserveFixed;
        private int _traceLastReserveFixed;
        private float _tracePredictedBaselineNextAnchorMax;
        private float _tracePredictedFinalNextAnchorMax;
        private float _traceFinalSelectedMaxAccelerationMax;
        private float _traceReserveAnchorSum;
        private float _traceReserveAnchorMax;
        private float _traceReserveOpeningSum;
        private float _traceReserveOpeningMax;
        private float _traceReserveRawSum;
        private float _traceReserveRawMax;

        // UI-only notification latch. This never participates in physics or qualification.
        // It prevents the red console notice from repeating every FixedUpdate while one
        // continuous predictive-reserve episode remains active.
        private bool _captureConsoleNoticeLatched;

        internal string Arm => _arm;
        internal bool Enabled => string.Equals(_arm, ArmB, System.StringComparison.Ordinal);
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;
        internal float CurrentFinalSelectedMaxAcceleration => _currentFinalSelectedMaxAcceleration;

        internal bool SetArm(string arm, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, ArmA, System.StringComparison.Ordinal) && !string.Equals(arm, ArmB, System.StringComparison.Ordinal))
            {
                error = "UNKNOWN_PREDICTIVE_TAIL_CAPTURE_ARM:" + (arm ?? "<null>");
                return false;
            }

            _arm = arm;
            _finite = true;
            _invalidReason = null;
            ClearCurrent(report);
            return true;
        }

        internal float Evaluate(
            bool traceEnabled,
            int fixedSequence,
            float fixedDeltaTime,
            bool dynamicActive,
            float anchorMeters,
            float radialOpeningRate,
            float parentTransportRadialClosing,
            float rawRadialClosing,
            float rawMagnitude,
            float baseSelectedMaxAcceleration,
            bool baseCaptureEligible,
            FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _currentFixedSequence = fixedSequence;
            _currentDynamic = dynamicActive;
            _currentBaseCaptureEligible = baseCaptureEligible;
            _currentAnchorMeters = anchorMeters;
            _currentRadialOpeningRate = radialOpeningRate;
            _currentParentTransportRadialClosing = parentTransportRadialClosing;
            _currentRawRadialClosing = rawRadialClosing;
            _currentRawMagnitude = rawMagnitude;
            _currentBaseSelectedMaxAcceleration = baseSelectedMaxAcceleration;

            if (!FiniteFloat(fixedDeltaTime) || fixedDeltaTime <= Epsilon ||
                !FiniteFloat(anchorMeters) || !FiniteFloat(radialOpeningRate) ||
                !FiniteFloat(parentTransportRadialClosing) || !FiniteFloat(rawRadialClosing) ||
                !FiniteFloat(rawMagnitude) || !FiniteFloat(baseSelectedMaxAcceleration))
            {
                return Invalidate("NON_FINITE_PREDICTIVE_TAIL_CAPTURE_INPUT", baseSelectedMaxAcceleration, report);
            }

            _currentAlignment = rawMagnitude > 0.0001f ? rawRadialClosing / rawMagnitude : 0f;
            if (!FiniteFloat(_currentAlignment))
                return Invalidate("NON_FINITE_PREDICTIVE_TAIL_CAPTURE_ALIGNMENT", baseSelectedMaxAcceleration, report);

            float positiveOpening = Mathf.Max(0f, radialOpeningRate);
            float positiveParentTransport = Mathf.Max(0f, parentTransportRadialClosing);
            float baselineAppliedMagnitude = Mathf.Min(rawMagnitude, Mathf.Max(0f, baseSelectedMaxAcceleration));
            _currentBaselineAppliedRadialClosing = Mathf.Max(0f, baselineAppliedMagnitude * _currentAlignment);
            _currentEstimatedBaselineRelativeClosingAcceleration = Mathf.Max(0f, _currentBaselineAppliedRadialClosing - positiveParentTransport);
            float dt2Half = 0.5f * fixedDeltaTime * fixedDeltaTime;
            _currentPredictedBaselineNextAnchorMeters = Mathf.Max(0f,
                anchorMeters + positiveOpening * fixedDeltaTime - _currentEstimatedBaselineRelativeClosingAcceleration * dt2Half);

            _currentPredictedBreach =
                dynamicActive &&
                baseCaptureEligible &&
                anchorMeters < FootAnkleConstraintTopologyIsolation.AnchorValidityMeters &&
                _currentPredictedBaselineNextAnchorMeters >= FootAnkleConstraintTopologyIsolation.AnchorValidityMeters &&
                rawMagnitude > baseSelectedMaxAcceleration + Epsilon;

            float finalSelected = baseSelectedMaxAcceleration;
            if (Enabled && _currentPredictedBreach)
                finalSelected = Mathf.Min(PredictiveReserveMaxAcceleration, Mathf.Max(baseSelectedMaxAcceleration, rawMagnitude));

            _currentFinalSelectedMaxAcceleration = finalSelected;
            _currentReserveActive = finalSelected > baseSelectedMaxAcceleration + Epsilon;

            float finalAppliedMagnitude = Mathf.Min(rawMagnitude, Mathf.Max(0f, finalSelected));
            _currentFinalAppliedRadialClosing = Mathf.Max(0f, finalAppliedMagnitude * _currentAlignment);
            _currentEstimatedFinalRelativeClosingAcceleration = Mathf.Max(0f, _currentFinalAppliedRadialClosing - positiveParentTransport);
            _currentPredictedFinalNextAnchorMeters = Mathf.Max(0f,
                anchorMeters + positiveOpening * fixedDeltaTime - _currentEstimatedFinalRelativeClosingAcceleration * dt2Half);
            _currentPredictedFinalBreach = _currentReserveActive &&
                _currentPredictedFinalNextAnchorMeters >= FootAnkleConstraintTopologyIsolation.AnchorValidityMeters;

            if (!FiniteFloat(_currentPredictedBaselineNextAnchorMeters) || !FiniteFloat(_currentPredictedFinalNextAnchorMeters) ||
                !FiniteFloat(_currentBaselineAppliedRadialClosing) || !FiniteFloat(_currentFinalAppliedRadialClosing) ||
                !FiniteFloat(_currentEstimatedBaselineRelativeClosingAcceleration) || !FiniteFloat(_currentEstimatedFinalRelativeClosingAcceleration) ||
                !FiniteFloat(_currentFinalSelectedMaxAcceleration))
            {
                return Invalidate("NON_FINITE_PREDICTIVE_TAIL_CAPTURE_RESULT", baseSelectedMaxAcceleration, report);
            }

            if (_currentFinalSelectedMaxAcceleration > PredictiveReserveMaxAcceleration + 0.0001f)
                return Invalidate("PREDICTIVE_TAIL_CAPTURE_MAXA_CONTRACT_BREACH", baseSelectedMaxAcceleration, report);

            bool unexpectedElevation = _currentReserveActive && (!Enabled || !_currentPredictedBreach || !baseCaptureEligible);

            // Pure operator feedback: when B actually enters a predictive reserve episode, emit one
            // red BepInEx console line on the rising edge so the tester immediately knows the run has
            // captured the rare tail condition. No threshold, selector, force, report, or verdict uses
            // this latch. It is intentionally reset after the episode ends so a later independent
            // capture can be announced as well.
            bool captureNoticeActive = Enabled && _currentPredictedBreach && _currentReserveActive;
            if (captureNoticeActive && !_captureConsoleNoticeLatched)
            {
                Plugin.Logger?.LogError(
                    "[ARL][V04171][已捕捉到] side=" + (report != null ? (report.Side ?? "<none>") : "<none>") +
                    " | fixed=" + fixedSequence +
                    " | anchorMm=" + F(anchorMeters * 1000f) +
                    " | radialOpen=" + F(radialOpeningRate) +
                    " | rawA=" + F(rawMagnitude) +
                    " | predNextMm(base-final)=" + F(_currentPredictedBaselineNextAnchorMeters * 1000f) + "-" + F(_currentPredictedFinalNextAnchorMeters * 1000f) +
                    " | MaxA(base-final)=" + F(baseSelectedMaxAcceleration) + "-" + F(_currentFinalSelectedMaxAcceleration) +
                    " | predictedFinalBreach=" + _currentPredictedFinalBreach);
            }
            _captureConsoleNoticeLatched = captureNoticeActive;

            if (traceEnabled)
            {
                _traceSamples++;
                if (dynamicActive) _traceDynamicSamples++;
                if (baseCaptureEligible) _traceBaseCaptureEligibleSamples++;
                if (_currentPredictedBreach) _tracePredictedBreachSamples++;
                if (_currentReserveActive)
                {
                    _traceReserveActiveSamples++;
                    if (_traceFirstReserveFixed == 0) _traceFirstReserveFixed = fixedSequence;
                    _traceLastReserveFixed = fixedSequence;
                    _traceReserveAnchorSum += anchorMeters;
                    _traceReserveAnchorMax = Mathf.Max(_traceReserveAnchorMax, anchorMeters);
                    _traceReserveOpeningSum += radialOpeningRate;
                    _traceReserveOpeningMax = Mathf.Max(_traceReserveOpeningMax, radialOpeningRate);
                    _traceReserveRawSum += rawMagnitude;
                    _traceReserveRawMax = Mathf.Max(_traceReserveRawMax, rawMagnitude);
                }
                if (_currentPredictedFinalBreach) _tracePredictedFinalBreachSamples++;
                if (unexpectedElevation) _traceUnexpectedElevationSamples++;
                _tracePredictedBaselineNextAnchorMax = Mathf.Max(_tracePredictedBaselineNextAnchorMax, _currentPredictedBaselineNextAnchorMeters);
                _tracePredictedFinalNextAnchorMax = Mathf.Max(_tracePredictedFinalNextAnchorMax, _currentPredictedFinalNextAnchorMeters);
                _traceFinalSelectedMaxAccelerationMax = Mathf.Max(_traceFinalSelectedMaxAccelerationMax, _currentFinalSelectedMaxAcceleration);
            }

            CopyToReport(report);
            return _currentFinalSelectedMaxAcceleration;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceSamples = _traceDynamicSamples = _traceBaseCaptureEligibleSamples = 0;
            _tracePredictedBreachSamples = _traceReserveActiveSamples = _tracePredictedFinalBreachSamples = 0;
            _traceUnexpectedElevationSamples = 0;
            _traceFirstReserveFixed = _traceLastReserveFixed = 0;
            _tracePredictedBaselineNextAnchorMax = _tracePredictedFinalNextAnchorMax = 0f;
            _traceFinalSelectedMaxAccelerationMax = 0f;
            _traceReserveAnchorSum = _traceReserveAnchorMax = 0f;
            _traceReserveOpeningSum = _traceReserveOpeningMax = 0f;
            _traceReserveRawSum = _traceReserveRawMax = 0f;
            _captureConsoleNoticeLatched = false;
            ClearCurrent(report);
        }

        internal void ClearCurrent(FootAnkleFoundationReport report)
        {
            _currentDynamic = false;
            _currentBaseCaptureEligible = false;
            _currentPredictedBreach = false;
            _currentReserveActive = false;
            _currentPredictedFinalBreach = false;
            _currentAnchorMeters = _currentRadialOpeningRate = _currentParentTransportRadialClosing = 0f;
            _currentRawRadialClosing = _currentRawMagnitude = _currentAlignment = 0f;
            _currentBaseSelectedMaxAcceleration = FootAnkleTranslationCaptureAuthorityIsolation.BaselineMaxAcceleration;
            _currentFinalSelectedMaxAcceleration = FootAnkleTranslationCaptureAuthorityIsolation.BaselineMaxAcceleration;
            _currentBaselineAppliedRadialClosing = _currentFinalAppliedRadialClosing = 0f;
            _currentEstimatedBaselineRelativeClosingAcceleration = _currentEstimatedFinalRelativeClosingAcceleration = 0f;
            _currentPredictedBaselineNextAnchorMeters = _currentPredictedFinalNextAnchorMeters = 0f;
            _currentFixedSequence = 0;
            _captureConsoleNoticeLatched = false;
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            report.TranslationPredictiveTailCaptureMode = Mode;
            report.TranslationPredictiveTailCaptureArm = _arm;
            report.TranslationPredictiveTailCaptureFinite = _finite;
            report.TranslationPredictiveTailCaptureInvalidReason = _invalidReason;
            report.TranslationPredictiveTailCaptureEnabled = Enabled;
            report.TranslationPredictiveTailCaptureTraceSamples = _traceSamples;
            report.TranslationPredictiveTailCaptureDynamicSamples = _traceDynamicSamples;
            report.TranslationPredictiveTailCaptureBaseEligibleSamples = _traceBaseCaptureEligibleSamples;
            report.TranslationPredictiveTailCapturePredictedBreachSamples = _tracePredictedBreachSamples;
            report.TranslationPredictiveTailCaptureReserveActiveSamples = _traceReserveActiveSamples;
            report.TranslationPredictiveTailCapturePredictedFinalBreachSamples = _tracePredictedFinalBreachSamples;
            report.TranslationPredictiveTailCaptureUnexpectedElevationSamples = _traceUnexpectedElevationSamples;
            report.TranslationPredictiveTailCapturePredictedBaselineAnchorMaxMeters = _tracePredictedBaselineNextAnchorMax;
            report.TranslationPredictiveTailCapturePredictedFinalAnchorMaxMeters = _tracePredictedFinalNextAnchorMax;
            report.TranslationPredictiveTailCaptureFinalSelectedMaxAccelerationMax = _traceFinalSelectedMaxAccelerationMax;
            report.TranslationPredictiveTailCaptureFirstReserveFixed = _traceFirstReserveFixed;
            report.TranslationPredictiveTailCaptureLastReserveFixed = _traceLastReserveFixed;

            report.TranslationPredictiveTailCaptureCurrentSummary =
                "arm=" + _arm +
                "/enabled=" + Enabled +
                "/fixed=" + _currentFixedSequence +
                "/dynamic=" + _currentDynamic +
                "/baseEligible=" + _currentBaseCaptureEligible +
                "/anchorMm=" + F(_currentAnchorMeters * 1000f) +
                "/radialOpen=" + F(_currentRadialOpeningRate) +
                "/transportRadial=" + F(_currentParentTransportRadialClosing) +
                "/rawRadial=" + F(_currentRawRadialClosing) +
                "/rawAbs=" + F(_currentRawMagnitude) +
                "/align=" + F(_currentAlignment) +
                "/baseMaxA=" + F(_currentBaseSelectedMaxAcceleration) +
                "/finalMaxA=" + F(_currentFinalSelectedMaxAcceleration) +
                "/baseRadialApplied=" + F(_currentBaselineAppliedRadialClosing) +
                "/finalRadialApplied=" + F(_currentFinalAppliedRadialClosing) +
                "/estRelCloseA(base-final)=" + F(_currentEstimatedBaselineRelativeClosingAcceleration) + "-" + F(_currentEstimatedFinalRelativeClosingAcceleration) +
                "/predNextMm(base-final)=" + F(_currentPredictedBaselineNextAnchorMeters * 1000f) + "-" + F(_currentPredictedFinalNextAnchorMeters * 1000f) +
                "/predictedBreach=" + _currentPredictedBreach +
                "/reserveActive=" + _currentReserveActive +
                "/predictedFinalBreach=" + _currentPredictedFinalBreach;

            float reserveDenom = Mathf.Max(1, _traceReserveActiveSamples);
            report.TranslationPredictiveTailCaptureTraceSummary =
                "n=" + _traceSamples +
                "/dynamic=" + _traceDynamicSamples +
                "/baseEligible=" + _traceBaseCaptureEligibleSamples +
                "/predictedBreach=" + _tracePredictedBreachSamples +
                "/reserveActive=" + _traceReserveActiveSamples +
                "/predictedFinalBreach=" + _tracePredictedFinalBreachSamples +
                "/unexpectedElevation=" + _traceUnexpectedElevationSamples +
                "/predAnchorMaxMm(base-final)=" + F(_tracePredictedBaselineNextAnchorMax * 1000f) + "-" + F(_tracePredictedFinalNextAnchorMax * 1000f) +
                "/finalSelectedMaxAMax=" + F(_traceFinalSelectedMaxAccelerationMax) +
                "/reserveMeanMax(anchorMm-open-raw)=" +
                    F((_traceReserveAnchorSum / reserveDenom) * 1000f) + "-" + F(_traceReserveAnchorMax * 1000f) + "/" +
                    F(_traceReserveOpeningSum / reserveDenom) + "-" + F(_traceReserveOpeningMax) + "/" +
                    F(_traceReserveRawSum / reserveDenom) + "-" + F(_traceReserveRawMax) +
                "/firstLastReserve=" + _traceFirstReserveFixed + "-" + _traceLastReserveFixed +
                "/constants(gateMm-baseCaptureMaxA-predictiveMaxA)=" +
                    F(FootAnkleConstraintTopologyIsolation.AnchorValidityMeters * 1000f) + "-" +
                    F(FootAnkleTranslationCaptureAuthorityIsolation.CaptureMaxAcceleration) + "-" +
                    F(PredictiveReserveMaxAcceleration) +
                "/model=ONE_STEP_RADIAL_WITH_MEASURED_PARENT_TRANSPORT_AND_BASE_APPLIED_RADIAL";
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/arm=" + _arm +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.TranslationPredictiveTailCaptureCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.TranslationPredictiveTailCaptureTraceSummary : "<none>");
        }

        private float Invalidate(string reason, float fallback, FootAnkleFoundationReport report)
        {
            _finite = false;
            _invalidReason = reason;
            _currentFinalSelectedMaxAcceleration = fallback;
            CopyToReport(report);
            return fallback;
        }

        private static bool FiniteFloat(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
    }
}
