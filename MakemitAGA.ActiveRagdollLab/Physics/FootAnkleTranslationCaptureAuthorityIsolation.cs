using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.3.73 frozen production translation-tail capture authority.
    ///
    /// v0.3.71 proved the narrow MaxA60->80 reserve physically activates bilaterally and materially compresses the
    /// >15/>20/>25 mm tail without material downstream regression. v0.3.72 then repeated the exact policy in a longer
    /// confirmation session: the reserve activated again and both Foot translation maxima stayed around 15 mm; the run
    /// ended only on the independent Left axial angular emergency envelope.  v0.3.73 therefore promotes the unchanged
    /// conditional law into the free-space production contract.  MaxA80 is NOT a new global cap: ordinary samples still
    /// use MaxA60, and 80 is selected only inside the same pre-registered capture-risk window.
    /// </summary>
    internal sealed class FootAnkleTranslationCaptureAuthorityIsolation
    {
        internal const string Mode = "ANKLE_TRANSLATION_TAIL_CONDITIONAL_CAPTURE_AUTHORITY_PRODUCTION_V0373";
        internal const string Baseline = "V0370_BASE_MAXA60";
        internal const string Production = "V0373_PRODUCTION_CONDITIONAL_MAXA80_CAPTURE";

        internal const float BaselineMaxAcceleration = 60f;
        internal const float CaptureMaxAcceleration = 80f;
        internal const float RiskAnchorMinMeters = 0.005f;
        internal const float RiskRadialOpeningMin = 0.25f;
        internal const float RiskComOpeningMin = 0.25f;
        internal const float RawRadialClosingMin = 60f;
        internal const float RawClosingAlignmentMin = 0.75f;
        internal const float ParentTransportRadialMin = 0f;

        private bool _finite = true;
        private string _invalidReason;

        private bool _currentDynamic;
        private bool _currentRiskAnchor;
        private bool _currentOpening;
        private bool _currentComOpening;
        private bool _currentRawRadialDemand;
        private bool _currentRawAlignment;
        private bool _currentParentNonOpposing;
        private bool _currentEligible;
        private bool _currentReserveActive;
        private bool _currentWouldClamp60;
        private bool _currentWouldClampSelected;
        private float _currentAnchorMeters;
        private float _currentRadialOpening;
        private float _currentComOpeningRate;
        private float _currentTransportRadialClosing;
        private float _currentRawRadialClosing;
        private float _currentRawMagnitude;
        private float _currentRawAlignmentValue;
        private float _currentSelectedMaxAcceleration = BaselineMaxAcceleration;

        private int _traceSamples;
        private int _traceDynamic;
        private int _traceRiskAnchor;
        private int _traceOpening;
        private int _traceComOpening;
        private int _traceEligible;
        private int _traceReserveActive;
        private int _traceWouldClamp60;
        private int _traceWouldClampSelected;
        private int _traceEligibleWouldClamp60;
        private float _traceEligibleAnchorSum;
        private float _traceEligibleAnchorMax;
        private float _traceEligibleRadialOpeningSum;
        private float _traceEligibleComOpeningSum;
        private float _traceEligibleRawRadialSum;
        private float _traceEligibleRawMagnitudeSum;
        private float _traceEligibleAlignmentSum;

        internal string Arm => Production;
        internal bool IsProductionPolicy => true;
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;
        internal float CurrentSelectedMaxAcceleration => _currentSelectedMaxAcceleration;
        internal bool CurrentEligible => _currentEligible;
        internal bool CurrentReserveActive => _currentReserveActive;

        internal static float SelectMaxAcceleration(
            bool dynamicActive,
            float anchorMeters,
            float radialOpeningRate,
            float comOpeningRate,
            float parentTransportRadialClosing,
            float rawRadialClosing,
            float rawMagnitude,
            out bool eligible)
        {
            float alignment = rawMagnitude > 0.0001f ? rawRadialClosing / rawMagnitude : 0f;
            eligible =
                dynamicActive &&
                anchorMeters >= RiskAnchorMinMeters &&
                anchorMeters < FootAnkleConstraintTopologyIsolation.AnchorValidityMeters &&
                radialOpeningRate >= RiskRadialOpeningMin &&
                comOpeningRate >= RiskComOpeningMin &&
                rawRadialClosing > RawRadialClosingMin &&
                alignment >= RawClosingAlignmentMin &&
                parentTransportRadialClosing >= ParentTransportRadialMin;
            return eligible ? CaptureMaxAcceleration : BaselineMaxAcceleration;
        }

        internal float Evaluate(
            bool traceEnabled,
            bool dynamicActive,
            float anchorMeters,
            float radialOpeningRate,
            float comOpeningRate,
            float parentTransportRadialClosing,
            float rawRadialClosing,
            float rawMagnitude,
            FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _currentDynamic = dynamicActive;
            _currentAnchorMeters = anchorMeters;
            _currentRadialOpening = radialOpeningRate;
            _currentComOpeningRate = comOpeningRate;
            _currentTransportRadialClosing = parentTransportRadialClosing;
            _currentRawRadialClosing = rawRadialClosing;
            _currentRawMagnitude = rawMagnitude;
            _currentRawAlignmentValue = rawMagnitude > 0.0001f ? rawRadialClosing / rawMagnitude : 0f;

            if (!FiniteFloat(anchorMeters) || !FiniteFloat(radialOpeningRate) || !FiniteFloat(comOpeningRate) ||
                !FiniteFloat(parentTransportRadialClosing) || !FiniteFloat(rawRadialClosing) || !FiniteFloat(rawMagnitude) ||
                !FiniteFloat(_currentRawAlignmentValue))
            {
                _finite = false;
                _invalidReason = "NON_FINITE_TRANSLATION_CAPTURE_AUTHORITY_INPUT";
                ClearEligibility();
                _currentSelectedMaxAcceleration = BaselineMaxAcceleration;
                CopyToReport(report);
                return BaselineMaxAcceleration;
            }

            _currentRiskAnchor = dynamicActive && anchorMeters >= RiskAnchorMinMeters && anchorMeters < FootAnkleConstraintTopologyIsolation.AnchorValidityMeters;
            _currentOpening = radialOpeningRate >= RiskRadialOpeningMin;
            _currentComOpening = comOpeningRate >= RiskComOpeningMin;
            _currentRawRadialDemand = rawRadialClosing > RawRadialClosingMin;
            _currentRawAlignment = _currentRawAlignmentValue >= RawClosingAlignmentMin;
            _currentParentNonOpposing = parentTransportRadialClosing >= ParentTransportRadialMin;
            _currentSelectedMaxAcceleration = SelectMaxAcceleration(
                dynamicActive, anchorMeters, radialOpeningRate, comOpeningRate, parentTransportRadialClosing,
                rawRadialClosing, rawMagnitude, out _currentEligible);
            _currentReserveActive = _currentEligible;
            _currentWouldClamp60 = rawMagnitude > BaselineMaxAcceleration;
            _currentWouldClampSelected = rawMagnitude > _currentSelectedMaxAcceleration;

            if (traceEnabled)
            {
                _traceSamples++;
                if (dynamicActive) _traceDynamic++;
                if (_currentRiskAnchor) _traceRiskAnchor++;
                if (_currentOpening) _traceOpening++;
                if (_currentComOpening) _traceComOpening++;
                if (_currentWouldClamp60) _traceWouldClamp60++;
                if (_currentWouldClampSelected) _traceWouldClampSelected++;
                if (_currentEligible)
                {
                    _traceEligible++;
                    if (_currentWouldClamp60) _traceEligibleWouldClamp60++;
                    _traceEligibleAnchorSum += anchorMeters;
                    _traceEligibleAnchorMax = Mathf.Max(_traceEligibleAnchorMax, anchorMeters);
                    _traceEligibleRadialOpeningSum += radialOpeningRate;
                    _traceEligibleComOpeningSum += comOpeningRate;
                    _traceEligibleRawRadialSum += rawRadialClosing;
                    _traceEligibleRawMagnitudeSum += rawMagnitude;
                    _traceEligibleAlignmentSum += _currentRawAlignmentValue;
                    _traceReserveActive++;
                }
            }

            CopyToReport(report);
            return _currentSelectedMaxAcceleration;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceSamples = _traceDynamic = _traceRiskAnchor = _traceOpening = _traceComOpening = 0;
            _traceEligible = _traceReserveActive = _traceWouldClamp60 = _traceWouldClampSelected = _traceEligibleWouldClamp60 = 0;
            _traceEligibleAnchorSum = _traceEligibleAnchorMax = 0f;
            _traceEligibleRadialOpeningSum = _traceEligibleComOpeningSum = 0f;
            _traceEligibleRawRadialSum = _traceEligibleRawMagnitudeSum = _traceEligibleAlignmentSum = 0f;
            ClearCurrent(report);
        }

        internal void ClearCurrent(FootAnkleFoundationReport report)
        {
            _currentDynamic = false;
            _currentAnchorMeters = _currentRadialOpening = _currentComOpeningRate = 0f;
            _currentTransportRadialClosing = _currentRawRadialClosing = _currentRawMagnitude = _currentRawAlignmentValue = 0f;
            ClearEligibility();
            _currentSelectedMaxAcceleration = BaselineMaxAcceleration;
            CopyToReport(report);
        }

        private void ClearEligibility()
        {
            _currentRiskAnchor = _currentOpening = _currentComOpening = false;
            _currentRawRadialDemand = _currentRawAlignment = _currentParentNonOpposing = false;
            _currentEligible = _currentReserveActive = false;
            _currentWouldClamp60 = _currentWouldClampSelected = false;
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            report.TranslationCaptureAuthorityMode = Mode;
            report.TranslationCaptureAuthorityArm = Arm;
            report.TranslationCaptureAuthorityFinite = _finite;
            report.TranslationCaptureAuthorityInvalidReason = _invalidReason;
            report.TranslationCaptureAuthorityCurrentSummary =
                "arm=" + Arm +
                "/dynamic=" + _currentDynamic +
                "/anchorMm=" + F(_currentAnchorMeters * 1000f) +
                "/radialOpen=" + F(_currentRadialOpening) +
                "/comOpen=" + F(_currentComOpeningRate) +
                "/transportRadialClosing=" + F(_currentTransportRadialClosing) +
                "/rawRadialClosing=" + F(_currentRawRadialClosing) +
                "/rawAbs=" + F(_currentRawMagnitude) +
                "/alignment=" + F(_currentRawAlignmentValue) +
                "/criteria(anchor-open-com-raw-align-transport)=" + _currentRiskAnchor + "-" + _currentOpening + "-" + _currentComOpening + "-" + _currentRawRadialDemand + "-" + _currentRawAlignment + "-" + _currentParentNonOpposing +
                "/eligible=" + _currentEligible +
                "/reserveActive=" + _currentReserveActive +
                "/selectedMaxA=" + F(_currentSelectedMaxAcceleration) +
                "/wouldClamp60=" + _currentWouldClamp60 +
                "/wouldClampSelected=" + _currentWouldClampSelected;

            float eligibleDenom = Mathf.Max(1, _traceEligible);
            report.TranslationCaptureAuthorityTraceSummary =
                "production=True" +
                "/n=" + _traceSamples +
                "/dynamic=" + _traceDynamic +
                "/riskAnchor=" + _traceRiskAnchor +
                "/opening=" + _traceOpening +
                "/comOpening=" + _traceComOpening +
                "/eligible=" + _traceEligible +
                "/reserveActive=" + _traceReserveActive +
                "/wouldClamp60=" + _traceWouldClamp60 +
                "/wouldClampSelected=" + _traceWouldClampSelected +
                "/eligibleWouldClamp60=" + _traceEligibleWouldClamp60 +
                "/eligibleMean(anchorMm-radialOpen-comOpen-rawRadial-rawAbs-align)=" +
                    F((_traceEligibleAnchorSum / eligibleDenom) * 1000f) + "-" +
                    F(_traceEligibleRadialOpeningSum / eligibleDenom) + "-" +
                    F(_traceEligibleComOpeningSum / eligibleDenom) + "-" +
                    F(_traceEligibleRawRadialSum / eligibleDenom) + "-" +
                    F(_traceEligibleRawMagnitudeSum / eligibleDenom) + "-" +
                    F(_traceEligibleAlignmentSum / eligibleDenom) +
                "/eligibleAnchorMaxMm=" + F(_traceEligibleAnchorMax * 1000f) +
                "/constants(base-capture-anchorMinMm-openMin-comMin-rawRadialMin-alignMin-parentRadialMin)=" +
                    F(BaselineMaxAcceleration) + "-" + F(CaptureMaxAcceleration) + "-" + F(RiskAnchorMinMeters * 1000f) + "-" +
                    F(RiskRadialOpeningMin) + "-" + F(RiskComOpeningMin) + "-" + F(RawRadialClosingMin) + "-" +
                    F(RawClosingAlignmentMin) + "-" + F(ParentTransportRadialMin);
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/arm=" + Arm +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.TranslationCaptureAuthorityCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.TranslationCaptureAuthorityTraceSummary : "<none>");
        }

        private static bool FiniteFloat(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
    }
}
