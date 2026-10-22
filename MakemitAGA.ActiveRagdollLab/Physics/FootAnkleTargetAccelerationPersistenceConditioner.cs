using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.3.68 Render-sample persistence conditioner for source-relative ankle target angular acceleration.
    ///
    /// This component is source-side only: it never writes Rigidbody state.  It converts the raw Render-derived
    /// target-A into the conditioned target consumed by the frozen v0.3.70 free-space Foot production controller.
    /// The mechanism was originally qualified as the v0.3.68 B arm.
    ///
    /// Rules are intentionally narrow and derived from v0.3.67 attribution rather than tuned by a sweep:
    ///   * |A| <= 20 rad/s^2: pass through immediately.
    ///   * first |A| > 20 sample: suppress as unconfirmed burst.
    ///   * consecutive |A| > 20 samples with direction cosine >= 0.5: confirm persistence and pass the
    ///     2-sample vector mean, bounded to 20 rad/s^2.
    ///   * invalid Render derivative, a low sample, or a direction break cancels persistence immediately.
    ///
    /// The final ankle controller still owns the unchanged MaxRelA30 and MaxTau0.35 limits, so this conditioner
    /// cannot add final actuator authority.
    /// </summary>
    internal sealed class FootAnkleTargetAccelerationPersistenceConditioner
    {
        internal const string Mode = "ANKLE_TARGET_A_PERSISTENCE_CONDITIONER_V0368";
        internal const float HighThreshold = 20f;
        internal const float AlignmentCosineThreshold = 0.5f;

        private bool _finite = true;
        private string _invalidReason;
        private bool _hasPreviousValid;
        private Vector3 _previousRaw;
        private bool _previousHigh;

        private Vector3 _currentRaw;
        private Vector3 _currentConditioned;
        private float _currentCosine;
        private string _currentStage = "UNSAMPLED";
        private bool _currentRawHigh;
        private bool _currentConfirmed;

        private int _traceRenderSamples;
        private int _traceValidSamples;
        private int _traceInvalidSamples;
        private int _traceLowPassThrough;
        private int _traceRawHigh;
        private int _traceSuppressedFirstHigh;
        private int _traceConfirmedHigh;
        private int _traceDirectionBreak;
        private int _traceConditionedNonZero;
        private float _traceRawSum;
        private float _traceRawMax;
        private float _traceConditionedSum;
        private float _traceConditionedMax;
        private float _traceConfirmedCosineSum;
        private int _traceConfirmedCosineSamples;

        internal Vector3 CurrentConditionedTargetAccelerationLocal => _currentConditioned;
        internal bool Finite => _finite;
        // Existing held Render decision, exposed as values; never parse formatted report text.
        internal string CurrentStage => _currentStage;
        internal bool CurrentRawValid => _hasPreviousValid;
        internal bool CurrentRawHigh => _currentRawHigh;
        internal bool CurrentConfirmed => _currentConfirmed;

        internal void ObserveRenderSample(
            bool traceEnabled,
            bool rawAccelerationValid,
            Vector3 rawAccelerationLocal,
            FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _currentRaw = Vector3.zero;
            _currentConditioned = Vector3.zero;
            _currentCosine = 0f;
            _currentStage = "INVALID_RESET";
            _currentRawHigh = false;
            _currentConfirmed = false;

            if (traceEnabled)
                _traceRenderSamples++;

            if (!rawAccelerationValid || !FiniteVector(rawAccelerationLocal))
            {
                _hasPreviousValid = false;
                _previousRaw = Vector3.zero;
                _previousHigh = false;
                if (traceEnabled)
                    _traceInvalidSamples++;
                CopyToReport(report);
                return;
            }

            _currentRaw = rawAccelerationLocal;
            float rawMagnitude = rawAccelerationLocal.magnitude;
            bool rawHigh = rawMagnitude > HighThreshold;
            _currentRawHigh = rawHigh;

            if (traceEnabled)
            {
                _traceValidSamples++;
                _traceRawSum += rawMagnitude;
                _traceRawMax = Mathf.Max(_traceRawMax, rawMagnitude);
                if (rawHigh) _traceRawHigh++;
            }

            if (!rawHigh)
            {
                // Normal target acceleration never waits for persistence confirmation.
                _currentConditioned = rawAccelerationLocal;
                _currentStage = "LOW_PASS_THROUGH";
                if (traceEnabled)
                    _traceLowPassThrough++;
            }
            else
            {
                bool previousHighValid = _hasPreviousValid && _previousHigh && _previousRaw.sqrMagnitude > 0.000001f;
                float cosine = previousHighValid ? Vector3.Dot(_previousRaw.normalized, rawAccelerationLocal.normalized) : 0f;
                _currentCosine = cosine;
                bool alignedPersistent = previousHighValid && cosine >= AlignmentCosineThreshold;

                if (alignedPersistent)
                {
                    Vector3 mean2 = (_previousRaw + rawAccelerationLocal) * 0.5f;
                    _currentConditioned = Vector3.ClampMagnitude(mean2, HighThreshold);
                    _currentStage = "HIGH_PERSISTENT_CONFIRMED_MEAN2";
                    _currentConfirmed = true;
                    if (traceEnabled)
                    {
                        _traceConfirmedHigh++;
                        _traceConfirmedCosineSum += cosine;
                        _traceConfirmedCosineSamples++;
                    }
                }
                else
                {
                    // The current high sample becomes a new candidate onset.  Suppressing this one sample is the
                    // exact mechanism intended to reject the single-packet population exposed by v0.3.67.
                    _currentConditioned = Vector3.zero;
                    _currentStage = previousHighValid ? "HIGH_DIRECTION_BREAK_SUPPRESS" : "HIGH_FIRST_SAMPLE_SUPPRESS";
                    if (traceEnabled)
                    {
                        _traceSuppressedFirstHigh++;
                        if (previousHighValid) _traceDirectionBreak++;
                    }
                }
            }

            if (!FiniteVector(_currentConditioned))
            {
                _finite = false;
                _invalidReason = "NON_FINITE_CONDITIONED_TARGET_A";
                _currentConditioned = Vector3.zero;
                _currentStage = "NON_FINITE_RESET";
                _hasPreviousValid = false;
                _previousRaw = Vector3.zero;
                _previousHigh = false;
                if (traceEnabled)
                    _traceInvalidSamples++;
                CopyToReport(report);
                return;
            }

            if (traceEnabled)
            {
                float conditionedMagnitude = _currentConditioned.magnitude;
                _traceConditionedSum += conditionedMagnitude;
                _traceConditionedMax = Mathf.Max(_traceConditionedMax, conditionedMagnitude);
                if (conditionedMagnitude > 0.000001f) _traceConditionedNonZero++;
            }

            _previousRaw = rawAccelerationLocal;
            _previousHigh = rawHigh;
            _hasPreviousValid = true;
            CopyToReport(report);
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            // Diagnostic reset only.  Preserve source-side persistence history so arl_trace(on) cannot create an
            // artificial high-A first-sample suppression or confirmation event.
            _traceRenderSamples = 0;
            _traceValidSamples = 0;
            _traceInvalidSamples = 0;
            _traceLowPassThrough = 0;
            _traceRawHigh = 0;
            _traceSuppressedFirstHigh = 0;
            _traceConfirmedHigh = 0;
            _traceDirectionBreak = 0;
            _traceConditionedNonZero = 0;
            _traceRawSum = 0f;
            _traceRawMax = 0f;
            _traceConditionedSum = 0f;
            _traceConditionedMax = 0f;
            _traceConfirmedCosineSum = 0f;
            _traceConfirmedCosineSamples = 0;
            CopyToReport(report);
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode + "/finite=" + _finite + "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.TargetAccelerationConditionerCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.TargetAccelerationConditionerTraceSummary : "<none>");
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null)
                return;

            report.TargetAccelerationConditionerMode = Mode;
            report.TargetAccelerationConditionerFinite = _finite;
            report.TargetAccelerationConditionerInvalidReason = _invalidReason;
            report.TargetAccelerationConditionerCurrentSummary =
                "stage=" + _currentStage +
                "/raw=" + V(_currentRaw) +
                "/rawMag=" + F(_currentRaw.magnitude) +
                "/conditioned=" + V(_currentConditioned) +
                "/conditionedMag=" + F(_currentConditioned.magnitude) +
                "/rawHigh=" + _currentRawHigh +
                "/confirmed=" + _currentConfirmed +
                "/prevCos=" + F(_currentCosine);

            float validDenom = Mathf.Max(1, _traceValidSamples);
            float confirmedDenom = Mathf.Max(1, _traceConfirmedCosineSamples);
            report.TargetAccelerationConditionerTraceSummary =
                "render=" + _traceRenderSamples +
                "/valid=" + _traceValidSamples +
                "/invalid=" + _traceInvalidSamples +
                "/rawMeanMax=" + F(_traceRawSum / validDenom) + "-" + F(_traceRawMax) +
                "/rawHigh20=" + _traceRawHigh +
                "/lowPass=" + _traceLowPassThrough +
                "/suppressedFirstHigh=" + _traceSuppressedFirstHigh +
                "/confirmedHigh=" + _traceConfirmedHigh +
                "/directionBreak=" + _traceDirectionBreak +
                "/conditionedNonZero=" + _traceConditionedNonZero +
                "/conditionedMeanMax=" + F(_traceConditionedSum / validDenom) + "-" + F(_traceConditionedMax) +
                "/confirmedCosMean=" + F(_traceConfirmedCosineSum / confirmedDenom) +
                "/thresholds(high-alignCos)=" + F(HighThreshold) + "-" + F(AlignmentCosineThreshold);
        }

        private static bool FiniteVector(Vector3 value) =>
            FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z);

        private static bool FiniteFloat(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
