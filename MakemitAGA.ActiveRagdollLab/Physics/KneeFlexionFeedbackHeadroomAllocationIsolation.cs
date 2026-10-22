using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.78 causal A/B for the residual ~2-fixed-step Knee lag observed after v0.2.77.
    ///
    /// Important correction from the initial design hypothesis: v0.2.75 parent angular transport is added AFTER
    /// the Knee relative flexion command is bounded, so parent transport and Knee relative feedback do NOT share one
    /// final authority pool in v0.2.77. Adding a new shared final clamp would create a new limitation rather than
    /// isolate an existing one.
    ///
    /// The real shared scalar ceiling is inside Knee flexion control itself:
    ///     (P + D feedback) + validated Knee angular-acceleration feedforward -> existing +/- LegKneeMaxA clamp.
    /// A preserves the exact v0.2.77 sum-then-clamp behavior.
    /// B changes only the allocation when A-FF points opposite the instantaneous P+D feedback: opposing Knee A-FF is
    /// suppressed for that fixed step so feedback is not cancelled while the physical Lower is still catching up.
    /// Same-direction A-FF is unchanged. The same +/- LegKneeMaxA final ceiling remains in force.
    ///
    /// This module owns no Rigidbody call and does not alter parent angular transport, positional tether, PCA,
    /// Kp/Kd, A-FF generation/gain/bound, or any Hip control.
    /// </summary>
    internal sealed class KneeFlexionFeedbackHeadroomAllocationIsolation
    {
        internal const string Mode = "KNEE_FLEXION_FEEDBACK_HEADROOM_ALLOCATION_CAUSAL_AB";
        internal const string ArmA = "A_V077_SUM_THEN_CLAMP";
        internal const string ArmB = "B_V078_FEEDBACK_PRIORITY_OPPOSING_AFF_SUPPRESSED";

        private bool _enabled;
        private int _steps;
        private int _activeSteps;
        private int _invalidSteps;
        private int _opposingSteps;
        private int _suppressedOpposingFfSteps;
        private int _sameDirectionSteps;
        private int _originalWouldClampSteps;
        private int _finalClampSteps;
        private float _currentPd;
        private float _currentInputFf;
        private float _currentEffectiveFf;
        private float _currentOriginalRaw;
        private float _currentAllocatedRaw;
        private float _currentApplied;
        private float _currentMaxA;
        private bool _currentOpposing;
        private bool _currentOriginalWouldClamp;
        private bool _currentFinalClamped;
        private float _peakPdAbs;
        private float _peakInputFfAbs;
        private float _peakOriginalRawAbs;
        private float _peakAllocatedRawAbs;
        private float _peakAppliedAbs;
        private float _suppressedFfAbsSum;
        private string _lastEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal string Arm => _enabled ? ArmB : ArmA;

        internal void SetEnabled(bool enabled, SingleLegChainReport report)
        {
            _enabled = enabled;
            ResetDiagnostics(report);
        }

        internal void ResetDiagnostics(SingleLegChainReport report)
        {
            _steps = 0;
            _activeSteps = 0;
            _invalidSteps = 0;
            _opposingSteps = 0;
            _suppressedOpposingFfSteps = 0;
            _sameDirectionSteps = 0;
            _originalWouldClampSteps = 0;
            _finalClampSteps = 0;
            _currentPd = 0f;
            _currentInputFf = 0f;
            _currentEffectiveFf = 0f;
            _currentOriginalRaw = 0f;
            _currentAllocatedRaw = 0f;
            _currentApplied = 0f;
            _currentMaxA = 0f;
            _currentOpposing = false;
            _currentOriginalWouldClamp = false;
            _currentFinalClamped = false;
            _peakPdAbs = 0f;
            _peakInputFfAbs = 0f;
            _peakOriginalRawAbs = 0f;
            _peakAllocatedRawAbs = 0f;
            _peakAppliedAbs = 0f;
            _suppressedFfAbsSum = 0f;
            _lastEvidence = "<none>";
            CopyToReport(report);
        }

        internal float Compose(
            float pdFeedbackSigned,
            float kneeAccelerationFeedforwardSigned,
            float maxAngularAcceleration,
            SingleLegChainReport report,
            out float effectiveFeedforwardSigned,
            out float allocatedRawSigned,
            out bool finalClamped)
        {
            _steps++;
            _currentPd = pdFeedbackSigned;
            _currentInputFf = kneeAccelerationFeedforwardSigned;
            _currentMaxA = Mathf.Max(0f, maxAngularAcceleration);

            if (!IsFinite(pdFeedbackSigned) || !IsFinite(kneeAccelerationFeedforwardSigned) || !IsFinite(maxAngularAcceleration))
            {
                _invalidSteps++;
                effectiveFeedforwardSigned = 0f;
                allocatedRawSigned = 0f;
                finalClamped = false;
                _currentEffectiveFf = 0f;
                _currentOriginalRaw = 0f;
                _currentAllocatedRaw = 0f;
                _currentApplied = 0f;
                _currentOpposing = false;
                _currentOriginalWouldClamp = false;
                _currentFinalClamped = false;
                _lastEvidence = "invalid=True/step=" + _steps;
                CopyToReport(report);
                return 0f;
            }

            _activeSteps++;
            _currentOriginalRaw = pdFeedbackSigned + kneeAccelerationFeedforwardSigned;
            _currentOpposing = pdFeedbackSigned * kneeAccelerationFeedforwardSigned < -1e-6f;
            if (_currentOpposing)
                _opposingSteps++;
            else if (Mathf.Abs(pdFeedbackSigned) > 1e-6f && Mathf.Abs(kneeAccelerationFeedforwardSigned) > 1e-6f)
                _sameDirectionSteps++;

            effectiveFeedforwardSigned = kneeAccelerationFeedforwardSigned;
            if (_enabled && _currentOpposing)
            {
                effectiveFeedforwardSigned = 0f;
                _suppressedOpposingFfSteps++;
                _suppressedFfAbsSum += Mathf.Abs(kneeAccelerationFeedforwardSigned);
            }

            allocatedRawSigned = pdFeedbackSigned + effectiveFeedforwardSigned;
            float applied = Mathf.Clamp(allocatedRawSigned, -_currentMaxA, _currentMaxA);
            finalClamped = Mathf.Abs(allocatedRawSigned - applied) > 0.0001f;

            _currentEffectiveFf = effectiveFeedforwardSigned;
            _currentAllocatedRaw = allocatedRawSigned;
            _currentApplied = applied;
            _currentOriginalWouldClamp = Mathf.Abs(_currentOriginalRaw) > _currentMaxA + 0.0001f;
            _currentFinalClamped = finalClamped;
            if (_currentOriginalWouldClamp) _originalWouldClampSteps++;
            if (finalClamped) _finalClampSteps++;

            _peakPdAbs = Mathf.Max(_peakPdAbs, Mathf.Abs(pdFeedbackSigned));
            _peakInputFfAbs = Mathf.Max(_peakInputFfAbs, Mathf.Abs(kneeAccelerationFeedforwardSigned));
            _peakOriginalRawAbs = Mathf.Max(_peakOriginalRawAbs, Mathf.Abs(_currentOriginalRaw));
            _peakAllocatedRawAbs = Mathf.Max(_peakAllocatedRawAbs, Mathf.Abs(allocatedRawSigned));
            _peakAppliedAbs = Mathf.Max(_peakAppliedAbs, Mathf.Abs(applied));
            _lastEvidence =
                "step=" + _steps +
                "/opposing=" + _currentOpposing +
                "/pd=" + F(pdFeedbackSigned) +
                "/ffIn=" + F(kneeAccelerationFeedforwardSigned) +
                "/ffEff=" + F(effectiveFeedforwardSigned) +
                "/rawA=" + F(_currentOriginalRaw) +
                "/allocatedA=" + F(allocatedRawSigned) +
                "/appliedA=" + F(applied) +
                "/maxA=" + F(_currentMaxA) +
                "/clamp=" + finalClamped;
            CopyToReport(report);
            return applied;
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.KneeFlexionAllocationMode = Mode;
            report.KneeFlexionAllocationArm = Arm;
            report.KneeFlexionAllocationEnabled = _enabled;
            report.KneeFlexionAllocationSteps = _steps;
            report.KneeFlexionAllocationActiveSteps = _activeSteps;
            report.KneeFlexionAllocationInvalidSteps = _invalidSteps;
            report.KneeFlexionAllocationOpposingSteps = _opposingSteps;
            report.KneeFlexionAllocationSuppressedOpposingFfSteps = _suppressedOpposingFfSteps;
            report.KneeFlexionAllocationSameDirectionSteps = _sameDirectionSteps;
            report.KneeFlexionAllocationOriginalWouldClampSteps = _originalWouldClampSteps;
            report.KneeFlexionAllocationFinalClampSteps = _finalClampSteps;
            report.KneeFlexionAllocationCurrentPd = _currentPd;
            report.KneeFlexionAllocationCurrentInputFf = _currentInputFf;
            report.KneeFlexionAllocationCurrentEffectiveFf = _currentEffectiveFf;
            report.KneeFlexionAllocationCurrentOriginalRaw = _currentOriginalRaw;
            report.KneeFlexionAllocationCurrentAllocatedRaw = _currentAllocatedRaw;
            report.KneeFlexionAllocationCurrentApplied = _currentApplied;
            report.KneeFlexionAllocationCurrentMaxA = _currentMaxA;
            report.KneeFlexionAllocationCurrentOpposing = _currentOpposing;
            report.KneeFlexionAllocationCurrentOriginalWouldClamp = _currentOriginalWouldClamp;
            report.KneeFlexionAllocationCurrentFinalClamped = _currentFinalClamped;
            report.KneeFlexionAllocationPeakPdAbs = _peakPdAbs;
            report.KneeFlexionAllocationPeakInputFfAbs = _peakInputFfAbs;
            report.KneeFlexionAllocationPeakOriginalRawAbs = _peakOriginalRawAbs;
            report.KneeFlexionAllocationPeakAllocatedRawAbs = _peakAllocatedRawAbs;
            report.KneeFlexionAllocationPeakAppliedAbs = _peakAppliedAbs;
            report.KneeFlexionAllocationSuppressedFfAbsMean = _suppressedOpposingFfSteps > 0
                ? _suppressedFfAbsSum / _suppressedOpposingFfSteps
                : 0f;
            report.KneeFlexionAllocationLastEvidence = _lastEvidence;
            report.KneeFlexionAllocationKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return "ARL Knee flexion feedback headroom allocation A/B | mode=" + Mode +
                   " | arm=" + Arm +
                   " | enabled=" + _enabled +
                   " | steps(active/total/invalid)=" + _activeSteps + "/" + _steps + "/" + _invalidSteps +
                   " | direction(opposing/suppressed/same)=" + _opposingSteps + "/" + _suppressedOpposingFfSteps + "/" + _sameDirectionSteps +
                   " | clamp(original/final)=" + _originalWouldClampSteps + "/" + _finalClampSteps +
                   " | current(PD/ffIn/ffEff/raw/allocated/applied/maxA)=" +
                   F(_currentPd) + "/" + F(_currentInputFf) + "/" + F(_currentEffectiveFf) + "/" +
                   F(_currentOriginalRaw) + "/" + F(_currentAllocatedRaw) + "/" + F(_currentApplied) + "/" + F(_currentMaxA) +
                   " | key=" + GetKey(report);
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            if (!_enabled)
                primary = "A_V077_SUM_THEN_CLAMP";
            else if (_invalidSteps > 0)
                primary = "B_INVALID_NON_FINITE_ALLOCATION_INPUT";
            else if (_activeSteps == 0)
                primary = "B_NO_EXPOSURE";
            else if (_opposingSteps == 0)
                primary = "B_NO_OPPOSING_AFF_EXPOSURE";
            else
                primary = "B_FEEDBACK_PRIORITY_ACTIVE_FROZEN";

            return "primary=" + primary +
                   " | mode=" + Mode +
                   " | arm=" + Arm +
                   " | steps(active/total/invalid)=" + _activeSteps + "/" + _steps + "/" + _invalidSteps +
                   " | direction(opposing/suppressed/same)=" + _opposingSteps + "/" + _suppressedOpposingFfSteps + "/" + _sameDirectionSteps +
                   " | clamp(original/final)=" + _originalWouldClampSteps + "/" + _finalClampSteps +
                   " | peak(PD/ffIn/raw/allocated/applied)=" + F(_peakPdAbs) + "/" + F(_peakInputFfAbs) + "/" + F(_peakOriginalRawAbs) + "/" + F(_peakAllocatedRawAbs) + "/" + F(_peakAppliedAbs) +
                   " | suppressedFfMean=" + F(_suppressedOpposingFfSteps > 0 ? _suppressedFfAbsSum / _suppressedOpposingFfSteps : 0f) +
                   " | knee(clamp/mean/max)=" +
                   (report != null ? report.TraceKneeClampSamples : 0) + "/" +
                   F(report != null ? report.TraceKneeRotationMeanDegrees : 0f) + "/" +
                   F(report != null ? report.TraceKneeRotationMaxDegrees : 0f);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
