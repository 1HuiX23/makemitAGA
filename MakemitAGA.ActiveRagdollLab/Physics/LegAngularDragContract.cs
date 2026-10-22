using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// Upper/Lower retain profile angularDrag. The v0.2.82 zero-drag intervention was rejected;
    /// v0.4.22 removes its mutation/restore path, while preserving the per-step fail-closed readback.
    /// This contract never writes Rigidbody state or silently repairs a mismatch.
    /// </summary>
    internal sealed class LegAngularDragContract
    {
        internal const string Mode = "UPPER_LOWER_PROFILE_ANGULAR_DRAG";
        private const float ReadbackTolerance = 0.0001f;
        private int _steps;
        private int _invalidSteps;
        private float _peakUpperError;
        private float _peakLowerError;
        private string _evidence = "NOT_SAMPLED";

        internal bool ValidateStep(Rigidbody upper, Rigidbody lower, float expected, SingleLegChainReport report)
        {
            _steps++;
            bool finite = upper != null && lower != null && IsFinite(expected) && expected >= 0f &&
                          IsFinite(upper.angularDrag) && IsFinite(lower.angularDrag);
            bool valid = false;
            if (finite)
            {
                float upperError = Mathf.Abs(upper.angularDrag - expected);
                float lowerError = Mathf.Abs(lower.angularDrag - expected);
                _peakUpperError = Mathf.Max(_peakUpperError, upperError);
                _peakLowerError = Mathf.Max(_peakLowerError, lowerError);
                valid = upperError <= ReadbackTolerance && lowerError <= ReadbackTolerance;
            }
            if (!valid) _invalidSteps++;
            _evidence = valid ? "PROFILE_DRAG_VALID" : (finite ? "DRAG_READBACK_MISMATCH" : "BODY_OR_DRAG_INVALID");
            CopyToReport(report, upper, lower, expected);
            return valid;
        }

        internal void ResetRunDiagnostics(SingleLegChainReport report, Rigidbody upper, Rigidbody lower, float expected)
        {
            _steps = 0;
            _invalidSteps = 0;
            _peakUpperError = 0f;
            _peakLowerError = 0f;
            _evidence = "NOT_SAMPLED";
            CopyToReport(report, upper, lower, expected);
        }

        internal void CopyToReport(SingleLegChainReport report, Rigidbody upper, Rigidbody lower, float expected)
        {
            if (report == null) return;
            report.LegAngularDragSteps = _steps;
            report.LegAngularDragInvalidSteps = _invalidSteps;
            report.LegAngularDragExpected = expected;
            report.LegAngularDragUpper = upper != null ? upper.angularDrag : float.NaN;
            report.LegAngularDragLower = lower != null ? lower.angularDrag : float.NaN;
            report.LegAngularDragPeakUpperError = _peakUpperError;
            report.LegAngularDragPeakLowerError = _peakLowerError;
            report.LegAngularDragEvidence = _evidence;
        }

        private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
    }
}
