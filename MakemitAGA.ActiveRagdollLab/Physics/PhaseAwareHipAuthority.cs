using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.36 Hip authority + continuous physical tracking health.
    ///
    /// PhysicalRecoveryPending now has two entry paths but one authoritative meaning:
    /// the physical UpperLeg is not currently considered healthy against the RAW source Hip target.
    /// 1) POST_SOURCE: Source Turn closes -> recovery pending + bounded 80-MaxA boost.
    /// 2) CONTINUOUS_HEALTH: outside Source Turn, tracking error crosses a hysteretic health gate ->
    ///    recovery pending at baseline MaxA only. This re-arms recovery even after an earlier recovery completed.
    ///
    /// The health path deliberately does NOT increase authority. Its first job is to suppress A-FF and keep
    /// diagnostics out of NORMAL while tracking is unhealthy, so v0.2.36 can test state semantics without
    /// adding torque authority.
    /// </summary>
    internal sealed class PhaseAwareHipAuthority
    {
        public const string ModeName = "PHASE_AWARE_HIP_RECOVERY_WITH_CONTINUOUS_HEALTH_REENTRY";

        private bool _previousSourceActive;
        private bool _physicalRecoveryPending;
        private bool _recoveryBoostActive;
        private float _recoveryStartTime;
        private float _recoveryStableSeconds;
        private float _healthEnterCandidateSeconds;
        private string _recoveryOrigin = "NONE";
        private string _phase = "BASELINE";
        private float _currentMaxA;
        private float _currentRecoveryErrorDegrees;

        private int _baselineSteps;
        private int _sourceSteps;
        private int _recoveryBoostSteps;
        private int _pendingBaselineSteps;
        private int _trackingUnhealthyBaselineSteps;
        private int _baselineClamps;
        private int _sourceClamps;
        private int _recoveryClamps;
        private float _sourceRawSum;
        private float _sourceRawMax;
        private float _sourceAppliedSum;
        private float _sourceAppliedMax;
        private float _recoveryRawSum;
        private float _recoveryRawMax;
        private float _recoveryAppliedSum;
        private float _recoveryAppliedMax;
        private int _recoveryStarted;
        private int _recoveryCompleted;
        private int _recoveryBoostTimedOut;
        private int _recoveryCensoredByNewSource;
        private int _continuousHealthReentered;
        private int _continuousHealthImmediateReentered;
        private float _continuousHealthPeakEntryErrorDegrees;

        public bool RecoveryActive => _physicalRecoveryPending;
        public bool PhysicalRecoveryPending => _physicalRecoveryPending;
        public bool RecoveryBoostActive => _recoveryBoostActive;
        public bool TrackingHealthUnhealthy => _physicalRecoveryPending && _recoveryOrigin == "CONTINUOUS_HEALTH";
        public float HealthEnterCandidateSeconds => _healthEnterCandidateSeconds;
        public string RecoveryOrigin => _recoveryOrigin;
        public float CurrentMaxAngularAcceleration => _currentMaxA;
        public string Phase => _phase;

        public void Reset(ActiveRagdollLabProfile profile, SingleLegChainReport report)
        {
            _previousSourceActive = false;
            _physicalRecoveryPending = false;
            _recoveryBoostActive = false;
            _recoveryStartTime = 0f;
            _recoveryStableSeconds = 0f;
            _healthEnterCandidateSeconds = 0f;
            _recoveryOrigin = "NONE";
            _phase = "BASELINE";
            _currentMaxA = profile != null ? profile.LegHipMaxAngularAcceleration : 0f;
            _currentRecoveryErrorDegrees = 0f;
            _baselineSteps = _sourceSteps = _recoveryBoostSteps = _pendingBaselineSteps = _trackingUnhealthyBaselineSteps = 0;
            _baselineClamps = _sourceClamps = _recoveryClamps = 0;
            _sourceRawSum = _sourceRawMax = _sourceAppliedSum = _sourceAppliedMax = 0f;
            _recoveryRawSum = _recoveryRawMax = _recoveryAppliedSum = _recoveryAppliedMax = 0f;
            _recoveryStarted = _recoveryCompleted = _recoveryBoostTimedOut = _recoveryCensoredByNewSource = 0;
            _continuousHealthReentered = _continuousHealthImmediateReentered = 0;
            _continuousHealthPeakEntryErrorDegrees = 0f;
            CopyToReport(profile, report, false);
        }

        public float Step(ActiveRagdollLabProfile profile, TwoBodyProxyReport torso, float rawPhysicalHipErrorDegrees, float now, float dt, SingleLegChainReport report)
        {
            _currentRecoveryErrorDegrees = Mathf.Max(0f, rawPhysicalHipErrorDegrees);
            float safeDt = Mathf.Max(0f, dt);
            float baseline = profile.LegHipMaxAngularAcceleration;
            float sourceMax = Mathf.Max(baseline, profile.LegHipSourceTurnMaxAngularAcceleration);
            float recoveryMax = Mathf.Max(baseline, profile.LegHipRecoveryMaxAngularAcceleration);
            bool sourceActive = torso != null && torso.SourceTurnEventActive;

            if (sourceActive)
            {
                if (_physicalRecoveryPending)
                    _recoveryCensoredByNewSource++;

                _physicalRecoveryPending = false;
                _recoveryBoostActive = false;
                _recoveryStableSeconds = 0f;
                _healthEnterCandidateSeconds = 0f;
                _recoveryOrigin = "NONE";
                _previousSourceActive = true;
                _phase = "SOURCE_TURN_BOOST";
                _currentMaxA = sourceMax;
                CopyToReport(profile, report, sourceMax > baseline + 0.0001f);
                return _currentMaxA;
            }

            if (_previousSourceActive)
            {
                EnterPostSourceRecovery(now);
            }
            _previousSourceActive = false;

            if (!_physicalRecoveryPending)
            {
                UpdateContinuousHealthReentry(profile, torso, rawPhysicalHipErrorDegrees, safeDt);
            }

            if (_physicalRecoveryPending)
            {
                if (rawPhysicalHipErrorDegrees <= profile.LegHipRecoveryExitErrorDegrees)
                    _recoveryStableSeconds += safeDt;
                else
                    _recoveryStableSeconds = 0f;

                if (_recoveryStableSeconds >= profile.LegHipRecoveryStableHoldSeconds)
                {
                    _physicalRecoveryPending = false;
                    _recoveryBoostActive = false;
                    _recoveryStableSeconds = 0f;
                    _healthEnterCandidateSeconds = 0f;
                    _recoveryOrigin = "NONE";
                    _recoveryCompleted++;
                }
                else if (_recoveryBoostActive && Mathf.Max(0f, now - _recoveryStartTime) >= profile.LegHipRecoveryMaxSeconds)
                {
                    // Boost timeout is not physical recovery completion.
                    _recoveryBoostActive = false;
                    _recoveryBoostTimedOut++;
                }
            }

            if (_physicalRecoveryPending && _recoveryBoostActive)
            {
                _phase = "HIP_RECOVERY_BOOST";
                _currentMaxA = recoveryMax;
                CopyToReport(profile, report, recoveryMax > baseline + 0.0001f);
                return _currentMaxA;
            }

            if (_physicalRecoveryPending)
            {
                _phase = _recoveryOrigin == "CONTINUOUS_HEALTH"
                    ? "TRACKING_UNHEALTHY_BASELINE"
                    : "PHYSICAL_RECOVERY_PENDING_BASELINE";
                _currentMaxA = baseline;
                CopyToReport(profile, report, false);
                return _currentMaxA;
            }

            _phase = "BASELINE";
            _currentMaxA = baseline;
            CopyToReport(profile, report, false);
            return _currentMaxA;
        }

        private void EnterPostSourceRecovery(float now)
        {
            _physicalRecoveryPending = true;
            _recoveryBoostActive = true;
            _recoveryStartTime = now;
            _recoveryStableSeconds = 0f;
            _healthEnterCandidateSeconds = 0f;
            _recoveryOrigin = "POST_SOURCE";
            _recoveryStarted++;
        }

        private void UpdateContinuousHealthReentry(ActiveRagdollLabProfile profile, TwoBodyProxyReport torso, float errorDegrees, float dt)
        {
            bool stableMotionWindow = IsStableMotionWindow(torso);
            bool immediate = errorDegrees >= profile.LegHipTrackingHealthImmediateErrorDegrees;
            bool candidate = stableMotionWindow && errorDegrees >= profile.LegHipTrackingHealthEnterErrorDegrees;

            if (immediate)
            {
                EnterContinuousHealthRecovery(errorDegrees, true);
                return;
            }

            if (candidate)
            {
                _healthEnterCandidateSeconds += dt;
                if (_healthEnterCandidateSeconds >= profile.LegHipTrackingHealthEnterHoldSeconds)
                    EnterContinuousHealthRecovery(errorDegrees, false);
            }
            else
            {
                _healthEnterCandidateSeconds = 0f;
            }
        }

        private void EnterContinuousHealthRecovery(float errorDegrees, bool immediate)
        {
            _physicalRecoveryPending = true;
            _recoveryBoostActive = false;
            _recoveryStableSeconds = 0f;
            _healthEnterCandidateSeconds = 0f;
            _recoveryOrigin = "CONTINUOUS_HEALTH";
            _recoveryStarted++;
            _continuousHealthReentered++;
            if (immediate) _continuousHealthImmediateReentered++;
            _continuousHealthPeakEntryErrorDegrees = Mathf.Max(_continuousHealthPeakEntryErrorDegrees, errorDegrees);
        }

        private static bool IsStableMotionWindow(TwoBodyProxyReport torso)
        {
            if (torso == null || torso.ControlRecoveryActive || torso.MotionTransition)
                return false;
            string phase = torso.MotionPhase ?? string.Empty;
            return phase.Equals("STEADY", System.StringComparison.OrdinalIgnoreCase) ||
                   phase.Equals("IDLE", System.StringComparison.OrdinalIgnoreCase);
        }

        public void RecordResult(ActiveRagdollLabProfile profile, float rawMagnitude, float appliedMagnitude, bool clamped, SingleLegChainReport report)
        {
            if (_phase == "SOURCE_TURN_BOOST")
            {
                _sourceSteps++;
                if (clamped) _sourceClamps++;
                _sourceRawSum += rawMagnitude;
                _sourceRawMax = Mathf.Max(_sourceRawMax, rawMagnitude);
                _sourceAppliedSum += appliedMagnitude;
                _sourceAppliedMax = Mathf.Max(_sourceAppliedMax, appliedMagnitude);
            }
            else if (_phase == "HIP_RECOVERY_BOOST")
            {
                _recoveryBoostSteps++;
                if (clamped) _recoveryClamps++;
                _recoveryRawSum += rawMagnitude;
                _recoveryRawMax = Mathf.Max(_recoveryRawMax, rawMagnitude);
                _recoveryAppliedSum += appliedMagnitude;
                _recoveryAppliedMax = Mathf.Max(_recoveryAppliedMax, appliedMagnitude);
            }
            else
            {
                _baselineSteps++;
                if (_phase == "PHYSICAL_RECOVERY_PENDING_BASELINE") _pendingBaselineSteps++;
                if (_phase == "TRACKING_UNHEALTHY_BASELINE") _trackingUnhealthyBaselineSteps++;
                if (clamped) _baselineClamps++;
            }
            CopyToReport(profile, report, _phase == "SOURCE_TURN_BOOST" || _recoveryBoostActive);
        }

        public string GetKey(ActiveRagdollLabProfile profile, SingleLegChainReport report)
        {
            float baseline = profile != null ? profile.LegHipMaxAngularAcceleration : 0f;
            float source = profile != null ? profile.LegHipSourceTurnMaxAngularAcceleration : 0f;
            float recovery = profile != null ? profile.LegHipRecoveryMaxAngularAcceleration : 0f;
            return
                "mode=" + ModeName +
                " | maxA(base/source/recoveryBoost)=" + F(baseline) + "/" + F(source) + "/" + F(recovery) +
                " | recoveryExit=" + F(profile != null ? profile.LegHipRecoveryExitErrorDegrees : 0f) + "deg/" + F(profile != null ? profile.LegHipRecoveryStableHoldSeconds : 0f) + "s" +
                " | healthReentry(enter/hold/immediate)=" + F(profile != null ? profile.LegHipTrackingHealthEnterErrorDegrees : 0f) + "deg/" + F(profile != null ? profile.LegHipTrackingHealthEnterHoldSeconds : 0f) + "s/" + F(profile != null ? profile.LegHipTrackingHealthImmediateErrorDegrees : 0f) + "deg" +
                " | boostMax=" + F(profile != null ? profile.LegHipRecoveryMaxSeconds : 0f) + "s" +
                " | current=" + _phase + "/" + F(_currentMaxA) + "/rawHipErr=" + F(_currentRecoveryErrorDegrees) +
                "/physicalPending=" + _physicalRecoveryPending + "/origin=" + _recoveryOrigin + "/recoveryBoost=" + _recoveryBoostActive + "/healthCandidate=" + F(_healthEnterCandidateSeconds) + "s" +
                " | steps(base/source/recoveryBoost/pendingBaseline/healthBaseline)=" + _baselineSteps + "/" + _sourceSteps + "/" + _recoveryBoostSteps + "/" + _pendingBaselineSteps + "/" + _trackingUnhealthyBaselineSteps +
                " | clamps(base/source/recoveryBoost)=" + _baselineClamps + "/" + _sourceClamps + "/" + _recoveryClamps +
                " | clampRate(source/recoveryBoost)=" + F(Rate(_sourceClamps, _sourceSteps)) + "/" + F(Rate(_recoveryClamps, _recoveryBoostSteps)) +
                " | sourceRaw(mean/max)=" + F(Mean(_sourceRawSum, _sourceSteps)) + "/" + F(_sourceRawMax) +
                " | sourceApplied(mean/max)=" + F(Mean(_sourceAppliedSum, _sourceSteps)) + "/" + F(_sourceAppliedMax) +
                " | recoveryRaw(mean/max)=" + F(Mean(_recoveryRawSum, _recoveryBoostSteps)) + "/" + F(_recoveryRawMax) +
                " | recoveryApplied(mean/max)=" + F(Mean(_recoveryAppliedSum, _recoveryBoostSteps)) + "/" + F(_recoveryAppliedMax) +
                " | physicalRecovery(start/completed/boostTimedOut/censoredByNewSource)=" + _recoveryStarted + "/" + _recoveryCompleted + "/" + _recoveryBoostTimedOut + "/" + _recoveryCensoredByNewSource +
                " | continuousHealth(reentered/immediate/peakEntryDeg)=" + _continuousHealthReentered + "/" + _continuousHealthImmediateReentered + "/" + F(_continuousHealthPeakEntryErrorDegrees) +
                " | traceHip(mean/max/clampSamples)=" + F(report != null ? report.TraceHipRotationMeanDegrees : 0f) + "/" + F(report != null ? report.TraceHipRotationMaxDegrees : 0f) + "/" + (report != null ? report.TraceHipClampSamples : 0);
        }

        private void CopyToReport(ActiveRagdollLabProfile profile, SingleLegChainReport report, bool boostActive)
        {
            if (report == null) return;
            report.HipAuthorityMode = ModeName;
            report.HipBaselineMaxAngularAcceleration = profile != null ? profile.LegHipMaxAngularAcceleration : 0f;
            report.HipSourceTurnMaxAngularAcceleration = profile != null ? profile.LegHipSourceTurnMaxAngularAcceleration : 0f;
            report.HipRecoveryMaxAngularAcceleration = profile != null ? profile.LegHipRecoveryMaxAngularAcceleration : 0f;
            report.HipAuthorityPhase = _phase;
            report.HipAuthorityCurrentMaxAngularAcceleration = _currentMaxA;
            report.HipAuthorityRecoveryErrorDegrees = _currentRecoveryErrorDegrees;
            report.HipAuthorityBoostActive = boostActive;
            report.HipPhysicalRecoveryPending = _physicalRecoveryPending;
            report.HipRecoveryBoostActive = _recoveryBoostActive;
            report.HipPhysicalRecoveryOrigin = _recoveryOrigin;
            report.HipTrackingHealthUnhealthy = TrackingHealthUnhealthy;
            report.HipTrackingHealthCandidateSeconds = _healthEnterCandidateSeconds;
            report.HipTrackingHealthReentryCount = _continuousHealthReentered;
            report.HipTrackingHealthImmediateReentryCount = _continuousHealthImmediateReentered;
            report.HipTrackingHealthPeakEntryErrorDegrees = _continuousHealthPeakEntryErrorDegrees;
            report.HipAuthorityBaselineFixedSteps = _baselineSteps;
            report.HipAuthoritySourceBoostFixedSteps = _sourceSteps;
            report.HipAuthorityRecoveryBoostFixedSteps = _recoveryBoostSteps;
            report.HipAuthorityPhysicalRecoveryPendingBaselineFixedSteps = _pendingBaselineSteps;
            report.HipAuthorityTrackingUnhealthyBaselineFixedSteps = _trackingUnhealthyBaselineSteps;
            report.HipAuthorityBaselineClampSamples = _baselineClamps;
            report.HipAuthoritySourceClampSamples = _sourceClamps;
            report.HipAuthorityRecoveryClampSamples = _recoveryClamps;
            report.HipAuthorityRecoveryStarted = _recoveryStarted;
            report.HipAuthorityRecoveryCompleted = _recoveryCompleted;
            report.HipAuthorityRecoveryTimedOut = _recoveryBoostTimedOut;
            report.HipAuthorityRecoveryCensoredByNewSource = _recoveryCensoredByNewSource;
        }

        private static float Mean(float sum, int n) => n > 0 ? sum / n : 0f;
        private static float Rate(int n, int d) => d > 0 ? (float)n / d : 0f;
        private static string F(float value) => value.ToString("G6", System.Globalization.CultureInfo.InvariantCulture);
    }
}
