using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.36 Knee off-axis authority + continuous physical tracking health.
    /// Mirrors the Hip semantics: post-Source recovery receives the bounded 40-MaxA boost, while a health
    /// re-entry discovered during ordinary motion keeps baseline 20 and only marks the chain unhealthy so
    /// A-FF/diagnostics cannot call it NORMAL.
    /// </summary>
    internal sealed class PhaseAwareSoftKneeAuthority
    {
        public const string ModeName = "PHASE_AWARE_SOFT_KNEE_RECOVERY_WITH_CONTINUOUS_HEALTH_REENTRY";

        private bool _previousSourceActive;
        private bool _physicalRecoveryPending;
        private bool _recoveryBoostActive;
        private float _legRecoveryStartTime;
        private float _recoveryStableSeconds;
        private float _healthEnterCandidateSeconds;
        private float _currentMaxA;
        private float _currentRecoveryErrorDegrees;
        private string _recoveryOrigin = "NONE";
        private string _phase = "BASELINE";
        private int _sourceBoostFixedSteps;
        private int _recoveryBoostFixedSteps;
        private int _pendingBaselineFixedSteps;
        private int _trackingUnhealthyBaselineFixedSteps;
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

        public void Reset(SingleLegChainReport report)
        {
            _previousSourceActive = false;
            _physicalRecoveryPending = false;
            _recoveryBoostActive = false;
            _legRecoveryStartTime = 0f;
            _recoveryStableSeconds = 0f;
            _healthEnterCandidateSeconds = 0f;
            _currentMaxA = report != null ? report.KneeSoftOffAxisMaxAngularAcceleration : 0f;
            _currentRecoveryErrorDegrees = 0f;
            _recoveryOrigin = "NONE";
            _phase = "BASELINE";
            _sourceBoostFixedSteps = 0;
            _recoveryBoostFixedSteps = 0;
            _pendingBaselineFixedSteps = 0;
            _trackingUnhealthyBaselineFixedSteps = 0;
            _recoveryStarted = 0;
            _recoveryCompleted = 0;
            _recoveryBoostTimedOut = 0;
            _recoveryCensoredByNewSource = 0;
            _continuousHealthReentered = 0;
            _continuousHealthImmediateReentered = 0;
            _continuousHealthPeakEntryErrorDegrees = 0f;
            CopyToReport(report, false);
        }

        public float Step(ActiveRagdollLabProfile profile, TwoBodyProxyReport torso, float kneeOffAxisErrorDegrees, float now, float dt, SingleLegChainReport report)
        {
            float safeDt = Mathf.Max(0f, dt);
            float baseline = profile.LegKneeSoftOffAxisMaxAngularAcceleration;
            float sourceMax = Mathf.Max(baseline, profile.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration);
            float recoveryMax = Mathf.Max(baseline, profile.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration);
            bool sourceActive = torso != null && torso.SourceTurnEventActive;
            _currentRecoveryErrorDegrees = Mathf.Max(0f, kneeOffAxisErrorDegrees);

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
                _sourceBoostFixedSteps++;
                _phase = "SOURCE_TURN_BOOST";
                _currentMaxA = sourceMax;
                CopyToReport(report, sourceMax > baseline + 0.0001f);
                return _currentMaxA;
            }

            if (_previousSourceActive)
                EnterPostSourceRecovery(now);
            _previousSourceActive = false;

            if (!_physicalRecoveryPending)
                UpdateContinuousHealthReentry(profile, torso, kneeOffAxisErrorDegrees, safeDt);

            if (_physicalRecoveryPending)
            {
                if (kneeOffAxisErrorDegrees <= profile.LegKneeSoftOffAxisRecoveryExitErrorDegrees)
                    _recoveryStableSeconds += safeDt;
                else
                    _recoveryStableSeconds = 0f;

                if (_recoveryStableSeconds >= profile.LegKneeSoftOffAxisRecoveryStableHoldSeconds)
                {
                    _physicalRecoveryPending = false;
                    _recoveryBoostActive = false;
                    _recoveryStableSeconds = 0f;
                    _healthEnterCandidateSeconds = 0f;
                    _recoveryOrigin = "NONE";
                    _recoveryCompleted++;
                }
                else if (_recoveryBoostActive && Mathf.Max(0f, now - _legRecoveryStartTime) >= profile.LegKneeSoftOffAxisRecoveryMaxSeconds)
                {
                    _recoveryBoostActive = false;
                    _recoveryBoostTimedOut++;
                }
            }

            if (_physicalRecoveryPending && _recoveryBoostActive)
            {
                _recoveryBoostFixedSteps++;
                _phase = "LEG_RECOVERY_BOOST";
                _currentMaxA = recoveryMax;
                CopyToReport(report, recoveryMax > baseline + 0.0001f);
                return _currentMaxA;
            }

            if (_physicalRecoveryPending)
            {
                if (_recoveryOrigin == "CONTINUOUS_HEALTH")
                {
                    _trackingUnhealthyBaselineFixedSteps++;
                    _phase = "TRACKING_UNHEALTHY_BASELINE";
                }
                else
                {
                    _pendingBaselineFixedSteps++;
                    _phase = "PHYSICAL_RECOVERY_PENDING_BASELINE";
                }
                _currentMaxA = baseline;
                CopyToReport(report, false);
                return _currentMaxA;
            }

            _phase = "BASELINE";
            _currentMaxA = baseline;
            CopyToReport(report, false);
            return _currentMaxA;
        }

        private void EnterPostSourceRecovery(float now)
        {
            _physicalRecoveryPending = true;
            _recoveryBoostActive = true;
            _legRecoveryStartTime = now;
            _recoveryStableSeconds = 0f;
            _healthEnterCandidateSeconds = 0f;
            _recoveryOrigin = "POST_SOURCE";
            _recoveryStarted++;
        }

        private void UpdateContinuousHealthReentry(ActiveRagdollLabProfile profile, TwoBodyProxyReport torso, float errorDegrees, float dt)
        {
            bool stableMotionWindow = IsStableMotionWindow(torso);
            bool immediate = errorDegrees >= profile.LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees;
            bool candidate = stableMotionWindow && errorDegrees >= profile.LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees;

            if (immediate)
            {
                EnterContinuousHealthRecovery(errorDegrees, true);
                return;
            }

            if (candidate)
            {
                _healthEnterCandidateSeconds += dt;
                if (_healthEnterCandidateSeconds >= profile.LegKneeSoftOffAxisTrackingHealthEnterHoldSeconds)
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

        private void CopyToReport(SingleLegChainReport report, bool boostActive)
        {
            if (report == null) return;
            report.KneeSoftOffAxisAuthorityMode = ModeName;
            report.KneeSoftOffAxisAuthorityPhase = _phase;
            report.KneeSoftOffAxisAuthorityCurrentMaxAngularAcceleration = _currentMaxA;
            report.KneeSoftOffAxisAuthorityBoostActive = boostActive;
            report.KneeSoftOffAxisPhysicalRecoveryPending = _physicalRecoveryPending;
            report.KneeSoftOffAxisRecoveryBoostActive = _recoveryBoostActive;
            report.KneeSoftOffAxisRecoveryErrorDegrees = _currentRecoveryErrorDegrees;
            report.KneeSoftOffAxisPhysicalRecoveryOrigin = _recoveryOrigin;
            report.KneeSoftOffAxisTrackingHealthUnhealthy = TrackingHealthUnhealthy;
            report.KneeSoftOffAxisTrackingHealthCandidateSeconds = _healthEnterCandidateSeconds;
            report.KneeSoftOffAxisTrackingHealthReentryCount = _continuousHealthReentered;
            report.KneeSoftOffAxisTrackingHealthImmediateReentryCount = _continuousHealthImmediateReentered;
            report.KneeSoftOffAxisTrackingHealthPeakEntryErrorDegrees = _continuousHealthPeakEntryErrorDegrees;
            report.KneeSoftOffAxisSourceBoostFixedSteps = _sourceBoostFixedSteps;
            report.KneeSoftOffAxisRecoveryBoostFixedSteps = _recoveryBoostFixedSteps;
            report.KneeSoftOffAxisPhysicalRecoveryPendingBaselineFixedSteps = _pendingBaselineFixedSteps;
            report.KneeSoftOffAxisTrackingUnhealthyBaselineFixedSteps = _trackingUnhealthyBaselineFixedSteps;
            report.KneeSoftOffAxisRecoveryStarted = _recoveryStarted;
            report.KneeSoftOffAxisRecoveryCompleted = _recoveryCompleted;
            report.KneeSoftOffAxisRecoveryTimedOut = _recoveryBoostTimedOut;
            report.KneeSoftOffAxisRecoveryCensoredByNewSource = _recoveryCensoredByNewSource;
        }
    }
}
