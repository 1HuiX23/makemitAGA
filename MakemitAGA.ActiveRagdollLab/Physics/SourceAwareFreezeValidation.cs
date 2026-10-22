using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.1.16 validation/lifecycle layer. Dynamics remain frozen from v0.1.11/v0.1.12.
    /// v0.1.14 Source-Close recovery timing and ALL acceptance thresholds remain unchanged.
    /// v0.1.16 adds drive-loss fail-fast and numerically robust 0.15 s hold evaluation; Dynamics remain unchanged.
    ///
    /// The validator deliberately separates three questions which v0.1.12 proved must not be mixed:
    /// 1) NORMAL_TRACKING: quiet/steady physics tracking outside source-turn attribution/recovery windows.
    /// 2) SOURCE_DISCONTINUITY_SURVIVAL: source Hips may jump tens of degrees in one Render sample;
    ///    the ARL is allowed to lag, but must stay finite, keep its CharacterJoint connected and avoid
    ///    persistent anchor/Parent-FF failure.
    /// 3) RECOVERY: only after the source event quiet window closes does the recovery clock start.
    ///    The dynamic Chest must return to low rotation error and a small anchor error within a bounded
    ///    post-close time. A new source event or trace stop censors the unfinished recovery instead of
    ///    falsely counting the upstream interruption as a controller failure.
    ///
    /// Aggregate validation does not write physics. The legacy FreezeRecoveryActive report is also consumed
    /// by leg phase/FF qualification in legacy scopes and the dynamic control arm A. V04760 arm B instead
    /// consumes DynamicTorsoRecoveryGate; preserve this validator for historical acceptance comparisons.
    /// </summary>
    internal sealed class SourceAwareFreezeValidation
    {
        internal const string ModeName = "SOURCE_AWARE_FREEZE_VALIDATION";
        internal const string PolicyName = "TWO_BODY_FREEZE_POLICY_V0_1_16_DRIVE_FAIL_FAST_AND_HOLD_ROBUSTNESS";

        // Deliberately fixed policy for the final two-body validation pass.  These are validation gates,
        // not controller gains.  Keeping them here makes the freeze decision reproducible and prevents
        // old profile files from silently changing the acceptance criteria.
        internal const int MinNormalSteadySamples = 250;
        internal const int MinNormalTurnSamples = 10;
        internal const float NormalSteadyRotationMeanMaxDegrees = 2.0f;
        internal const float NormalSteadyRotationPeakMaxDegrees = 10.0f;
        internal const float NormalSteadyChestPositionMeanMaxMeters = 0.060f;
        internal const float NormalSteadyAnchorRelativeSpeedMeanMax = 0.0010f;
        internal const float NormalSteadyAnchorRelativeSpeedPeakMax = 0.0050f;
        internal const float NormalTurnRotationMeanMaxDegrees = 15.0f;
        internal const float NormalTurnRotationPeakMaxDegrees = 40.0f;
        internal const int NormalAngularClampMaxSteps = 0;
        internal const int NormalParentFeedforwardClampMaxSteps = 0;

        internal const int MinSourceTurnEvents = 2;
        internal const int MinSourceDiscontinuityEvents = 1;
        internal const float SourceTurnMaxAnchorErrorMeters = 0.010f;
        internal const int SourceTurnParentFeedforwardClampMaxSteps = 0;

        internal const float RecoveryRotationLooseDegrees = 5.0f;
        internal const float RecoveryRotationStrictDegrees = 2.0f;
        internal const float RecoveryLooseMaxSecondsFromSourceClose = 1.50f;
        internal const float RecoveryStrictMaxSecondsFromSourceClose = 2.00f;
        internal const int MinCompletedRecoverySessions = 5;
        internal const float RecoveryAnchorErrorMaxMeters = 0.0020f;
        internal const float RecoveryStableHoldSeconds = 0.15f;
        // v0.1.16 numerical tolerance only: 0.1 ms. This is far below one physics tick and prevents
        // values such as 0.149994 s from failing a nominal 0.150000 s hold due to float accumulation.
        internal const float RecoveryStableHoldNumericalToleranceSeconds = 0.0001f;

        private bool _sessionActive;
        private bool _eventWasActive;
        private int _activeEventId;
        private float _activeEventStartTime;

        private bool _recoveryActive;
        private int _recoveryEventId;
        private float _recoveryEventStartTime;
        private float _recoveryCloseTime;
        private float _recoverySourceEventDurationSeconds;
        private float _looseStableStart = -1f;
        private float _strictStableStart = -1f;
        private bool _looseRecovered;
        private bool _strictRecovered;
        private float _looseRecoverySeconds;
        private float _strictRecoverySeconds;

        private int _normalSteadySamples;
        private float _normalSteadyRotationSum;
        private float _normalSteadyRotationMax;
        private float _normalSteadyChestPositionSum;
        private float _normalSteadyChestPositionMax;
        private float _normalSteadyAnchorRelativeSpeedSum;
        private float _normalSteadyAnchorRelativeSpeedMax;
        private int _normalTurnSamples;
        private float _normalTurnRotationSum;
        private float _normalTurnRotationMax;
        private float _normalTurnAnchorRelativeSpeedSum;
        private float _normalTurnAnchorRelativeSpeedMax;
        private int _normalAngularClampSteps;
        private int _normalParentFeedforwardClampSteps;

        private int _sourceEventFixedSteps;
        private int _sourceInvalidStateSteps;
        private int _sourceParentFeedforwardClampSteps;
        private float _sourceMaxAnchorErrorMeters;

        private int _recoveryStartedCount;
        private int _recoveryCompletedCount;
        private int _recoveryPassedCount;
        private int _recoveryFailedCount;
        private int _recoveryCensoredCount;
        private int _recoveryCensoredByNewSourceEventCount;
        private int _recoveryCensoredByTraceStopCount;
        private int _sourceEventOpenAtTraceStopCount;
        // Aggregates below include only completed/evaluable recovery sessions.
        private int _recoveryInvalidStateSteps;
        private int _recoveryParentFeedforwardClampSteps;
        private float _recoveryMaxLooseSeconds;
        private float _recoveryMaxStrictSeconds;
        private float _recoveryMaxAnchorErrorMeters;

        // Current recovery session counters are committed only when the session completes.
        // If a new source event or trace stop censors it, these values move to censored diagnostics
        // and cannot make RECOVERY_PASS fail.
        private int _currentRecoveryInvalidStateSteps;
        private int _currentRecoveryParentFeedforwardClampSteps;
        private float _currentRecoveryMaxAnchorErrorMeters;
        private int _censoredRecoveryInvalidStateSteps;
        private int _censoredRecoveryParentFeedforwardClampSteps;
        private float _censoredRecoveryMaxAnchorErrorMeters;
        private string _lastRecoveryResult = "<none>";

        // v0.1.15 Recovery Failure Attribution retained in v0.1.16. These values are observability-only and never feed control.
        private int _recoveryFailureAttributionCount;
        private string _lastRecoveryAttribution = "<none>";
        private int _currentRecoverySampleCount;
        private float _currentRecoveryRotationSum;
        private float _currentRecoveryRotationMinDegrees;
        private float _currentRecoveryRotationMaxDegrees;
        private float _currentRecoveryFirstLooseEntrySeconds;
        private float _currentRecoveryFirstStrictEntrySeconds;
        private float _currentRecoveryLooseIntervalStart;
        private float _currentRecoveryStrictIntervalStart;
        private float _currentRecoveryLongestLooseContinuousSeconds;
        private float _currentRecoveryLongestStrictContinuousSeconds;
        private int _currentRecoveryLooseEntryCount;
        private int _currentRecoveryStrictEntryCount;
        private int _currentRecoveryLooseRotationBreakCount;
        private int _currentRecoveryStrictRotationBreakCount;
        private int _currentRecoveryLooseAnchorBreakCount;
        private int _currentRecoveryStrictAnchorBreakCount;
        private float _currentRecoveryTargetSpeedSum;
        private float _currentRecoveryTargetSpeedMax;
        private float _currentRecoveryTargetRelativeAngularSpeedSum;
        private float _currentRecoveryTargetRelativeAngularSpeedMax;
        private string _currentRecoveryLastMotionPhase = "<none>";
        private bool _currentRecoveryLastMotionTransition;
        private float _currentRecoveryEndRotationDegrees;
        private float _currentRecoveryEndTargetSpeed;
        private float _currentRecoveryEndTargetRelativeAngularSpeed;

        public bool ValidationComplete { get; private set; }
        public bool NormalTrackingPass { get; private set; }
        public bool SourceDiscontinuitySurvivalPass { get; private set; }
        public bool RecoveryPass { get; private set; }
        public bool TwoBodyFreezeReady { get; private set; }
        public string DecisionReason { get; private set; } = "TRACE_NOT_COMPLETE";
        public bool TraceAborted { get; private set; }
        public string TraceAbortReason { get; private set; } = "<none>";
        public string TraceAbortStage { get; private set; } = "<none>";
        public string TraceAbortDetail { get; private set; } = "<none>";

        public void BeginTraceSession()
        {
            _sessionActive = true;
            _eventWasActive = false;
            _activeEventId = 0;
            _activeEventStartTime = 0f;
            ResetRecoveryState();

            _normalSteadySamples = 0;
            _normalSteadyRotationSum = 0f;
            _normalSteadyRotationMax = 0f;
            _normalSteadyChestPositionSum = 0f;
            _normalSteadyChestPositionMax = 0f;
            _normalSteadyAnchorRelativeSpeedSum = 0f;
            _normalSteadyAnchorRelativeSpeedMax = 0f;
            _normalTurnSamples = 0;
            _normalTurnRotationSum = 0f;
            _normalTurnRotationMax = 0f;
            _normalTurnAnchorRelativeSpeedSum = 0f;
            _normalTurnAnchorRelativeSpeedMax = 0f;
            _normalAngularClampSteps = 0;
            _normalParentFeedforwardClampSteps = 0;

            _sourceEventFixedSteps = 0;
            _sourceInvalidStateSteps = 0;
            _sourceParentFeedforwardClampSteps = 0;
            _sourceMaxAnchorErrorMeters = 0f;

            _recoveryStartedCount = 0;
            _recoveryCompletedCount = 0;
            _recoveryPassedCount = 0;
            _recoveryFailedCount = 0;
            _recoveryCensoredCount = 0;
            _recoveryCensoredByNewSourceEventCount = 0;
            _recoveryCensoredByTraceStopCount = 0;
            _sourceEventOpenAtTraceStopCount = 0;
            _recoveryInvalidStateSteps = 0;
            _recoveryParentFeedforwardClampSteps = 0;
            _recoveryMaxLooseSeconds = 0f;
            _recoveryMaxStrictSeconds = 0f;
            _recoveryMaxAnchorErrorMeters = 0f;
            _currentRecoveryInvalidStateSteps = 0;
            _currentRecoveryParentFeedforwardClampSteps = 0;
            _currentRecoveryMaxAnchorErrorMeters = 0f;
            _censoredRecoveryInvalidStateSteps = 0;
            _censoredRecoveryParentFeedforwardClampSteps = 0;
            _censoredRecoveryMaxAnchorErrorMeters = 0f;
            _lastRecoveryResult = "<none>";
            _recoveryFailureAttributionCount = 0;
            _lastRecoveryAttribution = "<none>";
            ResetCurrentRecoveryAttribution();

            ValidationComplete = false;
            NormalTrackingPass = false;
            SourceDiscontinuitySurvivalPass = false;
            RecoveryPass = false;
            TwoBodyFreezeReady = false;
            DecisionReason = "TRACE_RUNNING";
            TraceAborted = false;
            TraceAbortReason = "<none>";
            TraceAbortStage = "<none>";
            TraceAbortDetail = "<none>";
        }

        public void ObserveFixedStep(TwoBodyProxyReport report, SourceTurnDiscontinuityDiagnostics sourceTurn, float now)
        {
            if (!_sessionActive || report == null || sourceTurn == null)
                return;

            bool sourceEventActive = sourceTurn.EventActive;

            if (sourceEventActive && !_eventWasActive)
            {
                if (_recoveryActive)
                    CensorRecovery("CENSORED_BY_NEW_SOURCE_EVENT", now, true);

                _eventWasActive = true;
                _activeEventId = sourceTurn.CurrentEventId;
                _activeEventStartTime = now;
            }
            else if (!sourceEventActive && _eventWasActive)
            {
                _eventWasActive = false;
                BeginRecovery(now);
            }

            if (sourceEventActive)
            {
                ObserveSourceEventStep(report);
                return;
            }

            if (_recoveryActive)
            {
                ObserveRecoveryStep(report, now);
                return;
            }

            ObserveNormalStep(report);
        }

        public void AbortTraceSession(
            TwoBodyProxyReport report,
            SourceTurnDiscontinuityDiagnostics sourceTurn,
            float now,
            string reason,
            string stage,
            string detail)
        {
            if (!_sessionActive)
                return;

            if (_recoveryActive)
                CensorRecoveryForDriveAbort(now);

            _eventWasActive = false;
            _sessionActive = false;
            ValidationComplete = true;
            NormalTrackingPass = false;
            SourceDiscontinuitySurvivalPass = false;
            RecoveryPass = false;
            TwoBodyFreezeReady = false;
            TraceAborted = true;
            TraceAbortReason = string.IsNullOrEmpty(reason) ? "UNKNOWN_DRIVE_DROP" : reason;
            TraceAbortStage = string.IsNullOrEmpty(stage) ? "<none>" : stage;
            TraceAbortDetail = string.IsNullOrEmpty(detail) ? "<none>" : detail;
            DecisionReason = "DRIVE_DROPPED_DURING_TRACE";
        }

        public void EndTraceSession(TwoBodyProxyReport report, SourceTurnDiscontinuityDiagnostics sourceTurn, float now)
        {
            if (!_sessionActive)
                return;

            // v0.1.14+: trace stop is not evidence that the controller failed to recover.
            // If the source event is still open, there is no recovery window to evaluate.
            // If a recovery is in progress, mark it censored/not-evaluable instead of FAIL.
            if (_eventWasActive)
            {
                _eventWasActive = false;
                _sourceEventOpenAtTraceStopCount++;
                _lastRecoveryResult = "EVENT_" + _activeEventId + ":NOT_EVALUABLE_SOURCE_EVENT_OPEN_AT_TRACE_STOP";
            }
            else if (_recoveryActive)
            {
                CensorRecovery("CENSORED_BY_TRACE_STOP", now, false);
            }

            _sessionActive = false;
            Evaluate(report, sourceTurn);
        }

        public void CopyToReport(TwoBodyProxyReport report)
        {
            if (report == null)
                return;

            report.FreezeValidationMode = ModeName;
            report.FreezeValidationPolicy = PolicyName;
            report.FreezeValidationComplete = ValidationComplete;
            report.NormalTrackingPass = NormalTrackingPass;
            report.SourceDiscontinuitySurvivalPass = SourceDiscontinuitySurvivalPass;
            report.RecoveryPass = RecoveryPass;
            report.TwoBodyFreezeReady = TwoBodyFreezeReady;
            report.FreezeValidationReason = DecisionReason;
            report.FreezeValidationTraceAborted = TraceAborted;
            report.FreezeValidationAbortReason = TraceAbortReason;
            report.FreezeValidationAbortStage = TraceAbortStage;
            report.FreezeValidationAbortDetail = TraceAbortDetail;

            report.FreezeNormalSteadySampleCount = _normalSteadySamples;
            report.FreezeNormalSteadyRotationMeanDegrees = Mean(_normalSteadyRotationSum, _normalSteadySamples);
            report.FreezeNormalSteadyRotationMaxDegrees = _normalSteadyRotationMax;
            report.FreezeNormalSteadyChestPositionMeanMeters = Mean(_normalSteadyChestPositionSum, _normalSteadySamples);
            report.FreezeNormalSteadyChestPositionMaxMeters = _normalSteadyChestPositionMax;
            report.FreezeNormalSteadyAnchorRelativeSpeedMean = Mean(_normalSteadyAnchorRelativeSpeedSum, _normalSteadySamples);
            report.FreezeNormalSteadyAnchorRelativeSpeedMax = _normalSteadyAnchorRelativeSpeedMax;

            report.FreezeNormalTurnSampleCount = _normalTurnSamples;
            report.FreezeNormalTurnRotationMeanDegrees = Mean(_normalTurnRotationSum, _normalTurnSamples);
            report.FreezeNormalTurnRotationMaxDegrees = _normalTurnRotationMax;
            report.FreezeNormalTurnAnchorRelativeSpeedMean = Mean(_normalTurnAnchorRelativeSpeedSum, _normalTurnSamples);
            report.FreezeNormalTurnAnchorRelativeSpeedMax = _normalTurnAnchorRelativeSpeedMax;
            report.FreezeNormalAngularClampStepCount = _normalAngularClampSteps;
            report.FreezeNormalParentFeedforwardClampStepCount = _normalParentFeedforwardClampSteps;

            report.FreezeSourceEventFixedStepCount = _sourceEventFixedSteps;
            report.FreezeSourceInvalidStateStepCount = _sourceInvalidStateSteps;
            report.FreezeSourceParentFeedforwardClampStepCount = _sourceParentFeedforwardClampSteps;
            report.FreezeSourceMaxAnchorErrorMeters = _sourceMaxAnchorErrorMeters;

            report.FreezeRecoveryActive = _recoveryActive;
            report.FreezeRecoveryStartedCount = _recoveryStartedCount;
            report.FreezeRecoveryCompletedCount = _recoveryCompletedCount;
            report.FreezeRecoveryPassedCount = _recoveryPassedCount;
            report.FreezeRecoveryFailedCount = _recoveryFailedCount;
            report.FreezeRecoveryCensoredCount = _recoveryCensoredCount;
            report.FreezeRecoveryCensoredByNewSourceEventCount = _recoveryCensoredByNewSourceEventCount;
            report.FreezeRecoveryCensoredByTraceStopCount = _recoveryCensoredByTraceStopCount;
            report.FreezeSourceEventOpenAtTraceStopCount = _sourceEventOpenAtTraceStopCount;
            report.FreezeRecoveryMinCompletedRequired = MinCompletedRecoverySessions;
            report.FreezeRecoveryInvalidStateStepCount = _recoveryInvalidStateSteps;
            report.FreezeRecoveryParentFeedforwardClampStepCount = _recoveryParentFeedforwardClampSteps;
            report.FreezeCensoredRecoveryInvalidStateStepCount = _censoredRecoveryInvalidStateSteps;
            report.FreezeCensoredRecoveryParentFeedforwardClampStepCount = _censoredRecoveryParentFeedforwardClampSteps;
            report.FreezeCensoredRecoveryMaxAnchorErrorMeters = _censoredRecoveryMaxAnchorErrorMeters;
            report.FreezeRecoveryMaxUnder5DegreesSecondsFromSourceClose = _recoveryMaxLooseSeconds;
            report.FreezeRecoveryMaxUnder2DegreesSecondsFromSourceClose = _recoveryMaxStrictSeconds;
            report.FreezeRecoveryMaxAnchorErrorMeters = _recoveryMaxAnchorErrorMeters;
            report.FreezeLastRecoveryResult = _lastRecoveryResult;
            report.FreezeRecoveryFailureAttributionCount = _recoveryFailureAttributionCount;
            report.FreezeLastRecoveryAttribution = _lastRecoveryAttribution;
            report.FreezeValidationThresholds = GetThresholdSummary();
        }

        public string GetSummary(TwoBodyProxyReport report, SourceTurnDiscontinuityDiagnostics sourceTurn)
        {
            int sourceEvents = sourceTurn != null ? sourceTurn.EventCount : 0;
            int sourceDiscontinuities = sourceTurn != null ? sourceTurn.DiscontinuityEventCount : 0;
            return
                "policy=" + PolicyName +
                " | complete=" + ValidationComplete +
                " | NORMAL_TRACKING_PASS=" + NormalTrackingPass +
                " | SOURCE_DISCONTINUITY_SURVIVAL_PASS=" + SourceDiscontinuitySurvivalPass +
                " | RECOVERY_PASS=" + RecoveryPass +
                " | TWO_BODY_FREEZE_READY=" + TwoBodyFreezeReady +
                " | reason=" + DecisionReason +
                " | traceAbort=" + TraceAborted + "/" + TraceAbortReason + "/stage=" + TraceAbortStage +
                " | normalSteady(samples/rotMean/rotMax/posMean/anchorMean/anchorMax)=" +
                    _normalSteadySamples + "/" + F(Mean(_normalSteadyRotationSum, _normalSteadySamples)) + "/" + F(_normalSteadyRotationMax) + "/" +
                    F(Mean(_normalSteadyChestPositionSum, _normalSteadySamples)) + "/" + F(Mean(_normalSteadyAnchorRelativeSpeedSum, _normalSteadySamples)) + "/" + F(_normalSteadyAnchorRelativeSpeedMax) +
                " | normalTurn(samples/rotMean/rotMax/anchorMean/anchorMax)=" +
                    _normalTurnSamples + "/" + F(Mean(_normalTurnRotationSum, _normalTurnSamples)) + "/" + F(_normalTurnRotationMax) + "/" +
                    F(Mean(_normalTurnAnchorRelativeSpeedSum, _normalTurnSamples)) + "/" + F(_normalTurnAnchorRelativeSpeedMax) +
                " | normalClamp(angular/parentFF)=" + _normalAngularClampSteps + "/" + _normalParentFeedforwardClampSteps +
                " | sourceEvents(total/discontinuity)=" + sourceEvents + "/" + sourceDiscontinuities +
                " | sourceSurvival(fixed/invalid/parentFFClamp/maxAnchor)=" + _sourceEventFixedSteps + "/" + _sourceInvalidStateSteps + "/" + _sourceParentFeedforwardClampSteps + "/" + F(_sourceMaxAnchorErrorMeters) +
                " | recovery(start/completed/pass/fail/censored/newEvent/traceStop/max5FromClose/max2FromClose/maxAnchor)=" +
                    _recoveryStartedCount + "/" + _recoveryCompletedCount + "/" + _recoveryPassedCount + "/" + _recoveryFailedCount + "/" +
                    _recoveryCensoredCount + "/" + _recoveryCensoredByNewSourceEventCount + "/" + _recoveryCensoredByTraceStopCount + "/" +
                    F(_recoveryMaxLooseSeconds) + "/" + F(_recoveryMaxStrictSeconds) + "/" + F(_recoveryMaxAnchorErrorMeters) +
                " | censoredHealth(invalid/parentFFClamp/maxAnchor)=" + _censoredRecoveryInvalidStateSteps + "/" + _censoredRecoveryParentFeedforwardClampSteps + "/" + F(_censoredRecoveryMaxAnchorErrorMeters) +
                " | sourceOpenAtTraceStop=" + _sourceEventOpenAtTraceStopCount +
                " | recoveryFailureAttributionCount=" + _recoveryFailureAttributionCount +
                " | lastRecovery=" + _lastRecoveryResult +
                " | thresholds=" + GetThresholdSummary();
        }

        private void ObserveNormalStep(TwoBodyProxyReport report)
        {
            float anchorRelativeSpeed = report.MeasuredAnchorRelativeVelocity.magnitude;

            if (!report.MotionTransition && report.MotionPhase == "STEADY")
            {
                _normalSteadySamples++;
                _normalSteadyRotationSum += Mathf.Max(0f, report.RotationErrorDegrees);
                _normalSteadyRotationMax = Mathf.Max(_normalSteadyRotationMax, Mathf.Max(0f, report.RotationErrorDegrees));
                _normalSteadyChestPositionSum += Mathf.Max(0f, report.ChestPositionErrorMeters);
                _normalSteadyChestPositionMax = Mathf.Max(_normalSteadyChestPositionMax, Mathf.Max(0f, report.ChestPositionErrorMeters));
                _normalSteadyAnchorRelativeSpeedSum += anchorRelativeSpeed;
                _normalSteadyAnchorRelativeSpeedMax = Mathf.Max(_normalSteadyAnchorRelativeSpeedMax, anchorRelativeSpeed);

                if (report.AngularAccelerationClamped)
                    _normalAngularClampSteps++;
                if (report.ParentFeedforwardAccelerationClamped)
                    _normalParentFeedforwardClampSteps++;
            }
            else if (report.MotionTransition && report.MotionPhase == "TURN")
            {
                _normalTurnSamples++;
                _normalTurnRotationSum += Mathf.Max(0f, report.RotationErrorDegrees);
                _normalTurnRotationMax = Mathf.Max(_normalTurnRotationMax, Mathf.Max(0f, report.RotationErrorDegrees));
                _normalTurnAnchorRelativeSpeedSum += anchorRelativeSpeed;
                _normalTurnAnchorRelativeSpeedMax = Mathf.Max(_normalTurnAnchorRelativeSpeedMax, anchorRelativeSpeed);

                if (report.AngularAccelerationClamped)
                    _normalAngularClampSteps++;
                if (report.ParentFeedforwardAccelerationClamped)
                    _normalParentFeedforwardClampSteps++;
            }
        }

        private void ObserveSourceEventStep(TwoBodyProxyReport report)
        {
            _sourceEventFixedSteps++;
            _sourceMaxAnchorErrorMeters = Mathf.Max(_sourceMaxAnchorErrorMeters, Mathf.Max(0f, report.AnchorErrorMeters));
            if (!IsHealthy(report))
                _sourceInvalidStateSteps++;
            if (report.ParentFeedforwardAccelerationClamped)
                _sourceParentFeedforwardClampSteps++;
        }

        private void BeginRecovery(float now)
        {
            _recoveryActive = true;
            _recoveryEventId = _activeEventId;
            _recoveryEventStartTime = _activeEventStartTime;
            _recoveryCloseTime = now;
            _recoverySourceEventDurationSeconds = Mathf.Max(0f, _recoveryCloseTime - _recoveryEventStartTime);
            _looseStableStart = -1f;
            _strictStableStart = -1f;
            _looseRecovered = false;
            _strictRecovered = false;
            _looseRecoverySeconds = 0f;
            _strictRecoverySeconds = 0f;
            _currentRecoveryInvalidStateSteps = 0;
            _currentRecoveryParentFeedforwardClampSteps = 0;
            _currentRecoveryMaxAnchorErrorMeters = 0f;
            ResetCurrentRecoveryAttribution();
            _recoveryStartedCount++;
        }

        private void ObserveRecoveryStep(TwoBodyProxyReport report, float now)
        {
            if (!IsHealthy(report))
                _currentRecoveryInvalidStateSteps++;
            if (report.ParentFeedforwardAccelerationClamped)
                _currentRecoveryParentFeedforwardClampSteps++;

            _currentRecoveryMaxAnchorErrorMeters = Mathf.Max(_currentRecoveryMaxAnchorErrorMeters, Mathf.Max(0f, report.AnchorErrorMeters));

            bool anchorRecovered = report.AnchorErrorMeters <= RecoveryAnchorErrorMaxMeters;
            ObserveRecoveryAttributionStep(report, now, anchorRecovered);
            UpdateRecoveryThreshold(
                report.RotationErrorDegrees <= RecoveryRotationLooseDegrees && anchorRecovered,
                now,
                ref _looseStableStart,
                ref _looseRecovered,
                ref _looseRecoverySeconds);
            UpdateRecoveryThreshold(
                report.RotationErrorDegrees <= RecoveryRotationStrictDegrees && anchorRecovered,
                now,
                ref _strictStableStart,
                ref _strictRecovered,
                ref _strictRecoverySeconds);

            if (_looseRecovered && _strictRecovered)
            {
                FinalizeCurrentRecoveryIntervals(now);
                bool pass =
                    _looseRecoverySeconds <= RecoveryLooseMaxSecondsFromSourceClose &&
                    _strictRecoverySeconds <= RecoveryStrictMaxSecondsFromSourceClose;

                _recoveryCompletedCount++;
                if (pass)
                    _recoveryPassedCount++;
                else
                    _recoveryFailedCount++;

                CommitCurrentRecoveryHealthToCompleted();
                _recoveryMaxLooseSeconds = Mathf.Max(_recoveryMaxLooseSeconds, _looseRecoverySeconds);
                _recoveryMaxStrictSeconds = Mathf.Max(_recoveryMaxStrictSeconds, _strictRecoverySeconds);
                _lastRecoveryResult =
                    "EVENT_" + _recoveryEventId + ":" + (pass ? "PASS" : "TIME_LIMIT_FAIL") +
                    "/under5FromClose=" + F(_looseRecoverySeconds) +
                    "/under2FromClose=" + F(_strictRecoverySeconds) +
                    "/sourceDuration=" + F(_recoverySourceEventDurationSeconds) +
                    "/closeAge=" + F(Mathf.Max(0f, now - _recoveryCloseTime));

                Plugin.Logger?.LogWarning(
                    "[ARL][SOURCE_TURN_RECOVERY] event=" + _recoveryEventId +
                    " | pass=" + pass +
                    " | under5SecondsFromSourceClose=" + F(_looseRecoverySeconds) +
                    " | under2SecondsFromSourceClose=" + F(_strictRecoverySeconds) +
                    " | sourceEventDuration=" + F(_recoverySourceEventDurationSeconds) +
                    " | anchorMaxSession=" + F(_currentRecoveryMaxAnchorErrorMeters) +
                    " | stableHold=" + F(RecoveryStableHoldSeconds));

                if (!pass)
                    EmitRecoveryFailureAttribution("RECOVERY_TIME_LIMIT_FAIL_AFTER_SOURCE_CLOSE", Mathf.Max(0f, now - _recoveryCloseTime));

                ResetRecoveryState();
                return;
            }

            float elapsedFromSourceClose = Mathf.Max(0f, now - _recoveryCloseTime);
            if (elapsedFromSourceClose > RecoveryStrictMaxSecondsFromSourceClose + RecoveryStableHoldSeconds + RecoveryStableHoldNumericalToleranceSeconds)
                FailRecovery("RECOVERY_TIMEOUT_AFTER_SOURCE_CLOSE", now);
        }

        private void UpdateRecoveryThreshold(
            bool insideThreshold,
            float now,
            ref float stableStart,
            ref bool recovered,
            ref float recoverySeconds)
        {
            if (recovered)
                return;

            if (!insideThreshold)
            {
                stableStart = -1f;
                return;
            }

            if (stableStart < 0f)
                stableStart = now;

            if (!StableHoldSatisfied(now - stableStart))
                return;

            recovered = true;
            recoverySeconds = Mathf.Max(0f, stableStart - _recoveryCloseTime);
        }

        private void FailRecovery(string reason, float now)
        {
            if (!_recoveryActive)
                return;

            FinalizeCurrentRecoveryIntervals(now);
            _recoveryCompletedCount++;
            _recoveryFailedCount++;
            CommitCurrentRecoveryHealthToCompleted();
            float elapsedFromSourceClose = Mathf.Max(0f, now - _recoveryCloseTime);
            if (_looseRecovered)
                _recoveryMaxLooseSeconds = Mathf.Max(_recoveryMaxLooseSeconds, _looseRecoverySeconds);
            else
                _recoveryMaxLooseSeconds = Mathf.Max(_recoveryMaxLooseSeconds, elapsedFromSourceClose);
            if (_strictRecovered)
                _recoveryMaxStrictSeconds = Mathf.Max(_recoveryMaxStrictSeconds, _strictRecoverySeconds);
            else
                _recoveryMaxStrictSeconds = Mathf.Max(_recoveryMaxStrictSeconds, elapsedFromSourceClose);

            _lastRecoveryResult =
                "EVENT_" + _recoveryEventId + ":FAIL/" + reason +
                "/elapsedFromClose=" + F(elapsedFromSourceClose) +
                "/sourceDuration=" + F(_recoverySourceEventDurationSeconds);
            Plugin.Logger?.LogWarning(
                "[ARL][SOURCE_TURN_RECOVERY] event=" + _recoveryEventId +
                " | pass=False | evaluable=True | reason=" + reason +
                " | elapsedFromSourceClose=" + F(elapsedFromSourceClose) +
                " | sourceEventDuration=" + F(_recoverySourceEventDurationSeconds));
            EmitRecoveryFailureAttribution(reason, elapsedFromSourceClose);
            ResetRecoveryState();
        }

        private void CensorRecovery(string reason, float now, bool newSourceEvent)
        {
            if (!_recoveryActive)
                return;

            FinalizeCurrentRecoveryIntervals(now);
            _recoveryCensoredCount++;
            CommitCurrentRecoveryHealthToCensored();
            if (newSourceEvent)
                _recoveryCensoredByNewSourceEventCount++;
            else
                _recoveryCensoredByTraceStopCount++;

            float elapsedFromSourceClose = Mathf.Max(0f, now - _recoveryCloseTime);
            _lastRecoveryResult =
                "EVENT_" + _recoveryEventId + ":CENSORED/" + reason +
                "/elapsedFromClose=" + F(elapsedFromSourceClose) +
                "/sourceDuration=" + F(_recoverySourceEventDurationSeconds);
            Plugin.Logger?.LogWarning(
                "[ARL][SOURCE_TURN_RECOVERY] event=" + _recoveryEventId +
                " | pass=<not-evaluable> | censored=True | reason=" + reason +
                " | elapsedFromSourceClose=" + F(elapsedFromSourceClose) +
                " | sourceEventDuration=" + F(_recoverySourceEventDurationSeconds));
            ResetRecoveryState();
        }

        private void CensorRecoveryForDriveAbort(float now)
        {
            if (!_recoveryActive)
                return;

            FinalizeCurrentRecoveryIntervals(now);
            _recoveryCensoredCount++;
            CommitCurrentRecoveryHealthToCensored();
            float elapsedFromSourceClose = Mathf.Max(0f, now - _recoveryCloseTime);
            _lastRecoveryResult =
                "EVENT_" + _recoveryEventId + ":CENSORED/CENSORED_BY_DRIVE_ABORT" +
                "/elapsedFromClose=" + F(elapsedFromSourceClose) +
                "/sourceDuration=" + F(_recoverySourceEventDurationSeconds);
            Plugin.Logger?.LogWarning(
                "[ARL][SOURCE_TURN_RECOVERY] event=" + _recoveryEventId +
                " | pass=<not-evaluable> | censored=True | reason=CENSORED_BY_DRIVE_ABORT" +
                " | elapsedFromSourceClose=" + F(elapsedFromSourceClose) +
                " | sourceEventDuration=" + F(_recoverySourceEventDurationSeconds));
            ResetRecoveryState();
        }

        private void CommitCurrentRecoveryHealthToCompleted()
        {
            _recoveryInvalidStateSteps += _currentRecoveryInvalidStateSteps;
            _recoveryParentFeedforwardClampSteps += _currentRecoveryParentFeedforwardClampSteps;
            _recoveryMaxAnchorErrorMeters = Mathf.Max(_recoveryMaxAnchorErrorMeters, _currentRecoveryMaxAnchorErrorMeters);
        }

        private void CommitCurrentRecoveryHealthToCensored()
        {
            _censoredRecoveryInvalidStateSteps += _currentRecoveryInvalidStateSteps;
            _censoredRecoveryParentFeedforwardClampSteps += _currentRecoveryParentFeedforwardClampSteps;
            _censoredRecoveryMaxAnchorErrorMeters = Mathf.Max(_censoredRecoveryMaxAnchorErrorMeters, _currentRecoveryMaxAnchorErrorMeters);
        }

        private void Evaluate(TwoBodyProxyReport report, SourceTurnDiscontinuityDiagnostics sourceTurn)
        {
            if (TraceAborted)
            {
                ValidationComplete = true;
                NormalTrackingPass = false;
                SourceDiscontinuitySurvivalPass = false;
                RecoveryPass = false;
                TwoBodyFreezeReady = false;
                DecisionReason = "DRIVE_DROPPED_DURING_TRACE";
                return;
            }

            ValidationComplete = true;

            float steadyRotMean = Mean(_normalSteadyRotationSum, _normalSteadySamples);
            float steadyPosMean = Mean(_normalSteadyChestPositionSum, _normalSteadySamples);
            float steadyAnchorMean = Mean(_normalSteadyAnchorRelativeSpeedSum, _normalSteadySamples);
            float turnRotMean = Mean(_normalTurnRotationSum, _normalTurnSamples);

            bool normalDataEnough =
                _normalSteadySamples >= MinNormalSteadySamples &&
                _normalTurnSamples >= MinNormalTurnSamples;
            NormalTrackingPass =
                normalDataEnough &&
                IsHealthy(report) &&
                steadyRotMean <= NormalSteadyRotationMeanMaxDegrees &&
                _normalSteadyRotationMax <= NormalSteadyRotationPeakMaxDegrees &&
                steadyPosMean <= NormalSteadyChestPositionMeanMaxMeters &&
                steadyAnchorMean <= NormalSteadyAnchorRelativeSpeedMeanMax &&
                _normalSteadyAnchorRelativeSpeedMax <= NormalSteadyAnchorRelativeSpeedPeakMax &&
                turnRotMean <= NormalTurnRotationMeanMaxDegrees &&
                _normalTurnRotationMax <= NormalTurnRotationPeakMaxDegrees &&
                _normalAngularClampSteps <= NormalAngularClampMaxSteps &&
                _normalParentFeedforwardClampSteps <= NormalParentFeedforwardClampMaxSteps;

            int sourceEvents = sourceTurn != null ? sourceTurn.EventCount : 0;
            int sourceDiscontinuities = sourceTurn != null ? sourceTurn.DiscontinuityEventCount : 0;
            bool sourceDataEnough =
                sourceEvents >= MinSourceTurnEvents &&
                sourceDiscontinuities >= MinSourceDiscontinuityEvents;
            SourceDiscontinuitySurvivalPass =
                sourceDataEnough &&
                _sourceEventFixedSteps > 0 &&
                _sourceInvalidStateSteps == 0 &&
                _sourceParentFeedforwardClampSteps <= SourceTurnParentFeedforwardClampMaxSteps &&
                _sourceMaxAnchorErrorMeters <= SourceTurnMaxAnchorErrorMeters;

            bool recoveryDataEnough = _recoveryCompletedCount >= MinCompletedRecoverySessions;
            RecoveryPass =
                sourceDataEnough &&
                recoveryDataEnough &&
                _recoveryPassedCount == _recoveryCompletedCount &&
                _recoveryFailedCount == 0 &&
                _recoveryInvalidStateSteps == 0 &&
                _recoveryParentFeedforwardClampSteps == 0 &&
                _recoveryMaxLooseSeconds <= RecoveryLooseMaxSecondsFromSourceClose &&
                _recoveryMaxStrictSeconds <= RecoveryStrictMaxSecondsFromSourceClose;

            TwoBodyFreezeReady = NormalTrackingPass && SourceDiscontinuitySurvivalPass && RecoveryPass;

            if (TwoBodyFreezeReady)
            {
                DecisionReason = "ALL_GATES_PASS";
                return;
            }

            if (!normalDataEnough)
                DecisionReason = "INSUFFICIENT_NORMAL_DATA";
            else if (!NormalTrackingPass)
                DecisionReason = "NORMAL_TRACKING_GATE_FAIL";
            else if (!sourceDataEnough)
                DecisionReason = "INSUFFICIENT_SOURCE_TURN_DATA";
            else if (!SourceDiscontinuitySurvivalPass)
                DecisionReason = "SOURCE_DISCONTINUITY_SURVIVAL_GATE_FAIL";
            else if (_recoveryCompletedCount < MinCompletedRecoverySessions)
                DecisionReason = "INSUFFICIENT_COMPLETED_RECOVERY_DATA";
            else if (!RecoveryPass)
                DecisionReason = "RECOVERY_GATE_FAIL";
            else
                DecisionReason = "UNKNOWN_GATE_FAIL";
        }

        private static bool IsHealthy(TwoBodyProxyReport report)
        {
            return report != null &&
                   report.FiniteState &&
                   report.SourceAlive &&
                   report.CharacterJointPresent &&
                   report.CharacterJointConnectedBodyMatch;
        }

        private void ResetRecoveryState()
        {
            _recoveryActive = false;
            _recoveryEventId = 0;
            _recoveryEventStartTime = 0f;
            _recoveryCloseTime = 0f;
            _recoverySourceEventDurationSeconds = 0f;
            _looseStableStart = -1f;
            _strictStableStart = -1f;
            _looseRecovered = false;
            _strictRecovered = false;
            _looseRecoverySeconds = 0f;
            _strictRecoverySeconds = 0f;
            _currentRecoveryInvalidStateSteps = 0;
            _currentRecoveryParentFeedforwardClampSteps = 0;
            _currentRecoveryMaxAnchorErrorMeters = 0f;
            ResetCurrentRecoveryAttribution();
        }

        private void ResetCurrentRecoveryAttribution()
        {
            _currentRecoverySampleCount = 0;
            _currentRecoveryRotationSum = 0f;
            _currentRecoveryRotationMinDegrees = float.MaxValue;
            _currentRecoveryRotationMaxDegrees = 0f;
            _currentRecoveryFirstLooseEntrySeconds = -1f;
            _currentRecoveryFirstStrictEntrySeconds = -1f;
            _currentRecoveryLooseIntervalStart = -1f;
            _currentRecoveryStrictIntervalStart = -1f;
            _currentRecoveryLongestLooseContinuousSeconds = 0f;
            _currentRecoveryLongestStrictContinuousSeconds = 0f;
            _currentRecoveryLooseEntryCount = 0;
            _currentRecoveryStrictEntryCount = 0;
            _currentRecoveryLooseRotationBreakCount = 0;
            _currentRecoveryStrictRotationBreakCount = 0;
            _currentRecoveryLooseAnchorBreakCount = 0;
            _currentRecoveryStrictAnchorBreakCount = 0;
            _currentRecoveryTargetSpeedSum = 0f;
            _currentRecoveryTargetSpeedMax = 0f;
            _currentRecoveryTargetRelativeAngularSpeedSum = 0f;
            _currentRecoveryTargetRelativeAngularSpeedMax = 0f;
            _currentRecoveryLastMotionPhase = "<none>";
            _currentRecoveryLastMotionTransition = false;
            _currentRecoveryEndRotationDegrees = 0f;
            _currentRecoveryEndTargetSpeed = 0f;
            _currentRecoveryEndTargetRelativeAngularSpeed = 0f;
        }

        private void ObserveRecoveryAttributionStep(TwoBodyProxyReport report, float now, bool anchorRecovered)
        {
            float rotation = Mathf.Max(0f, report.RotationErrorDegrees);
            float elapsed = Mathf.Max(0f, now - _recoveryCloseTime);
            float targetSpeed = Mathf.Max(0f, report.MotionTargetSpeed);
            float targetRelAngularSpeed = report.TargetRelativeAngularVelocity.magnitude;

            _currentRecoverySampleCount++;
            _currentRecoveryRotationSum += rotation;
            _currentRecoveryRotationMinDegrees = Mathf.Min(_currentRecoveryRotationMinDegrees, rotation);
            _currentRecoveryRotationMaxDegrees = Mathf.Max(_currentRecoveryRotationMaxDegrees, rotation);
            _currentRecoveryTargetSpeedSum += targetSpeed;
            _currentRecoveryTargetSpeedMax = Mathf.Max(_currentRecoveryTargetSpeedMax, targetSpeed);
            _currentRecoveryTargetRelativeAngularSpeedSum += targetRelAngularSpeed;
            _currentRecoveryTargetRelativeAngularSpeedMax = Mathf.Max(_currentRecoveryTargetRelativeAngularSpeedMax, targetRelAngularSpeed);
            _currentRecoveryLastMotionPhase = report.MotionPhase ?? "<none>";
            _currentRecoveryLastMotionTransition = report.MotionTransition;
            _currentRecoveryEndRotationDegrees = rotation;
            _currentRecoveryEndTargetSpeed = targetSpeed;
            _currentRecoveryEndTargetRelativeAngularSpeed = targetRelAngularSpeed;

            ObserveAttributionThreshold(
                rotation <= RecoveryRotationLooseDegrees && anchorRecovered,
                rotation > RecoveryRotationLooseDegrees,
                anchorRecovered,
                elapsed,
                ref _currentRecoveryFirstLooseEntrySeconds,
                ref _currentRecoveryLooseIntervalStart,
                ref _currentRecoveryLongestLooseContinuousSeconds,
                ref _currentRecoveryLooseEntryCount,
                ref _currentRecoveryLooseRotationBreakCount,
                ref _currentRecoveryLooseAnchorBreakCount);

            ObserveAttributionThreshold(
                rotation <= RecoveryRotationStrictDegrees && anchorRecovered,
                rotation > RecoveryRotationStrictDegrees,
                anchorRecovered,
                elapsed,
                ref _currentRecoveryFirstStrictEntrySeconds,
                ref _currentRecoveryStrictIntervalStart,
                ref _currentRecoveryLongestStrictContinuousSeconds,
                ref _currentRecoveryStrictEntryCount,
                ref _currentRecoveryStrictRotationBreakCount,
                ref _currentRecoveryStrictAnchorBreakCount);
        }

        private void ObserveAttributionThreshold(
            bool inside,
            bool rotationOutside,
            bool anchorRecovered,
            float elapsed,
            ref float firstEntry,
            ref float intervalStart,
            ref float longest,
            ref int entryCount,
            ref int rotationBreakCount,
            ref int anchorBreakCount)
        {
            if (inside)
            {
                if (intervalStart < 0f)
                {
                    intervalStart = elapsed;
                    entryCount++;
                    if (firstEntry < 0f)
                        firstEntry = elapsed;
                }
                longest = Mathf.Max(longest, Mathf.Max(0f, elapsed - intervalStart));
                return;
            }

            if (intervalStart >= 0f)
            {
                longest = Mathf.Max(longest, Mathf.Max(0f, elapsed - intervalStart));
                if (!anchorRecovered)
                    anchorBreakCount++;
                else if (rotationOutside)
                    rotationBreakCount++;
                intervalStart = -1f;
            }
        }

        private void FinalizeCurrentRecoveryIntervals(float now)
        {
            float elapsed = Mathf.Max(0f, now - _recoveryCloseTime);
            if (_currentRecoveryLooseIntervalStart >= 0f)
                _currentRecoveryLongestLooseContinuousSeconds = Mathf.Max(_currentRecoveryLongestLooseContinuousSeconds, elapsed - _currentRecoveryLooseIntervalStart);
            if (_currentRecoveryStrictIntervalStart >= 0f)
                _currentRecoveryLongestStrictContinuousSeconds = Mathf.Max(_currentRecoveryLongestStrictContinuousSeconds, elapsed - _currentRecoveryStrictIntervalStart);
        }

        private void EmitRecoveryFailureAttribution(string reason, float elapsedFromSourceClose)
        {
            _recoveryFailureAttributionCount++;
            float rotationMin = _currentRecoverySampleCount > 0 ? _currentRecoveryRotationMinDegrees : 0f;
            float rotationMean = Mean(_currentRecoveryRotationSum, _currentRecoverySampleCount);
            float targetSpeedMean = Mean(_currentRecoveryTargetSpeedSum, _currentRecoverySampleCount);
            float targetRelAngularSpeedMean = Mean(_currentRecoveryTargetRelativeAngularSpeedSum, _currentRecoverySampleCount);
            string pattern = ClassifyRecoveryFailurePattern();

            _lastRecoveryAttribution =
                "EVENT_" + _recoveryEventId + ":" + reason +
                "/pattern=" + pattern +
                "/rot=" + F(rotationMin) + "/" + F(rotationMean) + "/" + F(_currentRecoveryRotationMaxDegrees) + "/end" + F(_currentRecoveryEndRotationDegrees) +
                "/first5=" + FOrNever(_currentRecoveryFirstLooseEntrySeconds) + "/first2=" + FOrNever(_currentRecoveryFirstStrictEntrySeconds) +
                "/longest5=" + F(_currentRecoveryLongestLooseContinuousSeconds) + "/longest2=" + F(_currentRecoveryLongestStrictContinuousSeconds) +
                "/entries5=" + _currentRecoveryLooseEntryCount + "/entries2=" + _currentRecoveryStrictEntryCount +
                "/break5=" + _currentRecoveryLooseRotationBreakCount + "/" + _currentRecoveryLooseAnchorBreakCount + "/break2=" + _currentRecoveryStrictRotationBreakCount + "/" + _currentRecoveryStrictAnchorBreakCount +
                "/phase=" + _currentRecoveryLastMotionPhase + "/transition=" + _currentRecoveryLastMotionTransition +
                "/speedMeanMaxEnd=" + F(targetSpeedMean) + "/" + F(_currentRecoveryTargetSpeedMax) + "/" + F(_currentRecoveryEndTargetSpeed) +
                "/relWMeanMaxEnd=" + F(targetRelAngularSpeedMean) + "/" + F(_currentRecoveryTargetRelativeAngularSpeedMax) + "/" + F(_currentRecoveryEndTargetRelativeAngularSpeed);

            Plugin.Logger?.LogWarning(
                "[ARL][RECOVERY_ATTRIBUTION] event=" + _recoveryEventId +
                " | reason=" + reason +
                " | pattern=" + pattern +
                " | elapsedFromSourceClose=" + F(elapsedFromSourceClose) +
                " | sourceEventDuration=" + F(_recoverySourceEventDurationSeconds) +
                " | rotation(min/mean/max/end)=" + F(rotationMin) + "/" + F(rotationMean) + "/" + F(_currentRecoveryRotationMaxDegrees) + "/" + F(_currentRecoveryEndRotationDegrees) +
                " | firstEntry5/2=" + FOrNever(_currentRecoveryFirstLooseEntrySeconds) + "/" + FOrNever(_currentRecoveryFirstStrictEntrySeconds) +
                " | longestContinuousBelow5/2=" + F(_currentRecoveryLongestLooseContinuousSeconds) + "/" + F(_currentRecoveryLongestStrictContinuousSeconds) +
                " | entries5/2=" + _currentRecoveryLooseEntryCount + "/" + _currentRecoveryStrictEntryCount +
                " | holdBreaks5(rot/anchor)=" + _currentRecoveryLooseRotationBreakCount + "/" + _currentRecoveryLooseAnchorBreakCount +
                " | holdBreaks2(rot/anchor)=" + _currentRecoveryStrictRotationBreakCount + "/" + _currentRecoveryStrictAnchorBreakCount +
                " | health(invalid/parentFFClamp/maxAnchor)=" + _currentRecoveryInvalidStateSteps + "/" + _currentRecoveryParentFeedforwardClampSteps + "/" + F(_currentRecoveryMaxAnchorErrorMeters) +
                " | endMotion=" + _currentRecoveryLastMotionPhase + "/transition=" + _currentRecoveryLastMotionTransition +
                " | targetSpeed(mean/max/end)=" + F(targetSpeedMean) + "/" + F(_currentRecoveryTargetSpeedMax) + "/" + F(_currentRecoveryEndTargetSpeed) +
                " | targetRelAngularSpeed(mean/max/end)=" + F(targetRelAngularSpeedMean) + "/" + F(_currentRecoveryTargetRelativeAngularSpeedMax) + "/" + F(_currentRecoveryEndTargetRelativeAngularSpeed));
        }


        private string ClassifyRecoveryFailurePattern()
        {
            if (_currentRecoveryFirstStrictEntrySeconds < 0f)
                return "NEVER_ENTERED_STRICT";
            if (!StableHoldSatisfied(_currentRecoveryLongestStrictContinuousSeconds))
            {
                if (_currentRecoveryStrictEntryCount > 1)
                    return "REPEATED_STRICT_HOLD_BREAK";
                return "STRICT_HOLD_TOO_SHORT";
            }
            if (_currentRecoveryEndRotationDegrees > RecoveryRotationStrictDegrees)
                return "RETURNED_ABOVE_STRICT";
            return "LATE_OR_GATE_EDGE";
        }

        private static string FOrNever(float value) => value < 0f ? "never" : F(value);

        internal static string GetThresholdSummary()
        {
            return
                "steadySamples>=" + MinNormalSteadySamples +
                ",normalTurnSamples>=" + MinNormalTurnSamples +
                ",steadyRotMean<=" + F(NormalSteadyRotationMeanMaxDegrees) +
                ",steadyRotMax<=" + F(NormalSteadyRotationPeakMaxDegrees) +
                ",steadyPosMean<=" + F(NormalSteadyChestPositionMeanMaxMeters) +
                ",steadyAnchorMean<=" + F(NormalSteadyAnchorRelativeSpeedMeanMax) +
                ",steadyAnchorMax<=" + F(NormalSteadyAnchorRelativeSpeedPeakMax) +
                ",normalTurnRotMean<=" + F(NormalTurnRotationMeanMaxDegrees) +
                ",normalTurnRotMax<=" + F(NormalTurnRotationPeakMaxDegrees) +
                ",normalAngularClamp<=" + NormalAngularClampMaxSteps +
                ",normalParentFFClamp<=" + NormalParentFeedforwardClampMaxSteps +
                ",sourceEvents>=" + MinSourceTurnEvents +
                ",sourceDiscontinuities>=" + MinSourceDiscontinuityEvents +
                ",sourceAnchorMax<=" + F(SourceTurnMaxAnchorErrorMeters) +
                ",sourceParentFFClamp<=" + SourceTurnParentFeedforwardClampMaxSteps +
                ",recoveryCompleted>=" + MinCompletedRecoverySessions +
                ",recovery5FromSourceClose<=" + F(RecoveryLooseMaxSecondsFromSourceClose) +
                ",recovery2FromSourceClose<=" + F(RecoveryStrictMaxSecondsFromSourceClose) +
                ",recoveryAnchor<=" + F(RecoveryAnchorErrorMaxMeters) +
                ",hold=" + F(RecoveryStableHoldSeconds) +
                ",holdNumericalTolerance=" + F(RecoveryStableHoldNumericalToleranceSeconds);
        }

        private static bool StableHoldSatisfied(float elapsed)
        {
            return elapsed + RecoveryStableHoldNumericalToleranceSeconds >= RecoveryStableHoldSeconds;
        }

        private static float Mean(float sum, int count) => count > 0 ? sum / count : 0f;
        private static string F(float value) => value.ToString("G6", CultureInfo.InvariantCulture);
    }
}
