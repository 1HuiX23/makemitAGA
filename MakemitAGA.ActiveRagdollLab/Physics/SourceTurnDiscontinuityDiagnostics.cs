using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.1.12 pure-diagnostic helper.
    ///
    /// Source Animator bones live on the render clock.  This class observes the RAW delta between
    /// consecutive real LateUpdate samples before the normal velocity low-pass is applied.  It does
    /// not feed any value back into CharacterJoint, PD, Parent FF or the Render->Physics Pose Bridge.
    ///
    /// The goal is attribution: distinguish a physically continuous but very fast turn from a source
    /// orientation discontinuity / extremely short blend. v0.1.16 keeps the full in-memory pre/post
    /// aggregation but defaults the per-sample context dump OFF to keep BepInEx logs manageable;
    /// EVENT/SUMMARY lines remain available, and profile can re-enable verbose context when needed.
    /// </summary>
    internal sealed class SourceTurnDiscontinuityDiagnostics
    {
        internal const string ModeName = "SOURCE_TURN_DISCONTINUITY_ATTRIBUTION";
        internal bool Enabled { get; set; } = true;

        private sealed class Sample
        {
            public int Sequence;
            public float Time;
            public float DeltaTime;
            public float RotationDeltaDegrees;
            public float YawDegrees;
            public float YawDeltaDegrees;
            public Vector3 RawAngularVelocity;
            public float RawAngularSpeed;
            public Vector3 RawAngularAcceleration;
            public float RawAngularAccelerationMagnitude;
            public float ChestRelativeRotationDeltaDegrees;
        }

        private readonly List<Sample> _history = new List<Sample>(96);

        private bool _sessionActive;
        private float _sessionStartTime;
        private bool _hasPreviousRawAngularVelocity;
        private Vector3 _previousRawAngularVelocity;

        private bool _eventActive;
        private int _eventId;
        private float _eventTriggerTime;
        private float _eventContextStartTime;
        private float _eventQuietAfterTime;
        private string _eventTriggerReason = "<none>";
        private bool _eventSawSingleSampleDiscontinuity;
        private float _eventSignedYawDeltaDegrees;
        private float _eventAbsoluteYawDeltaDegrees;
        private float _eventMaxRotationDeltaDegrees;
        private float _eventMaxAbsYawDeltaDegrees;
        private float _eventMaxRawAngularSpeed;
        private float _eventMaxRawAngularAcceleration;
        private float _eventMaxChestRelativeRotationDeltaDegrees;
        private float _eventMaxResponseRotationErrorDegrees;
        private float _eventMaxPoseBridgeRotationErrorDegrees;
        private float _eventMaxPoseBridgeCommandAngularSpeed;
        private float _eventMaxAnchorErrorMeters;
        private int _eventAngularClampFixedStepCount;

        // Current raw render-sample diagnostics.
        public float CurrentSampleDeltaSeconds { get; private set; }
        public float CurrentRotationDeltaDegrees { get; private set; }
        public float CurrentYawDegrees { get; private set; }
        public float CurrentYawDeltaDegrees { get; private set; }
        public Vector3 CurrentRawAngularVelocity { get; private set; }
        public float CurrentRawAngularSpeed { get; private set; }
        public Vector3 CurrentRawAngularAcceleration { get; private set; }
        public float CurrentRawAngularAccelerationMagnitude { get; private set; }
        public float CurrentChestRelativeRotationDeltaDegrees { get; private set; }

        // Trace-session aggregate diagnostics.
        public bool EventActive => _eventActive;
        public int CurrentEventId => _eventActive ? _eventId : 0;
        public int EventCount { get; private set; }
        public int DiscontinuityEventCount { get; private set; }
        public int RapidContinuousEventCount { get; private set; }
        public float MaxSingleSampleRotationDeltaDegrees { get; private set; }
        public float MaxSingleSampleAbsYawDeltaDegrees { get; private set; }
        public float MaxRawAngularSpeed { get; private set; }
        public float MaxRawAngularAcceleration { get; private set; }
        public float MaxChestRelativeRotationDeltaDegrees { get; private set; }
        public float MaxResponseRotationErrorDegrees { get; private set; }
        public float MaxPoseBridgeRotationErrorDegrees { get; private set; }
        public float MaxPoseBridgeCommandAngularSpeed { get; private set; }
        public float MaxAnchorErrorMeters { get; private set; }
        public int AngularClampFixedStepCount { get; private set; }
        public string LastEventClassification { get; private set; } = "<none>";
        public float LastEventSignedYawDeltaDegrees { get; private set; }
        public float LastEventAbsoluteYawDeltaDegrees { get; private set; }
        public float LastEventMaxSingleSampleRotationDeltaDegrees { get; private set; }
        public float LastEventMaxRawAngularSpeed { get; private set; }
        public float LastEventMaxResponseRotationErrorDegrees { get; private set; }

        public void ResetAll()
        {
            _history.Clear();
            _sessionActive = false;
            _sessionStartTime = 0f;
            _hasPreviousRawAngularVelocity = false;
            _previousRawAngularVelocity = Vector3.zero;
            ClearCurrentSample();
            ResetSessionStatistics();
        }

        public void BeginTraceSession()
        {
            // Keep the short history so an event immediately after arl_trace(on) can still be understood,
            // but never dump context older than the trace session itself.
            _sessionActive = true;
            _sessionStartTime = Time.unscaledTime;
            ResetSessionStatistics();
        }

        public void EndTraceSession()
        {
            if (_eventActive)
                FinalizeEvent("TRACE_STOP", Time.unscaledTime);
            _sessionActive = false;
        }

        public void ObserveRenderSample(
            int sequence,
            float sampleTime,
            float dt,
            Quaternion previousPelvisRotation,
            Quaternion currentPelvisRotation,
            Quaternion previousChestRelativeRotation,
            Quaternion currentChestRelativeRotation,
            Vector3 rawPelvisAngularVelocity,
            float rotationDeltaThresholdDegrees,
            float angularSpeedThreshold,
            float angularAccelerationThreshold,
            float preContextSeconds,
            float postContextSeconds,
            bool verboseContextLogging)
        {
            if (!Enabled) return;
            if (!IsFinite(sampleTime) || !IsFinite(dt) || dt < 0.001f || dt > 0.20f ||
                !IsFinite(previousPelvisRotation) || !IsFinite(currentPelvisRotation) ||
                !IsFinite(previousChestRelativeRotation) || !IsFinite(currentChestRelativeRotation) ||
                !IsFinite(rawPelvisAngularVelocity))
            {
                ClearCurrentSample();
                _hasPreviousRawAngularVelocity = false;
                _previousRawAngularVelocity = Vector3.zero;
                return;
            }

            float rotationDelta = Quaternion.Angle(previousPelvisRotation, currentPelvisRotation);
            float yawDegrees = ComputeYawDegrees(currentPelvisRotation);
            float yawDelta = ComputeSignedYawDelta(previousPelvisRotation, currentPelvisRotation);
            float chestRelativeDelta = Quaternion.Angle(previousChestRelativeRotation, currentChestRelativeRotation);
            Vector3 rawAngularAcceleration = _hasPreviousRawAngularVelocity
                ? (rawPelvisAngularVelocity - _previousRawAngularVelocity) / dt
                : Vector3.zero;

            _previousRawAngularVelocity = rawPelvisAngularVelocity;
            _hasPreviousRawAngularVelocity = true;

            Sample sample = new Sample
            {
                Sequence = sequence,
                Time = sampleTime,
                DeltaTime = dt,
                RotationDeltaDegrees = rotationDelta,
                YawDegrees = yawDegrees,
                YawDeltaDegrees = yawDelta,
                RawAngularVelocity = rawPelvisAngularVelocity,
                RawAngularSpeed = rawPelvisAngularVelocity.magnitude,
                RawAngularAcceleration = rawAngularAcceleration,
                RawAngularAccelerationMagnitude = rawAngularAcceleration.magnitude,
                ChestRelativeRotationDeltaDegrees = chestRelativeDelta
            };

            CurrentSampleDeltaSeconds = sample.DeltaTime;
            CurrentRotationDeltaDegrees = sample.RotationDeltaDegrees;
            CurrentYawDegrees = sample.YawDegrees;
            CurrentYawDeltaDegrees = sample.YawDeltaDegrees;
            CurrentRawAngularVelocity = sample.RawAngularVelocity;
            CurrentRawAngularSpeed = sample.RawAngularSpeed;
            CurrentRawAngularAcceleration = sample.RawAngularAcceleration;
            CurrentRawAngularAccelerationMagnitude = sample.RawAngularAccelerationMagnitude;
            CurrentChestRelativeRotationDeltaDegrees = sample.ChestRelativeRotationDeltaDegrees;

            _history.Add(sample);
            float historyCutoff = sampleTime - Mathf.Max(0.05f, preContextSeconds);
            while (_history.Count > 0 && _history[0].Time < historyCutoff)
                _history.RemoveAt(0);

            if (!_sessionActive)
                return;

            MaxSingleSampleRotationDeltaDegrees = Mathf.Max(MaxSingleSampleRotationDeltaDegrees, rotationDelta);
            MaxSingleSampleAbsYawDeltaDegrees = Mathf.Max(MaxSingleSampleAbsYawDeltaDegrees, Mathf.Abs(yawDelta));
            MaxRawAngularSpeed = Mathf.Max(MaxRawAngularSpeed, sample.RawAngularSpeed);
            MaxRawAngularAcceleration = Mathf.Max(MaxRawAngularAcceleration, sample.RawAngularAccelerationMagnitude);
            MaxChestRelativeRotationDeltaDegrees = Mathf.Max(MaxChestRelativeRotationDeltaDegrees, chestRelativeDelta);

            bool rotationTrigger = rotationDelta >= rotationDeltaThresholdDegrees;
            bool speedTrigger = sample.RawAngularSpeed >= angularSpeedThreshold;
            // Raw angular acceleration is intentionally unfiltered and can amplify tiny pose jitter.
            // Require a little real rotational motion so stationary numerical noise does not create events.
            bool accelerationTrigger =
                sample.RawAngularAccelerationMagnitude >= angularAccelerationThreshold &&
                (sample.RawAngularSpeed >= 0.75f || rotationDelta >= 2.0f);
            bool triggered = rotationTrigger || speedTrigger || accelerationTrigger;
            string triggerReason = BuildTriggerReason(rotationTrigger, speedTrigger, accelerationTrigger);

            if (_eventActive && sampleTime > _eventQuietAfterTime && !triggered)
                FinalizeEvent("QUIET_WINDOW_COMPLETE", sampleTime);

            if (triggered && !_eventActive)
            {
                StartEvent(sample, triggerReason, rotationTrigger, preContextSeconds, postContextSeconds, verboseContextLogging);
            }
            else if (triggered && _eventActive)
            {
                _eventQuietAfterTime = Mathf.Max(_eventQuietAfterTime, sampleTime + postContextSeconds);
                if (rotationTrigger)
                    _eventSawSingleSampleDiscontinuity = true;
            }

            if (_eventActive)
            {
                AccumulateEventSample(sample);
                if (verboseContextLogging)
                    Plugin.Logger?.LogInfo(FormatContextLine(sample, _eventId, sample.Time - _eventTriggerTime, false, _eventSignedYawDeltaDegrees));
            }
        }

        public void ObservePhysicsResponse(
            float rotationErrorDegrees,
            float anchorErrorMeters,
            float poseBridgeRotationErrorDegrees,
            Vector3 poseBridgeCommandAngularVelocity,
            bool angularAccelerationClamped)
        {
            if (!Enabled) return;
            if (!_sessionActive || !_eventActive)
                return;

            float cmdAngularSpeed = IsFinite(poseBridgeCommandAngularVelocity)
                ? poseBridgeCommandAngularVelocity.magnitude
                : 0f;

            _eventMaxResponseRotationErrorDegrees = Mathf.Max(_eventMaxResponseRotationErrorDegrees, Mathf.Max(0f, rotationErrorDegrees));
            _eventMaxAnchorErrorMeters = Mathf.Max(_eventMaxAnchorErrorMeters, Mathf.Max(0f, anchorErrorMeters));
            _eventMaxPoseBridgeRotationErrorDegrees = Mathf.Max(_eventMaxPoseBridgeRotationErrorDegrees, Mathf.Max(0f, poseBridgeRotationErrorDegrees));
            _eventMaxPoseBridgeCommandAngularSpeed = Mathf.Max(_eventMaxPoseBridgeCommandAngularSpeed, cmdAngularSpeed);
            if (angularAccelerationClamped)
                _eventAngularClampFixedStepCount++;

            MaxResponseRotationErrorDegrees = Mathf.Max(MaxResponseRotationErrorDegrees, _eventMaxResponseRotationErrorDegrees);
            MaxAnchorErrorMeters = Mathf.Max(MaxAnchorErrorMeters, _eventMaxAnchorErrorMeters);
            MaxPoseBridgeRotationErrorDegrees = Mathf.Max(MaxPoseBridgeRotationErrorDegrees, _eventMaxPoseBridgeRotationErrorDegrees);
            MaxPoseBridgeCommandAngularSpeed = Mathf.Max(MaxPoseBridgeCommandAngularSpeed, _eventMaxPoseBridgeCommandAngularSpeed);
            if (angularAccelerationClamped)
                AngularClampFixedStepCount++;
        }

        private void StartEvent(
            Sample triggerSample,
            string triggerReason,
            bool rotationTrigger,
            float preContextSeconds,
            float postContextSeconds,
            bool verboseContextLogging)
        {
            _eventActive = true;
            _eventId++;
            EventCount++;
            _eventTriggerTime = triggerSample.Time;
            _eventContextStartTime = Mathf.Max(_sessionStartTime, triggerSample.Time - Mathf.Max(0f, preContextSeconds));
            _eventQuietAfterTime = triggerSample.Time + Mathf.Max(0.05f, postContextSeconds);
            _eventTriggerReason = triggerReason;
            _eventSawSingleSampleDiscontinuity = rotationTrigger;
            _eventSignedYawDeltaDegrees = 0f;
            _eventAbsoluteYawDeltaDegrees = 0f;
            _eventMaxRotationDeltaDegrees = 0f;
            _eventMaxAbsYawDeltaDegrees = 0f;
            _eventMaxRawAngularSpeed = 0f;
            _eventMaxRawAngularAcceleration = 0f;
            _eventMaxChestRelativeRotationDeltaDegrees = 0f;
            _eventMaxResponseRotationErrorDegrees = 0f;
            _eventMaxPoseBridgeRotationErrorDegrees = 0f;
            _eventMaxPoseBridgeCommandAngularSpeed = 0f;
            _eventMaxAnchorErrorMeters = 0f;
            _eventAngularClampFixedStepCount = 0;

            Plugin.Logger?.LogWarning(
                "[ARL][SOURCE_TURN_EVENT] event=" + _eventId +
                " | trigger=" + triggerReason +
                " | sequence=" + triggerSample.Sequence +
                " | dt=" + F(triggerSample.DeltaTime) +
                " | pelvisRotationDeltaDeg=" + F(triggerSample.RotationDeltaDegrees) +
                " | pelvisYawDeg=" + F(triggerSample.YawDegrees) +
                " | pelvisYawDeltaDeg=" + F(triggerSample.YawDeltaDegrees) +
                " | rawAngularVel=" + V(triggerSample.RawAngularVelocity) + "/speed=" + F(triggerSample.RawAngularSpeed) +
                " | rawAngularAcc=" + V(triggerSample.RawAngularAcceleration) + "/mag=" + F(triggerSample.RawAngularAccelerationMagnitude) +
                " | chestRelativeRotationDeltaDeg=" + F(triggerSample.ChestRelativeRotationDeltaDegrees));

            // Dump already-seen samples immediately before the trigger.  Do not include the trigger sample
            // here because it is emitted once below as regular event context.
            for (int i = 0; i < _history.Count; i++)
            {
                Sample previous = _history[i];
                if (previous.Sequence == triggerSample.Sequence || previous.Time < _eventContextStartTime)
                    continue;
                AccumulateEventSample(previous);
                if (verboseContextLogging)
                    Plugin.Logger?.LogInfo(FormatContextLine(previous, _eventId, previous.Time - _eventTriggerTime, true, _eventSignedYawDeltaDegrees));
            }
        }

        private void AccumulateEventSample(Sample sample)
        {
            _eventSignedYawDeltaDegrees += sample.YawDeltaDegrees;
            _eventAbsoluteYawDeltaDegrees += Mathf.Abs(sample.YawDeltaDegrees);
            _eventMaxRotationDeltaDegrees = Mathf.Max(_eventMaxRotationDeltaDegrees, sample.RotationDeltaDegrees);
            _eventMaxAbsYawDeltaDegrees = Mathf.Max(_eventMaxAbsYawDeltaDegrees, Mathf.Abs(sample.YawDeltaDegrees));
            _eventMaxRawAngularSpeed = Mathf.Max(_eventMaxRawAngularSpeed, sample.RawAngularSpeed);
            _eventMaxRawAngularAcceleration = Mathf.Max(_eventMaxRawAngularAcceleration, sample.RawAngularAccelerationMagnitude);
            _eventMaxChestRelativeRotationDeltaDegrees = Mathf.Max(_eventMaxChestRelativeRotationDeltaDegrees, sample.ChestRelativeRotationDeltaDegrees);
        }

        private void FinalizeEvent(string closeReason, float closeTime)
        {
            if (!_eventActive)
                return;

            string classification = _eventSawSingleSampleDiscontinuity
                ? "SINGLE_SAMPLE_DISCONTINUITY"
                : "RAPID_CONTINUOUS_TURN";
            if (_eventSawSingleSampleDiscontinuity)
                DiscontinuityEventCount++;
            else
                RapidContinuousEventCount++;

            LastEventClassification = classification;
            LastEventSignedYawDeltaDegrees = _eventSignedYawDeltaDegrees;
            LastEventAbsoluteYawDeltaDegrees = _eventAbsoluteYawDeltaDegrees;
            LastEventMaxSingleSampleRotationDeltaDegrees = _eventMaxRotationDeltaDegrees;
            LastEventMaxRawAngularSpeed = _eventMaxRawAngularSpeed;
            LastEventMaxResponseRotationErrorDegrees = _eventMaxResponseRotationErrorDegrees;

            MaxSingleSampleRotationDeltaDegrees = Mathf.Max(MaxSingleSampleRotationDeltaDegrees, _eventMaxRotationDeltaDegrees);
            MaxSingleSampleAbsYawDeltaDegrees = Mathf.Max(MaxSingleSampleAbsYawDeltaDegrees, _eventMaxAbsYawDeltaDegrees);
            MaxRawAngularSpeed = Mathf.Max(MaxRawAngularSpeed, _eventMaxRawAngularSpeed);
            MaxRawAngularAcceleration = Mathf.Max(MaxRawAngularAcceleration, _eventMaxRawAngularAcceleration);
            MaxChestRelativeRotationDeltaDegrees = Mathf.Max(MaxChestRelativeRotationDeltaDegrees, _eventMaxChestRelativeRotationDeltaDegrees);
            MaxResponseRotationErrorDegrees = Mathf.Max(MaxResponseRotationErrorDegrees, _eventMaxResponseRotationErrorDegrees);
            MaxPoseBridgeRotationErrorDegrees = Mathf.Max(MaxPoseBridgeRotationErrorDegrees, _eventMaxPoseBridgeRotationErrorDegrees);
            MaxPoseBridgeCommandAngularSpeed = Mathf.Max(MaxPoseBridgeCommandAngularSpeed, _eventMaxPoseBridgeCommandAngularSpeed);
            MaxAnchorErrorMeters = Mathf.Max(MaxAnchorErrorMeters, _eventMaxAnchorErrorMeters);

            Plugin.Logger?.LogWarning(
                "[ARL][SOURCE_TURN_SUMMARY] event=" + _eventId +
                " | classification=" + classification +
                " | trigger=" + _eventTriggerReason +
                " | close=" + closeReason +
                " | durationFromTrigger=" + F(Mathf.Max(0f, closeTime - _eventTriggerTime)) +
                " | contextSignedYawDeltaDeg=" + F(_eventSignedYawDeltaDegrees) +
                " | contextAbsYawDeltaDeg=" + F(_eventAbsoluteYawDeltaDegrees) +
                " | maxSampleRotationDeltaDeg=" + F(_eventMaxRotationDeltaDegrees) +
                " | maxSampleAbsYawDeltaDeg=" + F(_eventMaxAbsYawDeltaDegrees) +
                " | maxRawAngularSpeed=" + F(_eventMaxRawAngularSpeed) +
                " | maxRawAngularAcceleration=" + F(_eventMaxRawAngularAcceleration) +
                " | maxChestRelativeRotationDeltaDeg=" + F(_eventMaxChestRelativeRotationDeltaDegrees) +
                " | responseMaxRotationErrorDeg=" + F(_eventMaxResponseRotationErrorDegrees) +
                " | responseMaxPoseBridgeRotationErrorDeg=" + F(_eventMaxPoseBridgeRotationErrorDegrees) +
                " | responseMaxPoseBridgeCmdAngularSpeed=" + F(_eventMaxPoseBridgeCommandAngularSpeed) +
                " | responseMaxAnchorError=" + F(_eventMaxAnchorErrorMeters) +
                " | responseAngularClampFixedSteps=" + _eventAngularClampFixedStepCount);

            _eventActive = false;
            _eventTriggerReason = "<none>";
        }

        private void ResetSessionStatistics()
        {
            _eventActive = false;
            _eventId = 0;
            _eventTriggerTime = 0f;
            _eventContextStartTime = 0f;
            _eventQuietAfterTime = 0f;
            _eventTriggerReason = "<none>";
            _eventSawSingleSampleDiscontinuity = false;
            EventCount = 0;
            DiscontinuityEventCount = 0;
            RapidContinuousEventCount = 0;
            MaxSingleSampleRotationDeltaDegrees = 0f;
            MaxSingleSampleAbsYawDeltaDegrees = 0f;
            MaxRawAngularSpeed = 0f;
            MaxRawAngularAcceleration = 0f;
            MaxChestRelativeRotationDeltaDegrees = 0f;
            MaxResponseRotationErrorDegrees = 0f;
            MaxPoseBridgeRotationErrorDegrees = 0f;
            MaxPoseBridgeCommandAngularSpeed = 0f;
            MaxAnchorErrorMeters = 0f;
            AngularClampFixedStepCount = 0;
            LastEventClassification = "<none>";
            LastEventSignedYawDeltaDegrees = 0f;
            LastEventAbsoluteYawDeltaDegrees = 0f;
            LastEventMaxSingleSampleRotationDeltaDegrees = 0f;
            LastEventMaxRawAngularSpeed = 0f;
            LastEventMaxResponseRotationErrorDegrees = 0f;
        }

        private void ClearCurrentSample()
        {
            CurrentSampleDeltaSeconds = 0f;
            CurrentRotationDeltaDegrees = 0f;
            CurrentYawDegrees = 0f;
            CurrentYawDeltaDegrees = 0f;
            CurrentRawAngularVelocity = Vector3.zero;
            CurrentRawAngularSpeed = 0f;
            CurrentRawAngularAcceleration = Vector3.zero;
            CurrentRawAngularAccelerationMagnitude = 0f;
            CurrentChestRelativeRotationDeltaDegrees = 0f;
        }

        private static string BuildTriggerReason(bool rotation, bool speed, bool acceleration)
        {
            string result = string.Empty;
            if (rotation) result = "ROTATION_DELTA";
            if (speed) result += (result.Length == 0 ? string.Empty : "+") + "ANGULAR_SPEED";
            if (acceleration) result += (result.Length == 0 ? string.Empty : "+") + "ANGULAR_ACCEL";
            return result.Length == 0 ? "<none>" : result;
        }

        private static string FormatContextLine(Sample sample, int eventId, float offsetSeconds, bool preTrigger, float cumulativeSignedYawDegrees)
        {
            return
                "[ARL][SOURCE_TURN_CONTEXT] event=" + eventId +
                " | offset=" + (offsetSeconds >= 0f ? "+" : string.Empty) + F(offsetSeconds) +
                " | pre=" + preTrigger +
                " | sequence=" + sample.Sequence +
                " | dt=" + F(sample.DeltaTime) +
                " | pelvisRotationDeltaDeg=" + F(sample.RotationDeltaDegrees) +
                " | pelvisYawDeg=" + F(sample.YawDegrees) +
                " | pelvisYawDeltaDeg=" + F(sample.YawDeltaDegrees) +
                " | contextCumulativeYawDeg=" + F(cumulativeSignedYawDegrees) +
                " | rawAngularVel=" + V(sample.RawAngularVelocity) + "/speed=" + F(sample.RawAngularSpeed) +
                " | rawAngularAcc=" + V(sample.RawAngularAcceleration) + "/mag=" + F(sample.RawAngularAccelerationMagnitude) +
                " | chestRelativeRotationDeltaDeg=" + F(sample.ChestRelativeRotationDeltaDegrees);
        }

        private static float ComputeYawDegrees(Quaternion rotation)
        {
            Vector3 forward = rotation * Vector3.forward;
            forward.y = 0f;
            if (forward.sqrMagnitude <= 0.000001f)
                return 0f;
            forward.Normalize();
            return Mathf.Atan2(forward.x, forward.z) * Mathf.Rad2Deg;
        }

        private static float ComputeSignedYawDelta(Quaternion previous, Quaternion current)
        {
            Vector3 previousForward = previous * Vector3.forward;
            Vector3 currentForward = current * Vector3.forward;
            previousForward.y = 0f;
            currentForward.y = 0f;
            if (previousForward.sqrMagnitude <= 0.000001f || currentForward.sqrMagnitude <= 0.000001f)
                return 0f;
            previousForward.Normalize();
            currentForward.Normalize();
            return Vector3.SignedAngle(previousForward, currentForward, Vector3.up);
        }

        private static bool IsFinite(float value) => !(float.IsNaN(value) || float.IsInfinity(value));

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        }

        private static string F(float value) => value.ToString("G7", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
