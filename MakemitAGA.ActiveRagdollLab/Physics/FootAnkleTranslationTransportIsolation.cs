using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.3.64 measured Lower ankle-point transport, promoted into the frozen free-space Foot baseline since v0.3.69.
    ///
    /// v0.3.63 proved that removing the Foot CharacterJoint is structurally viable at idle, but its bounded
    /// position/velocity tether crossed the unchanged 30 mm ankle gate at the first material source/parent transport
    /// event.  The tether never reached its 60 m/s^2 total clamp before that abort, which points to a causal timing
    /// gap rather than simple authority saturation: the child-only PD must wait for separation/velocity error before
    /// it can react to a moving Lower ankle endpoint.
    ///
    /// A preserves the exact v0.3.63 PD-only tether.  B measures the ACTUAL physical Lower ankle-point acceleration
    /// from consecutive FixedUpdate point velocities and adds it as common-mode transport feedforward before the same
    /// unchanged Kp/Kd correction and the same unchanged final 60 m/s^2 clamp.  No source future pose, Render-derived
    /// acceleration, new torque, AddForceAtPosition, equal/opposite Lower force, or authority increase is introduced.
    /// </summary>
    internal sealed class FootAnkleTranslationTransportIsolation
    {
        internal const string Mode = "ANKLE_TRANSLATION_TETHER_PARENT_ACCELERATION_FEEDFORWARD_CAUSAL_AB_V0364";
        internal const string ArmA = "A_V0363_PD_ONLY_TRANSLATION_TETHER";
        internal const string ArmB = "B_V0364_MEASURED_LOWER_ANKLE_ACCELERATION_FF";

        private bool _enabled;
        private bool _finite = true;
        private string _invalidReason;
        private bool _hasPreviousParentAnchorVelocity;
        private Vector3 _previousParentAnchorVelocity;

        private bool _currentActive;
        private bool _currentWarmup;
        private Vector3 _currentParentAnchorVelocityWorld;
        private Vector3 _currentFeedforwardAccelerationWorld;

        private int _traceSamples;
        private int _traceActiveSamples;
        private int _traceWarmupSamples;
        private int _traceInvalidSamples;
        private float _traceParentSpeedSum;
        private float _traceParentSpeedMax;
        private float _traceFeedforwardSum;
        private float _traceFeedforwardMax;

        internal bool Enabled => _enabled;
        internal string Arm => _enabled ? ArmB : ArmA;
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;
        internal Vector3 CurrentFeedforwardAccelerationWorld => _currentFeedforwardAccelerationWorld;

        internal void FreezeProductionArm()
        {
            _enabled = true;
            ResetCurrent();
        }

        internal bool Step(
            bool traceEnabled,
            bool tetherDynamicActive,
            Rigidbody lowerBody,
            Vector3 lowerAnchorLocal,
            float fixedDeltaTime,
            FootAnkleFoundationReport report,
            out Vector3 feedforwardAccelerationWorld)
        {
            feedforwardAccelerationWorld = Vector3.zero;
            _finite = true;
            _invalidReason = null;
            _currentActive = false;
            _currentWarmup = false;
            _currentParentAnchorVelocityWorld = Vector3.zero;
            _currentFeedforwardAccelerationWorld = Vector3.zero;

            if (!tetherDynamicActive)
            {
                // Do not carry stale finite-difference history across kinematic/off intervals.
                _hasPreviousParentAnchorVelocity = false;
                _previousParentAnchorVelocity = Vector3.zero;
                CopyToReport(report);
                return true;
            }

            if (lowerBody == null || !FiniteVector(lowerAnchorLocal) || !FiniteFloat(fixedDeltaTime) || fixedDeltaTime <= 0.0001f)
                return Invalidate("INVALID_ANKLE_PARENT_TRANSPORT_INPUT", traceEnabled, report);

            Vector3 lowerAnchorWorld = lowerBody.transform.TransformPoint(lowerAnchorLocal);
            Vector3 lowerLever = lowerAnchorWorld - lowerBody.worldCenterOfMass;
            Vector3 lowerAnchorVelocity = lowerBody.velocity + Vector3.Cross(lowerBody.angularVelocity, lowerLever);
            if (!FiniteVector(lowerAnchorWorld) || !FiniteVector(lowerLever) || !FiniteVector(lowerAnchorVelocity))
                return Invalidate("NON_FINITE_ANKLE_PARENT_ANCHOR_VELOCITY", traceEnabled, report);

            _currentParentAnchorVelocityWorld = lowerAnchorVelocity;

            if (!_enabled)
            {
                // A must remain the exact v0.3.63 PD-only tether.  Still seed the physical point velocity so switching
                // to B while drive is OFF starts from fresh history rather than a stale pre-run sample.
                _previousParentAnchorVelocity = lowerAnchorVelocity;
                _hasPreviousParentAnchorVelocity = true;
                AccumulateTrace(traceEnabled, false, false);
                CopyToReport(report);
                return true;
            }

            if (!_hasPreviousParentAnchorVelocity)
            {
                _previousParentAnchorVelocity = lowerAnchorVelocity;
                _hasPreviousParentAnchorVelocity = true;
                _currentWarmup = true;
                AccumulateTrace(traceEnabled, false, true);
                CopyToReport(report);
                return true;
            }

            Vector3 measuredParentAnchorAcceleration =
                (lowerAnchorVelocity - _previousParentAnchorVelocity) / fixedDeltaTime;
            _previousParentAnchorVelocity = lowerAnchorVelocity;
            if (!FiniteVector(measuredParentAnchorAcceleration))
                return Invalidate("NON_FINITE_ANKLE_PARENT_ANCHOR_ACCELERATION", traceEnabled, report);

            _currentActive = true;
            _currentFeedforwardAccelerationWorld = measuredParentAnchorAcceleration;
            feedforwardAccelerationWorld = measuredParentAnchorAcceleration;
            AccumulateTrace(traceEnabled, true, false);
            CopyToReport(report);
            return true;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _traceSamples = 0;
            _traceActiveSamples = 0;
            _traceWarmupSamples = 0;
            _traceInvalidSamples = 0;
            _traceParentSpeedSum = 0f;
            _traceParentSpeedMax = 0f;
            _traceFeedforwardSum = 0f;
            _traceFeedforwardMax = 0f;
            // Statistics reset must not perturb the physical derivative estimator.
            // Preserve previous Lower/anchor velocity history so arl_trace(on) is control-neutral.
            _finite = true;
            _invalidReason = null;
            CopyToReport(report);
        }

        internal void ClearCurrentState(FootAnkleFoundationReport report)
        {
            ResetCurrent();
            CopyToReport(report);
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/arm=" + Arm +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.AnkleTransportCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.AnkleTransportTraceSummary : "<none>");
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            report.AnkleTransportMode = Mode;
            report.AnkleTransportArm = Arm;
            report.AnkleTransportFinite = _finite;
            report.AnkleTransportInvalidReason = _invalidReason;
            report.AnkleTransportCurrentSummary =
                "arm=" + Arm +
                "/active=" + _currentActive +
                "/warmup=" + _currentWarmup +
                "/parentV=" + V(_currentParentAnchorVelocityWorld) +
                "/parentSpeed=" + F(_currentParentAnchorVelocityWorld.magnitude) +
                "/ffA=" + V(_currentFeedforwardAccelerationWorld) +
                "/ffMag=" + F(_currentFeedforwardAccelerationWorld.magnitude);

            float denom = Mathf.Max(1, _traceSamples);
            report.AnkleTransportTraceSummary =
                "n=" + _traceSamples +
                "/active=" + _traceActiveSamples +
                "/warmup=" + _traceWarmupSamples +
                "/invalid=" + _traceInvalidSamples +
                "/parentSpeedMeanMax=" + F(_traceParentSpeedSum / denom) + "-" + F(_traceParentSpeedMax) +
                "/ffAMeanMax=" + F(_traceFeedforwardSum / denom) + "-" + F(_traceFeedforwardMax) +
                "/source=ACTUAL_LOWER_ANKLE_POINT_FIXED_STEP_ACCELERATION" +
                "/finalAuthority=UNCHANGED_TETHER_MAXA60";
        }

        private void ResetCurrent()
        {
            _finite = true;
            _invalidReason = null;
            _hasPreviousParentAnchorVelocity = false;
            _previousParentAnchorVelocity = Vector3.zero;
            _currentActive = false;
            _currentWarmup = false;
            _currentParentAnchorVelocityWorld = Vector3.zero;
            _currentFeedforwardAccelerationWorld = Vector3.zero;
        }

        private void AccumulateTrace(bool traceEnabled, bool active, bool warmup)
        {
            if (!traceEnabled) return;
            _traceSamples++;
            if (active) _traceActiveSamples++;
            if (warmup) _traceWarmupSamples++;
            float parentSpeed = _currentParentAnchorVelocityWorld.magnitude;
            float ffMag = _currentFeedforwardAccelerationWorld.magnitude;
            _traceParentSpeedSum += parentSpeed;
            _traceParentSpeedMax = Mathf.Max(_traceParentSpeedMax, parentSpeed);
            _traceFeedforwardSum += ffMag;
            _traceFeedforwardMax = Mathf.Max(_traceFeedforwardMax, ffMag);
        }

        private bool Invalidate(string reason, bool traceEnabled, FootAnkleFoundationReport report)
        {
            _finite = false;
            _invalidReason = reason;
            if (traceEnabled)
            {
                _traceSamples++;
                _traceInvalidSamples++;
            }
            CopyToReport(report);
            return false;
        }

        private static bool FiniteFloat(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool FiniteVector(Vector3 value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z);
        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
