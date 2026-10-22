using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.3.78 frozen production common-mode angular transport for the Dynamic Foot.
    ///
    /// Historical context retained only as maintenance guardrails:
    /// - v0.3.75 proved that removing Foot angularDrag improves local Foot response but is system-level unsafe because
    ///   the Left Knee translation tether reached the 30 mm hard gate. The drag-zero route is permanently retired.
    /// - v0.3.76 introduced a bounded common-mode drag compensation while keeping Foot angularDrag at the profile value.
    /// - v0.3.77 confirmed the exact same law under stronger Lower angular-speed coverage with no Foot/Knee failure
    ///   migration, authorizing promotion.
    ///
    /// Production law:
    ///   measuredAlpha = (LowerW[n] - LowerW[n-1]) / fixedDeltaTime
    ///   dragCompRaw = FootAngularDrag * LowerWorldAngularVelocity
    ///   dragCompApplied = clampMagnitude(dragCompRaw, 20 rad/s^2)
    ///   requestedAlpha = clampMagnitude(measuredAlpha + dragCompApplied, 60 rad/s^2)
    ///   requestedTorque = I_world * requestedAlpha + gyro
    ///   appliedTorque = clampMagnitude(requestedTorque, 0.35 Nm)
    ///
    /// The compensation uses only the parent's world angular velocity. It intentionally does not cancel
    /// Foot-Lower relative angular velocity, so the real Rigidbody angularDrag continues to damp distal/relative motion.
    /// There is still exactly one Foot-only common-mode AddTorque and no direct Lower reaction write.
    /// </summary>
    internal sealed class FootAnkleAngularTransport
    {
        internal const string Mode = "ANKLE_COMMON_MODE_ANGULAR_DRAG_COMPENSATED_TRANSPORT_PRODUCTION_V0378";
        internal const string Production = "PRODUCTION_V0378_BOUNDED_COMMON_MODE_DRAG_COMPENSATION";

        internal const float MaxCommonModeAngularAcceleration = 60f;
        internal const float MaxCommonModeTorqueNm = 0.35f;
        internal const float MaxCommonModeDragCompensationAcceleration = 20f;
        private const float InertiaEpsilon = 0.000001f;

        private bool _finite = true;
        private string _invalidReason;
        private bool _hasPreviousLowerAngularVelocity;
        private Vector3 _previousLowerAngularVelocityWorld;

        private bool _currentActive;
        private bool _currentWarmup;
        private Vector3 _currentLowerAngularVelocityWorld;
        private Vector3 _currentMeasuredLowerAngularAccelerationWorld;
        private Vector3 _currentDragCompensationRawAccelerationWorld;
        private Vector3 _currentDragCompensationAppliedAccelerationWorld;
        private Vector3 _currentRequestedAngularAccelerationWorld;
        private Vector3 _currentAppliedAngularAccelerationWorld;
        private Vector3 _currentTorqueWorld;
        private bool _currentCompensationClamp;
        private bool _currentAccelerationClamp;
        private bool _currentTorqueClamp;
        private float _currentFootAngularDrag;

        private int _traceSamples;
        private int _traceActiveSamples;
        private int _traceWarmupSamples;
        private int _traceInvalidSamples;
        private int _traceCompensationClampSamples;
        private int _traceAccelerationClampSamples;
        private int _traceTorqueClampSamples;
        private float _traceLowerAngularSpeedSum;
        private float _traceLowerAngularSpeedMax;
        private float _traceMeasuredAccelerationSum;
        private float _traceMeasuredAccelerationMax;
        private float _traceCompensationRawAccelerationSum;
        private float _traceCompensationRawAccelerationMax;
        private float _traceCompensationAppliedAccelerationSum;
        private float _traceCompensationAppliedAccelerationMax;
        private float _traceRequestedAccelerationSum;
        private float _traceRequestedAccelerationMax;
        private float _traceAppliedAccelerationSum;
        private float _traceAppliedAccelerationMax;
        private float _traceTorqueSum;
        private float _traceTorqueMax;

        internal bool Enabled => true;
        internal string Policy => Production;
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;

        // v0.4.4.1 read-only diagnostic taps for contact-induced axial-debt attribution.
        internal bool CurrentActive => _currentActive;
        internal bool CurrentWarmup => _currentWarmup;
        internal bool CurrentAccelerationClamp => _currentAccelerationClamp;
        internal bool CurrentTorqueClamp => _currentTorqueClamp;
        internal Vector3 CurrentLowerAngularVelocityWorld => _currentLowerAngularVelocityWorld;
        internal Vector3 CurrentMeasuredLowerAngularAccelerationWorld => _currentMeasuredLowerAngularAccelerationWorld;
        internal Vector3 CurrentRequestedAngularAccelerationWorld => _currentRequestedAngularAccelerationWorld;
        internal Vector3 CurrentAppliedAngularAccelerationWorld => _currentAppliedAngularAccelerationWorld;
        internal Vector3 CurrentTorqueWorld => _currentTorqueWorld;

        internal bool Step(
            bool traceEnabled,
            bool dynamicActive,
            Rigidbody footBody,
            Rigidbody lowerBody,
            float fixedDeltaTime,
            FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _currentActive = false;
            _currentWarmup = false;
            _currentLowerAngularVelocityWorld = Vector3.zero;
            _currentMeasuredLowerAngularAccelerationWorld = Vector3.zero;
            _currentDragCompensationRawAccelerationWorld = Vector3.zero;
            _currentDragCompensationAppliedAccelerationWorld = Vector3.zero;
            _currentRequestedAngularAccelerationWorld = Vector3.zero;
            _currentAppliedAngularAccelerationWorld = Vector3.zero;
            _currentTorqueWorld = Vector3.zero;
            _currentCompensationClamp = false;
            _currentAccelerationClamp = false;
            _currentTorqueClamp = false;
            _currentFootAngularDrag = footBody != null ? footBody.angularDrag : 0f;

            if (!dynamicActive)
            {
                _hasPreviousLowerAngularVelocity = false;
                _previousLowerAngularVelocityWorld = Vector3.zero;
                CopyToReport(report);
                return true;
            }

            if (footBody == null || lowerBody == null || !FiniteFloat(fixedDeltaTime) || fixedDeltaTime <= 0.0001f ||
                footBody.isKinematic || footBody.useGravity || !ValidInertiaBody(footBody) ||
                !FiniteVector(lowerBody.angularVelocity) || !FiniteFloat(footBody.angularDrag) || footBody.angularDrag < 0f)
                return Invalidate("INVALID_ANKLE_ANGULAR_TRANSPORT_INPUT", traceEnabled, report);

            Vector3 lowerW = lowerBody.angularVelocity;
            _currentLowerAngularVelocityWorld = lowerW;

            if (!_hasPreviousLowerAngularVelocity)
            {
                _previousLowerAngularVelocityWorld = lowerW;
                _hasPreviousLowerAngularVelocity = true;
                _currentWarmup = true;
                AccumulateTrace(traceEnabled, false, true);
                CopyToReport(report);
                return true;
            }

            Vector3 measuredAlpha = (lowerW - _previousLowerAngularVelocityWorld) / fixedDeltaTime;
            _previousLowerAngularVelocityWorld = lowerW;
            if (!FiniteVector(measuredAlpha))
                return Invalidate("NON_FINITE_ANKLE_LOWER_ANGULAR_ACCELERATION", traceEnabled, report);

            // Production compensation: restore only the parent's/common world rotation lost to the child's real drag.
            // Never use Foot-Lower relative W here; relative/distal motion remains physically damped by Rigidbody drag.
            Vector3 compensationRaw = lowerW * footBody.angularDrag;
            if (!FiniteVector(compensationRaw))
                return Invalidate("NON_FINITE_ANKLE_COMMON_MODE_DRAG_COMPENSATION", traceEnabled, report);
            bool compensationClamp = compensationRaw.sqrMagnitude >
                                     MaxCommonModeDragCompensationAcceleration * MaxCommonModeDragCompensationAcceleration + 0.000001f;
            Vector3 compensationApplied = Vector3.ClampMagnitude(compensationRaw, MaxCommonModeDragCompensationAcceleration);

            Vector3 combinedRawAlpha = measuredAlpha + compensationApplied;
            if (!FiniteVector(combinedRawAlpha))
                return Invalidate("NON_FINITE_ANKLE_COMMON_MODE_COMBINED_ACCELERATION", traceEnabled, report);
            Vector3 requestedAlpha = Vector3.ClampMagnitude(combinedRawAlpha, MaxCommonModeAngularAcceleration);
            bool accelClamp = combinedRawAlpha.sqrMagnitude >
                              MaxCommonModeAngularAcceleration * MaxCommonModeAngularAcceleration + 0.000001f;

            Quaternion principalWorld = Normalize(footBody.rotation * footBody.inertiaTensorRotation);
            Vector3 gyro = GyroTorque(principalWorld, footBody.inertiaTensor, footBody.angularVelocity);
            Vector3 requestedTorque = MultiplyWorldInertia(principalWorld, footBody.inertiaTensor, requestedAlpha) + gyro;
            if (!FiniteVector(gyro) || !FiniteVector(requestedTorque))
                return Invalidate("NON_FINITE_ANKLE_ANGULAR_TRANSPORT_TORQUE", traceEnabled, report);

            bool torqueClamp = requestedTorque.sqrMagnitude > MaxCommonModeTorqueNm * MaxCommonModeTorqueNm + 0.000001f;
            Vector3 appliedTorque = Vector3.ClampMagnitude(requestedTorque, MaxCommonModeTorqueNm);
            Vector3 appliedAlpha = InverseWorldInertia(principalWorld, footBody.inertiaTensor, appliedTorque - gyro);
            if (!FiniteVector(appliedTorque) || !FiniteVector(appliedAlpha))
                return Invalidate("NON_FINITE_ANKLE_ANGULAR_TRANSPORT_REALIZATION", traceEnabled, report);

            // Exactly one child-only common-mode actuator. No additional torque lane and no direct Lower reaction.
            footBody.AddTorque(appliedTorque, ForceMode.Force);

            _currentActive = true;
            _currentMeasuredLowerAngularAccelerationWorld = measuredAlpha;
            _currentDragCompensationRawAccelerationWorld = compensationRaw;
            _currentDragCompensationAppliedAccelerationWorld = compensationApplied;
            _currentRequestedAngularAccelerationWorld = requestedAlpha;
            _currentAppliedAngularAccelerationWorld = appliedAlpha;
            _currentTorqueWorld = appliedTorque;
            _currentCompensationClamp = compensationClamp;
            _currentAccelerationClamp = accelClamp;
            _currentTorqueClamp = torqueClamp;

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
            _traceCompensationClampSamples = 0;
            _traceAccelerationClampSamples = 0;
            _traceTorqueClampSamples = 0;
            _traceLowerAngularSpeedSum = 0f;
            _traceLowerAngularSpeedMax = 0f;
            _traceMeasuredAccelerationSum = 0f;
            _traceMeasuredAccelerationMax = 0f;
            _traceCompensationRawAccelerationSum = 0f;
            _traceCompensationRawAccelerationMax = 0f;
            _traceCompensationAppliedAccelerationSum = 0f;
            _traceCompensationAppliedAccelerationMax = 0f;
            _traceRequestedAccelerationSum = 0f;
            _traceRequestedAccelerationMax = 0f;
            _traceAppliedAccelerationSum = 0f;
            _traceAppliedAccelerationMax = 0f;
            _traceTorqueSum = 0f;
            _traceTorqueMax = 0f;
            // Statistics reset must not perturb the physical derivative estimator.
            _finite = true;
            _invalidReason = null;
            CopyToReport(report);
        }

        internal void ClearCurrentState(FootAnkleFoundationReport report)
        {
            ResetCurrent();
            CopyToReport(report);
        }

        internal void RefreshFootAngularDragReadback(float footAngularDrag, FootAnkleFoundationReport report)
        {
            _currentFootAngularDrag = FiniteFloat(footAngularDrag) ? footAngularDrag : 0f;
            CopyToReport(report);
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/policy=" + Production +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.AnkleAngularTransportCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.AnkleAngularTransportTraceSummary : "<none>");
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            report.AnkleAngularTransportMode = Mode;
            report.AnkleAngularTransportPolicy = Production;
            report.AnkleAngularTransportFinite = _finite;
            report.AnkleAngularTransportInvalidReason = _invalidReason;
            report.AnkleAngularTransportCurrentSummary =
                "policy=" + Production +
                "/active=" + _currentActive +
                "/warmup=" + _currentWarmup +
                "/lowerW=" + V(_currentLowerAngularVelocityWorld) +
                "/measuredA=" + V(_currentMeasuredLowerAngularAccelerationWorld) +
                "/dragCompRaw=" + V(_currentDragCompensationRawAccelerationWorld) +
                "/dragCompApplied=" + V(_currentDragCompensationAppliedAccelerationWorld) +
                "/requestedA=" + V(_currentRequestedAngularAccelerationWorld) +
                "/appliedA=" + V(_currentAppliedAngularAccelerationWorld) +
                "/tau=" + V(_currentTorqueWorld) +
                "/footAngularDrag=" + F(_currentFootAngularDrag) +
                "/clampComp-A-T=" + _currentCompensationClamp + "-" + _currentAccelerationClamp + "-" + _currentTorqueClamp;

            float denom = Mathf.Max(1, _traceSamples);
            float activeDenom = Mathf.Max(1, _traceActiveSamples);
            report.AnkleAngularTransportTraceSummary =
                "n=" + _traceSamples +
                "/active=" + _traceActiveSamples +
                "/warmup=" + _traceWarmupSamples +
                "/invalid=" + _traceInvalidSamples +
                "/lowerWMeanMax=" + F(_traceLowerAngularSpeedSum / denom) + "-" + F(_traceLowerAngularSpeedMax) +
                "/measuredAMeanMax=" + F(_traceMeasuredAccelerationSum / activeDenom) + "-" + F(_traceMeasuredAccelerationMax) +
                "/dragCompRawMeanMax=" + F(_traceCompensationRawAccelerationSum / activeDenom) + "-" + F(_traceCompensationRawAccelerationMax) +
                "/dragCompAppliedMeanMax=" + F(_traceCompensationAppliedAccelerationSum / activeDenom) + "-" + F(_traceCompensationAppliedAccelerationMax) +
                "/requestedAMeanMax=" + F(_traceRequestedAccelerationSum / activeDenom) + "-" + F(_traceRequestedAccelerationMax) +
                "/appliedAMeanMax=" + F(_traceAppliedAccelerationSum / activeDenom) + "-" + F(_traceAppliedAccelerationMax) +
                "/tauMeanMax=" + F(_traceTorqueSum / activeDenom) + "-" + F(_traceTorqueMax) +
                "/clampComp-A-T=" + _traceCompensationClampSamples + "-" + _traceAccelerationClampSamples + "-" + _traceTorqueClampSamples +
                "/source=ACTUAL_LOWER_FIXED_STEP_ANGULAR_ACCELERATION_PLUS_PRODUCTION_COMMON_MODE_DRAG_COMPENSATION" +
                "/policy=" + Production +
                "/bounds(CompA-CommonA-MaxTau)=" + F(MaxCommonModeDragCompensationAcceleration) + "-" +
                    F(MaxCommonModeAngularAcceleration) + "-" + F(MaxCommonModeTorqueNm) +
                "/topology=ONE_FOOT_ONLY_COMMON_MODE_ADDTORQUE_NO_DIRECT_LOWER_REACTION";
        }

        private void ResetCurrent()
        {
            _finite = true;
            _invalidReason = null;
            _hasPreviousLowerAngularVelocity = false;
            _previousLowerAngularVelocityWorld = Vector3.zero;
            _currentActive = false;
            _currentWarmup = false;
            _currentLowerAngularVelocityWorld = Vector3.zero;
            _currentMeasuredLowerAngularAccelerationWorld = Vector3.zero;
            _currentDragCompensationRawAccelerationWorld = Vector3.zero;
            _currentDragCompensationAppliedAccelerationWorld = Vector3.zero;
            _currentRequestedAngularAccelerationWorld = Vector3.zero;
            _currentAppliedAngularAccelerationWorld = Vector3.zero;
            _currentTorqueWorld = Vector3.zero;
            _currentCompensationClamp = false;
            _currentAccelerationClamp = false;
            _currentTorqueClamp = false;
            // Keep the last real Rigidbody drag readback across diagnostic/current-state clears. v0.3.78 removed
            // the historical A/B restore setter, so zeroing this field here would make post-stop status falsely
            // report footAngularDrag=0 even though the Rigidbody remains profile-frozen.
        }

        private void AccumulateTrace(bool traceEnabled, bool active, bool warmup)
        {
            if (!traceEnabled) return;
            _traceSamples++;
            if (active) _traceActiveSamples++;
            if (warmup) _traceWarmupSamples++;
            float lowerW = _currentLowerAngularVelocityWorld.magnitude;
            _traceLowerAngularSpeedSum += lowerW;
            _traceLowerAngularSpeedMax = Mathf.Max(_traceLowerAngularSpeedMax, lowerW);
            if (!active) return;

            float measuredA = _currentMeasuredLowerAngularAccelerationWorld.magnitude;
            float compRaw = _currentDragCompensationRawAccelerationWorld.magnitude;
            float compApplied = _currentDragCompensationAppliedAccelerationWorld.magnitude;
            float requestedA = _currentRequestedAngularAccelerationWorld.magnitude;
            float appliedA = _currentAppliedAngularAccelerationWorld.magnitude;
            float tau = _currentTorqueWorld.magnitude;
            _traceMeasuredAccelerationSum += measuredA;
            _traceMeasuredAccelerationMax = Mathf.Max(_traceMeasuredAccelerationMax, measuredA);
            _traceCompensationRawAccelerationSum += compRaw;
            _traceCompensationRawAccelerationMax = Mathf.Max(_traceCompensationRawAccelerationMax, compRaw);
            _traceCompensationAppliedAccelerationSum += compApplied;
            _traceCompensationAppliedAccelerationMax = Mathf.Max(_traceCompensationAppliedAccelerationMax, compApplied);
            _traceRequestedAccelerationSum += requestedA;
            _traceRequestedAccelerationMax = Mathf.Max(_traceRequestedAccelerationMax, requestedA);
            _traceAppliedAccelerationSum += appliedA;
            _traceAppliedAccelerationMax = Mathf.Max(_traceAppliedAccelerationMax, appliedA);
            _traceTorqueSum += tau;
            _traceTorqueMax = Mathf.Max(_traceTorqueMax, tau);
            if (_currentCompensationClamp) _traceCompensationClampSamples++;
            if (_currentAccelerationClamp) _traceAccelerationClampSamples++;
            if (_currentTorqueClamp) _traceTorqueClampSamples++;
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

        private static bool ValidInertiaBody(Rigidbody body)
        {
            return body != null && FiniteQuaternion(body.rotation) && FiniteVector(body.angularVelocity) &&
                   FiniteVector(body.inertiaTensor) && FiniteQuaternion(body.inertiaTensorRotation) &&
                   body.inertiaTensor.x > InertiaEpsilon && body.inertiaTensor.y > InertiaEpsilon && body.inertiaTensor.z > InertiaEpsilon;
        }

        private static Vector3 GyroTorque(Quaternion principalWorld, Vector3 inertia, Vector3 angularVelocityWorld)
        {
            Vector3 angularMomentum = MultiplyWorldInertia(principalWorld, inertia, angularVelocityWorld);
            return Vector3.Cross(angularVelocityWorld, angularMomentum);
        }

        private static Vector3 MultiplyWorldInertia(Quaternion principalWorld, Vector3 inertia, Vector3 vectorWorld)
        {
            Vector3 local = Quaternion.Inverse(principalWorld) * vectorWorld;
            Vector3 mapped = new Vector3(local.x * inertia.x, local.y * inertia.y, local.z * inertia.z);
            return principalWorld * mapped;
        }

        private static Vector3 InverseWorldInertia(Quaternion principalWorld, Vector3 inertia, Vector3 torqueWorld)
        {
            Vector3 local = Quaternion.Inverse(principalWorld) * torqueWorld;
            Vector3 mapped = new Vector3(local.x / inertia.x, local.y / inertia.y, local.z / inertia.z);
            return principalWorld * mapped;
        }

        private static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (mag <= 0.000001f || !FiniteFloat(mag)) return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        private static bool FiniteFloat(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool FiniteVector(Vector3 value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z);
        private static bool FiniteQuaternion(Quaternion value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z) && FiniteFloat(value.w);
        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
