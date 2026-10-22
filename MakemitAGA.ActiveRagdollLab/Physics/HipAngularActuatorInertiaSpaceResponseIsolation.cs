using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.81 Hip angular actuator / inertia-space response-closure causal A/B.
    ///
    /// A preserves the historical actuator exactly: the already-final Hip world angular-acceleration command is sent to
    /// Rigidbody.AddTorque(command, ForceMode.Acceleration).
    ///
    /// B changes ONE backend variable only. The SAME desired world angular acceleration is converted into a physical
    /// world torque using the validated runtime inertia tensor and the rigid-body gyroscopic term:
    ///   tau = I_world * alpha_desired + omega x (I_world * omega)
    /// and is then sent through AddTorque(tau, ForceMode.Force).
    ///
    /// Existing Rigidbody.angularDrag is deliberately NOT compensated here. It remains identical in A/B so this test
    /// isolates acceleration-command -> inertia/gyro torque mapping rather than combining it with a damping experiment.
    /// This module contains no Rigidbody actuator call itself; SingleLegShadowChain owns the one actual AddTorque call.
    /// </summary>
    internal sealed class HipAngularActuatorInertiaSpaceResponseIsolation
    {
        internal const string Mode = "HIP_ANGULAR_ACTUATOR_INERTIA_SPACE_RESPONSE_CLOSURE_CAUSAL_AB";
        internal const string ArmA = "A_V080_FORCEMODE_ACCELERATION";
        internal const string ArmB = "B_V081_INERTIA_GYRO_TORQUE_FORCEMODE_FORCE";

        private const float InertiaEpsilon = 1e-6f;

        private bool _enabled;
        private int _steps;
        private int _activeSteps;
        private int _invalidSteps;
        private float _currentDesiredA;
        private float _currentAngularSpeed;
        private float _currentInertiaAlphaTorque;
        private float _currentGyroTorque;
        private float _currentTotalTorque;
        private float _currentPredictedA;
        private float _currentPredictionError;
        private float _currentPredictionAlignment;
        private float _currentGyroToInertiaRatio;
        private Vector3 _currentPhysicalTorqueWorld;
        private float _peakDesiredA;
        private float _peakAngularSpeed;
        private float _peakInertiaAlphaTorque;
        private float _peakGyroTorque;
        private float _peakTotalTorque;
        private float _peakPredictionError;
        private float _peakGyroToInertiaRatio;
        private string _lastEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal string Arm => _enabled ? ArmB : ArmA;
        internal bool CurrentValid { get; private set; } = true;
        // v0.3.3 read-only tap: exact physical torque already composed for the frozen v0.2.81 B actuator.

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
            _currentDesiredA = 0f;
            _currentAngularSpeed = 0f;
            _currentInertiaAlphaTorque = 0f;
            _currentGyroTorque = 0f;
            _currentTotalTorque = 0f;
            _currentPredictedA = 0f;
            _currentPredictionError = 0f;
            _currentPredictionAlignment = 0f;
            _currentGyroToInertiaRatio = 0f;
            _currentPhysicalTorqueWorld = Vector3.zero;
            _peakDesiredA = 0f;
            _peakAngularSpeed = 0f;
            _peakInertiaAlphaTorque = 0f;
            _peakGyroTorque = 0f;
            _peakTotalTorque = 0f;
            _peakPredictionError = 0f;
            _peakGyroToInertiaRatio = 0f;
            _lastEvidence = "<none>";
            CurrentValid = true;
            CopyToReport(report, null);
        }

        /// <summary>
        /// Compose the v0.2.81 B physical torque. In A this method still records diagnostics but returns the desired
        /// acceleration vector unchanged as the diagnostic output; the caller keeps ForceMode.Acceleration exactly.
        /// </summary>
        internal Vector3 Compose(
            Rigidbody body,
            Vector3 desiredAccelerationWorld,
            SingleLegChainReport report)
        {
            _steps++;
            CurrentValid = true;
            _currentDesiredA = desiredAccelerationWorld.magnitude;
            _currentAngularSpeed = body != null && IsFinite(body.angularVelocity) ? body.angularVelocity.magnitude : 0f;
            _currentInertiaAlphaTorque = 0f;
            _currentGyroTorque = 0f;
            _currentTotalTorque = 0f;
            _currentPredictedA = 0f;
            _currentPredictionError = 0f;
            _currentPredictionAlignment = 0f;
            _currentGyroToInertiaRatio = 0f;
            _currentPhysicalTorqueWorld = Vector3.zero;

            if (body == null || !IsFinite(desiredAccelerationWorld) ||
                !IsFinite(body.rotation) || !IsFinite(body.angularVelocity) ||
                !IsFinite(body.inertiaTensor) || !IsFinite(body.inertiaTensorRotation))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=missingBodyOrNonFiniteState";
                CopyToReport(report, body);
                return Vector3.zero;
            }

            Vector3 inertia = body.inertiaTensor;
            if (inertia.x <= InertiaEpsilon || inertia.y <= InertiaEpsilon || inertia.z <= InertiaEpsilon)
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=inertiaTensorNonPositive";
                CopyToReport(report, body);
                return Vector3.zero;
            }

            _activeSteps++;
            _peakDesiredA = Mathf.Max(_peakDesiredA, _currentDesiredA);
            _peakAngularSpeed = Mathf.Max(_peakAngularSpeed, _currentAngularSpeed);

            Quaternion principalWorld = Normalize(body.rotation * body.inertiaTensorRotation);
            Vector3 inertiaAlphaWorld = MultiplyWorldInertia(principalWorld, inertia, desiredAccelerationWorld);
            Vector3 angularMomentumWorld = MultiplyWorldInertia(principalWorld, inertia, body.angularVelocity);
            Vector3 gyroTorqueWorld = Vector3.Cross(body.angularVelocity, angularMomentumWorld);
            Vector3 physicalTorqueWorld = inertiaAlphaWorld + gyroTorqueWorld;

            if (!IsFinite(inertiaAlphaWorld) || !IsFinite(angularMomentumWorld) ||
                !IsFinite(gyroTorqueWorld) || !IsFinite(physicalTorqueWorld))
            {
                _invalidSteps++;
                CurrentValid = false;
                _lastEvidence = "step=" + _steps + "/invalid=nonFiniteTorqueComposition";
                CopyToReport(report, body);
                return Vector3.zero;
            }

            Vector3 predictedAccelerationWorld = InverseWorldInertia(
                principalWorld,
                inertia,
                physicalTorqueWorld - gyroTorqueWorld);

            _currentPhysicalTorqueWorld = physicalTorqueWorld;
            _currentInertiaAlphaTorque = inertiaAlphaWorld.magnitude;
            _currentGyroTorque = gyroTorqueWorld.magnitude;
            _currentTotalTorque = physicalTorqueWorld.magnitude;
            _currentPredictedA = predictedAccelerationWorld.magnitude;
            _currentPredictionError = (predictedAccelerationWorld - desiredAccelerationWorld).magnitude;
            _currentPredictionAlignment = Alignment(predictedAccelerationWorld, desiredAccelerationWorld);
            _currentGyroToInertiaRatio = _currentInertiaAlphaTorque > 1e-6f
                ? _currentGyroTorque / _currentInertiaAlphaTorque
                : 0f;

            _peakInertiaAlphaTorque = Mathf.Max(_peakInertiaAlphaTorque, _currentInertiaAlphaTorque);
            _peakGyroTorque = Mathf.Max(_peakGyroTorque, _currentGyroTorque);
            _peakTotalTorque = Mathf.Max(_peakTotalTorque, _currentTotalTorque);
            _peakPredictionError = Mathf.Max(_peakPredictionError, _currentPredictionError);
            _peakGyroToInertiaRatio = Mathf.Max(_peakGyroToInertiaRatio, _currentGyroToInertiaRatio);

            _lastEvidence =
                "step=" + _steps +
                "/arm=" + Arm +
                "/desiredA=" + F(_currentDesiredA) +
                "/w=" + F(_currentAngularSpeed) +
                "/tauI=" + F(_currentInertiaAlphaTorque) +
                "/tauGyro=" + F(_currentGyroTorque) +
                "/tau=" + F(_currentTotalTorque) +
                "/predA=" + F(_currentPredictedA) +
                "/predErr=" + F(_currentPredictionError) +
                "/align=" + F(_currentPredictionAlignment) +
                "/gyroRatio=" + F(_currentGyroToInertiaRatio);
            CopyToReport(report, body);

            // In A, caller ignores this vector and preserves AddTorque(desiredA, Acceleration).
            return _enabled ? physicalTorqueWorld : desiredAccelerationWorld;
        }

        internal void CopyToReport(SingleLegChainReport report, Rigidbody body)
        {
            if (report == null) return;
            report.HipAngularActuatorMode = Mode;
            report.HipAngularActuatorArm = Arm;
            report.HipAngularActuatorEnabled = _enabled;
            report.HipAngularActuatorSteps = _steps;
            report.HipAngularActuatorActiveSteps = _activeSteps;
            report.HipAngularActuatorInvalidSteps = _invalidSteps;
            report.HipAngularActuatorCurrentDesiredAcceleration = _currentDesiredA;
            report.HipAngularActuatorCurrentAngularSpeed = _currentAngularSpeed;
            report.HipAngularActuatorCurrentInertiaAlphaTorque = _currentInertiaAlphaTorque;
            report.HipAngularActuatorCurrentGyroTorque = _currentGyroTorque;
            report.HipAngularActuatorCurrentTotalTorque = _currentTotalTorque;
            report.HipAngularActuatorCurrentPredictedAcceleration = _currentPredictedA;
            report.HipAngularActuatorCurrentPredictionError = _currentPredictionError;
            report.HipAngularActuatorCurrentPredictionAlignment = _currentPredictionAlignment;
            report.HipAngularActuatorCurrentGyroToInertiaRatio = _currentGyroToInertiaRatio;
            report.HipAngularActuatorPeakDesiredAcceleration = _peakDesiredA;
            report.HipAngularActuatorPeakAngularSpeed = _peakAngularSpeed;
            report.HipAngularActuatorPeakInertiaAlphaTorque = _peakInertiaAlphaTorque;
            report.HipAngularActuatorPeakGyroTorque = _peakGyroTorque;
            report.HipAngularActuatorPeakTotalTorque = _peakTotalTorque;
            report.HipAngularActuatorPeakPredictionError = _peakPredictionError;
            report.HipAngularActuatorPeakGyroToInertiaRatio = _peakGyroToInertiaRatio;
            report.HipAngularActuatorBodyMass = body != null && IsFinite(body.mass) ? body.mass : 0f;
            report.HipAngularActuatorBodyAngularDrag = body != null && IsFinite(body.angularDrag) ? body.angularDrag : 0f;
            report.HipAngularActuatorBodyInertiaTensor = body != null && IsFinite(body.inertiaTensor) ? body.inertiaTensor : Vector3.zero;
            report.HipAngularActuatorLastEvidence = _lastEvidence;
            report.HipAngularActuatorKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report, Rigidbody body)
        {
            CopyToReport(report, body);
            return "ARL Hip angular actuator inertia-space response closure A/B | mode=" + Mode +
                   " | arm=" + Arm +
                   " | enabled=" + _enabled +
                   " | steps(active/total/invalid)=" + _activeSteps + "/" + _steps + "/" + _invalidSteps +
                   " | body(mass/drag/I)=" + F(report != null ? report.HipAngularActuatorBodyMass : 0f) + "/" +
                   F(report != null ? report.HipAngularActuatorBodyAngularDrag : 0f) + "/" +
                   V(report != null ? report.HipAngularActuatorBodyInertiaTensor : Vector3.zero) +
                   " | current(A/w/tauI/gyro/tau/predA/err/align)=" +
                   F(_currentDesiredA) + "/" + F(_currentAngularSpeed) + "/" + F(_currentInertiaAlphaTorque) + "/" +
                   F(_currentGyroTorque) + "/" + F(_currentTotalTorque) + "/" + F(_currentPredictedA) + "/" +
                   F(_currentPredictionError) + "/" + F(_currentPredictionAlignment) +
                   " | key=" + GetKey(report);
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            if (_invalidSteps > 0)
                primary = "B_INVALID_INERTIA_OR_TORQUE_COMPOSITION";
            else if (_enabled && _activeSteps == 0)
                primary = "B_NO_ACTUATOR_EXPOSURE";
            else if (!_enabled)
                primary = "A_V080_FORCEMODE_ACCELERATION_BASELINE";
            else
                primary = "B_INERTIA_SPACE_ACTUATOR_ACTIVE_FROZEN";

            return "primary=" + primary +
                   " | mode=" + Mode +
                   " | arm=" + Arm +
                   " | steps(active/total/invalid)=" + _activeSteps + "/" + _steps + "/" + _invalidSteps +
                   " | peak(desiredA/w/tauI/gyro/tau/predErr/gyroRatio)=" +
                   F(_peakDesiredA) + "/" + F(_peakAngularSpeed) + "/" + F(_peakInertiaAlphaTorque) + "/" +
                   F(_peakGyroTorque) + "/" + F(_peakTotalTorque) + "/" + F(_peakPredictionError) + "/" + F(_peakGyroToInertiaRatio) +
                   " | historicalResponseObserver=RETIRED_V0317";
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

        private static float Alignment(Vector3 a, Vector3 b)
        {
            float denom = a.magnitude * b.magnitude;
            if (!IsFinite(denom) || denom <= 1e-6f)
                return a.sqrMagnitude <= 1e-10f && b.sqrMagnitude <= 1e-10f ? 1f : 0f;
            return Mathf.Clamp(Vector3.Dot(a, b) / denom, -1f, 1f);
        }

        private static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!IsFinite(mag) || mag <= 1e-6f) return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(Quaternion value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        }

        private static string F(float value)
        {
            return IsFinite(value) ? value.ToString("0.######") : "nan";
        }

        private static string V(Vector3 value)
        {
            return "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
        }
    }
}
