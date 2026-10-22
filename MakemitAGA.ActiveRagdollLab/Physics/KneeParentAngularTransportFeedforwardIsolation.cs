using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.75 causal A/B for the serial-chain Lower angular transport gap exposed by v0.2.74.
    ///
    /// A keeps the exact v0.2.74 behavior: Lower receives only the existing Knee relative controller.
    /// B adds the exact same-step FINAL Hip angular acceleration command already sent to Upper as a world-space
    /// common-mode transport term on Lower, then adds the unchanged Knee relative command on top.
    ///
    /// This does NOT increase Knee relative authority, does not change Knee Kp/Kd/MaxA/PCA/CharacterJoint, and
    /// introduces no profile knob. The transport term is already bounded by the existing Hip final 40/80 ceiling.
    /// Historical note corrected in v0.3.34: the shared value is an angular-acceleration intent, but the current
    /// Upper Hip actuator may map that intent through explicit inertia/gyro and apply ForceMode.Force, while Lower
    /// receives the copied common-mode term through ForceMode.Acceleration. Therefore equal numeric intent does not
    /// by itself prove equal realized angular response; v0.3.34 observes that response separately. Any remaining
    /// relative motion is left to the unchanged Knee controller and PhysX response.
    /// </summary>
    internal sealed class KneeParentAngularTransportFeedforwardIsolation
    {
        internal const string Mode = "KNEE_PARENT_ANGULAR_TRANSPORT_FEEDFORWARD_CAUSAL_AB";
        internal const string ArmA = "A_V074_RELATIVE_KNEE_COMMAND_ONLY";
        internal const string ArmB = "B_V075_HIP_COMMAND_COMMON_MODE_TRANSPORT";

        private bool _enabled;
        private int _steps;
        private int _activeSteps;
        private int _invalidSteps;
        private float _transportMagnitude;
        private float _relativeMagnitude;
        private float _totalMagnitude;
        private float _peakTransportMagnitude;
        private float _peakRelativeMagnitude;
        private float _peakTotalMagnitude;
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
            _transportMagnitude = 0f;
            _relativeMagnitude = 0f;
            _totalMagnitude = 0f;
            _peakTransportMagnitude = 0f;
            _peakRelativeMagnitude = 0f;
            _peakTotalMagnitude = 0f;
            _lastEvidence = "<none>";
            CopyToReport(report);
        }

        internal Vector3 Compose(
            Vector3 finalHipAppliedAngularAccelerationWorld,
            Vector3 kneeRelativeCommandWorld,
            SingleLegChainReport report)
        {
            _steps++;
            _relativeMagnitude = kneeRelativeCommandWorld.magnitude;

            if (!_enabled)
            {
                _transportMagnitude = 0f;
                _totalMagnitude = _relativeMagnitude;
                _peakRelativeMagnitude = Mathf.Max(_peakRelativeMagnitude, _relativeMagnitude);
                _peakTotalMagnitude = Mathf.Max(_peakTotalMagnitude, _totalMagnitude);
                CopyToReport(report);
                return kneeRelativeCommandWorld;
            }

            if (!IsFinite(finalHipAppliedAngularAccelerationWorld) || !IsFinite(kneeRelativeCommandWorld))
            {
                _invalidSteps++;
                _transportMagnitude = 0f;
                _totalMagnitude = _relativeMagnitude;
                _lastEvidence = "invalid=True/step=" + _steps;
                CopyToReport(report);
                return kneeRelativeCommandWorld;
            }

            Vector3 total = finalHipAppliedAngularAccelerationWorld + kneeRelativeCommandWorld;
            if (!IsFinite(total))
            {
                _invalidSteps++;
                _transportMagnitude = 0f;
                _totalMagnitude = _relativeMagnitude;
                _lastEvidence = "invalidTotal=True/step=" + _steps;
                CopyToReport(report);
                return kneeRelativeCommandWorld;
            }

            _activeSteps++;
            _transportMagnitude = finalHipAppliedAngularAccelerationWorld.magnitude;
            _totalMagnitude = total.magnitude;
            _peakTransportMagnitude = Mathf.Max(_peakTransportMagnitude, _transportMagnitude);
            _peakRelativeMagnitude = Mathf.Max(_peakRelativeMagnitude, _relativeMagnitude);
            _peakTotalMagnitude = Mathf.Max(_peakTotalMagnitude, _totalMagnitude);
            _lastEvidence =
                "step=" + _steps +
                "/transportA=" + F(_transportMagnitude) +
                "/relativeA=" + F(_relativeMagnitude) +
                "/totalA=" + F(_totalMagnitude);
            CopyToReport(report);
            return total;
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.KneeParentAngularTransportMode = Mode;
            report.KneeParentAngularTransportArm = Arm;
            report.KneeParentAngularTransportEnabled = _enabled;
            report.KneeParentAngularTransportSteps = _steps;
            report.KneeParentAngularTransportActiveSteps = _activeSteps;
            report.KneeParentAngularTransportInvalidSteps = _invalidSteps;
            report.KneeParentAngularTransportCurrentTransportAcceleration = _transportMagnitude;
            report.KneeParentAngularTransportCurrentRelativeAcceleration = _relativeMagnitude;
            report.KneeParentAngularTransportCurrentTotalLowerAcceleration = _totalMagnitude;
            report.KneeParentAngularTransportPeakTransportAcceleration = _peakTransportMagnitude;
            report.KneeParentAngularTransportPeakRelativeAcceleration = _peakRelativeMagnitude;
            report.KneeParentAngularTransportPeakTotalLowerAcceleration = _peakTotalMagnitude;
            report.KneeParentAngularTransportLastEvidence = _lastEvidence;
            report.KneeParentAngularTransportKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return "ARL Knee parent angular transport A/B | mode=" + Mode +
                   " | arm=" + Arm +
                   " | enabled=" + _enabled +
                   " | steps(active/total/invalid)=" + _activeSteps + "/" + _steps + "/" + _invalidSteps +
                   " | current(transport/relative/totalA)=" + F(_transportMagnitude) + "/" + F(_relativeMagnitude) + "/" + F(_totalMagnitude) +
                   " | peak(transport/relative/totalA)=" + F(_peakTransportMagnitude) + "/" + F(_peakRelativeMagnitude) + "/" + F(_peakTotalMagnitude) +
                   " | key=" + GetKey(report);
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            if (!_enabled)
                primary = "A_V074_NO_PARENT_ANGULAR_TRANSPORT";
            else if (_invalidSteps > 0)
                primary = "B_INVALID_NON_FINITE_TRANSPORT_OBSERVED";
            else if (_activeSteps == 0)
                primary = "B_NO_EXPOSURE";
            else
                primary = "B_PARENT_ANGULAR_TRANSPORT_ACTIVE_FROZEN";

            return "primary=" + primary +
                   " | mode=" + Mode +
                   " | arm=" + Arm +
                   " | steps(active/total/invalid)=" + _activeSteps + "/" + _steps + "/" + _invalidSteps +
                   " | peak(transport/relative/totalA)=" + F(_peakTransportMagnitude) + "/" + F(_peakRelativeMagnitude) + "/" + F(_peakTotalMagnitude) +
                   " | knee(clamp/mean/max)=" +
                   (report != null ? report.TraceKneeClampSamples : 0) + "/" +
                   F(report != null ? report.TraceKneeRotationMeanDegrees : 0f) + "/" +
                   F(report != null ? report.TraceKneeRotationMaxDegrees : 0f);
        }

        private static bool IsFinite(Vector3 v)
        {
            return !float.IsNaN(v.x) && !float.IsInfinity(v.x) &&
                   !float.IsNaN(v.y) && !float.IsInfinity(v.y) &&
                   !float.IsNaN(v.z) && !float.IsInfinity(v.z);
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
