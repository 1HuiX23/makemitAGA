using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.50 — bounded whole-leg common-mode linear parent transport A/B.
    ///
    /// v0.2.49 showed a directional match between Hip-anchor linear acceleration and the large non-command UpperLeg
    /// angular residual. This module does not add any angular authority. Instead it applies the SAME bounded world-space
    /// linear acceleration at the COM of both UpperLeg and LowerLeg so the whole leg shares the kinematic Hip parent's
    /// translational acceleration before CharacterJoint offset constraints have to create it from a point reaction.
    ///
    /// The input is the same-FixedUpdate commanded linear acceleration of the ARL-owned INERTIALIZED Hip mechanical
    /// parent. REAL_PELVIS is never consumed. The acceleration bound reuses the already frozen torso parent-motion FF
    /// ceiling (MaxParentMotionFeedforwardAcceleration, default 20 m/s^2); no new profile knob is introduced.
    /// </summary>
    internal sealed class HipAnchorCommonModeLinearTransport
    {
        public const string ModeName = "BOUNDED_WHOLE_LEG_COMMON_MODE_HIP_ANCHOR_LINEAR_TRANSPORT_FF";
        public const string BoundSourceName = "REUSE_FROZEN_TORSO_MAX_PARENT_MOTION_FF_ACCELERATION";

        private bool _enabled = true;
        private bool _activeThisStep;
        private string _suppressionReason = "NOT_STEPPED";
        private Vector3 _rawWorld;
        private Vector3 _appliedWorld;
        // V04350 R2 publishes the actual COM command and its producer clock, including an evaluated OFF step.
        private int _currentFixed;
        private float _maxA;
        private bool _clamped;
        private int _steps;
        private int _activeSteps;
        private int _disabledSteps;
        private int _realParentSteps;
        private int _inactiveFrameSteps;
        private int _invalidSteps;
        private int _clampSteps;
        private float _peakRaw;
        private float _peakApplied;
        private float _peakRawToBound;
        private string _lastEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal bool ActiveThisStep => _activeThisStep;

        internal bool TrySetEnabled(bool enabled)
        {
            if (_enabled == enabled)
                return false;
            _enabled = enabled;
            return true;
        }

        internal bool StepAndApply(
            int fixedSequence,
            bool useInertializedMechanicalParent,
            bool strictMechanicalFrameActive,
            Vector3 commandedMechanicalParentLinearAccelerationWorld,
            float maxLinearAcceleration,
            Rigidbody upperBody,
            Rigidbody lowerBody,
            SingleLegChainReport report)
        {
            _currentFixed = fixedSequence;
            _steps++;
            _activeThisStep = false;
            _rawWorld = Vector3.zero;
            _appliedWorld = Vector3.zero;
            _maxA = Mathf.Max(0f, maxLinearAcceleration);
            _clamped = false;

            if (!_enabled)
            {
                _disabledSteps++;
                _suppressionReason = "DISABLED_AB";
                CopyToReport(report);
                return true;
            }
            if (!useInertializedMechanicalParent)
            {
                _realParentSteps++;
                _suppressionReason = "REAL_PELVIS_NEVER_CONSUMED";
                CopyToReport(report);
                return true;
            }
            if (!strictMechanicalFrameActive)
            {
                _inactiveFrameSteps++;
                _suppressionReason = "STRICT_MECHANICAL_FRAME_INACTIVE_NORMAL_BYPASS";
                CopyToReport(report);
                return true;
            }
            if (upperBody == null || lowerBody == null ||
                !Finite(commandedMechanicalParentLinearAccelerationWorld) ||
                !Finite(_maxA) || _maxA <= 0.0001f)
            {
                _invalidSteps++;
                _suppressionReason = "INVALID_INPUT";
                CopyToReport(report);
                return false;
            }

            _rawWorld = commandedMechanicalParentLinearAccelerationWorld;
            _appliedWorld = ClampMagnitude(_rawWorld, _maxA);
            _clamped = _rawWorld.sqrMagnitude > _maxA * _maxA + 0.000001f;
            _activeThisStep = true;
            _suppressionReason = "ACTIVE_INERTIALIZED_COMMON_MODE";
            _activeSteps++;
            if (_clamped)
                _clampSteps++;

            float rawMag = _rawWorld.magnitude;
            float appliedMag = _appliedWorld.magnitude;
            _peakRaw = Mathf.Max(_peakRaw, rawMag);
            _peakApplied = Mathf.Max(_peakApplied, appliedMag);
            _peakRawToBound = Mathf.Max(_peakRawToBound, _maxA > 0.0001f ? rawMag / _maxA : 0f);

            // ForceMode.Acceleration at Rigidbody COM: identical acceleration on both segments and no direct torque.
            upperBody.AddForce(_appliedWorld, ForceMode.Acceleration);
            lowerBody.AddForce(_appliedWorld, ForceMode.Acceleration);

            _lastEvidence =
                "raw=" + V(_rawWorld) + "/applied=" + V(_appliedWorld) +
                "/maxA=" + F(_maxA) + "/clamp=" + _clamped;
            CopyToReport(report);
            return true;
        }

        internal void ResetDiagnostics(SingleLegChainReport report)
        {
            _currentFixed = 0;
            _activeThisStep = false;
            _suppressionReason = "RESET";
            _rawWorld = Vector3.zero;
            _appliedWorld = Vector3.zero;
            _maxA = 0f;
            _clamped = false;
            _steps = 0;
            _activeSteps = 0;
            _disabledSteps = 0;
            _realParentSteps = 0;
            _inactiveFrameSteps = 0;
            _invalidSteps = 0;
            _clampSteps = 0;
            _peakRaw = 0f;
            _peakApplied = 0f;
            _peakRawToBound = 0f;
            _lastEvidence = "<none>";
            CopyToReport(report);
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null)
                return;
            report.HipAnchorLinearTransportMode = ModeName;
            report.HipAnchorLinearTransportBoundSource = BoundSourceName;
            report.HipAnchorLinearTransportEnabled = _enabled;
            report.HipAnchorLinearTransportActiveThisStep = _activeThisStep;
            report.HipAnchorLinearTransportCurrentFixed = _currentFixed;
            report.HipAnchorLinearTransportCurrentAppliedAccelerationWorld = _appliedWorld;
            report.HipAnchorLinearTransportSuppressionReason = _suppressionReason;
            report.HipAnchorLinearTransportCurrentRawAcceleration = _rawWorld.magnitude;
            report.HipAnchorLinearTransportCurrentAppliedAcceleration = _appliedWorld.magnitude;
            report.HipAnchorLinearTransportCurrentMaxAcceleration = _maxA;
            report.HipAnchorLinearTransportClampedThisStep = _clamped;
            report.HipAnchorLinearTransportSteps = _steps;
            report.HipAnchorLinearTransportActiveSteps = _activeSteps;
            report.HipAnchorLinearTransportDisabledSteps = _disabledSteps;
            report.HipAnchorLinearTransportRealParentSteps = _realParentSteps;
            report.HipAnchorLinearTransportInactiveFrameSteps = _inactiveFrameSteps;
            report.HipAnchorLinearTransportInvalidSteps = _invalidSteps;
            report.HipAnchorLinearTransportClampSteps = _clampSteps;
            report.HipAnchorLinearTransportPeakRawAcceleration = _peakRaw;
            report.HipAnchorLinearTransportPeakAppliedAcceleration = _peakApplied;
            report.HipAnchorLinearTransportPeakRawToBoundRatio = _peakRawToBound;
            report.HipAnchorLinearTransportLastEvidence = _lastEvidence;
            report.HipAnchorLinearTransportKey = GetKey();
        }

        internal string GetKey()
        {
            return
                "mode=" + ModeName +
                " | enabled=" + _enabled +
                " | boundSource=" + BoundSourceName +
                " | current(active/reason/raw/applied/maxA/clamp)=" + _activeThisStep + "/" + (_suppressionReason ?? "<none>") + "/" + F(_rawWorld.magnitude) + "/" + F(_appliedWorld.magnitude) + "/" + F(_maxA) + "/" + _clamped +
                " | steps(total/active/disabled/real/inactiveFrame/invalid/clamp)=" + _steps + "/" + _activeSteps + "/" + _disabledSteps + "/" + _realParentSteps + "/" + _inactiveFrameSteps + "/" + _invalidSteps + "/" + _clampSteps +
                " | peak(raw/applied/rawToBound)=" + F(_peakRaw) + "/" + F(_peakApplied) + "/" + F(_peakRawToBound) +
                " | application=UPPER_AND_LOWER_COM_SAME_ACCELERATION_FORCEMODE_ACCELERATION_NO_DIRECT_TORQUE" +
                " | rawPelvisLinearA=NEVER_USED_DIRECTLY" +
                " | last=" + (_lastEvidence ?? "<none>");
        }

        private static Vector3 ClampMagnitude(Vector3 value, float max)
        {
            if (max <= 0f)
                return Vector3.zero;
            float sqr = value.sqrMagnitude;
            if (sqr <= max * max)
                return value;
            float mag = Mathf.Sqrt(sqr);
            return mag > 0.000001f ? value * (max / mag) : Vector3.zero;
        }

        private static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
        private static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string F(float v) => v.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")";
    }
}
