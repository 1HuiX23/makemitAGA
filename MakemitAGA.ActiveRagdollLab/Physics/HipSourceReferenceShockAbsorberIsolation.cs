using System;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.71 causal A/B.
    ///
    /// v0.2.70 proved that suppressing the 25deg reference lag-guard rebase removes very large reference-side
    /// corrections but does NOT remove the ~100deg severe Hip fling. The remaining extreme cases are instead
    /// correlated with raw animation Source packets that can jump tens (or even >100) degrees in one Render sample.
    ///
    /// A = exact v0.2.70 input path: raw sampled Hip-local pose/W pass straight through.
    /// B = a stateful Hip-local Source accumulator inserted BEFORE the existing feasibility governor/vector capture.
    ///     It never applies Rigidbody force/torque. During Source it reuses the already-frozen strict Source W/A
    ///     budgets (6rad/s, 60rad/s^2 by default); after Source it reuses the frozen recovery budgets (8/80).
    ///     Pose is integrated only from the bounded W trajectory; the final raw handoff is allowed only at a numerically-negligible pose gap with an acceleration-feasible W gap.
    ///
    /// Raw animation samples remain authoritative for health/diagnostics. Only the downstream PHYSICAL CONTROL TARGET
    /// sees the absorbed pose/W in B, making this a clean test of Source-reference feasibility rather than authority.
    /// </summary>
    internal sealed class HipSourceReferenceShockAbsorberIsolation
    {
        public const string ModeName = "HIP_SOURCE_REFERENCE_SHOCK_ABSORPTION_CAUSAL_AB";
        public const string ArmA = "A_V070_RAW_SOURCE_REFERENCE";
        public const string ArmB = "B_V071_FEASIBILITY_LIMITED_SOURCE_ACCUMULATOR";
        private const float ContractTolerance = 0.02f;

        private bool _enabled;
        private bool _active;
        private Quaternion _outputPose = Quaternion.identity;
        private Vector3 _outputAngularVelocityLocal;
        private Quaternion _lastBypassPose = Quaternion.identity;
        private Vector3 _lastBypassW;
        private bool _hasBypassState;
        private float _releaseStableSeconds;

        private Quaternion _previousRawPacketPose = Quaternion.identity;
        private bool _hasPreviousRawPacket;
        private int _lastRenderSequence;

        private int _fixedSteps;
        private int _sourceSteps;
        private int _recoverySteps;
        private int _activationCount;
        private int _releaseCount;
        private int _speedLimitedSteps;
        private int _accelerationLimitedSteps;
        private int _rawPacketShockCount;
        private float _currentRawGapDegrees;
        private float _currentOutputStepDegrees;
        private float _currentOutputAngularSpeed;
        private float _currentOutputAngularAcceleration;
        private float _currentMaxW;
        private float _currentMaxA;
        private float _peakRawPacketDeltaDegrees;
        private float _peakRawGapDegrees;
        private float _peakOutputStepDegrees;
        private float _peakOutputAngularSpeed;
        private float _peakOutputAngularAcceleration;
        private float _peakAccelerationRatio;
        private bool _speedContractSatisfied = true;
        private bool _accelerationContractSatisfied = true;
        private string _lastEvidence = "<none>";
        private string _abortEvidence = "<none>";

        public bool Enabled => _enabled;
        public bool Active => _enabled && _active;
        public string Arm => _enabled ? ArmB : ArmA;
        public Quaternion OutputPose => _enabled ? _outputPose : _lastBypassPose;
        public Vector3 OutputAngularVelocityLocal => _enabled ? _outputAngularVelocityLocal : _lastBypassW;
        public bool OutputValid => Finite(OutputPose) && Finite(OutputAngularVelocityLocal);
        public bool SpeedContractSatisfied => _speedContractSatisfied;
        public bool AccelerationContractSatisfied => _accelerationContractSatisfied;

        public void SetEnabled(bool enabled, Quaternion currentRawPose, Vector3 currentRawW, SingleLegChainReport report)
        {
            _enabled = enabled;
            currentRawPose = Finite(currentRawPose) ? Normalize(currentRawPose) : Quaternion.identity;
            currentRawW = Finite(currentRawW) ? currentRawW : Vector3.zero;
            _active = false;
            _releaseStableSeconds = 0f;
            _outputPose = currentRawPose;
            _outputAngularVelocityLocal = currentRawW;
            _lastBypassPose = currentRawPose;
            _lastBypassW = currentRawW;
            _hasBypassState = true;
            ResetDiagnostics(report);
        }

        public void Reset(Quaternion currentRawPose, Vector3 currentRawW, SingleLegChainReport report)
        {
            currentRawPose = Finite(currentRawPose) ? Normalize(currentRawPose) : Quaternion.identity;
            currentRawW = Finite(currentRawW) ? currentRawW : Vector3.zero;
            _active = false;
            _releaseStableSeconds = 0f;
            _outputPose = currentRawPose;
            _outputAngularVelocityLocal = currentRawW;
            _lastBypassPose = currentRawPose;
            _lastBypassW = currentRawW;
            _hasBypassState = true;
            ResetDiagnostics(report);
        }

        public void ResetDiagnostics(SingleLegChainReport report)
        {
            _fixedSteps = 0;
            _sourceSteps = 0;
            _recoverySteps = 0;
            _activationCount = 0;
            _releaseCount = 0;
            _speedLimitedSteps = 0;
            _accelerationLimitedSteps = 0;
            _rawPacketShockCount = 0;
            _currentRawGapDegrees = 0f;
            _currentOutputStepDegrees = 0f;
            _currentOutputAngularSpeed = 0f;
            _currentOutputAngularAcceleration = 0f;
            _currentMaxW = 0f;
            _currentMaxA = 0f;
            _peakRawPacketDeltaDegrees = 0f;
            _peakRawGapDegrees = 0f;
            _peakOutputStepDegrees = 0f;
            _peakOutputAngularSpeed = 0f;
            _peakOutputAngularAcceleration = 0f;
            _peakAccelerationRatio = 0f;
            _speedContractSatisfied = true;
            _accelerationContractSatisfied = true;
            _lastEvidence = "<none>";
            _abortEvidence = "<none>";
            CopyToReport(report);
        }

        public void ObserveRenderPacket(Quaternion rawPose, int renderSequence, float shockThresholdDegrees)
        {
            if (!Finite(rawPose) || renderSequence <= 0 || renderSequence == _lastRenderSequence)
                return;
            rawPose = Normalize(rawPose);
            if (_hasPreviousRawPacket)
            {
                float delta = Quaternion.Angle(_previousRawPacketPose, rawPose);
                _peakRawPacketDeltaDegrees = Mathf.Max(_peakRawPacketDeltaDegrees, delta);
                if (delta >= Mathf.Max(0.1f, shockThresholdDegrees))
                    _rawPacketShockCount++;
            }
            _previousRawPacketPose = rawPose;
            _hasPreviousRawPacket = true;
            _lastRenderSequence = renderSequence;
        }

        public void Step(
            ActiveRagdollLabProfile profile,
            bool sourceActive,
            Quaternion rawPose,
            Vector3 rawAngularVelocityLocal,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            if (profile == null || !Finite(rawPose) || !Finite(rawAngularVelocityLocal) ||
                !Finite(fixedDeltaTime) || fixedDeltaTime <= 0.0001f)
            {
                _speedContractSatisfied = false;
                _accelerationContractSatisfied = false;
                CopyToReport(report);
                return;
            }

            rawPose = Normalize(rawPose);
            _fixedSteps++;

            if (!_enabled)
            {
                _active = false;
                _releaseStableSeconds = 0f;
                _outputPose = rawPose;
                _outputAngularVelocityLocal = rawAngularVelocityLocal;
                _lastBypassPose = rawPose;
                _lastBypassW = rawAngularVelocityLocal;
                _hasBypassState = true;
                ClearCurrent();
                CopyToReport(report);
                return;
            }

            if (!_active && !sourceActive)
            {
                _outputPose = rawPose;
                _outputAngularVelocityLocal = rawAngularVelocityLocal;
                _lastBypassPose = rawPose;
                _lastBypassW = rawAngularVelocityLocal;
                _hasBypassState = true;
                _releaseStableSeconds = 0f;
                ClearCurrent();
                CopyToReport(report);
                return;
            }

            if (!_active && sourceActive)
            {
                _active = true;
                _activationCount++;
                _releaseStableSeconds = 0f;
                if (_hasBypassState)
                {
                    _outputPose = _lastBypassPose;
                    _outputAngularVelocityLocal = _lastBypassW;
                }
                else
                {
                    _outputPose = rawPose;
                    _outputAngularVelocityLocal = Vector3.zero;
                }
            }

            bool sourcePhase = sourceActive;
            if (sourcePhase) _sourceSteps++; else _recoverySteps++;

            float maxW = Mathf.Max(0.1f, sourcePhase
                ? profile.LegHipControlFrameSourceMaxAngularSpeed
                : profile.LegHipControlFrameRecoveryMaxAngularSpeed);
            float maxA = Mathf.Max(0.1f, sourcePhase
                ? profile.LegHipControlFrameSourceMaxAngularAcceleration
                : profile.LegHipControlFrameRecoveryMaxAngularAcceleration);
            _currentMaxW = maxW;
            _currentMaxA = maxA;

            Quaternion previousPose = _outputPose;
            Vector3 previousW = _outputAngularVelocityLocal;
            Vector3 desiredW = ComputeBrakingDesiredAngularVelocity(previousPose, rawPose, maxA, maxW, fixedDeltaTime);

            Vector3 deltaW = desiredW - previousW;
            float maxDeltaW = maxA * fixedDeltaTime;
            if (deltaW.magnitude > maxDeltaW && deltaW.sqrMagnitude > 1e-12f)
            {
                deltaW = deltaW.normalized * maxDeltaW;
                _accelerationLimitedSteps++;
            }
            Vector3 nextW = previousW + deltaW;
            if (nextW.magnitude > maxW && nextW.sqrMagnitude > 1e-12f)
            {
                nextW = nextW.normalized * maxW;
                _speedLimitedSteps++;
            }

            Quaternion nextPose = IntegrateRotation(previousPose, nextW, fixedDeltaTime);
            float actualA = (nextW - previousW).magnitude / fixedDeltaTime;
            float speedRatio = nextW.magnitude / Mathf.Max(0.0001f, maxW);
            float aRatio = actualA / Mathf.Max(0.0001f, maxA);
            _speedContractSatisfied = Finite(speedRatio) && speedRatio <= 1f + ContractTolerance;
            _accelerationContractSatisfied = Finite(aRatio) && aRatio <= 1f + ContractTolerance;

            _outputPose = nextPose;
            _outputAngularVelocityLocal = nextW;
            _currentRawGapDegrees = Quaternion.Angle(_outputPose, rawPose);
            _currentOutputStepDegrees = Quaternion.Angle(previousPose, nextPose);
            _currentOutputAngularSpeed = nextW.magnitude;
            _currentOutputAngularAcceleration = actualA;
            _peakRawGapDegrees = Mathf.Max(_peakRawGapDegrees, _currentRawGapDegrees);
            _peakOutputStepDegrees = Mathf.Max(_peakOutputStepDegrees, _currentOutputStepDegrees);
            _peakOutputAngularSpeed = Mathf.Max(_peakOutputAngularSpeed, _currentOutputAngularSpeed);
            _peakOutputAngularAcceleration = Mathf.Max(_peakOutputAngularAcceleration, actualA);
            _peakAccelerationRatio = Mathf.Max(_peakAccelerationRatio, aRatio);

            if (!sourceActive)
            {
                float wGap = (_outputAngularVelocityLocal - rawAngularVelocityLocal).magnitude;
                // Keep the final pose handoff numerically negligible. The W handoff is independently bounded by
                // one recovery acceleration step, so returning to raw on the following bypass step cannot recreate
                // a hidden large angular-velocity impulse. 0.01deg is a numerical handoff tolerance, not a tuning knob.
                float releasePoseThreshold = Mathf.Min(
                    profile.LegHipControlFrameRecoveryExitAngleDegrees,
                    0.01f);
                float releaseWThreshold = Mathf.Min(
                    profile.LegHipControlFrameRecoveryExitAngularSpeed,
                    maxA * fixedDeltaTime);
                bool releaseCandidate = _currentRawGapDegrees <= releasePoseThreshold &&
                                        wGap <= releaseWThreshold;
                if (releaseCandidate)
                    _releaseStableSeconds += fixedDeltaTime;
                else
                    _releaseStableSeconds = 0f;

                if (_releaseStableSeconds >= profile.LegHipControlFrameRecoveryStableHoldSeconds)
                {
                    // Release only after pose is numerically coincident and W handoff is acceleration-feasible.
                    _active = false;
                    _releaseCount++;
                    _releaseStableSeconds = 0f;
                    _lastBypassPose = _outputPose;
                    _lastBypassW = _outputAngularVelocityLocal;
                    _hasBypassState = true;
                }
            }

            _lastEvidence =
                "phase=" + (sourcePhase ? "SOURCE" : "RECOVERY") +
                "/rawGapDeg=" + F(_currentRawGapDegrees) +
                "/stepDeg=" + F(_currentOutputStepDegrees) +
                "/W=" + F(_currentOutputAngularSpeed) + "/" + F(maxW) +
                "/A=" + F(_currentOutputAngularAcceleration) + "/" + F(maxA) +
                "/releaseHold=" + F(_releaseStableSeconds);
            CopyToReport(report);
        }

        public void MarkAbort(string reason, SingleLegChainReport report)
        {
            _abortEvidence =
                "reason=" + (reason ?? "UNKNOWN") +
                "/arm=" + Arm +
                "/active=" + Active +
                "/rawGapDeg=" + F(_currentRawGapDegrees) +
                "/stepDeg=" + F(_currentOutputStepDegrees) +
                "/W=" + F(_currentOutputAngularSpeed) + "/" + F(_currentMaxW) +
                "/A=" + F(_currentOutputAngularAcceleration) + "/" + F(_currentMaxA);
            CopyToReport(report);
        }

        public string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return
                "ARL Hip Source-reference shock absorption A/B | mode=" + ModeName +
                " | arm=" + Arm +
                " | enabled/active=" + _enabled + "/" + Active +
                " | budget(sourceW-A/recoveryW-A)=existing strict " +
                "6/60 -> 8/80 profile readback" +
                " | current(rawGap/step/W/maxW/A/maxA)=" +
                F(_currentRawGapDegrees) + "/" + F(_currentOutputStepDegrees) + "/" +
                F(_currentOutputAngularSpeed) + "/" + F(_currentMaxW) + "/" +
                F(_currentOutputAngularAcceleration) + "/" + F(_currentMaxA) +
                " | rawHealth=UNCHANGED";
        }

        public string GetKey()
        {
            string primary;
            if (!_enabled) primary = "A_RAW_SOURCE_REFERENCE_BASELINE";
            else if (_fixedSteps <= 0) primary = "B_SOURCE_ABSORBER_INSUFFICIENT_EXPOSURE";
            else if (!_speedContractSatisfied || !_accelerationContractSatisfied) primary = "B_SOURCE_ABSORBER_CONTRACT_BROKEN";
            else if (_rawPacketShockCount > 0 && _sourceSteps > 0) primary = "B_SOURCE_SHOCKS_FEASIBILITY_LIMITED";
            else primary = "B_SOURCE_ABSORBER_ACTIVE_NO_EXTREME_PACKET_OBSERVED";

            return
                "primary=" + primary +
                " | mode=" + ModeName +
                " | arm=" + Arm +
                " | active=" + Active +
                " | steps(total/source/recovery)=" + _fixedSteps + "/" + _sourceSteps + "/" + _recoverySteps +
                " | lifecycle(activate/release)=" + _activationCount + "/" + _releaseCount +
                " | limiting(speed/A/rawPacketShock)=" + _speedLimitedSteps + "/" + _accelerationLimitedSteps + "/" + _rawPacketShockCount +
                " | peak(rawPacket/rawGap/outputStep/W/A/Aratio)=" +
                F(_peakRawPacketDeltaDegrees) + "/" + F(_peakRawGapDegrees) + "/" + F(_peakOutputStepDegrees) + "/" +
                F(_peakOutputAngularSpeed) + "/" + F(_peakOutputAngularAcceleration) + "/" + F(_peakAccelerationRatio) +
                " | contract(speed/A)=" + _speedContractSatisfied + "/" + _accelerationContractSatisfied +
                " | last=" + (_lastEvidence ?? "<none>") +
                " | abort=" + (_abortEvidence ?? "<none>");
        }

        public void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.HipSourceReferenceShockAbsorberMode = ModeName;
            report.HipSourceReferenceShockAbsorberArm = Arm;
            report.HipSourceReferenceShockAbsorberEnabled = _enabled;
            report.HipSourceReferenceShockAbsorberActive = Active;
            report.HipSourceReferenceShockAbsorberFixedSteps = _fixedSteps;
            report.HipSourceReferenceShockAbsorberSourceSteps = _sourceSteps;
            report.HipSourceReferenceShockAbsorberRecoverySteps = _recoverySteps;
            report.HipSourceReferenceShockAbsorberActivations = _activationCount;
            report.HipSourceReferenceShockAbsorberReleases = _releaseCount;
            report.HipSourceReferenceShockAbsorberSpeedLimitedSteps = _speedLimitedSteps;
            report.HipSourceReferenceShockAbsorberAccelerationLimitedSteps = _accelerationLimitedSteps;
            report.HipSourceReferenceShockAbsorberRawPacketShockCount = _rawPacketShockCount;
            report.HipSourceReferenceShockAbsorberCurrentRawGapDegrees = _currentRawGapDegrees;
            report.HipSourceReferenceShockAbsorberCurrentOutputStepDegrees = _currentOutputStepDegrees;
            report.HipSourceReferenceShockAbsorberCurrentAngularSpeed = _currentOutputAngularSpeed;
            report.HipSourceReferenceShockAbsorberCurrentAngularAcceleration = _currentOutputAngularAcceleration;
            report.HipSourceReferenceShockAbsorberCurrentMaxAngularSpeed = _currentMaxW;
            report.HipSourceReferenceShockAbsorberCurrentMaxAngularAcceleration = _currentMaxA;
            report.HipSourceReferenceShockAbsorberPeakRawPacketDeltaDegrees = _peakRawPacketDeltaDegrees;
            report.HipSourceReferenceShockAbsorberPeakRawGapDegrees = _peakRawGapDegrees;
            report.HipSourceReferenceShockAbsorberPeakOutputStepDegrees = _peakOutputStepDegrees;
            report.HipSourceReferenceShockAbsorberPeakAngularSpeed = _peakOutputAngularSpeed;
            report.HipSourceReferenceShockAbsorberPeakAngularAcceleration = _peakOutputAngularAcceleration;
            report.HipSourceReferenceShockAbsorberPeakAccelerationRatio = _peakAccelerationRatio;
            report.HipSourceReferenceShockAbsorberSpeedContractSatisfied = _speedContractSatisfied;
            report.HipSourceReferenceShockAbsorberAccelerationContractSatisfied = _accelerationContractSatisfied;
            report.HipSourceReferenceShockAbsorberLastEvidence = _lastEvidence;
            report.HipSourceReferenceShockAbsorberAbortEvidence = _abortEvidence;
            report.HipSourceReferenceShockAbsorberKey = GetKey();
        }

        private void ClearCurrent()
        {
            _currentRawGapDegrees = 0f;
            _currentOutputStepDegrees = 0f;
            _currentOutputAngularSpeed = 0f;
            _currentOutputAngularAcceleration = 0f;
            _currentMaxW = 0f;
            _currentMaxA = 0f;
        }

        private static Vector3 ComputeBrakingDesiredAngularVelocity(Quaternion from, Quaternion to, float maxA, float maxW, float dt)
        {
            Quaternion delta = Normalize(to * Quaternion.Inverse(from));
            delta.ToAngleAxis(out float angleDegrees, out Vector3 axis);
            if (!Finite(angleDegrees) || !Finite(axis) || axis.sqrMagnitude < 1e-12f)
                return Vector3.zero;
            if (angleDegrees > 180f) angleDegrees -= 360f;
            float angleRad = angleDegrees * Mathf.Deg2Rad;
            if (Mathf.Abs(angleRad) < 1e-6f)
                return Vector3.zero;
            axis.Normalize();
            float stoppingLimitedW = Mathf.Sqrt(Mathf.Max(0f, 2f * maxA * Mathf.Abs(angleRad)));
            float oneStepSpeed = Mathf.Abs(angleRad) / Mathf.Max(0.0001f, dt);
            float desiredSpeed = Mathf.Min(maxW, Mathf.Min(stoppingLimitedW, oneStepSpeed));
            return axis * Mathf.Sign(angleRad) * desiredSpeed;
        }

        private static Quaternion IntegrateRotation(Quaternion pose, Vector3 angularVelocityLocal, float dt)
        {
            float speed = angularVelocityLocal.magnitude;
            if (!Finite(speed) || speed < 1e-7f)
                return Normalize(pose);
            Vector3 axis = angularVelocityLocal / speed;
            Quaternion step = Quaternion.AngleAxis(speed * dt * Mathf.Rad2Deg, axis);
            return Normalize(step * pose);
        }

        private static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!Finite(mag) || mag < 1e-8f) return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        private static bool Finite(Quaternion q) => Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w);
        private static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
        private static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string F(float v) => v.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
