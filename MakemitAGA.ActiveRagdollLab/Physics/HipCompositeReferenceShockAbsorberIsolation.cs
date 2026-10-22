using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.72 causal A/B.
    ///
    /// v0.2.71 proved that bounding only the Hip-local Source target is useful but incomplete: the real run reduced
    /// Hip peak from ~107.6deg to ~75.8deg and avoided safety abort, yet the final composed Upper world reference still
    /// reached ~7.14deg/step and ~222rad/s^2. Composite attribution showed parent contribution (~7.57deg) dominating
    /// the local contribution (~2.74deg). Therefore separately feasible parent/local trajectories do not guarantee a
    /// feasible product trajectory once their rotating-frame dynamics are composed.
    ///
    /// A = exact v0.2.71 physical-control path. This class is diagnostic/bypass only.
    /// B = after the existing v0.2.71 local absorber + v0.2.44 governor, compose the ACTUAL parent*HipLocal world target,
    ///     then pass that final world pose/W through a second stateful feasibility accumulator. The bounded world output
    ///     is converted back into an equivalent Hip-local pose/W in the unchanged current control-parent frame before PD.
    ///
    /// B never changes the real Pelvis, the inertialized mechanical parent, CharacterJoint, authority, local absorber,
    /// governor, vector capture, radial floor, or any Rigidbody actuator. It reuses the frozen Source/Recovery 6/60 ->
    /// 8/80 W/A budgets. Thus the only causal variable is whether the FINAL composite Upper world reference itself is
    /// feasibility-limited before the unchanged Hip controller consumes it.
    /// </summary>
    internal sealed class HipCompositeReferenceShockAbsorberIsolation
    {
        public const string ModeName = "HIP_COMPOSITE_WORLD_REFERENCE_SHOCK_ABSORPTION_CAUSAL_AB";
        public const string ArmA = "A_V071_LOCAL_ONLY_SOURCE_ABSORPTION";
        public const string ArmB = "B_V072_FINAL_COMPOSITE_WORLD_REFERENCE_ABSORBED";
        private const float ContractTolerance = 0.02f;

        private bool _enabled;
        private bool _active;
        private bool _outputAppliedThisStep;
        private Quaternion _outputWorldPose = Quaternion.identity;
        private Vector3 _outputWorldW;
        private Quaternion _lastBypassWorldPose = Quaternion.identity;
        private Vector3 _lastBypassWorldW;
        private bool _hasBypassState;
        private float _releaseStableSeconds;

        private Quaternion _previousInputWorldPose = Quaternion.identity;
        private Vector3 _previousInputWorldW;
        private bool _hasPreviousInput;

        private int _fixedSteps;
        private int _sourceSteps;
        private int _recoverySteps;
        private int _activationCount;
        private int _releaseCount;
        private int _speedLimitedSteps;
        private int _accelerationLimitedSteps;
        private int _inheritedOverSpeedSteps;
        private int _inputSpikeSteps;
        private float _currentInputGapDegrees;
        private float _currentInputStepDegrees;
        private float _currentInputAngularSpeed;
        private float _currentInputAngularAcceleration;
        private float _currentOutputStepDegrees;
        private float _currentOutputAngularSpeed;
        private float _currentOutputAngularAcceleration;
        private float _currentMaxW;
        private float _currentMaxA;
        private float _currentLocalCompensationDegrees;
        private bool _currentUpstreamReferenceActive;
        private float _peakInputGapDegrees;
        private float _peakInputStepDegrees;
        private float _peakInputAngularSpeed;
        private float _peakInputAngularAcceleration;
        private float _peakOutputStepDegrees;
        private float _peakOutputAngularSpeed;
        private float _peakOutputAngularAcceleration;
        private float _peakOutputAccelerationRatio;
        private float _peakLocalCompensationDegrees;
        private bool _speedContractSatisfied = true;
        private bool _accelerationContractSatisfied = true;
        private string _lastEvidence = "<none>";
        private string _abortEvidence = "<none>";

        public bool Enabled => _enabled;
        public bool Active => _enabled && _active;
        public bool OutputAppliedThisStep => _enabled && _outputAppliedThisStep;
        public string Arm => _enabled ? ArmB : ArmA;
        public Quaternion OutputWorldPose => _outputWorldPose;
        public Vector3 OutputWorldAngularVelocity => _outputWorldW;
        public bool OutputValid => Finite(_outputWorldPose) && Finite(_outputWorldW);
        public bool SpeedContractSatisfied => _speedContractSatisfied;
        public bool AccelerationContractSatisfied => _accelerationContractSatisfied;

        public void SetEnabled(bool enabled, Quaternion currentInputWorldPose, Vector3 currentInputWorldW, SingleLegChainReport report)
        {
            _enabled = enabled;
            currentInputWorldPose = Finite(currentInputWorldPose) ? Normalize(currentInputWorldPose) : Quaternion.identity;
            currentInputWorldW = Finite(currentInputWorldW) ? currentInputWorldW : Vector3.zero;
            _active = false;
            _outputAppliedThisStep = false;
            _releaseStableSeconds = 0f;
            _outputWorldPose = currentInputWorldPose;
            _outputWorldW = currentInputWorldW;
            _lastBypassWorldPose = currentInputWorldPose;
            _lastBypassWorldW = currentInputWorldW;
            _hasBypassState = true;
            _previousInputWorldPose = currentInputWorldPose;
            _previousInputWorldW = currentInputWorldW;
            _hasPreviousInput = true;
            ResetDiagnostics(report);
        }

        public void Reset(Quaternion currentInputWorldPose, Vector3 currentInputWorldW, SingleLegChainReport report)
        {
            currentInputWorldPose = Finite(currentInputWorldPose) ? Normalize(currentInputWorldPose) : Quaternion.identity;
            currentInputWorldW = Finite(currentInputWorldW) ? currentInputWorldW : Vector3.zero;
            _active = false;
            _outputAppliedThisStep = false;
            _releaseStableSeconds = 0f;
            _outputWorldPose = currentInputWorldPose;
            _outputWorldW = currentInputWorldW;
            _lastBypassWorldPose = currentInputWorldPose;
            _lastBypassWorldW = currentInputWorldW;
            _hasBypassState = true;
            _previousInputWorldPose = currentInputWorldPose;
            _previousInputWorldW = currentInputWorldW;
            _hasPreviousInput = true;
            ResetDiagnostics(report);
        }

        public void ResetDiagnostics(SingleLegChainReport report)
        {
            _fixedSteps = _sourceSteps = _recoverySteps = 0;
            _activationCount = _releaseCount = 0;
            _speedLimitedSteps = _accelerationLimitedSteps = _inheritedOverSpeedSteps = _inputSpikeSteps = 0;
            _currentInputGapDegrees = _currentInputStepDegrees = 0f;
            _currentInputAngularSpeed = _currentInputAngularAcceleration = 0f;
            _currentOutputStepDegrees = _currentOutputAngularSpeed = _currentOutputAngularAcceleration = 0f;
            _currentMaxW = _currentMaxA = 0f;
            _currentLocalCompensationDegrees = 0f;
            _currentUpstreamReferenceActive = false;
            _peakInputGapDegrees = _peakInputStepDegrees = 0f;
            _peakInputAngularSpeed = _peakInputAngularAcceleration = 0f;
            _peakOutputStepDegrees = _peakOutputAngularSpeed = _peakOutputAngularAcceleration = 0f;
            _peakOutputAccelerationRatio = _peakLocalCompensationDegrees = 0f;
            _speedContractSatisfied = true;
            _accelerationContractSatisfied = true;
            _lastEvidence = "<none>";
            _abortEvidence = "<none>";
            CopyToReport(report);
        }

        public void Step(
            ActiveRagdollLabProfile profile,
            bool sourceActive,
            bool upstreamReferenceActive,
            Quaternion inputWorldPose,
            Vector3 inputWorldAngularVelocity,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            _outputAppliedThisStep = false;
            if (profile == null || !Finite(inputWorldPose) || !Finite(inputWorldAngularVelocity) ||
                !Finite(fixedDeltaTime) || fixedDeltaTime <= 0.0001f)
            {
                _speedContractSatisfied = false;
                _accelerationContractSatisfied = false;
                CopyToReport(report);
                return;
            }

            inputWorldPose = Normalize(inputWorldPose);
            _currentUpstreamReferenceActive = upstreamReferenceActive;
            _fixedSteps++;
            ObserveInputDynamics(inputWorldPose, inputWorldAngularVelocity, fixedDeltaTime);

            if (!_enabled)
            {
                _active = false;
                _releaseStableSeconds = 0f;
                _outputWorldPose = inputWorldPose;
                _outputWorldW = inputWorldAngularVelocity;
                _lastBypassWorldPose = inputWorldPose;
                _lastBypassWorldW = inputWorldAngularVelocity;
                _hasBypassState = true;
                ClearCurrentOutput();
                CopyToReport(report);
                return;
            }

            if (!_active && !sourceActive)
            {
                _outputWorldPose = inputWorldPose;
                _outputWorldW = inputWorldAngularVelocity;
                _lastBypassWorldPose = inputWorldPose;
                _lastBypassWorldW = inputWorldAngularVelocity;
                _hasBypassState = true;
                _releaseStableSeconds = 0f;
                ClearCurrentOutput();
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
                    _outputWorldPose = _lastBypassWorldPose;
                    _outputWorldW = _lastBypassWorldW;
                }
                else
                {
                    _outputWorldPose = inputWorldPose;
                    _outputWorldW = inputWorldAngularVelocity;
                }
            }

            _outputAppliedThisStep = true;
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

            Quaternion previousPose = _outputWorldPose;
            Vector3 previousW = _outputWorldW;
            Vector3 desiredW = ComputeBrakingDesiredAngularVelocity(previousPose, inputWorldPose, maxA, maxW, fixedDeltaTime);
            if (desiredW.magnitude >= maxW - 0.0001f)
                _speedLimitedSteps++;

            Vector3 deltaW = desiredW - previousW;
            float maxDeltaW = maxA * fixedDeltaTime;
            if (deltaW.magnitude > maxDeltaW && deltaW.sqrMagnitude > 1e-12f)
            {
                deltaW = deltaW.normalized * maxDeltaW;
                _accelerationLimitedSteps++;
            }
            Vector3 nextW = previousW + deltaW;

            bool inheritedOverSpeed = previousW.magnitude > maxW * (1f + ContractTolerance);
            if (inheritedOverSpeed)
                _inheritedOverSpeedSteps++;
            else if (nextW.magnitude > maxW && nextW.sqrMagnitude > 1e-12f)
                nextW = nextW.normalized * maxW;

            Quaternion nextPose = IntegrateRotation(previousPose, nextW, fixedDeltaTime);
            float actualA = (nextW - previousW).magnitude / fixedDeltaTime;
            float speedRatio = nextW.magnitude / Mathf.Max(0.0001f, maxW);
            float aRatio = actualA / Mathf.Max(0.0001f, maxA);
            bool inheritedDecelerating = inheritedOverSpeed && nextW.magnitude <= previousW.magnitude + 0.0001f;
            _speedContractSatisfied = Finite(speedRatio) && (speedRatio <= 1f + ContractTolerance || inheritedDecelerating);
            _accelerationContractSatisfied = Finite(aRatio) && aRatio <= 1f + ContractTolerance;

            _outputWorldPose = nextPose;
            _outputWorldW = nextW;
            _currentInputGapDegrees = Quaternion.Angle(_outputWorldPose, inputWorldPose);
            _currentOutputStepDegrees = Quaternion.Angle(previousPose, nextPose);
            _currentOutputAngularSpeed = nextW.magnitude;
            _currentOutputAngularAcceleration = actualA;
            _peakInputGapDegrees = Mathf.Max(_peakInputGapDegrees, _currentInputGapDegrees);
            _peakOutputStepDegrees = Mathf.Max(_peakOutputStepDegrees, _currentOutputStepDegrees);
            _peakOutputAngularSpeed = Mathf.Max(_peakOutputAngularSpeed, _currentOutputAngularSpeed);
            _peakOutputAngularAcceleration = Mathf.Max(_peakOutputAngularAcceleration, actualA);
            _peakOutputAccelerationRatio = Mathf.Max(_peakOutputAccelerationRatio, aRatio);

            if (!sourceActive)
            {
                float wGap = (_outputWorldW - inputWorldAngularVelocity).magnitude;
                float releasePoseThreshold = Mathf.Min(profile.LegHipControlFrameRecoveryExitAngleDegrees, 0.01f);
                float releaseWThreshold = Mathf.Min(profile.LegHipControlFrameRecoveryExitAngularSpeed, maxA * fixedDeltaTime);
                bool releaseCandidate = !upstreamReferenceActive &&
                                        _currentInputGapDegrees <= releasePoseThreshold &&
                                        wGap <= releaseWThreshold;
                if (releaseCandidate) _releaseStableSeconds += fixedDeltaTime;
                else _releaseStableSeconds = 0f;

                if (_releaseStableSeconds >= profile.LegHipControlFrameRecoveryStableHoldSeconds)
                {
                    _active = false;
                    _releaseCount++;
                    _releaseStableSeconds = 0f;
                    _lastBypassWorldPose = _outputWorldPose;
                    _lastBypassWorldW = _outputWorldW;
                    _hasBypassState = true;
                }
            }

            _lastEvidence =
                "phase=" + (sourcePhase ? "SOURCE" : "RECOVERY") +
                "/inputGapDeg=" + F(_currentInputGapDegrees) +
                "/inputStepDeg=" + F(_currentInputStepDegrees) +
                "/inputW-A=" + F(_currentInputAngularSpeed) + "/" + F(_currentInputAngularAcceleration) +
                "/outputStepDeg=" + F(_currentOutputStepDegrees) +
                "/outputW=" + F(_currentOutputAngularSpeed) + "/" + F(maxW) +
                "/outputA=" + F(_currentOutputAngularAcceleration) + "/" + F(maxA) +
                "/localCompDeg=" + F(_currentLocalCompensationDegrees) +
                "/upstreamActive=" + upstreamReferenceActive +
                "/releaseHold=" + F(_releaseStableSeconds);
            CopyToReport(report);
        }

        public void CaptureLocalCompensation(Quaternion originalLocalTarget, Quaternion effectiveLocalTarget, SingleLegChainReport report)
        {
            _currentLocalCompensationDegrees = Finite(originalLocalTarget) && Finite(effectiveLocalTarget)
                ? Quaternion.Angle(originalLocalTarget, effectiveLocalTarget)
                : 0f;
            _peakLocalCompensationDegrees = Mathf.Max(_peakLocalCompensationDegrees, _currentLocalCompensationDegrees);
            CopyToReport(report);
        }

        public void MarkAbort(string reason, SingleLegChainReport report)
        {
            _abortEvidence =
                "reason=" + (reason ?? "UNKNOWN") +
                "/arm=" + Arm +
                "/active=" + Active +
                "/inputGapDeg=" + F(_currentInputGapDegrees) +
                "/inputW-A=" + F(_currentInputAngularSpeed) + "/" + F(_currentInputAngularAcceleration) +
                "/outputW-A=" + F(_currentOutputAngularSpeed) + "/" + F(_currentOutputAngularAcceleration) +
                "/localCompDeg=" + F(_currentLocalCompensationDegrees);
            CopyToReport(report);
        }

        public string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return
                "ARL Hip final-composite world-reference shock absorption A/B | mode=" + ModeName +
                " | arm=" + Arm +
                " | enabled/active=" + _enabled + "/" + Active +
                " | budget(sourceW-A/recoveryW-A)=existing strict 6/60 -> 8/80 profile readback" +
                " | current(inputGap/inputStep/inputW/inputA/outputStep/outputW/outputA/localComp)=" +
                F(_currentInputGapDegrees) + "/" + F(_currentInputStepDegrees) + "/" +
                F(_currentInputAngularSpeed) + "/" + F(_currentInputAngularAcceleration) + "/" +
                F(_currentOutputStepDegrees) + "/" + F(_currentOutputAngularSpeed) + "/" +
                F(_currentOutputAngularAcceleration) + "/" + F(_currentLocalCompensationDegrees) +
                " | upstreamReferenceActive=" + _currentUpstreamReferenceActive +
                " | realPelvis/mechanicalParent=UNCHANGED";
        }

        public string GetKey()
        {
            string primary;
            if (!_enabled) primary = "A_V071_LOCAL_ONLY_COMPOSITE_BASELINE";
            else if (_fixedSteps <= 0 || _sourceSteps <= 0) primary = "B_COMPOSITE_ABSORBER_INSUFFICIENT_SOURCE_EXPOSURE";
            else if (!_speedContractSatisfied || !_accelerationContractSatisfied) primary = "B_COMPOSITE_ABSORBER_CONTRACT_BROKEN";
            else if (_inputSpikeSteps > 0) primary = "B_FINAL_COMPOSITE_REFERENCE_FEASIBILITY_LIMITED";
            else primary = "B_COMPOSITE_ABSORBER_ACTIVE_NO_OVER_BUDGET_INPUT_SPIKE";

            return
                "primary=" + primary +
                " | mode=" + ModeName +
                " | arm=" + Arm +
                " | active=" + Active +
                " | steps(total/source/recovery)=" + _fixedSteps + "/" + _sourceSteps + "/" + _recoverySteps +
                " | lifecycle(activate/release)=" + _activationCount + "/" + _releaseCount +
                " | limiting(speed/A/inheritedOverW/inputSpike)=" + _speedLimitedSteps + "/" + _accelerationLimitedSteps + "/" + _inheritedOverSpeedSteps + "/" + _inputSpikeSteps +
                " | peak(inputGap/inputStep/inputW/inputA)=" + F(_peakInputGapDegrees) + "/" + F(_peakInputStepDegrees) + "/" + F(_peakInputAngularSpeed) + "/" + F(_peakInputAngularAcceleration) +
                " | peak(outputStep/outputW/outputA/Aratio/localComp)=" + F(_peakOutputStepDegrees) + "/" + F(_peakOutputAngularSpeed) + "/" + F(_peakOutputAngularAcceleration) + "/" + F(_peakOutputAccelerationRatio) + "/" + F(_peakLocalCompensationDegrees) +
                " | contract(speed/A)=" + _speedContractSatisfied + "/" + _accelerationContractSatisfied +
                " | last=" + (_lastEvidence ?? "<none>") +
                " | abort=" + (_abortEvidence ?? "<none>");
        }

        public void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.HipCompositeReferenceShockAbsorberMode = ModeName;
            report.HipCompositeReferenceShockAbsorberArm = Arm;
            report.HipCompositeReferenceShockAbsorberEnabled = _enabled;
            report.HipCompositeReferenceShockAbsorberActive = Active;
            report.HipCompositeReferenceShockAbsorberFixedSteps = _fixedSteps;
            report.HipCompositeReferenceShockAbsorberSourceSteps = _sourceSteps;
            report.HipCompositeReferenceShockAbsorberRecoverySteps = _recoverySteps;
            report.HipCompositeReferenceShockAbsorberActivations = _activationCount;
            report.HipCompositeReferenceShockAbsorberReleases = _releaseCount;
            report.HipCompositeReferenceShockAbsorberSpeedLimitedSteps = _speedLimitedSteps;
            report.HipCompositeReferenceShockAbsorberAccelerationLimitedSteps = _accelerationLimitedSteps;
            report.HipCompositeReferenceShockAbsorberInheritedOverSpeedSteps = _inheritedOverSpeedSteps;
            report.HipCompositeReferenceShockAbsorberInputSpikeSteps = _inputSpikeSteps;
            report.HipCompositeReferenceShockAbsorberCurrentInputGapDegrees = _currentInputGapDegrees;
            report.HipCompositeReferenceShockAbsorberCurrentInputStepDegrees = _currentInputStepDegrees;
            report.HipCompositeReferenceShockAbsorberCurrentInputAngularSpeed = _currentInputAngularSpeed;
            report.HipCompositeReferenceShockAbsorberCurrentInputAngularAcceleration = _currentInputAngularAcceleration;
            report.HipCompositeReferenceShockAbsorberCurrentOutputStepDegrees = _currentOutputStepDegrees;
            report.HipCompositeReferenceShockAbsorberCurrentOutputAngularSpeed = _currentOutputAngularSpeed;
            report.HipCompositeReferenceShockAbsorberCurrentOutputAngularAcceleration = _currentOutputAngularAcceleration;
            report.HipCompositeReferenceShockAbsorberCurrentMaxAngularSpeed = _currentMaxW;
            report.HipCompositeReferenceShockAbsorberCurrentMaxAngularAcceleration = _currentMaxA;
            report.HipCompositeReferenceShockAbsorberCurrentLocalCompensationDegrees = _currentLocalCompensationDegrees;
            report.HipCompositeReferenceShockAbsorberCurrentUpstreamReferenceActive = _currentUpstreamReferenceActive;
            report.HipCompositeReferenceShockAbsorberPeakInputGapDegrees = _peakInputGapDegrees;
            report.HipCompositeReferenceShockAbsorberPeakInputStepDegrees = _peakInputStepDegrees;
            report.HipCompositeReferenceShockAbsorberPeakInputAngularSpeed = _peakInputAngularSpeed;
            report.HipCompositeReferenceShockAbsorberPeakInputAngularAcceleration = _peakInputAngularAcceleration;
            report.HipCompositeReferenceShockAbsorberPeakOutputStepDegrees = _peakOutputStepDegrees;
            report.HipCompositeReferenceShockAbsorberPeakOutputAngularSpeed = _peakOutputAngularSpeed;
            report.HipCompositeReferenceShockAbsorberPeakOutputAngularAcceleration = _peakOutputAngularAcceleration;
            report.HipCompositeReferenceShockAbsorberPeakOutputAccelerationRatio = _peakOutputAccelerationRatio;
            report.HipCompositeReferenceShockAbsorberPeakLocalCompensationDegrees = _peakLocalCompensationDegrees;
            report.HipCompositeReferenceShockAbsorberSpeedContractSatisfied = _speedContractSatisfied;
            report.HipCompositeReferenceShockAbsorberAccelerationContractSatisfied = _accelerationContractSatisfied;
            report.HipCompositeReferenceShockAbsorberLastEvidence = _lastEvidence;
            report.HipCompositeReferenceShockAbsorberAbortEvidence = _abortEvidence;
            report.HipCompositeReferenceShockAbsorberKey = GetKey();
        }

        private void ObserveInputDynamics(Quaternion inputPose, Vector3 inputW, float dt)
        {
            _currentInputAngularSpeed = inputW.magnitude;
            _currentInputStepDegrees = 0f;
            _currentInputAngularAcceleration = 0f;
            if (_hasPreviousInput)
            {
                _currentInputStepDegrees = Quaternion.Angle(_previousInputWorldPose, inputPose);
                _currentInputAngularAcceleration = (inputW - _previousInputWorldW).magnitude / dt;
            }
            _peakInputStepDegrees = Mathf.Max(_peakInputStepDegrees, _currentInputStepDegrees);
            _peakInputAngularSpeed = Mathf.Max(_peakInputAngularSpeed, _currentInputAngularSpeed);
            _peakInputAngularAcceleration = Mathf.Max(_peakInputAngularAcceleration, _currentInputAngularAcceleration);
            // Diagnostic threshold mirrors the actual current phase budget when available. A > 80rad/s^2 is necessarily
            // outside both frozen Source/Recovery composite contracts; a >5deg fixed step is also an existing observer spike.
            if (_currentInputAngularAcceleration > 80.02f || _currentInputStepDegrees > 5f)
                _inputSpikeSteps++;
            _previousInputWorldPose = inputPose;
            _previousInputWorldW = inputW;
            _hasPreviousInput = true;
        }

        private void ClearCurrentOutput()
        {
            _currentInputGapDegrees = 0f;
            _currentOutputStepDegrees = 0f;
            _currentOutputAngularSpeed = 0f;
            _currentOutputAngularAcceleration = 0f;
            _currentMaxW = 0f;
            _currentMaxA = 0f;
            _currentLocalCompensationDegrees = 0f;
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

        private static Quaternion IntegrateRotation(Quaternion pose, Vector3 angularVelocityWorld, float dt)
        {
            float speed = angularVelocityWorld.magnitude;
            if (!Finite(speed) || speed < 1e-7f)
                return Normalize(pose);
            Vector3 axis = angularVelocityWorld / speed;
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
