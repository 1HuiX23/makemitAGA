using System;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.53 production state — exact v0.2.44/v0.2.51 physical-state-aware recovery-continuous governor.
    /// v0.2.52's post-Source 30deg recovery lead experiment is deliberately removed; all active phases again use
    /// LegHipRecoveryExitErrorDegrees (10deg default) as the physical lead envelope.
    ///
    /// v0.2.43 proved that replaying a Render packet over its historical packet interval reduces the one-step
    /// packet edge but creates reference lag and releases too early when Source Turn closes. This governor instead
    /// treats the newest Render Hip-local pose as the desired destination and generates a FixedUpdate trajectory that
    /// is constrained by CURRENT physical Hip state and CURRENT controller authority.
    ///
    /// Lifecycle is not tied to SourceTurn falling false. Once activated it remains active across physical recovery,
    /// control-frame recovery and stacked source events. It releases only after the physical Hip is healthy AND the
    /// governed pose/W have converged to the raw pose/W for the existing control-frame stable-hold window.
    ///
    /// No new profile knobs are introduced. Existing production semantics are reused:
    /// - physical lead envelope = LegHipRecoveryExitErrorDegrees (10 deg by default);
    /// - release pose/W/hold = control-frame recovery exit 1.5 deg / 0.5 rad/s / 0.10 s;
    /// - reference acceleration budget = 50% of current Hip MaxA, reserving the other half for tracking correction;
    /// - Render prediction validity ceiling = existing 0.20 s source-sample validity ceiling.
    ///
    /// Healthy Normal bypasses the governor exactly.
    /// v0.2.102 removes the completed late-Governor counterfactual/feasibility instrumentation; this class now
    /// contains only the frozen production governor and cheap branch-state mirrors used by status/reporting.
    /// </summary>
    internal sealed class HipFeasibilityReferenceGovernor
    {
        public const string ModeName = "PHYSICAL_RECOVERY_CONTINUOUS_FEASIBILITY_LIMITED_HIP_REFERENCE_GOVERNOR";
        private const float AuthorityReserveFraction = 0.50f;
        private const float MaxPredictionAgeSeconds = 0.20f;
        private const float ContractTolerance = 0.02f;

        private Quaternion _previousRawPacketPose = Quaternion.identity;
        private Quaternion _latestRawPacketPose = Quaternion.identity;
        private int _latestRawPacketSequence;
        private bool _hasPreviousRawPacket;
        private bool _hasLatestRawPacket;

        private bool _active;
        private Quaternion _outputPose = Quaternion.identity;
        private Vector3 _outputAngularVelocityLocal;
        private Quaternion _lastBypassPose = Quaternion.identity;
        private Vector3 _lastBypassAngularVelocityLocal;
        private bool _hasBypassState;
        private float _releaseStableSeconds;

        private int _activeFixedSteps;
        private int _normalBypassFixedSteps;
        private int _activationCount;
        private int _releaseCount;
        private int _leadCappedFixedSteps;
        // v0.2.98 read-only exact constraint-state diagnostics. These counters/flags mirror branches already
        // executed by the production governor; they never participate in target generation or release decisions.
        private int _accelerationLimitedFixedSteps;
        private int _brakingSpeedLimitedFixedSteps;
        private bool _leadCappedThisStep;
        private bool _accelerationLimitedThisStep;
        private bool _brakingSpeedLimitedThisStep;
        private bool _releaseEligibleThisStep;
        private float _currentPredictedRawAdvanceDegrees;
        private int _predictedRawFixedSteps;
        private int _sourceActiveFixedSteps;
        private int _physicalRecoveryFixedSteps;
        private int _controlFrameActiveFixedSteps;
        private float _peakRawPacketDeltaDegrees;
        private float _peakGovernedFixedStepDegrees;
        private float _peakRawToGovernedGapDegrees;
        private float _peakPhysicalLeadDegrees;
        private float _peakRawPhysicalErrorDegrees;
        private float _peakGovernedAngularSpeed;
        private float _peakGovernedAngularAcceleration;
        private float _peakAllowedAngularAcceleration;
        private float _peakAccelerationRatio;
        private float _peakPredictedRawAdvanceDegrees;
        private float _peakReleasePoseJumpDegrees;
        private float _peakReleaseAngularVelocityJump;
        private bool _accelerationContractSatisfied = true;
        private string _activationReason = "NONE";
        private string _lastEvidence = "<none>";

        public bool Active => _active;
        public bool OutputValid => Finite(_outputPose) && Finite(_outputAngularVelocityLocal);
        public Quaternion OutputPose => _outputPose;
        public Vector3 OutputAngularVelocityLocal => _outputAngularVelocityLocal;
        public bool AccelerationContractSatisfied => _accelerationContractSatisfied;
        public void Reset(Quaternion rawPose, Vector3 rawAngularVelocityLocal, SingleLegChainReport report)
        {
            rawPose = Finite(rawPose) ? Normalize(rawPose) : Quaternion.identity;
            Vector3 safeW = Finite(rawAngularVelocityLocal) ? rawAngularVelocityLocal : Vector3.zero;
            _active = false;
            _outputPose = rawPose;
            _outputAngularVelocityLocal = safeW;
            _lastBypassPose = rawPose;
            _lastBypassAngularVelocityLocal = safeW;
            _hasBypassState = true;
            _releaseStableSeconds = 0f;
            ResetDiagnostics();
            CopyToReport(report, 0f, 0f, 0f, 0f);
        }

        public void ResetDiagnostics()
        {
            _activeFixedSteps = 0;
            _normalBypassFixedSteps = 0;
            _activationCount = 0;
            _releaseCount = 0;
            _leadCappedFixedSteps = 0;
            _accelerationLimitedFixedSteps = 0;
            _brakingSpeedLimitedFixedSteps = 0;
            _leadCappedThisStep = false;
            _accelerationLimitedThisStep = false;
            _brakingSpeedLimitedThisStep = false;
            _releaseEligibleThisStep = false;
            _currentPredictedRawAdvanceDegrees = 0f;
            _predictedRawFixedSteps = 0;
            _sourceActiveFixedSteps = 0;
            _physicalRecoveryFixedSteps = 0;
            _controlFrameActiveFixedSteps = 0;
            _peakRawPacketDeltaDegrees = 0f;
            _peakGovernedFixedStepDegrees = 0f;
            _peakRawToGovernedGapDegrees = 0f;
            _peakPhysicalLeadDegrees = 0f;
            _peakRawPhysicalErrorDegrees = 0f;
            _peakGovernedAngularSpeed = 0f;
            _peakGovernedAngularAcceleration = 0f;
            _peakAllowedAngularAcceleration = 0f;
            _peakAccelerationRatio = 0f;
            _peakPredictedRawAdvanceDegrees = 0f;
            _peakReleasePoseJumpDegrees = 0f;
            _peakReleaseAngularVelocityJump = 0f;
            _accelerationContractSatisfied = true;
            _activationReason = "NONE";
            _lastEvidence = "<none>";
        }

        public void ObserveRenderPacket(Quaternion pose, int sequence, float sampleTime)
        {
            if (!Finite(pose) || sequence <= 0 || !Finite(sampleTime))
                return;
            pose = Normalize(pose);
            if (_hasLatestRawPacket && sequence == _latestRawPacketSequence)
                return;

            if (_hasLatestRawPacket)
            {
                _previousRawPacketPose = _latestRawPacketPose;
                _hasPreviousRawPacket = true;
            }

            _latestRawPacketPose = pose;
            _latestRawPacketSequence = sequence;
            _hasLatestRawPacket = true;

            if (_hasPreviousRawPacket)
            {
                float rawPacketDelta = Quaternion.Angle(_previousRawPacketPose, _latestRawPacketPose);
                _peakRawPacketDeltaDegrees = Mathf.Max(_peakRawPacketDeltaDegrees, rawPacketDelta);
            }
        }

        public void Step(
            ActiveRagdollLabProfile profile,
            bool sourceActive,
            bool controlFrameActive,
            bool physicalHipRecoveryPending,
            Quaternion rawPose,
            Vector3 rawAngularVelocityLocal,
            Quaternion physicalHipRelativePose,
            Vector3 physicalHipRelativeAngularVelocityLocal,
            float rawPhysicalHipErrorDegrees,
            int renderSequence,
            float renderSampleTime,
            float now,
            float fixedDeltaTime,
            float currentHipMaxAngularAcceleration,
            SingleLegChainReport report)
        {
            // Validate the frozen production inputs before executing the governor state update.
            if (profile == null || !Finite(rawPose) || !Finite(rawAngularVelocityLocal) ||
                !Finite(physicalHipRelativePose) || !Finite(physicalHipRelativeAngularVelocityLocal) ||
                !Finite(rawPhysicalHipErrorDegrees) || !Finite(now) || !Finite(fixedDeltaTime) || fixedDeltaTime <= 0.0001f ||
                !Finite(currentHipMaxAngularAcceleration) || currentHipMaxAngularAcceleration <= 0f)
            {
                _accelerationContractSatisfied = false;
                CopyToReport(report, 0f, 0f, 0f, 0f);
                return;
            }

            _leadCappedThisStep = false;
            _accelerationLimitedThisStep = false;
            _brakingSpeedLimitedThisStep = false;
            _releaseEligibleThisStep = false;
            _currentPredictedRawAdvanceDegrees = 0f;

            rawPose = Normalize(rawPose);
            physicalHipRelativePose = Normalize(physicalHipRelativePose);
            if (!_hasLatestRawPacket || renderSequence != _latestRawPacketSequence)
                ObserveRenderPacket(rawPose, renderSequence, renderSampleTime);

            bool hardHold = sourceActive || controlFrameActive || physicalHipRecoveryPending;
            if (sourceActive) _sourceActiveFixedSteps++;
            if (controlFrameActive) _controlFrameActiveFixedSteps++;
            if (physicalHipRecoveryPending) _physicalRecoveryFixedSteps++;
            _peakRawPhysicalErrorDegrees = Mathf.Max(_peakRawPhysicalErrorDegrees, rawPhysicalHipErrorDegrees);

            if (!_active && !hardHold)
            {
                BypassRaw(rawPose, rawAngularVelocityLocal);
                _normalBypassFixedSteps++;
                CopyToReport(report, 0f, 0f, 0f, 0f);
                return;
            }

            if (!_active)
            {
                _active = true;
                _activationCount++;
                _releaseStableSeconds = 0f;
                _activationReason = sourceActive ? "SOURCE_TURN" : (controlFrameActive ? "CONTROL_FRAME" : "PHYSICAL_HIP_RECOVERY");
                if (_hasBypassState)
                {
                    _outputPose = _lastBypassPose;
                    _outputAngularVelocityLocal = _lastBypassAngularVelocityLocal;
                }
                else
                {
                    _outputPose = physicalHipRelativePose;
                    _outputAngularVelocityLocal = physicalHipRelativeAngularVelocityLocal;
                }
            }

            float sourceAge = renderSequence > 0 && Finite(renderSampleTime)
                ? Mathf.Clamp(now - renderSampleTime, 0f, MaxPredictionAgeSeconds)
                : 0f;
            Quaternion predictedRawPose = PredictPose(rawPose, rawAngularVelocityLocal, sourceAge);
            float predictedAdvance = Quaternion.Angle(rawPose, predictedRawPose);
            _currentPredictedRawAdvanceDegrees = predictedAdvance;
            if (predictedAdvance > 0.0001f) _predictedRawFixedSteps++;
            _peakPredictedRawAdvanceDegrees = Mathf.Max(_peakPredictedRawAdvanceDegrees, predictedAdvance);

            // Existing physical recovery exit threshold is reused as a no-new-knob reference lead envelope.
            float maxLeadDegrees = Mathf.Max(0.25f, profile.LegHipRecoveryExitErrorDegrees);
            float physicalToDesired = Quaternion.Angle(physicalHipRelativePose, predictedRawPose);
            Quaternion feasibleGoal = predictedRawPose;
            if (physicalToDesired > maxLeadDegrees)
            {
                feasibleGoal = Normalize(Quaternion.Slerp(
                    physicalHipRelativePose,
                    predictedRawPose,
                    Mathf.Clamp01(maxLeadDegrees / Mathf.Max(0.0001f, physicalToDesired))));
                _leadCappedFixedSteps++;
                _leadCappedThisStep = true;
            }

            float allowedA = Mathf.Max(1f, currentHipMaxAngularAcceleration * AuthorityReserveFraction);
            _peakAllowedAngularAcceleration = Mathf.Max(_peakAllowedAngularAcceleration, allowedA);
            Vector3 previousW = _outputAngularVelocityLocal;
            Quaternion previousPose = _outputPose;

            Vector3 desiredW = ComputeBrakingDesiredAngularVelocity(_outputPose, feasibleGoal, allowedA, fixedDeltaTime, out _brakingSpeedLimitedThisStep);
            if (_brakingSpeedLimitedThisStep) _brakingSpeedLimitedFixedSteps++;
            Vector3 deltaW = desiredW - previousW;
            float maxDeltaW = allowedA * fixedDeltaTime;
            _accelerationLimitedThisStep = deltaW.magnitude > maxDeltaW && deltaW.sqrMagnitude > 1e-12f;
            if (_accelerationLimitedThisStep) _accelerationLimitedFixedSteps++;
            if (deltaW.magnitude > maxDeltaW && deltaW.sqrMagnitude > 1e-12f)
                deltaW = deltaW.normalized * maxDeltaW;
            Vector3 nextW = previousW + deltaW;
            Quaternion nextPose = IntegrateRotation(previousPose, nextW, fixedDeltaTime);

            float governedStep = Quaternion.Angle(previousPose, nextPose);
            float actualA = (nextW - previousW).magnitude / fixedDeltaTime;
            float aRatio = actualA / Mathf.Max(0.0001f, allowedA);
            _accelerationContractSatisfied = Finite(actualA) && actualA <= allowedA * (1f + ContractTolerance) + 0.0001f;

            _outputPose = nextPose;
            _outputAngularVelocityLocal = Finite(nextW) ? nextW : Vector3.zero;
            _activeFixedSteps++;

            float rawGap = Quaternion.Angle(_outputPose, rawPose);
            float physicalLead = Quaternion.Angle(physicalHipRelativePose, _outputPose);
            float wGap = (_outputAngularVelocityLocal - rawAngularVelocityLocal).magnitude;
            _peakGovernedFixedStepDegrees = Mathf.Max(_peakGovernedFixedStepDegrees, governedStep);
            _peakRawToGovernedGapDegrees = Mathf.Max(_peakRawToGovernedGapDegrees, rawGap);
            _peakPhysicalLeadDegrees = Mathf.Max(_peakPhysicalLeadDegrees, physicalLead);
            _peakGovernedAngularSpeed = Mathf.Max(_peakGovernedAngularSpeed, _outputAngularVelocityLocal.magnitude);
            _peakGovernedAngularAcceleration = Mathf.Max(_peakGovernedAngularAcceleration, actualA);
            _peakAccelerationRatio = Mathf.Max(_peakAccelerationRatio, aRatio);

            // Handoff must itself be dynamically feasible. Existing 1.5deg/0.5rad/s thresholds remain the outer
            // semantic gate, but the actual final raw handoff is also limited to what this reference trajectory could
            // legally absorb in one FixedUpdate under its current acceleration budget. This prevents a small-but-real
            // release W snap from bypassing the same A contract the governor enforces while active.
            float oneStepPoseCapacityDegrees =
                (_outputAngularVelocityLocal.magnitude * fixedDeltaTime + 0.5f * allowedA * fixedDeltaTime * fixedDeltaTime) * Mathf.Rad2Deg;
            float releasePoseThreshold = Mathf.Min(
                profile.LegHipControlFrameRecoveryExitAngleDegrees,
                Mathf.Max(0.01f, oneStepPoseCapacityDegrees));
            float releaseWThreshold = Mathf.Min(
                profile.LegHipControlFrameRecoveryExitAngularSpeed,
                allowedA * fixedDeltaTime);
            bool releaseEligible = !hardHold &&
                                   rawPhysicalHipErrorDegrees <= profile.LegHipRecoveryExitErrorDegrees &&
                                   rawGap <= releasePoseThreshold &&
                                   wGap <= releaseWThreshold;
            _releaseEligibleThisStep = releaseEligible;
            if (releaseEligible)
                _releaseStableSeconds += fixedDeltaTime;
            else
                _releaseStableSeconds = 0f;

            if (_releaseStableSeconds >= profile.LegHipControlFrameRecoveryStableHoldSeconds)
            {
                float releasePoseJump = Quaternion.Angle(_outputPose, rawPose);
                float releaseWJump = (_outputAngularVelocityLocal - rawAngularVelocityLocal).magnitude;
                _peakReleasePoseJumpDegrees = Mathf.Max(_peakReleasePoseJumpDegrees, releasePoseJump);
                _peakReleaseAngularVelocityJump = Mathf.Max(_peakReleaseAngularVelocityJump, releaseWJump);
                _releaseCount++;
                _active = false;
                _releaseStableSeconds = 0f;
                _outputPose = rawPose;
                _outputAngularVelocityLocal = rawAngularVelocityLocal;
                float releaseA = releaseWJump / fixedDeltaTime;
                _peakGovernedAngularAcceleration = Mathf.Max(_peakGovernedAngularAcceleration, releaseA);
                _peakAccelerationRatio = Mathf.Max(_peakAccelerationRatio, releaseA / Mathf.Max(0.0001f, allowedA));
                _accelerationContractSatisfied = _accelerationContractSatisfied &&
                    releaseA <= allowedA * (1f + ContractTolerance) + 0.0001f;
                _lastBypassPose = rawPose;
                _lastBypassAngularVelocityLocal = rawAngularVelocityLocal;
                _hasBypassState = true;
                _lastEvidence =
                    "RELEASE/render=" + renderSequence +
                    "/poseJumpDeg=" + F(releasePoseJump) +
                    "/wJump=" + F(releaseWJump) +
                    "/rawPhysicalErr=" + F(rawPhysicalHipErrorDegrees);
            }
            else
            {
                _lastEvidence =
                    "ACTIVE/reason=" + _activationReason +
                    "/render=" + renderSequence +
                    "/sourceAge=" + F(sourceAge) +
                    "/predAdvanceDeg=" + F(predictedAdvance) +
                    "/goalLeadDeg=" + F(physicalToDesired) +
                    "/outputLeadDeg=" + F(physicalLead) +
                    "/rawGapDeg=" + F(rawGap) +
                    "/stepDeg=" + F(governedStep) +
                    "/W=" + F(_outputAngularVelocityLocal.magnitude) +
                    "/A=" + F(actualA) + "/" + F(allowedA) +
                    "/releaseCandidate=" + F(_releaseStableSeconds);
            }

            CopyToReport(report, rawGap, physicalLead, actualA, allowedA);
        }

        public string GetKey()
        {
            return
                "mode=" + ModeName +
                " | active=" + _active +
                " | counts(active/bypass/activate/release/leadCap/accelLimit/brakeLimit/predict/source/recovery/frame)=" +
                _activeFixedSteps + "/" + _normalBypassFixedSteps + "/" + _activationCount + "/" + _releaseCount + "/" +
                _leadCappedFixedSteps + "/" + _accelerationLimitedFixedSteps + "/" + _brakingSpeedLimitedFixedSteps + "/" + _predictedRawFixedSteps + "/" + _sourceActiveFixedSteps + "/" +
                _physicalRecoveryFixedSteps + "/" + _controlFrameActiveFixedSteps +
                " | peak(rawPacket/governedStep/rawGap/physicalLead/rawPhysicalErr/W/A/allowedA/Aratio/predAdvance/releasePose/releaseW)=" +
                F(_peakRawPacketDeltaDegrees) + "/" + F(_peakGovernedFixedStepDegrees) + "/" + F(_peakRawToGovernedGapDegrees) + "/" +
                F(_peakPhysicalLeadDegrees) + "/" + F(_peakRawPhysicalErrorDegrees) + "/" + F(_peakGovernedAngularSpeed) + "/" +
                F(_peakGovernedAngularAcceleration) + "/" + F(_peakAllowedAngularAcceleration) + "/" + F(_peakAccelerationRatio) + "/" +
                F(_peakPredictedRawAdvanceDegrees) + "/" + F(_peakReleasePoseJumpDegrees) + "/" + F(_peakReleaseAngularVelocityJump) +
                " | currentConstraints(lead/accel/brake/releaseEligible/predAdvance)=" +
                _leadCappedThisStep + "/" + _accelerationLimitedThisStep + "/" + _brakingSpeedLimitedThisStep + "/" + _releaseEligibleThisStep + "/" + F(_currentPredictedRawAdvanceDegrees) +
                " | accelContract=" + _accelerationContractSatisfied +
                " | activation=" + (_activationReason ?? "NONE") +
                " | last=" + (_lastEvidence ?? "<none>");
        }

        private void BypassRaw(Quaternion rawPose, Vector3 rawAngularVelocityLocal)
        {
            _active = false;
            _outputPose = rawPose;
            _outputAngularVelocityLocal = rawAngularVelocityLocal;
            _lastBypassPose = rawPose;
            _lastBypassAngularVelocityLocal = rawAngularVelocityLocal;
            _hasBypassState = true;
            _releaseStableSeconds = 0f;
        }

        private static Vector3 ComputeBrakingDesiredAngularVelocity(Quaternion current, Quaternion target, float maxA, float dt, out bool brakingSpeedBoundActive)
        {
            brakingSpeedBoundActive = false;
            Quaternion delta = Normalize(target * Quaternion.Inverse(current));
            if (delta.w < 0f)
                delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);
            delta.ToAngleAxis(out float angleDeg, out Vector3 axis);
            if (!Finite(angleDeg) || !Finite(axis) || axis.sqrMagnitude < 1e-10f)
                return Vector3.zero;
            if (angleDeg > 180f) angleDeg -= 360f;
            float angleRad = Mathf.Abs(angleDeg) * Mathf.Deg2Rad;
            if (angleRad <= 1e-6f)
                return Vector3.zero;
            float stopSpeed = Mathf.Sqrt(Mathf.Max(0f, 2f * maxA * angleRad));
            float oneStepSpeed = angleRad / Mathf.Max(0.0001f, dt);
            brakingSpeedBoundActive = stopSpeed < oneStepSpeed;
            float speed = Mathf.Min(stopSpeed, oneStepSpeed);
            Vector3 direction = axis.normalized * Mathf.Sign(angleDeg);
            return direction * speed;
        }

        private static Quaternion PredictPose(Quaternion rawPose, Vector3 rawAngularVelocityLocal, float age)
        {
            if (!Finite(age) || age <= 0.0001f || !Finite(rawAngularVelocityLocal))
                return Normalize(rawPose);
            return IntegrateRotation(rawPose, rawAngularVelocityLocal, age);
        }

        private static Quaternion IntegrateRotation(Quaternion rotation, Vector3 angularVelocity, float dt)
        {
            if (!Finite(rotation) || !Finite(angularVelocity) || !Finite(dt) || dt <= 0f)
                return Normalize(rotation);
            float speed = angularVelocity.magnitude;
            if (speed <= 1e-7f)
                return Normalize(rotation);
            float angleDeg = speed * dt * Mathf.Rad2Deg;
            Quaternion delta = Quaternion.AngleAxis(angleDeg, angularVelocity / speed);
            return Normalize(delta * rotation);
        }

        private void CopyToReport(SingleLegChainReport report, float currentRawGap, float currentPhysicalLead, float currentA, float currentAllowedA)
        {
            if (report == null) return;
            report.HipReferenceGovernorMode = ModeName;
            report.HipReferenceGovernorKey = GetKey();
            report.HipReferenceGovernorActiveThisStep = _active;
            report.HipReferenceGovernorAccelerationContractSatisfiedThisStep = _accelerationContractSatisfied;
            report.HipReferenceGovernorActiveFixedSteps = _activeFixedSteps;
            report.HipReferenceGovernorNormalBypassFixedSteps = _normalBypassFixedSteps;
            report.HipReferenceGovernorActivationCount = _activationCount;
            report.HipReferenceGovernorReleaseCount = _releaseCount;
            report.HipReferenceGovernorLeadCappedFixedSteps = _leadCappedFixedSteps;
            report.HipReferenceGovernorAccelerationLimitedFixedSteps = _accelerationLimitedFixedSteps;
            report.HipReferenceGovernorBrakingSpeedLimitedFixedSteps = _brakingSpeedLimitedFixedSteps;
            report.HipReferenceGovernorLeadCappedThisStep = _leadCappedThisStep;
            report.HipReferenceGovernorAccelerationLimitedThisStep = _accelerationLimitedThisStep;
            report.HipReferenceGovernorBrakingSpeedLimitedThisStep = _brakingSpeedLimitedThisStep;
            report.HipReferenceGovernorReleaseEligibleThisStep = _releaseEligibleThisStep;
            report.HipReferenceGovernorCurrentPredictedRawAdvanceDegrees = _currentPredictedRawAdvanceDegrees;
            report.HipReferenceGovernorPredictedRawFixedSteps = _predictedRawFixedSteps;
            report.HipReferenceGovernorSourceActiveFixedSteps = _sourceActiveFixedSteps;
            report.HipReferenceGovernorPhysicalRecoveryFixedSteps = _physicalRecoveryFixedSteps;
            report.HipReferenceGovernorControlFrameActiveFixedSteps = _controlFrameActiveFixedSteps;
            report.HipReferenceGovernorPeakRawPacketDeltaDegrees = _peakRawPacketDeltaDegrees;
            report.HipReferenceGovernorPeakGovernedFixedStepDegrees = _peakGovernedFixedStepDegrees;
            report.HipReferenceGovernorPeakRawToGovernedGapDegrees = _peakRawToGovernedGapDegrees;
            report.HipReferenceGovernorPeakPhysicalLeadDegrees = _peakPhysicalLeadDegrees;
            report.HipReferenceGovernorPeakRawPhysicalErrorDegrees = _peakRawPhysicalErrorDegrees;
            report.HipReferenceGovernorPeakAngularSpeed = _peakGovernedAngularSpeed;
            report.HipReferenceGovernorPeakAngularAcceleration = _peakGovernedAngularAcceleration;
            report.HipReferenceGovernorPeakAllowedAngularAcceleration = _peakAllowedAngularAcceleration;
            report.HipReferenceGovernorPeakAccelerationRatio = _peakAccelerationRatio;
            report.HipReferenceGovernorPeakPredictedRawAdvanceDegrees = _peakPredictedRawAdvanceDegrees;
            report.HipReferenceGovernorPeakReleasePoseJumpDegrees = _peakReleasePoseJumpDegrees;
            report.HipReferenceGovernorPeakReleaseAngularVelocityJump = _peakReleaseAngularVelocityJump;
            report.HipReferenceGovernorCurrentRawGapDegrees = currentRawGap;
            report.HipReferenceGovernorCurrentPhysicalLeadDegrees = currentPhysicalLead;
            report.HipReferenceGovernorCurrentAngularAcceleration = currentA;
            report.HipReferenceGovernorCurrentAllowedAngularAcceleration = currentAllowedA;
            report.HipReferenceGovernorReleaseCandidateSeconds = _releaseStableSeconds;
            report.HipReferenceGovernorActivationReason = _activationReason;
            report.HipReferenceGovernorLastEvidence = _lastEvidence;
        }

        private static Quaternion Normalize(Quaternion q)
        {
            float m = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!Finite(m) || m <= 1e-8f) return Quaternion.identity;
            float inv = 1f / m;
            return new Quaternion(q.x * inv, q.y * inv, q.z * inv, q.w * inv);
        }

        private static bool Finite(float v) => !(float.IsNaN(v) || float.IsInfinity(v));
        private static bool Finite(Vector3 v) => Finite(v.x) && Finite(v.y) && Finite(v.z);
        private static bool Finite(Quaternion q) => Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w);
        private static string F(float v) => Finite(v) ? v.ToString("0.######") : "NaN";
    }
}
