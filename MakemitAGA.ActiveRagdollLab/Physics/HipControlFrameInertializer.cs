using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.56 — settled-origin strict mechanical command closure on top of the v0.2.51 activation continuity.
    ///
    /// While the strict branch is active, every FixedUpdate reconciles its pose/W origin to the actual PhysX-settled
    /// mechanical-parent proxy BEFORE the existing braking-aware ΔW limiter runs. This is not a second controller:
    /// the settled proxy is only the authoritative previous mechanical state, while this strict integrator remains
    /// the sole source of the next bounded W/A + rotation command. It closes the v0.2.55-observed gap where strict
    /// A=60 could be re-derived by the proxy pose delta as ~62rad/s² when the 3D rotation axis changed between steps.
    ///
    /// Fresh RAW→STRICT activation continuity from v0.2.51 is retained unchanged and remains separately diagnosed.
    ///
    /// Fresh Source activation now seeds the strict mechanical pose/W from the actual PhysX-settled Hip mechanical
    /// parent before running the existing braking-aware ΔW limiter. This removes the one-FixedUpdate handoff mismatch
    /// without loosening the 6/60 -> 8/80 contract, changing the 25-degree reference guard, or clamping proxy output.
    ///
    /// The reference branch used by Hip PD/governor retains the existing 25-degree lag guard. v0.2.48 adds a parallel
    /// STRICT mechanical branch that never consumes the lag-guard pose rebase. The Hip CharacterJoint mechanical proxy
    /// and parent-transport FF consume this strict branch, while Healthy motion remains raw/unfiltered after bounded
    /// mechanical recovery has genuinely converged.
    ///
    /// The virtual frame follows the real Pelvis with a braking-aware, acceleration-limited trajectory. v0.2.41 removes the
    /// old arrival snap and the post-integration hard speed normalization that could preserve |W|<=MaxW while silently
    /// violating |dW/dt|<=MaxA. The 25-degree lag guard remains unchanged for this single-variable production test.
    /// If an active interval inherits a pre-existing speed above MaxW from the deliberately raw healthy baseline, the
    /// integrator decelerates it at MaxA instead of fabricating an instantaneous speed clamp.
    /// </summary>
    internal sealed class HipControlFrameInertializer
    {
        public const string ModeName = "STRICT_WA_BRAKING_AWARE_SOURCE_TURN_CONTROL_FRAME_WITH_BOUNDED_LAG";
        public const string LagGuardRebaseModeName = "HIP_REFERENCE_LAG_GUARD_REBASE_ISOLATION_CAUSAL_AB";
        public const string LagGuardRebaseArmA = "A_V069_REFERENCE_LAG_GUARD_REBASE_ON";
        public const string LagGuardRebaseArmB = "B_V070_REFERENCE_LAG_GUARD_REBASE_SUPPRESSED";
        public const string IntegratorModeName = "STRICT_DELTA_W_BRAKING_AWARE_NO_ARRIVAL_SNAP";

        // v0.2.48 splits the strict physical branch from the guarded reference branch.
        // _rotation/_angularVelocityWorld are the guard-decoupled STRICT branch consumed by the Hip mechanical parent.
        // _referenceRotation is the post-guard reference branch consumed by Hip P/governor/composite attribution.
        private Quaternion _rotation = Quaternion.identity;
        private Quaternion _referenceRotation = Quaternion.identity;
        private Vector3 _angularVelocityWorld;
        private Vector3 _currentVirtualAngularAccelerationWorld;
        private bool _initialized;
        private bool _previousSourceActive;
        private bool _recoveryActive;
        private float _recoveryStartTime;
        private float _stableSeconds;
        private string _phase = "RAW_BASELINE";

        private int _baselineSteps;
        private int _sourceSteps;
        private int _recoverySteps;
        private int _speedLimitedSteps;
        private int _accelerationLimitedSteps;
        private int _recoveryStarted;
        private int _recoveryCompleted;
        private int _recoveryTimedOut;
        private int _recoveryCensoredByNewSource;
        private int _lagBoundCorrectionSteps;
        // v0.2.70 causal A/B: A preserves the v0.2.69 25deg reference-only lag rebase byte-for-byte;
        // B measures the same would-have correction but does not mutate _referenceRotation. Strict mechanical W/A
        // remains untouched in both arms.
        private bool _referenceLagGuardRebaseEnabled = true;
        private int _lagGuardSuppressedSteps;
        private float _currentSuppressedLagBoundCorrectionDegrees;
        private float _peakSuppressedLagBoundCorrectionDegrees;
        private float _totalSuppressedLagBoundCorrectionDegrees;
        private float _maxPreGuardFrameLagDegrees;
        private float _maxLagBoundCorrectionDegrees;
        private float _totalLagBoundCorrectionDegrees;
        private float _currentPreGuardFrameLagDegrees;
        private float _currentLagBoundCorrectionDegrees;
        private bool _lagBoundAppliedThisStep;
        private float _maxFrameLagDegrees;
        private float _maxVirtualAngularSpeed;
        private float _maxRealAngularSpeed;
        private float _maxAngularVelocityError;
        private float _maxStrictMechanicalLagDegrees;
        private bool _recoveryTimeoutReported;

        // v0.2.41 strict integrator contract diagnostics. These describe the stored virtual-frame W trajectory itself;
        // the composite UpperLeg world target can still contain additional dynamics from the live Hip-relative animation.
        private bool _strictIntegratorActiveThisStep;
        private bool _speedContractSatisfiedThisStep = true;
        private bool _accelerationContractSatisfiedThisStep = true;
        private bool _inheritedSpeedOverLimitThisStep;
        private float _currentIntegratorMaxSpeed;
        private float _currentIntegratorMaxAcceleration;
        private float _currentVirtualAngularAcceleration;
        private float _maxVirtualAngularAcceleration;
        private float _maxVirtualSpeedToLimitRatio;
        private float _maxVirtualAccelerationToLimitRatio;
        private int _brakingLimitedSteps;
        private int _inheritedSpeedOverLimitSteps;

        // v0.2.51 fresh RAW -> STRICT handoff diagnostics. These are state-continuity corrections, not authority.
        private bool _activationSyncAppliedThisStep;
        private int _activationSyncCount;
        private float _currentActivationSyncPoseDegrees;
        private float _currentActivationSyncAngularVelocityDelta;
        private float _peakActivationSyncPoseDegrees;
        private float _peakActivationSyncAngularVelocityDelta;

        // v0.2.56 active strict mechanical command-closure reconciliation. Unlike activation sync this may run every
        // strict FixedUpdate, but it only consumes the actual settled proxy pose/W as the previous-step origin. Raw
        // Pelvis state and the reference-side lag guard never enter this reconciliation.
        private bool _mechanicalClosureSyncAppliedThisStep;
        private int _mechanicalClosureSyncCount;
        private float _currentMechanicalClosureSyncPoseDegrees;
        private float _currentMechanicalClosureSyncAngularVelocityDelta;
        private float _peakMechanicalClosureSyncPoseDegrees;
        private float _peakMechanicalClosureSyncAngularVelocityDelta;

        // Reference-side output retains the 25deg lag guard.
        public Quaternion Rotation => _referenceRotation;
        public Vector3 AngularVelocityWorld => _angularVelocityWorld;

        // Mechanical-side output is the independent strict W/A branch and NEVER consumes the lag-guard pose rebase.
        public Quaternion StrictMechanicalRotation => _rotation;
        public Vector3 StrictMechanicalAngularVelocityWorld => _angularVelocityWorld;
        public Vector3 StrictMechanicalAngularAccelerationWorld => _currentVirtualAngularAccelerationWorld;
        public float StrictMechanicalCurrentMaxAngularAcceleration => _currentIntegratorMaxAcceleration;
        public bool Active => _phase == "SOURCE_TURN_INERTIALIZED" || _phase == "FRAME_RECOVERY_INERTIALIZED";
        public string Phase => _phase;
        public bool ReferenceLagGuardRebaseEnabled => _referenceLagGuardRebaseEnabled;
        public string ReferenceLagGuardRebaseArm => _referenceLagGuardRebaseEnabled ? LagGuardRebaseArmA : LagGuardRebaseArmB;

        public void Reset(Quaternion realPelvisRotation, Vector3 realPelvisAngularVelocity, ActiveRagdollLabProfile profile, SingleLegChainReport report)
        {
            _rotation = IsFinite(realPelvisRotation) ? Normalize(realPelvisRotation) : Quaternion.identity;
            _referenceRotation = _rotation;
            _angularVelocityWorld = IsFinite(realPelvisAngularVelocity) ? realPelvisAngularVelocity : Vector3.zero;
            _currentVirtualAngularAccelerationWorld = Vector3.zero;
            _initialized = true;
            _previousSourceActive = false;
            _recoveryActive = false;
            _recoveryStartTime = 0f;
            _stableSeconds = 0f;
            _phase = "RAW_BASELINE";
            _recoveryTimeoutReported = false;
            ResetDiagnostics();
            CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
        }

        public void ResetDiagnostics()
        {
            _baselineSteps = _sourceSteps = _recoverySteps = 0;
            _speedLimitedSteps = _accelerationLimitedSteps = 0;
            _recoveryStarted = _recoveryCompleted = _recoveryTimedOut = _recoveryCensoredByNewSource = 0;
            _lagBoundCorrectionSteps = 0;
            _lagGuardSuppressedSteps = 0;
            _currentSuppressedLagBoundCorrectionDegrees = 0f;
            _peakSuppressedLagBoundCorrectionDegrees = 0f;
            _totalSuppressedLagBoundCorrectionDegrees = 0f;
            _maxPreGuardFrameLagDegrees = 0f;
            _maxLagBoundCorrectionDegrees = 0f;
            _totalLagBoundCorrectionDegrees = 0f;
            _currentPreGuardFrameLagDegrees = 0f;
            _currentLagBoundCorrectionDegrees = 0f;
            _lagBoundAppliedThisStep = false;
            _maxFrameLagDegrees = 0f;
            _maxVirtualAngularSpeed = 0f;
            _maxRealAngularSpeed = 0f;
            _maxAngularVelocityError = 0f;
            _maxStrictMechanicalLagDegrees = 0f;
            _recoveryTimeoutReported = false;
            _strictIntegratorActiveThisStep = false;
            _speedContractSatisfiedThisStep = true;
            _accelerationContractSatisfiedThisStep = true;
            _inheritedSpeedOverLimitThisStep = false;
            _currentIntegratorMaxSpeed = 0f;
            _currentIntegratorMaxAcceleration = 0f;
            _currentVirtualAngularAcceleration = 0f;
            _currentVirtualAngularAccelerationWorld = Vector3.zero;
            _maxVirtualAngularAcceleration = 0f;
            _maxVirtualSpeedToLimitRatio = 0f;
            _maxVirtualAccelerationToLimitRatio = 0f;
            _brakingLimitedSteps = 0;
            _inheritedSpeedOverLimitSteps = 0;
            _activationSyncAppliedThisStep = false;
            _activationSyncCount = 0;
            _currentActivationSyncPoseDegrees = 0f;
            _currentActivationSyncAngularVelocityDelta = 0f;
            _peakActivationSyncPoseDegrees = 0f;
            _peakActivationSyncAngularVelocityDelta = 0f;
            _mechanicalClosureSyncAppliedThisStep = false;
            _mechanicalClosureSyncCount = 0;
            _currentMechanicalClosureSyncPoseDegrees = 0f;
            _currentMechanicalClosureSyncAngularVelocityDelta = 0f;
            _peakMechanicalClosureSyncPoseDegrees = 0f;
            _peakMechanicalClosureSyncAngularVelocityDelta = 0f;
        }

        public void Step(
            ActiveRagdollLabProfile profile,
            TwoBodyProxyReport torso,
            Quaternion realPelvisRotation,
            Vector3 realPelvisAngularVelocity,
            bool mechanicalActivationSyncEnabled,
            Quaternion settledMechanicalProxyRotation,
            Vector3 settledMechanicalProxyAngularVelocityWorld,
            float now,
            float dt,
            SingleLegChainReport report)
        {
            if (profile == null || !IsFinite(realPelvisRotation) || !IsFinite(realPelvisAngularVelocity) || !IsFinite(dt) || dt <= 0f)
                return;

            realPelvisRotation = Normalize(realPelvisRotation);
            if (!_initialized)
                Reset(realPelvisRotation, realPelvisAngularVelocity, profile, report);

            ClearActivationSyncStep();
            ClearMechanicalClosureSyncStep();

            if (!profile.LegHipControlFrameInertialization)
            {
                _rotation = realPelvisRotation;
                _referenceRotation = realPelvisRotation;
                _angularVelocityWorld = realPelvisAngularVelocity;
                _currentVirtualAngularAccelerationWorld = Vector3.zero;
                _previousSourceActive = torso != null && torso.SourceTurnEventActive;
                _recoveryActive = false;
                _stableSeconds = 0f;
                _phase = "RAW_BASELINE";
                _baselineSteps++;
                ClearStrictIntegratorStep();
                ClearLagBoundStep(realPelvisRotation);
                CaptureStats(realPelvisRotation, realPelvisAngularVelocity);
                CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
                return;
            }

            bool sourceActive = torso != null && torso.SourceTurnEventActive;
            if (sourceActive)
            {
                // v0.2.51: sync ONLY the fresh RAW_BASELINE -> SOURCE_TURN_INERTIALIZED handoff. If a new Source
                // event arrives while bounded recovery is already active, keep the existing strict trajectory and do
                // not introduce a pose/W rebase. REAL_PELVIS A/B arm also bypasses this path entirely.
                bool freshRawToStrictActivation = mechanicalActivationSyncEnabled &&
                    !_previousSourceActive && !_recoveryActive && _phase == "RAW_BASELINE";
                if (freshRawToStrictActivation)
                {
                    if (!IsFinite(settledMechanicalProxyRotation) || !IsFinite(settledMechanicalProxyAngularVelocityWorld))
                    {
                        // Fail closed through the existing v0.2.41 contract path rather than silently falling back to
                        // the mathematically newer RAW state. A bad settled handoff must never be hidden by a clamp.
                        _phase = "SOURCE_TURN_INERTIALIZED";
                        _strictIntegratorActiveThisStep = true;
                        _currentIntegratorMaxSpeed = Mathf.Max(0.1f, profile.LegHipControlFrameSourceMaxAngularSpeed);
                        _currentIntegratorMaxAcceleration = Mathf.Max(0.1f, profile.LegHipControlFrameSourceMaxAngularAcceleration);
                        _speedContractSatisfiedThisStep = false;
                        _accelerationContractSatisfiedThisStep = false;
                        CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
                        return;
                    }
                    ApplyActivationSync(settledMechanicalProxyRotation, settledMechanicalProxyAngularVelocityWorld);
                }
                else if (mechanicalActivationSyncEnabled)
                {
                    if (!TryApplyMechanicalClosureSync(settledMechanicalProxyRotation, settledMechanicalProxyAngularVelocityWorld))
                    {
                        MarkStrictMechanicalSyncFailure(
                            Mathf.Max(0.1f, profile.LegHipControlFrameSourceMaxAngularSpeed),
                            Mathf.Max(0.1f, profile.LegHipControlFrameSourceMaxAngularAcceleration));
                        _phase = "SOURCE_TURN_INERTIALIZED";
                        CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
                        return;
                    }
                }

                if (_recoveryActive) _recoveryCensoredByNewSource++;
                _recoveryActive = false;
                _stableSeconds = 0f;
                _previousSourceActive = true;
                _phase = "SOURCE_TURN_INERTIALIZED";
                _sourceSteps++;
                IntegrateToward(
                    realPelvisRotation,
                    realPelvisAngularVelocity,
                    Mathf.Max(0.1f, profile.LegHipControlFrameSourceMaxAngularSpeed),
                    Mathf.Max(0.1f, profile.LegHipControlFrameSourceMaxAngularAcceleration),
                    dt);
                // v0.2.48: reference branch starts from the strict branch each step, then the 25deg guard may rebase ONLY
                // the reference pose. The strict mechanical branch remains untouched for connectedBody + transport FF.
                _referenceRotation = _rotation;
                ApplyReferenceLagGuardArm(realPelvisRotation, profile.LegHipControlFrameMaxLagDegrees);
                CaptureStats(realPelvisRotation, realPelvisAngularVelocity);
                CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
                return;
            }

            if (_previousSourceActive)
            {
                _recoveryActive = true;
                _recoveryStartTime = now;
                _stableSeconds = 0f;
                _recoveryTimeoutReported = false;
                _recoveryStarted++;
            }
            _previousSourceActive = false;

            if (_recoveryActive)
            {
                _phase = "FRAME_RECOVERY_INERTIALIZED";
                _recoverySteps++;
                if (mechanicalActivationSyncEnabled &&
                    !TryApplyMechanicalClosureSync(settledMechanicalProxyRotation, settledMechanicalProxyAngularVelocityWorld))
                {
                    MarkStrictMechanicalSyncFailure(
                        Mathf.Max(0.1f, profile.LegHipControlFrameRecoveryMaxAngularSpeed),
                        Mathf.Max(0.1f, profile.LegHipControlFrameRecoveryMaxAngularAcceleration));
                    CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
                    return;
                }
                IntegrateToward(
                    realPelvisRotation,
                    realPelvisAngularVelocity,
                    Mathf.Max(0.1f, profile.LegHipControlFrameRecoveryMaxAngularSpeed),
                    Mathf.Max(0.1f, profile.LegHipControlFrameRecoveryMaxAngularAcceleration),
                    dt);
                _referenceRotation = _rotation;
                ApplyReferenceLagGuardArm(realPelvisRotation, profile.LegHipControlFrameMaxLagDegrees);

                // Mechanical recovery must converge on the STRICT branch before returning to raw baseline; otherwise the
                // CharacterJoint parent would receive a hidden snap when the guarded reference branch happens to be close.
                float lag = Quaternion.Angle(_rotation, realPelvisRotation);
                float wErr = (_angularVelocityWorld - realPelvisAngularVelocity).magnitude;
                if (lag <= profile.LegHipControlFrameRecoveryExitAngleDegrees &&
                    wErr <= profile.LegHipControlFrameRecoveryExitAngularSpeed)
                    _stableSeconds += dt;
                else
                    _stableSeconds = 0f;

                float elapsed = Mathf.Max(0f, now - _recoveryStartTime);
                if (_stableSeconds >= profile.LegHipControlFrameRecoveryStableHoldSeconds)
                {
                    _recoveryActive = false;
                    _recoveryCompleted++;
                    Snap(realPelvisRotation, realPelvisAngularVelocity);
                    ClearStrictIntegratorStep();
                    _phase = "RAW_BASELINE";
                }
                else if (elapsed >= profile.LegHipControlFrameRecoveryMaxSeconds && !_recoveryTimeoutReported)
                {
                    // v0.2.48: timeout is diagnostic only. Never snap the strict mechanical parent to raw merely because
                    // the old reference-side timer elapsed; keep bounded recovery until pose/W actually converge.
                    _recoveryTimedOut++;
                    _recoveryTimeoutReported = true;
                }

                CaptureStats(realPelvisRotation, realPelvisAngularVelocity);
                CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
                return;
            }

            // Healthy motion is deliberately unfiltered after BOTH guarded reference and strict mechanical branch have
            // completed bounded recovery.
            Snap(realPelvisRotation, realPelvisAngularVelocity);
            _phase = "RAW_BASELINE";
            _baselineSteps++;
            ClearStrictIntegratorStep();
            ClearLagBoundStep(realPelvisRotation);
            CaptureStats(realPelvisRotation, realPelvisAngularVelocity);
            CopyToReport(profile, realPelvisRotation, realPelvisAngularVelocity, report);
        }

        public string GetContractKey()
        {
            return
                "mode=" + IntegratorModeName +
                " | active=" + _strictIntegratorActiveThisStep +
                " | current(maxW/maxA/virtualW/virtualA/speedOK/accelOK/inherited)=" +
                    F(_currentIntegratorMaxSpeed) + "/" + F(_currentIntegratorMaxAcceleration) + "/" + F(_angularVelocityWorld.magnitude) + "/" + F(_currentVirtualAngularAcceleration) + "/" + _speedContractSatisfiedThisStep + "/" + _accelerationContractSatisfiedThisStep + "/" + _inheritedSpeedOverLimitThisStep +
                " | peak(virtualA/Wratio/Aratio)=" + F(_maxVirtualAngularAcceleration) + "/" + F(_maxVirtualSpeedToLimitRatio) + "/" + F(_maxVirtualAccelerationToLimitRatio) +
                " | steps(speedLimited/accelLimited/braking/inheritedOverSpeed)=" + _speedLimitedSteps + "/" + _accelerationLimitedSteps + "/" + _brakingLimitedSteps + "/" + _inheritedSpeedOverLimitSteps +
                " | activationSync(applied/count/poseDeg/wDelta/peakPose/peakW)=" + _activationSyncAppliedThisStep + "/" + _activationSyncCount + "/" + F(_currentActivationSyncPoseDegrees) + "/" + F(_currentActivationSyncAngularVelocityDelta) + "/" + F(_peakActivationSyncPoseDegrees) + "/" + F(_peakActivationSyncAngularVelocityDelta) +
                " | commandClosureSync(applied/count/poseDeg/wDelta/peakPose/peakW)=" + _mechanicalClosureSyncAppliedThisStep + "/" + _mechanicalClosureSyncCount + "/" + F(_currentMechanicalClosureSyncPoseDegrees) + "/" + F(_currentMechanicalClosureSyncAngularVelocityDelta) + "/" + F(_peakMechanicalClosureSyncPoseDegrees) + "/" + F(_peakMechanicalClosureSyncAngularVelocityDelta) +
                " | arrivalSnap=REMOVED" +
                " | actualWPostClamp=REMOVED" +
                " | lagGuard=" + ReferenceLagGuardRebaseArm + "_STRICT_MECHANICAL_BRANCH_DECOUPLED";
        }

        public string GetKey(ActiveRagdollLabProfile profile, SingleLegChainReport report)
        {
            return
                "mode=" + ModeName +
                " | enabled=" + (profile != null && profile.LegHipControlFrameInertialization) +
                " | phase=" + _phase +
                " | sourceLimit(w/a)=" + F(profile != null ? profile.LegHipControlFrameSourceMaxAngularSpeed : 0f) + "/" + F(profile != null ? profile.LegHipControlFrameSourceMaxAngularAcceleration : 0f) +
                " | recoveryLimit(w/a)=" + F(profile != null ? profile.LegHipControlFrameRecoveryMaxAngularSpeed : 0f) + "/" + F(profile != null ? profile.LegHipControlFrameRecoveryMaxAngularAcceleration : 0f) +
                " | lagBound=" + F(profile != null ? profile.LegHipControlFrameMaxLagDegrees : 0f) +
                " | lagGuardArm=" + ReferenceLagGuardRebaseArm +
                " | lagGuard(steps/maxPre/maxCorrection/totalCorrection)=" + _lagBoundCorrectionSteps + "/" + F(_maxPreGuardFrameLagDegrees) + "/" + F(_maxLagBoundCorrectionDegrees) + "/" + F(_totalLagBoundCorrectionDegrees) +
                " | lagGuardSuppressed(steps/current/peak/total)=" + _lagGuardSuppressedSteps + "/" + F(_currentSuppressedLagBoundCorrectionDegrees) + "/" + F(_peakSuppressedLagBoundCorrectionDegrees) + "/" + F(_totalSuppressedLagBoundCorrectionDegrees) +
                " | exit(angle/w/hold/max)=" + F(profile != null ? profile.LegHipControlFrameRecoveryExitAngleDegrees : 0f) + "/" + F(profile != null ? profile.LegHipControlFrameRecoveryExitAngularSpeed : 0f) + "/" + F(profile != null ? profile.LegHipControlFrameRecoveryStableHoldSeconds : 0f) + "/" + F(profile != null ? profile.LegHipControlFrameRecoveryMaxSeconds : 0f) +
                " | steps(base/source/recovery)=" + _baselineSteps + "/" + _sourceSteps + "/" + _recoverySteps +
                " | integrator=" + IntegratorModeName +
                " | limited(speed/accel/braking)=" + _speedLimitedSteps + "/" + _accelerationLimitedSteps + "/" + _brakingLimitedSteps +
                " | inheritedSpeedOverLimitSteps=" + _inheritedSpeedOverLimitSteps +
                " | contractCurrent(active/speedOK/accelOK/inherited/maxW/maxA/virtualW/virtualA)=" + _strictIntegratorActiveThisStep + "/" + _speedContractSatisfiedThisStep + "/" + _accelerationContractSatisfiedThisStep + "/" + _inheritedSpeedOverLimitThisStep + "/" + F(_currentIntegratorMaxSpeed) + "/" + F(_currentIntegratorMaxAcceleration) + "/" + F(_angularVelocityWorld.magnitude) + "/" + F(_currentVirtualAngularAcceleration) +
                " | contractPeak(virtualA/Wratio/Aratio)=" + F(_maxVirtualAngularAcceleration) + "/" + F(_maxVirtualSpeedToLimitRatio) + "/" + F(_maxVirtualAccelerationToLimitRatio) +
                " | activationSync(applied/count/poseDeg/wDelta/peakPose/peakW)=" + _activationSyncAppliedThisStep + "/" + _activationSyncCount + "/" + F(_currentActivationSyncPoseDegrees) + "/" + F(_currentActivationSyncAngularVelocityDelta) + "/" + F(_peakActivationSyncPoseDegrees) + "/" + F(_peakActivationSyncAngularVelocityDelta) +
                " | commandClosureSync(applied/count/poseDeg/wDelta/peakPose/peakW)=" + _mechanicalClosureSyncAppliedThisStep + "/" + _mechanicalClosureSyncCount + "/" + F(_currentMechanicalClosureSyncPoseDegrees) + "/" + F(_currentMechanicalClosureSyncAngularVelocityDelta) + "/" + F(_peakMechanicalClosureSyncPoseDegrees) + "/" + F(_peakMechanicalClosureSyncAngularVelocityDelta) +
                " | recovery(start/complete/timeout/censored)=" + _recoveryStarted + "/" + _recoveryCompleted + "/" + _recoveryTimedOut + "/" + _recoveryCensoredByNewSource +
                " | peak(frameLag/strictMechanicalLag/virtualW/realW/wErr)=" + F(_maxFrameLagDegrees) + "/" + F(_maxStrictMechanicalLagDegrees) + "/" + F(_maxVirtualAngularSpeed) + "/" + F(_maxRealAngularSpeed) + "/" + F(_maxAngularVelocityError) +
                " | current(frameLag/strictMechanicalLag/virtualW/virtualA/realW/wErr)=" + F(report != null ? report.HipControlFrameRotationLagDegrees : 0f) + "/" + F(report != null ? report.HipControlFrameStrictMechanicalRotationLagDegrees : 0f) + "/" + F(report != null ? report.HipControlFrameVirtualAngularSpeed : 0f) + "/" + F(report != null ? report.HipControlFrameVirtualAngularAcceleration : 0f) + "/" + F(report != null ? report.HipControlFrameRealPelvisAngularSpeed : 0f) + "/" + F(report != null ? report.HipControlFrameAngularVelocityError : 0f) +
                " | guardMechanicalDecoupled=True";
        }

        private void ClearActivationSyncStep()
        {
            _activationSyncAppliedThisStep = false;
            _currentActivationSyncPoseDegrees = 0f;
            _currentActivationSyncAngularVelocityDelta = 0f;
        }

        private bool TryApplyMechanicalClosureSync(Quaternion settledRotation, Vector3 settledAngularVelocityWorld)
        {
            if (!IsFinite(settledRotation) || !IsFinite(settledAngularVelocityWorld))
                return false;

            settledRotation = Normalize(settledRotation);
            _currentMechanicalClosureSyncPoseDegrees = Quaternion.Angle(_rotation, settledRotation);
            _currentMechanicalClosureSyncAngularVelocityDelta = (settledAngularVelocityWorld - _angularVelocityWorld).magnitude;
            _peakMechanicalClosureSyncPoseDegrees = Mathf.Max(_peakMechanicalClosureSyncPoseDegrees, _currentMechanicalClosureSyncPoseDegrees);
            _peakMechanicalClosureSyncAngularVelocityDelta = Mathf.Max(_peakMechanicalClosureSyncAngularVelocityDelta, _currentMechanicalClosureSyncAngularVelocityDelta);
            _mechanicalClosureSyncAppliedThisStep = true;
            _mechanicalClosureSyncCount++;

            // The settled kinematic proxy is the actual previous mechanical state. Re-anchoring STRICT here makes the
            // existing ΔW limiter the single truth for the next command instead of letting the proxy infer a second W/A
            // from two independently-evolved pose states.
            _rotation = settledRotation;
            _angularVelocityWorld = settledAngularVelocityWorld;
            return true;
        }

        private void MarkStrictMechanicalSyncFailure(float maxSpeed, float maxAcceleration)
        {
            _strictIntegratorActiveThisStep = true;
            _currentIntegratorMaxSpeed = maxSpeed;
            _currentIntegratorMaxAcceleration = maxAcceleration;
            _speedContractSatisfiedThisStep = false;
            _accelerationContractSatisfiedThisStep = false;
        }

        private void ClearMechanicalClosureSyncStep()
        {
            _mechanicalClosureSyncAppliedThisStep = false;
            _currentMechanicalClosureSyncPoseDegrees = 0f;
            _currentMechanicalClosureSyncAngularVelocityDelta = 0f;
        }

        private void ApplyActivationSync(Quaternion settledRotation, Vector3 settledAngularVelocityWorld)
        {
            settledRotation = Normalize(settledRotation);
            float poseCorrection = Quaternion.Angle(_rotation, settledRotation);
            float wCorrection = (_angularVelocityWorld - settledAngularVelocityWorld).magnitude;

            _rotation = settledRotation;
            _referenceRotation = settledRotation;
            _angularVelocityWorld = settledAngularVelocityWorld;
            _currentVirtualAngularAccelerationWorld = Vector3.zero;
            _currentVirtualAngularAcceleration = 0f;

            _activationSyncAppliedThisStep = true;
            _activationSyncCount++;
            _currentActivationSyncPoseDegrees = poseCorrection;
            _currentActivationSyncAngularVelocityDelta = wCorrection;
            _peakActivationSyncPoseDegrees = Mathf.Max(_peakActivationSyncPoseDegrees, poseCorrection);
            _peakActivationSyncAngularVelocityDelta = Mathf.Max(_peakActivationSyncAngularVelocityDelta, wCorrection);
        }

        private void ClearLagBoundStep(Quaternion realPelvisRotation)
        {
            _referenceRotation = _rotation;
            _currentPreGuardFrameLagDegrees = IsFinite(realPelvisRotation) ? Quaternion.Angle(_referenceRotation, realPelvisRotation) : 0f;
            _currentLagBoundCorrectionDegrees = 0f;
            _currentSuppressedLagBoundCorrectionDegrees = 0f;
            _lagBoundAppliedThisStep = false;
        }

        public void SetReferenceLagGuardRebaseEnabled(bool enabled)
        {
            _referenceLagGuardRebaseEnabled = enabled;
            _currentSuppressedLagBoundCorrectionDegrees = 0f;
        }

        public string GetReferenceLagGuardRebaseKey()
        {
            return
                "mode=" + LagGuardRebaseModeName +
                " | arm=" + ReferenceLagGuardRebaseArm +
                " | enabled=" + _referenceLagGuardRebaseEnabled +
                " | applied(steps/peak/total)=" + _lagBoundCorrectionSteps + "/" + F(_maxLagBoundCorrectionDegrees) + "/" + F(_totalLagBoundCorrectionDegrees) +
                " | suppressed(steps/current/peak/total)=" + _lagGuardSuppressedSteps + "/" + F(_currentSuppressedLagBoundCorrectionDegrees) + "/" + F(_peakSuppressedLagBoundCorrectionDegrees) + "/" + F(_totalSuppressedLagBoundCorrectionDegrees) +
                " | preLag(current/peak)=" + F(_currentPreGuardFrameLagDegrees) + "/" + F(_maxPreGuardFrameLagDegrees) +
                " | strictMechanical=UNCHANGED_GUARD_DECOUPLED";
        }

        private void ApplyReferenceLagGuardArm(Quaternion targetRotation, float maxLagDegrees)
        {
            if (_referenceLagGuardRebaseEnabled)
            {
                EnforceLagBound(targetRotation, maxLagDegrees);
                _currentSuppressedLagBoundCorrectionDegrees = 0f;
                return;
            }

            // B arm: calculate exactly the counterfactual excess the production guard would have removed, but keep the
            // reference pose equal to the strict branch. This isolates only the reference-side rebase/correction.
            targetRotation = Normalize(targetRotation);
            maxLagDegrees = Mathf.Clamp(maxLagDegrees, 0.1f, 179f);
            float preGuardLag = Quaternion.Angle(_referenceRotation, targetRotation);
            _currentPreGuardFrameLagDegrees = preGuardLag;
            _currentLagBoundCorrectionDegrees = 0f;
            _lagBoundAppliedThisStep = false;
            _maxPreGuardFrameLagDegrees = Mathf.Max(_maxPreGuardFrameLagDegrees, preGuardLag);
            float wouldCorrection = IsFinite(preGuardLag) ? Mathf.Max(0f, preGuardLag - maxLagDegrees) : 0f;
            _currentSuppressedLagBoundCorrectionDegrees = wouldCorrection;
            if (wouldCorrection > 1e-4f)
            {
                _lagGuardSuppressedSteps++;
                _peakSuppressedLagBoundCorrectionDegrees = Mathf.Max(_peakSuppressedLagBoundCorrectionDegrees, wouldCorrection);
                _totalSuppressedLagBoundCorrectionDegrees += wouldCorrection;
            }
        }

        private void EnforceLagBound(Quaternion targetRotation, float maxLagDegrees)
        {
            targetRotation = Normalize(targetRotation);
            maxLagDegrees = Mathf.Clamp(maxLagDegrees, 0.1f, 179f);

            float preGuardLag = Quaternion.Angle(_referenceRotation, targetRotation);
            _currentPreGuardFrameLagDegrees = preGuardLag;
            _currentLagBoundCorrectionDegrees = 0f;
            _lagBoundAppliedThisStep = false;
            _maxPreGuardFrameLagDegrees = Mathf.Max(_maxPreGuardFrameLagDegrees, preGuardLag);

            if (!IsFinite(preGuardLag) || preGuardLag <= maxLagDegrees + 1e-4f)
                return;

            // v0.2.31 bounded rebase: keep the existing limited virtual angular velocity (D reference),
            // but correct only the EXCESS orientation lag so P never sees a parent frame more than MaxLag behind.
            // This deliberately does not fabricate a huge virtual angular-velocity spike from the rebase.
            Quaternion delta = targetRotation * Quaternion.Inverse(_referenceRotation);
            Vector3 deltaVector = QuaternionToAngularVector(delta);
            float deltaRadians = deltaVector.magnitude;
            if (!IsFinite(deltaVector) || deltaRadians <= 1e-7f)
                return;

            float correctionDegrees = Mathf.Max(0f, preGuardLag - maxLagDegrees);
            Vector3 axis = deltaVector / deltaRadians;
            Quaternion correction = Quaternion.AngleAxis(correctionDegrees, axis);
            _referenceRotation = Normalize(correction * _referenceRotation);

            _lagBoundAppliedThisStep = true;
            _lagBoundCorrectionSteps++;
            _currentLagBoundCorrectionDegrees = correctionDegrees;
            _maxLagBoundCorrectionDegrees = Mathf.Max(_maxLagBoundCorrectionDegrees, correctionDegrees);
            _totalLagBoundCorrectionDegrees += correctionDegrees;
        }

        private void IntegrateToward(Quaternion targetRotation, Vector3 targetAngularVelocity, float maxSpeed, float maxAcceleration, float dt)
        {
            targetRotation = Normalize(targetRotation);
            maxSpeed = Mathf.Max(0.1f, maxSpeed);
            maxAcceleration = Mathf.Max(0.1f, maxAcceleration);
            dt = Mathf.Max(1e-5f, dt);

            Vector3 previousW = _angularVelocityWorld;
            float previousSpeed = previousW.magnitude;
            _strictIntegratorActiveThisStep = true;
            _currentIntegratorMaxSpeed = maxSpeed;
            _currentIntegratorMaxAcceleration = maxAcceleration;
            _currentVirtualAngularAcceleration = 0f;
            _currentVirtualAngularAccelerationWorld = Vector3.zero;
            _speedContractSatisfiedThisStep = true;
            _accelerationContractSatisfiedThisStep = true;
            _inheritedSpeedOverLimitThisStep = previousSpeed > maxSpeed + 1e-4f;
            if (_inheritedSpeedOverLimitThisStep)
                _inheritedSpeedOverLimitSteps++;

            Quaternion delta = targetRotation * Quaternion.Inverse(_rotation);
            Vector3 deltaVector = QuaternionToAngularVector(delta);
            float remaining = deltaVector.magnitude;

            // Target W is only a moving-target feed term. It is itself speed-bounded; the actual virtual W is then
            // reached through the strict MaxA delta-W limiter below. Never hard-normalize actual W after integration.
            Vector3 boundedTargetW = targetAngularVelocity;
            if (boundedTargetW.magnitude > maxSpeed)
            {
                boundedTargetW = boundedTargetW.normalized * maxSpeed;
                _speedLimitedSteps++;
            }

            Vector3 desiredW = boundedTargetW;
            if (remaining > 1e-7f)
            {
                Vector3 axis = deltaVector / remaining;

                // Braking-aware relative closing speed. As the virtual frame approaches the moving real Pelvis,
                // the extra correction component falls as sqrt(2*a*theta), so the integrator decelerates BEFORE
                // arrival instead of overshooting and snapping the pose/velocity to the target.
                float directRelativeCatchupSpeed = remaining / dt;
                float brakingRelativeSpeed = Mathf.Sqrt(Mathf.Max(0f, 2f * maxAcceleration * remaining));
                float correctionSpeed = Mathf.Min(directRelativeCatchupSpeed, brakingRelativeSpeed);
                if (brakingRelativeSpeed + 1e-5f < directRelativeCatchupSpeed && brakingRelativeSpeed < maxSpeed - 1e-4f)
                    _brakingLimitedSteps++;

                desiredW += axis * correctionSpeed;
            }

            if (desiredW.magnitude > maxSpeed)
            {
                desiredW = desiredW.normalized * maxSpeed;
                _speedLimitedSteps++;
            }

            Vector3 deltaW = desiredW - previousW;
            float maxDeltaW = maxAcceleration * dt;
            if (deltaW.magnitude > maxDeltaW && maxDeltaW > 0f)
            {
                deltaW = deltaW.normalized * maxDeltaW;
                _accelerationLimitedSteps++;
            }

            _angularVelocityWorld = previousW + deltaW;
            _currentVirtualAngularAccelerationWorld = deltaW / dt;
            _currentVirtualAngularAcceleration = _currentVirtualAngularAccelerationWorld.magnitude;

            // A speed above MaxW is legal ONLY when inherited from the deliberately raw healthy baseline. In that
            // physically unavoidable case the new integrator must monotonically reduce it under MaxA; it must never
            // fabricate an instantaneous speed clamp. Once inside the envelope, the convex delta-W update keeps W
            // inside because desiredW is already speed-bounded.
            float currentSpeed = _angularVelocityWorld.magnitude;
            bool inheritedSpeedDecelerating = _inheritedSpeedOverLimitThisStep && currentSpeed <= previousSpeed + 1e-4f;
            _speedContractSatisfiedThisStep = currentSpeed <= maxSpeed + 1e-3f || inheritedSpeedDecelerating;
            _accelerationContractSatisfiedThisStep = _currentVirtualAngularAcceleration <= maxAcceleration + 0.05f;

            _maxVirtualAngularAcceleration = Mathf.Max(_maxVirtualAngularAcceleration, _currentVirtualAngularAcceleration);
            _maxVirtualSpeedToLimitRatio = Mathf.Max(_maxVirtualSpeedToLimitRatio, currentSpeed / maxSpeed);
            _maxVirtualAccelerationToLimitRatio = Mathf.Max(_maxVirtualAccelerationToLimitRatio, _currentVirtualAngularAcceleration / maxAcceleration);

            // v0.2.41: no arrival snap. The quaternion is advanced ONLY by the acceleration-limited stored W.
            // If a coarse step crosses the instantaneous target, the next error vector changes direction and the
            // braking-aware velocity target decelerates/reverses under the same MaxA contract.
            float stepAngle = currentSpeed * dt;
            if (stepAngle > 1e-7f)
            {
                Vector3 axis = _angularVelocityWorld / currentSpeed;
                Quaternion step = Quaternion.AngleAxis(stepAngle * Mathf.Rad2Deg, axis);
                _rotation = Normalize(step * _rotation);
            }
        }

        private void ClearStrictIntegratorStep()
        {
            _strictIntegratorActiveThisStep = false;
            _speedContractSatisfiedThisStep = true;
            _accelerationContractSatisfiedThisStep = true;
            _inheritedSpeedOverLimitThisStep = false;
            _currentIntegratorMaxSpeed = 0f;
            _currentIntegratorMaxAcceleration = 0f;
            _currentVirtualAngularAcceleration = 0f;
            _currentVirtualAngularAccelerationWorld = Vector3.zero;
        }

        private void Snap(Quaternion rotation, Vector3 angularVelocity)
        {
            _rotation = Normalize(rotation);
            _referenceRotation = _rotation;
            _angularVelocityWorld = angularVelocity;
            _currentVirtualAngularAccelerationWorld = Vector3.zero;
        }

        private void CaptureStats(Quaternion realRotation, Vector3 realAngularVelocity)
        {
            float lag = Quaternion.Angle(_referenceRotation, realRotation);
            float strictMechanicalLag = Quaternion.Angle(_rotation, realRotation);
            float vw = _angularVelocityWorld.magnitude;
            float rw = realAngularVelocity.magnitude;
            float we = (_angularVelocityWorld - realAngularVelocity).magnitude;
            _maxFrameLagDegrees = Mathf.Max(_maxFrameLagDegrees, lag);
            _maxStrictMechanicalLagDegrees = Mathf.Max(_maxStrictMechanicalLagDegrees, strictMechanicalLag);
            _maxVirtualAngularSpeed = Mathf.Max(_maxVirtualAngularSpeed, vw);
            _maxRealAngularSpeed = Mathf.Max(_maxRealAngularSpeed, rw);
            _maxAngularVelocityError = Mathf.Max(_maxAngularVelocityError, we);
        }

        private void CopyToReport(ActiveRagdollLabProfile profile, Quaternion realRotation, Vector3 realAngularVelocity, SingleLegChainReport report)
        {
            if (report == null) return;
            report.HipControlFrameMode = ModeName;
            report.HipControlFrameInertializationEnabled = profile != null && profile.LegHipControlFrameInertialization;
            report.HipControlFramePhase = _phase;
            report.HipControlFrameActive = Active;
            report.HipControlFrameRotationLagDegrees = IsFinite(realRotation) ? Quaternion.Angle(_referenceRotation, realRotation) : 0f;
            report.HipControlFrameStrictMechanicalAngularVelocityWorld = _angularVelocityWorld;
            report.HipControlFrameStrictMechanicalAngularAccelerationWorld = _currentVirtualAngularAccelerationWorld;
            report.HipControlFrameStrictMechanicalRotationLagDegrees = IsFinite(realRotation) ? Quaternion.Angle(_rotation, realRotation) : 0f;
            report.HipControlFramePeakStrictMechanicalRotationLagDegrees = _maxStrictMechanicalLagDegrees;
            report.HipControlFrameLagGuardDecoupledFromMechanical = true;
            report.HipControlFrameVirtualAngularSpeed = _angularVelocityWorld.magnitude;
            report.HipControlFrameRealPelvisAngularSpeed = IsFinite(realAngularVelocity) ? realAngularVelocity.magnitude : 0f;
            report.HipControlFrameAngularVelocityError = IsFinite(realAngularVelocity) ? (_angularVelocityWorld - realAngularVelocity).magnitude : 0f;
            report.HipControlFrameIntegratorMode = IntegratorModeName;
            report.HipControlFrameStrictIntegratorActiveThisStep = _strictIntegratorActiveThisStep;
            report.HipControlFrameSpeedContractSatisfiedThisStep = _speedContractSatisfiedThisStep;
            report.HipControlFrameAccelerationContractSatisfiedThisStep = _accelerationContractSatisfiedThisStep;
            report.HipControlFrameInheritedSpeedOverLimitThisStep = _inheritedSpeedOverLimitThisStep;
            report.HipControlFrameIntegratorCurrentMaxAngularSpeed = _currentIntegratorMaxSpeed;
            report.HipControlFrameIntegratorCurrentMaxAngularAcceleration = _currentIntegratorMaxAcceleration;
            report.HipControlFrameVirtualAngularAcceleration = _currentVirtualAngularAcceleration;
            report.HipControlFramePeakVirtualAngularAcceleration = _maxVirtualAngularAcceleration;
            report.HipControlFramePeakVirtualSpeedToLimitRatio = _maxVirtualSpeedToLimitRatio;
            report.HipControlFramePeakVirtualAccelerationToLimitRatio = _maxVirtualAccelerationToLimitRatio;
            report.HipControlFrameBrakingLimitedFixedSteps = _brakingLimitedSteps;
            report.HipControlFrameInheritedSpeedOverLimitFixedSteps = _inheritedSpeedOverLimitSteps;
            report.HipControlFrameActivationSyncAppliedThisStep = _activationSyncAppliedThisStep;
            report.HipControlFrameActivationSyncCount = _activationSyncCount;
            report.HipControlFrameActivationSyncPoseCorrectionDegrees = _currentActivationSyncPoseDegrees;
            report.HipControlFrameActivationSyncAngularVelocityCorrection = _currentActivationSyncAngularVelocityDelta;
            report.HipControlFramePeakActivationSyncPoseCorrectionDegrees = _peakActivationSyncPoseDegrees;
            report.HipControlFramePeakActivationSyncAngularVelocityCorrection = _peakActivationSyncAngularVelocityDelta;
            report.HipControlFrameMechanicalClosureSyncAppliedThisStep = _mechanicalClosureSyncAppliedThisStep;
            report.HipControlFrameMechanicalClosureSyncCount = _mechanicalClosureSyncCount;
            report.HipControlFrameMechanicalClosureSyncPoseCorrectionDegrees = _currentMechanicalClosureSyncPoseDegrees;
            report.HipControlFrameMechanicalClosureSyncAngularVelocityCorrection = _currentMechanicalClosureSyncAngularVelocityDelta;
            report.HipControlFramePeakMechanicalClosureSyncPoseCorrectionDegrees = _peakMechanicalClosureSyncPoseDegrees;
            report.HipControlFramePeakMechanicalClosureSyncAngularVelocityCorrection = _peakMechanicalClosureSyncAngularVelocityDelta;
            report.HipControlFrameMaxLagDegrees = profile != null ? profile.LegHipControlFrameMaxLagDegrees : 0f;
            report.HipControlFrameReferenceLagGuardRebaseMode = LagGuardRebaseModeName;
            report.HipControlFrameReferenceLagGuardRebaseArm = ReferenceLagGuardRebaseArm;
            report.HipControlFrameReferenceLagGuardRebaseEnabled = _referenceLagGuardRebaseEnabled;
            report.HipControlFrameLagGuardSuppressedFixedSteps = _lagGuardSuppressedSteps;
            report.HipControlFrameCurrentSuppressedLagBoundCorrectionDegrees = _currentSuppressedLagBoundCorrectionDegrees;
            report.HipControlFramePeakSuppressedLagBoundCorrectionDegrees = _peakSuppressedLagBoundCorrectionDegrees;
            report.HipControlFrameTotalSuppressedLagBoundCorrectionDegrees = _totalSuppressedLagBoundCorrectionDegrees;
            report.HipControlFramePreGuardRotationLagDegrees = _currentPreGuardFrameLagDegrees;
            report.HipControlFrameLagBoundAppliedThisStep = _lagBoundAppliedThisStep;
            report.HipControlFrameLagBoundCorrectionDegrees = _currentLagBoundCorrectionDegrees;
            report.HipControlFrameLagBoundCorrectionFixedSteps = _lagBoundCorrectionSteps;
            report.HipControlFramePeakPreGuardRotationLagDegrees = _maxPreGuardFrameLagDegrees;
            report.HipControlFramePeakLagBoundCorrectionDegrees = _maxLagBoundCorrectionDegrees;
            report.HipControlFrameTotalLagBoundCorrectionDegrees = _totalLagBoundCorrectionDegrees;
            report.HipControlFrameBaselineFixedSteps = _baselineSteps;
            report.HipControlFrameSourceFixedSteps = _sourceSteps;
            report.HipControlFrameRecoveryFixedSteps = _recoverySteps;
            report.HipControlFrameSpeedLimitedFixedSteps = _speedLimitedSteps;
            report.HipControlFrameAccelerationLimitedFixedSteps = _accelerationLimitedSteps;
            report.HipControlFrameRecoveryStarted = _recoveryStarted;
            report.HipControlFrameRecoveryCompleted = _recoveryCompleted;
            report.HipControlFrameRecoveryTimedOut = _recoveryTimedOut;
            report.HipControlFrameRecoveryCensoredByNewSource = _recoveryCensoredByNewSource;
            report.HipControlFramePeakRotationLagDegrees = _maxFrameLagDegrees;
            report.HipControlFramePeakVirtualAngularSpeed = _maxVirtualAngularSpeed;
            report.HipControlFramePeakRealPelvisAngularSpeed = _maxRealAngularSpeed;
            report.HipControlFramePeakAngularVelocityError = _maxAngularVelocityError;
        }

        private static Vector3 QuaternionToAngularVector(Quaternion q)
        {
            q = Normalize(q);
            if (q.w < 0f) q = new Quaternion(-q.x, -q.y, -q.z, -q.w);
            float w = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (!IsFinite(angle) || sinHalf < 1e-6f)
                return Vector3.zero;
            Vector3 axis = new Vector3(q.x / sinHalf, q.y / sinHalf, q.z / sinHalf);
            if (!IsFinite(axis) || axis.sqrMagnitude < 1e-12f)
                return Vector3.zero;
            return axis.normalized * angle;
        }

        private static Quaternion Normalize(Quaternion q)
        {
            float mag = Mathf.Sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
            if (mag <= 1e-8f || !IsFinite(mag)) return Quaternion.identity;
            float inv = 1f / mag;
            return new Quaternion(q.x*inv, q.y*inv, q.z*inv, q.w*inv);
        }

        private static bool IsFinite(Quaternion q) => IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
        private static bool IsFinite(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        private static bool IsFinite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string F(float v) => v.ToString("G6", System.Globalization.CultureInfo.InvariantCulture);
    }
}
