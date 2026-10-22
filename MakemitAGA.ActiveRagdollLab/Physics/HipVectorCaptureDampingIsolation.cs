using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.2.69 high-error radial-away brake-floor causal A/B built on the v0.2.68 symmetric vector capture baseline.
    ///
    /// v0.2.68 real validation strongly reduced cross-target late capture, but a more fundamental same-side radial runaway
    /// remained: Hip error could grow from roughly 31deg to roughly 79deg while body motion stayed AWAY and the existing
    /// command still pointed TOWARD. v0.2.69 therefore keeps the entire v0.2.68 controller unchanged as A and changes
    /// exactly one authority-allocation rule in B: while capture is active, raw Hip error is at least 30deg, and physical
    /// radial body-minus-target speed is at least 20deg/s AWAY, the FINAL Hip acceleration must contain at least 40rad/s^2
    /// of corrective radial acceleration, while the existing final Hip ceiling remains 80rad/s^2.
    ///
    /// B decomposes physical body-minus-target relative angular velocity into:
    /// - radial: along the instantaneous raw Hip quaternion error axis;
    /// - tangential: orthogonal to that error axis.
    ///
    /// Radial AWAY/TOWARD symmetric extra-D and tangential damping remain byte-for-behavior v0.2.68 upstream. The new B
    /// floor is applied only after the normal relative-control + mechanical-parent-transport composition/clamp, so A follows
    /// the exact v0.2.68 command path. B does not add a second Rigidbody actuator: it only reallocates the already-available
    /// final acceleration vector inside the unchanged 80rad/s^2 ceiling, prioritizing the corrective radial component during
    /// the specific high-error AWAY gate. Orthogonal command is preserved as far as the 80rad/s^2 vector norm permits.
    ///
    /// Crucially, capture lifetime is physical-state based rather than animation-phase based: it may latch only from an
    /// authoritative PhysicalRecoveryPending state, but once latched it survives Recovery -> Source transitions until
    /// both raw Hip error and total physical relative angular speed are actually small. Source changes reference and
    /// authority as before; it does not erase an unresolved capture state.
    /// </summary>
    internal sealed class HipVectorCaptureDampingIsolation
    {
        internal const string Mode = "HIP_3D_VECTOR_CAPTURE_HIGH_ERROR_RADIAL_AWAY_BRAKE_FLOOR_CAUSAL_AB";
        internal const string LegacyArm = "VECTOR_OFF_V064_AXIAL_AWAY_BRAKING";
        internal const string StoppingGateArm = "V067_STOPPING_GATED_RADIAL_TOWARD_REFERENCE";
        internal const string SymmetricRadialArm = "A_V068_SYMMETRIC_RADIAL_CAPTURE_DAMPING";
        internal const string HighErrorAwayFloorArm = "B_V069_HIGH_ERROR_RADIAL_AWAY_BRAKE_FLOOR";

        private const float EnterErrorDegrees = 10f;
        private const float EnterMotionDegPerSec = 20f;
        private const float ExitMotionDegPerSec = 15f;
        private const float TangentialMeaningfulDegPerSec = 10f;
        private const float HighErrorAwayFloorErrorDegrees = 30f;
        private const float HighErrorAwayFloorSpeedDegPerSec = 20f;
        private const float HighErrorAwayBrakeFloorA = 40f;
        private const float FloorToleranceA = 0.01f;
        private const float Epsilon = 0.000001f;

        private bool _enabled;
        private bool _symmetricRadialEnabled;
        private bool _highErrorAwayBrakeFloorEnabled;
        private bool _captureActive;
        private bool _sourceCarryActive;
        private int _steps;
        private int _eligibleSteps;
        private int _activeSteps;
        private int _entries;
        private int _physicalExitCount;
        private int _sourceCarryEntries;
        private int _sourceCarrySteps;
        private int _radialAwayBrakeSteps;
        private int _radialTowardCaptureBrakeSteps;
        private int _tangentialDampingSteps;
        private int _supplementalClampSteps;
        private int _highErrorAwayFloorGateSteps;
        private int _highErrorAwayFloorRaisedSteps;
        private int _highErrorAwayFloorSatisfiedSteps;
        private int _highErrorAwayFloorUnsatisfiedSteps;
        private int _highErrorAwayFloorTangentialTrimSteps;
        private float _sumHighErrorAwayFloorDeltaA;
        private float _peakHighErrorAwayFloorDeltaA;
        private float _currentPreFloorRadialCommandA;
        private float _currentPostFloorRadialCommandA;
        private float _currentHighErrorAwayFloorDeltaA;
        private bool _currentHighErrorAwayFloorGate;
        private bool _currentHighErrorAwayFloorSatisfied;
        private Vector3 _currentErrorAxisWorld = Vector3.right;
        private float _sumSupplementalA;
        private float _sumRadialSupplementalA;
        private float _sumTangentialSupplementalA;
        private float _peakSupplementalA;
        private float _peakRadialSupplementalA;
        private float _peakTangentialSupplementalA;
        private float _peakAbsRadialWDegPerSec;
        private float _peakTangentialWDegPerSec;
        private float _peakStoppingAngleDeg;
        private float _peakStoppingRatio;
        private float _currentErrorDeg;
        private float _currentRadialWSignedDegPerSec;
        private float _currentTangentialWDegPerSec;
        private float _currentTotalWDegPerSec;
        private float _currentStoppingAngleDeg;
        private float _currentStoppingRatio;
        private float _currentRadialSupplementalA;
        private float _currentTangentialSupplementalA;
        private float _currentSupplementalA;
        private float _currentProductionMaxA;
        private float _currentEffectiveMaxA;
        private string _currentOrigin = "NONE";
        private string _lastEntry = "<none>";
        private string _lastExit = "<none>";
        private string _lastSourceCarry = "<none>";
        private string _abortEvidence = "<none>";

        internal bool Enabled => _enabled;
        internal bool SymmetricRadialEnabled => _symmetricRadialEnabled;
        internal bool HighErrorAwayBrakeFloorEnabled => _highErrorAwayBrakeFloorEnabled;
        internal string Arm => !_enabled ? LegacyArm : (!_symmetricRadialEnabled ? StoppingGateArm : (_highErrorAwayBrakeFloorEnabled ? HighErrorAwayFloorArm : SymmetricRadialArm));

        internal void SetEnabled(bool enabled, SingleLegChainReport report)
        {
            _enabled = enabled;
            if (!enabled)
            {
                _symmetricRadialEnabled = false;
                _highErrorAwayBrakeFloorEnabled = false;
            }
            ResetDiagnostics(report);
        }

        internal void SetSymmetricRadialEnabled(bool enabled, SingleLegChainReport report)
        {
            _symmetricRadialEnabled = enabled;
            if (!enabled) _highErrorAwayBrakeFloorEnabled = false;
            ResetDiagnostics(report);
        }

        internal void SetHighErrorAwayBrakeFloorEnabled(bool enabled, SingleLegChainReport report)
        {
            _highErrorAwayBrakeFloorEnabled = enabled;
            ResetDiagnostics(report);
        }

        internal void ResetDiagnostics(SingleLegChainReport report)
        {
            _captureActive = false;
            _sourceCarryActive = false;
            _steps = _eligibleSteps = _activeSteps = _entries = _physicalExitCount = 0;
            _sourceCarryEntries = _sourceCarrySteps = 0;
            _radialAwayBrakeSteps = _radialTowardCaptureBrakeSteps = _tangentialDampingSteps = 0;
            _supplementalClampSteps = 0;
            _highErrorAwayFloorGateSteps = _highErrorAwayFloorRaisedSteps = 0;
            _highErrorAwayFloorSatisfiedSteps = _highErrorAwayFloorUnsatisfiedSteps = 0;
            _highErrorAwayFloorTangentialTrimSteps = 0;
            _sumHighErrorAwayFloorDeltaA = _peakHighErrorAwayFloorDeltaA = 0f;
            _currentPreFloorRadialCommandA = _currentPostFloorRadialCommandA = _currentHighErrorAwayFloorDeltaA = 0f;
            _currentHighErrorAwayFloorGate = _currentHighErrorAwayFloorSatisfied = false;
            _currentErrorAxisWorld = Vector3.right;
            _sumSupplementalA = _sumRadialSupplementalA = _sumTangentialSupplementalA = 0f;
            _peakSupplementalA = _peakRadialSupplementalA = _peakTangentialSupplementalA = 0f;
            _peakAbsRadialWDegPerSec = _peakTangentialWDegPerSec = 0f;
            _peakStoppingAngleDeg = _peakStoppingRatio = 0f;
            _currentErrorDeg = _currentRadialWSignedDegPerSec = _currentTangentialWDegPerSec = _currentTotalWDegPerSec = 0f;
            _currentStoppingAngleDeg = _currentStoppingRatio = 0f;
            _currentRadialSupplementalA = _currentTangentialSupplementalA = _currentSupplementalA = 0f;
            _currentProductionMaxA = _currentEffectiveMaxA = 0f;
            _currentOrigin = "NONE";
            _lastEntry = _lastExit = _lastSourceCarry = "<none>";
            _abortEvidence = "<none>";
            CopyToReport(report);
        }

        /// <summary>
        /// Returns the supplemental relative-Hip acceleration. This class never mutates Rigidbody/Joint state.
        /// relativePhysicalWWorld must be physical body-minus-target angular velocity in world space.
        /// </summary>
        internal Vector3 ComposeSupplementalCapture(
            ActiveRagdollLabProfile profile,
            SingleLegChainReport report,
            bool sourceActive,
            int sourceId,
            Vector3 errorWorld,
            Vector3 relativePhysicalWWorld,
            float productionMaxA,
            out float effectiveMaxA)
        {
            _steps++;
            _currentProductionMaxA = Mathf.Max(0f, productionMaxA);
            _currentEffectiveMaxA = _currentProductionMaxA;
            _currentRadialSupplementalA = _currentTangentialSupplementalA = _currentSupplementalA = 0f;
            _currentStoppingAngleDeg = _currentStoppingRatio = 0f;
            effectiveMaxA = _currentProductionMaxA;

            float errorRad = errorWorld.magnitude;
            _currentErrorDeg = errorRad * Mathf.Rad2Deg;
            Vector3 axisWorld = errorRad > Epsilon ? errorWorld / errorRad : Vector3.right;
            float radialWRad = Vector3.Dot(relativePhysicalWWorld, axisWorld);
            Vector3 radialWWorld = axisWorld * radialWRad;
            Vector3 tangentialWWorld = relativePhysicalWWorld - radialWWorld;
            float tangentialWRad = tangentialWWorld.magnitude;
            float totalWRad = relativePhysicalWWorld.magnitude;
            _currentRadialWSignedDegPerSec = radialWRad * Mathf.Rad2Deg;
            _currentTangentialWDegPerSec = tangentialWRad * Mathf.Rad2Deg;
            _currentTotalWDegPerSec = totalWRad * Mathf.Rad2Deg;
            _currentOrigin = report != null ? (report.HipPhysicalRecoveryOrigin ?? "NONE") : "NONE";
            _currentErrorAxisWorld = axisWorld;
            _currentHighErrorAwayFloorGate = false;
            _currentHighErrorAwayFloorSatisfied = false;
            _currentPreFloorRadialCommandA = _currentPostFloorRadialCommandA = _currentHighErrorAwayFloorDeltaA = 0f;

            _peakAbsRadialWDegPerSec = Mathf.Max(_peakAbsRadialWDegPerSec, Mathf.Abs(_currentRadialWSignedDegPerSec));
            _peakTangentialWDegPerSec = Mathf.Max(_peakTangentialWDegPerSec, _currentTangentialWDegPerSec);

            if (!_enabled)
            {
                if (_captureActive) ExitCapture("ARM_DISABLED");
                CopyToReport(report);
                return Vector3.zero;
            }

            bool recoveryPending = report != null && report.HipPhysicalRecoveryPending;
            float recoveryExitDeg = profile != null ? Mathf.Max(0f, profile.LegHipRecoveryExitErrorDegrees) : 10f;

            // Entry is authorized only by the existing physical-recovery state. Once entered, phase labels cannot clear it.
            if (!_captureActive && recoveryPending &&
                _currentErrorDeg >= Mathf.Max(EnterErrorDegrees, recoveryExitDeg) &&
                _currentTotalWDegPerSec >= EnterMotionDegPerSec)
            {
                _captureActive = true;
                _entries++;
                _lastEntry = "origin=" + _currentOrigin + "/error=" + F(_currentErrorDeg) + "/totalWDegS=" + F(_currentTotalWDegPerSec);
            }

            if (!_captureActive)
            {
                CopyToReport(report);
                return Vector3.zero;
            }

            _eligibleSteps++;

            // Physical release: BOTH orientation error and residual relative angular speed must be captured.
            if (_currentErrorDeg <= recoveryExitDeg && _currentTotalWDegPerSec <= ExitMotionDegPerSec)
            {
                _physicalExitCount++;
                ExitCapture("PHYSICAL_ERROR_AND_SPEED_CAPTURED");
                CopyToReport(report);
                return Vector3.zero;
            }

            if (sourceActive)
            {
                if (!_sourceCarryActive)
                {
                    _sourceCarryActive = true;
                    _sourceCarryEntries++;
                    _lastSourceCarry = "source=" + sourceId + "/error=" + F(_currentErrorDeg) + "/rW=" + F(_currentRadialWSignedDegPerSec) + "/tW=" + F(_currentTangentialWDegPerSec);
                    Plugin.Logger?.LogWarning("[ARL][HIP_VECTOR_CAPTURE_SOURCE_CARRY] " + _lastSourceCarry);
                }
                _sourceCarrySteps++;
            }
            else
            {
                _sourceCarryActive = false;
            }

            float baselineMaxA = profile != null ? Mathf.Max(0f, profile.LegHipMaxAngularAcceleration) : _currentProductionMaxA;
            float recoveryMaxA = profile != null ? Mathf.Max(baselineMaxA, profile.LegHipRecoveryMaxAngularAcceleration) : _currentProductionMaxA;
            float supplementalBound = Mathf.Max(0f, recoveryMaxA - baselineMaxA);
            float damper = profile != null ? Mathf.Max(0f, profile.LegHipAngularDamper) : 0f;

            Vector3 requestedRadial = Vector3.zero;
            if (radialWRad < 0f)
            {
                // AWAY: same sign as v0.2.64 extra-D braking.
                requestedRadial = -radialWWorld * damper;
                if (requestedRadial.sqrMagnitude > Epsilon) _radialAwayBrakeSteps++;
            }
            else if (radialWRad > 0f)
            {
                if (_symmetricRadialEnabled)
                {
                    // v0.2.68 B: while capture is physically latched, TOWARD radial motion is damped just like AWAY.
                    // No theoretical stopping-angle gate: the v0.2.67 real run showed that gate was far too optimistic
                    // (12 gate activations versus 691 signed late-capture-risk samples).
                    requestedRadial = -radialWWorld * damper;
                    if (requestedRadial.sqrMagnitude > Epsilon) _radialTowardCaptureBrakeSteps++;
                }
                else
                {
                    // v0.2.67 A preserved byte-for-behavior: TOWARD braking starts only when the idealized
                    // supplemental-D stopping angle reaches the remaining raw quaternion error.
                    float designBrakeA = Mathf.Min(supplementalBound, damper * radialWRad);
                    if (designBrakeA > Epsilon)
                    {
                        float stoppingAngleRad = radialWRad * radialWRad / (2f * designBrakeA);
                        _currentStoppingAngleDeg = stoppingAngleRad * Mathf.Rad2Deg;
                        _currentStoppingRatio = _currentStoppingAngleDeg / Mathf.Max(0.1f, _currentErrorDeg);
                        _peakStoppingAngleDeg = Mathf.Max(_peakStoppingAngleDeg, _currentStoppingAngleDeg);
                        _peakStoppingRatio = Mathf.Max(_peakStoppingRatio, _currentStoppingRatio);
                        if (stoppingAngleRad >= errorRad)
                        {
                            requestedRadial = -radialWWorld * damper;
                            if (requestedRadial.sqrMagnitude > Epsilon) _radialTowardCaptureBrakeSteps++;
                        }
                    }
                }
            }

            Vector3 requestedTangential = Vector3.zero;
            if (_currentTangentialWDegPerSec >= TangentialMeaningfulDegPerSec)
            {
                requestedTangential = -tangentialWWorld * damper;
                if (requestedTangential.sqrMagnitude > Epsilon) _tangentialDampingSteps++;
            }

            Vector3 requested = requestedRadial + requestedTangential;
            Vector3 supplemental = requested;
            if (supplementalBound <= Epsilon)
            {
                supplemental = Vector3.zero;
            }
            else if (supplemental.magnitude > supplementalBound)
            {
                supplemental = supplemental.normalized * supplementalBound;
                _supplementalClampSteps++;
            }

            Vector3 appliedRadial = axisWorld * Vector3.Dot(supplemental, axisWorld);
            Vector3 appliedTangential = supplemental - appliedRadial;
            _currentRadialSupplementalA = appliedRadial.magnitude;
            _currentTangentialSupplementalA = appliedTangential.magnitude;
            _currentSupplementalA = supplemental.magnitude;
            _currentEffectiveMaxA = _currentSupplementalA > Epsilon ? Mathf.Max(_currentProductionMaxA, recoveryMaxA) : _currentProductionMaxA;
            effectiveMaxA = _currentEffectiveMaxA;

            if (_currentSupplementalA > Epsilon)
            {
                _activeSteps++;
                _sumSupplementalA += _currentSupplementalA;
                _sumRadialSupplementalA += _currentRadialSupplementalA;
                _sumTangentialSupplementalA += _currentTangentialSupplementalA;
                _peakSupplementalA = Mathf.Max(_peakSupplementalA, _currentSupplementalA);
                _peakRadialSupplementalA = Mathf.Max(_peakRadialSupplementalA, _currentRadialSupplementalA);
                _peakTangentialSupplementalA = Mathf.Max(_peakTangentialSupplementalA, _currentTangentialSupplementalA);
            }

            // v0.2.69 B gate is evaluated from the same raw physical phase-space used by the v0.2.68 vector controller.
            // The actual floor is applied later to the FINAL Hip acceleration after mechanical-parent transport/clamp.
            _currentHighErrorAwayFloorGate = _highErrorAwayBrakeFloorEnabled && _symmetricRadialEnabled &&
                _captureActive && _currentErrorDeg >= HighErrorAwayFloorErrorDegrees &&
                _currentRadialWSignedDegPerSec <= -HighErrorAwayFloorSpeedDegPerSec;

            CopyToReport(report);
            return supplemental;
        }

        /// <summary>
        /// v0.2.69 B: enforce a corrective radial floor on the FINAL Hip acceleration without raising its existing ceiling.
        /// This method only reshapes the already-centralized acceleration vector; it never calls Rigidbody/Joint APIs.
        /// </summary>
        internal Vector3 ApplyFinalHighErrorAwayBrakeFloor(Vector3 finalAccelerationWorld, float finalMaxA, SingleLegChainReport report)
        {
            _currentPreFloorRadialCommandA = Vector3.Dot(finalAccelerationWorld, _currentErrorAxisWorld);
            _currentPostFloorRadialCommandA = _currentPreFloorRadialCommandA;
            _currentHighErrorAwayFloorDeltaA = 0f;
            _currentHighErrorAwayFloorSatisfied = false;

            if (!_currentHighErrorAwayFloorGate)
            {
                CopyToReport(report);
                return finalAccelerationWorld;
            }

            _highErrorAwayFloorGateSteps++;
            float ceiling = Mathf.Max(0f, finalMaxA);
            float floorA = Mathf.Min(HighErrorAwayBrakeFloorA, ceiling);
            if (floorA <= Epsilon)
            {
                _highErrorAwayFloorUnsatisfiedSteps++;
                CopyToReport(report);
                return finalAccelerationWorld;
            }

            Vector3 axisWorld = _currentErrorAxisWorld.sqrMagnitude > Epsilon ? _currentErrorAxisWorld.normalized : Vector3.right;
            float radialA = Vector3.Dot(finalAccelerationWorld, axisWorld);
            Vector3 tangentialA = finalAccelerationWorld - axisWorld * radialA;

            Vector3 floored = finalAccelerationWorld;
            if (radialA < floorA)
            {
                float maxTangentialA = Mathf.Sqrt(Mathf.Max(0f, ceiling * ceiling - floorA * floorA));
                if (tangentialA.magnitude > maxTangentialA && tangentialA.sqrMagnitude > Epsilon)
                {
                    tangentialA = tangentialA.normalized * maxTangentialA;
                    _highErrorAwayFloorTangentialTrimSteps++;
                }
                floored = axisWorld * floorA + tangentialA;
                if (floored.magnitude > ceiling && ceiling > Epsilon)
                    floored = floored.normalized * ceiling;

                _currentHighErrorAwayFloorDeltaA = (floored - finalAccelerationWorld).magnitude;
                if (_currentHighErrorAwayFloorDeltaA > Epsilon)
                {
                    _highErrorAwayFloorRaisedSteps++;
                    _sumHighErrorAwayFloorDeltaA += _currentHighErrorAwayFloorDeltaA;
                    _peakHighErrorAwayFloorDeltaA = Mathf.Max(_peakHighErrorAwayFloorDeltaA, _currentHighErrorAwayFloorDeltaA);
                }
            }

            _currentPostFloorRadialCommandA = Vector3.Dot(floored, axisWorld);
            _currentHighErrorAwayFloorSatisfied = _currentPostFloorRadialCommandA + FloorToleranceA >= floorA;
            if (_currentHighErrorAwayFloorSatisfied) _highErrorAwayFloorSatisfiedSteps++;
            else _highErrorAwayFloorUnsatisfiedSteps++;

            CopyToReport(report);
            return floored;
        }

        internal void MarkAbort(string reason, SingleLegChainReport report)
        {
            _abortEvidence =
                "reason=" + (reason ?? "<none>") +
                "/arm=" + Arm +
                "/capture=" + _captureActive +
                "/sourceCarry=" + _sourceCarryActive +
                "/error=" + F(_currentErrorDeg) +
                "/W(r/t/total)=" + F(_currentRadialWSignedDegPerSec) + "/" + F(_currentTangentialWDegPerSec) + "/" + F(_currentTotalWDegPerSec) +
                "/stopDeg/ratio=" + F(_currentStoppingAngleDeg) + "/" + F(_currentStoppingRatio) +
                "/suppA=" + F(_currentSupplementalA) +
                "/floor(gate/pre/post/delta/satisfied)=" + _currentHighErrorAwayFloorGate + "/" + F(_currentPreFloorRadialCommandA) + "/" + F(_currentPostFloorRadialCommandA) + "/" + F(_currentHighErrorAwayFloorDeltaA) + "/" + _currentHighErrorAwayFloorSatisfied +
                "/maxA=" + F(_currentProductionMaxA) + "->" + F(_currentEffectiveMaxA);
            CopyToReport(report);
            Plugin.Logger?.LogWarning("[ARL][HIP_VECTOR_CAPTURE_ABORT_EVIDENCE] " + _abortEvidence);
            if (_highErrorAwayBrakeFloorEnabled)
                Plugin.Logger?.LogWarning("[ARL][HIP_RADIAL_AWAY_BRAKE_FLOOR_ABORT_EVIDENCE] " + GetHighErrorAwayBrakeFloorKey());
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            report.HipVectorCaptureMode = Mode;
            report.HipVectorCaptureArm = Arm;
            report.HipVectorCaptureEnabled = _enabled;
            report.HipVectorCaptureActive = _captureActive;
            report.HipVectorCaptureSourceCarryActive = _sourceCarryActive;
            report.HipVectorCaptureSteps = _steps;
            report.HipVectorCaptureEligibleSteps = _eligibleSteps;
            report.HipVectorCaptureActiveSteps = _activeSteps;
            report.HipVectorCaptureEntries = _entries;
            report.HipVectorCapturePhysicalExitCount = _physicalExitCount;
            report.HipVectorCaptureSourceCarryEntries = _sourceCarryEntries;
            report.HipVectorCaptureSourceCarrySteps = _sourceCarrySteps;
            report.HipVectorCaptureRadialAwayBrakeSteps = _radialAwayBrakeSteps;
            report.HipVectorCaptureRadialTowardCaptureBrakeSteps = _radialTowardCaptureBrakeSteps;
            report.HipVectorCaptureTangentialDampingSteps = _tangentialDampingSteps;
            report.HipVectorCaptureSupplementalClampSteps = _supplementalClampSteps;
            report.HipVectorCaptureHighErrorAwayBrakeFloorEnabled = _highErrorAwayBrakeFloorEnabled;
            report.HipVectorCaptureHighErrorAwayBrakeFloorGateSteps = _highErrorAwayFloorGateSteps;
            report.HipVectorCaptureHighErrorAwayBrakeFloorRaisedSteps = _highErrorAwayFloorRaisedSteps;
            report.HipVectorCaptureHighErrorAwayBrakeFloorSatisfiedSteps = _highErrorAwayFloorSatisfiedSteps;
            report.HipVectorCaptureHighErrorAwayBrakeFloorUnsatisfiedSteps = _highErrorAwayFloorUnsatisfiedSteps;
            report.HipVectorCaptureHighErrorAwayBrakeFloorTangentialTrimSteps = _highErrorAwayFloorTangentialTrimSteps;
            report.HipVectorCaptureCurrentHighErrorAwayBrakeFloorGate = _currentHighErrorAwayFloorGate;
            report.HipVectorCaptureCurrentHighErrorAwayBrakeFloorSatisfied = _currentHighErrorAwayFloorSatisfied;
            report.HipVectorCaptureCurrentPreFloorRadialCommandA = _currentPreFloorRadialCommandA;
            report.HipVectorCaptureCurrentPostFloorRadialCommandA = _currentPostFloorRadialCommandA;
            report.HipVectorCaptureCurrentHighErrorAwayBrakeFloorDeltaA = _currentHighErrorAwayFloorDeltaA;
            report.HipVectorCaptureMeanHighErrorAwayBrakeFloorDeltaA = Mean(_sumHighErrorAwayFloorDeltaA, _highErrorAwayFloorRaisedSteps);
            report.HipVectorCapturePeakHighErrorAwayBrakeFloorDeltaA = _peakHighErrorAwayFloorDeltaA;
            report.HipVectorCaptureHighErrorAwayBrakeFloorA = HighErrorAwayBrakeFloorA;
            report.HipVectorCaptureHighErrorAwayBrakeFloorKey = GetHighErrorAwayBrakeFloorKey();
            report.HipVectorCaptureCurrentErrorDegrees = _currentErrorDeg;
            report.HipVectorCaptureCurrentRadialWSignedDegPerSec = _currentRadialWSignedDegPerSec;
            report.HipVectorCaptureCurrentTangentialWDegPerSec = _currentTangentialWDegPerSec;
            report.HipVectorCaptureCurrentTotalWDegPerSec = _currentTotalWDegPerSec;
            report.HipVectorCaptureCurrentStoppingAngleDegrees = _currentStoppingAngleDeg;
            report.HipVectorCaptureCurrentStoppingRatio = _currentStoppingRatio;
            report.HipVectorCaptureCurrentRadialSupplementalA = _currentRadialSupplementalA;
            report.HipVectorCaptureCurrentTangentialSupplementalA = _currentTangentialSupplementalA;
            report.HipVectorCaptureCurrentSupplementalA = _currentSupplementalA;
            report.HipVectorCaptureCurrentProductionMaxA = _currentProductionMaxA;
            report.HipVectorCaptureCurrentEffectiveMaxA = _currentEffectiveMaxA;
            report.HipVectorCaptureMeanSupplementalA = Mean(_sumSupplementalA, _activeSteps);
            report.HipVectorCaptureMeanRadialSupplementalA = Mean(_sumRadialSupplementalA, _activeSteps);
            report.HipVectorCaptureMeanTangentialSupplementalA = Mean(_sumTangentialSupplementalA, _activeSteps);
            report.HipVectorCapturePeakSupplementalA = _peakSupplementalA;
            report.HipVectorCapturePeakRadialSupplementalA = _peakRadialSupplementalA;
            report.HipVectorCapturePeakTangentialSupplementalA = _peakTangentialSupplementalA;
            report.HipVectorCapturePeakAbsRadialWDegPerSec = _peakAbsRadialWDegPerSec;
            report.HipVectorCapturePeakTangentialWDegPerSec = _peakTangentialWDegPerSec;
            report.HipVectorCapturePeakStoppingAngleDegrees = _peakStoppingAngleDeg;
            report.HipVectorCapturePeakStoppingRatio = _peakStoppingRatio;
            report.HipVectorCaptureCurrentRecoveryOrigin = _currentOrigin;
            report.HipVectorCaptureLastEntryEvidence = _lastEntry;
            report.HipVectorCaptureLastExitReason = _lastExit;
            report.HipVectorCaptureLastSourceCarryEvidence = _lastSourceCarry;
            report.HipVectorCaptureAbortEvidence = _abortEvidence;
            report.HipVectorCaptureKey = GetKey(report);
        }

        internal string GetStatus(SingleLegChainReport report)
        {
            CopyToReport(report);
            return "ARL Hip 3D vector capture damping A/B | mode=" + Mode +
                   " | arm=" + Arm +
                   " | capture=" + _captureActive +
                   " | sourceCarry=" + _sourceCarryActive +
                   " | gate=enter:PhysicalRecoveryPending+error>=10deg+W>=20deg/s; exit:error<=recoveryExit+W<=15deg/s" +
                   " | radialArm=" + (!_symmetricRadialEnabled ? "v0.2.67 stopping-gated reference" : (_highErrorAwayBrakeFloorEnabled ? "B:v0.2.69 v0.2.68 symmetric + high-error AWAY final radial floor" : "A:v0.2.68 symmetric AWAY+TOWARD extra-D")) +
                   " | tangential=full relative-W damping while capture active" +
                   " | floorGate=B only: capture+error>=30deg+rW<=-20deg/s; finalCorrectiveRadial>=40rad/s^2" +
                   " | supplementalBound=recoveryMaxA-baselineMaxA | finalCeiling=existing recoveryMaxA80" +
                   " | current(error/rW/tW/suppA/floorGate/preFloor/postFloor/delta)=" + F(_currentErrorDeg) + "/" + F(_currentRadialWSignedDegPerSec) + "/" + F(_currentTangentialWDegPerSec) + "/" + F(_currentSupplementalA) + "/" + _currentHighErrorAwayFloorGate + "/" + F(_currentPreFloorRadialCommandA) + "/" + F(_currentPostFloorRadialCommandA) + "/" + F(_currentHighErrorAwayFloorDeltaA);
        }

        internal string GetHighErrorAwayBrakeFloorKey()
        {
            string primary;
            if (!_highErrorAwayBrakeFloorEnabled) primary = "A_V068_HIGH_ERROR_AWAY_FLOOR_OFF";
            else if (_highErrorAwayFloorGateSteps == 0) primary = "B_V069_HIGH_ERROR_AWAY_FLOOR_INSUFFICIENT_EXPOSURE";
            else if (_highErrorAwayFloorUnsatisfiedSteps > 0) primary = "B_V069_HIGH_ERROR_AWAY_FLOOR_NOT_ALWAYS_SATISFIED";
            else primary = "B_V069_HIGH_ERROR_AWAY_FINAL_RADIAL_FLOOR_ENFORCED";

            return "primary=" + primary +
                   " | arm=" + Arm +
                   " | gate=error>=30deg+rW<=-20deg/s+capture" +
                   " | floorA=" + F(HighErrorAwayBrakeFloorA) +
                   " | steps(gate/raised/satisfied/unsatisfied/tanTrim)=" + _highErrorAwayFloorGateSteps + "/" + _highErrorAwayFloorRaisedSteps + "/" + _highErrorAwayFloorSatisfiedSteps + "/" + _highErrorAwayFloorUnsatisfiedSteps + "/" + _highErrorAwayFloorTangentialTrimSteps +
                   " | delta(mean/peak)=" + F(Mean(_sumHighErrorAwayFloorDeltaA, _highErrorAwayFloorRaisedSteps)) + "/" + F(_peakHighErrorAwayFloorDeltaA) +
                   " | current(gate/pre/post/delta/satisfied)=" + _currentHighErrorAwayFloorGate + "/" + F(_currentPreFloorRadialCommandA) + "/" + F(_currentPostFloorRadialCommandA) + "/" + F(_currentHighErrorAwayFloorDeltaA) + "/" + _currentHighErrorAwayFloorSatisfied;
        }

        internal string GetKey(SingleLegChainReport report)
        {
            string primary;
            string armKey = !_symmetricRadialEnabled ? "V067_STOPPING_GATED" : (_highErrorAwayBrakeFloorEnabled ? "B_V069_HIGH_ERROR_AWAY_FLOOR" : "A_V068_SYMMETRIC");
            if (!_enabled) primary = "VECTOR_CAPTURE_OFF_V064_LEGACY_BASELINE";
            else if (!string.Equals(_abortEvidence, "<none>")) primary = armKey + "_VECTOR_CAPTURE_ABORTED_REQUIRES_CAUSAL_REVIEW";
            else if (_highErrorAwayBrakeFloorEnabled && _highErrorAwayFloorGateSteps > 0 && _highErrorAwayFloorUnsatisfiedSteps == 0) primary = "B_V069_HIGH_ERROR_AWAY_FINAL_RADIAL_FLOOR_ENFORCED_WITHIN_EXISTING_80_CEILING";
            else if (_highErrorAwayBrakeFloorEnabled && _highErrorAwayFloorGateSteps > 0) primary = "B_V069_HIGH_ERROR_AWAY_FLOOR_EXPOSED_WITH_UNSATISFIED_STEPS";
            else if (_sourceCarryEntries > 0 && _physicalExitCount > 0) primary = armKey + "_VECTOR_CAPTURE_CARRIED_ACROSS_SOURCE_AND_REACHED_PHYSICAL_EXIT";
            else if (_sourceCarryEntries > 0) primary = armKey + "_VECTOR_CAPTURE_SOURCE_CARRY_OBSERVED_NO_CONFIRMED_EXIT_YET";
            else if (_radialTowardCaptureBrakeSteps > 0 && _tangentialDampingSteps > 0) primary = armKey + "_RADIAL_AND_TANGENTIAL_DAMPING_ACTIVE";
            else primary = armKey + "_VECTOR_CAPTURE_INSUFFICIENT_EXPOSURE";

            return "primary=" + primary +
                   " | arm=" + Arm +
                   " | steps(total/eligible/active)=" + _steps + "/" + _eligibleSteps + "/" + _activeSteps +
                   " | capture(entries/exits/sourceCarryEntries/sourceCarrySteps)=" + _entries + "/" + _physicalExitCount + "/" + _sourceCarryEntries + "/" + _sourceCarrySteps +
                   " | actions(radialAway/radialTowardBrake/tangential/clamp)=" + _radialAwayBrakeSteps + "/" + _radialTowardCaptureBrakeSteps + "/" + _tangentialDampingSteps + "/" + _supplementalClampSteps +
                   " | floor(gate/raised/satisfied/unsatisfied/tanTrim,meanDelta/peakDelta,currentPre/post/delta)=" + _highErrorAwayFloorGateSteps + "/" + _highErrorAwayFloorRaisedSteps + "/" + _highErrorAwayFloorSatisfiedSteps + "/" + _highErrorAwayFloorUnsatisfiedSteps + "/" + _highErrorAwayFloorTangentialTrimSteps + "," + F(Mean(_sumHighErrorAwayFloorDeltaA, _highErrorAwayFloorRaisedSteps)) + "/" + F(_peakHighErrorAwayFloorDeltaA) + "," + F(_currentPreFloorRadialCommandA) + "/" + F(_currentPostFloorRadialCommandA) + "/" + F(_currentHighErrorAwayFloorDeltaA) +
                   " | supplemental(mean/peak,radialMean/peak,tanMean/peak)=" + F(Mean(_sumSupplementalA, _activeSteps)) + "/" + F(_peakSupplementalA) + "," + F(Mean(_sumRadialSupplementalA, _activeSteps)) + "/" + F(_peakRadialSupplementalA) + "," + F(Mean(_sumTangentialSupplementalA, _activeSteps)) + "/" + F(_peakTangentialSupplementalA) +
                   " | peaks(rW/tW/stopDeg/stopRatio)=" + F(_peakAbsRadialWDegPerSec) + "/" + F(_peakTangentialWDegPerSec) + "/" + F(_peakStoppingAngleDeg) + "/" + F(_peakStoppingRatio) +
                   " | lastEntry=" + _lastEntry +
                   " | lastCarry=" + _lastSourceCarry +
                   " | lastExit=" + _lastExit +
                   " | abort=" + _abortEvidence;
        }

        private void ExitCapture(string reason)
        {
            _captureActive = false;
            _sourceCarryActive = false;
            _lastExit = reason ?? "<none>";
        }

        private static float Mean(float sum, int count) => count > 0 ? sum / count : 0f;
        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
