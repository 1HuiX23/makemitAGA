using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.3.63 no-CharacterJoint translation tether, promoted as the frozen free-space positional backend since v0.3.69.
    ///
    /// A = the v0.3.62 Dynamic Foot baseline: CharacterJoint owns anchor translation and passive angular limits.
    /// B = CharacterJoint completely removed. An ARL-owned bounded translation-only tether acts at the Foot COM, while the unchanged v0.3.58 source-bone relative Ankle PD remains the only deliberate angular actuator. v0.3.64 can add a separately selected measured Lower ankle-point transport feedforward before this tether's unchanged final clamp.
    ///
    /// The historical B arm was qualified and is now the frozen free-space production positional backend. It remains
    /// intentionally one-way at the translation layer: no equal/opposite linear force is applied to Lower. This preserves
    /// the causal topology that removed the native solver's currentForce x lever and currentTorque paths
    /// identified by v0.3.61-v0.3.62 instead of replacing them with a new hidden parent reaction.
    ///
    /// Outside the V04300 authorized-STANCE isolation, the tether retains its original translational point-PD. V04300 changes only the Foot rotational planar damping contribution; it adds no new actuator. v0.3.73 promotes the already-qualified conditional MaxA80 reserve into the frozen production law: ordinary samples remain MaxA60 and only the pre-registered capture-risk window may use MaxA80. It never calls AddTorque,
    /// AddForceAtPosition, MovePosition/MoveRotation, or writes angular velocity. The existing 30 mm ankle positional
    /// fail-close remains unchanged. With the CharacterJoint absent, B adds only a read-only angular excursion monitor
    /// plus a broader emergency hard envelope; it does not physically bounce the Foot off an angular limit.
    /// </summary>
    internal sealed class FootAnkleConstraintTopologyIsolation
    {
        internal const string Mode = "ANKLE_CHARACTERJOINT_VS_ARL_TRANSLATION_TETHER_CAUSAL_AB_V0363";
        internal const string ArmA = "A_V0362_CHARACTERJOINT_BASELINE";
        internal const string ArmB = "B_V0363_ARL_TRANSLATION_ONLY_TETHER";

        // Reuse the already qualified Hip/Knee translation-tether scale for a clean first topology test.
        internal const float TetherPositionSpring = 400f;
        internal const float TetherVelocityDamper = 40f;
        internal const float TetherMaxAcceleration = 60f;
        internal const float AnchorValidityMeters = 0.030f;

        // v0.3.57's 30/25/15 values remain a read-only reference envelope. With no native angular limiter in B,
        // use a broader emergency-only fail-close envelope derived from the observed v0.3.58-v0.3.62 excursion range.
        internal const float HardFlexionDegrees = 45f;
        internal const float HardAxialDegrees = 40f;
        internal const float HardInversionDegrees = 30f;
        internal const float HardTotalDegrees = 60f;

        private bool _armB;
        private readonly FootStancePlanarRotationalDampingIsolation _stancePlanarDamping = new FootStancePlanarRotationalDampingIsolation();
        private readonly FootAnkleTranslationCaptureAuthorityIsolation _captureAuthority = new FootAnkleTranslationCaptureAuthorityIsolation();
        private readonly FootAnkleTranslationPredictiveTailCaptureIsolation _predictiveTailCapture = new FootAnkleTranslationPredictiveTailCaptureIsolation();
        private bool _finite = true;
        private string _invalidReason;

        private bool _currentDynamic;
        private float _currentAnchorError;
        private float _currentVelocityError;
        private Vector3 _currentPositionErrorWorld;
        private Vector3 _currentVelocityErrorWorld;
        private Vector3 _currentCorrectionAccelerationWorld;
        private Vector3 _currentTransportAccelerationWorld;
        private Vector3 _currentRawAccelerationWorld;
        private Vector3 _currentAppliedAccelerationWorld;
        private bool _currentClamped;
        private Vector3 _currentPhysicalBindDeltaDegrees;
        private Vector3 _currentReferenceRatio;
        private Vector3 _currentHardRatio;
        private float _currentTotalAngleDegrees;

        private int _traceSamples;
        private int _traceDynamicSamples;
        private int _traceAppliedSamples;
        private int _traceInvalid;
        private int _traceClamped;
        private int _traceReferenceEnvelopeExceed;
        private int _traceHardEnvelopeExceed;
        private float _traceAnchorSum;
        private float _traceAnchorMax;
        private float _traceVelocityErrorSum;
        private float _traceVelocityErrorMax;
        private float _traceRawAccelerationSum;
        private float _traceRawAccelerationMax;
        private float _traceAppliedAccelerationSum;
        private float _traceAppliedAccelerationMax;
        private Vector3 _tracePhysicalAxisAbsMax;
        private Vector3 _traceHardRatioMax;
        private float _traceTotalAngleMax;

        internal string Arm => _armB ? ArmB : ArmA;
        internal bool CharacterJointArm => !_armB;
        internal bool IsProductionSelectorArm => _armB;
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;
        internal string CaptureAuthorityArm => _captureAuthority.Arm;
        internal bool CaptureAuthorityProduction => _captureAuthority.IsProductionPolicy;
        internal string PredictiveTailCaptureArm => _predictiveTailCapture.Arm;
        internal bool PredictiveTailCaptureEnabled => _predictiveTailCapture.Enabled;
        internal bool PredictiveTailCaptureFinite => _predictiveTailCapture.Finite;
        internal bool StancePlanarDampingEnabled => _stancePlanarDamping.Enabled;
        internal void SetStancePlanarDamping(bool enabled, FootAnkleFoundationReport report) => _stancePlanarDamping.SetEnabled(enabled, report);
        internal void QuiesceStancePlanarDamping(FootAnkleFoundationReport report) => _stancePlanarDamping.QuiescePreserveTrace(report);
        internal void RecordStancePlanarDampingLoadedSolve(bool trace, int fixedSequence, bool loaded, FootAnkleFoundationReport report) =>
            _stancePlanarDamping.RecordLoadedSolve(trace, fixedSequence, loaded, report);
        internal string GetStancePlanarDampingSummary(FootAnkleFoundationReport report) => _stancePlanarDamping.GetSummary(report);

        internal string GetCaptureAuthorityStatus(FootAnkleFoundationReport report)
        {
            return _captureAuthority.GetStatus(report);
        }


        internal bool SetPredictiveTailCaptureArm(string arm, FootAnkleFoundationReport report, out string error)
        {
            return _predictiveTailCapture.SetArm(arm, report, out error);
        }

        internal void FreezeProductionArm()
        {
            _armB = true;
        }

        internal bool Step(
            bool traceEnabled,
            bool dynamicActive,
            Rigidbody footBody,
            Rigidbody lowerBody,
            Vector3 footAnchorLocal,
            Vector3 lowerAnchorLocal,
            Quaternion bindSourceRelative,
            Vector3 parentTransportFeedforwardWorld,
            int fixedSequence,
            float fixedDeltaTime,
            FootAnkleFoundationReport report,
            FootContactRockingTetherCouplingObserver couplingObserver,
            bool supportNormalValid,
            Vector3 supportNormal,
            bool exactLoadedStance)
        {
            _finite = true;
            _invalidReason = null;
            _currentDynamic = dynamicActive;
            _currentPositionErrorWorld = Vector3.zero;
            _currentVelocityErrorWorld = Vector3.zero;
            _currentCorrectionAccelerationWorld = Vector3.zero;
            _currentTransportAccelerationWorld = Vector3.zero;
            _currentRawAccelerationWorld = Vector3.zero;
            _currentAppliedAccelerationWorld = Vector3.zero;
            _currentClamped = false;

            if (footBody == null || lowerBody == null ||
                !FiniteVector(footAnchorLocal) || !FiniteVector(lowerAnchorLocal) || !FiniteQuaternion(bindSourceRelative))
                return Invalidate("MISSING_OR_NON_FINITE_ANKLE_CONSTRAINT_INPUT", traceEnabled, report);

            Vector3 footAnchorWorld = footBody.transform.TransformPoint(footAnchorLocal);
            Vector3 lowerAnchorWorld = lowerBody.transform.TransformPoint(lowerAnchorLocal);
            Vector3 footLever = footAnchorWorld - footBody.worldCenterOfMass;
            Vector3 lowerLever = lowerAnchorWorld - lowerBody.worldCenterOfMass;
            Vector3 footAnchorVelocity = footBody.velocity + Vector3.Cross(footBody.angularVelocity, footLever);
            Vector3 lowerAnchorVelocity = lowerBody.velocity + Vector3.Cross(lowerBody.angularVelocity, lowerLever);
            Vector3 positionError = lowerAnchorWorld - footAnchorWorld;
            Vector3 velocityError = lowerAnchorVelocity - footAnchorVelocity;
            float anchorError = positionError.magnitude;
            float velocityErrorMagnitude = velocityError.magnitude;

            Quaternion physicalRelative = Quaternion.Inverse(lowerBody.rotation) * footBody.rotation;
            Quaternion physicalBindDelta = Quaternion.Inverse(bindSourceRelative) * physicalRelative;
            Vector3 physicalBindDeltaDegrees = QuaternionToAngularVector(physicalBindDelta) * Mathf.Rad2Deg;
            float totalAngleDegrees = Quaternion.Angle(Quaternion.identity, physicalBindDelta);
            Vector3 referenceRatio = new Vector3(
                Mathf.Abs(physicalBindDeltaDegrees.x) / FootAnkleSourceDrivenRelativePdIsolation.FlexionLimitDegrees,
                Mathf.Abs(physicalBindDeltaDegrees.y) / FootAnkleSourceDrivenRelativePdIsolation.AxialLimitDegrees,
                Mathf.Abs(physicalBindDeltaDegrees.z) / FootAnkleSourceDrivenRelativePdIsolation.InversionLimitDegrees);
            Vector3 hardRatio = new Vector3(
                Mathf.Abs(physicalBindDeltaDegrees.x) / HardFlexionDegrees,
                Mathf.Abs(physicalBindDeltaDegrees.y) / HardAxialDegrees,
                Mathf.Abs(physicalBindDeltaDegrees.z) / HardInversionDegrees);

            if (!FiniteVector(footAnchorWorld) || !FiniteVector(lowerAnchorWorld) ||
                !FiniteVector(footAnchorVelocity) || !FiniteVector(lowerAnchorVelocity) ||
                !FiniteVector(positionError) || !FiniteVector(velocityError) || !FiniteVector(parentTransportFeedforwardWorld) ||
                !FiniteFloat(anchorError) || !FiniteFloat(velocityErrorMagnitude) ||
                !FiniteQuaternion(physicalRelative) || !FiniteQuaternion(physicalBindDelta) ||
                !FiniteVector(physicalBindDeltaDegrees) || !FiniteVector(referenceRatio) || !FiniteVector(hardRatio) ||
                !FiniteFloat(totalAngleDegrees))
                return Invalidate("NON_FINITE_ANKLE_CONSTRAINT_STATE", traceEnabled, report);

            _currentAnchorError = anchorError;
            _currentVelocityError = velocityErrorMagnitude;
            _currentPositionErrorWorld = positionError;
            _currentVelocityErrorWorld = velocityError;
            _currentPhysicalBindDeltaDegrees = physicalBindDeltaDegrees;
            _currentReferenceRatio = referenceRatio;
            _currentHardRatio = hardRatio;
            _currentTotalAngleDegrees = totalAngleDegrees;

            // V04300 eligibility comes only from existing production contact/load selectors, never observer readiness.
            // Keep the original point error for geometry, hard gates and historical cap eligibility signals.
            if (!_stancePlanarDamping.Evaluate(traceEnabled, fixedSequence, _armB && dynamicActive && exactLoadedStance,
                velocityError, Vector3.Cross(footBody.angularVelocity, footLever), supportNormalValid, supportNormal,
                report, out Vector3 effectiveVelocityError))
                return Invalidate(_stancePlanarDamping.InvalidReason, traceEnabled, report);

            // Diagnostic candidate is captured before either unchanged hard gate. No force has been written.
            couplingObserver.RecordEvaluation(traceEnabled, dynamicActive, fixedSequence, fixedDeltaTime,
                footBody, lowerBody, footLever, lowerLever, positionError, velocityError,
                parentTransportFeedforwardWorld, supportNormalValid, supportNormal,
                effectiveVelocityError, _stancePlanarDamping.CurrentActive, report);

            if (dynamicActive && anchorError >= AnchorValidityMeters)
                return Invalidate(_armB ? "ANKLE_TRANSLATION_TETHER_OVER_30MM" : "ANKLE_ANCHOR_OVER_30MM", traceEnabled, report);

            bool referenceExceeded = referenceRatio.x > 1f || referenceRatio.y > 1f || referenceRatio.z > 1f;
            bool hardExceeded = hardRatio.x > 1f || hardRatio.y > 1f || hardRatio.z > 1f || totalAngleDegrees > HardTotalDegrees;
            if (_armB && dynamicActive && hardExceeded)
                return Invalidate("ANKLE_TRANSLATION_TETHER_HARD_ANGULAR_ENVELOPE_EXCEEDED", traceEnabled, report);

            if (_armB && dynamicActive)
            {
                Vector3 correction = positionError * TetherPositionSpring + effectiveVelocityError * TetherVelocityDamper;
                Vector3 transport = parentTransportFeedforwardWorld;
                Vector3 raw = correction + transport;
                Vector3 applied = raw;
                float rawMagnitude = raw.magnitude;

                Vector3 radialClosingDirection = anchorError > 0.000001f ? positionError / anchorError : Vector3.zero;
                Vector3 comVelocityError = lowerBody.velocity - footBody.velocity;
                float radialOpeningRate = Vector3.Dot(velocityError, radialClosingDirection);
                float comOpeningRate = Vector3.Dot(comVelocityError, radialClosingDirection);
                float transportRadialClosing = Vector3.Dot(transport, radialClosingDirection);
                float rawRadialClosing = Vector3.Dot(raw, radialClosingDirection);
                float baseSelectedMaxAcceleration = _captureAuthority.Evaluate(
                    traceEnabled,
                    dynamicActive,
                    anchorError,
                    radialOpeningRate,
                    comOpeningRate,
                    transportRadialClosing,
                    rawRadialClosing,
                    rawMagnitude,
                    report);

                float selectedMaxAcceleration = _predictiveTailCapture.Evaluate(
                    traceEnabled,
                    fixedSequence,
                    fixedDeltaTime,
                    dynamicActive,
                    anchorError,
                    radialOpeningRate,
                    transportRadialClosing,
                    rawRadialClosing,
                    rawMagnitude,
                    baseSelectedMaxAcceleration,
                    _captureAuthority.CurrentEligible,
                    report);
                if (!_predictiveTailCapture.Finite)
                    return Invalidate(_predictiveTailCapture.InvalidReason ?? "ANKLE_TRANSLATION_PREDICTIVE_TAIL_CAPTURE_INVALID", traceEnabled, report);

                bool clamped = rawMagnitude > selectedMaxAcceleration && rawMagnitude > 0.0001f;
                if (clamped)
                    applied = raw * (selectedMaxAcceleration / rawMagnitude);

                if (!FiniteVector(raw) || !FiniteVector(applied) || !FiniteFloat(selectedMaxAcceleration))
                    return Invalidate("NON_FINITE_ANKLE_TRANSLATION_TETHER_COMMAND", traceEnabled, report);

                // Deliberately child-only and COM-only for this causal topology isolation.
                footBody.AddForce(applied, ForceMode.Acceleration);
                _stancePlanarDamping.RecordWritten(traceEnabled, fixedSequence, report);
                couplingObserver.RecordWrittenCommand(traceEnabled, fixedSequence, raw, applied,
                    selectedMaxAcceleration, clamped, report);
                _currentCorrectionAccelerationWorld = correction;
                _currentTransportAccelerationWorld = transport;
                _currentRawAccelerationWorld = raw;
                _currentAppliedAccelerationWorld = applied;
                _currentClamped = clamped;
            }

            if (traceEnabled)
            {
                _traceSamples++;
                if (dynamicActive) _traceDynamicSamples++;
                if (_armB && dynamicActive)
                {
                    _traceAppliedSamples++;
                    if (_currentClamped) _traceClamped++;
                }
                if (referenceExceeded) _traceReferenceEnvelopeExceed++;
                if (hardExceeded) _traceHardEnvelopeExceed++;
                _traceAnchorSum += anchorError;
                _traceAnchorMax = Mathf.Max(_traceAnchorMax, anchorError);
                _traceVelocityErrorSum += velocityErrorMagnitude;
                _traceVelocityErrorMax = Mathf.Max(_traceVelocityErrorMax, velocityErrorMagnitude);
                float rawA = _currentRawAccelerationWorld.magnitude;
                float appA = _currentAppliedAccelerationWorld.magnitude;
                _traceRawAccelerationSum += rawA;
                _traceRawAccelerationMax = Mathf.Max(_traceRawAccelerationMax, rawA);
                _traceAppliedAccelerationSum += appA;
                _traceAppliedAccelerationMax = Mathf.Max(_traceAppliedAccelerationMax, appA);
                _tracePhysicalAxisAbsMax = ComponentMax(_tracePhysicalAxisAbsMax, ComponentAbs(physicalBindDeltaDegrees));
                _traceHardRatioMax = ComponentMax(_traceHardRatioMax, hardRatio);
                _traceTotalAngleMax = Mathf.Max(_traceTotalAngleMax, totalAngleDegrees);
            }

            CopyToReport(report);
            return true;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceSamples = 0;
            _traceDynamicSamples = 0;
            _traceAppliedSamples = 0;
            _traceInvalid = 0;
            _traceClamped = 0;
            _traceReferenceEnvelopeExceed = 0;
            _traceHardEnvelopeExceed = 0;
            _traceAnchorSum = 0f;
            _traceAnchorMax = 0f;
            _traceVelocityErrorSum = 0f;
            _traceVelocityErrorMax = 0f;
            _traceRawAccelerationSum = 0f;
            _traceRawAccelerationMax = 0f;
            _traceAppliedAccelerationSum = 0f;
            _traceAppliedAccelerationMax = 0f;
            _tracePhysicalAxisAbsMax = Vector3.zero;
            _traceHardRatioMax = Vector3.zero;
            _traceTotalAngleMax = 0f;
            _stancePlanarDamping.ResetTrace(report);
            _captureAuthority.ResetTrace(report);
            _predictiveTailCapture.ResetTrace(report);
            ClearCurrentState(report);
        }

        internal void ClearCurrentState(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _currentDynamic = false;
            _currentAnchorError = 0f;
            _currentVelocityError = 0f;
            _currentPositionErrorWorld = Vector3.zero;
            _currentVelocityErrorWorld = Vector3.zero;
            _currentCorrectionAccelerationWorld = Vector3.zero;
            _currentTransportAccelerationWorld = Vector3.zero;
            _currentRawAccelerationWorld = Vector3.zero;
            _currentAppliedAccelerationWorld = Vector3.zero;
            _currentClamped = false;
            _currentPhysicalBindDeltaDegrees = Vector3.zero;
            _currentReferenceRatio = Vector3.zero;
            _currentHardRatio = Vector3.zero;
            _currentTotalAngleDegrees = 0f;
            _captureAuthority.ClearCurrent(report);
            _predictiveTailCapture.ClearCurrent(report);
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            _stancePlanarDamping.CopyToReport(report);
            _captureAuthority.CopyToReport(report);
            _predictiveTailCapture.CopyToReport(report);
            report.AnkleConstraintMode = Mode;
            report.AnkleConstraintArm = Arm;
            report.AnkleConstraintFinite = _finite;
            report.AnkleConstraintInvalidReason = _invalidReason;
            report.AnkleConstraintCurrentDynamic = _currentDynamic;
            report.AnkleConstraintCurrentCorrectionAccelerationWorld = _currentCorrectionAccelerationWorld;
            report.AnkleConstraintCurrentTransportAccelerationWorld = _currentTransportAccelerationWorld;
            report.AnkleConstraintCurrentRawAccelerationWorld = _currentRawAccelerationWorld;
            report.AnkleConstraintCurrentAppliedAccelerationWorld = _currentAppliedAccelerationWorld;
            report.AnkleConstraintCurrentClamped = _currentClamped;
            report.AnkleConstraintCurrentSummary =
                "arm=" + Arm +
                "/dynamic=" + _currentDynamic +
                "/anchorMm=" + F(_currentAnchorError * 1000f) +
                "/velErr=" + F(_currentVelocityError) +
                "/posErr=" + V(_currentPositionErrorWorld) +
                "/velErrV=" + V(_currentVelocityErrorWorld) +
                "/corrA=" + V(_currentCorrectionAccelerationWorld) +
                "/transportA=" + V(_currentTransportAccelerationWorld) +
                "/rawA=" + V(_currentRawAccelerationWorld) +
                "/appA=" + V(_currentAppliedAccelerationWorld) +
                "/clamp=" + _currentClamped +
                "/physDeg=" + V(_currentPhysicalBindDeltaDegrees) +
                "/refRatio=" + V(_currentReferenceRatio) +
                "/hardRatio=" + V(_currentHardRatio) +
                "/totalDeg=" + F(_currentTotalAngleDegrees);

            float denom = Mathf.Max(1, _traceSamples);
            float appliedDenom = Mathf.Max(1, _traceAppliedSamples);
            report.AnkleConstraintTraceSummary =
                "n=" + _traceSamples +
                "/dynamic=" + _traceDynamicSamples +
                "/applied=" + _traceAppliedSamples +
                "/invalid=" + _traceInvalid +
                "/anchorMeanMaxMm=" + F((_traceAnchorSum / denom) * 1000f) + "-" + F(_traceAnchorMax * 1000f) +
                "/velErrMeanMax=" + F(_traceVelocityErrorSum / denom) + "-" + F(_traceVelocityErrorMax) +
                "/rawAMeanMax=" + F(_traceRawAccelerationSum / appliedDenom) + "-" + F(_traceRawAccelerationMax) +
                "/appAMeanMax=" + F(_traceAppliedAccelerationSum / appliedDenom) + "-" + F(_traceAppliedAccelerationMax) +
                "/clamp=" + _traceClamped +
                "/refEnvelopeExceed=" + _traceReferenceEnvelopeExceed +
                "/hardEnvelopeExceed=" + _traceHardEnvelopeExceed +
                "/physAxisAbsMaxDeg=" + V(_tracePhysicalAxisAbsMax) +
                "/hardRatioMax=" + V(_traceHardRatioMax) +
                "/totalAngleMaxDeg=" + F(_traceTotalAngleMax) +
                "/composition=TRANSPORT_PLUS_RELATIVE_PD_BEFORE_SELECTED_FINAL_CLAMP" +
                "/tether(Kp-Kd-BaseMaxA-GateMm)=" + F(TetherPositionSpring) + "-" + F(TetherVelocityDamper) + "-" + F(TetherMaxAcceleration) + "-" + F(AnchorValidityMeters * 1000f) +
                "/captureArm=" + _captureAuthority.Arm + "/selectedMaxA=" + F(_captureAuthority.CurrentSelectedMaxAcceleration) +
                "/predictiveArm=" + _predictiveTailCapture.Arm + "/predictiveFinalMaxA=" + F(_predictiveTailCapture.CurrentFinalSelectedMaxAcceleration) +
                "/hardDeg(F-A-I-total)=" + F(HardFlexionDegrees) + "-" + F(HardAxialDegrees) + "-" + F(HardInversionDegrees) + "-" + F(HardTotalDegrees);
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/arm=" + Arm +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.AnkleConstraintCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.AnkleConstraintTraceSummary : "<none>");
        }

        private bool Invalidate(string reason, bool traceEnabled, FootAnkleFoundationReport report)
        {
            _finite = false;
            _invalidReason = reason;
            if (traceEnabled) _traceInvalid++;
            CopyToReport(report);
            return false;
        }

        private static Vector3 QuaternionToAngularVector(Quaternion q)
        {
            if (q.w < 0f)
            {
                q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
            }
            float w = Mathf.Clamp(q.w, -1f, 1f);
            float halfAngle = Mathf.Acos(w);
            float sinHalf = Mathf.Sin(halfAngle);
            if (Mathf.Abs(sinHalf) < 0.000001f)
                return Vector3.zero;
            Vector3 axis = new Vector3(q.x, q.y, q.z) / sinHalf;
            return axis * (2f * halfAngle);
        }

        private static Vector3 ComponentAbs(Vector3 value)
        {
            return new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));
        }

        private static Vector3 ComponentMax(Vector3 a, Vector3 b)
        {
            return new Vector3(Mathf.Max(a.x, b.x), Mathf.Max(a.y, b.y), Mathf.Max(a.z, b.z));
        }

        private static bool FiniteFloat(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool FiniteVector(Vector3 value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z);
        private static bool FiniteQuaternion(Quaternion value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z) && FiniteFloat(value.w);
        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
