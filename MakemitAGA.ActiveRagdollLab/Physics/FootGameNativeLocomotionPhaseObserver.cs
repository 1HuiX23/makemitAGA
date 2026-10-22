using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.5 read-only game-native locomotion stance/swing phase qualification.
    ///
    /// v0.4.4 proved that keeping both Foot colliders mechanically planted throughout MiSide's native
    /// point-to-point walk is the wrong experiment: static exact-support contact is healthy, while native
    /// locomotion can rotate/move the Lower rapidly enough that a physically planted Foot accumulates axial
    /// debt and correctly trips the existing 40 degree hard gate. v0.4.5 therefore returns to Foot-Collider-OFF
    /// free-space observation and asks a narrower question first: can the existing source animation plus the
    /// already-executed five-point sole probe identify repeatable per-foot STANCE and SWING phases without any
    /// user-controlled speed assumption?
    ///
    /// This observer issues no Physics query and writes no Rigidbody/Collider state. v0.4.7 may reuse the
    /// classifier state online during stance contact, but only contact-free samples are allowed to advance the
    /// historical v0.4.5 qualification counters. It consumes only:
    ///   - render-sampled source Foot linear velocity already produced by FootAnkleProxyFoundation;
    ///   - the existing five-point sole probe result;
    ///   - the current support normal/reference-up witness.
    ///
    /// Classification is deliberately conservative and hysteretic. The first version qualifies only that each
    /// foot exhibits at least one complete STANCE->SWING->STANCE cycle under game-native locomotion. It does NOT
    /// yet authorize collider switching or claim production gait control.
    /// </summary>
    internal sealed class FootGameNativeLocomotionPhaseObserver
    {
        internal const string Mode = "GAME_NATIVE_LOCOMOTION_STANCE_SWING_PHASE_QUALIFICATION_V0450";

        internal const float StanceMaxAbsClearanceMeters = 0.030f;
        internal const float SwingMinAbsClearanceMeters = 0.055f;
        internal const float StanceMaxTangentialSpeed = 0.35f;
        internal const float StanceMaxNormalSpeed = 0.22f;
        internal const float SwingMinTangentialSpeed = 0.65f;
        internal const float SwingMinNormalSpeed = 0.30f;
        internal const float PhaseConfirmSeconds = 0.050f;
        internal const int MinTraceSamples = 180;
        internal const int MinStanceSamples = 30;
        internal const int MinSwingSamples = 10;

        internal const string StateUnknown = "UNKNOWN";
        internal const string StateStance = "STANCE_CANDIDATE";
        internal const string StateToSwing = "TRANSITION_TO_SWING";
        internal const string StateSwing = "SWING_CANDIDATE";
        internal const string StateToStance = "TRANSITION_TO_STANCE";

        private bool _finite = true;
        private string _invalidReason;
        private string _state = StateUnknown;
        private string _lastReason = "<none>";

        private int _traceSamples;
        private int _eligibleSamples;
        private int _stanceSamples;
        private int _swingSamples;
        private int _transitionSamples;
        private int _unknownSamples;
        private int _stanceEntries;
        private int _swingEntries;
        private int _stanceToSwingTransitions;
        private int _swingToStanceTransitions;
        private int _ambiguousSamples;
        private int _noHitSamples;
        private int _fullFootprintSamples;
        private int _sameSurfaceSamples;
        private int _firstStanceFixed;
        private int _firstSwingFixed;
        private int _lastTransitionFixed;

        private float _candidateSeconds;
        private string _candidateTarget = StateUnknown;

        private float _stanceTangentialSpeedSum;
        private float _stanceTangentialSpeedMax;
        private float _swingTangentialSpeedSum;
        private float _swingTangentialSpeedMax;
        private float _stanceNormalSpeedAbsSum;
        private float _stanceNormalSpeedAbsMax;
        private float _swingNormalSpeedAbsSum;
        private float _swingNormalSpeedAbsMax;
        private float _stanceClearanceAbsSum;
        private float _stanceClearanceAbsMax;
        private float _swingClearanceAbsSum;
        private float _swingClearanceAbsMax;
        private int _stanceSurfaceId;
        private int _stanceSurfaceSwitches;
        private bool _currentStrongStanceSample;
        private bool _currentStrongSwingSample;

        internal bool Finite => _finite;
        internal string State => _state;
        internal bool CurrentStrongStanceSample => _currentStrongStanceSample;
        internal bool CurrentStrongSwingSample => _currentStrongSwingSample;

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _state = StateUnknown;
            _lastReason = "<none>";
            _traceSamples = 0;
            _eligibleSamples = 0;
            _stanceSamples = 0;
            _swingSamples = 0;
            _transitionSamples = 0;
            _unknownSamples = 0;
            _stanceEntries = 0;
            _swingEntries = 0;
            _stanceToSwingTransitions = 0;
            _swingToStanceTransitions = 0;
            _ambiguousSamples = 0;
            _noHitSamples = 0;
            _fullFootprintSamples = 0;
            _sameSurfaceSamples = 0;
            _firstStanceFixed = 0;
            _firstSwingFixed = 0;
            _lastTransitionFixed = 0;
            _candidateSeconds = 0f;
            _candidateTarget = StateUnknown;
            _stanceTangentialSpeedSum = 0f;
            _stanceTangentialSpeedMax = 0f;
            _swingTangentialSpeedSum = 0f;
            _swingTangentialSpeedMax = 0f;
            _stanceNormalSpeedAbsSum = 0f;
            _stanceNormalSpeedAbsMax = 0f;
            _swingNormalSpeedAbsSum = 0f;
            _swingNormalSpeedAbsMax = 0f;
            _stanceClearanceAbsSum = 0f;
            _stanceClearanceAbsMax = 0f;
            _swingClearanceAbsSum = 0f;
            _swingClearanceAbsMax = 0f;
            _stanceSurfaceId = 0;
            _stanceSurfaceSwitches = 0;
            _currentStrongStanceSample = false;
            _currentStrongSwingSample = false;
            CopyToReport(report, false);
        }

        internal void ObserveFixed(
            bool traceEnabled,
            bool qualificationSampleEligible,
            bool phaseTrackingEligible,
            int fixedSequence,
            float fixedDeltaTime,
            Vector3 sourceFootLinearVelocityWorld,
            Vector3 referenceUpWorld,
            int groundHitCount,
            bool footprintSameSurface,
            int groundColliderInstanceId,
            float groundMaxAbsClearanceMeters,
            Vector3 groundAverageNormalWorld,
            FootAnkleFoundationReport report)
        {
            if (!traceEnabled)
            {
                _currentStrongStanceSample = false;
                _currentStrongSwingSample = false;
                CopyToReport(report, false);
                return;
            }

            _traceSamples++;
            // v0.4.7 reuses the frozen classifier online while a confirmed STANCE may have contact enabled.
            // Phase state must keep updating so the first strong swing sample can release contact immediately.
            // Only contact-free samples contribute to the historical v0.4.5 qualification counters.
            if (!phaseTrackingEligible)
            {
                _currentStrongStanceSample = false;
                _currentStrongSwingSample = false;
                _lastReason = "PHASE_TRACKING_NOT_ELIGIBLE";
                CopyToReport(report, TraceQualified());
                return;
            }

            float dt = IsFinite(fixedDeltaTime) && fixedDeltaTime > 0f ? fixedDeltaTime : 0.02f;
            if (!IsFinite(sourceFootLinearVelocityWorld) || !IsFinite(referenceUpWorld) || !IsFinite(groundMaxAbsClearanceMeters))
            {
                Invalidate("NON_FINITE_GAIT_PHASE_SAMPLE");
                CopyToReport(report, false);
                return;
            }

            Vector3 normal = groundAverageNormalWorld;
            if (!IsFinite(normal) || normal.sqrMagnitude < 0.0001f)
                normal = referenceUpWorld;
            if (!IsFinite(normal) || normal.sqrMagnitude < 0.0001f)
                normal = Vector3.up;
            normal.Normalize();

            float normalSpeed = Vector3.Dot(sourceFootLinearVelocityWorld, normal);
            Vector3 tangential = sourceFootLinearVelocityWorld - normal * normalSpeed;
            float tangentialSpeed = tangential.magnitude;
            float normalSpeedAbs = Mathf.Abs(normalSpeed);
            float clearanceAbs = Mathf.Abs(groundMaxAbsClearanceMeters);
            if (!IsFinite(normalSpeedAbs) || !IsFinite(tangentialSpeed) || !IsFinite(clearanceAbs))
            {
                Invalidate("NON_FINITE_GAIT_PHASE_DECOMPOSITION");
                CopyToReport(report, false);
                return;
            }

            if (qualificationSampleEligible)
            {
                _eligibleSamples++;
                if (groundHitCount == 0) _noHitSamples++;
                if (groundHitCount == 5) _fullFootprintSamples++;
                if (footprintSameSurface) _sameSurfaceSamples++;
            }

            bool strongStance = groundHitCount == 5 && footprintSameSurface &&
                                clearanceAbs <= StanceMaxAbsClearanceMeters &&
                                normalSpeedAbs <= StanceMaxNormalSpeed &&
                                tangentialSpeed <= StanceMaxTangentialSpeed;
            bool strongSwing = groundHitCount == 0 ||
                               clearanceAbs >= SwingMinAbsClearanceMeters ||
                               normalSpeedAbs >= SwingMinNormalSpeed ||
                               tangentialSpeed >= SwingMinTangentialSpeed;

            // If both become true because a render-sampled velocity spike occurs exactly while all five probes are
            // still close to the floor, classify it as transition evidence rather than forcing either endpoint.
            if (strongStance && strongSwing)
            {
                strongStance = false;
                strongSwing = false;
                if (qualificationSampleEligible) _ambiguousSamples++;
            }
            _currentStrongStanceSample = strongStance;
            _currentStrongSwingSample = strongSwing;

            if (strongStance)
                HandleStanceEvidence(fixedSequence, dt, tangentialSpeed, normalSpeedAbs, clearanceAbs, groundColliderInstanceId, qualificationSampleEligible, report);
            else if (strongSwing)
                HandleSwingEvidence(fixedSequence, dt, tangentialSpeed, normalSpeedAbs, clearanceAbs, qualificationSampleEligible, report);
            else
                HandleTransitionEvidence(qualificationSampleEligible);

            bool qualified = TraceQualified();
            _lastReason = qualified ? "COMPLETE_NATIVE_GAIT_CYCLE_OBSERVED" : QualificationReason();
            CopyToReport(report, qualified);
        }

        internal void EndTrace(FootAnkleFoundationReport report)
        {
            bool qualified = TraceQualified();
            _lastReason = qualified ? "COMPLETE_NATIVE_GAIT_CYCLE_OBSERVED" : QualificationReason();
            CopyToReport(report, qualified);
        }

        private void HandleStanceEvidence(int fixedSequence, float dt, float tangentialSpeed, float normalSpeedAbs, float clearanceAbs, int surfaceId, bool qualificationSampleEligible, FootAnkleFoundationReport report)
        {
            if (_state == StateStance)
            {
                if (qualificationSampleEligible) AccumulateStance(tangentialSpeed, normalSpeedAbs, clearanceAbs, surfaceId);
                ClearCandidate();
                return;
            }

            if (_state == StateSwing || _state == StateToStance || _state == StateToSwing || _state == StateUnknown)
            {
                if (_state != StateToStance)
                {
                    _state = StateToStance;
                    _lastTransitionFixed = fixedSequence;
                    BeginCandidate(StateStance);
                }
                else
                {
                    ContinueCandidate(StateStance, dt);
                }

                if (qualificationSampleEligible) _transitionSamples++;
                if (_candidateSeconds >= PhaseConfirmSeconds)
                {
                    bool fromSwing = _swingEntries > 0;
                    _state = StateStance;
                    if (qualificationSampleEligible)
                    {
                        _stanceEntries++;
                        if (_firstStanceFixed == 0) _firstStanceFixed = fixedSequence;
                        if (fromSwing) _swingToStanceTransitions++;
                        AccumulateStance(tangentialSpeed, normalSpeedAbs, clearanceAbs, surfaceId);
                    }
                    Plugin.Logger?.LogInfo("[ARL][GAIT_PHASE_V0450] side=" + Clean(report != null ? report.Side : null) +
                                           " | enter=STANCE | fixed=" + fixedSequence +
                                           " | tangential=" + F(tangentialSpeed) +
                                           " | normalAbs=" + F(normalSpeedAbs) +
                                           " | clearanceMm=" + F(clearanceAbs * 1000f) +
                                           " | surfaceId=" + surfaceId +
                                           " | qualificationSample=" + qualificationSampleEligible);
                    ClearCandidate();
                }
                return;
            }
        }

        private void HandleSwingEvidence(int fixedSequence, float dt, float tangentialSpeed, float normalSpeedAbs, float clearanceAbs, bool qualificationSampleEligible, FootAnkleFoundationReport report)
        {
            if (_state == StateSwing)
            {
                if (qualificationSampleEligible) AccumulateSwing(tangentialSpeed, normalSpeedAbs, clearanceAbs);
                ClearCandidate();
                return;
            }

            if (_state == StateStance || _state == StateToSwing || _state == StateToStance || _state == StateUnknown)
            {
                if (_state != StateToSwing)
                {
                    _state = StateToSwing;
                    _lastTransitionFixed = fixedSequence;
                    BeginCandidate(StateSwing);
                }
                else
                {
                    ContinueCandidate(StateSwing, dt);
                }

                if (qualificationSampleEligible) _transitionSamples++;
                if (_candidateSeconds >= PhaseConfirmSeconds)
                {
                    bool fromStance = _stanceEntries > 0;
                    _state = StateSwing;
                    if (qualificationSampleEligible)
                    {
                        _swingEntries++;
                        if (_firstSwingFixed == 0) _firstSwingFixed = fixedSequence;
                        if (fromStance) _stanceToSwingTransitions++;
                        AccumulateSwing(tangentialSpeed, normalSpeedAbs, clearanceAbs);
                    }
                    Plugin.Logger?.LogInfo("[ARL][GAIT_PHASE_V0450] side=" + Clean(report != null ? report.Side : null) +
                                           " | enter=SWING | fixed=" + fixedSequence +
                                           " | tangential=" + F(tangentialSpeed) +
                                           " | normalAbs=" + F(normalSpeedAbs) +
                                           " | clearanceMm=" + F(clearanceAbs * 1000f) +
                                           " | qualificationSample=" + qualificationSampleEligible);
                    ClearCandidate();
                }
                return;
            }
        }

        private void HandleTransitionEvidence(bool qualificationSampleEligible)
        {
            if (qualificationSampleEligible)
            {
                _transitionSamples++;
                if (_state == StateUnknown)
                    _unknownSamples++;
            }
            ClearCandidate();
        }

        private void AccumulateStance(float tangentialSpeed, float normalSpeedAbs, float clearanceAbs, int surfaceId)
        {
            _stanceSamples++;
            _stanceTangentialSpeedSum += tangentialSpeed;
            _stanceTangentialSpeedMax = Mathf.Max(_stanceTangentialSpeedMax, tangentialSpeed);
            _stanceNormalSpeedAbsSum += normalSpeedAbs;
            _stanceNormalSpeedAbsMax = Mathf.Max(_stanceNormalSpeedAbsMax, normalSpeedAbs);
            _stanceClearanceAbsSum += clearanceAbs;
            _stanceClearanceAbsMax = Mathf.Max(_stanceClearanceAbsMax, clearanceAbs);
            if (surfaceId != 0)
            {
                if (_stanceSurfaceId != 0 && _stanceSurfaceId != surfaceId)
                    _stanceSurfaceSwitches++;
                _stanceSurfaceId = surfaceId;
            }
        }

        private void AccumulateSwing(float tangentialSpeed, float normalSpeedAbs, float clearanceAbs)
        {
            _swingSamples++;
            _swingTangentialSpeedSum += tangentialSpeed;
            _swingTangentialSpeedMax = Mathf.Max(_swingTangentialSpeedMax, tangentialSpeed);
            _swingNormalSpeedAbsSum += normalSpeedAbs;
            _swingNormalSpeedAbsMax = Mathf.Max(_swingNormalSpeedAbsMax, normalSpeedAbs);
            _swingClearanceAbsSum += clearanceAbs;
            _swingClearanceAbsMax = Mathf.Max(_swingClearanceAbsMax, clearanceAbs);
        }

        private void BeginCandidate(string target)
        {
            _candidateTarget = target;
            _candidateSeconds = 0f;
        }

        private void ContinueCandidate(string target, float dt)
        {
            if (!string.Equals(_candidateTarget, target, StringComparison.Ordinal))
            {
                BeginCandidate(target);
                return;
            }
            _candidateSeconds += dt;
        }

        private void ClearCandidate()
        {
            _candidateTarget = StateUnknown;
            _candidateSeconds = 0f;
        }

        private bool TraceQualified()
        {
            return _finite &&
                   _eligibleSamples >= MinTraceSamples &&
                   _stanceSamples >= MinStanceSamples &&
                   _swingSamples >= MinSwingSamples &&
                   _stanceEntries >= 2 &&
                   _swingEntries >= 1 &&
                   _stanceToSwingTransitions >= 1 &&
                   _swingToStanceTransitions >= 1;
        }

        private string QualificationReason()
        {
            if (!_finite) return "OBSERVER_NONFINITE";
            if (_eligibleSamples < MinTraceSamples) return "INSUFFICIENT_CONTACT_FREE_QUALIFICATION_TRACE";
            if (_stanceSamples < MinStanceSamples) return "STANCE_EVIDENCE_INSUFFICIENT";
            if (_swingSamples < MinSwingSamples) return "SWING_EVIDENCE_INSUFFICIENT";
            if (_stanceEntries < 2) return "SECOND_STANCE_ENTRY_NOT_OBSERVED";
            if (_swingEntries < 1) return "SWING_ENTRY_NOT_OBSERVED";
            if (_stanceToSwingTransitions < 1) return "STANCE_TO_SWING_NOT_OBSERVED";
            if (_swingToStanceTransitions < 1) return "SWING_TO_STANCE_NOT_OBSERVED";
            return "READY_FOR_REVIEW";
        }

        private void Invalidate(string reason)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            _lastReason = reason;
        }

        private void CopyToReport(FootAnkleFoundationReport report, bool qualified)
        {
            if (report == null) return;
            report.GameNativeGaitPhaseMode = Mode;
            report.GameNativeGaitPhaseFinite = _finite;
            report.GameNativeGaitPhaseInvalidReason = _invalidReason;
            report.GameNativeGaitPhaseState = _state;
            report.GameNativeGaitTraceSamples = _traceSamples;
            report.GameNativeGaitEligibleSamples = _eligibleSamples;
            report.GameNativeGaitStanceSamples = _stanceSamples;
            report.GameNativeGaitSwingSamples = _swingSamples;
            report.GameNativeGaitTransitionSamples = _transitionSamples;
            report.GameNativeGaitUnknownSamples = _unknownSamples;
            report.GameNativeGaitStanceEntries = _stanceEntries;
            report.GameNativeGaitSwingEntries = _swingEntries;
            report.GameNativeGaitStanceToSwingTransitions = _stanceToSwingTransitions;
            report.GameNativeGaitSwingToStanceTransitions = _swingToStanceTransitions;
            report.GameNativeGaitAmbiguousSamples = _ambiguousSamples;
            report.GameNativeGaitNoHitSamples = _noHitSamples;
            report.GameNativeGaitFullFootprintSamples = _fullFootprintSamples;
            report.GameNativeGaitSameSurfaceSamples = _sameSurfaceSamples;
            report.GameNativeGaitFirstStanceFixed = _firstStanceFixed;
            report.GameNativeGaitFirstSwingFixed = _firstSwingFixed;
            report.GameNativeGaitLastTransitionFixed = _lastTransitionFixed;
            report.GameNativeGaitStanceTangentialSpeedMean = _stanceTangentialSpeedSum / Mathf.Max(1, _stanceSamples);
            report.GameNativeGaitStanceTangentialSpeedMax = _stanceTangentialSpeedMax;
            report.GameNativeGaitSwingTangentialSpeedMean = _swingTangentialSpeedSum / Mathf.Max(1, _swingSamples);
            report.GameNativeGaitSwingTangentialSpeedMax = _swingTangentialSpeedMax;
            report.GameNativeGaitStanceNormalSpeedAbsMean = _stanceNormalSpeedAbsSum / Mathf.Max(1, _stanceSamples);
            report.GameNativeGaitStanceNormalSpeedAbsMax = _stanceNormalSpeedAbsMax;
            report.GameNativeGaitSwingNormalSpeedAbsMean = _swingNormalSpeedAbsSum / Mathf.Max(1, _swingSamples);
            report.GameNativeGaitSwingNormalSpeedAbsMax = _swingNormalSpeedAbsMax;
            report.GameNativeGaitStanceClearanceAbsMeanMeters = _stanceClearanceAbsSum / Mathf.Max(1, _stanceSamples);
            report.GameNativeGaitStanceClearanceAbsMaxMeters = _stanceClearanceAbsMax;
            report.GameNativeGaitSwingClearanceAbsMeanMeters = _swingClearanceAbsSum / Mathf.Max(1, _swingSamples);
            report.GameNativeGaitSwingClearanceAbsMaxMeters = _swingClearanceAbsMax;
            report.GameNativeGaitStanceSurfaceSwitches = _stanceSurfaceSwitches;
            report.GameNativeGaitTraceQualified = qualified;
            report.GameNativeGaitLastReason = _lastReason;
            report.GameNativeGaitTraceSummary =
                "n=" + _traceSamples +
                "/eligible=" + _eligibleSamples +
                "/state=" + _state +
                "/samples(S-W-T-U)=" + _stanceSamples + "-" + _swingSamples + "-" + _transitionSamples + "-" + _unknownSamples +
                "/entries(S-W)=" + _stanceEntries + "-" + _swingEntries +
                "/transitions(SW-WS)=" + _stanceToSwingTransitions + "-" + _swingToStanceTransitions +
                "/ambiguous=" + _ambiguousSamples +
                "/hits(no-full-same)=" + _noHitSamples + "-" + _fullFootprintSamples + "-" + _sameSurfaceSamples +
                "/firstFixed(S-W-lastT)=" + _firstStanceFixed + "-" + _firstSwingFixed + "-" + _lastTransitionFixed +
                "/stanceVtanMeanMax=" + F(report.GameNativeGaitStanceTangentialSpeedMean) + "-" + F(_stanceTangentialSpeedMax) +
                "/swingVtanMeanMax=" + F(report.GameNativeGaitSwingTangentialSpeedMean) + "-" + F(_swingTangentialSpeedMax) +
                "/stanceVnMeanMax=" + F(report.GameNativeGaitStanceNormalSpeedAbsMean) + "-" + F(_stanceNormalSpeedAbsMax) +
                "/swingVnMeanMax=" + F(report.GameNativeGaitSwingNormalSpeedAbsMean) + "-" + F(_swingNormalSpeedAbsMax) +
                "/clearMm(Smean-Smax-Wmean-Wmax)=" +
                    F(report.GameNativeGaitStanceClearanceAbsMeanMeters * 1000f) + "-" + F(_stanceClearanceAbsMax * 1000f) + "-" +
                    F(report.GameNativeGaitSwingClearanceAbsMeanMeters * 1000f) + "-" + F(_swingClearanceAbsMax * 1000f) +
                "/stanceSurfaceSwitch=" + _stanceSurfaceSwitches +
                "/TRACE_GAME_NATIVE_GAIT_PHASE_QUALIFIED=" + qualified +
                "/thresholds(clearS-clearW-vtanS-vnS-vtanW-vnW-hold-minN-minS-minW)=" +
                    F(StanceMaxAbsClearanceMeters) + "-" + F(SwingMinAbsClearanceMeters) + "-" +
                    F(StanceMaxTangentialSpeed) + "-" + F(StanceMaxNormalSpeed) + "-" +
                    F(SwingMinTangentialSpeed) + "-" + F(SwingMinNormalSpeed) + "-" + F(PhaseConfirmSeconds) + "-" +
                    MinTraceSamples + "-" + MinStanceSamples + "-" + MinSwingSamples +
                "/reason=" + Clean(_lastReason) +
                "/observerOnly=True/contactFreeQualification=True/noPhysicsWrite=True";
        }

        private static bool IsFinite(float value) => !(float.IsNaN(value) || float.IsInfinity(value));
        private static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        private static string Clean(string value) => string.IsNullOrEmpty(value) ? "<none>" : value.Replace('\r', ' ').Replace('\n', ' ');
        private static string F(float value) => IsFinite(value) ? value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture) : "NA";
    }
}
