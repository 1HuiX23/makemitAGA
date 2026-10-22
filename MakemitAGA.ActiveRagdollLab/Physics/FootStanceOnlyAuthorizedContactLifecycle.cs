using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.7 per-foot stance-only authorized contact lifecycle.
    ///
    /// v0.4.5 proved that the source animation provides repeatable STANCE/SWING phase evidence while
    /// v0.4.4 proved that keeping both Foot colliders planted through native walking is invalid.  This
    /// layer therefore reuses the frozen v0.4.5 classifier as a phase witness and the closed v0.4.1-v0.4.4
    /// exact-support contact stack as the mechanical primitive, but allows contact only for a confirmed
    /// STANCE on an exact currently-qualified static support Collider.
    ///
    /// Important timing rule: the first strong swing sample changes the gait state to TRANSITION_TO_SWING;
    /// this lifecycle sees that state in the same FixedUpdate and disables the Foot Collider before the
    /// remaining Foot controller / Unity physics step.  Landing is deliberately slower: contact is enabled
    /// only after confirmed STANCE plus a short same-Collider qualification hold.
    ///
    /// This class does not apply force/torque, does not raycast, does not alter gait thresholds and does not
    /// relax any existing Foot/Knee hard gate.  It only orchestrates the already-qualified contact shadow.
    /// </summary>
    internal sealed class FootStanceOnlyAuthorizedContactLifecycle
    {
        internal const string Mode = "STANCE_ONLY_AUTHORIZED_FOOT_CONTACT_LIFECYCLE_V0470_HOTFIX_V04161_ACTIVATION_READINESS";
        internal const float MinQualifiedStanceHoldSeconds = 0.08f;
        internal const int MinContactActivations = 2;
        internal const int MinContactReleases = 1;
        internal const int MinActiveSamples = 10;
        // v0.4.16.1: an exact support identity may remain historically qualified while the physical
        // sole is still catching up to the source pose.  Require two consecutive current-geometry-ready
        // FixedUpdates before enabling the collider so the first collision callback cannot inherit a
        // known-invalid support-normal witness from the activation boundary.
        internal const int MinActivationGeometryReadySamples = 2;

        private const string StateDisabled = "DISABLED";
        private const string StateWaiting = "WAIT_STANCE";
        private const string StateQualifying = "QUALIFY_STANCE_IDENTITY";
        private const string StatePrepared = "PREPARED_CONTACT_OFF";
        private const string StateActive = "STANCE_CONTACT_ACTIVE";
        private const string StateInvalid = "INVALID";

        private bool _enabled;
        private bool _finite = true;
        private string _invalidReason;
        private string _state = StateDisabled;
        private string _lastReason = "<none>";
        private float _qualifiedStanceSeconds;
        private int _qualifiedStanceColliderId;
        private string _qualifiedStanceColliderPath;
        private int _preparedColliderId;
        private string _preparedColliderPath;

        private int _fixedSamples;
        private int _activeSamples;
        private int _contactActivations;
        private int _prepareCount;
        private int _preparedReuseCount;
        private int _surfaceReprepareCount;
        private int _releaseCount;
        private int _transitionReleaseCount;
        private int _swingReleaseCount;
        private int _identityLossReleaseCount;
        private int _surfaceChangeReleaseCount;
        private int _identityRejectSamples;
        private int _activationGeometryRejectSamples;
        private int _activationGeometryReadySamples;
        private int _nonStanceActiveSamples;
        private int _enableFailures;
        private int _disableFailures;
        private int _firstContactFixed;
        private int _lastContactFixed;
        private int _lastReleaseFixed;

        internal bool Enabled => _enabled;
        internal bool Finite => _finite;
        internal bool ContactActive => _state == StateActive;

        internal void SetEnabled(bool enabled, FootQualifiedContactShadow shadow, BoxCollider footCollider, FootAnkleFoundationReport report)
        {
            _enabled = enabled;
            _finite = true;
            _invalidReason = null;
            _state = enabled ? StateWaiting : StateDisabled;
            _lastReason = enabled ? "WAITING_FOR_CONFIRMED_STANCE" : "DISABLED";
            _qualifiedStanceSeconds = 0f;
            _qualifiedStanceColliderId = 0;
            _qualifiedStanceColliderPath = null;
            _activationGeometryReadySamples = 0;
            _preparedColliderId = 0;
            _preparedColliderPath = null;

            if (!enabled && shadow != null)
                shadow.SetArmA(footCollider, report);
            CopyToReport(report);
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _fixedSamples = 0;
            _activeSamples = 0;
            _contactActivations = 0;
            _prepareCount = 0;
            _preparedReuseCount = 0;
            _surfaceReprepareCount = 0;
            _releaseCount = 0;
            _transitionReleaseCount = 0;
            _swingReleaseCount = 0;
            _identityLossReleaseCount = 0;
            _surfaceChangeReleaseCount = 0;
            _identityRejectSamples = 0;
            _activationGeometryRejectSamples = 0;
            _activationGeometryReadySamples = 0;
            _nonStanceActiveSamples = 0;
            _enableFailures = 0;
            _disableFailures = 0;
            _firstContactFixed = 0;
            _lastContactFixed = 0;
            _lastReleaseFixed = 0;
            _qualifiedStanceSeconds = 0f;
            _qualifiedStanceColliderId = 0;
            _qualifiedStanceColliderPath = null;
            if (_enabled && _finite)
            {
                _state = StateWaiting;
                _lastReason = "WAITING_FOR_CONFIRMED_STANCE";
            }
            CopyToReport(report);
        }

        internal bool Step(
            bool traceEnabled,
            int fixedSequence,
            float fixedDeltaTime,
            string gaitState,
            bool strongStanceSample,
            bool identityCurrentQualified,
            int identityColliderId,
            string identityColliderPath,
            Collider currentGroundCollider,
            FootQualifiedContactShadow shadow,
            BoxCollider footCollider,
            Transform sourceRoot,
            FootAnkleFoundationReport report)
        {
            if (!_enabled)
            {
                CopyToReport(report);
                return true;
            }
            if (!_finite || shadow == null || footCollider == null)
            {
                Invalidate(_invalidReason ?? "STANCE_CONTACT_DEPENDENCY_MISSING", report);
                return false;
            }

            if (traceEnabled) _fixedSamples++;
            float dt = IsFinite(fixedDeltaTime) && fixedDeltaTime > 0f ? fixedDeltaTime : 0.02f;
            bool confirmedStance = string.Equals(gaitState, FootGameNativeLocomotionPhaseObserver.StateStance, StringComparison.Ordinal);
            bool transitionToSwing = string.Equals(gaitState, FootGameNativeLocomotionPhaseObserver.StateToSwing, StringComparison.Ordinal);
            bool swing = string.Equals(gaitState, FootGameNativeLocomotionPhaseObserver.StateSwing, StringComparison.Ordinal);
            bool anyTransition = string.Equals(gaitState, FootGameNativeLocomotionPhaseObserver.StateToSwing, StringComparison.Ordinal) ||
                                 string.Equals(gaitState, FootGameNativeLocomotionPhaseObserver.StateToStance, StringComparison.Ordinal);

            if (shadow.Active)
            {
                // Contact authority is stricter than the hysteretic phase label: the current source/sole sample
                // must still satisfy the frozen strong-STANCE predicate. The first sample that leaves that
                // predicate releases contact even if the classifier remains STANCE_CANDIDATE inside hysteresis.
                if (!confirmedStance || !strongStanceSample)
                {
                    if (!ReleasePreparedContact(shadow, report, out string releaseError))
                    {
                        _disableFailures++;
                        Invalidate("STANCE_CONTACT_RELEASE_FAILED:" + Clean(releaseError), report);
                        return false;
                    }
                    _releaseCount++;
                    _lastReleaseFixed = fixedSequence;
                    if (transitionToSwing || anyTransition) _transitionReleaseCount++;
                    if (swing) _swingReleaseCount++;
                    _state = StatePrepared;
                    _lastReason = transitionToSwing ? "RELEASE_ON_TRANSITION_TO_SWING" : (swing ? "RELEASE_ON_SWING" : (!strongStanceSample ? "RELEASE_ON_STRONG_STANCE_EXIT" : "RELEASE_ON_NON_STANCE"));
                    Plugin.Logger?.LogInfo("[ARL][STANCE_CONTACT_V0470] side=" + Clean(report != null ? report.Side : null) +
                                           " | action=RELEASE | fixed=" + fixedSequence + " | gait=" + Clean(gaitState) +
                                           " | reason=" + _lastReason + " | authId=" + _preparedColliderId);
                    ResetStanceHold();
                    CopyToReport(report);
                    return true;
                }

                if (!IdentityMatchesPrepared(identityCurrentQualified, identityColliderId, currentGroundCollider, shadow))
                {
                    bool surfaceChanged = identityColliderId != 0 && _preparedColliderId != 0 && identityColliderId != _preparedColliderId;
                    if (!ReleasePreparedContact(shadow, report, out string releaseError))
                    {
                        _disableFailures++;
                        Invalidate("STANCE_CONTACT_IDENTITY_RELEASE_FAILED:" + Clean(releaseError), report);
                        return false;
                    }
                    _releaseCount++;
                    _lastReleaseFixed = fixedSequence;
                    if (surfaceChanged) _surfaceChangeReleaseCount++;
                    else _identityLossReleaseCount++;
                    _lastReason = surfaceChanged ? "RELEASE_ON_SUPPORT_ID_CHANGE" : "RELEASE_ON_IDENTITY_LOSS";
                    _state = StatePrepared;
                    LogIdentityRelease(fixedSequence, gaitState, identityCurrentQualified, identityColliderId, currentGroundCollider, shadow, report);
                    ResetStanceHold();
                    CopyToReport(report);
                    return true;
                }

                if (traceEnabled)
                {
                    _activeSamples++;
                    _lastContactFixed = fixedSequence;
                    if (!confirmedStance || !strongStanceSample) _nonStanceActiveSamples++;
                }
                _state = StateActive;
                _lastReason = "STANCE_CONTACT_HELD";
                CopyToReport(report);
                return true;
            }

            // Contact is OFF. Any transition/swing state immediately resets the landing hold.
            if (!confirmedStance || !strongStanceSample)
            {
                ResetStanceHold();
                _state = shadow.Prepared ? StatePrepared : StateWaiting;
                _lastReason = swing ? "SWING_CONTACT_OFF" : (anyTransition ? "TRANSITION_CONTACT_OFF" : (!strongStanceSample ? "STRONG_STANCE_SAMPLE_REQUIRED" : "WAITING_FOR_CONFIRMED_STANCE"));
                CopyToReport(report);
                return true;
            }

            // Confirmed stance still needs an exact v0.4.0 current identity witness before contact is eligible.
            if (!identityCurrentQualified || identityColliderId == 0 || currentGroundCollider == null || currentGroundCollider.GetInstanceID() != identityColliderId)
            {
                if (traceEnabled) _identityRejectSamples++;
                ResetStanceHold();
                _state = StateQualifying;
                _lastReason = "STANCE_IDENTITY_NOT_CURRENTLY_QUALIFIED";
                CopyToReport(report);
                return true;
            }

            if (_qualifiedStanceColliderId != identityColliderId)
            {
                _qualifiedStanceColliderId = identityColliderId;
                _qualifiedStanceColliderPath = identityColliderPath;
                _qualifiedStanceSeconds = 0f;
                _activationGeometryReadySamples = 0;
            }
            else
            {
                _qualifiedStanceSeconds += dt;
            }

            _state = StateQualifying;
            _lastReason = "STANCE_IDENTITY_HOLD";
            if (_qualifiedStanceSeconds < MinQualifiedStanceHoldSeconds)
            {
                _activationGeometryReadySamples = 0;
                CopyToReport(report);
                return true;
            }

            // v0.4.16.1 activation-readiness hotfix.  Contact identity is intentionally not sufficient here:
            // v0.4.16 exposed a landing where the same floor identity was valid but the physical Foot sole was
            // still ~0.11 m away from its target and the five-point support geometry stayed invalid for three
            // collision callbacks.  Gate only the OFF->ON transition on the current physical sole witness.
            // Once contact is active, the existing lifecycle/reaction observers retain their original semantics.
            bool activationGeometryReady = ActivationGeometryReady(report, identityColliderId, currentGroundCollider);
            if (!activationGeometryReady)
            {
                if (traceEnabled) _activationGeometryRejectSamples++;
                _activationGeometryReadySamples = 0;
                _state = StateQualifying;
                _lastReason = "STANCE_ACTIVATION_GEOMETRY_NOT_READY";
                CopyToReport(report);
                return true;
            }

            _activationGeometryReadySamples++;
            if (_activationGeometryReadySamples < MinActivationGeometryReadySamples)
            {
                _state = StateQualifying;
                _lastReason = "STANCE_ACTIVATION_GEOMETRY_READY_HOLD";
                CopyToReport(report);
                return true;
            }

            // Reuse a prepared exact support whenever the foot returns to the same floor. Reprepare only when the
            // authorized support identity changes. This avoids a full scene-isolation rebuild every stride.
            if (!shadow.Prepared || shadow.AuthorizedColliderId != identityColliderId)
            {
                if (shadow.Prepared && shadow.AuthorizedColliderId != identityColliderId)
                    _surfaceReprepareCount++;

                if (!shadow.PrepareArmB(footCollider, currentGroundCollider, sourceRoot, report, out string prepareError))
                {
                    Invalidate("STANCE_CONTACT_PREPARE_FAILED:" + Clean(prepareError), report);
                    return false;
                }
                _prepareCount++;
                _preparedColliderId = identityColliderId;
                _preparedColliderPath = identityColliderPath;
            }
            else
            {
                _preparedReuseCount++;
                _preparedColliderId = identityColliderId;
                _preparedColliderPath = identityColliderPath;
            }

            if (!shadow.EnablePreparedArmB(report, out string enableError))
            {
                _enableFailures++;
                Invalidate("STANCE_CONTACT_ENABLE_FAILED:" + Clean(enableError), report);
                return false;
            }

            _contactActivations++;
            if (_firstContactFixed == 0) _firstContactFixed = fixedSequence;
            _lastContactFixed = fixedSequence;
            _state = StateActive;
            _lastReason = "STANCE_CONTACT_ENABLED";
            Plugin.Logger?.LogInfo("[ARL][STANCE_CONTACT_V0470] side=" + Clean(report != null ? report.Side : null) +
                                   " | action=ENABLE | fixed=" + fixedSequence + " | gait=" + Clean(gaitState) +
                                   " | authId=" + identityColliderId + " | holdSec=" + F(_qualifiedStanceSeconds) +
                                   " | geometryReadyHold=" + _activationGeometryReadySamples + "/" + MinActivationGeometryReadySamples +
                                   " | surface=" + Clean(identityColliderPath));
            CopyToReport(report);
            return true;
        }

        internal void EndTrace(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
        }

        internal bool QuiescePreserveTrace(FootQualifiedContactShadow shadow, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (shadow != null && shadow.Active)
            {
                if (!shadow.DisableColliderPreservePreparation(report, out error))
                    return false;
            }
            _enabled = false;
            _state = shadow != null && shadow.Prepared ? StatePrepared : StateDisabled;
            _lastReason = "STOP_QUIESCE_CONTACT_OFF_TRACE_PRESERVED";
            CopyToReport(report);
            return true;
        }

        internal void DisableAndReset(FootQualifiedContactShadow shadow, BoxCollider footCollider, FootAnkleFoundationReport report)
        {
            if (shadow != null)
                shadow.SetArmA(footCollider, report);
            _enabled = false;
            _state = StateDisabled;
            _lastReason = "DISABLED";
            _qualifiedStanceSeconds = 0f;
            _qualifiedStanceColliderId = 0;
            _qualifiedStanceColliderPath = null;
            // Preserve the last authorized support identity in the trace summary after stop. A subsequent
            // SetEnabled() starts a fresh run and clears it.
            CopyToReport(report);
        }

        private bool ReleasePreparedContact(FootQualifiedContactShadow shadow, FootAnkleFoundationReport report, out string error)
        {
            return shadow.DisableColliderPreservePreparation(report, out error);
        }

        private bool IdentityMatchesPrepared(bool identityQualified, int identityColliderId, Collider currentGroundCollider, FootQualifiedContactShadow shadow)
        {
            return identityQualified && identityColliderId != 0 && _preparedColliderId != 0 &&
                   identityColliderId == _preparedColliderId && currentGroundCollider != null &&
                   currentGroundCollider.GetInstanceID() == _preparedColliderId &&
                   shadow.AuthorizedColliderId == _preparedColliderId;
        }

        // Event-only diagnostics after the existing release. Geometry qualification loss is distinct from
        // an entity change; these fields expose the frozen predicate without influencing contact authority.
        private void LogIdentityRelease(int fixedSequence, string gaitState, bool identityCurrentQualified,
            int identityColliderId, Collider currentGroundCollider, FootQualifiedContactShadow shadow, FootAnkleFoundationReport report)
        {
            try
            {
                string evidence = report == null ? " | supportEvidence=NA" :
                    " | probeId=" + report.GroundColliderInstanceId + "/hits=" + report.GroundProbeHitCount +
                    "/sameSurface=" + report.GroundFootprintSameSurface + "/static=" + !report.GroundColliderHasAttachedRigidbody +
                    "/minNormalDot=" + F(report.GroundFootprintMinNormalDotReferenceUp) +
                    "/centerClearanceMm=" + F(report.GroundCenterClearanceMeters * 1000f) +
                    "/spreadMm=" + F(report.GroundClearanceSpreadMeters * 1000f) + "/geometryReady=" + report.GroundGeometryReady +
                    " | identityThresholds(hits-minNormalDot-maxAbsCenterMm-maxSpreadMm)=5-" +
                    F(FootGroundContactIdentityQualificationObserver.MinNormalDot) + "-" +
                    F(FootGroundContactIdentityQualificationObserver.MaxAbsClearanceMeters * 1000f) + "-" +
                    F(FootGroundContactIdentityQualificationObserver.MaxSpreadMeters * 1000f);
                Plugin.Logger?.LogInfo("[ARL][STANCE_CONTACT_V0470] side=" + Clean(report != null ? report.Side : null) +
                    " | action=RELEASE | fixed=" + fixedSequence + " | gait=" + Clean(gaitState) + " | reason=" + _lastReason +
                    " | authId=" + _preparedColliderId + "/identityId=" + identityColliderId +
                    "/currentGroundId=" + (currentGroundCollider != null ? currentGroundCollider.GetInstanceID() : 0) +
                    "/shadowAuthId=" + (shadow != null ? shadow.AuthorizedColliderId : 0) +
                    "/identityCurrentQualified=" + identityCurrentQualified + evidence);
            }
            catch (Exception) { } // a diagnostic getter/logger failure must not change the completed release
        }

        private void ResetStanceHold()
        {
            _qualifiedStanceSeconds = 0f;
            _qualifiedStanceColliderId = 0;
            _qualifiedStanceColliderPath = null;
            _activationGeometryReadySamples = 0;
        }

        private static bool ActivationGeometryReady(FootAnkleFoundationReport report, int identityColliderId, Collider currentGroundCollider)
        {
            if (report == null || identityColliderId == 0 || currentGroundCollider == null) return false;
            Vector3 normal = report.GroundAverageNormalWorld;
            return report.GroundGeometryReady &&
                   report.GroundFootprintSameSurface &&
                   report.GroundColliderInstanceId == identityColliderId &&
                   currentGroundCollider.GetInstanceID() == identityColliderId &&
                   IsFinite(normal.x) && IsFinite(normal.y) && IsFinite(normal.z) &&
                   normal.sqrMagnitude > 0.00000001f;
        }

        private void Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            _state = StateInvalid;
            _lastReason = reason;
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            bool traceQualified = _finite &&
                                  _contactActivations >= MinContactActivations &&
                                  _releaseCount >= MinContactReleases &&
                                  _activeSamples >= MinActiveSamples &&
                                  _nonStanceActiveSamples == 0 &&
                                  _enableFailures == 0 && _disableFailures == 0;

            report.StanceContactLifecycleMode = Mode;
            report.StanceContactLifecycleEnabled = _enabled;
            report.StanceContactLifecycleFinite = _finite;
            report.StanceContactLifecycleInvalidReason = _invalidReason;
            report.StanceContactLifecycleState = _state;
            report.StanceContactLifecycleLastReason = _lastReason;
            report.StanceContactLifecycleQualifiedStanceSeconds = _qualifiedStanceSeconds;
            report.StanceContactLifecycleQualifiedColliderInstanceId = _qualifiedStanceColliderId;
            report.StanceContactLifecycleQualifiedColliderPath = _qualifiedStanceColliderPath;
            report.StanceContactLifecycleAuthorizedColliderInstanceId = _preparedColliderId;
            report.StanceContactLifecycleAuthorizedColliderPath = _preparedColliderPath;
            report.StanceContactLifecycleFixedSamples = _fixedSamples;
            report.StanceContactLifecycleActiveSamples = _activeSamples;
            report.StanceContactLifecycleContactActivations = _contactActivations;
            report.StanceContactLifecyclePrepareCount = _prepareCount;
            report.StanceContactLifecyclePreparedReuseCount = _preparedReuseCount;
            report.StanceContactLifecycleSurfaceReprepareCount = _surfaceReprepareCount;
            report.StanceContactLifecycleReleaseCount = _releaseCount;
            report.StanceContactLifecycleTransitionReleaseCount = _transitionReleaseCount;
            report.StanceContactLifecycleSwingReleaseCount = _swingReleaseCount;
            report.StanceContactLifecycleIdentityLossReleaseCount = _identityLossReleaseCount;
            report.StanceContactLifecycleSurfaceChangeReleaseCount = _surfaceChangeReleaseCount;
            report.StanceContactLifecycleIdentityRejectSamples = _identityRejectSamples;
            report.StanceContactLifecycleActivationGeometryRejectSamples = _activationGeometryRejectSamples;
            report.StanceContactLifecycleActivationGeometryReadySamples = _activationGeometryReadySamples;
            report.StanceContactLifecycleNonStanceActiveSamples = _nonStanceActiveSamples;
            report.StanceContactLifecycleEnableFailures = _enableFailures;
            report.StanceContactLifecycleDisableFailures = _disableFailures;
            report.StanceContactLifecycleFirstContactFixed = _firstContactFixed;
            report.StanceContactLifecycleLastContactFixed = _lastContactFixed;
            report.StanceContactLifecycleLastReleaseFixed = _lastReleaseFixed;
            report.StanceContactLifecycleTraceQualified = traceQualified;
            report.StanceContactLifecycleCurrentSummary =
                "enabled=" + _enabled + "/finite=" + _finite + "/state=" + _state +
                "/gait=" + Clean(report.GameNativeGaitPhaseState) +
                "/shadowPreparedActive=" + report.ContactShadowPrepared + "-" + report.ContactShadowActive +
                "/qualifiedHoldSec=" + F(_qualifiedStanceSeconds) +
                "/candidateId=" + _qualifiedStanceColliderId +
                "/activationReadyHold=" + _activationGeometryReadySamples + "/" + MinActivationGeometryReadySamples +
                "/authorizedId=" + _preparedColliderId +
                "/last=" + Clean(_lastReason);
            report.StanceContactLifecycleTraceSummary =
                "fixed=" + _fixedSamples + "/active=" + _activeSamples +
                "/activation=" + _contactActivations +
                "/prepare-reuse-reprepare=" + _prepareCount + "-" + _preparedReuseCount + "-" + _surfaceReprepareCount +
                "/release(total-transition-swing-idLoss-surfaceChange)=" + _releaseCount + "-" + _transitionReleaseCount + "-" + _swingReleaseCount + "-" + _identityLossReleaseCount + "-" + _surfaceChangeReleaseCount +
                "/identityReject=" + _identityRejectSamples +
                "/activationGeometryReject=" + _activationGeometryRejectSamples +
                "/activationReadyHold=" + _activationGeometryReadySamples + "/" + MinActivationGeometryReadySamples +
                "/nonStanceActive=" + _nonStanceActiveSamples +
                "/fail(enable-disable)=" + _enableFailures + "-" + _disableFailures +
                "/firstContact-lastContact-lastRelease=" + _firstContactFixed + "-" + _lastContactFixed + "-" + _lastReleaseFixed +
                "/TRACE_STANCE_CONTACT_LIFECYCLE_QUALIFIED=" + traceQualified +
                "/thresholds(identityHoldSec-geometryReadySamples-activation-release-activeSamples)=" + F(MinQualifiedStanceHoldSeconds) + "-" + MinActivationGeometryReadySamples + "-" + MinContactActivations + "-" + MinContactReleases + "-" + MinActiveSamples;
        }

        private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace('\r', ' ').Replace('\n', ' ');
        }
    }
}
