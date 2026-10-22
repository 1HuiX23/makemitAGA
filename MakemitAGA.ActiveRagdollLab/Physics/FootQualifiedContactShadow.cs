using System;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Acquisition;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.4.2 qualified-static-support Foot contact shadow with persistent source/self-collider isolation.
    ///
    /// Arm A keeps the Foot Collider OFF. Arm B may enable exactly one Foot BoxCollider against exactly one
    /// previously-qualified static support Collider while Gravity remains OFF.  Every active scene Collider that
    /// exists when B is prepared is isolated with Physics.IgnoreCollision except the authorized support pair.
    /// v0.4.1.1 additionally makes Foot↔Foot self-collision a coordinator-owned explicit pair because ARL proxy
    /// objects use HideFlags.DontSave and must not depend on generic scene enumeration coverage.
    /// v0.4.4.2 keeps that exact-support boundary but closes one lifecycle hole: source-character colliders that are
    /// inactive when B is prepared (notably CapsulePlace Mita) are registered proactively, re-audited while B runs,
    /// and may be quarantined on a late callback only when they are positively classified as source/self and the
    /// Physics.IgnoreCollision readback succeeds. Unknown external colliders still fail closed exactly as before.
    /// Any authorized-Collider lifecycle violation or non-zero raycast identity change is still latched invalid.
    ///
    /// This layer still proves only callback/isolation identity. v0.4.3 may consume the already-accepted Collision downstream and combine direct impulse readback with the sole raycast support normal. Neither layer yet claims load bearing, friction or standing readiness.
    /// </summary>
    internal sealed class FootQualifiedContactShadow
    {
        internal const string Mode = "QUALIFIED_STATIC_SUPPORT_FOOT_CONTACT_CALLBACK_SHADOW_V0410";
        internal const string ArmA = "A_V0410_FOOT_COLLIDER_OFF_IDENTITY_BASELINE";
        internal const string ArmB = "B_V0410_EXACT_STATIC_SUPPORT_CONTACT_SHADOW";
        internal const int MinRaycastMatchSamples = 50;
        internal const int MinMatchedCallbacks = 5;
        internal const int SourceIsolationRescanFixedInterval = 30;

        private sealed class OwnedIgnorePair
        {
            internal Collider Foot;
            internal Collider Other;
        }

        private readonly List<OwnedIgnorePair> _ownedIgnorePairs = new List<OwnedIgnorePair>();
        private readonly HashSet<int> _sourceColliderRegistry = new HashSet<int>();
        private readonly List<Collider> _sourceColliderRefs = new List<Collider>();
        private Transform _sourceRoot;
        private FootSupportSurfaceIsolation _supportSurface;
        internal void BindSupportSurface(FootSupportSurfaceIsolation surface) => _supportSurface = surface;
        private int _sourceIsolationRescanCountdown;
        private int _sourceRegistryCount;
        private int _sourceAuxiliaryRegistryCount;
        private int _sourceLateDiscoveries;
        private int _sourceIsolationRepairs;
        private int _sourceIsolationReadbackFailures;
        private int _sourceCallbackEnter;
        private int _sourceCallbackStay;
        private int _sourceCallbackExit;
        private int _sourceCallbacksQuarantined;
        private int _lastSourceColliderId;
        private string _lastSourceColliderPath;
        private string _arm = ArmA;
        private bool _prepared;
        private bool _active;
        private bool _finite = true;
        private string _invalidReason;
        private BoxCollider _footCollider;
        private Collider _authorizedCollider;
        private int _authorizedColliderId;
        private string _authorizedColliderPath;
        private int _sceneColliderCount;
        private int _ownedIsolationPairs;
        private int _alreadyIgnoredPairs;
        private int _isolationFailures;
        private int _restoreFailures;

        private int _traceFixedSamples;
        private int _traceRaycastMatchSamples;
        private int _traceRaycastNoHitSamples;
        private int _traceRaycastMismatchSamples;
        private int _traceEnter;
        private int _traceStay;
        private int _traceExit;
        private int _traceMatchedCallbacks;
        private int _traceUnexpectedCallbacks;
        private bool _contactActive;
        private string _lastCallbackPhase;
        private int _lastCallbackColliderId;
        private string _lastCallbackColliderPath;

        internal string Arm => _arm;
        internal bool Prepared => _prepared;
        internal bool Active => _active;
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;
        internal int AuthorizedColliderId => _authorizedColliderId;
        internal Collider AuthorizedCollider => _authorizedCollider;
        internal bool LastCallbackWasAuthorized { get; private set; }

        internal void SetArmA(BoxCollider footCollider, FootAnkleFoundationReport report)
        {
            int restoreFailures = DisablePreserveTrace(report);
            if (restoreFailures > 0)
            {
                _arm = ArmA;
                Invalidate("CONTACT_SHADOW_RESTORE_FAILED:" + restoreFailures, report);
                CopyToReport(report);
                return;
            }
            _arm = ArmA;
            _prepared = false;
            _active = false;
            _finite = true;
            _invalidReason = null;
            _footCollider = footCollider;
            _authorizedCollider = null;
            _authorizedColliderId = 0;
            _authorizedColliderPath = null;
            _sceneColliderCount = 0;
            _ownedIsolationPairs = 0;
            _alreadyIgnoredPairs = 0;
            _isolationFailures = 0;
            _restoreFailures = 0;
            ResetSourceIsolationState();
            CopyToReport(report);
        }

        internal bool PrepareArmB(BoxCollider footCollider, Collider authorizedCollider, Transform sourceRoot, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            int restoreFailures = DisablePreserveTrace(report);
            if (restoreFailures > 0)
            {
                error = "CONTACT_SHADOW_PREPARE_RESTORE_FAILED:" + restoreFailures;
                Invalidate(error, report);
                CopyToReport(report);
                return false;
            }
            _arm = ArmB;
            _prepared = false;
            _active = false;
            _finite = true;
            _invalidReason = null;
            _footCollider = footCollider;
            _authorizedCollider = authorizedCollider;
            _authorizedColliderId = authorizedCollider != null ? authorizedCollider.GetInstanceID() : 0;
            _authorizedColliderPath = ColliderPath(authorizedCollider);
            _sceneColliderCount = 0;
            _ownedIsolationPairs = 0;
            _alreadyIgnoredPairs = 0;
            _isolationFailures = 0;
            _restoreFailures = 0;
            ResetSourceIsolationState();
            _sourceRoot = sourceRoot;

            if (!ValidateAuthorizedPair(out error))
            {
                Invalidate(error ?? "AUTHORIZED_PAIR_INVALID", report);
                return false;
            }

            try
            {
                GameObject[] all = Object.FindObjectsOfType<GameObject>();
                if (all == null)
                    throw new InvalidOperationException("SCENE_GAMEOBJECT_ENUMERATION_RETURNED_NULL");

                for (int i = 0; i < all.Length; i++)
                {
                    GameObject go = all[i];
                    if (go == null || !go.activeInHierarchy)
                        continue;

                    Collider[] colliders = go.GetComponents<Collider>();
                    if (colliders == null)
                        continue;

                    for (int j = 0; j < colliders.Length; j++)
                    {
                        Collider other = colliders[j];
                        if (other == null)
                            continue;
                        _sceneColliderCount++;
                        if (other == _footCollider || other == _authorizedCollider)
                            continue;

                        bool alreadyIgnored = UnityEngine.Physics.GetIgnoreCollision(_footCollider, other);
                        if (alreadyIgnored)
                        {
                            _alreadyIgnoredPairs++;
                            continue;
                        }

                        UnityEngine.Physics.IgnoreCollision(_footCollider, other, true);
                        _ownedIgnorePairs.Add(new OwnedIgnorePair { Foot = _footCollider, Other = other });
                        _ownedIsolationPairs++;
                    }
                }

                // v0.4.4.2: active scene enumeration above cannot see inactive source proxies. Register the
                // source hierarchy including inactive children and the known motion-activated auxiliary proxy.
                // This does not authorize new contact: it only ensures source/self pairs are ignored.
                RefreshSourceIsolationRegistry(report, "PREPARE", true);

                if (UnityEngine.Physics.GetIgnoreCollision(_footCollider, _authorizedCollider))
                    throw new InvalidOperationException("AUTHORIZED_SUPPORT_PAIR_ALREADY_IGNORED");

                _prepared = true;
                CopyToReport(report);
                return true;
            }
            catch (Exception e)
            {
                _isolationFailures++;
                error = "CONTACT_SHADOW_ISOLATION_PREPARE_FAILED:" + e.GetType().Name + ":" + e.Message;
                RestoreOwnedIgnorePairs();
                Invalidate(error, report);
                return false;
            }
        }

        internal bool EnablePreparedArmB(FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!_prepared || _arm != ArmB)
            {
                error = "CONTACT_SHADOW_NOT_PREPARED";
                Invalidate(error, report);
                return false;
            }
            if (!ValidateAuthorizedPair(out error))
            {
                Invalidate(error ?? "AUTHORIZED_PAIR_INVALID_BEFORE_ENABLE", report);
                return false;
            }

            try
            {
                // v0.4.7 stance-only reuse can leave a prepared contact OFF throughout SWING. Source/self
                // colliders may change active state during that interval, so refresh + read back the persistent
                // source isolation immediately before every stance re-enable.
                RefreshSourceIsolationRegistry(report, "PRE_ENABLE_STANCE_V0470", true);
                if (!AuditRegisteredSourceIsolation(report))
                    throw new InvalidOperationException("SOURCE_COLLIDER_ISOLATION_PRE_ENABLE_READBACK_FAILED");

                _footCollider.isTrigger = false;
                _footCollider.enabled = true;
                _active = true;
                CopyToReport(report);
                return true;
            }
            catch (Exception e)
            {
                error = "CONTACT_SHADOW_ENABLE_FAILED:" + e.GetType().Name + ":" + e.Message;
                Invalidate(error, report);
                DisablePreserveTrace(report);
                return false;
            }
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceFixedSamples = 0;
            _traceRaycastMatchSamples = 0;
            _traceRaycastNoHitSamples = 0;
            _traceRaycastMismatchSamples = 0;
            _traceEnter = 0;
            _traceStay = 0;
            _traceExit = 0;
            _traceMatchedCallbacks = 0;
            _traceUnexpectedCallbacks = 0;
            _sourceCallbackEnter = 0;
            _sourceCallbackStay = 0;
            _sourceCallbackExit = 0;
            _sourceCallbacksQuarantined = 0;
            _contactActive = false;
            _lastCallbackPhase = null;
            _lastCallbackColliderId = 0;
            _lastCallbackColliderPath = null;
            CopyToReport(report);
        }

        internal bool ObserveFixed(bool traceEnabled, FootAnkleFoundationReport report)
        {
            if (_arm != ArmB || !_active)
            {
                CopyToReport(report);
                return _finite;
            }

            string reason;
            if (!ValidateAuthorizedPair(out reason))
            {
                Invalidate(reason ?? "AUTHORIZED_PAIR_INVALID_DURING_RUN", report);
                DisableColliderOnly();
                return false;
            }

            // Existing registered source/self colliders are cheap to verify every FixedUpdate. A broader
            // include-inactive scene scan is throttled and exists only to discover a source proxy instantiated after B prepare.
            if (!AuditRegisteredSourceIsolation(report))
            {
                Invalidate("SOURCE_COLLIDER_ISOLATION_READBACK_FAILED", report);
                DisableColliderOnly();
                return false;
            }
            _sourceIsolationRescanCountdown--;
            if (_sourceIsolationRescanCountdown <= 0)
            {
                _sourceIsolationRescanCountdown = SourceIsolationRescanFixedInterval;
                try { RefreshSourceIsolationRegistry(report, "RUNTIME_SCAN", true); }
                catch (Exception e)
                {
                    _sourceIsolationReadbackFailures++;
                    Invalidate("SOURCE_COLLIDER_RUNTIME_SCAN_FAILED:" + e.GetType().Name + ":" + e.Message, report);
                    DisableColliderOnly();
                    return false;
                }
            }

            if (traceEnabled)
            {
                _traceFixedSamples++;
                int rayId = report != null ? report.GroundColliderInstanceId : 0;
                if (rayId == 0)
                {
                    _traceRaycastNoHitSamples++;
                }
                else if (rayId == _authorizedColliderId)
                {
                    _traceRaycastMatchSamples++;
                }
                else
                {
                    _traceRaycastMismatchSamples++;
                    Invalidate("RAYCAST_IDENTITY_CHANGED:expected=" + _authorizedColliderId + ":actual=" + rayId, report);
                    DisableColliderOnly();
                    return false;
                }
            }

            CopyToReport(report);
            return _finite;
        }

        internal bool NotifyCollision(string phase, Collision collision, FootAnkleFoundationReport report)
        {
            LastCallbackWasAuthorized = false;
            if (_arm != ArmB || !_active)
                return true;

            Collider other = null;
            try { other = collision != null ? collision.collider : null; }
            catch { other = null; }

            int otherId = other != null ? other.GetInstanceID() : 0;
            string otherPath = ColliderPath(other);
            _lastCallbackPhase = phase;
            _lastCallbackColliderId = otherId;
            _lastCallbackColliderPath = otherPath;

            bool matched = other != null && other == _authorizedCollider && otherId == _authorizedColliderId;
            if (matched)
            {
                LastCallbackWasAuthorized = true;
                if (string.Equals(phase, "ENTER", StringComparison.Ordinal)) _traceEnter++;
                else if (string.Equals(phase, "STAY", StringComparison.Ordinal)) _traceStay++;
                else if (string.Equals(phase, "EXIT", StringComparison.Ordinal)) _traceExit++;

                _traceMatchedCallbacks++;
                if (string.Equals(phase, "EXIT", StringComparison.Ordinal))
                    _contactActive = false;
                else
                    _contactActive = true;
                CopyToReport(report);
                return true;
            }

            // v0.4.4.2 late-lifecycle fallback. Only positively identified source/self colliders are eligible.
            // The first callback may already correspond to one solver interaction, so record it separately, apply
            // IgnoreCollision immediately, verify the pair, and never forward this callback to mechanical-reaction
            // scoring. Unknown external colliders retain the historical strict fail-close behavior.
            if (other != null && IsSourceOwnedCollider(other))
            {
                if (string.Equals(phase, "ENTER", StringComparison.Ordinal)) _sourceCallbackEnter++;
                else if (string.Equals(phase, "STAY", StringComparison.Ordinal)) _sourceCallbackStay++;
                else if (string.Equals(phase, "EXIT", StringComparison.Ordinal)) _sourceCallbackExit++;

                bool wasNew = !_sourceColliderRegistry.Contains(otherId);
                if (!EnsureSourceIsolation(other, report, "CALLBACK_QUARANTINE", wasNew))
                {
                    _traceUnexpectedCallbacks++;
                    Invalidate("SOURCE_SELF_CONTACT_QUARANTINE_FAILED:phase=" + phase + ":actual=" + otherId + ":path=" + Clean(otherPath), report);
                    DisableColliderOnly();
                    return false;
                }

                _sourceCallbacksQuarantined++;
                _lastSourceColliderId = otherId;
                _lastSourceColliderPath = otherPath;
                CopyToReport(report);
                return true;
            }

            if (string.Equals(phase, "ENTER", StringComparison.Ordinal)) _traceEnter++;
            else if (string.Equals(phase, "STAY", StringComparison.Ordinal)) _traceStay++;
            else if (string.Equals(phase, "EXIT", StringComparison.Ordinal)) _traceExit++;
            _traceUnexpectedCallbacks++;
            Invalidate("UNAUTHORIZED_CONTACT_CALLBACK:phase=" + phase + ":expected=" + _authorizedColliderId + ":actual=" + otherId + ":path=" + Clean(otherPath), report);
            DisableColliderOnly();
            return false;
        }

        internal bool DisableColliderPreservePreparation(FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (_arm != ArmB || !_prepared)
            {
                error = "CONTACT_SHADOW_NOT_PREPARED_FOR_STANCE_RELEASE";
                return false;
            }

            try
            {
                if (_footCollider != null)
                    _footCollider.enabled = false;
                _active = false;
                _contactActive = false;
                LastCallbackWasAuthorized = false;
                CopyToReport(report);
                return true;
            }
            catch (Exception e)
            {
                error = "CONTACT_SHADOW_STANCE_RELEASE_FAILED:" + e.GetType().Name + ":" + e.Message;
                Invalidate(error, report);
                return false;
            }
        }

        internal int DisablePreserveTrace(FootAnkleFoundationReport report)
        {
            int failures = 0;
            try
            {
                if (_footCollider != null)
                    _footCollider.enabled = false;
            }
            catch { failures++; }
            _active = false;
            _prepared = false;
            _contactActive = false;
            LastCallbackWasAuthorized = false;
            failures += RestoreOwnedIgnorePairs();
            CopyToReport(report);
            return failures;
        }

        internal bool RuntimeContractValid(BoxCollider footCollider)
        {
            if (!_finite)
                return false;
            if (_arm != ArmB || !_active)
                return footCollider == null || !footCollider.enabled;
            // Keep the structural precheck limited to ownership/enable state. Authorized-Collider lifecycle validation
            // is performed immediately afterwards in ObserveFixed(), where a precise reason is latched into the
            // contact-shadow report before the existing bilateral fail-close propagates.
            return footCollider == _footCollider && footCollider != null && footCollider.enabled;
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            bool traceQualified = _finite && _arm == ArmB && _authorizedColliderId != 0 &&
                                  _traceFixedSamples >= MinRaycastMatchSamples &&
                                  _traceRaycastMatchSamples >= MinRaycastMatchSamples &&
                                  _traceRaycastMismatchSamples == 0 &&
                                  _traceUnexpectedCallbacks == 0 &&
                                  _traceMatchedCallbacks >= MinMatchedCallbacks;

            report.ContactShadowMode = Mode;
            report.ContactShadowArm = _arm;
            report.ContactShadowFinite = _finite;
            report.ContactShadowInvalidReason = _invalidReason;
            report.ContactShadowPrepared = _prepared;
            report.ContactShadowActive = _active;
            report.ContactShadowAuthorizedColliderInstanceId = _authorizedColliderId;
            report.ContactShadowAuthorizedColliderPath = _authorizedColliderPath;
            report.ContactShadowSceneColliderCount = _sceneColliderCount;
            report.ContactShadowOwnedIsolationPairs = _ownedIsolationPairs;
            report.ContactShadowAlreadyIgnoredPairs = _alreadyIgnoredPairs;
            report.ContactShadowIsolationFailures = _isolationFailures;
            report.ContactShadowRestoreFailures = _restoreFailures;
            report.ContactShadowSourceRegistryCount = _sourceRegistryCount;
            report.ContactShadowSourceAuxiliaryRegistryCount = _sourceAuxiliaryRegistryCount;
            report.ContactShadowSourceLateDiscoveries = _sourceLateDiscoveries;
            report.ContactShadowSourceIsolationRepairs = _sourceIsolationRepairs;
            report.ContactShadowSourceIsolationReadbackFailures = _sourceIsolationReadbackFailures;
            report.ContactShadowSourceCallbackEnter = _sourceCallbackEnter;
            report.ContactShadowSourceCallbackStay = _sourceCallbackStay;
            report.ContactShadowSourceCallbackExit = _sourceCallbackExit;
            report.ContactShadowSourceCallbacksQuarantined = _sourceCallbacksQuarantined;
            report.ContactShadowLastSourceColliderInstanceId = _lastSourceColliderId;
            report.ContactShadowLastSourceColliderPath = _lastSourceColliderPath;
            report.ContactShadowTraceFixedSamples = _traceFixedSamples;
            report.ContactShadowTraceRaycastMatchSamples = _traceRaycastMatchSamples;
            report.ContactShadowTraceRaycastNoHitSamples = _traceRaycastNoHitSamples;
            report.ContactShadowTraceRaycastMismatchSamples = _traceRaycastMismatchSamples;
            report.ContactShadowTraceEnter = _traceEnter;
            report.ContactShadowTraceStay = _traceStay;
            report.ContactShadowTraceExit = _traceExit;
            report.ContactShadowTraceMatchedCallbacks = _traceMatchedCallbacks;
            report.ContactShadowTraceUnexpectedCallbacks = _traceUnexpectedCallbacks;
            report.ContactShadowCurrentContactActive = _contactActive;
            report.ContactShadowLastCallbackPhase = _lastCallbackPhase;
            report.ContactShadowLastCallbackColliderInstanceId = _lastCallbackColliderId;
            report.ContactShadowLastCallbackColliderPath = _lastCallbackColliderPath;
            report.ContactShadowTraceQualified = traceQualified;
            report.ContactShadowCurrentSummary =
                "arm=" + _arm +
                "/prepared=" + _prepared +
                "/active=" + _active +
                "/authorizedId=" + _authorizedColliderId +
                "/authorizedSurface=" + Clean(_authorizedColliderPath) +
                "/footCollider=" + (_footCollider != null && _footCollider.enabled) +
                "/contactActive=" + _contactActive +
                "/sourceRegistry=" + _sourceRegistryCount +
                "/sourceAux=" + _sourceAuxiliaryRegistryCount +
                "/sourceReadbackFail=" + _sourceIsolationReadbackFailures;
            report.ContactShadowTraceSummary =
                "fixed=" + _traceFixedSamples +
                "/rayMatch=" + _traceRaycastMatchSamples +
                "/rayNoHit=" + _traceRaycastNoHitSamples +
                "/rayMismatch=" + _traceRaycastMismatchSamples +
                "/callback(E-S-X)=" + _traceEnter + "-" + _traceStay + "-" + _traceExit +
                "/matched=" + _traceMatchedCallbacks +
                "/unexpected=" + _traceUnexpectedCallbacks +
                "/sceneColliders=" + _sceneColliderCount +
                "/ownedIgnore=" + _ownedIsolationPairs +
                "/alreadyIgnored=" + _alreadyIgnoredPairs +
                "/sourceRegistry(aux)=" + _sourceRegistryCount + "(" + _sourceAuxiliaryRegistryCount + ")" +
                "/sourceLate=" + _sourceLateDiscoveries +
                "/sourceRepair=" + _sourceIsolationRepairs +
                "/sourceReadbackFail=" + _sourceIsolationReadbackFailures +
                "/sourceCallback(E-S-X)=" + _sourceCallbackEnter + "-" + _sourceCallbackStay + "-" + _sourceCallbackExit +
                "/sourceQuarantined=" + _sourceCallbacksQuarantined +
                "/lastSource=" + _lastSourceColliderId + ":" + Clean(_lastSourceColliderPath) +
                "/isolationFailures=" + _isolationFailures +
                "/restoreFailures=" + _restoreFailures +
                "/last=" + Clean(_lastCallbackPhase) + ":" + _lastCallbackColliderId + ":" + Clean(_lastCallbackColliderPath) +
                "/TRACE_CONTACT_SHADOW_QUALIFIED=" + traceQualified +
                "/thresholds(rayMatch-callback)=" + MinRaycastMatchSamples + "-" + MinMatchedCallbacks +
                "/gravity=False/mechanicalReaction=READ_DOWNSTREAM_V0430_SUPPORT_NORMAL_ONLY";
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.ContactShadowCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.ContactShadowTraceSummary : "<none>");
        }

        private void ResetSourceIsolationState()
        {
            _sourceRoot = null;
            _sourceIsolationRescanCountdown = SourceIsolationRescanFixedInterval;
            _sourceColliderRegistry.Clear();
            _sourceColliderRefs.Clear();
            _sourceRegistryCount = 0;
            _sourceAuxiliaryRegistryCount = 0;
            _sourceLateDiscoveries = 0;
            _sourceIsolationRepairs = 0;
            _sourceIsolationReadbackFailures = 0;
            _sourceCallbackEnter = 0;
            _sourceCallbackStay = 0;
            _sourceCallbackExit = 0;
            _sourceCallbacksQuarantined = 0;
            _lastSourceColliderId = 0;
            _lastSourceColliderPath = null;
            LastCallbackWasAuthorized = false;
        }

        private void RefreshSourceIsolationRegistry(FootAnkleFoundationReport report, string reason, bool includeAuxiliaryResources)
        {
            if (_footCollider == null)
                return;

            if (_sourceRoot != null)
            {
                Collider[] sourceColliders = _sourceRoot.gameObject.GetComponentsInChildren<Collider>(true);
                if (sourceColliders != null)
                {
                    for (int i = 0; i < sourceColliders.Length; i++)
                    {
                        Collider source = sourceColliders[i];
                        if (source == null || source == _footCollider || source == _authorizedCollider)
                            continue;
                        if (!EnsureSourceIsolation(source, report, reason + "_TARGET_ROOT", false))
                            throw new InvalidOperationException("SOURCE_TARGET_ROOT_IGNORE_READBACK_FAILED:" + source.GetInstanceID());
                    }
                }
            }

            if (!includeAuxiliaryResources)
                return;

            Collider[] all = null;
            try { all = Object.FindObjectsOfType<Collider>(true); }
            catch (Exception e) { throw new InvalidOperationException("SOURCE_AUX_ENUMERATION_FAILED", e); }
            if (all == null)
                return;

            _supportSurface?.RegisterSceneColliders(all);
            for (int i = 0; i < all.Length; i++)
            {
                Collider candidate = all[i];
                if (candidate == null || candidate == _footCollider || candidate == _authorizedCollider)
                    continue;
                if (!IsKnownSourceAuxiliaryCollider(candidate))
                    continue;
                if (!EnsureSourceIsolation(candidate, report, reason + "_AUX", true))
                    throw new InvalidOperationException("SOURCE_AUX_IGNORE_READBACK_FAILED:" + candidate.GetInstanceID());
            }
        }

        private bool AuditRegisteredSourceIsolation(FootAnkleFoundationReport report)
        {
            if (_footCollider == null)
                return true;

            // targetRoot children can be recreated while the scene remains loaded; refresh that local hierarchy
            // every FixedUpdate, then verify every retained source/self pair without any global scene enumeration.
            try { RefreshSourceIsolationRegistry(report, "FIXED_AUDIT", false); }
            catch
            {
                _sourceIsolationReadbackFailures++;
                return false;
            }

            for (int i = _sourceColliderRefs.Count - 1; i >= 0; i--)
            {
                Collider source = _sourceColliderRefs[i];
                if (source == null)
                {
                    _sourceColliderRefs.RemoveAt(i);
                    continue;
                }
                if (!EnsureSourceIsolation(source, report, "FIXED_READBACK", IsKnownSourceAuxiliaryCollider(source)))
                    return false;
            }
            return true;
        }

        private bool EnsureSourceIsolation(Collider other, FootAnkleFoundationReport report, string reason, bool auxiliary)
        {
            if (other == null || _footCollider == null || other == _footCollider || other == _authorizedCollider)
                return true;

            int id;
            try { id = other.GetInstanceID(); }
            catch { return false; }
            bool isNew = _sourceColliderRegistry.Add(id);
            if (isNew)
            {
                _sourceColliderRefs.Add(other);
                _sourceRegistryCount++;
                if (auxiliary || IsKnownSourceAuxiliaryCollider(other))
                    _sourceAuxiliaryRegistryCount++;
                if (!string.Equals(reason, "PREPARE_TARGET_ROOT", StringComparison.Ordinal) &&
                    !string.Equals(reason, "PREPARE_AUX", StringComparison.Ordinal))
                    _sourceLateDiscoveries++;
            }

            try
            {
                bool gameObjectActive = other.gameObject != null && other.gameObject.activeInHierarchy;
                bool colliderActive = gameObjectActive && other.enabled;

                // Important lifecycle rule: registration is persistent, IgnoreCollision is not assumed to be.
                // Inactive source proxies are retained in the registry but the pair is only (re)applied/read back
                // once the source Collider is physically active. ObserveFixed runs before the next physics step,
                // which closes the CapsulePlace Mita stop→move activation hole without authorizing new contact.
                if (!colliderActive)
                {
                    if (isNew)
                    {
                        _lastSourceColliderId = id;
                        _lastSourceColliderPath = ColliderPath(other);
                        Plugin.Logger?.LogInfo("[ARL][FOOT_SOURCE_ISOLATION_V0442] side=" + Clean(report != null ? report.Side : null) +
                            " | reason=" + Clean(reason) +
                            " | id=" + id +
                            " | auxiliary=" + (auxiliary || IsKnownSourceAuxiliaryCollider(other)) +
                            " | enabled=" + other.enabled +
                            " | active=" + gameObjectActive +
                            " | ignoreReadback=DEFERRED_INACTIVE | path=" + Clean(_lastSourceColliderPath));
                    }
                    return true;
                }

                bool ignored = UnityEngine.Physics.GetIgnoreCollision(_footCollider, other);
                if (!ignored)
                {
                    UnityEngine.Physics.IgnoreCollision(_footCollider, other, true);
                    _ownedIgnorePairs.Add(new OwnedIgnorePair { Foot = _footCollider, Other = other });
                    _ownedIsolationPairs++;
                    _sourceIsolationRepairs++;
                }

                bool readback = UnityEngine.Physics.GetIgnoreCollision(_footCollider, other);
                if (!readback)
                {
                    _sourceIsolationReadbackFailures++;
                    return false;
                }

                if (isNew)
                {
                    _lastSourceColliderId = id;
                    _lastSourceColliderPath = ColliderPath(other);
                    Plugin.Logger?.LogInfo("[ARL][FOOT_SOURCE_ISOLATION_V0442] side=" + Clean(report != null ? report.Side : null) +
                        " | reason=" + Clean(reason) +
                        " | id=" + id +
                        " | auxiliary=" + (auxiliary || IsKnownSourceAuxiliaryCollider(other)) +
                        " | enabled=" + other.enabled +
                        " | active=" + gameObjectActive +
                        " | ignoreReadback=True | path=" + Clean(_lastSourceColliderPath));
                }
                return true;
            }
            catch (Exception e)
            {
                _sourceIsolationReadbackFailures++;
                Plugin.Logger?.LogWarning("[ARL][FOOT_SOURCE_ISOLATION_V0442][FAILED] side=" + Clean(report != null ? report.Side : null) +
                    " | reason=" + Clean(reason) + " | id=" + id + " | path=" + Clean(ColliderPath(other)) +
                    " | error=" + e.GetType().Name + ":" + e.Message);
                return false;
            }
        }

        private bool IsSourceOwnedCollider(Collider collider)
        {
            if (collider == null || collider.transform == null)
                return false;
            Transform t = collider.transform;
            if (_sourceRoot != null && (t == _sourceRoot || t.IsChildOf(_sourceRoot)))
                return true;
            return IsKnownSourceAuxiliaryCollider(collider);
        }

        private static bool IsKnownSourceAuxiliaryCollider(Collider collider)
        {
            if (collider == null || collider.transform == null)
                return false;
            string path = ColliderPath(collider) ?? string.Empty;
            return path.IndexOf("CapsulePlace Mita", StringComparison.OrdinalIgnoreCase) >= 0 ||
                   path.IndexOf("CapsulePlaceMita", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private bool ValidateAuthorizedPair(out string reason)
        {
            reason = null;
            if (_footCollider == null)
            {
                reason = "FOOT_COLLIDER_MISSING";
                return false;
            }
            if (_authorizedCollider == null)
            {
                reason = "AUTHORIZED_COLLIDER_MISSING";
                return false;
            }
            if (_authorizedCollider.GetInstanceID() != _authorizedColliderId || _authorizedColliderId == 0)
            {
                reason = "AUTHORIZED_COLLIDER_ID_CHANGED";
                return false;
            }
            if (!_authorizedCollider.enabled)
            {
                reason = "AUTHORIZED_COLLIDER_DISABLED";
                return false;
            }
            if (_authorizedCollider.isTrigger)
            {
                reason = "AUTHORIZED_COLLIDER_BECAME_TRIGGER";
                return false;
            }
            if (_authorizedCollider.attachedRigidbody != null)
            {
                reason = "AUTHORIZED_COLLIDER_BECAME_DYNAMIC";
                return false;
            }
            try
            {
                if (UnityEngine.Physics.GetIgnoreCollision(_footCollider, _authorizedCollider))
                {
                    reason = "AUTHORIZED_PAIR_IGNORED";
                    return false;
                }
            }
            catch (Exception e)
            {
                reason = "AUTHORIZED_PAIR_QUERY_FAILED:" + e.GetType().Name;
                return false;
            }
            return true;
        }

        private int RestoreOwnedIgnorePairs()
        {
            int failures = 0;
            for (int i = _ownedIgnorePairs.Count - 1; i >= 0; i--)
            {
                OwnedIgnorePair pair = _ownedIgnorePairs[i];
                try
                {
                    if (pair.Foot != null && pair.Other != null)
                        UnityEngine.Physics.IgnoreCollision(pair.Foot, pair.Other, false);
                    // Drop only successful/dead pairs. An exception retains ownership for retry/rollback.
                    _ownedIgnorePairs.RemoveAt(i);
                }
                catch { failures++; }
            }
            _restoreFailures += failures;
            return failures;
        }

        private void DisableColliderOnly()
        {
            try
            {
                if (_footCollider != null)
                    _footCollider.enabled = false;
            }
            catch { }
            _active = false;
            _contactActive = false;
        }

        private void Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason))
                _invalidReason = reason;
            CopyToReport(report);
        }

        private static string ColliderPath(Collider collider)
        {
            if (collider == null || collider.transform == null)
                return null;
            try { return CharacterTargetFinder.GetPath(collider.transform) + "#" + collider.GetType().Name; }
            catch { return collider.name; }
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace('\r', ' ').Replace('\n', ' ');
        }
    }
}
