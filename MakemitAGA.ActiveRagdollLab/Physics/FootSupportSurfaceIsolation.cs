using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Il2CppInterop.Runtime;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04410: one pinned native floor versus a static 30 cm Box with the same top surface.
    /// Both arms pin the existing sole probes to their exact selected Collider. B also queries the
    /// original floor, so a Box must not invent support across holes, stairs or a room boundary.
    /// Original colliders/materials/transforms and the global collision matrix are never written.
    /// Only the two Foot colliders may contact the owned Box; existing Foot contact lifecycle still
    /// owns stance activation and Foot-to-original-floor isolation. This is not a GSP promotion.
    /// </summary>
    internal sealed class FootSupportSurfaceIsolation
    {
        internal const string Mode = "PINNED_ORIGINAL_VS_STATIC_BOX_SUPPORT_V04410";
        internal const string ArmA = "A_V04410_ORIGINAL_SUPPORT";
        internal const string ArmB = "B_V04410_STATIC_BOX_SUPPORT";
        internal const string IdleArm = "IDLE_V04410_NO_SUPPORT_OVERRIDE";
        internal const string CallbackSide = "V04410_SUPPORT_BOX";
        internal const float ThicknessMeters = 0.30f;
        internal const float PlaneToleranceMeters = 0.0002f;
        internal const int MinPairSamples = 50;

        private readonly List<Collider> _others = new List<Collider>();
        private readonly HashSet<int> _registered = new HashSet<int>();
        private GameObject _boxObject;
        private BoxCollider _box;
        private Collider _original, _left, _right;
        private PhysicMaterial _material;
        private Vector3 _originalPosition, _originalScale, _boundsCenter, _boundsExtents, _boxPosition, _boxSize;
        private Quaternion _originalRotation;
        private float _offset, _height;
        private int _layer, _originalId, _selectedId;
        private string _originalPath, _originalType, _arm = IdleArm, _reason;
        private string _originalFailureEvidence;
        private bool _active, _finite = true, _stoppedSafe = true;
        private int _pairLeft, _pairRight, _probeMatches, _originalMisses, _isolationRepairs, _unexpectedCallbacks;
        private int _boxLeftCallbacks, _boxRightCallbacks;

        internal bool Active => _active;
        internal bool Finite => _finite;
        internal bool StoppedSafe => !_active && _stoppedSafe && _box == null && _boxObject == null;
        internal string Arm => _arm;
        internal string InvalidReason => _reason;
        internal Collider SelectedCollider => _arm == ArmB ? _box : _original;
        internal int OriginalId => _originalId;
        internal int SelectedId => _selectedId;
        internal bool ExposureReady => _finite && _pairLeft >= MinPairSamples && _pairRight >= MinPairSamples &&
            _probeMatches > 0 && (_arm != ArmB || (_boxLeftCallbacks > 0 && _boxRightCallbacks > 0));
        internal static bool IsExperimentArm(string arm) => arm == ArmA || arm == ArmB;

        internal bool Configure(string arm, Collider original, Collider otherGround, Collider left, Collider right,
            FootAnkleFoundationReport leftReport, FootAnkleFoundationReport rightReport, out string error)
        {
            error = null;
            if (!StoppedSafe) return Fail("PREVIOUS_SUPPORT_NOT_RELEASED", out error);
            _finite = true; _reason = null; _originalFailureEvidence = null;
            ResetExposure();
            if (!IsExperimentArm(arm)) return Fail("UNKNOWN_SUPPORT_ARM", out error);
            if (original == null || otherGround != original || left == null || right == null || left == right ||
                left.enabled || right.enabled || !ProbeReady(leftReport) || !ProbeReady(rightReport))
                return Fail("REQUIRES_QUIESCED_FEET_AND_SAME_QUALIFIED_FLOOR", out error);
            _arm = arm; _original = original; _left = left; _right = right;
            try
            {
                if (!original.enabled || !original.gameObject.activeInHierarchy || original.isTrigger || original.attachedRigidbody != null)
                    return Fail("ORIGINAL_SUPPORT_NOT_ACTIVE_STATIC_SOLID", out error);
                _originalId = original.GetInstanceID();
                if (leftReport.GroundColliderInstanceId != _originalId || rightReport.GroundColliderInstanceId != _originalId)
                    return Fail("ORIGINAL_PROBE_ID_MISMATCH", out error);
                _originalPath = leftReport.GroundColliderPath;
                _originalType = original.TryCast<BoxCollider>() != null ? "BoxCollider" :
                    (original.TryCast<MeshCollider>() != null ? "MeshCollider" : original.GetType().Name);
                _originalPosition = original.transform.position; _originalRotation = original.transform.rotation;
                _originalScale = original.transform.lossyScale;
                Bounds bounds = original.bounds;
                _boundsCenter = bounds.center; _boundsExtents = bounds.extents;
                _layer = original.gameObject.layer; _material = original.sharedMaterial; _offset = original.contactOffset;
                if (!FiniteVector(_boundsCenter) || !FiniteVector(_boundsExtents) ||
                    _boundsExtents.x < 0.5f || _boundsExtents.z < 0.5f || _boundsExtents.x > 20f || _boundsExtents.z > 20f ||
                    !FiniteFloat(_offset) || _offset <= 0f)
                    return Fail("ORIGINAL_FLOOR_BOUNDS_OR_OFFSET_INVALID", out error);
                Vector3 center = (left.transform.position + right.transform.position) * 0.5f;
                Ray ray = new Ray(new Vector3(center.x, bounds.max.y + 1f, center.z), Vector3.down);
                if (!original.Raycast(ray, out RaycastHit hit, 2f + bounds.size.y) || !Horizontal(hit.normal))
                    return Fail("REQUIRES_HORIZONTAL_ORIGINAL_FLOOR", out error);
                _height = hit.point.y;
                if (!FiniteFloat(_height)) return Fail("NONFINITE_SUPPORT_HEIGHT", out error);
                _boxSize = new Vector3(bounds.size.x, ThicknessMeters, bounds.size.z);
                _boxPosition = new Vector3(bounds.center.x, _height - ThicknessMeters * 0.5f, bounds.center.z);
                if (arm == ArmB)
                {
                    // Disabled shape construction cannot enter a solve; enable and isolate synchronously
                    // before returning. No yield, Simulate, global layer change or game-object mutation.
                    _stoppedSafe = false;
                    _boxObject = new GameObject("ARL_V04410_ControlledSupport");
                    _boxObject.hideFlags = HideFlags.DontSave;
                    _boxObject.SetActive(false);
                    _boxObject.layer = _layer;
                    _boxObject.transform.position = _boxPosition;
                    _boxObject.transform.rotation = Quaternion.identity;
                    _boxObject.transform.localScale = Vector3.one;
                    _box = Il2CppComponentFactory.Add<BoxCollider>(_boxObject, "V04410_SUPPORT_BOX");
                    _box.enabled = false; _box.isTrigger = false;
                    _box.center = Vector3.zero; _box.size = _boxSize;
                    _box.sharedMaterial = _material;
                    var tracker = Il2CppComponentFactory.Add<FootContactShadowTracker>(_boxObject, "V04410_SUPPORT_CALLBACK");
                    tracker.Initialize(CallbackSide);
                    _boxObject.SetActive(true);
                    _box.enabled = true;
                    // contactOffset writes on disabled shapes were not reliable in the v0.4.39 target.
                    // This newly owned static shape is enabled only in this non-yielding setup section.
                    _box.contactOffset = _offset;
                    UnityEngine.Physics.SyncTransforms();
                    RegisterSceneColliders(Object.FindObjectsOfType<Collider>(true));
                    if (!AuditBoxIsolation(out error)) return false;
                    if (!_box.Raycast(ray, out RaycastHit boxHit, 2f + bounds.size.y) || !MatchingPlane(hit, boxHit))
                        return Fail("BOX_TOP_ALIGNMENT_READBACK_FAILED", out error);
                }
                _selectedId = SelectedCollider.GetInstanceID();
                _active = true; _stoppedSafe = false;
                return Validate(out error);
            }
            catch (Exception e) { return Fail("SUPPORT_SETUP_EXCEPTION:" + e.GetType().Name + ":" + e.Message, out error); }
        }

        // Reuse the contact stack's existing scene inventory. No new periodic global enumeration.
        internal void RegisterSceneColliders(Collider[] colliders)
        {
            if (_box == null || colliders == null) return;
            for (int i = 0; i < colliders.Length; i++)
            {
                Collider c = colliders[i];
                if (c == null || c == _box || c == _left || c == _right) continue;
                if (_registered.Add(c.GetInstanceID()))
                {
                    _others.Add(c);
                    // An inventory refresh can occur after the coordinator's pre-step audit.
                    // Isolate a newly discovered active pair before that same solve, not one Fixed later.
                    if (c.enabled && c.gameObject.activeInHierarchy)
                    {
                        UnityEngine.Physics.IgnoreCollision(_box, c, true);
                        if (!UnityEngine.Physics.GetIgnoreCollision(_box, c))
                            Fail("BOX_NEW_PAIR_ISOLATION_FAILED:" + c.GetInstanceID(), out _);
                    }
                }
            }
        }

        private bool AuditBoxIsolation(out string error)
        {
            error = null;
            if (_box == null) return Fail("OWNED_BOX_MISSING", out error);
            for (int i = _others.Count - 1; i >= 0; i--)
            {
                Collider c = _others[i];
                if (c == null) { _others.RemoveAt(i); continue; }
                // Inactive colliders retain registration, not an assumed IgnoreCollision state.
                if (!c.enabled || !c.gameObject.activeInHierarchy) continue;
                if (!UnityEngine.Physics.GetIgnoreCollision(_box, c))
                {
                    UnityEngine.Physics.IgnoreCollision(_box, c, true);
                    _isolationRepairs++;
                }
                if (!UnityEngine.Physics.GetIgnoreCollision(_box, c))
                    return Fail("BOX_OTHER_PAIR_ISOLATION_FAILED:" + c.GetInstanceID(), out error);
            }
            return true;
        }

        internal bool Validate(out string error)
        {
            error = _reason;
            if (!_finite) return false;
            if (!_active) return true;
            try
            {
                // Same predicates and native-read order as v0.4.41; only the first failed item is named.
                // Detailed object/ancestor reads are taken once on failure, never on the healthy hot path.
                if (_original == null) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_LOST:COLLIDER_MISSING", out error);
                if (!_original.enabled) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_LOST:COLLIDER_DISABLED", out error);
                if (!_original.gameObject.activeInHierarchy) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_LOST:INACTIVE_HIERARCHY", out error);
                if (_original.isTrigger) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_LOST:BECAME_TRIGGER", out error);
                if (_original.attachedRigidbody != null) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_LOST:ATTACHED_RIGIDBODY", out error);
                if (_original.GetInstanceID() != _originalId) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_LOST:COLLIDER_ID_CHANGED", out error);
                Bounds bounds = _original.bounds;
                if (!Near(_original.transform.position, _originalPosition)) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:POSITION", out error);
                if (!SameRotation(_original.transform.rotation, _originalRotation)) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:ROTATION", out error);
                if (!Near(_original.transform.lossyScale, _originalScale)) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:SCALE", out error);
                if (!Near(bounds.center, _boundsCenter)) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:BOUNDS_CENTER", out error);
                if (!Near(bounds.extents, _boundsExtents)) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:BOUNDS_EXTENTS", out error);
                if (_original.gameObject.layer != _layer) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:LAYER", out error);
                if (_original.sharedMaterial != _material) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:MATERIAL_REFERENCE", out error);
                if (!FootContactOffsetIsolation.Matches(_original.contactOffset, _offset)) return Fail("ORIGINAL_SUPPORT_ENVIRONMENT_CHANGED:CONTACT_OFFSET", out error);
                if (_arm == ArmB)
                {
                    if (_box == null || !_box.enabled || !_boxObject.activeInHierarchy || _box.isTrigger || _box.attachedRigidbody != null ||
                        _box.GetInstanceID() != _selectedId || !Near(_box.transform.position, _boxPosition) ||
                        !SameRotation(_box.transform.rotation, Quaternion.identity) || !Near(_box.transform.lossyScale, Vector3.one) ||
                        !Near(_box.center, Vector3.zero) || !Near(_box.size, _boxSize) || _boxObject.layer != _layer ||
                        _box.sharedMaterial != _material || !FootContactOffsetIsolation.Matches(_box.contactOffset, _offset))
                        return Fail("OWNED_BOX_CONTRACT_CHANGED", out error);
                    if (!AuditBoxIsolation(out error)) return false;
                }
                return true;
            }
            catch (Exception e) { return Fail("SUPPORT_VALIDATE_EXCEPTION:" + e.GetType().Name, out error); }
        }

        internal bool ValidateFootPair(Collider foot, bool trace, out string error)
        {
            error = _reason;
            if (!_finite) return false;
            if (!_active || foot == null || !foot.enabled) return true;
            try
            {
                if (foot != _left && foot != _right) return Fail("UNKNOWN_FOOT_OWNER", out error);
                if (SelectedCollider == null || UnityEngine.Physics.GetIgnoreCollision(foot, SelectedCollider))
                    return Fail("SELECTED_SUPPORT_FOOT_PAIR_IGNORED", out error);
                if (_arm == ArmB && !UnityEngine.Physics.GetIgnoreCollision(foot, _original))
                    return Fail("DUPLICATE_ORIGINAL_AND_BOX_SUPPORT", out error);
                if (trace) { if (foot == _left) _pairLeft++; else _pairRight++; }
                return true;
            }
            catch (Exception e) { return Fail("SUPPORT_PAIR_EXCEPTION:" + e.GetType().Name, out error); }
        }

        internal bool Raycast(Vector3 origin, Vector3 direction, float maxDistance, out RaycastHit selectedHit)
        {
            selectedHit = default;
            if (!_active || !_finite || _original == null) return false; // never fall back to an unpinned scene hit
            try
            {
                Ray ray = new Ray(origin, direction);
                if (!_original.Raycast(ray, out RaycastHit originalHit, maxDistance)) { _originalMisses++; return false; }
                if (!Horizontal(originalHit.normal)) { Fail("ORIGINAL_SUPPORT_NOT_HORIZONTAL", out _); return false; }
                if (_arm == ArmA) selectedHit = originalHit;
                else if (_box == null || !_box.Raycast(ray, out selectedHit, maxDistance) || !MatchingPlane(originalHit, selectedHit))
                { Fail("ORIGINAL_BOX_PROBE_PLANE_MISMATCH", out _); return false; }
                _probeMatches++;
                return true;
            }
            catch (Exception e) { Fail("PINNED_PROBE_EXCEPTION:" + e.GetType().Name, out _); return false; }
        }

        internal bool NotifyBoxCollision(string phase, Collision collision)
        {
            if (!_active || _arm != ArmB || phase == "EXIT") return true;
            try
            {
                Collider other = collision != null ? collision.collider : null;
                if (other == _left) { _boxLeftCallbacks++; return _finite; }
                if (other == _right) { _boxRightCallbacks++; return _finite; }
                _unexpectedCallbacks++;
                return Fail("UNEXPECTED_BOX_CONTACT:" + (other != null ? other.GetInstanceID() : 0), out _);
            }
            catch (Exception e) { return Fail("BOX_CALLBACK_EXCEPTION:" + e.GetType().Name, out _); }
        }

        internal void ResetExposure()
        {
            _pairLeft = _pairRight = _probeMatches = _originalMisses = _isolationRepairs = _unexpectedCallbacks = 0;
            _boxLeftCallbacks = _boxRightCallbacks = 0;
        }

        internal string Stop()
        {
            // Disable before deferred Destroy. No original collider/property needs restoration here;
            // Foot contact shadows separately restore the original Foot-to-scene ignore pairs.
            _active = false;
            // Attempt both independent ways to remove the owned surface from physics. A native
            // Collider setter exception must not skip GameObject deactivation or a later retry.
            int disableFailures = 0;
            try { if (_box != null) _box.enabled = false; }
            catch { disableFailures++; }
            try { if (_boxObject != null) _boxObject.SetActive(false); }
            catch { disableFailures++; }
            bool colliderDisabled = false, objectInactive = false;
            try { colliderDisabled = _box == null || !_box.enabled; }
            catch { disableFailures++; }
            try { objectInactive = _boxObject != null && !_boxObject.activeInHierarchy; }
            catch { disableFailures++; }
            try
            {
                _stoppedSafe = colliderDisabled || objectInactive;
                if (!_stoppedSafe) return "ARL support surface | stopped=False | reason=BOX_STILL_ACTIVE";
                if (_boxObject != null) Object.Destroy(_boxObject);
                _box = null; _boxObject = null;
                _others.Clear(); _registered.Clear();
                _original = _left = _right = null; _material = null;
                _arm = IdleArm;
                return "ARL support surface | stopped=True | ownedBoxCollisionOff=True | destroyDeferred=True | disableFailures=" + disableFailures;
            }
            catch (Exception e)
            {
                _stoppedSafe = false;
                return "ARL support surface | stopped=False | reason=" + e.GetType().Name + ":" + e.Message;
            }
        }

        internal string Summary() => "mode=" + Mode + "/arm=" + _arm + "/active=" + _active + "/finite=" + _finite +
            "/reason=" + (_reason ?? "<none>") + "/original=" + _originalId + ":" + (_originalPath ?? "<none>") +
            "/originalType=" + (_originalType ?? "<none>") + "/selected=" + _selectedId +
            "/height=" + F(_height) + "/boxThickness=" + F(ThicknessMeters) + "/boxSize=" + _boxSize +
            "/groundOffset=" + F(_offset) + "/sameLayer=" + _layer +
            "/pairSamples(left-right)=" + _pairLeft + "-" + _pairRight +
            "/matchingProbePoints=" + _probeMatches + "/originalMisses=" + _originalMisses +
            "/boxCallbacks(left-right)=" + _boxLeftCallbacks + "-" + _boxRightCallbacks +
            "/isolationRegistry=" + _others.Count + "/isolationRepairs=" + _isolationRepairs +
            "/unexpectedBoxCallbacks=" + _unexpectedCallbacks + "/exposure=" + ExposureReady + "/stoppedSafe=" + StoppedSafe +
            "/originalFailure={" + (_originalFailureEvidence ?? "<none>") + "}";

        private bool Fail(string reason, out string error)
        {
            _finite = false;
            if (_reason == null)
            {
                _reason = reason;
                if (reason.StartsWith("ORIGINAL_SUPPORT_", StringComparison.Ordinal))
                {
                    try
                    {
                        _originalFailureEvidence = CaptureOriginalFailureEvidence(reason);
                        Plugin.Logger?.LogWarning("[ARL][ORIGINAL_SUPPORT_FAILURE_V04410] " + _originalFailureEvidence);
                    }
                    catch { } // Diagnostics must never prevent the owned Box from being disabled below.
                }
            }
            error = _reason;
            // Unknown callback/scope faults stop the owned collision surface immediately;
            // the coordinator closes both Foot drives and finalizes the run at the next Fixed boundary.
            try { if (_box != null) _box.enabled = false; }
            catch { try { if (_boxObject != null) _boxObject.SetActive(false); } catch { } }
            return false;
        }
        private string CaptureOriginalFailureEvidence(string failedPredicate)
        {
            // Snapshot after the exact predicate failed, before Fail disables our Box. The predicate is
            // retained separately: later reads cannot retroactively replace it if the environment changes.
            var sb = new StringBuilder(1536);
            sb.Append("phase=FAILED_PREDICATE_BEFORE_BOX_DISABLE/predicate=").Append(failedPredicate)
                .Append("/arm=").Append(_arm).Append("/originalId=").Append(_originalId)
                .Append("/originalPath=").Append(_originalPath ?? "<none>").Append("/originalType=").Append(_originalType ?? "<none>")
                .Append("/expectedPosition=").Append(VectorText(_originalPosition)).Append("/expectedRotation=").Append(RotationText(_originalRotation))
                .Append("/expectedScale=").Append(VectorText(_originalScale)).Append("/expectedBoundsCenter=").Append(VectorText(_boundsCenter))
                .Append("/expectedBoundsExtents=").Append(VectorText(_boundsExtents)).Append("/expectedLayer=").Append(_layer)
                .Append("/expectedOffset=").Append(F(_offset));
            try
            {
                Collider c = _original;
                if (c == null) return sb.Append("/currentExists=False/ancestors=UNAVAILABLE").ToString();
                sb.Append("/currentExists=True");
                AppendFailureRead(sb, "currentId", () => c.GetInstanceID().ToString());
                AppendFailureRead(sb, "enabled", () => c.enabled.ToString());
                AppendFailureRead(sb, "activeSelf", () => c.gameObject.activeSelf.ToString());
                AppendFailureRead(sb, "activeInHierarchy", () => c.gameObject.activeInHierarchy.ToString());
                AppendFailureRead(sb, "isTrigger", () => c.isTrigger.ToString());
                AppendFailureRead(sb, "attachedRigidbody", () => { Rigidbody body = c.attachedRigidbody; return body == null ? "<none>" : body.GetInstanceID().ToString(); });
                AppendFailureRead(sb, "position", () => VectorText(c.transform.position));
                AppendFailureRead(sb, "rotation", () => RotationText(c.transform.rotation));
                AppendFailureRead(sb, "scale", () => VectorText(c.transform.lossyScale));
                AppendFailureRead(sb, "boundsCenter", () => VectorText(c.bounds.center));
                AppendFailureRead(sb, "boundsExtents", () => VectorText(c.bounds.extents));
                AppendFailureRead(sb, "layer", () => c.gameObject.layer.ToString());
                AppendFailureRead(sb, "materialMatches", () => (c.sharedMaterial == _material).ToString());
                AppendFailureRead(sb, "contactOffset", () => F(c.contactOffset));
                Transform t = c.transform;
                string firstInactive = "<none>";
                int count = 0;
                sb.Append("/ancestors(self-first)={");
                while (t != null && count < 16)
                {
                    GameObject go = t.gameObject;
                    string identity = go.name.Replace('\r', ' ').Replace('\n', ' ') + "#" + go.GetInstanceID();
                    bool selfActive = go.activeSelf;
                    if (!selfActive && firstInactive == "<none>") firstInactive = identity;
                    if (count > 0) sb.Append(" <- ");
                    sb.Append(identity).Append("[self=").Append(selfActive).Append(",hierarchy=").Append(go.activeInHierarchy).Append("]");
                    t = t.parent; count++;
                }
                sb.Append("}/ancestorTruncated=").Append(t != null).Append("/firstInactiveSelfOrAncestor=").Append(firstInactive);
            }
            catch (Exception e) { sb.Append("/snapshotIncomplete=").Append(e.GetType().Name); }
            return sb.ToString();
        }

        private static void AppendFailureRead(StringBuilder sb, string name, Func<string> read)
        {
            sb.Append('/').Append(name).Append('=');
            try { sb.Append(read()); }
            catch (Exception e) { sb.Append("READ_FAILED:").Append(e.GetType().Name); }
        }
        private static string VectorText(Vector3 v) => "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")";
        private static string RotationText(Quaternion q) => "(" + F(q.x) + "," + F(q.y) + "," + F(q.z) + "," + F(q.w) + ")";

        private static bool ProbeReady(FootAnkleFoundationReport r) => r != null && r.GroundGeometryReady &&
            r.GroundProbeHitCount == 5 && r.GroundFootprintSameSurface && !r.GroundColliderHasAttachedRigidbody;
        private static bool MatchingPlane(RaycastHit a, RaycastHit b) => FiniteFloat(a.distance) && FiniteFloat(b.distance) &&
            Mathf.Abs(a.distance - b.distance) <= PlaneToleranceMeters && Horizontal(a.normal) && Horizontal(b.normal);
        private static bool Horizontal(Vector3 n) => FiniteVector(n) && n.y > 0.9999f && Mathf.Abs(n.x) < 0.0001f && Mathf.Abs(n.z) < 0.0001f;
        private static bool Near(Vector3 a, Vector3 b) => FiniteVector(a) && FiniteVector(b) && (a - b).sqrMagnitude <= 0.00000001f;
        private static bool SameRotation(Quaternion a, Quaternion b) =>
            Mathf.Abs(a.x-b.x) + Mathf.Abs(a.y-b.y) + Mathf.Abs(a.z-b.z) + Mathf.Abs(a.w-b.w) <= 0.00001f;
        private static bool FiniteFloat(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool FiniteVector(Vector3 v) => FiniteFloat(v.x) && FiniteFloat(v.y) && FiniteFloat(v.z);
        private static string F(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
    }
}
