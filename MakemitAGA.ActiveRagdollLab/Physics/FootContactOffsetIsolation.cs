using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04390 changes only the ARL-owned Foot collider contactOffset. A restores its once-captured
    /// creation value; B selects 20 mm. The source Ground collider and Physics defaults are never written.
    /// This is a contact-generation margin, not sole thickness, a support force or a safety threshold.
    /// Native four-boundary exposure is counted by the existing V04290 observer, not by this selector.
    /// </summary>
    internal sealed class FootContactOffsetIsolation
    {
        internal const string Mode = "FOOT_CONTACT_OFFSET_CAUSAL_AB_V04390";
        internal const string IdleArm = "IDLE_V04390_CAPTURED_FOOT_CONTACT_OFFSET";
        internal const string ArmA = "A_V04390_CAPTURED_FOOT_CONTACT_OFFSET";
        internal const string ArmB = "B_V04390_FOOT_CONTACT_OFFSET_20MM";
        internal const float BOffsetMeters = 0.020f;
        internal const float ReadbackToleranceMeters = 0.000001f;

        private bool _initialized, _finite = true, _readbackValid;
        private IntPtr _ownerPointer;
        private int _ownerId, _ownerGameObjectId, _readbackFailures;
        private float _baseline, _selected, _actual;
        private string _arm = IdleArm, _invalidReason;
        private bool _isolatedWriteAttempted, _isolationBroken;
        private float _disabledWriteActual = float.NaN, _enabledWriteActual = float.NaN, _restoredDisabledActual = float.NaN;
        private string _writePath = "CAPTURED_NO_WRITE", _isolationState = "NOT_USED";
        private string _bodyOwnership = "NOT_CHECKED", _attachmentBefore = "NOT_READ", _attachmentEnabled = "NOT_READ";

        internal static bool IsExperimentArm(string arm) =>
            string.Equals(arm, ArmA, StringComparison.Ordinal) || string.Equals(arm, ArmB, StringComparison.Ordinal);
        internal static bool Matches(float actual, float expected) => Positive(actual) && Positive(expected) &&
            Mathf.Abs(actual - expected) <= ReadbackToleranceMeters;
        private static bool Positive(float value) => !float.IsNaN(value) && !float.IsInfinity(value) && value > 0f;

        internal bool Initialize(Collider footCollider, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (_initialized) return ValidateReadback(footCollider, report, out error);
            try
            {
                if (footCollider == null) return Fail("FOOT_CONTACT_OFFSET_COLLIDER_MISSING", report, out error);
                float baseline = footCollider.contactOffset;
                if (!Positive(baseline)) return Fail("FOOT_CONTACT_OFFSET_BASELINE_NON_POSITIVE_OR_NON_FINITE", report, out error);
                _ownerPointer = footCollider.Pointer;
                _ownerId = footCollider.GetInstanceID();
                GameObject ownerObject = footCollider.gameObject;
                _ownerGameObjectId = ownerObject != null ? ownerObject.GetInstanceID() : 0;
                if (_ownerPointer == IntPtr.Zero || _ownerId == 0 || _ownerGameObjectId == 0)
                    return Fail("FOOT_CONTACT_OFFSET_OWNER_INVALID", report, out error);
                _baseline = _selected = _actual = baseline;
                _disabledWriteActual = _restoredDisabledActual = baseline;
                _initialized = _readbackValid = true;
                _finite = true; _invalidReason = null;
                CopyToReport(report);
                return true;
            }
            catch (Exception e) { return Fail("FOOT_CONTACT_OFFSET_INIT_EXCEPTION:" + e.GetType().Name, report, out error); }
        }

        internal bool SetArm(string arm, Collider footCollider, Rigidbody footBody, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, IdleArm, StringComparison.Ordinal) && !IsExperimentArm(arm))
                return Fail("UNKNOWN_FOOT_CONTACT_OFFSET_ARM:" + (arm ?? "<null>"), report, out error);
            try
            {
                if (!OwnerMatches(footCollider)) return Fail("FOOT_CONTACT_OFFSET_OWNER_MISMATCH_OR_NOT_INITIALIZED", report, out error);
                // An incomplete collision-state restoration must not be healed by a later matching offset.
                // Bilateral setup will fall back to full foundation rollback in this case.
                if (_isolationBroken) return Fail("FOOT_CONTACT_OFFSET_COLLISION_ISOLATION_NOT_RESTORED", report, out error);
                if (footCollider.enabled) return Fail("FOOT_CONTACT_OFFSET_CHANGE_REQUIRES_QUIESCED_COLLIDER", report, out error);
                // Do not silently label an unchanged/decreased value as the widened B intervention.
                if (string.Equals(arm, ArmB, StringComparison.Ordinal) && _baseline >= BOffsetMeters - ReadbackToleranceMeters)
                    return Fail("FOOT_CONTACT_OFFSET_B_NOT_WIDER_THAN_CAPTURED_BASELINE", report, out error);
                float selected = string.Equals(arm, ArmB, StringComparison.Ordinal) ? BOffsetMeters : _baseline;
                _isolatedWriteAttempted = false;
                _disabledWriteActual = _enabledWriteActual = _restoredDisabledActual = float.NaN;
                _writePath = "DIRECT_DISABLED"; _isolationState = "NOT_USED";
                _bodyOwnership = "NOT_CHECKED"; _attachmentBefore = _attachmentEnabled = "NOT_READ";
                footCollider.contactOffset = selected;
                _arm = arm; _selected = selected;
                _finite = true; _invalidReason = null;
                _disabledWriteActual = _actual = footCollider.contactOffset;
                _restoredDisabledActual = _disabledWriteActual;
                if (!Positive(_actual))
                    return Fail("FOOT_CONTACT_OFFSET_READBACK_NON_POSITIVE_OR_NON_FINITE", report, out error);
                // This setup-only retry is bounded to one isolated native-shape write. Never yield,
                // simulate, move the body, or allow an enabled Foot collider into a physics solve here.
                // A matching disabled write follows the original path without touching collision state.
                if (!Matches(_actual, _selected))
                {
                    _writePath = "ISOLATED_PRECHECK";
                    _restoredDisabledActual = float.NaN;
                    if (!TryIsolatedShapeWrite(footCollider, footBody, out string isolatedError))
                        return Fail("FOOT_CONTACT_OFFSET_ISOLATED_WRITE_FAILED:" + isolatedError +
                            "/disabled-enabled-restoredMm=" + S(_disabledWriteActual * 1000f) + "-" + S(_enabledWriteActual * 1000f) + "-" + S(_restoredDisabledActual * 1000f) +
                            "/isolation=" + _isolationState + "/bodyOwnership=" + _bodyOwnership +
                            "/attachmentBefore=" + _attachmentBefore + "/attachmentEnabled=" + _attachmentEnabled, report, out error);
                }
                bool valid = ValidateReadback(footCollider, report, out error);
                CopyToReport(report);
                return valid;
            }
            catch (Exception e) { return Fail("FOOT_CONTACT_OFFSET_SET_EXCEPTION:" + e.GetType().Name, report, out error); }
        }

        private bool TryIsolatedShapeWrite(Collider footCollider, Rigidbody footBody, out string error)
        {
            error = null;
            if (footBody == null || footBody.Pointer == IntPtr.Zero)
            {
                error = "FOOT_BODY_MISSING";
                return false;
            }
            // Foot construction (also in V04380) places both components on the same owned GameObject.
            // R2 incorrectly used a disabled collider's physics attachment / managed wrapper pointer
            // as the creation-ownership prerequisite. Check the captured Unity object identity instead.
            GameObject colliderObject = footCollider.gameObject;
            GameObject bodyObject = footBody.gameObject;
            int colliderObjectId = colliderObject != null ? colliderObject.GetInstanceID() : 0;
            int bodyObjectId = bodyObject != null ? bodyObject.GetInstanceID() : 0;
            int bodyId = footBody.GetInstanceID();
            _bodyOwnership = "captured-collider-bodyObjectId=" + _ownerGameObjectId + "-" + colliderObjectId + "-" + bodyObjectId +
                ",bodyId=" + bodyId;
            if (bodyId == 0 || colliderObjectId != _ownerGameObjectId || bodyObjectId != _ownerGameObjectId)
            {
                error = "FOOT_BODY_GAMEOBJECT_OWNER_MISMATCH";
                return false;
            }
            Rigidbody attachedBefore = footCollider.attachedRigidbody;
            _attachmentBefore = attachedBefore == null ? "NULL" : "id=" + attachedBefore.GetInstanceID() +
                ",pointerMatches=" + (attachedBefore.Pointer == footBody.Pointer);
            bool capturedDetection = false, originalDetection = false, colliderOff = false, detectionRestored = false;
            string failure = null;
            try
            {
                originalDetection = footBody.detectCollisions;
                capturedDetection = true;
                footBody.detectCollisions = false;
                if (footBody.detectCollisions)
                    throw new InvalidOperationException("FOOT_COLLISION_ISOLATION_READBACK_FAILED");
                footCollider.enabled = true;
                if (!footCollider.enabled)
                    throw new InvalidOperationException("FOOT_NATIVE_SHAPE_ENABLE_READBACK_FAILED");
                Rigidbody attachedEnabled = footCollider.attachedRigidbody;
                _attachmentEnabled = attachedEnabled == null ? "NULL" : "id=" + attachedEnabled.GetInstanceID() +
                    ",pointerMatches=" + (attachedEnabled.Pointer == footBody.Pointer);
                if (attachedEnabled != null && attachedEnabled.GetInstanceID() != bodyId)
                    throw new InvalidOperationException("FOOT_ENABLED_ATTACHMENT_BODY_MISMATCH");
                // Count the actual setter attempt, not entry into the preliminary owner checks.
                _isolatedWriteAttempted = true;
                _writePath = "ISOLATED_ENABLED_ONCE";
                footCollider.contactOffset = _selected;
                _enabledWriteActual = footCollider.contactOffset;
                if (!Matches(_enabledWriteActual, _selected)) failure = "ENABLED_CONTACT_OFFSET_READBACK_MISMATCH";
            }
            catch (Exception e) { failure = e.GetType().Name + ":" + e.Message; }
            finally
            {
                // Always attempt to close the temporary collider, even after a setter/getter throws.
                try { footCollider.enabled = false; }
                catch (Exception e) { failure = failure ?? "COLLIDER_DISABLE_EXCEPTION:" + e.GetType().Name; }
                try { colliderOff = !footCollider.enabled; }
                catch (Exception e) { failure = failure ?? "COLLIDER_DISABLE_READBACK_EXCEPTION:" + e.GetType().Name; }
                if (capturedDetection && colliderOff)
                {
                    try
                    {
                        footBody.detectCollisions = originalDetection;
                        detectionRestored = footBody.detectCollisions == originalDetection;
                    }
                    catch (Exception e) { failure = failure ?? "COLLISION_DETECTION_RESTORE_EXCEPTION:" + e.GetType().Name; }
                }
                else if (!capturedDetection)
                    detectionRestored = true; // No detection write was attempted before capture succeeded.
                // If the collider could not be closed, keep body detection OFF and require destruction.
                _isolationBroken = !colliderOff || !detectionRestored;
                _isolationState = "colliderOff=" + colliderOff + ",detectionCaptured=" + capturedDetection +
                    ",detectionOriginal=" + originalDetection + ",detectionRestored=" + detectionRestored;
                if (colliderOff)
                {
                    try
                    {
                        _restoredDisabledActual = _actual = footCollider.contactOffset;
                        if (!Matches(_restoredDisabledActual, _selected))
                            failure = failure ?? "RESTORED_OFFSET_READBACK_MISMATCH";
                    }
                    catch (Exception e) { failure = failure ?? "RESTORED_OFFSET_READBACK_EXCEPTION:" + e.GetType().Name; }
                }
            }
            error = failure ?? (_isolationBroken ? "COLLISION_STATE_RESTORE_READBACK_FAILED" : null);
            return error == null;
        }

        internal bool ValidateReadback(Collider footCollider, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            try
            {
                if (!OwnerMatches(footCollider)) return Fail("FOOT_CONTACT_OFFSET_OWNER_MISMATCH_OR_NOT_INITIALIZED", report, out error);
                _actual = footCollider.contactOffset;
                _readbackValid = Matches(_actual, _selected);
                if (!_readbackValid)
                    // Freeze the first failing native value in the reason before any later read or rollback.
                    // Formatting uses only cached scalars and never retries the native getter.
                    return Fail("FOOT_CONTACT_OFFSET_READBACK_MISMATCH/baseline-selected-actualMm=" +
                        S(_baseline * 1000f) + "-" + S(_selected * 1000f) + "-" + S(_actual * 1000f) +
                        "/differenceMm=" + S((_actual - _selected) * 1000f) +
                        "/writePath=" + _writePath + "/disabled-enabled-restoredMm=" + S(_disabledWriteActual * 1000f) +
                        "-" + S(_enabledWriteActual * 1000f) + "-" + S(_restoredDisabledActual * 1000f), report, out error);
                CopyToReport(report, false);
                if (!_finite) error = _invalidReason;
                return _finite;
            }
            catch (Exception e) { return Fail("FOOT_CONTACT_OFFSET_READBACK_EXCEPTION:" + e.GetType().Name, report, out error); }
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _readbackFailures = 0;
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report, bool format = true)
        {
            if (report == null) return;
            // Cached formatting cannot retry a failing native getter on an error-reporting path.
            report.FootContactOffsetMode = Mode;
            report.FootContactOffsetArm = _arm;
            report.FootContactOffsetInitialized = _initialized;
            report.FootContactOffsetFinite = _finite;
            report.FootContactOffsetReadbackValid = _readbackValid;
            report.FootContactOffsetOwnerId = _ownerId;
            report.FootContactOffsetBaselineMeters = _baseline;
            report.FootContactOffsetSelectedMeters = _selected;
            report.FootContactOffsetActualMeters = _actual;
            report.FootContactOffsetReadbackFailures = _readbackFailures;
            report.FootContactOffsetBaselineRestored = _initialized && _finite && _readbackValid &&
                string.Equals(_arm, IdleArm, StringComparison.Ordinal) && Matches(_actual, _baseline);
            report.FootContactOffsetInvalidReason = _invalidReason;
            if (!format) return;
            report.FootContactOffsetSummary = "mode=" + Mode + "/arm=" + _arm + "/initialized=" + _initialized +
                "/finite=" + _finite + "/ownerId=" + _ownerId +
                "/baseline-selected-actualMm=" + S(_baseline * 1000f) + "-" + S(_selected * 1000f) + "-" + S(_actual * 1000f) +
                "/readbackValid=" + _readbackValid + "/readbackFailures=" + _readbackFailures +
                "/baselineRestored=" + report.FootContactOffsetBaselineRestored +
                "/writePath=" + _writePath + "/isolatedWriteAttempted=" + _isolatedWriteAttempted +
                "/disabled-enabled-restoredMm=" + S(_disabledWriteActual * 1000f) + "-" + S(_enabledWriteActual * 1000f) + "-" + S(_restoredDisabledActual * 1000f) +
                "/isolationBroken=" + _isolationBroken + "/isolation=" + _isolationState +
                "/bodyOwnership=" + _bodyOwnership + "/attachmentBefore=" + _attachmentBefore +
                "/attachmentEnabled=" + _attachmentEnabled + "/reason=" + (_invalidReason ?? "<none>");
        }

        private bool OwnerMatches(Collider collider) => _initialized && collider != null &&
            collider.Pointer == _ownerPointer && collider.GetInstanceID() == _ownerId;
        private bool Fail(string reason, FootAnkleFoundationReport report, out string error)
        {
            _finite = _readbackValid = false;
            _readbackFailures++;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            error = _invalidReason;
            CopyToReport(report);
            return false;
        }
        private static string S(float value) => value.ToString("G9", CultureInfo.InvariantCulture);
    }
}
