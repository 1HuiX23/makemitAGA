using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private readonly FootGroundReference _groundReference = new FootGroundReference();
        internal HipContactPreparationSample CaptureContactPreparation(PhysicsStepContext step, SourceStepContext source,
            int supportId, Vector3 floorCenter, Vector3 floorHalf, InitialGeometryPreflight geometry)
        {
            HipContactPreparationSample result = new HipContactPreparationSample { Geometry = geometry };
            if (!FreeSpaceNativeState(true) || _traceEnabled || !_report.RuntimeContext.Matches(step) ||
                !_report.RuntimeSourceContext.Matches(source)) return result;
            result.NativeOff = true;
            result.IdentityQualified = supportId != 0 && _report.ContactIdentityCurrentQualified &&
                _report.GroundColliderInstanceId == supportId && !_report.GroundColliderHasAttachedRigidbody;
            Vector3 lowerAnchor = _lowerBody.transform.TransformPoint(_lowerAnkleAnchorLocal);
            Vector3 footAnchor = _body.transform.TransformPoint(_footAnkleAnchorLocal);
            var context = FootStanceOnlyAuthorizedContactLifecycle.CaptureEntryContext(_report.FixedUpdateSequence,
                _body, lowerAnchor, _footAnkleAnchorLocal, _lowerBody.rotation * _sampledSourceRelative,
                _sampledFootPosition, _sampledFootRotation, Vector3.Distance(lowerAnchor, footAnchor), geometry.SourceAge);
            result.Motion = FootStanceOnlyAuthorizedContactLifecycle.CaptureEntryGeometryAt(
                _collider, floorCenter, floorHalf, context, step.Dt).Motion;
            return result;
        }
        internal bool GroundReferenceEnabled => _groundReference.Enabled;
        internal bool GroundReferenceApplied => _groundReference.Apply;
        internal bool GroundReferencePartial => _groundReference.AllowPartial;
        // Initial run qualification reads the same held source packet as the joined legs.
        // It never modifies calibration, body pose, collider dimensions or controller goals.
        internal InitialGeometryPreflight CaptureInitialGeometry(SourceStepContext source, float now,
            Vector3 floorCenter, Vector3 floorHalf)
        {
            if (!IsBuilt || _lowerBody == null || _body == null) return default;
            return CaptureInitialGeometryAt(source, now, floorCenter, floorHalf,
                _lowerBody.position, _lowerBody.rotation, _body.position, _body.rotation);
        }
        internal InitialGeometryPreflight ProjectInitialGeometry(SourceStepContext source, float now,
            Vector3 floorCenter, Vector3 floorHalf, Vector3 lower, Quaternion lowerRotation,
            out Vector3 ankle, out Quaternion footRotation, out Vector3 rawFoot, out Quaternion rawFootRotation)
        {
            // Mirror the existing physical-parent initializer, including its origin placement.
            // This is an expected initialized pose; the post-join native check remains mandatory.
            ankle = lower + lowerRotation * _lowerAnkleAnchorLocal;
            footRotation = lowerRotation * _sampledSourceRelative;
            rawFoot = _sampledFootPosition; rawFootRotation = _sampledFootRotation;
            return CaptureInitialGeometryAt(source, now, floorCenter, floorHalf, lower, lowerRotation, ankle, footRotation);
        }
        private InitialGeometryPreflight CaptureInitialGeometryAt(SourceStepContext source, float now,
            Vector3 floorCenter, Vector3 floorHalf, Vector3 lower, Quaternion lowerRotation,
            Vector3 actual, Quaternion actualRotation)
        {
            InitialGeometryPreflight result = default;
            if (!IsBuilt || !_report.RuntimeSourceContext.Matches(source) || _lowerBody == null) return result;
            Vector3 scale = _collider.transform.lossyScale, lowerScale = _lowerBody.transform.lossyScale;
            if (!IsFinite(scale) || !IsFinite(lowerScale) || scale.x <= 0f || scale.y <= 0f || scale.z <= 0f ||
                lowerScale.x <= 0f || lowerScale.y <= 0f || lowerScale.z <= 0f || !IsFinite(actual) || !IsFinite(lower) ||
                !IsFinite(_sampledFootPosition) || !ValidSourceRotation(actualRotation) || !ValidSourceRotation(lowerRotation) ||
                !ValidSourceRotation(_sampledFootRotation) || !ValidSourceRotation(_sampledSourceRelative)) return result;
            Vector3 center = Vector3.Scale(_collider.center - _footAnkleAnchorLocal, scale);
            Vector3 half = Vector3.Scale(_collider.size, scale) * .5f;
            if (!IsFinite(center) || !IsFinite(half) || half.x <= 0f || half.y <= 0f || half.z <= 0f) return result;
            Vector3 rawAnkle = _sampledFootPosition + _sampledFootRotation * Vector3.Scale(_footAnkleAnchorLocal, scale);
            Vector3 controlAnkle = lower + lowerRotation * Vector3.Scale(_lowerAnkleAnchorLocal, lowerScale);
            Vector3 actualAnkle = actual + actualRotation * Vector3.Scale(_footAnkleAnchorLocal, scale);
            Quaternion controlRotation = lowerRotation * _sampledSourceRelative;
            float top = floorCenter.y + floorHalf.y;
            float age = now - source.Time;
            if (!IsFinite(age)) return result;
            result.Valid = source.Valid;
            result.Calibrated = _report.SupportPlaneCalibrationApplied;
            // Match the existing leg/Foot age convention. FixedUpdate's unscaled
            // clock is phase-specific; a negative phase offset is not a new sample
            // or proof of a reversed source stream. Identity is checked separately.
            result.SourceAge = Mathf.Max(0f, age);
            result.RawMin = FootBoxGeometry.Minimum(rawAnkle, _sampledFootRotation, center, half) - top;
            result.ControlMin = FootBoxGeometry.Minimum(controlAnkle, controlRotation, center, half) - top;
            result.ActualMin = FootBoxGeometry.Minimum(actualAnkle, actualRotation, center, half) - top;
            result.FootprintsInside = FootBoxGeometry.Inside(rawAnkle, _sampledFootRotation, center, half, floorCenter, floorHalf) &&
                FootBoxGeometry.Inside(controlAnkle, controlRotation, center, half, floorCenter, floorHalf) &&
                FootBoxGeometry.Inside(actualAnkle, actualRotation, center, half, floorCenter, floorHalf);
            return result;
        }
        internal bool LandingReferenceCaptureRequested => _traceEnabled && _stanceContactLifecycle.ContinuousEntry &&
            !FormalStanceContactActive;

        internal bool FullLandingReferenceRequested(bool adopted) => LandingReferenceCaptureRequested &&
            (LandingReferenceCapturePolicy.NeedsFull(LandingObservationTiming.Run.Compact,
                _stanceContactLifecycle.EntryHistoryAvailable, adopted) ||
             (TransitionSide == 0 && LandingObservationTiming.Run.Transitions?.WantsLeftHeight == true));

        private FootLandingReferenceEvidence.LandingPath CaptureLandingReference(PhysicsStepContext step,
            int fixedSequence, LegTurnStepEvidence leg, Quaternion lowerRotation, Quaternion footRelative)
        {
            try
            {
                if (!leg.Matches(step, fixedSequence)) return default;
                Vector3 lowerScale = _lowerBody.transform.lossyScale, footScale = _body.transform.lossyScale;
                if (!IsFinite(lowerScale) || !IsFinite(footScale) ||
                    lowerScale.x <= 0f || lowerScale.y <= 0f || lowerScale.z <= 0f ||
                    footScale.x <= 0f || footScale.y <= 0f || footScale.z <= 0f) return default;
                Vector3 ankle = Vector3.Scale(_lowerAnkleAnchorLocal, lowerScale);
                return FootLandingReferenceEvidence.Build(step, _report.RuntimeSourceContext, fixedSequence, leg.LandingInput,
                    _sampledFootPosition + _sampledFootRotation * Vector3.Scale(_footAnkleAnchorLocal, footScale),
                    _lowerBody.position + lowerRotation * ankle, lowerRotation, ankle, leg.RawUpperR, footRelative);
            }
            catch { return default; } // Optional read failure cannot interrupt contact or control.
        }

    }
}
