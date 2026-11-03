using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private readonly FootGroundReference _groundReference = new FootGroundReference();
        internal bool GroundReferenceEnabled => _groundReference.Enabled;
        internal bool GroundReferenceApplied => _groundReference.Apply;
        internal bool GroundReferencePartial => _groundReference.AllowPartial;
        internal bool LandingReferenceCaptureRequested => _traceEnabled && _stanceContactLifecycle.ContinuousEntry &&
            !_contactShadow.Active && _stanceContactLifecycle.EntryHistoryAvailable;

        private FootLandingReferenceEvidence.LandingPath CaptureLandingReference(PhysicsStepContext step,
            int fixedSequence, LegTurnStepEvidence leg, Quaternion lowerRotation)
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
                    _lowerBody.position + lowerRotation * ankle, lowerRotation, ankle);
            }
            catch { return default; } // Optional read failure cannot interrupt contact or control.
        }

        // One input per genuine shared Render packet, then the existing planner owns R/W/A.
        // Read current native Lower pose in this Render sample; no cached Fixed pose is labelled current.
        private void PrepareGroundReferenceInput(SourceStepContext source, Quaternion raw, Vector3 rawW, float dt,
            out Quaternion goal, out Vector3 goalW)
        {
            goal = raw; goalW = rawW;
            if (!_groundReference.Enabled || !_ankleDrive.DynamicActive || !_relativeReference.Active) return;
            if (_groundReference.TryHold(source, raw, rawW, out goal, out goalW)) return;
            FootGroundReference.GroundSample candidate = new FootGroundReference.GroundSample { Reason = "CONTROLLED_SUPPORT_UNAVAILABLE", Goal = raw };
            try
            {
                if (_supportSurface != null && _supportSurface.TryGetEntryBox(_supportSurface.SelectedId, out Vector3 floorCenter, out Vector3 floorHalf))
                {
                    Vector3 scale = _collider.transform.lossyScale;
                    Vector3 lowerScale = _lowerBody.transform.lossyScale;
                    if (IsFinite(scale) && scale.x > 0f && scale.y > 0f && scale.z > 0f &&
                        IsFinite(lowerScale) && lowerScale.x > 0f && lowerScale.y > 0f && lowerScale.z > 0f)
                    {
                        Quaternion lower = _lowerBody.rotation;
                        // Position and rotation from the same Rigidbody pose; do not mix in a render Transform pose.
                        Vector3 ankle = _lowerBody.position + lower * Vector3.Scale(_lowerAnkleAnchorLocal, lowerScale);
                        candidate = FootGroundReference.Evaluate(ankle, lower, raw, _bindSourceRelative,
                            Vector3.Scale(_collider.center - _footAnkleAnchorLocal, scale),
                            Vector3.Scale(_collider.size, scale) * 0.5f, floorCenter, floorHalf, _groundReference.AllowPartial);
                    }
                }
            }
            catch (Exception) { candidate.Reason = "NATIVE_GEOMETRY_UNAVAILABLE"; }
            // Unavailable geometry only removes the optional goal. The existing native support,
            // body and clock contracts still independently fail-close before physical writes.
            _groundReference.ObserveRender(_traceEnabled, source, raw, rawW, dt, candidate, out goal, out goalW);
        }
    }
}
