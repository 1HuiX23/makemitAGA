using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        // Production entry used by real source sampling and the managed contract tests.
        // Both publication and input differentiation finish before a Knee packet is readable.
        private void ObserveFootReference(SourceStepContext source, Quaternion raw, Vector3 rawW, float dt,
            KneeFootGroundInput input)
        {
            PrepareGroundReferenceInput(source, raw, rawW, dt, input, out Quaternion goal, out Vector3 goalW, out FootGroundTrajectory.Input ground);
            _relativeReference.ObserveRender(_traceEnabled, source, goal, goalW, dt, ground);
            _groundReference.ValidateKneeProgressPublication(source, _relativeReference.Current);
            PublishKneeFootGroundPacket(source);
        }

        // One input per genuine shared Render packet, then the existing planner owns R/W/A.
        // Read current native Lower pose in this Render sample; no cached Fixed pose is labelled current.
        private void PrepareGroundReferenceInput(SourceStepContext source, Quaternion raw, Vector3 rawW, float dt,
            KneeFootGroundInput kneeFootInput, out Quaternion goal, out Vector3 goalW, out FootGroundTrajectory.Input trajectory)
        {
            goal = raw; goalW = rawW; trajectory = default;
            if (!_groundReference.Enabled || !_ankleDrive.DynamicActive || !_relativeReference.Active)
            { ClearKneeFootGroundPacket(); return; }
            if (_groundReference.TryHold(source, raw, rawW, out goal, out goalW)) return;
            ClearKneeFootGroundPacket();
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
                        if (TryPrepareKneeFootGroundPacket(source, kneeFootInput, raw, scale, lowerScale, floorCenter, floorHalf) &&
                            kneeFootInput.Apply)
                        {
                            candidate = _kneeFootGroundPacket.Sample.Foot;
                            candidate.JointChain = true;
                            var packet = _kneeFootGroundPacket;
                            trajectory = new FootGroundTrajectory.Input { Source = source, GeometryStep = packet.Input.CompletedStep,
                                ParentId = packet.Input.ParentId, UpperId = packet.Input.UpperId, LowerId = packet.Input.LowerId,
                                SupportId = packet.SupportId, Geometry = packet.Geometry, KneeGoal = packet.Sample.Goal,
                                NativeAnkle = ankle, NativeLower = lower };
                        }
                    }
                }
            }
            catch (Exception)
            {
                trajectory = default; ClearKneeFootGroundPacket();
                // A native read can fail AFTER Evaluate populated a valid candidate.
                // Discard that partial transaction, including Valid/Feasible/Goal;
                // changing only Reason would still let ResolveInput select its goal.
                candidate = new FootGroundReference.GroundSample { Reason = "NATIVE_GEOMETRY_UNAVAILABLE", Goal = raw };
            }
            // Unavailable geometry only removes the optional goal. The existing native support,
            // body and clock contracts still independently fail-close before physical writes.
            _groundReference.ChooseKneeProgress(_traceEnabled, source, raw, rawW, dt,
                _kneeFootGroundPacket, _relativeReference, ref candidate, ref trajectory);
            _groundReference.ObserveRender(_traceEnabled, source, raw, rawW, dt, candidate, out goal, out goalW);
            if (!_groundReference.Finite || !_groundReference.Current.Source.Matches(source) || !_groundReference.Current.JointChain)
                trajectory = default;
        }
    }
}
