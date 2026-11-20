using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private ParentHeightProbe.Trace _parentHeight;
        internal bool HasParentHeightTrial(PhysicsStepContext step, out ParentHeightProbe.Request trial)
        {
            trial = default;
            return _parentHeight != null && _parentHeight.TryTake(step, out trial);
        }

        internal void CompleteParentHeight(in ParentHeightProbe.Request trial, in ParentHeightProbe.Input input,
            in PelvisParentFrame leftParent, Vector3 leftLocalHip, FootAnkleProxyFoundation peer, in PelvisParentFrame peerParent, Vector3 peerLocalHip, HipAnchorTranslationPair leftPair, HipAnchorTranslationPair rightPair)
        {
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.ParentHeightProbe);
            ParentHeightProbe.Result result;
            try
            {
                var left = leftParent.Matches(trial.Step) ? ReadHeightFoot(trial, leftParent.SettledPoint(leftLocalHip), "LEFT") : default;
                var right = peer != null && peerParent.Matches(trial.Step) ?
                    peer.ReadHeightFoot(trial, peerParent.SettledPoint(peerLocalHip), "RIGHT") : default;
                result = ParentHeightProbe.Compare(trial, input, left, right, out var plan);
                if (result.Planned)
                {
                    try
                    {
                        if (leftPair.TryGetCommittedInput(trial.Step, out var l) && rightPair.TryGetCommittedInput(trial.Step, out var r))
                            result.Allocation = ParentHeightAllocation.Compare(input, plan, l, r, leftPair.Report.Current, rightPair.Report.Current);
                    }
                    catch { result.Allocation.State = ParentHeightAllocation.Status.Fault; }
                }
            }
            catch { result = new ParentHeightProbe.Result { Outcome = ParentHeightProbe.Status.Fault }; }
            _parentHeight.Complete(result);
        }

        private ParentHeightProbe.FootPose ReadHeightFoot(in ParentHeightProbe.Request trial, Vector3 hip, string side)
        {
            // Geometry is optional per side. A missing peer must not erase the receiver trial.
            try { return CaptureHeightFoot(trial, hip, side); }
            catch { return default; }
        }

        private ParentHeightProbe.FootPose CaptureHeightFoot(in ParentHeightProbe.Request trial, Vector3 hip, string side)
        {
            if (_report.Side != side || !IsBuilt || !_report.RuntimeContext.Matches(trial.Step) ||
                !_report.RuntimeSourceContext.Matches(trial.Source) || LastCompletedControlFixedSequence <= 0 ||
                LastCompletedControlFixedSequence != _report.FixedUpdateSequence ||
                (side == "LEFT" && _report.FixedUpdateSequence != trial.Local) || _supportSurface == null ||
                !_supportSurface.TryGetEntryBox(trial.SupportId, out var center, out var half) ||
                (center - trial.Center).sqrMagnitude > 1e-12f || (half - trial.Half).sqrMagnitude > 1e-12f) return default;
            var scale = _collider.transform.lossyScale;
            var pose = new ParentHeightProbe.FootPose
            {
                Hip = hip,
                Rotation = _body.rotation,
                LowerAnkle = _lowerBody.transform.TransformPoint(_lowerAnkleAnchorLocal),
                Ankle = _body.transform.TransformPoint(_footAnkleAnchorLocal),
                Center = Vector3.Scale(_collider.center - _footAnkleAnchorLocal, scale),
                Half = Vector3.Scale(_collider.size, scale) * .5f,
                Active = _contactShadow.Active
            };
            pose.Valid = _collider.enabled == pose.Active && PelvisParentFrame.Q(pose.Rotation) && IsFinite(pose.Hip) &&
                IsFinite(pose.Ankle) && IsFinite(pose.LowerAnkle) && IsFinite(pose.Center) && IsFinite(pose.Half) &&
                pose.Half.x > 0f && pose.Half.y > 0f && pose.Half.z > 0f;
            pose.Authorized = pose.Valid && pose.Active && ExactAuthorizedStanceSupportLoadEligible &&
                _report.ContactIdentityCurrentColliderInstanceId == trial.SupportId && _report.GroundColliderInstanceId == trial.SupportId;
            return pose;
        }
    }
}
