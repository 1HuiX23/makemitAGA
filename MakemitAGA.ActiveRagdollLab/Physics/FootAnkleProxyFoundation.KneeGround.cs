using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private KneeFootGroundPacket _kneeFootGroundPacket;
        internal void ClearKneeFootGroundPacket() { _kneeFootGroundPacket = default; }

        private bool TryPrepareKneeFootGroundPacket(SourceStepContext source, KneeFootGroundInput input, Quaternion rawFoot,
            Vector3 footScale, Vector3 lowerScale, Vector3 floorCenter, Vector3 floorHalf)
        {
            if (!input.Matches(source) || input.PreferKneeProgress != _groundReference.KneeProgressSelected ||
                !_report.RuntimeContext.Matches(input.CompletedStep) ||
                !_previousReferenceSupport.EligibleForSource(source, input.CompletedStep, _supportSurface.SelectedId) ||
                input.LowerId != _lowerBody.GetInstanceID()) return false;
            var geometry = new KneeGroundTarget.Geometry
            {
                Hip = input.Hip, Upper = input.Upper, UpperSpan = input.UpperSpan,
                LowerSpan = Vector3.Scale(_lowerAnkleAnchorLocal - input.LowerKneeAnchor, lowerScale),
                FootRelative = rawFoot, FootBindRelative = _bindSourceRelative,
                Center = Vector3.Scale(_collider.center - _footAnkleAnchorLocal, footScale),
                Half = Vector3.Scale(_collider.size, footScale) * .5f,
                FloorCenter = floorCenter, FloorHalf = floorHalf
            };
            KneeGroundTarget.Sample sample = KneeGroundTarget.Evaluate(geometry, input.RawKnee, input.Axis, input.AllowExtension);
            // A readable chain can still be outside this support Box. It must not
            // replace an independently valid native-Lower Foot goal with defaults.
            if (!sample.Valid || !sample.Foot.Valid) return false;
            sample.Source = source; sample.GeometryStep = input.CompletedStep;
            // Optional scalar evidence; do not turn an unsupported bound into a
            // control failure or recompute it for held Fixed consumers.
            try { sample.Feasibility = KneeGroundFeasibility.AtRender(geometry, input.RawKnee, input.Axis); }
            catch (System.Exception) { sample.Feasibility = default; }
            _kneeFootGroundPacket = new KneeFootGroundPacket
                { Input = input, Geometry = geometry, Sample = sample, SupportId = _supportSurface.SelectedId,
                    PreparationSupport = _previousReferenceSupport };
            return sample.Valid;
        }

        private void PublishKneeFootGroundPacket(SourceStepContext source)
        {
            // A failed/held-mismatch Foot publication cannot authorize half of a pair.
            if (_kneeFootGroundPacket.Published || !_kneeFootGroundPacket.Input.Matches(source) ||
                !_kneeFootGroundPacket.Sample.Valid || !_groundReference.Finite || !_relativeReference.Finite ||
                !_relativeReference.Current.Source.Matches(source) || !_groundReference.Current.Source.Matches(source)) return;
            if (_kneeFootGroundPacket.Input.Apply && (!_groundReference.Apply || !_groundReference.AllowPartial ||
                !_groundReference.Current.JointChain)) return;
            KneeGroundTarget.Sample sample = _kneeFootGroundPacket.Sample;
            sample.PlannedFoot = _relativeReference.Current.Rotation;
            sample.FootTrajectory = _relativeReference.Current.GroundTrajectory;
            sample.KneeProgress = _groundReference.CurrentKneeProgress;
            sample.Applied = _kneeFootGroundPacket.Input.Apply && sample.Candidate;
            _kneeFootGroundPacket.Sample = sample;
            _kneeFootGroundPacket.Published = true;
        }

        internal bool TryReadKneeFootGroundPacket(PhysicsStepContext step, SourceStepContext source,
            int parentId, int upperId, int lowerId, out KneeFootGroundPacket packet, out string reason)
        {
            packet = default; reason = "NO_CURRENT_PUBLISHED_JOINT_INPUT";
            if (!_ankleDrive.DynamicActive || !_groundReference.Enabled || !_groundReference.Finite ||
                !_relativeReference.Active || !_relativeReference.Finite || !_relativeReference.Current.Source.Matches(source) ||
                _supportSurface == null || _kneeFootGroundPacket.Input.PreferKneeProgress != _groundReference.KneeProgressSelected ||
                !_kneeFootGroundPacket.Matches(step, source, parentId, upperId, lowerId, _supportSurface.SelectedId))
                return false;
            packet = _kneeFootGroundPacket; reason = packet.Sample.Reason;
            return true;
        }
    }
}
