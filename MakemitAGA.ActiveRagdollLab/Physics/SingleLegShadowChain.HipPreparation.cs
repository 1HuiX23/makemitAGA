using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        private LandingPreparationCommand _landingPreparationCommand;
        private void CaptureLandingPreparationCommand(KneeFootGroundPacket packet)
        {
            var h = _hipNormalReference.LandingPreparation; var k = _kneeReference.Current.LandingPreparation;
            if ((!h.Applied && !k.Applied) || !packet.Published) return;
            var g = packet.Geometry;
            _landingPreparationCommand = new LandingPreparationCommand { Step = _runtimeStep, Source = _runtimeSource,
                HipApplied = h.Applied, KneeApplied = k.Applied, Approach = packet.PreparationSupport.StrongLandingApproach,
                HipGain = h.Applied ? h.Best.Risk - h.Baseline.Risk : 0f, KneeGain = k.Applied ? k.Best.Risk - k.Baseline.Risk : 0f,
                SupportId = packet.SupportId, Center = g.Center, Half = g.Half, FloorCenter = g.FloorCenter, FloorHalf = g.FloorHalf,
                GeometryPrepared = true, RenderPhysicalHip = g.Hip, UpperSpan = g.UpperSpan, LowerSpan = g.LowerSpan,
                HipBaseline = h.Applied ? h.BaselineR : _hipNormalReference.Rotation,
                KneeBaseline = k.Applied ? k.BaselineR : _kneeReference.Current.R,
                HipSelected = _hipNormalReference.Rotation, KneeSelected = _kneeReference.Current.R,
                PublishedFoot = packet.Sample.PlannedFoot };
        }
        private PhysicsStepContext _hipPublicationStep;
        private Quaternion _hipPublicationParentR;
        private Vector3 _hipPublicationParentW;
        private bool PublishHipNormalReference(Quaternion parentR, Vector3 parentW)
        {
            _controlTargetHipRelative = Quaternion.Inverse(parentR) * _hipNormalReference.Rotation;
            _controlTargetHipRelativeAngularVelocityLocal = Quaternion.Inverse(parentR) * (_hipNormalReference.W - parentW);
            // Source and V04810 non-source recovery start the WORLD absorber from the
            // last consumed goal. Local/raw health histories are never rebased.
            return _hipCompositeReferenceShockAbsorber.RecordAppliedNormalGoal(_hipNormalReference.Rotation, _hipNormalReference.W);
        }
        private bool FinalizeHipLandingPreparation(KneeFootGroundPacket packet, KneeReferenceState knee)
        {
            bool selected = _hipLandingPreparationSelected && PelvisRunScopes.HasHipCoupling(_dynamicScope);
            if (!_hipNormalReference.PrepareLanding(selected, _traceEnabled, packet, knee)) return false;
            if (!_hipNormalReference.LandingPreparation.Applied) return true;
            // Never fetch a later parent pose or publish in a different parent basis.
            return _hipPublicationStep.Matches(_runtimeStep) &&
                PublishHipNormalReference(_hipPublicationParentR, _hipPublicationParentW);
        }
    }
}
