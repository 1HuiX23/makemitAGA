using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        private readonly KneeRelativeReference _kneeReference = new KneeRelativeReference();
        private bool _kneePlannedFeedforwardSelected;
        internal bool KneeReferenceSelected => _kneeReference.Selected;
        internal bool KneePlannedFeedforwardSelected => _kneePlannedFeedforwardSelected;
        // Selection is seeded during continuous join, before contact scope is
        // committed. Execution below separately requires the final Hip scope.
        private bool KneeReferenceArmSelected => _dynamicParentActive && _torso != null &&
            (_torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmA ||
             _torso.PelvisReceiverControl.Arm == PelvisDynamicReceiverIsolation.DynamicHipArmB);
        private bool KneePlannedFeedforwardArmSelected => KneeReferenceArmSelected;

        private bool UpdateKneeReference()
        {
            if (_kneeReference.Selected != KneeReferenceArmSelected ||
                _kneePlannedFeedforwardSelected != KneePlannedFeedforwardArmSelected ||
                (_kneeReference.Selected && !PelvisRunScopes.HasHipCoupling(_dynamicScope))) return false;
            if (!_kneeReference.Selected) return true;
            if (!_kneeReference.Step(_traceEnabled, _runtimeStep, _runtimeSource,
                _sampledTargetKneeRelative, _sampledTargetKneeRelativeAngularVelocityLocal,
                _targetSampleTiming.AgeSeconds, _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds,
                _profile.LegKneeMaxAngularAcceleration / _profile.LegKneeAngularDamper,
                Mathf.Min(_profile.LegAngularAccelerationFeedforwardMaxAcceleration, _profile.LegKneeMaxAngularAcceleration),
                _profile.LegKneeAngularSpring / _profile.LegKneeAngularDamper)) return false;
            if (!_kneeReference.Current.Matches(_runtimeStep, _runtimeSource)) return false;
            _controlTargetKneeRelative = _kneeReference.Current.R;
            _controlTargetKneeRelativeAngularVelocityLocal = _kneeReference.Current.W;
            return true;
        }

        private Vector3 KneeTargetRelativeA => _kneeReference.Selected
            ? _kneeReference.Current.A : _sampledTargetKneeRelativeAngularAccelerationLocal;

        // Check each lane, not aggregate ActiveThisStep: both Hip arms may select
        // bounded Knee A while raw Hip A is still blocked by source/torso recovery.
        private bool LegFeedforwardGatesSatisfied()
        {
            if (!_legAngularAccelerationFeedforward.ActiveThisStep) return true;
            if (_torso == null || _torso.Report == null ||
                _hipAuthority.PhysicalRecoveryPending || _softKneeAuthority.PhysicalRecoveryPending ||
                (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization)) return false;
            bool sourceRecovery = _torso.ControlPhase.SourceTurnEventActive || _torso.ControlRecoveryActive;
            if (sourceRecovery && _legAngularAccelerationFeedforward.HipActiveThisStep) return false;
            if (_legAngularAccelerationFeedforward.KneeActiveThisStep && sourceRecovery &&
                (!_kneePlannedFeedforwardSelected || !_kneeReference.Current.Matches(_runtimeStep, _runtimeSource) ||
                 _kneeReference.Current.Stale)) return false;
            return true;
        }
    }
}
