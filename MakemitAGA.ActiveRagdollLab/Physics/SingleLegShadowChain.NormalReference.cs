using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        internal bool HipNormalReferenceMatches(bool selected, bool sampleAgePrediction) => _dynamicParentActive &&
            _hipNormalReference.Selected == selected && _hipNormalReference.SampleAgePrediction == (selected && sampleAgePrediction);

        private bool TryPrepareHipNormalRecovery(bool sourceActive, out bool requested)
        {
            requested = false;
            if (!_dynamicParentActive || !_hipNormalReference.Selected || sourceActive) return true;
            var previous = _hipNormalReference.Current; // Authoritative owner, never the trace packet.
            if (previous.Applied && (!_runtimeStep.Valid || !previous.Valid || !previous.Selected || !previous.Step.Valid ||
                previous.Step.Epoch != _runtimeStep.Epoch || previous.Step.Sequence + 1 != _runtimeStep.Sequence)) return false;
            if (_torso.Report == null) return false;

            // Do not include the composite absorber's own active flag in this hold:
            // it must be allowed to converge and release after external protection clears.
            bool protection = _torso.Report.ControlRecoveryActive ||
                _hipAuthority.PhysicalRecoveryPending || _softKneeAuthority.PhysicalRecoveryPending ||
                (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization) ||
                _hipSourceReferenceShockAbsorber.Active || _hipReferenceGovernor.Active;
            requested = protection && (previous.Applied || _hipCompositeReferenceShockAbsorber.NormalRecoveryHandoffActive);
            if (!requested) return true;
            if (!_hipCompositeReferenceShockAbsorber.Enabled) return false;
            // Seed only at this transition, from the real planner owner. During an
            // ongoing recovery the world accumulator keeps its own state unchanged.
            return !previous.Applied || _hipCompositeReferenceShockAbsorber.RecordAppliedNormalGoal(
                _hipNormalReference.Rotation, _hipNormalReference.W);
        }

        private bool UpdateHipNormalReference(Quaternion parentR, Vector3 parentW)
        {
            bool ordinary = _dynamicParentActive && _torso.Report != null &&
                !_torso.Report.SourceTurnEventActive && !_torso.Report.ControlRecoveryActive &&
                !_hipAuthority.PhysicalRecoveryPending && !_softKneeAuthority.PhysicalRecoveryPending &&
                !(_hipControlFrame.Active && _profile.LegHipControlFrameInertialization) && !HipFeedforwardGoalBridgeActive;
            Quaternion inputR = parentR * _controlTargetHipRelative;
            Vector3 inputW = parentW + parentR * _controlTargetHipRelativeAngularVelocityLocal;
            if (!_hipNormalReference.Step(_traceEnabled, _runtimeStep, _runtimeSource, ordinary, inputR, inputW,
                parentR, _controlTargetHipRelativeAngularVelocityLocal, _report.TargetRenderSampleAgeSeconds,
                _profile.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds,
                _profile.LegHipMaxAngularAcceleration / _profile.LegHipAngularDamper,
                Mathf.Min(_profile.LegAngularAccelerationFeedforwardMaxAcceleration, _profile.LegHipMaxAngularAcceleration),
                _profile.LegHipAngularSpring / _profile.LegHipAngularDamper)) return false;
            if (!_hipNormalReference.Current.Applied) return true;
            _controlTargetHipRelative = Quaternion.Inverse(parentR) * _hipNormalReference.Rotation;
            _controlTargetHipRelativeAngularVelocityLocal = Quaternion.Inverse(parentR) * (_hipNormalReference.W - parentW);
            // Source and V04810 non-source recovery start the WORLD absorber from the
            // last consumed goal. Local/raw health histories are never rebased.
            return _hipCompositeReferenceShockAbsorber.RecordAppliedNormalGoal(_hipNormalReference.Rotation, _hipNormalReference.W);
        }
    }
}
