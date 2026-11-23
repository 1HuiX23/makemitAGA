using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private bool _formalBeforePulse, _pulseSourceReady;
        private int _pulseSourceContinuity;
        private int _pulsePriorFloor, _pulseBodyId, _pulseRootId;
        private float _pulseAge;
        private Vector3 _pulseAnkle;
        private Quaternion _pulseRotation;

        internal LandingPulseWindow.Foot PulseFoot(in PhysicsStepContext step, int local)
        {
            var life = _stanceContactLifecycle;
            if (!_pulseSourceReady || !_ankleDrive.DynamicActive || !life.ContinuousEntry || !life.Finite ||
                !_previousReferenceSupport.Step.Matches(step) || !_report.RuntimeContext.Matches(step) ||
                LastCompletedControlFixedSequence != local || _gameNativeGaitPhase.CurrentPhaseFixed != local || _gameNativeGaitPhase.State == FootGameNativeLocomotionPhaseObserver.StateUnknown ||
                _supportSurface == null || !_supportSurface.MainRoomBox) return default;
            int floor = _supportSurface.SelectedId;
            bool active = FormalStanceContactActive, pre = life.Precontact.Physical;
            if ((_formalBeforePulse && _pulsePriorFloor != floor) || ((active || pre) &&
                _contactShadow.AuthorizedColliderId != floor) || (_report.PhysicalGroundContactPossible != (active || pre))) return default;
            var foot = new LandingPulseWindow.Foot { Step=step, Local=local, Source=_report.RuntimeSourceContext,
                SourceContinuity=_pulseSourceContinuity, Root=_pulseRootId, Body=_pulseBodyId, Floor=floor, Age=_pulseAge,
                Was=_formalBeforePulse, Active=active, Precontact=pre, Miss=life.FirstMissThisStep, PendingMiss=life.MissInProgress,
                Ordinary=life.OrdinaryEnabledThisStep, Recovery=life.RecoveryEnabledThisStep,
                LiftRelease=_formalBeforePulse && !active && _gameNativeGaitPhase.State == FootGameNativeLocomotionPhaseObserver.StateToSwing &&
                    life.ReleasedToSwing };
            if (foot.Miss || foot.Ordinary || foot.Recovery)
            {
                var entry = life.DiagnosticEntryGeometry;
                foot.HasEntryGeometry = entry.Valid && !entry.DiagnosticSelectedBox && entry.SupportId == floor &&
                    float.IsFinite(entry.MinClearance) && float.IsFinite(entry.AnchorGap);
                if (foot.Miss && !foot.HasEntryGeometry) return default;
                foot.Sole=entry.MinClearance; foot.Gap=entry.AnchorGap;
            }
            return foot;
        }

        // Checked only inside an open release. Both loaded and zero-load precontact are excluded.
        internal bool PulseFootprintFree(in PhysicsStepContext step, int floor)
        {
            if (!IsBuilt || _targetRoot == null || _body.GetInstanceID() != _pulseBodyId || _targetRoot.GetInstanceID() != _pulseRootId ||
                !_previousReferenceSupport.Step.Matches(step) || floor == 0 || _supportSurface.SelectedId != floor ||
                _collider.enabled || _contactShadow.Active || ExactAuthorizedStanceSupportLoadEligible ||
                _body.useGravity || _lowerBody.useGravity || _body.isKinematic || _lowerBody.isKinematic ||
                _soleRequalification.CurrentGroundCollider != _supportSurface.SelectedCollider ||
                !_supportSurface.TryGetTransitionBox(out var center, out var half)) return false;
            var gait = _gameNativeGaitPhase.CurrentEvidence;
            var scale = _collider.transform.lossyScale;
            return gait.Hits == 5 && gait.Same && gait.Surface == floor && gait.Normal.sqrMagnitude > .9f &&
                IsFinite(scale) && scale.x > 0f && scale.y > 0f && scale.z > 0f &&
                FootBoxGeometry.Inside(_pulseAnkle,_pulseRotation,Vector3.Scale(_collider.center-_footAnkleAnchorLocal,scale),
                    Vector3.Scale(_collider.size,scale)*.5f,center,half);
        }
    }
}
