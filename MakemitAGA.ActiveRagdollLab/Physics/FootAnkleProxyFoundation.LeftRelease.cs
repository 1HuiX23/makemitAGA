using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private LeftFootReleaseTrace _leftRelease;
        internal LeftFootReleaseTrace LeftRelease => _leftRelease;

        // PRE_N geometry and velocities are taken before releasing the formal
        // collider. Only the command receipt is read after all Foot owners finish.
        private Forecast.Frame CaptureLeftReleaseFrame(in PhysicsStepContext step, int local, Vector3 lower, Vector3 foot,
            Vector3 lowerVelocity, Quaternion rotation, in LegTurnStepEvidence leg, SwingKneeCandidate.Controller kneeController)
        {
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LeftFootRelease);
            try
            {
                if (_supportSurface == null || !_supportSurface.MainRoomBox || _body == null || _lowerBody == null || _collider == null ||
                    !_supportSurface.TryGetEntryBox(_supportSurface.SelectedId, out var center, out var half)) return default;
                var scale = _collider.transform.lossyScale;
                if (!IsFinite(scale) || scale.x <= 0f || scale.y <= 0f || scale.z <= 0f) return default;
                var box = new RightKneePivot.Frame { Dt = step.Dt, LowerAnkle = lower, FootAnkle = foot, FootR = rotation,
                    Center = Vector3.Scale(_collider.center - _footAnkleAnchorLocal, scale),
                    Half = Vector3.Scale(_collider.size, scale) * .5f, FloorCenter = center, FloorHalf = half };
                box.Box = RightFootTorqueForecast.Clearance(box, foot, rotation);
                var goal = _lowerBody.rotation * ControlRelativeRotation;
                bool legMatches = leg.Matches(step, local) && leg.Source.Matches(_report.RuntimeSourceContext);
                var ground = _soleRequalification.CurrentGroundCollider;
                bool five = ground != null && ground == _supportSurface.SelectedCollider &&
                    _report.GroundProbeHitCount == 5 && _report.GroundFootprintSameSurface && !_report.GroundColliderHasAttachedRigidbody &&
                    _report.GroundColliderInstanceId == _supportSurface.SelectedId && _report.GroundFootprintMinNormalDotReferenceUp >= .90f &&
                    _report.GroundFootprintMinNormalDotReferenceUp <= 1.0001f;
                var frame = new Forecast.Frame { FootId = _body.GetInstanceID(), LowerId = _lowerBody.GetInstanceID(),
                    Pose = new RightFootLandingForecast.Pose { Box = box, Com = _body.worldCenterOfMass, W = _body.angularVelocity,
                        Velocity = _body.velocity, Goal = goal, PresentGoal = RightFootTorqueForecast.Clearance(box, foot, goal), Gap = (foot - lower).magnitude },
                    LowerVelocity = lowerVelocity, FivePoint = five,
                    UpperId = legMatches ? leg.CommittedUpperId : 0, Anchors = legMatches ? leg.MeasuredAnchors : default,
                    SourceAnkleY = (_sampledFootPosition + _sampledFootRotation * Vector3.Scale(_footAnkleAnchorLocal, scale)).y,
                    HipY = legMatches && leg.MeasuredAnchors.Valid ? leg.PhysicalUpperHip.y : float.NaN,
                    KneeY = legMatches && leg.MeasuredAnchors.Valid ? leg.PhysicalLowerKnee.y : float.NaN };
                if (_gameNativeGaitPhase.CurrentPhaseFixed == local && Forecast.CanCompareKnee(frame, LandingPhase(_gameNativeGaitPhase.State)))
                {
                    frame.KneeExamined = true;
                    try { frame.Knee = CompareReleaseKnee(step, local, frame, leg, kneeController); }
                    catch { frame.Knee.Reason = SwingKneeCandidate.Status.Fault; }
                }
                return frame;
            }
            catch { return default; }
        }

        private SwingKneeCandidate.Result CompareReleaseKnee(in PhysicsStepContext step, int local,
            in Forecast.Frame frame, in LegTurnStepEvidence leg, SwingKneeCandidate.Controller controller)
        {
            var source = _report.RuntimeSourceContext;
            var k = controller.Read();
            if (!leg.Matches(step, local) || !leg.Source.Matches(source) || !leg.MeasuredAnchors.Valid ||
                !k.Matches(step, source) || !_relativeReference.Active || !_relativeReference.Finite ||
                !_relativeReference.Current.Source.Matches(source))
                return new SwingKneeCandidate.Result { Reason = SwingKneeCandidate.Status.Unavailable };
            var pose = new SwingKneeCandidate.Pose
            {
                Reference = new KneeGroundTarget.Geometry { Upper = leg.HipTargetR, FootRelative = _sampledSourceRelative, FootBindRelative = _bindSourceRelative },
                PhysicalUpper = leg.UpperR, PhysicalLower = _lowerBody.rotation, SelectedFoot = ControlRelativeRotation,
                HipW = leg.HipTargetW, Axis = controller.Axis, AllowExtension = controller.AllowExtension
            };
            return Forecast.CompareKnee(frame.Pose.Box, leg.MeasuredAnchors, pose, k, step, source);
        }

        private void ObserveLeftRelease(in PhysicsStepContext step, int local, float age, bool wasActive, int priorSupport,
            in Forecast.Frame frame, in LegTurnStepEvidence leg)
        {
            _leftRelease ??= new LeftFootReleaseTrace();
            bool active = FormalStanceContactActive;
            bool released = wasActive && !active;
            if (!released && !_leftRelease.Tracking) return;
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LeftFootRelease);
            try
            {
                var lifecycle = _stanceContactLifecycle;
                int eventSupport = active ? _contactShadow.AuthorizedColliderId : 0;
                if (lifecycle.FirstMissThisStep && !active)
                {
                    var g = lifecycle.DiagnosticEntryGeometry;
                    if (g.Valid && !g.DiagnosticSelectedBox) eventSupport = g.SupportId;
                }
                var t = new LeftFootReleaseTrace.Tick { Step = step, Source = _report.RuntimeSourceContext, Local = local, Age = age,
                    Support = _supportSurface?.SelectedId ?? 0, Released = released, ReleasedSupport = priorSupport, Active = active,
                    Gait = _gameNativeGaitPhase.CurrentPhaseFixed == local ? LandingPhase(_gameNativeGaitPhase.State) : FootLandingOpportunityAudit.Phase.Unknown,
                    ReleaseReason = released ? _report.StanceContactLifecycleLastReason : null,
                    Motion = frame.FootId != 0 && leg.Matches(step, local) && leg.Source.Matches(_report.RuntimeSourceContext) ? leg.SourcePhase : null,
                    Miss = lifecycle.FirstMissThisStep, Recovery = lifecycle.RecoveryEnabledThisStep, Ordinary = lifecycle.OrdinaryEnabledThisStep,
                    EventSupport = eventSupport, MissInProgress = lifecycle.MissInProgress };
                if (active && _leftRelease.WatchingLeadFormal)
                {
                    t.FormalFoot = _body.GetInstanceID();
                    t.FormalLower = _lowerBody.GetInstanceID();
                    t.FormalUpper = leg.Matches(step, local) && leg.Source.Matches(t.Source) ? leg.CommittedUpperId : 0;
                }
                bool unloaded = frame.Valid && Forecast.UnloadedFoot(active || _contactShadow.Active, _collider.enabled, _body.useGravity,
                    _stanceFootNativeGravity.CurrentApplied, _report.StanceFootNativeGravityCurrentScheduledFixed, ExactAuthorizedStanceSupportLoadEligible);
                RightFootLandingForecast.Motion motion = default;
                if (unloaded && _report.RuntimeContext.Matches(step) &&
                    LastCompletedControlFixedSequence == local && _report.AnkleConstraintCurrentDynamic && !_ankleConstraint.CharacterJointArm &&
                    _ankleDrive.DynamicActive && !_body.isKinematic && !_lowerBody.isKinematic &&
                    frame.FootId == _body.GetInstanceID() && frame.LowerId == _lowerBody.GetInstanceID())
                    motion = ReadIssuedFootMotion(frame.Pose);
                _leftRelease.Observe(t, frame, unloaded, motion);
            }
            catch { _leftRelease.Fault(); }
        }
    }
}
