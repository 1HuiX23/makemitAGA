using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        // This late read is still PRE_N. Both legs' commands have completed; PhysX has
        // not advanced. Use the same geometry/motion receipts as the verified 164/165 forecast.
        internal LeftFootReleaseForecast.Frame HandoffFrame(in Trace.Tick t, in LegTurnStepEvidence leg)
        {
            if (!_report.RuntimeContext.Matches(t.Step) || !_report.RuntimeSourceContext.Matches(t.Source.Clock) ||
                LastCompletedControlFixedSequence!=t.Local || _body==null || _lowerBody==null ||
                _body.GetInstanceID()!=t.Body || _supportSurface?.SelectedId!=t.Floor) return default;
            return CaptureReleaseFrame(t.Step,t.Local,t.Lower,t.Foot,_lowerBody.GetPointVelocity(t.Lower),_body.rotation,
                leg,default,compareKnee:false);
        }

        internal bool ReadHandoffMotion(in Trace.Tick t, in LeftFootReleaseForecast.Frame frame, out RightFootLandingForecast.Motion m)
        {
            m=default;
            if (!UnloadedReleaseFrame(frame)) return false;
            m=UnloadedReleaseMotion(t.Step,t.Local,frame);
            return true;
        }

        private int TransitionSide => _report.Side == "LEFT" ? 0 : _report.Side == "RIGHT" ? 1 : -1;

        private void CaptureTransitionSource(BilateralFootTransitionTrace trace, SourceStepContext source, Vector3 foot, Quaternion sourceRotation)
        {
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.FootTransitionSource);
            try
            {
                int side = TransitionSide;
                if (side < 0 || _targetRoot == null) { trace.Fault(); return; }
                var visual = default(FootTransitionSource.RenderPose);
                try
                {
                    var pose = _footObject.transform;
                    var scale = pose.lossyScale;
                    if (scale.x > 0f && scale.y > 0f && scale.z > 0f)
                    {
                        var bottom = _report.ColliderCenterLocal-Vector3.up*(_report.ColliderSizeLocal.y*.5f);
                        visual = FootTransitionSource.RenderPose.Capture(foot,sourceRotation,pose.position,pose.rotation,
                            Vector3.Scale(_footAnkleAnchorLocal,scale),Vector3.Scale(bottom,scale));
                    }
                }
                catch { } // Missing visible pose leaves the independent Render derivative usable.
                trace.Source(side).Observe(source, _targetRoot.GetInstanceID(), _targetRoot.position, _targetRoot.rotation, foot, visual);
            }
            catch { trace.Fault(); }
        }

        private void CaptureTransitionStep(BilateralFootTransitionTrace trace, in PhysicsStepContext step, int local,
            bool fresh, int held, float age, bool was, int priorFloor, Vector3 foot, Vector3 lower, Vector3 velocity,
            Quaternion rotation, in LegTurnStepEvidence leg, in FootStanceOnlyAuthorizedContactLifecycle.EntryContext entry,
            Quaternion releaseControl)
        {
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.FootTransitionPair);
            try
            {
                int side = TransitionSide;
                if (side < 0) { trace.Fault(); return; }
                var t = new BilateralFootTransitionTrace.Tick { Step = step, Local = local };
                var source = trace.Source(side).Current;
                if (!source.Clock.Matches(_report.RuntimeSourceContext) || !_report.RuntimeContext.Matches(step) ||
                    (source.World-_sampledFootLinearVelocityWorld).sqrMagnitude > 1e-8f ||
                    LastCompletedControlFixedSequence != local || _gameNativeGaitPhase.CurrentPhaseFixed != local ||
                    _supportSurface == null || !_supportSurface.TryGetTransitionBox(out Vector3 floorCenter,out Vector3 floorHalf))
                { trace.Submit(side,t); return; }

                var lifecycle = _stanceContactLifecycle;
                var gait = _gameNativeGaitPhase.CurrentEvidence;
                var scale = _collider.transform.lossyScale;
                var center = Vector3.Scale(_report.ColliderCenterLocal-_footAnkleAnchorLocal,scale);
                var half = Vector3.Scale(_report.ColliderSizeLocal,scale)*.5f;
                if (!IsFinite(scale) || half.x<=0f || half.y<=0f || half.z<=0f || !RightFootTorqueForecast.ValidRotation(rotation))
                { trace.Submit(side,t); return; }
                var baseLocal = center-Vector3.up*half.y;
                var arm = FootTransitionSource.LowestLever(rotation,center,half);
                var target = _sampledFootPosition + _sampledFootRotation * Vector3.Scale(_footAnkleAnchorLocal,scale);
                bool active = FormalStanceContactActive;
                int eventFloor = active || lifecycle.Precontact.Physical ? _contactShadow.AuthorizedColliderId : 0;
                if (lifecycle.FirstMissThisStep && !active)
                {
                    var g = lifecycle.DiagnosticEntryGeometry;
                    eventFloor = g.Valid && !g.DiagnosticSelectedBox ? g.SupportId : 0;
                }
                bool legId = leg.Step.Matches(step) && leg.Fixed == local && leg.Source.Matches(source.Clock);
                bool hip = legId && leg.TransitionHipValid;
                // This receipt is cleared by the lifecycle before every decision. Only
                // exact surface references from this solve can accompany the actual shoe.
                var geometry = lifecycle.DiagnosticEntryGeometry;
                bool references = !was && geometry.Valid && geometry.ReferencesValid &&
                    !geometry.DiagnosticSelectedBox && geometry.SupportId == _supportSurface.SelectedId &&
                    float.IsFinite(geometry.SourceMinClearance) && float.IsFinite(geometry.ControlMinClearance);
                t = new BilateralFootTransitionTrace.Tick {
                    Step=step, Local=local, Source=source, Held=held, Fresh=fresh, Age=age, Body=_body.GetInstanceID(),
                    Floor=_supportSurface.SelectedId, PriorFloor=priorFloor, EventFloor=eventFloor, Was=was, Active=active,
                    Entry=lifecycle.EntrySample, NativeDepthLatch=lifecycle.FirstMissDepthFixed,
                    Precontact=lifecycle.Precontact.Physical, Ordinary=lifecycle.OrdinaryEnabledThisStep, Miss=lifecycle.FirstMissThisStep,
                    Recovery=lifecycle.RecoveryEnabledThisStep, PendingMiss=lifecycle.MissInProgress,
                    Gait=gait, Phase=LandingPhase(_gameNativeGaitPhase.State), Motion=legId ? leg.SourcePhase : null,
                    ReleaseReason=was && !active ? _report.StanceContactLifecycleLastReason : null,
                    Foot=foot, Lower=lower, Target=target, Velocity=velocity,
                    Sole=foot+rotation*baseLocal, SourceSole=target+_sampledFootRotation*baseLocal,
                    CornerVelocity=velocity+Vector3.Cross(_body.angularVelocity,arm),
                    Inside=FootBoxGeometry.Inside(foot,rotation,center,half,floorCenter,floorHalf),
                    BoxMin=FootBoxGeometry.Minimum(foot,rotation,center,half)-(floorCenter.y+floorHalf.y),
                    EntryReferenceValid=references, SourceMin=geometry.SourceMinClearance, ControlMin=geometry.ControlMinClearance,
                    HipValid=hip, HipHeight=hip ? leg.TransitionHipHeight : 0f
                };
                if (side == 1)
                {
                    // For a qualified entry reuse the eight-corner native receipt. Weak STANCE and
                    // the release itself can lack that receipt. Project the same held source and selected
                    // control onto the current selected plane, marking the provenance separately from identity.
                    if (references)
                    {
                        t.RightEntryMin = geometry.MinClearance;
                        t.RightReference = BilateralFootTransitionTrace.Reference.Entry;
                    }
                    else if (was || (entry.FixedSequence == local && entry.ReferencesValid))
                    {
                        // Keep just the same-solve inputs on active and weak-stance steps. Only an
                        // observed release window projects them, including the immediately preceding step.
                        t.RightPose = new Trace.SelectedPose { Source = _sampledFootRotation,
                            Control = was ? releaseControl : entry.ControlRotation, Center = center, Half = half,
                            PlaneTop = floorCenter.y + floorHalf.y };
                        t.RightReference = Trace.Reference.SelectedPlane;
                    }
                    t.QualifiedIdentity = _report.ContactIdentityCurrentQualified &&
                        _report.ContactIdentityCurrentColliderInstanceId == t.Floor;
                }
                if (side == 0 && trace.WantsLeftHeight && legId && leg.Matches(step, local))
                    t.Height = LandingHeightBreakdown.Capture(t, leg.LandingInput, geometry.LandingPath, leg.ReceiverNormalResponse,
                        geometry.LandingGoal, _sampledFootRotation,
                        center, half, floorCenter.y + floorHalf.y);
                trace.Submit(side,t);
            }
            catch { trace.Fault(); }
        }
    }
}
