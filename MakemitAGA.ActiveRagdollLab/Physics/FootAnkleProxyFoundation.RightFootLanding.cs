using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.RightFootLandingForecast;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private RightFootLandingWindow _rightFootLanding;
        private void ObserveRightFootLanding(ref Sample s, in FootStanceOnlyAuthorizedContactLifecycle.EntryGeometry g,
            in FootStanceOnlyAuthorizedContactLifecycle.EntryContext entry, Vector3 lower, Vector3 foot, Quaternion rotation)
        {
            s.RightLandingObserved=true;
            if (_rightFootLanding?.Pending != true && !RightFootLandingWindow.BeforeContact(s)) return;
            using var timing=LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.RightFootLanding);
            try
            {
                _rightFootLanding ??= new RightFootLandingWindow();
                bool native = RightFootLandingWindow.Physical(s) && entry.FixedSequence==s.Fixed && entry.MotionValid &&
                    _body!=null && _lowerBody!=null;
                Forecast.Pose p=default;
                int footId=0, lowerId=0;
                if (native)
                {
                    footId=_body.GetInstanceID(); lowerId=_lowerBody.GetInstanceID();
                    p=new Forecast.Pose { Com=entry.Center, Velocity=entry.Velocity, W=entry.AngularVelocity, Goal=entry.ControlRotation,
                        PresentGoal=s.ControlAtActualAnkle, Gap=s.AnchorGap, Box=new RightKneePivot.Frame {
                            Dt=s.Step.Dt, Box=s.Min, FootR=rotation, LowerAnkle=lower, FootAnkle=foot,
                            Center=Vector3.Scale(g.LocalBoxCenter-_footAnkleAnchorLocal,g.BoxScale),
                            Half=Vector3.Scale(g.LocalBoxSize,g.BoxScale)*.5f, FloorCenter=s.FloorCenter, FloorHalf=s.FloorHalf } };
                }
                s.RightLandingRead=_rightFootLanding.Observe(s,p,footId,lowerId,native);
            }
            catch { _rightFootLanding?.Cut(RightFootLandingWindow.End.Fault); }
        }

        // The owning foot has finished all its force, torque and gravity selectors for
        // this step. It does not need either hip or a preceding thigh warning to predict.
        private void CompleteRightFootLanding(in PhysicsStepContext step)
        {
            var window=_rightFootLanding;
            if (window==null || !window.Requested(step)) return;
            using var timing=LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.RightFootLanding);
            Forecast.Motion motion=default;
            Forecast.Pair pair=default;
            Forecast.Result result;
            try { result=ReadRightFootCommand(window,step,out motion,out pair); }
            catch { result=new Forecast.Result { State=Forecast.Status.Fault }; }
            window.Complete(result,motion,pair);
        }
        private Forecast.Motion ReadIssuedFootMotion(in Forecast.Pose pose)
        {
            var angular = new RightFootTorqueForecast.BodyMotion
            {
                W = pose.W, Mass = _body.mass, Drag = _body.angularDrag, MaxW = _body.maxAngularVelocity,
                Inertia = _body.inertiaTensor, WorldPrincipal = pose.Box.FootR * _body.inertiaTensorRotation,
                Transport = _ankleAngularTransport.CurrentTorqueWorld, Drive = _ankleDrive.CurrentTorqueWorld,
                Support = _stanceSupportAngular.CurrentTorqueWorld
            };
            return new Forecast.Motion { Angular = angular, LinearDrag = _body.drag, Tether = _ankleConstraint.CurrentSequenceTether,
                AppliedLinearA = _report.AnkleConstraintCurrentAppliedAccelerationWorld };
        }

        private Forecast.Result ReadRightFootCommand(RightFootLandingWindow window, in PhysicsStepContext step, out Forecast.Motion motion, out Forecast.Pair pair)
        {
            motion=default; pair=default;
            if (_report.Side!="RIGHT" || !_report.RuntimeContext.Matches(step) || !_report.RuntimeSourceContext.Matches(window.Source) ||
                LastCompletedControlFixedSequence!=window.Local || _report.FixedUpdateSequence!=window.Local ||
                !_report.AnkleConstraintCurrentDynamic || _ankleConstraint.CharacterJointArm || !_ankleDrive.DynamicActive) return default;
            if (_body==null || _lowerBody==null || _body.isKinematic || _lowerBody.isKinematic ||
                !window.Bodies(_body.GetInstanceID(),_lowerBody.GetInstanceID())) return new Forecast.Result { State=Forecast.Status.BodyMismatch };
            if (_body.useGravity || _stanceFootNativeGravity.CurrentApplied || _contactShadow.Active || ExactAuthorizedStanceSupportLoadEligible)
                return new Forecast.Result { State=Forecast.Status.Loaded };
            var p=window.Pose;
            motion = ReadIssuedFootMotion(p);
            var result=Forecast.Predict(p,motion);
            bool needsPair=result.State==Forecast.Status.MissingDrive ||
                (result.State==Forecast.Status.OverGap && window.CouldUseSequencePair(p));
            if (needsPair && !_ankleDrive.CurrentStaleTargetW)
            {
                var lowerRotation=_lowerBody.rotation;
                pair=new Forecast.Pair { LowerMass=_lowerBody.mass, LowerInertia=_lowerBody.inertiaTensor,
                    LowerPrincipal=lowerRotation*_lowerBody.inertiaTensorRotation, LowerLever=(p.Box.LowerAnkle-_lowerBody.worldCenterOfMass).magnitude,
                    RealizedRelativeA=lowerRotation*_ankleDrive.CurrentAppliedRelativeAccelerationLocal };
                Forecast.Compare(p,motion,pair,ref result);
            }
            return result;
        }
    }
}
