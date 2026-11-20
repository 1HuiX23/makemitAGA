using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;
using Forecast = MakemitAGA.ActiveRagdollLab.Physics.RightFootTorqueForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private RightFootForecastWindow _rightFootForecast;

        private void ObserveRightFootForecast(in Sample s, in LegTurnStepEvidence leg,
            in FootStanceOnlyAuthorizedContactLifecycle.EntryGeometry entry, Vector3 lowerAnkle, Vector3 footAnkle, Quaternion footRotation)
        {
            if (s.RightThigh != RightThighWindow.Signal.Onset && _rightFootForecast?.Active != true) return;
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.RightFootForecast);
            try
            {
                _rightFootForecast ??= new RightFootForecastWindow();
                var p = new RightKneePivot.Frame {
                    Dt = s.Step.Dt, Box = s.Min, FootR = footRotation, LowerAnkle = lowerAnkle, FootAnkle = footAnkle,
                    UpperHip = leg.PhysicalUpperHip, UpperKnee = leg.PhysicalUpperKnee, LowerKnee = leg.PhysicalLowerKnee,
                    Center = Vector3.Scale(entry.LocalBoxCenter - _footAnkleAnchorLocal, entry.BoxScale),
                    Half = Vector3.Scale(entry.LocalBoxSize, entry.BoxScale) * .5f,
                    FloorCenter = s.FloorCenter, FloorHalf = s.FloorHalf
                };
                _rightFootForecast.Observe(s, p, _body.GetInstanceID(), _body.angularVelocity);
            }
            catch { _rightFootForecast?.Cut(RightFootForecastWindow.Interruption.Fault); }
        }

        // Invoked after this foot, both hips and the receiver completed their normal FOR_N writes.
        // The three direct AddTorque owners clear their current vector before every command, including inactive steps.
        internal void CompleteRightFootForecast(PhysicsStepContext step, in LegTurnStepEvidence leg,
            Rigidbody upper, bool kneeTransport, HipAnchorTranslationPair right)
        {
            if (_rightFootForecast == null || !_rightFootForecast.Requested(step)) return;
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.RightFootForecast);
            Forecast.Result result;
            try { result = ForecastFromCommand(step, leg, upper, kneeTransport, right); }
            catch { result = new Forecast.Result { State = Forecast.Status.Fault }; }
            _rightFootForecast.Complete(result);
        }
        private Forecast.Result ForecastFromCommand(PhysicsStepContext step, in LegTurnStepEvidence leg,
            Rigidbody upper, bool kneeTransport, HipAnchorTranslationPair right)
        {
            var req = _rightFootForecast.Current;
            var p = req.Pose;
            if (_report.Side != "RIGHT" || !_report.RuntimeContext.Matches(step) || !_report.RuntimeSourceContext.Matches(req.Source) ||
                LastCompletedControlFixedSequence != req.Local || _report.FixedUpdateSequence != req.Local ||
                right == null || !right.TryGetCommittedInput(step, out var input)) return default;
            var c = right.Report.Current;
            if (!leg.Matches(step, req.Local) || !leg.Source.Matches(req.Source) || !leg.KneeHeightsValid || !leg.UpperAnchorsValid ||
                !kneeTransport || !c.JointSolved || input.Seed.Fixed != req.Local || c.Fixed != req.Local ||
                !RightThighTracking.Same(leg.HipFinalA, c.AllocatedHipAlpha) ||
                !RightThighTracking.Same(leg.KneeTotalA, leg.KneeRelativeA + leg.HipFinalA))
                return new Forecast.Result { State = Forecast.Status.MissingLeg };
            if (upper == null || _lowerBody == null || _body == null || _body.isKinematic || _body.GetInstanceID() != req.Body ||
                upper.GetInstanceID() != input.Seed.ChildId || c.ChildId != input.Seed.ChildId ||
                !RightThighTracking.Same(upper.angularVelocity, input.ChildW) ||
                !RightThighTracking.Same(_body.angularVelocity, req.FootW))
                return new Forecast.Result { State = Forecast.Status.BodyMismatch };
            if (!PelvisParentFrame.V(p.LowerKnee) || !PelvisParentFrame.V(p.UpperKnee) ||
                !RightThighTracking.Same(input.Seed.ChildPoint, p.UpperHip) || !RightKneePivot.Near(leg.KneeHeights.z, p.LowerKnee.y))
                return new Forecast.Result { State = Forecast.Status.Geometry };
            var m = new RightThighTracking.Motion {
                UpperMaxW = upper.maxAngularVelocity, LowerMaxW = _lowerBody.maxAngularVelocity,
                LowerW = _lowerBody.angularVelocity, KneeRelativeA = leg.KneeRelativeA,
                KneeGapV = _lowerBody.GetPointVelocity(p.LowerKnee) - upper.GetPointVelocity(p.UpperKnee),
                AnkleGapV = _body.GetPointVelocity(p.FootAnkle) - _lowerBody.GetPointVelocity(p.LowerAnkle)
            };
            if (!PelvisParentFrame.F(m.UpperMaxW) || !PelvisParentFrame.F(m.LowerMaxW) || m.UpperMaxW <= 0f || m.LowerMaxW <= 0f ||
                !PelvisParentFrame.V(m.LowerW) || !PelvisParentFrame.V(m.KneeGapV) || !PelvisParentFrame.V(m.AnkleGapV) ||
                !PelvisParentFrame.V(input.ChildComV) || !PelvisParentFrame.F(c.ChildMass) || c.ChildMass <= 0f)
                return new Forecast.Result { State = Forecast.Status.InvalidMotion };
            RightThighTracking.ProjectAnchors(p, m, input, c, out _, out var lower, out var foot, out _);
            var motion = new Forecast.BodyMotion { W = req.FootW, Mass = _body.mass, MaxW = _body.maxAngularVelocity,
                Drag = _body.angularDrag, Inertia = _body.inertiaTensor, WorldPrincipal = p.FootR * _body.inertiaTensorRotation,
                Transport = _ankleAngularTransport.CurrentTorqueWorld, Drive = _ankleDrive.CurrentTorqueWorld,
                Support = _stanceSupportAngular.CurrentTorqueWorld };
            return Forecast.Predict(p, lower, foot, motion);
        }
    }
}
