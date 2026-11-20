using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private RightAnkleWindow _rightAnkle;

        private RightKneePivot.Frame CaptureRightKneePivot(in Sample s, in LegTurnStepEvidence leg,
            in FootStanceOnlyAuthorizedContactLifecycle.EntryGeometry entry, Vector3 lowerAnkle, Vector3 footAnkle, Quaternion footRotation)
        {
            if (!RightAnkleWindow.InSwingContext(s)) return new RightKneePivot.Frame { State = RightKneePivot.Capture.OutsideSwing };
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.RightKneePivotSnapshot);
            try
            {
                return SnapshotPivot(s, leg, lowerAnkle, footAnkle, footRotation,
                    Vector3.Scale(entry.LocalBoxCenter - _footAnkleAnchorLocal, entry.BoxScale),
                    Vector3.Scale(entry.LocalBoxSize, entry.BoxScale) * .5f);
            }
            catch { return new RightKneePivot.Frame { State = RightKneePivot.Capture.Fault }; }
        }

        private static RightKneePivot.Frame SnapshotPivot(in Sample s, in LegTurnStepEvidence leg, Vector3 lowerAnkle, Vector3 footAnkle,
            Quaternion footRotation, Vector3 center, Vector3 half)
        {
            var p = new RightKneePivot.Frame { State = RightKneePivot.Capture.MissingAnchors };
            if (!leg.Matches(s.Step, s.Fixed) || !leg.Source.Matches(s.Source) || !leg.UpperAnchorsValid ||
                !leg.KneeHeightsValid || !s.KneeHeightsValid || !s.AnkleHeightsValid || !s.ParentReady ||
                leg.LandingInput.Fixed != s.Fixed || !leg.LandingInput.Step.Matches(s.Step) ||
                !leg.LandingInput.Source.Matches(s.Source) || !leg.LandingInput.Parent.Matches(s.Step, s.Source)) return p;
            if (!RightKneePivot.Near(leg.LandingInput.Parent.ReferencePoint.y, s.ReferenceHipY) ||
                !RightKneePivot.Near(leg.LandingInput.Parent.ActualPoint.y, s.ActualHipY) ||
                !RightKneePivot.Near(leg.KneeHeights.y, s.KneeHeights.y) || !RightKneePivot.Near(leg.PhysicalLowerKnee.y, s.KneeHeights.z) ||
                !RightKneePivot.Near(lowerAnkle.y, s.AnkleHeights.y) || !RightKneePivot.Near(footAnkle.y, s.AnkleHeights.z))
                return new RightKneePivot.Frame { State = RightKneePivot.Capture.GeometryMismatch };
            return new RightKneePivot.Frame
            {
                State = RightKneePivot.Capture.Ready, Dt = s.Step.Dt, ReferenceHipY = s.ReferenceHipY, ParentHipY = s.ActualHipY,
                GoalKneeY = s.KneeHeights.y, RawAnkleY = s.AnkleHeights.x, Box = s.Min,
                UpperHip = leg.PhysicalUpperHip, UpperKnee = leg.PhysicalUpperKnee, LowerKnee = leg.PhysicalLowerKnee,
                LowerAnkle = lowerAnkle, FootAnkle = footAnkle, UpperR = leg.UpperR, GoalR = leg.HipTargetR,
                FootR = footRotation, Center = center, Half = half, FloorCenter = s.FloorCenter, FloorHalf = s.FloorHalf
            };
        }

        private void ObserveRightAnkle(in Sample s, in LegTurnStepEvidence leg, SwingKneeCandidate.Controller controller,
            Vector3 lowerAnkle, Vector3 footAnkle, Quaternion footRelative, Quaternion selectedFoot)
        {
            _rightAnkle ??= new RightAnkleWindow();
            var result = default(RightAnkleProbe.Result);
            if (RightAnkleWindow.Opens(s))
            {
                using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.RightAnkleProbe);
                try { result = CompareRightAnkle(s, leg, controller, lowerAnkle, footAnkle, footRelative, selectedFoot); }
                catch { result.State = RightAnkleProbe.Status.Fault; }
            }
            _rightAnkle.Observe(s, result);
        }

        private RightAnkleProbe.Result CompareRightAnkle(in Sample s, in LegTurnStepEvidence leg, SwingKneeCandidate.Controller controller,
            Vector3 lowerAnkle, Vector3 footAnkle, Quaternion footRelative, Quaternion selectedFoot)
        {
            var k = controller.Read();
            if (!leg.Matches(s.Step, s.Fixed) || !leg.Source.Matches(s.Source) || !leg.KneeHeightsValid ||
                !leg.KneeFeedforwardValid || !k.Matches(s.Step, s.Source) || !leg.LandingInput.Parent.Matches(s.Step, s.Source) ||
                !_relativeReference.Active || !_relativeReference.Finite || !_relativeReference.Current.Source.Matches(s.Source)) return default;
            var path = leg.HipGoalPath;
            var basis = SwingKneeCandidate.EarlyHip(path, leg.HipNormalReference, s.Step, s.Source, s.Fixed, leg.Modes,
                leg.HipNormalRecoveryHandoffStarted || leg.HipNormalRecoveryHandoffActive);
            if (basis == SwingKneeCandidate.HipPlacement.Missing) return default;
            if (basis == SwingKneeCandidate.HipPlacement.Unsupported) return new RightAnkleProbe.Result { State = RightAnkleProbe.Status.ProtectedHip };
            var scale = _collider.transform.lossyScale;
            if (!IsFinite(scale) || scale.x <= 0f || scale.y <= 0f || scale.z <= 0f) return new RightAnkleProbe.Result { State = RightAnkleProbe.Status.Geometry };
            var lower = _lowerBody.rotation;
            var hip = basis == SwingKneeCandidate.HipPlacement.SettledParent
                ? leg.LandingInput.Parent.ActualPoint : leg.LandingInput.Parent.ReferencePoint;
            var g = SwingGeometry(s, leg, hip, path.Final, lower, lowerAnkle, scale);
            var input = new RightAnkleProbe.Input
            {
                Geometry = g,
                Upper = path.PhysicalUpper,
                Lower = lower,
                RawKnee = Quaternion.Inverse(leg.RawUpperR) * leg.RawLowerR,
                Axis = controller.Axis,
                AllowExtension = controller.AllowExtension,
                FootRelative = footRelative,
                SelectedFoot = selectedFoot,
                Knee = leg.PhysicalLowerKnee,
                Gap = footAnkle - lowerAnkle,
                ObservedBox = s.Min,
                ObservedLowerY = s.AnkleHeights.y,
                SourceAnkleY = s.AnkleHeights.x,
                UpperW = leg.UpperW,
                HipTargetW = leg.HipTargetW,
                LowerW = _lowerBody.angularVelocity,
                BodyMaxW = _lowerBody.maxAngularVelocity,
                Kp = _profile.LegKneeAngularSpring,
                Kd = _profile.LegKneeAngularDamper,
                SoftKp = _profile.LegKneeSoftOffAxisAngularSpring,
                SoftKd = _profile.LegKneeSoftOffAxisAngularDamper,
                SoftLimit = controller.SoftLimit,
                FeedbackPriority = controller.FeedbackPriority,
                FlexLimit = _profile.LegKneeMaxAngularAcceleration,
                FfActive = leg.KneeFeedforwardActive,
                FfGain = leg.HipFeedforwardGain,
                FfLimit = leg.HipFeedforwardBound,
                SubmittedPd = leg.KneeFeedbackSigned,
                SubmittedFf = leg.KneeFeedforwardSelectedSigned,
                AllocatedFf = leg.KneeFeedforwardAllocatedSigned,
                SubmittedFlex = leg.KneeFlexionFinalSigned,
                RelativeCommand = leg.KneeRelativeA,
                TotalCommand = leg.KneeTotalA
            };
            var result = RightAnkleProbe.Compare(input, k, s.Step, s.Source);
            if (result.BaseMeasured)
            {
                // PRE_N point velocity is available before the solve. Its optional failure
                // cannot discard the independently closed original/candidate comparison.
                try
                {
                    float rise = _lowerBody.GetPointVelocity(input.Knee).y * s.Step.Dt;
                    if (IsFinite(rise)) { result.PivotForecast = true; result.PredictedKneeShift = rise; }
                }
                catch { result.PivotForecast = false; }
            }
            return result;
        }
    }
}
