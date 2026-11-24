using MakemitAGA.ActiveRagdollLab.Runtime;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        internal void ObserveLandingRise(in PhysicsStepContext step, SingleLegShadowChain right, Trace trace, bool leftFirst, bool pulseApplied)
        {
            if (!trace.WantsRiseStep(step)) return;
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LandingRise);
            try
            {
                _hipAnchorPair.TryGetCommittedInput(step, out var l);
                right._hipAnchorPair.TryGetCommittedInput(step, out var r);
                trace.ObserveRise(step, _torso.LandingRiseInput, l, r,
                    _hipAnchorPair.Report.Current, right._hipAnchorPair.Report.Current, _torso.HipAttitudeCommand,
                    _legTurnCurrent.ReceiverNormalResponse, leftFirst, pulseApplied, _torso.LandingHeightInput);
            }
            catch { trace.NoteRiseFault(step); }
        }

        internal void ObserveTurnHandoff(int side, in PhysicsStepContext step, Trace trace)
        {
            bool pending = trace.TryHandoff(side, step, out var tick, out var run, out string prohibit, out var early, out var braked, out var combined);
            var peak = side == 1 ? trace.RightPeak(step) : null;
            if (!pending && peak == null) return;
            void Stop(string why)
            {
                run?.Stop(why);
                early?.Stop(why);
                braked?.Stop(why);
                combined?.Stop(why);
            }
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.TurnHandoff);
            try
            {
                if (_footFoundation == null || !KneeAxisCalibrationApplied || !_legTurnCurrent.Matches(step, tick.Local) ||
                    !_legTurnCurrent.Source.Matches(tick.Source.Clock))
                { Stop("LegOrCalibration"); return; }
                var frame = _footFoundation.HandoffFrame(tick, _legTurnCurrent);
                if (!frame.Valid || !frame.FivePoint || !frame.Anchors.Valid)
                { Stop("MissingFrame"); return; }
                var b = frame.Pose.Box;
                var axis = GetCurrentKneeFlexionAxisWorld();
                var pose = new Trial.Pose
                {
                    Hip = frame.Anchors.UpperHip,
                    UpperKnee = frame.Anchors.UpperKnee,
                    LowerKnee = frame.Anchors.LowerKnee,
                    Lower = b.LowerAnkle,
                    Foot = b.FootAnkle,
                    Rotation = b.FootR,
                    Axis = axis,
                    AllowExtension = KneeGroundExtensionSelected,
                    Center = b.Center,
                    Half = b.Half,
                    FloorCenter = b.FloorCenter,
                    FloorHalf = b.FloorHalf
                };
                bool issue = run?.Visit(tick, pose, frame.UpperId, frame.LowerId, prohibit) == true;
                bool issueEarly = early?.Visit(tick, pose, frame.UpperId, frame.LowerId, prohibit) == true;
                bool issueBraked = braked?.Visit(tick, pose, frame.UpperId, frame.LowerId, prohibit) == true;
                bool issueCombined = combined?.Visit(tick, pose, frame.UpperId, frame.LowerId, prohibit) == true;
                peak?.ReadPeak(tick, pose, frame.UpperId, frame.LowerId);
                if (!issue && !issueEarly && !issueBraked && !issueCombined) return;
                if (!_footFoundation.ReadHandoffMotion(tick, frame, out var motion))
                { Stop("FootLoaded"); return; }
                var foot = RightFootLandingForecast.Predict(frame.Pose, motion);
                if (!foot.Predicted) { Stop(foot.State.ToString()); return; }
                var shin = ReadUnloadedShinCommand(step, tick.Source.Clock, frame, motion);
                if (shin.State != LeftShinForecast.Status.Predicted) { Stop(shin.State.ToString()); return; }

                // The forecast and the conditioned observation use the same material flexion axis.
                pose.Axis = shin.LowerTurn * axis;
                pose.Hip = shin.Hip; pose.UpperKnee = shin.UpperKnee; pose.LowerKnee = shin.LowerKnee;
                pose.Lower = shin.LowerAnkle; pose.Foot = foot.Foot; pose.Rotation = foot.R;
                var budget = new Trial.Budget
                {
                    Axis = axis,
                    HipA = _legTurnCurrent.HipFinalA,
                    HipCap = _legTurnCurrent.HipMaxA,
                    KneeA = _legTurnCurrent.KneeFlexionFinalSigned,
                    KneeCap = _profile.LegKneeMaxAngularAcceleration,
                    UpperW = shin.UpperW,
                    LowerW = shin.LowerW,
                    FootW = foot.W,
                    UpperMaxW = _upperBody.maxAngularVelocity,
                    LowerMaxW = _lowerBody.maxAngularVelocity,
                    FootMaxW = motion.Angular.MaxW
                };
                if (issue) run.Issue(tick, pose, budget);
                if (issueEarly) early.Issue(tick, pose, budget);
                var traction = new LandingFootTraction.Input(motion);
                if (issueBraked) braked.Issue(tick, pose, budget, traction);
                if (issueCombined) combined.Issue(tick, pose, budget, traction);
            }
            catch { Stop("CaptureFault"); }
        }
    }
}
