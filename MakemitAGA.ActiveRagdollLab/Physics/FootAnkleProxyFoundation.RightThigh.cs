using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private RightThighWindow _rightThigh;
        private RightThighWindow.Signal ObserveRightThigh(in Sample sample, in RightKneePivot.Frame pivot)
        {
            try
            {
                _rightThigh ??= new RightThighWindow();
                return _rightThigh.Observe(sample, pivot);
            }
            catch { _rightThigh?.Cut(); return RightThighWindow.Signal.None; }
        }

        internal void CompleteRightThigh(PhysicsStepContext step, in LegTurnStepEvidence leg, Rigidbody upper,
            bool kneeTransport, HipAnchorTranslationPair left, HipAnchorTranslationPair right)
        {
            if (_rightThigh == null || !_rightThigh.Requested(step)) return;
            using var timing = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.RightThighTracking);
            RightThighTracking.Result result = default;
            try
            {
                var req = _rightThigh.Current;
                if (left != null && right != null && _report.Side == "RIGHT" && leg.Matches(step, req.Local) && leg.Source.Matches(req.Source) &&
                    _report.RuntimeContext.Matches(step) && _report.RuntimeSourceContext.Matches(req.Source) &&
                    LastCompletedControlFixedSequence == req.Local && _report.FixedUpdateSequence == req.Local && kneeTransport &&
                    left.TryGetCommittedInput(step, out var l) && right.TryGetCommittedInput(step, out var r) &&
                    r.Seed.Fixed == req.Local && upper.GetInstanceID() == r.Seed.ChildId &&
                    RightThighTracking.Same(upper.angularVelocity, r.ChildW) && leg.HipGoalPath.Valid &&
                    FootRelativeReference.Log(leg.HipGoalPath.Final * Quaternion.Inverse(req.Pose.GoalR)).sqrMagnitude <= 1e-8f &&
                    FootRelativeReference.Log(leg.HipGoalPath.PhysicalUpper * Quaternion.Inverse(req.Pose.UpperR)).sqrMagnitude <= 1e-8f &&
                    RightThighTracking.Same(leg.HipFinalA, right.Report.Current.AllocatedHipAlpha) &&
                    RightThighTracking.Same(leg.KneeTotalA, leg.KneeRelativeA + leg.HipFinalA))
                {
                    var path = SwingKneeCandidate.EarlyHip(leg.HipGoalPath, leg.HipNormalReference, step, req.Source, req.Local,
                        leg.Modes, leg.HipNormalRecoveryHandoffStarted || leg.HipNormalRecoveryHandoffActive);
                    if (path == SwingKneeCandidate.HipPlacement.SettledParent || path == SwingKneeCandidate.HipPlacement.ReceiverReference)
                    {
                        var p = req.Pose;
                        var motion = new RightThighTracking.Motion {
                            HipMaxA = leg.HipMaxA, ReferenceMaxA = leg.HipNormalReference.MaxA, ReferenceMaxW = leg.HipNormalReference.MaxW,
                            UpperMaxW = upper.maxAngularVelocity, LowerMaxW = _lowerBody.maxAngularVelocity, FootMaxW = _body.maxAngularVelocity,
                            LowerW = _lowerBody.angularVelocity, FootW = _body.angularVelocity, KneeRelativeA = leg.KneeRelativeA,
                            KneeGapV = _lowerBody.GetPointVelocity(p.LowerKnee) - upper.GetPointVelocity(p.UpperKnee),
                            AnkleGapV = _body.GetPointVelocity(p.FootAnkle) - _lowerBody.GetPointVelocity(p.LowerAnkle)
                        };
                        result = RightThighTracking.Compare(p, motion, l, r, left.Report.Current, right.Report.Current);
                    }
                    else if (path == SwingKneeCandidate.HipPlacement.Unsupported) result.State = RightThighTracking.Status.Protected;
                    result.Path = path;
                }
            }
            catch { result.State = RightThighTracking.Status.Fault; }
            _rightThigh.Complete(result);
        }
    }
}
