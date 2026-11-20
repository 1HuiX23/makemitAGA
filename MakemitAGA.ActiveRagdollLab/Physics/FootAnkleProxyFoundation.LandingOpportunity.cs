using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootAnkleProxyFoundation
    {
        private readonly FootLandingOpportunityAudit _landingOpportunity = new FootLandingOpportunityAudit();
        private readonly SwingLandingWindow _swingWindow = new SwingLandingWindow();
        private readonly EarlyLandingWindow _earlyWindow = new EarlyLandingWindow();
        private bool _swingObserved, _earlyObserved;
        private static FootLandingOpportunityAudit.Phase LandingPhase(string gait) =>
            gait == FootGameNativeLocomotionPhaseObserver.StateStance ? FootLandingOpportunityAudit.Phase.Stance :
            gait == FootGameNativeLocomotionPhaseObserver.StateToStance ? FootLandingOpportunityAudit.Phase.Approach :
            gait == FootGameNativeLocomotionPhaseObserver.StateSwing ? FootLandingOpportunityAudit.Phase.Swing :
            gait == FootGameNativeLocomotionPhaseObserver.StateToSwing ? FootLandingOpportunityAudit.Phase.Lift : FootLandingOpportunityAudit.Phase.Unknown;

        private void ObserveLandingOpportunity(PhysicsStepContext step, int localFixed, float age, float gap,
            Quaternion actualRelative, Vector3 lowerAnkle, Vector3 footAnkle, in LegTurnStepEvidence leg, bool wasActive, SwingKneeCandidate.Controller swingKnee, Quaternion footWorldRotation,
            in FootStanceOnlyAuthorizedContactLifecycle.EntryContext entry)
        {
            var timing = LandingObservationTiming.Run;
            using var measurement = timing.Measure(wasActive ? LandingObservationTiming.Stage.ObserverActive :
                LandingObservationTiming.Stage.ObserverInactive);
            try
            {
                string gait = _gameNativeGaitPhase.State;
                var phase = LandingPhase(gait);
                if (timing.SwingProbe) _swingObserved = true;
                if (timing.EarlyProbe) _earlyObserved = true;
                if (wasActive && timing.Compact)
                {
                    // Active steps need only the original clock and gait edge for the next inactive step.
                    if (timing.SwingProbe) _swingWindow.Cut();
                    if (timing.EarlyProbe) { _earlyWindow.Cut(); _rightAnkle?.Cut(); _rightThigh?.Cut(); _rightFootForecast?.Cut(RightFootForecastWindow.Interruption.ActiveAtEntry); _rightFootLanding?.Cut(RightFootLandingWindow.End.Active); }
                    _landingOpportunity.ObserveActive(step, _report.RuntimeSourceContext, localFixed,
                        phase, _gameNativeGaitPhase.CurrentStrongStanceSample, FormalStanceContactActive);
                    return;
                }
                var g = _stanceContactLifecycle.DiagnosticEntryGeometry;
                bool legMatches = leg.Matches(step, localFixed) && leg.Source.Matches(_report.RuntimeSourceContext);
                var path = g.LandingPath;
                bool parent = path.Valid && path.ParentPlacementValid && path.Step.Matches(step) &&
                    path.Source.Matches(_report.RuntimeSourceContext) && path.Fixed == localFixed;
                var prep = leg.LandingPreparation;
                float rawY = path.ParentRaw.y, referenceY = path.ParentReference.y, actualY = path.ParentActual.y;
                if (!parent && timing.Compact && g.Valid && legMatches && !leg.LandingInput.Prepared)
                    parent = LandingReferenceCapturePolicy.TryParentHeights(leg.LandingInput, step,
                        _report.RuntimeSourceContext, localFixed, out rawY, out referenceY, out actualY);
                // Values are captured before lifecycle enable/release. The diagnostic
                // selected-Box fallback does not qualify current probe identity; the
                // copied geometry survives ResetStanceHold without rewriting PRE_N.
                var controlRelative = ControlRelativeRotation;
                var sample = new FootLandingOpportunityAudit.Sample
                {
                    Step = step, Source = _report.RuntimeSourceContext, Fixed = localFixed,
                    SupportId = g.SupportId, FloorCenter = g.FloorCenter, FloorHalf = g.FloorHalf,
                    Gait = phase, SourceMotion = FootLandingOpportunityAudit.ClassifySource(legMatches ? leg.SourcePhase : null),
                    WasActive = wasActive, NowActive = FormalStanceContactActive, PrecontactOrigin = _stanceContactLifecycle.PrecontactEventOrigin, FootSide = _report.Side, Strong = _gameNativeGaitPhase.CurrentStrongStanceSample,
                    DepthLatchFixed = _stanceContactLifecycle.FirstMissDepthFixed, FirstMiss = _stanceContactLifecycle.FirstMissThisStep, MissInProgress = _stanceContactLifecycle.MissInProgress,
                    OrdinaryEnabled = _stanceContactLifecycle.OrdinaryEnabledThisStep,
                    RecoveryEnabled = _stanceContactLifecycle.RecoveryEnabledThisStep,
                    Identity = _report.ContactIdentityCurrentQualified && g.SupportId != 0 &&
                        _report.ContactIdentityCurrentColliderInstanceId == g.SupportId,
                    FivePoint = _report.GroundGeometryReady && _report.GroundProbeHitCount == 5 &&
                        _report.GroundFootprintSameSurface && !_report.GroundColliderHasAttachedRigidbody &&
                        _report.GroundColliderInstanceId == g.SupportId,
                    Adopted = legMatches && prep.Applied && prep.Step.Matches(step) && prep.Source.Matches(_report.RuntimeSourceContext),
                    GeometryValid = g.Valid, MotionValid = g.MotionValid && g.LowestMotionValid,
                    TrackingValid = g.ReferencesValid && IsFinite(lowerAnkle) && IsFinite(footAnkle) &&
                        ValidSourceRotation(actualRelative) && ValidSourceRotation(controlRelative),
                    ParentValid = parent, FullFootprint = g.FullFootprint, SelectedBoxFallback = g.DiagnosticSelectedBox,
                    Min = g.MinClearance, PredictedMin = g.PredictedMinClearance, CornerVelocity = g.LowestCornerNormalVelocity,
                    AnchorGap = gap, SourceAge = age, RawMin = g.SourceMinClearance, ControlMin = g.ControlMinClearance,
                    AnkleDeltaY = footAnkle.y - lowerAnkle.y, RelativeErrorDegrees = Quaternion.Angle(actualRelative, controlRelative),
                    SourcePhase = legMatches ? leg.SourcePhase : null,
                    BoxReferencesValid = g.ReferencesValid,
                    KneeHeightsValid = legMatches && leg.KneeHeightsValid, KneeHeights = leg.KneeHeights,
                    AnkleHeightsValid = g.ReferencesValid && IsFinite(g.SourceAnkleY) && IsFinite(lowerAnkle) && IsFinite(footAnkle),
                    AnkleHeights = new Vector3(g.SourceAnkleY, lowerAnkle.y, footAnkle.y),
                    RawHipY = rawY, ReferenceHipY = referenceY, ActualHipY = actualY
                };
                RightKneePivot.Frame pivot = default;
                if (timing.SwingProbe)
                {
                    sample.SwingMeasured = true;
                    try
                    {
                        sample.SwingSignal = _swingWindow.Observe(sample);
                        if (sample.SwingSignal == SwingLandingWindow.Signal.Diverging)
                        {
                            using var probeTime = timing.Measure(LandingObservationTiming.Stage.SwingKneeProbe);
                            sample.SwingCandidate = CompareSwingKnee(sample, leg, swingKnee, lowerAnkle, footAnkle, actualRelative, controlRelative);
                            _swingWindow.Record(sample.SwingCandidate.Reason);
                        }
                    }
                    catch
                    {
                        _swingWindow.Fault();
                        if (sample.SwingSignal == SwingLandingWindow.Signal.Diverging)
                        { sample.SwingCandidate.Reason = SwingKneeCandidate.Status.Fault; _swingWindow.Record(sample.SwingCandidate.Reason); }
                    }
                }
                else if (timing.EarlyProbe)
                {
                    sample.ParentHeightMeasured = true;
                    sample.EarlyFoot = EarlyLandingWindow.Side(_report.Side);
                    try
                    {
                        sample.EarlySignal = _earlyWindow.Observe(sample, sample.EarlyFoot);
                        sample.EarlyOrigin = _earlyWindow.Origin;
                        sample.RightWarningOrigin = _earlyWindow.RightWarningOrigin;
                        sample.ParentHeightRequested = _earlyWindow.HeightTrial;
                        if (sample.EarlyFoot == EarlyLandingWindow.Foot.Right)
                        {
                            pivot = CaptureRightKneePivot(sample, leg, g, lowerAnkle, footAnkle, footWorldRotation);
                            ObserveRightAnkle(sample,leg,swingKnee,lowerAnkle,footAnkle,actualRelative,controlRelative);
                            sample.RightThigh = ObserveRightThigh(sample, pivot);
                            ObserveRightFootForecast(sample, leg, g, lowerAnkle, footAnkle, footWorldRotation);
                        }
                        if (sample.ParentHeightRequested)
                        {
                            _parentHeight ??= new ParentHeightProbe.Trace();
                            _parentHeight.Begin(new ParentHeightProbe.Request(sample,sample.EarlyOrigin,
                                _earlyWindow.HeightLoss,_earlyWindow.FollowChange));
                        }
                        if (EarlyLandingWindow.LeftTrial(sample.EarlySignal))
                        {
                            using var probeTime = timing.Measure(LandingObservationTiming.Stage.EarlyKneeProbe);
                            sample.SwingCandidate = CompareSwingKnee(sample, leg, swingKnee, lowerAnkle, footAnkle, actualRelative, controlRelative, early: true);
                            _earlyWindow.Record(sample.SwingCandidate.Reason);
                        }
                    }
                    catch
                    {
                        _earlyWindow.Fault(); _rightAnkle?.Cut(); _rightThigh?.Cut(); _rightFootForecast?.Cut(RightFootForecastWindow.Interruption.Fault);
                        if (EarlyLandingWindow.LeftTrial(sample.EarlySignal))
                        { sample.SwingCandidate.Reason = SwingKneeCandidate.Status.Fault; _earlyWindow.Record(sample.SwingCandidate.Reason); }
                    }
                    if (sample.EarlyFoot == EarlyLandingWindow.Foot.Right)
                        ObserveRightFootLanding(ref sample, g, entry, lowerAnkle, footAnkle, footWorldRotation);
                }
                _landingOpportunity.Observe(sample, timing.History, pivot);
            }
            catch { if (timing.SwingProbe) _swingWindow.Fault(); if (timing.EarlyProbe) { _earlyWindow.Fault(); _rightAnkle?.Cut(); _rightThigh?.Cut(); _rightFootForecast?.Cut(RightFootForecastWindow.Interruption.Fault); _rightFootLanding?.Cut(RightFootLandingWindow.End.Fault); } _landingOpportunity.Fault(); } // Optional diagnostics never interrupt or grant control.
        }

        private KneeGroundTarget.Geometry SwingGeometry(in FootLandingOpportunityAudit.Sample sample,
            in LegTurnStepEvidence leg, Vector3 hip, Quaternion upper, Quaternion lower, Vector3 lowerAnkle, Vector3 scale)
        {
            return new KneeGroundTarget.Geometry
            {
                Hip = hip, Upper = upper, UpperSpan = leg.UpperLocalSpan,
                LowerSpan = Quaternion.Inverse(lower) * (lowerAnkle - leg.PhysicalLowerKnee),
                FootRelative = _sampledSourceRelative, FootBindRelative = _bindSourceRelative,
                Center = Vector3.Scale(_collider.center - _footAnkleAnchorLocal, scale),
                Half = Vector3.Scale(_collider.size, scale) * .5f,
                FloorCenter = sample.FloorCenter, FloorHalf = sample.FloorHalf
            };
        }

        private SwingKneeCandidate.Result CompareSwingKnee(in FootLandingOpportunityAudit.Sample sample, in LegTurnStepEvidence leg,
            SwingKneeCandidate.Controller controller, Vector3 lowerAnkle, Vector3 footAnkle, Quaternion actualFoot, Quaternion selectedFoot, bool early = false)
        {
            var k = controller.Read();
            var h = leg.HipNormalReference;
            var receipt = new SwingKneeCandidate.Result { Reason = SwingKneeCandidate.Status.Unavailable };
            if (early)
            {
                receipt.HipModes = leg.Modes; receipt.RecoveryBits = leg.HipFeedforwardStateBits;
                receipt.Handoff = leg.HipNormalRecoveryHandoffActive || leg.HipNormalRecoveryHandoffStarted;
                receipt.HipReason = h.Matches(sample.Step) ? h.Reason : null;
                receipt.KneeFeedforward = leg.KneeFeedforwardValid ? leg.KneeFeedforwardReason : null;
            }
            if (!leg.Matches(sample.Step,sample.Fixed) || !leg.Source.Matches(sample.Source) || !leg.KneeHeightsValid ||
                !leg.LandingInput.Parent.Matches(sample.Step,sample.Source) || !k.Matches(sample.Step,sample.Source) ||
                !_relativeReference.Active || !_relativeReference.Current.Source.Matches(sample.Source))
            { receipt.Gate = "MISSING_LEG_KNEE_PARENT_OR_FOOT"; return receipt; }
            bool handoff = leg.HipNormalRecoveryHandoffActive || leg.HipNormalRecoveryHandoffStarted;
            var parent = leg.LandingInput.Parent;
            Vector3 hip = parent.ReferencePoint;
            Quaternion upperGoal = leg.HipTargetR, measuredUpper = leg.UpperR;
            if (early)
            {
                var path = leg.HipGoalPath;
                var basis = SwingKneeCandidate.EarlyHip(path,h,sample.Step,sample.Source,sample.Fixed,leg.Modes,handoff);
                if (basis == SwingKneeCandidate.HipPlacement.Missing || !_relativeReference.Finite)
                { receipt.Gate = "MISSING_COMMITTED_PATH_OR_FOOT"; return receipt; }
                if (basis == SwingKneeCandidate.HipPlacement.Unsupported)
                { receipt.Reason = SwingKneeCandidate.Status.Protected; receipt.Gate = "STALE_OR_UNSUPPORTED_HIP"; return receipt; }
                // The protected controller issued its own world R/W. Place that rotation at the
                // settled mechanical parent anchor and retain the real PRE_N knee and ankle gap.
                bool settled = basis == SwingKneeCandidate.HipPlacement.SettledParent;
                receipt.HipBasis = settled ? "SETTLED_PARENT" : "RECEIVER_REFERENCE";
                hip = settled ? parent.ActualPoint : parent.ReferencePoint;
                upperGoal = path.Final; measuredUpper = path.PhysicalUpper;
            }
            else if (!SwingKneeCandidate.OrdinaryHip(h,sample.Step,handoff))
            { receipt.Reason = SwingKneeCandidate.Status.Protected; return receipt; }
            var lower = _lowerBody.rotation;
            var scale = _collider.transform.lossyScale;
            if (!IsFinite(scale) || scale.x<=0f || scale.y<=0f || scale.z<=0f)
                { receipt.Reason = SwingKneeCandidate.Status.Geometry; receipt.Gate = "COLLIDER_SCALE"; return receipt; }
            var g = SwingGeometry(sample,leg,hip,upperGoal,lower,lowerAnkle,scale);
            var pose = new SwingKneeCandidate.Pose { Reference=g, PhysicalKnee=leg.PhysicalLowerKnee, AnkleGap=footAnkle-lowerAnkle,
                Axis=controller.Axis, AllowExtension=controller.AllowExtension, HipW=leg.HipTargetW, PhysicalUpper=measuredUpper,
                PhysicalLower=lower, PhysicalFootRelative=actualFoot, SelectedFoot=selectedFoot, ObservedBox=sample.Min, LowerAnkleY=lowerAnkle.y };
            var result = SwingKneeCandidate.Evaluate(pose,k,sample.Step,sample.Source,measureRoom: early);
            if (early)
            {
                result.HipBasis = receipt.HipBasis; result.HipReason = receipt.HipReason; result.HipModes = receipt.HipModes;
                result.RecoveryBits = receipt.RecoveryBits; result.Handoff = receipt.Handoff; result.KneeFeedforward = receipt.KneeFeedforward;
            }
            _swingWindow.KeepPose(sample.Fixed,sample.SupportId,pose,k,result);
            return result;
        }
    }
}
