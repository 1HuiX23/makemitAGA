using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using MakemitAGA.ActiveRagdollLab.Physics;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Optional evidence in the existing receiver ledger. No native getters, controller
    // calls, new update loop, per-step strings, or eligibility/physics feedback.
    internal sealed partial class PelvisReceiverReport
    {
        internal struct MotionInput
        {
            internal bool Valid, InStopWindow, SourceFresh, LinearClamped;
            internal PhysicsStepContext Step, ComReferenceStep, ComFeedbackStep;
            internal PelvisComReferencePlanner.ComPlan ComReference;
            internal Vector3 NativeGoalCom, NativeGoalV;
            internal int SourceSequence, SourceEvent, TimingGroup, Stage, StopStartFixed;
            internal float SourceAge, SourceDt, Time, StopForward;
            internal Vector3 TargetCom, TargetV, Error, VelocityError, RawError, PdA, FeedforwardA;
            internal Vector3 SourceV, SourceW, StopDirection, PredictedA, ObservedA, ResidualA;
        }
        private const int MotionRawPre = 24;
        private sealed class MotionWindow
        {
            // Same capacity in both windows. Raw partitions this array into 24 rolling
            // predecessors + 33 selected frames (24 pre, trigger, 8 post). Keeping the
            // segments disjoint preserves a closed peak while later onset history rolls.
            // Stop keeps all 57 selected frames (8 pre, trigger, 48 post).
            internal readonly Frame[] Frames = new Frame[57];
            internal int Offset, PreTarget = 8, PostTarget = 48;
            internal int Count, PreCount, PostCount, TriggerFixed;
            internal float Score;
            internal bool Closed;
            internal string End;
            internal void Reset()
            { Count = PreCount = PostCount = TriggerFixed = 0; Score = 0f; Closed = false; End = null; }
        }
        private readonly MotionWindow _motionRaw = new MotionWindow { Offset = MotionRawPre, PreTarget = MotionRawPre, PostTarget = 8 };
        private readonly MotionWindow _motionStop = new MotionWindow();
        private int _motionRawHistoryNext, _motionRawHistoryCount;
        internal int MotionIssued, MotionInputMissing, MotionResolved, MotionCensored, MotionGaps;
        internal int MotionLimitedIssued, MotionLimitedResolved;
        internal int MotionReferenceSamples, MotionReferenceMissing, MotionReferenceFeedback, MotionReferenceEnabled, MotionReferenceReduced;
        internal int MotionReferenceBrakeProtected, MotionRawEligibleIssued, MotionRawEligibleResolved;
        internal float MotionReferenceRemovedPeak, MotionReferenceProtectedPeak, MotionRawPeakIssued, MotionRawPeakResolved;
        internal float MotionMinIssuedQ = 1f, MotionMinResolvedQ = 1f, MotionPeakResidualA;
        internal double MotionLimitedSeconds, MotionLongestLimitedSeconds;
        private double _motionLimitedRun;
        private PhysicsStepContext _motionLast;
        private int _motionLastFixed;
        private string _motionIssue;

        private void ResetMotionEvidence()
        {
            _motionRaw.Reset(); _motionStop.Reset();
            _motionRawHistoryNext = _motionRawHistoryCount = 0;
            MotionIssued = MotionInputMissing = MotionResolved = MotionCensored = MotionGaps = 0;
            MotionLimitedIssued = MotionLimitedResolved = 0;
            MotionReferenceSamples = MotionReferenceMissing = MotionReferenceFeedback = MotionReferenceEnabled = MotionReferenceReduced = 0;
            MotionReferenceBrakeProtected = MotionRawEligibleIssued = MotionRawEligibleResolved = 0;
            MotionReferenceRemovedPeak = MotionReferenceProtectedPeak = MotionRawPeakIssued = MotionRawPeakResolved = 0f;
            MotionMinIssuedQ = MotionMinResolvedQ = 1f; MotionPeakResidualA = 0f;
            MotionLimitedSeconds = MotionLongestLimitedSeconds = _motionLimitedRun = 0d;
            _motionLast = new PhysicsStepContext(); _motionLastFixed = 0; _motionIssue = null;
        }
        internal void CaptureMotionInput(ref Frame f, PhysicsStepContext step)
        {
            if (!TraceRunning || !TraceHipTranslation) return;
            try
            {
                MotionIssued++;
                float q = f.HipAttitude.MotionScale;
                if (FiniteValue(q) && q >= 0f && q <= 1f)
                {
                    MotionMinIssuedQ = Mathf.Min(MotionMinIssuedQ, q);
                    if (q < 1f - 0.000001f) MotionLimitedIssued++;
                }
                TrackingFrame t = LastTracking;
                if (!f.HipReactionRecorded || !f.HipAttitude.Committed || !f.HipAttitude.AllocationMatches() || !f.HipAttitude.Baseline.Step.Matches(step) ||
                    !FiniteValue(q) || q < 0f || q > 1f || !t.Valid || !t.RuntimeContextExpected || !t.RuntimeContext.Matches(step) ||
                    t.Fixed != f.Fixed || t.Dt != f.Dt || step.Dt != f.Dt || f.Dt <= 0f ||
                    !FiniteVector(t.CommandCom) || !FiniteVector(t.CommandComV) || !FiniteVector(t.ComError) ||
                    !FiniteVector(t.ComVelocityError) || !FiniteVector(t.RawRenderComError) || !FiniteVector(t.PdA) ||
                    !FiniteVector(t.FeedforwardA) || !FiniteVector(t.SourceV) || !FiniteVector(t.SourceWVector) ||
                    !FiniteValue(t.SourceAge) || !FiniteValue(t.SourceDt) || !FiniteVector(t.StopDirection))
                { MotionInputMissing++; _motionIssue = "COMMAND_TRACKING_JOIN_INVALID"; return; }
                float mass = f.PelvisMass + f.ChestMass;
                Vector3 predicted = (f.PelvisJ + f.ChestJ + f.HipLeftJ + f.HipRightJ) / (mass * f.Dt);
                if (!FiniteValue(mass) || mass <= 0f || !FiniteVector(predicted) ||
                    (predicted - f.HipAttitude.TargetSystemAcceleration).magnitude > 0.0001f * (1f + f.HipAttitude.TargetSystemAcceleration.magnitude))
                { MotionInputMissing++; _motionIssue = "COMMAND_COM_MODEL_INVALID"; return; }
                f.Motion = new MotionInput { Valid = true, Step = step, Time = t.Time,
                    TargetCom = t.CommandCom, TargetV = t.CommandComV, Error = t.ComError, VelocityError = t.ComVelocityError,
                    RawError = t.RawRenderComError, PdA = t.PdA, FeedforwardA = t.FeedforwardA, LinearClamped = t.LinearClamped,
                    SourceV = t.SourceV, SourceW = t.SourceWVector, SourceSequence = t.SourceSequence, SourceEvent = t.SourceEventId,
                    SourceFresh = t.SourceFresh, SourceAge = t.SourceAge, SourceDt = t.SourceDt, TimingGroup = t.RawTimingGroup,
                    Stage = t.Stage, InStopWindow = t.InStopWindow, StopStartFixed = t.StopStartFixed,
                    StopDirection = t.StopDirection, StopForward = Mathf.Max(0f, -Vector3.Dot(t.ComError, t.StopDirection)),
                    PredictedA = predicted, NativeGoalCom = t.NativeGoalCom, NativeGoalV = t.NativeGoalV,
                    ComReference = t.ComReference, ComReferenceStep = t.ComReferenceStep, ComFeedbackStep = t.ComFeedbackStep };
                if (RawMotionEligible(f.Motion))
                {
                    MotionRawEligibleIssued++;
                    MotionRawPeakIssued = Mathf.Max(MotionRawPeakIssued, f.Motion.RawError.magnitude);
                }
                // Optional planning evidence cannot invalidate the physical impulse join.
                bool referenceValid = t.ComReference.ArrivalEnabled && t.ComReferenceStep.Matches(step) &&
                    (!t.ComReference.FeedbackAvailable || (t.ComFeedbackStep.Valid && t.ComFeedbackStep.Epoch == step.Epoch &&
                        t.ComFeedbackStep.Sequence + 1 == step.Sequence));
                if (!referenceValid) MotionReferenceMissing++;
                else
                {
                    MotionReferenceSamples++;
                    if (t.ComReference.FeedbackAvailable) MotionReferenceFeedback++;
                    if (t.ComReference.FeedbackEnabled) MotionReferenceEnabled++;
                    if (t.ComReference.RemovedAcceleration > 0f) MotionReferenceReduced++;
                    MotionReferenceRemovedPeak = Mathf.Max(MotionReferenceRemovedPeak, t.ComReference.RemovedAcceleration);
                    if (t.ComReference.PhysicalBrakeProtected)
                    {
                        MotionReferenceBrakeProtected++;
                        MotionReferenceProtectedPeak = Mathf.Max(MotionReferenceProtectedPeak, t.ComReference.ProposedReduction);
                    }
                }
            }
            catch { MotionInputMissing++; _motionIssue = "COMMAND_CAPTURE_EXCEPTION"; }
        }
        internal void ResolveMotionInput(ref Frame f, PhysicsStepContext next)
        {
            if (!TraceRunning || !TraceHipTranslation) return;
            try
            {
                if (!f.Motion.Valid || !f.Resolved || f.Censored || !next.Valid || !f.Motion.Step.Valid ||
                    next.Epoch != f.Motion.Step.Epoch || next.Sequence != f.Motion.Step.Sequence + 1 || f.Settled != f.Fixed + 1)
                { MotionGap("RESPONSE_INPUT_OR_CLOCK_GAP"); return; }
                float mass = f.PelvisMass + f.ChestMass;
                f.Motion.ObservedA = f.SystemDeltaP / (mass * f.Dt);
                f.Motion.ResidualA = f.Motion.ObservedA - f.Motion.PredictedA;
                if (!FiniteVector(f.Motion.ObservedA) || !FiniteVector(f.Motion.ResidualA))
                { MotionGap("RESPONSE_COM_NONFINITE"); return; }
                f.MotionNextStep = next; MotionResolved++;
                MotionPeakResidualA = Mathf.Max(MotionPeakResidualA, f.Motion.ResidualA.magnitude);
                float q = f.HipAttitude.MotionScale;
                bool adjacent = _motionLast.Valid && _motionLast.Epoch == f.Motion.Step.Epoch &&
                    _motionLast.Sequence + 1 == f.Motion.Step.Sequence && _motionLastFixed + 1 == f.Fixed;
                if (!adjacent)
                {
                    _motionLimitedRun = 0d;
                    _motionRawHistoryNext = _motionRawHistoryCount = 0;
                }
                if (q < 1f - 0.000001f)
                {
                    MotionLimitedResolved++; MotionLimitedSeconds += f.Dt; _motionLimitedRun += f.Dt;
                    MotionLongestLimitedSeconds = Math.Max(MotionLongestLimitedSeconds, _motionLimitedRun);
                    MotionMinResolvedQ = Mathf.Min(MotionMinResolvedQ, q);
                }
                else _motionLimitedRun = 0d;
                // Append posthistory before a stronger candidate replaces a window. Equal
                // scores keep the first occurrence. Windows never stitch across a gap/epoch.
                ContinueMotionWindow(_motionRaw, f); ContinueMotionWindow(_motionStop, f);
                // Reuse the old minimum-q window for the independent raw-animation
                // COM peak; per-side minimum-q witnesses and aggregate q counts remain.
                // Unknown/stale source ages cannot win this qualified peak. Issued-only
                // final commands remain counted, without a synthetic settled response.
                if (RawMotionEligible(f.Motion))
                {
                    MotionRawEligibleResolved++;
                    float rawError = f.Motion.RawError.magnitude;
                    MotionRawPeakResolved = Mathf.Max(MotionRawPeakResolved, rawError);
                    if (_motionRaw.TriggerFixed == 0 || rawError > _motionRaw.Score)
                        SelectMotionWindow(_motionRaw, f, rawError);
                }
                if (f.Motion.InStopWindow && f.Motion.TimingGroup <= 1 && f.Motion.StopForward > 0f &&
                    (_motionStop.TriggerFixed == 0 || f.Motion.StopForward > _motionStop.Score))
                    SelectMotionWindow(_motionStop, f, f.Motion.StopForward);
                // Only resolved, contiguous responses enter the history segment;
                // raw-peak eligibility does not erase useful neighbouring source ages.
                // Store AFTER selection so the trigger cannot become its own predecessor.
                _motionRaw.Frames[_motionRawHistoryNext] = f;
                _motionRawHistoryNext = (_motionRawHistoryNext + 1) % MotionRawPre;
                _motionRawHistoryCount = Math.Min(_motionRawHistoryCount + 1, MotionRawPre);
                _motionLast = f.Motion.Step; _motionLastFixed = f.Fixed;
            }
            catch { MotionGap("RESPONSE_CAPTURE_EXCEPTION"); }
        }
        private static bool RawMotionEligible(MotionInput m) => m.Valid && m.TimingGroup >= 0 && m.TimingGroup <= 1 &&
            FiniteValue(m.SourceAge) && m.SourceAge >= 0f && m.SourceAge <= RawTimingAgeLimit && FiniteVector(m.RawError);
        private void SelectMotionWindow(MotionWindow w, Frame trigger, float score)
        {
            w.Reset(); w.TriggerFixed = trigger.Fixed; w.Score = score; w.End = "POST_PENDING";
            for (int distance = 1; distance <= w.PreTarget; distance++)
            {
                bool found = false;
                if (w.Offset > 0)
                {
                    if (distance > _motionRawHistoryCount) break;
                    Frame prior = w.Frames[(_motionRawHistoryNext - distance + MotionRawPre) % MotionRawPre];
                    if (MotionPredecessor(prior, trigger, distance))
                    { w.Frames[w.Offset + w.PreTarget - distance] = prior; w.PreCount++; found = true; }
                }
                else
                {
                    for (int i = 0; i < Count; i++)
                    {
                        Frame prior = Ring[(Next - Count + i + Ring.Length) % Ring.Length];
                        if (MotionPredecessor(prior, trigger, distance))
                        { w.Frames[w.PreTarget - distance] = prior; w.PreCount++; found = true; break; }
                    }
                }
                if (!found) break;
            }
            for (int i = 0; i < w.PreCount; i++) w.Frames[w.Offset + i] = w.Frames[w.Offset + w.PreTarget - w.PreCount + i];
            w.Count = w.PreCount; w.Frames[w.Offset + w.Count++] = trigger;
        }
        private static bool MotionPredecessor(Frame prior, Frame trigger, int distance) =>
            prior.Resolved && !prior.Censored && prior.Motion.Valid && prior.MotionNextStep.Valid &&
            prior.Fixed == trigger.Fixed - distance && prior.Motion.Step.Epoch == trigger.Motion.Step.Epoch &&
            prior.Motion.Step.Sequence == trigger.Motion.Step.Sequence - distance;
        private static void ContinueMotionWindow(MotionWindow w, Frame f)
        {
            if (w.TriggerFixed == 0 || w.Closed) return;
            Frame last = w.Frames[w.Offset + w.Count - 1];
            if (last.Fixed == f.Fixed && last.Motion.Step.Matches(f.Motion.Step)) return;
            if (last.Fixed + 1 != f.Fixed || last.Motion.Step.Epoch != f.Motion.Step.Epoch ||
                last.Motion.Step.Sequence + 1 != f.Motion.Step.Sequence)
            { w.Closed = true; w.End = "CENSORED_RESPONSE_GAP"; return; }
            w.Frames[w.Offset + w.Count++] = f; w.PostCount++;
            if (w.PostCount == w.PostTarget) { w.Closed = true; w.End = "COMPLETE"; }
        }
        private void MotionGap(string issue)
        {
            MotionGaps++; _motionIssue = issue; _motionLimitedRun = 0d;
            _motionRawHistoryNext = _motionRawHistoryCount = 0;
            CloseMotionWindow(_motionRaw, "CENSORED_DIAGNOSTIC_GAP"); CloseMotionWindow(_motionStop, "CENSORED_DIAGNOSTIC_GAP");
        }
        private static void CloseMotionWindow(MotionWindow w, string reason)
        { if (w.TriggerFixed > 0 && !w.Closed) { w.Closed = true; w.End = reason; } }
        internal void CensorMotionInput(Frame f)
        { if (TraceHipTranslation && f.HipReactionRecorded && f.Censored) MotionCensored++; }
        internal void SealMotionEvidence()
        {
            CloseMotionWindow(_motionRaw, "CENSORED_TRACE_STOP"); CloseMotionWindow(_motionStop, "CENSORED_TRACE_STOP");
        }
        private void AppendMotionEvidence(StringBuilder sb)
        {
            if (!TraceHipTranslation) return;
            try
            {
                sb.Append("\nreceiverMotionEvidence=V041050_READ_ONLY/physicalArm=").Append(TraceArm ?? "<none>")
                    .Append("/issued-inputMissing-resolved-censored-gaps=").Append(MotionIssued).Append('|').Append(MotionInputMissing)
                    .Append('|').Append(MotionResolved).Append('|').Append(MotionCensored).Append('|').Append(MotionGaps)
                    .Append("/qLimited(issued-resolved-minIssued-minResolved)=").Append(MotionLimitedIssued).Append('|').Append(MotionLimitedResolved)
                    .Append('|').Append(F(MotionMinIssuedQ)).Append('|').Append(F(MotionMinResolvedQ))
                    .Append("/qLimitedSeconds(total-longest)=").Append(F((float)MotionLimitedSeconds)).Append('|').Append(F((float)MotionLongestLimitedSeconds))
                    .Append("/comReference(samples-missing-previousAvailable-feedbackEnabled-reduced)=").Append(MotionReferenceSamples).Append('|').Append(MotionReferenceMissing)
                    .Append('|').Append(MotionReferenceFeedback).Append('|').Append(MotionReferenceEnabled).Append('|').Append(MotionReferenceReduced)
                    .Append("/comReferenceRemovedAPeak=").Append(F(MotionReferenceRemovedPeak))
                    .Append("/comPhysicalBrakeProtected(count-proposedAPeak)=").Append(MotionReferenceBrakeProtected).Append('|').Append(F(MotionReferenceProtectedPeak))
                    .Append("/rawEligibility=FRESH_OR_HELD_LE100MS/rawEligible(issued-resolved)=").Append(MotionRawEligibleIssued).Append('|').Append(MotionRawEligibleResolved)
                    .Append("/rawQualifiedPeakMm(issued-resolved)=").Append(F(MotionRawPeakIssued * 1000f)).Append('|').Append(F(MotionRawPeakResolved * 1000f))
                    .Append("/systemResidualAPeak=").Append(F(MotionPeakResidualA)).Append("/issue=").Append(_motionIssue ?? "<none>")
                    .Append("/model=P_PLUS_C_MASS_WEIGHTED_COM_INPUT_LEDGER/qMeaning=SYSTEM_COM_AND_NET_UPPER_REQUEST/commonActuatorCapMps2=12/notPelvisPointA=True/notWholeBody=True/diagnosticFeedback=False");
                AppendMotionWindow(sb, "RAW_COM_PEAK", _motionRaw); AppendMotionWindow(sb, "STOP_FORWARD_PEAK", _motionStop);
            }
            catch { sb.Append("\nreceiverMotionEvidence=FORMAT_UNAVAILABLE"); }
        }
        private static void AppendComReference(StringBuilder sb, PelvisComReferencePlanner.ComPlan p, PhysicsStepContext step, PhysicsStepContext previous)
        {
            if (!p.ArrivalEnabled) return;
            sb.Append("/comPlanner=RELATIVE_V_ARRIVAL")
                .Append("/comPlannerStep(epoch-fixed-source-time-dt)=").Append(step.Epoch).Append('|').Append(step.Sequence).Append('|').Append(step.SourceSequence)
                .Append('|').Append(F(step.FixedTime)).Append('|').Append(F(step.Dt))
                .Append("/comDeficitFeedbackEnabled=").Append(p.FeedbackEnabled)
                .Append("/comPreviousAvailable=").Append(p.FeedbackAvailable)
                .Append("/comPreviousStep(epoch-fixed-source-time-dt)=").Append(previous.Epoch).Append('|').Append(previous.Sequence).Append('|').Append(previous.SourceSequence)
                .Append('|').Append(F(previous.FixedTime)).Append('|').Append(F(previous.Dt))
                .Append("/comPlan(relativeV-stopOffset-arrivalError)=").Append(V(p.RelativeV)).Append('|').Append(V(p.StoppingOffset)).Append('|').Append(V(p.ArrivalError))
                .Append("/comPlanA(candidate-final-removed)=").Append(V(p.CandidateA)).Append('|').Append(V(p.Acceleration)).Append('|').Append(F(p.RemovedAcceleration))
                .Append("/comBodyV-brakeProtected-proposedReduction=").Append(V(p.SettledVelocity)).Append('|').Append(p.PhysicalBrakeProtected).Append('|').Append(F(p.ProposedReduction))
                .Append("/comPreviousA(request-allocated)=").Append(V(p.PreviousRequestA)).Append('|').Append(V(p.PreviousAllocatedA));
        }
        private static void AppendMotionWindow(StringBuilder sb, string tag, MotionWindow w)
        {
            sb.Append("\nreceiverMotionWindow=").Append(tag).Append("/trigger=").Append(w.TriggerFixed)
                .Append("/score=").Append(F(w.Score)).Append("/pre-post=").Append(w.PreCount).Append('|').Append(w.PostCount)
                .Append("/preTarget-postTarget=").Append(w.PreTarget).Append('|').Append(w.PostTarget)
                .Append("/preMissing-postMissing=").Append(w.PreTarget - w.PreCount).Append('|').Append(w.PostTarget - w.PostCount)
                .Append("/result=").Append(w.TriggerFixed == 0 ? "NO_QUALIFIED_CANDIDATE" : w.End);
            for (int i = 0; i < w.Count; i++)
            {
                Frame f = w.Frames[w.Offset + i]; MotionInput m = f.Motion; PelvisHipAttitudeCommand a = f.HipAttitude;
                float mass = f.PelvisMass + f.ChestMass;
                sb.Append("\nreceiverMotionFrame=").Append(tag).Append("/fixed-next=").Append(f.Fixed).Append('|').Append(f.Settled)
                    .Append('/').Append(m.Step.Status()).Append("/next=").Append(f.MotionNextStep.Status())
                    .Append("/time-dt=").Append(F(m.Time)).Append('|').Append(F(f.Dt)).Append("/massPC=").Append(F(f.PelvisMass)).Append('|').Append(F(f.ChestMass))
                    .Append("/source(seq-event-fresh-age-dt-timing-stage)=").Append(m.SourceSequence).Append('|').Append(m.SourceEvent).Append('|').Append(m.SourceFresh)
                    .Append('|').Append(F(m.SourceAge)).Append('|').Append(F(m.SourceDt)).Append('|').Append(m.TimingGroup).Append('|').Append(m.Stage)
                    .Append("/sourceV-W=").Append(V(m.SourceV)).Append('|').Append(V(m.SourceW))
                    .Append("/targetCOM-V=").Append(V(m.TargetCom)).Append('|').Append(V(m.TargetV))
                    .Append("/nativeGoalCOM(p-v)=").Append(V(m.NativeGoalCom)).Append('|').Append(V(m.NativeGoalV))
                    .Append("/errorCOM-V-rawCOM=").Append(V(m.Error)).Append('|').Append(V(m.VelocityError)).Append('|').Append(V(m.RawError))
                    .Append("/COMvelocity(pre-post)=").Append(V((f.PelvisMass * f.PrePelvisV + f.ChestMass * f.PreChestV) / mass))
                    .Append('|').Append(V((f.PelvisMass * f.PostPelvisV + f.ChestMass * f.PostChestV) / mass))
                    .Append("/driveA(pd-ff-request-actuator)=").Append(V(m.PdA)).Append('|').Append(V(m.FeedforwardA)).Append('|').Append(V(a.Baseline.Acceleration)).Append('|').Append(V(a.CommonAcceleration))
                    .Append("/systemTargetA=").Append(V(a.TargetSystemAcceleration))
                    .Append("/q-linearClamped=").Append(F(a.MotionScale)).Append('|').Append(m.LinearClamped)
                    .Append("/HipJ(left-right)=").Append(V(f.HipLeftJ)).Append('|').Append(V(f.HipRightJ))
                    .Append("/systemA(predicted-observed-residual)=").Append(V(m.PredictedA)).Append('|').Append(V(m.ObservedA)).Append('|').Append(V(m.ResidualA))
                    .Append("/stop(active-start-direction-forwardMm)=").Append(m.InStopWindow).Append('|').Append(m.StopStartFixed)
                    .Append('|').Append(V(m.StopDirection)).Append('|').Append(F(m.StopForward * 1000f));
                AppendComReference(sb, m.ComReference, m.ComReferenceStep, m.ComFeedbackStep);
            }
        }
    }
}
