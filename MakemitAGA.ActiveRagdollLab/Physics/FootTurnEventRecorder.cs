using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V04650: numeric-only, bounded diagnostic for dynamic-parent runs; trace scope is pinned at start.
    // No controller, lifecycle, readiness or safety decision consumes this recorder.
    // Command N is joined to the settled pre-controller N+1 state using N's dt.
    // Windows are candidates for inspection, NOT an automatic overshoot certification.
    internal sealed partial class FootTurnEventRecorder
    {
        private const int Before = 16, After = 48, Capacity = Before + 1 + After;
        private const float QuietW = 0.05f;

        private struct Frame
        {
            internal PhysicsStepContext Step;
            internal int Fixed, Render, Held, NextSequence;
            internal float Age, SourceDt, Error, RawError, RawOpening, RelativeOpening;
            internal bool Fresh, SourceWValid, Stale, CommonWarmup, CommonActive;
            internal bool RelativeAClamp, RelativeTauClamp, CommonAClamp, CommonTauClamp, Resolved;
            internal Quaternion RawR, SourceRelR, LowerR, FootR, PelvisR, ReferenceR;
            internal Vector3 RawW, SourceW, SourceA, ErrorLocal, ErrorWorld, BodyRelW;
            internal Vector3 LowerW, FootW, PelvisW, ReferenceW;
            internal Vector3 P, D, FF, RelativePredictedA, RelativeTau;
            internal Vector3 CommonMeasuredA, CommonRequestedA, CommonTau;
            internal Vector3 NextLowerW, NextFootW, LockedNextErrorWorld;
            internal Vector3 MeasuredLowerA, MeasuredFootA;
            internal LegTurnStepEvidence Leg;
            internal bool LegValid;
            internal SourceStepContext CommandSource;
            internal StepProgress Progress;
            internal Vector3 ConditionedA;
            internal string ConditionerStage;
            internal bool ConditionerRawValid, ConditionerHigh, ConditionerConfirmed;
            internal bool ReferenceSelected, ReferenceActive, ReferenceWitnessValid;
            internal FootRelativeReference.Sample Reference;
            internal Quaternion RawSourceRelR;
            internal Vector3 RawSourceW, RawSourceA;
            internal float RawLocalError, ReferenceGap;
            internal bool HipWitnessValid;
            internal float HipRawLocalError, HipControlError, HipRawWorldError, HipTargetGap;
            internal HipProgress HipResponse;
            internal int HipScope;
            internal bool NormalPlanGapValid;
            internal float NormalPlanGap;
        }

        private sealed class Window
        {
            internal readonly Frame[] Frames = new Frame[Capacity];
            internal int Count, TriggerSequence, Remaining;
            internal float Score;
            internal bool Collecting;
            internal void Reset() { Count = TriggerSequence = Remaining = 0; Score = 0f; Collecting = false; }
        }

        private readonly Frame[] _ring = new Frame[Before];
        private readonly Window _peak = new Window(), _normalHip = new Window(), _hipPeak = new Window();
        private int _next, _count, _captured, _resolved, _censored, _invalid, _gaps, _normalHipCandidates;
        private bool _pending, _sealed, _hasSource, _sourceWValid;
        private PelvisRunScope _traceScope;
        private Frame _command;
        private SourceStepContext _source;
        private Quaternion _sourceR;
        private Vector3 _sourceW;
        private float _sourceDt;
        private int _legCaptured, _legUnavailable, _legRawWValid;
        private int _quietFootLegMoving, _quietFootLegQuiet, _quietFootLegUnknown;

        internal void Reset(PelvisRunScope scope = PelvisRunScope.DynamicLegFreeSpace)
        {
            _traceScope = scope;
            ResetStepProgress();
            ResetHipWitness();
            ResetHipScope();
            _next = _count = _captured = _resolved = _censored = _invalid = _gaps = _normalHipCandidates = 0;
            _pending = _sealed = false;
            _peak.Reset(); _normalHip.Reset(); _hipPeak.Reset();
            // Source derivative belongs only to this diagnostic; trace does not touch controller histories.
            _hasSource = _sourceWValid = false;
            _sourceW = Vector3.zero; _sourceDt = 0f;
            _legCaptured = _legUnavailable = _legRawWValid = 0;
            _quietFootLegMoving = _quietFootLegQuiet = _quietFootLegUnknown = 0;
        }

        internal void ObserveSource(SourceStepContext source, Quaternion soleRotation)
        {
            // Called only on true Render sampling, never from Fixed/status/held packets.
            if (!source.Valid || !Q(soleRotation)) { _hasSource = _sourceWValid = false; return; }
            if (_hasSource && source.Epoch == _source.Epoch && source.Sequence == _source.Sequence) return;
            _sourceDt = _hasSource ? source.Time - _source.Time : 0f;
            _sourceWValid = _hasSource && source.Epoch == _source.Epoch && source.Sequence == _source.Sequence + 1 &&
                _sourceDt > 0.0001f && _sourceDt < 0.25f;
            _sourceW = _sourceWValid ? Log(soleRotation * Quaternion.Inverse(_sourceR)) / _sourceDt : Vector3.zero;
            _sourceWValid &= V(_sourceW);
            _source = source; _sourceR = soleRotation; _hasSource = true;
        }

        internal void Capture(bool trace, PelvisParentFrame parent, FootAnkleFoundationReport report,
            Rigidbody lower, Rigidbody foot, Quaternion rawSole, Quaternion sourceRelative,
            Vector3 sourceW, Vector3 sourceA, FootAnkleSourceDrivenRelativePdIsolation relative,
            FootAnkleAngularTransport common, LegTurnStepEvidence leg, FootAnkleTargetAccelerationPersistenceConditioner conditioner,
            FootRelativeReference reference = null, Quaternion rawSourceRelative = default,
            Vector3 rawSourceW = default, Vector3 rawSourceA = default)
        {
            if (!trace || _sealed || !parent.Valid) return;
            try
            {
                if (_pending) { CensorPending(); _gaps++; BreakContinuity(); }
                PhysicsStepContext step = report.RuntimeContext;
                if (!parent.Matches(step) || !parent.Finite() || lower == null || foot == null ||
                    lower.isKinematic || foot.isKinematic || !relative.DynamicActive ||
                    report.FootControllerLastCompletedFixedSequence != report.FixedUpdateSequence)
                { _invalid++; BreakContinuity(); return; }
                Frame f = new Frame
                {
                    Step = step, Fixed = report.FixedUpdateSequence, Render = report.TargetRenderSampleSequence,
                    Fresh = report.TargetRenderSampleFreshAtFixedStep, Held = report.FixedStepsSinceTargetRenderSample,
                    Age = report.TargetRenderSampleAgeSeconds, SourceDt = _sourceDt,
                    SourceWValid = _sourceWValid && _source.Epoch == step.Epoch && _source.Sequence == step.SourceSequence,
                    RawR = rawSole, RawW = _sourceW, SourceRelR = sourceRelative, SourceW = sourceW, SourceA = sourceA,
                    LowerR = lower.rotation, FootR = foot.rotation, LowerW = lower.angularVelocity, FootW = foot.angularVelocity,
                    PelvisR = parent.Rotation, ReferenceR = parent.CommandRotation, PelvisW = parent.W, ReferenceW = parent.CommandW,
                    ErrorLocal = relative.CurrentPoseErrorLocal, BodyRelW = relative.CurrentBodyRelativeWLocal,
                    P = relative.CurrentPoseErrorLocal * FootAnkleSourceDrivenRelativePdIsolation.Kp,
                    D = (relative.CurrentTargetRelativeWLocal - relative.CurrentBodyRelativeWLocal) * FootAnkleSourceDrivenRelativePdIsolation.Kd,
                    FF = relative.CurrentTargetAccelerationFeedforwardLocal,
                    RelativePredictedA = relative.CurrentAppliedRelativeAccelerationLocal, RelativeTau = relative.CurrentTorqueWorld,
                    RelativeAClamp = relative.CurrentAccelerationClamp, RelativeTauClamp = relative.CurrentTorqueClamp,
                    Stale = relative.CurrentStaleTargetW, CommonWarmup = common.CurrentWarmup, CommonActive = common.CurrentActive,
                    CommonMeasuredA = common.CurrentMeasuredLowerAngularAccelerationWorld,
                    CommonRequestedA = common.CurrentRequestedAngularAccelerationWorld, CommonTau = common.CurrentTorqueWorld,
                    CommonAClamp = common.CurrentAccelerationClamp, CommonTauClamp = common.CurrentTorqueClamp
                };
                f.ErrorWorld = f.LowerR * f.ErrorLocal;
                f.Error = f.ErrorWorld.magnitude;
                Vector3 rawError = Log(f.RawR * Quaternion.Inverse(f.FootR));
                f.RawError = rawError.magnitude;
                // Positive = body moving away along the current shortest-error axis. Not d(Euler yaw)/dt.
                f.RawOpening = f.RawError > 0.000001f ? Vector3.Dot(rawError / f.RawError, f.RawW - f.FootW) : 0f;
                f.RelativeOpening = f.Error > 0.000001f ? Vector3.Dot(f.ErrorLocal / f.Error,
                    relative.CurrentTargetRelativeWLocal - f.BodyRelW) : 0f;
                if (!Finite(f)) { _invalid++; BreakContinuity(); return; }
                // Optional diagnostic coverage has its own counters. Missing/stale leg evidence must
                // never discard an otherwise valid Foot frame or affect any existing quality/safety gate.
                f.LegValid = leg.Matches(step, f.Fixed) && leg.Source.Matches(report.RuntimeSourceContext);
                if (f.LegValid)
                {
                    f.Leg = leg; _legCaptured++;
                    if (leg.RawWorldWValid) _legRawWValid++;
                }
                else _legUnavailable++;
                if (f.SourceWValid && f.Age <= FootAnkleSourceDrivenRelativePdIsolation.MaxFreshTargetAgeSeconds &&
                    f.RawW.magnitude <= QuietW)
                {
                    if (!f.LegValid || !leg.RawWorldWValid) _quietFootLegUnknown++;
                    else if (leg.RawUpperW.magnitude > QuietW || leg.RawLowerW.magnitude > QuietW) _quietFootLegMoving++;
                    else _quietFootLegQuiet++;
                }
                // Optional witness never changes the original packet/leg qualification.
                f.CommandSource = report.RuntimeSourceContext;
                f.ReferenceSelected = reference != null && reference.Selected;
                f.ReferenceActive = reference != null && reference.Active;
                if (reference != null) f.Reference = reference.Current;
                f.RawSourceRelR = reference != null ? rawSourceRelative : sourceRelative;
                f.RawSourceW = reference != null ? rawSourceW : sourceW;
                f.RawSourceA = reference != null ? rawSourceA : sourceA;
                f.ReferenceWitnessValid = Q(f.RawSourceRelR) && V(f.RawSourceW) && V(f.RawSourceA) &&
                    (!f.ReferenceSelected || (f.ReferenceActive && f.Reference.Source.Matches(f.CommandSource)));
                if (f.ReferenceWitnessValid)
                {
                    f.RawLocalError = Log((f.LowerR * f.RawSourceRelR) * Quaternion.Inverse(f.FootR)).magnitude;
                    f.ReferenceGap = Log(f.RawSourceRelR * Quaternion.Inverse(f.SourceRelR)).magnitude;
                    f.ReferenceWitnessValid = F(f.RawLocalError) && F(f.ReferenceGap);
                }
                if (conditioner != null)
                {
                    f.ConditionedA = conditioner.CurrentConditionedTargetAccelerationLocal;
                    f.ConditionerStage = conditioner.CurrentStage;
                    f.ConditionerRawValid = conditioner.CurrentRawValid;
                    f.ConditionerHigh = conditioner.CurrentRawHigh;
                    f.ConditionerConfirmed = conditioner.CurrentConfirmed;
                }
                RecordConditionerWitness(f);
                RecordReferenceWitness(f);
                try { CaptureHipWitness(ref f); }
                catch { f.HipWitnessValid = false; _hipMissing++; }
                try { CaptureHipScope(ref f); }
                catch { f.HipScope = ScopeUnavailable; f.NormalPlanGapValid = false; _hipScopeFailures++; }
                _command = f; _pending = true; _captured++;
            }
            catch { _invalid++; CensorPending(); BreakContinuity(); } // Diagnostic failure never changes the plant.
        }

        internal void ObserveSettled(bool trace, PhysicsStepContext step, FootFixedStepEvidence observed,
            Rigidbody lower, Rigidbody foot, SourceStepContext nextSource, Quaternion nextSourceRelative, float nextAge,
            LegTurnStepEvidence nextLeg)
        {
            if (!trace || !_pending || _sealed) return;
            try
            {
                Frame f = _command;
                if (!step.Valid || step.Epoch != f.Step.Epoch || step.Sequence != f.Step.Sequence + 1 ||
                    observed.FixedSequence != f.Fixed + 1 || !observed.Valid || lower == null || foot == null ||
                    lower.isKinematic || foot.isKinematic)
                { CensorPending(); _gaps++; BreakContinuity(); return; }
                Quaternion nextLowerR = lower.rotation, nextFootR = foot.rotation;
                if (!Q(nextLowerR) || !Q(nextFootR))
                { CensorPending(); _gaps++; BreakContinuity(); return; }
                f.NextSequence = step.Sequence;
                f.NextLowerW = observed.LowerAngularVelocityWorld; f.NextFootW = observed.FootAngularVelocityWorld;
                f.MeasuredLowerA = (f.NextLowerW - f.LowerW) / f.Step.Dt;
                f.MeasuredFootA = (f.NextFootW - f.FootW) / f.Step.Dt;
                // Hold command N's local source pose; N+1 Render changes cannot masquerade as physical closure.
                f.LockedNextErrorWorld = Log((nextLowerR * f.SourceRelR) * Quaternion.Inverse(nextFootR));
                if (!V(f.MeasuredLowerA) || !V(f.MeasuredFootA) || !V(f.LockedNextErrorWorld))
                { CensorPending(); _invalid++; BreakContinuity(); return; }
                f.Resolved = true; _pending = false; _resolved++;
                try { f.Progress = ResolveStepProgress(f, step, nextSource, nextSourceRelative, nextAge, nextLowerR, nextFootR); }
                catch { f.Progress = new StepProgress(); _progressFailures++; }
                try { f.HipResponse = ResolveHipProgress(f, step, nextSource, nextLeg, observed.FixedSequence); }
                catch { f.HipResponse = new HipProgress(); _hipResponseFailures++; }
                Record(f);
            }
            catch { _invalid++; CensorPending(); BreakContinuity(); }
        }

        internal void Seal()
        {
            if (_sealed) return;
            CensorPending(); _sealed = true;
        }

        private void CensorPending()
        {
            if (!_pending) return;
            Frame f = _command; _pending = false; _censored++; Record(f);
        }

        private void BreakContinuity()
        {
            _count = _next = 0;
            // Preserve missing-tail counts; never fill an interrupted window with a different epoch/run.
            _peak.Collecting = _normalHip.Collecting = _hipPeak.Collecting = false;
        }

        private void Record(Frame f)
        {
            Append(_peak, f); Append(_normalHip, f); Append(_hipPeak, f);
            // Both arms retain the worst RAW-animation local error. A smoother control goal must
            // not move the only peak window away from the largest remaining animation mismatch.
            float peakError = f.ReferenceWitnessValid ? f.RawLocalError : f.Error;
            if (_peak.Count == 0 || peakError > _peak.Score + 0.000001f) Start(_peak, f, peakError);
            // Reuse the old quiet-away window, not a fourth buffer. Both arms qualify through
            // the actual Hip FF owner. B's optional input-to-plan gap can also select a large lag.
            if (f.HipScope == ScopeNormal && f.HipWitnessValid)
            {
                _normalHipCandidates++;
                float score = Mathf.Max(f.HipRawLocalError, f.NormalPlanGapValid ? f.NormalPlanGap : 0f);
                if (_normalHip.Count == 0 || score > _normalHip.Score + 0.000001f) Start(_normalHip, f, score);
            }
            // Reuse the third window for final world-control Hip error. Raw-local and raw-world
            // comparisons remain in every frame and all-run summaries; the other two windows are independent.
            if (f.HipWitnessValid && (_hipPeak.Count == 0 || f.HipControlError > _hipPeak.Score + 0.000001f))
                Start(_hipPeak, f, f.HipControlError);
            _ring[_next] = f; _next = (_next + 1) % Before; _count = Math.Min(_count + 1, Before);
        }

        private static void Append(Window window, Frame f)
        {
            if (!window.Collecting || window.Remaining <= 0 || window.Count >= Capacity) return;
            window.Frames[window.Count++] = f; window.Remaining--;
        }

        private void Start(Window window, Frame f, float score)
        {
            window.Reset(); window.Score = score; window.TriggerSequence = f.Step.Sequence;
            for (int i = 0; i < _count; i++) window.Frames[window.Count++] = _ring[(_next - _count + i + Before) % Before];
            window.Frames[window.Count++] = f; window.Remaining = After; window.Collecting = true;
        }

        internal string GetDetails()
        {
            var sb = new StringBuilder(8192);
            sb.Append("FOOT_TURN_V04650/readOnly=True/scope=").Append(PelvisRunScopes.HasUpperWeight(_traceScope) ? "DYNAMIC_PARENT_UPPER_SELF_WEIGHT_CONTACT" : PelvisRunScopes.HasKneeCoupling(_traceScope) ? "DYNAMIC_PARENT_KNEE_NORMAL_COUPLING_CONTACT" : _traceScope == PelvisRunScope.DynamicLegLowerSelfWeightContact ? "DYNAMIC_PARENT_LOWER_SELF_WEIGHT_CONTACT" : _traceScope == PelvisRunScope.DynamicLegFootSelfWeightContact ? "DYNAMIC_PARENT_FOOT_SELF_WEIGHT_CONTACT" : _traceScope == PelvisRunScope.DynamicLegUnloadedContact ? "DYNAMIC_PARENT_UNLOADED_CONTACT" : "DYNAMIC_PARENT_FREE_SPACE").Append("/sealed=").Append(_sealed)
                .Append("/captured-resolved-censored-pending=").Append(_captured).Append('/').Append(_resolved).Append('/').Append(_censored).Append('/').Append(_pending)
                .Append("/invalid-gaps=").Append(_invalid).Append('/').Append(_gaps)
                .Append("/normalHipCandidates=").Append(_normalHipCandidates)
                .Append("/window=16pre+trigger+48post/angles=rotvecRad/W=radPerSec/A=radPerSec2/tau=Nm")
                .AppendLine("/openingPositive=AWAY/worldY=projection_NOT_EULER_YAW/response=TOTAL_PLANT_NOT_SINGLE_TORQUE_CAUSALITY");
            sb.Append("LEG_CHAIN_V04680/readOnly=True/captured-unavailable-rawWorldWValid=")
                .Append(_legCaptured).Append('/').Append(_legUnavailable).Append('/').Append(_legRawWValid)
                .Append("/quietFoot(legSourceMoving-legSourceQuiet-unknown)=").Append(_quietFootLegMoving)
                .Append('/').Append(_quietFootLegQuiet).Append('/').Append(_quietFootLegUnknown)
                .AppendLine("/rawW=EXISTING_RENDER_DERIVATIVE_BEFORE_CLAMP_BLEND/quiet=FOOT_W_LE0.05_FRESH_OR_HELD_LE120MS")
                .AppendLine("legUnits=ROTATION_QUATERNION_WORLD_W_RADSEC_ALPHA_RADSEC2/body=SETTLED_PRE_N/commands=FOR_N_NOT_MEASURED_ALPHA/parentA=PRE_CAP_CONTRIBUTION/NA=UNAVAILABLE_NOT_ZERO/hipTargetW=ACTUAL_D_TARGET_NOT_AFF_SOURCE/hipFinalA=ACTUATOR_ALPHA_REQUEST_NOT_MEASURED_RESPONSE/modes(bits)=1:FRAME,2:SOURCE_ABSORBER,4:HIP_GOVERNOR,8:COMPOSITE_ABSORBER,16:PHYSICAL_RECOVERY,32:RELATIVE_AFF,64:PARENT_TRANSPORT,128:HIP_CLAMP");
            AppendStepProgressSummary(sb);
            AppendReferenceSummary(sb);
            AppendHipSummary(sb);
            AppendHipScopeSummary(sb);
            sb.AppendLine("NORMAL_HIP_WINDOW_V04790/replaces=QUIET_RAW_WORLD_AWAY/triggerScope=NORMAL_WITH_VALID_HIP/scoreRad=MAX_RAW_LOCAL_BODY_ERROR_AND_VALID_B_INPUT_TO_PLAN_GAP/eachIndependentPeakMayDiffer=True/prePostMayCrossScopes=True/windowRowsNotRunHistogram=True/empty=NO_ELIGIBLE_NORMAL_HIP_EVIDENCE");
            sb.AppendLine("HIP_FF_GATE_V04770/readOnly=True/reason=ACTUAL_FF_OWNER_FIRST_MATCH/stateBits=1:NO_TORSO,2:SOURCE_TURN,4:SELECTED_TORSO_RECOVERY,8:HIP_RECOVERY,16:KNEE_RECOVERY,32:CONTROL_FRAME,64:HIP_UNHEALTHY,128:KNEE_UNHEALTHY,256:HIP_REFERENCE_GOAL_BRIDGE/healthBits=CONTEXT_NOT_EXTRA_FF_GATES/inputA=COHERENT_NORMAL_WORLD_A_WHEN_APPLIED_ELSE_RENDER_RELATIVE_A_IN_CONTROL_PARENT_WORLD_BASIS/selectedA=BEFORE_FINAL_HIP_SUM_AND_CAP_NOT_NET_BODY_A/staleThreshold=MAX_0_MAXAGE_PLUS_0.0001_SECONDS/NA=OPTIONAL_NOT_FOOT_OR_HIP_QUALIFICATION");
            sb.AppendLine("HIP_NORMAL_V04780/poseW=ONE_SHARED_FIXED_TRAJECTORY/normalFF=WORLD_PLANNER_A_NO_EXTRA_PARENT_A/limits=W_HIP40_OVER_KD12_A_FF20/notHumanLimits=True/NA=BASELINE_OR_OPTIONAL_ONLY/noNewRing=True");
            sb.AppendLine("KNEE_FF_GATE_V04770/readOnly=True/mode32=HIP_FF_ACTIVE_ONLY/kneeReason=ACTUAL_KNEE_LANE_FIRST_MATCH/input-selected=BEFORE_ORIGINAL_KNEE_ALLOCATION/allocated=AFTER_OPPOSING_SUPPRESSION_BEFORE_FINAL_CLAMP/finalFlexion=ORIGINAL_SCALAR_COMMAND_NOT_NATIVE_RESPONSE/normalKneeCap=40/goalScope=HIP_ONLY_KEEP_KNEE_IN_BOTH_CURRENT_DYNAMIC_ARMS");
            FormatWindow(sb, "PEAK_RAW_LOCAL_ERROR_V04720", _peak);
            FormatWindow(sb, "PEAK_NORMAL_HIP_RAW_OR_PLAN_GAP_V04790", _normalHip);
            FormatWindow(sb, "PEAK_HIP_CONTROL_WORLD_ERROR_V04740", _hipPeak);
            return sb.ToString();
        }

        private static void FormatWindow(StringBuilder sb, string name, Window w)
        {
            sb.Append(name).Append(" trigger=").Append(w.TriggerSequence).Append(" score=").Append(S(w.Score))
                .Append(" count=").Append(w.Count).Append(" postMissing=").Append(w.Remaining).AppendLine();
            for (int i = 0; i < w.Count; i++)
            {
                Frame f = w.Frames[i];
                sb.Append("turnFrame=").Append(f.Step.Epoch).Append(':').Append(f.Step.Sequence).Append('/').Append(f.Fixed)
                    .Append(" t/dt=").Append(S(f.Step.FixedTime)).Append('/').Append(S(f.Step.Dt))
                    .Append(" source/runtime-render-fresh-held-age-dt-Wvalid=").Append(f.Step.SourceSequence).Append('/').Append(f.Render)
                    .Append('/').Append(f.Fresh).Append('/').Append(f.Held).Append('/').Append(S(f.Age)).Append('/').Append(S(f.SourceDt)).Append('/').Append(f.SourceWValid)
                    .Append(" R(raw-rel-lower-foot-pelvis-reference)=").Append(R(f.RawR)).Append('/').Append(R(f.SourceRelR)).Append('/').Append(R(f.LowerR)).Append('/').Append(R(f.FootR)).Append('/').Append(R(f.PelvisR)).Append('/').Append(R(f.ReferenceR))
                    .Append(" Wworld(raw-lower-foot-pelvis-reference)=").Append(T(f.RawW)).Append('/').Append(T(f.LowerW)).Append('/').Append(T(f.FootW)).Append('/').Append(T(f.PelvisW)).Append('/').Append(T(f.ReferenceW))
                    .Append(" local(sourceW-sourceA-bodyRelW-E)=").Append(T(f.SourceW)).Append('/').Append(T(f.SourceA)).Append('/').Append(T(f.BodyRelW)).Append('/').Append(T(f.ErrorLocal))
                    .Append(" Eworld/angle/rawAngle/openingRelRaw=").Append(T(f.ErrorWorld)).Append('/').Append(S(f.Error)).Append('/').Append(S(f.RawError)).Append('/').Append(S(f.RelativeOpening)).Append(',').Append(S(f.RawOpening))
                    .Append(" cmdLocal(P-D-FF-predA)=").Append(T(f.P)).Append('/').Append(T(f.D)).Append('/').Append(T(f.FF)).Append('/').Append(T(f.RelativePredictedA))
                    .Append(" commonWorld(measuredA-requestA)=").Append(T(f.CommonMeasuredA)).Append('/').Append(T(f.CommonRequestedA))
                    .Append(" tauWorld(rel-common)=").Append(T(f.RelativeTau)).Append('/').Append(T(f.CommonTau))
                    .Append(" stale/warmup/active/clamps(relA-relTau-commonA-commonTau)=").Append(f.Stale).Append('/').Append(f.CommonWarmup).Append('/').Append(f.CommonActive)
                    .Append('/').Append(f.RelativeAClamp).Append('/').Append(f.RelativeTauClamp).Append('/').Append(f.CommonAClamp).Append('/').Append(f.CommonTauClamp)
                    .Append(" next=").Append(f.Resolved ? f.NextSequence.ToString(CultureInfo.InvariantCulture) : "CENSORED");
                if (f.Resolved)
                    sb.Append(" settled(Wlower-Wfoot-Alower-Afoot-lockedEworld)=").Append(T(f.NextLowerW)).Append('/').Append(T(f.NextFootW))
                        .Append('/').Append(T(f.MeasuredLowerA)).Append('/').Append(T(f.MeasuredFootA)).Append('/').Append(T(f.LockedNextErrorWorld));
                AppendLeg(sb, f);
                AppendStepProgress(sb, f);
                AppendReferenceWitness(sb, f);
                AppendHipWitness(sb, f);
                AppendHipFeedforwardWitness(sb, f);
                AppendKneeFeedforwardWitness(sb, f);
                AppendHipNormalReferenceWitness(sb, f);
                AppendHipScopeWitness(sb, f);
                sb.AppendLine();
            }
        }

        private static void AppendLeg(StringBuilder sb, Frame f)
        {
            if (!f.LegValid) { sb.Append(" leg=NA_CONTEXT_OR_COMMAND_UNAVAILABLE"); return; }
            LegTurnStepEvidence l = f.Leg;
            sb.Append(" leg(epoch-runtime-local-source-modes-maxA)=").Append(l.Step.Epoch).Append('/').Append(l.Step.Sequence)
                .Append('/').Append(l.Fixed).Append('/').Append(l.Source.Sequence).Append('/').Append(l.Modes).Append('/').Append(S(l.HipMaxA))
                .Append(" legRawW(upper-lower-dt)=");
            if (l.RawWorldWValid) sb.Append(T(l.RawUpperW)).Append('/').Append(T(l.RawLowerW)).Append('/').Append(S(l.RawSourceDt));
            else sb.Append("NA");
            sb.Append(" legR(rawUpper-rawLower-upper-hipTarget)=").Append(R(l.RawUpperR)).Append('/').Append(R(l.RawLowerR))
                .Append('/').Append(R(l.UpperR)).Append('/').Append(R(l.HipTargetR))
                .Append(" legW(upper-hipParent-hipTarget-kneeTargetRel)=").Append(T(l.UpperW)).Append('/').Append(T(l.HipParentW))
                .Append('/').Append(T(l.HipTargetW)).Append('/').Append(T(l.KneeTargetRelW))
                .Append(" legA(hipFinal-hipParentPreCap-kneeRelative-kneeTotal)=").Append(T(l.HipFinalA)).Append('/').Append(T(l.HipParentPreCapA))
                .Append('/').Append(T(l.KneeRelativeA)).Append('/').Append(T(l.KneeTotalA));

        }

        private static bool Finite(Frame f) => f.Step.Valid && Q(f.RawR) && Q(f.SourceRelR) && Q(f.LowerR) && Q(f.FootR) &&
            V(f.RawW) && V(f.SourceW) && V(f.SourceA) && V(f.LowerW) && V(f.FootW) && V(f.BodyRelW) && V(f.ErrorLocal) &&
            V(f.P) && V(f.D) && V(f.FF) && V(f.RelativePredictedA) && V(f.RelativeTau) && V(f.CommonMeasuredA) &&
            V(f.CommonRequestedA) && V(f.CommonTau) && F(f.Age) && f.Age >= 0f && F(f.SourceDt) &&
            F(f.Error) && F(f.RawError) && F(f.RawOpening) && F(f.RelativeOpening);

        // Shortest rotation vector, robust for tiny angles; quaternion sign does not change the result.
        private static Vector3 Log(Quaternion q)
        {
            float n = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!F(n) || n < 0.000001f) return new Vector3(float.NaN, float.NaN, float.NaN);
            float sign = q.w < 0f ? -1f : 1f;
            Vector3 v = new Vector3(q.x, q.y, q.z) * (sign / n);
            float m = v.magnitude;
            return m < 0.000001f ? v * 2f : v * (2f * Mathf.Atan2(m, q.w * sign / n) / m);
        }
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) && q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.000001f;
        private static string S(float v) => v.ToString("R", CultureInfo.InvariantCulture);
        private static string T(Vector3 v) => "(" + S(v.x) + "," + S(v.y) + "," + S(v.z) + ")";
        private static string R(Quaternion q) => "(" + S(q.x) + "," + S(q.y) + "," + S(q.z) + "," + S(q.w) + ")";
    }
}
