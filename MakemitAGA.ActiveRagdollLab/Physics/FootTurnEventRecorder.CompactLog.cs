using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootTurnEventRecorder
    {
        // Current Hip runs keep the same selected frames and all-run statistics.
        // Old planner internals, repeated poses and algebraic expansions are not
        // needed in every downstream row. Other scopes retain the detailed format.
        // Deduplication happens only at formatting: references are local to this
        // side/trace and always point backwards to a complete row in an earlier window.
        private void AppendCompactWindows(StringBuilder sb)
        {
            sb.AppendLine("turnLogFormat=" + RunLogPolicy.HipFormat + "/rows=NUMERIC_COMMAND_AND_RESPONSE/turnFrameRef=SAME_SIDE_PRIOR_WINDOW_EPOCH_SEQUENCE_LOCAL_FIXED/angles=RAD/W=RAD_PER_SEC/A=RAD_PER_SEC2/tau=NM/NA=UNAVAILABLE_NOT_ZERO/selectedWindowsUnchanged=True");
            sb.AppendLine("turnLogFields=SOURCE_CLOCK_RAW_CONTROL_ERRORS_SCOPE_GATES_CLAMPS_ACTUAL_COMMANDS_NEXT_RESPONSE/omitted=REPEATED_QUATERNIONS_PLANNER_INTERNALS_ALGEBRAIC_EXPANSIONS/fullHipGoalPath=HIP_POINT_RESPONSE_FRAMES/summaryDomain=ALL_CAPTURED_FIXED_STEPS");
            FormatCompactWindow(sb, "PEAK_RAW_LOCAL_ERROR_V04720", _peak, null, null);
            FormatCompactWindow(sb, "PEAK_NORMAL_HIP_RAW_OR_PLAN_GAP_V04790", _normalHip, _peak, null);
            FormatCompactWindow(sb, "PEAK_HIP_CONTROL_WORLD_ERROR_V04740", _hipPeak, _peak, _normalHip);
        }

        private static bool WindowContains(Window w, Frame f)
        {
            if (w == null) return false;
            for (int i = 0; i < w.Count; i++)
                if (w.Frames[i].Fixed == f.Fixed && w.Frames[i].Step.Matches(f.Step)) return true;
            return false;
        }

        private static void FormatCompactWindow(StringBuilder sb, string name, Window w, Window earlierA, Window earlierB)
        {
            sb.Append(name).Append(" trigger=").Append(w.TriggerSequence).Append(" score=").Append(S(w.Score))
                .Append(" count=").Append(w.Count).Append(" postMissing=").Append(w.Remaining).AppendLine();
            for (int i = 0; i < w.Count; i++)
            {
                Frame f = w.Frames[i];
                if (WindowContains(earlierA, f) || WindowContains(earlierB, f))
                {
                    sb.Append("turnFrameRef=").Append(f.Step.Epoch).Append(':').Append(f.Step.Sequence).Append('/').Append(f.Fixed).AppendLine();
                }
                else AppendCompactFrame(sb, f);
            }
        }

        private static void AppendCompactFrame(StringBuilder sb, Frame f)
        {
            sb.Append("turnFrame=").Append(f.Step.Epoch).Append(':').Append(f.Step.Sequence).Append('/').Append(f.Fixed)
                .Append(" t/dt=").Append(S(f.Step.FixedTime)).Append('/').Append(S(f.Step.Dt))
                .Append(" source(runtime-render-fresh-held-age-dt-Wvalid)=").Append(f.Step.SourceSequence).Append('/').Append(f.Render)
                .Append('/').Append(f.Fresh).Append('/').Append(f.Held).Append('/').Append(S(f.Age)).Append('/').Append(S(f.SourceDt)).Append('/').Append(f.SourceWValid)
                .Append(" footRad(rawLocal-control-rawWorld-goalGap)=").Append(f.ReferenceWitnessValid ? S(f.RawLocalError) : "NA")
                .Append('/').Append(S(f.Error)).Append('/').Append(S(f.RawError)).Append('/').Append(f.ReferenceWitnessValid ? S(f.ReferenceGap) : "NA")
                .Append(" footState(referenceSelected-active-stale-commonWarmup-active-clampsRelA-Tau-commonA-Tau)=")
                .Append(f.ReferenceSelected).Append('/').Append(f.ReferenceActive).Append('/').Append(f.Stale).Append('/').Append(f.CommonWarmup).Append('/').Append(f.CommonActive)
                .Append('/').Append(f.RelativeAClamp).Append('/').Append(f.RelativeTauClamp).Append('/').Append(f.CommonAClamp).Append('/').Append(f.CommonTauClamp)
                .Append(" footTau(rel-common)=").Append(T(f.RelativeTau)).Append('/').Append(T(f.CommonTau))
                .Append(" footW(raw-lower-foot)=").Append(f.SourceWValid ? T(f.RawW) : "NA").Append('/').Append(T(f.LowerW)).Append('/').Append(T(f.FootW))
                .Append(" next=").Append(f.Resolved ? f.NextSequence.ToString() : "CENSORED");
            StepProgress p = f.Progress;
            sb.Append(" footNextSource(epoch-seq-time-dt-age)=");
            if (f.Resolved && p.ReferenceValid)
                sb.Append(p.NextSource.Epoch).Append('/').Append(p.NextSource.Sequence).Append('/').Append(S(p.NextSource.Time))
                    .Append('/').Append(S(p.NextSource.Dt)).Append('/').Append(S(p.NextAge));
            else sb.Append("NA");
            sb.Append(" footStepRad(fresh-locked-next-physical-source-net-closure)=");
            if (f.Resolved && p.ReferenceValid)
                sb.Append(p.FreshPair).Append('/').Append(S(p.LockedError)).Append('/').Append(S(p.NextError)).Append('/')
                    .Append(S(p.PhysicalDelta)).Append('/').Append(S(p.SourceDelta)).Append('/').Append(S(p.NetDelta)).Append('/').Append(S(p.Closure));
            else sb.Append("NA");
            sb.Append(" actualA(lower-foot)=").Append(f.Resolved ? T(f.MeasuredLowerA) + "/" + T(f.MeasuredFootA) : "NA");
            AppendCompactHip(sb, f);
            sb.AppendLine();
        }

        private static void AppendCompactHip(StringBuilder sb, Frame f)
        {
            AppendHipScopeWitness(sb, f);
            sb.Append(" hipRad(rawLocal-controlWorld-rawWorld-goalGap)=").Append(f.HipWitnessValid ?
                S(f.HipRawLocalError) + "/" + S(f.HipControlError) + "/" + S(f.HipRawWorldError) + "/" + S(f.HipTargetGap) : "NA");
            HipProgress p = f.HipResponse;
            sb.Append(" hipStepRad(locked-next-physical-target-net-closure)=").Append(f.Resolved && p.Valid ?
                S(p.LockedError) + "/" + S(p.NextError) + "/" + S(p.PhysicalDelta) + "/" + S(p.TargetDelta) + "/" + S(p.NetDelta) + "/" + S(p.Closure) : "NA")
                .Append(" actualUpperA=").Append(f.Resolved && p.Valid ? T(p.ActualUpperA) : "NA");
            if (!f.LegValid) { sb.Append(" leg=NA_CONTEXT_OR_COMMAND_UNAVAILABLE"); return; }
            var l = f.Leg;
            sb.Append(" legClock(epoch-seq-local-source-modes)=").Append(l.Step.Epoch).Append('/').Append(l.Step.Sequence).Append('/').Append(l.Fixed)
                .Append('/').Append(l.Source.Sequence).Append('/').Append(l.Modes)
                .Append(" legRawW(upper-lower)=").Append(l.RawWorldWValid ? T(l.RawUpperW) + "/" + T(l.RawLowerW) : "NA")
                .Append(" legCommandA(hipActuator-kneeTotal)=").Append(T(l.HipFinalA)).Append('/').Append(T(l.KneeTotalA))
                .Append(" hipFF(active-clamped-bits-reason)=").Append(l.HipFeedforwardValid ?
                    l.HipFeedforwardActive + "/" + l.HipFeedforwardClamped + "/" + l.HipFeedforwardStateBits + "/" + (l.HipFeedforwardReason ?? "NA") : "NA")
                .Append(" kneeFF(active-clamped-reason)=").Append(l.KneeFeedforwardValid && l.KneeCommitted ?
                    l.KneeFeedforwardActive + "/" + l.KneeFeedforwardClamped + "/" + (l.KneeFeedforwardReason ?? "NA") : "NA");
            var n = l.HipNormalReference;
            sb.Append(" hipNormal(applied-alphaLimited-speedLimited-handoff-reason)=").Append(n.Matches(l.Step) ?
                n.Applied + "/" + n.AlphaLimited + "/" + n.SpeedLimited + "/" + l.HipNormalRecoveryHandoffActive + "/" + (n.Reason ?? "NA") : "NA");
            // Existing fixed windows only: enough to separate reference feedback
            // from actual physical response without restoring verbose pose dumps.
            sb.Append(" hipNormalInputV041080(receiverParent-inputW-plannedA)=").Append(n.Matches(l.Step) ?
                n.ReceiverParentReference + "/" + T(n.InputW) + "/" + T(n.A) : "NA");
        }
    }
}
