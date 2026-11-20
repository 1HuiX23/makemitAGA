using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Reuse the existing turn recorder's command N -> pre-lifecycle N+1 join.
    // Bounded value witnesses/totals only (V141 adds one geometry mismatch witness).
    // No clock, ring or native read of its own.
    internal sealed partial class LandingPreparationResponse
    {
        internal struct Sample
        {
            internal LandingPreparationCommand Command;
            internal bool Valid;
            internal float BeforeMin, AfterMin, BeforeGap, AfterGap;
            internal bool BeforeContact, AfterContact;
            internal LandingPreparationGeometry.Sample Geometry;
        }
        private int _commands, _resolved, _censored, _unavailable, _faults, _hip, _knee, _approach;
        private bool _hasWorst, _hasApproach;
        private Sample _worst, _worstApproach;
        internal void Reset() { _commands = _resolved = _censored = _unavailable = _faults = _hip = _knee = _approach = 0;
            _hasWorst = _hasApproach = false; _worst = _worstApproach = default; ResetGeometry(); }
        internal void Fault() { _faults++; }
        internal static Sample Measure(LandingPreparationCommand c, FootFixedStepEvidence before,
            Quaternion beforeR, PhysicsStepContext next, FootFixedStepEvidence after, Quaternion afterR, LandingPreparationGeometry.Sample geometry = default)
        {
            var r = new Sample { Command = c, Geometry = geometry };
            if (!c.Applied || !c.Step.Valid || !c.Source.Valid || c.Step.Epoch != c.Source.Epoch || c.Step.SourceSequence != c.Source.Sequence ||
                !next.Valid || next.Epoch != c.Step.Epoch || next.Sequence != c.Step.Sequence + 1 ||
                !before.Valid || !after.Valid || after.FixedSequence != before.FixedSequence + 1 ||
                !before.PreparationPoseValid || !after.PreparationPoseValid || c.SupportId == 0 ||
                before.PreparationSupportId != c.SupportId || after.PreparationSupportId != c.SupportId ||
                !PelvisParentFrame.V(before.FootAnchorWorld) || !PelvisParentFrame.V(after.FootAnchorWorld) ||
                !PelvisParentFrame.Q(beforeR) || !PelvisParentFrame.Q(afterR) ||
                !PelvisParentFrame.V(c.Center) || !PelvisParentFrame.V(c.Half) || !PelvisParentFrame.V(c.FloorCenter) || !PelvisParentFrame.V(c.FloorHalf) ||
                c.Half.x <= 0f || c.Half.y <= 0f || c.Half.z <= 0f || c.FloorHalf.x <= 0f || c.FloorHalf.y <= 0f || c.FloorHalf.z <= 0f ||
                !FootBoxGeometry.Inside(before.FootAnchorWorld, beforeR, c.Center, c.Half, c.FloorCenter, c.FloorHalf) ||
                !FootBoxGeometry.Inside(after.FootAnchorWorld, afterR, c.Center, c.Half, c.FloorCenter, c.FloorHalf)) return r;
            float top = c.FloorCenter.y + c.FloorHalf.y;
            r.BeforeMin = FootBoxGeometry.Minimum(before.FootAnchorWorld, beforeR, c.Center, c.Half) - top;
            r.AfterMin = FootBoxGeometry.Minimum(after.FootAnchorWorld, afterR, c.Center, c.Half) - top;
            r.BeforeGap = before.AnchorErrorMeters; r.AfterGap = after.AnchorErrorMeters;
            r.BeforeContact = before.ContactActiveBeforeLifecycle; r.AfterContact = after.ContactActiveBeforeLifecycle;
            r.Valid = PelvisParentFrame.F(r.BeforeMin) && PelvisParentFrame.F(r.AfterMin) &&
                PelvisParentFrame.F(r.BeforeGap) && PelvisParentFrame.F(r.AfterGap);
            // Optional decomposition must belong to PRE_N, never a newer source
            // or the next step. Its loss does not censor an otherwise valid response.
            if (!r.Valid || !geometry.Valid || !geometry.Step.Matches(c.Step) || !geometry.Source.Matches(c.Source) ||
                geometry.Fixed != before.FixedSequence || Mathf.Abs(geometry.Actual - r.BeforeMin) > .000005f)
                r.Geometry = default;
            return r;
        }
        internal void Observe(LandingPreparationCommand command, bool resolved, Sample sample)
        {
            if (!command.Applied) return;
            _commands++; if (command.HipApplied) _hip++; if (command.KneeApplied) _knee++; if (command.Approach) _approach++;
            if (!resolved) { _censored++; return; }
            if (!sample.Valid || !sample.Command.Step.Matches(command.Step) || !sample.Command.Source.Matches(command.Source) ||
                sample.Command.SupportId != command.SupportId || sample.Command.HipApplied != command.HipApplied ||
                sample.Command.KneeApplied != command.KneeApplied || sample.Command.Approach != command.Approach) { _unavailable++; return; }
            _resolved++; ObserveGeometry(command, sample);
            if (!_hasWorst || sample.AfterMin < _worst.AfterMin) { _hasWorst = true; _worst = sample; }
            if (command.Approach && (!_hasApproach || sample.AfterMin < _worstApproach.AfterMin)) { _hasApproach = true; _worstApproach = sample; }
        }
        internal string Summary() => "LANDING_PREPARATION_RESPONSE_V041400/commands-resolved-censored-unavailable=" + _commands + "|" + _resolved + "|" + _censored + "|" + _unavailable +
            "/hip-knee-approach=" + _hip + "|" + _knee + "|" + _approach + "/faults=" + _faults +
            "/domain=ACTUAL_FOOT_BOX_N_TO_SETTLED_N_PLUS_1_SAME_COMMAND_PLANE/causalIsolation=False/contactFlag=COLLIDER_AUTHORITY_NOT_CONFIRMED_LOAD" +
            (_hasWorst ? Text("LOWEST_AFTER", _worst) : "/witness=NONE") + (_hasApproach ? Text("APPROACH_LOWEST_AFTER", _worstApproach) : "/approachWitness=NONE") + GeometrySummary();
        private static string Text(string name, Sample s) => "\nLANDING_RESPONSE_WITNESS_V041400/selection=" + name + "/epoch-fixed-source=" + s.Command.Step.Epoch + "|" + s.Command.Step.Sequence + "|" + s.Command.Source.Sequence +
            "/support=" + s.Command.SupportId + "/applied(hip-knee-approach)=" + s.Command.HipApplied + "|" + s.Command.KneeApplied + "|" + s.Command.Approach +
            "/referenceGainMm(hip-knee)=" + N(s.Command.HipGain * 1000f) + "|" + N(s.Command.KneeGain * 1000f) +
            "/actualBoxMinMm(before-after)=" + N(s.BeforeMin * 1000f) + "|" + N(s.AfterMin * 1000f) +
            "/ankleGapMm(before-after)=" + N(s.BeforeGap * 1000f) + "|" + N(s.AfterGap * 1000f) +
            "/contactAuthority(before-after)=" + s.BeforeContact + "|" + s.AfterContact + "\n" + LandingPreparationGeometry.Text(s.Command, s.Geometry);
        private static string N(float v) => FootGroundTrajectory.N(v);
    }
}
