using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using Lead = MakemitAGA.ActiveRagdollLab.Physics.LeftFootLeadSequence;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LeftFootReleaseForecast;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class LeftFootReleaseTrace
    {
        private sealed class LeadEvent
        {
            internal int Origin;
            internal Lead.Step Last;
            internal string End;
        }

        // All prospective windows from one release share the same real landing.
        private sealed class ContactLink
        {
            internal int Upper, Lower, Foot, FormalLocal;
            internal string Formal, MissOutcome;
        }
        private int _leadRequests, _leadStarted, _leadVertical, _leadOrdinary, _leadRecovery;
        private readonly int[] _leadPaired = new int[Lead.Length];
        internal bool WatchingReleaseContact => _open?.Contact != null;
        private bool FollowingLead => _open?.Lead != null && _open.Lead.End == null;
        private void WatchContact(in Foot.Frame f)
        {
            _open.Contact ??= new ContactLink { Upper = f.UpperId, Lower = f.LowerId, Foot = f.FootId };
        }

        private void IssueLead(Row r)
        {
            var e = _open.Lead;
            if (e != null && e.End != null) return;
            if (e == null && !Lead.Requests(r.Before,r.Corrected,r.Shin)) return;
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LeftFootLead);
            if (e == null) _leadRequests++;
            var s = Lead.Predict(r.From.Local,r.From.Gait,r.Before,r.Corrected,r.Shin,r.Motion,e?.Last);
            r.Lead = s;
            if (!s.Issued)
            {
                if (e != null) e.End = s.Decision.ToString();
                return;
            }
            if (e == null)
            {
                WatchContact(r.Before);
                _open.Lead = e = new LeadEvent { Origin = s.Origin };
                _leadStarted++;
            }
            e.Last = s;
            if (s.Vertical == Foot.Decision.Trial) _leadVertical++;
        }

        private void ReadLead(Row r, in Foot.Frame next)
        {
            if (!FollowingLead) return;
            using var cost = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.LeftFootLead);
            var e = _open.Lead;
            if (r.Lead == null || r.Lead != e.Last || !r.ShinPaired || next.UpperId != _open.Contact.Upper)
            {
                e.End = "MissingAdjacentReceipt";
                return;
            }
            if (!next.FivePoint) { e.End = "ReadFloorWitness"; return; }
            Lead.Readback(r.Lead,next);
            if (!r.Lead.Read) { e.End = "MissingPose"; return; }
            _leadPaired[r.Lead.Depth-1]++;
            if (!r.Lead.Inside) e.End = "ReadFootprint";
            else if (r.Lead.ConditionedGap >= r.Motion.Tether.HardGap) e.End = "ReadHardGap";
            else if (r.Lead.Depth == Lead.Length) e.End = "ThreeSteps";
        }

        private void EndLead(string reason)
        {
            if (FollowingLead) _open.Lead.End = reason;
        }

        private void ContactOutcome(in Tick t)
        {
            if (!WatchingReleaseContact) return;
            var e = _open.Contact;
            bool sameFormal = t.Active && t.EventSupport == t.Support && t.FormalFoot == e.Foot &&
                t.FormalLower == e.Lower && t.FormalUpper == e.Upper;
            if (_open.Landing == "FirstMiss" && e.MissOutcome == null)
            {
                if (t.Recovery && sameFormal) e.MissOutcome = "Recovered";
                else if (t.Recovery) e.MissOutcome = "UnverifiedRecovery";
                else if (!t.MissInProgress || t.Miss) e.MissOutcome = "HoldEnded";
            }
            if (e.Formal != null || (!t.Ordinary && !t.Recovery)) return;
            if (!sameFormal) { e.Formal = "UnverifiedFormal"; return; }
            e.FormalLocal = t.Local;
            e.Formal = t.Recovery ? "Recovery" : "Ordinary";
            if (_open.Lead != null)
            {
                if (t.Recovery) _leadRecovery++; else _leadOrdinary++;
            }
        }

        private void AppendLeadSummary(StringBuilder b)
        {
            b.Append("\nLEFT_FOOT_LEAD_V041660/requests-started=").Append(_leadRequests).Append('|').Append(_leadStarted)
                .Append("/pairedAt1-2-3=").Append(_leadPaired[0]).Append('|').Append(_leadPaired[1]).Append('|').Append(_leadPaired[2])
                .Append("/verticalTrials=").Append(_leadVertical).Append("/actualFormalOrdinary-recovery=")
                .Append(_leadOrdinary).Append('|').Append(_leadRecovery).Append("/plantWrites=0");
        }

        private static void AppendLeadEvent(StringBuilder b, Release r)
        {
            var e = r.Lead;
            if (e == null) return;
            b.Append("\nLEFT_FOOT_LEAD_EVENT_V041660/epoch-support-release-origin=").Append(r.Origin.Step.Epoch).Append('|')
                .Append(r.Origin.ReleasedSupport).Append('|').Append(r.Origin.Local).Append('|').Append(e.Origin)
                .Append("/sequenceEnd=").Append(e.End ?? "Pending");
            AppendActualContact(b,r);
        }

        private static void AppendActualContact(StringBuilder b, Release r)
        {
            var e = r.Contact;
            b.Append("/firstLanding-local=").Append(r.Landing ?? "Pending").Append('|').Append(r.LandingLocal)
                .Append("/firstMissOutcome=").Append(e.MissOutcome ?? "NA")
                .Append("/actualFirstFormal-local=").Append(e.Formal ?? "Pending").Append('|').Append(e.FormalLocal);
        }

        private static void AppendLeadRow(StringBuilder b, Row r)
        {
            var s = r.Lead;
            if (s == null) return;
            b.Append("\nLEFT_FOOT_LEAD_STEP_V041660/origin-depth-from-to=").Append(s.Origin).Append('|').Append(s.Depth)
                .Append('|').Append(s.Local).Append('|').Append(r.Settled ? r.To.Local : 0)
                .Append("/action=").Append(s.Decision).Append("/paired=").Append(s.Read)
                .Append("/predictedAnkleDeltaMm=").Append(Vec(r.Command.FootPoint-r.Shin.LowerAnkle,1000))
                .Append("/issuedA=").Append(Vec(r.Motion.AppliedLinearA,1)).Append("/rawTetherA=").Append(Vec(r.Motion.Tether.Raw,1))
                .Append("/dt-drag-kp-kd-cap=").Append(N(r.From.Step.Dt)).Append('|').Append(N(r.Motion.LinearDrag)).Append('|')
                .Append(N(r.Motion.Tether.Kp)).Append('|').Append(N(r.Motion.Tether.Kd)).Append('|').Append(N(s.Cap));
            if (!s.Issued)
            {
                if (s.Decision == Lead.Action.NoRoom) b.Append("/neededXZmm=").Append(Mm(s.Needed));
                return;
            }
            b.Append("/needed-addedXZ-riseMm=").Append(Mm(s.Needed)).Append('|').Append(Mm(s.Added)).Append('|').Append(Mm(s.Rise))
                .Append("/vertical=").Append(s.Vertical?.ToString() ?? "NA").Append("/trialA=").Append(Vec(s.Applied,1))
                .Append("/endOffsetMm-endVelocityMps=").Append(Vec(s.Offset,1000)).Append('|').Append(Vec(s.Velocity,1))
                .Append("/predictedBaseSole-gap-trialSole-gap-sharedKneeMm=").Append(Mm(s.BaseSole)).Append('|')
                .Append(Mm(s.BaseGap)).Append('|').Append(Mm(s.TrialSole)).Append('|').Append(Mm(s.TrialGap)).Append('|').Append(Mm(s.KneeGap));
            if (!s.Read) return;
            b.Append("/actualSole-gap-kneeMm=").Append(Mm(s.ActualSole)).Append('|').Append(Mm(s.ActualGap)).Append('|').Append(Mm(s.ActualKnee))
                .Append("/conditionedSole-gapMm=").Append(Mm(s.ConditionedSole)).Append('|').Append(Mm(s.ConditionedGap))
                .Append("/inside-actualPass-trialPass=").Append(s.Inside).Append('|').Append(s.ActualPass).Append('|').Append(s.TrialPass);
        }
    }
}
