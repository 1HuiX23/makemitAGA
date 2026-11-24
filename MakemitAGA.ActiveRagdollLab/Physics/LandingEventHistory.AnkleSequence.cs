using System.Text;
using Seq = MakemitAGA.ActiveRagdollLab.Physics.RightAnkleLandingSequence;
using Mark = MakemitAGA.ActiveRagdollLab.Physics.RightAnkleLandingSequence.Mark;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class LandingEventHistory
    {
        private struct SequenceTotals
        {
            internal int Warm, Measured, Missing, Reopened, Held, NewSole, LostSole, UnscoredSafe, Torque, NewCross, First;
        }
        // The half-second history can start partway through a verified sequence.
        // Only a demonstrated cycle boundary excludes an earlier origin.
        private static bool SequenceInCycle(Record r,int start,int i,out Seq.Readback s)
        {
            var p=r.Points; var n=r.EarlyRows[i].Landing; s=n.Preparation;
            return s.Depth>0 && n.From==p[i-1].Local && n.From+1==p[i].Local &&
                s.Origin+s.Depth==p[i].Local && (start==0 || s.Origin>=p[start].Local);
        }
        private static void AppendAnkleSequence(StringBuilder b, Record r, int anchor)
        {
            var p=r.Points;
            b.Append("\nRIGHT_ANKLE_SEQUENCE_EVENT_V041530/epoch-support=").Append(r.Epoch).Append('|').Append(r.Support)
                .Append("/eventLocal=").Append(p[p.Length-1].Local).Append("/kind=").Append(r.Kind)
                .Append("/outcome=").Append(r.Kind==Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open")
                .Append("/anchorCaptured=").Append(anchor>=0);
            if (anchor<0) return;
            int start=CycleStart(p,anchor), paired=0, multi=0, oldest=0, maxDepth=0;
            var totals=new SequenceTotals[Seq.Count]; // Sealed event report only.
            for (int i=start+1;i<=anchor;i++)
            {
                if (!SequenceInCycle(r,start,i,out var s)) continue;
                if (s.Origin<p[start].Local) oldest++;
                paired++; if (s.Depth>maxDepth) maxDepth=s.Depth;
                if (s.Depth>=2) multi++;
                for (int lane=0;lane<Seq.Count;lane++)
                {
                    ref var t=ref totals[lane]; var v=s.Lane(lane);
                    if (s.Depth==1) { if (v.Has(Mark.Measured)) t.Warm++; continue; }
                    if (!v.Has(Mark.Measured)) { t.Missing++; if (s.SoleAndGap) t.UnscoredSafe++; continue; }
                    t.Measured++;
                    if (!s.GapPass && v.Has(Mark.GapPass)) t.Reopened++;
                    if (!s.GapPass && v.Has(Mark.HeldAndGap)) t.Held++;
                    bool newSole=!s.SoleAndGap && v.Has(Mark.SoleAndGap);
                    bool newCross=!s.OriginalTorqueCross && v.Has(Mark.TorqueCross);
                    if (newSole) t.NewSole++;
                    if (s.SoleAndGap && !v.Has(Mark.SoleAndGap)) t.LostSole++;
                    if (v.Has(Mark.TorqueGain)) t.Torque++;
                    if (newCross) t.NewCross++;
                    if ((newSole || newCross) && t.First==0) t.First=i;
                }
            }
            b.Append("/anchorLocal=").Append(p[anchor].Local).Append("/window-paired-multi-maxDepth-originBeforeHistory=")
                .Append(anchor-start).Append('|').Append(paired).Append('|').Append(multi).Append('|').Append(maxDepth).Append('|').Append(oldest);
            for (int lane=0;lane<Seq.Count;lane++)
            {
                var t=totals[lane];
                b.Append("/lane").Append(lane+1).Append("(warm-measured-unavailable-reopened-heldNew-soleNew-soleLost-safeUnscored-torqueGain-newTorqueCross)=")
                    .Append(t.Warm).Append('|').Append(t.Measured).Append('|').Append(t.Missing).Append('|').Append(t.Reopened).Append('|')
                    .Append(t.Held).Append('|').Append(t.NewSole).Append('|').Append(t.LostSole).Append('|').Append(t.UnscoredSafe)
                    .Append('|').Append(t.Torque).Append('|').Append(t.NewCross)
                    .Append("/firstAlignedEndpointLeadMs").Append(lane+1).Append('=').Append(Lead(p,anchor,t.First==0?-1:t.First));
            }
            for (int i=start+1;i<=anchor;i++)
            {
                if (!SequenceInCycle(r,start,i,out var s)) continue;
                var input=s.Command;
                b.Append("\nRIGHT_ANKLE_SEQUENCE_POINT_V041530/event-anchor-from-to-source-origin-depth=")
                    .Append(p[p.Length-1].Local).Append('|').Append(p[anchor].Local).Append('|').Append(p[i-1].Local).Append('|').Append(p[i].Local)
                    .Append('|').Append(p[i].Source).Append('|').Append(s.Origin).Append('|').Append(s.Depth)
                    .Append("/realGap-xz-signedY-sole-heldAtFoot-heldAtComMm=").Append(Number(s.Gap*1000d)).Append('|')
                    .Append(Number(s.Xz*1000d)).Append('|').Append(Number(s.SignedY*1000d)).Append('|').Append(Number(s.Box*1000d))
                    .Append('|').Append(Number(s.GoalFoot*1000d)).Append('|').Append(Number(s.GoalCom*1000d))
                    .Append("/originalTorqueCross=").Append(s.OriginalTorqueCross)
                    .Append("/fromShared-source=").Append(s.FromShared).Append('|').Append(s.FromSource)
                    .Append("/dt-drag-cap-pair=").Append(Number(input.Dt)).Append('|').Append(Number(input.Drag)).Append('|').Append(Number(input.Cap)).Append('|').Append(input.PairAvailable)
                    .Append("/issuedA=").Append(Number(input.AppliedA.x)).Append('|').Append(Number(input.AppliedA.y)).Append('|').Append(Number(input.AppliedA.z))
                    .Append("/fromFootMinusLowerMm=").Append(Number(input.AnkleDelta.x*1000d)).Append('|').Append(Number(input.AnkleDelta.y*1000d)).Append('|').Append(Number(input.AnkleDelta.z*1000d))
                    .Append("/endFootMinusLowerXzMm=").Append(Number(s.EndX*1000d)).Append('|').Append(Number(s.EndZ*1000d));
                for (int lane=0;lane<Seq.Count;lane++)
                {
                    var v=s.Lane(lane); b.Append("/lane").Append(lane+1).Append('=').Append((int)v.Flags);
                    if (v.Has(Mark.Measured)) b.Append('|').Append(Number(v.Gap*1000d)).Append('|').Append(Number(v.Xz*1000d))
                        .Append('|').Append(Number(v.TorqueGain*1000d)).Append('|').Append(Number(v.TorqueGapBound*1000d));
                    if (v.Has(Mark.TorqueTested)) b.Append('|').Append(v.TorqueState).Append('|').Append((int)v.TorqueRejections);
                }
            }
        }
    }
}
