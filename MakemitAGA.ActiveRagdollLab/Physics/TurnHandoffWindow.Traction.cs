using System.Text;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class TurnHandoffWindow
    {
        internal bool TryTractionPass(int target, bool twoSteps, out bool pass)
        {
            pass = false;
            int index = target - Start - 1;
            if (index < 0 || index >= _rows.Count) return false;
            var r = _rows[index];
            if (!r.Read || !r.BaselineAgrees || r.To != target || r.Traction == null) return false;
            var m = twoSteps ? r.Traction.Two : r.Traction.One;
            if (m.Status != Foot.State.Read) return false;
            pass = m.Pass;
            return true;
        }

        internal bool TryTwoStepMargin(int local, bool buffered, out Foot.Move move)
        {
            move = default;
            int index = local - Start - 1;
            if (index < 0 || index >= _rows.Count) return false;
            var row = _rows[index];
            if (!row.Read || !row.BaselineAgrees || row.To != local || row.Traction == null) return false;
            move = buffered ? row.Traction.BufferedTwo : row.Traction.Two;
            return move.Status == Foot.State.Read;
        }

        internal void AppendTraction(StringBuilder b, int first, TurnHandoffWindow early)
        {
            b.Append("\nTURN_TRACTION_LINK_V041710/start-first=").Append(Start).Append('|').Append(first)
                .Append("/early-braked-one-two=").Append(Grade(early.TryReadPass(first, out bool ep),ep)).Append('|')
                .Append(Grade(TryReadPass(first,out bool bp),bp)).Append('|')
                .Append(Grade(TryTractionPass(first,false,out bool fp),fp)).Append('|')
                .Append(Grade(TryTractionPass(first,true,out bool tp),tp)).Append("/brakedEnd=").Append(End ?? "Open");
            foreach (var r in _rows)
            {
                var f = r.Traction;
                if (f == null) continue;
                var cmd = f.Command;
                b.Append("\nTURN_TRACTION_V041710/start-from-to=").Append(Start).Append('|').Append(r.From).Append('|').Append(r.To)
                    .Append("/receiptValid=").Append(cmd.Valid).Append("/rawA-appliedA=").Append(V(cmd.Tether.Raw)).Append('|').Append(V(cmd.Applied))
                    .Append("/baseCap-selectedCap-Kp-Kd-drag-hardMm=").Append(N(cmd.Tether.BaseCap)).Append('|').Append(N(cmd.Tether.SelectedCap))
                    .Append('|').Append(N(cmd.Tether.Kp)).Append('|').Append(N(cmd.Tether.Kd)).Append('|').Append(N(cmd.Drag))
                    .Append('|').Append(Mm(cmd.Tether.HardGap));
                AppendMove(b,r,1,f.One); AppendMove(b,r,2,f.Two);
                AppendMargin(b,r,f.BufferedOne,f.BufferedTwo);
            }
        }

        private static string Grade(bool known,bool pass) => known ? (pass ? "Pass" : "Fail") : "Unavailable";
        private void AppendMove(StringBuilder b, Row row, int steps, in Foot.Move m)
        {
            b.Append("\nTURN_TRACTION_MOVE_V041710/start-from-to=").Append(Start).Append('|').Append(row.From).Append('|').Append(row.To)
                .Append("/steps=").Append(steps).Append("/status=").Append(m.Status);
            if (m.Status != Foot.State.Predicted && m.Status != Foot.State.Read) return;
            b.Append("/requestedM=").Append(V(m.RequestedShift)).Append("/soleNeedMm-verticallyPossible-fraction=")
                .Append(Mm(m.ClearanceNeed)).Append('|').Append(m.HeightReachable).Append('|').Append(N(m.Fraction))
                .Append("/usedA=").Append(V(m.Applied)).Append("/offsetM-velocityMps=").Append(V(m.Body.Position)).Append('|').Append(V(m.Body.Velocity))
                .Append("/predSole-gapMm=").Append(Mm(m.Forecast.Sole)).Append('|').Append(Mm(m.Forecast.Gap));
            if (m.Status == Foot.State.Read)
                b.Append("/readSole-gapMm=").Append(Mm(m.Settled.Sole)).Append('|').Append(Mm(m.Settled.Gap))
                    .Append("/readPlanarMm=").Append(Planar(m.Settled)).Append("/safe-pass=").Append(m.SafeRead).Append('|').Append(m.Pass);
        }

        private void AppendMargin(StringBuilder b, Row r, in Foot.Move seed, in Foot.Move two)
        {
            b.Append("\nTURN_TRACTION_MARGIN_V041720/start-from-to=").Append(Start).Append('|').Append(r.From).Append('|').Append(r.To)
                .Append("/currentSeedStatus-twoStepStatus=").Append(seed.Status).Append('|').Append(two.Status);
            if (two.Status != Foot.State.Predicted && two.Status != Foot.State.Read) return;
            b.Append("/usedA=").Append(V(two.Applied)).Append("/deltaVy=").Append(N(two.Body.Velocity.y))
                .Append("/heightReachable-fraction=").Append(two.HeightReachable).Append('|').Append(N(two.Fraction))
                .Append("/predSole-gapMm=").Append(Mm(two.Forecast.Sole)).Append('|').Append(Mm(two.Forecast.Gap));
            if (two.Status == Foot.State.Read)
                b.Append("/readSole-gap-worstMarginMm=").Append(Mm(two.Settled.Sole)).Append('|').Append(Mm(two.Settled.Gap))
                    .Append('|').Append(Mm(two.WorstMargin)).Append("/safe-ordinary-marginPass=").Append(two.SafeRead)
                    .Append('|').Append(two.Pass).Append('|').Append(two.MarginPass);
        }
    }

    internal sealed partial class BilateralFootTransitionTrace
    {
        private struct LandingComparison
        {
            internal int Paired, Before, After, Gained, Lost;
            internal void Add(bool oldKnown,bool oldPass,bool newKnown,bool newPass)
            {
                if (!oldKnown || !newKnown) return;
                Paired++;
                if (oldPass) Before++;
                if (newPass) After++;
                if (!oldPass && newPass) Gained++;
                if (oldPass && !newPass) Lost++;
            }
            internal void Write(StringBuilder b) => b.Append(Paired).Append('|').Append(Before).Append('|').Append(After)
                .Append('|').Append(Gained).Append('|').Append(Lost);
        }
        private void AppendTractionCounts(StringBuilder b)
        {
            int started = 0, first = 0;
            LandingComparison guard = default, one = default, two = default, band = default, margin = default;
            foreach (var w in _windows)
            {
                if (w.Braked == null) continue;
                started++;
                int local = FirstOpportunity(w);
                if (local == 0) continue;
                first++;
                bool e = w.Early.TryReadPass(local,out bool ep), g = w.Braked.TryReadPass(local,out bool gp);
                guard.Add(e,ep,g,gp);
                one.Add(g,gp,w.Braked.TryTractionPass(local,false,out bool p1),p1);
                two.Add(g,gp,w.Braked.TryTractionPass(local,true,out bool p2),p2);
                bool oldRead = w.Braked.TryTwoStepMargin(local,false,out var oldFoot);
                bool newRead = w.Braked.TryTwoStepMargin(local,true,out var newFoot);
                band.Add(oldRead,oldFoot.Pass,newRead,newFoot.Pass);
                margin.Add(oldRead,oldFoot.MarginPass,newRead,newFoot.MarginPass);
            }
            b.Append("\nTURN_TRACTION_COUNTS_V041710/started-first=").Append(started).Append('|').Append(first)
                .Append("/columns=paired-beforePass-afterPass-gained-lost/earlyToBraked=");
            guard.Write(b); b.Append("/brakedToOne="); one.Write(b); b.Append("/brakedToTwo="); two.Write(b);
            b.Append("\nTURN_MARGIN_COUNTS_V041720/started-first=").Append(started).Append('|').Append(first)
                .Append("/twoStepColumns=paired-beforePass-afterPass-gained-lost/originalBand=");
            band.Write(b); b.Append("/quarterMmBoth="); margin.Write(b);
        }
    }
}
