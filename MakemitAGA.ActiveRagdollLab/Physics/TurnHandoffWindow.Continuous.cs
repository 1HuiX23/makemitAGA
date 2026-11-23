using System.Text;
using Trace = MakemitAGA.ActiveRagdollLab.Physics.BilateralFootTransitionTrace;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Trial = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;
using Entry = MakemitAGA.ActiveRagdollLab.Physics.OrdinaryFootEntry;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class TurnHandoffWindow
    {
        private Entry.Trail _nativeEntry, _trialEntry;
        private int _sequenceStopLocal, _sequenceReads, _marginSteps;
        private float _leastMargin;
        private string _sequenceStop;

        private void EndContinuous(int local, string reason)
        {
            _sequenceStopLocal = local;
            _sequenceStop = reason;
        }

        private void IssueContinuous(Row row, in Trace.Tick t, Foot.Pair previous)
        {
            if (_sequenceStop != null) return;
            // Only the release may seed zero translation. Every subsequent solve
            // consumes its own settled position AND velocity, even before strong stance.
            ref var move = ref row.Traction.Continuous;
            move = Foot.Predict(row.Nominal, row.Plan, row.IssuedAim, row.Traction.Command,
                row.Dt, t.Foot - t.Lower, previous?.Continuous ?? default, row.From != Start, true);
            if (move.Status != Foot.State.Predicted) EndContinuous(t.Local, move.Status.ToString());
        }

        private void ReadContinuous(Row row, in Trace.Tick t, in Trial.Pose pose, in Trial.Aim aim, string prohibitIssue)
        {
            row.Entry = t.Entry;
            if (_sequenceStop != null) return;
            ref var move = ref row.Traction.Continuous;
            Foot.SettleOnLeg(ref move, pose, row.Conditional, aim, row.Actual, row.Traction.Command.Tether.HardGap);
            if (move.Status != Foot.State.Read || !move.SafeRead)
            { EndContinuous(t.Local, "UnsafeRead"); return; }
            _sequenceReads++;
            var error = CheckNativeEntry(t, row.Actual.Inside, prohibitIssue);
            if (error != null) { EndContinuous(t.Local, error); return; }

            var decision = _trialEntry.Observe(t.Entry, move.Settled.Inside, move.Settled.Sole);
            if (_trialEntry.StrongSamples <= 1) { _marginSteps = 0; _leastMargin = float.PositiveInfinity; }
            if (_trialEntry.StrongSamples > 0)
            {
                if (move.MarginPass) _marginSteps++;
                _leastMargin = System.Math.Min(_leastMargin, move.WorstMargin);
            }
            // After the first hypothetical enable, further free-space motion has no valid meaning.
            if (decision == Entry.Decision.Ordinary) EndContinuous(t.Local, "WouldEnter");
            else if (t.Ordinary || t.Miss) EndContinuous(t.Local,
                decision == Entry.Decision.Miss ? "DepthLatched" : "NotReadyAtFirst");
        }

        private string CheckNativeEntry(in Trace.Tick t, bool inside, string prohibitIssue)
        {
            if (prohibitIssue != null && !(prohibitIssue == "LandingResolved" && (t.Miss || t.Ordinary)))
                return prohibitIssue;
            if (t.Entry.Local != t.Local || (t.Entry.Floor != 0 && t.Entry.Floor != t.Floor) ||
                t.Precontact || t.Recovery || t.Active != t.Ordinary || (t.Miss && t.Ordinary) ||
                ((t.Miss || t.Ordinary) && t.EventFloor != t.Floor))
                return "EntryReceipt";

            if (t.Entry.Floor != 0 && (!float.IsFinite(t.Entry.NativeSole) ||
                System.Math.Abs(t.Entry.NativeSole - t.BoxMin) > FootBoxGeometry.SupportPlaneToleranceMeters ||
                t.Entry.NativeInside != inside))
                return "EntryGeometryMismatch";
            var native = _nativeEntry.Observe(t.Entry, t.Entry.NativeInside, t.Entry.NativeSole);
            if ((native == Entry.Decision.Miss) != t.Miss || (native == Entry.Decision.Ordinary) != t.Ordinary ||
                (t.Miss && _nativeEntry.DepthLatch != t.NativeDepthLatch))
                return "NativeDisagreement";
            return null;
        }

        internal void AppendContinuous(StringBuilder b, in Trace.Tick first, string landing, int resolved, int firstEligible)
        {
            b.Append("\nTURN_CONTINUOUS_V041740/start-first-resolved=").Append(Start).Append('|').Append(first.Local).Append('|').Append(resolved)
                .Append("/native=").Append(landing ?? "Open").Append("/nativeFirstEligible=").Append(firstEligible).Append("/nativeDepthLatch=").Append(first.NativeDepthLatch)
                .Append("/sequenceEnd=").Append(_sequenceStop ?? End ?? "Open").Append("/stopLocal=")
                .Append(_sequenceStop == null ? _lastVisit : _sequenceStopLocal).Append("/safeReads=").Append(_sequenceReads)
                .Append("/strongStart-nativeLatch-trialLatch=").Append(_nativeEntry.StrongStart).Append('|').Append(_nativeEntry.DepthLatch).Append('|').Append(_trialEntry.DepthLatch)
                .Append("/strongSteps-marginPass-readySamples=").Append(_trialEntry.StrongSamples).Append('|').Append(_marginSteps).Append('|').Append(_trialEntry.ReadySamples)
                .Append("/worstMarginMm=").Append(_trialEntry.StrongSamples > 0 ? Mm(_leastMargin) : "NA")
                .Append("/model=CONDITIONED_ON_ISSUED_LEG_AND_NATIVE_GAIT/footWrites=0");
            foreach (var row in _rows)
            {
                if (row.Traction == null || (_sequenceStop != null && row.From > _sequenceStopLocal)) break;
                var m = row.Traction.Continuous;
                if (m.Status != Foot.State.Read && m.Status != Foot.State.Predicted) continue;
                b.Append("\nC174/issue-read=").Append(row.From).Append('|').Append(m.Status == Foot.State.Read ? row.To : 0).Append("/status=").Append(m.Status)
                    .Append("/footOffsetM=").Append(V(m.Body.Position)).Append("/footDeltaVMps=").Append(V(m.Body.Velocity))
                    .Append("/predSole-gapMm=").Append(Mm(m.Forecast.Sole)).Append('|').Append(Mm(m.Forecast.Gap));
                if (m.Status != Foot.State.Read) continue;
                b.Append("/readSole-gapMm=").Append(Mm(m.Settled.Sole)).Append('|').Append(Mm(m.Settled.Gap))
                    .Append("/safe-marginPass=").Append(m.SafeRead).Append('|').Append(m.MarginPass)
                    .Append("/entryFloor-confirmed-ground-identityHeld=").Append(row.Entry.Floor).Append('|').Append(row.Entry.Confirmed)
                    .Append('|').Append(row.Entry.LegacyReady).Append('|').Append(row.Entry.IdentityHeld);
                if (row.Entry.Floor != 0) b.Append("/nativeEntrySoleMm=").Append(Mm(row.Entry.NativeSole));
            }
        }
    }
}
