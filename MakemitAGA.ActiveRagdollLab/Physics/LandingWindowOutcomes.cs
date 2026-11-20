using System;
using System.Text;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;
using Rejection = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Rejection;
using MissEnd = MakemitAGA.ActiveRagdollLab.Physics.LandingEventHistory.MissEnd;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Event denominators need only the latest qualifying step and a receipt for the open missed hold.
    // Multiple windows preceding the same landing must contribute one observation to that landing.
    internal sealed class LandingWindowOutcomes
    {
        private enum Prior { None, SameStep, Earlier }
        private struct Exposure
        {
            private int _total, _earlier, _same;
            internal void Add(Prior relation)
            {
                _total++;
                if (relation == Prior.Earlier) _earlier++;
                if (relation == Prior.SameStep) _same++;
            }
            internal string Text() => _total + "|" + _earlier + "|" + _same + "|" + (_total - _earlier - _same);
        }

        private readonly LandingEventHistory _history = new LandingEventHistory();
        private readonly Exposure[] _ends = new Exposure[Enum.GetValues(typeof(MissEnd)).Length];
        private Exposure _ordinary, _miss, _recovery;
        private Prior _priorAtMiss;
        private bool _knownMiss, _heldCandidate;
        private int _unknownRecovery, _lastQualified;
        private double _time, _lastQualifiedTime;

        internal void Reset()
        {
            _history.Reset(); Array.Clear(_ends);
            _ordinary = _miss = _recovery = default;
            _priorAtMiss = Prior.None;
            _knownMiss = _heldCandidate = false;
            _unknownRecovery = _lastQualified = 0;
            _time = _lastQualifiedTime = 0d;
        }

        internal void Cut(MissEnd reason)
        {
            EndMiss(reason);
            _lastQualified = 0;
            _history.ClearWindow();
        }

        private void EndMiss(MissEnd reason)
        {
            if (_knownMiss)
            {
                _ends[(int)reason].Add(_priorAtMiss);
                _history.EndMiss(reason);
            }
            _knownMiss = _heldCandidate = false;
        }

        internal static bool StartsNewSwing(Phase current, Phase previous) =>
            (current == Phase.Lift && previous != Phase.Lift && previous != Phase.Swing) ||
            (current == Phase.Swing && (previous == Phase.Approach || previous == Phase.Stance));

        internal void Observe(in Sample s, bool passes, bool sameSurface, bool newSwing,
            bool history = false, Rejection rejects = Rejection.None, in RightKneePivot.Frame pivot = default)
        {
            _time += s.Step.Dt;
            bool ordinary = !s.WasActive && s.NowActive && s.OrdinaryEnabled && !s.RecoveryEnabled;
            bool recovery = !s.WasActive && s.NowActive && s.RecoveryEnabled;
            if (!sameSurface || !s.HeightValid || s.WasActive) Cut(MissEnd.Discontinuity);
            if ((!s.MissInProgress && !recovery) || s.FirstMiss) EndMiss(MissEnd.HoldEnded);
            if (newSwing || _time - _lastQualifiedTime > LandingEventHistory.Horizon + 1e-6d) _lastQualified = 0;

            if (history && !s.WasActive) _history.Add(s, _time, rejects, pivot);
            // Evaluate the prior step before admitting the current sample, including on a new swing edge.
            Prior relation = _heldCandidate || _lastQualified != 0 ? Prior.Earlier : passes ? Prior.SameStep : Prior.None;
            if (s.FirstMiss)
            {
                _miss.Add(relation); _priorAtMiss = relation; _knownMiss = true;
                if (history) _history.Capture(s, true);
            }
            if (s.FirstMiss || s.MissInProgress)
            {
                _heldCandidate |= passes || _lastQualified != 0;
                _lastQualified = 0;
            }
            else if (passes)
            {
                _lastQualified = s.Step.Sequence;
                _lastQualifiedTime = _time;
            }
            if (ordinary)
            {
                _ordinary.Add(relation);
                if (history) _history.Capture(s, false);
            }
            if (recovery)
            {
                _recovery.Add(relation);
                if (!_knownMiss) _unknownRecovery++;
                EndMiss(MissEnd.Recovered);
            }
            if (ordinary || recovery) _lastQualified = 0;
            if (!s.HeightValid) _history.ClearWindow();
        }

        internal string Summary()
        {
            var b = new StringBuilder("\nLANDING_EVENT_LINK_V041440/firstEventMaxAfterLastGoodMs=500")
                .Append("/eventColumns=events-withEarlierWindow-sameStepOnly-noWindow")
                .Append("/ordinary=").Append(_ordinary.Text()).Append("/firstMiss=").Append(_miss.Text())
                .Append("/shallowRecovery=").Append(_recovery.Text())
                .Append("\nLANDING_EVENT_MISS_V041440/recoveryWithoutObservedOrigin=").Append(_unknownRecovery);
            foreach (MissEnd end in Enum.GetValues(typeof(MissEnd))) b.Append('/').Append(end).Append('=').Append(_ends[(int)end].Text());
            _history.Append(b);
            return b.ToString();
        }
    }
}
