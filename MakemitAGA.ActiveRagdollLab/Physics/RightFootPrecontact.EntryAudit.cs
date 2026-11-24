using System;
using System.Text;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class RightFootPrecontact
    {
        private EntryAudit _entryAudit;

        internal void BeginEntryAudit(int local, bool tracing, bool previousMissCounted) =>
            _entryAudit?.Begin(local, tracing, Tracking ? _current.Origin : 0, previousMissCounted);
        internal void ObserveExistingFormal() => _entryAudit?.ExistingFormal();
        internal void CompleteEntryAudit(bool ok, bool ordinary, bool recovery, bool firstMiss, int depth,
            bool missHeld, int linkedOrigin, bool formalActive) =>
            _entryAudit?.Complete(ok, ordinary, recovery, firstMiss, depth, missHeld, linkedOrigin, formalActive);

        // Only physical window ownership is carried between solves. A closed precontact
        // can precede an unrelated formal entry; keep its real origin distinct from the formal link.
        private sealed class EntryAudit
        {
            private enum Start { TraceStart, AfterFormal, Continuity, Fault }
            private const int EventLimit = 128;
            private struct Interval
            {
                internal Start StartReason;
                internal int Samples, Inexact, StartLocal, LastNewSwing, CarriedOrigin;
                internal int Opens, FirstOpen, LastOpen, LastCloseOrigin, LastCloseLocal;
                internal int Added35, FirstAdded35, LastAdded35;
                internal End LastClose;
            }
            private struct Landing
            {
                internal int Epoch, Shared, Local, Source, Support, Depth, LinkedOrigin;
                internal bool FirstMiss;
                internal Interval Interval;
            }
            private readonly Landing[] _landings = new Landing[EventLimit];
            private readonly Mode _selected;
            private Interval _span;
            private Landing _step;
            private int _fixed, _evaluations, _formalBypasses, _faults, _requests, _opens, _unexpected;
            private int _commonOpens, _addedOpens, _unknownOpens, _landingCount, _ordinary, _firstMiss, _unkeyed;
            private int _carry;
            private bool _live, _formalAtIssue, _previousMissCounted, _inexact, _windowInSpan, _keyed, _requested, _opened;

            internal EntryAudit(Mode selected) { _selected = selected; }

            internal void Begin(int local, bool tracing, int carriedOrigin, bool previousMissCounted)
            {
                _live = tracing;
                if (!tracing) return;
                _fixed++;
                _step = new Landing { Local = local };
                _carry = carriedOrigin;
                _formalAtIssue = _windowInSpan = _requested = _opened = _keyed = _inexact = false;
                _previousMissCounted = previousMissCounted;
            }

            internal void ExistingFormal()
            {
                if (!_live) return;
                _formalAtIssue = true;
                _formalBypasses++;
            }

            internal void Observe(in Sample s, bool same, bool newSwing, bool tracking, bool request)
            {
                if (!_live || _step.Local != s.Local) return;
                _evaluations++;
                if (!same && (_span.StartLocal != 0 || _carry != 0)) Cut(Start.Continuity);
                _windowInSpan = tracking && same;
                if (_span.StartLocal == 0)
                {
                    _span.StartLocal = s.Local;
                    _span.CarriedOrigin = _windowInSpan ? _carry : 0;
                }
                _span.Samples++;
                _inexact = !s.Clock || !s.Geometry || !s.Fresh || !s.PhaseKnown;
                if (_inexact) _span.Inexact++;
                if (newSwing) _span.LastNewSwing = s.Local;
                _keyed = s.Clock && s.Support != 0;
                _step.Epoch = s.Step.Epoch; _step.Shared = s.Step.Sequence;
                _step.Source = s.Source.Sequence; _step.Support = s.Support;
                _requested = request;
                if (request) _requests++;
            }

            internal void Opened(int local, EntryBand band)
            {
                if (!_live || _step.Local != local) return;
                _opened = true;
                _opens++;
                if (_span.Opens++ == 0) _span.FirstOpen = local;
                _span.LastOpen = local;
                if (band == EntryBand.Added35)
                {
                    _addedOpens++;
                    if (_span.Added35++ == 0) _span.FirstAdded35 = local;
                    _span.LastAdded35 = local;
                }
                else if (band == EntryBand.Common25) _commonOpens++;
                else _unknownOpens++;
            }
            internal void Closed(int origin, End reason, int local)
            {
                if (!_live || _step.Local != local || !(_windowInSpan || _opened)) return;
                _span.LastCloseLocal = local; _span.LastCloseOrigin = origin; _span.LastClose = reason;
            }
            private void Cut(Start reason) => _span = new Interval { StartReason = reason };

            internal void Complete(bool ok, bool ordinary, bool recovery, bool firstMiss, int depth,
                bool missHeld, int linkedOrigin, bool formalActive)
            {
                if (!_live) return;
                _live = false;
                if (!ok) _faults++;
                bool allowed = ok && _requested && !(ordinary || recovery || firstMiss || missHeld || _formalAtIssue || formalActive);
                if (ok && allowed != _opened) _unexpected++;
                if (ok && (ordinary || firstMiss))
                {
                    if (_keyed)
                    {
                        if (ordinary) _ordinary++;
                        if (firstMiss) _firstMiss++;
                        _step.FirstMiss = firstMiss; _step.Depth = depth; _step.LinkedOrigin = linkedOrigin; _step.Interval = _span;
                        if (_landingCount < EventLimit) _landings[_landingCount] = _step;
                        _landingCount++;
                    }
                    else _unkeyed++;
                }
                if (!ok) Cut(Start.Fault);
                else if (_formalAtIssue || ordinary || recovery || firstMiss) Cut(Start.AfterFormal);
                else if (_previousMissCounted)
                {
                    Cut(Start.AfterFormal);
                    if (!missHeld && _keyed)
                    {
                        _span.StartLocal = _step.Local; _span.Samples = 1; _span.Inexact = _inexact ? 1 : 0;
                    }
                }
            }

            internal void Append(StringBuilder b)
            {
                b.Append("\nRIGHT_PRECONTACT_ENTRY_V041830/side=RIGHT/nativeFirstMm=").Append(FirstMaximumMm(_selected))
                    .Append("/fixed-evaluated-formalBypass-fault=").Append(_fixed).Append('|').Append(_evaluations)
                    .Append('|').Append(_formalBypasses).Append('|').Append(_faults)
                    .Append("/nativeRequest-open-commitMismatch=").Append(_requests).Append('|').Append(_opens).Append('|').Append(_unexpected)
                    .Append("/openedCommon25-added35-unknown=").Append(_commonOpens).Append('|').Append(_addedOpens).Append('|').Append(_unknownOpens)
                    .Append("/ordinary-firstMiss-unkeyed=").Append(_ordinary).Append('|').Append(_firstMiss).Append('|').Append(_unkeyed)
                    .Append("/landingTotal-saved-omitted="); AppendCoverage(b, _landingCount, EventLimit);
                for (int i = 0; i < Math.Min(_landingCount, EventLimit); i++)
                {
                    ref var e = ref _landings[i]; ref var span = ref e.Interval;
                    int boundary = e.FirstMiss && e.Depth > 0 ? e.Depth : e.Local;
                    b.Append("\nRIGHT_PRECONTACT_LANDING_V041830/epoch-shared-local-source-support="); Key(b, e.Epoch, e.Shared, e.Local, e.Source, e.Support);
                    b.Append("/kind=").Append(e.FirstMiss ? "FirstMiss" : "Ordinary").Append("/depthLocal=").Append(e.Depth)
                        .Append("/start-local=").Append(span.StartReason).Append('|').Append(span.StartLocal).Append("/lastNewSwing=").Append(span.LastNewSwing)
                        .Append("/samples-inexact=").Append(span.Samples).Append('|').Append(span.Inexact)
                        .Append("/boundaryInSpan=").Append(span.StartLocal > 0 && span.StartLocal <= boundary && boundary <= e.Local)
                        .Append("/carriedOrigin=").Append(span.CarriedOrigin).Append("/nativeStarts-first-last=").Append(span.Opens)
                        .Append('|').Append(span.FirstOpen).Append('|').Append(span.LastOpen)
                        .Append("/lastCloseOrigin-local-end=").Append(span.LastCloseOrigin).Append('|').Append(span.LastCloseLocal)
                        .Append('|').Append(span.LastClose).Append("/formalLinkedOrigin=").Append(e.LinkedOrigin)
                        .Append("/added35Starts-first-last=").Append(span.Added35).Append('|').Append(span.FirstAdded35).Append('|').Append(span.LastAdded35);
                }
            }
            private static void Key(StringBuilder b, int epoch, int shared, int local, int source, int support) =>
                b.Append(epoch).Append('|').Append(shared).Append('|').Append(local).Append('|').Append(source).Append('|').Append(support);
            private static void AppendCoverage(StringBuilder b, int total, int limit) =>
                b.Append(total).Append('|').Append(Math.Min(total, limit)).Append('|').Append(Math.Max(0, total - limit));
        }
    }
}
