using System;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // One formal Hip run owns this main-thread instrument. No sampled duration is a control input.
    internal sealed class LandingObservationTiming
    {
        internal static readonly LandingObservationTiming Run = new LandingObservationTiming();
        internal enum Stage
        {
            FixedLoop, FixedTorso, FixedLegs, SourceLoop, SourceTorso, SourceLegs,
            LegFullCapture, LegParentCapture, FootPathBuild, LifecycleActive, LifecycleInactive,
            ObserverActive, ObserverInactive, EntryText,
            LegPrepare, HipPair, KneePd, FootFixed, FootProbe, BilateralEvidence, EarlyKneeProbe, ParentHeightProbe, RightAnkleProbe, RightKneePivotSnapshot, RightThighTracking, RightFootForecast, RightFootLanding, LeftFootRelease, LeftShinForecast, LeftFootLead, LeftLegPreparation, FootTransitionSource, FootTransitionPair, TurnHandoff, LandingRise, CombinedLanding
        }

        internal readonly struct Measurement : IDisposable
        {
            private readonly LandingObservationTiming _owner;
            private readonly Stage _stage;
            private readonly int _generation;
            private readonly long _start;
            private readonly bool _bothAtEntry;
            internal Measurement(LandingObservationTiming owner, Stage stage, bool bothAtEntry)
            {
                _owner = owner;
                _stage = stage;
                _bothAtEntry = bothAtEntry;
                _generation = owner._generation;
                _start = Stopwatch.GetTimestamp();
            }
            public void Dispose()
            {
                if (_owner != null && _owner.Running && _generation == _owner._generation)
                {
                    long ticks = Stopwatch.GetTimestamp() - _start;
                    _owner._times[(int)_stage].Add(ticks);
                    if (_bothAtEntry) _owner._bothTimes[(int)_stage].Add(ticks);
                }
            }
        }

        private struct Elapsed
        {
            internal long Count, Ticks, Max;
            internal void Add(long ticks)
            {
                Count++;
                Ticks += ticks;
                Max = Math.Max(Max, ticks);
            }
            internal string Text()
            {
                double ms = 1000d / Stopwatch.Frequency;
                return Count + "|" + N(Ticks * ms) + "|" + Mean(Ticks * ms, Count) + "|" +
                    (Count == 0 ? "NA" : N(Max * ms));
            }
        }

        // Bracket consecutive actual source samples. Both-active requires both endpoints and
        // every observed Fixed boundary in between to be active, including zero-Fixed render intervals.
        private struct FrameIntervals
        {
            internal long Count, Over33, Over50, Over100, Over250;
            internal double Total, Max;
            internal void Add(double seconds)
            {
                Count++;
                Total += seconds;
                Max = Math.Max(Max, seconds);
                if (seconds > 1d/30d) Over33++;
                if (seconds > .05d) Over50++;
                if (seconds > .1d) Over100++;
                if (seconds > .25d) Over250++;
            }
            internal string Text() => Count + "|" + N(Total) + "|" + (Total > 0d ? N(Count / Total) : "NA") +
                "|" + Mean(Total * 1000d, Count) + "|" + (Count == 0 ? "NA" : N(Max * 1000d)) +
                "|" + Over33 + "|" + Over50 + "|" + Over100 + "|" + Over250;
        }

        private readonly Elapsed[] _times = new Elapsed[Enum.GetValues(typeof(Stage)).Length];
        private readonly Elapsed[] _bothTimes = new Elapsed[(int)Stage.SourceLegs + 1];
        private FrameIntervals _all, _both, _other;
        private SourceStepContext _previous;
        private int _generation, _sourceBreaks;
        private bool _bothThroughout, _wasCompact, _withHistory, _withEarlyProbe;
        internal bool Running { get; private set; }
        internal bool Compact { get; private set; }
        internal bool History => Running && _withHistory;
        internal readonly SourceRotationCoverage SourceRotation = new SourceRotationCoverage();
        internal bool EarlyProbe => Running && _withEarlyProbe;
        internal bool LeftReleaseProbe => Running && _withLeftRelease;
        private bool _withLeftRelease;
        private BilateralFootTransitionTrace _transitions;
        internal BilateralFootTransitionTrace Transitions => Running ? _transitions : null;
        internal static string ModeName(bool compact) => compact ? "COMPACT" : "FULL";

        internal void Begin(string arm)
        {
            SourceRotation.Reset();
            _generation++;
            Running = PelvisDynamicReceiverIsolation.IsDynamicHipArm(arm);
            Compact = _wasCompact = Running;
            _withHistory = Running;
            _withLeftRelease = Running;
            _transitions = _withLeftRelease ? new BilateralFootTransitionTrace() : null;
            // Both v154 arms retain the same event windows and the same observation cost.
            _withEarlyProbe = false;
            Array.Clear(_times, 0, _times.Length);
            Array.Clear(_bothTimes, 0, _bothTimes.Length);
            _all = _both = _other = default;
            _previous = default;
            _bothThroughout = false;
            _sourceBreaks = 0;
        }

        internal Measurement Measure(Stage stage, bool bothAtCallbackEntry = false) =>
            Running ? new Measurement(this, stage, bothAtCallbackEntry && (int)stage < _bothTimes.Length) : default;

        internal void FixedBoundary(bool bothActive)
        {
            if (Running && !bothActive) _bothThroughout = false;
        }

        internal void Source(SourceStepContext source, bool bothActive)
        {
            if (!Running) return;
            // A repeated delivery has no elapsed interval and must not erase a single-support
            // boundary already observed between the two distinct render samples.
            if (_previous.Matches(source)) { _bothThroughout &= bothActive; return; }
            if (_previous.Valid && source.Valid && _previous.Epoch == source.Epoch &&
                source.Sequence == _previous.Sequence + 1 && source.Time >= _previous.Time)
            {
                double seconds = (double)source.Time - _previous.Time;
                _all.Add(seconds);
                if (_bothThroughout && bothActive) _both.Add(seconds);
                else _other.Add(seconds);
            }
            else if (_previous.Valid || !source.Valid) _sourceBreaks++;
            _previous = source;
            _bothThroughout = bothActive;
        }

        // Disable before quiesce or report formatting. A span crossing start/stop never enters another run.
        internal bool Seal()
        {
            if (!Running) return false;
            Running = Compact = false;
            _transitions?.Seal();
            return true;
        }

        internal string Summary()
        {
            var sb = new StringBuilder("\nLANDING_COST_V041430/mode=").Append(ModeName(_wasCompact))
                .Append("/eventHistory=").Append(_withHistory).Append("/earlyLandingProbe=").Append(_withEarlyProbe)
                .Append("/leftReleaseProbe=").Append(_withLeftRelease)
                .Append("/parentHeightProbe=").Append(_withEarlyProbe).Append("/rightAnkleProbe=").Append(_withEarlyProbe)
                .Append("/rightKneePivot=").Append(_withEarlyProbe)
                .Append("/hipPreparation=V142A/clock=MONOTONIC_WALL/inclusiveNested=True/partialBoundarySpansExcluded=True")
                .Append("/scope=ARL_MAIN_THREAD_CALLBACKS_EXCLUDES_SOLVER_RENDERER_AND_STOP_FORMATTING");
            foreach (Stage stage in Enum.GetValues(typeof(Stage)))
            {
                sb.Append("\nLANDING_COST_STAGE_V041430/stage=").Append(stage)
                    .Append("/calls-totalMs-meanMs-maxMs=").Append(_times[(int)stage].Text());
                if ((int)stage < _bothTimes.Length)
                    sb.Append("/bothActiveAtCallbackEntry=").Append(_bothTimes[(int)stage].Text());
            }
            sb.Append("\nLANDING_SOURCE_INTERVAL_V041430/sourceBreaks=").Append(_sourceBreaks)
                .Append("/measure=count-seconds-Hz-meanMs-maxMs-gt33_33-gt50-gt100-gt250")
                .Append("/all=").Append(_all.Text()).Append("/bothActiveThroughout=").Append(_both.Text())
                .Append("/other=").Append(_other.Text());
            if (_transitions != null) sb.Append(_transitions.Summary());
            SourceRotation.Append(sb);
            return sb.ToString();
        }
        private static string Mean(double sum, long count) => count == 0 ? "NA" : N(sum / count);
        private static string N(double n) => n.ToString("G9", CultureInfo.InvariantCulture);
    }
}
