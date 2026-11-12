using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Physics;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    internal struct HipContactPreparationSample
    {
        internal InitialGeometryPreflight Geometry;
        internal FootContactMotion Motion;
        internal bool NativeOff, IdentityQualified;
        internal string Issue => !NativeOff ? "PREPARATION_NATIVE_OR_CLOCK_CONTRACT" :
            Geometry.PreparationIssue ?? (!IdentityQualified ? "WAIT_CURRENT_FLOOR_IDENTITY" : Motion.QuietIssue);
    }

    // Second startup stage: settled, dynamic legs with contact/gravity/load selectors OFF.
    // It observes the existing Fixed loop and never enables contact, writes a pose or starts trace.
    // Runtime stance and load owners must qualify again after this preparation completes.
    internal sealed class HipContactPreparationWindow
    {
        internal const float HoldSeconds = FootContactGeometryLimits.RecoveryHoldSeconds;
        internal const float TimeoutSeconds = FoundationPoseWindow.TimeoutSeconds;
        private PhysicsStepContext _last, _qualifiedStep;
        private SourceStepContext _source;
        private float _requested, _elapsedFixed, _heldFixed, _firstSourceTime;
        private int _observations, _samples, _lastHoldSource;
        internal bool Pending { get; private set; }
        internal bool Qualified { get; private set; }
        internal string Reason { get; private set; } = "NOT_REQUESTED";
        internal float Elapsed { get; private set; }
        internal float StableSeconds { get; private set; }
        internal float PeakStableSeconds { get; private set; }
        internal HipContactPreparationSample Left { get; private set; }
        internal HipContactPreparationSample Right { get; private set; }
        internal void Reset()
        {
            _last = _qualifiedStep = default; _source = default;
            _requested = _elapsedFixed = _heldFixed = _firstSourceTime = Elapsed = StableSeconds = PeakStableSeconds = 0f;
            _observations = _samples = _lastHoldSource = 0;
            Pending = Qualified = false; Reason = "NOT_REQUESTED"; Left = Right = default;
        }
        internal void Begin(PhysicsStepContext joined)
        {
            if (!joined.Valid) throw new InvalidOperationException("HIP_CONTACT_PREPARATION_CLOCK_INVALID");
            Reset(); _last = joined; _requested = joined.UnscaledTime;
            Pending = true; Reason = "WAIT_SETTLED_NATIVE_GEOMETRY";
        }
        internal void Cancel(string reason)
        {
            if (!Pending) return;
            Pending = Qualified = false; Reason = reason;
        }
        private void ResetHold(string reason)
        { _heldFixed = StableSeconds = 0f; _samples = _lastHoldSource = 0; Reason = reason; }
        internal void Observe(PhysicsStepContext step, SourceStepContext source,
            HipContactPreparationSample left, HipContactPreparationSample right, string unavailable)
        {
            if (!Pending) return;
            Left = left; Right = right; _observations++;
            if (!step.Valid || !source.Valid || step.Epoch != _last.Epoch || step.Sequence != _last.Sequence + 1 ||
                source.Epoch != step.Epoch || source.Sequence != step.SourceSequence ||
                step.FixedTime < _last.FixedTime || step.UnscaledTime < _last.UnscaledTime ||
                (_source.Valid && (source.Sequence < _source.Sequence || source.Time < _source.Time ||
                    (source.Sequence == _source.Sequence && !source.Matches(_source)))))
            { Cancel("CLOCK_OR_SOURCE_DISCONTINUITY"); return; }
            bool gap = _source.Valid && source.Sequence != _source.Sequence &&
                (source.Time <= _source.Time || source.Time - _source.Time > FoundationPoseWindow.MaxSampleGap);
            _last = step; _source = source; _elapsedFixed += step.Dt;
            Elapsed = Math.Max(_elapsedFixed, step.UnscaledTime - _requested);
            if (Elapsed >= TimeoutSeconds) { Cancel("TIMEOUT:" + Reason); return; }
            if (unavailable != null) { ResetHold(unavailable); return; }
            if (gap || step.UnscaledTime - source.Time > InitialGeometryPreflight.MaxSourceAge)
            { ResetHold("WAIT_CONTIGUOUS_FRESH_SOURCE"); return; }
            if (left.Issue != null || right.Issue != null)
            { ResetHold("WAIT_BILATERAL_SETTLED_GEOMETRY"); return; }
            if (_samples == 0)
            { _firstSourceTime = source.Time; _samples = 1; _lastHoldSource = source.Sequence; }
            else
            {
                _heldFixed += step.Dt;
                if (_lastHoldSource != source.Sequence) { _lastHoldSource = source.Sequence; _samples++; }
            }
            StableSeconds = Math.Min(_heldFixed, source.Time - _firstSourceTime);
            PeakStableSeconds = Math.Max(PeakStableSeconds, StableSeconds);
            Reason = "WAIT_SETTLED_NATIVE_GEOMETRY";
            if (_samples >= 2 && StableSeconds + .000001f >= HoldSeconds)
            { Pending = false; Qualified = true; _qualifiedStep = step; Reason = "SETTLED_NATIVE_PREPARATION_READY"; }
        }
        internal bool QualifiedFor(PhysicsStepContext step) => Qualified && _qualifiedStep.Matches(step);
        private static string N(float v) => float.IsNaN(v) || float.IsInfinity(v) ? "NA" : v.ToString("G9", CultureInfo.InvariantCulture);
        private static string Sample(HipContactPreparationSample p) => !p.Geometry.Valid ? "NA" :
            "route=" + p.Geometry.PreparationRoute + ",actual-predictedMm=" + N(p.Geometry.ActualMin*1000f) + "," + N(p.Motion.PredictedMin*1000f) +
            ",corner-planar-angular=" + N(p.Motion.CornerNormalSpeed) + "," + N(p.Motion.PlanarSpeed) + "," + N(p.Motion.AngularSpeed);
        internal string Summary() => "hipContactPreparation=V041250_R2/pending-qualified=" + Pending + "|" + Qualified +
            "/reason=" + Reason + "/elapsed-stable-peakSeconds=" + N(Elapsed) + "|" + N(StableSeconds) + "|" + N(PeakStableSeconds) +
            "/observations-currentWindowSourceSamples=" + _observations + "|" + _samples +
            "/lastIssues=" + (Left.Issue ?? "PASS") + "|" + (Right.Issue ?? "PASS") + "/left={" + Sample(Left) + "}/right={" + Sample(Right) +
            "}/limits(hold-timeout)=0.15|15/domain=SETTLED_NATIVE_CONTACT_AND_LOAD_OFF_NOT_CONTACT_PERMISSION";
    }
}
