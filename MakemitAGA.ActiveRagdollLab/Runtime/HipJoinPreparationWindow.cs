using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Startup-only state; consumes shared clocks and stores two pose baselines, no ring,
    // native writes, controller reset or new update loop. Held packets never count as new samples.
    internal sealed class HipJoinPreparationWindow
    {
        internal const float HoldSeconds = FoundationPoseWindow.HoldSeconds;
        internal const float TimeoutSeconds = FoundationPoseWindow.TimeoutSeconds;
        private PhysicsStepContext _last, _qualifiedStep;
        private SourceStepContext _source;
        private HipJoinPose _left, _right, _firstLeft, _firstRight;
        private Vector3 _floorCenter, _floorHalf;
        private int _supportId, _samples, _windowSource, _observations;
        private float _requested, _elapsedFixed, _windowTime, _stableFixed, _worst = float.PositiveInfinity;
        private bool _haveFirst;
        private float _peakStableSeconds;
        private int _totalSourceSamples, _geometryBlockedSamples;
        internal bool Pending { get; private set; }
        internal bool Qualified { get; private set; }
        internal string Reason { get; private set; } = "NOT_REQUESTED";
        internal float Elapsed { get; private set; }
        internal float StableSeconds { get; private set; }
        internal HipJoinPose LastLeft { get; private set; }
        internal HipJoinPose LastRight { get; private set; }
        internal void Reset()
        {
            Pending = Qualified = false; Reason = "NOT_REQUESTED";
            _last = _qualifiedStep = default; _source = default; _supportId = _samples = _windowSource = _observations = 0;
            _requested = _elapsedFixed = _windowTime = _stableFixed = Elapsed = StableSeconds = 0f;
            _haveFirst = false; _worst = float.PositiveInfinity;
            _peakStableSeconds = 0f; _totalSourceSamples = _geometryBlockedSamples = 0;
            _left = _right = _firstLeft = _firstRight = LastLeft = LastRight = default;
            _floorCenter = _floorHalf = Vector3.zero;
        }
        internal void Begin(PhysicsStepContext step)
        {
            Reset();
            if (!step.Valid) throw new InvalidOperationException("HIP_JOIN_CLOCK_INVALID");
            _requested = step.UnscaledTime; Pending = true; Reason = "WAIT_STABLE_INITIAL_GEOMETRY";
        }
        internal void Cancel(string reason)
        {
            if (!Pending) return;
            Pending = false; Qualified = false; Reason = reason;
        }
        private void ResetHold(string reason)
        { _samples = _windowSource = 0; _stableFixed = StableSeconds = 0f; Reason = reason; }
        internal void Observe(PhysicsStepContext step, SourceStepContext source, int supportId,
            Vector3 floorCenter, Vector3 floorHalf, HipJoinPose left, HipJoinPose right, string unavailable)
        {
            if (!Pending) return;
            LastLeft = left; LastRight = right; _observations++;
            if (!step.Valid || !source.Valid || source.Epoch != step.Epoch || source.Sequence != step.SourceSequence ||
                (_last.Valid && (step.Epoch != _last.Epoch || step.Sequence != _last.Sequence + 1 ||
                    step.FixedTime < _last.FixedTime || step.UnscaledTime < _last.UnscaledTime)) ||
                (_source.Valid && (source.Epoch != _source.Epoch || source.Sequence < _source.Sequence || source.Time < _source.Time ||
                    (source.Sequence == _source.Sequence && !source.Matches(_source)))))
            { Cancel("CLOCK_OR_SOURCE_DISCONTINUITY"); return; }
            bool gap = _source.Valid && source.Sequence != _source.Sequence &&
                (source.Time <= _source.Time || source.Time - _source.Time > FoundationPoseWindow.MaxSampleGap);
            if (!_source.Valid || source.Sequence != _source.Sequence) _totalSourceSamples++;
            _last = step; _source = source; _elapsedFixed += step.Dt;
            Elapsed = Mathf.Max(step.UnscaledTime - _requested, _elapsedFixed);
            if (Elapsed >= TimeoutSeconds) { Cancel("TIMEOUT:" + Reason); return; }
            if (unavailable != null) { ResetHold(unavailable); return; }
            if (supportId == 0 || !PelvisParentFrame.V(floorCenter) || !PelvisParentFrame.V(floorHalf) ||
                floorHalf.x <= 0f || floorHalf.y <= 0f || floorHalf.z <= 0f)
            { ResetHold("SUPPORT_UNAVAILABLE"); return; }
            if (_supportId != 0 && !MatchesSupport(supportId, floorCenter, floorHalf))
            { Cancel("SUPPORT_CHANGED_DURING_PREPARATION"); return; }
            if (_supportId == 0) { _supportId = supportId; _floorCenter = floorCenter; _floorHalf = floorHalf; }
            if (!_haveFirst && left.Geometry.Valid && right.Geometry.Valid)
            { _firstLeft = left; _firstRight = right; _haveFirst = true; }
            if (left.Geometry.Valid && right.Geometry.Valid &&
                PelvisParentFrame.F(left.Geometry.ControlMin) && PelvisParentFrame.F(left.Geometry.ActualMin) &&
                PelvisParentFrame.F(right.Geometry.ControlMin) && PelvisParentFrame.F(right.Geometry.ActualMin))
                _worst = Mathf.Min(_worst, Mathf.Min(Mathf.Min(left.Geometry.ControlMin, left.Geometry.ActualMin),
                    Mathf.Min(right.Geometry.ControlMin, right.Geometry.ActualMin)));
            float age = Mathf.Max(0f, step.UnscaledTime - source.Time);
            if (age > InitialGeometryPreflight.MaxSourceAge || gap)
            { ResetHold("WAIT_CONTIGUOUS_FRESH_SOURCE"); return; }
            if (!left.Finite || !right.Finite) { ResetHold("CANDIDATE_POSE_INVALID"); return; }
            if (left.Geometry.PreparationIssue != null || right.Geometry.PreparationIssue != null)
            { _geometryBlockedSamples++; ResetHold("WAIT_BILATERAL_GEOMETRY"); return; }
            if (_samples > 0 && (!left.StableAgainst(_left) || !right.StableAgainst(_right))) ResetHold("WAIT_STABLE_POSE");
            if (_samples == 0)
            { _left = left; _right = right; _windowTime = source.Time; _samples = 1; _windowSource = source.Sequence; }
            else
            {
                _stableFixed += step.Dt;
                if (_windowSource != source.Sequence) { _samples++; _windowSource = source.Sequence; }
            }
            Reason = "WAIT_STABLE_INITIAL_GEOMETRY";
            StableSeconds = Mathf.Min(_stableFixed, source.Time - _windowTime);
            _peakStableSeconds = Mathf.Max(_peakStableSeconds, StableSeconds);
            if (_samples >= FoundationPoseWindow.MinimumSamples && StableSeconds >= HoldSeconds)
            { Pending = false; Qualified = true; _qualifiedStep = step; Reason = "STABLE_INITIAL_GEOMETRY"; }
        }
        internal bool QualifiedFor(PhysicsStepContext step) => Qualified && _qualifiedStep.Matches(step);
        internal bool MatchesSupport(int id, Vector3 center, Vector3 half) => id == _supportId && id != 0 &&
            PelvisParentFrame.V(center) && PelvisParentFrame.V(half) &&
            (center - _floorCenter).magnitude <= FootBoxGeometry.SupportPlaneToleranceMeters &&
            (half - _floorHalf).magnitude <= FootBoxGeometry.SupportPlaneToleranceMeters;
        private static string N(float v) => float.IsNaN(v) || float.IsInfinity(v) ? "NA" : v.ToString("G9", CultureInfo.InvariantCulture);
        private static string Geometry(HipJoinPose p) => !p.Geometry.Valid ? "NA" :
            N(p.Geometry.RawMin*1000f) + "," + N(p.Geometry.ControlMin*1000f) + "," + N(p.Geometry.ActualMin*1000f);
        internal string Summary() => "hipJoinPreparation=V041250_R2/pending-qualified=" + Pending + "|" + Qualified +
            "/reason=" + Reason + "/elapsed-stableSeconds=" + N(Elapsed) + "|" + N(StableSeconds) +
            "/observations-currentWindowSourceSamples=" + _observations + "|" + _samples + "/supportId=" + _supportId +
            "/totalSourceSamples-geometryBlocked-peakStableSeconds=" + _totalSourceSamples + "|" + _geometryBlockedSamples + "|" + N(_peakStableSeconds) +
            "/lastIssues=" + (LastLeft.Geometry.PreparationIssue ?? "PASS") + "|" + (LastRight.Geometry.PreparationIssue ?? "PASS") +
            "/routes=" + LastLeft.Geometry.PreparationRoute + "|" + LastRight.Geometry.PreparationRoute +
            "/firstProjectedMm(raw-control-body;L-R)=" + Geometry(_firstLeft) + ";" + Geometry(_firstRight) +
            "/lastProjectedMm(raw-control-body;L-R)=" + Geometry(LastLeft) + ";" + Geometry(LastRight) +
            "/worstProjectedMm=" + N(_worst*1000f) + "/limits(hold-timeout-sourceSamples)=1|15|12" +
            "/preparationMinMm=-3/domain=READ_ONLY_INITIALIZER_PROJECTION_NOT_CONTACT_PERMISSION";
    }
}
