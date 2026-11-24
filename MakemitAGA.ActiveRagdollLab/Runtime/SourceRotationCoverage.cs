using System.Globalization;
using System.Text;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Run-length evidence on the raw Render clock. A range [a,b] classifies the
    // increments ending at a..b. A missing range is unknown, including overflow.
    internal sealed class SourceRotationCoverage
    {
        internal enum Kind { Unknown, Continuous, Jump }
        private struct Span
        {
            internal Kind Kind;
            internal int Epoch, First, Last, PeakAt;
            internal float Peak, PeakDt, PeakResidual, Time, Threshold;
            internal Vector3 Axis;
        }

        internal const int Capacity = 256;
        internal const float HistorySeconds = .25f;
        internal const float TrendResidualFraction = .5f;
        private float _knownSince, _lastJump, _threshold;
        private Kind _window;
        private readonly Span[] _spans = new Span[Capacity];
        private SourceStepContext _previous;
        private Quaternion _rotation, _olderRotation;
        private bool _hasIncrement;
        private float _previousDt;
        private int _smoothOverBudget;
        private bool _hasPose, _full, _break;
        private int _used, _omitted, _continuous, _jump, _unknown;

        internal void Reset()
        {
            _previous=default; _hasPose=_full=_break=_hasIncrement=false; _smoothOverBudget=0;
            _used=_omitted=_continuous=_jump=_unknown=0;
            _window=Kind.Unknown; _lastJump=float.NegativeInfinity; _threshold=float.NaN;
        }

        internal void Observe(in SourceStepContext sample, Quaternion pose, float threshold)
        {
            if (sample.Matches(_previous)) return;
            bool valid=sample.Valid && FiniteRotation(pose);
            float dt=sample.Time-_previous.Time;
            var kind=Kind.Unknown;
            float angle=0f, residual=0f;
            bool measured=false;
            if (valid && _hasPose && sample.Epoch==_previous.Epoch && sample.Sequence==_previous.Sequence+1 &&
                dt>=.001f && dt<=.0501f && sample.Dt>=.001f && sample.Dt<=.0501f &&
                float.IsFinite(threshold) && threshold>0f && threshold<=180f)
            {
                angle=Quaternion.Angle(_rotation,pose);
                if (float.IsFinite(angle))
                {
                    measured = true;
                    bool exceedsSpeed = angle * Mathf.Deg2Rad > Physics.PelvisAngularReferencePlanner.MaxW * dt;
                    if (angle < threshold || !exceedsSpeed) kind = Kind.Continuous;
                    else if (_hasIncrement)
                    {
                        // At 30 Hz a smooth turn spans twice the angle it does at 60 Hz.
                        // Use the prior world-space increment, scaled by actual elapsed time.
                        var prior = _rotation * Quaternion.Inverse(_olderRotation);
                        var axis = ShortArcVector(prior);
                        float length = axis.magnitude;
                        var expected = _rotation;
                        if (length > .000001f)
                        {
                            float previousAngle = 2f * Mathf.Atan2(length,Mathf.Abs(prior.w)) * Mathf.Rad2Deg;
                            expected = Quaternion.AngleAxis(previousAngle * dt / _previousDt, axis / length) * _rotation;
                        }
                        residual = Quaternion.Angle(expected,pose);
                        if (float.IsFinite(residual))
                        {
                            kind = residual >= threshold * TrendResidualFraction ? Kind.Jump : Kind.Continuous;
                            if (kind == Kind.Continuous) _smoothOverBudget++;
                        }
                    }
                }
            }
            if (kind==Kind.Unknown || threshold!=_threshold)
            {
                _knownSince=sample.Time;
                _lastJump=float.NegativeInfinity;
            }
            if (kind==Kind.Jump) _lastJump=sample.Time;
            _window=kind==Kind.Unknown || sample.Time-_knownSince < HistorySeconds ? Kind.Unknown :
                sample.Time-_lastJump <= HistorySeconds ? Kind.Jump : Kind.Continuous;
            _hasIncrement=measured; _previousDt=dt; _olderRotation=_rotation;
            _previous=sample; _rotation=pose; _hasPose=valid; _threshold=threshold;
            if (kind==Kind.Jump) _jump++;
            else if (kind==Kind.Continuous) _continuous++;
            else _unknown++;
            if (!sample.Valid) { _omitted++; _break=true; return; }
            if (kind != Kind.Jump && _used>0 && !_full && !_break)
            {
                ref var last=ref _spans[_used-1];
                if (last.Epoch==sample.Epoch && last.Last+1==sample.Sequence && last.Kind==kind && last.Threshold==threshold)
                {
                    last.Last=sample.Sequence;
                    if (angle>last.Peak) { last.Peak=angle; last.PeakAt=sample.Sequence; last.PeakDt=dt; last.PeakResidual=residual; }
                    return;
                }
            }
            if (_full || _used==Capacity) { _omitted++; _full=true; return; }
            _break=false;
            Vector3 jumpAxis = Vector3.zero;
            if (kind == Kind.Jump)
            {
                var increment = pose * Quaternion.Inverse(_olderRotation);
                jumpAxis = ShortArcVector(increment).normalized;
            }
            _spans[_used++]=new Span { Epoch=sample.Epoch, First=sample.Sequence, Last=sample.Sequence, Kind=kind,
                Threshold=threshold, Peak=angle, PeakAt=sample.Sequence, PeakDt=dt, PeakResidual=residual, Time=sample.Time, Axis=jumpAxis };
        }

        // Captured at the first ordinary/missed decision before its recovery or final outcome.
        internal Kind AtOpportunity(in SourceStepContext source, in PhysicsStepContext step, float age)
        {
            if (_full || !source.Matches(_previous) || !step.Valid || step.Epoch != source.Epoch ||
                step.SourceSequence != source.Sequence || !float.IsFinite(age) || age < 0f ||
                age > Physics.FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds) return Kind.Unknown;
            // Fixed can consume the same pose several times. Aging a verified jump uses
            // elapsed seconds and does not extend the window or create another source sample.
            if (_window == Kind.Jump && source.Time - _lastJump + age > HistorySeconds) return Kind.Continuous;
            return _window;
        }

        private static Vector3 ShortArcVector(Quaternion q)
        {
            // q and -q describe the same rotation. At exactly 180 degrees use the
            // angular planner's deterministic axis tie, including for time extrapolation.
            var vector = new Vector3(q.x,q.y,q.z);
            bool flip = q.w < 0f || (q.w == 0f && (q.x < 0f ||
                (q.x == 0f && (q.y < 0f || (q.y == 0f && q.z < 0f)))));
            return flip ? -vector : vector;
        }

        private static bool FiniteRotation(Quaternion q)
        {
            float n=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
            return float.IsFinite(n) && Mathf.Abs(n-1f)<.002f;
        }

        internal void Append(StringBuilder b)
        {
            b.Append("\nSOURCE_ROTATION_COVERAGE_V041800/continuous-jump-unknown-omitted=").Append(_continuous).Append('|')
                .Append(_jump).Append('|').Append(_unknown).Append('|').Append(_omitted)
                .Append("/referenceSpeedLimitRadSec=").Append(Number(Physics.PelvisAngularReferencePlanner.MaxW))
                .Append("/trendResidualFraction=").Append(Number(TrendResidualFraction)).Append("/smoothOverBudget=").Append(_smoothOverBudget);
            for (int i=0;i<_used;i++)
            {
                var s=_spans[i];
                b.Append("\nS180/epoch-first-last=").Append(s.Epoch).Append('|').Append(s.First).Append('|').Append(s.Last)
                    .Append("/kind=").Append(s.Kind).Append("/thresholdDeg=").Append(Number(s.Threshold));
                if (s.Kind!=Kind.Unknown) b.Append("/peakAt-deg-dt=").Append(s.PeakAt).Append('|')
                    .Append(Number(s.Peak)).Append('|').Append(Number(s.PeakDt));
                if (s.Kind == Kind.Jump) b.Append("/peakTrendResidualDeg=").Append(Number(s.PeakResidual))
                    .Append("/sourceTime=").Append(Number(s.Time)).Append("/worldAxis=")
                    .Append(Number(s.Axis.x)).Append('|').Append(Number(s.Axis.y)).Append('|').Append(Number(s.Axis.z));
            }
        }

        private static string Number(float x) => x.ToString("R",CultureInfo.InvariantCulture);
    }
}
