using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Render-clock input adapter for the existing FootRelativeReference planner. No actuator,
    // body teleport, collision authority or independent clock. A retains v113 B; B also selects partial improvement.
    // Geometry is evaluated at the actual Lower ankle, not at the animated Foot position.
    internal sealed class FootGroundReference
    {
        internal const float UprightConeDegrees = 30f;
        internal const float MaxCorrectionDegrees = 15f;
        internal const float GoalClearanceMeters = 0f;
        private const int SearchIterations = 8;
        private const int PartialSearchSamples = 16;
        internal const float MinImprovementMeters = 0.0001f; // Ignore sub-0.1 mm input noise, not a contact tolerance.
        internal struct GroundSample
        {
            internal SourceStepContext Source;
            internal bool Valid, Feasible, Partial, Selected, BoundValid, DerivedW;
            internal string Reason;
            internal float RawMin, FlatMin, GoalMin, BestUprightMin, HeightDeficit, CorrectionDegrees;
            internal Quaternion Goal;
        }
        private bool _enabled, _apply, _finite = true, _hasInput, _previousCorrected, _traceApply, _allowPartial, _tracePartial;
        private string _issue;
        private SourceStepContext _lastSource;
        private Quaternion _lastRaw, _lastGoal;
        private Vector3 _lastRawW, _lastGoalW;
        private GroundSample _current;
        private int _samples, _feasible, _selected, _partialSelected, _heightDeficit, _limited, _unavailable;
        private float _peakDeficit, _peakCorrection;
        internal bool Enabled => _enabled;
        internal bool Apply => _apply;
        internal bool AllowPartial => _allowPartial;
        internal bool Finite => _finite;
        internal string Issue => _issue;
        internal GroundSample Current => _current;

        internal void Select(bool enabled, bool apply, bool allowPartial = false)
        {
            if (apply && !enabled) throw new InvalidOperationException("GROUND_REFERENCE_APPLY_REQUIRES_ENABLED");
            if (allowPartial && !apply) throw new InvalidOperationException("GROUND_PARTIAL_REQUIRES_APPLY");
            _allowPartial = allowPartial;
            _enabled = enabled; _apply = apply; _finite = true; _issue = null;
            _hasInput = _previousCorrected = false; _lastSource = default; _current = default;
            ResetTrace();
        }
        internal void SeedInput(SourceStepContext source, Quaternion raw, Vector3 rawW)
        {
            if (!_enabled) return;
            if (!source.Valid || !Q(raw) || !V(rawW)) { Invalidate("SEED_INPUT_INVALID"); return; }
            _lastSource = source; _lastRaw = _lastGoal = raw; _lastRawW = _lastGoalW = rawW;
            _hasInput = true; _previousCorrected = false;
        }
        internal void DisablePreserveTrace()
        {
            _enabled = _apply = _allowPartial = _hasInput = _previousCorrected = false;
        }
        internal void ResetTrace()
        {
            _traceApply = _apply; _tracePartial = _allowPartial;
            _samples = _feasible = _selected = _partialSelected = _heightDeficit = _limited = _unavailable = 0;
            _peakDeficit = _peakCorrection = 0f;
        }
        internal bool TryHold(SourceStepContext source, Quaternion raw, Vector3 rawW, out Quaternion goal, out Vector3 goalW)
        {
            goal = raw; goalW = rawW;
            if (!_enabled || !_hasInput || !source.Matches(_lastSource)) return false;
            if (!Q(raw) || !V(rawW) || FootRelativeReference.Log(raw * Quaternion.Inverse(_lastRaw)).magnitude > 0.000001f ||
                (rawW - _lastRawW).sqrMagnitude > 0.000000000001f) Invalidate("HELD_RAW_PAYLOAD_CHANGED");
            goal = _lastGoal; goalW = _lastGoalW;
            return true;
        }
        internal void ObserveRender(bool trace, SourceStepContext source, Quaternion raw, Vector3 rawW, float dt,
            GroundSample candidate, out Quaternion goal, out Vector3 goalW)
        {
            goal = raw; goalW = rawW;
            if (!_enabled || !_finite) return;
            if (TryHold(source, raw, rawW, out goal, out goalW)) return;
            if (!source.Valid || !Q(raw) || !V(rawW) || !F(dt) || dt <= 0.0001f || !_hasInput ||
                source.Epoch != _lastSource.Epoch || source.Sequence <= _lastSource.Sequence || source.Time <= _lastSource.Time ||
                Mathf.Abs(source.Time - _lastSource.Time - dt) > 0.00001f)
            { Invalidate("SOURCE_INPUT_OR_CLOCK_INVALID"); return; }
            bool selected = _apply && candidate.Valid && (candidate.Feasible || (_allowPartial && candidate.Partial));
            if (selected) goal = candidate.Goal;
            // Validate before Log/Inverse: a malformed candidate must latch the contract,
            // never throw midway through input differentiation or replace held history.
            if (!Q(goal)) { Invalidate("GOAL_INPUT_NONFINITE"); goal = raw; return; }
            bool derivative = selected || _previousCorrected;
            if (derivative)
                goalW = dt < 0.25f ? FootRelativeReference.Log(goal * Quaternion.Inverse(_lastGoal)) / dt : Vector3.zero;
            if (!Q(goal) || !V(goalW)) { Invalidate("GOAL_INPUT_NONFINITE"); goal = raw; goalW = rawW; return; }
            candidate.Source = source; candidate.Selected = selected; candidate.DerivedW = derivative;
            _current = candidate;
            _lastSource = source; _lastRaw = raw; _lastRawW = rawW; _lastGoal = goal; _lastGoalW = goalW;
            _previousCorrected = selected;
            if (trace)
            {
                _samples++;
                if (candidate.Feasible) _feasible++;
                if (selected) _selected++;
                if (selected && candidate.Partial) _partialSelected++;
                if (!candidate.Valid) _unavailable++;
                else if (candidate.HeightDeficit > 0f) _heightDeficit++;
                else if (!candidate.Feasible && candidate.RawMin < GoalClearanceMeters) _limited++;
                _peakDeficit = Mathf.Max(_peakDeficit, candidate.HeightDeficit);
                _peakCorrection = Mathf.Max(_peakCorrection, candidate.CorrectionDegrees);
            }
        }
        internal void Invalidate(string issue) { _finite = false; _issue = issue; }

        // Full Box support function. center/half are scaled local offsets FROM the ankle.
        // Equivalent to all eight vertices, including a nonzero collider center and ankle anchor.
        internal static float Minimum(Vector3 ankle, Quaternion rotation, Vector3 center, Vector3 half)
        {
            Vector3 x = rotation * Vector3.right, y = rotation * Vector3.up, z = rotation * Vector3.forward;
            return (ankle + rotation * center).y - Mathf.Abs(x.y) * half.x - Mathf.Abs(y.y) * half.y - Mathf.Abs(z.y) * half.z;
        }
        private static bool Inside(Vector3 ankle, Quaternion rotation, Vector3 center, Vector3 half, Vector3 floorCenter, Vector3 floorHalf)
        {
            Vector3 p = ankle + rotation * center;
            Vector3 x = rotation * Vector3.right, y = rotation * Vector3.up, z = rotation * Vector3.forward;
            float ex = Mathf.Abs(x.x) * half.x + Mathf.Abs(y.x) * half.y + Mathf.Abs(z.x) * half.z;
            float ez = Mathf.Abs(x.z) * half.x + Mathf.Abs(y.z) * half.y + Mathf.Abs(z.z) * half.z;
            return Mathf.Abs(p.x - floorCenter.x) + ex <= floorHalf.x && Mathf.Abs(p.z - floorCenter.z) + ez <= floorHalf.z;
        }
        // Exact maximum minimum-height over ALL yaw and up vectors within this upright cone,
        // when the ankle projects inside the XZ footprint and lies above the entire local Box.
        // It is an optimistic bound BEFORE relative-ROM/correction limits. Negative proves a
        // height deficit in this cone; positive does NOT prove our narrower candidate is reachable.
        internal static bool UprightBound(Vector3 ankle, Vector3 center, Vector3 half, float top, out float best)
        {
            best = 0f;
            float bx = half.x - Mathf.Abs(center.x), bz = half.z - Mathf.Abs(center.z);
            if (bx < 0f || bz < 0f || center.y + half.y > 0f) return false;
            float a = half.y - center.y;
            float theta = UprightConeDegrees * Mathf.Deg2Rad;
            float depth = Mathf.Min(a, a * Mathf.Cos(theta) + Mathf.Min(bx, bz) * Mathf.Sin(theta));
            best = ankle.y - top - depth;
            return F(best);
        }
        private static bool WithinRom(Quaternion relative, Quaternion bind)
        {
            Vector3 d = FootRelativeReference.Log(Quaternion.Inverse(bind) * relative) * Mathf.Rad2Deg;
            return V(d) && Mathf.Abs(d.x) <= FootAnkleSourceDrivenRelativePdIsolation.FlexionLimitDegrees &&
                Mathf.Abs(d.y) <= FootAnkleSourceDrivenRelativePdIsolation.AxialLimitDegrees &&
                Mathf.Abs(d.z) <= FootAnkleSourceDrivenRelativePdIsolation.InversionLimitDegrees;
        }
        private static Quaternion Blend(Quaternion from, Quaternion to, float fraction) =>
            FootRelativeReference.Advance(from, FootRelativeReference.Log(to * Quaternion.Inverse(from)), fraction);

        private static GroundSample EvaluateClear(Vector3 ankle, Quaternion lower, Quaternion rawRelative, Quaternion bind,
            Vector3 center, Vector3 half, Vector3 floorCenter, Vector3 floorHalf)
        {
            GroundSample s = new GroundSample { Goal = rawRelative, Reason = "INPUT_INVALID" };
            if (!V(ankle) || !Q(lower) || !Q(rawRelative) || !Q(bind) || !V(center) || !V(half) ||
                half.x <= 0f || half.y <= 0f || half.z <= 0f || !V(floorCenter) || !V(floorHalf) ||
                floorHalf.x <= 0f || floorHalf.y <= 0f || floorHalf.z <= 0f) return s;
            float top = floorCenter.y + floorHalf.y;
            Quaternion rawWorld = lower * rawRelative;
            s.RawMin = Minimum(ankle, rawWorld, center, half) - top;
            s.GoalMin = s.RawMin;
            s.FlatMin = ankle.y + center.y - half.y - top;
            s.Valid = F(s.RawMin) && F(s.FlatMin);
            if (!s.Valid) return s;
            s.BoundValid = UprightBound(ankle, center, half, top, out s.BestUprightMin);
            s.HeightDeficit = s.BoundValid ? Mathf.Max(0f, GoalClearanceMeters - s.BestUprightMin) : 0f;
            if (!Inside(ankle, rawWorld, center, half, floorCenter, floorHalf)) { s.Reason = "FOOTPRINT_OUTSIDE"; return s; }
            if (s.RawMin >= GoalClearanceMeters) { s.Reason = "RAW_TARGET_CLEAR"; return s; }
            if (s.HeightDeficit > 0f) { s.Reason = "ANKLE_HEIGHT_DEFICIT_IN_UPRIGHT_CONE"; return s; }
            // Never claim whole-ankle infeasibility from this one candidate when the bound does not prove it.
            Vector3 forward = rawWorld * Vector3.forward; forward.y = 0f;
            if (forward.sqrMagnitude < 0.0001f) { s.Reason = "HEADING_DEGENERATE"; return s; }
            Quaternion flat = Quaternion.LookRotation(forward.normalized, Vector3.up);
            float angle = FootRelativeReference.Log(flat * Quaternion.Inverse(rawWorld)).magnitude * Mathf.Rad2Deg;
            float hi = angle > MaxCorrectionDegrees ? MaxCorrectionDegrees / angle : 1f;
            Quaternion candidate = Blend(rawWorld, flat, hi);
            float min = Minimum(ankle, candidate, center, half) - top;
            if (min < GoalClearanceMeters) { s.Reason = "NO_CLEAR_GOAL_WITHIN_CORRECTION_LIMIT"; return s; }
            float lo = 0f;
            for (int i = 0; i < SearchIterations; i++)
            {
                float middle = (lo + hi) * 0.5f;
                Quaternion q = Blend(rawWorld, flat, middle);
                if (Minimum(ankle, q, center, half) - top >= GoalClearanceMeters) hi = middle;
                else lo = middle;
            }
            candidate = Blend(rawWorld, flat, hi);
            Quaternion relative = Quaternion.Inverse(lower) * candidate;
            s.GoalMin = Minimum(ankle, candidate, center, half) - top;
            s.CorrectionDegrees = FootRelativeReference.Log(candidate * Quaternion.Inverse(rawWorld)).magnitude * Mathf.Rad2Deg;
            if (!Q(relative) || !F(s.GoalMin) || s.GoalMin < GoalClearanceMeters || s.CorrectionDegrees > MaxCorrectionDegrees + 0.001f ||
                Vector3.Dot(candidate * Vector3.up, Vector3.up) < Mathf.Cos(UprightConeDegrees * Mathf.Deg2Rad) ||
                !Inside(ankle, candidate, center, half, floorCenter, floorHalf)) { s.Reason = "CANDIDATE_OUTSIDE_BOUNDS"; return s; }
            if (!WithinRom(relative, bind)) { s.Reason = "CANDIDATE_OUTSIDE_RELATIVE_ROM"; return s; }
            s.Feasible = true; s.Goal = relative; s.Reason = "FEASIBLE_GOAL";
            return s;
        }
        // Preserve the complete v113 feasible-only path bit-for-bit. Only B continues after
        // it rejects a goal. Search a bounded set on the SAME yaw-preserving flattening arc;
        // no global optimality, clearance guarantee or contact permission is inferred.
        internal static GroundSample Evaluate(Vector3 ankle, Quaternion lower, Quaternion rawRelative, Quaternion bind,
            Vector3 center, Vector3 half, Vector3 floorCenter, Vector3 floorHalf, bool allowPartial = false)
        {
            GroundSample s = EvaluateClear(ankle, lower, rawRelative, bind, center, half, floorCenter, floorHalf);
            if (!allowPartial || !s.Valid || s.Feasible || s.RawMin >= GoalClearanceMeters || s.Reason == "FOOTPRINT_OUTSIDE") return s;
            Quaternion rawWorld = lower * rawRelative;
            Vector3 forward = rawWorld * Vector3.forward; forward.y = 0f;
            if (forward.sqrMagnitude < 0.0001f) return s;
            Quaternion flat = Quaternion.LookRotation(forward.normalized, Vector3.up);
            float angle = FootRelativeReference.Log(flat * Quaternion.Inverse(rawWorld)).magnitude * Mathf.Rad2Deg;
            float end = angle > MaxCorrectionDegrees ? MaxCorrectionDegrees / angle : 1f;
            float bestMin = s.RawMin;
            // Start from the exact raw fallback even if the clear-only search inspected a rejected candidate.
            s.Goal = rawRelative; s.GoalMin = s.RawMin; s.CorrectionDegrees = 0f;
            for (int i = 1; i <= PartialSearchSamples; i++)
            {
                Quaternion world = Blend(rawWorld, flat, end * i / PartialSearchSamples);
                Quaternion relative = Quaternion.Inverse(lower) * world;
                float min = Minimum(ankle, world, center, half) - (floorCenter.y + floorHalf.y);
                float correction = FootRelativeReference.Log(world * Quaternion.Inverse(rawWorld)).magnitude * Mathf.Rad2Deg;
                if (!Q(relative) || !F(min) || !F(correction) || min <= bestMin || min - s.RawMin < MinImprovementMeters ||
                    correction > MaxCorrectionDegrees + 0.001f ||
                    Vector3.Dot(world * Vector3.up, Vector3.up) < Mathf.Cos(UprightConeDegrees * Mathf.Deg2Rad) ||
                    !WithinRom(relative, bind) || !Inside(ankle, world, center, half, floorCenter, floorHalf)) continue;
                bestMin = min; s.Goal = relative; s.GoalMin = min; s.CorrectionDegrees = correction;
                s.Feasible = min >= GoalClearanceMeters; s.Partial = !s.Feasible;
            }
            if (s.Feasible) s.Reason = "FEASIBLE_BOUNDED_PATH_SAMPLE";
            else if (s.Partial) s.Reason = "PARTIAL_PENETRATION_REDUCTION";
            return s;
        }

        internal static string SampleText(GroundSample s)
        {
            if (!s.Source.Valid) return "landingGoalV041140=NA";
            return "landingGoalV041140=RENDER_INPUT_ONLY/sourceSeq=" + s.Source.Sequence + "/sourceTime=" + S(s.Source.Time) +
                "/valid-feasible-partial-selected-derivedW=" + s.Valid + "-" + s.Feasible + "-" + s.Partial + "-" + s.Selected + "-" + s.DerivedW +
                "/reason=" + (s.Reason ?? "<none>") + "/raw-flat-candidateMinMm=" + S(s.RawMin * 1000f) + "|" + S(s.FlatMin * 1000f) + "|" + S(s.GoalMin * 1000f) +
                "/boundValid-bestUprightMin-deficitMm=" + s.BoundValid + "|" + S(s.BestUprightMin * 1000f) + "|" + S(s.HeightDeficit * 1000f) +
                "/correctionDeg=" + S(s.CorrectionDegrees) +
                "/candidateGain-residualMm=" + S((s.GoalMin - s.RawMin) * 1000f) + "|" + S(Mathf.Max(0f, -s.GoalMin) * 1000f);
        }
        internal string GetDetails() => "FOOT_GROUND_REFERENCE_V041140/enabled-apply-finite=" + _enabled + "-" + _apply + "-" + _finite +
            "/allowPartial=" + _allowPartial + "/tracePartial=" + _tracePartial + "/traceApply=" + _traceApply + "/issue=" + (_issue ?? "<none>") + "/render(feasible-selected-heightDeficit-limited-unavailable)=" +
            _samples + "|" + _feasible + "|" + _selected + "|" + _heightDeficit + "|" + _limited + "|" + _unavailable +
            "/partialSelected=" + _partialSelected + "/partialSearchSamples-minGainMm=" + PartialSearchSamples + "|" + S(MinImprovementMeters * 1000f) +
            "/peak(deficitMm-correctionDeg)=" + S(_peakDeficit * 1000f) + "|" + S(_peakCorrection) +
            "/limits(uprightCone-correction-clearanceMm)=" + S(UprightConeDegrees) + "|" + S(MaxCorrectionDegrees) + "|" + S(GoalClearanceMeters * 1000f) +
            "/planner=EXISTING_RENDER_FOOT_REFERENCE_W3.75_A20/plannerGap=SELECTED_INPUT_TO_REFERENCE/noPoseWrite=True/contactAuthority=False/" + SampleText(_current);
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < 0.01f;
        private static string S(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
    }
}
