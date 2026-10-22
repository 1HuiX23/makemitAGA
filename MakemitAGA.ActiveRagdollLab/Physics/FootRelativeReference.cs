using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Candidate, not production: V04720 trajectory with V04730 explicit long-gap recovery.
    // Raw animation/conditioner history remains owned by Foundation. No body, force, clock or observer loop here.
    internal sealed class FootRelativeReference
    {
        internal const string ArmA = "RAW_FOOT_REFERENCE_V04730", ArmB = "COHERENT_FOOT_REFERENCE_V04730";
        internal const float MaxA = FootAnkleSourceDrivenRelativePdIsolation.MaxTargetAccelerationFeedforward;
        // Engineering bounds from the unchanged controller, NOT certified human ankle limits.
        // MaxW is the speed whose zero-target D term alone spends MaxRelA; Kp/Kd is its slow tracking rate.
        internal const float MaxW = FootAnkleSourceDrivenRelativePdIsolation.MaxRelativeAngularAcceleration / FootAnkleSourceDrivenRelativePdIsolation.Kd;
        internal const float ArrivalRate = FootAnkleSourceDrivenRelativePdIsolation.Kp / FootAnkleSourceDrivenRelativePdIsolation.Kd;
        // A long source interval is NOT an interval over which the held reference was integrated.
        // Discard that missing time; permit one small planner step, never a catch-up loop or pose/W reset.
        // This is a reference-time bound, not the physics dt and not a human-motion limit.
        internal const float RecoveryMaxDt = 1f / 60f;

        internal struct Sample
        {
            internal SourceStepContext Source;
            internal Quaternion Rotation;
            internal Vector3 W, A, PreviousW;
            internal float Dt, RawGap, SourceElapsed, DiscardedElapsed;
            internal bool AlphaLimited, SpeedLimited, InheritedOverSpeed, GapRecovery;
        }

        private bool _selected, _active, _finite = true;
        private string _issue;
        private string _traceArm = ArmA;
        private Sample _current;
        private Quaternion _inputRotation;
        private Vector3 _inputW;
        private int _renderSamples, _alphaLimited, _speedLimited, _inheritedOverSpeed;
        private float _peakGap, _peakW, _peakA;
        private int _gapRecoveries;
        private float _peakSourceGap;
        private double _discardedSeconds;
        private SourceStepContext _lastInput;
        private float _lastSourceDt, _lastPlanDt;
        internal bool Selected => _selected;
        internal bool Active => _active;
        internal bool Finite => _finite;
        internal string Issue => _issue;
        internal Sample Current => _current;
        internal string Arm => _selected ? ArmB : ArmA;

        internal void Select(bool enabled)
        {
            _selected = enabled; _active = false; _finite = true; _issue = null;
            _current = new Sample();
            _inputRotation = Quaternion.identity; _inputW = Vector3.zero;
            _lastInput = new SourceStepContext(); _lastSourceDt = _lastPlanDt = 0f;
            ResetTrace();
        }

        internal bool Seed(SourceStepContext source, Quaternion rotation, Vector3 w)
        {
            if (!_selected) return true;
            if (!source.Valid || !Q(rotation) || !V(w)) return Invalid("SEED_INPUT_INVALID");
            // Same pose/W as the existing physical-parent Foot initialization, without a first-step rebase.
            _current = new Sample { Source = source, Rotation = Normalize(rotation), W = w, PreviousW = w };
            _inputRotation = rotation; _inputW = w;
            _active = true;
            return true;
        }

        internal void ObserveRender(bool trace, SourceStepContext source, Quaternion rawRotation, Vector3 rawW, float dt)
        {
            if (!_selected || !_active || !_finite) return;
            _lastInput = source; _lastSourceDt = dt; _lastPlanDt = 0f;
            if (!source.Valid || !Q(rawRotation) || !V(rawW)) { Invalid("SOURCE_INPUT_INVALID"); return; }
            if (source.Matches(_current.Source))
            {
                if (Log(rawRotation * Quaternion.Inverse(_inputRotation)).magnitude > 0.000001f || (rawW - _inputW).sqrMagnitude > 0.000000000001f)
                    Invalid("HELD_SOURCE_PAYLOAD_CHANGED");
                return; // An exact duplicate is held, never another integration step.
            }
            if (source.Epoch != _current.Source.Epoch) { Invalid("SOURCE_EPOCH_CHANGED"); return; }
            if (source.Sequence <= _current.Source.Sequence) { Invalid("SOURCE_SEQUENCE_NOT_ADVANCING"); return; }
            if (!F(dt) || dt <= 0.0001f) { Invalid("SOURCE_DT_NONFINITE_OR_NOT_ADVANCING"); return; }
            if (source.Time <= _current.Source.Time) { Invalid("SOURCE_TIME_NOT_ADVANCING"); return; }
            if (Mathf.Abs((source.Time - _current.Source.Time) - dt) > 0.00001f)
            { Invalid("SOURCE_DT_MISMATCH"); return; }
            try
            {
                bool recovery = dt >= 0.25f;
                float planDt = recovery ? Mathf.Min(RecoveryMaxDt, _current.Dt > 0.0001f ? _current.Dt : RecoveryMaxDt) : dt;
                // Foundation has already invalidated raw W/A across this gap. Keep the previous
                // reference pose/W and approach the new pose with a zero source-speed hint.
                // The following ordinary Render packet uses fresh raw history again.
                Sample next = Build(_current.Rotation, _current.W, rawRotation, recovery ? Vector3.zero : rawW, planDt);
                next.Source = source;
                next.SourceElapsed = dt; next.GapRecovery = recovery;
                next.DiscardedElapsed = recovery ? dt - planDt : 0f;
                _lastPlanDt = planDt;
                _current = next;
                _inputRotation = rawRotation; _inputW = rawW;
                if (trace)
                {
                    _renderSamples++;
                    if (next.AlphaLimited) _alphaLimited++;
                    if (next.SpeedLimited) _speedLimited++;
                    if (next.InheritedOverSpeed) _inheritedOverSpeed++;
                    _peakGap = Mathf.Max(_peakGap, next.RawGap);
                    _peakW = Mathf.Max(_peakW, next.W.magnitude);
                    _peakA = Mathf.Max(_peakA, next.A.magnitude);
                    if (recovery)
                    {
                        _gapRecoveries++; _peakSourceGap = Mathf.Max(_peakSourceGap, dt);
                        _discardedSeconds += next.DiscardedElapsed;
                    }
                }
            }
            catch { Invalid("REFERENCE_PLAN_INVALID"); }
        }

        internal static Sample Build(Quaternion rotation, Vector3 w, Quaternion rawRotation, Vector3 rawW, float dt)
        {
            if (!Q(rotation) || !Q(rawRotation) || !V(w) || !V(rawW) || !F(dt) || dt <= 0.0001f || dt >= 0.25f)
                throw new InvalidOperationException("FOOT_REFERENCE_INPUT_INVALID");
            // rawW describes the incoming Render interval, not an extrapolated next source pose.
            // Compare the raw endpoint against the old reference advanced over that same interval.
            Quaternion origin = Advance(rotation, rawW, dt);
            Vector3 e = Log(rawRotation * Quaternion.Inverse(origin));
            Vector3 relativeW = w - rawW;
            Vector3 stoppingOffset = relativeW * (relativeW.magnitude / (2f * MaxA) + dt);
            Vector3 desiredW = rawW + (e - stoppingOffset) * ArrivalRate;
            Vector3 limitedW = Vector3.ClampMagnitude(desiredW, MaxW);
            Vector3 delta = limitedW - w;
            Vector3 nextW = w + Vector3.ClampMagnitude(delta, MaxA * dt);
            // Do not hard-clip inherited initial speed: that would violate the acceleration contract.
            Vector3 a = (nextW - w) / dt;
            if (!V(nextW) || !V(a) || nextW.magnitude > Mathf.Max(MaxW, w.magnitude) + 0.0001f ||
                a.magnitude > MaxA + 0.002f || nextW.magnitude * dt >= Mathf.PI - 0.0001f)
                throw new InvalidOperationException("FOOT_REFERENCE_BUDGET_INVALID");
            Quaternion nextRotation = Advance(rotation, nextW, dt);
            return new Sample { Rotation = nextRotation, W = nextW, A = a, PreviousW = w, Dt = dt,
                RawGap = Log(rawRotation * Quaternion.Inverse(nextRotation)).magnitude,
                AlphaLimited = delta.magnitude > MaxA * dt, SpeedLimited = desiredW.magnitude > MaxW,
                InheritedOverSpeed = w.magnitude > MaxW + 0.0001f };
        }

        internal void DisablePreserveTrace() { _selected = _active = false; }
        internal void ResetTrace()
        {
            _traceArm = Arm;
            _renderSamples = _alphaLimited = _speedLimited = _inheritedOverSpeed = 0;
            _peakGap = _peakW = _peakA = 0f;
            _gapRecoveries = 0; _peakSourceGap = 0f; _discardedSeconds = 0d;
        }
        internal string GetDetails() => "FOOT_REFERENCE_V04730/arm=" + Arm + "/traceArm=" + _traceArm + "/selected-active-finite=" +
            _selected + "/" + _active + "/" + _finite + "/issue=" + (_issue ?? "<none>") +
            "/source(epoch-sequence)=" + _current.Source.Epoch + "/" + _current.Source.Sequence +
            "/render(alphaLimited-speedLimited-inheritedOverSpeed)=" + _renderSamples + "/" + _alphaLimited + "/" + _speedLimited + "/" + _inheritedOverSpeed +
            "/peak(gapDeg-W-A)=" + S(_peakGap * Mathf.Rad2Deg) + "/" + S(_peakW) + "/" + S(_peakA) +
            "/gapRecovery(count-peakSourceSeconds-discardedSeconds)=" + _gapRecoveries + "/" + S(_peakSourceGap) + "/" + S((float)_discardedSeconds) +
            "/lastInput(epoch-seq-time-sourceDt-planDt)=" + _lastInput.Epoch + "/" + _lastInput.Sequence + "/" + S(_lastInput.Time) + "/" + S(_lastSourceDt) + "/" + S(_lastPlanDt) +
            "/limits(W-A-arrivalRate)=" + S(MaxW) + "/" + S(MaxA) + "/" + S(ArrivalRate) +
            "/traceFFInput=" + (_traceArm == ArmB ? "COHERENT_REFERENCE_DERIVATIVE_BOUND20" : "RAW_RENDER_PERSISTENCE_CONDITIONED_BOUND20") +
            "/domain=TRUE_RENDER_ONLY_FIXED_HOLD/closure=POSE_EXP_NEXT_W_PLAN_DT_AND_A_DELTA_W_PLAN_DT/gapPolicy=DISCARD_MISSING_TIME_ONE_STEP_MAX_1_OVER_60_NO_REBASE/stale=ORIGINAL_CONTROLLER_POLICY/noRunRebase=True";

        private bool Invalid(string issue) { _finite = false; _issue = issue; return false; }
        private static Quaternion Advance(Quaternion q, Vector3 w, float dt)
        {
            float speed = w.magnitude;
            return speed > 0f ? Normalize(Quaternion.AngleAxis(speed * dt * Mathf.Rad2Deg, w / speed) * q) : Normalize(q);
        }
        private static Vector3 Log(Quaternion q)
        {
            q = Normalize(q);
            if (q.w < 0f || (q.w == 0f && (q.x < 0f || (q.x == 0f && (q.y < 0f || (q.y == 0f && q.z < 0f))))))
                q = new Quaternion(-q.x, -q.y, -q.z, -q.w);
            Vector3 v = new Vector3(q.x, q.y, q.z);
            float m = v.magnitude;
            return m > 0f ? v * (2f * Mathf.Atan2(m, q.w) / m) : Vector3.zero;
        }
        private static Quaternion Normalize(Quaternion q)
        {
            float n = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!F(n) || n <= 0f) throw new InvalidOperationException("FOOT_REFERENCE_ROTATION_INVALID");
            return new Quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
        }
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) &&
            F(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) && q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.00000001f;
        private static string S(float v) => v.ToString("R", CultureInfo.InvariantCulture);
    }
}
