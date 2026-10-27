using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // V04800: both dynamic arms keep the V04780 Fixed WORLD trajectory; only A extrapolates sample age.
    // No body/time reads, source differentiation, new force owner, catch-up loop or observer ring.
    internal sealed class HipNormalReference
    {
        internal const string ArmOff = "RENDER_HELD_HIP_NORMAL_V04780";
        internal const string ArmA = "COHERENT_HIP_NORMAL_AGE_PREDICTED_V04800";
        internal const string ArmB = "COHERENT_HIP_NORMAL_NO_AGE_PREDICTION_V04800";
        private bool _selected, _seeded, _sampleAgePrediction, _traceSampleAgePrediction;
        private Quaternion _rotation = Quaternion.identity;
        private Vector3 _w;
        private PhysicsStepContext _last;
        private SourceStepContext _source;
        private int _steps, _applied, _held, _alphaLimited, _speedLimited;
        private float _peakGap, _peakA, _peakW;
        private string _traceArm = ArmOff;
        internal HipNormalReferenceState Current { get; private set; }
        internal Quaternion Rotation => _rotation;
        internal Vector3 W => _w;
        internal bool Selected => _selected;
        internal bool SampleAgePrediction => _sampleAgePrediction;
        internal static string ArmFor(bool selected, bool sampleAgePrediction) => !selected ? ArmOff : sampleAgePrediction ? ArmA : ArmB;
        internal string Issue { get; private set; }

        internal void Reset(bool selected, Quaternion rotation, Vector3 w, bool sampleAgePrediction = true)
        {
            _selected = selected; _sampleAgePrediction = selected && sampleAgePrediction; _seeded = Q(rotation) && V(w);
            _rotation = _seeded ? Normalize(rotation) : Quaternion.identity;
            _w = _seeded ? w : Vector3.zero;
            _last = new PhysicsStepContext(); _source = new SourceStepContext();
            Current = new HipNormalReferenceState(); Issue = null; ResetTrace();
        }
        internal void ResetTrace()
        {
            _traceArm = ArmFor(_selected, _sampleAgePrediction);
            _traceSampleAgePrediction = _selected && _sampleAgePrediction;
            _steps = _applied = _held = _alphaLimited = _speedLimited = 0;
            _peakGap = _peakA = _peakW = 0f;
        }
        internal void DisablePreserveTrace() { _selected = _seeded = _sampleAgePrediction = false; Current = new HipNormalReferenceState(); }

        internal bool Step(bool trace, PhysicsStepContext step, SourceStepContext source, bool ordinary,
            Quaternion inputWorldR, Vector3 inputWorldW, Quaternion parentR, Vector3 relativeW,
            float age, float maxAge, float maxW, float maxA, float arrivalRate)
        {
            Current = new HipNormalReferenceState { Step = step, Selected = _selected, SampleAgePrediction = _sampleAgePrediction, Reason = "BASELINE" };
            if (!_selected) return true; // Non-dynamic legacy path remains an exact bypass.
            if (!_seeded || !step.Valid || !source.Valid || step.Epoch != source.Epoch || step.SourceSequence != source.Sequence ||
                !Q(inputWorldR) || !Q(parentR) || !V(inputWorldW) || !V(relativeW) || !F(age) || !F(maxAge) ||
                !F(maxW) || !F(maxA) || !F(arrivalRate) || maxW <= 0f || maxA <= 0f || arrivalRate <= 0f || step.Dt >= 0.25f)
                return Invalid("INPUT_OR_CLOCK_INVALID");
            if (_last.Valid && (step.Epoch != _last.Epoch || step.Sequence != _last.Sequence + 1 || step.FixedTime < _last.FixedTime))
                return Invalid("FIXED_STEP_NOT_CONSECUTIVE");
            if (_source.Valid && (source.Epoch != _source.Epoch || source.Sequence < _source.Sequence || source.Time < _source.Time ||
                (source.Sequence == _source.Sequence && !source.Matches(_source))))
                return Invalid("SOURCE_IDENTITY_CHANGED");
            try
            {
                bool freshEnough = age >= -0.0001f && age <= Mathf.Max(0f, maxAge) + 0.0001f;
                bool apply = ordinary && freshEnough;
                HipNormalReferenceState frame = new HipNormalReferenceState { Step = step, Valid = true, Selected = true,
                    Applied = apply, SampleAgePrediction = _sampleAgePrediction, Reason = apply ? "COHERENT_NORMAL" : ordinary ? "STALE_BYPASS" : "PROTECTED_BYPASS",
                    PreviousR = _rotation, PreviousW = _w, InputR = Normalize(inputWorldR), InputW = inputWorldW,
                    MaxW = maxW, MaxA = maxA };
                if (apply)
                {
                    // A retains the previous local-only sample extrapolation. B holds the observed pose goal;
                    // input W, stop reserve, Fixed integration, FF and all budgets remain identical.
                    // Never advance the parent twice or replace the actual source-age eligibility check.
                    frame.PredictionSeconds = _sampleAgePrediction ? Mathf.Clamp(age, 0f, Mathf.Max(0f, maxAge)) : 0f;
                    Vector3 localPredictionWorldW = parentR * relativeW;
                    Quaternion goal = Advance(frame.InputR, localPredictionWorldW, frame.PredictionSeconds);
                    Quaternion origin = Advance(_rotation, inputWorldW, step.Dt);
                    Vector3 e = Log(goal * Quaternion.Inverse(origin));
                    Vector3 relative = _w - inputWorldW;
                    Vector3 stop = relative * (relative.magnitude / (2f * maxA) + step.Dt);
                    Vector3 desiredW = inputWorldW + (e - stop) * arrivalRate;
                    Vector3 limitedW = Vector3.ClampMagnitude(desiredW, maxW);
                    Vector3 delta = limitedW - _w;
                    Vector3 nextW = _w + Vector3.ClampMagnitude(delta, maxA * step.Dt);
                    frame.A = (nextW - _w) / step.Dt;
                    frame.AlphaLimited = delta.magnitude > maxA * step.Dt;
                    frame.SpeedLimited = desiredW.magnitude > maxW;
                    // Preserve inherited W, decelerating within A, rather than hard clipping it on handoff.
                    if (!V(nextW) || !V(frame.A) || nextW.magnitude > Mathf.Max(maxW, _w.magnitude) + 0.0001f ||
                        frame.A.magnitude > maxA + 0.002f || nextW.magnitude * step.Dt >= Mathf.PI - 0.0001f)
                        return Invalid("TRAJECTORY_BUDGET_INVALID");
                    _rotation = Advance(_rotation, nextW, step.Dt); _w = nextW;
                }
                else
                {
                    // Track the old stack's actual output. On return, seed from that output; no body snap or W reset.
                    _rotation = frame.InputR; _w = inputWorldW;
                }
                if (trace)
                {
                    _steps++;
                    if (_source.Valid && source.Matches(_source)) _held++;
                    if (apply)
                    {
                        _applied++; if (frame.AlphaLimited) _alphaLimited++; if (frame.SpeedLimited) _speedLimited++;
                        _peakGap = Mathf.Max(_peakGap, Log(frame.InputR * Quaternion.Inverse(_rotation)).magnitude);
                        _peakW = Mathf.Max(_peakW, _w.magnitude); _peakA = Mathf.Max(_peakA, frame.A.magnitude);
                    }
                }
                _last = step; _source = source; Current = frame; return true;
            }
            catch { return Invalid("TRAJECTORY_NONFINITE"); }
        }
        internal string GetSummary() => "HIP_NORMAL_REFERENCE_V04800/selected=" + _selected + "/liveSampleAgePrediction=" + _sampleAgePrediction + "/traceSampleAgePrediction=" + _traceSampleAgePrediction + "/traceArm=" + _traceArm +
            "/steps-applied-held-alphaLimited-speedLimited=" + _steps + "/" + _applied + "/" + _held + "/" + _alphaLimited + "/" + _speedLimited +
            "/peak(rawGapDeg-W-A)=" + S(_peakGap * Mathf.Rad2Deg) + "/" + S(_peakW) + "/" + S(_peakA) +
            "/issue=" + (Issue ?? "<none>") + "/domain=ONE_SHARED_FIXED_STEP_WORLD_TRAJECTORY" +
            "/bounds=HIP_NORMAL_MAX_A_OVER_KD_AND_EXISTING_FF_BOUND/notHumanLimits=True" +
            "/FF=PLANNED_WORLD_A_BEFORE_ORIGINAL_GATE_SUM_AND_CAP/traceReset=STATISTICS_ONLY";

        private bool Invalid(string issue) { Issue = issue; return false; }
        private static Quaternion Advance(Quaternion q, Vector3 w, float dt)
        {
            float m = w.magnitude;
            return m > 0f ? Normalize(Quaternion.AngleAxis(m * dt * Mathf.Rad2Deg, w / m) * q) : Normalize(q);
        }
        private static Vector3 Log(Quaternion q)
        {
            q = Normalize(q);
            if (q.w < 0f || (q.w == 0f && (q.x < 0f || (q.x == 0f && (q.y < 0f || (q.y == 0f && q.z < 0f))))))
                q = new Quaternion(-q.x, -q.y, -q.z, -q.w);
            Vector3 v = new Vector3(q.x, q.y, q.z); float m = v.magnitude;
            return m > 0f ? v * (2f * Mathf.Atan2(m, q.w) / m) : Vector3.zero;
        }
        private static Quaternion Normalize(Quaternion q)
        {
            float n = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            if (!F(n) || n <= 0f) throw new InvalidOperationException("HIP_NORMAL_REFERENCE_ROTATION_INVALID");
            return new Quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
        }
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z) && F(v.sqrMagnitude);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w) &&
            F(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) && q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 0.00000001f;
        private static string S(float v) => v.ToString("R", CultureInfo.InvariantCulture);
    }
}
