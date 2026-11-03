using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // The selected reference owns one continuous relative R/W/A trajectory,
    // including source/recovery transitions. Both current Hip arms select it;
    // legacy non-Hip scopes retain the raw-input bypass. No body reads, source derivative
    // estimator, phase re-seeding, force write, or diagnostic-owned control state.
    internal sealed class KneeRelativeReference
    {
        internal const string ArmOff = "RAW_KNEE_REFERENCE_V041170";
        internal const string ArmOn = "COHERENT_KNEE_REFERENCE_V041170";
        private bool _selected, _seeded;
        private Quaternion _rotation = Quaternion.identity;
        private Vector3 _w;
        private PhysicsStepContext _last;
        private SourceStepContext _source;
        private string _traceArm = ArmOff;
        private int _steps, _held, _stale, _alphaLimited, _speedLimited;
        private float _peakGap, _peakW, _peakA;
        internal bool Selected => _selected;
        internal KneeReferenceState Current { get; private set; }
        internal string Issue { get; private set; }

        internal void Reset(bool selected, Quaternion rotation, Vector3 w)
        {
            _selected = selected;
            _seeded = PelvisParentFrame.Q(rotation) && PelvisParentFrame.V(w);
            _rotation = _seeded ? Normalize(rotation) : Quaternion.identity;
            _w = _seeded ? w : Vector3.zero;
            _last = new PhysicsStepContext(); _source = new SourceStepContext(); Current = new KneeReferenceState(); Issue = null;
            ResetTrace();
        }
        internal void ResetTrace()
        {
            _traceArm = _selected ? ArmOn : ArmOff;
            _steps = _held = _stale = _alphaLimited = _speedLimited = 0;
            _peakGap = _peakW = _peakA = 0f;
        }
        internal void DisablePreserveTrace() { _selected = _seeded = false; Current = new KneeReferenceState(); }

        internal bool Step(bool trace, PhysicsStepContext step, SourceStepContext source,
            Quaternion inputR, Vector3 inputW, float age, float maxAge, float maxW, float maxA, float arrivalRate)
        {
            Current = new KneeReferenceState();
            if (!_selected) return true;
            if (!_seeded || !step.Valid || !source.Valid || step.Epoch != source.Epoch || step.SourceSequence != source.Sequence ||
                step.Dt >= 0.25f || !PelvisParentFrame.Q(inputR) || !PelvisParentFrame.V(inputW) ||
                !PelvisParentFrame.F(age) || age < -0.0001f || !PelvisParentFrame.F(maxAge) || maxAge < 0f ||
                !PelvisParentFrame.F(maxW) || maxW <= 0f || !PelvisParentFrame.F(maxA) || maxA <= 0f ||
                !PelvisParentFrame.F(arrivalRate) || arrivalRate <= 0f)
                return Invalid("INPUT_OR_CLOCK_INVALID");
            if (_last.Valid && (step.Epoch != _last.Epoch || step.Sequence != _last.Sequence + 1 || step.FixedTime < _last.FixedTime))
                return Invalid("FIXED_STEP_NOT_CONSECUTIVE");
            if (_source.Valid && (source.Epoch != _source.Epoch || source.Sequence < _source.Sequence || source.Time < _source.Time ||
                (source.Sequence == _source.Sequence && !source.Matches(_source))))
                return Invalid("SOURCE_IDENTITY_CHANGED");
            try
            {
                KneeReferenceState frame = new KneeReferenceState
                {
                    Step = step, Source = source, Valid = true, Selected = true,
                    Stale = age > maxAge + 0.0001f, PreviousR = _rotation, PreviousW = _w,
                    InputR = Normalize(inputR), InputW = inputW, MaxW = maxW, MaxA = maxA
                };
                Vector3 desiredW = Vector3.zero;
                if (!frame.Stale)
                {
                    // Held pose is not age-extrapolated. Arrival/braking is evaluated
                    // once per shared Fixed step, in the same Upper-local basis.
                    Quaternion origin = Advance(_rotation, inputW, step.Dt);
                    Vector3 error = Log(frame.InputR * Quaternion.Inverse(origin));
                    Vector3 relativeW = _w - inputW;
                    Vector3 stop = relativeW * (relativeW.magnitude / (2f * maxA) + step.Dt);
                    desiredW = inputW + (error - stop) * arrivalRate;
                }
                // Stale input brakes the existing trajectory; it never snaps to raw
                // pose or resets W. The original stale/source/recovery FF gates remain.
                Vector3 limitedW = Vector3.ClampMagnitude(desiredW, maxW);
                Vector3 delta = limitedW - _w;
                Vector3 nextW = _w + Vector3.ClampMagnitude(delta, maxA * step.Dt);
                frame.A = (nextW - _w) / step.Dt;
                frame.AlphaLimited = delta.magnitude > maxA * step.Dt;
                frame.SpeedLimited = desiredW.magnitude > maxW;
                if (!PelvisParentFrame.V(nextW) || !PelvisParentFrame.V(frame.A) ||
                    nextW.magnitude > Mathf.Max(maxW, _w.magnitude) + 0.0001f ||
                    frame.A.magnitude > maxA + 0.002f || nextW.magnitude * step.Dt >= Mathf.PI - 0.0001f)
                    return Invalid("TRAJECTORY_BUDGET_INVALID");
                frame.R = Advance(_rotation, nextW, step.Dt); frame.W = nextW;
                if (trace)
                {
                    _steps++; if (_source.Valid && source.Matches(_source)) _held++;
                    if (frame.Stale) _stale++;
                    if (frame.AlphaLimited) _alphaLimited++;
                    if (frame.SpeedLimited) _speedLimited++;
                    _peakGap = Mathf.Max(_peakGap, Log(frame.InputR * Quaternion.Inverse(frame.R)).magnitude);
                    _peakW = Mathf.Max(_peakW, nextW.magnitude); _peakA = Mathf.Max(_peakA, frame.A.magnitude);
                }
                _rotation = frame.R; _w = nextW; _last = step; _source = source; Current = frame;
                return true;
            }
            catch { return Invalid("TRAJECTORY_NONFINITE"); }
        }

        internal string GetSummary() => "KNEE_REFERENCE_V041170/selected=" + _selected + "/traceArm=" + _traceArm +
            "/steps-held-stale-alphaLimited-speedLimited=" + _steps + "|" + _held + "|" + _stale + "|" + _alphaLimited + "|" + _speedLimited +
            "/peak(rawGapDeg-W-A)=" + S(_peakGap * Mathf.Rad2Deg) + "|" + S(_peakW) + "|" + S(_peakA) +
            "/domain=SHARED_FIXED_UPPER_LOCAL_RELATIVE_RWA/FF=CALLER_SELECTED_GATES_ORIGINAL_ALLOCATION_CAPS" +
            "/bounds=KNEE_MAX_A_OVER_KD_AND_EXISTING_FF_BOUND/notHumanLimits=True/traceReset=STATISTICS_ONLY/issue=" + (Issue ?? "<none>");

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
            if (!PelvisParentFrame.F(n) || n <= 0f) throw new InvalidOperationException("KNEE_REFERENCE_ROTATION_INVALID");
            return new Quaternion(q.x / n, q.y / n, q.z / n, q.w / n);
        }
        private static string S(float v) => v.ToString("R", CultureInfo.InvariantCulture);
    }
}
