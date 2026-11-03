using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One per-foot eligibility receipt, consumed by all existing serial-load owners.
    // Contact remains lifecycle-owned. No force, gravity, pose or clock is written here.
    // These v119 experiment bounds qualify the first full load, not human limits,
    // solver contact support or continuous tracking. Do not turn waiting into a PASS.
    internal sealed class StanceLoadAdmission
    {
        internal const float MaxGapMeters = 0.002f;
        internal const float MaxPenetrationMeters = 0.003f;
        internal const float MaxAnchorMeters = 0.005f;
        internal const float MaxAgeSeconds = 0.1f;
        internal const int ReadySamples = 2;
        private bool _selected, _finite = true, _admitted, _eligible, _traceSelected;
        private PhysicsStepContext _step;
        private int _local, _support, _ready;
        private string _reason = "OFF", _issue;
        private int _baseSamples, _waitingSamples, _eligibleSamples, _onsets;
        private int _footprintBlocked, _clearanceBlocked, _predictedBlocked, _anchorBlocked, _ageBlocked, _holdBlocked;
        private float _waitSeconds, _maxWaitSeconds, _eligibleSeconds;
        private int _firstAdmissionFixed, _lastAdmissionFixed, _lastGeometryFixed;
        private float _lastMin, _lastPredicted, _lastAnchor, _lastAge;
        internal bool Selected => _selected;
        internal bool HasStep => _step.Valid;
        internal bool Finite => _finite;
        internal string Issue => _issue;
        internal bool Eligible => _eligible;
        internal bool NeedsGeometry(int support) => _selected && (!_admitted || support != _support);
        internal bool Matches(PhysicsStepContext step, int local) => _finite && _local == local && _step.Matches(step);

        internal void Select(bool selected)
        {
            DisablePreserveTrace();
            _selected = selected; _finite = true; _issue = null;
        }
        internal void DisablePreserveTrace()
        {
            _selected = _admitted = _eligible = false; _ready = _support = _local = 0;
            _step = default; _waitSeconds = 0f; _reason = "OFF";
        }
        internal void ResetTrace()
        {
            // Never change a live admission, hold, clock or already-scheduled load.
            _traceSelected = _selected;
            _baseSamples = _waitingSamples = _eligibleSamples = _onsets = 0;
            _footprintBlocked = _clearanceBlocked = _predictedBlocked = _anchorBlocked = _ageBlocked = _holdBlocked = 0;
            _maxWaitSeconds = _eligibleSeconds = 0f;
            _firstAdmissionFixed = _lastAdmissionFixed = _lastGeometryFixed = 0;
            _lastMin = _lastPredicted = _lastAnchor = _lastAge = 0f;
        }

        internal bool Step(bool trace, PhysicsStepContext step, int local, int support, bool baseEligible,
            FootStanceOnlyAuthorizedContactLifecycle.EntryGeometry geometry)
        {
            if (!_finite) return false;
            if (_selected && (!step.Valid || local <= 0 || (_step.Valid &&
                (step.Epoch != _step.Epoch || step.Sequence != _step.Sequence + 1 || local != _local + 1))))
                return Invalid("LOAD_ADMISSION_CLOCK_DISCONTINUITY");
            bool wasEligible = _eligible;
            _step = step; _local = local; _eligible = false;
            if (!baseEligible)
            {
                _admitted = false; _ready = _support = 0; _waitSeconds = 0f;
                _reason = "NO_AUTHORIZED_CONTACT"; return true;
            }
            if (trace) _baseSamples++;
            if (!_selected)
            {
                _eligible = true; _reason = "LEGACY_DIRECT";
            }
            else
            {
                if (support == 0) return Invalid("LOAD_ADMISSION_SUPPORT_ID_MISSING");
                if (support != _support) { _admitted = false; _ready = 0; _waitSeconds = 0f; wasEligible = false; }
                _support = support;
                if (!_admitted)
                {
                    if (!geometry.Valid || !geometry.MotionValid || !F(geometry.MinClearance) ||
                        !F(geometry.PredictedMinClearance) || !F(geometry.AnchorGap) || geometry.AnchorGap < 0f ||
                        !F(geometry.SourceAge) || geometry.SourceAge < 0f)
                        return Invalid("LOAD_ADMISSION_GEOMETRY_OR_MOTION_INVALID");
                    if (trace)
                    {
                        _lastGeometryFixed = local; _lastMin = geometry.MinClearance;
                        _lastPredicted = geometry.PredictedMinClearance;
                        _lastAnchor = geometry.AnchorGap; _lastAge = geometry.SourceAge;
                    }
                    string blocked = null;
                    if (!geometry.FullFootprint) { blocked = "WAIT_FOOTPRINT"; if (trace) _footprintBlocked++; }
                    else if (geometry.MinClearance < -MaxPenetrationMeters || geometry.MinClearance > MaxGapMeters)
                    { blocked = "WAIT_CLEARANCE"; if (trace) _clearanceBlocked++; }
                    else if (geometry.PredictedMinClearance < -MaxPenetrationMeters || geometry.PredictedMinClearance > MaxGapMeters)
                    { blocked = "WAIT_PREDICTED_CLEARANCE"; if (trace) _predictedBlocked++; }
                    else if (geometry.AnchorGap > MaxAnchorMeters) { blocked = "WAIT_ANCHOR"; if (trace) _anchorBlocked++; }
                    else if (geometry.SourceAge > MaxAgeSeconds) { blocked = "WAIT_SOURCE_AGE"; if (trace) _ageBlocked++; }
                    if (blocked != null) { _ready = 0; _reason = blocked; }
                    else
                    {
                        _ready++;
                        _admitted = _ready >= ReadySamples;
                        _reason = _admitted ? "ADMITTED" : "WAIT_TWO_FIXED_SAMPLES";
                        if (trace && !_admitted) _holdBlocked++;
                    }
                }
                else _reason = "ADMITTED_HELD";
                _eligible = _admitted;
            }
            if (_eligible)
            {
                _waitSeconds = 0f;
                if (trace)
                {
                    _eligibleSamples++; _eligibleSeconds += step.Dt;
                    if (!wasEligible)
                    {
                        _onsets++; if (_firstAdmissionFixed == 0) _firstAdmissionFixed = local;
                        _lastAdmissionFixed = local;
                    }
                }
            }
            else
            {
                _waitSeconds += step.Dt;
                if (trace) { _waitingSamples++; if (_waitSeconds > _maxWaitSeconds) _maxWaitSeconds = _waitSeconds; }
            }
            return true;
        }

        private bool Invalid(string issue) { _finite = _eligible = _admitted = false; _issue = issue; return false; }
        private static bool F(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string S(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
        internal string Summary() => "STANCE_LOAD_ADMISSION_V041190/tracePolicy=" + (_traceSelected ? "NEAR_SURFACE_TWO_FIXED" : "LEGACY_DIRECT") +
            "/liveSelected-finite-eligible=" + _selected + "|" + _finite + "|" + _eligible + "/reason=" + _reason + "/issue=" + (_issue ?? "<none>") +
            "/baseEligible-waiting-admitted-onsets=" + _baseSamples + "|" + _waitingSamples + "|" + _eligibleSamples + "|" + _onsets +
            "/blockedFirst(footprint-clearance-predicted-anchor-age-hold)=" + _footprintBlocked + "|" + _clearanceBlocked + "|" + _predictedBlocked + "|" + _anchorBlocked + "|" + _ageBlocked + "|" + _holdBlocked +
            "/admittedSeconds-maxWaitSeconds=" + S(_eligibleSeconds) + "|" + S(_maxWaitSeconds) +
            "/first-lastAdmissionFixed=" + _firstAdmissionFixed + "|" + _lastAdmissionFixed +
            "/lastGeometry(fixed-minMm-predictedMm-anchorMm-ageSec)=" + _lastGeometryFixed + "|" + S(_lastMin * 1000f) + "|" + S(_lastPredicted * 1000f) + "|" + S(_lastAnchor * 1000f) + "|" + S(_lastAge) +
            "/limits(minMm-maxMm-anchorMm-ageSec-readySamples)=-3|2|5|0.1|2/receipt=PRE_SCHEDULE_ELIGIBILITY_NOT_SOLVED_LOAD/noTimeoutAdmission=True";
    }
}
