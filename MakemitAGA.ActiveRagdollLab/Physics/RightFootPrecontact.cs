using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // The selector owns a temporary collider window. Strong stance and load admission keep their own clocks.
    internal sealed partial class RightFootPrecontact
    {
        internal enum Mode { Off, FallingForecastRise5mm, NearGroundHold8mm, PreviousStepExpiryReview }
        internal enum End { None, Handoff, Lift, Rising, Floor, Clock, Stale, Depth, Gap, Timeout, Miss, Stop, Fault, TraceEnded, ExtensionLimit, GoalLost, NearGroundExit }
        internal const float MaxSeconds = .150f;
        private const int DetailLimit = 64;
        // Three normal v161 handoffs measured sub-micrometre negative clearance on the settled foot.
        // This allowance applies only to the lower edge of an already extended near-ground window.
        private const float NearGroundZeroToleranceMeters = .000001f;
        private const float NearGroundMaxMeters = .008f;
        private const float MinHeight = .001f;
        private const float MaxHeight = FootContactGeometryLimits.RecoveryMaxGapMeters;

        internal struct Sample
        {
            internal PhysicsStepContext Step;
            internal SourceStepContext Source;
            internal int Local, Support;
            internal bool ExactFloor, PhaseKnown, Lift, Swing;
            internal string Gait;
            internal Vector3 FloorCenter, FloorHalf, W;
            internal float Min, Predicted, CornerV, AnkleY, Gap, Age, ControlMin;
            internal bool Clock => Local > 0 && Step.Valid && Source.Valid && Step.Epoch == Source.Epoch &&
                Step.SourceSequence == Source.Sequence;
            internal bool Geometry => ExactFloor && Support != 0 && float.IsFinite(Min) && float.IsFinite(Predicted) &&
                float.IsFinite(CornerV) && float.IsFinite(AnkleY) && float.IsFinite(Gap) && Gap >= 0f;
            internal bool Fresh => float.IsFinite(Age) && Age >= 0f && Age <= FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds;
            internal bool Approach => Clock && Geometry && Fresh && PhaseKnown && !Lift &&
                Min >= MinHeight && Min <= MaxHeight && CornerV < 0f;
        }

        private struct Episode
        {
            internal int Epoch, Shared, Source, Origin, Support, Last, Solves, Callbacks, First, FirstNonzero, MissingImpulse, MissingW;
            internal int VerifiedUnloaded, LoadViolations, LastUnloadedLocal;
            internal int ExpiryLocal, ExtraSolves, RisingLocal, NearHoldFirstRise;
            internal float ExtraSeconds;
            internal bool Extended, RisingPending, PredictionOnlyAtBase, WideForecastAtBase, WideForecastAtRise, PreviousStepCandidate;
            internal float Seconds, StartMin, StartPred, StartGap, Low, Rise, PeakGap, FirstJ, FirstKick, NonzeroJ, NonzeroKick, PeakJ, PeakKick;
            internal End Reason;
        }

        private struct FormalImpact
        {
            internal int Start, Source, Epoch, Support, Prior, First, FirstNonzero;
            internal bool Recovery;
            internal float FirstJ, FirstKick, NonzeroJ, NonzeroKick;
        }
        private readonly FormalImpact[] _impactDetails = new FormalImpact[DetailLimit];
        private FormalImpact _impact;
        private int _impactStarts, _impactSaved, _impactWithCallback, _impactWithNonzero;
        private bool _impactLive;

        internal Mode Selected { get; private set; }
        internal bool Tracking { get; private set; }
        internal bool Physical => Tracking && Selected != Mode.Off;
        internal int LinkedOrigin { get; private set; }
        private Sample _last;
        private Episode _current;
        private Vector3 _windowFloorCenter, _windowFloorHalf;
        private readonly Episode[] _details = new Episode[DetailLimit];
        private readonly int[] _ends = new int[Enum.GetValues(typeof(End)).Length];
        private int _evaluated, _exact, _approach, _pairs;
        private int _started, _saved, _solves, _checked, _violations, _callbacks, _withCallback, _linkedSupport;
        private double _time, _linkedAt;
        private bool _locked, _positionRising;

        internal static Mode ForArm(string arm) =>
            PelvisDynamicReceiverIsolation.IsDynamicHipArm(arm) ? Mode.PreviousStepExpiryReview : Mode.Off;

        private static bool UsesNearGroundHold(Mode mode) => mode is Mode.NearGroundHold8mm or Mode.PreviousStepExpiryReview;
        private static bool IsPhysicalMode(Mode mode) => UsesNearGroundHold(mode) || mode == Mode.FallingForecastRise5mm;

        // The older first-rise mode is retained for recorded contract replays only.
        internal static Mode ForFoot(Mode requested, string side) =>
            string.Equals(side, "RIGHT", StringComparison.OrdinalIgnoreCase) && UsesNearGroundHold(requested) ? requested : Mode.Off;

        internal void Select(Mode mode) { Reset(); Selected = IsPhysicalMode(mode) ? mode : Mode.Off; }
        private void Reset()
        {
            FinishRisingReview("Reset", 0);
            FinishFollow("Reset");
            ResetExpiryCounters();
            ResetImpacts();
            Tracking = _locked = false;
            _current = default;
            _last = default;
            _evaluated = _exact = _approach = _pairs = 0;
            _carriedIn = _started = _saved = _solves = _checked = _violations = _callbacks = _withCallback = 0;
            ClearLink(); _time = 0d;
            Array.Clear(_ends, 0, _ends.Length);
        }

        // Trace boundaries have no authority over a live collider or the two-step motion history.
        private int _carriedIn;
        internal void ResetTrace()
        {
            ResetExpiryCounters();
            if (Tracking && _current.Extended) _cutoffs = _matches = _extensions = 1;
            if (_impactLive)
            {
                _impactStarts = _impactSaved = 1;
                _impactWithCallback = _impact.First != 0 ? 1 : 0;
                _impactWithNonzero = _impact.FirstNonzero != 0 ? 1 : 0;
                _impactDetails[0] = _impact;
            }
            else ResetImpacts();
            _evaluated = _exact = _approach = _pairs = 0;
            _saved = _started = _solves = _checked = _violations = _callbacks = _withCallback = _carriedIn = 0;
            Array.Clear(_ends, 0, _ends.Length);
            if (!Tracking) return;
            _carriedIn = _started = 1;
            _solves = _current.Solves; _checked = _current.VerifiedUnloaded; _violations = _current.LoadViolations;
            _callbacks = _current.Callbacks; _withCallback = _current.First == 0 ? 0 : 1;
        }

        private void ResetImpacts()
        {
            _impactLive = false;
            _impactStarts = _impactSaved = _impactWithCallback = _impactWithNonzero = 0;
        }

        internal void BeginFormal(bool recovery)
        {
            FinishFollow(recovery ? "Recovery" : "Ordinary");
            _impactLive = true; _impactStarts++;
            _impact = new FormalImpact { Start = _last.Local, Source = _last.Step.SourceSequence, Epoch = _last.Step.Epoch,
                Support = _last.Support, Prior = LinkedOrigin, Recovery = recovery,
                FirstJ = float.NaN, FirstKick = float.NaN, NonzeroJ = float.NaN, NonzeroKick = float.NaN };
            if (_impactSaved < DetailLimit) _impactDetails[_impactSaved++] = _impact;
        }

        internal void EndFormal() => _impactLive = false;
        internal bool NeedsFormalImpact(int support) => _impactLive && support == _impact.Support && _impact.FirstNonzero == 0;

        internal void FormalCallback(int local, int support, float signedJ, float deltaW)
        {
            if (!NeedsFormalImpact(support) || local < _impact.Start) return;
            if (_impact.First == 0)
            {
                _impact.First = local; _impact.FirstJ = signedJ; _impact.FirstKick = deltaW; _impactWithCallback++;
            }
            if (float.IsFinite(signedJ) && signedJ != 0f)
            {
                _impact.FirstNonzero = local; _impact.NonzeroJ = signedJ; _impact.NonzeroKick = deltaW; _impactWithNonzero++;
            }
            if (_impactStarts <= DetailLimit) _impactDetails[_impactStarts-1] = _impact;
        }

        private void ClearLink() { LinkedOrigin = _linkedSupport = 0; _linkedAt = 0d; }
        internal void AfterFormalEvent() { ClearLink(); _locked = true; }
        internal void BreakContinuity() { FinishFollow("ContinuityEnded"); _last = default; _locked = false; ClearLink(); }

        private static bool Adjacent(in Sample a, in Sample b) =>
            a.Clock && b.Clock && a.Step.Epoch == b.Step.Epoch && a.Step.Sequence + 1 == b.Step.Sequence &&
            a.Local + 1 == b.Local && b.Step.FixedTime >= a.Step.FixedTime && b.Step.UnscaledTime >= a.Step.UnscaledTime &&
            (a.Source.Sequence == b.Source.Sequence ? a.Source.Matches(b.Source) :
             b.Source.Sequence > a.Source.Sequence && b.Source.Time >= a.Source.Time);

        private static bool SameFloor(in Sample a, in Sample b) =>
            a.Support == b.Support && a.FloorCenter.Equals(b.FloorCenter) && a.FloorHalf.Equals(b.FloorHalf);

        internal static bool Candidate(in Sample previous, in Sample current) =>
            previous.Approach && current.Approach && Adjacent(previous, current) && SameFloor(previous, current) &&
            current.Min < previous.Min && current.AnkleY < previous.AnkleY &&
            current.Gap <= FootContactGeometryLimits.RecoveryMaxAnchorGapMeters &&
            current.Predicted >= -FootContactGeometryLimits.OrdinaryPenetrationMeters;

        // Called once before formal qualification. Opening is committed only after ordinary stance had first refusal.
        internal End Evaluate(in Sample s, bool missInProgress, out bool wantsOpen)
        {
            wantsOpen = false;
            var previous = _last;
            _last = s;
            _evaluated++;
            if (s.Clock && s.Geometry) _exact++;
            if (s.Approach) _approach++;
            if (s.Clock) _time += s.Step.Dt;
            bool adjacent = Adjacent(previous, s);
            bool same = adjacent && SameFloor(previous, s);
            bool newSwing = s.Swing && previous.PhaseKnown && !previous.Swing && !previous.Lift;
            if (_follow.Origin != 0) ObserveFollow(s, same, newSwing);
            if (!same || !s.Geometry || !s.Fresh || !s.PhaseKnown || s.Lift || newSwing ||
                (_linkedSupport != 0 && _linkedSupport != s.Support) || _time - _linkedAt > .5d + .000001d)
                ClearLink();

            if (Tracking)
            {
                if (!adjacent) return End.Clock;
                if (!same || !s.Geometry) return End.Floor;
                ObserveHeight(s);
                if (!s.Fresh) return End.Stale;
                if (!s.PhaseKnown || s.Lift || newSwing) return End.Lift;
                if (s.Gap >= FootContactGeometryLimits.AnchorValidityMeters) return End.Gap;
                if (s.Min < -FootContactGeometryLimits.RecoveryPenetrationMeters || s.Min > MaxHeight) return End.Depth;
                _positionRising = s.Min > previous.Min && s.AnkleY > previous.AnkleY;
                if (_current.Extended && UsesNearGroundHold(Selected))
                {
                    // Each additional solve must follow a verified unloaded native step in this window.
                    if (_current.LastUnloadedLocal != previous.Local || _current.VerifiedUnloaded != _current.Solves ||
                        _current.LoadViolations != 0) return End.Fault;
                    if (missInProgress) return End.Miss;
                    var budget = CheckExtendedBudget(s);
                    if (budget != End.None) return budget;
                    return s.Min >= -NearGroundZeroToleranceMeters && s.Min <= NearGroundMaxMeters ? End.None : End.NearGroundExit;
                }
                if (_current.RisingPending) return EvaluateRisingReview(s, missInProgress);
                if (_positionRising) return End.Rising;
                if (missInProgress) return End.Miss;
                return _current.Extended ? CheckExtendedBudget(s) : CheckBaseExpiry(previous, s);
            }
            // A timeout or shallow bounce cannot reopen contact repeatedly on the same descent.
            if (!adjacent || s.Lift || newSwing || (s.Geometry && s.Min > MaxHeight))
            {
                _locked = false;
                ClearLink();
            }
            wantsOpen = !_locked && !missInProgress && !newSwing && s.Step.Dt <= MaxSeconds && Candidate(previous, s);
            if (wantsOpen) _pairs++;
            return End.None;
        }

        internal void Open()
        {
            FinishFollow("NextWindow");
            var s = _last;
            _windowFloorCenter = s.FloorCenter;
            _windowFloorHalf = s.FloorHalf;
            _current = new Episode { Epoch = s.Step.Epoch, Shared = s.Step.Sequence, Source = s.Source.Sequence,
                Origin = s.Local, Last = s.Local, Support = s.Support, StartMin = s.Min, StartPred = s.Predicted,
                StartGap = s.Gap, Low = s.Min, PeakGap = s.Gap, FirstJ = float.NaN, FirstKick = float.NaN,
                NonzeroJ = float.NaN, NonzeroKick = float.NaN, PeakJ = float.NaN, PeakKick = float.NaN };
            Tracking = _locked = true;
            _started++; LinkedOrigin = s.Local; _linkedSupport = s.Support; _linkedAt = _time;
        }

        internal void CountSolve()
        {
            // A negative confirmation reaches this point only after formal stance, miss and friction checks.
            if (_current.RisingPending && _last.Local != _current.RisingLocal)
                FinishRisingReview("Resume", _last.Local);
            if (_current.Extended)
            {
                WriteReviewReadback(End.None, _last.Local);
                if (UsesNearGroundHold(Selected) && _positionRising && _current.NearHoldFirstRise == 0)
                    _current.NearHoldFirstRise = _last.Local;
                _current.ExtraSeconds += _last.Step.Dt;
                _current.ExtraSolves++;
            }
            _current.Seconds += _last.Step.Dt;
            _current.Solves++; _solves++;
        }

        internal bool CheckUnloaded(int local, bool loaded)
        {
            if (!Physical) return true;
            if (local != _last.Local || loaded) { _violations++; _current.LoadViolations++; return false; }
            if (_current.LastUnloadedLocal == local) return true;
            _current.LastUnloadedLocal = local;
            _checked++; _current.VerifiedUnloaded++;
            return true;
        }

        private void ObserveHeight(in Sample s)
        {
            _current.Last = s.Local;
            _current.Low = Mathf.Min(_current.Low, s.Min);
            _current.Rise = Mathf.Max(_current.Rise, s.Min - _current.Low);
            _current.PeakGap = Mathf.Max(_current.PeakGap, s.Gap);
        }

        internal void Callback(int local, int support, float signedJ, float deltaW)
        {
            if (!Physical || _last.Local != local || support != _current.Support) return;
            _callbacks++; _current.Callbacks++;
            if (_current.First == 0)
            {
                _current.First = local; _current.FirstJ = signedJ; _current.FirstKick = deltaW;
                _withCallback++;
            }
            if (_current.FirstNonzero == 0 && float.IsFinite(signedJ) && signedJ != 0f)
            {
                _current.FirstNonzero = local; _current.NonzeroJ = signedJ; _current.NonzeroKick = deltaW;
            }
            if (float.IsFinite(signedJ)) _current.PeakJ = MaxFinite(_current.PeakJ, Mathf.Abs(signedJ));
            else _current.MissingImpulse++;
            if (float.IsFinite(deltaW)) _current.PeakKick = MaxFinite(_current.PeakKick, deltaW);
            else _current.MissingW++;
        }
        private static float MaxFinite(float previous, float current) => float.IsFinite(previous) ? Mathf.Max(previous, current) : current;
        internal Vector3 PreSolveW => _last.W;

        internal void Close(End reason, int local)
        {
            if (!Tracking) return;
            if (_current.RisingPending)
                FinishRisingReview(reason == End.Rising ? "Release" : reason.ToString(), local);
            else if (reason == End.Rising && Selected == Mode.FallingForecastRise5mm && _last.Min > RisingReviewMeters &&
                EligibleRisingReview())
                WriteRisingReview("ControlRelease", local, local, _last.Predicted < _last.Min);
            CloseExpiry(reason, local);
            _current.Reason = reason;
            if (local > 0) _current.Last = local;
            if (_saved < _details.Length) _details[_saved++] = _current;
            _ends[(int)reason]++;
            Tracking = false;
            if (reason == End.Lift || reason == End.Clock) _locked = false;
            if (reason == End.Clock || reason == End.Floor || reason == End.Stale || reason == End.Lift ||
                reason == End.Stop || reason == End.Fault) ClearLink();
        }

        internal string Summary()
        {
            if (Selected == Mode.Off) return "";
            // StopTraceOwners snapshots before native quiesce. Include that last open window as censored.
            int detailCount = _saved + (Tracking && _saved < DetailLimit ? 1 : 0);
            var b = new StringBuilder("\nRIGHT_PRECONTACT_V041540/mode=").Append(Selected)
                .Append("/evaluated-exactMainMotion-descendingBand-actionablePair=").Append(_evaluated).Append('|').Append(_exact).Append('|').Append(_approach).Append('|').Append(_pairs)
                .Append("/started-saved-omitted=").Append(_started).Append('|').Append(detailCount).Append('|').Append(_started-detailCount).Append("/carriedIn=").Append(_carriedIn)
                .Append("/windowSteps-nativeVerifiedUnloaded-violations=").Append(_solves).Append('|').Append(_checked).Append('|').Append(_violations)
                .Append("/episodesWithCallback-enterStay=").Append(_withCallback).Append('|').Append(_callbacks);
            b.Append("/baseExpiry-matched-extended=").Append(_cutoffs).Append('|').Append(_matches).Append('|').Append(_extensions);
            for (int i = 1; i < _ends.Length; i++)
            {
                if ((End)i == End.NearGroundExit && !UsesNearGroundHold(Selected)) continue;
                b.Append('/').Append((End)i).Append('=').Append(_ends[i] + (Tracking && (End)i == End.TraceEnded ? 1 : 0));
            }
            for (int i = 0; i < detailCount; i++)
            {
                var e = i < _saved ? _details[i] : _current;
                if (i == _saved) e.Reason = End.TraceEnded;
                b.Append("\nRIGHT_PRECONTACT_EVENT_V041540/epoch-shared-origin-source-support=").Append(e.Epoch).Append('|').Append(e.Shared)
                    .Append('|').Append(e.Origin).Append('|').Append(e.Source).Append('|').Append(e.Support)
                    .Append("/end-local=").Append(e.Reason).Append('|').Append(e.Last)
                    .Append("/upcomingSolves-ms=").Append(e.Solves).Append('|').Append(N(e.Seconds*1000f))
                    .Append("/startMin-pred-gapMm=").Append(N(e.StartMin*1000f)).Append('|').Append(N(e.StartPred*1000f)).Append('|').Append(N(e.StartGap*1000f))
                    .Append("/low-riseFromLow-peakGapMm=").Append(N(e.Low*1000f)).Append('|').Append(N(e.Rise*1000f)).Append('|').Append(N(e.PeakGap*1000f))
                    .Append("/firstCallbackLocal-signedNormalNs-deltaWRadSec=").Append(e.First).Append('|').Append(N(e.FirstJ)).Append('|').Append(N(e.FirstKick))
                    .Append("/firstNonzeroLocal-signedNormalNs-deltaWRadSec=").Append(e.FirstNonzero).Append('|').Append(N(e.NonzeroJ)).Append('|').Append(N(e.NonzeroKick))
                    .Append("/peakAbsNormalNs-deltaWRadSec=").Append(N(e.PeakJ)).Append('|').Append(N(e.PeakKick))
                    .Append("/callbacks-missingImpulse-missingW=").Append(e.Callbacks).Append('|').Append(e.MissingImpulse).Append('|').Append(e.MissingW)
                    .Append("/verifiedUnloaded-violations=").Append(e.VerifiedUnloaded).Append('|').Append(e.LoadViolations)
                    .Append("/baseExpiryLocal-extraSolves-extraMs=").Append(e.ExpiryLocal).Append('|').Append(e.ExtraSolves).Append('|').Append(N(e.ExtraSeconds*1000f))
                    .Append("/risingReviewLocal-pending=").Append(e.RisingLocal).Append('|').Append(e.RisingPending);
                if (UsesNearGroundHold(Selected))
                    b.Append("/nearHoldFirstRiseLocal=").Append(e.NearHoldFirstRise).Append("/previousStepCandidate=").Append(e.PreviousStepCandidate);
            }
            b.Append("\nRIGHT_FORMAL_IMPACT_V041540/starts-saved-firstCallback-firstNonzero=")
                .Append(_impactStarts).Append('|').Append(_impactSaved).Append('|').Append(_impactWithCallback).Append('|').Append(_impactWithNonzero);
            for (int i = 0; i < _impactSaved; i++)
            {
                var f = _impactDetails[i];
                b.Append("\nRIGHT_FORMAL_IMPACT_EVENT_V041540/epoch-local-source-support=").Append(f.Epoch).Append('|').Append(f.Start)
                    .Append('|').Append(f.Source).Append('|').Append(f.Support).Append("/recovery=").Append(f.Recovery)
                    .Append("/priorWindowLocal=").Append(f.Prior).Append("/firstCallbackLocal-signedNormalNs-deltaWRadSec=")
                    .Append(f.First).Append('|').Append(N(f.FirstJ)).Append('|').Append(N(f.FirstKick))
                    .Append("/firstNonzeroLocal-signedNormalNs-deltaWRadSec=").Append(f.FirstNonzero).Append('|').Append(N(f.NonzeroJ)).Append('|').Append(N(f.NonzeroKick));
            }
            return b.ToString();
        }
        private static string N(float x) => float.IsFinite(x) ? x.ToString("G7", CultureInfo.InvariantCulture) : "NA";
    }
}
