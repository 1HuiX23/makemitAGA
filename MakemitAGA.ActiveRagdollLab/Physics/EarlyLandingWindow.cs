using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Per foot, called after the original commands. Classification needs only PRE_N and its past.
    internal sealed class EarlyLandingWindow
    {
        internal enum Foot : byte { Unknown, Left, Right }
        internal enum Signal : byte { None, LeftLift, LeftStart, LeftContinue, LeftSwing, RightSteady, RightLow, RightNearLow }
        internal const double FirstSwingSeconds = .05d;
        private const float Band = FootContactGeometryLimits.OrdinaryPenetrationMeters;
        private readonly int[] _signals = new int[(int)Signal.RightNearLow + 1];
        private readonly int[] _outcomes = new int[(int)SwingKneeCandidate.Status.Fault + 1];
        private PhysicsStepContext _step;
        private SourceStepContext _source;
        private int _local, _support, _origin, _faults, _clockBreaks, _throttled;
        private Vector3 _center, _half;
        private double _sinceStart, _sinceSwing;
        private Phase _previousPhase;
        private Foot _previousFoot;
        private bool _havePrevious, _expired;
        private float _startReferenceError, _startFollowError;
        private bool _haveHeightOrigin;
        private int _rightStarts, _rightHeldTicks, _rightCleared;
        internal int RightWarningOrigin { get; private set; }
        private void ClearRightWarning()
        {
            if (RightWarningOrigin!=0) _rightCleared++;
            RightWarningOrigin=0;
        }
        internal float HeightLoss { get; private set; }
        internal float FollowChange { get; private set; }
        internal bool HeightTrial { get; private set; }
        internal int Origin => _origin;
        internal static Foot Side(string name) => name == "LEFT" ? Foot.Left : name == "RIGHT" ? Foot.Right : Foot.Unknown;
        internal static bool LeftTrial(Signal s) => s >= Signal.LeftStart && s <= Signal.LeftSwing;
        internal static bool RightLow(Signal s) => s == Signal.RightLow || s == Signal.RightNearLow;

        internal void Cut()
        {
            ClearRightWarning(); _haveHeightOrigin=HeightTrial=false;
            _origin = 0; _havePrevious = _expired = false; _sinceStart = _sinceSwing = 0d;
        }

        internal void Reset()
        {
            Cut(); _step = default; _source = default; _local = _clockBreaks = _faults = _throttled = 0;
            _rightStarts=_rightHeldTicks=_rightCleared=0;
            Array.Clear(_signals); Array.Clear(_outcomes);
        }

        internal void Fault() { _faults++; Cut(); }
        internal void Record(SwingKneeCandidate.Status reason) => _outcomes[(int)reason]++;
        internal Signal Observe(in Sample s, Foot side)
        {
            Signal signal = Classify(s, side);
            // A near-plane sample starts an alert. Only contiguous qualified, low-ankle
            // samples can keep it alive; one recovered ankle or invalid sample cancels it.
            if (!RightLow(signal)) ClearRightWarning();
            else if (RightWarningOrigin==0 && signal==Signal.RightNearLow)
            { RightWarningOrigin=s.Fixed; _rightStarts++; }
            if (RightWarningOrigin!=0) _rightHeldTicks++;
            HeightTrial=false;
            if (signal==Signal.LeftStart && s.ParentReady)
            {
                _startReferenceError=s.ReferenceHipY-s.RawHipY;
                _startFollowError=s.ActualHipY-s.ReferenceHipY;
                _haveHeightOrigin=true;
            }
            else if (!LeftTrial(signal) || !s.ParentReady) _haveHeightOrigin=false;
            if (_haveHeightOrigin)
            {
                HeightLoss=_startReferenceError-(s.ReferenceHipY-s.RawHipY);
                FollowChange=(s.ActualHipY-s.ReferenceHipY)-_startFollowError;
                HeightTrial=HeightLoss>=Band;
            }
            _signals[(int)signal]++;
            return signal;
        }

        private Signal Classify(in Sample s, Foot side)
        {
            if (!s.ClockValid) { _clockBreaks++; Cut(); return Signal.None; }
            if (_step.Valid && (s.Step.Epoch < _step.Epoch || (s.Step.Epoch == _step.Epoch &&
                (s.Fixed <= _local || s.Step.Sequence <= _step.Sequence || s.Step.FixedTime < _step.FixedTime ||
                 s.Step.UnscaledTime < _step.UnscaledTime || s.Source.Sequence < _source.Sequence ||
                 s.Source.Time < _source.Time || (s.Source.Sequence == _source.Sequence && !s.Source.Matches(_source))))))
            { _clockBreaks++; Cut(); return Signal.None; }
            bool adjacent = _havePrevious && side == _previousFoot && s.Step.Epoch == _step.Epoch && s.Fixed == _local + 1 &&
                s.Step.Sequence == _step.Sequence + 1 && s.SupportId == _support &&
                (s.FloorCenter - _center).sqrMagnitude <= 1e-12f && (s.FloorHalf - _half).sqrMagnitude <= 1e-12f;
            if (!adjacent) Cut();
            _local = s.Fixed; _step = s.Step; _source = s.Source;
            if (side == Foot.Unknown || s.WasActive || s.NowActive || s.FirstMiss || s.MissInProgress ||
                (s.Gait != Phase.Lift && s.Gait != Phase.Swing) || s.SourceMotion != FootLandingOpportunityAudit.SourceKind.Motion ||
                !s.BoxReferencesValid || !PelvisParentFrame.F(s.RawMin) || s.SelectedBoxFallback ||
                (FootLandingOpportunityAudit.Screen(s) & (FootLandingOpportunityAudit.Rejection.Geometry |
                  FootLandingOpportunityAudit.Rejection.Footprint | FootLandingOpportunityAudit.Rejection.Motion |
                  FootLandingOpportunityAudit.Rejection.Age | FootLandingOpportunityAudit.Rejection.Gap |
                  FootLandingOpportunityAudit.Rejection.FivePoint | FootLandingOpportunityAudit.Rejection.Identity)) != 0)
            { Cut(); return Signal.None; }
            _havePrevious = true; _previousFoot = side; _support = s.SupportId; _center = s.FloorCenter; _half = s.FloorHalf;
            Phase previous = _previousPhase; _previousPhase = s.Gait;
            if (side == Foot.Right)
            {
                if (s.Gait != Phase.Swing || s.SourcePhase != "STEADY" || !s.AnkleHeightsValid ||
                    !PelvisParentFrame.V(s.AnkleHeights)) return Signal.None;
                if (s.AnkleHeights.y - s.AnkleHeights.x >= -Band) return Signal.RightSteady;
                return Mathf.Abs(s.RawMin) <= Band ? Signal.RightNearLow : Signal.RightLow;
            }

            // An observed lift can start mid-trace. Neither a swing nor a future stance may start a left trial.
            if (s.Gait == Phase.Lift && previous != Phase.Lift) { _origin = 0; _expired = false; }
            if (_origin != 0)
            {
                _sinceStart += s.Step.Dt;
                if (s.Gait == Phase.Swing) _sinceSwing = previous == Phase.Swing ? _sinceSwing + s.Step.Dt : 0d;
                if (_sinceStart <= LandingEventHistory.Horizon + 1e-6d && _sinceSwing <= FirstSwingSeconds + 1e-6d)
                    return s.Gait == Phase.Lift ? Signal.LeftContinue : Signal.LeftSwing;
                _origin = 0; _expired = true; _throttled++;
            }
            if (s.Gait != Phase.Lift) return Signal.None;
            if (_expired || s.RawMin >= -Band || Mathf.Abs(s.Min) > Band) return Signal.LeftLift;
            _origin = s.Fixed; _sinceStart = _sinceSwing = 0d;
            return Signal.LeftStart;
        }

        internal string Summary() => "\nEARLY_WINDOW_V041460/steps(none-leftLift-leftStart-leftContinue-leftSwing-rightSteady-rightLowOnly-rightNearLow)=" +
            string.Join("|", _signals) + "/leftProbe(none-unavailable-protected-geometry-budget-noRoom-noLegal-favorable-fault)=" +
            string.Join("|", _outcomes) + "/clockRejected-expired-faults=" + _clockBreaks + "|" + _throttled + "|" + _faults +
            "/rightWarnings(starts-heldTicks-cleared-open)=" + _rightStarts + "|" + _rightHeldTicks + "|" + _rightCleared + "|" + (RightWarningOrigin!=0) +
            "/bandMm=2/firstSwingMs=50/leftMaxMs=500/rightAnkle=LOWER_MINUS_SOURCE/rightNear=ABS_RAW_BOX_LE_2MM/shadowOnly=True";
    }
}
