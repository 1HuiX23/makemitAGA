using System;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Phase = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Phase;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Paired PRE_N receipts, including the contact policy actually selected FOR_N.
    // No method of a physics owner reads this object. Left/right callback order is immaterial.
    internal sealed partial class BilateralFootTransitionTrace
    {
        internal enum Kind { PairRelease, SingleRelease, Brake }
        internal const int MaxPerSideKind = 48;
        internal struct Tick
        {
            internal PhysicsStepContext Step;
            internal FootTransitionSource.Sample Source;
            internal int Local, Body, Floor, PriorFloor, EventFloor, Held;
            internal float Age, BoxMin, HipHeight, SourceMin, ControlMin, RightEntryMin;
            internal bool EntryReferenceValid, QualifiedIdentity;
            internal Reference RightReference;
            internal SelectedPose RightPose;
            internal OrdinaryFootEntry.Sample Entry;
            internal int NativeDepthLatch;
            internal LandingHeightBreakdown Height;
            internal bool Fresh, Inside, HipValid, Was, Active, Precontact, Miss, PendingMiss, Ordinary, Recovery;
            internal bool PreviousHit;
            internal Vector3 Target, Foot, Lower, Velocity, Sole, SourceSole, CornerVelocity;
            internal FootGaitEvidence Gait;
            internal Phase Phase;
            internal string Motion, ReleaseReason;
            internal bool Released => Was && !Active;
            internal bool Full => Gait.Hits == 5 && Gait.Same && Gait.Surface == Floor && Inside;
            internal bool NearFloor => Full && BoxMin >= -FootContactGeometryLimits.RecoveryPenetrationMeters &&
                BoxMin <= FootContactGeometryLimits.RecoveryMaxGapMeters;
            internal bool Usable => Step.Valid && Source.Valid && Source.Root != 0 && Source.Clock.Valid && Source.Clock.Epoch == Step.Epoch &&
                Source.Clock.Sequence == Step.SourceSequence && Local > 0 && Body != 0 && Floor != 0 && Phase != Phase.Unknown &&
                float.IsFinite(Age) && Age >= 0f && Age <= FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds &&
                (!Was || PriorFloor == Floor) && (!Active || !Precontact) && (!(Active || Precontact || Miss || Ordinary || Recovery) || EventFloor == Floor) &&
                HipPointPairMobility.V(Target) && HipPointPairMobility.V(Foot) && HipPointPairMobility.V(Lower) &&
                HipPointPairMobility.V(Velocity) && HipPointPairMobility.V(Sole) && HipPointPairMobility.V(SourceSole) &&
                HipPointPairMobility.V(CornerVelocity) && float.IsFinite(BoxMin) && HipPointPairMobility.V(Gait.Normal) &&
                Gait.Normal.sqrMagnitude > .9f && float.IsFinite(Gait.Tangent) && float.IsFinite(Gait.SignedNormal) && float.IsFinite(Gait.Clearance);
        }
        internal struct View
        {
            internal Tick Foot;
            internal bool OtherActive, OtherPrecontact, OtherReleased, OtherPreviousHit;
            internal Phase OtherPhase;
            internal int OtherLocal;
        }
        private struct Pair
        {
            internal Tick Left, Right;
            internal View At(int side)
            {
                Tick foot = side == 0 ? Left : Right, other = side == 0 ? Right : Left;
                return new View { Foot = foot, OtherLocal = other.Local, OtherActive = other.Active,
                    OtherPrecontact = other.Precontact, OtherPreviousHit = other.PreviousHit, OtherReleased = other.Released, OtherPhase = other.Phase };
            }
        }
        private readonly FootTransitionSource[] _sources = { new FootTransitionSource(), new FootTransitionSource() };
        private readonly List<Window> _windows = new List<Window>();
        private readonly Window[] _release = new Window[2], _brake = new Window[2];
        private readonly int[,] _found = new int[2,3], _skipped = new int[2,3];
        private readonly int[,] _lift = new int[2,5];
        private Pair _previous;
        private Tick _pending;
        private int _pendingSide = -1;
        private PhysicsStepContext _lastPairStep;
        private int _duplicates, _staleSides, _invalidSides, _conflicts;
        private int _reentrySeen;
        private int _pairs, _cuts, _rejected, _missingFoot, _faults, _sealedPartial, _unmatchedCallbacks;
        private readonly bool[] _hits = new bool[2];
        private bool _sealed;

        internal FootTransitionSource Source(int side) => _sources[side];

        // A matched ENTER or STAY stamps the previous completed PRE_N pair. The next
        // matching PRE_N+1 consumes one boolean regardless of the callback count.
        internal void NoteContact(int side, in PhysicsStepContext step, int local, int support, bool precontact)
        {
            if (_sealed || side < 0 || side > 1) return;
            var t = side == 0 ? _previous.Left : _previous.Right;
            if (!t.Step.Matches(step) || t.Local != local || t.Floor != support || !(precontact ? t.Precontact : t.Active))
            { _unmatchedCallbacks++; return; }
            _hits[side] = true;
        }

        internal void Fault()
        {
            if (_sealed) return;
            _faults++; _pendingSide = -1; Cut("CaptureFault");
            foreach (var source in _sources) { source.DropBrake(); source.Clear(); }
        }

        internal void Submit(int side, in Tick tick)
        {
            if (_sealed) return;
            if (side != 0 && side != 1) { _invalidSides++; return; }
            if (!tick.Step.Valid) { _rejected++; _pendingSide = -1; Cut("InvalidClock"); return; }
            if (SameSlot(tick.Step, _lastPairStep))
            {
                var recorded = side == 0 ? _previous.Left : _previous.Right;
                if (tick.Step.Matches(_lastPairStep) && SameReceipt(recorded,tick)) _duplicates++;
                else _conflicts++;
                return;
            }
            if (Older(tick.Step, _lastPairStep) || (_pendingSide >= 0 && Older(tick.Step, _pending.Step)))
            { _staleSides++; return; }
            if (_pendingSide >= 0)
            {
                if (SameSlot(_pending.Step, tick.Step))
                {
                    if (!tick.Step.Matches(_pending.Step) || (_pendingSide == side && !SameReceipt(_pending,tick)))
                    {
                        _conflicts++; _lastPairStep = tick.Step; _pendingSide = -1; Cut("ClockConflict"); return;
                    }
                    if (_pendingSide == side) { _duplicates++; return; }
                }
                else { _missingFoot++; _pendingSide = -1; Cut("MissingSide"); }
            }
            if (_pendingSide == -1) { _pending = tick; _pendingSide = side; return; }
            Pair pair = side == 0 ? new Pair { Left = tick, Right = _pending } : new Pair { Left = _pending, Right = tick };
            _pendingSide = -1; _lastPairStep = tick.Step;
            if (!pair.Left.Usable || !pair.Right.Usable || pair.Left.Body == pair.Right.Body ||
                pair.Left.Floor != pair.Right.Floor || pair.Left.Source.Root != pair.Right.Source.Root ||
                !pair.Left.Source.Clock.Matches(pair.Right.Source.Clock))
            {
                _rejected++; Cut("UnmatchedPair"); return;
            }
            if (_previous.Left.Local != 0 && (!Next(_previous.Left, pair.Left) || !Next(_previous.Right, pair.Right)))
                Cut("Discontinuity");
            pair.Left.PreviousHit = _hits[0]; pair.Right.PreviousHit = _hits[1];
            _hits[0] = _hits[1] = false;
            _pairs++;
            for (int i = 0; i < 2; i++) Record(i, pair.At(i), _previous.At(i));
            TrackHandoffs(pair);
            _previous = pair;
        }

        private static bool Next(in Tick previous, in Tick current) =>
            previous.Step.Epoch == current.Step.Epoch && previous.Step.Sequence+1 == current.Step.Sequence &&
            previous.Local+1 == current.Local && previous.Active == current.Was && previous.Body == current.Body && previous.Floor == current.Floor &&
            previous.Source.Root == current.Source.Root && previous.Source.Continuity == current.Source.Continuity &&
            (current.Source.Clock.Sequence == previous.Source.Clock.Sequence ? current.Source.Clock.Matches(previous.Source.Clock) :
                current.Source.Clock.Sequence > previous.Source.Clock.Sequence && current.Source.Clock.Time >= previous.Source.Clock.Time) &&
            current.Step.FixedTime >= previous.Step.FixedTime && current.Step.UnscaledTime >= previous.Step.UnscaledTime;

        private static bool SameReceipt(in Tick a, in Tick b) =>
            a.Local == b.Local && a.Body == b.Body && a.Floor == b.Floor && a.Source.Root == b.Source.Root &&
            a.Source.Clock.Matches(b.Source.Clock) && a.Was == b.Was && a.Active == b.Active && a.Precontact == b.Precontact;

        private static bool SameSlot(in PhysicsStepContext a, in PhysicsStepContext b) =>
            a.Valid && b.Valid && a.Epoch == b.Epoch && a.Sequence == b.Sequence;

        private static bool Older(in PhysicsStepContext step, in PhysicsStepContext other) =>
            step.Valid && other.Valid && (step.Epoch < other.Epoch ||
                step.Epoch == other.Epoch && step.Sequence < other.Sequence);

        private void Cut(string reason)
        {
            _risePending = null;
            StopHandoffs(reason);
            if (_previous.Left.Local != 0) _cuts++;
            for (int i=0; i<2; i++)
            {
                Close(ref _release[i], reason); Close(ref _brake[i], reason);
                _sources[i].DropBrake();
            }
            _hits[0] = _hits[1] = false; _previous = default;
        }
        private void Record(int side, in View point, in View prev)
        {
            var t = point.Foot;
            var source = _sources[side];
            if (_release[side] != null)
            {
                if (t.Released) Close(ref _release[side], "NextRelease");
                else Advance(ref _release[side], point, source);
            }
            if (t.Released)
            {
                var k = point.OtherReleased ? Kind.PairRelease : Kind.SingleRelease;
                if (k == Kind.PairRelease && t.Phase == Phase.Lift && point.OtherPhase == Phase.Lift)
                {
                    _lift[side,0]++;
                    if (t.Gait.Swing == FootGaitEvidence.Trigger.Tangential)
                    {
                        _lift[side,1]++;
                        var relative = t.Gait.WithVelocity(t.Source.Relative);
                        if (relative.Swing == FootGaitEvidence.Trigger.None) _lift[side,2]++;
                        if (relative.Stance)
                        {
                            _lift[side,3]++;
                            if (t.NearFloor && FootTransitionSource.Planar(t.CornerVelocity).magnitude <= FootContactGeometryLimits.RecoveryMaxPlanarSpeed)
                                _lift[side,4]++;
                        }
                    }
                }
                _release[side] = Open(side,k,point,prev,source.Travel,"FormalRelease",t.Source.Clock.Sequence);
            }

            if (_brake[side] != null)
            {
                float resumeSpeed = _brake[side].Quiet.Foot.Local != 0 ? FootGaitEvidence.StanceTangent : FootGaitEvidence.SwingTangent;
                bool resumed = FootTransitionSource.Planar(t.Source.World).magnitude >= resumeSpeed;
                if (resumed && _brake[side].Slowed && !t.Source.Clock.Matches(prev.Foot.Source.Clock))
                {
                    _brake[side].ReadRenders(source,t.Source,t.Source.Clock.Sequence-1);
                    Close(ref _brake[side], "MotionResumed");
                }
                else Advance(ref _brake[side], point, source);
            }
            bool footBrake = source.BrakeSequence > 0 && source.BrakeSequence <= t.Source.Clock.Sequence;
            bool phaseEdge = prev.Foot.Local != 0 && prev.Foot.Motion != t.Motion && (t.Motion == "DECEL" || t.Motion == "STOP");
            if ((footBrake || phaseEdge) && source.Travel.Recent(t.Source.Clock.Time))
            {
                if (_brake[side] == null)
                    _brake[side] = Open(side, Kind.Brake, point, prev, source.Travel,
                        footBrake ? "FootDecel" : t.Motion, footBrake ? source.BrakeSequence : t.Source.Clock.Sequence);
                if (_brake[side] != null)
                {
                    if (footBrake && _brake[side].FootBrakeSource == 0) _brake[side].FootBrakeSource = source.BrakeSequence;
                    if (phaseEdge && _brake[side].TorsoEdge == null) _brake[side].TorsoEdge = t.Motion;
                }
            }
            if (footBrake) source.ConsumeBrake();
        }
        private Window Open(int side, Kind kind, in View point, in View prev, FootTransitionSource.Heading heading, string trigger, int triggerSource)
        {
            bool reentry = side == 1 && kind != Kind.Brake && prev.Foot.Active && point.Foot.Released &&
                point.Foot.Phase == Phase.Stance && !point.Foot.Gait.Stance && point.Foot.ReleaseReason == "RELEASE_ON_STRONG_STANCE_EXIT";
            if (reentry) _reentrySeen++;
            if (++_found[side,(int)kind] > MaxPerSideKind) { _skipped[side,(int)kind]++; return null; }
            var w = new Window(side, kind, point, prev, heading, trigger, triggerSource, _sources[side], reentry);
            _windows.Add(w);
            return w;
        }
        private static void Advance(ref Window w, in View point, FootTransitionSource source)
        {
            if (point.Foot.Step.FixedTime - w.Origin.Foot.Step.FixedTime > 1.5f)
            { Close(ref w, "WindowLimit"); return; }
            w.Observe(point,source);
            if (w.Type != Kind.Brake && w.LandingFinal) Close(ref w, "LandingResolved");
        }
        private static void Close(ref Window w, string why)
        {
            if (w == null) return;
            w.EndReason = why; w = null;
        }
        internal void Seal()
        {
            if (_sealed) return;
            _sealed = true;
            _risePending = null;
            StopHandoffs("RunEnd");
            if (_pendingSide >= 0) _sealedPartial++;
            _pendingSide = -1;
            for (int i=0; i<2; i++)
            {
                Close(ref _release[i],"RunEnd"); Close(ref _brake[i],"RunEnd"); _sources[i].DropBrake();
            }
        }
    }
}
