using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Runtime;
using Sample = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Sample;
using Rejection = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit.Rejection;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // One ring per foot, on the same simulation clock as the contact lifecycle.
    // A status read never samples a Transform. Text is built only for a sealed report.
    internal sealed partial class LandingEventHistory
    {
        internal const int RingCapacity = 128, DetailPerKind = 64;
        internal const double Horizon = .5d;
        private const double Epsilon = 1e-6d;
        private enum Trigger { Ordinary, FirstMiss }
        internal enum MissEnd { Recovered, HoldEnded, Discontinuity, TraceEnded }

        private struct Point
        {
            internal double Time;
            internal int Local, Source, Flags, Rejections;
            internal bool PreviousEvent, RightLandingObserved;
            internal EarlyLandingWindow.Signal Early;
            internal bool HeightRequested;
            internal RightThighWindow.Signal RightThigh;
            internal int RightWarningOrigin;
            internal int Origin;
            internal FootLandingOpportunityAudit.Phase Phase;
            internal string SourcePhase;
            internal int Epoch, Support;
            internal SourceRotationCoverage.Kind Rotation;
            internal float Min, Raw, Control, Predicted, Velocity, Age, Gap;
            internal float HipRaw, HipReference, HipActual, KneeRaw, KneeGoal, KneeActual;
            internal float AnkleRaw, AnkleLower, AnkleFoot;

            internal Point(in Sample s, double time, Rejection rejects)
            {
                Epoch = s.Step.Epoch; Support = s.SupportId;
                Rotation = LandingObservationTiming.Run.Running ?
                    LandingObservationTiming.Run.SourceRotation.AtOpportunity(s.Source,s.Step,s.SourceAge) : SourceRotationCoverage.Kind.Unknown;
                Time = time; Local = s.Fixed; Source = s.Source.Sequence; Phase = s.Gait; SourcePhase = s.SourcePhase;
                Flags = (s.StrongLanding ? 1 : 0) | (s.Identity ? 2 : 0) | (s.FivePoint ? 4 : 0) |
                    (s.FullFootprint ? 8 : 0) | (s.Adopted ? 16 : 0) | (s.SelectedBoxFallback ? 32 : 0);
                Rejections = (int)rejects;
                PreviousEvent = s.FirstMiss || s.MissInProgress || s.OrdinaryEnabled || s.RecoveryEnabled;
                RightLandingObserved=s.RightLandingObserved;
                Early = s.EarlySignal; Origin = s.EarlyOrigin;
                HeightRequested=s.ParentHeightRequested; RightWarningOrigin=s.RightWarningOrigin; RightThigh=s.RightThigh;
                float floor = s.FloorCenter.y + s.FloorHalf.y;
                Min = s.HeightValid ? s.Min : float.NaN;
                bool references = s.HeightValid && s.BoxReferencesValid;
                Raw = references ? s.RawMin : float.NaN;
                Control = references ? s.ControlMin : float.NaN;
                Predicted = s.VelocityValid ? s.PredictedMin : float.NaN;
                Velocity = s.VelocityValid ? s.CornerVelocity : float.NaN;
                Age = s.SourceAge; Gap = s.AnchorGap;
                bool hip = s.HeightValid && s.ParentReady;
                HipRaw = hip ? s.RawHipY - floor : float.NaN;
                HipReference = hip ? s.ReferenceHipY - floor : float.NaN;
                HipActual = hip ? s.ActualHipY - floor : float.NaN;
                bool knee = s.HeightValid && s.KneeHeightsValid;
                KneeRaw = knee ? s.KneeHeights.x - floor : float.NaN;
                KneeGoal = knee ? s.KneeHeights.y - floor : float.NaN;
                KneeActual = knee ? s.KneeHeights.z - floor : float.NaN;
                bool ankle = s.HeightValid && s.AnkleHeightsValid;
                AnkleRaw = ankle ? s.AnkleHeights.x - floor : float.NaN;
                AnkleLower = ankle ? s.AnkleHeights.y - floor : float.NaN;
                AnkleFoot = ankle ? s.AnkleHeights.z - floor : float.NaN;
            }

            internal void Append(StringBuilder b, double end)
            {
                b.Append("\nH144,").Append(Local).Append(',').Append(Source).Append(',').Append(Number((Time-end)*1000))
                    .Append(',').Append(Phase).Append(',').Append(SourcePhase ?? "NA").Append(',').Append(Flags).Append(',').Append(Rejections);
                Mm(b, Min); Mm(b, Raw); Mm(b, Control); Mm(b, Predicted); Mm(b, Velocity); Mm(b, Age); Mm(b, Gap);
                Mm(b, HipRaw); Mm(b, HipReference); Mm(b, HipActual); Mm(b, KneeRaw); Mm(b, KneeGoal); Mm(b, KneeActual);
                Mm(b, AnkleRaw); Mm(b, AnkleLower); Mm(b, AnkleFoot);
            }
        }

        // These historical probe payloads are large and have no consumer in either formal arm.
        // Keep them out of the half-second physical history, including every saved event copy.
        private struct EarlyDetail
        {
            internal SwingKneeCandidate.Result Knee;
            internal RightFootLandingWindow.Readback Landing;
        }

        private sealed class Record
        {
            internal Trigger Kind;
            internal MissEnd? End;
            internal int Epoch, Shared, Support;
            internal bool Full, Limited, HeightMeasured;
            internal int DepthLatchFixed, PrecontactOrigin;
            internal string FootSide;
            internal SourceRotationCoverage.Kind SourceRotation, BoundaryRotation;
            internal int BoundaryLocal;
            internal EarlyLandingWindow.Foot EarlyFoot;
            internal Point[] Points;
            internal EarlyDetail[] EarlyRows;
            internal RightKneePivot.Frame[] Pivot;
        }

        private readonly Point[] _ring = new Point[RingCapacity];
        private RightKneePivot.Frame[] _pivotRing;
        private EarlyDetail[] _earlyRing;
        private readonly List<Record> _records = new List<Record>();
        private readonly int[] _totals = new int[2], _saved = new int[2], _full = new int[2], _limited = new int[2];
        private int _head, _count;
        private double _start, _lastOverwritten = double.NegativeInfinity;
        private Record _openDetail;
        private bool _enabled;

        internal void Reset()
        {
            ClearWindow();
            _records.Clear(); _openDetail = null; _enabled = false; _pivotRing = null; _earlyRing = null;
            Array.Clear(_totals); Array.Clear(_saved); Array.Clear(_full); Array.Clear(_limited);
        }

        internal void ClearWindow()
        {
            _head = _count = 0;
            _lastOverwritten = double.NegativeInfinity;
        }

        internal void Add(in Sample s, double time, Rejection rejects, in RightKneePivot.Frame pivot = default)
        {
            _enabled = true;
            if (_count == 0) _start = time;
            // Expire by elapsed Fixed dt, including at a changing solve rate. Keep the exact 500 ms endpoint.
            while (_count > 0 && time - _ring[_head].Time > Horizon + Epsilon)
            {
                _head = (_head + 1) % RingCapacity; _count--;
            }
            if (_count == RingCapacity)
            {
                _lastOverwritten = _ring[_head].Time;
                _head = (_head + 1) % RingCapacity; _count--;
            }
            int slot = (_head + _count) % RingCapacity;
            _ring[slot] = new Point(s, time, rejects);
            if (s.EarlyFoot != EarlyLandingWindow.Foot.Unknown)
            {
                _earlyRing ??= new EarlyDetail[RingCapacity];
                _earlyRing[slot] = new EarlyDetail { Knee = s.SwingCandidate, Landing = s.RightLandingRead };
            }
            else if (_earlyRing != null) _earlyRing[slot] = default;
            if (pivot.State != RightKneePivot.Capture.Disabled)
                _pivotRing ??= new RightKneePivot.Frame[RingCapacity];
            if (_pivotRing != null) _pivotRing[slot] = pivot;
            _count++;
        }

        internal void Capture(in Sample s, bool miss)
        {
            if (_count == 0) return;
            Trigger kind = miss ? Trigger.FirstMiss : Trigger.Ordinary;
            int index = (int)kind;
            double now = _ring[(_head + _count - 1) % RingCapacity].Time;
            bool limited = now - _lastOverwritten <= Horizon + Epsilon;
            bool full = !limited && now - _start + Epsilon >= Horizon;
            _totals[index]++;
            if (limited) _limited[index]++;
            if (full) _full[index]++;
            if (_saved[index] >= DetailPerKind) return;
            var record = new Record { Kind = kind, Epoch = s.Step.Epoch, Shared = s.Step.Sequence,
                Support = s.SupportId, Full = full, Limited = limited,
                DepthLatchFixed = s.DepthLatchFixed, PrecontactOrigin = s.PrecontactOrigin, FootSide = s.FootSide, EarlyFoot = s.EarlyFoot, HeightMeasured = s.ParentHeightMeasured, Points = new Point[_count] };
            int first = Math.Min(_count, RingCapacity - _head);
            Array.Copy(_ring, _head, record.Points, 0, first);
            Array.Copy(_ring, 0, record.Points, first, _count - first);
            // Freeze evidence at both clocks. A later source jump cannot explain a depth
            // latch that already rejected the same landing on an earlier solve.
            var lastPoint = record.Points[_count - 1];
            bool current = lastPoint.Local == s.Fixed && lastPoint.Epoch == s.Step.Epoch &&
                lastPoint.Source == s.Source.Sequence && lastPoint.Support == s.SupportId && s.SupportId != 0;
            record.SourceRotation = current ? lastPoint.Rotation : SourceRotationCoverage.Kind.Unknown;
            record.BoundaryLocal = miss && s.DepthLatchFixed > 0 ? s.DepthLatchFixed : s.Fixed;
            if (record.BoundaryLocal == s.Fixed) record.BoundaryRotation = record.SourceRotation;
            else
            {
                for (int n = _count - 2; n >= 0; --n)
                {
                    ref var point = ref record.Points[n];
                    if (point.Local < record.BoundaryLocal) break;
                    if (point.Local == record.BoundaryLocal && point.Epoch == record.Epoch && point.Support == record.Support)
                    { record.BoundaryRotation = point.Rotation; break; }
                }
            }
            if (_earlyRing != null && record.EarlyFoot != EarlyLandingWindow.Foot.Unknown)
            {
                record.EarlyRows = new EarlyDetail[_count];
                Array.Copy(_earlyRing, _head, record.EarlyRows, 0, first);
                Array.Copy(_earlyRing, 0, record.EarlyRows, first, _count - first);
            }
            if (_pivotRing != null && record.EarlyFoot == EarlyLandingWindow.Foot.Right)
            {
                record.Pivot = new RightKneePivot.Frame[_count];
                Array.Copy(_pivotRing, _head, record.Pivot, 0, first);
                Array.Copy(_pivotRing, 0, record.Pivot, first, _count - first);
            }
            _records.Add(record); _saved[index]++;
            if (miss) _openDetail = record;
        }

        internal void EndMiss(MissEnd end)
        {
            if (_openDetail != null) _openDetail.End = end;
            _openDetail = null;
        }

        internal void Append(StringBuilder b)
        {
            b.Append("\nLANDING_HISTORY_V041440/sampledInactive=").Append(_enabled)
                .Append("/horizonMs=500/ringCapacity=").Append(RingCapacity).Append("/detailPerKind=").Append(DetailPerKind)
                .Append("/columns=events-saved-omitted-fullHalfSecond-capacityLimited");
            for (int k = 0; k < _totals.Length; k++)
                b.Append('/').Append((Trigger)k).Append('=').Append(_totals[k]).Append('|').Append(_saved[k])
                    .Append('|').Append(_totals[k]-_saved[k]).Append('|').Append(_full[k]).Append('|').Append(_limited[k]);
            if (_records.Count == 0) return;
            b.Append("\nLANDING_HISTORY_COLUMNS_V041440/local,source,offsetMs,gait,sourcePhase,flags,rejectMask,")
                .Append("boxActualMm,boxSourceMm,boxControlMm,predictedMm,cornerMmPerSec,ageMs,gapMm,")
                .Append("hipSourceMm,hipReferenceMm,hipPhysicalMm,kneeSourceMm,kneeGoalMm,kneePhysicalMm,ankleSourceMm,ankleLowerMm,ankleFootMm")
                .Append("/flags=strong1+identity2+fivePoint4+footprint8+adopted16+fallback32/rejectBits=geometry1+footprint2+motion4+age8+gap16+fivePoint32+identity64+height128+descending256+projection512");
            if (_pivotRing != null)
                b.Append("\nRIGHT_PIVOT_V041490/window=RIGHT_STEADY_SWING_PRE4_AND_CENTER/align=LIVE_WARNING_AND_LAST_STEP_BEFORE_STRONG")
                    .Append("/model=SAME_STATE_POSE_ONLY/full=DOMINANT_TERM_TO_TARGET/recent=INCOMING_TRAVEL_SCALED_TO_DT")
                    .Append("/trialColumns=angleDeg-kneeRiseMm-ankleRiseMm-boxMm-kneeErrorGainMm-ankleErrorGainMm-penetrationGainMm-addedClearanceMm-inside-baseBand-trialBand")
                    .Append("/boxBandMm=-2|25/gapOver5=HISTORY_ONLY/feedbackToControl=False");
            foreach (var r in _records)
            {
                var last = r.Points[r.Points.Length-1];
                b.Append("\nLANDING_HISTORY_EVENT_V041440/kind=").Append(r.Kind).Append("/outcome=")
                    .Append(r.Kind == Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open")
                    .Append("/epoch-shared-local-source=").Append(r.Epoch).Append('|').Append(r.Shared).Append('|').Append(last.Local).Append('|').Append(last.Source)
                    .Append("/support=").Append(r.Support).Append("/samples=").Append(r.Points.Length)
                    .Append("/coverage=").Append(r.Limited ? "CAPACITY" : r.Full ? "FULL" : "LEFT_CENSORED");
                if (r.FootSide != null) b.Append("/side=").Append(r.FootSide).Append("/rightPrecontactOriginLocal=").Append(r.PrecontactOrigin);
                b.Append("/sourceRotation250ms=").Append(r.SourceRotation).Append("/sourceMotion=").Append(last.SourcePhase ?? "Unknown")
                    .Append("/rotationBoundaryLocal=").Append(r.BoundaryLocal).Append("/rotationAtBoundary=").Append(r.BoundaryRotation);
                foreach (var point in r.Points) point.Append(b, last.Time);
                if (r.EarlyRows != null) AppendEarly(b, r);
            }
        }
        private static int FindLocal(Point[] points, int local)
        {
            int i = points.Length - 1;
            while (i >= 0 && points[i].Local != local) i--;
            return i;
        }

        private static int StrongStart(Point[] points, int i)
        {
            if (i < 0 || (points[i].Flags & 3) != 3) return -1;
            while (i > 0 && (points[i-1].Flags & 3) == 3) i--;
            return i;
        }

        private static int CycleStart(Point[] points, int anchor)
        {
            int start = anchor;
            while (start > 0 && !points[start-1].PreviousEvent &&
                !LandingWindowOutcomes.StartsNewSwing(points[start].Phase, points[start-1].Phase)) start--;
            return start;
        }

        private static string Lead(Point[] points, int anchor, int i) =>
            i < 0 ? "NA" : Number((points[anchor].Time - points[i].Time) * 1000d);

        private static void AppendEarly(StringBuilder b, Record r)
        {
            var points = r.Points;
            int end = points.Length - 1;
            bool left = r.EarlyFoot == EarlyLandingWindow.Foot.Left, miss = r.Kind == Trigger.FirstMiss;
            int depth = miss ? FindLocal(points,r.DepthLatchFixed) : -1;
            int strong = StrongStart(points,miss ? depth : end);
            bool depthAnchor = left && miss;
            int anchor = depthAnchor ? depth : strong;
            b.Append("\nEARLY_EVENT_V041460/side=").Append(r.EarlyFoot).Append("/kind=").Append(r.Kind)
                .Append("/outcome=").Append(miss ? r.End?.ToString() ?? "Open" : "Enabled")
                .Append("/eventLocal=").Append(points[end].Local).Append("/anchor=").Append(depthAnchor ? "DEPTH_LATCH" : "STRONG_EXACT_START")
                .Append("/depthLocal=").Append(r.DepthLatchFixed).Append("/strongStartLocal=")
                .Append(strong > 0 ? points[strong].Local.ToString(CultureInfo.InvariantCulture) : "NA");
            if (anchor < 0 || (!depthAnchor && anchor == 0))
            {
                b.Append("/anchorCaptured=False");
                if (r.HeightMeasured) AppendPreparationEvent(b,r,-1,0);
                if (!left && points[end].RightLandingObserved) AppendRightFootLanding(b,r,-1);
                return;
            }
            b.Append("/strongEdge=");
            if (strong > 0)
                b.Append(points[strong-1].Phase).Append('|').Append(points[strong-1].SourcePhase ?? "NA")
                    .Append('|').Append(points[strong].Phase).Append('|').Append(points[strong].SourcePhase ?? "NA");
            else b.Append("NA");
            int start = CycleStart(points,anchor);
            int phaseSteps=0, eligible=0, origins=0, trials=0, good=0, broad=0, near=0;
            int firstBroad=-1, firstNear=-1, firstOrigin=-1;
            for (int i=start; i<anchor; i++)
            {
                ref readonly var pt = ref points[i];
                var sig = pt.Early;
                if (left ? pt.Phase == FootLandingOpportunityAudit.Phase.Lift :
                    pt.Phase == FootLandingOpportunityAudit.Phase.Swing && pt.SourcePhase == "STEADY") phaseSteps++;
                if (sig == EarlyLandingWindow.Signal.LeftLift || sig == EarlyLandingWindow.Signal.LeftStart ||
                    sig == EarlyLandingWindow.Signal.LeftContinue || sig >= EarlyLandingWindow.Signal.RightSteady) eligible++;
                if (sig == EarlyLandingWindow.Signal.LeftStart) { origins++; if (firstOrigin < 0) firstOrigin=i; }
                if (EarlyLandingWindow.LeftTrial(sig))
                { trials++; if (r.EarlyRows[i].Knee.Reason == SwingKneeCandidate.Status.Favorable) good++; }
                if (EarlyLandingWindow.RightLow(sig)) { broad++; if (firstBroad < 0) firstBroad=i; }
                if (sig == EarlyLandingWindow.Signal.RightNearLow) { near++; if (firstNear < 0) firstNear=i; }
            }
            b.Append("/anchorCaptured=True/anchorLocal=").Append(points[anchor].Local)
                .Append("/availableBeforeMs=").Append(Lead(points,anchor,start)).Append("/cycleBoundarySeen=").Append(start > 0)
                .Append("/phaseSteps-eligiblePhaseSteps=").Append(phaseSteps).Append('|').Append(eligible)
                .Append("/leftStarts-trials-favorable=").Append(origins).Append('|').Append(trials).Append('|').Append(good)
                .Append("/rightLow-nearLow=").Append(broad).Append('|').Append(near)
                .Append("/firstLeadMs(leftOrigin-rightLow-rightNearLow)=").Append(Lead(points,anchor,firstOrigin)).Append('|')
                .Append(Lead(points,anchor,firstBroad)).Append('|').Append(Lead(points,anchor,firstNear));
            for (int i=start; i<anchor; i++)
            {
                ref readonly var pt = ref points[i];
                if (!EarlyLandingWindow.LeftTrial(pt.Early) && pt.Early != EarlyLandingWindow.Signal.RightNearLow) continue;
                b.Append("\nEARLY_STEP_V041460/local=").Append(pt.Local).Append("/source=").Append(pt.Source)
                    .Append("/signal=").Append(pt.Early).Append("/originLocal=").Append(pt.Origin)
                    .Append("/leadMs=").Append(Lead(points,anchor,i)).Append("/rawBox-physicalBox-lowerMinusSourceAnkleMm=");
                b.Append(Number(1000d * pt.Raw)).Append('|').Append(Number(1000d * pt.Min)).Append('|')
                    .Append(Number(1000d * (pt.AnkleLower-pt.AnkleRaw)));
                if (EarlyLandingWindow.LeftTrial(pt.Early)) b.Append("/result=").Append(r.EarlyRows[i].Knee.EarlyDetail());
            }
            if (r.HeightMeasured) AppendPreparationEvent(b,r,anchor,start);
            if (!left && points[end].RightLandingObserved) AppendRightFootLanding(b,r,anchor);
        }
        private static void AppendPreparationEvent(StringBuilder b, Record r, int anchor, int start)
        {
            var p=r.Points;
            b.Append("\nLANDING_PREPARATION_EVENT_V041470/side=").Append(r.EarlyFoot)
                .Append("/eventLocal=").Append(p[p.Length-1].Local).Append("/kind=").Append(r.Kind)
                .Append("/outcome=").Append(r.Kind==Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open")
                .Append("/anchorCaptured=").Append(anchor>=0);
            if (anchor<0)
            {
                if (r.Pivot != null) { AppendPivotWindow(b, r, start, -1, -1, "Edge"); AppendThighEvent(b, r, start, -1); }
                return;
            }
            b.Append("/anchorLocal=").Append(p[anchor].Local);
            if (r.EarlyFoot==EarlyLandingWindow.Foot.Left)
            {
                int count=0, first=-1;
                for (int i=start;i<anchor;i++)
                    if (p[i].HeightRequested) { count++; if (first<0) first=i; }
                b.Append("/heightTrials=").Append(count).Append("/firstHeightLeadMs=").Append(Lead(p,anchor,first));
                return;
            }
            int firstNear=-1, lastCancel=-1;
            for (int i=start;i<anchor;i++)
            {
                if (firstNear<0 && p[i].Early==EarlyLandingWindow.Signal.RightNearLow) firstNear=i;
                if (i>start && p[i-1].RightWarningOrigin!=0 && p[i].RightWarningOrigin!=p[i-1].RightWarningOrigin) lastCancel=i;
            }
            // Use only the alert that was still live on the step preceding the observed strong edge.
            int origin=anchor>start ? p[anchor-1].RightWarningOrigin : 0;
            int index=origin>0 ? FindLocal(p,origin) : -1;
            b.Append("/firstNearLeadMs=").Append(Lead(p,anchor,firstNear))
                .Append("/lastCancelLocal=").Append(lastCancel<0 ? 0 : p[lastCancel].Local)
                .Append("/heldBefore=").Append(origin>0).Append("/heldOrigin=").Append(origin)
                .Append("/heldLeadMs=").Append(Lead(p,anchor,index>=start ? index : -1));
            for (int i=start;i<anchor;i++)
            {
                if (p[i].RightWarningOrigin!=p[i].Local || p[i].Early!=EarlyLandingWindow.Signal.RightNearLow) continue;
                int stop=i+1;
                while (stop<anchor && p[stop].RightWarningOrigin==p[i].Local) stop++;
                b.Append("\nRIGHT_ANKLE_EVENT_V041480/epoch-support=").Append(r.Epoch).Append('|').Append(r.Support)
                    .Append("/eventLocal=").Append(p[p.Length-1].Local).Append("/kind=").Append(r.Kind)
                    .Append("/outcome=").Append(r.Kind==Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open")
                    .Append("/onset=").Append(p[i].Local).Append("/leadMs=").Append(Lead(p,anchor,i))
                    .Append("/heldToEdge=").Append(stop==anchor).Append("/cancelLocal=").Append(stop<anchor ? p[stop].Local : 0);
                if (r.Pivot != null && i != anchor - 1)
                    AppendPivotWindow(b, r, start, i, anchor, "Warning");
            }
            if (r.Pivot != null)
            {
                int before = anchor - 1;
                bool isOnset = before >= start && p[before].RightWarningOrigin == p[before].Local &&
                    p[before].Early == EarlyLandingWindow.Signal.RightNearLow;
                AppendPivotWindow(b, r, start, before, anchor, isOnset ? "WarningAndEdge" : "Edge");
                AppendThighEvent(b, r, start, anchor);
            }
        }
        private static void AppendThighEvent(StringBuilder b, Record r, int start, int anchor)
        {
            var p = r.Points;
            b.Append("\nRIGHT_THIGH_EVENT_V041500/epoch-support=").Append(r.Epoch).Append('|').Append(r.Support)
                .Append("/eventLocal=").Append(p[p.Length-1].Local).Append("/kind=").Append(r.Kind)
                .Append("/outcome=").Append(r.Kind==Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open");
            if (anchor < 0)
            {
                b.Append("/anchorCaptured=False");
                AppendFootPoseEvent(b, r, anchor, 0, 0);
                return;
            }
            int first = -1, last = -1, onsets = 0, held = 0;
            for (int i = start; i < anchor; i++)
            {
                if (p[i].RightThigh == RightThighWindow.Signal.Held) held++;
                if (p[i].RightThigh != RightThighWindow.Signal.Onset) continue;
                onsets++;
                if (first < 0) first = i;
                last = i;
            }
            b.Append("/anchorLocal=").Append(p[anchor].Local).Append("/coveredPriorSteps=").Append(anchor-start)
                .Append("/onsets-held=").Append(onsets).Append('|').Append(held)
                .Append("/first-lastOnsetLocal=").Append(first < 0 ? 0 : p[first].Local).Append('|').Append(last < 0 ? 0 : p[last].Local)
                .Append("/firstLeadMs=").Append(Lead(p,anchor,first));
            AppendFootPoseEvent(b, r, anchor, first < 0 ? 0 : p[first].Local, last < 0 ? 0 : p[last].Local);
        }
        private static void AppendFootPoseEvent(StringBuilder b, Record r, int anchor, int first, int last)
        {
            var points = r.Points;
            b.Append("\nRIGHT_FOOT_POSE_EVENT_V041510/epoch-support=").Append(r.Epoch).Append('|').Append(r.Support)
                .Append("/eventLocal=").Append(points[points.Length-1].Local).Append("/kind=").Append(r.Kind)
                .Append("/outcome=").Append(r.Kind==Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open")
                .Append("/anchorCaptured=").Append(anchor >= 0);
            if (anchor < 0) return;
            var pt = points[anchor];
            // All eight command corners translate by exactly the physical vertical ankle separation.
            // Geometry at the first strong step is observed independently of any preceding onset.
            var goal = pt.Control + (pt.AnkleFoot - pt.AnkleLower);
            bool geometry = (pt.Flags & (2|4|8|32)) == (2|4|8) && float.IsFinite(pt.Min) &&
                float.IsFinite(goal) && float.IsFinite(pt.Gap) && pt.Gap >= 0f;
            bool fresh = float.IsFinite(pt.Age) && pt.Age >= 0f && pt.Age <= FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds;
            bool comparable = geometry && fresh;
            b.Append("/anchorLocal=").Append(pt.Local).Append("/first-latestOrigin=").Append(first).Append('|').Append(last)
                .Append("/comparable=").Append(comparable).Append("/flags-rejections=").Append(pt.Flags).Append('|').Append(pt.Rejections)
                .Append("/geometryComparable=").Append(geometry).Append("/sourceFresh=").Append(fresh)
                .Append("/sourceAgeMs=").Append(Number(1000d*pt.Age))
                .Append("/gapMm=").Append(Number(1000d*pt.Gap)).Append("/actual-goalAtFootMm=").Append(Number(1000d*pt.Min)).Append('|').Append(Number(1000d*goal))
                .Append("/actual-goalBand=").Append(RightFootTorqueForecast.Band(comparable ? pt.Min : float.NaN)).Append('|')
                .Append(RightFootTorqueForecast.Band(comparable ? goal : float.NaN));
        }
        private static void AppendRightFootLanding(StringBuilder b, Record r, int anchor)
        {
            AppendAnkleSequence(b,r,anchor);
            var p=r.Points;
            b.Append("\nRIGHT_FOOT_LANDING_EVENT_V041520/epoch-support=").Append(r.Epoch).Append('|').Append(r.Support)
                .Append("/eventLocal=").Append(p[p.Length-1].Local).Append("/kind=").Append(r.Kind)
                .Append("/outcome=").Append(r.Kind==Trigger.Ordinary ? "Enabled" : r.End?.ToString() ?? "Open")
                .Append("/anchorCaptured=").Append(anchor>=0);
            if (anchor<0) return;
            int start=CycleStart(p,anchor), read=0, opportunities=0, overGap=0, favorable=0, protectedSteps=0, goalOutside=0, entersBand=0;
            int first=-1, firstTrial=-1, lastTrial=-1, gainAboveError=0;
            float maxGain=0f;
            // Each result belongs to [from, from+1]. The edge can close a prediction,
            // but no command from the edge itself or the previous cycle can enter.
            for(int i=start+1;i<=anchor;i++)
            {
                var n=r.EarlyRows[i].Landing;
                if (n.From<=0 || n.From!=p[i-1].Local || n.From+1!=p[i].Local) continue;
                read++;
                if (n.State==RightFootLandingForecast.Status.ProtectInBand) protectedSteps++;
                if (n.State==RightFootLandingForecast.Status.GoalOutsideBand) goalOutside++;
                if (n.State>=RightFootLandingForecast.Status.OverGap) { opportunities++; if(first<0) first=i-1; }
                if (n.State==RightFootLandingForecast.Status.OverGap) overGap++;
                if (n.State!=RightFootLandingForecast.Status.Favorable) continue;
                favorable++; if(firstTrial<0) firstTrial=i-1;
                lastTrial=i-1; float gain=n.Trial-n.Baseline;
                if (RightFootTorqueForecast.Band(n.Trial)==RightFootTorqueForecast.HeightBand.InBand) entersBand++;
                if (gain>Math.Abs(n.Actual-n.Baseline)) gainAboveError++;
                maxGain=Math.Max(maxGain,gain);
            }
            b.Append("/anchorLocal=").Append(p[anchor].Local).Append("/coveredIntervals-read=").Append(anchor-start).Append('|').Append(read)
                .Append("/geometryOpportunities-over5-protected-goalOutside=").Append(opportunities).Append('|').Append(overGap)
                .Append('|').Append(protectedSteps).Append('|').Append(goalOutside)
                .Append("/firstOpportunityLocal=").Append(first<0?0:p[first].Local)
                .Append("/first-lastTrial=").Append(firstTrial<0?0:p[firstTrial].Local).Append('|').Append(lastTrial<0?0:p[lastTrial].Local)
                .Append("/favorable-gainAboveError=").Append(favorable).Append('|').Append(gainAboveError)
                .Append("/entersBand=").Append(entersBand).Append("/first-lastTrialLeadMs=").Append(Lead(p,anchor,firstTrial)).Append('|').Append(Lead(p,anchor,lastTrial))
                .Append("/maxPredictedGainMm=").Append(favorable==0?"NA":Number(1000d*maxGain));
        }
        private static void Mm(StringBuilder b, float v) => b.Append(',').Append(Number(v * 1000d));
        private static string Number(double v) => double.IsFinite(v) ? v.ToString("G8", CultureInfo.InvariantCulture) : "NA";
    }
}
