using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Entry = MakemitAGA.ActiveRagdollLab.Physics.OrdinaryFootEntry;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.LandingFootTraction;
using Leg = MakemitAGA.ActiveRagdollLab.Physics.TurnHandoffTrajectory;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Only issue-time data enters ranking. The following measured pose may discard a
    // frozen path. It cannot retroactively select a discarded sibling or restart it.
    internal sealed partial class CombinedLandingSearch
    {
        internal const int Width = 4;
        internal enum ReadGate { Unread, LegBounds, LegHeight, LegHorizontal, LowerHorizontal, FootBounds, FootHeight, FootHorizontal, Accepted }

        internal struct ShoeLoss
        {
            internal float AnkleXZ, SoleXZ, AnkleAlong, SoleAlong;
            internal ShoeLoss(in Leg.Measure m, in Leg.Measure native)
            {
                AnkleXZ = m.AnkleXZ - native.AnkleXZ;
                SoleXZ = m.SoleXZ - native.SoleXZ;
                AnkleAlong = Mathf.Abs(m.AnkleAlong) - Mathf.Abs(native.AnkleAlong);
                SoleAlong = Mathf.Abs(m.SoleAlong) - Mathf.Abs(native.SoleAlong);
            }
        }

        internal struct LowerLoss
        {
            internal float XZ, Along;
            internal LowerLoss(in Leg.Measure m, in Leg.Measure native, in Leg.Aim aim)
            {
                var lower = FootTransitionSource.Planar(m.Lower - aim.Target);
                var original = FootTransitionSource.Planar(native.Lower - aim.Target);
                XZ = lower.magnitude - original.magnitude;
                Along = Mathf.Abs(Vector3.Dot(lower, aim.Travel)) - Mathf.Abs(Vector3.Dot(original, aim.Travel));
            }
            internal float Worst => Mathf.Max(XZ, Along);
        }

        internal struct Path
        {
            internal int Id, Parent, MarginSamples;
            internal bool Protected, HasHorizontalRead, PreviewSlot;
            internal Evidence Horizon;
            internal ShoeLoss PreTraction, PostTraction;
            internal LowerLoss IssueLower, ReadLower;
            // Both body anchors have the same leg-induced shift. Traction moves only the shoe.
            // Remember any excess since release, even when the same ankle improves at the first opportunity.
            internal int FirstPreExcess, FirstLowerExcess;
            internal float PeakLowerLoss;
            internal ReadGate Gate;
            internal bool SafeRead => Gate == ReadGate.Accepted;
            internal Leg.Plan Leg;
            internal Foot.Move Move;
            internal Entry.Trail Entry;
            internal float LeastMargin;
        }

        internal sealed class Step
        {
            internal Path[] Paths = Array.Empty<Path>();
            internal Leg.Pose PhysicalPose;
            internal PhysicsStepContext IssueClock, VerifiedReadClock;
            internal bool HasPhysicalPose;
            internal Foot.Input Command;
            internal Vector3 StartGap;
            internal int Parents, LegOptions, Forecasts, Count, SafeReads;
            internal bool HaveHorizon;
            internal int JointChecked, Shortlisted, ExtraLegs, NextCommands;
            internal float? PreviewPeakShift; // Metres, frozen at issue for the retained strong, buffered preview.
        }

        private enum Preference { LegHeight, JointRoom, Diversity }

        private readonly bool _usePreview;
        internal CombinedLandingSearch(bool usePreview = true) { _usePreview = usePreview; }

        private Leg.Options _options = new Leg.Options();
        private Path[] _pool = new Path[Width * Leg.Options.Capacity];
        private int _count, _nextId;
        internal string Outcome { get; private set; }
        internal int EndLocal { get; private set; }
        internal Path LastRead { get; private set; }
        internal int LastReadLocal { get; private set; }

        // Event rows are retained for the stop-time report. The larger ranking buffer
        // belongs only to an active search and can be dropped on either stop path.
        internal void ReleaseScratch()
        {
            _pool = null;
            _options = null;
            _preview = null;
        }

        private void End(int local, string reason)
        {
            if (Outcome != null) return;
            Outcome = reason;
            EndLocal = local;
            ReleaseScratch();
        }

        internal Step Issue(int local, float dt, float hipY, in Leg.Pose pose, in Leg.Aim aim, in Leg.Budget budget,
            in Foot.Input command, Vector3 startGap, Step previous)
        {
            var step = new Step { Command = command, StartGap = startGap };
            if (Outcome != null) return step;
            bool hasCommand = command.Valid;
            _count = 0;
            Leg.Pose horizon = default;
            step.HaveHorizon = _usePreview && hasCommand && previous != null && previous.HasPhysicalPose &&
                TryHorizon(previous.PhysicalPose, pose, budget, dt, out horizon);
            if (step.HaveHorizon) (_preview ??= new PreviewScratch()).HasNominee = false;
            int pinned = -1;
            int parents = previous == null ? 1 : previous.Count;
            for (int i = 0; i < parents; i++)
            {
                var parent = previous == null ? new Path { Protected = true } : previous.Paths[i];
                if (previous != null && !parent.SafeRead) continue;
                step.Parents++;
                bool consider = step.HaveHorizon && parent.FirstLowerExcess == 0;
                var extra = consider ? _preview.Extra : null;
                var standard = Leg.Select(pose, aim, budget, dt, hipY, parent.Leg.Offset, true, true, _options, extra, distinctAccelerations: true);
                step.LegOptions += _options.Count;
                if (!hasCommand) continue;
                int strictCount = _options.Count, extraCount = extra?.Count ?? 0;
                step.ExtraLegs += extraCount;
                for (int j = 0; j < strictCount + extraCount; j++)
                {
                    bool strict = j < strictCount;
                    var leg = strict ? _options.Items[j] : extra.Items[j - strictCount];
                    var lower = consider ? new LowerLoss(leg.Trial, leg.Base, aim) : default;
                    bool joint = consider && lower.Worst <= Leg.Precision;
                    if (!strict && !joint) continue;
                    var move = Foot.Predict(pose, leg, aim, command, dt, startGap, parent.Move, previous != null, true);
                    bool original = strict && parent.Protected && standard.Issued && SameAngles(standard.Offset, leg.Offset);
                    var path = new Path {
                        Leg = leg, Move = move, Parent = parent.Id, Protected = original,
                        FirstPreExcess = parent.FirstPreExcess, FirstLowerExcess = parent.FirstLowerExcess,
                        PeakLowerLoss = parent.PeakLowerLoss, Entry = parent.Entry, MarginSamples = parent.MarginSamples, LeastMargin = parent.LeastMargin
                    };
                    if (move.Status == Foot.State.Predicted && strict)
                    {
                        if (original) pinned = _count;
                        _pool[_count++] = path;
                    }
                    if (!joint) continue;
                    path.Move = Foot.Balance(pose,leg,aim,command,dt,parent.Move.Body,move);
                    if (path.Move.Status != Foot.State.Predicted) continue;
                    if (!strict || path.Move.PlanarAdjusted) step.Forecasts++;
                    if (!strict && path.FirstPreExcess == 0) path.FirstPreExcess = local;
                    path.IssueLower = lower;
                    step.JointChecked++;
                    Offer(path, horizon, aim, dt);
                }
            }
            if (!hasCommand) { End(local, "MissingFootCommand"); return step; }
            step.Forecasts += _count;
            if (_count > 0 || (step.HaveHorizon && _preview.HasNominee)) step.Paths = new Path[Width];
            // Keep the continuous control, the height choice and room for different joint velocities.
            // A prediction never consumes a fifth slot, even when an old state shares its witness.
            if (pinned >= 0) Keep(step, pinned);
            Pick(step, Preference.LegHeight, dt);
            if (!step.HaveHorizon || step.Count < 2) Pick(step, Preference.JointRoom, dt);
            if (step.HaveHorizon && step.Count < Width - 1) Pick(step, Preference.Diversity, dt);
            if (step.HaveHorizon) PickPreview(step, horizon, pose, aim, budget, dt, hipY);
            while (step.Count < Width && Pick(step, Preference.Diversity, dt)) { }
            for (int i = 0; i < step.Count; i++)
            {
                ref var p = ref step.Paths[i];
                p.IssueLower = new LowerLoss(p.Leg.Trial, p.Leg.Base, aim);
                ObserveLower(ref p, p.IssueLower, local);
            }
            if (step.Count == 0) End(local, step.LegOptions == 0 && step.ExtraLegs == 0 ? "NoLegContinuation" : "NoSafeFootForecast");
            return step;
        }

        private static void ObserveLower(ref Path path, in LowerLoss loss, int local)
        {
            path.PeakLowerLoss = Mathf.Max(path.PeakLowerLoss, loss.Worst);
            if (path.FirstLowerExcess == 0 && loss.Worst > Leg.Precision) path.FirstLowerExcess = local;
        }

        private static bool SameAngles(in Leg.Offset a, in Leg.Offset b) =>
            a.Hip == b.Hip && a.Knee == b.Knee && a.HipRate == b.HipRate && a.KneeRate == b.KneeRate;

        private static bool SameVector(Vector3 a, Vector3 b) => a.x == b.x && a.y == b.y && a.z == b.z;

        private static bool SameState(in Path a, in Path b) =>
            SameAngles(a.Leg.Offset, b.Leg.Offset) &&
            SameVector(a.Move.Body.Position, b.Move.Body.Position) && SameVector(a.Move.Body.Velocity, b.Move.Body.Velocity) &&
            a.Entry.StrongStart == b.Entry.StrongStart && a.Entry.StrongSamples == b.Entry.StrongSamples &&
            a.Entry.DepthLatch == b.Entry.DepthLatch && a.Entry.ReadySamples == b.Entry.ReadySamples &&
            a.MarginSamples == b.MarginSamples && a.LeastMargin == b.LeastMargin &&
            (a.FirstPreExcess == 0) == (b.FirstPreExcess == 0) &&
            (a.FirstLowerExcess == 0) == (b.FirstLowerExcess == 0);

        private void Keep(Step step, int index)
        {
            var p = _pool[index];
            p.Id = ++_nextId;
            step.Paths[step.Count++] = p;
            _pool[index] = _pool[--_count];
        }

        private bool Pick(Step step, Preference preference, float dt)
        {
            if (step.Count == Width || _count == 0) return false;
            int best = -1;
            float bestScore = 0f;
            for (int i = 0; i < _count; i++)
            {
                ref var p = ref _pool[i];
                bool duplicate = false;
                for (int j = 0; j < step.Count; j++)
                    if (SameState(p, step.Paths[j])) { duplicate = true; break; }
                if (duplicate) continue;
                var score = Score(p, preference, dt, step);
                int priority = best < 0 ? -1 : CompareGeometry(p, _pool[best]);
                if (priority > 0 || (priority == 0 && score >= bestScore)) continue;
                best = i;
                bestScore = score;
            }
            if (best < 0) return false;
            Keep(step, best);
            return true;
        }

        // Favor paths still eligible in this same strong interval, then the two
        // quarter-millimetre margins on the current unloaded forecast.
        private static int CompareGeometry(in Path a, in Path b)
        {
            int order = (a.Entry.DepthLatch != 0).CompareTo(b.Entry.DepthLatch != 0);
            if (order != 0) return order;
            order = b.MarginSamples.CompareTo(a.MarginSamples);
            if (order != 0) return order;
            return Shortfall(a.Move.Forecast).CompareTo(Shortfall(b.Move.Forecast));
        }

        private static float Shortfall(in Leg.Measure m) =>
            Mathf.Max(0f, Mathf.Max(Foot.LandingMargin - Foot.Margin(m), m.Sole - FootContactGeometryLimits.RecoveryMaxGapMeters));

        private static float Score(in Path p, Preference preference, float dt, Step kept)
        {
            var a = p.Leg.Offset;
            if (preference == Preference.LegHeight)
                return Mathf.Max(Mathf.Abs(p.Leg.Wanted - p.Leg.Trial.LowerRise),
                    Mathf.Abs(p.Leg.Wanted - (p.Leg.Trial.Sole - p.Leg.Base.Sole)));
            if (preference == Preference.JointRoom) return a.Effort;
            // A distinct continuation stores a distinct posture *and velocity*.
            // Rank separation at the next free drift in joint radians without using a future target.
            float nearest = float.PositiveInfinity;
            for (int j = 0; j < kept.Count; j++)
            {
                var b = kept.Paths[j].Leg.Offset;
                float hip = (a.Hip - b.Hip) + dt * (a.HipRate - b.HipRate);
                float knee = (a.Knee - b.Knee) + dt * (a.KneeRate - b.KneeRate);
                nearest = Mathf.Min(nearest, hip * hip + knee * knee);
            }
            return -nearest;
        }

        internal void Read(int local, Step step, in Leg.Pose physical, in Leg.Aim aim, in Leg.Measure native,
            in Entry.Sample receipt, bool first)
        {
            if (Outcome != null) return;
            step.PhysicalPose = physical;
            step.HasPhysicalPose = true;
            Path best = default, admitted = default;
            for (int i = 0; i < step.Count; i++)
            {
                ref var path = ref step.Paths[i];
                var leg = Leg.Evaluate(physical, path.Leg.Offset, aim);
                bool preTracks = leg.Tracks(native);
                if (!preTracks && path.FirstPreExcess == 0) path.FirstPreExcess = local;
                float hardGap = step.Command.Tether.HardGap;
                Foot.SettleOnLeg(ref path.Move, physical, leg, aim, native, hardGap);
                path.PreTraction = new ShoeLoss(leg, native);
                path.PostTraction = new ShoeLoss(path.Move.Settled, native);
                path.ReadLower = new LowerLoss(leg, native, aim);
                path.HasHorizontalRead = true;
                ObserveLower(ref path, path.ReadLower, local);
                // Keep the original control's pre-traction gate. Alternative paths use their already
                // issued traction at the endpoint. The reserved slot also requires its own shank receipt.
                if (!leg.Safe) { path.Gate = ReadGate.LegBounds; continue; }
                if (!leg.MaintainsHeight(native)) { path.Gate = ReadGate.LegHeight; continue; }
                if (path.Protected && !preTracks) { path.Gate = ReadGate.LegHorizontal; continue; }
                if (path.PreviewSlot && path.FirstLowerExcess != 0) { path.Gate = ReadGate.LowerHorizontal; continue; }
                if (!path.Move.SafeRead)
                {
                    var foot = path.Move.Settled;
                    if (!foot.Safe || foot.Gap >= hardGap) path.Gate = ReadGate.FootBounds;
                    else path.Gate = foot.MaintainsHeight(native) ? ReadGate.FootHorizontal : ReadGate.FootHeight;
                    continue;
                }
                path.Gate = ReadGate.Accepted;
                step.SafeReads++;
                var decision = path.Entry.Observe(receipt, path.Move.Settled.Inside, path.Move.Settled.Sole);
                if (path.Entry.StrongSamples <= 1) { path.MarginSamples = 0; path.LeastMargin = float.PositiveInfinity; }
                if (path.Entry.StrongSamples > 0)
                {
                    if (path.Move.MarginPass) path.MarginSamples++;
                    path.LeastMargin = Mathf.Min(path.LeastMargin, path.Move.WorstMargin);
                }
                if (BetterRead(path, best)) best = path;
                if (decision == Entry.Decision.Ordinary && BetterRead(path, admitted)) admitted = path;
            }
            if (best.Id != 0) { LastRead = best; LastReadLocal = local; }
            if (admitted.Id != 0) { LastRead = admitted; End(local, "WouldEnter"); }
            else if (step.SafeReads == 0) End(local, "NoSafeRead");
            else if (first) End(local, best.Entry.DepthLatch != 0 ? "DepthLatched" : "NotReadyAtFirst");
        }

        private static bool BetterRead(in Path p, in Path current)
        {
            if (current.Id == 0) return true;
            if ((p.Entry.DepthLatch == 0) != (current.Entry.DepthLatch == 0)) return p.Entry.DepthLatch == 0;
            if (p.MarginSamples != current.MarginSamples) return p.MarginSamples > current.MarginSamples;
            if (p.LeastMargin != current.LeastMargin) return p.LeastMargin > current.LeastMargin;
            return p.Protected && !current.Protected;
        }
    }
}
