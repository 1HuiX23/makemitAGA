using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralFootTransitionTrace
    {
        private readonly Window[] _handoff = new Window[2];
        private sealed partial class Window
        {
            internal TurnHandoffWindow Handoff, Early, Braked, Combined;
            internal int FirstEligibleLocal;
            internal bool RightControlEligible;
            internal int WaitingPrecontact;
            internal string ControlExit;
        }

        private static bool TurnLift(in Tick before, in Tick at) =>
            before.Local > 0 && before.Motion == "IDLE" && at.Motion == "TURN" && at.Released &&
            at.Phase == FootLandingOpportunityAudit.Phase.Lift && at.ReleaseReason == "RELEASE_ON_TRANSITION_TO_SWING";

        private void TrackHandoffs(in Pair pair)
        {
            for (int side = 0; side < 2; side++)
            {
                var parent = _handoff[side];
                if (parent == null) continue;
                int local = side == 0 ? pair.Left.Local : pair.Right.Local;
                if (parent.Handoff.Missed(local)) parent.Handoff.Stop("MissingCompletion");
                if (parent.Early?.Missed(local) == true) parent.Early.Stop("MissingCompletion");
                if (parent.Braked?.Missed(local) == true) parent.Braked.Stop("MissingCompletion");
                if (parent.Combined?.Missed(local) == true) parent.Combined.Stop("MissingCompletion");
                if (parent.Handoff.End != null && (parent.Early == null || parent.Early.End != null) &&
                    (parent.Braked == null || parent.Braked.End != null) && (parent.Combined == null || parent.Combined.End != null)) _handoff[side] = null;
            }
            var left = _release[0];
            if (left?.Type == Kind.PairRelease && left.Origin.Foot.Local == pair.Left.Local &&
                TurnLift(_previous.Left, pair.Left) && TurnLift(_previous.Right, pair.Right))
                StartHandoff(0, left, pair.Left, FootTransitionSource.Planar(pair.Left.Source.World).normalized);

            // In 168 B the right foot was in PRECONTACT at quiet step 5327.
            // Select the episode now; defer the first free prediction until a genuinely
            // contact-free solve. Zero normal impulse is not a collider-off receipt.
            var right = _brake[1];
            if (right == null || right.Handoff != null || right.ControlExit != null) return;
            if (right.Quiet.Foot.Local == pair.Right.Local && pair.Left.Active && !pair.Right.Active &&
                right.First.Foot.Local == 0 && right.Heading.Axis.sqrMagnitude > .99f) right.RightControlEligible = true;
            if (!right.RightControlEligible) return;
            if (!pair.Left.Active || pair.Right.Active || right.First.Foot.Local != 0)
                right.ControlExit = "SupportOrEntry";
            else if (FootTransitionSource.Planar(pair.Right.Source.World).magnitude > FootGaitEvidence.StanceTangent)
                right.ControlExit = "MotionResumed";
            else if (pair.Right.Precontact) right.WaitingPrecontact++;
            else StartHandoff(1, right, pair.Right, right.Heading.Axis);
        }

        private void StartHandoff(int side, Window parent, in Tick tick, UnityEngine.Vector3 axis)
        {
            _handoff[side]?.Handoff.Stop("Superseded");
            _handoff[side]?.Early?.Stop("Superseded");
            _handoff[side]?.Braked?.Stop("Superseded");
            _handoff[side]?.Combined?.Stop("Superseded");
            parent.Handoff = new TurnHandoffWindow(tick.Local, axis, side == 0 ? TurnHandoffWindow.Policy.LeftReactive : TurnHandoffWindow.Policy.RightStop);
            if (side == 0)
            {
                parent.Early = new TurnHandoffWindow(tick.Local, axis, TurnHandoffWindow.Policy.LeftEarly);
                parent.Braked = new TurnHandoffWindow(tick.Local, axis, TurnHandoffWindow.Policy.LeftBraked);
                parent.Combined = new TurnHandoffWindow(tick.Local, axis, TurnHandoffWindow.Policy.LeftCombined);
            }
            _handoff[side] = parent;
        }

        internal bool TryHandoff(int side, in PhysicsStepContext step, out Tick tick, out TurnHandoffWindow run, out string prohibit, out TurnHandoffWindow early, out TurnHandoffWindow braked, out TurnHandoffWindow combined)
        {
            tick = side == 0 ? _previous.Left : _previous.Right;
            var parent = _handoff[side];
            run = early = braked = combined = null; prohibit = null;
            if (_sealed || parent == null || !tick.Step.Matches(step)) return false;
            if (parent.Handoff.Pending(tick.Local)) run = parent.Handoff;
            if (parent.Early?.Pending(tick.Local) == true) early = parent.Early;
            if (parent.Braked?.Pending(tick.Local) == true) braked = parent.Braked;
            if (parent.Combined?.Pending(tick.Local) == true) combined = parent.Combined;
            if (run == null && early == null && braked == null && combined == null) return false;
            // A left approach/stance classification can precede the native entry. The
            // unloaded-motion receipt determines whether another free solve can be issued.
            if (parent.EndReason != null) prohibit = parent.EndReason;
            else if (side == 0 && (tick.Miss || tick.Ordinary || tick.Recovery) && parent.First.Foot.Local != tick.Local)
                prohibit = "OtherEntry";
            else if (side == 1 && (!_previous.Left.Active ||
                FootTransitionSource.Planar(tick.Source.World).magnitude > FootGaitEvidence.StanceTangent))
                prohibit = "RightControlEnded";
            return true;
        }

        // Peek only when the measured, previously frozen travel axis establishes a new peak.
        // Selection of the three offsets never consumes a peak or eventual landing outcome.
        internal TurnHandoffWindow RightPeak(in PhysicsStepContext step)
        {
            var window = _brake[1];
            return !_sealed && window?.Handoff != null && _previous.Right.Step.Matches(step) &&
                window.Ankle.PeakAt == _previous.Right.Local ? window.Handoff : null;
        }

        private void AppendHandoffCounts(System.Text.StringBuilder b)
        {
            for (int side = 0; side < 2; side++)
            {
                var byDepth = new TurnHandoffWindow.Count[TurnHandoffTrajectory.ComparisonDepth];
                int started = 0, completed = 0, quiet = 0;
                foreach (var w in _windows)
                {
                    if (w.Side != side) continue;
                    if (w.RightControlEligible) quiet++;
                    if (w.Handoff == null) continue;
                    started++;
                    if (w.Handoff.End == (side == 0 ? "FirstOpportunity" : "ThreeReadbacks")) completed++;
                    w.Handoff.Accumulate(byDepth);
                }
                b.Append(side == 0 ? "\nTURN_LANDING_REACTIVE_COUNTS_V041700/side=" : "\nTURN_HANDOFF_COUNTS_V041690/side=").Append(side == 0 ? "LEFT" : "RIGHT")
                    .Append("/retainedStarted-complete=").Append(started).Append('|').Append(completed)
                    .Append("/rightQuietEligible=").Append(quiet);
                for (int depth = 0; depth < byDepth.Length; depth++)
                {
                    var c = byDepth[depth];
                    b.Append("/step").Append(depth + 1).Append("(issued-read-calibrated-basePass-trialPass-new-lost-improved)=")
                        .Append(c.Issued).Append('|').Append(c.Read).Append('|').Append(c.Calibrated).Append('|')
                        .Append(c.OriginalPass).Append('|').Append(c.TrialPass).Append('|').Append(c.NewPass)
                        .Append('|').Append(c.LostPass).Append('|').Append(c.Improved);
                }
            }
            AppendLeftLandingCounts(b);
        }

        private void StopHandoffs(string reason)
        {
            for (int side = 0; side < 2; side++)
            {
                _handoff[side]?.Handoff.Stop(reason);
                _handoff[side]?.Early?.Stop(reason);
                _handoff[side]?.Braked?.Stop(reason);
                _handoff[side]?.Combined?.Stop(reason);
                _handoff[side] = null;
            }
        }
    }
}
