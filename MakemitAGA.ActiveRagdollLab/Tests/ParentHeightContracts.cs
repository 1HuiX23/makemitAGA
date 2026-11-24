using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Probe = MakemitAGA.ActiveRagdollLab.Physics.ParentHeightProbe;
using Foot = MakemitAGA.ActiveRagdollLab.Physics.EarlyLandingWindow.Foot;

namespace ARL.Contracts
{
    internal static class ParentHeightContracts
    {
        internal static void Run()
        {
            Check.Run("v147 relative hip loss and right warning require the same continuous qualified interval", Windows);
            Check.Run("v147 same-state COM candidate retains vector bounds and distinguishes both foot responses", Geometry);
            Check.Run("v147 original and alternative bilateral allocations use value copies and the live position gate", Allocation);
            Check.Run("v147 historical floor, moving goals and deficit feedback use the same reference budget", MovingGoal);
            Check.Run("v147 strong-edge report retains early false alerts and their cancellations", Events);
            Check.Run("v147 truncated and incomplete bilateral probes retain their denominator", Capacity);
        }

        private static Audit.Sample Sample(int n, Audit.Phase phase = Audit.Phase.Swing)
        {
            var s = SwingKneeContracts.Sample(n, 1f / 60f, n / 60f);
            s.ParentValid = true; s.SourcePhase = "STEADY"; s.Gait = phase; s.Min = -.001f; s.RawMin = -.004f;
            s.RawHipY = 1f; s.ReferenceHipY = .99f; s.ActualHipY = .985f;
            s.FloorCenter = new Vector3(0, -.1f, 0);
            s.KneeHeights = new Vector3(.6f, .59f, .59f);
            s.AnkleHeights = new Vector3(.104f, .1f, .1f);
            return s;
        }
        private static void Windows()
        {
            var w = new EarlyLandingWindow(); var s = Sample(1, Audit.Phase.Lift);
            w.Observe(s, Foot.Left); Check.That(w.Origin == 1 && !w.HeightTrial, "absolute initial error is not height loss");
            s = Sample(2); s.ReferenceHipY += .2f; s.RawHipY += .2f; s.ActualHipY += .2f;
            w.Observe(s, Foot.Left); Check.That(!w.HeightTrial, "common world translation");
            s = Sample(3); s.ReferenceHipY -= .003f; s.ActualHipY -= .004f;
            w.Observe(s, Foot.Left); Check.That(w.HeightTrial, "worsening reference starts a trial");
            Check.Near(w.HeightLoss, .003f, .000001f, "loss is relative to lift"); Check.Near(w.FollowChange, -.001f, .000001f, "separate body follow");
            s = Sample(4); s.ParentValid = false; w.Observe(s, Foot.Left);
            s = Sample(5); s.ReferenceHipY -= .02f; w.Observe(s, Foot.Left);
            Check.That(!w.HeightTrial, "missing heights interrupt the original comparison");
            w.Reset();
            s = Sample(1); s.RawMin = .001f; w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 1, "right alert from current sample");
            s = Sample(2); s.RawMin = .004f; w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 1, "still low, raw may leave band");
            s = Sample(3); s.RawMin = .001f; s.AnkleHeights.y = s.AnkleHeights.x; w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 0, "low ankle recovers");
            s = Sample(4); w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 0, "broad-only cannot rearm");
            s = Sample(5); s.RawMin = .001f; w.Observe(s, Foot.Right);
            s = Sample(6); s.RawMin = .001f; s.SupportId = 999; w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 6, "surface resets and can start fresh");
            s = Sample(7); s.SupportId = 999; s.FivePoint = false; w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 0, "missing five points cut");
            s = Sample(8); s.RawMin = 0; w.Observe(s, Foot.Right);
            w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 0, "duplicate cannot carry or rearm");
            s = Sample(9); s.RawMin = 0; w.Observe(s, Foot.Right);
            s = Sample(10); s.SourcePhase = "TURN"; w.Observe(s, Foot.Right); Check.That(w.RightWarningOrigin == 0, "leaving steady cancels");
            Check.That(w.Summary().Contains("rightWarnings(starts-heldTicks-cleared-open)=5|6|5|False"), "right lifetime accounting");
        }

        private static Probe.Input Input(in PhysicsStepContext step)
        {
            return new Probe.Input
            {
                Step = step,
                Arm = PelvisDynamicReceiverIsolation.DynamicHipArmB,
                Position = Vector3.up,
                Goal = Vector3.up,
                CommittedPosition = Vector3.up,
                PositionLimit = .1f,
                ActualCom = Vector3.up,
                ReceiverReady = true
            };
        }
        private static Probe.FootPose Pose(float clearance)
        {
            return new Probe.FootPose
            {
                Valid = true,
                Hip = Vector3.up,
                LowerAnkle = Vector3.up * .1f,
                Ankle = Vector3.up * (.1f + clearance),
                Center = new Vector3(.02f, 0, .01f),
                Half = new Vector3(.03f, .1f, .10f),
                Rotation = Quaternion.identity
            };
        }
        private static void Geometry()
        {
            var s = Sample(6); var t = new Probe.Request(s, 1, .008f, -.002f); var input = Input(s.Step);
            var l = Pose(s.Min); var r = Pose(.0002f); r.Active = r.Authorized = true;
            var good = Probe.Compare(t, input, l, r, out var plan);
            Check.That(good.Outcome == Probe.Status.LeftImproved && good.Right == Probe.Peer.Authorized && good.RightInside, "clear both footprints");
            Check.Near(good.Delta.y, 4f * .008f * s.Step.Dt, 2e-7f, "computed displacement, not full requested height");
            Check.Near(good.RightGapGrowth, good.Delta.y, 2e-7f, "free-comoving peer");
            Check.Near(good.PinnedReachChange, good.Delta.y, 2e-7f, "fixed opposite ankle");
            Check.That(good.CandidateA <= PelvisComReferencePlanner.MaxA + .001f && good.CandidateV <= PelvisComReferencePlanner.MaxV + .001f, "original COM limits");
            var raised = t; raised.AnkleError = .004f;
            var mixed = Probe.Compare(raised, input, l, r, out _);
            Check.That(mixed.Outcome == Probe.Status.Mixed && mixed.AnkleGain < 0f, "opposing ankle evidence");
            var rightUnknown = Probe.Compare(t, input, l, default, out _);
            Check.That(rightUnknown.Outcome == good.Outcome && !rightUnknown.RightMeasured && rightUnknown.Right == Probe.Peer.Unknown, "peer failure independent");
            r.Authorized = false; Check.That(Probe.Compare(t, input, l, r, out _).Right == Probe.Peer.ActiveUnqualified, "active alone cannot certify support");
            r.Ankle.x = t.Half.x; Check.That(!Probe.Compare(t, input, l, r, out _).RightMeasured, "peer outside its surface is unknown");
            l.LowerAnkle.y += .001f; Check.That(Probe.Compare(t, input, l, r, out _).Outcome == Probe.Status.LeftGeometry, "native closure rejected");
            l = Pose(t.Box); l.Ankle.x = t.Half.x; Check.That(Probe.Compare(t, input, l, r, out _).Outcome == Probe.Status.Footprint, "edge");
            input.Step.Epoch++; Check.That(!Probe.Compare(t, input, l, r, out _).Planned, "foreign receiver clock");
            input = Input(s.Step); input.Arm = PelvisDynamicReceiverIsolation.DynamicHipArmA;
            var peer = Pose(.0002f); peer.Active = peer.Authorized = true;
            var same = Probe.Compare(t, input, Pose(s.Min), peer, out _);
            Check.That(same.Planned && same.Outcome == good.Outcome && same.Delta.Equals(good.Delta) &&
                same.RightGapGrowth == good.RightGapGrowth, "both arms retain the 181 B parent-height probe");
            input.Arm = "LEGACY";
            Check.That(!Probe.Compare(t, input, Pose(s.Min), peer, out _).Planned, "unselected scope rejected");
        }

        private static void Allocation()
        {
            var s = Sample(6); var t = new Probe.Request(s, 1, .008f, 0f); var input = Input(s.Step);
            var geo = Probe.Compare(t, input, Pose(s.Min), Pose(0f), out var plan);
            var baseline = PelvisDynamicReceiverIsolation.Baseline(s.Step);
            baseline.Acceleration = Vector3.zero; baseline.UpwardRetentionSelected = baseline.BoundedTradeSelected = true;
            baseline.UpwardRetentionEligible = false;
            var l = HipAllocationContracts.Input(baseline, true, Vector3.zero, Vector3.zero, Vector3.zero, -Vector3.right * .1f);
            var r = HipAllocationContracts.Input(baseline, false, Vector3.zero, Vector3.zero, Vector3.zero, Vector3.right * .1f);
            bool ok = HipBilateralForceSolver.TryPlan(l, r, out var actualL, out var actualR);
            Check.That(ok && geo.Planned, "baseline solved");
            var alt = ParentHeightAllocation.Compare(input, plan, l, r, actualL, actualR);
            Check.That(alt.State == ParentHeightAllocation.Status.Measured && alt.TrialRequestY > alt.BaseRequestY &&
                alt.TrialAllocatedY > alt.BaseAllocatedY && alt.RightForceDeltaNorm > 0, "real force allocator sees the changed COM");
            Check.Near(l.Seed.ParentInput.Acceleration.y, 0, 0, "left actual input untouched");
            Check.Near(r.Seed.ParentInput.Acceleration.y, 0, 0, "right actual input untouched");
            var otherForce = actualR; otherForce.Force.x += .1f;
            Check.That(ParentHeightAllocation.Compare(input, plan, l, r, actualL, otherForce).State == ParentHeightAllocation.Status.BaselineMismatch, "original right force must close");
            input.PositionLimit = .0001f;
            Check.That(ParentHeightAllocation.Compare(input, plan, l, r, actualL, actualR).State == ParentHeightAllocation.Status.PositionGate, "existing physical error gate");
            input = Input(s.Step); input.VelocityError.y = .01f;
            Check.That(ParentHeightAllocation.Compare(input, plan, l, r, actualL, actualR).State == ParentHeightAllocation.Status.BaselineMismatch, "original receiver command must close");
            input = Input(s.Step);
            Check.That(ParentHeightAllocation.Compare(input, plan, l, r, actualR, actualL).State == ParentHeightAllocation.Status.Missing, "opposite receipt");
        }

        private static void MovingGoal()
        {
            var rng = new Random(147);
            for (int i = 0; i < 300; i++)
            {
                var s = Sample(i + 5); var t = new Probe.Request(s, 2, .004f + (float)rng.NextDouble() * .008f, 0f);
                var o = Input(s.Step); var previous = s.Step; previous.Sequence--; previous.SourceSequence--; previous.FixedTime -= previous.Dt;
                o.Support = new SupportReferenceContext { ForStep = s.Step, ObservedStep = previous, SupportId = s.SupportId, StanceMask = 2 };
                o.PositionError = new Vector3(.001f, -.002f, 0);
                o.FloorY = .999f + (float)rng.NextDouble() * .003f;
                o.GoalV = new Vector3(.2f, (float)rng.NextDouble() * .35f - .17f, 0);
                o.Velocity = new Vector3(.35f, (float)rng.NextDouble() * .15f - .08f, 0);
                o.Feedback = true; o.PreviousRequest = new Vector3(.1f, .4f, .2f);
                o.PreviousAllocated = o.PreviousRequest * .75f; o.SettledV = new Vector3(.33f, -.04f, .01f);
                PelvisComReferencePlanner.ComPlan Solve(Vector3 goal) => PelvisDynamicReceiverIsolation.BuildComReferencePlan(o.Arm,
                    o.Position, o.Velocity, goal, o.GoalV, s.Step.Dt, s.Step, o.Feedback, o.PreviousRequest, o.PreviousAllocated, o.SettledV, o.Support, o.FloorY);
                var original = Solve(o.Goal); o.CommittedPosition = original.Position; o.CommittedV = original.Velocity;
                var r = Probe.Compare(t, o, Pose(t.Box), Pose(0), out var trial);
                var independent = Solve(o.Goal + Vector3.up * t.Loss);
                Check.That(r.Planned && r.FloorSelected && trial.FeedbackAvailable && trial.SupportBrakingSelected, "preserve support floor, braking and feedback");
                Check.Near((trial.Position - independent.Position).magnitude, 0f, 1e-8f, "identical full replay");
                Check.That(r.CandidateV <= PelvisComReferencePlanner.MaxV + .001f && r.CandidateA <= PelvisComReferencePlanner.MaxA + .001f, "bounded trial");
                var foreign = o; foreign.Support.SupportId++;
                Check.That(!Probe.Compare(t, foreign, Pose(t.Box), default, out _).Planned, "do not combine different controlled floors");
            }
        }
        private static void Events()
        {
            var w = new EarlyLandingWindow(); var history = new LandingEventHistory();
            for (int i = 1; i <= 7; i++)
            {
                var s = Sample(i); s.EarlyFoot = Foot.Right; s.ParentHeightMeasured = true;
                s.RawMin = i == 1 || i == 4 ? .001f : .005f;
                if (i == 3) s.AnkleHeights.y = s.AnkleHeights.x;
                if (i >= 6) { s.Gait = Audit.Phase.Approach; s.Strong = true; }
                s.EarlySignal = w.Observe(s, Foot.Right); s.RightWarningOrigin = w.RightWarningOrigin;
                s.ParentHeightRequested = w.HeightTrial;
                s.OrdinaryEnabled = i == 7;
                history.Add(s, i / 60d, Audit.Screen(s)); if (i == 7) history.Capture(s, false);
            }
            var b = new StringBuilder(); history.Append(b); string log = b.ToString();
            Check.That(log.Contains("firstNearLeadMs=83.333333/lastCancelLocal=3/heldBefore=True/heldOrigin=4/heldLeadMs=33.333333"),
                "normal event retains the canceled first alarm and the surviving later episode");
        }

        private static void Capacity()
        {
            var tr = new Probe.Trace();
            Check.That(tr.Summary() == Probe.Trace.EmptySummary, "empty trace has the same schema");
            for (int i = 1; i <= Probe.Trace.Capacity + 2; i++)
            {
                var s = Sample(i); tr.Begin(new Probe.Request(s, i, .003f, 0f));
                if (i == 1) continue;
                var wrong = s.Step; wrong.Epoch--;
                Check.That(!tr.TryTake(wrong, out _) && tr.TryTake(s.Step, out _) && !tr.TryTake(s.Step, out _), "one bilateral completion " + i);
                tr.Complete(new Probe.Result { Outcome = Probe.Status.InputMissing });
            }
            string text = tr.Summary();
            Check.That(text.Contains("trials=130") && text.Contains("=1|129|0|0|0|0|0|0/omitted=2") &&
                text.Split("PARENT_HEIGHT_STEP_V041470").Length == 129, "retain failed completions and omitted rows");
        }
    }
}
