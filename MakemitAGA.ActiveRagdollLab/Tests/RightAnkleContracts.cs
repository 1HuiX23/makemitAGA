using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;
using Audit = MakemitAGA.ActiveRagdollLab.Physics.FootLandingOpportunityAudit;
using Probe = MakemitAGA.ActiveRagdollLab.Physics.RightAnkleProbe;

namespace ARL.Contracts
{
    internal static class RightAnkleContracts
    {
        internal static void Run()
        {
            Check.Run("v148 qualified right onset retains only adjacent history and links cancelled controls", Windows);
            Check.Run("v148 projected ankle and eight corners follow the budgeted Knee command", Command);
            Check.Run("v148 nonzero world-frame transport, off-axis ceiling and opposing feedforward", MovingCommand);
            Check.Run("v148 submitted baseline, existing preparation and source identity are mandatory", Guards);
            Check.Run("v148 original next-step readback, missing geometry, truncation and inactive hot path", Readback);
        }
        private static Audit.Sample Sample(int i)
        {
            var s = SwingKneeContracts.Sample(i, 1f / 60f, i / 60f);
            s.EarlyFoot = EarlyLandingWindow.Foot.Right; s.SourcePhase = "STEADY"; s.RawMin = 0f;
            s.ParentValid = true; s.RawHipY = 1f; s.ReferenceHipY = .995f; s.ActualHipY = .990f;
            s.AnkleHeights = new Vector3(.1f, .096f, .097f); s.AnchorGap = .006f;
            return s;
        }
        private static void Observe(EarlyLandingWindow w, RightAnkleWindow trace, ref Audit.Sample s)
        {
            s.EarlySignal = w.Observe(s, s.EarlyFoot); s.RightWarningOrigin = w.RightWarningOrigin;
            s.ParentHeightMeasured = true;
            trace.Observe(s, default);
        }
        private static void Windows()
        {
            var w = new EarlyLandingWindow(); var trace = new RightAnkleWindow(); var history = new LandingEventHistory();
            for (int i = 1; i <= 9; i++)
            {
                var s = Sample(i); if (i >= 5) s.AnchorGap = .005f;
                if (i == 6) s.AnkleHeights.y = s.AnkleHeights.x;
                if (i >= 8) { s.Strong = true; s.Gait = Audit.Phase.Approach; }
                if (i == 9) s.OrdinaryEnabled = true;
                Observe(w, trace, ref s);
                Check.That(RightAnkleWindow.Opens(s) == (i == 5 || i == 7), "only the first eligible step and the rearm");
                history.Add(s, i / 60d, Audit.Screen(s));
                if (i == 9) history.Capture(s, false);
            }
            var b = new StringBuilder(); history.Append(b); string events = b.ToString();
            Check.That(events.Contains("/onset=5/leadMs=") && events.Contains("/heldToEdge=False/cancelLocal=6") &&
                events.Contains("/onset=7/leadMs=") && events.Contains("/heldToEdge=True/cancelLocal=0"), "ordinary cancel and rearm attached independently");
            string text = trace.Summary();
            Check.That(text.Contains("starts-saved-omitted=2|2|0") && text.Contains("next(resolved-censored-pending)=2|0|0") &&
                text.Contains("RIGHT_ANKLE_PRE_V041480/onset=5/local-source=1|1") && text.Contains("/previous=4/result=Missing"), "gap history cannot authorize");
            for (int change = 0; change < 5; change++)
            {
                trace = new RightAnkleWindow(); w = new EarlyLandingWindow(); var old = Sample(1); Observe(w, trace, ref old);
                var s = Sample(change == 0 ? 3 : 2); s.AnchorGap = .004f;
                if (change == 1) s.SupportId++; if (change == 2) s.FloorCenter.x += .01f;
                if (change == 3) { s.Step.Epoch++; s.Source.Epoch++; }
                if (change == 4) trace.Cut();
                Observe(w, trace, ref s);
                Check.That(trace.Summary().Contains("/previous=0/result=Missing"), "no stale trajectory across boundary " + change);
            }
        }
        private static Probe.Input Fixture(float dt, out KneeReferenceState k)
        {
            var s = Sample(3); s.Step.Dt = dt;
            var q = Quaternion.AngleAxis(30f, Vector3.right);
            k = new KneeReferenceState
            {
                Valid = true,
                Selected = true,
                Step = s.Step,
                Source = s.Source,
                R = q,
                PreviousR = q,
                InputR = q,
                MaxW = 4f,
                MaxA = 20f
            };
            var g = new KneeGroundTarget.Geometry
            {
                Hip = new Vector3(0, .9f, 0),
                Upper = Quaternion.identity,
                UpperSpan = Vector3.up * (-.4f),
                LowerSpan = Vector3.up * (-.4f),
                Half = new Vector3(.06f, .03f, .07f),
                FloorCenter = new Vector3(0, -.1f, 0),
                FloorHalf = new Vector3(2f, .1f, 2f),
                FootRelative = Quaternion.Inverse(q),
                FootBindRelative = Quaternion.Inverse(q)
            };
            var knee = Vector3.up * (.023f - (q * g.LowerSpan).y);
            return new Probe.Input
            {
                Geometry = g,
                Upper = Quaternion.identity,
                Lower = q,
                RawKnee = q,
                Axis = Vector3.right,
                FootRelative = g.FootRelative,
                SelectedFoot = g.FootRelative,
                Knee = knee,
                Gap = Vector3.up * .001f,
                SourceAnkleY = .033f,
                ObservedLowerY = .023f,
                ObservedBox = -.006f,
                BodyMaxW = 7f,
                Kp = 45f,
                Kd = 9f,
                SoftKp = 20f,
                SoftKd = 4f,
                SoftLimit = 20f,
                FlexLimit = 40f,
                FfActive = true,
                FfGain = 1f,
                FfLimit = 20f,
                FeedbackPriority = true,
                AllowExtension = true,
                LowerW = Vector3.zero,
                UpperW = Vector3.zero,
                HipTargetW = Vector3.zero,
                RelativeCommand = Vector3.zero,
                TotalCommand = Vector3.zero,
                SubmittedFf = 0f,
                AllocatedFf = 0f
            };
        }
        private static float CornerMin(Vector3 ankle, Quaternion q, KneeGroundTarget.Geometry g)
        {
            float min = float.PositiveInfinity;
            for (int x = -1; x <= 1; x += 2) for (int y = -1; y <= 1; y += 2) for (int z = -1; z <= 1; z += 2)
                    {
                        var corner = g.Center + new Vector3(x * g.Half.x, y * g.Half.y, z * g.Half.z);
                        min = Mathf.Min(min, (ankle + q * corner).y - g.FloorCenter.y - g.FloorHalf.y);
                    }
            return min;
        }
        private static void Command()
        {
            foreach (float hz in new[] { 30f, 60f, 120f })
            {
                var p = Fixture(1f / hz, out var k);
                var r = Probe.Compare(p, k, k.Step, k.Source);
                Check.That(r.State == Probe.Status.Improved, "bounded reference and physically projected gain at " + hz + " Hz: " + r.State);
                var goal = p.RawKnee * Quaternion.AngleAxis(r.Sign * KneeGroundTarget.MaxCorrectionDegrees, p.Axis);
                var lane = LandingPreparationMath.MakeLane(k.PreviousR, k.PreviousW, goal, k.InputW, k.R, k.W, k.Step.Dt, k.MaxW, k.MaxA, p.Axis, true);
                Check.That(LandingPreparationMath.TryLane(lane, p.Geometry, r.Index, out var rot, out var w), "winning legal sample");
                var axis = p.Axis; var a = (w - k.PreviousW) / k.Step.Dt;
                float angle = Vector3.Dot(FootRelativeReference.Log(rot * Quaternion.Inverse(k.R)), axis);
                float pd = p.Kp * angle + p.Kd * Vector3.Dot(w, axis);
                float ff = Mathf.Clamp(p.FfGain * Vector3.Dot(a, axis), -p.FfLimit, p.FfLimit);
                float flex = Mathf.Clamp(pd + (p.FeedbackPriority && pd * ff < -1e-6f ? 0f : ff), -p.FlexLimit, p.FlexLimit);
                Check.Near(r.TrialFlex, flex, .0002f, "P/D plus allowed FF, bounded by original Knee port");
                var q = Quaternion.AngleAxis(flex * k.Step.Dt * k.Step.Dt * Mathf.Rad2Deg, axis) * p.Lower;
                var ankle = p.Knee + q * p.Geometry.LowerSpan;
                Check.Near(r.Candidate.Ankle, ankle.y, .000002f, "one applied acceleration, not a teleported reference");
                Check.Near(r.Candidate.Box, CornerMin(ankle + p.Gap, q * p.FootRelative, p.Geometry), .000002f, "same measured ankle gap and all eight corners");
                Check.That(r.RefA <= r.MaxA + .0001f && r.RefW <= r.MaxW + .0001f && r.TrialBodyW <= r.BodyMaxW && Math.Abs(r.TrialFlex) <= r.FlexLimit, "three budget domains");
                var limit = p; limit.FlexLimit = .1f;
                var sat = Probe.Compare(limit, k, k.Step, k.Source);
                Check.That(sat.State == Probe.Status.BelowGain && sat.CandidateMeasured && Math.Abs(sat.TrialFlex) <= .1f && sat.CandidateClamped, "saturated reference gain is not an actuator gain");
                var blocked = p; blocked.FfActive = false;
                var fb = Probe.Compare(blocked, k, k.Step, k.Source);
                Check.That(fb.CandidateMeasured && fb.TrialFf == 0f && fb.TrialFlex < r.TrialFlex, "original FF suppression carries through");
            }
        }
        private static void MovingCommand()
        {
            var p = Fixture(1f / 60f, out var k);
            k.W = Vector3.right * .12f;
            k.A = k.W / k.Step.Dt;
            k.R = FootRelativeReference.Advance(k.PreviousR, k.W, k.Step.Dt);
            p.Upper = Quaternion.AngleAxis(37f, Vector3.up);
            p.Lower = p.Upper * k.R;
            var axis = p.Upper * Vector3.right;
            var offAxis = p.Upper * Vector3.forward;
            var hipTransport = Vector3.up * 4f + axis * 2f;
            p.UpperW = Vector3.up * .2f;
            p.LowerW = p.UpperW + axis * .34f + offAxis * .7f;
            p.SoftLimit = 1.1f;
            p.BodyMaxW = .65f;
            p.FfLimit = 4f;
            var ankle = p.Knee + p.Lower * p.Geometry.LowerSpan;
            p.ObservedLowerY = ankle.y;
            p.ObservedBox = CornerMin(ankle + p.Gap, p.Lower * p.FootRelative, p.Geometry);
            p.SubmittedPd = (.12f - .34f) * p.Kd;
            p.SubmittedFf = p.FfLimit;
            foreach (bool priority in new[] { true, false })
            {
                p.FeedbackPriority = priority;
                p.AllocatedFf = priority ? 0f : p.FfLimit;
                p.SubmittedFlex = p.SubmittedPd + p.AllocatedFf;
                p.RelativeCommand = axis * p.SubmittedFlex - offAxis * p.SoftLimit;
                p.TotalCommand = p.RelativeCommand + hipTransport;
                var r = Probe.Compare(p, k, k.Step, k.Source);
                Check.That(r.BaseMeasured, "nonzero native receipt must close: " + r.State);
                Check.Near(r.BaseFlex, p.SubmittedFlex, .0002f, "opposing feedforward priority");
                var w = Vector3.ClampMagnitude(p.LowerW + p.TotalCommand * k.Step.Dt, p.BodyMaxW);
                var lower = Quaternion.AngleAxis(w.magnitude * Mathf.Rad2Deg * k.Step.Dt, w.normalized) * p.Lower;
                var predicted = p.Knee + lower * p.Geometry.LowerSpan;
                Check.Near(r.Baseline.Box, CornerMin(predicted + p.Gap, lower * p.FootRelative, p.Geometry), .000002f, "transport outside relative cap, world-speed limited");
                Check.Near(r.Baseline.Ankle, predicted.y, .000002f, "asymmetric measured angular velocity");
                var corrupt = p;
                corrupt.SubmittedFf = 0f;
                Check.That(Probe.Compare(corrupt, k, k.Step, k.Source).State == Probe.Status.BaselineMismatch, "suppressed allocation cannot hide an incorrect FF readback");
            }
        }
        private static void Guards()
        {
            var p = Fixture(1f / 60f, out var k);
            var bad = p; bad.SubmittedFlex = .5f;
            Check.That(Probe.Compare(bad, k, k.Step, k.Source).State == Probe.Status.BaselineMismatch, "receipt must reproduce");
            bad = p; bad.RelativeCommand.x = float.PositiveInfinity;
            Check.That(Probe.Compare(bad, k, k.Step, k.Source).State == Probe.Status.InvalidReference, "nonfinite native input");
            bad = p; bad.SubmittedPd = float.PositiveInfinity;
            Check.That(Probe.Compare(bad, k, k.Step, k.Source).State == Probe.Status.BaselineMismatch, "nonfinite receipt");
            bad = p; bad.ObservedBox += .004f;
            Check.That(Probe.Compare(bad, k, k.Step, k.Source).State == Probe.Status.Geometry, "measured foot must close");
            var saved = k; k.GroundTrajectory.Applied = true;
            Check.That(Probe.Compare(p, k, k.Step, k.Source).State == Probe.Status.GroundKnee, "no stacking a previous correction");
            k = saved; k.LandingPreparation.Applied = true;
            Check.That(Probe.Compare(p, k, k.Step, k.Source).State == Probe.Status.LandingKnee, "existing landing command");
            k = saved; k.Stale = true; Check.That(Probe.Compare(p, k, k.Step, k.Source).State == Probe.Status.StaleKnee, "stale reference");
            k = saved; var other = k.Source; other.Sequence++;
            Check.That(Probe.Compare(p, k, k.Step, other).State == Probe.Status.Missing, "held source identity");
            bad = p; bad.BodyMaxW = .0002f; var cap = Probe.Compare(bad, k, k.Step, k.Source);
            Check.That(cap.CandidateMeasured && cap.State == Probe.Status.BelowGain && cap.TrialBodyW <= bad.BodyMaxW + .000001f, "whole-vector body speed cap");
        }
        private static void Readback()
        {
            var trace = new RightAnkleWindow(); var w = new EarlyLandingWindow();
            for (int i = 1; i <= 260; i++)
            {
                var s = Sample(i); if ((i & 1) != 0) s.AnchorGap = .004f;
                if ((i & 1) == 0) s.FivePoint = false;
                Observe(w, trace, ref s);
            }
            var text = trace.Summary();
            Check.That(text.Contains("starts-saved-omitted=130|128|2") && text.Contains("next(resolved-censored-pending)=0|130|0"), "omitted samples and invalid nexts stay in denominators");
            var lone = Sample(261); lone.AnchorGap = .004f; Observe(w, trace, ref lone);
            Check.That(trace.Summary().Contains("next(resolved-censored-pending)=0|130|1"), "no future sample at stop");
            trace.Cut(); Check.That(trace.Summary().Contains("next(resolved-censored-pending)=0|131|0"), "one active or interrupted step cancels pending readback");
            trace = new RightAnkleWindow();
            for (int j = 1; j <= 100; j++) { var s = Sample(j); trace.Observe(s, default); }
            long bytes = GC.GetAllocatedBytesForCurrentThread();
            for (int j = 101; j <= 2100; j++) { var s = Sample(j); trace.Observe(s, default); }
            Check.That(bytes == GC.GetAllocatedBytesForCurrentThread(), "ordinary right observation allocates no managed memory");
        }
    }
}
