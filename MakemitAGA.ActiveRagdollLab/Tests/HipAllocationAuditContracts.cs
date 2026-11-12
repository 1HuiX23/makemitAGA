using System;
using System.Reflection;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class HipAllocationAuditContracts
    {
        internal static void Run()
        {
            Check.Run("optional allocation context cannot alter the physical receipt", ContextIsolation);
            Check.Run("same-input candidates preserve budgets and expose angular cost", Candidates);
            Check.Run("allocation attribution distinguishes the nine constraint cuts and ties", CutFixtures);
            Check.Run("allocation audit rejects stale or mismatched evidence without touching commands", Faults);
            Check.Run("allocation summary is bounded, trace-gated and resettable", ReportLifecycle);
            Check.Run("allocation audit retains no per-step heap allocations", NoHeap);
        }
        private static PelvisHipCommandFrame Parent(int offset = 0)
        {
            var p = PelvisDynamicReceiverIsolation.Baseline(PelvisDynamicReceiverIsolation.LoggedStep(offset));
            p.Acceleration = new Vector3(1f, 4f, 1f);
            p.AllocationAudit = Context(p);
            return p;
        }
        private static HipAllocationAuditContext Context(PelvisHipCommandFrame p)
        {
            var previous = p.Step; previous.Sequence--;
            var reference = new PelvisComReferencePlanner.ComPlan { ArrivalEnabled = true,
                SupportInput = new SupportReferenceContext { ForStep = p.Step, ObservedStep = previous, SupportId = 301, StanceMask = 3 } };
            return HipAllocationAuditContext.Capture(true, p.Step, p.Step, reference, .01f, p.Acceleration);
        }
        private static void Fixture(int offset, out HipAnchorPairInput l, out HipAnchorPairInput r,
            out HipAnchorPairReport.Command lc, out HipAnchorPairReport.Command rc)
        {
            var p = Parent(offset);
            l = HipAllocationContracts.Input(p, true, 6f * Vector3.up, Vector3.zero, .2f * Vector3.up, -.1f * Vector3.right);
            r = HipAllocationContracts.Input(p, false, 6f * Vector3.up, Vector3.zero, .2f * Vector3.up, .1f * Vector3.right);
            Check.That(HipBilateralForceSolver.TryPlan(l, r, out lc, out rc), "uniform allocation fixture");
        }
        private static PelvisHipAttitudeCommand Completed(HipAnchorPairReport.Command l, HipAnchorPairReport.Command r)
        {
            Check.That(PelvisHipAttitudeCommand.TryPlanWithReaction(l.ParentInput, -l.Force - r.Force,
                l.ParentTorque + r.ParentTorque, l.BilateralAngularReaction, out var c), "completed receiver fixture");
            c.MotionScale = l.MotionScale; c.CommonAcceleration = l.CommonAcceleration; c.Committed = true;
            return c;
        }
        private static void SamePhysical(HipAnchorPairReport.Command a, HipAnchorPairReport.Command b)
        {
            Check.That(a.ParentInput.SameAs(b.ParentInput), "physical identity ignores optional evidence");
            foreach (var f in typeof(HipAnchorPairReport.Command).GetFields(BindingFlags.Instance | BindingFlags.NonPublic))
                if (f.Name != nameof(HipAnchorPairReport.Command.ParentInput))
                    Check.That(object.Equals(f.GetValue(a), f.GetValue(b)), "exact command field " + f.Name);
        }
        private static void ContextIsolation()
        {
            Fixture(0, out var l, out var r, out var a, out var b);
            var valid = l.Seed.ParentInput.AllocationAudit;
            Check.That(valid.Selected && valid.Eligible, "current completed support selects comparison");
            for (int i = 0; i < 5; i++)
            {
                var context = valid;
                if (i == 0) context = default;
                if (i == 1) context.ErrorY = float.NaN;
                if (i == 2) context.Step.Epoch++;
                if (i == 3) context.Step.Sequence--;
                if (i == 4) context.ErrorY = -1f;
                l.Seed.ParentInput.AllocationAudit = r.Seed.ParentInput.AllocationAudit = context;
                Check.That(l.Ready && r.Ready, "optional fault leaves input ready");
                Check.That(HipBilateralForceSolver.TryPlan(l, r, out var c, out var d), "optional fault leaves planner available");
                SamePhysical(a, c); SamePhysical(b, d);
                Check.That(!HipBilateralForceSolver.TryAudit(l, r, c, d, out _), "invalid evidence affects audit only");
            }
            var p = Parent(); var stale = p.Step; stale.Sequence--;
            Check.That(!HipAllocationAuditContext.Capture(true, p.Step, stale, default, .01f, p.Acceleration).Eligible,
                "selected alone does not grant support eligibility");
            Check.That(!HipAllocationAuditContext.Capture(false, p.Step, p.Step, default, .01f, p.Acceleration).Selected,
                "A has no audit input");
        }
        private static void Candidates()
        {
            var random = new Random(128); int accepted = 0, compared = 0, full = 0, fallback = 0, useful = 0, better = 0;
            int zeroSameQ = 0, validTrade = 0, mask = 0;
            for (int i = 0; i < 1200; i++)
            {
                var p = Parent(i);
                p.Acceleration = new Vector3(R(random, 5f), .1f + (float)random.NextDouble() * 8f, R(random, 5f));
                p.ReceiverTorque = V(random, .5f);
                bool simple = (i & 1) == 0;
                if (!simple) Check.That(PelvisChestPointResponse.TryCreate(2f + (float)random.NextDouble() * 15f,
                    3f + (float)random.NextDouble() * 20f, new Vector3(.2f, .4f, .3f),
                    Quaternion.AngleAxis(R(random, 180f), new Vector3(1f, 2f, 3f)), new Vector3(.03f, .3f, -.05f), out p.Response), "varied receiver");
                p.AllocationAudit = Context(p);
                float span = i % 10 == 0 ? 0f : 8f, feed = i % 10 == 0 ? 0f : 25f;
                var l = HipAllocationContracts.Input(p, true, V(random, span), V(random, feed),
                    new Vector3(R(random, .15f), .2f, R(random, .1f)), new Vector3(-.1f, -.05f, .03f),
                    simple ? 1f : 1f + (float)random.NextDouble() * 24f,
                    simple ? new Vector3(1f, 1f, 1f) : new Vector3(.15f, .08f, .2f), Quaternion.AngleAxis(R(random, 180f), Vector3.forward));
                var r = HipAllocationContracts.Input(p, false, V(random, span), V(random, feed),
                    new Vector3(R(random, .15f), .2f, R(random, .1f)), new Vector3(.1f, -.05f, .03f),
                    simple ? 1f : 1f + (float)random.NextDouble() * 24f,
                    simple ? new Vector3(1f, 1f, 1f) : new Vector3(.2f, .12f, .15f), Quaternion.AngleAxis(R(random, 180f), Vector3.right));
                if (!HipBilateralForceSolver.TryPlan(l, r, out var a, out var b)) continue;
                accepted++;
                Check.That(HipBilateralForceSolver.TryAudit(l, r, a, b, out var s) && s.Valid, "accepted allocation audited at case " + i);
                var beforeL = a; var beforeR = b;
                Check.That(HipBilateralForceSolver.TryPlan(l, r, out a, out b), "audit leaves same solve available");
                SamePhysical(beforeL, a); SamePhysical(beforeR, b);
                if (a.MotionScale == 1f) { full++; Check.That(!s.Compared && s.FullRequestOver == 0 && s.UniformCuts == 0, "full request needs no candidates"); }
                if (!a.FullFeedbackFeasible) { fallback++; Check.That(!s.Compared && s.UniformCuts == 0, "fallback is not an upper-cut event"); }
                if (!s.Compared) continue;
                compared++; mask |= s.UniformCuts;
                Check.That(s.SameQValid && s.UniformCuts != 0, "limited full-feedback interval identifies a cut");
                var probeL = l; var probeR = r;
                p.UpwardRetentionSelected = p.UpwardRetentionEligible = true;
                probeL.Seed.ParentInput = probeR.Seed.ParentInput = p;
                Check.That(HipBilateralForceSolver.TryPlan(probeL, probeR, out var oldB, out _) && oldB.MotionScale == a.MotionScale &&
                    oldB.UpwardExtraAcceleration == s.SameQGain, "same-q candidate exactly reproduces the retained v127 B algorithm");
                if (s.SameQGain < HipAllocationAuditReport.GainThreshold) zeroSameQ++;
                if (!s.TradeValid)
                {
                    Check.That(s.TradeIssue != HipAllocationCandidateIssue.None, "rejected optional candidate retains its reason");
                    continue;
                }
                validTrade++;
                CheckTrade(l, r, s);
                if (s.TradeGain >= HipAllocationAuditReport.GainThreshold) useful++;
                if (s.TradeGain - s.SameQGain >= HipAllocationAuditReport.GainThreshold) better++;
            }
            Console.WriteLine("  audit synthetic cases: accepted=" + accepted + ", q1=" + full + ", fallback=" + fallback +
                ", compared=" + compared + ", tradeValid=" + validTrade + ", gainGE0.01=" + useful + ", beatsSameQ=" + better + ", cutMask=" + mask);
            Check.That(accepted > 1000 && full > 10 && fallback > 100 && compared > 100 && validTrade > 100 && useful > 10 && better > 0 && zeroSameQ > 20,
                "coverage includes full, fallback, limited, useful, negligible and cost-tradeoff inputs");
        }
        private static void CheckTrade(HipAnchorPairInput l, HipAnchorPairInput r, HipAllocationAuditSample s)
        {
            var p = l.Seed.ParentInput; var f = s.TradeLeftForce; var g = s.TradeRightForce;
            Check.That(s.TradeT >= 0f && s.TradeT <= 1f && s.TradeUpperScale >= 0f && s.TradeUpperScale <= s.Q,
                "trade never increases Upper authority");
            Check.That(s.TradeTarget.x == s.Q * p.Acceleration.x && s.TradeTarget.z == s.Q * p.Acceleration.z &&
                s.TradeTarget.y >= s.BaselineY && s.TradeTarget.y <= p.Acceleration.y + .00001f,
                "fixed horizontal target and bounded upward retention");
            Check.That((s.TradeCommon - (f + g) / p.Response.TotalMass - s.TradeTarget).magnitude < .001f,
                "independent system COM closure");
            Vector3 lc = -Vector3.Cross(l.Seed.ChildPoint - l.Seed.ChildCom, f), rc = -Vector3.Cross(r.Seed.ChildPoint - r.Seed.ChildCom, g);
            Vector3 pointL = f / l.Seed.ChildMass + (f + g) / p.Response.TotalMass,
                pointR = g / r.Seed.ChildMass + (f + g) / p.Response.TotalMass;
            Check.That(s.TradeCommon.magnitude <= 12.0001f && pointL.magnitude <= 60.0001f && pointR.magnitude <= 60.0001f &&
                lc.magnitude <= 8.0001f && rc.magnitude <= 8.0001f &&
                (s.TradeUpperScale * l.PureTorque + lc).magnitude <= MathF.Max(8f, l.PureTorque.magnitude) + .0001f &&
                (s.TradeUpperScale * r.PureTorque + rc).magnitude <= MathF.Max(8f, r.PureTorque.magnitude) + .0001f,
                "all seven force/common/Upper budgets independently checked");
            var reaction = -s.TradeUpperScale * l.Angular.RelativeTorque - s.TradeUpperScale * r.Angular.RelativeTorque;
            Vector3 moment = Vector3.Cross(l.Seed.ParentPoint - l.Seed.ParentCom, -f) + Vector3.Cross(r.Seed.ParentPoint - r.Seed.ParentCom, -g);
            Vector3 effective = moment - p.Response.ChestFraction * Vector3.Cross(p.Response.ChestLever, -f - g);
            Check.That((p.ReceiverTorque - effective).magnitude <= 8.0001f && (p.ReceiverTorque - effective + reaction).magnitude <= 8.0001f,
                "both external and total receiver torque bounded independently");
            var parentAlpha = p.Response.PredictAngular(Vector3.zero, p.ReceiverTorque + reaction, p.ParentW);
            var parentA = p.Response.PredictCom(Vector3.zero, parentAlpha, p.ParentW);
            Vector3 childAlphaL = HipPointPairMobility.InverseTensor(l.ChildInertia, l.ChildPrincipal, s.TradeUpperScale * l.PureTorque),
                childAlphaR = HipPointPairMobility.InverseTensor(r.ChildInertia, r.ChildPrincipal, s.TradeUpperScale * r.PureTorque);
            Vector3 errorL = pointL + Vector3.Cross(childAlphaL, l.Seed.ChildPoint - l.Seed.ChildCom) - parentA -
                Vector3.Cross(parentAlpha, l.Seed.ParentPoint - l.Seed.ParentCom) - s.TradeCommon - l.Seed.FeedbackA;
            Vector3 errorR = pointR + Vector3.Cross(childAlphaR, r.Seed.ChildPoint - r.Seed.ChildCom) - parentA -
                Vector3.Cross(parentAlpha, r.Seed.ParentPoint - r.Seed.ParentCom) - s.TradeCommon - r.Seed.FeedbackA;
            Check.That(errorL.magnitude < .001f && errorR.magnitude < .001f, "same full point feedback after tradeoff");
            Check.Near(s.UpperTorqueCost.x, (s.Q - s.TradeUpperScale) * l.PureTorque.magnitude, .00001f, "left cost is disclosed");
            Check.Near(s.UpperTorqueCost.y, (s.Q - s.TradeUpperScale) * r.PureTorque.magnitude, .00001f, "right cost is disclosed");
        }
        private static void CutFixtures()
        {
            // Independently constructed limiting cases, including correction/actuator
            // ties. Full-request violations alone do not establish the final cut.
            foreach (int scenario in new[] { 0, 1, 2, 3, 4, 5, 6 })
            {
                var p = Parent(); p.Acceleration = (scenario == 0 ? 12f : 8f) * Vector3.up; p.ReceiverTorque = Vector3.zero;
                Vector3 fl = Vector3.zero, fr = Vector3.zero, cl = Vector3.zero, cr = Vector3.zero;
                Vector3 pl = Vector3.zero, pr = Vector3.zero, tl = Vector3.zero, tr = Vector3.zero;
                int expected = 1;
                if (scenario == 1 || scenario == 2)
                { fl = (scenario == 1 ? 55f : -55f) * Vector3.up; fr = -fl; expected = 1 << scenario; }
                if (scenario == 3 || scenario == 4)
                {
                    fl = (scenario == 3 ? 35f : -35f) * Vector3.up; fr = -fl;
                    if (scenario == 3) cl = .2f * Vector3.right; else cr = .2f * Vector3.right;
                    expected = (1 << scenario) | (1 << (scenario + 2));
                }
                if (scenario == 5) { tl = tr = 6f * Vector3.up; expected = 1 << 7; }
                if (scenario == 6)
                { p.Acceleration = 10f * Vector3.up; tl = tr = Vector3.forward; pl = pr = Vector3.right; expected = 1 << 8; }
                p.AllocationAudit = Context(p);
                var l = HipAllocationContracts.Input(p, true, tl, fl, cl, pl);
                var r = HipAllocationContracts.Input(p, false, tr, fr, cr, pr);
                Check.That(HipBilateralForceSolver.TryPlan(l, r, out var a, out var b), "cut fixture allocates " + scenario);
                Check.That(HipBilateralForceSolver.TryAudit(l, r, a, b, out var s) && s.UniformCuts == expected,
                    "cut fixture " + scenario + " actual=" + s.UniformCuts + " expected=" + expected);
            }
        }
        private static void Faults()
        {
            for (int i = 0; i < 10; i++)
            {
                Fixture(0, out var l, out var r, out var a, out var b);
                switch (i)
                {
                    case 0: r.Seed.ParentInput.AllocationAudit.ErrorY += .001f; break;
                    case 1: r.Seed.ParentInput.AllocationAudit.Eligible = false; break;
                    case 2: b.Step.Sequence--; break;
                    case 3: b.ParentId++; break;
                    case 4: b.ChildId = a.ChildId; break;
                    case 5: b.CommonAcceleration += Vector3.up; break;
                    case 6: b.PeerForce += Vector3.right; break;
                    case 7: b.JointSolved = false; break;
                    case 8: b.MotionScale *= .9f; break;
                    case 9: b.Fixed++; break;
                }
                Check.That(!HipBilateralForceSolver.TryAudit(l, r, a, b, out _), "audit rejects mismatch " + i);
                Check.That(HipBilateralForceSolver.TryPlan(l, r, out _, out _), "optional evidence rejection leaves live planner available " + i);
            }
        }
        private static void ReportLifecycle()
        {
            var report = new HipAllocationAuditReport();
            Fixture(0, out var l, out var r, out var a, out var b); var complete = Completed(a, b);
            report.Observe(false, l, r, a, b, complete);
            Check.That(report.Samples == 0 && report.Invalid == 0, "trace off is silent");
            report.Observe(true, l, r, a, b, complete);
            Check.That(report.Samples == 1 && report.Compared == 1 && report.Invalid == 0, "once per bilateral input");
            report.Observe(true, l, r, a, b, complete);
            Check.That(report.Samples == 1 && report.Duplicate == 1, "duplicate does not double-count");
            Fixture(2, out l, out r, out a, out b); report.Observe(true, l, r, a, b, Completed(a, b));
            Check.That(report.Samples == 2 && report.Gaps == 1, "missing shared step exposed");
            Fixture(1, out l, out r, out a, out b); report.Observe(true, l, r, a, b, Completed(a, b));
            Check.That(report.Invalid == 1 && report.Samples == 2, "stale evidence rejected");
            Fixture(3, out l, out r, out a, out b); complete = Completed(a, b); complete.Committed = false;
            report.Observe(true, l, r, a, b, complete);
            Check.That(report.Invalid == 2 && report.Samples == 2, "uncompleted input rejected");
            complete = Completed(a, b); complete.HipForce += Vector3.up;
            report.Observe(true, l, r, a, b, complete);
            Check.That(report.Invalid == 3 && report.Samples == 2, "different completed force rejected");
            l.Seed.Step.Epoch++; r.Seed.Step.Epoch++;
            l.Seed.ParentInput.Step.Epoch++; r.Seed.ParentInput.Step.Epoch++;
            report.Observe(true, l, r, a, b, Completed(a, b));
            Check.That(report.Invalid == 4, "new epoch requires explicit reset");
            report.NoteMissing();
            string text = report.Details(true);
            Check.That(text.Contains("candidateExecuted=False") && text.Contains("globalOptimality=False") &&
                text.Split('\n').Length <= 5 && text.Length < 4500, "bounded summary declares evidence domain");
            report.Reset();
            Check.That(report.Samples == 0 && report.Invalid == 0 && report.Duplicate == 0 && report.Missing == 0 && report.Gaps == 0 &&
                report.Compared == 0 && report.Details(true).IndexOf("hipAllocationAuditFrame=", StringComparison.Ordinal) < 0,
                "reset clears counters and retained snapshots");
            Fixture(3, out l, out r, out a, out b);
            complete = Completed(a, b); report.Observe(true, l, r, a, b, complete);
            string stopped = report.Details(true); report.Observe(false, l, r, a, b, complete);
            Check.That(report.Details(true) == stopped, "trace stop freezes evidence");
        }
        private static void NoHeap()
        {
            Fixture(0, out var l, out var r, out var a, out var b);
            var complete = Completed(a, b); var report = new HipAllocationAuditReport();
            report.Observe(true, l, r, a, b, complete); report.Reset(); // JIT/warm-up outside the measurement.
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 256; i++)
            {
                var step = PelvisDynamicReceiverIsolation.LoggedStep(i);
                var p = l.Seed.ParentInput; p.Step = p.AllocationAudit.Step = step;
                l.Seed.Step = r.Seed.Step = a.Step = b.Step = step;
                l.Seed.Fixed = r.Seed.Fixed = a.Fixed = b.Fixed = 11 + i;
                l.Seed.ParentInput = r.Seed.ParentInput = a.ParentInput = b.ParentInput = complete.Baseline = p;
                report.Observe(true, l, r, a, b, complete);
            }
            long bytes = GC.GetAllocatedBytesForCurrentThread() - before;
            Check.That(report.Samples == 256 && report.Compared == 256 && report.Invalid == 0 && bytes == 0,
                "bounded hot path: samples=" + report.Samples + ", invalid=" + report.Invalid + ", allocatedBytes=" + bytes);
        }
        private static float R(Random r, float span) => ((float)r.NextDouble() * 2f - 1f) * span;
        private static Vector3 V(Random r, float span) => new Vector3(R(r, span), R(r, span), R(r, span));
    }
}
