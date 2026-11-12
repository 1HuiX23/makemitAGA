using System;
using ARL.Contracts;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class PelvisDynamicReceiverIsolation
    {
        internal static void RunHipCompletionContracts()
        {
            Check.Run("retained upward allocation crosses real precommit and receiver completion", UpwardCompletion);
            Check.Run("Hip receipt joins after variable preparation with trace on/off", CompletionOffsets);
            Check.Run("Hip receipt rejects stale clocks and mismatched pairs before writes", CompletionInvalidInputs);
            Check.Run("Hip receipt is committed once and only after returned native writes", CompletionTransactions);
            Check.Run("receiver evidence resolves only the adjacent shared step", CompletionResolution);
            Check.Run("receiver receipt ring wraps without reusing closed inputs", CompletionRing);
            Check.Run("non-Hip receiver trace resolves without inventing Hip inputs", ReceiverOnlyTrace);
        }

        internal static PhysicsStepContext LoggedStep(int offset = 0) => new PhysicsStepContext
        {
            Epoch = 7, Sequence = 6296 + offset, SourceSequence = 6084 + offset / 2,
            FixedTime = (6296 + offset) / 60f, Dt = 1f / 60f, UnscaledTime = (6296 + offset) / 60f
        };
        internal static PelvisHipCommandFrame Baseline(PhysicsStepContext step)
        {
            Check.That(PelvisChestPointResponse.TryCreate(4f, 6f, new Vector3(1f, 1f, 1f),
                Quaternion.identity, Vector3.zero, out var response), "valid native parent fixture");
            return new PelvisHipCommandFrame { Committed = true, JointTorqueReaction = true, Step = step,
                BodyId = 101, ChestBodyId = 102, Acceleration = Vector3.right, ReceiverTorque = .2f * Vector3.up,
                ParentW = Vector3.zero, ReferenceRotation = Quaternion.identity, ReferenceW = Vector3.zero, Response = response };
        }
        internal static HipAnchorPairReport Pair(PelvisHipCommandFrame parent, int fixedStep, bool left)
        {
            Check.That(HipAngularRequest.TryCreate(new Vector3(1f, 1f, 1f), Quaternion.identity, Vector3.zero,
                .1f * Vector3.up, .1f * Vector3.up, .1f * Vector3.up, Vector3.zero, .1f * Vector3.up,
                20f, out var angular), "valid angular request fixture");
            var force = (left ? .4f : .6f) * Vector3.right;
            var c = new HipAnchorPairReport.Command
            {
                Step = parent.Step, Fixed = fixedStep, ChildId = left ? 201 : 202, ParentId = parent.BodyId, ParentInput = parent,
                JointSolved = true, FullFeedbackFeasible = true, MotionScale = 1f, Scale = 1f, Angular = angular,
                ParentAngularReaction = -.1f * Vector3.up, BilateralAngularReaction = -.2f * Vector3.up,
                Force = force, PeerForce = (left ? .6f : .4f) * Vector3.right, CommonAcceleration = 1.1f * Vector3.right,
                ParentTorque = Vector3.zero, ChildTorque = Vector3.zero, GeometryMoment = Vector3.zero,
                ChildMass = 1f, ParentMass = 4f, ChildCom = Vector3.zero, ParentCom = Vector3.zero,
                ChildPoint = Vector3.zero, ParentPoint = Vector3.zero, ChildV = Vector3.zero, ParentV = Vector3.zero,
                Error = Vector3.zero, VelocityError = Vector3.zero, FeedbackA = force, DesiredA = force, AppliedA = force,
                NetHipTorque = angular.RelativeTorque, AllocatedHipAlpha = angular.RelativeAlpha,
                UnboundedForce = force, PureHipTorque = angular.RelativeTorque, KnownBiasA = Vector3.zero,
                PredictedNetA = force, FeedbackResidualA = Vector3.zero, UpperCorrectionTorque = Vector3.zero,
                UpperActuatorTorque = angular.RelativeTorque, UpperCorrectionAlpha = Vector3.zero, UpperActuatorLimitNm = 8f,
                PointBudgetRatio = 1f, UpperCorrectionBudgetRatio = 1f, UpperActuatorBudgetRatio = 1f,
                ReceiverBudgetRatio = 1f, CommonBudgetRatio = 1f, BilateralDampingPower = 0f,
                DampingPower = 0f, PairPower = 0f, ParentResponseAlpha = Vector3.zero
            };
            return new HipAnchorPairReport { Selected = true, Finite = true, CurrentApplied = true, Current = c,
                ParentId = c.ParentId, ChildId = c.ChildId, Issue = null, RollbackFailures = 0 };
        }
        private void Produce(PhysicsStepContext step, bool trace = true, bool upward = false)
        {
            _hipCommand = Baseline(step); _hipAttitudeCommand = default; HipCompletionIssue = null;
            if (upward)
            {
                _hipCommand.Acceleration = 4f * Vector3.up;
                _hipCommand.UpwardRetentionSelected = _hipCommand.UpwardRetentionEligible = true;
            }
            Report.TraceRunning = trace;
            if (!trace) return;
            CountStep("IDLE");
            CaptureReceiverTraceInput(step, step.Dt, _pelvis.velocity, _chest.velocity,
                _pelvis.mass * _hipCommand.Acceleration, _chest.mass * _hipCommand.Acceleration, _hipCommand.ReceiverTorque);
        }
        private static PelvisDynamicReceiverIsolation Fixture(out HipAnchorPairReport left, out HipAnchorPairReport right,
            int legFixed = 11, bool trace = true)
        {
            var receiver = new PelvisDynamicReceiverIsolation(); receiver.Produce(LoggedStep(), trace);
            left = Pair(receiver._hipCommand, legFixed, true); right = Pair(receiver._hipCommand, legFixed, false);
            return receiver;
        }
        private static PelvisDynamicReceiverIsolation UpwardFixture(bool trace, out HipAnchorPairReport l, out HipAnchorPairReport rr)
        {
            var receiver = new PelvisDynamicReceiverIsolation(); receiver.Produce(LoggedStep(), trace, true);
            var parent = receiver._hipCommand;
            var li = HipAllocationContracts.Input(parent, true, 6f * Vector3.up, Vector3.zero, .2f * Vector3.up, -.1f * Vector3.right);
            var ri = HipAllocationContracts.Input(parent, false, 6f * Vector3.up, Vector3.zero, .2f * Vector3.up, .1f * Vector3.right);
            Check.That(HipBilateralForceSolver.TryPlan(li, ri, out var lc, out var rc) && lc.UpwardExtraAcceleration > .1f,
                "production solver generates retained input");
            Check.That(HipAnchorTranslationPair.CheckActualPrecommit(li, lc) && HipAnchorTranslationPair.CheckActualPrecommit(ri, rc),
                "real precommit accepts new force/target closure");
            var broken = lc; broken.UpwardExtraAcceleration = 0f;
            Check.That(!HipAnchorTranslationPair.CheckActualPrecommit(li, broken), "real precommit rejects the old q-only target");
            l = Pair(parent, 11, true); rr = Pair(parent, 11, false); l.Current = lc; rr.Current = rc;
            return receiver;
        }
        private static void UpwardCompletion()
        {
            foreach (bool trace in new[] { false, true })
            {
                var r = UpwardFixture(trace, out var l, out var rr);
                Check.That(r.CompleteHipTranslationReaction(LoggedStep(), l, rr), "retained completion succeeds: " + r.HipCompletionIssue);
                var c = r._hipAttitudeCommand;
                Check.That(c.Matches(r._hipCommand) && c.UpwardExtraAcceleration == l.Current.UpwardExtraAcceleration,
                    "actual receipt preserves the extra input for next-step deficit feedback");
                Check.Near(c.TargetSystemAcceleration.y, 4f, 1e-5f, "unscaled upward request recovered in this feasible fixture");
                Check.Near(r._pelvis.ForceSum.y, 4f * (l.Current.CommonAcceleration.y - 4f), 1e-5f, "native delta uses common actuator, not SYSTEM target");
                Check.That(r._pelvis.ForceCalls == 1 && r._chest.ForceCalls == 1 && r._pelvis.TorqueCalls == 1,
                    "no additional force owner or repeated native write");
                for (int fault = 0; fault < 9; fault++)
                {
                    r = UpwardFixture(trace, out l, out rr);
                    if (fault == 0) rr.Current.UpwardExtraAcceleration = 0f;
                    if (fault == 1) l.Current.UpwardExtraAcceleration = rr.Current.UpwardExtraAcceleration = 0f;
                    if (fault == 2) l.Current.UpwardExtraAcceleration = rr.Current.UpwardExtraAcceleration = -1f;
                    if (fault == 3) l.Current.UpwardExtraAcceleration = rr.Current.UpwardExtraAcceleration = 4f;
                    if (fault == 4) l.Current.UpwardExtraAcceleration = rr.Current.UpwardExtraAcceleration = float.NaN;
                    if (fault == 5) l.Current.FullFeedbackFeasible = rr.Current.FullFeedbackFeasible = false;
                    if (fault == 6) l.Current.CommonAcceleration.y = rr.Current.CommonAcceleration.y = 13f;
                    if (fault >= 7)
                    {
                        r._hipCommand.UpwardRetentionEligible = false;
                        if (fault == 8) r._hipCommand.UpwardRetentionSelected = false;
                        l.Current.ParentInput = rr.Current.ParentInput = r._hipCommand;
                    }
                    Check.That(!r.CompleteHipTranslationReaction(LoggedStep(), l, rr) && !r._hipAttitudeCommand.Committed,
                        "retention corruption rejected before completion " + fault);
                    Check.That(r._pelvis.ForceCalls == 0 && r._chest.ForceCalls == 0 && r._pelvis.TorqueCalls == 0,
                        "corrupt receipt writes nothing, trace=" + trace);
                }
            }
        }

        private static void CompletionOffsets()
        {
            foreach (int warmup in new[] { 0, 1, 10, 17, 600 })
            foreach (bool trace in new[] { false, true })
            foreach (string arm in new[] { DynamicHipArmA, DynamicHipArmB })
            {
                var r = Fixture(out var left, out var right, warmup + 1, trace); r._control.Arm = arm;
                Check.That(r.CompleteHipTranslationReaction(LoggedStep(), left, right), "same shared step accepts any preparation offset: " + r.HipCompletionIssue);
                Check.That(r._hipAttitudeCommand.Matches(r._hipCommand) && r._expectHipCompletion, "committed physical receipt stays valid");
                Check.That(r._pelvis.ForceCalls == 1 && r._chest.ForceCalls == 1 && r._pelvis.TorqueCalls == 1, "exactly one completion write set");
                Check.Near(r._pelvis.ForceSum.x, .4f, 1e-5f, "allocated pelvis delta unchanged");
                Check.Near(r._chest.ForceSum.x, .6f, 1e-5f, "allocated chest delta unchanged");
                Check.Near(r._pelvis.TorqueSum.y, -.2f, 1e-5f, "relative reaction unchanged");
                Check.That(r.Report.HipReactionRecords == (trace ? 1 : 0), "trace changes evidence only");
                if (trace) Check.That(r.Report.Ring[r._pending].InputStep.Matches(LoggedStep()), "actual producer retained shared context");
            }
        }
        private static void CompletionInvalidInputs()
        {
            for (int scenario = 0; scenario < 17; scenario++)
            {
                var r = Fixture(out var l, out var rr, 1); var step = LoggedStep();
                var frame = r.Report.Ring[r._pending];
                switch (scenario)
                {
                    case 0: frame.InputStep.Epoch++; break;
                    case 1: frame.InputStep.Sequence--; break;
                    case 2: frame.InputStep.SourceSequence--; break;
                    case 3: frame.InputStep.Dt *= 2f; break;
                    case 4: frame.Fixed++; break;
                    case 5: frame.Resolved = true; break;
                    case 6: frame.Censored = true; break;
                    case 7: frame.HipReactionRecorded = true; break;
                    case 8: l.Current.Step.SourceSequence++; break;
                    case 9: rr.Current.ParentInput.ChestBodyId++; break;
                    case 10: rr.Current.Fixed++; break;
                    case 11: l.Current.PeerForce.x++; break;
                    case 12: l.Current.CommonAcceleration.x++; rr.Current.CommonAcceleration.x++; break;
                    case 13: l.Current.ParentTorque.y = 100f; break;
                    case 14: rr.ChildId = l.ChildId; break;
                    case 15: r._bodyMatches = false; break;
                    case 16: step.Dt = float.NaN; break;
                }
                r.Report.Ring[r._pending] = frame;
                Check.That(!r.CompleteHipTranslationReaction(step, l, rr), "invalid input rejected " + scenario);
                Check.That(r.HipCompletionIssue != null && !r._hipAttitudeCommand.Committed, "cause retained without a committed receipt");
                Check.That(r._pelvis.ForceCalls == 0 && r._chest.ForceCalls == 0 && r._pelvis.TorqueCalls == 0, "invalid receipt causes no native writes");
            }
        }
        private static void CompletionTransactions()
        {
            var r = Fixture(out var l, out var rr);
            Check.That(r.CompleteHipTranslationReaction(LoggedStep(), l, rr), "first completion succeeds");
            Check.That(!r.CompleteHipTranslationReaction(LoggedStep(), l, rr) && r._pelvis.ForceCalls == 1 && r.Report.HipReactionRecords == 1,
                "duplicate cannot add force or duplicate evidence");
            for (int fault = 0; fault < 4; fault++)
            {
                r = Fixture(out l, out rr);
                if (fault == 0) r._pelvis.FailForceCalls.Add(1);
                if (fault == 1) r._chest.FailForceCalls.Add(1);
                if (fault >= 2) r._pelvis.FailTorque = true;
                if (fault == 3) r._chest.FailForceCalls.Add(2); // rollback failure must not skip the other owner
                Check.That(!r.CompleteHipTranslationReaction(LoggedStep(), l, rr), "native failure rejects completion");
                Check.That(!r._hipAttitudeCommand.Committed && !r.Report.Finite && r.Report.HipReactionRecords == 0,
                    "failed transaction cannot publish success");
                Check.That(r.HipCompletionIssue.StartsWith("ALLOCATION_WRITE_EXCEPTION:"), "exact native stage survives invalidation");
                Check.Near(r._pelvis.ForceSum.magnitude, 0f, 1e-6f, "every returned pelvis force is rolled back");
                if (fault < 3) Check.Near(r._chest.ForceSum.magnitude, 0f, 1e-6f, "every returned chest force is rolled back");
                else Check.That(r.Report.FaultEvents == 2 && r._pelvis.ForceCalls == 2, "rollback error recorded while other rollback still runs");
            }
        }
        private static void CompletionResolution()
        {
            foreach (int scenario in new[] { 0, 1, 2, 3, 4, 5 })
            {
                var r = Fixture(out var l, out var rr);
                Check.That(r.CompleteHipTranslationReaction(LoggedStep(), l, rr), "completion before resolve");
                var next = LoggedStep(1);
                if (scenario == 1) next.Epoch++;
                if (scenario == 2) next.Sequence++;
                if (scenario == 3) next = LoggedStep();
                if (scenario == 4) next.Dt *= 2f; // response belongs to the old command dt
                if (scenario == 5) r._pelvis.velocity.x = float.NaN;
                r.ResolvePending(next);
                bool valid = scenario == 0 || scenario == 4;
                Check.That(r.Report.Resolved == (valid ? 1 : 0) && r.Report.Censored == (valid ? 0 : 1), "shared next-step validation " + scenario);
                Check.That(r._pending == -1 && r.Report.Finite == valid, "pending consumed once with explicit validity");
            }
        }
        private static void ReceiverOnlyTrace()
        {
            var r = new PelvisDynamicReceiverIsolation(); r.Report.TraceHipTranslation = false;
            r.Produce(LoggedStep()); r.ResolvePending(LoggedStep(1));
            Check.That(r.Report.Resolved == 1 && r.Report.Finite && r.Report.HipReactionRecords == 0,
                "ordinary receiver trace uses shared provenance without requiring a Hip command");
            r = Fixture(out _, out _); r.ResolvePending(LoggedStep(1));
            Check.That(!r.Report.Finite && r.Report.Issue == "RECEIVER_HIP_INPUT_MISSING" && r.Report.Censored == 1,
                "Hip scope still requires actual completed bilateral input");
        }

        private static void CompletionRing()
        {
            var r = new PelvisDynamicReceiverIsolation();
            for (int i = 0; i < 24; i++)
            {
                var step = LoggedStep(i); if (i > 0) r.ResolvePending(step);
                r.Produce(step); var l = Pair(r._hipCommand, i + 11, true); var rr = Pair(r._hipCommand, i + 11, false);
                Check.That(r.CompleteHipTranslationReaction(step, l, rr), "ring receipt at " + i);
                Check.That(r.Report.Ring[r._pending].InputStep.Matches(step), "ring slot owns the current input");
            }
            r.ResolvePending(LoggedStep(24));
            Check.That(r.Report.Resolved == 24 && r.Report.Count == 8 && r.Report.HipReactionRecords == 24 && r.Report.FaultEvents == 0,
                "bounded ring retains correct ownership beyond wrap");
        }
    }
}
