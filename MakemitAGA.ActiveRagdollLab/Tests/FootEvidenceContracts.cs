using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace ARL.Contracts
{
    internal static class FootEvidenceContracts
    {
        internal static void Run()
        {
            Check.Run("foot observed/evaluated/completed domains", SampleDomains);
            Check.Run("foot trace-end value capture and restart", Capture);
        }
        private static FootFixedStepEvidence Observed(int sequence, bool valid, float anchor, float tracking) =>
            new FootFixedStepEvidence
            {
                FixedSequence = sequence, Valid = valid, ContactActiveBeforeLifecycle = true,
                AnchorErrorMeters = anchor, TrackingErrorDegrees = tracking,
                LowerAnkleVelocityWorld = new Vector3(sequence, 2f, 3f),
                FootAnkleVelocityWorld = new Vector3(4f, sequence, 6f),
                LowerAngularVelocityWorld = new Vector3(7f, 8f, sequence),
                FootAngularVelocityWorld = new Vector3(10f, 11f, sequence)
            };

        private static void SampleDomains()
        {
            var state = new FootControlEvidenceState(); var report = new FootAnkleFoundationReport();
            state.Publish(report);
            Check.That(report.TraceEvidencePhase == FootEvidenceTracePhase.NotStarted && !report.TraceEndCapture.Available, "before trace there is no captured result");
            state.Reset();
            state.ObserveSource(new Vector3(3f, 4f, 0f));
            state.BeginObservation(); state.Observe(true, Observed(11, true, .003f, 5f));
            state.EvaluateConstraint(11); state.EvaluateDrive(11); state.CompleteController(11);
            state.CompleteObservation(3f, .002f, 4f, 10f, new Vector3(-1f, 2f, 3f), new Vector3(2f, -3f, 4f));
            state.BeginObservation(); state.Observe(true, Observed(12, false, .5f, 100f));
            state.EvaluateConstraint(12);
            state.Publish(report);
            var evidence = report.ControlEvidence;
            Check.That(evidence.PreviousObservedStep.FixedSequence == 11 && evidence.CurrentObservedStep.FixedSequence == 12, "failed observation still has its own identity");
            Check.That(evidence.PreviousObservedStep.ContactActiveBeforeLifecycle && evidence.PreviousObservedStep.LowerAnkleVelocityWorld.x == 11f && evidence.CurrentObservedStep.FootAngularVelocityWorld.z == 12f, "pre-lifecycle contact and velocity samples stay attached to their own packet");
            Check.That(evidence.TraceObservedFixedSamples == 1 && evidence.TraceObservedTrackingErrorMaxDegrees == 5f, "invalid observation does not contribute to valid observation maxima");
            Check.That(evidence.AnkleConstraintLastEvaluatedFixedSequence == 12 && evidence.AnkleDriveLastEvaluatedFixedSequence == 11 && state.LastCompletedControlFixedSequence == 11, "a gate failure cannot advance unevaluated drive or completed controller identity");
            Check.That(evidence.TraceSamples == 1 && evidence.TraceSourceRenderStepSamples == 1 && evidence.TraceSourceRenderStepAngleMaxDegrees == 5f, "tail samples and render samples stay separate");
            // A second finite tail keeps its sample count distinct from the absolute Fixed identity.
            // No force is simulated here; reaching this tail is decided by the real owner/native gates.
            state.BeginObservation(); state.Observe(true, Observed(13, true, .004f, 7f));
            state.CompleteController(13);
            state.CompleteObservation(7f, .004f, 8f, 20f, new Vector3(-3f, 1f, 5f), new Vector3(1f, -5f, 6f));
            state.Publish(report); evidence = report.ControlEvidence;
            Check.That(evidence.TraceSamples == 2 && state.LastCompletedControlFixedSequence == 13, "trace sample count and absolute completed identity retain different meanings");
            Check.Near(evidence.TraceAnkleRelativeErrorMeanDegrees, 5f, .000001f, "trace mean includes both original tail observations");
            Check.Near(evidence.TraceAnkleAnchorErrorMeanMeters, .003f, .000001f, "anchor mean retains original units");
            Check.That(evidence.TraceSourceFromBindAxisMinDegrees.x == -3f && evidence.TraceSourceFromBindAxisMaxDegrees.z == 5f && evidence.TraceSourceAngularVelocityAxisAbsMax.y == 5f, "axis min/max and absolute max preserve signed evidence");
            state.BeginObservation(); // A subsequent native packet read fails before Observe can publish it.
            state.Publish(report); evidence = report.ControlEvidence;
            Check.That(evidence.PreviousObservedStep.FixedSequence == 13 && evidence.CurrentObservedStep.FixedSequence == 13 && evidence.TraceSamples == 2, "predecessor advances at its original boundary even when the next packet is never published");
        }

        private static void Capture()
        {
            var state = new FootControlEvidenceState(); var report = new FootAnkleFoundationReport();
            state.Reset(); state.BeginObservation(); state.Observe(true, Observed(8, true, .002f, 2f)); state.CompleteController(8);
            var source = new SourceStepContext { Epoch = 1, Sequence = 1, Time = .1f, Dt = .01f };
            var response = new FootTurnResponseReceipt(8, 7, 1, false, true);
            state.Seal(RuntimeContracts.Step(8), source, response, true, false, true, true, "drive=True");
            state.Publish(report); var first = report.TraceEndCapture;
            Check.That(first.Available && report.TraceEvidencePhase == FootEvidenceTracePhase.Captured, "trace end captures a distinct result");
            Check.That(first.DriveActive && !first.Kinematic && first.RuntimeContext.Sequence == 8, "normal trace end does not pretend the drive was already off");
            Check.That(first.TurnResponse.Captured == 8 && first.TurnResponse.Resolved == 7 && first.TurnResponse.Censored == 1 && !first.TurnResponse.Pending, "censored tail remains distinct from a measured zero response");
            state.BeginObservation(); state.Observe(false, Observed(9, true, .01f, 10f)); state.CompleteController(9); state.Publish(report);
            Check.That(report.ControlEvidence.CurrentObservedStep.FixedSequence == 9 && report.TraceEndCapture.Evidence.CurrentObservedStep.FixedSequence == 8, "live observation advances without mutating captured evidence");
            var copy = report.ControlEvidence; copy.FootControllerLastCompletedFixedSequence = 999;
            Check.That(state.LastCompletedControlFixedSequence == 9 && report.ControlEvidence.FootControllerLastCompletedFixedSequence == 9, "mutating a returned value copy cannot alter owner or projection");
            state.Seal(RuntimeContracts.Step(9), source, default, false, true, false, false, "drive=False"); state.Publish(report);
            Check.That(report.TraceEndCapture.AnkleDriveCurrentSummary == "drive=True" && report.TraceEndCapture.RuntimeContext.Sequence == 8, "duplicate seal cannot replace the original stop boundary");
            state.Reset(); state.Publish(report);
            Check.That(report.TraceEvidencePhase == FootEvidenceTracePhase.Recording && !report.TraceEndCapture.Available && report.ControlEvidence.TraceSamples == 0 && state.LastCompletedControlFixedSequence == 0, "new trace resets only this evidence domain and clears the old capture");
            state.Seal(RuntimeContracts.Step(10), source, default, false, true, false, false, "drive=False"); state.Publish(report);
            Check.That(!report.TraceEndCapture.DriveActive && report.TraceEndCapture.Kinematic, "a trace ended after safe-off records the actual off boundary");
        }
    }
}
