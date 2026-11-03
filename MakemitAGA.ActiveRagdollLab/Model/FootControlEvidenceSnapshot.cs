using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Value projection only. FootControlEvidenceState owns all mutation; callers receive a copy.
    internal struct FootControlEvidenceSnapshot
    {
        public FootFixedStepEvidence PreviousObservedStep;
        public FootFixedStepEvidence CurrentObservedStep;
        public int AnkleConstraintLastEvaluatedFixedSequence;
        public int AnkleDriveLastEvaluatedFixedSequence;
        public int FootControllerLastCompletedFixedSequence;
        public int TraceObservedFixedSamples;
        public float TraceObservedAnchorErrorMaxMeters;
        public float TraceObservedTrackingErrorMaxDegrees;
        public int TraceSourceRenderStepSamples;
        public float TraceSourceRenderStepAngleMaxDegrees;

        // Historical TraceSamples counts finite ObserveFixed calls reaching the tail. Preserve that
        // boundary: it is not a count of nonzero writes or of resolved next-step physics responses.
        public int TraceSamples;
        public float TraceAnkleRelativeErrorMeanDegrees;
        public float TraceAnkleRelativeErrorMaxDegrees;
        public float TraceAnkleAnchorErrorMeanMeters;
        public float TraceAnkleAnchorErrorMaxMeters;
        public float TraceSourceFromBindAngleMeanDegrees;
        public float TraceSourceFromBindAngleMaxDegrees;
        public float TraceSourceAngularSpeedMean;
        public float TraceSourceAngularSpeedMax;
        public Vector3 TraceSourceFromBindAxisMinDegrees;
        public Vector3 TraceSourceFromBindAxisMaxDegrees;
        public Vector3 TraceSourceFromBindAxisAbsMaxDegrees;
        public Vector3 TraceSourceAngularVelocityAxisMin;
        public Vector3 TraceSourceAngularVelocityAxisMax;
        public Vector3 TraceSourceAngularVelocityAxisAbsMax;
    }

    internal enum FootEvidenceTracePhase { NotStarted, Recording, Captured }

    // Counts belong to FootTurnEventRecorder's dynamic-parent command/next-settled-response join.
    // Zero captured means no command witness, not a measured zero response. A censored command has
    // no attributable next settled readback; it says nothing about whether native physics ran.
    internal readonly struct FootTurnResponseReceipt
    {
        public readonly int Captured, Resolved, Censored;
        public readonly bool Pending, Sealed;

        public FootTurnResponseReceipt(int captured, int resolved, int censored, bool pending, bool sealedTrace)
        {
            Captured = captured; Resolved = resolved; Censored = censored;
            Pending = pending; Sealed = sealedTrace;
        }
    }

    // EndTrace is not DriveOff. Capture the already-published state at that boundary, without another
    // native read. Value copies and the captured string remain independent of later live/status writes.
    internal readonly struct FootTraceEndCapture
    {
        public readonly bool Available;
        public readonly PhysicsStepContext RuntimeContext;
        public readonly SourceStepContext SourceContext;
        public readonly FootControlEvidenceSnapshot Evidence;
        public readonly FootTurnResponseReceipt TurnResponse;
        public readonly bool DriveActive, Kinematic, GravityEnabled, ColliderEnabled;
        public readonly string AnkleDriveCurrentSummary;

        public FootTraceEndCapture(PhysicsStepContext runtime, SourceStepContext source,
            FootControlEvidenceSnapshot evidence, FootTurnResponseReceipt response,
            bool driveActive, bool kinematic, bool gravityEnabled, bool colliderEnabled, string driveSummary)
        {
            Available = true; RuntimeContext = runtime; SourceContext = source;
            Evidence = evidence; TurnResponse = response;
            DriveActive = driveActive; Kinematic = kinematic;
            GravityEnabled = gravityEnabled; ColliderEnabled = colliderEnabled;
            AnkleDriveCurrentSummary = driveSummary;
        }
    }
}
