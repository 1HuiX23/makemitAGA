using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Concrete Foot evidence owner. It receives values already sampled by the controller and never
    // reads a Rigidbody, source Transform, clock, report input, or qualification/actuation policy.
    internal sealed class FootControlEvidenceState
    {
        private FootControlEvidenceSnapshot _current;
        private FootTraceEndCapture _capture;
        private FootEvidenceTracePhase _phase;
        private float _errorSum, _anchorSum, _sourceAngularSpeedSum, _sourceFromBindAngleSum;

        internal int LastCompletedControlFixedSequence => _current.FootControllerLastCompletedFixedSequence;

        internal void Reset()
        {
            _current = default;
            _capture = default;
            _phase = FootEvidenceTracePhase.Recording;
            _errorSum = _anchorSum = _sourceAngularSpeedSum = _sourceFromBindAngleSum = 0f;
        }

        internal void ObserveSource(Vector3 renderStepDegrees)
        {
            _current.TraceSourceRenderStepSamples++;
            _current.TraceSourceRenderStepAngleMaxDegrees = Mathf.Max(
                _current.TraceSourceRenderStepAngleMaxDegrees, renderStepDegrees.magnitude);
        }

        // Preserve the existing predecessor publication before construction of the current native
        // readback packet. If a later getter throws, Current remains the last complete observation.
        internal void BeginObservation() => _current.PreviousObservedStep = _current.CurrentObservedStep;

        internal void Observe(bool trace, FootFixedStepEvidence observed)
        {
            _current.CurrentObservedStep = observed;
            if (trace && observed.Valid)
            {
                _current.TraceObservedFixedSamples++;
                _current.TraceObservedAnchorErrorMaxMeters = Mathf.Max(_current.TraceObservedAnchorErrorMaxMeters, observed.AnchorErrorMeters);
                _current.TraceObservedTrackingErrorMaxDegrees = Mathf.Max(_current.TraceObservedTrackingErrorMaxDegrees, observed.TrackingErrorDegrees);
            }
        }

        internal void EvaluateConstraint(int fixedSequence) => _current.AnkleConstraintLastEvaluatedFixedSequence = fixedSequence;
        internal void EvaluateDrive(int fixedSequence) => _current.AnkleDriveLastEvaluatedFixedSequence = fixedSequence;
        internal void CompleteController(int fixedSequence) => _current.FootControllerLastCompletedFixedSequence = fixedSequence;

        // Called at the original finite ObserveFixed tail, after every required controller/schedule succeeds.
        internal void CompleteObservation(float relativeError, float anchorError, float sourceAngularSpeed,
            float sourceFromBindAngle, Vector3 sourceFromBindAxis, Vector3 sourceAngularVelocity)
        {
            _current.TraceSamples++;
            _errorSum += relativeError;
            _anchorSum += anchorError;
            _sourceAngularSpeedSum += sourceAngularSpeed;
            _sourceFromBindAngleSum += sourceFromBindAngle;
            _current.TraceAnkleRelativeErrorMeanDegrees = _errorSum / _current.TraceSamples;
            _current.TraceAnkleRelativeErrorMaxDegrees = Mathf.Max(_current.TraceAnkleRelativeErrorMaxDegrees, relativeError);
            _current.TraceAnkleAnchorErrorMeanMeters = _anchorSum / _current.TraceSamples;
            _current.TraceAnkleAnchorErrorMaxMeters = Mathf.Max(_current.TraceAnkleAnchorErrorMaxMeters, anchorError);
            _current.TraceSourceFromBindAngleMeanDegrees = _sourceFromBindAngleSum / _current.TraceSamples;
            _current.TraceSourceFromBindAngleMaxDegrees = Mathf.Max(_current.TraceSourceFromBindAngleMaxDegrees, sourceFromBindAngle);
            _current.TraceSourceAngularSpeedMean = _sourceAngularSpeedSum / _current.TraceSamples;
            _current.TraceSourceAngularSpeedMax = Mathf.Max(_current.TraceSourceAngularSpeedMax, sourceAngularSpeed);
            if (_current.TraceSamples == 1)
            {
                _current.TraceSourceFromBindAxisMinDegrees = sourceFromBindAxis;
                _current.TraceSourceFromBindAxisMaxDegrees = sourceFromBindAxis;
                _current.TraceSourceAngularVelocityAxisMin = sourceAngularVelocity;
                _current.TraceSourceAngularVelocityAxisMax = sourceAngularVelocity;
            }
            else
            {
                _current.TraceSourceFromBindAxisMinDegrees = ComponentMin(_current.TraceSourceFromBindAxisMinDegrees, sourceFromBindAxis);
                _current.TraceSourceFromBindAxisMaxDegrees = ComponentMax(_current.TraceSourceFromBindAxisMaxDegrees, sourceFromBindAxis);
                _current.TraceSourceAngularVelocityAxisMin = ComponentMin(_current.TraceSourceAngularVelocityAxisMin, sourceAngularVelocity);
                _current.TraceSourceAngularVelocityAxisMax = ComponentMax(_current.TraceSourceAngularVelocityAxisMax, sourceAngularVelocity);
            }
            _current.TraceSourceFromBindAxisAbsMaxDegrees = ComponentMaxAbs(_current.TraceSourceFromBindAxisAbsMaxDegrees, sourceFromBindAxis);
            _current.TraceSourceAngularVelocityAxisAbsMax = ComponentMaxAbs(_current.TraceSourceAngularVelocityAxisAbsMax, sourceAngularVelocity);
        }

        internal void Seal(PhysicsStepContext runtime, SourceStepContext source, FootTurnResponseReceipt response,
            bool driveActive, bool kinematic, bool gravityEnabled, bool colliderEnabled, string driveSummary)
        {
            if (_phase != FootEvidenceTracePhase.Recording) return;
            _capture = new FootTraceEndCapture(runtime, source, _current, response,
                driveActive, kinematic, gravityEnabled, colliderEnabled, driveSummary);
            _phase = FootEvidenceTracePhase.Captured;
        }

        internal void Publish(FootAnkleFoundationReport report)
        {
            report.PublishControlEvidence(_current, _phase, _capture);
        }

        private static Vector3 ComponentMin(Vector3 a, Vector3 b)
        {
            return new Vector3(Mathf.Min(a.x, b.x), Mathf.Min(a.y, b.y), Mathf.Min(a.z, b.z));
        }

        private static Vector3 ComponentMax(Vector3 a, Vector3 b)
        {
            return new Vector3(Mathf.Max(a.x, b.x), Mathf.Max(a.y, b.y), Mathf.Max(a.z, b.z));
        }

        private static Vector3 ComponentMaxAbs(Vector3 current, Vector3 value)
        {
            return new Vector3(
                Mathf.Max(current.x, Mathf.Abs(value.x)),
                Mathf.Max(current.y, Mathf.Abs(value.y)),
                Mathf.Max(current.z, Mathf.Abs(value.z)));
        }
    }
}
