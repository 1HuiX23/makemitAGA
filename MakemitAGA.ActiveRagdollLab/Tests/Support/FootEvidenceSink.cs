// A storage-only projection sink for the real FootControlEvidenceState. Production
// report formatting/qualification and native lifecycle require separate integration and game verification.
namespace MakemitAGA.ActiveRagdollLab.Model
{
    internal sealed class FootAnkleFoundationReport
    {
        internal FootControlEvidenceSnapshot ControlEvidence { get; private set; }
        internal FootEvidenceTracePhase TraceEvidencePhase { get; private set; }
        internal FootTraceEndCapture TraceEndCapture { get; private set; }
        internal void PublishControlEvidence(FootControlEvidenceSnapshot evidence,
            FootEvidenceTracePhase phase, FootTraceEndCapture capture)
        { ControlEvidence = evidence; TraceEvidencePhase = phase; TraceEndCapture = capture; }
    }
}
