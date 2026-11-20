using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    // Copy only scalars used by this text view. Reading a completed snapshot never touches its native owner.
    internal struct FootContactEntrySnapshot
    {
        internal string TracePolicy, LivePolicy;
        internal bool Ready, AtEnable, HistoryMotion;
        internal int Fixed, Missed, Recovered, RecoveryReject, HistoryCount, GeometryReject, GeometryInvalid, Evaluated;
        internal float HoldSeconds, Peak, EnabledPeak, IdentityHold;
        internal Box Geometry;
        internal struct Box
        {
            internal bool Valid, FullFootprint, MotionValid, ReferencesValid;
            internal float MinClearance, SourceMinClearance, ControlMinClearance, PredictedMinClearance;
            internal float MaxCornerNormalSpeed, PlanarSpeed, AngularSpeed, AnchorGap, SourceAge;
            internal string Issue;
            internal string Format() =>
                "valid-full-motion-reference=" + Valid + "-" + FullFootprint + "-" + MotionValid + "-" + ReferencesValid +
                "/minMm=" + (Valid ? F(MinClearance * 1000f) : "NA") +
                "/source-controlMinMm=" + (ReferencesValid ? F(SourceMinClearance * 1000f) + "|" + F(ControlMinClearance * 1000f) : "NA") +
                "/predictedMinMm=" + (MotionValid ? F(PredictedMinClearance * 1000f) : "NA") +
                "/cornerV-planarV-W-anchorMm-ageSec=" + (MotionValid ? F(MaxCornerNormalSpeed) + "|" + F(PlanarSpeed) + "|" +
                    F(AngularSpeed) + "|" + F(AnchorGap * 1000f) + "|" + F(SourceAge) : "NA") + "/issue=" + Clean(Issue);
        }
        internal string Format() =>
            "tracePolicy=" + TracePolicy + "/livePolicy=" + LivePolicy + "/entryEvidenceReady=" + Ready +
                "/geometrySampleFixed=" + Fixed + "/geometrySampleRole=" + (Fixed == 0 ? "NONE" : (AtEnable ? "ENABLE_PRE_CONTROL" : "INACTIVE_PRE_CONTROL")) +
                "/boxValid=" + Geometry.Valid + "/fullFootprint=" + Geometry.FullFootprint +
                "/boxMinMm=" + (Geometry.Valid ? F(Geometry.MinClearance * 1000f) : "NA") +
                "/geometryIssue=" + Clean(Geometry.Issue) + "/missedStances=" + Missed +
                "/recovered-unrecovered=" + Recovered + "-" + (Missed - Recovered) +
                "/recoveryHoldSec=" + F(HoldSeconds) + "/recoveryRejected=" + RecoveryReject +
                "/history(logs-sourceMotionCovered)=" + HistoryCount + "|" + HistoryMotion +
                "/reject-invalid=" + GeometryReject + "-" + GeometryInvalid +
                "/evaluatedActivations=" + Evaluated + "/penetrationPeakMm=" + F(Peak * 1000f) +
                "/enabledPenetrationPeakMm=" + F(EnabledPeak * 1000f) +
                "/traceIdentityHoldSec=" + F(IdentityHold) +
                "/maxPenetration-gapMm=" + F(FootContactGeometryLimits.OrdinaryPenetrationMeters * 1000f) + "-" + F(FootContactGeometryLimits.RecoveryMaxGapMeters * 1000f) +
                "/recoveryLimits(mm-sec-cornerV-planarV-W-anchorMm-ageSec)=" + F(FootContactGeometryLimits.RecoveryPenetrationMeters * 1000f) + "-" +
                F(FootContactGeometryLimits.RecoveryHoldSeconds) + "-" + F(FootContactGeometryLimits.RecoveryMaxCornerNormalSpeed) + "-" + F(FootContactGeometryLimits.RecoveryMaxPlanarSpeed) + "-" +
                F(FootContactGeometryLimits.RecoveryMaxAngularSpeed) + "-" + F(FootContactGeometryLimits.RecoveryMaxAnchorGapMeters * 1000f) + "-" + F(FootContactGeometryLimits.RecoveryMaxSourceAgeSeconds) +
                "/geometrySample=" + Geometry.Format();
        private static string F(float value) => value.ToString("0.######", CultureInfo.InvariantCulture);
        private static string Clean(string value) => string.IsNullOrEmpty(value) ? "<none>" :
            value.Replace('\r', ' ').Replace('\n', ' ');
    }

    internal sealed partial class FootAnkleFoundationReport
    {
        private FootContactEntrySnapshot _entrySnapshot;
        private bool _hasEntrySnapshot;
        private string _entryText;
        public string ContactEntrySummary
        {
            get
            {
                if (!_hasEntrySnapshot || _entryText != null) return _entryText;
                using var measurement = LandingObservationTiming.Run.Measure(LandingObservationTiming.Stage.EntryText);
                return _entryText = _entrySnapshot.Format();
            }
        }
        internal void PublishEntrySummary(in FootContactEntrySnapshot snapshot, bool defer)
        {
            _entrySnapshot = snapshot;
            _hasEntrySnapshot = true;
            _entryText = null;
            if (!defer) _ = ContactEntrySummary;
        }
    }
}
