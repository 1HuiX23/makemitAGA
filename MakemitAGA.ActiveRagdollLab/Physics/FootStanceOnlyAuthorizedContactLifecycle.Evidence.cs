using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootStanceOnlyAuthorizedContactLifecycle
    {
        // Ordered geometric substitution, not a unique causal force/energy decomposition:
        // raw Foot -> actual Lower ankle position -> actual Lower rotation -> selected relative pose.
        // Fix the CONTROL minimum corner before taking differences; independent minima cannot be added.
        // Native transforms are never read here. A missing diagnostic input cannot alter admission.
        internal struct EntryReferenceGeometry
        {
            internal bool Valid;
            internal int Corners, ControlCorner, ActualCorner;
            internal float AnchorMin, ParentMin, ActualMin;
            internal float SourceAtCorner, AnchorAtCorner, ParentAtCorner, ControlAtCorner, ActualAtCorner;
        }

        private static void CaptureEntryReferenceContext(ref EntryContext c, Vector3 lowerAnkleWorld,
            Vector3 scaledFootAnchor, Quaternion sourceRelativeRotation, Quaternion physicalLowerRotation)
        {
            try
            {
                c.DecompositionValid = c.ReferencesValid && FiniteRotation(sourceRelativeRotation) && FiniteRotation(physicalLowerRotation);
                if (!c.DecompositionValid) return;
                c.RawAtLowerOrigin = lowerAnkleWorld - c.SourceRotation * scaledFootAnchor;
                c.RawAtLowerRotation = physicalLowerRotation * sourceRelativeRotation;
                c.RawAtLowerRotationOrigin = lowerAnkleWorld - c.RawAtLowerRotation * scaledFootAnchor;
                c.DecompositionValid = FiniteVector(c.RawAtLowerOrigin) && FiniteVector(c.RawAtLowerRotationOrigin) && FiniteRotation(c.RawAtLowerRotation);
            }
            catch (Exception) { c.DecompositionValid = false; }
        }

        private static EntryReferenceGeometry BeginEntryReferenceGeometry(bool eligible) => new EntryReferenceGeometry
        {
            Valid = eligible, ControlCorner = -1, ActualCorner = -1,
            AnchorMin = float.PositiveInfinity, ParentMin = float.PositiveInfinity,
            ControlAtCorner = float.PositiveInfinity, ActualMin = float.PositiveInfinity
        };

        private static void ObserveEntryReferenceCorner(ref EntryReferenceGeometry path, EntryContext context,
            Vector3 scaledCorner, float supportTop, int corner, float source, float control, float actual)
        {
            if (!path.Valid) return;
            try
            {
                float anchored = (context.RawAtLowerOrigin + context.SourceRotation * scaledCorner).y - supportTop;
                float parent = (context.RawAtLowerRotationOrigin + context.RawAtLowerRotation * scaledCorner).y - supportTop;
                if (!IsFinite(source) || !IsFinite(anchored) || !IsFinite(parent) || !IsFinite(control) || !IsFinite(actual))
                { path.Valid = false; return; }
                path.Corners++;
                path.AnchorMin = Mathf.Min(path.AnchorMin, anchored);
                path.ParentMin = Mathf.Min(path.ParentMin, parent);
                if (actual < path.ActualMin) { path.ActualMin = actual; path.ActualCorner = corner; }
                if (control < path.ControlAtCorner)
                {
                    path.ControlCorner = corner;
                    path.SourceAtCorner = source;
                    path.AnchorAtCorner = anchored;
                    path.ParentAtCorner = parent;
                    path.ControlAtCorner = control;
                    path.ActualAtCorner = actual;
                }
            }
            catch (Exception) { path.Valid = false; }
        }

        private static string EntryReferenceGeometryText(EntryReferenceGeometry p)
        {
            if (!p.Valid) return "referencePathV041120=NA";
            float anchor = p.AnchorAtCorner - p.SourceAtCorner;
            float parent = p.ParentAtCorner - p.AnchorAtCorner;
            float relative = p.ControlAtCorner - p.ParentAtCorner;
            float tracking = p.ActualAtCorner - p.ControlAtCorner;
            float closure = p.ActualAtCorner - p.SourceAtCorner - (anchor + parent + relative + tracking);
            return "referencePathV041120=ORDERED_SAME_CONTROL_CORNER" +
                "/control-actualCorner=" + p.ControlCorner + "|" + p.ActualCorner +
                "/anchor-parentMinMm=" + F(p.AnchorMin * 1000f) + "|" + F(p.ParentMin * 1000f) +
                "/cornerHeightMm(source-anchor-parent-control-actual)=" + F(p.SourceAtCorner * 1000f) + "|" +
                F(p.AnchorAtCorner * 1000f) + "|" + F(p.ParentAtCorner * 1000f) + "|" + F(p.ControlAtCorner * 1000f) + "|" + F(p.ActualAtCorner * 1000f) +
                "/cornerDeltaMm(anchor-parent-relative-tracking)=" + F(anchor * 1000f) + "|" + F(parent * 1000f) + "|" + F(relative * 1000f) + "|" + F(tracking * 1000f) +
                "/pathClosureMm=" + F(closure * 1000f);
        }

        // Nine inactive pre-control samples, two dumps per trace. First miss is unrestricted;
        // reserve the remaining slot for a source-motion window if the first one was idle/unknown.
        // History is never read by admission/recovery. No per-Fixed formatting or new report file.
        private const int EntryHistoryCapacity = 9;
        private const int MaxEntryHistoryLogs = 2;
        internal bool EntryHistoryAvailable => _entryHistoryLogs < MaxEntryHistoryLogs;
        private readonly EntryFrame[] _entryHistory = new EntryFrame[EntryHistoryCapacity];
        private int _entryHistoryNext, _entryHistoryCount, _entryHistoryLastFixed, _entryHistoryLogs;
        private bool _entryHistoryMotionLogged;

        private struct EntryFrame
        {
            internal int Fixed;
            internal float Dt;
            internal string Gait;
            internal bool Strong, Identity;
            internal EntryGeometry Geometry;
        }

        private void ResetEntryHistory()
        {
            _entryHistoryNext = _entryHistoryCount = _entryHistoryLastFixed = _entryHistoryLogs = 0;
            _entryHistoryMotionLogged = false;
        }

        private void RecordEntryHistory(int fixedSequence, float dt, string gait, bool strong, bool identity, EntryGeometry geometry)
        {
            if (_entryHistoryCount > 0 && fixedSequence != _entryHistoryLastFixed + 1)
                _entryHistoryNext = _entryHistoryCount = 0;
            _entryHistory[_entryHistoryNext] = new EntryFrame
            { Fixed = fixedSequence, Dt = dt, Gait = gait, Strong = strong, Identity = identity, Geometry = geometry };
            _entryHistoryNext = (_entryHistoryNext + 1) % EntryHistoryCapacity;
            if (_entryHistoryCount < EntryHistoryCapacity) _entryHistoryCount++;
            _entryHistoryLastFixed = fixedSequence;
        }

        private void LogEntryHistory(FootAnkleFoundationReport report, int trigger)
        {
            if (_entryHistoryLogs >= MaxEntryHistoryLogs || _entryHistoryCount == 0) return;
            bool sourceMotion = HasSourceMotionHistory();
            if (_entryHistoryLogs > 0 && !_entryHistoryMotionLogged && !sourceMotion) return;
            _entryHistoryLogs++;
            _entryHistoryMotionLogged |= sourceMotion;
            StringBuilder text = new StringBuilder(3072);
            text.Append("[ARL][CONTACT_ENTRY_WINDOW_V041180] side=").Append(Clean(report != null ? report.Side : null));
            text.Append(" | trigger=").Append(trigger).Append(" | policy=").Append(EntryPolicy);
            text.Append(" | selection=FIRST_ANY_THEN_RESERVE_SOURCE_MOTION | sourceMotion=").Append(sourceMotion);
            text.Append(" | domain=INACTIVE_PRE_LIFECYCLE | rows=").Append(_entryHistoryCount);
            text.Append(" | prediction=CONSTANT_POINT_V_ONLY | source=RAW_RENDER | control=ACTUAL_LOWER_PLUS_SELECTED_RELATIVE");
            text.Append(" | referencePath=SOURCE_ANCHOR_PARENT_RELATIVE_ACTUAL | cornerOrder=XYZ_NEGATIVE_FIRST | pathUse=GEOMETRY_ONLY_NO_FEEDBACK");
            for (int i = 0; i < _entryHistoryCount; i++)
            {
                int index = (_entryHistoryNext - _entryHistoryCount + i + EntryHistoryCapacity) % EntryHistoryCapacity;
                EntryFrame frame = _entryHistory[index];
                text.Append("\nentryFrame=").Append(frame.Fixed).Append("/dt=").Append(F(frame.Dt)).Append("/gait=").Append(Clean(frame.Gait));
                text.Append("/strong-identity=").Append(frame.Strong).Append('-').Append(frame.Identity);
                text.Append("/sourcePhase=").Append(Clean(frame.Geometry.SourcePhase));
                text.Append("/source(fresh-heldDepth-dt)=").Append(frame.Geometry.SourceFresh).Append('|')
                    .Append(frame.Geometry.SourceHeldDepth).Append('|').Append(F(frame.Geometry.SourceDt));
                text.Append('/').Append(EntryGeometryText(frame.Geometry));
                text.Append('/').Append(EntryReferenceGeometryText(frame.Geometry.ReferencePath));
                text.Append('/').Append(FootGroundReference.SampleText(frame.Geometry.LandingGoal));
                text.Append('/').Append(FootLandingReferenceEvidence.SampleText(frame.Geometry.LandingPath));
            }
            LogEvent(text.ToString());
        }

        private bool HasSourceMotionHistory()
        {
            for (int i = 0; i < _entryHistoryCount; i++)
            {
                int index = (_entryHistoryNext - _entryHistoryCount + i + EntryHistoryCapacity) % EntryHistoryCapacity;
                string phase = _entryHistory[index].Geometry.SourcePhase;
                // Explicit existing source classes; null/unknown and physical recovery do not prove source motion.
                if (phase == "START" || phase == "STOP" || phase == "TURN" || phase == "ACCEL" ||
                    phase == "DECEL" || phase == "STEADY" || phase == "LOW_SPEED") return true;
            }
            return false;
        }

        private static string EntryGeometryText(EntryGeometry g) =>
            "valid-full-motion-reference=" + g.Valid + "-" + g.FullFootprint + "-" + g.MotionValid + "-" + g.ReferencesValid +
            "/minMm=" + (g.Valid ? F(g.MinClearance * 1000f) : "NA") +
            "/source-controlMinMm=" + (g.ReferencesValid ? F(g.SourceMinClearance * 1000f) + "|" + F(g.ControlMinClearance * 1000f) : "NA") +
            "/predictedMinMm=" + (g.MotionValid ? F(g.PredictedMinClearance * 1000f) : "NA") +
            "/cornerV-planarV-W-anchorMm-ageSec=" + (g.MotionValid ? F(g.MaxCornerNormalSpeed) + "|" + F(g.PlanarSpeed) + "|" +
                F(g.AngularSpeed) + "|" + F(g.AnchorGap * 1000f) + "|" + F(g.SourceAge) : "NA") + "/issue=" + Clean(g.Issue);
    }
}
