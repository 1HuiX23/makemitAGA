using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class FootStanceOnlyAuthorizedContactLifecycle
    {
        // v111 A is the strict v110 B entry, including its missed-candidate latch.
        // B adds only quiet, shallow requalification of the SAME still-confirmed candidate.
        // Ordinary entry stays at 2 mm. Recovery's 3 mm / 150 ms are experimental, not a safety certification.
        // No pose correction, force, new raycast or authority outside confirmed strong STANCE.
        internal const float ContinuousIdentityHoldSeconds = FootGameNativeLocomotionPhaseObserver.PhaseConfirmSeconds;
        internal const float MaxEntryPenetrationMeters = 0.002f;
        internal const float MaxEntryGapMeters = 0.025f;
        internal const float RecoveryMaxPenetrationMeters = 0.003f;
        internal const float RecoveryHoldSeconds = 0.15f;
        internal const float RecoveryMaxCornerNormalSpeed = 0.03f;
        internal const float RecoveryMaxPlanarSpeed = 0.05f;
        internal const float RecoveryMaxAngularSpeed = 0.5f;
        internal const float RecoveryMaxAnchorGapMeters = 0.005f;
        internal const float RecoveryMaxSourceAgeSeconds = 0.1f;
        private const int MaxMissedEntryLogs = 8;
        private bool _continuousEntry, _entryRecovery;
        private bool _entryWindowMissed, _entryMissCounted, _entryTraceContinuous, _entryTraceRecovery;
        private bool _entryRecoveryPending;
        private float _entryRecoverySeconds;
        private int _entryRecoveryLastFixed, _entryRecoveredStances, _entryRecoveryRejectSamples;
        private int _entryMissedStances, _entryGeometryInvalidSamples, _entryGeometryRejectSamples;
        private int _entryEvaluatedActivations, _entryMissedLogs;
        private float _entryPenetrationPeak, _entryEnabledPenetrationPeak;
        private EntryGeometry _entryGeometry;
        private int _entryGeometryFixed;
        private bool _entryGeometryAtEnable;

        internal bool ContinuousEntry => _continuousEntry;
        internal bool EntryRecovery => _entryRecovery;
        internal string EntryPolicy => EntryPolicyName(_continuousEntry, _entryRecovery);
        private static string EntryPolicyName(bool continuous, bool recovery) => !continuous ? "LEGACY_SERIAL_ENTRY" :
            (recovery ? "SHALLOW_RECOVERY_V041110" : "STRICT_BOX_V041100");

        internal void SelectContinuousEntry(bool enabled, FootAnkleFoundationReport report, bool recovery = false)
        {
            if (_enabled) throw new InvalidOperationException("CONTACT_ENTRY_POLICY_REQUIRES_LIFECYCLE_OFF");
            if (recovery && !enabled) throw new InvalidOperationException("CONTACT_RECOVERY_REQUIRES_CONTINUOUS_ENTRY");
            _continuousEntry = enabled;
            _entryRecovery = recovery;
            ResetStanceHold();
            CopyToReport(report);
        }

        private void ResetEntryTrace()
        {
            _entryWindowMissed = _entryMissCounted = false;
            _entryTraceContinuous = _continuousEntry;
            _entryTraceRecovery = _entryRecovery;
            _entryRecoveredStances = _entryRecoveryRejectSamples = 0;
            ResetEntryRecoveryHold();
            ResetEntryHistory();
            _entryMissedStances = _entryGeometryInvalidSamples = _entryGeometryRejectSamples = 0;
            _entryEvaluatedActivations = _entryMissedLogs = 0;
            _entryPenetrationPeak = _entryEnabledPenetrationPeak = 0f;
            _entryGeometry = new EntryGeometry();
            _entryGeometryFixed = 0;
            _entryGeometryAtEnable = false;
        }

        internal struct EntryGeometry
        {
            internal string SourcePhase;
            internal bool SourceFresh;
            internal int SourceHeldDepth;
            internal float SourceDt;
            internal bool Valid, FullFootprint, MotionValid, ReferencesValid;
            internal float MinClearance, PredictedMinClearance, MaxCornerNormalSpeed;
            internal float PlanarSpeed, AngularSpeed, AnchorGap, SourceAge;
            internal float SourceMinClearance, ControlMinClearance;
            internal string Issue;
            internal EntryReferenceGeometry ReferencePath;
            internal FootGroundReference.GroundSample LandingGoal;
            internal FootLandingReferenceEvidence.LandingPath LandingPath;
        }

        internal struct EntryContext
        {
            internal string SourcePhase;
            internal bool SourceFresh;
            internal int SourceHeldDepth;
            internal float SourceDt;
            internal int FixedSequence;
            internal bool MotionValid, ReferencesValid;
            internal Vector3 Center, Velocity, AngularVelocity, SourceOrigin, ControlOrigin;
            internal Quaternion SourceRotation, ControlRotation;
            internal float AnchorGap, SourceAge;
            internal bool DecompositionValid;
            internal Vector3 RawAtLowerOrigin, RawAtLowerRotationOrigin;
            internal Quaternion RawAtLowerRotation;
            internal FootGroundReference.GroundSample LandingGoal;
            internal FootLandingReferenceEvidence.LandingPath LandingPath;
        }

        // Read actual COM kinematics once; the same eight corners supply motion and signed clearance.
        // Reference poses are diagnostic only. They cannot grant contact or alter the controller target.
        internal static EntryContext CaptureEntryContext(int fixedSequence, Rigidbody body, Vector3 lowerAnkleWorld,
            Vector3 footAnchorLocal, Quaternion controlRotation, Vector3 sourceOrigin, Quaternion sourceRotation,
            float anchorGap, float sourceAge, Quaternion sourceRelativeRotation = default, Quaternion physicalLowerRotation = default)
        {
            EntryContext c = new EntryContext { FixedSequence = fixedSequence, AnchorGap = anchorGap, SourceAge = sourceAge };
            try
            {
                if (body == null) return c;
                c.Center = body.worldCenterOfMass;
                c.Velocity = body.velocity;
                c.AngularVelocity = body.angularVelocity;
                c.MotionValid = !body.isKinematic && FiniteVector(c.Center) && FiniteVector(c.Velocity) &&
                    FiniteVector(c.AngularVelocity) && IsFinite(anchorGap) && anchorGap >= 0f &&
                    IsFinite(sourceAge) && sourceAge >= 0f;
                Vector3 scale = body.transform.lossyScale;
                c.SourceOrigin = sourceOrigin;
                c.SourceRotation = sourceRotation;
                c.ControlRotation = controlRotation;
                c.ControlOrigin = lowerAnkleWorld - controlRotation * Vector3.Scale(footAnchorLocal, scale);
                c.ReferencesValid = FiniteVector(scale) && scale.x > 0f && scale.y > 0f && scale.z > 0f &&
                    FiniteVector(sourceOrigin) && FiniteVector(c.ControlOrigin) && FiniteRotation(sourceRotation) && FiniteRotation(controlRotation);
                CaptureEntryReferenceContext(ref c, lowerAnkleWorld, Vector3.Scale(footAnchorLocal, scale), sourceRelativeRotation, physicalLowerRotation);
            }
            catch (Exception) { c.MotionValid = c.ReferencesValid = false; }
            return c;
        }

        private static bool FiniteRotation(Quaternion q) => IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w) &&
            Mathf.Abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1f) < 0.01f;

        // Disabled Collider.bounds can be empty. Read the actual Box vertices via its transform instead.
        // The support owner exposes only its validated, axis-aligned, unit-scale controlled Box.
        internal static EntryGeometry CaptureEntryGeometry(BoxCollider foot, FootSupportSurfaceIsolation support, int identity,
            EntryContext context = default, float dt = 0f)
        {
            EntryGeometry g = new EntryGeometry
            {
                Issue = "CONTROLLED_BOX_REQUIRED", LandingGoal = context.LandingGoal, LandingPath = context.LandingPath,
                SourcePhase = context.SourcePhase, SourceFresh = context.SourceFresh,
                SourceHeldDepth = context.SourceHeldDepth, SourceDt = context.SourceDt
            };
            try
            {
                if (foot == null || support == null || !support.TryGetEntryBox(identity, out Vector3 center, out Vector3 half)) return g;
                g.LandingPath.SupportTop = center.y + half.y;
                g.LandingPath.Valid &= IsFinite(g.LandingPath.SupportTop);
                Vector3 size = foot.size, localCenter = foot.center;
                if (!FiniteVector(size) || !FiniteVector(localCenter) || size.x <= 0f || size.y <= 0f || size.z <= 0f)
                { g.Issue = "FOOT_BOX_INVALID"; return g; }
                Transform transform = foot.transform;
                Vector3 scale = transform.lossyScale;
                if (!FiniteVector(scale) || scale.x <= 0f || scale.y <= 0f || scale.z <= 0f)
                { g.Issue = "FOOT_SCALE_INVALID"; return g; }
                float min = float.PositiveInfinity;
                float predicted = float.PositiveInfinity, sourceMin = float.PositiveInfinity, controlMin = float.PositiveInfinity;
                float maxNormalSpeed = 0f;
                bool motion = context.MotionValid && IsFinite(dt) && dt > 0f;
                bool inside = true;
                EntryReferenceGeometry path = BeginEntryReferenceGeometry(context.DecompositionValid);
                // All eight corners, including the corners outside the old five-point inset footprint.
                for (int x = -1; x <= 1; x += 2)
                    for (int y = -1; y <= 1; y += 2)
                        for (int z = -1; z <= 1; z += 2)
                        {
                            Vector3 local = localCenter + new Vector3(x * size.x, y * size.y, z * size.z) * 0.5f;
                            Vector3 p = transform.TransformPoint(local);
                            if (!FiniteVector(p)) { g.Issue = "FOOT_CORNER_NONFINITE"; return g; }
                            min = Mathf.Min(min, p.y - (center.y + half.y));
                            inside &= Mathf.Abs(p.x - center.x) <= half.x && Mathf.Abs(p.z - center.z) <= half.z;
                            if (motion)
                            {
                                Vector3 v = context.Velocity + Vector3.Cross(context.AngularVelocity, p - context.Center);
                                maxNormalSpeed = Mathf.Max(maxNormalSpeed, Mathf.Abs(v.y));
                                predicted = Mathf.Min(predicted, p.y + v.y * dt - (center.y + half.y));
                            }
                            if (context.ReferencesValid)
                            {
                                Vector3 offset = Vector3.Scale(local, scale);
                                float source = (context.SourceOrigin + context.SourceRotation * offset).y - (center.y + half.y);
                                float control = (context.ControlOrigin + context.ControlRotation * offset).y - (center.y + half.y);
                                sourceMin = Mathf.Min(sourceMin, source);
                                controlMin = Mathf.Min(controlMin, control);
                                if (path.Valid)
                                {
                                    int corner = (x + 1) * 2 + (y + 1) + (z + 1) / 2;
                                    ObserveEntryReferenceCorner(ref path, context, offset, center.y + half.y, corner, source, control, p.y - (center.y + half.y));
                                }
                            }
                        }
                g.Valid = IsFinite(min);
                g.FullFootprint = inside;
                g.MinClearance = min;
                g.MotionValid = motion && IsFinite(predicted) && IsFinite(maxNormalSpeed);
                g.PredictedMinClearance = predicted;
                g.MaxCornerNormalSpeed = maxNormalSpeed;
                g.PlanarSpeed = new Vector3(context.Velocity.x, 0f, context.Velocity.z).magnitude;
                g.AngularSpeed = context.AngularVelocity.magnitude;
                g.AnchorGap = context.AnchorGap;
                g.SourceAge = context.SourceAge;
                g.ReferencesValid = context.ReferencesValid && IsFinite(sourceMin) && IsFinite(controlMin);
                g.SourceMinClearance = sourceMin;
                g.ControlMinClearance = controlMin;
                path.Valid &= g.Valid && g.ReferencesValid && path.Corners == 8;
                g.ReferencePath = path;
                g.Issue = g.Valid ? (inside ? "OK" : "FOOTPRINT_OUTSIDE_CONTROLLED_BOX") : "CLEARANCE_NONFINITE";
                return g;
            }
            catch (Exception e) { g.Issue = "ENTRY_GEOMETRY_READ:" + e.GetType().Name; return g; }
        }

        private static bool FiniteVector(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);

        private bool ContinuousEntryReady(bool confirmedStance, bool legacyGeometryReady, int fixedSequence, float dt,
            string gaitState, FootAnkleFoundationReport report)
        {
            if (!_entryGeometry.Valid)
            {
                _entryGeometryInvalidSamples++;
                Invalidate("STANCE_ENTRY_GEOMETRY_INVALID:" + _entryGeometry.Issue, report);
                return false;
            }
            _entryPenetrationPeak = Mathf.Max(_entryPenetrationPeak, -_entryGeometry.MinClearance);
            if (_entryGeometry.MinClearance < -MaxEntryPenetrationMeters && !_entryWindowMissed)
            {
                _entryWindowMissed = true;
            }
            if (confirmedStance && _entryWindowMissed && !_entryMissCounted)
            {
                _entryMissCounted = true;
                _entryMissedStances++;
                if (_entryMissedLogs < MaxMissedEntryLogs)
                {
                    _entryMissedLogs++;
                    LogEvent("[ARL][STANCE_CONTACT_V0470] side=" + Clean(report != null ? report.Side : null) +
                        " | action=ENTRY_MISSED | fixed=" + fixedSequence + " | gait=" + Clean(gaitState) +
                        " | boxMinMm=" + F(_entryGeometry.MinClearance * 1000f) +
                        " | policy=" + EntryPolicy + " | contactRemainsOff=True | datasetEntryReady=False");
                }
                LogEntryHistory(report, fixedSequence);
            }
            _entryRecoveryPending = false;
            if (_entryWindowMissed && _entryRecovery &&
                RecoveryEntryReady(confirmedStance, legacyGeometryReady, fixedSequence, dt))
            {
                _entryRecoveryPending = true;
                _lastReason = "SHALLOW_STANCE_ENTRY_REQUALIFIED";
                return true;
            }
            bool geometryReady = !_entryWindowMissed && _entryGeometry.FullFootprint && legacyGeometryReady &&
                _entryGeometry.MinClearance <= MaxEntryGapMeters;
            if (!geometryReady)
            {
                _entryGeometryRejectSamples++;
                _activationGeometryReadySamples = 0;
                _lastReason = _entryWindowMissed
                    ? (_entryRecovery ? "STANCE_ENTRY_MISSED_WAIT_SHALLOW_REQUALIFICATION" : "STANCE_ENTRY_WINDOW_MISSED_WAIT_NEW_STANCE")
                    : "STANCE_ENTRY_BOX_NOT_READY";
                return false;
            }
            _activationGeometryReadySamples++;
            _lastReason = "STANCE_ENTRY_CONCURRENT_QUALIFICATION";
            return confirmedStance && _qualifiedStanceSeconds + 0.000001f >= ContinuousIdentityHoldSeconds &&
                _activationGeometryReadySamples >= MinActivationGeometryReadySamples;
        }

        private bool RecoveryEntryReady(bool confirmedStance, bool legacyGeometryReady, int fixedSequence, float dt)
        {
            EntryGeometry g = _entryGeometry;
            // One-step constant-point-velocity projection is a conservative admission screen, NOT a solver prediction.
            // Body quietness is physical: a stationary animation cannot stand in for a moving Foot.
            bool ready = confirmedStance && _entryMissCounted && legacyGeometryReady && g.Valid && g.FullFootprint &&
                g.MotionValid && g.SourceAge <= RecoveryMaxSourceAgeSeconds && g.AnchorGap <= RecoveryMaxAnchorGapMeters &&
                g.MinClearance >= -RecoveryMaxPenetrationMeters && g.MinClearance <= MaxEntryGapMeters &&
                g.PredictedMinClearance >= -RecoveryMaxPenetrationMeters &&
                g.MaxCornerNormalSpeed <= RecoveryMaxCornerNormalSpeed && g.PlanarSpeed <= RecoveryMaxPlanarSpeed &&
                g.AngularSpeed <= RecoveryMaxAngularSpeed;
            if (!ready)
            {
                _entryRecoveryRejectSamples++;
                ResetEntryRecoveryHold();
                return false;
            }
            // First valid sample starts at zero. Duplicate/skipped local steps cannot borrow a prior hold.
            _entryRecoverySeconds = _entryRecoveryLastFixed > 0 && fixedSequence == _entryRecoveryLastFixed + 1
                ? _entryRecoverySeconds + dt : 0f;
            _entryRecoveryLastFixed = fixedSequence;
            return _entryRecoverySeconds + 0.000001f >= RecoveryHoldSeconds &&
                _qualifiedStanceSeconds + 0.000001f >= ContinuousIdentityHoldSeconds;
        }

        private void ResetEntryRecoveryHold()
        {
            _entryRecoverySeconds = 0f;
            _entryRecoveryLastFixed = 0;
            _entryRecoveryPending = false;
        }

        private void CommitEntryActivation()
        {
            // Called only after EnablePreparedArmB succeeds. A native failure cannot count as recovered.
            if (_entryRecoveryPending && _entryMissCounted) _entryRecoveredStances++;
        }

        private void CopyEntryToReport(FootAnkleFoundationReport report)
        {
            report.ContactEntryEvidenceReady = !_entryTraceContinuous || (_finite && _entryMissedStances == _entryRecoveredStances &&
                _entryGeometryInvalidSamples == 0 && _entryEvaluatedActivations == _contactActivations && _contactActivations > 0);
            report.ContactEntrySummary = "tracePolicy=" + EntryPolicyName(_entryTraceContinuous, _entryTraceRecovery) + "/livePolicy=" + EntryPolicy + "/entryEvidenceReady=" + report.ContactEntryEvidenceReady +
                "/geometrySampleFixed=" + _entryGeometryFixed + "/geometrySampleRole=" + (_entryGeometryFixed == 0 ? "NONE" : (_entryGeometryAtEnable ? "ENABLE_PRE_CONTROL" : "INACTIVE_PRE_CONTROL")) +
                "/boxValid=" + _entryGeometry.Valid + "/fullFootprint=" + _entryGeometry.FullFootprint +
                "/boxMinMm=" + (_entryGeometry.Valid ? F(_entryGeometry.MinClearance * 1000f) : "NA") +
                "/geometryIssue=" + Clean(_entryGeometry.Issue) + "/missedStances=" + _entryMissedStances +
                "/recovered-unrecovered=" + _entryRecoveredStances + "-" + (_entryMissedStances - _entryRecoveredStances) +
                "/recoveryHoldSec=" + F(_entryRecoverySeconds) + "/recoveryRejected=" + _entryRecoveryRejectSamples +
                "/history(logs-sourceMotionCovered)=" + _entryHistoryLogs + "|" + _entryHistoryMotionLogged +
                "/reject-invalid=" + _entryGeometryRejectSamples + "-" + _entryGeometryInvalidSamples +
                "/evaluatedActivations=" + _entryEvaluatedActivations + "/penetrationPeakMm=" + F(_entryPenetrationPeak * 1000f) +
                "/enabledPenetrationPeakMm=" + F(_entryEnabledPenetrationPeak * 1000f) +
                "/traceIdentityHoldSec=" + F(_entryTraceContinuous ? ContinuousIdentityHoldSeconds : MinQualifiedStanceHoldSeconds) +
                "/maxPenetration-gapMm=" + F(MaxEntryPenetrationMeters * 1000f) + "-" + F(MaxEntryGapMeters * 1000f) +
                "/recoveryLimits(mm-sec-cornerV-planarV-W-anchorMm-ageSec)=" + F(RecoveryMaxPenetrationMeters * 1000f) + "-" +
                F(RecoveryHoldSeconds) + "-" + F(RecoveryMaxCornerNormalSpeed) + "-" + F(RecoveryMaxPlanarSpeed) + "-" +
                F(RecoveryMaxAngularSpeed) + "-" + F(RecoveryMaxAnchorGapMeters * 1000f) + "-" + F(RecoveryMaxSourceAgeSeconds) +
                "/geometrySample=" + EntryGeometryText(_entryGeometry);
        }

        private static void LogEvent(string text)
        {
            try { Plugin.Logger?.LogInfo(text); }
            catch (Exception) { } // A logger failure must not turn a completed native toggle into a failed control step.
        }
    }
}
