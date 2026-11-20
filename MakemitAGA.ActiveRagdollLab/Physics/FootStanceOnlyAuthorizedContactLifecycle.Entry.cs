using System;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;
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
        internal const float MaxEntryPenetrationMeters = FootContactGeometryLimits.OrdinaryPenetrationMeters;
        internal const float MaxEntryGapMeters = FootContactGeometryLimits.RecoveryMaxGapMeters;
        internal const float RecoveryHoldSeconds = FootContactGeometryLimits.RecoveryHoldSeconds;
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

        // Read-only event markers. Keep the first depth violation even when stance confirmation arrives later.
        internal bool FirstMissThisStep { get; private set; }
        private int _depthLatchFixed;
        internal int FirstMissDepthFixed => FirstMissThisStep ? _depthLatchFixed : 0;
        internal bool MissInProgress => _entryMissCounted;
        internal bool OrdinaryEnabledThisStep { get; private set; }
        internal bool RecoveryEnabledThisStep { get; private set; }
        internal bool EntryHistoryAvailable => _entryHistoryLogs < MaxEntryHistoryLogs;
        internal EntryGeometry DiagnosticEntryGeometry { get; private set; }
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
            _entryTraceContinuous = _continuousEntry;
            _entryTraceRecovery = _entryRecovery;
            _entryRecoveredStances = _entryRecoveryRejectSamples = 0;
            ResetEntryHistory();
            // A still-unresolved miss can recover during the new trace without another first-miss event.
            _entryMissedStances = _enabled && _entryMissCounted && _state != StateActive ? 1 : 0;
            _entryGeometryInvalidSamples = _entryGeometryRejectSamples = 0;
            _entryEvaluatedActivations = _entryMissedLogs = 0;
            _entryPenetrationPeak = _entryEnabledPenetrationPeak = 0f;
        }

        internal struct EntryGeometry
        {
            internal float SourceAnkleY;
            internal string SourcePhase;
            internal bool SourceFresh;
            internal int SourceHeldDepth;
            internal int SupportId;
            internal Vector3 FloorCenter, FloorHalf;
            internal Vector3 LocalBoxCenter, LocalBoxSize, BoxScale;
            internal bool LowestMotionValid, DiagnosticSelectedBox;
            internal float LowestCornerNormalVelocity;
            internal float SourceDt;
            internal bool Valid, FullFootprint, MotionValid, ReferencesValid;
            internal float MinClearance, PredictedMinClearance, MaxCornerNormalSpeed;
            internal float PlanarSpeed, AngularSpeed, AnchorGap, SourceAge;
            internal float SourceMinClearance, ControlMinClearance;
            internal string Issue;
            internal EntryReferenceGeometry ReferencePath;
            internal FootGroundReference.GroundSample LandingGoal;
            internal FootLandingReferenceEvidence.LandingPath LandingPath;
            internal PelvisNormalResponseEvidence ReceiverNormalResponse;
            internal FootContactMotion Motion => new FootContactMotion
            {
                Valid = Valid && MotionValid, FullFootprint = FullFootprint,
                Min = MinClearance, PredictedMin = PredictedMinClearance, CornerNormalSpeed = MaxCornerNormalSpeed,
                PlanarSpeed = PlanarSpeed, AngularSpeed = AngularSpeed, AnchorGap = AnchorGap, SourceAge = SourceAge
            };
        }

        internal struct EntryContext
        {
            internal float SourceAnkleY;
            internal string SourcePhase;
            internal bool SourceFresh;
            internal int SourceHeldDepth;
            internal float SourceDt;
            internal int FixedSequence;
            internal bool FootAnchorYValid, CurrentGait;
            internal float FootAnchorY;
            internal bool MotionValid, ReferencesValid;
            internal Vector3 Center, Velocity, AngularVelocity, SourceOrigin, ControlOrigin;
            internal Quaternion SourceRotation, ControlRotation;
            internal float AnchorGap, SourceAge;
            internal bool DecompositionValid;
            internal Vector3 RawAtLowerOrigin, RawAtLowerRotationOrigin;
            internal Quaternion RawAtLowerRotation;
            internal FootGroundReference.GroundSample LandingGoal;
            internal FootLandingReferenceEvidence.LandingPath LandingPath;
            internal PelvisNormalResponseEvidence ReceiverNormalResponse;
        }

        // Read actual COM kinematics once; the same eight corners supply motion and signed clearance.
        // The existing control pose also scores precontact expiry. Formal stance uses actual geometry.
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
                Vector3 scaledAnchor = Vector3.Scale(footAnchorLocal, scale);
                Vector3 sourceAnchorOffset = sourceRotation * scaledAnchor;
                c.SourceAnkleY = sourceOrigin.y + sourceAnchorOffset.y;
                c.ControlOrigin = lowerAnkleWorld - controlRotation * scaledAnchor;
                c.ReferencesValid = FiniteVector(scale) && scale.x > 0f && scale.y > 0f && scale.z > 0f &&
                    FiniteVector(sourceOrigin) && FiniteVector(c.ControlOrigin) && FiniteRotation(sourceRotation) && FiniteRotation(controlRotation);
                CaptureEntryReferenceContext(ref c, lowerAnkleWorld, sourceAnchorOffset, scaledAnchor, sourceRelativeRotation, physicalLowerRotation);
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
            EntryGeometry rejected = EntryGeometryContext(context);
            try
            {
                if (foot != null && support != null && support.TryGetEntryBox(identity, out Vector3 center, out Vector3 half))
                {
                    var geometry = CaptureEntryGeometryAt(foot, center, half, context, dt);
                    geometry.SupportId = identity;
                    return geometry;
                }
                return rejected;
            }
            catch (Exception e) { rejected.Issue = "ENTRY_GEOMETRY_READ:" + e.GetType().Name; return rejected; }
        }

        private static EntryGeometry EntryGeometryContext(EntryContext context) => new EntryGeometry
        {
            Issue = "CONTROLLED_BOX_REQUIRED", SourceAnkleY = context.SourceAnkleY, LandingGoal = context.LandingGoal, LandingPath = context.LandingPath,
            SourcePhase = context.SourcePhase, SourceFresh = context.SourceFresh,
            ReceiverNormalResponse = context.ReceiverNormalResponse,
            SourceHeldDepth = context.SourceHeldDepth, SourceDt = context.SourceDt
        };

        // Shared eight-corner readback. Startup supplies the pinned original-floor Box
        // preview; runtime still requires TryGetEntryBox above and owns all contact authority.
        internal static EntryGeometry CaptureEntryGeometryAt(BoxCollider foot, Vector3 center, Vector3 half,
            EntryContext context, float dt)
        {
            EntryGeometry g = EntryGeometryContext(context);
            try
            {
                if (foot == null || !FiniteVector(center) || !FiniteVector(half) || half.x <= 0f || half.y <= 0f || half.z <= 0f) return g;
                g.FloorCenter = center; g.FloorHalf = half;
                g.LandingPath.SupportTop = center.y + half.y;
                g.LandingPath.Valid &= IsFinite(g.LandingPath.SupportTop);
                Vector3 size = foot.size, localCenter = foot.center;
                if (!FiniteVector(size) || !FiniteVector(localCenter) || size.x <= 0f || size.y <= 0f || size.z <= 0f)
                { g.Issue = "FOOT_BOX_INVALID"; return g; }
                Transform transform = foot.transform;
                Vector3 scale = transform.lossyScale;
                if (!FiniteVector(scale) || scale.x <= 0f || scale.y <= 0f || scale.z <= 0f)
                { g.Issue = "FOOT_SCALE_INVALID"; return g; }
                // Retain the values already read for all eight PRE_N corners.
                g.LocalBoxCenter = localCenter; g.LocalBoxSize = size; g.BoxScale = scale;
                float min = float.PositiveInfinity;
                float predicted = float.PositiveInfinity, sourceMin = float.PositiveInfinity, controlMin = float.PositiveInfinity;
                float maxNormalSpeed = 0f;
                FootLowestCornerMotion lowestMotion = default;
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
                                lowestMotion.Add(p.y - (center.y + half.y), v.y);
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
                g.LowestMotionValid = motion && lowestMotion.Valid;
                g.LowestCornerNormalVelocity = lowestMotion.Velocity;
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
                _depthLatchFixed = fixedSequence;
            }
            if (confirmedStance && _entryWindowMissed && !_entryMissCounted)
            {
                _entryMissCounted = true;
                _entryMissedStances++;
                FirstMissThisStep = true;
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
            bool ready = confirmedStance && _entryMissCounted && legacyGeometryReady && g.Motion.QuietIssue == null;
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
            // The native pair was enabled or handed over in place before this formal event.
            if (_entryRecoveryPending && _entryMissCounted)
            {
                _entryRecoveredStances++;
                RecoveryEnabledThisStep = true;
            }
            else OrdinaryEnabledThisStep = true;
        }

        private void CopyEntryToReport(FootAnkleFoundationReport report)
        {
            report.ContactEntryEvidenceReady = !_entryTraceContinuous || (_finite && _entryMissedStances == _entryRecoveredStances &&
                _entryGeometryInvalidSamples == 0 && _entryEvaluatedActivations == _contactActivations && _contactActivations > 0);
            report.PublishEntrySummary(new FootContactEntrySnapshot
            {
                TracePolicy = EntryPolicyName(_entryTraceContinuous, _entryTraceRecovery), LivePolicy = EntryPolicy,
                Ready = report.ContactEntryEvidenceReady, Fixed = _entryGeometryFixed, AtEnable = _entryGeometryAtEnable,
                Geometry = GeometrySummary(_entryGeometry), Missed = _entryMissedStances, Recovered = _entryRecoveredStances,
                HoldSeconds = _entryRecoverySeconds, RecoveryReject = _entryRecoveryRejectSamples,
                HistoryCount = _entryHistoryLogs, HistoryMotion = _entryHistoryMotionLogged,
                GeometryReject = _entryGeometryRejectSamples, GeometryInvalid = _entryGeometryInvalidSamples,
                Evaluated = _entryEvaluatedActivations, Peak = _entryPenetrationPeak, EnabledPeak = _entryEnabledPenetrationPeak,
                IdentityHold = _entryTraceContinuous ? ContinuousIdentityHoldSeconds : MinQualifiedStanceHoldSeconds
            }, LandingObservationTiming.Run.Compact);
        }

        private static void LogEvent(string text)
        {
            try { Plugin.Logger?.LogInfo(text); }
            catch (Exception) { } // A logger failure must not turn a completed native toggle into a failed control step.
        }
    }
}
