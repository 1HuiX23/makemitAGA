using System;
using System.Globalization;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04290 reads the same authorized Ground object across N -> N+1, even when the next sole probe
    /// has NO_HIT. The early settled tap precedes native-gravity clearing and contact lifecycle writes.
    /// These are discrete snapshots, not proof that an entity was continuously active between samples.
    /// No controller, production finite flag, contact authority or safety gate consumes these data.
    /// V04350 retains raw V04330 impulses, compares a damping model and audits actual unloaded steps.
    /// An endpoint position/velocity mismatch is descriptive, not a measured PhysX positional correction.
    /// </summary>
    internal sealed class FootSupportEntitySolveBoundaryObserver
    {
        internal const string Mode = "READ_ONLY_SUPPORT_ENTITY_SOLVE_BOUNDARY_V04290";
        internal const int MinimumResolvedSolves = 50;
        private const int Capacity = 12;
        private const int MaximumAncestors = 24;
        private const int MaximumOffsetGroundBaselines = 16;
        internal const string MotionMode = "READ_ONLY_RAW_DRAG_MODEL_RELEASE_WINDOWS_V04350";
        internal const string NormalWindowMode = "READ_ONLY_RAW_AND_DRAG_MODEL_RELEASE_WINDOW_ACCOUNTING_V04350";
        internal const string CallbackMomentumMode = "READ_ONLY_CALLBACK_MOMENTUM_AND_ANGULAR_MODEL_V04370";
        private const int MotionPreSolves = 2, MotionPostSolves = 2, MotionWindowCapacity = 5;
        private const int MaximumMotionWindows = 8, MotionCooldownSolves = 12;
        private const float MotionAngularDeltaMin = 5f, MotionRotationMinDegrees = 3f, MotionComRiseMinMeters = 0.003f;
        private const float NormalResidualCandidateMinNs = 0.1f, NativeJoinTolerance = 0.00001f;

        private sealed class GroundIdentity
        {
            internal Collider Collider;
            internal Transform[] Ancestors;
            internal int Id;
            internal string Path;
            internal bool Truncated;
        }

        private struct Entity
        {
            internal bool Exists, Enabled, ActiveSelf, ActiveHierarchy, Trigger, AncestorLinksMatch, AncestorsObserved;
            internal int Id, Layer, ParentId;
            internal ulong AncestorAlive, AncestorSelf, AncestorHierarchy;
            internal Vector3 Position, Scale, BoundsCenter, BoundsExtents;
            internal Quaternion Rotation;
            internal bool ContactOffsetRead;
            internal float ContactOffset;
        }

        private struct FootState
        {
            internal IntPtr BodyPointer;
            internal Entity Collider;
            internal bool BodyExists, Gravity, Kinematic, DetectCollisionsRead, DetectCollisions;
            internal bool PairFilterRead, PairIgnored, LayerFilterRead, LayerIgnored, ReverseLayerFilterRead, ReverseLayerIgnored;
            internal int BodyId, CollisionMode, SolverVelocity, SolverPosition;
            internal Vector3 Com, Velocity, W;
        }

        private struct Probe
        {
            internal int Id, Hits;
            internal bool Ready, SameSurface;
            internal bool NormalValid;
            internal float MaxAbsClearance, Spread, CenterClearance, MinClearance, MaxClearance;
            internal Vector3 Normal;
        }

        private struct GroundOffsetBaseline
        {
            internal int Id;
            internal float Offset;
        }

        private enum ContactOffsetEvidence
        {
            NOT_APPLICABLE, MATCHED, PARAMETER_MISMATCH, ENTITY_CHANGED_WITHIN_SOLVE, READBACK_UNAVAILABLE
        }

        private enum ContactOffsetEnvironmentEvidence
        {
            NOT_APPLICABLE, STABLE_AT_CAPTURED_BOUNDARIES, CHANGED_WITHIN_SOLVE,
            SUPPORT_LOST_WITHIN_SOLVE, SUPPORT_UNAVAILABLE_AT_BOUNDARY
        }

        private struct Frame
        {
            internal int Fixed, Settled, Render, Held, Enter, Stay, Exit, Projected, ProjectionInvalid;
            internal int FirstCallbackAuthorizedId, LastCallbackAuthorizedId, SettledAuthorizedId;
            internal bool Fresh, Loaded, Resolved, Censored, PreContact, SettledContact, ProbeRecorded, PositionSolverB;
            internal int BaselineSolverVelocity;
            internal bool CcdExperimentSelected;
            internal int ExpectedCollisionMode;
            internal string CollisionArm;
            internal bool ContactOffsetSelected, OffsetGroundBaselineKnown, OffsetGroundSwitched;
            internal string ContactOffsetArm;
            internal float ExpectedContactOffset, ExpectedGroundContactOffset;
            internal float Age, Dt;
            internal GroundIdentity Ground;
            internal Entity PreGround, FirstCallbackGround, LastCallbackGround, SettledGround;
            internal FootState PreFoot, FirstCallbackFoot, LastCallbackFoot, SettledFoot;
            internal Probe PreProbe, SettledProbe;
            internal Vector3 GroundJ, FirstReactionJ, LastReactionJ, EnterJ, StayJ;
            // Prefixes refer to the chronological first/last callback snapshots, including EXIT.
            // FirstReactionJ instead refers to the first projected ENTER/STAY and can be a different callback.
            internal Vector3 FirstCallbackGroundPrefixJ, LastCallbackGroundPrefixJ;
            internal string FirstCallbackPhase, LastCallbackPhase;
            internal float ReactionMagnitudeSum;
            internal int CensusTotal, CensusOther, CensusRejected;
            internal bool SourcePreStepValid, SourceSettledRecorded, SourceSettledStepValid, PreAnchorValid, SettledAnchorValid;
            internal int SourceSettledRender;
            internal bool SourceSettledFresh;
            internal int SourceSettledHeld;
            internal float SourceSettledAge;
            internal string PreGait, SettledGaitBeforeLifecycle;
            internal float PreAnchor, SettledAnchor;
            internal Vector3 SourcePreStepDegrees, SourceSettledStepDegrees;
            internal bool MotionCandidate, NormalRecorded, NormalJoined, NormalCandidate;
            internal FootContactRockingTetherCouplingObserver.NormalLoad Normal;
            internal bool AngularRecorded, AngularJoined;
            internal string AngularJoinReason;
            internal FootContactAngularImpulseAttributionObserver.AngularSolve Angular;
        }

        private struct Motion
        {
            internal bool NormalValid;
            internal Vector3 ComDelta, ProxyDelta, DeltaW, PositionVelocityMismatch;
            internal float AngularDelta, RotationDegrees, ComNormalDelta, ProxyNormalDelta, ComNormalVelocityDelta;
        }

        // Partial windows contain explicit gaps. Only contiguous, mass/owner/velocity-consistent
        // audited segments telescope; inputs are world vectors projected onto one window normal.
        private struct WindowLedger
        {
            internal int Count, Segments, Unloaded, Unresolved, MissingLoaded, BoundaryBreaks, LoadedAudited, UnloadedAudited, MissingUnloaded, ModelCount, ModelMissing;
            internal Vector3 LowerDrag, FootDrag;
            internal Vector3 LowerKnown, FootKnown, Ground, LowerObserved, FootObserved;
            private Vector3 _lowerClosedEndpoint, _footClosedEndpoint;
            private FootContactRockingTetherCouplingObserver.NormalLoad _first, _last;
            private int _footId;
            private bool _segment;
            private void CloseSegment()
            {
                if (!_segment) return;
                _lowerClosedEndpoint += _first.LowerMass * (_last.SettledLowerV - _first.PreLowerV);
                _footClosedEndpoint += _first.FootMass * (_last.SettledFootV - _first.PreFootV);
                _segment = false;
            }
            internal void Add(Frame f)
            {
                if (!f.Loaded) Unloaded++;
                if (!f.NormalJoined)
                {
                    CloseSegment();
                    if (!f.Resolved || f.Censored) Unresolved++;
                    else if (!f.Loaded) MissingUnloaded++;
                    else MissingLoaded++;
                    return;
                }
                var d = f.Normal;
                bool contiguous = _segment && d.PreFixed == _last.SettledFixed && _footId == f.PreFoot.BodyId &&
                    Mathf.Abs(d.LowerMass - _first.LowerMass) <= 0.0001f && Mathf.Abs(d.FootMass - _first.FootMass) <= 0.0001f &&
                    (d.PreLowerV - _last.SettledLowerV).magnitude <= NativeJoinTolerance &&
                    (d.PreFootV - _last.SettledFootV).magnitude <= NativeJoinTolerance;
                if (_segment && !contiguous) { BoundaryBreaks++; CloseSegment(); }
                if (!_segment) { _first = d; _footId = f.PreFoot.BodyId; _segment = true; Segments++; }
                _last = d; Count++;
                if (f.Loaded) LoadedAudited++; else UnloadedAudited++;
                if (d.DragModelValid) { LowerDrag += d.LowerDragWorld; FootDrag += d.FootDragWorld; ModelCount++; }
                else ModelMissing++;
                LowerKnown += d.LowerKnownWorld;
                FootKnown += d.FootCommandWorld + d.GroundWorld;
                Ground += d.GroundWorld;
                LowerObserved += d.LowerDeltaPWorld;
                FootObserved += d.FootDeltaPWorld;
            }
            internal string Format(Vector3 n)
            {
                string scope = "audited-segments-unloaded-unresolved-missingLoaded-boundaryBreaks=" + Count + "-" + Segments + "-" +
                    Unloaded + "-" + Unresolved + "-" + MissingLoaded + "-" + BoundaryBreaks +
                    "/loadedAudited-unloadedAudited-missingUnloaded=" + LoadedAudited + "-" + UnloadedAudited + "-" + MissingUnloaded +
                    "/dragModelValid-missing=" + ModelCount + "-" + ModelMissing;
                if (Count == 0) return scope + "/Ns=NA_NO_AUDITED_SOLVE";
                if (!V(n) || Mathf.Abs(n.sqrMagnitude - 1f) > NativeJoinTolerance) return scope + "/Ns=NA_NO_REFERENCE_NORMAL";
                Vector3 lowerEndpoint = _lowerClosedEndpoint + (_segment ? _first.LowerMass * (_last.SettledLowerV - _first.PreLowerV) : Vector3.zero);
                Vector3 footEndpoint = _footClosedEndpoint + (_segment ? _first.FootMass * (_last.SettledFootV - _first.PreFootV) : Vector3.zero);
                if (!V(LowerKnown) || !V(FootKnown) || !V(Ground) || !V(LowerObserved) || !V(FootObserved) || !V(lowerEndpoint) || !V(footEndpoint))
                    throw new InvalidOperationException("CUMULATIVE_LEDGER_NONFINITE");
                float lowerR = Vector3.Dot(LowerObserved - LowerKnown, n), footR = Vector3.Dot(FootObserved - FootKnown, n);
                return scope + "/fixedNormal=" + W(n) + "/Ns:lowerKnown=" + S(Vector3.Dot(LowerKnown, n)) +
                    "/footKnownIncludingGround=" + S(Vector3.Dot(FootKnown, n)) + "/groundCallback=" + S(Vector3.Dot(Ground, n)) +
                    "/lowerDeltaP=" + S(Vector3.Dot(LowerObserved, n)) + "/footDeltaP=" + S(Vector3.Dot(FootObserved, n)) +
                    "/lowerResidual=" + S(lowerR) + "/footResidual=" + S(footR) + "/pairResidual=" + S(lowerR + footR) +
                    "/segmentEndpointDeltaP(lower-foot)=" + S(Vector3.Dot(lowerEndpoint, n)) + "-" + S(Vector3.Dot(footEndpoint, n)) +
                    "/sumMinusSegmentEndpoint(lower-foot)=" + S(Vector3.Dot(LowerObserved - lowerEndpoint, n)) + "-" + S(Vector3.Dot(FootObserved - footEndpoint, n)) +
                    "/dragModelJ(lower-foot)=" + (ModelMissing == 0 ? S(Vector3.Dot(LowerDrag, n)) + "-" + S(Vector3.Dot(FootDrag, n)) : "NA_INCOMPLETE_MODEL") +
                    "/residualAfterDragModel(lower-foot-pair)=" + (ModelMissing == 0 ?
                        S(lowerR - Vector3.Dot(LowerDrag, n)) + "-" + S(footR - Vector3.Dot(FootDrag, n)) + "-" +
                        S(lowerR + footR - Vector3.Dot(LowerDrag + FootDrag, n)) : "NA_INCOMPLETE_MODEL");
            }
        }

        private readonly Frame[] _ring = new Frame[Capacity];
        private GroundIdentity _identity;
        private int _next, _count, _pending = -1, _captured, _allResolved, _loadedResolved, _censored;
        private int _loadedNoReactionCallback, _loadedProbeNoHit, _loadedEntityChanged, _exitCallbacks, _unjoinedCallbacks;
        private int _layerReadReactionContradictions, _positionSolverResolved;
        private int _ccdResolved, _ccdMismatch;
        private int _offsetResolved, _offsetMismatch, _offsetBoundaryChanged, _offsetReadUnavailable;
        private int _offsetEnvironmentExcluded, _offsetSupportLost;
        private int _offsetGroundCount, _offsetGroundSwitches, _lastOffsetGroundId;
        private bool _offsetGroundCapacityReached;
        // Capture each Ground ID once per trace. No eviction or recapture on return: that would hide
        // a real parameter change. A full cache leaves new Grounds unverified without changing physics.
        private readonly GroundOffsetBaseline[] _offsetGroundBaselines = new GroundOffsetBaseline[MaximumOffsetGroundBaselines];
        private bool _finite = true;
        private string _invalidReason;
        // Copies survive ring wrap. Only one five-solve window is pending; V04390 reuses it and adds only contactOffset scalars to existing reads.
        private readonly Frame[] _motionWindow = new Frame[MotionWindowCapacity];
        // V04430: numeric copies only during simulation. At most the existing 8 x 5 samples;
        // no growing string queue and no additional native reads. Formatting is report-time only.
        private readonly Frame[] _sealedMotionFrames = new Frame[MaximumMotionWindows * MotionWindowCapacity];
        private readonly int[] _sealedCounts = new int[MaximumMotionWindows];
        private readonly int[] _sealedPreCounts = new int[MaximumMotionWindows];
        private readonly int[] _sealedPostRemaining = new int[MaximumMotionWindows];
        private readonly int[] _sealedTriggers = new int[MaximumMotionWindows];
        private readonly string[] _sealedReasons = new string[MaximumMotionWindows];
        private int _motionCount, _motionPreCount, _motionPostRemaining, _motionTriggerFixed;
        private int _motionEligible, _motionCandidates, _motionStarted, _motionFlushed, _motionIncomplete;
        private int _motionProbeIncomplete;
        private int _motionCovered, _motionCooldownSuppressed, _motionLimitSuppressed;
        private bool _motionActive, _motionFinite = true;
        private string _motionInvalidReason;
        private float _motionPeakAngularDelta, _motionPeakRotationDegrees, _motionPeakComRiseMeters;
        private bool _normalWindowFinite = true;
        private string _normalWindowInvalidReason;
        private int _normalJoined, _normalMissing, _normalMismatch, _normalCandidates, _normalOnlyCandidates, _motionCriteriaCandidates;
        private float _normalPeakLowerResidual, _normalPeakFootResidual, _normalPeakPairResidual;
        private int _unloadedJoined, _unloadedMissing, _unloadedMismatch;
        private int _callbackCensusTotal, _callbackCensusUnjoined, _callbackCensusOther, _callbackCensusFailed;
        private int _dragModelJoined, _dragModelMissing;
        private bool _callbackAuditFinite = true;
        private string _callbackAuditInvalidReason;
        private int _angularLoadedJoined, _angularLoadedMissing, _angularLoadedMismatch;
        private int _angularModelInputsValid, _angularModelInputsUnavailable;

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            Array.Clear(_ring, 0, _ring.Length);
            _next = _count = _captured = _allResolved = _loadedResolved = _censored = 0;
            _loadedNoReactionCallback = _loadedProbeNoHit = _loadedEntityChanged = _exitCallbacks = _unjoinedCallbacks = 0;
            _pending = -1;
            _layerReadReactionContradictions = _positionSolverResolved = 0;
            _ccdResolved = _ccdMismatch = 0;
            _offsetResolved = _offsetMismatch = _offsetBoundaryChanged = _offsetReadUnavailable = 0;
            _offsetEnvironmentExcluded = _offsetSupportLost = 0;
            _offsetGroundCount = _offsetGroundSwitches = _lastOffsetGroundId = 0;
            _offsetGroundCapacityReached = false;
            Array.Clear(_offsetGroundBaselines, 0, _offsetGroundBaselines.Length);
            _identity = null;
            _finite = true;
            _invalidReason = null;
            Array.Clear(_motionWindow, 0, _motionWindow.Length);
            Array.Clear(_sealedMotionFrames, 0, _sealedMotionFrames.Length);
            Array.Clear(_sealedCounts, 0, _sealedCounts.Length);
            Array.Clear(_sealedPreCounts, 0, _sealedPreCounts.Length);
            Array.Clear(_sealedPostRemaining, 0, _sealedPostRemaining.Length);
            Array.Clear(_sealedTriggers, 0, _sealedTriggers.Length);
            Array.Clear(_sealedReasons, 0, _sealedReasons.Length);
            _motionCount = _motionPreCount = _motionPostRemaining = _motionTriggerFixed = 0;
            _motionEligible = _motionCandidates = _motionStarted = _motionFlushed = _motionIncomplete = 0;
            _motionProbeIncomplete = 0;
            _motionCovered = _motionCooldownSuppressed = _motionLimitSuppressed = 0;
            _motionActive = false;
            _motionFinite = true;
            _motionInvalidReason = null;
            _motionPeakAngularDelta = _motionPeakRotationDegrees = _motionPeakComRiseMeters = 0f;
            _normalWindowFinite = true; _normalWindowInvalidReason = null;
            _normalJoined = _normalMissing = _normalMismatch = _normalCandidates = _normalOnlyCandidates = _motionCriteriaCandidates = 0;
            _normalPeakLowerResidual = _normalPeakFootResidual = _normalPeakPairResidual = 0f;
            _unloadedJoined = _unloadedMissing = _unloadedMismatch = 0;
            _callbackCensusTotal = _callbackCensusUnjoined = _callbackCensusOther = _callbackCensusFailed = 0;
            _dragModelJoined = _dragModelMissing = 0;
            _callbackAuditFinite = true; _callbackAuditInvalidReason = null;
            _angularLoadedJoined = _angularLoadedMissing = _angularLoadedMismatch = 0;
            _angularModelInputsValid = _angularModelInputsUnavailable = 0;
            CopyToReport(report);
        }

        internal void CapturePreSolve(bool trace, int fixedSequence, bool loaded, bool contact,
            Collider authorizedGround, Collider footCollider, Rigidbody foot, FootAnkleFoundationReport report)
        {
            if (!trace) return;
            try
            {
                if (_pending >= 0 && _ring[_pending].Fixed == fixedSequence) return;
                if (_pending >= 0)
                {
                    Invalid("PENDING_SOLVE_NOT_SETTLED");
                    ClearPendingPreserveTrace(report);
                }
                GroundIdentity identity = GetIdentity(authorizedGround);
                bool collisionModeKnown = FootCollisionDetectionModeIsolation.TryGetMode(report.FootCollisionDetectionModeArm,
                    out CollisionDetectionMode selectedCollisionMode);
                Frame f = new Frame
                {
                    Fixed = fixedSequence, Render = report.TargetRenderSampleSequence,
                    Fresh = report.TargetRenderSampleFreshAtFixedStep, Held = report.FixedStepsSinceTargetRenderSample,
                    Age = report.TargetRenderSampleAgeSeconds, Dt = Time.fixedDeltaTime,
                    Ground = identity, Loaded = loaded, PreContact = contact,
                    PositionSolverB = report.FootPositionSolverEnabled && report.FootPositionSolverArm == FootSolverIterationBaselineContract.PositionArmB,
                    BaselineSolverVelocity = report.FootVelocitySolverBaselineVelocity,
                    CcdExperimentSelected = collisionModeKnown && FootCollisionDetectionModeIsolation.IsActiveArm(report.FootCollisionDetectionModeArm),
                    ExpectedCollisionMode = (int)selectedCollisionMode, CollisionArm = report.FootCollisionDetectionModeArm,
                    ContactOffsetSelected = report.FootContactOffsetInitialized && report.FootContactOffsetFinite &&
                        FootContactOffsetIsolation.IsExperimentArm(report.FootContactOffsetArm),
                    ContactOffsetArm = report.FootContactOffsetArm,
                    ExpectedContactOffset = report.FootContactOffsetSelectedMeters,
                    PreGround = ReadGround(identity), PreFoot = ReadFoot(footCollider, foot, identity.Collider),
                    PreProbe = ReadProbe(report),
                    SourcePreStepValid = report.SourceAnkleRenderStepValid,
                    SourcePreStepDegrees = report.SourceAnkleRenderStepDegrees,
                    PreGait = report.GameNativeGaitPhaseState,
                    PreAnchorValid = report.CurrentObservedStep.Valid && report.CurrentObservedStep.FixedSequence == fixedSequence,
                    PreAnchor = report.CurrentObservedStep.AnchorErrorMeters
                };
                PrepareContactOffsetGroundBaseline(ref f);
                _ring[_next] = f;
                _pending = _next;
                _next = (_next + 1) % Capacity;
                _count = Math.Min(Capacity, _count + 1);
                _captured++;
            }
            catch (Exception e) { Invalid("PRE_SOLVE_READ_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        private void PrepareContactOffsetGroundBaseline(ref Frame f)
        {
            if (!f.ContactOffsetSelected || !f.Loaded || !f.PreGround.Exists || f.PreGround.Id == 0) return;
            f.OffsetGroundSwitched = _lastOffsetGroundId != 0 && _lastOffsetGroundId != f.PreGround.Id;
            if (f.OffsetGroundSwitched) _offsetGroundSwitches++;
            _lastOffsetGroundId = f.PreGround.Id;
            for (int i = 0; i < _offsetGroundCount; i++)
            {
                if (_offsetGroundBaselines[i].Id != f.PreGround.Id) continue;
                f.ExpectedGroundContactOffset = _offsetGroundBaselines[i].Offset;
                f.OffsetGroundBaselineKnown = true;
                return;
            }
            if (!f.PreGround.ContactOffsetRead || !F(f.PreGround.ContactOffset) || f.PreGround.ContactOffset <= 0f) return;
            if (_offsetGroundCount >= _offsetGroundBaselines.Length)
            {
                _offsetGroundCapacityReached = true;
                return;
            }
            _offsetGroundBaselines[_offsetGroundCount++] = new GroundOffsetBaseline
            {
                Id = f.PreGround.Id, Offset = f.PreGround.ContactOffset
            };
            f.ExpectedGroundContactOffset = f.PreGround.ContactOffset;
            f.OffsetGroundBaselineKnown = true;
        }

        internal void ObserveAuthorizedCallback(bool trace, int preFixed, string phase, int authorizedId,
            bool projectionValid, Vector3 impulse, Collider footCollider, Rigidbody foot, FootAnkleFoundationReport report)
        {
            if (!trace) return;
            if (phase == "EXIT") _exitCallbacks++;
            if (_pending < 0 || _ring[_pending].Fixed != preFixed)
            {
                _unjoinedCallbacks++;
                CopyToReport(report, false);
                return;
            }
            try
            {
                Frame f = _ring[_pending];
                Entity groundState = ReadGround(f.Ground); // retain N's exact object; never use the new raycast identity
                FootState footState = ReadFoot(footCollider, foot, f.Ground.Collider);
                bool firstCallback = f.Enter + f.Stay + f.Exit == 0;
                if (firstCallback)
                {
                    f.FirstCallbackGround = groundState;
                    f.FirstCallbackFoot = footState;
                    f.FirstCallbackAuthorizedId = authorizedId;
                    f.FirstCallbackPhase = phase;
                }
                f.LastCallbackGround = groundState;
                f.LastCallbackFoot = footState;
                f.LastCallbackAuthorizedId = authorizedId;
                f.LastCallbackPhase = phase;
                if (phase == "EXIT") f.Exit++; // EXIT has no projected reaction impulse
                else if (phase == "ENTER" || phase == "STAY")
                {
                    if (phase == "ENTER") f.Enter++; else f.Stay++;
                    if (authorizedId == f.Ground.Id && projectionValid && V(impulse))
                    {
                        if (f.Projected == 0) f.FirstReactionJ = impulse;
                        f.LastReactionJ = impulse; f.ReactionMagnitudeSum += impulse.magnitude;
                        if (phase == "ENTER") f.EnterJ += impulse; else f.StayJ += impulse;
                        f.Projected++; f.GroundJ += impulse;
                    }
                    else
                    {
                        f.ProjectionInvalid++;
                        if (f.Loaded) Invalid(authorizedId != f.Ground.Id ? "LOADED_CALLBACK_GROUND_ID_MISMATCH" : "LOADED_ENTER_STAY_PROJECTION_INVALID");
                    }
                }
                else Invalid("UNKNOWN_AUTHORIZED_CALLBACK_PHASE");
                if (firstCallback) f.FirstCallbackGroundPrefixJ = f.GroundJ;
                f.LastCallbackGroundPrefixJ = f.GroundJ;
                _ring[_pending] = f;
            }
            catch (Exception e) { Invalid("CALLBACK_READ_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        internal void ObserveSettledBeforeControllerPreparation(bool trace, int fixedSequence, bool contact,
            int authorizedId, Collider footCollider, Rigidbody foot, FootAnkleFoundationReport report)
        {
            if (!trace || _pending < 0) return;
            try
            {
                Frame f = _ring[_pending];
                if (fixedSequence != f.Fixed + 1)
                {
                    Invalid("SETTLED_FIXED_SEQUENCE_MISMATCH");
                    ClearPendingPreserveTrace(report);
                    return;
                }
                f.Settled = fixedSequence;
                f.SettledGround = ReadGround(f.Ground);
                f.SettledFoot = ReadFoot(footCollider, foot, f.Ground.Collider);
                f.SettledContact = contact;
                f.SettledAuthorizedId = authorizedId;
                f.Resolved = true;
                _ring[_pending] = f;
                _pending = -1;
                _allResolved++;
                if (f.Loaded)
                {
                    _loadedResolved++;
                    bool entityChanged = EntityChanged(f);
                    if (PositionSolverInterventionResolved(f)) _positionSolverResolved++;
                    if (CcdModeInterventionResolved(f)) _ccdResolved++;
                    if (CcdModeReadbackMismatch(f)) _ccdMismatch++;
                    AccumulateContactOffsetEvidence(f, entityChanged);
                    if (f.Enter + f.Stay == 0) _loadedNoReactionCallback++;
                    if (entityChanged) _loadedEntityChanged++;
                    if (LayerReadReactionContradiction(f)) _layerReadReactionContradictions++;
                }
                ObserveMotionWindow(f, report);
            }
            catch (Exception e)
            {
                Invalid("EARLY_SETTLED_READ_EXCEPTION:" + e.GetType().Name);
                ClearPendingPreserveTrace(report);
            }
            CopyToReport(report, false);
        }

        internal void RecordSettledProbe(bool trace, int fixedSequence, FootAnkleFoundationReport report)
        {
            if (!trace || _count == 0) return;
            try
            {
                int i = (_next - 1 + Capacity) % Capacity;
                Frame f = _ring[i];
                if (!f.Resolved || f.Settled != fixedSequence || f.ProbeRecorded) return;
                f.SettledProbe = ReadProbe(report);
                f.ProbeRecorded = true;
                f.SourceSettledRecorded = report.FixedUpdateSequence == fixedSequence;
                f.SourceSettledRender = report.TargetRenderSampleSequence;
                f.SourceSettledFresh = report.TargetRenderSampleFreshAtFixedStep;
                f.SourceSettledHeld = report.FixedStepsSinceTargetRenderSample;
                f.SourceSettledAge = report.TargetRenderSampleAgeSeconds;
                f.SourceSettledStepValid = f.SourceSettledRecorded && report.SourceAnkleRenderStepValid;
                f.SourceSettledStepDegrees = report.SourceAnkleRenderStepDegrees;
                // Gait still describes the previous lifecycle here. It is not the next contact decision.
                f.SettledGaitBeforeLifecycle = report.GameNativeGaitPhaseState;
                f.SettledAnchorValid = report.CurrentObservedStep.Valid && report.CurrentObservedStep.FixedSequence == fixedSequence;
                f.SettledAnchor = report.CurrentObservedStep.AnchorErrorMeters;
                _ring[i] = f;
                UpdateMotionWindowProbe(f, report);
                if (f.Loaded && f.SettledProbe.Hits == 0) _loadedProbeNoHit++;
                // Per-step long dumps retired V04430; ring, peak/counter and sealed windows retain evidence.
            }
            catch (Exception e) { Invalid("SETTLED_PROBE_READ_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        internal void RecordSettledNormalLoad(bool trace, int fixedSequence,
            FootContactRockingTetherCouplingObserver.NormalLoad sample, FootAnkleFoundationReport report)
        {
            if (!trace || _count == 0) return;
            try
            {
                int i = (_next - 1 + Capacity) % Capacity;
                Frame f = _ring[i];
                if (!f.Resolved || f.Settled != fixedSequence || f.NormalRecorded) return;
                f.NormalRecorded = true; f.Normal = sample;
                if (!sample.Captured || !sample.Resolved)
                {
                    if (f.Loaded) _normalMissing++; else _unloadedMissing++;
                    NormalWindowInvalid(f.Loaded ? "LOADED_SOURCE_LEDGER_NOT_RESOLVED" : "UNLOADED_SOURCE_LEDGER_NOT_RESOLVED");
                }
                else if (!NormalSampleMatchesFrame(f))
                {
                    if (f.Loaded) _normalMismatch++; else _unloadedMismatch++;
                    NormalWindowInvalid("LEDGER_NATIVE_CLOCK_OR_ENDPOINT_MISMATCH");
                }
                else
                {
                    f.NormalJoined = true;
                    if (sample.DragModelValid) _dragModelJoined++; else _dragModelMissing++;
                    if (!f.Loaded) _unloadedJoined++;
                    else
                    {
                        _normalJoined++; // old qualification still counts loaded solves only
                        _normalPeakLowerResidual = Mathf.Max(_normalPeakLowerResidual, Mathf.Abs(sample.LowerResidual));
                        _normalPeakFootResidual = Mathf.Max(_normalPeakFootResidual, Mathf.Abs(sample.FootResidual));
                        _normalPeakPairResidual = Mathf.Max(_normalPeakPairResidual, Mathf.Abs(sample.PairResidual));
                        f.NormalCandidate = NormalResidualThresholdExceeded(sample); // raw, unchanged threshold/domain
                        if (f.NormalCandidate)
                        {
                            _normalCandidates++;
                            if (!f.MotionCandidate)
                            { _normalOnlyCandidates++; _motionCandidates++; SelectMotionWindow(f); }
                        }
                    }
                }
                _ring[i] = f;
                UpdateMotionWindowFrame(f); // late probe owns completion, never append/decrement a second time
            }
            catch (Exception e) { NormalWindowInvalid("JOIN_EXCEPTION:" + e.GetType().Name); }
            CopyToReport(report, false);
        }

        internal void RecordSettledAngular(bool trace, int fixedSequence,
            FootContactAngularImpulseAttributionObserver.AngularSolve sample, FootAnkleFoundationReport report)
        {
            if (!trace || _count == 0) return;
            try
            {
                int i = (_next - 1 + Capacity) % Capacity;
                Frame f = _ring[i];
                if (!f.Resolved || f.Settled != fixedSequence || f.AngularRecorded) return;
                f.AngularRecorded = true; f.Angular = sample;
                f.AngularJoined = AngularSampleMatchesFrame(f);
                f.AngularJoinReason = f.AngularJoined ? "JOINED" :
                    (!sample.Resolved ? "NO_RESOLVED_AUTHORIZED_ANGULAR_SAMPLE" : "CLOCK_ENDPOINT_OR_CALLBACK_MISMATCH");
                if (f.Loaded)
                {
                    if (f.AngularJoined)
                    {
                        _angularLoadedJoined++;
                        if (AngularModelInputsValid(sample)) _angularModelInputsValid++;
                        else _angularModelInputsUnavailable++;
                    }
                    else if (!sample.Resolved) _angularLoadedMissing++;
                    else _angularLoadedMismatch++;
                }
                _ring[i] = f;
                UpdateMotionWindowFrame(f);
            }
            catch (Exception e)
            {
                _callbackAuditFinite = false;
                if (_callbackAuditInvalidReason == null) _callbackAuditInvalidReason = "ANGULAR_JOIN_EXCEPTION:" + e.GetType().Name;
            }
            // This diagnostic domain is deliberately absent from controller/safety/data-qualification decisions.
            CopyToReport(report, false);
        }

        private static bool AngularSampleMatchesFrame(Frame f)
        {
            var a = f.Angular;
            return NativePairEligible(f) && a.Resolved && a.PreFixed == f.Fixed && a.SettledFixed == f.Settled &&
                a.PreBodyPointer != IntPtr.Zero && a.PreBodyPointer == f.PreFoot.BodyPointer &&
                a.SettledBodyPointer == f.SettledFoot.BodyPointer && a.PreBodyPointer == a.SettledBodyPointer &&
                F(a.Dt) && Mathf.Abs(a.Dt - f.Dt) <= 0.0000001f && F(a.BoundsRadius) && a.BoundsRadius >= 0f &&
                V(a.PreW) && V(a.SettledW) && V(a.PreL) && V(a.SettledL) &&
                V(a.TransportJ) && V(a.PdJ) && V(a.SupportJ) && V(a.ArlJ) && V(a.GroundJ) &&
                (a.PreW - f.PreFoot.W).magnitude <= NativeJoinTolerance &&
                (a.SettledW - f.SettledFoot.W).magnitude <= NativeJoinTolerance &&
                (a.GroundJ - f.GroundJ).magnitude <= NativeJoinTolerance &&
                (a.TransportJ + a.PdJ + a.SupportJ - a.ArlJ).magnitude <= NativeJoinTolerance &&
                a.ProjectedCallbacks == f.Projected && a.AllCallbacks == f.Enter + f.Stay + f.Exit;
        }

        private static bool NormalSampleMatchesFrame(Frame f)
        {
            var d = f.Normal;
            return NativePairEligible(f) && d.Captured && d.Resolved && FootContactRockingTetherCouplingObserver.NormalLedgerConsistent(d) &&
                d.Loaded == f.Loaded && d.PreFixed == f.Fixed && d.SettledFixed == f.Settled &&
                d.HipLinearFixed == f.Fixed && V(d.HipLinearWorld) &&
                d.UpperLoadFixed == f.Fixed && V(d.UpperLoadLowerKneeWorld) && V(d.UpperLoadLowerAnkleWorld) && V(d.UpperLoadFootWorld) &&
                F(d.Dt) && Mathf.Abs(d.Dt - f.Dt) <= 0.0000001f && F(d.LowerMass) && d.LowerMass > 0f && F(d.FootMass) && d.FootMass > 0f &&
                (!f.Loaded || (d.NormalValid && V(d.PreNormal) && Mathf.Abs(d.PreNormal.sqrMagnitude - 1f) <= NativeJoinTolerance)) &&
                V(d.PreLowerV) && V(d.SettledLowerV) && V(d.PreFootV) && V(d.SettledFootV) &&
                V(d.LowerKnownWorld) && V(d.FootCommandWorld) && V(d.GroundWorld) && V(d.LowerDeltaPWorld) && V(d.FootDeltaPWorld) &&
                F(d.LowerResidual) && F(d.FootResidual) && F(d.PairResidual) &&
                (d.PreFootV - f.PreFoot.Velocity).magnitude <= NativeJoinTolerance &&
                (d.SettledFootV - f.SettledFoot.Velocity).magnitude <= NativeJoinTolerance &&
                (d.GroundWorld - f.GroundJ).magnitude <= NativeJoinTolerance;
        }

        internal void ObserveCallbackCensus(bool trace, int preFixed, bool authorized, bool accepted)
        {
            if (!trace) return;
            // Census only: no second Collision/native-state read and no unauthorized J in Ground ledger.
            _callbackCensusTotal++;
            if (_pending < 0 || _ring[_pending].Fixed != preFixed) _callbackCensusUnjoined++;
            else
            {
                Frame f = _ring[_pending]; f.CensusTotal++;
                if (!authorized) f.CensusOther++;
                if (!accepted) f.CensusRejected++;
                _ring[_pending] = f;
            }
            if (!authorized) _callbackCensusOther++;
            if (!accepted) _callbackCensusFailed++;
        }

        private static bool NormalResidualThresholdExceeded(FootContactRockingTetherCouplingObserver.NormalLoad d) =>
            Mathf.Abs(d.LowerResidual) >= NormalResidualCandidateMinNs || Mathf.Abs(d.FootResidual) >= NormalResidualCandidateMinNs ||
            Mathf.Abs(d.PairResidual) >= NormalResidualCandidateMinNs;

        internal void ClearPendingPreserveTrace(FootAnkleFoundationReport report)
        {
            if (_pending >= 0)
            {
                Frame f = _ring[_pending];
                f.Censored = true;
                _ring[_pending] = f;
                if (_motionActive && _motionCount < MotionWindowCapacity &&
                    _motionCount > 0 && f.Fixed == _motionWindow[_motionCount - 1].Fixed + 1)
                    _motionWindow[_motionCount++] = f; // pre-only censored context never counts as a resolved post solve
                _pending = -1;
                _censored++;
            }
            FlushMotionWindow("PENDING_CLEARED_OR_TRACE_BOUNDARY", report);
            CopyToReport(report, false);
        }

        internal void EndTrace(FootAnkleFoundationReport report)
        {
            ClearPendingPreserveTrace(report);
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report, bool format = true)
        {
            if (report == null) return;
            report.SupportEntityBoundaryMode = Mode;
            report.SupportEntityBoundaryFinite = _finite;
            report.SupportEntityBoundaryInvalidReason = _invalidReason;
            report.SupportEntityBoundaryResolvedLoadedSolves = _loadedResolved;
            report.SupportEntityBoundaryPositionSolverResolvedSolves = _positionSolverResolved;
            report.SupportEntityBoundaryCcdResolvedSolves = _ccdResolved;
            report.SupportEntityBoundaryCcdMismatchSolves = _ccdMismatch;
            report.SupportEntityBoundaryContactOffsetResolvedSolves = _offsetResolved;
            report.SupportEntityBoundaryContactOffsetMismatchSolves = _offsetMismatch;
            report.SupportEntityBoundaryContactOffsetBoundaryChangedSolves = _offsetBoundaryChanged;
            report.SupportEntityBoundaryContactOffsetReadUnavailableSolves = _offsetReadUnavailable;
            report.SupportEntityBoundaryContactOffsetEnvironmentExcludedSolves = _offsetEnvironmentExcluded;
            report.SupportEntityBoundaryContactOffsetSupportLostSolves = _offsetSupportLost;
            report.SupportEntityBoundaryContactOffsetGroundBaselines = _offsetGroundCount;
            report.SupportEntityBoundaryContactOffsetGroundSwitches = _offsetGroundSwitches;
            report.SupportEntityBoundaryEvidenceReady = _finite && _loadedResolved >= MinimumResolvedSolves;
            report.SupportEntityBoundaryMotionMode = MotionMode;
            report.SupportEntityBoundaryMotionFinite = _motionFinite;
            report.SupportEntityBoundaryMotionInvalidReason = _motionInvalidReason;
            report.SupportNormalWindowMode = NormalWindowMode;
            report.SupportNormalWindowFinite = _normalWindowFinite;
            report.SupportNormalWindowInvalidReason = _normalWindowInvalidReason;
            report.SupportNormalWindowJoinedSolves = _normalJoined;
            report.SupportNormalWindowEvidenceReady = _normalWindowFinite && _motionFinite && _normalJoined >= MinimumResolvedSolves;
            report.SupportCallbackMomentumMode = CallbackMomentumMode;
            report.SupportCallbackMomentumFinite = _callbackAuditFinite;
            report.SupportCallbackMomentumInvalidReason = _callbackAuditInvalidReason;
            if (!format) return;
            report.SupportCallbackMomentumTraceSummary = "loadedAngular(joined-missing-mismatch)=" +
                _angularLoadedJoined + "-" + _angularLoadedMissing + "-" + _angularLoadedMismatch +
                "/loadedAngularModelInputs(valid-unavailable)=" + _angularModelInputsValid + "-" + _angularModelInputsUnavailable +
                "/scope=EXISTING_BOUNDED_WINDOWS/newNativeReads=PRE_ANGULAR_DRAG_AND_MAX_W_ONLY/noNewObserver=True" +
                "/partition=WHOLE_STEP_MODEL_ASSIGNED_TO_PRE_FIRST_CALLBACK_NOT_SOLVER_PASS_TIMING" +
                "/inferredResultant=ENDPOINT_MINUS_COMMAND_AND_DRAG_MODEL_NOT_MEASURED_GRF" +
                "/angularRemainder=DELTA_L_MINUS_THREE_ARL_TORQUES_INCLUDES_ANGULAR_DRAG_AND_SOLVER" +
                "/angularModel=ORDINARY_PRE_SOLVER_DAMPING_CAP_AND_INERTIA_BASIS_COMPARISON_NOT_NATIVE_FORCE" +
                "/ccdReportCoverage=UNVERIFIED_NATIVE_PAIR_FLAGS_AND_PASS_COUNT_NOT_EXPOSED" +
                "/equivalentLever=COMPATIBILITY_NOT_REAL_CONTACT_POINT/observerOnly=True";
            report.SupportNormalWindowTraceSummary = "joined-missing-mismatch=" + _normalJoined + "-" + _normalMissing + "-" + _normalMismatch +
                "/unloadedJoined-missing-mismatch=" + _unloadedJoined + "-" + _unloadedMissing + "-" + _unloadedMismatch +
                "/dragModelJoined-missing=" + _dragModelJoined + "-" + _dragModelMissing +
                "/candidates(union-motion-residual-residualOnly)=" + _motionCandidates + "-" + _motionCriteriaCandidates + "-" + _normalCandidates + "-" + _normalOnlyCandidates +
                "/peakAbsResidualNs(lower-foot-pair)=" + S(_normalPeakLowerResidual) + "-" + S(_normalPeakFootResidual) + "-" + S(_normalPeakPairResidual) +
                "/residualCandidateMinNs=" + S(NormalResidualCandidateMinNs) + "/candidateOnlyNotSafetyGate=True" +
                "/cumulative=FIXED_REFERENCE_NORMAL_WORLD_VECTOR_SUMS_AND_CONTIGUOUS_AUDITED_SEGMENT_ENDPOINTS" +
                "/missing=NA_NOT_ZERO/unloaded=ACTUAL_WORLD_COMMANDS_FIXED_WINDOW_PROJECTION/callbackJ=AUTHORIZED_REPORTED_PAIR_SUM_NOT_COMPLETE_GRF" +
                "/callbackCensus(total-unjoined-other-rejected)=" + _callbackCensusTotal + "-" + _callbackCensusUnjoined + "-" + _callbackCensusOther + "-" + _callbackCensusFailed +
                "/CCDPassCoverage=UNVERIFIED/noContactPointRead=True/model=" + FootContactRockingTetherCouplingObserver.DragModel +
                "/residual=UNEXPLAINED_ENDPOINT_LEDGER_NOT_MEASURED_FORCE";
            report.SupportEntityBoundaryMotionTraceSummary = "loadedEligible-candidates-started-sealed-nativeIncomplete-probeIncomplete=" +
                _motionEligible + "-" + _motionCandidates + "-" + _motionStarted + "-" + _motionFlushed + "-" + _motionIncomplete + "-" + _motionProbeIncomplete +
                "/covered-cooldownSuppressed-limitSuppressed=" + _motionCovered + "-" + _motionCooldownSuppressed + "-" + _motionLimitSuppressed +
                "/pending-postRemaining=" + _motionActive + "-" + _motionPostRemaining +
                "/peak(dWradPerSec-poseDeg-positiveComNormalMm)=" + S(_motionPeakAngularDelta) + "-" + S(_motionPeakRotationDegrees) + "-" + S(_motionPeakComRiseMeters * 1000f) +
                "/output=DEFERRED_REPORT_ONLY_NO_FIXED_WINDOW_TEXT/window(pre-post-maxPerSide-cooldownSolves)=" + MotionPreSolves + "-" + MotionPostSolves + "-" + MaximumMotionWindows + "-" + MotionCooldownSolves +
                "/threshold(dWradPerSec-poseDeg-ComRiseMm)=" + S(MotionAngularDeltaMin) + "-" + S(MotionRotationMinDegrees) + "-" + S(MotionComRiseMinMeters * 1000f) +
                "/sourceStep=RELATIVE_RENDER_SAMPLE_NOT_FIXED_DELTA/kinematicMismatch=ENDPOINT_INDICATOR_NOT_MEASURED_SOLVER_CORRECTION/observerOnly=True";
            report.SupportEntityBoundaryTraceSummary = "captured-resolved-loaded-censored=" + _captured + "-" + _allResolved + "-" + _loadedResolved + "-" + _censored +
                " | loadedNoEnterStay-probeNoHit-entityChanged=" + _loadedNoReactionCallback + "-" + _loadedProbeNoHit + "-" + _loadedEntityChanged +
                " | authorizedEXIT-unjoinedCallbacks=" + _exitCallbacks + "-" + _unjoinedCallbacks +
                " | positionSolverBResolved=" + _positionSolverResolved +
                " | ccdMode(resolved-mismatch)=" + _ccdResolved + "-" + _ccdMismatch +
                " | contactOffset(resolved-mismatch)=" + _offsetResolved + "-" + _offsetMismatch +
                "/entityChangedWithinSolve-readUnavailable=" + _offsetBoundaryChanged + "-" + _offsetReadUnavailable +
                "/environmentExcluded-supportLostWithinSolve=" + _offsetEnvironmentExcluded + "-" + _offsetSupportLost +
                "/groundBaselines-switchesAcrossLoadedSolves=" + _offsetGroundCount + "-" + _offsetGroundSwitches +
                "/groundBaselineCache(capacity-limitReached)=" + MaximumOffsetGroundBaselines + "-" + _offsetGroundCapacityReached +
                " | layerReadReactionContradictions=" + _layerReadReactionContradictions +
                "/layerGetterSemantics=NATIVE_BEHAVIOR_UNQUALIFIED_NO_GLOBAL_MATRIX_WRITE";
            var sb = new StringBuilder();
            for (int k = 0; k < _count; k++) sb.AppendLine(Row(_ring[(_next - _count + k + Capacity) % Capacity]));
            for (int window = 0; window < _motionFlushed; window++)
                sb.AppendLine(FormatSealedMotionWindow(window, report.Side));
            report.SupportEntityBoundaryRecentFrames = sb.ToString();
        }

        private static bool NativeMotionEligible(Frame f) => f.Loaded && NativePairEligible(f);

        private static bool NativePairEligible(Frame f) => f.Resolved && !f.Censored && F(f.Dt) && f.Dt > 0f &&
            f.PreFoot.BodyExists && f.SettledFoot.BodyExists && f.PreFoot.BodyId != 0 && f.PreFoot.BodyId == f.SettledFoot.BodyId &&
            f.PreFoot.Collider.Exists && f.SettledFoot.Collider.Exists && f.PreFoot.Collider.Id != 0 &&
            f.PreFoot.Collider.Id == f.SettledFoot.Collider.Id;

        private static bool MotionThresholdExceeded(Motion m) => m.AngularDelta >= MotionAngularDeltaMin ||
            m.RotationDegrees >= MotionRotationMinDegrees || (m.NormalValid && m.ComNormalDelta >= MotionComRiseMinMeters);

        private static Motion MeasureMotion(Frame f)
        {
            Motion m = new Motion
            {
                ComDelta = f.SettledFoot.Com - f.PreFoot.Com,
                ProxyDelta = f.SettledFoot.Collider.Position - f.PreFoot.Collider.Position,
                DeltaW = f.SettledFoot.W - f.PreFoot.W,
                RotationDegrees = Quaternion.Angle(f.PreFoot.Collider.Rotation, f.SettledFoot.Collider.Rotation),
                NormalValid = f.PreProbe.NormalValid && f.PreProbe.Id == f.Ground.Id
            };
            m.AngularDelta = m.DeltaW.magnitude;
            // Endpoint integration mismatch is only a kinematic indicator. It does not measure PhysX
            // positional correction, penetration, contact-point forces, or a complete reaction ledger.
            m.PositionVelocityMismatch = m.ComDelta - f.SettledFoot.Velocity * f.Dt;
            if (m.NormalValid)
            {
                m.ComNormalDelta = Vector3.Dot(m.ComDelta, f.PreProbe.Normal);
                m.ProxyNormalDelta = Vector3.Dot(m.ProxyDelta, f.PreProbe.Normal);
                m.ComNormalVelocityDelta = Vector3.Dot(f.SettledFoot.Velocity - f.PreFoot.Velocity, f.PreProbe.Normal);
            }
            if (!V(m.ComDelta) || !V(m.ProxyDelta) || !V(m.DeltaW) || !V(m.PositionVelocityMismatch) ||
                !F(m.RotationDegrees) || !F(m.AngularDelta) || !F(m.ComNormalDelta) || !F(m.ProxyNormalDelta) || !F(m.ComNormalVelocityDelta))
                throw new InvalidOperationException("NON_FINITE_MOTION_DERIVATION");
            return m;
        }

        private void ObserveMotionWindow(Frame f, FootAnkleFoundationReport report)
        {
            if (!_motionFinite) return;
            try
            {
                // A skipped probe must not leave a complete window pending forever or merge two events.
                if (_motionActive && _motionPostRemaining == 0) FlushMotionWindow("POST_PROBE_NOT_RECORDED", report);
                if (_motionActive && f.Fixed != _motionWindow[_motionCount - 1].Fixed + 1)
                    FlushMotionWindow("FIXED_SEQUENCE_GAP", report);
                if (!_motionFinite) return;
                if (_motionActive)
                {
                    _motionWindow[_motionCount++] = f;
                    _motionPostRemaining--;
                }
                if (!NativeMotionEligible(f)) return;
                Motion m = MeasureMotion(f);
                _motionEligible++;
                _motionPeakAngularDelta = Mathf.Max(_motionPeakAngularDelta, m.AngularDelta);
                _motionPeakRotationDegrees = Mathf.Max(_motionPeakRotationDegrees, m.RotationDegrees);
                if (m.NormalValid) _motionPeakComRiseMeters = Mathf.Max(_motionPeakComRiseMeters, m.ComNormalDelta);
                f.MotionCandidate = MotionThresholdExceeded(m);
                _ring[(_next - 1 + Capacity) % Capacity] = f;
                UpdateMotionWindowFrame(f);
                if (!f.MotionCandidate) return;
                _motionCriteriaCandidates++; _motionCandidates++;
                SelectMotionWindow(f);
            }
            catch (Exception e) { MotionInvalid("WINDOW_CAPTURE_EXCEPTION:" + e.GetType().Name); }
        }

        private void SelectMotionWindow(Frame f)
        {
            if (!_motionFinite) return;
            if (_motionActive) { _motionCovered++; return; }
            if (_motionStarted >= MaximumMotionWindows) { _motionLimitSuppressed++; return; }
            if (_motionStarted > 0 && f.Fixed - _motionTriggerFixed < MotionCooldownSolves)
            { _motionCooldownSuppressed++; return; }
            _motionCount = 0;
            for (int offset = MotionPreSolves; offset > 0; offset--)
                for (int k = 0; k < _count; k++)
                {
                    Frame previous = _ring[(_next - _count + k + Capacity) % Capacity];
                    if (previous.Fixed == f.Fixed - offset && previous.Resolved && !previous.Censored)
                    { _motionWindow[_motionCount++] = previous; break; }
                }
            _motionPreCount = _motionCount;
            _motionWindow[_motionCount++] = f;
            _motionTriggerFixed = f.Fixed;
            _motionPostRemaining = MotionPostSolves;
            _motionStarted++;
            _motionActive = true;
        }

        private void UpdateMotionWindowFrame(Frame f)
        {
            if (!_motionActive) return;
            for (int i = 0; i < _motionCount; i++)
                if (_motionWindow[i].Fixed == f.Fixed) { _motionWindow[i] = f; break; }
        }

        private void UpdateMotionWindowProbe(Frame f, FootAnkleFoundationReport report)
        {
            UpdateMotionWindowFrame(f);
            if (!_motionActive) return;
            if (_motionPostRemaining == 0) FlushMotionWindow("POST_CONTEXT_RECORDED", report);
        }

        private void FlushMotionWindow(string reason, FootAnkleFoundationReport report)
        {
            if (!_motionActive) return;
            _motionActive = false;
            try
            {
                if (_motionFlushed >= MaximumMotionWindows)
                { MotionInvalid("SEALED_WINDOW_CAPACITY_EXCEEDED"); return; }
                int window = _motionFlushed;
                bool nativeComplete = _motionPreCount == MotionPreSolves && _motionPostRemaining == 0;
                bool probesComplete = true;
                for (int i = 0; i < _motionCount; i++)
                {
                    Frame f = _motionWindow[i];
                    _sealedMotionFrames[window * MotionWindowCapacity + i] = f;
                    nativeComplete &= f.Resolved && !f.Censored;
                    probesComplete &= f.ProbeRecorded;
                }
                if (!nativeComplete) _motionIncomplete++;
                if (!probesComplete) _motionProbeIncomplete++;
                _sealedCounts[window] = _motionCount;
                _sealedPreCounts[window] = _motionPreCount;
                _sealedPostRemaining[window] = _motionPostRemaining;
                _sealedTriggers[window] = _motionTriggerFixed;
                _sealedReasons[window] = reason;
                _motionFlushed++; // legacy field now counts sealed windows, not synchronous log writes
            }
            catch (Exception e) { MotionInvalid("WINDOW_SEAL_EXCEPTION:" + e.GetType().Name); }
            finally
            {
                Array.Clear(_motionWindow, 0, _motionWindow.Length);
                _motionCount = _motionPreCount = _motionPostRemaining = 0;
            }
        }

        private string FormatSealedMotionWindow(int window, string side)
        {
            int offset = window * MotionWindowCapacity, count = _sealedCounts[window], trigger = _sealedTriggers[window];
            bool nativeComplete = _sealedPreCounts[window] == MotionPreSolves && _sealedPostRemaining[window] == 0;
            bool probesComplete = true;
            for (int i = 0; i < count; i++)
            {
                Frame f = _sealedMotionFrames[offset + i];
                nativeComplete &= f.Resolved && !f.Censored;
                probesComplete &= f.ProbeRecorded;
            }
            Vector3 normalBasis = Vector3.zero;
            bool triggerBasis = false;
            for (int i = 0; i < count; i++)
                if (_sealedMotionFrames[offset + i].NormalJoined && _sealedMotionFrames[offset + i].Normal.NormalValid && _sealedMotionFrames[offset + i].Fixed == trigger)
                { normalBasis = _sealedMotionFrames[offset + i].Normal.PreNormal; triggerBasis = true; break; }
            if (!triggerBasis)
                for (int i = 0; i < count; i++)
                    if (_sealedMotionFrames[offset + i].NormalJoined && _sealedMotionFrames[offset + i].Normal.NormalValid) { normalBasis = _sealedMotionFrames[offset + i].Normal.PreNormal; break; }
            WindowLedger ledger = default;
            var sb = new StringBuilder();
            sb.Append("[ARL][FOOT_SUPPORT_SOLVE_WINDOW_V04350] side=").Append(side ?? "<none>")
                .Append(" | event=").Append((window + 1)).Append(" | triggerPreFixed=").Append(trigger)
                .Append(" | reason=").Append(_sealedReasons[window]).Append(" | nativeContextComplete=").Append(nativeComplete)
                .Append("/probesComplete=").Append(probesComplete).Append(" | preAvailable-postRemaining=")
                .Append(_sealedPreCounts[window]).Append('-').Append(_sealedPostRemaining[window])
                .Append(" | normalBasis=").Append(triggerBasis ? "TRIGGER_PRE_NORMAL" : "FIRST_AUDITED_PRE_NORMAL_OR_NONE")
                .Append("/N=").Append(W(normalBasis)).Append(" | candidateMinAbsResidualNs=").Append(S(NormalResidualCandidateMinNs))
                .Append(" | angularSource=UNRESOLVED/kinematicMismatch=ENDPOINT_INDICATOR_NOT_MEASURED_SOLVER_CORRECTION")
                .Append(" | dragModel=").Append(FootContactRockingTetherCouplingObserver.DragModel)
                .Append("/CCDPassCoverage=UNVERIFIED/callbackJ=AUTHORIZED_REPORTED_PAIR_SUM")
                .Append(" | proxyPose=COLLIDER_TRANSFORM_WORLD_SNAPSHOT/reusesNativeSnapshots=True/observerOnly=True");
            for (int i = 0; i < count; i++)
            {
                Frame f = _sealedMotionFrames[offset + i];
                string role = f.Fixed == trigger ? "TRIGGER" : (f.Fixed < trigger ? "PRE" : "POST");
                sb.AppendLine().Append("[ARL][FOOT_SUPPORT_SOLVE_FRAME_V04350] side=").Append(side ?? "<none>")
                    .Append(" | event=").Append((window + 1)).Append(" | role=").Append(role).Append(" | ").Append(MotionFrameRow(f));
                AppendCallbackMomentum(sb, f, normalBasis);
                AppendNormalWindowLedger(sb, f, ref ledger, normalBasis);
            }
            sb.AppendLine().Append("[ARL][FOOT_SUPPORT_NORMAL_WINDOW_SUM_V04350] side=").Append(side ?? "<none>")
                .Append(" | event=").Append((window + 1)).Append(" | nativeContextComplete=").Append(nativeComplete)
                .Append("/probesComplete=").Append(probesComplete).Append(" | ").Append(NormalWindowSum(ref ledger, normalBasis))
                .Append(" | scope=AUDITED_LOADED_AND_UNLOADED_SEGMENTS/inputRevision=").Append(FootContactRockingTetherCouplingObserver.InputRevision)
                .Append("/callbackJ=NOT_COMPLETE_GRF/residual=NOT_MEASURED_SOLVER_CORRECTION");
            return sb.ToString();
        }

        private void AppendNormalWindowLedger(StringBuilder sb, Frame f, ref WindowLedger ledger, Vector3 n)
        {
            try
            {
                ledger.Add(f);
                string status = !f.Resolved || f.Censored ? "NA_UNRESOLVED_OR_CENSORED" :
                    (!f.NormalRecorded ? "NA_JOIN_NOT_RECORDED" : (!f.NormalJoined ? "NA_SOURCE_MISSING_OR_NATIVE_MISMATCH" :
                    (f.Loaded ? "JOINED_LOADED" : "JOINED_UNLOADED_WORLD")));
                sb.Append(" | candidate(motion-normalResidual)=").Append(f.MotionCandidate).Append('-').Append(f.NormalCandidate)
                    .Append(" | normalLedger=").Append(status);
                if (f.NormalJoined)
                {
                    sb.Append('/').Append(FootContactRockingTetherCouplingObserver.FormatNormalLoad(f.Normal, f.Censored));
                    if (V(n) && Mathf.Abs(n.sqrMagnitude - 1f) <= NativeJoinTolerance)
                    {
                        var d = f.Normal;
                        Vector3 lowerR = d.LowerDeltaPWorld - d.LowerKnownWorld;
                        Vector3 footR = d.FootDeltaPWorld - d.FootCommandWorld - d.GroundWorld;
                        sb.Append("/windowStepRawResidualNs(lower-foot-pair)=").Append(S(Vector3.Dot(lowerR, n))).Append('-')
                            .Append(S(Vector3.Dot(footR, n))).Append('-').Append(S(Vector3.Dot(lowerR + footR, n)));
                        sb.Append("/windowStepDragModelJNs(lower-foot)=").Append(d.DragModelValid ? S(Vector3.Dot(d.LowerDragWorld, n)) + "-" + S(Vector3.Dot(d.FootDragWorld, n)) : "NA");
                        sb.Append("/windowStepResidualAfterDragModelNs(lower-foot-pair)=").Append(d.DragModelValid ?
                            S(Vector3.Dot(lowerR - d.LowerDragWorld, n)) + "-" + S(Vector3.Dot(footR - d.FootDragWorld, n)) + "-" +
                            S(Vector3.Dot(lowerR + footR - d.LowerDragWorld - d.FootDragWorld, n)) : "NA");
                    }
                    else sb.Append("/windowStepNs=NA_NO_REFERENCE_NORMAL");
                }
                sb.Append(" | cumulative={").Append(NormalWindowSum(ref ledger, n)).Append('}');
            }
            catch (Exception e) { NormalWindowInvalid("WINDOW_LEDGER_OUTPUT_EXCEPTION:" + e.GetType().Name); sb.Append(" | normalLedger=NA_OUTPUT_EXCEPTION"); }
        }

        private string NormalWindowSum(ref WindowLedger ledger, Vector3 n)
        {
            try { return ledger.Format(n); }
            catch (Exception e) { NormalWindowInvalid("WINDOW_SUM_EXCEPTION:" + e.GetType().Name); return "NA_NONFINITE_CUMULATIVE_WINDOW"; }
        }

        private void NormalWindowInvalid(string reason)
        {
            _normalWindowFinite = false;
            if (_normalWindowInvalidReason == null) _normalWindowInvalidReason = reason;
        }

        private struct CallbackMomentum
        {
            internal bool HasCallbacks;
            internal Vector3 FirstDeltaP, MiddleDeltaP, TailDeltaP;
            internal Vector3 FirstResidual, MiddleResidual, TailResidual;
            internal Vector3 Residual, InferredGround, PartitionClosure;
        }

        private static bool TryMeasureCallbackMomentum(Frame f, out CallbackMomentum m)
        {
            m = default;
            if (!f.NormalJoined || !f.Normal.DragModelValid || !NativePairEligible(f)) return false;
            var d = f.Normal;
            if (!F(d.FootMass) || d.FootMass <= 0f || !V(d.FootDragWorld)) return false;
            m.InferredGround = d.FootDeltaPWorld - d.FootCommandWorld - d.FootDragWorld;
            m.Residual = m.InferredGround - f.GroundJ;
            if (!V(m.InferredGround) || !V(m.Residual)) return false;
            m.HasCallbacks = f.Enter + f.Stay + f.Exit > 0;
            if (!m.HasCallbacks) return true; // total endpoint comparison remains useful; callback phases are NA.
            if (!f.FirstCallbackFoot.BodyExists || !f.LastCallbackFoot.BodyExists ||
                f.FirstCallbackFoot.BodyId != f.PreFoot.BodyId || f.LastCallbackFoot.BodyId != f.PreFoot.BodyId ||
                f.FirstCallbackFoot.Collider.Id != f.PreFoot.Collider.Id || f.LastCallbackFoot.Collider.Id != f.PreFoot.Collider.Id ||
                !V(f.FirstCallbackFoot.Velocity) || !V(f.LastCallbackFoot.Velocity) ||
                !V(f.FirstCallbackFoot.W) || !V(f.LastCallbackFoot.W) || !V(f.PreFoot.W) || !V(f.SettledFoot.W) ||
                !V(f.FirstCallbackGroundPrefixJ) || !V(f.LastCallbackGroundPrefixJ)) return false;
            m.FirstDeltaP = d.FootMass * (f.FirstCallbackFoot.Velocity - d.PreFootV);
            m.MiddleDeltaP = d.FootMass * (f.LastCallbackFoot.Velocity - f.FirstCallbackFoot.Velocity);
            m.TailDeltaP = d.FootMass * (d.SettledFootV - f.LastCallbackFoot.Velocity);
            // Algebraic partition only: assign the WHOLE scheduled input/model to the first interval.
            // Callbacks are managed snapshots, not individually measured PhysX passes or force intervals.
            m.FirstResidual = m.FirstDeltaP - d.FootCommandWorld - d.FootDragWorld - f.FirstCallbackGroundPrefixJ;
            m.MiddleResidual = m.MiddleDeltaP - (f.LastCallbackGroundPrefixJ - f.FirstCallbackGroundPrefixJ);
            m.TailResidual = m.TailDeltaP - (f.GroundJ - f.LastCallbackGroundPrefixJ);
            m.PartitionClosure = m.FirstResidual + m.MiddleResidual + m.TailResidual - m.Residual;
            return V(m.FirstDeltaP) && V(m.MiddleDeltaP) && V(m.TailDeltaP) && V(m.FirstResidual) &&
                V(m.MiddleResidual) && V(m.TailResidual) && V(m.PartitionClosure);
        }

        private void AppendCallbackMomentum(StringBuilder sb, Frame f, Vector3 n)
        {
            try
            {
                sb.Append(" | callbackMomentumV04360={");
                if (!TryMeasureCallbackMomentum(f, out CallbackMomentum m))
                {
                    sb.Append("NA_NO_JOINED_MODEL_OR_CALLBACK_ENDPOINT");
                }
                else
                {
                    sb.Append("reportedGroundJNs=").Append(W(f.GroundJ))
                        .Append("/inferredGroundJNs=").Append(W(m.InferredGround))
                        .Append("/endpointGapNs=").Append(W(m.Residual))
                        .Append("/fixedNormal(reported-inferred-gap)=").Append(N(f.GroundJ, n)).Append('-')
                        .Append(N(m.InferredGround, n)).Append('-').Append(N(m.Residual, n))
                        .Append("/endpointSupportStable=").Append(EndpointSupportStable(f))
                        .Append("/callbackCount(E-S-X)=").Append(f.Enter).Append('-').Append(f.Stay).Append('-').Append(f.Exit);
                    if (m.HasCallbacks)
                    {
                        sb.Append("/phase(first-last)=").Append(f.FirstCallbackPhase ?? "<none>").Append('-').Append(f.LastCallbackPhase ?? "<none>")
                            .Append("/reportedPrefixJNs(first-last-total)=").Append(W(f.FirstCallbackGroundPrefixJ)).Append('-')
                            .Append(W(f.LastCallbackGroundPrefixJ)).Append('-').Append(W(f.GroundJ))
                            .Append("/deltaPNs(pre-first/first-last/last-settled)=").Append(W(m.FirstDeltaP)).Append('-')
                            .Append(W(m.MiddleDeltaP)).Append('-').Append(W(m.TailDeltaP))
                            .Append("/gapPartitionNs(first-middle-tail)=").Append(W(m.FirstResidual)).Append('-')
                            .Append(W(m.MiddleResidual)).Append('-').Append(W(m.TailResidual))
                            .Append("/gapPartitionNormalNs(first-middle-tail)=").Append(N(m.FirstResidual, n)).Append('-')
                            .Append(N(m.MiddleResidual, n)).Append('-').Append(N(m.TailResidual, n))
                            .Append("/partitionClosureErrorNs=").Append(S(m.PartitionClosure.magnitude))
                            .Append("/deltaWradPerSec(pre-first/first-last/last-settled)=").Append(W(f.FirstCallbackFoot.W - f.PreFoot.W)).Append('-')
                            .Append(W(f.LastCallbackFoot.W - f.FirstCallbackFoot.W)).Append('-').Append(W(f.SettledFoot.W - f.LastCallbackFoot.W));
                    }
                    else sb.Append("/phases=NA_NO_AUTHORIZED_CALLBACK");
                    sb.Append("/partition=ALGEBRAIC_WHOLE_STEP_MODEL_NOT_SOLVER_PASS_TIMING");
                    sb.Append("/angularCompatibility={").Append(AngularCompatibilityRow(f, m.InferredGround)).Append('}');
                }
                sb.Append("/inferredJ=MODEL_DEPENDENT_NOT_MEASURED_GRF/observerOnly=True}");
            }
            catch (Exception e)
            {
                _callbackAuditFinite = false;
                if (_callbackAuditInvalidReason == null) _callbackAuditInvalidReason = "OUTPUT_EXCEPTION:" + e.GetType().Name;
                sb.Append("NA_CALLBACK_MOMENTUM_OUTPUT_EXCEPTION}");
            }
        }

        private static string AngularCompatibilityRow(Frame f, Vector3 inferredGround)
        {
            if (!f.AngularJoined) return "NA_" + (f.AngularJoinReason ?? "ANGULAR_JOIN_NOT_RECORDED");
            var a = f.Angular;
            Vector3 deltaL = a.SettledL - a.PreL;
            Vector3 remainder = deltaL - a.ArlJ;
            if (!V(deltaL) || !V(remainder)) return "NA_NONFINITE_ANGULAR_REMAINDER";
            return "joined=True/pre-settled=" + a.PreFixed + "-" + a.SettledFixed +
                "/deltaLNmS=" + W(deltaL) + "/arlTorqueJNmS(transport-pd-support-sum)=" +
                W(a.TransportJ) + "-" + W(a.PdJ) + "-" + W(a.SupportJ) + "-" + W(a.ArlJ) +
                "/nonArlAngularRemainderNmS=" + W(remainder) + "/preBoundsRadiusMm=" + S(a.BoundsRadius * 1000f) +
                "/reportedResultant={" + EquivalentLeverRow(f.GroundJ, remainder, a.BoundsRadius) + "}" +
                "/inferredResultant={" + EquivalentLeverRow(inferredGround, remainder, a.BoundsRadius) + "}" +
                "/angularModelV04370={" + AngularModelRow(a, f.GroundJ, inferredGround) + "}" +
                "/KIncludesAngularDragAndSolver=True/lever=COMPATIBILITY_NOT_REAL_CONTACT_POINT";
        }

        private struct AngularModel
        {
            internal float DampingFactor;
            internal bool CapActive;
            internal Vector3 CommandW, DampedW, ExpectedW;
            internal Vector3 DragJ, CapJ, InertiaBasisTerm, VelocityRemainder, Closure;
        }

        private static bool AngularModelInputsValid(FootContactAngularImpulseAttributionObserver.AngularSolve a) =>
            a.ModelParametersAvailable && F(a.PreAngularDrag) && a.PreAngularDrag >= 0f &&
            F(a.PreMaxAngularVelocity) && a.PreMaxAngularVelocity >= 0f && F(a.Dt) && a.Dt > 0f &&
            V(a.PreInertia) && a.PreInertia.x > 0.000001f && a.PreInertia.y > 0.000001f && a.PreInertia.z > 0.000001f &&
            V(a.SettledInertia) && a.SettledInertia.x > 0.000001f && a.SettledInertia.y > 0.000001f && a.SettledInertia.z > 0.000001f &&
            UnitQuaternion(a.PrePrincipalWorld) && UnitQuaternion(a.SettledPrincipalWorld);

        private static bool UnitQuaternion(Quaternion q)
        {
            float sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
            return F(sq) && Mathf.Abs(sq - 1f) <= 0.0001f;
        }

        private static Vector3 ApplyWorldInertia(Vector3 inertia, Quaternion principalWorld, Vector3 w)
        {
            Vector3 local = Quaternion.Inverse(principalWorld) * w;
            return principalWorld * new Vector3(inertia.x * local.x, inertia.y * local.y, inertia.z * local.z);
        }

        private static Vector3 ApplyWorldInverseInertia(Vector3 inertia, Quaternion principalWorld, Vector3 j)
        {
            Vector3 local = Quaternion.Inverse(principalWorld) * j;
            return principalWorld * new Vector3(local.x / inertia.x, local.y / inertia.y, local.z / inertia.z);
        }

        private static bool TryMeasureAngularModel(FootContactAngularImpulseAttributionObserver.AngularSolve a, out AngularModel m)
        {
            m = default;
            if (!AngularModelInputsValid(a) || !V(a.PreW) || !V(a.SettledW) || !V(a.PreL) || !V(a.SettledL) || !V(a.ArlJ)) return false;
            if ((ApplyWorldInertia(a.PreInertia, a.PrePrincipalWorld, a.PreW) - a.PreL).magnitude > NativeJoinTolerance ||
                (ApplyWorldInertia(a.SettledInertia, a.SettledPrincipalWorld, a.SettledW) - a.SettledL).magnitude > NativeJoinTolerance) return false;
            float dampingDt = a.PreAngularDrag * a.Dt;
            if (!F(dampingDt)) return false;
            m.DampingFactor = Mathf.Max(0f, 1f - dampingDt);
            m.CommandW = a.PreW + ApplyWorldInverseInertia(a.PreInertia, a.PrePrincipalWorld, a.ArlJ);
            m.DampedW = m.CommandW * m.DampingFactor;
            float dampedMagnitude = m.DampedW.magnitude;
            if (!V(m.CommandW) || !V(m.DampedW) || !F(dampedMagnitude)) return false;
            m.CapActive = dampedMagnitude > a.PreMaxAngularVelocity;
            m.ExpectedW = m.CapActive ? m.DampedW * (a.PreMaxAngularVelocity / dampedMagnitude) : m.DampedW;
            m.DragJ = ApplyWorldInertia(a.PreInertia, a.PrePrincipalWorld, m.DampedW - m.CommandW);
            m.CapJ = ApplyWorldInertia(a.PreInertia, a.PrePrincipalWorld, m.ExpectedW - m.DampedW);
            Vector3 settledInPreBasis = ApplyWorldInertia(a.PreInertia, a.PrePrincipalWorld, a.SettledW);
            // Endpoint inertia reorientation is not a separately measured external torque.
            m.InertiaBasisTerm = a.SettledL - settledInPreBasis;
            m.VelocityRemainder = ApplyWorldInertia(a.PreInertia, a.PrePrincipalWorld, a.SettledW - m.ExpectedW);
            m.Closure = a.SettledL - a.PreL - a.ArlJ - m.DragJ - m.CapJ - m.InertiaBasisTerm - m.VelocityRemainder;
            return V(m.ExpectedW) && V(m.DragJ) && V(m.CapJ) && V(m.InertiaBasisTerm) && V(m.VelocityRemainder) &&
                V(m.Closure) && F(m.Closure.magnitude);
        }

        private static string AngularModelRow(FootContactAngularImpulseAttributionObserver.AngularSolve a,
            Vector3 reportedGround, Vector3 inferredGround)
        {
            if (!TryMeasureAngularModel(a, out AngularModel m))
                return "NA_" + (a.ModelUnavailableReason ?? "INVALID_MODEL_INPUT_OR_CACHED_INERTIA");
            return "available=True/preParameters(drag-maxW)=" + S(a.PreAngularDrag) + "-" + S(a.PreMaxAngularVelocity) +
                "/inertiaKgM2(pre-settled)=" + W(a.PreInertia) + "-" + W(a.SettledInertia) +
                "/principalWorld(pre-settled)=" + QuaternionRow(a.PrePrincipalWorld) + "-" + QuaternionRow(a.SettledPrincipalWorld) +
                "/LNmS(pre-settled)=" + W(a.PreL) + "-" + W(a.SettledL) +
                "/dampingFactor=" + S(m.DampingFactor) + "/preSolverCapActive=" + m.CapActive +
                "/W(command-damped-predictedPreSolver)=" + W(m.CommandW) + "-" + W(m.DampedW) + "-" + W(m.ExpectedW) +
                "/angularDragModelJNmS=" + W(m.DragJ) + "/angularCapModelJNmS=" + W(m.CapJ) +
                "/endpointInertiaBasisTermNmS=" + W(m.InertiaBasisTerm) +
                "/preInertiaVelocityRemainderNmS=" + W(m.VelocityRemainder) + "/modelClosureErrorNmS=" + S(m.Closure.magnitude) +
                "/reportedResultantAfterModel={" + EquivalentLeverRow(reportedGround, m.VelocityRemainder, a.BoundsRadius) + "}" +
                "/inferredResultantAfterModel={" + EquivalentLeverRow(inferredGround, m.VelocityRemainder, a.BoundsRadius) + "}" +
                "/parameters=PRE_SNAPSHOT_CONSTANT_STEP_ASSUMPTION/model=ORDINARY_UNCONSTRAINED_DAMP_THEN_CAP_IN_PRE_INERTIA_BASIS" +
                "/remainderIncludesContactGyroAndNativeSolver=True/notMeasuredTorque=True/noCCDPassAttribution=True";
        }

        private static string EquivalentLeverRow(Vector3 j, Vector3 k, float radius)
        {
            float jSq = j.sqrMagnitude;
            if (!V(j) || !V(k) || !F(jSq) || jSq <= 0.000000000001f) return "NA_ZERO_OR_NONFINITE_RESULTANT";
            Vector3 lever = Vector3.Cross(j, k) / jSq;
            Vector3 parallelK = j * (Vector3.Dot(k, j) / jSq);
            float leverMm = lever.magnitude * 1000f;
            float ratio = F(radius) && radius > 0.000001f ? lever.magnitude / radius : 0f;
            if (!V(lever * 1000f) || !V(parallelK) || !F(leverMm) || !F(ratio)) return "NA_NONFINITE_COMPATIBILITY";
            return "eqLeverMm=" + W(lever * 1000f) + "/magMm=" + S(leverMm) +
                "/radiusRatio=" + (F(radius) && radius > 0.000001f ? S(ratio) : "NA_NO_RADIUS") +
                "/parallelAngularRemainderNmS=" + W(parallelK);
        }

        private static string QuaternionRow(Quaternion q) => "(" + S(q.x) + "," + S(q.y) + "," + S(q.z) + "," + S(q.w) + ")";

        private static string N(Vector3 value, Vector3 normal) => V(normal) && Mathf.Abs(normal.sqrMagnitude - 1f) <= NativeJoinTolerance
            ? S(Vector3.Dot(value, normal)) : "NA_NO_FIXED_NORMAL";

        private static bool EndpointSupportStable(Frame f) => f.PreGround.Exists && f.SettledGround.Exists &&
            f.PreGround.Id == f.SettledGround.Id && f.PreGround.Enabled && f.SettledGround.Enabled &&
            f.PreGround.ActiveHierarchy && f.SettledGround.ActiveHierarchy && !EntityChanged(f);

        private static string MotionFrameRow(Frame f)
        {
            string row = "pre-earlySettled=" + f.Fixed + "-" + f.Settled + "/dt=" + S(f.Dt) +
                " | loaded-resolved-censored=" + f.Loaded + "-" + f.Resolved + "-" + f.Censored +
                " | ccdArm=" + (f.CollisionArm ?? "<none>") + "/expectedMode=" + f.ExpectedCollisionMode +
                "/observedResolved=" + CcdModeInterventionResolved(f) + "/readbackMismatch=" + CcdModeReadbackMismatch(f) +
                " | " + ContactOffsetFrameSummary(f) +
                " | render-fresh-held-age=" + f.Render + "-" + f.Fresh + "-" + f.Held + "-" + S(f.Age) +
                " | sourceRelativeRenderStep(preValid/preDeg-settledRecorded/settledRender/settledValid/settledDeg)=" +
                f.SourcePreStepValid + "/" + W(f.SourcePreStepDegrees) + "-" + f.SourceSettledRecorded + "/" + f.SourceSettledRender + "/" +
                f.SourceSettledStepValid + "/" + W(f.SourceSettledStepDegrees) +
                " | settledSourceClock(fresh-held-age)=" + (f.SourceSettledRecorded ? f.SourceSettledFresh + "-" + f.SourceSettledHeld + "-" + S(f.SourceSettledAge) : "NA") +
                " | gait(pre-settledBeforeLifecycle)=" + (f.PreGait ?? "<none>") + "-" + (f.SettledGaitBeforeLifecycle ?? "<none>") +
                " | ankleAnchorMm(pre-settled)=" + (f.PreAnchorValid ? S(f.PreAnchor * 1000f) : "NA") + "-" +
                (f.SettledAnchorValid ? S(f.SettledAnchor * 1000f) : "NA") +
                " | groundId=" + f.Ground.Id + "/path=" + f.Ground.Path +
                " | contact(pre-settled)=" + f.PreContact + "-" + f.SettledContact +
                " | callbacks(E-S-X-projected-invalid)=" + f.Enter + "-" + f.Stay + "-" + f.Exit + "-" + f.Projected + "-" + f.ProjectionInvalid +
                "/reportedJ(first-last-enterSum-staySum)=" + (f.Projected > 0 ? W(f.FirstReactionJ) + "-" + W(f.LastReactionJ) + "-" + W(f.EnterJ) + "-" + W(f.StayJ) : "NA_NO_PROJECTED_CALLBACK") +
                "/sumReportedJMagnitudeNs=" + S(f.ReactionMagnitudeSum) + "/CCDPassCoverage=UNVERIFIED" +
                "/callbackCensus(total-other-rejected)=" + f.CensusTotal + "-" + f.CensusOther + "-" + f.CensusRejected +
                "/GroundJ=" + W(f.GroundJ) + " | pre={" + MotionBody(f.PreFoot) + "}/probe={" + P(f.PreProbe) + "}";
            if (f.Enter + f.Stay + f.Exit > 0)
                row += " | callbackAuthorizedId(first-last)=" + f.FirstCallbackAuthorizedId + "-" + f.LastCallbackAuthorizedId +
                    " | firstCallback={" + MotionBody(f.FirstCallbackFoot) + "} | lastCallback={" + MotionBody(f.LastCallbackFoot) + "}";
            if (!f.Resolved) return row + " | derived=NA_UNRESOLVED_PRE_ONLY";
            row += " | earlySettled={" + MotionBody(f.SettledFoot) + "}/probeRecorded=" + f.ProbeRecorded + "/probe={" + P(f.SettledProbe) + "}" +
                " | settledAuthorizedId=" + f.SettledAuthorizedId + "/entityChanged=" + EntityChanged(f) + "/positionSolverBResolved=" + PositionSolverInterventionResolved(f) +
                " | ground(pre-firstCallback-lastCallback-settled)={" + MotionGround(f.PreGround) + "};{" +
                (f.Enter + f.Stay + f.Exit > 0 ? MotionGround(f.FirstCallbackGround) : "NA") + "};{" +
                (f.Enter + f.Stay + f.Exit > 0 ? MotionGround(f.LastCallbackGround) : "NA") + "};{" + MotionGround(f.SettledGround) + "}";
            if (!NativeMotionEligible(f)) return row + " | derived=NA_NOT_ELIGIBLE_LOADED_NATIVE_PAIR";
            Motion m = MeasureMotion(f);
            return row + " | dCOMmm=" + W(m.ComDelta * 1000f) + "/normalMm=" + (m.NormalValid ? S(m.ComNormalDelta * 1000f) : "NA") +
                " | dProxyMm=" + W(m.ProxyDelta * 1000f) + "/normalMm=" + (m.NormalValid ? S(m.ProxyNormalDelta * 1000f) : "NA") +
                " | proxyPoseDeltaDeg=" + S(m.RotationDegrees) + "/dWradPerSec=" + W(m.DeltaW) + "/mag=" + S(m.AngularDelta) +
                " | dCOMVnormal=" + (m.NormalValid ? S(m.ComNormalVelocityDelta) : "NA") +
                " | COMDeltaMinusSettledVdtMm=" + W(m.PositionVelocityMismatch * 1000f) +
                " | preNormalValid=" + m.NormalValid + "/N=" + W(f.PreProbe.Normal) +
                " | poseAdvanceV04380={" + PoseAdvanceRow(f, m) + "}";
        }

        private static string PoseAdvanceRow(Frame f, Motion m)
        {
            // Endpoint comparison only: this ratio is not an effective timestep or a CCD pass clock.
            if (!F(f.Dt) || f.Dt <= 0f || !V(f.SettledFoot.W) || !F(m.RotationDegrees))
                return "available=False/reason=INVALID_ENDPOINT_OR_DT";
            float endWTurnDegrees = f.SettledFoot.W.magnitude * f.Dt * Mathf.Rad2Deg;
            if (!F(endWTurnDegrees) || endWTurnDegrees <= 0.0001f)
                return "available=False/reason=END_W_TURN_ZERO_TINY_OR_NONFINITE";
            float ratio = m.RotationDegrees / endWTurnDegrees;
            if (!F(ratio)) return "available=False/reason=NONFINITE_RATIO";
            return "available=True/observedPoseDeg=" + S(m.RotationDegrees) + "/endWTimesDtDeg=" + S(endWTurnDegrees) +
                "/netPoseToEndWTurnRatio=" + S(ratio) +
                "/pose=COLLIDER_WORLD_ENDPOINTS/notIntegrationDt=True/notCCDPassTiming=True/observerOnly=True";
        }

        private static string MotionBody(FootState s) => "bodyExists-id=" + s.BodyExists + "-" + s.BodyId +
            "/colliderId=" + s.Collider.Id + "/position=" + W(s.Collider.Position) + "/rotation=" + R(s.Collider.Rotation) +
            "/COM=" + W(s.Com) + "/V=" + W(s.Velocity) + "/W=" + W(s.W) +
            "/solver(velocity-position)=" + s.SolverVelocity + "-" + s.SolverPosition +
            "/gravity-kinematic-CCD=" + s.Gravity + "-" + s.Kinematic + "-" + s.CollisionMode;

        private static string MotionGround(Entity s) => "exists-id=" + s.Exists + "-" + s.Id +
            "/enabled-self-hierarchy-parent=" + s.Enabled + "-" + s.ActiveSelf + "-" + s.ActiveHierarchy + "-" + s.ParentId +
            "/position=" + W(s.Position) + "/rotation=" + R(s.Rotation);

        private void MotionInvalid(string reason)
        {
            _motionFinite = false;
            _motionActive = false;
            Array.Clear(_motionWindow, 0, _motionWindow.Length);
            _motionCount = _motionPreCount = _motionPostRemaining = 0;
            if (_motionInvalidReason == null) _motionInvalidReason = reason;
            // Separate diagnostic domain: never poison the existing entity qualification or controller.
        }

        private GroundIdentity GetIdentity(Collider ground)
        {
            int id = ground != null ? ground.GetInstanceID() : 0;
            bool pathMatches = _identity != null && _identity.Id == id && _identity.Collider == ground;
            if (pathMatches)
                for (int i = 0; i < _identity.Ancestors.Length; i++)
                {
                    Transform t = _identity.Ancestors[i];
                    Transform expectedParent = i + 1 < _identity.Ancestors.Length ? _identity.Ancestors[i + 1] : null;
                    if (t == null || (!(i + 1 == _identity.Ancestors.Length && _identity.Truncated) && t.parent != expectedParent))
                    { pathMatches = false; break; }
                }
            if (pathMatches) return _identity;
            var identity = new GroundIdentity { Collider = ground, Id = id, Ancestors = new Transform[0], Path = "<none>" };
            if (ground != null)
            {
                var ancestors = new Transform[MaximumAncestors];
                var path = new StringBuilder();
                Transform t = ground.transform;
                int count = 0;
                while (t != null && count < MaximumAncestors)
                {
                    ancestors[count++] = t;
                    if (path.Length > 0) path.Append(" <- ");
                    path.Append(t.name).Append('#').Append(t.GetInstanceID());
                    t = t.parent;
                }
                identity.Truncated = t != null;
                Array.Resize(ref ancestors, count);
                identity.Ancestors = ancestors;
                identity.Path = path.ToString();
            }
            _identity = identity;
            return identity;
        }

        private static Entity ReadCollider(Collider collider)
        {
            Entity s = new Entity { Exists = collider != null };
            if (!s.Exists) return s;
            s.Id = collider.GetInstanceID();
            s.Enabled = collider.enabled;
            s.Trigger = collider.isTrigger;
            s.Layer = collider.gameObject.layer;
            s.ActiveSelf = collider.gameObject.activeSelf;
            s.ActiveHierarchy = collider.gameObject.activeInHierarchy;
            Transform t = collider.transform;
            s.ParentId = t.parent != null ? t.parent.GetInstanceID() : 0;
            s.Position = t.position;
            s.Rotation = t.rotation;
            s.Scale = t.lossyScale;
            Bounds b = collider.bounds;
            s.BoundsCenter = b.center;
            s.BoundsExtents = b.extents;
            // V04390 adds one scalar to these existing boundary snapshots, with an independent missing marker.
            // A failing offset read cannot erase historical entity/velocity evidence or retry through formatting.
            try
            {
                s.ContactOffset = collider.contactOffset;
                s.ContactOffsetRead = F(s.ContactOffset) && s.ContactOffset > 0f;
            }
            catch { s.ContactOffsetRead = false; }
            if (!V(s.Position) || !V(s.Scale) || !V(s.BoundsCenter) || !V(s.BoundsExtents) || !Q(s.Rotation))
                throw new InvalidOperationException("NON_FINITE_COLLIDER_SNAPSHOT");
            return s;
        }

        private static Entity ReadGround(GroundIdentity identity)
        {
            Entity s = ReadCollider(identity.Collider);
            s.AncestorsObserved = true;
            s.AncestorLinksMatch = true;
            for (int i = 0; i < identity.Ancestors.Length; i++)
            {
                Transform t = identity.Ancestors[i];
                if (t == null) { s.AncestorLinksMatch = false; continue; }
                ulong bit = 1UL << i;
                s.AncestorAlive |= bit;
                if (t.gameObject.activeSelf) s.AncestorSelf |= bit;
                if (t.gameObject.activeInHierarchy) s.AncestorHierarchy |= bit;
                Transform expectedParent = i + 1 < identity.Ancestors.Length ? identity.Ancestors[i + 1] : null;
                if (!(i + 1 == identity.Ancestors.Length && identity.Truncated) && t.parent != expectedParent) s.AncestorLinksMatch = false;
            }
            return s;
        }

        private FootState ReadFoot(Collider collider, Rigidbody body, Collider ground)
        {
            FootState s = new FootState { Collider = ReadCollider(collider), BodyExists = body != null };
            if (s.BodyExists)
            {
                s.BodyPointer = body.Pointer;
                s.BodyId = body.GetInstanceID();
                s.Gravity = body.useGravity;
                s.Kinematic = body.isKinematic;
                s.CollisionMode = (int)body.collisionDetectionMode;
                s.SolverVelocity = body.solverVelocityIterations;
                s.SolverPosition = body.solverIterations;
                s.Com = body.worldCenterOfMass;
                s.Velocity = body.velocity;
                s.W = body.angularVelocity;
                if (!V(s.Com) || !V(s.Velocity) || !V(s.W)) throw new InvalidOperationException("NON_FINITE_FOOT_SNAPSHOT");
                try { s.DetectCollisions = body.detectCollisions; s.DetectCollisionsRead = true; }
                catch (Exception e) { Invalid("DETECT_COLLISIONS_READ_EXCEPTION:" + e.GetType().Name); }
            }
            if (collider != null && ground != null)
            {
                try { s.PairIgnored = UnityEngine.Physics.GetIgnoreCollision(collider, ground); s.PairFilterRead = true; }
                catch (Exception e) { Invalid("PAIR_FILTER_READ_EXCEPTION:" + e.GetType().Name); }
                try { s.LayerIgnored = UnityEngine.Physics.GetIgnoreLayerCollision(collider.gameObject.layer, ground.gameObject.layer); s.LayerFilterRead = true; }
                catch (Exception e) { Invalid("LAYER_FILTER_READ_EXCEPTION:" + e.GetType().Name); }
                // Symmetric query checks argument-order/readback consistency only; it is not independent physics truth.
                try { s.ReverseLayerIgnored = UnityEngine.Physics.GetIgnoreLayerCollision(ground.gameObject.layer, collider.gameObject.layer); s.ReverseLayerFilterRead = true; }
                catch (Exception e) { Invalid("REVERSE_LAYER_FILTER_READ_EXCEPTION:" + e.GetType().Name); }
            }
            return s;
        }

        private static Probe ReadProbe(FootAnkleFoundationReport report) => new Probe
        {
            Id = report.GroundColliderInstanceId, Hits = report.GroundProbeHitCount,
            Ready = report.GroundGeometryReady, SameSurface = report.GroundFootprintSameSurface,
            MaxAbsClearance = Mathf.Max(Mathf.Abs(report.GroundClearanceMinMeters), Mathf.Abs(report.GroundClearanceMaxMeters)), Spread = report.GroundClearanceSpreadMeters,
            CenterClearance = report.GroundCenterClearanceMeters,
            MinClearance = report.GroundClearanceMinMeters, MaxClearance = report.GroundClearanceMaxMeters,
            NormalValid = report.GroundGeometryReady && report.GroundFootprintSameSurface && report.GroundProbeHitCount >= 3 &&
                V(report.GroundAverageNormalWorld) && report.GroundAverageNormalWorld.sqrMagnitude > 0.00000001f,
            Normal = V(report.GroundAverageNormalWorld) && report.GroundAverageNormalWorld.sqrMagnitude > 0.00000001f
                ? report.GroundAverageNormalWorld.normalized : Vector3.zero
        };

        private static bool Changed(Entity a, Entity b) =>
            a.Exists != b.Exists || a.Id != b.Id || a.Enabled != b.Enabled || a.ActiveSelf != b.ActiveSelf ||
            a.ActiveHierarchy != b.ActiveHierarchy || a.Trigger != b.Trigger || a.Layer != b.Layer || a.ParentId != b.ParentId ||
            a.AncestorAlive != b.AncestorAlive || a.AncestorSelf != b.AncestorSelf || a.AncestorHierarchy != b.AncestorHierarchy ||
            a.AncestorLinksMatch != b.AncestorLinksMatch ||
            (a.Position - b.Position).sqrMagnitude > 0.00000001f ||
            (a.BoundsCenter - b.BoundsCenter).sqrMagnitude > 0.00000001f || (a.BoundsExtents - b.BoundsExtents).sqrMagnitude > 0.00000001f ||
            (a.Scale - b.Scale).sqrMagnitude > 0.00000001f || (a.Exists && b.Exists && Quaternion.Angle(a.Rotation, b.Rotation) > 0.01f);

        private static bool FootBoundaryChanged(FootState a, FootState b) =>
            a.BodyExists != b.BodyExists || a.BodyId != b.BodyId || a.Kinematic != b.Kinematic || a.Gravity != b.Gravity ||
            a.CollisionMode != b.CollisionMode || a.SolverVelocity != b.SolverVelocity || a.SolverPosition != b.SolverPosition ||
            a.Collider.Exists != b.Collider.Exists || a.Collider.Id != b.Collider.Id ||
            a.Collider.Enabled != b.Collider.Enabled || a.Collider.ActiveSelf != b.Collider.ActiveSelf ||
            a.Collider.ActiveHierarchy != b.Collider.ActiveHierarchy || a.Collider.Layer != b.Collider.Layer || a.Collider.Trigger != b.Collider.Trigger ||
            a.Collider.ParentId != b.Collider.ParentId || (a.Collider.Scale - b.Collider.Scale).sqrMagnitude > 0.00000001f ||
            a.DetectCollisionsRead != b.DetectCollisionsRead || a.DetectCollisions != b.DetectCollisions ||
            a.PairFilterRead != b.PairFilterRead || a.PairIgnored != b.PairIgnored ||
            a.LayerFilterRead != b.LayerFilterRead || a.LayerIgnored != b.LayerIgnored ||
            a.ReverseLayerFilterRead != b.ReverseLayerFilterRead || a.ReverseLayerIgnored != b.ReverseLayerIgnored;

        private static bool PositionSolverCountsMatch(FootState s, int velocity) => s.BodyExists && velocity > 0 &&
            s.SolverVelocity == velocity && s.SolverPosition == FootSolverIterationBaselineContract.BPositionIterations;

        // A selected B or an unresolved last command is not an observed intervention. Match all four native boundaries.
        private static bool PositionSolverInterventionResolved(Frame f) => f.PositionSolverB && f.Loaded && f.Resolved && !f.Censored &&
            f.Enter + f.Stay > 0 && f.Projected > 0 && f.ProjectionInvalid == 0 &&
            PositionSolverCountsMatch(f.PreFoot, f.BaselineSolverVelocity) &&
            PositionSolverCountsMatch(f.FirstCallbackFoot, f.BaselineSolverVelocity) &&
            PositionSolverCountsMatch(f.LastCallbackFoot, f.BaselineSolverVelocity) &&
            PositionSolverCountsMatch(f.SettledFoot, f.BaselineSolverVelocity);

        private static bool CcdStateMatches(FootState s, int expected) =>
            s.BodyExists && !s.Kinematic && s.CollisionMode == expected;

        private static bool CcdModeReadbackMismatch(Frame f) => f.CcdExperimentSelected && f.Loaded && f.Resolved && !f.Censored &&
            (!CcdStateMatches(f.PreFoot, f.ExpectedCollisionMode) || !CcdStateMatches(f.SettledFoot, f.ExpectedCollisionMode) ||
             (f.Enter + f.Stay + f.Exit > 0 && (!CcdStateMatches(f.FirstCallbackFoot, f.ExpectedCollisionMode) ||
                 !CcdStateMatches(f.LastCallbackFoot, f.ExpectedCollisionMode))));

        private static bool CcdModeInterventionResolved(Frame f) => f.CcdExperimentSelected && f.Loaded && f.Resolved && !f.Censored &&
            f.Settled == f.Fixed + 1 && f.Enter + f.Stay > 0 && f.Projected > 0 && f.ProjectionInvalid == 0 &&
            f.PreFoot.BodyPointer != IntPtr.Zero && f.PreFoot.BodyPointer == f.SettledFoot.BodyPointer &&
            f.PreFoot.BodyPointer == f.FirstCallbackFoot.BodyPointer && f.PreFoot.BodyPointer == f.LastCallbackFoot.BodyPointer &&
            !CcdModeReadbackMismatch(f);

        private static bool OffsetBoundaryReadable(Entity ground, FootState foot) =>
            ground.Exists && ground.Id != 0 && ground.ContactOffsetRead && F(ground.ContactOffset) && ground.ContactOffset > 0f &&
            foot.Collider.Exists && foot.Collider.Id != 0 && foot.Collider.ContactOffsetRead &&
            F(foot.Collider.ContactOffset) && foot.Collider.ContactOffset > 0f;

        private static bool OffsetBoundaryIdentityMatches(Frame f, Entity ground, FootState foot) =>
            ground.Id == f.PreGround.Id && foot.Collider.Id == f.PreFoot.Collider.Id;

        private static bool OffsetBoundaryMatches(Frame f, Entity ground, FootState foot) =>
            FootContactOffsetIsolation.Matches(ground.ContactOffset, f.ExpectedGroundContactOffset) &&
            FootContactOffsetIsolation.Matches(foot.Collider.ContactOffset, f.ExpectedContactOffset);

        // Across-solve Ground switches have their own counter and per-ID baseline. Inside a solve,
        // changed owners and missing reads remain unverified; neither is labelled a numeric drift.
        private static ContactOffsetEvidence ClassifyContactOffsetEvidence(Frame f)
        {
            if (!f.ContactOffsetSelected || !f.Loaded || !f.Resolved || f.Censored)
                return ContactOffsetEvidence.NOT_APPLICABLE;
            bool callbacks = f.Enter + f.Stay + f.Exit > 0;
            if (!f.OffsetGroundBaselineKnown || !OffsetBoundaryReadable(f.PreGround, f.PreFoot) ||
                !OffsetBoundaryReadable(f.SettledGround, f.SettledFoot) ||
                (callbacks && (!OffsetBoundaryReadable(f.FirstCallbackGround, f.FirstCallbackFoot) ||
                    !OffsetBoundaryReadable(f.LastCallbackGround, f.LastCallbackFoot))))
                return ContactOffsetEvidence.READBACK_UNAVAILABLE;
            if (!OffsetBoundaryIdentityMatches(f, f.SettledGround, f.SettledFoot) || f.SettledAuthorizedId != f.PreGround.Id ||
                (callbacks && (f.FirstCallbackAuthorizedId != f.PreGround.Id || f.LastCallbackAuthorizedId != f.PreGround.Id ||
                    !OffsetBoundaryIdentityMatches(f, f.FirstCallbackGround, f.FirstCallbackFoot) ||
                    !OffsetBoundaryIdentityMatches(f, f.LastCallbackGround, f.LastCallbackFoot))))
                return ContactOffsetEvidence.ENTITY_CHANGED_WITHIN_SOLVE;
            if (!OffsetBoundaryMatches(f, f.PreGround, f.PreFoot) || !OffsetBoundaryMatches(f, f.SettledGround, f.SettledFoot) ||
                (callbacks && (!OffsetBoundaryMatches(f, f.FirstCallbackGround, f.FirstCallbackFoot) ||
                    !OffsetBoundaryMatches(f, f.LastCallbackGround, f.LastCallbackFoot))))
                return ContactOffsetEvidence.PARAMETER_MISMATCH;
            return ContactOffsetEvidence.MATCHED;
        }

        private static bool ContactOffsetReadbackMismatch(Frame f) =>
            ClassifyContactOffsetEvidence(f) == ContactOffsetEvidence.PARAMETER_MISMATCH;

        // R5 reuses the captured native states. Numeric offset agreement cannot establish support:
        // the same collider ID can survive while a room ancestor disables its collision presence.
        // Read failures remain unavailable, rather than being reported as an observed support loss.
        private static bool SupportEnvironmentReadsPresent(Entity ground, FootState foot) =>
            ground.AncestorsObserved && (!foot.BodyExists || foot.DetectCollisionsRead);

        private static bool SupportEnvironmentEnabled(Entity ground, FootState foot) =>
            ground.Exists && ground.Id != 0 && ground.Enabled && ground.ActiveSelf && ground.ActiveHierarchy && !ground.Trigger &&
            ground.AncestorLinksMatch && ground.AncestorAlive != 0 &&
            ground.AncestorSelf == ground.AncestorAlive && ground.AncestorHierarchy == ground.AncestorAlive &&
            foot.Collider.Exists && foot.Collider.Id != 0 && foot.Collider.Enabled &&
            foot.Collider.ActiveSelf && foot.Collider.ActiveHierarchy && !foot.Collider.Trigger &&
            foot.BodyExists && !foot.Kinematic && foot.DetectCollisions;

        private static ContactOffsetEnvironmentEvidence ClassifyContactOffsetEnvironment(Frame f)
        {
            if (!f.ContactOffsetSelected || !f.Loaded || !f.Resolved || f.Censored)
                return ContactOffsetEnvironmentEvidence.NOT_APPLICABLE;
            return ClassifyContactOffsetEnvironmentBoundary(f, EntityChanged(f));
        }

        private static ContactOffsetEnvironmentEvidence ClassifyContactOffsetEnvironmentBoundary(Frame f, bool entityChanged)
        {
            if (!f.ContactOffsetSelected || !f.Loaded || !f.Resolved || f.Censored)
                return ContactOffsetEnvironmentEvidence.NOT_APPLICABLE;
            bool callbacks = f.Enter + f.Stay + f.Exit > 0;
            if (!SupportEnvironmentReadsPresent(f.PreGround, f.PreFoot) ||
                !SupportEnvironmentReadsPresent(f.SettledGround, f.SettledFoot) ||
                (callbacks && (!SupportEnvironmentReadsPresent(f.FirstCallbackGround, f.FirstCallbackFoot) ||
                    !SupportEnvironmentReadsPresent(f.LastCallbackGround, f.LastCallbackFoot))))
                return ContactOffsetEnvironmentEvidence.SUPPORT_UNAVAILABLE_AT_BOUNDARY;
            if (!SupportEnvironmentEnabled(f.PreGround, f.PreFoot))
                return ContactOffsetEnvironmentEvidence.SUPPORT_UNAVAILABLE_AT_BOUNDARY;
            if (!SupportEnvironmentEnabled(f.SettledGround, f.SettledFoot) ||
                (callbacks && (!SupportEnvironmentEnabled(f.FirstCallbackGround, f.FirstCallbackFoot) ||
                    !SupportEnvironmentEnabled(f.LastCallbackGround, f.LastCallbackFoot))))
                return ContactOffsetEnvironmentEvidence.SUPPORT_LOST_WITHIN_SOLVE;
            if (entityChanged) return ContactOffsetEnvironmentEvidence.CHANGED_WITHIN_SOLVE;
            return ContactOffsetEnvironmentEvidence.STABLE_AT_CAPTURED_BOUNDARIES;
        }

        private static bool ContactOffsetEnvironmentExcluded(Frame f)
        {
            ContactOffsetEnvironmentEvidence environment = ClassifyContactOffsetEnvironment(f);
            return environment != ContactOffsetEnvironmentEvidence.NOT_APPLICABLE &&
                environment != ContactOffsetEnvironmentEvidence.STABLE_AT_CAPTURED_BOUNDARIES;
        }

        private void AccumulateContactOffsetEvidence(Frame f, bool entityChanged)
        {
            ContactOffsetEvidence offset = ClassifyContactOffsetEvidence(f);
            // Reuse EarlySettled's existing comparison; classification adds no native reads or
            // repeated entity/Quaternion comparisons to the per-solve accumulation path.
            ContactOffsetEnvironmentEvidence environment = ClassifyContactOffsetEnvironmentBoundary(f, entityChanged);
            if (ContactOffsetSolveQualified(f, offset, environment)) _offsetResolved++;
            if (offset == ContactOffsetEvidence.PARAMETER_MISMATCH) _offsetMismatch++;
            // Count a changed boundary once, even when identity and activation both changed.
            if (offset == ContactOffsetEvidence.ENTITY_CHANGED_WITHIN_SOLVE ||
                (environment != ContactOffsetEnvironmentEvidence.NOT_APPLICABLE && entityChanged)) _offsetBoundaryChanged++;
            if (offset == ContactOffsetEvidence.READBACK_UNAVAILABLE) _offsetReadUnavailable++;
            if (environment != ContactOffsetEnvironmentEvidence.NOT_APPLICABLE &&
                environment != ContactOffsetEnvironmentEvidence.STABLE_AT_CAPTURED_BOUNDARIES) _offsetEnvironmentExcluded++;
            if (environment == ContactOffsetEnvironmentEvidence.SUPPORT_LOST_WITHIN_SOLVE) _offsetSupportLost++;
        }

        private static bool ContactOffsetSolveQualified(Frame f, ContactOffsetEvidence offset, ContactOffsetEnvironmentEvidence environment) =>
            f.ContactOffsetSelected &&
            CcdModeInterventionResolved(f) && PositionSolverInterventionResolved(f) &&
            offset == ContactOffsetEvidence.MATCHED && environment == ContactOffsetEnvironmentEvidence.STABLE_AT_CAPTURED_BOUNDARIES;

        private static bool ContactOffsetInterventionResolved(Frame f) =>
            ContactOffsetSolveQualified(f, ClassifyContactOffsetEvidence(f), ClassifyContactOffsetEnvironment(f));

        private static string ContactOffsetFrameSummary(Frame f) =>
            "contactOffsetArm=" + f.ContactOffsetArm + "/selectedMm=" + S(f.ExpectedContactOffset * 1000f) +
            "/observedResolved=" + ContactOffsetInterventionResolved(f) + "/readbackMismatch=" + ContactOffsetReadbackMismatch(f) +
            "/evidence=" + ClassifyContactOffsetEvidence(f) + "/evidenceScope=PARAMETER_VALUES_AND_OWNERS_ONLY" +
            "/environmentEvidence=" + ClassifyContactOffsetEnvironment(f) +
            "/environmentExcluded=" + ContactOffsetEnvironmentExcluded(f) +
            "/groundBaselineKnown-mm=" + f.OffsetGroundBaselineKnown + "-" +
            (f.OffsetGroundBaselineKnown ? S(f.ExpectedGroundContactOffset * 1000f) : "NA") +
            "/groundSwitchAcrossLoadedSolves=" + f.OffsetGroundSwitched;

        private static bool EntityChanged(Frame f) => Changed(f.PreGround, f.SettledGround) || FootBoundaryChanged(f.PreFoot, f.SettledFoot) ||
            f.Ground.Id != f.SettledAuthorizedId || f.PreContact != f.SettledContact ||
            ((f.Enter + f.Stay + f.Exit > 0) && (f.FirstCallbackAuthorizedId != f.Ground.Id || f.LastCallbackAuthorizedId != f.Ground.Id ||
                Changed(f.PreGround, f.FirstCallbackGround) || Changed(f.PreGround, f.LastCallbackGround) ||
                FootBoundaryChanged(f.PreFoot, f.FirstCallbackFoot) || FootBoundaryChanged(f.PreFoot, f.LastCallbackFoot)));

        private static bool BothLayersIgnored(FootState s) => s.LayerFilterRead && s.LayerIgnored && s.ReverseLayerFilterRead && s.ReverseLayerIgnored;
        private static bool LayerReadReactionContradiction(Frame f) => f.Projected > 0 && f.GroundJ.sqrMagnitude > 0.0000000001f &&
            BothLayersIgnored(f.PreFoot) && BothLayersIgnored(f.FirstCallbackFoot) && BothLayersIgnored(f.LastCallbackFoot) &&
            f.Resolved && BothLayersIgnored(f.SettledFoot);

        private static string Row(Frame f)
        {
            string row = "pre-earlySettled=" + f.Fixed + "-" + f.Settled + " | loaded-resolved-censored=" + f.Loaded + "-" + f.Resolved + "-" + f.Censored +
                " | positionSolverB-selected-resolved=" + f.PositionSolverB + "-" + PositionSolverInterventionResolved(f) +
                " | " + ContactOffsetFrameSummary(f) +
                " | render-fresh-held-age=" + f.Render + "-" + f.Fresh + "-" + f.Held + "-" + S(f.Age) + "/dt=" + S(f.Dt) +
                " | exactGroundId=" + f.Ground.Id + "/path=" + f.Ground.Path + "/pathTruncated=" + f.Ground.Truncated +
                " | preContact=" + f.PreContact + " | preGround={" + E(f.PreGround) + "} | preFoot={" + B(f.PreFoot) + "} | preProbe={" + P(f.PreProbe) + "}" +
                " | callbacks(ENTER-STAY-EXIT-projected-invalid)=" + f.Enter + "-" + f.Stay + "-" + f.Exit + "-" + f.Projected + "-" + f.ProjectionInvalid + "/GroundJ=" + W(f.GroundJ);
            if (f.Enter + f.Stay + f.Exit > 0) row += " | callbackAuthorizedId(first-last)=" + f.FirstCallbackAuthorizedId + "-" + f.LastCallbackAuthorizedId +
                " | firstCallbackGround={" + E(f.FirstCallbackGround) + "}/Foot={" + B(f.FirstCallbackFoot) +
                "} | lastCallbackGround={" + E(f.LastCallbackGround) + "}/Foot={" + B(f.LastCallbackFoot) + "}";
            if (!f.Resolved) return row;
            return row + " | earlySettledContact=" + f.SettledContact + "/authorizedId=" + f.SettledAuthorizedId +
                " | earlySettledGround={" + E(f.SettledGround) + "}/Foot={" + B(f.SettledFoot) + "} | entitySnapshotChanged=" + EntityChanged(f) +
                " | layerReadReactionContradiction=" + LayerReadReactionContradiction(f) + "/nativeLayerSemantics=UNQUALIFIED" +
                " | settledProbeRecorded=" + f.ProbeRecorded + "/probe={" + P(f.SettledProbe) + "}";
        }

        private static string E(Entity s) => "exists-id=" + s.Exists + "-" + s.Id + "/enabled-self-hierarchy-trigger=" + s.Enabled + "-" + s.ActiveSelf + "-" + s.ActiveHierarchy + "-" + s.Trigger +
            "/layer-parentId=" + s.Layer + "-" + s.ParentId +
            (s.AncestorsObserved ? "/ancestorMasks(alive-self-hierarchy)=" + s.AncestorAlive.ToString("X") + "-" + s.AncestorSelf.ToString("X") + "-" + s.AncestorHierarchy.ToString("X") +
                "/ancestorLinksMatch=" + s.AncestorLinksMatch : "/ancestors=NOT_OBSERVED") + "/position=" + W(s.Position) + "/rotation=" + R(s.Rotation) + "/scale=" + W(s.Scale) + "/boundsCenter=" + W(s.BoundsCenter) + "/extents=" + W(s.BoundsExtents) +
            "/contactOffsetRead-mm=" + s.ContactOffsetRead + "-" + (s.ContactOffsetRead ? S(s.ContactOffset * 1000f) : "NA");
        private static string B(FootState s) => "collider={" + E(s.Collider) + "}/bodyExists-id=" + s.BodyExists + "-" + s.BodyId +
            "/solver(velocity-position)=" + s.SolverVelocity + "-" + s.SolverPosition +
            "/gravity-kinematic-CCD=" + s.Gravity + "-" + s.Kinematic + "-" + s.CollisionMode + "/pairFilterRead-ignored=" + s.PairFilterRead + "-" + s.PairIgnored +
            "/detectCollisionsRead-enabled=" + s.DetectCollisionsRead + "-" + s.DetectCollisions + "/layerFilterRead-ignored=" + s.LayerFilterRead + "-" + s.LayerIgnored +
            "/reverseLayerFilterRead-ignored=" + s.ReverseLayerFilterRead + "-" + s.ReverseLayerIgnored +
            "/layerReadSymmetric=" + (s.LayerFilterRead && s.ReverseLayerFilterRead ? (s.LayerIgnored == s.ReverseLayerIgnored ? "True" : "False") : "UNKNOWN") +
            "/COM=" + W(s.Com) + "/V=" + W(s.Velocity) + "/W=" + W(s.W);
        private static string P(Probe s) => "id-hits-ready-same=" + s.Id + "-" + s.Hits + "-" + s.Ready + "-" + s.SameSurface +
            "/maxAbs-spreadMm=" + S(s.MaxAbsClearance * 1000f) + "-" + S(s.Spread * 1000f) +
            "/center-min-maxMm=" + (s.Hits > 0 && F(s.CenterClearance) ? S(s.CenterClearance * 1000f) : "NA") + "-" +
            (s.Hits > 0 && F(s.MinClearance) ? S(s.MinClearance * 1000f) : "NA") + "-" +
            (s.Hits > 0 && F(s.MaxClearance) ? S(s.MaxClearance * 1000f) : "NA") +
            "/normalValid=" + s.NormalValid + "/N=" + W(s.Normal);
        private void Invalid(string reason) { _finite = false; if (_invalidReason == null) _invalidReason = reason; }
        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z);
        private static bool Q(Quaternion q) => F(q.x) && F(q.y) && F(q.z) && F(q.w);
        private static string S(float x) => x.ToString("G9", CultureInfo.InvariantCulture);
        private static string W(Vector3 v) => "(" + S(v.x) + "," + S(v.y) + "," + S(v.z) + ")";
        private static string R(Quaternion q) => "(" + S(q.x) + "," + S(q.y) + "," + S(q.z) + "," + S(q.w) + ")";
    }
}
