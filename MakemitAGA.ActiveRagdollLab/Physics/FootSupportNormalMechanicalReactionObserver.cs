using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.3.2 read-only lateral-probe alternate-collider attribution hotfix on the v0.4.3 mechanical-reaction attribution.
    ///
    /// v0.4.2.1 proved that Collision.impulse, Collision.relativeVelocity and callback-time Rigidbody state are
    /// readable and stable on the exact authorized support pair, while Collision.GetContact(int) throws
    /// MissingMethodException on this Unity 2021.3 IL2CPP/Il2CppInterop runtime. v0.4.3 therefore removed that
    /// dead ContactPoint path instead of treating an interop limitation as a physical failure. v0.4.3.1 then proved
    /// support-normal gaps are real geometry/footprint episodes rather than ray-miss or normal-number failures. v0.4.3.2
    /// keeps the same physical path and strict qualification rule, and attributes each such episode to the exact five
    /// already-executed sole probes plus any alternate Collider identity they saw.
    ///
    /// The support normal is supplied by the already-qualified five-point sole raycast witness. The observer only
    /// accepts it when the current sole geometry is ready, all footprint points belong to one surface, and that
    /// surface is the exact collider already authorized by the v0.4.1 contact-shadow gate. Collision.impulse is then
    /// decomposed into signed support-normal and tangential components. No force/torque/GRF is injected.
    ///
    /// The pre-physics -> callback delta remains a response witness, not a pure solver-only delta, because the same
    /// physics step also integrates the frozen Foot controller and drag. Collision.impulse remains the direct solver
    /// witness. Qualification in this observer means the attribution dataset is internally valid and reviewable; it
    /// does NOT mean load-bearing, friction, balance, CoP, or gravity-standing production is ready.
    /// </summary>
    internal sealed class FootSupportNormalMechanicalReactionObserver
    {
        internal const string Mode = "STATIC_SUPPORT_NORMAL_MECHANICAL_REACTION_ATTRIBUTION_V0430_HOTFIX2_LATERAL_PROBE_ALTERNATE_COLLIDER_ATTRIBUTION";
        internal const int MinSupportNormalSamples = 5;
        internal const int MinProjectionSamples = 5;
        internal const int MinImpulseReadSamples = 5;
        internal const int MinRelativeVelocityReadSamples = 5;
        internal const int MinPrePhysicsResponseSamples = 5;
        internal const int MinNonZeroImpulseSamples = 1;
        internal const float NonZeroImpulseEpsilon = 0.000001f;
        private const int MaxInvalidEpisodeRecords = 24;
        private const int MaxAlternateColliderRecords = 12;

        private struct ProbeAttributionSample
        {
            internal int CallbackIndex;
            internal int PreFixedSequence;
            internal int CenterId;
            internal int ToeId;
            internal int HeelId;
            internal int LeftId;
            internal int RightId;
            internal bool CenterHit;
            internal bool ToeHit;
            internal bool HeelHit;
            internal bool LeftHit;
            internal bool RightHit;
            internal float CenterClearance;
            internal float ToeClearance;
            internal float HeelClearance;
            internal float LeftClearance;
            internal float RightClearance;
            internal float MaxAbsClearance;
            internal float Spread;
            internal bool LeftDiffersFromCenter;
            internal bool RightDiffersFromCenter;
            internal bool ToeDiffersFromCenter;
            internal bool HeelDiffersFromCenter;
            internal bool PoseValid;
            internal Vector3 PhysicalPosition;
            internal Vector3 TargetSolePosition;
            internal Vector3 PhysicalEulerDegrees;
            internal Vector3 TargetSoleEulerDegrees;
            internal float PhysicalTargetPositionErrorMeters;
            internal float PhysicalTargetRotationErrorDegrees;
        }

        private struct InvalidEpisodeRecord
        {
            internal int EpisodeIndex;
            internal ProbeAttributionSample First;
            internal ProbeAttributionSample PeakSpread;
            internal ProbeAttributionSample Last;
        }

        private bool _finite = true;
        private string _invalidReason;
        private string _lastReadFailureStage;
        private string _lastReadFailureType;
        private string _lastReadFailureMessage;

        private bool _preValid;
        private int _preFixedSequence;
        private float _preDt;
        private Vector3 _prePosition;
        private Vector3 _preVelocity;
        private Vector3 _preAngularVelocity;

        private int _traceCallbacks;
        private int _traceSupportNormalSamples;
        private int _traceSupportNormalInvalidSamples;
        private int _traceSupportColliderMismatchSamples;

        // v0.4.3.1 diagnostic-only decomposition. Primary causes are mutually exclusive and must sum
        // exactly to _traceSupportNormalInvalidSamples. Predicate counters are intentionally non-exclusive.
        private int _traceSupportInvalidAuthorizedIdUnavailable;
        private int _traceSupportInvalidSupportIdUnavailable;
        private int _traceSupportInvalidIdMismatch;
        private int _traceSupportInvalidGeometryNotReady;
        private int _traceSupportInvalidFootprintNotSameSurface;
        private int _traceSupportInvalidNormalNonFiniteOrZero;
        private int _traceSupportPredicateGeometryNotReady;
        private int _traceSupportPredicateFootprintNotSameSurface;
        private int _traceSupportPredicateNormalNonFiniteOrZero;
        private int _traceSupportPredicateHitCountLt5;
        private int _traceSupportPredicateHitCountLt3;
        private int _traceSupportPredicatePrimarySurfaceNotSame;
        private int _traceSupportPredicateClearanceGateFail;
        private int _traceSupportPredicateSpreadGateFail;
        private int _traceSupportPredicateNormalDotGateFail;
        private int _traceSupportInvalidEnter;
        private int _traceSupportInvalidStay;
        private int _traceSupportInvalidOtherPhase;
        private int _traceSupportInvalidEpisodes;
        private int _traceSupportInvalidCurrentStreak;
        private int _traceSupportInvalidMaxStreak;
        private int _traceSupportInvalidFirstCallbackIndex;
        private int _traceSupportInvalidLastCallbackIndex;
        private int _traceSupportInvalidFirstPreFixedSequence;
        private int _traceSupportInvalidLastPreFixedSequence;
        private bool _previousSupportValidityKnown;
        private bool _previousSupportValid;

        // v0.4.3.2 read-only attribution. These counters classify which lateral probe diverged from
        // the center/primary support surface. Episode records retain first/peak-spread/last snapshots only.
        private int _traceInvalidLateralLeftOnlySamples;
        private int _traceInvalidLateralRightOnlySamples;
        private int _traceInvalidLateralBothSamples;
        private int _traceInvalidNoLateralDivergenceSamples;
        private int _traceInvalidPrimaryProbeDivergenceSamples;
        private readonly int[] _alternateColliderIds = new int[MaxAlternateColliderRecords];
        private readonly int[] _alternateColliderCounts = new int[MaxAlternateColliderRecords];
        private readonly string[] _alternateColliderPaths = new string[MaxAlternateColliderRecords];
        private int _alternateColliderRecordCount;
        private int _alternateColliderOverflowCount;
        private string _alternateColliderCatalogCache = "<none>";
        private readonly InvalidEpisodeRecord[] _invalidEpisodeRecords = new InvalidEpisodeRecord[MaxInvalidEpisodeRecords];
        private int _invalidEpisodeRecordCount;
        private int _invalidEpisodeRecordOverflowCount;
        private bool _invalidEpisodeActive;
        private InvalidEpisodeRecord _activeInvalidEpisode;
        private string _invalidEpisodeAttributionCache = "<none>";

        private int _traceProjectionSamples;
        private int _traceReadFailures;
        private int _tracePrePhysicsSamples;
        private int _traceResponseSamples;
        private int _traceImpulseReadSamples;
        private int _traceImpulseReadFailures;
        private int _traceRelativeVelocityReadSamples;
        private int _traceRelativeVelocityReadFailures;
        private int _tracePostBodyReadSamples;
        private int _tracePostBodyReadFailures;
        private int _traceNonZeroImpulseSamples;
        private int _traceAlignedImpulseSamples;
        private int _traceOpposedImpulseSamples;
        private int _traceNearZeroImpulseSamples;
        private int _traceImpulseDeltaVComparisonSamples;
        private int _traceRelativeNormalProjectionSamples;

        private float _impulseMagnitudeSum;
        private float _signedNormalImpulseSum;
        private float _absNormalImpulseSum;
        private float _tangentImpulseMagnitudeSum;
        private float _normalShareAbsSum;
        private float _tangentShareSum;
        private float _relativeSpeedSum;
        private float _signedRelativeNormalSpeedSum;
        private float _relativeNormalSpeedAbsSum;
        private float _observedDeltaVSum;
        private float _impulseDeltaVSum;
        private float _deltaVCosAbsSum;
        private int _deltaVCosSamples;
        private float _observedDeltaWSum;
        private float _positionResidualSum;

        private float _traceImpulseMagnitudeMax;
        private float _traceSignedNormalImpulseMin;
        private float _traceSignedNormalImpulseMax;
        private bool _signedNormalImpulseInitialized;
        private float _traceAbsNormalImpulseMax;
        private float _traceTangentImpulseMagnitudeMax;
        private float _traceNormalShareAbsMin = 1f;
        private float _traceTangentShareMax;
        private float _traceRelativeSpeedMax;
        private float _traceRelativeNormalSpeedAbsMax;
        private float _traceObservedDeltaVMax;
        private float _traceImpulseDeltaVMax;
        private float _traceDeltaVCosAbsMin = 1f;
        private float _traceObservedDeltaWMax;
        private float _tracePositionResidualMax;

        private string _lastPhase;
        private Vector3 _lastSupportNormal;
        private Vector3 _lastImpulse;
        private Vector3 _lastRelativeVelocity;
        private float _lastSignedNormalImpulse;
        private float _lastTangentImpulse;
        private bool _lastProjectionValidThisCallback;
        private float _lastNormalShareAbs;
        private Vector3 _lastObservedDeltaV;
        private Vector3 _lastObservedDeltaW;
        private float _lastPositionResidual;
        private int _lastCollisionPreFixedSequence;

        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;

        // v0.4.4.1 read-only diagnostic taps. The sequence is latched at collision-callback time so a
        // subsequent CapturePrePhysics call cannot make a previous callback look like the current step.
        internal Vector3 LastImpulseWorld => _lastImpulse;
        internal Vector3 LastSupportNormalWorld => _lastSupportNormal;
        internal float LastSignedNormalImpulse => _lastSignedNormalImpulse;
        internal float LastTangentImpulse => _lastTangentImpulse;
        internal bool LastProjectionValidThisCallback => _lastProjectionValidThisCallback;
        internal int LastCollisionPreFixedSequence => _lastCollisionPreFixedSequence;
        internal string LastPhase => _lastPhase;

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _lastReadFailureStage = null;
            _lastReadFailureType = null;
            _lastReadFailureMessage = null;

            _preValid = false;
            _preFixedSequence = 0;
            _preDt = 0f;
            _prePosition = Vector3.zero;
            _preVelocity = Vector3.zero;
            _preAngularVelocity = Vector3.zero;

            _traceCallbacks = 0;
            _traceSupportNormalSamples = 0;
            _traceSupportNormalInvalidSamples = 0;
            _traceSupportColliderMismatchSamples = 0;
            _traceSupportInvalidAuthorizedIdUnavailable = 0;
            _traceSupportInvalidSupportIdUnavailable = 0;
            _traceSupportInvalidIdMismatch = 0;
            _traceSupportInvalidGeometryNotReady = 0;
            _traceSupportInvalidFootprintNotSameSurface = 0;
            _traceSupportInvalidNormalNonFiniteOrZero = 0;
            _traceSupportPredicateGeometryNotReady = 0;
            _traceSupportPredicateFootprintNotSameSurface = 0;
            _traceSupportPredicateNormalNonFiniteOrZero = 0;
            _traceSupportPredicateHitCountLt5 = 0;
            _traceSupportPredicateHitCountLt3 = 0;
            _traceSupportPredicatePrimarySurfaceNotSame = 0;
            _traceSupportPredicateClearanceGateFail = 0;
            _traceSupportPredicateSpreadGateFail = 0;
            _traceSupportPredicateNormalDotGateFail = 0;
            _traceSupportInvalidEnter = 0;
            _traceSupportInvalidStay = 0;
            _traceSupportInvalidOtherPhase = 0;
            _traceSupportInvalidEpisodes = 0;
            _traceSupportInvalidCurrentStreak = 0;
            _traceSupportInvalidMaxStreak = 0;
            _traceSupportInvalidFirstCallbackIndex = 0;
            _traceSupportInvalidLastCallbackIndex = 0;
            _traceSupportInvalidFirstPreFixedSequence = 0;
            _traceSupportInvalidLastPreFixedSequence = 0;
            _previousSupportValidityKnown = false;
            _previousSupportValid = false;
            _traceInvalidLateralLeftOnlySamples = 0;
            _traceInvalidLateralRightOnlySamples = 0;
            _traceInvalidLateralBothSamples = 0;
            _traceInvalidNoLateralDivergenceSamples = 0;
            _traceInvalidPrimaryProbeDivergenceSamples = 0;
            Array.Clear(_alternateColliderIds, 0, _alternateColliderIds.Length);
            Array.Clear(_alternateColliderCounts, 0, _alternateColliderCounts.Length);
            Array.Clear(_alternateColliderPaths, 0, _alternateColliderPaths.Length);
            _alternateColliderRecordCount = 0;
            _alternateColliderOverflowCount = 0;
            _alternateColliderCatalogCache = "<none>";
            Array.Clear(_invalidEpisodeRecords, 0, _invalidEpisodeRecords.Length);
            _invalidEpisodeRecordCount = 0;
            _invalidEpisodeRecordOverflowCount = 0;
            _invalidEpisodeActive = false;
            _activeInvalidEpisode = new InvalidEpisodeRecord();
            _invalidEpisodeAttributionCache = "<none>";
            _traceProjectionSamples = 0;
            _traceReadFailures = 0;
            _tracePrePhysicsSamples = 0;
            _traceResponseSamples = 0;
            _traceImpulseReadSamples = 0;
            _traceImpulseReadFailures = 0;
            _traceRelativeVelocityReadSamples = 0;
            _traceRelativeVelocityReadFailures = 0;
            _tracePostBodyReadSamples = 0;
            _tracePostBodyReadFailures = 0;
            _traceNonZeroImpulseSamples = 0;
            _traceAlignedImpulseSamples = 0;
            _traceOpposedImpulseSamples = 0;
            _traceNearZeroImpulseSamples = 0;
            _traceImpulseDeltaVComparisonSamples = 0;
            _traceRelativeNormalProjectionSamples = 0;

            _impulseMagnitudeSum = 0f;
            _signedNormalImpulseSum = 0f;
            _absNormalImpulseSum = 0f;
            _tangentImpulseMagnitudeSum = 0f;
            _normalShareAbsSum = 0f;
            _tangentShareSum = 0f;
            _relativeSpeedSum = 0f;
            _signedRelativeNormalSpeedSum = 0f;
            _relativeNormalSpeedAbsSum = 0f;
            _observedDeltaVSum = 0f;
            _impulseDeltaVSum = 0f;
            _deltaVCosAbsSum = 0f;
            _deltaVCosSamples = 0;
            _observedDeltaWSum = 0f;
            _positionResidualSum = 0f;

            _traceImpulseMagnitudeMax = 0f;
            _traceSignedNormalImpulseMin = 0f;
            _traceSignedNormalImpulseMax = 0f;
            _signedNormalImpulseInitialized = false;
            _traceAbsNormalImpulseMax = 0f;
            _traceTangentImpulseMagnitudeMax = 0f;
            _traceNormalShareAbsMin = 1f;
            _traceTangentShareMax = 0f;
            _traceRelativeSpeedMax = 0f;
            _traceRelativeNormalSpeedAbsMax = 0f;
            _traceObservedDeltaVMax = 0f;
            _traceImpulseDeltaVMax = 0f;
            _traceDeltaVCosAbsMin = 1f;
            _traceObservedDeltaWMax = 0f;
            _tracePositionResidualMax = 0f;

            _lastPhase = null;
            _lastSupportNormal = Vector3.zero;
            _lastImpulse = Vector3.zero;
            _lastRelativeVelocity = Vector3.zero;
            _lastSignedNormalImpulse = 0f;
            _lastTangentImpulse = 0f;
            _lastNormalShareAbs = 0f;
            _lastObservedDeltaV = Vector3.zero;
            _lastObservedDeltaW = Vector3.zero;
            _lastPositionResidual = 0f;
            _lastCollisionPreFixedSequence = 0;
            CopyToReport(report);
        }

        internal void CapturePrePhysics(
            bool traceEnabled,
            bool contactShadowActive,
            Rigidbody body,
            float fixedDeltaTime,
            int fixedSequence,
            FootAnkleFoundationReport report)
        {
            _preValid = false;
            if (!traceEnabled || !contactShadowActive || body == null || body.isKinematic)
            {
                CopyToReport(report);
                return;
            }

            try
            {
                Vector3 position = body.position;
                Vector3 velocity = body.velocity;
                Vector3 angularVelocity = body.angularVelocity;
                if (!IsFinite(position) || !IsFinite(velocity) || !IsFinite(angularVelocity) || !IsFinite(fixedDeltaTime) || fixedDeltaTime <= 0f)
                {
                    InvalidateNumeric("NON_FINITE_PRE_PHYSICS_SAMPLE");
                    CopyToReport(report);
                    return;
                }

                _preValid = true;
                _preFixedSequence = fixedSequence;
                _preDt = fixedDeltaTime;
                _prePosition = position;
                _preVelocity = velocity;
                _preAngularVelocity = angularVelocity;
                _tracePrePhysicsSamples++;
            }
            catch (Exception e)
            {
                RecordReadFailure("PRE_PHYSICS_RIGIDBODY", e);
            }
            CopyToReport(report);
        }

        internal void ObserveCollision(
            bool traceEnabled,
            bool contactShadowActive,
            Rigidbody body,
            Vector3 targetSolePosition,
            Quaternion targetSoleRotation,
            Collider authorizedCollider,
            Vector3 supportNormalWorld,
            bool supportGeometryReady,
            bool supportFootprintSameSurface,
            int supportColliderInstanceId,
            int supportHitCount,
            bool supportPrimarySameSurface,
            float supportMaxAbsClearanceMeters,
            float supportClearanceSpreadMeters,
            float supportNormalDotReferenceUp,
            float supportReadyAbsClearanceMeters,
            float supportReadySpreadMeters,
            float supportNormalDotMin,
            FootSoleProbeSnapshot probeSnapshot,
            string phase,
            Collision collision,
            FootAnkleFoundationReport report)
        {
            _lastProjectionValidThisCallback = false;
            if (!traceEnabled || !contactShadowActive || body == null || collision == null)
            {
                CopyToReport(report);
                return;
            }

            _traceCallbacks++;
            _lastPhase = phase;
            _lastCollisionPreFixedSequence = _preFixedSequence;

            if (string.Equals(phase, "EXIT", StringComparison.Ordinal))
            {
                CloseInvalidEpisode(report, "CONTACT_EXIT");
                _lastImpulse = Vector3.zero;
                _lastRelativeVelocity = Vector3.zero;
                _lastSignedNormalImpulse = 0f;
                _lastTangentImpulse = 0f;
                _lastNormalShareAbs = 0f;
                // A later ENTER starts a new timing episode even if the previous contact ended while the
                // support witness was invalid. EXIT itself is not a support-normal observation.
                _previousSupportValidityKnown = false;
                _traceSupportInvalidCurrentStreak = 0;
                CopyToReport(report);
                return;
            }

            int authorizedId = authorizedCollider != null ? authorizedCollider.GetInstanceID() : 0;
            bool authorizedIdAvailable = authorizedId != 0;
            bool supportIdAvailable = supportColliderInstanceId != 0;
            bool supportIdMatches = authorizedIdAvailable && supportIdAvailable && supportColliderInstanceId == authorizedId;
            bool normalFiniteAndNonZero = IsFinite(supportNormalWorld) && supportNormalWorld.sqrMagnitude > 0.00000001f;
            bool supportNormalValid = supportGeometryReady && supportFootprintSameSurface && supportIdMatches && normalFiniteAndNonZero;

            // Raw predicate diagnostics are deliberately non-exclusive.
            if (!supportGeometryReady) _traceSupportPredicateGeometryNotReady++;
            if (!supportFootprintSameSurface) _traceSupportPredicateFootprintNotSameSurface++;
            if (!normalFiniteAndNonZero) _traceSupportPredicateNormalNonFiniteOrZero++;
            if (supportHitCount < 5) _traceSupportPredicateHitCountLt5++;
            if (supportHitCount < 3) _traceSupportPredicateHitCountLt3++;
            if (!supportPrimarySameSurface) _traceSupportPredicatePrimarySurfaceNotSame++;
            if (!IsFinite(supportMaxAbsClearanceMeters) || supportMaxAbsClearanceMeters > supportReadyAbsClearanceMeters)
                _traceSupportPredicateClearanceGateFail++;
            if (!IsFinite(supportClearanceSpreadMeters) || supportClearanceSpreadMeters > supportReadySpreadMeters)
                _traceSupportPredicateSpreadGateFail++;
            if (!IsFinite(supportNormalDotReferenceUp) || supportNormalDotReferenceUp < supportNormalDotMin)
                _traceSupportPredicateNormalDotGateFail++;

            if (supportNormalValid)
            {
                if (_previousSupportValidityKnown && !_previousSupportValid)
                    CloseInvalidEpisode(report, "RECOVERED");
                _lastSupportNormal = supportNormalWorld.normalized;
                _traceSupportNormalSamples++;
                _traceSupportInvalidCurrentStreak = 0;
            }
            else
            {
                _traceSupportNormalInvalidSamples++;
                ProbeAttributionSample attribution = CaptureProbeAttribution(probeSnapshot, body, targetSolePosition, targetSoleRotation);
                RecordProbeDivergence(attribution, probeSnapshot);

                // Primary-cause precedence makes this decomposition mutually exclusive. Keep ID authority first,
                // then geometry/footprint, then the normal vector itself.
                if (!authorizedIdAvailable)
                    _traceSupportInvalidAuthorizedIdUnavailable++;
                else if (!supportIdAvailable)
                    _traceSupportInvalidSupportIdUnavailable++;
                else if (!supportIdMatches)
                {
                    _traceSupportInvalidIdMismatch++;
                    _traceSupportColliderMismatchSamples++;
                }
                else if (!supportGeometryReady)
                    _traceSupportInvalidGeometryNotReady++;
                else if (!supportFootprintSameSurface)
                    _traceSupportInvalidFootprintNotSameSurface++;
                else
                    _traceSupportInvalidNormalNonFiniteOrZero++;

                if (string.Equals(phase, "ENTER", StringComparison.Ordinal)) _traceSupportInvalidEnter++;
                else if (string.Equals(phase, "STAY", StringComparison.Ordinal)) _traceSupportInvalidStay++;
                else _traceSupportInvalidOtherPhase++;

                if (!_previousSupportValidityKnown || _previousSupportValid)
                {
                    _traceSupportInvalidEpisodes++;
                    BeginInvalidEpisode(attribution);
                }
                else
                {
                    UpdateInvalidEpisode(attribution);
                }
                _traceSupportInvalidCurrentStreak++;
                _traceSupportInvalidMaxStreak = Math.Max(_traceSupportInvalidMaxStreak, _traceSupportInvalidCurrentStreak);
                if (_traceSupportInvalidFirstCallbackIndex == 0)
                {
                    _traceSupportInvalidFirstCallbackIndex = _traceCallbacks;
                    _traceSupportInvalidFirstPreFixedSequence = _preFixedSequence;
                }
                _traceSupportInvalidLastCallbackIndex = _traceCallbacks;
                _traceSupportInvalidLastPreFixedSequence = _preFixedSequence;
            }
            _previousSupportValidityKnown = true;
            _previousSupportValid = supportNormalValid;

            Vector3 impulse = Vector3.zero;
            bool impulseValid = false;
            try
            {
                impulse = collision.impulse;
                if (!IsFinite(impulse))
                {
                    InvalidateNumeric("NON_FINITE_COLLISION_IMPULSE");
                }
                else
                {
                    impulseValid = true;
                    _traceImpulseReadSamples++;
                    _lastImpulse = impulse;
                    float impulseMag = impulse.magnitude;
                    _impulseMagnitudeSum += impulseMag;
                    _traceImpulseMagnitudeMax = Mathf.Max(_traceImpulseMagnitudeMax, impulseMag);
                    if (impulseMag > NonZeroImpulseEpsilon)
                        _traceNonZeroImpulseSamples++;
                }
            }
            catch (Exception e)
            {
                _traceImpulseReadFailures++;
                RecordReadFailure("COLLISION_IMPULSE", e);
            }

            Vector3 relativeVelocity = Vector3.zero;
            bool relativeVelocityValid = false;
            try
            {
                relativeVelocity = collision.relativeVelocity;
                if (!IsFinite(relativeVelocity))
                {
                    InvalidateNumeric("NON_FINITE_COLLISION_RELATIVE_VELOCITY");
                }
                else
                {
                    relativeVelocityValid = true;
                    _traceRelativeVelocityReadSamples++;
                    _lastRelativeVelocity = relativeVelocity;
                    float relativeSpeed = relativeVelocity.magnitude;
                    _relativeSpeedSum += relativeSpeed;
                    _traceRelativeSpeedMax = Mathf.Max(_traceRelativeSpeedMax, relativeSpeed);
                }
            }
            catch (Exception e)
            {
                _traceRelativeVelocityReadFailures++;
                RecordReadFailure("COLLISION_RELATIVE_VELOCITY", e);
            }

            if (supportNormalValid && impulseValid)
            {
                _lastProjectionValidThisCallback = true;
                float impulseMag = impulse.magnitude;
                float signedNormalImpulse = Vector3.Dot(impulse, _lastSupportNormal);
                float absNormalImpulse = Mathf.Abs(signedNormalImpulse);
                Vector3 tangentImpulseVector = impulse - _lastSupportNormal * signedNormalImpulse;
                float tangentImpulse = tangentImpulseVector.magnitude;
                float normalShareAbs = impulseMag > NonZeroImpulseEpsilon ? Mathf.Clamp01(absNormalImpulse / impulseMag) : 0f;
                float tangentShare = impulseMag > NonZeroImpulseEpsilon ? Mathf.Clamp01(tangentImpulse / impulseMag) : 0f;

                _traceProjectionSamples++;
                _lastSignedNormalImpulse = signedNormalImpulse;
                _lastTangentImpulse = tangentImpulse;
                _lastNormalShareAbs = normalShareAbs;
                _signedNormalImpulseSum += signedNormalImpulse;
                _absNormalImpulseSum += absNormalImpulse;
                _tangentImpulseMagnitudeSum += tangentImpulse;
                _normalShareAbsSum += normalShareAbs;
                _tangentShareSum += tangentShare;
                _traceAbsNormalImpulseMax = Mathf.Max(_traceAbsNormalImpulseMax, absNormalImpulse);
                _traceTangentImpulseMagnitudeMax = Mathf.Max(_traceTangentImpulseMagnitudeMax, tangentImpulse);
                _traceNormalShareAbsMin = _traceProjectionSamples == 1 ? normalShareAbs : Mathf.Min(_traceNormalShareAbsMin, normalShareAbs);
                _traceTangentShareMax = Mathf.Max(_traceTangentShareMax, tangentShare);
                if (!_signedNormalImpulseInitialized)
                {
                    _traceSignedNormalImpulseMin = signedNormalImpulse;
                    _traceSignedNormalImpulseMax = signedNormalImpulse;
                    _signedNormalImpulseInitialized = true;
                }
                else
                {
                    _traceSignedNormalImpulseMin = Mathf.Min(_traceSignedNormalImpulseMin, signedNormalImpulse);
                    _traceSignedNormalImpulseMax = Mathf.Max(_traceSignedNormalImpulseMax, signedNormalImpulse);
                }

                if (signedNormalImpulse > NonZeroImpulseEpsilon) _traceAlignedImpulseSamples++;
                else if (signedNormalImpulse < -NonZeroImpulseEpsilon) _traceOpposedImpulseSamples++;
                else _traceNearZeroImpulseSamples++;

                if (relativeVelocityValid)
                {
                    float signedRelativeNormalSpeed = Vector3.Dot(relativeVelocity, _lastSupportNormal);
                    float absRelativeNormalSpeed = Mathf.Abs(signedRelativeNormalSpeed);
                    _traceRelativeNormalProjectionSamples++;
                    _signedRelativeNormalSpeedSum += signedRelativeNormalSpeed;
                    _relativeNormalSpeedAbsSum += absRelativeNormalSpeed;
                    _traceRelativeNormalSpeedAbsMax = Mathf.Max(_traceRelativeNormalSpeedAbsMax, absRelativeNormalSpeed);
                }
            }

            if (_preValid)
            {
                try
                {
                    Vector3 postVelocity = body.velocity;
                    Vector3 postAngularVelocity = body.angularVelocity;
                    Vector3 postPosition = body.position;
                    float mass = body.mass;
                    if (!IsFinite(postVelocity) || !IsFinite(postAngularVelocity) || !IsFinite(postPosition) || !IsFinite(mass) || mass <= 0f)
                    {
                        InvalidateNumeric("NON_FINITE_CALLBACK_RIGIDBODY_READBACK");
                    }
                    else
                    {
                        _tracePostBodyReadSamples++;
                        Vector3 observedDeltaV = postVelocity - _preVelocity;
                        Vector3 observedDeltaW = postAngularVelocity - _preAngularVelocity;
                        Vector3 noAccelerationPosition = _prePosition + _preVelocity * _preDt;
                        float positionResidual = (postPosition - noAccelerationPosition).magnitude;
                        if (!IsFinite(observedDeltaV) || !IsFinite(observedDeltaW) || !IsFinite(positionResidual))
                        {
                            InvalidateNumeric("NON_FINITE_PRE_TO_CALLBACK_RESPONSE");
                        }
                        else
                        {
                            _traceResponseSamples++;
                            _lastObservedDeltaV = observedDeltaV;
                            _lastObservedDeltaW = observedDeltaW;
                            _lastPositionResidual = positionResidual;

                            float observedDeltaVMag = observedDeltaV.magnitude;
                            float observedDeltaWMag = observedDeltaW.magnitude;
                            _observedDeltaVSum += observedDeltaVMag;
                            _observedDeltaWSum += observedDeltaWMag;
                            _positionResidualSum += positionResidual;
                            _traceObservedDeltaVMax = Mathf.Max(_traceObservedDeltaVMax, observedDeltaVMag);
                            _traceObservedDeltaWMax = Mathf.Max(_traceObservedDeltaWMax, observedDeltaWMag);
                            _tracePositionResidualMax = Mathf.Max(_tracePositionResidualMax, positionResidual);

                            if (impulseValid)
                            {
                                Vector3 impulseDeltaV = impulse / mass;
                                if (!IsFinite(impulseDeltaV))
                                {
                                    InvalidateNumeric("NON_FINITE_IMPULSE_DELTA_V");
                                }
                                else
                                {
                                    _traceImpulseDeltaVComparisonSamples++;
                                    float impulseDeltaVMag = impulseDeltaV.magnitude;
                                    _impulseDeltaVSum += impulseDeltaVMag;
                                    _traceImpulseDeltaVMax = Mathf.Max(_traceImpulseDeltaVMax, impulseDeltaVMag);
                                    if (observedDeltaVMag > NonZeroImpulseEpsilon && impulseDeltaVMag > NonZeroImpulseEpsilon)
                                    {
                                        float cosAbs = Mathf.Abs(Vector3.Dot(observedDeltaV, impulseDeltaV) / (observedDeltaVMag * impulseDeltaVMag));
                                        cosAbs = Mathf.Clamp01(cosAbs);
                                        _deltaVCosAbsSum += cosAbs;
                                        _deltaVCosSamples++;
                                        _traceDeltaVCosAbsMin = _deltaVCosSamples == 1 ? cosAbs : Mathf.Min(_traceDeltaVCosAbsMin, cosAbs);
                                    }
                                }
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                    _tracePostBodyReadFailures++;
                    RecordReadFailure("CALLBACK_RIGIDBODY", e);
                }
            }

            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;

            bool readbackClean = _traceReadFailures == 0;
            bool solverWitness = _finite && readbackClean &&
                                 _traceImpulseReadSamples >= MinImpulseReadSamples &&
                                 _traceRelativeVelocityReadSamples >= MinRelativeVelocityReadSamples &&
                                 _traceResponseSamples >= MinPrePhysicsResponseSamples &&
                                 _traceNonZeroImpulseSamples >= MinNonZeroImpulseSamples;
            bool supportNormalWitness = _finite &&
                                        _traceSupportNormalSamples >= MinSupportNormalSamples &&
                                        _traceProjectionSamples >= MinProjectionSamples &&
                                        _traceSupportNormalInvalidSamples == 0 &&
                                        _traceSupportColliderMismatchSamples == 0;
            bool qualified = solverWitness && supportNormalWitness;

            report.MechanicalReactionMode = Mode;
            report.MechanicalReactionFinite = _finite;
            report.MechanicalReactionInvalidReason = _invalidReason;
            report.MechanicalReactionReadbackClean = readbackClean;
            report.MechanicalReactionLastReadFailureStage = _lastReadFailureStage;
            report.MechanicalReactionLastReadFailureType = _lastReadFailureType;
            report.MechanicalReactionLastReadFailureMessage = _lastReadFailureMessage;
            report.MechanicalReactionSolverWitnessQualified = solverWitness;
            report.MechanicalReactionSupportNormalWitnessQualified = supportNormalWitness;
            report.MechanicalReactionTraceCallbacks = _traceCallbacks;
            report.MechanicalReactionTraceSupportNormalSamples = _traceSupportNormalSamples;
            report.MechanicalReactionTraceSupportNormalInvalidSamples = _traceSupportNormalInvalidSamples;
            report.MechanicalReactionTraceSupportNormalValidRatio = Ratio(_traceSupportNormalSamples, _traceSupportNormalSamples + _traceSupportNormalInvalidSamples);
            report.MechanicalReactionTraceSupportColliderMismatchSamples = _traceSupportColliderMismatchSamples;
            report.MechanicalReactionTraceSupportInvalidAuthorizedIdUnavailable = _traceSupportInvalidAuthorizedIdUnavailable;
            report.MechanicalReactionTraceSupportInvalidSupportIdUnavailable = _traceSupportInvalidSupportIdUnavailable;
            report.MechanicalReactionTraceSupportInvalidIdMismatch = _traceSupportInvalidIdMismatch;
            report.MechanicalReactionTraceSupportInvalidGeometryNotReady = _traceSupportInvalidGeometryNotReady;
            report.MechanicalReactionTraceSupportInvalidFootprintNotSameSurface = _traceSupportInvalidFootprintNotSameSurface;
            report.MechanicalReactionTraceSupportInvalidNormalNonFiniteOrZero = _traceSupportInvalidNormalNonFiniteOrZero;
            report.MechanicalReactionTraceSupportPredicateGeometryNotReady = _traceSupportPredicateGeometryNotReady;
            report.MechanicalReactionTraceSupportPredicateFootprintNotSameSurface = _traceSupportPredicateFootprintNotSameSurface;
            report.MechanicalReactionTraceSupportPredicateNormalNonFiniteOrZero = _traceSupportPredicateNormalNonFiniteOrZero;
            report.MechanicalReactionTraceSupportPredicateHitCountLt5 = _traceSupportPredicateHitCountLt5;
            report.MechanicalReactionTraceSupportPredicateHitCountLt3 = _traceSupportPredicateHitCountLt3;
            report.MechanicalReactionTraceSupportPredicatePrimarySurfaceNotSame = _traceSupportPredicatePrimarySurfaceNotSame;
            report.MechanicalReactionTraceSupportPredicateClearanceGateFail = _traceSupportPredicateClearanceGateFail;
            report.MechanicalReactionTraceSupportPredicateSpreadGateFail = _traceSupportPredicateSpreadGateFail;
            report.MechanicalReactionTraceSupportPredicateNormalDotGateFail = _traceSupportPredicateNormalDotGateFail;
            report.MechanicalReactionTraceSupportInvalidEnter = _traceSupportInvalidEnter;
            report.MechanicalReactionTraceSupportInvalidStay = _traceSupportInvalidStay;
            report.MechanicalReactionTraceSupportInvalidOtherPhase = _traceSupportInvalidOtherPhase;
            report.MechanicalReactionTraceSupportInvalidEpisodes = _traceSupportInvalidEpisodes;
            report.MechanicalReactionTraceSupportInvalidMaxStreak = _traceSupportInvalidMaxStreak;
            report.MechanicalReactionTraceSupportInvalidFirstCallbackIndex = _traceSupportInvalidFirstCallbackIndex;
            report.MechanicalReactionTraceSupportInvalidLastCallbackIndex = _traceSupportInvalidLastCallbackIndex;
            report.MechanicalReactionTraceSupportInvalidFirstPreFixedSequence = _traceSupportInvalidFirstPreFixedSequence;
            report.MechanicalReactionTraceSupportInvalidLastPreFixedSequence = _traceSupportInvalidLastPreFixedSequence;
            report.MechanicalReactionTraceInvalidLateralLeftOnlySamples = _traceInvalidLateralLeftOnlySamples;
            report.MechanicalReactionTraceInvalidLateralRightOnlySamples = _traceInvalidLateralRightOnlySamples;
            report.MechanicalReactionTraceInvalidLateralBothSamples = _traceInvalidLateralBothSamples;
            report.MechanicalReactionTraceInvalidNoLateralDivergenceSamples = _traceInvalidNoLateralDivergenceSamples;
            report.MechanicalReactionTraceInvalidPrimaryProbeDivergenceSamples = _traceInvalidPrimaryProbeDivergenceSamples;
            report.MechanicalReactionTraceAlternateColliderUniqueCount = _alternateColliderRecordCount;
            report.MechanicalReactionTraceAlternateColliderOverflowCount = _alternateColliderOverflowCount;
            report.MechanicalReactionTraceAlternateColliderCatalog = _alternateColliderCatalogCache;
            report.MechanicalReactionTraceInvalidEpisodeAttribution = _invalidEpisodeAttributionCache;
            report.MechanicalReactionTraceProjectionSamples = _traceProjectionSamples;
            report.MechanicalReactionTraceReadFailures = _traceReadFailures;
            report.MechanicalReactionTracePrePhysicsSamples = _tracePrePhysicsSamples;
            report.MechanicalReactionTraceResponseSamples = _traceResponseSamples;
            report.MechanicalReactionTraceImpulseReadSamples = _traceImpulseReadSamples;
            report.MechanicalReactionTraceImpulseReadFailures = _traceImpulseReadFailures;
            report.MechanicalReactionTraceRelativeVelocityReadSamples = _traceRelativeVelocityReadSamples;
            report.MechanicalReactionTraceRelativeVelocityReadFailures = _traceRelativeVelocityReadFailures;
            report.MechanicalReactionTracePostBodyReadSamples = _tracePostBodyReadSamples;
            report.MechanicalReactionTracePostBodyReadFailures = _tracePostBodyReadFailures;
            report.MechanicalReactionTraceNonZeroImpulseSamples = _traceNonZeroImpulseSamples;
            report.MechanicalReactionTraceAlignedImpulseSamples = _traceAlignedImpulseSamples;
            report.MechanicalReactionTraceOpposedImpulseSamples = _traceOpposedImpulseSamples;
            report.MechanicalReactionTraceNearZeroImpulseSamples = _traceNearZeroImpulseSamples;

            report.MechanicalReactionTraceImpulseMagnitudeMean = Mean(_impulseMagnitudeSum, _traceImpulseReadSamples);
            report.MechanicalReactionTraceImpulseMagnitudeMax = _traceImpulseMagnitudeMax;
            report.MechanicalReactionTraceSignedNormalImpulseMean = Mean(_signedNormalImpulseSum, _traceProjectionSamples);
            report.MechanicalReactionTraceSignedNormalImpulseMin = _signedNormalImpulseInitialized ? _traceSignedNormalImpulseMin : 0f;
            report.MechanicalReactionTraceSignedNormalImpulseMax = _signedNormalImpulseInitialized ? _traceSignedNormalImpulseMax : 0f;
            report.MechanicalReactionTraceAbsNormalImpulseMean = Mean(_absNormalImpulseSum, _traceProjectionSamples);
            report.MechanicalReactionTraceAbsNormalImpulseMax = _traceAbsNormalImpulseMax;
            report.MechanicalReactionTraceTangentImpulseMagnitudeMean = Mean(_tangentImpulseMagnitudeSum, _traceProjectionSamples);
            report.MechanicalReactionTraceTangentImpulseMagnitudeMax = _traceTangentImpulseMagnitudeMax;
            report.MechanicalReactionTraceNormalShareAbsMean = Mean(_normalShareAbsSum, _traceProjectionSamples);
            report.MechanicalReactionTraceNormalShareAbsMin = _traceProjectionSamples > 0 ? _traceNormalShareAbsMin : 0f;
            report.MechanicalReactionTraceTangentShareMean = Mean(_tangentShareSum, _traceProjectionSamples);
            report.MechanicalReactionTraceTangentShareMax = _traceTangentShareMax;
            report.MechanicalReactionTraceAlignedImpulseRatio = Ratio(_traceAlignedImpulseSamples, _traceProjectionSamples);
            report.MechanicalReactionTraceOpposedImpulseRatio = Ratio(_traceOpposedImpulseSamples, _traceProjectionSamples);
            report.MechanicalReactionTraceRelativeSpeedMean = Mean(_relativeSpeedSum, _traceRelativeVelocityReadSamples);
            report.MechanicalReactionTraceRelativeSpeedMax = _traceRelativeSpeedMax;
            report.MechanicalReactionTraceSignedRelativeNormalSpeedMean = Mean(_signedRelativeNormalSpeedSum, _traceRelativeNormalProjectionSamples);
            report.MechanicalReactionTraceRelativeNormalSpeedAbsMean = Mean(_relativeNormalSpeedAbsSum, _traceRelativeNormalProjectionSamples);
            report.MechanicalReactionTraceRelativeNormalSpeedAbsMax = _traceRelativeNormalSpeedAbsMax;
            report.MechanicalReactionTraceObservedDeltaVMean = Mean(_observedDeltaVSum, _traceResponseSamples);
            report.MechanicalReactionTraceObservedDeltaVMax = _traceObservedDeltaVMax;
            report.MechanicalReactionTraceImpulseDeltaVMean = Mean(_impulseDeltaVSum, _traceImpulseDeltaVComparisonSamples);
            report.MechanicalReactionTraceImpulseDeltaVMax = _traceImpulseDeltaVMax;
            report.MechanicalReactionTraceDeltaVCosAbsMean = Mean(_deltaVCosAbsSum, _deltaVCosSamples);
            report.MechanicalReactionTraceDeltaVCosAbsMin = _deltaVCosSamples > 0 ? _traceDeltaVCosAbsMin : 0f;
            report.MechanicalReactionTraceObservedDeltaWMean = Mean(_observedDeltaWSum, _traceResponseSamples);
            report.MechanicalReactionTraceObservedDeltaWMax = _traceObservedDeltaWMax;
            report.MechanicalReactionTracePositionResidualMean = Mean(_positionResidualSum, _traceResponseSamples);
            report.MechanicalReactionTracePositionResidualMax = _tracePositionResidualMax;
            report.MechanicalReactionTraceQualified = qualified;

            report.MechanicalReactionCurrentSummary =
                "last=" + Clean(_lastPhase) +
                "/supportN=" + V(_lastSupportNormal) +
                "/impulse=" + V(_lastImpulse) +
                "/relativeV=" + V(_lastRelativeVelocity) +
                "/signedNormalJ=" + F(_lastSignedNormalImpulse) +
                "/tangentJ=" + F(_lastTangentImpulse) +
                "/normalShareAbs=" + F(_lastNormalShareAbs) +
                "/dV=" + V(_lastObservedDeltaV) +
                "/dW=" + V(_lastObservedDeltaW) +
                "/posResidualMm=" + F(_lastPositionResidual * 1000f) +
                "/preFixed=" + _preFixedSequence +
                "/lastReadFail=" + Clean(_lastReadFailureStage) + ":" + Clean(_lastReadFailureType);

            report.MechanicalReactionTraceSummary =
                "callbacks=" + _traceCallbacks +
                "/supportN=" + _traceSupportNormalSamples +
                "/supportNInvalid=" + _traceSupportNormalInvalidSamples +
                "/supportNValidRatio=" + F(report.MechanicalReactionTraceSupportNormalValidRatio) +
                "/supportIdMismatch=" + _traceSupportColliderMismatchSamples +
                "/invalidPrimary(authMissing-supportIdMissing-idMismatch-geometry-footprint-normal)=" +
                    _traceSupportInvalidAuthorizedIdUnavailable + "-" +
                    _traceSupportInvalidSupportIdUnavailable + "-" +
                    _traceSupportInvalidIdMismatch + "-" +
                    _traceSupportInvalidGeometryNotReady + "-" +
                    _traceSupportInvalidFootprintNotSameSurface + "-" +
                    _traceSupportInvalidNormalNonFiniteOrZero +
                "/invalidPredicate(geometry-footprint-normal-hitLt5-hitLt3-primarySurface-clear-spread-normalDot)=" +
                    _traceSupportPredicateGeometryNotReady + "-" +
                    _traceSupportPredicateFootprintNotSameSurface + "-" +
                    _traceSupportPredicateNormalNonFiniteOrZero + "-" +
                    _traceSupportPredicateHitCountLt5 + "-" +
                    _traceSupportPredicateHitCountLt3 + "-" +
                    _traceSupportPredicatePrimarySurfaceNotSame + "-" +
                    _traceSupportPredicateClearanceGateFail + "-" +
                    _traceSupportPredicateSpreadGateFail + "-" +
                    _traceSupportPredicateNormalDotGateFail +
                "/invalidPhase(enter-stay-other)=" + _traceSupportInvalidEnter + "-" + _traceSupportInvalidStay + "-" + _traceSupportInvalidOtherPhase +
                "/invalidEpisodes-maxStreak=" + _traceSupportInvalidEpisodes + "-" + _traceSupportInvalidMaxStreak +
                "/invalidCallbackFirstLast=" + _traceSupportInvalidFirstCallbackIndex + "-" + _traceSupportInvalidLastCallbackIndex +
                "/invalidPreFixedFirstLast=" + _traceSupportInvalidFirstPreFixedSequence + "-" + _traceSupportInvalidLastPreFixedSequence +
                "/invalidPrimarySum-conserved=" +
                    (_traceSupportInvalidAuthorizedIdUnavailable + _traceSupportInvalidSupportIdUnavailable + _traceSupportInvalidIdMismatch +
                     _traceSupportInvalidGeometryNotReady + _traceSupportInvalidFootprintNotSameSurface + _traceSupportInvalidNormalNonFiniteOrZero) + "-" +
                    ((_traceSupportInvalidAuthorizedIdUnavailable + _traceSupportInvalidSupportIdUnavailable + _traceSupportInvalidIdMismatch +
                      _traceSupportInvalidGeometryNotReady + _traceSupportInvalidFootprintNotSameSurface + _traceSupportInvalidNormalNonFiniteOrZero) == _traceSupportNormalInvalidSamples) +
                "/lateralDivergence(L-R-B-none)=" + _traceInvalidLateralLeftOnlySamples + "-" + _traceInvalidLateralRightOnlySamples + "-" + _traceInvalidLateralBothSamples + "-" + _traceInvalidNoLateralDivergenceSamples +
                "/primaryProbeDivergence=" + _traceInvalidPrimaryProbeDivergenceSamples +
                "/alternateCollider(unique-overflow)=" + _alternateColliderRecordCount + "-" + _alternateColliderOverflowCount +
                "/episodeRecord(stored-overflow)=" + _invalidEpisodeRecordCount + "-" + _invalidEpisodeRecordOverflowCount +
                "/projection=" + _traceProjectionSamples +
                "/readFail=" + _traceReadFailures +
                "/stageFail(imp-rel-post)=" + _traceImpulseReadFailures + "-" + _traceRelativeVelocityReadFailures + "-" + _tracePostBodyReadFailures +
                "/pre=" + _tracePrePhysicsSamples +
                "/response=" + _traceResponseSamples +
                "/impulseRead=" + _traceImpulseReadSamples +
                "/relVelRead=" + _traceRelativeVelocityReadSamples +
                "/nonZeroImpulse=" + _traceNonZeroImpulseSamples +
                "/aligned-opposed-nearZero=" + _traceAlignedImpulseSamples + "-" + _traceOpposedImpulseSamples + "-" + _traceNearZeroImpulseSamples +
                "/solverWitness=" + solverWitness +
                "/supportNormalWitness=" + supportNormalWitness +
                "/readbackClean=" + readbackClean +
                "/impulseMeanMax=" + F(report.MechanicalReactionTraceImpulseMagnitudeMean) + "-" + F(report.MechanicalReactionTraceImpulseMagnitudeMax) +
                "/signedNormalJMeanMinMax=" + F(report.MechanicalReactionTraceSignedNormalImpulseMean) + "-" + F(report.MechanicalReactionTraceSignedNormalImpulseMin) + "-" + F(report.MechanicalReactionTraceSignedNormalImpulseMax) +
                "/absNormalJMeanMax=" + F(report.MechanicalReactionTraceAbsNormalImpulseMean) + "-" + F(report.MechanicalReactionTraceAbsNormalImpulseMax) +
                "/tangentJMeanMax=" + F(report.MechanicalReactionTraceTangentImpulseMagnitudeMean) + "-" + F(report.MechanicalReactionTraceTangentImpulseMagnitudeMax) +
                "/normalShareAbsMeanMin=" + F(report.MechanicalReactionTraceNormalShareAbsMean) + "-" + F(report.MechanicalReactionTraceNormalShareAbsMin) +
                "/tangentShareMeanMax=" + F(report.MechanicalReactionTraceTangentShareMean) + "-" + F(report.MechanicalReactionTraceTangentShareMax) +
                "/alignedOpposedRatio=" + F(report.MechanicalReactionTraceAlignedImpulseRatio) + "-" + F(report.MechanicalReactionTraceOpposedImpulseRatio) +
                "/relSpeedMeanMax=" + F(report.MechanicalReactionTraceRelativeSpeedMean) + "-" + F(report.MechanicalReactionTraceRelativeSpeedMax) +
                "/relNormalSignedMean=" + F(report.MechanicalReactionTraceSignedRelativeNormalSpeedMean) +
                "/relNormalAbsMeanMax=" + F(report.MechanicalReactionTraceRelativeNormalSpeedAbsMean) + "-" + F(report.MechanicalReactionTraceRelativeNormalSpeedAbsMax) +
                "/obsDvMeanMax=" + F(report.MechanicalReactionTraceObservedDeltaVMean) + "-" + F(report.MechanicalReactionTraceObservedDeltaVMax) +
                "/impulseDvMeanMax=" + F(report.MechanicalReactionTraceImpulseDeltaVMean) + "-" + F(report.MechanicalReactionTraceImpulseDeltaVMax) +
                "/dVCosAbsMeanMin=" + F(report.MechanicalReactionTraceDeltaVCosAbsMean) + "-" + F(report.MechanicalReactionTraceDeltaVCosAbsMin) +
                "/obsDwMeanMax=" + F(report.MechanicalReactionTraceObservedDeltaWMean) + "-" + F(report.MechanicalReactionTraceObservedDeltaWMax) +
                "/posResidualMeanMaxMm=" + F(report.MechanicalReactionTracePositionResidualMean * 1000f) + "-" + F(report.MechanicalReactionTracePositionResidualMax * 1000f) +
                "/lastReadFail=" + Clean(_lastReadFailureStage) + ":" + Clean(_lastReadFailureType) +
                "/TRACE_SUPPORT_NORMAL_REACTION_QUALIFIED=" + qualified +
                "/thresholds(supportN-projection-impulse-relVel-response-nonZeroImpulse)=" + MinSupportNormalSamples + "-" + MinProjectionSamples + "-" + MinImpulseReadSamples + "-" + MinRelativeVelocityReadSamples + "-" + MinPrePhysicsResponseSamples + "-" + MinNonZeroImpulseSamples +
                "/observerOnly=True/gravityReadback=STANCE_NATIVE_GRAVITY_OWNER/noForceWrite=True";
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + (report != null ? report.MechanicalReactionMode : "<none>") +
                   "/finite=" + (report != null && report.MechanicalReactionFinite) +
                   "/reason=" + (report != null ? Clean(report.MechanicalReactionInvalidReason) : "<none>") +
                   " | current=" + (report != null ? report.MechanicalReactionCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.MechanicalReactionTraceSummary : "<none>") +
                   " | alternateColliders=" + (report != null ? Clean(report.MechanicalReactionTraceAlternateColliderCatalog) : "<none>") +
                   " | invalidEpisodes=" + (report != null ? Clean(report.MechanicalReactionTraceInvalidEpisodeAttribution) : "<none>");
        }

        internal void EndTrace(FootAnkleFoundationReport report)
        {
            CloseInvalidEpisode(report, "TRACE_END");
            CopyToReport(report);
        }

        private ProbeAttributionSample CaptureProbeAttribution(FootSoleProbeSnapshot snapshot, Rigidbody body, Vector3 targetSolePosition, Quaternion targetSoleRotation)
        {
            int centerId = ProbeId(snapshot.Center);
            int toeId = ProbeId(snapshot.Toe);
            int heelId = ProbeId(snapshot.Heel);
            int leftId = ProbeId(snapshot.Left);
            int rightId = ProbeId(snapshot.Right);
            ProbeAttributionSample sample = new ProbeAttributionSample
            {
                CallbackIndex = _traceCallbacks,
                PreFixedSequence = _preFixedSequence,
                CenterId = centerId,
                ToeId = toeId,
                HeelId = heelId,
                LeftId = leftId,
                RightId = rightId,
                CenterHit = snapshot.Center.Hit,
                ToeHit = snapshot.Toe.Hit,
                HeelHit = snapshot.Heel.Hit,
                LeftHit = snapshot.Left.Hit,
                RightHit = snapshot.Right.Hit,
                CenterClearance = snapshot.Center.ClearanceMeters,
                ToeClearance = snapshot.Toe.ClearanceMeters,
                HeelClearance = snapshot.Heel.ClearanceMeters,
                LeftClearance = snapshot.Left.ClearanceMeters,
                RightClearance = snapshot.Right.ClearanceMeters,
                MaxAbsClearance = snapshot.MaxAbsClearanceMeters,
                Spread = snapshot.SpreadMeters,
                LeftDiffersFromCenter = snapshot.Center.Hit && snapshot.Left.Hit && centerId != 0 && leftId != centerId,
                RightDiffersFromCenter = snapshot.Center.Hit && snapshot.Right.Hit && centerId != 0 && rightId != centerId,
                ToeDiffersFromCenter = snapshot.Center.Hit && snapshot.Toe.Hit && centerId != 0 && toeId != centerId,
                HeelDiffersFromCenter = snapshot.Center.Hit && snapshot.Heel.Hit && centerId != 0 && heelId != centerId
            };

            try
            {
                if (body != null)
                {
                    Quaternion physicalRotation = body.rotation;
                    Quaternion targetRotation = targetSoleRotation;
                    Vector3 physicalPosition = body.position;
                    Vector3 targetPosition = targetSolePosition;
                    if (IsFinite(physicalPosition) && IsFinite(targetPosition) && IsFinite(physicalRotation) && IsFinite(targetRotation))
                    {
                        sample.PoseValid = true;
                        sample.PhysicalPosition = physicalPosition;
                        sample.TargetSolePosition = targetPosition;
                        sample.PhysicalEulerDegrees = physicalRotation.eulerAngles;
                        sample.TargetSoleEulerDegrees = targetRotation.eulerAngles;
                        sample.PhysicalTargetPositionErrorMeters = (physicalPosition - targetPosition).magnitude;
                        sample.PhysicalTargetRotationErrorDegrees = Quaternion.Angle(physicalRotation, targetRotation);
                    }
                }
            }
            catch
            {
                // Optional attribution-only pose context must never invalidate the mechanical witness.
                sample.PoseValid = false;
            }
            return sample;
        }

        private void RecordProbeDivergence(ProbeAttributionSample sample, FootSoleProbeSnapshot snapshot)
        {
            if (sample.LeftDiffersFromCenter && sample.RightDiffersFromCenter) _traceInvalidLateralBothSamples++;
            else if (sample.LeftDiffersFromCenter) _traceInvalidLateralLeftOnlySamples++;
            else if (sample.RightDiffersFromCenter) _traceInvalidLateralRightOnlySamples++;
            else _traceInvalidNoLateralDivergenceSamples++;

            if (sample.ToeDiffersFromCenter || sample.HeelDiffersFromCenter)
                _traceInvalidPrimaryProbeDivergenceSamples++;

            RegisterAlternateCollider(snapshot.Center, sample.CenterId, sample.CenterId);
            RegisterAlternateCollider(snapshot.Toe, sample.ToeId, sample.CenterId);
            RegisterAlternateCollider(snapshot.Heel, sample.HeelId, sample.CenterId);
            RegisterAlternateCollider(snapshot.Left, sample.LeftId, sample.CenterId);
            RegisterAlternateCollider(snapshot.Right, sample.RightId, sample.CenterId);
        }

        private void RegisterAlternateCollider(FootSoleProbePointSnapshot point, int pointId, int centerId)
        {
            if (!point.Hit || point.Collider == null || pointId == 0 || centerId == 0 || pointId == centerId)
                return;

            for (int i = 0; i < _alternateColliderRecordCount; i++)
            {
                if (_alternateColliderIds[i] == pointId)
                {
                    _alternateColliderCounts[i]++;
                    return;
                }
            }

            if (_alternateColliderRecordCount >= MaxAlternateColliderRecords)
            {
                _alternateColliderOverflowCount++;
                return;
            }

            int slot = _alternateColliderRecordCount++;
            _alternateColliderIds[slot] = pointId;
            _alternateColliderCounts[slot] = 1;
            _alternateColliderPaths[slot] = ColliderPath(point.Collider);
            RebuildAlternateColliderCatalogCache();
        }

        private void BeginInvalidEpisode(ProbeAttributionSample sample)
        {
            _invalidEpisodeActive = true;
            _activeInvalidEpisode = new InvalidEpisodeRecord
            {
                EpisodeIndex = _traceSupportInvalidEpisodes,
                First = sample,
                PeakSpread = sample,
                Last = sample
            };
        }

        private void UpdateInvalidEpisode(ProbeAttributionSample sample)
        {
            if (!_invalidEpisodeActive)
            {
                BeginInvalidEpisode(sample);
                return;
            }
            _activeInvalidEpisode.Last = sample;
            if (sample.Spread > _activeInvalidEpisode.PeakSpread.Spread)
                _activeInvalidEpisode.PeakSpread = sample;
        }

        private void CloseInvalidEpisode(FootAnkleFoundationReport report, string closeReason)
        {
            if (!_invalidEpisodeActive)
                return;

            if (_invalidEpisodeRecordCount < MaxInvalidEpisodeRecords)
                _invalidEpisodeRecords[_invalidEpisodeRecordCount++] = _activeInvalidEpisode;
            else
                _invalidEpisodeRecordOverflowCount++;

            RebuildAlternateColliderCatalogCache();
            string side = report != null ? Clean(report.Side) : "<none>";
            string message = "[ARL][SUPPORT_NORMAL_INVALID_EPISODE_V0432] side=" + side +
                             " | close=" + Clean(closeReason) +
                             " | episode=" + _activeInvalidEpisode.EpisodeIndex +
                             " | first={" + FormatProbeSample(_activeInvalidEpisode.First) + "}" +
                             " | peakSpread={" + FormatProbeSample(_activeInvalidEpisode.PeakSpread) + "}" +
                             " | last={" + FormatProbeSample(_activeInvalidEpisode.Last) + "}" +
                             " | alternateColliders={" + _alternateColliderCatalogCache + "}";
            Plugin.Logger?.LogWarning(message);

            _invalidEpisodeActive = false;
            _activeInvalidEpisode = new InvalidEpisodeRecord();
            RebuildInvalidEpisodeAttributionCache();
        }

        private void RebuildAlternateColliderCatalogCache()
        {
            if (_alternateColliderRecordCount <= 0)
            {
                _alternateColliderCatalogCache = "<none>";
                return;
            }

            string value = string.Empty;
            for (int i = 0; i < _alternateColliderRecordCount; i++)
            {
                if (i > 0) value += ";";
                value += _alternateColliderIds[i] + "x" + _alternateColliderCounts[i] + "=" + Clean(_alternateColliderPaths[i]);
            }
            if (_alternateColliderOverflowCount > 0)
                value += ";overflow=" + _alternateColliderOverflowCount;
            _alternateColliderCatalogCache = value;
        }

        private void RebuildInvalidEpisodeAttributionCache()
        {
            if (_invalidEpisodeRecordCount <= 0)
            {
                _invalidEpisodeAttributionCache = "<none>";
                return;
            }

            string value = string.Empty;
            for (int i = 0; i < _invalidEpisodeRecordCount; i++)
            {
                InvalidEpisodeRecord episode = _invalidEpisodeRecords[i];
                if (i > 0) value += ";";
                value += "E" + episode.EpisodeIndex + "[F:" + FormatProbeSampleCompact(episode.First) +
                         "|P:" + FormatProbeSampleCompact(episode.PeakSpread) +
                         "|L:" + FormatProbeSampleCompact(episode.Last) + "]";
            }
            if (_invalidEpisodeRecordOverflowCount > 0)
                value += ";overflow=" + _invalidEpisodeRecordOverflowCount;
            _invalidEpisodeAttributionCache = value;
        }

        private static string FormatProbeSample(ProbeAttributionSample sample)
        {
            return "cb=" + sample.CallbackIndex +
                   "/fx=" + sample.PreFixedSequence +
                   "/div=" + DivergenceCode(sample) +
                   "/C=" + ProbeText(sample.CenterHit, sample.CenterId, sample.CenterClearance) +
                   "/T=" + ProbeText(sample.ToeHit, sample.ToeId, sample.ToeClearance) +
                   "/H=" + ProbeText(sample.HeelHit, sample.HeelId, sample.HeelClearance) +
                   "/L=" + ProbeText(sample.LeftHit, sample.LeftId, sample.LeftClearance) +
                   "/R=" + ProbeText(sample.RightHit, sample.RightId, sample.RightClearance) +
                   "/maxAbsMm=" + F(sample.MaxAbsClearance * 1000f) +
                   "/spreadMm=" + F(sample.Spread * 1000f) +
                   "/pose=" + (sample.PoseValid
                       ? "physP" + V(sample.PhysicalPosition) + "/targetP" + V(sample.TargetSolePosition) +
                         "/physEuler" + V(sample.PhysicalEulerDegrees) + "/targetEuler" + V(sample.TargetSoleEulerDegrees) +
                         "/errMmDeg=" + F(sample.PhysicalTargetPositionErrorMeters * 1000f) + "-" + F(sample.PhysicalTargetRotationErrorDegrees)
                       : "NA");
        }

        private static string FormatProbeSampleCompact(ProbeAttributionSample sample)
        {
            return sample.CallbackIndex + "@" + sample.PreFixedSequence +
                   ",div=" + DivergenceCode(sample) +
                   ",ids=" + sample.CenterId + "/" + sample.ToeId + "/" + sample.HeelId + "/" + sample.LeftId + "/" + sample.RightId +
                   ",clrMm=" + ClearanceValue(sample.CenterHit, sample.CenterClearance) + "/" + ClearanceValue(sample.ToeHit, sample.ToeClearance) + "/" + ClearanceValue(sample.HeelHit, sample.HeelClearance) + "/" + ClearanceValue(sample.LeftHit, sample.LeftClearance) + "/" + ClearanceValue(sample.RightHit, sample.RightClearance) +
                   ",spread=" + F(sample.Spread * 1000f) +
                   ",poseErrMmDeg=" + (sample.PoseValid ? F(sample.PhysicalTargetPositionErrorMeters * 1000f) + "/" + F(sample.PhysicalTargetRotationErrorDegrees) : "NA");
        }

        private static string DivergenceCode(ProbeAttributionSample sample)
        {
            string code = string.Empty;
            if (sample.ToeDiffersFromCenter) code += "T";
            if (sample.HeelDiffersFromCenter) code += "H";
            if (sample.LeftDiffersFromCenter) code += "L";
            if (sample.RightDiffersFromCenter) code += "R";
            return string.IsNullOrEmpty(code) ? "NONE" : code;
        }

        private static string ProbeText(bool hit, int id, float clearance)
        {
            return hit ? id + "@" + F(clearance * 1000f) + "mm" : "NO_HIT";
        }

        private static string ClearanceValue(bool hit, float clearance)
        {
            return hit ? F(clearance * 1000f) : "NA";
        }

        private static int ProbeId(FootSoleProbePointSnapshot point)
        {
            return point.Hit && point.Collider != null ? point.Collider.GetInstanceID() : 0;
        }

        private static string ColliderPath(Collider collider)
        {
            return collider != null && collider.transform != null ? TransformPath(collider.transform) + "#" + collider.GetType().Name : "<none>";
        }

        private static string TransformPath(Transform transform)
        {
            if (transform == null) return "<none>";
            string path = transform.name ?? "<unnamed>";
            Transform p = transform.parent;
            int guard = 0;
            while (p != null && guard++ < 64)
            {
                path = (p.name ?? "<unnamed>") + "/" + path;
                p = p.parent;
            }
            return path;
        }

        private void RecordReadFailure(string stage, Exception e)
        {
            _traceReadFailures++;
            _lastReadFailureStage = stage ?? "UNKNOWN";
            _lastReadFailureType = e != null ? e.GetType().Name : "UnknownException";
            _lastReadFailureMessage = e != null ? Clean(e.Message) : "<none>";
            if (string.IsNullOrEmpty(_invalidReason))
                _invalidReason = "READBACK_STAGE_FAILURE:" + _lastReadFailureStage + ":" + _lastReadFailureType;
        }

        private void InvalidateNumeric(string reason)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason))
                _invalidReason = reason;
        }

        private static float Mean(float sum, int count) => count > 0 ? sum / count : 0f;
        private static float Ratio(int numerator, int denominator) => denominator > 0 ? (float)numerator / denominator : 0f;
        private static bool IsFinite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool IsFinite(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        private static bool IsFinite(Quaternion q) => IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace('\r', ' ').Replace('\n', ' ');
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
