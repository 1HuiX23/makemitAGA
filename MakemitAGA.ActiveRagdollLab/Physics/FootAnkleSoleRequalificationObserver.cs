using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // v0.4.3.2 read-only snapshot of the already-executed five-point sole probe.
    // Collider references are retained only long enough for downstream diagnostic attribution;
    // no extra raycast or physics authority is introduced.
    internal struct FootSoleProbePointSnapshot
    {
        internal bool Hit;
        internal Collider Collider;
        internal float ClearanceMeters;
    }

    internal struct FootSoleProbeSnapshot
    {
        internal FootSoleProbePointSnapshot Center;
        internal FootSoleProbePointSnapshot Toe;
        internal FootSoleProbePointSnapshot Heel;
        internal FootSoleProbePointSnapshot Left;
        internal FootSoleProbePointSnapshot Right;
        internal float MaxAbsClearanceMeters;
        internal float SpreadMeters;
    }

    /// <summary>
    /// ARL v0.3.56.1 support-plane-calibrated sole-frame / ankle-ROM / ground-readiness requalification.
    ///
    /// v0.3.55 proved that a nearly free (±170 deg) passive ankle makes the Dynamic Foot behave like a
    /// pendulous body even though the ankle anchor itself stays tight. v0.3.56 then proved the ground probe itself
    /// is valid but the retained estimated ankleHeight leaves the sole about 48 mm above the flat room floor.
    /// v0.3.56.1 adds a strict one-shot build-time support-plane height calibration, then measures what is still
    /// required before a bounded passive ankle can be
    /// reintroduced:
    ///   1) sole +Z really points toward the source toe rather than the heel;
    ///   2) source Lower->sole relative ROM in bind-sole coordinates (X flex, Y axial, Z inversion);
    ///   3) the proxy sole bottom is geometrically at the environment floor during quiet IDLE windows.
    ///
    /// The ground probe is a READ-ONLY Physics.Raycast witness.  Foot Collider stays disabled, so
    /// GroundGeometryReady must never be interpreted as solver contact.  Actual contact remains impossible
    /// until a later explicitly-authorized Collider/GSP stage.
    /// </summary>
    internal sealed class FootAnkleSoleRequalificationObserver
    {
        internal const string Mode = "SOLE_FRAME_ANKLE_ROM_GROUND_READINESS_READ_ONLY_V03561";

        private const float GroundProbeLiftMeters = 0.080f;
        private const float GroundProbeBelowMeters = 0.250f;
        private const float GroundNormalDotMin = 0.65f;
        private const float GroundReadyAbsClearanceMeters = 0.025f;
        private const float GroundReadySpreadMeters = 0.025f;
        private const float IdleGroundMaxFootSpeed = 0.15f;
        private const float RayAdvanceEpsilon = 0.002f;
        private const int MaxRejectedRayHits = 6;
        private const int RomHistogramBins = 181;

        // v0.3.56.1 one-shot support-plane calibration gate.  This is intentionally stricter than
        // the runtime GEOMETRY_READY witness: all five points must see the same flat support surface,
        // and the correction must stay inside a plausible footwear/anatomical envelope.
        private const float CalibrationNormalDotMin = 0.90f;
        private const float CalibrationMaxSpreadMeters = 0.010f;
        private const float CalibrationMaxAbsCorrectionMeters = 0.150f;

        internal struct SupportPlaneCalibrationCandidate
        {
            internal bool Valid;
            internal float CenterClearanceMeters;
            internal int HitCount;
            internal float SpreadMeters;
            internal float NormalDotUp;
            internal bool SameSurface;
            internal string SurfacePath;
            internal string Reason;
        }

        internal SupportPlaneCalibrationCandidate GetSupportPlaneCalibrationCandidate(
            Transform soleFrame,
            Vector3 colliderCenterLocal,
            Vector3 colliderSizeLocal,
            Transform sourceFoot,
            Transform targetRoot,
            Transform ownerRoot)
        {
            SupportPlaneCalibrationCandidate candidate = new SupportPlaneCalibrationCandidate
            {
                Valid = false,
                CenterClearanceMeters = 0f,
                HitCount = 0,
                SpreadMeters = 0f,
                NormalDotUp = 0f,
                SameSurface = false,
                SurfacePath = null,
                Reason = "UNQUALIFIED"
            };

            if (soleFrame == null || !FiniteVector(colliderCenterLocal) || !FiniteVector(colliderSizeLocal) ||
                colliderSizeLocal.x <= 0f || colliderSizeLocal.y <= 0f || colliderSizeLocal.z <= 0f)
            {
                candidate.Reason = "INVALID_SOLE_FRAME_OR_GEOMETRY";
                return candidate;
            }

            Vector3 referenceUp = targetRoot != null ? targetRoot.up : Vector3.up;
            if (!FiniteVector(referenceUp) || referenceUp.sqrMagnitude < 0.0001f)
                referenceUp = Vector3.up;
            referenceUp.Normalize();

            float bottomY = colliderCenterLocal.y - colliderSizeLocal.y * 0.5f;
            float x = colliderSizeLocal.x * 0.40f;
            float z = colliderSizeLocal.z * 0.40f;
            GroundHit centerHit = ProbePoint(soleFrame.TransformPoint(new Vector3(colliderCenterLocal.x, bottomY, colliderCenterLocal.z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit toeHit = ProbePoint(soleFrame.TransformPoint(new Vector3(colliderCenterLocal.x, bottomY, colliderCenterLocal.z + z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit heelHit = ProbePoint(soleFrame.TransformPoint(new Vector3(colliderCenterLocal.x, bottomY, colliderCenterLocal.z - z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit leftHit = ProbePoint(soleFrame.TransformPoint(new Vector3(colliderCenterLocal.x - x, bottomY, colliderCenterLocal.z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit rightHit = ProbePoint(soleFrame.TransformPoint(new Vector3(colliderCenterLocal.x + x, bottomY, colliderCenterLocal.z)), referenceUp, sourceFoot, targetRoot, ownerRoot);

            candidate.HitCount = (centerHit.Valid ? 1 : 0) + (toeHit.Valid ? 1 : 0) + (heelHit.Valid ? 1 : 0) +
                                 (leftHit.Valid ? 1 : 0) + (rightHit.Valid ? 1 : 0);
            if (!centerHit.Valid || candidate.HitCount != 5)
            {
                candidate.Reason = !centerHit.Valid ? "CENTER_NO_HIT" : "REQUIRES_5_OF_5_HITS";
                return candidate;
            }

            candidate.SameSurface = toeHit.Collider == centerHit.Collider && heelHit.Collider == centerHit.Collider &&
                                    leftHit.Collider == centerHit.Collider && rightHit.Collider == centerHit.Collider;
            candidate.SurfacePath = ColliderPath(centerHit.Collider);
            candidate.NormalDotUp = Mathf.Min(centerHit.NormalDotUp, Mathf.Min(toeHit.NormalDotUp,
                Mathf.Min(heelHit.NormalDotUp, Mathf.Min(leftHit.NormalDotUp, rightHit.NormalDotUp))));

            float min = float.PositiveInfinity;
            float max = float.NegativeInfinity;
            AddClearance(centerHit, ref min, ref max);
            AddClearance(toeHit, ref min, ref max);
            AddClearance(heelHit, ref min, ref max);
            AddClearance(leftHit, ref min, ref max);
            AddClearance(rightHit, ref min, ref max);
            candidate.SpreadMeters = Mathf.Max(0f, max - min);
            candidate.CenterClearanceMeters = centerHit.Clearance;

            if (!candidate.SameSurface)
            {
                candidate.Reason = "MULTIPLE_SUPPORT_SURFACES";
                return candidate;
            }
            if (candidate.NormalDotUp < CalibrationNormalDotMin)
            {
                candidate.Reason = "SUPPORT_NORMAL_TOO_STEEP";
                return candidate;
            }
            if (candidate.SpreadMeters > CalibrationMaxSpreadMeters)
            {
                candidate.Reason = "SUPPORT_PLANE_NOT_FLAT_ENOUGH";
                return candidate;
            }
            if (!FiniteFloat(candidate.CenterClearanceMeters) || Mathf.Abs(candidate.CenterClearanceMeters) > CalibrationMaxAbsCorrectionMeters)
            {
                candidate.Reason = "CORRECTION_OUT_OF_RANGE";
                return candidate;
            }

            candidate.Valid = true;
            candidate.Reason = "PASS";
            return candidate;
        }

        private bool _finite = true;
        private string _invalidReason;

        private float _currentToeForwardDot;
        private float _currentSoleUpDotReferenceUp;
        private Vector3 _currentBindSoleDeltaDegrees;
        private bool _currentGroundReady;
        private int _currentGroundHits;
        private float _currentCenterClearance;
        private bool _currentCenterHit;
        private Collider _currentCenterCollider;
        private float _currentToeClearance;
        private bool _currentToeHit;
        private Collider _currentToeCollider;
        private float _currentHeelClearance;
        private bool _currentHeelHit;
        private Collider _currentHeelCollider;
        private float _currentLeftClearance;
        private bool _currentLeftHit;
        private Collider _currentLeftCollider;
        private float _currentRightClearance;
        private bool _currentRightHit;
        private Collider _currentRightCollider;
        private float _currentGroundMinClearance;
        private float _currentGroundMaxClearance;
        private float _currentGroundSpread;
        private float _currentGroundNormalDotUp;
        private float _currentFootprintMinNormalDotUp;
        private Vector3 _currentGroundAverageNormalWorld;
        private string _currentGroundPath;
        private bool _currentPrimarySameSurface;
        private bool _currentFootprintSameSurface;
        private int _currentGroundColliderInstanceId;
        private bool _currentGroundColliderHasAttachedRigidbody;
        private Collider _currentGroundCollider;
        private int _cachedGroundColliderInstanceId;
        private string _cachedGroundColliderPath;

        private FootSupportSurfaceIsolation _supportSurface;
        internal void BindSupportSurface(FootSupportSurfaceIsolation surface) => _supportSurface = surface;

        internal Collider CurrentGroundCollider => _currentGroundCollider;
        internal Vector3 CurrentGroundAverageNormalWorld => _currentGroundAverageNormalWorld;

        // v0.4.3.1 observer-only support-normal diagnostics. Expose the already-computed sole probe
        // state so downstream attribution can decompose transient invalid samples without issuing any
        // extra Physics.Raycast calls or changing the v0.3.56.1 ground-readiness contract.
        internal int CurrentGroundHitCount => _currentGroundHits;
        internal bool CurrentPrimarySameSurface => _currentPrimarySameSurface;
        internal float CurrentGroundMaxAbsClearanceMeters => Mathf.Max(Mathf.Abs(_currentGroundMinClearance), Mathf.Abs(_currentGroundMaxClearance));
        internal float CurrentGroundSpreadMeters => _currentGroundSpread;
        internal float CurrentGroundNormalDotUp => _currentGroundNormalDotUp;
        internal float GroundReadyAbsClearanceThreshold => GroundReadyAbsClearanceMeters;
        internal float GroundReadySpreadThreshold => GroundReadySpreadMeters;
        internal float GroundNormalDotThreshold => GroundNormalDotMin;

        internal FootSoleProbeSnapshot GetCurrentProbeSnapshot()
        {
            return new FootSoleProbeSnapshot
            {
                Center = new FootSoleProbePointSnapshot { Hit = _currentCenterHit, Collider = _currentCenterCollider, ClearanceMeters = _currentCenterClearance },
                Toe = new FootSoleProbePointSnapshot { Hit = _currentToeHit, Collider = _currentToeCollider, ClearanceMeters = _currentToeClearance },
                Heel = new FootSoleProbePointSnapshot { Hit = _currentHeelHit, Collider = _currentHeelCollider, ClearanceMeters = _currentHeelClearance },
                Left = new FootSoleProbePointSnapshot { Hit = _currentLeftHit, Collider = _currentLeftCollider, ClearanceMeters = _currentLeftClearance },
                Right = new FootSoleProbePointSnapshot { Hit = _currentRightHit, Collider = _currentRightCollider, ClearanceMeters = _currentRightClearance },
                MaxAbsClearanceMeters = CurrentGroundMaxAbsClearanceMeters,
                SpreadMeters = _currentGroundSpread
            };
        }

        private int _traceSamples;
        private float _traceToeDotSum;
        private float _traceToeDotMin;
        private int _normalRomSamples;
        private float _flexAbsSum;
        private float _flexAbsMax;
        private float _axialAbsSum;
        private float _axialAbsMax;
        private float _inversionAbsSum;
        private float _inversionAbsMax;
        private readonly int[] _flexHist = new int[RomHistogramBins];
        private readonly int[] _axialHist = new int[RomHistogramBins];
        private readonly int[] _inversionHist = new int[RomHistogramBins];
        private int _idleGroundEligible;
        private int _idleGroundReady;
        private int _idleGroundNoHit;
        private float _idleCenterClearanceSum;
        private int _idleCenterClearanceSamples;
        private float _idleClearanceAbsMax;
        private float _idleSpreadMax;

        internal bool Observe(
            bool traceEnabled,
            string phase,
            Rigidbody footBody,
            BoxCollider footCollider,
            Transform sourceFoot,
            Transform sourceToe,
            Transform targetRoot,
            Transform ownerRoot,
            Quaternion bindSourceRelative,
            Quaternion sampledSourceRelative,
            Vector3 sampledFootLinearVelocityWorld,
            FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;

            if (footBody == null || footCollider == null || sourceFoot == null || sourceToe == null ||
                !FiniteQuaternion(bindSourceRelative) || !FiniteQuaternion(sampledSourceRelative) ||
                !FiniteVector(sampledFootLinearVelocityWorld))
                return Invalidate("MISSING_OR_NON_FINITE_SOLE_REQUAL_INPUT", report);

            Vector3 referenceUp = targetRoot != null ? targetRoot.up : Vector3.up;
            if (!FiniteVector(referenceUp) || referenceUp.sqrMagnitude < 0.0001f)
                referenceUp = Vector3.up;
            referenceUp.Normalize();

            Vector3 ankleToToe = sourceToe.position - sourceFoot.position;
            Vector3 soleForward = footBody.rotation * Vector3.forward;
            Vector3 soleUp = footBody.rotation * Vector3.up;
            if (!FiniteVector(ankleToToe) || ankleToToe.sqrMagnitude < 0.000001f ||
                !FiniteVector(soleForward) || soleForward.sqrMagnitude < 0.000001f ||
                !FiniteVector(soleUp) || soleUp.sqrMagnitude < 0.000001f)
                return Invalidate("NON_FINITE_OR_DEGENERATE_SOLE_FRAME", report);

            ankleToToe.Normalize();
            soleForward.Normalize();
            soleUp.Normalize();
            _currentToeForwardDot = Vector3.Dot(soleForward, ankleToToe);
            _currentSoleUpDotReferenceUp = Vector3.Dot(soleUp, referenceUp);

            Quaternion bindLocalDelta = Quaternion.Inverse(bindSourceRelative) * sampledSourceRelative;
            if (!FiniteQuaternion(bindLocalDelta))
                return Invalidate("NON_FINITE_BIND_SOLE_DELTA", report);
            _currentBindSoleDeltaDegrees = QuaternionToAngularVector(bindLocalDelta) * Mathf.Rad2Deg;
            if (!FiniteVector(_currentBindSoleDeltaDegrees))
                return Invalidate("NON_FINITE_BIND_SOLE_ROTATION_VECTOR", report);

            ProbeGround(footBody, footCollider, referenceUp, sourceFoot, targetRoot, ownerRoot);

            if (traceEnabled)
            {
                _traceSamples++;
                _traceToeDotSum += _currentToeForwardDot;
                if (_traceSamples == 1) _traceToeDotMin = _currentToeForwardDot;
                else _traceToeDotMin = Mathf.Min(_traceToeDotMin, _currentToeForwardDot);

                if (IsNormalRomPhase(phase))
                {
                    float flexAbs = Mathf.Abs(_currentBindSoleDeltaDegrees.x);
                    float axialAbs = Mathf.Abs(_currentBindSoleDeltaDegrees.y);
                    float inversionAbs = Mathf.Abs(_currentBindSoleDeltaDegrees.z);
                    _normalRomSamples++;
                    _flexAbsSum += flexAbs;
                    _axialAbsSum += axialAbs;
                    _inversionAbsSum += inversionAbs;
                    _flexAbsMax = Mathf.Max(_flexAbsMax, flexAbs);
                    _axialAbsMax = Mathf.Max(_axialAbsMax, axialAbs);
                    _inversionAbsMax = Mathf.Max(_inversionAbsMax, inversionAbs);
                    AddHistogram(_flexHist, flexAbs);
                    AddHistogram(_axialHist, axialAbs);
                    AddHistogram(_inversionHist, inversionAbs);
                }

                if (string.Equals(phase, "IDLE", StringComparison.OrdinalIgnoreCase) &&
                    sampledFootLinearVelocityWorld.magnitude <= IdleGroundMaxFootSpeed)
                {
                    _idleGroundEligible++;
                    if (_currentGroundHits == 0)
                        _idleGroundNoHit++;
                    if (_currentGroundReady)
                        _idleGroundReady++;
                    if (_currentCenterHit)
                    {
                        _idleCenterClearanceSamples++;
                        _idleCenterClearanceSum += _currentCenterClearance;
                    }
                    _idleClearanceAbsMax = Mathf.Max(_idleClearanceAbsMax, MaxCurrentAbsClearance());
                    _idleSpreadMax = Mathf.Max(_idleSpreadMax, _currentGroundSpread);
                }
            }

            CopyToReport(report);
            return true;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceSamples = 0;
            _traceToeDotSum = 0f;
            _traceToeDotMin = 0f;
            _normalRomSamples = 0;
            _flexAbsSum = 0f;
            _flexAbsMax = 0f;
            _axialAbsSum = 0f;
            _axialAbsMax = 0f;
            _inversionAbsSum = 0f;
            _inversionAbsMax = 0f;
            Array.Clear(_flexHist, 0, _flexHist.Length);
            Array.Clear(_axialHist, 0, _axialHist.Length);
            Array.Clear(_inversionHist, 0, _inversionHist.Length);
            _idleGroundEligible = 0;
            _idleGroundReady = 0;
            _idleGroundNoHit = 0;
            _idleCenterClearanceSum = 0f;
            _idleCenterClearanceSamples = 0;
            _idleClearanceAbsMax = 0f;
            _idleSpreadMax = 0f;
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            report.SoleRequalificationMode = Mode;
            report.SoleRequalificationFinite = _finite;
            report.SoleRequalificationInvalidReason = _invalidReason;
            report.SoleToeForwardDot = _currentToeForwardDot;
            report.SoleUpReferenceUpDot = _currentSoleUpDotReferenceUp;
            report.SourceBindSoleDeltaDegrees = _currentBindSoleDeltaDegrees;
            report.GroundGeometryReady = _currentGroundReady;
            report.GroundProbeHitCount = _currentGroundHits;
            report.GroundCenterClearanceMeters = _currentCenterHit ? _currentCenterClearance : float.PositiveInfinity;
            report.GroundClearanceMinMeters = _currentGroundHits > 0 ? _currentGroundMinClearance : float.PositiveInfinity;
            report.GroundClearanceMaxMeters = _currentGroundHits > 0 ? _currentGroundMaxClearance : float.PositiveInfinity;
            report.GroundClearanceSpreadMeters = _currentGroundSpread;
            report.GroundNormalDotReferenceUp = _currentGroundNormalDotUp;
            report.GroundFootprintMinNormalDotReferenceUp = _currentFootprintMinNormalDotUp;
            report.GroundAverageNormalWorld = _currentGroundAverageNormalWorld;
            report.GroundColliderPath = _currentGroundPath;
            report.GroundFootprintSameSurface = _currentFootprintSameSurface;
            report.GroundColliderInstanceId = _currentGroundColliderInstanceId;
            report.GroundColliderHasAttachedRigidbody = _currentGroundColliderHasAttachedRigidbody;
            // v0.4.1: this observer owns raycast/readiness only. The Foot foundation/contact-shadow layer owns
            // PhysicalGroundContactPossible so a status-time CopyToReport cannot overwrite an active B collider.

            report.SoleRequalificationCurrentSummary =
                "toeDot=" + F(_currentToeForwardDot) +
                "/upDot=" + F(_currentSoleUpDotReferenceUp) +
                "/romDeg(FlexX-AxialY-InvZ)=" + V(_currentBindSoleDeltaDegrees) +
                "/ground=" + (_currentGroundReady ? "GEOMETRY_READY" : (_currentGroundHits > 0 ? "NOT_READY" : "NO_HIT")) +
                "/hits=" + _currentGroundHits + "/5" +
                "/clearMm(C-T-H-L-R)=" + ClearanceText(_currentCenterHit, _currentCenterClearance) + "-" +
                    ClearanceText(_currentToeHit, _currentToeClearance) + "-" +
                    ClearanceText(_currentHeelHit, _currentHeelClearance) + "-" +
                    ClearanceText(_currentLeftHit, _currentLeftClearance) + "-" +
                    ClearanceText(_currentRightHit, _currentRightClearance) +
                "/minMaxSpreadMm=" + (_currentGroundHits > 0 ? F(_currentGroundMinClearance * 1000f) + "-" + F(_currentGroundMaxClearance * 1000f) + "-" + F(_currentGroundSpread * 1000f) : "NA-NA-NA") +
                "/normalDot=" + F(_currentGroundNormalDotUp) +
                "/footprintNormalMin=" + F(_currentFootprintMinNormalDotUp) +
                "/avgNormalWorld=" + V(_currentGroundAverageNormalWorld) +
                "/samePrimarySurface=" + _currentPrimarySameSurface +
                "/sameFootprintSurface=" + _currentFootprintSameSurface +
                "/surfaceId=" + _currentGroundColliderInstanceId +
                "/surfaceStatic=" + !_currentGroundColliderHasAttachedRigidbody +
                "/surface=" + Clean(_currentGroundPath) +
                "/probeOnly=True";

            float normalDenom = Mathf.Max(1, _normalRomSamples);
            float idleDenom = Mathf.Max(1, _idleGroundEligible);
            float centerDenom = Mathf.Max(1, _idleCenterClearanceSamples);
            report.SoleRequalificationTraceSummary =
                "n=" + _traceSamples +
                "/toeDotMeanMin=" + F(_traceToeDotSum / Mathf.Max(1, _traceSamples)) + "-" + F(_traceToeDotMin) +
                "/normalRomN=" + _normalRomSamples +
                "/flexAbs(mean-p95-max)=" + F(_flexAbsSum / normalDenom) + "-" + F(Percentile95(_flexHist, _normalRomSamples)) + "-" + F(_flexAbsMax) +
                "/axialAbs=" + F(_axialAbsSum / normalDenom) + "-" + F(Percentile95(_axialHist, _normalRomSamples)) + "-" + F(_axialAbsMax) +
                "/invAbs=" + F(_inversionAbsSum / normalDenom) + "-" + F(Percentile95(_inversionHist, _normalRomSamples)) + "-" + F(_inversionAbsMax) +
                "/idleGroundN=" + _idleGroundEligible +
                "/ready=" + _idleGroundReady +
                "/ratio=" + F(_idleGroundReady / idleDenom) +
                "/noHit=" + _idleGroundNoHit +
                "/centerClearMeanMm=" + F((_idleCenterClearanceSum / centerDenom) * 1000f) +
                "/clearAbsMaxMm=" + F(_idleClearanceAbsMax * 1000f) +
                "/spreadMaxMm=" + F(_idleSpreadMax * 1000f) +
                "/thresholds(clear-spread-normal-speed)=" + F(GroundReadyAbsClearanceMeters) + "-" + F(GroundReadySpreadMeters) + "-" + F(GroundNormalDotMin) + "-" + F(IdleGroundMaxFootSpeed) +
                "/probeOnly=True";
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            return "mode=" + Mode +
                   "/finite=" + _finite +
                   "/reason=" + (_invalidReason ?? "<none>") +
                   " | current=" + (report != null ? report.SoleRequalificationCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.SoleRequalificationTraceSummary : "<none>");
        }

        private void ProbeGround(
            Rigidbody footBody,
            BoxCollider footCollider,
            Vector3 referenceUp,
            Transform sourceFoot,
            Transform targetRoot,
            Transform ownerRoot)
        {
            Vector3 center = footCollider.center;
            Vector3 size = footCollider.size;
            float bottomY = center.y - size.y * 0.5f;
            float x = size.x * 0.40f;
            float z = size.z * 0.40f;

            GroundHit centerHit = ProbePoint(footBody.transform.TransformPoint(new Vector3(center.x, bottomY, center.z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit toeHit = ProbePoint(footBody.transform.TransformPoint(new Vector3(center.x, bottomY, center.z + z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit heelHit = ProbePoint(footBody.transform.TransformPoint(new Vector3(center.x, bottomY, center.z - z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit leftHit = ProbePoint(footBody.transform.TransformPoint(new Vector3(center.x - x, bottomY, center.z)), referenceUp, sourceFoot, targetRoot, ownerRoot);
            GroundHit rightHit = ProbePoint(footBody.transform.TransformPoint(new Vector3(center.x + x, bottomY, center.z)), referenceUp, sourceFoot, targetRoot, ownerRoot);

            _currentCenterHit = centerHit.Valid; _currentCenterClearance = centerHit.Clearance; _currentCenterCollider = centerHit.Collider;
            _currentToeHit = toeHit.Valid; _currentToeClearance = toeHit.Clearance; _currentToeCollider = toeHit.Collider;
            _currentHeelHit = heelHit.Valid; _currentHeelClearance = heelHit.Clearance; _currentHeelCollider = heelHit.Collider;
            _currentLeftHit = leftHit.Valid; _currentLeftClearance = leftHit.Clearance; _currentLeftCollider = leftHit.Collider;
            _currentRightHit = rightHit.Valid; _currentRightClearance = rightHit.Clearance; _currentRightCollider = rightHit.Collider;
            _currentGroundHits = (centerHit.Valid ? 1 : 0) + (toeHit.Valid ? 1 : 0) + (heelHit.Valid ? 1 : 0) + (leftHit.Valid ? 1 : 0) + (rightHit.Valid ? 1 : 0);

            GroundHit representative = centerHit.Valid ? centerHit :
                (toeHit.Valid ? toeHit : (heelHit.Valid ? heelHit : (leftHit.Valid ? leftHit : rightHit)));
            _currentGroundNormalDotUp = representative.Valid ? representative.NormalDotUp : 0f;
            _currentFootprintMinNormalDotUp = _currentGroundHits == 5
                ? Mathf.Min(centerHit.NormalDotUp, Mathf.Min(toeHit.NormalDotUp, Mathf.Min(heelHit.NormalDotUp, Mathf.Min(leftHit.NormalDotUp, rightHit.NormalDotUp))))
                : 0f;
            Vector3 normalSum = Vector3.zero;
            if (centerHit.Valid) normalSum += centerHit.NormalWorld;
            if (toeHit.Valid) normalSum += toeHit.NormalWorld;
            if (heelHit.Valid) normalSum += heelHit.NormalWorld;
            if (leftHit.Valid) normalSum += leftHit.NormalWorld;
            if (rightHit.Valid) normalSum += rightHit.NormalWorld;
            _currentGroundAverageNormalWorld = normalSum.sqrMagnitude > 0.00000001f ? normalSum.normalized : Vector3.zero;
            _currentGroundCollider = representative.Valid ? representative.Collider : null;
            _currentGroundColliderInstanceId = _currentGroundCollider != null ? _currentGroundCollider.GetInstanceID() : 0;
            _currentGroundColliderHasAttachedRigidbody = _currentGroundCollider != null && _currentGroundCollider.attachedRigidbody != null;
            if (_currentGroundColliderInstanceId == 0)
            {
                _currentGroundPath = null;
            }
            else if (_cachedGroundColliderInstanceId == _currentGroundColliderInstanceId && !string.IsNullOrEmpty(_cachedGroundColliderPath))
            {
                _currentGroundPath = _cachedGroundColliderPath;
            }
            else
            {
                _cachedGroundColliderInstanceId = _currentGroundColliderInstanceId;
                _cachedGroundColliderPath = ColliderPath(representative.Collider);
                _currentGroundPath = _cachedGroundColliderPath;
            }
            _currentPrimarySameSurface = centerHit.Valid && toeHit.Valid && heelHit.Valid &&
                                         centerHit.Collider != null &&
                                         toeHit.Collider == centerHit.Collider &&
                                         heelHit.Collider == centerHit.Collider;
            _currentFootprintSameSurface = _currentGroundHits == 5 && centerHit.Collider != null &&
                                           toeHit.Collider == centerHit.Collider && heelHit.Collider == centerHit.Collider &&
                                           leftHit.Collider == centerHit.Collider && rightHit.Collider == centerHit.Collider;

            if (_currentGroundHits <= 0)
            {
                _currentGroundMinClearance = 0f;
                _currentGroundMaxClearance = 0f;
                _currentGroundSpread = 0f;
                _currentGroundAverageNormalWorld = Vector3.zero;
                _currentGroundReady = false;
                return;
            }

            float min = float.PositiveInfinity;
            float max = float.NegativeInfinity;
            AddClearance(centerHit, ref min, ref max);
            AddClearance(toeHit, ref min, ref max);
            AddClearance(heelHit, ref min, ref max);
            AddClearance(leftHit, ref min, ref max);
            AddClearance(rightHit, ref min, ref max);
            _currentGroundMinClearance = min;
            _currentGroundMaxClearance = max;
            _currentGroundSpread = Mathf.Max(0f, max - min);

            float maxAbs = MaxCurrentAbsClearance();
            bool primaryFootprint = centerHit.Valid && toeHit.Valid && heelHit.Valid && _currentPrimarySameSurface;
            _currentGroundReady = primaryFootprint && _currentGroundHits >= 3 &&
                                  maxAbs <= GroundReadyAbsClearanceMeters &&
                                  _currentGroundSpread <= GroundReadySpreadMeters &&
                                  _currentGroundNormalDotUp >= GroundNormalDotMin;
        }

        private GroundHit ProbePoint(Vector3 soleBottomPoint, Vector3 referenceUp, Transform sourceFoot, Transform targetRoot, Transform ownerRoot)
        {
            GroundHit result = new GroundHit();
            Vector3 direction = -referenceUp;
            Vector3 origin = soleBottomPoint + referenceUp * GroundProbeLiftMeters;
            float remaining = GroundProbeLiftMeters + GroundProbeBelowMeters;
            float travelled = 0f;

            for (int i = 0; i < MaxRejectedRayHits && remaining > 0.001f; i++)
            {
                RaycastHit hit;
                // V04410 exact-Collider queries avoid coincident original/Box scene-hit ambiguity.
                // A bound but invalid experiment never falls back to an unrelated scene collider.
                bool found = _supportSurface != null
                    ? _supportSurface.Raycast(origin, direction, remaining, out hit)
                    : UnityEngine.Physics.Raycast(origin, direction, out hit, remaining);
                if (!found)
                    return result;

                float distance = Mathf.Max(0f, hit.distance);
                travelled += distance;
                Collider collider = hit.collider;
                float normalDot = FiniteVector(hit.normal) ? Vector3.Dot(hit.normal.normalized, referenceUp) : -1f;
                if (IsGroundCandidate(collider, hit.normal, referenceUp, sourceFoot, targetRoot, ownerRoot))
                {
                    result.Valid = true;
                    result.Clearance = travelled - GroundProbeLiftMeters;
                    result.NormalDotUp = normalDot;
                    result.NormalWorld = hit.normal.normalized;
                    result.Collider = collider;
                    return result;
                }

                float advance = distance + RayAdvanceEpsilon;
                if (advance <= 0f) advance = RayAdvanceEpsilon;
                origin += direction * advance;
                travelled += RayAdvanceEpsilon;
                remaining -= advance;
            }
            return result;
        }

        private static bool IsGroundCandidate(Collider collider, Vector3 normal, Vector3 referenceUp, Transform sourceFoot, Transform targetRoot, Transform ownerRoot)
        {
            if (collider == null || !collider.enabled || collider.isTrigger)
                return false;
            Transform t = collider.transform;
            if (t == null)
                return false;
            if (ownerRoot != null && (t == ownerRoot || t.IsChildOf(ownerRoot)))
                return false;
            if (targetRoot != null && (t == targetRoot || t.IsChildOf(targetRoot)))
                return false;
            if (sourceFoot != null && (t == sourceFoot || t.IsChildOf(sourceFoot)))
                return false;
            string path = TransformPath(t);
            if (path.IndexOf("CapsulePlace Mita", StringComparison.OrdinalIgnoreCase) >= 0 ||
                path.IndexOf("CapsulePlaceMita", StringComparison.OrdinalIgnoreCase) >= 0)
                return false;
            if (!FiniteVector(normal) || normal.sqrMagnitude < 0.0001f)
                return false;
            float dot = Vector3.Dot(normal.normalized, referenceUp);
            return dot >= GroundNormalDotMin;
        }

        private static bool IsNormalRomPhase(string phase)
        {
            return string.Equals(phase, "IDLE", StringComparison.OrdinalIgnoreCase) ||
                   string.Equals(phase, "NORMAL_STEADY", StringComparison.OrdinalIgnoreCase) ||
                   string.Equals(phase, "NORMAL_TRANSITION", StringComparison.OrdinalIgnoreCase);
        }

        private static void AddClearance(GroundHit hit, ref float min, ref float max)
        {
            if (!hit.Valid) return;
            min = Mathf.Min(min, hit.Clearance);
            max = Mathf.Max(max, hit.Clearance);
        }

        private float MaxCurrentAbsClearance()
        {
            float max = 0f;
            if (_currentCenterHit) max = Mathf.Max(max, Mathf.Abs(_currentCenterClearance));
            if (_currentToeHit) max = Mathf.Max(max, Mathf.Abs(_currentToeClearance));
            if (_currentHeelHit) max = Mathf.Max(max, Mathf.Abs(_currentHeelClearance));
            if (_currentLeftHit) max = Mathf.Max(max, Mathf.Abs(_currentLeftClearance));
            if (_currentRightHit) max = Mathf.Max(max, Mathf.Abs(_currentRightClearance));
            return max;
        }

        private static void AddHistogram(int[] histogram, float absDegrees)
        {
            if (histogram == null || histogram.Length == 0) return;
            int index = Mathf.Clamp(Mathf.FloorToInt(Mathf.Abs(absDegrees)), 0, histogram.Length - 1);
            histogram[index]++;
        }

        private static float Percentile95(int[] histogram, int count)
        {
            if (histogram == null || count <= 0) return 0f;
            int target = Mathf.Max(1, Mathf.CeilToInt(count * 0.95f));
            int cumulative = 0;
            for (int i = 0; i < histogram.Length; i++)
            {
                cumulative += histogram[i];
                if (cumulative >= target)
                    return i + 0.5f;
            }
            return histogram.Length - 1;
        }

        private bool Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            _invalidReason = reason;
            CopyToReport(report);
            return false;
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

        private static string ClearanceText(bool hit, float clearance)
        {
            return hit ? F(clearance * 1000f) : "NA";
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace('\r', ' ').Replace('\n', ' ');
        }

        private static Vector3 QuaternionToAngularVector(Quaternion q)
        {
            if (q.w < 0f)
            {
                q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
            }
            float w = Mathf.Clamp(q.w, -1f, 1f);
            float angle = 2f * Mathf.Acos(w);
            if (angle > Mathf.PI) angle -= 2f * Mathf.PI;
            float sinHalf = Mathf.Sqrt(Mathf.Max(0f, 1f - w * w));
            if (sinHalf <= 0.0001f || Mathf.Abs(angle) <= 0.000001f)
                return Vector3.zero;
            return new Vector3(q.x / sinHalf, q.y / sinHalf, q.z / sinHalf) * angle;
        }

        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
        private static bool FiniteFloat(float value) => !(float.IsNaN(value) || float.IsInfinity(value));
        private static bool FiniteVector(Vector3 value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z);
        private static bool FiniteQuaternion(Quaternion value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z) && FiniteFloat(value.w);

        private struct GroundHit
        {
            public bool Valid;
            public float Clearance;
            public float NormalDotUp;
            public Vector3 NormalWorld;
            public Collider Collider;
        }
    }
}
