using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.0 Stage-E read-only support/contact identity qualification.
    /// Consumes the existing v0.3.56.1 five-point sole raycast witness; it performs no additional Physics queries
    /// and never enables colliders, gravity or solver contact.  The purpose is only to establish a stable, exact
    /// scene-Collider identity suitable for a later causal contact experiment.
    /// </summary>
    internal sealed class FootGroundContactIdentityQualificationObserver
    {
        internal const string Mode = "FOOT_GROUND_CONTACT_IDENTITY_QUALIFICATION_READ_ONLY_V0400";
        internal const float MinNormalDot = 0.90f;
        internal const float MaxAbsClearanceMeters = 0.025f;
        internal const float MaxSpreadMeters = 0.025f;
        internal const float QuietSpeedMax = 0.15f;
        internal const float MinStableSeconds = 0.50f;
        internal const int MinQuietEligibleSamples = 100;
        internal const float MinQuietQualifiedRatio = 0.80f;

        private bool _finite = true;
        private string _invalidReason;
        private bool _traceInvalidLatched;
        private string _traceInvalidReason;
        private bool _currentQualified;
        private bool _currentStaticCollider;
        private int _currentColliderId;
        private string _currentColliderPath;
        private float _currentStableSeconds;

        private int _quietEligible;
        private int _quietQualified;
        private int _quietNoHit;
        private int _quietDynamicRejected;
        private int _identitySwitches;
        private int _previousQuietQualifiedColliderId;
        private int _stableSegments;
        private float _longestStableSeconds;
        private int _segmentColliderId;
        private string _segmentColliderPath;
        private float _segmentSeconds;
        private bool _segmentAlreadyStable;
        private int _lastStableColliderId;
        private string _lastStableColliderPath;
        private bool _traceEndedStable;
        private int _endingStableColliderId;
        private string _endingStableColliderPath;
        private bool _observeTraceEnabled;

        internal void Observe(bool traceEnabled, string phase, Vector3 footVelocityWorld, float fixedDeltaTime, FootAnkleFoundationReport report)
        {
            _observeTraceEnabled = traceEnabled;
            _finite = true;
            _invalidReason = null;
            if (report == null || !FiniteVector(footVelocityWorld) || !FiniteFloat(fixedDeltaTime) || fixedDeltaTime <= 0f)
            {
                Invalidate("INVALID_CONTACT_IDENTITY_INPUT", report);
                return;
            }

            _currentStaticCollider = report.GroundColliderInstanceId != 0 && !report.GroundColliderHasAttachedRigidbody;
            _currentColliderId = report.GroundColliderInstanceId;
            _currentColliderPath = report.GroundColliderPath;
            _currentQualified = report.GroundProbeHitCount == 5 &&
                                report.GroundFootprintSameSurface &&
                                _currentStaticCollider &&
                                report.GroundFootprintMinNormalDotReferenceUp >= MinNormalDot &&
                                FiniteFloat(report.GroundCenterClearanceMeters) &&
                                Mathf.Abs(report.GroundCenterClearanceMeters) <= MaxAbsClearanceMeters &&
                                report.GroundClearanceSpreadMeters <= MaxSpreadMeters;

            bool quietEligible = string.Equals(phase, "IDLE", StringComparison.OrdinalIgnoreCase) &&
                                 footVelocityWorld.magnitude <= QuietSpeedMax;

            if (!traceEnabled || !quietEligible)
            {
                _segmentColliderId = 0;
                _segmentColliderPath = null;
                _segmentSeconds = 0f;
                _segmentAlreadyStable = false;
                _currentStableSeconds = 0f;
                CopyNumericToReport(report);
                return;
            }

            _quietEligible++;
            if (report.GroundProbeHitCount == 0) _quietNoHit++;
            if (report.GroundColliderInstanceId != 0 && report.GroundColliderHasAttachedRigidbody) _quietDynamicRejected++;

            if (!_currentQualified)
            {
                _segmentColliderId = 0;
                _segmentColliderPath = null;
                _segmentSeconds = 0f;
                _segmentAlreadyStable = false;
                _currentStableSeconds = 0f;
                CopyNumericToReport(report);
                return;
            }

            _quietQualified++;
            if (_previousQuietQualifiedColliderId != 0 && _previousQuietQualifiedColliderId != _currentColliderId)
                _identitySwitches++;
            _previousQuietQualifiedColliderId = _currentColliderId;
            if (_segmentColliderId == _currentColliderId && _segmentColliderId != 0)
            {
                _segmentSeconds += fixedDeltaTime;
            }
            else
            {
                _segmentColliderId = _currentColliderId;
                _segmentColliderPath = _currentColliderPath;
                _segmentSeconds = fixedDeltaTime;
                _segmentAlreadyStable = false;
            }

            _currentStableSeconds = _segmentSeconds;
            _longestStableSeconds = Mathf.Max(_longestStableSeconds, _segmentSeconds);
            if (!_segmentAlreadyStable && _segmentSeconds >= MinStableSeconds)
            {
                _segmentAlreadyStable = true;
                _stableSegments++;
                _lastStableColliderId = _segmentColliderId;
                _lastStableColliderPath = _segmentColliderPath;
            }
            else if (_segmentAlreadyStable)
            {
                _lastStableColliderId = _segmentColliderId;
                _lastStableColliderPath = _segmentColliderPath;
            }

            CopyNumericToReport(report);
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceInvalidLatched = false;
            _traceInvalidReason = null;
            _quietEligible = 0;
            _quietQualified = 0;
            _quietNoHit = 0;
            _quietDynamicRejected = 0;
            _identitySwitches = 0;
            _previousQuietQualifiedColliderId = 0;
            _stableSegments = 0;
            _longestStableSeconds = 0f;
            _segmentColliderId = 0;
            _segmentColliderPath = null;
            _segmentSeconds = 0f;
            _segmentAlreadyStable = false;
            _lastStableColliderId = 0;
            _lastStableColliderPath = null;
            _traceEndedStable = false;
            _endingStableColliderId = 0;
            _endingStableColliderPath = null;
            _currentStableSeconds = 0f;
            CopyNumericToReport(report);
        }


        internal void EndTrace(FootAnkleFoundationReport report)
        {
            bool effectiveFinite = _finite && !_traceInvalidLatched;
            _traceEndedStable = effectiveFinite && _currentQualified && _segmentAlreadyStable &&
                                _segmentColliderId != 0 && _segmentSeconds >= MinStableSeconds;
            _endingStableColliderId = _traceEndedStable ? _segmentColliderId : 0;
            _endingStableColliderPath = _traceEndedStable ? _segmentColliderPath : null;
            CopyNumericToReport(report);
        }

        private void CopyNumericToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            float ratio = _quietQualified / (float)Mathf.Max(1, _quietEligible);
            bool effectiveFinite = _finite && !_traceInvalidLatched;
            bool traceQualified = effectiveFinite && _quietEligible >= MinQuietEligibleSamples &&
                                  ratio >= MinQuietQualifiedRatio &&
                                  _longestStableSeconds >= MinStableSeconds &&
                                  _lastStableColliderId != 0;
            report.ContactIdentityMode = Mode;
            report.ContactIdentityFinite = effectiveFinite;
            report.ContactIdentityInvalidReason = _traceInvalidLatched ? _traceInvalidReason : _invalidReason;
            report.ContactIdentityCurrentQualified = _currentQualified;
            report.ContactIdentityCurrentStaticCollider = _currentStaticCollider;
            report.ContactIdentityCurrentColliderInstanceId = _currentColliderId;
            report.ContactIdentityCurrentColliderPath = _currentColliderPath;
            report.ContactIdentityCurrentStableSeconds = _currentStableSeconds;
            report.ContactIdentityTraceQuietEligibleSamples = _quietEligible;
            report.ContactIdentityTraceQuietQualifiedSamples = _quietQualified;
            report.ContactIdentityTraceQuietQualifiedRatio = ratio;
            report.ContactIdentityTraceNoHitSamples = _quietNoHit;
            report.ContactIdentityTraceDynamicRejectedSamples = _quietDynamicRejected;
            report.ContactIdentityTraceSwitches = _identitySwitches;
            report.ContactIdentityTraceStableSegments = _stableSegments;
            report.ContactIdentityTraceLongestStableSeconds = _longestStableSeconds;
            report.ContactIdentityLastStableColliderInstanceId = _lastStableColliderId;
            report.ContactIdentityLastStableColliderPath = _lastStableColliderPath;
            report.ContactIdentityTraceEndedStable = _traceEndedStable;
            report.ContactIdentityEndingStableColliderInstanceId = _endingStableColliderId;
            report.ContactIdentityEndingStableColliderPath = _endingStableColliderPath;
            report.ContactIdentityTraceQualified = traceQualified;
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            CopyNumericToReport(report);
            float ratio = report.ContactIdentityTraceQuietQualifiedRatio;
            bool traceQualified = report.ContactIdentityTraceQualified;
            report.ContactIdentityCurrentSummary =
                "qualified=" + _currentQualified +
                "/static=" + _currentStaticCollider +
                "/id=" + _currentColliderId +
                "/stableSec=" + F(_currentStableSeconds) +
                "/surface=" + Clean(_currentColliderPath);
            report.ContactIdentityTraceSummary =
                "quiet=" + _quietEligible +
                "/qualified=" + _quietQualified +
                "/ratio=" + F(ratio) +
                "/noHit=" + _quietNoHit +
                "/dynamicReject=" + _quietDynamicRejected +
                "/switches=" + _identitySwitches +
                "/stableSegments=" + _stableSegments +
                "/longestStableSec=" + F(_longestStableSeconds) +
                "/lastStableId=" + _lastStableColliderId +
                "/lastStableSurface=" + Clean(_lastStableColliderPath) +
                "/endedStable=" + _traceEndedStable +
                "/endingStableId=" + _endingStableColliderId +
                "/endingStableSurface=" + Clean(_endingStableColliderPath) +
                "/TRACE_IDENTITY_QUALIFIED=" + traceQualified +
                "/thresholds(quiet-ratio-stableSec)=" + MinQuietEligibleSamples + "-" + F(MinQuietQualifiedRatio) + "-" + F(MinStableSeconds) +
                "/identityEvidenceOnly=True";
        }

        internal string GetStatus(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            bool effectiveFinite = _finite && !_traceInvalidLatched;
            string effectiveReason = _traceInvalidLatched ? _traceInvalidReason : _invalidReason;
            return "mode=" + Mode +
                   "/finite=" + effectiveFinite +
                   "/reason=" + (effectiveReason ?? "<none>") +
                   " | current=" + (report != null ? report.ContactIdentityCurrentSummary : "<none>") +
                   " | trace=" + (report != null ? report.ContactIdentityTraceSummary : "<none>");
        }

        private void Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            _invalidReason = reason;
            if (_observeTraceEnabled)
            {
                _traceInvalidLatched = true;
                if (string.IsNullOrEmpty(_traceInvalidReason)) _traceInvalidReason = reason;
            }
            _currentQualified = false;
            CopyNumericToReport(report);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace('\r', ' ').Replace('\n', ' ');
        }

        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static bool FiniteFloat(float value) => !(float.IsNaN(value) || float.IsInfinity(value));
        private static bool FiniteVector(Vector3 value) => FiniteFloat(value.x) && FiniteFloat(value.y) && FiniteFloat(value.z);
    }
}
