using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.14 narrow causal A/B for transmitting exact authorized STANCE planar ground authority
    /// one level upstream from Foot to the Physical Lower body.
    ///
    /// Historical v0.4.14 A/B froze the v0.4.13 plant while testing this authority. v0.4.16 promotes
    /// the B law as the single stance common-mode ground owner; Foot world-ground anti-slip is retired.
    /// Zero-friction contact, translation tether/capture, angular controllers, Gravity OFF and all hard gates remain frozen.
    /// A computes the exact same Lower-ankle ground-anchor command but applies no force.
    /// B applies only the bounded support-plane acceleration at Physical Lower Rigidbody COM via
    /// Rigidbody.AddForce(..., ForceMode.Acceleration). No AddForceAtPosition is used, so this layer adds
    /// no direct yaw couple by construction.
    ///
    /// The stance anchor is the Physical Lower ankle point captured at the beginning of each authorized
    /// contact episode. One-step support-normal readback gaps suppress authority for that FixedUpdate but
    /// do not destroy the episode anchor. Contact release or authorized-surface change starts a new anchor.
    /// </summary>
    internal sealed class FootStanceLowerGroundAnchorAuthority
    {
        internal const string Mode = "STANCE_LOWER_GROUND_ANCHOR_BOUNDED_PLANAR_AUTHORITY_V04140";
        internal const string ArmA = "A_V04140_FROZEN_V04130_PLANT_LOWER_GROUND_OBSERVER_ONLY";
        internal const string ArmB = "B_V04140_LOWER_COM_BOUNDED_PLANAR_GROUND_ANCHOR";
        internal const float Kp = 20f;
        internal const float Kd = 6f;
        internal const float MaxAcceleration = 12f;
        internal const int MinEligibleSamples = 30;
        internal const int MinAppliedSamples = 30;

        private const float Epsilon = 0.0000001f;

        private string _arm = ArmA;
        private bool _finite = true;
        private string _invalidReason;
        private bool _episodeActive;
        private int _episodeSurfaceId;
        private Vector3 _lowerAnchorWorld;

        private int _samples;
        private int _eligibleSamples;
        private int _supportNormalGapSamples;
        private int _episodesStarted;
        private int _appliedSamples;
        private int _clampSamples;
        private int _nonStanceAppliedSamples;
        private int _firstAppliedFixed;
        private int _lastAppliedFixed;
        private float _positionErrorSum;
        private float _positionErrorMax;
        private float _velocityErrorSum;
        private float _velocityErrorMax;
        private float _rawAccelerationSum;
        private float _rawAccelerationMax;
        private float _appliedAccelerationSum;
        private float _appliedAccelerationMax;

        internal string Arm => _arm;
        internal bool Enabled => string.Equals(_arm, ArmB, StringComparison.Ordinal);
        internal bool Finite => _finite;

        internal bool SetArm(string arm, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, ArmA, StringComparison.Ordinal) && !string.Equals(arm, ArmB, StringComparison.Ordinal))
            {
                error = "UNKNOWN_LOWER_GROUND_AUTHORITY_ARM:" + (arm ?? "<null>");
                return false;
            }

            _arm = arm;
            _finite = true;
            _invalidReason = null;
            ResetEpisode();
            CopyToReport(report, Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero, false, false);
            return true;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            ResetEpisode();
            _samples = 0;
            _eligibleSamples = 0;
            _supportNormalGapSamples = 0;
            _episodesStarted = 0;
            _appliedSamples = 0;
            _clampSamples = 0;
            _nonStanceAppliedSamples = 0;
            _firstAppliedFixed = 0;
            _lastAppliedFixed = 0;
            _positionErrorSum = 0f;
            _positionErrorMax = 0f;
            _velocityErrorSum = 0f;
            _velocityErrorMax = 0f;
            _rawAccelerationSum = 0f;
            _rawAccelerationMax = 0f;
            _appliedAccelerationSum = 0f;
            _appliedAccelerationMax = 0f;
            CopyToReport(report, Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero, false, false);
        }

        internal bool Step(
            bool traceEnabled,
            int fixedSequence,
            bool stanceContactActive,
            bool supportNormalValid,
            bool zeroFriction,
            int authorizedColliderId,
            Rigidbody lowerBody,
            Vector3 lowerAnkleAnchorLocal,
            Vector3 supportNormalWorld,
            FootAnkleFoundationReport report)
        {
            if (traceEnabled) _samples++;

            bool contactWindow = stanceContactActive && zeroFriction;
            if (!contactWindow)
            {
                ResetEpisode();
                CopyToReport(report, Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero, false, false);
                return true;
            }

            if (lowerBody == null || lowerBody.isKinematic || !IsFinite(lowerAnkleAnchorLocal) ||
                !IsFinite(lowerBody.position) || !IsFinite(lowerBody.rotation) || !IsFinite(lowerBody.velocity) || !IsFinite(lowerBody.angularVelocity))
                return Invalidate("NONFINITE_OR_MISSING_LOWER_GROUND_AUTHORITY_BODY", report);

            if (authorizedColliderId == 0)
                return Invalidate("AUTHORIZED_SUPPORT_ID_MISSING_DURING_LOWER_GROUND_AUTHORITY", report);

            Vector3 lowerPointWorld = lowerBody.transform.TransformPoint(lowerAnkleAnchorLocal);
            Vector3 leverWorld = lowerPointWorld - lowerBody.worldCenterOfMass;
            Vector3 lowerPointVelocityWorld = lowerBody.velocity + Vector3.Cross(lowerBody.angularVelocity, leverWorld);
            if (!IsFinite(lowerPointWorld) || !IsFinite(lowerPointVelocityWorld))
                return Invalidate("NONFINITE_LOWER_ANKLE_POINT_STATE", report);

            if (_episodeActive && _episodeSurfaceId != authorizedColliderId)
                ResetEpisode();

            if (!supportNormalValid)
            {
                if (traceEnabled) _supportNormalGapSamples++;
                CopyToReport(report, Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero, false, false);
                return true;
            }

            if (!IsFinite(supportNormalWorld) || supportNormalWorld.sqrMagnitude <= Epsilon)
                return Invalidate("NONFINITE_OR_ZERO_LOWER_GROUND_SUPPORT_NORMAL", report);

            if (!_episodeActive)
            {
                _episodeActive = true;
                _episodeSurfaceId = authorizedColliderId;
                _lowerAnchorWorld = lowerPointWorld;
                if (traceEnabled) _episodesStarted++;
            }

            Vector3 n = supportNormalWorld.normalized;
            Vector3 positionErrorPlanar = Vector3.ProjectOnPlane(_lowerAnchorWorld - lowerPointWorld, n);
            Vector3 velocityErrorPlanar = Vector3.ProjectOnPlane(-lowerPointVelocityWorld, n);
            Vector3 rawAcceleration = Kp * positionErrorPlanar + Kd * velocityErrorPlanar;
            float rawMagnitude = rawAcceleration.magnitude;
            bool clamped = rawMagnitude > MaxAcceleration;
            Vector3 boundedAcceleration = clamped && rawMagnitude > Epsilon
                ? rawAcceleration * (MaxAcceleration / rawMagnitude)
                : rawAcceleration;

            if (!IsFinite(positionErrorPlanar) || !IsFinite(velocityErrorPlanar) || !IsFinite(rawAcceleration) || !IsFinite(boundedAcceleration))
                return Invalidate("NONFINITE_LOWER_GROUND_AUTHORITY_COMMAND", report);

            if (traceEnabled)
            {
                _eligibleSamples++;
                float posMag = positionErrorPlanar.magnitude;
                float velMag = velocityErrorPlanar.magnitude;
                _positionErrorSum += posMag;
                _velocityErrorSum += velMag;
                _rawAccelerationSum += rawMagnitude;
                _positionErrorMax = Mathf.Max(_positionErrorMax, posMag);
                _velocityErrorMax = Mathf.Max(_velocityErrorMax, velMag);
                _rawAccelerationMax = Mathf.Max(_rawAccelerationMax, rawMagnitude);
                // Count command saturation in both causal arms. Arm A computes the exact same bounded
                // command as B but does not write it, so this counter remains directly comparable.
                if (clamped) _clampSamples++;
            }

            bool applied = Enabled && boundedAcceleration.sqrMagnitude > Epsilon * Epsilon;
            if (applied)
            {
                // Intentionally apply at Physical Lower COM. The point being regulated is the Lower ankle point,
                // but v0.4.14 does not introduce a direct moment via AddForceAtPosition.
                lowerBody.AddForce(boundedAcceleration, ForceMode.Acceleration);
                if (traceEnabled)
                {
                    _appliedSamples++;
                    if (!stanceContactActive) _nonStanceAppliedSamples++;
                    _appliedAccelerationSum += boundedAcceleration.magnitude;
                    _appliedAccelerationMax = Mathf.Max(_appliedAccelerationMax, boundedAcceleration.magnitude);
                    if (_firstAppliedFixed == 0) _firstAppliedFixed = fixedSequence;
                    _lastAppliedFixed = fixedSequence;
                }
            }

            CopyToReport(report, positionErrorPlanar, velocityErrorPlanar, rawAcceleration,
                applied ? boundedAcceleration : Vector3.zero, clamped, applied);
            return true;
        }

        internal void EndTrace(FootAnkleFoundationReport report)
        {
            CopyToReport(
                report,
                report != null ? report.StanceLowerGroundAuthorityPositionErrorPlanarWorld : Vector3.zero,
                report != null ? report.StanceLowerGroundAuthorityVelocityErrorPlanarWorld : Vector3.zero,
                report != null ? report.StanceLowerGroundAuthorityRawAccelerationWorld : Vector3.zero,
                report != null ? report.StanceLowerGroundAuthorityAppliedAccelerationWorld : Vector3.zero,
                report != null && report.StanceLowerGroundAuthorityCurrentClamped,
                report != null && report.StanceLowerGroundAuthorityCurrentApplied);
        }

        internal void CopyToReport(
            FootAnkleFoundationReport report,
            Vector3 positionErrorPlanar,
            Vector3 velocityErrorPlanar,
            Vector3 rawAcceleration,
            Vector3 appliedAcceleration,
            bool clamped,
            bool applied)
        {
            if (report == null) return;
            int eligibleN = Mathf.Max(1, _eligibleSamples);
            int appliedN = Mathf.Max(1, _appliedSamples);
            report.StanceLowerGroundAuthorityMode = Mode;
            report.StanceLowerGroundAuthorityArm = _arm;
            report.StanceLowerGroundAuthorityFinite = _finite;
            report.StanceLowerGroundAuthorityInvalidReason = _invalidReason;
            report.StanceLowerGroundAuthorityEnabled = Enabled;
            report.StanceLowerGroundAuthorityKp = Kp;
            report.StanceLowerGroundAuthorityKd = Kd;
            report.StanceLowerGroundAuthorityMaxAcceleration = MaxAcceleration;
            report.StanceLowerGroundAuthorityTraceSamples = _samples;
            report.StanceLowerGroundAuthorityEligibleSamples = _eligibleSamples;
            report.StanceLowerGroundAuthoritySupportNormalGapSamples = _supportNormalGapSamples;
            report.StanceLowerGroundAuthorityEpisodesStarted = _episodesStarted;
            report.StanceLowerGroundAuthorityAppliedSamples = _appliedSamples;
            report.StanceLowerGroundAuthorityClampSamples = _clampSamples;
            report.StanceLowerGroundAuthorityNonStanceAppliedSamples = _nonStanceAppliedSamples;
            report.StanceLowerGroundAuthorityPositionErrorMeanMeters = _eligibleSamples > 0 ? _positionErrorSum / eligibleN : 0f;
            report.StanceLowerGroundAuthorityPositionErrorMaxMeters = _positionErrorMax;
            report.StanceLowerGroundAuthorityVelocityErrorMean = _eligibleSamples > 0 ? _velocityErrorSum / eligibleN : 0f;
            report.StanceLowerGroundAuthorityVelocityErrorMax = _velocityErrorMax;
            report.StanceLowerGroundAuthorityRawAccelerationMean = _eligibleSamples > 0 ? _rawAccelerationSum / eligibleN : 0f;
            report.StanceLowerGroundAuthorityRawAccelerationMax = _rawAccelerationMax;
            report.StanceLowerGroundAuthorityAppliedAccelerationMean = _appliedSamples > 0 ? _appliedAccelerationSum / appliedN : 0f;
            report.StanceLowerGroundAuthorityAppliedAccelerationMax = _appliedAccelerationMax;
            report.StanceLowerGroundAuthorityPositionErrorPlanarWorld = positionErrorPlanar;
            report.StanceLowerGroundAuthorityVelocityErrorPlanarWorld = velocityErrorPlanar;
            report.StanceLowerGroundAuthorityRawAccelerationWorld = rawAcceleration;
            report.StanceLowerGroundAuthorityAppliedAccelerationWorld = appliedAcceleration;
            report.StanceLowerGroundAuthorityCurrentClamped = clamped;
            report.StanceLowerGroundAuthorityCurrentApplied = applied;
            report.StanceLowerGroundAuthorityDirectYawTorqueMax = 0f;
            report.StanceLowerGroundAuthorityFirstAppliedFixed = _firstAppliedFixed;
            report.StanceLowerGroundAuthorityLastAppliedFixed = _lastAppliedFixed;
            report.StanceLowerGroundAuthoritySummary =
                "mode=" + Mode + "/arm=" + _arm + "/finite=" + _finite + "/enabled=" + Enabled +
                "/n-eligible-gap-applied-clamp=" + _samples + "-" + _eligibleSamples + "-" + _supportNormalGapSamples + "-" + _appliedSamples + "-" + _clampSamples +
                "/episodesStarted=" + _episodesStarted + "/nonStanceApplied=" + _nonStanceAppliedSamples +
                "/lowerAnchorPosErrMeanMaxMm=" + F(report.StanceLowerGroundAuthorityPositionErrorMeanMeters * 1000f) + "-" + F(_positionErrorMax * 1000f) +
                "/lowerAnchorVelErrMeanMax=" + F(report.StanceLowerGroundAuthorityVelocityErrorMean) + "-" + F(_velocityErrorMax) +
                "/rawAMeanMax=" + F(report.StanceLowerGroundAuthorityRawAccelerationMean) + "-" + F(_rawAccelerationMax) +
                "/appliedAMeanMax=" + F(report.StanceLowerGroundAuthorityAppliedAccelerationMean) + "-" + F(_appliedAccelerationMax) +
                "/directYawTauMax=0/firstLastApplied=" + _firstAppliedFixed + "-" + _lastAppliedFixed +
                "/constants(Kp-Kd-MaxA)=" + F(Kp) + "-" + F(Kd) + "-" + F(MaxAcceleration) +
                "/reason=" + (string.IsNullOrEmpty(_invalidReason) ? "<none>" : _invalidReason);
        }

        private bool Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            CopyToReport(report, Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero, false, false);
            return false;
        }

        private void ResetEpisode()
        {
            _episodeActive = false;
            _episodeSurfaceId = 0;
            _lowerAnchorWorld = Vector3.zero;
        }

        private static bool IsFinite(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        private static bool IsFinite(Quaternion q) => IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
        private static bool IsFinite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string F(float v) => v.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
