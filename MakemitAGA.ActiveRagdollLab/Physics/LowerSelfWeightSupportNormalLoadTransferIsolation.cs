using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.21 independent support-normal Lower self-weight load-transfer causal A/B.
    ///
    /// Both arms freeze the v0.4.20 B environment: Foot + Lower exact-authorized-STANCE native gravity ON,
    /// V04200 support-normal equal/opposite Foot-tether reciprocity ON, zero-friction contact, V04140 planar
    /// Lower authority, V04171 predictive guard and every historical hard gate unchanged.
    ///
    /// A adds no new force. B represents an idealized axial ankle/support constraint for only the Lower's
    /// native self-weight component along the currently authorized support normal. The force is computed from
    /// real Physics.gravity and Lower mass, then applied as an equal-and-opposite ForceMode.Force pair:
    ///   Lower += +Fload, Foot += -Fload,
    /// where Fload = mLower * max(0,-dot(g,n)) * n.
    /// This does not create net system force; it transfers Lower self-weight to Foot/Ground. It deliberately
    /// excludes horizontal load, dynamic/inertial load, torque, PD gains, tracking authority and gate changes.
    /// </summary>
    internal sealed class LowerSelfWeightSupportNormalLoadTransferIsolation
    {
        internal const string Mode = "STANCE_LOWER_SELF_WEIGHT_SUPPORT_NORMAL_LOAD_TRANSFER_CAUSAL_AB_V04210";
        internal const string ArmA = "A_V04210_V04200_B_RECIPROCAL_ONLY";
        internal const string ArmB = "B_V04210_LOWER_SELF_WEIGHT_SUPPORT_NORMAL_TRANSFER";
        internal const int MinEligibleSamples = 30;
        internal const int MinAppliedSamples = 30;

        private const float Epsilon = 0.0000001f;

        private string _arm = ArmA;
        private bool _finite = true;
        private string _invalidReason;
        private int _traceSamples;
        private int _eligibleSamples;
        private int _appliedSamples;
        private int _nonStanceAppliedSamples;
        private int _firstAppliedFixed;
        private int _lastAppliedFixed;
        private float _expectedForceSum;
        private float _expectedForceMax;
        private float _expectedImpulseSum;
        private float _expectedImpulseMax;
        private float _lowerSupportAccelerationSum;
        private float _lowerSupportAccelerationMax;
        private float _footLoadAccelerationAbsSum;
        private float _footLoadAccelerationAbsMax;
        private bool _currentApplied;
        private int _currentFixed;
        private float _currentLowerMass;
        private float _currentFootMass;
        private float _currentExpectedForce;
        private float _currentExpectedImpulse;
        private float _currentLowerSupportAcceleration;
        private float _currentFootLoadAcceleration;
        private Vector3 _currentTransferForceWorld;

        internal string Arm => _arm;
        internal bool Enabled => string.Equals(_arm, ArmB, StringComparison.Ordinal);
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;

        internal bool SetArm(string arm, SingleLegChainReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, ArmA, StringComparison.Ordinal) && !string.Equals(arm, ArmB, StringComparison.Ordinal))
            {
                error = "UNKNOWN_LOWER_SELF_WEIGHT_TRANSFER_ARM:" + (arm ?? "<null>");
                return false;
            }
            _arm = arm;
            _finite = true;
            _invalidReason = null;
            ClearCurrent();
            CopyToReport(report);
            return true;
        }

        internal void ResetTrace(SingleLegChainReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceSamples = 0;
            _eligibleSamples = 0;
            _appliedSamples = 0;
            _nonStanceAppliedSamples = 0;
            _firstAppliedFixed = 0;
            _lastAppliedFixed = 0;
            _expectedForceSum = 0f;
            _expectedForceMax = 0f;
            _expectedImpulseSum = 0f;
            _expectedImpulseMax = 0f;
            _lowerSupportAccelerationSum = 0f;
            _lowerSupportAccelerationMax = 0f;
            _footLoadAccelerationAbsSum = 0f;
            _footLoadAccelerationAbsMax = 0f;
            ClearCurrent();
            CopyToReport(report);
        }

        internal bool Step(
            bool traceEnabled,
            int fixedSequence,
            bool exactAuthorizedStance,
            bool zeroFriction,
            Rigidbody lowerBody,
            Rigidbody footBody,
            Vector3 supportNormalWorld,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            if (traceEnabled) _traceSamples++;
            ClearCurrent();

            if (lowerBody == null || footBody == null)
                return Invalidate("LOWER_SELF_WEIGHT_TRANSFER_BODY_MISSING", report);
            if (lowerBody.isKinematic || footBody.isKinematic ||
                !IsFinite(lowerBody.position) || !IsFinite(lowerBody.rotation) || !IsFinite(lowerBody.velocity) || !IsFinite(lowerBody.angularVelocity) ||
                !IsFinite(footBody.position) || !IsFinite(footBody.rotation) || !IsFinite(footBody.velocity) || !IsFinite(footBody.angularVelocity) ||
                !IsFinite(lowerBody.mass) || lowerBody.mass <= Epsilon || !IsFinite(footBody.mass) || footBody.mass <= Epsilon)
                return Invalidate("LOWER_SELF_WEIGHT_TRANSFER_BODY_STATE_INVALID", report);
            if (!IsFinite(fixedDeltaTime) || fixedDeltaTime <= Epsilon)
                return Invalidate("LOWER_SELF_WEIGHT_TRANSFER_FIXED_DT_INVALID", report);

            _currentLowerMass = lowerBody.mass;
            _currentFootMass = footBody.mass;

            if (!exactAuthorizedStance || !zeroFriction)
            {
                CopyToReport(report);
                return true;
            }
            if (!IsFinite(supportNormalWorld) || supportNormalWorld.sqrMagnitude <= Epsilon)
                return Invalidate("LOWER_SELF_WEIGHT_TRANSFER_SUPPORT_NORMAL_INVALID", report);

            Vector3 gravity = UnityEngine.Physics.gravity;
            if (!IsFinite(gravity))
                return Invalidate("LOWER_SELF_WEIGHT_TRANSFER_GRAVITY_INVALID", report);

            Vector3 n = supportNormalWorld.normalized;
            float expectedNormalAcceleration = Mathf.Max(0f, -Vector3.Dot(gravity, n));
            float expectedForce = lowerBody.mass * expectedNormalAcceleration;
            float expectedImpulse = expectedForce * fixedDeltaTime;
            Vector3 transferForce = expectedForce * n;
            float lowerSupportA = expectedForce / lowerBody.mass;
            float footLoadA = -expectedForce / footBody.mass;

            if (!IsFinite(expectedNormalAcceleration) || !IsFinite(expectedForce) || !IsFinite(expectedImpulse) ||
                !IsFinite(transferForce) || !IsFinite(lowerSupportA) || !IsFinite(footLoadA))
                return Invalidate("LOWER_SELF_WEIGHT_TRANSFER_ATTRIBUTION_NONFINITE", report);

            if (traceEnabled)
            {
                _eligibleSamples++;
                _expectedForceSum += expectedForce;
                _expectedForceMax = Mathf.Max(_expectedForceMax, expectedForce);
                _expectedImpulseSum += expectedImpulse;
                _expectedImpulseMax = Mathf.Max(_expectedImpulseMax, expectedImpulse);
                _lowerSupportAccelerationSum += lowerSupportA;
                _lowerSupportAccelerationMax = Mathf.Max(_lowerSupportAccelerationMax, Mathf.Abs(lowerSupportA));
                _footLoadAccelerationAbsSum += Mathf.Abs(footLoadA);
                _footLoadAccelerationAbsMax = Mathf.Max(_footLoadAccelerationAbsMax, Mathf.Abs(footLoadA));
            }

            _currentExpectedForce = expectedForce;
            _currentExpectedImpulse = expectedImpulse;
            _currentLowerSupportAcceleration = lowerSupportA;
            _currentFootLoadAcceleration = footLoadA;
            _currentTransferForceWorld = transferForce;

            if (Enabled)
            {
                bool lowerApplied = false;
                try
                {
                    lowerBody.AddForce(transferForce, ForceMode.Force);
                    lowerApplied = true;
                    footBody.AddForce(-transferForce, ForceMode.Force);
                }
                catch (Exception e)
                {
                    if (lowerApplied)
                    {
                        try { lowerBody.AddForce(-transferForce, ForceMode.Force); }
                        catch { }
                    }
                    return Invalidate("LOWER_SELF_WEIGHT_TRANSFER_FORCE_PAIR_EXCEPTION:" + e.GetType().Name, report);
                }

                _currentApplied = true;
                _currentFixed = fixedSequence;
                if (traceEnabled)
                {
                    _appliedSamples++;
                    if (_firstAppliedFixed == 0) _firstAppliedFixed = fixedSequence;
                    _lastAppliedFixed = fixedSequence;
                }
            }

            CopyToReport(report);
            return true;
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            float denom = Mathf.Max(1, _eligibleSamples);
            report.StanceLowerSelfWeightTransferMode = Mode;
            report.StanceLowerSelfWeightTransferArm = _arm;
            report.StanceLowerSelfWeightTransferFinite = _finite;
            report.StanceLowerSelfWeightTransferInvalidReason = _invalidReason;
            report.StanceLowerSelfWeightTransferEnabled = Enabled;
            report.StanceLowerSelfWeightTransferTraceSamples = _traceSamples;
            report.StanceLowerSelfWeightTransferEligibleSamples = _eligibleSamples;
            report.StanceLowerSelfWeightTransferAppliedSamples = _appliedSamples;
            report.StanceLowerSelfWeightTransferNonStanceAppliedSamples = _nonStanceAppliedSamples;
            report.StanceLowerSelfWeightTransferExpectedForceMean = _eligibleSamples > 0 ? _expectedForceSum / denom : 0f;
            report.StanceLowerSelfWeightTransferExpectedForceMax = _expectedForceMax;
            report.StanceLowerSelfWeightTransferExpectedImpulseMean = _eligibleSamples > 0 ? _expectedImpulseSum / denom : 0f;
            report.StanceLowerSelfWeightTransferExpectedImpulseMax = _expectedImpulseMax;
            report.StanceLowerSelfWeightTransferLowerSupportAccelerationMean = _eligibleSamples > 0 ? _lowerSupportAccelerationSum / denom : 0f;
            report.StanceLowerSelfWeightTransferLowerSupportAccelerationMax = _lowerSupportAccelerationMax;
            report.StanceLowerSelfWeightTransferFootLoadAccelerationAbsMean = _eligibleSamples > 0 ? _footLoadAccelerationAbsSum / denom : 0f;
            report.StanceLowerSelfWeightTransferFootLoadAccelerationAbsMax = _footLoadAccelerationAbsMax;
            report.StanceLowerSelfWeightTransferCurrentApplied = _currentApplied;
            report.StanceLowerSelfWeightTransferCurrentFixed = _currentFixed;
            report.StanceLowerSelfWeightTransferCurrentLowerMass = _currentLowerMass;
            report.StanceLowerSelfWeightTransferCurrentFootMass = _currentFootMass;
            report.StanceLowerSelfWeightTransferCurrentExpectedForce = _currentExpectedForce;
            report.StanceLowerSelfWeightTransferCurrentExpectedImpulse = _currentExpectedImpulse;
            report.StanceLowerSelfWeightTransferCurrentLowerSupportAcceleration = _currentLowerSupportAcceleration;
            report.StanceLowerSelfWeightTransferCurrentFootLoadAcceleration = _currentFootLoadAcceleration;
            report.StanceLowerSelfWeightTransferCurrentForceWorld = _currentTransferForceWorld;
            report.StanceLowerSelfWeightTransferFirstAppliedFixed = _firstAppliedFixed;
            report.StanceLowerSelfWeightTransferLastAppliedFixed = _lastAppliedFixed;
            report.StanceLowerSelfWeightTransferSummary =
                "mode=" + Mode +
                "/arm=" + _arm +
                "/finite=" + _finite +
                "/enabled=" + Enabled +
                "/n-eligible-applied-nonStance=" + _traceSamples + "-" + _eligibleSamples + "-" + _appliedSamples + "-" + _nonStanceAppliedSamples +
                "/expectedForceMeanMaxN=" + F(report.StanceLowerSelfWeightTransferExpectedForceMean) + "-" + F(report.StanceLowerSelfWeightTransferExpectedForceMax) +
                "/expectedImpulseMeanMaxNs=" + F(report.StanceLowerSelfWeightTransferExpectedImpulseMean) + "-" + F(report.StanceLowerSelfWeightTransferExpectedImpulseMax) +
                "/lowerSupportAMeanMax=" + F(report.StanceLowerSelfWeightTransferLowerSupportAccelerationMean) + "-" + F(report.StanceLowerSelfWeightTransferLowerSupportAccelerationMax) +
                "/footLoadAAbsMeanMax=" + F(report.StanceLowerSelfWeightTransferFootLoadAccelerationAbsMean) + "-" + F(report.StanceLowerSelfWeightTransferFootLoadAccelerationAbsMax) +
                "/current(applied-fixed-lowerMass-footMass-force-impulse-lowerA-footA-F)=" + _currentApplied + "-" + _currentFixed + "-" + F(_currentLowerMass) + "-" + F(_currentFootMass) + "-" + F(_currentExpectedForce) + "-" + F(_currentExpectedImpulse) + "-" + F(_currentLowerSupportAcceleration) + "-" + F(_currentFootLoadAcceleration) + "-" + V(_currentTransferForceWorld) +
                "/firstLastApplied=" + _firstAppliedFixed + "-" + _lastAppliedFixed +
                "/topology=LOWER_SELF_WEIGHT_SUPPORT_NORMAL_EQUAL_OPPOSITE_FORCE_PAIR" +
                "/reason=" + (_invalidReason ?? "<none>");
        }

        private bool Invalidate(string reason, SingleLegChainReport report)
        {
            _finite = false;
            _invalidReason = reason;
            ClearCurrent();
            CopyToReport(report);
            return false;
        }

        private void ClearCurrent()
        {
            _currentApplied = false;
            _currentFixed = 0;
            _currentLowerMass = 0f;
            _currentFootMass = 0f;
            _currentExpectedForce = 0f;
            _currentExpectedImpulse = 0f;
            _currentLowerSupportAcceleration = 0f;
            _currentFootLoadAcceleration = 0f;
            _currentTransferForceWorld = Vector3.zero;
        }

        private static bool IsFinite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static bool IsFinite(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        private static bool IsFinite(Quaternion q) => IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
        private static string F(float value) => value.ToString("G9", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
