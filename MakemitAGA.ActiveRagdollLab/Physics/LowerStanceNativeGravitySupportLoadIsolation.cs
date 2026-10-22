using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.19 Lower native-gravity support-load causal A/B.
    ///
    /// Both arms freeze the v0.4.18 baseline: Foot native gravity is ON only for exact authorized
    /// zero-friction STANCE solves, V04171 predictive-tail protection is ON/B, Physical Lower planar
    /// ground authority remains 20/6/12, Upper/torso gravity and GSP remain OFF.
    ///
    /// A keeps Physical Lower native gravity OFF. B enables native gravity only on the Dynamic Lower
    /// Rigidbody for the same exact authorized STANCE solve. Controller math always starts with Lower
    /// gravity OFF; B schedules useGravity=true only after all frozen Hip/Knee/Foot controllers have
    /// completed for the upcoming Unity physics solve. The next FixedUpdate clears it before any
    /// controller/structural contract is evaluated.
    ///
    /// This layer does not add force/torque. It records the theoretical Lower self-weight impulse and
    /// the already-existing Knee translation tether's signed support-normal acceleration so the runtime
    /// audit can determine whether Lower load reaches Ground or is locally supported by the Knee tether.
    /// </summary>
    internal sealed class LowerStanceNativeGravitySupportLoadIsolation
    {
        internal const string Mode = "STANCE_LOWER_NATIVE_GRAVITY_SUPPORT_LOAD_CAUSAL_AB_V04190";
        internal const string ArmA = "A_V04190_FROZEN_V04180_FOOT_ONLY_GRAVITY";
        internal const string ArmB = "B_V04190_EXACT_STANCE_LOWER_NATIVE_GRAVITY_ON";
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
        private int _gravityReadbackFailures;
        private int _firstAppliedFixed;
        private int _lastAppliedFixed;

        private float _expectedNormalForceSum;
        private float _expectedNormalForceMax;
        private float _expectedNormalImpulseSum;
        private float _expectedNormalImpulseMax;
        private float _lowerSupportNormalSpeedAbsSum;
        private float _lowerSupportNormalSpeedAbsMax;
        private float _kneeTetherSupportNormalAccelerationSum;
        private float _kneeTetherSupportNormalAccelerationMin;
        private float _kneeTetherSupportNormalAccelerationMax;
        private bool _kneeTetherSupportNormalAccelerationHasSample;

        private Vector3 _currentGravityWorld;
        private float _currentLowerMass;
        private bool _currentApplied;
        private int _currentScheduledFixed;

        internal string Arm => _arm;
        internal bool Enabled => string.Equals(_arm, ArmB, StringComparison.Ordinal);
        internal bool Finite => _finite;
        internal bool CurrentApplied => _currentApplied;

        internal bool SetArm(string arm, Rigidbody lowerBody, SingleLegChainReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, ArmA, StringComparison.Ordinal) && !string.Equals(arm, ArmB, StringComparison.Ordinal))
            {
                error = "UNKNOWN_LOWER_NATIVE_GRAVITY_ARM:" + (arm ?? "<null>");
                return false;
            }

            if (!ForceGravityOff(lowerBody, out error))
            {
                _finite = false;
                _invalidReason = "LOWER_GRAVITY_FAIL_CLOSED_READBACK:" + (error ?? "UNKNOWN");
                CopyToReport(report);
                return false;
            }

            _arm = arm;
            _finite = true;
            _invalidReason = null;
            _currentApplied = false;
            _currentScheduledFixed = 0;
            _currentGravityWorld = UnityEngine.Physics.gravity;
            _currentLowerMass = lowerBody != null ? lowerBody.mass : 0f;
            CopyToReport(report);
            return true;
        }

        internal bool PrepareControllerStep(Rigidbody lowerBody, SingleLegChainReport report)
        {
            if (lowerBody == null)
                return Invalidate("LOWER_NATIVE_GRAVITY_BODY_MISSING", report);

            if (lowerBody.useGravity)
            {
                try { lowerBody.useGravity = false; }
                catch (Exception e)
                {
                    return Invalidate("LOWER_NATIVE_GRAVITY_PRESTEP_DISABLE_EXCEPTION:" + e.GetType().Name, report);
                }
                if (lowerBody.useGravity)
                {
                    _gravityReadbackFailures++;
                    return Invalidate("LOWER_NATIVE_GRAVITY_PRESTEP_DISABLE_READBACK_FAILED", report);
                }
            }

            _currentApplied = false;
            _currentScheduledFixed = 0;
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
            _gravityReadbackFailures = 0;
            _firstAppliedFixed = 0;
            _lastAppliedFixed = 0;
            _expectedNormalForceSum = 0f;
            _expectedNormalForceMax = 0f;
            _expectedNormalImpulseSum = 0f;
            _expectedNormalImpulseMax = 0f;
            _lowerSupportNormalSpeedAbsSum = 0f;
            _lowerSupportNormalSpeedAbsMax = 0f;
            _kneeTetherSupportNormalAccelerationSum = 0f;
            _kneeTetherSupportNormalAccelerationMin = 0f;
            _kneeTetherSupportNormalAccelerationMax = 0f;
            _kneeTetherSupportNormalAccelerationHasSample = false;
            _currentGravityWorld = UnityEngine.Physics.gravity;
            _currentLowerMass = 0f;
            _currentApplied = false;
            _currentScheduledFixed = 0;
            CopyToReport(report);
        }

        internal bool SchedulePhysicsStep(
            bool traceEnabled,
            int fixedSequence,
            bool exactAuthorizedStance,
            Rigidbody lowerBody,
            Vector3 supportNormalWorld,
            Vector3 kneeTetherAppliedAccelerationWorld,
            float fixedDeltaTime,
            SingleLegChainReport report)
        {
            if (traceEnabled) _traceSamples++;

            if (lowerBody == null || lowerBody.isKinematic || !IsFinite(lowerBody.position) || !IsFinite(lowerBody.rotation) ||
                !IsFinite(lowerBody.velocity) || !IsFinite(lowerBody.angularVelocity) || !IsFinite(lowerBody.mass) || lowerBody.mass <= Epsilon)
                return Invalidate("LOWER_NATIVE_GRAVITY_BODY_STATE_INVALID", report);

            Vector3 gravity = UnityEngine.Physics.gravity;
            if (!IsFinite(gravity) || !IsFinite(fixedDeltaTime) || fixedDeltaTime <= Epsilon)
                return Invalidate("LOWER_NATIVE_GRAVITY_ENVIRONMENT_INVALID", report);

            _currentGravityWorld = gravity;
            _currentLowerMass = lowerBody.mass;

            if (!exactAuthorizedStance)
            {
                if (lowerBody.useGravity || _currentApplied)
                    _nonStanceAppliedSamples++;
                if (!EnsureGravityState(lowerBody, false, out string disableError))
                    return Invalidate("LOWER_NATIVE_GRAVITY_NONSTANCE_DISABLE_FAILED:" + disableError, report);
                _currentApplied = false;
                _currentScheduledFixed = 0;
                CopyToReport(report);
                return true;
            }

            if (!IsFinite(supportNormalWorld) || supportNormalWorld.sqrMagnitude <= Epsilon)
                return Invalidate("LOWER_NATIVE_GRAVITY_SUPPORT_NORMAL_INVALID", report);
            if (!IsFinite(kneeTetherAppliedAccelerationWorld))
                return Invalidate("LOWER_NATIVE_GRAVITY_KNEE_TETHER_ACCELERATION_INVALID", report);

            Vector3 n = supportNormalWorld.normalized;
            float expectedNormalAcceleration = Mathf.Max(0f, -Vector3.Dot(gravity, n));
            float expectedNormalForce = lowerBody.mass * expectedNormalAcceleration;
            float expectedNormalImpulse = expectedNormalForce * fixedDeltaTime;
            float normalSpeedAbs = Mathf.Abs(Vector3.Dot(lowerBody.velocity, n));
            float kneeSupportA = Vector3.Dot(kneeTetherAppliedAccelerationWorld, n);
            if (!IsFinite(expectedNormalForce) || !IsFinite(expectedNormalImpulse) || !IsFinite(normalSpeedAbs) || !IsFinite(kneeSupportA))
                return Invalidate("LOWER_NATIVE_GRAVITY_ATTRIBUTION_NONFINITE", report);

            _eligibleSamples++;
            _expectedNormalForceSum += expectedNormalForce;
            _expectedNormalForceMax = Mathf.Max(_expectedNormalForceMax, expectedNormalForce);
            _expectedNormalImpulseSum += expectedNormalImpulse;
            _expectedNormalImpulseMax = Mathf.Max(_expectedNormalImpulseMax, expectedNormalImpulse);
            _lowerSupportNormalSpeedAbsSum += normalSpeedAbs;
            _lowerSupportNormalSpeedAbsMax = Mathf.Max(_lowerSupportNormalSpeedAbsMax, normalSpeedAbs);
            _kneeTetherSupportNormalAccelerationSum += kneeSupportA;
            if (!_kneeTetherSupportNormalAccelerationHasSample)
            {
                _kneeTetherSupportNormalAccelerationMin = kneeSupportA;
                _kneeTetherSupportNormalAccelerationMax = kneeSupportA;
                _kneeTetherSupportNormalAccelerationHasSample = true;
            }
            else
            {
                _kneeTetherSupportNormalAccelerationMin = Mathf.Min(_kneeTetherSupportNormalAccelerationMin, kneeSupportA);
                _kneeTetherSupportNormalAccelerationMax = Mathf.Max(_kneeTetherSupportNormalAccelerationMax, kneeSupportA);
            }

            bool requested = Enabled;
            if (!EnsureGravityState(lowerBody, requested, out string stateError))
                return Invalidate("LOWER_NATIVE_GRAVITY_SCHEDULE_READBACK_FAILED:" + stateError, report);

            _currentApplied = requested;
            _currentScheduledFixed = requested ? fixedSequence : 0;
            if (requested)
            {
                _appliedSamples++;
                if (_firstAppliedFixed == 0) _firstAppliedFixed = fixedSequence;
                _lastAppliedFixed = fixedSequence;
            }

            CopyToReport(report);
            return true;
        }

        internal bool QuiescePreserveTrace(Rigidbody lowerBody, SingleLegChainReport report, out string error)
        {
            bool ok = ForceGravityOff(lowerBody, out error);
            _currentApplied = false;
            _currentScheduledFixed = 0;
            if (!ok)
            {
                _finite = false;
                _invalidReason = "LOWER_NATIVE_GRAVITY_QUIESCE_FAILED:" + (error ?? "UNKNOWN");
            }
            CopyToReport(report);
            return ok;
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            float eligibleN = Mathf.Max(1, _eligibleSamples);
            report.StanceLowerNativeGravityMode = Mode;
            report.StanceLowerNativeGravityArm = _arm;
            report.StanceLowerNativeGravityFinite = _finite;
            report.StanceLowerNativeGravityInvalidReason = _invalidReason;
            report.StanceLowerNativeGravityEnabled = Enabled;
            report.StanceLowerNativeGravityTraceSamples = _traceSamples;
            report.StanceLowerNativeGravityEligibleSamples = _eligibleSamples;
            report.StanceLowerNativeGravityAppliedSamples = _appliedSamples;
            report.StanceLowerNativeGravityNonStanceAppliedSamples = _nonStanceAppliedSamples;
            report.StanceLowerNativeGravityReadbackFailures = _gravityReadbackFailures;
            report.StanceLowerNativeGravityExpectedNormalForceMean = _eligibleSamples > 0 ? _expectedNormalForceSum / eligibleN : 0f;
            report.StanceLowerNativeGravityExpectedNormalForceMax = _expectedNormalForceMax;
            report.StanceLowerNativeGravityExpectedNormalImpulseMean = _eligibleSamples > 0 ? _expectedNormalImpulseSum / eligibleN : 0f;
            report.StanceLowerNativeGravityExpectedNormalImpulseMax = _expectedNormalImpulseMax;
            report.StanceLowerNativeGravitySupportNormalSpeedAbsMean = _eligibleSamples > 0 ? _lowerSupportNormalSpeedAbsSum / eligibleN : 0f;
            report.StanceLowerNativeGravitySupportNormalSpeedAbsMax = _lowerSupportNormalSpeedAbsMax;
            report.StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMean = _eligibleSamples > 0 ? _kneeTetherSupportNormalAccelerationSum / eligibleN : 0f;
            report.StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMin = _kneeTetherSupportNormalAccelerationHasSample ? _kneeTetherSupportNormalAccelerationMin : 0f;
            report.StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMax = _kneeTetherSupportNormalAccelerationHasSample ? _kneeTetherSupportNormalAccelerationMax : 0f;
            report.StanceLowerNativeGravityGravityWorld = _currentGravityWorld;
            report.StanceLowerNativeGravityGravityMagnitude = _currentGravityWorld.magnitude;
            report.StanceLowerNativeGravityLowerMass = _currentLowerMass;
            report.StanceLowerNativeGravityCurrentApplied = _currentApplied;
            report.StanceLowerNativeGravityCurrentScheduledFixed = _currentScheduledFixed;
            report.StanceLowerNativeGravityFirstAppliedFixed = _firstAppliedFixed;
            report.StanceLowerNativeGravityLastAppliedFixed = _lastAppliedFixed;
            report.StanceLowerNativeGravitySummary =
                "mode=" + Mode +
                "/arm=" + _arm +
                "/finite=" + _finite +
                "/enabled=" + Enabled +
                "/n-eligible-applied-nonStance-readbackFail=" + _traceSamples + "-" + _eligibleSamples + "-" + _appliedSamples + "-" + _nonStanceAppliedSamples + "-" + _gravityReadbackFailures +
                "/expectedNormalForceMeanMaxN=" + F(report.StanceLowerNativeGravityExpectedNormalForceMean) + "-" + F(_expectedNormalForceMax) +
                "/expectedNormalImpulseMeanMaxNs=" + F(report.StanceLowerNativeGravityExpectedNormalImpulseMean) + "-" + F(_expectedNormalImpulseMax) +
                "/supportNormalSpeedAbsMeanMax=" + F(report.StanceLowerNativeGravitySupportNormalSpeedAbsMean) + "-" + F(_lowerSupportNormalSpeedAbsMax) +
                "/kneeTetherSupportNormalA(mean-min-max)=" + F(report.StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMean) + "-" + F(report.StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMin) + "-" + F(report.StanceLowerNativeGravityKneeTetherSupportNormalAccelerationMax) +
                "/gravity=" + V(_currentGravityWorld) +
                "/gMag=" + F(_currentGravityWorld.magnitude) +
                "/mass=" + F(_currentLowerMass) +
                "/currentApplied=" + _currentApplied +
                "/currentFixed=" + _currentScheduledFixed +
                "/firstLastApplied=" + _firstAppliedFixed + "-" + _lastAppliedFixed +
                "/reason=" + (_invalidReason ?? "<none>");
        }

        private bool Invalidate(string reason, SingleLegChainReport report)
        {
            _finite = false;
            _invalidReason = reason;
            _currentApplied = false;
            _currentScheduledFixed = 0;
            CopyToReport(report);
            return false;
        }

        private bool EnsureGravityState(Rigidbody body, bool enabled, out string error)
        {
            error = null;
            if (body == null)
            {
                error = "BODY_MISSING";
                return false;
            }
            try
            {
                if (body.useGravity != enabled)
                    body.useGravity = enabled;
                if (body.useGravity != enabled)
                {
                    _gravityReadbackFailures++;
                    error = "READBACK_MISMATCH";
                    return false;
                }
                return true;
            }
            catch (Exception e)
            {
                _gravityReadbackFailures++;
                error = e.GetType().Name;
                return false;
            }
        }

        private static bool ForceGravityOff(Rigidbody body, out string error)
        {
            error = null;
            if (body == null) return true;
            try
            {
                if (body.useGravity) body.useGravity = false;
                if (body.useGravity)
                {
                    error = "READBACK_STILL_ON";
                    return false;
                }
                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name;
                return false;
            }
        }

        private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        private static bool IsFinite(Quaternion value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
