using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.17 native-gravity selector; v0.4.18 qualified the Foot self-weight path and v0.4.19 freezes B as a support-load primitive.
    ///
    /// ArmA is retained only as safe rollback/off state. v0.4.19 formal A/B keeps this selector on ArmB in both arms.
    /// B enables native Unity gravity only on the Dynamic Foot Rigidbody, and only for an exact
    /// authorized zero-friction STANCE physics step with a valid support normal. Torso, Upper,
    /// Lower and GSP gravity remain OFF. This deliberately introduces only the Foot's own native
    /// self-weight before any upstream body-weight / full-body standing experiment.
    ///
    /// Controller math still executes with Foot gravity OFF. At the end of the FixedUpdate, after
    /// the frozen translation/angular controllers and pre-physics reaction witness have completed,
    /// B schedules Foot useGravity=true for the upcoming Unity solve. At the beginning of the next
    /// controller step gravity is cleared again before any frozen structural/controller contract is
    /// evaluated. SWING/transition/contact loss therefore returns gravity OFF in the same control step.
    ///
    /// Authorized collision callbacks additionally split support-normal impulse samples by whether
    /// native Foot gravity was active for that physics solve. This is observation only; no extra
    /// force/torque is injected and no comparison threshold is encoded into the physical plant.
    /// </summary>
    internal sealed class FootStanceNativeGravitySupportLoadIsolation
    {
        internal const string Mode = "STANCE_FOOT_NATIVE_GRAVITY_SUPPORT_LOAD_CAUSAL_AB_V04170";
        internal const string ArmA = "A_V04170_FROZEN_V04160_FOOT_GRAVITY_OFF";
        internal const string ArmB = "B_V04170_EXACT_STANCE_FOOT_NATIVE_GRAVITY_ON";
        internal const int MinEligibleSamples = 30;
        internal const int MinAppliedSamples = 30;
        internal const int MinReactionCallbackSamples = 10;

        private const float Epsilon = 0.0000001f;

        private string _arm = ArmA;
        private bool _finite = true;
        private string _invalidReason;

        private int _traceSamples;
        private int _eligibleSamples;
        private int _appliedSamples;
        private int _nonStanceAppliedSamples;
        private int _gravityReadbackFailures;
        private int _callbackSamples;
        private int _callbackGravityOnSamples;
        private int _callbackGravityOffSamples;
        private int _firstAppliedFixed;
        private int _lastAppliedFixed;

        private float _expectedNormalForceSum;
        private float _expectedNormalForceMax;
        private float _expectedNormalImpulseSum;
        private float _expectedNormalImpulseMax;
        private float _supportNormalSpeedAbsSum;
        private float _supportNormalSpeedAbsMax;
        private float _tetherSupportNormalAccelerationSum;
        private float _tetherSupportNormalAccelerationMin;
        private float _tetherSupportNormalAccelerationMax;
        private bool _tetherSupportNormalAccelerationHasSample;
        private float _gravityOnSignedNormalImpulseSum;
        private float _gravityOnSignedNormalImpulseMax;
        private float _gravityOffSignedNormalImpulseSum;
        private float _gravityOffSignedNormalImpulseMax;

        private Vector3 _currentGravityWorld;
        private float _currentFootMass;
        private bool _currentApplied;
        private int _currentScheduledFixed;

        internal string Arm => _arm;
        internal bool Enabled => string.Equals(_arm, ArmB, StringComparison.Ordinal);
        internal bool Finite => _finite;
        internal bool CurrentApplied => _currentApplied;

        internal bool SetArm(string arm, Rigidbody footBody, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, ArmA, StringComparison.Ordinal) && !string.Equals(arm, ArmB, StringComparison.Ordinal))
            {
                error = "UNKNOWN_FOOT_NATIVE_GRAVITY_ARM:" + (arm ?? "<null>");
                return false;
            }

            if (!ForceGravityOff(footBody, out error))
            {
                _finite = false;
                _invalidReason = "FOOT_GRAVITY_FAIL_CLOSED_READBACK:" + (error ?? "UNKNOWN");
                CopyToReport(report);
                return false;
            }

            _arm = arm;
            _finite = true;
            _invalidReason = null;
            _currentApplied = false;
            _currentScheduledFixed = 0;
            _currentGravityWorld = UnityEngine.Physics.gravity;
            _currentFootMass = footBody != null ? footBody.mass : 0f;
            CopyToReport(report);
            return true;
        }

        /// <summary>
        /// Clear the previous physics-step gravity before frozen Foot controllers inspect their
        /// structural contract. This is expected in B after a gravity-loaded stance solve.
        /// </summary>
        internal bool PrepareControllerStep(Rigidbody footBody, FootAnkleFoundationReport report)
        {
            if (footBody == null)
                return Invalidate("FOOT_NATIVE_GRAVITY_BODY_MISSING", report);

            if (footBody.useGravity)
            {
                try { footBody.useGravity = false; }
                catch (Exception e)
                {
                    return Invalidate("FOOT_NATIVE_GRAVITY_PRESTEP_DISABLE_EXCEPTION:" + e.GetType().Name, report);
                }
                if (footBody.useGravity)
                {
                    _gravityReadbackFailures++;
                    return Invalidate("FOOT_NATIVE_GRAVITY_PRESTEP_DISABLE_READBACK_FAILED", report);
                }
            }

            _currentApplied = false;
            _currentScheduledFixed = 0;
            CopyToReport(report);
            return true;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _traceSamples = 0;
            _eligibleSamples = 0;
            _appliedSamples = 0;
            _nonStanceAppliedSamples = 0;
            _gravityReadbackFailures = 0;
            _callbackSamples = 0;
            _callbackGravityOnSamples = 0;
            _callbackGravityOffSamples = 0;
            _firstAppliedFixed = 0;
            _lastAppliedFixed = 0;
            _expectedNormalForceSum = 0f;
            _expectedNormalForceMax = 0f;
            _expectedNormalImpulseSum = 0f;
            _expectedNormalImpulseMax = 0f;
            _supportNormalSpeedAbsSum = 0f;
            _supportNormalSpeedAbsMax = 0f;
            _tetherSupportNormalAccelerationSum = 0f;
            _tetherSupportNormalAccelerationMin = 0f;
            _tetherSupportNormalAccelerationMax = 0f;
            _tetherSupportNormalAccelerationHasSample = false;
            _gravityOnSignedNormalImpulseSum = 0f;
            _gravityOnSignedNormalImpulseMax = 0f;
            _gravityOffSignedNormalImpulseSum = 0f;
            _gravityOffSignedNormalImpulseMax = 0f;
            _currentGravityWorld = UnityEngine.Physics.gravity;
            _currentFootMass = 0f;
            _currentApplied = false;
            _currentScheduledFixed = 0;
            CopyToReport(report);
        }

        /// <summary>
        /// Called after every frozen Foot controller has completed for this FixedUpdate and immediately
        /// before the upcoming Unity solve. The only physical write in B is Rigidbody.useGravity.
        /// </summary>
        internal bool SchedulePhysicsStep(
            bool traceEnabled,
            int fixedSequence,
            bool stanceContactActive,
            bool supportNormalValid,
            bool zeroFriction,
            Rigidbody footBody,
            Vector3 supportNormalWorld,
            Vector3 tetherAppliedAccelerationWorld,
            float fixedDeltaTime,
            FootAnkleFoundationReport report)
        {
            if (traceEnabled) _traceSamples++;

            if (footBody == null || footBody.isKinematic || !IsFinite(footBody.position) || !IsFinite(footBody.rotation) ||
                !IsFinite(footBody.velocity) || !IsFinite(footBody.angularVelocity) || !IsFinite(footBody.mass) || footBody.mass <= Epsilon)
                return Invalidate("FOOT_NATIVE_GRAVITY_BODY_STATE_INVALID", report);

            Vector3 gravity = UnityEngine.Physics.gravity;
            if (!IsFinite(gravity) || !IsFinite(fixedDeltaTime) || fixedDeltaTime <= Epsilon)
                return Invalidate("FOOT_NATIVE_GRAVITY_ENVIRONMENT_INVALID", report);

            _currentGravityWorld = gravity;
            _currentFootMass = footBody.mass;

            bool eligible = stanceContactActive && supportNormalValid && zeroFriction;
            if (!eligible)
            {
                if (!EnsureGravityState(footBody, false, out string disableError))
                    return Invalidate("FOOT_NATIVE_GRAVITY_NONSTANCE_DISABLE_FAILED:" + disableError, report);
                _currentApplied = false;
                _currentScheduledFixed = 0;
                CopyToReport(report);
                return true;
            }

            if (!IsFinite(supportNormalWorld) || supportNormalWorld.sqrMagnitude <= Epsilon)
                return Invalidate("FOOT_NATIVE_GRAVITY_SUPPORT_NORMAL_INVALID", report);

            Vector3 n = supportNormalWorld.normalized;
            float expectedNormalAcceleration = Mathf.Max(0f, -Vector3.Dot(gravity, n));
            float expectedNormalForce = footBody.mass * expectedNormalAcceleration;
            float expectedNormalImpulse = expectedNormalForce * fixedDeltaTime;
            if (!IsFinite(tetherAppliedAccelerationWorld))
                return Invalidate("FOOT_NATIVE_GRAVITY_TETHER_ACCELERATION_INVALID", report);
            float supportNormalSpeedAbs = Mathf.Abs(Vector3.Dot(footBody.velocity, n));
            float tetherSupportNormalAcceleration = Vector3.Dot(tetherAppliedAccelerationWorld, n);
            if (!IsFinite(expectedNormalAcceleration) || !IsFinite(expectedNormalForce) || !IsFinite(expectedNormalImpulse) || !IsFinite(supportNormalSpeedAbs) || !IsFinite(tetherSupportNormalAcceleration))
                return Invalidate("FOOT_NATIVE_GRAVITY_EXPECTED_LOAD_INVALID", report);

            if (traceEnabled)
            {
                _eligibleSamples++;
                _expectedNormalForceSum += expectedNormalForce;
                _expectedNormalForceMax = Mathf.Max(_expectedNormalForceMax, expectedNormalForce);
                _expectedNormalImpulseSum += expectedNormalImpulse;
                _expectedNormalImpulseMax = Mathf.Max(_expectedNormalImpulseMax, expectedNormalImpulse);
                _supportNormalSpeedAbsSum += supportNormalSpeedAbs;
                _supportNormalSpeedAbsMax = Mathf.Max(_supportNormalSpeedAbsMax, supportNormalSpeedAbs);
                _tetherSupportNormalAccelerationSum += tetherSupportNormalAcceleration;
                if (!_tetherSupportNormalAccelerationHasSample)
                {
                    _tetherSupportNormalAccelerationMin = tetherSupportNormalAcceleration;
                    _tetherSupportNormalAccelerationMax = tetherSupportNormalAcceleration;
                    _tetherSupportNormalAccelerationHasSample = true;
                }
                else
                {
                    _tetherSupportNormalAccelerationMin = Mathf.Min(_tetherSupportNormalAccelerationMin, tetherSupportNormalAcceleration);
                    _tetherSupportNormalAccelerationMax = Mathf.Max(_tetherSupportNormalAccelerationMax, tetherSupportNormalAcceleration);
                }
            }

            bool requestedGravity = Enabled;
            if (!EnsureGravityState(footBody, requestedGravity, out string stateError))
                return Invalidate("FOOT_NATIVE_GRAVITY_WRITE_READBACK_FAILED:" + stateError, report);

            _currentApplied = requestedGravity;
            _currentScheduledFixed = requestedGravity ? fixedSequence : 0;
            if (traceEnabled && requestedGravity)
            {
                _appliedSamples++;
                if (!stanceContactActive || !supportNormalValid || !zeroFriction)
                    _nonStanceAppliedSamples++;
                if (_firstAppliedFixed == 0) _firstAppliedFixed = fixedSequence;
                _lastAppliedFixed = fixedSequence;
            }

            CopyToReport(report);
            return true;
        }

        /// <summary>
        /// Callback-time support reaction attribution. At this point useGravity still represents the
        /// state that Unity used for the just-solved physics step.
        /// </summary>
        internal void ObserveAuthorizedReaction(
            bool traceEnabled,
            Rigidbody footBody,
            Vector3 supportNormalWorld,
            float signedNormalImpulse,
            FootAnkleFoundationReport report)
        {
            if (!traceEnabled || !_finite || footBody == null || !IsFinite(signedNormalImpulse) ||
                !IsFinite(supportNormalWorld) || supportNormalWorld.sqrMagnitude <= Epsilon)
            {
                CopyToReport(report);
                return;
            }

            _callbackSamples++;
            if (footBody.useGravity)
            {
                _callbackGravityOnSamples++;
                _gravityOnSignedNormalImpulseSum += signedNormalImpulse;
                _gravityOnSignedNormalImpulseMax = Mathf.Max(_gravityOnSignedNormalImpulseMax, signedNormalImpulse);
            }
            else
            {
                _callbackGravityOffSamples++;
                _gravityOffSignedNormalImpulseSum += signedNormalImpulse;
                _gravityOffSignedNormalImpulseMax = Mathf.Max(_gravityOffSignedNormalImpulseMax, signedNormalImpulse);
            }
            CopyToReport(report);
        }

        internal bool QuiescePreserveTrace(Rigidbody footBody, FootAnkleFoundationReport report, out string error)
        {
            bool ok = ForceGravityOff(footBody, out error);
            if (!ok)
            {
                _gravityReadbackFailures++;
                _finite = false;
                if (string.IsNullOrEmpty(_invalidReason))
                    _invalidReason = "FOOT_NATIVE_GRAVITY_QUIESCE_FAILED:" + (error ?? "UNKNOWN");
            }
            _currentApplied = false;
            _currentScheduledFixed = 0;
            CopyToReport(report);
            return ok;
        }

        internal void EndTrace(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            int eligibleN = Mathf.Max(1, _eligibleSamples);
            int gravityOnCallbackN = Mathf.Max(1, _callbackGravityOnSamples);
            int gravityOffCallbackN = Mathf.Max(1, _callbackGravityOffSamples);

            report.StanceFootNativeGravityMode = Mode;
            report.StanceFootNativeGravityArm = _arm;
            report.StanceFootNativeGravityFinite = _finite;
            report.StanceFootNativeGravityInvalidReason = _invalidReason;
            report.StanceFootNativeGravityEnabled = Enabled;
            report.StanceFootNativeGravityTraceSamples = _traceSamples;
            report.StanceFootNativeGravityEligibleSamples = _eligibleSamples;
            report.StanceFootNativeGravityAppliedSamples = _appliedSamples;
            report.StanceFootNativeGravityNonStanceAppliedSamples = _nonStanceAppliedSamples;
            report.StanceFootNativeGravityReadbackFailures = _gravityReadbackFailures;
            report.StanceFootNativeGravityCallbackSamples = _callbackSamples;
            report.StanceFootNativeGravityCallbackGravityOnSamples = _callbackGravityOnSamples;
            report.StanceFootNativeGravityCallbackGravityOffSamples = _callbackGravityOffSamples;
            report.StanceFootNativeGravityExpectedNormalForceMean = _eligibleSamples > 0 ? _expectedNormalForceSum / eligibleN : 0f;
            report.StanceFootNativeGravityExpectedNormalForceMax = _expectedNormalForceMax;
            report.StanceFootNativeGravityExpectedNormalImpulseMean = _eligibleSamples > 0 ? _expectedNormalImpulseSum / eligibleN : 0f;
            report.StanceFootNativeGravityExpectedNormalImpulseMax = _expectedNormalImpulseMax;
            report.StanceFootNativeGravitySupportNormalSpeedAbsMean = _eligibleSamples > 0 ? _supportNormalSpeedAbsSum / eligibleN : 0f;
            report.StanceFootNativeGravitySupportNormalSpeedAbsMax = _supportNormalSpeedAbsMax;
            report.StanceFootNativeGravityTetherSupportNormalAccelerationMean = _eligibleSamples > 0 ? _tetherSupportNormalAccelerationSum / eligibleN : 0f;
            report.StanceFootNativeGravityTetherSupportNormalAccelerationMin = _tetherSupportNormalAccelerationHasSample ? _tetherSupportNormalAccelerationMin : 0f;
            report.StanceFootNativeGravityTetherSupportNormalAccelerationMax = _tetherSupportNormalAccelerationHasSample ? _tetherSupportNormalAccelerationMax : 0f;
            report.StanceFootNativeGravityGravityOnSignedNormalImpulseMean = _callbackGravityOnSamples > 0 ? _gravityOnSignedNormalImpulseSum / gravityOnCallbackN : 0f;
            report.StanceFootNativeGravityGravityOnSignedNormalImpulseMax = _gravityOnSignedNormalImpulseMax;
            report.StanceFootNativeGravityGravityOffSignedNormalImpulseMean = _callbackGravityOffSamples > 0 ? _gravityOffSignedNormalImpulseSum / gravityOffCallbackN : 0f;
            report.StanceFootNativeGravityGravityOffSignedNormalImpulseMax = _gravityOffSignedNormalImpulseMax;
            report.StanceFootNativeGravityGravityWorld = _currentGravityWorld;
            report.StanceFootNativeGravityGravityMagnitude = _currentGravityWorld.magnitude;
            report.StanceFootNativeGravityFootMass = _currentFootMass;
            report.StanceFootNativeGravityCurrentApplied = _currentApplied;
            report.StanceFootNativeGravityCurrentScheduledFixed = _currentScheduledFixed;
            report.StanceFootNativeGravityFirstAppliedFixed = _firstAppliedFixed;
            report.StanceFootNativeGravityLastAppliedFixed = _lastAppliedFixed;
            report.StanceFootNativeGravitySummary =
                "mode=" + Mode + "/arm=" + _arm + "/finite=" + _finite + "/enabled=" + Enabled +
                "/n-eligible-applied-nonStance-readbackFail=" + _traceSamples + "-" + _eligibleSamples + "-" + _appliedSamples + "-" + _nonStanceAppliedSamples + "-" + _gravityReadbackFailures +
                "/callbacks(total-gravityOn-gravityOff)=" + _callbackSamples + "-" + _callbackGravityOnSamples + "-" + _callbackGravityOffSamples +
                "/expectedNormalForceMeanMaxN=" + F(report.StanceFootNativeGravityExpectedNormalForceMean) + "-" + F(_expectedNormalForceMax) +
                "/expectedNormalImpulseMeanMaxNs=" + F(report.StanceFootNativeGravityExpectedNormalImpulseMean) + "-" + F(_expectedNormalImpulseMax) +
                "/supportNormalSpeedAbsMeanMax=" + F(report.StanceFootNativeGravitySupportNormalSpeedAbsMean) + "-" + F(_supportNormalSpeedAbsMax) +
                "/tetherSupportNormalA(mean-min-max)=" + F(report.StanceFootNativeGravityTetherSupportNormalAccelerationMean) + "-" + F(report.StanceFootNativeGravityTetherSupportNormalAccelerationMin) + "-" + F(report.StanceFootNativeGravityTetherSupportNormalAccelerationMax) +
                "/reactionSignedJMeanMax(gravityOn)=" + F(report.StanceFootNativeGravityGravityOnSignedNormalImpulseMean) + "-" + F(_gravityOnSignedNormalImpulseMax) +
                "/reactionSignedJMeanMax(gravityOff)=" + F(report.StanceFootNativeGravityGravityOffSignedNormalImpulseMean) + "-" + F(_gravityOffSignedNormalImpulseMax) +
                "/gravity=" + V(_currentGravityWorld) + "/gMag=" + F(_currentGravityWorld.magnitude) + "/mass=" + F(_currentFootMass) +
                "/currentApplied=" + _currentApplied + "/currentFixed=" + _currentScheduledFixed +
                "/firstLastApplied=" + _firstAppliedFixed + "-" + _lastAppliedFixed +
                "/reason=" + (string.IsNullOrEmpty(_invalidReason) ? "<none>" : _invalidReason);
        }

        private bool EnsureGravityState(Rigidbody footBody, bool enabled, out string error)
        {
            error = null;
            if (footBody == null)
            {
                error = "BODY_MISSING";
                _gravityReadbackFailures++;
                return false;
            }
            try
            {
                if (footBody.useGravity != enabled)
                    footBody.useGravity = enabled;
                if (footBody.useGravity != enabled)
                {
                    error = "READBACK_MISMATCH";
                    _gravityReadbackFailures++;
                    return false;
                }
                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + ":" + e.Message;
                _gravityReadbackFailures++;
                return false;
            }
        }

        private static bool ForceGravityOff(Rigidbody footBody, out string error)
        {
            error = null;
            if (footBody == null) return true;
            try
            {
                if (footBody.useGravity) footBody.useGravity = false;
                if (footBody.useGravity)
                {
                    error = "READBACK_STILL_TRUE";
                    return false;
                }
                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + ":" + e.Message;
                return false;
            }
        }

        private bool Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            CopyToReport(report);
            return false;
        }

        private static bool IsFinite(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        private static bool IsFinite(Quaternion q) => IsFinite(q.x) && IsFinite(q.y) && IsFinite(q.z) && IsFinite(q.w);
        private static bool IsFinite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
        private static string F(float v) => v.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")";
    }
}
