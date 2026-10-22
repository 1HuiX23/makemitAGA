using System;
using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04260 causal A/B: a finite, passive Foot-only angular damping channel on exact static support.
    /// No attitude/source target, yaw torque, linear force, angularVelocity clamp or ContactPoint read.
    /// It can damp subsequent rocking; a zero-W pre-solve cannot anticipate a new contact impulse.
    /// Scalar minimum principal inertia keeps torque parallel to -W_perp, preserving zero normal torque
    /// even for anisotropic world inertia. This does not promise zero yaw angular acceleration: I^-1
    /// can couple axes. No native Joint or Lower reaction torque is introduced in this isolated experiment.
    /// </summary>
    internal sealed class FootStanceSupportNormalAngularStabilization
    {
        internal const string Mode = "STANCE_FOOT_SUPPORT_NORMAL_ANGULAR_STABILIZATION_CAUSAL_AB_V04260";
        internal const string ArmA = "A_V04260_SUPPORT_NORMAL_ANGULAR_DAMPING_OFF";
        internal const string ArmB = "B_V04260_BOUNDED_SUPPORT_NORMAL_ANGULAR_DAMPING";
        internal const float DecayRatePerSecond = 30f;
        internal const float MaxTorqueNm = 1f;
        internal const int MinimumEligibleSamples = 50;

        private string _arm = ArmA;
        private bool _finite = true;
        private string _reason;
        private int _eligibleSamples, _appliedSamples, _clampedSamples;
        private float _maxTorque, _maxNormalTorque, _maxPower;
        private float _workJoules;
        private Vector3 _normal, _perpW, _torque;
        private bool _eligible, _applied, _clamped;
        private int _fixedSequence;
        private float _dt;

        internal string Arm => _arm;
        internal bool Enabled => string.Equals(_arm, ArmB, StringComparison.Ordinal);
        internal bool Finite => _finite;
        internal Vector3 CurrentTorqueWorld => _torque;

        internal bool SetArm(string arm, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, ArmA, StringComparison.Ordinal) && !string.Equals(arm, ArmB, StringComparison.Ordinal))
            {
                error = "UNKNOWN_SUPPORT_NORMAL_ANGULAR_ARM";
                return Invalidate(error, report);
            }
            _arm = arm;
            ResetTrace(report);
            return true;
        }

        internal void ClearCurrentState(FootAnkleFoundationReport report)
        {
            _normal = _perpW = _torque = Vector3.zero;
            _eligible = _applied = _clamped = false;
            CopyToReport(report, false);
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _reason = null;
            _eligibleSamples = _appliedSamples = _clampedSamples = 0;
            _maxTorque = _maxNormalTorque = _maxPower = _workJoules = 0f;
            _fixedSequence = 0;
            _dt = 0f;
            ClearCurrentState(report);
            CopyToReport(report);
        }

        internal bool Step(bool traceEnabled, int fixedSequence, float dt, bool exactAuthorizedStance,
            bool selectedCcdExperiment, bool zeroFriction, bool noJointDynamic, Rigidbody footBody,
            Vector3 supportNormalWorld, FootAnkleFoundationReport report)
        {
            ClearCurrentState(report);
            _fixedSequence = fixedSequence;
            _dt = dt;
            if (!_finite) return false;
            _eligible = exactAuthorizedStance && selectedCcdExperiment && zeroFriction && noJointDynamic &&
                        footBody != null && !footBody.isKinematic;
            if (!_eligible)
            {
                CopyToReport(report, false);
                return true;
            }
            try
            {
                Vector3 w = footBody.angularVelocity;
                Vector3 inertia = footBody.inertiaTensor;
                if (!IsFinite(dt) || dt <= 0f || !IsFinite(w) || !IsFinite(inertia) ||
                    inertia.x <= 0f || inertia.y <= 0f || inertia.z <= 0f ||
                    !IsFinite(supportNormalWorld) || !IsFinite(supportNormalWorld.sqrMagnitude) || supportNormalWorld.sqrMagnitude < 0.00000001f)
                    return Invalidate("SUPPORT_NORMAL_ANGULAR_INPUT_INVALID", report);

                _normal = supportNormalWorld.normalized;
                _perpW = w - _normal * Vector3.Dot(w, _normal);
                if (traceEnabled) _eligibleSamples++;
                if (!Enabled)
                {
                    CopyToReport(report, false);
                    return true;
                }

                // Time-step conditioned gain: dt*k/Imin < 1, independent of principal-axis orientation.
                // Torque direction stays exactly opposite roll/pitch W; clamping only reduces magnitude.
                float minInertia = Mathf.Min(inertia.x, Mathf.Min(inertia.y, inertia.z));
                float gain = minInertia * (1f - Mathf.Exp(-DecayRatePerSecond * dt)) / dt;
                Vector3 rawTorque = -gain * _perpW;
                float rawMagnitude = rawTorque.magnitude;
                if (!IsFinite(gain) || !IsFinite(rawTorque) || !IsFinite(rawMagnitude))
                    return Invalidate("SUPPORT_NORMAL_ANGULAR_RAW_COMMAND_INVALID", report);
                _clamped = rawMagnitude > MaxTorqueNm;
                _torque = _clamped ? rawTorque * (MaxTorqueNm / rawMagnitude) : rawTorque;
                float power = Vector3.Dot(_torque, w);
                float normalTorque = Mathf.Abs(Vector3.Dot(_torque, _normal));
                if (!IsFinite(_torque) || !IsFinite(power) || !IsFinite(normalTorque) ||
                    power > 0.00001f || normalTorque > 0.00001f || _torque.magnitude > MaxTorqueNm + 0.00001f)
                    return Invalidate("SUPPORT_NORMAL_ANGULAR_COMMAND_CONTRACT_FAILED", report);

                if (_torque.sqrMagnitude > 0f)
                {
                    footBody.AddTorque(_torque, ForceMode.Force);
                    _applied = true;
                }
                if (traceEnabled)
                {
                    if (_applied) _appliedSamples++;
                    if (_clamped) _clampedSamples++;
                    _maxTorque = Mathf.Max(_maxTorque, _torque.magnitude);
                    _maxNormalTorque = Mathf.Max(_maxNormalTorque, normalTorque);
                    _maxPower = Mathf.Max(_maxPower, power);
                    _workJoules += power * dt; // commanded pre-step work proxy, not measured solver energy.
                }
                CopyToReport(report, false);
                return true;
            }
            catch (Exception e)
            {
                return Invalidate("SUPPORT_NORMAL_ANGULAR_EXCEPTION:" + e.GetType().Name, report);
            }
        }

        internal void CopyToReport(FootAnkleFoundationReport report, bool formatSummary = true)
        {
            if (report == null) return;
            report.StanceSupportAngularArm = _arm;
            report.StanceSupportAngularEnabled = Enabled;
            report.StanceSupportAngularFinite = _finite;
            report.StanceSupportAngularInvalidReason = _reason;
            report.StanceSupportAngularEligibleSamples = _eligibleSamples;
            report.StanceSupportAngularAppliedSamples = _appliedSamples;
            report.StanceSupportAngularClampedSamples = _clampedSamples;
            report.StanceSupportAngularCurrentTorqueWorld = _torque;
            report.StanceSupportAngularMaxNormalTorqueNm = _maxNormalTorque;
            report.StanceSupportAngularMaxPowerWatts = _maxPower;
            if (!formatSummary) return;
            report.StanceSupportAngularSummary = "mode=" + Mode + "/arm=" + _arm + "/finite=" + _finite +
                "/ratePerSec=" + F(DecayRatePerSecond) + "/maxTorqueNm=" + F(MaxTorqueNm) +
                "/fixed=" + _fixedSequence + "/eligible-applied-clamped=" + _eligible + "-" + _applied + "-" + _clamped +
                "/normal=" + V(_normal) + "/perpW=" + V(_perpW) + "/torque=" + V(_torque) + "/J=" + V(_torque * _dt) +
                "/samples(eligible-applied-clamped)=" + _eligibleSamples + "-" + _appliedSamples + "-" + _clampedSamples +
                "/maxTorque-normalTorque-power=" + F(_maxTorque) + "-" + F(_maxNormalTorque) + "-" + F(_maxPower) +
                "/commandedWorkProxyJ=" + F(_workJoules) + "/reason=" + (_reason ?? "<none>");
        }

        private bool Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            if (_reason == null) _reason = reason;
            ClearCurrentState(report);
            CopyToReport(report);
            return false;
        }

        private static bool IsFinite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool IsFinite(Vector3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
        private static string F(float v) => v.ToString("G9", CultureInfo.InvariantCulture);
        private static string V(Vector3 v) => "(" + F(v.x) + "," + F(v.y) + "," + F(v.z) + ")";
    }
}
