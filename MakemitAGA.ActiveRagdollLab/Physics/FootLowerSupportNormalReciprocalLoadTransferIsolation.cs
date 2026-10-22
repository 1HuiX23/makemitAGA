using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// ARL v0.4.20 support-normal reciprocal load-transfer causal A/B.
    ///
    /// Both arms freeze the v0.4.19 B environment: Foot native gravity ON in exact authorized STANCE,
    /// Physical Lower native gravity ON for the same solve, zero-friction V0470 contact, frozen V04140
    /// planar Lower authority and the unchanged v0.3.63/v0.3.73 Foot translation tether.
    ///
    /// A keeps the historical child-only Foot tether topology. B takes only the signed support-normal
    /// component of the force that the existing Foot tether has ALREADY applied at the Foot COM and applies
    /// the exact equal-and-opposite force at the Physical Lower COM. No new PD law, gain, clamp, horizontal
    /// force, torque, AddForceAtPosition or hard-gate relaxation is introduced.
    ///
    /// Because the Foot tether actuator uses ForceMode.Acceleration, equal/opposite reciprocity must be
    /// constructed in force units: F_foot,n = m_foot * dot(a_applied,n) * n; F_lower = -F_foot,n.
    /// B therefore uses ForceMode.Force on Lower. Its magnitude can never exceed the support-normal
    /// component of the already-issued Foot tether force.
    /// </summary>
    internal sealed class FootLowerSupportNormalReciprocalLoadTransferIsolation
    {
        internal const string Mode = "STANCE_FOOT_LOWER_SUPPORT_NORMAL_RECIPROCAL_LOAD_TRANSFER_CAUSAL_AB_V04200";
        internal const string ArmA = "A_V04200_V04190_B_LOWER_GRAVITY_CHILD_ONLY_FOOT_TETHER";
        internal const string ArmB = "B_V04200_SUPPORT_NORMAL_EQUAL_OPPOSITE_RECIPROCAL";
        internal const int MinEligibleSamples = 30;
        internal const int MinAppliedSamples = 30;

        private const float Epsilon = 0.0000001f;
        private const float BoundToleranceNewtons = 0.01f;

        private string _arm = ArmA;
        private bool _finite = true;
        private string _invalidReason;

        private int _traceSamples;
        private int _eligibleSamples;
        private int _appliedSamples;
        private int _nonStanceAppliedSamples;
        private int _zeroReactionSamples;
        private int _upwardLowerReactionSamples;
        private int _downwardLowerReactionSamples;
        private int _firstAppliedFixed;
        private int _lastAppliedFixed;

        private float _footTetherSupportNormalAccelerationSum;
        private float _footTetherSupportNormalAccelerationMin;
        private float _footTetherSupportNormalAccelerationMax;
        private bool _footTetherSupportNormalAccelerationHasSample;
        private float _reciprocalForceAbsSum;
        private float _reciprocalForceAbsMax;
        private float _lowerReactionSupportNormalAccelerationSum;
        private float _lowerReactionSupportNormalAccelerationMin;
        private float _lowerReactionSupportNormalAccelerationMax;
        private bool _lowerReactionSupportNormalAccelerationHasSample;

        private bool _currentApplied;
        private int _currentFixed;
        private float _currentFootMass;
        private float _currentLowerMass;
        private float _currentFootTetherSupportNormalAcceleration;
        private float _currentFootTetherSupportNormalForce;
        private float _currentLowerReactionSupportNormalAcceleration;
        private Vector3 _currentReactionForceWorld;

        internal string Arm => _arm;
        internal bool Enabled => string.Equals(_arm, ArmB, StringComparison.Ordinal);
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;
        internal bool CurrentApplied => _currentApplied;

        internal bool SetArm(string arm, SingleLegChainReport report, out string error)
        {
            error = null;
            if (!string.Equals(arm, ArmA, StringComparison.Ordinal) && !string.Equals(arm, ArmB, StringComparison.Ordinal))
            {
                error = "UNKNOWN_SUPPORT_NORMAL_RECIPROCAL_ARM:" + (arm ?? "<null>");
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
            _zeroReactionSamples = 0;
            _upwardLowerReactionSamples = 0;
            _downwardLowerReactionSamples = 0;
            _firstAppliedFixed = 0;
            _lastAppliedFixed = 0;
            _footTetherSupportNormalAccelerationSum = 0f;
            _footTetherSupportNormalAccelerationMin = 0f;
            _footTetherSupportNormalAccelerationMax = 0f;
            _footTetherSupportNormalAccelerationHasSample = false;
            _reciprocalForceAbsSum = 0f;
            _reciprocalForceAbsMax = 0f;
            _lowerReactionSupportNormalAccelerationSum = 0f;
            _lowerReactionSupportNormalAccelerationMin = 0f;
            _lowerReactionSupportNormalAccelerationMax = 0f;
            _lowerReactionSupportNormalAccelerationHasSample = false;
            ClearCurrent();
            CopyToReport(report);
        }

        internal bool Step(
            bool traceEnabled,
            int fixedSequence,
            bool exactAuthorizedStance,
            bool zeroFriction,
            Rigidbody lowerBody,
            FootAnkleFoundationReport footReport,
            Vector3 supportNormalWorld,
            SingleLegChainReport report)
        {
            if (traceEnabled) _traceSamples++;
            ClearCurrent();

            if (lowerBody == null || footReport == null || !footReport.Built || !footReport.Finite)
                return Invalidate("RECIPROCAL_LOAD_TRANSFER_FOUNDATION_MISSING_OR_INVALID", report);
            if (lowerBody.isKinematic || !IsFinite(lowerBody.position) || !IsFinite(lowerBody.rotation) ||
                !IsFinite(lowerBody.velocity) || !IsFinite(lowerBody.angularVelocity) ||
                !IsFinite(lowerBody.mass) || lowerBody.mass <= Epsilon)
                return Invalidate("RECIPROCAL_LOAD_TRANSFER_LOWER_BODY_STATE_INVALID", report);

            float footMass = footReport.Mass;
            Vector3 footAppliedAcceleration = footReport.AnkleConstraintCurrentAppliedAccelerationWorld;
            _currentFootMass = footMass;
            _currentLowerMass = lowerBody.mass;

            if (!IsFinite(footMass) || footMass <= Epsilon || !IsFinite(footAppliedAcceleration))
                return Invalidate("RECIPROCAL_LOAD_TRANSFER_FOOT_TETHER_STATE_INVALID", report);

            if (!exactAuthorizedStance || !zeroFriction)
            {
                if (_currentApplied) _nonStanceAppliedSamples++;
                CopyToReport(report);
                return true;
            }

            if (!footReport.AnkleConstraintCurrentDynamic)
                return Invalidate("RECIPROCAL_LOAD_TRANSFER_FOOT_TETHER_NOT_DYNAMIC_IN_AUTHORIZED_STANCE", report);
            if (!IsFinite(supportNormalWorld) || supportNormalWorld.sqrMagnitude <= Epsilon)
                return Invalidate("RECIPROCAL_LOAD_TRANSFER_SUPPORT_NORMAL_INVALID", report);

            Vector3 n = supportNormalWorld.normalized;
            float footSupportA = Vector3.Dot(footAppliedAcceleration, n);
            float footSupportForceSigned = footMass * footSupportA;
            Vector3 reactionForce = -footSupportForceSigned * n;
            float reactionForceAbs = reactionForce.magnitude;
            float lowerReactionSupportA = Vector3.Dot(reactionForce / lowerBody.mass, n);
            float sourceForceBound = footMass * footAppliedAcceleration.magnitude;

            if (!IsFinite(footSupportA) || !IsFinite(footSupportForceSigned) || !IsFinite(reactionForce) ||
                !IsFinite(reactionForceAbs) || !IsFinite(lowerReactionSupportA) || !IsFinite(sourceForceBound))
                return Invalidate("RECIPROCAL_LOAD_TRANSFER_ATTRIBUTION_NONFINITE", report);
            if (reactionForceAbs > sourceForceBound + BoundToleranceNewtons)
                return Invalidate("RECIPROCAL_LOAD_TRANSFER_FORCE_BOUND_VIOLATION", report);

            _eligibleSamples++;
            _currentFootTetherSupportNormalAcceleration = footSupportA;
            _currentFootTetherSupportNormalForce = footSupportForceSigned;
            _currentLowerReactionSupportNormalAcceleration = lowerReactionSupportA;
            _currentReactionForceWorld = reactionForce;

            _footTetherSupportNormalAccelerationSum += footSupportA;
            if (!_footTetherSupportNormalAccelerationHasSample)
            {
                _footTetherSupportNormalAccelerationMin = footSupportA;
                _footTetherSupportNormalAccelerationMax = footSupportA;
                _footTetherSupportNormalAccelerationHasSample = true;
            }
            else
            {
                _footTetherSupportNormalAccelerationMin = Mathf.Min(_footTetherSupportNormalAccelerationMin, footSupportA);
                _footTetherSupportNormalAccelerationMax = Mathf.Max(_footTetherSupportNormalAccelerationMax, footSupportA);
            }

            _reciprocalForceAbsSum += reactionForceAbs;
            _reciprocalForceAbsMax = Mathf.Max(_reciprocalForceAbsMax, reactionForceAbs);
            _lowerReactionSupportNormalAccelerationSum += lowerReactionSupportA;
            if (!_lowerReactionSupportNormalAccelerationHasSample)
            {
                _lowerReactionSupportNormalAccelerationMin = lowerReactionSupportA;
                _lowerReactionSupportNormalAccelerationMax = lowerReactionSupportA;
                _lowerReactionSupportNormalAccelerationHasSample = true;
            }
            else
            {
                _lowerReactionSupportNormalAccelerationMin = Mathf.Min(_lowerReactionSupportNormalAccelerationMin, lowerReactionSupportA);
                _lowerReactionSupportNormalAccelerationMax = Mathf.Max(_lowerReactionSupportNormalAccelerationMax, lowerReactionSupportA);
            }

            if (reactionForceAbs <= Epsilon)
                _zeroReactionSamples++;
            else if (lowerReactionSupportA > 0f)
                _upwardLowerReactionSamples++;
            else if (lowerReactionSupportA < 0f)
                _downwardLowerReactionSamples++;

            if (Enabled)
            {
                try
                {
                    // Exact equal-and-opposite support-normal force, applied at Lower COM: no added moment.
                    lowerBody.AddForce(reactionForce, ForceMode.Force);
                }
                catch (Exception e)
                {
                    return Invalidate("RECIPROCAL_LOAD_TRANSFER_ADDFORCE_EXCEPTION:" + e.GetType().Name, report);
                }

                _currentApplied = true;
                _currentFixed = fixedSequence;
                _appliedSamples++;
                if (_firstAppliedFixed == 0) _firstAppliedFixed = fixedSequence;
                _lastAppliedFixed = fixedSequence;
            }

            CopyToReport(report);
            return true;
        }

        internal void CopyToReport(SingleLegChainReport report)
        {
            if (report == null) return;
            float eligibleDenom = Mathf.Max(1, _eligibleSamples);
            report.StanceFootLowerReciprocalMode = Mode;
            report.StanceFootLowerReciprocalArm = _arm;
            report.StanceFootLowerReciprocalFinite = _finite;
            report.StanceFootLowerReciprocalInvalidReason = _invalidReason;
            report.StanceFootLowerReciprocalEnabled = Enabled;
            report.StanceFootLowerReciprocalTraceSamples = _traceSamples;
            report.StanceFootLowerReciprocalEligibleSamples = _eligibleSamples;
            report.StanceFootLowerReciprocalAppliedSamples = _appliedSamples;
            report.StanceFootLowerReciprocalNonStanceAppliedSamples = _nonStanceAppliedSamples;
            report.StanceFootLowerReciprocalZeroReactionSamples = _zeroReactionSamples;
            report.StanceFootLowerReciprocalUpwardLowerReactionSamples = _upwardLowerReactionSamples;
            report.StanceFootLowerReciprocalDownwardLowerReactionSamples = _downwardLowerReactionSamples;
            report.StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMean = _eligibleSamples > 0 ? _footTetherSupportNormalAccelerationSum / eligibleDenom : 0f;
            report.StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMin = _footTetherSupportNormalAccelerationHasSample ? _footTetherSupportNormalAccelerationMin : 0f;
            report.StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMax = _footTetherSupportNormalAccelerationHasSample ? _footTetherSupportNormalAccelerationMax : 0f;
            report.StanceFootLowerReciprocalForceAbsMean = _eligibleSamples > 0 ? _reciprocalForceAbsSum / eligibleDenom : 0f;
            report.StanceFootLowerReciprocalForceAbsMax = _reciprocalForceAbsMax;
            report.StanceFootLowerReciprocalLowerSupportNormalAccelerationMean = _eligibleSamples > 0 ? _lowerReactionSupportNormalAccelerationSum / eligibleDenom : 0f;
            report.StanceFootLowerReciprocalLowerSupportNormalAccelerationMin = _lowerReactionSupportNormalAccelerationHasSample ? _lowerReactionSupportNormalAccelerationMin : 0f;
            report.StanceFootLowerReciprocalLowerSupportNormalAccelerationMax = _lowerReactionSupportNormalAccelerationHasSample ? _lowerReactionSupportNormalAccelerationMax : 0f;
            report.StanceFootLowerReciprocalCurrentApplied = _currentApplied;
            report.StanceFootLowerReciprocalCurrentFixed = _currentFixed;
            report.StanceFootLowerReciprocalCurrentFootMass = _currentFootMass;
            report.StanceFootLowerReciprocalCurrentLowerMass = _currentLowerMass;
            report.StanceFootLowerReciprocalCurrentFootTetherSupportNormalAcceleration = _currentFootTetherSupportNormalAcceleration;
            report.StanceFootLowerReciprocalCurrentFootTetherSupportNormalForce = _currentFootTetherSupportNormalForce;
            report.StanceFootLowerReciprocalCurrentLowerSupportNormalAcceleration = _currentLowerReactionSupportNormalAcceleration;
            report.StanceFootLowerReciprocalCurrentReactionForceWorld = _currentReactionForceWorld;
            report.StanceFootLowerReciprocalFirstAppliedFixed = _firstAppliedFixed;
            report.StanceFootLowerReciprocalLastAppliedFixed = _lastAppliedFixed;
            report.StanceFootLowerReciprocalSummary =
                "mode=" + Mode +
                "/arm=" + _arm +
                "/finite=" + _finite +
                "/enabled=" + Enabled +
                "/n-eligible-applied-nonStance=" + _traceSamples + "-" + _eligibleSamples + "-" + _appliedSamples + "-" + _nonStanceAppliedSamples +
                "/reactionSign(zero-up-down)=" + _zeroReactionSamples + "-" + _upwardLowerReactionSamples + "-" + _downwardLowerReactionSamples +
                "/footTetherSupportNormalA(mean-min-max)=" + F(report.StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMean) + "-" + F(report.StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMin) + "-" + F(report.StanceFootLowerReciprocalFootTetherSupportNormalAccelerationMax) +
                "/reciprocalForceAbsMeanMaxN=" + F(report.StanceFootLowerReciprocalForceAbsMean) + "-" + F(report.StanceFootLowerReciprocalForceAbsMax) +
                "/lowerReactionSupportNormalA(mean-min-max)=" + F(report.StanceFootLowerReciprocalLowerSupportNormalAccelerationMean) + "-" + F(report.StanceFootLowerReciprocalLowerSupportNormalAccelerationMin) + "-" + F(report.StanceFootLowerReciprocalLowerSupportNormalAccelerationMax) +
                "/current(applied-fixed-footMass-lowerMass-footA-footForce-lowerA-force)=" + _currentApplied + "-" + _currentFixed + "-" + F(_currentFootMass) + "-" + F(_currentLowerMass) + "-" + F(_currentFootTetherSupportNormalAcceleration) + "-" + F(_currentFootTetherSupportNormalForce) + "-" + F(_currentLowerReactionSupportNormalAcceleration) + "-" + V(_currentReactionForceWorld) +
                "/firstLastApplied=" + _firstAppliedFixed + "-" + _lastAppliedFixed +
                "/topology=SUPPORT_NORMAL_ONLY_EQUAL_OPPOSITE_FORCE_AT_LOWER_COM" +
                "/reason=" + (_invalidReason ?? "<none>");
        }

        private void ClearCurrent()
        {
            _currentApplied = false;
            _currentFixed = 0;
            _currentFootMass = 0f;
            _currentLowerMass = 0f;
            _currentFootTetherSupportNormalAcceleration = 0f;
            _currentFootTetherSupportNormalForce = 0f;
            _currentLowerReactionSupportNormalAcceleration = 0f;
            _currentReactionForceWorld = Vector3.zero;
        }

        private bool Invalidate(string reason, SingleLegChainReport report)
        {
            _finite = false;
            _invalidReason = reason;
            _currentApplied = false;
            _currentFixed = 0;
            CopyToReport(report);
            return false;
        }

        private static bool IsFinite(float value) => !float.IsNaN(value) && !float.IsInfinity(value);
        private static bool IsFinite(Vector3 value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        private static bool IsFinite(Quaternion value) => IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z) && IsFinite(value.w);
        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";
    }
}
