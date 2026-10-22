using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04300 changes one tether term only in the existing exact authorized loaded-STANCE environment.
    /// Foot omega x ankle-lever is removed from planar damping; its normal part and the entire Lower
    /// ankle-point velocity remain. This class owns no Rigidbody write, contact authority or new cast.
    /// Raw normal damping is preserved; the existing uniform vector clamp can change its applied scale.
    /// Selection and accumulated evidence survive quiesce; only explicit new trace resets run evidence.
    /// </summary>
    internal sealed class FootStancePlanarRotationalDampingIsolation
    {
        internal const string Mode = "STANCE_PLANAR_FOOT_ROTATION_DAMPING_ISOLATION_V04300";
        internal const string ArmA = "A_V04290_FULL_POINT_DAMPING_BASELINE";
        internal const string ArmB = "B_V04300_REMOVE_FOOT_ROT_PLANAR_DAMPING";
        internal const int MinimumLoadedSolves = 50;
        private bool _enabled, _quiesced, _finite = true, _active, _written;
        private string _invalidReason;
        private int _fixed, _lastLoaded = -1, _evaluated, _eligible, _writes, _nonzeroWrites, _loaded;
        private Vector3 _footRotPlanar;
        private float _normalDeltaMax, _removedDampingMax;

        internal bool Enabled => _enabled;
        internal string Arm => _enabled ? ArmB : ArmA;
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;
        internal bool CurrentActive => _active;

        internal void SetEnabled(bool enabled, FootAnkleFoundationReport report)
        {
            _enabled = enabled;
            _quiesced = !enabled;
            ClearCurrent(report);
        }

        internal bool Evaluate(bool trace, int fixedSequence, bool exactLoadedStance,
            Vector3 originalVelocityError, Vector3 footRotPointVelocity, bool normalValid, Vector3 normal,
            FootAnkleFoundationReport report, out Vector3 effectiveVelocityError)
        {
            _fixed = fixedSequence;
            _written = false;
            _active = _enabled && !_quiesced && exactLoadedStance && normalValid;
            _footRotPlanar = Vector3.zero;
            effectiveVelocityError = originalVelocityError;
            if (trace) _evaluated++;
            if (_active)
            {
                if (!V(originalVelocityError) || !V(footRotPointVelocity) || !V(normal) || normal.sqrMagnitude <= 0.00000001f)
                {
                    _finite = false;
                    if (_invalidReason == null) _invalidReason = "NON_FINITE_V04300_DAMPING_INPUT";
                    CopyToReport(report);
                    return false;
                }
                Vector3 n = normal.normalized;
                _footRotPlanar = footRotPointVelocity - n * Vector3.Dot(footRotPointVelocity, n);
                effectiveVelocityError = originalVelocityError + _footRotPlanar;
                if (!V(effectiveVelocityError))
                {
                    _finite = false;
                    if (_invalidReason == null) _invalidReason = "NON_FINITE_V04300_EFFECTIVE_DAMPING";
                    CopyToReport(report);
                    return false;
                }
                if (trace)
                {
                    _eligible++;
                    _normalDeltaMax = Mathf.Max(_normalDeltaMax,
                        Mathf.Abs(Vector3.Dot((effectiveVelocityError - originalVelocityError) * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper, n)));
                    _removedDampingMax = Mathf.Max(_removedDampingMax,
                        _footRotPlanar.magnitude * FootAnkleConstraintTopologyIsolation.TetherVelocityDamper);
                }
            }
            CopyToReport(report);
            return true;
        }

        internal void RecordWritten(bool trace, int fixedSequence, FootAnkleFoundationReport report)
        {
            if (_fixed == fixedSequence && _active && !_written)
            {
                _written = true;
                if (trace)
                {
                    _writes++;
                    if (_footRotPlanar.sqrMagnitude > 0.0000000001f) _nonzeroWrites++;
                }
            }
            CopyToReport(report);
        }

        internal void RecordLoadedSolve(bool trace, int fixedSequence, bool actualLoaded, FootAnkleFoundationReport report)
        {
            if (trace && actualLoaded && _active && _written && _fixed == fixedSequence && _lastLoaded != fixedSequence)
            {
                _loaded++;
                _lastLoaded = fixedSequence;
            }
            CopyToReport(report);
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _finite = true;
            _invalidReason = null;
            _evaluated = _eligible = _writes = _nonzeroWrites = _loaded = 0;
            _lastLoaded = -1;
            _normalDeltaMax = _removedDampingMax = 0f;
            _quiesced = !_enabled;
            ClearCurrent(report);
        }

        internal void QuiescePreserveTrace(FootAnkleFoundationReport report)
        {
            _quiesced = true;
            ClearCurrent(report);
        }

        private void ClearCurrent(FootAnkleFoundationReport report)
        {
            _active = _written = false;
            _footRotPlanar = Vector3.zero;
            CopyToReport(report);
        }

        internal void CopyToReport(FootAnkleFoundationReport report)
        {
            if (report == null) return;
            report.StancePlanarDampingMode = Mode;
            report.StancePlanarDampingArm = Arm;
            report.StancePlanarDampingEnabled = _enabled;
            report.StancePlanarDampingQuiesced = _quiesced;
            report.StancePlanarDampingFinite = _finite;
            report.StancePlanarDampingInvalidReason = _invalidReason;
            report.StancePlanarDampingCurrentActive = _active;
            report.StancePlanarDampingLoadedSolves = _loaded;
            report.StancePlanarDampingNonzeroWrites = _nonzeroWrites;
            // Summary is formatted on report requests, not each hot-path Evaluate/write/capture.
        }

        internal string GetSummary(FootAnkleFoundationReport report)
        {
            CopyToReport(report);
            report.StancePlanarDampingSummary = "arm=" + Arm + "/enabled=" + _enabled + "/quiesced=" + _quiesced +
                "/finite=" + _finite + "/reason=" + (_invalidReason ?? "<none>") +
                " | evaluated-eligible-written-nonzero-loaded=" + _evaluated + "-" + _eligible + "-" + _writes + "-" + _nonzeroWrites + "-" + _loaded +
                " | maxRawNormalDampingDelta=" + _normalDeltaMax.ToString("G9", CultureInfo.InvariantCulture) +
                "/maxRemovedFootRotPlanarD=" + _removedDampingMax.ToString("G9", CultureInfo.InvariantCulture);
            return report.StancePlanarDampingSummary;
        }

        private static bool F(float x) => !float.IsNaN(x) && !float.IsInfinity(x);
        private static bool V(Vector3 v) => F(v.x) && F(v.y) && F(v.z);
    }
}
