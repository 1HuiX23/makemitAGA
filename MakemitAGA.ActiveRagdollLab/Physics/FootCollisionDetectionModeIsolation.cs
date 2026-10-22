using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// V04390 freezes Foot ContinuousSpeculative in both contactOffset arms.
    /// The historical Discrete V04240 baseline is only the idle/cleanup state.
    /// The completed V04380 ContinuousDynamic selector is retired; its evidence remains in the version record.
    /// Mode selection alone is not observed CCD exposure or a mechanism verdict.
    /// </summary>
    internal sealed class FootCollisionDetectionModeIsolation
    {
        internal const string Mode = "FROZEN_FOOT_SPECULATIVE_CCD_BACKGROUND_V04390";
        internal const string IdleArm = "IDLE_V04390_DISCRETE_FOOT_COLLISION";
        internal const string ActiveArm = "FROZEN_V04390_CONTINUOUS_SPECULATIVE_FOOT_COLLISION";

        private string _arm = IdleArm;
        private bool _finite = true;
        private string _invalidReason;

        internal static bool IsActiveArm(string arm) =>
            string.Equals(arm, ActiveArm, StringComparison.Ordinal);

        internal static bool TryGetMode(string arm, out CollisionDetectionMode mode)
        {
            mode = CollisionDetectionMode.Discrete;
            if (string.Equals(arm, IdleArm, StringComparison.Ordinal)) return true;
            if (string.Equals(arm, ActiveArm, StringComparison.Ordinal))
            { mode = CollisionDetectionMode.ContinuousSpeculative; return true; }
            return false;
        }

        internal string Arm => _arm;
        internal bool EnabledSpeculative => IsActiveArm(_arm);
        internal bool Finite => _finite;
        internal string InvalidReason => _invalidReason;

        internal bool SetArm(string arm, Rigidbody footBody, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (footBody == null)
            {
                error = "FOOT_COLLISION_MODE_BODY_MISSING";
                return Invalidate(error, footBody, report);
            }
            if (!TryGetMode(arm, out CollisionDetectionMode requested))
            {
                error = "UNKNOWN_FOOT_COLLISION_MODE_ARM:" + (arm ?? "<null>");
                return Invalidate(error, footBody, report);
            }

            try
            {
                footBody.collisionDetectionMode = requested;
                CollisionDetectionMode readback = footBody.collisionDetectionMode;
                if (readback != requested)
                {
                    error = "FOOT_COLLISION_MODE_READBACK_MISMATCH:requested=" + requested + ":actual=" + readback;
                    return Invalidate(error, footBody, report);
                }

                _arm = arm;
                _finite = true;
                _invalidReason = null;
                CopyToReport(footBody, report);
                if (!_finite) error = _invalidReason;
                return _finite;
            }
            catch (Exception e)
            {
                error = "FOOT_COLLISION_MODE_SET_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                return Invalidate(error, footBody, report);
            }
        }

        internal bool ValidateReadback(Rigidbody footBody, FootAnkleFoundationReport report)
        {
            if (footBody == null)
                return Invalidate("FOOT_COLLISION_MODE_BODY_MISSING", footBody, report);

            if (!TryGetMode(_arm, out CollisionDetectionMode expected))
                return Invalidate("UNKNOWN_SELECTED_FOOT_COLLISION_MODE_ARM", footBody, report);
            try
            {
                CollisionDetectionMode actual = footBody.collisionDetectionMode;
                if (actual != expected)
                    return Invalidate("FOOT_COLLISION_MODE_RUNTIME_READBACK_MISMATCH:expected=" + expected + ":actual=" + actual, footBody, report);
                CopyToReport(footBody, report);
                return _finite;
            }
            catch (Exception e)
            {
                return Invalidate("FOOT_COLLISION_MODE_READBACK_EXCEPTION:" + e.GetType().Name + ":" + e.Message, footBody, report);
            }
        }

        internal void CopyToReport(Rigidbody footBody, FootAnkleFoundationReport report)
        {
            if (report == null) return;
            // Invalidate also formats the report: a failing getter must not escape that error path.
            string readback = "NA_BODY_MISSING";
            try
            {
                if (footBody != null)
                {
                    CollisionDetectionMode actual = footBody.collisionDetectionMode;
                    readback = actual.ToString();
                    if (!TryGetMode(_arm, out CollisionDetectionMode expected) || actual != expected)
                    {
                        _finite = false;
                        if (string.IsNullOrEmpty(_invalidReason))
                            _invalidReason = "FOOT_COLLISION_MODE_REPORT_READBACK_MISMATCH:actual=" + actual;
                    }
                }
            }
            catch (Exception e)
            {
                readback = "NA_READBACK_EXCEPTION:" + e.GetType().Name;
                _finite = false;
                if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = readback;
            }
            report.FootCollisionDetectionModeExperiment = Mode;
            report.FootCollisionDetectionModeArm = _arm;
            report.FootCollisionDetectionModeFinite = _finite;
            report.FootCollisionDetectionModeInvalidReason = _invalidReason;
            report.FootCollisionDetectionModeReadback = readback;
            report.FootCollisionDetectionModeSummary =
                "mode=" + Mode +
                "/arm=" + _arm +
                "/finite=" + _finite +
                "/continuousSpeculative=" + string.Equals(_arm, ActiveArm, StringComparison.Ordinal) +
                "/readback=" + readback +
                "/reason=" + (_invalidReason ?? "<none>");
        }

        private bool Invalidate(string reason, Rigidbody footBody, FootAnkleFoundationReport report)
        {
            _finite = false;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            CopyToReport(footBody, report);
            return false;
        }
    }
}
