using System;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// One owner captures/restores both creation-time counts. Retired V04270 velocity B=8 stays absent.
    /// V04310 isolates only Foot positional/contact iterations; velocity always remains the captured original.
    /// Preserve run failures and exposure evidence during quiesce; explicit new-run setup resets trace.
    /// </summary>
    internal sealed class FootSolverIterationBaselineContract
    {
        internal const string Mode = "CAPTURED_FOOT_VELOCITY_BASELINE_AND_POSITION_OWNER_V04310";
        internal const string ArmA = "A_V04270_CAPTURED_FOOT_VELOCITY_SOLVER_BASELINE";
        internal const string PositionMode = "FOOT_POSITION_SOLVER_ITERATION_ISOLATION_V04310";
        internal const string PositionArmA = "A_V04310_CAPTURED_FOOT_POSITION_SOLVER";
        internal const string PositionArmB = "B_V04310_FOOT_POSITION_SOLVER_12";
        internal const int BPositionIterations = 12;
        internal const int MinimumEligibleSolves = 50;

        private bool _initialized;
        private bool _finite = true;
        private string _invalidReason;
        private bool _quiesced = true;
        private bool _readbackValid;
        private int _baselineVelocity, _baselinePosition;
        private int _actualVelocity, _actualPosition;
        private int _eligibleSolves, _readbackFailures;
        private string _positionArm = PositionArmA;
        private int _positionInterventionSolves;
        private int _lastExposureFixed = -1;

        internal string Arm => ArmA; // historical A identifier retained for report compatibility
        internal bool Finite => _initialized && _finite;
        internal string PositionArm => _positionArm;
        internal bool PositionEnabled => _positionArm == PositionArmB && !_quiesced;
        private int SelectedPosition => _positionArm == PositionArmB ? BPositionIterations : _baselinePosition;
        private int ExpectedPosition => _quiesced ? _baselinePosition : SelectedPosition;
        internal bool BaselineRestored => _initialized && _readbackValid &&
            _actualVelocity == _baselineVelocity && _actualPosition == _baselinePosition;

        internal bool Initialize(Rigidbody body, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (_initialized)
            {
                error = "V04270_BASELINE_ALREADY_CAPTURED";
                return Invalidate(error, report);
            }
            try
            {
                if (body == null) throw new InvalidOperationException("FOOT_BODY_MISSING");
                int velocity = body.solverVelocityIterations;
                int position = body.solverIterations;
                if (velocity <= 0 || position <= 0) throw new InvalidOperationException("NONPOSITIVE_SOLVER_COUNT");
                _baselineVelocity = _actualVelocity = velocity;
                _baselinePosition = _actualPosition = position;
                _initialized = _readbackValid = true;
                CopyToReport(report);
                return true;
            }
            catch (Exception e)
            {
                error = "V04270_BASELINE_CAPTURE_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                return Invalidate(error, report);
            }
        }

        internal bool SelectCapturedBaseline(Rigidbody body, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!_initialized || body == null)
            {
                error = !_initialized ? "V04270_BASELINE_UNKNOWN" : "V04270_BODY_MISSING";
                return Invalidate(error, report);
            }
            _readbackValid = false;
            try
            {
                if (!RestoreCapturedCounts(body, report, out error)) return false;
                _positionArm = PositionArmA;
                _quiesced = true;
                _finite = true;
                _invalidReason = null;
                ResetTrace(report);
                CopyToReport(report);
                return true;
            }
            catch (Exception e)
            {
                error = "V04270_BASELINE_SET_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                return Invalidate(error, report);
            }
        }

        internal bool SelectPositionArm(string arm, Rigidbody body, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            if (!_initialized || body == null || (arm != PositionArmA && arm != PositionArmB))
            {
                error = "V04310_BASELINE_BODY_OR_ARM_INVALID";
                return Invalidate(error, report);
            }
            // Never lower an unexpectedly high captured original or call a no-change selection an intervention.
            if (arm == PositionArmB && _baselinePosition >= BPositionIterations)
            {
                error = "V04310_B_NOT_ABOVE_CAPTURED_POSITION:" + _baselinePosition;
                return Invalidate(error, report);
            }
            _positionArm = arm;
            _quiesced = false;
            _readbackValid = false;
            try
            {
                body.solverIterations = SelectedPosition;
                if (!ValidateReadback(body, report))
                {
                    error = _invalidReason;
                    return false;
                }
                CopyToReport(report);
                return true;
            }
            catch (Exception e)
            {
                error = "V04310_POSITION_SET_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                return Invalidate(error, report);
            }
        }

        internal bool ValidateReadback(Rigidbody body, FootAnkleFoundationReport report)
        {
            _readbackValid = false;
            if (!_initialized || body == null) return Invalidate("V04270_BASELINE_OR_BODY_MISSING", report);
            try
            {
                _actualVelocity = body.solverVelocityIterations;
                _actualPosition = body.solverIterations;
                int expected = _baselineVelocity;
                if (_actualVelocity != expected || _actualPosition != ExpectedPosition)
                    return Invalidate("V04310_RUNTIME_READBACK_MISMATCH:expectedVelocity=" + expected + ":expectedPosition=" + ExpectedPosition + ":velocity=" + _actualVelocity + ":position=" + _actualPosition, report);
                _readbackValid = true;
                CopyToReport(report, false);
                return _finite; // Successful later readback does not heal an invalid run.
            }
            catch (Exception e)
            {
                return Invalidate("V04270_READBACK_EXCEPTION:" + e.GetType().Name + ":" + e.Message, report);
            }
        }

        internal bool QuiescePreserveTrace(Rigidbody body, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            _quiesced = true;
            _readbackValid = false;
            if (!_initialized || body == null)
            {
                error = "V04270_RESTORE_BASELINE_OR_BODY_MISSING";
                return Invalidate(error, report);
            }
            try
            {
                if (!RestoreCapturedCounts(body, report, out error)) return false;
                CopyToReport(report);
                return true; // Restoration success is independent of run validity.
            }
            catch (Exception e)
            {
                error = "V04270_RESTORE_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                return Invalidate(error, report);
            }
        }

        private bool RestoreCapturedCounts(Rigidbody body, FootAnkleFoundationReport report, out string error)
        {
            error = null;
            // Independent restore attempts: a velocity setter exception must not leave positional B applied.
            try { body.solverVelocityIterations = _baselineVelocity; }
            catch (Exception e) { error = "VELOCITY_RESTORE:" + e.GetType().Name; }
            try { body.solverIterations = _baselinePosition; }
            catch (Exception e) { error = (error == null ? "" : error + ";") + "POSITION_RESTORE:" + e.GetType().Name; }
            try
            {
                _actualVelocity = body.solverVelocityIterations;
                _actualPosition = body.solverIterations;
                _readbackValid = _actualVelocity == _baselineVelocity && _actualPosition == _baselinePosition;
            }
            catch (Exception e) { error = (error == null ? "" : error + ";") + "RESTORE_READBACK:" + e.GetType().Name; }
            if (!_readbackValid || error != null)
            {
                error = "V04310_RESTORE_FAILED:" + (error ?? "COUNT_MISMATCH") + ":velocity=" + _actualVelocity + ":position=" + _actualPosition;
                return Invalidate(error, report);
            }
            return true;
        }

        internal bool RecordPreSolveExposure(bool trace, int fixedSequence, bool exactLoadedStance,
            Rigidbody body, FootAnkleFoundationReport report)
        {
            if (!ValidateReadback(body, report)) return false;
            if (!trace || !exactLoadedStance || fixedSequence == _lastExposureFixed) return true;
            _lastExposureFixed = fixedSequence;
            _eligibleSolves++;
            if (PositionEnabled && _actualPosition == BPositionIterations) _positionInterventionSolves++;
            CopyToReport(report, false);
            return true;
        }

        internal void ResetTrace(FootAnkleFoundationReport report)
        {
            _eligibleSolves = _readbackFailures = 0;
            _positionInterventionSolves = 0;
            _lastExposureFixed = -1;
            CopyToReport(report, false);
        }

        internal void CopyToReport(FootAnkleFoundationReport report, bool format = true)
        {
            if (report == null) return;
            report.FootVelocitySolverMode = Mode;
            report.FootVelocitySolverArm = ArmA;
            report.FootVelocitySolverInitialized = _initialized;
            report.FootVelocitySolverFinite = Finite;
            report.FootVelocitySolverInvalidReason = _invalidReason;
            report.FootVelocitySolverBaselineVelocity = _baselineVelocity;
            report.FootVelocitySolverBaselinePosition = _baselinePosition;
            report.FootVelocitySolverSelectedVelocity = _baselineVelocity;
            report.FootVelocitySolverActualVelocity = _actualVelocity;
            report.FootVelocitySolverActualPosition = _actualPosition;
            report.FootVelocitySolverReadbackValid = _readbackValid;
            report.FootVelocitySolverQuiesced = _quiesced;
            report.FootVelocitySolverBaselineRestored = BaselineRestored;
            report.FootVelocitySolverEligibleSolves = _eligibleSolves;
            report.FootVelocitySolverReadbackFailures = _readbackFailures;
            report.FootPositionSolverMode = PositionMode;
            report.FootPositionSolverArm = _positionArm;
            report.FootPositionSolverSelectedPosition = SelectedPosition;
            report.FootPositionSolverEnabled = PositionEnabled;
            report.FootPositionSolverInterventionSolves = _positionInterventionSolves;
            if (format) report.FootVelocitySolverSummary = "mode=" + Mode + "/arm=" + ArmA +
                "/initialized=" + _initialized + "/finite=" + Finite + "/baseline(velocity-position)=" + _baselineVelocity + "-" + _baselinePosition +
                "/selectedVelocity=" + _baselineVelocity + "/actual(velocity-position)=" + _actualVelocity + "-" + _actualPosition +
                "/readbackValid=" + _readbackValid + "/quiesced=" + _quiesced + "/baselineRestored=" + BaselineRestored +
                "/eligibleSolves=" + _eligibleSolves + "/readbackFailures=" + _readbackFailures +
                "/reason=" + (_invalidReason ?? "<none>");
            if (format) report.FootPositionSolverSummary = "mode=" + PositionMode + "/arm=" + _positionArm +
                "/finite=" + Finite + "/quiesced=" + _quiesced + "/enabled=" + PositionEnabled +
                "/baseline(velocity-position)=" + _baselineVelocity + "-" + _baselinePosition +
                "/selected(velocity-position)=" + _baselineVelocity + "-" + SelectedPosition +
                "/actual(velocity-position)=" + _actualVelocity + "-" + _actualPosition +
                "/readbackValid=" + _readbackValid + "/baselineRestored=" + BaselineRestored +
                "/loadedInterventionSolves=" + _positionInterventionSolves + "/readbackFailures=" + _readbackFailures +
                "/reason=" + (_invalidReason ?? "<none>");
        }

        private bool Invalidate(string reason, FootAnkleFoundationReport report)
        {
            _finite = false;
            _readbackFailures++;
            if (string.IsNullOrEmpty(_invalidReason)) _invalidReason = reason;
            CopyToReport(report);
            return false;
        }
    }
}
