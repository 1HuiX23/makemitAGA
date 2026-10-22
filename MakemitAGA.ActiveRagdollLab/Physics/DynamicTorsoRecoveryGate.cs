using System.Globalization;
using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Scoped replacement for the legacy virtual-anchor recovery qualification, never a safety bypass.
    // Both run arms observe it; only B consumes it. The old freeze validator remains intact.
    internal sealed class DynamicTorsoRecoveryGate
    {
        internal const string LegacyPolicy = "LEGACY_FREEZE_RECOVERY";
        internal const string DynamicPolicy = "DYNAMIC_TORSO_RECOVERY_V04760";
        internal const float ComBandMeters = 0.03f, PelvisBandDegrees = 2f, NativeAnchorBandMeters = 0.002f;
        // Preserve the actual Chest PD-relative error and the old strict hold threshold.
        internal const float ChestBandDegrees = SourceAwareFreezeValidation.RecoveryRotationStrictDegrees;
        internal const float HoldSeconds = SourceAwareFreezeValidation.RecoveryStableHoldSeconds;
        private PhysicsStepContext _last;
        private bool _pending = true, _holding;
        private float _hold;
        private int _valid, _invalid, _pendingSteps, _legacyOnly, _dynamicOnly;
        internal TorsoRecoveryState Current { get; private set; }

        internal void ResetState()
        {
            _last = new PhysicsStepContext(); _pending = true; _holding = false; _hold = 0f;
            Current = new TorsoRecoveryState { Active = true, Reason = "NOT_SAMPLED" };
        }
        // Trace changes counters only. It must not reopen a gate or restart its physical hold.
        internal void ResetDiagnostics() { _valid = _invalid = _pendingSteps = _legacyOnly = _dynamicOnly = 0; }

        internal void Step(PhysicsStepContext step, bool inputValid, bool sourceTurn,
            float comError, float pelvisError, float chestError, float nativeGap, float legacyGap)
        {
            if (Current.Matches(step)) return; // Same packet cannot earn hold twice.
            bool consecutive = _last.Valid && step.Valid && step.Epoch == _last.Epoch &&
                step.Sequence == _last.Sequence + 1 && step.SourceSequence >= _last.SourceSequence &&
                step.FixedTime >= _last.FixedTime && step.UnscaledTime >= _last.UnscaledTime;
            bool valid = step.Valid && inputValid && Nonnegative(comError) && Nonnegative(pelvisError) &&
                Nonnegative(chestError) && Nonnegative(nativeGap);
            // A sequence/epoch break starts a new qualification; unobserved intervals earn no hold.
            if (!consecutive) { _pending = true; _holding = false; _hold = 0f; }
            string reason;
            if (!valid)
            { _pending = true; _holding = false; _hold = 0f; reason = "INVALID_INPUT"; }
            else if (sourceTurn)
            { _pending = true; _holding = false; _hold = 0f; reason = "SOURCE_TURN"; }
            else if (!_pending) reason = "RECOVERED";
            else if (chestError > ChestBandDegrees)
            { _holding = false; _hold = 0f; reason = "CHEST_RELATIVE_ERROR"; }
            else if (nativeGap > NativeAnchorBandMeters)
            { _holding = false; _hold = 0f; reason = "NATIVE_ANCHOR_GAP"; }
            else if (comError > ComBandMeters || pelvisError > PelvisBandDegrees)
            { _holding = false; _hold = 0f; reason = "GOVERNED_PELVIS_ERROR"; }
            else
            {
                // Accumulate Fixed dt between consecutive observed in-band points, not wall/render time.
                if (_holding && consecutive) _hold += step.Dt;
                else _hold = 0f;
                _holding = true;
                if (_hold + SourceAwareFreezeValidation.RecoveryStableHoldNumericalToleranceSeconds >= HoldSeconds)
                    _pending = false;
                reason = _pending ? "STABLE_HOLD" : "RECOVERED";
            }
            _last = valid ? step : new PhysicsStepContext();
            Current = new TorsoRecoveryState { Step = step, Valid = valid, Active = _pending, Reason = reason,
                HoldSeconds = _hold, ComErrorMeters = comError, PelvisErrorDegrees = pelvisError,
                ChestRelativeErrorDegrees = chestError, NativeAnchorGapMeters = nativeGap, LegacyAnchorGapMeters = legacyGap };
        }

        internal void Record(bool legacyActive)
        {
            if (!Current.Valid) { _invalid++; return; }
            _valid++;
            if (Current.Active) _pendingSteps++;
            if (legacyActive && !Current.Active) _legacyOnly++;
            if (!legacyActive && Current.Active) _dynamicOnly++;
        }
        internal string GetSummary(bool selected) => "TORSO_RECOVERY_V04760 | policy=" +
            (selected ? DynamicPolicy : LegacyPolicy) + " | dynamicRole=" + (selected ? "CONTROL" : "SHADOW") +
            " | valid-invalid-pending-legacyOnly-dynamicOnly=" + _valid + "/" + _invalid + "/" + _pendingSteps + "/" + _legacyOnly + "/" + _dynamicOnly +
            " | current(valid-active-reason-hold)=" + Current.Valid + "/" + Current.Active + "/" + Current.Reason + "/" +
            Current.HoldSeconds.ToString("R", CultureInfo.InvariantCulture) +
            " | bands(COMm-pelvisDeg-chestRelativeDeg-nativeGapM-holdS)=0.03/2/2/0.002/0.15 | timeoutRelease=False | holdClock=CONSECUTIVE_FIXED_DT";
        private static bool Nonnegative(float x) => !float.IsNaN(x) && !float.IsInfinity(x) && x >= 0f;
    }
}
