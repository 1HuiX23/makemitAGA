namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class RightFootPrecontact
    {
        // Above 3 mm the v161 B control requires the eight-corner forecast to fall on this rise.
        private const float RisingReviewMeters = .003f;
        private const float ForecastRisingReviewMeters = .005f;

        private bool EligibleRisingReview() =>
            _current.Extended && _current.RisingLocal == 0 &&
            _current.VerifiedUnloaded == _current.Solves && _current.LoadViolations == 0 &&
            _last.Min >= 0f && _last.Min <= ForecastRisingReviewMeters &&
            float.IsFinite(_last.ControlMin) && _last.ControlMin < -FootContactGeometryLimits.OrdinaryPenetrationMeters &&
            FitsExtraStep(_last.Step.Dt);

        // The lifecycle makes this decision only after ordinary entry, recovery and miss.
        internal bool TryKeepAtBoundary(End reason)
        {
            if (reason != End.Rising) return TryExtendAtExpiry(reason);
            if (Selected != Mode.FallingForecastRise5mm || !EligibleRisingReview()) return false;
            bool wide = _last.Min > RisingReviewMeters;
            bool wideForecast = wide && _last.Predicted < _last.Min;
            if (wide && !wideForecast) return false;
            _current.WideForecastAtRise = wideForecast;
            _current.RisingLocal = _last.Local;
            _current.RisingPending = true;
            WriteRisingReview("HoldOneSolve", _last.Local, _last.Local, wideForecast);
            return true;
        }

        private End EvaluateRisingReview(in Sample s, bool missInProgress)
        {
            if (missInProgress) return End.Miss;
            // Require the actual pre-solve native readback from the held step. Adjacency includes both clocks.
            if (_current.LastUnloadedLocal != _current.RisingLocal || _current.VerifiedUnloaded != _current.Solves ||
                _current.LoadViolations != 0) return End.Fault;
            var budget = CheckExtendedBudget(s);
            if (budget != End.None) return budget;
            // The lowest corner can already be moving down while both sampled positions still rise.
            return s.CornerV < 0f ? End.None : End.Rising;
        }

        private void FinishRisingReview(string action, int local)
        {
            if (!_current.RisingPending) return;
            WriteRisingReview(action, _current.RisingLocal, local, _current.WideForecastAtRise);
            _current.RisingPending = false;
        }

        private void WriteRisingReview(string action, int rising, int decision, bool wideForecast)
        {
            if (_log == null) return;
            var s = _last;
            bool measured = decision == s.Local && s.Fresh && OnWindowFloor(s);
            _log("[ARL][RIGHT_PRECONTACT_RISING_V041560] mode=" + Selected + "/action=" + action +
                "/epoch-origin-support=" + _current.Epoch + "|" + _current.Origin + "|" + _current.Support +
                "/rising-decision-shared-source=" + rising + "|" + decision + "|" + s.Step.Sequence + "|" + s.Source.Sequence +
                "/min-controlMm-cornerMmSec=" + Mm(measured, s.Min) + "|" + Mm(measured, s.ControlMin) + "|" + Mm(measured, s.CornerV) +
                "/extraBefore-nextDtMs=" + N(_current.ExtraSeconds*1000f) + "|" + N(s.Step.Dt*1000f) +
                "/scheduled-unloaded-violations=" + _current.Solves + "|" + _current.VerifiedUnloaded + "|" + _current.LoadViolations +
                "/predictedMinMm=" + Mm(measured, s.Predicted) + "/wideForecastAtRise=" + wideForecast +
                "/reviewLimitMm=" + N(ForecastRisingReviewMeters*1000f));
        }
    }
}
