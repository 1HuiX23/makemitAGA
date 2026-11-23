using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class SingleLegShadowChain
    {
        internal HipAnchorTranslationPair HipForceOwner => _hipAnchorPair;
        internal LandingPulseWindow.Foot PulseFoot(in PhysicsStepContext step) =>
            _runtimeStep.Matches(step) && _footFoundation != null ? _footFoundation.PulseFoot(step,_fixedUpdateSequence) : default;
        internal bool PulseFree(in PhysicsStepContext step, int floor) =>
            !_upperBody.useGravity && !_upperBody.isKinematic && _footFoundation.PulseFootprintFree(step,floor);

        internal void FinishPulseStep(in PhysicsStepContext step, bool applied)
        {
            if (applied)
            {
                var cmd = _hipAnchorPair.Report.Current;
                _parentCommand.HipCommand = cmd.ParentInput;
                _lastHipAppliedAngularAccelerationWorld = cmd.AllocatedHipAlpha;
                if (_traceEnabled)
                {
                    _legTurnCurrent.HipFinalA = cmd.AllocatedHipAlpha;
                    _legTurnCurrent.KneeAngular.HipReallocated = true;
                }
            }
            _hipAnchorPair.FinishFootDecision(step);
            if (applied && _traceEnabled && _report.Side == "LEFT") TracePulseAngular(step);
        }

        // Keep the same-step evidence even when the conditional leg path ended earlier.
        private void TracePulseAngular(in PhysicsStepContext step)
        {
            try
            {
                var leg = _legTurnCurrent;
                var cmd = _hipAnchorPair.Report.Current;
                var knee = leg.KneeAngular;
                bool matched = leg.Matches(step, _fixedUpdateSequence) && leg.Source.Matches(_runtimeSource) &&
                    cmd.JointSolved && cmd.Step.Matches(step) && cmd.Fixed == leg.Fixed && cmd.ChildId == leg.CommittedUpperId &&
                    knee.HipReallocated && knee.Matches(_kneeParentAngularTransport.Enabled, leg.HipFinalA,
                        cmd.AllocatedHipAlpha, leg.KneeRelativeA, leg.KneeTotalA);
                static string R(float value) => value.ToString("R", System.Globalization.CultureInfo.InvariantCulture);
                Plugin.Logger?.LogInfo("[ARL][TURN_PULSE_ANGULAR_V041740] leftLocal-shared=" + _fixedUpdateSequence + "|" + step.Sequence +
                    "/matched=" + matched + "/radSec2(finalHipMinusIssuedTransport-kneeClosure-finalHipReceiptError)=" +
                    R((leg.HipFinalA - knee.Parent).magnitude) + "|" + R((leg.KneeTotalA - (knee.Parent + leg.KneeRelativeA)).magnitude) +
                    "|" + R((leg.HipFinalA - cmd.AllocatedHipAlpha).magnitude));
            }
            catch { } // Optional reporting follows the physical commit.
        }
    }
}
