using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal sealed partial class BilateralLegCoordinator
    {
        private readonly LandingPulseWindow _landingPulse = new LandingPulseWindow(WriteLandingPulse);

        private static void WriteLandingPulse(string evidence)
        {
            try { Plugin.Logger?.LogInfo("[ARL][TURN_RISE_PULSE_V041730] " + evidence); }
            catch { } // Optional logging cannot change a physical decision.
        }

        private bool CompleteLandingPulse(in PhysicsStepContext step, SingleLegShadowChain left, SingleLegShadowChain right,
            bool leftFirst, bool decisionPending, out bool applied)
        {
            applied=false;
            var l = left.PulseFoot(step);
            var r = right.PulseFoot(step);
            ref readonly var lc = ref left.HipAnchorPair.Current;
            ref readonly var rc = ref right.HipAnchorPair.Current;
            bool free = _landingPulse.Advance(step,_torso.MotionPhaseForLanding,l,r,lc,rc);
            if (decisionPending && free && _torso.LandingRiseInput.WitnessKnown(l.Floor) &&
                !_torso.LandingRiseInput.Support.Matches(step) && lc.ParentV.y <= 0f &&
                left.PulseFree(step,l.Floor) && right.PulseFree(step,r.Floor) &&
                left.HipForceOwner.TryGetCommittedInput(step,out var li) && right.HipForceOwner.TryGetCommittedInput(step,out var ri) &&
                LandingRiseAllocation.TryChoose(step,l.Floor,_torso.LandingRiseInput,li,ri,lc,rc,_torso.HipAttitudeCommand,leftFirst,out var choice))
            {
                bool use = PelvisDynamicReceiverIsolation.IsDynamicHipArm(_torso.PelvisReceiverControl.Arm);
                if (use && !_torso.ApplyLandingPulse(step,l.Floor,left.HipForceOwner,right.HipForceOwner,leftFirst,choice)) return false;
                applied=use;
                _landingPulse.RecordIssue(step,choice,lc.ParentV.y,applied);
            }
            if (decisionPending)
            {
                left.FinishPulseStep(step,applied);
                right.FinishPulseStep(step,applied);
                _torso.FinishLandingPulse(step);
            }
            return true;
        }
    }
}
