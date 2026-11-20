using MakemitAGA.ActiveRagdollLab.Model;
using MakemitAGA.ActiveRagdollLab.Runtime;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    internal static class LandingReferenceCapturePolicy
    {
        internal static bool NeedsFull(bool compact, bool historyOpen, bool adopted) =>
            !compact || historyOpen || adopted;

        // A parent-only receipt has no valid full-chain pose. Its own identity and
        // geometry are sufficient for the three parent heights and nothing else.
        internal static bool TryParentHeights(in LegLandingReferenceInput input, PhysicsStepContext step,
            SourceStepContext source, int localFixed, out float raw, out float reference, out float actual)
        {
            raw = reference = actual = 0f;
            if (input.Prepared || localFixed <= 0 || input.Fixed != localFixed || !input.Step.Matches(step) ||
                !input.Source.Matches(source) || !input.Parent.Matches(step, source) ||
                !PelvisParentFrame.V(input.RawHip)) return false;
            var r = input.Parent.ReferencePoint;
            var a = input.Parent.ActualPoint;
            if (!PelvisParentFrame.V(r) || !PelvisParentFrame.V(a)) return false;
            raw = input.RawHip.y; reference = r.y; actual = a.y;
            return true;
        }
    }
}
