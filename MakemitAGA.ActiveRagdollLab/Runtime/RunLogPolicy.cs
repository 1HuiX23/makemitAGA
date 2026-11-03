using System;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    // Presentation only. Never used for sampling, controller dt, trace statistics,
    // event windows, readiness or safety. The trace owner supplies its frozen scope.
    internal static class RunLogPolicy
    {
        internal const string HipFormat = "HIP_COM_V041060";
        internal static bool CompactLegWindows(PelvisRunScope scope) => PelvisRunScopes.HasHipCoupling(scope);
        internal static float PeriodicInterval(float configured, bool hipTrace) => hipTrace ? Math.Max(configured, 5f) : configured;
    }
}
