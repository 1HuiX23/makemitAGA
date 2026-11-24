using System;

namespace ARL.Contracts
{
    internal static class Check
    {
        private static int _assertions, _groups, _failedGroups;
        internal static void That(bool condition, string description)
        {
            _assertions++;
            if (!condition) throw new InvalidOperationException(description);
        }
        internal static void Near(float actual, float expected, float tolerance, string description) =>
            That(float.IsFinite(actual) && MathF.Abs(actual - expected) <= tolerance,
                description + " (actual=" + actual + ", expected=" + expected + ")");
        internal static void Throws<T>(Action action, string description) where T : Exception
        {
            try { action(); }
            catch (T) { That(true, description); return; }
            That(false, description);
        }
        internal static void Run(string name, Action test)
        {
            _groups++;
            try { test(); Console.WriteLine("PASS " + name); }
            catch (Exception e) { _failedGroups++; Console.Error.WriteLine("FAIL " + name + ": " + e); }
        }
        internal static int Finish()
        {
            Console.WriteLine("CONTRACTS groups=" + _groups + " assertions=" + _assertions + " failedGroups=" + _failedGroups);
            return _failedGroups == 0 ? 0 : 1;
        }
    }

    internal static class Program
    {
        public static int Main()
        {
            Console.WriteLine("ARL managed contracts; no Unity/IL2CPP/PhysX certification.");
            RuntimeContracts.Run();
            RightFootPrecontactContracts.Run();
            LeftFootReleaseContracts.Run();
            LeftShinContracts.Run();
            LeftFootLeadContracts.Run();
            LeftLegPreparationContracts.Run();
            FootTransitionContracts.Run();
            TurnHandoffContracts.Run();
            TurnLandingContracts.Run();
            LandingTractionContracts.Run();
            LandingRiseContracts.Run();
            LandingPulseContracts.Run();
            ContinuousLandingContracts.Run();
            CombinedLandingContracts.Run();
            LandingTrackingContracts.Run();
            LandingPreviewContracts.Run();
            TurnHeightContracts.Run();
            Maintenance170Contracts.Run();
            CoordinatorContracts.Run();
            FoundationContracts.Run();
            TrajectoryContracts.Run();
            SampleTimingContracts.Run();
            FootEvidenceContracts.Run();
            SupportReferenceContracts.Run();
            SupportBrakingContracts.Run();
            SupportRecoveryContracts.Run();
            HipLandingPreparationContracts.Run();
            LandingResponseContracts.Run();
            LandingGeometryContracts.Run();
            LandingEvidenceContracts.Run();
            LandingOpportunityContracts.Run();
            LandingObservationContracts.Run();
            LandingEventHistoryContracts.Run();
            SwingKneeContracts.Run();
            EarlyLandingContracts.Run();
            ParentHeightContracts.Run();
            RightAnkleContracts.Run();
            RightKneePivotContracts.Run();
            RightThighTrackingContracts.Run();
            RightFootTorqueContracts.Run();
            RightFootLandingContracts.Run();
            RightAnkleSequenceContracts.Run();
            KneeLandingPreparationContracts.Run();
            ComFeedbackContracts.Run();
            KneeGroundContracts.Run();
            KneeGroundExtensionContracts.Run();
            KneeGroundTrajectoryContracts.Run();
            KneeGroundFeasibilityContracts.Run();
            FootGroundTrajectoryContracts.Run();
            MakemitAGA.ActiveRagdollLab.Physics.FootAnkleProxyFoundation.RunKneeProgressContracts();
            MakemitAGA.ActiveRagdollLab.Physics.FootAnkleProxyFoundation.RunReferenceInputContracts();
            MakemitAGA.ActiveRagdollLab.Physics.BilateralLegCoordinator.RunDynamicContactBoundaryContracts();
            HipJoinContracts.Run();
            HipContactPreparationContracts.Run();
            MakemitAGA.ActiveRagdollLab.Physics.SingleLegShadowChain.RunKneeReferenceContracts();
            MakemitAGA.ActiveRagdollLab.Physics.PelvisDynamicReceiverIsolation.RunHipCompletionContracts();
            MakemitAGA.ActiveRagdollLab.Physics.HipPointResponseAudit.RunRotationWindowContracts();
            HipAllocationContracts.Run();
            HipBoundedTradeContracts.Run();
            LoadTraceContracts.Run();
            return Check.Finish();
        }
    }
}
