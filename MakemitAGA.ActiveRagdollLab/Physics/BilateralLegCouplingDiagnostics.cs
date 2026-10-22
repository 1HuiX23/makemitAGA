using System;
using MakemitAGA.ActiveRagdollLab.Model;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.3.0 read-only bilateral foundation observer.
    /// It intentionally avoids performance thresholds. The only new pass/fail condition is exact lifecycle/clock parity
    /// between the two frozen Single-Leg instances. Per-leg safety remains owned by the v0.2.122 freeze certification.
    /// </summary>
    internal sealed class BilateralLegCouplingDiagnostics
    {
        internal const string Mode = "BILATERAL_FROZEN_SINGLE_LEG_REPLICATION_CLOCK_PARITY_READ_ONLY_V0300";

        internal void Reset(BilateralLegReport report, bool bilateralEnabled, bool distinctSourceChains, string primarySide, string secondarySide)
        {
            if (report == null) return;
            report.Mode = Mode;
            report.BilateralEnabled = bilateralEnabled;
            report.DistinctSourceChains = distinctSourceChains;
            report.PrimarySide = primarySide;
            report.SecondarySide = secondarySide;
            report.TraceEnabled = false;
            report.ClockSamples = 0;
            report.FixedSequenceMismatchSamples = 0;
            report.RenderSequenceMismatchSamples = 0;
            report.RenderFreshnessMismatchSamples = 0;
            report.RenderHeldDepthMismatchSamples = 0;
            report.DriveStateMismatchSamples = 0;
            report.MaxFixedSequenceDelta = 0;
            report.MaxRenderSequenceDelta = 0;
            report.MaxHeldDepthDelta = 0;
            report.SimultaneousHipClampSamples = 0;
            report.SimultaneousKneeClampSamples = 0;
            report.FailClosedEvents = 0;
            report.LastFailClosedReason = null;
            report.ClockParityPass = false;
            report.FrozenReplicationPass = false;
            report.FoundationReady = false;
            report.Reason = "TRACE_NOT_FINALIZED";
            report.Key = null;
        }

        internal void Observe(BilateralLegReport report, SingleLegChainReport primary, SingleLegChainReport secondary)
        {
            if (report == null || primary == null || secondary == null)
                return;

            report.ClockSamples++;
            int fixedDelta = Math.Abs(primary.FixedUpdateSequence - secondary.FixedUpdateSequence);
            int renderDelta = Math.Abs(primary.TargetRenderSampleSequence - secondary.TargetRenderSampleSequence);
            int heldDelta = Math.Abs(primary.FixedStepsSinceTargetRenderSample - secondary.FixedStepsSinceTargetRenderSample);

            if (fixedDelta != 0) report.FixedSequenceMismatchSamples++;
            if (renderDelta != 0) report.RenderSequenceMismatchSamples++;
            if (primary.TargetRenderSampleFreshAtFixedStep != secondary.TargetRenderSampleFreshAtFixedStep)
                report.RenderFreshnessMismatchSamples++;
            if (heldDelta != 0) report.RenderHeldDepthMismatchSamples++;
            if (primary.DriveEnabled != secondary.DriveEnabled) report.DriveStateMismatchSamples++;

            if (fixedDelta > report.MaxFixedSequenceDelta) report.MaxFixedSequenceDelta = fixedDelta;
            if (renderDelta > report.MaxRenderSequenceDelta) report.MaxRenderSequenceDelta = renderDelta;
            if (heldDelta > report.MaxHeldDepthDelta) report.MaxHeldDepthDelta = heldDelta;
            if (primary.HipAccelerationClamped && secondary.HipAccelerationClamped) report.SimultaneousHipClampSamples++;
            if (primary.KneeAccelerationClamped && secondary.KneeAccelerationClamped) report.SimultaneousKneeClampSamples++;

            CopyCurrent(report, primary, secondary);
        }

        internal string Finalize(BilateralLegReport report, SingleLegChainReport primary, SingleLegChainReport secondary, bool built, bool distinctSourceChains)
        {
            if (report == null)
                return "ARL bilateral foundation | report=null";

            CopyCurrent(report, primary, secondary);
            report.Built = built;
            report.DistinctSourceChains = distinctSourceChains;
            bool hasTrace = report.ClockSamples > 0 && report.PrimaryTraceSamples > 0 && report.SecondaryTraceSamples > 0;
            report.ClockParityPass = hasTrace &&
                                     report.FixedSequenceMismatchSamples == 0 &&
                                     report.RenderSequenceMismatchSamples == 0 &&
                                     report.RenderFreshnessMismatchSamples == 0 &&
                                     report.RenderHeldDepthMismatchSamples == 0 &&
                                     report.DriveStateMismatchSamples == 0;
            report.FrozenReplicationPass = built && distinctSourceChains &&
                                           report.FailClosedEvents == 0 &&
                                           primary != null && secondary != null &&
                                           primary.SingleLegFreezeReady && secondary.SingleLegFreezeReady &&
                                           !primary.DriveAborted && !secondary.DriveAborted;
            report.FoundationReady = report.ClockParityPass && report.FrozenReplicationPass;

            if (!built) report.Reason = "BILATERAL_NOT_BUILT";
            else if (!distinctSourceChains) report.Reason = "SOURCE_CHAIN_OVERLAP";
            else if (!hasTrace) report.Reason = "INSUFFICIENT_BILATERAL_TRACE";
            else if (!report.ClockParityPass) report.Reason = "BILATERAL_CLOCK_PARITY_FAIL";
            else if (report.FailClosedEvents > 0) report.Reason = "BILATERAL_FAIL_CLOSED_EVENT_OBSERVED";
            else if (primary == null || !primary.SingleLegFreezeReady) report.Reason = "PRIMARY_SINGLE_LEG_FREEZE_NOT_READY";
            else if (secondary == null || !secondary.SingleLegFreezeReady) report.Reason = "SECONDARY_SINGLE_LEG_FREEZE_NOT_READY";
            else if (primary.DriveAborted || secondary.DriveAborted) report.Reason = "LEG_DRIVE_ABORTED";
            else report.Reason = "BILATERAL_FROZEN_REPLICATION_PASS";

            report.Key =
                "MODE=" + Mode +
                "|CLOCK_N=" + report.ClockSamples +
                "|MISMATCH_FIXED=" + report.FixedSequenceMismatchSamples +
                "|RENDER=" + report.RenderSequenceMismatchSamples +
                "|FRESH=" + report.RenderFreshnessMismatchSamples +
                "|HELD=" + report.RenderHeldDepthMismatchSamples +
                "|DRIVE=" + report.DriveStateMismatchSamples +
                "|FAIL_CLOSED=" + report.FailClosedEvents +
                "|FREEZE=" + report.PrimarySingleLegFreezeReady + "/" + report.SecondarySingleLegFreezeReady +
                "|ABORT=" + report.PrimaryDriveAborted + "/" + report.SecondaryDriveAborted +
                "|CLOCK_PARITY=" + report.ClockParityPass +
                "|FROZEN_REPLICATION=" + report.FrozenReplicationPass +
                "|READY=" + report.FoundationReady +
                "|REASON=" + report.Reason;
            return report.GetConsoleSummary();
        }

        internal void CopyCurrent(BilateralLegReport report, SingleLegChainReport primary, SingleLegChainReport secondary)
        {
            if (report == null) return;
            if (primary != null)
            {
                report.PrimaryDriveEnabled = primary.DriveEnabled;
                report.PrimaryTraceSamples = primary.TraceSamples;
                report.PrimaryHipMeanDegrees = primary.TraceHipRotationMeanDegrees;
                report.PrimaryHipMaxDegrees = primary.TraceHipRotationMaxDegrees;
                report.PrimaryKneeMeanDegrees = primary.TraceKneeRotationMeanDegrees;
                report.PrimaryKneeMaxDegrees = primary.TraceKneeRotationMaxDegrees;
                report.PrimaryKneeOffAxisMeanDegrees = primary.TraceKneeOffAxisErrorMeanDegrees;
                report.PrimaryPeakHipAnchorMeters = primary.PeakHipAnchorErrorMeters;
                report.PrimaryPeakKneeAnchorMeters = primary.PeakKneeAnchorErrorMeters;
                report.PrimarySingleLegFreezeReady = primary.SingleLegFreezeReady;
                report.PrimaryDriveAborted = primary.DriveAborted;
                report.PrimaryDriveAbortReason = primary.DriveAbortReason;
            }
            if (secondary != null)
            {
                report.SecondaryDriveEnabled = secondary.DriveEnabled;
                report.SecondaryTraceSamples = secondary.TraceSamples;
                report.SecondaryHipMeanDegrees = secondary.TraceHipRotationMeanDegrees;
                report.SecondaryHipMaxDegrees = secondary.TraceHipRotationMaxDegrees;
                report.SecondaryKneeMeanDegrees = secondary.TraceKneeRotationMeanDegrees;
                report.SecondaryKneeMaxDegrees = secondary.TraceKneeRotationMaxDegrees;
                report.SecondaryKneeOffAxisMeanDegrees = secondary.TraceKneeOffAxisErrorMeanDegrees;
                report.SecondaryPeakHipAnchorMeters = secondary.PeakHipAnchorErrorMeters;
                report.SecondaryPeakKneeAnchorMeters = secondary.PeakKneeAnchorErrorMeters;
                report.SecondarySingleLegFreezeReady = secondary.SingleLegFreezeReady;
                report.SecondaryDriveAborted = secondary.DriveAborted;
                report.SecondaryDriveAbortReason = secondary.DriveAbortReason;
            }
        }
    }
}
