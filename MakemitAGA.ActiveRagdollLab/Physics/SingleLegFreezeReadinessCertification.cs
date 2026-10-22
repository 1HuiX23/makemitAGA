using MakemitAGA.ActiveRagdollLab.Model;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.2.122 read-only Single-Leg freeze certification. This deliberately adds no new tracking-performance
    /// threshold. It composes the already-frozen production contract, the existing PCA/inertia requirements,
    /// the existing 30 mm translation-tether fail-closed contract, existing recovery timeout/abort semantics,
    /// and the already-established Two-Body ALL_GATES_PASS validation.
    /// </summary>
    internal sealed class SingleLegFreezeReadinessCertification
    {
        internal const string Mode = "SINGLE_LEG_FROZEN_BASELINE_CONSOLIDATION_FREEZE_READINESS_V0122";
        internal const string Policy = "EXISTING_SAFETY_CONTRACT_ONLY_NO_NEW_PERFORMANCE_THRESHOLDS";

        internal void Reset(SingleLegChainReport report)
        {
            if (report == null) return;
            report.SingleLegFreezeCertificationMode = Mode;
            report.SingleLegFreezeCertificationPolicy = Policy;
            report.SingleLegFreezeCertificationComplete = false;
            report.SingleLegFreezeProductionContractPass = false;
            report.SingleLegFreezeCalibrationPass = false;
            report.SingleLegFreezeInertiaPass = false;
            report.SingleLegFreezeTwoBodyPass = false;
            report.SingleLegFreezeRunSafetyPass = false;
            report.SingleLegFreezeReady = false;
            report.SingleLegFreezeReason = "TRACE_NOT_FINALIZED";
            report.SingleLegFreezeProductionContractKey = null;
            report.SingleLegFreezeKey = null;
        }

        internal string Evaluate(
            SingleLegChainReport report,
            TwoBodyProxyReport torso,
            bool traceFinalized,
            bool productionContractPass,
            string productionContractKey,
            bool calibrationPass,
            bool inertiaPass)
        {
            if (report == null)
                return "mode=" + Mode + " | complete=False | ready=False | reason=REPORT_MISSING";

            bool hasTrace = report.TraceSamples > 0;
            bool twoBodyComplete = torso != null && torso.FreezeValidationComplete;
            bool twoBodyPass = twoBodyComplete && torso.TwoBodyFreezeReady && !torso.FreezeValidationTraceAborted;

            bool tetherPass =
                report.HipCharacterJointResponseIsolationInvalidAnchorSteps == 0 &&
                report.KneeCharacterJointResponseIsolationInvalidAnchorSteps == 0 &&
                report.HipCharacterJointResponseIsolationPeakAnchorErrorMeters <= HipCharacterJointResponseIsolation.TetherAnchorValidityMeters + 0.000001f &&
                report.KneeCharacterJointResponseIsolationPeakAnchorErrorMeters <= KneeCharacterJointResponseIsolation.TetherAnchorValidityMeters + 0.000001f;

            bool recoveryTimeoutPass =
                report.HipAuthorityRecoveryTimedOut == 0 &&
                report.HipControlFrameRecoveryTimedOut == 0 &&
                report.KneeSoftOffAxisRecoveryTimedOut == 0;

            bool runSafetyPass = traceFinalized && hasTrace && report.Finite && !report.DriveAborted && tetherPass && recoveryTimeoutPass;
            bool complete = traceFinalized && hasTrace && twoBodyComplete;
            bool ready = complete && productionContractPass && calibrationPass && inertiaPass && twoBodyPass && runSafetyPass;

            string reason;
            if (!traceFinalized) reason = "TRACE_NOT_FINALIZED";
            else if (!hasTrace) reason = "NO_SINGLE_LEG_TRACE_SAMPLES";
            else if (!twoBodyComplete) reason = "TWO_BODY_FREEZE_VALIDATION_INCOMPLETE";
            else if (!productionContractPass) reason = "FROZEN_PRODUCTION_CONTRACT_MISMATCH";
            else if (!calibrationPass) reason = "KNEE_PCA_NOT_APPLIED_OR_NOT_READY";
            else if (!inertiaPass) reason = "RIGIDBODY_INERTIA_FOUNDATION_NOT_VALIDATED";
            else if (!twoBodyPass) reason = "TWO_BODY_FREEZE_NOT_READY:" + (torso != null ? (torso.FreezeValidationReason ?? "<none>") : "MISSING");
            else if (!report.Finite) reason = "SINGLE_LEG_NONFINITE";
            else if (report.DriveAborted) reason = "SINGLE_LEG_DRIVE_ABORTED:" + (report.DriveAbortReason ?? "<none>");
            else if (!tetherPass) reason = "TRANSLATION_TETHER_30MM_CONTRACT_FAIL";
            else if (!recoveryTimeoutPass) reason = "SINGLE_LEG_RECOVERY_TIMEOUT";
            else reason = "ALL_EXISTING_FREEZE_CONTRACTS_PASS";

            report.SingleLegFreezeCertificationMode = Mode;
            report.SingleLegFreezeCertificationPolicy = Policy;
            report.SingleLegFreezeCertificationComplete = complete;
            report.SingleLegFreezeProductionContractPass = productionContractPass;
            report.SingleLegFreezeCalibrationPass = calibrationPass;
            report.SingleLegFreezeInertiaPass = inertiaPass;
            report.SingleLegFreezeTwoBodyPass = twoBodyPass;
            report.SingleLegFreezeRunSafetyPass = runSafetyPass;
            report.SingleLegFreezeReady = ready;
            report.SingleLegFreezeReason = reason;
            report.SingleLegFreezeProductionContractKey = productionContractKey;
            report.SingleLegFreezeKey =
                "mode=" + Mode +
                "|complete=" + complete +
                "|production=" + productionContractPass +
                "|pca=" + calibrationPass +
                "|inertia=" + inertiaPass +
                "|twoBody=" + twoBodyPass +
                "|runSafety=" + runSafetyPass +
                "|ready=" + ready +
                "|reason=" + reason +
                "|samples=" + report.TraceSamples +
                "|tetherInvalid(H/K)=" + report.HipCharacterJointResponseIsolationInvalidAnchorSteps + "/" + report.KneeCharacterJointResponseIsolationInvalidAnchorSteps +
                "|tetherPeak(H/K)=" + F(report.HipCharacterJointResponseIsolationPeakAnchorErrorMeters) + "/" + F(report.KneeCharacterJointResponseIsolationPeakAnchorErrorMeters) +
                "|recoveryTimeout(Hip/Frame/KneeSoft)=" + report.HipAuthorityRecoveryTimedOut + "/" + report.HipControlFrameRecoveryTimedOut + "/" + report.KneeSoftOffAxisRecoveryTimedOut +
                "|aborted=" + report.DriveAborted;
            return GetSummary(report);
        }

        internal string GetSummary(SingleLegChainReport report)
        {
            if (report == null)
                return "mode=" + Mode + " | complete=False | ready=False | reason=REPORT_MISSING";
            return "mode=" + Mode +
                   " | policy=" + Policy +
                   " | complete=" + report.SingleLegFreezeCertificationComplete +
                   " | production=" + report.SingleLegFreezeProductionContractPass +
                   " | pca=" + report.SingleLegFreezeCalibrationPass +
                   " | inertia=" + report.SingleLegFreezeInertiaPass +
                   " | twoBody=" + report.SingleLegFreezeTwoBodyPass +
                   " | runSafety=" + report.SingleLegFreezeRunSafetyPass +
                   " | SINGLE_LEG_FREEZE_READY=" + report.SingleLegFreezeReady +
                   " | reason=" + (report.SingleLegFreezeReason ?? "<none>") +
                   " | samples=" + report.TraceSamples +
                   " | tetherInvalid(H/K)=" + report.HipCharacterJointResponseIsolationInvalidAnchorSteps + "/" + report.KneeCharacterJointResponseIsolationInvalidAnchorSteps +
                   " | tetherPeak(H/K)=" + F(report.HipCharacterJointResponseIsolationPeakAnchorErrorMeters) + "/" + F(report.KneeCharacterJointResponseIsolationPeakAnchorErrorMeters) +
                   " | timeout(Hip/Frame/KneeSoft)=" + report.HipAuthorityRecoveryTimedOut + "/" + report.HipControlFrameRecoveryTimedOut + "/" + report.KneeSoftOffAxisRecoveryTimedOut +
                   " | aborted=" + report.DriveAborted + "/" + (report.DriveAbortReason ?? "<none>");
        }

        private static string F(float value) => value.ToString("0.######", System.Globalization.CultureInfo.InvariantCulture);
    }
}
