using System;
using System.Text;
using MakemitAGA.ActiveRagdollLab.Physics;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    internal static partial class ActiveRagdollLabRuntime
    {
        // Only this adapter knows the old reports/physics objects. Session coordination
        // consumes typed readiness and explicit operations, never parses display strings.
        private sealed class PelvisRunPort : IPelvisRunPort
        {
            public string Scene => _sceneName + "#" + _sceneHandle;
            public static string ResolveRequestedArm(string requestedArm, PelvisRunScope scope)
            {
                if (PelvisRunScopes.HasHipCoupling(scope))
                    return string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicHipArm
                        ? PelvisDynamicReceiverIsolation.DynamicHipArm : null;
                if (PelvisRunScopes.HasUpperWeight(scope))
                    return string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicUpperLoadArm
                        ? PelvisDynamicReceiverIsolation.DynamicUpperLoadArm : null;
                if (PelvisRunScopes.HasKneeCoupling(scope))
                    return string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicKneeCouplingArm
                        ? PelvisDynamicReceiverIsolation.DynamicKneeCouplingArm : null;
                if (scope == PelvisRunScope.DynamicLegLowerSelfWeightContact)
                    return string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicLowerLoadArm
                        ? PelvisDynamicReceiverIsolation.DynamicLowerLoadArm : null;
                if (scope == PelvisRunScope.DynamicLegFootSelfWeightContact)
                    return string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicFootLoadArm
                        ? PelvisDynamicReceiverIsolation.DynamicFootLoadArm : null;
                if (scope == PelvisRunScope.DynamicLegUnloadedContact)
                    return string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicContactArm
                        ? PelvisDynamicReceiverIsolation.DynamicContactArm : null;
                if (scope == PelvisRunScope.DynamicLegFreeSpace)
                {
                    if (string.Equals(requestedArm, "a", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicLegArmA)
                        return PelvisDynamicReceiverIsolation.DynamicLegArmA;
                    if (string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.DynamicLegArmB)
                        return PelvisDynamicReceiverIsolation.DynamicLegArmB;
                    return null;
                }
                return string.Equals(requestedArm, "b", StringComparison.OrdinalIgnoreCase) || requestedArm == PelvisDynamicReceiverIsolation.ArmB
                    ? PelvisDynamicReceiverIsolation.ArmB : null;
            }
            public string ResolveArm(string requestedArm) => ResolveRequestedArm(requestedArm, _pelvisScope);
            public string Scope => PelvisRunScopes.Name(_pelvisScope);
            public string StopInstruction => PelvisRunScopes.Command(_pelvisScope) + "(stop)";
            public bool CleanupPending => _cleanupPending;
            public string Preflight()
            {
                if (_cleanupPending) return "CLEANUP_PENDING";
                if (_proxy == null || !_proxy.IsBuilt) return _pelvisWithLegs ? "BUILD_REQUIRED | 先执行arl_test_setup。" : "BUILD_REQUIRED | 先执行arl_pelvis_test_setup。";
                if (AnkleTestActive) return "ANKLE_TEST_ACTIVE";
                if (_pelvisWithLegs)
                {
                    if (_legChain == null || !_legChain.IsBuilt) return "BILATERAL_BUILD_REQUIRED | 先arl_test_setup及PCA、hip_stack。";
                    if (!PelvisTestActive && !_legChain.DynamicJoinReady) return "DYNAMIC_JOIN_NOT_READY | 需要双侧PCA/stack、驱动OFF、接触及负载OFF、预览关节已移除；查看arl_test_status。 | " + _legChain.FreeSpaceBoundarySummary();
                }
                else if (_legChain != null) return "TORSO_ONLY_BUILD_REQUIRED | 请先arl_test_cleanup。";
                if (TraceActive && !PelvisTestActive) return "OTHER_TRACE_ACTIVE";
                if (_proxy.Report.GravityEnabled || _proxy.Report.CollidersEnabled) return "TORSO_GRAVITY_AND_COLLIDERS_MUST_BE_OFF";
                return null;
            }
            public void Prepare(string arm)
            {
                if (_pelvisWithLegs) _legChain.SelectFootRelativeReference(true);
                _proxy.SetDrive(false);
                string select = _proxy.SetPelvisReceiver(arm);
                if (!_proxy.PelvisReceiver.Selected || _proxy.PelvisReceiver.Arm != arm || !_proxy.PelvisReceiver.Finite)
                    throw new InvalidOperationException("RECEIVER_SELECT_FAILED:" + select);
                if (!_proxy.BindPelvisLegReference(_legTarget, _secondaryLegTarget))
                    throw new InvalidOperationException("LEG_PARENT_REFERENCE_BIND_FAILED:" + (_proxy.LegParentReferenceIssue ?? "SOURCE_TARGET_OR_NATIVE_POSE"));
                string drive = _proxy.SetDrive(true);
                if (!_proxy.DriveEnabled || !_proxy.PelvisReceiver.Active || !_proxy.PelvisReceiver.Finite)
                    throw new InvalidOperationException("RECEIVER_DRIVE_FAILED:" + drive);
            }
            public void AdvancePreparation(PhysicsStepContext step)
            {
                if (_pelvisWithLegs && _proxy.PelvisReceiverReady && !_legChain.DynamicParentsActive)
                    _legChain.ActivateDynamicParents(step);
                if (_pelvisWithContact && _legChain.DynamicParentsActive && !_legChain.DynamicContactSelected)
                {
                    string contact = _legChain.PrepareDynamicContact(_pelvisScope);
                    _logger?.LogInfo("[ARL][DYNAMIC_CONTACT_PREPARED] " + contact);
                }
            }
            private bool EnvironmentReady => _legChain != null && (_pelvisWithContact
                ? (_legChain.DynamicContactSelected ? _legChain.DynamicContactMatchesScope(_pelvisScope) : !_pelvisTest.Session.Started && _legChain.FreeSpaceReady)
                : !_legChain.DynamicContactSelected && _legChain.FreeSpaceReady);
            public bool Healthy => _proxy != null && _proxy.DriveEnabled && _proxy.Report.FiniteState && _proxy.PelvisReceiver.Finite &&
                (_pelvisWithLegs ? (_proxy.Report.UseDynamicTorsoRecovery &&
                    _proxy.Report.HipGoalGateOnlyHip &&
                    _proxy.Report.UseCoherentHipNormalReference &&
                    _proxy.Report.HipNormalSampleAgePrediction == (_proxy.PelvisReceiver.Arm == PelvisDynamicReceiverIsolation.DynamicLegArmA) &&
                    _legChain != null && EnvironmentReady && !_legChain.AnyDriveAborted &&
                    _legChain.FootRelativeReferenceMatches(true) &&
                    (!_legChain.DynamicParentsActive || _legChain.HipNormalReferenceMatches(_proxy.Report.UseCoherentHipNormalReference, _proxy.Report.HipNormalSampleAgePrediction)) &&
                    (!_pelvisTest.Session.Started || _legChain.DynamicParentsActive)) : _legChain == null);
            public string Issue
            {
                get
                {
                    if (_proxy == null) return "RECEIVER_MISSING";
                    if (_pelvisWithLegs && _legChain != null)
                    {
                        if (!_proxy.Report.UseDynamicTorsoRecovery || !_proxy.Report.HipGoalGateOnlyHip ||
                            !_proxy.Report.UseCoherentHipNormalReference ||
                            _proxy.Report.HipNormalSampleAgePrediction != (_proxy.PelvisReceiver.Arm == PelvisDynamicReceiverIsolation.DynamicLegArmA) ||
                            (_legChain.DynamicParentsActive && !_legChain.HipNormalReferenceMatches(_proxy.Report.UseCoherentHipNormalReference, _proxy.Report.HipNormalSampleAgePrediction)))
                            return "HIP_NORMAL_REFERENCE_V04800_POLICY_CONTRACT_LOST";
                        if (_legChain.AnyDriveAborted)
                        {
                            string primary = _legChain.PrimaryReport?.DriveAbortReason;
                            string secondary = _legChain.SecondaryReport?.DriveAbortReason;
                            if (!string.IsNullOrEmpty(primary)) return primary;
                            if (!string.IsNullOrEmpty(secondary)) return secondary;
                            return "LEG_DRIVE_ABORT";
                        }
                        if (!EnvironmentReady) return _pelvisWithContact ? "DYNAMIC_CONTACT_CONTRACT_LOST | " + _legChain.DynamicContactBoundarySummary() : "DYNAMIC_LEG_FREE_SPACE_CONTRACT_LOST | " + _legChain.FreeSpaceBoundarySummary();
                        if (!_legChain.FootRelativeReferenceMatches(true))
                            return "FOOT_REFERENCE_V04730_BILATERAL_CONTRACT_LOST";
                        if (_pelvisTest.Session.Started && !_legChain.DynamicParentsActive) return "DYNAMIC_LEG_DRIVE_LOST";
                    }
                    return _proxy?.PelvisReceiver?.Issue;
                }
            }
            public bool Ready => _proxy != null && _proxy.PelvisReceiverReady && (!_pelvisWithLegs ||
                (_legChain != null && _legChain.DynamicParentsActive && (!_pelvisWithContact || _legChain.DynamicContactMatchesScope(_pelvisScope))));
            public bool Tracing => _proxy != null && _proxy.Report.TraceEnabled && _proxy.PelvisReceiver.TraceRunning &&
                (!_pelvisWithLegs || (_legChain != null && _legChain.TraceFullyEnabled));
            public string ReadinessSummary() => "passiveResolved=" + (_proxy?.PelvisReceiver?.PassiveResolved ?? 0) +
                " | legReferenceBound=" + (_proxy != null && _proxy.LegParentReferenceBound) +
                " | legsJoined=" + (_pelvisWithLegs && _legChain != null && _legChain.DynamicParentsActive) +
                " | footStepAttribution=READ_ONLY_V04710" +
                " | footRelativeReference=" + (_pelvisWithLegs ? FootRelativeReference.ArmB : "NOT_BUILT") +
                " | footReferenceBilateralMatched=" + (_legChain != null && _legChain.FootRelativeReferenceMatches(true)) +
                " | torsoRecoveryPolicy=" + (_proxy != null && _proxy.Report.UseDynamicTorsoRecovery ? DynamicTorsoRecoveryGate.DynamicPolicy : DynamicTorsoRecoveryGate.LegacyPolicy) +
                " | hipGoalGate=" + (_pelvisWithLegs ? (_proxy != null && _proxy.Report.HipGoalGateOnlyHip ? "HIP_ONLY_KEEP_KNEE" : "HIP_AND_KNEE") : "LEGACY_SCOPE") +
                " | hipNormalReference=" + HipNormalReference.ArmFor(_pelvisWithLegs && _proxy != null && _proxy.Report.UseCoherentHipNormalReference,
                    _pelvisWithLegs && _proxy != null && _proxy.Report.HipNormalSampleAgePrediction) +
                " | hipNormalRecoveryHandoff=" + (_pelvisWithLegs ? HipCompositeReferenceShockAbsorberIsolation.NormalRecoveryHandoffMode : "NOT_BUILT") +
                " | referenceBudget=" + (_proxy?.PelvisReceiver?.ReferenceBudgetMode ?? "<none>") +
                " | referencePlanner=" + (_proxy?.PelvisReceiver?.ReferencePlannerMode ?? "<none>") + " | sameInputComparison=" + _pelvisWithLegs + "/scope=PELVIS_PLANNER_REFERENCE_ONLY" +
                (_pelvisWithLegs && _legChain != null ? " | " + (_pelvisWithContact ? _legChain.DynamicContactBoundarySummary() : _legChain.FreeSpaceBoundarySummary()) : "");
            public void BeginTrace()
            {
                _proxy.SetTrace(true);
                if (_pelvisWithLegs) _legChain.SetTrace(true);
            }
            public string QuiesceAndCapture(bool started, string header, StringBuilder cleanup)
            {
                if (_proxy != null && _proxy.Report.TraceEnabled) _proxy.SetTrace(false);
                if (_pelvisWithLegs && _legChain != null && _legChain.TraceEnabled) _legChain.SetTrace(false);
                string recovery = _proxy?.TorsoRecoverySummary ?? "TORSO_RECOVERY=UNAVAILABLE";
                string legs = _pelvisWithLegs ? CaptureDynamicLegResult(started) : "legsFeet=NOT_BUILT";
                if (_legChain != null) cleanup.AppendLine(_legChain.SetDrive(false));
                if (_proxy != null) cleanup.AppendLine(_proxy.QuiescePelvisReceiverPreserveTrace());
                string native = "<no-native>";
                bool nativeReadback = _proxy != null && _proxy.TryGetPelvisReceiverNativeState(out native);
                bool ready = started && _proxy != null && _proxy.PelvisReceiver.ExposureReady && nativeReadback;
                return (started ? "resultScope=COMPLETED_RUN" : "resultScope=REJECTED_SETUP") + " | " + header +
                    " | RECEIVER_EXPOSURE_DATASET_READY=" + ready + " | RECEIVER_TRACKING_DATASET_READY=" +
                    (ready && _proxy.PelvisReceiver.TrackingComplete) + " | scope=" + Scope + " | HipReaction=" + (PelvisRunScopes.HasHipCoupling(_pelvisScope) ? "TRANSLATION_AT_HIP_ANCHORS" : "NOT_ROUTED") + " | hipAngularControllerReaction=OFF | torsoGravity=OFF\n" +
                    "PELVIS_LEG_REFERENCE_DATASET_READY=" + (ready && _proxy.PelvisReceiver.ParentReferenceReady) + " | scope=VIRTUAL_BILATERAL_HIP_REFERENCE_ONLY\n" +
                    (_proxy?.PelvisReceiver?.GetDetails() ?? "<no-receiver>") + "\n" + native +
                    "\n" + recovery + "\n[TORSO_AT_FINALIZE]\n" + (_proxy?.Report?.GetConsoleSummary() ?? "<not-built>") + "\n" + legs;
            }
            private static string CaptureDynamicLegResult(bool started)
            {
                if (_legChain == null) return PelvisRunScopes.Dataset(_pelvisScope) + "=False | legs=<missing>";
                bool ready = started && _proxy != null && _proxy.PelvisReceiver.ExposureReady && _legChain.SharedTraceContextReady && _legChain.DynamicParentsActive && (_pelvisWithContact ? _legChain.DynamicContactMatchesScope(_pelvisScope) : _legChain.FreeSpaceReady) && !_legChain.AnyDriveAborted &&
                    _legChain.BilateralReport.ClockParityPass && FootRunSafety(_legChain.PrimaryFootReport) && FootRunSafety(_legChain.SecondaryFootReport);
                return (_pelvisWithContact ? _legChain.DynamicContactEvidence(ready, _pelvisScope) : "DYNAMIC_LEG_FREE_SPACE_DATASET_READY=" + ready + " | scope=NO_CONTACT_NO_GRAVITY_NO_HIP_REACTION | loadedSupportCertified=False\n" +
                    _legChain.FreeSpaceBoundarySummary()) + "\n" + _legChain.DynamicParentSummary() + (PelvisRunScopes.HasHipCoupling(_pelvisScope) ? "\n[HIP_ANCHOR_PAIR_AT_FINALIZE]\n" + _legChain.HipAnchorPairDetails() :
                    PelvisRunScopes.HasUpperWeight(_pelvisScope) ? "\n[HIP_WRENCH_INPUT_AUDIT_AT_FINALIZE]\n" + _legChain.HipPelvisLoadPath.GetDetails() : "") +
                    "\n[PRIMARY_LEG_AT_FINALIZE]\n" + _legChain.PrimaryReport.GetFullText() +
                    "\n[SECONDARY_LEG_AT_FINALIZE]\n" + _legChain.SecondaryReport.GetFullText() +
                    "\n[PRIMARY_FOOT_AT_FINALIZE]\n" + _legChain.PrimaryFootReport.GetFullText(_pelvisWithContact) +
                    "\n[SECONDARY_FOOT_AT_FINALIZE]\n" + _legChain.SecondaryFootReport.GetFullText(_pelvisWithContact);
            }
            public bool Restore(StringBuilder cleanup)
            {
                try { return RestorePelvisTestSafeState(cleanup); }
                catch (Exception e)
                {
                    _cleanupPending = true;
                    cleanup.AppendLine("RESTORE_EXCEPTION=" + e.GetType().Name);
                    cleanup.AppendLine(RollbackInternal("RECEIVER_RESTORE_EXCEPTION"));
                    return false;
                }
            }
            public string LiveDetails() => (_proxy?.PelvisReceiver?.GetDetails() ?? "<no-receiver>") + "\n" + (_proxy?.TorsoRecoverySummary ?? "<no-recovery>") + "\n" + (_proxy?.GetPelvisReceiverNativeState() ?? "<no-native>") +
                (_pelvisWithLegs ? "\n" + (_legChain?.DynamicParentSummary() ?? "<no-legs>") + (_pelvisWithContact && _legChain != null ? "\n" + _legChain.DynamicContactBoundarySummary() : "") : "");
            public string Notice() => _automaticStop.Status();
            public void BeginNoticeSession(string stopCommand) { _automaticStop.BeginDriveSession(stopCommand); }
            public void AutomaticStop(string reason) { RecordAutomaticStop("PELVIS_BUNDLE", reason); }
            public void Emit(string text, bool warning, bool deferred)
            {
                if (deferred) _deferredCompletedOutput = text;
                else if (warning) _logger?.LogWarning(text);
                else _logger?.LogInfo(text);
            }
            public void ShowStarted(string text) { try { ActiveRagdollConsolePatch.TryPrintGame(text); } catch { } }
            public void WriteReports() { WriteAllReports(); }
        }

    }
}
