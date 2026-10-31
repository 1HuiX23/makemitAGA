using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    /// <summary>
    /// v0.3.x bilateral coordinator. It owns two instances of the already frozen SingleLegShadowChain.
    /// No controller math is duplicated here. Both sides share the same immutable profile and Two-Body parent.
    /// </summary>
    internal sealed partial class BilateralLegCoordinator
    {
        private readonly ActiveRagdollLabProfile _profile;
        private readonly SingleLegTargetReport _primaryTarget;
        private readonly SingleLegTargetReport _secondaryTarget;
        private readonly Transform _targetRoot;
        private readonly TwoBodyShadowProxy _torso;
        private readonly bool _bilateralEnabled;
        private readonly BilateralLegReport _report = new BilateralLegReport();
        private readonly KneeSafetyContextReport _kneeSafetyReport = new KneeSafetyContextReport();
        private readonly BilateralLegCouplingDiagnostics _diagnostics = new BilateralLegCouplingDiagnostics();
        private readonly KneeSafetyContextRecorder _kneeSafety = new KneeSafetyContextRecorder();

        private SingleLegShadowChain _primary;
        private SingleLegShadowChain _secondary;
        private readonly StepContextCursor _contextCursor = new StepContextCursor();
        private int _traceContextSamples, _traceContextMismatches;
        private bool _traceEnabled;
        private bool _destroyed;
        private bool _distinctSourceChains;
        private bool _bilateralFailClosedLatched;
        private readonly FootSupportSurfaceIsolation _supportSurface = new FootSupportSurfaceIsolation();
        internal FootSupportSurfaceIsolation SupportSurface => _supportSurface;
        private readonly HipPelvisLoadPathObserver _hipPelvisLoadPath = new HipPelvisLoadPathObserver();
        internal HipPelvisLoadPathReport HipPelvisLoadPath => _hipPelvisLoadPath.Report;

        internal string ConfigureFootSupportSurface(string arm)
        {
            if (!IsBuilt || _primary == null || _secondary == null)
                return "ARL support surface | success=False | reason=BILATERAL_FOUNDATION_REQUIRED";
            _primary.RefreshFootSupportProbe();
            _secondary.RefreshFootSupportProbe();
            bool ok = _supportSurface.Configure(arm, _primary.CurrentFootGround, _secondary.CurrentFootGround,
                _primary.FootContactShadowCollider, _secondary.FootContactShadowCollider,
                PrimaryFootReport, SecondaryFootReport, out string error);
            if (ok)
            {
                _primary.BindFootSupportSurface(_supportSurface);
                _secondary.BindFootSupportSurface(_supportSurface);
                _primary.RefreshFootSupportProbe();
                _secondary.RefreshFootSupportProbe();
                ok = _supportSurface.Finite && PrimaryFootReport.GroundGeometryReady && SecondaryFootReport.GroundGeometryReady &&
                    PrimaryFootReport.GroundColliderInstanceId == _supportSurface.SelectedId &&
                    SecondaryFootReport.GroundColliderInstanceId == _supportSurface.SelectedId;
                if (!ok) error = _supportSurface.InvalidReason ?? "SELECTED_FLOOR_BILATERAL_PROBE_NOT_READY";
            }
            string evidence = _supportSurface.Summary();
            if (!ok) evidence += " | cleanup=" + StopFootSupportSurface();
            return "ARL support surface | success=" + ok + " | reason=" + (error ?? "<none>") + " | " + evidence;
        }

        internal string StopFootSupportSurface()
        {
            // Feet are quiesced by the caller. Always detach the probe even if object disposal fails.
            _primary?.BindFootSupportSurface(null);
            _secondary?.BindFootSupportSurface(null);
            return _supportSurface.Stop();
        }

        // v0.4.1.1: Foot proxy objects use HideFlags.DontSave, so per-foot scene enumeration is not a
        // sufficient structural guarantee that LEFT↔RIGHT Foot self-collision is isolated. The coordinator
        // therefore owns this one bilateral pair explicitly and validates it before any Foot collider is enabled.
        private Collider _contactShadowSelfPairPrimary;
        private Collider _contactShadowSelfPairSecondary;
        private bool _contactShadowSelfPairOwned;
        private bool _contactShadowSelfPairReadback;
        private int _contactShadowSelfPairRestoreFailures;

        public BilateralLegCoordinator(
            ActiveRagdollLabProfile profile,
            SingleLegTargetReport primaryTarget,
            SingleLegTargetReport secondaryTarget,
            Transform targetRoot,
            TwoBodyShadowProxy torso,
            bool bilateralEnabled)
        {
            _profile = profile ?? throw new ArgumentNullException(nameof(profile));
            _primaryTarget = primaryTarget ?? throw new ArgumentNullException(nameof(primaryTarget));
            _secondaryTarget = secondaryTarget;
            _targetRoot = targetRoot ?? throw new ArgumentNullException(nameof(targetRoot));
            _torso = torso ?? throw new ArgumentNullException(nameof(torso));
            _bilateralEnabled = bilateralEnabled;
        }

        public SingleLegChainReport Report => _primary != null ? _primary.Report : null;
        public SingleLegChainReport PrimaryReport => _primary != null ? _primary.Report : null;
        public SingleLegChainReport SecondaryReport => _secondary != null ? _secondary.Report : null;
        public BilateralLegReport BilateralReport => _report;
        public KneeSafetyContextReport KneeSafetyReport => _kneeSafetyReport;
        public FootAnkleFoundationReport PrimaryFootReport => _primary != null ? _primary.FootReport : null;
        public FootAnkleFoundationReport SecondaryFootReport => _secondary != null ? _secondary.FootReport : null;
        public bool IsBuilt => !_destroyed && _primary != null && _primary.IsBuilt && (!_bilateralEnabled || (_secondary != null && _secondary.IsBuilt));
        public bool DriveEnabled => _primary != null && _primary.DriveEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.DriveEnabled));
        public bool AnyDriveEnabled => (_primary != null && _primary.DriveEnabled) || (_secondary != null && _secondary.DriveEnabled);
        public bool LastRollbackSucceeded { get; private set; } = true;
        // Any-active is used by runtime cleanup so a historical/exceptional half-open child cannot be hidden by
        // the coordinator latch. Fully-enabled is used for atomic start/no-op decisions.
        public bool TraceEnabled => _traceEnabled || (_primary != null && _primary.TraceEnabled) || (_secondary != null && _secondary.TraceEnabled);
        public bool TraceFullyEnabled => _traceEnabled && _primary != null && _primary.TraceEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.TraceEnabled));
        public bool KneeAxisCalibrationCollecting => (_primary != null && _primary.KneeAxisCalibrationCollecting) || (_secondary != null && _secondary.KneeAxisCalibrationCollecting);
        public bool BilateralEnabled => _bilateralEnabled;
        public bool FrozenProductionStackReady => _primary != null && _primary.FrozenProductionStackReady && (!_bilateralEnabled || (_secondary != null && _secondary.FrozenProductionStackReady));
        public bool AnyDriveAborted => (_primary != null && _primary.Report != null && _primary.Report.DriveAborted) || (_secondary != null && _secondary.Report != null && _secondary.Report.DriveAborted);
        public bool FootProductionBaselineReady => _primary != null && _primary.FootProductionBaselineReady && (!_bilateralEnabled || (_secondary != null && _secondary.FootProductionBaselineReady));
        public bool FootRuntimeContractReady => _primary != null && _primary.FootRuntimeContractReady && (!_bilateralEnabled || (_secondary != null && _secondary.FootRuntimeContractReady));
        public string AnkleTransportArm => _primary != null ? _primary.AnkleTransportArm : FootAnkleTranslationTransportIsolation.ArmA;
        public string AnkleAngularTransportPolicy => _primary != null ? _primary.AnkleAngularTransportPolicy : FootAnkleAngularTransport.Production;
        public string FootAnkleDriveArm => _primary != null ? _primary.FootAnkleDriveArm : FootAnkleSourceDrivenRelativePdIsolation.ArmA;
        public string AnkleConstraintArm => _primary != null ? _primary.AnkleConstraintArm : FootAnkleConstraintTopologyIsolation.ArmA;
        public string AnkleTargetAccelerationMode => _primary != null ? _primary.AnkleTargetAccelerationMode : FootAnkleSourceDrivenRelativePdIsolation.ProductionTargetAccelerationMode;
        public string TranslationCaptureAuthorityArm => _primary != null ? _primary.TranslationCaptureAuthorityArm : FootAnkleTranslationCaptureAuthorityIsolation.Production;
        public bool TranslationCaptureAuthorityArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.TranslationCaptureAuthorityArm, _secondary.TranslationCaptureAuthorityArm, StringComparison.Ordinal)));
        public string TranslationPredictiveTailCaptureArm => _primary != null ? _primary.TranslationPredictiveTailCaptureArm : FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA;
        public bool TranslationPredictiveTailCaptureEnabled => _primary != null && _primary.TranslationPredictiveTailCaptureEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.TranslationPredictiveTailCaptureEnabled));
        public bool TranslationPredictiveTailCaptureFinite => _primary != null && _primary.TranslationPredictiveTailCaptureFinite && (!_bilateralEnabled || (_secondary != null && _secondary.TranslationPredictiveTailCaptureFinite));
        public bool TranslationPredictiveTailCaptureArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.TranslationPredictiveTailCaptureArm, _secondary.TranslationPredictiveTailCaptureArm, StringComparison.Ordinal)));
        public string FootContactShadowArm => _primary != null ? _primary.FootContactShadowArm : FootQualifiedContactShadow.ArmA;
        public bool FootContactShadowActive => _primary != null && _primary.FootContactShadowActive && (!_bilateralEnabled || (_secondary != null && _secondary.FootContactShadowActive));
        public bool FootContactShadowFinite => _primary != null && _primary.FootContactShadowFinite && (!_bilateralEnabled || (_secondary != null && _secondary.FootContactShadowFinite));
        public bool StanceOnlyContactLifecycleEnabled => _primary != null && _primary.StanceContactLifecycleEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.StanceContactLifecycleEnabled));
        public bool StanceOnlyContactLifecycleFinite => _primary != null && _primary.StanceContactLifecycleFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceContactLifecycleFinite));
        public string StanceContactFrictionIsolationArm => _primary != null ? _primary.StanceContactFrictionIsolationArm : FootStanceContactFrictionIsolation.ArmA;
        public bool StanceContactFrictionIsolationFinite => _primary != null && _primary.StanceContactFrictionIsolationFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceContactFrictionIsolationFinite));
        public bool StanceContactFrictionIsolationArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.StanceContactFrictionIsolationArm, _secondary.StanceContactFrictionIsolationArm, StringComparison.Ordinal)));
        public bool StanceContactZeroFriction => _primary != null && _primary.StanceContactZeroFriction && (!_bilateralEnabled || (_secondary != null && _secondary.StanceContactZeroFriction));
        public string StanceLowerGroundAuthorityArm => _primary != null ? _primary.StanceLowerGroundAuthorityArm : FootStanceLowerGroundAnchorAuthority.ArmA;
        public bool StanceLowerGroundAuthorityEnabled => _primary != null && _primary.StanceLowerGroundAuthorityEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.StanceLowerGroundAuthorityEnabled));
        public bool StanceLowerGroundAuthorityFinite => _primary != null && _primary.StanceLowerGroundAuthorityFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceLowerGroundAuthorityFinite));
        public bool StanceLowerGroundAuthorityArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.StanceLowerGroundAuthorityArm, _secondary.StanceLowerGroundAuthorityArm, StringComparison.Ordinal)));
        public string StanceFootNativeGravityArm => _primary != null ? _primary.StanceFootNativeGravityArm : FootStanceNativeGravitySupportLoadIsolation.ArmA;
        public bool StanceFootNativeGravityEnabled => _primary != null && _primary.StanceFootNativeGravityEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.StanceFootNativeGravityEnabled));
        public bool StanceFootNativeGravityFinite => _primary != null && _primary.StanceFootNativeGravityFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceFootNativeGravityFinite));
        public bool StanceFootNativeGravityArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.StanceFootNativeGravityArm, _secondary.StanceFootNativeGravityArm, StringComparison.Ordinal)));
        public string StanceLowerNativeGravityArm => _primary != null ? _primary.StanceLowerNativeGravityArm : LowerStanceNativeGravitySupportLoadIsolation.ArmA;
        public bool StanceLowerNativeGravityEnabled => _primary != null && _primary.StanceLowerNativeGravityEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.StanceLowerNativeGravityEnabled));
        public bool StanceLowerNativeGravityFinite => _primary != null && _primary.StanceLowerNativeGravityFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceLowerNativeGravityFinite));
        public bool StanceLowerNativeGravityArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.StanceLowerNativeGravityArm, _secondary.StanceLowerNativeGravityArm, StringComparison.Ordinal)));
        internal string UpperSelfWeightArm => _primary?.UpperSelfWeightArm ?? UpperSelfWeightSupportNormalLoadTransferIsolation.IdleArm;
        internal bool UpperSelfWeightArmsMatch => _primary != null && (!_bilateralEnabled ||
            (_secondary != null && _primary.UpperSelfWeightArm == _secondary.UpperSelfWeightArm));
        internal bool UpperSelfWeightFinite => _primary != null && _primary.UpperSelfWeightFinite &&
            (!_bilateralEnabled || (_secondary != null && _secondary.UpperSelfWeightFinite));
        internal bool UpperSelfWeightOff => (_primary == null || _primary.UpperSelfWeightOff) && (_secondary == null || _secondary.UpperSelfWeightOff);
        internal string SetUpperSelfWeightTransfer(string arm)
        {
            const string prefix = "ARL bilateral Upper self-weight transfer | ";
            bool selected = UpperSelfWeightSupportNormalLoadTransferIsolation.IsExperimentArm(arm);
            if (selected && (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null) || AnyDriveEnabled || TraceEnabled))
                return prefix + "success=False | reason=BUILT_BILATERAL_DRIVE_TRACE_OFF_REQUIRED";
            string pError = null, sError = null;
            bool pOk = _primary == null || _primary.SetUpperSelfWeightTransfer(arm, out pError);
            bool sOk = _secondary == null || _secondary.SetUpperSelfWeightTransfer(arm, out sError);
            bool contract = selected ? UpperSelfWeightArmsMatch && UpperSelfWeightFinite && UpperSelfWeightArm == arm : UpperSelfWeightOff;
            if (!pOk || !sOk || !contract)
            {
                _primary?.SetUpperSelfWeightTransfer(UpperSelfWeightSupportNormalLoadTransferIsolation.IdleArm, out _);
                _secondary?.SetUpperSelfWeightTransfer(UpperSelfWeightSupportNormalLoadTransferIsolation.IdleArm, out _);
                return prefix + "success=False | primary=" + pError + " | secondary=" + sError + " | rolledBackOff=" + UpperSelfWeightOff;
            }
            return prefix + "success=True | arm=" + arm + " | enabled=" + (arm == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB) +
                " | scope=EXPLICIT_STATIC_UPPER_LOWER_FOOT_COM_FORCE_PAIRS";
        }
        internal string QuiesceUpperSelfWeightPreserveTrace()
        {
            string pError = null, sError = null;
            bool pOk = _primary == null || _primary.QuiesceUpperSelfWeightPreserveTrace(out pError);
            bool sOk = _secondary == null || _secondary.QuiesceUpperSelfWeightPreserveTrace(out sError);
            return "ARL bilateral Upper self-weight quiesce | success=" + (pOk && sOk) + " | primary=" + pError + " | secondary=" + sError;
        }

        internal string KneeSupportReactionArm => _primary?.KneeSupportReactionArm ?? KneeSupportNormalReactionIsolation.ArmA;
        internal bool KneeSupportReactionArmsMatch => _primary != null && (!_bilateralEnabled ||
            (_secondary != null && _primary.KneeSupportReactionArm == _secondary.KneeSupportReactionArm));
        internal bool KneeSupportReactionFinite => _primary != null && _primary.KneeSupportReactionFinite &&
            (!_bilateralEnabled || (_secondary != null && _secondary.KneeSupportReactionFinite));
        internal bool KneeSupportReactionOff => (_primary == null || _primary.KneeSupportReactionOff) &&
            (_secondary == null || _secondary.KneeSupportReactionOff);
        internal string SetKneeSupportNormalReaction(bool enabled)
        {
            const string prefix = "ARL bilateral Knee support-normal reaction | ";
            if (enabled && (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null) || AnyDriveEnabled || TraceEnabled))
                return prefix + "success=False | reason=BUILT_BILATERAL_DRIVE_TRACE_OFF_REQUIRED";
            string pError = null, sError = null;
            bool pOk = _primary == null || _primary.SetKneeSupportNormalReaction(enabled, out pError);
            bool sOk = _secondary == null || _secondary.SetKneeSupportNormalReaction(enabled, out sError);
            string expected = enabled ? KneeSupportNormalReactionIsolation.ArmB : KneeSupportNormalReactionIsolation.ArmA;
            bool contract = enabled ? KneeSupportReactionArmsMatch && KneeSupportReactionFinite && KneeSupportReactionArm == expected : KneeSupportReactionOff;
            if (!pOk || !sOk || !contract)
            {
                _primary?.SetKneeSupportNormalReaction(false, out _);
                _secondary?.SetKneeSupportNormalReaction(false, out _);
                return prefix + "success=False | primary=" + pError + " | secondary=" + sError + " | rolledBackOff=" + KneeSupportReactionOff;
            }
            return prefix + "success=True | arm=" + expected + " | enabled=" + enabled + " | scope=EXACT_STANCE_SUPPORT_NORMAL_COM_ONLY";
        }

        public string StanceFootLowerReciprocalArm => _primary != null ? _primary.StanceFootLowerReciprocalArm : FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmA;
        public bool StanceFootLowerReciprocalEnabled => _primary != null && _primary.StanceFootLowerReciprocalEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.StanceFootLowerReciprocalEnabled));
        public bool StanceFootLowerReciprocalFinite => _primary != null && _primary.StanceFootLowerReciprocalFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceFootLowerReciprocalFinite));
        public bool StanceFootLowerReciprocalArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.StanceFootLowerReciprocalArm, _secondary.StanceFootLowerReciprocalArm, StringComparison.Ordinal)));
        public string StanceLowerSelfWeightTransferArm => _primary != null ? _primary.StanceLowerSelfWeightTransferArm : LowerSelfWeightSupportNormalLoadTransferIsolation.ArmA;
        public bool StanceLowerSelfWeightTransferEnabled => _primary != null && _primary.StanceLowerSelfWeightTransferEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.StanceLowerSelfWeightTransferEnabled));
        public bool StanceLowerSelfWeightTransferFinite => _primary != null && _primary.StanceLowerSelfWeightTransferFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceLowerSelfWeightTransferFinite));
        public bool StanceLowerSelfWeightTransferArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.StanceLowerSelfWeightTransferArm, _secondary.StanceLowerSelfWeightTransferArm, StringComparison.Ordinal)));
        public string FootPositionSolverArm => _primary != null ? _primary.FootPositionSolverArm : FootSolverIterationBaselineContract.PositionArmA;
        public bool FootPositionSolverEnabled => _primary != null && _primary.FootPositionSolverEnabled &&
            (!_bilateralEnabled || (_secondary != null && _secondary.FootPositionSolverEnabled));
        public bool FootPositionSolverOff => _primary != null && !_primary.FootPositionSolverEnabled &&
            _primary.FootPositionSolverArm == FootSolverIterationBaselineContract.PositionArmA &&
            (!_bilateralEnabled || (_secondary != null && !_secondary.FootPositionSolverEnabled &&
                _secondary.FootPositionSolverArm == FootSolverIterationBaselineContract.PositionArmA));
        public bool FootPositionSolverArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null &&
            string.Equals(_primary.FootPositionSolverArm, _secondary.FootPositionSolverArm, StringComparison.Ordinal)));
        public string FootVelocitySolverArm => _primary != null ? _primary.FootVelocitySolverArm : FootSolverIterationBaselineContract.ArmA;
        public bool FootVelocitySolverFinite => _primary != null && _primary.FootVelocitySolverFinite && (!_bilateralEnabled || (_secondary != null && _secondary.FootVelocitySolverFinite));
        public bool FootVelocitySolverArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.FootVelocitySolverArm, _secondary.FootVelocitySolverArm, StringComparison.Ordinal)));
        public bool FootVelocitySolverBaselineRestored => _primary != null && _primary.FootVelocitySolverBaselineRestored && (!_bilateralEnabled || (_secondary != null && _secondary.FootVelocitySolverBaselineRestored));
        public string StanceSupportAngularArm => _primary != null ? _primary.StanceSupportAngularArm : FootStanceSupportNormalAngularStabilization.ArmA;
        public bool StanceSupportAngularEnabled => _primary != null && _primary.StanceSupportAngularEnabled && (!_bilateralEnabled || (_secondary != null && _secondary.StanceSupportAngularEnabled));
        public bool StanceSupportAngularFinite => _primary != null && _primary.StanceSupportAngularFinite && (!_bilateralEnabled || (_secondary != null && _secondary.StanceSupportAngularFinite));
        public bool StanceSupportAngularArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.StanceSupportAngularArm, _secondary.StanceSupportAngularArm, StringComparison.Ordinal)));

        public string FootCollisionDetectionModeArm => _primary != null ? _primary.FootCollisionDetectionModeArm : FootCollisionDetectionModeIsolation.IdleArm;
        public bool FootCollisionDetectionModeFinite => _primary != null && _primary.FootCollisionDetectionModeFinite && (!_bilateralEnabled || (_secondary != null && _secondary.FootCollisionDetectionModeFinite));
        public bool FootCollisionDetectionModeArmsMatch => _primary != null && (!_bilateralEnabled || (_secondary != null && string.Equals(_primary.FootCollisionDetectionModeArm, _secondary.FootCollisionDetectionModeArm, StringComparison.Ordinal)));
        public string PrimarySide => _primaryTarget != null ? _primaryTarget.RequestedSide : null;
        public string SecondarySide => _secondaryTarget != null ? _secondaryTarget.RequestedSide : null;

        public string Build()
        {
            if (IsBuilt)
                return GetStatus();

            _distinctSourceChains = ValidateDistinctTargets(_primaryTarget, _secondaryTarget, _bilateralEnabled);
            if (!_distinctSourceChains)
                return "ARL bilateral build：source chain overlap / invalid target pairing；未创建物理对象。";

            try
            {
                // Historical closure logs are emitted by the primary only. The secondary still computes the exact same
                // reports/freeze certification but does not duplicate every immutable v0.2.x closure line at trace stop.
                _primary = new SingleLegShadowChain(_profile, _primaryTarget, _targetRoot, _torso);
                string primaryBuild = _primary.Build();
                if (!_primary.IsBuilt)
                {
                    string cleanup = RollbackInternal("primary-build-failed", out bool cleanupSucceeded);
                    return primaryBuild + (cleanupSucceeded ? string.Empty : "\nARL bilateral build cleanup incomplete | " + cleanup);
                }

                string secondaryBuild = "secondary=disabled";
                if (_bilateralEnabled)
                {
                    _secondary = new SingleLegShadowChain(_profile, _secondaryTarget, _targetRoot, _torso);
                    secondaryBuild = _secondary.Build();
                    if (!_secondary.IsBuilt)
                    {
                        string cleanup = RollbackInternal("secondary-build-failed", out bool cleanupSucceeded);
                        return primaryBuild + "\n" + secondaryBuild + "\nARL bilateral build：secondary chain失败，已整体回滚。" +
                               (cleanupSucceeded ? string.Empty : "\nARL bilateral build cleanup incomplete | " + cleanup);
                    }
                }

                _diagnostics.Reset(_report, _bilateralEnabled, _distinctSourceChains, PrimarySide, SecondarySide);
                _kneeSafety.Reset(_kneeSafetyReport);
                _report.Built = true;
                _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);

                return
                    "ARL bilateral build | success=True" +
                    " | enabled=" + _bilateralEnabled +
                    " | primary=" + (PrimarySide ?? "<none>") +
                    " | secondary=" + (SecondarySide ?? "<none>") +
                    " | distinct=" + _distinctSourceChains +
                    "\n" + primaryBuild +
                    (_bilateralEnabled ? "\n" + secondaryBuild : string.Empty);
            }
            catch (Exception e)
            {
                string cleanup = RollbackInternal("exception:" + e.GetType().Name, out bool cleanupSucceeded);
                return "ARL bilateral build 失败：" + e.GetType().Name + " / " + e.Message +
                       (cleanupSucceeded ? string.Empty : " | cleanupIncomplete=True | " + cleanup);
            }
        }

        internal bool SourceContextReady => _contextCursor.Source.Valid && _torso.CompletedSource.Matches(_contextCursor.Source);
        internal bool SharedTraceContextReady => _traceContextSamples > 0 && _traceContextMismatches == 0;
        internal bool NativeBodiesSafeOff => _primary != null && _secondary != null &&
            _primary.NativeBodiesSafeOff && _secondary.NativeBodiesSafeOff;
        internal bool DynamicParentsActive => _primary != null && _secondary != null &&
            _primary.DynamicParentActive && _secondary.DynamicParentActive;
        internal bool HipNormalReferenceMatches(bool selected, bool sampleAgePrediction) => _primary != null && _secondary != null &&
            _primary.HipNormalReferenceMatches(selected, sampleAgePrediction) && _secondary.HipNormalReferenceMatches(selected, sampleAgePrediction);
        internal bool FootRelativeReferenceMatches(bool enabled) => _primary != null && _secondary != null &&
            _primary.FootRelativeReferenceMatches(enabled) && _secondary.FootRelativeReferenceMatches(enabled);
        internal void SelectFootRelativeReference(bool enabled)
        {
            if (_primary == null || _secondary == null || !NativeBodiesSafeOff || TraceEnabled)
                throw new InvalidOperationException("FOOT_REFERENCE_V04730_REQUIRES_BILATERAL_SAFE_OFF");
            bool previous = _primary.FootRelativeReferenceSelected;
            if (_secondary.FootRelativeReferenceSelected != previous)
                throw new InvalidOperationException("FOOT_REFERENCE_V04730_SELECTOR_PARITY_LOST");
            try
            {
                if (!_primary.SelectFootRelativeReference(enabled) || !_secondary.SelectFootRelativeReference(enabled) ||
                    !FootRelativeReferenceMatches(enabled))
                    throw new InvalidOperationException("FOOT_REFERENCE_V04730_SELECT_READBACK_FAILED");
            }
            catch
            {
                try { _secondary.SelectFootRelativeReference(previous); }
                finally { _primary.SelectFootRelativeReference(previous); }
                throw;
            }
        }
        internal string RestoreFootRelativeReferenceBaseline()
        {
            // Cleanup is independent per side even after a partial selector failure.
            bool primary = false, secondary = false;
            try { primary = _primary == null || _primary.SelectFootRelativeReference(false); }
            finally { secondary = _secondary == null || _secondary.SelectFootRelativeReference(false); }
            if (!primary || !secondary || !FootRelativeReferenceMatches(false))
                throw new InvalidOperationException("FOOT_REFERENCE_V04730_BASELINE_RESTORE_FAILED");
            return "footReferenceBaselineRestored=True";
        }
        internal bool FreeSpaceReady => !DynamicContactSelected && _bilateralEnabled && _primary != null && _secondary != null && _supportSurface.StoppedSafe && !_contactShadowSelfPairOwned &&
            _primary.FreeSpaceSelectorsOff && _secondary.FreeSpaceSelectorsOff &&
            _primary.FreeSpaceNativeReady && _secondary.FreeSpaceNativeReady;
        internal bool DynamicJoinReady => SourceContextReady && FreeSpaceReady &&
            _primary.DynamicJoinFoundationReady && _secondary.DynamicJoinFoundationReady;
        // On-demand only. Existing owner state/readbacks are reused; no extra observer or native sampling loop.
        internal string FreeSpaceBoundarySummary() => "freeSpaceBoundary=V04820" +
            "/supportReleased=" + _supportSurface.StoppedSafe +
            "/footPairReleased=" + !_contactShadowSelfPairOwned +
            "/selectors(primary-secondary)=" + (_primary?.FreeSpaceSelectorIssue ?? FreeSpaceBoundaryIssue.MissingFoundation) + ":" +
            (_secondary?.FreeSpaceSelectorIssue ?? FreeSpaceBoundaryIssue.MissingFoundation) +
            "/predictiveCapture(primary-secondary)=" + (_primary?.TranslationPredictiveTailCaptureArm ?? "<missing>") + ":" +
            (_secondary?.TranslationPredictiveTailCaptureArm ?? "<missing>") + "/loadedSupportCertified=False";
        internal string DynamicParentSummary() => "[DYNAMIC_LEG_PARENT]\n" + (_primary?.DynamicParentSummary() ?? "primary=missing") +
            "\n" + (_secondary?.DynamicParentSummary() ?? "secondary=missing") + "\n" + GetStepContextStatus() +
            "\nLEFT/" + (_primary?.FootRelativeReferenceStatus ?? "<missing>") +
            "\nRIGHT/" + (_secondary?.FootRelativeReferenceStatus ?? "<missing>");

        internal void ActivateDynamicParents(PhysicsStepContext step)
        {
            if (!DynamicJoinReady || !_contextCursor.Fixed.Matches(step) ||
                !_torso.TryGetLegParentCommand(step, out LegParentCommandFrame command) || !command.PhysicalParent.Matches(step))
                throw new InvalidOperationException("BILATERAL_DYNAMIC_PARENT_JOIN_NOT_READY");
            // Preflight both sides and compute all initial values before the first native write.
            LegInitialState primary = _primary.PrepareDynamicJoin(step, command.PhysicalParent);
            LegInitialState secondary = _secondary.PrepareDynamicJoin(step, command.PhysicalParent);
            _bilateralFailClosedLatched = false;
            try
            {
                _primary.ActivateDynamicJoin(step, command.PhysicalParent, primary);
                _secondary.ActivateDynamicJoin(step, command.PhysicalParent, secondary);
                if (!DynamicParentsActive || !FreeSpaceReady) throw new InvalidOperationException("BILATERAL_DYNAMIC_JOIN_READBACK_FAILED");
            }
            catch { try { _secondary.SetDrive(false); } finally { _primary.SetDrive(false); } throw; }
        }
        internal string GetStepContextStatus() => "legStepContext=V04620" +
            "/source(epoch-sequence)=" + _contextCursor.Source.Epoch + "-" + _contextCursor.Source.Sequence +
            "/" + _contextCursor.Fixed.Status() + "/traceSamples=" + _traceContextSamples +
            "/mismatches=" + _traceContextMismatches + "/localFixed(left-right)=" +
            (PrimaryReport?.FixedUpdateSequence ?? 0) + "-" + (SecondaryReport?.FixedUpdateSequence ?? 0) +
            "/parentMatched(left-right)=" + (PrimaryReport != null && PrimaryReport.RuntimeParentMatched) + "-" +
            (SecondaryReport != null && SecondaryReport.RuntimeParentMatched);

        public void SampleSourcePose(SourceStepContext source)
        {
            if (!IsBuilt) return;
            _contextCursor.AcceptSource(source);
            if (!_torso.CompletedSource.Matches(source))
                throw new InvalidOperationException("LEG_PARENT_SOURCE_NOT_CURRENT");
            _primary.SampleSourcePose(source);
            _secondary?.SampleSourcePose(source);
        }

        public void FixedUpdate(PhysicsStepContext step)
        {
            if (!IsBuilt) return;
            _contextCursor.AcceptFixed(step);
            var parent = new LegParentCommandFrame();
            if (AnyDriveEnabled && (!_contextCursor.SourceMatches(step) || !_torso.TryGetLegParentCommand(step, out parent)))
                throw new InvalidOperationException("LEG_PARENT_COMMAND_NOT_CURRENT");

            if (DynamicParentsActive && !DynamicContactSelected && !_supportSurface.StoppedSafe)
                throw new InvalidOperationException("DYNAMIC_FREE_SPACE_SUPPORT_NOT_RELEASED");
            if (DynamicParentsActive && !DynamicContactSelected && _contactShadowSelfPairOwned)
                throw new InvalidOperationException("DYNAMIC_FREE_SPACE_FOOT_PAIR_NOT_RELEASED");
            if (_supportSurface.Active && !_supportSurface.Validate(out string surfaceError))
            {
                FailCloseStanceContactIsolation("V04410_SUPPORT_SCOPE_INVALID:" + surfaceError);
                return;
            }

            // v0.4.7 structural safety: per-foot contact may turn on/off independently, but the bilateral
            // Foot↔Foot self-ignore pair must remain true for the whole stance-contact experiment. Validate
            // it before either child can enable a Foot collider for this physics step.
            if (StanceOnlyContactLifecycleEnabled && !ValidateBilateralFootSelfCollisionIsolation(out string stanceSelfPairError))
            {
                FailCloseStanceContactIsolation("BILATERAL_FOOT_SELF_COLLISION_RUNTIME_READBACK_FAILED:" + (stanceSelfPairError ?? "<none>"));
                return;
            }

            if (DynamicContactSelected && !DynamicContactReady)
                throw new InvalidOperationException("DYNAMIC_CONTACT_CONTRACT_LOST:" + DynamicContactIssue);

            _primary.FixedUpdate(step, parent);
            _secondary?.FixedUpdate(step, parent);
            if (_primary != null && PelvisRunScopes.HasHipCoupling(_primary.DynamicScope) && DriveEnabled && !AnyDriveAborted)
            {
                if (!_primary.CommitBilateralHipPairs(step, _secondary))
                    FailCloseStanceContactIsolation("HIP_BILATERAL_FORCE_COMPLETION_FAILED:" +
                        (_primary.HipAnchorPair.Issue ?? (_secondary?.HipAnchorPair.Issue) ?? "INPUT_OR_CLOCK"));
                else if (!_torso.CompleteHipTranslationReaction(step, _primary.HipAnchorPair, _secondary.HipAnchorPair))
                    FailCloseStanceContactIsolation("HIP_PAIR_RECEIVER_ATTITUDE_COMPLETION_FAILED");
                else if (!_primary.CompleteHipControllerStep(step) || !_secondary.CompleteHipControllerStep(step))
                    FailCloseStanceContactIsolation("HIP_CONTROLLER_CONTINUATION_FAILED");
                else
                {
                    // The receiver has committed the bounded same-step correction.
                    // All actual commands are now known. Diagnostic composition writes no forces
                    // and cannot become a second owner or a readiness/safety prerequisite.
                    _primary.CompleteHipPointResponse(step, _secondary.HipAnchorPair);
                    _secondary.CompleteHipPointResponse(step, _primary.HipAnchorPair);
                }
            }
            if (_traceEnabled)
            {
                _traceContextSamples++;
                if (!PrimaryReport.RuntimeContext.Matches(step) || !PrimaryReport.RuntimeParentMatched ||
                    (_bilateralEnabled && (SecondaryReport == null || !SecondaryReport.RuntimeContext.Matches(step) || !SecondaryReport.RuntimeParentMatched)))
                    _traceContextMismatches++;
            }

            // Observe first so an asymmetric abort/drive transition is preserved in the read-only parity evidence.
            if (_traceEnabled && _bilateralEnabled && _secondary != null)
            {
                _diagnostics.Observe(_report, _primary.Report, _secondary.Report);
                // v0.3.12 minimal hard-safety context ring retained after the v0.3.10 timing study closed.
                _kneeSafety.ObserveFixedStep(_kneeSafetyReport, _primary, _secondary);
                bool primaryIsLeft = string.Equals(PrimarySide, "LEFT", StringComparison.OrdinalIgnoreCase);
                _hipPelvisLoadPath.Observe(DriveEnabled, _torso,
                    primaryIsLeft ? PrimaryReport : SecondaryReport, primaryIsLeft ? SecondaryReport : PrimaryReport,
                    step, primaryIsLeft ? _primary.DynamicHipCommand : _secondary.DynamicHipCommand,
                    primaryIsLeft ? _secondary.DynamicHipCommand : _primary.DynamicHipCommand);
            }
            else
            {
                _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
            }

            // v0.3.0 bilateral-only lifecycle safety: one per-side fail-fast must never leave the opposite side
            // dynamically active on subsequent physics steps. This does not alter controller math; it only propagates
            // the existing per-side shutdown after both chains have completed the current coordinator FixedUpdate.
            if (_bilateralEnabled && _secondary != null)
            {
                bool driveMismatch = _primary.DriveEnabled != _secondary.DriveEnabled;
                bool anyAbort = _primary.Report.DriveAborted || _secondary.Report.DriveAborted;
                if (driveMismatch || anyAbort)
                    FailCloseBothSides(driveMismatch, anyAbort);
            }
        }

        public string SetDrive(bool enabled)
        {
            // OFF is a cleanup operation and must remain available even if a historical/exceptional half-built
            // coordinator has lost one child. ON still requires the complete bilateral foundation.
            if (!enabled)
            {
                _hipPelvisLoadPath.EndTrace();
                if (_primary == null && _secondary == null)
                    return "ARL leg drive | enabled=False | already-cleared=True";
                string secondaryOff = "secondary=disabled", primaryOff = "primary=missing";
                Exception secondaryFailure = null, primaryFailure = null;
                try { if (_secondary != null) secondaryOff = _secondary.SetDrive(false); }
                catch (Exception e) { secondaryFailure = e; }
                try { if (_primary != null) primaryOff = _primary.SetDrive(false); }
                catch (Exception e) { primaryFailure = e; }
                // Attempt BOTH native owners before formatting or reporting an exception. A failed OFF
                // must propagate to session restore/full rollback, never be returned as a successful stop.
                if (secondaryFailure != null || primaryFailure != null)
                    throw new InvalidOperationException("BILATERAL_DRIVE_OFF_FAILED:secondary=" +
                        (secondaryFailure?.GetType().Name ?? "<none>") + ":primary=" + (primaryFailure?.GetType().Name ?? "<none>"));
                _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
                return "ARL bilateral leg drive | enabled=False\nprimary: " + primaryOff + (_bilateralEnabled ? "\nsecondary: " + secondaryOff : string.Empty);
            }

            if (!IsBuilt)
                return "ARL leg drive：bilateral foundation尚未创建。";

            // v0.3.0 only validates the certified frozen baseline. Do not allow a bilateral run to start if either
            // side skipped the existing post-PCA stack setup; this is a production-contract preflight, not a new gate.
            if (!_primary.FrozenProductionStackReady)
                return "ARL bilateral leg drive：primary frozen production stack未READY；请先 arl_hip_stack(on)。";
            if (_bilateralEnabled && !_secondary.FrozenProductionStackReady)
                return "ARL bilateral leg drive：secondary frozen production stack未READY；请先 arl_hip_stack(on)。";

            _bilateralFailClosedLatched = false;
            string primaryOn = _primary.SetDrive(true);
            if (!_primary.DriveEnabled)
                return "ARL bilateral leg drive：primary启动失败。\n" + primaryOn;

            if (_bilateralEnabled)
            {
                string secondaryOn = _secondary.SetDrive(true);
                if (!_secondary.DriveEnabled)
                {
                    _primary.SetDrive(false);
                    _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
                    return "ARL bilateral leg drive：secondary启动失败；primary已立即回退为OFF。\n" + secondaryOn;
                }
                _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
                return "ARL bilateral leg drive | enabled=True\nprimary: " + primaryOn + "\nsecondary: " + secondaryOn;
            }

            _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
            return primaryOn;
        }

        public string SetHipExperimentStack(bool enabled)
        {
            if (!IsBuilt) return "ARL Hip experiment stack：bilateral foundation尚未创建。";

            if (!enabled)
            {
                string secondaryOff = _secondary != null ? _secondary.SetHipExperimentStack(false) : "secondary=disabled";
                string primaryOff = _primary.SetHipExperimentStack(false);
                return Combine("ARL bilateral frozen stack | enabled=False", primaryOff, secondaryOff);
            }

            // Bilateral v0.3.0 must not silently leave one side on a different frozen stack.
            // Bring primary up first, verify the existing post-PCA readiness contract, then do secondary.
            string primaryOn = _primary.SetHipExperimentStack(true);
            if (!_primary.FrozenProductionStackReady)
                return "ARL bilateral frozen stack：primary未达到 frozen production stack ready。\n" + primaryOn;

            if (_bilateralEnabled)
            {
                string secondaryOn = _secondary.SetHipExperimentStack(true);
                if (!_secondary.FrozenProductionStackReady)
                {
                    _primary.SetHipExperimentStack(false);
                    return "ARL bilateral frozen stack：secondary配置失败；primary已回退为OFF，避免半栈状态。\n" + secondaryOn;
                }
                return Combine("ARL bilateral frozen stack | enabled=True", primaryOn, secondaryOn);
            }

            return primaryOn;
        }

        public string GetHipExperimentStackStatus() => CombineStatus("ARL bilateral frozen stack", c => c.GetHipExperimentStackStatus());

        public string GetSingleLegFreezeStatus() => CombineStatus("ARL bilateral per-leg freeze", c => c.GetSingleLegFreezeStatus());

        public string SetKneeAxisCalibration(bool enabled)
        {
            if (!IsBuilt) return "ARL knee axis calibration：bilateral foundation尚未创建。";
            if (enabled)
            {
                string p = _primary.SetKneeAxisCalibration(true);
                if (!_primary.KneeAxisCalibrationCollecting)
                    return "ARL bilateral knee axis calibration：primary启动失败。\n" + p;
                if (_bilateralEnabled)
                {
                    string s = _secondary.SetKneeAxisCalibration(true);
                    if (!_secondary.KneeAxisCalibrationCollecting)
                    {
                        _primary.SetKneeAxisCalibration(false);
                        return "ARL bilateral knee axis calibration：secondary启动失败；primary收集已停止。\n" + s;
                    }
                    return Combine("ARL bilateral knee axis calibration | collecting=True", p, s);
                }
                return p;
            }

            string secondaryStop = _secondary != null ? _secondary.SetKneeAxisCalibration(false) : "secondary=disabled";
            string primaryStop = _primary.SetKneeAxisCalibration(false);
            return Combine("ARL bilateral knee axis calibration | collecting=False", primaryStop, secondaryStop);
        }

        public string ApplyCalibratedKneeAxis()
        {
            if (!IsBuilt) return "ARL knee axis apply：bilateral foundation尚未创建。";

            // Apply only when both independently collected PCA candidates have passed the existing readiness gate.
            // This keeps the bilateral foundation symmetric without inventing any new axis-quality threshold.
            if (!_primary.KneeAxisCalibrationReady)
                return "ARL bilateral knee axis apply：primary candidate尚未 READY；未修改任一侧。";
            if (_bilateralEnabled && !_secondary.KneeAxisCalibrationReady)
                return "ARL bilateral knee axis apply：secondary candidate尚未 READY；未修改任一侧。";

            string primaryApply = _primary.ApplyCalibratedKneeAxis();
            if (!_primary.KneeAxisCalibrationApplied)
                return "ARL bilateral knee axis apply：primary应用失败；secondary未修改。\n" + primaryApply;

            if (_bilateralEnabled)
            {
                string secondaryApply = _secondary.ApplyCalibratedKneeAxis();
                if (!_secondary.KneeAxisCalibrationApplied)
                {
                    // Applying an axis is configuration-only (drive/trace are off). If the second side fails, restore
                    // both to the initial CharacterJoint-right basis rather than continue with asymmetric calibration.
                    string secondaryRollback = _secondary.ResetKneeAxisToInitial();
                    string primaryRollback = _primary.ResetKneeAxisToInitial();
                    return "ARL bilateral knee axis apply：secondary应用失败；两侧均已回退到 initial axis。" +
                           "\nprimaryApply: " + primaryApply +
                           "\nsecondaryApply: " + secondaryApply +
                           "\nprimaryRollback: " + primaryRollback +
                           "\nsecondaryRollback: " + secondaryRollback;
                }
                return Combine("ARL bilateral knee axis apply | success=True", primaryApply, secondaryApply);
            }

            return primaryApply;
        }

        public string ResetKneeAxisToInitial()
        {
            if (!IsBuilt) return "ARL knee axis reset：bilateral foundation尚未创建。";
            string p = _primary.ResetKneeAxisToInitial();
            string s = _secondary != null ? _secondary.ResetKneeAxisToInitial() : "secondary=disabled";
            return Combine("ARL bilateral knee axis reset", p, s);
        }

        public string GetKneeAxisCalibrationStatus() => CombineStatus("ARL bilateral knee axis calibration", c => c.GetKneeAxisCalibrationStatus());

        public string SetGravity(bool enabled)
        {
            if (enabled && !IsBuilt) return "ARL leg gravity：bilateral foundation尚未创建。";
            if (enabled && AnyDriveEnabled) return "ARL bilateral leg gravity：拒绝开启。frozen leg drive 固定 Gravity=OFF；请先停止 drive。";
            if (_primary == null && _secondary == null) return "ARL bilateral leg gravity | enabled=" + enabled + " | already-cleared=True";
            string p = _primary != null ? _primary.SetGravity(enabled) : "primary=missing";
            string s = _secondary != null ? _secondary.SetGravity(enabled) : "secondary=disabled";
            return Combine("ARL bilateral leg gravity", p, s);
        }

        public string SetColliders(bool enabled)
        {
            if (enabled && !IsBuilt) return "ARL leg colliders：bilateral foundation尚未创建。";
            if (enabled && AnyDriveEnabled) return "ARL bilateral leg colliders：拒绝开启。frozen leg drive 固定 Colliders=OFF；请先停止 drive。";
            if (_primary == null && _secondary == null) return "ARL bilateral leg colliders | enabled=" + enabled + " | already-cleared=True";
            string p = _primary != null ? _primary.SetColliders(enabled) : "primary=missing";
            string s = _secondary != null ? _secondary.SetColliders(enabled) : "secondary=disabled";
            return Combine("ARL bilateral leg colliders", p, s);
        }

        public string SetVisual(bool enabled)
        {
            if (!IsBuilt) return "ARL leg visual：bilateral foundation尚未创建。";
            return Combine("ARL bilateral leg visual", _primary.SetVisual(enabled), _secondary != null ? _secondary.SetVisual(enabled) : "secondary=disabled");
        }

        public string SetTrace(bool enabled)
        {
            // Like drive OFF, trace OFF is teardown and must work for a stale child even if full IsBuilt is false.
            if (enabled && !IsBuilt) return "ARL leg trace：bilateral foundation尚未创建。";

            bool pTrace = _primary != null && _primary.TraceEnabled;
            bool sTrace = _secondary != null && _secondary.TraceEnabled;
            bool fullyActive = pTrace && (!_bilateralEnabled || sTrace);
            bool anyActive = pTrace || (_bilateralEnabled && sTrace) || _traceEnabled;

            if (enabled && fullyActive && _traceEnabled)
                return "ARL bilateral trace | enabled=True | alreadyInRequestedState=True | action=NO_OP";
            if (!enabled && !anyActive)
            {
                _traceEnabled = false;
                _report.TraceEnabled = false;
                _hipPelvisLoadPath.EndTrace();
                return "ARL bilateral trace | enabled=False | alreadyInRequestedState=True | action=NO_OP";
            }

            // Normalize an impossible half-open child state before restarting. This may only be expected after an
            // exception or an older build; normal v0.3.28 activation is atomic. Finalize only the child that was
            // actually tracing so an inactive side does not manufacture an empty certification window.
            string repair = null;
            if (enabled && (pTrace || sTrace || _traceEnabled) && !(fullyActive && _traceEnabled))
            {
                string pRepair = pTrace ? _primary.SetTrace(false) : "primary=already-off";
                string sRepair = (_secondary != null && sTrace) ? _secondary.SetTrace(false) : "secondary=already-off";
                _traceEnabled = false;
                _report.TraceEnabled = false;
                repair = "halfOpenTraceRepaired=True | " + pRepair + " | " + sRepair;
                _hipPelvisLoadPath.EndTrace();
                Plugin.Logger?.LogWarning("[ARL][BILATERAL_TRACE_STATE_REPAIR] " + repair);
                pTrace = false;
                sTrace = false;
            }

            if (enabled)
            {
                // Preflight all prerequisites before changing either side. This keeps trace activation atomic even
                // if this coordinator is called outside ActiveRagdollLabRuntime in a future extension.
                if (!_primary.DriveEnabled || (_bilateralEnabled && !_secondary.DriveEnabled))
                    return "ARL bilateral trace：拒绝启动。两侧 frozen leg drive 必须先处于 ON。";
                if (_primary.KneeAxisCalibrationCollecting || (_bilateralEnabled && _secondary.KneeAxisCalibrationCollecting))
                    return "ARL bilateral trace：拒绝启动。至少一侧 Knee axis calibration 正在收集。";

                _supportSurface.ResetExposure();
                string p = _primary.SetTrace(true);
                if (!_primary.TraceEnabled)
                    return "ARL bilateral trace：primary 启动失败；未修改 secondary。\n" + p;

                string s = _secondary != null ? _secondary.SetTrace(true) : "secondary=disabled";
                if (_bilateralEnabled && !_secondary.TraceEnabled)
                {
                    string rollback = _primary.SetTrace(false);
                    return "ARL bilateral trace：secondary 启动失败；primary trace 已回滚。\nsecondary: " + s + "\nprimaryRollback: " + rollback;
                }

                _traceEnabled = true;
                _traceContextSamples = _traceContextMismatches = 0;
                _diagnostics.Reset(_report, _bilateralEnabled, _distinctSourceChains, PrimarySide, SecondarySide);
                _kneeSafety.BeginTrace(_kneeSafetyReport);
                bool dynamicHipAudit = DynamicParentsActive && _bilateralEnabled && _secondary != null &&
                    PelvisRunScopes.HasUpperWeight(_primary.DynamicScope) && PelvisRunScopes.HasUpperWeight(_secondary.DynamicScope) && !PelvisRunScopes.HasHipCoupling(_primary.DynamicScope);
                _hipPelvisLoadPath.BeginTrace(_bilateralEnabled && UpperSelfWeightArmsMatch &&
                    UpperSelfWeightArm == UpperSelfWeightSupportNormalLoadTransferIsolation.ArmB &&
                    (!DynamicParentsActive || dynamicHipAudit), _torso, dynamicHipAudit);
                _report.Built = IsBuilt;
                _report.TraceEnabled = true;
                _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
                return Combine("ARL bilateral trace | enabled=True" + (repair != null ? " | " + repair : string.Empty), p, s);
            }

            bool coordinatorSessionWasActive = _traceEnabled;
            _hipPelvisLoadPath.EndTrace();
            string secondaryStop = (_secondary != null && _secondary.TraceEnabled) ? _secondary.SetTrace(false) : "secondary=already-off";
            string primaryStop = (_primary != null && _primary.TraceEnabled) ? _primary.SetTrace(false) : "primary=already-off";
            _traceEnabled = false;
            _report.TraceEnabled = false;

            string bilateral = "ARL bilateral foundation | enabled=" + _bilateralEnabled;
            if (coordinatorSessionWasActive)
            {
                string kneeSafety = _kneeSafety.Finalize(_kneeSafetyReport);
                Plugin.Logger?.LogWarning("[ARL][KNEE_SAFETY_CONTEXT] " + kneeSafety);
                Plugin.Logger?.LogWarning("[ARL][KNEE_SAFETY_CONTEXT_KEY] " + (_kneeSafetyReport.Key ?? "<none>"));
                bilateral = _bilateralEnabled
                    ? _diagnostics.Finalize(_report, PrimaryReport, SecondaryReport, IsBuilt, _distinctSourceChains)
                    : "ARL bilateral foundation | enabled=False";
                if (_bilateralEnabled)
                {
                    Plugin.Logger?.LogWarning("[ARL][BILATERAL_FOUNDATION] " + bilateral);
                    Plugin.Logger?.LogWarning("[ARL][BILATERAL_FOUNDATION_KEY] " + (_report.Key ?? "<none>"));
                }
            }
            else
            {
                // Child-only stale trace repair: do not fabricate a new coordinator/knee-safety session.
                _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
            }
            return Combine("ARL bilateral trace | enabled=False | " + bilateral, primaryStop, secondaryStop);
        }

        public string GetKneeSafetyContextStatus()
        {
            return _kneeSafety.GetStatus(_kneeSafetyReport);
        }

        public string GetFootSoleRequalificationStatus()
        {
            if (!IsBuilt) return "ARL bilateral Foot sole requalification | built=False";
            string p = _primary != null ? _primary.GetFootSoleRequalificationStatus() : "primary=missing";
            string sResult = _secondary != null ? _secondary.GetFootSoleRequalificationStatus() : "secondary=disabled";
            return Combine("ARL bilateral Foot sole requalification", p, sResult);
        }

        public string QuiesceStanceOnlyContactPreserveTrace()
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral stance-only contact quiesce | built=False";

            bool pOk = _primary.QuiesceStanceContactLifecyclePreserveTrace(out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.QuiesceStanceContactLifecyclePreserveTrace(out sError);
            string secondaryError = _secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"));
            return "ARL bilateral stance-only contact quiesce | success=" + (pOk && sOk) +
                   " | FootCollider=OFF | tracePreserved=True | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                   " | secondary=" + secondaryError;
        }

        public string SetStanceContactFrictionIsolation(bool zeroFriction)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral stance contact friction isolation | built=False";

            string arm = zeroFriction ? FootStanceContactFrictionIsolation.ArmB : FootStanceContactFrictionIsolation.ArmA;
            bool pOk = _primary.SetStanceContactFrictionIsolationArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetStanceContactFrictionIsolationArm(arm, out sError);
            bool match = StanceContactFrictionIsolationArmsMatch;
            if (!pOk || !sOk || !match || !StanceContactFrictionIsolationFinite)
            {
                // Fail closed to the original/default material on both feet. This selector never needs
                // contact to remain active if its material readback is inconsistent.
                _primary.SetStanceContactFrictionIsolationArm(FootStanceContactFrictionIsolation.ArmA, out _);
                _secondary?.SetStanceContactFrictionIsolationArm(FootStanceContactFrictionIsolation.ArmA, out _);
                return "ARL bilateral stance contact friction isolation | success=False | requested=" + arm +
                       " | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                       " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"))) +
                       " | armsMatch=" + match;
            }

            return "ARL bilateral stance contact friction isolation | success=True | arm=" + arm +
                   " | zeroFriction=" + zeroFriction + " | armsMatch=True";
        }

        public string SetTranslationPredictiveTailCapture(bool enabled)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral Foot translation predictive tail capture | built=False";

            string arm = enabled ? FootAnkleTranslationPredictiveTailCaptureIsolation.ArmB : FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA;
            bool pOk = _primary.SetTranslationPredictiveTailCaptureArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetTranslationPredictiveTailCaptureArm(arm, out sError);
            bool match = TranslationPredictiveTailCaptureArmsMatch;
            if (!pOk || !sOk || !match || !TranslationPredictiveTailCaptureFinite)
            {
                _primary.SetTranslationPredictiveTailCaptureArm(FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA, out _);
                _secondary?.SetTranslationPredictiveTailCaptureArm(FootAnkleTranslationPredictiveTailCaptureIsolation.ArmA, out _);
                return "ARL bilateral Foot translation predictive tail capture | success=False | requested=" + arm +
                       " | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                       " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"))) +
                       " | armsMatch=" + match;
            }

            return "ARL bilateral Foot translation predictive tail capture | success=True | arm=" + arm +
                   " | enabled=" + enabled + " | armsMatch=True | baseMaxA=" +
                   FootAnkleTranslationCaptureAuthorityIsolation.CaptureMaxAcceleration + " | predictiveMaxA=" +
                   FootAnkleTranslationPredictiveTailCaptureIsolation.PredictiveReserveMaxAcceleration +
                   " | gateMm=" + (FootAnkleConstraintTopologyIsolation.AnchorValidityMeters * 1000f);
        }

        public string SetStanceLowerGroundAnchorAuthority(bool enabled)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral stance Lower ground-anchor authority | built=False";

            string arm = enabled ? FootStanceLowerGroundAnchorAuthority.ArmB : FootStanceLowerGroundAnchorAuthority.ArmA;
            bool pOk = _primary.SetStanceLowerGroundAuthorityArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetStanceLowerGroundAuthorityArm(arm, out sError);
            bool match = StanceLowerGroundAuthorityArmsMatch;
            if (!pOk || !sOk || !match || !StanceLowerGroundAuthorityFinite)
            {
                _primary.SetStanceLowerGroundAuthorityArm(FootStanceLowerGroundAnchorAuthority.ArmA, out _);
                _secondary?.SetStanceLowerGroundAuthorityArm(FootStanceLowerGroundAnchorAuthority.ArmA, out _);
                return "ARL bilateral stance Lower ground-anchor authority | success=False | requested=" + arm +
                       " | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                       " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"))) +
                       " | armsMatch=" + match;
            }

            return "ARL bilateral stance Lower ground-anchor authority | success=True | arm=" + arm +
                   " | enabled=" + enabled + " | armsMatch=True | Kp/Kd/MaxA=" +
                   FootStanceLowerGroundAnchorAuthority.Kp + "/" + FootStanceLowerGroundAnchorAuthority.Kd + "/" + FootStanceLowerGroundAnchorAuthority.MaxAcceleration;
        }

        public string SetStanceFootNativeGravityLoad(bool enabled)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral stance Foot native-gravity support load | built=False";

            string arm = enabled ? FootStanceNativeGravitySupportLoadIsolation.ArmB : FootStanceNativeGravitySupportLoadIsolation.ArmA;
            bool pOk = _primary.SetStanceFootNativeGravityArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetStanceFootNativeGravityArm(arm, out sError);
            bool match = StanceFootNativeGravityArmsMatch;
            if (!pOk || !sOk || !match || !StanceFootNativeGravityFinite)
            {
                _primary.SetStanceFootNativeGravityArm(FootStanceNativeGravitySupportLoadIsolation.ArmA, out _);
                _secondary?.SetStanceFootNativeGravityArm(FootStanceNativeGravitySupportLoadIsolation.ArmA, out _);
                return "ARL bilateral stance Foot native-gravity support load | success=False | requested=" + arm +
                       " | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                       " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"))) +
                       " | armsMatch=" + match;
            }

            return "ARL bilateral stance Foot native-gravity support load | success=True | arm=" + arm +
                   " | enabled=" + enabled + " | armsMatch=True | scope=FOOT_ONLY_EXACT_AUTHORIZED_STANCE | globalGravity=" + UnityEngine.Physics.gravity;
        }

        public string SetStanceLowerNativeGravityLoad(bool enabled)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral stance Lower native-gravity support load | built=False";

            string arm = enabled ? LowerStanceNativeGravitySupportLoadIsolation.ArmB : LowerStanceNativeGravitySupportLoadIsolation.ArmA;
            bool pOk = _primary.SetStanceLowerNativeGravityArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetStanceLowerNativeGravityArm(arm, out sError);
            bool match = StanceLowerNativeGravityArmsMatch;
            if (!pOk || !sOk || !match || !StanceLowerNativeGravityFinite)
            {
                _primary.SetStanceLowerNativeGravityArm(LowerStanceNativeGravitySupportLoadIsolation.ArmA, out _);
                _secondary?.SetStanceLowerNativeGravityArm(LowerStanceNativeGravitySupportLoadIsolation.ArmA, out _);
                return "ARL bilateral stance Lower native-gravity support load | success=False | requested=" + arm +
                       " | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                       " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"))) +
                       " | armsMatch=" + match;
            }

            return "ARL bilateral stance Lower native-gravity support load | success=True | arm=" + arm +
                   " | enabled=" + enabled + " | armsMatch=True | scope=LOWER_ONLY_EXACT_AUTHORIZED_STANCE | globalGravity=" + UnityEngine.Physics.gravity;
        }

        public string SetStanceFootLowerReciprocalLoadTransfer(bool enabled)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral Foot<->Lower support-normal reciprocal load transfer | built=False";

            string arm = enabled ? FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmB : FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmA;
            bool pOk = _primary.SetStanceFootLowerReciprocalArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetStanceFootLowerReciprocalArm(arm, out sError);
            bool match = StanceFootLowerReciprocalArmsMatch;
            if (!pOk || !sOk || !match || !StanceFootLowerReciprocalFinite)
            {
                _primary.SetStanceFootLowerReciprocalArm(FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmA, out _);
                _secondary?.SetStanceFootLowerReciprocalArm(FootLowerSupportNormalReciprocalLoadTransferIsolation.ArmA, out _);
                return "ARL bilateral Foot<->Lower support-normal reciprocal load transfer | success=False | requested=" + arm +
                       " | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                       " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"))) +
                       " | armsMatch=" + match;
            }

            return "ARL bilateral Foot<->Lower support-normal reciprocal load transfer | success=True | arm=" + arm +
                   " | enabled=" + enabled + " | armsMatch=True | scope=EXACT_AUTHORIZED_STANCE_SUPPORT_NORMAL_ONLY | forceLaw=EQUAL_OPPOSITE_OF_ACTUAL_FOOT_TETHER_FORCE";
        }

        internal bool StancePlanarDampingEnabled => _primary != null && _primary.StancePlanarDampingEnabled &&
            (!_bilateralEnabled || (_secondary != null && _secondary.StancePlanarDampingEnabled));
        internal bool StancePlanarDampingOff => (_primary == null || !_primary.StancePlanarDampingEnabled) &&
            (_secondary == null || !_secondary.StancePlanarDampingEnabled);
        public string SetStancePlanarDamping(bool enabled)
        {
            if (_primary == null || !_primary.IsBuilt || (_bilateralEnabled && (_secondary == null || !_secondary.IsBuilt)))
                return "ARL bilateral stance planar damping | success=False | reason=FOUNDATION_NOT_BUILT";
            bool pOk = _primary.SetStancePlanarDamping(enabled);
            bool sOk = _secondary == null || _secondary.SetStancePlanarDamping(enabled);
            bool matches = enabled ? StancePlanarDampingEnabled : StancePlanarDampingOff;
            if (!pOk || !sOk || !matches)
            {
                _primary.SetStancePlanarDamping(false);
                _secondary?.SetStancePlanarDamping(false);
                return "ARL bilateral stance planar damping | success=False | rollbackOff=" + StancePlanarDampingOff;
            }
            return "ARL bilateral stance planar damping | success=True | arm=" +
                (enabled ? FootStancePlanarRotationalDampingIsolation.ArmB : FootStancePlanarRotationalDampingIsolation.ArmA);
        }

        public string SetFootPositionSolverArm(string arm)
        {
            if (_traceEnabled || _primary == null || !_primary.IsBuilt || (_bilateralEnabled && (_secondary == null || !_secondary.IsBuilt)))
                return "ARL bilateral Foot position solver | success=False | reason=TRACE_ACTIVE_OR_FOUNDATION_NOT_BUILT";
            bool pOk = _primary.SetFootPositionSolverArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetFootPositionSolverArm(arm, out sError);
            bool expectedActive = arm == FootSolverIterationBaselineContract.PositionArmB;
            if (!pOk || !sOk || !FootPositionSolverArmsMatch || !FootVelocitySolverFinite ||
                !string.Equals(FootPositionSolverArm, arm, StringComparison.Ordinal) || FootPositionSolverEnabled != expectedActive)
            {
                string restored = RestoreFootSolverIterationBaseline();
                return "ARL bilateral Foot position solver | success=False | primary=" + pError + " | secondary=" + sError +
                    " | rollbackBaseline=" + FootVelocitySolverBaselineRestored + " | restore={" + restored + "}";
            }
            return "ARL bilateral Foot position solver | success=True | arm=" + arm + " | velocityIterations=CAPTURED_UNCHANGED" +
                " | left={" + PrimaryFootReport.FootPositionSolverSummary + "} | right={" +
                (SecondaryFootReport != null ? SecondaryFootReport.FootPositionSolverSummary : "<disabled>") + "}";
        }

        public string RestoreFootSolverIterationBaseline()
        {
            if (_primary == null || !_primary.IsBuilt || (_bilateralEnabled && (_secondary == null || !_secondary.IsBuilt)))
                return "ARL bilateral Foot velocity solver | success=False | reason=FOUNDATION_NOT_BUILT";
            const string arm = FootSolverIterationBaselineContract.ArmA;
            bool pOk = _primary.RestoreFootSolverIterationBaseline(out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.RestoreFootSolverIterationBaseline(out sError);
            if (!pOk || !sOk || !FootVelocitySolverArmsMatch || !FootVelocitySolverFinite)
            {
                bool pRestored = _primary.RestoreFootSolverIterationBaseline(out string pRestoreError);
                string sRestoreError = null;
                bool sRestored = _secondary == null || _secondary.RestoreFootSolverIterationBaseline(out sRestoreError);
                return "ARL bilateral Foot velocity solver | success=False | primary=" + pError + " | secondary=" + sError +
                    " | rollbackBaseline=" + (pRestored && sRestored && FootVelocitySolverBaselineRestored) +
                    " | restoreErrors=" + pRestoreError + "/" + sRestoreError;
            }
            return "ARL bilateral Foot velocity solver | success=True | arm=" + arm + " | positionIterations=CAPTURED_UNCHANGED" +
                " | left={" + PrimaryFootReport.FootVelocitySolverSummary + "} | right={" + (SecondaryFootReport != null ? SecondaryFootReport.FootVelocitySolverSummary : "<disabled>") + "}";
        }

        public string SetStanceSupportAngularStabilization(bool enabled)
        {
            if (_primary == null || !_primary.IsBuilt || (_bilateralEnabled && (_secondary == null || !_secondary.IsBuilt)))
                return "ARL bilateral support angular | success=False | reason=FOUNDATION_NOT_BUILT";
            string arm = enabled ? FootStanceSupportNormalAngularStabilization.ArmB : FootStanceSupportNormalAngularStabilization.ArmA;
            bool pOk = _primary.SetStanceSupportAngularArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetStanceSupportAngularArm(arm, out sError);
            if (!pOk || !sOk || !StanceSupportAngularArmsMatch || !StanceSupportAngularFinite)
            {
                _primary.SetStanceSupportAngularArm(FootStanceSupportNormalAngularStabilization.ArmA, out _);
                _secondary?.SetStanceSupportAngularArm(FootStanceSupportNormalAngularStabilization.ArmA, out _);
                return "ARL bilateral support angular | success=False | primary=" + pError + " | secondary=" + sError;
            }
            return "ARL bilateral support angular | success=True | arm=" + arm + " | enabled=" + StanceSupportAngularEnabled +
                   " | finite=" + StanceSupportAngularFinite + " | ratePerSec=30 | maxTorqueNm=1 | yawTorque=ZERO";
        }

        public string SetFootCollisionDetectionModeArm(string arm)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral Foot collision mode | built=False";

            if (!FootCollisionDetectionModeIsolation.TryGetMode(arm, out _))
                return "ARL bilateral Foot collision mode | success=False | reason=UNKNOWN_ARM:" + (arm ?? "<null>");
            bool pOk = _primary.SetFootCollisionDetectionModeArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetFootCollisionDetectionModeArm(arm, out sError);
            bool match = FootCollisionDetectionModeArmsMatch;
            if (!pOk || !sOk || !match || !FootCollisionDetectionModeFinite)
            {
                _primary.SetFootCollisionDetectionModeArm(FootCollisionDetectionModeIsolation.IdleArm, out _);
                _secondary?.SetFootCollisionDetectionModeArm(FootCollisionDetectionModeIsolation.IdleArm, out _);
                return "ARL bilateral Foot collision mode | success=False | arm=" + arm +
                       " | primary=" + (pError ?? "<none>") + " | secondary=" + (sError ?? "<none>") +
                       " | match=" + match + " | finite=" + FootCollisionDetectionModeFinite;
            }
            return "ARL bilateral Foot collision mode | success=True | arm=" + FootCollisionDetectionModeArm +
                   " | readback=" + PrimaryFootReport?.FootCollisionDetectionModeReadback +
                   " | finite=" + FootCollisionDetectionModeFinite;
        }

        public string SetFootContactOffsetArm(string arm)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral Foot contact offset | built=False";
            if (!FootContactOffsetIsolation.IsExperimentArm(arm) && !string.Equals(arm, FootContactOffsetIsolation.IdleArm, StringComparison.Ordinal))
                return "ARL bilateral Foot contact offset | success=False | reason=UNKNOWN_ARM:" + (arm ?? "<null>");
            bool pOk = _primary.SetFootContactOffsetArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetFootContactOffsetArm(arm, out sError);
            if (!pOk || !sOk)
            {
                // Cache both failed states before restoring. The report belongs to the live owner and
                // successful restoration must not erase the requested/actual values from this attempt.
                string failedPrimary = PrimaryFootReport?.FootContactOffsetSummary ?? "<none>";
                string failedSecondary = SecondaryFootReport?.FootContactOffsetSummary ?? "<none>";
                // Restore both captured values even when just one setter/readback failed.
                bool pRestore = _primary.SetFootContactOffsetArm(FootContactOffsetIsolation.IdleArm, out string pRestoreError);
                string sRestoreError = null;
                bool sRestore = _secondary == null || _secondary.SetFootContactOffsetArm(FootContactOffsetIsolation.IdleArm, out sRestoreError);
                return "ARL bilateral Foot contact offset | success=False | requestedArm=" + arm + " | primary=" + (pError ?? "<none>") +
                    " | secondary=" + (sError ?? "<none>") + " | rollback=" + pRestore + "-" + sRestore +
                    " | rollbackReason(left-right)=" + (pRestoreError ?? "<none>") + ":" + (sRestoreError ?? "<none>") +
                    " | failedBeforeRestore(left-right)={" + failedPrimary + "}:{" + failedSecondary + "}" +
                    " | restored(left-right)={" + (PrimaryFootReport?.FootContactOffsetSummary ?? "<none>") +
                    "}:{" + (SecondaryFootReport?.FootContactOffsetSummary ?? "<none>") + "}";
            }
            return "ARL bilateral Foot contact offset | success=True | arm=" + arm +
                " | left={" + (PrimaryFootReport?.FootContactOffsetSummary ?? "<none>") +
                "} | right={" + (SecondaryFootReport?.FootContactOffsetSummary ?? "<none>") + "}";
        }

        public string SetStanceLowerSelfWeightSupportTransfer(bool enabled)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral Lower self-weight support-normal load transfer | built=False";

            string arm = enabled ? LowerSelfWeightSupportNormalLoadTransferIsolation.ArmB : LowerSelfWeightSupportNormalLoadTransferIsolation.ArmA;
            bool pOk = _primary.SetStanceLowerSelfWeightTransferArm(arm, out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.SetStanceLowerSelfWeightTransferArm(arm, out sError);
            bool match = StanceLowerSelfWeightTransferArmsMatch;
            if (!pOk || !sOk || !match || !StanceLowerSelfWeightTransferFinite)
            {
                _primary.SetStanceLowerSelfWeightTransferArm(LowerSelfWeightSupportNormalLoadTransferIsolation.ArmA, out _);
                _secondary?.SetStanceLowerSelfWeightTransferArm(LowerSelfWeightSupportNormalLoadTransferIsolation.ArmA, out _);
                return "ARL bilateral Lower self-weight support-normal load transfer | success=False | requested=" + arm +
                       " | primary=" + (pOk ? "<none>" : (pError ?? "<none>")) +
                       " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "<none>" : (sError ?? "<none>"))) +
                       " | armsMatch=" + match;
            }

            return "ARL bilateral Lower self-weight support-normal load transfer | success=True | arm=" + arm +
                   " | enabled=" + enabled + " | armsMatch=True | scope=EXACT_AUTHORIZED_STANCE_SUPPORT_NORMAL_ONLY" +
                   " | forceLaw=Fload=mLower*max(0,-dot(g,n));Lower+=Fload;Foot-=Fload;ForceMode.Force";
        }

        public string QuiesceStanceLowerNativeGravityPreserveTrace()
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral stance Lower native-gravity quiesce | built=False";
            bool pOk = _primary.QuiesceStanceLowerNativeGravityPreserveTrace(out string pError);
            string sError = null;
            bool sOk = _secondary == null || _secondary.QuiesceStanceLowerNativeGravityPreserveTrace(out sError);
            return "ARL bilateral stance Lower native-gravity quiesce | success=" + (pOk && sOk) +
                   " | primary=" + (pOk ? "OK" : (pError ?? "UNKNOWN")) +
                   " | secondary=" + (_secondary == null ? "<disabled>" : (sOk ? "OK" : (sError ?? "UNKNOWN")));
        }

        public string SetStanceOnlyContactQualification(bool enabled)
        {
            if (!IsBuilt || _primary == null || (_bilateralEnabled && _secondary == null))
                return "ARL bilateral stance-only contact | built=False";

            // Always start from a fully contact-off state. The coordinator-owned Foot↔Foot pair is then
            // established once for the whole v0.4.7 run so per-foot shadow preparation can treat it as
            // pre-existing isolation and never accidentally own/restore the pair on individual strides.
            DisableFootContactShadowPreserveTraceInternal();

            if (!enabled)
            {
                _primary.DisableStanceContactLifecycle();
                _secondary?.DisableStanceContactLifecycle();
                RestoreBilateralFootSelfCollisionIsolation();
                return "ARL bilateral stance-only contact | enabled=False | FootCollider=OFF";
            }

            if (!PrepareBilateralFootSelfCollisionIsolation(out string selfPairError))
            {
                _primary.DisableStanceContactLifecycle();
                _secondary?.DisableStanceContactLifecycle();
                return "ARL bilateral stance-only contact | enabled=False | reason=BILATERAL_FOOT_SELF_COLLISION_ISOLATION_FAILED:" + (selfPairError ?? "<none>");
            }

            _primary.SetStanceContactLifecycleEnabled(true);
            _secondary?.SetStanceContactLifecycleEnabled(true);
            if (!ValidateBilateralFootSelfCollisionIsolation(out string readbackError))
            {
                _primary.DisableStanceContactLifecycle();
                _secondary?.DisableStanceContactLifecycle();
                RestoreBilateralFootSelfCollisionIsolation();
                return "ARL bilateral stance-only contact | enabled=False | reason=BILATERAL_FOOT_SELF_COLLISION_READBACK_FAILED:" + (readbackError ?? "<none>");
            }

            return "ARL bilateral stance-only contact | enabled=True | policy=PER_FOOT_CONFIRMED_STANCE_PLUS_CURRENT_EXACT_STATIC_IDENTITY" +
                   " | swingTransitionRelease=SAME_FIXEDUPDATE | landingHoldSec=" + FootStanceOnlyAuthorizedContactLifecycle.MinQualifiedStanceHoldSeconds.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture) +
                   " | bilateralSelfIgnore=" + _contactShadowSelfPairReadback;
        }

        public string SetFootContactShadowArmA()
        {
            if (!IsBuilt) return "ARL bilateral Foot contact shadow A | built=False";
            int restoreFailures = DisableFootContactShadowPreserveTraceInternal();
            _contactShadowSelfPairRestoreFailures = 0;
            _primary?.SetFootContactShadowArmA();
            _secondary?.SetFootContactShadowArmA();
            return "ARL bilateral Foot contact shadow | arm=" + FootQualifiedContactShadow.ArmA +
                   " | active=False | restoreFailures=" + restoreFailures;
        }

        public bool EnablePreparedFootContactShadowArmB(out string detail)
        {
            detail = null;
            if (_primary == null || (_bilateralEnabled && _secondary == null))
            {
                detail = "BILATERAL_FOOT_FOUNDATION_NOT_READY";
                return false;
            }

            if (!ValidateBilateralFootSelfCollisionIsolation(out string selfPairError))
            {
                DisableFootContactShadowPreserveTraceInternal();
                detail = "BILATERAL_FOOT_SELF_COLLISION_PRE_ENABLE_READBACK_FAILED:" + (selfPairError ?? "<none>");
                return false;
            }

            if (!_primary.EnablePreparedFootContactShadowArmB(out string pError))
            {
                DisableFootContactShadowPreserveTraceInternal();
                detail = "PRIMARY_ENABLE_FAILED:" + (pError ?? "<none>");
                return false;
            }
            if (_secondary != null && !_secondary.EnablePreparedFootContactShadowArmB(out string sError))
            {
                DisableFootContactShadowPreserveTraceInternal();
                detail = "SECONDARY_ENABLE_FAILED:" + (sError ?? "<none>");
                return false;
            }

            if (!ValidateBilateralFootSelfCollisionIsolation(out string postEnableSelfPairError))
            {
                DisableFootContactShadowPreserveTraceInternal();
                detail = "BILATERAL_FOOT_SELF_COLLISION_POST_ENABLE_READBACK_FAILED:" + (postEnableSelfPairError ?? "<none>");
                return false;
            }

            detail = "enabled=True | arm=" + FootQualifiedContactShadow.ArmB +
                     " | bilateralSelfIgnore=" + _contactShadowSelfPairReadback;
            return true;
        }

        public string DisableFootContactShadowPreserveTrace()
        {
            int failures = DisableFootContactShadowPreserveTraceInternal();
            return "ARL bilateral Foot contact shadow | active=False | restoreFailures=" + failures;
        }

        private int DisableFootContactShadowPreserveTraceInternal()
        {
            int failures = 0;
            if (_primary != null) failures += _primary.DisableFootContactShadowPreserveTrace();
            if (_secondary != null) failures += _secondary.DisableFootContactShadowPreserveTrace();
            failures += RestoreBilateralFootSelfCollisionIsolation();
            return failures;
        }

        private bool PrepareBilateralFootSelfCollisionIsolation(out string error)
        {
            error = null;
            if (RestoreBilateralFootSelfCollisionIsolation() != 0)
            {
                error = "PREVIOUS_BILATERAL_FOOT_SELF_PAIR_NOT_RELEASED";
                return false;
            }
            _contactShadowSelfPairPrimary = _primary != null ? _primary.FootContactShadowCollider : null;
            _contactShadowSelfPairSecondary = _secondary != null ? _secondary.FootContactShadowCollider : null;
            _contactShadowSelfPairOwned = false;
            _contactShadowSelfPairReadback = false;

            if (!_bilateralEnabled)
            {
                _contactShadowSelfPairReadback = true;
                return true;
            }
            if (_contactShadowSelfPairPrimary == null || _contactShadowSelfPairSecondary == null ||
                _contactShadowSelfPairPrimary == _contactShadowSelfPairSecondary)
            {
                error = "BILATERAL_FOOT_COLLIDER_PAIR_MISSING_OR_NOT_DISTINCT";
                return false;
            }

            try
            {
                bool alreadyIgnored = UnityEngine.Physics.GetIgnoreCollision(_contactShadowSelfPairPrimary, _contactShadowSelfPairSecondary);
                if (!alreadyIgnored)
                {
                    UnityEngine.Physics.IgnoreCollision(_contactShadowSelfPairPrimary, _contactShadowSelfPairSecondary, true);
                    _contactShadowSelfPairOwned = true;
                }
                return ValidateBilateralFootSelfCollisionIsolation(out error);
            }
            catch (Exception e)
            {
                error = "BILATERAL_FOOT_SELF_IGNORE_PREPARE_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                _contactShadowSelfPairReadback = false;
                return false;
            }
        }

        private bool ValidateBilateralFootSelfCollisionIsolation(out string error)
        {
            error = null;
            if (!_bilateralEnabled)
            {
                _contactShadowSelfPairReadback = true;
                return true;
            }
            if (_contactShadowSelfPairPrimary == null || _contactShadowSelfPairSecondary == null)
            {
                _contactShadowSelfPairReadback = false;
                error = "BILATERAL_FOOT_SELF_PAIR_NOT_PREPARED";
                return false;
            }
            try
            {
                _contactShadowSelfPairReadback = UnityEngine.Physics.GetIgnoreCollision(_contactShadowSelfPairPrimary, _contactShadowSelfPairSecondary);
                if (!_contactShadowSelfPairReadback)
                {
                    error = "BILATERAL_FOOT_SELF_PAIR_NOT_IGNORED";
                    return false;
                }
                return true;
            }
            catch (Exception e)
            {
                _contactShadowSelfPairReadback = false;
                error = "BILATERAL_FOOT_SELF_IGNORE_QUERY_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                return false;
            }
        }

        private int RestoreBilateralFootSelfCollisionIsolation()
        {
            int failures = 0;
            try
            {
                if (_contactShadowSelfPairOwned && _contactShadowSelfPairPrimary != null && _contactShadowSelfPairSecondary != null)
                    UnityEngine.Physics.IgnoreCollision(_contactShadowSelfPairPrimary, _contactShadowSelfPairSecondary, false);
            }
            catch
            {
                failures++;
            }

            _contactShadowSelfPairRestoreFailures += failures;
            // Retain the exact owned pair after a native restore failure. A later retry must
            // restore this pair, rather than forget it and capture the modified state as baseline.
            if (failures != 0) return failures;
            _contactShadowSelfPairPrimary = null;
            _contactShadowSelfPairSecondary = null;
            _contactShadowSelfPairOwned = false;
            _contactShadowSelfPairReadback = false;
            return failures;
        }

        public bool NotifyFootContactShadowCollision(string side, string phase, Collision collision)
        {
            if (string.Equals(side, FootSupportSurfaceIsolation.CallbackSide, StringComparison.Ordinal))
                return _supportSurface.NotifyBoxCollision(phase, collision);
            if (_primary != null && string.Equals(side, PrimarySide, StringComparison.OrdinalIgnoreCase))
                return _primary.NotifyFootContactShadowCollision(phase, collision);
            if (_secondary != null && string.Equals(side, SecondarySide, StringComparison.OrdinalIgnoreCase))
                return _secondary.NotifyFootContactShadowCollision(phase, collision);
            return false;
        }

        public string GetFootAnkleStatus()
        {
            if (!IsBuilt)
                return "ARL bilateral foot/ankle foundation | built=False";
            string p = _primary != null ? _primary.GetFootAnkleStatus() : "primary=missing";
            string s = _secondary != null ? _secondary.GetFootAnkleStatus() : "secondary=disabled";
            FootAnkleFoundationReport pr = PrimaryFootReport;
            FootAnkleFoundationReport sr = SecondaryFootReport;
            bool parity = !_bilateralEnabled || (pr != null && sr != null &&
                          pr.FixedUpdateSequence == sr.FixedUpdateSequence &&
                          pr.TargetRenderSampleSequence == sr.TargetRenderSampleSequence &&
                          pr.TraceSamples == sr.TraceSamples);
            return Combine("ARL bilateral foot/ankle foundation | clockParity=" + parity, p, s);
        }

        public string GetFootAnkleFullReport()
        {
            string p = PrimaryFootReport != null ? PrimaryFootReport.GetFullText() : "primary foot report unavailable";
            string s = SecondaryFootReport != null ? SecondaryFootReport.GetFullText() : "secondary foot report unavailable";
            return p + (_bilateralEnabled ? "\n\n--- SECONDARY ---\n" + s : string.Empty);
        }

        public string ResetPose()
        {
            if (!IsBuilt) return "ARL reset：bilateral foundation尚未创建。";
            string secondaryReset = _secondary != null ? _secondary.ResetPose() : "secondary=disabled";
            string primaryReset = _primary.ResetPose();
            return Combine("ARL bilateral reset", primaryReset, secondaryReset);
        }

        public string GetStatus()
        {
            RefreshFinalizedReadinessSnapshot();
            return _report.GetConsoleSummary() +
                   "\nprimary: " + (_primary != null ? _primary.Report.GetConsoleSummary() : "<none>") +
                   (_bilateralEnabled ? "\nsecondary: " + (_secondary != null ? _secondary.Report.GetConsoleSummary() : "<none>") : string.Empty) +
                   "\n" + GetFootAnkleStatus();
        }

        public string GetBilateralStatus()
        {
            RefreshFinalizedReadinessSnapshot();
            return _report.GetConsoleSummary() + " | " + GetStepContextStatus() + " | key=" + (_report.Key ?? "<not-finalized>");
        }

        private void RefreshFinalizedReadinessSnapshot()
        {
            // During an active trace, readiness is intentionally not finalized. After trace stop, however,
            // TwoBody freeze may already have transitioned to READY while the cached per-leg/bilateral reports
            // still reflect the pre-finalization sample. Refresh both child certifications first and then
            // recompute the bilateral verdict from the existing clock statistics. This is diagnostic/state
            // truthfulness only; it never changes drive, controller or safety state.
            if (!_traceEnabled)
            {
                _primary?.RefreshSingleLegFreezeReadiness();
                _secondary?.RefreshSingleLegFreezeReadiness();
                _diagnostics.Finalize(_report, PrimaryReport, SecondaryReport, IsBuilt, _distinctSourceChains);
            }
            else
            {
                _diagnostics.CopyCurrent(_report, PrimaryReport, SecondaryReport);
                _report.Built = IsBuilt;
            }
            _report.TraceEnabled = _traceEnabled;
        }

        public string Rollback(string reason)
        {
            // Old child destructors discard some handles. Do not let a second empty
            // teardown certify a previously failed one; this legacy case needs restart.
            if (_destroyed && !LastRollbackSucceeded)
                return "ARL bilateral rollback | success=False | cleanupPending=True | failedLegacyTeardownRequiresRestart=True";
            string detail = RollbackInternal(reason, out bool success);
            _destroyed = true;
            _report.Built = false;
            _report.FoundationReady = false;
            LastRollbackSucceeded = success;
            _report.Reason = (LastRollbackSucceeded ? "ROLLED_BACK:" : "ROLLBACK_WITH_ERRORS:") + reason;
            return "ARL bilateral rollback | success=" + LastRollbackSucceeded + " | reason=" + reason + " | " + detail;
        }

        private string RollbackInternal(string reason, out bool success)
        {
            _hipPelvisLoadPath.EndTrace();
            success = true;
            string driveStop;
            try { driveStop = SetDrive(false); }
            catch (Exception e) { success = false; driveStop = "BILATERAL_DRIVE_OFF_EXCEPTION=" + e.GetType().Name; }
            // Quiesce both feet before detaching support probes/restoring collision pairs. Teardown
            // continues after a failed attempt; child destructors still get their independent attempts.
            string surfaceStop;
            try { surfaceStop = StopFootSupportSurface(); }
            catch (Exception e) { success = false; surfaceStop = "SUPPORT_RELEASE_EXCEPTION=" + e.GetType().Name; }
            if (!_supportSurface.StoppedSafe) success = false;
            int contactShadowRestoreFailures = RestoreBilateralFootSelfCollisionIsolation();
            if (contactShadowRestoreFailures > 0)
                success = false;
            string secondary = "secondary=not-built";
            string primary = "primary=not-built";
            try
            {
                if (_secondary != null)
                {
                    secondary = "secondary=" + _secondary.Rollback(reason);
                    if (!_secondary.LastRollbackSucceeded)
                        success = false;
                }
            }
            catch (Exception e)
            {
                success = false;
                secondary = "secondary_failed=" + e.GetType().Name + ":" + e.Message;
                Plugin.Logger?.LogWarning("[ARL][BILATERAL_ROLLBACK_CHILD_FAILED] side=secondary | " + e.GetType().Name + " / " + e.Message);
            }
            try
            {
                if (_primary != null)
                {
                    primary = "primary=" + _primary.Rollback(reason);
                    if (!_primary.LastRollbackSucceeded)
                        success = false;
                }
            }
            catch (Exception e)
            {
                success = false;
                primary = "primary_failed=" + e.GetType().Name + ":" + e.Message;
                Plugin.Logger?.LogWarning("[ARL][BILATERAL_ROLLBACK_CHILD_FAILED] side=primary | " + e.GetType().Name + " / " + e.Message);
            }
            _secondary = null;
            _primary = null;
            _traceEnabled = false;
            return primary + " | " + secondary + " | driveQuiesce=" + driveStop + " | contactShadowSelfPairRestoreFailures=" + contactShadowRestoreFailures + " | " + surfaceStop;
        }

        private void FailCloseStanceContactIsolation(string detail)
        {
            _primary?.CancelQueuedHipPair(_contextCursor.Fixed);
            _secondary?.CancelQueuedHipPair(_contextCursor.Fixed);
            string reason = "stanceContactIsolation=True|" + (detail ?? "<none>");
            if (!_bilateralFailClosedLatched)
            {
                _bilateralFailClosedLatched = true;
                _report.FailClosedEvents++;
                _report.LastFailClosedReason = reason;
                ActiveRagdollLabRuntime.RecordAutomaticStop("BILATERAL_CONTACT", detail);
                Plugin.Logger?.LogError("[ARL][BILATERAL_FAIL_CLOSED] " + reason);
            }

            try
            {
                _primary?.DisableStanceContactLifecycle();
                _secondary?.DisableStanceContactLifecycle();
                DisableFootContactShadowPreserveTraceInternal();
            }
            finally { SetDrive(false); }
        }

        private void FailCloseBothSides(bool driveMismatch, bool anyAbort)
        {
            _primary?.CancelQueuedHipPair(_contextCursor.Fixed);
            _secondary?.CancelQueuedHipPair(_contextCursor.Fixed);
            string reason =
                "driveMismatch=" + driveMismatch +
                "|abort=" + anyAbort +
                "|primaryAbort=" + _primary.Report.DriveAborted + "/" + (_primary.Report.DriveAbortReason ?? "<none>") +
                "|secondaryAbort=" + _secondary.Report.DriveAborted + "/" + (_secondary.Report.DriveAbortReason ?? "<none>");

            if (!_bilateralFailClosedLatched)
            {
                _bilateralFailClosedLatched = true;
                _report.FailClosedEvents++;
                _report.LastFailClosedReason = reason;
                ActiveRagdollLabRuntime.RecordAutomaticStop("BILATERAL_COORDINATOR", reason);
                Plugin.Logger?.LogError("[ARL][BILATERAL_FAIL_CLOSED] " + reason);
            }

            // v0.4.1.1/v0.4.7: a bilateral fail-close must also atomically close the phase-gated Foot
            // contact boundary and restore coordinator-owned self-isolation while preserving trace evidence.
            try
            {
                _primary?.DisableStanceContactLifecycle();
                _secondary?.DisableStanceContactLifecycle();
                DisableFootContactShadowPreserveTraceInternal();
            }
            finally { SetDrive(false); }
        }

        private string CombineStatus(string header, Func<SingleLegShadowChain, string> get)
        {
            if (!IsBuilt) return header + " | built=False";
            string p = get(_primary);
            string s = _secondary != null ? get(_secondary) : "secondary=disabled";
            return Combine(header, p, s);
        }

        private string Combine(string header, string primary, string secondary)
        {
            return header +
                   "\nprimary(" + (PrimarySide ?? "<none>") + "): " + primary +
                   (_bilateralEnabled ? "\nsecondary(" + (SecondarySide ?? "<none>") + "): " + secondary : string.Empty);
        }

        private static bool ValidateDistinctTargets(SingleLegTargetReport primary, SingleLegTargetReport secondary, bool bilateralEnabled)
        {
            if (primary == null || !primary.Success || primary.UpperLeg == null || primary.LowerLeg == null || primary.Foot == null)
                return false;
            if (!bilateralEnabled)
                return true;
            if (secondary == null || !secondary.Success || secondary.UpperLeg == null || secondary.LowerLeg == null || secondary.Foot == null)
                return false;
            if (primary.UpperLeg == secondary.UpperLeg ||
                primary.LowerLeg == secondary.LowerLeg ||
                primary.Foot == secondary.Foot ||
                (primary.Toe != null && secondary.Toe != null && primary.Toe == secondary.Toe))
                return false;
            return !string.Equals(primary.RequestedSide, secondary.RequestedSide, StringComparison.OrdinalIgnoreCase);
        }
    }
}
