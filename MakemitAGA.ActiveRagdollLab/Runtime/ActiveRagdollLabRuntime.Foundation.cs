using System;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Runtime
{
    internal static partial class ActiveRagdollLabRuntime
    {
        private static FoundationPoseWindow _foundationWindow = new FoundationPoseWindow();
        private static readonly Vector3[] _foundationPositions = new Vector3[FoundationPoseWindow.BoneCount];
        private static readonly Quaternion[] _foundationRotations = new Quaternion[FoundationPoseWindow.BoneCount];
        private static Action _completeFoundationSetup;
        private static bool _foundationPoseQualified;
        internal static bool UsesStableFoundationSetup => _profile != null && _profile.SingleLegEnabled &&
            _profile.BilateralLegEnabled && _profile.FootAnkleFoundationEnabled;
        internal static bool FoundationSetupPending => _completeFoundationSetup != null && _foundationWindow.Pending;
        internal static string FoundationSetupStatus() => "foundationSetup | pending=" + FoundationSetupPending +
            " | poseQualified=" + _foundationPoseQualified + " | reason=" + _foundationWindow.Reason +
            " | stableSeconds=" + _foundationWindow.StableSeconds.ToString("F3", System.Globalization.CultureInfo.InvariantCulture);

        internal static string RequestStableFoundationSetup(Action complete)
        {
            if (FoundationSetupPending) return "ARL [SETUP_PENDING] " + FoundationSetupStatus();
            if (_shutdown || _cleanupPending || HasActiveFoundation || !AuditReady ||
                _legTarget == null || !_legTarget.FootFoundationReady ||
                _secondaryLegTarget == null || !_secondaryLegTarget.FootFoundationReady || complete == null)
                return "ARL [SETUP_STOP] 双侧源骨/foot foundation 未就绪；查看 audit，勿继续 drive/PCA。";
            _foundationPoseQualified = false;
            _foundationWindow.Begin(Time.unscaledTime);
            _completeFoundationSetup = complete;
            string text = "ARL [SETUP_PENDING] 请让米塔在同一平地站稳；连续稳定约1秒后自动建模，最长等待15秒。看到[SETUP_READY]再继续drive/PCA。";
            _logger?.LogInfo("[ARL][FOUNDATION_SETUP][PENDING] " + text);
            return text;
        }
        private static void CancelFoundationSetup(string reason)
        {
            bool pending = FoundationSetupPending;
            _completeFoundationSetup = null; _foundationPoseQualified = false;
            _foundationWindow.Cancel(reason);
            if (pending) _logger?.LogInfo("[ARL][FOUNDATION_SETUP][CANCELLED] reason=" + reason);
        }
        private static void CaptureFoundationBone(int index, Transform bone)
        {
            if (bone == null) throw new InvalidOperationException("FOUNDATION_SOURCE_BONE_LOST");
            _foundationPositions[index] = bone.position; _foundationRotations[index] = bone.rotation;
        }
        // Runs only while setup is pending, on the shared genuine source clock. No second
        // sampler, new physics clock, per-frame allocation or persistent observer is added.
        private static bool AdvanceFoundationSetup(SourceStepContext source)
        {
            if (!FoundationSetupPending) return false;
            try
            {
                CaptureFoundationBone(0, _audit.Pelvis);
                CaptureFoundationBone(1, _legTarget.UpperLeg); CaptureFoundationBone(2, _legTarget.LowerLeg);
                CaptureFoundationBone(3, _legTarget.Foot);
                CaptureFoundationBone(4, _secondaryLegTarget.UpperLeg); CaptureFoundationBone(5, _secondaryLegTarget.LowerLeg);
                CaptureFoundationBone(6, _secondaryLegTarget.Foot);
                _foundationWindow.Observe(source, _foundationPositions, _foundationRotations);
                if (_foundationWindow.Pending) return true;
                Action complete = _completeFoundationSetup; _completeFoundationSetup = null;
                if (!_foundationWindow.Qualified) { ShowFoundationSetupStop(_foundationWindow.Reason); return true; }
                // Build reads the same live Transforms synchronously. Its binding invalidation
                // requires returning from this LateUpdate; the next frame starts the new epoch.
                // Audit owns pose-dependent frame/axis measurements as well as references.
                // Refresh them once at the qualified pose; never combine an old audit frame
                // with new Foot/Hip bindings or silently switch the observed character.
                var expectedAudit = _audit; var expectedLeft = _legTarget; var expectedRight = _secondaryLegTarget;
                AuditCore(null, true);
                if (!AuditReady || _legTarget == null || _secondaryLegTarget == null ||
                    _audit.TargetRoot != expectedAudit.TargetRoot || _audit.Pelvis != expectedAudit.Pelvis ||
                    _audit.Chest != expectedAudit.Chest || _audit.Spine != expectedAudit.Spine ||
                    _legTarget.UpperLeg != expectedLeft.UpperLeg || _legTarget.LowerLeg != expectedLeft.LowerLeg ||
                    _legTarget.Foot != expectedLeft.Foot || _legTarget.Toe != expectedLeft.Toe ||
                    _secondaryLegTarget.UpperLeg != expectedRight.UpperLeg || _secondaryLegTarget.LowerLeg != expectedRight.LowerLeg ||
                    _secondaryLegTarget.Foot != expectedRight.Foot || _secondaryLegTarget.Toe != expectedRight.Toe)
                    throw new InvalidOperationException("QUALIFIED_SOURCE_BINDING_CHANGED");
                _logger?.LogInfo("[ARL][FOUNDATION_SETUP][POSE_QUALIFIED] epoch=" + source.Epoch + " | source=" + source.Sequence +
                    " | ankleHeightDifferenceMm=" + (Mathf.Abs(_foundationPositions[3].y - _foundationPositions[6].y) * 1000f));
                complete();
                _foundationPoseQualified = IsBuilt && _legChain != null && _legChain.IsBuilt;
                if (!_foundationPoseQualified) ShowFoundationSetupStop("BUILD_OR_SAFE_STATE_FAILED");
                else _logger?.LogInfo("[ARL][FOUNDATION_SETUP][SETUP_READY] " + FoundationSetupStatus());
            }
            catch (Exception e)
            {
                string reason = "SETUP_EXCEPTION:" + e.GetType().Name + ":" + e.Message;
                CancelFoundationSetup(reason);
                if (HasActiveFoundation) RollbackInternal(reason);
                ShowFoundationSetupStop(reason);
            }
            return true;
        }
        private static void ShowFoundationSetupStop(string reason)
        {
            _logger?.LogWarning("[ARL][FOUNDATION_SETUP][STOP] reason=" + reason + " | " + FoundationSetupStatus());
            ActiveRagdollConsolePatch.TryPrintGame("<color=red>ARL [SETUP_STOP] 未完成建模：" + reason +
                "</color> 让米塔站稳后重新arl_test_setup；不要继续drive/PCA。");
        }
    }
}
