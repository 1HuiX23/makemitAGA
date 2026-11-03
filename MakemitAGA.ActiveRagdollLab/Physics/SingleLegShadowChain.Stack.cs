using System;
using MakemitAGA.ActiveRagdollLab.Runtime;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Physics
{
    // Frozen stack selection and readiness. Methods are moved without changing their order of invocation.
    internal sealed partial class SingleLegShadowChain
    {

        private string SetHipMomentumRecoveryBraking(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip momentum-aware recovery braking A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip momentum-aware recovery braking A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip momentum-aware recovery braking A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip momentum-aware recovery braking A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip momentum-aware recovery braking A/B：v0.2.72 composite absorber B 依赖当前 baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipSourceReferenceShockAbsorber.Enabled)
                return "ARL Hip momentum-aware recovery braking A/B：v0.2.71 Source absorber B 依赖当前 baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipVectorCaptureDamping.Enabled)
                return "ARL Hip momentum-aware recovery braking A/B：vector capture 正在使用 legacy-ON 作为固定基线；内部 frozen-stack 依赖未满足。";

            _hipMomentumRecoveryBraking.SetEnabled(enabled, _report);
            ResetTraceStats();
            UpdateReport();
            return GetHipMomentumRecoveryBrakingStatus();
        }

        private string GetHipMomentumRecoveryBrakingStatus()
        {
            if (!IsBuilt)
                return "ARL Hip momentum-aware recovery braking A/B | built=False";
            _hipMomentumRecoveryBraking.CopyToReport(_report);
            return _hipMomentumRecoveryBraking.GetStatus(_report);
        }

        private string SetHipVectorCaptureDamping(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip 3D vector capture damping A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip 3D vector capture damping A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip 3D vector capture damping A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip 3D vector capture damping A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (enabled && !_hipMomentumRecoveryBraking.Enabled)
                return "ARL Hip 3D vector capture damping A/B：B arm 需要 v0.2.64 legacy baseline 已 ON。内部 frozen-stack 依赖未满足。";

            if (!enabled && _hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip 3D vector capture damping：v0.2.72 composite absorber B 依赖 vector baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipSourceReferenceShockAbsorber.Enabled)
                return "ARL Hip 3D vector capture damping：v0.2.71 Source absorber B 依赖 vector baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled)
                return "ARL Hip 3D vector capture damping：v0.2.69 high-error radial-away brake-floor B 正在启用。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipVectorCaptureDamping.SymmetricRadialEnabled)
                return "ARL Hip 3D vector capture damping：v0.2.68 symmetric radial baseline 正在启用。内部 frozen-stack 依赖未满足。";

            _hipVectorCaptureDamping.SetEnabled(enabled, _report);
            ResetTraceStats();
            UpdateReport();
            return GetHipVectorCaptureDampingStatus();
        }

        private string SetHipSymmetricRadialCapture(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip symmetric radial capture A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip symmetric radial capture A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip symmetric radial capture A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip symmetric radial capture A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (enabled && !_hipVectorCaptureDamping.Enabled)
                return "ARL Hip symmetric radial capture A/B：B 需要 v0.2.67 vector capture 已 ON。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipMomentumRecoveryBraking.Enabled)
                return "ARL Hip symmetric radial capture A/B：固定 legacy baseline 尚未 ON。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip symmetric radial capture A/B：v0.2.72 composite absorber B 依赖 symmetric baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipSourceReferenceShockAbsorber.Enabled)
                return "ARL Hip symmetric radial capture A/B：v0.2.71 Source absorber B 依赖 symmetric baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled)
                return "ARL Hip symmetric radial capture A/B：v0.2.69 brake-floor B 依赖 v0.2.68 symmetric baseline。内部 frozen-stack 依赖未满足。";

            _hipVectorCaptureDamping.SetSymmetricRadialEnabled(enabled, _report);
            ResetTraceStats();
            UpdateReport();
            return GetHipVectorCaptureDampingStatus();
        }

        private string SetHipHighErrorAwayBrakeFloor(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip high-error radial-away brake-floor A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip high-error radial-away brake-floor A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip high-error radial-away brake-floor A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip high-error radial-away brake-floor A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (enabled && !_hipVectorCaptureDamping.Enabled)
                return "ARL Hip high-error radial-away brake-floor A/B：B 需要 v0.2.67 vector foundation 已 ON。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipVectorCaptureDamping.SymmetricRadialEnabled)
                return "ARL Hip high-error radial-away brake-floor A/B：B 以 v0.2.68 symmetric radial 为 A baseline。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipMomentumRecoveryBraking.Enabled)
                return "ARL Hip high-error radial-away brake-floor A/B：固定 legacy baseline 尚未 ON。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip high-error radial-away brake-floor：v0.2.72 composite absorber B 依赖 v0.2.69 floor baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipSourceReferenceShockAbsorber.Enabled)
                return "ARL Hip high-error radial-away brake-floor：v0.2.71 Source absorber B 依赖 v0.2.69 floor baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && !_hipControlFrame.ReferenceLagGuardRebaseEnabled)
                return "ARL Hip high-error radial-away brake-floor：v0.2.70 lag-guard rebase B 依赖 v0.2.69 floor baseline。内部 frozen-stack 依赖未满足。";

            _hipVectorCaptureDamping.SetHighErrorAwayBrakeFloorEnabled(enabled, _report);
            ResetTraceStats();
            UpdateReport();
            return GetHipHighErrorAwayBrakeFloorStatus();
        }

        private string GetHipHighErrorAwayBrakeFloorStatus()
        {
            if (!IsBuilt)
                return "ARL Hip high-error radial-away brake-floor A/B | built=False";
            _hipVectorCaptureDamping.CopyToReport(_report);
            return _hipVectorCaptureDamping.GetStatus(_report);
        }

        private string SetHipReferenceLagGuardRebase(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip reference lag-guard rebase A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip reference lag-guard rebase A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip reference lag-guard rebase A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip reference lag-guard rebase A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && !_hipMomentumRecoveryBraking.Enabled)
                return "ARL Hip reference lag-guard rebase B：需要 v0.2.69 production baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && !_hipVectorCaptureDamping.Enabled)
                return "ARL Hip reference lag-guard rebase B：需要 vector capture ON。内部 frozen-stack 依赖未满足。";
            if (!enabled && !_hipVectorCaptureDamping.SymmetricRadialEnabled)
                return "ARL Hip reference lag-guard rebase B：需要 v0.2.68 symmetric radial baseline ON。内部 frozen-stack 依赖未满足。";
            if (!enabled && !_hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled)
                return "ARL Hip reference lag-guard rebase B：需要 v0.2.69 radial-away floor ON。内部 frozen-stack 依赖未满足。";
            if (enabled && _hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip reference lag-guard rebase：v0.2.72 composite absorber B 固定依赖 rebase-suppressed baseline。内部 frozen-stack 依赖未满足。";
            if (enabled && _hipSourceReferenceShockAbsorber.Enabled)
                return "ARL Hip reference lag-guard rebase：v0.2.71 Source absorber B 固定依赖 rebase-suppressed baseline。内部 frozen-stack 依赖未满足。";

            _hipControlFrame.SetReferenceLagGuardRebaseEnabled(enabled);
            ResetTraceStats();
            UpdateReport();
            return GetHipReferenceLagGuardRebaseStatus();
        }

        private string GetHipReferenceLagGuardRebaseStatus()
        {
            if (!IsBuilt)
                return "ARL Hip reference lag-guard rebase A/B | built=False";
            UpdateReport();
            return _hipControlFrame.GetReferenceLagGuardRebaseKey();
        }

        private string SetHipSourceReferenceShockAbsorber(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip Source-reference shock absorber A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip Source-reference shock absorber A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip Source-reference shock absorber A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip Source-reference shock absorber A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (enabled && !_hipMomentumRecoveryBraking.Enabled)
                return "ARL Hip Source-reference shock absorber B：需要 v0.2.69 baseline。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipVectorCaptureDamping.Enabled)
                return "ARL Hip Source-reference shock absorber B：需要 vector capture ON。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipVectorCaptureDamping.SymmetricRadialEnabled)
                return "ARL Hip Source-reference shock absorber B：需要 v0.2.68 symmetric radial baseline ON。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled)
                return "ARL Hip Source-reference shock absorber B：需要 v0.2.69 radial-away floor ON。内部 frozen-stack 依赖未满足。";
            if (enabled && _hipControlFrame.ReferenceLagGuardRebaseEnabled)
                return "ARL Hip Source-reference shock absorber B：v0.2.71 baseline 固定采用 v0.2.70 rebase-suppressed arm。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip Source-reference shock absorber：v0.2.72 composite absorber B 依赖 v0.2.71 local absorber。内部 frozen-stack 依赖未满足。";

            _hipSourceReferenceShockAbsorber.SetEnabled(
                enabled,
                _sampledTargetHipRelative,
                _sampledTargetHipRelativeAngularVelocityLocal,
                _report);
            _sourceControlTargetHipRelative = _sampledTargetHipRelative;
            _sourceControlTargetHipRelativeAngularVelocityLocal = _sampledTargetHipRelativeAngularVelocityLocal;
            ResetTraceStats();
            UpdateReport();
            return GetHipSourceReferenceShockAbsorberStatus();
        }

        private string GetHipSourceReferenceShockAbsorberStatus()
        {
            if (!IsBuilt)
                return "ARL Hip Source-reference shock absorber A/B | built=False";
            _hipSourceReferenceShockAbsorber.CopyToReport(_report);
            return _hipSourceReferenceShockAbsorber.GetStatus(_report);
        }

        private string SetHipCompositeReferenceShockAbsorber(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip final-composite world-reference shock absorber A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip final-composite world-reference shock absorber A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip final-composite world-reference shock absorber A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip final-composite world-reference shock absorber A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _kneeCharacterJointResponseIsolation.Enabled)
                return "ARL Hip final-composite world-reference shock absorber A/B：v0.2.76 B 依赖完整 Hip reference baseline。内部 frozen-stack 依赖未满足。";
            if (!enabled && _hipCharacterJointResponseIsolation.Enabled)
                return "ARL Hip final-composite world-reference shock absorber A/B：v0.2.73 B 依赖完整 v0.2.72 composite baseline。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipSourceReferenceShockAbsorber.Enabled)
                return "ARL Hip final-composite world-reference shock absorber B：需要 v0.2.71 Hip-local Source absorber ON。内部 frozen-stack 依赖未满足。";
            if (enabled && _hipControlFrame.ReferenceLagGuardRebaseEnabled)
                return "ARL Hip final-composite world-reference shock absorber B：baseline 固定采用 v0.2.70 rebase-suppressed arm。内部 frozen-stack 依赖未满足。";
            if (enabled && (!_hipMomentumRecoveryBraking.Enabled || !_hipVectorCaptureDamping.Enabled ||
                            !_hipVectorCaptureDamping.SymmetricRadialEnabled || !_hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled))
                return "ARL Hip final-composite world-reference shock absorber B：需要 v0.2.69 momentum/vector/symmetric/floor baseline 全部 ON。";
            if (enabled && !_hipMechanicalParent.UseInertialized)
                return "ARL Hip final-composite world-reference shock absorber B：固定 baseline 要求 inertialized mechanical parent。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipMechanicalParentTransport.Enabled)
                return "ARL Hip final-composite world-reference shock absorber B：固定 baseline 要求 parent transport ON。内部 frozen-stack 依赖未满足。";
            if (enabled && !_hipAnchorLinearTransport.Enabled)
                return "ARL Hip final-composite world-reference shock absorber B：固定 baseline 要求 anchor linear transport ON。内部 frozen-stack 依赖未满足。";

            Quaternion parentRotation = (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization)
                ? _hipControlFrame.Rotation
                : _pelvisBody.rotation;
            Vector3 parentW = (_hipControlFrame.Active && _profile.LegHipControlFrameInertialization)
                ? _hipControlFrame.AngularVelocityWorld
                : _measuredPelvisAngularVelocity;
            Quaternion inputWorldPose = parentRotation * _controlTargetHipRelative;
            Vector3 inputWorldW = parentW + parentRotation * _controlTargetHipRelativeAngularVelocityLocal;
            _hipCompositeReferenceShockAbsorber.SetEnabled(enabled, inputWorldPose, inputWorldW, _report);
            ResetTraceStats();
            UpdateReport();
            return GetHipCompositeReferenceShockAbsorberStatus();
        }

        private string GetHipCompositeReferenceShockAbsorberStatus()
        {
            if (!IsBuilt)
                return "ARL Hip final-composite world-reference shock absorber A/B | built=False";
            _hipCompositeReferenceShockAbsorber.CopyToReport(_report);
            return _hipCompositeReferenceShockAbsorber.GetStatus(_report);
        }

        private string GetHipVectorCaptureDampingStatus()
        {
            if (!IsBuilt)
                return "ARL Hip 3D vector capture damping A/B | built=False";
            _hipVectorCaptureDamping.CopyToReport(_report);
            return _hipVectorCaptureDamping.GetStatus(_report);
        }

        private string SetHipCharacterJointResponseIsolation(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip CharacterJoint response isolation：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip CharacterJoint response isolation：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip CharacterJoint response isolation：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip CharacterJoint response isolation：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _kneeCharacterJointResponseIsolation.Enabled)
                return "ARL Hip CharacterJoint response isolation：v0.2.76 B 仍启用。内部 frozen-stack 依赖未满足。";

            if (enabled)
            {
                if (_hipCharacterJointResponseIsolation.Enabled)
                    return GetHipCharacterJointResponseIsolationStatus();
                if (_hipJoint == null)
                    return "ARL Hip CharacterJoint response isolation：无法进入 B，Hip CharacterJoint 已缺失。";
                if (!_hipCompositeReferenceShockAbsorber.Enabled || !_hipSourceReferenceShockAbsorber.Enabled)
                    return "ARL Hip CharacterJoint response isolation：v0.2.73 B 要求 v0.2.72 composite absorber 与 v0.2.71 Source absorber 都保持 ON。";
                if (!_hipMechanicalParent.UseInertialized || !_hipMechanicalParentTransport.Enabled || !_hipAnchorLinearTransport.Enabled)
                    return "ARL Hip CharacterJoint response isolation：v0.2.73 B 要求 inertialized mechanical parent + angular transport + anchor linear transport production baseline。";
                if (!_hipJoint.enablePreprocessing)
                    return "ARL Hip CharacterJoint response isolation：frozen stack 要求 solver preprocessing baseline 已恢复；当前内部依赖未满足。";

                CharacterJoint jointToRemove = _hipJoint;
                _hipJoint = null;
                _hipCharacterJointResponseIsolation.SetEnabled(true, _report);
                try { Object.Destroy(jointToRemove); }
                catch (Exception e)
                {
                    _hipCharacterJointResponseIsolation.SetEnabled(false, _report);
                    _hipJoint = jointToRemove;
                    return "ARL Hip CharacterJoint response isolation：删除 Hip CharacterJoint 失败：" + e.GetType().Name + "/" + e.Message;
                }
            }
            else
            {
                if (!_hipCharacterJointResponseIsolation.Enabled)
                    return GetHipCharacterJointResponseIsolationStatus();
                if (_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled)
                    return "ARL Hip CharacterJoint response isolation：v0.2.74 Upper-only tether B 仍启用。内部 frozen-stack 依赖未满足。";
                if (_hipJoint != null)
                    return "ARL Hip CharacterJoint response isolation：A 恢复前发现意外 Hip CharacterJoint，拒绝重复创建。";
                _hipCharacterJointResponseIsolation.SetEnabled(false, _report);
                try
                {
                    _hipJoint = Il2CppComponentFactory.Add<CharacterJoint>(_upperObject, "LEG_RESTORE_HIP_CHARACTER_JOINT_V073_A");
                    ConfigureHipJoint();
                }
                catch (Exception e)
                {
                    _hipJoint = null;
                    _hipCharacterJointResponseIsolation.SetEnabled(true, _report);
                    return "ARL Hip CharacterJoint response isolation：恢复 A CharacterJoint 失败；保持 B fail-closed：" + e.GetType().Name + "/" + e.Message;
                }
            }

            ResetTraceStats();
            UpdateReport();
            return GetHipCharacterJointResponseIsolationStatus();
        }

        private string GetHipCharacterJointResponseIsolationStatus()
        {
            if (!IsBuilt)
                return "ARL Hip CharacterJoint response isolation | built=False";
            UpdateJointPresence();
            return _hipCharacterJointResponseIsolation.GetStatus(_report);
        }

        private string SetHipTranslationTetherSerialChain(bool upperOnly)
        {
            if (!IsBuilt)
                return "ARL Hip translation tether serial-chain A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip translation tether serial-chain A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip translation tether serial-chain A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip translation tether serial-chain A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!upperOnly && _kneeCharacterJointResponseIsolation.Enabled)
                return "ARL Hip translation tether serial-chain A/B：v0.2.76 B 仍启用。内部 frozen-stack 依赖未满足。";
            if (!_hipCharacterJointResponseIsolation.Enabled || _hipJoint != null)
                return "ARL Hip translation tether serial-chain A/B：v0.2.74 需要先进入 v0.2.73 B。内部 frozen-stack 依赖未满足。";
            if (!upperOnly && _hipAnchorAccelerationFeedforward.Enabled)
                return "ARL Hip translation tether serial-chain A/B：v0.2.79 B 依赖 Upper-only Hip tether。内部 frozen-stack 依赖未满足。";
            if (!upperOnly && _kneeParentAngularTransport.Enabled)
                return "ARL Hip translation tether serial-chain A/B：v0.2.75 Knee parent transport 仍启用。内部 frozen-stack 依赖未满足。";

            _hipCharacterJointResponseIsolation.SetUpperOnlyTetherEnabled(upperOnly, _report);
            ResetTraceStats();
            UpdateReport();
            return GetHipTranslationTetherSerialChainStatus();
        }

        private string GetHipTranslationTetherSerialChainStatus()
        {
            if (!IsBuilt)
                return "ARL Hip translation tether serial-chain A/B | built=False";
            UpdateJointPresence();
            return _hipCharacterJointResponseIsolation.GetSerialChainStatus(_report);
        }

        private string SetKneeParentAngularTransport(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Knee parent angular transport A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Knee parent angular transport A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Knee parent angular transport A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Knee parent angular transport A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _kneeCharacterJointResponseIsolation.Enabled)
                return "ARL Knee parent angular transport A/B：v0.2.76 B 正在使用该 transport baseline。内部 frozen-stack 依赖未满足。";
            if (!_hipCharacterJointResponseIsolation.Enabled || _hipJoint != null || !_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled)
                return "ARL Knee parent angular transport A/B：v0.2.75 需要 v0.2.74 B baseline。内部 frozen-stack 依赖未满足。";

            _kneeParentAngularTransport.SetEnabled(enabled, _report);
            ResetTraceStats();
            UpdateReport();
            return GetKneeParentAngularTransportStatus();
        }

        private string GetKneeParentAngularTransportStatus()
        {
            if (!IsBuilt)
                return "ARL Knee parent angular transport A/B | built=False";
            _kneeParentAngularTransport.CopyToReport(_report);
            return _kneeParentAngularTransport.GetStatus(_report);
        }

        private string SetKneeCharacterJointResponseIsolation(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Knee CharacterJoint response isolation：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Knee CharacterJoint response isolation：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Knee CharacterJoint response isolation：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Knee CharacterJoint response isolation：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _kneeAnchorAccelerationFeedforward.Enabled)
                return "ARL Knee CharacterJoint response isolation：v0.2.77 B 依赖 v0.2.76 Knee tether。内部 frozen-stack 依赖未满足。";

            if (enabled)
            {
                if (_kneeCharacterJointResponseIsolation.Enabled)
                    return GetKneeCharacterJointResponseIsolationStatus();
                if (!_hipCharacterJointResponseIsolation.Enabled || _hipJoint != null || !_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled || !_kneeParentAngularTransport.Enabled)
                    return "ARL Knee CharacterJoint response isolation：v0.2.76 B 需要完整 v0.2.75 B baseline。内部 frozen-stack 依赖未满足。";
                if (_kneeJoint == null)
                    return "ARL Knee CharacterJoint response isolation：无法进入 B，Knee CharacterJoint 已缺失。";

                _kneeFlexionAxisLowerLocal = _kneeJoint.axis.sqrMagnitude > 0.0001f ? _kneeJoint.axis.normalized : _kneeFlexionAxisLowerLocal;
                _kneeChildAnchorLowerLocal = _kneeJoint.anchor;
                _kneeParentAnchorUpperLocal = _kneeJoint.connectedAnchor;
                CharacterJoint jointToRemove = _kneeJoint;
                _kneeCharacterJointResponseIsolation.SetEnabled(true, _report);
                try { Object.Destroy(jointToRemove); }
                catch (Exception e)
                {
                    _kneeCharacterJointResponseIsolation.SetEnabled(false, _report);
                    _kneeJoint = jointToRemove;
                    return "ARL Knee CharacterJoint response isolation：删除 Knee CharacterJoint 失败：" + e.GetType().Name + "/" + e.Message;
                }
                _kneeJoint = null;
            }
            else
            {
                if (!_kneeCharacterJointResponseIsolation.Enabled)
                    return GetKneeCharacterJointResponseIsolationStatus();
                if (_kneeJoint != null)
                    return "ARL Knee CharacterJoint response isolation：A 恢复前发现意外 Knee CharacterJoint，拒绝重复创建。";

                _kneeCharacterJointResponseIsolation.SetEnabled(false, _report);
                try
                {
                    _kneeJoint = Il2CppComponentFactory.Add<CharacterJoint>(_lowerObject, "LEG_RESTORE_KNEE_CHARACTER_JOINT_V076_A");
                    ConfigureKneeJoint();
                }
                catch (Exception e)
                {
                    _kneeJoint = null;
                    _kneeCharacterJointResponseIsolation.SetEnabled(true, _report);
                    return "ARL Knee CharacterJoint response isolation：恢复 A CharacterJoint 失败；保持 B fail-closed：" + e.GetType().Name + "/" + e.Message;
                }
            }

            ResetBodyRelativeAngularVelocityEstimator();
            ResetMeasuredTransportKinematics();
            ResetTraceStats();
            SampleSourcePoseInternal(false);
            FollowSourceKinematicPreview();
            UpdateReport();
            return GetKneeCharacterJointResponseIsolationStatus();
        }

        private string GetKneeCharacterJointResponseIsolationStatus()
        {
            if (!IsBuilt)
                return "ARL Knee CharacterJoint response isolation | built=False";
            UpdateJointPresence();
            return _kneeCharacterJointResponseIsolation.GetStatus(_report);
        }

        private string SetKneeAnchorAccelerationFeedforward(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Knee anchor acceleration FF A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Knee anchor acceleration FF A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Knee anchor acceleration FF A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Knee anchor acceleration FF A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _kneeFlexionAllocation.Enabled)
                return "ARL Knee anchor acceleration FF A/B：v0.2.78 B 依赖 v0.2.77 baseline。内部 frozen-stack 依赖未满足。";
            if (enabled && (!_kneeCharacterJointResponseIsolation.Enabled || _kneeJoint != null || !_kneeParentAngularTransport.Enabled))
                return "ARL Knee anchor acceleration FF B：需要完整 v0.2.76 B baseline。内部 frozen-stack 依赖未满足。";

            _kneeAnchorAccelerationFeedforward.SetEnabled(enabled, _report);
            ResetTraceStats();
            SampleSourcePoseInternal(false);
            FollowSourceKinematicPreview();
            UpdateReport();
            return GetKneeAnchorAccelerationFeedforwardStatus();
        }

        private string GetKneeAnchorAccelerationFeedforwardStatus()
        {
            if (!IsBuilt)
                return "ARL Knee anchor acceleration FF A/B | built=False";
            UpdateJointPresence();
            return _kneeAnchorAccelerationFeedforward.GetStatus(_report);
        }

        private string SetKneeFlexionFeedbackHeadroomAllocation(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Knee flexion feedback headroom allocation A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Knee flexion feedback headroom allocation A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Knee flexion feedback headroom allocation A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Knee flexion feedback headroom allocation A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _hipAnchorAccelerationFeedforward.Enabled)
                return "ARL Knee flexion feedback headroom allocation A/B：v0.2.79 B 依赖完整 v0.2.78 baseline。内部 frozen-stack 依赖未满足。";
            if (enabled && (!_kneeAnchorAccelerationFeedforward.Enabled || !_kneeCharacterJointResponseIsolation.Enabled || _kneeJoint != null || !_kneeParentAngularTransport.Enabled))
                return "ARL Knee flexion allocation B：需要完整 v0.2.77 B baseline。内部 frozen-stack 依赖未满足。";

            _kneeFlexionAllocation.SetEnabled(enabled, _report);
            ResetTraceStats();
            UpdateReport();
            return GetKneeFlexionFeedbackHeadroomAllocationStatus();
        }

        private string GetKneeFlexionFeedbackHeadroomAllocationStatus()
        {
            if (!IsBuilt)
                return "ARL Knee flexion feedback headroom allocation A/B | built=False";
            _kneeFlexionAllocation.CopyToReport(_report);
            return _kneeFlexionAllocation.GetStatus(_report);
        }

        private string SetHipAnchorAccelerationFeedforward(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip anchor acceleration FF A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip anchor acceleration FF A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip anchor acceleration FF A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip anchor acceleration FF A/B：拒绝切换。请先停止 Knee axis calibration。";
            if (!enabled && _hipAngularActuator.Enabled)
                return "ARL Hip anchor acceleration FF A/B：v0.2.81 B 依赖完整 v0.2.79 baseline。内部 frozen-stack 依赖未满足。";
            if (enabled && (!_hipCharacterJointResponseIsolation.Enabled || _hipJoint != null ||
                            !_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled || !_hipMechanicalParent.UseInertialized ||
                            !_kneeParentAngularTransport.Enabled || !_kneeCharacterJointResponseIsolation.Enabled || _kneeJoint != null ||
                            !_kneeAnchorAccelerationFeedforward.Enabled || !_kneeFlexionAllocation.Enabled))
                return "ARL Hip anchor acceleration FF B：需要完整 v0.2.78 B baseline。内部 frozen-stack 依赖未满足。";

            _hipAnchorAccelerationFeedforward.SetEnabled(enabled, _report);
            ResetTraceStats();
            SampleSourcePoseInternal(false);
            FollowSourceKinematicPreview();
            UpdateReport();
            return GetHipAnchorAccelerationFeedforwardStatus();
        }

        private string GetHipAnchorAccelerationFeedforwardStatus()
        {
            if (!IsBuilt)
                return "ARL Hip anchor acceleration FF A/B | built=False";
            _hipAnchorAccelerationFeedforward.CopyToReport(_report);
            return _hipAnchorAccelerationFeedforward.GetStatus(_report);
        }

        private string SetHipAngularInertiaActuator(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip inertia-space actuator A/B：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip inertia-space actuator A/B：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip inertia-space actuator A/B：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip inertia-space actuator A/B：拒绝切换。请先停止 Knee axis calibration。";

            if (enabled && (!_hipCharacterJointResponseIsolation.Enabled || _hipJoint != null ||
                            !_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled ||
                            !_kneeParentAngularTransport.Enabled || !_kneeCharacterJointResponseIsolation.Enabled || _kneeJoint != null ||
                            !_kneeAnchorAccelerationFeedforward.Enabled || !_kneeFlexionAllocation.Enabled ||
                            !_hipAnchorAccelerationFeedforward.Enabled || !_hipMechanicalParent.UseInertialized ||
                            !_hipMechanicalParentTransport.Enabled || !_hipAnchorLinearTransport.Enabled))
            {
                return "ARL Hip inertia-space actuator B：需要完整 v0.2.79 B baseline。内部 frozen-stack 依赖未满足。";
            }

            _hipAngularActuator.SetEnabled(enabled, _report);
            ResetTraceStats();
            UpdateReport();
            return GetHipAngularInertiaActuatorStatus();
        }

        private string GetHipAngularInertiaActuatorStatus()
        {
            if (!IsBuilt)
                return "ARL Hip inertia-space actuator A/B | built=False";
            return _hipAngularActuator.GetStatus(_report, _upperBody);
        }

        public string SetHipExperimentStack(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip experiment stack：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip experiment stack：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip experiment stack：拒绝切换。请先 arl_trace(off)。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip experiment stack：拒绝切换。请先停止 Knee axis calibration。";

            if (enabled)
            {

                // v0.2.83: stack(on) now means the complete validated post-PCA Single-Leg baseline.
                // Refuse before explicit PCA apply so a failed convenience command cannot leave a silent half-stack.
                if (!_kneeAxisCalibrator.Applied || !_kneeAxisCalibrator.Ready || _kneeAxisMode != "DATA_DRIVEN_CALIBRATED")
                    return "ARL Hip experiment stack：拒绝 ON。请先完成 arl_knee_axis_calibrate(off) 并 arl_knee_axis_apply；PCA apply 成功后本命令才会一次建立完整 v0.2.74-v0.2.81 baseline。";

                // v0.2.88 causal variable is TRUE Healthy Normal parent transport only. stack(on) always restores
                // A/OFF. Dedicated post-frame physical-recovery support remains part of the common physical baseline.

                // v0.2.82 zero-drag was rejected. v0.4.22 removes that unreachable branch;
                // retain the live Upper/Lower profile-drag readback gate without silently repairing drift.
                if (!_legAngularDragContract.ValidateStep(_upperBody, _lowerBody, _profile.AngularDrag, _report))
                    return "ARL Hip experiment stack：Upper/Lower angularDrag 与 profile 不一致，拒绝建立 stack。";

                if (!_hipCharacterJointResponseIsolation.Enabled)
                {
                    SetHipMechanicalParentMode("inertialized");
                    SetHipMechanicalParentTransportFeedforward(true);
                    SetHipAnchorLinearTransport(true);
                }
                if (!_hipMomentumRecoveryBraking.Enabled) SetHipMomentumRecoveryBraking(true);
                if (!_hipVectorCaptureDamping.Enabled) SetHipVectorCaptureDamping(true);
                if (!_hipVectorCaptureDamping.SymmetricRadialEnabled) SetHipSymmetricRadialCapture(true);
                if (!_hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled) SetHipHighErrorAwayBrakeFloor(true);
                if (_hipControlFrame.ReferenceLagGuardRebaseEnabled) SetHipReferenceLagGuardRebase(false);
                if (!_hipSourceReferenceShockAbsorber.Enabled) SetHipSourceReferenceShockAbsorber(true);
                if (!_hipCompositeReferenceShockAbsorber.Enabled) SetHipCompositeReferenceShockAbsorber(true);
                if (!_hipCharacterJointResponseIsolation.Enabled) SetHipCharacterJointResponseIsolation(true);

                // Validated post-PCA dependency chain. The component setters below are private and exist only
                // to construct/rollback the frozen stack atomically; historical A/B replay lives in archived versions.
                if (!_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled) SetHipTranslationTetherSerialChain(true);
                if (!_kneeParentAngularTransport.Enabled) SetKneeParentAngularTransport(true);
                if (!_kneeCharacterJointResponseIsolation.Enabled) SetKneeCharacterJointResponseIsolation(true);
                if (!_kneeAnchorAccelerationFeedforward.Enabled) SetKneeAnchorAccelerationFeedforward(true);
                if (!_kneeFlexionAllocation.Enabled) SetKneeFlexionFeedbackHeadroomAllocation(true);
                if (!_hipAnchorAccelerationFeedforward.Enabled) SetHipAnchorAccelerationFeedforward(true);
                if (!_hipAngularActuator.Enabled) SetHipAngularInertiaActuator(true);

                if (!IsCompletePostPcaStackReady())
                {
                    // Fail closed: a convenience command must never leave a hidden partial stack.
                    SetHipExperimentStack(false);
                    return "ARL Hip experiment stack：ON 未能建立完整 post-PCA baseline；已按依赖逆序回滚，未保留半配置状态。请查看 arl_hip_stack_status。";
                }
            }
            else
            {
                if (_hipAngularActuator.Enabled) SetHipAngularInertiaActuator(false);
                if (_hipAnchorAccelerationFeedforward.Enabled) SetHipAnchorAccelerationFeedforward(false);
                if (_kneeFlexionAllocation.Enabled) SetKneeFlexionFeedbackHeadroomAllocation(false);
                if (_kneeAnchorAccelerationFeedforward.Enabled) SetKneeAnchorAccelerationFeedforward(false);
                if (_kneeCharacterJointResponseIsolation.Enabled) SetKneeCharacterJointResponseIsolation(false);
                if (_kneeParentAngularTransport.Enabled) SetKneeParentAngularTransport(false);
                if (_hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled) SetHipTranslationTetherSerialChain(false);
                if (_hipCharacterJointResponseIsolation.Enabled) SetHipCharacterJointResponseIsolation(false);
                if (_hipCompositeReferenceShockAbsorber.Enabled) SetHipCompositeReferenceShockAbsorber(false);
                if (_hipSourceReferenceShockAbsorber.Enabled) SetHipSourceReferenceShockAbsorber(false);
                if (!_hipControlFrame.ReferenceLagGuardRebaseEnabled) SetHipReferenceLagGuardRebase(true);
                if (_hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled) SetHipHighErrorAwayBrakeFloor(false);
                if (_hipVectorCaptureDamping.SymmetricRadialEnabled) SetHipSymmetricRadialCapture(false);
                if (_hipVectorCaptureDamping.Enabled) SetHipVectorCaptureDamping(false);
                if (_hipMomentumRecoveryBraking.Enabled) SetHipMomentumRecoveryBraking(false);
            }

            ResetTraceStats();
            UpdateReport();
            return GetHipExperimentStackStatus();
        }

        private bool IsCompletePostPcaStackReady()
        {
            return _kneeAxisCalibrator.Applied &&
                   _kneeAxisCalibrator.Ready &&
                   _kneeAxisMode == "DATA_DRIVEN_CALIBRATED" &&
                   _hipMomentumRecoveryBraking.Enabled &&
                   _hipVectorCaptureDamping.Enabled &&
                   _hipVectorCaptureDamping.SymmetricRadialEnabled &&
                   _hipVectorCaptureDamping.HighErrorAwayBrakeFloorEnabled &&
                   !_hipControlFrame.ReferenceLagGuardRebaseEnabled &&
                   _hipSourceReferenceShockAbsorber.Enabled &&
                   _hipCompositeReferenceShockAbsorber.Enabled &&
                   _hipCharacterJointResponseIsolation.Enabled &&
                   _hipJoint == null &&
                   _hipCharacterJointResponseIsolation.UpperOnlyTetherEnabled &&
                   _kneeParentAngularTransport.Enabled &&
                   _kneeCharacterJointResponseIsolation.Enabled &&
                   _kneeJoint == null &&
                   _kneeAnchorAccelerationFeedforward.Enabled &&
                   _kneeFlexionAllocation.Enabled &&
                   _hipAnchorAccelerationFeedforward.Enabled &&
                   _hipAngularActuator.Enabled &&
                   _upperBody != null && Mathf.Abs(_upperBody.angularDrag - _profile.AngularDrag) <= 0.0001f &&
                   _lowerBody != null && Mathf.Abs(_lowerBody.angularDrag - _profile.AngularDrag) <= 0.0001f &&
                   _hipMechanicalParent.UseInertialized &&
                   _hipMechanicalParentTransport.Enabled &&
                   _hipAnchorLinearTransport.Enabled;
        }
        public string GetHipExperimentStackStatus()
        {
            if (!IsBuilt)
                return "ARL Hip experiment stack | built=False";
            bool ready = IsCompletePostPcaStackReady();
            return "ARL Hip experiment stack | ready=" + ready +
                   " | pcaApplied=" + _kneeAxisCalibrator.Applied +
                   " | topology=NO_HIP_JOINT+UPPER_ONLY_HIP_TETHER+NO_KNEE_JOINT+KNEE_TETHER" +
                   " | parent=INERTIALIZED" +
                   " | strictTransport=" + (_hipMechanicalParentTransport.Enabled ? "V048_ON" : "OFF") +
                   " | healthyNormalTransport=" + _hipMechanicalParentTransport.NormalTransportArm +
                   " | recoverySupport=" + (_hipMechanicalParentTransport.RecoveryTransportEnabled ? "DEDICATED_ON" : "OFF") +
                   " | actuator=" + _hipAngularActuator.Arm +
                   " | angularDrag=" + F(_upperBody != null ? _upperBody.angularDrag : 0f) +
                   " | hipMaxA=" + F(_profile.LegHipMaxAngularAcceleration) + "/" + F(_profile.LegHipSourceTurnMaxAngularAcceleration) + "/" + F(_profile.LegHipRecoveryMaxAngularAcceleration) +
                   " | singleLegFreeze=" + SingleLegFreezeReadinessCertification.Mode;
        }

        private string SetHipMechanicalParentMode(string mode)
        {
            if (!IsBuilt)
                return "ARL Hip mechanical parent：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip mechanical parent：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip mechanical parent：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip mechanical parent：拒绝切换。请先停止 Knee axis calibration。";
            if (_hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip mechanical parent：v0.2.72 composite absorber B 固定 inertialized parent baseline。内部 frozen-stack 依赖未满足。";

            string normalized = (mode ?? string.Empty).Trim();
            bool useInertialized;
            if (normalized.Equals("real", StringComparison.OrdinalIgnoreCase) ||
                normalized.Equals("raw", StringComparison.OrdinalIgnoreCase) ||
                normalized.Equals("a", StringComparison.OrdinalIgnoreCase) ||
                normalized.Equals(HipMechanicalParentProxy.RealPelvisMode, StringComparison.OrdinalIgnoreCase))
            {
                useInertialized = false;
            }
            else if (normalized.Equals("inertialized", StringComparison.OrdinalIgnoreCase) ||
                     normalized.Equals("proxy", StringComparison.OrdinalIgnoreCase) ||
                     normalized.Equals("b", StringComparison.OrdinalIgnoreCase) ||
                     normalized.Equals(HipMechanicalParentProxy.InertializedProxyMode, StringComparison.OrdinalIgnoreCase))
            {
                useInertialized = true;
            }
            else
            {
                return "ARL Hip mechanical parent：无效内部模式参数；仅接受 real/inertialized。";
            }

            _hipMechanicalParent.TrySetMode(useInertialized);
            SampleSourcePoseInternal(false);
            _hipControlFrame.Reset(_pelvisBody.rotation, Vector3.zero, _profile, _report);
            _hipMechanicalParent.ResetPose(GetRawPelvisHipAnchorWorld(), _pelvisBody.rotation);
            ApplyHipMechanicalParentConnection();
            FollowSourceKinematicPreview();
            _hipMechanicalParentTransport.ResetDiagnostics(_report);
            _hipAnchorLinearTransport.ResetDiagnostics(_report);
            UpdateReport();
            return GetHipMechanicalParentStatus();
        }

        private string GetHipMechanicalParentStatus()
        {
            if (!IsBuilt)
                return "ARL Hip mechanical parent | built=False";
            _hipMechanicalParent.CopyToReport(_report, _hipJoint);
            _hipMechanicalParentTransport.CopyToReport(_report);
            return "ARL Hip mechanical parent | " + _hipMechanicalParent.GetKey(_hipJoint);
        }

        private string SetHipMechanicalParentTransportFeedforward(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip parent transport FF：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip parent transport FF：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip parent transport FF：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip parent transport FF：拒绝切换。请先停止 Knee axis calibration。";
            if (_hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip parent transport FF：v0.2.72 composite absorber B 固定 production transport baseline。内部 frozen-stack 依赖未满足。";

            _hipMechanicalParentTransport.TrySetEnabled(enabled);
            _hipMechanicalParentTransport.ResetDiagnostics(_report);
            _hipAnchorLinearTransport.ResetDiagnostics(_report);
            UpdateReport();
            return GetHipMechanicalParentTransportStatus();
        }

        private string GetHipMechanicalParentTransportStatus()
        {
            if (!IsBuilt)
                return "ARL Hip parent transport FF | built=False";
            _hipMechanicalParentTransport.CopyToReport(_report);
            return "ARL Hip parent transport FF | " + _report.HipMechanicalParentTransportKey;
        }

        public string GetSingleLegFreezeStatus()
        {
            if (!IsBuilt)
                return "ARL v0.2.122 Single-Leg freeze readiness | built=False";
            RefreshSingleLegFreezeReadiness();
            return _singleLegFreezeCertification.GetSummary(_report);
        }

        // v0.3.28.1 maintenance: refresh the cached freeze verdict without producing a console/log side effect.
        // Bilateral status snapshots use this after trace finalization so they cannot report stale per-leg readiness
        // merely because arl_bilateral_status was queried before arl_leg_freeze_status.
        internal void RefreshSingleLegFreezeReadiness()
        {
            if (!IsBuilt)
                return;
            UpdateReport();
            EvaluateSingleLegFreezeReadiness(!_traceEnabled);
        }

        private bool IsFrozenProductionContractReady(out string key)
        {
            bool stack = IsCompletePostPcaStackReady();
            bool targetW = Mathf.Abs(_profile.TargetVelocityBlend - 0.50f) <= 0.0001f;
            bool healthyNormalOff = !_hipMechanicalParentTransport.NormalTransportEnabled;
            bool recoverySupport = _hipMechanicalParentTransport.RecoveryTransportEnabled;
            bool drag = Mathf.Abs(_profile.AngularDrag - 3f) <= 0.0001f &&
                        _upperBody != null && Mathf.Abs(_upperBody.angularDrag - 3f) <= 0.0001f &&
                        _lowerBody != null && Mathf.Abs(_lowerBody.angularDrag - 3f) <= 0.0001f;
            bool hipPdAuthority = Mathf.Abs(_profile.LegHipAngularSpring - 55f) <= 0.0001f &&
                                  Mathf.Abs(_profile.LegHipAngularDamper - 12f) <= 0.0001f &&
                                  Mathf.Abs(_profile.LegHipMaxAngularAcceleration - 40f) <= 0.0001f &&
                                  Mathf.Abs(_profile.LegHipSourceTurnMaxAngularAcceleration - 80f) <= 0.0001f &&
                                  Mathf.Abs(_profile.LegHipRecoveryMaxAngularAcceleration - 80f) <= 0.0001f;
            bool kneePd = Mathf.Abs(_profile.LegKneeAngularSpring - 45f) <= 0.0001f &&
                          Mathf.Abs(_profile.LegKneeAngularDamper - 9f) <= 0.0001f &&
                          Mathf.Abs(_profile.LegKneeMaxAngularAcceleration - 40f) <= 0.0001f;
            bool kneeSoft = Mathf.Abs(_profile.LegKneeSoftOffAxisAngularSpring - 30f) <= 0.0001f &&
                            Mathf.Abs(_profile.LegKneeSoftOffAxisAngularDamper - 8f) <= 0.0001f &&
                            Mathf.Abs(_profile.LegKneeSoftOffAxisMaxAngularAcceleration - 20f) <= 0.0001f &&
                            Mathf.Abs(_profile.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration - 40f) <= 0.0001f &&
                            Mathf.Abs(_profile.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration - 40f) <= 0.0001f;
            bool angularAccelFf = _profile.LegAngularAccelerationFeedforward &&
                                  Mathf.Abs(_profile.LegAngularAccelerationFeedforwardGain - 1f) <= 0.0001f &&
                                  Mathf.Abs(_profile.LegAngularAccelerationFeedforwardMaxAcceleration - 20f) <= 0.0001f &&
                                  _legAngularAccelerationFeedforward.Enabled;
            bool safeEnvironment = !_gravityEnabled && !_collidersEnabled;
            bool ankleDriveProduction = _footFoundation == null || _footFoundation.AnkleDriveProductionArm;
            bool ankleConstraintProduction = _footFoundation == null || _footFoundation.AnkleConstraintProductionArm;
            bool ankleTransportProduction = _footFoundation == null || _footFoundation.AnkleTransportFeedforwardEnabled;
            bool ankleAngularTransportProduction = _footFoundation == null || _footFoundation.AnkleAngularTransportFeedforwardEnabled;
            bool ankleCommonModeCompProduction = _footFoundation == null || _footFoundation.AnkleAngularTransportProductionReady;
            bool ankleTargetAProduction = _footFoundation == null || _footFoundation.AnkleTargetAccelerationFeedforwardEnabled;
            // v0.3.73 promotes the previously-qualified conditional MaxA80 capture law into the frozen free-space production contract.
            bool ankleCaptureAuthorityProduction = _footFoundation == null || _footFoundation.TranslationCaptureAuthorityProduction;
            bool production = stack && targetW && healthyNormalOff && recoverySupport && drag && hipPdAuthority && kneePd && kneeSoft && angularAccelFf && safeEnvironment && ankleDriveProduction && ankleConstraintProduction && ankleTransportProduction && ankleAngularTransportProduction && ankleCommonModeCompProduction && ankleTargetAProduction && ankleCaptureAuthorityProduction;
            key = "stack=" + stack +
                  "|targetW_alpha0P50=" + targetW +
                  "|healthyNormalTransportOff=" + healthyNormalOff +
                  "|dedicatedRecoveryOn=" + recoverySupport +
                  "|drag3=" + drag +
                  "|hipPD55_12_MaxA40_80_80=" + hipPdAuthority +
                  "|kneePD45_9_40=" + kneePd +
                  "|kneeSoft30_8_20_40_40=" + kneeSoft +
                  "|angularAccelFF1x20=" + angularAccelFf +
                  "|gravityOff_collidersOff=" + safeEnvironment +
                  "|footV058AnkleDriveB=" + ankleDriveProduction +
                  "|ankleConstraintV063B=" + ankleConstraintProduction +
                  "|ankleTransportV064B=" + ankleTransportProduction +
                  "|ankleAngularTransportV065B=" + ankleAngularTransportProduction +
                  "|ankleCommonModeDragCompV078Production=" + ankleCommonModeCompProduction +
                  "|ankleTargetAFFV068Conditioned=" + ankleTargetAProduction +
                  "|ankleCaptureAuthorityV073ProductionConditional80=" + ankleCaptureAuthorityProduction;
            return production;
        }

        private string EvaluateSingleLegFreezeReadiness(bool traceFinalized)
        {
            string contractKey;
            bool production = IsFrozenProductionContractReady(out contractKey);
            bool calibration = _kneeAxisCalibrator.Applied && _kneeAxisCalibrator.Ready && _kneeAxisMode == "DATA_DRIVEN_CALIBRATED";
            bool inertia = _rigidbodyMassProperties.BuildFoundationValidated;
            return _singleLegFreezeCertification.Evaluate(_report, _torso != null ? _torso.Report : null, traceFinalized, production, contractKey, calibration, inertia);
        }

        private string SetHipAnchorLinearTransport(bool enabled)
        {
            if (!IsBuilt)
                return "ARL Hip anchor linear transport：单腿链尚未创建。";
            if (_driveEnabled)
                return "ARL Hip anchor linear transport：拒绝切换。请先 arl_leg_drive(off)。";
            if (_traceEnabled)
                return "ARL Hip anchor linear transport：拒绝切换。请先 arl_trace(off)，避免 A/B 统计混合。";
            if (_kneeAxisCalibrator.Collecting)
                return "ARL Hip anchor linear transport：拒绝切换。请先停止 Knee axis calibration。";
            if (_hipCompositeReferenceShockAbsorber.Enabled)
                return "ARL Hip anchor linear transport：v0.2.72 composite absorber B 固定 production linear baseline。内部 frozen-stack 依赖未满足。";

            _hipAnchorLinearTransport.TrySetEnabled(enabled);
            _hipAnchorLinearTransport.ResetDiagnostics(_report);
            UpdateReport();
            return GetHipAnchorLinearTransportStatus();
        }

        private string GetHipAnchorLinearTransportStatus()
        {
            if (!IsBuilt)
                return "ARL Hip anchor linear transport | built=False";
            _hipAnchorLinearTransport.CopyToReport(_report);
            return "ARL Hip anchor linear transport | " + _report.HipAnchorLinearTransportKey;
        }
    }
}
