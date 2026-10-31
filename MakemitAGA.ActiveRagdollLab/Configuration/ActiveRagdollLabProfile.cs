using System;
using System.Globalization;
using System.IO;
using BepInEx;
using BepInEx.Logging;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Configuration
{
    internal sealed class ActiveRagdollLabProfile
    {
        public const string FileName = "MakemitAGA.ActiveRagdollLab.Profile.txt";

        public string TargetName = "MitaPerson Mita";
        public string PelvisBonePath = "Armature/Hips";
        public string SpineBonePath = "Armature/Hips/Spine";
        public string ChestBonePath = "Armature/Hips/Spine/Chest";

        public float PelvisMass = 8f;
        public float ChestMass = 12f;
        public Vector3 PelvisSizeScale = new Vector3(0.70f, 0.34f, 0.52f);
        public Vector3 ChestSizeScale = new Vector3(0.86f, 0.68f, 0.56f);
        public float MinBoxDimension = 0.10f;
        public float MaxBoxDimension = 0.55f;

        // Frozen Two-Body torso baseline. These values are independent from later leg/feet work.
        // MaxLinearVelocity is also used by the Render->Physics pose bridge and safety clamps.
        public float MaxLinearVelocity = 6f;
        public float AngularSpring = 55f;
        public float AngularDamper = 12f;
        public float MaxAngularAcceleration = 40f;
        public float AngularDrag = 3f;
        public float MaxAngularVelocity = 18f;
        public float DriveDisableDistance = 0.80f;
        public float TargetVelocityBlend = 0.50f;
        public float TraceIntervalSeconds = 0.50f;
        // Diagnostic log-volume controls only; never enter physics control.
        public bool TraceVerboseSnapshots = false;
        public bool SourceTurnVerboseContextLogging = false;

        // Motion-phase classifier thresholds: diagnostics/freeze validation only.
        public float MotionIdleSpeedThreshold = 0.15f;
        public float MotionSteadyMinSpeed = 0.50f;
        public float MotionLinearAccelerationThreshold = 3.00f;
        public float MotionTurnAngularVelocityThreshold = 1.20f;
        public float MotionTurnAngularAccelerationThreshold = 12.0f;
        public float MotionTransitionHoldSeconds = 0.30f;

        // Source-turn attribution thresholds: diagnostics only.
        public float SourceTurnEventRotationDeltaDegrees = 20.0f;
        public float SourceTurnEventAngularSpeedThreshold = 4.0f;
        public float SourceTurnEventAngularAccelerationThreshold = 30.0f;
        public float SourceTurnContextPreSeconds = 0.50f;
        public float SourceTurnContextPostSeconds = 0.75f;

        // Render -> Physics Pose Bridge parameters for the frozen torso command path.
        public float PoseBridgeMaxExtrapolationSeconds = 0.10f;
        public float PoseBridgePositionCorrectionGain = 6.0f;
        public float PoseBridgeMaxPositionCorrectionSpeed = 0.75f;
        public float PoseBridgeRotationCorrectionGain = 6.0f;
        public float PoseBridgeMaxRotationCorrectionAngularSpeed = 3.0f;

        // Frozen torso parent-motion feedforward; transports parent motion without world-position error feedback.
        public bool ParentMotionFeedforward = true;
        public float ParentMotionFeedforwardGain = 12f;
        public float MaxParentMotionFeedforwardAcceleration = 20f;

        // WIDE_ISOLATION keeps broad diagnostic limits; anatomical fields remain for a future explicit mode switch.
        public bool WideJointLimitIsolation = true;
        public float WideJointLimitDegrees = 170f;

        public float ChestLowTwistLimitDegrees = -35f;
        public float ChestHighTwistLimitDegrees = 35f;
        public float ChestSwing1LimitDegrees = 45f;
        public float ChestSwing2LimitDegrees = 35f;
        public float JointAnchorSafetyDistance = 0.15f;

        public bool DefaultColliders = false;
        public bool DefaultGravity = false;
        public float PreviewAlpha = 0.28f;

        // Frozen bilateral-leg foundation. Torso parameters above remain independent.
        public bool SingleLegEnabled = true;
        // true replicates the same frozen leg controller to the opposite side. singleLegSide remains the primary side.
        public bool BilateralLegEnabled = true;
        public string SingleLegSide = "Left";
        // Optional explicit primary-leg paths. Leave all three empty for the validated automatic resolver.
        public string SingleLegUpperBonePath = string.Empty;
        public string SingleLegLowerBonePath = string.Empty;
        public string SingleLegFootBonePath = string.Empty;
        public float UpperLegMass = 7.0f;
        public float LowerLegMass = 4.0f;
        // v0.4.3: no profile value changes. v0.3.78 Foot/Ankle physics and v0.4.1.x contact boundary stay frozen; only read-only support-normal reaction attribution is added.
        // B adds one source-bone-driven relative Ankle PD realized as an equal/opposite Lower<->Foot torque pair; constants remain code-frozen, not Profile sweep knobs.
        // The calibrated vertical offset is frozen for the run; Gravity/Collider/contact/GSP remain OFF. Footwear state remains intentionally unresolved.
        public bool FootAnkleFoundationEnabled = true;
        public float FootMass = 1.0f;
        public float LegSegmentThickness = 0.10f;
        public float LegHipAngularSpring = 55f;
        public float LegHipAngularDamper = 12f;
        public float LegHipMaxAngularAcceleration = 40f;
        // Phase-aware Hip authority: baseline 40; bounded Source/Recovery ceilings 80/80. Kp/Kd remain unchanged.
        public float LegHipSourceTurnMaxAngularAcceleration = 80f;
        public float LegHipRecoveryMaxAngularAcceleration = 80f;
        public float LegHipRecoveryExitErrorDegrees = 10f;
        public float LegHipRecoveryStableHoldSeconds = 0.15f;
        public float LegHipRecoveryMaxSeconds = 2.0f;
        // Continuous Hip tracking-health hysteresis; re-entry suppresses normal-only aids without raising MaxA.
        public float LegHipTrackingHealthEnterErrorDegrees = 15f;
        public float LegHipTrackingHealthEnterHoldSeconds = 0.10f;
        public float LegHipTrackingHealthImmediateErrorDegrees = 30f;
        // Bounded inertialized Hip parent frame during Source/Recovery discontinuities.
        public bool LegHipControlFrameInertialization = true;
        public float LegHipControlFrameSourceMaxAngularSpeed = 6f;
        public float LegHipControlFrameSourceMaxAngularAcceleration = 60f;
        public float LegHipControlFrameRecoveryMaxAngularSpeed = 8f;
        public float LegHipControlFrameRecoveryMaxAngularAcceleration = 80f;
        // Hard bound on real-vs-virtual parent orientation separation.
        public float LegHipControlFrameMaxLagDegrees = 25f;
        public float LegHipControlFrameRecoveryExitAngleDegrees = 1.5f;
        public float LegHipControlFrameRecoveryExitAngularSpeed = 0.5f;
        public float LegHipControlFrameRecoveryStableHoldSeconds = 0.10f;
        public float LegHipControlFrameRecoveryMaxSeconds = 1.5f;
        public float LegKneeAngularSpring = 45f;
        public float LegKneeAngularDamper = 9f;
        public float LegKneeMaxAngularAcceleration = 40f;
        // Frozen Knee backend: flexion PD plus software off-axis PD; physical Knee joint is removed by the post-PCA stack.
        public float LegKneeSoftOffAxisAngularSpring = 30f;
        public float LegKneeSoftOffAxisAngularDamper = 8f;
        public float LegKneeSoftOffAxisMaxAngularAcceleration = 20f;
        // Phase-aware Knee off-axis authority: baseline 20; bounded Source/Recovery ceilings 40/40.
        public float LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration = 40f;
        public float LegKneeSoftOffAxisRecoveryMaxAngularAcceleration = 40f;
        public float LegKneeSoftOffAxisRecoveryExitErrorDegrees = 5f;
        public float LegKneeSoftOffAxisRecoveryStableHoldSeconds = 0.15f;
        public float LegKneeSoftOffAxisRecoveryMaxSeconds = 2.0f;
        public float LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees = 10f;
        public float LegKneeSoftOffAxisTrackingHealthEnterHoldSeconds = 0.10f;
        public float LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees = 25f;
        public float LegDriveDisableDistance = 0.65f;

        // Validated target-relative angular-acceleration feedforward: production ON / gain=1 / bound=20.
        // Source/Recovery/control-frame/stale-sample suppression remains runtime-owned.
        public bool LegAngularAccelerationFeedforward = true;
        public float LegAngularAccelerationFeedforwardGain = 1.0f;
        public float LegAngularAccelerationFeedforwardMaxAcceleration = 20.0f;
        public float LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds = 0.10f;

        public string SourcePath;
        public bool LoadedFromFile;
        public string LoadMessage;

        public static string ConfigPath => Path.Combine(Paths.ConfigPath, FileName);

        public static string EnsureTemplate(ManualLogSource logger)
        {
            string path = ConfigPath;
            try
            {
                if (File.Exists(path))
                    return "ARL 配置已存在：" + path;

                Directory.CreateDirectory(Path.GetDirectoryName(path));
                File.WriteAllText(path, GetTemplateText());
                logger?.LogInfo("[ARL][PROFILE] template-created | path=" + path);
                return "已创建 ARL 配置模板：" + path;
            }
            catch (Exception e)
            {
                logger?.LogWarning("[ARL][PROFILE] template-create-failed | " + e);
                return "创建 ARL 配置失败：" + e.GetType().Name + " / " + e.Message;
            }
        }

        public static ActiveRagdollLabProfile Load(ManualLogSource logger)
        {
            ActiveRagdollLabProfile profile = new ActiveRagdollLabProfile
            {
                SourcePath = ConfigPath,
                LoadedFromFile = false,
                LoadMessage = "using-built-in-safe-defaults"
            };

            EnsureTemplate(logger);

            try
            {
                if (!File.Exists(ConfigPath))
                    return profile;

                string[] lines = File.ReadAllLines(ConfigPath);
                for (int i = 0; i < lines.Length; i++)
                {
                    string raw = lines[i] ?? string.Empty;
                    string line = raw.Trim();
                    if (line.Length == 0 || line.StartsWith("#", StringComparison.Ordinal))
                        continue;

                    int equals = line.IndexOf('=');
                    if (equals <= 0)
                        continue;

                    string key = line.Substring(0, equals).Trim();
                    string value = line.Substring(equals + 1).Trim();
                    Apply(profile, key, value);
                }

                ValidateAndClamp(profile);
                profile.LoadedFromFile = true;
                profile.LoadMessage = "loaded";
                logger?.LogInfo(
                    "[ARL][PROFILE] reloaded | path=" + ConfigPath +
                    " | target=" + profile.TargetName +
                    " | pelvis=" + profile.PelvisBonePath +
                    " | spine=" + profile.SpineBonePath +
                    " | chest=" + profile.ChestBonePath +
                    " | driveBackend=RELATIVE_JOINT_PD_PLUS_PARENT_FEEDFORWARD" +
                    " | diagnostics=SOURCE_AWARE_FREEZE_VALIDATION+SOURCE_TURN_DISCONTINUITY_ATTRIBUTION" +
                    " | parentFeedforward=" + profile.ParentMotionFeedforward +
                    " | jointLimitMode=" + (profile.WideJointLimitIsolation ? "WIDE_ISOLATION" : "ANATOMICAL") +
                    " | singleLeg=" + profile.SingleLegEnabled + "/" + profile.SingleLegSide +
                " | bilateralLeg=" + profile.BilateralLegEnabled);
            }
            catch (Exception e)
            {
                profile.LoadMessage = "load-failed:" + e.GetType().Name + "/" + e.Message;
                logger?.LogWarning("[ARL][PROFILE] load-failed | " + e);
            }

            return profile;
        }

        public string GetStatusText()
        {
            return
                "ARL profile | loaded=" + LoadedFromFile +
                " | source=" + SourcePath +
                " | target=" + TargetName +
                " | pelvis=" + PelvisBonePath +
                " | spine=" + SpineBonePath +
                " | chest=" + ChestBonePath +
                " | masses=" + F(PelvisMass) + "/" + F(ChestMass) +
                " | pelvisScale=" + V(PelvisSizeScale) +
                " | chestScale=" + V(ChestSizeScale) +
                " | boxClamp=" + F(MinBoxDimension) + ".." + F(MaxBoxDimension) +
                " | driveBackend=RELATIVE_JOINT_PD_PLUS_PARENT_FEEDFORWARD" +
                " | angularVelocityMode=RENDER_SAMPLED_TARGET_RELATIVE_PLUS_FIXED_SETTLED_BODY_DELTA" +
                " | drivePhaseSync=FIXEDUPDATE_PARENT_PLUS_RENDER_SAMPLE_HOLD_VELOCITY_AND_POSE_BRIDGE_CONSISTENT_PARENT_FF" +
                " | poseBridgeMode=RENDER_POSE_VELOCITY_PREDICT_CORRECT" +
                " | parentFFReference=POSE_BRIDGE_COMMAND(frozen)" +
                " | parentFeedforward=" + ParentMotionFeedforward +
                " | parentFF=" + F(ParentMotionFeedforwardGain) + "/" + F(MaxParentMotionFeedforwardAcceleration) +
                " | poseBridge=" + F(PoseBridgeMaxExtrapolationSeconds) + "/" + F(PoseBridgePositionCorrectionGain) + "/" + F(PoseBridgeMaxPositionCorrectionSpeed) + "/" + F(PoseBridgeRotationCorrectionGain) + "/" + F(PoseBridgeMaxRotationCorrectionAngularSpeed) +
                " | jointLimitMode=" + (WideJointLimitIsolation ? "WIDE_ISOLATION" : "ANATOMICAL") +
                " | wideJointLimit=" + F(WideJointLimitDegrees) +
                " | angularPD=" + F(AngularSpring) + "/" + F(AngularDamper) + "/" + F(MaxAngularAcceleration) +
                " | maxLinearVelocity=" + F(MaxLinearVelocity) +
                " | maxAngularVelocity=" + F(MaxAngularVelocity) +
                " | driveDisableDistance=" + F(DriveDisableDistance) +
                " | targetVelocityBlend=" + F(TargetVelocityBlend) +
                " | traceInterval=" + F(TraceIntervalSeconds) +
                " | traceLogMode=" + (TraceVerboseSnapshots ? "VERBOSE" : "COMPACT") +
                " | sourceTurnContextLog=" + (SourceTurnVerboseContextLogging ? "VERBOSE" : "SUMMARY_ONLY") +
                " | motionThresholds=" + F(MotionIdleSpeedThreshold) + "/" + F(MotionSteadyMinSpeed) + "/" + F(MotionLinearAccelerationThreshold) + "/" + F(MotionTurnAngularVelocityThreshold) + "/" + F(MotionTurnAngularAccelerationThreshold) + "/hold=" + F(MotionTransitionHoldSeconds) +
                " | sourceTurnEventThresholds=rotDelta" + F(SourceTurnEventRotationDeltaDegrees) + "/rawW" + F(SourceTurnEventAngularSpeedThreshold) + "/rawA" + F(SourceTurnEventAngularAccelerationThreshold) + "/context=" + F(SourceTurnContextPreSeconds) + "+" + F(SourceTurnContextPostSeconds) +
                " | freezeValidation=TWO_BODY_FREEZE_POLICY_V0_1_16_DRIVE_FAIL_FAST_AND_HOLD_ROBUSTNESS" +
                " | chestLimits=" + F(ChestLowTwistLimitDegrees) + ".." + F(ChestHighTwistLimitDegrees) +
                "/" + F(ChestSwing1LimitDegrees) + "/" + F(ChestSwing2LimitDegrees) +
                " | jointAnchorSafety=" + F(JointAnchorSafetyDistance) +
                " | gravity=" + DefaultGravity +
                " | colliders=" + DefaultColliders +
                " | singleLeg=" + SingleLegEnabled + "/" + SingleLegSide +
                " | bilateralLeg=" + BilateralLegEnabled +
                " | legExplicitPaths=" + (string.IsNullOrWhiteSpace(SingleLegUpperBonePath) ? "AUTO" : "EXPLICIT") +
                " | legMass=" + F(UpperLegMass) + "/" + F(LowerLegMass) +
                " | footFoundation=" + FootAnkleFoundationEnabled + "/V0358_SOURCE_BONE_RELATIVE_ANKLE_PD+V0363_NO_JOINT_TETHER+V0364_LINEAR_PARENT_TRANSPORT+V0378_DRAG_COMPENSATED_COMMON_MODE_ANGULAR_TRANSPORT_PRODUCTION+V0368_PERSISTENCE_TARGET_A_FF+V0373_CONDITIONAL_CAPTURE/mass=" + F(FootMass) +
                " | legHipPD=" + F(LegHipAngularSpring) + "/" + F(LegHipAngularDamper) + "/baseMaxA=" + F(LegHipMaxAngularAcceleration) + "/sourceMaxA=" + F(LegHipSourceTurnMaxAngularAcceleration) + "/recoveryMaxA=" + F(LegHipRecoveryMaxAngularAcceleration) +
                "/recoveryExit=" + F(LegHipRecoveryExitErrorDegrees) + "deg/hold=" + F(LegHipRecoveryStableHoldSeconds) + "s/max=" + F(LegHipRecoveryMaxSeconds) + "s" +
                "/healthReentry=" + F(LegHipTrackingHealthEnterErrorDegrees) + "deg/" + F(LegHipTrackingHealthEnterHoldSeconds) + "s/immediate=" + F(LegHipTrackingHealthImmediateErrorDegrees) + "deg" +
                "/governorLead(source/recovery)=" + F(LegHipRecoveryExitErrorDegrees) + "/" + F(LegHipRecoveryExitErrorDegrees) + "deg" +
                " | hipControlFrame=" + (LegHipControlFrameInertialization ? "INERTIALIZED_SOURCE_TURN" : "RAW_PARENT_FRAME") +
                "/sourceW-A=" + F(LegHipControlFrameSourceMaxAngularSpeed) + "/" + F(LegHipControlFrameSourceMaxAngularAcceleration) +
                "/recoveryW-A=" + F(LegHipControlFrameRecoveryMaxAngularSpeed) + "/" + F(LegHipControlFrameRecoveryMaxAngularAcceleration) +
                "/maxLag=" + F(LegHipControlFrameMaxLagDegrees) + "deg" +
                " | legKneePD=" + F(LegKneeAngularSpring) + "/" + F(LegKneeAngularDamper) + "/" + F(LegKneeMaxAngularAcceleration) +
                " | legKneeSoftOffAxisPD=" + F(LegKneeSoftOffAxisAngularSpring) + "/" + F(LegKneeSoftOffAxisAngularDamper) + "/baseMaxA=" + F(LegKneeSoftOffAxisMaxAngularAcceleration) +
                "/sourceMaxA=" + F(LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration) + "/recoveryMaxA=" + F(LegKneeSoftOffAxisRecoveryMaxAngularAcceleration) +
                "/recoveryExit=" + F(LegKneeSoftOffAxisRecoveryExitErrorDegrees) + "deg/hold=" + F(LegKneeSoftOffAxisRecoveryStableHoldSeconds) + "s/max=" + F(LegKneeSoftOffAxisRecoveryMaxSeconds) + "s" +
                "/healthReentry=" + F(LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees) + "deg/" + F(LegKneeSoftOffAxisTrackingHealthEnterHoldSeconds) + "s/immediate=" + F(LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees) + "deg" +
                " | legKneeBackend=WIDE_CHARACTER_JOINT_PLUS_SOFTWARE_OFFAXIS_PD" +
                " | legAngularAccelerationFF=" + LegAngularAccelerationFeedforward + "/" + F(LegAngularAccelerationFeedforwardGain) + "/" + F(LegAngularAccelerationFeedforwardMaxAcceleration) + "/maxAge=" + F(LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds) + "/normal=PRODUCTION_ON" +
                " | kneeAxisCalibration=DATA_DRIVEN_PCA_EXPLICIT_APPLY" +
                " | legStage=BILATERAL_FOUNDATION_FROM_FROZEN_SINGLE_LEG_V0300" +
                " | kneeTorqueTopology=CHILD_ONLY_PRODUCTION | kneeConstraintBackend=WIDE_CHARACTER_JOINT_PLUS_SOFTWARE_OFFAXIS_PD | legMassProperties=EXPLICIT_UNIFORM_BOX_VALIDATED | legDiagnostics=SINGLE_LEG_TRANSPORT_AND_FLEXION_AXIS_ATTRIBUTION+DATA_DRIVEN_KNEE_AXIS_CALIBRATION+ANGULAR_PHASE_TIMING_ATTRIBUTION_0_TO_10+ANGULAR_ACCELERATION_ACTUATION_FF_VALIDATED_BOUND20" +
                " | detail=" + LoadMessage;
        }

        private static void Apply(ActiveRagdollLabProfile p, string key, string value)
        {
            if (key.Equals("targetName", StringComparison.OrdinalIgnoreCase)) p.TargetName = value;
            else if (key.Equals("pelvisBonePath", StringComparison.OrdinalIgnoreCase)) p.PelvisBonePath = NormalizePath(value);
            else if (key.Equals("spineBonePath", StringComparison.OrdinalIgnoreCase)) p.SpineBonePath = NormalizePath(value);
            else if (key.Equals("chestBonePath", StringComparison.OrdinalIgnoreCase)) p.ChestBonePath = NormalizePath(value);
            else if (key.Equals("pelvisMass", StringComparison.OrdinalIgnoreCase)) p.PelvisMass = ParseFloat(value, p.PelvisMass);
            else if (key.Equals("chestMass", StringComparison.OrdinalIgnoreCase)) p.ChestMass = ParseFloat(value, p.ChestMass);
            else if (key.Equals("pelvisSizeScale", StringComparison.OrdinalIgnoreCase)) p.PelvisSizeScale = ParseVector3(value, p.PelvisSizeScale);
            else if (key.Equals("chestSizeScale", StringComparison.OrdinalIgnoreCase)) p.ChestSizeScale = ParseVector3(value, p.ChestSizeScale);
            else if (key.Equals("minBoxDimension", StringComparison.OrdinalIgnoreCase)) p.MinBoxDimension = ParseFloat(value, p.MinBoxDimension);
            else if (key.Equals("maxBoxDimension", StringComparison.OrdinalIgnoreCase)) p.MaxBoxDimension = ParseFloat(value, p.MaxBoxDimension);
            else if (key.Equals("maxLinearVelocity", StringComparison.OrdinalIgnoreCase)) p.MaxLinearVelocity = ParseFloat(value, p.MaxLinearVelocity);
            else if (key.Equals("angularSpring", StringComparison.OrdinalIgnoreCase)) p.AngularSpring = ParseFloat(value, p.AngularSpring);
            else if (key.Equals("angularDamper", StringComparison.OrdinalIgnoreCase)) p.AngularDamper = ParseFloat(value, p.AngularDamper);
            else if (key.Equals("maxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.MaxAngularAcceleration = ParseFloat(value, p.MaxAngularAcceleration);
            else if (key.Equals("angularDrag", StringComparison.OrdinalIgnoreCase)) p.AngularDrag = ParseFloat(value, p.AngularDrag);
            else if (key.Equals("maxAngularVelocity", StringComparison.OrdinalIgnoreCase)) p.MaxAngularVelocity = ParseFloat(value, p.MaxAngularVelocity);
            else if (key.Equals("driveDisableDistance", StringComparison.OrdinalIgnoreCase)) p.DriveDisableDistance = ParseFloat(value, p.DriveDisableDistance);
            else if (key.Equals("targetVelocityBlend", StringComparison.OrdinalIgnoreCase)) p.TargetVelocityBlend = ParseFloat(value, p.TargetVelocityBlend);
            else if (key.Equals("traceIntervalSeconds", StringComparison.OrdinalIgnoreCase)) p.TraceIntervalSeconds = ParseFloat(value, p.TraceIntervalSeconds);
            else if (key.Equals("traceVerboseSnapshots", StringComparison.OrdinalIgnoreCase)) p.TraceVerboseSnapshots = ParseBool(value, p.TraceVerboseSnapshots);
            else if (key.Equals("sourceTurnVerboseContextLogging", StringComparison.OrdinalIgnoreCase)) p.SourceTurnVerboseContextLogging = ParseBool(value, p.SourceTurnVerboseContextLogging);
            else if (key.Equals("motionIdleSpeedThreshold", StringComparison.OrdinalIgnoreCase)) p.MotionIdleSpeedThreshold = ParseFloat(value, p.MotionIdleSpeedThreshold);
            else if (key.Equals("motionSteadyMinSpeed", StringComparison.OrdinalIgnoreCase)) p.MotionSteadyMinSpeed = ParseFloat(value, p.MotionSteadyMinSpeed);
            else if (key.Equals("motionLinearAccelerationThreshold", StringComparison.OrdinalIgnoreCase)) p.MotionLinearAccelerationThreshold = ParseFloat(value, p.MotionLinearAccelerationThreshold);
            else if (key.Equals("motionTurnAngularVelocityThreshold", StringComparison.OrdinalIgnoreCase)) p.MotionTurnAngularVelocityThreshold = ParseFloat(value, p.MotionTurnAngularVelocityThreshold);
            else if (key.Equals("motionTurnAngularAccelerationThreshold", StringComparison.OrdinalIgnoreCase)) p.MotionTurnAngularAccelerationThreshold = ParseFloat(value, p.MotionTurnAngularAccelerationThreshold);
            else if (key.Equals("motionTransitionHoldSeconds", StringComparison.OrdinalIgnoreCase)) p.MotionTransitionHoldSeconds = ParseFloat(value, p.MotionTransitionHoldSeconds);
            else if (key.Equals("sourceTurnEventRotationDeltaDegrees", StringComparison.OrdinalIgnoreCase)) p.SourceTurnEventRotationDeltaDegrees = ParseFloat(value, p.SourceTurnEventRotationDeltaDegrees);
            else if (key.Equals("sourceTurnEventAngularSpeedThreshold", StringComparison.OrdinalIgnoreCase)) p.SourceTurnEventAngularSpeedThreshold = ParseFloat(value, p.SourceTurnEventAngularSpeedThreshold);
            else if (key.Equals("sourceTurnEventAngularAccelerationThreshold", StringComparison.OrdinalIgnoreCase)) p.SourceTurnEventAngularAccelerationThreshold = ParseFloat(value, p.SourceTurnEventAngularAccelerationThreshold);
            else if (key.Equals("sourceTurnContextPreSeconds", StringComparison.OrdinalIgnoreCase)) p.SourceTurnContextPreSeconds = ParseFloat(value, p.SourceTurnContextPreSeconds);
            else if (key.Equals("sourceTurnContextPostSeconds", StringComparison.OrdinalIgnoreCase)) p.SourceTurnContextPostSeconds = ParseFloat(value, p.SourceTurnContextPostSeconds);
            else if (key.Equals("parentMotionFeedforward", StringComparison.OrdinalIgnoreCase)) p.ParentMotionFeedforward = ParseBool(value, p.ParentMotionFeedforward);
            else if (key.Equals("parentMotionFeedforwardGain", StringComparison.OrdinalIgnoreCase)) p.ParentMotionFeedforwardGain = ParseFloat(value, p.ParentMotionFeedforwardGain);
            else if (key.Equals("maxParentMotionFeedforwardAcceleration", StringComparison.OrdinalIgnoreCase)) p.MaxParentMotionFeedforwardAcceleration = ParseFloat(value, p.MaxParentMotionFeedforwardAcceleration);
            else if (key.Equals("poseBridgeMaxExtrapolationSeconds", StringComparison.OrdinalIgnoreCase)) p.PoseBridgeMaxExtrapolationSeconds = ParseFloat(value, p.PoseBridgeMaxExtrapolationSeconds);
            else if (key.Equals("poseBridgePositionCorrectionGain", StringComparison.OrdinalIgnoreCase)) p.PoseBridgePositionCorrectionGain = ParseFloat(value, p.PoseBridgePositionCorrectionGain);
            else if (key.Equals("poseBridgeMaxPositionCorrectionSpeed", StringComparison.OrdinalIgnoreCase)) p.PoseBridgeMaxPositionCorrectionSpeed = ParseFloat(value, p.PoseBridgeMaxPositionCorrectionSpeed);
            else if (key.Equals("poseBridgeRotationCorrectionGain", StringComparison.OrdinalIgnoreCase)) p.PoseBridgeRotationCorrectionGain = ParseFloat(value, p.PoseBridgeRotationCorrectionGain);
            else if (key.Equals("poseBridgeMaxRotationCorrectionAngularSpeed", StringComparison.OrdinalIgnoreCase)) p.PoseBridgeMaxRotationCorrectionAngularSpeed = ParseFloat(value, p.PoseBridgeMaxRotationCorrectionAngularSpeed);
            else if (key.Equals("wideJointLimitIsolation", StringComparison.OrdinalIgnoreCase)) p.WideJointLimitIsolation = ParseBool(value, p.WideJointLimitIsolation);
            else if (key.Equals("wideJointLimitDegrees", StringComparison.OrdinalIgnoreCase)) p.WideJointLimitDegrees = ParseFloat(value, p.WideJointLimitDegrees);
            else if (key.Equals("chestLowTwistLimitDegrees", StringComparison.OrdinalIgnoreCase)) p.ChestLowTwistLimitDegrees = ParseFloat(value, p.ChestLowTwistLimitDegrees);
            else if (key.Equals("chestHighTwistLimitDegrees", StringComparison.OrdinalIgnoreCase)) p.ChestHighTwistLimitDegrees = ParseFloat(value, p.ChestHighTwistLimitDegrees);
            else if (key.Equals("chestSwing1LimitDegrees", StringComparison.OrdinalIgnoreCase)) p.ChestSwing1LimitDegrees = ParseFloat(value, p.ChestSwing1LimitDegrees);
            else if (key.Equals("chestSwing2LimitDegrees", StringComparison.OrdinalIgnoreCase)) p.ChestSwing2LimitDegrees = ParseFloat(value, p.ChestSwing2LimitDegrees);
            else if (key.Equals("jointAnchorSafetyDistance", StringComparison.OrdinalIgnoreCase)) p.JointAnchorSafetyDistance = ParseFloat(value, p.JointAnchorSafetyDistance);
            else if (key.Equals("singleLegEnabled", StringComparison.OrdinalIgnoreCase)) p.SingleLegEnabled = ParseBool(value, p.SingleLegEnabled);
            else if (key.Equals("bilateralLegEnabled", StringComparison.OrdinalIgnoreCase)) p.BilateralLegEnabled = ParseBool(value, p.BilateralLegEnabled);
            else if (key.Equals("singleLegSide", StringComparison.OrdinalIgnoreCase)) p.SingleLegSide = value;
            else if (key.Equals("singleLegUpperBonePath", StringComparison.OrdinalIgnoreCase)) p.SingleLegUpperBonePath = NormalizePath(value);
            else if (key.Equals("singleLegLowerBonePath", StringComparison.OrdinalIgnoreCase)) p.SingleLegLowerBonePath = NormalizePath(value);
            else if (key.Equals("singleLegFootBonePath", StringComparison.OrdinalIgnoreCase)) p.SingleLegFootBonePath = NormalizePath(value);
            else if (key.Equals("upperLegMass", StringComparison.OrdinalIgnoreCase)) p.UpperLegMass = ParseFloat(value, p.UpperLegMass);
            else if (key.Equals("lowerLegMass", StringComparison.OrdinalIgnoreCase)) p.LowerLegMass = ParseFloat(value, p.LowerLegMass);
            else if (key.Equals("footAnkleFoundationEnabled", StringComparison.OrdinalIgnoreCase)) p.FootAnkleFoundationEnabled = ParseBool(value, p.FootAnkleFoundationEnabled);
            else if (key.Equals("footMass", StringComparison.OrdinalIgnoreCase)) p.FootMass = ParseFloat(value, p.FootMass);
            else if (key.Equals("legSegmentThickness", StringComparison.OrdinalIgnoreCase)) p.LegSegmentThickness = ParseFloat(value, p.LegSegmentThickness);
            else if (key.Equals("legHipAngularSpring", StringComparison.OrdinalIgnoreCase)) p.LegHipAngularSpring = ParseFloat(value, p.LegHipAngularSpring);
            else if (key.Equals("legHipAngularDamper", StringComparison.OrdinalIgnoreCase)) p.LegHipAngularDamper = ParseFloat(value, p.LegHipAngularDamper);
            else if (key.Equals("legHipMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegHipMaxAngularAcceleration = ParseFloat(value, p.LegHipMaxAngularAcceleration);
            else if (key.Equals("legHipSourceTurnMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegHipSourceTurnMaxAngularAcceleration = ParseFloat(value, p.LegHipSourceTurnMaxAngularAcceleration);
            else if (key.Equals("legHipRecoveryMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegHipRecoveryMaxAngularAcceleration = ParseFloat(value, p.LegHipRecoveryMaxAngularAcceleration);
            else if (key.Equals("legHipRecoveryExitErrorDegrees", StringComparison.OrdinalIgnoreCase)) p.LegHipRecoveryExitErrorDegrees = ParseFloat(value, p.LegHipRecoveryExitErrorDegrees);
            else if (key.Equals("legHipRecoveryStableHoldSeconds", StringComparison.OrdinalIgnoreCase)) p.LegHipRecoveryStableHoldSeconds = ParseFloat(value, p.LegHipRecoveryStableHoldSeconds);
            else if (key.Equals("legHipRecoveryMaxSeconds", StringComparison.OrdinalIgnoreCase)) p.LegHipRecoveryMaxSeconds = ParseFloat(value, p.LegHipRecoveryMaxSeconds);
            else if (key.Equals("legHipTrackingHealthEnterErrorDegrees", StringComparison.OrdinalIgnoreCase)) p.LegHipTrackingHealthEnterErrorDegrees = ParseFloat(value, p.LegHipTrackingHealthEnterErrorDegrees);
            else if (key.Equals("legHipTrackingHealthEnterHoldSeconds", StringComparison.OrdinalIgnoreCase)) p.LegHipTrackingHealthEnterHoldSeconds = ParseFloat(value, p.LegHipTrackingHealthEnterHoldSeconds);
            else if (key.Equals("legHipTrackingHealthImmediateErrorDegrees", StringComparison.OrdinalIgnoreCase)) p.LegHipTrackingHealthImmediateErrorDegrees = ParseFloat(value, p.LegHipTrackingHealthImmediateErrorDegrees);
            else if (key.Equals("legHipControlFrameInertialization", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameInertialization = ParseBool(value, p.LegHipControlFrameInertialization);
            else if (key.Equals("legHipControlFrameSourceMaxAngularSpeed", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameSourceMaxAngularSpeed = ParseFloat(value, p.LegHipControlFrameSourceMaxAngularSpeed);
            else if (key.Equals("legHipControlFrameSourceMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameSourceMaxAngularAcceleration = ParseFloat(value, p.LegHipControlFrameSourceMaxAngularAcceleration);
            else if (key.Equals("legHipControlFrameRecoveryMaxAngularSpeed", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameRecoveryMaxAngularSpeed = ParseFloat(value, p.LegHipControlFrameRecoveryMaxAngularSpeed);
            else if (key.Equals("legHipControlFrameRecoveryMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameRecoveryMaxAngularAcceleration = ParseFloat(value, p.LegHipControlFrameRecoveryMaxAngularAcceleration);
            else if (key.Equals("legHipControlFrameMaxLagDegrees", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameMaxLagDegrees = ParseFloat(value, p.LegHipControlFrameMaxLagDegrees);
            else if (key.Equals("legHipControlFrameRecoveryExitAngleDegrees", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameRecoveryExitAngleDegrees = ParseFloat(value, p.LegHipControlFrameRecoveryExitAngleDegrees);
            else if (key.Equals("legHipControlFrameRecoveryExitAngularSpeed", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameRecoveryExitAngularSpeed = ParseFloat(value, p.LegHipControlFrameRecoveryExitAngularSpeed);
            else if (key.Equals("legHipControlFrameRecoveryStableHoldSeconds", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameRecoveryStableHoldSeconds = ParseFloat(value, p.LegHipControlFrameRecoveryStableHoldSeconds);
            else if (key.Equals("legHipControlFrameRecoveryMaxSeconds", StringComparison.OrdinalIgnoreCase)) p.LegHipControlFrameRecoveryMaxSeconds = ParseFloat(value, p.LegHipControlFrameRecoveryMaxSeconds);
            else if (key.Equals("legKneeAngularSpring", StringComparison.OrdinalIgnoreCase)) p.LegKneeAngularSpring = ParseFloat(value, p.LegKneeAngularSpring);
            else if (key.Equals("legKneeAngularDamper", StringComparison.OrdinalIgnoreCase)) p.LegKneeAngularDamper = ParseFloat(value, p.LegKneeAngularDamper);
            else if (key.Equals("legKneeMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegKneeMaxAngularAcceleration = ParseFloat(value, p.LegKneeMaxAngularAcceleration);
            else if (key.Equals("legKneeSoftOffAxisAngularSpring", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisAngularSpring = ParseFloat(value, p.LegKneeSoftOffAxisAngularSpring);
            else if (key.Equals("legKneeSoftOffAxisAngularDamper", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisAngularDamper = ParseFloat(value, p.LegKneeSoftOffAxisAngularDamper);
            else if (key.Equals("legKneeSoftOffAxisMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisMaxAngularAcceleration = ParseFloat(value, p.LegKneeSoftOffAxisMaxAngularAcceleration);
            else if (key.Equals("legKneeSoftOffAxisSourceTurnMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration = ParseFloat(value, p.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration);
            else if (key.Equals("legKneeSoftOffAxisRecoveryMaxAngularAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration = ParseFloat(value, p.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration);
            else if (key.Equals("legKneeSoftOffAxisRecoveryExitErrorDegrees", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisRecoveryExitErrorDegrees = ParseFloat(value, p.LegKneeSoftOffAxisRecoveryExitErrorDegrees);
            else if (key.Equals("legKneeSoftOffAxisRecoveryStableHoldSeconds", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisRecoveryStableHoldSeconds = ParseFloat(value, p.LegKneeSoftOffAxisRecoveryStableHoldSeconds);
            else if (key.Equals("legKneeSoftOffAxisRecoveryMaxSeconds", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisRecoveryMaxSeconds = ParseFloat(value, p.LegKneeSoftOffAxisRecoveryMaxSeconds);
            else if (key.Equals("legKneeSoftOffAxisTrackingHealthEnterErrorDegrees", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees = ParseFloat(value, p.LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees);
            else if (key.Equals("legKneeSoftOffAxisTrackingHealthEnterHoldSeconds", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisTrackingHealthEnterHoldSeconds = ParseFloat(value, p.LegKneeSoftOffAxisTrackingHealthEnterHoldSeconds);
            else if (key.Equals("legKneeSoftOffAxisTrackingHealthImmediateErrorDegrees", StringComparison.OrdinalIgnoreCase)) p.LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees = ParseFloat(value, p.LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees);
            else if (key.Equals("legDriveDisableDistance", StringComparison.OrdinalIgnoreCase)) p.LegDriveDisableDistance = ParseFloat(value, p.LegDriveDisableDistance);
            else if (key.Equals("legAngularAccelerationFeedforward", StringComparison.OrdinalIgnoreCase)) p.LegAngularAccelerationFeedforward = ParseBool(value, p.LegAngularAccelerationFeedforward);
            else if (key.Equals("legAngularAccelerationFeedforwardGain", StringComparison.OrdinalIgnoreCase)) p.LegAngularAccelerationFeedforwardGain = ParseFloat(value, p.LegAngularAccelerationFeedforwardGain);
            else if (key.Equals("legAngularAccelerationFeedforwardMaxAcceleration", StringComparison.OrdinalIgnoreCase)) p.LegAngularAccelerationFeedforwardMaxAcceleration = ParseFloat(value, p.LegAngularAccelerationFeedforwardMaxAcceleration);
            else if (key.Equals("legAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds", StringComparison.OrdinalIgnoreCase)) p.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds = ParseFloat(value, p.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds);
            else if (key.Equals("defaultColliders", StringComparison.OrdinalIgnoreCase)) p.DefaultColliders = ParseBool(value, p.DefaultColliders);
            else if (key.Equals("defaultGravity", StringComparison.OrdinalIgnoreCase)) p.DefaultGravity = ParseBool(value, p.DefaultGravity);
            else if (key.Equals("previewAlpha", StringComparison.OrdinalIgnoreCase)) p.PreviewAlpha = ParseFloat(value, p.PreviewAlpha);
            // 旧配置文件中的未知/已退役键会被安全忽略；sourceTurn* 等仍在使用的诊断键继续兼容。
            // Two-Body freeze gate 仍固定为 v0.1.16 已验证策略；wideJointLimitIsolation 缺失时默认 true。
        }

        private static void ValidateAndClamp(ActiveRagdollLabProfile p)
        {
            p.TargetName = string.IsNullOrWhiteSpace(p.TargetName) ? "MitaPerson Mita" : p.TargetName.Trim();
            p.PelvisBonePath = RequirePath(p.PelvisBonePath, "Armature/Hips");
            p.SpineBonePath = RequirePath(p.SpineBonePath, "Armature/Hips/Spine");
            p.ChestBonePath = RequirePath(p.ChestBonePath, "Armature/Hips/Spine/Chest");

            p.PelvisMass = ClampFinite(p.PelvisMass, 0.5f, 40f, 8f);
            p.ChestMass = ClampFinite(p.ChestMass, 0.5f, 40f, 12f);
            p.PelvisSizeScale = ClampVector(p.PelvisSizeScale, 0.10f, 2.0f, new Vector3(0.70f, 0.34f, 0.52f));
            p.ChestSizeScale = ClampVector(p.ChestSizeScale, 0.10f, 2.0f, new Vector3(0.86f, 0.68f, 0.56f));
            p.MinBoxDimension = ClampFinite(p.MinBoxDimension, 0.03f, 0.40f, 0.10f);
            p.MaxBoxDimension = ClampFinite(p.MaxBoxDimension, p.MinBoxDimension + 0.02f, 1.5f, 0.55f);

            p.MaxLinearVelocity = ClampFinite(p.MaxLinearVelocity, 0.5f, 30f, 6f);
            p.AngularSpring = ClampFinite(p.AngularSpring, 0f, 1000f, 55f);
            p.AngularDamper = ClampFinite(p.AngularDamper, 0f, 200f, 12f);
            p.MaxAngularAcceleration = ClampFinite(p.MaxAngularAcceleration, 1f, 300f, 40f);
            p.AngularDrag = ClampFinite(p.AngularDrag, 0f, 20f, 3f);
            p.MaxAngularVelocity = ClampFinite(p.MaxAngularVelocity, 1f, 100f, 18f);
            p.DriveDisableDistance = ClampFinite(p.DriveDisableDistance, 0.15f, 3f, 0.80f);
            p.TargetVelocityBlend = ClampFinite(p.TargetVelocityBlend, 0.05f, 1.0f, 0.50f);
            p.TraceIntervalSeconds = ClampFinite(p.TraceIntervalSeconds, 0.10f, 5.0f, 0.50f);
            p.MotionIdleSpeedThreshold = ClampFinite(p.MotionIdleSpeedThreshold, 0.01f, 1.0f, 0.15f);
            p.MotionSteadyMinSpeed = ClampFinite(p.MotionSteadyMinSpeed, p.MotionIdleSpeedThreshold + 0.05f, 3.0f, 0.50f);
            p.MotionLinearAccelerationThreshold = ClampFinite(p.MotionLinearAccelerationThreshold, 0.1f, 20f, 3.00f);
            p.MotionTurnAngularVelocityThreshold = ClampFinite(p.MotionTurnAngularVelocityThreshold, 0.05f, 10f, 1.20f);
            p.MotionTurnAngularAccelerationThreshold = ClampFinite(p.MotionTurnAngularAccelerationThreshold, 0.1f, 50f, 12.0f);
            p.MotionTransitionHoldSeconds = ClampFinite(p.MotionTransitionHoldSeconds, 0f, 2f, 0.30f);
            p.SourceTurnEventRotationDeltaDegrees = ClampFinite(p.SourceTurnEventRotationDeltaDegrees, 2f, 120f, 20.0f);
            p.SourceTurnEventAngularSpeedThreshold = ClampFinite(p.SourceTurnEventAngularSpeedThreshold, 0.5f, 30f, 4.0f);
            p.SourceTurnEventAngularAccelerationThreshold = ClampFinite(p.SourceTurnEventAngularAccelerationThreshold, 2f, 500f, 30.0f);
            p.SourceTurnContextPreSeconds = ClampFinite(p.SourceTurnContextPreSeconds, 0.10f, 2.0f, 0.50f);
            p.SourceTurnContextPostSeconds = ClampFinite(p.SourceTurnContextPostSeconds, 0.10f, 3.0f, 0.75f);
            p.ParentMotionFeedforwardGain = ClampFinite(p.ParentMotionFeedforwardGain, 0f, 80f, 12f);
            p.MaxParentMotionFeedforwardAcceleration = ClampFinite(p.MaxParentMotionFeedforwardAcceleration, 1f, 100f, 20f);
            p.PoseBridgeMaxExtrapolationSeconds = ClampFinite(p.PoseBridgeMaxExtrapolationSeconds, 0.01f, 0.30f, 0.10f);
            p.PoseBridgePositionCorrectionGain = ClampFinite(p.PoseBridgePositionCorrectionGain, 0f, 30f, 6.0f);
            p.PoseBridgeMaxPositionCorrectionSpeed = ClampFinite(p.PoseBridgeMaxPositionCorrectionSpeed, 0.05f, 5f, 0.75f);
            p.PoseBridgeRotationCorrectionGain = ClampFinite(p.PoseBridgeRotationCorrectionGain, 0f, 30f, 6.0f);
            p.PoseBridgeMaxRotationCorrectionAngularSpeed = ClampFinite(p.PoseBridgeMaxRotationCorrectionAngularSpeed, 0.1f, 20f, 3.0f);
            p.WideJointLimitDegrees = ClampFinite(p.WideJointLimitDegrees, 120f, 175f, 170f);
            p.ChestLowTwistLimitDegrees = ClampFinite(p.ChestLowTwistLimitDegrees, -175f, -1f, -35f);
            p.ChestHighTwistLimitDegrees = ClampFinite(p.ChestHighTwistLimitDegrees, 1f, 175f, 35f);
            p.ChestSwing1LimitDegrees = ClampFinite(p.ChestSwing1LimitDegrees, 1f, 175f, 45f);
            p.ChestSwing2LimitDegrees = ClampFinite(p.ChestSwing2LimitDegrees, 1f, 175f, 35f);
            p.JointAnchorSafetyDistance = ClampFinite(p.JointAnchorSafetyDistance, 0.03f, 0.50f, 0.15f);
            p.SingleLegSide = NormalizeLegSide(p.SingleLegSide);
            p.UpperLegMass = ClampFinite(p.UpperLegMass, 0.5f, 30f, 7f);
            p.LowerLegMass = ClampFinite(p.LowerLegMass, 0.25f, 20f, 4f);
            p.FootMass = ClampFinite(p.FootMass, 0.1f, 10f, 1f);
            p.LegSegmentThickness = ClampFinite(p.LegSegmentThickness, 0.04f, 0.30f, 0.10f);
            p.LegHipAngularSpring = ClampFinite(p.LegHipAngularSpring, 0f, 500f, 55f);
            p.LegHipAngularDamper = ClampFinite(p.LegHipAngularDamper, 0f, 100f, 12f);
            p.LegHipMaxAngularAcceleration = ClampFinite(p.LegHipMaxAngularAcceleration, 1f, 200f, 40f);
            p.LegHipSourceTurnMaxAngularAcceleration = ClampFinite(p.LegHipSourceTurnMaxAngularAcceleration, p.LegHipMaxAngularAcceleration, 200f, 80f);
            p.LegHipRecoveryMaxAngularAcceleration = ClampFinite(p.LegHipRecoveryMaxAngularAcceleration, p.LegHipMaxAngularAcceleration, 200f, 80f);
            p.LegHipRecoveryExitErrorDegrees = ClampFinite(p.LegHipRecoveryExitErrorDegrees, 1f, 45f, 10f);
            p.LegHipRecoveryStableHoldSeconds = ClampFinite(p.LegHipRecoveryStableHoldSeconds, 0.02f, 1.0f, 0.15f);
            p.LegHipRecoveryMaxSeconds = ClampFinite(p.LegHipRecoveryMaxSeconds, 0.25f, 5.0f, 2.0f);
            p.LegHipTrackingHealthEnterErrorDegrees = ClampFinite(p.LegHipTrackingHealthEnterErrorDegrees, p.LegHipRecoveryExitErrorDegrees + 1f, 90f, 15f);
            p.LegHipTrackingHealthEnterHoldSeconds = ClampFinite(p.LegHipTrackingHealthEnterHoldSeconds, 0.02f, 0.50f, 0.10f);
            p.LegHipTrackingHealthImmediateErrorDegrees = ClampFinite(p.LegHipTrackingHealthImmediateErrorDegrees, p.LegHipTrackingHealthEnterErrorDegrees, 120f, 30f);
            p.LegHipControlFrameSourceMaxAngularSpeed = ClampFinite(p.LegHipControlFrameSourceMaxAngularSpeed, 0.5f, 18f, 6f);
            p.LegHipControlFrameSourceMaxAngularAcceleration = ClampFinite(p.LegHipControlFrameSourceMaxAngularAcceleration, 5f, 300f, 60f);
            p.LegHipControlFrameRecoveryMaxAngularSpeed = ClampFinite(p.LegHipControlFrameRecoveryMaxAngularSpeed, 0.5f, 18f, 8f);
            p.LegHipControlFrameRecoveryMaxAngularAcceleration = ClampFinite(p.LegHipControlFrameRecoveryMaxAngularAcceleration, 5f, 300f, 80f);
            p.LegHipControlFrameMaxLagDegrees = ClampFinite(p.LegHipControlFrameMaxLagDegrees, 5f, 179f, 25f);
            p.LegHipControlFrameRecoveryExitAngleDegrees = ClampFinite(p.LegHipControlFrameRecoveryExitAngleDegrees, 0.25f, 10f, 1.5f);
            p.LegHipControlFrameRecoveryExitAngularSpeed = ClampFinite(p.LegHipControlFrameRecoveryExitAngularSpeed, 0.05f, 3f, 0.5f);
            p.LegHipControlFrameRecoveryStableHoldSeconds = ClampFinite(p.LegHipControlFrameRecoveryStableHoldSeconds, 0.05f, 0.5f, 0.10f);
            p.LegHipControlFrameRecoveryMaxSeconds = ClampFinite(p.LegHipControlFrameRecoveryMaxSeconds, 0.25f, 3f, 1.5f);
            p.LegKneeAngularSpring = ClampFinite(p.LegKneeAngularSpring, 0f, 500f, 45f);
            p.LegKneeAngularDamper = ClampFinite(p.LegKneeAngularDamper, 0f, 100f, 9f);
            p.LegKneeMaxAngularAcceleration = ClampFinite(p.LegKneeMaxAngularAcceleration, 1f, 200f, 40f);
            p.LegKneeSoftOffAxisAngularSpring = ClampFinite(p.LegKneeSoftOffAxisAngularSpring, 0f, 500f, 30f);
            p.LegKneeSoftOffAxisAngularDamper = ClampFinite(p.LegKneeSoftOffAxisAngularDamper, 0f, 100f, 8f);
            p.LegKneeSoftOffAxisMaxAngularAcceleration = ClampFinite(p.LegKneeSoftOffAxisMaxAngularAcceleration, 1f, 100f, 20f);
            p.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration = ClampFinite(p.LegKneeSoftOffAxisSourceTurnMaxAngularAcceleration, p.LegKneeSoftOffAxisMaxAngularAcceleration, 120f, 40f);
            p.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration = ClampFinite(p.LegKneeSoftOffAxisRecoveryMaxAngularAcceleration, p.LegKneeSoftOffAxisMaxAngularAcceleration, 120f, 40f);
            p.LegKneeSoftOffAxisRecoveryExitErrorDegrees = ClampFinite(p.LegKneeSoftOffAxisRecoveryExitErrorDegrees, 0.5f, 30f, 5f);
            p.LegKneeSoftOffAxisRecoveryStableHoldSeconds = ClampFinite(p.LegKneeSoftOffAxisRecoveryStableHoldSeconds, 0.02f, 1.0f, 0.15f);
            p.LegKneeSoftOffAxisRecoveryMaxSeconds = ClampFinite(p.LegKneeSoftOffAxisRecoveryMaxSeconds, 0.25f, 5.0f, 2.0f);
            p.LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees = ClampFinite(p.LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees, p.LegKneeSoftOffAxisRecoveryExitErrorDegrees + 0.5f, 60f, 10f);
            p.LegKneeSoftOffAxisTrackingHealthEnterHoldSeconds = ClampFinite(p.LegKneeSoftOffAxisTrackingHealthEnterHoldSeconds, 0.02f, 0.50f, 0.10f);
            p.LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees = ClampFinite(p.LegKneeSoftOffAxisTrackingHealthImmediateErrorDegrees, p.LegKneeSoftOffAxisTrackingHealthEnterErrorDegrees, 90f, 25f);
            p.LegDriveDisableDistance = ClampFinite(p.LegDriveDisableDistance, 0.15f, 2f, 0.65f);
            p.LegAngularAccelerationFeedforwardGain = ClampFinite(p.LegAngularAccelerationFeedforwardGain, 0f, 4f, 1f);
            p.LegAngularAccelerationFeedforwardMaxAcceleration = ClampFinite(p.LegAngularAccelerationFeedforwardMaxAcceleration, 0f, 40f, 20f);
            p.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds = ClampFinite(p.LegAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds, 0.01f, 0.25f, 0.10f);
            p.PreviewAlpha = ClampFinite(p.PreviewAlpha, 0.05f, 0.80f, 0.28f);
        }

        private static string NormalizeLegSide(string value)
        {
            string side = (value ?? string.Empty).Trim();
            return side.Equals("right", StringComparison.OrdinalIgnoreCase) || side.Equals("r", StringComparison.OrdinalIgnoreCase)
                ? "Right"
                : "Left";
        }

        private static string NormalizePath(string value)
        {
            return (value ?? string.Empty).Trim().Trim('/').Replace('\\', '/');
        }

        private static string RequirePath(string value, string fallback)
        {
            string normalized = NormalizePath(value);
            return normalized.Length == 0 ? fallback : normalized;
        }

        private static float ParseFloat(string value, float fallback)
        {
            return float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out float parsed)
                ? parsed
                : fallback;
        }

        private static bool ParseBool(string value, bool fallback)
        {
            if (bool.TryParse(value, out bool parsed))
                return parsed;
            if (value == "1" || value.Equals("on", StringComparison.OrdinalIgnoreCase))
                return true;
            if (value == "0" || value.Equals("off", StringComparison.OrdinalIgnoreCase))
                return false;
            return fallback;
        }

        private static Vector3 ParseVector3(string value, Vector3 fallback)
        {
            string[] parts = (value ?? string.Empty).Split(',');
            if (parts.Length != 3)
                return fallback;
            if (!float.TryParse(parts[0].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float x) ||
                !float.TryParse(parts[1].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float y) ||
                !float.TryParse(parts[2].Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out float z))
                return fallback;
            return new Vector3(x, y, z);
        }

        private static float ClampFinite(float value, float min, float max, float fallback)
        {
            if (float.IsNaN(value) || float.IsInfinity(value))
                value = fallback;
            return Mathf.Clamp(value, min, max);
        }

        private static Vector3 ClampVector(Vector3 value, float min, float max, Vector3 fallback)
        {
            if (!IsFinite(value))
                value = fallback;
            return new Vector3(
                Mathf.Clamp(value.x, min, max),
                Mathf.Clamp(value.y, min, max),
                Mathf.Clamp(value.z, min, max));
        }

        private static bool IsFinite(Vector3 value)
        {
            return !(float.IsNaN(value.x) || float.IsInfinity(value.x) ||
                     float.IsNaN(value.y) || float.IsInfinity(value.y) ||
                     float.IsNaN(value.z) || float.IsInfinity(value.z));
        }

        private static string F(float value) => value.ToString("G6", CultureInfo.InvariantCulture);
        private static string V(Vector3 value) => "(" + F(value.x) + "," + F(value.y) + "," + F(value.z) + ")";

        private static string GetTemplateText()
        {
            return
                "# MakemitAGA Active Ragdoll Lab 0.4.100 - Hip Motion Feasibility Allocation\r\n" +
                "# 保存后执行：arl_profile_reload\r\n" +
                "# 当前主线：v0.4.100在arl_pelvis_hip_test(b)验证纠偏优先的共同运动预算；先暂存双侧Hip角请求，联合削减骨盆共同COM与Upper净角请求，再继续Knee/Foot；完整纠偏不可行时显式标记fallback；保留Upper同拍-r×F和平移质量求力，遵守力诱导相对点加速度60m/s²、Upper补偿8Nm、实际执行器max(8Nm,原基线幅值)、骨盆总矩8Nm预算；原骨盆基础请求仍限2Nm；物理arm=B_V041000_HIP_MOTION_FEASIBILITY_ALLOCATION；双端点力对拓扑/目标链/安全门及95射线范围修复保持；纯Hip角控制器反力、Torso重力及Foot-tether reciprocal关闭。\r\n" +
                "# 本文件只保留仍由活动运行时消费的配置；历史实验参数与 A/B 细节请查长期版本记录或旧版本 ZIP。\r\n" +
                "\r\n" +
                "# --- 目标与 Two-Body 几何 ---\r\n" +
                "targetName=MitaPerson Mita\r\n" +
                "pelvisBonePath=Armature/Hips\r\n" +
                "spineBonePath=Armature/Hips/Spine\r\n" +
                "chestBonePath=Armature/Hips/Spine/Chest\r\n" +
                "pelvisMass=8\r\n" +
                "chestMass=12\r\n" +
                "pelvisSizeScale=0.70,0.34,0.52\r\n" +
                "chestSizeScale=0.86,0.68,0.56\r\n" +
                "minBoxDimension=0.10\r\n" +
                "maxBoxDimension=0.55\r\n" +
                "\r\n" +
                "# --- Two-Body 冻结控制与诊断 ---\r\n" +
                "# 运动阶段与 source-turn 阈值只用于 trace/冻结验收，不进入控制。\r\n" +
                "maxLinearVelocity=6\r\n" +
                "angularSpring=55\r\n" +
                "angularDamper=12\r\n" +
                "maxAngularAcceleration=40\r\n" +
                "angularDrag=3\r\n" +
                "maxAngularVelocity=18\r\n" +
                "driveDisableDistance=0.80\r\n" +
                "targetVelocityBlend=0.50\r\n" +
                "traceIntervalSeconds=0.50\r\n" +
                "traceVerboseSnapshots=false\r\n" +
                "sourceTurnVerboseContextLogging=false\r\n" +
                "motionIdleSpeedThreshold=0.15\r\n" +
                "motionSteadyMinSpeed=0.50\r\n" +
                "motionLinearAccelerationThreshold=3.00\r\n" +
                "motionTurnAngularVelocityThreshold=1.20\r\n" +
                "motionTurnAngularAccelerationThreshold=12.0\r\n" +
                "motionTransitionHoldSeconds=0.30\r\n" +
                "sourceTurnEventRotationDeltaDegrees=20\r\n" +
                "sourceTurnEventAngularSpeedThreshold=4\r\n" +
                "sourceTurnEventAngularAccelerationThreshold=30\r\n" +
                "sourceTurnContextPreSeconds=0.50\r\n" +
                "sourceTurnContextPostSeconds=0.75\r\n" +
                "\r\n" +
                "# --- Torso pose bridge / parent feedforward / joint safety ---\r\n" +
                "# WIDE_ISOLATION=true 时人体窄限位字段仅作为未来恢复 ANATOMICAL 模式的保留配置。\r\n" +
                "parentMotionFeedforward=true\r\n" +
                "parentMotionFeedforwardGain=12\r\n" +
                "maxParentMotionFeedforwardAcceleration=20\r\n" +
                "poseBridgeMaxExtrapolationSeconds=0.10\r\n" +
                "poseBridgePositionCorrectionGain=6.0\r\n" +
                "poseBridgeMaxPositionCorrectionSpeed=0.75\r\n" +
                "poseBridgeRotationCorrectionGain=6.0\r\n" +
                "poseBridgeMaxRotationCorrectionAngularSpeed=3.0\r\n" +
                "wideJointLimitIsolation=true\r\n" +
                "wideJointLimitDegrees=170\r\n" +
                "chestLowTwistLimitDegrees=-35\r\n" +
                "chestHighTwistLimitDegrees=35\r\n" +
                "chestSwing1LimitDegrees=45\r\n" +
                "chestSwing2LimitDegrees=35\r\n" +
                "jointAnchorSafetyDistance=0.15\r\n" +
                "\r\n" +
                "# --- Bilateral frozen leg foundation ---\r\n" +
                "# 当前生产链要求 Knee PCA 显式 apply 后再 arl_hip_stack(on)；Hip/Knee hard tether safety 仍为 30 mm fail-close。显式骨骼路径默认留空，AUTO 失败时再填写。\r\n" +
                "singleLegEnabled=true\r\n" +
                "bilateralLegEnabled=true\r\n" +
                "singleLegSide=Left\r\n" +
                "singleLegUpperBonePath=\r\n" +
                "singleLegLowerBonePath=\r\n" +
                "singleLegFootBonePath=\r\n" +
                "upperLegMass=7.0\r\n" +
                "lowerLegMass=4.0\r\n" +
                "# v0.4.7：Profile键值继续不变；stance-only contact复用冻结的v0.4.5 gait阈值与v0.4.0 exact-support identity阈值，不新增参数。v0.4.6 discontinuity reconciliation不推广，正式A/B均恢复raw native-gait reference；Gravity/GSP/增益/MaxA/MaxTau/hard gate不变。\r\n" +
                "footAnkleFoundationEnabled=true\r\n" +
                "footMass=1.0\r\n" +
                "legSegmentThickness=0.10\r\n" +
                "legHipAngularSpring=55\r\n" +
                "legHipAngularDamper=12\r\n" +
                "legHipMaxAngularAcceleration=40\r\n" +
                "legHipSourceTurnMaxAngularAcceleration=80\r\n" +
                "legHipRecoveryMaxAngularAcceleration=80\r\n" +
                "legHipRecoveryExitErrorDegrees=10\r\n" +
                "legHipRecoveryStableHoldSeconds=0.15\r\n" +
                "legHipRecoveryMaxSeconds=2.0\r\n" +
                "legHipTrackingHealthEnterErrorDegrees=15\r\n" +
                "legHipTrackingHealthEnterHoldSeconds=0.10\r\n" +
                "legHipTrackingHealthImmediateErrorDegrees=30\r\n" +
                "legHipControlFrameInertialization=true\r\n" +
                "legHipControlFrameSourceMaxAngularSpeed=6\r\n" +
                "legHipControlFrameSourceMaxAngularAcceleration=60\r\n" +
                "legHipControlFrameRecoveryMaxAngularSpeed=8\r\n" +
                "legHipControlFrameRecoveryMaxAngularAcceleration=80\r\n" +
                "legHipControlFrameMaxLagDegrees=25\r\n" +
                "legHipControlFrameRecoveryExitAngleDegrees=1.5\r\n" +
                "legHipControlFrameRecoveryExitAngularSpeed=0.5\r\n" +
                "legHipControlFrameRecoveryStableHoldSeconds=0.10\r\n" +
                "legHipControlFrameRecoveryMaxSeconds=1.5\r\n" +
                "legKneeAngularSpring=45\r\n" +
                "legKneeAngularDamper=9\r\n" +
                "legKneeMaxAngularAcceleration=40\r\n" +
                "legKneeSoftOffAxisAngularSpring=30\r\n" +
                "legKneeSoftOffAxisAngularDamper=8\r\n" +
                "legKneeSoftOffAxisMaxAngularAcceleration=20\r\n" +
                "legKneeSoftOffAxisSourceTurnMaxAngularAcceleration=40\r\n" +
                "legKneeSoftOffAxisRecoveryMaxAngularAcceleration=40\r\n" +
                "legKneeSoftOffAxisRecoveryExitErrorDegrees=5\r\n" +
                "legKneeSoftOffAxisRecoveryStableHoldSeconds=0.15\r\n" +
                "legKneeSoftOffAxisRecoveryMaxSeconds=2.0\r\n" +
                "legKneeSoftOffAxisTrackingHealthEnterErrorDegrees=10\r\n" +
                "legKneeSoftOffAxisTrackingHealthEnterHoldSeconds=0.10\r\n" +
                "legKneeSoftOffAxisTrackingHealthImmediateErrorDegrees=25\r\n" +
                "legDriveDisableDistance=0.65\r\n" +
                "\r\n" +
                "# --- Validated angular-acceleration feedforward ---\r\n" +
                "# 已验证 production baseline：ON / gain=1 / bound=20；Source/Recovery/陈旧样本等抑制逻辑由运行时负责。\r\n" +
                "legAngularAccelerationFeedforward=true\r\n" +
                "legAngularAccelerationFeedforwardGain=1.0\r\n" +
                "legAngularAccelerationFeedforwardMaxAcceleration=20.0\r\n" +
                "legAngularAccelerationFeedforwardMaxSourceSampleAgeSeconds=0.10\r\n" +
                "\r\n" +
                "# --- 默认安全状态与预览 ---\r\n" +
                "# 默认不启用物理 Collider/Gravity；后续 feet/GSP 接触阶段再由专门测试流程显式开启。\r\n" +
                "defaultColliders=false\r\n" +
                "defaultGravity=false\r\n" +
                "previewAlpha=0.28\r\n";
        }
    }
}
