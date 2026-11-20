using System;
using BepInEx;
using BepInEx.Logging;
using HarmonyLib;
using Il2CppInterop.Runtime.Injection;
using MakemitAGA.ActiveRagdollLab.Physics;
using MakemitAGA.ActiveRagdollLab.Runtime;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MakemitAGA.ActiveRagdollLab
{
    [BepInPlugin(
        "com.alex.miside.activeragdolllab",
        "MakemitAGA Active Ragdoll Lab",
        Plugin.PluginVersion)]
    public sealed class Plugin : BepInEx.Unity.IL2CPP.BasePlugin
    {
        internal const string PluginVersion = "0.4.166";
        // Both arms retain the right-foot policy while B samples the left release on the pinned Main floor.
        internal const string SourceRevision = "V041660_LEFT_FOOT_HORIZONTAL_LEAD_20261120";
        internal static ManualLogSource Logger;
        private Harmony _harmony;
        private UnityEngine.Events.UnityAction<Scene, LoadSceneMode> _sceneLoadedHandler;
        private ActiveRagdollLabRunner _runner;
        private bool _unloading;

        public override void Load()
        {
            Logger = Log;
            if (_sceneLoadedHandler != null || _runner != null || _harmony != null)
            {
                if (!_unloading) return;
                if (!Unload()) throw new InvalidOperationException("PREVIOUS_PLUGIN_RELEASE_INCOMPLETE");
            }
            _unloading = false;
            try { LoadCore(); }
            catch
            {
                Unload(); // Loading may fail after component creation or subscription.
                throw;
            }
        }

        private void LoadCore()
        {
            Logger.LogInfo("[ARL] Active Ragdoll Lab " + PluginVersion + " loaded | sourceRevision=" + SourceRevision +
                " | Both arms use the v163 B right-foot contact policy. B compares three unloaded steps after an early horizontal left-foot tether correction. See arl_help for the current run.");
            try
            {
                ClassInjector.RegisterTypeInIl2Cpp<RagdollProxyContactTracker>();
                Logger.LogInfo("[ARL] Registered torso IL2CPP collision callback bridge.");
            }
            catch (Exception e)
            {
                Logger.LogWarning("[ARL] Torso contact tracker registration did not complete (type may already be registered): " + e.GetType().Name + " / " + e.Message);
            }
            try
            {
                ClassInjector.RegisterTypeInIl2Cpp<FootContactShadowTracker>();
                Logger.LogInfo("[ARL] Registered Foot/support-Box callback bridge.");
            }
            catch (Exception e)
            {
                Logger.LogWarning("[ARL] Foot contact-shadow tracker registration did not complete (type may already be registered): " + e.GetType().Name + " / " + e.Message);
            }

            ActiveRagdollLabRuntime.Initialize(Logger);
            _runner = AddComponent<ActiveRagdollLabRunner>();

            _sceneLoadedHandler = (Action<Scene, LoadSceneMode>)OnSceneLoaded;
            SceneManager.sceneLoaded += _sceneLoadedHandler;

            _harmony = new Harmony("com.alex.miside.activeragdolllab");
            _harmony.PatchAll(typeof(ActiveRagdollConsolePatch));
            Logger.LogInfo("[ARL][BASELINE] build=" + SourceRevision + " | supportProbe=FINITE_SEGMENT_NO_PADDING | hipAttitude=BASELINE2_POINT_COMPENSATION_RELATIVE_REACTION_EXTERNAL_AND_TOTAL8_NM | evidence=RAW_PRE24_POST8_STOP_PRE8_POST48_PREVIOUS_RECEIPT_NEXT_RESPONSE | comReference=V105_A_RELATIVE_V_ARRIVAL_DEFICIT_FEEDBACK_ON | testArms=" + PelvisDynamicReceiverIsolation.DynamicHipArmA + "/" + PelvisDynamicReceiverIsolation.DynamicHipArmB + " | logFormat=HIP_COM_V041060 | periodicLogMinSeconds=5 | authorityAudit=STOP_TIME_RETAINED_RECEIPTS | command=arl_pelvis_hip_test(a|b|stop) | scope=DYNAMIC_PELVIS_LEGS_HIP_ANCHOR_PAIR_CONTACT | parent=DYNAMIC_PELVIS_SETTLED_POINT | contact=EXACT_STANCE | footGravity=EXACT_STANCE_NATIVE | lowerGravity=EXACT_STANCE_NATIVE | upperGravity=EXACT_STANCE_NATIVE | torsoGravity=OFF | loadTransfer=LOWER_AND_UPPER_SELF_WEIGHT_SERIAL_PAIRS | kneeNormalCoupling=V04430_REDUCED_MASS_PD_PAIR | clocks=TORSO_LEG_FOOT_SHARED_CONTEXT | hipTranslation=FEEDBACK_PRIORITY_SYSTEM_COM_RESPONSE | HipReactionToPelvis=TRANSLATION_AT_HIP_ANCHORS | hipAngularControllerReaction=BOTH_RELATIVE_ONLY | hipNormalParent=BOTH_RECEIVER_TARGET_NORMAL_ONLY | contactEntry=BOTH_SHALLOW_REQUALIFICATION | footGroundReference=BOTH_OPTIONAL_COMPLETED_KNEE_WAYPOINT | kneeReference=BOTH_CONTINUOUS_UPPER_LOCAL_RWA | kneePlannedFF=BOTH_SOURCE_TORSO_RECOVERY | loadAdmission=BOTH_NEAR_SURFACE_TWO_FIXED | parentNormalReference=BOTH_BOUNDED_V041220 | referencePreparation=BOTH_STRONG_TO_STANCE_OR_CONFIRMED | normalBudget=BOTH_NORMAL_PRIORITY_KEEP_TANGENT_BRAKING | normalFeedback=BOTH_Y40_12.727922 | kneeGroundTarget=BOTH_JOINT_RENDER_PACKET_KNEE5_FOOT15 | systemAllocation=BOTH_CONDITIONAL_CAP10 | allocationEvidence=POST_COMPLETION_ACTUAL_RECEIPTS | candidateExecution=BOTH_CAP10_WHEN_BETTER | kneeTrajectory=BOTH_GROUND_PRIORITY_ORIGINAL_W_A | kneeInputDirection=BOTH_BOUNDED_EXTENSION_MIN_BEND5 | footTrajectory=BOTH_V134_GROUND_CHOICE_ORIGINAL_W_A | footKneeProgress=BOTH_PREVIEWED_WAYPOINT | supportReferenceBraking=BOTH_HELD_FLOOR_REFERENCE_STOPPING_RESERVE | supportNormalRecovery=BOTH_V137B_ACTUAL_COM_HELD_FLOOR_PD_EXISTING_BUDGET | kneeLandingPreparation=BOTH_V139B_DUAL_BASIS_GUARDED | hipLandingPreparation=BOTH_OFF_V142A | landingResponse=ACTUAL_BOX_NEXT_SETTLED_NO_FEEDBACK | landingGeometry=V141_ORDERED_HEIGHT_ALL_ADOPTED_TOTALS_SAME_STATE_INCREMENT_PROBE | landingOpportunity=V142_ALL_INACTIVE_NO_CONTACT_AUTHORITY | landingExperiment=" + SourceRevision + " | jointFeasibility=BOTH_READ_ONLY_HEIGHT_CEILING_SUBNOISE_SPLIT | allocationCaps=UNCHANGED_NINE_BALLS | setupLocation=MAIN_ENTIRE_SESSION | tangentFeedback=BOTH20_9 | normalResponseEvidence=PREVIOUS_INPUT_TO_CURRENT_SETTLED | stanceWitness=PREVIOUS_COMPLETED_FIXED_AND_RENDER_FRESHNESS | physicalFrameAgeGates=BOTH_RETAINED | entryReferenceAudit=SAME_CORNER_UPSTREAM_PARENT_PLACEMENT_KNEE_RECEIPT | entryWindows=FIRST_ANY_THEN_RESERVE_SOURCE_MOTION | foundationSetup=STABLE_SOURCE_WINDOW | hipJoinPreparation=PROJECTED_PREPARATION_1S_12SOURCE_TIMEOUT15S | hipContactPreparation=SETTLED_CONTACT_LOAD_OFF_150MS_TIMEOUT15S | initialGeometry=PREPARATION_RAW_CONTROL_ACTUAL_MINUS3_MM | contactGeometry=ORDINARY2_RECOVERY3_MM_UNCHANGED | gsp=False");
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            if (!_unloading) ActiveRagdollLabRuntime.OnSceneChanged(scene.name, scene.handle);
        }

        public override bool Unload()
        {
            _unloading = true;
            bool complete;
            try { complete = ActiveRagdollLabRuntime.Shutdown("Plugin.Unload"); }
            catch (Exception e) { UnloadFailure("runtime", e); complete = false; }
            try
            {
                if (_sceneLoadedHandler != null) SceneManager.sceneLoaded -= _sceneLoadedHandler;
                _sceneLoadedHandler = null;
            }
            catch (Exception e) { UnloadFailure("scene", e); complete = false; }
            try
            {
                if (_runner != null)
                {
                    _runner.enabled = false; // Destroy is deferred; prevent a second callback after reload.
                    UnityEngine.Object.Destroy(_runner);
                }
                _runner = null;
            }
            catch (Exception e) { UnloadFailure("runner", e); complete = false; }
            try { _harmony?.UnpatchSelf(); _harmony = null; }
            catch (Exception e) { UnloadFailure("patches", e); complete = false; }
            return complete;
        }

        // Optional diagnostic output may outlive a BepInEx listener during stop or unload.
        internal static void TryLogWarning(string message)
        {
            try { Logger?.LogWarning(message); }
            catch { }
        }

        private static void UnloadFailure(string owner, Exception e)
        {
            TryLogWarning("[ARL] Unload incomplete | owner=" + owner + " | " + e);
        }
    }

    public sealed class ActiveRagdollLabRunner : MonoBehaviour
    {
        private void LateUpdate()
        {
            try { ActiveRagdollLabRuntime.SampleSourcePoseLateUpdate(); }
            finally
            {
                try { ActiveRagdollLabRuntime.FlushAutomaticStopNotice(); }
                finally { ActiveRagdollLabRuntime.FlushDeferredDiagnostics(); }
            }
        }

        private void FixedUpdate()
        {
            ActiveRagdollLabRuntime.FixedUpdate();
        }

        private void OnApplicationQuit()
        {
            ActiveRagdollLabRuntime.Shutdown("ActiveRagdollLabRunner.OnApplicationQuit");
        }
    }
}
