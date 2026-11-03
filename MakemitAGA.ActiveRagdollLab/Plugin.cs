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
        internal const string PluginVersion = "0.4.119";
        // Near-surface full serial stance-load admission; fixed attachment and actuator budgets.
        internal const string SourceRevision = "V041190_STANCE_LOAD_ADMISSION_20261102";
        internal static ManualLogSource Logger;
        private Harmony _harmony;
        private Action<Scene, LoadSceneMode> _sceneLoadedHandler;
        private bool _shutdown;

        public override void Load()
        {
            Logger = Log;
            Logger.LogInfo("[ARL] Active Ragdoll Lab " + PluginVersion + " loaded | sourceRevision=" + SourceRevision +
                " | V041190 near-surface full stance-load admission A/B" +
                " | shared session/clock; explicit free-space/unloaded/Foot/Lower-self-weight/Knee-normal/Upper-self-weight/Hip-anchor-pair scopes. The Knee scope retains Foot/Lower weight and adds the existing exact-stance Knee normal pair; Upper gravity only in the Upper/Hip scopes; Torso gravity and tether reciprocal OFF; Hip translation reaction only in the new Hip scope; joint-relative angular reaction ON in both Hip arms; both Hip arms share continuous Knee R/W/A and the same Foot ground goal; both use bounded planned Knee A during source/torso recovery; B alone delays full serial load until current and one-step predicted sole geometry qualify; legacy ankle regression retained.");
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
            this.AddComponent<ActiveRagdollLabRunner>();

            _sceneLoadedHandler = OnSceneLoaded;
            SceneManager.sceneLoaded += _sceneLoadedHandler;

            _harmony = new Harmony("com.alex.miside.activeragdolllab");
            _harmony.PatchAll(typeof(ActiveRagdollConsolePatch));
            Logger.LogInfo("[ARL][BASELINE] build=V041190_STANCE_LOAD_ADMISSION | supportProbe=FINITE_SEGMENT_NO_PADDING | hipAttitude=BASELINE2_POINT_COMPENSATION_RELATIVE_REACTION_EXTERNAL_AND_TOTAL8_NM | evidence=RAW_PRE24_POST8_STOP_PRE8_POST48_PREVIOUS_RECEIPT_NEXT_RESPONSE | comReference=V105_A_RELATIVE_V_ARRIVAL_DEFICIT_FEEDBACK_ON | physicalArms=A_V041190_DIRECT_STANCE_LOAD/B_V041190_NEAR_SURFACE_LOAD_ADMISSION | logFormat=HIP_COM_V041060 | periodicLogMinSeconds=5 | authorityAudit=STOP_TIME_RETAINED_RECEIPTS | command=arl_pelvis_hip_test(a|b|stop) | scope=DYNAMIC_PELVIS_LEGS_HIP_ANCHOR_PAIR_CONTACT | parent=DYNAMIC_PELVIS_SETTLED_POINT | contact=EXACT_STANCE | footGravity=EXACT_STANCE_NATIVE | lowerGravity=EXACT_STANCE_NATIVE | upperGravity=EXACT_STANCE_NATIVE | torsoGravity=OFF | loadTransfer=LOWER_AND_UPPER_SELF_WEIGHT_SERIAL_PAIRS | kneeNormalCoupling=V04430_REDUCED_MASS_PD_PAIR | clocks=TORSO_LEG_FOOT_SHARED_CONTEXT | hipTranslation=FEEDBACK_PRIORITY_SYSTEM_COM_RESPONSE | HipReactionToPelvis=TRANSLATION_AT_HIP_ANCHORS | hipAngularControllerReaction=BOTH_RELATIVE_ONLY | hipNormalParent=BOTH_RECEIVER_TARGET_NORMAL_ONLY | contactEntry=BOTH_SHALLOW_REQUALIFICATION | footGroundReference=BOTH_FEASIBLE_OR_PARTIAL_RENDER_GOAL | kneeReference=BOTH_CONTINUOUS_UPPER_LOCAL_RWA | kneePlannedFF=BOTH_SOURCE_TORSO_RECOVERY | loadAdmission=A_DIRECT_B_NEAR_SURFACE_TWO_FIXED | physicalFrameAgeGates=BOTH_RETAINED | entryReferenceAudit=SAME_CORNER_UPSTREAM_PARENT_PLACEMENT_KNEE_RECEIPT | entryWindows=FIRST_ANY_THEN_RESERVE_SOURCE_MOTION | gsp=False");
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            ActiveRagdollLabRuntime.OnSceneChanged(scene.name, scene.handle);
        }

        public override bool Unload()
        {
            Shutdown("Plugin.Unload");
            return true;
        }

        internal void Shutdown(string reason)
        {
            if (_shutdown)
                return;
            _shutdown = true;

            try
            {
                if (_sceneLoadedHandler != null)
                    SceneManager.sceneLoaded -= _sceneLoadedHandler;
            }
            catch { }

            try { ActiveRagdollLabRuntime.Shutdown(reason); }
            catch (Exception e)
            {
                // Diagnostic delivery must not prevent the remaining teardown attempt.
                try { Logger?.LogWarning("[ARL] Shutdown failed: " + e); }
                catch { }
            }
            try { _harmony?.UnpatchSelf(); }
            catch { }
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
