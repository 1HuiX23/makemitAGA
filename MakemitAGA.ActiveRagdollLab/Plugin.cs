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
        "0.4.100")]
    public sealed class Plugin : BepInEx.Unity.IL2CPP.BasePlugin
    {
        // V04930 uses the Chest-COM-constrained parent point response and retains committed common acceleration FF.
        internal const string SourceRevision = "V041000_HIP_MOTION_FEASIBILITY_ALLOCATION_20261031";
        internal static ManualLogSource Logger;
        private Harmony _harmony;
        private Action<Scene, LoadSceneMode> _sceneLoadedHandler;
        private bool _shutdown;

        public override void Load()
        {
            Logger = Log;
            Logger.LogInfo("[ARL] Active Ragdoll Lab 0.4.100 loaded | sourceRevision=" + SourceRevision +
                " | V041000 feedback-priority common COM and Upper torque allocation before Knee/Foot; explicit bounded fallback" +
                " | shared session/clock; explicit free-space/unloaded/Foot/Lower-self-weight/Knee-normal/Upper-self-weight/Hip-anchor-pair scopes. The Knee scope retains Foot/Lower weight and adds the existing exact-stance Knee normal pair; Upper gravity only in the Upper/Hip scopes; Torso gravity and tether reciprocal OFF; Hip translation reaction only in the new Hip scope; angular-controller reaction OFF; legacy ankle regression retained.");
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
            Logger.LogInfo("[ARL][BASELINE] build=V041000_HIP_MOTION_FEASIBILITY_ALLOCATION | supportProbe=FINITE_SEGMENT_NO_PADDING | hipAttitude=BASELINE2_PLUS_NET_HIP_FF_TOTAL8_NM | evidence=MOTION_SCALE_FEEDBACK_DEFICIT_FINAL_COMMAND_RESPONSE | physicalArm=B_V041000_HIP_MOTION_FEASIBILITY_ALLOCATION | command=arl_pelvis_hip_test(b|stop) | scope=DYNAMIC_PELVIS_LEGS_HIP_ANCHOR_PAIR_CONTACT | parent=DYNAMIC_PELVIS_SETTLED_POINT | contact=EXACT_STANCE | footGravity=EXACT_STANCE_NATIVE | lowerGravity=EXACT_STANCE_NATIVE | upperGravity=EXACT_STANCE_NATIVE | torsoGravity=OFF | loadTransfer=LOWER_AND_UPPER_SELF_WEIGHT_SERIAL_PAIRS | kneeNormalCoupling=V04430_REDUCED_MASS_PD_PAIR | clocks=TORSO_LEG_FOOT_SHARED_CONTEXT | hipTranslation=FEEDBACK_PRIORITY_BILATERAL_MOTION_ALLOCATION | HipReactionToPelvis=TRANSLATION_AT_HIP_ANCHORS | hipAngularControllerReaction=OFF | gsp=False");
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
            catch (Exception e) { Logger?.LogWarning("[ARL] Shutdown failed: " + e); }
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
