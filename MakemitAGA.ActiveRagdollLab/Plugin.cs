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
        "0.4.89")]
    public sealed class Plugin : BepInEx.Unity.IL2CPP.BasePlugin
    {
        // V04890 audits committed Hip wrenches without changing the V04880 physical scope.
        internal const string SourceRevision = "V04890_DYNAMIC_HIP_WRENCH_AUDIT_20261027";
        internal static ManualLogSource Logger;
        private Harmony _harmony;
        private Action<Scene, LoadSceneMode> _sceneLoadedHandler;
        private bool _shutdown;

        public override void Load()
        {
            Logger = Log;
            Logger.LogInfo("[ARL] Active Ragdoll Lab 0.4.89 loaded | sourceRevision=" + SourceRevision +
                " | V04890 read-only dynamic Hip wrench/point/power audit; V04880 physical plant and FIX1/FIX2 retained" +
                " | shared session/clock; explicit free-space/unloaded/Foot/Lower-self-weight/Knee-normal/Upper-self-weight scopes. The Knee scope retains Foot/Lower weight and adds the existing exact-stance Knee normal pair; Upper gravity only in the Upper scope; Torso gravity, tether reciprocal and Hip reaction OFF; legacy ankle regression retained.");
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
            Logger.LogInfo("[ARL][BASELINE] build=V04890_DYNAMIC_HIP_WRENCH_AUDIT | command=arl_pelvis_upper_load_test(b|stop) | scope=DYNAMIC_PELVIS_LEGS_UPPER_SELF_WEIGHT_CONTACT | parent=DYNAMIC_PELVIS_SETTLED_POINT | contact=EXACT_STANCE | footGravity=EXACT_STANCE_NATIVE | lowerGravity=EXACT_STANCE_NATIVE | upperGravity=EXACT_STANCE_NATIVE | torsoGravity=OFF | loadTransfer=LOWER_AND_UPPER_SELF_WEIGHT_SERIAL_PAIRS | kneeNormalCoupling=V04430_REDUCED_MASS_PD_PAIR | clocks=TORSO_LEG_FOOT_SHARED_CONTEXT | hipWrenchAudit=READ_ONLY_V04890 | HipReactionToPelvis=NOT_ROUTED | gsp=False");
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
