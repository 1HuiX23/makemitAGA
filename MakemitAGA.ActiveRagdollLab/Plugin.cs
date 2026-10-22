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
        "0.4.76")]
    public sealed class Plugin : BepInEx.Unity.IL2CPP.BasePlugin
    {
        // V04760 scopes torso recovery to the dynamic plant; both arms retain coherent Foot and Pelvis braking B.
        internal const string SourceRevision = "V04760_DYNAMIC_TORSO_RECOVERY_20261022";
        internal static ManualLogSource Logger;
        private Harmony _harmony;
        private Action<Scene, LoadSceneMode> _sceneLoadedHandler;
        private bool _shutdown;

        public override void Load()
        {
            Logger = Log;
            Logger.LogInfo("[ARL] Active Ragdoll Lab 0.4.76 loaded | sourceRevision=" + SourceRevision +
                " | V04760 torso recovery A/B; coherent Foot reference and Pelvis braking B in both arms; force limits retained" +
                " | shared session/clock; native parent snapshots; free space only. Contact, gravity, load transfer and Hip reaction remain OFF; legacy ankle regression retained.");
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
                Logger.LogInfo("[ARL] Registered Foot/support-Box callback bridge. Joined normal-load/angular/boundary ledgers retained; duplicate V04110/V04130/V04230 and V04400 experiment retired.");
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
            Logger.LogInfo("[ARL][BASELINE] build=V04760_DYNAMIC_TORSO_RECOVERY | command=arl_pelvis_leg_test(a|b|stop) | scope=DYNAMIC_PELVIS_LEGS_FREE_SPACE | parent=DYNAMIC_PELVIS_SETTLED_POINT | contact=OFF | gravity=OFF | loadTransfer=OFF | clocks=TORSO_LEG_FOOT_SHARED_CONTEXT | HipReactionToPelvis=NOT_ROUTED | gsp=False");
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
