/*
 * 独立运行时原始几何获取实验。
 *
 * 与 Spatial Reconstruction Lab 的关键隔离：
 *   - DLL：MakemitAGA.GeometrySourceProbe.dll
 *   - GUID：com.alex.miside.geometrysourceprobe
 *   - 命令：gsp_*
 *   - 不读取、不修改 SRL 的任何静态状态；只在 GSP 自有临时对象上创建审计预览与固定被动碰撞体，不替换游戏对象。
 */
using System;
using BepInEx;
using BepInEx.Logging;
using HarmonyLib;
using Il2CppInterop.Runtime.Injection;
using MakemitAGA.GeometrySourceProbe.Runtime;
using MakemitAGA.GeometrySourceProbe.Visualization;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MakemitAGA.GeometrySourceProbe
{
    [BepInPlugin(
        "com.alex.miside.geometrysourceprobe",
        "MakemitAGA Geometry Source Probe",
        "0.2.3")]
    public sealed class Plugin : BepInEx.Unity.IL2CPP.BasePlugin
    {
        internal static Plugin Instance;
        internal static ManualLogSource Logger;
        internal static GeometryProbeRunner Runner;

        private Harmony _harmony;
        private Action<Scene, LoadSceneMode> _sceneLoadedHandler;
        private bool _shutdown;

        public override void Load()
        {
            Instance = this;
            Logger = Log;

            Logger.LogInfo(
                "[GSP] Geometry Source Probe 0.2.3 loaded (ACTIVE-ROOM INCREMENTAL FLOOR ATLAS FOUNDATION; ARL-AGNOSTIC). " +
                "Use gsp_help in the game console.");

            try
            {
                ClassInjector.RegisterTypeInIl2Cpp<
                    SpatialInstanceProxyContactTracker>();
                Logger.LogInfo(
                    "[GSP] Registered IL2CPP tracker for controlled instance-proxy collision events; contact surface point/normal are derived by managed OBB geometry.");
            }
            catch (Exception e)
            {
                Logger.LogWarning(
                    "[GSP] Instance-proxy contact tracker registration did not complete " +
                    "(the type may already be registered): " +
                    e.GetType().Name + " / " + e.Message);
            }

            Runner = this.AddComponent<GeometryProbeRunner>();
            GeometrySourceProbeRuntime.Initialize(Logger);

            _sceneLoadedHandler = OnSceneLoaded;
            SceneManager.sceneLoaded += _sceneLoadedHandler;

            _harmony = new Harmony(
                "com.alex.miside.geometrysourceprobe");
            _harmony.PatchAll(typeof(GeometryProbeConsolePatch));
        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            GeometrySourceProbeRuntime.OnSceneChanged(
                scene.name,
                scene.handle);
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

            try
            {
                GeometrySourceProbeRuntime.Shutdown(reason);
            }
            catch (Exception e)
            {
                Logger?.LogWarning(
                    "[GSP] Runtime shutdown failed: " + e.Message);
            }

            try { _harmony?.UnpatchSelf(); }
            catch { }
        }
    }

    public sealed class GeometryProbeRunner : MonoBehaviour
    {
        private void Update()
        {
            GeometrySourceProbeRuntime.UpdateHouseGeometryCensus();
            GeometrySourceProbeRuntime.UpdateMultipartAggregationAudit();
            // floor orchestrator 必须在 MultipartObjectAggregator.Update 之后运行，
            // 才能在同一帧消费刚完成的地板审计快照并启动下一块地板。
            GeometrySourceProbeRuntime.UpdateHouseFloorEnvironment();
            GeometrySourceProbeRuntime.UpdateInstanceProxyContactTest();
        }

        private void FixedUpdate()
        {
            GeometrySourceProbeRuntime.FixedUpdateInstanceProxyContactTest();
        }

        private void OnApplicationQuit()
        {
            Plugin.Instance?.Shutdown(
                "GeometryProbeRunner.OnApplicationQuit");
        }

        private void OnDestroy()
        {
            Plugin.Instance?.Shutdown(
                "GeometryProbeRunner.OnDestroy");
        }
    }
}
