/*
 * Plugin.cs
 * MakemitAGA 0.2.3：后端 UTF-8、可靠重启与退出清理修复版。
 *
 * 保留原有模块初始化顺序，只重写 OnlineAIApiServer.exe 生命周期：
 * 1. 启动时强制 Python UTF-8；
 * 2. 传递 MiSide 父进程 PID 和随机 shutdown token；
 * 3. 停止时先请求 /shutdown，再 Kill(entireProcessTree: true) 兜底；
 * 4. MainThreadRunner.OnApplicationQuit / OnDestroy 负责可靠退出通知；
 * 5. 首次启动清理旧版遗留的同名孤儿后端进程。
 */

using System;
using System.Diagnostics;
using System.IO;
using System.Net.Http;
using System.Text;
using System.Threading;
using BepInEx;
using BepInEx.Logging;
using HarmonyLib;
using MakemitAGA.Dialogue;
using MakemitAGA.Mita_self;
using MakemitAGA.Mita_self.Mita_tools;
using MakemitAGA.World;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MakemitAGA
{
    [BepInPlugin(
        "com.yourname.miside.freedialogue",
        "Miside AI Modular",
        "0.2.3")]
    public class Plugin : BepInEx.Unity.IL2CPP.BasePlugin
    {
        private const string BackendExeName = "OnlineAIApiServer.exe";
        private const string BackendProcessName = "OnlineAIApiServer";
        private const string BackendShutdownUrl =
            "http://127.0.0.1:8080/shutdown";

        private const int GracefulShutdownRequestTimeoutMs = 1500;
        private const int GracefulExitWaitMs = 3000;
        private const int ForcedExitWaitMs = 3000;

        private static readonly HttpClient ShutdownHttpClient =
            new HttpClient
            {
                Timeout = TimeSpan.FromMilliseconds(
                    GracefulShutdownRequestTimeoutMs)
            };

        internal static ManualLogSource Logger;
        internal static Plugin Instance;

        public static MainThreadRunner Runner { get; private set; }

        private Process _backendProcess;
        private bool _backendExitReported;
        private bool _shutdownStarted;
        private bool _startupStaleCleanupDone;
        private string _backendShutdownToken;

        public override void Load()
        {
            Logger = Log;
            Instance = this;

            try
            {
                Console.OutputEncoding = Encoding.UTF8;
                Console.InputEncoding = Encoding.UTF8;
            }
            catch { }

            Logger.LogInfo(
                "Miside AI Modular 0.2.3（后端生命周期修复版）已启动!");

            ClothChange.Init();

            SceneManager.sceneLoaded +=
                (Action<Scene, LoadSceneMode>)OnSceneLoaded;

            Runner = this.AddComponent<MainThreadRunner>();

            // Formula3D 只在这里保存日志引用，不会在菜单阶段立即寻找场景对象。
            // 真正的 DialogueQuest Mita 模板会在第一次显示文字时延迟捕获。
            GameUIManager.Initialize(Logger);

            // 只注册类型，不在插件加载阶段创建依赖场景对象的组件。
            Mita_sit.EnsureIl2CppTypeRegistered();

            StartBackendServer();

            var harmony =
                new Harmony("com.yourname.miside.freedialogue");

            SeatVlmIntegration.Init(Logger, harmony);
            harmony.PatchAll(typeof(VisionPatches));
            harmony.PatchAll(typeof(DialoguePatches));
            harmony.PatchAll(typeof(Patch_Location3WalkToToilet));
            harmony.PatchAll(typeof(Patch_AnimatorFunctions));
            harmony.PatchAll(typeof(MitaSitOwnershipPatches));
            harmony.PatchAll(typeof(ClothesPatch));
        }

        private void OnSceneLoaded(
            Scene scene,
            LoadSceneMode mode)
        {
            EnvironmentManager.ClearState();

            // Dialogue_3DText、Symbol、Texture2D 都属于当前 Unity 场景。
            // 必须先取消旧协程并清除旧模板，避免 MissingReferenceException。
            GameUIManager.OnSceneChanged(
                scene.name,
                scene.handle);

            SeatVlmIntegration.ResetForSceneChange(
                "sceneLoaded: " +
                scene.name +
                " / " +
                scene.handle);

            EnvironmentManager.Init();
        }

        public override bool Unload()
        {
            ShutdownForGameExit("Plugin.Unload");
            return true;
        }

        /// <summary>
        /// 由真正挂在 Unity GameObject 上的 MainThreadRunner 调用。
        /// OnApplicationQuit、OnDestroy、Unload 同时触发时只执行一次。
        /// </summary>
        internal void ShutdownForGameExit(string reason)
        {
            if (_shutdownStarted)
                return;

            _shutdownStarted = true;

            Logger?.LogInfo(
                "[Shutdown] begin | reason=" +
                (reason ?? "<unknown>"));

            try
            {
                SeatVlmIntegration.Shutdown();
            }
            catch (Exception e)
            {
                Logger?.LogWarning(
                    "[Shutdown] SeatVLM cleanup failed: " +
                    e.Message);
            }

            try
            {
                // 销毁 Formula3D 私有模板、当前句、运行时纹理和协程。
                GameUIManager.Shutdown(reason ?? "game-exit");
            }
            catch (Exception e)
            {
                Logger?.LogWarning(
                    "[Shutdown] Formula3D cleanup failed: " +
                    e.Message);
            }

            StopBackendServer(
                reason ?? "game-exit",
                requestGracefulShutdown: true);

            Logger?.LogInfo("[Shutdown] complete");
        }

        internal void StartBackendServer()
        {
            CleanupLegacyBackendDiagnosticFiles();

            string exePath =
                Path.Combine(
                    Paths.PluginPath,
                    BackendExeName);

            if (!File.Exists(exePath))
            {
                Logger.LogWarning(
                    "[Backend] 未找到：" +
                    exePath);
                return;
            }

            // 本次 MiSide 进程只执行一次。
            // 负责清理旧版本残留的孤儿 OnlineAIApiServer.exe。
            if (!_startupStaleCleanupDone)
            {
                _startupStaleCleanupDone = true;
                CleanupStaleBackendProcessesAtStartup();
            }

            try
            {
                if (_backendProcess != null &&
                    !_backendProcess.HasExited)
                {
                    Logger.LogInfo(
                        "[Backend] start ignored; tracked process is running" +
                        " | pid=" +
                        _backendProcess.Id);
                    return;
                }
            }
            catch
            {
                DisposeTrackedBackendProcess();
            }

            try
            {
                _backendExitReported = false;
                _backendShutdownToken =
                    Guid.NewGuid().ToString("N");

                var startInfo =
                    new ProcessStartInfo
                    {
                        FileName = exePath,
                        WorkingDirectory = Paths.PluginPath,
                        UseShellExecute = false,
                        CreateNoWindow = true,
                        RedirectStandardOutput = true,
                        RedirectStandardError = true,
                        StandardOutputEncoding = Encoding.UTF8,
                        StandardErrorEncoding = Encoding.UTF8
                    };

                // StandardOutputEncoding 只负责 C# 解码。
                // 下面两个变量才会要求 Python 自己按 UTF-8 写 stdout/stderr。
                startInfo.Environment["PYTHONUTF8"] = "1";
                startInfo.Environment["PYTHONIOENCODING"] =
                    "utf-8:backslashreplace";

                startInfo.Environment["MISIDE_CONFIG_PATH"] =
                    ClothChange.ConfigPathForBackend;

                startInfo.Environment["MISIDE_CACHE_PATH"] =
                    Path.Combine(
                        Paths.PluginPath,
                        "cache.jpg");

                // Python watchdog 会监视 MiSide 父进程。
                // 即使 Unity 退出回调没有运行，父进程消失后后端也会自停。
                startInfo.Environment["MISIDE_PARENT_PID"] =
                    Environment.ProcessId.ToString();

                // 只有当前游戏实例知道的随机关闭令牌。
                startInfo.Environment["MISIDE_SHUTDOWN_TOKEN"] =
                    _backendShutdownToken;

                _backendProcess =
                    new Process
                    {
                        StartInfo = startInfo,
                        EnableRaisingEvents = true
                    };

                _backendProcess.OutputDataReceived +=
                    delegate (
                        object sender,
                        DataReceivedEventArgs e)
                    {
                        if (!string.IsNullOrEmpty(e.Data))
                        {
                            Logger?.LogInfo(
                                "[Backend] " +
                                e.Data);
                        }
                    };

                _backendProcess.ErrorDataReceived +=
                    delegate (
                        object sender,
                        DataReceivedEventArgs e)
                    {
                        if (!string.IsNullOrEmpty(e.Data))
                        {
                            Logger?.LogError(
                                "[Backend] " +
                                e.Data);
                        }
                    };

                _backendProcess.Start();
                _backendProcess.BeginOutputReadLine();
                _backendProcess.BeginErrorReadLine();

                Logger.LogInfo(
                    "Python后端已启动。" +
                    "pid=" +
                    _backendProcess.Id +
                    " | parentPid=" +
                    Environment.ProcessId);
            }
            catch (Exception e)
            {
                Logger.LogError(
                    "后端启动失败: " +
                    e);

                DisposeTrackedBackendProcess();
            }
        }

        /// <summary>
        /// 保持原有 void 接口，SeatVlmIntegration 无需修改。
        /// 现在会真正等待旧后端停止后再启动新后端。
        /// </summary>
        internal void RestartBackendServer()
        {
            int oldPid = TryGetTrackedBackendPid();

            Logger?.LogInfo(
                "[Backend] restart begin" +
                " | oldPid=" +
                oldPid);

            StopBackendServer(
                "console-restart",
                requestGracefulShutdown: true);

            StartBackendServer();

            Logger?.LogInfo(
                "[Backend] restart launch complete" +
                " | oldPid=" +
                oldPid +
                " | newStatus=" +
                GetBackendStatus());
        }

        internal string GetBackendStatus()
        {
            int totalNamedProcesses =
                CountBackendProcesses();

            if (_backendProcess == null)
            {
                return
                    "backend=<null>" +
                    " | namedProcesses=" +
                    totalNamedProcesses;
            }

            try
            {
                return _backendProcess.HasExited
                    ? "backend=exited code=" +
                      _backendProcess.ExitCode +
                      " | namedProcesses=" +
                      totalNamedProcesses
                    : "backend=running pid=" +
                      _backendProcess.Id +
                      " | namedProcesses=" +
                      totalNamedProcesses;
            }
            catch (Exception e)
            {
                return
                    "backend=status-error " +
                    e.Message +
                    " | namedProcesses=" +
                    totalNamedProcesses;
            }
        }

        internal void PollBackendProcess()
        {
            if (_backendProcess == null ||
                _backendExitReported)
            {
                return;
            }

            try
            {
                if (!_backendProcess.HasExited)
                    return;

                _backendExitReported = true;

                Logger.LogError(
                    "[Backend] 进程提前退出" +
                    " | pid=" +
                    _backendProcess.Id +
                    " | exitCode=" +
                    _backendProcess.ExitCode);
            }
            catch
            {
                // 退出阶段 Process 可能已经 Dispose。
            }
        }

        private void StopBackendServer(
            string reason,
            bool requestGracefulShutdown)
        {
            Process process = _backendProcess;
            int pid = TryGetTrackedBackendPid();

            // 先通知真正监听 8080 的 Python server。
            bool gracefulRequested = false;
            if (requestGracefulShutdown)
            {
                gracefulRequested =
                    TryRequestGracefulShutdown(reason);
            }

            if (process != null)
            {
                try
                {
                    if (!process.HasExited)
                    {
                        if (gracefulRequested)
                        {
                            Logger?.LogInfo(
                                "[Backend] graceful shutdown requested" +
                                " | reason=" +
                                reason +
                                " | pid=" +
                                pid);

                            process.WaitForExit(
                                GracefulExitWaitMs);
                        }

                        if (!process.HasExited)
                        {
                            Logger?.LogWarning(
                                "[Backend] graceful shutdown incomplete;" +
                                " killing process tree" +
                                " | pid=" +
                                pid);

                            process.Kill(
                                entireProcessTree: true);

                            if (!process.WaitForExit(
                                ForcedExitWaitMs))
                            {
                                Logger?.LogWarning(
                                    "[Backend] process tree still did not" +
                                    " report exit within timeout" +
                                    " | pid=" +
                                    pid);
                            }
                        }
                    }

                    int exitCode =
                        process.HasExited
                            ? process.ExitCode
                            : int.MinValue;

                    Logger?.LogInfo(
                        "[Backend] stopped" +
                        " | reason=" +
                        reason +
                        " | pid=" +
                        pid +
                        " | exitCode=" +
                        exitCode);
                }
                catch (Exception e)
                {
                    Logger?.LogError(
                        "[Backend] stop failed" +
                        " | reason=" +
                        reason +
                        " | pid=" +
                        pid +
                        " | error=" +
                        e);
                }
            }
            else if (!gracefulRequested)
            {
                Logger?.LogInfo(
                    "[Backend] stop: no tracked process" +
                    " | reason=" +
                    reason);
            }

            DisposeTrackedBackendProcess();
            _backendShutdownToken = null;
            _backendExitReported = false;
        }

        private bool TryRequestGracefulShutdown(
            string reason)
        {
            if (string.IsNullOrWhiteSpace(
                _backendShutdownToken))
            {
                return false;
            }

            try
            {
                using (var request =
                    new HttpRequestMessage(
                        HttpMethod.Post,
                        BackendShutdownUrl))
                {
                    request.Headers.TryAddWithoutValidation(
                        "X-MiSide-Shutdown-Token",
                        _backendShutdownToken);

                    request.Content =
                        new StringContent(
                            reason ?? "",
                            Encoding.UTF8,
                            "text/plain");

                    using (var cts =
                        new CancellationTokenSource(
                            GracefulShutdownRequestTimeoutMs))
                    using (HttpResponseMessage response =
                        ShutdownHttpClient
                            .SendAsync(request, cts.Token)
                            .GetAwaiter()
                            .GetResult())
                    {
                        return response.IsSuccessStatusCode;
                    }
                }
            }
            catch (Exception e)
            {
                Logger?.LogWarning(
                    "[Backend] graceful shutdown request failed: " +
                    e.GetType().Name +
                    " / " +
                    e.Message);
                return false;
            }
        }

        private int TryGetTrackedBackendPid()
        {
            try
            {
                return _backendProcess?.Id ?? -1;
            }
            catch
            {
                return -1;
            }
        }

        private void DisposeTrackedBackendProcess()
        {
            Process process = _backendProcess;
            _backendProcess = null;

            if (process == null)
                return;

            try { process.CancelOutputRead(); }
            catch { }

            try { process.CancelErrorRead(); }
            catch { }

            try { process.Dispose(); }
            catch { }
        }

        private static int CountBackendProcesses()
        {
            try
            {
                Process[] processes =
                    Process.GetProcessesByName(
                        BackendProcessName);

                int count = processes.Length;

                for (int i = 0;
                     i < processes.Length;
                     i++)
                {
                    try { processes[i].Dispose(); }
                    catch { }
                }

                return count;
            }
            catch
            {
                return -1;
            }
        }

        private static void CleanupStaleBackendProcessesAtStartup()
        {
            Process[] processes;

            try
            {
                processes =
                    Process.GetProcessesByName(
                        BackendProcessName);
            }
            catch (Exception e)
            {
                Logger?.LogWarning(
                    "[Backend] stale-process scan failed: " +
                    e.Message);
                return;
            }

            int killed = 0;

            for (int i = 0;
                 i < processes.Length;
                 i++)
            {
                Process process = processes[i];

                try
                {
                    if (process.HasExited)
                        continue;

                    int pid = process.Id;

                    process.Kill(
                        entireProcessTree: true);

                    process.WaitForExit(
                        ForcedExitWaitMs);

                    killed++;

                    Logger?.LogWarning(
                        "[Backend] removed stale process" +
                        " | pid=" +
                        pid);
                }
                catch (Exception e)
                {
                    Logger?.LogWarning(
                        "[Backend] unable to remove stale process: " +
                        e.Message);
                }
                finally
                {
                    try { process.Dispose(); }
                    catch { }
                }
            }

            if (killed > 0)
            {
                // 给 Windows 一点时间释放 8080。
                Thread.Sleep(250);

                Logger?.LogInfo(
                    "[Backend] stale-process cleanup complete" +
                    " | killed=" +
                    killed);
            }
        }

        private static void CleanupLegacyBackendDiagnosticFiles()
        {
            string[] legacyNames =
            {
                "backend_boot.log",
                "backend_last_prompt.txt",
                "backend_last_reply.txt",
                "backend_last_error.txt",
                "backend_startup_error.txt"
            };

            int deleted = 0;

            for (int i = 0;
                 i < legacyNames.Length;
                 i++)
            {
                try
                {
                    string path =
                        Path.Combine(
                            Paths.PluginPath,
                            legacyNames[i]);

                    if (!File.Exists(path))
                        continue;

                    File.Delete(path);
                    deleted++;
                }
                catch (Exception e)
                {
                    Logger?.LogWarning(
                        "[Backend] 无法删除旧诊断文件 " +
                        legacyNames[i] +
                        "：" +
                        e.Message);
                }
            }

            if (deleted > 0)
            {
                Logger?.LogInfo(
                    "[Backend] 已清理旧版分散诊断文件：" +
                    deleted);
            }
        }
    }

    /// <summary>
    /// 这是实际存在于 Unity GameObject 上的 MonoBehaviour。
    /// BasePlugin 中的私有 OnApplicationQuit 不应被当作可靠 Unity 回调，
    /// 因此退出清理放在这里。
    /// </summary>
    public class MainThreadRunner : MonoBehaviour
    {
        private void Update()
        {
            Plugin.Instance?.PollBackendProcess();

            // Seat VLM 的后台结果只能在 Unity 主线程消费。
            SeatVlmController.Tick();

            // 关键：公式使用 RawImage 显示，但入场/淡入动画仍由原生
            // Dialogue_Symbol 修改隐藏的 UI.Text 颜色。这里必须每帧把
            // Text 的颜色（尤其 Alpha）同步给 RawImage，否则公式会占位
            // 但始终完全透明。不要删除或降低为低频调用。
            GameUIManager.Tick();
        }

        private void OnApplicationQuit()
        {
            Plugin.Instance?.ShutdownForGameExit(
                "MainThreadRunner.OnApplicationQuit");
        }

        private void OnDestroy()
        {
            Plugin.Instance?.ShutdownForGameExit(
                "MainThreadRunner.OnDestroy");
        }
    }
}