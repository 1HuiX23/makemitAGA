/*
 * =================================================================================================
 * SeatVlmIntegration.cs
 * =================================================================================================
 *
 * 正式主项目入口：
 *   - 解析 svt_start <目标>、svt_status、svt_cancel、svt_clear、debug_svt；
 *   - 统一管理场景切换、调试可见性和临时展示清理；
 *   - svt_start 没有默认目标，缺少参数只显示用法；
 *   - svt_clear 永远保留分析 Collider 与动作 Action Proxy Collider。
 * =================================================================================================
 */

using System;
using BepInEx.Logging;
using HarmonyLib;
using MakemitAGA.World;

namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class SeatVlmIntegration
    {
        private static bool _initialized;
        private static bool _debugEnabled;

        public static bool DebugEnabled => _debugEnabled;

        public static void Init(
            ManualLogSource log,
            Harmony harmony)
        {
            if (_initialized)
                return;

            _initialized = true;

            SeatSurfaceAnalysisRuntime.Init(log);
            SeatSurfaceAnalysisRuntime.TryLoadDepthAssets();
            SeatSurfaceVlmPreviewManager.Initialize();

            SetDebugVisible(false);

            log?.LogInfo(
                "[SeatVLM] integrated production pipeline ready." +
                " Commands: svt_start <target>, svt_status, svt_cancel, " +
                "svt_clear, debug_svt, svt_backend_status, svt_backend_restart.");
        }

        public static bool TryHandleConsoleCommand(
            string rawCommand)
        {
            string command =
                (rawCommand ?? "").Trim();

            if (command.Length == 0)
                return false;

            string startTarget;
            bool isStartCommand;

            TryParseStartCommand(
                command,
                out isStartCommand,
                out startTarget);

            if (isStartCommand)
            {
                if (string.IsNullOrWhiteSpace(startTarget))
                {
                    Print(
                        "<color=yellow>用法：svt_start 目标，例如 svt_start 沙发</color>");
                    return true;
                }

                SeatVlmController.StartTarget(
                    startTarget,
                    "console");
                return true;
            }

            if (command.Equals(
                "svt_status",
                StringComparison.OrdinalIgnoreCase))
            {
                Print(
                    SeatVlmController.GetStatusText() +
                    " | debug_svt=" +
                    _debugEnabled);
                return true;
            }

            if (command.Equals(
                "svt_cancel",
                StringComparison.OrdinalIgnoreCase))
            {
                SeatVlmController.Cancel("console");
                ClearTransientArtifacts("cancel", false);
                Print("Seat VLM 已取消。");
                return true;
            }

            if (command.Equals(
                "svt_clear",
                StringComparison.OrdinalIgnoreCase))
            {
                SeatVlmController.ClearCommandState("svt_clear");
                ClearTransientArtifacts("svt_clear", false);
                Print(
                    "Seat VLM 临时展示已清理；分析 Collider 与动作代理 Collider 已保留。");
                return true;
            }

            if (command.Equals(
                    "debug_svt",
                    StringComparison.OrdinalIgnoreCase) ||
                command.StartsWith(
                    "debug_svt ",
                    StringComparison.OrdinalIgnoreCase))
            {
                bool enabled = !_debugEnabled;

                if (command.Length > "debug_svt".Length)
                {
                    string value =
                        command.Substring(
                            "debug_svt".Length)
                        .Trim();

                    if (value.Equals("on", StringComparison.OrdinalIgnoreCase) ||
                        value.Equals("1", StringComparison.OrdinalIgnoreCase) ||
                        value.Equals("true", StringComparison.OrdinalIgnoreCase))
                    {
                        enabled = true;
                    }
                    else if (value.Equals("off", StringComparison.OrdinalIgnoreCase) ||
                             value.Equals("0", StringComparison.OrdinalIgnoreCase) ||
                             value.Equals("false", StringComparison.OrdinalIgnoreCase))
                    {
                        enabled = false;
                    }
                    else
                    {
                        Print("用法：debug_svt 或 debug_svt on/off");
                        return true;
                    }
                }

                SetDebugVisible(enabled);
                Print(
                    "Seat VLM debug visible=" +
                    enabled);
                return true;
            }

            if (command.Equals(
                "svt_backend_status",
                StringComparison.OrdinalIgnoreCase))
            {
                Print(
                    Plugin.Instance == null
                        ? "Plugin.Instance=null"
                        : Plugin.Instance.GetBackendStatus());
                return true;
            }

            if (command.Equals(
                "svt_backend_restart",
                StringComparison.OrdinalIgnoreCase))
            {
                Plugin.Instance?.RestartBackendServer();
                Print("后端重启请求已发送。");
                return true;
            }

            return false;
        }

        public static void ClearTransientArtifacts(
            string reason,
            bool resetController)
        {
            if (resetController)
            {
                SeatVlmController.ClearCommandState(reason);
            }

            SeatVlmDebugVisuals.ClearAll();

            SeatVlmVisionManager.EndSnapshotPreparation();

            SeatSurfaceVlmPreviewManager.CancelAndClear();

            SeatSurfaceAnalysisRuntime
                .ClearTransientVisualsPreserveSurface();

            // svt_clear 的语义包含“退出调试展示”。Collider 保留，所有 Renderer 回到隐藏状态。
            SetDebugVisible(false);

            Plugin.Logger?.LogInfo(
                "[SeatVLM] transient artifacts cleared" +
                " | reason=" + reason +
                " | analysisColliderPreserved=true" +
                " | actionProxyPreserved=true");
        }

        public static void ResetForSceneChange(
            string reason)
        {
            SeatVlmController.ResetForSceneChange(reason);
            SeatVlmVisionManager.ClearSceneReferences();
            SeatVlmObjectDetector.ClearSceneCache();
            SeatVlmDebugVisuals.ClearAll();
            SeatSurfaceVlmPreviewManager.CancelAndClear();
            SeatSurfaceAnalysisRuntime.ClearAll();
            SeatActionProxyRuntime.ClearAll();
            SetDebugVisible(false);
        }

        public static void Shutdown()
        {
            try
            {
                SeatVlmController.Cancel("plugin unload");
                SeatVlmVisionManager.ClearSceneReferences();
                SeatVlmDebugVisuals.ClearAll();
                SeatSurfaceVlmPreviewManager.CancelAndClear();
                SeatSurfaceAnalysisRuntime.ClearAll();
                SeatActionProxyRuntime.ClearAll();
            }
            catch { }
        }

        private static void SetDebugVisible(
            bool visible)
        {
            _debugEnabled = visible;
            SeatVlmDebugVisuals.SetVisible(visible);
            SeatSurfaceAnalysisRuntime.SetDebugRenderersVisible(visible);
            SeatActionProxyRuntime.SetDebugVisible(visible);
        }

        private static void TryParseStartCommand(
            string command,
            out bool isStartCommand,
            out string target)
        {
            isStartCommand = false;
            target = null;

            if (command.Equals(
                "svt_start",
                StringComparison.OrdinalIgnoreCase))
            {
                isStartCommand = true;
                return;
            }

            if (command.StartsWith(
                "svt_start ",
                StringComparison.OrdinalIgnoreCase))
            {
                isStartCommand = true;
                target = command.Substring(10).Trim();
                return;
            }

            if (command.StartsWith(
                    "svt_start(",
                    StringComparison.OrdinalIgnoreCase) &&
                command.EndsWith(")",
                    StringComparison.Ordinal))
            {
                isStartCommand = true;
                target = command.Substring(
                    10,
                    command.Length - 11)
                    .Trim()
                    .Trim('"', '\'');
            }
        }

        private static void Print(
            string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); }
            catch { }
        }
    }
}
