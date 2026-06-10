/*
 * =================================================================================================
 * TopSurfaceSeatProxyPlugin.cs
 * =================================================================================================
 *
 * 主项目整合版入口。
 *
 * 注意：
 *   这个文件不再是独立 BepInEx 插件，不再监听 F3/F9/Delete 等快捷键。
 *   它只提供 TopSurfaceSeatProxyIntegration.Init(...)，由 MakemitAGA.Plugin.Load() 主动调用。
 *
 * 在 MakemitAGA.Plugin.Load() 里创建 Harmony 后加入：
 *
 *     TopSurfaceSeatProxyIntegration.Init(Logger, harmony);
 *
 * 当前只保留 6 个控制台指令：
 *   ts_scan()              -> 默认 FakeCollider(0.5, 0.5, 0.08)
 *   ts_scan(,0.9,)         -> width 默认，depth=0.9，height 默认
 *   ts_scan(1,1,1)         -> FakeCollider(1, 1, 1)
 *   ts_bed_sit             -> 扫描 hardcoded Bed top，生成有方向的唯一 seat proxy，延迟桥接 Mita_sit
 *   ts_scan_top            -> FakeCollider(Top)
 *   ts_clear               -> 清除代理/调试物体
 *   ts_hide                -> 隐藏调试 renderers
 *   ts_show                -> 显示调试 renderers
 *
 * AssetBundle：
 *   使用 MakemitAGA 同一个 BepInEx/plugins/mita_actions。
 *   需要把 depthseat 的三个资源也打进 mita_actions：
 *     depthseat_eyedepthreplacement.shader
 *     depthtoeye.shader
 *     depthtoeye_mat.mat
 *
 * =================================================================================================
 */

using System;
using System.Globalization;
using BepInEx.Logging;
using HarmonyLib;

namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class TopSurfaceSeatProxyIntegration
    {
        private static bool _initialized;

        public static void Init(ManualLogSource log, Harmony harmony)
        {
            if (_initialized) return;
            _initialized = true;

            TopSurfaceSeatProxyRuntime.Init(log);

            log?.LogWarning("[TopSurfaceSeatProxy] Integrated init entered. v0.9.1-main");
            log?.LogWarning("[TopSurfaceSeatProxy] Hotkeys disabled. Console commands: ts_scan, ts_bed_sit, ts_scan_top, ts_clear, ts_hide, ts_show.");

            try { harmony.PatchAll(typeof(TopSurfaceSeatProxyGameControllerUpdatePatch)); }
            catch (Exception e) { log?.LogError("[TopSurfaceSeatProxy] GameController.Update patch failed: " + e); }

            try { harmony.PatchAll(typeof(TopSurfaceSeatProxyConsoleCommandPatch)); }
            catch (Exception e) { log?.LogError("[TopSurfaceSeatProxy] ConsoleCommandsGame.Command patch failed: " + e); }

            TopSurfaceSeatProxyRuntime.TryLoadDepthAssets();
        }

        public static void Shutdown()
        {
            try { TopSurfaceSeatProxyRuntime.ClearAll(); } catch { }
        }
    }

    [HarmonyPatch(typeof(GameController), "Update")]
    internal static class TopSurfaceSeatProxyGameControllerUpdatePatch
    {
        private static void Postfix()
        {
            TopSurfaceSeatProxyRuntime.TickFromPatch();
        }
    }

    [HarmonyPatch(typeof(ConsoleCommandsGame), "Command")]
    internal static class TopSurfaceSeatProxyConsoleCommandPatch
    {
        private static bool Prefix(string code)
        {
            if (string.IsNullOrWhiteSpace(code)) return true;

            string name;
            string[] args;
            if (!TryParseCommand(code, out name, out args))
                return true;

            if (name.Equals("ts_scan", StringComparison.OrdinalIgnoreCase))
            {
                float width = 0.50f;
                float depth = 0.50f;
                float height = 0.08f;

                if (!TryApplyOptionalFloatArgs(args, ref width, ref depth, ref height))
                {
                    TryConsolePrint("<color=red>ts_scan 参数错误。示例：ts_scan(), ts_scan(,0.9,), ts_scan(1,1,1)</color>");
                    return false;
                }

                TopSurfaceSeatProxyRuntime.ScanFromScreenCenter(
                    "console-ts_scan",
                    FakeColliderRequest.Fixed(width, depth, height));

                TryConsolePrint("TopSurfaceSeatProxy: FakeCollider(" +
                                width.ToString("0.###", CultureInfo.InvariantCulture) + "," +
                                depth.ToString("0.###", CultureInfo.InvariantCulture) + "," +
                                height.ToString("0.###", CultureInfo.InvariantCulture) + ") requested.");
                return false;
            }

            if (name.Equals("ts_bed_sit", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.ScanHardcodedBedTopMeshAndSit("console-ts_bed_sit");
                TryConsolePrint("TopSurfaceSeatProxy: Bed top sit bridge requested.");
                return false;
            }

            if (name.Equals("ts_scan_top", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.ScanFromScreenCenter("console-ts_scan_top", FakeColliderRequest.Top(0.08f));
                TryConsolePrint("TopSurfaceSeatProxy: FakeCollider(Top) requested.");
                return false;
            }

            if (name.Equals("ts_clear", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.ClearAll();
                TryConsolePrint("TopSurfaceSeatProxy: cleared.");
                return false;
            }

            if (name.Equals("ts_hide", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.SetDebugRenderersVisible(false);
                TryConsolePrint("TopSurfaceSeatProxy: debug renderers hidden.");
                return false;
            }

            if (name.Equals("ts_show", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.SetDebugRenderersVisible(true);
                TryConsolePrint("TopSurfaceSeatProxy: debug renderers shown.");
                return false;
            }

            return true;
        }

        private static bool TryParseCommand(string code, out string name, out string[] args)
        {
            name = null;
            args = Array.Empty<string>();

            string trimmed = code.Trim();
            if (string.IsNullOrEmpty(trimmed))
                return false;

            int open = trimmed.IndexOf('(');
            if (open < 0)
            {
                name = trimmed;
                return true;
            }

            int close = trimmed.LastIndexOf(')');
            if (close < open)
                return false;

            name = trimmed.Substring(0, open).Trim();

            string tail = trimmed.Substring(close + 1).Trim();
            if (!string.IsNullOrEmpty(tail))
                return false;

            string inside = trimmed.Substring(open + 1, close - open - 1);
            args = inside.Split(',');
            return !string.IsNullOrWhiteSpace(name);
        }

        private static bool TryApplyOptionalFloatArgs(string[] args, ref float width, ref float depth, ref float height)
        {
            if (args == null || args.Length == 0)
                return true;

            if (args.Length > 3)
                return false;

            float[] values = new float[] { width, depth, height };

            for (int i = 0; i < args.Length; i++)
            {
                string s = args[i] == null ? "" : args[i].Trim();

                // 空参数表示沿用默认值：
                //   ts_scan(,0.9,) -> width 默认，depth=0.9，height 默认
                if (string.IsNullOrEmpty(s))
                    continue;

                float value;
                if (!float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out value))
                    return false;

                if (value <= 0f || float.IsNaN(value) || float.IsInfinity(value))
                    return false;

                values[i] = value;
            }

            width = values[0];
            depth = values[1];
            height = values[2];
            return true;
        }

        private static void TryConsolePrint(string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); } catch { }
        }
    }
}
