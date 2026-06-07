/*
 * =================================================================================================
 * TopSurfaceSeatProxyPlugin.cs
 * =================================================================================================
 *
 * 这个文件负责：
 *   1. BepInEx 插件入口。
 *   2. Harmony patch GameController.Update。
 *   3. Harmony patch ConsoleCommandsGame.Command。
 *   4. 把按键/控制台命令转发给 TopSurfaceSeatProxyRuntime。
 *
 * 为什么拆出来：
 *   这个文件通常不需要频繁改。真正会反复调整的是 Runtime 里的扫描算法、surfaceMask、
 *   FakeCollider 生成策略。
 *
 * 当前测试环境：
 *   游戏：MiSide
 *   Unity：2021.3.35
 *   打包：IL2CPP
 *   Mod 框架：BepInEx.Unity.IL2CPP-win-x64-6.0.0-pre.2
 *   平台：Windows
 *
 * 命令：
 *   TAB / F9       默认 FakeCollider(0.5, 0.5, 0.08)
 *   DELETE         清除
 *
 *   ts_scan        默认 FakeCollider(0.5, 0.5, 0.08)
 *   ts_scan_05     默认 FakeCollider(0.5, 0.5, 0.08)
 *   ts_scan_1      FakeCollider(1.0, 1.0, 0.08)
 *   ts_scan_top    FakeCollider(Top)
 *   ts_top         FakeCollider(Top)
 *   ts_clear       清除
 *   ts_hide        隐藏调试 renderer，只保留 collider
 *   ts_show        显示调试 renderer
 *
 * 重要说明：
 *   FakeCollider(Top) 不是完整 3D whole。
 *   它的含义是：扫描目标物体的 top-view 上方可见表面，然后生成一个 top-surface 代理。
 *   真正的 FakeCollider(whole) 未来应该走多视角 / TSDF / voxel / marching cubes 或多 box tiling。
 *
 * =================================================================================================
 */

using System;
using BepInEx;
using BepInEx.Logging;
using BepInEx.Unity.IL2CPP;
using HarmonyLib;

namespace TopSurfaceSeatProxyTest
{
    [BepInPlugin("com.test.topsurfaceseatproxy.ours", "Top Surface Seat Proxy Tester Ours", "0.7.0-ours")]
    public class TopSurfaceSeatProxyTesterPlugin : BasePlugin
    {
        internal static ManualLogSource LogSource;
        private Harmony _harmony;

        public override void Load()
        {
            LogSource = Log;
            TopSurfaceSeatProxyRuntime.Init(LogSource);

            LogSource.LogWarning("[TopSurfaceSeatProxyTester] Load entered. v0.7.0-ours");
            LogSource.LogWarning("[TopSurfaceSeatProxyTester] TAB/F9 = FakeCollider(0.5,0.5,0.08); DELETE = clear.");
            LogSource.LogWarning("[TopSurfaceSeatProxyTester] Console: ts_scan / ts_scan_05 / ts_scan_1 / ts_scan_top / ts_clear / ts_hide / ts_show.");

            _harmony = new Harmony("com.test.topsurfaceseatproxy.ours.v070");

            try
            {
                _harmony.PatchAll(typeof(GameControllerUpdatePatch));
                LogSource.LogWarning("[TopSurfaceSeatProxyTester] GameController.Update patch requested.");
            }
            catch (Exception e)
            {
                LogSource.LogError("[TopSurfaceSeatProxyTester] Failed to patch GameController.Update: " + e);
            }

            try
            {
                _harmony.PatchAll(typeof(ConsoleCommandPatch));
                LogSource.LogWarning("[TopSurfaceSeatProxyTester] ConsoleCommandsGame.Command patch requested.");
            }
            catch (Exception e)
            {
                LogSource.LogError("[TopSurfaceSeatProxyTester] Failed to patch ConsoleCommandsGame.Command: " + e);
            }

            TopSurfaceSeatProxyRuntime.TryLoadDepthAssets();
        }

        public override bool Unload()
        {
            try { _harmony?.UnpatchSelf(); } catch { }
            try { TopSurfaceSeatProxyRuntime.ClearAll(); } catch { }

            return true;
        }
    }

    [HarmonyPatch(typeof(GameController), "Update")]
    internal static class GameControllerUpdatePatch
    {
        private static void Postfix()
        {
            TopSurfaceSeatProxyRuntime.TickFromPatch();
        }
    }

    [HarmonyPatch(typeof(ConsoleCommandsGame), "Command")]
    internal static class ConsoleCommandPatch
    {
        private static bool Prefix(string code)
        {
            if (string.IsNullOrWhiteSpace(code))
                return true;

            string cmd = code.Trim();

            if (cmd.Equals("ts_scan", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("top_scan", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("seat_scan", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("ts_scan_05", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.ScanFromScreenCenter("console", FakeColliderRequest.DefaultSeat());
                TryConsolePrint("TopSurfaceSeatProxy: FakeCollider(0.5,0.5,0.08) requested.");
                return false;
            }

            if (cmd.Equals("ts_scan_1", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("ts_scan_100", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.ScanFromScreenCenter("console", FakeColliderRequest.Fixed(1.0f, 1.0f, 0.08f));
                TryConsolePrint("TopSurfaceSeatProxy: FakeCollider(1,1,0.08) requested.");
                return false;
            }

            if (cmd.Equals("ts_scan_top", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("ts_top", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("top_proxy", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.ScanFromScreenCenter("console", FakeColliderRequest.Top(0.08f));
                TryConsolePrint("TopSurfaceSeatProxy: FakeCollider(Top) requested.");
                return false;
            }

            if (cmd.Equals("ts_clear", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("top_clear", StringComparison.OrdinalIgnoreCase) ||
                cmd.Equals("seat_clear", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.ClearAll();
                TryConsolePrint("TopSurfaceSeatProxy: cleared.");
                return false;
            }

            if (cmd.Equals("ts_crosshair", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.RecreateCrosshairFromConsole();
                TryConsolePrint("TopSurfaceSeatProxy: crosshair recreated.");
                return false;
            }

            if (cmd.Equals("ts_hide", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.SetDebugRenderersVisible(false);
                TryConsolePrint("TopSurfaceSeatProxy: debug renderers hidden.");
                return false;
            }

            if (cmd.Equals("ts_show", StringComparison.OrdinalIgnoreCase))
            {
                TopSurfaceSeatProxyRuntime.SetDebugRenderersVisible(true);
                TryConsolePrint("TopSurfaceSeatProxy: debug renderers shown.");
                return false;
            }

            return true;
        }

        private static void TryConsolePrint(string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); } catch { }
        }
    }
}