/*
 * TopSurfaceSeatProxyPlugin.cs
 *
 * 旧整合入口兼容层。正式入口已经迁移到 SeatVlmIntegration.cs，
 * 不再注册 ts_* 命令或 GameController.Update 热键测试补丁。
 */

using BepInEx.Logging;
using HarmonyLib;

namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class TopSurfaceSeatProxyIntegration
    {
        public static void Init(ManualLogSource log, Harmony harmony)
            => SeatVlmIntegration.Init(log, harmony);

        public static void Shutdown()
            => SeatVlmIntegration.Shutdown();
    }
}
