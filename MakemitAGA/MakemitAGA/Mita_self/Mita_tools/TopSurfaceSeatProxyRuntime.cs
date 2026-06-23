/*
 * TopSurfaceSeatProxyRuntime.cs
 *
 * 兼容层：旧代码名称继续可解析，但真正的网状分析面生成逻辑已经迁移到
 * World/SeatSurfaceAnalysisMesh.cs。这里不再包含任何 Mesh 生成实现。
 */

using BepInEx.Logging;
using MakemitAGA.World;

namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class TopSurfaceSeatProxyRuntime
    {
        public static void Init(ManualLogSource log)
            => SeatSurfaceAnalysisRuntime.Init(log);

        public static void TryLoadDepthAssets(bool forceReload = false)
            => SeatSurfaceAnalysisRuntime.TryLoadDepthAssets(forceReload);

        public static void SetDebugRenderersVisible(bool visible)
            => SeatSurfaceAnalysisRuntime.SetDebugRenderersVisible(visible);

        public static void ClearAll()
            => SeatSurfaceAnalysisRuntime.ClearAll();
    }
}
