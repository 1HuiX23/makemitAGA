/*
 * [文件说明]: 视觉系统的 Harmony 钩子
 * 
 * [分析过程]:
 * 1. 游戏场景切换或加载时会销毁非原生对象。
 * 2. 为了保证摄像机始终跟随，我们选择"寄生"在 MitaPerson 的 Update 循环中。
 * 
 * [主要功能]:
 * 1. HookMitaUpdate (LateUpdate): 每一帧强制同步摄像机位置，确保画面不抖动且不丢失。
 */
using HarmonyLib;

namespace MakemitAGA.Mita_self
{
    public class VisionPatches
    {
        [HarmonyPatch(typeof(MitaPerson), "LateUpdate")]
        [HarmonyPostfix]
        public static void HookMitaUpdate(MitaPerson __instance)
        {
            MitaVisionManager.UpdateMita(__instance);

            // Seat VLM 使用独立冻结相机，避免干扰原有 look 指令的相机生命周期。
            Mita_tools.SeatVlmVisionManager.UpdateMita(__instance);
            Mita_tools.SeatSurfaceVlmPreviewManager.UpdateMita(__instance);
        }
    }
}