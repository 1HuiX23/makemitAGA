/*
 * =================================================================================================
 * FormulaLayoutPolicy.cs
 * =================================================================================================
 *
 * 职责
 * ----
 * 负责公式在 MiSide 3D 对话中的两个独立问题：
 *
 * 1. 公式占几个“游戏字符宽度”（1～4 格）；
 * 2. 公式纹理在该区域内实际显示多大。
 *
 * 为什么 v0.3.10 不再把一个字符永远写死为 72
 * ------------------------------------------------
 * 72 是原生 Text 的 fontSize，也是早期默认字体中常见的全角字 advance。
 * 但游戏真正的艺术字体由 GlobalGame.fontUse 在运行时加载，它的中文字符宽度可能明显
 * 小于 72。若公式仍按固定 72 排版，就会出现：
 *
 * - 普通文字已经变窄；
 * - 公式仍占旧字体的宽度；
 * - 公式前后看起来空隙偏大；
 * - 整句中心位置也会略有偏差。
 *
 * 因此，本版把“一个公式字符格”的宽度作为运行时参数 nativeCellAdvance 传入。
 * GameUIManager 会从游戏艺术字体的 CharacterInfo.advance 中测量该值。
 *
 * 高度调整
 * --------
 * 数学分式、积分号天然比中文方块字高，但截图中旧的 92 UI 单位略显突出。
 * 本版把公式视觉最大高度轻微降低到 84：
 *
 * - 普通 x、x² 不会显得比艺术字高很多；
 * - 分式、积分和根号仍保留足够清晰度；
 * - 不改变根 Symbol 的 110 高度、Collider 或物理效果。
 *
 * 修改注意
 * --------
 * - NativeReferenceCharacterAdvance=72 只是 Skia 公式宽度分类的“历史参考值”；
 * - 真正布局时优先使用 GameUIManager 测得的 nativeCellAdvance；
 * - 不要把 FormulaVisualMaxHeight 改得低于约 78，否则复杂分式会变得太小；
 * - 不要把它改回 92 后又额外缩放 RawImage，否则会产生两套互相叠加的尺寸规则。
 * =================================================================================================
 */

using System;

namespace MakemitAGA.Dialogue
{
    internal static class FormulaLayoutPolicy
    {
        /*
         * 公式渲染与历史测量使用的参考字符宽度。
         * 这不是最终的游戏字符间隔；最终值由 nativeCellAdvance 决定。
         */
        public const float NativeReferenceCharacterAdvance = 72f;

        /*
         * 为旧代码和最后兜底保留的默认值。
         * 新版 GameUIManager 正常情况下会传入实测的艺术字体宽度。
         */
        public const float StandardCharacterAdvance =
            NativeReferenceCharacterAdvance;

        public const float SymbolRectHeight = 110f;

        /*
         * v0.3.10：92 → 84。
         * 只是降低公式可见纹理高度，不移动整行，也不改变物理碰撞盒。
         */
        public const float FormulaVisualMaxHeight = 84f;

        public const float RectHorizontalPadding = 8f;

        /*
         * 防止异常字体返回过小或过大的 advance。
         * 该范围足以覆盖 MiSide 的中文艺术字体，同时避免一个损坏字形破坏整句布局。
         */
        private const float MinNativeCellAdvance = 48f;
        private const float MaxNativeCellAdvance = 78f;

        /// <summary>
        /// 按游戏运行时字体的实际字符宽度，把公式分类为 1～4 格。
        ///
        /// pixelWidth/pixelHeight 是 Skia 裁剪后的真实公式像素尺寸。
        /// 先把它等比换算到最终最大高度，再计算相当于几个游戏艺术字宽度。
        /// </summary>
        public static int ChooseCellSpan(
            int pixelWidth,
            int pixelHeight,
            float nativeCellAdvance)
        {
            if (pixelWidth <= 0 ||
                pixelHeight <= 0)
            {
                return 1;
            }

            float cellAdvance =
                SanitizeNativeCellAdvance(
                    nativeCellAdvance);

            float normalizedWidth =
                pixelWidth *
                (FormulaVisualMaxHeight /
                 pixelHeight);

            float characterWidths =
                normalizedWidth /
                cellAdvance;

            /*
             * 减去少量容差，避免刚好 1.00 格的公式因浮点误差被判成 2 格。
             */
            int span =
                (int)Math.Ceiling(
                    Math.Max(
                        0.01f,
                        characterWidths - 0.05f));

            return Math.Clamp(span, 1, 4);
        }

        /// <summary>
        /// 兼容旧调用。新代码应优先使用带 nativeCellAdvance 的重载。
        /// </summary>
        public static int ChooseCellSpan(
            int pixelWidth,
            int pixelHeight)
        {
            return ChooseCellSpan(
                pixelWidth,
                pixelHeight,
                StandardCharacterAdvance);
        }

        public static float GetAdvance(
            int cellSpan,
            float nativeCellAdvance)
        {
            return
                SanitizeNativeCellAdvance(
                    nativeCellAdvance) *
                Math.Clamp(cellSpan, 1, 4);
        }

        /// <summary>
        /// 兼容旧调用。
        /// </summary>
        public static float GetAdvance(
            int cellSpan)
        {
            return GetAdvance(
                cellSpan,
                StandardCharacterAdvance);
        }

        public static float GetRectWidth(
            int cellSpan,
            float nativeCellAdvance)
        {
            return
                GetAdvance(
                    cellSpan,
                    nativeCellAdvance) +
                RectHorizontalPadding;
        }

        /// <summary>
        /// 兼容旧调用。
        /// </summary>
        public static float GetRectWidth(
            int cellSpan)
        {
            return GetRectWidth(
                cellSpan,
                StandardCharacterAdvance);
        }

        /// <summary>
        /// 计算公式 RawImage 在公式格子内的实际宽高。
        ///
        /// 纹理始终保持长宽比：
        /// - 宽度不能超过当前 1～4 个艺术字格；
        /// - 高度不能超过 FormulaVisualMaxHeight；
        /// - 特别长的公式会在 4 格空间内继续等比缩小。
        /// </summary>
        public static void GetVisualSize(
            int pixelWidth,
            int pixelHeight,
            int cellSpan,
            float nativeCellAdvance,
            out float visualWidth,
            out float visualHeight)
        {
            float maxWidth =
                Math.Max(
                    1f,
                    GetAdvance(
                        cellSpan,
                        nativeCellAdvance) - 8f);

            float maxHeight =
                FormulaVisualMaxHeight;

            if (pixelWidth <= 0 ||
                pixelHeight <= 0)
            {
                visualWidth = maxWidth;
                visualHeight = maxHeight;
                return;
            }

            float scale =
                Math.Min(
                    maxWidth / pixelWidth,
                    maxHeight / pixelHeight);

            visualWidth =
                Math.Max(
                    1f,
                    pixelWidth * scale);

            visualHeight =
                Math.Max(
                    1f,
                    pixelHeight * scale);
        }

        /// <summary>
        /// 兼容旧调用。
        /// </summary>
        public static void GetVisualSize(
            int pixelWidth,
            int pixelHeight,
            int cellSpan,
            out float visualWidth,
            out float visualHeight)
        {
            GetVisualSize(
                pixelWidth,
                pixelHeight,
                cellSpan,
                StandardCharacterAdvance,
                out visualWidth,
                out visualHeight);
        }

        public static float SanitizeNativeCellAdvance(
            float value)
        {
            if (float.IsNaN(value) ||
                float.IsInfinity(value) ||
                value <= 0f)
            {
                return StandardCharacterAdvance;
            }

            return Math.Clamp(
                value,
                MinNativeCellAdvance,
                MaxNativeCellAdvance);
        }
    }
}
