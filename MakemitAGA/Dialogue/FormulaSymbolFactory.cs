/*
 * =================================================================================================
 * FormulaSymbolFactory.cs
 * =================================================================================================
 *
 * 职责
 * ----
 * 把 MiSide 原生的单字符 Symbol 克隆成两种运行时对象：
 *
 * 1. 普通文字 Symbol
 *    继续使用原生 UnityEngine.UI.Text，由 Dialogue_Symbol 控制入场动画、颜色、物理掉落和销毁。
 *
 * 2. 公式 Symbol
 *    保留原生 Text / ShadowText 组件作为 Dialogue_Symbol 的“状态载体”，但把它们隐藏；
 *    真正可见的内容由两层 RawImage 显示：
 *
 *       FormulaMainImage     —— 主公式颜色，通常为紫色；
 *       FormulaShadowImage   —— 原生风格白色底板，比主公式略大，并与主公式中心重合。
 *
 * 为什么不能删除原生 Text
 * ----------------------
 * Dialogue_Symbol.StartComponent、Update、Jump 等原生方法会持续访问根 Text 和 shadowText：
 *
 * - 修改 Alpha；
 * - 修改颜色；
 * - 判断生命周期；
 * - 切换材质；
 * - 执行销毁。
 *
 * 因此公式对象必须“保留但隐藏”这些 Text，而不能删除或替换它们。
 * GameUIManager.Tick() 会每帧调用 FormulaVisualRecord.SyncColors()，
 * 把隐藏 Text 的颜色和 Alpha 同步给 RawImage。
 *
 * v0.3.10：运行时艺术字体宽度驱动公式排版
 * -------------------------------------------
 * 实际测试证明：即使白色层与主公式同心，只要两层尺寸不同，复杂公式仍可能出现
 * “局部看起来偏移”的错觉。积分号、分数线、根号等图形并不是对称矩形，因此简单放大
 * 整张图片并不能保证每一处白边厚度一致。
 *
 * 普通文字和公式使用两条完全不同的字体路径：
 *
 * - 普通文字：UnityEngine.UI.Text，字体来自游戏原生 Dialogue_3DText.font；
 * - 数学公式：CSharpMath + SkiaSharp，使用数学字体渲染为透明纹理。
 *
 * CSharpMath 的字体不会自动影响普通 UI.Text。不过为了防止未来更换模板、场景或
 * Dialogue_3DText 时意外继承到别的字体，本版在创建每个普通字符时显式执行：
 *
 *     text.font = GameUIManager.ResolveNativeFontForText(...)
 *
 * 解析器优先读取 GlobalGame.fontUse，并使用原生 Start Postfix 的观察结果作为第二保险。
 * 只修正普通文字字体来源，不改变公式、排版、白边或物理行为。
 *
 * 重要坐标关系
 * ------------
 * 根 Symbol 的 RectTransform pivot 是 (0, 0.5)，也就是“左中”。
 * FormulaMainImage 的 anchors 是 (0.5, 0.5)，因此它会自动位于公式框中心。
 *
 * 原生 ShadowText 本身也被放到：
 *
 *     x = rectWidth / 2
 *     y = 0
 *     z = 1
 *
 * FormulaShadowImage 是 ShadowText 的子对象，并使用中心锚点。
 * 所以只要它的 anchoredPosition 为 Vector2.zero，就会和主公式精确同心。
 *
 * 修改注意
 * --------
 * - 不要重新加入 (2, -2) 这类偏移，除非明确想要普通投影而不是原生白底板；
 * - 不要把 NativeWhiteOutlineThickness 设得太大，否则分数横线和小字符会显得臃肿；
 * - 不要移动主公式的 RectTransform 来修白底位置，否则会同时破坏布局和 Collider 对齐；
 * - 不要让 ShadowImage 成为主 Image 的子物体，原生 shadowText 的颜色动画需要独立同步；
 * - 所有 Unity 对象创建必须在主线程中执行。
 * =================================================================================================
 */

using System;
using Il2CppInterop.Runtime.InteropTypes.Arrays;
using UnityEngine;
using UnityEngine.UI;
using Object = UnityEngine.Object;

namespace MakemitAGA.Dialogue
{
    /// <summary>
    /// 一张已经转换为 Unity Texture2D 的公式纹理，以及它的排版信息。
    /// </summary>
    internal sealed class FormulaTextureEntry
    {
        public string Latex;
        public Texture2D Texture;
        public int PixelWidth;
        public int PixelHeight;
        public int CellSpan;

        /*
         * 当前游戏艺术字体一个全角字符的实测 advance。
         *
         * 公式生成、整句测量和 Symbol Rect 必须使用同一个值；
         * 否则视觉宽度、Collider 宽度和下一字符起点会互相不一致。
         */
        public float NativeCellAdvance;
    }

    /// <summary>
    /// 保存公式 Symbol 中需要每帧同步的原生 Text 与 RawImage。
    ///
    /// 原生 Dialogue_Symbol 仍然只认识 Text，因此：
    /// - 原生动画修改 Text.color；
    /// - GameUIManager.Tick() 调用 SyncColors()；
    /// - RawImage 获得相同 RGB 和 Alpha。
    /// </summary>
    internal sealed class FormulaVisualRecord
    {
        public GameObject SymbolObject;
        public Text MainText;
        public Text ShadowText;
        public RawImage MainImage;
        public RawImage ShadowImage;

        public void SyncColors()
        {
            if (SymbolObject == null)
                return;

            if (MainText != null && MainImage != null)
                MainImage.color = MainText.color;

            if (ShadowText != null && ShadowImage != null)
                ShadowImage.color = ShadowText.color;
        }
    }

    internal static class FormulaSymbolFactory
    {
        // 原生单字符 Symbol 的 UI Rect 大小。
        public const float NativeRectWidth = 80f;
        public const float NativeRectHeight = 110f;

        /*
         * 公式和普通文字共用这一白边粗度。
         *
         * 粗边由 UnityEngine.UI.Outline 向四周扩展，而不是移动或放大另一份字形。
         * 2 UI 单位在当前原生字号下能够形成清楚但不过厚的白边。
         *
         * 如果未来觉得白边太细，可尝试 2.5f；
         * 如果觉得分数线或小字符被白色吞没，可降到 1.5f。
         */
        private const float NativeWhiteOutlineThickness = 2.0f;

        private static readonly Vector2 FormulaBackingOffset =
            Vector2.zero;

        /// <summary>
        /// 取得游戏运行时真正使用的普通文字字体。
        ///
        /// 不能只返回 dialogue.font：Formula3D 私有模板不会运行原生 Start，
        /// 因而该字段可能仍是预制件默认字体。
        ///
        /// GameUIManager 会优先使用 GlobalGame.fontUse，并以原生 Start Postfix
        /// 捕获到的最终字体作为第二保险。公式本身仍使用 CSharpMath 数学字体。
        /// </summary>
        private static Font ResolveNativeGameFont(
            Dialogue_3DText dialogue,
            Text clonedText)
        {
            return GameUIManager.ResolveNativeFontForText(
                dialogue,
                clonedText);
        }

        /// <summary>
        /// 克隆并创建一个普通字符。
        ///
        /// 普通字符不使用 CSharpMath，也不使用下载包中的数学字体。
        /// 它继续使用游戏原生 UnityEngine.UI.Text，并在创建时显式锁定
        /// 当前 Dialogue_3DText.font。
        /// </summary>
        public static GameObject CreateTextSymbol(
            Dialogue_3DText dialogue,
            GameObject template,
            char character,
            float anchoredX,
            float symbolScale)
        {
            GameObject symbol = CloneTemplate(
                template,
                "AI3D_Text_" + ((int)character).ToString("X4"));

            if (symbol == null)
                return null;

            try
            {
                ConfigureRootRect(
                    symbol,
                    NativeRectWidth,
                    anchoredX,
                    symbolScale);

                Text text = symbol.GetComponent<Text>();
                if (text == null)
                {
                    throw new InvalidOperationException(
                        "Native Symbol template has no UI.Text.");
                }

                /*
                 * 显式锁定游戏运行时艺术字体。
                 *
                 * 这里不能只信任 dialogue.font，因为手动私有模板不会执行原生 Start。
                 * ResolveNativeGameFont 会优先返回 GlobalGame.fontUse。
                 */
                Font nativeFont =
                    ResolveNativeGameFont(
                        dialogue,
                        text);

                if (nativeFont != null)
                    text.font = nativeFont;

                text.text = character.ToString();
                text.alignment = TextAnchor.MiddleLeft;

                Dialogue_Symbol dialogueSymbol =
                    symbol.GetComponent<Dialogue_Symbol>();

                if (dialogueSymbol == null)
                {
                    throw new InvalidOperationException(
                        "Native Symbol template has no Dialogue_Symbol.");
                }

                /*
                 * shadowText 虽然在 v0.3.8 后不再负责可见白边，但原生
                 * Dialogue_Symbol 仍可能访问它，所以继续让它使用同一个游戏字体。
                 */
                if (nativeFont != null &&
                    dialogueSymbol.shadowText != null)
                {
                    dialogueSymbol.shadowText.font =
                        nativeFont;
                }

                /*
                 * StartComponent 读取当前 RectTransform、Text、shadowText，
                 * 初始化原生入场动画和物理状态。
                 * 必须在 Symbol 具有完整原生组件的情况下调用。
                 */
                symbol.SetActive(true);

                dialogueSymbol.StartComponent(
                    dialogue,
                    -1f,
                    dialogue.noiseStart,
                    dialogue.noise,
                    GetStartRotation(dialogue));

                /*
                 * v0.3.8：普通文字也使用和公式相同的“同尺寸白色轮廓”。
                 *
                 * 原生 shadowText 是另一份独立 Text；在当前克隆/手动排版路径中，
                 * 它的局部位置和字形边界可能与主 Text 出现少量偏移。
                 *
                 * 因此：
                 * 1. 保留 shadowText 组件，供 Dialogue_Symbol 继续读写状态；
                 * 2. 只关闭它的可视渲染；
                 * 3. 在主 Text 自身添加 Outline，直接从同一个字形轮廓向外加粗。
                 *
                 * useGraphicAlpha=true 会让白边自动跟随主文字的原生淡入/淡出，
                 * 不需要额外的每帧同步代码。
                 */
                ConfigureNativeWhiteOutline(text);

                if (dialogueSymbol.shadowText != null)
                    dialogueSymbol.shadowText.enabled = false;

                return symbol;
            }
            catch
            {
                Object.Destroy(symbol);
                throw;
            }
        }

        /// <summary>
        /// 创建一块公式 Symbol。
        ///
        /// 公式仍然是“一块”物理对象：
        /// - 根对象包含 Rigidbody、BoxCollider 和 Dialogue_Symbol；
        /// - 主公式与白色底板都是它的 UI 子对象；
        /// - 掉落时两层会作为同一个刚体一起运动。
        /// </summary>
        public static GameObject CreateFormulaSymbol(
            Dialogue_3DText dialogue,
            GameObject template,
            FormulaTextureEntry formula,
            float anchoredX,
            float symbolScale,
            out FormulaVisualRecord visualRecord)
        {
            visualRecord = null;

            GameObject symbol = CloneTemplate(
                template,
                "AI3D_Formula_" + formula.CellSpan + "Cells");

            if (symbol == null)
                return null;

            try
            {
                float rectWidth =
                    FormulaLayoutPolicy.GetRectWidth(
                        formula.CellSpan,
                        formula.NativeCellAdvance);

                ConfigureRootRect(
                    symbol,
                    rectWidth,
                    anchoredX,
                    symbolScale);

                Text mainText = symbol.GetComponent<Text>();
                Dialogue_Symbol dialogueSymbol =
                    symbol.GetComponent<Dialogue_Symbol>();

                if (mainText == null ||
                    dialogueSymbol == null)
                {
                    throw new InvalidOperationException(
                        "Formula Symbol template is missing Text or Dialogue_Symbol.");
                }

                /*
                 * “□”只是给原生组件保留一个合法字符。
                 * 它稍后会被隐藏，玩家不会看到。
                 */
                mainText.text = "□";
                mainText.alignment = TextAnchor.MiddleLeft;

                Text shadowText = dialogueSymbol.shadowText;
                RectTransform shadowRect =
                    shadowText != null
                        ? shadowText.rectTransform
                        : null;

                if (shadowRect != null)
                {
                    /*
                     * 根 Symbol pivot 为左中，所以公式框的几何中心在 rectWidth / 2。
                     *
                     * shadowText 是原生对象，保留它的 z=1 层级关系，
                     * 只把尺寸和水平中心调整到当前公式宽度。
                     */
                    shadowRect.sizeDelta = new Vector2(
                        rectWidth,
                        FormulaLayoutPolicy.SymbolRectHeight);

                    shadowRect.pivot =
                        new Vector2(0.5f, 0.5f);

                    shadowRect.anchorMin =
                        new Vector2(0.5f, 0.5f);

                    shadowRect.anchorMax =
                        new Vector2(0.5f, 0.5f);

                    shadowRect.anchoredPosition =
                        Vector2.zero;

                    shadowRect.localPosition =
                        new Vector3(
                            rectWidth * 0.5f,
                            0f,
                            1f);
                }

                /*
                 * visualWidth / visualHeight 是主公式真实显示尺寸。
                 * 1~4 字宽只决定它有多少排版空间；超长公式会在空间内等比缩小。
                 */
                FormulaLayoutPolicy.GetVisualSize(
                    formula.PixelWidth,
                    formula.PixelHeight,
                    formula.CellSpan,
                    formula.NativeCellAdvance,
                    out float visualWidth,
                    out float visualHeight);

                RawImage shadowImage = null;

                if (shadowText != null)
                {
                    /*
                     * v0.3.8 原生风格修复：
                     *
                     * 1. 白色层与主公式使用完全相同的宽、高和中心；
                     * 2. 给白色 RawImage 添加 Unity UI Outline；
                     * 3. Outline 从同一 Alpha 轮廓向四周扩展，因此不会因整体缩放而错位。
                     *
                     * RawImage 本体被主公式完全覆盖，玩家主要看到的是 Outline 露出的白边。
                     * useGraphicAlpha=true 会让白边跟随 RawImage 的原生 Alpha 入场动画。
                     */
                    shadowImage = CreateRawImage(
                        "FormulaShadowImage",
                        shadowText.transform,
                        formula.Texture,
                        visualWidth,
                        visualHeight,
                        FormulaBackingOffset,
                        shadowText.color);

                    /*
                     * 与普通文字共用相同粗度和配置，避免两套画风逐渐分叉。
                     */
                    ConfigureNativeWhiteOutline(
                        shadowImage);
                }

                /*
                 * 主公式保持原来的精确尺寸和中心位置。
                 */
                RawImage mainImage = CreateRawImage(
                    "FormulaMainImage",
                    symbol.transform,
                    formula.Texture,
                    visualWidth,
                    visualHeight,
                    Vector2.zero,
                    mainText.color);

                /*
                 * 公式的碰撞盒按它占据的字符宽度扩大。
                 * 本次白底视觉调整不改变 Collider，避免影响物理掉落行为。
                 */
                BoxCollider box =
                    symbol.GetComponent<BoxCollider>();

                if (box != null)
                {
                    box.center = new Vector3(
                        rectWidth * 0.5f,
                        0f,
                        0f);

                    box.size = new Vector3(
                        Math.Max(
                            10f,
                            rectWidth - 30f),
                        80f,
                        10f);
                }

                /*
                 * StartComponent 必须在隐藏 Text 之前调用。
                 * 原生方法会读取 Text / shadowText 的颜色、材质和 RectTransform。
                 */
                symbol.SetActive(true);

                dialogueSymbol.StartComponent(
                    dialogue,
                    -1f,
                    dialogue.noiseStart,
                    dialogue.noise,
                    GetStartRotation(dialogue));

                /*
                 * 调用完成后才隐藏原生 Text。
                 * 组件本身仍然存在，供 Dialogue_Symbol.Update 持续修改状态。
                 */
                mainText.enabled = false;

                if (shadowText != null)
                    shadowText.enabled = false;

                visualRecord =
                    new FormulaVisualRecord
                    {
                        SymbolObject = symbol,
                        MainText = mainText,
                        ShadowText = shadowText,
                        MainImage = mainImage,
                        ShadowImage = shadowImage
                    };

                /*
                 * 立即同步一次，避免等待下一帧前短暂出现错误颜色。
                 * 后续每帧由 GameUIManager.Tick() 继续同步。
                 */
                visualRecord.SyncColors();

                return symbol;
            }
            catch
            {
                Object.Destroy(symbol);
                throw;
            }
        }

        /// <summary>
        /// 把 Skia 生成的 PNG 字节转换成 Unity Texture2D。
        ///
        /// IL2CPP 注意：
        /// ImageConversion.LoadImage 需要 Il2CppStructArray&lt;byte&gt;，
        /// 不能直接把托管 byte[] 传进去。
        /// </summary>
        public static bool TryCreateUnityTexture(
            string name,
            byte[] pngBytes,
            out Texture2D texture,
            out string error)
        {
            texture = null;
            error = null;

            if (pngBytes == null ||
                pngBytes.Length == 0)
            {
                error =
                    "PNG byte array is empty.";
                return false;
            }

            try
            {
                texture = new Texture2D(
                    2,
                    2,
                    TextureFormat.RGBA32,
                    false);

                texture.name = name;
                texture.wrapMode =
                    TextureWrapMode.Clamp;

                texture.filterMode =
                    FilterMode.Bilinear;

                var il2CppBytes =
                    new Il2CppStructArray<byte>(
                        pngBytes.Length);

                for (int i = 0;
                     i < pngBytes.Length;
                     i++)
                {
                    il2CppBytes[i] =
                        pngBytes[i];
                }

                bool loaded =
                    ImageConversion.LoadImage(
                        texture,
                        il2CppBytes,
                        false);

                if (!loaded)
                {
                    Object.Destroy(texture);
                    texture = null;

                    error =
                        "Unity ImageConversion.LoadImage returned false.";

                    return false;
                }

                return true;
            }
            catch (Exception e)
            {
                if (texture != null)
                {
                    Object.Destroy(texture);
                    texture = null;
                }

                error =
                    e.GetType().Name +
                    ": " +
                    e.Message;

                return false;
            }
        }

        /// <summary>
        /// 克隆原生 Symbol 模板。
        /// 克隆后先保持 inactive，避免组件在配置完成前执行原生生命周期。
        /// </summary>
        private static GameObject CloneTemplate(
            GameObject template,
            string name)
        {
            if (template == null)
                return null;

            Transform parent =
                template.transform.parent;

            GameObject symbol =
                Object.Instantiate(
                    template,
                    parent);

            symbol.SetActive(false);
            symbol.name = name;

            return symbol;
        }

        /// <summary>
        /// 设置根 Symbol 的排版矩形和世界缩放。
        ///
        /// anchoredX 使用 Dialogue_3DText 的本地 UI 坐标；
        /// symbolScale 通常为原生的 0.0015。
        /// </summary>
        private static void ConfigureRootRect(
            GameObject symbol,
            float width,
            float anchoredX,
            float symbolScale)
        {
            RectTransform rect =
                symbol.GetComponent<RectTransform>();

            if (rect == null)
            {
                throw new InvalidOperationException(
                    "Symbol template has no RectTransform.");
            }

            rect.anchorMin =
                new Vector2(0.5f, 0.5f);

            rect.anchorMax =
                new Vector2(0.5f, 0.5f);

            rect.pivot =
                new Vector2(0f, 0.5f);

            rect.sizeDelta =
                new Vector2(
                    width,
                    FormulaLayoutPolicy.SymbolRectHeight);

            rect.anchoredPosition =
                new Vector2(
                    anchoredX,
                    0f);

            rect.localScale =
                new Vector3(
                    symbolScale,
                    symbolScale,
                    symbolScale);
        }

        /// <summary>
        /// 创建一层透明 RawImage。
        ///
        /// parent 决定它属于主公式层还是原生 shadowText 层。
        /// 所有图层都使用中心锚点，因此 offset=0 时精确同心。
        /// </summary>
        private static RawImage CreateRawImage(
            string name,
            Transform parent,
            Texture2D texture,
            float width,
            float height,
            Vector2 offset,
            Color color)
        {
            var imageObject =
                new GameObject(name);

            imageObject.transform.SetParent(
                parent,
                false);

            RectTransform rect =
                imageObject
                    .AddComponent<RectTransform>();

            RawImage image =
                imageObject
                    .AddComponent<RawImage>();

            rect.anchorMin =
                new Vector2(0.5f, 0.5f);

            rect.anchorMax =
                new Vector2(0.5f, 0.5f);

            rect.pivot =
                new Vector2(0.5f, 0.5f);

            rect.sizeDelta =
                new Vector2(width, height);

            rect.anchoredPosition =
                offset;

            rect.localScale =
                Vector3.one;

            rect.localRotation =
                Quaternion.identity;

            image.texture = texture;
            image.color = color;
            image.raycastTarget = false;

            return image;
        }

        /// <summary>
        /// 为任意 Unity UI Graphic 添加与主字形同尺寸的白色粗轮廓。
        ///
        /// Graphic 可以是：
        /// - 普通文字的 UI.Text；
        /// - 公式白色层的 RawImage。
        ///
        /// Outline 是 BaseMeshEffect，它复制同一份字形/图片顶点并向四周偏移，
        /// 因而不会出现“另一张图片放大后局部对不齐”的问题。
        /// </summary>
        private static void ConfigureNativeWhiteOutline(
            Graphic graphic)
        {
            if (graphic == null)
                return;

            Outline outline =
                graphic.GetComponent<Outline>();

            if (outline == null)
            {
                outline =
                    graphic.gameObject
                        .AddComponent<Outline>();
            }

            outline.effectColor =
                new Color(1f, 1f, 1f, 1f);

            outline.effectDistance =
                new Vector2(
                    NativeWhiteOutlineThickness,
                    NativeWhiteOutlineThickness);

            /*
             * 让 Outline 的 Alpha 乘上 Graphic 自身的 Alpha。
             * 普通 Text 的淡入淡出会自动带动白边；
             * 公式 RawImage 的 Alpha 则由 GameUIManager.Tick() 同步。
             */
            outline.useGraphicAlpha = true;
        }

        /// <summary>
        /// 与原生 Dialogue_Symbol.StartComponent 的主题分支保持一致。
        /// </summary>
        private static float GetStartRotation(
            Dialogue_3DText dialogue)
        {
            return dialogue != null &&
                   (int)dialogue.themeDialogue == 6
                ? 240f
                : 20f;
        }
    }
}
