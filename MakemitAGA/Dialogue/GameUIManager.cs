/*
 * =================================================================================================
 * GameUIManager.cs
 * =================================================================================================
 *
 * 作用：
 *   MakemitAGA 的 AI 3D 对话总入口。普通字符继续克隆 MiSide 原生 Symbol，LaTeX 公式则由
 *   CSharpMath + SkiaSharp 渲染为透明纹理，再放入同一套 Dialogue_Symbol 物理生命周期中。
 *
 * 输入：
 *   ShowLongText(string) 接收后端返回的混合文本，支持 $...$、$$...$$、\(...\)、\[...\]。
 *
 * 输出：
 *   逐字/逐公式打印、跟随 DialogueQuest Mita、延迟后统一执行 Dialogue_Symbol.Jump()，
 *   最后销毁本句对象和运行时 Texture2D。
 *
 * 关键约束：
 *   - 所有 Unity API 只能在主线程调用；
 *   - Skia 只负责在内存中生成 PNG，Texture2D 仍在 Unity 主线程创建；
 *   - 私有 Dialogue_3DText 模板每个场景重新捕获，不能跨场景复用；
 *   - 普通文字必须使用运行时 GlobalGame.fontUse，不能只继承未初始化预制件字体；
 *   - 不依赖原来的 PendingInjections 注入路径，但保留该字典供旧 Patch 编译兼容；
 *   - 自动掉落间隔由 Formula3DTextApi 全局控制；
 *   - 公式宽度使用游戏运行时艺术字体的实测 advance，不能再假设固定 72。
 * =================================================================================================
 */
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using BepInEx.Logging;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace MakemitAGA.Dialogue
{
    internal sealed class PreparedDialogueToken
    {
        public FormulaToken Source;
        public FormulaTextureEntry Formula;
    }

    public static class GameUIManager
    {
        private const float CharacterInterval = 0.05f;
        private const float DefaultSymbolScale = 0.0015f;
        private const float DropCleanupDelay = 1.5f;
        private const float BetweenSentenceDelay = 0.2f;

        /*
         * 用于测量当前游戏艺术字体的代表性全角字符。
         *
         * 不只测一个字符，是为了避免某个艺术字存在特殊 side bearing。
         * 取中位数比平均值更不容易被异常字形影响。
         */
        private const string NativeAdvanceProbeCharacters =
            "字公式中文测试";

        private static ManualLogSource _log;

        private static GameObject _privateTemplateRoot;
        private static Dialogue_3DText _templateDialogue;
        private static Transform _dialogueParent;
        private static int _templateSceneHandle = int.MinValue;

        /*
         * =========================================================================================
         * 游戏运行时字体缓存
         * =========================================================================================
         *
         * MiSide 的可爱艺术字体并不是可靠地保存在未激活预制件里。
         * 原生 Dialogue_3DText.Start() 会在运行时读取 GlobalGame.fontUse，然后把它写入：
         *
         *   1. Dialogue_3DText.font
         *   2. exampleSymbol 根 Text.font
         *   3. Dialogue_Symbol.shadowText.font
         *
         * Formula3D 为了手动混排普通字符和公式，会关闭 Dialogue_3DText.enabled，
         * 所以私有模板不会执行原生 Start()。若只继承模板字段，就可能拿到预制默认字体，
         * 看起来像系统黑体/微软雅黑，而不是游戏中的艺术字。
         *
         * 本缓存有两条权威来源：
         *   - GlobalGame.fontUse：游戏全局真正使用的字体；
         *   - 原生 Dialogue_3DText.Start Postfix：已经完成初始化的实际对话字体。
         */
        private static Font _runtimeNativeFont;
        private static int _runtimeNativeFontSceneHandle = int.MinValue;
        private static string _runtimeNativeFontSource = "<none>";

        private static Coroutine _currentSequenceRoutine;
        private static int _sequenceSerial;

        private static GameObject _activeRoot;
        private static Dialogue_3DText _activeDialogue;
        private static GameObject _activeSymbolTemplate;
        private static bool _activeDropped;

        private static readonly List<GameObject> ActiveSymbols = new();
        private static readonly List<FormulaVisualRecord> ActiveFormulaVisuals = new();
        private static readonly List<Texture2D> ActiveTextures = new();

        // 仅为旧版 DialoguePatches.InjectText 保留。新公式路径不会向这里写入内容。
        [Obsolete("Formula3D integrated renderer no longer uses PendingInjections.")]
        public static readonly Dictionary<Dialogue_3DText, string> PendingInjections = new();

        public static void Initialize(ManualLogSource log)
        {
            _log = log;
            _log?.LogInfo("[Formula3D] GameUIManager initialized.");

            // 这里失败并不算错误：主菜单早期 GlobalGame 可能尚未完成初始化。
            // 真正显示前和原生 Start Postfix 中还会继续尝试。
            TryRefreshRuntimeNativeFontFromGlobal(
                "GameUIManager.Initialize");
        }

        /// <summary>
        /// 由 Dialogue_3DText.Start 的 Harmony Postfix 调用。
        /// 此时游戏已经执行完字体选择，因此 __instance.font 是可靠的最终结果。
        /// </summary>
        public static void NotifyNativeDialogueStarted(
            Dialogue_3DText dialogue)
        {
            if (dialogue == null ||
                IsPluginOwnedDialogue(dialogue))
            {
                return;
            }

            Font observed = null;

            try
            {
                observed = dialogue.font;
            }
            catch { }

            // 极少数情况下根字段为空，但 exampleSymbol 已经获得正确字体。
            if (observed == null)
            {
                try
                {
                    if (dialogue.exampleSymbol != null)
                    {
                        Text text =
                            dialogue.exampleSymbol
                                .GetComponent<Text>();

                        if (text != null)
                            observed = text.font;
                    }
                }
                catch { }
            }

            CacheRuntimeNativeFont(
                observed,
                "native-start:" +
                SafeDialogueName(dialogue));
        }

        /// <summary>
        /// 普通文字统一使用的字体解析入口。
        ///
        /// 优先级：
        /// 1. GlobalGame.fontUse；
        /// 2. 原生 Start Postfix 观察到的最终字体；
        /// 3. 当前 dialogue.font；
        /// 4. 克隆 Text 自带字体。
        ///
        /// 公式不走这里；公式继续由 CSharpMath 数学字体渲染。
        /// </summary>
        internal static Font ResolveNativeFontForText(
            Dialogue_3DText dialogue,
            Text clonedText)
        {
            Font globalFont = TryGetGlobalGameFont();

            if (globalFont != null)
            {
                CacheRuntimeNativeFont(
                    globalFont,
                    "GlobalGame.fontUse");

                return globalFont;
            }

            if (IsRuntimeNativeFontUsable())
                return _runtimeNativeFont;

            try
            {
                if (dialogue != null &&
                    dialogue.font != null)
                {
                    return dialogue.font;
                }
            }
            catch { }

            try
            {
                return clonedText != null
                    ? clonedText.font
                    : null;
            }
            catch
            {
                return null;
            }
        }

        /// <summary>
        /// 把游戏运行时艺术字体写入对话根和 exampleSymbol。
        ///
        /// 只设置 dialogue.font 不够，因为我们真正克隆的是 exampleSymbol；
        /// 根 Text 和 shadowText 也必须同步，否则新字符仍可能继承错误字体。
        /// </summary>
        private static void ApplyRuntimeNativeFontToDialogue(
            Dialogue_3DText dialogue,
            string reason)
        {
            if (dialogue == null)
                return;

            Text rootText = null;

            try
            {
                if (dialogue.exampleSymbol != null)
                {
                    rootText =
                        dialogue.exampleSymbol
                            .GetComponent<Text>();
                }
            }
            catch { }

            Font nativeFont =
                ResolveNativeFontForText(
                    dialogue,
                    rootText);

            if (nativeFont == null)
            {
                _log?.LogWarning(
                    "[Formula3D][Font] No runtime native font available" +
                    " | reason=" + reason);
                return;
            }

            try { dialogue.font = nativeFont; }
            catch { }

            try
            {
                if (rootText != null)
                    rootText.font = nativeFont;
            }
            catch { }

            try
            {
                if (dialogue.exampleSymbol != null)
                {
                    Dialogue_Symbol symbol =
                        dialogue.exampleSymbol
                            .GetComponent<Dialogue_Symbol>();

                    if (symbol != null &&
                        symbol.shadowText != null)
                    {
                        symbol.shadowText.font =
                            nativeFont;
                    }
                }
            }
            catch { }

            _log?.LogDebug(
                "[Formula3D][Font] Applied runtime native font" +
                " | font=" + SafeFontName(nativeFont) +
                " | reason=" + reason);
        }

        /// <summary>
        /// 读取原生 Dialogue_3DText.Start 使用的权威静态字体。
        /// </summary>
        private static Font TryGetGlobalGameFont()
        {
            try
            {
                Font font = GlobalGame.fontUse;
                return font != null ? font : null;
            }
            catch
            {
                // 过早调用时 GlobalGame 可能还没有初始化；稍后再试即可。
                return null;
            }
        }

        private static bool TryRefreshRuntimeNativeFontFromGlobal(
            string source)
        {
            Font font = TryGetGlobalGameFont();

            if (font == null)
                return false;

            CacheRuntimeNativeFont(
                font,
                source + " -> GlobalGame.fontUse");

            return true;
        }

        private static void CacheRuntimeNativeFont(
            Font font,
            string source)
        {
            if (font == null)
                return;

            int sceneHandle =
                SceneManager.GetActiveScene().handle;

            bool changed =
                _runtimeNativeFont == null ||
                _runtimeNativeFont != font ||
                _runtimeNativeFontSceneHandle != sceneHandle;

            _runtimeNativeFont = font;
            _runtimeNativeFontSceneHandle = sceneHandle;
            _runtimeNativeFontSource =
                string.IsNullOrWhiteSpace(source)
                    ? "<unknown>"
                    : source;

            if (changed)
            {
                _log?.LogInfo(
                    "[Formula3D][Font] Runtime native font captured" +
                    " | font=" + SafeFontName(font) +
                    " | source=" + _runtimeNativeFontSource +
                    " | scene=" + sceneHandle);
            }
        }

        private static bool IsRuntimeNativeFontUsable()
        {
            try
            {
                return
                    _runtimeNativeFont != null &&
                    _runtimeNativeFontSceneHandle ==
                        SceneManager.GetActiveScene().handle;
            }
            catch
            {
                return false;
            }
        }

        private static void ClearRuntimeNativeFont(
            string reason)
        {
            _runtimeNativeFont = null;
            _runtimeNativeFontSceneHandle = int.MinValue;
            _runtimeNativeFontSource = "<none>";

            _log?.LogDebug(
                "[Formula3D][Font] Runtime font cache cleared" +
                " | reason=" + reason);
        }

        private static string SafeFontName(Font font)
        {
            if (font == null)
                return "<null>";

            try
            {
                return string.IsNullOrWhiteSpace(font.name)
                    ? "<unnamed-font>"
                    : font.name;
            }
            catch
            {
                return "<font-name-unavailable>";
            }
        }

        private static string SafeDialogueName(
            Dialogue_3DText dialogue)
        {
            if (dialogue == null)
                return "<null>";

            try
            {
                return dialogue.gameObject != null
                    ? dialogue.gameObject.name
                    : "<no-gameobject>";
            }
            catch
            {
                return "<dialogue-name-unavailable>";
            }
        }

        public static void Tick()
        {
            for (int i = ActiveFormulaVisuals.Count - 1; i >= 0; i--)
            {
                FormulaVisualRecord record = ActiveFormulaVisuals[i];
                if (record == null || record.SymbolObject == null)
                {
                    ActiveFormulaVisuals.RemoveAt(i);
                    continue;
                }

                try { record.SyncColors(); }
                catch { ActiveFormulaVisuals.RemoveAt(i); }
            }
        }

        public static void FindAndCacheAssets()
        {
            int sceneHandle = SceneManager.GetActiveScene().handle;

            if (IsUsableTemplate(_templateDialogue) &&
                _privateTemplateRoot != null &&
                _templateSceneHandle == sceneHandle &&
                _dialogueParent != null)
            {
                return;
            }

            ClearTemplateOnly("recache");

            Transform parent = FindDialogueQuestMitaTransform();
            if (parent == null)
            {
                _log?.LogWarning("[Formula3D] DialogueQuest Mita not found.");
                return;
            }

            Dialogue_3DText source = FindBestTemplateUnder(parent);
            if (!IsUsableTemplate(source))
            {
                _log?.LogWarning("[Formula3D] No usable Dialogue_3DText template under DialogueQuest Mita.");
                return;
            }

            GameObject clone = null;
            try
            {
                Vector3 localPosition = source.transform.localPosition;
                Quaternion localRotation = source.transform.localRotation;
                Vector3 localScale = source.transform.localScale;

                clone = Object.Instantiate(source.gameObject, parent);
                clone.name = "AI_Formula3D_PrivateTemplate";
                clone.SetActive(false);
                clone.transform.localPosition = localPosition;
                clone.transform.localRotation = localRotation;
                clone.transform.localScale = localScale;

                Dialogue_3DText dialogue = clone.GetComponent<Dialogue_3DText>();
                if (!IsUsableTemplate(dialogue))
                    throw new InvalidOperationException("Cloned private template is incomplete.");

                ConfigureDialogueAsManualRoot(dialogue);

                // 私有模板不会执行原生 Start，因此必须在这里补齐运行时艺术字体。
                ApplyRuntimeNativeFontToDialogue(
                    dialogue,
                    "private-template-capture");

                dialogue.exampleSymbol.SetActive(false);

                _privateTemplateRoot = clone;
                _templateDialogue = dialogue;
                _dialogueParent = parent;
                _templateSceneHandle = sceneHandle;

                _log?.LogInfo(
                    "[Formula3D] Private template ready" +
                    " | parent=" + GetTransformPath(parent) +
                    " | sizeSymbol=" + dialogue.sizeSymbol.ToString("F6") +
                    " | font=" + SafeFontName(dialogue.font));
            }
            catch (Exception e)
            {
                _log?.LogError("[Formula3D] Template capture failed: " + e);
                if (clone != null) Object.Destroy(clone);
                ClearTemplateOnly("capture-failed");
            }
        }

        public static void ShowLongText(string fullText)
        {
            if (Plugin.Runner == null)
            {
                _log?.LogWarning("[Formula3D] MainThreadRunner is not ready.");
                return;
            }

            FindAndCacheAssets();
            if (!IsUsableTemplate(_templateDialogue))
            {
                PrintGame("无法显示 AI 3D 文字：当前场景没有可用的 DialogueQuest Mita 模板。");
                return;
            }

            Queue<List<FormulaToken>> queue = BuildSentenceQueue(fullText);
            if (queue.Count == 0)
                return;

            CancelSequenceAndDestroyActive("new-show");
            int serial = ++_sequenceSerial;

            _currentSequenceRoutine = Plugin.Runner.StartCoroutine(
                PlaySequenceRoutine(queue, serial).WrapToIl2Cpp());
        }

        public static void DropCurrent()
        {
            DropActiveInternal("manual-command");
        }

        public static void ClearCurrent(string reason)
        {
            CancelSequenceAndDestroyActive(reason ?? "manual-clear");
        }

        public static void OnSceneChanged(string sceneName, int sceneHandle)
        {
            CancelSequenceAndDestroyActive("scene-change:" + sceneName);
            ClearTemplateOnly("scene-change:" + sceneName);
            ClearRuntimeNativeFont("scene-change:" + sceneName);
            _log?.LogInfo("[Formula3D] Scene reset | name=" + sceneName + " | handle=" + sceneHandle);
        }

        public static void Shutdown(string reason)
        {
            CancelSequenceAndDestroyActive("shutdown:" + reason);
            ClearTemplateOnly("shutdown:" + reason);
            ClearRuntimeNativeFont("shutdown:" + reason);
        }

        private static IEnumerator PlaySequenceRoutine(
            Queue<List<FormulaToken>> queue,
            int serial)
        {
            while (queue.Count > 0 && serial == _sequenceSerial)
            {
                // 上一句如果因异常/取消留下对象，必须在准备新纹理之前清理。
                // 注意：不能在 CreateActiveDialogueRoot() 里清理，否则刚生成的公式纹理也会被销毁。
                DestroyActiveLine("before-prepare-sentence");

                List<FormulaToken> line = queue.Dequeue();
                List<PreparedDialogueToken> prepared = new();

                bool needsSkia = false;
                for (int i = 0; i < line.Count; i++)
                {
                    if (line[i].Kind == FormulaTokenKind.Formula)
                    {
                        needsSkia = true;
                        break;
                    }
                }

                bool skiaReady = !needsSkia || NativeSkiaBootstrap.Initialize(_log);

                for (int i = 0; i < line.Count; i++)
                {
                    FormulaToken token = line[i];
                    PreparedDialogueToken item = new() { Source = token };

                    if (token.Kind == FormulaTokenKind.Formula)
                    {
                        FormulaTextureEntry formula = null;
                        string error = null;

                        bool formulaPrepared =
                            skiaReady &&
                            TryPrepareFormula(
                                token.Content,
                                out formula,
                                out error);

                        if (!formulaPrepared)
                        {
                            _log?.LogWarning(
                                "[Formula3D] Formula fallback" +
                                " | latex=" + token.Content +
                                " | error=" +
                                (error ??
                                 (skiaReady
                                     ? "Formula render failed"
                                     : "Skia unavailable")));

                            item.Source = FormulaToken.Text("[公式:" + token.Content + "]");
                        }
                        else
                        {
                            item.Formula = formula;
                        }
                    }

                    prepared.Add(item);
                    yield return null;
                }

                if (serial != _sequenceSerial)
                    yield break;

                if (!CreateActiveDialogueRoot())
                {
                    DestroyActiveLine("root-unavailable");
                    yield break;
                }

                float totalAdvance = MeasureTotalAdvance(prepared, _activeDialogue.font);
                float symbolScale = GetNativeSymbolScale(_activeDialogue);
                _activeDialogue.sizeSymbol = symbolScale;

                float x = -totalAdvance * symbolScale * 0.5f;
                _activeDialogue.xPrint = x;

                for (int i = 0; i < prepared.Count; i++)
                {
                    if (serial != _sequenceSerial)
                        yield break;

                    PreparedDialogueToken item = prepared[i];

                    if (item.Source.Kind == FormulaTokenKind.Formula && item.Formula != null)
                    {
                        try
                        {
                            GameObject symbol = FormulaSymbolFactory.CreateFormulaSymbol(
                                _activeDialogue,
                                _activeSymbolTemplate,
                                item.Formula,
                                x,
                                symbolScale,
                                out FormulaVisualRecord visual);

                            RegisterSymbol(symbol);
                            if (visual != null) ActiveFormulaVisuals.Add(visual);
                        }
                        catch (Exception e)
                        {
                            _log?.LogError("[Formula3D] Formula symbol creation failed: " + e);
                        }

                        x +=
                            FormulaLayoutPolicy.GetAdvance(
                                item.Formula.CellSpan,
                                item.Formula.NativeCellAdvance) *
                            symbolScale;
                        _activeDialogue.xPrint = x;
                        yield return new WaitForSeconds(CharacterInterval);
                        continue;
                    }

                    string text = item.Source.Content ?? string.Empty;
                    if (_activeDialogue.font != null && text.Length > 0)
                        _activeDialogue.font.RequestCharactersInTexture(text, 72);

                    for (int c = 0; c < text.Length; c++)
                    {
                        if (serial != _sequenceSerial)
                            yield break;

                        char character = text[c];
                        float advance = GetCharacterAdvance(_activeDialogue.font, character);

                        if (character != ' ' && character != '\t')
                        {
                            try
                            {
                                GameObject symbol = FormulaSymbolFactory.CreateTextSymbol(
                                    _activeDialogue,
                                    _activeSymbolTemplate,
                                    character,
                                    x,
                                    symbolScale);

                                RegisterSymbol(symbol);
                            }
                            catch (Exception e)
                            {
                                _log?.LogError("[Formula3D] Text symbol creation failed: " + e);
                            }
                        }

                        x += advance * symbolScale;
                        _activeDialogue.xPrint = x;
                        yield return new WaitForSeconds(CharacterInterval);
                    }
                }

                float delay = Formula3DTextApi.AutoDropDelaySeconds;
                if (delay >= 0f)
                {
                    if (delay > 0f)
                        yield return new WaitForSeconds(delay);
                    else
                        yield return null;

                    if (serial != _sequenceSerial)
                        yield break;

                    DropActiveInternal("auto-timer");
                }
                else
                {
                    while (!_activeDropped && serial == _sequenceSerial)
                        yield return null;
                }

                if (serial != _sequenceSerial)
                    yield break;

                yield return new WaitForSeconds(DropCleanupDelay);
                DestroyActiveLine("sentence-finished");
                yield return new WaitForSeconds(BetweenSentenceDelay);
            }

            if (serial == _sequenceSerial)
                _currentSequenceRoutine = null;
        }

        private static Queue<List<FormulaToken>> BuildSentenceQueue(string fullText)
        {
            List<FormulaToken> parsed = FormulaTokenParser.Parse(fullText ?? string.Empty);
            Queue<List<FormulaToken>> queue = new();
            List<FormulaToken> current = new();
            StringBuilder text = new();

            for (int i = 0; i < parsed.Count; i++)
            {
                FormulaToken token = parsed[i];
                if (token.Kind == FormulaTokenKind.Formula)
                {
                    FlushTextToken(current, text);
                    current.Add(token);
                    continue;
                }

                string value = token.Content ?? string.Empty;
                for (int c = 0; c < value.Length; c++)
                {
                    char ch = value[c];
                    if (ch == '\r' || ch == '\n')
                    {
                        if (text.Length == 0 || text[text.Length - 1] != ' ')
                            text.Append(' ');
                        continue;
                    }

                    text.Append(ch);

                    if (IsSentenceTerminator(value, c))
                    {
                        FlushTextToken(current, text);
                        EnqueueIfNotEmpty(queue, current);
                        current = new List<FormulaToken>();
                    }
                }
            }

            FlushTextToken(current, text);
            EnqueueIfNotEmpty(queue, current);
            return queue;
        }

        private static bool IsSentenceTerminator(string text, int index)
        {
            char c = text[index];
            if (c == '。' || c == '！' || c == '？' || c == '!' || c == '?')
                return true;

            if (c != '.')
                return false;

            bool leftDigit = index > 0 && char.IsDigit(text[index - 1]);
            bool rightDigit = index + 1 < text.Length && char.IsDigit(text[index + 1]);
            return !(leftDigit && rightDigit);
        }

        private static void FlushTextToken(List<FormulaToken> line, StringBuilder text)
        {
            if (text.Length == 0)
                return;

            line.Add(FormulaToken.Text(text.ToString()));
            text.Clear();
        }

        private static void EnqueueIfNotEmpty(Queue<List<FormulaToken>> queue, List<FormulaToken> line)
        {
            if (line == null || line.Count == 0)
                return;

            bool hasContent = false;
            for (int i = 0; i < line.Count; i++)
            {
                if (!string.IsNullOrWhiteSpace(line[i].Content))
                {
                    hasContent = true;
                    break;
                }
            }

            if (hasContent)
                queue.Enqueue(line);
        }

        private static bool TryPrepareFormula(
            string latex,
            out FormulaTextureEntry formula,
            out string error)
        {
            formula = null;
            error = null;

            if (!FormulaRenderService.TryRender(latex, out FormulaRasterResult raster, out error))
                return false;

            /*
             * 使用当前游戏艺术字体的真实全角字符宽度。
             * 这会让公式 1～4 格、整句居中和前后间隔都与普通文字保持一致。
             */
            float nativeCellAdvance =
                MeasureNativeCellAdvance(
                    _templateDialogue != null
                        ? _templateDialogue.font
                        : null);

            int span =
                FormulaLayoutPolicy.ChooseCellSpan(
                    raster.PixelWidth,
                    raster.PixelHeight,
                    nativeCellAdvance);

            if (!FormulaSymbolFactory.TryCreateUnityTexture(
                "AI3D_FormulaTexture_" + ActiveTextures.Count,
                raster.PngBytes,
                out Texture2D texture,
                out error))
            {
                return false;
            }

            ActiveTextures.Add(texture);
            formula = new FormulaTextureEntry
            {
                Latex = latex,
                Texture = texture,
                PixelWidth = raster.PixelWidth,
                PixelHeight = raster.PixelHeight,
                CellSpan = span,
                NativeCellAdvance =
                    nativeCellAdvance
            };

            _log?.LogInfo(
                "[Formula3D] Formula prepared" +
                " | latex=" + latex +
                " | pixels=" + raster.PixelWidth + "x" + raster.PixelHeight +
                " | cells=" + span +
                " | nativeCellAdvance=" +
                nativeCellAdvance.ToString("F2"));

            return true;
        }

        private static bool CreateActiveDialogueRoot()
        {
            FindAndCacheAssets();

            if (!IsUsableTemplate(_templateDialogue) || _dialogueParent == null)
                return false;

            try
            {
                Vector3 localPosition = _privateTemplateRoot.transform.localPosition;
                Quaternion localRotation = _privateTemplateRoot.transform.localRotation;
                Vector3 localScale = _privateTemplateRoot.transform.localScale;

                _activeRoot = Object.Instantiate(_privateTemplateRoot, _dialogueParent);
                _activeRoot.name = "AI_3DText_Instance";
                _activeRoot.SetActive(false);
                _activeRoot.transform.localPosition = localPosition;
                _activeRoot.transform.localRotation = localRotation;
                _activeRoot.transform.localScale = localScale;

                _activeDialogue = _activeRoot.GetComponent<Dialogue_3DText>();
                if (!IsUsableTemplate(_activeDialogue))
                    throw new InvalidOperationException("Active Dialogue_3DText clone is incomplete.");

                ConfigureDialogueAsManualRoot(_activeDialogue);

                // 每句再次应用，允许游戏在运行中切换语言或全局字体。
                ApplyRuntimeNativeFontToDialogue(
                    _activeDialogue,
                    "active-dialogue-root");

                _activeSymbolTemplate = _activeDialogue.exampleSymbol;
                _activeSymbolTemplate.SetActive(false);
                _activeDropped = false;
                _activeRoot.SetActive(true);

                return true;
            }
            catch (Exception e)
            {
                _log?.LogError("[Formula3D] Create active root failed: " + e);
                DestroyActiveLine("create-root-failed");
                return false;
            }
        }

        private static void ConfigureDialogueAsManualRoot(Dialogue_3DText dialogue)
        {
            dialogue.enabled = false;
            dialogue.nextText = null;
            dialogue.timeShow = -1f;
            dialogue.timeFinish = 9999f;
            dialogue.timePrint = CharacterInterval;
            dialogue.stop = false;
            dialogue.voicePlay = false;
            dialogue.textPrint = string.Empty;
            dialogue.textPrintNow = string.Empty;
            dialogue.indexChar = 0;
            dialogue.align = Dialogue_3DText.Alignment3DText.Middle;
            dialogue.symbolObjects = new Il2CppSystem.Collections.Generic.List<GameObject>();
        }

        private static void RegisterSymbol(GameObject symbol)
        {
            if (symbol == null || _activeDialogue == null)
                return;

            ActiveSymbols.Add(symbol);
            _activeDialogue.symbolObjects.Add(symbol);
        }

        private static void DropActiveInternal(string source)
        {
            if (_activeRoot == null || ActiveSymbols.Count == 0)
            {
                PrintGame("当前没有可掉落的 AI 3D 文字。");
                return;
            }

            if (_activeDropped)
                return;

            _activeDropped = true;
            if (_activeDialogue != null)
                _activeDialogue.stop = true;

            int dropped = 0;
            for (int i = 0; i < ActiveSymbols.Count; i++)
            {
                GameObject obj = ActiveSymbols[i];
                if (obj == null) continue;

                try
                {
                    Dialogue_Symbol symbol = obj.GetComponent<Dialogue_Symbol>();
                    if (symbol != null && !symbol.destroy)
                    {
                        symbol.Jump();
                        dropped++;
                    }
                }
                catch (Exception e)
                {
                    _log?.LogWarning("[Formula3D] Drop failed: " + e.Message);
                }
            }

            _log?.LogInfo("[Formula3D] Drop started | source=" + source + " | symbols=" + dropped);
        }

        private static float MeasureTotalAdvance(List<PreparedDialogueToken> prepared, Font font)
        {
            float total = 0f;

            for (int i = 0; i < prepared.Count; i++)
            {
                PreparedDialogueToken item = prepared[i];
                if (item.Source.Kind == FormulaTokenKind.Formula && item.Formula != null)
                {
                    total +=
                        FormulaLayoutPolicy.GetAdvance(
                            item.Formula.CellSpan,
                            item.Formula.NativeCellAdvance);
                    continue;
                }

                string text = item.Source.Content ?? string.Empty;
                if (font != null && text.Length > 0)
                    font.RequestCharactersInTexture(text, 72);

                for (int c = 0; c < text.Length; c++)
                    total += GetCharacterAdvance(font, text[c]);
            }

            return Math.Max(
                MeasureNativeCellAdvance(font),
                total);
        }

        /// <summary>
        /// 测量游戏当前艺术字体中“一个普通全角字符”实际占用的水平宽度。
        ///
        /// Unity Font.GetCharacterInfo 返回的是当前 fontSize=72 下的 advance。
        /// 我们对多个常用 CJK 字符取中位数，避免单个艺术字的特殊留白影响公式排版。
        ///
        /// 测量失败时仍回退到历史 72，不会让整句无法显示。
        /// </summary>
        private static float MeasureNativeCellAdvance(
            Font font)
        {
            if (font == null)
            {
                return
                    FormulaLayoutPolicy
                        .StandardCharacterAdvance;
            }

            try
            {
                font.RequestCharactersInTexture(
                    NativeAdvanceProbeCharacters,
                    72);
            }
            catch { }

            List<float> advances =
                new List<float>();

            for (int i = 0;
                 i <
                    NativeAdvanceProbeCharacters.Length;
                 i++)
            {
                char character =
                    NativeAdvanceProbeCharacters[i];

                try
                {
                    if (font.GetCharacterInfo(
                            character,
                            out CharacterInfo info,
                            72) &&
                        info.advance > 0)
                    {
                        advances.Add(
                            info.advance);
                    }
                }
                catch { }
            }

            if (advances.Count == 0)
            {
                return
                    FormulaLayoutPolicy
                        .StandardCharacterAdvance;
            }

            advances.Sort();

            float median;

            int middle =
                advances.Count / 2;

            if ((advances.Count & 1) == 0)
            {
                median =
                    (advances[middle - 1] +
                     advances[middle]) *
                    0.5f;
            }
            else
            {
                median =
                    advances[middle];
            }

            return
                FormulaLayoutPolicy
                    .SanitizeNativeCellAdvance(
                        median);
        }

        private static float GetNativeSymbolScale(Dialogue_3DText dialogue)
        {
            float result = dialogue != null ? dialogue.sizeSymbol : DefaultSymbolScale;
            if (result <= 0.00001f || float.IsNaN(result) || float.IsInfinity(result))
                result = DefaultSymbolScale;
            return result;
        }

        private static float GetCharacterAdvance(Font font, char character)
        {
            if (font != null)
            {
                try
                {
                    font.GetCharacterInfo(character, out CharacterInfo info, 72);
                    if (info.advance > 0) return info.advance;
                }
                catch { }
            }

            return character == ' ' || character == '\t'
                ? 36f
                : FormulaLayoutPolicy.StandardCharacterAdvance;
        }

        private static void CancelSequenceAndDestroyActive(string reason)
        {
            _sequenceSerial++;

            if (_currentSequenceRoutine != null && Plugin.Runner != null)
            {
                try { Plugin.Runner.StopCoroutine(_currentSequenceRoutine); }
                catch { }
            }

            _currentSequenceRoutine = null;
            DestroyActiveLine(reason);
        }

        private static void DestroyActiveLine(string reason)
        {
            if (_activeRoot != null)
            {
                try { Object.Destroy(_activeRoot); }
                catch { }
            }

            _activeRoot = null;
            _activeDialogue = null;
            _activeSymbolTemplate = null;
            _activeDropped = false;
            ActiveSymbols.Clear();
            ActiveFormulaVisuals.Clear();

            for (int i = 0; i < ActiveTextures.Count; i++)
            {
                Texture2D texture = ActiveTextures[i];
                if (texture == null) continue;
                try { Object.Destroy(texture); }
                catch { }
            }

            ActiveTextures.Clear();
            _log?.LogDebug("[Formula3D] Active line cleared | reason=" + reason);
        }

        private static void ClearTemplateOnly(string reason)
        {
            if (_privateTemplateRoot != null)
            {
                try { Object.Destroy(_privateTemplateRoot); }
                catch { }
            }

            _privateTemplateRoot = null;
            _templateDialogue = null;
            _dialogueParent = null;
            _templateSceneHandle = int.MinValue;
            _log?.LogDebug("[Formula3D] Template cleared | reason=" + reason);
        }

        private static Transform FindDialogueQuestMitaTransform()
        {
            try
            {
                GameObject direct = GameObject.Find("DialogueQuest Mita");
                if (direct != null) return direct.transform;
            }
            catch { }

            try
            {
                GameObject world = GameObject.Find("World");
                if (world == null) return null;

                Stack<Transform> stack = new();
                stack.Push(world.transform);

                while (stack.Count > 0)
                {
                    Transform current = stack.Pop();
                    if (current == null) continue;

                    if (string.Equals(current.name, "DialogueQuest Mita", StringComparison.Ordinal))
                        return current;

                    for (int i = current.childCount - 1; i >= 0; i--)
                    {
                        Transform child = current.GetChild(i);
                        if (child != null) stack.Push(child);
                    }
                }
            }
            catch { }

            return null;
        }

        private static Dialogue_3DText FindBestTemplateUnder(Transform root)
        {
            if (root == null) return null;

            Stack<Transform> stack = new();
            stack.Push(root);
            Dialogue_3DText fallback = null;

            while (stack.Count > 0)
            {
                Transform current = stack.Pop();
                if (current == null) continue;

                try
                {
                    Dialogue_3DText dialogue = current.GetComponent<Dialogue_3DText>();
                    if (IsUsableTemplate(dialogue) && !IsPluginOwnedDialogue(dialogue))
                    {
                        if (!current.gameObject.activeInHierarchy)
                            return dialogue;
                        fallback ??= dialogue;
                    }
                }
                catch { }

                for (int i = current.childCount - 1; i >= 0; i--)
                {
                    try
                    {
                        Transform child = current.GetChild(i);
                        if (child != null) stack.Push(child);
                    }
                    catch { }
                }
            }

            return fallback;
        }

        private static bool IsPluginOwnedDialogue(Dialogue_3DText dialogue)
        {
            try
            {
                string name = dialogue?.gameObject?.name ?? string.Empty;
                return name.StartsWith("AI_Formula3D_", StringComparison.OrdinalIgnoreCase) ||
                       name.Equals("AI_3DText_Instance", StringComparison.OrdinalIgnoreCase);
            }
            catch { return false; }
        }

        private static bool IsUsableTemplate(Dialogue_3DText dialogue)
        {
            try
            {
                return dialogue != null &&
                       dialogue.gameObject != null &&
                       dialogue.exampleSymbol != null &&
                       dialogue.exampleSymbol.GetComponent<Dialogue_Symbol>() != null &&
                       dialogue.exampleSymbol.GetComponent<RectTransform>() != null;
            }
            catch { return false; }
        }

        private static string GetTransformPath(Transform target)
        {
            if (target == null) return "<null>";
            List<string> parts = new();
            Transform current = target;
            while (current != null)
            {
                try { parts.Add(current.name); }
                catch { parts.Add("<unknown>"); }
                try { current = current.parent; }
                catch { current = null; }
            }
            parts.Reverse();
            return string.Join("/", parts);
        }

        private static void PrintGame(string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); }
            catch { _log?.LogInfo("[Formula3D][GameConsole] " + text); }
        }
    }
}
