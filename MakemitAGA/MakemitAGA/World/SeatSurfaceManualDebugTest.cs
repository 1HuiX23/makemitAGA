/*
 * =================================================================================================
 * SeatSurfaceManualDebugTest.cs
 * =================================================================================================
 *
 * 手动演示/排查入口，不连接 VLM，也不会覆盖正式 svt_start 的分析结果。
 *
 * 控制台命令由 SeatVlmIntegration.cs 解析：
 *   debug_svt_test(Bed)
 *       对名称完全匹配的最近物体执行完整 SeatSurface 分析，并显示当前 debug_svt 对应的
 *       青/绿/红/橙/紫网格。
 *
 *   debug_svt_mesh(Bed)
 *       对名称完全匹配的最近物体执行顶视扫描，只显示青色完整高度图。
 *
 *   svt_test_clear
 *       只清理由上述两个命令生成的测试对象，不影响正式 svt_start、svt_clear、
 *       分析 MeshCollider 或 SeatActionProxy。
 *
 * 重名选择规则：
 *   1. 只匹配当前活动场景中 activeInHierarchy 的 GameObject；
 *   2. 名称不区分大小写，但必须完整相同；
 *   3. 优先选择离当前主游戏摄像机最近的对象；
 *   4. 没有可用摄像机时，回退到 MitaPerson；再失败则以世界原点作为参考。
 *
 * 重要实现约束：
 *   - 测试对象保存在独立列表中，不加入正式 _created；
 *   - 测试 Renderer 会从正式 _debugRenderers 中移除，因此 svt_clear/debug_svt 不会隐藏它们；
 *   - 测试用 MeshCollider 会被禁用，只用于复用网格构建结果，避免演示面干扰角色物理；
 *   - 场景切换和插件卸载仍会自动清理测试对象，防止静态引用残留。
 * =================================================================================================
 */

using System;
using System.Collections;
using System.Collections.Generic;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.World
{
    internal static partial class SeatSurfaceAnalysisRuntime
    {
        private enum ManualDebugViewMode
        {
            FullDebug,
            MeshOnly
        }

        private static readonly List<GameObject> _manualDebugCreated =
            new List<GameObject>();

        private static readonly List<Renderer> _manualDebugRenderers =
            new List<Renderer>();

        // SeatSurfaceSeatability.cs 会读取这个序号，以便在 svt_test_clear 或下一次测试时停止批处理。
        private static int _manualDebugScanSerial;
        private static bool _manualDebugScanInProgress;
        private static string _manualDebugScanStage = "Idle";

        public static bool ManualDebugScanInProgress
        {
            get { return _manualDebugScanInProgress; }
        }

        public static string GetManualDebugTestStatus()
        {
            return
                "manualDebugInProgress=" + _manualDebugScanInProgress +
                " | stage=" + _manualDebugScanStage +
                " | serial=" + _manualDebugScanSerial +
                " | roots=" + _manualDebugCreated.Count +
                " | renderers=" + _manualDebugRenderers.Count;
        }

        public static void RunManualDebugTestByName(
            string targetName,
            bool meshOnly,
            string source)
        {
            targetName = NormalizeManualTargetName(targetName);

            if (string.IsNullOrWhiteSpace(targetName))
            {
                TryConsolePrint(
                    meshOnly
                        ? "<color=yellow>用法：debug_svt_mesh(物体真实名称)</color>"
                        : "<color=yellow>用法：debug_svt_test(物体真实名称)</color>");
                return;
            }

            if (Plugin.Runner == null)
            {
                LogError("[SVT ManualDebug] Plugin.Runner is null.");
                TryConsolePrint("<color=red>SVT 测试失败：Plugin.Runner 不存在。</color>");
                return;
            }

            if (_scanInProgress)
            {
                TryConsolePrint(
                    "<color=yellow>正式 svt_start 扫描仍在运行，请等待完成或先 svt_cancel。</color>");
                return;
            }

            int exactMatchCount;
            float selectedDistance;
            string referenceName;
            string findError;

            GameObject target = FindNearestNamedSceneObject(
                targetName,
                out exactMatchCount,
                out selectedDistance,
                out referenceName,
                out findError);

            if (target == null)
            {
                LogWarning(
                    "[SVT ManualDebug] Target not found" +
                    " | requested=" + targetName +
                    " | error=" + findError);

                TryConsolePrint(
                    "<color=red>没有找到活动状态且名称完全匹配的物体：</color>" +
                    targetName +
                    (string.IsNullOrEmpty(findError)
                        ? string.Empty
                        : "\n" + findError));
                return;
            }

            // 下一次测试只替换上一次“测试展示”，不会触碰正式 svt_start 的对象。
            ClearManualDebugTest("replace-before-new-test", false);

            int serial = _manualDebugScanSerial;
            _manualDebugScanInProgress = true;
            _manualDebugScanStage = "Queued";

            ManualDebugViewMode mode =
                meshOnly
                    ? ManualDebugViewMode.MeshOnly
                    : ManualDebugViewMode.FullDebug;

            LogInfo(
                "[SVT ManualDebug] queued" +
                " | mode=" + mode +
                " | requested=" + targetName +
                " | selected=" + target.name +
                " | path=" + GetTransformPath(target.transform) +
                " | exactMatches=" + exactMatchCount +
                " | reference=" + referenceName +
                " | distance=" + selectedDistance.ToString("F3") +
                " | source=" + source);

            Plugin.Runner.StartCoroutine(
                ManualDebugScanRoutine(
                    target,
                    targetName,
                    exactMatchCount,
                    selectedDistance,
                    referenceName,
                    mode,
                    source,
                    serial)
                .WrapToIl2Cpp());
        }

        /// <summary>
        /// 默认会在控制台打印结果；场景切换/插件卸载可传 printResult=false 静默清理。
        /// </summary>
        public static void ClearManualDebugTest(
            string reason,
            bool printResult = true)
        {
            _manualDebugScanSerial++;
            _manualDebugScanInProgress = false;
            _manualDebugScanStage = "Idle";

            // 如果某个测试 Renderer 还被正式 debug 列表持有，先移除，避免后续访问已销毁对象。
            for (int i = _manualDebugRenderers.Count - 1; i >= 0; i--)
            {
                Renderer renderer = _manualDebugRenderers[i];
                if (renderer != null)
                    _debugRenderers.Remove(renderer);
            }

            _manualDebugRenderers.Clear();

            for (int i = _manualDebugCreated.Count - 1; i >= 0; i--)
                DestroyObject(_manualDebugCreated[i]);

            _manualDebugCreated.Clear();

            LogInfo(
                "[SVT ManualDebug] cleared" +
                " | reason=" + reason +
                " | productionSurfacePreserved=true");

            if (printResult)
            {
                TryConsolePrint(
                    "SVT 手动测试展示已清理；正式 svt_start 结果未受影响。");
            }
        }

        /// <summary>
        /// Coroutine exception wrapper.
        ///
        /// C# iterator methods do not allow yield return/yield break inside a try block
        /// that also has a catch clause (CS1626). The actual coroutine therefore lives
        /// in ManualDebugScanRoutineCore(), which uses try/finally only. This wrapper
        /// advances the core iterator inside a small non-yielding try/catch and yields
        /// the captured Current value afterwards.
        /// </summary>
        private static IEnumerator ManualDebugScanRoutine(
            GameObject target,
            string requestedName,
            int exactMatchCount,
            float selectedDistance,
            string referenceName,
            ManualDebugViewMode mode,
            string source,
            int serial)
        {
            IEnumerator core =
                ManualDebugScanRoutineCore(
                    target,
                    requestedName,
                    exactMatchCount,
                    selectedDistance,
                    referenceName,
                    mode,
                    source,
                    serial);

            try
            {
                while (true)
                {
                    bool movedNext = false;
                    object current = null;
                    Exception caught = null;

                    // No yield statement is present inside this try/catch.
                    try
                    {
                        movedNext = core.MoveNext();

                        if (movedNext)
                            current = core.Current;
                    }
                    catch (Exception e)
                    {
                        caught = e;
                    }

                    if (caught != null)
                    {
                        FinishManualDebugFailure(
                            serial,
                            caught.GetType().Name +
                            " / " + caught.Message);

                        LogError(
                            "[SVT ManualDebug] exception: " +
                            caught);

                        yield break;
                    }

                    if (!movedNext)
                        yield break;

                    yield return current;
                }
            }
            finally
            {
                IDisposable disposable =
                    core as IDisposable;

                if (disposable != null)
                    disposable.Dispose();
            }
        }

        /// <summary>
        /// Actual multi-frame scan implementation.
        /// It intentionally has no catch clause around yield statements; cleanup is
        /// handled by finally, while ManualDebugScanRoutine catches MoveNext errors.
        /// </summary>
        private static IEnumerator ManualDebugScanRoutineCore(
            GameObject target,
            string requestedName,
            int exactMatchCount,
            float selectedDistance,
            string referenceName,
            ManualDebugViewMode mode,
            string source,
            int serial)
        {
            EnsureMaterials();
            TryLoadDepthAssets();

            Bounds targetBounds;
            Bounds rendererBounds;
            Bounds colliderBounds;
            bool hasRendererBounds;
            bool hasColliderBounds;
            bool colliderExpandedVerticalRange;

            if (!TryGetTargetScanBounds(
                target,
                out targetBounds,
                out rendererBounds,
                out colliderBounds,
                out hasRendererBounds,
                out hasColliderBounds,
                out colliderExpandedVerticalRange))
            {
                FinishManualDebugFailure(
                    serial,
                    "目标没有可用 Renderer/Collider Bounds：" + requestedName);
                yield break;
            }

            if (serial != _manualDebugScanSerial)
                yield break;

            _manualDebugScanStage = "Preparing";

            Vector3 syntheticTopPoint =
                new Vector3(
                    targetBounds.center.x,
                    Mathf.Max(
                        targetBounds.center.y,
                        targetBounds.max.y - 0.05f),
                    targetBounds.center.z);

            Bounds volume = BuildScanVolume(
                syntheticTopPoint,
                true,
                targetBounds,
                FakeColliderRequest.Top(DefaultSeatProxyHeight));

            int scanLayer = PickScanLayer();

            LogInfo(
                "[SVT ManualDebug] start" +
                " | mode=" + mode +
                " | source=" + source +
                " | requested=" + requestedName +
                " | selected=" + target.name +
                " | path=" + GetTransformPath(target.transform) +
                " | exactMatches=" + exactMatchCount +
                " | reference=" + referenceName +
                " | distance=" + selectedDistance.ToString("F3") +
                " | scanBoundsCenter=" + Vec(targetBounds.center) +
                " | scanBoundsSize=" + Vec(targetBounds.size) +
                " | rendererBounds=" +
                (hasRendererBounds
                    ? Vec(rendererBounds.center) + " / " + Vec(rendererBounds.size)
                    : "<none>") +
                " | colliderBounds=" +
                (hasColliderBounds
                    ? Vec(colliderBounds.center) + " / " + Vec(colliderBounds.size)
                    : "<none>") +
                " | colliderExpandedY=" + colliderExpandedVerticalRange +
                " | scanVolume=" + Vec(volume.center) + " / " + Vec(volume.size));

            // 避开触发命令的输入帧，再执行 GPU ReadPixels。
            yield return null;

            List<LayerBackup> layerBackups =
                new List<LayerBackup>();

            GameObject scanCameraObject = null;
            Camera scanCamera = null;
            GameObject resultRoot = null;
            bool completed = false;

            try
            {
                if (serial != _manualDebugScanSerial)
                    yield break;

                _manualDebugScanStage = "GpuCapture";

                AssignLayerRecursive(
                    target,
                    scanLayer,
                    layerBackups);

                scanCameraObject =
                    new GameObject(
                        "BedSeatability_SVTTest_ScanCamera");

                scanCameraObject.hideFlags =
                    HideFlags.HideAndDontSave;

                scanCamera =
                    scanCameraObject.AddComponent<Camera>();

                ConfigureScanCamera(
                    scanCamera,
                    volume,
                    scanLayer);

                List<Vector3> vertices =
                    new List<Vector3>();

                List<int> triangles =
                    new List<int>();

                HeightfieldStats heightfield;
                DepthPath depthPath;

                bool captured =
                    CaptureTopViewToHeightfield(
                        scanCamera,
                        volume,
                        vertices,
                        triangles,
                        out heightfield,
                        out depthPath);

                // 尽早恢复目标原 Layer，避免批处理分析持续数帧时影响场景。
                RestoreLayers(layerBackups);
                layerBackups.Clear();

                if (!captured ||
                    !heightfield.valid ||
                    vertices.Count < 3 ||
                    triangles.Count < 3)
                {
                    FinishManualDebugFailure(
                        serial,
                        "顶视扫描失败。depthPath=" + depthPath +
                        " vertices=" + vertices.Count +
                        " triangles=" + (triangles.Count / 3));
                    yield break;
                }

                yield return null;

                if (serial != _manualDebugScanSerial)
                    yield break;

                _manualDebugScanStage = "BuildingMesh";

                resultRoot =
                    new GameObject(
                        "BedSeatability_SVTTest_ResultRoot_" +
                        serial + "_" + SanitizeObjectName(target.name));

                _manualDebugCreated.Add(resultRoot);

                if (mode == ManualDebugViewMode.MeshOnly)
                {
                    GameObject proxyMeshCollider =
                        BuildTopMeshColliderFromStats(
                            resultRoot.transform,
                            heightfield);

                    if (proxyMeshCollider == null)
                    {
                        FinishManualDebugFailure(
                            serial,
                            "完整高度图 MeshCollider 创建失败。");
                        yield break;
                    }

                    proxyMeshCollider.name =
                        "BedSeatability_SVTTest_SeatSurface_ProxyMeshCollider_Cyan";

                    DisableTestCollider(proxyMeshCollider);
                    AdoptManualRenderer(
                        proxyMeshCollider,
                        _proxyColliderVisualMat,
                        true);
                }
                else
                {
                    // 1. 与正式流程相同的青色完整高度图可视副本。
                    GameObject proxyVisual =
                        BuildHeightfieldMesh(
                            resultRoot.transform,
                            vertices,
                            triangles);

                    if (proxyVisual != null)
                    {
                        proxyVisual.name =
                            "BedSeatability_SVTTest_SeatSurface_ProxyColliderVisual_Cyan";

                        proxyVisual.transform.position +=
                            Vector3.up * ProxyVisualExtraLift;

                        AdoptManualRenderer(
                            proxyVisual,
                            _proxyColliderVisualMat,
                            true);
                    }

                    // 2. 与正式 debug_svt 相同的完整代理 MeshCollider 网格。
                    GameObject proxyMeshCollider =
                        BuildTopMeshColliderFromStats(
                            resultRoot.transform,
                            heightfield);

                    if (proxyMeshCollider != null)
                    {
                        proxyMeshCollider.name =
                            "BedSeatability_SVTTest_SeatSurface_ProxyMeshCollider";

                        // 测试对象只负责演示，禁用重复物理碰撞。
                        DisableTestCollider(proxyMeshCollider);
                        AdoptManualRenderer(
                            proxyMeshCollider,
                            null,
                            true);
                    }

                    yield return null;

                    if (serial != _manualDebugScanSerial)
                        yield break;

                    _manualDebugScanStage = "GeneralAnalysis";

                    SeatabilityStats seatability =
                        CreateSeatabilityStats(heightfield);

                    IEnumerator analysis =
                        AnalyzeSeatabilityBatched(
                            target,
                            targetBounds,
                            heightfield,
                            seatability,
                            serial,
                            true);

                    while (analysis.MoveNext())
                    {
                        if (serial != _manualDebugScanSerial)
                            yield break;

                        yield return analysis.Current;
                    }

                    if (serial != _manualDebugScanSerial)
                        yield break;

                    _manualDebugScanStage = "BuildingOverlays";

                    AdoptManualRenderer(
                        BuildSeatabilityOverlay(
                            resultRoot.transform,
                            heightfield,
                            seatability.validSeatCenters,
                            true,
                            "BedSeatability_SVTTest_ValidSupport_Green",
                            _validSeatSurfaceMat,
                            SeatOverlayLift),
                        null,
                        true);

                    yield return null;

                    AdoptManualRenderer(
                        BuildSeatabilityOverlay(
                            resultRoot.transform,
                            heightfield,
                            seatability.validSeatCenters,
                            false,
                            "BedSeatability_SVTTest_InvalidSupport_Red",
                            _invalidSeatSurfaceMat,
                            InvalidOverlayLift),
                        null,
                        true);

                    yield return null;

                    AdoptManualRenderer(
                        BuildSeatabilityOverlay(
                            resultRoot.transform,
                            heightfield,
                            seatability.heightWarningCenters,
                            true,
                            "BedSeatability_SVTTest_HeightWarning_Orange",
                            _heightWarningSurfaceMat,
                            HeightWarningOverlayLift),
                        null,
                        true);

                    yield return null;

                    AdoptManualRenderer(
                        BuildSeatabilityOverlay(
                            resultRoot.transform,
                            heightfield,
                            seatability.actionValidCenters,
                            true,
                            "BedSeatability_SVTTest_ActionValid_Purple",
                            _actionSeatSurfaceMat,
                            ActionOverlayLift),
                        null,
                        true);

                    LogInfo(
                        "[SVT ManualDebug] analysis summary" +
                        " | target=" + target.name +
                        " | proxyCells=" + seatability.proxySurfaceCells +
                        " | supportValid=" + seatability.validSeatCells +
                        " | softHeight=" + seatability.heightWarningCells +
                        " | actionValid=" + seatability.actionValidCells +
                        " | floorFound=" + seatability.floorFound +
                        " | floorY=" + seatability.floorY.ToString("F3"));
                }

                if (serial != _manualDebugScanSerial)
                    yield break;

                _manualDebugScanStage = "Completed";
                _manualDebugScanInProgress = false;
                completed = true;

                string completedMessage =
                    "SVT 手动测试完成" +
                    " | mode=" + mode +
                    " | target=" + target.name +
                    " | exactMatches=" + exactMatchCount +
                    " | selectedDistance=" + selectedDistance.ToString("F2") +
                    "m | depthPath=" + depthPath;

                LogInfo("[SVT ManualDebug] " + completedMessage);

                TryConsolePrint(
                    (mode == ManualDebugViewMode.MeshOnly
                        ? "<color=cyan>只显示青色完整高度图。</color> "
                        : "<color=green>已显示完整 debug_svt 分析网格。</color> ") +
                    completedMessage +
                    "\n使用 svt_test_clear 单独清理。");
            }
            finally
            {
                RestoreLayers(layerBackups);

                try
                {
                    if (scanCamera != null)
                        scanCamera.targetTexture = null;
                }
                catch { }

                DestroyObject(scanCameraObject);

                if (!completed &&
                    serial == _manualDebugScanSerial)
                {
                    _manualDebugScanInProgress = false;

                    if (resultRoot != null)
                    {
                        RemoveManualRenderersUnderRoot(
                            resultRoot.transform);

                        _manualDebugCreated.Remove(resultRoot);
                        DestroyObject(resultRoot);
                    }

                    if (_manualDebugScanStage != "Failed")
                        _manualDebugScanStage = "Cancelled";
                }
            }
        }

        private static GameObject FindNearestNamedSceneObject(
            string targetName,
            out int exactMatchCount,
            out float selectedDistance,
            out string referenceName,
            out string error)
        {
            exactMatchCount = 0;
            selectedDistance = float.MaxValue;
            referenceName = "WorldOrigin";
            error = null;

            Vector3 referencePosition = Vector3.zero;
            Camera camera = FindBestCamera();

            if (camera != null)
            {
                referencePosition = camera.transform.position;
                referenceName =
                    "Camera:" + camera.name;
            }
            else
            {
                MitaPerson mita = null;
                try { mita = Object.FindObjectOfType<MitaPerson>(); }
                catch { }

                if (mita != null)
                {
                    referencePosition = mita.transform.position;
                    referenceName =
                        "Mita:" + mita.name;
                }
            }

            GameObject[] all = null;
            try { all = Object.FindObjectsOfType<GameObject>(); }
            catch (Exception e)
            {
                error =
                    "枚举场景对象失败：" +
                    e.GetType().Name + " / " + e.Message;
                return null;
            }

            if (all == null || all.Length == 0)
            {
                error = "当前活动场景没有可枚举的 GameObject。";
                return null;
            }

            GameObject best = null;
            string bestPath = null;

            for (int i = 0; i < all.Length; i++)
            {
                GameObject candidate = all[i];

                if (candidate == null ||
                    !candidate.activeInHierarchy ||
                    IsOwnVisual(candidate))
                {
                    continue;
                }

                string candidateName =
                    candidate.name ?? string.Empty;

                if (!candidateName.Equals(
                    targetName,
                    StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                exactMatchCount++;

                Bounds scanBounds;
                Bounds rendererBounds;
                Bounds colliderBounds;
                bool hasRendererBounds;
                bool hasColliderBounds;
                bool colliderExpandedVerticalRange;

                if (!TryGetTargetScanBounds(
                    candidate,
                    out scanBounds,
                    out rendererBounds,
                    out colliderBounds,
                    out hasRendererBounds,
                    out hasColliderBounds,
                    out colliderExpandedVerticalRange))
                {
                    LogWarning(
                        "[SVT ManualDebug] exact-name candidate skipped: no usable bounds" +
                        " | path=" + GetTransformPath(candidate.transform));
                    continue;
                }

                float distance =
                    Vector3.Distance(
                        referencePosition,
                        scanBounds.center);

                string path =
                    GetTransformPath(candidate.transform);

                bool better =
                    best == null ||
                    distance < selectedDistance - 0.0001f ||
                    (Mathf.Abs(distance - selectedDistance) <= 0.0001f &&
                     string.Compare(
                         path,
                         bestPath,
                         StringComparison.OrdinalIgnoreCase) < 0);

                if (better)
                {
                    best = candidate;
                    bestPath = path;
                    selectedDistance = distance;
                }
            }

            if (exactMatchCount == 0)
            {
                error =
                    "没有发现名称完全等于 “" +
                    targetName +
                    "” 的活动对象。";
            }
            else if (best == null)
            {
                error =
                    "找到了 " + exactMatchCount +
                    " 个同名对象，但它们都没有可用 Renderer/Collider Bounds。";
            }

            return best;
        }

        private static void RemoveManualRenderersUnderRoot(
            Transform root)
        {
            if (root == null)
                return;

            for (int i = _manualDebugRenderers.Count - 1; i >= 0; i--)
            {
                Renderer renderer = _manualDebugRenderers[i];

                if (renderer == null)
                {
                    _manualDebugRenderers.RemoveAt(i);
                    continue;
                }

                Transform rendererTransform = renderer.transform;
                bool belongsToRoot =
                    rendererTransform == root ||
                    rendererTransform.IsChildOf(root);

                if (!belongsToRoot)
                    continue;

                _debugRenderers.Remove(renderer);
                _manualDebugRenderers.RemoveAt(i);
            }
        }

        private static void AdoptManualRenderer(
            GameObject owner,
            Material materialOverride,
            bool visible)
        {
            if (owner == null)
                return;

            Renderer renderer = null;
            try { renderer = owner.GetComponent<Renderer>(); }
            catch { }

            if (renderer == null)
                return;

            // BuildHeightfieldMesh/BuildSeatabilityOverlay 会自动注册到正式 debug 列表；
            // 这里立即转移到测试列表，使 svt_clear/debug_svt 不会控制它。
            _debugRenderers.Remove(renderer);

            if (materialOverride != null)
            {
                try { renderer.material = materialOverride; }
                catch { }
            }

            try { renderer.enabled = visible; }
            catch { }

            if (!_manualDebugRenderers.Contains(renderer))
                _manualDebugRenderers.Add(renderer);
        }

        private static void DisableTestCollider(
            GameObject owner)
        {
            if (owner == null)
                return;

            try
            {
                MeshCollider meshCollider =
                    owner.GetComponent<MeshCollider>();

                if (meshCollider != null)
                    meshCollider.enabled = false;
            }
            catch { }
        }

        private static void FinishManualDebugFailure(
            int serial,
            string error)
        {
            if (serial != _manualDebugScanSerial)
                return;

            _manualDebugScanInProgress = false;
            _manualDebugScanStage = "Failed";

            LogError(
                "[SVT ManualDebug] failed" +
                " | error=" + error);

            TryConsolePrint(
                "<color=red>SVT 手动测试失败：</color>" + error);
        }

        private static string NormalizeManualTargetName(
            string value)
        {
            string result =
                (value ?? string.Empty).Trim();

            if (result.Length >= 2)
            {
                if ((result[0] == '"' &&
                     result[result.Length - 1] == '"') ||
                    (result[0] == '\'' &&
                     result[result.Length - 1] == '\''))
                {
                    result = result.Substring(
                        1,
                        result.Length - 2)
                        .Trim();
                }
            }

            return result;
        }

        private static string SanitizeObjectName(
            string value)
        {
            if (string.IsNullOrEmpty(value))
                return "Unnamed";

            char[] chars = value.ToCharArray();

            for (int i = 0; i < chars.Length; i++)
            {
                char c = chars[i];
                if (c == '/' || c == '\\' || c == ':' ||
                    c == '*' || c == '?' || c == '"' ||
                    c == '<' || c == '>' || c == '|')
                {
                    chars[i] = '_';
                }
            }

            return new string(chars);
        }
    }
}
