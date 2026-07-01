/*
 * =================================================================================================
 * SeatSurfaceSelectionLifecycle.cs
 * =================================================================================================
 *
 * 作用：连接 VLM select_2D 与分析网格，并管理扫描结果、调试显示和资源生命周期。

 * 主要逻辑：
 *   - 深度 AssetBundle 资源加载；
 *   - 直接选点、最近紫色有效点吸附与来源校验；
 *   - Composite Preview 数据源；
 *   - 自动/手动清理，保留分析与动作代理 Collider；
 *   - debug_svt 显示控制、材质创建、对象销毁与 IL2CPP List 转换。
 *
 * 维护约束：
 *   - 本文件是 SeatSurfaceAnalysisRuntime 的 partial 实现；
 *   - 共享状态、常量和嵌套数据结构统一定义在 SeatSurfaceAnalysisMesh.cs；
 *   - 不要在多个 partial 文件中重复声明同名字段或方法；
 *   - 拆分只改变源码组织，不改变运行流程、Collider、VLM 或调试行为。
 * =================================================================================================
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using BepInEx;
using BepInEx.Logging;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using MakemitAGA.Mita_self.Mita_tools;
using HarmonyLib;
using UnityEngine;
using UnityEngine.AI;
using Object = UnityEngine.Object;

namespace MakemitAGA.World
{
    internal static partial class SeatSurfaceAnalysisRuntime
    {
        public static void TryLoadDepthAssets(bool forceReload = false)
        {
            if (!forceReload && _depthToEyeMat != null && _eyeDepthReplacementShader != null)
                return;

            if (forceReload) { _depthToEyeMat = null; _eyeDepthReplacementShader = null; }
            string path;
            try { path = Path.Combine(Paths.PluginPath, BundleFileName); }
            catch { path = BundleFileName; }

            if (!string.IsNullOrEmpty(path) && File.Exists(path))
            {
                try
                {
                    Shader shader;
                    if (ICallAssetBundleLoader.TryLoadShaderByName(path, EyeDepthReplacementShaderAssetName, out shader) && shader != null)
                    {
                        _eyeDepthReplacementShader = shader;
                        LogInfo("Loaded EyeDepthReplacement shader from mita_actions: " + shader.name);
                    }
                }
                catch (Exception e) { LogWarning("EyeDepthReplacement shader AB load failed: " + e.Message); }

                try
                {
                    Material mat;
                    if (ICallAssetBundleLoader.TryLoadMaterialByName(path, MaterialName, out mat) && mat != null)
                    {
                        _depthToEyeMat = mat;
                        LogInfo("Loaded depth material from mita_actions: " + mat.name);
                    }
                }
                catch (Exception e) { LogWarning("DepthToEye material AB load failed: " + e.Message); }
            }
            else LogWarning("Depth AssetBundle not found: " + path);

            if (_eyeDepthReplacementShader == null)
            {
                try
                {
                    Shader fallback = Shader.Find(EyeDepthReplacementShaderFindName);
                    if (fallback != null) { _eyeDepthReplacementShader = fallback; LogInfo("Loaded EyeDepthReplacement shader from Shader.Find: " + fallback.name); }
                }
                catch { }
            }

            if (_depthToEyeMat == null)
            {
                try
                {
                    Shader fallback = Shader.Find(ShaderName);
                    if (fallback != null) { _depthToEyeMat = new Material(fallback); LogInfo("Loaded DepthToEye material from Shader.Find fallback."); }
                }
                catch { }
            }

            if (_eyeDepthReplacementShader == null) LogWarning("EyeDepthReplacement shader unavailable. Depth capture will probably fail.");
        }

        public static bool IsScanInProgress
        {
            get { return _scanInProgress; }
        }

        public static bool LastScanSucceeded
        {
            get
            {
                return
                    !_scanInProgress &&
                    _scanStage == "Completed" &&
                    _lastSelectionDataValid;
            }
        }

        public static string ScanStage
        {
            get { return _scanStage; }
        }

        public static bool TrySelectActionPoint(
            Camera camera,
            float x,
            float yTop,
            out SeatSurfaceSelectionResult result,
            out string error)
        {
            result = null;
            error = null;

            if (!ValidateSelectionSource(camera, out error))
                return false;

            if (!IsNormalized(x) ||
                !IsNormalized(yTop))
            {
                error = "select_2D 坐标必须位于 0 到 1。";
                return false;
            }

            Ray ray =
                camera.ViewportPointToRay(
                    new Vector3(
                        x,
                        1f - yTop,
                        0f));

            RaycastHit[] hits = null;

            try
            {
                hits = Physics.RaycastAll(
                    ray,
                    RayDistance,
                    -1,
                    QueryTriggerInteraction.Ignore);
            }
            catch (Exception e)
            {
                error =
                    "代理射线失败：" +
                    e.GetType().Name +
                    " / " +
                    e.Message;

                return false;
            }

            if (hits == null ||
                hits.Length == 0)
            {
                error = "select_2D 射线没有命中代理表面。";
                return false;
            }

            Array.Sort(
                hits,
                delegate (RaycastHit a, RaycastHit b)
                {
                    return a.distance.CompareTo(b.distance);
                });

            RaycastHit proxyHit =
                new RaycastHit();

            bool foundProxy = false;

            for (int i = 0;
                 i < hits.Length;
                 i++)
            {
                Transform hitTransform =
                    hits[i].transform;

                if (hitTransform == null)
                    continue;

                if (_lastProxyColliderObject != null &&
                    (hitTransform ==
                        _lastProxyColliderObject.transform ||
                     hitTransform.IsChildOf(
                        _lastProxyColliderObject.transform)))
                {
                    proxyHit = hits[i];
                    foundProxy = true;
                    break;
                }
            }

            if (!foundProxy)
            {
                error =
                    "二维点没有命中当前目标的代理 MeshCollider。";

                return false;
            }

            int gridX;
            int gridZ;

            if (!TryWorldToGrid(
                proxyHit.point,
                out gridX,
                out gridZ))
            {
                error =
                    "代理命中点无法映射到高度图网格。";

                return false;
            }

            if (!_lastSelectionSeatability
                .actionValidCenters[gridX, gridZ])
            {
                bool supportValid =
                    _lastSelectionSeatability
                        .validSeatCenters[gridX, gridZ];

                error =
                    supportValid
                        ? "该点能够承重，但不属于紫色床边动作有效区。"
                        : "该点不属于绿色支撑有效区。";

                return false;
            }

            Vector2 selectedPoint =
                new Vector2(x, yTop);

            result = BuildSelectionResult(
                gridX,
                gridZ,
                selectedPoint,
                selectedPoint,
                false,
                0f);

            return result != null;
        }

        public static bool TryFindNearestActionPoint(
            Camera camera,
            Vector2 originalViewportTopLeft,
            out SeatSurfaceSelectionResult result,
            out string error)
        {
            result = null;
            error = null;

            if (!ValidateSelectionSource(
                camera,
                out error))
            {
                return false;
            }

            float bestScore =
                float.MaxValue;

            float bestDistance =
                float.MaxValue;

            int actionValidCount = 0;
            int visibleActionValidCount = 0;

            int bestX = -1;
            int bestZ = -1;
            Vector2 bestViewport =
                Vector2.zero;

            for (int z = 0;
                 z < MeshGrid;
                 z++)
            {
                for (int x = 0;
                     x < MeshGrid;
                     x++)
                {
                    if (!_lastSelectionSeatability
                        .actionValidCenters[x, z])
                    {
                        continue;
                    }

                    actionValidCount++;

                    Vector3 world =
                        GridToWorld(
                            _lastSelectionHeightfield,
                            x,
                            z,
                            _lastSelectionHeightfield
                                .heights[x, z]);

                    Vector3 viewport =
                        camera.WorldToViewportPoint(
                            world);

                    if (viewport.z <= 0f)
                        continue;

                    Vector2 topLeft =
                        new Vector2(
                            viewport.x,
                            1f - viewport.y);

                    if (!IsNormalized(topLeft.x) ||
                        !IsNormalized(topLeft.y))
                    {
                        continue;
                    }

                    visibleActionValidCount++;

                    float distance =
                        Vector2.Distance(
                            originalViewportTopLeft,
                            topLeft);

                    float heightWarningPenalty =
                        _lastSelectionSeatability
                            .heightWarningCenters[x, z]
                            ? 0.025f
                            : 0f;

                    float score =
                        distance +
                        heightWarningPenalty;

                    if (score < bestScore)
                    {
                        bestScore = score;
                        bestDistance = distance;
                        bestX = x;
                        bestZ = z;
                        bestViewport = topLeft;
                    }
                }
            }

            if (bestX < 0)
            {
                error =
                    "当前代理没有任何可见的紫色动作有效点。" +
                    " actionValidTotal=" +
                    actionValidCount +
                    " | visibleInFrozenCamera=" +
                    visibleActionValidCount +
                    " | cameraPos=" +
                    camera.transform.position +
                    " | cameraForward=" +
                    camera.transform.forward;

                return false;
            }

            if (bestDistance >
                MaxSnapViewportDistance)
            {
                error =
                    "最近紫色动作点距离模型原始选择过远：" +
                    bestDistance.ToString(
                        "0.###",
                        System.Globalization.CultureInfo.InvariantCulture);

                return false;
            }

            result = BuildSelectionResult(
                bestX,
                bestZ,
                originalViewportTopLeft,
                bestViewport,
                true,
                bestDistance);

            return result != null;
        }

        private static bool ValidateSelectionSource(
            Camera camera,
            out string error)
        {
            error = null;

            if (camera == null)
            {
                error =
                    "辅助图对应的冻结选择相机不存在。";

                return false;
            }

            if (!_lastSelectionDataValid ||
                _lastSelectionSeatability == null ||
                !_lastSelectionHeightfield.valid ||
                _lastProxyColliderObject == null)
            {
                error =
                    "可坐代理数据尚未准备好。";

                return false;
            }

            return true;
        }

        private static bool TryWorldToGrid(
            Vector3 world,
            out int gridX,
            out int gridZ)
        {
            gridX = -1;
            gridZ = -1;

            Bounds volume =
                _lastSelectionHeightfield.volume;

            if (volume.size.x <= 0.001f ||
                volume.size.z <= 0.001f)
            {
                return false;
            }

            float nx =
                (world.x - volume.min.x) /
                volume.size.x;

            float nz =
                (world.z - volume.min.z) /
                volume.size.z;

            if (nx < -0.01f ||
                nx > 1.01f ||
                nz < -0.01f ||
                nz > 1.01f)
            {
                return false;
            }

            gridX =
                Mathf.Clamp(
                    Mathf.RoundToInt(
                        nx * (MeshGrid - 1)),
                    0,
                    MeshGrid - 1);

            gridZ =
                Mathf.Clamp(
                    Mathf.RoundToInt(
                        nz * (MeshGrid - 1)),
                    0,
                    MeshGrid - 1);

            return
                _lastSelectionHeightfield
                    .surfaceMask[gridX, gridZ];
        }

        private static SeatSurfaceSelectionResult
            BuildSelectionResult(
                int gridX,
                int gridZ,
                Vector2 originalViewportTopLeft,
                Vector2 selectedViewportTopLeft,
                bool snapped,
                float snapDistance)
        {
            if (!_lastSelectionSeatability
                .actionValidCenters[gridX, gridZ])
            {
                return null;
            }

            Vector3 seatPoint =
                GridToWorld(
                    _lastSelectionHeightfield,
                    gridX,
                    gridZ,
                    _lastSelectionHeightfield
                        .heights[gridX, gridZ]);

            Vector3 floorPoint =
                _lastSelectionSeatability
                    .actionFloorPoints[gridX, gridZ];

            Vector3 outward =
                _lastSelectionSeatability
                    .actionOutwardDirections[gridX, gridZ];

            return new SeatSurfaceSelectionResult
            {
                Target = _lastTargetObject,
                OriginalViewportTopLeft =
                    originalViewportTopLeft,
                SelectedViewportTopLeft =
                    selectedViewportTopLeft,
                WorldSeatPoint = seatPoint,
                FloorPoint = floorPoint,
                OutwardDirection = outward,
                HeightAboveFloor =
                    seatPoint.y -
                    floorPoint.y,
                IsSnapped = snapped,
                SnapViewportDistance =
                    snapDistance,
                HeightWarning =
                    _lastSelectionSeatability
                        .heightWarningCenters[
                            gridX,
                            gridZ]
            };
        }

        private static bool IsNormalized(
            float value)
        {
            return
                !float.IsNaN(value) &&
                !float.IsInfinity(value) &&
                value >= 0f &&
                value <= 1f;
        }

        public static bool HasCompositePreviewSource
        {
            get
            {
                GameObject bed;
                Transform resultRoot;
                string error;

                return TryGetCompositePreviewSource(
                    out bed,
                    out resultRoot,
                    out error);
            }
        }

        public static bool TryGetCompositePreviewSource(
            out GameObject target,
            out Transform resultRoot,
            out string error)
        {
            target = _lastTargetObject;
            resultRoot = _lastResultRoot;
            error = null;

            if (resultRoot == null)
            {
                error = "SeatSurface analysis result root is null.";
                return false;
            }

            if (target == null)
            {
                error = "The analysis result exists, but the selected target is no longer valid.";
                return false;
            }

            try
            {
                if (resultRoot.gameObject == null)
                {
                    error = "SeatSurface analysis result root GameObject is invalid.";
                    return false;
                }

                if (target.gameObject == null)
                {
                    error = "Selected target GameObject is invalid.";
                    return false;
                }
            }
            catch (Exception e)
            {
                error =
                    "Preview source validation exception: " +
                    e.GetType().Name +
                    " / " +
                    e.Message;

                return false;
            }

            return true;
        }

        public static string GetCompositePreviewSourceStatus()
        {
            GameObject target;
            Transform resultRoot;
            string error;

            bool ready = TryGetCompositePreviewSource(
                out target,
                out resultRoot,
                out error);

            return
                "ready=" + ready +
                " | target=" +
                (target == null ? "<null>" : target.name) +
                " | root=" +
                (resultRoot == null ? "<null>" : resultRoot.name) +
                " | createdCount=" + _created.Count +
                " | error=" +
                (string.IsNullOrEmpty(error) ? "<none>" : error);
        }

        public static GameObject LastTargetObject
        {
            get { return _lastTargetObject; }
        }

        public static Transform LastResultRoot
        {
            get { return _lastResultRoot; }
        }

        public static GameObject ActionMarkerRoot
        {
            get { return _actionMarkerRoot; }
        }

        public static GameObject ClearanceDebugRoot
        {
            get { return _clearanceDebugRoot; }
        }

        public static GameObject ScanVolumeDebugRoot
        {
            get { return _scanVolumeDebugRoot; }
        }

        public static bool DebugRenderersVisible
        {
            get { return _debugRenderersVisible; }
        }

        /// <summary>
        /// 清理本轮展示内容，但保留分析 MeshCollider 与选择数据。
        ///
        /// svt_clear 和模型完成后的自动清理调用这里：
        /// 彩色网格、扫描框和动作标记会隐藏，Collider 不会被销毁，
        /// 因而不会影响已经派生出的 SeatActionProxy 或后续物理查询。
        /// </summary>
        public static void ClearTransientVisualsPreserveSurface()
        {
            SetDebugRenderersVisible(false);

            try
            {
                if (_actionMarkerRoot != null)
                    _actionMarkerRoot.SetActive(false);

                if (_clearanceDebugRoot != null)
                    _clearanceDebugRoot.SetActive(false);

                if (_scanVolumeDebugRoot != null)
                    _scanVolumeDebugRoot.SetActive(false);

                if (_crosshairRoot != null)
                    _crosshairRoot.SetActive(false);
            }
            catch { }

            _actionMarkersVisible = false;
            _clearanceDebugVisible = false;
            _scanVolumeVisible = false;
        }

        public static void ClearAll()
        {
            _scanSerial++;
            _scanInProgress = false;
            _scanStage = "Idle";

            ClearProxyOnly();
            DestroyObject(_crosshairRoot);
            _crosshairRoot = null;
            _crosshairCamera = null;
        }

        private static void ClearProxyOnly()
        {
            _pendingBedTopSit = false;
            _pendingBedTopSitMita = null;
            _pendingBedTopSitGoto = null;
            _pendingBedTopSitSeatProxyName = null;
            _pendingBridgeSit = false;
            _pendingBridgeTargetName = null;
            for (int i = _created.Count - 1; i >= 0; i--) DestroyObject(_created[i]);
            _created.Clear();
            _debugRenderers.Clear();
            _actionMarkerRoot = null;
            _clearanceDebugRoot = null;
            _scanVolumeDebugRoot = null;

            _lastResultRoot = null;
            _lastTargetObject = null;
            _lastScanVolume = new Bounds();

            _lastSelectionHeightfield =
                new HeightfieldStats();

            _lastSelectionSeatability = null;
            _lastProxyColliderObject = null;
            _lastSelectionDataValid = false;
            _hasLastScanVolume = false;

            _actionMarkersVisible = ShowActionMarkersByDefault;
            _clearanceDebugVisible = ShowClearanceDebugByDefault;
            _scanVolumeVisible = false;
            _debugRenderersVisible = false;
        }

        public static void RecreateCrosshairFromConsole()
        {
            DestroyObject(_crosshairRoot);
            _crosshairRoot = null;
            _crosshairCamera = null;
            EnsureCrosshair();
        }

        public static void ToggleActionMarkers()
        {
            _actionMarkersVisible = !_actionMarkersVisible;

            if (_actionMarkerRoot != null)
                _actionMarkerRoot.SetActive(_actionMarkersVisible);

            LogInfo(
                "Action markers visible=" +
                _actionMarkersVisible);
        }

        public static void ToggleClearanceDebug()
        {
            _clearanceDebugVisible =
                !_clearanceDebugVisible;

            if (_clearanceDebugRoot != null)
            {
                _clearanceDebugRoot.SetActive(
                    _clearanceDebugVisible);
            }

            LogInfo(
                "Clearance debug visible=" +
                _clearanceDebugVisible);
        }

        public static void ToggleScanVolume()
        {
            if (_scanVolumeDebugRoot != null)
            {
                DestroyObject(_scanVolumeDebugRoot);
                _scanVolumeDebugRoot = null;
                _scanVolumeVisible = false;

                LogInfo(
                    "Scan volume visible=false (destroyed).");

                return;
            }

            if (!_hasLastScanVolume ||
                _lastResultRoot == null)
            {
                LogWarning(
                    "Scan volume unavailable: run TAB/bst_scan first.");

                return;
            }

            _scanVolumeDebugRoot =
                CreateDebugBoxObject(
                    _lastScanVolume,
                    _lastResultRoot,
                    "BedSeatability_ScanVolume_DebugBox");

            _scanVolumeVisible =
                _scanVolumeDebugRoot != null;

            LogInfo(
                "Scan volume visible=" +
                _scanVolumeVisible +
                " (created lazily).");
        }

        public static void SetDebugRenderersVisible(bool visible)
        {
            _debugRenderersVisible = visible;

            for (int i = _debugRenderers.Count - 1; i >= 0; i--)
            {
                Renderer r = _debugRenderers[i];
                if (r == null)
                {
                    _debugRenderers.RemoveAt(i);
                    continue;
                }

                try { r.enabled = visible; }
                catch { }
            }

            LogInfo(
                "SetDebugRenderersVisible(" +
                visible +
                ") applied to " +
                _debugRenderers.Count +
                " renderers.");
        }

        private static void RegisterDebugRenderer(Renderer r)
        {
            if (r == null) return;

            _debugRenderers.Add(r);

            // 正式项目默认不把分析网格显示到玩家主画面。
            // 私有辅助相机截图时会临时打开；debug_svt 也可以显式切换。
            try { r.enabled = _debugRenderersVisible; }
            catch { }
        }

        private static void EnsureCrosshair()
        {
            Camera cam = FindBestCamera();
            if (cam == null) return;
            if (_crosshairRoot != null && _crosshairCamera == cam)
            {
                _crosshairRoot.transform.SetParent(cam.transform, false);
                _crosshairRoot.transform.localPosition = new Vector3(0f, 0f, Mathf.Max(CrosshairDepth, cam.nearClipPlane + 0.15f));
                _crosshairRoot.transform.localRotation = Quaternion.identity;
                _crosshairRoot.SetActive(_debugRenderersVisible);
                return;
            }

            DestroyObject(_crosshairRoot);
            EnsureMaterials();
            _crosshairRoot = new GameObject("TopSurfaceSeatProxy_Crosshair");
            _crosshairRoot.transform.SetParent(cam.transform, false);
            _crosshairRoot.transform.localPosition = new Vector3(0f, 0f, Mathf.Max(CrosshairDepth, cam.nearClipPlane + 0.15f));
            _crosshairRoot.transform.localRotation = Quaternion.identity;
            _crosshairCamera = cam;
            _crosshairRoot.SetActive(_debugRenderersVisible);

            GameObject h = GameObject.CreatePrimitive(PrimitiveType.Cube);
            h.name = "Crosshair_Horizontal";
            h.transform.SetParent(_crosshairRoot.transform, false);
            h.transform.localScale = new Vector3(0.055f, 0.0045f, 0.0045f);
            SetRendererMaterial(h, _redMat, false);
            DisableCollider(h);

            GameObject v = GameObject.CreatePrimitive(PrimitiveType.Cube);
            v.name = "Crosshair_Vertical";
            v.transform.SetParent(_crosshairRoot.transform, false);
            v.transform.localScale = new Vector3(0.0045f, 0.055f, 0.0045f);
            SetRendererMaterial(v, _redMat, false);
            DisableCollider(v);
        }

        private static Camera FindBestCamera()
        {
            Camera main = null;
            try { main = Camera.main; } catch { }
            if (IsUsableCamera(main)) return main;

            Camera[] cameras = null;
            try { cameras = Object.FindObjectsOfType<Camera>(); } catch { }
            if (cameras == null || cameras.Length == 0) return null;

            Camera best = null;
            float bestScore = float.MinValue;
            for (int i = 0; i < cameras.Length; i++)
            {
                Camera cam = cameras[i];
                if (!IsUsableCamera(cam)) continue;
                string name = cam.name ?? string.Empty;
                float score = cam.depth;
                if (cam.targetTexture == null) score += 1000f;
                if (cam.gameObject.activeInHierarchy) score += 100f;
                if (name.IndexOf("Main", StringComparison.OrdinalIgnoreCase) >= 0) score += 20f;
                if (name.IndexOf("Snapshot", StringComparison.OrdinalIgnoreCase) >= 0) score -= 500f;
                if (name.IndexOf("Ghost", StringComparison.OrdinalIgnoreCase) >= 0) score -= 500f;
                if (name.IndexOf("TopSurfaceSeatProxy", StringComparison.OrdinalIgnoreCase) >= 0) score -= 1000f;
                if (name.IndexOf("UI", StringComparison.OrdinalIgnoreCase) >= 0) score -= 200f;
                if (score > bestScore) { bestScore = score; best = cam; }
            }
            return best;
        }

        private static bool IsUsableCamera(Camera cam)
        {
            return cam != null && cam.enabled && cam.gameObject != null && cam.gameObject.activeInHierarchy;
        }

        private static bool IsOwnVisual(GameObject go)
        {
            if (go == null) return false;
            Transform t = go.transform;
            while (t != null)
            {
                string n = t.name ?? string.Empty;
                if (n.StartsWith("BedSeatability", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("TopSurfaceSeatProxy", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("DepthSeatProxy", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("DepthBedTop", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("DepthGotoPoint", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("SelectedBedPoint_", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("NearestFloorNavPoint_", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("ScanCenter_", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("ScanUp_", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("PlayerRay_", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("DebugBox_", StringComparison.OrdinalIgnoreCase)) return true;
                if (n.StartsWith("Crosshair_", StringComparison.OrdinalIgnoreCase)) return true;
                t = t.parent;
            }
            return false;
        }

        private static void EnsureMaterials()
        {
            if (_redMat != null) return;
            _redMat = MakeMaterial(new Color(1f, 0.02f, 0.02f, 1f), false);
            _greenMat = MakeMaterial(new Color(0.05f, 1f, 0.05f, 1f), false);
            _cyanMat = MakeMaterial(new Color(0.05f, 1f, 1f, 1f), false);
            _orangeMat = MakeMaterial(new Color(1f, 0.45f, 0.02f, 1f), false);
            _heightfieldMat = MakeMaterial(new Color(0.05f, 0.85f, 1f, 0.24f), true);
            _seatBoxMat = MakeMaterial(new Color(0f, 1f, 1f, 0.24f), true);

            _proxyColliderVisualMat = MakeMaterial(
                new Color(0.00f, 0.72f, 1.00f, 0.50f),
                true);

            _validSeatSurfaceMat = MakeMaterial(
                new Color(0.00f, 0.88f, 0.08f, 0.78f),
                true);

            _invalidSeatSurfaceMat = MakeMaterial(
                new Color(0.96f, 0.00f, 0.04f, 0.76f),
                true);

            _candidatePointMat = MakeMaterial(
                new Color(1f, 0.90f, 0.03f, 1f),
                false);

            _floorReferenceMat = MakeMaterial(
                new Color(0.12f, 0.35f, 1f, 1f),
                false);

            _actionSeatSurfaceMat = MakeMaterial(
                new Color(0.54f, 0.00f, 0.96f, 0.94f),
                true);

            _heightWarningSurfaceMat = MakeMaterial(
                new Color(1.00f, 0.34f, 0.00f, 0.86f),
                true);

            _actionArrowMat = MakeMaterial(
                new Color(1f, 0.90f, 0.03f, 1f),
                false);

            _clearanceCapsuleMat = MakeMaterial(
                new Color(0.08f, 0.45f, 1f, 0.22f),
                true);

            _legSpaceMat = MakeMaterial(
                new Color(1f, 0.12f, 0.62f, 0.24f),
                true);

            // Transparent classification order, rendered from general to specific.
            if (_proxyColliderVisualMat != null)
                _proxyColliderVisualMat.renderQueue = 5100;

            if (_invalidSeatSurfaceMat != null)
                _invalidSeatSurfaceMat.renderQueue = 5200;

            if (_validSeatSurfaceMat != null)
                _validSeatSurfaceMat.renderQueue = 5300;

            if (_heightWarningSurfaceMat != null)
                _heightWarningSurfaceMat.renderQueue = 5400;

            if (_actionSeatSurfaceMat != null)
                _actionSeatSurfaceMat.renderQueue = 5500;
        }

        private static Material MakeMaterial(Color color, bool transparent)
        {
            Shader shader = null;
            try { shader = Shader.Find("Hidden/Internal-Colored"); } catch { }
            if (shader == null) { try { shader = Shader.Find("Unlit/Color"); } catch { } }
            if (shader == null) { try { shader = Shader.Find("Sprites/Default"); } catch { } }
            if (shader == null) { try { shader = Shader.Find("Standard"); } catch { } }
            Material mat = shader != null ? new Material(shader) : new Material(Shader.Find("Diffuse"));
            mat.color = color;
            try { mat.SetColor("_Color", color); } catch { }
            if (transparent)
            {
                try { mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha); } catch { }
                try { mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha); } catch { }
                try { mat.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off); } catch { }
                try { mat.SetInt("_ZWrite", 0); } catch { }
                try { mat.EnableKeyword("_ALPHABLEND_ON"); } catch { }
                mat.renderQueue = 5000;
            }
            else
            {
                try { mat.SetInt("_ZWrite", 1); } catch { }
                mat.renderQueue = 5000;
            }
            return mat;
        }

        private static void CreateSphere(string name, Vector3 pos, float radius, Material mat, Transform parent, bool debugRenderer)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = name;
            go.transform.position = pos;
            go.transform.localScale = Vector3.one * radius;
            if (parent != null) go.transform.SetParent(parent, true);
            SetRendererMaterial(go, mat, debugRenderer);
            DisableCollider(go);
        }

        private static void CreateAxis(string name, Vector3 start, Vector3 direction, float length, Material mat, Transform parent, bool debugRenderer)
        {
            if (direction.sqrMagnitude < 0.0001f) direction = Vector3.up;
            direction.Normalize();
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = name;
            go.transform.position = start + direction * (length * 0.5f);
            go.transform.rotation = Quaternion.FromToRotation(Vector3.up, direction);
            go.transform.localScale = new Vector3(0.025f, length * 0.5f, 0.025f);
            if (parent != null) go.transform.SetParent(parent, true);
            SetRendererMaterial(go, mat, debugRenderer);
            DisableCollider(go);
        }

        private static void SetRendererMaterial(GameObject go, Material mat, bool debugRenderer)
        {
            try
            {
                Renderer r = go.GetComponent<Renderer>();
                if (r != null)
                {
                    r.material = mat;
                    if (debugRenderer) RegisterDebugRenderer(r);
                }
            }
            catch { }
        }

        private static void DisableCollider(GameObject go)
        {
            try
            {
                Collider c = go.GetComponent<Collider>();
                if (c != null) Object.Destroy(c);
            }
            catch { }
        }

        private static void ReleaseRT(RenderTexture rt)
        {
            if (rt == null) return;
            try { if (RenderTexture.active == rt) RenderTexture.active = null; } catch { }
            try { rt.Release(); } catch { }
            DestroyObject(rt);
        }

        private static void DestroyObject(Object obj)
        {
            if (obj == null) return;
            try { Object.Destroy(obj); } catch { }
        }

        private static string Vec(Vector3 v)
        {
            return "(" + v.x.ToString("F3") + ", " + v.y.ToString("F3") + ", " + v.z.ToString("F3") + ")";
        }

        private static void LogInfo(string message)
        {
            try { _log?.LogInfo("[TopSurfaceSeatProxy] " + message); } catch { }
        }

        private static void LogWarning(string message)
        {
            // 主项目整合版只写 BepInEx logger。
            // 不再额外 Debug.LogWarning，否则 BepInEx 控制台会出现一条 Miside AI Modular + 一条 Unity 的重复日志。
            try { _log?.LogWarning("[TopSurfaceSeatProxy] " + message); } catch { }
        }

        private static void LogError(string message)
        {
            try { _log?.LogError("[TopSurfaceSeatProxy] " + message); } catch { }
        }

        private static void TryConsolePrint(string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); } catch { }
        }

        private static Il2CppSystem.Collections.Generic.List<Vector3> ToIl2CppVector3List(List<Vector3> src)
        {
            var dst = new Il2CppSystem.Collections.Generic.List<Vector3>();
            if (src == null) return dst;
            for (int i = 0; i < src.Count; i++) dst.Add(src[i]);
            return dst;
        }

        private static Il2CppSystem.Collections.Generic.List<int> ToIl2CppIntList(List<int> src)
        {
            var dst = new Il2CppSystem.Collections.Generic.List<int>();
            if (src == null) return dst;
            for (int i = 0; i < src.Count; i++) dst.Add(src[i]);
            return dst;
        }
    }
}
