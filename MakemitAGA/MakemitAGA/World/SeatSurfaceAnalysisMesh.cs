/*
 * =================================================================================================
 * SeatSurfaceAnalysisMesh.cs
 * =================================================================================================
 *
 * 作用：生成“环境理解层”的稀疏高度图 MeshCollider。
 *
 * 它负责：
 *   1. 对模型选中的任意 Unity GameObject 做完整高度顶部扫描；
 *   2. 保留沙发靠背、扶手、坐垫等互不连接的高度岛；
 *   3. 计算青/绿/红/橙/紫分类区域；
 *   4. 为 VLM 的 select_2D、最近有效点吸附和物理验证提供查询数据。
 *
 * 重要架构边界：
 *   - 本文件生成的网状面用于“理解和查询”，不是米塔最终坐姿的稳定执行面；
 *   - 最终连续小平面由 World/SeatActionProxy.cs 生成；
 *   - 所有彩色 Renderer 默认隐藏，只有 debug_svt 才会显示；
 *   - Collider 始终保留，因此隐藏调试图形不会破坏物理查询。
 *
 * 历史说明：
 *   本实现从独立 Seat VLM 测试项目的 BedSeatabilityRuntime 演进而来。
 *   文件中仍保留少量旧 Bed 命名和兼容入口，正式流程只调用
 *   RunTargetSeatabilityTest / TrySelectActionPoint / TryFindNearestActionPoint。
 *
 * 拆分后的文件职责：
 *   1. SeatSurfaceAnalysisMesh.cs          —— 公共契约、共享状态、主扫描编排；
 *   2. SeatSurfaceScanCapture.cs           —— 深度相机、完整高度扫描、高度图重建；
 *   3. SeatSurfaceSeatability.cs           —— 承重、边缘、地面、身体与腿部空间判定；
 *   4. SeatSurfaceVisualization.cs         —— 分类网格、代理 MeshCollider 与调试几何；
 *   5. SeatSurfaceNavigation.cs            —— NavMesh/物理落脚点与旧桥接兼容逻辑；
 *   6. SeatSurfaceSceneQuery.cs            —— Bounds、目标识别、Layer 与场景射线查询；
 *   7. SeatSurfaceSelectionLifecycle.cs    —— VLM 选点、吸附、生命周期、材质与通用工具。
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
    internal enum FakeColliderMode { FixedSize, Top }

    internal struct FakeColliderRequest
    {
        public FakeColliderMode mode;
        public float width;
        public float depth;
        public float height;
        public bool clampToSupportedSurface;

        public static FakeColliderRequest DefaultSeat()
        {
            return Fixed(0.50f, 0.50f, 0.08f);
        }

        public static FakeColliderRequest Fixed(float width, float depth, float height)
        {
            return new FakeColliderRequest
            {
                mode = FakeColliderMode.FixedSize,
                width = width,
                depth = depth,
                height = height,
                clampToSupportedSurface = true
            };
        }

        public static FakeColliderRequest Top(float height)
        {
            return new FakeColliderRequest
            {
                mode = FakeColliderMode.Top,
                width = 0f,
                depth = 0f,
                height = height,
                clampToSupportedSurface = true
            };
        }
    }


    internal sealed class SeatSurfaceSelectionResult
    {
        public GameObject Target;
        public Vector2 OriginalViewportTopLeft;
        public Vector2 SelectedViewportTopLeft;
        public Vector3 WorldSeatPoint;
        public Vector3 FloorPoint;
        public Vector3 OutwardDirection;
        public float HeightAboveFloor;
        public bool IsSnapped;
        public float SnapViewportDistance;
        public bool HeightWarning;

        public string ToJson(string selectionType)
        {
            return
                "{" +
                "\"success\":true," +
                "\"selectionType\":\"" + JsonEscape(selectionType) + "\"," +
                "\"target\":\"" + JsonEscape(Target == null ? "" : Target.name) + "\"," +
                "\"targetPath\":\"" + JsonEscape(GetPath(Target == null ? null : Target.transform)) + "\"," +
                "\"originalViewport\":[" + F(OriginalViewportTopLeft.x) + "," + F(OriginalViewportTopLeft.y) + "]," +
                "\"selectedViewport\":[" + F(SelectedViewportTopLeft.x) + "," + F(SelectedViewportTopLeft.y) + "]," +
                "\"worldSeatPoint\":[" + F(WorldSeatPoint.x) + "," + F(WorldSeatPoint.y) + "," + F(WorldSeatPoint.z) + "]," +
                "\"floorPoint\":[" + F(FloorPoint.x) + "," + F(FloorPoint.y) + "," + F(FloorPoint.z) + "]," +
                "\"outwardDirection\":[" + F(OutwardDirection.x) + "," + F(OutwardDirection.y) + "," + F(OutwardDirection.z) + "]," +
                "\"heightAboveFloor\":" + F(HeightAboveFloor) + "," +
                "\"isSnapped\":" + (IsSnapped ? "true" : "false") + "," +
                "\"snapViewportDistance\":" + F(SnapViewportDistance) + "," +
                "\"heightWarning\":" + (HeightWarning ? "true" : "false") +
                "}";
        }

        private static string F(float value)
        {
            return value.ToString(
                "0.####",
                System.Globalization.CultureInfo.InvariantCulture);
        }

        private static string JsonEscape(string value)
        {
            return (value ?? "")
                .Replace("\\", "\\\\")
                .Replace("\"", "\\\"")
                .Replace("\r", "\\r")
                .Replace("\n", "\\n");
        }

        private static string GetPath(Transform transform)
        {
            if (transform == null)
                return "";

            string path = transform.name;

            for (Transform parent = transform.parent;
                 parent != null;
                 parent = parent.parent)
            {
                path = parent.name + "/" + path;
            }

            return path;
        }
    }

    internal static partial class SeatSurfaceAnalysisRuntime
    {
        private static ManualLogSource _log;

        private static bool _firstTickLogged;
        private static int _tickCount;

        private static GameObject _crosshairRoot;
        private static Camera _crosshairCamera;

        private static readonly List<GameObject> _created = new List<GameObject>();
        private static readonly List<Renderer> _debugRenderers = new List<Renderer>();

        private static Material _depthToEyeMat;
        private static Shader _eyeDepthReplacementShader;
        private static Material _heightfieldMat;
        private static Material _seatBoxMat;
        private static Material _redMat;
        private static Material _greenMat;
        private static Material _cyanMat;
        private static Material _orangeMat;

        // Bed seatability test visualization materials.
        private static Material _proxyColliderVisualMat;
        private static Material _validSeatSurfaceMat;
        private static Material _invalidSeatSurfaceMat;
        private static Material _candidatePointMat;
        private static Material _floorReferenceMat;
        private static Material _actionSeatSurfaceMat;
        private static Material _heightWarningSurfaceMat;
        private static Material _actionArrowMat;
        private static Material _clearanceCapsuleMat;
        private static Material _legSpaceMat;

        private static GameObject _actionMarkerRoot;
        private static GameObject _clearanceDebugRoot;
        private static GameObject _scanVolumeDebugRoot;

        private static Transform _lastResultRoot;
        private static GameObject _lastTargetObject;
        private static Bounds _lastScanVolume;
        private static bool _hasLastScanVolume;

        private static bool _actionMarkersVisible = false;
        private static bool _clearanceDebugVisible = false;
        private static bool _scanVolumeVisible = false;
        private static bool _debugRenderersVisible = false;

        private static int _scanSerial;
        private static bool _scanInProgress;
        private static string _scanStage = "Idle";

        // Last completed seat-surface data used by the VLM select_2D stage.
        private static HeightfieldStats _lastSelectionHeightfield;
        private static SeatabilityStats _lastSelectionSeatability;
        private static GameObject _lastProxyColliderObject;
        private static bool _lastSelectionDataValid;

        private const float MaxSnapViewportDistance = 0.28f;

        private const string BundleFileName = "mita_actions";
        private const string MaterialName = "DepthToEye_Mat";
        private const string ShaderName = "Hidden/DepthSeat/DepthToEye";
        private const string EyeDepthReplacementShaderAssetName = "DepthSeat_EyeDepthReplacement";
        private const string EyeDepthReplacementShaderFindName = "Hidden/DepthSeat/EyeDepthReplacement";

        private const int CaptureSize = 256;
        private const int MeshGrid = 64;

        // Integration/performance settings.
        // General checks are cheap; action checks include physics/NavMesh and use a smaller batch.
        private const int GeneralAnalysisCellsPerFrame = 256;
        private const int ActionAnalysisCellsPerFrame = 48;

        // Developer marker spheres/capsules are no longer generated in the clean pipeline.
        private const bool GenerateDeveloperActionMarkers = false;
        private const int DefaultScanLayer = 30;

        private const float FixedScanPadding = 0.18f;
        private const float FixedScanMinSize = 0.62f;
        private const float FixedScanMaxSize = 1.50f;
        private const float TopScanPadding = 0.10f;
        private const float TopScanMaxSize = 5.00f;

        private const float ScanBoxBelowHit = 0.28f;
        private const float ScanBoxAboveHitNoBounds = 0.55f;
        private const float ScanBoxAboveHitWithBounds = 0.16f;
        private const float ScanBoxRendererTopMargin = 0.08f;
        private const float ScanBoxTargetTopExtra = 0.18f;
        private const float ScanBoxMaxHeight = 0.70f;

        // Generic furniture scan:
        // Top mode now covers the selected object's complete rough vertical extent.
        // Renderer bounds define the visual X/Z footprint; non-trigger Collider bounds
        // may expand the Y range when the collision proxy extends lower/higher.
        private const float TopScanBottomPadding = 0.08f;
        private const float TopScanTopPadding = 0.12f;
        private const float TopScanMaxVerticalHeight = 2.50f;

        // Preserve multiple disconnected top levels such as sofa back + arm + cushion.
        // The old fixed 0.22m global median filter was appropriate for a bed but removed
        // legitimate lower sofa cushions when the backrest was the highest surface.
        private const float GenericHeightDeviationVolumeFactor = 0.90f;
        private const float GenericMaxHeightDeviation = 1.35f;

        private const float CameraDistance = 1.50f;
        private const float MinOrthoSize = 0.35f;
        private const float OrthoPadding = 0.04f;
        private const float VisualLift = 0.025f;

        private const int HeightFillIterations = 4;
        private const float MaxHeightDeviationFromMedian = 0.22f;
        private const float HeightSmoothStrength = 0.35f;
        private const int SurfaceMaskDilateIterations = 1;
        private const int MinSurfaceIslandCells = 8;
        private const float MaxTriangleHeightDelta = 0.20f;

        private const string SeatProxyName = "DepthSeatProxy_Bed";
        private const float DefaultSeatProxyWidth = 0.50f;
        private const float DefaultSeatProxyDepth = 0.50f;
        private const float DefaultSeatProxyHeight = 0.08f;
        private const float MinFakeColliderSupportCoverage = 0.55f;
        private const float MinFakeColliderSize = 0.20f;
        private const bool SeatProxyRendererVisibleByDefault = false;

        private const float CrosshairDepth = 0.55f;
        private const float RayDistance = 30.0f;

        // ---------------------------------------------------------------------------------
        // Bed-only seatability test parameters.
        //
        // These values describe the minimum support patch around a candidate seat center.
        // They do NOT create a fixed seat box. They only classify the top-surface heightfield.
        // ---------------------------------------------------------------------------------
        private const float SeatTestWidth = 0.42f;
        private const float SeatTestDepth = 0.32f;
        private const float SeatMinSupportCoverage = 0.80f;
        private const float SeatMaxPatchHeightRange = 0.105f;
        private const float SeatMinNormalY = 0.62f;

        // 0.60m 是理想高度；0.60~0.63m 仍允许，但用橙色标记并降低动作候选优先级。
        private const float SeatIdealHeightAboveFloor = 0.60f;
        private const float SeatHardMaxHeightAboveFloor = 0.63f;

        // “床边坐姿动作有效区”参数。
        // 一个点不仅要能承重，还要靠近代理边界，并且边界外侧存在可站立地板/NavMesh。
        private const float ActionMinEdgeInset = 0.07f;
        private const float ActionMaxEdgeInset = 0.34f;
        private const float ActionOutsideStandDistance = 0.48f;
        private const float ActionFloorProbeHeight = 1.20f;
        private const float ActionFloorProbeDistance = 4.00f;
        private const float ActionNavMeshRadius = 0.42f;
        private const float ActionMaxNavMeshSnapDistance = 0.24f;
        private const float ActionMaxNavMeshVerticalOffset = 0.15f;

        private const float ActionMinSeatFloorDrop = 0.16f;
        private const float ActionMaxSeatFloorDrop = 0.72f;

        // Mita standing body clearance at the outside floor point.
        private const float ActionBodyRadius = 0.23f;
        private const float ActionBodyHeight = 1.55f;
        private const float ActionBodyBottomClearance = 0.08f;

        // Short approach corridor from the selected NavMesh point toward the bed edge.
        private const int ActionCorridorSamples = 4;
        private const float ActionCorridorNearEdgeClearance = 0.20f;

        // Free volume required for legs in front of the seated character.
        private const float ActionLegSpaceWidth = 0.46f;
        private const float ActionLegSpaceDepth = 0.48f;
        private const float ActionLegSpaceHeight = 0.65f;
        private const float ActionLegSpaceBottomClearance = 0.08f;

        // Low colliders whose highest point remains close to the floor are treated as floor/rug.
        private const float ActionFloorLikeColliderTolerance = 0.14f;

        private const float SeatOverlayLift = 0.050f;
        private const float InvalidOverlayLift = 0.038f;
        private const float HeightWarningOverlayLift = 0.060f;
        private const float ActionOverlayLift = 0.072f;
        private const float ProxyVisualExtraLift = 0.010f;

        private const float CandidateMarkerSpacing = 0.40f;
        private const int MaxCandidateMarkers = 12;
        private const bool ShowScanVolumeByDefault = false;
        private const bool ShowActionMarkersByDefault = false;
        private const bool ShowClearanceDebugByDefault = false;

        private const string HardcodedBedName = "Bed";
        private const string BedTopMeshColliderName = "DepthBedTopMeshCollider";
        private const string BedGotoPointName = "DepthGotoPoint_Bed";
        private const float BedSeatProxyWidth = 0.50f;
        private const float BedSeatProxyDepth = 0.50f;
        private const float BedSeatProxyHeight = 0.08f;
        private const float BedGotoCubeSize = 0.12f;
        private const float BedGotoArriveDistance = 0.75f;
        private const float BedGotoTimeout = 18.0f;
        // v0.9.1 不再周期性重复 AiWalkToTarget。
        // v0.8.1 中反复 repath 会打断/重启动画队列，表现为原生抬手动作抽搐。
        private const float BedGotoRepathInterval = 9999.0f;
        private const bool F3AutoCallMitaSit = true;

        private static bool _pendingBedTopSit;
        private static float _pendingBedTopSitDeadline;
        private static float _pendingBedTopSitNextRepathTime;
        private static MitaPerson _pendingBedTopSitMita;
        private static Transform _pendingBedTopSitGoto;
        private static string _pendingBedTopSitSeatProxyName;
        private static float _pendingBedTopSitLastLogTime;

        // 避免在 MakemitAGA 未加载时，每次测试都刷屏。
        private static bool _mitaSitMissingWarned;

        // v0.9.1:
        // 第一次 F3 时同帧 Destroy 旧代理、创建新代理、立刻 GameObject.Find 容易被 Mita_sit 找到旧对象或未同步 collider。
        // 所以每次生成唯一代理名，并延迟 2 帧再桥接调用 Mita_sit.Sit(uniqueName)。
        private static int _seatProxySerial;
        private static bool _pendingBridgeSit;
        private static int _pendingBridgeFrame;
        private static string _pendingBridgeTargetName;

        private struct LayerBackup { public GameObject go; public int layer; }
        private enum DepthPath { None, EyeDepthReplacement, DepthToEyeMaterial, BuiltInDepthNormalsFallback }

        private struct DepthPoint
        {
            public bool valid;
            public Vector3 world;
            public float eyeDepth;
            public int px;
            public int py;
        }

        private struct HeightfieldStats
        {
            public bool valid;
            public int rawPixelValidCount;
            public int rawCellValidCount;
            public int keptCellCount;
            public int surfaceCellCount;
            public int removedOutliers;
            public float medianY;
            public float minY;
            public float maxY;
            public Vector3 center;
            public Bounds volume;
            public bool[,] surfaceMask;
            public float[,] heights;
        }


        private enum ActionRejectReason
        {
            None,
            Floor,
            NavMesh,
            NavMeshOffset,
            Drop,
            BodyClearance,
            CorridorClearance,
            LegClearance
        }

        private sealed class SeatabilityStats
        {
            // General physical support validity.
            public bool[,] validSeatCenters;

            // Cells in the soft height band: > 0.60m and <= 0.63m.
            public bool[,] heightWarningCenters;

            // Cells suitable specifically for the current edge-sit action.
            public bool[,] actionValidCenters;
            public Vector3[,] actionFloorPoints;
            public Vector3[,] actionOutwardDirections;
            public float[,] actionEdgeDistances;

            public int proxySurfaceCells;
            public int validSeatCells;
            public int heightWarningCells;
            public int actionValidCells;

            public int rejectedSlope;
            public int rejectedCoverage;
            public int rejectedFlatness;
            public int rejectedHeight;

            public int rejectedActionEdgeDistance;
            public int rejectedActionFloor;
            public int rejectedActionNavMesh;
            public int rejectedActionNavMeshOffset;
            public int rejectedActionDrop;
            public int rejectedActionBodyClearance;
            public int rejectedActionCorridorClearance;
            public int rejectedActionLegClearance;

            public bool floorFound;
            public float floorY;
            public float cellSizeX;
            public float cellSizeZ;
        }

        private struct SeatCandidatePoint
        {
            public Vector3 world;
            public Vector3 floorWorld;
            public Vector3 outward;
            public float edgeDistance;
            public float score;
        }

        public static void Init(ManualLogSource log) { _log = log; }

        public static void TickFromPatch()
        {
            _tickCount++;

            if (!_firstTickLogged)
            {
                _firstTickLogged = true;
                LogInfo("First GameController.Update tick received. Hotkeys disabled; console commands only.");
            }

            try
            {
                EnsureMaterials();
                EnsureCrosshair();

                // v0.9.1:
                // 主项目整合版不再监听 F3/F9/Delete，也不在 Update 中自动扫描。
                // 扫描只允许由控制台指令触发：
                //   ts_scan()
                //   ts_bed_sit
                //   ts_scan_top
                //
                // 之前 v0.9.0 在删除 hotkey if 语句时，误留下了函数体，
                // 导致每帧都执行扫描，BepInEx 控制台刷屏并严重卡顿。
                TickPendingBedSit();
                TickPendingBridgeSit();
            }
            catch (Exception e)
            {
                LogError("Tick exception: " + e);
            }
        }

        /// <summary>
        /// Compatibility wrapper used by the current TAB test.
        /// Future VLM integration should call RunTargetSeatabilityTest(selectedObject, source)
        /// directly after select_object().
        /// </summary>
        public static void RunBedSeatabilityTest(string source)
        {
            GameObject bed = FindHardcodedBedObject();

            if (bed == null)
            {
                LogError("[SeatSurface] Cannot find exact GameObject named Bed.");
                TryConsolePrint("<color=red>SeatSurface：没有找到名为 Bed 的目标物体。</color>");
                return;
            }

            RunTargetSeatabilityTest(
                bed,
                source + "-bed-wrapper");
        }

        /// <summary>
        /// Generic entry point for any selected Unity GameObject.
        ///
        /// The current algorithm evaluates an upward-facing top surface and an edge-sit action.
        /// It is suitable for beds, benches, stools, many chairs and sofas, but the caller must
        /// still reject obviously non-seat semantic targets before starting this pipeline.
        /// </summary>
        public static void RunTargetSeatabilityTest(
            GameObject target,
            string source)
        {
            if (target == null)
            {
                LogError("[SeatSurface] target is null.");
                return;
            }

            if (Plugin.Runner == null)
            {
                LogError("[SeatSurface] Plugin.Runner is null.");
                return;
            }

            _scanSerial++;
            int serial = _scanSerial;

            _scanInProgress = true;
            _scanStage = "Queued";

            Plugin.Runner.StartCoroutine(
                ScanTargetRoutine(
                    target,
                    source,
                    serial)
                .WrapToIl2Cpp());
        }

        public static string GetPipelineStatus()
        {
            return
                "scanInProgress=" + _scanInProgress +
                " | scanStage=" + _scanStage +
                " | scanSerial=" + _scanSerial +
                " | preview=" +
                GetCompositePreviewSourceStatus();
        }

        private static IEnumerator ScanTargetRoutine(
            GameObject target,
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
                _scanInProgress = false;
                _scanStage = "Failed-NoTargetBounds";

                LogError(
                    "[SeatSurface] Target has no usable Renderer/Collider bounds: " +
                    target.name);

                TryConsolePrint(
                    "<color=red>SeatSurface：目标没有可用 Renderer/Collider Bounds。</color>");

                yield break;
            }

            // Cancel and remove the previous result before storing the new target.
            ClearProxyOnly();

            if (serial != _scanSerial)
                yield break;

            _lastTargetObject = target; // Compatibility field; now stores the generic selected target.
            _scanStage = "Preparing";

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
                FakeColliderRequest.Top(
                    DefaultSeatProxyHeight));

            int scanLayer = PickScanLayer();

            LogInfo(
                "[SeatSurface] Pipeline start" +
                " | source=" + source +
                " | target=" + target.name +
                " | path=" + GetTransformPath(target.transform) +
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
                " | scanVolume=" + Vec(volume.center) +
                " / " + Vec(volume.size));

            // Let the triggering input frame finish before GPU capture.
            yield return null;

            List<LayerBackup> layerBackups =
                new List<LayerBackup>();

            GameObject scanCameraObject = null;
            Camera scanCamera = null;

            try
            {
                if (serial != _scanSerial)
                    yield break;

                _scanStage = "GpuCapture";

                AssignLayerRecursive(
                    target,
                    scanLayer,
                    layerBackups);

                scanCameraObject =
                    new GameObject(
                        "SeatSurface_ScanCamera");

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

                if (!captured ||
                    !heightfield.valid ||
                    vertices.Count < 3 ||
                    triangles.Count < 3)
                {
                    _scanStage = "Failed-Capture";

                    LogError(
                        "[SeatSurface] Top scan failed." +
                        " depthPath=" + depthPath +
                        " vertices=" + vertices.Count +
                        " triangles=" +
                        (triangles.Count / 3));

                    yield break;
                }

                // Restore the target's original layers immediately after capture.
                RestoreLayers(layerBackups);
                layerBackups.Clear();

                yield return null;

                if (serial != _scanSerial)
                    yield break;

                _scanStage = "BuildingProxy";

                GameObject root =
                    new GameObject(
                        "SeatSurface_ResultRoot");

                _created.Add(root);

                GameObject proxyVisual =
                    BuildHeightfieldMesh(
                        root.transform,
                        vertices,
                        triangles);

                if (proxyVisual != null)
                {
                    proxyVisual.name =
                        "SeatSurface_ProxyColliderVisual_Cyan";

                    MeshRenderer renderer =
                        proxyVisual.GetComponent<MeshRenderer>();

                    if (renderer != null)
                    {
                        renderer.material =
                            _proxyColliderVisualMat;

                        RegisterDebugRenderer(renderer);
                    }

                    proxyVisual.transform.position +=
                        Vector3.up *
                        ProxyVisualExtraLift;
                }

                GameObject proxyCollider =
                    BuildTopMeshColliderFromStats(
                        root.transform,
                        heightfield);

                if (proxyCollider != null)
                {
                    proxyCollider.name =
                        "SeatSurface_ProxyMeshCollider";
                }

                _lastProxyColliderObject = proxyCollider;

                yield return null;

                if (serial != _scanSerial)
                    yield break;

                _scanStage = "GeneralAnalysis";

                SeatabilityStats seatability =
                    CreateSeatabilityStats(heightfield);

                IEnumerator analysis =
                    AnalyzeSeatabilityBatched(
                        target,
                        targetBounds,
                        heightfield,
                        seatability,
                        serial);

                while (analysis.MoveNext())
                    yield return analysis.Current;

                if (serial != _scanSerial)
                    yield break;

                _scanStage = "BuildingOverlays";

                BuildSeatabilityOverlay(
                    root.transform,
                    heightfield,
                    seatability.validSeatCenters,
                    true,
                    "SeatSurface_ValidSupport_Green",
                    _validSeatSurfaceMat,
                    SeatOverlayLift);

                yield return null;

                BuildSeatabilityOverlay(
                    root.transform,
                    heightfield,
                    seatability.validSeatCenters,
                    false,
                    "SeatSurface_InvalidSupport_Red",
                    _invalidSeatSurfaceMat,
                    InvalidOverlayLift);

                yield return null;

                BuildSeatabilityOverlay(
                    root.transform,
                    heightfield,
                    seatability.heightWarningCenters,
                    true,
                    "SeatSurface_HeightWarning_Orange",
                    _heightWarningSurfaceMat,
                    HeightWarningOverlayLift);

                yield return null;

                BuildSeatabilityOverlay(
                    root.transform,
                    heightfield,
                    seatability.actionValidCenters,
                    true,
                    "SeatSurface_ActionValid_Purple",
                    _actionSeatSurfaceMat,
                    ActionOverlayLift);

                // Clean integration build:
                // no yellow candidate spheres, long arrows, floor spheres,
                // debug scan boxes, standing capsules or leg-space boxes.
                int markerCount = 0;

                _lastResultRoot = root.transform;
                _lastTargetObject = target;
                _lastScanVolume = volume;

                _lastSelectionHeightfield = heightfield;
                _lastSelectionSeatability = seatability;
                _lastSelectionDataValid =
                    proxyCollider != null &&
                    seatability != null &&
                    seatability.actionValidCenters != null;
                _hasLastScanVolume = true;
                _scanVolumeDebugRoot = null;
                _scanVolumeVisible = false;

                _scanStage = "Completed";
                _scanInProgress = false;

                string message =
                    "SeatSurface completed" +
                    " | target=" + target.name +
                    " | depthPath=" + depthPath +
                    " | proxyCells=" +
                    seatability.proxySurfaceCells +
                    " | supportValid=" +
                    seatability.validSeatCells +
                    " | softHeight=" +
                    seatability.heightWarningCells +
                    " | actionValid=" +
                    seatability.actionValidCells +
                    " | actionMarkers=" + markerCount +
                    " | floorFound=" +
                    seatability.floorFound +
                    " | floorY=" +
                    seatability.floorY.ToString("F3") +
                    " | rejectSlope=" +
                    seatability.rejectedSlope +
                    " | rejectCoverage=" +
                    seatability.rejectedCoverage +
                    " | rejectFlatness=" +
                    seatability.rejectedFlatness +
                    " | rejectHeight=" +
                    seatability.rejectedHeight +
                    " | rejectActionEdge=" +
                    seatability.rejectedActionEdgeDistance +
                    " | rejectActionFloor=" +
                    seatability.rejectedActionFloor +
                    " | rejectActionNav=" +
                    seatability.rejectedActionNavMesh +
                    " | rejectActionNavOffset=" +
                    seatability.rejectedActionNavMeshOffset +
                    " | rejectActionDrop=" +
                    seatability.rejectedActionDrop +
                    " | rejectActionBody=" +
                    seatability.rejectedActionBodyClearance +
                    " | rejectActionCorridor=" +
                    seatability.rejectedActionCorridorClearance +
                    " | rejectActionLeg=" +
                    seatability.rejectedActionLegClearance;

                LogInfo(message);
                LogInfo(
                    "[CompositePreviewSource] " +
                    GetCompositePreviewSourceStatus());

                TryConsolePrint(
                    "<color=green>SeatSurface 计算完成。</color> " +
                    message);
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

                if (serial == _scanSerial &&
                    _scanStage != "Completed")
                {
                    _scanInProgress = false;
                }
            }
        }

    }
}
