/*
 * =================================================================================================
 * TopSurfaceSeatProxyRuntime.cs
 * =================================================================================================
 *
 * 核心运行时文件。
 *
 * 当前功能：
 *   ts_scan()：扫描准星目标小区域，生成默认 0.5×0.5×0.08 的 DepthSeatProxy_Bed。
 *   ts_bed_sit：硬编码寻找名为 Bed 的物体，玩家准星需要指向 Bed 表面上的目标点；
 *        扫描完整 Bed top surface，生成 DepthBedTopMeshCollider；
 *        在玩家选中的床面点生成 DepthSeatProxy_Bed_xxx；
 *        在离该床面点最近的可达 NavMesh 地面点生成隐藏 cube DepthGotoPoint_Bed；
 *        延迟 2 帧后调用 Mita_sit.Sit("DepthSeatProxy_Bed_xxx")。
 *
 * 为什么这样写：
 *   1. 旧的 _CameraDepthTexture 路径在隐藏相机里容易读到主相机/上一相机残留深度。
 *      所以主路径改为 scanCam.RenderWithShader(EyeDepthReplacement, "")，直接输出扫描相机自己的 eye depth。
 *   2. heightfield 用 surfaceMask 区分真实表面和空区域，避免床外空区域被补成大平面。
 *   3. 可视化 mesh 加 VisualLift 防止 z-fighting；真正 MeshCollider 不加 VisualLift，否则碰撞面会悬空。
 *   4. goto() 只能粗糙走向物体，所以 F3 测试创建隐藏小 cube 作为精确导航目标。
 *
 * 未来设计：
 *   - 当前 Top MeshCollider 是 top-view 2.5D 表面，适合躺下、爬床、IK 接触等。
 *   - 真正 whole 物体需要多视角体素/TSDF/marching cubes 或多 collider tiling。
 *   - 坐下仍建议保留 0.5×0.5×0.08 BoxCollider，因为 Mita_sit 更需要稳定规则座面。
 *
 * =================================================================================================
 */

using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using BepInEx;
using BepInEx.Logging;
using HarmonyLib;
using UnityEngine;
using UnityEngine.AI;
using Object = UnityEngine.Object;

namespace MakemitAGA.Mita_self.Mita_tools
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

    internal static class TopSurfaceSeatProxyRuntime
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

        private const string BundleFileName = "mita_actions";
        private const string MaterialName = "DepthToEye_Mat";
        private const string ShaderName = "Hidden/DepthSeat/DepthToEye";
        private const string EyeDepthReplacementShaderAssetName = "DepthSeat_EyeDepthReplacement";
        private const string EyeDepthReplacementShaderFindName = "Hidden/DepthSeat/EyeDepthReplacement";

        private const int CaptureSize = 256;
        private const int MeshGrid = 64;
        private const int DefaultScanLayer = 30;

        private const float FixedScanPadding = 0.18f;
        private const float FixedScanMinSize = 0.62f;
        private const float FixedScanMaxSize = 1.50f;
        private const float TopScanPadding = 0.10f;
        private const float TopScanMaxSize = 4.00f;

        private const float ScanBoxBelowHit = 0.28f;
        private const float ScanBoxAboveHitNoBounds = 0.55f;
        private const float ScanBoxAboveHitWithBounds = 0.16f;
        private const float ScanBoxRendererTopMargin = 0.08f;
        private const float ScanBoxTargetTopExtra = 0.18f;
        private const float ScanBoxMaxHeight = 0.70f;

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
        private const bool SeatProxyRendererVisibleByDefault = true;

        private const float CrosshairDepth = 0.55f;
        private const float RayDistance = 30.0f;

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

        public static void Init(ManualLogSource log) { _log = log; }

        public static void TickFromPatch()
        {
            _tickCount++;

            if (!_firstTickLogged)
            {
                _firstTickLogged = true;
                LogWarning("First GameController.Update tick received. Hotkeys disabled; console commands only.");
            }

            try
            {
                EnsureMaterials();
                //EnsureCrosshair();
                //红色准星

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

        public static void ScanFromScreenCenter(string source, FakeColliderRequest request)
        {
            EnsureMaterials();
            TryLoadDepthAssets();

            Camera playerCam = FindBestCamera();
            if (playerCam == null) { LogError("No usable camera found."); return; }

            Ray ray = playerCam.ScreenPointToRay(new Vector3(Screen.width * 0.5f, Screen.height * 0.5f, 0f));
            RaycastHit hit;
            if (!TryRaycastScene(ray, out hit))
            {
                LogWarning("Screen center raycast missed.");
                TryConsolePrint("TopSurfaceSeatProxy: raycast missed.");
                return;
            }

            GameObject hitObject = hit.collider != null ? hit.collider.gameObject : null;
            GameObject targetRoot = GuessScanRoot(hitObject, hit.point);
            if (targetRoot == null) { LogError("Cannot determine target root."); return; }

            ClearProxyOnly();

            Bounds targetBounds;
            bool hasTargetBounds = TryGetRendererBounds(targetRoot, out targetBounds);
            Bounds volume = BuildScanVolume(hit.point, hasTargetBounds, targetBounds, request);
            int scanLayer = PickScanLayer();

            LogWarning("Scan setup: target=" + targetRoot.name +
                       ", hit=" + Vec(hit.point) +
                       ", volumeCenter=" + Vec(volume.center) +
                       ", volumeSize=" + Vec(volume.size) +
                       ", mode=" + request.mode);

            List<LayerBackup> backups = new List<LayerBackup>();
            GameObject scanCameraObj = null;
            Camera scanCam = null;

            try
            {
                AssignLayerRecursive(targetRoot, scanLayer, backups);
                scanCameraObj = new GameObject("TopSurfaceSeatProxy_ScanCamera");
                scanCameraObj.hideFlags = HideFlags.HideAndDontSave;
                scanCam = scanCameraObj.AddComponent<Camera>();
                ConfigureScanCamera(scanCam, volume, scanLayer);

                List<Vector3> vertices = new List<Vector3>();
                List<int> triangles = new List<int>();
                HeightfieldStats stats;
                DepthPath depthPath;
                bool ok = CaptureTopViewToHeightfield(scanCam, volume, vertices, triangles, out stats, out depthPath);

                if (!ok || vertices.Count < 3 || triangles.Count < 3 || !stats.valid)
                {
                    LogError("Top surface scan failed. vertices=" + vertices.Count + ", triangles=" + (triangles.Count / 3) + ", depthPath=" + depthPath);
                    return;
                }

                GameObject root = new GameObject("TopSurfaceSeatProxy_ResultRoot");
                _created.Add(root);

                GameObject meshObj = BuildHeightfieldMesh(root.transform, vertices, triangles);
                CreateDebugBox(volume, root.transform);
                CreateSphere("ScanCenter_Red", stats.center + Vector3.up * 0.06f, 0.055f, _redMat, root.transform, true);
                CreateAxis("ScanUp_Green", stats.center + Vector3.up * 0.02f, Vector3.up, 0.60f, _greenMat, root.transform, true);
                CreateAxis("PlayerRay_Orange", hit.point, ray.direction, 0.70f, _orangeMat, root.transform, true);

                GameObject fakeCollider = FakeCollider(root.transform, stats, request);

                string msg = "Top surface scan done from " + source +
                             " | target=" + targetRoot.name +
                             " | depthPath=" + depthPath +
                             " | colliderMode=" + request.mode +
                             " | seatY=" + stats.medianY.ToString("F3") +
                             " | surfaceCells=" + stats.surfaceCellCount +
                             " | mesh=" + (meshObj != null ? meshObj.name : "<null>") +
                             " | proxy=" + (fakeCollider != null ? fakeCollider.name : "<none>");
                LogWarning(msg);
                TryConsolePrint(msg);
            }
            catch (Exception e)
            {
                LogError("ScanFromScreenCenter exception: " + e);
            }
            finally
            {
                RestoreLayers(backups);
                try { if (scanCam != null) scanCam.targetTexture = null; } catch { }
                DestroyObject(scanCameraObj);
            }
        }

        public static void ScanHardcodedBedTopMeshAndSit(string source)
        {
            EnsureMaterials();
            TryLoadDepthAssets();

            Camera playerCam = FindBestCamera();
            if (playerCam == null) { LogError("ts_bed_sit Bed test failed: no usable camera."); return; }

            GameObject bed = FindHardcodedBedObject();
            if (bed == null)
            {
                LogError("ts_bed_sit Bed test failed: cannot find GameObject named Bed.");
                TryConsolePrint("<color=red>没有找到名为 Bed 的物体。</color>");
                return;
            }

            Bounds bedBounds;
            if (!TryGetRendererBounds(bed, out bedBounds))
            {
                LogError("ts_bed_sit Bed test failed: Bed has no renderer bounds.");
                return;
            }

            Ray ray = playerCam.ScreenPointToRay(new Vector3(Screen.width * 0.5f, Screen.height * 0.5f, 0f));
            RaycastHit bedHit;
            if (!TryRaycastSpecificTarget(ray, bed, out bedHit))
            {
                LogWarning("ts_bed_sit Bed test: ray did not hit Bed. Please aim at the bed surface.");
                TryConsolePrint("<color=yellow>ts_bed_sit 需要准星指向 Bed 表面。</color>");
                return;
            }

            Vector3 selectedPointFromRay = bedHit.point;
            ClearProxyOnly();

            int scanLayer = PickScanLayer();
            Bounds volume = BuildScanVolume(selectedPointFromRay, true, bedBounds, FakeColliderRequest.Top(BedSeatProxyHeight));

            LogWarning("ts_bed_sit Bed test setup: bed=" + bed.name +
                       ", selectedRayPoint=" + Vec(selectedPointFromRay) +
                       ", bedBounds=" + Vec(bedBounds.center) + " / " + Vec(bedBounds.size) +
                       ", volume=" + Vec(volume.center) + " / " + Vec(volume.size));

            List<LayerBackup> backups = new List<LayerBackup>();
            GameObject scanCameraObj = null;
            Camera scanCam = null;

            try
            {
                AssignLayerRecursive(bed, scanLayer, backups);
                scanCameraObj = new GameObject("TopSurfaceSeatProxy_ScanCamera");
                scanCameraObj.hideFlags = HideFlags.HideAndDontSave;
                scanCam = scanCameraObj.AddComponent<Camera>();
                ConfigureScanCamera(scanCam, volume, scanLayer);

                List<Vector3> vertices = new List<Vector3>();
                List<int> triangles = new List<int>();
                HeightfieldStats stats;
                DepthPath depthPath;

                bool ok = CaptureTopViewToHeightfield(scanCam, volume, vertices, triangles, out stats, out depthPath);
                if (!ok || vertices.Count < 3 || triangles.Count < 3 || !stats.valid)
                {
                    LogError("ts_bed_sit Bed test failed: scan failed. vertices=" + vertices.Count + ", triangles=" + (triangles.Count / 3));
                    TryConsolePrint("<color=red>Bed Top 扫描失败。</color>");
                    return;
                }

                GameObject root = new GameObject("TopSurfaceSeatProxy_BedTopSitRoot");
                _created.Add(root);

                GameObject visualMeshObj = BuildHeightfieldMesh(root.transform, vertices, triangles);
                GameObject bedTopCollider = BuildTopMeshColliderFromStats(root.transform, stats);
                CreateDebugBox(volume, root.transform);
                CreateSphere("SelectedBedPoint_Red", selectedPointFromRay + Vector3.up * 0.06f, 0.045f, _redMat, root.transform, true);

                Vector3 selectedSurfacePoint;
                if (!TryGetSurfacePointAtXZ(stats, selectedPointFromRay.x, selectedPointFromRay.z, out selectedSurfacePoint))
                {
                    selectedSurfacePoint = stats.center;
                    LogWarning("ts_bed_sit Bed test: selected point not inside surfaceMask; fallback to stats.center=" + Vec(selectedSurfacePoint));
                }

                Vector3 navPoint;
                bool hasNavPoint = TryFindNearestReachableNavPointNearSurface(selectedSurfacePoint, bedBounds, out navPoint);

                // v0.9.1 关键修复：
                // Mita_sit.BuildSeatProbe 使用 target.forward 判断“座椅/床沿正面”。
                // 旧版 proxy.rotation = identity，forward 固定世界 +Z，导致坐下方向平行于床边。
                // 这里把 proxy.forward 设置为“床面点 -> 最近可达地面点”的方向，让 Mita_sit 的
                // ApproachPosition 和 DockRotation 都垂直于床边。
                Vector3 seatForward = hasNavPoint ? FlattenXZ(navPoint - selectedSurfacePoint) : EstimateOutwardForwardFromBounds(selectedSurfacePoint, bedBounds);
                if (seatForward.sqrMagnitude < 0.0001f) seatForward = EstimateOutwardForwardFromBounds(selectedSurfacePoint, bedBounds);
                if (seatForward.sqrMagnitude < 0.0001f) seatForward = Vector3.forward;
                seatForward.Normalize();

                Quaternion seatRotation = Quaternion.LookRotation(seatForward, Vector3.up);
                string seatProxyName = SeatProxyName + "_" + (++_seatProxySerial);
                GameObject seatProxy = CreateSeatProxyAtSurfacePoint(root.transform, selectedSurfacePoint, BedSeatProxyWidth, BedSeatProxyDepth, BedSeatProxyHeight, seatRotation, seatProxyName);

                GameObject gotoPoint = null;
                if (hasNavPoint)
                {
                    gotoPoint = CreateHiddenGotoPoint(root.transform, navPoint);
                    CreateSphere("NearestFloorNavPoint_Red", navPoint + Vector3.up * 0.08f, 0.045f, _redMat, root.transform, true);
                    CreateAxis("SeatProxyForward_Green", selectedSurfacePoint + Vector3.up * 0.10f, seatForward, 0.55f, _greenMat, root.transform, true);
                }

                string msg = "ts_bed_sit unique-proxy delayed-bridge test done from " + source +
                             " | depthPath=" + depthPath +
                             " | selectedSurface=" + Vec(selectedSurfacePoint) +
                             " | navPoint=" + (hasNavPoint ? Vec(navPoint) : "<none>") +
                             " | visualMesh=" + (visualMeshObj != null ? visualMeshObj.name : "<null>") +
                             " | meshCollider=" + (bedTopCollider != null ? bedTopCollider.name : "<null>") +
                             " | seatProxy=" + (seatProxy != null ? seatProxy.name : "<null>");
                LogWarning(msg);
                TryConsolePrint(msg);

                if (F3AutoCallMitaSit && seatProxy != null)
                {
                    // v0.9.1：
                    // 不再由测试器先 AiWalkToTarget，再 timeout 后调用 Mita_sit。
                    // v0.8.2 已证明这种“两套移动系统串联”会导致跟踪 root 错误、等待超时、
                    // 以及 Mita_sit 二次接管时出现诡异拉拽。
                    // 现在测试器只负责生成有正确 forward 的 DepthSeatProxy_Bed_xxx；
                    // 真正的走近、转身、坐下全部交给 MakemitAGA.Mita_sit。
                    LogWarning("F3 v0.9.1 delayed bridge: scheduling Mita_sit for unique proxy " + seatProxyName + ".");
                    ScheduleBridgeSit(seatProxyName);
                }
            }
            catch (Exception e)
            {
                LogError("ScanHardcodedBedTopMeshAndSit exception: " + e);
            }
            finally
            {
                RestoreLayers(backups);
                try { if (scanCam != null) scanCam.targetTexture = null; } catch { }
                DestroyObject(scanCameraObj);
            }
        }

        private static Bounds BuildScanVolume(Vector3 hitPoint, bool hasTargetBounds, Bounds targetBounds, FakeColliderRequest request)
        {
            float centerX = hitPoint.x;
            float centerZ = hitPoint.z;
            float sizeX;
            float sizeZ;

            if (request.mode == FakeColliderMode.Top && hasTargetBounds)
            {
                centerX = targetBounds.center.x;
                centerZ = targetBounds.center.z;
                sizeX = Mathf.Clamp(targetBounds.size.x + TopScanPadding, FixedScanMinSize, TopScanMaxSize);
                sizeZ = Mathf.Clamp(targetBounds.size.z + TopScanPadding, FixedScanMinSize, TopScanMaxSize);
            }
            else
            {
                float requestedX = request.width > 0f ? request.width : DefaultSeatProxyWidth;
                float requestedZ = request.depth > 0f ? request.depth : DefaultSeatProxyDepth;
                sizeX = Mathf.Clamp(requestedX + FixedScanPadding, FixedScanMinSize, FixedScanMaxSize);
                sizeZ = Mathf.Clamp(requestedZ + FixedScanPadding, FixedScanMinSize, FixedScanMaxSize);
            }

            float minY;
            float maxY;
            if (hasTargetBounds)
            {
                float rendererTop = targetBounds.max.y + ScanBoxRendererTopMargin;
                float rendererBottom = targetBounds.min.y - 0.05f;
                minY = Mathf.Max(hitPoint.y - ScanBoxBelowHit, rendererBottom);
                maxY = Mathf.Max(hitPoint.y + ScanBoxAboveHitWithBounds, rendererTop);
                maxY = Mathf.Min(maxY, rendererTop + ScanBoxTargetTopExtra);
                if (maxY <= minY + 0.08f) maxY = minY + 0.35f;
            }
            else
            {
                minY = hitPoint.y - ScanBoxBelowHit;
                maxY = hitPoint.y + ScanBoxAboveHitNoBounds;
            }

            if (maxY - minY > ScanBoxMaxHeight)
            {
                maxY = Mathf.Min(maxY, hitPoint.y + ScanBoxAboveHitNoBounds);
                minY = maxY - ScanBoxMaxHeight;
            }

            return new Bounds(new Vector3(centerX, (minY + maxY) * 0.5f, centerZ), new Vector3(sizeX, maxY - minY, sizeZ));
        }

        private static void ConfigureScanCamera(Camera scanCam, Bounds volume, int scanLayer)
        {
            Vector3 center = volume.center;
            scanCam.enabled = false;
            scanCam.orthographic = true;
            scanCam.transform.position = new Vector3(center.x, volume.max.y + CameraDistance, center.z);
            scanCam.transform.rotation = Quaternion.LookRotation(Vector3.down, Vector3.forward);
            scanCam.orthographicSize = Mathf.Max(MinOrthoSize, Mathf.Max(volume.size.x, volume.size.z) * 0.5f + OrthoPadding);
            scanCam.aspect = 1f;
            scanCam.nearClipPlane = 0.02f;
            scanCam.farClipPlane = Mathf.Max(CameraDistance + volume.size.y + 0.50f, CameraDistance * 2.5f);
            scanCam.clearFlags = CameraClearFlags.SolidColor;
            scanCam.backgroundColor = Color.black;
            scanCam.cullingMask = 1 << scanLayer;
            scanCam.depthTextureMode |= DepthTextureMode.Depth;
        }

        private static bool CaptureTopViewToHeightfield(Camera scanCam, Bounds volume, List<Vector3> vertices, List<int> triangles, out HeightfieldStats stats, out DepthPath depthPath)
        {
            stats = new HeightfieldStats();
            depthPath = DepthPath.None;

            if (_eyeDepthReplacementShader != null)
            {
                if (CaptureReplacementEyeDepthPath(scanCam, volume, vertices, triangles, out stats))
                {
                    depthPath = DepthPath.EyeDepthReplacement;
                    LogWarning("Depth capture path: EyeDepthReplacement direct render.");
                    return true;
                }
                vertices.Clear();
                triangles.Clear();
                stats = new HeightfieldStats();
                LogWarning("EyeDepthReplacement path failed; trying fallback paths.");
            }

            if (_depthToEyeMat != null)
            {
                if (CaptureDepthToEyePath(scanCam, volume, vertices, triangles, out stats))
                {
                    depthPath = DepthPath.DepthToEyeMaterial;
                    LogWarning("Depth capture path: old DepthToEye material fallback.");
                    return true;
                }
                vertices.Clear();
                triangles.Clear();
                stats = new HeightfieldStats();
            }

            return false;
        }

        private static bool CaptureReplacementEyeDepthPath(Camera scanCam, Bounds volume, List<Vector3> vertices, List<int> triangles, out HeightfieldStats stats)
        {
            stats = new HeightfieldStats();
            if (_eyeDepthReplacementShader == null) return false;

            RenderTexture depthRT = null;
            Texture2D depthTex = null;

            try
            {
                RenderTextureFormat depthFormat = RenderTextureFormat.ARGBFloat;
                TextureFormat texFormat = TextureFormat.RGBAFloat;
                if (!SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.ARGBFloat))
                {
                    depthFormat = RenderTextureFormat.ARGBHalf;
                    texFormat = TextureFormat.RGBAHalf;
                }

                depthRT = new RenderTexture(CaptureSize, CaptureSize, 24, depthFormat);
                depthRT.name = "TopSurface_ReplacementEyeDepthRT";
                depthRT.Create();
                scanCam.targetTexture = depthRT;
                scanCam.RenderWithShader(_eyeDepthReplacementShader, "");

                depthTex = new Texture2D(CaptureSize, CaptureSize, texFormat, false, true);
                RenderTexture old = RenderTexture.active;
                try
                {
                    RenderTexture.active = depthRT;
                    depthTex.ReadPixels(new Rect(0, 0, CaptureSize, CaptureSize), 0, 0, false);
                    depthTex.Apply(false, false);
                }
                finally
                {
                    RenderTexture.active = old;
                }

                return BuildTopHeightfieldMesh(scanCam, depthTex, volume, vertices, triangles, out stats);
            }
            catch (Exception e)
            {
                LogError("CaptureReplacementEyeDepthPath exception: " + e);
                return false;
            }
            finally
            {
                if (scanCam != null) scanCam.targetTexture = null;
                ReleaseRT(depthRT);
                DestroyObject(depthTex);
            }
        }

        private static bool CaptureDepthToEyePath(Camera scanCam, Bounds volume, List<Vector3> vertices, List<int> triangles, out HeightfieldStats stats)
        {
            stats = new HeightfieldStats();
            RenderTexture colorRT = null;
            RenderTexture depthRT = null;
            Texture2D depthTex = null;

            try
            {
                colorRT = new RenderTexture(CaptureSize, CaptureSize, 24, RenderTextureFormat.ARGB32);
                colorRT.Create();

                RenderTextureFormat depthFormat = RenderTextureFormat.ARGBFloat;
                TextureFormat texFormat = TextureFormat.RGBAFloat;
                if (!SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.ARGBFloat))
                {
                    depthFormat = RenderTextureFormat.ARGBHalf;
                    texFormat = TextureFormat.RGBAHalf;
                }

                depthRT = new RenderTexture(CaptureSize, CaptureSize, 0, depthFormat);
                depthRT.Create();
                scanCam.targetTexture = colorRT;
                scanCam.Render();
                Graphics.Blit(colorRT, depthRT, _depthToEyeMat);

                depthTex = new Texture2D(CaptureSize, CaptureSize, texFormat, false, true);
                RenderTexture old = RenderTexture.active;
                try
                {
                    RenderTexture.active = depthRT;
                    depthTex.ReadPixels(new Rect(0, 0, CaptureSize, CaptureSize), 0, 0, false);
                    depthTex.Apply(false, false);
                }
                finally { RenderTexture.active = old; }

                return BuildTopHeightfieldMesh(scanCam, depthTex, volume, vertices, triangles, out stats);
            }
            catch (Exception e)
            {
                LogError("CaptureDepthToEyePath exception: " + e);
                return false;
            }
            finally
            {
                if (scanCam != null) scanCam.targetTexture = null;
                ReleaseRT(colorRT);
                ReleaseRT(depthRT);
                DestroyObject(depthTex);
            }
        }

        private static bool BuildTopHeightfieldMesh(Camera cam, Texture2D depthTex, Bounds volume, List<Vector3> allVertices, List<int> allTriangles, out HeightfieldStats stats)
        {
            stats = new HeightfieldStats();
            bool[,] valid = new bool[MeshGrid, MeshGrid];
            float[,] height = new float[MeshGrid, MeshGrid];
            for (int z = 0; z < MeshGrid; z++)
                for (int x = 0; x < MeshGrid; x++)
                    height[x, z] = volume.center.y;

            List<float> pixelHeights = new List<float>();
            float minX = volume.min.x;
            float maxX = volume.max.x;
            float minZ = volume.min.z;
            float maxZ = volume.max.z;
            float invWidth = Mathf.Abs(maxX - minX) > 0.0001f ? 1f / (maxX - minX) : 1f;
            float invDepth = Mathf.Abs(maxZ - minZ) > 0.0001f ? 1f / (maxZ - minZ) : 1f;
            int rawPixelValid = 0;

            for (int py = 0; py < CaptureSize; py++)
            {
                for (int px = 0; px < CaptureSize; px++)
                {
                    DepthPoint p;
                    if (!TryReadDepthPoint(cam, depthTex, px, py, out p)) continue;
                    if (!volume.Contains(p.world)) continue;

                    rawPixelValid++;
                    pixelHeights.Add(p.world.y);

                    int gx = Mathf.Clamp(Mathf.FloorToInt((p.world.x - minX) * invWidth * MeshGrid), 0, MeshGrid - 1);
                    int gz = Mathf.Clamp(Mathf.FloorToInt((p.world.z - minZ) * invDepth * MeshGrid), 0, MeshGrid - 1);
                    if (!valid[gx, gz] || p.world.y > height[gx, gz])
                    {
                        valid[gx, gz] = true;
                        height[gx, gz] = p.world.y;
                    }
                }
            }

            if (pixelHeights.Count < 16)
            {
                LogWarning("Top heightfield has too few valid depth pixels: " + pixelHeights.Count);
                return false;
            }

            pixelHeights.Sort();
            float median = pixelHeights[pixelHeights.Count / 2];
            int rawCellValid = 0;
            int kept = 0;
            int removed = 0;

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!valid[x, z]) continue;
                    rawCellValid++;
                    if (Mathf.Abs(height[x, z] - median) > MaxHeightDeviationFromMedian)
                    {
                        valid[x, z] = false;
                        removed++;
                    }
                    else kept++;
                }
            }

            if (kept < 16) return false;

            bool[,] mask = BuildSurfaceMask(valid);
            FillMissingHeightsMasked(mask, valid, height, median);
            SmoothHeightsMasked(mask, height, HeightSmoothStrength);

            List<float> finalHeights = new List<float>();
            List<float> seatPatchHeights = new List<float>();
            float minY = float.MaxValue;
            float maxY = float.MinValue;
            float sumY = 0f;
            int surfaceCells = 0;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float wz = Mathf.Lerp(minZ, maxZ, tz);
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!mask[x, z]) continue;
                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float wx = Mathf.Lerp(minX, maxX, tx);
                    float y = height[x, z];
                    surfaceCells++;
                    finalHeights.Add(y);
                    sumY += y;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                    if (Mathf.Abs(wx - volume.center.x) <= DefaultSeatProxyWidth * 0.5f && Mathf.Abs(wz - volume.center.z) <= DefaultSeatProxyDepth * 0.5f)
                        seatPatchHeights.Add(y);
                }
            }

            if (surfaceCells < 8) return false;
            finalHeights.Sort();
            seatPatchHeights.Sort();
            float seatY = seatPatchHeights.Count > 0 ? seatPatchHeights[seatPatchHeights.Count / 2] : finalHeights[finalHeights.Count / 2];
            float avgY = sumY / finalHeights.Count;

            int[,] index = new int[MeshGrid, MeshGrid];
            for (int z = 0; z < MeshGrid; z++)
                for (int x = 0; x < MeshGrid; x++)
                    index[x, z] = -1;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float wz = Mathf.Lerp(minZ, maxZ, tz);
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!mask[x, z]) continue;
                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float wx = Mathf.Lerp(minX, maxX, tx);
                    index[x, z] = allVertices.Count;
                    allVertices.Add(new Vector3(wx, height[x, z] + VisualLift, wz));
                }
            }

            BuildTrianglesFromGrid(mask, height, index, allVertices, allTriangles);

            stats.valid = true;
            stats.rawPixelValidCount = rawPixelValid;
            stats.rawCellValidCount = rawCellValid;
            stats.keptCellCount = kept;
            stats.surfaceCellCount = surfaceCells;
            stats.removedOutliers = removed;
            stats.medianY = seatY;
            stats.minY = minY;
            stats.maxY = maxY;
            stats.center = new Vector3(volume.center.x, seatY, volume.center.z);
            stats.volume = volume;
            stats.surfaceMask = mask;
            stats.heights = height;

            LogWarning("Top heightfield mesh built. rawPixels=" + rawPixelValid + ", rawCells=" + rawCellValid + ", kept=" + kept + ", surfaceCells=" + surfaceCells + ", seatY=" + seatY.ToString("F3") + ", avgY=" + avgY.ToString("F3"));
            return true;
        }

        private static void BuildTrianglesFromGrid(bool[,] mask, float[,] height, int[,] index, List<Vector3> verts, List<int> tris)
        {
            for (int z = 0; z < MeshGrid - 1; z++)
            {
                for (int x = 0; x < MeshGrid - 1; x++)
                {
                    int i00 = index[x, z];
                    int i10 = index[x + 1, z];
                    int i01 = index[x, z + 1];
                    int i11 = index[x + 1, z + 1];

                    if (i00 >= 0 && i01 >= 0 && i10 >= 0 && CanConnectHeightTriangle(height[x, z], height[x, z + 1], height[x + 1, z]))
                        AddTriangleFacingUp(tris, verts, i00, i01, i10);

                    if (i11 >= 0 && i10 >= 0 && i01 >= 0 && CanConnectHeightTriangle(height[x + 1, z + 1], height[x + 1, z], height[x, z + 1]))
                        AddTriangleFacingUp(tris, verts, i11, i10, i01);
                }
            }
        }

        private static bool CanConnectHeightTriangle(float a, float b, float c)
        {
            float min = Mathf.Min(a, Mathf.Min(b, c));
            float max = Mathf.Max(a, Mathf.Max(b, c));
            return (max - min) <= MaxTriangleHeightDelta;
        }

        private static bool[,] BuildSurfaceMask(bool[,] valid)
        {
            bool[,] mask = new bool[MeshGrid, MeshGrid];
            for (int z = 0; z < MeshGrid; z++)
                for (int x = 0; x < MeshGrid; x++)
                    mask[x, z] = valid[x, z];

            for (int i = 0; i < SurfaceMaskDilateIterations; i++) mask = DilateMask(mask);
            RemoveSmallIslands(mask, MinSurfaceIslandCells);
            return mask;
        }

        private static bool[,] DilateMask(bool[,] src)
        {
            bool[,] dst = new bool[MeshGrid, MeshGrid];
            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (src[x, z]) { dst[x, z] = true; continue; }
                    bool near = false;
                    for (int dz = -1; dz <= 1 && !near; dz++)
                    for (int dx = -1; dx <= 1 && !near; dx++)
                    {
                        int nx = x + dx;
                        int nz = z + dz;
                        if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;
                        if (src[nx, nz]) near = true;
                    }
                    dst[x, z] = near;
                }
            }
            return dst;
        }

        private static void RemoveSmallIslands(bool[,] mask, int minCells)
        {
            bool[,] visited = new bool[MeshGrid, MeshGrid];
            List<Vector2Int> comp = new List<Vector2Int>();
            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!mask[x, z] || visited[x, z]) continue;
                    comp.Clear();
                    FloodCollect(mask, visited, x, z, comp);
                    if (comp.Count < minCells)
                        for (int i = 0; i < comp.Count; i++) mask[comp[i].x, comp[i].y] = false;
                }
            }
        }

        private static void FloodCollect(bool[,] mask, bool[,] visited, int sx, int sz, List<Vector2Int> output)
        {
            Queue<Vector2Int> q = new Queue<Vector2Int>();
            visited[sx, sz] = true;
            q.Enqueue(new Vector2Int(sx, sz));
            while (q.Count > 0)
            {
                Vector2Int p = q.Dequeue();
                output.Add(p);
                for (int dz = -1; dz <= 1; dz++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dz == 0) continue;
                    int nx = p.x + dx;
                    int nz = p.y + dz;
                    if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;
                    if (visited[nx, nz] || !mask[nx, nz]) continue;
                    visited[nx, nz] = true;
                    q.Enqueue(new Vector2Int(nx, nz));
                }
            }
        }

        private static void FillMissingHeightsMasked(bool[,] mask, bool[,] valid, float[,] height, float fallback)
        {
            for (int iter = 0; iter < HeightFillIterations; iter++)
            {
                bool changed = false;
                bool[,] nextValid = new bool[MeshGrid, MeshGrid];
                float[,] nextHeight = new float[MeshGrid, MeshGrid];
                for (int z = 0; z < MeshGrid; z++)
                {
                    for (int x = 0; x < MeshGrid; x++)
                    {
                        nextValid[x, z] = valid[x, z];
                        nextHeight[x, z] = height[x, z];
                        if (!mask[x, z] || valid[x, z]) continue;
                        float sum = 0f;
                        int count = 0;
                        for (int dz = -1; dz <= 1; dz++)
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            if (dx == 0 && dz == 0) continue;
                            int nx = x + dx;
                            int nz = z + dz;
                            if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;
                            if (!mask[nx, nz] || !valid[nx, nz]) continue;
                            sum += height[nx, nz];
                            count++;
                        }
                        if (count > 0)
                        {
                            nextValid[x, z] = true;
                            nextHeight[x, z] = sum / count;
                            changed = true;
                        }
                    }
                }
                for (int z = 0; z < MeshGrid; z++)
                for (int x = 0; x < MeshGrid; x++)
                {
                    valid[x, z] = nextValid[x, z];
                    height[x, z] = nextHeight[x, z];
                }
                if (!changed) break;
            }

            for (int z = 0; z < MeshGrid; z++)
            for (int x = 0; x < MeshGrid; x++)
            {
                if (mask[x, z] && !valid[x, z])
                {
                    valid[x, z] = true;
                    height[x, z] = fallback;
                }
            }
        }

        private static void SmoothHeightsMasked(bool[,] mask, float[,] height, float strength)
        {
            if (strength <= 0f) return;
            float[,] copy = new float[MeshGrid, MeshGrid];
            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!mask[x, z]) { copy[x, z] = height[x, z]; continue; }
                    float sum = 0f;
                    int count = 0;
                    for (int dz = -1; dz <= 1; dz++)
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        int nx = x + dx;
                        int nz = z + dz;
                        if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;
                        if (!mask[nx, nz]) continue;
                        sum += height[nx, nz];
                        count++;
                    }
                    float avg = count > 0 ? sum / count : height[x, z];
                    copy[x, z] = Mathf.Lerp(height[x, z], avg, strength);
                }
            }
            for (int z = 0; z < MeshGrid; z++)
                for (int x = 0; x < MeshGrid; x++) height[x, z] = copy[x, z];
        }

        private static bool TryReadDepthPoint(Camera cam, Texture2D depthTex, int px, int py, out DepthPoint p)
        {
            p = new DepthPoint { px = px, py = py, valid = false };
            Color c = depthTex.GetPixel(px, py);
            float eye = c.r;
            if (float.IsNaN(eye) || float.IsInfinity(eye)) return false;
            if (eye <= cam.nearClipPlane + 0.01f) return false;
            if (eye >= cam.farClipPlane * 0.96f) return false;
            p.eyeDepth = eye;
            p.world = UnprojectEyeDepth(cam, px + 0.5f, py + 0.5f, eye);
            p.valid = true;
            return true;
        }

        private static Vector3 UnprojectEyeDepth(Camera cam, float screenX, float screenY, float eyeDepth)
        {
            Ray ray = cam.ScreenPointToRay(new Vector3(screenX, screenY, 0f));
            float denom = Vector3.Dot(ray.direction, cam.transform.forward);
            if (Mathf.Abs(denom) < 0.0001f) denom = denom >= 0f ? 0.0001f : -0.0001f;
            float originEye = Vector3.Dot(ray.origin - cam.transform.position, cam.transform.forward);
            float rayDistance = (eyeDepth - originEye) / denom;
            if (rayDistance < 0f) rayDistance = eyeDepth / Mathf.Abs(denom);
            return ray.origin + ray.direction * rayDistance;
        }

        private static GameObject BuildHeightfieldMesh(Transform parent, List<Vector3> vertices, List<int> triangles)
        {
            GameObject go = new GameObject("TopSurfaceSeatProxy_HeightfieldMesh");
            go.transform.SetParent(parent, true);
            Mesh mesh = new Mesh { name = "TopSurfaceSeatProxy_HeightfieldMeshData" };
            mesh.SetVertices(ToIl2CppVector3List(vertices));
            mesh.SetTriangles(ToIl2CppIntList(triangles), 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            MeshFilter mf = go.AddComponent<MeshFilter>();
            MeshRenderer mr = go.AddComponent<MeshRenderer>();
            mf.mesh = mesh;
            mr.material = _heightfieldMat;
            RegisterDebugRenderer(mr);
            return go;
        }

        private static GameObject BuildTopMeshColliderFromStats(Transform parent, HeightfieldStats stats)
        {
            if (!stats.valid || stats.surfaceMask == null || stats.heights == null) return null;

            List<Vector3> verts = new List<Vector3>();
            List<int> tris = new List<int>();
            int[,] index = new int[MeshGrid, MeshGrid];
            for (int z = 0; z < MeshGrid; z++)
                for (int x = 0; x < MeshGrid; x++) index[x, z] = -1;

            Bounds volume = stats.volume;
            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float wz = Mathf.Lerp(volume.min.z, volume.max.z, tz);
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!stats.surfaceMask[x, z]) continue;
                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float wx = Mathf.Lerp(volume.min.x, volume.max.x, tx);
                    index[x, z] = verts.Count;
                    // MeshCollider 不加 VisualLift，必须贴真实表面。
                    verts.Add(new Vector3(wx, stats.heights[x, z], wz));
                }
            }

            BuildTrianglesFromGrid(stats.surfaceMask, stats.heights, index, verts, tris);
            if (verts.Count < 3 || tris.Count < 3) return null;

            GameObject go = new GameObject(BedTopMeshColliderName);
            if (parent != null) go.transform.SetParent(parent, true);

            Mesh mesh = new Mesh { name = BedTopMeshColliderName + "_Mesh" };
            mesh.SetVertices(ToIl2CppVector3List(verts));
            mesh.SetTriangles(ToIl2CppIntList(tris), 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            MeshFilter mf = go.AddComponent<MeshFilter>();
            mf.mesh = mesh;
            MeshRenderer mr = go.AddComponent<MeshRenderer>();
            mr.material = _heightfieldMat;
            mr.enabled = false;
            RegisterDebugRenderer(mr);

            MeshCollider mc = go.AddComponent<MeshCollider>();
            mc.sharedMesh = null;
            mc.sharedMesh = mesh;
            mc.convex = false;
            mc.isTrigger = false;

            LogWarning("Bed Top MeshCollider created. vertices=" + verts.Count + ", triangles=" + (tris.Count / 3));
            return go;
        }

        private static GameObject FakeCollider(Transform parent, HeightfieldStats stats, FakeColliderRequest request)
        {
            if (!stats.valid || stats.surfaceMask == null || stats.heights == null) return null;

            float width = request.width;
            float depth = request.depth;
            float height = request.height;
            Vector3 topCenter = stats.center;

            if (request.mode == FakeColliderMode.Top)
            {
                if (!TryGetSupportedTopBounds(stats, out topCenter, out width, out depth)) return null;
                if (height <= 0f) height = DefaultSeatProxyHeight;
            }
            else
            {
                if (width <= 0f) width = DefaultSeatProxyWidth;
                if (depth <= 0f) depth = DefaultSeatProxyDepth;
                if (height <= 0f) height = DefaultSeatProxyHeight;

                float coverage = CalculateSupportCoverage(stats, topCenter.x, topCenter.z, width, depth);
                if (coverage < MinFakeColliderSupportCoverage && request.clampToSupportedSurface)
                {
                    Vector3 newCenter;
                    float newWidth;
                    float newDepth;
                    if (TryClampRectToSupportedSurface(stats, topCenter.x, topCenter.z, width, depth, out newCenter, out newWidth, out newDepth))
                    {
                        topCenter.x = newCenter.x;
                        topCenter.z = newCenter.z;
                        width = newWidth;
                        depth = newDepth;
                    }
                    else return null;
                }

                topCenter.y = EstimateRectSeatHeight(stats, topCenter.x, topCenter.z, width, depth);
            }

            return CreateSeatProxyAtSurfacePoint(parent, topCenter, width, depth, height, Quaternion.identity);
        }

        private static GameObject CreateSeatProxyAtSurfacePoint(Transform parent, Vector3 topCenter, float width, float depth, float height, Quaternion rotation)
        {
            return CreateSeatProxyAtSurfacePoint(parent, topCenter, width, depth, height, rotation, SeatProxyName);
        }

        private static GameObject CreateSeatProxyAtSurfacePoint(Transform parent, Vector3 topCenter, float width, float depth, float height, Quaternion rotation, string proxyName)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = proxyName;
            go.transform.position = new Vector3(topCenter.x, topCenter.y - height * 0.5f, topCenter.z);
            go.transform.rotation = rotation;
            go.transform.localScale = new Vector3(width, height, depth);
            if (parent != null) go.transform.SetParent(parent, true);

            Collider col = null;
            try { col = go.GetComponent<Collider>(); } catch { }
            if (col != null) col.isTrigger = false;

            Renderer r = null;
            try { r = go.GetComponent<Renderer>(); } catch { }
            if (r != null)
            {
                r.material = _seatBoxMat;
                r.enabled = SeatProxyRendererVisibleByDefault;
                RegisterDebugRenderer(r);
            }

            try { Physics.SyncTransforms(); } catch { }

            LogWarning("Seat proxy created. name=" + go.name + ", top=" + Vec(topCenter) + ", forward=" + Vec(go.transform.forward) + ", size=(" + width.ToString("F2") + "," + height.ToString("F2") + "," + depth.ToString("F2") + ")");
            return go;
        }

        private static bool TryGetSurfacePointAtXZ(HeightfieldStats stats, float worldX, float worldZ, out Vector3 point)
        {
            point = stats.center;
            if (!stats.valid || stats.surfaceMask == null || stats.heights == null) return false;

            Bounds volume = stats.volume;
            int gx = Mathf.Clamp(Mathf.RoundToInt(Mathf.InverseLerp(volume.min.x, volume.max.x, worldX) * (MeshGrid - 1)), 0, MeshGrid - 1);
            int gz = Mathf.Clamp(Mathf.RoundToInt(Mathf.InverseLerp(volume.min.z, volume.max.z, worldZ) * (MeshGrid - 1)), 0, MeshGrid - 1);

            int bestX = -1;
            int bestZ = -1;
            float bestD2 = float.MaxValue;
            for (int radius = 0; radius <= 10; radius++)
            {
                for (int dz = -radius; dz <= radius; dz++)
                for (int dx = -radius; dx <= radius; dx++)
                {
                    int x = gx + dx;
                    int z = gz + dz;
                    if (x < 0 || x >= MeshGrid || z < 0 || z >= MeshGrid) continue;
                    if (!stats.surfaceMask[x, z]) continue;
                    float d2 = dx * dx + dz * dz;
                    if (d2 < bestD2) { bestD2 = d2; bestX = x; bestZ = z; }
                }
                if (bestX >= 0) break;
            }

            if (bestX < 0) return false;
            float tx = MeshGrid == 1 ? 0.5f : bestX / (float)(MeshGrid - 1);
            float tz = MeshGrid == 1 ? 0.5f : bestZ / (float)(MeshGrid - 1);
            float px = Mathf.Lerp(volume.min.x, volume.max.x, tx);
            float pz = Mathf.Lerp(volume.min.z, volume.max.z, tz);
            if (bestX == gx && bestZ == gz) { px = worldX; pz = worldZ; }
            point = new Vector3(px, stats.heights[bestX, bestZ], pz);
            return true;
        }

        private static float CalculateSupportCoverage(HeightfieldStats stats, float centerX, float centerZ, float width, float depth)
        {
            int supported = 0;
            int total = 0;
            Bounds volume = stats.volume;
            for (int z = 0; z < MeshGrid; z++)
            {
                float wz = Mathf.Lerp(volume.min.z, volume.max.z, z / (float)(MeshGrid - 1));
                if (Mathf.Abs(wz - centerZ) > depth * 0.5f) continue;
                for (int x = 0; x < MeshGrid; x++)
                {
                    float wx = Mathf.Lerp(volume.min.x, volume.max.x, x / (float)(MeshGrid - 1));
                    if (Mathf.Abs(wx - centerX) > width * 0.5f) continue;
                    total++;
                    if (stats.surfaceMask[x, z]) supported++;
                }
            }
            return total <= 0 ? 0f : supported / (float)total;
        }

        private static float EstimateRectSeatHeight(HeightfieldStats stats, float centerX, float centerZ, float width, float depth)
        {
            List<float> ys = new List<float>();
            Bounds volume = stats.volume;
            for (int z = 0; z < MeshGrid; z++)
            {
                float wz = Mathf.Lerp(volume.min.z, volume.max.z, z / (float)(MeshGrid - 1));
                if (Mathf.Abs(wz - centerZ) > depth * 0.5f) continue;
                for (int x = 0; x < MeshGrid; x++)
                {
                    float wx = Mathf.Lerp(volume.min.x, volume.max.x, x / (float)(MeshGrid - 1));
                    if (Mathf.Abs(wx - centerX) > width * 0.5f) continue;
                    if (!stats.surfaceMask[x, z]) continue;
                    ys.Add(stats.heights[x, z]);
                }
            }
            if (ys.Count == 0) return stats.medianY;
            ys.Sort();
            int index = Mathf.Clamp(Mathf.RoundToInt(ys.Count * 0.40f), 0, ys.Count - 1);
            return ys[index];
        }

        private static bool TryClampRectToSupportedSurface(HeightfieldStats stats, float centerX, float centerZ, float width, float depth, out Vector3 newCenter, out float newWidth, out float newDepth)
        {
            newCenter = new Vector3(centerX, stats.medianY, centerZ);
            newWidth = width;
            newDepth = depth;
            Bounds volume = stats.volume;
            float minX = float.MaxValue, maxX = float.MinValue, minZ = float.MaxValue, maxZ = float.MinValue;
            for (int z = 0; z < MeshGrid; z++)
            {
                float wz = Mathf.Lerp(volume.min.z, volume.max.z, z / (float)(MeshGrid - 1));
                if (Mathf.Abs(wz - centerZ) > depth * 0.5f) continue;
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!stats.surfaceMask[x, z]) continue;
                    float wx = Mathf.Lerp(volume.min.x, volume.max.x, x / (float)(MeshGrid - 1));
                    if (Mathf.Abs(wx - centerX) > width * 0.5f) continue;
                    minX = Mathf.Min(minX, wx); maxX = Mathf.Max(maxX, wx); minZ = Mathf.Min(minZ, wz); maxZ = Mathf.Max(maxZ, wz);
                }
            }
            if (minX == float.MaxValue || minZ == float.MaxValue) return false;
            newWidth = Mathf.Max(MinFakeColliderSize, maxX - minX);
            newDepth = Mathf.Max(MinFakeColliderSize, maxZ - minZ);
            float cx = (minX + maxX) * 0.5f;
            float cz = (minZ + maxZ) * 0.5f;
            float cy = EstimateRectSeatHeight(stats, cx, cz, newWidth, newDepth);
            newCenter = new Vector3(cx, cy, cz);
            return true;
        }

        private static bool TryGetSupportedTopBounds(HeightfieldStats stats, out Vector3 center, out float width, out float depth)
        {
            center = stats.center;
            width = 0f;
            depth = 0f;
            Bounds volume = stats.volume;
            float minX = float.MaxValue, maxX = float.MinValue, minZ = float.MaxValue, maxZ = float.MinValue;
            for (int z = 0; z < MeshGrid; z++)
            {
                float wz = Mathf.Lerp(volume.min.z, volume.max.z, z / (float)(MeshGrid - 1));
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!stats.surfaceMask[x, z]) continue;
                    float wx = Mathf.Lerp(volume.min.x, volume.max.x, x / (float)(MeshGrid - 1));
                    minX = Mathf.Min(minX, wx); maxX = Mathf.Max(maxX, wx); minZ = Mathf.Min(minZ, wz); maxZ = Mathf.Max(maxZ, wz);
                }
            }
            if (minX == float.MaxValue || minZ == float.MaxValue) return false;
            width = Mathf.Max(MinFakeColliderSize, maxX - minX);
            depth = Mathf.Max(MinFakeColliderSize, maxZ - minZ);
            float cx = (minX + maxX) * 0.5f;
            float cz = (minZ + maxZ) * 0.5f;
            float cy = EstimateRectSeatHeight(stats, cx, cz, width, depth);
            center = new Vector3(cx, cy, cz);
            return true;
        }

        private static void AddTriangleFacingUp(List<int> triangles, List<Vector3> vertices, int ia, int ib, int ic)
        {
            Vector3 a = vertices[ia], b = vertices[ib], c = vertices[ic];
            Vector3 n = Vector3.Cross(b - a, c - a);
            if (n.y < 0f) { triangles.Add(ia); triangles.Add(ic); triangles.Add(ib); }
            else { triangles.Add(ia); triangles.Add(ib); triangles.Add(ic); }
        }

        private static GameObject FindHardcodedBedObject()
        {
            GameObject[] all = null;
            try { all = Object.FindObjectsOfType<GameObject>(); } catch { }
            if (all == null) return null;

            GameObject best = null;
            int bestScore = int.MinValue;
            for (int i = 0; i < all.Length; i++)
            {
                GameObject go = all[i];
                if (go == null) continue;
                string n = go.name ?? string.Empty;
                int score = int.MinValue;
                if (n.Equals(HardcodedBedName, StringComparison.OrdinalIgnoreCase)) score = 1000;
                else if (n.IndexOf(HardcodedBedName, StringComparison.OrdinalIgnoreCase) >= 0 && n.IndexOf("Bedroom", StringComparison.OrdinalIgnoreCase) < 0) score = 200;
                else continue;

                string path = GetTransformPath(go.transform);
                if (path.IndexOf("Bedroom", StringComparison.OrdinalIgnoreCase) >= 0) score += 100;
                if (go.activeInHierarchy) score += 10;
                Renderer r = null;
                try { r = go.GetComponentInChildren<Renderer>(true); } catch { }
                if (r != null) score += 20;

                if (score > bestScore) { bestScore = score; best = go; }
            }

            if (best != null) LogWarning("FindHardcodedBedObject: selected=" + best.name + ", path=" + GetTransformPath(best.transform));
            return best;
        }

        private static string GetTransformPath(Transform t)
        {
            if (t == null) return "<null>";
            List<string> parts = new List<string>();
            while (t != null) { parts.Add(t.name); t = t.parent; }
            parts.Reverse();
            return string.Join("/", parts.ToArray());
        }

        private static bool TryRaycastSpecificTarget(Ray ray, GameObject targetRoot, out RaycastHit bestHit)
        {
            bestHit = new RaycastHit();
            if (targetRoot == null) return false;
            RaycastHit[] hits = null;
            try { hits = Physics.RaycastAll(ray, RayDistance, -1, QueryTriggerInteraction.Ignore); } catch { }
            if (hits == null || hits.Length == 0) return false;

            bool found = false;
            float bestDistance = float.MaxValue;
            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit h = hits[i];
                if (h.collider == null || h.distance < 0.03f || IsOwnVisual(h.collider.gameObject)) continue;
                if (!IsSameOrChildOf(h.collider.transform, targetRoot.transform)) continue;
                if (h.distance < bestDistance) { bestDistance = h.distance; bestHit = h; found = true; }
            }
            return found;
        }

        private static bool IsSameOrChildOf(Transform t, Transform root)
        {
            while (t != null)
            {
                if (t == root) return true;
                t = t.parent;
            }
            return false;
        }

        private static Vector3 FlattenXZ(Vector3 v)
        {
            v.y = 0f;
            return v;
        }

        private static Vector3 EstimateOutwardForwardFromBounds(Vector3 surfacePoint, Bounds bounds)
        {
            // 没有 NavMesh/floor point 时，根据 surfacePoint 到 bounds 四条边的距离，
            // 选择离它最近的一条边作为“外侧方向”。
            float dxMin = Mathf.Abs(surfacePoint.x - bounds.min.x);
            float dxMax = Mathf.Abs(bounds.max.x - surfacePoint.x);
            float dzMin = Mathf.Abs(surfacePoint.z - bounds.min.z);
            float dzMax = Mathf.Abs(bounds.max.z - surfacePoint.z);

            float best = dxMin;
            Vector3 dir = Vector3.left;

            if (dxMax < best) { best = dxMax; dir = Vector3.right; }
            if (dzMin < best) { best = dzMin; dir = Vector3.back; }
            if (dzMax < best) { best = dzMax; dir = Vector3.forward; }

            return dir;
        }

        private static bool TryFindNearestReachableNavPointNearSurface(Vector3 surfacePoint, Bounds targetBounds, out Vector3 navPoint)
        {
            navPoint = surfacePoint;

            // v0.9.1：
            //   第一优先级仍然是 NavMesh，因为 MitaPerson.AiWalkToTarget 最终也更适合走 NavMesh。
            //   但 v0.8.0 的日志显示 navPoint=<none>，说明仅靠 SamplePosition/CalculatePath 太脆。
            //   所以这里增加 Physics.Raycast 地板回退：
            //
            //     1. 先在目标床面点周围做环形 NavMesh 采样。
            //     2. 如果失败，再从床边周围向下打射线找 Floor/Carpet/Rug 等真实地面。
            //     3. 找到地面后，优先把它吸附回 NavMesh；如果吸附失败，也会创建隐藏 cube。
            //
            //   这样独立测试项目至少能稳定生成 DepthGotoPoint_Bed；
            //   MakemitAGA 存在时再尝试让 Mita 走过去并 Sit。

            if (TryFindNearestReachableNavMeshPointNearSurface(surfacePoint, targetBounds, out navPoint))
                return true;

            LogWarning("NavMesh point search failed. Trying Physics floor fallback near selected bed surface.");

            Vector3 floorPoint;
            if (!TryFindNearestFloorPointByPhysics(surfacePoint, targetBounds, out floorPoint))
            {
                LogWarning("Physics floor fallback also failed near " + Vec(surfacePoint));
                return false;
            }

            // 如果 Physics 找到地板，再尽量吸附到附近 NavMesh。
            NavMeshHit navHit;
            if (NavMesh.SamplePosition(floorPoint, out navHit, 1.25f, -1))
            {
                navPoint = navHit.position;
                LogWarning("Physics floor fallback snapped to NavMesh. floor=" + Vec(floorPoint) +
                           ", navPoint=" + Vec(navPoint) +
                           ", xzDistance=" + XZDistance(surfacePoint, navPoint).ToString("F2"));
                return true;
            }

            // 没有 NavMesh 也返回 floorPoint。这样至少可以生成 DepthGotoPoint_Bed 供 UnityExplorer 检查。
            navPoint = floorPoint;
            LogWarning("Using raw Physics floor point without NavMesh snap. floor=" + Vec(navPoint) +
                       ", xzDistance=" + XZDistance(surfacePoint, navPoint).ToString("F2"));
            return true;
        }

        private static bool TryFindNearestReachableNavMeshPointNearSurface(Vector3 surfacePoint, Bounds targetBounds, out Vector3 navPoint)
        {
            navPoint = surfacePoint;

            MitaPerson mita = null;
            try { mita = Object.FindObjectOfType<MitaPerson>(); } catch { }

            Vector3 start = mita != null ? mita.transform.position : surfacePoint;

            // v0.9.1：增加半径上限。床中心到床边可能接近 1m；再考虑床旁地毯/障碍，需要更大搜索圈。
            float[] radii = new float[]
            {
                0.35f, 0.50f, 0.70f, 0.90f, 1.10f, 1.35f, 1.60f, 1.90f,
                2.25f, 2.70f, 3.15f, 3.60f
            };

            float bestScore = float.MaxValue;
            Vector3 best = Vector3.zero;
            bool found = false;
            NavMeshPath path = new NavMeshPath();

            for (int r = 0; r < radii.Length; r++)
            {
                float radius = radii[r];
                int steps = Mathf.Max(20, Mathf.RoundToInt(radius * 28f));

                for (int i = 0; i < steps; i++)
                {
                    float angle = Mathf.PI * 2f * (i / (float)steps);
                    Vector3 sample = surfacePoint + new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * radius;

                    NavMeshHit navHit;
                    if (!NavMesh.SamplePosition(sample, out navHit, 0.85f, -1))
                        continue;

                    // 避免选到床体内部或床底。
                    if (IsXZInsideBounds(navHit.position, targetBounds, 0.05f))
                        continue;

                    float pathLen = 0f;
                    if (mita != null)
                    {
                        if (!NavMesh.CalculatePath(start, navHit.position, -1, path) ||
                            path.status != NavMeshPathStatus.PathComplete)
                            continue;

                        pathLen = EstimatePathLength(path);
                    }

                    float surfaceDist = XZDistance(surfacePoint, navHit.position);
                    float score = surfaceDist + pathLen * 0.08f;

                    if (score < bestScore)
                    {
                        bestScore = score;
                        best = navHit.position;
                        found = true;
                    }
                }

                if (found)
                    break;
            }

            if (!found)
                return false;

            navPoint = best;
            LogWarning("Nearest reachable NavMesh point found. surface=" + Vec(surfacePoint) +
                       ", navPoint=" + Vec(navPoint) +
                       ", xzDistance=" + XZDistance(surfacePoint, navPoint).ToString("F2"));
            return true;
        }

        private static bool TryFindNearestFloorPointByPhysics(Vector3 surfacePoint, Bounds targetBounds, out Vector3 floorPoint)
        {
            floorPoint = surfacePoint;

            float[] radii = new float[]
            {
                0.45f, 0.65f, 0.85f, 1.05f, 1.25f, 1.50f, 1.80f, 2.15f, 2.55f, 3.00f
            };

            float rayStartY = Mathf.Max(targetBounds.max.y + 1.35f, surfacePoint.y + 1.35f);
            float rayLength = 5.0f;

            float bestScore = float.MaxValue;
            Vector3 best = Vector3.zero;
            bool found = false;

            for (int r = 0; r < radii.Length; r++)
            {
                float radius = radii[r];
                int steps = Mathf.Max(24, Mathf.RoundToInt(radius * 32f));

                for (int i = 0; i < steps; i++)
                {
                    float angle = Mathf.PI * 2f * (i / (float)steps);
                    Vector3 offset = new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * radius;
                    Vector3 sampleXZ = surfacePoint + offset;

                    // 避免在床的 XZ 范围内向下找点，这通常会打到床、床底或床下不可走区域。
                    if (IsXZInsideBounds(sampleXZ, targetBounds, 0.08f))
                        continue;

                    Vector3 origin = new Vector3(sampleXZ.x, rayStartY, sampleXZ.z);

                    RaycastHit[] hits = null;
                    try { hits = Physics.RaycastAll(origin, Vector3.down, rayLength, -1, QueryTriggerInteraction.Ignore); }
                    catch { hits = null; }

                    if (hits == null || hits.Length == 0)
                        continue;

                    SortRaycastHitsByDistance(hits);

                    for (int h = 0; h < hits.Length; h++)
                    {
                        RaycastHit hit = hits[h];

                        if (hit.collider == null)
                            continue;

                        GameObject hitGo = hit.collider.gameObject;
                        if (IsOwnVisual(hitGo))
                            continue;

                        // 不能把床、床垫、枕头当作地面点。
                        if (IsXZInsideBounds(hit.point, targetBounds, 0.05f))
                            continue;

                        // 地面点应该低于床面。否则容易把桌面、凳面、柜子顶当作可达地面。
                        if (hit.point.y > surfacePoint.y - 0.12f)
                            continue;

                        string n = hitGo != null ? (hitGo.name ?? string.Empty) : string.Empty;
                        string path = hitGo != null ? GetTransformPath(hitGo.transform) : string.Empty;

                        // 命名命中地板/地毯时加分，但不强制要求，避免游戏对象命名不稳定。
                        float nameBonus = 0f;
                        string lower = (n + "/" + path).ToLowerInvariant();
                        if (lower.Contains("floor")) nameBonus -= 0.25f;
                        if (lower.Contains("carpet")) nameBonus -= 0.18f;
                        if (lower.Contains("rug")) nameBonus -= 0.18f;

                        float surfaceDist = XZDistance(surfacePoint, hit.point);
                        float yPenalty = Mathf.Abs(hit.point.y - targetBounds.min.y) * 0.05f;
                        float score = surfaceDist + yPenalty + nameBonus;

                        if (score < bestScore)
                        {
                            bestScore = score;
                            best = hit.point;
                            found = true;
                        }

                        break;
                    }
                }

                if (found)
                    break;
            }

            if (!found)
                return false;

            floorPoint = best;
            LogWarning("Nearest Physics floor point found. surface=" + Vec(surfacePoint) +
                       ", floor=" + Vec(floorPoint) +
                       ", xzDistance=" + XZDistance(surfacePoint, floorPoint).ToString("F2"));
            return true;
        }

        private static void SortRaycastHitsByDistance(RaycastHit[] hits)
        {
            if (hits == null) return;

            Array.Sort(hits, delegate (RaycastHit a, RaycastHit b)
            {
                return a.distance.CompareTo(b.distance);
            });
        }

        private static bool IsXZInsideBounds(Vector3 p, Bounds b, float margin)
        {
            return p.x >= b.min.x - margin && p.x <= b.max.x + margin && p.z >= b.min.z - margin && p.z <= b.max.z + margin;
        }

        private static float EstimatePathLength(NavMeshPath path)
        {
            if (path == null || path.corners == null || path.corners.Length < 2) return 0f;
            float sum = 0f;
            for (int i = 1; i < path.corners.Length; i++) sum += Vector3.Distance(path.corners[i - 1], path.corners[i]);
            return sum;
        }

        private static float XZDistance(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x;
            float dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }

        private static GameObject CreateHiddenGotoPoint(Transform parent, Vector3 navPoint)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = BedGotoPointName;
            go.transform.position = navPoint + Vector3.up * (BedGotoCubeSize * 0.5f);
            go.transform.rotation = Quaternion.identity;
            go.transform.localScale = Vector3.one * BedGotoCubeSize;
            if (parent != null) go.transform.SetParent(parent, true);

            Renderer r = null;
            try { r = go.GetComponent<Renderer>(); } catch { }
            if (r != null) r.enabled = false;
            Collider c = null;
            try { c = go.GetComponent<Collider>(); } catch { }
            if (c != null) c.isTrigger = true;
            LogWarning("Hidden goto cube created: " + go.name + " at " + Vec(go.transform.position));
            return go;
        }

        private static void StartMitaWalkToGotoPointThenSit(Transform gotoTransform, string seatProxyName)
        {
            if (gotoTransform == null) { TryCallMitaSitByReflection(seatProxyName); return; }
            MitaPerson mita = null;
            try { mita = Object.FindObjectOfType<MitaPerson>(); } catch { }
            if (mita == null) { TryCallMitaSitByReflection(seatProxyName); return; }

            try { mita.MagnetOff(); } catch { }
            SetDialoguePatchesBool("IsAIControllingMovement", true);
            SetDialoguePatchesBool("IsInternalCall", true);
            try { mita.AiWalkToTarget(gotoTransform); }
            catch (Exception e)
            {
                LogWarning("AiWalkToTarget failed: " + e.GetType().Name + " " + e.Message);
                SetDialoguePatchesBool("IsInternalCall", false);
                TryCallMitaSitByReflection(seatProxyName);
                return;
            }
            SetDialoguePatchesBool("IsInternalCall", false);

            _pendingBedTopSit = true;
            _pendingBedTopSitMita = mita;
            _pendingBedTopSitGoto = gotoTransform;
            _pendingBedTopSitSeatProxyName = seatProxyName;
            _pendingBedTopSitDeadline = Time.time + BedGotoTimeout;
            _pendingBedTopSitNextRepathTime = Time.time + BedGotoRepathInterval;
            TryConsolePrint("米塔正在走向床边最近可达点，随后尝试坐下。");
        }

        private static void TickPendingBedSit()
        {
            if (!_pendingBedTopSit)
                return;

            MitaPerson mita = _pendingBedTopSitMita;
            Transform gotoT = _pendingBedTopSitGoto;

            if (mita == null || gotoT == null)
            {
                FinishPendingBedSit(true, "pending target lost; fallback sit");
                return;
            }

            Vector3 mitaPos = mita.transform.position;
            Vector3 targetPos = gotoT.position;

            float dist = XZDistance(mitaPos, targetPos);

            NavMeshAgent agent = null;
            try { agent = mita.GetComponent<NavMeshAgent>(); } catch { agent = null; }

            float remaining = 9999f;
            bool agentArrived = false;

            if (agent != null && agent.enabled)
            {
                try
                {
                    remaining = agent.remainingDistance;

                    // remainingDistance 在刚开始计算 path 时可能是 Infinity。
                    // 只有 path 不在 pending，且 remainingDistance 有意义时才参与判断。
                    if (!agent.pathPending &&
                        !float.IsInfinity(remaining) &&
                        !float.IsNaN(remaining) &&
                        remaining <= 0.55f)
                    {
                        agentArrived = true;
                    }
                }
                catch { }
            }

            bool arrived = dist <= BedGotoArriveDistance || agentArrived;
            bool timeout = Time.time >= _pendingBedTopSitDeadline;

            // 限频日志。v0.8.1 只能看到最后 timeout，无法判断为什么没有 arrived。
            if (Time.time >= _pendingBedTopSitLastLogTime + 1.0f)
            {
                _pendingBedTopSitLastLogTime = Time.time;
                LogWarning("Pending BedTopSit progress. dist=" + dist.ToString("F2") +
                           ", remaining=" + (remaining >= 9998f ? "<none>" : remaining.ToString("F2")) +
                           ", arrived=" + arrived +
                           ", timeout=" + timeout);
            }

            if (arrived)
            {
                FinishPendingBedSit(true, "arrived");
                return;
            }

            if (timeout)
            {
                // v0.9.1：
                //   v0.8.1 的日志显示米塔肉眼已经走到附近，但由于 transform root / NavMeshAgent remainingDistance
                //   没有满足严格阈值，最终 reason=timeout, shouldSit=False。
                //   这里 timeout 时也进入 Mita_sit，由 Mita_sit 自己再做 WalkToDockPoint / ManualApproach 安全校验。
                FinishPendingBedSit(true, "timeout; force bridge sit");
                return;
            }

            // v0.9.1：
            //   不再每 0.85 秒重复 AiWalkToTarget。
            //   重复调用会让游戏原生动作/动画队列反复被重启，表现为“吃饼干抬手动作来回抽搐”。
            //   路径交给第一次 AiWalkToTarget 和 NavMeshAgent 自己处理。
        }

        private static void FinishPendingBedSit(bool shouldSit, string reason)
        {
            string seatName = _pendingBedTopSitSeatProxyName;
            MitaPerson mita = _pendingBedTopSitMita;
            _pendingBedTopSit = false;
            _pendingBedTopSitMita = null;
            _pendingBedTopSitGoto = null;
            _pendingBedTopSitSeatProxyName = null;
            _pendingBridgeSit = false;
            _pendingBridgeTargetName = null;
            SetDialoguePatchesBool("IsAIControllingMovement", false);
            SetDialoguePatchesBool("IsInternalCall", false);
            if (mita != null)
            {
                TryCallMitaSharpStop(mita);
                try { mita.MagnetOff(); } catch { }

                NavMeshAgent agent = null;
                try { agent = mita.GetComponent<NavMeshAgent>(); } catch { agent = null; }
                if (agent != null && agent.enabled)
                {
                    try
                    {
                        agent.velocity = Vector3.zero;
                        agent.isStopped = true;
                        agent.ResetPath();
                    }
                    catch { }
                }
            }
            LogWarning("Pending BedTopSit finished. reason=" + reason + ", shouldSit=" + shouldSit);
            if (shouldSit && !string.IsNullOrEmpty(seatName)) TryCallMitaSitByReflection(seatName);
        }

        private static void TryCallMitaSharpStop(MitaPerson mita)
        {
            if (mita == null) return;
            try
            {
                MethodInfo m = AccessTools.Method(mita.GetType(), "AiShraplyStop");
                if (m != null) m.Invoke(mita, null);
            }
            catch { }
        }

        private static void ScheduleBridgeSit(string targetName)
        {
            try { Physics.SyncTransforms(); } catch { }

            _pendingBridgeSit = true;
            _pendingBridgeTargetName = targetName;
            _pendingBridgeFrame = Time.frameCount + 2;

            LogWarning("Bridge sit scheduled. target=" + targetName + ", frame=" + _pendingBridgeFrame);
        }

        private static void TickPendingBridgeSit()
        {
            if (!_pendingBridgeSit) return;
            if (Time.frameCount < _pendingBridgeFrame) return;

            string target = _pendingBridgeTargetName;

            _pendingBridgeSit = false;
            _pendingBridgeTargetName = null;

            try { Physics.SyncTransforms(); } catch { }

            LogWarning("Bridge sit executing after delay. target=" + target);
            TryCallMitaSitByReflection(target);
        }

        private static void TryCallMitaSitByReflection(string targetName)
        {
            try
            {
                // v0.9.1：
                // 不再使用 Harmony AccessTools.TypeByName。
                // 旧版本 TypeByName 会遍历许多 Unity/IL2CPP 程序集，导致大量 ReflectionTypeLoadException 警告。
                // 这里只做精确 Assembly.GetType；必要时只枚举 MakemitAGA/Mita 命名的插件程序集。
                Type t = FindTypeByNames(
                    "MakemitAGA.Mita_self.Mita_tools.Mita_sit",
                    "MakemitAGA.Mita_self.Mita_sit",
                    "MakemitAGA.Mita_sit",
                    "Mita_sit");

                if (t == null)
                {
                    if (!_mitaSitMissingWarned)
                    {
                        _mitaSitMissingWarned = true;
                        LogWarning("Mita_sit is not available yet. Standalone bridge created proxies only. target=" + targetName);
                        TryConsolePrint("<color=yellow>Mita_sit 当前不可用：已生成扫描代理和 GotoPoint，但不会执行坐下。</color>");
                    }
                    return;
                }

                MethodInfo m = t.GetMethod(
                    "Sit",
                    BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static,
                    null,
                    new Type[] { typeof(string) },
                    null);

                if (m == null)
                {
                    LogWarning("Found Mita_sit type but cannot find static Sit(string). type=" + t.FullName);
                    return;
                }

                LogWarning("Calling Mita_sit.Sit(\"" + targetName + "\") via bridge. type=" + t.FullName + ", assembly=" + t.Assembly.GetName().Name);
                TryConsolePrint("桥接调用 Mita_sit.Sit(" + targetName + ")");

                m.Invoke(null, new object[] { targetName });
            }
            catch (Exception e)
            {
                LogError("TryCallMitaSitByReflection exception: " + e);
            }
        }

        private static void SetDialoguePatchesBool(string fieldName, bool value)
        {
            try
            {
                Type t = FindTypeByNames(
                    "MakemitAGA.Mita_self.DialoguePatches",
                    "MakemitAGA.DialoguePatches",
                    "DialoguePatches");

                if (t == null) return;

                FieldInfo f = t.GetField(fieldName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static);
                if (f == null) return;

                f.SetValue(null, value);
            }
            catch { }
        }

        private static Type FindTypeByNames(params string[] names)
        {
            if (names == null || names.Length == 0)
                return null;

            Assembly[] assemblies = null;
            try { assemblies = AppDomain.CurrentDomain.GetAssemblies(); }
            catch { assemblies = null; }

            if (assemblies == null)
                return null;

            // 第一轮：精确 GetType，不枚举程序集类型，通常不会触发 ReflectionTypeLoadException。
            for (int a = 0; a < assemblies.Length; a++)
            {
                Assembly asm = assemblies[a];
                if (asm == null) continue;

                for (int i = 0; i < names.Length; i++)
                {
                    try
                    {
                        Type t = asm.GetType(names[i], false, false);
                        if (t != null)
                            return t;
                    }
                    catch { }
                }
            }

            // 第二轮：只枚举可能是用户插件的程序集，避免扫描 Assembly-CSharp / UnityEngine.* 刷屏。
            for (int a = 0; a < assemblies.Length; a++)
            {
                Assembly asm = assemblies[a];
                if (asm == null) continue;

                string asmName = string.Empty;
                try { asmName = asm.GetName().Name ?? string.Empty; } catch { }

                if (!LooksLikeUserMitaAssembly(asmName))
                    continue;

                Type[] types = GetTypesSafe(asm);
                if (types == null) continue;

                for (int tIndex = 0; tIndex < types.Length; tIndex++)
                {
                    Type t = types[tIndex];
                    if (t == null) continue;

                    for (int i = 0; i < names.Length; i++)
                    {
                        string wanted = names[i];

                        if (string.Equals(t.FullName, wanted, StringComparison.Ordinal) ||
                            string.Equals(t.Name, wanted, StringComparison.Ordinal))
                        {
                            return t;
                        }
                    }
                }
            }

            return null;
        }

        private static bool LooksLikeUserMitaAssembly(string asmName)
        {
            if (string.IsNullOrEmpty(asmName))
                return false;

            return asmName.IndexOf("Makemit", StringComparison.OrdinalIgnoreCase) >= 0 ||
                   asmName.IndexOf("Mita", StringComparison.OrdinalIgnoreCase) >= 0 ||
                   asmName.IndexOf("AGA", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private static Type[] GetTypesSafe(Assembly asm)
        {
            if (asm == null) return null;

            try
            {
                return asm.GetTypes();
            }
            catch (ReflectionTypeLoadException e)
            {
                return e.Types;
            }
            catch
            {
                return null;
            }
        }

        private static bool TryGetRendererBounds(GameObject root, out Bounds bounds)
        {
            bounds = new Bounds();
            if (root == null) return false;
            Renderer[] renderers = null;
            try { renderers = root.GetComponentsInChildren<Renderer>(true); } catch { }
            if (renderers == null || renderers.Length == 0) return false;
            return TryCalculateRendererBounds(renderers, out bounds);
        }

        private static int PickScanLayer()
        {
            int[] candidates = { 30, 29, 28, 27, 26, 25 };
            GameObject[] all = null;
            try { all = Object.FindObjectsOfType<GameObject>(); } catch { }
            if (all == null || all.Length == 0) return DefaultScanLayer;

            int bestLayer = DefaultScanLayer;
            int bestCount = int.MaxValue;
            for (int i = 0; i < candidates.Length; i++)
            {
                int layer = candidates[i];
                int count = 0;
                for (int j = 0; j < all.Length; j++)
                {
                    GameObject go = all[j];
                    if (go == null || go.layer != layer || IsOwnVisual(go)) continue;
                    Renderer r = null;
                    try { r = go.GetComponent<Renderer>(); } catch { }
                    if (r != null && r.enabled && go.activeInHierarchy) count++;
                }
                if (count < bestCount) { bestCount = count; bestLayer = layer; if (count == 0) break; }
            }
            return bestLayer;
        }

        private static GameObject GuessScanRoot(GameObject hitObject, Vector3 hitPoint)
        {
            if (hitObject == null) return null;
            Transform cursor = hitObject.transform;
            GameObject best = hitObject;
            float bestScore = float.MinValue;

            for (int depth = 0; depth <= 6 && cursor != null; depth++, cursor = cursor.parent)
            {
                string nodeName = cursor.name ?? string.Empty;
                if (IsSceneOrRoomRootName(nodeName)) break;
                Renderer[] renderers = null;
                try { renderers = cursor.GetComponentsInChildren<Renderer>(true); } catch { }
                if (renderers == null || renderers.Length == 0) continue;
                Bounds b;
                if (!TryCalculateRendererBounds(renderers, out b)) continue;
                float size = b.size.magnitude;
                float volume = b.size.x * b.size.y * b.size.z;
                if (size > 6.0f || volume > 18.0f) continue;

                float score = 0f;
                if (b.Contains(hitPoint)) score += 120f;
                score -= b.SqrDistance(hitPoint) * 30f;
                score -= Mathf.Max(0f, size - 2.5f) * 8f;
                score -= Mathf.Max(0f, volume - 4.0f) * 2f;
                if (IsBedLikeName(nodeName)) score += 55f;
                if (IsSeatLikeName(nodeName)) score += 35f;
                if (IsTableLikeName(nodeName)) score += 25f;
                score -= depth * 2f;
                if (cursor.gameObject == hitObject && IsLikelyFurnitureName(nodeName)) score += 40f;
                if (score > bestScore) { bestScore = score; best = cursor.gameObject; }
            }

            LogWarning("GuessScanRoot: hitObject=" + hitObject.name + ", selected=" + best.name + ", score=" + bestScore.ToString("F2"));
            return best;
        }

        private static bool IsSceneOrRoomRootName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            return n == "bedroom" || n.Contains("room") || n.Contains("scene") || n.Contains("level") || n.Contains("location") || n.Contains("environment") || n.Contains("interior");
        }

        private static bool IsLikelyFurnitureName(string name) { return IsBedLikeName(name) || IsSeatLikeName(name) || IsTableLikeName(name); }
        private static bool IsBedLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            if (n.Contains("bedroom")) return false;
            return n == "bed" || n.StartsWith("bed_") || n.EndsWith("_bed") || n.Contains("mattress") || n.Contains("blanket") || n.Contains("futon");
        }
        private static bool IsSeatLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            return n.Contains("chair") || n.Contains("stool") || n.Contains("sofa") || n.Contains("seat") || n.Contains("bench") || n.Contains("couch");
        }
        private static bool IsTableLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            return n.Contains("table") || n.Contains("desk") || n.Contains("counter");
        }

        private static bool TryCalculateRendererBounds(Renderer[] renderers, out Bounds bounds)
        {
            bounds = new Bounds();
            bool has = false;
            for (int i = 0; i < renderers.Length; i++)
            {
                Renderer r = renderers[i];
                if (r == null || IsOwnVisual(r.gameObject)) continue;
                try
                {
                    if (!has) { bounds = r.bounds; has = true; }
                    else bounds.Encapsulate(r.bounds);
                }
                catch { }
            }
            return has;
        }

        private static bool TryRaycastScene(Ray ray, out RaycastHit bestHit)
        {
            bestHit = new RaycastHit();
            RaycastHit[] hits = null;
            try { hits = Physics.RaycastAll(ray, RayDistance, -1, QueryTriggerInteraction.Ignore); } catch { }
            if (hits == null || hits.Length == 0) return false;
            bool found = false;
            float bestDistance = float.MaxValue;
            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit h = hits[i];
                if (h.collider == null || h.distance < 0.03f || IsOwnVisual(h.collider.gameObject)) continue;
                if (h.distance < bestDistance) { bestDistance = h.distance; bestHit = h; found = true; }
            }
            return found;
        }

        private static void AssignLayerRecursive(GameObject root, int layer, List<LayerBackup> backups)
        {
            if (root == null) return;
            backups.Add(new LayerBackup { go = root, layer = root.layer });
            root.layer = layer;
            Transform t = root.transform;
            if (t == null) return;
            for (int i = 0; i < t.childCount; i++)
            {
                Transform child = t.GetChild(i);
                if (child != null) AssignLayerRecursive(child.gameObject, layer, backups);
            }
        }

        private static void RestoreLayers(List<LayerBackup> backups)
        {
            if (backups == null) return;
            for (int i = 0; i < backups.Count; i++)
            {
                try { if (backups[i].go != null) backups[i].go.layer = backups[i].layer; } catch { }
            }
        }

        private static void CreateDebugBox(Bounds b, Transform parent)
        {
            GameObject root = new GameObject("ScanVolume_DebugBox");
            root.transform.SetParent(parent, true);
            Vector3 c = b.center;
            Vector3 e = b.extents;
            Vector3[] p =
            {
                c + new Vector3(-e.x, -e.y, -e.z), c + new Vector3( e.x, -e.y, -e.z),
                c + new Vector3(-e.x, -e.y,  e.z), c + new Vector3( e.x, -e.y,  e.z),
                c + new Vector3(-e.x,  e.y, -e.z), c + new Vector3( e.x,  e.y, -e.z),
                c + new Vector3(-e.x,  e.y,  e.z), c + new Vector3( e.x,  e.y,  e.z),
            };
            CreateLine(p[0], p[1], root.transform); CreateLine(p[0], p[2], root.transform); CreateLine(p[3], p[1], root.transform); CreateLine(p[3], p[2], root.transform);
            CreateLine(p[4], p[5], root.transform); CreateLine(p[4], p[6], root.transform); CreateLine(p[7], p[5], root.transform); CreateLine(p[7], p[6], root.transform);
            CreateLine(p[0], p[4], root.transform); CreateLine(p[1], p[5], root.transform); CreateLine(p[2], p[6], root.transform); CreateLine(p[3], p[7], root.transform);
        }

        private static void CreateLine(Vector3 a, Vector3 b, Transform parent)
        {
            Vector3 dir = b - a;
            float len = dir.magnitude;
            if (len < 0.001f) return;
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = "DebugBox_Line";
            go.transform.position = a + dir * 0.5f;
            go.transform.rotation = Quaternion.FromToRotation(Vector3.up, dir.normalized);
            go.transform.localScale = new Vector3(0.01f, len * 0.5f, 0.01f);
            go.transform.SetParent(parent, true);
            SetRendererMaterial(go, _cyanMat, true);
            DisableCollider(go);
        }

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
                        LogWarning("Loaded EyeDepthReplacement shader from mita_actions: " + shader.name);
                    }
                }
                catch (Exception e) { LogWarning("EyeDepthReplacement shader AB load failed: " + e.Message); }

                try
                {
                    Material mat;
                    if (ICallAssetBundleLoader.TryLoadMaterialByName(path, MaterialName, out mat) && mat != null)
                    {
                        _depthToEyeMat = mat;
                        LogWarning("Loaded depth material from mita_actions: " + mat.name);
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
                    if (fallback != null) { _eyeDepthReplacementShader = fallback; LogWarning("Loaded EyeDepthReplacement shader from Shader.Find: " + fallback.name); }
                }
                catch { }
            }

            if (_depthToEyeMat == null)
            {
                try
                {
                    Shader fallback = Shader.Find(ShaderName);
                    if (fallback != null) { _depthToEyeMat = new Material(fallback); LogWarning("Loaded DepthToEye material from Shader.Find fallback."); }
                }
                catch { }
            }

            if (_eyeDepthReplacementShader == null) LogWarning("EyeDepthReplacement shader unavailable. Depth capture will probably fail.");
        }

        public static void ClearAll()
        {
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
        }

        public static void RecreateCrosshairFromConsole()
        {
            DestroyObject(_crosshairRoot);
            _crosshairRoot = null;
            _crosshairCamera = null;
            EnsureCrosshair();
        }

        public static void SetDebugRenderersVisible(bool visible)
        {
            for (int i = _debugRenderers.Count - 1; i >= 0; i--)
            {
                Renderer r = _debugRenderers[i];
                if (r == null) { _debugRenderers.RemoveAt(i); continue; }
                try { r.enabled = visible; } catch { }
            }
            LogWarning("SetDebugRenderersVisible(" + visible + ") applied to " + _debugRenderers.Count + " renderers.");
        }

        private static void RegisterDebugRenderer(Renderer r)
        {
            if (r != null) _debugRenderers.Add(r);
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
                _crosshairRoot.SetActive(true);
                return;
            }

            DestroyObject(_crosshairRoot);
            EnsureMaterials();
            _crosshairRoot = new GameObject("TopSurfaceSeatProxy_Crosshair");
            _crosshairRoot.transform.SetParent(cam.transform, false);
            _crosshairRoot.transform.localPosition = new Vector3(0f, 0f, Mathf.Max(CrosshairDepth, cam.nearClipPlane + 0.15f));
            _crosshairRoot.transform.localRotation = Quaternion.identity;
            _crosshairCamera = cam;

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
            _heightfieldMat = MakeMaterial(new Color(1f, 0f, 1f, 0.32f), true);
            _seatBoxMat = MakeMaterial(new Color(0f, 1f, 1f, 0.24f), true);
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
