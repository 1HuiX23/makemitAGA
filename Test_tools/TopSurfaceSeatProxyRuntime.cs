/*
 * =================================================================================================
 * TopSurfaceSeatProxyRuntime.cs
 * =================================================================================================
 *
 * 这个文件负责核心逻辑：
 *
 *   1. 从屏幕中心 Raycast 选中目标物体。
 *   2. 临时把目标物体切到专用扫描 Layer。
 *   3. 用隐藏正交相机从目标上方扫描。
 *   4. 使用 EyeDepthReplacement shader 直接渲染 scanCam 自己的 eye depth。
 *   5. 反投影 depth 到世界坐标。
 *   6. 按 XZ bin 生成 top surface heightfield。
 *   7. 使用 surfaceMask 区分真实目标表面和空区域。
 *   8. 只在 surfaceMask 内生成 heightfield mesh，不再把床外空区域补成大平面。
 *   9. 生成 FakeCollider：
 *        - FixedSize: FakeCollider(0.5,0.5,0.08) / FakeCollider(1,1,0.08)
 *        - Top:       FakeCollider(Top)
 *
 * 为什么要这么写：
 * -------------------------------------------------------------------------------------------------
 * 早期方案用 _CameraDepthTexture + Graphics.Blit 读取深度，但在隐藏扫描相机里经常读到主相机
 * 或上一相机残留的全局 depth，导致生成一团和床面对不上的奇怪曲面。
 *
 * 现在主路径改成：
 *
 *   scanCam.RenderWithShader(EyeDepthReplacement, "")
 *
 * replacement shader 在渲染目标物体时直接输出 eye depth，因此 depth 来源就是 scanCam 本身。
 *
 * 已经踩过的坑：
 * -------------------------------------------------------------------------------------------------
 * 1. Bedroom 会被误判成 Bed。
 *    因为 "Bedroom" 包含 "Bed"。所以 GuessScanRoot 里明确排除 room/scene/environment 根节点。
 *
 * 2. 床外空区域被 FillMissingHeights 补成一大片平面。
 *    所以现在引入 surfaceMask，只在真正有目标表面或目标表面邻域的区域补洞和平滑。
 *
 * 3. 默认 0.5x0.5 看起来像 1x1。
 *    原因是扫描 volume 以前固定 1.1x1.1，调试 heightfield 很大，容易误认为 collider 很大。
 *    现在 FixedSize 模式会根据请求尺寸动态缩小扫描范围。
 *
 * 4. ts_scan_top 看起来也像 1x1。
 *    原因是 Top 模式以前仍然使用固定 1.1x1.1 扫描盒。
 *    现在 Top 模式使用 targetBounds 的 XZ 范围扫描整个目标上方。
 *
 * 未来设计：
 * -------------------------------------------------------------------------------------------------
 * FakeCollider(Top) 不是完整 whole。
 * 它只代表 top-view 可见表面的上方代理，适合坐下、躺下、爬上床、脚底 IK 等任务。
 *
 * 真正 FakeCollider(whole) 应该未来单独做：
 *   多视角 depth -> voxel / TSDF / occupancy -> marching cubes
 * 或：
 *   surfaceMask tiled boxes / 多 BoxCollider 拼接
 *
 * 当前 MVP：
 *   先把“可见 top surface -> 可用动作代理”做好。
 *
 * =================================================================================================
 */

using System;
using System.Collections.Generic;
using System.IO;
using BepInEx;
using BepInEx.Logging;
using UnityEngine;
using Object = UnityEngine.Object;

namespace TopSurfaceSeatProxyTest
{
    internal enum FakeColliderMode
    {
        FixedSize,

        // 注意：Top 不是完整 whole。
        // 它代表“从目标上方看到的可见 top surface 区域”。
        Top
    }

    internal struct FakeColliderRequest
    {
        public FakeColliderMode mode;
        public float width;
        public float depth;
        public float height;
        public bool clampToSupportedSurface;

        public static FakeColliderRequest DefaultSeat()
        {
            return new FakeColliderRequest
            {
                mode = FakeColliderMode.FixedSize,
                width = 0.50f,
                depth = 0.50f,
                height = 0.08f,
                clampToSupportedSurface = true
            };
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
        private static float _nextHeartbeatTime;
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

        private const string BundleFileName = "depthseat.test";

        private const string MaterialName = "DepthToEye_Mat";
        private const string ShaderName = "Hidden/DepthSeat/DepthToEye";

        private const string EyeDepthReplacementShaderAssetName = "DepthSeat_EyeDepthReplacement";
        private const string EyeDepthReplacementShaderFindName = "Hidden/DepthSeat/EyeDepthReplacement";

        private const int DefaultScanLayer = 30;

        private const int CaptureSize = 256;
        private const int MeshGrid = 64;

        // FixedSize 模式不再固定扫 1.1m。
        // 默认 0.5 collider 会扫约 0.68m，足够估计高度，又不会让调试面看起来像 1x1。
        private const float FixedScanPadding = 0.18f;
        private const float FixedScanMinSize = 0.62f;
        private const float FixedScanMaxSize = 1.50f;

        // Top 模式扫描整个 targetBounds 的 XZ。
        // 加一点 padding 是为了防止边缘采样漏掉，但不能太大，否则床外空区域太多。
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

        private const bool AddHeightfieldMeshCollider = false;

        private const string SeatProxyName = "DepthSeatProxy_Bed";

        private const float DefaultSeatProxyWidth = 0.50f;
        private const float DefaultSeatProxyDepth = 0.50f;
        private const float DefaultSeatProxyHeight = 0.08f;

        private const float MinFakeColliderSupportCoverage = 0.55f;
        private const float MinFakeColliderSize = 0.20f;

        private const bool SeatProxyRendererVisibleByDefault = true;
        private const bool EnableOldDepthToEyeFallback = true;
        private const bool EnableDepthNormalsFallback = true;

        private const float CrosshairDepth = 0.55f;
        private const float RayDistance = 30.0f;

        private struct LayerBackup
        {
            public GameObject go;
            public int layer;
        }

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

        private enum DepthPath
        {
            None,
            EyeDepthReplacement,
            DepthToEyeMaterial,
            BuiltInDepthNormalsFallback
        }

        public static void Init(ManualLogSource log)
        {
            _log = log;
        }

        public static void TickFromPatch()
        {
            _tickCount++;

            if (!_firstTickLogged)
            {
                _firstTickLogged = true;
                LogWarning("First GameController.Update tick received.");
            }

            if (Time.time >= _nextHeartbeatTime)
            {
                _nextHeartbeatTime = Time.time + 5.0f;
                LogWarning("Heartbeat. tick=" + _tickCount + ", time=" + Time.time.ToString("F1"));
            }

            try
            {
                EnsureMaterials();
                EnsureCrosshair();

                if (Input.GetKeyDown(KeyCode.Tab) || Input.GetKeyDown(KeyCode.F9))
                {
                    ScanFromScreenCenter("key", FakeColliderRequest.DefaultSeat());
                }

                if (Input.GetKeyDown(KeyCode.Delete))
                {
                    ClearAll();
                    LogWarning("DELETE detected. Cleared visuals and proxies.");
                }
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
            if (playerCam == null)
            {
                LogError("No usable camera found.");
                return;
            }

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

            if (targetRoot == null)
            {
                LogError("Cannot determine target root.");
                return;
            }

            ClearProxyOnly();

            Bounds targetBounds;
            bool hasTargetBounds = TryGetRendererBounds(targetRoot, out targetBounds);

            Bounds volume = BuildScanVolume(hit.point, hasTargetBounds, targetBounds, request);
            int scanLayer = PickScanLayer();

            LogWarning("Scan setup: hit=" + Vec(hit.point) +
                       ", volumeCenter=" + Vec(volume.center) +
                       ", volumeSize=" + Vec(volume.size) +
                       ", scanLayer=" + scanLayer +
                       ", colliderMode=" + request.mode +
                       ", requestSize=(" + request.width.ToString("F2") + "," + request.depth.ToString("F2") + "," + request.height.ToString("F2") + ")" +
                       ", targetBounds=" + (hasTargetBounds ? (Vec(targetBounds.center) + " / " + Vec(targetBounds.size)) : "<none>"));

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
                    LogError("Top surface scan failed. vertices=" + vertices.Count +
                             ", triangles=" + (triangles.Count / 3) +
                             ", stats.valid=" + stats.valid +
                             ", depthPath=" + depthPath);
                    TryConsolePrint("TopSurfaceSeatProxy: scan failed.");
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

                string msg =
                    "Top surface scan done from " + source +
                    " | target=" + targetRoot.name +
                    " | hitObject=" + (hitObject != null ? hitObject.name : "<null>") +
                    " | depthPath=" + depthPath +
                    " | colliderMode=" + request.mode +
                    " | center=" + Vec(stats.center) +
                    " | seatY=" + stats.medianY.ToString("F3") +
                    " | minY=" + stats.minY.ToString("F3") +
                    " | maxY=" + stats.maxY.ToString("F3") +
                    " | rawPixels=" + stats.rawPixelValidCount +
                    " | rawCells=" + stats.rawCellValidCount +
                    " | keptCells=" + stats.keptCellCount +
                    " | surfaceCells=" + stats.surfaceCellCount +
                    " | removed=" + stats.removedOutliers +
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

                try
                {
                    if (scanCam != null)
                        scanCam.targetTexture = null;
                }
                catch { }

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
                // Top 模式：用整个目标物体的 XZ bounds。
                // 这就是你提出的 FakeCollider(Top)：不是 whole，只是 top-view 上方扫描。
                centerX = targetBounds.center.x;
                centerZ = targetBounds.center.z;

                sizeX = Mathf.Clamp(targetBounds.size.x + TopScanPadding, FixedScanMinSize, TopScanMaxSize);
                sizeZ = Mathf.Clamp(targetBounds.size.z + TopScanPadding, FixedScanMinSize, TopScanMaxSize);
            }
            else
            {
                // FixedSize 模式：扫描范围跟 collider request 走。
                // 默认 0.5x0.5 会扫约 0.68x0.68，不再固定 1.1x1.1。
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

                if (maxY <= minY + 0.08f)
                    maxY = minY + 0.35f;
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

            Vector3 center = new Vector3(centerX, (minY + maxY) * 0.5f, centerZ);
            Vector3 size = new Vector3(sizeX, maxY - minY, sizeZ);

            return new Bounds(center, size);
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

        private static bool CaptureTopViewToHeightfield(
            Camera scanCam,
            Bounds volume,
            List<Vector3> vertices,
            List<int> triangles,
            out HeightfieldStats stats,
            out DepthPath depthPath)
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
            else
            {
                LogWarning("EyeDepthReplacement shader unavailable; trying fallback paths.");
            }

            if (EnableOldDepthToEyeFallback && _depthToEyeMat != null)
            {
                if (CaptureDepthToEyePath(scanCam, volume, vertices, triangles, out stats))
                {
                    depthPath = DepthPath.DepthToEyeMaterial;
                    LogWarning("Depth capture path: old AssetBundle/DepthToEye material.");
                    return true;
                }

                vertices.Clear();
                triangles.Clear();
                stats = new HeightfieldStats();
                LogWarning("Old DepthToEye material path failed.");
            }

            if (EnableDepthNormalsFallback)
            {
                if (CaptureDepthNormalsFallback(scanCam, volume, vertices, triangles, out stats))
                {
                    depthPath = DepthPath.BuiltInDepthNormalsFallback;
                    LogWarning("Depth capture path: built-in DepthNormals fallback.");
                    return true;
                }
            }

            return false;
        }

        private static bool CaptureReplacementEyeDepthPath(
            Camera scanCam,
            Bounds volume,
            List<Vector3> vertices,
            List<int> triangles,
            out HeightfieldStats stats)
        {
            stats = new HeightfieldStats();

            if (_eyeDepthReplacementShader == null)
                return false;

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
                    LogWarning("ARGBFloat unsupported; using ARGBHalf.");
                }

                depthRT = new RenderTexture(CaptureSize, CaptureSize, 24, depthFormat);
                depthRT.name = "TopSurface_ReplacementEyeDepthRT";
                depthRT.Create();

                scanCam.targetTexture = depthRT;

                // 核心：直接从 scanCam 渲染 eye depth，避免 _CameraDepthTexture 全局污染。
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

        private static bool CaptureDepthToEyePath(
            Camera scanCam,
            Bounds volume,
            List<Vector3> vertices,
            List<int> triangles,
            out HeightfieldStats stats)
        {
            stats = new HeightfieldStats();

            RenderTexture colorRT = null;
            RenderTexture depthRT = null;
            Texture2D depthTex = null;

            try
            {
                colorRT = new RenderTexture(CaptureSize, CaptureSize, 24, RenderTextureFormat.ARGB32);
                colorRT.name = "TopSurface_ColorRT";
                colorRT.Create();

                RenderTextureFormat depthFormat = RenderTextureFormat.ARGBFloat;
                TextureFormat texFormat = TextureFormat.RGBAFloat;

                if (!SystemInfo.SupportsRenderTextureFormat(RenderTextureFormat.ARGBFloat))
                {
                    depthFormat = RenderTextureFormat.ARGBHalf;
                    texFormat = TextureFormat.RGBAHalf;
                    LogWarning("ARGBFloat unsupported; using ARGBHalf.");
                }

                depthRT = new RenderTexture(CaptureSize, CaptureSize, 0, depthFormat);
                depthRT.name = "TopSurface_EyeDepthRT";
                depthRT.Create();

                scanCam.targetTexture = colorRT;
                scanCam.Render();

                // 旧路径：依赖 _CameraDepthTexture。
                // 保留它只是为了 fallback，不再作为正式主路径判断。
                Graphics.Blit(colorRT, depthRT, _depthToEyeMat);

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

        private static bool CaptureDepthNormalsFallback(
            Camera scanCam,
            Bounds volume,
            List<Vector3> vertices,
            List<int> triangles,
            out HeightfieldStats stats)
        {
            stats = new HeightfieldStats();

            Shader depthNormalsShader = null;
            try { depthNormalsShader = Shader.Find("Hidden/Internal-DepthNormalsTexture"); } catch { depthNormalsShader = null; }

            if (depthNormalsShader == null)
            {
                LogWarning("DepthNormals fallback unavailable: Hidden/Internal-DepthNormalsTexture not found.");
                return false;
            }

            RenderTexture dnRT = null;
            Texture2D dnTex = null;
            Texture2D eyeTex = null;

            try
            {
                dnRT = new RenderTexture(CaptureSize, CaptureSize, 24, RenderTextureFormat.ARGB32);
                dnRT.name = "TopSurface_DepthNormalsRT";
                dnRT.Create();

                scanCam.targetTexture = dnRT;
                scanCam.RenderWithShader(depthNormalsShader, "RenderType");

                dnTex = new Texture2D(CaptureSize, CaptureSize, TextureFormat.RGBA32, false, true);

                RenderTexture old = RenderTexture.active;
                try
                {
                    RenderTexture.active = dnRT;
                    dnTex.ReadPixels(new Rect(0, 0, CaptureSize, CaptureSize), 0, 0, false);
                    dnTex.Apply(false, false);
                }
                finally
                {
                    RenderTexture.active = old;
                }

                TextureFormat eyeFormat = TextureFormat.RGBAFloat;
                try
                {
                    if (!SystemInfo.SupportsTextureFormat(TextureFormat.RGBAFloat))
                        eyeFormat = TextureFormat.RGBAHalf;
                }
                catch
                {
                    eyeFormat = TextureFormat.RGBAHalf;
                }

                eyeTex = new Texture2D(CaptureSize, CaptureSize, eyeFormat, false, true);

                int nonFar = 0;

                for (int y = 0; y < CaptureSize; y++)
                {
                    for (int x = 0; x < CaptureSize; x++)
                    {
                        Color c = dnTex.GetPixel(x, y);
                        float depth01 = DecodeFloatRG(c.b, c.a);
                        float eye = Mathf.Lerp(scanCam.nearClipPlane, scanCam.farClipPlane, depth01);

                        if (depth01 > 0.0001f && depth01 < 0.9990f) nonFar++;

                        eyeTex.SetPixel(x, y, new Color(eye, 0f, 0f, 1f));
                    }
                }

                eyeTex.Apply(false, false);

                LogWarning("DepthNormals fallback decoded nonFarPixels=" + nonFar);

                return BuildTopHeightfieldMesh(scanCam, eyeTex, volume, vertices, triangles, out stats);
            }
            catch (Exception e)
            {
                LogError("CaptureDepthNormalsFallback exception: " + e);
                return false;
            }
            finally
            {
                if (scanCam != null) scanCam.targetTexture = null;
                ReleaseRT(dnRT);
                DestroyObject(dnTex);
                DestroyObject(eyeTex);
            }
        }

        private static float DecodeFloatRG(float x, float y)
        {
            return x + y * (1f / 255f);
        }

        private static bool BuildTopHeightfieldMesh(
            Camera cam,
            Texture2D depthTex,
            Bounds volume,
            List<Vector3> allVertices,
            List<int> allTriangles,
            out HeightfieldStats stats)
        {
            stats = new HeightfieldStats();

            bool[,] valid = new bool[MeshGrid, MeshGrid];
            float[,] height = new float[MeshGrid, MeshGrid];

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    valid[x, z] = false;
                    height[x, z] = volume.center.y;
                }
            }

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

                    // top-view 扫描中，同一 XZ cell 保留最高可见表面。
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
            float initialMedian = pixelHeights[pixelHeights.Count / 2];

            int rawCellValid = 0;
            int removed = 0;
            int keptAfterOutlier = 0;

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!valid[x, z]) continue;

                    rawCellValid++;

                    if (Mathf.Abs(height[x, z] - initialMedian) > MaxHeightDeviationFromMedian)
                    {
                        valid[x, z] = false;
                        removed++;
                    }
                    else
                    {
                        keptAfterOutlier++;
                    }
                }
            }

            if (keptAfterOutlier < 16)
            {
                LogWarning("Top heightfield has too few kept cells after outlier removal: kept=" +
                           keptAfterOutlier + ", rawCells=" + rawCellValid + ", rawPixels=" + rawPixelValid);
                return false;
            }

            bool[,] surfaceMask = BuildSurfaceMask(valid);

            FillMissingHeightsMasked(surfaceMask, valid, height, initialMedian);
            SmoothHeightsMasked(surfaceMask, height, HeightSmoothStrength);

            List<float> finalHeights = new List<float>();
            List<float> seatPatchHeights = new List<float>();

            float minY = float.MaxValue;
            float maxY = float.MinValue;
            float sumY = 0f;
            int surfaceCellCount = 0;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float worldZ = Mathf.Lerp(minZ, maxZ, tz);

                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!surfaceMask[x, z]) continue;

                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float worldX = Mathf.Lerp(minX, maxX, tx);

                    float y = height[x, z];

                    surfaceCellCount++;
                    finalHeights.Add(y);
                    sumY += y;

                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;

                    if (Mathf.Abs(worldX - volume.center.x) <= DefaultSeatProxyWidth * 0.5f &&
                        Mathf.Abs(worldZ - volume.center.z) <= DefaultSeatProxyDepth * 0.5f)
                    {
                        seatPatchHeights.Add(y);
                    }
                }
            }

            if (surfaceCellCount < 8)
            {
                LogWarning("Surface mask has too few cells: " + surfaceCellCount);
                return false;
            }

            finalHeights.Sort();
            seatPatchHeights.Sort();

            float medianY = seatPatchHeights.Count > 0
                ? seatPatchHeights[seatPatchHeights.Count / 2]
                : finalHeights[finalHeights.Count / 2];

            float avgY = sumY / finalHeights.Count;

            int baseIndex = allVertices.Count;
            int[,] index = new int[MeshGrid, MeshGrid];

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                    index[x, z] = -1;
            }

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float worldZ = Mathf.Lerp(minZ, maxZ, tz);

                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!surfaceMask[x, z]) continue;

                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float worldX = Mathf.Lerp(minX, maxX, tx);

                    Vector3 v = new Vector3(worldX, height[x, z] + VisualLift, worldZ);

                    index[x, z] = allVertices.Count;
                    allVertices.Add(v);
                }
            }

            for (int z = 0; z < MeshGrid - 1; z++)
            {
                for (int x = 0; x < MeshGrid - 1; x++)
                {
                    int i00 = index[x, z];
                    int i10 = index[x + 1, z];
                    int i01 = index[x, z + 1];
                    int i11 = index[x + 1, z + 1];

                    if (i00 >= 0 && i01 >= 0 && i10 >= 0)
                    {
                        if (CanConnectHeightTriangle(height[x, z], height[x, z + 1], height[x + 1, z]))
                            AddTriangleFacingUp(allTriangles, allVertices, i00, i01, i10);
                    }

                    if (i11 >= 0 && i10 >= 0 && i01 >= 0)
                    {
                        if (CanConnectHeightTriangle(height[x + 1, z + 1], height[x + 1, z], height[x, z + 1]))
                            AddTriangleFacingUp(allTriangles, allVertices, i11, i10, i01);
                    }
                }
            }

            stats.valid = true;
            stats.rawPixelValidCount = rawPixelValid;
            stats.rawCellValidCount = rawCellValid;
            stats.keptCellCount = keptAfterOutlier;
            stats.surfaceCellCount = surfaceCellCount;
            stats.removedOutliers = removed;
            stats.medianY = medianY;
            stats.minY = minY;
            stats.maxY = maxY;
            stats.center = new Vector3(volume.center.x, medianY, volume.center.z);
            stats.volume = volume;
            stats.surfaceMask = surfaceMask;
            stats.heights = height;

            LogWarning(
                "Top heightfield mesh built. rawPixels=" + stats.rawPixelValidCount +
                ", rawCells=" + stats.rawCellValidCount +
                ", keptCells=" + stats.keptCellCount +
                ", surfaceCells=" + stats.surfaceCellCount +
                ", removedOutliers=" + stats.removedOutliers +
                ", seatY=" + medianY.ToString("F3") +
                ", avgY=" + avgY.ToString("F3") +
                ", minY=" + minY.ToString("F3") +
                ", maxY=" + maxY.ToString("F3") +
                ", vertices=" + (allVertices.Count - baseIndex) +
                ", triangles=" + (allTriangles.Count / 3));

            return true;
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
            {
                for (int x = 0; x < MeshGrid; x++)
                    mask[x, z] = valid[x, z];
            }

            for (int i = 0; i < SurfaceMaskDilateIterations; i++)
                mask = DilateMask(mask);

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
                    if (src[x, z])
                    {
                        dst[x, z] = true;
                        continue;
                    }

                    bool near = false;

                    for (int dz = -1; dz <= 1 && !near; dz++)
                    {
                        for (int dx = -1; dx <= 1 && !near; dx++)
                        {
                            int nx = x + dx;
                            int nz = z + dz;

                            if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;

                            if (src[nx, nz])
                                near = true;
                        }
                    }

                    dst[x, z] = near;
                }
            }

            return dst;
        }

        private static void RemoveSmallIslands(bool[,] mask, int minCells)
        {
            bool[,] visited = new bool[MeshGrid, MeshGrid];
            List<Vector2Int> component = new List<Vector2Int>();

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!mask[x, z] || visited[x, z]) continue;

                    component.Clear();
                    FloodCollect(mask, visited, x, z, component);

                    if (component.Count < minCells)
                    {
                        for (int i = 0; i < component.Count; i++)
                        {
                            Vector2Int p = component[i];
                            mask[p.x, p.y] = false;
                        }
                    }
                }
            }
        }

        private static void FloodCollect(bool[,] mask, bool[,] visited, int startX, int startZ, List<Vector2Int> output)
        {
            Queue<Vector2Int> q = new Queue<Vector2Int>();

            visited[startX, startZ] = true;
            q.Enqueue(new Vector2Int(startX, startZ));

            while (q.Count > 0)
            {
                Vector2Int p = q.Dequeue();
                output.Add(p);

                for (int dz = -1; dz <= 1; dz++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        if (dx == 0 && dz == 0) continue;

                        int nx = p.x + dx;
                        int nz = p.y + dz;

                        if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;
                        if (visited[nx, nz]) continue;
                        if (!mask[nx, nz]) continue;

                        visited[nx, nz] = true;
                        q.Enqueue(new Vector2Int(nx, nz));
                    }
                }
            }
        }

        private static void FillMissingHeightsMasked(bool[,] surfaceMask, bool[,] valid, float[,] height, float fallback)
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

                        if (!surfaceMask[x, z]) continue;
                        if (valid[x, z]) continue;

                        float sum = 0f;
                        int count = 0;

                        for (int dz = -1; dz <= 1; dz++)
                        {
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                if (dx == 0 && dz == 0) continue;

                                int nx = x + dx;
                                int nz = z + dz;

                                if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;
                                if (!surfaceMask[nx, nz]) continue;
                                if (!valid[nx, nz]) continue;

                                sum += height[nx, nz];
                                count++;
                            }
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
                {
                    for (int x = 0; x < MeshGrid; x++)
                    {
                        valid[x, z] = nextValid[x, z];
                        height[x, z] = nextHeight[x, z];
                    }
                }

                if (!changed) break;
            }

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!surfaceMask[x, z]) continue;

                    if (!valid[x, z])
                    {
                        valid[x, z] = true;
                        height[x, z] = fallback;
                    }
                }
            }
        }

        private static void SmoothHeightsMasked(bool[,] surfaceMask, float[,] height, float strength)
        {
            if (strength <= 0f) return;

            float[,] copy = new float[MeshGrid, MeshGrid];

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!surfaceMask[x, z])
                    {
                        copy[x, z] = height[x, z];
                        continue;
                    }

                    float sum = 0f;
                    int count = 0;

                    for (int dz = -1; dz <= 1; dz++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            int nx = x + dx;
                            int nz = z + dz;

                            if (nx < 0 || nx >= MeshGrid || nz < 0 || nz >= MeshGrid) continue;
                            if (!surfaceMask[nx, nz]) continue;

                            sum += height[nx, nz];
                            count++;
                        }
                    }

                    float avg = count > 0 ? sum / count : height[x, z];
                    copy[x, z] = Mathf.Lerp(height[x, z], avg, strength);
                }
            }

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                    height[x, z] = copy[x, z];
            }
        }

        private static bool TryReadDepthPoint(Camera cam, Texture2D depthTex, int px, int py, out DepthPoint p)
        {
            p = new DepthPoint();
            p.px = px;
            p.py = py;
            p.valid = false;

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

            if (Mathf.Abs(denom) < 0.0001f)
                denom = denom >= 0f ? 0.0001f : -0.0001f;

            // ScreenPointToRay 的 origin 通常在 near plane，而不是 camera transform.position。
            // eyeDepth 是 view-space 深度，所以需要减掉 originEye。
            float originEye = Vector3.Dot(ray.origin - cam.transform.position, cam.transform.forward);
            float rayDistance = (eyeDepth - originEye) / denom;

            if (rayDistance < 0f)
                rayDistance = eyeDepth / Mathf.Abs(denom);

            return ray.origin + ray.direction * rayDistance;
        }

        private static GameObject BuildHeightfieldMesh(Transform parent, List<Vector3> vertices, List<int> triangles)
        {
            GameObject go = new GameObject("TopSurfaceSeatProxy_HeightfieldMesh");
            go.transform.SetParent(parent, true);
            go.transform.position = Vector3.zero;
            go.transform.rotation = Quaternion.identity;
            go.transform.localScale = Vector3.one;

            Mesh mesh = new Mesh();
            mesh.name = "TopSurfaceSeatProxy_HeightfieldMeshData";

            mesh.SetVertices(ToIl2CppVector3List(vertices));
            mesh.SetTriangles(ToIl2CppIntList(triangles), 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            MeshFilter mf = go.AddComponent<MeshFilter>();
            MeshRenderer mr = go.AddComponent<MeshRenderer>();

            mf.mesh = mesh;
            mr.material = _heightfieldMat;
            RegisterDebugRenderer(mr);

            if (AddHeightfieldMeshCollider)
            {
                MeshCollider mc = go.AddComponent<MeshCollider>();
                mc.sharedMesh = mesh;
                mc.convex = false;
                mc.isTrigger = false;
                LogWarning("MeshCollider added to heightfield mesh.");
            }

            return go;
        }

        private static GameObject FakeCollider(Transform parent, HeightfieldStats stats, FakeColliderRequest request)
        {
            if (!stats.valid || stats.surfaceMask == null || stats.heights == null)
            {
                LogWarning("FakeCollider failed: invalid heightfield stats.");
                return null;
            }

            float width = request.width;
            float depth = request.depth;
            float height = request.height;

            Vector3 topCenter = stats.center;

            if (request.mode == FakeColliderMode.Top)
            {
                if (!TryGetSupportedTopBounds(stats, out topCenter, out width, out depth))
                {
                    LogWarning("FakeCollider(Top) failed: no supported top bounds.");
                    return null;
                }

                if (height <= 0f) height = DefaultSeatProxyHeight;
            }
            else
            {
                if (width <= 0f) width = DefaultSeatProxyWidth;
                if (depth <= 0f) depth = DefaultSeatProxyDepth;
                if (height <= 0f) height = DefaultSeatProxyHeight;

                float coverage = CalculateSupportCoverage(stats, topCenter.x, topCenter.z, width, depth);

                if (coverage < MinFakeColliderSupportCoverage)
                {
                    LogWarning("FakeCollider support coverage too low: " + coverage.ToString("F2") +
                               " for requested size " + width.ToString("F2") + "x" + depth.ToString("F2"));

                    if (request.clampToSupportedSurface)
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

                            LogWarning("FakeCollider clamped to supported surface. newTopCenter=" + Vec(topCenter) +
                                       ", newSize=" + width.ToString("F2") + "x" + depth.ToString("F2"));
                        }
                        else
                        {
                            LogWarning("FakeCollider clamp failed; collider not created.");
                            return null;
                        }
                    }
                    else
                    {
                        LogWarning("FakeCollider rejected because support coverage is too low.");
                        return null;
                    }
                }

                topCenter.y = EstimateRectSeatHeight(stats, topCenter.x, topCenter.z, width, depth);
            }

            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = SeatProxyName;

            float centerY = topCenter.y - height * 0.5f;

            go.transform.position = new Vector3(topCenter.x, centerY, topCenter.z);
            go.transform.rotation = Quaternion.identity;
            go.transform.localScale = new Vector3(width, height, depth);

            if (parent != null)
                go.transform.SetParent(parent, true);

            BoxCollider bc = null;
            try { bc = go.GetComponent<BoxCollider>(); } catch { bc = null; }

            if (bc != null)
                bc.isTrigger = false;

            Renderer r = null;
            try { r = go.GetComponent<Renderer>(); } catch { r = null; }

            if (r != null)
            {
                r.material = _seatBoxMat;
                r.enabled = SeatProxyRendererVisibleByDefault;
                RegisterDebugRenderer(r);
            }

            LogWarning(
                "FakeCollider created. mode=" + request.mode +
                ", name=" + go.name +
                ", topY=" + topCenter.y.ToString("F3") +
                ", center=" + Vec(go.transform.position) +
                ", size=(" + width.ToString("F2") + ", " +
                           height.ToString("F2") + ", " +
                           depth.ToString("F2") + ")");

            return go;
        }

        private static float CalculateSupportCoverage(HeightfieldStats stats, float centerX, float centerZ, float width, float depth)
        {
            int supported = 0;
            int total = 0;

            Bounds volume = stats.volume;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float worldZ = Mathf.Lerp(volume.min.z, volume.max.z, tz);

                if (Mathf.Abs(worldZ - centerZ) > depth * 0.5f) continue;

                for (int x = 0; x < MeshGrid; x++)
                {
                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float worldX = Mathf.Lerp(volume.min.x, volume.max.x, tx);

                    if (Mathf.Abs(worldX - centerX) > width * 0.5f) continue;

                    total++;

                    if (stats.surfaceMask[x, z])
                        supported++;
                }
            }

            if (total <= 0) return 0f;

            return supported / (float)total;
        }

        private static float EstimateRectSeatHeight(HeightfieldStats stats, float centerX, float centerZ, float width, float depth)
        {
            List<float> ys = new List<float>();
            Bounds volume = stats.volume;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float worldZ = Mathf.Lerp(volume.min.z, volume.max.z, tz);

                if (Mathf.Abs(worldZ - centerZ) > depth * 0.5f) continue;

                for (int x = 0; x < MeshGrid; x++)
                {
                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float worldX = Mathf.Lerp(volume.min.x, volume.max.x, tx);

                    if (Mathf.Abs(worldX - centerX) > width * 0.5f) continue;
                    if (!stats.surfaceMask[x, z]) continue;

                    ys.Add(stats.heights[x, z]);
                }
            }

            if (ys.Count == 0)
                return stats.medianY;

            ys.Sort();

            // 对坐姿更稳：用 40% 分位，减少枕头/局部凸起把座面抬太高的概率。
            int index = Mathf.Clamp(Mathf.RoundToInt(ys.Count * 0.40f), 0, ys.Count - 1);
            return ys[index];
        }

        private static bool TryClampRectToSupportedSurface(
            HeightfieldStats stats,
            float centerX,
            float centerZ,
            float width,
            float depth,
            out Vector3 newCenter,
            out float newWidth,
            out float newDepth)
        {
            newCenter = new Vector3(centerX, stats.medianY, centerZ);
            newWidth = width;
            newDepth = depth;

            Bounds volume = stats.volume;

            float minX = float.MaxValue;
            float maxX = float.MinValue;
            float minZ = float.MaxValue;
            float maxZ = float.MinValue;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float worldZ = Mathf.Lerp(volume.min.z, volume.max.z, tz);

                if (Mathf.Abs(worldZ - centerZ) > depth * 0.5f) continue;

                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!stats.surfaceMask[x, z]) continue;

                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float worldX = Mathf.Lerp(volume.min.x, volume.max.x, tx);

                    if (Mathf.Abs(worldX - centerX) > width * 0.5f) continue;

                    if (worldX < minX) minX = worldX;
                    if (worldX > maxX) maxX = worldX;
                    if (worldZ < minZ) minZ = worldZ;
                    if (worldZ > maxZ) maxZ = worldZ;
                }
            }

            if (minX == float.MaxValue || minZ == float.MaxValue)
                return false;

            newWidth = Mathf.Max(MinFakeColliderSize, maxX - minX);
            newDepth = Mathf.Max(MinFakeColliderSize, maxZ - minZ);

            if (newWidth < MinFakeColliderSize || newDepth < MinFakeColliderSize)
                return false;

            float newX = (minX + maxX) * 0.5f;
            float newZ = (minZ + maxZ) * 0.5f;
            float newY = EstimateRectSeatHeight(stats, newX, newZ, newWidth, newDepth);

            newCenter = new Vector3(newX, newY, newZ);
            return true;
        }

        private static bool TryGetSupportedTopBounds(HeightfieldStats stats, out Vector3 center, out float width, out float depth)
        {
            center = stats.center;
            width = 0f;
            depth = 0f;

            Bounds volume = stats.volume;

            float minX = float.MaxValue;
            float maxX = float.MinValue;
            float minZ = float.MaxValue;
            float maxZ = float.MinValue;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz = MeshGrid == 1 ? 0.5f : z / (float)(MeshGrid - 1);
                float worldZ = Mathf.Lerp(volume.min.z, volume.max.z, tz);

                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!stats.surfaceMask[x, z]) continue;

                    float tx = MeshGrid == 1 ? 0.5f : x / (float)(MeshGrid - 1);
                    float worldX = Mathf.Lerp(volume.min.x, volume.max.x, tx);

                    if (worldX < minX) minX = worldX;
                    if (worldX > maxX) maxX = worldX;
                    if (worldZ < minZ) minZ = worldZ;
                    if (worldZ > maxZ) maxZ = worldZ;
                }
            }

            if (minX == float.MaxValue || minZ == float.MaxValue)
                return false;

            width = Mathf.Max(MinFakeColliderSize, maxX - minX);
            depth = Mathf.Max(MinFakeColliderSize, maxZ - minZ);

            float cx = (minX + maxX) * 0.5f;
            float cz = (minZ + maxZ) * 0.5f;
            float cy = EstimateRectSeatHeight(stats, cx, cz, width, depth);

            center = new Vector3(cx, cy, cz);

            LogWarning("FakeCollider(Top) supported bounds: center=" + Vec(center) +
                       ", size=" + width.ToString("F2") + "x" + depth.ToString("F2"));

            return true;
        }

        private static void AddTriangleFacingUp(List<int> triangles, List<Vector3> vertices, int ia, int ib, int ic)
        {
            Vector3 a = vertices[ia];
            Vector3 b = vertices[ib];
            Vector3 c = vertices[ic];

            Vector3 n = Vector3.Cross(b - a, c - a);

            if (n.y < 0f)
            {
                triangles.Add(ia);
                triangles.Add(ic);
                triangles.Add(ib);
            }
            else
            {
                triangles.Add(ia);
                triangles.Add(ib);
                triangles.Add(ic);
            }
        }

        private static bool TryGetRendererBounds(GameObject root, out Bounds bounds)
        {
            bounds = new Bounds();

            if (root == null) return false;

            Renderer[] renderers = null;
            try { renderers = root.GetComponentsInChildren<Renderer>(true); } catch { renderers = null; }

            if (renderers == null || renderers.Length == 0) return false;

            return TryCalculateRendererBounds(renderers, out bounds);
        }

        private static int PickScanLayer()
        {
            int[] candidates = { 30, 29, 28, 27, 26, 25 };
            int bestLayer = DefaultScanLayer;
            int bestCount = int.MaxValue;

            GameObject[] all = null;
            try { all = Object.FindObjectsOfType<GameObject>(); } catch { all = null; }

            if (all == null || all.Length == 0) return bestLayer;

            for (int i = 0; i < candidates.Length; i++)
            {
                int layer = candidates[i];
                int count = 0;

                for (int j = 0; j < all.Length; j++)
                {
                    GameObject go = all[j];
                    if (go == null || go.layer != layer) continue;
                    if (IsOwnVisual(go)) continue;

                    Renderer r = null;
                    try { r = go.GetComponent<Renderer>(); } catch { r = null; }

                    if (r != null && r.enabled && go.activeInHierarchy) count++;
                }

                if (count < bestCount)
                {
                    bestCount = count;
                    bestLayer = layer;

                    if (count == 0) break;
                }
            }

            if (bestCount > 0)
                LogWarning("PickScanLayer: no empty candidate layer found; using layer " + bestLayer + " with activeRendererCount=" + bestCount);
            else
                LogWarning("PickScanLayer: selected empty layer " + bestLayer);

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

                if (IsSceneOrRoomRootName(nodeName))
                {
                    LogWarning("GuessScanRoot: stop at scene/room root candidate: " + nodeName);
                    break;
                }

                Renderer[] renderers = null;

                try { renderers = cursor.GetComponentsInChildren<Renderer>(true); } catch { renderers = null; }

                if (renderers == null || renderers.Length == 0) continue;

                Bounds b;
                if (!TryCalculateRendererBounds(renderers, out b)) continue;

                float dist = b.SqrDistance(hitPoint);
                float size = b.size.magnitude;
                float volume = b.size.x * b.size.y * b.size.z;

                if (size > 6.0f || volume > 18.0f)
                {
                    LogWarning("GuessScanRoot: reject too-large ancestor " + nodeName + " size=" + Vec(b.size) + " volume=" + volume.ToString("F2"));
                    continue;
                }

                float score = 0f;

                if (b.Contains(hitPoint)) score += 120f;
                score -= dist * 30f;

                score -= Mathf.Max(0f, size - 2.5f) * 8f;
                score -= Mathf.Max(0f, volume - 4.0f) * 2f;

                if (IsBedLikeName(nodeName)) score += 55f;
                if (IsSeatLikeName(nodeName)) score += 35f;
                if (IsTableLikeName(nodeName)) score += 25f;

                score -= depth * 2.0f;

                if (cursor.gameObject == hitObject && IsLikelyFurnitureName(nodeName))
                    score += 40f;

                if (score > bestScore)
                {
                    bestScore = score;
                    best = cursor.gameObject;
                }
            }

            LogWarning("GuessScanRoot: hitObject=" + hitObject.name + ", selected=" + best.name + ", score=" + bestScore.ToString("F2"));

            return best;
        }

        private static bool IsSceneOrRoomRootName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;

            string n = name.ToLowerInvariant();

            return n == "bedroom" ||
                   n.Contains("room") ||
                   n.Contains("scene") ||
                   n.Contains("level") ||
                   n.Contains("location") ||
                   n.Contains("environment") ||
                   n.Contains("interior");
        }

        private static bool IsLikelyFurnitureName(string name)
        {
            return IsBedLikeName(name) || IsSeatLikeName(name) || IsTableLikeName(name);
        }

        private static bool IsBedLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;

            string n = name.ToLowerInvariant();

            if (n.Contains("bedroom")) return false;

            return n == "bed" ||
                   n.StartsWith("bed_") || n.EndsWith("_bed") ||
                   n.StartsWith("bed ") || n.EndsWith(" bed") ||
                   n.Contains("/bed") || n.Contains("bed/") ||
                   n.Contains("mattress") ||
                   n.Contains("blanket") ||
                   n.Contains("futon");
        }

        private static bool IsSeatLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;

            string n = name.ToLowerInvariant();

            return n.Contains("chair") ||
                   n.Contains("stool") ||
                   n.Contains("sofa") ||
                   n.Contains("seat") ||
                   n.Contains("bench") ||
                   n.Contains("couch");
        }

        private static bool IsTableLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;

            string n = name.ToLowerInvariant();

            return n.Contains("table") ||
                   n.Contains("desk") ||
                   n.Contains("counter");
        }

        private static bool TryCalculateRendererBounds(Renderer[] renderers, out Bounds bounds)
        {
            bounds = new Bounds();

            bool has = false;

            for (int i = 0; i < renderers.Length; i++)
            {
                Renderer r = renderers[i];

                if (r == null) continue;
                if (IsOwnVisual(r.gameObject)) continue;

                try
                {
                    if (!has)
                    {
                        bounds = r.bounds;
                        has = true;
                    }
                    else
                    {
                        bounds.Encapsulate(r.bounds);
                    }
                }
                catch { }
            }

            return has;
        }

        private static bool TryRaycastScene(Ray ray, out RaycastHit bestHit)
        {
            bestHit = new RaycastHit();

            RaycastHit[] hits = null;
            try { hits = Physics.RaycastAll(ray, RayDistance, -1, QueryTriggerInteraction.Ignore); } catch { hits = null; }

            if (hits == null || hits.Length == 0) return false;

            bool found = false;
            float bestDistance = float.MaxValue;

            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit h = hits[i];

                if (h.collider == null) continue;
                if (h.distance < 0.03f) continue;
                if (IsOwnVisual(h.collider.gameObject)) continue;

                if (h.distance < bestDistance)
                {
                    bestDistance = h.distance;
                    bestHit = h;
                    found = true;
                }
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

                if (child == null) continue;

                AssignLayerRecursive(child.gameObject, layer, backups);
            }
        }

        private static void RestoreLayers(List<LayerBackup> backups)
        {
            if (backups == null) return;

            for (int i = 0; i < backups.Count; i++)
            {
                try
                {
                    if (backups[i].go != null)
                        backups[i].go.layer = backups[i].layer;
                }
                catch { }
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
                c + new Vector3(-e.x, -e.y, -e.z),
                c + new Vector3( e.x, -e.y, -e.z),
                c + new Vector3(-e.x, -e.y,  e.z),
                c + new Vector3( e.x, -e.y,  e.z),
                c + new Vector3(-e.x,  e.y, -e.z),
                c + new Vector3( e.x,  e.y, -e.z),
                c + new Vector3(-e.x,  e.y,  e.z),
                c + new Vector3( e.x,  e.y,  e.z),
            };

            CreateLine(p[0], p[1], root.transform);
            CreateLine(p[0], p[2], root.transform);
            CreateLine(p[3], p[1], root.transform);
            CreateLine(p[3], p[2], root.transform);

            CreateLine(p[4], p[5], root.transform);
            CreateLine(p[4], p[6], root.transform);
            CreateLine(p[7], p[5], root.transform);
            CreateLine(p[7], p[6], root.transform);

            CreateLine(p[0], p[4], root.transform);
            CreateLine(p[1], p[5], root.transform);
            CreateLine(p[2], p[6], root.transform);
            CreateLine(p[3], p[7], root.transform);
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
            if (forceReload)
            {
                _depthToEyeMat = null;
                _eyeDepthReplacementShader = null;
            }

            string path = null;

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
                        LogWarning("Loaded EyeDepthReplacement shader by iCall AssetBundle: " + shader.name);
                    }
                    else
                    {
                        LogWarning("EyeDepthReplacement shader not loaded from AB.");
                    }
                }
                catch (Exception e)
                {
                    LogWarning("EyeDepthReplacement shader AB load failed: " + e.GetType().Name + " " + e.Message);
                }

                try
                {
                    Material mat;

                    if (ICallAssetBundleLoader.TryLoadMaterialByName(path, MaterialName, out mat) && mat != null)
                    {
                        _depthToEyeMat = mat;
                        LogWarning("Loaded depth material by iCall AssetBundle: " + mat.name);
                    }
                    else
                    {
                        LogWarning("DepthToEye material not loaded from AB.");
                    }
                }
                catch (Exception e)
                {
                    LogWarning("DepthToEye material AB load failed: " + e.GetType().Name + " " + e.Message);
                }
            }
            else
            {
                LogWarning("Depth AssetBundle not found: " + path);
            }

            if (_eyeDepthReplacementShader == null)
            {
                try
                {
                    Shader fallback = Shader.Find(EyeDepthReplacementShaderFindName);

                    if (fallback != null)
                    {
                        _eyeDepthReplacementShader = fallback;
                        LogWarning("Loaded EyeDepthReplacement shader from Shader.Find: " + fallback.name);
                    }
                }
                catch { }
            }

            if (_depthToEyeMat == null)
            {
                try
                {
                    Shader fallback = Shader.Find(ShaderName);

                    if (fallback != null)
                    {
                        _depthToEyeMat = new Material(fallback);
                        LogWarning("Loaded DepthToEye material from Shader.Find fallback.");
                    }
                }
                catch { }
            }

            if (_eyeDepthReplacementShader == null)
                LogWarning("EyeDepthReplacement shader unavailable. Old depth paths may be used.");

            if (_depthToEyeMat == null)
                LogWarning("DepthToEye material unavailable. Only EyeDepthReplacement or DepthNormals fallback can work.");
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
            for (int i = _created.Count - 1; i >= 0; i--)
                DestroyObject(_created[i]);

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

                if (r == null)
                {
                    _debugRenderers.RemoveAt(i);
                    continue;
                }

                try { r.enabled = visible; } catch { }
            }

            LogWarning("SetDebugRenderersVisible(" + visible + ") applied to " + _debugRenderers.Count + " renderers.");
        }

        private static void RegisterDebugRenderer(Renderer r)
        {
            if (r == null) return;

            _debugRenderers.Add(r);
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
            h.transform.localPosition = Vector3.zero;
            h.transform.localRotation = Quaternion.identity;
            h.transform.localScale = new Vector3(0.055f, 0.0045f, 0.0045f);
            SetRendererMaterial(h, _redMat, false);
            DisableCollider(h);

            GameObject v = GameObject.CreatePrimitive(PrimitiveType.Cube);
            v.name = "Crosshair_Vertical";
            v.transform.SetParent(_crosshairRoot.transform, false);
            v.transform.localPosition = Vector3.zero;
            v.transform.localRotation = Quaternion.identity;
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
            try { cameras = Object.FindObjectsOfType<Camera>(); } catch { cameras = null; }

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
                if (name.IndexOf("Camera", StringComparison.OrdinalIgnoreCase) >= 0) score += 5f;

                if (name.IndexOf("Snapshot", StringComparison.OrdinalIgnoreCase) >= 0) score -= 500f;
                if (name.IndexOf("Ghost", StringComparison.OrdinalIgnoreCase) >= 0) score -= 500f;
                if (name.IndexOf("TopSurfaceSeatProxy", StringComparison.OrdinalIgnoreCase) >= 0) score -= 1000f;
                if (name.IndexOf("UI", StringComparison.OrdinalIgnoreCase) >= 0) score -= 200f;

                if (score > bestScore)
                {
                    bestScore = score;
                    best = cam;
                }
            }

            return best;
        }

        private static bool IsUsableCamera(Camera cam)
        {
            if (cam == null) return false;
            if (!cam.enabled) return false;
            if (cam.gameObject == null || !cam.gameObject.activeInHierarchy) return false;

            return true;
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

            try { shader = Shader.Find("Hidden/Internal-Colored"); } catch { shader = null; }
            if (shader == null) { try { shader = Shader.Find("Unlit/Color"); } catch { shader = null; } }
            if (shader == null) { try { shader = Shader.Find("Sprites/Default"); } catch { shader = null; } }
            if (shader == null) { try { shader = Shader.Find("Standard"); } catch { shader = null; } }

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
                try { mat.DisableKeyword("_ALPHATEST_ON"); } catch { }
                try { mat.DisableKeyword("_ALPHAPREMULTIPLY_ON"); } catch { }

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

                    if (debugRenderer)
                        RegisterDebugRenderer(r);
                }
            }
            catch { }
        }

        private static void DisableCollider(GameObject go)
        {
            try
            {
                Collider c = go.GetComponent<Collider>();

                if (c != null)
                    Object.Destroy(c);
            }
            catch { }
        }

        private static void ReleaseRT(RenderTexture rt)
        {
            if (rt == null) return;

            try
            {
                if (RenderTexture.active == rt)
                    RenderTexture.active = null;
            }
            catch { }

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
            try { _log?.LogWarning("[TopSurfaceSeatProxyTester] " + message); } catch { }
            try { Debug.LogWarning("[TopSurfaceSeatProxyTester] " + message); } catch { }
        }

        private static void LogError(string message)
        {
            try { _log?.LogError("[TopSurfaceSeatProxyTester] " + message); } catch { }
            try { Debug.LogError("[TopSurfaceSeatProxyTester] " + message); } catch { }
        }

        private static void TryConsolePrint(string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); } catch { }
        }

        private static Il2CppSystem.Collections.Generic.List<Vector3> ToIl2CppVector3List(List<Vector3> src)
        {
            var dst = new Il2CppSystem.Collections.Generic.List<Vector3>();

            if (src == null) return dst;

            for (int i = 0; i < src.Count; i++)
                dst.Add(src[i]);

            return dst;
        }

        private static Il2CppSystem.Collections.Generic.List<int> ToIl2CppIntList(List<int> src)
        {
            var dst = new Il2CppSystem.Collections.Generic.List<int>();

            if (src == null) return dst;

            for (int i = 0; i < src.Count; i++)
                dst.Add(src[i]);

            return dst;
        }
    }
}