/*
 * =================================================================================================
 * SeatSurfaceScanCapture.cs
 * =================================================================================================
 *
 * 作用：负责从屏幕中心或指定目标建立扫描体积，并通过深度相机生成稀疏高度图。

 * 主要逻辑：
 *   - FixedSize / Top 两种扫描入口；
 *   - 完整目标高度范围计算；
 *   - EyeDepthReplacement / DepthToEye 深度路径；
 *   - 深度反投影、空洞填充、岛屿过滤、平滑与三角形连接。
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

            LogInfo("Scan setup: target=" + targetRoot.name +
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

        private static Bounds BuildScanVolume(
            Vector3 hitPoint,
            bool hasTargetBounds,
            Bounds targetBounds,
            FakeColliderRequest request)
        {
            float centerX = hitPoint.x;
            float centerZ = hitPoint.z;
            float sizeX;
            float sizeZ;

            bool fullTargetTopMode =
                request.mode == FakeColliderMode.Top &&
                hasTargetBounds;

            if (fullTargetTopMode)
            {
                centerX = targetBounds.center.x;
                centerZ = targetBounds.center.z;

                sizeX = Mathf.Clamp(
                    targetBounds.size.x + TopScanPadding,
                    FixedScanMinSize,
                    TopScanMaxSize);

                sizeZ = Mathf.Clamp(
                    targetBounds.size.z + TopScanPadding,
                    FixedScanMinSize,
                    TopScanMaxSize);
            }
            else
            {
                float requestedX =
                    request.width > 0f
                        ? request.width
                        : DefaultSeatProxyWidth;

                float requestedZ =
                    request.depth > 0f
                        ? request.depth
                        : DefaultSeatProxyDepth;

                sizeX = Mathf.Clamp(
                    requestedX + FixedScanPadding,
                    FixedScanMinSize,
                    FixedScanMaxSize);

                sizeZ = Mathf.Clamp(
                    requestedZ + FixedScanPadding,
                    FixedScanMinSize,
                    FixedScanMaxSize);
            }

            float minY;
            float maxY;

            if (fullTargetTopMode)
            {
                // Do not center a thin 0.70m band around the highest point.
                // Scan the selected object's entire rough vertical extent so lower
                // cushions remain available even when a sofa backrest is much higher.
                minY =
                    targetBounds.min.y -
                    TopScanBottomPadding;

                maxY =
                    targetBounds.max.y +
                    TopScanTopPadding;

                if (maxY - minY >
                    TopScanMaxVerticalHeight)
                {
                    // Pathological/oversized hierarchy protection. Keep the upper part,
                    // because this remains a top-surface pipeline.
                    minY =
                        maxY -
                        TopScanMaxVerticalHeight;
                }

                if (maxY <= minY + 0.08f)
                    maxY = minY + 0.35f;
            }
            else if (hasTargetBounds)
            {
                float rendererTop =
                    targetBounds.max.y +
                    ScanBoxRendererTopMargin;

                float rendererBottom =
                    targetBounds.min.y -
                    0.05f;

                minY = Mathf.Max(
                    hitPoint.y - ScanBoxBelowHit,
                    rendererBottom);

                maxY = Mathf.Max(
                    hitPoint.y +
                    ScanBoxAboveHitWithBounds,
                    rendererTop);

                maxY = Mathf.Min(
                    maxY,
                    rendererTop +
                    ScanBoxTargetTopExtra);

                if (maxY <= minY + 0.08f)
                    maxY = minY + 0.35f;
            }
            else
            {
                minY =
                    hitPoint.y -
                    ScanBoxBelowHit;

                maxY =
                    hitPoint.y +
                    ScanBoxAboveHitNoBounds;
            }

            if (!fullTargetTopMode &&
                maxY - minY >
                ScanBoxMaxHeight)
            {
                maxY = Mathf.Min(
                    maxY,
                    hitPoint.y +
                    ScanBoxAboveHitNoBounds);

                minY =
                    maxY -
                    ScanBoxMaxHeight;
            }

            return new Bounds(
                new Vector3(
                    centerX,
                    (minY + maxY) * 0.5f,
                    centerZ),
                new Vector3(
                    sizeX,
                    maxY - minY,
                    sizeZ));
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
                    LogInfo("Depth capture path: EyeDepthReplacement direct render.");
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
                    LogInfo("Depth capture path: old DepthToEye material fallback.");
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

            float allowedHeightDeviation =
                Mathf.Clamp(
                    volume.size.y *
                    GenericHeightDeviationVolumeFactor,
                    MaxHeightDeviationFromMedian,
                    GenericMaxHeightDeviation);

            int rawCellValid = 0;
            int kept = 0;
            int removed = 0;

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!valid[x, z]) continue;
                    rawCellValid++;
                    if (Mathf.Abs(height[x, z] - median) > allowedHeightDeviation)
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

            LogInfo(
                "Top heightfield mesh built." +
                " rawPixels=" + rawPixelValid +
                ", rawCells=" + rawCellValid +
                ", kept=" + kept +
                ", surfaceCells=" + surfaceCells +
                ", seatY=" + seatY.ToString("F3") +
                ", avgY=" + avgY.ToString("F3") +
                ", medianY=" + median.ToString("F3") +
                ", allowedHeightDeviation=" +
                allowedHeightDeviation.ToString("F3") +
                ", scanHeight=" +
                volume.size.y.ToString("F3"));
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

    }
}
