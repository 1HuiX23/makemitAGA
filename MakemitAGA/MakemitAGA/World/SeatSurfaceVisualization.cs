/*
 * =================================================================================================
 * SeatSurfaceVisualization.cs
 * =================================================================================================
 *
 * 作用：根据分析结果创建可查询的 MeshCollider、分类覆盖网格与可选调试几何。

 * 主要逻辑：
 *   - 红/绿/橙/紫分类网格；
 *   - 完整高度图 Mesh 与非凸分析 MeshCollider；
 *   - 固定尺寸 FakeCollider 和局部座面代理辅助；
 *   - 动作箭头、站立胶囊、腿部空间等调试显示。
 *
 * 注意：这些 Renderer 默认隐藏；Collider 是否存在不依赖 debug_svt。
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
        private static GameObject BuildSeatabilityOverlay(
            Transform parent,
            HeightfieldStats heightfield,
            bool[,] validSeatCenters,
            bool buildValid,
            string objectName,
            Material material,
            float lift)
        {
            if (!heightfield.valid ||
                heightfield.surfaceMask == null ||
                validSeatCenters == null)
            {
                return null;
            }

            List<Vector3> vertices = new List<Vector3>();
            List<int> triangles = new List<int>();

            float cellX =
                heightfield.volume.size.x /
                Mathf.Max(1f, MeshGrid - 1f);

            float cellZ =
                heightfield.volume.size.z /
                Mathf.Max(1f, MeshGrid - 1f);

            float halfX = cellX * 0.44f;
            float halfZ = cellZ * 0.44f;

            for (int z = 0; z < MeshGrid; z++)
            {
                float tz =
                    MeshGrid == 1
                        ? 0.5f
                        : z / (float)(MeshGrid - 1);

                float worldZ = Mathf.Lerp(
                    heightfield.volume.min.z,
                    heightfield.volume.max.z,
                    tz);

                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!heightfield.surfaceMask[x, z])
                        continue;

                    bool valid = validSeatCenters[x, z];
                    if (valid != buildValid)
                        continue;

                    float tx =
                        MeshGrid == 1
                            ? 0.5f
                            : x / (float)(MeshGrid - 1);

                    float worldX = Mathf.Lerp(
                        heightfield.volume.min.x,
                        heightfield.volume.max.x,
                        tx);

                    float y =
                        heightfield.heights[x, z] + lift;

                    int start = vertices.Count;

                    vertices.Add(new Vector3(
                        worldX - halfX,
                        y,
                        worldZ - halfZ));

                    vertices.Add(new Vector3(
                        worldX + halfX,
                        y,
                        worldZ - halfZ));

                    vertices.Add(new Vector3(
                        worldX - halfX,
                        y,
                        worldZ + halfZ));

                    vertices.Add(new Vector3(
                        worldX + halfX,
                        y,
                        worldZ + halfZ));

                    // Up-facing quads.
                    triangles.Add(start + 0);
                    triangles.Add(start + 2);
                    triangles.Add(start + 1);

                    triangles.Add(start + 3);
                    triangles.Add(start + 1);
                    triangles.Add(start + 2);
                }
            }

            if (vertices.Count < 3 ||
                triangles.Count < 3)
            {
                return null;
            }

            GameObject go = new GameObject(objectName);
            if (parent != null)
                go.transform.SetParent(parent, true);

            Mesh mesh = new Mesh
            {
                name = objectName + "_Mesh"
            };

            mesh.SetVertices(
                ToIl2CppVector3List(vertices));

            mesh.SetTriangles(
                ToIl2CppIntList(triangles),
                0);

            mesh.RecalculateNormals();
            mesh.RecalculateBounds();

            MeshFilter filter =
                go.AddComponent<MeshFilter>();

            MeshRenderer renderer =
                go.AddComponent<MeshRenderer>();

            filter.mesh = mesh;
            renderer.material = material;
            RegisterDebugRenderer(renderer);

            return go;
        }

        private static int CreateRepresentativeActionMarkers(
            Transform parent,
            Bounds bedBounds,
            HeightfieldStats heightfield,
            SeatabilityStats seatability)
        {
            _actionMarkerRoot = new GameObject(
                "BedSeatability_ActionMarkers_Yellow");

            _actionMarkerRoot.transform.SetParent(parent, true);

            _clearanceDebugRoot = new GameObject(
                "BedSeatability_ClearanceVolumes_Debug");

            _clearanceDebugRoot.transform.SetParent(parent, true);

            _actionMarkersVisible = ShowActionMarkersByDefault;
            _clearanceDebugVisible = ShowClearanceDebugByDefault;

            _actionMarkerRoot.SetActive(
                _actionMarkersVisible);

            _clearanceDebugRoot.SetActive(
                _clearanceDebugVisible);

            List<SeatCandidatePoint> all =
                new List<SeatCandidatePoint>();

            float preferredEdgeInset =
                (ActionMinEdgeInset + ActionMaxEdgeInset) * 0.5f;

            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (!seatability.actionValidCenters[x, z])
                        continue;

                    Vector3 world = GridToWorld(
                        heightfield,
                        x,
                        z,
                        heightfield.heights[x, z] + 0.095f);

                    Vector3 floor =
                        seatability.actionFloorPoints[x, z];

                    Vector3 outward =
                        seatability.actionOutwardDirections[x, z];

                    float edgeDistance =
                        seatability.actionEdgeDistances[x, z];

                    float heightPenalty =
                        seatability.heightWarningCenters[x, z]
                            ? 1.0f
                            : 0f;

                    float score =
                        Mathf.Abs(
                            edgeDistance - preferredEdgeInset) +
                        heightPenalty * 0.25f;

                    all.Add(new SeatCandidatePoint
                    {
                        world = world,
                        floorWorld = floor,
                        outward = outward,
                        edgeDistance = edgeDistance,
                        score = score
                    });
                }
            }

            all.Sort(
                delegate (
                    SeatCandidatePoint a,
                    SeatCandidatePoint b)
                {
                    return a.score.CompareTo(b.score);
                });

            List<Vector3> selected =
                new List<Vector3>();

            for (int i = 0;
                 i < all.Count &&
                 selected.Count < MaxCandidateMarkers;
                 i++)
            {
                SeatCandidatePoint candidate = all[i];

                bool tooClose = false;

                for (int j = 0; j < selected.Count; j++)
                {
                    Vector2 a = new Vector2(
                        candidate.world.x,
                        candidate.world.z);

                    Vector2 b = new Vector2(
                        selected[j].x,
                        selected[j].z);

                    if (Vector2.Distance(a, b) <
                        CandidateMarkerSpacing)
                    {
                        tooClose = true;
                        break;
                    }
                }

                if (tooClose)
                    continue;

                selected.Add(candidate.world);

                int index = selected.Count;

                CreateSphere(
                    "BedSeatability_ActionSeat_Yellow_" + index,
                    candidate.world,
                    0.060f,
                    _candidatePointMat,
                    _actionMarkerRoot.transform,
                    true);

                Vector3 arrowEnd =
                    candidate.floorWorld +
                    Vector3.up * 0.085f;

                CreateAxis(
                    "BedSeatability_ActionDirection_Yellow_" + index,
                    candidate.world,
                    arrowEnd - candidate.world,
                    Vector3.Distance(
                        candidate.world,
                        arrowEnd),
                    _actionArrowMat,
                    _actionMarkerRoot.transform,
                    true);

                CreateSphere(
                    "BedSeatability_ActionFloor_Yellow_" + index,
                    arrowEnd,
                    0.045f,
                    _actionArrowMat,
                    _actionMarkerRoot.transform,
                    true);

                CreateStandingCapsuleDebugVisual(
                    "BedSeatability_StandingCapsule_Blue_" + index,
                    candidate.floorWorld,
                    _clearanceDebugRoot.transform);

                CreateLegSpaceDebugVisual(
                    "BedSeatability_LegSpace_Pink_" + index,
                    candidate.world,
                    candidate.floorWorld,
                    candidate.outward,
                    candidate.edgeDistance,
                    _clearanceDebugRoot.transform);
            }

            return selected.Count;
        }

        private static void CreateStandingCapsuleDebugVisual(
            string objectName,
            Vector3 floorPoint,
            Transform parent)
        {
            GameObject capsule =
                GameObject.CreatePrimitive(
                    PrimitiveType.Capsule);

            capsule.name = objectName;
            capsule.transform.SetParent(parent, true);

            capsule.transform.position =
                floorPoint +
                Vector3.up *
                (ActionBodyBottomClearance +
                 ActionBodyHeight * 0.5f);

            // Unity primitive capsule: default total height 2, radius 0.5.
            capsule.transform.localScale =
                new Vector3(
                    ActionBodyRadius * 2f,
                    ActionBodyHeight * 0.5f,
                    ActionBodyRadius * 2f);

            SetRendererMaterial(
                capsule,
                _clearanceCapsuleMat,
                true);

            DisableCollider(capsule);
        }

        private static void CreateLegSpaceDebugVisual(
            string objectName,
            Vector3 seatPoint,
            Vector3 floorPoint,
            Vector3 outward,
            float edgeDistance,
            Transform parent)
        {
            outward.y = 0f;

            if (outward.sqrMagnitude < 0.001f)
                return;

            outward.Normalize();

            Vector3 boundaryPoint =
                seatPoint +
                outward * edgeDistance;

            GameObject box =
                GameObject.CreatePrimitive(
                    PrimitiveType.Cube);

            box.name = objectName;
            box.transform.SetParent(parent, true);

            box.transform.rotation =
                Quaternion.LookRotation(
                    outward,
                    Vector3.up);

            box.transform.position =
                boundaryPoint +
                outward *
                (ActionLegSpaceDepth * 0.5f + 0.04f);

            box.transform.position =
                new Vector3(
                    box.transform.position.x,
                    floorPoint.y +
                    ActionLegSpaceBottomClearance +
                    ActionLegSpaceHeight * 0.5f,
                    box.transform.position.z);

            box.transform.localScale =
                new Vector3(
                    ActionLegSpaceWidth,
                    ActionLegSpaceHeight,
                    ActionLegSpaceDepth);

            SetRendererMaterial(
                box,
                _legSpaceMat,
                true);

            DisableCollider(box);
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

            LogInfo("Bed Top MeshCollider created. vertices=" + verts.Count + ", triangles=" + (tris.Count / 3));
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

            LogInfo("Seat proxy created. name=" + go.name + ", top=" + Vec(topCenter) + ", forward=" + Vec(go.transform.forward) + ", size=(" + width.ToString("F2") + "," + height.ToString("F2") + "," + depth.ToString("F2") + ")");
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

    }
}
