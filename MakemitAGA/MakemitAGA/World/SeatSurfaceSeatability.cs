/*
 * =================================================================================================
 * SeatSurfaceSeatability.cs
 * =================================================================================================
 *
 * 作用：把高度图转换成“能否坐、是否适合动作”的物理判定数据。

 * 主要逻辑：
 *   - 支撑覆盖率与法线/坡度检查；
 *   - 座面边缘和外侧方向估计；
 *   - 地面点、站立胶囊、接近走廊与腿部空间检查；
 *   - 绿色支撑区、橙色警告区和紫色动作有效区的统计。
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
        private static SeatabilityStats CreateSeatabilityStats(
            HeightfieldStats heightfield)
        {
            return new SeatabilityStats
            {
                validSeatCenters =
                    new bool[MeshGrid, MeshGrid],

                heightWarningCenters =
                    new bool[MeshGrid, MeshGrid],

                actionValidCenters =
                    new bool[MeshGrid, MeshGrid],

                actionFloorPoints =
                    new Vector3[MeshGrid, MeshGrid],

                actionOutwardDirections =
                    new Vector3[MeshGrid, MeshGrid],

                actionEdgeDistances =
                    new float[MeshGrid, MeshGrid],

                cellSizeX =
                    heightfield.volume.size.x /
                    Mathf.Max(1f, MeshGrid - 1f),

                cellSizeZ =
                    heightfield.volume.size.z /
                    Mathf.Max(1f, MeshGrid - 1f)
            };
        }

        private static IEnumerator AnalyzeSeatabilityBatched(
            GameObject target,
            Bounds targetBounds,
            HeightfieldStats heightfield,
            SeatabilityStats result,
            int serial)
        {
            float floorY;

            result.floorFound =
                TryEstimateFloorYAroundBed(
                    target,
                    targetBounds,
                    heightfield.minY,
                    out floorY);

            result.floorY = floorY;

            int radiusX =
                Mathf.Max(
                    1,
                    Mathf.CeilToInt(
                        (SeatTestWidth * 0.5f) /
                        Mathf.Max(
                            0.001f,
                            result.cellSizeX)));

            int radiusZ =
                Mathf.Max(
                    1,
                    Mathf.CeilToInt(
                        (SeatTestDepth * 0.5f) /
                        Mathf.Max(
                            0.001f,
                            result.cellSizeZ)));

            int processedThisFrame = 0;

            // First pass: cheap general support checks.
            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (serial != _scanSerial)
                        yield break;

                    if (heightfield.surfaceMask[x, z])
                    {
                        result.proxySurfaceCells++;

                        float normalY =
                            EstimateHeightfieldNormalY(
                                heightfield,
                                x,
                                z,
                                result.cellSizeX,
                                result.cellSizeZ);

                        bool slopeOk =
                            normalY >= SeatMinNormalY;

                        float coverage;
                        float patchHeightRange;

                        EvaluateSeatFootprint(
                            heightfield,
                            x,
                            z,
                            radiusX,
                            radiusZ,
                            out coverage,
                            out patchHeightRange);

                        bool coverageOk =
                            coverage >=
                            SeatMinSupportCoverage;

                        bool flatnessOk =
                            patchHeightRange <=
                            SeatMaxPatchHeightRange;

                        float heightAboveFloor =
                            result.floorFound
                                ? heightfield.heights[x, z] -
                                  result.floorY
                                : 0f;

                        bool hardHeightOk =
                            !result.floorFound ||
                            heightAboveFloor <=
                            SeatHardMaxHeightAboveFloor;

                        bool softHeightWarning =
                            result.floorFound &&
                            heightAboveFloor >
                                SeatIdealHeightAboveFloor &&
                            heightAboveFloor <=
                                SeatHardMaxHeightAboveFloor;

                        bool valid =
                            slopeOk &&
                            coverageOk &&
                            flatnessOk &&
                            hardHeightOk;

                        result.validSeatCenters[x, z] =
                            valid;

                        result.heightWarningCenters[x, z] =
                            valid &&
                            softHeightWarning;

                        if (valid)
                        {
                            result.validSeatCells++;

                            if (softHeightWarning)
                                result.heightWarningCells++;
                        }
                        else
                        {
                            if (!slopeOk)
                                result.rejectedSlope++;

                            if (!coverageOk)
                                result.rejectedCoverage++;

                            if (!flatnessOk)
                                result.rejectedFlatness++;

                            if (!hardHeightOk)
                                result.rejectedHeight++;
                        }
                    }

                    processedThisFrame++;

                    if (processedThisFrame >=
                        GeneralAnalysisCellsPerFrame)
                    {
                        processedThisFrame = 0;
                        yield return null;
                    }
                }
            }

            _scanStage = "ActionAnalysis";
            processedThisFrame = 0;

            // Second pass: expensive edge, floor, NavMesh and clearance checks.
            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (serial != _scanSerial)
                        yield break;

                    if (result.validSeatCenters[x, z])
                    {
                        float edgeDistance;
                        Vector3 outward;

                        if (!TryFindNearestSurfaceBoundary(
                            heightfield,
                            x,
                            z,
                            result.cellSizeX,
                            result.cellSizeZ,
                            out edgeDistance,
                            out outward))
                        {
                            result
                                .rejectedActionEdgeDistance++;
                        }
                        else
                        {
                            result.actionEdgeDistances[x, z] =
                                edgeDistance;

                            result
                                .actionOutwardDirections[x, z] =
                                outward;

                            if (edgeDistance <
                                    ActionMinEdgeInset ||
                                edgeDistance >
                                    ActionMaxEdgeInset)
                            {
                                result
                                    .rejectedActionEdgeDistance++;
                            }
                            else
                            {
                                Vector3 seatPoint =
                                    GridToWorld(
                                        heightfield,
                                        x,
                                        z,
                                        heightfield.heights[x, z]);

                                Vector3 floorPoint;
                                float seatFloorDrop;
                                ActionRejectReason rejectReason;

                                bool actionFloorOk =
                                    TryFindActionFloorPoint(
                                        target.transform,
                                        seatPoint,
                                        outward,
                                        edgeDistance,
                                        out floorPoint,
                                        out seatFloorDrop,
                                        out rejectReason);

                                if (!actionFloorOk)
                                {
                                    CountActionReject(
                                        result,
                                        rejectReason);
                                }
                                else
                                {
                                    result
                                        .actionFloorPoints[x, z] =
                                        floorPoint;

                                    result
                                        .actionValidCenters[x, z] =
                                        true;

                                    result.actionValidCells++;
                                }
                            }
                        }
                    }

                    processedThisFrame++;

                    if (processedThisFrame >=
                        ActionAnalysisCellsPerFrame)
                    {
                        processedThisFrame = 0;
                        yield return null;
                    }
                }
            }

            if (!result.floorFound)
            {
                LogWarning(
                    "[SeatSurface] General floor reference was not found. " +
                    "General height classification ignored the height limit, " +
                    "while action-valid points still required their own outside floor.");
            }
        }

        private static void CountActionReject(
            SeatabilityStats result,
            ActionRejectReason rejectReason)
        {
            switch (rejectReason)
            {
                case ActionRejectReason.Floor:
                    result.rejectedActionFloor++;
                    break;

                case ActionRejectReason.NavMesh:
                    result.rejectedActionNavMesh++;
                    break;

                case ActionRejectReason.NavMeshOffset:
                    result.rejectedActionNavMeshOffset++;
                    break;

                case ActionRejectReason.Drop:
                    result.rejectedActionDrop++;
                    break;

                case ActionRejectReason.BodyClearance:
                    result.rejectedActionBodyClearance++;
                    break;

                case ActionRejectReason.CorridorClearance:
                    result.rejectedActionCorridorClearance++;
                    break;

                case ActionRejectReason.LegClearance:
                    result.rejectedActionLegClearance++;
                    break;

                default:
                    result.rejectedActionFloor++;
                    break;
            }
        }

        private static Vector3 GridToWorld(
            HeightfieldStats heightfield,
            int x,
            int z,
            float y)
        {
            float tx =
                MeshGrid == 1
                    ? 0.5f
                    : x / (float)(MeshGrid - 1);

            float tz =
                MeshGrid == 1
                    ? 0.5f
                    : z / (float)(MeshGrid - 1);

            return new Vector3(
                Mathf.Lerp(
                    heightfield.volume.min.x,
                    heightfield.volume.max.x,
                    tx),
                y,
                Mathf.Lerp(
                    heightfield.volume.min.z,
                    heightfield.volume.max.z,
                    tz));
        }

        private static bool TryFindNearestSurfaceBoundary(
            HeightfieldStats heightfield,
            int centerX,
            int centerZ,
            float cellSizeX,
            float cellSizeZ,
            out float edgeDistance,
            out Vector3 outward)
        {
            edgeDistance = float.MaxValue;
            outward = Vector3.zero;

            int[,] dirs =
            {
                { -1,  0 },
                {  1,  0 },
                {  0, -1 },
                {  0,  1 }
            };

            Vector3[] worldDirs =
            {
                Vector3.left,
                Vector3.right,
                Vector3.back,
                Vector3.forward
            };

            float[] stepSizes =
            {
                cellSizeX,
                cellSizeX,
                cellSizeZ,
                cellSizeZ
            };

            int maxSteps = MeshGrid;

            for (int d = 0; d < 4; d++)
            {
                int dx = dirs[d, 0];
                int dz = dirs[d, 1];

                for (int step = 1; step <= maxSteps; step++)
                {
                    int x = centerX + dx * step;
                    int z = centerZ + dz * step;

                    bool outside =
                        x < 0 ||
                        x >= MeshGrid ||
                        z < 0 ||
                        z >= MeshGrid ||
                        !heightfield.surfaceMask[x, z];

                    if (!outside)
                        continue;

                    // The center is approximately (step - 0.5) cells inside the boundary.
                    float distance =
                        Mathf.Max(0f, step - 0.5f) *
                        stepSizes[d];

                    if (distance < edgeDistance)
                    {
                        edgeDistance = distance;
                        outward = worldDirs[d];
                    }

                    break;
                }
            }

            return
                edgeDistance < float.MaxValue &&
                outward.sqrMagnitude > 0.5f;
        }

        private static bool TryFindActionFloorPoint(
            Transform bedTarget,
            Vector3 seatPoint,
            Vector3 outward,
            float edgeDistance,
            out Vector3 floorPoint,
            out float seatFloorDrop,
            out ActionRejectReason rejectReason)
        {
            floorPoint = Vector3.zero;
            seatFloorDrop = 0f;
            rejectReason = ActionRejectReason.None;

            outward.y = 0f;

            if (outward.sqrMagnitude < 0.001f)
            {
                rejectReason = ActionRejectReason.Floor;
                return false;
            }

            outward.Normalize();

            Vector3 boundaryPoint =
                seatPoint + outward * edgeDistance;

            Vector3 outsideXZ =
                boundaryPoint +
                outward * ActionOutsideStandDistance;

            Vector3 rayOrigin =
                new Vector3(
                    outsideXZ.x,
                    seatPoint.y + ActionFloorProbeHeight,
                    outsideXZ.z);

            RaycastHit[] hits = null;

            try
            {
                hits = Physics.RaycastAll(
                    rayOrigin,
                    Vector3.down,
                    ActionFloorProbeDistance,
                    -1,
                    QueryTriggerInteraction.Ignore);
            }
            catch { }

            if (hits == null || hits.Length == 0)
            {
                rejectReason = ActionRejectReason.Floor;
                return false;
            }

            Array.Sort(
                hits,
                delegate (RaycastHit a, RaycastHit b)
                {
                    return a.distance.CompareTo(b.distance);
                });

            Vector3 physicsFloor = Vector3.zero;
            bool floorHit = false;

            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit hit = hits[i];

                if (hit.collider == null ||
                    hit.transform == null)
                {
                    continue;
                }

                if (IsOwnVisual(hit.collider.gameObject))
                    continue;

                if (IsTransformPartOfTarget(
                    hit.transform,
                    bedTarget))
                {
                    continue;
                }

                if (hit.normal.y < 0.45f)
                    continue;

                if (hit.point.y >= seatPoint.y - 0.05f)
                    continue;

                physicsFloor = hit.point;
                floorHit = true;
                break;
            }

            if (!floorHit)
            {
                rejectReason = ActionRejectReason.Floor;
                return false;
            }

            const int allNavMeshAreas = -1;
            NavMeshHit navHit;

            if (!NavMesh.SamplePosition(
                physicsFloor,
                out navHit,
                ActionNavMeshRadius,
                allNavMeshAreas))
            {
                rejectReason = ActionRejectReason.NavMesh;
                return false;
            }

            Vector2 physicsFloorXZ =
                new Vector2(
                    physicsFloor.x,
                    physicsFloor.z);

            Vector2 navFloorXZ =
                new Vector2(
                    navHit.position.x,
                    navHit.position.z);

            float navHorizontalOffset =
                Vector2.Distance(
                    physicsFloorXZ,
                    navFloorXZ);

            float navVerticalOffset =
                Mathf.Abs(
                    navHit.position.y -
                    physicsFloor.y);

            if (navHorizontalOffset >
                    ActionMaxNavMeshSnapDistance ||
                navVerticalOffset >
                    ActionMaxNavMeshVerticalOffset)
            {
                rejectReason =
                    ActionRejectReason.NavMeshOffset;

                return false;
            }

            floorPoint = navHit.position;
            seatFloorDrop =
                seatPoint.y - floorPoint.y;

            if (seatFloorDrop < ActionMinSeatFloorDrop ||
                seatFloorDrop > ActionMaxSeatFloorDrop)
            {
                rejectReason = ActionRejectReason.Drop;
                return false;
            }

            // 1) Can Mita stand at the NavMesh point without intersecting furniture?
            if (!IsStandingCapsuleClear(
                floorPoint,
                bedTarget))
            {
                rejectReason =
                    ActionRejectReason.BodyClearance;

                return false;
            }

            // 2) Is the short path from the stand point toward the bed edge clear?
            if (!IsApproachCorridorClear(
                floorPoint,
                boundaryPoint,
                outward,
                bedTarget))
            {
                rejectReason =
                    ActionRejectReason.CorridorClearance;

                return false;
            }

            // 3) Is there enough free volume for the legs in front of the seat?
            if (!IsLegSpaceClear(
                floorPoint,
                boundaryPoint,
                outward,
                bedTarget))
            {
                rejectReason =
                    ActionRejectReason.LegClearance;

                return false;
            }

            rejectReason = ActionRejectReason.None;
            return true;
        }

        private static bool IsStandingCapsuleClear(
            Vector3 floorPoint,
            Transform bedTarget)
        {
            Vector3 bottomCenter =
                floorPoint +
                Vector3.up *
                (ActionBodyRadius +
                 ActionBodyBottomClearance);

            Vector3 topCenter =
                floorPoint +
                Vector3.up *
                (ActionBodyHeight -
                 ActionBodyRadius +
                 ActionBodyBottomClearance);

            var overlaps = Physics.OverlapCapsule(
                bottomCenter,
                topCenter,
                ActionBodyRadius,
                -1,
                QueryTriggerInteraction.Ignore);

            return !ContainsBlockingCollider(
                overlaps,
                bedTarget,
                floorPoint.y);
        }

        private static bool IsApproachCorridorClear(
            Vector3 floorPoint,
            Vector3 boundaryPoint,
            Vector3 outward,
            Transform bedTarget)
        {
            Vector3 nearEdgePoint =
                boundaryPoint +
                outward *
                ActionCorridorNearEdgeClearance;

            nearEdgePoint.y = floorPoint.y;

            for (int i = 1;
                 i <= ActionCorridorSamples;
                 i++)
            {
                float t =
                    i /
                    (float)(ActionCorridorSamples + 1);

                Vector3 sampleFloor =
                    Vector3.Lerp(
                        floorPoint,
                        nearEdgePoint,
                        t);

                if (!IsStandingCapsuleClear(
                    sampleFloor,
                    bedTarget))
                {
                    return false;
                }
            }

            return true;
        }

        private static bool IsLegSpaceClear(
            Vector3 floorPoint,
            Vector3 boundaryPoint,
            Vector3 outward,
            Transform bedTarget)
        {
            outward.y = 0f;

            if (outward.sqrMagnitude < 0.001f)
                return false;

            outward.Normalize();

            Quaternion rotation =
                Quaternion.LookRotation(
                    outward,
                    Vector3.up);

            Vector3 center =
                boundaryPoint +
                outward *
                (ActionLegSpaceDepth * 0.5f + 0.04f);

            center.y =
                floorPoint.y +
                ActionLegSpaceBottomClearance +
                ActionLegSpaceHeight * 0.5f;

            Vector3 halfExtents =
                new Vector3(
                    ActionLegSpaceWidth * 0.5f,
                    ActionLegSpaceHeight * 0.5f,
                    ActionLegSpaceDepth * 0.5f);

            var overlaps = Physics.OverlapBox(
                center,
                halfExtents,
                rotation,
                -1,
                QueryTriggerInteraction.Ignore);

            return !ContainsBlockingCollider(
                overlaps,
                bedTarget,
                floorPoint.y);
        }

        private static bool ContainsBlockingCollider(
            Il2CppInterop.Runtime.InteropTypes.Arrays.Il2CppReferenceArray<Collider> overlaps,
            Transform bedTarget,
            float floorY)
        {
            if (overlaps == null ||
                overlaps.Length == 0)
            {
                return false;
            }

            for (int i = 0;
                 i < overlaps.Length;
                 i++)
            {
                Collider collider = overlaps[i];

                if (collider == null ||
                    !collider.enabled ||
                    collider.isTrigger)
                {
                    continue;
                }

                if (IsOwnVisual(collider.gameObject))
                    continue;

                if (IsTransformPartOfTarget(
                    collider.transform,
                    bedTarget))
                {
                    // The original Bed collider is known to differ from the visible/proxy shape.
                    // Do not let it invalidate proxy-based seatability.
                    continue;
                }

                if (IsLikelyCharacterOrPlayer(
                    collider.transform))
                {
                    // Keep the test deterministic even if Mita/player happens to stand nearby.
                    continue;
                }

                try
                {
                    if (collider.bounds.max.y <=
                        floorY +
                        ActionFloorLikeColliderTolerance)
                    {
                        continue;
                    }
                }
                catch { }

                return true;
            }

            return false;
        }

        private static bool IsLikelyCharacterOrPlayer(
            Transform transform)
        {
            Transform current = transform;

            while (current != null)
            {
                string name =
                    (current.name ?? "")
                    .ToLowerInvariant();

                if (name.Contains("mitaperson") ||
                    name.Contains("playerperson") ||
                    name == "player" ||
                    name.StartsWith("player "))
                {
                    return true;
                }

                current = current.parent;
            }

            return false;
        }

        private static float EstimateHeightfieldNormalY(
            HeightfieldStats heightfield,
            int x,
            int z,
            float cellSizeX,
            float cellSizeZ)
        {
            float center = heightfield.heights[x, z];

            float left = TryGetNeighbourHeight(
                heightfield,
                x - 1,
                z,
                center);

            float right = TryGetNeighbourHeight(
                heightfield,
                x + 1,
                z,
                center);

            float back = TryGetNeighbourHeight(
                heightfield,
                x,
                z - 1,
                center);

            float front = TryGetNeighbourHeight(
                heightfield,
                x,
                z + 1,
                center);

            float dx =
                (right - left) /
                Mathf.Max(0.001f, cellSizeX * 2f);

            float dz =
                (front - back) /
                Mathf.Max(0.001f, cellSizeZ * 2f);

            Vector3 normal = new Vector3(-dx, 1f, -dz).normalized;
            return normal.y;
        }

        private static float TryGetNeighbourHeight(
            HeightfieldStats heightfield,
            int x,
            int z,
            float fallback)
        {
            if (x < 0 || x >= MeshGrid ||
                z < 0 || z >= MeshGrid)
            {
                return fallback;
            }

            if (!heightfield.surfaceMask[x, z])
                return fallback;

            return heightfield.heights[x, z];
        }

        private static void EvaluateSeatFootprint(
            HeightfieldStats heightfield,
            int centerX,
            int centerZ,
            int radiusX,
            int radiusZ,
            out float coverage,
            out float heightRange)
        {
            int total = 0;
            int supported = 0;

            float minHeight = float.MaxValue;
            float maxHeight = float.MinValue;

            for (int dz = -radiusZ; dz <= radiusZ; dz++)
            {
                for (int dx = -radiusX; dx <= radiusX; dx++)
                {
                    total++;

                    int x = centerX + dx;
                    int z = centerZ + dz;

                    if (x < 0 || x >= MeshGrid ||
                        z < 0 || z >= MeshGrid)
                    {
                        continue;
                    }

                    if (!heightfield.surfaceMask[x, z])
                        continue;

                    supported++;

                    float h = heightfield.heights[x, z];
                    if (h < minHeight) minHeight = h;
                    if (h > maxHeight) maxHeight = h;
                }
            }

            coverage =
                total > 0
                    ? supported / (float)total
                    : 0f;

            heightRange =
                supported > 0
                    ? maxHeight - minHeight
                    : float.MaxValue;
        }

        private static bool TryEstimateFloorYAroundBed(
            GameObject bed,
            Bounds bedBounds,
            float surfaceMinY,
            out float floorY)
        {
            floorY = 0f;
            List<float> samples = new List<float>();

            float margin = 0.22f;
            float topY = bedBounds.max.y + 1.00f;

            Vector3[] xzSamples =
            {
                new Vector3(bedBounds.min.x - margin, topY, bedBounds.center.z),
                new Vector3(bedBounds.max.x + margin, topY, bedBounds.center.z),
                new Vector3(bedBounds.center.x, topY, bedBounds.min.z - margin),
                new Vector3(bedBounds.center.x, topY, bedBounds.max.z + margin),

                new Vector3(bedBounds.min.x - margin, topY, bedBounds.min.z - margin),
                new Vector3(bedBounds.min.x - margin, topY, bedBounds.max.z + margin),
                new Vector3(bedBounds.max.x + margin, topY, bedBounds.min.z - margin),
                new Vector3(bedBounds.max.x + margin, topY, bedBounds.max.z + margin)
            };

            for (int i = 0; i < xzSamples.Length; i++)
            {
                RaycastHit[] hits = null;

                try
                {
                    hits = Physics.RaycastAll(
                        xzSamples[i],
                        Vector3.down,
                        8f,
                        -1,
                        QueryTriggerInteraction.Ignore);
                }
                catch { }

                if (hits == null || hits.Length == 0)
                    continue;

                Array.Sort(
                    hits,
                    delegate (RaycastHit a, RaycastHit b)
                    {
                        return a.distance.CompareTo(b.distance);
                    });

                for (int h = 0; h < hits.Length; h++)
                {
                    RaycastHit hit = hits[h];

                    if (hit.collider == null ||
                        hit.transform == null)
                    {
                        continue;
                    }

                    if (IsOwnVisual(hit.collider.gameObject))
                        continue;

                    if (IsTransformPartOfTarget(
                        hit.transform,
                        bed.transform))
                    {
                        continue;
                    }

                    if (hit.normal.y < 0.45f)
                        continue;

                    if (hit.point.y >= surfaceMinY - 0.06f)
                        continue;

                    samples.Add(hit.point.y);
                    break;
                }
            }

            if (samples.Count == 0)
            {
                // NavMesh fallback near the four sides of the bed.
                const int allNavMeshAreas = -1;

                for (int i = 0; i < xzSamples.Length; i++)
                {
                    NavMeshHit navHit;

                    if (NavMesh.SamplePosition(
                        xzSamples[i],
                        out navHit,
                        2.0f,
                        allNavMeshAreas))
                    {
                        if (navHit.position.y < surfaceMinY - 0.06f)
                            samples.Add(navHit.position.y);
                    }
                }
            }

            if (samples.Count == 0)
                return false;

            samples.Sort();
            floorY = samples[samples.Count / 2];
            return true;
        }

        private static bool IsTransformPartOfTarget(
            Transform candidate,
            Transform target)
        {
            if (candidate == null || target == null)
                return false;

            return
                candidate == target ||
                candidate.IsChildOf(target) ||
                target.IsChildOf(candidate);
        }

    }
}
