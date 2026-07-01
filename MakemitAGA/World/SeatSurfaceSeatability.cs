/*
 * =================================================================================================
 * SeatSurfaceSeatability.cs
 * =================================================================================================
 *
 * 作用：把高度图转换成“能否坐、是否适合动作”的物理判定数据。

 * 主要逻辑：
 *   - 支撑覆盖率与法线/坡度检查；
 *   - 座面边缘和外侧方向估计（同时识别无表面轮廓与向下高度断层）；
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
            int serial,
            bool manualDebugTest = false)
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
                    if (IsSeatabilityAnalysisCancelled(
                        serial,
                        manualDebugTest))
                    {
                        yield break;
                    }

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

            if (!manualDebugTest)
                _scanStage = "ActionAnalysis";

            processedThisFrame = 0;

            // Second pass: expensive edge, floor, NavMesh and clearance checks.
            for (int z = 0; z < MeshGrid; z++)
            {
                for (int x = 0; x < MeshGrid; x++)
                {
                    if (IsSeatabilityAnalysisCancelled(
                        serial,
                        manualDebugTest))
                    {
                        yield break;
                    }

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

                            // “紫色动作带”只限制臀部候选点在座面边缘内侧的深度。
                            //
                            // v0.3.4 将这条动作带整体向外侧移动：最小内缩减小，
                            // 最大内缩同时收窄。这样既允许臀部更靠近床沿，又避免紫色
                            // 深入床中央太多。前一阶段还轻微放宽了软垫圆角的坡度/起伏，
                            // 因此原先紧邻紫色的部分温和红区有机会进入此处继续接受安全检查。
                            //
                            // 这里通过之后仍会执行 TryFindActionFloorPoint()，后者会继续检查：
                            //   - 边缘外是否存在真实地板；
                            //   - NavMesh 是否可达且吸附偏移合理；
                            //   - 站立身体胶囊是否会撞到家具；
                            //   - 从站立点走向床边的短走廊是否畅通；
                            //   - 坐姿腿部空间是否足够。
                            //
                            // 因此扩大紫色带不会把靠墙、靠床头柜、无地板或会穿模的位置
                            // 直接放行；它只是不再要求臀部深度必须非常精确。
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


        private static bool IsSeatabilityAnalysisCancelled(
            int serial,
            bool manualDebugTest)
        {
            return manualDebugTest
                ? serial != _manualDebugScanSerial
                : serial != _scanSerial;
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

        /// <summary>
        /// 从一个已经通过绿色承重检查的网格中心，沿 X/Z 四个方向寻找最近的
        /// “当前座面平台边缘”，并返回该边缘距离以及朝向家具外侧的方向。
        ///
        /// 边缘现在有两种来源：
        ///
        /// A. 几何轮廓边缘
        ///    - 搜索走出高度图范围；或
        ///    - 下一格没有任何扫描表面。
        ///
        /// B. 高度断层边缘
        ///    - 下一格仍然存在扫描表面；
        ///    - 但它比候选座面低至少 ActionBoundaryMinDownwardDrop。
        ///
        /// B 是本次修复的核心。完整家具扫描会同时保留床垫和较低床架；床垫到边时，
        /// surfaceMask 仍然可能为 true，因此只靠 A 会把床架最外侧误认为床垫边缘。
        ///
        /// 这里只把“向下落差”当作边缘：
        ///    centerHeight - sampledHeight >= threshold
        /// 向上升高通常代表靠背、扶手或枕头，不应该直接被解释为米塔面向的外侧。
        /// 即使某个内部高度层被暂时识别为候选边缘，后续的地板、NavMesh、身体、
        /// 接近走廊与腿部空间检查仍会负责淘汰无法真正执行坐姿的方向。
        /// </summary>
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

            // 调用者只会传入绿色有效中心，但仍做防御检查，避免以后复用此方法时
            // 因无表面中心或非法索引读到默认高度，产生一个完全错误的边缘方向。
            if (centerX < 0 ||
                centerX >= MeshGrid ||
                centerZ < 0 ||
                centerZ >= MeshGrid ||
                !heightfield.surfaceMask[centerX, centerZ])
            {
                return false;
            }

            float centerHeight =
                heightfield.heights[centerX, centerZ];

            // 仅检查四个主轴方向，与旧行为保持一致。
            // 这样不会在本次修复中同时改变动作方向的离散规则，便于单独验证
            // “高度断层是否恢复 Bed 紫色区域”。未来如需斜向边缘，可另开独立改动。
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

                    bool outsideGrid =
                        x < 0 ||
                        x >= MeshGrid ||
                        z < 0 ||
                        z >= MeshGrid;

                    bool missingSurface =
                        !outsideGrid &&
                        !heightfield.surfaceMask[x, z];

                    bool downwardHeightBoundary = false;

                    if (!outsideGrid &&
                        !missingSurface)
                    {
                        float sampledHeight =
                            heightfield.heights[x, z];

                        // 使用候选座面中心高度作为“当前平台”的参考高度，而不是使用
                        // 上一个格子的高度。这样即使边缘经过平滑后分散到两三个网格，
                        // 累积落差仍能被识别；同时我们只接受向下落差，向上的靠背不会
                        // 被误认成外侧。
                        float downwardDrop =
                            centerHeight - sampledHeight;

                        downwardHeightBoundary =
                            downwardDrop >=
                            ActionBoundaryMinDownwardDrop;
                    }

                    bool reachedBoundary =
                        outsideGrid ||
                        missingSurface ||
                        downwardHeightBoundary;

                    if (!reachedBoundary)
                        continue;

                    // 当前 step 指向的是“边缘外侧第一格”或“明显更低的第一格”。
                    // 因此把边界近似放在上一格与当前格的中间，继续沿用旧公式：
                    //   (step - 0.5) * cellSize
                    // 这能保持 ActionMinEdgeInset / ActionMaxEdgeInset 的原有标定意义。
                    float distance =
                        Mathf.Max(0f, step - 0.5f) *
                        stepSizes[d];

                    if (distance < edgeDistance)
                    {
                        edgeDistance = distance;
                        outward = worldDirs[d];
                    }

                    // 每个方向只取遇到的第一个边缘。继续向外搜索会跨过当前座面平台，
                    // 很可能再次找到床架或家具整体轮廓，反而回到旧问题。
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