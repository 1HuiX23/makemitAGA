using System;
using System.Collections.Generic;
using System.Text;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// 对已经统一到世界坐标的托管三角形数据执行：
    /// 1. 位置容差焊接；
    /// 2. 按共享边拆分连通组件；
    /// 3. 按 AABB 距离把相邻组件归并为现实空间实例候选；
    /// 4. 把观察实例关联到最近的碰撞实例。
    ///
    /// 本类只读托管数组，不创建 Unity Mesh、Collider、Rigidbody 或 PhysicMaterial。
    /// </summary>
    internal static class SpatialInstanceAnalyzer
    {
        private sealed class UnionFind
        {
            private readonly int[] _parent;
            private readonly byte[] _rank;

            public UnionFind(int count)
            {
                _parent = new int[count];
                _rank = new byte[count];
                for (int i = 0; i < count; i++)
                    _parent[i] = i;
            }

            public int Find(int value)
            {
                int root = value;
                while (_parent[root] != root)
                    root = _parent[root];
                while (_parent[value] != value)
                {
                    int next = _parent[value];
                    _parent[value] = root;
                    value = next;
                }
                return root;
            }

            public void Union(int a, int b)
            {
                int ra = Find(a);
                int rb = Find(b);
                if (ra == rb)
                    return;
                if (_rank[ra] < _rank[rb])
                    _parent[ra] = rb;
                else if (_rank[ra] > _rank[rb])
                    _parent[rb] = ra;
                else
                {
                    _parent[rb] = ra;
                    _rank[ra]++;
                }
            }
        }

        private sealed class EdgeInfo
        {
            public int Count;
            public int ForwardCount;
            public int FirstTriangle = -1;
        }

        private readonly struct QuantizedKey : IEquatable<QuantizedKey>
        {
            public readonly long X;
            public readonly long Y;
            public readonly long Z;

            public QuantizedKey(long x, long y, long z)
            {
                X = x;
                Y = y;
                Z = z;
            }

            public bool Equals(QuantizedKey other)
            {
                return X == other.X && Y == other.Y && Z == other.Z;
            }

            public override bool Equals(object obj)
            {
                return obj is QuantizedKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = hash * 31 + X.GetHashCode();
                    hash = hash * 31 + Y.GetHashCode();
                    hash = hash * 31 + Z.GetHashCode();
                    return hash;
                }
            }
        }

        private sealed class ComponentData
        {
            public SpatialComponentReport Report;
            public Bounds Bounds;
            public readonly List<int> Triangles = new List<int>();
            public readonly HashSet<int> CanonicalVertices = new HashSet<int>();
            public readonly HashSet<int> RouteIds = new HashSet<int>();
            public readonly HashSet<string> Renderers = new HashSet<string>(StringComparer.Ordinal);
            public readonly HashSet<string> Roles = new HashSet<string>(StringComparer.Ordinal);
            public bool HasPhysical;
            public bool HasVisualOnly;
        }

        private sealed class InstanceData
        {
            public SpatialInstanceReport Report;
            public Bounds Bounds;
            public readonly List<ComponentData> Components = new List<ComponentData>();
            public readonly HashSet<int> CanonicalVertices = new HashSet<int>();
            public readonly HashSet<int> RouteIds = new HashSet<int>();
            public readonly HashSet<string> Renderers = new HashSet<string>(StringComparer.Ordinal);
            public readonly HashSet<string> Roles = new HashSet<string>(StringComparer.Ordinal);
        }

        public static SpatialInstanceAuditReport Analyze(
            string channel,
            string targetPath,
            List<Vector3> positions,
            List<int> indices,
            List<int> triangleRouteIds,
            List<MultipartPartReport> parts,
            float weldTolerance)
        {
            SpatialInstanceAuditReport audit = new SpatialInstanceAuditReport
            {
                Channel = channel,
                WeldTolerance = Mathf.Max(0.000001f, weldTolerance),
                InputVertices = positions != null ? positions.Count : 0,
                InputTriangles = indices != null ? indices.Count / 3 : 0,
                Detail = "starting"
            };

            if (positions == null || indices == null ||
                positions.Count == 0 || indices.Count < 3)
            {
                audit.Detail = "no-geometry";
                return audit;
            }

            int canonicalCount;
            List<Vector3> canonicalPositions;
            int[] weldMap = BuildWeldMap(
                positions,
                audit.WeldTolerance,
                out canonicalCount,
                out canonicalPositions);
            audit.WeldedVertices = canonicalCount;

            int triangleCount = indices.Count / 3;
            UnionFind triangleUnion = new UnionFind(triangleCount);
            bool[] validTriangles = new bool[triangleCount];
            int[] mappedA = new int[triangleCount];
            int[] mappedB = new int[triangleCount];
            int[] mappedC = new int[triangleCount];
            Dictionary<ulong, EdgeInfo> globalEdges =
                new Dictionary<ulong, EdgeInfo>(Math.Max(16, triangleCount * 2));

            for (int triangle = 0; triangle < triangleCount; triangle++)
            {
                int ia = indices[triangle * 3];
                int ib = indices[triangle * 3 + 1];
                int ic = indices[triangle * 3 + 2];
                if (ia < 0 || ib < 0 || ic < 0 ||
                    ia >= positions.Count || ib >= positions.Count || ic >= positions.Count)
                    continue;
                int a = weldMap[ia];
                int b = weldMap[ib];
                int c = weldMap[ic];
                if (a == b || b == c || c == a)
                    continue;
                Vector3 pa = canonicalPositions[a];
                Vector3 pb = canonicalPositions[b];
                Vector3 pc = canonicalPositions[c];
                if (Vector3.Cross(pb - pa, pc - pa).sqrMagnitude <= 1e-16f)
                    continue;

                validTriangles[triangle] = true;
                mappedA[triangle] = a;
                mappedB[triangle] = b;
                mappedC[triangle] = c;
                audit.ValidTriangles++;
                AddGlobalEdge(globalEdges, triangleUnion, a, b, triangle);
                AddGlobalEdge(globalEdges, triangleUnion, b, c, triangle);
                AddGlobalEdge(globalEdges, triangleUnion, c, a, triangle);
            }

            Dictionary<int, ComponentData> byRoot =
                new Dictionary<int, ComponentData>();
            for (int triangle = 0; triangle < triangleCount; triangle++)
            {
                if (!validTriangles[triangle])
                    continue;
                int root = triangleUnion.Find(triangle);
                if (!byRoot.TryGetValue(root, out ComponentData data))
                {
                    data = new ComponentData();
                    byRoot.Add(root, data);
                }
                data.Triangles.Add(triangle);
            }

            Dictionary<int, MultipartPartReport> partBySequence =
                BuildPartMap(parts);
            List<ComponentData> components = new List<ComponentData>(byRoot.Values);
            for (int i = 0; i < components.Count; i++)
            {
                BuildComponentReport(
                    components[i],
                    channel,
                    targetPath,
                    positions,
                    indices,
                    triangleRouteIds,
                    partBySequence,
                    mappedA,
                    mappedB,
                    mappedC,
                    canonicalPositions);
            }

            components.Sort(CompareComponents);
            for (int i = 0; i < components.Count; i++)
            {
                components[i].Report.ComponentIndex = i + 1;
                components[i].Report.ComponentKey = BuildStableKey(
                    "SC",
                    targetPath,
                    channel,
                    components[i].Bounds,
                    components[i].RouteIds,
                    components[i].Report.Triangles);
                audit.Components.Add(components[i].Report);
            }

            audit.ConnectedComponents = components.Count;
            List<InstanceData> instances = BuildSpatialInstances(
                components,
                channel,
                targetPath);
            instances.Sort(CompareInstances);
            for (int i = 0; i < instances.Count; i++)
            {
                FinalizeInstanceReport(instances[i], i + 1, channel, targetPath, canonicalPositions);
                audit.Instances.Add(instances[i].Report);
                if (instances[i].Report.NeedsUnityExplorerReview)
                    audit.UnityExplorerReviewInstances++;
            }

            audit.SpatialInstances = audit.Instances.Count;
            audit.ProxyDrafts = 0;
            for (int i = 0; i < audit.Instances.Count; i++)
            {
                SpatialInstanceReport item = audit.Instances[i];
                if (item != null && item.ProxyDraftBuilt)
                    audit.ProxyDrafts++;
            }
            audit.Analyzed = true;
            audit.Detail = "components-and-spatial-instances-built";
            return audit;
        }

        public static void AssociateObservationToCollision(
            SpatialInstanceAuditReport observation,
            SpatialInstanceAuditReport collision)
        {
            if (observation == null || !observation.Analyzed)
                return;

            observation.AssociatedToCollisionInstances = 0;
            observation.UnassociatedObservationInstances = 0;
            for (int i = 0; i < observation.Instances.Count; i++)
            {
                SpatialInstanceReport item = observation.Instances[i];
                if (item == null)
                    continue;

                SpatialInstanceReport best = null;
                float bestGap = float.PositiveInfinity;
                if (collision != null && collision.Analyzed)
                {
                    Bounds a = new Bounds(item.Center, item.Size);
                    for (int c = 0; c < collision.Instances.Count; c++)
                    {
                        SpatialInstanceReport candidate = collision.Instances[c];
                        if (candidate == null)
                            continue;
                        Bounds b = new Bounds(candidate.Center, candidate.Size);
                        float gap = BoundsGapDistance(a, b);
                        if (gap < bestGap)
                        {
                            bestGap = gap;
                            best = candidate;
                        }
                    }
                }

                float associationLimit = Mathf.Max(
                    0.015f,
                    Mathf.Min(0.10f, item.MaxDimension * 0.20f));
                if (best != null && bestGap <= associationLimit)
                {
                    item.AssociatedCollisionInstanceKey = best.InstanceKey;
                    item.CollisionAssociationGap = bestGap;
                    item.AssociationStatus = bestGap <= 0.00001f
                        ? "OVERLAPS_COLLISION_INSTANCE"
                        : "NEAR_COLLISION_INSTANCE";
                    observation.AssociatedToCollisionInstances++;
                }
                else
                {
                    item.AssociatedCollisionInstanceKey = null;
                    item.CollisionAssociationGap = float.IsInfinity(bestGap) ? -1f : bestGap;
                    item.AssociationStatus = collision == null || !collision.Analyzed ||
                        collision.Instances.Count == 0
                        ? "NO_COLLISION_INSTANCES"
                        : "NO_COLLISION_ASSOCIATION_WITHIN_LIMIT";
                    observation.UnassociatedObservationInstances++;
                    if (!item.ContainsPhysicalRoute)
                    {
                        item.NeedsUnityExplorerReview = true;
                        item.ReviewReason = AppendReason(
                            item.ReviewReason,
                            "没有关联碰撞实例；请在 Unity Explorer 中确认它是画面、玻璃、阴影、特效还是应补建物理代理的实体");
                    }
                }
            }

            observation.UnityExplorerReviewInstances = 0;
            for (int i = 0; i < observation.Instances.Count; i++)
            {
                SpatialInstanceReport item = observation.Instances[i];
                if (item != null && item.NeedsUnityExplorerReview)
                    observation.UnityExplorerReviewInstances++;
            }
        }

        private static Dictionary<int, MultipartPartReport> BuildPartMap(
            List<MultipartPartReport> parts)
        {
            Dictionary<int, MultipartPartReport> map =
                new Dictionary<int, MultipartPartReport>();
            if (parts == null)
                return map;
            for (int i = 0; i < parts.Count; i++)
            {
                MultipartPartReport part = parts[i];
                if (part != null && part.Sequence > 0)
                    map[part.Sequence] = part;
            }
            return map;
        }

        private static void BuildComponentReport(
            ComponentData data,
            string channel,
            string targetPath,
            List<Vector3> positions,
            List<int> indices,
            List<int> triangleRouteIds,
            Dictionary<int, MultipartPartReport> partBySequence,
            int[] mappedA,
            int[] mappedB,
            int[] mappedC,
            List<Vector3> canonicalPositions)
        {
            Dictionary<ulong, EdgeInfo> edges =
                new Dictionary<ulong, EdgeInfo>(Math.Max(16, data.Triangles.Count * 2));
            Dictionary<int, List<int>> boundaryAdjacency =
                new Dictionary<int, List<int>>();
            bool haveBounds = false;
            Bounds bounds = new Bounds();
            float area = 0f;

            for (int i = 0; i < data.Triangles.Count; i++)
            {
                int triangle = data.Triangles[i];
                int a = mappedA[triangle];
                int b = mappedB[triangle];
                int c = mappedC[triangle];
                data.CanonicalVertices.Add(a);
                data.CanonicalVertices.Add(b);
                data.CanonicalVertices.Add(c);
                AddLocalEdge(edges, a, b);
                AddLocalEdge(edges, b, c);
                AddLocalEdge(edges, c, a);

                Vector3 pa = canonicalPositions[a];
                Vector3 pb = canonicalPositions[b];
                Vector3 pc = canonicalPositions[c];
                area += Vector3.Cross(pb - pa, pc - pa).magnitude * 0.5f;
                if (!haveBounds)
                {
                    bounds = new Bounds(pa, Vector3.zero);
                    haveBounds = true;
                }
                bounds.Encapsulate(pa);
                bounds.Encapsulate(pb);
                bounds.Encapsulate(pc);

                int routeId = triangleRouteIds != null && triangle < triangleRouteIds.Count
                    ? triangleRouteIds[triangle]
                    : 0;
                if (routeId > 0)
                {
                    data.RouteIds.Add(routeId);
                    if (partBySequence.TryGetValue(routeId, out MultipartPartReport part))
                    {
                        if (!string.IsNullOrEmpty(part.RendererName))
                            data.Renderers.Add(part.RendererName);
                        if (!string.IsNullOrEmpty(part.ResolvedRole))
                            data.Roles.Add(part.ResolvedRole);
                        if (IsPhysicalRole(part.ResolvedRole))
                            data.HasPhysical = true;
                        if (string.Equals(part.ResolvedRole, "VISUAL_ONLY", StringComparison.Ordinal))
                            data.HasVisualOnly = true;
                    }
                }
            }

            int boundary = 0;
            int nonManifold = 0;
            int orientationConflict = 0;
            foreach (KeyValuePair<ulong, EdgeInfo> pair in edges)
            {
                EdgeInfo edge = pair.Value;
                if (edge.Count == 1)
                {
                    boundary++;
                    DecodeEdge(pair.Key, out int a, out int b);
                    AddAdjacency(boundaryAdjacency, a, b);
                    AddAdjacency(boundaryAdjacency, b, a);
                }
                else if (edge.Count > 2)
                {
                    nonManifold++;
                }
                if (edge.Count == 2 && edge.ForwardCount != 1)
                    orientationConflict++;
            }

            CountBoundaryComponents(
                boundaryAdjacency,
                out int loops,
                out int openChains);
            Vector3 size = haveBounds ? bounds.size : Vector3.zero;
            float maxDimension = Mathf.Max(size.x, Mathf.Max(size.y, size.z));
            float minDimension = Mathf.Min(size.x, Mathf.Min(size.y, size.z));
            float thinRatio = maxDimension > 0f ? minDimension / maxDimension : 0f;
            bool watertight = boundary == 0 && nonManifold == 0 && orientationConflict == 0;
            string classification;
            if (watertight)
                classification = "CLOSED_COMPONENT";
            else if (thinRatio <= 0.02f)
                classification = "THIN_OPEN_SHEET";
            else if (nonManifold > 0 || orientationConflict > 0)
                classification = "TOPOLOGY_CONFLICT_COMPONENT";
            else
                classification = "OPEN_SURFACE_COMPONENT";

            bool review = false;
            string reviewReason = null;
            if (data.HasPhysical && !watertight)
            {
                review = true;
                reviewReason = "物理候选仍有开放边界；需要确认是有意开口、独立按钮/底座还是建模缺口";
            }
            if (!data.HasPhysical && data.HasVisualOnly)
            {
                review = true;
                reviewReason = AppendReason(
                    reviewReason,
                    "纯视觉组件；需要确认是画面、玻璃、阴影或其他特效");
            }
            if (data.RouteIds.Count > 1)
            {
                review = true;
                reviewReason = AppendReason(
                    reviewReason,
                    "同一连通组件跨越多条 Renderer/SubMesh 路由");
            }

            data.Bounds = bounds;
            data.Report = new SpatialComponentReport
            {
                Channel = channel,
                Triangles = data.Triangles.Count,
                CanonicalVertices = data.CanonicalVertices.Count,
                BoundaryEdges = boundary,
                BoundaryLoops = loops,
                OpenBoundaryChains = openChains,
                NonManifoldEdges = nonManifold,
                OrientationConflicts = orientationConflict,
                Watertight = watertight,
                SurfaceArea = area,
                MaxDimension = maxDimension,
                MinDimension = minDimension,
                ThinRatio = thinRatio,
                Bounds = FormatBounds(bounds),
                Center = bounds.center,
                Size = bounds.size,
                SourceRouteIds = JoinInts(data.RouteIds),
                SourceRenderers = JoinStrings(data.Renderers),
                SourceRoles = JoinStrings(data.Roles),
                Classification = classification,
                NeedsUnityExplorerReview = review,
                ReviewReason = reviewReason
            };
        }

        private static List<InstanceData> BuildSpatialInstances(
            List<ComponentData> components,
            string channel,
            string targetPath)
        {
            List<InstanceData> result = new List<InstanceData>();
            if (components == null || components.Count == 0)
                return result;

            UnionFind componentUnion = new UnionFind(components.Count);
            for (int a = 0; a < components.Count; a++)
            {
                for (int b = a + 1; b < components.Count; b++)
                {
                    if (ShouldLinkComponents(components[a], components[b], out float ignoredGap))
                        componentUnion.Union(a, b);
                }
            }

            Dictionary<int, InstanceData> byRoot =
                new Dictionary<int, InstanceData>();
            for (int i = 0; i < components.Count; i++)
            {
                int root = componentUnion.Find(i);
                if (!byRoot.TryGetValue(root, out InstanceData instance))
                {
                    instance = new InstanceData();
                    byRoot.Add(root, instance);
                }
                instance.Components.Add(components[i]);
            }

            foreach (InstanceData instance in byRoot.Values)
            {
                bool haveBounds = false;
                float maxAcceptedGap = 0f;
                for (int i = 0; i < instance.Components.Count; i++)
                {
                    ComponentData component = instance.Components[i];
                    if (!haveBounds)
                    {
                        instance.Bounds = component.Bounds;
                        haveBounds = true;
                    }
                    else
                    {
                        instance.Bounds.Encapsulate(component.Bounds);
                    }
                    foreach (int vertex in component.CanonicalVertices)
                        instance.CanonicalVertices.Add(vertex);
                    foreach (int route in component.RouteIds)
                        instance.RouteIds.Add(route);
                    foreach (string renderer in component.Renderers)
                        instance.Renderers.Add(renderer);
                    foreach (string role in component.Roles)
                        instance.Roles.Add(role);
                }

                for (int a = 0; a < instance.Components.Count; a++)
                {
                    for (int b = a + 1; b < instance.Components.Count; b++)
                    {
                        if (ShouldLinkComponents(
                            instance.Components[a],
                            instance.Components[b],
                            out float gap))
                            maxAcceptedGap = Mathf.Max(maxAcceptedGap, gap);
                    }
                }

                instance.Report = new SpatialInstanceReport
                {
                    Channel = channel,
                    Components = instance.Components.Count,
                    CanonicalVertices = instance.CanonicalVertices.Count,
                    MaximumAcceptedLinkGap = maxAcceptedGap,
                    Bounds = FormatBounds(instance.Bounds),
                    Center = instance.Bounds.center,
                    Size = instance.Bounds.size,
                    SourceRouteIds = JoinInts(instance.RouteIds),
                    SourceRenderers = JoinStrings(instance.Renderers),
                    SourceRoles = JoinStrings(instance.Roles),
                    AssociationStatus = string.Equals(channel, "collision", StringComparison.Ordinal)
                        ? "COLLISION_SOURCE_INSTANCE"
                        : "PENDING_COLLISION_ASSOCIATION"
                };
                result.Add(instance);
            }
            return result;
        }

        private static bool ShouldLinkComponents(
            ComponentData a,
            ComponentData b,
            out float gap)
        {
            gap = BoundsGapDistance(a.Bounds, b.Bounds);
            float scaleA = Mathf.Max(a.Bounds.size.x, Mathf.Max(a.Bounds.size.y, a.Bounds.size.z));
            float scaleB = Mathf.Max(b.Bounds.size.x, Mathf.Max(b.Bounds.size.y, b.Bounds.size.z));
            float smaller = Mathf.Max(0.0001f, Mathf.Min(scaleA, scaleB));
            float limit = Mathf.Clamp(smaller * 0.12f, 0.003f, 0.06f);

            bool thinA = a.Report != null && a.Report.ThinRatio <= 0.03f;
            bool thinB = b.Report != null && b.Report.ThinRatio <= 0.03f;
            if (thinA || thinB)
                limit = Mathf.Max(limit, Mathf.Min(0.05f, smaller * 0.30f));

            if (gap > limit)
                return false;

            float centerDistance = Vector3.Distance(a.Bounds.center, b.Bounds.center);
            float larger = Mathf.Max(scaleA, scaleB);
            if (centerDistance > larger * 1.75f + limit)
                return false;

            return true;
        }

        private static void FinalizeInstanceReport(
            InstanceData instance,
            int index,
            string channel,
            string targetPath,
            List<Vector3> canonicalPositions)
        {
            SpatialInstanceReport report = instance.Report;
            report.InstanceIndex = index;
            report.InstanceKey = BuildStableKey(
                "SI",
                targetPath,
                channel,
                instance.Bounds,
                instance.RouteIds,
                instance.Components.Count);

            List<int> componentIds = new List<int>();
            float area = 0f;
            int triangles = 0;
            int closed = 0;
            int open = 0;
            bool hasPhysical = false;
            bool hasVisual = false;
            bool review = false;
            string reviewReason = null;
            for (int i = 0; i < instance.Components.Count; i++)
            {
                SpatialComponentReport component = instance.Components[i].Report;
                if (component == null)
                    continue;
                componentIds.Add(component.ComponentIndex);
                area += component.SurfaceArea;
                triangles += component.Triangles;
                if (component.Watertight)
                    closed++;
                else
                    open++;
                if (instance.Components[i].HasPhysical)
                    hasPhysical = true;
                if (instance.Components[i].HasVisualOnly)
                    hasVisual = true;
                if (component.NeedsUnityExplorerReview)
                {
                    review = true;
                    reviewReason = AppendReason(reviewReason, component.ReviewReason);
                }
            }

            Vector3 size = instance.Bounds.size;
            float maxDimension = Mathf.Max(size.x, Mathf.Max(size.y, size.z));
            float minDimension = Mathf.Min(size.x, Mathf.Min(size.y, size.z));
            float thinRatio = maxDimension > 0f ? minDimension / maxDimension : 0f;

            string recommendedRole;
            if (string.Equals(channel, "collision", StringComparison.Ordinal))
            {
                recommendedRole = open == 0
                    ? "CLOSED_PHYSICAL_INSTANCE_CANDIDATE"
                    : "PHYSICAL_PROXY_OR_REPAIR_REQUIRED";
            }
            else if (hasPhysical && hasVisual)
            {
                recommendedRole = "PHYSICAL_INSTANCE_WITH_VISUAL_OVERLAY";
            }
            else if (hasPhysical)
            {
                recommendedRole = "OBSERVATION_OF_PHYSICAL_INSTANCE";
            }
            else if (thinRatio <= 0.03f)
            {
                recommendedRole = "VISUAL_SHEET_OR_EFFECT_INSTANCE";
            }
            else
            {
                recommendedRole = "VISUAL_DECORATION_INSTANCE";
            }

            if (instance.Components.Count > 1 && open > 0)
            {
                review = true;
                reviewReason = AppendReason(
                    reviewReason,
                    "多个开放组件因空间接近被归为同一实例；请确认它们是否属于同一现实物体");
            }
            if (!hasPhysical && string.Equals(channel, "observation", StringComparison.Ordinal))
            {
                review = true;
                reviewReason = AppendReason(
                    reviewReason,
                    "当前实例只有视觉路由，没有物理主体");
            }

            report.ComponentIds = JoinInts(componentIds);
            report.Triangles = triangles;
            report.WatertightComponents = closed;
            report.OpenComponents = open;
            report.SurfaceArea = area;
            report.MaxDimension = maxDimension;
            report.MinDimension = minDimension;
            report.ThinRatio = thinRatio;
            report.ContainsPhysicalRoute = hasPhysical;
            report.ContainsVisualOnlyRoute = hasVisual;
            report.RecommendedRole = recommendedRole;
            report.NeedsUnityExplorerReview = review;
            report.ReviewReason = reviewReason;

            if (string.Equals(channel, "collision", StringComparison.Ordinal))
            {
                BuildOrientedBoxProxyDraft(instance, canonicalPositions, report);
                TryBuildLayeredStructureCompoundDraft(instance, report);
            }
        }


        /// <summary>
        /// 通用识别“两个或更多水平浅层板 + 一个覆盖主要范围的高框架”结构。
        /// 水平层板分别近似为薄盒；覆盖框架只生成四个角柱审计草案，避免
        /// 单一大盒封死层间开放空间。该方法不读取对象名称、不创建 Unity 组件，
        /// 结果始终标记为需要人工复核。
        /// </summary>
        private static void TryBuildLayeredStructureCompoundDraft(
            InstanceData instance,
            SpatialInstanceReport report)
        {
            if (instance == null || report == null || instance.Components.Count < 3)
                return;

            Vector3 whole = instance.Bounds.size;
            if (whole.y <= 0.40f || whole.x <= 0.25f || whole.z <= 0.25f)
                return;

            List<ComponentData> horizontalSlabs = new List<ComponentData>();
            ComponentData enclosingFrame = null;
            for (int i = 0; i < instance.Components.Count; i++)
            {
                ComponentData component = instance.Components[i];
                if (component == null || component.Report == null)
                    continue;
                Vector3 size = component.Bounds.size;

                bool slabLike = component.Report.Watertight &&
                    size.y <= whole.y * 0.22f &&
                    size.x >= whole.x * 0.55f &&
                    size.z >= whole.z * 0.55f &&
                    component.Report.SurfaceArea >= 0.08f;
                if (slabLike)
                {
                    horizontalSlabs.Add(component);
                    continue;
                }

                bool enclosingFrameLike = component.Report.Watertight &&
                    size.y >= whole.y * 0.70f &&
                    size.x >= whole.x * 0.70f &&
                    size.z >= whole.z * 0.70f;
                if (enclosingFrameLike &&
                    (enclosingFrame == null || size.y > enclosingFrame.Bounds.size.y))
                {
                    enclosingFrame = component;
                }
            }

            if (horizontalSlabs.Count < 2 || enclosingFrame == null)
                return;

            horizontalSlabs.Sort((a, b) =>
                a.Bounds.center.y.CompareTo(b.Bounds.center.y));
            report.CompoundParts.Clear();
            float slabMargin = Mathf.Clamp(
                Mathf.Max(whole.x, whole.z) * 0.004f,
                0.002f,
                0.006f);
            for (int i = 0; i < horizontalSlabs.Count; i++)
            {
                ComponentData slab = horizontalSlabs[i];
                Vector3 size = slab.Bounds.size +
                    new Vector3(slabMargin * 2f, 0.004f, slabMargin * 2f);
                size.y = Mathf.Max(size.y, 0.025f);
                report.CompoundParts.Add(new CompoundBoxPartReport
                {
                    Kind = "HORIZONTAL_SLAB_BOX",
                    SourceComponentIds = slab.Report.ComponentIndex.ToString(),
                    Center = slab.Bounds.center,
                    Size = size,
                    EulerDegrees = Vector3.zero,
                    Reason = "大型闭合水平浅组件；单独保留层间开放空间"
                });
            }

            Bounds frameBounds = enclosingFrame.Bounds;
            float postThickness = Mathf.Clamp(
                Mathf.Min(frameBounds.size.x, frameBounds.size.z) * 0.055f,
                0.028f,
                0.060f);
            float minX = frameBounds.min.x + postThickness * 0.5f;
            float maxX = frameBounds.max.x - postThickness * 0.5f;
            float minZ = frameBounds.min.z + postThickness * 0.5f;
            float maxZ = frameBounds.max.z - postThickness * 0.5f;
            Vector3[] centers =
            {
                new Vector3(minX, frameBounds.center.y, minZ),
                new Vector3(minX, frameBounds.center.y, maxZ),
                new Vector3(maxX, frameBounds.center.y, minZ),
                new Vector3(maxX, frameBounds.center.y, maxZ)
            };
            for (int i = 0; i < centers.Length; i++)
            {
                report.CompoundParts.Add(new CompoundBoxPartReport
                {
                    Kind = "ENCLOSING_FRAME_CORNER_POST",
                    SourceComponentIds = enclosingFrame.Report.ComponentIndex.ToString(),
                    Center = centers[i],
                    Size = new Vector3(
                        postThickness,
                        frameBounds.size.y,
                        postThickness),
                    EulerDegrees = Vector3.zero,
                    Reason = "覆盖主要范围的高框架角柱近似；避免整体盒封闭内部空间"
                });
            }

            float compoundVolume = 0f;
            for (int i = 0; i < report.CompoundParts.Count; i++)
            {
                report.CompoundParts[i].PartIndex = i + 1;
                compoundVolume += SafeVolume(report.CompoundParts[i].Size);
            }

            int expectedParts = horizontalSlabs.Count + centers.Length;
            report.CompoundProxyDraftBuilt =
                report.CompoundParts.Count == expectedParts;
            report.CompoundProxyFitMethod =
                "LAYERED_HORIZONTAL_SLABS_PLUS_CORNER_POSTS_AUDIT_ONLY";
            report.CompoundProxyParts = report.CompoundParts.Count;
            report.CompoundProxyNeedsReview = true;
            report.CompoundProxyReviewReason =
                "通用复合盒仅为审计草案；层板与角柱保留开放空间，但其他横杆、脚轮、把手和细节尚未单独拟合";
            if (!report.CompoundProxyDraftBuilt)
                return;

            report.ProxyDraftBuilt = true;
            report.ProxyDraftType = "COMPOUND_BOX";
            report.ProxyFitMethod = report.CompoundProxyFitMethod;
            report.ProxyCenter = instance.Bounds.center;
            report.ProxyRawSize = instance.Bounds.size;
            report.ProxySize = instance.Bounds.size;
            report.ProxyEulerDegrees = Vector3.zero;
            report.ProxyAabbVolume = SafeVolume(instance.Bounds.size);
            report.ProxyRawVolume = compoundVolume;
            report.ProxyInflatedVolume = compoundVolume;
            report.ProxyVolumeRatioToAabb = report.ProxyAabbVolume > 1e-12f
                ? compoundVolume / report.ProxyAabbVolume
                : 0f;
            report.ProxyThicknessInflation = 0f;
            report.ProxyNeedsReview = true;
            report.ProxyReviewReason = report.CompoundProxyReviewReason;
            report.NeedsUnityExplorerReview = true;
            report.ReviewReason = AppendReason(
                report.ReviewReason,
                report.CompoundProxyReviewReason);
        }


        public static void ApplyInstanceProfiles(
            SpatialInstanceAuditReport audit,
            string targetPath,
            string targetName)
        {
            if (audit == null || !audit.Analyzed)
                return;

            audit.MatchedInstanceProfiles = 0;
            for (int i = 0; i < audit.Instances.Count; i++)
            {
                SpatialInstanceReport item = audit.Instances[i];
                if (item == null)
                    continue;

                SpatialInstancePhysicsProfileSnapshot profile =
                    SpatialInstancePhysicsWhitelist.Resolve(
                        targetPath,
                        targetName,
                        item.InstanceKey,
                        item.AssociatedCollisionInstanceKey,
                        item.Channel);
                item.PhysicsProfile = profile;
                if (profile == null || !profile.Matched)
                    continue;

                audit.MatchedInstanceProfiles++;
                if (profile.ProxyTypeConfigured)
                    item.ProxyDraftType = profile.ProxyType;

                if (item.ProxyDraftBuilt && profile.MinimumThicknessConfigured)
                {
                    float requested = Mathf.Max(0.001f, profile.MinimumThicknessMeters);
                    Vector3 adjusted = item.ProxySize;
                    adjusted.z = Mathf.Max(adjusted.z, requested);
                    item.ProxySize = adjusted;
                    item.ProxyThicknessInflation =
                        Mathf.Max(0f, adjusted.z - item.ProxyRawSize.z);
                    item.ProxyInflatedVolume = SafeVolume(adjusted);
                }
            }
        }

        private sealed class AxisProjection
        {
            public Vector3 Axis;
            public float Size;
        }

        private static void BuildOrientedBoxProxyDraft(
            InstanceData instance,
            List<Vector3> canonicalPositions,
            SpatialInstanceReport report)
        {
            report.ProxyDraftBuilt = false;
            report.ProxyFitMethod = "PCA_OBB_AUDIT_ONLY";
            report.ProxyDraftType = "<none>";
            report.ProxyNeedsReview = false;
            report.ProxyReviewReason = null;

            if (instance == null || canonicalPositions == null ||
                instance.CanonicalVertices.Count < 3)
            {
                report.ProxyNeedsReview = true;
                report.ProxyReviewReason = "可用空间顶点不足，无法生成定向盒草案";
                return;
            }

            List<Vector3> points = new List<Vector3>(instance.CanonicalVertices.Count);
            foreach (int index in instance.CanonicalVertices)
            {
                if (index >= 0 && index < canonicalPositions.Count)
                    points.Add(canonicalPositions[index]);
            }
            if (points.Count < 3)
            {
                report.ProxyNeedsReview = true;
                report.ProxyReviewReason = "有效空间顶点不足，无法生成定向盒草案";
                return;
            }

            Vector3 mean = Vector3.zero;
            for (int i = 0; i < points.Count; i++)
                mean += points[i];
            mean /= points.Count;

            float[,] covariance = new float[3, 3];
            for (int i = 0; i < points.Count; i++)
            {
                Vector3 d = points[i] - mean;
                covariance[0, 0] += d.x * d.x;
                covariance[0, 1] += d.x * d.y;
                covariance[0, 2] += d.x * d.z;
                covariance[1, 1] += d.y * d.y;
                covariance[1, 2] += d.y * d.z;
                covariance[2, 2] += d.z * d.z;
            }
            float inverse = 1f / Mathf.Max(1, points.Count);
            covariance[0, 0] *= inverse;
            covariance[0, 1] *= inverse;
            covariance[0, 2] *= inverse;
            covariance[1, 1] *= inverse;
            covariance[1, 2] *= inverse;
            covariance[2, 2] *= inverse;
            covariance[1, 0] = covariance[0, 1];
            covariance[2, 0] = covariance[0, 2];
            covariance[2, 1] = covariance[1, 2];

            JacobiEigenDecomposition(
                covariance,
                out float[] ignoredEigenvalues,
                out Vector3[] eigenvectors);
            if (eigenvectors == null || eigenvectors.Length != 3)
            {
                report.ProxyNeedsReview = true;
                report.ProxyReviewReason = "PCA 特征轴求解失败";
                return;
            }

            List<AxisProjection> axes = new List<AxisProjection>(3);
            for (int i = 0; i < 3; i++)
            {
                Vector3 axis = eigenvectors[i];
                if (axis.sqrMagnitude <= 1e-12f)
                    axis = i == 0 ? Vector3.right : (i == 1 ? Vector3.up : Vector3.forward);
                axis.Normalize();
                ProjectPoints(points, axis, out float min, out float max);
                axes.Add(new AxisProjection { Axis = axis, Size = max - min });
            }
            axes.Sort((a, b) => b.Size.CompareTo(a.Size));

            Vector3 axisX = axes[0].Axis.normalized;
            Vector3 axisY = axes[1].Axis - axisX * Vector3.Dot(axes[1].Axis, axisX);
            if (axisY.sqrMagnitude <= 1e-10f)
                axisY = FindPerpendicular(axisX);
            axisY.Normalize();
            Vector3 preferredZ = axes[2].Axis.normalized;
            Vector3 axisZ = Vector3.Cross(axisX, axisY);
            if (axisZ.sqrMagnitude <= 1e-10f)
                axisZ = FindPerpendicular(axisX);
            axisZ.Normalize();
            if (Vector3.Dot(axisZ, preferredZ) < 0f)
                axisZ = -axisZ;
            axisY = Vector3.Cross(axisZ, axisX).normalized;

            ProjectPoints(points, axisX, out float minX, out float maxX);
            ProjectPoints(points, axisY, out float minY, out float maxY);
            ProjectPoints(points, axisZ, out float minZ, out float maxZ);
            Vector3 rawSize = new Vector3(
                Mathf.Max(0f, maxX - minX),
                Mathf.Max(0f, maxY - minY),
                Mathf.Max(0f, maxZ - minZ));
            Vector3 center =
                axisX * ((minX + maxX) * 0.5f) +
                axisY * ((minY + maxY) * 0.5f) +
                axisZ * ((minZ + maxZ) * 0.5f);

            float maxDimension = Mathf.Max(rawSize.x, Mathf.Max(rawSize.y, rawSize.z));
            float defaultMinimumThickness = Mathf.Clamp(maxDimension * 0.025f, 0.010f, 0.030f);
            float planarMargin = Mathf.Clamp(maxDimension * 0.004f, 0.001f, 0.004f);
            Vector3 inflatedSize = new Vector3(
                rawSize.x + planarMargin * 2f,
                rawSize.y + planarMargin * 2f,
                Mathf.Max(rawSize.z + planarMargin * 2f, defaultMinimumThickness));

            Quaternion rotation = Quaternion.LookRotation(axisZ, axisY);
            float rawVolume = SafeVolume(rawSize);
            float inflatedVolume = SafeVolume(inflatedSize);
            float aabbVolume = SafeVolume(instance.Bounds.size);
            float thinRatio = maxDimension > 0f ? rawSize.z / maxDimension : 0f;

            report.ProxyDraftBuilt = IsFinite(center) && IsFinite(inflatedSize) &&
                IsFinite(rotation.eulerAngles) && maxDimension > 0f;
            report.ProxyDraftType = thinRatio <= 0.20f
                ? "ORIENTED_THIN_BOX"
                : "ORIENTED_BOX";
            report.ProxyCenter = center;
            report.ProxyRawSize = rawSize;
            report.ProxySize = inflatedSize;
            report.ProxyEulerDegrees = rotation.eulerAngles;
            report.ProxyAabbVolume = aabbVolume;
            report.ProxyRawVolume = rawVolume;
            report.ProxyInflatedVolume = inflatedVolume;
            report.ProxyVolumeRatioToAabb = aabbVolume > 1e-12f
                ? inflatedVolume / aabbVolume
                : 0f;
            report.ProxyThicknessInflation = Mathf.Max(0f, inflatedSize.z - rawSize.z);

            if (!report.ProxyDraftBuilt)
            {
                report.ProxyNeedsReview = true;
                report.ProxyReviewReason = "定向盒结果包含无效数值";
            }
            else if (report.ProxyDraftType == "ORIENTED_BOX" && report.OpenComponents > 1)
            {
                report.ProxyNeedsReview = true;
                report.ProxyReviewReason = "多个开放组件被一个普通定向盒覆盖；未来可能需要组合代理";
            }
        }

        private static void JacobiEigenDecomposition(
            float[,] input,
            out float[] eigenvalues,
            out Vector3[] eigenvectors)
        {
            float[,] a = new float[3, 3];
            float[,] v = new float[3, 3];
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    a[r, c] = input[r, c];
                    v[r, c] = r == c ? 1f : 0f;
                }
            }

            for (int sweep = 0; sweep < 32; sweep++)
            {
                int p = 0;
                int q = 1;
                float largest = Mathf.Abs(a[0, 1]);
                if (Mathf.Abs(a[0, 2]) > largest)
                {
                    p = 0;
                    q = 2;
                    largest = Mathf.Abs(a[0, 2]);
                }
                if (Mathf.Abs(a[1, 2]) > largest)
                {
                    p = 1;
                    q = 2;
                    largest = Mathf.Abs(a[1, 2]);
                }
                if (largest <= 1e-12f)
                    break;

                float app = a[p, p];
                float aqq = a[q, q];
                float apq = a[p, q];
                float phi = 0.5f * Mathf.Atan2(2f * apq, aqq - app);
                float c = Mathf.Cos(phi);
                float sn = Mathf.Sin(phi);

                for (int k = 0; k < 3; k++)
                {
                    if (k == p || k == q)
                        continue;
                    float akp = a[k, p];
                    float akq = a[k, q];
                    float newKp = c * akp - sn * akq;
                    float newKq = sn * akp + c * akq;
                    a[k, p] = newKp;
                    a[p, k] = newKp;
                    a[k, q] = newKq;
                    a[q, k] = newKq;
                }
                a[p, p] = c * c * app - 2f * sn * c * apq + sn * sn * aqq;
                a[q, q] = sn * sn * app + 2f * sn * c * apq + c * c * aqq;
                a[p, q] = 0f;
                a[q, p] = 0f;

                for (int k = 0; k < 3; k++)
                {
                    float vip = v[k, p];
                    float viq = v[k, q];
                    v[k, p] = c * vip - sn * viq;
                    v[k, q] = sn * vip + c * viq;
                }
            }

            eigenvalues = new[] { a[0, 0], a[1, 1], a[2, 2] };
            eigenvectors = new[]
            {
                new Vector3(v[0, 0], v[1, 0], v[2, 0]).normalized,
                new Vector3(v[0, 1], v[1, 1], v[2, 1]).normalized,
                new Vector3(v[0, 2], v[1, 2], v[2, 2]).normalized
            };
        }

        private static void ProjectPoints(
            List<Vector3> points,
            Vector3 axis,
            out float min,
            out float max)
        {
            min = float.PositiveInfinity;
            max = float.NegativeInfinity;
            for (int i = 0; i < points.Count; i++)
            {
                float value = Vector3.Dot(points[i], axis);
                if (value < min)
                    min = value;
                if (value > max)
                    max = value;
            }
            if (float.IsInfinity(min) || float.IsInfinity(max))
            {
                min = 0f;
                max = 0f;
            }
        }

        private static Vector3 FindPerpendicular(Vector3 axis)
        {
            Vector3 candidate = Mathf.Abs(axis.y) < 0.90f ? Vector3.up : Vector3.right;
            Vector3 perpendicular = Vector3.Cross(axis, candidate);
            if (perpendicular.sqrMagnitude <= 1e-10f)
                perpendicular = Vector3.Cross(axis, Vector3.forward);
            return perpendicular.normalized;
        }

        private static float SafeVolume(Vector3 size)
        {
            return Mathf.Max(0f, size.x) * Mathf.Max(0f, size.y) * Mathf.Max(0f, size.z);
        }

        private static bool IsFinite(Vector3 value)
        {
            return !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                !float.IsNaN(value.z) && !float.IsInfinity(value.z);
        }

        private static int CompareComponents(ComponentData a, ComponentData b)
        {
            int x = a.Bounds.center.x.CompareTo(b.Bounds.center.x);
            if (x != 0)
                return x;
            int z = a.Bounds.center.z.CompareTo(b.Bounds.center.z);
            if (z != 0)
                return z;
            int y = a.Bounds.center.y.CompareTo(b.Bounds.center.y);
            if (y != 0)
                return y;
            return a.Triangles.Count.CompareTo(b.Triangles.Count);
        }

        private static int CompareInstances(InstanceData a, InstanceData b)
        {
            int x = a.Bounds.center.x.CompareTo(b.Bounds.center.x);
            if (x != 0)
                return x;
            int z = a.Bounds.center.z.CompareTo(b.Bounds.center.z);
            if (z != 0)
                return z;
            int y = a.Bounds.center.y.CompareTo(b.Bounds.center.y);
            if (y != 0)
                return y;
            return a.Components.Count.CompareTo(b.Components.Count);
        }

        private static int[] BuildWeldMap(
            List<Vector3> positions,
            float tolerance,
            out int canonicalCount,
            out List<Vector3> canonical)
        {
            tolerance = Mathf.Max(tolerance, 0.000001f);
            float inverse = 1f / tolerance;
            int[] map = new int[positions.Count];
            canonical = new List<Vector3>();
            Dictionary<QuantizedKey, List<int>> cells =
                new Dictionary<QuantizedKey, List<int>>();
            float toleranceSquared = tolerance * tolerance;

            for (int i = 0; i < positions.Count; i++)
            {
                Vector3 point = positions[i];
                long qx = (long)Math.Floor(point.x * inverse);
                long qy = (long)Math.Floor(point.y * inverse);
                long qz = (long)Math.Floor(point.z * inverse);
                int found = -1;
                for (int dx = -1; dx <= 1 && found < 0; dx++)
                {
                    for (int dy = -1; dy <= 1 && found < 0; dy++)
                    {
                        for (int dz = -1; dz <= 1 && found < 0; dz++)
                        {
                            QuantizedKey key = new QuantizedKey(qx + dx, qy + dy, qz + dz);
                            if (!cells.TryGetValue(key, out List<int> candidates))
                                continue;
                            for (int c = 0; c < candidates.Count; c++)
                            {
                                int candidate = candidates[c];
                                if ((canonical[candidate] - point).sqrMagnitude <= toleranceSquared)
                                {
                                    found = candidate;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (found < 0)
                {
                    found = canonical.Count;
                    canonical.Add(point);
                    QuantizedKey own = new QuantizedKey(qx, qy, qz);
                    if (!cells.TryGetValue(own, out List<int> list))
                    {
                        list = new List<int>();
                        cells.Add(own, list);
                    }
                    list.Add(found);
                }
                map[i] = found;
            }

            canonicalCount = canonical.Count;
            return map;
        }

        private static void AddGlobalEdge(
            Dictionary<ulong, EdgeInfo> edges,
            UnionFind union,
            int from,
            int to,
            int triangle)
        {
            int min = from < to ? from : to;
            int max = from < to ? to : from;
            ulong key = ((ulong)(uint)min << 32) | (uint)max;
            if (!edges.TryGetValue(key, out EdgeInfo info))
            {
                info = new EdgeInfo { FirstTriangle = triangle };
                edges.Add(key, info);
            }
            else if (info.FirstTriangle >= 0)
            {
                union.Union(info.FirstTriangle, triangle);
            }
            info.Count++;
            if (from == min)
                info.ForwardCount++;
        }

        private static void AddLocalEdge(
            Dictionary<ulong, EdgeInfo> edges,
            int from,
            int to)
        {
            int min = from < to ? from : to;
            int max = from < to ? to : from;
            ulong key = ((ulong)(uint)min << 32) | (uint)max;
            if (!edges.TryGetValue(key, out EdgeInfo info))
            {
                info = new EdgeInfo();
                edges.Add(key, info);
            }
            info.Count++;
            if (from == min)
                info.ForwardCount++;
        }

        private static void DecodeEdge(ulong key, out int a, out int b)
        {
            a = unchecked((int)(key >> 32));
            b = unchecked((int)(key & 0xffffffffu));
        }

        private static void AddAdjacency(
            Dictionary<int, List<int>> adjacency,
            int from,
            int to)
        {
            if (!adjacency.TryGetValue(from, out List<int> list))
            {
                list = new List<int>();
                adjacency.Add(from, list);
            }
            list.Add(to);
        }

        private static void CountBoundaryComponents(
            Dictionary<int, List<int>> adjacency,
            out int loops,
            out int openChains)
        {
            loops = 0;
            openChains = 0;
            HashSet<int> visited = new HashSet<int>();
            foreach (KeyValuePair<int, List<int>> pair in adjacency)
            {
                int start = pair.Key;
                if (visited.Contains(start))
                    continue;
                bool allDegreeTwo = true;
                Stack<int> stack = new Stack<int>();
                stack.Push(start);
                visited.Add(start);
                while (stack.Count > 0)
                {
                    int current = stack.Pop();
                    if (!adjacency.TryGetValue(current, out List<int> neighbors))
                    {
                        allDegreeTwo = false;
                        continue;
                    }
                    if (neighbors.Count != 2)
                        allDegreeTwo = false;
                    for (int i = 0; i < neighbors.Count; i++)
                    {
                        if (visited.Add(neighbors[i]))
                            stack.Push(neighbors[i]);
                    }
                }
                if (allDegreeTwo)
                    loops++;
                else
                    openChains++;
            }
        }

        private static float BoundsGapDistance(Bounds a, Bounds b)
        {
            Vector3 amin = a.min;
            Vector3 amax = a.max;
            Vector3 bmin = b.min;
            Vector3 bmax = b.max;
            float dx = Mathf.Max(0f, Mathf.Max(bmin.x - amax.x, amin.x - bmax.x));
            float dy = Mathf.Max(0f, Mathf.Max(bmin.y - amax.y, amin.y - bmax.y));
            float dz = Mathf.Max(0f, Mathf.Max(bmin.z - amax.z, amin.z - bmax.z));
            return Mathf.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        private static bool IsPhysicalRole(string role)
        {
            return string.Equals(role, "PHYSICAL_CORE", StringComparison.Ordinal) ||
                string.Equals(role, "PHYSICAL_DETAIL", StringComparison.Ordinal);
        }

        private static string BuildStableKey(
            string prefix,
            string targetPath,
            string channel,
            Bounds bounds,
            HashSet<int> routeIds,
            int discriminator)
        {
            StringBuilder source = new StringBuilder();
            source.Append(targetPath ?? string.Empty).Append('|')
                .Append(channel ?? string.Empty).Append('|')
                .Append(Quantize(bounds.center.x)).Append(',')
                .Append(Quantize(bounds.center.y)).Append(',')
                .Append(Quantize(bounds.center.z)).Append('|')
                .Append(Quantize(bounds.size.x)).Append(',')
                .Append(Quantize(bounds.size.y)).Append(',')
                .Append(Quantize(bounds.size.z)).Append('|')
                .Append(discriminator).Append('|')
                .Append(JoinInts(routeIds));
            ulong hash = Fnv1a64(source.ToString());
            return prefix + "-" + (channel ?? "unknown").ToUpperInvariant() + "-" +
                hash.ToString("X16");
        }

        private static long Quantize(float value)
        {
            return (long)Math.Round(value * 10000.0);
        }

        private static ulong Fnv1a64(string text)
        {
            const ulong offset = 14695981039346656037UL;
            const ulong prime = 1099511628211UL;
            ulong hash = offset;
            if (text == null)
                return hash;
            for (int i = 0; i < text.Length; i++)
            {
                char ch = text[i];
                hash ^= (byte)(ch & 0xff);
                hash *= prime;
                hash ^= (byte)(ch >> 8);
                hash *= prime;
            }
            return hash;
        }

        private static string JoinInts(IEnumerable<int> values)
        {
            if (values == null)
                return "<none>";
            List<int> sorted = new List<int>();
            foreach (int value in values)
                sorted.Add(value);
            sorted.Sort();
            if (sorted.Count == 0)
                return "<none>";
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < sorted.Count; i++)
            {
                if (i > 0)
                    sb.Append(',');
                sb.Append(sorted[i]);
            }
            return sb.ToString();
        }

        private static string JoinStrings(IEnumerable<string> values)
        {
            if (values == null)
                return "<none>";
            List<string> sorted = new List<string>();
            foreach (string value in values)
            {
                if (!string.IsNullOrEmpty(value))
                    sorted.Add(value);
            }
            sorted.Sort(StringComparer.Ordinal);
            if (sorted.Count == 0)
                return "<none>";
            return string.Join(",", sorted.ToArray());
        }

        private static string AppendReason(string current, string addition)
        {
            if (string.IsNullOrEmpty(addition))
                return current;
            if (string.IsNullOrEmpty(current))
                return addition;
            if (current.IndexOf(addition, StringComparison.Ordinal) >= 0)
                return current;
            return current + "; " + addition;
        }

        private static string FormatBounds(Bounds bounds)
        {
            return "center=" + bounds.center.ToString("G9") +
                "/size=" + bounds.size.ToString("G9") +
                "/min=" + bounds.min.ToString("G9") +
                "/max=" + bounds.max.ToString("G9");
        }
    }
}
