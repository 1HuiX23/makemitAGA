using System;
using System.Collections.Generic;
using System.Diagnostics;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// 第一章四房间的只读几何普查。
    ///
    /// 这不是正式迭代功能，也不创建 Mesh、Collider 或 Rigidbody。
    /// 它按房间下一层对象根节点进行分类；只有“单 MeshRenderer + 单路由子网格”
    /// 才调用已经验证过的 D3D11 read-only staging 路线做完整拓扑分析。
    /// 多 Renderer / 多路由对象会明确标记为需要聚合，避免把第一段子网格误判为整个物体。
    /// </summary>
    internal static class HouseGeometryCensus
    {
        private const string ExpectedHousePath =
            "World/House/HouseGame Tamagotchi/House";
        private static readonly string[] DefaultRooms =
            { "Bedroom", "Kitchen", "Main", "Toilet" };

        private sealed class PendingObject
        {
            public string Room;
            public GameObject Root;
        }

        private sealed class EdgeInfo
        {
            public int Count;
            public int ForwardCount;
            public int FirstTriangle = -1;
        }

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

        private static ManualLogSource _logger;
        private static readonly List<PendingObject> Pending =
            new List<PendingObject>();
        private static HouseGeometryCensusReport _report;
        private static Stopwatch _stopwatch;
        private static int _nextIndex;
        private static float _nextAllowedRealtime;

        public static bool IsRunning => _report != null && _report.Running;

        public static string Start(
            ManualLogSource logger,
            string roomFilter,
            out HouseGeometryCensusReport report)
        {
            Cancel("restart");
            _logger = logger;
            _report = new HouseGeometryCensusReport
            {
                Started = true,
                Running = false,
                SceneName = SceneManager.GetActiveScene().name,
                RoomFilter = string.IsNullOrWhiteSpace(roomFilter)
                    ? "all-four-rooms"
                    : roomFilter.Trim(),
                HousePath = ExpectedHousePath
            };
            report = _report;

            GameObject house = FindByExactPath(ExpectedHousePath);
            if (house == null)
            {
                _report.Completed = true;
                _report.Detail = "house-root-not-found:" + ExpectedHousePath;
                _logger?.LogWarning("[GSP][HOUSE_CENSUS] start failed | " + _report.ToSummaryText());
                return "<color=red>未找到四房间父节点：</color> " + ExpectedHousePath;
            }

            string[] rooms = ResolveRooms(roomFilter);
            for (int i = 0; i < rooms.Length; i++)
            {
                Transform room = FindDirectChild(house.transform, rooms[i]);
                if (room == null)
                {
                    _logger?.LogWarning(
                        "[GSP][HOUSE_CENSUS][ROOM_MISSING] room=" + rooms[i] +
                        " | parent=" + ExpectedHousePath);
                    continue;
                }

                _report.RoomsFound++;
                int childCount = Safe(() => room.childCount, 0);
                _logger?.LogInfo(
                    "[GSP][HOUSE_CENSUS][ROOM] room=" + rooms[i] +
                    " | path=" + TargetFinder.GetPath(room) +
                    " | active=" + room.gameObject.activeInHierarchy +
                    " | directObjectRoots=" + childCount);

                for (int childIndex = 0; childIndex < childCount; childIndex++)
                {
                    Transform child = null;
                    try { child = room.GetChild(childIndex); }
                    catch { }
                    if (child == null || child.gameObject == null)
                        continue;
                    if ((child.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal))
                        continue;
                    Pending.Add(new PendingObject
                    {
                        Room = rooms[i],
                        Root = child.gameObject
                    });
                }
            }

            _report.ObjectsQueued = Pending.Count;
            _report.Running = Pending.Count > 0;
            _report.Completed = Pending.Count == 0;
            _report.Detail = Pending.Count > 0
                ? "queued; one direct room-object root is processed per update; no collider cooking"
                : "no direct room-object roots found";
            _nextIndex = 0;
            _nextAllowedRealtime = Time.realtimeSinceStartup + 0.1f;
            _stopwatch = Stopwatch.StartNew();

            _logger?.LogWarning(
                "[GSP][HOUSE_CENSUS] begin" +
                " | build=0.2.3-active-room-incremental-floor-atlas-foundation" +
                " | scene=" + _report.SceneName +
                " | house=" + ExpectedHousePath +
                " | roomFilter=" + _report.RoomFilter +
                " | roomsFound=" + _report.RoomsFound +
                " | queued=" + _report.ObjectsQueued +
                " | processing=one-object-per-update" +
                " | readback=quiet-read-only-D3D11-staging" +
                " | colliderCreation=False" +
                " | IMPORTANT=multi-renderer-or-multi-submesh objects are not called watertight; they are classified for aggregation");

            return Pending.Count > 0
                ? "<color=#66ddff>四房间中间几何普查已启动。</color> " +
                  "rooms=" + _report.RoomsFound +
                  "，objects=" + _report.ObjectsQueued +
                  "。它会逐帧扫描并把每个对象的一行分类写入 LogOutput；执行 gsp_house_census_status 查看进度。"
                : "<color=yellow>四房间已找到，但没有排入任何直接对象根节点。</color>";
        }

        public static void Update()
        {
            if (_report == null || !_report.Running)
                return;
            if (Time.realtimeSinceStartup < _nextAllowedRealtime)
                return;

            if (_nextIndex >= Pending.Count)
            {
                Complete(false, "all-queued-objects-processed");
                return;
            }

            PendingObject pending = Pending[_nextIndex++];
            HouseGeometryCensusObjectReport item = AnalyzeObject(
                pending,
                _nextIndex,
                Pending.Count);
            _report.Objects.Add(item);
            _report.ObjectsProcessed++;
            _report.LastObject = item.Room + "/" + item.ObjectName;
            Accumulate(item);
            _report.ElapsedSeconds = ElapsedSeconds();

            _logger?.LogInfo(
                "[GSP][HOUSE_CENSUS][OBJECT] " + item.ToLogLine());
            if (item.LikelyPickable)
            {
                _logger?.LogWarning(
                    "[GSP][HOUSE_CENSUS][PICKABLE] room=" + item.Room +
                    " | object=" + item.ObjectName +
                    " | class=" + item.Classification +
                    " | watertight=" + item.Watertight +
                    " | coverage=" + item.Coverage +
                    " | boundary=" + item.BoundaryEdges +
                    " | nonManifold=" + item.NonManifoldEdges +
                    " | components=" + item.ConnectedComponents +
                    " | action=" + item.RecommendedAction);
            }

            _nextAllowedRealtime = Time.realtimeSinceStartup + 0.03f;
            if (_nextIndex >= Pending.Count)
                Complete(false, "all-queued-objects-processed");
        }

        public static string Cancel(string reason)
        {
            bool existed = _report != null || Pending.Count > 0;
            if (_report != null && _report.Running)
                Complete(true, reason ?? "cancelled");
            Pending.Clear();
            _nextIndex = 0;
            _nextAllowedRealtime = 0f;
            if (!string.IsNullOrEmpty(reason) &&
                reason.StartsWith("runtime-clear:", StringComparison.Ordinal))
            {
                _report = null;
                _stopwatch = null;
            }
            return existed
                ? "GSP 四房间几何普查已停止。"
                : "当前没有运行中的四房间几何普查。";
        }

        public static string GetStatusText()
        {
            if (_report == null)
                return "当前没有四房间几何普查。执行 gsp_house_census_start。";
            _report.ElapsedSeconds = ElapsedSeconds();
            return _report.ToStatusText();
        }

        public static string GetReportText()
        {
            if (_report == null)
                return "当前没有四房间几何普查报告。执行 gsp_house_census_start。";
            _report.ElapsedSeconds = ElapsedSeconds();
            return _report.ToSummaryText();
        }

        public static HouseGeometryCensusReport CurrentReport => _report;

        private static HouseGeometryCensusObjectReport AnalyzeObject(
            PendingObject pending,
            int sequence,
            int total)
        {
            GameObject root = pending.Root;
            HouseGeometryCensusObjectReport result =
                new HouseGeometryCensusObjectReport
                {
                    Sequence = sequence,
                    Room = pending.Room,
                    ObjectName = root != null ? root.name : "<null>",
                    ObjectPath = TargetFinder.GetPath(root != null ? root.transform : null),
                    ActiveSelf = root != null && root.activeSelf,
                    ActiveInHierarchy = root != null && root.activeInHierarchy,
                    DirectChildren = root != null ? Safe(() => root.transform.childCount, 0) : 0,
                    Coverage = "none",
                    ReadbackDetail = "not-attempted",
                    Classification = "UNCLASSIFIED",
                    RecommendedAction = "review"
                };

            if (root == null)
            {
                result.Classification = "METADATA_ONLY";
                result.RecommendedAction = "null-root-skip";
                return result;
            }

            MeshRenderer[] meshRenderers =
                SafeGetComponentsInChildren<MeshRenderer>(root);
            SkinnedMeshRenderer[] skinned =
                SafeGetComponentsInChildren<SkinnedMeshRenderer>(root);
            Collider[] colliders =
                SafeGetComponentsInChildren<Collider>(root);
            int rigidbodyCount = CountRigidbodiesRecursive(root.transform, 0);

            result.MeshRenderers = meshRenderers.Length;
            result.SkinnedRenderers = skinned.Length;
            result.Colliders = colliders.Length;
            result.Rigidbodies = rigidbodyCount;
            for (int i = 0; i < colliders.Length; i++)
            {
                Collider collider = colliders[i];
                if (collider == null)
                    continue;
                if (Safe(() => collider.enabled, false))
                    result.EnabledColliders++;
                if (Safe(() => collider.isTrigger, false))
                    result.TriggerColliders++;
            }

            if (TryGetBounds(root, out Bounds bounds))
            {
                result.Bounds = FormatBounds(bounds);
                float x = Mathf.Abs(bounds.size.x);
                float y = Mathf.Abs(bounds.size.y);
                float z = Mathf.Abs(bounds.size.z);
                result.MaximumDimension = Mathf.Max(x, Mathf.Max(y, z));
                result.MinimumDimension = Mathf.Min(x, Mathf.Min(y, z));
                result.ThinAxisRatio = result.MaximumDimension > 1e-6f
                    ? result.MinimumDimension / result.MaximumDimension
                    : 0f;
            }
            else
            {
                result.Bounds = "<none>";
            }

            CountRoutes(
                meshRenderers,
                out int candidateRenderers,
                out int totalRoutedSubMeshes,
                out bool layoutSupported,
                out string layoutReason);
            result.CandidateMeshRenderers = candidateRenderers;
            result.RoutedSubMeshes = totalRoutedSubMeshes;
            result.CurrentProbeFullCoverage =
                candidateRenderers == 1 && totalRoutedSubMeshes == 1;
            result.Coverage = result.CurrentProbeFullCoverage
                ? "full-single-renderer-single-route"
                : (candidateRenderers > 0
                    ? "not-full-current-probe-needs-aggregation"
                    : "no-static-mesh-route");

            EvaluatePickable(root.name, result);

            if (candidateRenderers == 0)
            {
                result.Classification = skinned.Length > 0
                    ? "SKINNED_SPECIAL_ADAPTER_REQUIRED"
                    : "METADATA_ONLY";
                result.RecommendedAction = skinned.Length > 0
                    ? "do-not-use-static-buffer-route; add skinned/bake policy"
                    : "no MeshRenderer route; keep as hierarchy/collider/semantic metadata";
                return result;
            }

            if (!result.CurrentProbeFullCoverage)
            {
                result.Classification = result.LikelyPickable
                    ? "PICKABLE_AGGREGATION_REQUIRED"
                    : "MULTIPART_AGGREGATION_REQUIRED";
                result.RecommendedAction =
                    "enumerate every renderer and routed submesh, merge by object root, then run whole-object topology";
                result.ReadbackDetail =
                    "skipped-to-avoid-first-route-being-misreported-as-whole-object";
                return result;
            }

            if (!layoutSupported)
            {
                result.Classification = "SPECIAL_LAYOUT_ADAPTER_REQUIRED";
                result.RecommendedAction = layoutReason;
                result.ReadbackDetail = "preflight-skip:" + layoutReason;
                return result;
            }

            result.ReadbackAttempted = true;
            D3D11FullSubmeshReadbackReport readback = null;
            D3D11SubmeshGeometry geometry = null;
            try
            {
                // 传 null logger：保留原生安全边界与验证，但避免每个对象输出几十行 COM 细节。
                readback = D3D11FullSubmeshReadback.Probe(root, null, out geometry);
            }
            catch (Exception e)
            {
                result.ReadbackSuccess = false;
                result.ReadbackDetail = e.GetType().Name + ":" + e.Message;
            }

            if (readback != null)
            {
                result.ReadbackSuccess = readback.Success;
                result.ReadbackDetail = readback.Detail;
                result.MeshName = readback.MeshName;
                result.RendererPath = readback.RendererPath;
                result.Vertices = readback.DecodedPositionCount;
                result.Indices = readback.DecodedIndexCount;
                result.Triangles = readback.TriangleCount;
                result.InvalidTriangles = readback.OutOfRangeIndexCount;
                result.RepeatedIndexTriangles = readback.RepeatedIndexTriangleCount;
                result.ZeroAreaTriangles = readback.ZeroAreaTriangleCount;
            }

            if (!result.ReadbackSuccess || geometry == null)
            {
                result.Classification = "READBACK_FAILED_OR_UNSUPPORTED";
                result.RecommendedAction =
                    "inspect layout/route and add a dedicated safe adapter before topology or collider use";
                return result;
            }

            AnalyzeTopology(geometry, result);
            if (result.NonManifoldEdges > 0 ||
                result.OrientationConflicts > 0 ||
                result.DuplicateTriangles > 0)
            {
                result.Classification = result.LikelyPickable
                    ? "PICKABLE_REPAIR_REQUIRED"
                    : "TOPOLOGY_REPAIR_REQUIRED";
                result.RecommendedAction =
                    "repair non-manifold/orientation/duplicate topology before volume, convex decomposition or dynamic use";
            }
            else if (!result.Watertight)
            {
                result.Classification = result.LikelyPickable
                    ? "PICKABLE_REPAIR_REQUIRED"
                    : "OPEN_STATIC_SURFACE";
                result.RecommendedAction = result.LikelyPickable
                    ? "close boundary loops or build a validated convex/compound proxy before pick-up"
                    : "usable only as candidate static surface after collider cooking/contact audit; do not treat as closed volume";
            }
            else if (result.LikelyPickable)
            {
                result.Classification = "PICKABLE_GEOMETRY_READY";
                result.RecommendedAction =
                    "proceed to convex decomposition/compound collider, mass-center and grasp-point tests; not yet dynamic-ready by this census alone";
            }
            else
            {
                result.Classification = "WATERTIGHT_STATIC_GEOMETRY";
                result.RecommendedAction =
                    "eligible for later static MeshCollider sampling audit; dynamic use still requires convex/compound proxy";
            }

            return result;
        }

        private static void AnalyzeTopology(
            D3D11SubmeshGeometry geometry,
            HouseGeometryCensusObjectReport result)
        {
            int[] indices = geometry.LocalIndices;
            Vector3[] positions = geometry.WorldPositions;
            int triangleCount = indices != null ? indices.Length / 3 : 0;
            if (indices == null || positions == null || triangleCount <= 0)
                return;

            Dictionary<ulong, EdgeInfo> edges =
                new Dictionary<ulong, EdgeInfo>(Math.Max(16, triangleCount * 2));
            Dictionary<string, int> triangleKeys =
                new Dictionary<string, int>(Math.Max(16, triangleCount));
            UnionFind components = new UnionFind(triangleCount);

            for (int triangle = 0; triangle < triangleCount; triangle++)
            {
                int a = indices[triangle * 3];
                int b = indices[triangle * 3 + 1];
                int c = indices[triangle * 3 + 2];
                if (a < 0 || b < 0 || c < 0 ||
                    a >= positions.Length || b >= positions.Length || c >= positions.Length)
                    continue;

                int s0 = a;
                int s1 = b;
                int s2 = c;
                Sort3(ref s0, ref s1, ref s2);
                string triKey = s0 + ":" + s1 + ":" + s2;
                if (triangleKeys.ContainsKey(triKey))
                    result.DuplicateTriangles++;
                else
                    triangleKeys.Add(triKey, triangle);

                AddEdge(edges, components, a, b, triangle);
                AddEdge(edges, components, b, c, triangle);
                AddEdge(edges, components, c, a, triangle);
            }

            foreach (KeyValuePair<ulong, EdgeInfo> pair in edges)
            {
                EdgeInfo edge = pair.Value;
                if (edge.Count == 1)
                    result.BoundaryEdges++;
                else if (edge.Count > 2)
                    result.NonManifoldEdges++;
                if (edge.Count == 2 &&
                    (edge.ForwardCount == 0 || edge.ForwardCount == 2))
                    result.OrientationConflicts++;
            }

            HashSet<int> roots = new HashSet<int>();
            for (int triangle = 0; triangle < triangleCount; triangle++)
                roots.Add(components.Find(triangle));
            result.ConnectedComponents = roots.Count;
            result.TopologyAnalyzed = true;
            result.Watertight =
                result.BoundaryEdges == 0 &&
                result.NonManifoldEdges == 0 &&
                result.OrientationConflicts == 0 &&
                result.ConnectedComponents == 1 &&
                result.DuplicateTriangles == 0 &&
                result.InvalidTriangles == 0 &&
                result.RepeatedIndexTriangles == 0 &&
                result.ZeroAreaTriangles == 0;
        }

        private static void AddEdge(
            Dictionary<ulong, EdgeInfo> edges,
            UnionFind components,
            int from,
            int to,
            int triangle)
        {
            uint min = (uint)Math.Min(from, to);
            uint max = (uint)Math.Max(from, to);
            ulong key = ((ulong)min << 32) | max;
            bool forward = (uint)from == min;
            if (!edges.TryGetValue(key, out EdgeInfo edge))
            {
                edge = new EdgeInfo
                {
                    Count = 1,
                    ForwardCount = forward ? 1 : 0,
                    FirstTriangle = triangle
                };
                edges.Add(key, edge);
            }
            else
            {
                edge.Count++;
                if (forward)
                    edge.ForwardCount++;
                if (edge.FirstTriangle >= 0)
                    components.Union(edge.FirstTriangle, triangle);
            }
        }

        private static void CountRoutes(
            MeshRenderer[] renderers,
            out int candidateRenderers,
            out int routedSubMeshes,
            out bool layoutSupported,
            out string layoutReason)
        {
            candidateRenderers = 0;
            routedSubMeshes = 0;
            layoutSupported = true;
            layoutReason = "supported";

            for (int i = 0; i < renderers.Length; i++)
            {
                MeshRenderer renderer = renderers[i];
                if (renderer == null)
                    continue;
                MeshFilter filter = Safe(() => renderer.GetComponent<MeshFilter>(), (MeshFilter)null);
                Mesh mesh = filter != null ? Safe(() => filter.sharedMesh, (Mesh)null) : null;
                if (mesh == null)
                    continue;

                int subMeshCount = Safe(() => mesh.subMeshCount, 0);
                int start = Safe(() => renderer.subMeshStartIndex, 0);
                int materialCount = 0;
                try
                {
                    var materials = renderer.sharedMaterials;
                    materialCount = materials != null ? materials.Length : 0;
                }
                catch { }
                int count = materialCount > 0 ? materialCount : 1;
                if (start < 0 || start >= subMeshCount)
                    continue;
                if (start + count > subMeshCount)
                    count = subMeshCount - start;
                if (count <= 0)
                    continue;

                candidateRenderers++;
                routedSubMeshes += count;

                if (candidateRenderers == 1 && count == 1)
                {
                    try
                    {
                        SubMeshDescriptor descriptor = mesh.GetSubMesh(start);
                        if (descriptor.topology != MeshTopology.Triangles)
                        {
                            layoutSupported = false;
                            layoutReason = "topology=" + descriptor.topology;
                        }
                        VertexAttributeFormat format =
                            mesh.GetVertexAttributeFormat(VertexAttribute.Position);
                        int dimension =
                            mesh.GetVertexAttributeDimension(VertexAttribute.Position);
                        if (format != VertexAttributeFormat.Float32 || dimension != 3)
                        {
                            layoutSupported = false;
                            layoutReason =
                                "Position=" + format + "x" + dimension +
                                "; current readback validator supports Float32x3";
                        }
                        if (descriptor.vertexCount > 120000 || descriptor.indexCount > 360000)
                        {
                            layoutSupported = false;
                            layoutReason =
                                "route-too-large-for-intermediate-census:v=" + descriptor.vertexCount +
                                ",i=" + descriptor.indexCount;
                        }
                    }
                    catch (Exception e)
                    {
                        layoutSupported = false;
                        layoutReason = "layout-preflight:" + e.GetType().Name + ":" + e.Message;
                    }
                }
            }
        }

        private static void EvaluatePickable(
            string name,
            HouseGeometryCensusObjectReport result)
        {
            string lower = (name ?? string.Empty).ToLowerInvariant();
            string[] positive =
            {
                "cup", "mug", "glass", "bottle", "can", "bowl", "plate",
                "spoon", "fork", "book", "phone", "remote", "ball", "toy",
                "clock", "pillow", "trash", "bucket", "food", "fruit", "apple",
                "knife", "key", "pen", "brush", "soap", "towel", "paper"
            };
            string[] staticWords =
            {
                "floor", "wall", "ceiling", "room", "curtain", "carpet",
                "wardrobe", "table", "bed", "chair", "mirror", "socket",
                "door", "lighting", "house"
            };

            int score = 0;
            string reason = string.Empty;
            for (int i = 0; i < positive.Length; i++)
            {
                if (lower.Contains(positive[i]))
                {
                    score += 3;
                    reason += "name:" + positive[i] + ",";
                    break;
                }
            }
            for (int i = 0; i < staticWords.Length; i++)
            {
                if (lower.Contains(staticWords[i]))
                {
                    score -= 2;
                    reason += "static-name:" + staticWords[i] + ",";
                    break;
                }
            }
            if (result.Rigidbodies > 0)
            {
                score += 3;
                reason += "rigidbody,";
            }
            if (result.MaximumDimension > 0f && result.MaximumDimension <= 0.8f)
            {
                score += 1;
                reason += "small-bounds,";
            }
            if (result.MaximumDimension > 1.5f)
            {
                score -= 2;
                reason += "large-bounds,";
            }
            result.LikelyPickable = score >= 2;
            result.PickableReason = string.IsNullOrEmpty(reason)
                ? "no-positive-heuristic"
                : reason.TrimEnd(',');
        }

        private static void Accumulate(HouseGeometryCensusObjectReport item)
        {
            if (_report == null || item == null)
                return;
            if (item.ReadbackAttempted)
                _report.ReadbackAttempted++;
            if (item.ReadbackSuccess)
                _report.ReadbackSucceeded++;
            if (item.CurrentProbeFullCoverage)
                _report.FullCoverageObjects++;
            if (item.Watertight)
                _report.WatertightObjects++;
            if (item.TopologyAnalyzed && item.BoundaryEdges > 0)
                _report.OpenObjects++;
            if (item.NonManifoldEdges > 0 || item.OrientationConflicts > 0)
                _report.NonManifoldObjects++;
            if (item.Classification == "MULTIPART_AGGREGATION_REQUIRED" ||
                item.Classification == "PICKABLE_AGGREGATION_REQUIRED")
                _report.MultiPartObjects++;
            if (item.Classification == "SKINNED_SPECIAL_ADAPTER_REQUIRED")
                _report.SkinnedOnlyObjects++;
            if (item.Classification == "METADATA_ONLY")
                _report.MetadataOnlyObjects++;
            if (item.LikelyPickable)
                _report.PickableCandidates++;
            if (item.Classification == "PICKABLE_GEOMETRY_READY")
                _report.PickableReady++;
            if (item.Classification == "PICKABLE_REPAIR_REQUIRED")
                _report.PickableRepairRequired++;
            if (item.Classification == "PICKABLE_AGGREGATION_REQUIRED")
                _report.PickableAggregationRequired++;
        }

        private static void Complete(bool cancelled, string detail)
        {
            if (_report == null)
                return;
            _report.Running = false;
            _report.Completed = !cancelled;
            _report.Cancelled = cancelled;
            _report.Detail = detail;
            _report.ElapsedSeconds = ElapsedSeconds();
            try { _stopwatch?.Stop(); }
            catch { }

            _logger?.LogWarning(
                "[GSP][HOUSE_CENSUS] final | " + _report.ToSummaryText());
            _logger?.LogInfo(
                "[GSP][HOUSE_CENSUS] classification-meaning" +
                " | PICKABLE_GEOMETRY_READY=single-route and mathematically watertight, but still needs convex/compound dynamic proxy" +
                " | PICKABLE_REPAIR_REQUIRED=pickable heuristic plus open/non-manifold/duplicate defect" +
                " | PICKABLE_AGGREGATION_REQUIRED=possible pickable object has multiple renderers/routes; whole object not yet judged" +
                " | OPEN_STATIC_SURFACE=not a closed volume; may still become a static surface after separate cooking/contact audit" +
                " | IMPORTANT=this census never creates a Collider and never claims dynamic-rigidbody readiness");
            Pending.Clear();
        }

        private static float ElapsedSeconds()
        {
            try
            {
                if (_stopwatch != null)
                    return (float)_stopwatch.Elapsed.TotalSeconds;
            }
            catch { }
            return 0f;
        }

        private static string[] ResolveRooms(string roomFilter)
        {
            if (string.IsNullOrWhiteSpace(roomFilter) ||
                string.Equals(roomFilter.Trim(), "all", StringComparison.OrdinalIgnoreCase))
                return DefaultRooms;

            string requested = roomFilter.Trim();
            for (int i = 0; i < DefaultRooms.Length; i++)
            {
                if (string.Equals(requested, DefaultRooms[i], StringComparison.OrdinalIgnoreCase))
                    return new[] { DefaultRooms[i] };
            }
            return DefaultRooms;
        }

        private static GameObject FindByExactPath(string expectedPath)
        {
            Scene scene = SceneManager.GetActiveScene();
            GameObject[] roots;
            try { roots = scene.GetRootGameObjects(); }
            catch { return null; }
            for (int i = 0; i < roots.Length; i++)
            {
                GameObject found = FindRecursive(roots[i], expectedPath, 0);
                if (found != null)
                    return found;
            }
            return null;
        }

        private static GameObject FindRecursive(
            GameObject current,
            string expectedPath,
            int depth)
        {
            if (current == null || depth > 128)
                return null;
            if (string.Equals(
                TargetFinder.GetPath(current.transform),
                expectedPath,
                StringComparison.Ordinal))
                return current;

            int count = Safe(() => current.transform.childCount, 0);
            for (int i = 0; i < count; i++)
            {
                Transform child = null;
                try { child = current.transform.GetChild(i); }
                catch { }
                if (child == null)
                    continue;
                GameObject found = FindRecursive(child.gameObject, expectedPath, depth + 1);
                if (found != null)
                    return found;
            }
            return null;
        }

        private static Transform FindDirectChild(Transform parent, string name)
        {
            if (parent == null)
                return null;
            int count = Safe(() => parent.childCount, 0);
            for (int i = 0; i < count; i++)
            {
                Transform child = null;
                try { child = parent.GetChild(i); }
                catch { }
                if (child != null && string.Equals(
                    child.name,
                    name,
                    StringComparison.OrdinalIgnoreCase))
                    return child;
            }
            return null;
        }

        private static int CountRigidbodiesRecursive(Transform root, int depth)
        {
            if (root == null || depth > 128)
                return 0;
            int count = 0;
            try
            {
                // 避免本游戏 IL2CPP 中 GetComponentsInChildren<Rigidbody>() 的
                // 泛型 MethodInfoStore 初始化风险；逐 Transform 读取单个 Rigidbody。
                if (root.GetComponent<Rigidbody>() != null)
                    count++;
            }
            catch { }
            int children = Safe(() => root.childCount, 0);
            for (int i = 0; i < children; i++)
            {
                Transform child = null;
                try { child = root.GetChild(i); }
                catch { }
                if (child != null)
                    count += CountRigidbodiesRecursive(child, depth + 1);
            }
            return count;
        }

        private static bool TryGetBounds(GameObject root, out Bounds bounds)
        {
            bounds = new Bounds();
            bool has = false;
            Renderer[] renderers =
                SafeGetComponentsInChildren<Renderer>(root);
            for (int i = 0; i < renderers.Length; i++)
            {
                Renderer renderer = renderers[i];
                if (renderer == null)
                    continue;
                Bounds item = Safe(() => renderer.bounds, default(Bounds));
                if (!has) { bounds = item; has = true; }
                else bounds.Encapsulate(item);
            }
            Collider[] colliders =
                SafeGetComponentsInChildren<Collider>(root);
            for (int i = 0; i < colliders.Length; i++)
            {
                Collider collider = colliders[i];
                if (collider == null)
                    continue;
                Bounds item = Safe(() => collider.bounds, default(Bounds));
                if (!has) { bounds = item; has = true; }
                else bounds.Encapsulate(item);
            }
            return has;
        }

        private static string FormatBounds(Bounds bounds)
        {
            return
                "center=(" + bounds.center.x.ToString("F4") + "," +
                bounds.center.y.ToString("F4") + "," +
                bounds.center.z.ToString("F4") + ")/size=(" +
                bounds.size.x.ToString("F4") + "," +
                bounds.size.y.ToString("F4") + "," +
                bounds.size.z.ToString("F4") + ")";
        }

        private static void Sort3(ref int a, ref int b, ref int c)
        {
            if (a > b) { int t = a; a = b; b = t; }
            if (b > c) { int t = b; b = c; c = t; }
            if (a > b) { int t = a; a = b; b = t; }
        }

        private static T Safe<T>(Func<T> getter, T fallback)
        {
            try { return getter(); }
            catch { return fallback; }
        }

        private static T[] SafeGetComponentsInChildren<T>(GameObject target)
            where T : Component
        {
            if (target == null)
                return Array.Empty<T>();

            try
            {
                return target.GetComponentsInChildren<T>(true) ??
                    Array.Empty<T>();
            }
            catch
            {
                return Array.Empty<T>();
            }
        }
    }
}
