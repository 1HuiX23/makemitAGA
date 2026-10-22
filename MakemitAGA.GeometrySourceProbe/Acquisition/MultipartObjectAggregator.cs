using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using BepInEx;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;
using MakemitAGA.GeometrySourceProbe.Runtime;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// 多 Renderer / 多路由 SubMesh 的只读聚合审计。
    ///
    /// 两套聚合结果：
    /// 1. Observation：实体主体 + 视觉表面，排除阴影/特效；
    /// 2. Collision：只包含物理主体/物理细节。
    ///
    /// 本模块不会创建 Unity Mesh、Collider、Rigidbody 或 PhysicMaterial。
    /// 空间焊接只用于拓扑审计副本，不会改变原始视觉几何。
    /// 每条路由在进入聚合前都会独立验证 raw/world-baked 与 renderer localToWorld 两种坐标候选，
    /// 只有引用顶点落在 Renderer 世界 bounds 内的候选才会被接受。
    /// </summary>
    internal static class MultipartObjectAggregator
    {
        private const float UpdateDelaySeconds = 0.04f;
        private const string FullReportFileName =
            "MakemitAGA.GeometrySourceProbe.MultipartInstanceReport.txt";

        private sealed class RouteWork
        {
            public MeshRenderer Renderer;
            public Mesh Mesh;
            public int RoutedSubMeshIndex;
            public int RoutedCount;
            public string MaterialName;
            public string ShaderName;
            public ObjectPhysicsWhitelist.PartDecision Decision;
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

        private static ManualLogSource _logger;
        private static GameObject _target;
        private static readonly List<RouteWork> Pending = new List<RouteWork>();
        private static readonly List<Vector3> ObservationPositions = new List<Vector3>();
        private static readonly List<int> ObservationIndices = new List<int>();
        private static readonly List<Vector3> CollisionPositions = new List<Vector3>();
        private static readonly List<int> CollisionIndices = new List<int>();
        private static readonly List<int> ObservationTriangleRouteIds = new List<int>();
        private static readonly List<int> CollisionTriangleRouteIds = new List<int>();

        // 最近一次成功审计的碰撞聚合几何快照。它只保存 GSP 已经验证并统一到世界坐标的
        // 托管数组，用于后续 STATIC_NONCONVEX_MESH 静态被动碰撞体创建。
        // 新审计、场景切换或运行时清理会主动丢弃该快照，避免跨目标误用。
        private static Vector3[] CompletedCollisionPositions;
        private static int[] CompletedCollisionIndices;
        private static Bounds CompletedCollisionBounds;
        private static string CompletedCollisionTargetName;
        private static string CompletedCollisionTargetPath;
        private static bool CompletedCollisionSnapshotReady;

        private static MultipartAggregationReport _report;
        private static Stopwatch _stopwatch;
        private static int _nextIndex;
        private static float _nextAllowedRealtime;

        public static bool IsRunning => _report != null && _report.Running;
        public static MultipartAggregationReport CurrentReport => _report;

        /// <summary>
        /// 复制最近一次成功多部件审计的世界空间碰撞聚合几何。
        /// 调用方获得独立数组所有权；本方法不会创建 Unity Mesh，也不会回读源 Mesh/GPU buffer。
        /// </summary>
        public static bool TryCopyCompletedCollisionGeometry(
            out Vector3[] worldPositions,
            out int[] indices,
            out Bounds worldBounds,
            out string targetName,
            out string targetPath,
            out string error)
        {
            worldPositions = null;
            indices = null;
            worldBounds = default;
            targetName = CompletedCollisionTargetName;
            targetPath = CompletedCollisionTargetPath;
            error = null;

            if (!CompletedCollisionSnapshotReady ||
                CompletedCollisionPositions == null ||
                CompletedCollisionIndices == null)
            {
                error = "completed-collision-geometry-snapshot-not-ready";
                return false;
            }

            try
            {
                worldPositions = (Vector3[])CompletedCollisionPositions.Clone();
                indices = (int[])CompletedCollisionIndices.Clone();
                worldBounds = CompletedCollisionBounds;
                return true;
            }
            catch (Exception e)
            {
                worldPositions = null;
                indices = null;
                worldBounds = default;
                error = e.GetType().Name + ":" + e.Message;
                return false;
            }
        }


        /// <summary>
        /// v0.2.3 active-room incremental floor atlas 使用的精确目标入口。
        /// 调用方已经在 HouseFloorRegistry 中完成 room-scoped 唯一绑定，因此这里绝不再按
        /// “同名 + 相机最近”重新选择目标。其余 D3D11 readback / 世界坐标规范化 / 语义过滤 /
        /// 拓扑与空间实例审计完全复用普通 multipart pipeline。
        /// </summary>
        public static string StartExactTarget(
            ManualLogSource logger,
            GameObject target,
            out MultipartAggregationReport report)
        {
            Cancel("restart");
            _logger = logger;
            ClearWorkingGeometry();
            Pending.Clear();

            _report = new MultipartAggregationReport
            {
                Started = true,
                Running = false,
                Completed = false,
                SceneName = SceneManager.GetActiveScene().name,
                TargetName = target != null ? target.name : null,
                Detail = "starting-exact-target"
            };
            report = _report;

            if (target == null || !target.activeInHierarchy)
            {
                _report.Completed = true;
                _report.Detail = "exact-target-null-or-inactive";
                return "<color=red>精确多部件审计启动失败：</color> target null/inactive。";
            }
            if ((target.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal))
            {
                _report.Completed = true;
                _report.Detail = "exact-target-is-gsp-owned";
                return "<color=red>精确多部件审计拒绝 GSP 自有对象。</color>";
            }

            _target = target;
            _report.ExactMatches = 1;
            _report.SelectedDistance = 0f;
            _report.ReferenceName = "HouseFloorRegistryExactTarget";
            _report.TargetName = _target.name;
            _report.TargetPath = TargetFinder.GetPath(_target.transform);

            ObjectPhysicsWhitelist.Reload(logger);
            SpatialInstancePhysicsWhitelist.Reload(logger);
            _report.PhysicsProfile = ObjectPhysicsWhitelist.ResolveObjectProfile(
                _report.TargetPath,
                _report.TargetName);

            MeshRenderer[] renderers = SafeGetComponentsInChildren<MeshRenderer>(_target);
            SkinnedMeshRenderer[] skinned = SafeGetComponentsInChildren<SkinnedMeshRenderer>(_target);
            _report.MeshRenderers = renderers.Length;
            _report.SkinnedRenderers = skinned.Length;
            _report.TargetRendererBounds = CalculateRendererAggregateBounds(renderers);

            for (int i = 0; i < renderers.Length; i++)
                AddRendererRoutes(renderers[i]);

            _report.RoutesQueued = Pending.Count;
            _report.Running = Pending.Count > 0;
            _report.Completed = Pending.Count == 0;
            _report.Detail = Pending.Count > 0
                ? "queued-exact-target; one renderer route per update; no Unity Mesh/Collider/Rigidbody creation"
                : (skinned.Length > 0
                    ? "no static MeshRenderer routes; skinned routes deferred"
                    : "no MeshRenderer routes");

            _nextIndex = 0;
            _nextAllowedRealtime = Time.realtimeSinceStartup + 0.1f;
            _stopwatch = Stopwatch.StartNew();

            _logger?.LogWarning(
                "[GSP][MULTIPART] begin-exact-target" +
                " | build=0.2.3-active-room-incremental-floor-atlas-foundation" +
                " | target=" + _report.TargetName +
                " | path=" + _report.TargetPath +
                " | renderers=mesh:" + renderers.Length +
                ",skinned:" + skinned.Length +
                " | routes=" + Pending.Count +
                " | binding=HouseFloorRegistryExactTarget" +
                " | readback=read-only-D3D11-staging" +
                " | coordinateNormalization=per-route-raw-vs-localToWorld-with-renderer-bounds-gate" +
                " | colliderCreation=False" +
                " | rigidbodyCreation=False");

            if (Pending.Count == 0)
                return "<color=yellow>精确目标存在，但没有可处理的普通 MeshRenderer 路由。</color> " +
                    "SkinnedRenderer=" + skinned.Length;

            return "<color=#66ddff>精确多部件聚合审计已启动。</color> target=" +
                _report.TargetName + "，routes=" + Pending.Count + "。";
        }

        public static string Start(
            ManualLogSource logger,
            string targetName,
            out MultipartAggregationReport report)
        {
            Cancel("restart");
            _logger = logger;
            ClearWorkingGeometry();
            Pending.Clear();

            _report = new MultipartAggregationReport
            {
                Started = true,
                Running = false,
                Completed = false,
                SceneName = SceneManager.GetActiveScene().name,
                TargetName = targetName,
                Detail = "starting"
            };
            report = _report;

            if (string.IsNullOrWhiteSpace(targetName))
            {
                _report.Completed = true;
                _report.Detail = "target-name-empty";
                return "用法：gsp_multipart_audit(对象名称)";
            }

            _target = TargetFinder.FindNearestExactName(
                targetName.Trim(),
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);
            _report.ExactMatches = exactMatches;
            _report.SelectedDistance = distance;
            _report.ReferenceName = referenceName;

            if (_target == null)
            {
                _report.Completed = true;
                _report.Detail = error;
                _logger?.LogWarning(
                    "[GSP][MULTIPART] start-failed | target=" + targetName +
                    " | error=" + error);
                return "<color=red>多部件聚合审计启动失败：</color> " + error;
            }

            _report.TargetName = _target.name;
            _report.TargetPath = TargetFinder.GetPath(_target.transform);
            ObjectPhysicsWhitelist.Reload(logger);
            SpatialInstancePhysicsWhitelist.Reload(logger);
            _report.PhysicsProfile = ObjectPhysicsWhitelist.ResolveObjectProfile(
                _report.TargetPath,
                _report.TargetName);

            MeshRenderer[] renderers = SafeGetComponentsInChildren<MeshRenderer>(_target);
            SkinnedMeshRenderer[] skinned = SafeGetComponentsInChildren<SkinnedMeshRenderer>(_target);
            _report.MeshRenderers = renderers.Length;
            _report.SkinnedRenderers = skinned.Length;
            _report.TargetRendererBounds = CalculateRendererAggregateBounds(renderers);

            for (int i = 0; i < renderers.Length; i++)
                AddRendererRoutes(renderers[i]);

            _report.RoutesQueued = Pending.Count;
            _report.Running = Pending.Count > 0;
            _report.Completed = Pending.Count == 0;
            _report.Detail = Pending.Count > 0
                ? "queued; one renderer route per update; no Unity Mesh/Collider/Rigidbody creation"
                : (skinned.Length > 0
                    ? "no static MeshRenderer routes; skinned routes deferred"
                    : "no MeshRenderer routes");

            _nextIndex = 0;
            _nextAllowedRealtime = Time.realtimeSinceStartup + 0.1f;
            _stopwatch = Stopwatch.StartNew();

            _logger?.LogWarning(
                "[GSP][MULTIPART] begin" +
                " | build=0.2.3-active-room-incremental-floor-atlas-foundation" +
                " | target=" + _report.TargetName +
                " | path=" + _report.TargetPath +
                " | matches=" + exactMatches +
                " | renderers=mesh:" + renderers.Length +
                ",skinned:" + skinned.Length +
                " | routes=" + Pending.Count +
                " | partWhitelist=" + ObjectPhysicsWhitelist.Path +
                " | instanceWhitelist=" + SpatialInstancePhysicsWhitelist.Path +
                " | aggregation=observation-and-collision-separated" +
                " | topology=raw-plus-position-welded-audit-copy" +
                " | spatialDecomposition=welded-edge-components-plus-adaptive-AABB-instance-clustering" +
                " | coordinateNormalization=per-route-raw-vs-localToWorld-with-renderer-bounds-gate" +
                " | colliderCreation=False" +
                " | rigidbodyCreation=False" +
                " | IMPORTANT=shadow/effect routes default excluded; whitelist may override every part");

            if (Pending.Count == 0)
                return "<color=yellow>找到对象，但没有可处理的普通 MeshRenderer 路由。</color> " +
                    "SkinnedRenderer=" + skinned.Length;

            return
                "<color=#66ddff>多部件聚合审计已启动。</color> target=" +
                _report.TargetName + "，routes=" + Pending.Count +
                "。每帧处理一个部件；执行 gsp_multipart_status 查看进度。";
        }

        public static void Update()
        {
            if (_report == null || !_report.Running)
                return;
            if (Time.realtimeSinceStartup < _nextAllowedRealtime)
                return;

            if (_nextIndex >= Pending.Count)
            {
                Complete(false, "all-routes-processed");
                return;
            }

            RouteWork work = Pending[_nextIndex];
            _nextIndex++;
            ProcessRoute(work, _nextIndex);
            _report.RoutesProcessed++;
            _report.ElapsedSeconds = ElapsedSeconds();
            _nextAllowedRealtime = Time.realtimeSinceStartup + UpdateDelaySeconds;

            if (_nextIndex >= Pending.Count)
                Complete(false, "all-routes-processed");
        }

        public static string Cancel(string reason)
        {
            bool existed = _report != null || Pending.Count > 0;
            if (_report != null && _report.Running)
                Complete(true, reason ?? "cancelled");
            Pending.Clear();
            ClearWorkingGeometry();
            _target = null;
            _nextIndex = 0;
            _nextAllowedRealtime = 0f;
            if (string.Equals(reason, "restart", StringComparison.Ordinal) ||
                (!string.IsNullOrEmpty(reason) &&
                 reason.StartsWith("runtime-clear:", StringComparison.Ordinal)))
            {
                ClearCompletedCollisionGeometry();
            }
            if (!string.IsNullOrEmpty(reason) &&
                reason.StartsWith("runtime-clear:", StringComparison.Ordinal))
            {
                _report = null;
                _stopwatch = null;
            }
            return existed
                ? "GSP 多部件聚合审计已停止。"
                : "当前没有运行中的多部件聚合审计。";
        }

        public static string GetStatusText()
        {
            if (_report == null)
                return "当前没有多部件聚合审计。执行 gsp_multipart_audit(对象名称)。";
            _report.ElapsedSeconds = ElapsedSeconds();
            return _report.ToStatusText();
        }

        public static string GetReportText()
        {
            if (_report == null)
                return "当前没有多部件聚合审计报告。执行 gsp_multipart_audit(对象名称)。";
            _report.ElapsedSeconds = ElapsedSeconds();
            return _report.ToSummaryText();
        }

        /// <summary>
        /// 把完整实例报告写入独立文本文件，游戏内控制台只返回短摘要。
        /// 这样不会再让数万字符的报告被 Unity 文本系统转换成超过 65000 顶点的文字 Mesh。
        /// </summary>
        public static string WriteFullReportAndGetConsoleSummary()
        {
            if (_report == null)
                return "当前没有多部件聚合审计报告。执行 gsp_instance_audit(对象名称)。";

            _report.ElapsedSeconds = ElapsedSeconds();
            string fullText = _report.ToFullText();
            string configPath;
            try { configPath = Paths.ConfigPath; }
            catch { configPath = AppContext.BaseDirectory; }
            string path = System.IO.Path.Combine(configPath, FullReportFileName);

            try
            {
                Directory.CreateDirectory(configPath);
                StringBuilder file = new StringBuilder(fullText.Length + 512);
                file.AppendLine("# MakemitAGA GSP multipart/spatial instance full report");
                file.AppendLine("# Generated: " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"));
                file.AppendLine("# Target: " + (_report.TargetPath ?? _report.TargetName ?? "<none>"));
                file.AppendLine("# The game console intentionally shows only a short summary to avoid Unity text-mesh vertex overflow.");
                file.AppendLine();
                file.Append(fullText);
                File.WriteAllText(path, file.ToString(), Encoding.UTF8);

                _logger?.LogInfo(
                    "[GSP][MULTIPART] full-report-exported" +
                    " | path=" + path +
                    " | chars=" + fullText.Length +
                    " | target=" + _report.TargetName);
                return BuildSafeConsoleReportSummary(path, fullText.Length);
            }
            catch (Exception e)
            {
                string error = e.GetType().Name + ":" + e.Message;
                _logger?.LogWarning(
                    "[GSP][MULTIPART] full-report-export-failed | " + error);
                return
                    "<color=red>完整实例报告写入失败：</color> " + error + "\n" +
                    BuildSafeConsoleReportSummary("<write-failed>", fullText.Length);
            }
        }

        private static string BuildSafeConsoleReportSummary(
            string path,
            int fullTextCharacters)
        {
            SpatialInstanceAuditReport observation = _report != null
                ? _report.ObservationSpatialInstances
                : null;
            SpatialInstanceAuditReport collision = _report != null
                ? _report.CollisionSpatialInstances
                : null;

            StringBuilder sb = new StringBuilder(1200);
            sb.AppendLine("<color=#66ddff>GSP 实例报告摘要（游戏内安全输出）</color>");
            sb.AppendLine(
                "目标=" + (_report != null ? _report.TargetName : "<none>") +
                " | completed=" + (_report != null && _report.Completed) +
                " | success=" + (_report != null && _report.Success));
            sb.AppendLine(
                "路由=" + (_report != null ? _report.RoutesProcessed : 0) +
                "/" + (_report != null ? _report.RoutesQueued : 0) +
                " | readback=" + (_report != null ? _report.RoutesReadbackSucceeded : 0) +
                "/" + (_report != null ? _report.RoutesReadbackAttempted : 0) +
                " | 阴影/特效排除=" + (_report != null ? _report.ExcludedEffectRoutes : 0) +
                " | 独立子物体排除=" + (_report != null ? _report.ExcludedChildObjectRoutes : 0));
            sb.AppendLine(
                "观察实例=" + (observation != null ? observation.SpatialInstances : 0) +
                " | 碰撞实例=" + (collision != null ? collision.SpatialInstances : 0) +
                " | 代理草案=" + (collision != null ? collision.ProxyDrafts : 0) +
                " | 实例配置匹配=" + (collision != null ? collision.MatchedInstanceProfiles : 0));
            sb.AppendLine(
                "观察几何=v:" + (_report != null ? _report.ObservationVertices : 0) +
                ",t:" + (_report != null ? _report.ObservationTriangles : 0) +
                " | 碰撞几何=v:" + (_report != null ? _report.CollisionVertices : 0) +
                ",t:" + (_report != null ? _report.CollisionTriangles : 0));
            sb.AppendLine("完整报告字符数=" + fullTextCharacters);
            sb.AppendLine("完整报告文件=" + path);
            sb.Append("详细部件、连通组件、PCA 定向盒和白名单内容已写入文件，不再一次性绘制到游戏控制台。");
            return sb.ToString();
        }

        private static void AddRendererRoutes(MeshRenderer renderer)
        {
            if (renderer == null)
                return;
            MeshFilter filter = null;
            Mesh mesh = null;
            try
            {
                filter = renderer.GetComponent<MeshFilter>();
                mesh = filter != null ? filter.sharedMesh : null;
            }
            catch { }
            if (mesh == null)
                return;

            int start = Safe(() => renderer.subMeshStartIndex, 0);
            int subMeshCount = Safe(() => mesh.subMeshCount, 0);
            if (start < 0 || start >= subMeshCount)
                return;

            int materialCount = 0;
            try
            {
                var materials = renderer.sharedMaterials;
                materialCount = materials != null ? materials.Length : 0;
            }
            catch { }
            int routedCount = materialCount > 0 ? materialCount : 1;
            if (start + routedCount > subMeshCount)
                routedCount = subMeshCount - start;
            if (routedCount <= 0)
                return;

            for (int localRoute = 0; localRoute < routedCount; localRoute++)
            {
                int routeIndex = start + localRoute;
                string materialName = GetMaterialName(renderer, localRoute);
                string shaderName = GetShaderName(renderer, localRoute);
                string rendererPath = TargetFinder.GetPath(renderer.transform);
                string autoRole;
                bool observation;
                bool collision;
                string reason;
                EvaluateAutomaticRole(
                    renderer,
                    materialName,
                    shaderName,
                    out autoRole,
                    out observation,
                    out collision,
                    out reason);

                ObjectPhysicsWhitelist.PartDecision decision =
                    ObjectPhysicsWhitelist.ResolvePart(
                        _report.TargetPath,
                        _report.TargetName,
                        rendererPath,
                        renderer.name,
                        materialName,
                        shaderName,
                        autoRole,
                        observation,
                        collision,
                        reason);

                Pending.Add(new RouteWork
                {
                    Renderer = renderer,
                    Mesh = mesh,
                    RoutedSubMeshIndex = routeIndex,
                    RoutedCount = routedCount,
                    MaterialName = materialName,
                    ShaderName = shaderName,
                    Decision = decision
                });
            }
        }

        private static void ProcessRoute(RouteWork work, int sequence)
        {
            MultipartPartReport part = new MultipartPartReport
            {
                Sequence = sequence,
                RendererName = work.Renderer != null ? work.Renderer.name : "<null>",
                RendererPath = TargetFinder.GetPath(
                    work.Renderer != null ? work.Renderer.transform : null),
                RoutedSubMeshIndex = work.RoutedSubMeshIndex,
                MaterialName = work.MaterialName,
                ShaderName = work.ShaderName,
                RendererEnabled = Safe(() => work.Renderer.enabled, false),
                ActiveInHierarchy = work.Renderer != null && work.Renderer.gameObject.activeInHierarchy,
                ShadowCastingMode = Safe(() => work.Renderer.shadowCastingMode.ToString(), "<unknown>"),
                AutoRole = work.Decision.AutoRole,
                ResolvedRole = work.Decision.ResolvedRole,
                WhitelistMatched = work.Decision.Matched,
                WhitelistSource = work.Decision.RuleSource,
                IncludeObservation = work.Decision.IncludeObservation,
                IncludeCollision = work.Decision.IncludeCollision,
                DecisionReason = work.Decision.Reason,
                ReadbackDetail = "not-attempted"
            };

            CountRole(part.ResolvedRole);
            if (part.WhitelistMatched)
                _report.WhitelistOverrides++;

            bool needsGeometry = part.IncludeObservation || part.IncludeCollision;
            D3D11SubmeshGeometry geometry = null;
            D3D11FullSubmeshReadbackReport readback = null;
            if (needsGeometry)
            {
                part.ReadbackAttempted = true;
                _report.RoutesReadbackAttempted++;
                try
                {
                    readback = D3D11FullSubmeshReadback.ProbeRoute(
                        _target,
                        work.Renderer,
                        work.RoutedSubMeshIndex,
                        null,
                        out geometry);
                }
                catch (Exception e)
                {
                    part.ReadbackSuccess = false;
                    part.ReadbackDetail = e.GetType().Name + ":" + e.Message;
                }
            }
            else
            {
                part.ReadbackDetail = "excluded-before-readback-by-role";
                part.ResolvedCoordinateMode = "not-required-by-role";
                part.CoordinateReason = "route excluded before readback by resolved role";
            }

            if (readback != null)
            {
                part.ReadbackSuccess = readback.Success;
                part.ReadbackDetail = readback.Detail;
                part.Vertices = readback.DecodedPositionCount;
                part.Indices = readback.DecodedIndexCount;
                part.Triangles = readback.TriangleCount;
                part.ZeroAreaTriangles = readback.ZeroAreaTriangleCount;
                part.ReadbackCoordinateMode = readback.CoordinateMode;
                part.RawBounds = readback.RawDecodedBounds;
                part.TransformedBounds = readback.TransformedDecodedBounds;
                part.RendererBounds = readback.RendererBounds;
            }

            if (part.ReadbackSuccess && geometry != null)
            {
                _report.RoutesReadbackSucceeded++;
                NormalizeRouteToWorld(work.Renderer, geometry, part);
                if (part.AggregationAccepted)
                {
                    _report.CoordinateAcceptedRoutes++;
                    if (part.CoordinateAmbiguous)
                        _report.CoordinateAmbiguousRoutes++;
                    if (part.TransformApplied)
                        _report.CoordinateTransformedRoutes++;
                    else
                        _report.CoordinateRawWorldRoutes++;

                    if (part.IncludeObservation)
                        AppendGeometry(geometry, ObservationPositions, ObservationIndices, ObservationTriangleRouteIds, sequence);
                    if (part.IncludeCollision)
                        AppendGeometry(geometry, CollisionPositions, CollisionIndices, CollisionTriangleRouteIds, sequence);
                }
                else
                {
                    _report.CoordinateRejectedRoutes++;
                }
            }

            _report.LastPart = part.RendererName + "/subMesh" + part.RoutedSubMeshIndex;
            _report.Parts.Add(part);
            _logger?.LogInfo("[GSP][MULTIPART][PART] " + part.ToLogLine());
        }

        private static void LogSpatialAudit(
            SpatialInstanceAuditReport audit,
            bool includeKeyComponents)
        {
            if (audit == null)
                return;
            _logger?.LogWarning("[GSP][SPATIAL] summary | " + audit.ToCompactText(audit.Channel));
            for (int i = 0; i < audit.Instances.Count; i++)
            {
                SpatialInstanceReport instance = audit.Instances[i];
                if (instance == null)
                    continue;
                _logger?.LogInfo("[GSP][SPATIAL][INSTANCE] " + instance.ToLogLine());
                if (instance.CompoundProxyDraftBuilt)
                {
                    _logger?.LogWarning(
                        "[GSP][SPATIAL][COMPOUND_DRAFT] target=" + _report.TargetName +
                        " | key=" + instance.InstanceKey +
                        " | parts=" + instance.CompoundProxyParts +
                        " | fit=" + instance.CompoundProxyFitMethod +
                        " | reason=" + instance.CompoundProxyReviewReason);
                    for (int p = 0; p < instance.CompoundParts.Count; p++)
                    {
                        CompoundBoxPartReport part = instance.CompoundParts[p];
                        if (part != null)
                            _logger?.LogInfo("[GSP][SPATIAL][COMPOUND_PART] " + part.ToLogLine());
                    }
                }
                if (instance.NeedsUnityExplorerReview)
                {
                    _logger?.LogWarning(
                        "[GSP][SPATIAL][UE_REVIEW] target=" + _report.TargetName +
                        " | key=" + instance.InstanceKey +
                        " | channel=" + instance.Channel +
                        " | bounds=" + instance.Bounds +
                        " | renderers=" + instance.SourceRenderers +
                        " | reason=" + instance.ReviewReason);
                }
            }

            if (!includeKeyComponents)
            {
                _logger?.LogInfo(
                    "[GSP][SPATIAL][COMPONENTS_OMITTED] channel=" + audit.Channel +
                    " | total=" + audit.Components.Count +
                    " | reason=observation-channel-usually-mirrors-collision");
                return;
            }

            List<SpatialComponentReport> selected = SelectKeyComponents(audit.Components, 14);
            for (int i = 0; i < selected.Count; i++)
            {
                SpatialComponentReport component = selected[i];
                if (component != null)
                    _logger?.LogInfo("[GSP][SPATIAL][KEY_COMPONENT] " + component.ToLogLine());
            }
            if (selected.Count < audit.Components.Count)
            {
                _logger?.LogInfo(
                    "[GSP][SPATIAL][COMPONENTS_TRUNCATED] channel=" + audit.Channel +
                    " | total=" + audit.Components.Count +
                    " | logged=" + selected.Count +
                    " | selection=area/dimension/review-balanced" +
                    " | fullReport=" + FullReportFileName);
            }
        }

        private static List<SpatialComponentReport> SelectKeyComponents(
            List<SpatialComponentReport> source,
            int limit)
        {
            List<SpatialComponentReport> result = new List<SpatialComponentReport>();
            if (source == null || source.Count == 0 || limit <= 0)
                return result;
            if (source.Count <= limit)
            {
                result.AddRange(source);
                return result;
            }
            List<SpatialComponentReport> ranked = new List<SpatialComponentReport>(source);
            ranked.Sort((a, b) =>
            {
                if (a == null) return 1;
                if (b == null) return -1;
                float scoreA = a.SurfaceArea * 100f + a.MaxDimension * 10f +
                    (a.NeedsUnityExplorerReview ? 1f : 0f);
                float scoreB = b.SurfaceArea * 100f + b.MaxDimension * 10f +
                    (b.NeedsUnityExplorerReview ? 1f : 0f);
                int score = scoreB.CompareTo(scoreA);
                if (score != 0) return score;
                return a.ComponentIndex.CompareTo(b.ComponentIndex);
            });
            HashSet<string> keys = new HashSet<string>(StringComparer.Ordinal);
            for (int i = 0; i < ranked.Count && result.Count < limit; i++)
            {
                SpatialComponentReport item = ranked[i];
                if (item == null)
                    continue;
                string key = item.ComponentKey ?? ("index:" + item.ComponentIndex);
                if (keys.Add(key))
                    result.Add(item);
            }
            result.Sort((a, b) => a.ComponentIndex.CompareTo(b.ComponentIndex));
            return result;
        }

        public static string ExportSpatialInstanceDraft()
        {
            if (_report == null || !_report.Completed || !_report.Success)
                return "当前没有已完成且成功的空间实例报告。请先执行 gsp_instance_audit(对象名称)。";

            string configPath;
            try { configPath = Paths.ConfigPath; }
            catch { configPath = AppContext.BaseDirectory; }
            string path = System.IO.Path.Combine(
                configPath,
                "MakemitAGA.GeometrySourceProbe.SpatialInstanceDraft.txt");
            try
            {
                StringBuilder sb = new StringBuilder();
                sb.AppendLine("# GSP spatial instance mapping draft; generated from read-only mainline audit.");
                sb.AppendLine("# This file is not applied automatically and is not yet the authoritative physics whitelist.");
                sb.AppendLine("# targetPath|instanceKey|channel|associatedCollisionKey|center|size|sourceRoutes|sourceRenderers|recommendedRole|proxyType|proxyCenter|proxyRawSize|proxySize|proxyEuler|proxyRatioToAabb|profileMatched|fixedToWorld|pickable|massKg|staticFriction|dynamicFriction|bounciness|materialTag|notes");
                AppendDraftInstances(sb, _report.CollisionSpatialInstances);
                AppendDraftInstances(sb, _report.ObservationSpatialInstances);
                File.WriteAllText(path, sb.ToString());
                _logger?.LogInfo(
                    "[GSP][SPATIAL] draft-exported | path=" + path +
                    " | target=" + _report.TargetName);
                return "已导出空间实例映射草案：" + path;
            }
            catch (Exception e)
            {
                string error = e.GetType().Name + ":" + e.Message;
                _logger?.LogWarning("[GSP][SPATIAL] draft-export-failed | " + error);
                return "导出空间实例映射草案失败：" + error;
            }
        }

        private static void AppendDraftInstances(
            StringBuilder sb,
            SpatialInstanceAuditReport audit)
        {
            if (sb == null || audit == null)
                return;
            for (int i = 0; i < audit.Instances.Count; i++)
            {
                SpatialInstanceReport item = audit.Instances[i];
                if (item == null)
                    continue;
                sb.Append(CleanDraft(_report.TargetPath)).Append('|')
                  .Append(CleanDraft(item.InstanceKey)).Append('|')
                  .Append(CleanDraft(item.Channel)).Append('|')
                  .Append(CleanDraft(item.AssociatedCollisionInstanceKey)).Append('|')
                  .Append(CleanDraft(item.Center.ToString("G9"))).Append('|')
                  .Append(CleanDraft(item.Size.ToString("G9"))).Append('|')
                  .Append(CleanDraft(item.SourceRouteIds)).Append('|')
                  .Append(CleanDraft(item.SourceRenderers)).Append('|')
                  .Append(CleanDraft(item.RecommendedRole)).Append('|')
                  .Append(CleanDraft(item.ProxyDraftType)).Append('|')
                  .Append(CleanDraft(item.ProxyCenter.ToString("G9"))).Append('|')
                  .Append(CleanDraft(item.ProxyRawSize.ToString("G9"))).Append('|')
                  .Append(CleanDraft(item.ProxySize.ToString("G9"))).Append('|')
                  .Append(CleanDraft(item.ProxyEulerDegrees.ToString("G9"))).Append('|')
                  .Append(item.ProxyVolumeRatioToAabb.ToString("G9")).Append('|');
                SpatialInstancePhysicsProfileSnapshot profile = item.PhysicsProfile;
                sb.Append(profile != null && profile.Matched).Append('|')
                  .Append(profile != null && profile.FixedToWorldConfigured ? profile.FixedToWorld.ToString() : string.Empty).Append('|')
                  .Append(profile != null && profile.PickableConfigured ? profile.Pickable.ToString() : string.Empty).Append('|')
                  .Append(profile != null && profile.MassConfigured ? profile.MassKg.ToString("G9") : string.Empty).Append('|')
                  .Append(profile != null && profile.StaticFrictionConfigured ? profile.StaticFriction.ToString("G9") : string.Empty).Append('|')
                  .Append(profile != null && profile.DynamicFrictionConfigured ? profile.DynamicFriction.ToString("G9") : string.Empty).Append('|')
                  .Append(profile != null && profile.BouncinessConfigured ? profile.Bounciness.ToString("G9") : string.Empty).Append('|')
                  .Append(CleanDraft(profile != null ? profile.MaterialTag : null)).Append('|')
                  .Append(CleanDraft(profile != null && !string.IsNullOrEmpty(profile.Notes) ? profile.Notes : item.ReviewReason))
                  .AppendLine();
            }
        }

        private static string CleanDraft(string value)
        {
            if (string.IsNullOrEmpty(value))
                return string.Empty;
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }

        private static void CountRole(string role)
        {
            if (string.Equals(role, "PHYSICAL_CORE", StringComparison.Ordinal) ||
                string.Equals(role, "PHYSICAL_DETAIL", StringComparison.Ordinal))
                _report.PhysicalRoutes++;
            else if (string.Equals(role, "VISUAL_ONLY", StringComparison.Ordinal))
                _report.VisualOnlyRoutes++;
            else if (string.Equals(role, "SHADOW_EFFECT_EXCLUDE", StringComparison.Ordinal) ||
                string.Equals(role, "VISUAL_EFFECT_EXCLUDE", StringComparison.Ordinal))
                _report.ExcludedEffectRoutes++;
            else if (string.Equals(role, "CHILD_OBJECT_EXCLUDE", StringComparison.Ordinal))
                _report.ExcludedChildObjectRoutes++;
            else
                _report.UnknownRoutes++;
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

            if (!cancelled)
            {
                _report.ObservationVertices = ObservationPositions.Count;
                _report.ObservationIndices = ObservationIndices.Count;
                _report.ObservationTriangles = ObservationIndices.Count / 3;
                _report.CollisionVertices = CollisionPositions.Count;
                _report.CollisionIndices = CollisionIndices.Count;
                _report.CollisionTriangles = CollisionIndices.Count / 3;
                _report.ObservationWorldBounds = ObservationPositions.Count > 0
                    ? FormatBounds(CalculateBounds(ObservationPositions))
                    : "<none>";
                _report.CollisionWorldBounds = CollisionPositions.Count > 0
                    ? FormatBounds(CalculateBounds(CollisionPositions))
                    : "<none>";
                _report.ObservationRawTopology = AnalyzeTopology(
                    ObservationPositions,
                    ObservationIndices,
                    0f,
                    false);
                _report.ObservationWeldedTopology = AnalyzeTopology(
                    ObservationPositions,
                    ObservationIndices,
                    ComputeWeldTolerance(ObservationPositions),
                    true);
                _report.CollisionRawTopology = AnalyzeTopology(
                    CollisionPositions,
                    CollisionIndices,
                    0f,
                    false);
                _report.CollisionWeldedTopology = AnalyzeTopology(
                    CollisionPositions,
                    CollisionIndices,
                    ComputeWeldTolerance(CollisionPositions),
                    true);
                _report.CollisionSpatialInstances = SpatialInstanceAnalyzer.Analyze(
                    "collision",
                    _report.TargetPath,
                    CollisionPositions,
                    CollisionIndices,
                    CollisionTriangleRouteIds,
                    _report.Parts,
                    ComputeWeldTolerance(CollisionPositions));
                _report.ObservationSpatialInstances = SpatialInstanceAnalyzer.Analyze(
                    "observation",
                    _report.TargetPath,
                    ObservationPositions,
                    ObservationIndices,
                    ObservationTriangleRouteIds,
                    _report.Parts,
                    ComputeWeldTolerance(ObservationPositions));
                SpatialInstanceAnalyzer.AssociateObservationToCollision(
                    _report.ObservationSpatialInstances,
                    _report.CollisionSpatialInstances);
                SpatialInstanceAnalyzer.ApplyInstanceProfiles(
                    _report.CollisionSpatialInstances,
                    _report.TargetPath,
                    _report.TargetName);
                SpatialInstanceAnalyzer.ApplyInstanceProfiles(
                    _report.ObservationSpatialInstances,
                    _report.TargetPath,
                    _report.TargetName);

                LogSpatialAudit(_report.CollisionSpatialInstances, true);
                LogSpatialAudit(_report.ObservationSpatialInstances, false);

                bool requiredFailed = false;
                for (int i = 0; i < _report.Parts.Count; i++)
                {
                    MultipartPartReport part = _report.Parts[i];
                    if (part != null &&
                        (part.IncludeObservation || part.IncludeCollision) &&
                        (!part.ReadbackSuccess || !part.AggregationAccepted))
                    {
                        requiredFailed = true;
                        break;
                    }
                }
                _report.Success = !requiredFailed && _report.RoutesProcessed == _report.RoutesQueued;

                if (_report.Success &&
                    CollisionPositions.Count > 0 &&
                    CollisionIndices.Count >= 3)
                {
                    CompletedCollisionPositions = CollisionPositions.ToArray();
                    CompletedCollisionIndices = CollisionIndices.ToArray();
                    CompletedCollisionBounds = CalculateBounds(CollisionPositions);
                    CompletedCollisionTargetName = _report.TargetName;
                    CompletedCollisionTargetPath = _report.TargetPath;
                    CompletedCollisionSnapshotReady = true;
                    _logger?.LogInfo(
                        "[GSP][MULTIPART] collision-geometry-snapshot-ready" +
                        " | target=" + _report.TargetName +
                        " | vertices=" + CompletedCollisionPositions.Length +
                        " | indices=" + CompletedCollisionIndices.Length +
                        " | triangles=" + (CompletedCollisionIndices.Length / 3));
                }
                else
                {
                    ClearCompletedCollisionGeometry();
                }
            }
            else
            {
                ClearCompletedCollisionGeometry();
            }
            _logger?.LogWarning(
                "[GSP][MULTIPART] final | " + _report.ToSummaryText());
            _logger?.LogInfo(
                "[GSP][MULTIPART] role-meaning" +
                " | PHYSICAL_CORE/DETAIL=eligible for collision aggregate candidate, not yet dynamic-ready" +
                " | VISUAL_ONLY=kept for observation but excluded from collision" +
                " | SHADOW_EFFECT_EXCLUDE/VISUAL_EFFECT_EXCLUDE=not read or aggregated by default" +
                " | CHILD_OBJECT_EXCLUDE=independent child object omitted from current parent aggregation but still available as a future scan target" +
                " | coordinateNormalization=raw/world-baked-or-renderer-localToWorld selected per route and gated by Renderer world bounds" +
                " | weldedTopology=audit-copy-only; original render geometry is never modified" +
                " | spatialInstance=connected components are clustered by adaptive world-space AABB gap; this is a review candidate, not semantic truth" +
                " | instanceKey=deterministic draft identifier based on target path, channel, quantized bounds and source routes" +
                " | proxyDraft=PCA/compound audit only; STATIC_NONCONVEX_MESH uses the saved collision aggregate snapshot" +
                " | instanceProfile=exact instanceKey mass/friction/fixed/pickable/proxy metadata only" +
                " | mass/friction=configuration metadata; Rigidbody/PhysicMaterial creation is handled only by explicit later-stage modules");

            Pending.Clear();
            ClearWorkingGeometry();
            _target = null;
        }

        private static void EvaluateAutomaticRole(
            MeshRenderer renderer,
            string materialName,
            string shaderName,
            out string role,
            out bool includeObservation,
            out bool includeCollision,
            out string reason)
        {
            string rendererName = renderer != null ? renderer.name : string.Empty;
            // 不把整个对象根路径放进自动部件分类，否则对象名称中的语义词可能
            // 把主体、视觉覆盖层和特效误判为同一类。自动判断只看当前 Renderer、材质和 Shader；
            // 具体对象路径交给可编辑白名单处理。
            string combined = (
                rendererName + " " + materialName + " " + shaderName)
                .ToLowerInvariant();

            ShadowCastingMode shadowMode = Safe(
                () => renderer.shadowCastingMode,
                ShadowCastingMode.On);
            if (shadowMode == ShadowCastingMode.ShadowsOnly ||
                ContainsAny(combined, "shadow", "blob", "projector"))
            {
                role = "SHADOW_EFFECT_EXCLUDE";
                includeObservation = false;
                includeCollision = false;
                reason = "shadow/effect signature";
                return;
            }

            if (ContainsAny(
                combined,
                "glow",
                "effect",
                "_fx",
                " fx",
                "decal",
                "aura",
                "beam",
                "lightcone",
                "particle"))
            {
                role = "VISUAL_EFFECT_EXCLUDE";
                includeObservation = false;
                includeCollision = false;
                reason = "visual-effect signature";
                return;
            }

            if (ContainsAny(
                combined,
                "picture",
                "image",
                "photo",
                "screen",
                "display",
                "canvas",
                "quadstreet",
                "poster"))
            {
                role = "VISUAL_ONLY";
                includeObservation = true;
                includeCollision = false;
                reason = "flat visual content signature";
                return;
            }

            bool transparent = false;
            try
            {
                var materials = renderer.sharedMaterials;
                if (materials != null)
                {
                    for (int i = 0; i < materials.Length; i++)
                    {
                        Material material = materials[i];
                        if (material != null && material.renderQueue >= 3000)
                        {
                            transparent = true;
                            break;
                        }
                    }
                }
            }
            catch { }

            if (transparent && ContainsAny(combined, "glass", "lens", "window"))
            {
                role = "VISUAL_ONLY";
                includeObservation = true;
                includeCollision = false;
                reason = "transparent glass/lens route; collision requires explicit whitelist or proxy";
                return;
            }

            role = "PHYSICAL_CORE";
            includeObservation = true;
            includeCollision = true;
            reason = "default solid-renderer candidate; whitelist may downgrade or exclude";
        }

        private static void NormalizeRouteToWorld(
            MeshRenderer renderer,
            D3D11SubmeshGeometry geometry,
            MultipartPartReport part)
        {
            part.AggregationAccepted = false;
            part.ResolvedCoordinateMode = "rejected";
            part.CoordinateReason = "coordinate-audit-not-run";
            if (renderer == null || geometry == null ||
                geometry.RawPositions == null || geometry.RawPositions.Length == 0 ||
                geometry.LocalIndices == null || geometry.LocalIndices.Length == 0)
            {
                part.CoordinateReason = "missing renderer/raw positions/local indices";
                return;
            }

            Vector3[] raw = geometry.RawPositions;
            Vector3[] transformed = new Vector3[raw.Length];
            for (int i = 0; i < raw.Length; i++)
                transformed[i] = renderer.transform.TransformPoint(raw[i]);

            Bounds rendererBounds = renderer.bounds;
            float tolerance = ComputeRouteBoundsTolerance(rendererBounds);
            bool rawInside = ReferencedVerticesInsideBounds(
                raw, geometry.LocalIndices, rendererBounds, tolerance);
            bool transformedInside = ReferencedVerticesInsideBounds(
                transformed, geometry.LocalIndices, rendererBounds, tolerance);
            Bounds rawBounds = CalculateReferencedBounds(raw, geometry.LocalIndices);
            Bounds transformedBounds = CalculateReferencedBounds(transformed, geometry.LocalIndices);

            part.RawReferencedInsideRendererBounds = rawInside;
            part.TransformedReferencedInsideRendererBounds = transformedInside;
            part.CoordinateAmbiguous = rawInside && transformedInside;
            part.RendererBounds = FormatBounds(rendererBounds);
            part.RawBounds = FormatBounds(rawBounds);
            part.TransformedBounds = FormatBounds(transformedBounds);

            Vector3[] selected = null;
            bool transformApplied = false;
            string mode = "rejected";
            string reason;

            if (rawInside && !transformedInside)
            {
                selected = raw;
                mode = "raw/world-baked";
                reason = "only raw referenced vertices fit Renderer world bounds";
            }
            else if (!rawInside && transformedInside)
            {
                selected = transformed;
                transformApplied = true;
                mode = "renderer-local-to-world";
                reason = "only localToWorld-transformed referenced vertices fit Renderer world bounds";
            }
            else if (rawInside && transformedInside)
            {
                bool staticBatch = Safe(() => renderer.isPartOfStaticBatch, false);
                float rawScore = CandidateBoundsScore(rawBounds, rendererBounds);
                float transformedScore = CandidateBoundsScore(transformedBounds, rendererBounds);
                if (staticBatch && rawScore <= transformedScore + 0.25f)
                {
                    selected = raw;
                    mode = "raw/world-baked-ambiguous";
                    reason = "both candidates fit; static-batch preference selected raw; rawScore=" +
                        rawScore.ToString("G6") + ",transformedScore=" + transformedScore.ToString("G6");
                }
                else if (!staticBatch && transformedScore <= rawScore + 0.25f)
                {
                    selected = transformed;
                    transformApplied = true;
                    mode = "renderer-local-to-world-ambiguous";
                    reason = "both candidates fit; non-static Renderer preference selected localToWorld; rawScore=" +
                        rawScore.ToString("G6") + ",transformedScore=" + transformedScore.ToString("G6");
                }
                else if (rawScore <= transformedScore)
                {
                    selected = raw;
                    mode = "raw/world-baked-ambiguous-score";
                    reason = "both candidates fit; lower bounds score selected raw; rawScore=" +
                        rawScore.ToString("G6") + ",transformedScore=" + transformedScore.ToString("G6");
                }
                else
                {
                    selected = transformed;
                    transformApplied = true;
                    mode = "renderer-local-to-world-ambiguous-score";
                    reason = "both candidates fit; lower bounds score selected localToWorld; rawScore=" +
                        rawScore.ToString("G6") + ",transformedScore=" + transformedScore.ToString("G6");
                }
            }
            else
            {
                reason = "neither raw nor localToWorld-transformed referenced vertices fit Renderer world bounds";
            }

            if (selected == null)
            {
                part.CoordinateReason = reason;
                part.WorldBounds = "<rejected>";
                part.Bounds = part.WorldBounds;
                return;
            }

            Bounds worldBounds = CalculateReferencedBounds(selected, geometry.LocalIndices);
            bool accepted = ReferencedVerticesInsideBounds(
                selected, geometry.LocalIndices, rendererBounds, tolerance);
            part.TransformApplied = transformApplied;
            part.ResolvedCoordinateMode = mode;
            part.WorldBounds = FormatBounds(worldBounds);
            part.Bounds = part.WorldBounds;
            part.AggregationAccepted = accepted;
            part.CoordinateReason = reason +
                "; tolerance=" + tolerance.ToString("G9") +
                "; accepted=" + accepted;

            if (accepted)
            {
                geometry.WorldPositions = selected;
                geometry.WorldBounds = worldBounds;
                geometry.CoordinateMode = mode;
            }
        }

        private static bool ReferencedVerticesInsideBounds(
            Vector3[] positions,
            int[] indices,
            Bounds bounds,
            float tolerance)
        {
            if (positions == null || indices == null || indices.Length == 0)
                return false;
            Vector3 min = bounds.min - Vector3.one * tolerance;
            Vector3 max = bounds.max + Vector3.one * tolerance;
            bool any = false;
            HashSet<int> visited = new HashSet<int>();
            for (int i = 0; i < indices.Length; i++)
            {
                int index = indices[i];
                if (index < 0 || index >= positions.Length || !visited.Add(index))
                    continue;
                any = true;
                Vector3 p = positions[index];
                if (!IsFinite(p) ||
                    p.x < min.x || p.y < min.y || p.z < min.z ||
                    p.x > max.x || p.y > max.y || p.z > max.z)
                    return false;
            }
            return any;
        }

        private static float ComputeRouteBoundsTolerance(Bounds bounds)
        {
            float scale = Mathf.Max(bounds.size.x, Mathf.Max(bounds.size.y, bounds.size.z));
            return Mathf.Max(1e-5f, scale * 1e-4f);
        }

        private static float CandidateBoundsScore(Bounds candidate, Bounds reference)
        {
            float scale = Mathf.Max(1e-5f,
                Mathf.Max(reference.size.x, Mathf.Max(reference.size.y, reference.size.z)));
            float center = Vector3.Distance(candidate.center, reference.center) / scale;
            Vector3 delta = candidate.size - reference.size;
            float size = delta.magnitude / scale;
            return center + size * 0.25f;
        }

        private static Bounds CalculateReferencedBounds(
            Vector3[] positions,
            int[] indices)
        {
            if (positions == null || indices == null || indices.Length == 0)
                return new Bounds(Vector3.zero, Vector3.zero);
            bool have = false;
            Vector3 min = Vector3.zero;
            Vector3 max = Vector3.zero;
            HashSet<int> visited = new HashSet<int>();
            for (int i = 0; i < indices.Length; i++)
            {
                int index = indices[i];
                if (index < 0 || index >= positions.Length || !visited.Add(index))
                    continue;
                Vector3 value = positions[index];
                if (!have)
                {
                    min = max = value;
                    have = true;
                }
                else
                {
                    min = Vector3.Min(min, value);
                    max = Vector3.Max(max, value);
                }
            }
            if (!have)
                return new Bounds(Vector3.zero, Vector3.zero);
            Bounds bounds = new Bounds();
            bounds.SetMinMax(min, max);
            return bounds;
        }

        private static Bounds CalculateBounds(List<Vector3> positions)
        {
            if (positions == null || positions.Count == 0)
                return new Bounds(Vector3.zero, Vector3.zero);
            Vector3 min = positions[0];
            Vector3 max = positions[0];
            for (int i = 1; i < positions.Count; i++)
            {
                min = Vector3.Min(min, positions[i]);
                max = Vector3.Max(max, positions[i]);
            }
            Bounds bounds = new Bounds();
            bounds.SetMinMax(min, max);
            return bounds;
        }

        private static string CalculateRendererAggregateBounds(MeshRenderer[] renderers)
        {
            if (renderers == null || renderers.Length == 0)
                return "<none>";
            bool have = false;
            Bounds aggregate = new Bounds();
            for (int i = 0; i < renderers.Length; i++)
            {
                MeshRenderer renderer = renderers[i];
                if (renderer == null)
                    continue;
                Bounds bounds;
                try { bounds = renderer.bounds; }
                catch { continue; }
                if (!have)
                {
                    aggregate = bounds;
                    have = true;
                }
                else
                {
                    aggregate.Encapsulate(bounds);
                }
            }
            return have ? FormatBounds(aggregate) : "<none>";
        }

        private static Bounds CalculateBounds(Vector3[] positions)
        {
            if (positions == null || positions.Length == 0)
                return new Bounds(Vector3.zero, Vector3.zero);
            Vector3 min = positions[0];
            Vector3 max = positions[0];
            for (int i = 1; i < positions.Length; i++)
            {
                min = Vector3.Min(min, positions[i]);
                max = Vector3.Max(max, positions[i]);
            }
            Bounds bounds = new Bounds();
            bounds.SetMinMax(min, max);
            return bounds;
        }

        private static string FormatBounds(Bounds bounds)
        {
            return "center=" + bounds.center.ToString("G9") +
                "/size=" + bounds.size.ToString("G9") +
                "/min=" + bounds.min.ToString("G9") +
                "/max=" + bounds.max.ToString("G9");
        }

        private static bool IsFinite(Vector3 value)
        {
            return !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                !float.IsNaN(value.z) && !float.IsInfinity(value.z);
        }

        private static void AppendGeometry(
            D3D11SubmeshGeometry geometry,
            List<Vector3> positions,
            List<int> indices,
            List<int> triangleRouteIds,
            int routeSequence)
        {
            if (geometry == null ||
                geometry.WorldPositions == null ||
                geometry.LocalIndices == null)
                return;

            // 聚合时只复制真正被该路由索引引用的顶点，并按完整三角形追加。
            // triangleRouteIds 与聚合三角形一一对应，用于后续连通组件和空间实例追溯来源。
            int[] remap = new int[geometry.WorldPositions.Length];
            for (int i = 0; i < remap.Length; i++)
                remap[i] = -1;

            int triangleCount = geometry.LocalIndices.Length / 3;
            for (int triangle = 0; triangle < triangleCount; triangle++)
            {
                int sourceA = geometry.LocalIndices[triangle * 3];
                int sourceB = geometry.LocalIndices[triangle * 3 + 1];
                int sourceC = geometry.LocalIndices[triangle * 3 + 2];
                if (sourceA < 0 || sourceB < 0 || sourceC < 0 ||
                    sourceA >= geometry.WorldPositions.Length ||
                    sourceB >= geometry.WorldPositions.Length ||
                    sourceC >= geometry.WorldPositions.Length)
                    continue;

                int a = GetOrAppendVertex(sourceA, geometry.WorldPositions, remap, positions);
                int b = GetOrAppendVertex(sourceB, geometry.WorldPositions, remap, positions);
                int c = GetOrAppendVertex(sourceC, geometry.WorldPositions, remap, positions);
                indices.Add(a);
                indices.Add(b);
                indices.Add(c);
                triangleRouteIds.Add(routeSequence);
            }
        }

        private static int GetOrAppendVertex(
            int sourceIndex,
            Vector3[] worldPositions,
            int[] remap,
            List<Vector3> aggregatePositions)
        {
            int aggregateIndex = remap[sourceIndex];
            if (aggregateIndex >= 0)
                return aggregateIndex;
            aggregateIndex = aggregatePositions.Count;
            remap[sourceIndex] = aggregateIndex;
            aggregatePositions.Add(worldPositions[sourceIndex]);
            return aggregateIndex;
        }

        private static GeometryTopologyAudit AnalyzeTopology(
            List<Vector3> positions,
            List<int> indices,
            float weldTolerance,
            bool weld)
        {
            GeometryTopologyAudit audit = new GeometryTopologyAudit
            {
                Vertices = positions != null ? positions.Count : 0,
                Indices = indices != null ? indices.Count : 0,
                Triangles = indices != null ? indices.Count / 3 : 0,
                WeldTolerance = weld ? weldTolerance : 0f
            };
            if (positions == null || indices == null ||
                positions.Count == 0 || indices.Count < 3)
                return audit;

            int[] mapped = null;
            int vertexCount = positions.Count;
            if (weld)
            {
                mapped = BuildWeldMap(positions, weldTolerance, out vertexCount);
                audit.WeldedVertices = vertexCount;
            }
            else
            {
                audit.WeldedVertices = positions.Count;
            }

            int triangleCount = indices.Count / 3;
            UnionFind union = new UnionFind(triangleCount);
            bool[] validTriangles = new bool[triangleCount];
            Dictionary<ulong, EdgeInfo> edges =
                new Dictionary<ulong, EdgeInfo>(Math.Max(16, triangleCount * 2));
            Dictionary<string, int> triangleKeys =
                new Dictionary<string, int>(Math.Max(16, triangleCount));

            for (int triangle = 0; triangle < triangleCount; triangle++)
            {
                int a = indices[triangle * 3];
                int b = indices[triangle * 3 + 1];
                int c = indices[triangle * 3 + 2];
                if (a < 0 || b < 0 || c < 0 ||
                    a >= positions.Count || b >= positions.Count || c >= positions.Count)
                    continue;
                if (mapped != null)
                {
                    a = mapped[a];
                    b = mapped[b];
                    c = mapped[c];
                }
                if (a == b || b == c || c == a)
                    continue;
                validTriangles[triangle] = true;

                int s0 = a;
                int s1 = b;
                int s2 = c;
                Sort3(ref s0, ref s1, ref s2);
                string triangleKey = s0 + ":" + s1 + ":" + s2;
                if (triangleKeys.ContainsKey(triangleKey))
                    audit.DuplicateTriangles++;
                else
                    triangleKeys.Add(triangleKey, triangle);

                AddEdge(edges, union, a, b, triangle);
                AddEdge(edges, union, b, c, triangle);
                AddEdge(edges, union, c, a, triangle);
            }

            Dictionary<int, List<int>> boundaryAdjacency =
                new Dictionary<int, List<int>>();
            foreach (KeyValuePair<ulong, EdgeInfo> pair in edges)
            {
                EdgeInfo edge = pair.Value;
                if (edge.Count == 1)
                {
                    audit.BoundaryEdges++;
                    DecodeEdge(pair.Key, out int a, out int b);
                    AddAdjacency(boundaryAdjacency, a, b);
                    AddAdjacency(boundaryAdjacency, b, a);
                }
                else if (edge.Count > 2)
                {
                    audit.NonManifoldEdges++;
                }
                if (edge.Count == 2 && edge.ForwardCount != 1)
                    audit.OrientationConflicts++;
            }

            CountBoundaryComponents(
                boundaryAdjacency,
                out audit.BoundaryLoops,
                out audit.OpenBoundaryChains);

            HashSet<int> roots = new HashSet<int>();
            for (int triangle = 0; triangle < triangleCount; triangle++)
            {
                if (validTriangles[triangle])
                    roots.Add(union.Find(triangle));
            }
            audit.ConnectedComponents = roots.Count;
            audit.Watertight =
                audit.BoundaryEdges == 0 &&
                audit.NonManifoldEdges == 0 &&
                audit.OrientationConflicts == 0 &&
                audit.ConnectedComponents == 1;
            audit.Analyzed = true;
            return audit;
        }

        private static int[] BuildWeldMap(
            List<Vector3> positions,
            float tolerance,
            out int canonicalCount)
        {
            tolerance = Mathf.Max(tolerance, 0.000001f);
            float inverse = 1f / tolerance;
            int[] map = new int[positions.Count];
            List<Vector3> canonical = new List<Vector3>();
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
                            QuantizedKey neighbor = new QuantizedKey(qx + dx, qy + dy, qz + dz);
                            if (!cells.TryGetValue(neighbor, out List<int> candidates))
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

        private static float ComputeWeldTolerance(List<Vector3> positions)
        {
            if (positions == null || positions.Count == 0)
                return 0.00001f;
            Vector3 min = positions[0];
            Vector3 max = positions[0];
            for (int i = 1; i < positions.Count; i++)
            {
                min = Vector3.Min(min, positions[i]);
                max = Vector3.Max(max, positions[i]);
            }
            Vector3 size = max - min;
            float scale = Mathf.Max(size.x, Mathf.Max(size.y, size.z));
            return Mathf.Max(0.00001f, scale * 0.00002f);
        }

        private static void AddEdge(
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
                info = new EdgeInfo
                {
                    FirstTriangle = triangle
                };
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

        private static string GetMaterialName(MeshRenderer renderer, int localRoute)
        {
            try
            {
                var materials = renderer.sharedMaterials;
                if (materials != null && localRoute >= 0 && localRoute < materials.Length)
                {
                    Material material = materials[localRoute];
                    return material != null ? material.name : "<null-material>";
                }
            }
            catch { }
            return "<no-material>";
        }

        private static string GetShaderName(MeshRenderer renderer, int localRoute)
        {
            try
            {
                var materials = renderer.sharedMaterials;
                if (materials != null && localRoute >= 0 && localRoute < materials.Length)
                {
                    Material material = materials[localRoute];
                    Shader shader = material != null ? material.shader : null;
                    return shader != null ? shader.name : "<no-shader>";
                }
            }
            catch { }
            return "<no-shader>";
        }

        private static bool ContainsAny(string text, params string[] values)
        {
            if (string.IsNullOrEmpty(text) || values == null)
                return false;
            for (int i = 0; i < values.Length; i++)
            {
                if (!string.IsNullOrEmpty(values[i]) &&
                    text.IndexOf(values[i], StringComparison.OrdinalIgnoreCase) >= 0)
                    return true;
            }
            return false;
        }

        private static void Sort3(ref int a, ref int b, ref int c)
        {
            if (a > b) { int t = a; a = b; b = t; }
            if (b > c) { int t = b; b = c; c = t; }
            if (a > b) { int t = a; a = b; b = t; }
        }

        private static void ClearCompletedCollisionGeometry()
        {
            CompletedCollisionPositions = null;
            CompletedCollisionIndices = null;
            CompletedCollisionBounds = default;
            CompletedCollisionTargetName = null;
            CompletedCollisionTargetPath = null;
            CompletedCollisionSnapshotReady = false;
        }

        private static void ClearWorkingGeometry()
        {
            ObservationPositions.Clear();
            ObservationIndices.Clear();
            CollisionPositions.Clear();
            CollisionIndices.Clear();
            ObservationTriangleRouteIds.Clear();
            CollisionTriangleRouteIds.Clear();
        }

        private static float ElapsedSeconds()
        {
            try
            {
                return _stopwatch != null
                    ? (float)_stopwatch.Elapsed.TotalSeconds
                    : 0f;
            }
            catch { return 0f; }
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
                return target.GetComponentsInChildren<T>(true) ?? Array.Empty<T>();
            }
            catch
            {
                return Array.Empty<T>();
            }
        }
    }
}
