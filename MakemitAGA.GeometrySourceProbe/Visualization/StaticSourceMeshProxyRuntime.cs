using System;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Acquisition;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    /// <summary>
    /// 把最近一次成功多部件审计的碰撞聚合世界空间几何，创建成 GSP 自己拥有的
    /// 半透明精确 Mesh 预览与静态非凸 MeshCollider。
    ///
    /// 安全边界：
    /// - 只接受显式 enabled=true、proxyType=STATIC_NONCONVEX_MESH、
    ///   fixedToWorld=true、pickable=false 的唯一 collision 实例；
    /// - 只消费 MultipartObjectAggregator 保存的托管几何快照，不重新读取源 Mesh/GPU；
    /// - MeshCollider 始终 convex=false、isTrigger=false、无 Rigidbody；
    /// - Collider 创建并完成 cooking/bounds 审计后立即关闭，必须显式开启；
    /// - 不修改或禁用源 Renderer、源 Collider、源 Transform；
    /// - rollback、场景切换与插件卸载只删除 GSP 自己创建的对象、Mesh 与 Material。
    /// </summary>
    internal static class StaticSourceMeshProxyRuntime
    {
        private static GameObject _root;
        private static GameObject _target;
        private static Mesh _mesh;
        private static Material _material;
        private static MeshRenderer _renderer;
        private static MeshCollider _collider;
        private static ManualLogSource _logger;
        private static StaticSourceMeshProxyRuntimeReport _report;
        private static bool _visualVisible;
        private static bool _colliderEnabled;
        private static Vector3[] _localVertices;
        private static int[] _localIndices;
        private static Vector3 _expectedRootPosition;
        private static Quaternion _expectedRootRotation;
        private static Vector3 _expectedRootScale;
        private static Bounds _expectedWorldBounds;
        private static int _expectedLayer = -1;
        private static string _lastCleanupError;

        public static StaticSourceMeshProxyRuntimeReport CurrentReport => _report;
        public static string LastCleanupError => _lastCleanupError;
        public static bool Exists => _root != null && _mesh != null && _collider != null;
        public static GameObject RuntimeTarget => _target;
        public static MeshCollider RuntimeCollider => _collider;
        public static Bounds RuntimeBounds => _collider != null ? _collider.bounds : default;
        public static int RuntimeLayer => _expectedLayer;
        public static bool ColliderEnabled => _colliderEnabled;

        public static string Build(
            MultipartAggregationReport source,
            GameObject target,
            ManualLogSource logger)
        {
            _logger = logger;
            Clear("rebuild-before-create", false);
            string priorCleanupError = _lastCleanupError;

            _report = new StaticSourceMeshProxyRuntimeReport
            {
                Started = true,
                TargetName = source != null ? source.TargetName : null,
                TargetPath = source != null ? source.TargetPath : null,
                Detail = "validating-static-source-mesh-candidate"
            };

            if (!string.IsNullOrEmpty(priorCleanupError))
                return Fail("前一份 static-mesh provider 清理未完成，拒绝重建：" + priorCleanupError);

            if (source == null || !source.Completed || !source.Success)
                return Fail("当前没有已完成且成功的实例审计。请先执行 gsp_instance_audit(对象名称)。");
            if (!ValidateBoundTarget(source, target, out string targetError))
                return Fail(targetError);

            SpatialInstanceAuditReport collision = source.CollisionSpatialInstances;
            if (collision == null || !collision.Analyzed || collision.Instances.Count != 1)
                return Fail("STATIC_NONCONVEX_MESH 当前只接受恰好一个碰撞现实实例；当前=" +
                    (collision != null ? collision.Instances.Count : 0) + "。");

            SpatialInstanceReport instance = collision.Instances[0];
            SpatialInstancePhysicsProfileSnapshot profile =
                instance != null ? instance.PhysicsProfile : null;
            _report.CollisionInstances = collision.Instances.Count;
            _report.ProfileMatched = profile != null && profile.Matched;
            _report.ProfileProxyType = profile != null && profile.ProxyTypeConfigured
                ? profile.ProxyType
                : null;
            _report.FixedToWorld = profile != null &&
                profile.FixedToWorldConfigured && profile.FixedToWorld;
            _report.Pickable = profile != null &&
                profile.PickableConfigured && profile.Pickable;

            if (profile == null || !profile.Matched)
                return Fail("碰撞实例没有匹配实例级物理配置。");
            if (!profile.EnabledConfigured || !profile.Enabled)
                return Fail("实例配置没有显式 enabled=true。");
            if (!profile.ProxyTypeConfigured ||
                !string.Equals(profile.ProxyType, "STATIC_NONCONVEX_MESH", StringComparison.OrdinalIgnoreCase))
                return Fail("实例配置 proxyType 必须为 STATIC_NONCONVEX_MESH；当前=" +
                    (profile.ProxyTypeConfigured ? profile.ProxyType : "<unset>") + "。");
            if (!profile.FixedToWorldConfigured || !profile.FixedToWorld)
                return Fail("静态非凸源网格只接受 fixedToWorld=true。");
            if (!profile.PickableConfigured || profile.Pickable)
                return Fail("静态非凸源网格只接受 pickable=false。");

            if (source.CollisionWeldedTopology == null ||
                !source.CollisionWeldedTopology.Analyzed)
                return Fail("碰撞焊接拓扑报告不存在。");

            _report.RawBoundaryEdges = source.CollisionRawTopology != null
                ? source.CollisionRawTopology.BoundaryEdges
                : 0;
            _report.WeldedBoundaryEdges = source.CollisionWeldedTopology.BoundaryEdges;
            _report.WeldedComponents = source.CollisionWeldedTopology.ConnectedComponents;
            _report.NonManifoldEdges = source.CollisionWeldedTopology.NonManifoldEdges;
            _report.OrientationConflicts = source.CollisionWeldedTopology.OrientationConflicts;
            _report.DuplicateTriangles = source.CollisionWeldedTopology.DuplicateTriangles;

            // 开放边界对无 Rigidbody 的静态三角网格不是自动拒绝条件；
            // 非流形、方向冲突和重复三角形仍作为严格安全门。
            if (_report.NonManifoldEdges != 0 ||
                _report.OrientationConflicts != 0 ||
                _report.DuplicateTriangles != 0)
            {
                return Fail("焊接拓扑安全门未通过：nonManifold=" +
                    _report.NonManifoldEdges + "，orientationConflict=" +
                    _report.OrientationConflicts + "，duplicates=" +
                    _report.DuplicateTriangles + "。");
            }

            if (!MultipartObjectAggregator.TryCopyCompletedCollisionGeometry(
                out Vector3[] worldVertices,
                out int[] indices,
                out Bounds worldBounds,
                out string snapshotTargetName,
                out string snapshotTargetPath,
                out string snapshotError))
            {
                return Fail("无法取得完成审计的碰撞几何快照：" + snapshotError);
            }

            if (!string.Equals(snapshotTargetPath, source.TargetPath, StringComparison.Ordinal) ||
                !string.Equals(snapshotTargetName, source.TargetName, StringComparison.Ordinal))
            {
                return Fail("碰撞几何快照与当前报告目标不一致，拒绝跨对象创建。");
            }

            _report.VertexCount = worldVertices != null ? worldVertices.Length : 0;
            _report.IndexCount = indices != null ? indices.Length : 0;
            _report.TriangleCount = _report.IndexCount / 3;
            _report.GeometryBounds = FormatBounds(worldBounds);

            if (!ValidateGeometry(worldVertices, indices, out string validationError))
                return Fail(validationError);

            try
            {
                Vector3 center = worldBounds.center;
                Vector3[] localVertices = new Vector3[worldVertices.Length];
                for (int i = 0; i < worldVertices.Length; i++)
                    localVertices[i] = worldVertices[i] - center;
                _localVertices = (Vector3[])localVertices.Clone();
                _localIndices = (int[])indices.Clone();
                _target = target;
                _expectedRootPosition = center;
                _expectedRootRotation = Quaternion.identity;
                _expectedRootScale = Vector3.one;
                _expectedWorldBounds = worldBounds;
                _expectedLayer = target.layer;

                _mesh = new Mesh
                {
                    name = "GSP_StaticSourceMesh_" + Sanitize(source.TargetName),
                    indexFormat = localVertices.Length >= 65000
                        ? IndexFormat.UInt32
                        : IndexFormat.UInt16
                };
                _report.MeshName = _mesh.name;
                _report.IndexFormat = _mesh.indexFormat.ToString();
                _mesh.vertices = localVertices;
                _mesh.subMeshCount = 1;
                _mesh.SetIndices(indices, MeshTopology.Triangles, 0, false);
                _mesh.RecalculateBounds();
                _mesh.RecalculateNormals();
                _report.MeshCreated = true;

                _material = CreateMaterial();
                if (_material == null)
                    throw new InvalidOperationException("cannot-create-static-source-mesh-material");

                _root = new GameObject(
                    "GSP_StaticSourceMeshRuntime_" + Sanitize(source.TargetName));
                _report.RootName = _root.name;
                _root.transform.position = _expectedRootPosition;
                _root.transform.rotation = _expectedRootRotation;
                _root.transform.localScale = _expectedRootScale;
                _root.layer = _expectedLayer;

                MeshFilter filter = _root.AddComponent<MeshFilter>();
                _report.MeshFilterCreated = filter != null;
                if (filter == null)
                    throw new InvalidOperationException("cannot-add-MeshFilter");
                filter.sharedMesh = _mesh;

                _renderer = _root.AddComponent<MeshRenderer>();
                _report.RendererCreated = _renderer != null;
                if (_renderer == null)
                    throw new InvalidOperationException("cannot-add-MeshRenderer");
                _renderer.sharedMaterial = _material;
                _renderer.shadowCastingMode = ShadowCastingMode.Off;
                _renderer.receiveShadows = false;
                _report.RendererBounds = FormatBounds(_renderer.bounds);
                _report.RendererBoundsMatch = BoundsApproximatelyEqual(
                    _renderer.bounds, worldBounds);

                _collider = _root.AddComponent<MeshCollider>();
                _report.ColliderCreated = _collider != null;
                if (_collider == null)
                    throw new InvalidOperationException("cannot-add-MeshCollider");

                // 先启用并赋值以触发 Unity 默认 cooking，再做 bounds/安全审计；
                // 审计成功后立即关闭，避免创建命令本身改变场景物理。
                _collider.enabled = true;
                _collider.convex = false;
                _collider.isTrigger = false;
                _collider.sharedMesh = null;
                _collider.sharedMesh = _mesh;
                Physics.SyncTransforms();

                _report.SharedMeshAssigned = _collider.sharedMesh == _mesh;
                _report.Convex = _collider.convex;
                _report.IsTrigger = _collider.isTrigger;
                _report.RigidbodyPresent = _collider.attachedRigidbody != null ||
                    _root.GetComponent<Rigidbody>() != null;
                _report.ColliderBounds = FormatBounds(_collider.bounds);
                _report.ColliderBoundsMatch = BoundsApproximatelyEqual(
                    _collider.bounds, worldBounds);

                _visualVisible = true;
                _colliderEnabled = false;
                _renderer.enabled = true;
                _collider.enabled = false;
                Physics.SyncTransforms();

                _report.VisualVisible = _visualVisible;
                _report.ColliderEnabled = _colliderEnabled;
                _report.Completed = true;
                _report.Success =
                    _report.MeshCreated &&
                    _report.MeshFilterCreated &&
                    _report.RendererCreated &&
                    _report.ColliderCreated &&
                    _report.SharedMeshAssigned &&
                    !_report.Convex &&
                    !_report.IsTrigger &&
                    !_report.RigidbodyPresent &&
                    _report.RendererBoundsMatch &&
                    _report.ColliderBoundsMatch;
                _report.Detail = _report.Success
                    ? "exact collision aggregate created as GSP-owned static non-convex MeshCollider; visual on; collider intentionally off; source object unchanged"
                    : "static source mesh objects were created but one or more cooking/bounds/safety checks failed";

                if (!_report.Success)
                    return Fail(_report.Detail);

                _logger?.LogWarning(
                    "[GSP][STATIC_SOURCE_MESH] build-complete | " +
                    _report.ToStatusText());
                return
                    "<color=#66ddff>静态源网格被动碰撞代理创建成功。</color> " +
                    "vertices=" + _report.VertexCount +
                    "，triangles=" + _report.TriangleCount +
                    "，convex=False，Rigidbody=0。半透明精确网格已显示，Collider 初始关闭。";
            }
            catch (Exception e)
            {
                return Fail(e.GetType().Name + " / " + e.Message);
            }
        }

        public static string GetStatusText()
        {
            return _report == null
                ? "当前没有静态源网格代理。先完成实例审计并执行 gsp_instance_static_mesh_build。"
                : _report.ToDetailedText();
        }

        public static string SetVisual(bool visible)
        {
            if (_root == null || _renderer == null)
                return "当前没有静态源网格代理。先执行 gsp_instance_static_mesh_build。";
            _visualVisible = visible;
            _renderer.enabled = visible;
            UpdateReportState();
            _logger?.LogInfo(
                "[GSP][STATIC_SOURCE_MESH] visual=" + visible);
            return "静态源网格半透明预览=" + visible + "。";
        }

        public static string SetCollider(bool enabled)
        {
            if (_root == null || _collider == null)
                return "当前没有静态源网格代理。先执行 gsp_instance_static_mesh_build。";

            if (!enabled)
            {
                try { _collider.enabled = false; }
                catch (Exception e)
                {
                    _colliderEnabled = false;
                    UpdateReportState();
                    return "<color=red>Collider 关闭异常：</color> " + e.GetType().Name + ":" + e.Message;
                }
                _colliderEnabled = false;
                Physics.SyncTransforms();
                UpdateReportState();
                return "静态非凸 MeshCollider=False；半透明预览状态不变。";
            }

            if (!ValidateEnablePreflight(out string error))
            {
                ForceDisableCollider();
                _logger?.LogWarning("[GSP][STATIC_SOURCE_MESH] enable-rejected | " + error);
                return "<color=red>拒绝启用静态源网格 Collider：</color> " + error + "；Collider 已尽力保持关闭。";
            }

            try
            {
                _collider.enabled = true;
                Physics.SyncTransforms();
                if (!_collider.enabled)
                    throw new InvalidOperationException("collider-enable-readback-failed");
                if (!BoundsApproximatelyEqual(_collider.bounds, _expectedWorldBounds))
                    throw new InvalidOperationException("collider-bounds-drift-after-enable");
            }
            catch (Exception e)
            {
                ForceDisableCollider();
                _logger?.LogWarning("[GSP][STATIC_SOURCE_MESH] enable-failed | " + e.GetType().Name + ":" + e.Message);
                return "<color=red>拒绝启用静态源网格 Collider：</color> " + e.GetType().Name + ":" + e.Message + "；Collider 已尽力保持关闭。";
            }

            if (!ValidateEnablePreflight(out error))
            {
                ForceDisableCollider();
                _logger?.LogWarning("[GSP][STATIC_SOURCE_MESH] post-enable-rejected | " + error);
                return "<color=red>Collider 启用后读回安全门失败：</color> " + error + "；已重新关闭。";
            }

            _colliderEnabled = true;
            UpdateReportState();
            _logger?.LogWarning(
                "[GSP][STATIC_SOURCE_MESH] collider-enabled=True" +
                " | vertices=" + (_report != null ? _report.VertexCount : 0) +
                " | triangles=" + (_report != null ? _report.TriangleCount : 0) +
                " | layer=" + _expectedLayer +
                " | rigidbodies=0" +
                " | secondEnablePreflight=PASS");
            return
                "<color=#66ddff>静态非凸 MeshCollider 已开启。</color> 二次启用安全门 PASS；convex=False，trigger=False，Rigidbody=0，源对象未修改。";
        }

        public static string Rollback()
        {
            bool existed = _root != null || _mesh != null || _collider != null;
            Clear("manual-rollback", true);
            if (!existed)
                return "当前没有需要回滚的静态源网格代理。";
            return string.IsNullOrEmpty(_lastCleanupError)
                ? "静态源网格代理已请求完整回滚：GSP-owned Collider、预览对象、Mesh 与材质均已进入清理。"
                : "<color=red>静态源网格代理回滚遇到异常：</color> " + _lastCleanupError;
        }

        public static void Clear(string reason)
        {
            Clear(reason, true);
        }

        private static bool ValidateEnablePreflight(out string error)
        {
            error = null;
            if (_root == null || _target == null || _mesh == null || _collider == null ||
                _renderer == null || _report == null || !_report.Success)
            {
                error = "provider-runtime-not-ready";
                return false;
            }
            if (!string.Equals(TargetFinder.GetPath(_target.transform), _report.TargetPath, StringComparison.Ordinal))
            {
                error = "target-identity-lost";
                return false;
            }
            if (_target.layer != _expectedLayer || _root.layer != _expectedLayer)
            {
                error = "layer-drift:expected=" + _expectedLayer + ",target=" + _target.layer + ",provider=" + _root.layer;
                return false;
            }
            if (!Approximately(_root.transform.position, _expectedRootPosition) ||
                Quaternion.Angle(_root.transform.rotation, _expectedRootRotation) > 0.05f ||
                !Approximately(_root.transform.localScale, _expectedRootScale))
            {
                error = "provider-transform-drift";
                return false;
            }
            if (_root.GetComponent<Rigidbody>() != null || _collider.attachedRigidbody != null)
            {
                error = "unexpected-Rigidbody-present";
                return false;
            }
            if (_collider.sharedMesh != _mesh)
            {
                error = "sharedMesh-drift";
                return false;
            }
            if (_collider.convex)
            {
                error = "convex-drift:true";
                return false;
            }
            if (_collider.isTrigger)
            {
                error = "trigger-drift:true";
                return false;
            }
            return true;
        }

        private static bool ValidateBoundTarget(
            MultipartAggregationReport source,
            GameObject target,
            out string error)
        {
            error = null;
            if (target == null)
            {
                error = "无法按审计 targetPath 唯一重绑定源对象。";
                return false;
            }
            string actualPath = TargetFinder.GetPath(target.transform);
            if (!string.Equals(actualPath, source.TargetPath, StringComparison.Ordinal))
            {
                error = "目标身份不一致：auditPath=" + source.TargetPath + "，runtimePath=" + actualPath;
                return false;
            }
            return true;
        }

        private static void ForceDisableCollider()
        {
            try
            {
                if (_collider != null)
                    _collider.enabled = false;
            }
            catch { }
            _colliderEnabled = false;
            try { Physics.SyncTransforms(); }
            catch { }
            UpdateReportState();
        }

        private static bool Approximately(Vector3 a, Vector3 b)
        {
            return Mathf.Abs(a.x - b.x) <= 0.0001f &&
                   Mathf.Abs(a.y - b.y) <= 0.0001f &&
                   Mathf.Abs(a.z - b.z) <= 0.0001f;
        }

        private static bool ValidateGeometry(
            Vector3[] vertices,
            int[] indices,
            out string error)
        {
            error = null;
            if (vertices == null || vertices.Length == 0)
            {
                error = "碰撞聚合顶点为空。";
                return false;
            }
            if (indices == null || indices.Length < 3 || indices.Length % 3 != 0)
            {
                error = "碰撞聚合索引为空或不是完整三角形列表。";
                return false;
            }

            for (int i = 0; i < vertices.Length; i++)
            {
                if (!IsFinite(vertices[i]))
                    _report.NonFiniteVertices++;
            }

            for (int i = 0; i + 2 < indices.Length; i += 3)
            {
                int a = indices[i];
                int b = indices[i + 1];
                int c = indices[i + 2];
                if (a < 0 || b < 0 || c < 0 ||
                    a >= vertices.Length || b >= vertices.Length || c >= vertices.Length)
                {
                    _report.OutOfRangeIndices++;
                    continue;
                }
                if (a == b || b == c || c == a)
                {
                    _report.RepeatedIndexTriangles++;
                    continue;
                }
                Vector3 cross = Vector3.Cross(vertices[b] - vertices[a], vertices[c] - vertices[a]);
                if (!IsFinite(cross) || cross.sqrMagnitude <= 1e-16f)
                    _report.ZeroAreaTriangles++;
            }

            if (_report.NonFiniteVertices != 0 ||
                _report.OutOfRangeIndices != 0 ||
                _report.RepeatedIndexTriangles != 0 ||
                _report.ZeroAreaTriangles != 0)
            {
                error =
                    "碰撞聚合几何严格验证未通过：nonFinite=" +
                    _report.NonFiniteVertices + "，outOfRange=" +
                    _report.OutOfRangeIndices + "，repeated=" +
                    _report.RepeatedIndexTriangles + "，zeroArea=" +
                    _report.ZeroAreaTriangles + "。";
                return false;
            }
            return true;
        }

        private static void UpdateReportState()
        {
            if (_report == null)
                return;
            _report.VisualVisible = _renderer != null && _renderer.enabled;
            _report.ColliderEnabled = _collider != null && _collider.enabled;
            _report.RigidbodyPresent =
                (_root != null && _root.GetComponent<Rigidbody>() != null) ||
                (_collider != null && _collider.attachedRigidbody != null);
        }

        private static string Fail(string message)
        {
            if (_report != null)
            {
                _report.Completed = true;
                _report.Success = false;
                _report.Detail = message;
            }
            Clear("build-failed", false);
            _logger?.LogWarning(
                "[GSP][STATIC_SOURCE_MESH] build-failed | " + message);
            return "<color=red>静态源网格代理创建失败：</color> " + message;
        }

        private static void Clear(string reason, bool discardReport)
        {
            string errors = null;
            try { if (_collider != null) _collider.enabled = false; }
            catch (Exception e) { errors = AppendCleanupError(errors, "colliderOff", e); }
            try { if (_renderer != null) _renderer.enabled = false; }
            catch (Exception e) { errors = AppendCleanupError(errors, "rendererOff", e); }
            try { if (_root != null) Object.Destroy(_root); }
            catch (Exception e) { errors = AppendCleanupError(errors, "destroyRoot", e); }
            try { if (_mesh != null) Object.Destroy(_mesh); }
            catch (Exception e) { errors = AppendCleanupError(errors, "destroyMesh", e); }
            try { if (_material != null) Object.Destroy(_material); }
            catch (Exception e) { errors = AppendCleanupError(errors, "destroyMaterial", e); }

            _root = null;
            _target = null;
            _mesh = null;
            _material = null;
            _renderer = null;
            _collider = null;
            _localVertices = null;
            _localIndices = null;
            _expectedRootPosition = Vector3.zero;
            _expectedRootRotation = Quaternion.identity;
            _expectedRootScale = Vector3.one;
            _expectedWorldBounds = new Bounds();
            _expectedLayer = -1;
            _visualVisible = false;
            _colliderEnabled = false;
            _lastCleanupError = errors;
            if (discardReport)
                _report = null;
            if (string.IsNullOrEmpty(errors))
                _logger?.LogInfo("[GSP][STATIC_SOURCE_MESH] cleared | reason=" + (reason ?? "<none>"));
            else
                _logger?.LogWarning("[GSP][STATIC_SOURCE_MESH] cleanup-error | reason=" + (reason ?? "<none>") + " | " + errors);
        }

        private static string AppendCleanupError(string current, string stage, Exception e)
        {
            string item = stage + ":" + e.GetType().Name + ":" + e.Message;
            return string.IsNullOrEmpty(current) ? item : current + ";" + item;
        }


        /// <summary>
        /// 从当前静态源网格缓存中查找离给定世界点最近的三角形，并返回朝向
        /// towardWorldPoint 一侧的世界空间法线。该结果是托管派生值，不冒充 Unity ContactPoint。
        /// </summary>
        public static bool TryDeriveNearestTriangleNormal(
            Vector3 worldPoint,
            Vector3 towardWorldPoint,
            out Vector3 worldNormal,
            out Vector3 nearestWorldPoint,
            out int triangleIndex,
            out float distance)
        {
            worldNormal = Vector3.zero;
            nearestWorldPoint = worldPoint;
            triangleIndex = -1;
            distance = float.PositiveInfinity;

            if (_root == null || _localVertices == null ||
                _localIndices == null || _localIndices.Length < 3)
                return false;

            try
            {
                Vector3 localPoint = _root.transform.InverseTransformPoint(worldPoint);
                float bestSqr = float.PositiveInfinity;
                Vector3 bestPoint = Vector3.zero;
                Vector3 bestNormal = Vector3.zero;
                int bestTriangle = -1;

                for (int i = 0; i + 2 < _localIndices.Length; i += 3)
                {
                    int ia = _localIndices[i];
                    int ib = _localIndices[i + 1];
                    int ic = _localIndices[i + 2];
                    if ((uint)ia >= (uint)_localVertices.Length ||
                        (uint)ib >= (uint)_localVertices.Length ||
                        (uint)ic >= (uint)_localVertices.Length)
                        continue;

                    Vector3 a = _localVertices[ia];
                    Vector3 b = _localVertices[ib];
                    Vector3 c = _localVertices[ic];
                    Vector3 cross = Vector3.Cross(b - a, c - a);
                    float crossSqr = cross.sqrMagnitude;
                    if (!IsFinite(crossSqr) || crossSqr <= 1e-16f)
                        continue;

                    Vector3 candidate = ClosestPointOnTriangle(localPoint, a, b, c);
                    float sqr = (candidate - localPoint).sqrMagnitude;
                    if (!IsFinite(sqr) || sqr >= bestSqr)
                        continue;

                    bestSqr = sqr;
                    bestPoint = candidate;
                    bestNormal = cross / Mathf.Sqrt(crossSqr);
                    bestTriangle = i / 3;
                }

                if (bestTriangle < 0 || !IsFinite(bestNormal))
                    return false;

                nearestWorldPoint = _root.transform.TransformPoint(bestPoint);
                worldNormal = _root.transform.TransformDirection(bestNormal).normalized;
                Vector3 toward = towardWorldPoint - nearestWorldPoint;
                if (toward.sqrMagnitude > 1e-12f && Vector3.Dot(worldNormal, toward) < 0f)
                    worldNormal = -worldNormal;

                triangleIndex = bestTriangle;
                distance = Mathf.Sqrt(bestSqr);
                return IsFinite(worldNormal) && worldNormal.sqrMagnitude > 0.5f;
            }
            catch
            {
                worldNormal = Vector3.zero;
                nearestWorldPoint = worldPoint;
                triangleIndex = -1;
                distance = float.PositiveInfinity;
                return false;
            }
        }

        private static Vector3 ClosestPointOnTriangle(
            Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ap = p - a;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f)
                return a;

            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3)
                return b;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                return a + v * ab;
            }

            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6)
                return c;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                return a + w * ac;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                Vector3 bc = c - b;
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return b + w * bc;
            }

            float denominator = 1f / (va + vb + vc);
            float faceV = vb * denominator;
            float faceW = vc * denominator;
            return a + ab * faceV + ac * faceW;
        }

        private static Material CreateMaterial()
        {
            Shader shader = Shader.Find("Legacy Shaders/Transparent/Diffuse");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                return null;

            Color color = new Color(0.10f, 1.00f, 0.55f, 0.38f);
            Material material = new Material(shader)
            {
                name = "GSP_StaticSourceMeshMaterial",
                color = color
            };
            try
            {
                if (material.HasProperty("_Color"))
                    material.SetColor("_Color", color);
                if (material.HasProperty("_ZWrite"))
                    material.SetInt("_ZWrite", 0);
                material.renderQueue = 3000;
            }
            catch { }
            return material;
        }

        private static bool BoundsApproximatelyEqual(Bounds a, Bounds b)
        {
            float scale = Mathf.Max(1f, Mathf.Max(a.size.magnitude, b.size.magnitude));
            float tolerance = Mathf.Max(0.0005f, scale * 0.0005f);
            return Vector3.Distance(a.center, b.center) <= tolerance &&
                Vector3.Distance(a.size, b.size) <= tolerance;
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        private static string FormatBounds(Bounds bounds)
        {
            return "center=" + bounds.center.ToString("G9") +
                "/size=" + bounds.size.ToString("G9");
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
                return "Unnamed";
            char[] chars = value.ToCharArray();
            for (int i = 0; i < chars.Length; i++)
            {
                char c = chars[i];
                if (!(char.IsLetterOrDigit(c) || c == '_' || c == '-'))
                    chars[i] = '_';
            }
            return new string(chars);
        }
    }
}
