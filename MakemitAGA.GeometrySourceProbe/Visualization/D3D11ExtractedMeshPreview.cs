using System;
using System.Collections.Generic;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    /// <summary>
    /// v0.1.22：把 D3D11 staging 读回的路由子网格创建为独立、可删除的 Unity Mesh 预览，并支持静态非凸 MeshCollider 与后续受控物理球测试。
    ///
    /// 重要边界：
    /// - 不修改源 Mesh、源材质或源 Transform；
    /// - 普通预览命令不创建 MeshCollider；只有显式 collider 审计命令才在副本上添加一个非凸静态 MeshCollider；
    /// - 永不创建 Rigidbody；
    /// - 只创建 GSP_* 临时 GameObject、Mesh 与 Material；
    /// - 世界空间顶点会减去 bounds.center，预览根节点放在该中心；
    /// - Clear 会恢复源 Renderer 的原 enabled 状态并销毁全部临时资源。
    /// </summary>
    internal static class D3D11ExtractedMeshPreview
    {
        private sealed class RendererState
        {
            public Renderer Renderer;
            public bool Enabled;
        }

        private static readonly List<RendererState> SourceStates =
            new List<RendererState>();

        private static GameObject _root;
        private static GameObject _target;
        private static Mesh _mesh;
        private static Material _material;
        private static MeshRenderer _previewRenderer;
        private static MeshCollider _meshCollider;
        private static string _targetName;
        private static string _targetPath;
        private static Bounds _sourceBounds;
        private static Bounds _geometryBounds;
        private static Vector3[] _localVertices;
        private static int[] _localIndices;
        private static bool _hasBounds;
        private static int _vertexCount;
        private static int _indexCount;
        private static PreviewMode _mode = PreviewMode.Off;

        public static PreviewMode Mode => _mode;
        public static bool Exists => _root != null && _mesh != null;
        public static bool ColliderExists => _meshCollider != null;
        public static GameObject Target => _target;
        public static MeshCollider ExtractedCollider => _meshCollider;
        public static Bounds SourceBounds => _sourceBounds;
        public static Bounds GeometryBounds => _geometryBounds;
        public static string TargetName => _targetName;
        public static string TargetPath => _targetPath;
        public static int VertexCount => _vertexCount;
        public static int IndexCount => _indexCount;
        public static int TriangleCount => _indexCount / 3;
        public static int PreviewObjectCount => Exists ? 1 : 0;

        public static bool Build(
            D3D11SubmeshGeometry geometry,
            out D3D11MeshPreviewReport report,
            out string error)
        {
            report = new D3D11MeshPreviewReport
            {
                Completed = false,
                Success = false,
                ColliderCreated = false,
                InitialMode = PreviewMode.Off.ToString()
            };
            error = null;
            Clear();

            if (geometry == null)
            {
                error = "geometry-null";
                report.Detail = error;
                return false;
            }

            report.TargetName = geometry.TargetName;
            report.TargetPath = geometry.TargetPath;
            report.RendererPath = geometry.RendererPath;
            report.SourceMeshName = geometry.SourceMeshName;
            report.RoutedSubMeshIndex = geometry.RoutedSubMeshIndex;
            report.CoordinateMode = geometry.CoordinateMode;
            report.VertexCount = geometry.WorldPositions != null
                ? geometry.WorldPositions.Length
                : 0;
            report.IndexCount = geometry.LocalIndices != null
                ? geometry.LocalIndices.Length
                : 0;
            report.TriangleCount = report.IndexCount / 3;
            report.GeometryBounds = FormatBounds(geometry.WorldBounds);

            _target = geometry.Target;
            _targetName = geometry.TargetName;
            _targetPath = geometry.TargetPath;
            _geometryBounds = geometry.WorldBounds;
            _vertexCount = report.VertexCount;
            _indexCount = report.IndexCount;

            if (geometry.Target == null || geometry.SourceRenderer == null)
            {
                error = "target-or-source-renderer-null";
                report.Detail = error;
                return false;
            }
            if (geometry.WorldPositions == null ||
                geometry.WorldPositions.Length == 0)
            {
                error = "world-position-array-empty";
                report.Detail = error;
                return false;
            }
            if (geometry.LocalIndices == null ||
                geometry.LocalIndices.Length < 3 ||
                geometry.LocalIndices.Length % 3 != 0)
            {
                error = "local-index-array-invalid";
                report.Detail = error;
                return false;
            }

            try
            {
                CaptureSourceStates(geometry.Target);
                Bounds sourceBounds = geometry.SourceRenderer.bounds;
                _sourceBounds = sourceBounds;
                _hasBounds = true;
                report.SourceBounds = FormatBounds(sourceBounds);
                report.GeometryMatchesSourceBounds = BoundsApproximatelyEqual(
                    geometry.WorldBounds,
                    sourceBounds);

                Vector3 center = geometry.WorldBounds.center;
                report.RootWorldPosition = center;
                Vector3[] localVertices =
                    new Vector3[geometry.WorldPositions.Length];
                for (int i = 0; i < localVertices.Length; i++)
                    localVertices[i] = geometry.WorldPositions[i] - center;

                // 保留一份由 GSP 自己拥有的托管几何，用于碰撞回调时推导
                // 最近三角形法线。它不会回读 Unity Mesh，也不触碰源缓冲区。
                _localVertices = localVertices;
                _localIndices = (int[])geometry.LocalIndices.Clone();

                _mesh = new Mesh
                {
                    name = "GSP_D3D11_Extracted_" +
                        Sanitize(geometry.TargetName) +
                        "_SubMesh" + geometry.RoutedSubMeshIndex,
                    indexFormat = localVertices.Length >= 65000
                        ? IndexFormat.UInt32
                        : IndexFormat.UInt16
                };
                _mesh.vertices = localVertices;
                _mesh.subMeshCount = 1;
                _mesh.SetIndices(
                    _localIndices,
                    MeshTopology.Triangles,
                    0,
                    false);
                _mesh.RecalculateBounds();
                _mesh.RecalculateNormals();
                report.MeshCreated = true;
                report.PreviewMeshLocalBounds =
                    FormatBounds(_mesh.bounds);

                _material = CreateMaterial();
                if (_material == null)
                    throw new InvalidOperationException(
                        "cannot-create-preview-material");
                report.MaterialCreated = true;

                _root = new GameObject(
                    "GSP_D3D11_Visual_" +
                    Sanitize(geometry.TargetName));
                report.PreviewObjectName = _root.name;
                _root.transform.position = center;
                _root.transform.rotation = Quaternion.identity;
                _root.transform.localScale = Vector3.one;

                MeshFilter filter = _root.AddComponent<MeshFilter>();
                report.MeshFilterCreated = filter != null;
                if (filter == null)
                    throw new InvalidOperationException(
                        "cannot-add-MeshFilter");
                filter.sharedMesh = _mesh;

                _previewRenderer = _root.AddComponent<MeshRenderer>();
                report.MeshRendererCreated = _previewRenderer != null;
                if (_previewRenderer == null)
                    throw new InvalidOperationException(
                        "cannot-add-MeshRenderer");
                _previewRenderer.sharedMaterial = _material;
                _previewRenderer.shadowCastingMode =
                    ShadowCastingMode.Off;
                _previewRenderer.receiveShadows = false;

                Bounds previewBounds = _previewRenderer.bounds;
                report.PreviewRendererBounds =
                    FormatBounds(previewBounds);
                report.PreviewMatchesGeometryBounds =
                    BoundsApproximatelyEqual(
                        previewBounds,
                        geometry.WorldBounds);
                report.PreviewMatchesSourceBounds =
                    BoundsApproximatelyEqual(
                        previewBounds,
                        sourceBounds);

                SetMode(PreviewMode.Overlay);
                report.InitialMode = PreviewMode.Overlay.ToString();
                report.Completed = true;
                report.Success =
                    report.MeshCreated &&
                    report.MaterialCreated &&
                    report.MeshFilterCreated &&
                    report.MeshRendererCreated &&
                    !report.ColliderCreated &&
                    report.PreviewMatchesGeometryBounds;
                report.Detail = report.Success
                    ? "independent translucent Mesh preview created; source object unchanged and no collider added"
                    : "preview object was created but one or more validation checks failed";

                if (!report.Success)
                    error = report.Detail;
                return report.Success;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + " / " + e.Message;
                report.Completed = true;
                report.Success = false;
                report.Detail = error;
                Clear();
                return false;
            }
        }

        public static bool BuildStaticMeshColliderAudit(
            out D3D11MeshColliderAuditReport report,
            out string error)
        {
            report = new D3D11MeshColliderAuditReport
            {
                Completed = false,
                Success = false,
                TargetName = _targetName,
                TargetPath = _targetPath,
                PreviewExists = Exists,
                PreviewObjectName = _root != null ? _root.name : null,
                PreviewMeshName = _mesh != null ? _mesh.name : null,
                VertexCount = _vertexCount,
                IndexCount = _indexCount,
                TriangleCount = _indexCount / 3,
                CookingPath =
                    "MeshCollider.sharedMesh assignment / Unity default cooking options"
            };
            error = null;

            if (!Exists || _previewRenderer == null)
            {
                error = "preview-not-built";
                report.Completed = true;
                report.Detail = error;
                return false;
            }

            if (_meshCollider != null)
            {
                try { Object.Destroy(_meshCollider); }
                catch { }
                _meshCollider = null;
            }

            try
            {
                report.SourceBounds = _hasBounds
                    ? FormatBounds(_sourceBounds)
                    : "<unknown>";
                Bounds previewBounds = _previewRenderer.bounds;
                report.PreviewBounds = FormatBounds(previewBounds);

                _meshCollider = _root.AddComponent<MeshCollider>();
                report.ColliderCreated = _meshCollider != null;
                if (_meshCollider == null)
                    throw new InvalidOperationException(
                        "cannot-add-MeshCollider");

                // 非凸 + 无 Rigidbody = Unity 静态三角网格碰撞体。
                // sharedMesh 先清空再赋值，显式触发当前副本 Mesh 的 cooking。
                _meshCollider.enabled = true;
                _meshCollider.convex = false;
                _meshCollider.isTrigger = false;
                _meshCollider.sharedMesh = null;
                _meshCollider.sharedMesh = _mesh;

                Physics.SyncTransforms();
                report.PhysicsSyncCalled = true;

                report.ColliderEnabled = _meshCollider.enabled;
                report.SharedMeshAssigned =
                    _meshCollider.sharedMesh != null;
                report.SharedMeshMatchesPreviewMesh =
                    _meshCollider.sharedMesh == _mesh;
                report.Convex = _meshCollider.convex;
                report.IsTrigger = _meshCollider.isTrigger;
                report.AttachedRigidbodyPresent =
                    _meshCollider.attachedRigidbody != null;

                Bounds colliderBounds = _meshCollider.bounds;
                report.ColliderBounds = FormatBounds(colliderBounds);
                report.ColliderMatchesPreviewBounds =
                    BoundsApproximatelyEqual(
                        colliderBounds,
                        previewBounds);
                report.ColliderMatchesSourceBounds =
                    _hasBounds && BoundsApproximatelyEqual(
                        colliderBounds,
                        _sourceBounds);

                report.Completed = true;
                report.Success =
                    report.ColliderCreated &&
                    report.ColliderEnabled &&
                    report.SharedMeshAssigned &&
                    report.SharedMeshMatchesPreviewMesh &&
                    !report.Convex &&
                    !report.IsTrigger &&
                    !report.AttachedRigidbodyPresent &&
                    report.PhysicsSyncCalled &&
                    report.ColliderMatchesPreviewBounds &&
                    (!_hasBounds || report.ColliderMatchesSourceBounds);
                report.Detail = report.Success
                    ? "static non-convex MeshCollider cooked on the extracted copy; no Rigidbody or source-object modification"
                    : "MeshCollider was created but one or more cooking/bounds checks failed";
                if (!report.Success)
                    error = report.Detail;
                return report.Success;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + " / " + e.Message;
                report.Completed = true;
                report.Success = false;
                report.Detail = error;
                if (_meshCollider != null)
                {
                    try { Object.Destroy(_meshCollider); }
                    catch { }
                    _meshCollider = null;
                }
                return false;
            }
        }


        /// <summary>
        /// 为未来观测适配器复制一份由调用方拥有的世界空间顶点和本地索引。
        /// 数据来自 GSP 已验证的托管缓存，不会重新读取 Unity Mesh 或 GPU buffer。
        /// </summary>
        public static bool TryCopyWorldGeometry(
            out Vector3[] worldVertices,
            out int[] localIndices,
            out Bounds worldBounds)
        {
            worldVertices = null;
            localIndices = null;
            worldBounds = default;
            if (_root == null || _localVertices == null ||
                _localIndices == null || !_hasBounds)
            {
                return false;
            }

            try
            {
                worldVertices = new Vector3[_localVertices.Length];
                for (int i = 0; i < _localVertices.Length; i++)
                {
                    worldVertices[i] =
                        _root.transform.TransformPoint(_localVertices[i]);
                }
                localIndices = (int[])_localIndices.Clone();
                worldBounds = _geometryBounds;
                return true;
            }
            catch
            {
                worldVertices = null;
                localIndices = null;
                worldBounds = default;
                return false;
            }
        }

        /// <summary>
        /// 把世界坐标点归一化到以提取 bounds 中心为原点的近似 [-1,1] 空间。
        /// 该接口可直接供后续 ML-Agents/VLM 适配器使用，但 GSP 不决定最终网络输入。
        /// </summary>
        public static bool TryNormalizeWorldPoint(
            Vector3 worldPoint,
            out Vector3 normalizedPoint)
        {
            normalizedPoint = Vector3.zero;
            if (!_hasBounds)
                return false;

            Vector3 extents = _geometryBounds.extents;
            float ex = Mathf.Max(Mathf.Abs(extents.x), 0.000001f);
            float ey = Mathf.Max(Mathf.Abs(extents.y), 0.000001f);
            float ez = Mathf.Max(Mathf.Abs(extents.z), 0.000001f);
            Vector3 delta = worldPoint - _geometryBounds.center;
            normalizedPoint = new Vector3(
                delta.x / ex,
                delta.y / ey,
                delta.z / ez);
            return IsFiniteVector(normalizedPoint);
        }

        /// <summary>
        /// 从 GSP 自己缓存的提取网格中查找离接触点最近的三角形，并返回
        /// 朝向球心一侧的世界空间法线。该值是明确标记的派生诊断值，
        /// 不冒充 Unity Collision.GetContact(0).normal。
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
            {
                return false;
            }

            try
            {
                Vector3 localPoint =
                    _root.transform.InverseTransformPoint(worldPoint);
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
                    {
                        continue;
                    }

                    Vector3 a = _localVertices[ia];
                    Vector3 b = _localVertices[ib];
                    Vector3 c = _localVertices[ic];
                    Vector3 cross = Vector3.Cross(b - a, c - a);
                    float crossSqr = cross.sqrMagnitude;
                    if (!IsFinite(crossSqr) || crossSqr <= 1e-16f)
                        continue;

                    Vector3 candidate =
                        ClosestPointOnTriangle(localPoint, a, b, c);
                    float sqr = (candidate - localPoint).sqrMagnitude;
                    if (!IsFinite(sqr) || sqr >= bestSqr)
                        continue;

                    bestSqr = sqr;
                    bestPoint = candidate;
                    bestNormal = cross / Mathf.Sqrt(crossSqr);
                    bestTriangle = i / 3;
                }

                if (bestTriangle < 0 || !IsFiniteVector(bestNormal))
                    return false;

                nearestWorldPoint =
                    _root.transform.TransformPoint(bestPoint);
                worldNormal =
                    _root.transform.TransformDirection(bestNormal).normalized;

                Vector3 toward = towardWorldPoint - nearestWorldPoint;
                if (toward.sqrMagnitude > 1e-12f &&
                    Vector3.Dot(worldNormal, toward) < 0f)
                {
                    worldNormal = -worldNormal;
                }

                triangleIndex = bestTriangle;
                distance = Mathf.Sqrt(bestSqr);
                return IsFiniteVector(worldNormal) &&
                    worldNormal.sqrMagnitude > 0.5f;
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
            Vector3 p,
            Vector3 a,
            Vector3 b,
            Vector3 c)
        {
            // Real-Time Collision Detection (Christer Ericson) 的标准区域测试。
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
            if (va <= 0f && (d4 - d3) >= 0f &&
                (d5 - d6) >= 0f)
            {
                Vector3 bc = c - b;
                float w = (d4 - d3) /
                    ((d4 - d3) + (d5 - d6));
                return b + w * bc;
            }

            float denominator = 1f / (va + vb + vc);
            float faceV = vb * denominator;
            float faceW = vc * denominator;
            return a + ab * faceV + ac * faceW;
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        private static bool IsFiniteVector(Vector3 value)
        {
            return IsFinite(value.x) &&
                IsFinite(value.y) &&
                IsFinite(value.z);
        }

        public static D3D11EnvironmentObservationReport BuildObservationReport()
        {
            D3D11EnvironmentObservationReport report =
                new D3D11EnvironmentObservationReport
                {
                    Completed = true,
                    Success = Exists && _hasBounds &&
                        _localVertices != null &&
                        _localIndices != null,
                    SchemaVersion = "gsp-env-observation-v1",
                    ContractStage =
                        "foundation-not-final-ML-vector",
                    TargetName = _targetName,
                    TargetPath = _targetPath,
                    CoordinateSpace = "world",
                    RootFrame = _root != null
                        ? "GSP extracted bounds-center root"
                        : "<none>",
                    GeometryReady = Exists,
                    ColliderReady = ColliderExists,
                    VertexCount = _vertexCount,
                    IndexCount = _indexCount,
                    TriangleCount = _indexCount / 3,
                    PointNormalization =
                        "p_norm=(p_world-bounds.center)/max(bounds.extents,epsilon)",
                    SupportsWorldGeometryCopy =
                        Exists && _localVertices != null &&
                        _localIndices != null,
                    SupportsNearestTriangleSurfaceQuery =
                        Exists && _localVertices != null &&
                        _localIndices != null,
                    SupportsSurfaceNormal =
                        Exists && _localVertices != null &&
                        _localIndices != null,
                    SupportsContactPoint = ColliderExists,
                    SupportsContactImpulse = ColliderExists,
                    SupportsBoundsNormalization = _hasBounds,
                    IntendedConsumers =
                        "Active-Ragdoll environment adapter / ML-Agents observation adapter / VLM world-target validation",
                    NotYetIncluded =
                        "ragdoll joint pose and velocity; joint torque history; character self-collision; semantic object labels; VLM task target; finger/toe direct-control channels",
                    Detail = Exists
                        ? "neutral geometry/contact DTO is ready without referencing ML-Agents assemblies; later adapters may batch, normalize or downsample it"
                        : "no extracted geometry exists; build a D3D11 mesh/collider first"
                };

            if (_hasBounds)
            {
                report.BoundsCenter = FormatVector(_geometryBounds.center);
                report.BoundsSize = FormatVector(_geometryBounds.size);
                report.BoundsMin = FormatVector(_geometryBounds.min);
                report.BoundsMax = FormatVector(_geometryBounds.max);
            }
            return report;
        }

        public static bool TryParseMode(
            string text,
            out PreviewMode mode)
        {
            return ExactMeshPreview.TryParseMode(text, out mode);
        }

        public static bool SetMode(PreviewMode mode)
        {
            if (!Exists)
                return false;

            _mode = mode;
            bool showPreview =
                mode == PreviewMode.Overlay ||
                mode == PreviewMode.Extracted;
            bool showSource = mode != PreviewMode.Extracted;

            if (_root != null)
                _root.SetActive(showPreview);

            for (int i = 0; i < SourceStates.Count; i++)
            {
                RendererState state = SourceStates[i];
                if (state?.Renderer == null)
                    continue;
                state.Renderer.enabled = showSource
                    ? state.Enabled
                    : false;
            }
            return true;
        }

        public static void Clear()
        {
            RestoreSourceRenderers();

            if (_root != null)
            {
                Object.Destroy(_root);
                _root = null;
            }
            if (_mesh != null)
            {
                Object.Destroy(_mesh);
                _mesh = null;
            }
            if (_material != null)
            {
                Object.Destroy(_material);
                _material = null;
            }

            _previewRenderer = null;
            _meshCollider = null;
            _target = null;
            _targetName = null;
            _targetPath = null;
            _sourceBounds = default;
            _geometryBounds = default;
            _localVertices = null;
            _localIndices = null;
            _hasBounds = false;
            _vertexCount = 0;
            _indexCount = 0;
            SourceStates.Clear();
            _mode = PreviewMode.Off;
        }

        private static void CaptureSourceStates(GameObject target)
        {
            SourceStates.Clear();
            Renderer[] renderers;
            try
            {
                renderers =
                    target.GetComponentsInChildren<Renderer>(true);
            }
            catch
            {
                renderers = Array.Empty<Renderer>();
            }

            for (int i = 0; i < renderers.Length; i++)
            {
                Renderer renderer = renderers[i];
                if (renderer == null || renderer.gameObject == null)
                    continue;
                if ((renderer.gameObject.name ?? string.Empty)
                    .StartsWith("GSP_", StringComparison.Ordinal))
                {
                    continue;
                }

                SourceStates.Add(new RendererState
                {
                    Renderer = renderer,
                    Enabled = renderer.enabled
                });
            }
        }

        private static void RestoreSourceRenderers()
        {
            for (int i = 0; i < SourceStates.Count; i++)
            {
                RendererState state = SourceStates[i];
                if (state?.Renderer != null)
                    state.Renderer.enabled = state.Enabled;
            }
        }

        private static Material CreateMaterial()
        {
            Shader shader =
                Shader.Find("Legacy Shaders/Transparent/Diffuse");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                return null;

            Color color =
                new Color(0.05f, 0.90f, 1.00f, 0.42f);
            Material material = new Material(shader)
            {
                name = "GSP_D3D11_VisualMeshMaterial",
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

        private static bool BoundsApproximatelyEqual(
            Bounds a,
            Bounds b)
        {
            float scale = Mathf.Max(
                1f,
                Mathf.Max(a.size.magnitude, b.size.magnitude));
            float tolerance = Mathf.Max(0.0005f, scale * 0.0005f);
            return
                Vector3.Distance(a.center, b.center) <= tolerance &&
                Vector3.Distance(a.size, b.size) <= tolerance;
        }

        private static string FormatBounds(Bounds bounds)
        {
            return
                "center=" + FormatVector(bounds.center) +
                "/size=" + FormatVector(bounds.size);
        }

        private static string FormatVector(Vector3 value)
        {
            return "(" +
                value.x.ToString("G9") + "," +
                value.y.ToString("G9") + "," +
                value.z.ToString("G9") + ")";
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
                return "Unnamed";

            char[] chars = value.ToCharArray();
            for (int i = 0; i < chars.Length; i++)
            {
                char c = chars[i];
                if (!(char.IsLetterOrDigit(c) ||
                    c == '_' || c == '-'))
                {
                    chars[i] = '_';
                }
            }
            return new string(chars);
        }
    }
}
