using System;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Acquisition;
using MakemitAGA.GeometrySourceProbe.Model;
using MakemitAGA.GeometrySourceProbe.Visualization;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Runtime
{
    internal static class GeometrySourceProbeRuntime
    {
        private static ManualLogSource _logger;
        private static GeometryProbeReport _report;
        private static GeometryInventoryReport _inventoryReport;
        private static GeometryLayoutReport _layoutReport;
        private static BufferHandleProbeReport _handleReport;
        private static RendererRouteReport _rendererRouteReport;
        private static D3D11FullSubmeshReadbackReport _d3d11FullSubmeshReadbackReport;
        private static D3D11MeshPreviewReport _d3d11MeshPreviewReport;
        private static D3D11MeshColliderAuditReport _d3d11MeshColliderAuditReport;
        private static D3D11EnvironmentObservationReport _d3d11EnvironmentObservationReport;
        private static HouseGeometryCensusReport _houseGeometryCensusReport;
        private static MultipartAggregationReport _multipartAggregationReport;
        private static GameObject _target;
        private static string _sceneName;
        private static int _sceneHandle;

        public static void Initialize(ManualLogSource logger)
        {
            _logger = logger;
        }


        public static string Inventory(string targetName)
        {
            Clear("new-inventory");

            GameObject target = TargetFinder.FindNearestExactName(
                targetName,
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);

            if (target == null)
            {
                _inventoryReport = new GeometryInventoryReport
                {
                    Completed = false,
                    TargetName = targetName,
                    ExactMatches = exactMatches,
                    SelectedDistance = distance,
                    ReferenceName = referenceName,
                    Summary = error
                };

                _logger?.LogWarning(
                    "[GSP] Inventory failed | " + _inventoryReport.ToText(false));
                return "<color=red>GSP 安全清单失败：</color> " + error;
            }

            _target = target;
            _inventoryReport = MeshSourceAcquirer.Inventory(
                target,
                exactMatches,
                distance,
                referenceName,
                _logger);

            return
                "<color=#66ddff>安全几何清单完成。</color> " +
                "MeshFilter=" + _inventoryReport.MeshFilters +
                " / Skinned=" + _inventoryReport.SkinnedRenderers +
                " / MeshCollider=" + _inventoryReport.MeshColliders +
                " / entries=" + _inventoryReport.Entries.Count +
                "。执行 gsp_inventory_report 查看逐组件元数据。";
        }


        public static string Layout(string targetName)
        {
            Clear("new-layout");

            GameObject target = TargetFinder.FindNearestExactName(
                targetName,
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);

            if (target == null)
            {
                _layoutReport = new GeometryLayoutReport
                {
                    Completed = false,
                    TargetName = targetName,
                    ExactMatches = exactMatches,
                    SelectedDistance = distance,
                    ReferenceName = referenceName,
                    Summary = error
                };
                _logger?.LogWarning("[GSP] Layout failed | " + _layoutReport.ToText(false));
                return "<color=red>GSP 布局清单失败：</color> " + error;
            }

            _target = target;
            _layoutReport = MeshLayoutInspector.Layout(
                target,
                exactMatches,
                distance,
                referenceName,
                _logger);

            return
                "<color=#66ddff>只读 Mesh 布局清单完成。</color> " +
                "entries=" + _layoutReport.Entries.Count +
                "。没有取得 GPU buffer，也没有读取顶点或索引。" +
                "执行 gsp_layout_report 查看详细结果。";
        }

        public static string ProbeHandle(string targetName, string requestedBuffer)
        {
            // 不调用完整 Clear，避免把上一步 layout 报告清掉；只清理预览和几何报告。
            ClearAllPreviews();
            _report = null;

            GameObject target = TargetFinder.FindNearestExactName(
                targetName,
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);

            if (target == null)
            {
                _handleReport = new BufferHandleProbeReport
                {
                    Completed = false,
                    Success = false,
                    TargetName = targetName,
                    RequestedBuffer = requestedBuffer,
                    Detail = error
                };
                _logger?.LogWarning("[GSP] Handle probe failed | " + _handleReport.ToText());
                return "<color=red>GSP 句柄探测失败：</color> " + error;
            }

            _target = target;
            _handleReport = MeshLayoutInspector.ProbeExistingHandle(
                target,
                requestedBuffer,
                _logger);

            if (!_handleReport.Success)
            {
                return
                    "<color=red>现有 GPU buffer 句柄探测未成功。</color> " +
                    "执行 gsp_handle_report 查看边界位置和失败原因。";
            }

            return
                "<color=#66ddff>现有 GPU buffer 句柄可取得。</color> " +
                "request=" + _handleReport.RequestedBuffer +
                " / count=" + _handleReport.BufferCount +
                " / stride=" + _handleReport.BufferStride +
                " / target=" + _handleReport.BufferTarget +
                " / unchanged=" + _handleReport.TargetsUnchanged +
                "。本次没有读取任何 buffer 数据。";
        }

        public static string InspectRendererRoute(string targetName)
        {
            // 只更新 Renderer 路由报告，不清掉已有 layout/handle 结果。
            ClearAllPreviews();
            _report = null;

            GameObject target = TargetFinder.FindNearestExactName(
                targetName,
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);

            if (target == null)
            {
                _rendererRouteReport = new RendererRouteReport
                {
                    Completed = false,
                    TargetName = targetName,
                    ExactMatches = exactMatches,
                    SelectedDistance = distance,
                    ReferenceName = referenceName,
                    Summary = error
                };
                _logger?.LogWarning(
                    "[GSP] Renderer route failed | " + _rendererRouteReport.ToText(false));
                return "<color=red>GSP Renderer 路由检查失败：</color> " + error;
            }

            _target = target;
            _rendererRouteReport = RendererRouteInspector.Inspect(
                target,
                exactMatches,
                distance,
                referenceName,
                _logger);

            return
                "<color=#66ddff>Renderer 静态批处理路由检查完成。</color> " +
                "entries=" + _rendererRouteReport.Entries.Count +
                "。本次没有调用 GPU API；执行 gsp_renderer_report 查看子网格区间。";
        }


        public static string ProbeD3D11FullSubmesh(string targetName)
        {
            // 保留此前的单三角形报告；本轮只更新完整子网格统计报告。
            ClearAllPreviews();
            _report = null;

            GameObject target = TargetFinder.FindNearestExactName(
                targetName,
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);

            if (target == null)
            {
                _d3d11FullSubmeshReadbackReport =
                    new D3D11FullSubmeshReadbackReport
                    {
                        Completed = false,
                        Success = false,
                        TargetName = targetName,
                        Detail = error
                    };
                _logger?.LogWarning(
                    "[GSP] D3D11 full-submesh readback failed | " +
                    _d3d11FullSubmeshReadbackReport.ToText());
                return
                    "<color=red>GSP D3D11 完整子网格读回失败：</color> " +
                    error;
            }

            _target = target;
            _d3d11FullSubmeshReadbackReport =
                D3D11FullSubmeshReadback.Probe(target, _logger);

            if (!_d3d11FullSubmeshReadbackReport.Success)
            {
                return
                    "<color=red>D3D11 完整子网格验证未通过。</color> " +
                    "执行 gsp_d3d11_submesh_report 查看索引范围、Position、bounds、退化三角形与 COM 清理统计。";
            }

            return
                "<color=#66ddff>D3D11 完整子网格读回成功。</color> " +
                "indices=" + _d3d11FullSubmeshReadbackReport.DecodedIndexCount +
                " / vertices=" + _d3d11FullSubmeshReadbackReport.DecodedPositionCount +
                " / triangles=" + _d3d11FullSubmeshReadbackReport.TriangleCount +
                " / outOfRange=" + _d3d11FullSubmeshReadbackReport.OutOfRangeIndexCount +
                " / nonFinite=" + _d3d11FullSubmeshReadbackReport.NonFinitePositionCount +
                " / repeated=" + _d3d11FullSubmeshReadbackReport.RepeatedIndexTriangleCount +
                " / zeroArea=" + _d3d11FullSubmeshReadbackReport.ZeroAreaTriangleCount +
                " / coordinateMode=" + _d3d11FullSubmeshReadbackReport.CoordinateMode +
                " / unchanged=" + _d3d11FullSubmeshReadbackReport.TargetsUnchanged +
                "。本次只统计验证，没有创建新的 Unity Mesh。";
        }

        public static string GetD3D11FullSubmeshReportText()
        {
            return _d3d11FullSubmeshReadbackReport == null
                ? "当前没有 D3D11 完整子网格报告。先执行 gsp_d3d11_submesh(对象名称)。"
                : _d3d11FullSubmeshReadbackReport.ToText();
        }

        public static string BuildD3D11MeshPreview(string targetName)
        {
            // 两类预览互斥，防止旧的 CPU 路径预览与新 GPU 路径预览重叠。
            ClearAllPreviews();
            _report = null;

            GameObject target = TargetFinder.FindNearestExactName(
                targetName,
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);

            if (target == null)
            {
                _d3d11MeshPreviewReport =
                    new D3D11MeshPreviewReport
                    {
                        Completed = false,
                        Success = false,
                        TargetName = targetName,
                        Detail = error
                    };
                _logger?.LogWarning(
                    "[GSP] D3D11 mesh preview failed | " +
                    _d3d11MeshPreviewReport.ToText());
                return
                    "<color=red>GSP D3D11 可视化副本失败：</color> " +
                    error;
            }

            _target = target;
            _d3d11FullSubmeshReadbackReport =
                D3D11FullSubmeshReadback.Probe(
                    target,
                    _logger,
                    out D3D11SubmeshGeometry geometry);

            if (!_d3d11FullSubmeshReadbackReport.Success ||
                geometry == null)
            {
                _d3d11MeshPreviewReport =
                    new D3D11MeshPreviewReport
                    {
                        Completed = true,
                        Success = false,
                        TargetName = targetName,
                        Detail =
                            "full-submesh readback did not provide validated geometry"
                    };
                return
                    "<color=red>D3D11 完整几何读回未通过，未创建可视化副本。</color> " +
                    "执行 gsp_d3d11_submesh_report 查看原因。";
            }

            bool built = D3D11ExtractedMeshPreview.Build(
                geometry,
                out _d3d11MeshPreviewReport,
                out string previewError);

            _logger?.LogInfo(
                "[GSP] D3D11 mesh preview result | " +
                _d3d11MeshPreviewReport.ToText());

            if (!built)
            {
                return
                    "<color=red>D3D11 几何已读回，但可视化副本创建失败。</color> " +
                    (previewError ?? "<unknown>") +
                    "。执行 gsp_d3d11_mesh_report 查看详情。";
            }

            return
                "<color=#66ddff>D3D11 独立可视化副本创建成功。</color> " +
                "vertices=" + _d3d11MeshPreviewReport.VertexCount +
                " / indices=" + _d3d11MeshPreviewReport.IndexCount +
                " / triangles=" + _d3d11MeshPreviewReport.TriangleCount +
                " / boundsMatch=" +
                    _d3d11MeshPreviewReport.PreviewMatchesSourceBounds +
                " / mode=overlay" +
                " / collider=False。" +
                "可用 gsp_d3d11_mesh_mode(extracted/source/overlay/off) 切换显示，" +
                "或 gsp_d3d11_mesh_clear 删除副本。";
        }

        public static string BuildD3D11MeshColliderAudit(string targetName)
        {
            // 先走已经验证的完整 D3D11 读回与独立 Mesh 创建流程；
            // 只有预览副本创建成功后，才在该副本上添加静态非凸 MeshCollider。
            string previewResult = BuildD3D11MeshPreview(targetName);
            if (_d3d11MeshPreviewReport == null ||
                !_d3d11MeshPreviewReport.Success ||
                !D3D11ExtractedMeshPreview.Exists)
            {
                _d3d11MeshColliderAuditReport =
                    new D3D11MeshColliderAuditReport
                    {
                        Completed = true,
                        Success = false,
                        TargetName = targetName,
                        Detail =
                            "validated preview Mesh was not available; " +
                            previewResult
                    };
                _logger?.LogWarning(
                    "[GSP] D3D11 static MeshCollider audit skipped | " +
                    _d3d11MeshColliderAuditReport.ToText());
                return
                    "<color=red>D3D11 静态 MeshCollider 审计未开始：</color> " +
                    "独立 Mesh 副本创建失败。先查看 gsp_d3d11_mesh_report。";
            }

            bool cooked =
                D3D11ExtractedMeshPreview.BuildStaticMeshColliderAudit(
                    out _d3d11MeshColliderAuditReport,
                    out string colliderError);

            if (_d3d11MeshPreviewReport != null)
            {
                _d3d11MeshPreviewReport.ColliderCreated =
                    _d3d11MeshColliderAuditReport.ColliderCreated;
                if (cooked)
                {
                    _d3d11MeshPreviewReport.Detail =
                        "independent translucent Mesh preview plus audited static non-convex MeshCollider; source object unchanged";
                }
            }

            _logger?.LogInfo(
                "[GSP] D3D11 static MeshCollider audit result | " +
                _d3d11MeshColliderAuditReport.ToText());

            if (!cooked)
            {
                return
                    "<color=red>D3D11 Mesh 已提取，但静态 MeshCollider 烹饪审计未通过。</color> " +
                    (colliderError ?? "<unknown>") +
                    "。执行 gsp_d3d11_collider_report 查看详情。";
            }

            return
                "<color=#66ddff>D3D11 静态 MeshCollider 烹饪审计成功。</color> " +
                "vertices=" + _d3d11MeshColliderAuditReport.VertexCount +
                " / triangles=" +
                    _d3d11MeshColliderAuditReport.TriangleCount +
                " / convex=False" +
                " / rigidbody=False" +
                " / boundsMatch=" +
                    _d3d11MeshColliderAuditReport.ColliderMatchesSourceBounds +
                "。本轮没有 Rigidbody、动态接触或源对象修改。";
        }

        public static string GetD3D11MeshColliderAuditReportText()
        {
            return _d3d11MeshColliderAuditReport == null
                ? "当前没有 D3D11 静态 MeshCollider 审计报告。先执行 gsp_d3d11_collider(对象名称)。"
                : _d3d11MeshColliderAuditReport.ToText();
        }

        public static string ClearD3D11MeshColliderAudit()
        {
            bool existed = D3D11ExtractedMeshPreview.Exists;
            D3D11ExtractedMeshPreview.Clear();
            _d3d11EnvironmentObservationReport = null;
            _logger?.LogInfo(
                "[GSP] D3D11 static MeshCollider audit cleared | existed=" +
                existed);
            return existed
                ? "D3D11 MeshCollider 审计对象、Mesh 与材质已删除，原 Renderer 状态已恢复。"
                : "当前没有需要删除的 D3D11 MeshCollider 审计对象。";
        }

        public static string BuildD3D11ObservationFoundation(
            string targetName)
        {
            string colliderResult = BuildD3D11MeshColliderAudit(targetName);
            if (_d3d11MeshColliderAuditReport == null ||
                !_d3d11MeshColliderAuditReport.Success ||
                !D3D11ExtractedMeshPreview.Exists)
            {
                _d3d11EnvironmentObservationReport =
                    new D3D11EnvironmentObservationReport
                    {
                        Completed = true,
                        Success = false,
                        SchemaVersion = "gsp-env-observation-v1",
                        ContractStage =
                            "foundation-not-final-ML-vector",
                        TargetName = targetName,
                        Detail =
                            "validated extracted geometry/collider was unavailable; " +
                            colliderResult
                    };
                return
                    "<color=red>D3D11 环境观测基础未建立：</color> " +
                    "先查看 gsp_d3d11_collider_report。";
            }

            _d3d11EnvironmentObservationReport =
                D3D11ExtractedMeshPreview.BuildObservationReport();
            _logger?.LogInfo(
                "[GSP] D3D11 environment observation foundation | " +
                _d3d11EnvironmentObservationReport.ToText());

            return _d3d11EnvironmentObservationReport.Success
                ? "<color=#66ddff>D3D11 环境观测基础已建立。</color> " +
                    "schema=" +
                    _d3d11EnvironmentObservationReport.SchemaVersion +
                    " / worldGeometry=True / surfaceQuery=True / collider=True。" +
                    "本版不引用 ML-Agents；后续由独立 Adapter 选择降采样、批量化和关节观测拼接。"
                : "<color=red>D3D11 环境观测基础报告未通过。</color> " +
                    (_d3d11EnvironmentObservationReport.Detail ??
                        "<unknown>");
        }

        public static string GetD3D11ObservationFoundationReportText()
        {
            if (D3D11ExtractedMeshPreview.Exists)
            {
                _d3d11EnvironmentObservationReport =
                    D3D11ExtractedMeshPreview.BuildObservationReport();
            }

            return _d3d11EnvironmentObservationReport == null
                ? "当前没有 D3D11 环境观测基础报告。先执行 gsp_d3d11_observation(对象名称)。"
                : _d3d11EnvironmentObservationReport.ToText();
        }

        public static string SetD3D11MeshPreviewMode(string text)
        {
            if (!D3D11ExtractedMeshPreview.TryParseMode(
                text,
                out PreviewMode mode))
            {
                return
                    "用法：gsp_d3d11_mesh_mode(overlay/extracted/source/off)";
            }


            if (!D3D11ExtractedMeshPreview.SetMode(mode))
            {
                return
                    "<color=yellow>当前没有 D3D11 可视化副本。</color> " +
                    "先执行 gsp_d3d11_mesh(对象名称)。";
            }

            _logger?.LogInfo(
                "[GSP] D3D11 mesh preview mode | mode=" + mode +
                " | objects=" +
                    D3D11ExtractedMeshPreview.PreviewObjectCount);
            return
                "GSP D3D11 可视化模式=" + mode +
                "，对象数=" +
                    D3D11ExtractedMeshPreview.PreviewObjectCount +
                "。";
        }

        public static string ClearD3D11MeshPreview()
        {
            bool existed = D3D11ExtractedMeshPreview.Exists;
            D3D11ExtractedMeshPreview.Clear();
            _d3d11EnvironmentObservationReport = null;
            _logger?.LogInfo(
                "[GSP] D3D11 mesh preview cleared | existed=" + existed);
            return existed
                ? "D3D11 可视化副本已删除，原 Renderer 状态已恢复。"
                : "当前没有需要删除的 D3D11 可视化副本。";
        }

        public static string GetD3D11MeshPreviewReportText()
        {
            return _d3d11MeshPreviewReport == null
                ? "当前没有 D3D11 可视化副本报告。先执行 gsp_d3d11_mesh(对象名称)。"
                : _d3d11MeshPreviewReport.ToText();
        }

        public static string GetRendererRouteReportText()
        {
            return _rendererRouteReport == null
                ? "当前没有 GSP Renderer 路由报告。先执行 gsp_renderer(对象名称)。"
                : _rendererRouteReport.ToText(true);
        }

        public static string GetLayoutReportText()
        {
            return _layoutReport == null
                ? "当前没有 GSP 布局清单。先执行 gsp_layout(对象名称)。"
                : _layoutReport.ToText(true);
        }

        public static string GetHandleReportText()
        {
            return _handleReport == null
                ? "当前没有 GSP 句柄探测报告。先执行 gsp_handle(对象名称,vertex0/index)。"
                : _handleReport.ToText();
        }

        public static string Probe(string targetName, ProbePreference preference)
        {
            Clear("new-probe");

            GameObject target = TargetFinder.FindNearestExactName(
                targetName,
                out int exactMatches,
                out float distance,
                out string referenceName,
                out string error);

            if (target == null)
            {
                _report = new GeometryProbeReport
                {
                    Completed = false,
                    TargetName = targetName,
                    ExactMatches = exactMatches,
                    SelectedDistance = distance,
                    ReferenceName = referenceName,
                    Preference = preference,
                    Summary = error
                };

                _logger?.LogWarning("[GSP] Probe failed | " + _report.ToText(false));
                return "<color=red>GSP 探测失败：</color> " + error;
            }

            _target = target;
            _report = MeshSourceAcquirer.Probe(
                target,
                exactMatches,
                distance,
                referenceName,
                preference,
                _logger);

            bool previewBuilt = false;
            string previewError = null;
            if (_report.Snapshots.Count > 0)
            {
                previewBuilt = ExactMeshPreview.Build(
                    target,
                    _report.Snapshots,
                    out previewError);
            }

            string full = _report.ToText(false) +
                " | previewBuilt=" + previewBuilt +
                " | previewError=" + (previewError ?? "<none>");
            _logger?.LogInfo("[GSP] Probe result | " + full);

            if (_report.Snapshots.Count == 0)
            {
                return
                    "<color=red>没有取得可重建的精确网格。</color> " +
                    "请执行 gsp_report 查看每条路径的失败原因。";
            }

            return
                "<color=#66ddff>精确几何探测完成。</color> " +
                "selected=" + _report.SelectedSnapshots +
                " / vertices=" + _report.SelectedVertices +
                " / indices=" + _report.SelectedIndices +
                " / preview=" + (previewBuilt ? "overlay" : "failed") +
                "。执行 gsp_report 查看全部来源。";
        }

        public static string StartHouseGeometryCensus(string roomFilter)
        {
            // 两种批量 D3D11 审计不并发，避免同一帧连续创建多个 staging 任务。
            HouseFloorEnvironmentRuntime.Clear("house-census-start");
            MultipartObjectAggregator.Cancel("house-census-start");
            // 房间普查与实例聚合不并发；保留已经完成的单物体观测与固定碰撞体审计报告。
            string result = HouseGeometryCensus.Start(
                _logger,
                roomFilter,
                out _houseGeometryCensusReport);
            return result;
        }

        public static void UpdateHouseGeometryCensus()
        {
            HouseGeometryCensus.Update();
            _houseGeometryCensusReport = HouseGeometryCensus.CurrentReport;
        }

        public static string GetHouseGeometryCensusStatusText()
        {
            return HouseGeometryCensus.GetStatusText();
        }

        public static string GetHouseGeometryCensusReportText()
        {
            _houseGeometryCensusReport = HouseGeometryCensus.CurrentReport;
            return HouseGeometryCensus.GetReportText();
        }

        public static string CancelHouseGeometryCensus()
        {
            string result = HouseGeometryCensus.Cancel("manual-cancel");
            _houseGeometryCensusReport = HouseGeometryCensus.CurrentReport;
            return result;
        }

        public static string StartMultipartAggregationAudit(string targetName)
        {
            InstanceProxyContactTest.Stop("new-instance-audit", false);
            HouseFloorEnvironmentRuntime.Clear("new-instance-audit");
            EnvironmentProviderRuntime.Clear("new-instance-audit");
            HouseGeometryCensus.Cancel("multipart-audit-start");
            return MultipartObjectAggregator.Start(
                _logger,
                targetName,
                out _multipartAggregationReport);
        }

        public static void UpdateMultipartAggregationAudit()
        {
            MultipartObjectAggregator.Update();
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
        }

        public static string GetMultipartAggregationStatusText()
        {
            return MultipartObjectAggregator.GetStatusText();
        }

        public static string GetMultipartAggregationReportText()
        {
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            return MultipartObjectAggregator.GetReportText();
        }

        public static string CancelMultipartAggregationAudit()
        {
            string result = MultipartObjectAggregator.Cancel("manual-cancel");
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            return result;
        }

        public static string ExportSpatialInstanceDraft()
        {
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            return MultipartObjectAggregator.ExportSpatialInstanceDraft();
        }

        public static string EnsurePhysicsWhitelistTemplate()
        {
            return ObjectPhysicsWhitelist.EnsureTemplate(_logger);
        }

        public static string ReloadPhysicsWhitelist()
        {
            return ObjectPhysicsWhitelist.Reload(_logger);
        }

        public static string GetPhysicsWhitelistStatusText()
        {
            return ObjectPhysicsWhitelist.GetStatusText();
        }

        public static string EnsureInstanceProfileWhitelistTemplate()
        {
            return SpatialInstancePhysicsWhitelist.EnsureTemplate(_logger);
        }

        public static string ReloadInstanceProfileWhitelist()
        {
            return SpatialInstancePhysicsWhitelist.Reload(_logger);
        }

        public static string GetInstanceProfileWhitelistStatusText()
        {
            return SpatialInstancePhysicsWhitelist.GetStatusText();
        }

        public static string SeedCurrentInstanceProfiles()
        {
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            return SpatialInstancePhysicsWhitelist.SeedCurrentReport(
                _multipartAggregationReport,
                _logger);
        }

        public static string RemoveCurrentInstanceProfileSeed()
        {
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            return SpatialInstancePhysicsWhitelist.RemoveCurrentReportSeed(
                _multipartAggregationReport,
                _logger);
        }

        public static string BuildCurrentEnvironmentProvider()
        {
            InstanceProxyContactTest.Stop("environment-provider-build", false);
            HouseFloorEnvironmentRuntime.Clear("single-environment-provider-build");
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            return EnvironmentProviderRuntime.Build(_multipartAggregationReport, _logger);
        }

        public static string GetCurrentEnvironmentProviderStatusText()
        {
            return EnvironmentProviderRuntime.GetStatusText();
        }

        public static string SetCurrentEnvironmentProviderVisual(bool visible)
        {
            return EnvironmentProviderRuntime.SetVisual(visible);
        }

        public static string SetCurrentEnvironmentProviderCollider(bool enabled)
        {
            if (InstanceProxyContactTest.IsRunning)
                return "受控接触验证正在运行；测试完成或取消后才能切换 environment Collider。";
            return EnvironmentProviderRuntime.SetCollider(enabled);
        }

        public static string RollbackCurrentEnvironmentProvider()
        {
            InstanceProxyContactTest.Stop("environment-provider-rollback", false);
            return EnvironmentProviderRuntime.Rollback();
        }

        public static string StartHouseFloorEnvironmentBuild()
        {
            HouseGeometryCensus.Cancel("house-floor-build-start");
            InstanceProxyContactTest.Stop("house-floor-build-start", false);
            EnvironmentProviderRuntime.Clear("house-floor-build-start");
            return HouseFloorEnvironmentRuntime.StartBuild(_logger);
        }

        public static void UpdateHouseFloorEnvironment()
        {
            HouseFloorEnvironmentRuntime.Update();
        }

        public static string GetHouseFloorEnvironmentStatusText()
        {
            return HouseFloorEnvironmentRuntime.GetStatusText();
        }

        public static string SetHouseFloorEnvironmentVisual(bool visible)
        {
            return HouseFloorEnvironmentRuntime.SetVisual(visible);
        }

        public static string SetHouseFloorEnvironmentColliders(bool enabled)
        {
            if (InstanceProxyContactTest.IsRunning)
                return "受控接触验证正在运行；测试完成或取消后才能切换四房间地板 Collider。";
            return HouseFloorEnvironmentRuntime.SetColliders(enabled);
        }

        public static string RollbackHouseFloorEnvironment()
        {
            InstanceProxyContactTest.Stop("house-floor-provider-rollback", false);
            return HouseFloorEnvironmentRuntime.Rollback();
        }

        public static string GetCurrentInstanceProxyStatusText()
        {
            return SpatialInstanceProxyRuntime.GetStatusText();
        }

        public static string WriteMultipartAggregationReportAndGetConsoleSummary()
        {
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            return MultipartObjectAggregator.WriteFullReportAndGetConsoleSummary();
        }

        public static string BuildCurrentInstanceProxies()
        {
            InstanceProxyContactTest.Stop("proxy-rebuild", false);
            HouseFloorEnvironmentRuntime.Clear("low-level-box-proxy-build");
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_proxy_build");
            StaticSourceMeshProxyRuntime.Clear("box-proxy-build");
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            GameObject target = null;
            if (_multipartAggregationReport != null &&
                !string.IsNullOrWhiteSpace(_multipartAggregationReport.TargetPath))
            {
                target = TargetFinder.FindUniqueExactPath(
                    _multipartAggregationReport.TargetPath,
                    out int ignoredMatches,
                    out string ignoredError);
            }
            return SpatialInstanceProxyRuntime.Build(
                _multipartAggregationReport,
                target,
                _logger);
        }

        public static string SetCurrentInstanceProxyVisual(bool visible)
        {
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_proxy_visual");
            return SpatialInstanceProxyRuntime.SetVisual(visible);
        }

        public static string SetCurrentInstanceProxyColliders(bool enabled)
        {
            if (InstanceProxyContactTest.IsRunning)
                return "受控接触验证正在运行；测试完成或取消后才能手动切换 Collider。";
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_proxy_collider");
            return SpatialInstanceProxyRuntime.SetColliders(enabled);
        }

        public static string RollbackCurrentInstanceProxies()
        {
            InstanceProxyContactTest.Stop("manual-proxy-rollback", false);
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_proxy_rollback");
            return SpatialInstanceProxyRuntime.Rollback();
        }

        public static string BuildCurrentStaticSourceMeshProxy()
        {
            InstanceProxyContactTest.Stop("static-source-mesh-build", false);
            HouseFloorEnvironmentRuntime.Clear("low-level-static-mesh-build");
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_static_mesh_build");
            SpatialInstanceProxyRuntime.Clear("static-source-mesh-build");
            _multipartAggregationReport = MultipartObjectAggregator.CurrentReport;
            GameObject target = null;
            if (_multipartAggregationReport != null &&
                !string.IsNullOrWhiteSpace(_multipartAggregationReport.TargetPath))
            {
                target = TargetFinder.FindUniqueExactPath(
                    _multipartAggregationReport.TargetPath,
                    out int ignoredMatches,
                    out string ignoredError);
            }
            return StaticSourceMeshProxyRuntime.Build(
                _multipartAggregationReport,
                target,
                _logger);
        }

        public static string GetCurrentStaticSourceMeshProxyStatusText()
        {
            return StaticSourceMeshProxyRuntime.GetStatusText();
        }

        public static string SetCurrentStaticSourceMeshVisual(bool visible)
        {
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_static_mesh_visual");
            return StaticSourceMeshProxyRuntime.SetVisual(visible);
        }

        public static string SetCurrentStaticSourceMeshCollider(bool enabled)
        {
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_static_mesh_collider");
            return StaticSourceMeshProxyRuntime.SetCollider(enabled);
        }

        public static string RollbackCurrentStaticSourceMeshProxy()
        {
            EnvironmentProviderRuntime.InvalidateForLowLevelCommand("gsp_instance_static_mesh_rollback");
            return StaticSourceMeshProxyRuntime.Rollback();
        }

        public static string StartInstanceProxyContactTest()
        {
            return InstanceProxyContactTest.Start(_logger);
        }

        public static void UpdateInstanceProxyContactTest()
        {
            InstanceProxyContactTest.Update();
        }

        public static void FixedUpdateInstanceProxyContactTest()
        {
            InstanceProxyContactTest.FixedUpdate();
        }

        public static string GetInstanceProxyContactTestStatusText()
        {
            return InstanceProxyContactTest.GetStatusText();
        }

        public static string GetInstanceProxyContactTestReportText()
        {
            return InstanceProxyContactTest.GetReportText();
        }

        public static string CancelInstanceProxyContactTest()
        {
            return InstanceProxyContactTest.Stop("manual-cancel", true);
        }

        public static string GetInventoryReportText()
        {
            return _inventoryReport == null
                ? "当前没有 GSP 安全清单。先执行 gsp_inventory(对象名称)。"
                : _inventoryReport.ToText(true);
        }

        public static string GetReportText()
        {
            return _report == null
                ? "当前没有 GSP 报告。先执行 gsp_probe(对象名称,auto)。"
                : _report.ToText(true);
        }

        public static string SetPreviewMode(string text)
        {
            if (!ExactMeshPreview.TryParseMode(text, out PreviewMode mode))
                return "用法：gsp_preview(overlay/extracted/source/off)";

            if (_report == null || _report.Snapshots.Count == 0)
                return "<color=yellow>当前没有可显示的精确网格。</color>";

            if (ExactMeshPreview.PreviewObjectCount == 0)
            {
                if (!ExactMeshPreview.Build(
                    _target,
                    _report.Snapshots,
                    out string error))
                {
                    return "<color=red>预览创建失败：</color> " + error;
                }
            }

            ExactMeshPreview.SetMode(mode);
            _logger?.LogInfo(
                "[GSP] Preview mode | mode=" + mode +
                " | objects=" + ExactMeshPreview.PreviewObjectCount);
            return
                "GSP 预览模式=" + mode +
                "，对象数=" + ExactMeshPreview.PreviewObjectCount + "。";
        }

        public static string ExportObj()
        {
            try
            {
                string result = ObjExporter.Export(_report);
                _logger?.LogInfo("[GSP] " + result);
                return result;
            }
            catch (Exception e)
            {
                _logger?.LogError("[GSP] OBJ export failed: " + e);
                return "<color=red>OBJ 导出失败：</color> " + e.GetType().Name + " / " + e.Message;
            }
        }

        private static void ClearAllPreviews()
        {
            ExactMeshPreview.Clear();
            D3D11ExtractedMeshPreview.Clear();
            InstanceProxyContactTest.Stop("runtime-clear-all-previews", false);
            HouseFloorEnvironmentRuntime.Clear("runtime-clear-all-previews");
            EnvironmentProviderRuntime.Clear("runtime-clear-all-previews");
        }

        private static string PointerText(IntPtr pointer)
        {
            return "0x" +
                unchecked((ulong)pointer.ToInt64()).ToString("X");
        }

        public static void Clear(string reason)
        {
            HouseGeometryCensus.Cancel("runtime-clear:" + reason);
            MultipartObjectAggregator.Cancel("runtime-clear:" + reason);
            ClearAllPreviews();
            _report = null;
            _inventoryReport = null;
            _layoutReport = null;
            _handleReport = null;
            _rendererRouteReport = null;
            _d3d11FullSubmeshReadbackReport = null;
            _d3d11MeshPreviewReport = null;
            _d3d11MeshColliderAuditReport = null;
            _d3d11EnvironmentObservationReport = null;
            _houseGeometryCensusReport = null;
            _multipartAggregationReport = null;
            _target = null;
            _logger?.LogInfo("[GSP] Cleared | reason=" + reason);
            // 预留的非 shutdown 清理扩展点当前故意为空。使用显式空代码块，
            // 避免空 if 语句末尾分号在复制或合并时再次丢失并造成编译错误。
            if (reason == null || !reason.StartsWith("shutdown:", StringComparison.Ordinal))
            {
                // reserved for future non-shutdown cleanup diagnostics
            }
        }

        public static void OnSceneChanged(string sceneName, int sceneHandle)
        {
            _sceneName = sceneName;
            _sceneHandle = sceneHandle;
            Clear("scene-change:" + sceneName + "#" + sceneHandle);
        }

        public static void Shutdown(string reason)
        {
            Clear("shutdown:" + reason);
        }
    }
}
