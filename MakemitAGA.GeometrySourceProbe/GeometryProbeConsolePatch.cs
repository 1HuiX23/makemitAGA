using System;
using HarmonyLib;
using MakemitAGA.GeometrySourceProbe.Acquisition;
using MakemitAGA.GeometrySourceProbe.Model;
using MakemitAGA.GeometrySourceProbe.Runtime;

namespace MakemitAGA.GeometrySourceProbe
{
    /// <summary>
    /// 只消费 gsp_ 命令，因此可以与现有 SRL 插件同时存在。
    /// </summary>
    public static class GeometryProbeConsolePatch
    {
        [HarmonyPatch(typeof(ConsoleCommandsGame), "Command")]
        [HarmonyPrefix]
        public static bool InterceptCommand(string code)
        {
            string command = (code ?? string.Empty).Trim();
            if (!command.StartsWith(
                "gsp_",
                StringComparison.OrdinalIgnoreCase))
            {
                return true;
            }

            try
            {
                if (TryHandle(command))
                    return false;
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError(
                    "[GSP] Command failed: " + e);
                PrintGame(
                    "<color=red>GSP 命令失败：</color> " +
                    e.GetType().Name + " / " + e.Message);
                return false;
            }

            PrintGame(
                "<color=yellow>未知 GSP 命令。</color> 输入 gsp_help 查看帮助。");
            return false;
        }

        private static bool TryHandle(string command)
        {
            if (command.Equals("gsp_help", StringComparison.OrdinalIgnoreCase))
            {
                PrintHelp();
                return true;
            }

            if (command.Equals("gsp_multipart_status", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetMultipartAggregationStatusText());
                return true;
            }

            if (command.Equals("gsp_multipart_report", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.WriteMultipartAggregationReportAndGetConsoleSummary());
                return true;
            }

            if (command.Equals("gsp_instance_export_draft", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_export_draft()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.ExportSpatialInstanceDraft());
                return true;
            }

            if (command.Equals("gsp_multipart_cancel", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_multipart_cancel()", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_cancel", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_cancel()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.CancelMultipartAggregationAudit());
                return true;
            }

            if (command.Equals("gsp_physics_whitelist_template", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_physics_whitelist_template()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.EnsurePhysicsWhitelistTemplate());
                return true;
            }

            if (command.Equals("gsp_physics_whitelist_reload", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_physics_whitelist_reload()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.ReloadPhysicsWhitelist());
                return true;
            }

            if (command.Equals("gsp_physics_whitelist_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetPhysicsWhitelistStatusText());
                return true;
            }

            if (command.Equals("gsp_instance_profile_template", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_profile_template()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.EnsureInstanceProfileWhitelistTemplate());
                return true;
            }

            if (command.Equals("gsp_instance_profile_reload", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_profile_reload()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.ReloadInstanceProfileWhitelist());
                return true;
            }

            if (command.Equals("gsp_instance_profile_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetInstanceProfileWhitelistStatusText());
                return true;
            }

            if (command.Equals("gsp_instance_profile_seed_current", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_profile_seed_current()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(
                    "<color=yellow>实例级配置种子：</color> 将按当前已完成的碰撞空间实例写入精确 instanceKey 可编辑配置草案。" +
                    "质量、摩擦、固定状态和代理类型只写配置，不会创建或修改 Unity 物理组件。");
                PrintGame(GeometrySourceProbeRuntime.SeedCurrentInstanceProfiles());
                return true;
            }

            if (command.Equals("gsp_instance_profile_remove_current_seed", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_profile_remove_current_seed()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.RemoveCurrentInstanceProfileSeed());
                return true;
            }

            if (command.Equals("gsp_floor_build", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_floor_build()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(
                    "<color=yellow>GSP v0.2.3 活动房间增量地板 Atlas：</color> 将按固定 House/Room 语义观察 " +
                    "Bedroom→FloorBedroom、Kitchen→FloorKitchen、Main→RoomMain、Toilet→FloorToilet，" +
                    "当前 active 的房间，并在该房间首次出现时复用只读 D3D11 几何管线捕获 floor。" +
                    "未激活房间只记 WAITING_ACTIVE；已捕获 floor 不随源房间失活；Collider 初始关闭、Rigidbody=0；不引用 ARL DLL。");
                PrintGame(GeometrySourceProbeRuntime.StartHouseFloorEnvironmentBuild());
                return true;
            }

            if (command.Equals("gsp_floor_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetHouseFloorEnvironmentStatusText());
                return true;
            }

            if (TryParseCall(command, "gsp_floor_visual", out string[] floorVisualArgs))
            {
                if (!TryParseOnOff(floorVisualArgs, out bool visible))
                {
                    PrintGame("用法：gsp_floor_visual(on/off)");
                    return true;
                }
                PrintGame(GeometrySourceProbeRuntime.SetHouseFloorEnvironmentVisual(visible));
                return true;
            }

            if (TryParseCall(command, "gsp_floor_collider", out string[] floorColliderArgs))
            {
                if (!TryParseOnOff(floorColliderArgs, out bool enabled))
                {
                    PrintGame("用法：gsp_floor_collider(on/off)");
                    return true;
                }
                if (enabled)
                {
                    PrintGame(
                        "<color=yellow>即将开启已捕获的 GSP 地板 Collider：</color> 每块会重新验证 cached provenance、Transform、" +
                        "sharedMesh/convex/trigger、layer、Rigidbody 与 bounds；源房间当前失活不是失败。");
                }
                PrintGame(GeometrySourceProbeRuntime.SetHouseFloorEnvironmentColliders(enabled));
                return true;
            }

            if (command.Equals("gsp_floor_rollback", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_floor_rollback()", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_floor_clear", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_floor_clear()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.RollbackHouseFloorEnvironment());
                return true;
            }

            if (command.Equals("gsp_environment_build", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_environment_build()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(
                    "<color=yellow>GSP v0.2.3 单对象模块化 environment provider（保留）：</color> 将根据当前实例配置自动选择 " +
                    "STATIC_NONCONVEX_MESH 或 Box/Compound 固定代理。只创建 GSP-owned Collider，Collider 初始关闭；" +
                    "不引用或调用 ARL DLL，唯一共享面是 Unity Physics。");
                PrintGame(GeometrySourceProbeRuntime.BuildCurrentEnvironmentProvider());
                return true;
            }

            if (command.Equals("gsp_environment_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetCurrentEnvironmentProviderStatusText());
                return true;
            }

            if (TryParseCall(command, "gsp_environment_visual", out string[] environmentVisualArgs))
            {
                if (!TryParseOnOff(environmentVisualArgs, out bool visible))
                {
                    PrintGame("用法：gsp_environment_visual(on/off)");
                    return true;
                }
                PrintGame(GeometrySourceProbeRuntime.SetCurrentEnvironmentProviderVisual(visible));
                return true;
            }

            if (TryParseCall(command, "gsp_environment_collider", out string[] environmentColliderArgs))
            {
                if (!TryParseOnOff(environmentColliderArgs, out bool enabled))
                {
                    PrintGame("用法：gsp_environment_collider(on/off)");
                    return true;
                }
                if (enabled)
                {
                    PrintGame(
                        "<color=yellow>即将开启 GSP environment Collider：</color> 每次 OFF→ON 都会重新验证 " +
                        "targetPath、Transform、shape/sharedMesh、layer 与 Rigidbody；任一失败都会 fail-close。" );
                }
                PrintGame(GeometrySourceProbeRuntime.SetCurrentEnvironmentProviderCollider(enabled));
                return true;
            }

            if (command.Equals("gsp_environment_rollback", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_environment_rollback()", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_environment_clear", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_environment_clear()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.RollbackCurrentEnvironmentProvider());
                return true;
            }

            if (command.Equals("gsp_instance_proxy_build", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_proxy_build()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(
                    "<color=yellow>实例单盒/复合盒实际创建测试：</color> 只接受显式 enabled=true、fixedToWorld=true、pickable=false 的碰撞实例规则。" +
                    "ORIENTED_THIN_BOX/ORIENTED_BOX 创建单盒；COMPOUND_BOX 按草案创建多个子盒。Collider 初始保持关闭，不创建 Rigidbody，也不修改源对象。");
                PrintGame(GeometrySourceProbeRuntime.BuildCurrentInstanceProxies());
                return true;
            }

            if (command.Equals("gsp_instance_proxy_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetCurrentInstanceProxyStatusText());
                return true;
            }

            if (TryParseCall(command, "gsp_instance_proxy_visual", out string[] proxyVisualArgs))
            {
                if (!TryParseOnOff(proxyVisualArgs, out bool visible))
                {
                    PrintGame("用法：gsp_instance_proxy_visual(on/off)");
                    return true;
                }
                PrintGame(GeometrySourceProbeRuntime.SetCurrentInstanceProxyVisual(visible));
                return true;
            }

            if (TryParseCall(command, "gsp_instance_proxy_collider", out string[] proxyColliderArgs))
            {
                if (!TryParseOnOff(proxyColliderArgs, out bool enabled))
                {
                    PrintGame("用法：gsp_instance_proxy_collider(on/off)");
                    return true;
                }
                if (enabled)
                {
                    PrintGame(
                        "<color=yellow>即将开启 GSP 静态 BoxCollider：</color> 不会创建 Rigidbody，" +
                        "但代理将开始参与真实物理碰撞。可随时执行 gsp_instance_proxy_collider(off) 或 gsp_instance_proxy_rollback。");
                }
                PrintGame(GeometrySourceProbeRuntime.SetCurrentInstanceProxyColliders(enabled));
                return true;
            }

            if (command.Equals("gsp_instance_proxy_contact_test", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_proxy_contact_test()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(
                    "<color=yellow>实例代理受控接触验证：</color> 将临时开启当前 GSP 单盒/复合盒 Collider，" +
                    "依次从正面中心、侧面中心和正面边缘发射小型无重力刚体球。每次只保留一个测试球，" +
                    "仅接受与目标代理 BoxCollider 匹配的真实 OnCollisionEnter；完成或取消后恢复原 Collider 状态。");
                PrintGame(GeometrySourceProbeRuntime.StartInstanceProxyContactTest());
                return true;
            }

            if (command.Equals("gsp_instance_proxy_contact_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetInstanceProxyContactTestStatusText());
                return true;
            }

            if (command.Equals("gsp_instance_proxy_contact_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetInstanceProxyContactTestReportText());
                return true;
            }

            if (command.Equals("gsp_instance_proxy_contact_cancel", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_proxy_contact_cancel()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.CancelInstanceProxyContactTest());
                return true;
            }

            if (command.Equals("gsp_instance_proxy_rollback", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_proxy_rollback()", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_proxy_clear", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_proxy_clear()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.RollbackCurrentInstanceProxies());
                return true;
            }

            if (command.Equals("gsp_instance_static_mesh_build", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_static_mesh_build()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(
                    "<color=yellow>固定复杂家具静态源网格创建：</color> 只接受显式 STATIC_NONCONVEX_MESH、" +
                    "fixedToWorld=true、pickable=false 的唯一碰撞实例。将使用最近一次成功审计保存的世界空间碰撞聚合几何，" +
                    "创建 GSP 独立半透明 Mesh 与静态非凸 MeshCollider。不会创建 Rigidbody，不修改源对象，Collider 初始关闭。");
                PrintGame(GeometrySourceProbeRuntime.BuildCurrentStaticSourceMeshProxy());
                return true;
            }

            if (command.Equals("gsp_instance_static_mesh_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetCurrentStaticSourceMeshProxyStatusText());
                return true;
            }

            if (TryParseCall(command, "gsp_instance_static_mesh_visual", out string[] staticMeshVisualArgs))
            {
                if (!TryParseOnOff(staticMeshVisualArgs, out bool visible))
                {
                    PrintGame("用法：gsp_instance_static_mesh_visual(on/off)");
                    return true;
                }
                PrintGame(GeometrySourceProbeRuntime.SetCurrentStaticSourceMeshVisual(visible));
                return true;
            }

            if (TryParseCall(command, "gsp_instance_static_mesh_collider", out string[] staticMeshColliderArgs))
            {
                if (!TryParseOnOff(staticMeshColliderArgs, out bool enabled))
                {
                    PrintGame("用法：gsp_instance_static_mesh_collider(on/off)");
                    return true;
                }
                if (enabled)
                {
                    PrintGame(
                        "<color=yellow>即将开启静态非凸 MeshCollider：</color> 该代理无 Rigidbody，" +
                        "用于固定环境被动碰撞。可随时关闭或执行 gsp_instance_static_mesh_rollback。");
                }
                PrintGame(GeometrySourceProbeRuntime.SetCurrentStaticSourceMeshCollider(enabled));
                return true;
            }

            if (command.Equals("gsp_instance_static_mesh_rollback", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_static_mesh_rollback()", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_static_mesh_clear", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_instance_static_mesh_clear()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.RollbackCurrentStaticSourceMeshProxy());
                return true;
            }

            if (TryParseCall(command, "gsp_instance_audit", out string[] instanceArgs))
            {
                if (instanceArgs.Length != 1 || string.IsNullOrWhiteSpace(instanceArgs[0]))
                {
                    PrintGame("用法：gsp_instance_audit(对象名称)，例如 gsp_instance_audit(TargetObject)");
                    return true;
                }
                PrintGame(
                    "<color=yellow>空间实例/连通组件审计：</color> 将先完成多路由读取和世界坐标统一，" +
                    "再按空间焊接共享边拆组件、按世界 AABB 距离归并现实物体实例候选，" +
                    "并为碰撞实例计算 PCA 定向盒代理草案。不会创建 Collider 或 Rigidbody。");
                PrintGame(GeometrySourceProbeRuntime.StartMultipartAggregationAudit(instanceArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(command, "gsp_multipart_audit", out string[] multipartArgs))
            {
                if (multipartArgs.Length != 1 || string.IsNullOrWhiteSpace(multipartArgs[0]))
                {
                    PrintGame("用法：gsp_multipart_audit(对象名称)，例如 gsp_multipart_audit(TargetObject)");
                    return true;
                }
                PrintGame(
                    "<color=yellow>多部件聚合审计：</color> 将逐 Renderer/逐路由读取，" +
                    "自动区分实体、视觉画面和阴影特效，并输出原始/空间焊接拓扑及空间实例候选。不会创建 Collider 或 Rigidbody。");
                PrintGame(GeometrySourceProbeRuntime.StartMultipartAggregationAudit(multipartArgs[0].Trim()));
                return true;
            }

            if (command.Equals("gsp_house_census_status", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetHouseGeometryCensusStatusText());
                return true;
            }

            if (command.Equals("gsp_house_census_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetHouseGeometryCensusReportText());
                return true;
            }

            if (command.Equals("gsp_house_census_cancel", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_house_census_cancel()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.CancelHouseGeometryCensus());
                return true;
            }

            if (command.Equals("gsp_house_census_start", StringComparison.OrdinalIgnoreCase) ||
                command.Equals("gsp_house_census_start()", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(
                    "<color=yellow>第一章四房间几何普查：</color> 将定位 Bedroom、Kitchen、Main、Toilet，" +
                    "按房间下一层物体根节点逐帧做只读分类。不会创建 MeshCollider/Rigidbody；多 Renderer 或多子网格对象只标记为需要聚合，不会误报为水密。完整逐对象结果写入 LogOutput。");
                PrintGame(GeometrySourceProbeRuntime.StartHouseGeometryCensus(null));
                return true;
            }

            if (TryParseCall(command, "gsp_house_census_start", out string[] houseCensusArgs))
            {
                if (houseCensusArgs.Length != 1 || string.IsNullOrWhiteSpace(houseCensusArgs[0]))
                {
                    PrintGame("用法：gsp_house_census_start 或 gsp_house_census_start(Bedroom/Kitchen/Main/Toilet/all)");
                    return true;
                }
                PrintGame(
                    "<color=yellow>房间定向几何普查：</color> room=" + houseCensusArgs[0].Trim() +
                    "。只读、逐帧执行，不创建任何 Collider。完整逐对象结果写入 LogOutput。");
                PrintGame(GeometrySourceProbeRuntime.StartHouseGeometryCensus(houseCensusArgs[0].Trim()));
                return true;
            }

            if (command.Equals("gsp_inventory_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetInventoryReportText());
                return true;
            }

            if (command.Equals("gsp_layout_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetLayoutReportText());
                return true;
            }

            if (command.Equals("gsp_handle_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetHandleReportText());
                return true;
            }

            if (command.Equals("gsp_renderer_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetRendererRouteReportText());
                return true;
            }

            if (command.Equals("gsp_d3d11_submesh_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetD3D11FullSubmeshReportText());
                return true;
            }

            if (command.Equals("gsp_d3d11_mesh_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetD3D11MeshPreviewReportText());
                return true;
            }

            if (command.Equals("gsp_d3d11_collider_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetD3D11MeshColliderAuditReportText());
                return true;
            }

            if (command.Equals("gsp_d3d11_observation_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetD3D11ObservationFoundationReportText());
                return true;
            }

            if (command.Equals("gsp_d3d11_mesh_clear", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.ClearD3D11MeshPreview());
                return true;
            }

            if (command.Equals("gsp_d3d11_collider_clear", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.ClearD3D11MeshColliderAudit());
                return true;
            }

            if (command.Equals("gsp_report", StringComparison.OrdinalIgnoreCase))
            {
                PrintGame(GeometrySourceProbeRuntime.GetReportText());
                return true;
            }

            if (command.Equals("gsp_clear", StringComparison.OrdinalIgnoreCase))
            {
                GeometrySourceProbeRuntime.Clear("manual-clear");
                PrintGame("GSP 已清理当前报告、预览和临时 Mesh。");
                return true;
            }

            if (command.Equals("gsp_export_obj", StringComparison.OrdinalIgnoreCase))
            {
                string result = GeometrySourceProbeRuntime.ExportObj();
                PrintGame(result);
                return true;
            }

            if (TryParseCall(
                command,
                "gsp_d3d11_submesh",
                out string[] d3d11SubmeshArgs))
            {
                if (d3d11SubmeshArgs.Length != 1 ||
                    string.IsNullOrWhiteSpace(d3d11SubmeshArgs[0]))
                {
                    PrintGame(
                        "用法：gsp_d3d11_submesh(对象完整名称)");
                    return true;
                }

                PrintGame(
                    "<color=yellow>D3D11 完整子网格验证：</color> 本命令会读取 Renderer 路由子网格的完整索引区间和完整顶点区间，" +
                    "统计索引范围、引用顶点、Position、bounds 与退化三角形；不会创建新 Mesh、不会修改源 buffer target。");
                PrintGame(
                    GeometrySourceProbeRuntime.ProbeD3D11FullSubmesh(
                        d3d11SubmeshArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(
                command,
                "gsp_d3d11_observation",
                out string[] d3d11ObservationArgs))
            {
                if (d3d11ObservationArgs.Length != 1 ||
                    string.IsNullOrWhiteSpace(d3d11ObservationArgs[0]))
                {
                    PrintGame(
                        "用法：gsp_d3d11_observation(对象完整名称)");
                    return true;
                }

                PrintGame(
                    "<color=yellow>D3D11 环境观测基础：</color> 本命令会建立经过验证的世界空间 Mesh 与静态 MeshCollider，" +
                    "并输出一个不依赖 ML-Agents 程序集的中立观测 DTO。它保留 bounds、三角形表面查询、接触点/法线/冲量通道，" +
                    "供未来 Active Ragdoll、ML-Agents 和 VLM 目标点适配层使用。不会在本版决定最终神经网络向量长度。");
                PrintGame(
                    GeometrySourceProbeRuntime.BuildD3D11ObservationFoundation(
                        d3d11ObservationArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(
                command,
                "gsp_d3d11_collider",
                out string[] d3d11ColliderArgs))
            {
                if (d3d11ColliderArgs.Length != 1 ||
                    string.IsNullOrWhiteSpace(d3d11ColliderArgs[0]))
                {
                    PrintGame(
                        "用法：gsp_d3d11_collider(对象完整名称)");
                    return true;
                }

                PrintGame(
                    "<color=yellow>D3D11 静态 MeshCollider 烹饪审计：</color> 本命令会先完整读回并创建独立 Mesh 副本，" +
                    "随后只在该 GSP 副本上添加一个非凸、非 Trigger、无 Rigidbody 的 MeshCollider，" +
                    "调用 Physics.SyncTransforms 并比较 Collider/Preview/源 Renderer bounds。不会修改原对象，也不会执行动态接触。");
                PrintGame(
                    GeometrySourceProbeRuntime.BuildD3D11MeshColliderAudit(
                        d3d11ColliderArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(
                command,
                "gsp_d3d11_mesh_mode",
                out string[] d3d11MeshModeArgs))
            {
                if (d3d11MeshModeArgs.Length != 1 ||
                    string.IsNullOrWhiteSpace(d3d11MeshModeArgs[0]))
                {
                    PrintGame(
                        "用法：gsp_d3d11_mesh_mode(overlay/extracted/source/off)");
                    return true;
                }

                PrintGame(
                    GeometrySourceProbeRuntime.SetD3D11MeshPreviewMode(
                        d3d11MeshModeArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(
                command,
                "gsp_d3d11_mesh",
                out string[] d3d11MeshArgs))
            {
                if (d3d11MeshArgs.Length != 1 ||
                    string.IsNullOrWhiteSpace(d3d11MeshArgs[0]))
                {
                    PrintGame(
                        "用法：gsp_d3d11_mesh(对象完整名称)");
                    return true;
                }

                PrintGame(
                    "<color=yellow>D3D11 独立可视化副本：</color> 本命令会完整读回 Renderer 路由子网格，" +
                    "创建半透明、可删除的独立 Unity Mesh 与原物体叠加；不会添加 Collider、不会隐藏或修改原对象。");
                PrintGame(
                    GeometrySourceProbeRuntime.BuildD3D11MeshPreview(
                        d3d11MeshArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(command, "gsp_layout", out string[] layoutArgs))
            {
                if (layoutArgs.Length != 1 || string.IsNullOrWhiteSpace(layoutArgs[0]))
                {
                    PrintGame("用法：gsp_layout(对象完整名称)");
                    return true;
                }

                PrintGame(GeometrySourceProbeRuntime.Layout(layoutArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(command, "gsp_handle", out string[] handleArgs))
            {
                if (handleArgs.Length != 2 ||
                    string.IsNullOrWhiteSpace(handleArgs[0]) ||
                    string.IsNullOrWhiteSpace(handleArgs[1]))
                {
                    PrintGame("用法：gsp_handle(对象完整名称,vertex0/index)");
                    return true;
                }

                PrintGame(
                    "<color=yellow>注意：</color> 该命令会调用一次现有 GPU buffer getter，" +
                    "但不会修改 target，也不会读取数据。");
                PrintGame(GeometrySourceProbeRuntime.ProbeHandle(
                    handleArgs[0].Trim(),
                    handleArgs[1].Trim()));
                return true;
            }

            if (TryParseCall(command, "gsp_renderer", out string[] rendererArgs))
            {
                if (rendererArgs.Length != 1 || string.IsNullOrWhiteSpace(rendererArgs[0]))
                {
                    PrintGame("用法：gsp_renderer(对象完整名称)");
                    return true;
                }

                PrintGame(GeometrySourceProbeRuntime.InspectRendererRoute(
                    rendererArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(command, "gsp_inventory", out string[] inventoryArgs))
            {
                if (inventoryArgs.Length != 1 || string.IsNullOrWhiteSpace(inventoryArgs[0]))
                {
                    PrintGame("用法：gsp_inventory(对象完整名称)");
                    return true;
                }

                PrintGame(GeometrySourceProbeRuntime.Inventory(inventoryArgs[0].Trim()));
                return true;
            }

            if (TryParseCall(command, "gsp_probe", out string[] probeArgs))
            {
                if (probeArgs.Length < 1 || string.IsNullOrWhiteSpace(probeArgs[0]))
                {
                    PrintGame("用法：gsp_probe(对象完整名称,auto)");
                    return true;
                }

                ProbePreference preference = ProbePreference.Auto;
                if (probeArgs.Length >= 2 &&
                    !ProbePreferenceParser.TryParse(probeArgs[1], out preference))
                {
                    PrintGame(
                        "来源只能是 auto、cpu、meshdata、bake、gpu 或 collider。");
                    return true;
                }

                string result = GeometrySourceProbeRuntime.Probe(
                    probeArgs[0].Trim(),
                    preference);
                PrintGame(result);
                return true;
            }

            if (TryParseCall(command, "gsp_preview", out string[] previewArgs))
            {
                if (previewArgs.Length != 1)
                {
                    PrintGame(
                        "用法：gsp_preview(overlay/extracted/source/off)");
                    return true;
                }

                string result = GeometrySourceProbeRuntime.SetPreviewMode(
                    previewArgs[0]);
                PrintGame(result);
                return true;
            }

            return false;
        }

        private static bool TryParseOnOff(string[] args, out bool value)
        {
            value = false;
            if (args == null || args.Length != 1)
                return false;
            string text = (args[0] ?? string.Empty).Trim();
            if (text.Equals("on", StringComparison.OrdinalIgnoreCase) ||
                text.Equals("true", StringComparison.OrdinalIgnoreCase) ||
                text == "1")
            {
                value = true;
                return true;
            }
            if (text.Equals("off", StringComparison.OrdinalIgnoreCase) ||
                text.Equals("false", StringComparison.OrdinalIgnoreCase) ||
                text == "0")
            {
                value = false;
                return true;
            }
            return false;
        }

        private static void PrintHelp()
        {
            PrintGame(
                "<color=#66ddff>Geometry Source Probe 0.2.3 — 活动房间增量地板 Atlas 基础版</color>\n" +
                "gsp_instance_audit(对象)          读取多路由几何、统一世界坐标、拆分组件与现实实例\n" +
                "gsp_instance_status               查看审计进度与摘要\n" +
                "gsp_instance_report               游戏内显示短摘要；完整报告写入 config 文本文件\n" +
                "gsp_instance_export_draft         导出稳定 instanceKey、位置与代理草案\n" +
                "gsp_instance_cancel               中止当前实例审计\n" +
                "gsp_physics_whitelist_template    创建部件角色/质量/摩擦白名单模板\n" +
                "gsp_physics_whitelist_reload      热重载部件白名单\n" +
                "gsp_physics_whitelist_status      查看部件白名单状态；空配置保持为空，不注入对象规则\n" +
                "gsp_instance_profile_template     创建实例级固定/可拿取/代理类型配置模板\n" +
                "gsp_instance_profile_reload       热重载实例配置\n" +
                "gsp_instance_profile_status       查看实例配置状态\n" +
                "gsp_instance_profile_seed_current 按当前 instanceKey 生成可编辑配置草案\n" +
                "gsp_instance_profile_remove_current_seed 删除当前对象自动生成的配置草案块\n" +
                "gsp_floor_build                    ARL地面主线：启动active-room watcher；当前房间先捕获，其余房间访问时自动补捕获\n" +
                "gsp_floor_status                   查看CAPTURED/WAITING_ACTIVE/FAILED、sourceActive与持久provider真值\n" +
                "gsp_floor_visual(on/off)           显示/隐藏当前已捕获地板；future capture自动继承该状态\n" +
                "gsp_floor_collider(on/off)         切换当前已捕获地板碰撞；每次ON重新fail-close，源房间失活不阻止cached floor\n" +
                "gsp_floor_rollback                 独立回滚四块GSP-owned地板Mesh/Collider/Material\n" +
                "gsp_environment_build              正式入口：按实例配置自动创建固定环境 provider（Mesh 或 Box/Compound）\n" +
                "gsp_environment_status             查看 targetPath 身份、layer、provider Collider/Rigidbody 与源原生碰撞摘要\n" +
                "gsp_environment_visual(on/off)     控制当前 provider 可视化\n" +
                "gsp_environment_collider(on/off)   控制真实环境碰撞；每次开启都会重新 fail-close 预检\n" +
                "gsp_environment_rollback           独立回滚当前 GSP environment provider\n" +
                "以下 instance_proxy / instance_static_mesh 命令保留为低层诊断；Feet/Ground-Contact 正式组合优先使用 gsp_floor_*，单家具仍可用 gsp_environment_*。\n" +
                "gsp_instance_proxy_build          创建半透明单盒/复合盒与默认关闭的静态 BoxCollider\n" +
                "gsp_instance_proxy_status         查看代理数量、安全状态和实例键\n" +
                "gsp_instance_proxy_visual(on/off) 独立控制代理可视化\n" +
                "gsp_instance_proxy_collider(on/off) 独立控制代理真实碰撞\n" +
                "gsp_instance_proxy_contact_test   对每个代理执行真实碰撞 + 托管OBB表面点/法线验证\n" +
                "gsp_instance_proxy_contact_status 查看接触测试进度\n" +
                "gsp_instance_proxy_contact_report 查看逐项事件、OBB表面点、法线门槛和耗时\n" +
                "gsp_instance_proxy_contact_cancel 取消测试并恢复 Collider 状态\n" +
                "gsp_instance_proxy_rollback       删除全部 GSP 盒代理并回滚\n" +
                "gsp_instance_static_mesh_build     从当前碰撞聚合快照创建静态非凸源网格代理\n" +
                "gsp_instance_static_mesh_status    查看 Mesh cooking、bounds、拓扑与安全状态\n" +
                "gsp_instance_static_mesh_visual(on/off) 控制精确源网格半透明预览\n" +
                "gsp_instance_static_mesh_collider(on/off) 控制静态非凸 MeshCollider\n" +
                "gsp_instance_static_mesh_rollback  删除静态源网格代理、Mesh 与材质\n" +
                "v0.2.3 正式组合边界：GSP 与 ARL 继续分别编译为独立 DLL；source room activation 与 GSP floor provider lifecycle 分离；GSP 不引用 ARL，消费者只通过 Unity Physics 使用 provider。\n" +
                "gsp_house_census_start(...)       保留的房间几何普查工具\n" +
                "gsp_d3d11_submesh(对象)           保留的完整 D3D11 子网格底层诊断\n" +
                "gsp_d3d11_observation(对象)       建立中立环境观测 DTO\n" +
                "gsp_clear                         清理全部 GSP 临时对象与报告\n" +
                "备注：超长详细报告不得直接输出到游戏控制台；Unity 文本会生成文字 Mesh，" +
                "历史上曾触发 65000 顶点上限。当前统一采用短摘要 + 文件导出，并保留全局输出长度安全门。"
            );
        }

        private static bool TryParseCall(
            string command,
            string verb,
            out string[] args)
        {
            args = Array.Empty<string>();
            if (!command.StartsWith(
                verb,
                StringComparison.OrdinalIgnoreCase))
            {
                return false;
            }

            string tail = command.Substring(verb.Length).Trim();
            if (tail.Length < 2 || tail[0] != '(' || tail[tail.Length - 1] != ')')
                return false;

            string inner = tail.Substring(1, tail.Length - 2);
            args = inner.Split(',');
            for (int i = 0; i < args.Length; i++)
                args[i] = args[i].Trim();
            return true;
        }

        private const int MaxGameConsoleCharacters = 6000;
        private const int SafePrefixCharacters = 5200;

        /// <summary>
        /// 所有游戏内控制台输出的统一安全门。Unity 控制台文本会生成文字 Mesh；
        /// 单次超长文本可能触发 65000 顶点限制，因此绝不把超预算全文直接交给 UI。
        /// 完整内容仍写入 BepInEx 日志，调用者可另外提供独立报告文件。
        /// </summary>
        internal static void PrintGame(string text)
        {
            string value = text ?? string.Empty;
            if (value.Length > MaxGameConsoleCharacters)
            {
                Plugin.Logger?.LogWarning(
                    "[GSP][GameConsole] output-over-budget" +
                    " | chars=" + value.Length +
                    " | budget=" + MaxGameConsoleCharacters +
                    " | action=truncate-game-ui-and-preserve-bepinex-log");
                Plugin.Logger?.LogInfo(
                    "[GSP][GameConsole][FULL_OUTPUT]\n" + value);

                int prefixLength = Math.Min(SafePrefixCharacters, value.Length);
                value = value.Substring(0, prefixLength) +
                    "\n<color=yellow>游戏内输出因超过安全长度已自动截断。完整内容已写入 BepInEx 日志；实例报告请查看独立报告文件。</color>";
            }

            try { ConsoleMain.ConsolePrintGame(value); }
            catch
            {
                Plugin.Logger?.LogInfo(
                    "[GSP][GameConsole] " + value);
            }
        }
    }
}
