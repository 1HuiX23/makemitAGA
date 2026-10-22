using System;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Acquisition;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    /// <summary>
    /// GSP v0.2.1 统一固定环境提供者。
    ///
    /// 设计边界：
    /// - 仍然只生成 MakemitAGA.GeometrySourceProbe.dll；不引用 ARL DLL、不调用 ARL 命令/类型；
    /// - 根据实例配置在既有 STATIC_NONCONVEX_MESH 与 Box/Compound runtime 中二选一；
    /// - 对外只暴露 GSP-owned static Collider 的 build/status/visual/collider/rollback 生命周期；
    /// - 唯一共享面是 Unity Scene / Physics。消费者如何使用 Collider 由消费者自己决定。
    /// </summary>
    internal static class EnvironmentProviderRuntime
    {
        private const string BackendStaticMesh = "STATIC_NONCONVEX_MESH";
        private const string BackendBox = "BOX_OR_COMPOUND";

        private static ManualLogSource _logger;
        private static EnvironmentProviderReport _report;

        public static EnvironmentProviderReport CurrentReport => _report;
        public static bool Exists =>
            StaticSourceMeshProxyRuntime.Exists || SpatialInstanceProxyRuntime.Exists;

        public static string Build(
            MultipartAggregationReport source,
            ManualLogSource logger)
        {
            _logger = logger;
            string previousCleanupError = ClearOwnedBackends("environment-rebuild");
            _report = null;

            _report = new EnvironmentProviderReport
            {
                Started = true,
                TargetName = source != null ? source.TargetName : null,
                TargetPath = source != null ? source.TargetPath : null,
                Detail = "validating-provider-candidate"
            };

            if (!string.IsNullOrEmpty(previousCleanupError))
                return Fail("前一份 environment provider 清理出现异常，拒绝建立新的正式 provider：" + previousCleanupError);

            if (source == null || !source.Completed || !source.Success)
                return Fail("当前没有已完成且成功的实例审计。请先执行 gsp_instance_audit(对象名称)。");
            if (source.CollisionSpatialInstances == null ||
                !source.CollisionSpatialInstances.Analyzed ||
                source.CollisionSpatialInstances.Instances.Count == 0)
            {
                return Fail("当前审计没有 collision spatial instance。拒绝创建空环境 provider。");
            }

            GameObject target = TargetFinder.FindUniqueExactPath(
                source.TargetPath,
                out int exactMatches,
                out string bindError);
            _report.TargetPathMatches = exactMatches;
            _report.TargetIdentityValid = target != null && exactMatches == 1;
            if (target == null)
                return Fail("targetPath 唯一重绑定失败：" + bindError);

            _report.TargetLayer = target.layer;
            AuditSourceNativePhysics(target);

            if (!TryChooseBackend(source.CollisionSpatialInstances, out string backend, out string chooseError))
                return Fail(chooseError);
            _report.Backend = backend;

            string result;
            if (string.Equals(backend, BackendStaticMesh, StringComparison.Ordinal))
            {
                SpatialInstanceProxyRuntime.Clear("environment-static-mesh-build");
                result = StaticSourceMeshProxyRuntime.Build(source, target, logger);
                _report.Success =
                    StaticSourceMeshProxyRuntime.CurrentReport != null &&
                    StaticSourceMeshProxyRuntime.CurrentReport.Success &&
                    StaticSourceMeshProxyRuntime.Exists;
            }
            else
            {
                StaticSourceMeshProxyRuntime.Clear("environment-box-build");
                result = SpatialInstanceProxyRuntime.Build(source, target, logger);
                _report.Success =
                    SpatialInstanceProxyRuntime.CurrentReport != null &&
                    SpatialInstanceProxyRuntime.CurrentReport.Success &&
                    SpatialInstanceProxyRuntime.Exists;
            }

            RefreshRuntimeTruth();
            _report.Completed = true;
            if (!_report.Success || !_report.TargetIdentityValid || !_report.LayerIdentityValid || _report.ProviderRigidbodies != 0)
            {
                string reason = "backend build/readback 未形成完整固定环境 provider：" + _report.ToStatusText();
                ClearOwnedBackends("environment-build-not-accepted");
                _report.Success = false;
                _report.Detail = reason;
                _logger?.LogWarning("[GSP][ENVIRONMENT_PROVIDER] build-rejected | " + _report.ToStatusText());
                return "<color=red>GSP environment provider 创建未通过安全门。</color> " + reason;
            }

            _report.Detail = "provider-ready-collider-intentionally-off";
            _logger?.LogWarning("[GSP][ENVIRONMENT_PROVIDER] build-complete | " + _report.ToStatusText());
            return
                "<color=#66ddff>GSP 固定环境 provider 已创建。</color> backend=" + backend +
                "，targetLayer=" + _report.TargetLayer +
                "，providerColliders=" + _report.ProviderColliders +
                "，Rigidbody=0，Collider 初始关闭。" +
                "两个模块无需程序集引用；后续消费者只通过 Unity Physics 使用该环境代理。";
        }

        public static string GetStatusText()
        {
            if (_report == null)
                return "当前没有 GSP environment provider。先完成 gsp_instance_audit(...)，再执行 gsp_environment_build。";
            RefreshRuntimeTruth();
            return _report.ToStatusText();
        }

        public static string SetVisual(bool visible)
        {
            if (_report == null || !_report.Success)
                return "当前没有通过安全门的 GSP environment provider。";
            string result = IsStaticBackend()
                ? StaticSourceMeshProxyRuntime.SetVisual(visible)
                : SpatialInstanceProxyRuntime.SetVisual(visible);
            RefreshRuntimeTruth();
            return result;
        }

        public static string SetCollider(bool enabled)
        {
            if (_report == null || !_report.Success)
                return "当前没有通过安全门的 GSP environment provider。";

            // 每次 enable 前再次确认完整 targetPath 仍唯一，避免场景运行过程中身份漂移。
            GameObject rebound = TargetFinder.FindUniqueExactPath(
                _report.TargetPath,
                out int matches,
                out string bindError);
            _report.TargetPathMatches = matches;
            _report.TargetIdentityValid = rebound != null && matches == 1;
            if (enabled && !_report.TargetIdentityValid)
            {
                ForceBackendOff();
                RefreshRuntimeTruth();
                return "<color=red>拒绝启用 environment Collider：</color> targetPath 身份已失效：" + bindError;
            }

            string result = IsStaticBackend()
                ? StaticSourceMeshProxyRuntime.SetCollider(enabled)
                : SpatialInstanceProxyRuntime.SetColliders(enabled);
            RefreshRuntimeTruth();

            if (enabled && (!_report.ProviderCollidersEnabled || !_report.LayerIdentityValid || _report.ProviderRigidbodies != 0))
            {
                ForceBackendOff();
                RefreshRuntimeTruth();
                return "<color=red>environment Collider 启用后统一读回失败，已 fail-close。</color> " + _report.ToStatusText();
            }
            return result;
        }

        public static string Rollback()
        {
            bool existed = Exists || _report != null;
            string errors = ClearOwnedBackends("environment-manual-rollback");
            if (_report != null)
            {
                _report.ProviderColliders = 0;
                _report.ProviderCollidersEnabled = false;
                _report.ProviderRigidbodies = 0;
                _report.Success = false;
                _report.Detail = string.IsNullOrEmpty(errors)
                    ? "rollback-requested-without-exception"
                    : "rollback-error:" + errors;
            }
            return !existed
                ? "当前没有需要回滚的 GSP environment provider。"
                : string.IsNullOrEmpty(errors)
                    ? "<color=#66ddff>GSP environment provider 已请求完整回滚。</color> GSP-owned Collider/预览/Mesh/Material 均已进入清理；源对象未修改。"
                    : "<color=red>GSP environment provider 回滚遇到异常：</color> " + errors;
        }

        public static void Clear(string reason)
        {
            Clear(reason, true);
        }

        public static void InvalidateForLowLevelCommand(string reason)
        {
            if (_report == null)
                return;
            _report.Success = false;
            _report.Detail = "provider-invalidated-by-low-level-command:" + reason;
        }

        private static void Clear(string reason, bool discardReport)
        {
            string errors = ClearOwnedBackends(reason);
            if (!string.IsNullOrEmpty(errors))
                _logger?.LogWarning("[GSP][ENVIRONMENT_PROVIDER] cleanup-error | reason=" + reason + " | " + errors);
            if (discardReport)
                _report = null;
        }

        private static string ClearOwnedBackends(string reason)
        {
            string errors = null;
            try
            {
                SpatialInstanceProxyRuntime.Clear(reason);
                if (!string.IsNullOrEmpty(SpatialInstanceProxyRuntime.LastCleanupError))
                    errors = Append(errors, "box:" + SpatialInstanceProxyRuntime.LastCleanupError);
            }
            catch (Exception e) { errors = Append(errors, "box:" + e.GetType().Name + ":" + e.Message); }
            try
            {
                StaticSourceMeshProxyRuntime.Clear(reason);
                if (!string.IsNullOrEmpty(StaticSourceMeshProxyRuntime.LastCleanupError))
                    errors = Append(errors, "mesh:" + StaticSourceMeshProxyRuntime.LastCleanupError);
            }
            catch (Exception e) { errors = Append(errors, "mesh:" + e.GetType().Name + ":" + e.Message); }
            return errors;
        }

        private static bool TryChooseBackend(
            SpatialInstanceAuditReport collision,
            out string backend,
            out string error)
        {
            backend = null;
            error = null;
            int staticMesh = 0;
            int box = 0;
            int unresolved = 0;
            int enabledUnsupported = 0;

            for (int i = 0; i < collision.Instances.Count; i++)
            {
                SpatialInstanceReport instance = collision.Instances[i];
                SpatialInstancePhysicsProfileSnapshot profile = instance != null ? instance.PhysicsProfile : null;
                if (instance == null || profile == null || !profile.Matched || !profile.EnabledConfigured)
                {
                    unresolved++;
                    continue;
                }
                if (!profile.Enabled)
                    continue; // 显式 enabled=false 允许作为人工排除；未配置/未匹配则不能静默遗漏。
                if (!profile.FixedToWorldConfigured || !profile.FixedToWorld ||
                    !profile.PickableConfigured || profile.Pickable ||
                    !profile.ProxyTypeConfigured)
                {
                    enabledUnsupported++;
                    continue;
                }

                string type = profile.ProxyType ?? string.Empty;
                if (string.Equals(type, BackendStaticMesh, StringComparison.OrdinalIgnoreCase))
                    staticMesh++;
                else if (string.Equals(type, "ORIENTED_THIN_BOX", StringComparison.OrdinalIgnoreCase) ||
                         string.Equals(type, "ORIENTED_BOX", StringComparison.OrdinalIgnoreCase) ||
                         string.Equals(type, "COMPOUND_BOX", StringComparison.OrdinalIgnoreCase))
                    box++;
                else
                    enabledUnsupported++;
            }

            if (unresolved != 0)
            {
                error = "存在 " + unresolved + " 个 collision instance 没有显式匹配 enabled 配置；正式 environment provider 不允许静默构建部分环境。请先补齐实例配置，或显式 enabled=false 排除。";
                return false;
            }
            if (enabledUnsupported != 0)
            {
                error = "存在 " + enabledUnsupported + " 个 enabled collision instance 不满足 v0.2.1 固定环境 provider 契约（必须 fixedToWorld=true、pickable=false 且使用已支持 proxyType）。";
                return false;
            }
            if (staticMesh > 0 && box > 0)
            {
                error = "v0.2.1 暂不允许同一目标混合 STATIC_NONCONVEX_MESH 与 Box/Compound provider；请在实例配置中选择单一固定代理族。";
                return false;
            }
            if (staticMesh > 0)
            {
                if (collision.Instances.Count != 1 || staticMesh != 1)
                {
                    error = "STATIC_NONCONVEX_MESH provider 当前要求 collision spatial instance 恰好为 1。";
                    return false;
                }
                backend = BackendStaticMesh;
                return true;
            }
            if (box > 0)
            {
                backend = BackendBox;
                return true;
            }

            error = "没有找到显式启用且满足固定环境契约的 collision instance profile。";
            return false;
        }

        private static void RefreshRuntimeTruth()
        {
            if (_report == null)
                return;

            GameObject target = null;
            try
            {
                target = TargetFinder.FindUniqueExactPath(
                    _report.TargetPath,
                    out int matches,
                    out string ignoredError);
                _report.TargetPathMatches = matches;
                _report.TargetIdentityValid = target != null && matches == 1;
            }
            catch
            {
                _report.TargetPathMatches = 0;
                _report.TargetIdentityValid = false;
            }

            if (target != null)
            {
                _report.TargetLayer = target.layer;
                AuditSourceNativePhysics(target);
            }
            else
            {
                _report.TargetLayer = -1;
                _report.SourceNativeColliders = 0;
                _report.SourceNativeEnabledColliders = 0;
                _report.SourceNativeRigidbodies = 0;
            }

            if (IsStaticBackend())
            {
                StaticSourceMeshProxyRuntimeReport r = StaticSourceMeshProxyRuntime.CurrentReport;
                _report.ProviderLayer = StaticSourceMeshProxyRuntime.RuntimeLayer;
                _report.ProviderColliders = StaticSourceMeshProxyRuntime.Exists ? 1 : 0;
                _report.ProviderCollidersEnabled = StaticSourceMeshProxyRuntime.ColliderEnabled;
                _report.ProviderRigidbodies = r != null && r.RigidbodyPresent ? 1 : 0;
            }
            else
            {
                SpatialInstanceProxyRuntimeReport r = SpatialInstanceProxyRuntime.CurrentReport;
                _report.ProviderLayer = SpatialInstanceProxyRuntime.RuntimeLayer;
                _report.ProviderColliders = r != null ? r.Colliders : 0;
                _report.ProviderCollidersEnabled = r != null && r.CollidersEnabled;
                _report.ProviderRigidbodies = r != null ? r.Rigidbodies : 0;
            }

            _report.LayerIdentityValid =
                _report.TargetIdentityValid &&
                _report.TargetLayer >= 0 &&
                _report.ProviderLayer == _report.TargetLayer;
        }

        private static void AuditSourceNativePhysics(GameObject target)
        {
            _report.SourceNativeColliders = 0;
            _report.SourceNativeEnabledColliders = 0;
            _report.SourceNativeRigidbodies = 0;
            if (target == null)
                return;
            try
            {
                Collider[] colliders = target.GetComponentsInChildren<Collider>(true);
                for (int i = 0; i < colliders.Length; i++)
                {
                    Collider c = colliders[i];
                    if (c == null || (c.gameObject.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal))
                        continue;
                    _report.SourceNativeColliders++;
                    if (c.enabled)
                        _report.SourceNativeEnabledColliders++;
                }
            }
            catch { }
            try
            {
                Rigidbody[] bodies = target.GetComponentsInChildren<Rigidbody>(true);
                for (int i = 0; i < bodies.Length; i++)
                    if (bodies[i] != null)
                        _report.SourceNativeRigidbodies++;
            }
            catch { }
        }

        private static void ForceBackendOff()
        {
            try
            {
                if (IsStaticBackend())
                    StaticSourceMeshProxyRuntime.SetCollider(false);
                else
                    SpatialInstanceProxyRuntime.SetColliders(false);
            }
            catch { }
        }

        private static bool IsStaticBackend()
        {
            return _report != null && string.Equals(_report.Backend, BackendStaticMesh, StringComparison.Ordinal);
        }

        private static string Fail(string message)
        {
            if (_report != null)
            {
                _report.Completed = true;
                _report.Success = false;
                _report.Detail = message;
            }
            ClearOwnedBackends("environment-build-failed");
            _logger?.LogWarning("[GSP][ENVIRONMENT_PROVIDER] build-failed | " + message);
            return "<color=red>GSP environment provider 创建失败：</color> " + message;
        }

        private static string Append(string current, string item)
        {
            return string.IsNullOrEmpty(current) ? item : current + ";" + item;
        }
    }
}
