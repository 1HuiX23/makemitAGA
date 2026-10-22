using System;
using System.Collections.Generic;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Acquisition;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    internal sealed class SpatialInstanceProxyContactTarget
    {
        public int ProxyIndex;
        public string InstanceKey;
        public BoxCollider Collider;
        public Transform Transform;
        public Vector3 Size;
    }

    /// <summary>
    /// 把已经完成审计、已经匹配实例白名单的单盒或复合盒草案，创建成 GSP 自己拥有的
    /// 静态 BoxCollider + 半透明 Cube 预览。COMPOUND_BOX 的每个子盒独立创建，
    /// 但统一归属于同一个 GSP 根节点并共享启停、回滚生命周期。
    ///
    /// 安全边界：
    /// - 只接受显式 enabled=true、fixedToWorld=true、pickable=false 的 collision 实例规则；
    /// - 支持 ORIENTED_THIN_BOX / ORIENTED_BOX / COMPOUND_BOX；
    /// - 创建后 Collider 默认关闭，必须显式 gsp_instance_proxy_collider(on)；
    /// - 不创建 Rigidbody，不修改源对象，不禁用游戏原 Collider；
    /// - rollback/scene change/plugin unload 只销毁 GSP 自己创建的根节点和材质。
    /// </summary>
    internal static class SpatialInstanceProxyRuntime
    {
        private sealed class Entry
        {
            public string InstanceKey;
            public string ParentInstanceKey;
            public int CompoundPartIndex;
            public string CompoundPartKind;
            public GameObject GameObject;
            public MeshRenderer Renderer;
            public BoxCollider Collider;
            public Material Material;
            public SpatialInstanceProxyEntryReport Report;
            public Vector3 ExpectedPosition;
            public Quaternion ExpectedRotation;
            public Vector3 ExpectedScale;
            public Vector3 ExpectedColliderCenter;
            public Vector3 ExpectedColliderSize;
            public int ExpectedLayer;
        }

        private static readonly List<Entry> Entries = new List<Entry>();
        private static GameObject _root;
        private static GameObject _target;
        private static ManualLogSource _logger;
        private static SpatialInstanceProxyRuntimeReport _report;
        private static bool _visualVisible;
        private static bool _collidersEnabled;
        private static string _lastCleanupError;

        public static SpatialInstanceProxyRuntimeReport CurrentReport => _report;
        public static string LastCleanupError => _lastCleanupError;
        public static bool CollidersEnabled => _collidersEnabled;
        public static bool Exists => _root != null || Entries.Count != 0;
        public static GameObject RuntimeTarget => _target;
        public static int RuntimeLayer => Entries.Count > 0 && Entries[0] != null
            ? Entries[0].ExpectedLayer
            : -1;

        public static bool TryGetContactTargets(
            List<SpatialInstanceProxyContactTarget> destination,
            out string targetName,
            out bool collidersEnabled,
            out string error)
        {
            targetName = _report != null ? _report.TargetName : null;
            collidersEnabled = _collidersEnabled;
            error = null;
            if (destination == null)
            {
                error = "destination-list-null";
                return false;
            }
            destination.Clear();
            if (_root == null || Entries.Count == 0 || _report == null || !_report.Success)
            {
                error = "当前没有通过安全门的实例代理。请先执行 gsp_instance_proxy_build。";
                return false;
            }
            if (CountLiveRigidbodies() != 0)
            {
                error = "代理对象上检测到 Rigidbody；拒绝开始静态代理接触验证。";
                return false;
            }

            for (int i = 0; i < Entries.Count; i++)
            {
                Entry entry = Entries[i];
                if (entry == null || entry.GameObject == null || entry.Collider == null)
                    continue;
                Vector3 size = entry.GameObject.transform.localScale;
                if (!IsFinite(size) || size.x <= 0f || size.y <= 0f || size.z <= 0f)
                    continue;
                destination.Add(new SpatialInstanceProxyContactTarget
                {
                    ProxyIndex = i + 1,
                    InstanceKey = entry.InstanceKey,
                    Collider = entry.Collider,
                    Transform = entry.GameObject.transform,
                    Size = size
                });
            }

            if (destination.Count == 0)
            {
                error = "实例代理列表存在，但没有可用于接触测试的有效 BoxCollider。";
                return false;
            }
            return true;
        }

        internal static bool TrySetCollidersForContactTest(bool enabled, out string error)
        {
            bool ok = TrySetColliderState(enabled, out error);
            _logger?.LogInfo(
                "[GSP][INSTANCE_PROXY] contact-test-collider-state=" + enabled +
                " | accepted=" + ok +
                " | colliders=" + CountLiveColliders() +
                (string.IsNullOrEmpty(error) ? string.Empty : " | error=" + error));
            return ok;
        }

        public static string Build(
            MultipartAggregationReport source,
            GameObject target,
            ManualLogSource logger)
        {
            _logger = logger;
            Clear("rebuild-before-create", false);
            string priorCleanupError = _lastCleanupError;

            _report = new SpatialInstanceProxyRuntimeReport
            {
                Started = true,
                TargetName = source != null ? source.TargetName : null,
                TargetPath = source != null ? source.TargetPath : null,
                VisualVisible = true,
                CollidersEnabled = false,
                Detail = "validating-current-spatial-instance-report"
            };

            if (!string.IsNullOrEmpty(priorCleanupError))
                return Fail("前一份 Box/Compound provider 清理未完成，拒绝重建：" + priorCleanupError);

            if (source == null || !source.Completed || !source.Success)
                return Fail("当前没有已完成且成功的空间实例报告。请先执行 gsp_instance_audit(对象名称)。");
            if (!ValidateBoundTarget(source, target, out string targetError))
                return Fail(targetError);
            _target = target;

            SpatialInstanceAuditReport collision = source.CollisionSpatialInstances;
            if (collision == null || !collision.Analyzed || collision.Instances.Count == 0)
                return Fail("当前报告没有可用的碰撞空间实例。请先确认 collisionSpatial instances > 0。");

            _root = new GameObject(
                "GSP_InstanceProxyRuntime_" + Sanitize(source.TargetName));
            _root.transform.position = Vector3.zero;
            _root.transform.rotation = Quaternion.identity;
            _root.transform.localScale = Vector3.one;
            _report.RootName = _root.name;

            int inheritedLayer = target != null ? target.layer : 0;
            for (int i = 0; i < collision.Instances.Count; i++)
            {
                SpatialInstanceReport instance = collision.Instances[i];
                if (instance == null)
                    continue;

                _report.Candidates++;
                if (!instance.ProxyDraftBuilt)
                {
                    _report.SkippedNoDraft++;
                    continue;
                }

                SpatialInstancePhysicsProfileSnapshot profile = instance.PhysicsProfile;
                if (profile == null || !profile.Matched)
                {
                    _report.SkippedNoProfile++;
                    continue;
                }
                if (!profile.EnabledConfigured || !profile.Enabled)
                {
                    _report.SkippedDisabled++;
                    continue;
                }

                string proxyType = profile.ProxyTypeConfigured
                    ? profile.ProxyType
                    : instance.ProxyDraftType;
                if (!IsSupportedProxyType(proxyType))
                {
                    _report.SkippedUnsupportedProxy++;
                    continue;
                }
                if (!profile.FixedToWorldConfigured || !profile.FixedToWorld)
                {
                    _report.SkippedNotFixedToWorld++;
                    continue;
                }
                if (!profile.PickableConfigured || profile.Pickable)
                {
                    _report.SkippedPickable++;
                    continue;
                }

                if (IsCompoundProxyType(proxyType))
                {
                    if (!TryCreateCompoundEntries(
                        instance,
                        proxyType,
                        inheritedLayer,
                        profile,
                        out List<Entry> createdEntries,
                        out string compoundError))
                    {
                        _report.SkippedCompoundInvalid++;
                        _logger?.LogWarning(
                            "[GSP][INSTANCE_PROXY] skipped-compound-create-failure | key=" +
                            instance.InstanceKey + " | error=" + compoundError);
                        continue;
                    }

                    _report.CompoundInstances++;
                    _report.CompoundPartsCreated += createdEntries.Count;
                    for (int c = 0; c < createdEntries.Count; c++)
                        AcceptEntry(createdEntries[c]);
                    continue;
                }

                if (!TryCreateEntry(
                    instance.InstanceKey,
                    proxyType,
                    instance.ProxyCenter,
                    instance.ProxySize,
                    instance.ProxyEulerDegrees,
                    null,
                    0,
                    inheritedLayer,
                    _report.Created,
                    profile,
                    out Entry entry,
                    out string createError))
                {
                    _logger?.LogWarning(
                        "[GSP][INSTANCE_PROXY] skipped-create-failure | key=" +
                        instance.InstanceKey + " | error=" + createError);
                    continue;
                }

                AcceptEntry(entry);
            }

            _visualVisible = true;
            _collidersEnabled = false;
            ApplyVisualState();
            ApplyColliderState();

            _report.Completed = true;
            _report.Success =
                _report.Created > 0 &&
                _report.Colliders == _report.Created &&
                _report.Renderers == _report.Created &&
                _report.Rigidbodies == 0;
            _report.VisualVisible = _visualVisible;
            _report.CollidersEnabled = _collidersEnabled;
            _report.Detail = _report.Success
                ? "static single/compound box proxies created; visual on; colliders intentionally off until explicit enable"
                : "proxy build completed but one or more creation/safety checks failed";

            if (!_report.Success)
            {
                string failure = _report.ToStatusText();
                _logger?.LogWarning("[GSP][INSTANCE_PROXY] build-not-accepted | " + failure);
                Clear("build-not-accepted", false);
                return "<color=red>实例代理创建未通过安全门。</color> " + failure;
            }

            _logger?.LogWarning(
                "[GSP][INSTANCE_PROXY] build-complete | " + _report.ToDetailedText());
            return
                "<color=#66ddff>实例单盒/复合盒代理已创建。</color> " +
                "created=" + _report.Created +
                "，compoundInstances=" + _report.CompoundInstances +
                "，compoundParts=" + _report.CompoundPartsCreated +
                "，visual=on，collider=off，Rigidbody=0。" +
                "请先观察半透明盒位置，再执行 gsp_instance_proxy_collider(on)。";
        }

        public static string SetVisual(bool visible)
        {
            if (_root == null || Entries.Count == 0)
                return "当前没有实例代理。先执行 gsp_instance_proxy_build。";

            _visualVisible = visible;
            ApplyVisualState();
            UpdateReportState();
            _logger?.LogInfo(
                "[GSP][INSTANCE_PROXY] visual=" + visible +
                " | entries=" + Entries.Count);
            return visible
                ? "<color=#66ddff>实例代理半透明可视化已开启。</color>"
                : "实例代理半透明可视化已关闭；Collider 状态不变。";
        }

        public static string SetColliders(bool enabled)
        {
            if (_root == null || Entries.Count == 0)
                return "当前没有实例代理。先执行 gsp_instance_proxy_build。";

            bool accepted = TrySetColliderState(enabled, out string error);
            _logger?.LogWarning(
                "[GSP][INSTANCE_PROXY] collider-request=" + enabled +
                " | accepted=" + accepted +
                " | colliders=" + CountLiveColliders() +
                " | rigidbodies=" + CountLiveRigidbodies() +
                (string.IsNullOrEmpty(error) ? string.Empty : " | error=" + error));
            if (!accepted)
                return "<color=red>拒绝切换实例代理 Collider：</color> " + error + "；全部 Collider 已尽力保持关闭。";
            return enabled
                ? "<color=#66ddff>实例代理 BoxCollider 已开启。</color> 二次启用安全门已通过；代理保持静态、无 Rigidbody，源对象未修改。"
                : "实例代理 BoxCollider 已关闭；半透明可视化状态不变。";
        }

        public static string GetStatusText()
        {
            if (_report == null)
                return "当前没有实例代理运行时报告。先完成审计并执行 gsp_instance_proxy_build。";
            UpdateReportState();
            return _report.ToStatusText();
        }

        public static string Rollback()
        {
            int count = Entries.Count;
            Clear("manual-rollback", true);
            return string.IsNullOrEmpty(_lastCleanupError)
                ? "<color=#66ddff>实例代理已请求完整回滚。</color> 已清理 GSP 根节点、" +
                  count + " 个临时盒代理及其材质；游戏原对象和原 Collider 未修改。"
                : "<color=red>实例代理回滚遇到异常：</color> " + _lastCleanupError;
        }

        public static void Clear(string reason)
        {
            Clear(reason, true);
        }

        private static void AcceptEntry(Entry entry)
        {
            if (entry == null)
                return;
            Entries.Add(entry);
            _report.Entries.Add(entry.Report);
            _report.Created++;
            if (entry.Collider != null)
                _report.Colliders++;
            if (entry.Renderer != null)
                _report.Renderers++;
            if (entry.Report != null && entry.Report.RigidbodyPresent)
                _report.Rigidbodies++;

            _logger?.LogInfo(
                "[GSP][INSTANCE_PROXY][CREATED] " +
                (entry.Report != null ? entry.Report.ToLogLine() : "<null-report>"));
        }

        private static bool TryCreateCompoundEntries(
            SpatialInstanceReport instance,
            string proxyType,
            int layer,
            SpatialInstancePhysicsProfileSnapshot profile,
            out List<Entry> createdEntries,
            out string error)
        {
            createdEntries = new List<Entry>();
            error = null;
            if (instance == null || !instance.CompoundProxyDraftBuilt ||
                instance.CompoundParts == null || instance.CompoundParts.Count == 0)
            {
                error = "compound-draft-missing-or-empty";
                return false;
            }

            for (int i = 0; i < instance.CompoundParts.Count; i++)
            {
                CompoundBoxPartReport part = instance.CompoundParts[i];
                if (part == null)
                {
                    error = "compound-part-null:index=" + (i + 1);
                    DestroyTemporaryEntries(createdEntries);
                    createdEntries.Clear();
                    return false;
                }

                if (!TryCreateEntry(
                    instance.InstanceKey,
                    proxyType,
                    part.Center,
                    part.Size,
                    part.EulerDegrees,
                    part.Kind,
                    part.PartIndex > 0 ? part.PartIndex : i + 1,
                    layer,
                    _report.Created + createdEntries.Count,
                    profile,
                    out Entry entry,
                    out string createError))
                {
                    error = "part=" + (i + 1) + ":" + createError;
                    DestroyTemporaryEntries(createdEntries);
                    createdEntries.Clear();
                    return false;
                }
                createdEntries.Add(entry);
            }

            return createdEntries.Count == instance.CompoundParts.Count;
        }

        private static bool TryCreateEntry(
            string parentInstanceKey,
            string proxyType,
            Vector3 center,
            Vector3 size,
            Vector3 eulerDegrees,
            string compoundPartKind,
            int compoundPartIndex,
            int layer,
            int zeroBasedGlobalIndex,
            SpatialInstancePhysicsProfileSnapshot profile,
            out Entry entry,
            out string error)
        {
            entry = null;
            error = null;
            GameObject go = null;
            Material material = null;
            try
            {
                if (!IsFinite(center) ||
                    !IsFinite(size) ||
                    !IsFinite(eulerDegrees) ||
                    size.x <= 0f || size.y <= 0f || size.z <= 0f)
                {
                    error = "invalid-proxy-center-size-or-euler";
                    return false;
                }

                go = GameObject.CreatePrimitive(PrimitiveType.Cube);
                if (go == null)
                {
                    error = "CreatePrimitive-returned-null";
                    return false;
                }

                string entryKey = parentInstanceKey;
                string partSuffix = string.Empty;
                if (compoundPartIndex > 0)
                {
                    partSuffix = "_Part_" + compoundPartIndex + "_" +
                        Sanitize(compoundPartKind);
                    entryKey = parentInstanceKey + "#P" + compoundPartIndex + "-" +
                        (compoundPartKind ?? "PART");
                }

                string name =
                    (compoundPartIndex > 0 ? "GSP_CompoundProxy_" : "GSP_Proxy_") +
                    (zeroBasedGlobalIndex + 1) + partSuffix + "_" +
                    Sanitize(parentInstanceKey);
                go.name = name;
                go.layer = layer;
                go.transform.SetParent(_root.transform, false);
                go.transform.position = center;
                go.transform.rotation = Quaternion.Euler(eulerDegrees);
                go.transform.localScale = size;

                BoxCollider collider = go.GetComponent<BoxCollider>();
                if (collider == null)
                    collider = go.AddComponent<BoxCollider>();
                if (collider == null)
                {
                    error = "cannot-create-BoxCollider";
                    return false;
                }
                collider.isTrigger = false;
                collider.enabled = false;

                MeshRenderer renderer = go.GetComponent<MeshRenderer>();
                if (renderer == null)
                {
                    error = "primitive-MeshRenderer-missing";
                    return false;
                }
                material = CreatePreviewMaterial(zeroBasedGlobalIndex, compoundPartKind);
                if (material == null)
                {
                    error = "cannot-create-preview-material";
                    return false;
                }
                renderer.sharedMaterial = material;
                renderer.shadowCastingMode = ShadowCastingMode.Off;
                renderer.receiveShadows = false;
                renderer.enabled = true;

                Rigidbody rigidbody = go.GetComponent<Rigidbody>();
                SpatialInstanceProxyEntryReport report =
                    new SpatialInstanceProxyEntryReport
                    {
                        Index = zeroBasedGlobalIndex + 1,
                        InstanceKey = entryKey,
                        ParentInstanceKey = parentInstanceKey,
                        CompoundPartIndex = compoundPartIndex,
                        CompoundPartKind = compoundPartKind,
                        ProxyType = proxyType,
                        ObjectName = name,
                        Center = center,
                        Size = size,
                        EulerDegrees = eulerDegrees,
                        GameObjectCreated = true,
                        RendererCreated = renderer != null,
                        ColliderCreated = collider != null,
                        ColliderEnabled = collider != null && collider.enabled,
                        RigidbodyPresent = rigidbody != null,
                        FixedToWorld = profile != null && profile.FixedToWorld,
                        Pickable = profile != null && profile.Pickable,
                        Layer = layer,
                        Detail = compoundPartIndex > 0
                            ? "GSP-owned static compound child; collider initially disabled; source object unchanged"
                            : "GSP-owned static primitive; collider initially disabled; source object unchanged"
                    };

                if (report.RigidbodyPresent)
                {
                    error = "unexpected-Rigidbody-present";
                    return false;
                }

                entry = new Entry
                {
                    InstanceKey = entryKey,
                    ParentInstanceKey = parentInstanceKey,
                    CompoundPartIndex = compoundPartIndex,
                    CompoundPartKind = compoundPartKind,
                    GameObject = go,
                    Renderer = renderer,
                    Collider = collider,
                    Material = material,
                    Report = report,
                    ExpectedPosition = center,
                    ExpectedRotation = Quaternion.Euler(eulerDegrees),
                    ExpectedScale = size,
                    ExpectedColliderCenter = collider.center,
                    ExpectedColliderSize = collider.size,
                    ExpectedLayer = layer
                };
                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + ":" + e.Message;
                return false;
            }
            finally
            {
                if (entry == null)
                {
                    if (go != null)
                        Object.Destroy(go);
                    if (material != null)
                        Object.Destroy(material);
                }
            }
        }

        private static void DestroyTemporaryEntries(List<Entry> entries)
        {
            if (entries == null)
                return;
            for (int i = 0; i < entries.Count; i++)
            {
                Entry item = entries[i];
                if (item?.Collider != null)
                    item.Collider.enabled = false;
                if (item?.Renderer != null)
                    item.Renderer.enabled = false;
                if (item?.GameObject != null)
                    Object.Destroy(item.GameObject);
                if (item?.Material != null)
                    Object.Destroy(item.Material);
            }
        }

        private static void ApplyVisualState()
        {
            for (int i = 0; i < Entries.Count; i++)
            {
                Entry item = Entries[i];
                if (item?.Renderer != null)
                    item.Renderer.enabled = _visualVisible;
            }
        }

        private static void ApplyColliderState()
        {
            // Build-time OFF path only. Runtime OFF→ON transitions must use TrySetColliderState,
            // which revalidates identity/transform/shape/layer/Rigidbody before and after enabling.
            for (int i = 0; i < Entries.Count; i++)
            {
                Entry item = Entries[i];
                if (item?.Collider != null)
                {
                    item.Collider.enabled = _collidersEnabled;
                    if (item.Report != null)
                        item.Report.ColliderEnabled = item.Collider.enabled;
                }
            }
        }

        private static bool TrySetColliderState(bool enabled, out string error)
        {
            error = null;
            if (_root == null || Entries.Count == 0 || _report == null || !_report.Success)
            {
                error = "runtime-provider-not-ready";
                ForceDisableAllColliders();
                return false;
            }

            if (!enabled)
            {
                ForceDisableAllColliders();
                _collidersEnabled = false;
                UpdateReportState();
                Physics.SyncTransforms();
                return true;
            }

            if (!ValidateRuntimeState(out error))
            {
                ForceDisableAllColliders();
                _collidersEnabled = false;
                UpdateReportState();
                return false;
            }

            try
            {
                for (int i = 0; i < Entries.Count; i++)
                {
                    Entry item = Entries[i];
                    if (item == null || item.Collider == null)
                        throw new InvalidOperationException("collider-missing:index=" + (i + 1));
                    item.Collider.enabled = true;
                }
                Physics.SyncTransforms();
                for (int i = 0; i < Entries.Count; i++)
                {
                    Entry item = Entries[i];
                    if (item?.Collider == null || !item.Collider.enabled)
                        throw new InvalidOperationException("collider-enable-readback-failed:index=" + (i + 1));
                }
            }
            catch (Exception e)
            {
                error = e.GetType().Name + ":" + e.Message;
                ForceDisableAllColliders();
                _collidersEnabled = false;
                UpdateReportState();
                return false;
            }

            if (!ValidateRuntimeState(out error))
            {
                ForceDisableAllColliders();
                _collidersEnabled = false;
                UpdateReportState();
                return false;
            }

            _collidersEnabled = true;
            UpdateReportState();
            return true;
        }

        private static bool ValidateRuntimeState(out string error)
        {
            error = null;
            if (_target == null || _report == null ||
                !string.Equals(TargetFinder.GetPath(_target.transform), _report.TargetPath, StringComparison.Ordinal))
            {
                error = "target-identity-lost";
                return false;
            }
            if (_root == null || !Approximately(_root.transform.position, Vector3.zero) ||
                Quaternion.Angle(_root.transform.rotation, Quaternion.identity) > 0.05f ||
                !Approximately(_root.transform.localScale, Vector3.one))
            {
                error = "provider-root-transform-drift";
                return false;
            }
            if (CountLiveRigidbodies() != 0)
            {
                error = "unexpected-Rigidbody-present";
                return false;
            }

            for (int i = 0; i < Entries.Count; i++)
            {
                Entry item = Entries[i];
                if (item == null || item.GameObject == null || item.Collider == null)
                {
                    error = "entry-or-collider-missing:index=" + (i + 1);
                    return false;
                }
                if (item.GameObject.transform.parent != _root.transform)
                {
                    error = "entry-parent-drift:index=" + (i + 1);
                    return false;
                }
                if (item.GameObject.layer != item.ExpectedLayer || item.ExpectedLayer != _target.layer)
                {
                    error = "layer-drift:index=" + (i + 1) + ",expected=" + item.ExpectedLayer + ",actual=" + item.GameObject.layer + ",target=" + _target.layer;
                    return false;
                }
                if (!Approximately(item.GameObject.transform.position, item.ExpectedPosition) ||
                    Quaternion.Angle(item.GameObject.transform.rotation, item.ExpectedRotation) > 0.05f ||
                    !Approximately(item.GameObject.transform.localScale, item.ExpectedScale))
                {
                    error = "entry-transform-drift:index=" + (i + 1);
                    return false;
                }
                if (item.Collider.isTrigger ||
                    !Approximately(item.Collider.center, item.ExpectedColliderCenter) ||
                    !Approximately(item.Collider.size, item.ExpectedColliderSize) ||
                    item.Collider.attachedRigidbody != null ||
                    item.GameObject.GetComponent<Rigidbody>() != null)
                {
                    error = "collider-shape-or-rigidbody-drift:index=" + (i + 1);
                    return false;
                }
            }
            return true;
        }

        private static void ForceDisableAllColliders()
        {
            for (int i = 0; i < Entries.Count; i++)
            {
                Entry item = Entries[i];
                if (item?.Collider == null)
                    continue;
                try { item.Collider.enabled = false; }
                catch { }
                if (item.Report != null)
                    item.Report.ColliderEnabled = false;
            }
            try { Physics.SyncTransforms(); }
            catch { }
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

        private static bool Approximately(Vector3 a, Vector3 b)
        {
            return Mathf.Abs(a.x - b.x) <= 0.0001f &&
                   Mathf.Abs(a.y - b.y) <= 0.0001f &&
                   Mathf.Abs(a.z - b.z) <= 0.0001f;
        }

        private static void UpdateReportState()
        {
            if (_report == null)
                return;
            _report.VisualVisible = _visualVisible;
            _report.CollidersEnabled = _collidersEnabled;
            _report.Colliders = CountLiveColliders();
            _report.Renderers = CountLiveRenderers();
            _report.Rigidbodies = CountLiveRigidbodies();
        }

        private static int CountLiveColliders()
        {
            int count = 0;
            for (int i = 0; i < Entries.Count; i++)
                if (Entries[i]?.Collider != null)
                    count++;
            return count;
        }

        private static int CountLiveRenderers()
        {
            int count = 0;
            for (int i = 0; i < Entries.Count; i++)
                if (Entries[i]?.Renderer != null)
                    count++;
            return count;
        }

        private static int CountLiveRigidbodies()
        {
            int count = 0;
            for (int i = 0; i < Entries.Count; i++)
            {
                GameObject go = Entries[i]?.GameObject;
                if (go != null && go.GetComponent<Rigidbody>() != null)
                    count++;
            }
            return count;
        }

        private static void Clear(string reason, bool discardReport)
        {
            string errors = null;
            for (int i = 0; i < Entries.Count; i++)
            {
                Entry item = Entries[i];
                try { if (item?.Collider != null) item.Collider.enabled = false; }
                catch (Exception e) { errors = AppendCleanupError(errors, "colliderOff#" + (i + 1), e); }
                try { if (item?.Renderer != null) item.Renderer.enabled = false; }
                catch (Exception e) { errors = AppendCleanupError(errors, "rendererOff#" + (i + 1), e); }
                try { if (item?.GameObject != null) Object.Destroy(item.GameObject); }
                catch (Exception e) { errors = AppendCleanupError(errors, "destroyGO#" + (i + 1), e); }
                try { if (item?.Material != null) Object.Destroy(item.Material); }
                catch (Exception e) { errors = AppendCleanupError(errors, "destroyMaterial#" + (i + 1), e); }
            }
            Entries.Clear();

            try
            {
                if (_root != null)
                    Object.Destroy(_root);
            }
            catch (Exception e) { errors = AppendCleanupError(errors, "destroyRoot", e); }
            _root = null;

            _target = null;
            _visualVisible = false;
            _collidersEnabled = false;
            _lastCleanupError = errors;
            if (discardReport)
                _report = null;
            if (string.IsNullOrEmpty(errors))
                _logger?.LogInfo("[GSP][INSTANCE_PROXY] cleared | reason=" + (reason ?? "<none>"));
            else
                _logger?.LogWarning("[GSP][INSTANCE_PROXY] cleanup-error | reason=" + (reason ?? "<none>") + " | " + errors);
        }

        private static string AppendCleanupError(string current, string stage, Exception e)
        {
            string item = stage + ":" + e.GetType().Name + ":" + e.Message;
            return string.IsNullOrEmpty(current) ? item : current + ";" + item;
        }

        private static string Fail(string message)
        {
            if (_report != null)
            {
                _report.Completed = true;
                _report.Success = false;
                _report.Detail = message;
            }
            Clear("build-validation-failed", false);
            _logger?.LogWarning("[GSP][INSTANCE_PROXY] build-failed | " + message);
            return "<color=red>实例代理创建失败：</color> " + message;
        }

        private static bool IsSupportedProxyType(string value)
        {
            return
                string.Equals(value, "ORIENTED_THIN_BOX", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(value, "ORIENTED_BOX", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(value, "COMPOUND_BOX", StringComparison.OrdinalIgnoreCase);
        }


        private static bool IsCompoundProxyType(string value)
        {
            return string.Equals(value, "COMPOUND_BOX", StringComparison.OrdinalIgnoreCase);
        }

        private static Material CreatePreviewMaterial(int index, string compoundPartKind)
        {
            Shader shader = Shader.Find("Legacy Shaders/Transparent/Diffuse");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                return null;

            Color[] colors =
            {
                new Color(0.05f, 0.90f, 1.00f, 0.28f),
                new Color(1.00f, 0.35f, 0.85f, 0.28f),
                new Color(1.00f, 0.82f, 0.12f, 0.28f),
                new Color(0.35f, 1.00f, 0.35f, 0.28f)
            };
            Color color;
            if (string.Equals(compoundPartKind, "HORIZONTAL_SLAB_BOX", StringComparison.OrdinalIgnoreCase))
                color = new Color(0.05f, 0.90f, 1.00f, 0.25f);
            else if (string.Equals(compoundPartKind, "ENCLOSING_FRAME_CORNER_POST", StringComparison.OrdinalIgnoreCase))
                color = new Color(1.00f, 0.82f, 0.12f, 0.30f);
            else
                color = colors[Math.Abs(index) % colors.Length];
            Material material = new Material(shader)
            {
                name = "GSP_InstanceProxyMaterial_" + (index + 1),
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

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "Unknown";
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
