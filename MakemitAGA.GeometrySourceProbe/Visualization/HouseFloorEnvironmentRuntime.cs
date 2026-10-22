using System;
using System.Collections.Generic;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Acquisition;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    /// <summary>
    /// GSP v0.2.3 active-room incremental floor atlas provider。
    ///
    /// MiSide 的房间 activation 由玩家所在位置驱动：非玩家房间通常会整体失活，
    /// 因而“4/4 floor 必须同时 active”不是合法前提。当前实现把 source lifecycle 与 provider lifecycle 分离：
    /// - 四个房间只是长期语义槽位；未激活房间为 WAITING_ACTIVE，不是 failure；
    /// - 某房间第一次真实 active 时，复用已验证的只读 D3D11 staging pipeline 捕获其地板；
    /// - 捕获后创建 parentless GSP-owned static non-convex MeshCollider，并独立持有到 scene-change/rollback；
    /// - source room 后续 SetActive(false) 不会使已捕获 floor 消失，也不会阻止 provider OFF→ON；
    /// - GSP 不读取、不引用 ARL DLL，唯一共享面仍是 Unity Scene / Physics。
    /// </summary>
    internal static class HouseFloorEnvironmentRuntime
    {
        private sealed class RuntimeFloor
        {
            public HouseFloorRegistry.FloorBinding Binding;
            public HouseFloorProviderItemReport Report;
            public GameObject Root;
            public Mesh Mesh;
            public Material Material;
            public MeshRenderer Renderer;
            public MeshCollider Collider;
            public Vector3 ExpectedPosition;
            public Quaternion ExpectedRotation = Quaternion.identity;
            public Vector3 ExpectedScale = Vector3.one;
            public Bounds ExpectedBounds;
            public int ExpectedLayer = -1;
        }

        private const float DiscoveryIntervalSeconds = 0.50f;

        private static readonly List<RuntimeFloor> Floors = new List<RuntimeFloor>();
        // 固定四个语义槽位。slot 本身长期存在；只有当前 active 时才会被解析成带 Target/TargetPath 的 binding。
        private static readonly List<HouseFloorRegistry.FloorBinding> PendingBindings =
            new List<HouseFloorRegistry.FloorBinding>();

        private static ManualLogSource _logger;
        private static HouseFloorProviderReport _report;
        private static int _currentIndex = -1;
        private static float _nextDiscoveryRealtime;
        private static string _lastCleanupError;

        public static HouseFloorProviderReport CurrentReport => _report;
        public static bool Exists => Floors.Count > 0;
        public static bool IsRunning => _report != null && _report.Running;
        public static string LastCleanupError => _lastCleanupError;

        public static string StartBuild(ManualLogSource logger)
        {
            _logger = logger;

            MultipartObjectAggregator.Cancel("runtime-clear:house-floor-build-start");
            EnvironmentProviderRuntime.Clear("house-floor-build-start");
            InstanceProxyContactTest.Stop("house-floor-build-start", false);

            string cleanupError = ClearOwnedFloors("house-floor-rebuild");
            _report = new HouseFloorProviderReport
            {
                Started = true,
                Running = false,
                Completed = false,
                Success = false,
                Operational = false,
                WatcherActive = true,
                DesiredVisual = false,
                DesiredCollider = false,
                SceneName = SceneManager.GetActiveScene().name,
                Stage = "waiting-for-active-room",
                ExpectedFloors = HouseFloorRegistry.ExpectedFloorCount,
                Detail = "progressive-capture-watcher-starting"
            };

            if (!string.IsNullOrEmpty(cleanupError))
                return Fail("上一份 house-floor atlas 清理出现异常，拒绝重建：" + cleanupError);

            PendingBindings.Clear();
            PendingBindings.AddRange(HouseFloorRegistry.CreateExpectedBindings());
            if (PendingBindings.Count != HouseFloorRegistry.ExpectedFloorCount)
                return Fail("floor semantic slot 数量不完整：" + PendingBindings.Count + "/" + HouseFloorRegistry.ExpectedFloorCount);

            for (int i = 0; i < PendingBindings.Count; i++)
            {
                HouseFloorRegistry.FloorBinding binding = PendingBindings[i];
                _report.Floors.Add(new HouseFloorProviderItemReport
                {
                    RoomName = binding.RoomName,
                    FloorObjectName = binding.FloorObjectName,
                    RoomPath = binding.RoomPath,
                    TargetPath = null,
                    TargetIdentityValid = false,
                    SourceCurrentlyActive = false,
                    SourceActiveAtCapture = false,
                    CachedPersistent = false,
                    CaptureWaiting = true,
                    CaptureFailed = false,
                    Detail = "WAITING_ACTIVE"
                });
            }

            _nextDiscoveryRealtime = 0f;
            RefreshRuntimeTruth();
            TryStartNextCapture(true);

            _logger?.LogWarning(
                "[GSP][HOUSE_FLOOR] atlas-start" +
                " | build=0.2.3-active-room-incremental-floor-atlas-foundation" +
                " | expected=4" +
                " | semantics=Bedroom:FloorBedroom,Kitchen:FloorKitchen,Main:RoomMain,Toilet:FloorToilet" +
                " | simultaneousActivationRequired=False" +
                " | sourceRoomActivationIsSupportTruth=False" +
                " | cachedFloorsPersistAcrossRoomDeactivation=True" +
                " | collidersInitiallyOff=True" +
                " | rigidbodies=0" +
                " | dllCoupling=NONE_UNITY_PHYSICS_ONLY");

            return "<color=#66ddff>GSP 房间生命周期感知地板 atlas 已启动。</color> " +
                "只捕获当前 active 的房间；其余显示 WAITING_ACTIVE。玩家之后进入其它房间时会自动补捕获，" +
                "已捕获的 GSP-owned floor 不随源房间失活。执行 gsp_floor_status 查看进度。";
        }

        public static void Update()
        {
            if (_report == null || !_report.WatcherActive)
                return;

            if (_report.Running)
            {
                MultipartAggregationReport audit = MultipartObjectAggregator.CurrentReport;
                if (audit == null || audit.Running || !audit.Completed)
                    return;
                ConsumeCurrentAudit(audit);
                return;
            }

            if (Time.realtimeSinceStartup < _nextDiscoveryRealtime)
                return;

            if (!_report.Completed)
                TryStartNextCapture(true);
            else
            {
                RefreshRuntimeTruth();
                _nextDiscoveryRealtime = Time.realtimeSinceStartup + DiscoveryIntervalSeconds;
            }
        }

        public static string GetStatusText()
        {
            if (_report == null)
                return "当前没有 GSP house-floor atlas。执行 gsp_floor_build 启动 active-room incremental capture。";
            RefreshRuntimeTruth();
            return _report.ToStatusText();
        }

        public static string SetVisual(bool visible)
        {
            if (_report == null || !_report.WatcherActive)
                return "当前没有活动的 house-floor atlas watcher。先执行 gsp_floor_build。";

            _report.DesiredVisual = visible;
            for (int i = 0; i < Floors.Count; i++)
            {
                RuntimeFloor floor = Floors[i];
                if (floor.Renderer == null)
                    return FailRuntime("visual-renderer-missing:" + floor.Binding.RoomName);
                try { floor.Renderer.enabled = visible; }
                catch (Exception e) { return FailRuntime("visual-toggle-failed:" + e.GetType().Name + ":" + e.Message); }
            }

            return "GSP 已捕获地板可视化=" + visible + "，captured=" + Floors.Count + "/" + _report.ExpectedFloors +
                "。尚未访问的房间将在捕获后自动继承该 visual 状态。";
        }

        public static string SetColliders(bool enabled)
        {
            if (_report == null || !_report.WatcherActive)
                return "当前没有活动的 house-floor atlas watcher。先执行 gsp_floor_build。";

            if (!enabled)
            {
                _report.DesiredCollider = false;
                ForceAllCollidersOff();
                RefreshRuntimeTruth();
                return "<color=#66ddff>GSP 已捕获地板 Collider=False。</color> enabled=0/" + _report.BuiltFloors + "。";
            }

            if (Floors.Count == 0)
                return "当前还没有任何 CAPTURED floor；拒绝把 future capture 默认为 Collider ON。先等待当前房间完成捕获。";

            if (!ValidateAllBeforeEnable(out string validationError))
            {
                _report.DesiredCollider = false;
                ForceAllCollidersOff();
                RefreshRuntimeTruth();
                return "<color=red>拒绝启用已捕获地板 Collider：</color> " + validationError;
            }

            for (int i = 0; i < Floors.Count; i++)
            {
                try { Floors[i].Collider.enabled = true; }
                catch (Exception e)
                {
                    _report.DesiredCollider = false;
                    ForceAllCollidersOff();
                    RefreshRuntimeTruth();
                    return "<color=red>切换地板 Collider 失败，已 fail-close：</color> " +
                        Floors[i].Binding.RoomName + "/" + e.GetType().Name + ":" + e.Message;
                }
            }
            try { Physics.SyncTransforms(); }
            catch { }
            RefreshRuntimeTruth();

            string readbackError = string.Empty;
            if (_report.EnabledFloors != _report.BuiltFloors || !ValidateAllAfterEnable(out readbackError))
            {
                _report.DesiredCollider = false;
                ForceAllCollidersOff();
                RefreshRuntimeTruth();
                return "<color=red>地板 Collider 启用后读回失败，已 fail-close：</color> " + readbackError;
            }

            _report.DesiredCollider = true;
            _logger?.LogWarning(
                "[GSP][HOUSE_FLOOR] second-enable-preflight=PASS" +
                " | enabled=" + _report.EnabledFloors + "/" + _report.BuiltFloors +
                " | expectedSemanticFloors=" + _report.ExpectedFloors +
                " | sourceActiveRequired=False" +
                " | rigidbodies=0" +
                " | cachedIdentity=True");

            return "<color=#66ddff>GSP 已捕获地板 Collider=True。</color> enabled=" +
                _report.EnabledFloors + "/" + _report.BuiltFloors +
                "；尚未捕获的房间以后成功 capture 时会先做同样 preflight，再继承 ON。";
        }

        public static string Rollback()
        {
            bool existed = Floors.Count > 0 || _report != null;
            MultipartObjectAggregator.Cancel("runtime-clear:house-floor-manual-rollback");
            if (_report != null)
                _report.WatcherActive = false;
            string errors = ClearOwnedFloors("house-floor-manual-rollback");
            if (_report != null)
            {
                _report.Running = false;
                _report.Completed = true;
                _report.Success = false;
                _report.Operational = false;
                _report.EnabledFloors = 0;
                _report.DesiredCollider = false;
                _report.DesiredVisual = false;
                _report.Stage = "rolled-back";
                _report.Detail = string.IsNullOrEmpty(errors)
                    ? "rollback-requested-without-exception"
                    : "rollback-error:" + errors;
            }
            return !existed
                ? "当前没有需要回滚的 GSP house-floor atlas。"
                : string.IsNullOrEmpty(errors)
                    ? "<color=#66ddff>GSP house-floor atlas 已请求完整回滚。</color> 已捕获的 GSP-owned Mesh/Collider/Material/预览均进入清理；源房间/源地板未修改。"
                    : "<color=red>house-floor atlas 回滚出现异常：</color> " + errors;
        }

        public static void Clear(string reason)
        {
            MultipartObjectAggregator.Cancel("runtime-clear:house-floor-clear:" + reason);
            if (_report != null)
                _report.WatcherActive = false;
            string errors = ClearOwnedFloors(reason);
            if (!string.IsNullOrEmpty(errors))
                _logger?.LogWarning("[GSP][HOUSE_FLOOR] cleanup-error | reason=" + reason + " | " + errors);
            PendingBindings.Clear();
            _currentIndex = -1;
            _nextDiscoveryRealtime = 0f;
            _report = null;
        }

        private static void StartCurrentAudit()
        {
            if (_report == null || _currentIndex < 0 || _currentIndex >= PendingBindings.Count)
                return;

            HouseFloorRegistry.FloorBinding binding = PendingBindings[_currentIndex];
            HouseFloorProviderItemReport item = _report.Floors[_currentIndex];
            GameObject target = binding.Target;
            if (target == null || !target.activeInHierarchy)
            {
                ReturnCurrentToWaiting("source-deactivated-before-audit-start");
                return;
            }

            string actualPath;
            try { actualPath = TargetFinder.GetPath(target.transform); }
            catch { actualPath = null; }
            if (!string.Equals(actualPath, binding.TargetPath, StringComparison.Ordinal))
            {
                MarkCurrentFailed("target-path-changed-before-audit:expected=" + binding.TargetPath + ",actual=" + actualPath);
                return;
            }

            _report.Running = true;
            _report.CurrentIndex = _currentIndex;
            _report.CurrentRoom = binding.RoomName;
            _report.Stage = "capturing-active-room:" + binding.RoomName;
            _report.Detail = "read-only-geometry-acquisition:" + binding.TargetPath;

            item.CaptureAttempts++;
            item.CaptureWaiting = false;
            item.CaptureFailed = false;
            item.SourceCurrentlyActive = true;
            item.TargetIdentityValid = true;
            item.TargetPath = binding.TargetPath;
            item.TargetLayer = target.layer;
            item.Detail = "CAPTURING";

            string startResult = MultipartObjectAggregator.StartExactTarget(
                _logger,
                target,
                out MultipartAggregationReport ignoredReport);
            _logger?.LogInfo(
                "[GSP][HOUSE_FLOOR] capture-start" +
                " | room=" + binding.RoomName +
                " | target=" + binding.FloorObjectName +
                " | attempt=" + item.CaptureAttempts +
                " | path=" + binding.TargetPath +
                " | result=" + Clean(startResult));

            MultipartAggregationReport current = MultipartObjectAggregator.CurrentReport;
            if (current == null)
                MarkCurrentFailed("地板审计没有创建 report：room=" + binding.RoomName);
        }

        private static bool TryStartNextCapture(bool immediate)
        {
            if (_report == null || !_report.WatcherActive || _report.Running)
                return false;

            if (!immediate && Time.realtimeSinceStartup < _nextDiscoveryRealtime)
                return false;
            _nextDiscoveryRealtime = Time.realtimeSinceStartup + DiscoveryIntervalSeconds;

            RefreshRuntimeTruth();
            for (int i = 0; i < PendingBindings.Count; i++)
            {
                HouseFloorProviderItemReport item = _report.Floors[i];
                if (item.CachedPersistent || item.CaptureFailed)
                    continue;

                HouseFloorRegistry.FloorBinding expected = PendingBindings[i];
                if (!HouseFloorRegistry.TryResolveActive(
                    expected,
                    out HouseFloorRegistry.FloorBinding resolved,
                    out string detail,
                    out bool uniquenessFailure))
                {
                    item.SourceCurrentlyActive = false;
                    item.CaptureWaiting = !uniquenessFailure;
                    item.CaptureFailed = uniquenessFailure;
                    item.Detail = detail;
                    if (uniquenessFailure)
                        _logger?.LogWarning("[GSP][HOUSE_FLOOR] slot-failed | room=" + item.RoomName + " | " + Clean(detail));
                    continue;
                }

                PendingBindings[i] = resolved;
                item.SourceCurrentlyActive = true;
                item.TargetPath = resolved.TargetPath;
                item.TargetIdentityValid = resolved.Target != null;
                item.TargetLayer = resolved.Target != null ? resolved.Target.layer : -1;
                item.CaptureWaiting = false;
                item.Detail = "ACTIVE_DISCOVERED";
                _currentIndex = i;
                StartCurrentAudit();
                return _report.Running;
            }

            RefreshRuntimeTruth();
            if (_report.BuiltFloors == _report.ExpectedFloors && _report.FailedFloors == 0)
                CompleteBuild();
            else
            {
                _report.Stage = _report.FailedFloors > 0 ? "partial-atlas-with-failed-slot" : "waiting-for-active-room";
                _report.Detail = _report.FailedFloors > 0
                    ? "one-or-more-room-slots-failed; rerun gsp_floor_build after investigation"
                    : "inactive rooms are expected; visit another room to continue capture";
            }
            return false;
        }

        private static void ConsumeCurrentAudit(MultipartAggregationReport audit)
        {
            if (_report == null || _currentIndex < 0 || _currentIndex >= PendingBindings.Count)
                return;

            HouseFloorRegistry.FloorBinding binding = PendingBindings[_currentIndex];
            HouseFloorProviderItemReport itemReport = _report.Floors[_currentIndex];

            if (binding.Target == null || !binding.Target.activeInHierarchy)
            {
                ReturnCurrentToWaiting("source-deactivated-during-capture");
                return;
            }

            itemReport.AuditCompleted = true;
            itemReport.AuditSuccess = audit.Success;
            itemReport.VertexCount = audit.CollisionVertices;
            itemReport.IndexCount = audit.CollisionIndices;
            itemReport.TriangleCount = audit.CollisionTriangles;
            if (audit.CollisionRawTopology != null)
                itemReport.RawBoundaryEdges = audit.CollisionRawTopology.BoundaryEdges;
            if (audit.CollisionWeldedTopology != null)
            {
                itemReport.WeldedBoundaryEdges = audit.CollisionWeldedTopology.BoundaryEdges;
                itemReport.WeldedComponents = audit.CollisionWeldedTopology.ConnectedComponents;
                itemReport.NonManifoldEdges = audit.CollisionWeldedTopology.NonManifoldEdges;
                itemReport.OrientationConflicts = audit.CollisionWeldedTopology.OrientationConflicts;
                itemReport.DuplicateTriangles = audit.CollisionWeldedTopology.DuplicateTriangles;
            }

            if (!audit.Success)
            {
                if (binding.Target == null || !binding.Target.activeInHierarchy)
                {
                    ReturnCurrentToWaiting("source-deactivated-during-multipart-audit");
                    return;
                }
                MarkCurrentFailed("multipart-audit-failed:" + audit.Detail);
                return;
            }
            if (!string.Equals(audit.TargetPath, binding.TargetPath, StringComparison.Ordinal))
            {
                MarkCurrentFailed("audit-target-path-mismatch:expected=" + binding.TargetPath + ",actual=" + audit.TargetPath);
                return;
            }
            if (audit.CollisionWeldedTopology == null ||
                itemReport.NonManifoldEdges != 0 ||
                itemReport.OrientationConflicts != 0 ||
                itemReport.DuplicateTriangles != 0)
            {
                MarkCurrentFailed(
                    "topology-safety-gate-failed:nonManifold=" + itemReport.NonManifoldEdges +
                    ",orientationConflict=" + itemReport.OrientationConflicts +
                    ",duplicateTriangles=" + itemReport.DuplicateTriangles);
                return;
            }

            if (!MultipartObjectAggregator.TryCopyCompletedCollisionGeometry(
                out Vector3[] worldVertices,
                out int[] indices,
                out Bounds worldBounds,
                out string snapshotTargetName,
                out string snapshotTargetPath,
                out string snapshotError))
            {
                MarkCurrentFailed("collision-snapshot-not-ready:" + snapshotError);
                return;
            }
            if (!string.Equals(snapshotTargetPath, binding.TargetPath, StringComparison.Ordinal) ||
                !string.Equals(snapshotTargetName, binding.FloorObjectName, StringComparison.OrdinalIgnoreCase))
            {
                MarkCurrentFailed("collision-snapshot-identity-mismatch");
                return;
            }

            if (!BuildFloor(binding, itemReport, worldVertices, indices, worldBounds, out string buildError))
            {
                if (buildError != null && buildError.StartsWith("source-target-inactive", StringComparison.Ordinal))
                {
                    ReturnCurrentToWaiting(buildError);
                    return;
                }
                MarkCurrentFailed("provider-build-failed:" + buildError);
                return;
            }

            RuntimeFloor floor = FindRuntimeFloor(binding.RoomName);
            if (floor == null)
            {
                MarkCurrentFailed("provider-created-but-runtime-floor-not-found");
                return;
            }

            try
            {
                if (floor.Renderer != null)
                    floor.Renderer.enabled = _report.DesiredVisual;
            }
            catch (Exception e)
            {
                RemoveRuntimeFloor(binding.RoomName);
                MarkCurrentFailed("desired-visual-apply-failed:" + e.GetType().Name + ":" + e.Message);
                return;
            }

            if (_report.DesiredCollider)
            {
                if (!ValidateFloorBeforeEnable(floor, out string enableError))
                {
                    RemoveRuntimeFloor(binding.RoomName);
                    MarkCurrentFailed("new-floor-enable-preflight-failed:" + enableError);
                    return;
                }
                try
                {
                    floor.Collider.enabled = true;
                    Physics.SyncTransforms();
                }
                catch (Exception e)
                {
                    RemoveRuntimeFloor(binding.RoomName);
                    MarkCurrentFailed("new-floor-enable-failed:" + e.GetType().Name + ":" + e.Message);
                    return;
                }
                if (!floor.Collider.enabled || !BoundsApproximatelyEqual(floor.Collider.bounds, floor.ExpectedBounds))
                {
                    RemoveRuntimeFloor(binding.RoomName);
                    MarkCurrentFailed("new-floor-enable-readback-failed");
                    return;
                }
            }

            _logger?.LogWarning(
                "[GSP][HOUSE_FLOOR] floor-captured" +
                " | room=" + binding.RoomName +
                " | target=" + binding.FloorObjectName +
                " | path=" + binding.TargetPath +
                " | layer=" + itemReport.TargetLayer +
                " | vertices=" + itemReport.VertexCount +
                " | triangles=" + itemReport.TriangleCount +
                " | yRange=" + itemReport.MinY.ToString("F4") + ".." + itemReport.MaxY.ToString("F4") +
                " | sourceMayDeactivateAfterCapture=True" +
                " | cachedPersistent=True" +
                " | colliderEnabled=" + floor.Collider.enabled +
                " | Rigidbody=0");

            _report.Running = false;
            _report.CurrentIndex = -1;
            _report.CurrentRoom = null;
            _currentIndex = -1;
            RefreshRuntimeTruth();
            if (_report.BuiltFloors == _report.ExpectedFloors && _report.FailedFloors == 0)
                CompleteBuild();
            else
            {
                _report.Stage = "waiting-for-active-room";
                _report.Detail = "capture-complete-for-one-room;waiting-for-next-room-activation";
                TryStartNextCapture(true);
            }
        }

        private static void ReturnCurrentToWaiting(string detail)
        {
            if (_report == null || _currentIndex < 0 || _currentIndex >= _report.Floors.Count)
                return;
            HouseFloorProviderItemReport item = _report.Floors[_currentIndex];
            item.AuditCompleted = false;
            item.AuditSuccess = false;
            item.SourceCurrentlyActive = false;
            item.CaptureWaiting = true;
            item.CaptureFailed = false;
            item.Detail = "WAITING_ACTIVE_AFTER_TRANSIENT_DEACTIVATION:" + detail;
            MultipartObjectAggregator.Cancel("runtime-clear:house-floor-transient-deactivation");
            _report.Running = false;
            _report.CurrentIndex = -1;
            _report.CurrentRoom = null;
            _currentIndex = -1;
            _report.Stage = "waiting-for-active-room";
            _report.Detail = detail;
            _nextDiscoveryRealtime = Time.realtimeSinceStartup + DiscoveryIntervalSeconds;
            RefreshRuntimeTruth();
        }

        private static void MarkCurrentFailed(string detail)
        {
            if (_report == null || _currentIndex < 0 || _currentIndex >= _report.Floors.Count)
                return;
            HouseFloorProviderItemReport item = _report.Floors[_currentIndex];
            item.CaptureWaiting = false;
            item.CaptureFailed = true;
            item.Detail = detail;
            MultipartObjectAggregator.Cancel("runtime-clear:house-floor-slot-failed");
            _logger?.LogWarning("[GSP][HOUSE_FLOOR] slot-failed | room=" + item.RoomName + " | " + Clean(detail));
            _report.Running = false;
            _report.CurrentIndex = -1;
            _report.CurrentRoom = null;
            _currentIndex = -1;
            _report.Stage = "partial-atlas-with-failed-slot";
            _report.Detail = detail;
            RefreshRuntimeTruth();
            TryStartNextCapture(true);
        }

        private static RuntimeFloor FindRuntimeFloor(string roomName)
        {
            for (int i = 0; i < Floors.Count; i++)
            {
                RuntimeFloor floor = Floors[i];
                if (floor != null && floor.Binding != null &&
                    string.Equals(floor.Binding.RoomName, roomName, StringComparison.OrdinalIgnoreCase))
                    return floor;
            }
            return null;
        }

        private static void RemoveRuntimeFloor(string roomName)
        {
            for (int i = Floors.Count - 1; i >= 0; i--)
            {
                RuntimeFloor floor = Floors[i];
                if (floor == null || floor.Binding == null ||
                    !string.Equals(floor.Binding.RoomName, roomName, StringComparison.OrdinalIgnoreCase))
                    continue;
                DestroyFloor(floor, out string cleanupError);
                Floors.RemoveAt(i);
                if (!string.IsNullOrEmpty(cleanupError))
                    _logger?.LogWarning("[GSP][HOUSE_FLOOR] partial-floor-cleanup-error | room=" + roomName + " | " + cleanupError);
            }

            if (_report != null)
            {
                for (int i = 0; i < _report.Floors.Count; i++)
                {
                    HouseFloorProviderItemReport item = _report.Floors[i];
                    if (!string.Equals(item.RoomName, roomName, StringComparison.OrdinalIgnoreCase))
                        continue;
                    item.CachedPersistent = false;
                    item.ColliderCreated = false;
                    item.ColliderEnabled = false;
                    item.MeshCreated = false;
                    item.ProviderRigidbodies = 0;
                    item.ProviderLayer = -1;
                    break;
                }
            }
        }

        private static bool BuildFloor(
            HouseFloorRegistry.FloorBinding binding,
            HouseFloorProviderItemReport report,
            Vector3[] worldVertices,
            int[] indices,
            Bounds worldBounds,
            out string error)
        {
            error = null;
            if (!ValidateGeometry(worldVertices, indices, out error))
                return false;

            GameObject target = binding.Target;
            if (target == null || !target.activeInHierarchy)
            {
                error = "source-target-inactive-before-build";
                return false;
            }
            string actualPath;
            try { actualPath = TargetFinder.GetPath(target.transform); }
            catch { actualPath = null; }
            if (!string.Equals(actualPath, binding.TargetPath, StringComparison.Ordinal))
            {
                error = "target-path-drift-before-build:actual=" + actualPath;
                return false;
            }

            report.WorldBounds = "center=" + worldBounds.center.ToString("G9") + "/size=" + worldBounds.size.ToString("G9");
            report.MinY = worldBounds.min.y;
            report.MaxY = worldBounds.max.y;

            RuntimeFloor floor = new RuntimeFloor
            {
                Binding = binding,
                Report = report,
                ExpectedPosition = worldBounds.center,
                ExpectedRotation = Quaternion.identity,
                ExpectedScale = Vector3.one,
                ExpectedBounds = worldBounds,
                ExpectedLayer = target.layer
            };

            try
            {
                Vector3[] localVertices = new Vector3[worldVertices.Length];
                for (int i = 0; i < worldVertices.Length; i++)
                    localVertices[i] = worldVertices[i] - worldBounds.center;

                floor.Mesh = new Mesh
                {
                    name = "GSP_HouseFloorMesh_" + Sanitize(binding.RoomName),
                    indexFormat = localVertices.Length >= 65000 ? IndexFormat.UInt32 : IndexFormat.UInt16
                };
                floor.Mesh.vertices = localVertices;
                floor.Mesh.subMeshCount = 1;
                floor.Mesh.SetIndices(indices, MeshTopology.Triangles, 0, false);
                floor.Mesh.RecalculateBounds();
                floor.Mesh.RecalculateNormals();
                report.MeshCreated = true;

                floor.Material = CreateMaterial(binding.RoomName);
                if (floor.Material == null)
                    throw new InvalidOperationException("cannot-create-floor-preview-material");

                floor.Root = new GameObject("GSP_HouseFloor_" + Sanitize(binding.RoomName));
                floor.Root.transform.position = floor.ExpectedPosition;
                floor.Root.transform.rotation = floor.ExpectedRotation;
                floor.Root.transform.localScale = floor.ExpectedScale;
                floor.Root.layer = floor.ExpectedLayer;

                MeshFilter filter = floor.Root.AddComponent<MeshFilter>();
                if (filter == null)
                    throw new InvalidOperationException("cannot-add-MeshFilter");
                filter.sharedMesh = floor.Mesh;

                floor.Renderer = floor.Root.AddComponent<MeshRenderer>();
                if (floor.Renderer == null)
                    throw new InvalidOperationException("cannot-add-MeshRenderer");
                floor.Renderer.sharedMaterial = floor.Material;
                floor.Renderer.enabled = false;

                floor.Collider = floor.Root.AddComponent<MeshCollider>();
                if (floor.Collider == null)
                    throw new InvalidOperationException("cannot-add-MeshCollider");
                floor.Collider.sharedMesh = floor.Mesh;
                floor.Collider.convex = false;
                floor.Collider.isTrigger = false;
                floor.Collider.enabled = true;
                try { Physics.SyncTransforms(); } catch { }

                if (floor.Collider.attachedRigidbody != null || floor.Root.GetComponent<Rigidbody>() != null)
                    throw new InvalidOperationException("floor-provider-rigidbody-present");
                if (!BoundsApproximatelyEqual(floor.Collider.bounds, worldBounds))
                    throw new InvalidOperationException("floor-collider-bounds-mismatch");

                floor.Collider.enabled = false;
                try { Physics.SyncTransforms(); } catch { }

                Floors.Add(floor);
                report.ColliderCreated = true;
                report.ColliderEnabled = false;
                report.ProviderRigidbodies = 0;
                report.TargetIdentityValid = true;
                report.SourceCurrentlyActive = target.activeInHierarchy;
                report.SourceActiveAtCapture = true;
                report.CachedPersistent = true;
                report.CaptureWaiting = false;
                report.CaptureFailed = false;
                report.TargetLayer = target.layer;
                report.ProviderLayer = floor.Root.layer;
                report.LayerIdentityValid = report.TargetLayer == report.ProviderLayer;
                AuditSourceNativePhysics(target, report);
                report.Detail = "CAPTURED_GSP_OWNED_STATIC_NONCONVEX_FLOOR";
                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + ":" + e.Message;
                DestroyFloor(floor, out string cleanupError);
                if (!string.IsNullOrEmpty(cleanupError))
                    error += ";cleanup=" + cleanupError;
                return false;
            }
        }

        private static void CompleteBuild()
        {
            RefreshRuntimeTruth();
            _report.Running = false;
            _report.Completed = _report.BuiltFloors == _report.ExpectedFloors;
            _report.Success = _report.Completed && _report.FailedFloors == 0;
            _report.Operational = _report.BuiltFloors > 0;
            _report.Stage = _report.Success ? "atlas-complete" : "waiting-for-active-room";
            _report.CurrentIndex = -1;
            _report.CurrentRoom = null;
            _report.Detail = _report.Success
                ? "four-room-floor-atlas-complete;cached-floors-persist-across-room-deactivation"
                : "progressive-capture-incomplete";

            if (_report.Success)
                _logger?.LogWarning("[GSP][HOUSE_FLOOR] atlas-complete | " + _report.ToStatusText());
        }

        private static bool ValidateAllBeforeEnable(out string error)
        {
            error = null;
            if (Floors.Count == 0)
            {
                error = "no-captured-floor";
                return false;
            }

            for (int i = 0; i < Floors.Count; i++)
            {
                RuntimeFloor floor = Floors[i];
                if (!ValidateFloorBeforeEnable(floor, out error))
                {
                    error = floor.Binding.RoomName + ":" + error;
                    return false;
                }
            }
            return true;
        }

        private static bool ValidateFloorBeforeEnable(RuntimeFloor floor, out string error)
        {
            error = null;
            if (floor == null || floor.Root == null || floor.Mesh == null || floor.Collider == null)
            {
                error = "owned-runtime-object-missing";
                return false;
            }
            if (!floor.Report.CachedPersistent || !floor.Report.SourceActiveAtCapture || string.IsNullOrEmpty(floor.Binding.TargetPath))
            {
                error = "capture-provenance-invalid";
                return false;
            }
            if (floor.Root.layer != floor.ExpectedLayer)
            {
                error = "provider-layer-drift:provider=" + floor.Root.layer + ",expected=" + floor.ExpectedLayer;
                return false;
            }
            if (Vector3.Distance(floor.Root.transform.position, floor.ExpectedPosition) > 0.0005f ||
                Quaternion.Angle(floor.Root.transform.rotation, floor.ExpectedRotation) > 0.05f ||
                Vector3.Distance(floor.Root.transform.localScale, floor.ExpectedScale) > 0.0005f)
            {
                error = "provider-transform-drift";
                return false;
            }
            if (floor.Collider.sharedMesh != floor.Mesh || floor.Collider.convex || floor.Collider.isTrigger)
            {
                error = "collider-shape-contract-drift";
                return false;
            }
            if (floor.Collider.attachedRigidbody != null || floor.Root.GetComponent<Rigidbody>() != null)
            {
                error = "provider-rigidbody-present";
                return false;
            }

            // 源房间当前 active 时额外验证 source identity/layer；源房间失活本身不是失败。
            if (HouseFloorRegistry.TryResolveActive(
                floor.Binding,
                out HouseFloorRegistry.FloorBinding active,
                out string activeDetail,
                out bool uniquenessFailure))
            {
                if (!string.Equals(active.TargetPath, floor.Binding.TargetPath, StringComparison.Ordinal))
                {
                    error = "active-source-identity-drift:captured=" + floor.Binding.TargetPath + ",current=" + active.TargetPath;
                    return false;
                }
                if (active.Target == null || active.Target.layer != floor.ExpectedLayer)
                {
                    error = "active-source-layer-drift:current=" + (active.Target != null ? active.Target.layer : -1) +
                        ",expected=" + floor.ExpectedLayer;
                    return false;
                }
            }
            else if (uniquenessFailure)
            {
                error = "active-source-uniqueness-failed:" + activeDetail;
                return false;
            }

            return true;
        }

        private static bool ValidateAllAfterEnable(out string error)
        {
            error = null;
            for (int i = 0; i < Floors.Count; i++)
            {
                RuntimeFloor floor = Floors[i];
                if (floor.Collider == null || !floor.Collider.enabled)
                {
                    error = floor.Binding.RoomName + ":collider-not-enabled";
                    return false;
                }
                if (!BoundsApproximatelyEqual(floor.Collider.bounds, floor.ExpectedBounds))
                {
                    error = floor.Binding.RoomName + ":bounds-readback-mismatch";
                    return false;
                }
            }
            return true;
        }

        private static void RefreshRuntimeTruth()
        {
            if (_report == null)
                return;

            int built = 0;
            int enabled = 0;
            int activeSources = 0;
            int waiting = 0;
            int failed = 0;
            int resolved = 0;
            int audited = 0;

            for (int i = 0; i < _report.Floors.Count; i++)
            {
                HouseFloorProviderItemReport item = _report.Floors[i];
                RuntimeFloor floor = FindRuntimeFloor(item.RoomName);
                if (floor != null && floor.Root != null && floor.Mesh != null && floor.Collider != null)
                {
                    built++;
                    item.CachedPersistent = true;
                    item.ColliderCreated = true;
                    item.ColliderEnabled = floor.Collider.enabled;
                    if (item.ColliderEnabled) enabled++;
                    item.ProviderRigidbodies = floor.Root.GetComponent<Rigidbody>() != null ? 1 : 0;
                    item.ProviderLayer = floor.Root.layer;
                    item.TargetIdentityValid = !string.IsNullOrEmpty(item.TargetPath);
                    item.LayerIdentityValid = item.ProviderLayer == floor.ExpectedLayer && item.TargetLayer == floor.ExpectedLayer;
                }

                HouseFloorRegistry.FloorBinding expected = i < PendingBindings.Count ? PendingBindings[i] : null;
                HouseFloorRegistry.FloorBinding active = null;
                string activeDetail = null;
                bool uniquenessFailure = false;
                bool sourceResolved = expected != null && HouseFloorRegistry.TryResolveActive(
                    expected,
                    out active,
                    out activeDetail,
                    out uniquenessFailure);
                if (sourceResolved)
                {
                    item.SourceCurrentlyActive = true;
                    activeSources++;
                    if (!string.IsNullOrEmpty(active.TargetPath)) resolved++;

                    if (item.CachedPersistent)
                    {
                        item.TargetIdentityValid = string.Equals(item.TargetPath, active.TargetPath, StringComparison.Ordinal);
                        item.TargetLayer = active.Target != null ? active.Target.layer : item.TargetLayer;
                        item.LayerIdentityValid = item.TargetIdentityValid && floor != null && floor.Root != null &&
                            item.ProviderLayer == item.TargetLayer;
                        if (active.Target != null) AuditSourceNativePhysics(active.Target, item);
                    }
                }
                else
                {
                    item.SourceCurrentlyActive = false;
                    if (uniquenessFailure && !item.CachedPersistent)
                    {
                        item.CaptureFailed = true;
                        item.CaptureWaiting = false;
                        item.Detail = activeDetail;
                    }
                }

                if (!string.IsNullOrEmpty(item.TargetPath) && !item.SourceCurrentlyActive) resolved++;
                if (item.AuditCompleted && item.AuditSuccess) audited++;
                if (item.CaptureFailed) failed++;
                else if (!item.CachedPersistent && !(_report.Running && _report.CurrentIndex == i))
                {
                    item.CaptureWaiting = true;
                    waiting++;
                    if (string.IsNullOrEmpty(item.Detail) || item.Detail == "UNSEEN") item.Detail = "WAITING_ACTIVE";
                }
            }

            _report.ResolvedFloors = Math.Min(resolved, _report.ExpectedFloors);
            _report.AuditedFloors = audited;
            _report.BuiltFloors = built;
            _report.EnabledFloors = enabled;
            _report.ActiveSourceFloors = activeSources;
            _report.WaitingFloors = waiting;
            _report.FailedFloors = failed;
            _report.Operational = built > 0;
            _report.Completed = built == _report.ExpectedFloors && failed == 0;
            _report.Success = _report.Completed;
        }

        private static string ClearOwnedFloors(string reason)
        {
            string errors = null;
            ForceAllCollidersOff();
            for (int i = Floors.Count - 1; i >= 0; i--)
            {
                DestroyFloor(Floors[i], out string itemError);
                if (!string.IsNullOrEmpty(itemError))
                    errors = Append(errors, itemError);
            }
            Floors.Clear();
            PendingBindings.Clear();
            _currentIndex = -1;
            _lastCleanupError = errors;
            if (!string.IsNullOrEmpty(errors))
                _logger?.LogWarning("[GSP][HOUSE_FLOOR] owned-cleanup-error | reason=" + reason + " | " + errors);
            return errors;
        }

        private static void DestroyFloor(RuntimeFloor floor, out string error)
        {
            error = null;
            if (floor == null)
                return;
            try { if (floor.Collider != null) floor.Collider.enabled = false; }
            catch (Exception e) { error = Append(error, "disable:" + e.GetType().Name + ":" + e.Message); }
            try { if (floor.Root != null) Object.Destroy(floor.Root); }
            catch (Exception e) { error = Append(error, "root:" + e.GetType().Name + ":" + e.Message); }
            try { if (floor.Mesh != null) Object.Destroy(floor.Mesh); }
            catch (Exception e) { error = Append(error, "mesh:" + e.GetType().Name + ":" + e.Message); }
            try { if (floor.Material != null) Object.Destroy(floor.Material); }
            catch (Exception e) { error = Append(error, "material:" + e.GetType().Name + ":" + e.Message); }
        }

        private static void ForceAllCollidersOff()
        {
            for (int i = 0; i < Floors.Count; i++)
            {
                try { if (Floors[i].Collider != null) Floors[i].Collider.enabled = false; }
                catch { }
            }
            try { Physics.SyncTransforms(); }
            catch { }
        }

        private static void AuditSourceNativePhysics(GameObject target, HouseFloorProviderItemReport report)
        {
            report.SourceNativeColliders = 0;
            report.SourceNativeEnabledColliders = 0;
            report.SourceNativeRigidbodies = 0;
            if (target == null)
                return;
            try
            {
                Collider[] colliders = target.GetComponentsInChildren<Collider>(true);
                for (int i = 0; i < colliders.Length; i++)
                {
                    Collider collider = colliders[i];
                    if (collider == null || (collider.gameObject.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal))
                        continue;
                    report.SourceNativeColliders++;
                    if (collider.enabled)
                        report.SourceNativeEnabledColliders++;
                }
            }
            catch { }
            try
            {
                Rigidbody[] bodies = target.GetComponentsInChildren<Rigidbody>(true);
                for (int i = 0; i < bodies.Length; i++)
                    if (bodies[i] != null)
                        report.SourceNativeRigidbodies++;
            }
            catch { }
        }

        private static bool ValidateGeometry(Vector3[] vertices, int[] indices, out string error)
        {
            error = null;
            if (vertices == null || vertices.Length < 3)
            {
                error = "vertex-array-empty";
                return false;
            }
            if (indices == null || indices.Length < 3 || indices.Length % 3 != 0)
            {
                error = "index-array-invalid";
                return false;
            }
            for (int i = 0; i < vertices.Length; i++)
            {
                if (!IsFinite(vertices[i]))
                {
                    error = "non-finite-vertex:" + i;
                    return false;
                }
            }
            for (int i = 0; i < indices.Length; i++)
            {
                if (indices[i] < 0 || indices[i] >= vertices.Length)
                {
                    error = "index-out-of-range:" + i + "=" + indices[i];
                    return false;
                }
            }
            return true;
        }

        private static Material CreateMaterial(string roomName)
        {
            Shader shader = Shader.Find("Legacy Shaders/Transparent/Diffuse");
            if (shader == null) shader = Shader.Find("Sprites/Default");
            if (shader == null) shader = Shader.Find("Standard");
            if (shader == null) return null;

            Color color = new Color(0.10f, 0.75f, 1.00f, 0.28f);
            Material material = new Material(shader)
            {
                name = "GSP_HouseFloorMaterial_" + Sanitize(roomName),
                color = color
            };
            try
            {
                if (material.HasProperty("_Color")) material.SetColor("_Color", color);
                if (material.HasProperty("_ZWrite")) material.SetInt("_ZWrite", 0);
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

        private static string Fail(string message)
        {
            if (_report != null)
            {
                _report.Running = false;
                _report.WatcherActive = false;
                _report.Completed = true;
                _report.Success = false;
                _report.Operational = false;
                _report.Stage = "fatal-failed";
                _report.Detail = message;
            }
            MultipartObjectAggregator.Cancel("runtime-clear:house-floor-fatal-failed");
            string cleanup = ClearOwnedFloors("house-floor-fatal-failed");
            if (!string.IsNullOrEmpty(cleanup) && _report != null)
                _report.Detail += ";cleanup=" + cleanup;
            _logger?.LogWarning("[GSP][HOUSE_FLOOR] fatal-failed | " + message);
            return "<color=red>GSP house-floor atlas 启动失败：</color> " + message;
        }

        private static string FailRuntime(string message)
        {
            ForceAllCollidersOff();
            if (_report != null)
                _report.DesiredCollider = false;
            RefreshRuntimeTruth();
            if (_report != null)
            {
                _report.Operational = _report.BuiltFloors > 0;
                _report.Stage = "runtime-fail-close";
                _report.Detail = message;
            }
            _logger?.LogWarning("[GSP][HOUSE_FLOOR] runtime-fail-close | " + message);
            return "<color=red>GSP house-floor atlas 运行时安全门失败：</color> " + message;
        }

        private static string Append(string current, string item)
        {
            return string.IsNullOrEmpty(current) ? item : current + ";" + item;
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrWhiteSpace(value)) return "Unnamed";
            char[] chars = value.ToCharArray();
            for (int i = 0; i < chars.Length; i++)
            {
                char c = chars[i];
                if (!(char.IsLetterOrDigit(c) || c == '_' || c == '-')) chars[i] = '_';
            }
            return new string(chars);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value)) return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
