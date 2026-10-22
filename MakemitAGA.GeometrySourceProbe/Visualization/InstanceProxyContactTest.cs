using System;
using System.Collections.Generic;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    /// <summary>
    /// v0.1.23 主线：对已经创建的静态定向盒代理执行可回滚真实接触验证。
    ///
    /// 每个代理依次测试正面中心、侧面中心、正面边缘三个位置。测试球无重力、
    /// 每次只存在一个；只接受与当前目标 BoxCollider 完全匹配的 OnCollisionEnter。
    /// 测试开始时记录代理 Collider 原状态，完成/取消/场景切换时恢复。
    /// </summary>
    internal static class InstanceProxyContactTest
    {
        private sealed class PlannedCase
        {
            public SpatialInstanceProxyContactTarget Target;
            public string Kind;
            public int FaceAxis;
            public Vector2 NormalizedFaceOffset;
        }

        private const float ProbeRadius = 0.018f;
        private const float SpawnGap = 0.085f;
        private const float LaunchSpeed = 1.35f;
        private const float CaseTimeoutSeconds = 1.25f;
        private const float PostContactHoldSeconds = 0.12f;
        private const float NormalEpsilon = 0.000001f;
        private const float NormalAlignmentThreshold = 0.90f;
        private const float SurfacePointTolerance = 0.055f;

        private static readonly List<SpatialInstanceProxyContactTarget> Targets = new List<SpatialInstanceProxyContactTarget>();
        private static readonly List<PlannedCase> Plan = new List<PlannedCase>();
        private static ManualLogSource _logger;
        private static InstanceProxyContactTestReport _report;
        private static int _nextCaseIndex;
        private static int _activeProbeId;
        private static int _nextProbeId = 1;
        private static PlannedCase _activeCase;
        private static InstanceProxyContactCaseReport _activeCaseReport;
        private static GameObject _probeObject;
        private static Rigidbody _probeBody;
        private static SphereCollider _probeCollider;
        private static float _sessionStartTime;
        private static float _caseStartTime;
        private static float _finishCaseAt;
        private static bool _contactObserved;
        private static bool _collidersWereEnabled;
        private static bool _running;

        public static bool IsRunning => _running;

        public static string Start(ManualLogSource logger)
        {
            Stop("restart", false);
            _logger = logger;
            _report = new InstanceProxyContactTestReport
            {
                Started = true,
                Running = false,
                Detail = "validating-runtime-proxies"
            };

            if (!SpatialInstanceProxyRuntime.TryGetContactTargets(
                Targets,
                out string targetName,
                out bool collidersEnabled,
                out string error))
            {
                _report.Completed = true;
                _report.Success = false;
                _report.Detail = error;
                return "<color=red>实例代理接触测试无法开始：</color> " + error;
            }

            _collidersWereEnabled = collidersEnabled;
            _report.TargetName = targetName;
            _report.ProxyCount = Targets.Count;
            _report.ProxyCollidersWereEnabled = _collidersWereEnabled;

            for (int i = 0; i < Targets.Count; i++)
            {
                SpatialInstanceProxyContactTarget target = Targets[i];
                Plan.Add(new PlannedCase
                {
                    Target = target,
                    Kind = "FRONT_CENTER",
                    FaceAxis = 2,
                    NormalizedFaceOffset = Vector2.zero
                });
                Plan.Add(new PlannedCase
                {
                    Target = target,
                    Kind = "SIDE_CENTER",
                    FaceAxis = 0,
                    NormalizedFaceOffset = Vector2.zero
                });
                Plan.Add(new PlannedCase
                {
                    Target = target,
                    Kind = "FRONT_EDGE",
                    FaceAxis = 2,
                    NormalizedFaceOffset = new Vector2(0.76f, 0f)
                });
            }

            _report.PlannedCases = Plan.Count;
            _sessionStartTime = Time.time;
            _nextCaseIndex = 0;
            if (!SpatialInstanceProxyRuntime.TrySetCollidersForContactTest(true, out string enableError))
            {
                _report.Completed = true;
                _report.Success = false;
                _report.Detail = "provider-enable-preflight-failed:" + enableError;
                Targets.Clear();
                Plan.Clear();
                return "<color=red>实例代理接触测试无法开始：</color> provider 二次启用安全门失败：" + enableError;
            }

            _running = true;
            _report.Running = true;
            _report.Detail = "controlled rigidbody probes active";
            Physics.SyncTransforms();

            _logger?.LogWarning(
                "[GSP][INSTANCE_PROXY_CONTACT] begin" +
                " | target=" + targetName +
                " | proxies=" + Targets.Count +
                " | plannedCases=" + Plan.Count +
                " | collidersBefore=" + _collidersWereEnabled +
                " | gravity=False" +
                " | oneTemporaryRigidbodyAtATime=True" +
                " | contactGeometry=managed-obb-closest-surface" +
                " | pointTolerance=" + SurfacePointTolerance.ToString("F3") +
                " | normalAlignmentThreshold=" + NormalAlignmentThreshold.ToString("F2"));

            return
                "<color=#66ddff>实例代理受控接触测试已开始。</color> " +
                "proxies=" + Targets.Count +
                "，cases=" + Plan.Count +
                "，每个代理测试正面/侧面/边缘，并用托管 OBB 算法验证表面点与法线；完成后自动恢复 Collider 状态。";
        }

        public static void Update()
        {
            if (!_running || _report == null)
                return;

            _report.ElapsedSeconds = Math.Max(0f, Time.time - _sessionStartTime);
            _report.TemporaryRigidbodiesRemaining = _probeBody != null ? 1 : 0;

            if (_activeCaseReport == null)
                return;

            float elapsed = Time.time - _caseStartTime;
            if (_contactObserved)
            {
                if (Time.time >= _finishCaseAt)
                {
                    bool passed = StrictCasePassed(_activeCaseReport);
                    FinishActiveCase(
                        passed,
                        false,
                        passed
                            ? "matching OnCollisionEnter plus managed OBB surface gate passed"
                            : BuildStrictGateFailureDetail(_activeCaseReport));
                }
                return;
            }

            if (elapsed >= CaseTimeoutSeconds)
                FinishActiveCase(false, true, "timeout waiting for matching OnCollisionEnter");
        }

        public static void FixedUpdate()
        {
            if (!_running)
                return;

            if (_activeCaseReport != null)
                return;

            if (_nextCaseIndex >= Plan.Count)
            {
                Complete();
                return;
            }

            SpawnNextCase();
        }

        public static void NotifyCollisionEnter(int probeId, Collision collision)
        {
            if (!_running || probeId <= 0 || probeId != _activeProbeId ||
                _activeCase == null || _activeCase.Target == null ||
                _activeCase.Target.Collider == null || collision == null ||
                _activeCaseReport == null)
            {
                return;
            }

            // 只读取 collision.collider 用于确认实际命中的目标代理。
            // 不调用 Collision.GetContact / contacts / contactCount：
            // MiSide 当前 IL2CPP interop 对 ContactPoint 桥接会触发 MissingMethodException。
            Collider hit = collision.collider;
            if (hit == null ||
                hit.GetInstanceID() != _activeCase.Target.Collider.GetInstanceID())
            {
                return;
            }

            _contactObserved = true;
            _finishCaseAt = Time.time + PostContactHoldSeconds;
            _activeCaseReport.CollisionEventObserved = true;
            _activeCaseReport.TargetColliderMatched = true;
            _activeCaseReport.ContactCount++;

            // 碰撞回调位于物理步内，优先读取 Rigidbody.position；
            // 插值开启时 Transform.position 可能仍是渲染插值位置。
            Vector3 sphereCenter = _probeBody != null
                ? _probeBody.position
                : (_probeObject != null
                    ? _probeObject.transform.position
                    : Vector3.zero);
            _activeCaseReport.ProbeCenterAtEvent = sphereCenter;

            if (TryComputeManagedObbSurface(
                _activeCase.Target.Collider,
                sphereCenter,
                out Vector3 surfacePoint,
                out Vector3 surfaceNormal,
                out float centerToSurfaceDistance,
                out string geometryDetail))
            {
                _activeCaseReport.ContactPoint = surfacePoint;
                _activeCaseReport.ContactNormal = surfaceNormal;
                _activeCaseReport.ContactPointValid = IsFinite(surfacePoint);
                _activeCaseReport.ContactNormalValid =
                    IsFinite(surfaceNormal) &&
                    surfaceNormal.sqrMagnitude > NormalEpsilon * NormalEpsilon;
                _activeCaseReport.CenterToSurfaceDistance =
                    centerToSurfaceDistance;
                _activeCaseReport.SurfacePointError =
                    Vector3.Distance(
                        surfacePoint,
                        _activeCaseReport.ExpectedSurfacePoint);
                _activeCaseReport.SurfacePointWithinTolerance =
                    _activeCaseReport.ContactPointValid &&
                    _activeCaseReport.SurfacePointError <= SurfacePointTolerance;

                if (_activeCaseReport.ContactNormalValid)
                {
                    Vector3 normal = surfaceNormal.normalized;
                    _activeCaseReport.ContactNormal = normal;
                    _activeCaseReport.NormalAlignment =
                        Vector3.Dot(
                            normal,
                            _activeCaseReport.ExpectedNormal.normalized);
                    _activeCaseReport.NormalAlignmentPassed =
                        _activeCaseReport.NormalAlignment >=
                        NormalAlignmentThreshold;
                }

                _activeCaseReport.ContactGeometrySource =
                    "managed-obb-closest-surface";
                _activeCaseReport.GeometryDetail = geometryDetail;
            }
            else
            {
                _activeCaseReport.ContactGeometrySource =
                    "managed-obb-closest-surface";
                _activeCaseReport.GeometryDetail =
                    "analytic surface computation failed: " + geometryDetail;
            }

            _logger?.LogInfo(
                "[GSP][INSTANCE_PROXY_CONTACT][ENTER] " +
                _activeCaseReport.ToLogLine());
        }

        public static string GetStatusText()
        {
            if (_report == null)
                return "当前没有实例代理接触测试报告。";
            _report.ElapsedSeconds = _report.Completed
                ? _report.ElapsedSeconds
                : Math.Max(0f, Time.time - _sessionStartTime);
            _report.TemporaryRigidbodiesRemaining = _probeBody != null ? 1 : 0;
            return _report.ToStatusText();
        }

        public static string GetReportText()
        {
            if (_report == null)
                return "当前没有实例代理接触测试报告。";
            return _report.ToDetailedText();
        }

        public static string Stop(string reason, bool userVisible)
        {
            bool existed = _running || _probeObject != null || Plan.Count > 0;
            CleanupProbe();

            if (_running)
            {
                _running = false;
                if (_report != null)
                {
                    _report.Running = false;
                    _report.Completed = true;
                    _report.Cancelled = true;
                    _report.Success = false;
                    _report.ElapsedSeconds = Math.Max(0f, Time.time - _sessionStartTime);
                    _report.Detail = "cancelled: " + (reason ?? "<none>");
                }
            }

            if (existed)
                RestoreColliderState();
            Targets.Clear();
            Plan.Clear();
            _activeCase = null;
            _activeCaseReport = null;
            _contactObserved = false;
            _activeProbeId = 0;

            if (existed)
            {
                _logger?.LogInfo(
                    "[GSP][INSTANCE_PROXY_CONTACT] stopped" +
                    " | reason=" + (reason ?? "<none>") +
                    " | collidersRestored=True");
            }

            if (userVisible)
                return existed
                    ? "实例代理接触测试已取消；临时刚体已删除，代理 Collider 状态已恢复。"
                    : "当前没有正在运行的实例代理接触测试。";
            return string.Empty;
        }

        private static void SpawnNextCase()
        {
            _activeCase = Plan[_nextCaseIndex++];
            SpatialInstanceProxyContactTarget target = _activeCase.Target;
            if (target == null || target.Collider == null || target.Transform == null)
            {
                CreateImmediateFailure("target proxy destroyed before case start");
                return;
            }

            // 使用 BoxCollider 自己的 center / size 生成样本点，而不是假定永远是
            // 默认 unit cube。这样未来代理若调整局部 Collider 参数，测试仍保持正确。
            BoxCollider box = target.Collider;
            Vector3 half = box.size * 0.5f;
            Vector3 localFacePoint = box.center;
            Vector3 localOutward;
            if (_activeCase.FaceAxis == 0)
            {
                localFacePoint.x += half.x;
                localFacePoint.y +=
                    _activeCase.NormalizedFaceOffset.x * half.y;
                localFacePoint.z +=
                    _activeCase.NormalizedFaceOffset.y * half.z;
                localOutward = Vector3.right;
            }
            else
            {
                localFacePoint.x +=
                    _activeCase.NormalizedFaceOffset.x * half.x;
                localFacePoint.y +=
                    _activeCase.NormalizedFaceOffset.y * half.y;
                localFacePoint.z += half.z;
                localOutward = Vector3.forward;
            }
            Vector3 surfacePoint =
                target.Transform.TransformPoint(localFacePoint);
            Vector3 expectedNormal =
                TransformLocalNormal(target.Transform, localOutward);
            Vector3 spawn = surfacePoint + expectedNormal * (ProbeRadius + SpawnGap);
            Vector3 velocity = -expectedNormal * LaunchSpeed;

            _activeCaseReport = new InstanceProxyContactCaseReport
            {
                Sequence = _report.Cases.Count + 1,
                ProxyIndex = target.ProxyIndex,
                InstanceKey = target.InstanceKey,
                SampleKind = _activeCase.Kind,
                Started = true,
                SpawnPosition = spawn,
                InitialVelocity = velocity,
                ExpectedSurfacePoint = surfacePoint,
                ExpectedNormal = expectedNormal,
                NormalAlignmentThreshold = NormalAlignmentThreshold,
                SurfacePointTolerance = SurfacePointTolerance,
                Detail = "probe spawned"
            };
            _report.Cases.Add(_activeCaseReport);
            _report.StartedCases++;
            _caseStartTime = Time.time;
            _contactObserved = false;
            _finishCaseAt = 0f;
            _activeProbeId = _nextProbeId++;

            try
            {
                _probeObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                _probeObject.name =
                    "GSP_InstanceProxyContactProbe_" + _activeCaseReport.Sequence;
                _probeObject.transform.position = spawn;
                _probeObject.transform.localScale = Vector3.one * (ProbeRadius * 2f);
                _probeObject.layer = target.Collider.gameObject.layer;

                _probeCollider = _probeObject.GetComponent<SphereCollider>();
                if (_probeCollider == null)
                    _probeCollider = _probeObject.AddComponent<SphereCollider>();
                _probeCollider.radius = 0.5f;
                _probeCollider.isTrigger = false;

                _probeBody = _probeObject.AddComponent<Rigidbody>();
                _probeBody.mass = 0.05f;
                _probeBody.useGravity = false;
                _probeBody.drag = 0f;
                _probeBody.angularDrag = 0.05f;
                _probeBody.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
                _probeBody.interpolation = RigidbodyInterpolation.Interpolate;
                _probeBody.constraints = RigidbodyConstraints.FreezeRotation;

                SpatialInstanceProxyContactTracker tracker =
                    _probeObject.AddComponent<SpatialInstanceProxyContactTracker>();
                if (tracker == null)
                    throw new InvalidOperationException("cannot-add-contact-tracker");
                tracker.Initialize(_activeProbeId);

                IgnoreAllNonTargetColliders(target.Collider);
                Physics.SyncTransforms();
                _probeBody.velocity = velocity;
                _report.TemporaryRigidbodiesCreated++;
                _report.TemporaryRigidbodiesRemaining = 1;

                _logger?.LogInfo(
                    "[GSP][INSTANCE_PROXY_CONTACT][SPAWN] " +
                    _activeCaseReport.ToLogLine());
            }
            catch (Exception e)
            {
                FinishActiveCase(
                    false,
                    false,
                    "spawn failed: " + e.GetType().Name + ":" + e.Message);
            }
        }

        private static void IgnoreAllNonTargetColliders(BoxCollider target)
        {
            if (_probeCollider == null || target == null)
                return;
            Collider[] colliders = Object.FindObjectsOfType<Collider>();
            if (colliders == null)
                return;
            for (int i = 0; i < colliders.Length; i++)
            {
                Collider other = colliders[i];
                if (other == null || other == _probeCollider || other == target)
                    continue;
                try { Physics.IgnoreCollision(_probeCollider, other, true); }
                catch { }
            }
        }

        private static void CreateImmediateFailure(string detail)
        {
            _activeCaseReport = new InstanceProxyContactCaseReport
            {
                Sequence = _report.Cases.Count + 1,
                ProxyIndex = _activeCase?.Target != null ? _activeCase.Target.ProxyIndex : -1,
                InstanceKey = _activeCase?.Target != null ? _activeCase.Target.InstanceKey : null,
                SampleKind = _activeCase?.Kind,
                Started = true
            };
            _report.Cases.Add(_activeCaseReport);
            _report.StartedCases++;
            FinishActiveCase(false, false, detail);
        }

        private static void FinishActiveCase(bool passed, bool timedOut, string detail)
        {
            if (_activeCaseReport == null)
                return;

            _activeCaseReport.Completed = true;
            _activeCaseReport.Passed = passed;
            _activeCaseReport.TimedOut = timedOut;
            _activeCaseReport.ElapsedSeconds = Math.Max(0f, Time.time - _caseStartTime);
            _activeCaseReport.Detail = detail;
            _report.CompletedCases++;
            if (_activeCaseReport.CollisionEventObserved)
                _report.EventObservedCases++;
            if (_activeCaseReport.TargetColliderMatched)
                _report.TargetMatchedCases++;
            if (_activeCaseReport.ContactPointValid)
                _report.SurfacePointValidCases++;
            if (_activeCaseReport.ContactNormalValid)
                _report.SurfaceNormalValidCases++;
            if (_activeCaseReport.SurfacePointWithinTolerance)
                _report.SurfacePointGatePassedCases++;
            if (_activeCaseReport.NormalAlignmentPassed)
                _report.NormalGatePassedCases++;

            if (passed)
                _report.PassedCases++;
            else
                _report.FailedCases++;
            if (timedOut)
                _report.TimedOutCases++;

            _logger?.LogWarning(
                "[GSP][INSTANCE_PROXY_CONTACT][CASE] " +
                _activeCaseReport.ToLogLine());

            CleanupProbe();
            _activeCase = null;
            _activeCaseReport = null;
            _activeProbeId = 0;
            _contactObserved = false;
        }

        private static void Complete()
        {
            CleanupProbe();
            _running = false;
            RestoreColliderState();

            _report.Running = false;
            _report.Completed = true;
            _report.Cancelled = false;
            _report.Success =
                _report.PlannedCases > 0 &&
                _report.CompletedCases == _report.PlannedCases &&
                _report.EventObservedCases == _report.PlannedCases &&
                _report.TargetMatchedCases == _report.PlannedCases &&
                _report.SurfacePointValidCases == _report.PlannedCases &&
                _report.SurfaceNormalValidCases == _report.PlannedCases &&
                _report.SurfacePointGatePassedCases == _report.PlannedCases &&
                _report.NormalGatePassedCases == _report.PlannedCases &&
                _report.PassedCases == _report.PlannedCases &&
                _report.FailedCases == 0 &&
                _report.TemporaryRigidbodiesRemaining == 0 &&
                _report.ProxyCollidersRestored;
            _report.ElapsedSeconds = Math.Max(0f, Time.time - _sessionStartTime);
            _report.Detail = _report.Success
                ? "all collision events, target matches and managed OBB surface gates passed; runtime state restored"
                : "one or more event/target/managed-surface gates failed; runtime state was restored";

            _logger?.LogWarning(
                "[GSP][INSTANCE_PROXY_CONTACT] final | " +
                _report.ToDetailedText());

            Targets.Clear();
            Plan.Clear();
            _activeCase = null;
            _activeCaseReport = null;
        }

        private static void RestoreColliderState()
        {
            bool restored = SpatialInstanceProxyRuntime.TrySetCollidersForContactTest(
                _collidersWereEnabled, out string restoreError);
            if (_report != null)
            {
                _report.ProxyCollidersRestored =
                    restored &&
                    SpatialInstanceProxyRuntime.CollidersEnabled ==
                    _collidersWereEnabled;
                if (!restored)
                    _report.Detail = "collider-restore-failed:" + restoreError;
            }
        }

        private static void CleanupProbe()
        {
            if (_probeObject != null)
                Object.Destroy(_probeObject);
            _probeObject = null;
            _probeBody = null;
            _probeCollider = null;
            if (_report != null)
                _report.TemporaryRigidbodiesRemaining = 0;
        }

        private static bool StrictCasePassed(
            InstanceProxyContactCaseReport item)
        {
            return
                item != null &&
                item.CollisionEventObserved &&
                item.TargetColliderMatched &&
                item.ContactPointValid &&
                item.ContactNormalValid &&
                item.SurfacePointWithinTolerance &&
                item.NormalAlignmentPassed;
        }

        private static string BuildStrictGateFailureDetail(
            InstanceProxyContactCaseReport item)
        {
            if (item == null)
                return "strict gate failed: case report missing";
            return
                "strict gate failed" +
                "; event=" + item.CollisionEventObserved +
                "; target=" + item.TargetColliderMatched +
                "; point=" + item.ContactPointValid +
                "; pointGate=" + item.SurfacePointWithinTolerance +
                "; pointError=" + item.SurfacePointError.ToString("F5") +
                "; normal=" + item.ContactNormalValid +
                "; normalGate=" + item.NormalAlignmentPassed +
                "; alignment=" + item.NormalAlignment.ToString("F5") +
                "; geometry=" + (item.GeometryDetail ?? "<none>");
        }

        /// <summary>
        /// 在纯托管代码中求“球心到定向 BoxCollider 的最近表面点与外法线”。
        ///
        /// 这里只使用 Transform / BoxCollider 的中心和尺寸，不访问 Collision.GetContact、
        /// Collision.contacts 或 ContactPoint。这样避开当前 MiSide IL2CPP interop 中
        /// ContactPoint 桥接的 MissingMethodException，同时仍能严格验证接触位置与方向。
        /// </summary>
        private static bool TryComputeManagedObbSurface(
            BoxCollider box,
            Vector3 worldPoint,
            out Vector3 surfacePointWorld,
            out Vector3 surfaceNormalWorld,
            out float centerToSurfaceDistance,
            out string detail)
        {
            surfacePointWorld = Vector3.zero;
            surfaceNormalWorld = Vector3.zero;
            centerToSurfaceDistance = 0f;
            detail = null;

            if (box == null || box.transform == null || !IsFinite(worldPoint))
            {
                detail = "invalid box/transform/worldPoint";
                return false;
            }

            Transform transform = box.transform;
            Vector3 half = box.size * 0.5f;
            if (!IsFinite(box.center) || !IsFinite(half) ||
                half.x <= 0f || half.y <= 0f || half.z <= 0f)
            {
                detail = "invalid BoxCollider center/size";
                return false;
            }

            Vector3 localPoint = transform.InverseTransformPoint(worldPoint);
            if (!IsFinite(localPoint))
            {
                detail = "InverseTransformPoint returned non-finite value";
                return false;
            }

            Vector3 relative = localPoint - box.center;
            Vector3 clampedRelative = new Vector3(
                Mathf.Clamp(relative.x, -half.x, half.x),
                Mathf.Clamp(relative.y, -half.y, half.y),
                Mathf.Clamp(relative.z, -half.z, half.z));

            bool outside =
                Mathf.Abs(relative.x) > half.x ||
                Mathf.Abs(relative.y) > half.y ||
                Mathf.Abs(relative.z) > half.z;

            Vector3 surfaceLocal;
            if (outside)
            {
                surfaceLocal = box.center + clampedRelative;
                surfacePointWorld = transform.TransformPoint(surfaceLocal);
                Vector3 delta = worldPoint - surfacePointWorld;
                centerToSurfaceDistance = delta.magnitude;
                if (!IsFinite(surfacePointWorld) ||
                    !IsFinite(delta) ||
                    centerToSurfaceDistance <= NormalEpsilon)
                {
                    detail = "outside closest point had no stable outward delta";
                    return false;
                }

                surfaceNormalWorld = delta / centerToSurfaceDistance;
                detail = "outside-point; localClamp";
            }
            else
            {
                // 点位于盒内或恰好在表面时，最近面不能靠 clamp 得到。
                // 枚举六个面并用世界距离选择最近面，兼容非均匀缩放。
                float bestDistanceSquared = float.PositiveInfinity;
                Vector3 bestSurface = Vector3.zero;
                Vector3 bestLocalNormal = Vector3.zero;

                EvaluateFaceCandidate(
                    transform, box.center, relative, half, 0, 1f,
                    worldPoint, ref bestDistanceSquared,
                    ref bestSurface, ref bestLocalNormal);
                EvaluateFaceCandidate(
                    transform, box.center, relative, half, 0, -1f,
                    worldPoint, ref bestDistanceSquared,
                    ref bestSurface, ref bestLocalNormal);
                EvaluateFaceCandidate(
                    transform, box.center, relative, half, 1, 1f,
                    worldPoint, ref bestDistanceSquared,
                    ref bestSurface, ref bestLocalNormal);
                EvaluateFaceCandidate(
                    transform, box.center, relative, half, 1, -1f,
                    worldPoint, ref bestDistanceSquared,
                    ref bestSurface, ref bestLocalNormal);
                EvaluateFaceCandidate(
                    transform, box.center, relative, half, 2, 1f,
                    worldPoint, ref bestDistanceSquared,
                    ref bestSurface, ref bestLocalNormal);
                EvaluateFaceCandidate(
                    transform, box.center, relative, half, 2, -1f,
                    worldPoint, ref bestDistanceSquared,
                    ref bestSurface, ref bestLocalNormal);

                if (float.IsInfinity(bestDistanceSquared) ||
                    !IsFinite(bestSurface) ||
                    !IsFinite(bestLocalNormal))
                {
                    detail = "inside nearest-face enumeration failed";
                    return false;
                }

                surfacePointWorld = bestSurface;
                surfaceNormalWorld =
                    TransformLocalNormal(transform, bestLocalNormal);
                centerToSurfaceDistance =
                    Mathf.Sqrt(Mathf.Max(0f, bestDistanceSquared));
                detail = "inside-or-on-surface; nearest-world-face";
            }

            if (!IsFinite(surfacePointWorld) ||
                !IsFinite(surfaceNormalWorld) ||
                surfaceNormalWorld.sqrMagnitude <=
                NormalEpsilon * NormalEpsilon)
            {
                detail = "managed OBB result was non-finite or zero-normal";
                return false;
            }

            surfaceNormalWorld.Normalize();
            return true;
        }

        private static void EvaluateFaceCandidate(
            Transform transform,
            Vector3 boxCenter,
            Vector3 relative,
            Vector3 half,
            int axis,
            float sign,
            Vector3 worldPoint,
            ref float bestDistanceSquared,
            ref Vector3 bestSurfaceWorld,
            ref Vector3 bestLocalNormal)
        {
            Vector3 candidateRelative = relative;
            Vector3 localNormal = Vector3.zero;
            if (axis == 0)
            {
                candidateRelative.x = sign * half.x;
                localNormal.x = sign;
            }
            else if (axis == 1)
            {
                candidateRelative.y = sign * half.y;
                localNormal.y = sign;
            }
            else
            {
                candidateRelative.z = sign * half.z;
                localNormal.z = sign;
            }

            Vector3 candidateWorld =
                transform.TransformPoint(boxCenter + candidateRelative);
            if (!IsFinite(candidateWorld))
                return;

            float distanceSquared =
                (worldPoint - candidateWorld).sqrMagnitude;
            if (distanceSquared < bestDistanceSquared)
            {
                bestDistanceSquared = distanceSquared;
                bestSurfaceWorld = candidateWorld;
                bestLocalNormal = localNormal;
            }
        }

        private static Vector3 TransformLocalNormal(
            Transform transform,
            Vector3 localNormal)
        {
            if (transform == null || localNormal.sqrMagnitude <= 0f)
                return Vector3.zero;

            // 用两个世界点之差而不是 TransformDirection，保留负缩放时的朝向。
            Vector3 origin = transform.TransformPoint(Vector3.zero);
            Vector3 outward =
                transform.TransformPoint(localNormal.normalized);
            Vector3 world = outward - origin;
            return world.sqrMagnitude > NormalEpsilon * NormalEpsilon
                ? world.normalized
                : Vector3.zero;
        }

        private static bool IsFinite(Vector3 value)
        {
            return
                !float.IsNaN(value.x) && !float.IsInfinity(value.x) &&
                !float.IsNaN(value.y) && !float.IsInfinity(value.y) &&
                !float.IsNaN(value.z) && !float.IsInfinity(value.z);
        }
    }
}
