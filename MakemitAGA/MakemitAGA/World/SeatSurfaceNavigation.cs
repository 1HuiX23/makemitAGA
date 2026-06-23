/*
 * =================================================================================================
 * SeatSurfaceNavigation.cs
 * =================================================================================================
 *
 * 作用：处理座点周围的 NavMesh/物理可达点，并保留旧测试桥接所需的兼容代码。

 * 主要逻辑：
 *   - 坐姿方向、床边/家具边缘外侧方向；
 *   - 最近可达 NavMesh 点和物理地面点；
 *   - 临时 goto 目标及旧版 Mita_sit 反射桥接；
 *   - 与正式 SeatPose 执行路径共存的历史兼容入口。
 *
 * 维护约束：
 *   - 本文件是 SeatSurfaceAnalysisRuntime 的 partial 实现；
 *   - 共享状态、常量和嵌套数据结构统一定义在 SeatSurfaceAnalysisMesh.cs；
 *   - 不要在多个 partial 文件中重复声明同名字段或方法；
 *   - 拆分只改变源码组织，不改变运行流程、Collider、VLM 或调试行为。
 * =================================================================================================
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using BepInEx;
using BepInEx.Logging;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using MakemitAGA.Mita_self.Mita_tools;
using HarmonyLib;
using UnityEngine;
using UnityEngine.AI;
using Object = UnityEngine.Object;

namespace MakemitAGA.World
{
    internal static partial class SeatSurfaceAnalysisRuntime
    {
        private static void AddTriangleFacingUp(List<int> triangles, List<Vector3> vertices, int ia, int ib, int ic)
        {
            Vector3 a = vertices[ia], b = vertices[ib], c = vertices[ic];
            Vector3 n = Vector3.Cross(b - a, c - a);
            if (n.y < 0f) { triangles.Add(ia); triangles.Add(ic); triangles.Add(ib); }
            else { triangles.Add(ia); triangles.Add(ib); triangles.Add(ic); }
        }

        private static GameObject FindHardcodedBedObject()
        {
            GameObject[] all = null;
            try { all = Object.FindObjectsOfType<GameObject>(); } catch { }
            if (all == null) return null;

            GameObject best = null;
            int bestScore = int.MinValue;
            for (int i = 0; i < all.Length; i++)
            {
                GameObject go = all[i];
                if (go == null) continue;
                string n = go.name ?? string.Empty;
                int score = int.MinValue;
                if (n.Equals(HardcodedBedName, StringComparison.OrdinalIgnoreCase)) score = 1000;
                else if (n.IndexOf(HardcodedBedName, StringComparison.OrdinalIgnoreCase) >= 0 && n.IndexOf("Bedroom", StringComparison.OrdinalIgnoreCase) < 0) score = 200;
                else continue;

                string path = GetTransformPath(go.transform);
                if (path.IndexOf("Bedroom", StringComparison.OrdinalIgnoreCase) >= 0) score += 100;
                if (go.activeInHierarchy) score += 10;
                Renderer r = null;
                try { r = go.GetComponentInChildren<Renderer>(true); } catch { }
                if (r != null) score += 20;

                if (score > bestScore) { bestScore = score; best = go; }
            }

            if (best != null) LogWarning("FindHardcodedBedObject: selected=" + best.name + ", path=" + GetTransformPath(best.transform));
            return best;
        }

        private static string GetTransformPath(Transform t)
        {
            if (t == null) return "<null>";
            List<string> parts = new List<string>();
            while (t != null) { parts.Add(t.name); t = t.parent; }
            parts.Reverse();
            return string.Join("/", parts.ToArray());
        }

        private static bool TryRaycastSpecificTarget(Ray ray, GameObject targetRoot, out RaycastHit bestHit)
        {
            bestHit = new RaycastHit();
            if (targetRoot == null) return false;
            RaycastHit[] hits = null;
            try { hits = Physics.RaycastAll(ray, RayDistance, -1, QueryTriggerInteraction.Ignore); } catch { }
            if (hits == null || hits.Length == 0) return false;

            bool found = false;
            float bestDistance = float.MaxValue;
            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit h = hits[i];
                if (h.collider == null || h.distance < 0.03f || IsOwnVisual(h.collider.gameObject)) continue;
                if (!IsSameOrChildOf(h.collider.transform, targetRoot.transform)) continue;
                if (h.distance < bestDistance) { bestDistance = h.distance; bestHit = h; found = true; }
            }
            return found;
        }

        private static bool IsSameOrChildOf(Transform t, Transform root)
        {
            while (t != null)
            {
                if (t == root) return true;
                t = t.parent;
            }
            return false;
        }

        private static Vector3 FlattenXZ(Vector3 v)
        {
            v.y = 0f;
            return v;
        }

        private static Vector3 EstimateOutwardForwardFromBounds(Vector3 surfacePoint, Bounds bounds)
        {
            // 没有 NavMesh/floor point 时，根据 surfacePoint 到 bounds 四条边的距离，
            // 选择离它最近的一条边作为“外侧方向”。
            float dxMin = Mathf.Abs(surfacePoint.x - bounds.min.x);
            float dxMax = Mathf.Abs(bounds.max.x - surfacePoint.x);
            float dzMin = Mathf.Abs(surfacePoint.z - bounds.min.z);
            float dzMax = Mathf.Abs(bounds.max.z - surfacePoint.z);

            float best = dxMin;
            Vector3 dir = Vector3.left;

            if (dxMax < best) { best = dxMax; dir = Vector3.right; }
            if (dzMin < best) { best = dzMin; dir = Vector3.back; }
            if (dzMax < best) { best = dzMax; dir = Vector3.forward; }

            return dir;
        }

        private static bool TryFindNearestReachableNavPointNearSurface(Vector3 surfacePoint, Bounds targetBounds, out Vector3 navPoint)
        {
            navPoint = surfacePoint;

            // v0.9.1：
            //   第一优先级仍然是 NavMesh，因为 MitaPerson.AiWalkToTarget 最终也更适合走 NavMesh。
            //   但 v0.8.0 的日志显示 navPoint=<none>，说明仅靠 SamplePosition/CalculatePath 太脆。
            //   所以这里增加 Physics.Raycast 地板回退：
            //
            //     1. 先在目标床面点周围做环形 NavMesh 采样。
            //     2. 如果失败，再从床边周围向下打射线找 Floor/Carpet/Rug 等真实地面。
            //     3. 找到地面后，优先把它吸附回 NavMesh；如果吸附失败，也会创建隐藏 cube。
            //
            //   这样独立测试项目至少能稳定生成 DepthGotoPoint_Bed；
            //   MakemitAGA 存在时再尝试让 Mita 走过去并 Sit。

            if (TryFindNearestReachableNavMeshPointNearSurface(surfacePoint, targetBounds, out navPoint))
                return true;

            LogWarning("NavMesh point search failed. Trying Physics floor fallback near selected bed surface.");

            Vector3 floorPoint;
            if (!TryFindNearestFloorPointByPhysics(surfacePoint, targetBounds, out floorPoint))
            {
                LogWarning("Physics floor fallback also failed near " + Vec(surfacePoint));
                return false;
            }

            // 如果 Physics 找到地板，再尽量吸附到附近 NavMesh。
            NavMeshHit navHit;
            if (NavMesh.SamplePosition(floorPoint, out navHit, 1.25f, -1))
            {
                navPoint = navHit.position;
                LogWarning("Physics floor fallback snapped to NavMesh. floor=" + Vec(floorPoint) +
                           ", navPoint=" + Vec(navPoint) +
                           ", xzDistance=" + XZDistance(surfacePoint, navPoint).ToString("F2"));
                return true;
            }

            // 没有 NavMesh 也返回 floorPoint。这样至少可以生成 DepthGotoPoint_Bed 供 UnityExplorer 检查。
            navPoint = floorPoint;
            LogWarning("Using raw Physics floor point without NavMesh snap. floor=" + Vec(navPoint) +
                       ", xzDistance=" + XZDistance(surfacePoint, navPoint).ToString("F2"));
            return true;
        }

        private static bool TryFindNearestReachableNavMeshPointNearSurface(Vector3 surfacePoint, Bounds targetBounds, out Vector3 navPoint)
        {
            navPoint = surfacePoint;

            MitaPerson mita = null;
            try { mita = Object.FindObjectOfType<MitaPerson>(); } catch { }

            Vector3 start = mita != null ? mita.transform.position : surfacePoint;

            // v0.9.1：增加半径上限。床中心到床边可能接近 1m；再考虑床旁地毯/障碍，需要更大搜索圈。
            float[] radii = new float[]
            {
                0.35f, 0.50f, 0.70f, 0.90f, 1.10f, 1.35f, 1.60f, 1.90f,
                2.25f, 2.70f, 3.15f, 3.60f
            };

            float bestScore = float.MaxValue;
            Vector3 best = Vector3.zero;
            bool found = false;
            NavMeshPath path = new NavMeshPath();

            for (int r = 0; r < radii.Length; r++)
            {
                float radius = radii[r];
                int steps = Mathf.Max(20, Mathf.RoundToInt(radius * 28f));

                for (int i = 0; i < steps; i++)
                {
                    float angle = Mathf.PI * 2f * (i / (float)steps);
                    Vector3 sample = surfacePoint + new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * radius;

                    NavMeshHit navHit;
                    if (!NavMesh.SamplePosition(sample, out navHit, 0.85f, -1))
                        continue;

                    // 避免选到床体内部或床底。
                    if (IsXZInsideBounds(navHit.position, targetBounds, 0.05f))
                        continue;

                    float pathLen = 0f;
                    if (mita != null)
                    {
                        if (!NavMesh.CalculatePath(start, navHit.position, -1, path) ||
                            path.status != NavMeshPathStatus.PathComplete)
                            continue;

                        pathLen = EstimatePathLength(path);
                    }

                    float surfaceDist = XZDistance(surfacePoint, navHit.position);
                    float score = surfaceDist + pathLen * 0.08f;

                    if (score < bestScore)
                    {
                        bestScore = score;
                        best = navHit.position;
                        found = true;
                    }
                }

                if (found)
                    break;
            }

            if (!found)
                return false;

            navPoint = best;
            LogWarning("Nearest reachable NavMesh point found. surface=" + Vec(surfacePoint) +
                       ", navPoint=" + Vec(navPoint) +
                       ", xzDistance=" + XZDistance(surfacePoint, navPoint).ToString("F2"));
            return true;
        }

        private static bool TryFindNearestFloorPointByPhysics(Vector3 surfacePoint, Bounds targetBounds, out Vector3 floorPoint)
        {
            floorPoint = surfacePoint;

            float[] radii = new float[]
            {
                0.45f, 0.65f, 0.85f, 1.05f, 1.25f, 1.50f, 1.80f, 2.15f, 2.55f, 3.00f
            };

            float rayStartY = Mathf.Max(targetBounds.max.y + 1.35f, surfacePoint.y + 1.35f);
            float rayLength = 5.0f;

            float bestScore = float.MaxValue;
            Vector3 best = Vector3.zero;
            bool found = false;

            for (int r = 0; r < radii.Length; r++)
            {
                float radius = radii[r];
                int steps = Mathf.Max(24, Mathf.RoundToInt(radius * 32f));

                for (int i = 0; i < steps; i++)
                {
                    float angle = Mathf.PI * 2f * (i / (float)steps);
                    Vector3 offset = new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)) * radius;
                    Vector3 sampleXZ = surfacePoint + offset;

                    // 避免在床的 XZ 范围内向下找点，这通常会打到床、床底或床下不可走区域。
                    if (IsXZInsideBounds(sampleXZ, targetBounds, 0.08f))
                        continue;

                    Vector3 origin = new Vector3(sampleXZ.x, rayStartY, sampleXZ.z);

                    RaycastHit[] hits = null;
                    try { hits = Physics.RaycastAll(origin, Vector3.down, rayLength, -1, QueryTriggerInteraction.Ignore); }
                    catch { hits = null; }

                    if (hits == null || hits.Length == 0)
                        continue;

                    SortRaycastHitsByDistance(hits);

                    for (int h = 0; h < hits.Length; h++)
                    {
                        RaycastHit hit = hits[h];

                        if (hit.collider == null)
                            continue;

                        GameObject hitGo = hit.collider.gameObject;
                        if (IsOwnVisual(hitGo))
                            continue;

                        // 不能把床、床垫、枕头当作地面点。
                        if (IsXZInsideBounds(hit.point, targetBounds, 0.05f))
                            continue;

                        // 地面点应该低于床面。否则容易把桌面、凳面、柜子顶当作可达地面。
                        if (hit.point.y > surfacePoint.y - 0.12f)
                            continue;

                        string n = hitGo != null ? (hitGo.name ?? string.Empty) : string.Empty;
                        string path = hitGo != null ? GetTransformPath(hitGo.transform) : string.Empty;

                        // 命名命中地板/地毯时加分，但不强制要求，避免游戏对象命名不稳定。
                        float nameBonus = 0f;
                        string lower = (n + "/" + path).ToLowerInvariant();
                        if (lower.Contains("floor")) nameBonus -= 0.25f;
                        if (lower.Contains("carpet")) nameBonus -= 0.18f;
                        if (lower.Contains("rug")) nameBonus -= 0.18f;

                        float surfaceDist = XZDistance(surfacePoint, hit.point);
                        float yPenalty = Mathf.Abs(hit.point.y - targetBounds.min.y) * 0.05f;
                        float score = surfaceDist + yPenalty + nameBonus;

                        if (score < bestScore)
                        {
                            bestScore = score;
                            best = hit.point;
                            found = true;
                        }

                        break;
                    }
                }

                if (found)
                    break;
            }

            if (!found)
                return false;

            floorPoint = best;
            LogWarning("Nearest Physics floor point found. surface=" + Vec(surfacePoint) +
                       ", floor=" + Vec(floorPoint) +
                       ", xzDistance=" + XZDistance(surfacePoint, floorPoint).ToString("F2"));
            return true;
        }

        private static void SortRaycastHitsByDistance(RaycastHit[] hits)
        {
            if (hits == null) return;

            Array.Sort(hits, delegate (RaycastHit a, RaycastHit b)
            {
                return a.distance.CompareTo(b.distance);
            });
        }

        private static bool IsXZInsideBounds(Vector3 p, Bounds b, float margin)
        {
            return p.x >= b.min.x - margin && p.x <= b.max.x + margin && p.z >= b.min.z - margin && p.z <= b.max.z + margin;
        }

        private static float EstimatePathLength(NavMeshPath path)
        {
            if (path == null || path.corners == null || path.corners.Length < 2) return 0f;
            float sum = 0f;
            for (int i = 1; i < path.corners.Length; i++) sum += Vector3.Distance(path.corners[i - 1], path.corners[i]);
            return sum;
        }

        private static float XZDistance(Vector3 a, Vector3 b)
        {
            float dx = a.x - b.x;
            float dz = a.z - b.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }

        private static GameObject CreateHiddenGotoPoint(Transform parent, Vector3 navPoint)
        {
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cube);
            go.name = BedGotoPointName;
            go.transform.position = navPoint + Vector3.up * (BedGotoCubeSize * 0.5f);
            go.transform.rotation = Quaternion.identity;
            go.transform.localScale = Vector3.one * BedGotoCubeSize;
            if (parent != null) go.transform.SetParent(parent, true);

            Renderer r = null;
            try { r = go.GetComponent<Renderer>(); } catch { }
            if (r != null) r.enabled = false;
            Collider c = null;
            try { c = go.GetComponent<Collider>(); } catch { }
            if (c != null) c.isTrigger = true;
            LogWarning("Hidden goto cube created: " + go.name + " at " + Vec(go.transform.position));
            return go;
        }

        private static void StartMitaWalkToGotoPointThenSit(Transform gotoTransform, string seatProxyName)
        {
            if (gotoTransform == null) { TryCallMitaSitByReflection(seatProxyName); return; }
            MitaPerson mita = null;
            try { mita = Object.FindObjectOfType<MitaPerson>(); } catch { }
            if (mita == null) { TryCallMitaSitByReflection(seatProxyName); return; }

            try { mita.MagnetOff(); } catch { }
            SetDialoguePatchesBool("IsAIControllingMovement", true);
            SetDialoguePatchesBool("IsInternalCall", true);
            try { mita.AiWalkToTarget(gotoTransform); }
            catch (Exception e)
            {
                LogWarning("AiWalkToTarget failed: " + e.GetType().Name + " " + e.Message);
                SetDialoguePatchesBool("IsInternalCall", false);
                TryCallMitaSitByReflection(seatProxyName);
                return;
            }
            SetDialoguePatchesBool("IsInternalCall", false);

            _pendingBedTopSit = true;
            _pendingBedTopSitMita = mita;
            _pendingBedTopSitGoto = gotoTransform;
            _pendingBedTopSitSeatProxyName = seatProxyName;
            _pendingBedTopSitDeadline = Time.time + BedGotoTimeout;
            _pendingBedTopSitNextRepathTime = Time.time + BedGotoRepathInterval;
            TryConsolePrint("米塔正在走向床边最近可达点，随后尝试坐下。");
        }

        private static void TickPendingBedSit()
        {
            if (!_pendingBedTopSit)
                return;

            MitaPerson mita = _pendingBedTopSitMita;
            Transform gotoT = _pendingBedTopSitGoto;

            if (mita == null || gotoT == null)
            {
                FinishPendingBedSit(true, "pending target lost; fallback sit");
                return;
            }

            Vector3 mitaPos = mita.transform.position;
            Vector3 targetPos = gotoT.position;

            float dist = XZDistance(mitaPos, targetPos);

            NavMeshAgent agent = null;
            try { agent = mita.GetComponent<NavMeshAgent>(); } catch { agent = null; }

            float remaining = 9999f;
            bool agentArrived = false;

            if (agent != null && agent.enabled)
            {
                try
                {
                    remaining = agent.remainingDistance;

                    // remainingDistance 在刚开始计算 path 时可能是 Infinity。
                    // 只有 path 不在 pending，且 remainingDistance 有意义时才参与判断。
                    if (!agent.pathPending &&
                        !float.IsInfinity(remaining) &&
                        !float.IsNaN(remaining) &&
                        remaining <= 0.55f)
                    {
                        agentArrived = true;
                    }
                }
                catch { }
            }

            bool arrived = dist <= BedGotoArriveDistance || agentArrived;
            bool timeout = Time.time >= _pendingBedTopSitDeadline;

            // 限频日志。v0.8.1 只能看到最后 timeout，无法判断为什么没有 arrived。
            if (Time.time >= _pendingBedTopSitLastLogTime + 1.0f)
            {
                _pendingBedTopSitLastLogTime = Time.time;
                LogWarning("Pending BedTopSit progress. dist=" + dist.ToString("F2") +
                           ", remaining=" + (remaining >= 9998f ? "<none>" : remaining.ToString("F2")) +
                           ", arrived=" + arrived +
                           ", timeout=" + timeout);
            }

            if (arrived)
            {
                FinishPendingBedSit(true, "arrived");
                return;
            }

            if (timeout)
            {
                // v0.9.1：
                //   v0.8.1 的日志显示米塔肉眼已经走到附近，但由于 transform root / NavMeshAgent remainingDistance
                //   没有满足严格阈值，最终 reason=timeout, shouldSit=False。
                //   这里 timeout 时也进入 Mita_sit，由 Mita_sit 自己再做 WalkToDockPoint / ManualApproach 安全校验。
                FinishPendingBedSit(true, "timeout; force bridge sit");
                return;
            }

            // v0.9.1：
            //   不再每 0.85 秒重复 AiWalkToTarget。
            //   重复调用会让游戏原生动作/动画队列反复被重启，表现为“吃饼干抬手动作来回抽搐”。
            //   路径交给第一次 AiWalkToTarget 和 NavMeshAgent 自己处理。
        }

        private static void FinishPendingBedSit(bool shouldSit, string reason)
        {
            string seatName = _pendingBedTopSitSeatProxyName;
            MitaPerson mita = _pendingBedTopSitMita;
            _pendingBedTopSit = false;
            _pendingBedTopSitMita = null;
            _pendingBedTopSitGoto = null;
            _pendingBedTopSitSeatProxyName = null;
            _pendingBridgeSit = false;
            _pendingBridgeTargetName = null;
            SetDialoguePatchesBool("IsAIControllingMovement", false);
            SetDialoguePatchesBool("IsInternalCall", false);
            if (mita != null)
            {
                TryCallMitaSharpStop(mita);
                try { mita.MagnetOff(); } catch { }

                NavMeshAgent agent = null;
                try { agent = mita.GetComponent<NavMeshAgent>(); } catch { agent = null; }
                if (agent != null && agent.enabled)
                {
                    try
                    {
                        agent.velocity = Vector3.zero;
                        agent.isStopped = true;
                        agent.ResetPath();
                    }
                    catch { }
                }
            }
            LogWarning("Pending BedTopSit finished. reason=" + reason + ", shouldSit=" + shouldSit);
            if (shouldSit && !string.IsNullOrEmpty(seatName)) TryCallMitaSitByReflection(seatName);
        }

        private static void TryCallMitaSharpStop(MitaPerson mita)
        {
            if (mita == null) return;
            try
            {
                MethodInfo m = AccessTools.Method(mita.GetType(), "AiShraplyStop");
                if (m != null) m.Invoke(mita, null);
            }
            catch { }
        }

        private static void ScheduleBridgeSit(string targetName)
        {
            try { Physics.SyncTransforms(); } catch { }

            _pendingBridgeSit = true;
            _pendingBridgeTargetName = targetName;
            _pendingBridgeFrame = Time.frameCount + 2;

            LogWarning("Bridge sit scheduled. target=" + targetName + ", frame=" + _pendingBridgeFrame);
        }

        private static void TickPendingBridgeSit()
        {
            if (!_pendingBridgeSit) return;
            if (Time.frameCount < _pendingBridgeFrame) return;

            string target = _pendingBridgeTargetName;

            _pendingBridgeSit = false;
            _pendingBridgeTargetName = null;

            try { Physics.SyncTransforms(); } catch { }

            LogWarning("Bridge sit executing after delay. target=" + target);
            TryCallMitaSitByReflection(target);
        }

        private static void TryCallMitaSitByReflection(string targetName)
        {
            try
            {
                // v0.9.1：
                // 不再使用 Harmony AccessTools.TypeByName。
                // 旧版本 TypeByName 会遍历许多 Unity/IL2CPP 程序集，导致大量 ReflectionTypeLoadException 警告。
                // 这里只做精确 Assembly.GetType；必要时只枚举 MakemitAGA/Mita 命名的插件程序集。
                Type t = FindTypeByNames(
                    "MakemitAGA.Mita_self.Mita_tools.Mita_sit",
                    "MakemitAGA.Mita_self.Mita_sit",
                    "MakemitAGA.Mita_sit",
                    "Mita_sit");

                if (t == null)
                {
                    if (!_mitaSitMissingWarned)
                    {
                        _mitaSitMissingWarned = true;
                        LogWarning("Mita_sit is not available yet. Standalone bridge created proxies only. target=" + targetName);
                        TryConsolePrint("<color=yellow>Mita_sit 当前不可用：已生成扫描代理和 GotoPoint，但不会执行坐下。</color>");
                    }
                    return;
                }

                MethodInfo m = t.GetMethod(
                    "Sit",
                    BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static,
                    null,
                    new Type[] { typeof(string) },
                    null);

                if (m == null)
                {
                    LogWarning("Found Mita_sit type but cannot find static Sit(string). type=" + t.FullName);
                    return;
                }

                LogWarning("Calling Mita_sit.Sit(\"" + targetName + "\") via bridge. type=" + t.FullName + ", assembly=" + t.Assembly.GetName().Name);
                TryConsolePrint("桥接调用 Mita_sit.Sit(" + targetName + ")");

                m.Invoke(null, new object[] { targetName });
            }
            catch (Exception e)
            {
                LogError("TryCallMitaSitByReflection exception: " + e);
            }
        }

        private static void SetDialoguePatchesBool(string fieldName, bool value)
        {
            try
            {
                Type t = FindTypeByNames(
                    "MakemitAGA.Mita_self.DialoguePatches",
                    "MakemitAGA.DialoguePatches",
                    "DialoguePatches");

                if (t == null) return;

                FieldInfo f = t.GetField(fieldName, BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static);
                if (f == null) return;

                f.SetValue(null, value);
            }
            catch { }
        }

        private static Type FindTypeByNames(params string[] names)
        {
            if (names == null || names.Length == 0)
                return null;

            Assembly[] assemblies = null;
            try { assemblies = AppDomain.CurrentDomain.GetAssemblies(); }
            catch { assemblies = null; }

            if (assemblies == null)
                return null;

            // 第一轮：精确 GetType，不枚举程序集类型，通常不会触发 ReflectionTypeLoadException。
            for (int a = 0; a < assemblies.Length; a++)
            {
                Assembly asm = assemblies[a];
                if (asm == null) continue;

                for (int i = 0; i < names.Length; i++)
                {
                    try
                    {
                        Type t = asm.GetType(names[i], false, false);
                        if (t != null)
                            return t;
                    }
                    catch { }
                }
            }

            // 第二轮：只枚举可能是用户插件的程序集，避免扫描 Assembly-CSharp / UnityEngine.* 刷屏。
            for (int a = 0; a < assemblies.Length; a++)
            {
                Assembly asm = assemblies[a];
                if (asm == null) continue;

                string asmName = string.Empty;
                try { asmName = asm.GetName().Name ?? string.Empty; } catch { }

                if (!LooksLikeUserMitaAssembly(asmName))
                    continue;

                Type[] types = GetTypesSafe(asm);
                if (types == null) continue;

                for (int tIndex = 0; tIndex < types.Length; tIndex++)
                {
                    Type t = types[tIndex];
                    if (t == null) continue;

                    for (int i = 0; i < names.Length; i++)
                    {
                        string wanted = names[i];

                        if (string.Equals(t.FullName, wanted, StringComparison.Ordinal) ||
                            string.Equals(t.Name, wanted, StringComparison.Ordinal))
                        {
                            return t;
                        }
                    }
                }
            }

            return null;
        }

        private static bool LooksLikeUserMitaAssembly(string asmName)
        {
            if (string.IsNullOrEmpty(asmName))
                return false;

            return asmName.IndexOf("Makemit", StringComparison.OrdinalIgnoreCase) >= 0 ||
                   asmName.IndexOf("Mita", StringComparison.OrdinalIgnoreCase) >= 0 ||
                   asmName.IndexOf("AGA", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private static Type[] GetTypesSafe(Assembly asm)
        {
            if (asm == null) return null;

            try
            {
                return asm.GetTypes();
            }
            catch (ReflectionTypeLoadException e)
            {
                return e.Types;
            }
            catch
            {
                return null;
            }
        }

    }
}
