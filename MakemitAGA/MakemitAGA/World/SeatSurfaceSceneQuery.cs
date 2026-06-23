/*
 * =================================================================================================
 * SeatSurfaceSceneQuery.cs
 * =================================================================================================
 *
 * 作用：统一处理目标 Bounds、Renderer/Collider 合并、场景射线和临时扫描 Layer。

 * 主要逻辑：
 *   - Renderer Bounds 为 X/Z 主依据，过滤后的 Collider Bounds 补充 Y；
 *   - 家具/房间根节点判断与扫描目标猜测；
 *   - 目标 Layer 临时替换和恢复；
 *   - 扫描体积框、射线与基础场景查询工具。
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
        private static bool TryGetTargetScanBounds(
            GameObject root,
            out Bounds scanBounds,
            out Bounds rendererBounds,
            out Bounds colliderBounds,
            out bool hasRendererBounds,
            out bool hasColliderBounds,
            out bool colliderExpandedVerticalRange)
        {
            scanBounds = new Bounds();
            rendererBounds = new Bounds();
            colliderBounds = new Bounds();

            hasRendererBounds =
                TryGetRendererBounds(
                    root,
                    out rendererBounds);

            hasColliderBounds =
                TryGetUsableColliderBounds(
                    root,
                    hasRendererBounds,
                    rendererBounds,
                    out colliderBounds);

            colliderExpandedVerticalRange = false;

            if (!hasRendererBounds &&
                !hasColliderBounds)
            {
                return false;
            }

            if (hasRendererBounds)
            {
                // Visual Renderer bounds remain authoritative in X/Z.
                scanBounds = rendererBounds;

                if (hasColliderBounds)
                {
                    Vector3 min =
                        scanBounds.min;

                    Vector3 max =
                        scanBounds.max;

                    float oldMinY = min.y;
                    float oldMaxY = max.y;

                    min.y = Mathf.Min(
                        min.y,
                        colliderBounds.min.y);

                    max.y = Mathf.Max(
                        max.y,
                        colliderBounds.max.y);

                    scanBounds.SetMinMax(
                        min,
                        max);

                    colliderExpandedVerticalRange =
                        min.y < oldMinY - 0.005f ||
                        max.y > oldMaxY + 0.005f;
                }

                return true;
            }

            // Renderer-less target fallback.
            scanBounds = colliderBounds;
            colliderExpandedVerticalRange = true;
            return true;
        }

        private static bool TryGetUsableColliderBounds(
            GameObject root,
            bool hasRendererBounds,
            Bounds rendererBounds,
            out Bounds bounds)
        {
            bounds = new Bounds();

            if (root == null)
                return false;

            Collider[] colliders = null;

            try
            {
                colliders =
                    root.GetComponentsInChildren<Collider>(
                        true);
            }
            catch { }

            if (colliders == null ||
                colliders.Length == 0)
            {
                return false;
            }

            bool has = false;

            Bounds horizontalGate =
                rendererBounds;

            if (hasRendererBounds)
            {
                horizontalGate.Expand(
                    new Vector3(
                        0.60f,
                        10f,
                        0.60f));
            }

            for (int i = 0;
                 i < colliders.Length;
                 i++)
            {
                Collider collider =
                    colliders[i];

                if (collider == null ||
                    !collider.enabled ||
                    collider.isTrigger ||
                    collider.gameObject == null ||
                    !collider.gameObject.activeInHierarchy ||
                    IsOwnVisual(collider.gameObject))
                {
                    continue;
                }

                Bounds candidate;

                try
                {
                    candidate = collider.bounds;
                }
                catch
                {
                    continue;
                }

                if (candidate.size.x < 0.002f ||
                    candidate.size.y < 0.002f ||
                    candidate.size.z < 0.002f)
                {
                    continue;
                }

                if (hasRendererBounds)
                {
                    bool horizontalOverlap =
                        candidate.max.x >=
                            horizontalGate.min.x &&
                        candidate.min.x <=
                            horizontalGate.max.x &&
                        candidate.max.z >=
                            horizontalGate.min.z &&
                        candidate.min.z <=
                            horizontalGate.max.z;

                    if (!horizontalOverlap)
                        continue;

                    float maxColliderX =
                        Mathf.Max(
                            rendererBounds.size.x *
                            2.25f,
                            rendererBounds.size.x +
                            0.80f);

                    float maxColliderZ =
                        Mathf.Max(
                            rendererBounds.size.z *
                            2.25f,
                            rendererBounds.size.z +
                            0.80f);

                    float maxColliderY =
                        Mathf.Max(
                            rendererBounds.size.y *
                            3.00f,
                            rendererBounds.size.y +
                            1.50f);

                    if (candidate.size.x >
                            maxColliderX ||
                        candidate.size.z >
                            maxColliderZ ||
                        candidate.size.y >
                            maxColliderY)
                    {
                        continue;
                    }
                }

                if (!has)
                {
                    bounds = candidate;
                    has = true;
                }
                else
                {
                    bounds.Encapsulate(
                        candidate);
                }
            }

            return has;
        }

        private static bool TryGetRendererBounds(GameObject root, out Bounds bounds)
        {
            bounds = new Bounds();
            if (root == null) return false;
            Renderer[] renderers = null;
            try { renderers = root.GetComponentsInChildren<Renderer>(true); } catch { }
            if (renderers == null || renderers.Length == 0) return false;
            return TryCalculateRendererBounds(renderers, out bounds);
        }

        private static int PickScanLayer()
        {
            int[] candidates = { 30, 29, 28, 27, 26, 25 };
            GameObject[] all = null;
            try { all = Object.FindObjectsOfType<GameObject>(); } catch { }
            if (all == null || all.Length == 0) return DefaultScanLayer;

            int bestLayer = DefaultScanLayer;
            int bestCount = int.MaxValue;
            for (int i = 0; i < candidates.Length; i++)
            {
                int layer = candidates[i];
                int count = 0;
                for (int j = 0; j < all.Length; j++)
                {
                    GameObject go = all[j];
                    if (go == null || go.layer != layer || IsOwnVisual(go)) continue;
                    Renderer r = null;
                    try { r = go.GetComponent<Renderer>(); } catch { }
                    if (r != null && r.enabled && go.activeInHierarchy) count++;
                }
                if (count < bestCount) { bestCount = count; bestLayer = layer; if (count == 0) break; }
            }
            return bestLayer;
        }

        private static GameObject GuessScanRoot(GameObject hitObject, Vector3 hitPoint)
        {
            if (hitObject == null) return null;
            Transform cursor = hitObject.transform;
            GameObject best = hitObject;
            float bestScore = float.MinValue;

            for (int depth = 0; depth <= 6 && cursor != null; depth++, cursor = cursor.parent)
            {
                string nodeName = cursor.name ?? string.Empty;
                if (IsSceneOrRoomRootName(nodeName)) break;
                Renderer[] renderers = null;
                try { renderers = cursor.GetComponentsInChildren<Renderer>(true); } catch { }
                if (renderers == null || renderers.Length == 0) continue;
                Bounds b;
                if (!TryCalculateRendererBounds(renderers, out b)) continue;
                float size = b.size.magnitude;
                float volume = b.size.x * b.size.y * b.size.z;
                if (size > 6.0f || volume > 18.0f) continue;

                float score = 0f;
                if (b.Contains(hitPoint)) score += 120f;
                score -= b.SqrDistance(hitPoint) * 30f;
                score -= Mathf.Max(0f, size - 2.5f) * 8f;
                score -= Mathf.Max(0f, volume - 4.0f) * 2f;
                if (IsBedLikeName(nodeName)) score += 55f;
                if (IsSeatLikeName(nodeName)) score += 35f;
                if (IsTableLikeName(nodeName)) score += 25f;
                score -= depth * 2f;
                if (cursor.gameObject == hitObject && IsLikelyFurnitureName(nodeName)) score += 40f;
                if (score > bestScore) { bestScore = score; best = cursor.gameObject; }
            }

            LogInfo("GuessScanRoot: hitObject=" + hitObject.name + ", selected=" + best.name + ", score=" + bestScore.ToString("F2"));
            return best;
        }

        private static bool IsSceneOrRoomRootName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            return n == "bedroom" || n.Contains("room") || n.Contains("scene") || n.Contains("level") || n.Contains("location") || n.Contains("environment") || n.Contains("interior");
        }

        private static bool IsLikelyFurnitureName(string name) { return IsBedLikeName(name) || IsSeatLikeName(name) || IsTableLikeName(name); }
        private static bool IsBedLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            if (n.Contains("bedroom")) return false;
            return n == "bed" || n.StartsWith("bed_") || n.EndsWith("_bed") || n.Contains("mattress") || n.Contains("blanket") || n.Contains("futon");
        }
        private static bool IsSeatLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            return n.Contains("chair") || n.Contains("stool") || n.Contains("sofa") || n.Contains("seat") || n.Contains("bench") || n.Contains("couch");
        }
        private static bool IsTableLikeName(string name)
        {
            if (string.IsNullOrEmpty(name)) return false;
            string n = name.ToLowerInvariant();
            return n.Contains("table") || n.Contains("desk") || n.Contains("counter");
        }

        private static bool TryCalculateRendererBounds(Renderer[] renderers, out Bounds bounds)
        {
            bounds = new Bounds();
            bool has = false;
            for (int i = 0; i < renderers.Length; i++)
            {
                Renderer r = renderers[i];

                if (r == null ||
                    !r.enabled ||
                    r.gameObject == null ||
                    !r.gameObject.activeInHierarchy ||
                    IsOwnVisual(r.gameObject))
                {
                    continue;
                }
                try
                {
                    if (!has) { bounds = r.bounds; has = true; }
                    else bounds.Encapsulate(r.bounds);
                }
                catch { }
            }
            return has;
        }

        private static bool TryRaycastScene(Ray ray, out RaycastHit bestHit)
        {
            bestHit = new RaycastHit();
            RaycastHit[] hits = null;
            try { hits = Physics.RaycastAll(ray, RayDistance, -1, QueryTriggerInteraction.Ignore); } catch { }
            if (hits == null || hits.Length == 0) return false;
            bool found = false;
            float bestDistance = float.MaxValue;
            for (int i = 0; i < hits.Length; i++)
            {
                RaycastHit h = hits[i];
                if (h.collider == null || h.distance < 0.03f || IsOwnVisual(h.collider.gameObject)) continue;
                if (h.distance < bestDistance) { bestDistance = h.distance; bestHit = h; found = true; }
            }
            return found;
        }

        private static void AssignLayerRecursive(GameObject root, int layer, List<LayerBackup> backups)
        {
            if (root == null) return;
            backups.Add(new LayerBackup { go = root, layer = root.layer });
            root.layer = layer;
            Transform t = root.transform;
            if (t == null) return;
            for (int i = 0; i < t.childCount; i++)
            {
                Transform child = t.GetChild(i);
                if (child != null) AssignLayerRecursive(child.gameObject, layer, backups);
            }
        }

        private static void RestoreLayers(List<LayerBackup> backups)
        {
            if (backups == null) return;
            for (int i = 0; i < backups.Count; i++)
            {
                try { if (backups[i].go != null) backups[i].go.layer = backups[i].layer; } catch { }
            }
        }

        private static GameObject CreateDebugBoxObject(
            Bounds b,
            Transform parent,
            string objectName)
        {
            GameObject root =
                new GameObject(objectName);

            root.transform.SetParent(parent, true);

            Vector3 c = b.center;
            Vector3 e = b.extents;

            Vector3[] p =
            {
                c + new Vector3(-e.x, -e.y, -e.z),
                c + new Vector3( e.x, -e.y, -e.z),
                c + new Vector3(-e.x, -e.y,  e.z),
                c + new Vector3( e.x, -e.y,  e.z),
                c + new Vector3(-e.x,  e.y, -e.z),
                c + new Vector3( e.x,  e.y, -e.z),
                c + new Vector3(-e.x,  e.y,  e.z),
                c + new Vector3( e.x,  e.y,  e.z),
            };

            CreateLine(p[0], p[1], root.transform);
            CreateLine(p[0], p[2], root.transform);
            CreateLine(p[3], p[1], root.transform);
            CreateLine(p[3], p[2], root.transform);

            CreateLine(p[4], p[5], root.transform);
            CreateLine(p[4], p[6], root.transform);
            CreateLine(p[7], p[5], root.transform);
            CreateLine(p[7], p[6], root.transform);

            CreateLine(p[0], p[4], root.transform);
            CreateLine(p[1], p[5], root.transform);
            CreateLine(p[2], p[6], root.transform);
            CreateLine(p[3], p[7], root.transform);

            return root;
        }

        private static void CreateDebugBox(Bounds b, Transform parent)
        {
            GameObject root = new GameObject("ScanVolume_DebugBox");
            root.transform.SetParent(parent, true);
            Vector3 c = b.center;
            Vector3 e = b.extents;
            Vector3[] p =
            {
                c + new Vector3(-e.x, -e.y, -e.z), c + new Vector3( e.x, -e.y, -e.z),
                c + new Vector3(-e.x, -e.y,  e.z), c + new Vector3( e.x, -e.y,  e.z),
                c + new Vector3(-e.x,  e.y, -e.z), c + new Vector3( e.x,  e.y, -e.z),
                c + new Vector3(-e.x,  e.y,  e.z), c + new Vector3( e.x,  e.y,  e.z),
            };
            CreateLine(p[0], p[1], root.transform); CreateLine(p[0], p[2], root.transform); CreateLine(p[3], p[1], root.transform); CreateLine(p[3], p[2], root.transform);
            CreateLine(p[4], p[5], root.transform); CreateLine(p[4], p[6], root.transform); CreateLine(p[7], p[5], root.transform); CreateLine(p[7], p[6], root.transform);
            CreateLine(p[0], p[4], root.transform); CreateLine(p[1], p[5], root.transform); CreateLine(p[2], p[6], root.transform); CreateLine(p[3], p[7], root.transform);
        }

        private static void CreateLine(Vector3 a, Vector3 b, Transform parent)
        {
            Vector3 dir = b - a;
            float len = dir.magnitude;
            if (len < 0.001f) return;
            GameObject go = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            go.name = "DebugBox_Line";
            go.transform.position = a + dir * 0.5f;
            go.transform.rotation = Quaternion.FromToRotation(Vector3.up, dir.normalized);
            go.transform.localScale = new Vector3(0.01f, len * 0.5f, 0.01f);
            go.transform.SetParent(parent, true);
            SetRendererMaterial(go, _cyanMat, true);
            DisableCollider(go);
        }

    }
}
