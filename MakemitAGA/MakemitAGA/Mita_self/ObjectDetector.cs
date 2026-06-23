/*
 * [文件说明]: 视觉坐标转换与物体查找算法
 * 
 * [分析过程]:
 * 1. AI 返回的是 [0-1000] 的归一化坐标 (左上原点)，Unity 使用 [0-1] Viewport 坐标 (左下原点)。需要转换。
 * 2. [痛点解决]: 简单的 Raycast 无法检测大物体（如床），因为中心点可能不在框内。
 * 3. [解决方案]: 
 *    - 使用 "层级感知" (Hierarchy Aware)：从 House -> Room -> Prop 遍历，将父物体及其子物体的所有 Renderer 合并计算包围盒。
 *    - 使用 "2D包围盒重叠" (Rect Overlaps)：计算物体在屏幕上的投影矩形，与 AI 框选矩形求交集。
 * 
 * [主要功能]:
 * 1. FindObjectsInBox(): 接收 AI 坐标，使用 SnapshotCamera 进行投影计算，返回候选物体列表。
 * 2. HighlightObject(): 在目标位置生成一个临时的绿色半透明方块作为反馈。
 */
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

namespace MakemitAGA.Mita_self
{
    public static class ObjectDetector
    {
        private static Transform _cachedHouseRoot;
        private static GameObject _highlightBox;

        // 查找房屋根节点
        private static Transform GetHouseRoot()
        {
            if (_cachedHouseRoot != null) return _cachedHouseRoot;
            var allObjs = UnityEngine.Object.FindObjectsOfType<GameObject>();
            foreach (var go in allObjs)
            {
                if (go.name == "HouseGame Tamagotchi")
                {
                    var house = go.transform.Find("House");
                    if (house != null) { _cachedHouseRoot = house; return _cachedHouseRoot; }
                }
            }
            return null;
        }

        public static List<GameObject> FindObjectsInBox(float x1, float y1, float x2, float y2)
        {
            Camera cam = MitaVisionManager.SnapshotCamera;
            if (cam == null) return new List<GameObject>();

            // 1. 坐标转换
            float minX = x1 / 1000f;
            float maxX = x2 / 1000f;
            float minY = 1f - (y2 / 1000f);
            float maxY = 1f - (y1 / 1000f);
            Rect searchRect = Rect.MinMaxRect(minX, minY, maxX, maxY);
            Vector2 searchCenter = searchRect.center;

            Plugin.Logger.LogInfo($"[视觉] 搜索区域: {searchRect}");

            // 2. 获取候选物体 (层级遍历)
            List<Transform> propsToCheck = new List<Transform>();
            Transform houseRoot = GetHouseRoot();

            if (houseRoot != null)
            {
                int roomCount = houseRoot.childCount;
                for (int i = 0; i < roomCount; i++)
                {
                    Transform room = houseRoot.GetChild(i);
                    if (!room.gameObject.activeInHierarchy) continue;
                    int propCount = room.childCount;
                    for (int j = 0; j < propCount; j++)
                    {
                        Transform prop = room.GetChild(j);
                        if (prop.gameObject.activeInHierarchy) propsToCheck.Add(prop);
                    }
                }
            }
            else
            {
                // 降级方案
                var renderers = UnityEngine.Object.FindObjectsOfType<Renderer>();
                foreach (var r in renderers)
                {
                    if (r.transform.parent != null) propsToCheck.Add(r.transform.parent);
                    else propsToCheck.Add(r.transform);
                }
                propsToCheck = propsToCheck.Distinct().ToList();
            }

            var candidates = new List<(GameObject obj, float score)>();

            // 3. 筛选
            foreach (var prop in propsToCheck)
            {
                string name = prop.name.ToLower();
                if (name.Contains("blink") || name.Contains("shadow") || name.Contains("particle") || name.Contains("beam"))
                    continue;

                Bounds combinedBounds = new Bounds();
                bool hasBounds = false;
                var renderers = prop.GetComponentsInChildren<Renderer>();
                foreach (var r in renderers)
                {
                    if (!r.enabled) continue;
                    if (hasBounds) combinedBounds.Encapsulate(r.bounds);
                    else { combinedBounds = r.bounds; hasBounds = true; }
                }

                if (!hasBounds) continue;

                Rect screenRect = GetScreenRect(cam, combinedBounds);
                if (screenRect.width == 0) continue;

                if (searchRect.Overlaps(screenRect))
                {
                    float dist = Vector3.Distance(cam.transform.position, combinedBounds.center);
                    float centerOffset = Vector2.Distance(screenRect.center, searchCenter);
                    // 面积权重修正
                    float sizeScore = 1.0f / (screenRect.width * screenRect.height + 0.1f);

                    float score = dist * 1.0f + centerOffset * 3.0f + sizeScore * 0.5f;
                    candidates.Add((prop.gameObject, score));
                }
            }

            // 4. 排序返回
            return candidates.OrderBy(c => c.score).Select(c => c.obj).Take(10).ToList();
        }

        //public static void HighlightObject(GameObject target)
        //{
        //    if (target == null) return;
        //    if (_highlightBox != null) UnityEngine.Object.Destroy(_highlightBox);

        //    Bounds bounds = new Bounds(target.transform.position, Vector3.zero);
        //    bool hasBounds = false;
        //    var renderers = target.GetComponentsInChildren<Renderer>();
        //    foreach (var r in renderers)
        //    {
        //        if (hasBounds) bounds.Encapsulate(r.bounds);
        //        else { bounds = r.bounds; hasBounds = true; }
        //    }
        //    if (!hasBounds) return;

        //    _highlightBox = GameObject.CreatePrimitive(PrimitiveType.Cube);
        //    _highlightBox.name = "AI_Highlight_Box";
        //    UnityEngine.Object.Destroy(_highlightBox.GetComponent<Collider>());

        //    _highlightBox.transform.position = bounds.center;
        //    _highlightBox.transform.localScale = bounds.size * 1.05f;

        //    Material mat = new Material(Shader.Find("Sprites/Default"));
        //    mat.color = new Color(0f, 1f, 0f, 0.25f); // 绿色高亮
        //    var rBox = _highlightBox.GetComponent<Renderer>();
        //    rBox.material = mat;

        //    UnityEngine.Object.Destroy(_highlightBox, 4.0f);
        //}

        private static Rect GetScreenRect(Camera cam, Bounds bounds)
        {
            Vector3 c = bounds.center;
            Vector3 e = bounds.extents;
            // 8个角点
            Vector3[] corners = new Vector3[]
            {
                new Vector3(c.x-e.x, c.y-e.y, c.z-e.z), new Vector3(c.x+e.x, c.y-e.y, c.z-e.z),
                new Vector3(c.x-e.x, c.y-e.y, c.z+e.z), new Vector3(c.x+e.x, c.y-e.y, c.z+e.z),
                new Vector3(c.x-e.x, c.y+e.y, c.z-e.z), new Vector3(c.x+e.x, c.y+e.y, c.z-e.z),
                new Vector3(c.x-e.x, c.y+e.y, c.z+e.z), new Vector3(c.x+e.x, c.y+e.y, c.z+e.z)
            };

            float minX = 1f, maxX = 0f, minY = 1f, maxY = 0f;
            bool anyVisible = false;

            foreach (Vector3 corner in corners)
            {
                Vector3 v = cam.WorldToViewportPoint(corner);
                if (v.z > 0)
                {
                    anyVisible = true;
                    minX = Mathf.Min(minX, v.x);
                    maxX = Mathf.Max(maxX, v.x);
                    minY = Mathf.Min(minY, v.y);
                    maxY = Mathf.Max(maxY, v.y);
                }
            }
            if (!anyVisible) return Rect.zero;
            return Rect.MinMaxRect(Mathf.Clamp01(minX), Mathf.Clamp01(minY), Mathf.Clamp01(maxX), Mathf.Clamp01(maxY));
        }
    }
}