using System;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    internal static class TargetFinder
    {
        public static GameObject FindNearestExactName(
            string targetName,
            out int exactMatches,
            out float selectedDistance,
            out string referenceName,
            out string error)
        {
            exactMatches = 0;
            selectedDistance = float.MaxValue;
            referenceName = "WorldOrigin";
            error = null;

            if (string.IsNullOrWhiteSpace(targetName))
            {
                error = "对象名称为空。";
                return null;
            }

            Vector3 reference = Vector3.zero;
            Camera camera = FindBestCamera();
            if (camera != null)
            {
                reference = camera.transform.position;
                referenceName = "Camera:" + camera.name;
            }

            GameObject[] all;
            try { all = Object.FindObjectsOfType<GameObject>(); }
            catch (Exception e)
            {
                error = "枚举场景对象失败：" + e.GetType().Name + " / " + e.Message;
                return null;
            }

            GameObject best = null;
            string bestPath = null;

            for (int i = 0; i < all.Length; i++)
            {
                GameObject candidate = all[i];
                if (candidate == null || !candidate.activeInHierarchy)
                    continue;
                if ((candidate.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal))
                    continue;
                if (!(candidate.name ?? string.Empty).Equals(
                    targetName,
                    StringComparison.OrdinalIgnoreCase))
                    continue;

                exactMatches++;
                if (!TryGetBounds(candidate, out Bounds bounds))
                    continue;

                float distance = Vector3.Distance(reference, bounds.center);
                string path = GetPath(candidate.transform);
                if (best == null ||
                    distance < selectedDistance - 0.0001f ||
                    (Mathf.Abs(distance - selectedDistance) <= 0.0001f &&
                     string.Compare(path, bestPath, StringComparison.OrdinalIgnoreCase) < 0))
                {
                    best = candidate;
                    bestPath = path;
                    selectedDistance = distance;
                }
            }

            if (exactMatches == 0)
                error = "没有找到名称完整等于 “" + targetName + "” 的活动对象。";
            else if (best == null)
                error = "找到了同名对象，但没有 Renderer 或 Collider Bounds。";

            return best;
        }

        /// <summary>
        /// 按审计时保存的完整层级路径重新绑定目标。
        /// provider build 不允许退回“同名 + 最近相机”选择，否则同名对象可能继承错误 layer/Collider 身份。
        /// 只接受活动、非 GSP 自有对象的唯一精确匹配；0 个或多个匹配都 fail-close。
        /// </summary>
        public static GameObject FindUniqueExactPath(
            string targetPath,
            out int exactMatches,
            out string error)
        {
            exactMatches = 0;
            error = null;
            if (string.IsNullOrWhiteSpace(targetPath))
            {
                error = "目标完整路径为空。";
                return null;
            }

            GameObject[] all;
            try { all = Object.FindObjectsOfType<GameObject>(); }
            catch (Exception e)
            {
                error = "枚举场景对象失败：" + e.GetType().Name + " / " + e.Message;
                return null;
            }

            GameObject matched = null;
            for (int i = 0; i < all.Length; i++)
            {
                GameObject candidate = all[i];
                if (candidate == null || !candidate.activeInHierarchy)
                    continue;
                if ((candidate.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal))
                    continue;

                string path;
                try { path = GetPath(candidate.transform); }
                catch { continue; }
                if (!string.Equals(path, targetPath, StringComparison.Ordinal))
                    continue;

                exactMatches++;
                matched = candidate;
            }

            if (exactMatches == 1 && matched != null)
                return matched;

            if (exactMatches == 0)
                error = "没有找到完整路径精确等于 “" + targetPath + "” 的活动对象。";
            else
                error = "完整路径出现 " + exactMatches + " 个活动匹配，无法建立唯一 provider 身份：" + targetPath;
            return null;
        }

        public static string GetPath(Transform transform)
        {
            if (transform == null)
                return "<null>";

            string path = transform.name;
            Transform parent = transform.parent;
            int guard = 0;
            while (parent != null && guard++ < 128)
            {
                path = parent.name + "/" + path;
                parent = parent.parent;
            }
            return path;
        }

        private static Camera FindBestCamera()
        {
            try
            {
                if (Camera.main != null)
                    return Camera.main;
                Camera[] cameras = Object.FindObjectsOfType<Camera>();
                for (int i = 0; i < cameras.Length; i++)
                {
                    if (cameras[i] != null && cameras[i].enabled && cameras[i].gameObject.activeInHierarchy)
                        return cameras[i];
                }
            }
            catch { }
            return null;
        }

        private static bool TryGetBounds(GameObject root, out Bounds bounds)
        {
            bounds = new Bounds();
            bool has = false;

            try
            {
                Renderer[] renderers = root.GetComponentsInChildren<Renderer>(true);
                for (int i = 0; i < renderers.Length; i++)
                {
                    Renderer renderer = renderers[i];
                    if (renderer == null)
                        continue;
                    if (!has) { bounds = renderer.bounds; has = true; }
                    else bounds.Encapsulate(renderer.bounds);
                }
            }
            catch { }

            try
            {
                Collider[] colliders = root.GetComponentsInChildren<Collider>(true);
                for (int i = 0; i < colliders.Length; i++)
                {
                    Collider collider = colliders[i];
                    if (collider == null)
                        continue;
                    if (!has) { bounds = collider.bounds; has = true; }
                    else bounds.Encapsulate(collider.bounds);
                }
            }
            catch { }

            return has;
        }
    }
}
