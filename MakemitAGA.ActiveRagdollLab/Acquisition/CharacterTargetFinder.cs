using System;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.ActiveRagdollLab.Acquisition
{
    internal static class CharacterTargetFinder
    {
        public static GameObject FindNearestExactName(
            string targetName,
            out int exactMatches,
            out string selectedPath,
            out string error)
        {
            exactMatches = 0;
            selectedPath = null;
            error = null;

            if (string.IsNullOrWhiteSpace(targetName))
            {
                error = "目标名称为空。";
                return null;
            }

            Vector3 reference = Vector3.zero;
            bool hasReference = false;
            try
            {
                Camera main = Camera.main;
                if (main != null && main.transform != null)
                {
                    reference = main.transform.position;
                    hasReference = true;
                }
            }
            catch
            {
                // Selection remains deterministic without a camera: use canonical path order below rather than
                // silently treating world origin as if it were the user's reference point.
                hasReference = false;
            }

            GameObject[] all;
            try
            {
                all = Object.FindObjectsOfType<GameObject>();
            }
            catch (Exception e)
            {
                error = "枚举活动 GameObject 失败：" + e.GetType().Name + " / " + e.Message;
                return null;
            }

            GameObject best = null;
            float bestDistance = float.MaxValue;
            string bestPath = null;

            for (int i = 0; i < all.Length; i++)
            {
                GameObject candidate = all[i];
                if (candidate == null || !candidate.activeInHierarchy)
                    continue;

                string name = candidate.name ?? string.Empty;
                if (name.StartsWith("ARL_", StringComparison.Ordinal))
                    continue;
                if (!name.Equals(targetName.Trim(), StringComparison.OrdinalIgnoreCase))
                    continue;

                exactMatches++;
                float distance = hasReference ? Vector3.Distance(reference, candidate.transform.position) : 0f;
                string path = GetPath(candidate.transform);
                bool better = best == null;
                if (!better && hasReference)
                {
                    better = distance < bestDistance - 0.0001f ||
                             (Mathf.Abs(distance - bestDistance) <= 0.0001f &&
                              string.Compare(path, bestPath, StringComparison.OrdinalIgnoreCase) < 0);
                }
                else if (!better)
                {
                    better = string.Compare(path, bestPath, StringComparison.OrdinalIgnoreCase) < 0;
                }
                if (better)
                {
                    best = candidate;
                    bestDistance = distance;
                    bestPath = path;
                }
            }

            if (best == null)
            {
                error = exactMatches == 0
                    ? "没有找到名称完整等于 “" + targetName + "” 的活动对象。"
                    : "同名对象选择失败。";
                return null;
            }

            selectedPath = bestPath;
            return best;
        }

        public static string GetPath(Transform transform)
        {
            if (transform == null)
                return "<null>";

            string path = transform.name;
            Transform parent = transform.parent;
            int guard = 0;
            while (parent != null && guard++ < 256)
            {
                path = parent.name + "/" + path;
                parent = parent.parent;
            }
            return path;
        }

        public static Transform FindRelativePath(Transform root, string relativePath)
        {
            if (root == null || string.IsNullOrWhiteSpace(relativePath))
                return null;

            string[] parts = relativePath.Trim().Trim('/').Replace('\\', '/').Split('/');
            Transform current = root;
            for (int i = 0; i < parts.Length; i++)
            {
                string expected = parts[i].Trim();
                if (expected.Length == 0)
                    continue;

                Transform next = null;
                int childCount = 0;
                try { childCount = current.childCount; }
                catch { return null; }

                for (int childIndex = 0; childIndex < childCount; childIndex++)
                {
                    Transform child;
                    try { child = current.GetChild(childIndex); }
                    catch { return null; }
                    if (child != null &&
                        (child.name ?? string.Empty).Equals(expected, StringComparison.OrdinalIgnoreCase))
                    {
                        next = child;
                        break;
                    }
                }

                if (next == null)
                    return null;
                current = next;
            }

            return current;
        }
    }
}
