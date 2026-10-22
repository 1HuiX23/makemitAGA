using System;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Configuration;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MakemitAGA.ActiveRagdollLab.Acquisition
{
    internal static class BoneHierarchyAuditor
    {
        public static BoneAuditReport Audit(string targetName, ActiveRagdollLabProfile profile)
        {
            BoneAuditReport report = new BoneAuditReport
            {
                Completed = true,
                Success = false,
                SceneName = SafeSceneName(),
                TargetName = targetName
            };

            GameObject target = CharacterTargetFinder.FindNearestExactName(
                targetName,
                out int matches,
                out string targetPath,
                out string findError);

            report.ExactMatches = matches;
            report.TargetPath = targetPath;
            if (target == null)
            {
                report.Error = findError;
                return report;
            }

            report.TargetRoot = target.transform;
            report.Pelvis = CharacterTargetFinder.FindRelativePath(target.transform, profile.PelvisBonePath);
            report.Spine = CharacterTargetFinder.FindRelativePath(target.transform, profile.SpineBonePath);
            report.Chest = CharacterTargetFinder.FindRelativePath(target.transform, profile.ChestBonePath);
            report.PelvisPath = report.Pelvis != null ? CharacterTargetFinder.GetPath(report.Pelvis) : "<missing>";
            report.SpinePath = report.Spine != null ? CharacterTargetFinder.GetPath(report.Spine) : "<missing>";
            report.ChestPath = report.Chest != null ? CharacterTargetFinder.GetPath(report.Chest) : "<missing>";

            Dictionary<string, int> nameCounts = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            Traverse(target.transform, target.transform, report, nameCounts, 0);

            foreach (KeyValuePair<string, int> pair in nameCounts)
            {
                if (pair.Value > 1)
                    report.DuplicateBoneNameCount += pair.Value - 1;
            }

            if (report.Pelvis == null || report.Chest == null)
            {
                report.Error =
                    "核心骨骼路径未全部解析。pelvis=" + profile.PelvisBonePath +
                    "，chest=" + profile.ChestBonePath +
                    "。请检查 BepInEx/config/" + ActiveRagdollLabProfile.FileName;
                return report;
            }

            report.PelvisToChestDistance = Vector3.Distance(report.Pelvis.position, report.Chest.position);
            report.PelvisToSpineDistance = report.Spine != null
                ? Vector3.Distance(report.Pelvis.position, report.Spine.position)
                : 0f;

            if (!IsFinite(report.PelvisToChestDistance) || report.PelvisToChestDistance < 0.05f)
            {
                report.Error = "Hips 到 Chest 的世界距离无效或过小：" + report.PelvisToChestDistance;
                return report;
            }

            Vector3 up = (report.Chest.position - report.Pelvis.position).normalized;
            Vector3 forward = Vector3.ProjectOnPlane(target.transform.forward, up);
            if (forward.sqrMagnitude < 0.0001f)
                forward = Vector3.ProjectOnPlane(Vector3.forward, up);
            if (forward.sqrMagnitude < 0.0001f)
                forward = Vector3.Cross(Vector3.right, up);
            forward.Normalize();

            report.CharacterUp = up;
            report.CharacterForward = forward;
            report.Success = true;
            report.Error = null;
            return report;
        }

        private static void Traverse(
            Transform root,
            Transform current,
            BoneAuditReport report,
            Dictionary<string, int> nameCounts,
            int depth)
        {
            if (current == null || depth > 256)
                return;

            report.TransformCount++;
            string name = current.name ?? "<unnamed>";
            if (nameCounts.TryGetValue(name, out int count)) nameCounts[name] = count + 1;
            else nameCounts[name] = 1;

            int animatorCount = CountComponents<Animator>(current.gameObject);
            int rigidbodyCount = CountComponents<Rigidbody>(current.gameObject);
            int jointCount = CountComponents<Joint>(current.gameObject);
            int colliderCount = CountComponents<Collider>(current.gameObject);
            report.AnimatorCount += animatorCount;
            report.RigidbodyCount += rigidbodyCount;
            report.JointCount += jointCount;
            report.ColliderCount += colliderCount;

            string relative = GetRelativePath(root, current);
            report.Lines.Add(
                relative +
                " | pos=" + Format(current.position) +
                " | rot=" + Format(current.rotation.eulerAngles) +
                " | components=Animator:" + animatorCount +
                ",Rigidbody:" + rigidbodyCount +
                ",Joint:" + jointCount +
                ",Collider:" + colliderCount);

            int childCount = 0;
            try { childCount = current.childCount; }
            catch { }
            for (int i = 0; i < childCount; i++)
            {
                Transform child = null;
                try { child = current.GetChild(i); }
                catch { }
                if (child != null)
                    Traverse(root, child, report, nameCounts, depth + 1);
            }
        }

        private static int CountComponents<T>(GameObject gameObject) where T : Component
        {
            try
            {
                T[] items = gameObject.GetComponents<T>();
                return items == null ? 0 : items.Length;
            }
            catch
            {
                try { return gameObject.GetComponent<T>() != null ? 1 : 0; }
                catch { return 0; }
            }
        }

        private static string GetRelativePath(Transform root, Transform current)
        {
            if (root == current)
                return ".";
            string path = current.name;
            Transform parent = current.parent;
            int guard = 0;
            while (parent != null && parent != root && guard++ < 256)
            {
                path = parent.name + "/" + path;
                parent = parent.parent;
            }
            return path;
        }

        private static bool IsFinite(float value)
        {
            return !(float.IsNaN(value) || float.IsInfinity(value));
        }

        private static string Format(Vector3 value)
        {
            return "(" + value.x.ToString("G7") + "," + value.y.ToString("G7") + "," + value.z.ToString("G7") + ")";
        }

        private static string SafeSceneName()
        {
            try { return SceneManager.GetActiveScene().name; }
            catch { return "<unknown>"; }
        }
    }
}
