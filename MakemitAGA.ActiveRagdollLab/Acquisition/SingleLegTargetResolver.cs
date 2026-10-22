using System;
using System.Collections.Generic;
using MakemitAGA.ActiveRagdollLab.Model;
using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Acquisition
{
    /// <summary>
    /// v0.2.0 单腿阶段的目标骨解析器。
    ///
    /// 当前解析优先级：
    /// 1) profile 显式路径；
    /// 2) Animator Humanoid HumanBodyBones；
    /// 3) MiSide 已实机确认的 canonical 层级；
    /// 4) 限定在 Armature/Hips 子树内的结构化名称 fallback。
    ///
    /// 重要：不能在整个 MitaPerson 下做无约束模糊搜索。MiSide 同时存在 IKLifeCharacter/RotateLegs
    /// 等辅助目标（LegLeft/LegRight/Legs Knee），它们不是实际骨架，绝不能作为 ARL Rigidbody 目标。
    /// 整个解析过程只读取源角色，不给源角色添加任何 Component，也不会修改 Transform。
    /// </summary>
    internal static class SingleLegTargetResolver
    {
        public static SingleLegTargetReport Resolve(Transform targetRoot, string side, string explicitUpperPath, string explicitLowerPath, string explicitFootPath)
        {
            SingleLegTargetReport report = new SingleLegTargetReport
            {
                RequestedSide = NormalizeSide(side),
                Success = false,
                ResolutionMode = "UNRESOLVED",
                ChainValidation = "NOT_VALIDATED"
            };

            if (targetRoot == null)
            {
                report.Error = "targetRoot=null";
                return report;
            }

            bool explicitRequested =
                !string.IsNullOrWhiteSpace(explicitUpperPath) ||
                !string.IsNullOrWhiteSpace(explicitLowerPath) ||
                !string.IsNullOrWhiteSpace(explicitFootPath);
            if (explicitRequested)
            {
                if (TryResolveExplicitPaths(targetRoot, report, explicitUpperPath, explicitLowerPath, explicitFootPath))
                    FinalizeGeometry(report, targetRoot);
                return report;
            }

            if (TryResolveHumanoid(targetRoot, report))
            {
                FinalizeGeometry(report, targetRoot);
                return report;
            }

            if (TryResolveMiSideCanonical(targetRoot, report))
            {
                FinalizeGeometry(report, targetRoot);
                return report;
            }

            if (TryResolveByHierarchy(targetRoot, report))
            {
                FinalizeGeometry(report, targetRoot);
                return report;
            }

            report.Error =
                "未解析到完整单腿链。已尝试 Explicit -> Humanoid -> MiSide canonical -> Hips-subtree structural fallback。" +
                " MiSide 预期主骨链为 Hips/Left leg/Left knee/Left ankle（右侧对称）；" +
                " IKLifeCharacter/RotateLegs、Collider*、Twist*、*Target 均不作为候选。";
            if (string.IsNullOrEmpty(report.ResolutionDetail))
                report.ResolutionDetail = "all-resolution-stages-failed";
            return report;
        }

        private static bool TryResolveExplicitPaths(
            Transform targetRoot,
            SingleLegTargetReport report,
            string upperPath,
            string lowerPath,
            string footPath)
        {
            bool any = !string.IsNullOrWhiteSpace(upperPath) || !string.IsNullOrWhiteSpace(lowerPath) || !string.IsNullOrWhiteSpace(footPath);
            if (!any)
                return false;
            if (string.IsNullOrWhiteSpace(upperPath) || string.IsNullOrWhiteSpace(lowerPath) || string.IsNullOrWhiteSpace(footPath))
            {
                report.Error = "显式单腿路径必须 Upper/Lower/Foot 三项全部填写，或三项全部留空使用 AUTO。";
                report.ResolutionDetail = "explicit-paths-partial";
                return false;
            }

            Transform upper = CharacterTargetFinder.FindRelativePath(targetRoot, upperPath);
            Transform lower = CharacterTargetFinder.FindRelativePath(targetRoot, lowerPath);
            Transform foot = CharacterTargetFinder.FindRelativePath(targetRoot, footPath);
            if (!TryAssignAndValidate(
                    targetRoot,
                    report,
                    upper,
                    lower,
                    foot,
                    "EXPLICIT_PROFILE_PATHS",
                    "explicit profile paths",
                    false))
            {
                if (string.IsNullOrEmpty(report.Error))
                    report.Error = "显式单腿路径未全部命中或结构验证失败。";
                return false;
            }

            return true;
        }

        private static bool TryResolveHumanoid(Transform targetRoot, SingleLegTargetReport report)
        {
            Animator[] animators = null;
            try { animators = targetRoot.gameObject.GetComponentsInChildren<Animator>(true); }
            catch
            {
                try
                {
                    Animator one = targetRoot.gameObject.GetComponentInChildren<Animator>();
                    if (one != null) animators = new[] { one };
                }
                catch { }
            }

            if (animators == null)
                return false;

            bool left = report.RequestedSide == "LEFT";
            HumanBodyBones upperBone = left ? HumanBodyBones.LeftUpperLeg : HumanBodyBones.RightUpperLeg;
            HumanBodyBones lowerBone = left ? HumanBodyBones.LeftLowerLeg : HumanBodyBones.RightLowerLeg;
            HumanBodyBones footBone = left ? HumanBodyBones.LeftFoot : HumanBodyBones.RightFoot;

            for (int i = 0; i < animators.Length; i++)
            {
                Animator animator = animators[i];
                if (animator == null)
                    continue;

                Transform upper = null;
                Transform lower = null;
                Transform foot = null;
                try
                {
                    upper = animator.GetBoneTransform(upperBone);
                    lower = animator.GetBoneTransform(lowerBone);
                    foot = animator.GetBoneTransform(footBone);
                }
                catch { }

                if (upper == null || lower == null || foot == null)
                    continue;

                if (TryAssignAndValidate(
                        targetRoot,
                        report,
                        upper,
                        lower,
                        foot,
                        "HUMANOID_ANIMATOR",
                        "Animator.GetBoneTransform HumanBodyBones",
                        false))
                    return true;
            }

            return false;
        }

        /// <summary>
        /// MiSide 当前 MitaPerson Mita 已实机确认的标准骨链：
        /// Armature/Hips/Left leg/Left knee/Left ankle/Left toe（右侧对称）。
        /// v0.2.0 动力学只使用 leg/knee 两个段，ankle 作为 LowerLeg 末端参考，toe 暂不建刚体。
        /// </summary>
        private static bool TryResolveMiSideCanonical(Transform targetRoot, SingleLegTargetReport report)
        {
            Transform hips = CharacterTargetFinder.FindRelativePath(targetRoot, "Armature/Hips");
            if (hips == null)
            {
                report.ResolutionDetail = "MiSide canonical: Armature/Hips not found";
                return false;
            }

            bool left = report.RequestedSide == "LEFT";
            string upperName = left ? "Left leg" : "Right leg";
            string lowerName = left ? "Left knee" : "Right knee";
            string footName = left ? "Left ankle" : "Right ankle";

            report.CandidateRootPath = CharacterTargetFinder.GetPath(hips);

            Transform upper = FindDirectChildIgnoreCase(hips, upperName);
            Transform lower = upper != null ? FindDirectChildIgnoreCase(upper, lowerName) : null;
            Transform foot = lower != null ? FindDirectChildIgnoreCase(lower, footName) : null;

            string expected = "Hips/" + upperName + "/" + lowerName + "/" + footName;
            if (upper == null || lower == null || foot == null)
            {
                report.ResolutionDetail =
                    "MiSide canonical miss: expected=" + expected +
                    " | hips=" + CharacterTargetFinder.GetPath(hips) +
                    " | upper=" + (upper != null ? upper.name : "<missing>") +
                    " | lower=" + (lower != null ? lower.name : "<missing>") +
                    " | foot=" + (foot != null ? foot.name : "<missing>");
                return false;
            }

            return TryAssignAndValidate(
                targetRoot,
                report,
                upper,
                lower,
                foot,
                "MISIDE_CANONICAL_HIERARCHY",
                "direct-child validated: " + expected + " | toe/helper children intentionally excluded",
                true);
        }

        /// <summary>
        /// 最后的通用 fallback 仍然是数据/结构驱动，但搜索范围被严格限制在真实 Hips 骨架子树中。
        /// 这样不会误选 MitaPerson 下平级的 IKLifeCharacter/RotateLegs/LegLeft 等辅助目标。
        /// </summary>
        private static bool TryResolveByHierarchy(Transform targetRoot, SingleLegTargetReport report)
        {
            Transform hips = FindHipsRoot(targetRoot);
            if (hips == null)
            {
                report.ResolutionDetail = "structural fallback: Hips root not found";
                return false;
            }

            report.CandidateRootPath = CharacterTargetFinder.GetPath(hips);

            string[] upperNames;
            string[] lowerNames;
            string[] footNames;
            if (report.RequestedSide == "LEFT")
            {
                // MiSide 的 "Left leg" 是 UpperLeg 根骨，不再作为 LowerLeg 候选。
                upperNames = new[] { "leftleg", "leftupperleg", "leftthigh", "upperlegl", "thighl", "lupperleg", "lthigh" };
                lowerNames = new[] { "leftknee", "leftlowerleg", "leftcalf", "lowerlegl", "calfl", "llowerleg", "lcalf" };
                // MiSide 的 ankle 是本阶段 Foot reference；真正 toe 留给未来 Foot Rigidbody。
                footNames = new[] { "leftankle", "leftfoot", "footl", "lfoot" };
            }
            else
            {
                upperNames = new[] { "rightleg", "rightupperleg", "rightthigh", "upperlegr", "thighr", "rupperleg", "rthigh" };
                lowerNames = new[] { "rightknee", "rightlowerleg", "rightcalf", "lowerlegr", "calfr", "rlowerleg", "rcalf" };
                footNames = new[] { "rightankle", "rightfoot", "footr", "rfoot" };
            }

            Transform upper = FindFirstByNormalizedNameInSubtree(hips, upperNames, false);
            Transform lower = upper != null ? FindFirstByNormalizedNameInSubtree(upper, lowerNames, false) : null;
            Transform foot = lower != null ? FindFirstByNormalizedNameInSubtree(lower, footNames, false) : null;

            if (upper == null || lower == null || foot == null)
            {
                report.ResolutionDetail =
                    "structural fallback miss" +
                    " | hips=" + CharacterTargetFinder.GetPath(hips) +
                    " | upper=" + (upper != null ? CharacterTargetFinder.GetPath(upper) : "<missing>") +
                    " | lower=" + (lower != null ? CharacterTargetFinder.GetPath(lower) : "<missing>") +
                    " | foot=" + (foot != null ? CharacterTargetFinder.GetPath(foot) : "<missing>");
                return false;
            }

            return TryAssignAndValidate(
                targetRoot,
                report,
                upper,
                lower,
                foot,
                "HIPS_SUBTREE_STRUCTURAL_FALLBACK",
                "name match inside Hips subtree + descendant-chain validation",
                false);
        }

        private static bool TryAssignAndValidate(
            Transform targetRoot,
            SingleLegTargetReport report,
            Transform upper,
            Transform lower,
            Transform foot,
            string mode,
            string detail,
            bool requireDirectChain)
        {
            report.Success = false;
            report.ChainValidation = "FAILED";

            if (upper == null || lower == null || foot == null)
            {
                report.Error = "单腿链存在空节点：Upper/Lower/Foot 必须全部命中。";
                return false;
            }
            if (upper == lower || upper == foot || lower == foot)
            {
                report.Error = "单腿链节点重复：Upper/Lower/Foot 必须是三个不同 Transform。";
                return false;
            }
            if (!IsDescendantOf(upper, targetRoot) || !IsDescendantOf(lower, targetRoot) || !IsDescendantOf(foot, targetRoot))
            {
                report.Error = "单腿链越出目标角色子树。";
                return false;
            }
            if (IsForbiddenHelperName(upper.name) || IsForbiddenHelperName(lower.name) || IsForbiddenHelperName(foot.name))
            {
                report.Error = "单腿链命中 Collider/IK/Target/Twist/Skirt 辅助节点，已拒绝。";
                return false;
            }

            if (requireDirectChain)
            {
                if (lower.parent != upper || foot.parent != lower)
                {
                    report.Error = "MiSide canonical 结构无效：Lower 必须是 Upper 直接子节点，Foot 必须是 Lower 直接子节点。";
                    return false;
                }
            }
            else if (!IsDescendantOf(lower, upper) || !IsDescendantOf(foot, lower))
            {
                report.Error = "单腿链层级关系无效：Lower 必须位于 Upper 后代，Foot 必须位于 Lower 后代。";
                return false;
            }

            report.UpperLeg = upper;
            report.LowerLeg = lower;
            report.Foot = foot;
            report.UpperLegPath = CharacterTargetFinder.GetPath(upper);
            report.LowerLegPath = CharacterTargetFinder.GetPath(lower);
            report.FootPath = CharacterTargetFinder.GetPath(foot);
            report.ResolutionMode = mode;
            report.ResolutionDetail = detail;
            report.ChainValidation = requireDirectChain ? "PASS_DIRECT_PARENT_CHAIN" : "PASS_DESCENDANT_CHAIN";
            report.Success = true;
            report.Error = null;
            return true;
        }

        private static void FinalizeGeometry(SingleLegTargetReport report, Transform targetRoot)
        {
            if (!report.Success || report.UpperLeg == null || report.LowerLeg == null || report.Foot == null)
                return;

            report.UpperLegLength = Vector3.Distance(report.UpperLeg.position, report.LowerLeg.position);
            report.LowerLegLength = Vector3.Distance(report.LowerLeg.position, report.Foot.position);
            if (!IsFinite(report.UpperLegLength) || !IsFinite(report.LowerLegLength) ||
                report.UpperLegLength < 0.05f || report.LowerLegLength < 0.05f)
            {
                report.Success = false;
                report.ChainValidation = "FAILED_GEOMETRY_LENGTH";
                report.Error = "单腿骨段长度无效：upper=" + report.UpperLegLength + " lower=" + report.LowerLegLength;
                return;
            }

            ResolveFootEndpoint(report);

            Vector3 characterRight = targetRoot != null ? targetRoot.right : Vector3.right;
            // Human knee 的主要屈伸轴通常接近角色左右轴。v0.2.0 只把它当作 Hinge 初始轴，
            // 并把实际值完整报告出来；后续版本会基于实机数据决定是否需要自动轴校准。
            if (characterRight.sqrMagnitude < 0.0001f)
                characterRight = Vector3.right;
            report.KneeFlexionAxisWorld = characterRight.normalized;
            report.KneeFlexionAxisLowerLocal = report.LowerLeg.InverseTransformDirection(report.KneeFlexionAxisWorld).normalized;
            report.ChainValidation += "+GEOMETRY_PASS";
        }


        private static void ResolveFootEndpoint(SingleLegTargetReport report)
        {
            report.FootFoundationReady = false;
            report.Toe = null;
            report.ToePath = null;
            report.ToeResolution = "UNRESOLVED";
            report.FootLength = 0f;
            if (report.Foot == null)
                return;

            bool left = report.RequestedSide == "LEFT";
            string canonicalName = left ? "Left toe" : "Right toe";
            Transform toe = FindDirectChildIgnoreCase(report.Foot, canonicalName);
            string mode = "DIRECT_CHILD_CANONICAL";

            if (toe == null)
            {
                string[] candidates = left
                    ? new[] { "lefttoe", "lefttoes", "toel", "ltoe", "toesl" }
                    : new[] { "righttoe", "righttoes", "toer", "rtoe", "toesr" };
                toe = FindFirstByNormalizedNameInSubtree(report.Foot, candidates, false);
                mode = "FOOT_SUBTREE_STRUCTURAL_FALLBACK";
            }

            if (toe == null || toe == report.Foot || !IsDescendantOf(toe, report.Foot) || IsForbiddenHelperName(toe.name))
            {
                report.ToeResolution = "UNRESOLVED_FROM_FOOT_SUBTREE";
                return;
            }

            float length = Vector3.Distance(report.Foot.position, toe.position);
            if (!IsFinite(length) || length < 0.03f || length > 0.60f)
            {
                report.ToeResolution = "INVALID_FOOT_LENGTH:" + length;
                return;
            }

            report.Toe = toe;
            report.ToePath = CharacterTargetFinder.GetPath(toe);
            report.ToeResolution = mode;
            report.FootLength = length;
            report.FootFoundationReady = true;
            report.ChainValidation += "+FOOT_ENDPOINT_PASS";
        }

        private static Transform FindHipsRoot(Transform targetRoot)
        {
            Transform hips = CharacterTargetFinder.FindRelativePath(targetRoot, "Armature/Hips");
            if (hips != null)
                return hips;

            Transform armature = FindDirectChildIgnoreCase(targetRoot, "Armature");
            if (armature != null)
            {
                Transform directHips = FindDirectChildIgnoreCase(armature, "Hips");
                if (directHips != null)
                    return directHips;
            }

            List<Transform> all = new List<Transform>();
            if (!TryCollectComplete(armature ?? targetRoot, all, 0))
                return null;
            return FindFirstByNormalizedName(all, new[] { "hips", "pelvis" });
        }

        private static Transform FindDirectChildIgnoreCase(Transform parent, string expectedName)
        {
            if (parent == null || string.IsNullOrEmpty(expectedName))
                return null;
            int childCount = 0;
            try { childCount = parent.childCount; }
            catch { return null; }

            for (int i = 0; i < childCount; i++)
            {
                Transform child;
                try { child = parent.GetChild(i); }
                catch { return null; }
                if (child != null &&
                    (child.name ?? string.Empty).Equals(expectedName, StringComparison.OrdinalIgnoreCase))
                    return child;
            }
            return null;
        }

        private static Transform FindFirstByNormalizedNameInSubtree(Transform root, string[] candidates, bool includeRoot)
        {
            List<Transform> all = new List<Transform>();
            if (!TryCollectComplete(root, all, 0))
                return null;
            int start = includeRoot ? 0 : 1;
            for (int c = 0; c < candidates.Length; c++)
            {
                string wanted = candidates[c];
                for (int i = start; i < all.Count; i++)
                {
                    Transform t = all[i];
                    if (t == null || IsForbiddenHelperName(t.name))
                        continue;
                    string normalized = NormalizeName(t.name);
                    if (normalized == wanted || normalized.EndsWith(wanted, StringComparison.Ordinal))
                        return t;
                }
            }
            return null;
        }

        private static string NormalizeSide(string side)
        {
            string value = (side ?? string.Empty).Trim();
            if (value.Equals("right", StringComparison.OrdinalIgnoreCase) || value.Equals("r", StringComparison.OrdinalIgnoreCase))
                return "RIGHT";
            return "LEFT";
        }

        private static bool TryCollectComplete(Transform current, List<Transform> output, int depth)
        {
            // Structural fallback must never select from a partially enumerated hierarchy. If IL2CPP traversal
            // fails at any node, reject this fallback stage and let the resolver fail closed instead of treating
            // the prefix gathered so far as the complete Hips subtree.
            if (current == null || depth > 256)
                return false;
            output.Add(current);

            int childCount;
            try { childCount = current.childCount; }
            catch { return false; }
            for (int i = 0; i < childCount; i++)
            {
                Transform child;
                try { child = current.GetChild(i); }
                catch { return false; }
                if (child == null || !TryCollectComplete(child, output, depth + 1))
                    return false;
            }
            return true;
        }

        private static Transform FindFirstByNormalizedName(List<Transform> all, string[] candidates)
        {
            for (int c = 0; c < candidates.Length; c++)
            {
                string wanted = candidates[c];
                for (int i = 0; i < all.Count; i++)
                {
                    Transform t = all[i];
                    if (t == null || IsForbiddenHelperName(t.name))
                        continue;
                    string normalized = NormalizeName(t.name);
                    if (normalized == wanted || normalized.EndsWith(wanted, StringComparison.Ordinal))
                        return t;
                }
            }
            return null;
        }

        private static bool IsForbiddenHelperName(string value)
        {
            string normalized = NormalizeName(value);
            if (normalized.Length == 0)
                return false;
            return normalized.StartsWith("collider", StringComparison.Ordinal) ||
                   normalized.StartsWith("ik", StringComparison.Ordinal) ||
                   normalized.EndsWith("target", StringComparison.Ordinal) ||
                   normalized.Contains("twist") ||
                   normalized.StartsWith("skirt", StringComparison.Ordinal);
        }

        private static string NormalizeName(string value)
        {
            if (string.IsNullOrEmpty(value))
                return string.Empty;
            char[] buffer = new char[value.Length];
            int count = 0;
            for (int i = 0; i < value.Length; i++)
            {
                char ch = char.ToLowerInvariant(value[i]);
                if (char.IsLetterOrDigit(ch))
                    buffer[count++] = ch;
            }
            return new string(buffer, 0, count);
        }

        private static bool IsDescendantOf(Transform child, Transform ancestor)
        {
            Transform current = child;
            int guard = 0;
            while (current != null && guard++ < 256)
            {
                if (current == ancestor)
                    return true;
                current = current.parent;
            }
            return false;
        }

        private static bool IsFinite(float value)
        {
            return !(float.IsNaN(value) || float.IsInfinity(value));
        }
    }
}
