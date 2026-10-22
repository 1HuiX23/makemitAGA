using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using BepInEx;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// 可编辑、可热重载的对象物理白名单。
    ///
    /// 文件采用竖线分隔，避免引入 JSON 依赖：
    /// objectPattern|partPattern|role|observation|collision|massKg|staticFriction|dynamicFriction|bounciness|materialTag|notes
    ///
    /// objectPattern 匹配对象完整路径或对象名；partPattern 匹配 Renderer 路径、Renderer 名、材质名或 Shader 名。
    /// 支持 * 与 ? 通配符。空字段或 auto 表示不覆盖自动判断。
    /// </summary>
    internal static class ObjectPhysicsWhitelist
    {
        internal sealed class PartDecision
        {
            public string AutoRole;
            public string ResolvedRole;
            public bool IncludeObservation;
            public bool IncludeCollision;
            public bool Matched;
            public string RuleSource;
            public string Reason;
        }

        private sealed class Rule
        {
            public int LineNumber;
            public string ObjectPattern;
            public string PartPattern;
            public string Role;
            public bool? IncludeObservation;
            public bool? IncludeCollision;
            public float? MassKg;
            public float? StaticFriction;
            public float? DynamicFriction;
            public float? Bounciness;
            public string MaterialTag;
            public string Notes;
            public int Specificity;
        }

        private static readonly List<Rule> Rules = new List<Rule>();
        private static string _path;
        private static bool _loaded;
        private static string _lastError;
        private static int _loadedRules;

        public static string Path
        {
            get
            {
                EnsurePath();
                return _path;
            }
        }

        public static string EnsureTemplate(ManualLogSource logger)
        {
            EnsurePath();
            try
            {
                Directory.CreateDirectory(System.IO.Path.GetDirectoryName(_path));
                if (!File.Exists(_path))
                {
                    File.WriteAllText(_path, TemplateText());
                    logger?.LogInfo("[GSP][PHYSICS_WHITELIST] template-created | path=" + _path);
                    return "已创建对象物理白名单模板：" + _path;
                }
                return "对象物理白名单已存在：" + _path;
            }
            catch (Exception e)
            {
                _lastError = e.GetType().Name + ":" + e.Message;
                logger?.LogWarning("[GSP][PHYSICS_WHITELIST] template-create-failed | " + _lastError);
                return "创建对象物理白名单模板失败：" + _lastError;
            }
        }


        /// <summary>
        /// 配置文件只负责保存用户确认的对象/部件语义。
        /// 源码不会按对象名称恢复、注入或覆盖任何规则。
        /// </summary>
        public static string Reload(ManualLogSource logger)
        {
            EnsureTemplate(logger);
            Rules.Clear();
            _loaded = false;
            _loadedRules = 0;
            _lastError = null;

            try
            {
                string[] lines = File.ReadAllLines(_path);
                for (int i = 0; i < lines.Length; i++)
                {
                    string line = (lines[i] ?? string.Empty).Trim();
                    if (line.Length == 0 || line.StartsWith("#", StringComparison.Ordinal))
                        continue;

                    string[] fields = line.Split('|');
                    if (fields.Length < 2)
                    {
                        logger?.LogWarning(
                            "[GSP][PHYSICS_WHITELIST] ignored malformed line | line=" + (i + 1));
                        continue;
                    }

                    Rule rule = new Rule
                    {
                        LineNumber = i + 1,
                        ObjectPattern = Field(fields, 0, "*"),
                        PartPattern = Field(fields, 1, "*"),
                        Role = NormalizeRole(Field(fields, 2, "AUTO")),
                        IncludeObservation = ParseNullableBool(Field(fields, 3, "auto")),
                        IncludeCollision = ParseNullableBool(Field(fields, 4, "auto")),
                        MassKg = ParseNullableFloat(Field(fields, 5, string.Empty)),
                        StaticFriction = ParseNullableFloat(Field(fields, 6, string.Empty)),
                        DynamicFriction = ParseNullableFloat(Field(fields, 7, string.Empty)),
                        Bounciness = ParseNullableFloat(Field(fields, 8, string.Empty)),
                        MaterialTag = Field(fields, 9, string.Empty),
                        Notes = Field(fields, 10, string.Empty)
                    };
                    rule.Specificity = PatternSpecificity(rule.ObjectPattern) * 4 +
                        PatternSpecificity(rule.PartPattern);
                    Rules.Add(rule);
                }

                _loaded = true;
                _loadedRules = Rules.Count;
                logger?.LogInfo(
                    "[GSP][PHYSICS_WHITELIST] reloaded | path=" + _path +
                    " | rules=" + _loadedRules);
                return "对象物理白名单已重载。rules=" + _loadedRules + "，path=" + _path;
            }
            catch (Exception e)
            {
                _lastError = e.GetType().Name + ":" + e.Message;
                logger?.LogWarning("[GSP][PHYSICS_WHITELIST] reload-failed | " + _lastError);
                return "对象物理白名单重载失败：" + _lastError;
            }
        }

        public static string GetStatusText()
        {
            EnsurePath();
            return
                "GSP object physics whitelist | loaded=" + _loaded +
                " | rules=" + _loadedRules +
                " | path=" + _path +
                " | error=" + (string.IsNullOrEmpty(_lastError) ? "<none>" : _lastError);
        }

        public static ObjectPhysicsProfileSnapshot ResolveObjectProfile(
            string objectPath,
            string objectName)
        {
            EnsureLoaded(null);
            ObjectPhysicsProfileSnapshot result = new ObjectPhysicsProfileSnapshot
            {
                WhitelistLoaded = _loaded,
                WhitelistPath = Path
            };

            Rule best = null;
            for (int i = 0; i < Rules.Count; i++)
            {
                Rule rule = Rules[i];
                if (!MatchesObject(rule, objectPath, objectName))
                    continue;
                if (!IsWholeObjectRule(rule.PartPattern))
                    continue;
                if (best == null || rule.Specificity > best.Specificity ||
                    (rule.Specificity == best.Specificity && rule.LineNumber > best.LineNumber))
                    best = rule;
            }

            if (best == null)
                return result;

            result.ObjectRuleMatched = true;
            result.ObjectRuleSource = Source(best);
            if (best.MassKg.HasValue)
            {
                result.MassConfigured = true;
                result.MassKg = best.MassKg.Value;
            }
            if (best.StaticFriction.HasValue)
            {
                result.StaticFrictionConfigured = true;
                result.StaticFriction = best.StaticFriction.Value;
            }
            if (best.DynamicFriction.HasValue)
            {
                result.DynamicFrictionConfigured = true;
                result.DynamicFriction = best.DynamicFriction.Value;
            }
            if (best.Bounciness.HasValue)
            {
                result.BouncinessConfigured = true;
                result.Bounciness = best.Bounciness.Value;
            }
            result.MaterialTag = best.MaterialTag;
            result.Notes = best.Notes;
            return result;
        }

        public static PartDecision ResolvePart(
            string objectPath,
            string objectName,
            string rendererPath,
            string rendererName,
            string materialName,
            string shaderName,
            string autoRole,
            bool autoObservation,
            bool autoCollision,
            string autoReason)
        {
            EnsureLoaded(null);
            PartDecision decision = new PartDecision
            {
                AutoRole = autoRole,
                ResolvedRole = autoRole,
                IncludeObservation = autoObservation,
                IncludeCollision = autoCollision,
                Reason = autoReason
            };

            string partDescriptor =
                (rendererPath ?? string.Empty) + "\n" +
                (rendererName ?? string.Empty) + "\n" +
                (materialName ?? string.Empty) + "\n" +
                (shaderName ?? string.Empty);

            Rule best = null;
            for (int i = 0; i < Rules.Count; i++)
            {
                Rule rule = Rules[i];
                if (!MatchesObject(rule, objectPath, objectName))
                    continue;
                bool hasPartBehaviorOverride =
                    !string.Equals(rule.Role, "AUTO", StringComparison.Ordinal) ||
                    rule.IncludeObservation.HasValue ||
                    rule.IncludeCollision.HasValue;
                if (!hasPartBehaviorOverride)
                    continue;
                if (!WildcardMatchAnyLine(partDescriptor, rule.PartPattern))
                    continue;
                if (best == null || rule.Specificity > best.Specificity ||
                    (rule.Specificity == best.Specificity && rule.LineNumber > best.LineNumber))
                    best = rule;
            }

            if (best == null)
                return decision;

            decision.Matched = true;
            decision.RuleSource = Source(best);
            if (!string.Equals(best.Role, "AUTO", StringComparison.Ordinal))
            {
                decision.ResolvedRole = best.Role;
                ApplyRoleDefaults(
                    best.Role,
                    out decision.IncludeObservation,
                    out decision.IncludeCollision);
            }
            if (best.IncludeObservation.HasValue)
                decision.IncludeObservation = best.IncludeObservation.Value;
            if (best.IncludeCollision.HasValue)
                decision.IncludeCollision = best.IncludeCollision.Value;
            decision.Reason =
                "whitelist override from " + Source(best) +
                (string.IsNullOrEmpty(best.Notes) ? string.Empty : "; " + best.Notes) +
                "; auto=" + autoReason;
            return decision;
        }

        private static void EnsureLoaded(ManualLogSource logger)
        {
            if (!_loaded)
                Reload(logger);
        }

        private static void EnsurePath()
        {
            if (!string.IsNullOrEmpty(_path))
                return;
            string config = null;
            try { config = Paths.ConfigPath; }
            catch { }
            if (string.IsNullOrEmpty(config))
                config = AppContext.BaseDirectory;
            _path = System.IO.Path.Combine(
                config,
                "MakemitAGA.GeometrySourceProbe.ObjectPhysicsWhitelist.txt");
        }

        private static string Field(string[] fields, int index, string fallback)
        {
            if (fields == null || index < 0 || index >= fields.Length)
                return fallback;
            string value = (fields[index] ?? string.Empty).Trim();
            return value.Length == 0 ? fallback : value;
        }

        private static void ApplyRoleDefaults(
            string role,
            out bool observation,
            out bool collision)
        {
            observation = false;
            collision = false;
            if (string.Equals(role, "PHYSICAL_CORE", StringComparison.Ordinal) ||
                string.Equals(role, "PHYSICAL_DETAIL", StringComparison.Ordinal))
            {
                observation = true;
                collision = true;
            }
            else if (string.Equals(role, "VISUAL_ONLY", StringComparison.Ordinal) ||
                string.Equals(role, "UNKNOWN_REVIEW", StringComparison.Ordinal))
            {
                observation = true;
                collision = false;
            }
        }

        private static string NormalizeRole(string value)
        {
            string role = (value ?? "AUTO").Trim().ToUpperInvariant();
            switch (role)
            {
                case "AUTO":
                case "PHYSICAL_CORE":
                case "PHYSICAL_DETAIL":
                case "VISUAL_ONLY":
                case "SHADOW_EFFECT_EXCLUDE":
                case "VISUAL_EFFECT_EXCLUDE":
                case "CHILD_OBJECT_EXCLUDE":
                case "UNKNOWN_REVIEW":
                case "SKINNED_DEFER":
                    return role;
                default:
                    return "AUTO";
            }
        }

        private static bool? ParseNullableBool(string value)
        {
            if (string.IsNullOrWhiteSpace(value) ||
                string.Equals(value, "auto", StringComparison.OrdinalIgnoreCase))
                return null;
            if (bool.TryParse(value, out bool parsed))
                return parsed;
            return null;
        }

        private static float? ParseNullableFloat(string value)
        {
            if (string.IsNullOrWhiteSpace(value) ||
                string.Equals(value, "auto", StringComparison.OrdinalIgnoreCase))
                return null;
            if (float.TryParse(
                value,
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out float parsed))
                return parsed;
            return null;
        }

        private static bool MatchesObject(Rule rule, string objectPath, string objectName)
        {
            return WildcardMatch(objectPath ?? string.Empty, rule.ObjectPattern) ||
                WildcardMatch(objectName ?? string.Empty, rule.ObjectPattern);
        }

        private static bool WildcardMatchAnyLine(string text, string pattern)
        {
            if (string.Equals(pattern, "*", StringComparison.Ordinal))
                return true;
            string[] lines = (text ?? string.Empty).Split('\n');
            for (int i = 0; i < lines.Length; i++)
            {
                if (WildcardMatch(lines[i], pattern))
                    return true;
            }
            return false;
        }

        private static bool WildcardMatch(string text, string pattern)
        {
            text = text ?? string.Empty;
            pattern = string.IsNullOrEmpty(pattern) ? "*" : pattern;
            int t = 0;
            int p = 0;
            int star = -1;
            int checkpoint = -1;
            while (t < text.Length)
            {
                if (p < pattern.Length &&
                    (pattern[p] == '?' ||
                     char.ToUpperInvariant(pattern[p]) == char.ToUpperInvariant(text[t])))
                {
                    p++;
                    t++;
                }
                else if (p < pattern.Length && pattern[p] == '*')
                {
                    star = p++;
                    checkpoint = t;
                }
                else if (star >= 0)
                {
                    p = star + 1;
                    t = ++checkpoint;
                }
                else
                {
                    return false;
                }
            }
            while (p < pattern.Length && pattern[p] == '*')
                p++;
            return p == pattern.Length;
        }

        private static bool IsWholeObjectRule(string partPattern)
        {
            return string.IsNullOrEmpty(partPattern) ||
                string.Equals(partPattern, "*", StringComparison.Ordinal) ||
                string.Equals(partPattern, "<object>", StringComparison.OrdinalIgnoreCase);
        }

        private static int PatternSpecificity(string pattern)
        {
            if (string.IsNullOrEmpty(pattern))
                return 0;
            int score = 0;
            for (int i = 0; i < pattern.Length; i++)
            {
                if (pattern[i] != '*' && pattern[i] != '?')
                    score++;
            }
            return score;
        }

        private static string Source(Rule rule)
        {
            return "line:" + rule.LineNumber +
                ",object:" + rule.ObjectPattern +
                ",part:" + rule.PartPattern;
        }

        private static string TemplateText()
        {
            return
@"# MakemitAGA Geometry Source Probe - Object Physics Whitelist
#
# 这是一份长期维护的对象/部件物理配置表。保存后在游戏控制台执行：
#   gsp_physics_whitelist_reload
# 本文件只保存并加载配置元数据，不会直接创建 Rigidbody、Collider 或 PhysicMaterial。
#
# 字段（使用 | 分隔）：
# objectPattern|partPattern|role|observation|collision|massKg|staticFriction|dynamicFriction|bounciness|materialTag|notes
#
# objectPattern：匹配对象完整路径或对象名称，支持 * 与 ?。
# partPattern：匹配 Renderer 路径、Renderer 名、材质名或 Shader 名。* 表示整个对象/所有部件。
# role：AUTO / PHYSICAL_CORE / PHYSICAL_DETAIL / VISUAL_ONLY /
#       SHADOW_EFFECT_EXCLUDE / VISUAL_EFFECT_EXCLUDE / CHILD_OBJECT_EXCLUDE / UNKNOWN_REVIEW / SKINNED_DEFER
# observation、collision：true / false / auto
# massKg、摩擦、弹性：空白表示暂不配置。数值使用小数点，不要写单位。
#
# 安全原则：
# 1. 阴影、光晕、Decal 等默认不进入碰撞聚合。
# 2. 图片/屏幕可进入视觉观测，但通常不进入碰撞聚合。
# 3. mass 与 friction 只记录配置；白名单本身不会创建 Rigidbody 或 PhysicMaterial。
# 4. 越具体、越靠后的匹配规则优先。
#
# 示例（全部以 # 注释，不会生效；确认实际路径后再取消 #）：
# */TargetObject|*DisplayOverlay*|VISUAL_ONLY|true|false|||||display-overlay|显示层保留观察、排除碰撞
# */TargetObject|*Shadow*|SHADOW_EFFECT_EXCLUDE|false|false|||||shadow-effect|阴影或投影面在读取前排除
# */TargetObject|DetachedChild|CHILD_OBJECT_EXCLUDE|false|false|||||independent-child|独立子物体不并入当前父对象
# */TargetObject|*|PHYSICAL_CORE|true|true|||||unclassified-solid|主体进入观察与碰撞候选；物理参数由实例配置维护
";
        }
    }
}
