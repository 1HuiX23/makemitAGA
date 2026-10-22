using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using BepInEx;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// 空间实例级物理白名单。
    ///
    /// objectPattern|instancePattern|channel|enabled|proxyType|fixedToWorld|pickable|
    /// massKg|staticFriction|dynamicFriction|bounciness|materialTag|minimumThicknessMeters|notes
    ///
    /// instancePattern 优先匹配 collision instanceKey；观察实例若已经关联碰撞实例，
    /// 也会使用 associatedCollisionKey 参与匹配。配置只进入审计报告，不应用到 Unity 物理对象。
    /// </summary>
    internal static class SpatialInstancePhysicsWhitelist
    {
        private sealed class Rule
        {
            public int LineNumber;
            public string ObjectPattern;
            public string InstancePattern;
            public string Channel;
            public bool? Enabled;
            public string ProxyType;
            public bool? FixedToWorld;
            public bool? Pickable;
            public float? MassKg;
            public float? StaticFriction;
            public float? DynamicFriction;
            public float? Bounciness;
            public string MaterialTag;
            public float? MinimumThicknessMeters;
            public string Notes;
            public int Specificity;
        }

        private static readonly List<Rule> Rules = new List<Rule>();
        private static string _path;
        private static bool _loaded;
        private static int _loadedRules;
        private static string _lastError;

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
                    logger?.LogInfo("[GSP][INSTANCE_PROFILE] template-created | path=" + _path);
                    return "已创建空间实例物理白名单模板：" + _path;
                }
                return "空间实例物理白名单已存在：" + _path;
            }
            catch (Exception e)
            {
                _lastError = e.GetType().Name + ":" + e.Message;
                logger?.LogWarning("[GSP][INSTANCE_PROFILE] template-create-failed | " + _lastError);
                return "创建空间实例物理白名单失败：" + _lastError;
            }
        }


        /// <summary>
        /// 实例配置只从用户文件读取。源码不会按对象名称恢复、注入或覆盖
        /// 代理类型、固定状态、可拿取状态、质量、摩擦或材质。
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
                        logger?.LogWarning("[GSP][INSTANCE_PROFILE] ignored malformed line | line=" + (i + 1));
                        continue;
                    }

                    Rule rule = new Rule
                    {
                        LineNumber = i + 1,
                        ObjectPattern = Field(fields, 0, "*"),
                        InstancePattern = Field(fields, 1, "*"),
                        Channel = NormalizeChannel(Field(fields, 2, "any")),
                        Enabled = ParseNullableBool(Field(fields, 3, "auto")),
                        ProxyType = NormalizeProxyType(Field(fields, 4, "AUTO")),
                        FixedToWorld = ParseNullableBool(Field(fields, 5, "auto")),
                        Pickable = ParseNullableBool(Field(fields, 6, "auto")),
                        MassKg = ParseNullableFloat(Field(fields, 7, string.Empty)),
                        StaticFriction = ParseNullableFloat(Field(fields, 8, string.Empty)),
                        DynamicFriction = ParseNullableFloat(Field(fields, 9, string.Empty)),
                        Bounciness = ParseNullableFloat(Field(fields, 10, string.Empty)),
                        MaterialTag = Field(fields, 11, string.Empty),
                        MinimumThicknessMeters = ParseNullableFloat(Field(fields, 12, string.Empty)),
                        Notes = Field(fields, 13, string.Empty)
                    };
                    rule.Specificity =
                        PatternSpecificity(rule.ObjectPattern) * 8 +
                        PatternSpecificity(rule.InstancePattern) * 2 +
                        (string.Equals(rule.Channel, "any", StringComparison.Ordinal) ? 0 : 1);
                    Rules.Add(rule);
                }

                _loaded = true;
                _loadedRules = Rules.Count;
                logger?.LogInfo("[GSP][INSTANCE_PROFILE] reloaded | path=" + _path + " | rules=" + _loadedRules);
                return "空间实例物理白名单已重载。rules=" + _loadedRules + "，path=" + _path;
            }
            catch (Exception e)
            {
                _lastError = e.GetType().Name + ":" + e.Message;
                logger?.LogWarning("[GSP][INSTANCE_PROFILE] reload-failed | " + _lastError);
                return "空间实例物理白名单重载失败：" + _lastError;
            }
        }

        public static string GetStatusText()
        {
            EnsurePath();
            return
                "GSP spatial instance physics whitelist | loaded=" + _loaded +
                " | rules=" + _loadedRules +
                " | path=" + _path +
                " | error=" + (string.IsNullOrEmpty(_lastError) ? "<none>" : _lastError);
        }

        public static SpatialInstancePhysicsProfileSnapshot Resolve(
            string objectPath,
            string objectName,
            string instanceKey,
            string associatedCollisionKey,
            string channel)
        {
            EnsureLoaded(null);
            SpatialInstancePhysicsProfileSnapshot result =
                new SpatialInstancePhysicsProfileSnapshot
                {
                    WhitelistLoaded = _loaded,
                    WhitelistPath = Path
                };

            Rule best = null;
            string bestSelector = null;
            for (int i = 0; i < Rules.Count; i++)
            {
                Rule rule = Rules[i];
                if (!MatchesObject(rule, objectPath, objectName))
                    continue;
                if (!MatchesChannel(rule.Channel, channel))
                    continue;

                string selector = null;
                if (WildcardMatch(instanceKey ?? string.Empty, rule.InstancePattern))
                    selector = instanceKey;
                else if (!string.IsNullOrEmpty(associatedCollisionKey) &&
                    WildcardMatch(associatedCollisionKey, rule.InstancePattern))
                    selector = associatedCollisionKey;
                if (selector == null)
                    continue;

                if (best == null || rule.Specificity > best.Specificity ||
                    (rule.Specificity == best.Specificity && rule.LineNumber > best.LineNumber))
                {
                    best = rule;
                    bestSelector = selector;
                }
            }

            if (best == null)
                return result;

            result.Matched = true;
            result.RuleSource = Source(best);
            result.MatchedInstanceSelector = bestSelector;
            if (best.Enabled.HasValue)
            {
                result.EnabledConfigured = true;
                result.Enabled = best.Enabled.Value;
            }
            if (!string.Equals(best.ProxyType, "AUTO", StringComparison.Ordinal))
            {
                result.ProxyTypeConfigured = true;
                result.ProxyType = best.ProxyType;
            }
            if (best.FixedToWorld.HasValue)
            {
                result.FixedToWorldConfigured = true;
                result.FixedToWorld = best.FixedToWorld.Value;
            }
            if (best.Pickable.HasValue)
            {
                result.PickableConfigured = true;
                result.Pickable = best.Pickable.Value;
            }
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
            if (best.MinimumThicknessMeters.HasValue)
            {
                result.MinimumThicknessConfigured = true;
                result.MinimumThicknessMeters = best.MinimumThicknessMeters.Value;
            }
            result.MaterialTag = best.MaterialTag;
            result.Notes = best.Notes;
            return result;
        }

        public static string SeedCurrentReport(
            MultipartAggregationReport report,
            ManualLogSource logger)
        {
            if (report == null || !report.Completed || !report.Success ||
                report.CollisionSpatialInstances == null ||
                !report.CollisionSpatialInstances.Analyzed ||
                report.CollisionSpatialInstances.Instances.Count == 0)
            {
                return "当前没有已完成且包含碰撞空间实例的成功报告。请先执行 gsp_instance_audit(对象名称)。";
            }

            EnsureTemplate(logger);
            string targetToken = StableMarkerToken(report.TargetPath);
            string beginMarker = "# BEGIN GSP INSTANCE PROFILE DRAFT " + targetToken;
            string endMarker = "# END GSP INSTANCE PROFILE DRAFT " + targetToken;
            try
            {
                string existing = File.Exists(_path) ? File.ReadAllText(_path) : string.Empty;
                existing = RemoveMarkedBlock(existing, beginMarker, endMarker, out bool ignoredFound, out string removeError);
                if (!string.IsNullOrEmpty(removeError))
                    return "旧实例配置草案块损坏，停止自动覆盖：" + removeError;

                StringBuilder block = new StringBuilder();
                block.AppendLine();
                block.AppendLine(beginMarker);
                block.AppendLine("# 自动从当前碰撞空间实例生成。数值是自动生成的待校准草案；只有通过运行时安全门的字段才会被物理模块采用。");
                for (int i = 0; i < report.CollisionSpatialInstances.Instances.Count; i++)
                {
                    SpatialInstanceReport item = report.CollisionSpatialInstances.Instances[i];
                    if (item == null || string.IsNullOrEmpty(item.InstanceKey))
                        continue;

                    string proxyType = string.IsNullOrEmpty(item.ProxyDraftType)
                        ? "AUTO"
                        : item.ProxyDraftType;
                    // 自动草案只复制几何推导出的代理类型和最小厚度。
                    // 固定/可拿取、质量、摩擦、弹性和材质都需要用户显式标定，
                    // 不允许按对象名称猜测。
                    string fixedValue = "auto";
                    string pickableValue = "auto";
                    string mass = string.Empty;
                    string staticFriction = string.Empty;
                    string dynamicFriction = string.Empty;
                    string bounce = string.Empty;
                    string material = "unclassified";
                    string minThickness = item.ProxyDraftBuilt
                        ? Math.Max(0.01f, item.ProxyRawSize.z).ToString("G9", CultureInfo.InvariantCulture)
                        : string.Empty;
                    string notes =
                        "自动草案；instanceIndex=" + item.InstanceIndex +
                        "；proxy=" + proxyType +
                        "；请在长期使用前重新标定质量、摩擦与固定状态";

                    block.Append(CleanField(report.TargetPath)).Append('|')
                         .Append(CleanField(item.InstanceKey)).Append('|')
                         .Append("collision|true|")
                         .Append(CleanField(proxyType)).Append('|')
                         .Append(fixedValue).Append('|')
                         .Append(pickableValue).Append('|')
                         .Append(mass).Append('|')
                         .Append(staticFriction).Append('|')
                         .Append(dynamicFriction).Append('|')
                         .Append(bounce).Append('|')
                         .Append(CleanField(material)).Append('|')
                         .Append(minThickness).Append('|')
                         .Append(CleanField(notes)).AppendLine();
                }
                block.AppendLine(endMarker);

                File.WriteAllText(_path, existing.TrimEnd() + Environment.NewLine + block.ToString());
                string reload = Reload(logger);
                logger?.LogInfo(
                    "[GSP][INSTANCE_PROFILE] current-report-seeded | target=" + report.TargetName +
                    " | instances=" + report.CollisionSpatialInstances.Instances.Count +
                    " | path=" + _path);
                return "已按当前碰撞空间实例写入精确 instanceKey 配置草案。" + reload;
            }
            catch (Exception e)
            {
                _lastError = e.GetType().Name + ":" + e.Message;
                logger?.LogWarning("[GSP][INSTANCE_PROFILE] seed-current-failed | " + _lastError);
                return "写入当前实例配置草案失败：" + _lastError;
            }
        }

        public static string RemoveCurrentReportSeed(
            MultipartAggregationReport report,
            ManualLogSource logger)
        {
            if (report == null || string.IsNullOrEmpty(report.TargetPath))
                return "当前没有可用于定位规则块的空间实例报告。";
            EnsureTemplate(logger);
            string targetToken = StableMarkerToken(report.TargetPath);
            string beginMarker = "# BEGIN GSP INSTANCE PROFILE DRAFT " + targetToken;
            string endMarker = "# END GSP INSTANCE PROFILE DRAFT " + targetToken;
            try
            {
                string existing = File.Exists(_path) ? File.ReadAllText(_path) : string.Empty;
                string updated = RemoveMarkedBlock(existing, beginMarker, endMarker, out bool found, out string error);
                if (!string.IsNullOrEmpty(error))
                    return "实例配置草案块损坏，停止自动删除：" + error;
                if (!found)
                {
                    Reload(logger);
                    return "未找到当前对象的实例配置草案块，无需删除。";
                }
                File.WriteAllText(_path, updated);
                string reload = Reload(logger);
                logger?.LogInfo("[GSP][INSTANCE_PROFILE] current-report-seed-removed | target=" + report.TargetName);
                return "已删除当前对象的实例配置草案块。" + reload;
            }
            catch (Exception e)
            {
                _lastError = e.GetType().Name + ":" + e.Message;
                return "删除当前实例配置草案失败：" + _lastError;
            }
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
                "MakemitAGA.GeometrySourceProbe.SpatialInstancePhysicsWhitelist.txt");
        }

        private static string Field(string[] fields, int index, string fallback)
        {
            if (fields == null || index < 0 || index >= fields.Length)
                return fallback;
            string value = (fields[index] ?? string.Empty).Trim();
            return value.Length == 0 ? fallback : value;
        }

        private static string NormalizeChannel(string value)
        {
            string channel = (value ?? "any").Trim().ToLowerInvariant();
            if (channel == "collision" || channel == "observation")
                return channel;
            return "any";
        }

        private static string NormalizeProxyType(string value)
        {
            string proxy = (value ?? "AUTO").Trim().ToUpperInvariant();
            switch (proxy)
            {
                case "AUTO":
                case "ORIENTED_THIN_BOX":
                case "ORIENTED_BOX":
                case "COMPOUND_BOX":
                case "STATIC_NONCONVEX_MESH":
                case "COMPOUND_REVIEW":
                case "NONE":
                    return proxy;
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

        private static bool MatchesChannel(string ruleChannel, string actualChannel)
        {
            return string.Equals(ruleChannel, "any", StringComparison.Ordinal) ||
                string.Equals(ruleChannel, actualChannel ?? string.Empty, StringComparison.OrdinalIgnoreCase);
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
                ",instance:" + rule.InstancePattern +
                ",channel:" + rule.Channel;
        }

        private static string StableMarkerToken(string value)
        {
            ulong hash = 14695981039346656037UL;
            const ulong prime = 1099511628211UL;
            string text = value ?? string.Empty;
            for (int i = 0; i < text.Length; i++)
            {
                char ch = text[i];
                hash ^= (byte)(ch & 0xff);
                hash *= prime;
                hash ^= (byte)(ch >> 8);
                hash *= prime;
            }
            return hash.ToString("X16");
        }

        private static string RemoveMarkedBlock(
            string existing,
            string beginMarker,
            string endMarker,
            out bool found,
            out string error)
        {
            found = false;
            error = null;
            existing = existing ?? string.Empty;
            int begin = existing.IndexOf(beginMarker, StringComparison.Ordinal);
            if (begin < 0)
                return existing;
            int end = existing.IndexOf(endMarker, begin, StringComparison.Ordinal);
            if (end < 0)
            {
                error = "missing-end-marker:" + endMarker;
                return existing;
            }
            end += endMarker.Length;
            while (end < existing.Length &&
                (existing[end] == '\r' || existing[end] == '\n'))
                end++;
            found = true;
            return existing.Remove(begin, end - begin);
        }

        private static string CleanField(string value)
        {
            if (string.IsNullOrEmpty(value))
                return string.Empty;
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }

        private static string TemplateText()
        {
            return
@"# MakemitAGA Geometry Source Probe - Spatial Instance Physics Whitelist
#
# 保存后执行：gsp_instance_profile_reload
# 由当前审计报告生成精确 instanceKey 配置草案：gsp_instance_profile_seed_current
#
# 字段：
# objectPattern|instancePattern|channel|enabled|proxyType|fixedToWorld|pickable|massKg|staticFriction|dynamicFriction|bounciness|materialTag|minimumThicknessMeters|notes
#
# channel：collision / observation / any
# proxyType：AUTO / ORIENTED_THIN_BOX / ORIENTED_BOX / COMPOUND_BOX / STATIC_NONCONVEX_MESH / COMPOUND_REVIEW / NONE
# fixedToWorld、pickable、enabled：true / false / auto
# minimumThicknessMeters：主要影响薄盒草案；STATIC_NONCONVEX_MESH 使用审计保存的精确聚合几何。
#
# 安全说明：
# 1. 本文件本身不创建物理组件；盒代理使用 gsp_instance_proxy_build，静态源网格使用 gsp_instance_static_mesh_build。
# 2. 质量与摩擦数值必须在真实动态测试后重新标定；自动种子只是验证规则匹配。
# 3. 建议使用精确 instanceKey。规则越具体、越靠后，优先级越高。
#
# 示例：
# */FixedDecoration|SI-COLLISION-*|collision|true|ORIENTED_THIN_BOX|true|false|||||unclassified-fixed|固定薄装饰示例；请改为精确 instanceKey
# */PortableObject|SI-COLLISION-*|collision|true|ORIENTED_BOX|false|true|||||unclassified-portable|可移动物体示例；质量和摩擦必须实测后填写
";
        }
    }
}
