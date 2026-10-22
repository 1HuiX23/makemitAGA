using System;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 空间实例级物理白名单的只读快照。
    /// 该配置模型只保存物理元数据；是否创建 Collider/Rigidbody 由独立运行时模块及安全门决定。
    /// </summary>
    internal sealed class SpatialInstancePhysicsProfileSnapshot
    {
        public bool WhitelistLoaded;
        public string WhitelistPath;
        public bool Matched;
        public string RuleSource;
        public string MatchedInstanceSelector;

        public bool EnabledConfigured;
        public bool Enabled;
        public bool ProxyTypeConfigured;
        public string ProxyType;
        public bool FixedToWorldConfigured;
        public bool FixedToWorld;
        public bool PickableConfigured;
        public bool Pickable;
        public bool MassConfigured;
        public float MassKg;
        public bool StaticFrictionConfigured;
        public float StaticFriction;
        public bool DynamicFrictionConfigured;
        public float DynamicFriction;
        public bool BouncinessConfigured;
        public float Bounciness;
        public bool MinimumThicknessConfigured;
        public float MinimumThicknessMeters;
        public string MaterialTag;
        public string Notes;

        public string ToCompactText()
        {
            return
                "loaded:" + WhitelistLoaded +
                ",matched:" + Matched +
                ",source:" + Clean(RuleSource) +
                ",selector:" + Clean(MatchedInstanceSelector) +
                ",enabled:" + BoolValue(EnabledConfigured, Enabled) +
                ",proxyType:" + (ProxyTypeConfigured ? Clean(ProxyType) : "<unset>") +
                ",fixedToWorld:" + BoolValue(FixedToWorldConfigured, FixedToWorld) +
                ",pickable:" + BoolValue(PickableConfigured, Pickable) +
                ",massKg:" + FloatValue(MassConfigured, MassKg) +
                ",staticFriction:" + FloatValue(StaticFrictionConfigured, StaticFriction) +
                ",dynamicFriction:" + FloatValue(DynamicFrictionConfigured, DynamicFriction) +
                ",bounciness:" + FloatValue(BouncinessConfigured, Bounciness) +
                ",minThickness:" + FloatValue(MinimumThicknessConfigured, MinimumThicknessMeters) +
                ",materialTag:" + Clean(MaterialTag) +
                ",notes:" + Clean(Notes);
        }

        private static string BoolValue(bool configured, bool value)
        {
            return configured ? value.ToString() : "<unset>";
        }

        private static string FloatValue(bool configured, float value)
        {
            return configured ? value.ToString("G9") : "<unset>";
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
