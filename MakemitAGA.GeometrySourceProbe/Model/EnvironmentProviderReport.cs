using System;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// GSP v0.2.1 起保留的单对象统一固定环境提供者状态；v0.2.3 继续兼容。
    /// 该 DTO 只描述 GSP 自己拥有的 Unity Collider 及其源对象身份；
    /// 不包含、也不引用任何 ARL 类型或程序集。
    /// </summary>
    internal sealed class EnvironmentProviderReport
    {
        public bool Started;
        public bool Completed;
        public bool Success;
        public string Backend;
        public string TargetName;
        public string TargetPath;
        public int TargetPathMatches;
        public bool TargetIdentityValid;
        public int TargetLayer = -1;
        public int ProviderLayer = -1;
        public bool LayerIdentityValid;
        public int ProviderColliders;
        public bool ProviderCollidersEnabled;
        public int ProviderRigidbodies;
        public int SourceNativeColliders;
        public int SourceNativeEnabledColliders;
        public int SourceNativeRigidbodies;
        public string Detail;

        public string ToStatusText()
        {
            return
                "GSP single-object environment provider v0.2.3-compatible" +
                " | completed=" + Completed +
                " | success=" + Success +
                " | backend=" + Clean(Backend) +
                " | target=" + Clean(TargetName) +
                " | path=" + Clean(TargetPath) +
                " | binding=matches:" + TargetPathMatches +
                ",identity:" + TargetIdentityValid +
                " | layer=target:" + TargetLayer +
                ",provider:" + ProviderLayer +
                ",match:" + LayerIdentityValid +
                " | provider=colliders:" + ProviderColliders +
                ",enabled:" + ProviderCollidersEnabled +
                ",rigidbodies:" + ProviderRigidbodies +
                " | sourceNative=colliders:" + SourceNativeColliders +
                ",enabled:" + SourceNativeEnabledColliders +
                ",rigidbodies:" + SourceNativeRigidbodies +
                " | dllCoupling=NONE_UNITY_PHYSICS_ONLY" +
                " | detail=" + Clean(Detail);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
