using UnityEngine;

namespace MakemitAGA.ActiveRagdollLab.Model
{
    /// <summary>
    /// 单腿目标骨解析结果。只保存源骨引用与只读几何数据。
    /// v0.2.0 解析器会额外记录候选根、解析细节与结构验证结果，便于在不同角色/场景下定位失败原因。
    /// </summary>
    internal sealed class SingleLegTargetReport
    {
        public bool Success;
        public string Error;
        public string RequestedSide;
        public string ResolutionMode;
        public string ResolutionDetail;
        public string CandidateRootPath;
        public string ChainValidation;
        public Transform UpperLeg;
        public Transform LowerLeg;
        public Transform Foot;
        // v0.3.29 Foot/Ankle foundation: Toe is an endpoint/reference only; source hierarchy remains read-only.
        public Transform Toe;
        public string UpperLegPath;
        public string LowerLegPath;
        public string FootPath;
        public string ToePath;
        public string ToeResolution;
        public bool FootFoundationReady;
        public float UpperLegLength;
        public float LowerLegLength;
        public float FootLength;
        public Vector3 KneeFlexionAxisWorld;
        public Vector3 KneeFlexionAxisLowerLocal;
    }
}
