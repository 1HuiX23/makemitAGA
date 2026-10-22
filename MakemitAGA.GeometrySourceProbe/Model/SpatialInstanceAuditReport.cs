using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 空间焊接后的一块三角形连通组件。
    /// 组件只存在于托管审计数据中，不会创建或修改 Unity Mesh。
    /// </summary>
    internal sealed class SpatialComponentReport
    {
        public int ComponentIndex;
        public string ComponentKey;
        public string Channel;
        public int Triangles;
        public int CanonicalVertices;
        public int BoundaryEdges;
        public int BoundaryLoops;
        public int OpenBoundaryChains;
        public int NonManifoldEdges;
        public int OrientationConflicts;
        public bool Watertight;
        public float SurfaceArea;
        public float MaxDimension;
        public float MinDimension;
        public float ThinRatio;
        public string Bounds;
        public Vector3 Center;
        public Vector3 Size;
        public string SourceRouteIds;
        public string SourceRenderers;
        public string SourceRoles;
        public string Classification;
        public bool NeedsUnityExplorerReview;
        public string ReviewReason;

        public string ToLogLine()
        {
            return
                "channel=" + Clean(Channel) +
                " | component=" + ComponentIndex +
                " | key=" + Clean(ComponentKey) +
                " | geometry=t:" + Triangles +
                ",weldedV:" + CanonicalVertices +
                ",area:" + SurfaceArea.ToString("G9") +
                " | topology=boundary:" + BoundaryEdges +
                ",loops:" + BoundaryLoops +
                ",openChains:" + OpenBoundaryChains +
                ",nonManifold:" + NonManifoldEdges +
                ",orientationConflict:" + OrientationConflicts +
                ",watertight:" + Watertight +
                " | dimensions=max:" + MaxDimension.ToString("G9") +
                ",min:" + MinDimension.ToString("G9") +
                ",thinRatio:" + ThinRatio.ToString("G9") +
                " | bounds=" + Clean(Bounds) +
                " | sources=routes:" + Clean(SourceRouteIds) +
                ",renderers:" + Clean(SourceRenderers) +
                ",roles:" + Clean(SourceRoles) +
                " | class=" + Clean(Classification) +
                " | unityExplorerReview=" + NeedsUnityExplorerReview +
                ",reason:" + Clean(ReviewReason);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    /// <summary>
    /// 复合盒代理中的一个已审计子盒草案。v0.1.25 可由显式运行时命令创建实际静态 BoxCollider。
    /// </summary>
    internal sealed class CompoundBoxPartReport
    {
        public int PartIndex;
        public string Kind;
        public string SourceComponentIds;
        public Vector3 Center;
        public Vector3 Size;
        public Vector3 EulerDegrees;
        public string Reason;

        public string ToLogLine()
        {
            return
                "part=" + PartIndex +
                " | kind=" + Clean(Kind) +
                " | sourceComponents=" + Clean(SourceComponentIds) +
                " | center=" + Center.ToString("G9") +
                " | size=" + Size.ToString("G9") +
                " | euler=" + EulerDegrees.ToString("G9") +
                " | reason=" + Clean(Reason);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    /// <summary>
    /// 若干彼此接近或互相覆盖的连通组件被归并成一个“现实空间实例候选”。
    /// 例如数字钟外壳、按钮和显示面可以属于同一个实例；
    /// Unity 层级中分散在房间各处的多幅图片则应拆成多个实例。
    /// </summary>
    internal sealed class SpatialInstanceReport
    {
        public int InstanceIndex;
        public string InstanceKey;
        public string Channel;
        public int Components;
        public string ComponentIds;
        public int Triangles;
        public int CanonicalVertices;
        public int WatertightComponents;
        public int OpenComponents;
        public float SurfaceArea;
        public float MaxDimension;
        public float MinDimension;
        public float ThinRatio;
        public float MaximumAcceptedLinkGap;
        public string Bounds;
        public Vector3 Center;
        public Vector3 Size;
        public string SourceRouteIds;
        public string SourceRenderers;
        public string SourceRoles;
        public bool ContainsPhysicalRoute;
        public bool ContainsVisualOnlyRoute;
        public string AssociatedCollisionInstanceKey;
        public float CollisionAssociationGap;
        public string AssociationStatus;
        public string RecommendedRole;

        // 只读代理草案：不会创建 BoxCollider。
        public bool ProxyDraftBuilt;
        public string ProxyDraftType;
        public string ProxyFitMethod;
        public Vector3 ProxyCenter;
        public Vector3 ProxyRawSize;
        public Vector3 ProxySize;
        public Vector3 ProxyEulerDegrees;
        public float ProxyAabbVolume;
        public float ProxyRawVolume;
        public float ProxyInflatedVolume;
        public float ProxyVolumeRatioToAabb;
        public float ProxyThicknessInflation;
        public bool ProxyNeedsReview;
        public string ProxyReviewReason;
        public SpatialInstancePhysicsProfileSnapshot PhysicsProfile;

        public bool CompoundProxyDraftBuilt;
        public string CompoundProxyFitMethod;
        public int CompoundProxyParts;
        public bool CompoundProxyNeedsReview;
        public string CompoundProxyReviewReason;
        public readonly List<CompoundBoxPartReport> CompoundParts =
            new List<CompoundBoxPartReport>();

        public bool NeedsUnityExplorerReview;
        public string ReviewReason;

        public string ToLogLine()
        {
            return
                "channel=" + Clean(Channel) +
                " | instance=" + InstanceIndex +
                " | key=" + Clean(InstanceKey) +
                " | components=" + Components +
                ",ids:" + Clean(ComponentIds) +
                " | geometry=t:" + Triangles +
                ",weldedV:" + CanonicalVertices +
                ",area:" + SurfaceArea.ToString("G9") +
                " | topology=closedComponents:" + WatertightComponents +
                ",openComponents:" + OpenComponents +
                " | dimensions=max:" + MaxDimension.ToString("G9") +
                ",min:" + MinDimension.ToString("G9") +
                ",thinRatio:" + ThinRatio.ToString("G9") +
                ",maxAcceptedLinkGap:" + MaximumAcceptedLinkGap.ToString("G9") +
                " | bounds=" + Clean(Bounds) +
                " | sources=routes:" + Clean(SourceRouteIds) +
                ",renderers:" + Clean(SourceRenderers) +
                ",roles:" + Clean(SourceRoles) +
                ",hasPhysical:" + ContainsPhysicalRoute +
                ",hasVisualOnly:" + ContainsVisualOnlyRoute +
                " | association=status:" + Clean(AssociationStatus) +
                ",collisionKey:" + Clean(AssociatedCollisionInstanceKey) +
                ",gap:" + CollisionAssociationGap.ToString("G9") +
                " | recommendedRole=" + Clean(RecommendedRole) +
                " | proxy=built:" + ProxyDraftBuilt +
                ",type:" + Clean(ProxyDraftType) +
                ",fit:" + Clean(ProxyFitMethod) +
                ",center:" + ProxyCenter.ToString("G9") +
                ",rawSize:" + ProxyRawSize.ToString("G9") +
                ",size:" + ProxySize.ToString("G9") +
                ",euler:" + ProxyEulerDegrees.ToString("G9") +
                ",aabbVolume:" + ProxyAabbVolume.ToString("G9") +
                ",rawVolume:" + ProxyRawVolume.ToString("G9") +
                ",inflatedVolume:" + ProxyInflatedVolume.ToString("G9") +
                ",ratioToAabb:" + ProxyVolumeRatioToAabb.ToString("G9") +
                ",thicknessInflation:" + ProxyThicknessInflation.ToString("G9") +
                ",review:" + ProxyNeedsReview +
                ",reason:" + Clean(ProxyReviewReason) +
                " | compound=built:" + CompoundProxyDraftBuilt +
                ",parts:" + CompoundProxyParts +
                ",fit:" + Clean(CompoundProxyFitMethod) +
                ",review:" + CompoundProxyNeedsReview +
                ",reason:" + Clean(CompoundProxyReviewReason) +
                " | instanceProfile=" +
                (PhysicsProfile != null ? PhysicsProfile.ToCompactText() : "<none>") +
                " | unityExplorerReview=" + NeedsUnityExplorerReview +
                ",reason:" + Clean(ReviewReason);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    internal sealed class SpatialInstanceAuditReport
    {
        public string Channel;
        public bool Analyzed;
        public string Detail;
        public float WeldTolerance;
        public int InputVertices;
        public int InputTriangles;
        public int ValidTriangles;
        public int WeldedVertices;
        public int ConnectedComponents;
        public int SpatialInstances;
        public int UnityExplorerReviewInstances;
        public int AssociatedToCollisionInstances;
        public int UnassociatedObservationInstances;
        public int ProxyDrafts;
        public int MatchedInstanceProfiles;
        public readonly List<SpatialComponentReport> Components =
            new List<SpatialComponentReport>();
        public readonly List<SpatialInstanceReport> Instances =
            new List<SpatialInstanceReport>();

        public string ToCompactText(string label)
        {
            return label + "=analyzed:" + Analyzed +
                ",channel:" + Clean(Channel) +
                ",inputV:" + InputVertices +
                ",inputT:" + InputTriangles +
                ",validT:" + ValidTriangles +
                ",weldedV:" + WeldedVertices +
                ",components:" + ConnectedComponents +
                ",instances:" + SpatialInstances +
                ",ueReview:" + UnityExplorerReviewInstances +
                ",associatedCollision:" + AssociatedToCollisionInstances +
                ",unassociatedObservation:" + UnassociatedObservationInstances +
                ",proxyDrafts:" + ProxyDrafts +
                ",matchedProfiles:" + MatchedInstanceProfiles +
                ",weldTolerance:" + WeldTolerance.ToString("G9") +
                ",detail:" + Clean(Detail);
        }

        public void AppendInstanceText(StringBuilder sb, string prefix)
        {
            if (sb == null)
                return;
            for (int i = 0; i < Instances.Count; i++)
            {
                SpatialInstanceReport instance = Instances[i];
                sb.Append('\n').Append(prefix).Append("Instance[")
                  .Append(i + 1).Append("]=")
                  .Append(instance != null ? instance.ToLogLine() : "<null>");
            }
        }

        public void AppendCompoundText(StringBuilder sb, string prefix)
        {
            if (sb == null)
                return;
            for (int i = 0; i < Instances.Count; i++)
            {
                SpatialInstanceReport instance = Instances[i];
                if (instance == null)
                    continue;
                for (int c = 0; c < instance.CompoundParts.Count; c++)
                {
                    CompoundBoxPartReport part = instance.CompoundParts[c];
                    sb.Append('\n').Append(prefix).Append("CompoundPart[")
                      .Append(i + 1).Append('.').Append(c + 1).Append("]=")
                      .Append(part != null ? part.ToLogLine() : "<null>");
                }
            }
        }

        public void AppendComponentText(StringBuilder sb, string prefix)
        {
            if (sb == null)
                return;
            for (int i = 0; i < Components.Count; i++)
            {
                SpatialComponentReport component = Components[i];
                sb.Append('\n').Append(prefix).Append("Component[")
                  .Append(i + 1).Append("]=")
                  .Append(component != null ? component.ToLogLine() : "<null>");
            }
        }

        public void AppendDetailedText(StringBuilder sb, string prefix)
        {
            AppendInstanceText(sb, prefix);
            AppendCompoundText(sb, prefix);
            AppendComponentText(sb, prefix);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
