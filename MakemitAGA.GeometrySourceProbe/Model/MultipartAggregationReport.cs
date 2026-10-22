using System;
using System.Collections.Generic;
using System.Text;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    internal sealed class GeometryTopologyAudit
    {
        public bool Analyzed;
        public int Vertices;
        public int Indices;
        public int Triangles;
        public int BoundaryEdges;
        public int BoundaryLoops;
        public int OpenBoundaryChains;
        public int NonManifoldEdges;
        public int OrientationConflicts;
        public int ConnectedComponents;
        public int DuplicateTriangles;
        public bool Watertight;
        public float WeldTolerance;
        public int WeldedVertices;

        public string ToCompactText(string label)
        {
            return label + "=analyzed:" + Analyzed +
                ",v:" + Vertices +
                ",i:" + Indices +
                ",t:" + Triangles +
                ",boundary:" + BoundaryEdges +
                ",loops:" + BoundaryLoops +
                ",openChains:" + OpenBoundaryChains +
                ",nonManifold:" + NonManifoldEdges +
                ",orientationConflict:" + OrientationConflicts +
                ",components:" + ConnectedComponents +
                ",duplicates:" + DuplicateTriangles +
                ",watertight:" + Watertight +
                ",weldTolerance:" + WeldTolerance.ToString("G9") +
                ",weldedVertices:" + WeldedVertices;
        }
    }

    internal sealed class ObjectPhysicsProfileSnapshot
    {
        public bool WhitelistLoaded;
        public string WhitelistPath;
        public bool ObjectRuleMatched;
        public string ObjectRuleSource;
        public bool MassConfigured;
        public float MassKg;
        public bool StaticFrictionConfigured;
        public float StaticFriction;
        public bool DynamicFrictionConfigured;
        public float DynamicFriction;
        public bool BouncinessConfigured;
        public float Bounciness;
        public string MaterialTag;
        public string Notes;

        public string ToCompactText()
        {
            return
                "loaded:" + WhitelistLoaded +
                ",matched:" + ObjectRuleMatched +
                ",source:" + Clean(ObjectRuleSource) +
                ",massKg:" + Value(MassConfigured, MassKg) +
                ",staticFriction:" + Value(StaticFrictionConfigured, StaticFriction) +
                ",dynamicFriction:" + Value(DynamicFrictionConfigured, DynamicFriction) +
                ",bounciness:" + Value(BouncinessConfigured, Bounciness) +
                ",materialTag:" + Clean(MaterialTag) +
                ",notes:" + Clean(Notes);
        }

        private static string Value(bool configured, float value)
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

    internal sealed class MultipartPartReport
    {
        public int Sequence;
        public string RendererName;
        public string RendererPath;
        public int RoutedSubMeshIndex;
        public string MaterialName;
        public string ShaderName;
        public bool RendererEnabled;
        public bool ActiveInHierarchy;
        public string ShadowCastingMode;
        public string AutoRole;
        public string ResolvedRole;
        public bool WhitelistMatched;
        public string WhitelistSource;
        public bool IncludeObservation;
        public bool IncludeCollision;
        public bool ReadbackAttempted;
        public bool ReadbackSuccess;
        public string ReadbackDetail;
        public int Vertices;
        public int Indices;
        public int Triangles;
        public int ZeroAreaTriangles;
        public string ReadbackCoordinateMode;
        public string ResolvedCoordinateMode;
        public bool TransformApplied;
        public bool CoordinateAmbiguous;
        public bool RawReferencedInsideRendererBounds;
        public bool TransformedReferencedInsideRendererBounds;
        public bool AggregationAccepted;
        public string RendererBounds;
        public string RawBounds;
        public string TransformedBounds;
        public string WorldBounds;
        public string CoordinateReason;
        public string Bounds;
        public string DecisionReason;

        public string ToLogLine()
        {
            return
                "seq=" + Sequence +
                " | renderer=" + Clean(RendererName) +
                " | rendererPath=" + Clean(RendererPath) +
                " | route=" + RoutedSubMeshIndex +
                " | material=" + Clean(MaterialName) +
                " | shader=" + Clean(ShaderName) +
                " | state=enabled:" + RendererEnabled +
                ",active:" + ActiveInHierarchy +
                ",shadowCasting:" + Clean(ShadowCastingMode) +
                " | role=auto:" + Clean(AutoRole) +
                ",resolved:" + Clean(ResolvedRole) +
                ",whitelist:" + WhitelistMatched +
                ",source:" + Clean(WhitelistSource) +
                " | include=observation:" + IncludeObservation +
                ",collision:" + IncludeCollision +
                " | readback=attempted:" + ReadbackAttempted +
                ",success:" + ReadbackSuccess +
                ",detail:" + Clean(ReadbackDetail) +
                " | geometry=v:" + Vertices +
                ",i:" + Indices +
                ",t:" + Triangles +
                ",zeroArea:" + ZeroAreaTriangles +
                " | coordinate=readback:" + Clean(ReadbackCoordinateMode) +
                ",resolved:" + Clean(ResolvedCoordinateMode) +
                ",transformApplied:" + TransformApplied +
                ",ambiguous:" + CoordinateAmbiguous +
                ",rawInside:" + RawReferencedInsideRendererBounds +
                ",transformedInside:" + TransformedReferencedInsideRendererBounds +
                ",accepted:" + AggregationAccepted +
                " | bounds=renderer:" + Clean(RendererBounds) +
                ",raw:" + Clean(RawBounds) +
                ",transformed:" + Clean(TransformedBounds) +
                ",world:" + Clean(WorldBounds) +
                " | coordinateReason=" + Clean(CoordinateReason) +
                " | reason=" + Clean(DecisionReason);
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    internal sealed class MultipartAggregationReport
    {
        public bool Started;
        public bool Running;
        public bool Completed;
        public bool Cancelled;
        public bool Success;
        public string SceneName;
        public string TargetName;
        public string TargetPath;
        public int ExactMatches;
        public string ReferenceName;
        public float SelectedDistance;
        public int MeshRenderers;
        public int SkinnedRenderers;
        public int RoutesQueued;
        public int RoutesProcessed;
        public int RoutesReadbackAttempted;
        public int RoutesReadbackSucceeded;
        public int PhysicalRoutes;
        public int VisualOnlyRoutes;
        public int ExcludedEffectRoutes;
        public int ExcludedChildObjectRoutes;
        public int UnknownRoutes;
        public int WhitelistOverrides;
        public int CoordinateAcceptedRoutes;
        public int CoordinateRejectedRoutes;
        public int CoordinateRawWorldRoutes;
        public int CoordinateTransformedRoutes;
        public int CoordinateAmbiguousRoutes;
        public int ObservationVertices;
        public int ObservationIndices;
        public int ObservationTriangles;
        public int CollisionVertices;
        public int CollisionIndices;
        public int CollisionTriangles;
        public string TargetRendererBounds;
        public string ObservationWorldBounds;
        public string CollisionWorldBounds;
        public float ElapsedSeconds;
        public string LastPart;
        public string Detail;
        public ObjectPhysicsProfileSnapshot PhysicsProfile;
        public GeometryTopologyAudit ObservationRawTopology;
        public GeometryTopologyAudit ObservationWeldedTopology;
        public GeometryTopologyAudit CollisionRawTopology;
        public GeometryTopologyAudit CollisionWeldedTopology;
        public SpatialInstanceAuditReport ObservationSpatialInstances;
        public SpatialInstanceAuditReport CollisionSpatialInstances;
        public readonly List<MultipartPartReport> Parts =
            new List<MultipartPartReport>();

        public string ToStatusText()
        {
            return
                "GSP multipart audit | running=" + Running +
                " | completed=" + Completed +
                " | cancelled=" + Cancelled +
                " | target=" + Clean(TargetName) +
                " | progress=" + RoutesProcessed + "/" + RoutesQueued +
                " | readback=" + RoutesReadbackSucceeded + "/" + RoutesReadbackAttempted +
                " | roles=physical:" + PhysicalRoutes +
                ",visual:" + VisualOnlyRoutes +
                ",effectExcluded:" + ExcludedEffectRoutes +
                ",childExcluded:" + ExcludedChildObjectRoutes +
                ",unknown:" + UnknownRoutes +
                " | coordinate=accepted:" + CoordinateAcceptedRoutes +
                ",rejected:" + CoordinateRejectedRoutes +
                ",rawWorld:" + CoordinateRawWorldRoutes +
                ",transformed:" + CoordinateTransformedRoutes +
                ",ambiguous:" + CoordinateAmbiguousRoutes +
                " | spatial=observationInstances:" +
                (ObservationSpatialInstances != null ? ObservationSpatialInstances.SpatialInstances : 0) +
                ",collisionInstances:" +
                (CollisionSpatialInstances != null ? CollisionSpatialInstances.SpatialInstances : 0) +
                " | last=" + Clean(LastPart) +
                " | elapsed=" + ElapsedSeconds.ToString("F2") + "s";
        }

        public string ToSummaryText()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("GSP 多部件聚合、空间实例与碰撞代理审计");
            sb.Append(" | completed=").Append(Completed);
            sb.Append(" | cancelled=").Append(Cancelled);
            sb.Append(" | success=").Append(Success);
            sb.Append(" | scene=").Append(Clean(SceneName));
            sb.Append(" | target=").Append(Clean(TargetName));
            sb.Append(" | path=").Append(Clean(TargetPath));
            sb.Append(" | matches=").Append(ExactMatches);
            sb.Append(" | renderers=mesh:").Append(MeshRenderers)
              .Append(",skinned:").Append(SkinnedRenderers);
            sb.Append(" | routes=processed:").Append(RoutesProcessed)
              .Append('/').Append(RoutesQueued)
              .Append(",readback:").Append(RoutesReadbackSucceeded)
              .Append('/').Append(RoutesReadbackAttempted);
            sb.Append(" | roles=physical:").Append(PhysicalRoutes)
              .Append(",visualOnly:").Append(VisualOnlyRoutes)
              .Append(",effectExcluded:").Append(ExcludedEffectRoutes)
              .Append(",childExcluded:").Append(ExcludedChildObjectRoutes)
              .Append(",unknown:").Append(UnknownRoutes)
              .Append(",whitelistOverrides:").Append(WhitelistOverrides);
            sb.Append(" | coordinate=accepted:").Append(CoordinateAcceptedRoutes)
              .Append(",rejected:").Append(CoordinateRejectedRoutes)
              .Append(",rawWorld:").Append(CoordinateRawWorldRoutes)
              .Append(",transformed:").Append(CoordinateTransformedRoutes)
              .Append(",ambiguous:").Append(CoordinateAmbiguousRoutes);
            sb.Append(" | observation=v:").Append(ObservationVertices)
              .Append(",i:").Append(ObservationIndices)
              .Append(",t:").Append(ObservationTriangles);
            sb.Append(" | collision=v:").Append(CollisionVertices)
              .Append(",i:").Append(CollisionIndices)
              .Append(",t:").Append(CollisionTriangles);
            sb.Append(" | worldBounds=targetRenderers:").Append(Clean(TargetRendererBounds))
              .Append(",observation:").Append(Clean(ObservationWorldBounds))
              .Append(",collision:").Append(Clean(CollisionWorldBounds));
            sb.Append(" | profile=").Append(
                PhysicsProfile != null ? PhysicsProfile.ToCompactText() : "<none>");
            if (ObservationRawTopology != null)
                sb.Append(" | ").Append(ObservationRawTopology.ToCompactText("observationRaw"));
            if (ObservationWeldedTopology != null)
                sb.Append(" | ").Append(ObservationWeldedTopology.ToCompactText("observationWelded"));
            if (CollisionRawTopology != null)
                sb.Append(" | ").Append(CollisionRawTopology.ToCompactText("collisionRaw"));
            if (CollisionWeldedTopology != null)
                sb.Append(" | ").Append(CollisionWeldedTopology.ToCompactText("collisionWelded"));
            if (ObservationSpatialInstances != null)
                sb.Append(" | ").Append(ObservationSpatialInstances.ToCompactText("observationSpatial"));
            if (CollisionSpatialInstances != null)
                sb.Append(" | ").Append(CollisionSpatialInstances.ToCompactText("collisionSpatial"));
            sb.Append(" | elapsed=").Append(ElapsedSeconds.ToString("F2")).Append('s');
            sb.Append(" | detail=").Append(Clean(Detail));

            for (int i = 0; i < Parts.Count; i++)
            {
                MultipartPartReport part = Parts[i];
                sb.Append("\npart[").Append(i + 1).Append("]=")
                  .Append(part != null ? part.ToLogLine() : "<null>");
            }
            if (CollisionSpatialInstances != null)
                CollisionSpatialInstances.AppendInstanceText(sb, "collisionSpatial");
            if (ObservationSpatialInstances != null)
                ObservationSpatialInstances.AppendInstanceText(sb, "observationSpatial");
            return sb.ToString();
        }

        public string ToFullText()
        {
            StringBuilder sb = new StringBuilder(ToSummaryText());
            if (CollisionSpatialInstances != null)
            {
                CollisionSpatialInstances.AppendCompoundText(sb, "collisionSpatial");
                CollisionSpatialInstances.AppendComponentText(sb, "collisionSpatial");
            }
            if (ObservationSpatialInstances != null)
            {
                if (SpatialChannelsMirrorEachOther())
                    sb.Append("\nobservationSpatialComponents=<mirrors collision channel; duplicate component lines omitted>");
                else
                {
                    ObservationSpatialInstances.AppendCompoundText(sb, "observationSpatial");
                    ObservationSpatialInstances.AppendComponentText(sb, "observationSpatial");
                }
            }
            return sb.ToString();
        }

        private bool SpatialChannelsMirrorEachOther()
        {
            SpatialInstanceAuditReport a = CollisionSpatialInstances;
            SpatialInstanceAuditReport b = ObservationSpatialInstances;
            if (a == null || b == null || !a.Analyzed || !b.Analyzed)
                return false;
            return a.InputVertices == b.InputVertices &&
                a.InputTriangles == b.InputTriangles &&
                a.ValidTriangles == b.ValidTriangles &&
                a.WeldedVertices == b.WeldedVertices &&
                a.ConnectedComponents == b.ConnectedComponents &&
                a.SpatialInstances == b.SpatialInstances;
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
