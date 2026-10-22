using System;
using System.Collections.Generic;
using System.Text;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    internal sealed class HouseGeometryCensusObjectReport
    {
        public int Sequence;
        public string Room;
        public string ObjectName;
        public string ObjectPath;
        public bool ActiveSelf;
        public bool ActiveInHierarchy;
        public int DirectChildren;
        public int MeshRenderers;
        public int CandidateMeshRenderers;
        public int RoutedSubMeshes;
        public int SkinnedRenderers;
        public int Colliders;
        public int EnabledColliders;
        public int TriggerColliders;
        public int Rigidbodies;
        public string Bounds;
        public float MaximumDimension;
        public float MinimumDimension;
        public float ThinAxisRatio;
        public bool CurrentProbeFullCoverage;
        public string Coverage;
        public bool ReadbackAttempted;
        public bool ReadbackSuccess;
        public string ReadbackDetail;
        public string MeshName;
        public string RendererPath;
        public int Vertices;
        public int Indices;
        public int Triangles;
        public int InvalidTriangles;
        public int RepeatedIndexTriangles;
        public int ZeroAreaTriangles;
        public int BoundaryEdges;
        public int NonManifoldEdges;
        public int OrientationConflicts;
        public int ConnectedComponents;
        public int DuplicateTriangles;
        public bool Watertight;
        public bool TopologyAnalyzed;
        public bool LikelyPickable;
        public string PickableReason;
        public string Classification;
        public string RecommendedAction;

        public string ToLogLine()
        {
            return
                "seq=" + Sequence +
                " | room=" + Room +
                " | object=" + ObjectName +
                " | path=" + ObjectPath +
                " | active=" + ActiveInHierarchy +
                " | directChildren=" + DirectChildren +
                " | renderers=mesh:" + MeshRenderers +
                ",candidate:" + CandidateMeshRenderers +
                ",routedSubMeshes:" + RoutedSubMeshes +
                ",skinned:" + SkinnedRenderers +
                " | physics=colliders:" + Colliders +
                ",enabled:" + EnabledColliders +
                ",triggers:" + TriggerColliders +
                ",rigidbodies:" + Rigidbodies +
                " | bounds=" + Bounds +
                " | dimensions=max:" + MaximumDimension.ToString("F4") +
                ",min:" + MinimumDimension.ToString("F4") +
                ",thinRatio:" + ThinAxisRatio.ToString("F6") +
                " | coverage=" + Coverage +
                " | readback=attempted:" + ReadbackAttempted +
                ",success:" + ReadbackSuccess +
                ",detail:" + Clean(ReadbackDetail) +
                " | mesh=" + Clean(MeshName) +
                " | geometry=v:" + Vertices +
                ",i:" + Indices +
                ",t:" + Triangles +
                ",invalid:" + InvalidTriangles +
                ",repeated:" + RepeatedIndexTriangles +
                ",zeroArea:" + ZeroAreaTriangles +
                " | topology=analyzed:" + TopologyAnalyzed +
                ",boundary:" + BoundaryEdges +
                ",nonManifold:" + NonManifoldEdges +
                ",orientationConflict:" + OrientationConflicts +
                ",components:" + ConnectedComponents +
                ",duplicateTriangles:" + DuplicateTriangles +
                ",watertight:" + Watertight +
                " | pickable=" + LikelyPickable +
                ",reason:" + Clean(PickableReason) +
                " | class=" + Classification +
                " | action=" + RecommendedAction;
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }

    internal sealed class HouseGeometryCensusReport
    {
        public bool Started;
        public bool Running;
        public bool Completed;
        public bool Cancelled;
        public string SceneName;
        public string HousePath;
        public string RoomFilter;
        public int RoomsFound;
        public int ObjectsQueued;
        public int ObjectsProcessed;
        public int ReadbackAttempted;
        public int ReadbackSucceeded;
        public int FullCoverageObjects;
        public int WatertightObjects;
        public int OpenObjects;
        public int NonManifoldObjects;
        public int MultiPartObjects;
        public int SkinnedOnlyObjects;
        public int MetadataOnlyObjects;
        public int PickableCandidates;
        public int PickableReady;
        public int PickableRepairRequired;
        public int PickableAggregationRequired;
        public float ElapsedSeconds;
        public string LastObject;
        public string Detail;
        public readonly List<HouseGeometryCensusObjectReport> Objects =
            new List<HouseGeometryCensusObjectReport>();

        public string ToStatusText()
        {
            return
                "GSP house census | running=" + Running +
                " | completed=" + Completed +
                " | cancelled=" + Cancelled +
                " | scene=" + SceneName +
                " | house=" + HousePath +
                " | rooms=" + RoomsFound +
                " | progress=" + ObjectsProcessed + "/" + ObjectsQueued +
                " | readback=" + ReadbackSucceeded + "/" + ReadbackAttempted +
                " | last=" + (string.IsNullOrEmpty(LastObject) ? "<none>" : LastObject) +
                " | elapsed=" + ElapsedSeconds.ToString("F2") + "s";
        }

        public string ToSummaryText()
        {
            StringBuilder sb = new StringBuilder();
            sb.Append("GSP 第一章四房间几何普查");
            sb.Append(" | completed=").Append(Completed);
            sb.Append(" | cancelled=").Append(Cancelled);
            sb.Append(" | scene=").Append(SceneName);
            sb.Append(" | house=").Append(HousePath);
            sb.Append(" | roomFilter=").Append(RoomFilter);
            sb.Append(" | rooms=").Append(RoomsFound);
            sb.Append(" | objects=").Append(ObjectsProcessed).Append('/').Append(ObjectsQueued);
            sb.Append(" | readback=").Append(ReadbackSucceeded).Append('/').Append(ReadbackAttempted);
            sb.Append(" | fullCoverage=").Append(FullCoverageObjects);
            sb.Append(" | watertight=").Append(WatertightObjects);
            sb.Append(" | open=").Append(OpenObjects);
            sb.Append(" | nonManifold=").Append(NonManifoldObjects);
            sb.Append(" | multipart=").Append(MultiPartObjects);
            sb.Append(" | skinnedOnly=").Append(SkinnedOnlyObjects);
            sb.Append(" | metadataOnly=").Append(MetadataOnlyObjects);
            sb.Append(" | pickable=candidates:").Append(PickableCandidates)
              .Append(",ready:").Append(PickableReady)
              .Append(",repair:").Append(PickableRepairRequired)
              .Append(",aggregation:").Append(PickableAggregationRequired);
            sb.Append(" | elapsed=").Append(ElapsedSeconds.ToString("F2")).Append('s');
            sb.Append(" | detail=").Append(Detail ?? "<none>");

            AppendCandidateList(sb, "pickable-ready", "PICKABLE_GEOMETRY_READY");
            AppendCandidateList(sb, "pickable-repair", "PICKABLE_REPAIR_REQUIRED");
            AppendCandidateList(sb, "pickable-aggregation", "PICKABLE_AGGREGATION_REQUIRED");
            return sb.ToString();
        }

        private void AppendCandidateList(StringBuilder sb, string label, string classification)
        {
            int written = 0;
            for (int i = 0; i < Objects.Count; i++)
            {
                HouseGeometryCensusObjectReport item = Objects[i];
                if (item == null || !string.Equals(item.Classification, classification, StringComparison.Ordinal))
                    continue;
                if (written == 0)
                    sb.Append("\n").Append(label).Append('=');
                else
                    sb.Append("; ");
                sb.Append(item.Room).Append('/').Append(item.ObjectName)
                  .Append("[").Append(item.RecommendedAction).Append(']');
                written++;
                if (written >= 40)
                {
                    sb.Append("; ...");
                    break;
                }
            }
            if (written == 0)
                sb.Append("\n").Append(label).Append("=<none>");
        }
    }
}
