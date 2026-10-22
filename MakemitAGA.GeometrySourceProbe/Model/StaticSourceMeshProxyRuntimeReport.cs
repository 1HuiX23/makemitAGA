using System;
using System.Text;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 最近一次成功多部件审计的碰撞聚合几何，被实际创建为静态非凸 MeshCollider 后的状态。
    /// 该对象始终由 GSP 独立拥有；不修改源 Mesh、Renderer、Collider 或 Transform。
    /// </summary>
    internal sealed class StaticSourceMeshProxyRuntimeReport
    {
        public bool Started;
        public bool Completed;
        public bool Success;
        public string TargetName;
        public string TargetPath;
        public string RootName;
        public string MeshName;
        public bool ProfileMatched;
        public string ProfileProxyType;
        public bool FixedToWorld;
        public bool Pickable;
        public int CollisionInstances;
        public int VertexCount;
        public int IndexCount;
        public int TriangleCount;
        public string IndexFormat;
        public int OutOfRangeIndices;
        public int NonFiniteVertices;
        public int RepeatedIndexTriangles;
        public int ZeroAreaTriangles;
        public int RawBoundaryEdges;
        public int WeldedBoundaryEdges;
        public int WeldedComponents;
        public int NonManifoldEdges;
        public int OrientationConflicts;
        public int DuplicateTriangles;
        public bool MeshCreated;
        public bool MeshFilterCreated;
        public bool RendererCreated;
        public bool ColliderCreated;
        public bool SharedMeshAssigned;
        public bool Convex;
        public bool IsTrigger;
        public bool RigidbodyPresent;
        public bool RendererBoundsMatch;
        public bool ColliderBoundsMatch;
        public bool VisualVisible;
        public bool ColliderEnabled;
        public string GeometryBounds;
        public string RendererBounds;
        public string ColliderBounds;
        public string Detail;

        public string ToStatusText()
        {
            return
                "GSP static source mesh proxy" +
                " | completed=" + Completed +
                " | success=" + Success +
                " | target=" + Clean(TargetName) +
                " | profile=" + ProfileMatched + ":" + Clean(ProfileProxyType) +
                " | fixed=" + FixedToWorld +
                " | pickable=" + Pickable +
                " | instances=" + CollisionInstances +
                " | geometry=v:" + VertexCount +
                ",i:" + IndexCount +
                ",t:" + TriangleCount +
                ",indexFormat:" + Clean(IndexFormat) +
                " | validation=outOfRange:" + OutOfRangeIndices +
                ",nonFinite:" + NonFiniteVertices +
                ",repeated:" + RepeatedIndexTriangles +
                ",zeroArea:" + ZeroAreaTriangles +
                ",nonManifold:" + NonManifoldEdges +
                ",orientationConflict:" + OrientationConflicts +
                ",duplicates:" + DuplicateTriangles +
                " | created=mesh:" + MeshCreated +
                ",filter:" + MeshFilterCreated +
                ",renderer:" + RendererCreated +
                ",collider:" + ColliderCreated +
                ",sharedMesh:" + SharedMeshAssigned +
                " | physics=convex:" + Convex +
                ",trigger:" + IsTrigger +
                ",rigidbody:" + RigidbodyPresent +
                " | bounds=rendererMatch:" + RendererBoundsMatch +
                ",colliderMatch:" + ColliderBoundsMatch +
                " | visual=" + VisualVisible +
                " | colliderEnabled=" + ColliderEnabled +
                " | root=" + Clean(RootName) +
                " | detail=" + Clean(Detail);
        }

        public string ToDetailedText()
        {
            StringBuilder sb = new StringBuilder(ToStatusText());
            sb.Append("\nrawBoundary=").Append(RawBoundaryEdges)
              .Append(" | weldedBoundary=").Append(WeldedBoundaryEdges)
              .Append(" | weldedComponents=").Append(WeldedComponents)
              .Append("\ngeometryBounds=").Append(Clean(GeometryBounds))
              .Append("\nrendererBounds=").Append(Clean(RendererBounds))
              .Append("\ncolliderBounds=").Append(Clean(ColliderBounds))
              .Append("\nmesh=").Append(Clean(MeshName));
            return sb.ToString();
        }

        private static string Clean(string value)
        {
            if (string.IsNullOrEmpty(value))
                return "<none>";
            return value.Replace("\r", " ").Replace("\n", " ").Replace("|", "/");
        }
    }
}
