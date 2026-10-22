using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// v0.1.18：D3D11 完整子网格生成独立 Unity Mesh 副本的结果。
    /// </summary>
    public sealed class D3D11MeshPreviewReport
    {
        public bool Completed;
        public bool Success;
        public string TargetName;
        public string TargetPath;
        public string RendererPath;
        public string SourceMeshName;
        public int RoutedSubMeshIndex;
        public string CoordinateMode;
        public int VertexCount;
        public int IndexCount;
        public int TriangleCount;
        public bool MeshCreated;
        public bool MaterialCreated;
        public bool MeshFilterCreated;
        public bool MeshRendererCreated;
        public bool ColliderCreated;
        public string PreviewObjectName;
        public Vector3 RootWorldPosition;
        public string SourceBounds;
        public string GeometryBounds;
        public string PreviewMeshLocalBounds;
        public string PreviewRendererBounds;
        public bool GeometryMatchesSourceBounds;
        public bool PreviewMatchesGeometryBounds;
        public bool PreviewMatchesSourceBounds;
        public string InitialMode;
        public string Detail;

        public string ToText()
        {
            return
                "GSP D3D11 mesh preview" +
                " | completed=" + Completed +
                " | success=" + Success +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | rendererPath=" + (RendererPath ?? "<none>") +
                " | sourceMesh=" + (SourceMeshName ?? "<none>") +
                " | subMesh=" + RoutedSubMeshIndex +
                " | coordinateMode=" + (CoordinateMode ?? "<unknown>") +
                " | vertices=" + VertexCount +
                " | indices=" + IndexCount +
                " | triangles=" + TriangleCount +
                " | created=mesh:" + MeshCreated +
                    ",material:" + MaterialCreated +
                    ",filter:" + MeshFilterCreated +
                    ",renderer:" + MeshRendererCreated +
                    ",collider:" + ColliderCreated +
                " | object=" + (PreviewObjectName ?? "<none>") +
                " | root=" + FormatVector(RootWorldPosition) +
                " | sourceBounds=" + (SourceBounds ?? "<none>") +
                " | geometryBounds=" + (GeometryBounds ?? "<none>") +
                " | meshLocalBounds=" + (PreviewMeshLocalBounds ?? "<none>") +
                " | previewBounds=" + (PreviewRendererBounds ?? "<none>") +
                " | matches=geometrySource:" + GeometryMatchesSourceBounds +
                    ",previewGeometry:" + PreviewMatchesGeometryBounds +
                    ",previewSource:" + PreviewMatchesSourceBounds +
                " | initialMode=" + (InitialMode ?? "<none>") +
                " | detail=" + (Detail ?? "<none>");
        }

        private static string FormatVector(Vector3 value)
        {
            return "(" +
                value.x.ToString("G9") + "," +
                value.y.ToString("G9") + "," +
                value.z.ToString("G9") + ")";
        }
    }
}
