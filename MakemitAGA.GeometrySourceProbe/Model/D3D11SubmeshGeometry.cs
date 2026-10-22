using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 由 D3D11 staging 读回得到的单个 Renderer 路由子网格。
    /// WorldPositions 已统一转换为世界空间；LocalIndices 从 0 开始。
    /// 该对象只在读回完成后短暂传给可视化模块，不持有任何 COM 资源。
    /// </summary>
    internal sealed class D3D11SubmeshGeometry
    {
        public GameObject Target;
        public MeshRenderer SourceRenderer;
        public string TargetName;
        public string TargetPath;
        public string RendererPath;
        public string SourceMeshName;
        public int RoutedSubMeshIndex;
        public int FirstVertex;
        public int VertexCount;
        public int IndexCount;
        public string CoordinateMode;
        // RawPositions 是 GPU 中原始 Position 数据。静态合批对象通常已经是世界坐标，
        // 普通 MeshRenderer 通常仍是 Renderer 局部坐标。聚合器会再次独立审计后决定是否应用 localToWorld。
        public Vector3[] RawPositions;
        public Vector3[] WorldPositions;
        public int[] LocalIndices;
        public Bounds WorldBounds;
    }
}
