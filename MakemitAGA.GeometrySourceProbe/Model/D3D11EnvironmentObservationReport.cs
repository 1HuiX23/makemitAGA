namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// v0.1.22：面向未来 Active Ragdoll、ML-Agents 与 VLM 目标点控制的
    /// 环境几何观测契约基础报告。
    ///
    /// 本报告不引用 ML-Agents 程序集，也不决定最终神经网络向量长度；它只把
    /// 已验证的世界空间几何、表面查询能力和固定碰撞体能力整理成稳定 DTO，供后续
    /// Adapter 转换为 Observation、Ray/Surface Query 或 VLM 指定目标点约束。
    /// </summary>
    public sealed class D3D11EnvironmentObservationReport
    {
        public bool Completed;
        public bool Success;
        public string SchemaVersion;
        public string ContractStage;
        public string TargetName;
        public string TargetPath;
        public string CoordinateSpace;
        public string RootFrame;
        public bool GeometryReady;
        public bool ColliderReady;
        public int VertexCount;
        public int IndexCount;
        public int TriangleCount;
        public string BoundsCenter;
        public string BoundsSize;
        public string BoundsMin;
        public string BoundsMax;
        public string PointNormalization;
        public bool SupportsWorldGeometryCopy;
        public bool SupportsNearestTriangleSurfaceQuery;
        public bool SupportsSurfaceNormal;
        public bool SupportsContactPoint;
        public bool SupportsContactImpulse;
        public bool SupportsBoundsNormalization;
        public string IntendedConsumers;
        public string NotYetIncluded;
        public string Detail;

        public string ToText()
        {
            return
                "GSP D3D11 environment observation foundation" +
                " | completed=" + Completed +
                " | success=" + Success +
                " | schema=" + (SchemaVersion ?? "<none>") +
                " | stage=" + (ContractStage ?? "<none>") +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | frame=coordinate:" + (CoordinateSpace ?? "<none>") +
                    ",root:" + (RootFrame ?? "<none>") +
                " | geometry=ready:" + GeometryReady +
                    ",collider:" + ColliderReady +
                    ",vertices:" + VertexCount +
                    ",indices:" + IndexCount +
                    ",triangles:" + TriangleCount +
                " | bounds=center:" + (BoundsCenter ?? "<none>") +
                    ",size:" + (BoundsSize ?? "<none>") +
                    ",min:" + (BoundsMin ?? "<none>") +
                    ",max:" + (BoundsMax ?? "<none>") +
                " | normalization=" +
                    (PointNormalization ?? "<none>") +
                " | surfaceQueries=worldGeometryCopy:" +
                    SupportsWorldGeometryCopy +
                    ",nearestTriangle:" +
                    SupportsNearestTriangleSurfaceQuery +
                    ",normal:" + SupportsSurfaceNormal +
                    ",boundsNormalization:" +
                    SupportsBoundsNormalization +
                " | contactChannels=point:" + SupportsContactPoint +
                    ",impulse:" + SupportsContactImpulse +
                " | intendedConsumers=" +
                    (IntendedConsumers ?? "<none>") +
                " | notYetIncluded=" +
                    (NotYetIncluded ?? "<none>") +
                " | detail=" + (Detail ?? "<none>");
        }
    }
}
