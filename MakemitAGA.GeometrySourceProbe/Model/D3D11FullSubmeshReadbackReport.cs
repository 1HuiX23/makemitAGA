using System;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// v0.1.17：Renderer 路由子网格的完整 D3D11 staging 读回与统计验证报告。
    /// 只保存统计与摘要，不在日志中输出完整 payload。
    /// </summary>
    public sealed class D3D11FullSubmeshReadbackReport
    {
        public bool Completed;
        public bool Success;
        public string TargetName;
        public string TargetPath;
        public string RendererPath;
        public bool IsPartOfStaticBatch;
        public string MeshName;
        public int RoutedSubMeshIndex;
        public int RoutedSubMeshCount;
        public int IndexStart;
        public int IndexCount;
        public int BaseVertex;
        public int FirstVertex;
        public int VertexCount;
        public string GraphicsDeviceType;
        public string GraphicsDeviceName;
        public string GraphicsDeviceVersion;

        public int IndexBufferCount;
        public int IndexBufferStride;
        public string IndexBufferTarget;
        public bool IndexBufferValid;
        public uint IndexSourceByteWidth;
        public IntPtr IndexSourcePointer;
        public int IndexRequestedByteOffset;
        public int IndexRequestedByteCount;
        public int IndexMapHResult;
        public bool IndexMapSucceeded;
        public bool IndexUnmapCalled;
        public bool IndexPayloadAllZero;
        public string IndexPayloadSample;
        public string IndexPayloadHash;
        public int DecodedIndexCount;
        public int EffectiveIndexMinimum;
        public int EffectiveIndexMaximum;
        public int LocalIndexMinimum;
        public int LocalIndexMaximum;
        public int OutOfRangeIndexCount;
        public int UniqueReferencedVertexCount;
        public int UnusedVertexCount;

        public int VertexBufferCount;
        public int VertexBufferStride;
        public string VertexBufferTarget;
        public bool VertexBufferValid;
        public uint VertexSourceByteWidth;
        public IntPtr VertexSourcePointer;
        public string PositionFormat;
        public int PositionDimension;
        public int PositionStream;
        public int PositionOffset;
        public int VertexRequestedByteOffset;
        public int VertexRequestedByteCount;
        public int VertexMapHResult;
        public bool VertexMapSucceeded;
        public bool VertexUnmapCalled;
        public bool VertexPayloadAllZero;
        public string VertexPayloadSample;
        public string VertexPayloadHash;
        public int DecodedPositionCount;
        public int NonFinitePositionCount;
        public int ZeroPositionCount;
        public int RawOutsideRendererBoundsCount;
        public int TransformedOutsideRendererBoundsCount;
        public int ReferencedRawOutsideRendererBoundsCount;
        public int ReferencedTransformedOutsideRendererBoundsCount;
        public string RawDecodedBounds;
        public string TransformedDecodedBounds;
        public string RendererBounds;
        public bool RawCoordinatesMatchRendererBounds;
        public bool TransformedCoordinatesMatchRendererBounds;
        public string CoordinateMode;

        public int TriangleCount;
        public int TrailingIndexCount;
        public int InvalidTriangleIndexCount;
        public int RepeatedIndexTriangleCount;
        public int ZeroAreaTriangleCount;
        public float MinimumTriangleArea;
        public float MaximumTriangleArea;
        public double AverageTriangleArea;

        public uint StagingByteWidth;
        public IntPtr DevicePointer;
        public IntPtr ImmediateContextPointer;
        public IntPtr StagingBufferPointer;
        public IntPtr MultithreadInterfacePointer;
        public int MultithreadQueryHResult;
        public bool MultithreadProtectedBefore;
        public bool MultithreadProtectionEnabledByGsp;
        public bool MultithreadProtectedDuring;
        public bool MultithreadEnterCalled;
        public bool MultithreadLeaveCalled;
        public bool MultithreadProtectionRestored;
        public int CreateBufferHResult;
        public bool IndexCopyIssued;
        public bool VertexCopyIssued;
        public uint MultithreadReleaseCount;
        public uint StagingReleaseCount;
        public uint ContextReleaseCount;
        public uint DeviceReleaseCount;
        public bool MultithreadReleased;
        public bool StagingReleased;
        public bool ContextReleased;
        public bool DeviceReleased;

        public string VertexTargetBefore;
        public string VertexTargetAfter;
        public string IndexTargetBefore;
        public string IndexTargetAfter;
        public bool TargetsUnchanged;
        public float ElapsedMilliseconds;
        public string Detail;

        public string ToText()
        {
            return
                "GSP D3D11 full submesh" +
                " | completed=" + Completed +
                " | success=" + Success +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | rendererPath=" + (RendererPath ?? "<none>") +
                " | staticBatch=" + IsPartOfStaticBatch +
                " | mesh=" + (MeshName ?? "<none>") +
                " | subMesh=" + RoutedSubMeshIndex +
                " | routedSubMeshCount=" + RoutedSubMeshCount +
                " | indexRange=" + IndexStart + "+" + IndexCount +
                " | vertexRange=" + FirstVertex + "+" + VertexCount +
                " | baseVertex=" + BaseVertex +
                " | deviceType=" + (GraphicsDeviceType ?? "<unknown>") +
                " | indexBuffer=" + IndexBufferCount + "x" + IndexBufferStride +
                    "/" + (IndexBufferTarget ?? "<unknown>") +
                    "/valid=" + IndexBufferValid +
                " | indexSource=" + FormatPointer(IndexSourcePointer) +
                    "/bytes=" + IndexSourceByteWidth +
                " | indexRead=" + IndexRequestedByteOffset + "+" + IndexRequestedByteCount +
                    "/map=" + FormatHResult(IndexMapHResult) +
                    "/zero=" + IndexPayloadAllZero +
                    "/hash=" + (IndexPayloadHash ?? "<none>") +
                    "/sample=" + (IndexPayloadSample ?? "<none>") +
                " | indices=" + DecodedIndexCount +
                    "/effectiveMinMax=" + EffectiveIndexMinimum + "," + EffectiveIndexMaximum +
                    "/localMinMax=" + LocalIndexMinimum + "," + LocalIndexMaximum +
                    "/outOfRange=" + OutOfRangeIndexCount +
                    "/uniqueUsed=" + UniqueReferencedVertexCount +
                    "/unused=" + UnusedVertexCount +
                " | vertexBuffer=" + VertexBufferCount + "x" + VertexBufferStride +
                    "/" + (VertexBufferTarget ?? "<unknown>") +
                    "/valid=" + VertexBufferValid +
                " | vertexSource=" + FormatPointer(VertexSourcePointer) +
                    "/bytes=" + VertexSourceByteWidth +
                " | position=" + (PositionFormat ?? "<none>") + "x" + PositionDimension +
                    "/stream" + PositionStream + "/offset" + PositionOffset +
                " | vertexRead=" + VertexRequestedByteOffset + "+" + VertexRequestedByteCount +
                    "/map=" + FormatHResult(VertexMapHResult) +
                    "/zero=" + VertexPayloadAllZero +
                    "/hash=" + (VertexPayloadHash ?? "<none>") +
                    "/sample=" + (VertexPayloadSample ?? "<none>") +
                " | positions=" + DecodedPositionCount +
                    "/nonFinite=" + NonFinitePositionCount +
                    "/zero=" + ZeroPositionCount +
                    "/rawOutside=" + RawOutsideRendererBoundsCount +
                    "/transformedOutside=" + TransformedOutsideRendererBoundsCount +
                    "/refRawOutside=" + ReferencedRawOutsideRendererBoundsCount +
                    "/refTransformedOutside=" + ReferencedTransformedOutsideRendererBoundsCount +
                    "/mode=" + (CoordinateMode ?? "<unknown>") +
                " | rawBounds=" + (RawDecodedBounds ?? "<none>") +
                " | transformedBounds=" + (TransformedDecodedBounds ?? "<none>") +
                " | rendererBounds=" + (RendererBounds ?? "<none>") +
                " | triangles=" + TriangleCount +
                    "/trailing=" + TrailingIndexCount +
                    "/invalid=" + InvalidTriangleIndexCount +
                    "/repeated=" + RepeatedIndexTriangleCount +
                    "/zeroArea=" + ZeroAreaTriangleCount +
                    "/areaMin=" + MinimumTriangleArea.ToString("G9") +
                    "/areaMax=" + MaximumTriangleArea.ToString("G9") +
                    "/areaAvg=" + AverageTriangleArea.ToString("G9") +
                " | staging=" + FormatPointer(StagingBufferPointer) +
                    "/bytes=" + StagingByteWidth +
                    "/create=" + FormatHResult(CreateBufferHResult) +
                    "/indexCopy=" + IndexCopyIssued +
                    "/vertexCopy=" + VertexCopyIssued +
                " | mt=" + FormatPointer(MultithreadInterfacePointer) +
                    "/qi=" + FormatHResult(MultithreadQueryHResult) +
                    "/before=" + MultithreadProtectedBefore +
                    "/during=" + MultithreadProtectedDuring +
                    "/enter=" + MultithreadEnterCalled +
                    "/leave=" + MultithreadLeaveCalled +
                    "/restored=" + MultithreadProtectionRestored +
                " | released=multithread:" + MultithreadReleased + "(" + MultithreadReleaseCount + ")" +
                    ",staging:" + StagingReleased + "(" + StagingReleaseCount + ")" +
                    ",context:" + ContextReleased + "(" + ContextReleaseCount + ")" +
                    ",device:" + DeviceReleased + "(" + DeviceReleaseCount + ")" +
                " | vertexTarget=" + (VertexTargetBefore ?? "<none>") +
                    "->" + (VertexTargetAfter ?? "<none>") +
                " | indexTarget=" + (IndexTargetBefore ?? "<none>") +
                    "->" + (IndexTargetAfter ?? "<none>") +
                " | unchanged=" + TargetsUnchanged +
                " | elapsed=" + ElapsedMilliseconds.ToString("F1") + "ms" +
                " | detail=" + (Detail ?? "<none>");
        }

        private static string FormatPointer(IntPtr pointer)
        {
            return pointer == IntPtr.Zero
                ? "0x0"
                : "0x" + pointer.ToInt64().ToString("X");
        }

        private static string FormatHResult(int hresult)
        {
            return "0x" + unchecked((uint)hresult).ToString("X8") +
                   "(" + hresult + ")";
        }
    }
}
