using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using UnityEngine.Rendering;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// v0.1.17：完整读取 Renderer 路由子网格的索引区间和顶点区间，并做只读统计验证。
    /// 可选地把已验证的本地索引与世界空间 Position 作为纯托管几何交给可视化层；
    /// D3D11 读回层自身仍不创建 Unity Mesh。
    ///
    /// 安全边界：
    /// - 只支持 Direct3D11；
    /// - 源 Mesh / GraphicsBuffer / ID3D11Buffer 只读，不 AddRef、不 Release；
    /// - 只创建一个 GSP 独占的 STAGING + CPU_READ buffer；
    /// - 只复制路由子网格对应的 index bytes 和 vertex bytes；
    /// - Map 后复制到托管 byte[]，立即 Unmap；
    /// - 只 Release 由 GetDevice、GetImmediateContext、QueryInterface 或 CreateBuffer
    ///   返回并由 GSP 持有的 COM 引用；
    /// - 不创建 Unity Mesh，不修改 buffer target，不写源资源。
    /// </summary>
    internal static class D3D11FullSubmeshReadback
    {
        private const int IUnknownQueryInterfaceSlot = 0;
        private const int IUnknownReleaseSlot = 2;
        private const int DeviceChildGetDeviceSlot = 3;
        private const int BufferGetDescSlot = 10;
        private const int DeviceCreateBufferSlot = 3;
        private const int DeviceGetImmediateContextSlot = 40;
        private const int ContextMapSlot = 14;
        private const int ContextUnmapSlot = 15;
        private const int ContextCopySubresourceRegionSlot = 46;
        private const int MultithreadEnterSlot = 3;
        private const int MultithreadLeaveSlot = 4;
        private const int MultithreadSetProtectedSlot = 5;
        private const int MultithreadGetProtectedSlot = 6;

        private static readonly Guid IidId3D11Multithread =
            new Guid("9B7E4E00-342C-4106-A19F-4F2704F689F0");

        private const uint D3D11UsageDefault = 0;
        private const uint D3D11UsageStaging = 3;
        private const uint D3D11BindVertexBuffer = 0x1;
        private const uint D3D11BindIndexBuffer = 0x2;
        private const uint D3D11CpuAccessRead = 0x20000;
        private const uint D3D11MapRead = 1;
        private const float ZeroAreaThreshold = 1e-10f;

        [StructLayout(LayoutKind.Sequential)]
        private struct D3D11BufferDesc
        {
            public uint ByteWidth;
            public uint Usage;
            public uint BindFlags;
            public uint CPUAccessFlags;
            public uint MiscFlags;
            public uint StructureByteStride;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct D3D11Box
        {
            public uint Left;
            public uint Top;
            public uint Front;
            public uint Right;
            public uint Bottom;
            public uint Back;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct D3D11MappedSubresource
        {
            public IntPtr Data;
            public uint RowPitch;
            public uint DepthPitch;
        }

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate int QueryInterfaceDelegate(
            IntPtr self,
            ref Guid interfaceId,
            out IntPtr result);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate uint ReleaseDelegate(IntPtr self);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate void GetDeviceDelegate(IntPtr self, out IntPtr device);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate void GetDescDelegate(IntPtr self, out D3D11BufferDesc desc);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate int CreateBufferDelegate(
            IntPtr self,
            ref D3D11BufferDesc desc,
            IntPtr initialData,
            out IntPtr buffer);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate void GetImmediateContextDelegate(
            IntPtr self,
            out IntPtr context);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate void CopySubresourceRegionDelegate(
            IntPtr self,
            IntPtr destinationResource,
            uint destinationSubresource,
            uint destinationX,
            uint destinationY,
            uint destinationZ,
            IntPtr sourceResource,
            uint sourceSubresource,
            ref D3D11Box sourceBox);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate int MapDelegate(
            IntPtr self,
            IntPtr resource,
            uint subresource,
            uint mapType,
            uint mapFlags,
            out D3D11MappedSubresource mapped);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate void UnmapDelegate(
            IntPtr self,
            IntPtr resource,
            uint subresource);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate void MultithreadEnterDelegate(IntPtr self);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate void MultithreadLeaveDelegate(IntPtr self);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate int SetMultithreadProtectedDelegate(
            IntPtr self,
            int enabled);

        [UnmanagedFunctionPointer(CallingConvention.StdCall)]
        private delegate int GetMultithreadProtectedDelegate(IntPtr self);

        private sealed class RoutedCandidate
        {
            public MeshRenderer Renderer;
            public MeshFilter Filter;
            public Mesh Mesh;
            public int RoutedSubMeshIndex;
            public int RoutedSubMeshCount;
        }

        public static D3D11FullSubmeshReadbackReport Probe(
            GameObject target,
            ManualLogSource logger)
        {
            return Probe(target, logger, out _);
        }

        public static D3D11FullSubmeshReadbackReport Probe(
            GameObject target,
            ManualLogSource logger,
            out D3D11SubmeshGeometry geometry)
        {
            return ProbeInternal(
                target,
                logger,
                out geometry,
                null,
                -1);
        }

        /// <summary>
        /// 读取指定 MeshRenderer 的指定路由 SubMesh。
        /// 供多部件聚合审计逐路线调用；仍然只读，不修改 Mesh buffer target。
        /// </summary>
        public static D3D11FullSubmeshReadbackReport ProbeRoute(
            GameObject target,
            MeshRenderer renderer,
            int routedSubMeshIndex,
            ManualLogSource logger,
            out D3D11SubmeshGeometry geometry)
        {
            return ProbeInternal(
                target,
                logger,
                out geometry,
                renderer,
                routedSubMeshIndex);
        }

        private static D3D11FullSubmeshReadbackReport ProbeInternal(
            GameObject target,
            ManualLogSource logger,
            out D3D11SubmeshGeometry geometry,
            MeshRenderer forcedRenderer,
            int forcedRoutedSubMeshIndex)
        {
            geometry = null;
            Stopwatch stopwatch = Stopwatch.StartNew();
            D3D11FullSubmeshReadbackReport report =
                new D3D11FullSubmeshReadbackReport
                {
                    TargetName = target != null ? target.name : null,
                    TargetPath = TargetFinder.GetPath(
                        target != null ? target.transform : null),
                    MultithreadQueryHResult = unchecked((int)0x80004002),
                    CreateBufferHResult = unchecked((int)0x80004005),
                    IndexMapHResult = unchecked((int)0x80004005),
                    VertexMapHResult = unchecked((int)0x80004005),
                    EffectiveIndexMinimum = int.MaxValue,
                    EffectiveIndexMaximum = int.MinValue,
                    LocalIndexMinimum = int.MaxValue,
                    LocalIndexMaximum = int.MinValue,
                    MinimumTriangleArea = float.MaxValue
                };

            if (target == null)
            {
                report.Completed = false;
                report.Success = false;
                report.Detail = "target-null";
                return report;
            }

            RoutedCandidate candidate = forcedRenderer != null
                ? BuildSpecificRoutedCandidate(
                    forcedRenderer,
                    forcedRoutedSubMeshIndex)
                : FindFirstRoutedCandidate(target);
            if (candidate == null || candidate.Mesh == null)
            {
                report.Completed = true;
                report.Success = false;
                report.Detail =
                    "no MeshRenderer + MeshFilter/sharedMesh routed candidate";
                return report;
            }

            MeshRenderer renderer = candidate.Renderer;
            Mesh mesh = candidate.Mesh;
            report.RendererPath = TargetFinder.GetPath(renderer.transform);
            report.IsPartOfStaticBatch = Safe(
                () => renderer.isPartOfStaticBatch,
                false);
            report.MeshName = Safe(() => mesh.name, "<metadata-error>");
            report.RoutedSubMeshIndex = candidate.RoutedSubMeshIndex;
            report.RoutedSubMeshCount = candidate.RoutedSubMeshCount;
            report.VertexTargetBefore = Safe(
                () => mesh.vertexBufferTarget.ToString(),
                "<getter-error>");
            report.IndexTargetBefore = Safe(
                () => mesh.indexBufferTarget.ToString(),
                "<getter-error>");
            report.GraphicsDeviceType = Safe(
                () => SystemInfo.graphicsDeviceType.ToString(),
                "<getter-error>");
            report.GraphicsDeviceName = Safe(
                () => SystemInfo.graphicsDeviceName,
                "<getter-error>");
            report.GraphicsDeviceVersion = Safe(
                () => SystemInfo.graphicsDeviceVersion,
                "<getter-error>");

            GraphicsBuffer indexBuffer = null;
            GraphicsBuffer vertexBuffer = null;
            IntPtr indexSourcePointer = IntPtr.Zero;
            IntPtr vertexSourcePointer = IntPtr.Zero;
            IntPtr devicePointer = IntPtr.Zero;
            IntPtr contextPointer = IntPtr.Zero;
            IntPtr stagingPointer = IntPtr.Zero;
            IntPtr multithreadPointer = IntPtr.Zero;
            UnmapDelegate unmap = null;
            MultithreadLeaveDelegate multithreadLeave = null;
            SetMultithreadProtectedDelegate setMultithreadProtected = null;
            GetMultithreadProtectedDelegate getMultithreadProtected = null;
            bool multithreadEntered = false;
            bool multithreadProtectionChanged = false;

            try
            {
                logger?.LogWarning(
                    "[GSP] D3D11 full-submesh readback begin" +
                    " | target=" + report.TargetName +
                    " | path=" + report.TargetPath +
                    " | renderer=" + report.RendererPath +
                    " | staticBatch=" + report.IsPartOfStaticBatch +
                    " | mesh=" + report.MeshName +
                    " | routedSubMesh=" + report.RoutedSubMeshIndex +
                    " | deviceType=" + report.GraphicsDeviceType +
                    " | IMPORTANT=read-only full routed index/vertex ranges; no Unity Mesh creation");

                if (IntPtr.Size != 8)
                {
                    throw new NotSupportedException(
                        "process pointer size is " + (IntPtr.Size * 8) +
                        "-bit; this build requires x64");
                }

                if (!string.Equals(
                    report.GraphicsDeviceType,
                    GraphicsDeviceType.Direct3D11.ToString(),
                    StringComparison.Ordinal))
                {
                    throw new NotSupportedException(
                        "graphics device is " + report.GraphicsDeviceType +
                        "; this command only supports Direct3D11");
                }

                logger?.LogInfo(
                    "[GSP] D3D11 portability guard" +
                    " | pointerBits=" + (IntPtr.Size * 8) +
                    " | backend=" + report.GraphicsDeviceType +
                    " | addressMode=runtime-discovered" +
                    " | fixedNativeAddresses=False" +
                    " | bufferLayout=queried-from-Mesh-and-GetDesc");

                SubMeshDescriptor subMesh = mesh.GetSubMesh(
                    report.RoutedSubMeshIndex);
                report.IndexStart = subMesh.indexStart;
                report.IndexCount = subMesh.indexCount;
                report.BaseVertex = subMesh.baseVertex;
                report.FirstVertex = subMesh.firstVertex;
                report.VertexCount = subMesh.vertexCount;

                logger?.LogInfo(
                    "[GSP] Routed submesh descriptor" +
                    " | topology=" + subMesh.topology +
                    " | indexStart=" + subMesh.indexStart +
                    " | indexCount=" + subMesh.indexCount +
                    " | baseVertex=" + subMesh.baseVertex +
                    " | firstVertex=" + subMesh.firstVertex +
                    " | vertexCount=" + subMesh.vertexCount);

                if (subMesh.topology != MeshTopology.Triangles)
                    throw new NotSupportedException(
                        "full validator requires triangle topology; actual=" +
                        subMesh.topology);
                if (subMesh.indexCount <= 0 || subMesh.vertexCount <= 0)
                    throw new InvalidOperationException(
                        "routed submesh has no indices or vertices");

                VertexAttributeFormat positionFormat =
                    mesh.GetVertexAttributeFormat(VertexAttribute.Position);
                int positionDimension =
                    mesh.GetVertexAttributeDimension(VertexAttribute.Position);
                int positionStream =
                    mesh.GetVertexAttributeStream(VertexAttribute.Position);
                int positionOffset =
                    mesh.GetVertexAttributeOffset(VertexAttribute.Position);
                report.PositionFormat = positionFormat.ToString();
                report.PositionDimension = positionDimension;
                report.PositionStream = positionStream;
                report.PositionOffset = positionOffset;

                if (positionFormat != VertexAttributeFormat.Float32 ||
                    positionDimension != 3 ||
                    positionStream < 0 ||
                    positionOffset < 0)
                {
                    throw new NotSupportedException(
                        "full validator requires Position Float32x3; actual=" +
                        positionFormat + "x" + positionDimension +
                        "/stream" + positionStream +
                        "/offset" + positionOffset);
                }

                logger?.LogWarning(
                    "[GSP] Native boundary enter | call=Mesh.GetIndexBuffer" +
                    " | mesh=" + report.MeshName);
                indexBuffer = mesh.GetIndexBuffer();
                logger?.LogInfo(
                    "[GSP] Native boundary returned | call=Mesh.GetIndexBuffer");
                if (indexBuffer == null)
                    throw new InvalidOperationException(
                        "Unity returned null index GraphicsBuffer wrapper");

                report.IndexBufferCount = indexBuffer.count;
                report.IndexBufferStride = indexBuffer.stride;
                report.IndexBufferTarget = indexBuffer.target.ToString();
                report.IndexBufferValid = indexBuffer.IsValid();
                if (!report.IndexBufferValid)
                    throw new InvalidOperationException(
                        "index GraphicsBuffer.IsValid=false");
                if (report.IndexBufferStride != 2 &&
                    report.IndexBufferStride != 4)
                {
                    throw new NotSupportedException(
                        "index stride must be 2 or 4; actual=" +
                        report.IndexBufferStride);
                }

                logger?.LogWarning(
                    "[GSP] Native boundary enter | call=Mesh.GetVertexBuffer(" +
                    positionStream + ") | mesh=" + report.MeshName);
                vertexBuffer = mesh.GetVertexBuffer(positionStream);
                logger?.LogInfo(
                    "[GSP] Native boundary returned | call=Mesh.GetVertexBuffer(" +
                    positionStream + ")");
                if (vertexBuffer == null)
                    throw new InvalidOperationException(
                        "Unity returned null vertex GraphicsBuffer wrapper");

                report.VertexBufferCount = vertexBuffer.count;
                report.VertexBufferStride = vertexBuffer.stride;
                report.VertexBufferTarget = vertexBuffer.target.ToString();
                report.VertexBufferValid = vertexBuffer.IsValid();
                if (!report.VertexBufferValid)
                    throw new InvalidOperationException(
                        "vertex GraphicsBuffer.IsValid=false");
                if (report.VertexBufferStride <= 0 ||
                    report.VertexBufferStride > 1024)
                {
                    throw new InvalidOperationException(
                        "unsafe vertex stride=" + report.VertexBufferStride);
                }
                if (positionOffset + 12 > report.VertexBufferStride)
                    throw new InvalidOperationException(
                        "Position bytes exceed vertex record stride");
                if (mesh.GetVertexBufferStride(positionStream) !=
                    report.VertexBufferStride)
                {
                    throw new InvalidOperationException(
                        "Mesh and GraphicsBuffer vertex strides differ");
                }

                report.IndexRequestedByteOffset = checked(
                    subMesh.indexStart * report.IndexBufferStride);
                report.IndexRequestedByteCount = checked(
                    subMesh.indexCount * report.IndexBufferStride);
                report.VertexRequestedByteOffset = checked(
                    subMesh.firstVertex * report.VertexBufferStride);
                report.VertexRequestedByteCount = checked(
                    subMesh.vertexCount * report.VertexBufferStride);

                logger?.LogInfo(
                    "[GSP] Full routed byte ranges" +
                    " | indexBytes=[" + report.IndexRequestedByteOffset +
                    "," + (report.IndexRequestedByteOffset +
                        report.IndexRequestedByteCount) + ")" +
                    " | vertexBytes=[" + report.VertexRequestedByteOffset +
                    "," + (report.VertexRequestedByteOffset +
                        report.VertexRequestedByteCount) + ")" +
                    " | managedPayload=" +
                        (report.IndexRequestedByteCount +
                         report.VertexRequestedByteCount) + " bytes");

                logger?.LogWarning(
                    "[GSP] Native pointer boundary enter" +
                    " | call=index GraphicsBuffer.GetNativeBufferPtr");
                indexSourcePointer = indexBuffer.GetNativeBufferPtr();
                report.IndexSourcePointer = indexSourcePointer;
                logger?.LogInfo(
                    "[GSP] Native pointer boundary returned" +
                    " | indexPointer=" + FormatPointer(indexSourcePointer));
                if (indexSourcePointer == IntPtr.Zero)
                    throw new InvalidOperationException(
                        "index GetNativeBufferPtr returned zero");

                logger?.LogWarning(
                    "[GSP] Native pointer boundary enter" +
                    " | call=vertex GraphicsBuffer.GetNativeBufferPtr");
                vertexSourcePointer = vertexBuffer.GetNativeBufferPtr();
                report.VertexSourcePointer = vertexSourcePointer;
                logger?.LogInfo(
                    "[GSP] Native pointer boundary returned" +
                    " | vertexPointer=" + FormatPointer(vertexSourcePointer));
                if (vertexSourcePointer == IntPtr.Zero)
                    throw new InvalidOperationException(
                        "vertex GetNativeBufferPtr returned zero");

                IntPtr indexVTable = ReadVTable(
                    indexSourcePointer,
                    "index source ID3D11Buffer",
                    logger);
                GetDescDelegate getIndexDesc = ResolveDelegate<GetDescDelegate>(
                    indexVTable,
                    BufferGetDescSlot,
                    "index ID3D11Buffer::GetDesc",
                    logger);
                getIndexDesc(indexSourcePointer, out D3D11BufferDesc indexDesc);
                report.IndexSourceByteWidth = indexDesc.ByteWidth;

                IntPtr vertexVTable = ReadVTable(
                    vertexSourcePointer,
                    "vertex source ID3D11Buffer",
                    logger);
                GetDescDelegate getVertexDesc = ResolveDelegate<GetDescDelegate>(
                    vertexVTable,
                    BufferGetDescSlot,
                    "vertex ID3D11Buffer::GetDesc",
                    logger);
                getVertexDesc(vertexSourcePointer, out D3D11BufferDesc vertexDesc);
                report.VertexSourceByteWidth = vertexDesc.ByteWidth;

                logger?.LogInfo(
                    "[GSP] D3D11 source descriptors" +
                    " | index=" + indexDesc.ByteWidth +
                        "/usage" + indexDesc.Usage +
                        "/bind0x" + indexDesc.BindFlags.ToString("X") +
                    " | vertex=" + vertexDesc.ByteWidth +
                        "/usage" + vertexDesc.Usage +
                        "/bind0x" + vertexDesc.BindFlags.ToString("X"));

                long expectedIndexBytes = checked(
                    (long)report.IndexBufferCount *
                    report.IndexBufferStride);
                long expectedVertexBytes = checked(
                    (long)report.VertexBufferCount *
                    report.VertexBufferStride);
                if (indexDesc.ByteWidth != expectedIndexBytes)
                    throw new InvalidOperationException(
                        "index ByteWidth differs from count*stride");
                if (vertexDesc.ByteWidth != expectedVertexBytes)
                    throw new InvalidOperationException(
                        "vertex ByteWidth differs from count*stride");
                if ((indexDesc.BindFlags & D3D11BindIndexBuffer) == 0)
                    throw new InvalidOperationException(
                        "index source lacks D3D11_BIND_INDEX_BUFFER");
                if ((vertexDesc.BindFlags & D3D11BindVertexBuffer) == 0)
                    throw new InvalidOperationException(
                        "vertex source lacks D3D11_BIND_VERTEX_BUFFER");
                if (indexDesc.Usage != D3D11UsageDefault ||
                    vertexDesc.Usage != D3D11UsageDefault)
                {
                    throw new InvalidOperationException(
                        "source Usage must be DEFAULT(0)");
                }
                if ((long)report.IndexRequestedByteOffset +
                    report.IndexRequestedByteCount > indexDesc.ByteWidth)
                {
                    throw new InvalidOperationException(
                        "routed index byte range exceeds source buffer");
                }
                if ((long)report.VertexRequestedByteOffset +
                    report.VertexRequestedByteCount > vertexDesc.ByteWidth)
                {
                    throw new InvalidOperationException(
                        "routed vertex byte range exceeds source buffer");
                }

                GetDeviceDelegate getDevice = ResolveDelegate<GetDeviceDelegate>(
                    indexVTable,
                    DeviceChildGetDeviceSlot,
                    "ID3D11DeviceChild::GetDevice",
                    logger);
                logger?.LogWarning(
                    "[GSP] D3D11 device boundary enter" +
                    " | NOTE=returned device reference will be Released by GSP");
                getDevice(indexSourcePointer, out devicePointer);
                report.DevicePointer = devicePointer;
                logger?.LogInfo(
                    "[GSP] D3D11 device boundary returned" +
                    " | device=" + FormatPointer(devicePointer));
                if (devicePointer == IntPtr.Zero)
                    throw new InvalidOperationException(
                        "ID3D11Buffer.GetDevice returned zero");

                IntPtr deviceVTable = ReadVTable(
                    devicePointer,
                    "ID3D11Device",
                    logger);
                CreateBufferDelegate createBuffer =
                    ResolveDelegate<CreateBufferDelegate>(
                        deviceVTable,
                        DeviceCreateBufferSlot,
                        "ID3D11Device::CreateBuffer",
                        logger);
                GetImmediateContextDelegate getImmediateContext =
                    ResolveDelegate<GetImmediateContextDelegate>(
                        deviceVTable,
                        DeviceGetImmediateContextSlot,
                        "ID3D11Device::GetImmediateContext",
                        logger);

                uint stagingWidth = checked((uint)Math.Max(
                    report.IndexRequestedByteCount,
                    report.VertexRequestedByteCount));
                if (stagingWidth == 0)
                    throw new InvalidOperationException(
                        "computed staging ByteWidth is zero");

                D3D11BufferDesc stagingDesc = new D3D11BufferDesc
                {
                    ByteWidth = stagingWidth,
                    Usage = D3D11UsageStaging,
                    BindFlags = 0,
                    CPUAccessFlags = D3D11CpuAccessRead,
                    MiscFlags = 0,
                    StructureByteStride = 0
                };

                logger?.LogWarning(
                    "[GSP] D3D11 staging creation boundary enter" +
                    " | byteWidth=" + stagingDesc.ByteWidth +
                    " | usage=STAGING(3) | cpuAccess=READ(0x20000)");
                report.CreateBufferHResult = createBuffer(
                    devicePointer,
                    ref stagingDesc,
                    IntPtr.Zero,
                    out stagingPointer);
                report.StagingBufferPointer = stagingPointer;
                logger?.LogInfo(
                    "[GSP] D3D11 staging creation boundary returned" +
                    " | hr=" + FormatHResult(report.CreateBufferHResult) +
                    " | staging=" + FormatPointer(stagingPointer));
                if (report.CreateBufferHResult < 0 ||
                    stagingPointer == IntPtr.Zero)
                {
                    throw new InvalidOperationException(
                        "CreateBuffer failed | hr=" +
                        FormatHResult(report.CreateBufferHResult));
                }

                IntPtr stagingVTable = ReadVTable(
                    stagingPointer,
                    "staging ID3D11Buffer",
                    logger);
                GetDescDelegate getStagingDesc =
                    ResolveDelegate<GetDescDelegate>(
                        stagingVTable,
                        BufferGetDescSlot,
                        "staging ID3D11Buffer::GetDesc",
                        logger);
                getStagingDesc(
                    stagingPointer,
                    out D3D11BufferDesc actualStagingDesc);
                report.StagingByteWidth = actualStagingDesc.ByteWidth;
                if (actualStagingDesc.ByteWidth != stagingWidth ||
                    actualStagingDesc.Usage != D3D11UsageStaging ||
                    actualStagingDesc.BindFlags != 0 ||
                    (actualStagingDesc.CPUAccessFlags &
                        D3D11CpuAccessRead) == 0)
                {
                    throw new InvalidOperationException(
                        "created staging descriptor does not match readback requirements");
                }

                logger?.LogWarning(
                    "[GSP] D3D11 immediate-context boundary enter" +
                    " | NOTE=returned context reference will be Released by GSP");
                getImmediateContext(devicePointer, out contextPointer);
                report.ImmediateContextPointer = contextPointer;
                logger?.LogInfo(
                    "[GSP] D3D11 immediate-context boundary returned" +
                    " | context=" + FormatPointer(contextPointer));
                if (contextPointer == IntPtr.Zero)
                    throw new InvalidOperationException(
                        "GetImmediateContext returned zero");

                IntPtr contextVTable = ReadVTable(
                    contextPointer,
                    "ID3D11DeviceContext",
                    logger);
                MapDelegate map = ResolveDelegate<MapDelegate>(
                    contextVTable,
                    ContextMapSlot,
                    "ID3D11DeviceContext::Map",
                    logger);
                unmap = ResolveDelegate<UnmapDelegate>(
                    contextVTable,
                    ContextUnmapSlot,
                    "ID3D11DeviceContext::Unmap",
                    logger);
                CopySubresourceRegionDelegate copy =
                    ResolveDelegate<CopySubresourceRegionDelegate>(
                        contextVTable,
                        ContextCopySubresourceRegionSlot,
                        "ID3D11DeviceContext::CopySubresourceRegion",
                        logger);

                QueryInterfaceDelegate queryInterface =
                    ResolveDelegate<QueryInterfaceDelegate>(
                        contextVTable,
                        IUnknownQueryInterfaceSlot,
                        "IUnknown::QueryInterface(ID3D11Multithread)",
                        logger);
                Guid multithreadId = IidId3D11Multithread;
                report.MultithreadQueryHResult = queryInterface(
                    contextPointer,
                    ref multithreadId,
                    out multithreadPointer);
                report.MultithreadInterfacePointer = multithreadPointer;
                logger?.LogInfo(
                    "[GSP] D3D11 multithread QueryInterface returned" +
                    " | hr=" + FormatHResult(report.MultithreadQueryHResult) +
                    " | pointer=" + FormatPointer(multithreadPointer));
                if (report.MultithreadQueryHResult < 0 ||
                    multithreadPointer == IntPtr.Zero)
                {
                    throw new InvalidOperationException(
                        "ID3D11Multithread QueryInterface failed");
                }

                IntPtr multithreadVTable = ReadVTable(
                    multithreadPointer,
                    "ID3D11Multithread",
                    logger);
                MultithreadEnterDelegate multithreadEnter =
                    ResolveDelegate<MultithreadEnterDelegate>(
                        multithreadVTable,
                        MultithreadEnterSlot,
                        "ID3D11Multithread::Enter",
                        logger);
                multithreadLeave =
                    ResolveDelegate<MultithreadLeaveDelegate>(
                        multithreadVTable,
                        MultithreadLeaveSlot,
                        "ID3D11Multithread::Leave",
                        logger);
                setMultithreadProtected =
                    ResolveDelegate<SetMultithreadProtectedDelegate>(
                        multithreadVTable,
                        MultithreadSetProtectedSlot,
                        "ID3D11Multithread::SetMultithreadProtected",
                        logger);
                getMultithreadProtected =
                    ResolveDelegate<GetMultithreadProtectedDelegate>(
                        multithreadVTable,
                        MultithreadGetProtectedSlot,
                        "ID3D11Multithread::GetMultithreadProtected",
                        logger);

                report.MultithreadProtectedBefore =
                    getMultithreadProtected(multithreadPointer) != 0;
                if (!report.MultithreadProtectedBefore)
                {
                    logger?.LogWarning(
                        "[GSP] D3D11 multithread protection enable boundary enter" +
                        " | temporary=True | restoreAfterReadback=True");
                    setMultithreadProtected(multithreadPointer, 1);
                    multithreadProtectionChanged = true;
                    report.MultithreadProtectionEnabledByGsp = true;
                }
                report.MultithreadProtectedDuring =
                    getMultithreadProtected(multithreadPointer) != 0;
                if (!report.MultithreadProtectedDuring)
                    throw new InvalidOperationException(
                        "ID3D11Multithread protection remains disabled");

                logger?.LogWarning(
                    "[GSP] D3D11 full-submesh critical section enter");
                multithreadEnter(multithreadPointer);
                multithreadEntered = true;
                report.MultithreadEnterCalled = true;

                byte[] indexPayload = ReadRange(
                    "index",
                    contextPointer,
                    stagingPointer,
                    indexSourcePointer,
                    report.IndexRequestedByteOffset,
                    report.IndexRequestedByteCount,
                    copy,
                    map,
                    unmap,
                    logger,
                    out int indexMapHResult,
                    out bool indexCopyIssued,
                    out bool indexUnmapCalled);
                report.IndexMapHResult = indexMapHResult;
                report.IndexMapSucceeded = indexMapHResult >= 0;
                report.IndexCopyIssued = indexCopyIssued;
                report.IndexUnmapCalled = indexUnmapCalled;
                report.IndexPayloadAllZero = IsAllZero(indexPayload);
                report.IndexPayloadHash = Fnv1A64(indexPayload);
                report.IndexPayloadSample = SampleHex(indexPayload, 24);

                int[] localIndices = DecodeAndValidateIndices(
                    indexPayload,
                    report.IndexBufferStride,
                    subMesh,
                    report,
                    out bool[] referencedVertices);

                byte[] vertexPayload = ReadRange(
                    "vertex",
                    contextPointer,
                    stagingPointer,
                    vertexSourcePointer,
                    report.VertexRequestedByteOffset,
                    report.VertexRequestedByteCount,
                    copy,
                    map,
                    unmap,
                    logger,
                    out int vertexMapHResult,
                    out bool vertexCopyIssued,
                    out bool vertexUnmapCalled);
                report.VertexMapHResult = vertexMapHResult;
                report.VertexMapSucceeded = vertexMapHResult >= 0;
                report.VertexCopyIssued = vertexCopyIssued;
                report.VertexUnmapCalled = vertexUnmapCalled;
                report.VertexPayloadAllZero = IsAllZero(vertexPayload);
                report.VertexPayloadHash = Fnv1A64(vertexPayload);
                report.VertexPayloadSample = SampleHex(vertexPayload, 24);

                Vector3[] rawPositions = DecodePositions(
                    vertexPayload,
                    report.VertexBufferStride,
                    positionOffset,
                    renderer,
                    referencedVertices,
                    report);
                ValidateTriangles(localIndices, rawPositions, report);

                multithreadLeave(multithreadPointer);
                multithreadEntered = false;
                report.MultithreadLeaveCalled = true;
                logger?.LogInfo(
                    "[GSP] D3D11 full-submesh critical section left");

                if (report.IndexPayloadAllZero)
                    throw new InvalidOperationException(
                        "full index payload is entirely zero");
                if (report.VertexPayloadAllZero)
                    throw new InvalidOperationException(
                        "full vertex payload is entirely zero");
                if (report.DecodedIndexCount != subMesh.indexCount)
                    throw new InvalidOperationException(
                        "decoded index count mismatch");
                if (report.DecodedPositionCount != subMesh.vertexCount)
                    throw new InvalidOperationException(
                        "decoded position count mismatch");
                if (report.OutOfRangeIndexCount != 0)
                    throw new InvalidOperationException(
                        "one or more effective indices are outside routed vertex range");
                if (report.NonFinitePositionCount != 0)
                    throw new InvalidOperationException(
                        "one or more Position values are NaN or Infinity");
                if (report.InvalidTriangleIndexCount != 0)
                    throw new InvalidOperationException(
                        "one or more triangles reference invalid local indices");
                if (!report.RawCoordinatesMatchRendererBounds &&
                    !report.TransformedCoordinatesMatchRendererBounds)
                {
                    throw new InvalidOperationException(
                        "neither raw nor transformed full vertex set fits renderer bounds");
                }

                Vector3[] worldPositions;
                if (report.RawCoordinatesMatchRendererBounds)
                {
                    worldPositions = rawPositions;
                }
                else
                {
                    worldPositions = new Vector3[rawPositions.Length];
                    for (int i = 0; i < rawPositions.Length; i++)
                    {
                        worldPositions[i] =
                            renderer.transform.TransformPoint(rawPositions[i]);
                    }
                }

                geometry = new D3D11SubmeshGeometry
                {
                    Target = target,
                    SourceRenderer = renderer,
                    TargetName = report.TargetName,
                    TargetPath = report.TargetPath,
                    RendererPath = report.RendererPath,
                    SourceMeshName = report.MeshName,
                    RoutedSubMeshIndex = report.RoutedSubMeshIndex,
                    FirstVertex = report.FirstVertex,
                    VertexCount = report.VertexCount,
                    IndexCount = report.IndexCount,
                    CoordinateMode = report.CoordinateMode,
                    RawPositions = rawPositions,
                    WorldPositions = worldPositions,
                    LocalIndices = localIndices,
                    WorldBounds = CalculateBounds(worldPositions)
                };

                report.Success = true;
                report.Detail =
                    "full routed submesh readback validated all indices and positions; " +
                    "validated managed geometry is available to the optional preview layer";
            }
            catch (Exception e)
            {
                report.Success = false;
                report.Detail = e.GetType().Name + " / " + e.Message;
                logger?.LogError(
                    "[GSP] D3D11 full-submesh readback failure | " +
                    report.Detail);
            }
            finally
            {
                if (multithreadEntered &&
                    multithreadLeave != null &&
                    multithreadPointer != IntPtr.Zero)
                {
                    try
                    {
                        multithreadLeave(multithreadPointer);
                        multithreadEntered = false;
                        report.MultithreadLeaveCalled = true;
                        logger?.LogWarning(
                            "[GSP] D3D11 cleanup left critical section after failure");
                    }
                    catch (Exception e)
                    {
                        logger?.LogError(
                            "[GSP] D3D11 cleanup Leave failed | " +
                            e.GetType().Name + " / " + e.Message);
                    }
                }

                if (multithreadProtectionChanged &&
                    setMultithreadProtected != null &&
                    multithreadPointer != IntPtr.Zero)
                {
                    try
                    {
                        logger?.LogWarning(
                            "[GSP] D3D11 multithread protection restore boundary enter" +
                            " | restore=False");
                        setMultithreadProtected(multithreadPointer, 0);
                        report.MultithreadProtectionRestored =
                            getMultithreadProtected != null &&
                            getMultithreadProtected(multithreadPointer) == 0;
                        logger?.LogInfo(
                            "[GSP] D3D11 multithread protection restore returned" +
                            " | restored=" +
                            report.MultithreadProtectionRestored);
                    }
                    catch (Exception e)
                    {
                        logger?.LogError(
                            "[GSP] D3D11 multithread restore failed | " +
                            e.GetType().Name + " / " + e.Message);
                    }
                }
                else if (multithreadPointer != IntPtr.Zero)
                {
                    report.MultithreadProtectionRestored = true;
                }

                report.MultithreadReleased = TryReleaseOwned(
                    multithreadPointer,
                    "ID3D11Multithread",
                    logger,
                    out uint multithreadCount);
                report.MultithreadReleaseCount = multithreadCount;

                report.StagingReleased = TryReleaseOwned(
                    stagingPointer,
                    "staging ID3D11Buffer",
                    logger,
                    out uint stagingCount);
                report.StagingReleaseCount = stagingCount;

                report.ContextReleased = TryReleaseOwned(
                    contextPointer,
                    "ID3D11DeviceContext",
                    logger,
                    out uint contextCount);
                report.ContextReleaseCount = contextCount;

                report.DeviceReleased = TryReleaseOwned(
                    devicePointer,
                    "ID3D11Device",
                    logger,
                    out uint deviceCount);
                report.DeviceReleaseCount = deviceCount;

                DisposeWrapper(vertexBuffer, "vertex", logger);
                DisposeWrapper(indexBuffer, "index", logger);

                report.VertexTargetAfter = Safe(
                    () => mesh.vertexBufferTarget.ToString(),
                    "<getter-error>");
                report.IndexTargetAfter = Safe(
                    () => mesh.indexBufferTarget.ToString(),
                    "<getter-error>");
                report.TargetsUnchanged =
                    string.Equals(
                        report.VertexTargetBefore,
                        report.VertexTargetAfter,
                        StringComparison.Ordinal) &&
                    string.Equals(
                        report.IndexTargetBefore,
                        report.IndexTargetAfter,
                        StringComparison.Ordinal);

                bool ownedCleanupComplete =
                    (multithreadPointer == IntPtr.Zero ||
                        report.MultithreadReleased) &&
                    (stagingPointer == IntPtr.Zero ||
                        report.StagingReleased) &&
                    (contextPointer == IntPtr.Zero ||
                        report.ContextReleased) &&
                    (devicePointer == IntPtr.Zero ||
                        report.DeviceReleased);

                if (!report.TargetsUnchanged ||
                    !ownedCleanupComplete ||
                    (multithreadPointer != IntPtr.Zero &&
                        !report.MultithreadProtectionRestored))
                {
                    report.Success = false;
                    report.Detail =
                        (report.Detail ?? string.Empty) +
                        " | target state, COM cleanup or multithread restoration incomplete";
                }
            }

            if (report.EffectiveIndexMinimum == int.MaxValue)
                report.EffectiveIndexMinimum = 0;
            if (report.EffectiveIndexMaximum == int.MinValue)
                report.EffectiveIndexMaximum = 0;
            if (report.LocalIndexMinimum == int.MaxValue)
                report.LocalIndexMinimum = 0;
            if (report.LocalIndexMaximum == int.MinValue)
                report.LocalIndexMaximum = 0;
            if (report.MinimumTriangleArea == float.MaxValue)
                report.MinimumTriangleArea = 0f;

            stopwatch.Stop();
            report.ElapsedMilliseconds =
                (float)stopwatch.Elapsed.TotalMilliseconds;
            report.Completed = true;
            logger?.LogInfo(
                "[GSP] D3D11 full-submesh readback result | " +
                report.ToText());
            return report;
        }

        private static byte[] ReadRange(
            string label,
            IntPtr contextPointer,
            IntPtr stagingPointer,
            IntPtr sourcePointer,
            int sourceOffset,
            int byteCount,
            CopySubresourceRegionDelegate copy,
            MapDelegate map,
            UnmapDelegate unmap,
            ManualLogSource logger,
            out int mapHResult,
            out bool copyIssued,
            out bool unmapCalled)
        {
            copyIssued = false;
            unmapCalled = false;
            mapHResult = unchecked((int)0x80004005);
            if (byteCount <= 0)
                throw new ArgumentOutOfRangeException(
                    nameof(byteCount),
                    label + " byte count must be positive");

            D3D11Box sourceBox = new D3D11Box
            {
                Left = checked((uint)sourceOffset),
                Top = 0,
                Front = 0,
                Right = checked((uint)(sourceOffset + byteCount)),
                Bottom = 1,
                Back = 1
            };

            logger?.LogWarning(
                "[GSP] D3D11 full " + label + " copy boundary enter" +
                " | sourceBytes=[" + sourceBox.Left +
                "," + sourceBox.Right + ")" +
                " | destinationBytes=[0," + byteCount + ")");
            copy(
                contextPointer,
                stagingPointer,
                0,
                0,
                0,
                0,
                sourcePointer,
                0,
                ref sourceBox);
            copyIssued = true;

            logger?.LogWarning(
                "[GSP] D3D11 full " + label + " map boundary enter" +
                " | bytes=" + byteCount);
            mapHResult = map(
                contextPointer,
                stagingPointer,
                0,
                D3D11MapRead,
                0,
                out D3D11MappedSubresource mappedResource);
            logger?.LogInfo(
                "[GSP] D3D11 full " + label + " map boundary returned" +
                " | hr=" + FormatHResult(mapHResult) +
                " | data=" + FormatPointer(mappedResource.Data) +
                " | rowPitch=" + mappedResource.RowPitch);
            if (mapHResult < 0 || mappedResource.Data == IntPtr.Zero)
            {
                throw new InvalidOperationException(
                    label + " Map failed | hr=" +
                    FormatHResult(mapHResult));
            }

            byte[] payload = new byte[byteCount];
            try
            {
                Marshal.Copy(
                    mappedResource.Data,
                    payload,
                    0,
                    byteCount);
            }
            finally
            {
                unmap(contextPointer, stagingPointer, 0);
                unmapCalled = true;
            }

            logger?.LogInfo(
                "[GSP] D3D11 full " + label + " payload copied" +
                " | bytes=" + payload.Length +
                " | allZero=" + IsAllZero(payload) +
                " | hash=" + Fnv1A64(payload) +
                " | sample=" + SampleHex(payload, 16));
            return payload;
        }

        private static int[] DecodeAndValidateIndices(
            byte[] payload,
            int stride,
            SubMeshDescriptor subMesh,
            D3D11FullSubmeshReadbackReport report,
            out bool[] referenced)
        {
            int count = subMesh.indexCount;
            if (payload.Length != checked(count * stride))
                throw new InvalidOperationException(
                    "index payload length does not match descriptor");

            int[] localIndices = new int[count];
            referenced = new bool[subMesh.vertexCount];
            int unique = 0;
            for (int i = 0; i < count; i++)
            {
                int raw;
                if (stride == 2)
                {
                    raw = BitConverter.ToUInt16(payload, i * 2);
                }
                else if (stride == 4)
                {
                    uint value = BitConverter.ToUInt32(payload, i * 4);
                    if (value > int.MaxValue)
                        throw new OverflowException(
                            "UInt32 index exceeds Int32.MaxValue at " + i);
                    raw = (int)value;
                }
                else
                {
                    throw new NotSupportedException(
                        "index stride must be 2 or 4");
                }

                long effectiveLong = (long)raw + subMesh.baseVertex;
                if (effectiveLong < int.MinValue ||
                    effectiveLong > int.MaxValue)
                {
                    throw new OverflowException(
                        "effective index overflow at " + i);
                }
                int effective = (int)effectiveLong;
                int local = effective - subMesh.firstVertex;
                localIndices[i] = local;

                if (effective < report.EffectiveIndexMinimum)
                    report.EffectiveIndexMinimum = effective;
                if (effective > report.EffectiveIndexMaximum)
                    report.EffectiveIndexMaximum = effective;
                if (local < report.LocalIndexMinimum)
                    report.LocalIndexMinimum = local;
                if (local > report.LocalIndexMaximum)
                    report.LocalIndexMaximum = local;

                if (local < 0 || local >= subMesh.vertexCount)
                {
                    report.OutOfRangeIndexCount++;
                }
                else if (!referenced[local])
                {
                    referenced[local] = true;
                    unique++;
                }
            }

            report.DecodedIndexCount = count;
            report.UniqueReferencedVertexCount = unique;
            report.UnusedVertexCount = subMesh.vertexCount - unique;
            return localIndices;
        }

        private static Vector3[] DecodePositions(
            byte[] payload,
            int stride,
            int positionOffset,
            MeshRenderer renderer,
            bool[] referencedVertices,
            D3D11FullSubmeshReadbackReport report)
        {
            int count = report.VertexCount;
            if (payload.Length != checked(count * stride))
                throw new InvalidOperationException(
                    "vertex payload length does not match descriptor");

            Vector3[] rawPositions = new Vector3[count];
            Vector3 rawMin = Vector3.zero;
            Vector3 rawMax = Vector3.zero;
            Vector3 transformedMin = Vector3.zero;
            Vector3 transformedMax = Vector3.zero;
            bool haveFinite = false;
            Bounds rendererBounds = renderer.bounds;
            float tolerance = ComputeBoundsTolerance(rendererBounds);

            for (int i = 0; i < count; i++)
            {
                int offset = checked(i * stride + positionOffset);
                float x = BitConverter.ToSingle(payload, offset);
                float y = BitConverter.ToSingle(payload, offset + 4);
                float z = BitConverter.ToSingle(payload, offset + 8);
                Vector3 raw = new Vector3(x, y, z);
                rawPositions[i] = raw;
                report.DecodedPositionCount++;

                if (!IsFinite(raw))
                {
                    report.NonFinitePositionCount++;
                    continue;
                }

                if (Mathf.Abs(raw.x) <= 1e-20f &&
                    Mathf.Abs(raw.y) <= 1e-20f &&
                    Mathf.Abs(raw.z) <= 1e-20f)
                {
                    report.ZeroPositionCount++;
                }

                Vector3 transformed =
                    renderer.transform.TransformPoint(raw);
                bool rawInside = ContainsWithTolerance(
                    rendererBounds,
                    raw,
                    tolerance);
                bool transformedInside = ContainsWithTolerance(
                    rendererBounds,
                    transformed,
                    tolerance);
                if (!rawInside)
                    report.RawOutsideRendererBoundsCount++;
                if (!transformedInside)
                    report.TransformedOutsideRendererBoundsCount++;
                if (referencedVertices != null &&
                    i < referencedVertices.Length &&
                    referencedVertices[i])
                {
                    if (!rawInside)
                        report.ReferencedRawOutsideRendererBoundsCount++;
                    if (!transformedInside)
                        report.ReferencedTransformedOutsideRendererBoundsCount++;
                }

                if (!haveFinite)
                {
                    rawMin = rawMax = raw;
                    transformedMin = transformedMax = transformed;
                    haveFinite = true;
                }
                else
                {
                    rawMin = Vector3.Min(rawMin, raw);
                    rawMax = Vector3.Max(rawMax, raw);
                    transformedMin = Vector3.Min(transformedMin, transformed);
                    transformedMax = Vector3.Max(transformedMax, transformed);
                }
            }

            report.RendererBounds = FormatBounds(rendererBounds);
            if (haveFinite)
            {
                report.RawDecodedBounds =
                    FormatMinMaxBounds(rawMin, rawMax);
                report.TransformedDecodedBounds =
                    FormatMinMaxBounds(transformedMin, transformedMax);
            }
            else
            {
                report.RawDecodedBounds = "<no-finite-position>";
                report.TransformedDecodedBounds = "<no-finite-position>";
            }

            report.RawCoordinatesMatchRendererBounds =
                report.NonFinitePositionCount == 0 &&
                report.UniqueReferencedVertexCount > 0 &&
                report.ReferencedRawOutsideRendererBoundsCount == 0;
            report.TransformedCoordinatesMatchRendererBounds =
                report.NonFinitePositionCount == 0 &&
                report.UniqueReferencedVertexCount > 0 &&
                report.ReferencedTransformedOutsideRendererBoundsCount == 0;
            report.CoordinateMode = report.RawCoordinatesMatchRendererBounds
                ? "raw/world-baked"
                : (report.TransformedCoordinatesMatchRendererBounds
                    ? "renderer-transform"
                    : "unmatched");
            return rawPositions;
        }

        private static void ValidateTriangles(
            int[] localIndices,
            Vector3[] positions,
            D3D11FullSubmeshReadbackReport report)
        {
            report.TrailingIndexCount = localIndices.Length % 3;
            report.TriangleCount = localIndices.Length / 3;
            double areaSum = 0.0;
            int measuredAreaCount = 0;

            for (int triangle = 0;
                triangle < report.TriangleCount;
                triangle++)
            {
                int baseIndex = triangle * 3;
                int a = localIndices[baseIndex];
                int b = localIndices[baseIndex + 1];
                int c = localIndices[baseIndex + 2];

                if (a < 0 || a >= positions.Length ||
                    b < 0 || b >= positions.Length ||
                    c < 0 || c >= positions.Length)
                {
                    report.InvalidTriangleIndexCount++;
                    continue;
                }

                if (a == b || b == c || c == a)
                    report.RepeatedIndexTriangleCount++;

                Vector3 p0 = positions[a];
                Vector3 p1 = positions[b];
                Vector3 p2 = positions[c];
                if (!IsFinite(p0) ||
                    !IsFinite(p1) ||
                    !IsFinite(p2))
                {
                    report.InvalidTriangleIndexCount++;
                    continue;
                }

                float area = 0.5f *
                    Vector3.Cross(p1 - p0, p2 - p0).magnitude;
                if (!IsFinite(area))
                {
                    report.InvalidTriangleIndexCount++;
                    continue;
                }

                if (area <= ZeroAreaThreshold)
                    report.ZeroAreaTriangleCount++;
                if (area < report.MinimumTriangleArea)
                    report.MinimumTriangleArea = area;
                if (area > report.MaximumTriangleArea)
                    report.MaximumTriangleArea = area;
                areaSum += area;
                measuredAreaCount++;
            }

            report.AverageTriangleArea = measuredAreaCount > 0
                ? areaSum / measuredAreaCount
                : 0.0;
        }

        private static Bounds CalculateBounds(Vector3[] positions)
        {
            if (positions == null || positions.Length == 0)
                return new Bounds();

            Bounds bounds = new Bounds(positions[0], Vector3.zero);
            for (int i = 1; i < positions.Length; i++)
                bounds.Encapsulate(positions[i]);
            return bounds;
        }

        private static RoutedCandidate BuildSpecificRoutedCandidate(
            MeshRenderer renderer,
            int routedSubMeshIndex)
        {
            if (renderer == null)
                return null;

            MeshFilter filter = null;
            Mesh mesh = null;
            try
            {
                filter = renderer.GetComponent<MeshFilter>();
                mesh = filter != null ? filter.sharedMesh : null;
            }
            catch { }
            if (filter == null || mesh == null)
                return null;

            int materialCount = 0;
            try
            {
                var materials = renderer.sharedMaterials;
                materialCount = materials != null ? materials.Length : 0;
            }
            catch { }

            int start = Safe(() => renderer.subMeshStartIndex, 0);
            int subMeshCount = Safe(() => mesh.subMeshCount, 0);
            if (start < 0 || start >= subMeshCount)
                return null;

            int routedCount = materialCount > 0 ? materialCount : 1;
            if (start + routedCount > subMeshCount)
                routedCount = subMeshCount - start;
            if (routedCount <= 0 ||
                routedSubMeshIndex < start ||
                routedSubMeshIndex >= start + routedCount)
                return null;

            return new RoutedCandidate
            {
                Renderer = renderer,
                Filter = filter,
                Mesh = mesh,
                RoutedSubMeshIndex = routedSubMeshIndex,
                RoutedSubMeshCount = 1
            };
        }

        private static RoutedCandidate FindFirstRoutedCandidate(
            GameObject target)
        {
            MeshRenderer[] renderers =
                target.GetComponentsInChildren<MeshRenderer>(true);
            RoutedCandidate fallback = null;

            for (int i = 0; i < renderers.Length; i++)
            {
                MeshRenderer renderer = renderers[i];
                if (renderer == null)
                    continue;

                MeshFilter filter = renderer.GetComponent<MeshFilter>();
                Mesh mesh = filter != null ? filter.sharedMesh : null;
                if (filter == null || mesh == null)
                    continue;

                int materialCount = 0;
                try
                {
                    var materials = renderer.sharedMaterials;
                    materialCount = materials != null ? materials.Length : 0;
                }
                catch { }

                int start = Safe(() => renderer.subMeshStartIndex, 0);
                int subMeshCount = Safe(() => mesh.subMeshCount, 0);
                if (start < 0 || start >= subMeshCount)
                    continue;

                int routedCount = materialCount > 0 ? materialCount : 1;
                if (start + routedCount > subMeshCount)
                    routedCount = subMeshCount - start;
                if (routedCount <= 0)
                    continue;

                RoutedCandidate candidate = new RoutedCandidate
                {
                    Renderer = renderer,
                    Filter = filter,
                    Mesh = mesh,
                    RoutedSubMeshIndex = start,
                    RoutedSubMeshCount = routedCount
                };

                if (renderer.enabled &&
                    renderer.gameObject.activeInHierarchy)
                {
                    return candidate;
                }

                if (fallback == null)
                    fallback = candidate;
            }

            return fallback;
        }

        private static IntPtr ReadVTable(
            IntPtr objectPointer,
            string label,
            ManualLogSource logger)
        {
            if (objectPointer == IntPtr.Zero)
                throw new ArgumentException(
                    label + " object pointer is zero");

            logger?.LogWarning(
                "[GSP] D3D11 vtable boundary enter" +
                " | object=" + label +
                " | pointer=" + FormatPointer(objectPointer));
            IntPtr vtable = Marshal.ReadIntPtr(objectPointer);
            logger?.LogInfo(
                "[GSP] D3D11 vtable boundary returned" +
                " | object=" + label +
                " | vtable=" + FormatPointer(vtable));
            if (vtable == IntPtr.Zero)
                throw new InvalidOperationException(
                    label + " vtable pointer is zero");
            return vtable;
        }

        private static T ResolveDelegate<T>(
            IntPtr vtable,
            int slot,
            string label,
            ManualLogSource logger)
            where T : Delegate
        {
            IntPtr function = Marshal.ReadIntPtr(
                vtable,
                checked(slot * IntPtr.Size));
            logger?.LogInfo(
                "[GSP] D3D11 COM route resolved" +
                " | method=" + label +
                " | slot=" + slot +
                " | function=" + FormatPointer(function));
            if (function == IntPtr.Zero)
                throw new InvalidOperationException(
                    label + " function pointer is zero");
            return Marshal.GetDelegateForFunctionPointer<T>(function);
        }

        private static bool TryReleaseOwned(
            IntPtr pointer,
            string label,
            ManualLogSource logger,
            out uint remainingCount)
        {
            remainingCount = 0;
            if (pointer == IntPtr.Zero)
                return false;

            try
            {
                IntPtr vtable = Marshal.ReadIntPtr(pointer);
                IntPtr releaseFunction = Marshal.ReadIntPtr(
                    vtable,
                    IUnknownReleaseSlot * IntPtr.Size);
                if (releaseFunction == IntPtr.Zero)
                    throw new InvalidOperationException(
                        "IUnknown::Release pointer is zero");

                ReleaseDelegate release =
                    Marshal.GetDelegateForFunctionPointer<ReleaseDelegate>(
                        releaseFunction);
                logger?.LogWarning(
                    "[GSP] D3D11 owned COM cleanup boundary enter" +
                    " | call=" + label + ".Release" +
                    " | pointer=" + FormatPointer(pointer));
                remainingCount = release(pointer);
                logger?.LogInfo(
                    "[GSP] D3D11 owned COM cleanup boundary returned" +
                    " | call=" + label + ".Release" +
                    " | remainingRefCount=" + remainingCount);
                return true;
            }
            catch (Exception e)
            {
                logger?.LogError(
                    "[GSP] D3D11 owned COM cleanup failed" +
                    " | object=" + label +
                    " | " + e.GetType().Name +
                    " / " + e.Message);
                return false;
            }
        }

        private static void DisposeWrapper(
            GraphicsBuffer buffer,
            string label,
            ManualLogSource logger)
        {
            if (buffer == null)
                return;

            try
            {
                buffer.Dispose();
                logger?.LogInfo(
                    "[GSP] D3D11 " + label +
                    " GraphicsBuffer wrapper disposed" +
                    " | underlying Mesh and source COM ownership unchanged");
            }
            catch (Exception e)
            {
                logger?.LogWarning(
                    "[GSP] D3D11 " + label +
                    " wrapper dispose failed | " +
                    e.GetType().Name + " / " + e.Message);
            }
        }

        private static float ComputeBoundsTolerance(Bounds bounds)
        {
            float scale = Mathf.Max(
                Mathf.Abs(bounds.extents.x),
                Mathf.Max(
                    Mathf.Abs(bounds.extents.y),
                    Mathf.Abs(bounds.extents.z)));
            return Mathf.Max(0.0001f, scale * 0.0001f);
        }

        private static bool ContainsWithTolerance(
            Bounds bounds,
            Vector3 point,
            float tolerance)
        {
            Vector3 minimum = bounds.min;
            Vector3 maximum = bounds.max;
            return
                point.x >= minimum.x - tolerance &&
                point.x <= maximum.x + tolerance &&
                point.y >= minimum.y - tolerance &&
                point.y <= maximum.y + tolerance &&
                point.z >= minimum.z - tolerance &&
                point.z <= maximum.z + tolerance;
        }

        private static bool IsFinite(Vector3 value)
        {
            return IsFinite(value.x) &&
                   IsFinite(value.y) &&
                   IsFinite(value.z);
        }

        private static bool IsFinite(float value)
        {
            return !float.IsNaN(value) &&
                   !float.IsInfinity(value);
        }

        private static bool IsAllZero(byte[] payload)
        {
            if (payload == null || payload.Length == 0)
                return true;
            for (int i = 0; i < payload.Length; i++)
            {
                if (payload[i] != 0)
                    return false;
            }
            return true;
        }

        private static string Fnv1A64(byte[] payload)
        {
            const ulong offsetBasis = 14695981039346656037UL;
            const ulong prime = 1099511628211UL;
            ulong hash = offsetBasis;
            for (int i = 0; i < payload.Length; i++)
            {
                hash ^= payload[i];
                hash *= prime;
            }
            return "fnv1a64:" + hash.ToString("X16");
        }

        private static string SampleHex(byte[] payload, int edgeCount)
        {
            if (payload == null || payload.Length == 0)
                return "<empty>";
            int edge = Math.Max(1, edgeCount);
            if (payload.Length <= edge * 2)
                return JoinHex(payload, 0, payload.Length);
            return
                JoinHex(payload, 0, edge) +
                " ... " +
                JoinHex(payload, payload.Length - edge, edge);
        }

        private static string JoinHex(
            byte[] payload,
            int offset,
            int count)
        {
            string[] parts = new string[count];
            for (int i = 0; i < count; i++)
                parts[i] = payload[offset + i].ToString("X2");
            return string.Join(" ", parts);
        }

        private static string FormatBounds(Bounds bounds)
        {
            return
                "center=" + FormatVector(bounds.center) +
                "/size=" + FormatVector(bounds.size);
        }

        private static string FormatMinMaxBounds(
            Vector3 minimum,
            Vector3 maximum)
        {
            Vector3 center = (minimum + maximum) * 0.5f;
            Vector3 size = maximum - minimum;
            return
                "center=" + FormatVector(center) +
                "/size=" + FormatVector(size) +
                "/min=" + FormatVector(minimum) +
                "/max=" + FormatVector(maximum);
        }

        private static string FormatVector(Vector3 value)
        {
            return
                "(" + value.x.ToString("G9") +
                "," + value.y.ToString("G9") +
                "," + value.z.ToString("G9") + ")";
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

        private static T Safe<T>(Func<T> getter, T fallback)
        {
            try { return getter(); }
            catch { return fallback; }
        }
    }
}
