using System.Collections.Generic;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 只描述 Mesh 的顶点布局、子网格范围和现有 GPU target 标记。
    /// 不读取顶点/索引，不调用 GetVertexBuffer/GetIndexBuffer，也不修改 Mesh。
    /// </summary>
    public sealed class MeshLayoutEntry
    {
        public string ComponentType;
        public string ComponentPath;
        public string MeshName;
        public bool IsReadable;
        public int VertexCount;
        public int VertexBufferCount;
        public int SubMeshCount;
        public string IndexFormat;
        public string VertexBufferTarget;
        public string IndexBufferTarget;
        public readonly List<string> VertexStreams = new List<string>();
        public readonly List<string> Attributes = new List<string>();
        public readonly List<string> SubMeshes = new List<string>();
        public readonly List<string> Errors = new List<string>();

        public string ToText(int ordinal)
        {
            System.Text.StringBuilder builder = new System.Text.StringBuilder();
            builder.AppendLine(
                ordinal + ". " + ComponentType + " | " + ComponentPath +
                " | mesh=" + (MeshName ?? "<none>") +
                " | readable=" + IsReadable +
                " | v=" + VertexCount +
                " | streams=" + VertexBufferCount +
                " | sub=" + SubMeshCount +
                " | index=" + (IndexFormat ?? "<unknown>") +
                " | vertexTarget=" + (VertexBufferTarget ?? "<unknown>") +
                " | indexTarget=" + (IndexBufferTarget ?? "<unknown>"));

            for (int i = 0; i < VertexStreams.Count; i++)
                builder.AppendLine("   stream " + VertexStreams[i]);
            for (int i = 0; i < Attributes.Count; i++)
                builder.AppendLine("   attribute " + Attributes[i]);
            for (int i = 0; i < SubMeshes.Count; i++)
                builder.AppendLine("   submesh " + SubMeshes[i]);
            for (int i = 0; i < Errors.Count; i++)
                builder.AppendLine("   error " + Errors[i]);

            return builder.ToString().TrimEnd();
        }
    }

    public sealed class GeometryLayoutReport
    {
        public bool Completed;
        public string TargetName;
        public string TargetPath;
        public string ReferenceName;
        public int ExactMatches;
        public float SelectedDistance;
        public float ElapsedMilliseconds;
        public string Summary;
        public readonly List<MeshLayoutEntry> Entries = new List<MeshLayoutEntry>();

        public string ToText(bool includeEntries)
        {
            string header =
                "GSP layout | completed=" + Completed +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | reference=" + (ReferenceName ?? "<none>") +
                " | exactMatches=" + ExactMatches +
                " | distance=" + SelectedDistance.ToString("F3") + "m" +
                " | entries=" + Entries.Count +
                " | elapsed=" + ElapsedMilliseconds.ToString("F1") + "ms" +
                " | summary=" + (Summary ?? "<none>");

            if (!includeEntries || Entries.Count == 0)
                return header;

            System.Text.StringBuilder builder = new System.Text.StringBuilder(header);
            builder.AppendLine();
            builder.AppendLine("--- read-only mesh layout ---");
            for (int i = 0; i < Entries.Count; i++)
                builder.AppendLine(Entries[i].ToText(i + 1));
            return builder.ToString().TrimEnd();
        }
    }

    /// <summary>
    /// 单次现有 GPU buffer 句柄探测的结果。
    /// 该测试不读取 buffer 内容，也不改变 Mesh target 标记。
    /// </summary>
    public sealed class BufferHandleProbeReport
    {
        public bool Completed;
        public bool Success;
        public string TargetName;
        public string TargetPath;
        public string ComponentType;
        public string ComponentPath;
        public string MeshName;
        public string RequestedBuffer;
        public int CandidateMeshes;
        public int BufferCount;
        public int BufferStride;
        public string BufferTarget;
        public bool BufferValid;
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
                "GSP handle | completed=" + Completed +
                " | success=" + Success +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | component=" + (ComponentType ?? "<none>") +
                " | componentPath=" + (ComponentPath ?? "<none>") +
                " | mesh=" + (MeshName ?? "<none>") +
                " | request=" + (RequestedBuffer ?? "<none>") +
                " | candidates=" + CandidateMeshes +
                " | bufferCount=" + BufferCount +
                " | bufferStride=" + BufferStride +
                " | bufferTarget=" + (BufferTarget ?? "<unknown>") +
                " | bufferValid=" + BufferValid +
                " | vertexTarget=" + (VertexTargetBefore ?? "<unknown>") +
                "->" + (VertexTargetAfter ?? "<unknown>") +
                " | indexTarget=" + (IndexTargetBefore ?? "<unknown>") +
                "->" + (IndexTargetAfter ?? "<unknown>") +
                " | unchanged=" + TargetsUnchanged +
                " | elapsed=" + ElapsedMilliseconds.ToString("F1") + "ms" +
                " | detail=" + (Detail ?? "<none>");
        }
    }
}
