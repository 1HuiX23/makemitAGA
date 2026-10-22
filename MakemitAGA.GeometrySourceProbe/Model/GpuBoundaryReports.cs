using System.Collections.Generic;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 描述目标 Renderer 如何引用静态批处理 Mesh。
    /// 该报告只读取 Renderer/Mesh 元数据，不取得或读取 GPU buffer。
    /// </summary>
    public sealed class RendererRouteEntry
    {
        public string RendererPath;
        public string RendererType;
        public bool Enabled;
        public bool Active;
        public bool IsPartOfStaticBatch;
        public int SubMeshStartIndex;
        public int MaterialCount;
        public int RoutedSubMeshStart;
        public int RoutedSubMeshCount;
        public string MeshName;
        public int MeshSubMeshCount;
        public int MeshVertexCount;
        public string RendererBounds;
        public string MeshBounds;
        public string Detail;

        public string ToText(int ordinal)
        {
            return
                ordinal + ". " + (RendererType ?? "<none>") +
                " | path=" + (RendererPath ?? "<none>") +
                " | enabled=" + Enabled +
                " | active=" + Active +
                " | staticBatch=" + IsPartOfStaticBatch +
                " | subMeshStartIndex=" + SubMeshStartIndex +
                " | materials=" + MaterialCount +
                " | routedSubMeshes=" + RoutedSubMeshStart + "+" + RoutedSubMeshCount +
                " | mesh=" + (MeshName ?? "<none>") +
                " | meshSub=" + MeshSubMeshCount +
                " | meshV=" + MeshVertexCount +
                " | rendererBounds=" + (RendererBounds ?? "<none>") +
                " | meshBounds=" + (MeshBounds ?? "<none>") +
                " | detail=" + (Detail ?? "<none>");
        }
    }

    public sealed class RendererRouteReport
    {
        public bool Completed;
        public string TargetName;
        public string TargetPath;
        public int ExactMatches;
        public float SelectedDistance;
        public string ReferenceName;
        public float ElapsedMilliseconds;
        public string Summary;
        public readonly List<RendererRouteEntry> Entries = new List<RendererRouteEntry>();

        public string ToText(bool includeEntries)
        {
            string header =
                "GSP renderer route | completed=" + Completed +
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
            builder.AppendLine("--- renderer to static-batch route ---");
            for (int i = 0; i < Entries.Count; i++)
                builder.AppendLine(Entries[i].ToText(i + 1));
            return builder.ToString().TrimEnd();
        }
    }


}
