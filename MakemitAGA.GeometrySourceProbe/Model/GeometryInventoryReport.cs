using System.Collections.Generic;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 完全不读取顶点、索引、MeshData 或 GraphicsBuffer 的安全资源清单。
    /// 它只记录 Unity 已公开的组件与 Mesh 元数据，用于在尝试任何提取 API 前
    /// 判断目标究竟由 MeshFilter、SkinnedMeshRenderer 还是 MeshCollider 组成。
    /// </summary>
    public sealed class GeometryInventoryEntry
    {
        public string ComponentType;
        public string ComponentPath;
        public string MeshName;
        public bool MeshPresent;
        public bool IsReadable;
        public int VertexCount;
        public int SubMeshCount;
        public string IndexFormat;
        public string Bounds;
        public bool ComponentEnabled;
        public bool GameObjectActive;
        public bool RendererPresent;
        public bool RendererEnabled;
        public bool ColliderConvex;
        public bool ColliderTrigger;
        public string Detail;

        public string ToLine()
        {
            return
                ComponentType + " | " + ComponentPath +
                " | mesh=" + (MeshName ?? "<none>") +
                " | present=" + MeshPresent +
                " | readable=" + IsReadable +
                " | v=" + VertexCount +
                " | sub=" + SubMeshCount +
                " | index=" + (IndexFormat ?? "<unknown>") +
                " | bounds=" + (Bounds ?? "<unknown>") +
                " | componentEnabled=" + ComponentEnabled +
                " | active=" + GameObjectActive +
                " | renderer=" + RendererPresent + "/" + RendererEnabled +
                " | convex=" + ColliderConvex +
                " | trigger=" + ColliderTrigger +
                " | detail=" + (Detail ?? "<none>");
        }
    }

    public sealed class GeometryInventoryReport
    {
        public bool Completed;
        public string TargetName;
        public string TargetPath;
        public string ReferenceName;
        public int ExactMatches;
        public float SelectedDistance;
        public int MeshFilters;
        public int SkinnedRenderers;
        public int MeshColliders;
        public int Renderers;
        public float ElapsedMilliseconds;
        public string Summary;
        public readonly List<GeometryInventoryEntry> Entries =
            new List<GeometryInventoryEntry>();

        public string ToText(bool includeEntries)
        {
            string header =
                "GSP inventory | completed=" + Completed +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | reference=" + (ReferenceName ?? "<none>") +
                " | exactMatches=" + ExactMatches +
                " | distance=" + SelectedDistance.ToString("F3") + "m" +
                " | renderers=" + Renderers +
                " | meshFilters=" + MeshFilters +
                " | skinned=" + SkinnedRenderers +
                " | meshColliders=" + MeshColliders +
                " | entries=" + Entries.Count +
                " | elapsed=" + ElapsedMilliseconds.ToString("F1") + "ms" +
                " | summary=" + (Summary ?? "<none>");

            if (!includeEntries || Entries.Count == 0)
                return header;

            System.Text.StringBuilder builder = new System.Text.StringBuilder(header);
            builder.AppendLine();
            builder.AppendLine("--- safe metadata inventory ---");
            for (int i = 0; i < Entries.Count; i++)
                builder.AppendLine((i + 1) + ". " + Entries[i].ToLine());
            return builder.ToString().TrimEnd();
        }
    }
}
