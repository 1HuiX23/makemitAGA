using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Model
{
    /// <summary>
    /// 已被转换为世界坐标的只读快照。预览不会继续依赖原对象 Transform。
    /// </summary>
    public sealed class MeshSnapshot
    {
        public string SourceKind;
        public string ComponentType;
        public string ComponentPath;
        public string MeshName;
        public string Exactness;
        public Vector3[] WorldVertices;
        public readonly List<int[]> TriangleSubmeshes = new List<int[]>();
        public Bounds WorldBounds;

        public int VertexCount => WorldVertices == null ? 0 : WorldVertices.Length;

        public int IndexCount
        {
            get
            {
                int total = 0;
                for (int i = 0; i < TriangleSubmeshes.Count; i++)
                    total += TriangleSubmeshes[i]?.Length ?? 0;
                return total;
            }
        }

        public Mesh BuildUnityMesh(string name)
        {
            if (WorldVertices == null || WorldVertices.Length == 0)
                return null;

            Mesh mesh = new Mesh();
            mesh.name = name;
            mesh.indexFormat = WorldVertices.Length >= 65000
                ? IndexFormat.UInt32
                : IndexFormat.UInt16;
            mesh.vertices = WorldVertices;
            mesh.subMeshCount = Math.Max(1, TriangleSubmeshes.Count);

            if (TriangleSubmeshes.Count == 0)
            {
                mesh.triangles = Array.Empty<int>();
            }
            else
            {
                for (int i = 0; i < TriangleSubmeshes.Count; i++)
                {
                    mesh.SetIndices(
                        TriangleSubmeshes[i] ?? Array.Empty<int>(),
                        MeshTopology.Triangles,
                        i,
                        false);
                }
            }

            mesh.RecalculateBounds();
            mesh.RecalculateNormals();
            return mesh;
        }

        public static Bounds CalculateBounds(Vector3[] vertices)
        {
            if (vertices == null || vertices.Length == 0)
                return new Bounds();

            Bounds bounds = new Bounds(vertices[0], Vector3.zero);
            for (int i = 1; i < vertices.Length; i++)
                bounds.Encapsulate(vertices[i]);
            return bounds;
        }
    }

    public sealed class SourceAttemptReport
    {
        public string ComponentType;
        public string ComponentPath;
        public string SourceKind;
        public string MeshName;
        public bool IsReadable;
        public bool Success;
        public bool Selected;
        public int Vertices;
        public int Indices;
        public int Submeshes;
        public string Exactness;
        public string Detail;

        public string ToLine()
        {
            return
                ComponentType + " | " + ComponentPath +
                " | source=" + SourceKind +
                " | mesh=" + (MeshName ?? "<none>") +
                " | readable=" + IsReadable +
                " | success=" + Success +
                " | selected=" + Selected +
                " | v=" + Vertices +
                " | i=" + Indices +
                " | sub=" + Submeshes +
                " | exactness=" + (Exactness ?? "<none>") +
                " | detail=" + (Detail ?? "<none>");
        }
    }

    public sealed class GeometryProbeReport
    {
        public bool Completed;
        public string TargetName;
        public string TargetPath;
        public string ReferenceName;
        public int ExactMatches;
        public float SelectedDistance;
        public ProbePreference Preference;
        public int MeshFilters;
        public int SkinnedRenderers;
        public int MeshColliders;
        public int SelectedSnapshots;
        public int SelectedVertices;
        public int SelectedIndices;
        public float ElapsedMilliseconds;
        public string Summary;
        public readonly List<SourceAttemptReport> Attempts = new List<SourceAttemptReport>();
        public readonly List<MeshSnapshot> Snapshots = new List<MeshSnapshot>();

        public string ToText(bool includeAttempts)
        {
            string header =
                "GSP report | completed=" + Completed +
                " | target=" + (TargetName ?? "<none>") +
                " | path=" + (TargetPath ?? "<none>") +
                " | reference=" + (ReferenceName ?? "<none>") +
                " | exactMatches=" + ExactMatches +
                " | distance=" + SelectedDistance.ToString("F3") + "m" +
                " | preference=" + Preference +
                " | meshFilters=" + MeshFilters +
                " | skinned=" + SkinnedRenderers +
                " | meshColliders=" + MeshColliders +
                " | selected=" + SelectedSnapshots +
                " | vertices=" + SelectedVertices +
                " | indices=" + SelectedIndices +
                " | elapsed=" + ElapsedMilliseconds.ToString("F1") + "ms" +
                " | summary=" + (Summary ?? "<none>");

            if (!includeAttempts || Attempts.Count == 0)
                return header;

            System.Text.StringBuilder builder = new System.Text.StringBuilder(header);
            builder.AppendLine();
            builder.AppendLine("--- source attempts ---");
            for (int i = 0; i < Attempts.Count; i++)
                builder.AppendLine((i + 1) + ". " + Attempts[i].ToLine());
            return builder.ToString().TrimEnd();
        }
    }
}
