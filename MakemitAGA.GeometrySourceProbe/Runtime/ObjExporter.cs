using System;
using System.Globalization;
using System.IO;
using System.Text;
using BepInEx;
using MakemitAGA.GeometrySourceProbe.Model;

namespace MakemitAGA.GeometrySourceProbe.Runtime
{
    internal static class ObjExporter
    {
        public static string Export(GeometryProbeReport report)
        {
            if (report == null || report.Snapshots.Count == 0)
                return "<color=yellow>当前没有可导出的已选择网格。</color>";

            string directory = Path.Combine(
                Paths.ConfigPath,
                "MakemitAGA.GeometrySourceProbe",
                "Exports");
            Directory.CreateDirectory(directory);

            string fileName =
                Sanitize(report.TargetName) + "_" +
                DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture) +
                ".obj";
            string path = Path.Combine(directory, fileName);

            using StreamWriter writer = new StreamWriter(path, false, new UTF8Encoding(false));
            writer.WriteLine("# MakemitAGA Geometry Source Probe 0.1.9");
            writer.WriteLine("# target: " + report.TargetPath);
            writer.WriteLine("# vertices are already in Unity world coordinates");

            int vertexBase = 1;
            for (int s = 0; s < report.Snapshots.Count; s++)
            {
                MeshSnapshot snapshot = report.Snapshots[s];
                writer.WriteLine();
                writer.WriteLine("o " + Sanitize(snapshot.ComponentPath + "_" + snapshot.SourceKind));
                writer.WriteLine("# source=" + snapshot.SourceKind + " exactness=" + snapshot.Exactness);

                for (int i = 0; i < snapshot.WorldVertices.Length; i++)
                {
                    UnityEngine.Vector3 v = snapshot.WorldVertices[i];
                    writer.WriteLine(string.Format(
                        CultureInfo.InvariantCulture,
                        "v {0:R} {1:R} {2:R}",
                        v.x,
                        v.y,
                        v.z));
                }

                for (int sub = 0; sub < snapshot.TriangleSubmeshes.Count; sub++)
                {
                    writer.WriteLine("g submesh_" + sub);
                    int[] indices = snapshot.TriangleSubmeshes[sub];
                    for (int i = 0; i + 2 < indices.Length; i += 3)
                    {
                        writer.WriteLine(
                            "f " + (vertexBase + indices[i]) + " " +
                            (vertexBase + indices[i + 1]) + " " +
                            (vertexBase + indices[i + 2]));
                    }
                }

                vertexBase += snapshot.WorldVertices.Length;
            }

            return
                "<color=#66ddff>GSP OBJ 已导出：</color> " + path;
        }

        private static string Sanitize(string value)
        {
            if (string.IsNullOrWhiteSpace(value))
                return "Unnamed";

            char[] chars = value.ToCharArray();
            for (int i = 0; i < chars.Length; i++)
            {
                char c = chars[i];
                if (!(char.IsLetterOrDigit(c) || c == '_' || c == '-'))
                    chars[i] = '_';
            }
            return new string(chars);
        }
    }
}
