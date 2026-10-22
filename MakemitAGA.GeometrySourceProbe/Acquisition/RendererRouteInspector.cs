using System;
using System.Diagnostics;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// 读取 MeshRenderer 到静态批处理 Combined Mesh 的路由元数据。
    /// 不调用任何 GPU API。
    /// </summary>
    internal static class RendererRouteInspector
    {
        public static RendererRouteReport Inspect(
            GameObject target,
            int exactMatches,
            float selectedDistance,
            string referenceName,
            ManualLogSource logger)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();
            RendererRouteReport report = new RendererRouteReport
            {
                TargetName = target?.name,
                TargetPath = TargetFinder.GetPath(target?.transform),
                ExactMatches = exactMatches,
                SelectedDistance = selectedDistance,
                ReferenceName = referenceName
            };

            if (target == null)
            {
                report.Completed = false;
                report.Summary = "target-null";
                return report;
            }

            MeshRenderer[] renderers = target.GetComponentsInChildren<MeshRenderer>(true);
            logger?.LogInfo(
                "[GSP] Renderer route begin | target=" + report.TargetName +
                " | path=" + report.TargetPath +
                " | meshRenderers=" + renderers.Length +
                " | gpuCalls=disabled");

            for (int i = 0; i < renderers.Length; i++)
            {
                MeshRenderer renderer = renderers[i];
                MeshFilter filter = renderer != null ? renderer.GetComponent<MeshFilter>() : null;
                Mesh mesh = filter != null ? filter.sharedMesh : null;

                int materialCount = 0;
                try
                {
                    var materials = renderer.sharedMaterials;
                    materialCount = materials != null ? materials.Length : 0;
                }
                catch { }

                int start = Safe(() => renderer.subMeshStartIndex, 0);
                int meshSubCount = mesh != null ? Safe(() => mesh.subMeshCount, 0) : 0;
                int routedCount = materialCount;
                if (start < 0)
                    start = 0;
                if (routedCount < 0)
                    routedCount = 0;
                if (start > meshSubCount)
                    routedCount = 0;
                else if (start + routedCount > meshSubCount)
                    routedCount = meshSubCount - start;

                RendererRouteEntry entry = new RendererRouteEntry
                {
                    RendererPath = TargetFinder.GetPath(renderer?.transform),
                    RendererType = renderer?.GetType().Name,
                    Enabled = Safe(() => renderer.enabled, false),
                    Active = renderer != null && renderer.gameObject.activeInHierarchy,
                    IsPartOfStaticBatch = Safe(() => renderer.isPartOfStaticBatch, false),
                    SubMeshStartIndex = Safe(() => renderer.subMeshStartIndex, 0),
                    MaterialCount = materialCount,
                    RoutedSubMeshStart = start,
                    RoutedSubMeshCount = routedCount,
                    MeshName = mesh != null ? Safe(() => mesh.name, "<metadata-error>") : "<none>",
                    MeshSubMeshCount = meshSubCount,
                    MeshVertexCount = mesh != null ? Safe(() => mesh.vertexCount, 0) : 0,
                    RendererBounds = FormatBounds(Safe(() => renderer.bounds, default(Bounds))),
                    MeshBounds = mesh != null ? FormatBounds(Safe(() => mesh.bounds, default(Bounds))) : "<none>",
                    Detail = mesh == null
                        ? "MeshRenderer has no MeshFilter/sharedMesh"
                        : (Safe(() => renderer.isPartOfStaticBatch, false)
                            ? "static-batch route; only routed submesh interval belongs to this renderer"
                            : "not statically batched; renderer normally starts at submesh zero")
                };

                report.Entries.Add(entry);
                logger?.LogInfo("[GSP] Renderer route entry | " + entry.ToText(i + 1));
            }

            stopwatch.Stop();
            report.ElapsedMilliseconds = (float)stopwatch.Elapsed.TotalMilliseconds;
            report.Completed = true;
            report.Summary = renderers.Length == 0
                ? "no MeshRenderer found under target"
                : "renderer route metadata captured; no GPU buffer handle or payload was requested";
            logger?.LogInfo("[GSP] Renderer route result | " + report.ToText(false));
            return report;
        }

        private static string FormatBounds(Bounds bounds)
        {
            return
                "center=(" + bounds.center.x.ToString("F4") + "," +
                bounds.center.y.ToString("F4") + "," +
                bounds.center.z.ToString("F4") + ")/size=(" +
                bounds.size.x.ToString("F4") + "," +
                bounds.size.y.ToString("F4") + "," +
                bounds.size.z.ToString("F4") + ")";
        }

        private static T Safe<T>(Func<T> getter, T fallback)
        {
            try { return getter(); }
            catch { return fallback; }
        }
    }
}
