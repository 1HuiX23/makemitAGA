using System;
using System.Collections.Generic;
using System.Diagnostics;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using UnityEngine.Rendering;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// v0.1.7 保留的只读布局与“现有句柄”验证器。
    ///
    /// 永久安全边界：
    /// - 不写 vertexBufferTarget / indexBufferTarget。
    /// - 不调用 GraphicsBuffer.GetData、AsyncGPUReadback、Graphics.CopyBuffer。
    /// - Layout() 不取得任何 GPU buffer 句柄。
    /// - ProbeExistingHandle() 只有用户显式执行 gsp_handle 时才调用一个 getter，
    ///   并立刻 Dispose 返回的包装句柄；不释放 Mesh 自己拥有的底层 buffer。
    /// </summary>
    internal static class MeshLayoutInspector
    {
        private sealed class MeshCandidate
        {
            public string ComponentType;
            public string ComponentPath;
            public Mesh Mesh;
        }

        public static GeometryLayoutReport Layout(
            GameObject target,
            int exactMatches,
            float selectedDistance,
            string referenceName,
            ManualLogSource logger)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();
            GeometryLayoutReport report = new GeometryLayoutReport
            {
                TargetName = target?.name,
                TargetPath = TargetFinder.GetPath(target?.transform),
                ReferenceName = referenceName,
                ExactMatches = exactMatches,
                SelectedDistance = selectedDistance
            };

            if (target == null)
            {
                report.Completed = false;
                report.Summary = "target-null";
                return report;
            }

            List<MeshCandidate> candidates = CollectCandidates(target);
            logger?.LogInfo(
                "[GSP] Read-only layout begin | target=" + report.TargetName +
                " | path=" + report.TargetPath +
                " | candidates=" + candidates.Count +
                " | bufferGetters=disabled | targetSetters=disabled | dataReadback=disabled");

            for (int i = 0; i < candidates.Count; i++)
            {
                MeshLayoutEntry entry = InspectMesh(candidates[i]);
                report.Entries.Add(entry);
                logger?.LogInfo(
                    "[GSP] Layout entry | component=" + entry.ComponentType +
                    " | path=" + entry.ComponentPath +
                    " | mesh=" + entry.MeshName +
                    " | readable=" + entry.IsReadable +
                    " | v=" + entry.VertexCount +
                    " | streams=" + entry.VertexBufferCount +
                    " | sub=" + entry.SubMeshCount +
                    " | vertexTarget=" + entry.VertexBufferTarget +
                    " | indexTarget=" + entry.IndexBufferTarget +
                    " | attributes=" + entry.Attributes.Count +
                    " | errors=" + entry.Errors.Count);

                for (int s = 0; s < entry.VertexStreams.Count; s++)
                    logger?.LogInfo("[GSP] Layout stream | " + entry.VertexStreams[s]);
                for (int a = 0; a < entry.Attributes.Count; a++)
                    logger?.LogInfo("[GSP] Layout attribute | " + entry.Attributes[a]);
                for (int sm = 0; sm < entry.SubMeshes.Count; sm++)
                    logger?.LogInfo("[GSP] Layout submesh | " + entry.SubMeshes[sm]);
                for (int e = 0; e < entry.Errors.Count; e++)
                    logger?.LogWarning("[GSP] Layout metadata error | " + entry.Errors[e]);
            }

            stopwatch.Stop();
            report.ElapsedMilliseconds = (float)stopwatch.Elapsed.TotalMilliseconds;
            report.Completed = true;
            report.Summary =
                candidates.Count == 0
                    ? "no MeshFilter, SkinnedMeshRenderer or MeshCollider mesh was found"
                    : "read-only layout captured; no GPU buffer handle or data was requested";
            logger?.LogInfo("[GSP] Read-only layout result | " + report.ToText(false));
            return report;
        }

        public static BufferHandleProbeReport ProbeExistingHandle(
            GameObject target,
            string requestedBuffer,
            ManualLogSource logger)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();
            BufferHandleProbeReport report = new BufferHandleProbeReport
            {
                TargetName = target?.name,
                TargetPath = TargetFinder.GetPath(target?.transform),
                RequestedBuffer = requestedBuffer
            };

            if (target == null)
            {
                report.Completed = false;
                report.Detail = "target-null";
                return report;
            }

            List<MeshCandidate> candidates = CollectCandidates(target);
            report.CandidateMeshes = candidates.Count;
            if (candidates.Count == 0)
            {
                report.Completed = true;
                report.Success = false;
                report.Detail = "no mesh candidate";
                return report;
            }

            MeshCandidate candidate = candidates[0];
            Mesh mesh = candidate.Mesh;
            report.ComponentType = candidate.ComponentType;
            report.ComponentPath = candidate.ComponentPath;
            report.MeshName = Safe(() => mesh.name, "<metadata-error>");
            report.VertexTargetBefore = Safe(() => mesh.vertexBufferTarget.ToString(), "<getter-error>");
            report.IndexTargetBefore = Safe(() => mesh.indexBufferTarget.ToString(), "<getter-error>");

            string normalized = (requestedBuffer ?? string.Empty).Trim().ToLowerInvariant();
            logger?.LogWarning(
                "[GSP] Existing handle probe begin | target=" + report.TargetName +
                " | component=" + report.ComponentType +
                " | path=" + report.ComponentPath +
                " | mesh=" + report.MeshName +
                " | request=" + normalized +
                " | IMPORTANT=no target setter, no GetData, no readback, one native getter only");

            GraphicsBuffer buffer = null;
            try
            {
                if (normalized == "index")
                {
                    logger?.LogWarning(
                        "[GSP] Native boundary enter | call=Mesh.GetIndexBuffer | mesh=" + report.MeshName);
                    buffer = mesh.GetIndexBuffer();
                    logger?.LogInfo("[GSP] Native boundary returned | call=Mesh.GetIndexBuffer");
                }
                else if (normalized.StartsWith("vertex", StringComparison.Ordinal))
                {
                    string suffix = normalized.Substring("vertex".Length);
                    if (!int.TryParse(suffix, out int stream))
                        throw new ArgumentException("vertex request must be vertex0, vertex1, ...");

                    int streamCount = mesh.vertexBufferCount;
                    if (stream < 0 || stream >= streamCount)
                        throw new ArgumentOutOfRangeException(
                            nameof(requestedBuffer),
                            "stream=" + stream + " but vertexBufferCount=" + streamCount);

                    logger?.LogWarning(
                        "[GSP] Native boundary enter | call=Mesh.GetVertexBuffer(" + stream + ") | mesh=" + report.MeshName);
                    buffer = mesh.GetVertexBuffer(stream);
                    logger?.LogInfo(
                        "[GSP] Native boundary returned | call=Mesh.GetVertexBuffer(" + stream + ")");
                }
                else
                {
                    throw new ArgumentException("request must be vertex0, vertex1, ... or index");
                }

                if (buffer == null)
                    throw new InvalidOperationException("Unity returned a null GraphicsBuffer wrapper");

                report.BufferCount = buffer.count;
                report.BufferStride = buffer.stride;
                report.BufferTarget = buffer.target.ToString();
                report.BufferValid = buffer.IsValid();
                report.Success = report.BufferValid;
                report.Detail = report.Success
                    ? "existing GraphicsBuffer wrapper acquired; payload was not read"
                    : "GraphicsBuffer wrapper returned but IsValid=false";
            }
            catch (Exception e)
            {
                report.Success = false;
                report.Detail = e.GetType().Name + " / " + e.Message;
            }
            finally
            {
                if (buffer != null)
                {
                    try
                    {
                        buffer.Dispose();
                        logger?.LogInfo(
                            "[GSP] Existing handle wrapper disposed | underlying Mesh ownership unchanged");
                    }
                    catch (Exception e)
                    {
                        logger?.LogWarning(
                            "[GSP] Buffer wrapper dispose failed | " + e.GetType().Name + " / " + e.Message);
                    }
                }
            }

            report.VertexTargetAfter = Safe(() => mesh.vertexBufferTarget.ToString(), "<getter-error>");
            report.IndexTargetAfter = Safe(() => mesh.indexBufferTarget.ToString(), "<getter-error>");
            report.TargetsUnchanged =
                string.Equals(report.VertexTargetBefore, report.VertexTargetAfter, StringComparison.Ordinal) &&
                string.Equals(report.IndexTargetBefore, report.IndexTargetAfter, StringComparison.Ordinal);

            stopwatch.Stop();
            report.ElapsedMilliseconds = (float)stopwatch.Elapsed.TotalMilliseconds;
            report.Completed = true;
            logger?.LogInfo("[GSP] Existing handle probe result | " + report.ToText());
            return report;
        }

        private static MeshLayoutEntry InspectMesh(MeshCandidate candidate)
        {
            Mesh mesh = candidate.Mesh;
            MeshLayoutEntry entry = new MeshLayoutEntry
            {
                ComponentType = candidate.ComponentType,
                ComponentPath = candidate.ComponentPath,
                MeshName = Safe(() => mesh.name, "<metadata-error>"),
                IsReadable = Safe(() => mesh.isReadable, false),
                VertexCount = Safe(() => mesh.vertexCount, 0),
                VertexBufferCount = Safe(() => mesh.vertexBufferCount, 0),
                SubMeshCount = Safe(() => mesh.subMeshCount, 0),
                IndexFormat = Safe(() => mesh.indexFormat.ToString(), "<metadata-error>"),
                VertexBufferTarget = Safe(() => mesh.vertexBufferTarget.ToString(), "<getter-error>"),
                IndexBufferTarget = Safe(() => mesh.indexBufferTarget.ToString(), "<getter-error>")
            };

            for (int stream = 0; stream < entry.VertexBufferCount; stream++)
            {
                try
                {
                    int stride = mesh.GetVertexBufferStride(stream);
                    entry.VertexStreams.Add(
                        stream + " | stride=" + stride + " bytes");
                }
                catch (Exception e)
                {
                    entry.Errors.Add(
                        "GetVertexBufferStride(" + stream + ") | " +
                        e.GetType().Name + " / " + e.Message);
                }
            }

            try
            {
                var descriptors = mesh.GetVertexAttributes();
                if (descriptors != null)
                {
                    for (int i = 0; i < descriptors.Length; i++)
                    {
                        VertexAttributeDescriptor descriptor = descriptors[i];
                        int offset = -1;
                        try { offset = mesh.GetVertexAttributeOffset(descriptor.attribute); }
                        catch { }

                        entry.Attributes.Add(
                            i + " | attribute=" + descriptor.attribute +
                            " | format=" + descriptor.format +
                            " | dimension=" + descriptor.dimension +
                            " | stream=" + descriptor.stream +
                            " | offset=" + offset);
                    }
                }
            }
            catch (Exception e)
            {
                entry.Errors.Add(
                    "GetVertexAttributes | " + e.GetType().Name + " / " + e.Message);
            }

            for (int sub = 0; sub < entry.SubMeshCount; sub++)
            {
                try
                {
                    SubMeshDescriptor descriptor = mesh.GetSubMesh(sub);
                    entry.SubMeshes.Add(
                        sub + " | topology=" + descriptor.topology +
                        " | indexStart=" + descriptor.indexStart +
                        " | indexCount=" + descriptor.indexCount +
                        " | baseVertex=" + descriptor.baseVertex +
                        " | firstVertex=" + descriptor.firstVertex +
                        " | vertexCount=" + descriptor.vertexCount +
                        " | boundsCenter=" + FormatVector(descriptor.bounds.center) +
                        " | boundsSize=" + FormatVector(descriptor.bounds.size));
                }
                catch (Exception e)
                {
                    entry.Errors.Add(
                        "GetSubMesh(" + sub + ") | " + e.GetType().Name + " / " + e.Message);
                }
            }

            return entry;
        }

        private static List<MeshCandidate> CollectCandidates(GameObject target)
        {
            List<MeshCandidate> result = new List<MeshCandidate>();

            MeshFilter[] filters = SafeGetComponentsInChildren<MeshFilter>(target);
            for (int i = 0; i < filters.Length; i++)
            {
                MeshFilter component = filters[i];
                Mesh mesh = null;
                try { mesh = component?.sharedMesh; }
                catch { }
                if (component != null && mesh != null)
                {
                    result.Add(new MeshCandidate
                    {
                        ComponentType = "MeshFilter",
                        ComponentPath = TargetFinder.GetPath(component.transform),
                        Mesh = mesh
                    });
                }
            }

            SkinnedMeshRenderer[] skinned = SafeGetComponentsInChildren<SkinnedMeshRenderer>(target);
            for (int i = 0; i < skinned.Length; i++)
            {
                SkinnedMeshRenderer component = skinned[i];
                Mesh mesh = null;
                try { mesh = component?.sharedMesh; }
                catch { }
                if (component != null && mesh != null)
                {
                    result.Add(new MeshCandidate
                    {
                        ComponentType = "SkinnedMeshRenderer",
                        ComponentPath = TargetFinder.GetPath(component.transform),
                        Mesh = mesh
                    });
                }
            }

            MeshCollider[] colliders = SafeGetComponentsInChildren<MeshCollider>(target);
            for (int i = 0; i < colliders.Length; i++)
            {
                MeshCollider component = colliders[i];
                Mesh mesh = null;
                try { mesh = component?.sharedMesh; }
                catch { }
                if (component != null && mesh != null)
                {
                    result.Add(new MeshCandidate
                    {
                        ComponentType = "MeshCollider",
                        ComponentPath = TargetFinder.GetPath(component.transform),
                        Mesh = mesh
                    });
                }
            }

            return result;
        }

        private static T[] SafeGetComponentsInChildren<T>(GameObject target)
            where T : Component
        {
            try { return target.GetComponentsInChildren<T>(true) ?? Array.Empty<T>(); }
            catch { return Array.Empty<T>(); }
        }

        private static TValue Safe<TValue>(Func<TValue> action, TValue fallback)
        {
            try { return action(); }
            catch { return fallback; }
        }

        private static string FormatVector(Vector3 value)
        {
            return
                "(" + value.x.ToString("F4") + "," +
                value.y.ToString("F4") + "," +
                value.z.ToString("F4") + ")";
        }
    }
}
