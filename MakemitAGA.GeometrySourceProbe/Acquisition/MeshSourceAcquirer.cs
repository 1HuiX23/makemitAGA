using System;
using System.Collections.Generic;
using System.Diagnostics;
using BepInEx.Logging;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Acquisition
{
    /// <summary>
    /// v0.1.7 安全获取器（保留 v0.1.3 的防崩溃路由）。
    ///
    /// 重要安全边界：
    /// - 绝不修改原 Mesh.vertexBufferTarget / indexBufferTarget。
    /// - 普通构建不调用 GraphicsBuffer、GetVertexBuffer 或 GetIndexBuffer。
    /// - 用户选择的来源在“调用前”完成路由，不再先运行全部路径再事后选择。
    /// - GPU 模式只生成禁用报告；经实机证实 set_vertexBufferTarget 会在
    ///   MiSide Unity 2021.3 IL2CPP 原生层触发 AccessViolation，C# try/catch 无法拦截。
    /// </summary>
    internal static class MeshSourceAcquirer
    {
        private sealed class ComponentCandidate
        {
            public string ComponentType;
            public string ComponentPath;
            public readonly List<(SourceAttemptReport report, MeshSnapshot snapshot)> Results =
                new List<(SourceAttemptReport report, MeshSnapshot snapshot)>();
        }

        public static GeometryInventoryReport Inventory(
            GameObject target,
            int exactMatches,
            float selectedDistance,
            string referenceName,
            ManualLogSource logger)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();
            GeometryInventoryReport report = new GeometryInventoryReport
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

            MeshFilter[] filters = SafeGetComponentsInChildren<MeshFilter>(target);
            SkinnedMeshRenderer[] skinned = SafeGetComponentsInChildren<SkinnedMeshRenderer>(target);
            MeshCollider[] colliders = SafeGetComponentsInChildren<MeshCollider>(target);
            Renderer[] renderers = SafeGetComponentsInChildren<Renderer>(target);

            report.MeshFilters = filters.Length;
            report.SkinnedRenderers = skinned.Length;
            report.MeshColliders = colliders.Length;
            report.Renderers = renderers.Length;

            logger?.LogInfo(
                "[GSP] Safe inventory begin | target=" + report.TargetName +
                " | path=" + report.TargetPath +
                " | renderers=" + renderers.Length +
                " | meshFilters=" + filters.Length +
                " | skinned=" + skinned.Length +
                " | meshColliders=" + colliders.Length);

            for (int i = 0; i < filters.Length; i++)
            {
                MeshFilter filter = filters[i];
                if (filter == null)
                    continue;

                MeshRenderer renderer = null;
                try { renderer = filter.GetComponent<MeshRenderer>(); }
                catch { }

                GeometryInventoryEntry entry = BuildInventoryEntry(
                    "MeshFilter",
                    filter.transform,
                    SafeSharedMesh(filter),
                    true,
                    filter.gameObject != null && filter.gameObject.activeInHierarchy,
                    renderer != null,
                    renderer != null && renderer.enabled,
                    false,
                    false,
                    "metadata-only; no vertex/index access");
                report.Entries.Add(entry);
                logger?.LogInfo("[GSP] Inventory entry | " + entry.ToLine());
            }

            for (int i = 0; i < skinned.Length; i++)
            {
                SkinnedMeshRenderer renderer = skinned[i];
                if (renderer == null)
                    continue;

                GeometryInventoryEntry entry = BuildInventoryEntry(
                    "SkinnedMeshRenderer",
                    renderer.transform,
                    SafeSharedMesh(renderer),
                    renderer.enabled,
                    renderer.gameObject != null && renderer.gameObject.activeInHierarchy,
                    true,
                    renderer.enabled,
                    false,
                    false,
                    "metadata-only; BakeMesh not called");
                report.Entries.Add(entry);
                logger?.LogInfo("[GSP] Inventory entry | " + entry.ToLine());
            }

            for (int i = 0; i < colliders.Length; i++)
            {
                MeshCollider collider = colliders[i];
                if (collider == null)
                    continue;

                GeometryInventoryEntry entry = BuildInventoryEntry(
                    "MeshCollider",
                    collider.transform,
                    SafeSharedMesh(collider),
                    collider.enabled,
                    collider.gameObject != null && collider.gameObject.activeInHierarchy,
                    false,
                    false,
                    SafeConvex(collider),
                    collider.isTrigger,
                    "metadata-only; collision source may be simplified");
                report.Entries.Add(entry);
                logger?.LogInfo("[GSP] Inventory entry | " + entry.ToLine());
            }

            stopwatch.Stop();
            report.ElapsedMilliseconds = (float)stopwatch.Elapsed.TotalMilliseconds;
            report.Completed = true;
            report.Summary =
                "safe component and mesh metadata captured; no geometry extraction API was invoked";
            logger?.LogInfo("[GSP] Safe inventory result | " + report.ToText(false));
            return report;
        }

        public static GeometryProbeReport Probe(
            GameObject target,
            int exactMatches,
            float selectedDistance,
            string referenceName,
            ProbePreference preference,
            ManualLogSource logger)
        {
            Stopwatch stopwatch = Stopwatch.StartNew();
            GeometryProbeReport report = new GeometryProbeReport
            {
                TargetName = target?.name,
                TargetPath = TargetFinder.GetPath(target?.transform),
                ReferenceName = referenceName,
                ExactMatches = exactMatches,
                SelectedDistance = selectedDistance,
                Preference = preference
            };

            if (target == null)
            {
                report.Completed = false;
                report.Summary = "target-null";
                return report;
            }

            MeshFilter[] filters = SafeGetComponentsInChildren<MeshFilter>(target);
            SkinnedMeshRenderer[] skinned = SafeGetComponentsInChildren<SkinnedMeshRenderer>(target);
            MeshCollider[] meshColliders = SafeGetComponentsInChildren<MeshCollider>(target);

            report.MeshFilters = filters.Length;
            report.SkinnedRenderers = skinned.Length;
            report.MeshColliders = meshColliders.Length;

            logger?.LogInfo(
                "[GSP] Routed probe begin | target=" + report.TargetName +
                " | path=" + report.TargetPath +
                " | preference=" + preference +
                " | meshFilters=" + filters.Length +
                " | skinned=" + skinned.Length +
                " | meshColliders=" + meshColliders.Length +
                " | gpuNativeCalls=disabled");

            List<ComponentCandidate> renderCandidates = new List<ComponentCandidate>();
            List<ComponentCandidate> colliderCandidates = new List<ComponentCandidate>();

            bool inspectRenderSources = preference != ProbePreference.Collider;
            if (inspectRenderSources)
            {
                for (int i = 0; i < filters.Length; i++)
                {
                    MeshFilter filter = filters[i];
                    if (filter == null || filter.sharedMesh == null)
                        continue;

                    ComponentCandidate candidate = NewCandidate(
                        "MeshFilter",
                        filter.transform,
                        logger,
                        filter.sharedMesh);
                    ProbeStaticMesh(
                        filter.sharedMesh,
                        filter.transform,
                        candidate,
                        "render-static",
                        preference,
                        logger);
                    renderCandidates.Add(candidate);
                }

                for (int i = 0; i < skinned.Length; i++)
                {
                    SkinnedMeshRenderer renderer = skinned[i];
                    if (renderer == null)
                        continue;

                    ComponentCandidate candidate = NewCandidate(
                        "SkinnedMeshRenderer",
                        renderer.transform,
                        logger,
                        renderer.sharedMesh);
                    ProbeSkinned(renderer, candidate, preference, logger);
                    renderCandidates.Add(candidate);
                }
            }

            for (int i = 0; i < renderCandidates.Count; i++)
                SelectForComponent(renderCandidates[i], preference, true, report);

            // Collider 只在显式 collider 模式，或 auto 没有取得任何视觉快照时运行。
            // 这样既避免无意义的额外 API 调用，也不会把简化碰撞网格误认为视觉原网格。
            bool probeColliders =
                preference == ProbePreference.Collider ||
                (preference == ProbePreference.Auto && report.Snapshots.Count == 0);

            if (probeColliders)
            {
                for (int i = 0; i < meshColliders.Length; i++)
                {
                    MeshCollider collider = meshColliders[i];
                    if (collider == null || collider.sharedMesh == null)
                        continue;

                    ComponentCandidate candidate = NewCandidate(
                        "MeshCollider",
                        collider.transform,
                        logger,
                        collider.sharedMesh);
                    ProbeColliderMesh(collider, candidate, preference, logger);
                    colliderCandidates.Add(candidate);
                }

                for (int i = 0; i < colliderCandidates.Count; i++)
                    SelectForComponent(colliderCandidates[i], preference, true, report);
            }

            for (int i = 0; i < renderCandidates.Count; i++)
                AppendAttempts(renderCandidates[i], report);
            for (int i = 0; i < colliderCandidates.Count; i++)
                AppendAttempts(colliderCandidates[i], report);

            report.SelectedSnapshots = report.Snapshots.Count;
            for (int i = 0; i < report.Snapshots.Count; i++)
            {
                report.SelectedVertices += report.Snapshots[i].VertexCount;
                report.SelectedIndices += report.Snapshots[i].IndexCount;
            }

            stopwatch.Stop();
            report.ElapsedMilliseconds = (float)stopwatch.Elapsed.TotalMilliseconds;
            report.Completed = true;

            if (preference == ProbePreference.Gpu)
            {
                report.Summary =
                    "GPU path is intentionally disabled after a confirmed native AccessViolation on Mesh.set_vertexBufferTarget";
            }
            else if (report.SelectedSnapshots > 0)
            {
                report.Summary =
                    "selected safe source snapshots are ready for overlay/extracted preview";
            }
            else
            {
                report.Summary =
                    "no requested safe source produced a complete position-and-triangle snapshot";
            }

            logger?.LogInfo("[GSP] Routed probe acquisition complete | " + report.ToText(false));
            return report;
        }

        private static ComponentCandidate NewCandidate(
            string componentType,
            Transform transform,
            ManualLogSource logger,
            Mesh mesh)
        {
            ComponentCandidate candidate = new ComponentCandidate
            {
                ComponentType = componentType,
                ComponentPath = TargetFinder.GetPath(transform)
            };

            logger?.LogInfo(
                "[GSP] Begin component | type=" + componentType +
                " | path=" + candidate.ComponentPath +
                " | mesh=" + SafeMeshName(mesh) +
                " | readable=" + SafeIsReadable(mesh));
            return candidate;
        }

        private static void ProbeStaticMesh(
            Mesh mesh,
            Transform transform,
            ComponentCandidate candidate,
            string role,
            ProbePreference preference,
            ManualLogSource logger)
        {
            switch (preference)
            {
                case ProbePreference.Auto:
                case ProbePreference.Cpu:
                    AddAttempt(candidate, logger, "CPU", () => TryAcquireCpu(
                        mesh,
                        transform,
                        candidate.ComponentType,
                        candidate.ComponentPath,
                        "CPU",
                        role + "; direct Mesh API"));
                    if (preference == ProbePreference.Auto)
                    {
                        AddDisabledMeshData(candidate, mesh, "MeshData", role);
                        AddDisabledGpu(candidate, mesh, "GPU", role);
                    }
                    break;

                case ProbePreference.MeshData:
                    AddDisabledMeshData(candidate, mesh, "MeshData", role);
                    break;

                case ProbePreference.Gpu:
                    AddDisabledGpu(candidate, mesh, "GPU", role);
                    break;

                case ProbePreference.Bake:
                    AddNotApplicable(
                        candidate,
                        mesh,
                        "Bake",
                        "Bake applies only to SkinnedMeshRenderer; no API invoked");
                    break;
            }
        }

        private static void ProbeSkinned(
            SkinnedMeshRenderer renderer,
            ComponentCandidate candidate,
            ProbePreference preference,
            ManualLogSource logger)
        {
            Mesh sharedMesh = renderer.sharedMesh;
            switch (preference)
            {
                case ProbePreference.Auto:
                {
                    AddAttempt(candidate, logger, "Bake", () => TryAcquireBakedSkinned(
                        renderer,
                        candidate.ComponentType,
                        candidate.ComponentPath));

                    bool bakeSucceeded = HasSuccessfulSnapshot(candidate, "Bake");
                    if (!bakeSucceeded && sharedMesh != null)
                    {
                        AddAttempt(candidate, logger, "CPU", () => TryAcquireCpu(
                            sharedMesh,
                            renderer.transform,
                            candidate.ComponentType,
                            candidate.ComponentPath,
                            "CPU",
                            "skinned-bind-pose; direct Mesh API fallback"));
                    }
                    else if (bakeSucceeded)
                    {
                        AddNotApplicable(
                            candidate,
                            sharedMesh,
                            "CPU",
                            "not run: BakeMesh already captured the current deformed pose");
                    }

                    AddDisabledMeshData(candidate, sharedMesh, "MeshData", "skinned-bind-pose");
                    AddDisabledGpu(candidate, sharedMesh, "GPU", "skinned-render-mesh");
                    break;
                }

                case ProbePreference.Bake:
                    AddAttempt(candidate, logger, "Bake", () => TryAcquireBakedSkinned(
                        renderer,
                        candidate.ComponentType,
                        candidate.ComponentPath));
                    break;

                case ProbePreference.Cpu:
                    if (sharedMesh != null)
                    {
                        AddAttempt(candidate, logger, "CPU", () => TryAcquireCpu(
                            sharedMesh,
                            renderer.transform,
                            candidate.ComponentType,
                            candidate.ComponentPath,
                            "CPU",
                            "skinned-bind-pose; direct Mesh API"));
                    }
                    else
                    {
                        AddNotApplicable(candidate, null, "CPU", "sharedMesh is null");
                    }
                    break;

                case ProbePreference.MeshData:
                    AddDisabledMeshData(candidate, sharedMesh, "MeshData", "skinned-bind-pose");
                    break;

                case ProbePreference.Gpu:
                    AddDisabledGpu(candidate, sharedMesh, "GPU", "skinned-render-mesh");
                    break;
            }
        }

        private static void ProbeColliderMesh(
            MeshCollider collider,
            ComponentCandidate candidate,
            ProbePreference preference,
            ManualLogSource logger)
        {
            Mesh mesh = collider.sharedMesh;
            if (preference == ProbePreference.Collider || preference == ProbePreference.Auto)
            {
                AddAttempt(candidate, logger, "Collider", () => TryAcquireCpu(
                    mesh,
                    collider.transform,
                    candidate.ComponentType,
                    candidate.ComponentPath,
                    "Collider",
                    "existing MeshCollider.sharedMesh; collision geometry only"));
            }
        }

        private static void AddAttempt(
            ComponentCandidate candidate,
            ManualLogSource logger,
            string sourceKind,
            Func<(SourceAttemptReport report, MeshSnapshot snapshot)> action)
        {
            logger?.LogInfo(
                "[GSP] Begin source attempt | component=" + candidate.ComponentType +
                " | path=" + candidate.ComponentPath +
                " | source=" + sourceKind);

            (SourceAttemptReport report, MeshSnapshot snapshot) result;
            try
            {
                result = action();
            }
            catch (Exception e)
            {
                SourceAttemptReport failed = new SourceAttemptReport
                {
                    ComponentType = candidate.ComponentType,
                    ComponentPath = candidate.ComponentPath,
                    SourceKind = sourceKind,
                    Success = false,
                    Detail = "managed wrapper failure; " + e.GetType().Name + " / " + e.Message
                };
                result = (failed, null);
            }

            candidate.Results.Add(result);
            logger?.LogInfo("[GSP] End source attempt | " + result.report.ToLine());
        }

        private static void AddDisabledMeshData(
            ComponentCandidate candidate,
            Mesh mesh,
            string sourceKind,
            string role)
        {
            SourceAttemptReport attempt = NewAttempt(
                candidate.ComponentType,
                candidate.ComponentPath,
                sourceKind,
                mesh,
                role +
                "; skipped: this game's generated IL2CPP interop does not expose a usable MeshData API");
            candidate.Results.Add((attempt, null));
        }

        private static void AddDisabledGpu(
            ComponentCandidate candidate,
            Mesh mesh,
            string sourceKind,
            string role)
        {
            SourceAttemptReport attempt = NewAttempt(
                candidate.ComponentType,
                candidate.ComponentPath,
                sourceKind,
                mesh,
                role +
                "; disabled: native AccessViolation was confirmed on Mesh.set_vertexBufferTarget; no GPU API was invoked");
            candidate.Results.Add((attempt, null));
        }

        private static void AddNotApplicable(
            ComponentCandidate candidate,
            Mesh mesh,
            string sourceKind,
            string detail)
        {
            SourceAttemptReport attempt = NewAttempt(
                candidate.ComponentType,
                candidate.ComponentPath,
                sourceKind,
                mesh,
                detail);
            candidate.Results.Add((attempt, null));
        }

        private static bool HasSuccessfulSnapshot(
            ComponentCandidate candidate,
            string sourceKind)
        {
            for (int i = 0; i < candidate.Results.Count; i++)
            {
                if (candidate.Results[i].snapshot != null &&
                    candidate.Results[i].report.Success &&
                    string.Equals(
                        candidate.Results[i].report.SourceKind,
                        sourceKind,
                        StringComparison.OrdinalIgnoreCase))
                {
                    return true;
                }
            }
            return false;
        }

        private static (SourceAttemptReport report, MeshSnapshot snapshot) TryAcquireBakedSkinned(
            SkinnedMeshRenderer renderer,
            string componentType,
            string componentPath)
        {
            SourceAttemptReport attempt = NewAttempt(
                componentType,
                componentPath,
                "Bake",
                renderer?.sharedMesh,
                "current deformed SkinnedMeshRenderer snapshot");

            Mesh baked = null;
            try
            {
                if (renderer == null)
                    throw new InvalidOperationException("renderer is null");

                baked = new Mesh();
                baked.name = "GSP_Baked_" + (renderer.sharedMesh?.name ?? renderer.name);

                // false：保留 renderer 本地坐标；随后统一由 localToWorldMatrix 转到世界坐标。
                renderer.BakeMesh(baked, false);

                MeshSnapshot snapshot = BuildSnapshotFromReadableMesh(
                    baked,
                    renderer.transform.localToWorldMatrix,
                    attempt,
                    "Bake",
                    "current-skinned-pose-exact");

                attempt.Success = snapshot != null;
                attempt.Detail = attempt.Success
                    ? "BakeMesh succeeded; current skinned pose captured"
                    : "BakeMesh returned no complete triangle snapshot";
                return (attempt, snapshot);
            }
            catch (Exception e)
            {
                attempt.Success = false;
                attempt.Detail = e.GetType().Name + " / " + e.Message;
                return (attempt, null);
            }
            finally
            {
                if (baked != null)
                    Object.Destroy(baked);
            }
        }

        private static (SourceAttemptReport report, MeshSnapshot snapshot) TryAcquireCpu(
            Mesh mesh,
            Transform transform,
            string componentType,
            string componentPath,
            string sourceKind,
            string detailPrefix)
        {
            SourceAttemptReport attempt = NewAttempt(
                componentType,
                componentPath,
                sourceKind,
                mesh,
                detailPrefix);

            try
            {
                if (mesh == null)
                    throw new InvalidOperationException("mesh is null");
                if (transform == null)
                    throw new InvalidOperationException("transform is null");

                MeshSnapshot snapshot = BuildSnapshotFromReadableMesh(
                    mesh,
                    transform.localToWorldMatrix,
                    attempt,
                    sourceKind,
                    ResolveExactness(componentType, detailPrefix, "static-render-mesh-exact"));

                attempt.Success = snapshot != null;
                attempt.Detail = attempt.Success
                    ? detailPrefix + "; complete CPU snapshot"
                    : detailPrefix + "; no complete triangle snapshot";
                return (attempt, snapshot);
            }
            catch (Exception e)
            {
                attempt.Success = false;
                attempt.Detail = detailPrefix + "; " + e.GetType().Name + " / " + e.Message;
                return (attempt, null);
            }
        }

        private static MeshSnapshot BuildSnapshotFromReadableMesh(
            Mesh mesh,
            Matrix4x4 localToWorld,
            SourceAttemptReport attempt,
            string sourceKind,
            string exactness)
        {
            if (mesh == null)
                return null;

            Vector3[] localVertices = mesh.vertices;
            if (localVertices == null || localVertices.Length == 0)
                return null;

            Vector3[] worldVertices = TransformVertices(localVertices, localToWorld);
            List<int[]> submeshes = new List<int[]>();

            int subMeshCount = mesh.subMeshCount;
            for (int s = 0; s < subMeshCount; s++)
            {
                MeshTopology topology = mesh.GetTopology(s);
                if (topology != MeshTopology.Triangles)
                    continue;

                int[] indices = mesh.GetIndices(s, true);
                if (indices != null && indices.Length >= 3)
                    submeshes.Add(indices);
            }

            if (submeshes.Count == 0)
                return null;

            MeshSnapshot snapshot = new MeshSnapshot
            {
                SourceKind = sourceKind,
                ComponentType = attempt.ComponentType,
                ComponentPath = attempt.ComponentPath,
                MeshName = mesh.name,
                Exactness = exactness,
                WorldVertices = worldVertices,
                WorldBounds = MeshSnapshot.CalculateBounds(worldVertices)
            };
            snapshot.TriangleSubmeshes.AddRange(submeshes);

            attempt.Vertices = snapshot.VertexCount;
            attempt.Indices = snapshot.IndexCount;
            attempt.Submeshes = snapshot.TriangleSubmeshes.Count;
            attempt.Exactness = exactness;
            return snapshot;
        }

        private static string ResolveExactness(
            string componentType,
            string detailPrefix,
            string staticExactness)
        {
            if (componentType == "MeshCollider")
                return "collision-source-not-render-exact";

            if ((detailPrefix ?? string.Empty).StartsWith(
                "skinned-bind-pose",
                StringComparison.OrdinalIgnoreCase))
            {
                return "skinned-shared-mesh-bind-pose-not-current-pose";
            }

            return staticExactness;
        }

        private static void SelectForComponent(
            ComponentCandidate candidate,
            ProbePreference preference,
            bool allowSelection,
            GeometryProbeReport report)
        {
            if (candidate == null || !allowSelection)
                return;

            int selectedIndex = FindPreferredResult(
                candidate.Results,
                preference,
                candidate.ComponentType);
            if (selectedIndex < 0)
                return;

            candidate.Results[selectedIndex].report.Selected = true;
            report.Snapshots.Add(candidate.Results[selectedIndex].snapshot);
        }

        private static int FindPreferredResult(
            List<(SourceAttemptReport report, MeshSnapshot snapshot)> results,
            ProbePreference preference,
            string componentType)
        {
            string[] priority;
            switch (preference)
            {
                case ProbePreference.Cpu:
                    priority = new[] { "CPU" };
                    break;
                case ProbePreference.MeshData:
                    priority = componentType == "MeshCollider"
                        ? new[] { "ColliderMeshData" }
                        : new[] { "MeshData" };
                    break;
                case ProbePreference.Bake:
                    priority = new[] { "Bake" };
                    break;
                case ProbePreference.Gpu:
                    priority = componentType == "MeshCollider"
                        ? new[] { "ColliderGPU" }
                        : new[] { "GPU" };
                    break;
                case ProbePreference.Collider:
                    priority = new[] { "Collider" };
                    break;
                default:
                    priority = componentType == "SkinnedMeshRenderer"
                        ? new[] { "Bake", "CPU" }
                        : componentType == "MeshCollider"
                            ? new[] { "Collider" }
                            : new[] { "CPU" };
                    break;
            }

            for (int p = 0; p < priority.Length; p++)
            {
                for (int i = 0; i < results.Count; i++)
                {
                    if (results[i].snapshot != null &&
                        results[i].report.Success &&
                        string.Equals(
                            results[i].report.SourceKind,
                            priority[p],
                            StringComparison.OrdinalIgnoreCase))
                    {
                        return i;
                    }
                }
            }
            return -1;
        }

        private static void AppendAttempts(
            ComponentCandidate candidate,
            GeometryProbeReport report)
        {
            for (int i = 0; i < candidate.Results.Count; i++)
                report.Attempts.Add(candidate.Results[i].report);
        }

        private static SourceAttemptReport NewAttempt(
            string componentType,
            string componentPath,
            string sourceKind,
            Mesh mesh,
            string detail)
        {
            return new SourceAttemptReport
            {
                ComponentType = componentType,
                ComponentPath = componentPath,
                SourceKind = sourceKind,
                MeshName = SafeMeshName(mesh),
                IsReadable = SafeIsReadable(mesh),
                Success = false,
                Selected = false,
                Vertices = SafeVertexCount(mesh),
                Submeshes = SafeSubMeshCount(mesh),
                Detail = detail
            };
        }

        private static GeometryInventoryEntry BuildInventoryEntry(
            string componentType,
            Transform transform,
            Mesh mesh,
            bool componentEnabled,
            bool active,
            bool rendererPresent,
            bool rendererEnabled,
            bool convex,
            bool trigger,
            string detail)
        {
            return new GeometryInventoryEntry
            {
                ComponentType = componentType,
                ComponentPath = TargetFinder.GetPath(transform),
                MeshName = SafeMeshName(mesh),
                MeshPresent = mesh != null,
                IsReadable = SafeIsReadable(mesh),
                VertexCount = SafeVertexCount(mesh),
                SubMeshCount = SafeSubMeshCount(mesh),
                IndexFormat = SafeIndexFormat(mesh),
                Bounds = SafeBounds(mesh),
                ComponentEnabled = componentEnabled,
                GameObjectActive = active,
                RendererPresent = rendererPresent,
                RendererEnabled = rendererEnabled,
                ColliderConvex = convex,
                ColliderTrigger = trigger,
                Detail = detail
            };
        }

        private static Mesh SafeSharedMesh(MeshFilter filter)
        {
            try { return filter?.sharedMesh; }
            catch { return null; }
        }

        private static Mesh SafeSharedMesh(SkinnedMeshRenderer renderer)
        {
            try { return renderer?.sharedMesh; }
            catch { return null; }
        }

        private static Mesh SafeSharedMesh(MeshCollider collider)
        {
            try { return collider?.sharedMesh; }
            catch { return null; }
        }

        private static bool SafeConvex(MeshCollider collider)
        {
            try { return collider != null && collider.convex; }
            catch { return false; }
        }

        private static string SafeMeshName(Mesh mesh)
        {
            try { return mesh == null ? null : mesh.name; }
            catch { return "<metadata-error>"; }
        }

        private static bool SafeIsReadable(Mesh mesh)
        {
            try { return mesh != null && mesh.isReadable; }
            catch { return false; }
        }

        private static int SafeVertexCount(Mesh mesh)
        {
            try { return mesh == null ? 0 : mesh.vertexCount; }
            catch { return 0; }
        }

        private static int SafeSubMeshCount(Mesh mesh)
        {
            try { return mesh == null ? 0 : mesh.subMeshCount; }
            catch { return 0; }
        }

        private static string SafeIndexFormat(Mesh mesh)
        {
            try { return mesh == null ? null : mesh.indexFormat.ToString(); }
            catch { return "<metadata-error>"; }
        }

        private static string SafeBounds(Mesh mesh)
        {
            try
            {
                if (mesh == null)
                    return null;
                Bounds bounds = mesh.bounds;
                return
                    "center=" + FormatVector(bounds.center) +
                    "/size=" + FormatVector(bounds.size);
            }
            catch
            {
                return "<metadata-error>";
            }
        }

        private static string FormatVector(Vector3 value)
        {
            return
                "(" + value.x.ToString("F4") + "," +
                value.y.ToString("F4") + "," +
                value.z.ToString("F4") + ")";
        }

        private static T[] SafeGetComponentsInChildren<T>(GameObject target)
            where T : Component
        {
            try { return target.GetComponentsInChildren<T>(true) ?? Array.Empty<T>(); }
            catch { return Array.Empty<T>(); }
        }

        private static Vector3[] TransformVertices(
            Vector3[] localVertices,
            Matrix4x4 localToWorld)
        {
            Vector3[] world = new Vector3[localVertices.Length];
            for (int i = 0; i < localVertices.Length; i++)
                world[i] = localToWorld.MultiplyPoint3x4(localVertices[i]);
            return world;
        }
    }
}
