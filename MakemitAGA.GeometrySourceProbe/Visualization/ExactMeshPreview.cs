using System;
using System.Collections.Generic;
using MakemitAGA.GeometrySourceProbe.Model;
using UnityEngine;
using Object = UnityEngine.Object;

namespace MakemitAGA.GeometrySourceProbe.Visualization
{
    internal enum PreviewMode
    {
        Off,
        Source,
        Overlay,
        Extracted
    }

    internal static class ExactMeshPreview
    {
        private sealed class RendererState
        {
            public Renderer Renderer;
            public bool Enabled;
        }

        private static readonly List<RendererState> SourceStates = new List<RendererState>();
        private static readonly List<Mesh> PreviewMeshes = new List<Mesh>();
        private static GameObject _root;
        private static Material _material;
        private static PreviewMode _mode = PreviewMode.Off;

        public static PreviewMode Mode => _mode;
        public static int PreviewObjectCount => _root == null ? 0 : _root.transform.childCount;

        public static bool Build(
            GameObject target,
            IReadOnlyList<MeshSnapshot> snapshots,
            out string error)
        {
            error = null;
            Clear();

            if (target == null)
            {
                error = "target-null";
                return false;
            }
            if (snapshots == null || snapshots.Count == 0)
            {
                error = "没有可预览的已选择网格。";
                return false;
            }

            CaptureSourceStates(target);
            _material = CreateMaterial();
            if (_material == null)
            {
                error = "无法创建预览材质。";
                RestoreSourceRenderers();
                return false;
            }

            _root = new GameObject("GSP_ExactSourcePreviewRoot");
            _root.transform.position = Vector3.zero;
            _root.transform.rotation = Quaternion.identity;
            _root.transform.localScale = Vector3.one;

            int created = 0;
            for (int i = 0; i < snapshots.Count; i++)
            {
                MeshSnapshot snapshot = snapshots[i];
                if (snapshot == null)
                    continue;

                Mesh mesh = snapshot.BuildUnityMesh(
                    "GSP_Exact_" + i + "_" + Sanitize(snapshot.MeshName));
                if (mesh == null)
                    continue;

                GameObject child = new GameObject(
                    "GSP_Preview_" + i + "_" + Sanitize(snapshot.SourceKind));
                child.transform.SetParent(_root.transform, false);
                child.transform.localPosition = Vector3.zero;
                child.transform.localRotation = Quaternion.identity;
                child.transform.localScale = Vector3.one;

                MeshFilter filter = child.AddComponent<MeshFilter>();
                MeshRenderer renderer = child.AddComponent<MeshRenderer>();
                filter.sharedMesh = mesh;
                renderer.sharedMaterial = _material;
                renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                renderer.receiveShadows = false;

                PreviewMeshes.Add(mesh);
                created++;
            }

            if (created == 0)
            {
                error = "快照存在，但没有创建出 Unity Mesh。";
                Clear();
                return false;
            }

            SetMode(PreviewMode.Overlay);
            return true;
        }

        public static bool TryParseMode(string text, out PreviewMode mode)
        {
            string normalized = (text ?? string.Empty).Trim().ToLowerInvariant();
            switch (normalized)
            {
                case "on":
                case "overlay": mode = PreviewMode.Overlay; return true;
                case "extracted": mode = PreviewMode.Extracted; return true;
                case "source": mode = PreviewMode.Source; return true;
                case "off": mode = PreviewMode.Off; return true;
                default: mode = PreviewMode.Off; return false;
            }
        }

        public static void SetMode(PreviewMode mode)
        {
            _mode = mode;
            bool showPreview = mode == PreviewMode.Overlay || mode == PreviewMode.Extracted;
            bool showSource = mode != PreviewMode.Extracted;

            if (_root != null)
                _root.SetActive(showPreview);

            for (int i = 0; i < SourceStates.Count; i++)
            {
                RendererState state = SourceStates[i];
                if (state?.Renderer == null)
                    continue;
                state.Renderer.enabled = showSource ? state.Enabled : false;
            }
        }

        public static void Clear()
        {
            RestoreSourceRenderers();

            if (_root != null)
            {
                Object.Destroy(_root);
                _root = null;
            }

            for (int i = 0; i < PreviewMeshes.Count; i++)
            {
                if (PreviewMeshes[i] != null)
                    Object.Destroy(PreviewMeshes[i]);
            }
            PreviewMeshes.Clear();

            if (_material != null)
            {
                Object.Destroy(_material);
                _material = null;
            }

            SourceStates.Clear();
            _mode = PreviewMode.Off;
        }

        private static void CaptureSourceStates(GameObject target)
        {
            SourceStates.Clear();
            Renderer[] renderers;
            try { renderers = target.GetComponentsInChildren<Renderer>(true); }
            catch { renderers = Array.Empty<Renderer>(); }

            for (int i = 0; i < renderers.Length; i++)
            {
                Renderer renderer = renderers[i];
                if (renderer == null || renderer.gameObject == null)
                    continue;
                if ((renderer.gameObject.name ?? string.Empty).StartsWith("GSP_", StringComparison.Ordinal))
                    continue;

                SourceStates.Add(new RendererState
                {
                    Renderer = renderer,
                    Enabled = renderer.enabled
                });
            }
        }

        private static void RestoreSourceRenderers()
        {
            for (int i = 0; i < SourceStates.Count; i++)
            {
                RendererState state = SourceStates[i];
                if (state?.Renderer != null)
                    state.Renderer.enabled = state.Enabled;
            }
        }

        private static Material CreateMaterial()
        {
            Shader shader = Shader.Find("Legacy Shaders/Transparent/Diffuse");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                return null;

            Material material = new Material(shader)
            {
                name = "GSP_ExactSourcePreviewMaterial",
                color = new Color(0.05f, 0.90f, 1.00f, 0.48f)
            };

            try
            {
                if (material.HasProperty("_Color"))
                    material.SetColor("_Color", new Color(0.05f, 0.90f, 1.00f, 0.48f));
                if (material.HasProperty("_ZWrite"))
                    material.SetInt("_ZWrite", 0);
                material.renderQueue = 3000;
            }
            catch { }

            return material;
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
