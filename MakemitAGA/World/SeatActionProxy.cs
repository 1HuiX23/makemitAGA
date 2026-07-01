/*
 * =================================================================================================
 * SeatActionProxy.cs
 * =================================================================================================
 *
 * 作用：生成“动作执行层”的小型连续 MeshCollider 平面。
 *
 * SeatSurfaceAnalysisMesh 提供完整、稀疏且可能包含多个高度岛的环境表示；
 * 米塔真正坐下时不直接依赖整张高分辨率网格，而是在最终紫色座点附近建立一个
 * 小而连续、法线稳定的薄 MeshCollider。这样既保留环境理解能力，也避免动画受到
 * 高度图噪声、三角形接缝和复杂边缘的影响。
 *
 * 生命周期：
 *   - 每次 Seat VLM 成功后创建/替换当前 Action Proxy；
 *   - svt_clear 只隐藏调试 Renderer，不销毁 Collider；
 *   - 新座点或场景切换时才销毁旧 Proxy；
 *   - 默认不可见，debug_svt 开启后显示为紫色半透明薄面。
 * =================================================================================================
 */

using System;
using System.Collections.Generic;
using UnityEngine;

namespace MakemitAGA.World
{
    /// <summary>
    /// 从 VLM 和物理分析层传递给 Mita_sit 的稳定坐姿数据。
    /// Collider 只是执行辅助；真正的数据接口是这个对象。
    /// </summary>
    public sealed class SeatPose
    {
        public GameObject Target;
        public GameObject ActionProxy;

        public Vector3 WorldSeatPoint;
        public Vector3 SurfaceNormal;
        public Vector3 OutwardDirection;
        public Vector3 FloorPoint;
        public Quaternion SeatRotation;

        public float HeightAboveFloor;
        public float SupportWidth;
        public float SupportDepth;
        public float ProxyThickness;

        public bool WasSnapped;
        public bool HeightWarning;
        public float Confidence;
    }

    internal static class SeatActionProxyRuntime
    {
        private const float DefaultWidth = 0.48f;
        private const float DefaultDepth = 0.34f;
        private const float DefaultThickness = 0.08f;

        private static int _serial;
        private static GameObject _currentProxy;
        private static Mesh _currentMesh;
        private static Material _debugMaterial;
        private static bool _debugVisible;

        public static SeatPose CurrentPose { get; private set; }
        public static GameObject CurrentProxy => _currentProxy;

        /// <summary>
        /// 把分析层输出转换成稳定、连续的局部动作平面。
        /// </summary>
        public static bool TryCreateFromSelection(
            SeatSurfaceSelectionResult selection,
            out SeatPose pose,
            out string error)
        {
            pose = null;
            error = null;

            if (selection == null)
            {
                error = "SeatSurfaceSelectionResult is null.";
                return false;
            }

            if (selection.Target == null)
            {
                error = "Selected target is no longer valid.";
                return false;
            }

            Vector3 outward = selection.OutwardDirection;
            outward.y = 0f;

            if (outward.sqrMagnitude < 0.0001f)
            {
                try { outward = selection.Target.transform.forward; }
                catch { outward = Vector3.forward; }

                outward.y = 0f;
            }

            if (outward.sqrMagnitude < 0.0001f)
                outward = Vector3.forward;

            outward.Normalize();

            Quaternion rotation =
                Quaternion.LookRotation(outward, Vector3.up);

            try
            {
                ClearAll();

                _serial++;

                GameObject root =
                    new GameObject(
                        "SeatActionProxy_" + _serial);

                root.transform.position =
                    selection.WorldSeatPoint;

                root.transform.rotation =
                    rotation;

                // 让静态家具未来即使发生移动，座点也可以随目标一起移动。
                // worldPositionStays=true 保持当前扫描得到的世界坐标不变。
                root.transform.SetParent(
                    selection.Target.transform,
                    true);

                Mesh mesh = BuildThinBoxMesh(
                    DefaultWidth,
                    DefaultDepth,
                    DefaultThickness);

                MeshFilter filter =
                    root.AddComponent<MeshFilter>();

                filter.sharedMesh = mesh;

                MeshCollider collider =
                    root.AddComponent<MeshCollider>();

                collider.sharedMesh = mesh;
                collider.convex = false;
                collider.isTrigger = false;

                MeshRenderer renderer =
                    root.AddComponent<MeshRenderer>();

                renderer.sharedMaterial =
                    GetDebugMaterial();

                renderer.enabled =
                    _debugVisible;

                try
                {
                    root.layer =
                        selection.Target.layer;
                }
                catch { }

                _currentProxy = root;
                _currentMesh = mesh;

                pose = new SeatPose
                {
                    Target = selection.Target,
                    ActionProxy = root,
                    WorldSeatPoint = selection.WorldSeatPoint,
                    SurfaceNormal = Vector3.up,
                    OutwardDirection = outward,
                    FloorPoint = selection.FloorPoint,
                    SeatRotation = rotation,
                    HeightAboveFloor = selection.HeightAboveFloor,
                    SupportWidth = DefaultWidth,
                    SupportDepth = DefaultDepth,
                    ProxyThickness = DefaultThickness,
                    WasSnapped = selection.IsSnapped,
                    HeightWarning = selection.HeightWarning,
                    Confidence = selection.IsSnapped ? 0.90f : 1.00f
                };

                CurrentPose = pose;

                Plugin.Logger?.LogInfo(
                    "[SeatActionProxy] created" +
                    " | name=" + root.name +
                    " | target=" + selection.Target.name +
                    " | seat=" + selection.WorldSeatPoint +
                    " | floor=" + selection.FloorPoint +
                    " | outward=" + outward +
                    " | size=" +
                    DefaultWidth.ToString("0.00") + "x" +
                    DefaultDepth.ToString("0.00") + "x" +
                    DefaultThickness.ToString("0.00"));

                return true;
            }
            catch (Exception e)
            {
                error =
                    e.GetType().Name +
                    " / " +
                    e.Message;

                ClearAll();
                return false;
            }
        }

        public static void SetDebugVisible(bool visible)
        {
            _debugVisible = visible;

            if (_currentProxy == null)
                return;

            try
            {
                MeshRenderer renderer =
                    _currentProxy.GetComponent<MeshRenderer>();

                if (renderer != null)
                    renderer.enabled = visible;
            }
            catch { }
        }

        public static string GetStatusText()
        {
            return
                "actionProxy=" +
                (_currentProxy == null
                    ? "<none>"
                    : _currentProxy.name) +
                " | target=" +
                (CurrentPose == null ||
                 CurrentPose.Target == null
                    ? "<none>"
                    : CurrentPose.Target.name) +
                " | debugVisible=" +
                _debugVisible;
        }

        public static void ClearAll()
        {
            GameObject oldProxy = _currentProxy;
            Mesh oldMesh = _currentMesh;

            _currentProxy = null;
            _currentMesh = null;
            CurrentPose = null;

            if (oldProxy != null)
            {
                try { UnityEngine.Object.Destroy(oldProxy); }
                catch { }
            }

            if (oldMesh != null)
            {
                try { UnityEngine.Object.Destroy(oldMesh); }
                catch { }
            }
        }

        private static Mesh BuildThinBoxMesh(
            float width,
            float depth,
            float thickness)
        {
            float hx = width * 0.5f;
            float hz = depth * 0.5f;
            float bottom = -Mathf.Abs(thickness);

            // 顶面局部 Y=0，因此 root.position 就是最终座面高度。
            Vector3[] vertices =
            {
                new Vector3(-hx, 0f, -hz),
                new Vector3( hx, 0f, -hz),
                new Vector3( hx, 0f,  hz),
                new Vector3(-hx, 0f,  hz),
                new Vector3(-hx, bottom, -hz),
                new Vector3( hx, bottom, -hz),
                new Vector3( hx, bottom,  hz),
                new Vector3(-hx, bottom,  hz)
            };

            int[] triangles =
            {
                0, 2, 1, 0, 3, 2,       // top
                4, 5, 6, 4, 6, 7,       // bottom
                0, 1, 5, 0, 5, 4,       // back
                3, 7, 6, 3, 6, 2,       // front
                0, 4, 7, 0, 7, 3,       // left
                1, 2, 6, 1, 6, 5        // right
            };

            Mesh mesh = new Mesh();
            mesh.name = "SeatActionProxy_ContinuousMesh";

            // MiSide 是 IL2CPP。这里不能把托管 Vector3[] / int[] 直接交给 Mesh，
            // 必须转换为 Il2CppSystem.Collections.Generic.List。
            var vertexList =
                new Il2CppSystem.Collections.Generic.List<Vector3>();

            for (int i = 0; i < vertices.Length; i++)
                vertexList.Add(vertices[i]);

            var triangleList =
                new Il2CppSystem.Collections.Generic.List<int>();

            for (int i = 0; i < triangles.Length; i++)
                triangleList.Add(triangles[i]);

            mesh.SetVertices(vertexList);
            mesh.SetTriangles(triangleList, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        private static Material GetDebugMaterial()
        {
            if (_debugMaterial != null)
                return _debugMaterial;

            Shader shader = null;

            try { shader = Shader.Find("Sprites/Default"); }
            catch { }

            if (shader == null)
            {
                try { shader = Shader.Find("Unlit/Color"); }
                catch { }
            }

            if (shader == null)
                shader = Shader.Find("Standard");

            _debugMaterial = new Material(shader);
            _debugMaterial.color =
                new Color(0.55f, 0.05f, 1f, 0.75f);

            try
            {
                _debugMaterial.SetInt(
                    "_SrcBlend",
                    (int)UnityEngine.Rendering.BlendMode.SrcAlpha);

                _debugMaterial.SetInt(
                    "_DstBlend",
                    (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);

                _debugMaterial.SetInt("_ZWrite", 0);
                _debugMaterial.EnableKeyword("_ALPHABLEND_ON");
                _debugMaterial.renderQueue = 5600;
            }
            catch { }

            return _debugMaterial;
        }
    }
}
