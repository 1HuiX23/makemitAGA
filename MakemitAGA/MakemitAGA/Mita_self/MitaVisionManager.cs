/*
 * [文件说明]: 摄像机管理、快照系统与截图保存
 * 
 * [分析过程]:
 * 1. 我们通过 UnityExplorer 发现了 Mita 的头部骨骼 (Head Bone)，决定将相机挂载于此实现第一人称。
 * 2. [痛点解决]: 米塔在截图后和 AI 返回结果期间会移动头部，导致 AI 返回的坐标与当前画面不符（视差问题）。
 * 3. [解决方案]: 引入 SnapshotCamera (幽灵相机)。在截图瞬间克隆当前相机参数并冻结，后续的坐标计算全部基于这个静止的幽灵相机。
 * 
 * [主要功能]:
 * 1. SetupCamera(): 在 Mita 头部创建 InternalCamera，设置 depth=1000 覆盖屏幕。
 * 2. SaveCurrentViewToDisk(): 
 *    - 创建 SnapshotCamera 冻结当前视角。
 *    - 使用 RenderTexture 截图。
 *    - 使用 ImageConversion.EncodeToJPG 保存为 cache.jpg (解决 IL2CPP 兼容性)。
 * 3. ToggleView(): Tab 键切换玩家/米塔视角。
 */
using UnityEngine;
using System.IO;
using BepInEx;

namespace MakemitAGA.Mita_self
{
    public static class MitaVisionManager
    {
        public static readonly Vector3 FixedPos = new Vector3(0f, 0.12f, 0.15f);
        public static readonly Vector3 FixedRot = new Vector3(0f, 0f, 0f);
        public static float FixedFOV = 60f;

        public static Camera InternalCamera;
        public static GameObject SnapshotCamObj;
        public static Camera SnapshotCamera;

        public static bool IsViewActive = false;
        private static int _lastMitaId = -1;

        public static void UpdateMita(MitaPerson mita)
        {
            if (mita.GetInstanceID() != _lastMitaId || InternalCamera == null)
            {
                SetupCamera(mita);
                _lastMitaId = mita.GetInstanceID();
            }

            if (Input.GetKeyDown(KeyCode.Tab)) ToggleView();

            if (IsViewActive && InternalCamera != null)
            {
                InternalCamera.transform.localPosition = FixedPos;
                InternalCamera.transform.localEulerAngles = FixedRot;
            }
        }

        private static void SetupCamera(MitaPerson mita)
        {
            Transform head = FindChildRecursive(mita.transform, "Head");
            if (head == null) return;

            var camObj = new GameObject("Mita_Internal_Eye");
            camObj.transform.SetParent(head, false);

            InternalCamera = camObj.AddComponent<Camera>();
            InternalCamera.depth = 1000f;
            InternalCamera.nearClipPlane = 0.01f;
            InternalCamera.fieldOfView = FixedFOV;
            InternalCamera.clearFlags = CameraClearFlags.Skybox;
            InternalCamera.targetTexture = null;

            camObj.SetActive(IsViewActive);
        }

        public static void ToggleView()
        {
            if (InternalCamera != null)
            {
                IsViewActive = !IsViewActive;
                InternalCamera.gameObject.SetActive(IsViewActive);
            }
        }

        public static void SaveCurrentViewToDisk()
        {
            if (InternalCamera == null) return;

            // 更新快照相机
            if (SnapshotCamObj != null) Object.Destroy(SnapshotCamObj);
            SnapshotCamObj = new GameObject("Mita_Snapshot_Ghost");
            SnapshotCamera = SnapshotCamObj.AddComponent<Camera>();
            SnapshotCamera.CopyFrom(InternalCamera);
            SnapshotCamObj.transform.position = InternalCamera.transform.position;
            SnapshotCamObj.transform.rotation = InternalCamera.transform.rotation;
            SnapshotCamera.enabled = false;

            // 截图
            int width = 1280;
            int height = 720;
            RenderTexture rt = new RenderTexture(width, height, 24);
            InternalCamera.targetTexture = rt;
            InternalCamera.Render();

            RenderTexture.active = rt;
            Texture2D tex = new Texture2D(width, height, TextureFormat.RGB24, false);
            tex.ReadPixels(new Rect(0, 0, width, height), 0, 0);
            tex.Apply();

            byte[] bytes = ImageConversion.EncodeToJPG(tex, 85);
            string path = Path.Combine(Paths.PluginPath, "cache.jpg");
            try { File.WriteAllBytes(path, bytes); } catch { }

            InternalCamera.targetTexture = null;
            RenderTexture.active = null;
            Object.Destroy(rt);
            Object.Destroy(tex);
        }

        private static Transform FindChildRecursive(Transform parent, string name)
        {
            if (parent.name == name) return parent;
            for (int i = 0; i < parent.childCount; i++)
            {
                var res = FindChildRecursive(parent.GetChild(i), name);
                if (res != null) return res;
            }
            return null;
        }
    }
}