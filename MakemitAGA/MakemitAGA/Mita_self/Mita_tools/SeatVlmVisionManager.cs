/*
 * SeatVlmVisionManager.cs
 * 创建米塔头部视觉相机，并在 LateUpdate 后冻结一次稳定世界姿态。
 * 只生成 BepInEx/plugins/cache.jpg，不写其他文本或 JSON 文件。
 */
using System;
using System.Collections;
using System.IO;
using BepInEx;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using UnityEngine;

using MakemitAGA.World;
namespace MakemitAGA.Mita_self.Mita_tools
{
    /// <summary>
    /// 米塔头部摄像头与冻结 SnapshotCamera。
    ///
    /// 这里严格沿用 MakemitAGA 的处理方式：
    /// 1. SetupCamera 只把相机挂到 Head，不假定创建当帧方向已经正确；
    /// 2. 需要截图时才临时激活相机 GameObject；
    /// 3. 在 MitaPerson.LateUpdate 的 Harmony Postfix 中，连续若干帧强制写入
    ///    FixedLocalPosition / FixedLocalEuler；
    /// 4. 等姿态稳定后冻结 SnapshotCamera；
    /// 5. 真正截图仍由 InternalCamera.Render() 完成；
    /// 6. SnapshotCamera 只保存当时的世界位置/旋转，后续候选投影和 select_2D 都用它。
    /// </summary>
    internal static class SeatVlmVisionManager
    {
        public static readonly Vector3 FixedLocalPosition =
            new Vector3(0f, 0.12f, 0.15f);

        public static readonly Vector3 FixedLocalEuler =
            new Vector3(0f, 0f, 0f);

        public static float FixedFov = 60f;

        public static Camera InternalCamera { get; private set; }
        public static Camera SnapshotCamera { get; private set; }
        public static Transform CurrentMitaRoot { get; private set; }

        private static GameObject _internalCameraObject;
        private static GameObject _snapshotCameraObject;
        private static Transform _headTransform;
        private static int _lastMitaId = -1;

        private static bool _snapshotPreparationActive;
        private static int _poseSyncCount;
        private static int _lastPoseSyncFrame = -1;

        public static bool IsReady
        {
            get
            {
                return
                    InternalCamera != null &&
                    _internalCameraObject != null &&
                    _headTransform != null &&
                    CurrentMitaRoot != null;
            }
        }

        public static bool IsSnapshotPoseReady
        {
            get
            {
                return
                    _snapshotPreparationActive &&
                    _poseSyncCount >=
                    SeatVlmConfig.CameraPoseWarmupLateUpdateFrames;
            }
        }

        /// <summary>
        /// 由 MitaPerson.LateUpdate Postfix 调用。
        /// 相机方向校正必须发生在游戏自己的 Animator / IK / Head 更新之后。
        /// </summary>
        public static void UpdateMita(MitaPerson mita)
        {
            if (mita == null) return;

            int id;
            try { id = mita.GetInstanceID(); }
            catch { return; }

            if (id != _lastMitaId || InternalCamera == null)
            {
                SetupCamera(mita);
                _lastMitaId = id;
            }

            if (!_snapshotPreparationActive || InternalCamera == null)
                return;

            try
            {
                if (_internalCameraObject != null &&
                    !_internalCameraObject.activeSelf)
                {
                    _internalCameraObject.SetActive(true);
                }

                ForceFixedLocalPose();

                // 一个 frame 只计数一次，避免同帧存在多个 MitaPerson LateUpdate 时误判稳定。
                if (_lastPoseSyncFrame != Time.frameCount)
                {
                    _lastPoseSyncFrame = Time.frameCount;
                    _poseSyncCount++;

                    Plugin.Logger?.LogInfo(
                        "[Vision] Head camera pose sync " +
                        _poseSyncCount + "/" +
                        SeatVlmConfig.CameraPoseWarmupLateUpdateFrames +
                        " | localPos=" + InternalCamera.transform.localPosition +
                        " | localEuler=" + InternalCamera.transform.localEulerAngles +
                        " | forward=" + InternalCamera.transform.forward);
                }
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogWarning(
                    "[Vision] LateUpdate camera pose sync failed: " +
                    e.GetType().Name + " / " + e.Message);
            }
        }

        private static void SetupCamera(MitaPerson mita)
        {
            DestroyInternalCamera();

            Transform head = FindChildRecursive(mita.transform, "Head");
            if (head == null)
            {
                Plugin.Logger?.LogWarning("[Vision] 找不到 Mita Head 骨骼。");
                return;
            }

            CurrentMitaRoot = mita.transform;
            _headTransform = head;

            _internalCameraObject = new GameObject("VT_Mita_Internal_Eye");
            _internalCameraObject.transform.SetParent(head, false);

            // 与 MakemitAGA 一致：
            // 创建阶段不把“刚创建出来的 transform”当作最终可用姿态。
            // 真正的 FixedPos / FixedRot 在 LateUpdate Postfix 中反复写入。
            InternalCamera = _internalCameraObject.AddComponent<Camera>();
            InternalCamera.depth = 1000f;
            InternalCamera.nearClipPlane = 0.01f;
            InternalCamera.fieldOfView = FixedFov;
            InternalCamera.clearFlags = CameraClearFlags.Skybox;
            InternalCamera.targetTexture = null;
            InternalCamera.enabled = false;

            // 平时保持 inactive；截图前才临时激活并等待 LateUpdate 校正。
            _internalCameraObject.SetActive(false);

            Plugin.Logger?.LogInfo(
                "[Vision] 米塔头部相机已创建，等待截图时进行 LateUpdate 姿态校正：" +
                GetTransformPath(head));
        }

        public static bool BeginSnapshotPreparation(out string error)
        {
            error = null;

            if (!IsReady)
            {
                error = "InternalCamera / Head 尚未准备好。";
                return false;
            }

            try
            {
                _snapshotPreparationActive = true;
                _poseSyncCount = 0;
                _lastPoseSyncFrame = -1;

                _internalCameraObject.SetActive(true);
                InternalCamera.enabled = false;
                ForceFixedLocalPose();

                Plugin.Logger?.LogInfo(
                    "[Vision] Snapshot preparation started. " +
                    "Waiting for Mita LateUpdate pose sync.");

                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + " / " + e.Message;
                _snapshotPreparationActive = false;
                return false;
            }
        }

        public static void EndSnapshotPreparation()
        {
            _snapshotPreparationActive = false;
            _poseSyncCount = 0;
            _lastPoseSyncFrame = -1;

            try
            {
                if (InternalCamera != null)
                {
                    InternalCamera.targetTexture = null;
                    InternalCamera.enabled = false;
                }

                if (_internalCameraObject != null)
                    _internalCameraObject.SetActive(false);
            }
            catch { }
        }

        /// <summary>
        /// 只有完成 LateUpdate warm-up 后才能调用。
        /// 截图由 InternalCamera.Render() 完成；SnapshotCamera 只冻结相机矩阵。
        /// </summary>
        public static bool SavePreparedViewToDisk(out string error)
        {
            error = null;

            if (!IsSnapshotPoseReady)
            {
                error =
                    "头部相机尚未完成 LateUpdate 姿态校正，sync=" +
                    _poseSyncCount + "/" +
                    SeatVlmConfig.CameraPoseWarmupLateUpdateFrames;

                return false;
            }

            DestroySnapshotCamera();

            RenderTexture rt = null;
            Texture2D tex = null;
            RenderTexture oldActive = RenderTexture.active;
            bool previousDebugVisibility = SeatVlmDebugVisuals.Visible;

            try
            {
                SeatVlmDebugVisuals.SetVisible(false);
                ForceFixedLocalPose();

                _snapshotCameraObject =
                    new GameObject("VT_Mita_Snapshot_Ghost");

                SnapshotCamera =
                    _snapshotCameraObject.AddComponent<Camera>();

                SnapshotCamera.CopyFrom(InternalCamera);
                _snapshotCameraObject.transform.position =
                    InternalCamera.transform.position;
                _snapshotCameraObject.transform.rotation =
                    InternalCamera.transform.rotation;

                SnapshotCamera.enabled = false;
                SnapshotCamera.targetTexture = null;

                rt = new RenderTexture(
                    SeatVlmConfig.ScreenshotWidth,
                    SeatVlmConfig.ScreenshotHeight,
                    24);

                // 与 MakemitAGA 一致：真正渲染用头部 InternalCamera。
                InternalCamera.targetTexture = rt;
                InternalCamera.Render();

                RenderTexture.active = rt;

                tex = new Texture2D(
                    SeatVlmConfig.ScreenshotWidth,
                    SeatVlmConfig.ScreenshotHeight,
                    TextureFormat.RGB24,
                    false);

                tex.ReadPixels(
                    new Rect(
                        0,
                        0,
                        SeatVlmConfig.ScreenshotWidth,
                        SeatVlmConfig.ScreenshotHeight),
                    0,
                    0);

                tex.Apply();

                byte[] bytes = ImageConversion.EncodeToJPG(
                    tex,
                    SeatVlmConfig.ScreenshotJpegQuality);

                string path =
                    Path.Combine(Paths.PluginPath, "cache.jpg");

                File.WriteAllBytes(path, bytes);

                Plugin.Logger?.LogInfo(
                    "[Vision] cache.jpg saved." +
                    " bytes=" + bytes.Length +
                    " | cameraPos=" + SnapshotCamera.transform.position +
                    " | cameraEuler=" + SnapshotCamera.transform.eulerAngles +
                    " | cameraForward=" + SnapshotCamera.transform.forward +
                    " | headForward=" +
                    (_headTransform == null
                        ? Vector3.zero
                        : _headTransform.forward));

                return true;
            }
            catch (Exception e)
            {
                error = e.GetType().Name + " / " + e.Message;
                Plugin.Logger?.LogError("[Vision] 截图失败：" + error);
                return false;
            }
            finally
            {
                try
                {
                    if (InternalCamera != null)
                        InternalCamera.targetTexture = null;

                    if (SnapshotCamera != null)
                        SnapshotCamera.targetTexture = null;
                }
                catch { }

                RenderTexture.active = oldActive;

                if (rt != null)
                {
                    try { rt.Release(); } catch { }
                    UnityEngine.Object.Destroy(rt);
                }

                if (tex != null)
                    UnityEngine.Object.Destroy(tex);

                SeatVlmDebugVisuals.SetVisible(previousDebugVisibility);
            }
        }

        public static void StartStandaloneSnapshot()
        {
            if (Plugin.Runner == null)
            {
                Plugin.Logger?.LogError(
                    "[Vision] Runner is null; cannot start standalone snapshot.");
                return;
            }

            Plugin.Runner.StartCoroutine(
                StandaloneSnapshotRoutine().WrapToIl2Cpp());
        }

        private static IEnumerator StandaloneSnapshotRoutine()
        {
            string error;

            if (!BeginSnapshotPreparation(out error))
            {
                PrintGame("<color=red>截图准备失败：</color>" + error);
                yield break;
            }

            float start = Time.realtimeSinceStartup;

            while (!IsSnapshotPoseReady)
            {
                if (Time.realtimeSinceStartup - start >
                    SeatVlmConfig.CameraPoseWarmupTimeoutSeconds)
                {
                    EndSnapshotPreparation();
                    PrintGame("<color=red>头部摄像头姿态校正超时。</color>");
                    yield break;
                }

                yield return null;
            }

            // 等到当前帧所有渲染前更新完成，再执行手动 Render。
            yield return new WaitForEndOfFrame();

            bool ok = SavePreparedViewToDisk(out error);
            EndSnapshotPreparation();

            if (ok)
                PrintGame("当前米塔视角已保存到 cache.jpg。");
            else
                PrintGame("<color=red>截图失败：</color>" + error);
        }

        public static void ClearSceneReferences()
        {
            EndSnapshotPreparation();
            DestroySnapshotCamera();
            DestroyInternalCamera();

            CurrentMitaRoot = null;
            _headTransform = null;
            _lastMitaId = -1;
        }

        private static void ForceFixedLocalPose()
        {
            if (InternalCamera == null) return;

            InternalCamera.transform.localPosition =
                FixedLocalPosition;

            InternalCamera.transform.localEulerAngles =
                FixedLocalEuler;
        }

        private static void DestroyInternalCamera()
        {
            GameObject old = _internalCameraObject;

            _internalCameraObject = null;
            InternalCamera = null;

            if (old != null)
                UnityEngine.Object.Destroy(old);
        }

        private static void DestroySnapshotCamera()
        {
            GameObject old = _snapshotCameraObject;

            _snapshotCameraObject = null;
            SnapshotCamera = null;

            if (old != null)
                UnityEngine.Object.Destroy(old);
        }

        private static Transform FindChildRecursive(
            Transform parent,
            string name)
        {
            if (parent == null) return null;

            if (string.Equals(
                parent.name,
                name,
                StringComparison.OrdinalIgnoreCase))
            {
                return parent;
            }

            for (int i = 0; i < parent.childCount; i++)
            {
                Transform found =
                    FindChildRecursive(parent.GetChild(i), name);

                if (found != null)
                    return found;
            }

            return null;
        }

        private static string GetTransformPath(Transform t)
        {
            if (t == null) return "<null>";

            string path = t.name;
            Transform p = t.parent;

            while (p != null)
            {
                path = p.name + "/" + path;
                p = p.parent;
            }

            return path;
        }

        private static void PrintGame(string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); }
            catch { }
        }
    }
}
