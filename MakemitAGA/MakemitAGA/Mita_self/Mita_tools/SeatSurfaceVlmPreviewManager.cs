/*
 * SeatSurfaceVlmPreviewManager.cs
 * 用冻结相机生成纯白高对比物理辅助图与吸附反馈图。
 * 所有临时材质替换都在同一 EndOfFrame 原子阶段内完成并恢复，避免灰屏闪烁。
 * 唯一磁盘输出是 cache.jpg。
 */

/*
 * SeatSurfaceVlmPreviewManager.cs
 *
 * First point selection uses one 960x540 auxiliary image in cache.jpg.
 * Only after a physical failure do we create a 1920x540 original+auxiliary
 * feedback image with a red X and a green snap suggestion.
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using BepInEx;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using Il2CppInterop.Runtime.InteropTypes.Arrays;
using UnityEngine;
using Object = UnityEngine.Object;

using MakemitAGA.World;
namespace MakemitAGA.Mita_self.Mita_tools
{
    internal enum SeatPreviewMode
    {
        None,
        AuxiliaryOnly,
        SnapFeedbackComposite
    }

    internal static class SeatSurfaceVlmPreviewManager
    {
        public const int PanelWidth = 960;
        public const int PanelHeight = 540;
        public const int CompositeWidth = PanelWidth * 2;

        private const int CameraWarmupFrames = 3;
        private const float CameraWarmupTimeout = 2.0f;

        // The auxiliary image must use exactly the same frozen camera pose as
        // the original get_screen image. A second head-following camera is not
        // stable across long VLM round trips because Mita may rotate, animate or
        // move before select_object finishes.
        private static GameObject _cameraObject;
        private static Camera _internalCamera;

        private static Transform _headTransform;
        private static Transform _mitaRoot;
        private static int _lastMitaId = -1;

        // Player-controlled character. The player is not under MitaPerson and is
        // not reliably located under World/House. The independent probe confirmed
        // that GameObject.Find("Person") is the exact visible player root.
        private static GameObject _cachedPlayerPerson;

        private static Vector3 _frozenWorldPosition;
        private static Quaternion _frozenWorldRotation;
        private static bool _hasFrozenCameraPose;

        private static GameObject _selectionCameraObject;
        private static Camera _selectionCamera;

        private static bool _preparing;
        private static int _poseSyncCount;
        private static int _lastPoseSyncFrame = -1;
        private static bool _captureInProgress;
        private static int _captureSerial;

        private static GameObject _captureSourceTarget;
        private static Transform _captureSourceRoot;
        private static SeatPreviewMode _captureMode;
        private static Vector2 _feedbackOriginalPoint;
        private static Vector2 _feedbackSnapPoint;

        private static bool _lastCaptureSucceeded;
        private static string _lastCaptureError;
        private static SeatPreviewMode _lastCaptureMode;

        private static Material _sceneFadeMaterial;
        private static Material _targetHighlightMaterial;

        private static Transform _cachedHouseRoot;
        private static Renderer[] _cachedHouseRenderers;
        private static int _rendererCacheSceneHandle = int.MinValue;

        private sealed class RendererBackup
        {
            public Renderer renderer;
            public Il2CppReferenceArray<Material> sharedMaterials;
            public bool enabled;
        }

        private sealed class ActiveBackup
        {
            public GameObject gameObject;
            public bool activeSelf;
        }

        private sealed class RendererEnabledBackup
        {
            public Renderer renderer;
            public bool enabled;
        }

        public static bool IsCaptureInProgress
        {
            get { return _captureInProgress; }
        }

        public static bool LastCaptureSucceeded
        {
            get { return _lastCaptureSucceeded; }
        }

        public static string LastCaptureError
        {
            get { return _lastCaptureError; }
        }

        public static SeatPreviewMode LastCaptureMode
        {
            get { return _lastCaptureMode; }
        }

        public static Camera SelectionCamera
        {
            get { return _selectionCamera; }
        }

        public static void Initialize()
        {
            EnsurePreviewMaterials();
        }

        public static void UpdateMita(MitaPerson mita)
        {
            if (mita == null)
                return;

            int id;

            try
            {
                id = mita.GetInstanceID();
            }
            catch
            {
                return;
            }

            if (id != _lastMitaId ||
                _mitaRoot == null)
            {
                _lastMitaId = id;
                _mitaRoot = mita.transform;
                _headTransform =
                    FindChildRecursive(
                        mita.transform,
                        "Head");
            }

            if (!_preparing ||
                _internalCamera == null ||
                !_hasFrozenCameraPose)
            {
                return;
            }

            try
            {
                if (_cameraObject != null &&
                    !_cameraObject.activeSelf)
                {
                    _cameraObject.SetActive(true);
                }

                ForceFixedPose();

                if (_lastPoseSyncFrame !=
                    Time.frameCount)
                {
                    _lastPoseSyncFrame =
                        Time.frameCount;

                    _poseSyncCount++;

                    Plugin.Logger?.LogInfo(
                        "[SeatPreview] frozen camera sync " +
                        _poseSyncCount +
                        "/" +
                        CameraWarmupFrames +
                        " | pos=" +
                        _internalCamera.transform.position +
                        " | forward=" +
                        _internalCamera.transform.forward);
                }
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogWarning(
                    "[SeatPreview] frozen camera sync failed: " +
                    e.Message);
            }
        }

        public static bool StartAuxiliaryCaptureToCache(string source)
        {
            return StartCapture(
                SeatPreviewMode.AuxiliaryOnly,
                Vector2.zero,
                Vector2.zero,
                source);
        }

        public static bool StartSnapFeedbackCaptureToCache(
            Vector2 originalPoint,
            Vector2 snapPoint,
            string source)
        {
            return StartCapture(
                SeatPreviewMode.SnapFeedbackComposite,
                originalPoint,
                snapPoint,
                source);
        }

        private static bool StartCapture(
            SeatPreviewMode mode,
            Vector2 originalPoint,
            Vector2 snapPoint,
            string source)
        {
            if (_captureInProgress)
            {
                _lastCaptureError =
                    "已有预览截图正在进行。";

                return false;
            }

            GameObject target;
            Transform resultRoot;
            string sourceError;

            if (!SeatSurfaceAnalysisRuntime
                .TryGetCompositePreviewSource(
                    out target,
                    out resultRoot,
                    out sourceError))
            {
                _lastCaptureError = sourceError;
                return false;
            }

            string cameraError;

            if (!PrepareFrozenCameraFromOriginalSnapshot(
                out cameraError))
            {
                _lastCaptureError =
                    "无法复用 get_screen 冻结相机：" +
                    cameraError;

                return false;
            }

            if (Plugin.Runner == null)
            {
                _lastCaptureError =
                    "Runner 尚未准备好。";

                return false;
            }

            _captureSerial++;
            _captureInProgress = true;
            _lastCaptureSucceeded = false;
            _lastCaptureError = null;
            _lastCaptureMode = SeatPreviewMode.None;

            _captureSourceTarget = target;
            _captureSourceRoot = resultRoot;
            _captureMode = mode;
            _feedbackOriginalPoint = originalPoint;
            _feedbackSnapPoint = snapPoint;

            Plugin.Logger?.LogInfo(
                "[SeatPreview] start" +
                " | serial=" + _captureSerial +
                " | mode=" + mode +
                " | source=" + source);

            Plugin.Runner.StartCoroutine(
                CaptureRoutine(_captureSerial)
                    .WrapToIl2Cpp());

            return true;
        }

        public static void CancelCaptureOnly()
        {
            _captureSerial++;
            _captureInProgress = false;
            _captureSourceTarget = null;
            _captureSourceRoot = null;
            _captureMode = SeatPreviewMode.None;
            EndPreparation();
        }

        public static void CancelAndClear()
        {
            CancelCaptureOnly();
            InvalidateRendererCache();
            DestroyCamera();
            DestroySelectionCamera();

            _lastMitaId = -1;
            _headTransform = null;
            _mitaRoot = null;
            _cachedPlayerPerson = null;
            _hasFrozenCameraPose = false;
            _lastCaptureSucceeded = false;
            _lastCaptureError = null;
            _lastCaptureMode = SeatPreviewMode.None;
        }

        private static IEnumerator CaptureRoutine(
            int serial)
        {
            Texture2D original = null;
            Texture2D auxiliary = null;
            Texture2D output = null;

            Transform resultRoot =
                _captureSourceRoot;

            GameObject target =
                _captureSourceTarget;

            SeatPreviewMode mode =
                _captureMode;

            if (!BeginPreparation())
            {
                FinishCaptureFailure(
                    serial,
                    "无法开始头部相机预览准备。");

                yield break;
            }

            float started =
                Time.realtimeSinceStartup;

            while (_poseSyncCount <
                   CameraWarmupFrames)
            {
                if (serial != _captureSerial)
                {
                    EndPreparation();
                    yield break;
                }

                if (Time.realtimeSinceStartup -
                    started >
                    CameraWarmupTimeout)
                {
                    EndPreparation();

                    FinishCaptureFailure(
                        serial,
                        "头部相机 LateUpdate 校正超时。");

                    yield break;
                }

                yield return null;
            }

            // Wait until the normal game cameras have finished this frame.
            // Temporary material/renderer changes are then applied and restored
            // synchronously before Unity gets a chance to render the next frame.
            yield return new WaitForEndOfFrame();

            Exception stageException = null;
            string stageName = null;

            try
            {
                if (serial != _captureSerial)
                    yield break;

                // --------------------------------------------------------------
                // Atomic scene capture.
                //
                // IMPORTANT:
                // This method contains no yield. It performs:
                //   hide characters/debug
                //   optional original capture
                //   auxiliary material pass
                //   auxiliary capture
                //   complete restoration
                // all in the same end-of-frame callback.
                //
                // Therefore the main gameplay camera can never render the temporary
                // gray/white material state on a later frame.
                // --------------------------------------------------------------
                stageException = null;
                stageName =
                    "atomic-scene-capture";

                try
                {
                    CaptureSceneFramesAtomically(
                        resultRoot,
                        target,
                        mode,
                        out original,
                        out auxiliary);
                }
                catch (Exception e)
                {
                    stageException = e;
                }

                if (stageException != null)
                {
                    ReportCaptureStageFailure(
                        serial,
                        stageName,
                        stageException);

                    yield break;
                }

                // At this point all live-scene renderers/materials/active states
                // have already been restored. Subsequent yields are immersion-safe.
                yield return null;

                // --------------------------------------------------------------
                // Compose the final in-memory output after scene restoration.
                // --------------------------------------------------------------
                stageException = null;
                stageName =
                    "compose-output";

                try
                {
                    if (mode ==
                        SeatPreviewMode.AuxiliaryOnly)
                    {
                        output = auxiliary;
                        auxiliary = null;
                    }
                    else
                    {
                        output = ComposePanels(
                            original,
                            auxiliary);

                        DrawFeedbackOverlay(
                            output,
                            _feedbackOriginalPoint,
                            _feedbackSnapPoint);
                    }
                }
                catch (Exception e)
                {
                    stageException = e;
                }

                if (stageException != null)
                {
                    ReportCaptureStageFailure(
                        serial,
                        stageName,
                        stageException);

                    yield break;
                }

                yield return null;

                // --------------------------------------------------------------
                // Encode and save cache.jpg.
                // --------------------------------------------------------------
                stageException = null;
                stageName =
                    "save-cache";

                try
                {
                    SaveTexture(
                        output,
                        "cache.jpg");

                    _lastCaptureSucceeded = true;
                    _lastCaptureError = null;
                    _lastCaptureMode = mode;

                    Plugin.Logger?.LogInfo(
                        "[SeatPreview] SUCCESS" +
                        " | mode=" + mode +
                        " | output=" +
                        Path.Combine(
                            Paths.PluginPath,
                            "cache.jpg"));
                }
                catch (Exception e)
                {
                    stageException = e;
                }

                if (stageException != null)
                {
                    ReportCaptureStageFailure(
                        serial,
                        stageName,
                        stageException);

                    yield break;
                }
            }
            finally
            {
                // CaptureSceneFramesAtomically already restores all live-scene state
                // in its own finally block. The outer finally only owns textures and
                // preview-camera lifecycle.
                DestroyTexture(original);
                DestroyTexture(auxiliary);
                DestroyTexture(output);

                EndPreparation();

                if (serial == _captureSerial)
                {
                    _captureInProgress = false;
                    _captureSourceTarget = null;
                    _captureSourceRoot = null;
                    _captureMode = SeatPreviewMode.None;
                }
            }
        }

        private static void CaptureSceneFramesAtomically(
            Transform resultRoot,
            GameObject target,
            SeatPreviewMode mode,
            out Texture2D original,
            out Texture2D auxiliary)
        {
            original = null;
            auxiliary = null;

            var rendererBackups =
                new List<RendererBackup>();

            var activeBackups =
                new List<ActiveBackup>();

            var selfRendererBackups =
                new List<RendererEnabledBackup>();

            var playerRendererBackups =
                new List<RendererEnabledBackup>();

            bool previousDebugVisible =
                SeatSurfaceAnalysisRuntime
                    .DebugRenderersVisible;

            bool previousToolDebugVisible =
                SeatVlmDebugVisuals.Visible;

            float mutationStartedAt =
                Time.realtimeSinceStartup;

            try
            {
                SeatVlmDebugVisuals.SetVisible(false);

                HideSelfRenderers(
                    selfRendererBackups);

                HidePlayerRenderers(
                    playerRendererBackups);

                // For the left panel of snap feedback, remove only the generated
                // seatability result while keeping the exact frozen camera pose.
                if (mode ==
                    SeatPreviewMode.SnapFeedbackComposite)
                {
                    if (resultRoot != null)
                    {
                        activeBackups.Add(
                            new ActiveBackup
                            {
                                gameObject =
                                    resultRoot.gameObject,
                                activeSelf =
                                    resultRoot.gameObject.activeSelf
                            });

                        resultRoot.gameObject
                            .SetActive(false);
                    }

                    ForceFixedPose();

                    original = CaptureTexture(
                        _internalCamera,
                        PanelWidth,
                        PanelHeight);
                }

                if (resultRoot != null)
                {
                    resultRoot.gameObject
                        .SetActive(true);
                }

                SeatSurfaceAnalysisRuntime
                    .SetDebugRenderersVisible(true);

                HideAuxiliaryNoise(
                    resultRoot,
                    activeBackups);

                HideCharacterPreviewArtifacts(
                    activeBackups);

                ApplyAuxiliaryMaterials(
                    target,
                    resultRoot,
                    rendererBackups);

                ForceFixedPose();

                auxiliary = CaptureTexture(
                    _internalCamera,
                    PanelWidth,
                    PanelHeight);

                UpdateSelectionCameraSnapshot();
            }
            finally
            {
                // Restore every live-scene mutation before this normal method returns.
                // Since the coroutine does not yield inside this method, the gameplay
                // camera never sees the temporary white/gray material pass.
                RestorePlayerRenderers(
                    playerRendererBackups);

                RestoreSelfRenderers(
                    selfRendererBackups);

                RestoreMaterials(
                    rendererBackups);

                RestoreActiveStates(
                    activeBackups);

                SeatSurfaceAnalysisRuntime
                    .SetDebugRenderersVisible(
                        previousDebugVisible);

                SeatVlmDebugVisuals.SetVisible(
                    previousToolDebugVisible);

                float mutationMilliseconds =
                    (Time.realtimeSinceStartup -
                     mutationStartedAt) *
                    1000f;

                Plugin.Logger?.LogInfo(
                    "[SeatPreview] atomic scene capture restored" +
                    " | mode=" +
                    mode +
                    " | durationMs=" +
                    mutationMilliseconds.ToString("0.0") +
                    " | sceneStateRestoredBeforeYield=true");
            }
        }

        private static void ReportCaptureStageFailure(
            int serial,
            string stage,
            Exception exception)
        {
            string error =
                "stage=" +
                (stage ?? "<unknown>") +
                " | " +
                (exception == null
                    ? "unknown exception"
                    : exception.GetType().Name +
                      " / " +
                      exception.Message);

            FinishCaptureFailure(
                serial,
                error);

            Plugin.Logger?.LogError(
                "[SeatPreview] capture stage failed" +
                " | " +
                error +
                (exception == null
                    ? ""
                    : "\n" +
                      exception));
        }

        private static void FinishCaptureFailure(
            int serial,
            string error)
        {
            if (serial != _captureSerial)
                return;

            _lastCaptureSucceeded = false;
            _lastCaptureError = error;
            _lastCaptureMode = SeatPreviewMode.None;
            _captureInProgress = false;

            Plugin.Logger?.LogError(
                "[SeatPreview] FAILED: " +
                error);
        }

        private static void UpdateSelectionCameraSnapshot()
        {
            DestroySelectionCamera();

            _selectionCameraObject =
                new GameObject(
                    "SVT_SelectionCameraSnapshot");

            _selectionCameraObject.hideFlags =
                HideFlags.HideAndDontSave;

            _selectionCamera =
                _selectionCameraObject
                    .AddComponent<Camera>();

            _selectionCamera.CopyFrom(
                _internalCamera);

            _selectionCameraObject
                .transform.position =
                _internalCamera.transform.position;

            _selectionCameraObject
                .transform.rotation =
                _internalCamera.transform.rotation;

            _selectionCamera.aspect =
                PanelWidth /
                (float)PanelHeight;

            _selectionCamera.enabled = false;
            _selectionCamera.targetTexture = null;
        }

        private static void DestroySelectionCamera()
        {
            GameObject old =
                _selectionCameraObject;

            _selectionCameraObject = null;
            _selectionCamera = null;

            if (old != null)
                Object.Destroy(old);
        }

        private static void DrawFeedbackOverlay(
            Texture2D texture,
            Vector2 originalTopLeft,
            Vector2 snapTopLeft)
        {
            if (texture == null)
                return;

            int originalX =
                PanelWidth +
                Mathf.RoundToInt(
                    originalTopLeft.x *
                    (PanelWidth - 1));

            int originalY =
                Mathf.RoundToInt(
                    (1f - originalTopLeft.y) *
                    (PanelHeight - 1));

            int snapX =
                PanelWidth +
                Mathf.RoundToInt(
                    snapTopLeft.x *
                    (PanelWidth - 1));

            int snapY =
                Mathf.RoundToInt(
                    (1f - snapTopLeft.y) *
                    (PanelHeight - 1));

            DrawLine(
                texture,
                originalX,
                originalY,
                snapX,
                snapY,
                new Color(1f, 0.82f, 0.05f, 1f),
                3);

            DrawX(
                texture,
                originalX,
                originalY,
                15,
                new Color(1f, 0.03f, 0.03f, 1f),
                4);

            DrawRing(
                texture,
                snapX,
                snapY,
                17,
                new Color(0.05f, 1f, 0.15f, 1f),
                5);

            texture.Apply(false, false);
        }

        private static void DrawX(
            Texture2D texture,
            int cx,
            int cy,
            int radius,
            Color color,
            int thickness)
        {
            for (int d = -radius;
                 d <= radius;
                 d++)
            {
                for (int t = -thickness;
                     t <= thickness;
                     t++)
                {
                    SetPixelSafe(
                        texture,
                        cx + d,
                        cy + d + t,
                        color);

                    SetPixelSafe(
                        texture,
                        cx + d,
                        cy - d + t,
                        color);
                }
            }
        }

        private static void DrawRing(
            Texture2D texture,
            int cx,
            int cy,
            int radius,
            Color color,
            int thickness)
        {
            int outer = radius + thickness;
            int inner = Mathf.Max(0, radius - thickness);
            int outerSq = outer * outer;
            int innerSq = inner * inner;

            for (int y = -outer;
                 y <= outer;
                 y++)
            {
                for (int x = -outer;
                     x <= outer;
                     x++)
                {
                    int distance =
                        x * x +
                        y * y;

                    if (distance <= outerSq &&
                        distance >= innerSq)
                    {
                        SetPixelSafe(
                            texture,
                            cx + x,
                            cy + y,
                            color);
                    }
                }
            }
        }

        private static void DrawLine(
            Texture2D texture,
            int x0,
            int y0,
            int x1,
            int y1,
            Color color,
            int thickness)
        {
            int dx = Mathf.Abs(x1 - x0);
            int dy = Mathf.Abs(y1 - y0);
            int steps = Mathf.Max(dx, dy);

            if (steps <= 0)
                return;

            for (int i = 0;
                 i <= steps;
                 i++)
            {
                float t =
                    i / (float)steps;

                int x =
                    Mathf.RoundToInt(
                        Mathf.Lerp(x0, x1, t));

                int y =
                    Mathf.RoundToInt(
                        Mathf.Lerp(y0, y1, t));

                for (int ox = -thickness;
                     ox <= thickness;
                     ox++)
                {
                    for (int oy = -thickness;
                         oy <= thickness;
                         oy++)
                    {
                        SetPixelSafe(
                            texture,
                            x + ox,
                            y + oy,
                            color);
                    }
                }
            }
        }

        private static void SetPixelSafe(
            Texture2D texture,
            int x,
            int y,
            Color color)
        {
            if (x < 0 ||
                x >= texture.width ||
                y < 0 ||
                y >= texture.height)
            {
                return;
            }

            texture.SetPixel(
                x,
                y,
                color);
        }

        private static bool PrepareFrozenCameraFromOriginalSnapshot(
            out string error)
        {
            error = null;

            Camera source =
                SeatVlmVisionManager.SnapshotCamera;

            if (source == null)
            {
                error =
                    "SeatVlmVisionManager.SnapshotCamera 不存在；" +
                    "必须先完成 get_screen。";

                return false;
            }

            try
            {
                DestroyCamera();

                _cameraObject =
                    new GameObject(
                        "SVT_FrozenPreviewCamera");

                _cameraObject.hideFlags =
                    HideFlags.HideAndDontSave;

                // Deliberately do not parent this camera to Mita's Head.
                // It must remain at the exact get_screen world pose even if Mita
                // animates or turns during later model requests.
                _internalCamera =
                    _cameraObject.AddComponent<Camera>();

                _internalCamera.CopyFrom(source);

                _frozenWorldPosition =
                    source.transform.position;

                _frozenWorldRotation =
                    source.transform.rotation;

                _internalCamera.transform.position =
                    _frozenWorldPosition;

                _internalCamera.transform.rotation =
                    _frozenWorldRotation;

                _internalCamera.aspect =
                    PanelWidth /
                    (float)PanelHeight;

                _internalCamera.clearFlags =
                    CameraClearFlags.SolidColor;

                _internalCamera.backgroundColor =
                    Color.white;

                _internalCamera.enabled = false;
                _internalCamera.targetTexture = null;

                _hasFrozenCameraPose = true;
                _cameraObject.SetActive(false);

                Plugin.Logger?.LogInfo(
                    "[SeatPreview] frozen camera copied from get_screen SnapshotCamera" +
                    " | pos=" +
                    _frozenWorldPosition +
                    " | euler=" +
                    _frozenWorldRotation.eulerAngles +
                    " | forward=" +
                    (_frozenWorldRotation *
                     Vector3.forward) +
                    " | fov=" +
                    _internalCamera.fieldOfView +
                    " | cullingMask=" +
                    _internalCamera.cullingMask);

                return true;
            }
            catch (Exception e)
            {
                error =
                    e.GetType().Name +
                    " / " +
                    e.Message;

                DestroyCamera();
                return false;
            }
        }

        private static bool BeginPreparation()
        {
            if (_internalCamera == null ||
                _cameraObject == null ||
                !_hasFrozenCameraPose)
            {
                return false;
            }

            _preparing = true;
            _poseSyncCount = 0;
            _lastPoseSyncFrame = -1;

            _cameraObject.SetActive(true);
            _internalCamera.enabled = false;

            ForceFixedPose();
            return true;
        }

        private static void EndPreparation()
        {
            _preparing = false;
            _poseSyncCount = 0;
            _lastPoseSyncFrame = -1;

            try
            {
                if (_internalCamera != null)
                {
                    _internalCamera.targetTexture = null;
                    _internalCamera.enabled = false;
                }

                if (_cameraObject != null)
                    _cameraObject.SetActive(false);
            }
            catch { }
        }

        private static void ForceFixedPose()
        {
            if (_internalCamera == null ||
                !_hasFrozenCameraPose)
            {
                return;
            }

            _internalCamera.transform.position =
                _frozenWorldPosition;

            _internalCamera.transform.rotation =
                _frozenWorldRotation;
        }

        private static GameObject ResolvePlayerPerson()
        {
            if (_cachedPlayerPerson != null)
            {
                try
                {
                    if (_cachedPlayerPerson.gameObject != null)
                        return _cachedPlayerPerson;
                }
                catch
                {
                    _cachedPlayerPerson = null;
                }
            }

            try
            {
                _cachedPlayerPerson =
                    GameObject.Find("Person");
            }
            catch
            {
                _cachedPlayerPerson = null;
            }

            if (_cachedPlayerPerson == null)
            {
                try
                {
                    PlayerPerson typedPlayer =
                        UnityEngine.Object
                            .FindObjectOfType<PlayerPerson>();

                    if (typedPlayer != null)
                    {
                        _cachedPlayerPerson =
                            typedPlayer.gameObject;
                    }
                }
                catch { }
            }

            if (_cachedPlayerPerson != null)
            {
                Plugin.Logger?.LogInfo(
                    "[SeatPreview] player root resolved" +
                    " | path=" +
                    GetTransformPath(
                        _cachedPlayerPerson.transform));
            }
            else
            {
                Plugin.Logger?.LogWarning(
                    "[SeatPreview] player root not found" +
                    " | GameObject.Find(\"Person\") and PlayerPerson fallback failed.");
            }

            return _cachedPlayerPerson;
        }

        private static void HidePlayerRenderers(
            List<RendererEnabledBackup> backups)
        {
            GameObject playerRoot =
                ResolvePlayerPerson();

            if (playerRoot == null ||
                backups == null)
            {
                return;
            }

            Renderer[] renderers = null;

            try
            {
                renderers =
                    playerRoot
                        .GetComponentsInChildren<Renderer>(
                            true);
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogWarning(
                    "[SeatPreview] player renderer lookup failed" +
                    " | " +
                    e.GetType().Name +
                    " / " +
                    e.Message);
            }

            if (renderers == null ||
                renderers.Length == 0)
            {
                Plugin.Logger?.LogWarning(
                    "[SeatPreview] player root has no renderers" +
                    " | path=" +
                    GetTransformPath(
                        playerRoot.transform));

                return;
            }

            int hidden = 0;

            for (int i = 0;
                 i < renderers.Length;
                 i++)
            {
                Renderer renderer =
                    renderers[i];

                if (renderer == null)
                    continue;

                bool enabled;

                try
                {
                    enabled =
                        renderer.enabled;
                }
                catch
                {
                    continue;
                }

                backups.Add(
                    new RendererEnabledBackup
                    {
                        renderer = renderer,
                        enabled = enabled
                    });

                if (!enabled)
                    continue;

                try
                {
                    renderer.enabled = false;
                    hidden++;
                }
                catch { }
            }

            Plugin.Logger?.LogInfo(
                "[SeatPreview] player renderers hidden" +
                " | root=" +
                GetTransformPath(
                    playerRoot.transform) +
                " | enabledHidden=" +
                hidden +
                " | total=" +
                backups.Count);
        }

        private static void RestorePlayerRenderers(
            List<RendererEnabledBackup> backups)
        {
            RestoreRendererEnabledStates(
                backups,
                "player");
        }

        private static void HideSelfRenderers(
            List<RendererEnabledBackup> backups)
        {
            if (_mitaRoot == null ||
                backups == null)
            {
                return;
            }

            Renderer[] renderers = null;

            try
            {
                renderers =
                    _mitaRoot.GetComponentsInChildren<Renderer>(
                        true);
            }
            catch { }

            if (renderers == null)
                return;

            int hidden = 0;

            for (int i = 0;
                 i < renderers.Length;
                 i++)
            {
                Renderer renderer =
                    renderers[i];

                if (renderer == null)
                    continue;

                bool enabled = false;

                try
                {
                    enabled = renderer.enabled;
                }
                catch
                {
                    continue;
                }

                backups.Add(
                    new RendererEnabledBackup
                    {
                        renderer = renderer,
                        enabled = enabled
                    });

                if (!enabled)
                    continue;

                try
                {
                    renderer.enabled = false;
                    hidden++;
                }
                catch { }
            }

            Plugin.Logger?.LogInfo(
                "[SeatPreview] self renderers hidden=" +
                hidden +
                " | total=" +
                backups.Count);
        }

        private static void RestoreSelfRenderers(
            List<RendererEnabledBackup> backups)
        {
            RestoreRendererEnabledStates(
                backups,
                "mita");
        }

        private static void RestoreRendererEnabledStates(
            List<RendererEnabledBackup> backups,
            string category)
        {
            if (backups == null)
                return;

            int restored = 0;

            for (int i = backups.Count - 1;
                 i >= 0;
                 i--)
            {
                RendererEnabledBackup backup =
                    backups[i];

                if (backup == null ||
                    backup.renderer == null)
                {
                    continue;
                }

                try
                {
                    backup.renderer.enabled =
                        backup.enabled;

                    restored++;
                }
                catch { }
            }

            backups.Clear();

            if (restored > 0)
            {
                Plugin.Logger?.LogInfo(
                    "[SeatPreview] renderer states restored" +
                    " | category=" +
                    category +
                    " | count=" +
                    restored);
            }
        }

        private static Texture2D CaptureTexture(
            Camera camera,
            int width,
            int height)
        {
            RenderTexture rt = null;
            Texture2D texture = null;
            RenderTexture previous =
                RenderTexture.active;

            try
            {
                rt = new RenderTexture(
                    width,
                    height,
                    24,
                    RenderTextureFormat.ARGB32);

                camera.targetTexture = rt;
                camera.Render();

                RenderTexture.active = rt;

                texture = new Texture2D(
                    width,
                    height,
                    TextureFormat.RGB24,
                    false);

                texture.ReadPixels(
                    new Rect(
                        0,
                        0,
                        width,
                        height),
                    0,
                    0);

                texture.Apply();
                return texture;
            }
            finally
            {
                try
                {
                    camera.targetTexture = null;
                }
                catch { }

                RenderTexture.active = previous;

                if (rt != null)
                {
                    try { rt.Release(); }
                    catch { }

                    Object.Destroy(rt);
                }
            }
        }

        /// <summary>
        /// Compose the two panels without Graphics.DrawTexture / GL.PushMatrix.
        ///
        /// The previous implementation stopped inside the IL2CPP runtime immediately
        /// after auxiliary.jpg was written. Graphics.DrawTexture is intended mainly for
        /// GUI/render callbacks and is not reliable here from an arbitrary coroutine.
        ///
        /// Primary path:
        ///   render two textured quads with a private orthographic camera.
        ///
        /// Fallback path:
        ///   slow CPU GetPixel/SetPixel copy. It is only used if the private camera path
        ///   throws a normal managed exception.
        /// </summary>
        private static Texture2D ComposePanels(
            Texture2D left,
            Texture2D right)
        {
            Plugin.Logger?.LogInfo(
                "[SeatPreview] compose stage START" +
                " | left=" + left.width + "x" + left.height +
                " | right=" + right.width + "x" + right.height);

            try
            {
                Texture2D result =
                    ComposePanelsWithPrivateCamera(
                        left,
                        right);

                Plugin.Logger?.LogInfo(
                    "[SeatPreview] compose stage PRIVATE_CAMERA SUCCESS");

                return result;
            }
            catch (Exception e)
            {
                Plugin.Logger?.LogError(
                    "[SeatPreview] private-camera composition failed; " +
                    "trying CPU fallback: " +
                    e.GetType().Name +
                    " / " +
                    e.Message);

                Texture2D fallback =
                    ComposePanelsCpuFallback(
                        left,
                        right);

                Plugin.Logger?.LogInfo(
                    "[SeatPreview] compose stage CPU_FALLBACK SUCCESS");

                return fallback;
            }
        }

        private static Texture2D ComposePanelsWithPrivateCamera(
            Texture2D left,
            Texture2D right)
        {
            const int composeLayer = 31;

            GameObject root = null;
            GameObject cameraObject = null;
            GameObject leftQuad = null;
            GameObject rightQuad = null;
            GameObject dividerQuad = null;

            Camera composeCamera = null;
            RenderTexture rt = null;
            Texture2D result = null;

            Material leftMaterial = null;
            Material rightMaterial = null;
            Material dividerMaterial = null;

            RenderTexture previous =
                RenderTexture.active;

            try
            {
                root =
                    new GameObject(
                        "BST_PrivateCompositeStage");

                root.hideFlags =
                    HideFlags.HideAndDontSave;

                // Keep this miniature render stage far away from the real room.
                root.transform.position =
                    new Vector3(
                        10000f,
                        10000f,
                        10000f);

                cameraObject =
                    new GameObject(
                        "BST_PrivateCompositeCamera");

                cameraObject.hideFlags =
                    HideFlags.HideAndDontSave;

                cameraObject.transform
                    .SetParent(root.transform, false);

                cameraObject.transform.localPosition =
                    new Vector3(0f, 0f, -10f);

                cameraObject.transform.localRotation =
                    Quaternion.identity;

                cameraObject.layer =
                    composeLayer;

                composeCamera =
                    cameraObject.AddComponent<Camera>();

                composeCamera.enabled = false;
                composeCamera.orthographic = true;
                composeCamera.orthographicSize = 0.5f;
                composeCamera.aspect =
                    CompositeWidth /
                    (float)PanelHeight;

                composeCamera.nearClipPlane = 0.1f;
                composeCamera.farClipPlane = 20f;
                composeCamera.clearFlags =
                    CameraClearFlags.SolidColor;

                composeCamera.backgroundColor =
                    Color.black;

                composeCamera.cullingMask =
                    1 << composeLayer;

                float totalWorldWidth =
                    composeCamera.orthographicSize *
                    2f *
                    composeCamera.aspect;

                float panelWorldWidth =
                    totalWorldWidth * 0.5f;

                leftQuad =
                    CreateCompositeQuad(
                        "BST_Composite_Left",
                        root.transform,
                        composeLayer,
                        new Vector3(
                            -panelWorldWidth * 0.5f,
                            0f,
                            0f),
                        new Vector3(
                            panelWorldWidth,
                            1f,
                            1f),
                        left,
                        out leftMaterial);

                rightQuad =
                    CreateCompositeQuad(
                        "BST_Composite_Right",
                        root.transform,
                        composeLayer,
                        new Vector3(
                            panelWorldWidth * 0.5f,
                            0f,
                            0f),
                        new Vector3(
                            panelWorldWidth,
                            1f,
                            1f),
                        right,
                        out rightMaterial);

                float dividerWorldWidth =
                    totalWorldWidth *
                    (4f / CompositeWidth);

                dividerQuad =
                    CreateSolidCompositeQuad(
                        "BST_Composite_Divider",
                        root.transform,
                        composeLayer,
                        new Vector3(
                            0f,
                            0f,
                            -0.02f),
                        new Vector3(
                            dividerWorldWidth,
                            1f,
                            1f),
                        Color.black,
                        out dividerMaterial);

                rt =
                    new RenderTexture(
                        CompositeWidth,
                        PanelHeight,
                        24,
                        RenderTextureFormat.ARGB32);

                rt.name =
                    "BST_CompositeRenderTexture";

                rt.Create();

                composeCamera.targetTexture =
                    rt;

                Plugin.Logger?.LogInfo(
                    "[SeatPreview] private compose camera rendering...");

                composeCamera.Render();

                Plugin.Logger?.LogInfo(
                    "[SeatPreview] private compose camera rendered.");

                RenderTexture.active =
                    rt;

                result =
                    new Texture2D(
                        CompositeWidth,
                        PanelHeight,
                        TextureFormat.RGB24,
                        false);

                result.ReadPixels(
                    new Rect(
                        0,
                        0,
                        CompositeWidth,
                        PanelHeight),
                    0,
                    0,
                    false);

                result.Apply(
                    false,
                    false);

                Plugin.Logger?.LogInfo(
                    "[SeatPreview] composite pixels read successfully.");

                return result;
            }
            catch
            {
                if (result != null)
                {
                    Object.Destroy(result);
                    result = null;
                }

                throw;
            }
            finally
            {
                try
                {
                    if (composeCamera != null)
                        composeCamera.targetTexture = null;
                }
                catch { }

                RenderTexture.active =
                    previous;

                if (rt != null)
                {
                    try { rt.Release(); }
                    catch { }

                    Object.Destroy(rt);
                }

                if (leftMaterial != null)
                    Object.Destroy(leftMaterial);

                if (rightMaterial != null)
                    Object.Destroy(rightMaterial);

                if (dividerMaterial != null)
                    Object.Destroy(dividerMaterial);

                if (root != null)
                    Object.Destroy(root);
            }
        }

        private static GameObject CreateCompositeQuad(
            string name,
            Transform parent,
            int layer,
            Vector3 localPosition,
            Vector3 localScale,
            Texture2D texture,
            out Material material)
        {
            GameObject quad =
                GameObject.CreatePrimitive(
                    PrimitiveType.Quad);

            quad.name = name;
            quad.hideFlags =
                HideFlags.HideAndDontSave;

            quad.layer = layer;

            quad.transform
                .SetParent(parent, false);

            quad.transform.localPosition =
                localPosition;

            quad.transform.localRotation =
                Quaternion.identity;

            quad.transform.localScale =
                localScale;

            material =
                CreateCompositeTextureMaterial(
                    texture);

            MeshRenderer renderer =
                quad.GetComponent<MeshRenderer>();

            if (renderer != null)
                renderer.sharedMaterial =
                    material;

            Collider collider =
                quad.GetComponent<Collider>();

            if (collider != null)
                collider.enabled = false;

            return quad;
        }

        private static GameObject CreateSolidCompositeQuad(
            string name,
            Transform parent,
            int layer,
            Vector3 localPosition,
            Vector3 localScale,
            Color color,
            out Material material)
        {
            GameObject quad =
                GameObject.CreatePrimitive(
                    PrimitiveType.Quad);

            quad.name = name;
            quad.hideFlags =
                HideFlags.HideAndDontSave;

            quad.layer = layer;

            quad.transform
                .SetParent(parent, false);

            quad.transform.localPosition =
                localPosition;

            quad.transform.localRotation =
                Quaternion.identity;

            quad.transform.localScale =
                localScale;

            Shader shader =
                FindCompositeShader();

            material =
                new Material(shader);

            material.color =
                color;

            try
            {
                material.SetColor(
                    "_Color",
                    color);

                material.SetInt(
                    "_Cull",
                    (int)UnityEngine.Rendering
                        .CullMode.Off);
            }
            catch { }

            MeshRenderer renderer =
                quad.GetComponent<MeshRenderer>();

            if (renderer != null)
                renderer.sharedMaterial =
                    material;

            Collider collider =
                quad.GetComponent<Collider>();

            if (collider != null)
                collider.enabled = false;

            return quad;
        }

        private static Material CreateCompositeTextureMaterial(
            Texture2D texture)
        {
            Shader shader =
                FindCompositeShader();

            Material material =
                new Material(shader);

            material.color =
                Color.white;

            material.mainTexture =
                texture;

            try
            {
                material.SetTexture(
                    "_MainTex",
                    texture);

                material.SetColor(
                    "_Color",
                    Color.white);

                material.SetInt(
                    "_Cull",
                    (int)UnityEngine.Rendering
                        .CullMode.Off);

                material.SetInt(
                    "_ZWrite",
                    1);
            }
            catch { }

            return material;
        }

        private static Shader FindCompositeShader()
        {
            Shader shader = null;

            try
            {
                shader =
                    Shader.Find(
                        "Unlit/Texture");
            }
            catch { }

            if (shader == null)
            {
                try
                {
                    shader =
                        Shader.Find(
                            "Sprites/Default");
                }
                catch { }
            }

            if (shader == null)
            {
                try
                {
                    shader =
                        Shader.Find(
                            "Unlit/Color");
                }
                catch { }
            }

            if (shader == null)
            {
                throw new InvalidOperationException(
                    "No usable shader found for composite stage.");
            }

            return shader;
        }

        private static Texture2D ComposePanelsCpuFallback(
            Texture2D left,
            Texture2D right)
        {
            Texture2D result =
                new Texture2D(
                    CompositeWidth,
                    PanelHeight,
                    TextureFormat.RGB24,
                    false);

            int dividerHalfWidth = 2;

            for (int y = 0;
                 y < PanelHeight;
                 y++)
            {
                for (int x = 0;
                     x < PanelWidth;
                     x++)
                {
                    result.SetPixel(
                        x,
                        y,
                        left.GetPixel(x, y));

                    result.SetPixel(
                        PanelWidth + x,
                        y,
                        right.GetPixel(x, y));
                }

                for (int dx = -dividerHalfWidth;
                     dx < dividerHalfWidth;
                     dx++)
                {
                    int dividerX =
                        PanelWidth + dx;

                    if (dividerX >= 0 &&
                        dividerX < CompositeWidth)
                    {
                        result.SetPixel(
                            dividerX,
                            y,
                            Color.black);
                    }
                }

                if (y % 90 == 0)
                {
                    Plugin.Logger?.LogInfo(
                        "[SeatPreview] CPU fallback row " +
                        y +
                        "/" +
                        PanelHeight);
                }
            }

            result.Apply(
                false,
                false);

            return result;
        }

        private static void ApplyAuxiliaryMaterials(
            GameObject target,
            Transform resultRoot,
            List<RendererBackup> backups)
        {
            EnsurePreviewMaterials();

            int changedCount = 0;
            int skippedCharacterCount = 0;
            int skippedSpecialRendererCount = 0;
            int skippedQuestUtilityCount = 0;
            int skippedOutsideHouseCount = 0;

            Transform houseRoot =
                ResolveHouseRoot(target);

            Plugin.Logger?.LogInfo(
                "[SeatPreview] auxiliary house root=" +
                (houseRoot == null
                    ? "<null>"
                    : GetTransformPath(houseRoot)));

            Renderer[] renderers =
                GetCachedHouseRenderers(
                    target,
                    houseRoot);

            if (renderers == null)
                return;

            for (int i = 0;
                 i < renderers.Length;
                 i++)
            {
                Renderer renderer =
                    renderers[i];

                if (renderer == null ||
                    !renderer.enabled ||
                    renderer.transform == null)
                {
                    continue;
                }

                if (resultRoot != null &&
                    (renderer.transform ==
                        resultRoot ||
                     renderer.transform.IsChildOf(
                        resultRoot)))
                {
                    // Keep the proxy classification colors.
                    continue;
                }

                if (_cameraObject != null &&
                    renderer.transform.IsChildOf(
                        _cameraObject.transform))
                {
                    continue;
                }

                // IMPORTANT:
                // Do not replace materials on Mita/player character renderers.
                // Some character hierarchies contain hidden full-body effect meshes or
                // proxy volumes. Applying our flat transparent material makes those
                // normally invisible meshes appear as a large purple/grey box around
                // the whole character, and some special renderers do not restore cleanly.
                if (IsCharacterHierarchy(renderer.transform))
                {
                    Il2CppReferenceArray<Material>
                        characterMaterials = null;

                    try
                    {
                        characterMaterials =
                            renderer.sharedMaterials;
                    }
                    catch { }

                    backups.Add(
                        new RendererBackup
                        {
                            renderer = renderer,
                            sharedMaterials =
                                characterMaterials,
                            enabled = renderer.enabled
                        });

                    try
                    {
                        renderer.enabled = false;
                        skippedCharacterCount++;
                    }
                    catch { }

                    continue;
                }

                // Skip renderer types that often depend on special shaders/material
                // semantics. Replacing them with a flat material can expose invisible
                // volumes, particles, trails or line geometry.
                if (renderer is SkinnedMeshRenderer ||
                    renderer is ParticleSystemRenderer ||
                    renderer is TrailRenderer ||
                    renderer is LineRenderer)
                {
                    Il2CppReferenceArray<Material>
                        specialMaterials = null;

                    try
                    {
                        specialMaterials =
                            renderer.sharedMaterials;
                    }
                    catch { }

                    backups.Add(
                        new RendererBackup
                        {
                            renderer = renderer,
                            sharedMaterials =
                                specialMaterials,
                            enabled = renderer.enabled
                        });

                    try
                    {
                        renderer.enabled = false;
                        skippedSpecialRendererCount++;
                    }
                    catch { }

                    continue;
                }

                // UnityExplorer confirmed that the persistent magenta box is:
                // World/Quests/Quest 1 Start/Times/MeshVisible
                //
                // It is a pre-existing quest helper mesh, not an object generated by this plugin.
                // Its original shader makes it invisible/special-purpose. Replacing that material
                // exposes the entire box. Explicitly exclude quest/time/helper renderers.
                if (IsQuestOrUtilityRenderer(
                    renderer.transform))
                {
                    skippedQuestUtilityCount++;
                    continue;
                }

                bool isTarget =
                    target != null &&
                    (renderer.transform ==
                        target.transform ||
                     renderer.transform.IsChildOf(
                        target.transform));

                // Only fade real room/house renderers. Do not touch World/Quests,
                // dialogue helpers, hidden timing meshes, cameras, UI or other systems.
                if (!isTarget &&
                    houseRoot != null &&
                    renderer.transform != houseRoot &&
                    !renderer.transform.IsChildOf(
                        houseRoot))
                {
                    skippedOutsideHouseCount++;
                    continue;
                }

                Il2CppReferenceArray<Material>
                    oldMaterials = null;

                try
                {
                    oldMaterials =
                        renderer.sharedMaterials;
                }
                catch { }

                backups.Add(
                    new RendererBackup
                    {
                        renderer = renderer,
                        sharedMaterials = oldMaterials,
                        enabled = renderer.enabled
                    });

                try
                {
                    Material replacementMaterial =
                        isTarget
                            ? _targetHighlightMaterial
                            : _sceneFadeMaterial;

                    int materialCount =
                        oldMaterials == null ||
                        oldMaterials.Length <= 0
                            ? 1
                            : oldMaterials.Length;

                    var replacementMaterials =
                        new Il2CppReferenceArray<Material>(
                            materialCount);

                    for (int materialIndex = 0;
                         materialIndex < materialCount;
                         materialIndex++)
                    {
                        replacementMaterials[materialIndex] =
                            replacementMaterial;
                    }

                    renderer.sharedMaterials =
                        replacementMaterials;

                    changedCount++;
                }
                catch (Exception e)
                {
                    Plugin.Logger?.LogWarning(
                        "[SeatPreview] material replace skipped" +
                        " | path=" +
                        GetTransformPath(renderer.transform) +
                        " | error=" +
                        e.GetType().Name +
                        " / " +
                        e.Message);
                }
            }

            Plugin.Logger?.LogInfo(
                "[SeatPreview] auxiliary material pass" +
                " | changed=" + changedCount +
                " | skippedCharacter=" + skippedCharacterCount +
                " | skippedSpecialRenderer=" + skippedSpecialRendererCount +
                " | skippedQuestUtility=" + skippedQuestUtilityCount +
                " | skippedOutsideHouse=" + skippedOutsideHouseCount);
        }

        public static void InvalidateRendererCache()
        {
            _cachedHouseRoot = null;
            _cachedHouseRenderers = null;
            _rendererCacheSceneHandle = int.MinValue;
        }

        private static Renderer[] GetCachedHouseRenderers(
            GameObject target,
            Transform resolvedHouseRoot)
        {
            int sceneHandle =
                UnityEngine.SceneManagement
                    .SceneManager
                    .GetActiveScene()
                    .handle;

            bool cacheValid =
                _cachedHouseRenderers != null &&
                _cachedHouseRoot != null &&
                resolvedHouseRoot ==
                    _cachedHouseRoot &&
                _rendererCacheSceneHandle ==
                    sceneHandle;

            if (cacheValid)
                return _cachedHouseRenderers;

            _cachedHouseRoot =
                resolvedHouseRoot;

            _rendererCacheSceneHandle =
                sceneHandle;

            if (_cachedHouseRoot == null)
            {
                _cachedHouseRenderers =
                    target == null
                        ? new Renderer[0]
                        : target.GetComponentsInChildren<Renderer>(
                            true);
            }
            else
            {
                _cachedHouseRenderers =
                    _cachedHouseRoot
                        .GetComponentsInChildren<Renderer>(
                            true);
            }

            Plugin.Logger?.LogInfo(
                "[SeatPreview] renderer cache rebuilt" +
                " | sceneHandle=" +
                sceneHandle +
                " | houseRoot=" +
                (_cachedHouseRoot == null
                    ? "<null>"
                    : GetTransformPath(
                        _cachedHouseRoot)) +
                " | rendererCount=" +
                (_cachedHouseRenderers == null
                    ? 0
                    : _cachedHouseRenderers.Length));

            return _cachedHouseRenderers;
        }

        private static Transform ResolveHouseRoot(
            GameObject target)
        {
            if (target == null)
                return null;

            Transform current =
                target.transform;

            Transform fallbackHouse =
                null;

            while (current != null)
            {
                string name =
                    (current.name ?? "")
                    .Trim();

                if (name.Equals(
                    "House",
                    StringComparison.OrdinalIgnoreCase))
                {
                    fallbackHouse = current;

                    Transform parent =
                        current.parent;

                    if (parent != null &&
                        (parent.name ?? "")
                        .IndexOf(
                            "HouseGame",
                            StringComparison.OrdinalIgnoreCase) >= 0)
                    {
                        return current;
                    }
                }

                current = current.parent;
            }

            return fallbackHouse;
        }

        private static bool IsQuestOrUtilityRenderer(
            Transform transform)
        {
            Transform current =
                transform;

            while (current != null)
            {
                string name =
                    (current.name ?? "")
                    .Trim()
                    .ToLowerInvariant();

                if (name == "quests" ||
                    name.StartsWith("quest ") ||
                    name == "times" ||
                    name == "meshvisible" ||
                    name == "meshinvisible" ||
                    name.Contains("triggerbox") ||
                    name.Contains("trigger_box") ||
                    name.Contains("visibilityvolume") ||
                    name.Contains("visibility_volume"))
                {
                    return true;
                }

                current = current.parent;
            }

            return false;
        }

        private static bool IsCharacterHierarchy(
            Transform transform)
        {
            if (transform == null)
                return false;

            if (_mitaRoot != null &&
                (transform == _mitaRoot ||
                 transform.IsChildOf(_mitaRoot)))
            {
                return true;
            }

            GameObject playerRoot =
                ResolvePlayerPerson();

            if (playerRoot != null &&
                (transform ==
                    playerRoot.transform ||
                 transform.IsChildOf(
                    playerRoot.transform)))
            {
                return true;
            }

            Transform current = transform;

            while (current != null)
            {
                string name =
                    (current.name ?? "")
                    .Trim()
                    .ToLowerInvariant();

                if (name.Contains("mitaperson") ||
                    name.Contains("playerperson") ||
                    name == "mita" ||
                    name.StartsWith("mita ") ||
                    name == "player" ||
                    name.StartsWith("player "))
                {
                    return true;
                }

                current = current.parent;
            }

            return false;
        }

        private static void RestoreMaterials(
            List<RendererBackup> backups)
        {
            for (int i = backups.Count - 1;
                 i >= 0;
                 i--)
            {
                RendererBackup backup =
                    backups[i];

                if (backup == null ||
                    backup.renderer == null)
                {
                    continue;
                }

                try
                {
                    if (backup.sharedMaterials != null)
                    {
                        backup.renderer.sharedMaterials =
                            backup.sharedMaterials;
                    }

                    backup.renderer.enabled =
                        backup.enabled;
                }
                catch { }
            }

            backups.Clear();
        }

        private static void HideAuxiliaryNoise(
            Transform resultRoot,
            List<ActiveBackup> backups)
        {
            if (resultRoot == null)
                return;

            Transform[] transforms = null;

            try
            {
                transforms =
                    resultRoot.GetComponentsInChildren<Transform>(
                        true);
            }
            catch { }

            if (transforms == null)
                return;

            for (int i = 0;
                 i < transforms.Length;
                 i++)
            {
                Transform t = transforms[i];

                if (t == null ||
                    t == resultRoot)
                {
                    continue;
                }

                string name =
                    t.name ?? "";

                bool hide =
                    name.StartsWith(
                        "BedSeatability_ActionMarkers",
                        StringComparison.OrdinalIgnoreCase) ||
                    name.StartsWith(
                        "BedSeatability_ClearanceVolumes",
                        StringComparison.OrdinalIgnoreCase) ||
                    name.StartsWith(
                        "BedSeatability_ScanVolume",
                        StringComparison.OrdinalIgnoreCase) ||
                    name.StartsWith(
                        "BedSeatability_FloorReference",
                        StringComparison.OrdinalIgnoreCase);

                if (!hide)
                    continue;

                GameObject go = t.gameObject;

                backups.Add(
                    new ActiveBackup
                    {
                        gameObject = go,
                        activeSelf = go.activeSelf
                    });

                go.SetActive(false);
            }
        }

        private static void HideCharacterPreviewArtifacts(
            List<ActiveBackup> backups)
        {
            if (_mitaRoot == null)
                return;

            Transform[] transforms = null;

            try
            {
                transforms =
                    _mitaRoot.GetComponentsInChildren<Transform>(
                        true);
            }
            catch { }

            if (transforms == null)
                return;

            for (int i = 0;
                 i < transforms.Length;
                 i++)
            {
                Transform t = transforms[i];

                if (t == null ||
                    t == _mitaRoot)
                {
                    continue;
                }

                string name =
                    (t.name ?? "")
                    .ToLowerInvariant();

                bool suspicious =
                    name.Contains("proxy") ||
                    name.Contains("volume") ||
                    name.Contains("bounds") ||
                    name.Contains("debugbox") ||
                    name.Contains("debug_box") ||
                    name.Contains("triggerbox") ||
                    name.Contains("trigger_box");

                if (!suspicious)
                    continue;

                GameObject go = t.gameObject;

                backups.Add(
                    new ActiveBackup
                    {
                        gameObject = go,
                        activeSelf = go.activeSelf
                    });

                try
                {
                    go.SetActive(false);
                }
                catch { }
            }
        }

        private static void RestoreActiveStates(
            List<ActiveBackup> backups)
        {
            for (int i = backups.Count - 1;
                 i >= 0;
                 i--)
            {
                ActiveBackup backup =
                    backups[i];

                if (backup == null ||
                    backup.gameObject == null)
                {
                    continue;
                }

                try
                {
                    backup.gameObject.SetActive(
                        backup.activeSelf);
                }
                catch { }
            }

            backups.Clear();
        }

        private static void EnsurePreviewMaterials()
        {
            if (_sceneFadeMaterial == null)
            {
                _sceneFadeMaterial =
                    MakePreviewMaterial(
                        new Color(
                            1f,
                            1f,
                            1f,
                            1f),
                        2000);
            }

            if (_targetHighlightMaterial == null)
            {
                _targetHighlightMaterial =
                    MakePreviewMaterial(
                        new Color(
                            0.58f,
                            0.58f,
                            0.58f,
                            1f),
                        2100);
            }
        }

        private static Material MakePreviewMaterial(
            Color color,
            int renderQueue)
        {
            Shader shader = null;

            try
            {
                shader =
                    Shader.Find("Unlit/Color");
            }
            catch { }

            if (shader == null)
            {
                try
                {
                    shader =
                        Shader.Find(
                            "Sprites/Default");
                }
                catch { }
            }

            if (shader == null)
            {
                shader =
                    Shader.Find("Standard");
            }

            Material material =
                new Material(shader);

            material.color = color;

            bool transparent =
                color.a < 0.999f;

            try
            {
                material.SetColor(
                    "_Color",
                    color);

                material.SetInt(
                    "_Cull",
                    (int)UnityEngine.Rendering
                        .CullMode.Off);

                if (transparent)
                {
                    material.SetInt(
                        "_SrcBlend",
                        (int)UnityEngine.Rendering
                            .BlendMode.SrcAlpha);

                    material.SetInt(
                        "_DstBlend",
                        (int)UnityEngine.Rendering
                            .BlendMode.OneMinusSrcAlpha);

                    material.SetInt(
                        "_ZWrite",
                        0);

                    material.EnableKeyword(
                        "_ALPHABLEND_ON");
                }
                else
                {
                    material.SetInt(
                        "_SrcBlend",
                        (int)UnityEngine.Rendering
                            .BlendMode.One);

                    material.SetInt(
                        "_DstBlend",
                        (int)UnityEngine.Rendering
                            .BlendMode.Zero);

                    material.SetInt(
                        "_ZWrite",
                        1);

                    material.DisableKeyword(
                        "_ALPHABLEND_ON");
                }
            }
            catch { }

            material.renderQueue =
                renderQueue;

            return material;
        }

        private static void SaveTexture(
            Texture2D texture,
            string fileName)
        {
            if (texture == null)
                throw new InvalidOperationException(
                    "Texture is null: " +
                    fileName);

            // 正式版本只允许覆盖 cache.jpg，防止后续调试代码重新引入
            // auxiliary.jpg、preview_meta.json 等 plugins 目录输出。
            if (!string.Equals(
                fileName,
                "cache.jpg",
                StringComparison.OrdinalIgnoreCase))
            {
                throw new InvalidOperationException(
                    "Seat preview may only write cache.jpg.");
            }

            byte[] bytes =
                ImageConversion.EncodeToJPG(
                    texture,
                    90);

            string path =
                Path.Combine(
                    Paths.PluginPath,
                    fileName);

            File.WriteAllBytes(
                path,
                bytes);

            Plugin.Logger?.LogInfo(
                "[SeatPreview] saved " +
                fileName +
                " bytes=" +
                bytes.Length);
        }

        private static void DestroyTexture(
            Texture2D texture)
        {
            if (texture != null)
                Object.Destroy(texture);
        }

        private static void DestroyCamera()
        {
            GameObject old =
                _cameraObject;

            _cameraObject = null;
            _internalCamera = null;
            _hasFrozenCameraPose = false;

            if (old != null)
                Object.Destroy(old);
        }

        private static Transform FindChildRecursive(
            Transform parent,
            string targetName)
        {
            if (parent == null)
                return null;

            if (string.Equals(
                parent.name,
                targetName,
                StringComparison.OrdinalIgnoreCase))
            {
                return parent;
            }

            for (int i = 0;
                 i < parent.childCount;
                 i++)
            {
                Transform result =
                    FindChildRecursive(
                        parent.GetChild(i),
                        targetName);

                if (result != null)
                    return result;
            }

            return null;
        }

        private static string GetTransformPath(
            Transform t)
        {
            if (t == null)
                return "<null>";

            string path = t.name;
            Transform parent = t.parent;

            while (parent != null)
            {
                path =
                    parent.name +
                    "/" +
                    path;

                parent = parent.parent;
            }

            return path;
        }

        private static void PrintGame(
            string text)
        {
            try
            {
                ConsoleMain.ConsolePrintGame(text);
            }
            catch { }
        }
    }
}
