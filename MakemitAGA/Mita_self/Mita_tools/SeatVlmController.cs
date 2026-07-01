/*
 * SeatVlmController.cs
 * 正式 Seat VLM 状态机：截图、区域圈选、真实对象选择、表面扫描、选点、吸附确认、
 * SeatPose/ActionProxy 创建以及自动调用 Mita_sit。
 * 不写 result.json；所有工具过程和最终结果输出到 BepInEx 控制台。
 */
using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using UnityEngine;

using MakemitAGA.World;
using MakemitAGA.Connection;
namespace MakemitAGA.Mita_self.Mita_tools
{
    internal enum SeatVlmState
    {
        Idle,
        WaitingForGetScreen,
        PreparingOriginalScreen,
        WaitingForObjectRegion,
        WaitingForObjectSelection,
        BuildingSeatSurface,
        CapturingAuxiliary,
        WaitingForSurfacePoint,
        CapturingSnapFeedback,
        WaitingForSnapDecision,
        Completed,
        Failed,
        Cancelled
    }

    internal sealed class ParsedToolCall
    {
        public string Name;
        public string Arguments;

        // True only when one trailing, whitelisted tool tag omitted </tool_call>
        // and the parser safely repaired it locally.
        public bool WasClosingTagRepaired;
    }

    internal static class SeatVlmController
    {
        private static readonly Regex ToolRegex =
            new Regex(
                @"<tool_call>\s*(.*?)\s*</tool_call>",
                RegexOptions.IgnoreCase |
                RegexOptions.Singleline);

        private static readonly Regex FlattenedToolRegex =
            new Regex(
                @"tool_call\s*(.*?)\s*tool_call",
                RegexOptions.IgnoreCase |
                RegexOptions.Singleline);

        // Safety net for a single tool block that reaches the end of the reply
        // without the required closing </tool_call>. The extracted body must
        // still match the strict whitelist grammar below.
        private static readonly Regex TrailingUnclosedToolRegex =
            new Regex(
                @"<tool_call>\s*(.*?)\s*$",
                RegexOptions.IgnoreCase |
                RegexOptions.Singleline);

        private static SeatVlmState _state =
            SeatVlmState.Idle;

        private static bool _requestInFlight;
        private static int _turnCount;
        private static int _pointAttemptCount;
        private static int _runSerial;
        private static int _requestSerial;
        private static string _task;
        private static string _lastToolResult;

        private static List<DetectedObjectCandidate>
            _candidates;

        private static DetectedObjectCandidate
            _selected;

        private static SeatSurfaceSelectionResult
            _pendingSnap;

        private static string _pendingSnapReason;

        public static void Tick()
        {
        }

        public static void StartTarget(
            string targetDescription,
            string source)
        {
            if (string.IsNullOrWhiteSpace(targetDescription))
            {
                PrintGame(
                    "<color=yellow>用法：svt_start 目标，例如 svt_start 沙发</color>");
                return;
            }

            if (_requestInFlight ||
                IsBusyState(_state))
            {
                PrintGame(
                    "<color=yellow>上一条请求、扫描或截图仍在进行。</color>");
                return;
            }

            ResetRuntime();
            _runSerial++;

            string target =
                targetDescription.Trim();

            _task =
                "找到当前视角中的“" +
                target +
                "”，选择真实 Unity 对象，并在物理辅助图的紫色动作有效区域中自由选择一个适合坐下的位置。";

            _state =
                SeatVlmState.WaitingForGetScreen;

            Plugin.Logger?.LogInfo(
                "[SeatVLM] START" +
                " | run=" + _runSerial +
                " | source=" + source +
                " | target=" + target +
                " | task=" + _task);

            PrintGame(
                "<color=yellow>Seat VLM 已启动：</color>" +
                target);

            SendModelRequest(
                "这是第一步。当前没有可用截图，只允许调用 get_screen。");
        }

        public static void Cancel(
            string reason)
        {
            _state =
                SeatVlmState.Cancelled;

            _requestInFlight = false;
            _pendingSnap = null;
            _pendingSnapReason = null;

            SeatVlmVisionManager
                .EndSnapshotPreparation();

            SeatSurfaceVlmPreviewManager
                .CancelCaptureOnly();

            Plugin.Logger?.LogInfo(
                "[SeatVLM] CANCELLED: " +
                reason);
        }

        public static void ResetForSceneChange(
            string reason)
        {
            Cancel(reason);

            _state =
                SeatVlmState.Idle;

            _turnCount = 0;
            _pointAttemptCount = 0;
        }

        /// <summary>
        /// 控制台 svt_clear 使用：停止当前网络/截图状态并回到 Idle，
        /// 但不销毁 World 层生成的分析 Collider 和 Action Proxy。
        /// </summary>
        public static void ClearCommandState(
            string reason)
        {
            if (_requestInFlight ||
                IsBusyState(_state) ||
                (_state != SeatVlmState.Idle &&
                 _state != SeatVlmState.Completed &&
                 _state != SeatVlmState.Failed &&
                 _state != SeatVlmState.Cancelled))
            {
                Cancel(reason);
            }

            _state = SeatVlmState.Idle;
            _requestInFlight = false;
            _turnCount = 0;
            _pointAttemptCount = 0;
            _task = null;
            _lastToolResult = null;
            _candidates = null;
            _selected = null;
            _pendingSnap = null;
            _pendingSnapReason = null;
        }

        public static string GetStatusText()
        {
            return
                "state=" + _state +
                " | request=" +
                _requestInFlight +
                " | turn=" +
                _turnCount +
                "/" +
                SeatVlmConfig.MaxModelTurns +
                " | selected=" +
                (_selected == null
                    ? "<none>"
                    : _selected.Name) +
                " | scan=" +
                SeatSurfaceAnalysisRuntime
                    .GetPipelineStatus() +
                " | previewBusy=" +
                SeatSurfaceVlmPreviewManager
                    .IsCaptureInProgress +
                " | pendingSnap=" +
                (_pendingSnap != null) +
                " | " +
                SeatActionProxyRuntime.GetStatusText();
        }

        private static bool IsBusyState(
            SeatVlmState state)
        {
            return
                state ==
                    SeatVlmState.PreparingOriginalScreen ||
                state ==
                    SeatVlmState.BuildingSeatSurface ||
                state ==
                    SeatVlmState.CapturingAuxiliary ||
                state ==
                    SeatVlmState.CapturingSnapFeedback;
        }

        private static void ResetRuntime()
        {
            _state =
                SeatVlmState.Idle;

            _requestInFlight = false;
            _turnCount = 0;
            _pointAttemptCount = 0;

            _task = null;
            _lastToolResult = null;
            _candidates = null;
            _selected = null;
            _pendingSnap = null;
            _pendingSnapReason = null;

            SeatVlmDebugVisuals.ClearAll();

            SeatSurfaceVlmPreviewManager
                .CancelCaptureOnly();

            SeatSurfaceAnalysisRuntime.ClearAll();

            // ClearAll 会重建本轮分析状态；随后恢复用户通过 debug_svt 选择的可见性。
            SeatVlmDebugVisuals.SetVisible(
                SeatVlmIntegration.DebugEnabled);

            SeatSurfaceAnalysisRuntime.SetDebugRenderersVisible(
                SeatVlmIntegration.DebugEnabled);
        }

        private static void SendModelRequest(
            string instruction)
        {
            if (_requestInFlight)
            {
                Fail(
                    "内部错误：重复发送模型请求。");

                return;
            }

            if (_turnCount >=
                SeatVlmConfig.MaxModelTurns)
            {
                Fail(
                    "模型调用次数超过限制。");

                return;
            }

            _turnCount++;
            _requestSerial++;
            _requestInFlight = true;

            string prompt =
                BuildPrompt(instruction);

            bool includeImage =
                ShouldIncludeImageForState(
                    _state);

            Plugin.Logger?.LogInfo(
                "[SeatVLM][Request " +
                _requestSerial +
                "] START" +
                " | state=" + _state +
                " | includeImage=" +
                includeImage +
                " | promptChars=" +
                prompt.Length);

            Plugin.Runner.StartCoroutine(
                RequestRoutine(
                    prompt,
                    _requestSerial,
                    includeImage,
                    _state.ToString())
                .WrapToIl2Cpp());
        }

        private static IEnumerator RequestRoutine(
            string prompt,
            int requestId,
            bool includeImage,
            string stateName)
        {
            var chunkQueue =
                new ConcurrentQueue<string>();

            var stageQueue =
                new ConcurrentQueue<string>();

            var task =
                SeatVlmAIClient
                    .GetResponseStreamingAsync(
                        prompt,
                        includeImage,
                        _runSerial,
                        requestId,
                        stateName,
                        chunk =>
                            chunkQueue.Enqueue(chunk),
                        stage =>
                            stageQueue.Enqueue(stage));

            float startedAt =
                Time.realtimeSinceStartup;

            float nextProgressAt =
                startedAt +
                SeatVlmConfig
                    .RequestProgressIntervalSeconds;

            StringBuilder streamBuffer =
                new StringBuilder();

            bool printedAnyChunk = false;

            while (!task.IsCompleted)
            {
                DrainStageQueue(
                    stageQueue,
                    requestId);

                DrainChunkQueue(
                    chunkQueue,
                    streamBuffer,
                    requestId,
                    false,
                    ref printedAnyChunk);

                float now =
                    Time.realtimeSinceStartup;

                if (now >= nextProgressAt)
                {
                    Plugin.Logger?.LogInfo(
                        "[SeatVLM][Request " +
                        requestId +
                        "] WAITING" +
                        " | elapsed=" +
                        (now - startedAt)
                            .ToString(
                                "0.0",
                                CultureInfo.InvariantCulture) +
                        "s | state=" +
                        _state);

                    nextProgressAt =
                        now +
                        SeatVlmConfig
                            .RequestProgressIntervalSeconds;
                }

                yield return null;
            }

            DrainStageQueue(
                stageQueue,
                requestId);

            DrainChunkQueue(
                chunkQueue,
                streamBuffer,
                requestId,
                true,
                ref printedAnyChunk);

            _requestInFlight = false;

            if (task.IsFaulted)
            {
                string fault =
                    task.Exception?
                        .GetBaseException()
                        .Message ??
                    "unknown task fault";

                Fail(
                    "AI 请求失败：" +
                    fault);

                yield break;
            }

            string response =
                task.Result ?? "";

            Plugin.Logger?.LogInfo(
                "[SeatVLM][Request " +
                requestId +
                "] COMPLETE" +
                " | chars=" +
                response.Length);

            if (!printedAnyChunk &&
                !string.IsNullOrEmpty(response))
            {
                Plugin.Logger?.LogInfo(
                    "[SeatVLM][Request " +
                    requestId +
                    "][FULL RESPONSE] " +
                    MakeConsoleSafe(response));
            }

            if (response.StartsWith(
                "AI_ERROR:",
                StringComparison.Ordinal))
            {
                Fail(response);
                yield break;
            }

            HandleModelReply(response);
        }

        private static void HandleModelReply(
            string raw)
        {
            ParsedToolCall call;
            string error;

            if (!TryParseSingleToolCall(
                raw,
                out call,
                out error))
            {
                Plugin.Logger?.LogWarning(
                    "[SeatVLM] tool parse failed" +
                    " | error=" + error +
                    " | raw=" +
                    MakeConsoleSafe(raw));

                SendModelRequest(
                    "上一条回复无效：" +
                    error +
                    "。当前状态=" +
                    _state +
                    "。只输出当前状态允许的一个工具，" +
                    "并确保从 <tool_call> 开始、以 </tool_call> 完整结束；" +
                    "不要遗漏结束标签。");

                return;
            }

            if (call.WasClosingTagRepaired)
            {
                Plugin.Logger?.LogWarning(
                    "[SeatVLM] repaired incomplete tool tag" +
                    " | appended=</tool_call>" +
                    " | tool=" +
                    call.Name +
                    " | state=" +
                    _state);
            }

            Plugin.Logger?.LogInfo(
                "[SeatVLM] tool=" +
                call.Name +
                " | repairedClosingTag=" +
                call.WasClosingTagRepaired +
                " | state=" +
                _state);

            ExecuteTool(call);
        }

        private static void ExecuteTool(
            ParsedToolCall call)
        {
            string name =
                call.Name.ToLowerInvariant();

            if (name == "get_screen")
            {
                ExecuteGetScreen();
                return;
            }

            if (name == "get_object_list")
            {
                ExecuteGetObjectList(
                    call.Arguments);

                return;
            }

            if (name == "select_object")
            {
                ExecuteSelectObject(
                    call.Arguments);

                return;
            }

            if (name == "select_2d")
            {
                ExecuteSelect2D(
                    call.Arguments);

                return;
            }

            if (name == "confirm_snap")
            {
                ExecuteConfirmSnap();
                return;
            }

            SendModelRequest(
                "未知工具：" +
                call.Name);
        }

        private static void ExecuteGetScreen()
        {
            if (_state !=
                SeatVlmState.WaitingForGetScreen)
            {
                WrongState(
                    "get_screen",
                    SeatVlmState
                        .WaitingForGetScreen);

                return;
            }

            _state =
                SeatVlmState
                    .PreparingOriginalScreen;

            Plugin.Runner.StartCoroutine(
                PrepareOriginalScreenRoutine()
                    .WrapToIl2Cpp());
        }

        private static IEnumerator
            PrepareOriginalScreenRoutine()
        {
            string error;

            if (!SeatVlmVisionManager
                .BeginSnapshotPreparation(
                    out error))
            {
                Fail(
                    "get_screen 准备失败：" +
                    error);

                yield break;
            }

            float started =
                Time.realtimeSinceStartup;

            while (!SeatVlmVisionManager
                .IsSnapshotPoseReady)
            {
                if (Time.realtimeSinceStartup -
                    started >
                    SeatVlmConfig
                        .CameraPoseWarmupTimeoutSeconds)
                {
                    SeatVlmVisionManager
                        .EndSnapshotPreparation();

                    Fail(
                        "头部摄像机姿态校正超时。");

                    yield break;
                }

                yield return null;
            }

            yield return
                new WaitForEndOfFrame();

            bool ok =
                SeatVlmVisionManager
                    .SavePreparedViewToDisk(
                        out error);

            SeatVlmVisionManager
                .EndSnapshotPreparation();

            if (!ok)
            {
                Fail(
                    "get_screen 失败：" +
                    error);

                yield break;
            }

            _state =
                SeatVlmState
                    .WaitingForObjectRegion;

            _lastToolResult =
                "{\"tool\":\"get_screen\"," +
                "\"success\":true," +
                "\"image\":\"cache.jpg\"," +
                "\"width\":" +
                SeatVlmConfig.ScreenshotWidth +
                ",\"height\":" +
                SeatVlmConfig.ScreenshotHeight +
                "}";

            SendModelRequest(
                "请观察 cache.jpg，使用 get_object_list 圈出目标物体区域。");
        }

        private static void ExecuteGetObjectList(
            string arguments)
        {
            if (_state !=
                SeatVlmState
                    .WaitingForObjectRegion)
            {
                WrongState(
                    "get_object_list",
                    SeatVlmState
                        .WaitingForObjectRegion);

                return;
            }

            float[] values;
            string error;

            if (!TryParseRegionArguments(
                arguments,
                out values,
                out error))
            {
                SendModelRequest(
                    "get_object_list 参数错误：" +
                    error);

                return;
            }

            Rect rect;

            _candidates =
                SeatVlmObjectDetector
                    .FindObjectsInRegion(
                        values[0],
                        values[1],
                        values[2],
                        values[3],
                        out rect);

            _lastToolResult =
                SeatVlmObjectDetector
                    .BuildCandidatesJson(
                        rect,
                        _candidates);

            if (_candidates.Count == 0)
            {
                SendModelRequest(
                    "该区域没有检测到真实 Unity 物体。请重新圈选。结果：" +
                    _lastToolResult);

                return;
            }

            _state =
                SeatVlmState
                    .WaitingForObjectSelection;

            SendModelRequest(
                "候选 JSON：" +
                _lastToolResult +
                "。请调用 select_object，优先使用 id。");
        }

        private static void ExecuteSelectObject(
            string argument)
        {
            if (_state !=
                SeatVlmState
                    .WaitingForObjectSelection)
            {
                WrongState(
                    "select_object",
                    SeatVlmState
                        .WaitingForObjectSelection);

                return;
            }

            argument =
                NormalizeSelectObjectArgument(
                    argument);

            string error;

            if (!SeatVlmTargetPointSelector
                .TrySelectCandidate(
                    _candidates,
                    argument,
                    out _selected,
                    out error))
            {
                SendModelRequest(
                    "select_object 失败：" +
                    error +
                    "。候选仍为：" +
                    _lastToolResult);

                return;
            }

            _lastToolResult =
                "{\"tool\":\"select_object\"," +
                "\"success\":true," +
                "\"selected\":" +
                _selected.ToJson() +
                "}";

            _state =
                SeatVlmState
                    .BuildingSeatSurface;

            Plugin.Runner.StartCoroutine(
                BuildSurfaceAndRequestPointRoutine()
                    .WrapToIl2Cpp());
        }

        private static IEnumerator
            BuildSurfaceAndRequestPointRoutine()
        {
            SeatSurfaceAnalysisRuntime
                .RunTargetSeatabilityTest(
                    _selected.Object,
                    "seat-vlm-selected-object");

            float started =
                Time.realtimeSinceStartup;

            while (SeatSurfaceAnalysisRuntime
                .IsScanInProgress)
            {
                if (Time.realtimeSinceStartup -
                    started >
                    SeatVlmConfig
                        .SeatScanTimeoutSeconds)
                {
                    Fail(
                        "目标代理扫描超时：" +
                        SeatSurfaceAnalysisRuntime
                            .GetPipelineStatus());

                    yield break;
                }

                yield return null;
            }

            if (!SeatSurfaceAnalysisRuntime
                .LastScanSucceeded)
            {
                Fail(
                    "目标代理扫描失败：" +
                    SeatSurfaceAnalysisRuntime
                        .GetPipelineStatus());

                yield break;
            }

            _state =
                SeatVlmState
                    .CapturingAuxiliary;

            if (!SeatSurfaceVlmPreviewManager
                .StartAuxiliaryCaptureToCache(
                    "after-select_object"))
            {
                Fail(
                    "辅助图启动失败：" +
                    SeatSurfaceVlmPreviewManager
                        .LastCaptureError);

                yield break;
            }

            started =
                Time.realtimeSinceStartup;

            while (SeatSurfaceVlmPreviewManager
                .IsCaptureInProgress)
            {
                if (Time.realtimeSinceStartup -
                    started >
                    SeatVlmConfig
                        .PreviewCaptureTimeoutSeconds)
                {
                    SeatSurfaceVlmPreviewManager
                        .CancelCaptureOnly();

                    Fail(
                        "辅助图生成超时。");

                    yield break;
                }

                yield return null;
            }

            if (!SeatSurfaceVlmPreviewManager
                .LastCaptureSucceeded)
            {
                Fail(
                    "辅助图生成失败：" +
                    SeatSurfaceVlmPreviewManager
                        .LastCaptureError);

                yield break;
            }

            _state =
                SeatVlmState
                    .WaitingForSurfacePoint;

            _lastToolResult =
                "{\"tool\":\"seat_surface_ready\"," +
                "\"success\":true," +
                "\"image\":\"cache.jpg\"," +
                "\"imageKind\":\"auxiliary_only\"," +
                "\"target\":" +
                _selected.ToJson() +
                "}";

            SendModelRequest(
                "cache.jpg 现在是单张物理辅助图。" +
                "请在紫色边缘坐姿动作有效区域中自由选择符合视觉意图的位置。" +
                "请完整输出 <tool_call>select_2D[x,y]</tool_call>，" +
                "并确认最后包含 </tool_call>；不要输出未闭合标签。");
        }

        private static void ExecuteSelect2D(
            string arguments)
        {
            if (_state !=
                    SeatVlmState
                        .WaitingForSurfacePoint &&
                _state !=
                    SeatVlmState
                        .WaitingForSnapDecision)
            {
                SendModelRequest(
                    "当前状态不允许 select_2D：" +
                    _state);

                return;
            }

            float[] values;
            string parseError;

            if (!TryParsePointArguments(
                arguments,
                out values,
                out parseError))
            {
                RequestPointRetry(
                    "select_2D 参数错误：" +
                    parseError);

                return;
            }

            _pointAttemptCount++;

            Vector2 originalPoint =
                new Vector2(
                    values[0],
                    values[1]);

            SeatSurfaceSelectionResult
                directResult;

            string validationError;

            if (SeatSurfaceAnalysisRuntime
                .TrySelectActionPoint(
                    SeatSurfaceVlmPreviewManager
                        .SelectionCamera,
                    values[0],
                    values[1],
                    out directResult,
                    out validationError))
            {
                string selectionType =
                    _state ==
                        SeatVlmState
                            .WaitingForSnapDecision
                        ? "reselected_original_valid"
                        : "original_valid";

                CompleteSelection(
                    directResult,
                    selectionType);

                return;
            }

            SeatSurfaceSelectionResult
                snapResult;

            string snapError;

            if (!SeatSurfaceAnalysisRuntime
                .TryFindNearestActionPoint(
                    SeatSurfaceVlmPreviewManager
                        .SelectionCamera,
                    originalPoint,
                    out snapResult,
                    out snapError))
            {
                RequestPointRetry(
                    validationError +
                    "；并且未找到可接受的最近紫色建议点：" +
                    snapError);

                return;
            }

            _pendingSnap =
                snapResult;

            _pendingSnapReason =
                validationError;

            _state =
                SeatVlmState
                    .CapturingSnapFeedback;

            Plugin.Runner.StartCoroutine(
                CaptureSnapFeedbackRoutine()
                    .WrapToIl2Cpp());
        }

        private static IEnumerator
            CaptureSnapFeedbackRoutine()
        {
            if (!SeatSurfaceVlmPreviewManager
                .StartSnapFeedbackCaptureToCache(
                    _pendingSnap
                        .OriginalViewportTopLeft,
                    _pendingSnap
                        .SelectedViewportTopLeft,
                    "snap-feedback"))
            {
                Fail(
                    "吸附反馈图启动失败：" +
                    SeatSurfaceVlmPreviewManager
                        .LastCaptureError);

                yield break;
            }

            float started =
                Time.realtimeSinceStartup;

            while (SeatSurfaceVlmPreviewManager
                .IsCaptureInProgress)
            {
                if (Time.realtimeSinceStartup -
                    started >
                    SeatVlmConfig
                        .PreviewCaptureTimeoutSeconds)
                {
                    SeatSurfaceVlmPreviewManager
                        .CancelCaptureOnly();

                    Fail(
                        "吸附反馈图生成超时。");

                    yield break;
                }

                yield return null;
            }

            if (!SeatSurfaceVlmPreviewManager
                .LastCaptureSucceeded)
            {
                Fail(
                    "吸附反馈图生成失败：" +
                    SeatSurfaceVlmPreviewManager
                        .LastCaptureError);

                yield break;
            }

            _state =
                SeatVlmState
                    .WaitingForSnapDecision;

            _lastToolResult =
                "{\"tool\":\"snap_suggestion\"," +
                "\"success\":true," +
                "\"reason\":\"" +
                JsonEscape(_pendingSnapReason) +
                "\"," +
                "\"suggestion\":" +
                _pendingSnap.ToJson(
                    "pending_snap") +
                "}";

            SendModelRequest(
                "cache.jpg 是左右反馈图。" +
                "左侧为原始场景；右侧红 X 是你原来的无效点，" +
                "绿色圆环是最近紫色建议点。" +
                "若建议合理，请完整输出 <tool_call>confirm_snap</tool_call>。" +
                "若不合理，请完整输出 <tool_call>select_2D[x,y]</tool_call>。" +
                "必须包含最后的 </tool_call>，不要只输出未闭合的开始标签；" +
                "select_2D 仍使用右侧面板局部坐标。");
        }

        private static void ExecuteConfirmSnap()
        {
            if (_state !=
                SeatVlmState
                    .WaitingForSnapDecision)
            {
                WrongState(
                    "confirm_snap",
                    SeatVlmState
                        .WaitingForSnapDecision);

                return;
            }

            if (_pendingSnap == null)
            {
                Fail(
                    "confirm_snap 没有待确认建议点。");

                return;
            }

            CompleteSelection(
                _pendingSnap,
                "confirmed_snap");
        }

        private static void CompleteSelection(
            SeatSurfaceSelectionResult result,
            string selectionType)
        {
            SeatPose pose;
            string proxyError;

            if (!SeatActionProxyRuntime.TryCreateFromSelection(
                result,
                out pose,
                out proxyError))
            {
                Fail(
                    "局部连续动作代理创建失败：" +
                    proxyError);
                return;
            }

            _state =
                SeatVlmState.Completed;

            _lastToolResult =
                result.ToJson(
                    selectionType);

            Plugin.Logger?.LogInfo(
                "[SeatVLM] COMPLETED " +
                _lastToolResult);

            PrintGame(
                "<color=green>Seat VLM 完成，正在执行坐下。</color>\n" +
                "对象：" +
                (result.Target == null
                    ? "<null>"
                    : result.Target.name) +
                "\n座点：" +
                result.WorldSeatPoint +
                "\n地面点：" +
                result.FloorPoint +
                "\n朝向：" +
                result.OutwardDirection);

            _pendingSnap = null;
            _pendingSnapReason = null;

            // 正式整合的最后一步：把 SeatPose 交给动作系统。
            // Mita_sit 使用局部连续 Action Proxy，而不是直接踩整张稀疏分析网格。
            Mita_sit.Sit(pose);

            // 等价于自动执行 svt_clear：清除激光、候选框、彩色展示和截图相机，
            // 但保留分析 Collider 与 SeatActionProxy Collider。
            SeatVlmIntegration.ClearTransientArtifacts(
                "auto-complete",
                false);
        }

        private static void RequestPointRetry(
            string reason)
        {
            if (_pointAttemptCount >=
                SeatVlmConfig.MaxPointAttempts)
            {
                Fail(
                    "select_2D 尝试次数超过限制：" +
                    reason);

                return;
            }

            _pendingSnap = null;
            _pendingSnapReason = null;

            _state =
                SeatVlmState
                    .WaitingForSurfacePoint;

            if (SeatSurfaceVlmPreviewManager
                .LastCaptureMode !=
                SeatPreviewMode.AuxiliaryOnly)
            {
                Plugin.Runner.StartCoroutine(
                    RestoreAuxiliaryAndRetryRoutine(
                        reason)
                    .WrapToIl2Cpp());

                return;
            }

            SendModelRequest(
                "上一点无效：" +
                reason +
                "。请在当前单张辅助图的紫色区域中重新选择 select_2D[x,y]。");
        }

        private static IEnumerator
            RestoreAuxiliaryAndRetryRoutine(
                string reason)
        {
            _state =
                SeatVlmState
                    .CapturingAuxiliary;

            if (!SeatSurfaceVlmPreviewManager
                .StartAuxiliaryCaptureToCache(
                    "restore-after-retry"))
            {
                Fail(
                    "恢复单张辅助图失败：" +
                    SeatSurfaceVlmPreviewManager
                        .LastCaptureError);

                yield break;
            }

            while (SeatSurfaceVlmPreviewManager
                .IsCaptureInProgress)
            {
                yield return null;
            }

            if (!SeatSurfaceVlmPreviewManager
                .LastCaptureSucceeded)
            {
                Fail(
                    "恢复单张辅助图失败：" +
                    SeatSurfaceVlmPreviewManager
                        .LastCaptureError);

                yield break;
            }

            _state =
                SeatVlmState
                    .WaitingForSurfacePoint;

            SendModelRequest(
                "上一点无效：" +
                reason +
                "。cache.jpg 已恢复为单张辅助图，" +
                "请在紫色区域重新选择 select_2D[x,y]。");
        }

        private static bool ShouldIncludeImageForState(
            SeatVlmState state)
        {
            return
                state ==
                    SeatVlmState
                        .WaitingForObjectRegion ||
                state ==
                    SeatVlmState
                        .WaitingForObjectSelection ||
                state ==
                    SeatVlmState
                        .WaitingForSurfacePoint ||
                state ==
                    SeatVlmState
                        .WaitingForSnapDecision;
        }

        private static string BuildPrompt(
            string instruction)
        {
            StringBuilder sb =
                new StringBuilder();

            sb.AppendLine(
                SeatVlmConfig
                    .BuildToolSpecification());

            sb.AppendLine();
            sb.AppendLine(
                "当前任务：" +
                _task);

            sb.AppendLine(
                "当前状态：" +
                _state);

            sb.AppendLine(
                "当前轮次：" +
                _turnCount +
                "/" +
                SeatVlmConfig.MaxModelTurns);

            if (!string.IsNullOrEmpty(
                _lastToolResult))
            {
                sb.AppendLine(
                    "上一工具结果：");

                sb.AppendLine(
                    _lastToolResult);
            }

            sb.AppendLine(
                "当前指令：");

            sb.AppendLine(
                instruction);

            sb.AppendLine(
                "当前流程只输出一个完整工具标签，不要输出其他文字。");

            sb.AppendLine(
                "格式硬规则：每个 <tool_call> 都必须以 </tool_call> 完整闭合；" +
                "发送前检查结尾，绝不能遗漏结束标签。");

            return sb.ToString();
        }

        private static bool TryParseSingleToolCall(
            string raw,
            out ParsedToolCall call,
            out string error)
        {
            call = null;
            error = null;

            string input =
                raw ?? "";

            int openingTagCount =
                Regex.Matches(
                    input,
                    @"<tool_call>",
                    RegexOptions.IgnoreCase)
                    .Count;

            int closingTagCount =
                Regex.Matches(
                    input,
                    @"</tool_call>",
                    RegexOptions.IgnoreCase)
                    .Count;

            MatchCollection completeMatches =
                ToolRegex.Matches(input);

            if (completeMatches.Count > 1 ||
                openingTagCount > 1 ||
                closingTagCount > 1)
            {
                error =
                    "一次回复只能包含一个工具标签；" +
                    "opening=" +
                    openingTagCount +
                    "，closing=" +
                    closingTagCount +
                    "，complete=" +
                    completeMatches.Count;

                return false;
            }

            string body = null;
            bool repairedClosingTag = false;

            if (completeMatches.Count == 1)
            {
                if (openingTagCount != 1 ||
                    closingTagCount != 1)
                {
                    error =
                        "工具标签结构不一致；" +
                        "opening=" +
                        openingTagCount +
                        "，closing=" +
                        closingTagCount;

                    return false;
                }

                body =
                    completeMatches[0]
                        .Groups[1]
                        .Value
                        .Trim();
            }
            else
            {
                MatchCollection flattenedMatches =
                    FlattenedToolRegex
                        .Matches(input);

                if (flattenedMatches.Count == 1 &&
                    openingTagCount == 0 &&
                    closingTagCount == 0)
                {
                    body =
                        flattenedMatches[0]
                            .Groups[1]
                            .Value
                            .Trim();
                }
                else if (flattenedMatches.Count > 1)
                {
                    error =
                        "扁平化 tool_call 数量必须为 1，实际=" +
                        flattenedMatches.Count;

                    return false;
                }
                else if (openingTagCount == 1 &&
                         closingTagCount == 0)
                {
                    Match unclosed =
                        TrailingUnclosedToolRegex
                            .Match(input);

                    if (!unclosed.Success)
                    {
                        error =
                            "检测到未闭合的 <tool_call>，" +
                            "但它不是回复末尾唯一的结构化工具块。";

                        return false;
                    }

                    body =
                        unclosed
                            .Groups[1]
                            .Value
                            .Trim();

                    repairedClosingTag = true;
                }
                else
                {
                    error =
                        "未找到完整的 <tool_call>...</tool_call>。";

                    return false;
                }
            }

            if (!TryParseWhitelistedToolBody(
                body,
                out call,
                out error))
            {
                return false;
            }

            call.WasClosingTagRepaired =
                repairedClosingTag;

            return true;
        }

        private static bool TryParseWhitelistedToolBody(
            string body,
            out ParsedToolCall call,
            out string error)
        {
            call = null;
            error = null;

            body =
                (body ?? "").Trim();

            Match match;

            match =
                Regex.Match(
                    body,
                    @"^get_screen\s*(?:\(\s*\))?$",
                    RegexOptions.IgnoreCase);

            if (match.Success)
            {
                call =
                    new ParsedToolCall
                    {
                        Name = "get_screen",
                        Arguments = ""
                    };

                return true;
            }

            match =
                Regex.Match(
                    body,
                    @"^get_object_list\s*\[(.*?)\]\s*$",
                    RegexOptions.IgnoreCase |
                    RegexOptions.Singleline);

            if (match.Success)
            {
                call =
                    new ParsedToolCall
                    {
                        Name =
                            "get_object_list",
                        Arguments =
                            match.Groups[1]
                                .Value
                    };

                return true;
            }

            match =
                Regex.Match(
                    body,
                    @"^select_object\s*(?:\((.*?)\)|\[(.*?)\])\s*$",
                    RegexOptions.IgnoreCase |
                    RegexOptions.Singleline);

            if (match.Success)
            {
                string argument =
                    !string.IsNullOrEmpty(
                        match.Groups[1].Value)
                        ? match.Groups[1].Value
                        : match.Groups[2].Value;

                call =
                    new ParsedToolCall
                    {
                        Name =
                            "select_object",
                        Arguments =
                            argument
                    };

                return true;
            }

            match =
                Regex.Match(
                    body,
                    @"^select_2d\s*\[(.*?)\]\s*$",
                    RegexOptions.IgnoreCase |
                    RegexOptions.Singleline);

            if (match.Success)
            {
                call =
                    new ParsedToolCall
                    {
                        Name =
                            "select_2d",
                        Arguments =
                            match.Groups[1]
                                .Value
                    };

                return true;
            }

            match =
                Regex.Match(
                    body,
                    @"^confirm_snap\s*(?:\(\s*\))?$",
                    RegexOptions.IgnoreCase);

            if (match.Success)
            {
                call =
                    new ParsedToolCall
                    {
                        Name =
                            "confirm_snap",
                        Arguments = ""
                    };

                return true;
            }

            error =
                "无法识别或不允许的工具正文：" +
                body;

            return false;
        }

        private static bool TryParseRegionArguments(
            string raw,
            out float[] values,
            out string error)
        {
            if (TryParseFloatList(
                raw,
                4,
                out values,
                out error))
            {
                return ValidateRegion(
                    values,
                    out error);
            }

            Match robust =
                Regex.Match(
                    (raw ?? "").Trim(),
                    @"^L(\d{1,4})T(\d{1,4})W(\d{1,4})H(\d{1,4})$",
                    RegexOptions.IgnoreCase);

            if (!robust.Success)
                return false;

            values = new float[4];

            for (int i = 0; i < 4; i++)
            {
                int integerValue;

                if (!int.TryParse(
                    robust.Groups[i + 1].Value,
                    NumberStyles.Integer,
                    CultureInfo.InvariantCulture,
                    out integerValue))
                {
                    error =
                        "稳健区域坐标解析失败。";

                    return false;
                }

                values[i] =
                    integerValue / 1000f;
            }

            return ValidateRegion(
                values,
                out error);
        }

        private static bool ValidateRegion(
            float[] values,
            out string error)
        {
            error = null;

            if (values == null ||
                values.Length < 4 ||
                !IsNormalized(values[0]) ||
                !IsNormalized(values[1]) ||
                values[2] <= 0f ||
                values[3] <= 0f ||
                values[0] + values[2] > 1.001f ||
                values[1] + values[3] > 1.001f)
            {
                error =
                    "区域必须位于图片 0~1 范围内。";

                return false;
            }

            return true;
        }

        private static bool TryParsePointArguments(
            string raw,
            out float[] values,
            out string error)
        {
            if (TryParseFloatList(
                raw,
                2,
                out values,
                out error))
            {
                if (IsNormalized(values[0]) &&
                    IsNormalized(values[1]))
                {
                    return true;
                }

                error =
                    "二维坐标必须位于 0~1。";

                return false;
            }

            Match robust =
                Regex.Match(
                    (raw ?? "").Trim(),
                    @"^X(\d{1,4})Y(\d{1,4})$",
                    RegexOptions.IgnoreCase);

            if (!robust.Success)
                return false;

            values = new float[2];

            for (int i = 0; i < 2; i++)
            {
                int integerValue;

                if (!int.TryParse(
                    robust.Groups[i + 1].Value,
                    NumberStyles.Integer,
                    CultureInfo.InvariantCulture,
                    out integerValue))
                {
                    error =
                        "稳健二维坐标解析失败。";

                    return false;
                }

                values[i] =
                    integerValue / 1000f;
            }

            if (!IsNormalized(values[0]) ||
                !IsNormalized(values[1]))
            {
                error =
                    "二维坐标必须位于 0~1。";

                return false;
            }

            error = null;
            return true;
        }

        private static bool TryParseFloatList(
            string raw,
            int expected,
            out float[] values,
            out string error)
        {
            values = null;
            error = null;

            string[] parts =
                (raw ?? "")
                    .Split(
                        new char[]
                        {
                            ',',
                            ' ',
                            '\t',
                            '\r',
                            '\n'
                        },
                        StringSplitOptions
                            .RemoveEmptyEntries);

            if (parts.Length != expected)
            {
                error =
                    "参数数量应为 " +
                    expected +
                    "，实际=" +
                    parts.Length;

                return false;
            }

            values = new float[expected];

            for (int i = 0;
                 i < expected;
                 i++)
            {
                if (!float.TryParse(
                    parts[i],
                    NumberStyles.Float,
                    CultureInfo.InvariantCulture,
                    out values[i]))
                {
                    error =
                        "无法解析数字：" +
                        parts[i];

                    values = null;
                    return false;
                }
            }

            return true;
        }

        private static string NormalizeSelectObjectArgument(
            string raw)
        {
            string value =
                (raw ?? "").Trim();

            if (value.StartsWith(
                "ID",
                StringComparison.OrdinalIgnoreCase))
            {
                value =
                    value.Substring(2).Trim();
            }

            if (value.Length >= 2 &&
                ((value[0] == '"' &&
                  value[value.Length - 1] == '"') ||
                 (value[0] == '\'' &&
                  value[value.Length - 1] == '\'')))
            {
                value =
                    value.Substring(
                        1,
                        value.Length - 2)
                        .Trim();
            }

            return value;
        }

        private static void WrongState(
            string tool,
            SeatVlmState expected)
        {
            SendModelRequest(
                "工具顺序错误：" +
                tool +
                " 只允许在 " +
                expected +
                "，当前状态=" +
                _state);
        }

        private static void Fail(
            string reason)
        {
            _state =
                SeatVlmState.Failed;

            _requestInFlight = false;

            Plugin.Logger?.LogError(
                "[SeatVLM] FAILED: " +
                reason);

            PrintGame(
                "<color=red>Seat VLM 失败：</color>" +
                reason);
        }

        private static bool IsNormalized(
            float value)
        {
            return
                !float.IsNaN(value) &&
                !float.IsInfinity(value) &&
                value >= 0f &&
                value <= 1f;
        }

        private static string JsonEscape(
            string value)
        {
            return (value ?? "")
                .Replace("\\", "\\\\")
                .Replace("\"", "\\\"")
                .Replace("\r", "\\r")
                .Replace("\n", "\\n");
        }

        private static void DrainStageQueue(
            ConcurrentQueue<string> queue,
            int requestId)
        {
            string stage;

            while (queue.TryDequeue(
                out stage))
            {
                Plugin.Logger?.LogInfo(
                    "[SeatVLM][Request " +
                    requestId +
                    "][HTTP] " +
                    MakeConsoleSafe(stage));
            }
        }

        private static void DrainChunkQueue(
            ConcurrentQueue<string> queue,
            StringBuilder buffer,
            int requestId,
            bool flush,
            ref bool printedAnyChunk)
        {
            string chunk;

            while (queue.TryDequeue(
                out chunk))
            {
                if (string.IsNullOrEmpty(chunk))
                    continue;

                buffer.Append(chunk);

                while (buffer.Length >=
                    SeatVlmConfig.StreamConsoleChunkSize)
                {
                    string piece =
                        buffer.ToString(
                            0,
                            SeatVlmConfig.StreamConsoleChunkSize);

                    buffer.Remove(
                        0,
                        SeatVlmConfig.StreamConsoleChunkSize);

                    PrintStreamPiece(
                        requestId,
                        piece);

                    printedAnyChunk = true;
                }
            }

            if (flush &&
                buffer.Length > 0)
            {
                PrintStreamPiece(
                    requestId,
                    buffer.ToString());

                buffer.Length = 0;
                printedAnyChunk = true;
            }
        }

        private static void PrintStreamPiece(
            int requestId,
            string piece)
        {
            if (!SeatVlmConfig
                .StreamBackendResponseToBepInExConsole)
            {
                return;
            }

            Plugin.Logger?.LogInfo(
                "[SeatVLM][Request " +
                requestId +
                "][STREAM] " +
                MakeConsoleSafe(piece));
        }

        private static string MakeConsoleSafe(
            string value)
        {
            return (value ?? "")
                .Replace("\r", "\\r")
                .Replace("\n", "\\n")
                .Replace("\t", "\\t");
        }

        private static void PrintGame(
            string text)
        {
            try
            {
                ConsoleMain
                    .ConsolePrintGame(text);
            }
            catch
            {
            }
        }
    }
}
