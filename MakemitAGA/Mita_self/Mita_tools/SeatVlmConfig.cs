/*
 * SeatVlmConfig.cs
 * Seat VLM 正式流程的网络、截图、超时与工具协议常量。
 * svt_start 不存在默认目标；调用方必须显式传入目标描述。
 */
using System;

using MakemitAGA.World;
namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class SeatVlmConfig
    {
        public const string BackendUrl =
            "http://127.0.0.1:8080/";

        public const string BackendHealthUrl =
            "http://127.0.0.1:8080/health";

        public const string BackendExeName =
            "OnlineAIApiServer.exe";

        public const bool StartBackendAutomatically =
            true;

        public const int BackendHealthMaxAttempts = 60;
        public const int BackendHealthRetryDelayMilliseconds = 500;
        public const int BackendHealthRequestTimeoutSeconds = 2;

        public const int ScreenshotWidth = 1280;
        public const int ScreenshotHeight = 720;
        public const int ScreenshotJpegQuality = 88;

        public const int MaxModelTurns = 14;
        public const int MaxCandidates = 32;
        public const int MaxPointAttempts = 5;

        public const int CameraPoseWarmupLateUpdateFrames = 3;
        public const float CameraPoseWarmupTimeoutSeconds = 2.0f;
        public const float SeatScanTimeoutSeconds = 35.0f;
        public const float PreviewCaptureTimeoutSeconds = 18.0f;

        public const int MaxTransientHttpRetries = 3;
        public const float HttpRetryBaseDelaySeconds = 1.5f;
        public const float RequestProgressIntervalSeconds = 2.0f;

        // One upstream model call must not block the game for five or six minutes.
        // 120 seconds is comfortably above the observed 25~40 second successful
        // vision requests, but short enough to fail clearly when the upstream stalls.
        public const int ModelHttpTimeoutSeconds = 120;

        public const bool StreamBackendResponseToBepInExConsole = true;
        public const int StreamConsoleChunkSize = 160;

        public static string BuildToolSpecification()
        {
            return
@"你正在控制游戏内的视觉座点工具链。当前正式工具链中，每次回复只能输出一个工具标签，不要输出解释、思考过程或 Markdown。

【工具标签格式硬规则】
1. 每一个工具调用都必须同时包含开始标签 <tool_call> 和结束标签 </tool_call>。
2. 绝对不能只输出 <tool_call>工具正文 而遗漏最后的 </tool_call>。
3. 输出前必须检查：工具调用的最后一段字符确实是 </tool_call>。
4. 即使未来回复中同时包含普通对话文字，所有工具标签也仍必须独立、完整闭合。
5. 正确示例：<tool_call>confirm_snap</tool_call>
6. 错误示例：<tool_call>confirm_snap

工具与顺序：

第一步：
<tool_call>get_screen</tool_call>
游戏会保存当前米塔视角。

第二步：
<tool_call>get_object_list[0.2,0.3,0.5,0.3]</tool_call>
四个数依次为 left、top、width、height，使用图片左上角原点和 0~1 归一化坐标。

第三步：
<tool_call>select_object(0)</tool_call>
只能选择游戏返回的真实 Unity 候选 id 或精确名字。

select_object 成功后，游戏会自动扫描目标并生成物理代理，不需要额外工具。

第四步第一次选择：
游戏会发送一张单独的物理辅助图。
青色=完整代理表面，绿色=可承重，紫色=适合边缘坐姿动作，红色=不可用，橙色=高度软警告。
请自由选择一个落在紫色区域、且符合自然坐姿意图的位置：
<tool_call>select_2D[0.43,0.61]</tool_call>
select_2D 坐标使用当前辅助图自身的左上角原点 0~1 坐标。

如果模型点的位置物理无效，游戏才会计算最近的紫色建议点，并发送左右反馈图：
左侧=正常原始画面；
右侧=物理辅助画面；
红色 X=你原来的无效点；
绿色圆环=系统建议吸附点；
黄色线=调整方向。

反馈图阶段：
如果绿色建议点符合你的视觉意图，输出：
<tool_call>confirm_snap</tool_call>

如果建议点不合适，请直接重新选择：
<tool_call>select_2D[0.40,0.65]</tool_call>
此时坐标仍使用右侧辅助面板自身的局部 0~1 坐标，不使用整张左右拼图坐标。

禁止一次输出多个工具。
再次提醒：发送前检查完整闭合标签；每个 <tool_call> 都必须有对应的 </tool_call>。";
        }
    }
}
