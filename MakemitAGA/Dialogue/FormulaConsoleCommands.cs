/*
 * =================================================================================================
 * FormulaConsoleCommands.cs
 * =================================================================================================
 *
 * 【职责】
 *   只负责解析 Formula3D 的调试命令，并把操作转发给 GameUIManager / Formula3DTextApi。
 *   本类不创建 GameObject、不渲染公式，也不拥有任何场景资源。
 *
 * 【命令】
 *   math3d_demo                 综合演示。
 *   math3d_show(...)            显示自定义普通文字 + LaTeX。
 *   math3d_drop                 手动掉落当前句。
 *   math3d_clear                清理当前 Formula3D 内容。
 *   math3d_autodrop             查询自动掉落设置。
 *   math3d_autodrop(秒数/off)   修改自动掉落设置。
 *
 * 【维护陷阱】
 *   1. math3d_show 使用“最外层前缀 + 最后一个右括号”的轻量解析，不是通用命令语法树。
 *      公式正文可以包含 LaTeX 花括号，但若未来允许末尾多余字符，需要重写解析器。
 *   2. 游戏控制台中反斜杠直接写；只有 C# 字符串字面量里才需要写成 \\frac。
 *   3. 本方法返回 true 表示命令已经消费，上层 DialoguePatches 不应继续交给原游戏处理。
 * =================================================================================================
 */
using System;
using System.Globalization;

namespace MakemitAGA.Dialogue
{
    public static class FormulaConsoleCommands
    {
        private const string ShowPrefix = "math3d_show(";
        private const string AutoDropPrefix = "math3d_autodrop(";

        /// <summary>
        /// 尝试消费一条控制台命令。返回 false 时，上层可继续按普通游戏命令处理。
        /// </summary>
        public static bool TryHandle(string rawCommand)
        {
            string command = (rawCommand ?? string.Empty).Trim();

            if (command.Equals("math3d_demo", StringComparison.OrdinalIgnoreCase))
            {
                GameUIManager.ShowLongText(
                    "一字公式 $\\frac{3}{4}$，短公式 $x^2$，" +
                    "中公式 $\\frac{a+b}{c+d}$，" +
                    "长公式 $\\int_0^1 \\frac{x^2+1}{\\sqrt{x+2}}\\,dx$。");
                return true;
            }

            if (command.Equals("math3d_drop", StringComparison.OrdinalIgnoreCase))
            {
                GameUIManager.DropCurrent();
                return true;
            }

            if (command.Equals("math3d_clear", StringComparison.OrdinalIgnoreCase))
            {
                GameUIManager.ClearCurrent("console-math3d_clear");
                PrintGame("Formula3D objects cleared.");
                return true;
            }

            if (command.Equals("math3d_autodrop", StringComparison.OrdinalIgnoreCase))
            {
                float current = Formula3DTextApi.AutoDropDelaySeconds;
                PrintGame(current >= 0f
                    ? "Formula3D 自动掉落：打印完成后 " + current.ToString("0.##") + " 秒。"
                    : "Formula3D 自动掉落：已关闭；输入 math3d_drop 后才继续下一句。");
                return true;
            }

            if (command.StartsWith(AutoDropPrefix, StringComparison.OrdinalIgnoreCase) &&
                command.EndsWith(")", StringComparison.Ordinal))
            {
                string value = command.Substring(
                    AutoDropPrefix.Length,
                    command.Length - AutoDropPrefix.Length - 1).Trim();

                if (value.Equals("off", StringComparison.OrdinalIgnoreCase) ||
                    value.Equals("disable", StringComparison.OrdinalIgnoreCase) || value == "-1")
                {
                    Formula3DTextApi.DisableAutoDrop();
                    PrintGame("Formula3D 自动掉落已关闭。");
                    return true;
                }

                if (!float.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out float seconds) ||
                    float.IsNaN(seconds) || float.IsInfinity(seconds) || seconds < 0f)
                {
                    PrintGame("用法：math3d_autodrop(4.5)；关闭：math3d_autodrop(off)");
                    return true;
                }

                Formula3DTextApi.SetAutoDropDelaySeconds(seconds);
                PrintGame("Formula3D 自动掉落间隔已设为 " +
                          Formula3DTextApi.AutoDropDelaySeconds.ToString("0.##") + " 秒。");
                return true;
            }

            if (command.StartsWith(ShowPrefix, StringComparison.OrdinalIgnoreCase) &&
                command.EndsWith(")", StringComparison.Ordinal))
            {
                string content = command.Substring(
                    ShowPrefix.Length,
                    command.Length - ShowPrefix.Length - 1);

                if (string.IsNullOrWhiteSpace(content))
                    PrintGame("用法：math3d_show(设函数 $f(x)=x^2$。)");
                else
                    GameUIManager.ShowLongText(content);

                return true;
            }

            return false;
        }

        private static void PrintGame(string text)
        {
            try { ConsoleMain.ConsolePrintGame(text); }
            catch { Plugin.Logger?.LogInfo("[Formula3D][GameConsole] " + text); }
        }
    }
}