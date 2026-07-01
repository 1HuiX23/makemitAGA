using System;
using System.Collections.Generic;
using System.Text;

namespace MakemitAGA.Dialogue
{
    internal enum FormulaTokenKind
    {
        Text,
        Formula
    }

    internal sealed class FormulaToken
    {
        public FormulaTokenKind Kind;
        public string Content;
        public bool DisplayStyle;

        public static FormulaToken Text(string text)
        {
            return new FormulaToken
            {
                Kind = FormulaTokenKind.Text,
                Content = text ?? string.Empty,
                DisplayStyle = false
            };
        }

        public static FormulaToken Formula(
            string latex,
            bool displayStyle)
        {
            return new FormulaToken
            {
                Kind = FormulaTokenKind.Formula,
                Content = latex ?? string.Empty,
                DisplayStyle = displayStyle
            };
        }
    }

    /// <summary>
    /// 第一版只解析公式边界，不尝试实现完整 Markdown。
    /// 支持：$...$、$$...$$、\(...\)、\[...\]。
    /// 反斜杠转义的 \$ 不会被视为公式开头。
    /// </summary>
    internal static class FormulaTokenParser
    {
        public static List<FormulaToken> Parse(string input)
        {
            var result = new List<FormulaToken>();
            var text = new StringBuilder();

            string source = input ?? string.Empty;
            int i = 0;

            while (i < source.Length)
            {
                if (TryReadFormula(
                    source,
                    i,
                    out int consumed,
                    out string latex,
                    out bool displayStyle))
                {
                    FlushText(result, text);

                    if (!string.IsNullOrWhiteSpace(latex))
                    {
                        result.Add(
                            FormulaToken.Formula(
                                latex.Trim(),
                                displayStyle));
                    }

                    i += consumed;
                    continue;
                }

                // \$ 只显示普通美元符号。
                if (source[i] == '\\' &&
                    i + 1 < source.Length &&
                    source[i + 1] == '$')
                {
                    text.Append('$');
                    i += 2;
                    continue;
                }

                char c = source[i];

                // 第一版单行测试：换行转换为空格，避免破坏原生单行布局。
                if (c == '\r' || c == '\n')
                {
                    if (text.Length == 0 ||
                        text[text.Length - 1] != ' ')
                    {
                        text.Append(' ');
                    }
                }
                else
                {
                    text.Append(c);
                }

                i++;
            }

            FlushText(result, text);

            /*
             * Markdown/LLM 常输出：
             *
             *     中文文字 $x^2$，继续中文
             *
             * 两侧空格在网页排版中很常见，但在逐字 3D 中文对话里会变成真实的半字宽，
             * 看起来像公式与文字之间多出一块空白。
             *
             * 这里只在相邻可见字符属于中文/CJK 或中文标点时去掉边界 ASCII 空格。
             * 英文句子如 “let $x$ be” 会继续保留空格。
             */
            NormalizeCjkFormulaBoundarySpaces(result);

            return result;
        }

        private static void NormalizeCjkFormulaBoundarySpaces(
            List<FormulaToken> tokens)
        {
            if (tokens == null ||
                tokens.Count == 0)
            {
                return;
            }

            for (int i = 0;
                 i < tokens.Count;
                 i++)
            {
                FormulaToken token = tokens[i];

                if (token == null ||
                    token.Kind !=
                        FormulaTokenKind.Formula)
                {
                    continue;
                }

                if (i > 0)
                {
                    FormulaToken previous =
                        tokens[i - 1];

                    if (previous != null &&
                        previous.Kind ==
                            FormulaTokenKind.Text)
                    {
                        previous.Content =
                            TrimTrailingFormulaSpaceForCjk(
                                previous.Content);
                    }
                }

                if (i + 1 < tokens.Count)
                {
                    FormulaToken next =
                        tokens[i + 1];

                    if (next != null &&
                        next.Kind ==
                            FormulaTokenKind.Text)
                    {
                        next.Content =
                            TrimLeadingFormulaSpaceForCjk(
                                next.Content);
                    }
                }
            }

            /*
             * 边界空格被删除后，可能留下空 Text token。
             * 删除它们可避免后续分句与测量处理无意义的空节点。
             */
            for (int i = tokens.Count - 1;
                 i >= 0;
                 i--)
            {
                FormulaToken token = tokens[i];

                if (token != null &&
                    token.Kind ==
                        FormulaTokenKind.Text &&
                    string.IsNullOrEmpty(
                        token.Content))
                {
                    tokens.RemoveAt(i);
                }
            }
        }

        private static string TrimTrailingFormulaSpaceForCjk(
            string value)
        {
            if (string.IsNullOrEmpty(value))
                return value;

            int visibleIndex =
                value.Length - 1;

            while (visibleIndex >= 0 &&
                   IsAsciiLayoutSpace(
                       value[visibleIndex]))
            {
                visibleIndex--;
            }

            if (visibleIndex < 0 ||
                !IsCjkOrCjkPunctuation(
                    value[visibleIndex]))
            {
                return value;
            }

            return value.Substring(
                0,
                visibleIndex + 1);
        }

        private static string TrimLeadingFormulaSpaceForCjk(
            string value)
        {
            if (string.IsNullOrEmpty(value))
                return value;

            int visibleIndex = 0;

            while (visibleIndex < value.Length &&
                   IsAsciiLayoutSpace(
                       value[visibleIndex]))
            {
                visibleIndex++;
            }

            if (visibleIndex >= value.Length ||
                !IsCjkOrCjkPunctuation(
                    value[visibleIndex]))
            {
                return value;
            }

            return value.Substring(
                visibleIndex);
        }

        private static bool IsAsciiLayoutSpace(
            char value)
        {
            return
                value == ' ' ||
                value == '\t';
        }

        private static bool IsCjkOrCjkPunctuation(
            char value)
        {
            int code = value;

            return
                // CJK Unified Ideographs
                (code >= 0x4E00 &&
                 code <= 0x9FFF) ||

                // CJK Extension A
                (code >= 0x3400 &&
                 code <= 0x4DBF) ||

                // Hiragana / Katakana
                (code >= 0x3040 &&
                 code <= 0x30FF) ||

                // Hangul
                (code >= 0xAC00 &&
                 code <= 0xD7AF) ||

                // CJK Symbols and Punctuation
                (code >= 0x3000 &&
                 code <= 0x303F) ||

                // Fullwidth forms
                (code >= 0xFF00 &&
                 code <= 0xFFEF);
        }

        private static bool TryReadFormula(
            string source,
            int start,
            out int consumed,
            out string latex,
            out bool displayStyle)
        {
            consumed = 0;
            latex = null;
            displayStyle = false;

            if (start >= source.Length)
                return false;

            if (StartsWith(source, start, "$$"))
            {
                return TryReadDelimited(
                    source,
                    start,
                    "$$",
                    "$$",
                    true,
                    out consumed,
                    out latex,
                    out displayStyle);
            }

            if (StartsWith(source, start, "\\["))
            {
                return TryReadDelimited(
                    source,
                    start,
                    "\\[",
                    "\\]",
                    true,
                    out consumed,
                    out latex,
                    out displayStyle);
            }

            if (StartsWith(source, start, "\\("))
            {
                return TryReadDelimited(
                    source,
                    start,
                    "\\(",
                    "\\)",
                    false,
                    out consumed,
                    out latex,
                    out displayStyle);
            }

            if (source[start] == '$' &&
                !IsEscaped(source, start))
            {
                return TryReadSingleDollar(
                    source,
                    start,
                    out consumed,
                    out latex,
                    out displayStyle);
            }

            return false;
        }

        private static bool TryReadDelimited(
            string source,
            int start,
            string opener,
            string closer,
            bool isDisplay,
            out int consumed,
            out string latex,
            out bool displayStyle)
        {
            consumed = 0;
            latex = null;
            displayStyle = isDisplay;

            int contentStart = start + opener.Length;
            int end = source.IndexOf(
                closer,
                contentStart,
                StringComparison.Ordinal);

            if (end < 0)
                return false;

            latex = source.Substring(
                contentStart,
                end - contentStart);

            consumed =
                end + closer.Length - start;

            return true;
        }

        private static bool TryReadSingleDollar(
            string source,
            int start,
            out int consumed,
            out string latex,
            out bool displayStyle)
        {
            consumed = 0;
            latex = null;
            displayStyle = false;

            for (int i = start + 1; i < source.Length; i++)
            {
                if (source[i] == '$' &&
                    !IsEscaped(source, i))
                {
                    // $$ 由前面的分支处理；这里不吞并双美元块。
                    if (i + 1 < source.Length &&
                        source[i + 1] == '$')
                    {
                        continue;
                    }

                    latex = source.Substring(
                        start + 1,
                        i - start - 1);

                    consumed = i - start + 1;
                    return true;
                }
            }

            return false;
        }

        private static bool StartsWith(
            string source,
            int index,
            string value)
        {
            if (index + value.Length > source.Length)
                return false;

            return string.CompareOrdinal(
                source,
                index,
                value,
                0,
                value.Length) == 0;
        }

        private static bool IsEscaped(
            string source,
            int index)
        {
            int slashCount = 0;

            for (int i = index - 1;
                 i >= 0 && source[i] == '\\';
                 i--)
            {
                slashCount++;
            }

            return (slashCount & 1) != 0;
        }

        private static void FlushText(
            List<FormulaToken> result,
            StringBuilder text)
        {
            if (text.Length == 0)
                return;

            result.Add(FormulaToken.Text(text.ToString()));
            text.Clear();
        }
    }
}