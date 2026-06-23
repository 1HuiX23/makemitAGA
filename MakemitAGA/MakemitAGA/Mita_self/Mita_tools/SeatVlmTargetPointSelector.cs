/*
 * SeatVlmTargetPointSelector.cs
 * 从候选 id 或精确名字中选择唯一真实 Unity 对象。
 */
using System;
using System.Collections.Generic;

using MakemitAGA.World;
namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class SeatVlmTargetPointSelector
    {
        public static bool TrySelectCandidate(
            List<DetectedObjectCandidate> candidates,
            string argument,
            out DetectedObjectCandidate selected,
            out string error)
        {
            selected = null;
            error = null;

            if (candidates == null ||
                candidates.Count == 0)
            {
                error = "当前没有候选物体。";
                return false;
            }

            string value =
                TrimQuotes(
                    (argument ?? "").Trim());

            int id;

            if (int.TryParse(
                value,
                out id))
            {
                for (int i = 0;
                     i < candidates.Count;
                     i++)
                {
                    DetectedObjectCandidate candidate =
                        candidates[i];

                    if (candidate.Id == id &&
                        candidate.IsAlive)
                    {
                        selected = candidate;
                        SeatVlmDebugVisuals.DrawSelectedBounds(
                            selected.Bounds);

                        return true;
                    }
                }

                error =
                    "候选列表中不存在 id=" +
                    id;

                return false;
            }

            var exact =
                new List<DetectedObjectCandidate>();

            for (int i = 0;
                 i < candidates.Count;
                 i++)
            {
                DetectedObjectCandidate candidate =
                    candidates[i];

                if (!candidate.IsAlive)
                    continue;

                if (string.Equals(
                        candidate.Name,
                        value,
                        StringComparison.OrdinalIgnoreCase) ||
                    string.Equals(
                        candidate.Path,
                        value,
                        StringComparison.OrdinalIgnoreCase))
                {
                    exact.Add(candidate);
                }
            }

            if (exact.Count == 1)
            {
                selected = exact[0];

                SeatVlmDebugVisuals.DrawSelectedBounds(
                    selected.Bounds);

                return true;
            }

            error =
                exact.Count > 1
                    ? "名字存在多个匹配，请改用候选 id。"
                    : "候选列表中不存在精确名字：" +
                      value;

            return false;
        }

        private static string TrimQuotes(
            string value)
        {
            if (string.IsNullOrEmpty(value))
                return value;

            if (value.Length >= 2 &&
                ((value[0] == '"' &&
                  value[value.Length - 1] == '"') ||
                 (value[0] == '\'' &&
                  value[value.Length - 1] == '\'')))
            {
                return value.Substring(
                    1,
                    value.Length - 2)
                    .Trim();
            }

            return value;
        }
    }
}
