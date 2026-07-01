/*
 * =================================================================================================
 * Formula3DTextApi.cs
 * =================================================================================================
 *
 * 【职责】
 *   Formula3D 的稳定、公开配置入口。当前只管理“整句打印完成后多久自动掉落”。
 *
 * 【约定】
 *   AutoDropDelaySeconds >= 0 ：启用自动掉落；0 表示下一帧立即掉落。
 *   AutoDropDelaySeconds <  0 ：关闭自动掉落，由 math3d_drop 或其他代码手动触发。
 *
 * 【为什么单独放一个 API 类】
 *   GameUIManager 是复杂的 Unity 生命周期实现，不应让其他模块直接修改它的私有字段。
 *   将可配置项集中在这里，未来可安全接入 config.json、设置界面或 AI tool_call。
 *
 * 【维护陷阱】
 *   1. 不要把“负数”改成 0；负数是关闭自动掉落的哨兵值。
 *   2. 这里不调用任何 Unity API，因此可被普通 C# 代码安全读取。
 *   3. 若新增设置，仍应由 GameUIManager 在主线程实际执行 Unity 行为。
 * =================================================================================================
 */
using System;

namespace MakemitAGA.Dialogue
{
    /// <summary>
    /// MakemitAGA 主项目的 Formula3D 稳定配置接口。
    ///
    /// 说明：
    ///   - AutoDropDelaySeconds >= 0：一句内容打印完成后，等待该秒数并自动执行物理掉落；
    ///   - AutoDropDelaySeconds < 0：关闭自动掉落，继续由 math3d_drop 手动触发；
    ///   - 该设置是当前游戏进程内的全局值，场景切换不会重置，游戏重启后恢复默认值。
    ///
    /// 未来可以从 config.json 读取配置并调用：
    ///   Formula3DTextApi.SetAutoDropDelaySeconds(value);
    /// </summary>
    public static class Formula3DTextApi
    {
        // 与原 GameUIManager “打印完后留出阅读时间再掉落”的风格接近。
        // 用户可以通过公开接口或内置调试命令随时修改。
        private static float _autoDropDelaySeconds = 4.0f;

        public static float AutoDropDelaySeconds
        {
            get { return _autoDropDelaySeconds; }
        }

        public static bool AutoDropEnabled
        {
            get { return _autoDropDelaySeconds >= 0f; }
        }

        /// <summary>
        /// 设置全局阅读等待时间。最大限制为一小时，防止配置错误造成几乎永久卡住。
        /// 负数统一转换为 -1，表示关闭自动掉落。
        /// </summary>
        public static void SetAutoDropDelaySeconds(float seconds)
        {
            if (float.IsNaN(seconds) ||
                float.IsInfinity(seconds))
            {
                throw new ArgumentOutOfRangeException(
                    nameof(seconds),
                    "Auto-drop delay must be a finite number.");
            }

            _autoDropDelaySeconds =
                seconds < 0f
                    ? -1f
                    : Math.Min(seconds, 3600f);
        }

        public static void DisableAutoDrop()
        {
            _autoDropDelaySeconds = -1f;
        }
    }
}