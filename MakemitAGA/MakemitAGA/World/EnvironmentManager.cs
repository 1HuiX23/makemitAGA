/*
 * [模块名称]: 环境与特效管理器 (EnvironmentManager)
 * [版本]: v1.2 (Final Stable)
 * 
 * [功能描述]: 
 *    1. 全局光照接管：绕过游戏原生易报错的更新逻辑，手动控制时间 (0.0-1.0) 和颜色。
 *    2. 自动校准 (Auto-Sync)：初始化时自动吸取当前场景的完美光照作为基准，适应不同关卡。
 *    3. 平滑过渡系统：支持协程 (Coroutine) 实现时间和颜色的渐变效果。
 *    4. 视觉特效集成：黑屏、血屏、反色、花屏、电视闪烁等恐怖/叙事特效。
 *    
 * [架构说明]:
 *    - 本类为静态类，由 Plugin.cs 在 OnSceneLoaded 事件中通过 Init() 唤醒。
 *    - 为了防止跨场景引用失效，必须在每次加载场景前调用 ClearState()。
 */

using System;
using System.Collections;
using BepInEx.Unity.IL2CPP.Utils.Collections; // 关键：提供 .WrapToIl2Cpp() 扩展方法，用于协程
using UnityEngine;
using static Location21_World;

namespace MakemitAGA.World
{
    public static class EnvironmentManager
    {
        // =========================================================
        // 1. 引用缓存 (Cache References)
        // =========================================================
        // 我们缓存这些组件引用，避免在每一帧渲染时重复调用 GameObject.Find，提升性能。

        // 游戏原本控制时间数据的核心脚本
        private static Location21_World _worldController;
        // 游戏原本控制后期特效(Post-Processing)的脚本
        private static PlayerCameraEffects _cameraEffects;
        // 控制黑屏过渡 UI 的脚本
        private static BlackScreen _blackScreenController;

        private static Transform _worldRoot;

        // 室内灯光物体缓存（用于实现“天黑自动开灯”逻辑）
        private static GameObject _mainLight;   // 主卧/客厅灯
        private static GameObject _toiletLight; // 厕所灯

        // 存放 Colorful FX 滤镜脚本的子物体容器 (通常位于相机层级下)
        private static GameObject _cameraPersonsObj;

        // =========================================================
        // 2. 状态数据 (State Data)
        // =========================================================

        // 当前正在使用的环境氛围色。
        // 协程过渡时会不断更新这个值，保证下一次过渡从当前颜色无缝开始。
        private static Color _currentAmbienceColor = new Color(1f, 0.763f, 0.636f, 0.149f); // 默认暖白

        // 备份游戏刚启动时的"完美原色"，用于 ResetColor 指令一键还原。
        private static Color _backupDefaultColor;
        private static bool _hasBackup = false;

        // =========================================================
        // 3. 生命周期管理 (Lifecycle)
        // =========================================================

        /// <summary>
        /// 清理静态引用。
        /// <para>必须在 Plugin.OnSceneLoaded 的第一步调用。</para>
        /// <para>防止在新场景中操作已经被销毁的旧对象（会导致 Unity 崩溃或报错）。</para>
        /// </summary>
        public static void ClearState()
        {
            _worldController = null;
            _cameraEffects = null;
            _blackScreenController = null;
            _worldRoot = null;
            _mainLight = null;
            _toiletLight = null;
            _cameraPersonsObj = null;

            // 注意：_currentAmbienceColor 和 _backupDefaultColor 不需要清空
            // 保留它们可以让新场景继承之前的颜色设置，体验更连贯。

            Plugin.Logger.LogInfo("[EnvironmentManager] 静态引用已清理。");
        }

        /// <summary>
        /// 初始化环境管理器。
        /// <para>查找场景物体，注入组件，并进行光照校准。</para>
        /// </summary>
        public static void Init()
        {
            // 严谨判空：检查引用是否为 null，或者 Unity C++ 对象是否已被销毁
            if (_worldController != null && !_worldController.Equals(null)) return;

            // 1. 查找世界根节点
            GameObject worldObj = GameObject.Find("World");
            if (worldObj == null)
            {
                // 在主菜单或加载界面找不到 World 是正常的，静默返回即可
                return;
            }
            _worldRoot = worldObj.transform;

            // 2. 接管 Location21_World
            _worldController = worldObj.GetComponent<Location21_World>();
            bool isNewComponent = false;

            // 如果原生场景没有挂这个脚本(比如某些室内关卡)，我们手动挂上去
            if (_worldController == null)
            {
                _worldController = worldObj.AddComponent<Location21_World>();
                isNewComponent = true;
                Plugin.Logger.LogInfo("[EnvironmentManager] 手动注入 Location21_World 组件。");
            }

            // 【关键步骤】禁用组件！
            // 原因：游戏原生的 Location21_World.Update() 写得非常不健壮，
            // 一旦缺少某些引用（如材质数组）就会每帧报错。
            // 我们禁用它，只把它当作存放数据的容器，渲染逻辑由我们接管。
            _worldController.enabled = false;

            // 3. 初始化数据结构
            // 必须手动初始化 timeDay 类，否则访问会报空指针
            if (_worldController.timeDay == null) _worldController.timeDay = new LocationTimeDay();
            var td = _worldController.timeDay;

            // 填充默认颜色配置 (参考原 Mod 数值)
            td.colorMorning = new Color(1f, 0.718f, 0f, 0.05f);
            td.colorDay = new Color(1f, 0.763f, 0.636f, 0.149f);
            td.colorEvening = new Color(0.787f, 1f, 0.983f, 0.05f);
            td.colorNight = new Color(0.024f, 0f, 0.32f, 1f);

            // 太阳颜色默认同步环境色
            td.colorSunMorning = td.colorMorning;
            td.colorSunDay = td.colorDay;
            td.colorSunEvening = td.colorEvening;
            td.colorSunNight = td.colorNight;

            // 填充空数组，防止任何潜在的原生代码遍历报错
            td.particlesDust = new UnityEngine.ParticleSystem[0];
            td.particleSunLight = new LocationTimeDayParticleLight[0];
            td.particlesDustRate = new AnimationCurve();

            // 4. 绑定场景物体
            Transform sunTrans = _worldRoot.Find("House/Sun");
            if (sunTrans != null) td.sun = sunTrans.GetComponent<Light>();

            // 查找室内灯光组 (用于 SetActive 开关)
            string lightPath = "House/HouseGameNormal Tamagotchi/HouseGame Tamagotchi/Lighting/";
            Transform mainL = _worldRoot.Find(lightPath + "Main Lighting");
            Transform toiletL = _worldRoot.Find(lightPath + "Toilet Lighting");
            if (mainL != null) _mainLight = mainL.gameObject;
            if (toiletL != null) _toiletLight = toiletL.gameObject;

            // 5. 查找特效组件
            _cameraEffects = UnityEngine.Object.FindObjectOfType<PlayerCameraEffects>();
            if (_cameraEffects != null)
            {
                // Colorful FX 的脚本通常挂在相机下的 CameraPersons 子物体上
                Transform camChild = _cameraEffects.transform.Find("CameraPersons");
                if (camChild != null) _cameraPersonsObj = camChild.gameObject;
            }
            _blackScreenController = UnityEngine.Object.FindObjectOfType<BlackScreen>();

            // 6. 【吸星大法】自动校准逻辑
            // 如果是我们新注入的组件，说明此时游戏画面是原生的、完美的。
            // 我们直接吸取当前的 RenderSettings 和 Light 颜色作为我们的“白天标准”。
            if (isNewComponent)
            {
                // 强制逻辑时间为白天 (0.5)
                _worldController.day = 0.5f;
                _worldController.SetTimeDay(0.5f);

                // 吸取真实颜色
                Color realSky = RenderSettings.ambientSkyColor;
                Color realSun = (td.sun != null) ? td.sun.color : Color.white;

                // 篡改默认配置：把当前颜色定义为“标准白天色”
                td.colorDay = realSky;
                td.colorSunDay = realSun;
                _currentAmbienceColor = realSky;

                // 备份原始颜色，用于 ResetColor
                if (!_hasBackup)
                {
                    _backupDefaultColor = realSky;
                    _hasBackup = true;
                    Plugin.Logger.LogInfo($"[EnvironmentManager] 已备份原始环境色: {realSky}");
                }
            }

            Plugin.Logger.LogInfo("[EnvironmentManager] 环境系统就绪。");
        }

        // =========================================================
        // 4. 对外接口：时间与颜色 (Public API)
        // =========================================================

        /// <summary>
        /// 获取当前时间信息的字符串 (调试用)。
        /// </summary>
        public static string GetCurrentTimeInfo()
        {
            Init();
            if (_worldController == null) return "未初始化";
            return $"当前时间值: {_worldController.day:F4}";
        }

        /// <summary>
        /// 设置游戏时间。
        /// </summary>
        /// <param name="targetTime">0.0 (黑夜) ~ 0.5 (正午) ~ 1.0 (黑夜)</param>
        /// <param name="duration">过渡耗时(秒)。0表示瞬间切换。</param>
        public static string SetTime(float targetTime, float duration)
        {
            Init();
            if (_worldController == null) return "环境控制器未就绪";

            // 启动平滑过渡协程
            if (duration > 0.1f)
            {
                // 注意：在非 MonoBehaviour 类中启动协程，需要依赖 Plugin.Runner
                // 并且需要 WrapToIl2Cpp() 转换，这是 IL2CPP 插件的标准写法
                Plugin.Runner.StartCoroutine(TimeTransitionRoutine(targetTime, duration).WrapToIl2Cpp());
                return $"开始在 {duration}秒 内过渡时间...";
            }

            // 瞬间切换
            _worldController.day = targetTime;
            _worldController.SetTimeDay(targetTime);
            UpdateVisualsManual(); // 立即刷新画面
            return $"时间已设为 {targetTime:F2}";
        }

        /// <summary>
        /// 设置环境氛围颜色 (如红色恐怖氛围)。
        /// </summary>
        public static string SetColor(float r, float g, float b, float duration)
        {
            Init();
            if (_worldController == null) return "环境控制器未就绪";

            Color target = new Color(r, g, b, 1f);

            if (duration > 0.1f)
            {
                Plugin.Runner.StartCoroutine(ColorTransitionRoutine(target, duration).WrapToIl2Cpp());
                return $"开始在 {duration}秒 内过渡颜色...";
            }

            _worldController.timeDay.colorDay = target;
            _currentAmbienceColor = target;
            UpdateVisualsManual();
            return "氛围颜色已修改";
        }

        /// <summary>
        /// 还原为游戏默认颜色 (后悔药)。
        /// </summary>
        public static string ResetColor(float duration)
        {
            if (!_hasBackup) return "无原始颜色备份";
            // 调用 SetColor 恢复备份的颜色
            return SetColor(_backupDefaultColor.r, _backupDefaultColor.g, _backupDefaultColor.b, duration);
        }

        // =========================================================
        // 5. 对外接口：视觉特效 (Visual Effects)
        // =========================================================

        /// <summary>
        /// 控制黑屏遮罩。
        /// </summary>
        /// <param name="active">true=变黑, false=变亮</param>
        /// <param name="instant">true=瞬间, false=渐变</param>
        public static string SetBlackScreen(bool active, bool instant)
        {
            Init();
            if (_blackScreenController == null) return "未找到黑屏组件";
            _blackScreenController.HoldBlack(active, instant);
            return active ? "黑屏已开启" : "黑屏已关闭";
        }

        /// <summary>
        /// 开启/关闭血屏暗角 (受伤效果)。
        /// </summary>
        public static string SetBlood(bool active)
        {
            Init();
            if (_cameraEffects == null) return "未找到特效组件";
            _cameraEffects.FastVegnetteActive(active);
            return active ? "血屏已开启" : "血屏已关闭";
        }

        /// <summary>
        /// 开启/关闭反色滤镜 (精神污染)。
        /// </summary>
        public static string SetNegative(bool active)
        {
            return ToggleFilter("Negative", active); // 对应 Colorful.Negative
        }

        /// <summary>
        /// 通用滤镜开关 (私有辅助方法)。
        /// 通过反射遍历组件名称来开关滤镜。
        /// </summary>
        private static string ToggleFilter(string componentName, bool active)
        {
            Init();
            if (_cameraPersonsObj == null) return "未找到滤镜容器";

            var components = _cameraPersonsObj.GetComponents<MonoBehaviour>();
            foreach (var comp in components)
            {
                // 使用 IL2CPP 类型名称进行匹配 (忽略命名空间)
                // 例如: "Colorful.Negative" -> "Negative"
                string typeName = comp.GetIl2CppType().Name;
                if (typeName.Equals(componentName, StringComparison.OrdinalIgnoreCase))
                {
                    comp.enabled = active;
                    return active ? $"滤镜 {componentName} 已开启" : $"滤镜 {componentName} 已关闭";
                }
            }
            return $"未找到滤镜 {componentName}";
        }

        /// <summary>
        /// [协程] 瞬态特效：花屏 (Datamosh)。
        /// 画面涂抹/卡顿效果，模拟视频编码错误。
        /// </summary>
        public static IEnumerator GlitchRoutine(float duration)
        {
            Init();
            if (_cameraEffects == null) yield break;

            _cameraEffects.EffectDatamosh(true); // 开启
            yield return new WaitForSeconds(duration);
            _cameraEffects.EffectDatamosh(false); // 自动关闭
        }

        /// <summary>
        /// [协程] 瞬态特效：传送故障 (Colorful.Glitch)。
        /// RGB 色彩分离抖动，适合配合瞬间黑屏使用，模拟传送。
        /// </summary>
        public static IEnumerator TeleportGlitchRoutine(float duration)
        {
            Init();
            if (_cameraPersonsObj == null) yield break;

            ToggleFilter("Glitch", true); // 手动开启组件
            yield return new WaitForSeconds(duration);
            ToggleFilter("Glitch", false); // 自动关闭
        }

        /// <summary>
        /// 触发电视开关机闪烁特效。
        /// </summary>
        public static void TriggerTV()
        {
            Init();
            if (_cameraEffects != null) _cameraEffects.EffectClickTelevision();
        }

        // =========================================================
        // 6. 核心逻辑：协程与手动渲染管线 (Core Pipeline)
        // =========================================================

        // 时间过渡协程
        private static IEnumerator TimeTransitionRoutine(float target, float duration)
        {
            float start = _worldController.day;
            float t = 0f;
            while (t < duration)
            {
                t += Time.deltaTime;
                float p = t / duration; // 进度 0-1
                float cur = Mathf.Lerp(start, target, p); // 插值

                // 应用到控制器并刷新画面
                _worldController.day = cur;
                _worldController.SetTimeDay(cur);
                UpdateVisualsManual();

                yield return null; // 等待下一帧
            }
            // 确保最终值精确
            _worldController.day = target;
            UpdateVisualsManual();
        }

        // 颜色过渡协程
        private static IEnumerator ColorTransitionRoutine(Color target, float duration)
        {
            Color start = _currentAmbienceColor;
            float t = 0f;
            while (t < duration)
            {
                t += Time.deltaTime;
                float p = t / duration;
                Color cur = Color.Lerp(start, target, p);

                // 更新白天的基准色，这样 UpdateVisualsManual 里的插值逻辑就会使用新颜色
                _worldController.timeDay.colorDay = cur;
                _currentAmbienceColor = cur;

                // 刷新画面 (必须调用 SetTimeDay 触发计算)
                UpdateVisualsManual();

                yield return null;
            }
        }

        // 手动渲染引擎 (替代游戏原生的 UpdateTimeDay)
        // 根据 _worldController.day 的值，手动设置天空、太阳和灯光
        private static void UpdateVisualsManual()
        {
            if (_worldController == null || _worldController.timeDay == null) return;

            var td = _worldController.timeDay;
            float d = _worldController.day;

            // 0.0 - 0.25 (深夜 -> 早晨)
            if (d < 0.25f) ApplyPhase(td.colorNight, td.colorMorning, td.colorSunNight, td.colorSunMorning, d, true);
            // 0.25 - 0.5 (早晨 -> 正午)
            else if (d < 0.5f) ApplyPhase(td.colorMorning, td.colorDay, td.colorSunMorning, td.colorSunDay, d, false);
            // 0.5 - 0.75 (正午 -> 傍晚)
            else if (d < 0.75f) ApplyPhase(td.colorDay, td.colorEvening, td.colorSunDay, td.colorSunEvening, d, false);
            // 0.75 - 1.0 (傍晚 -> 深夜)
            else ApplyPhase(td.colorEvening, td.colorNight, td.colorSunEvening, td.colorSunNight, d, true);
        }

        // 插值计算并应用
        // t: 当前总时间 (如 0.6)
        // l: 是否开灯 (true/false)
        private static void ApplyPhase(Color c1, Color c2, Color s1, Color s2, float t, bool l)
        {
            // 将当前时间段的时间归一化到 0-1 (例如 0.25-0.5 区间内，0.375 会变成 0.5)
            float p = (t % 0.25f) * 4f;
            p = Mathf.Clamp01(p);

            // 插值计算天空盒颜色
            RenderSettings.ambientSkyColor = Color.Lerp(c1, c2, p);

            // 插值计算太阳颜色
            if (_worldController.timeDay.sun != null)
                _worldController.timeDay.sun.color = Color.Lerp(s1, s2, p);

            // 室内灯光开关控制
            if (_mainLight != null && _mainLight.activeSelf != l) _mainLight.SetActive(l);
            if (_toiletLight != null && _toiletLight.activeSelf != l) _toiletLight.SetActive(l);
        }
    }
}