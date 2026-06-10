/*
 * [文件说明]: 插件主入口与生命周期管理
 * 
 * [分析过程]:
 * 1. 我们需要一个 BepInEx 插件入口来加载 Harmony 补丁。
 * 2. 为了提升用户体验，我们不希望用户手动启动 Python 后端。因此引入了 Process 类来自动管理 exe 的启动与关闭。
 * 3. 引入了 MainThreadRunner，因为很多 Unity API (如 Instantiate) 只能在主线程调用，而网络回调往往在子线程。
 * 
 * [主要功能]:
 * 1. BepInEx.Load(): 初始化日志，设置控制台编码。
 * 2. 挂载 MainThreadRunner 组件。
 * 3. StartBackendServer(): 自动寻找并启动 Plugins 目录下的 "OnlineAIApiServer.exe"。
 * 4. KillBackend(): 游戏退出时自动杀掉 Python 进程。
 * 5. 加载所有 Harmony 补丁类。
 */
using System.Diagnostics;
using System.IO;
using BepInEx;
using BepInEx.Logging;
using HarmonyLib;
using MakemitAGA.Mita_self;
using MakemitAGA.Mita_self.Mita_tools;
using MakemitAGA.World;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MakemitAGA
{
    [BepInPlugin("com.yourname.miside.freedialogue", "Miside AI Modular", "0.1.0")]
    public class Plugin : BepInEx.Unity.IL2CPP.BasePlugin
    {
        internal static ManualLogSource Logger;
        public static MainThreadRunner Runner { get; private set; }
        private Process _backendProcess;

        public override void Load()
        {
            Logger = Log;
            try { System.Console.OutputEncoding = System.Text.Encoding.UTF8; } catch { }

            Logger.LogInfo("Miside AI 模块化重构版已启动!");

            ClothChange.Init();
            // 当任何一个场景加载完毕时，都会触发 OnSceneLoaded
            SceneManager.sceneLoaded += (System.Action<Scene, LoadSceneMode>)OnSceneLoaded;
            // 挂载协程运行器
            Runner = this.AddComponent<MainThreadRunner>();

            // 只注册 Mita_sit 的 IL2CPP 类型，不在 Load 阶段立刻 AddComponent。
            // 组件会在第一次执行 sit(...) 时懒加载，避免插件加载期因自定义 MonoBehaviour 泛型 AddComponent 失败而整包加载失败。
            Mita_sit.EnsureIl2CppTypeRegistered();

            // 启动后端
            StartBackendServer();

            // 应用补丁
            var harmony = new Harmony("com.yourname.miside.freedialogue");
            TopSurfaceSeatProxyIntegration.Init(Logger, harmony);
            harmony.PatchAll(typeof(VisionPatches));
            harmony.PatchAll(typeof(DialoguePatches));
            harmony.PatchAll(typeof(Patch_Location3WalkToToilet));
            harmony.PatchAll(typeof(Patch_AnimatorFunctions));
            harmony.PatchAll(typeof(MitaSitOwnershipPatches));
            harmony.PatchAll(typeof(ClothesPatch));

        }

        private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
        {
            // 每次场景加载，旧的物体肯定被销毁了。
            // 必须先清理静态引用，防止 EnvironmentManager 拿着前朝的剑斩本朝的官。
            EnvironmentManager.ClearState();

            // 然后重新初始化，抓取当前场景的新物体
            EnvironmentManager.Init();
        }

        public override bool Unload()
        {
            KillBackend();
            return true;
        }

        private void OnApplicationQuit()
        {
            KillBackend();
        }

        private void StartBackendServer()
        {
            string exePath = Path.Combine(Paths.PluginPath, "OnlineAIApiServer.exe");
            if (!File.Exists(exePath)) return;

            try
            {
                ProcessStartInfo startInfo = new ProcessStartInfo();
                startInfo.FileName = exePath;
                startInfo.WorkingDirectory = Paths.PluginPath;
                startInfo.UseShellExecute = true;
                _backendProcess = Process.Start(startInfo);
                Logger.LogInfo("Python后端已启动。");
            }
            catch (System.Exception e) { Logger.LogError($"后端启动失败: {e.Message}"); }
        }

        private void KillBackend()
        {
            if (_backendProcess != null && !_backendProcess.HasExited)
                try { _backendProcess.Kill(); } catch { }
        }
    }

    public class MainThreadRunner : MonoBehaviour { }
}

/*
=== Plugin.cs 核心架构 ===

1. 核心变量声明：
   - internal static ManualLogSource Logger;
     作用：定义一个全局静态日志记录器。使用 static 是为了让项目中其他脚本（如 DialoguePatches）可以直接通过 Plugin.Logger 访问日志系统，无需传递实例。
   
   - public static MainThreadRunner Runner { get; private set; }
     作用：定义全局的主线程运行器。
     原理：Unity 的核心 API（如创建物体、修改UI）是非线程安全的，必须在主线程运行。而网络请求通常在后台线程。这个对象充当桥梁，让后台线程可以委托它在主线程执行任务。

   - private Process _backendProcess;
     作用：保存外部 Python AI 程序的进程句柄。我们需要这个变量来监控 Python 程序的状态，并在游戏退出时通过它来关闭 Python 程序，防止内存泄漏。

2. Load() 函数：
   - 作用：BepInEx IL2CPP 插件的入口点（相当于普通 Unity 脚本的 Awake 或 Start）。
   - 关键操作：
     a. Runner = this.AddComponent<MainThreadRunner>();
        将主线程运行器组件挂载到当前插件的 GameObject 上，使其生效。
     b. StartBackendServer();
        启动 Python 后端。
     c. harmony.PatchAll(...);
        自动扫描并应用所有带有 [HarmonyPatch] 标签的钩子函数，激活游戏修改逻辑。

3. 生命周期与清理函数：
   - public override bool Unload()
     作用：BepInEx 特有的生命周期函数。当插件被管理器卸载或热重载时调用。
     返回值：返回 true 表示“我已清理完毕，允许卸载”；返回 false 表示拒绝卸载。这是与加载器的“协商”过程。
     
   - private void OnApplicationQuit()
     作用：Unity 引擎标准的生命周期函数。当游戏窗口关闭（Alt+F4 或点击退出）时触发。
     区别：这是“强制命令”，游戏即将结束，必须在此处做最后的清理（如杀死 Python 进程），没有拒绝的权利。

   - private void KillBackend()
     作用：安全关闭 Python 进程的封装函数。
     逻辑：先检查进程是否存在且未退出，然后尝试 Kill。使用 try-catch 包裹是为了防止进程已经结束时报错导致游戏崩溃。

4. 辅助类定义：
   - public class MainThreadRunner : MonoBehaviour { }
     作用：定义一个空的 MonoBehaviour 类。
     意义：虽然它里面没有代码，但只有继承了 MonoBehaviour 的组件才能使用 StartCoroutine（开启协程）。我们需要利用它作为载体，把异步的网络回调代码“拉回”到 Unity 的主线程循环中执行。

5. ProcessStartInfo：
   作用：C# 系统类，用于配置启动外部程序（Python）的参数，如文件路径、是否隐藏黑窗口等。
*/