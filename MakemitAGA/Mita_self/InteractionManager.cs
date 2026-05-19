using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices; // 新增：用于 Marshal
using BepInEx;
using Il2CppInterop.Runtime;
using HarmonyLib;

namespace MakemitAGA.Mita_self
{
    // 拦截补丁 (保持)
    [HarmonyPatch("Location3WalkToToilet", "LateUpdate")]
    public static class Patch_Location3WalkToToilet { public static bool Prefix() { return !InteractionManager.IsInteracting; } }

    [HarmonyPatch("Animator_FunctionsOverride", "Update")]
    public static class Patch_AnimatorFunctions { public static bool Prefix() { return !InteractionManager.IsInteracting; } }

    public static class InteractionManager
    {
        public static bool IsInteracting = false;

        // ICall 加载
        private delegate IntPtr LoadFromFileDelegate(IntPtr path, uint crc, ulong offset);
        private delegate IntPtr LoadAssetDelegate(IntPtr bundle, IntPtr name, IntPtr type);
        private static LoadFromFileDelegate _loadFromFileFunc;
        private static LoadAssetDelegate _loadAssetFunc;
        private static IntPtr _loadedBundlePtr = IntPtr.Zero;

        // 只缓存 Controller，不要 Avatar
        private static RuntimeAnimatorController _sitController;
        private static RuntimeAnimatorController _originalController;
        private static AnimatorOverrideController _originalAnimOver;
        private static MonoBehaviour _charLookScript;
        private static MonoBehaviour _animFuncScript;

        public static T GetCompSafe<T>(GameObject obj) where T : UnityEngine.Object
        {
            var comp = obj.GetComponent(Il2CppType.Of<T>());
            if (comp != null) return comp.TryCast<T>();
            return null;
        }

        private static void InitializeICalls()
        {
            if (_loadFromFileFunc != null) return;
            _loadFromFileFunc = IL2CPP.ResolveICall<LoadFromFileDelegate>("UnityEngine.AssetBundle::LoadFromFile_Internal(System.String,System.UInt32,System.UInt64)");
            _loadAssetFunc = IL2CPP.ResolveICall<LoadAssetDelegate>("UnityEngine.AssetBundle::LoadAsset_Internal(System.String,System.Type)");
        }

        public static void LoadResources()
        {
            if (_sitController != null) return;
            InitializeICalls();

            // 加载 Bundle (这里改回加载你自己的 mita_actions)
            if (_loadedBundlePtr == IntPtr.Zero)
            {
                var allObjects = Resources.FindObjectsOfTypeAll(Il2CppType.Of<UnityEngine.Object>());
                if (allObjects != null) { foreach (var obj in allObjects) { if (obj != null && obj.name == "mita_actions") { _loadedBundlePtr = obj.Pointer; break; } } }

                if (_loadedBundlePtr == IntPtr.Zero)
                {
                    string path = Path.Combine(Paths.PluginPath, "mita_actions");
                    if (File.Exists(path)) { IntPtr pathPtr = IL2CPP.ManagedStringToIl2Cpp(path); _loadedBundlePtr = _loadFromFileFunc(pathPtr, 0, 0); }
                }
            }

            if (_loadedBundlePtr != IntPtr.Zero)
            {
                // 只加载 Controller
                IntPtr type = Il2CppType.Of<RuntimeAnimatorController>().Pointer;
                IntPtr name = IL2CPP.ManagedStringToIl2Cpp("Override_Sit");
                IntPtr ptr = _loadAssetFunc(_loadedBundlePtr, name, type);

                if (ptr != IntPtr.Zero)
                {
                    _sitController = new RuntimeAnimatorController(ptr);
                    Plugin.Logger.LogInfo($"✅ 动作控制器加载成功: {_sitController.name}");
                }
                else
                {
                    Plugin.Logger.LogError("❌ 未找到 Override_Sit.controller");
                }
            }
        }

        public static IEnumerator PerformSit(string ignoredArg)
        {
            LoadResources();
            if (_sitController == null) yield break;

            var mita = UnityEngine.Object.FindObjectOfType<MitaPerson>();
            if (mita == null) yield break;

            var animator = GetCompSafe<Animator>(mita.gameObject);
            if (animator == null) animator = mita.GetComponentInChildren<Animator>();
            if (animator == null) yield break;

            ConsoleMain.ConsolePrintGame(">>> 启动：只换脑模式 <<<");

            IsInteracting = true;
            mita.MagnetOff();

            var agent = GetCompSafe<UnityEngine.AI.NavMeshAgent>(mita.gameObject);
            if (agent != null) { agent.isStopped = true; agent.enabled = false; }

            // 关闭 IK (必做)
            _charLookScript = FindScript(mita.gameObject, "Character_Look");
            if (_charLookScript != null)
            {
                SetBoolField(_charLookScript, "activeBodyIK", false);
                SetBoolField(_charLookScript, "canRotateBody", false);
            }

            // 备份
            if (_originalController == null) _originalController = animator.runtimeAnimatorController;
            _animFuncScript = FindScript(mita.gameObject, "Animator_FunctionsOverride");
            if (_animFuncScript != null) _originalAnimOver = GetField<AnimatorOverrideController>(_animFuncScript, "animOver");

            // 注入脚本
            var overrideCtrl = new AnimatorOverrideController(_sitController);
            if (_animFuncScript != null) SetField(_animFuncScript, "animOver", overrideCtrl);

            // 【核心】只替换 Controller，绝对不碰 Avatar
            // 不要写 animator.avatar = ...;
            animator.runtimeAnimatorController = overrideCtrl;

            animator.applyRootMotion = false;
            animator.cullingMode = AnimatorCullingMode.AlwaysAnimate;
            animator.enabled = true;

            animator.Rebind();
            animator.Update(0f);

            // 锁定位置
            float timer = 0f;
            Vector3 lockPos = mita.transform.position;
            Quaternion lockRot = mita.transform.rotation;

            while (timer < 3.0f)
            {
                mita.transform.position = lockPos;
                mita.transform.rotation = lockRot;
                timer += Time.deltaTime;

                // 守护
                if (animator.runtimeAnimatorController != overrideCtrl)
                    animator.runtimeAnimatorController = overrideCtrl;

                yield return null;
            }

            ConsoleMain.ConsolePrintGame("结束。");
        }

        public static void ResetMita()
        {
            IsInteracting = false;
            if (_charLookScript != null) { SetBoolField(_charLookScript, "activeBodyIK", true); SetBoolField(_charLookScript, "canRotateBody", true); }

            var mita = UnityEngine.Object.FindObjectOfType<MitaPerson>();
            if (mita)
            {
                var animator = GetCompSafe<Animator>(mita.gameObject);
                if (animator != null)
                {
                    if (_originalController != null) animator.runtimeAnimatorController = _originalController;
                    if (_animFuncScript != null && _originalAnimOver != null) SetField(_animFuncScript, "animOver", _originalAnimOver);
                    animator.applyRootMotion = true;
                    animator.Rebind();
                }
                var agent = GetCompSafe<UnityEngine.AI.NavMeshAgent>(mita.gameObject);
                if (agent != null) agent.enabled = true;
            }
            ConsoleMain.ConsolePrintGame("恢复。");
        }

        // ... 反射工具 (保持不变，省略以节省篇幅) ...
        private static MonoBehaviour FindScript(GameObject go, string scriptName) { var s = go.GetComponentsInChildren<MonoBehaviour>(); foreach (var i in s) if (i.GetIl2CppType().Name == scriptName) return i; return null; }
        private static Il2CppSystem.Object BoxBool(bool value) { IntPtr ptr = Marshal.AllocHGlobal(1); try { Marshal.WriteByte(ptr, value ? (byte)1 : (byte)0); IntPtr classPtr = Il2CppClassPointerStore<bool>.NativeClassPtr; IntPtr boxedPtr = IL2CPP.il2cpp_value_box(classPtr, ptr); return new Il2CppSystem.Object(boxedPtr); } finally { Marshal.FreeHGlobal(ptr); } }
        private static void SetBoolField(MonoBehaviour script, string fieldName, bool value) { var t = script.GetIl2CppType(); var f = Il2CppSystem.Reflection.BindingFlags.Instance | Il2CppSystem.Reflection.BindingFlags.Public | Il2CppSystem.Reflection.BindingFlags.NonPublic; var field = t.GetField(fieldName, f); if (field != null) { try { field.SetValue(script, BoxBool(value)); } catch { } return; } var prop = t.GetProperty(fieldName, f); if (prop != null) try { prop.SetValue(script, BoxBool(value), null); } catch { } }
        private static void SetField<T>(MonoBehaviour script, string fieldName, T value) where T : Il2CppSystem.Object { var t = script.GetIl2CppType(); var f = Il2CppSystem.Reflection.BindingFlags.Instance | Il2CppSystem.Reflection.BindingFlags.Public | Il2CppSystem.Reflection.BindingFlags.NonPublic; var field = t.GetField(fieldName, f); if (field != null) try { field.SetValue(script, value); } catch { } }
        private static T GetField<T>(MonoBehaviour script, string fieldName) where T : Il2CppSystem.Object { var t = script.GetIl2CppType(); var f = Il2CppSystem.Reflection.BindingFlags.Instance | Il2CppSystem.Reflection.BindingFlags.Public | Il2CppSystem.Reflection.BindingFlags.NonPublic; var field = t.GetField(fieldName, f); if (field != null) { var v = field.GetValue(script); if (v != null) return v.TryCast<T>(); } return default(T); }
    }
}