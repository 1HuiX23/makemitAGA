/*
 * =================================================================================================
 * Il2CppAssetBundleLoader.cs
 * =================================================================================================
 *
 * 这个文件负责：
 *   用 IL2CPP.ResolveICall 调用 Unity AssetBundle 的底层接口。
 *
 * 为什么要这样写：
 * -------------------------------------------------------------------------------------------------
 * 在当前 MiSide + BepInEx IL2CPP 环境中，普通 AssetBundle.LoadFromFile 可能报错或不稳定。
 * 你们之前的主项目和 MitaAI 项目也都遇到过类似问题，所以我们继续走 iCall 路线。
 *
 * 这里封装：
 *   UnityEngine.AssetBundle::LoadFromFile_Internal(System.String,System.UInt32,System.UInt64)
 *   UnityEngine.AssetBundle::LoadAsset_Internal(System.String,System.Type)
 *   UnityEngine.AssetBundle::GetAllAssetNames
 *   UnityEngine.AssetBundle::Unload
 *
 * 已经踩过的坑：
 * -------------------------------------------------------------------------------------------------
 * 1. 同一个 Unity 进程里不能重复加载同一个 AssetBundle。
 *    如果 Unity 报：
 *      can't be loaded because another AssetBundle with the same files is already loaded
 *    或 LoadFromFile_Internal returned null，
 *    通常需要完全重启游戏，并确保 plugins 里只有一个测试 DLL 在加载这个 AB。
 *
 * 2. 静态构造里一次性 ResolveICall 如果有一个失败，会导致整个类不可用。
 *    所以这里用 EnsureResolved() 分项 resolve，失败也会打日志，而不是直接炸掉。
 *
 * 3. AB 资源名会变成小写路径：
 *      assets/depthseat/depthseat_eyedepthreplacement.shader
 *      assets/depthseat/depthtoeye_mat.mat
 *    所以 FindAssetName 用 Path.GetFileNameWithoutExtension 做宽松匹配。
 *
 * 未来设计：
 * -------------------------------------------------------------------------------------------------
 * 等这个加载器稳定后，不建议继续频繁改。
 * 主项目里可以把它放进一个通用工具命名空间，例如 MakemitAGA.Runtime.UnityAssetTools。
 *
 * =================================================================================================
 */

using System;
using System.IO;
using Il2CppInterop.Runtime;
using Il2CppInterop.Runtime.InteropTypes.Arrays;
using UnityEngine;

namespace TopSurfaceSeatProxyTest
{
    internal static class ICallAssetBundleLoader
    {
        private delegate IntPtr LoadFromFileInternalDelegate(IntPtr path, uint crc, ulong offset);
        private delegate IntPtr LoadAssetInternalDelegate(IntPtr bundle, IntPtr name, IntPtr type);
        private delegate IntPtr GetAllAssetNamesDelegate(IntPtr bundle);
        private delegate void UnloadDelegate(IntPtr bundle, bool unloadAllLoadedObjects);

        private static LoadFromFileInternalDelegate _loadFromFileInternal;
        private static LoadAssetInternalDelegate _loadAssetInternal;
        private static GetAllAssetNamesDelegate _getAllAssetNamesInternal;
        private static UnloadDelegate _unloadInternal;

        private static bool _resolved;

        private static IntPtr _cachedBundlePtr = IntPtr.Zero;
        private static string _cachedBundlePath;

        private static Material _cachedMaterial;
        private static Shader _cachedEyeDepthShader;

        public static bool TryLoadMaterialByName(string bundlePath, string materialNameWithoutExt, out Material mat)
        {
            mat = null;

            if (_cachedMaterial != null)
            {
                mat = _cachedMaterial;
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Reusing cached depth material: " + mat.name);
                return true;
            }

            IntPtr bundlePtr;

            if (!TryGetBundle(bundlePath, out bundlePtr)) return false;

            string assetName = FindAssetName(bundlePtr, materialNameWithoutExt);

            if (string.IsNullOrEmpty(assetName))
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] Material asset not found: " + materialNameWithoutExt);
                return false;
            }

            try
            {
                IntPtr assetPtr = _loadAssetInternal(
                    bundlePtr,
                    IL2CPP.ManagedStringToIl2Cpp(assetName),
                    Il2CppType.Of<Material>().Pointer);

                if (assetPtr == IntPtr.Zero)
                {
                    Debug.LogError("[TopSurfaceSeatProxyTester] LoadAsset_Internal returned null for material: " + assetName);
                    return false;
                }

                mat = new Material(assetPtr);

                if (mat == null)
                {
                    Debug.LogError("[TopSurfaceSeatProxyTester] Material wrapper creation failed.");
                    return false;
                }

                _cachedMaterial = mat;
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Loaded Material: " + assetName + " / " + mat.name);
                return true;
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] TryLoadMaterialByName exception: " + e);
                return false;
            }
        }

        public static bool TryLoadShaderByName(string bundlePath, string shaderNameWithoutExt, out Shader shader)
        {
            shader = null;

            if (_cachedEyeDepthShader != null)
            {
                shader = _cachedEyeDepthShader;
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Reusing cached shader: " + shader.name);
                return true;
            }

            IntPtr bundlePtr;

            if (!TryGetBundle(bundlePath, out bundlePtr)) return false;

            string assetName = FindAssetName(bundlePtr, shaderNameWithoutExt);

            if (string.IsNullOrEmpty(assetName))
            {
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Shader asset not found in bundle: " + shaderNameWithoutExt);
                return false;
            }

            try
            {
                IntPtr assetPtr = _loadAssetInternal(
                    bundlePtr,
                    IL2CPP.ManagedStringToIl2Cpp(assetName),
                    Il2CppType.Of<Shader>().Pointer);

                if (assetPtr == IntPtr.Zero)
                {
                    Debug.LogError("[TopSurfaceSeatProxyTester] LoadAsset_Internal returned null for shader: " + assetName);
                    return false;
                }

                shader = new Shader(assetPtr);

                if (shader == null)
                {
                    Debug.LogError("[TopSurfaceSeatProxyTester] Shader wrapper creation failed.");
                    return false;
                }

                _cachedEyeDepthShader = shader;
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Loaded Shader: " + assetName + " / " + shader.name);
                return true;
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] TryLoadShaderByName exception: " + e);
                return false;
            }
        }

        private static bool TryGetBundle(string bundlePath, out IntPtr bundlePtr)
        {
            bundlePtr = IntPtr.Zero;

            if (string.IsNullOrWhiteSpace(bundlePath) || !File.Exists(bundlePath))
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] AssetBundle path invalid: " + bundlePath);
                return false;
            }

            if (!EnsureResolved())
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] iCall delegates are unavailable. AssetBundle loading skipped.");
                return false;
            }

            if (_loadFromFileInternal == null || _loadAssetInternal == null || _getAllAssetNamesInternal == null)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] Required AssetBundle iCalls are missing.");
                return false;
            }

            if (_cachedBundlePtr != IntPtr.Zero)
            {
                bundlePtr = _cachedBundlePtr;
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Reusing cached AssetBundle pointer: " + _cachedBundlePath);
                return true;
            }

            try
            {
                bundlePtr = _loadFromFileInternal(IL2CPP.ManagedStringToIl2Cpp(bundlePath), 0u, 0UL);

                if (bundlePtr == IntPtr.Zero)
                {
                    Debug.LogError(
                        "[TopSurfaceSeatProxyTester] LoadFromFile_Internal returned null. " +
                        "Usually the same AssetBundle is already loaded in this game process. " +
                        "Fully restart the game and keep only one test DLL active. path=" + bundlePath);
                    return false;
                }

                _cachedBundlePtr = bundlePtr;
                _cachedBundlePath = bundlePath;

                Debug.LogWarning("[TopSurfaceSeatProxyTester] Loaded AssetBundle by iCall: " + bundlePath);
                return true;
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] TryGetBundle exception: " + e);
                return false;
            }
        }

        private static bool EnsureResolved()
        {
            if (_resolved) return true;

            _resolved = true;

            try
            {
                _loadFromFileInternal = IL2CPP.ResolveICall<LoadFromFileInternalDelegate>(
                    "UnityEngine.AssetBundle::LoadFromFile_Internal(System.String,System.UInt32,System.UInt64)");
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Resolved AssetBundle LoadFromFile_Internal.");
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] Failed to resolve LoadFromFile_Internal: " + e.GetType().Name + " " + e.Message);
            }

            try
            {
                _loadAssetInternal = IL2CPP.ResolveICall<LoadAssetInternalDelegate>(
                    "UnityEngine.AssetBundle::LoadAsset_Internal(System.String,System.Type)");
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Resolved AssetBundle LoadAsset_Internal.");
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] Failed to resolve LoadAsset_Internal: " + e.GetType().Name + " " + e.Message);
            }

            try
            {
                _getAllAssetNamesInternal = IL2CPP.ResolveICall<GetAllAssetNamesDelegate>(
                    "UnityEngine.AssetBundle::GetAllAssetNames");
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Resolved AssetBundle GetAllAssetNames.");
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] Failed to resolve GetAllAssetNames: " + e.GetType().Name + " " + e.Message);
            }

            try
            {
                _unloadInternal = IL2CPP.ResolveICall<UnloadDelegate>(
                    "UnityEngine.AssetBundle::Unload");
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Resolved AssetBundle Unload.");
            }
            catch (Exception e)
            {
                Debug.LogWarning("[TopSurfaceSeatProxyTester] Failed to resolve Unload. Non-fatal: " + e.GetType().Name + " " + e.Message);
            }

            return _loadFromFileInternal != null && _loadAssetInternal != null && _getAllAssetNamesInternal != null;
        }

        private static string FindAssetName(IntPtr bundlePtr, string assetNameWithoutExt)
        {
            try
            {
                IntPtr namesPtr = _getAllAssetNamesInternal(bundlePtr);

                if (namesPtr == IntPtr.Zero) return null;

                Il2CppStringArray names = new Il2CppStringArray(namesPtr);

                if (names == null || names.Length == 0) return null;

                string target = assetNameWithoutExt.ToLowerInvariant();

                Debug.LogWarning("[TopSurfaceSeatProxyTester] Asset names in depth bundle:");

                for (int i = 0; i < names.Length; i++)
                {
                    string assetName = names[i];
                    Debug.LogWarning("  " + assetName);

                    if (string.IsNullOrEmpty(assetName)) continue;

                    string fileNoExt = Path.GetFileNameWithoutExtension(assetName).ToLowerInvariant();
                    string fileFull = Path.GetFileName(assetName).ToLowerInvariant();

                    if (fileNoExt == target || fileFull == target)
                    {
                        Debug.LogWarning("[TopSurfaceSeatProxyTester] Found target asset: " + assetName);
                        return assetName;
                    }
                }

                return null;
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxyTester] FindAssetName exception: " + e);
                return null;
            }
        }
    }
}