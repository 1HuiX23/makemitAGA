/*
 * =================================================================================================
 * Il2CppAssetBundleLoader.cs
 * =================================================================================================
 *
 * 稳定工具文件：用 IL2CPP.ResolveICall 调用 Unity AssetBundle 底层接口。
 *
 * 为什么要这样写：
 *   在 MiSide + Unity IL2CPP + BepInEx 环境里，普通 AssetBundle.LoadFromFile 可能报错或不稳定。
 *   你们主项目和 MitaAI 也遇到过类似问题，所以这里继续走 iCall 路线。
 *
 * 已经遇到的坑：
 *   1. 同一个 Unity 进程不能重复加载同一 AB。报同文件已加载时，需要重启游戏，
 *      并确保 plugins 里只有一个测试 DLL 会加载 mita_actions。
 *   2. 不要在静态构造器里一次性 ResolveICall；任何一个失败都会导致整个类初始化失败。
 *      这里用 EnsureResolved() 分项 resolve。
 *   3. AB 资源名经常是小写路径，如 assets/depthseat/depthseat_eyedepthreplacement.shader。
 *      所以 FindAssetName 使用文件名无扩展名做宽松匹配。
 *
 * 未来：
 *   这个文件稳定后尽量不要改，可以移入主项目通用工具层。
 * =================================================================================================
 */

using System;
using System.IO;
using Il2CppInterop.Runtime;
using Il2CppInterop.Runtime.InteropTypes.Arrays;
using UnityEngine;

namespace MakemitAGA.Mita_self.Mita_tools
{
    internal static class ICallAssetBundleLoader
    {
        private const bool VerboseLoaderLogs = false;
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
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Reusing cached material: " + mat.name);
                return true;
            }

            IntPtr bundlePtr;
            if (!TryGetBundlePointer(bundlePath, out bundlePtr)) return false;

            string assetName = FindAssetName(bundlePtr, materialNameWithoutExt);
            if (string.IsNullOrEmpty(assetName))
            {
                Debug.LogError("[TopSurfaceSeatProxy] Material asset not found: " + materialNameWithoutExt);
                return false;
            }

            try
            {
                IntPtr assetPtr = _loadAssetInternal(bundlePtr, IL2CPP.ManagedStringToIl2Cpp(assetName), Il2CppType.Of<Material>().Pointer);
                if (assetPtr == IntPtr.Zero)
                {
                    Debug.LogError("[TopSurfaceSeatProxy] LoadAsset_Internal returned null for material: " + assetName);
                    return false;
                }

                mat = new Material(assetPtr);
                _cachedMaterial = mat;
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Loaded Material: " + assetName + " / " + mat.name);
                return mat != null;
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxy] TryLoadMaterialByName exception: " + e);
                return false;
            }
        }

        public static bool TryLoadShaderByName(string bundlePath, string shaderNameWithoutExt, out Shader shader)
        {
            shader = null;
            if (_cachedEyeDepthShader != null)
            {
                shader = _cachedEyeDepthShader;
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Reusing cached shader: " + shader.name);
                return true;
            }

            IntPtr bundlePtr;
            if (!TryGetBundlePointer(bundlePath, out bundlePtr)) return false;

            string assetName = FindAssetName(bundlePtr, shaderNameWithoutExt);
            if (string.IsNullOrEmpty(assetName))
            {
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Shader asset not found in bundle: " + shaderNameWithoutExt);
                return false;
            }

            try
            {
                IntPtr assetPtr = _loadAssetInternal(bundlePtr, IL2CPP.ManagedStringToIl2Cpp(assetName), Il2CppType.Of<Shader>().Pointer);
                if (assetPtr == IntPtr.Zero)
                {
                    Debug.LogError("[TopSurfaceSeatProxy] LoadAsset_Internal returned null for shader: " + assetName);
                    return false;
                }

                shader = new Shader(assetPtr);
                _cachedEyeDepthShader = shader;
                Debug.LogWarning("[TopSurfaceSeatProxy] Loaded Shader: " + assetName + " / " + shader.name);
                return shader != null;
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxy] TryLoadShaderByName exception: " + e);
                return false;
            }
        }

        public static bool TryGetBundlePointer(string bundlePath, out IntPtr bundlePtr)
        {
            bundlePtr = IntPtr.Zero;
            if (string.IsNullOrWhiteSpace(bundlePath) || !File.Exists(bundlePath))
            {
                Debug.LogError("[TopSurfaceSeatProxy] AssetBundle path invalid: " + bundlePath);
                return false;
            }

            if (!EnsureResolved()) return false;
            if (_loadFromFileInternal == null || _loadAssetInternal == null || _getAllAssetNamesInternal == null)
                return false;

            if (_cachedBundlePtr != IntPtr.Zero)
            {
                if (string.Equals(_cachedBundlePath, bundlePath, StringComparison.OrdinalIgnoreCase))
                {
                    bundlePtr = _cachedBundlePtr;
                    if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Reusing cached AssetBundle pointer: " + _cachedBundlePath);
                    return true;
                }

                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Cached AssetBundle path differs. cached=" + _cachedBundlePath + ", requested=" + bundlePath);
            }

            try
            {
                bundlePtr = _loadFromFileInternal(IL2CPP.ManagedStringToIl2Cpp(bundlePath), 0u, 0UL);
                if (bundlePtr == IntPtr.Zero)
                {
                    Debug.LogError("[TopSurfaceSeatProxy] LoadFromFile_Internal returned null. Same AssetBundle may already be loaded. Restart game. path=" + bundlePath);
                    return false;
                }

                _cachedBundlePtr = bundlePtr;
                _cachedBundlePath = bundlePath;
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Loaded AssetBundle by iCall: " + bundlePath);
                return true;
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxy] TryGetBundlePointer exception: " + e);
                return false;
            }
        }

        private static bool EnsureResolved()
        {
            if (_resolved) return true;
            _resolved = true;

            try
            {
                _loadFromFileInternal = IL2CPP.ResolveICall<LoadFromFileInternalDelegate>("UnityEngine.AssetBundle::LoadFromFile_Internal(System.String,System.UInt32,System.UInt64)");
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Resolved AssetBundle LoadFromFile_Internal.");
            }
            catch (Exception e) { Debug.LogError("[TopSurfaceSeatProxy] Resolve LoadFromFile_Internal failed: " + e.Message); }

            try
            {
                _loadAssetInternal = IL2CPP.ResolveICall<LoadAssetInternalDelegate>("UnityEngine.AssetBundle::LoadAsset_Internal(System.String,System.Type)");
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Resolved AssetBundle LoadAsset_Internal.");
            }
            catch (Exception e) { Debug.LogError("[TopSurfaceSeatProxy] Resolve LoadAsset_Internal failed: " + e.Message); }

            try
            {
                _getAllAssetNamesInternal = IL2CPP.ResolveICall<GetAllAssetNamesDelegate>("UnityEngine.AssetBundle::GetAllAssetNames");
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Resolved AssetBundle GetAllAssetNames.");
            }
            catch (Exception e) { Debug.LogError("[TopSurfaceSeatProxy] Resolve GetAllAssetNames failed: " + e.Message); }

            try
            {
                _unloadInternal = IL2CPP.ResolveICall<UnloadDelegate>("UnityEngine.AssetBundle::Unload");
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Resolved AssetBundle Unload.");
            }
            catch (Exception e) { if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Resolve Unload failed, non-fatal: " + e.Message); }

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
                if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Asset names in mita_actions bundle:");

                for (int i = 0; i < names.Length; i++)
                {
                    string assetName = names[i];
                    if (VerboseLoaderLogs) Debug.LogWarning("  " + assetName);
                    if (string.IsNullOrEmpty(assetName)) continue;

                    string fileNoExt = Path.GetFileNameWithoutExtension(assetName).ToLowerInvariant();
                    string fileFull = Path.GetFileName(assetName).ToLowerInvariant();

                    if (fileNoExt == target || fileFull == target)
                    {
                        if (VerboseLoaderLogs) Debug.LogWarning("[TopSurfaceSeatProxy] Found target asset: " + assetName);
                        return assetName;
                    }
                }
            }
            catch (Exception e)
            {
                Debug.LogError("[TopSurfaceSeatProxy] FindAssetName exception: " + e);
            }

            return null;
        }
    }
}
