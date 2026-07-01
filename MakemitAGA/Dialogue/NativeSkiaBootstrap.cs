/*
 * =================================================================================================
 * NativeSkiaBootstrap.cs
 * =================================================================================================
 *
 * 【职责】
 *   可靠加载 SkiaSharp 所需的 Windows x64 原生 libSkiaSharp.dll，并避免和其他 Mod 冲突。
 *
 * 【正式加载链】
 *   csproj EmbeddedResource
 *   -> 提取到 BepInEx/cache/MakemitAGA/Formula3D/...
 *   -> 改用插件专属文件名
 *   -> 给 SkiaSharp 程序集注册 DllImportResolver
 *   -> NativeLibrary.Load(绝对路径)
 *
 * 【为什么不用 plugins/libSkiaSharp.dll】
 *   多个 Mod 可能携带不同 SkiaSharp 版本；公共文件名会造成先加载者胜出、导出函数不匹配。
 *   专属缓存路径和专属文件名可显著降低冲突风险。
 *
 * 【诊断回退】
 *   若项目文件漏了 EmbeddedResource，本类会尝试插件同目录和 FormulaRuntime 目录。
 *   回退仅用于排错，正式发布仍应保证资源嵌入成功。
 *
 * 【维护陷阱】
 *   1. SkiaSharp 托管包版本和 NativeFileName 中的版本必须一致。
 *   2. DllImportResolver 对同一程序集只能注册一次；热重载时 InvalidOperationException 可接受。
 *   3. _nativeHandle 必须长期保留，不能加载后立即 Free，否则后续原生调用会崩溃。
 *   4. 此类初始化结果会缓存；一次失败后本进程不会自动重试，修复部署后应重启游戏。
 * =================================================================================================
 */
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using BepInEx;
using BepInEx.Logging;
using SkiaSharp;

namespace MakemitAGA.Dialogue
{
    /// <summary>
    /// Skia 原生运行库隔离器。
    ///
    /// 正常路径：
    ///   1. csproj 把 win-x64/libSkiaSharp.dll 作为 EmbeddedResource 嵌入；
    ///   2. 运行时提取到 BepInEx/cache 下的插件专属目录；
    ///   3. 使用唯一文件名，避免和其他 Mod 的 libSkiaSharp.dll 相互覆盖；
    ///   4. 为 SkiaSharp 程序集注册 DllImportResolver。
    ///
    /// 诊断回退：
    ///   如果用户把 .cs 文件复制进已有项目而漏掉 EmbeddedResource 配置，
    ///   也会尝试从插件目录或 runtimes/win-x64/native 目录读取原生 DLL。
    ///   该回退主要用于排错；正式发布仍推荐嵌入资源。
    /// </summary>
    internal static class NativeSkiaBootstrap
    {
        private const string PreferredResourceName =
            "MakemitAGA.Dialogue.Native.libSkiaSharp.dll";

        private const string NativeFileName =
            "libSkiaSharp.MakemitAGA.Formula3D.2.88.9.x64.dll";

        private static readonly object Sync = new object();

        private static bool _initialized;
        private static bool _success;
        private static string _nativePath;
        private static IntPtr _nativeHandle = IntPtr.Zero;

        /// <summary>
        /// 进程内只初始化一次。成功与失败都会缓存，因此部署修复后的 DLL 后需要完整重启游戏。
        /// </summary>
        public static bool Initialize(ManualLogSource log)
        {
            lock (Sync)
            {
                if (_initialized)
                    return _success;

                _initialized = true;

                try
                {
                    Assembly pluginAssembly =
                        typeof(NativeSkiaBootstrap).Assembly;

                    string outputDirectory = Path.Combine(
                        Paths.CachePath,
                        "MakemitAGA",
                        "Formula3D",
                        "native-x64-2.88.9");

                    Directory.CreateDirectory(outputDirectory);

                    _nativePath = Path.Combine(
                        outputDirectory,
                        NativeFileName);

                    string resourceName;
                    using (Stream resource = TryOpenNativeResource(
                               pluginAssembly,
                               out resourceName))
                    {
                        if (resource != null)
                        {
                            WriteStreamIfChanged(
                                resource,
                                _nativePath);

                            log?.LogInfo(
                                "[Formula3D][Skia] Extracted embedded native payload" +
                                " | resource=" + resourceName +
                                " | path=" + _nativePath);
                        }
                        else
                        {
                            string externalSource =
                                FindExternalNativeSource(pluginAssembly);

                            if (string.IsNullOrWhiteSpace(externalSource))
                            {
                                string resources = string.Join(
                                    ", ",
                                    pluginAssembly.GetManifestResourceNames());

                                throw new FileNotFoundException(
                                    "Embedded native resource was not found and no external fallback exists." +
                                    " Expected=" + PreferredResourceName +
                                    " | manifestResources=[" + resources + "]" +
                                    " | assembly=" + pluginAssembly.Location);
                            }

                            CopyFileIfChanged(
                                externalSource,
                                _nativePath);

                            log?.LogWarning(
                                "[Formula3D][Skia] Embedded resource missing; using external fallback" +
                                " | source=" + externalSource +
                                " | isolatedPath=" + _nativePath);
                        }
                    }

                    Assembly skiaAssembly =
                        typeof(SKObject).Assembly;

                    try
                    {
                        NativeLibrary.SetDllImportResolver(
                            skiaAssembly,
                            ResolveSkiaImport);
                    }
                    catch (InvalidOperationException)
                    {
                        // 同一程序集只能注册一个 resolver。
                        // 热重载或另一个 Skia 使用者可能已提前注册；绝对路径预加载仍有价值。
                        log?.LogWarning(
                            "[Formula3D][Skia] A DllImportResolver is already registered for SkiaSharp; " +
                            "continuing with absolute-path preload.");
                    }

                    _nativeHandle =
                        NativeLibrary.Load(_nativePath);

                    // 尽早触发一次原生调用，便于直接暴露位数或导出不匹配。
                    SKColorType colorType =
                        SKImageInfo.PlatformColorType;

                    log?.LogInfo(
                        "[Formula3D][Skia] Native runtime ready" +
                        " | path=" + _nativePath +
                        " | platformColorType=" + colorType);

                    _success = true;
                    return true;
                }
                catch (Exception e)
                {
                    log?.LogError(
                        "[Formula3D][Skia] Native initialization failed: " +
                        e);

                    _success = false;
                    return false;
                }
            }
        }

        /// <summary>先查固定 LogicalName，再按后缀查找，兼容 RootNamespace/AssemblyName 变化。</summary>
        private static Stream TryOpenNativeResource(
            Assembly assembly,
            out string resourceName)
        {
            resourceName = null;

            Stream exact = assembly.GetManifestResourceStream(
                PreferredResourceName);

            if (exact != null)
            {
                resourceName = PreferredResourceName;
                return exact;
            }

            // 用户修改 RootNamespace/AssemblyName 后，默认逻辑名可能出现前缀变化。
            // 因此再按文件名后缀宽松搜索一次。
            string fallbackName = assembly
                .GetManifestResourceNames()
                .FirstOrDefault(name =>
                    name.EndsWith(
                        ".Native.libSkiaSharp.dll",
                        StringComparison.OrdinalIgnoreCase) ||
                    name.EndsWith(
                        ".libSkiaSharp.dll",
                        StringComparison.OrdinalIgnoreCase));

            if (string.IsNullOrWhiteSpace(fallbackName))
                return null;

            resourceName = fallbackName;
            return assembly.GetManifestResourceStream(
                fallbackName);
        }

        private static string FindExternalNativeSource(
            Assembly pluginAssembly)
        {
            string assemblyDirectory = null;

            try
            {
                assemblyDirectory = Path.GetDirectoryName(
                    pluginAssembly.Location);
            }
            catch { }

            var candidates = new List<string>();

            if (!string.IsNullOrWhiteSpace(assemblyDirectory))
            {
                candidates.Add(Path.Combine(
                    assemblyDirectory,
                    "libSkiaSharp.dll"));

                candidates.Add(Path.Combine(
                    assemblyDirectory,
                    "runtimes",
                    "win-x64",
                    "native",
                    "libSkiaSharp.dll"));

                candidates.Add(Path.Combine(
                    assemblyDirectory,
                    NativeFileName));
            }

            candidates.Add(Path.Combine(
                Paths.PluginPath,
                "MakemitAGA",
                "FormulaRuntime",
                "libSkiaSharp.dll"));

            candidates.Add(Path.Combine(
                Paths.PluginPath,
                "MakemitAGA",
                "FormulaRuntime",
                "runtimes",
                "win-x64",
                "native",
                "libSkiaSharp.dll"));

            foreach (string candidate in candidates)
            {
                try
                {
                    if (File.Exists(candidate))
                        return candidate;
                }
                catch { }
            }

            return null;
        }

        private static void WriteStreamIfChanged(
            Stream source,
            string destination)
        {
            bool shouldWrite =
                !File.Exists(destination) ||
                new FileInfo(destination).Length != source.Length;

            if (!shouldWrite)
                return;

            using (FileStream output = new FileStream(
                       destination,
                       FileMode.Create,
                       FileAccess.Write,
                       FileShare.Read))
            {
                source.CopyTo(output);
            }
        }

        private static void CopyFileIfChanged(
            string source,
            string destination)
        {
            FileInfo sourceInfo = new FileInfo(source);

            bool shouldCopy =
                !File.Exists(destination) ||
                new FileInfo(destination).Length != sourceInfo.Length;

            if (!shouldCopy)
                return;

            File.Copy(
                source,
                destination,
                true);
        }

        /// <summary>
        /// SkiaSharp 的 DllImport 回调。只响应名称中包含 SkiaSharp 的请求，其他原生库交回默认解析器。
        /// </summary>
        private static IntPtr ResolveSkiaImport(
            string libraryName,
            Assembly assembly,
            DllImportSearchPath? searchPath)
        {
            if (string.IsNullOrWhiteSpace(libraryName) ||
                libraryName.IndexOf(
                    "SkiaSharp",
                    StringComparison.OrdinalIgnoreCase) < 0)
            {
                return IntPtr.Zero;
            }

            lock (Sync)
            {
                if (_nativeHandle == IntPtr.Zero &&
                    !string.IsNullOrWhiteSpace(_nativePath))
                {
                    _nativeHandle =
                        NativeLibrary.Load(_nativePath);
                }

                return _nativeHandle;
            }
        }
    }
}