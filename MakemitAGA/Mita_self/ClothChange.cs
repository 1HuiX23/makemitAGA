/*
 * [模块名称]: 米塔换装与光标管理系统 (Cloth & Cursor System)
 * [文件路径]: MakemitAGA/Mita_self/ClothChange.cs
 * [功能描述]: 
 *    1. 管理米塔的服装切换 (修改 GlobalGame 变量 + 刷新模型)。
 *    2. 管理鼠标光标切换 (底层资源替换)。
 *    3. 数据持久化 (使用 config.json 保存状态)。
 *    4. 解锁检测 (读取游戏原生存档文件)。
 */

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.Json;              // 引入 .NET 6 原生 JSON 库 (不需要额外 DLL)
using System.Text.Json.Serialization;// 用于控制 JSON 字段的读写行为
using HarmonyLib;
using BepInEx;                    // 用于 Hook 游戏方法
using UnityEngine;                   // Unity 核心 API
using Il2CppInterop.Runtime;         // IL2CPP 互操作库 (处理 Type 转换)

namespace MakemitAGA.Mita_self
{
    // ==================================================================================
    // 1. 配置数据结构 (SharedConfig)
    // ==================================================================================
    // 这个类定义了 config.json 的文件结构。
    // 我们特意保留了 API_KEY 等字段，是为了和 Python 后端共用同一个配置文件，实现"一处配置，两端通用"。
    public class SharedConfig
    {
        // [JsonInclude] 确保 private/public 字段都能被 System.Text.Json 序列化
        [JsonInclude] public string API_KEY = "";
        [JsonInclude] public string MODEL_ID = "";
        [JsonInclude] public string SYSTEM_PROMPT = "我们的默认提示词";
        [JsonInclude] public string BASE_URL = "https://api-inference.modelscope.cn/v1/chat/completions";
        [JsonInclude] public int MAX_TOKENS = 512;
        [JsonInclude] public float TEMPERATURE = 0.1f;
        [JsonInclude] public int CONNECT_TIMEOUT_SECONDS = 20;
        [JsonInclude] public int READ_TIMEOUT_SECONDS = 240;

        // 后端文件调试开关：
        // false（默认）= plugins 目录不生成 backend_*.txt / backend_*.log；
        // true          = 所有启动、Prompt、Reply、异常合并写入 backend_debug.txt。
        // BepInEx 控制台中的实时后端交互日志不受此开关影响。
        [JsonInclude] public bool WRITE_BACKEND_DEBUG_FILE = false;

        // --- Mod 专属字段 ---
        [JsonInclude] public string CurrentOutfitId = "Original";   // 当前衣服的内部ID
        [JsonInclude] public string CurrentCursorName = "Cursor";   // 当前光标的贴图名 (独立保存)
        [JsonInclude] public bool UnlockAllOutfits = false;         // 作弊开关：是否强制解锁所有衣服
    }

    public static class ClothChange
    {
        // ==================================================================================
        // 2. 内部数据定义 (OutfitData)
        // ==================================================================================
        // 用来连接"我们的逻辑"和"游戏的数据"。
        public class OutfitData
        {
            public string Id;           // 我们的 ID (如 "Vampire")，用于 config.json 和控制台指令
            public string GameCode;     // 游戏内部代码 (如 "HellVamp")，对应 GlobalGame.clothMita
            public int VariantId;       // 变体 ID (如校服有 0,1,2 三种颜色)
            public string Description;  // 描述文本 (未来可发给 AI，让它知道自己穿的啥)
            public string DefaultCursor;// 这套衣服官方配套的光标名字

            public OutfitData(string id, string code, int var, string desc, string cursor)
            { Id = id; GameCode = code; VariantId = var; Description = desc; DefaultCursor = cursor; }
        }

        // ==================================================================================
        // 3. 服装数据库 (Outfits)
        // ==================================================================================
        // 静态列表，列出所有支持的服装。
        public static readonly List<OutfitData> Outfits = new List<OutfitData>()
        {
            // 索引 0: 原版
            new OutfitData("Original", "original", 0, "经典的白色连衣裙，那是初见时的模样。", "Cursor"),
            // 索引 1-3: 校服系列 (VariantId 不同)
            new OutfitData("SchoolBlue", "FIIdClSchool", 0, "蓝色的学校制服，看起来青春洋溢。", "Cursor"),
            new OutfitData("SchoolRed", "FIIdClSchool", 1, "红色的学校制服变体，热情活泼。", "Cursor"),
            new OutfitData("SchoolDark", "FIIdClSchool", 2, "深色的学校制服变体，沉稳干练。", "Cursor"),
            // 索引 4: 圣诞 (需要解锁)
            new OutfitData("Christmas", "Chirfns", 0, "红白相间的圣诞服，充满节日的喜庆气氛。", "Cursor Christmas"),
            // 索引 5: 吸血鬼 (需要解锁)
            new OutfitData("Vampire", "HellVamp", 0, "黑红配色的吸血鬼伯爵装，带有一丝危险的优雅。", "Cursor Halloween"),
        };

        // 运行时状态
        private static int _currentIndex = 0; // 当前在 Outfits 列表中的索引
        private static SharedConfig _configData = new SharedConfig(); // 内存中的配置数据

        // --- 路径助手 ---
        // 用户明确使用 plugins/config.json 作为共享配置。
        // 旧整合版曾迁移到 BepInEx/config/MakemitAGA.json；这里保留一次性反向迁移，
        // 避免升级后丢失 API_KEY、模型参数或换装状态。
        private static string AssemblyDirectory =>
            Path.GetDirectoryName(
                Assembly.GetExecutingAssembly().Location);

        private static string ConfigPath =>
            Path.Combine(
                AssemblyDirectory,
                "config.json");

        private static string PreviousConfigPath =>
            Path.Combine(
                Paths.ConfigPath,
                "MakemitAGA.json");

        public static string ConfigPathForBackend =>
            ConfigPath;
        // 获取游戏原生存档路径 (C:/Users/xxx/AppData/LocalLow/AIHASTO/MiSideFull/Save/Clothes)
        // 这是判断玩家是否合法解锁衣服的关键文件
        private static string GameSavePath => Path.Combine(Application.persistentDataPath, "Save", "Clothes");

        // 缓存已解锁的衣服代码，避免频繁读文件 IO
        private static HashSet<string> _unlockedGameCodes = new HashSet<string>();

        // ==================================================================================
        // 4. 初始化流程 (Init)
        // ==================================================================================
        // 在 Plugin.Load() 中调用，只执行一次。
        public static void Init()
        {
            LoadConfigFromJson(); // 1. 读取 config.json
            RefreshGameUnlocks(); // 2. 读取游戏存档

            // 3. 将 json 里的 ID 同步到 _currentIndex，方便后续逻辑使用
            var saved = Outfits.FirstOrDefault(x => x.Id == _configData.CurrentOutfitId);
            if (saved != null)
            {
                _currentIndex = Outfits.IndexOf(saved);
                Plugin.Logger.LogInfo($"[ClothChange] 状态初始化完成：上次穿着 {saved.Id}");
            }
        }

        // ==================================================================================
        // 5. 应用状态 (ApplySavedOutfitState)
        // ==================================================================================
        // ★ 核心方法 ★
        // 这个方法会被 Harmony 补丁调用 (在 MitaClothes.Start 时)。
        // 它的作用是：每当场景加载、米塔生成时，强行把她的衣服改回我们 json 里保存的样子。
        // 如果没有这一步，玩家切换场景后衣服就会重置。
        public static void ApplySavedOutfitState()
        {
            RefreshGameUnlocks(); // 保险起见，刷新一下解锁状态
            var data = Outfits[_currentIndex];

            // 检查合法性：如果 json 里存了"吸血鬼"但玩家没解锁，就回滚到原版
            // 防止修改 json 导致的逻辑错误或作弊
            if (CheckIsUnlocked(data))
            {
                GlobalGame.clothMita = data.GameCode;
                GlobalGame.clothVariantMita = data.VariantId;
            }
            else
            {
                GlobalGame.clothMita = "original";
                GlobalGame.clothVariantMita = 0;
            }

            // 恢复光标 (注意：这里用的是 Config 里的光标名，而不是衣服默认的)
            // 这实现了"衣服和光标分离"的功能
            SetCursorByName(_configData.CurrentCursorName);
        }

        // ==================================================================================
        // 6. 控制台接口：按索引换装 (SetOutfitByIndex)
        // ==================================================================================
        // 给 cloth1 - cloth6 指令使用
        public static string SetOutfitByIndex(int index)
        {
            RefreshGameUnlocks();
            if (index < 0 || index >= Outfits.Count) return "无效的服装编号 (请检查列表)";

            var data = Outfits[index];
            // 权限检查
            if (!CheckIsUnlocked(data)) return $"服装 [{data.Description}] 尚未解锁，无法更换。";

            // 执行换装
            SetOutfitByData(data);
            return $"已换上: {data.Description}";
        }

        // ==================================================================================
        // 7. 控制台接口：独立换光标 (SetCursorByName)
        // ==================================================================================
        // 给 mouse1 - mouse3 指令使用，也供 AI 调用
        public static string SetCursorByName(string cursorTargetName)
        {
            // 模糊匹配：允许只输 "Christmas" 自动补全为 "Cursor Christmas"
            if (cursorTargetName.Equals("Cursor Original", StringComparison.OrdinalIgnoreCase)) cursorTargetName = "Cursor";
            if (cursorTargetName.Equals("Cursor Default", StringComparison.OrdinalIgnoreCase)) cursorTargetName = "Cursor";

            // --- 暴力资源搜索 ---
            // 因为光标贴图没有挂在场景物体上，而是存在内存资源里
            // 我们使用 Resources.FindObjectsOfTypeAll 扫描整个内存
            var il2cppType = Il2CppType.Of<Texture2D>();
            var allTextures = Resources.FindObjectsOfTypeAll(il2cppType);
            Texture2D targetTex = null;

            foreach (var obj in allTextures)
            {
                // 安全类型转换：Il2CppObject -> Texture2D
                var tex = obj.TryCast<Texture2D>();
                if (tex != null && tex.name.Equals(cursorTargetName, StringComparison.OrdinalIgnoreCase))
                {
                    targetTex = tex;
                    break; // 找到了就立刻停止，节省性能
                }
            }

            if (targetTex != null)
            {
                // 调用 Unity 底层 API 设置光标
                // Vector2.zero 表示点击热点在左上角
                Cursor.SetCursor(targetTex, Vector2.zero, CursorMode.Auto);

                // 如果光标变了，保存到 json
                if (_configData.CurrentCursorName != targetTex.name)
                {
                    _configData.CurrentCursorName = targetTex.name;
                    SaveConfigToJson();
                }
                return "鼠标光标已更新。";
            }
            return "无法找到指定的光标样式 (请检查资源名称)。";
        }

        // ==================================================================================
        // 8. 内部核心逻辑：执行换装 (SetOutfitByData)
        // ==================================================================================
        private static void SetOutfitByData(OutfitData data)
        {
            // 1. 修改游戏全局变量 (这是游戏读取衣服的依据)
            GlobalGame.clothMita = data.GameCode;
            GlobalGame.clothVariantMita = data.VariantId;

            // 2. 更新内存数据
            _configData.CurrentOutfitId = data.Id;
            _configData.CurrentCursorName = data.DefaultCursor; // 换整套衣服时，重置光标为默认
            _currentIndex = Outfits.IndexOf(data);

            // 3. 立即存盘 (防止崩溃丢失)
            SaveConfigToJson();

            // 4. 刷新 3D 模型
            RefreshModel();

            // 5. 刷新光标
            SetCursorByName(data.DefaultCursor);
        }

        // --- 刷新模型实现 ---
        private static void RefreshModel()
        {
            // 在场景中寻找米塔的物体
            GameObject personObj = GameObject.Find("MitaPerson Mita");
            if (personObj == null) return;

            // 获取换装脚本
            MitaClothes clothes = personObj.GetComponent<MitaClothes>();
            if (clothes != null)
            {
                // ★ 关键点：防止自杀 ★
                // MitaClothes 原逻辑是换完衣服就 Destroy(this)。
                // 我们必须设为 true，强迫它活下来，这样我们才能多次换装。
                clothes.dontDestroyStart = true;
                try { clothes.ReCloth(); } catch { }
            }
        }

        // ==================================================================================
        // 9. 解锁检测逻辑 (RefreshGameUnlocks & CheckIsUnlocked)
        // ==================================================================================
        private static void RefreshGameUnlocks()
        {
            _unlockedGameCodes.Clear();
            _unlockedGameCodes.Add("original"); // 原版默认就有

            // 读取 Save/Clothes 文件
            if (File.Exists(GameSavePath))
            {
                try
                {
                    string[] lines = File.ReadAllLines(GameSavePath);
                    foreach (var line in lines)
                        if (!string.IsNullOrWhiteSpace(line))
                            _unlockedGameCodes.Add(line.Trim()); // 存入 HashSet，方便快速查找
                }
                catch { }
            }
        }

        private static bool CheckIsUnlocked(OutfitData data)
        {
            // 优先级 1: 配置文件里的作弊开关
            if (_configData.UnlockAllOutfits) return true;
            // 优先级 2: 校服默认解锁
            if (data.Id.StartsWith("School")) return true;
            // 优先级 3: 检查存档记录
            if (_unlockedGameCodes.Contains(data.GameCode)) return true;

            return false;
        }

        // ==================================================================================
        // 10. JSON 序列化与反序列化
        // ==================================================================================
        private static void LoadConfigFromJson()
        {
            string sourcePath = ConfigPath;

            // v0.2.2 兼容：
            // 如果 plugins/config.json 尚不存在，而上一版的
            // BepInEx/config/MakemitAGA.json 存在，则读取旧文件并迁回 plugins。
            if (!File.Exists(sourcePath) &&
                File.Exists(PreviousConfigPath))
            {
                sourcePath = PreviousConfigPath;
            }

            if (File.Exists(sourcePath))
            {
                try
                {
                    string json = File.ReadAllText(sourcePath);
                    var options = new JsonSerializerOptions
                    {
                        ReadCommentHandling = JsonCommentHandling.Skip,
                        IncludeFields = true
                    };

                    _configData =
                        JsonSerializer.Deserialize<SharedConfig>(
                            json,
                            options);

                    if (_configData == null)
                        _configData = new SharedConfig();

                    if (!string.Equals(
                        sourcePath,
                        ConfigPath,
                        StringComparison.OrdinalIgnoreCase))
                    {
                        Plugin.Logger?.LogInfo(
                            "[ClothChange] config migrated back to " +
                            ConfigPath);
                    }

                    // 即使已有 config.json，也重新序列化一次：
                    // 这样升级后会自动补上 WRITE_BACKEND_DEBUG_FILE=false。
                    SaveConfigToJson();
                }
                catch (Exception e)
                {
                    Plugin.Logger?.LogWarning(
                        "[ClothChange] config read failed; using defaults: " +
                        e.Message);

                    _configData = new SharedConfig();
                    SaveConfigToJson();
                }
            }
            else
            {
                SaveConfigToJson();
            }
        }

        private static void SaveConfigToJson()
        {
            try
            {
                // 格式化输出 (缩进)，方便人类阅读
                var options = new JsonSerializerOptions { WriteIndented = true, IncludeFields = true };
                string json = JsonSerializer.Serialize(_configData, options);
                File.WriteAllText(ConfigPath, json);
            }
            catch { }
        }
    }

    // ==================================================================================
    // 11. Harmony 补丁
    // ==================================================================================
    [HarmonyPatch(typeof(MitaClothes))]
    public static class ClothesPatch
    {
        // 拦截 Start 方法，在原代码执行之前 (Prefix) 运行
        [HarmonyPatch("Start")]
        [HarmonyPrefix]
        public static void StartPrefix(MitaClothes __instance)
        {
            // 1. 挂免死金牌
            __instance.dontDestroyStart = true;
            // 2. 趁机把我们保存的衣服 ID 塞进去
            // 这样游戏一开始加载出来就是我们想要的衣服，而不是存档里的默认衣服
            ClothChange.ApplySavedOutfitState();
        }
    }
}