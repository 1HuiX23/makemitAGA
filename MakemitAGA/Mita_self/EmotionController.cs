using HarmonyLib;
using UnityEngine;
using System.Reflection;
using BepInEx.Logging;
using System;

namespace MakemitAGA.Mita_self
{
    // 这是一个完全独立设计的控制器，用于封装对游戏内部 API 的反射调用
    public static class EmotionController
    {
        private static MethodInfo _faceLayerMethod;
        private static MethodInfo _faceEmotionTypeMethod;
        private static Type _emotionEnumType;

        // 初始化反射缓存，避免每次调用都查找，提高性能
        public static void Initialize()
        {
            if (_faceLayerMethod != null) return;

            // 获取 MitaPerson 的类型
            var mitaType = typeof(MitaPerson); // 或者通过 Type.GetType 查找

            // 缓存 FaceLayer 方法 (用于设置权重)
            _faceLayerMethod = AccessTools.Method(mitaType, "FaceLayer");

            // 缓存 FaceEmotionType 方法 (用于设置具体表情)
            _faceEmotionTypeMethod = AccessTools.Method(mitaType, "FaceEmotionType");

            // 获取 EmotionType 枚举类型 (它是 FaceEmotionType 的第一个参数类型)
            if (_faceEmotionTypeMethod != null)
            {
                var parameters = _faceEmotionTypeMethod.GetParameters();
                if (parameters.Length > 0)
                {
                    _emotionEnumType = parameters[0].ParameterType;
                }
            }

            Plugin.Logger.LogInfo($"[EmotionController] 反射初始化完成: Layer={_faceLayerMethod != null}, Emotion={_faceEmotionTypeMethod != null}");
        }

        // 统一的调用入口
        public static void ApplyEmotion(MonoBehaviour mitaInstance, int emotionId)
        {
            if (mitaInstance == null) return;

            // 1. 激活表情层级 (这是游戏机制要求的核心步骤)
            // 参数 1 通常代表 Face Layer，权重设为 1
            if (_faceLayerMethod != null)
            {
                try
                {
                    _faceLayerMethod.Invoke(mitaInstance, new object[] { 1 });
                }
                catch (System.Exception e)
                {
                    Plugin.Logger.LogError($"激活表情层失败: {e.Message}");
                }
            }

            // 2. 设置具体表情 ID
            if (_faceEmotionTypeMethod != null && _emotionEnumType != null)
            {
                try
                {
                    // 动态转换 int 到枚举
                    object enumVal = Enum.ToObject(_emotionEnumType, emotionId);
                    _faceEmotionTypeMethod.Invoke(mitaInstance, new object[] { enumVal });
                    Plugin.Logger.LogInfo($"[表情] 已切换至 ID: {emotionId}");
                }
                catch (System.Exception e)
                {
                    Plugin.Logger.LogError($"应用表情失败: {e.Message}");
                }
            }
        }
    }
}