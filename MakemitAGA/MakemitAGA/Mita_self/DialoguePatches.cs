/*
 * [文件说明]: 对话与交互的核心钩子 (say, look, goto, anim)
 * 
 * [分析过程]:
 * 1. 拦截 ConsoleCommandsGame.Command 获取用户输入。
 * 2. [痛点解决]: NavMesh 寻路时，直接设置坐标会导致米塔隔墙蹭。我们引入了 NavMesh.SamplePosition 获取合法路径点。
 * 3. 动画播放需要处理 Layer 权重，否则表情会被身体动作覆盖。
 * 
 * [主要功能]:
 * 1. InterceptCommand(): 分发 say, look, goto, come, anim, faceid 指令。
 * 2. HandleAICommand(): 协调截图、AI请求、物体查找流程。
 * 3. WalkToObj(): 智能寻路 + 强制夺权 (Anti-Magnet) + 急停 (AiShraplyStop)。
 * 4. InjectText(): 在 Dialogue_3DText.Start 后注入 AI 文本并手动重排版 (解决居中问题)。
 */
/*
 * [文件说明]: 游戏原生逻辑的阻断器
 * 
 * [分析过程]:
 * 1. 当我们希望 AI 接管米塔移动时 (IsAIControlled)，必须彻底切断她"自动跟随玩家"的念头。
 * 2. 通过 Harmony Prefix 返回 false 来跳过原生方法执行。
 * 
 * [主要功能]:
 * 1. InterceptMagnet(): 当 MitaStateManager.IsAIControlled 为 true 时，拦截 MagnetToTarget 调用。
 */
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BepInEx.Unity.IL2CPP.Utils.Collections;
using HarmonyLib;
using MakemitAGA.Connection;
using MakemitAGA.Dialogue;
using MakemitAGA.World;
using MakemitAGA.Mita_self.Mita_tools;
using UnityEngine;
using UnityEngine.AI;

namespace MakemitAGA.Mita_self
{
    public class DialoguePatches
    {
        public static bool IsAIControllingMovement = false;
        public static bool IsInternalCall = false;

        // 缓存：房间名 -> 地板对象
        private static Dictionary<string, GameObject> _floorCache = new Dictionary<string, GameObject>();

        // ----------------------------------------------------------------
        // 补丁 1: 3D 文字注入 (保持不变)
        // ----------------------------------------------------------------
        [HarmonyPatch(typeof(Dialogue_3DText), "Start")]
        [HarmonyPostfix]
        public static void InjectText(Dialogue_3DText __instance)
        {
            if (GameUIManager.PendingInjections.TryGetValue(__instance, out string aiReply))
            {
                __instance.textPrint = aiReply;
                if (__instance.font != null) __instance.font.RequestCharactersInTexture(aiReply, 72);

                float totalWidth = 0f;
                if (__instance.font != null)
                {
                    foreach (char c in aiReply)
                    {
                        CharacterInfo info;
                        __instance.font.GetCharacterInfo(c, out info, 72);
                        totalWidth += info.advance;
                    }
                }

                __instance.xPrint = 0f;
                __instance.xPrint -= (totalWidth * __instance.sizeSymbol * 0.5f);
                __instance.indexChar = 0;
                __instance.textPrintNow = "";
                __instance.align = Dialogue_3DText.Alignment3DText.Middle;
                GameUIManager.PendingInjections.Remove(__instance);
            }
        }

        // ----------------------------------------------------------------
        // 补丁 2: 霸权拦截
        // ----------------------------------------------------------------
        [HarmonyPatch(typeof(MitaPerson), "MagnetToTarget")]
        [HarmonyPrefix]
        public static bool BlockMagnetIfAIControlled()
        {
            return !IsAIControllingMovement;
        }

        [HarmonyPatch(typeof(MitaPerson), "AiWalkToTarget", new Type[] { typeof(Transform) })]
        [HarmonyPrefix]
        public static bool BlockWalkIfAIControlled()
        {
            if (IsAIControllingMovement && !IsInternalCall) return false;
            return true;
        }

        // ----------------------------------------------------------------
        // 补丁 3: 拦截指令
        // ----------------------------------------------------------------
        [HarmonyPatch(typeof(ConsoleCommandsGame), "Command")]
        [HarmonyPrefix]
        public static bool InterceptCommand(string code)
        {
            string command = code.Trim();

            // --- 基础对话指令 ---
            if (command.StartsWith("say ", StringComparison.OrdinalIgnoreCase))
            {
                string userInput = command.Substring(4).Trim();
                if (!string.IsNullOrEmpty(userInput))
                    Plugin.Runner.StartCoroutine(HandleAICommand(userInput, false).WrapToIl2Cpp());
                return false;
            }

            if (command.StartsWith("look ", StringComparison.OrdinalIgnoreCase))
            {
                string target = command.Substring(5).Trim();
                if (!string.IsNullOrEmpty(target))
                    Plugin.Runner.StartCoroutine(HandleAICommand(target, true).WrapToIl2Cpp());
                return false;
            }

            // --- 动画与移动 ---
            if (command.StartsWith("anim ", StringComparison.OrdinalIgnoreCase))
            {
                string animName = command.Substring(5).Trim();
                Plugin.Runner.StartCoroutine(PlayAnimTest(animName).WrapToIl2Cpp());
                return false;
            }

            if (command.Equals("listanims", StringComparison.OrdinalIgnoreCase))
            {
                Plugin.Runner.StartCoroutine(ListAnimations().WrapToIl2Cpp());
                return false;
            }

            if (command.Equals("create_test_cube", StringComparison.OrdinalIgnoreCase))
            {
                CreateTestObject.CreateTestCubes();
                return false;
            }

            if (IsSitCommand(command))
            {
                string targetName = ParseSitCommandArgument(command);
                if (string.IsNullOrWhiteSpace(targetName))
                {
                    ConsoleMain.ConsolePrintGame("用法: sit(TestChair_High)");
                    return false;
                }

                Mita_sit.Sit(targetName);
                return false;
            }

            if (command.Equals("reset_mita", StringComparison.OrdinalIgnoreCase))
            {
                // reset_mita 对应测试阶段的 F8 恢复逻辑。
                // Mita_sit.UnlockAndResume() 内部会根据当前状态决定：
                // 1. 如果正在走向座椅/准备入座：取消动作并恢复原生状态；
                // 2. 如果已经坐稳：执行完整起身流程，然后还原 Animator / FinalIK / NavMeshAgent / 控制权锁；
                // 3. 如果没有坐姿会话：EnsureInstance 后不会破坏当前状态，只会尝试做轻量恢复。
                Mita_sit.UnlockAndResume();
                return false;
            }

            if (command.StartsWith("faceid ", StringComparison.OrdinalIgnoreCase))
            {
                if (int.TryParse(command.Substring(7).Trim(), out int id))
                    Plugin.Runner.StartCoroutine(SetFaceId(id).WrapToIl2Cpp());
                return false;
            }

            if (command.Equals("come", StringComparison.OrdinalIgnoreCase))
            {
                // 如果当前由 Mita_sit 接管坐姿，先走完整起身/还原流程，再恢复原生跟随。
                if (Mita_sit.HasActiveSession)
                {
                    Mita_sit.UnlockAndResume();
                    return false;
                }

                IsAIControllingMovement = false;
                Plugin.Runner.StartCoroutine(ResumeFollowPlayer().WrapToIl2Cpp());
                return false;
            }

            if (command.StartsWith("goto ", StringComparison.OrdinalIgnoreCase))
            {
                string targetName = command.Substring(5).Trim();
                IsAIControllingMovement = true;
                Plugin.Runner.StartCoroutine(WalkToObj(targetName).WrapToIl2Cpp());
                return false;
            }

            // --- 服装与光标 ---
            if (command.StartsWith("cloth", StringComparison.OrdinalIgnoreCase))
            {
                string numStr = command.Substring(5).Trim();
                if (int.TryParse(numStr, out int index))
                {
                    string result = ClothChange.SetOutfitByIndex(index - 1);
                    ConsoleMain.ConsolePrintGame(result);
                }
                else ConsoleMain.ConsolePrintGame("请输入 cloth1 到 cloth6。");
                return false;
            }

            if (command.StartsWith("mouse", StringComparison.OrdinalIgnoreCase))
            {
                string numStr = command.Substring(5).Trim();
                string targetCursor = "";
                if (numStr == "1") targetCursor = "Cursor";
                else if (numStr == "2") targetCursor = "Cursor Christmas";
                else if (numStr == "3") targetCursor = "Cursor Halloween";

                if (!string.IsNullOrEmpty(targetCursor))
                {
                    string result = ClothChange.SetCursorByName(targetCursor);
                    ConsoleMain.ConsolePrintGame(result);
                }
                else ConsoleMain.ConsolePrintGame("请输入 mouse1-3。");
                return false;
            }

            // =========================================================
            //  核心补充：环境与特效指令 (之前漏掉的部分)
            // =========================================================

            // gettime
            if (command.Equals("gettime", StringComparison.OrdinalIgnoreCase))
            {
                string info = EnvironmentManager.GetCurrentTimeInfo();
                ConsoleMain.ConsolePrintGame(info);
                return false;
            }

            // time 0.5 [duration]
            if (command.StartsWith("timee ", StringComparison.OrdinalIgnoreCase))
            {
                string[] p = command.Substring(5).Trim().Split(' ');
                float v = 0.5f, d = 0f;
                if (p.Length > 0) float.TryParse(p[0], out v);
                if (p.Length > 1) float.TryParse(p[1], out d);
                string res = EnvironmentManager.SetTime(v, d);
                ConsoleMain.ConsolePrintGame(res);
                return false;
            }

            // color 1 0 0 [duration]
            if (command.StartsWith("color ", StringComparison.OrdinalIgnoreCase))
            {
                string[] p = command.Substring(6).Trim().Split(' ');
                if (p.Length >= 3)
                {
                    float r = 0, g = 0, b = 0, d = 0;
                    float.TryParse(p[0], out r); float.TryParse(p[1], out g); float.TryParse(p[2], out b);
                    if (p.Length >= 4) float.TryParse(p[3], out d);
                    string res = EnvironmentManager.SetColor(r, g, b, d);
                    ConsoleMain.ConsolePrintGame(res);
                }
                return false;
            }

            // resetcolor [duration]
            if (command.StartsWith("resetcolor", StringComparison.OrdinalIgnoreCase))
            {
                string[] p = command.Split(' '); float d = 0;
                if (p.Length > 1) float.TryParse(p[1], out d);
                ConsoleMain.ConsolePrintGame(EnvironmentManager.ResetColor(d));
                return false;
            }

            // black on/off [instant]
            if (command.StartsWith("black ", StringComparison.OrdinalIgnoreCase))
            {
                bool active = command.ToLower().Contains("on");
                bool instant = command.Contains("1");
                ConsoleMain.ConsolePrintGame(EnvironmentManager.SetBlackScreen(active, instant));
                return false;
            }

            // blood on/off
            if (command.StartsWith("blood ", StringComparison.OrdinalIgnoreCase))
            {
                ConsoleMain.ConsolePrintGame(EnvironmentManager.SetBlood(command.ToLower().Contains("on")));
                return false;
            }

            // negative on/off
            if (command.StartsWith("negative ", StringComparison.OrdinalIgnoreCase))
            {
                ConsoleMain.ConsolePrintGame(EnvironmentManager.SetNegative(command.ToLower().Contains("on")));
                return false;
            }

            // glitch (10s)
            if (command.Equals("glitch", StringComparison.OrdinalIgnoreCase))
            {
                Plugin.Runner.StartCoroutine(EnvironmentManager.GlitchRoutine(10.0f).WrapToIl2Cpp());
                ConsoleMain.ConsolePrintGame("花屏特效 (10s)");
                return false;
            }

            // glitch2 (Teleport 3s)
            if (command.Equals("glitch2", StringComparison.OrdinalIgnoreCase))
            {
                Plugin.Runner.StartCoroutine(EnvironmentManager.TeleportGlitchRoutine(3.0f).WrapToIl2Cpp());
                ConsoleMain.ConsolePrintGame("传送特效 (3s)");
                return false;
            }

            // tv
            if (command.Equals("tv", StringComparison.OrdinalIgnoreCase))
            {
                EnvironmentManager.TriggerTV();
                return false;
            }

            return true;
        }

        private static bool IsSitCommand(string command)
        {
            if (string.IsNullOrWhiteSpace(command)) return false;
            return command.Equals("sit", StringComparison.OrdinalIgnoreCase)
                || command.StartsWith("sit ", StringComparison.OrdinalIgnoreCase)
                || command.StartsWith("sit(", StringComparison.OrdinalIgnoreCase);
        }

        private static string ParseSitCommandArgument(string command)
        {
            if (string.IsNullOrWhiteSpace(command)) return string.Empty;

            string trimmed = command.Trim();

            // 支持 sit(TargetName) / sit("Target Name") / sit TargetName 三种格式。
            if (trimmed.StartsWith("sit(", StringComparison.OrdinalIgnoreCase) && trimmed.EndsWith(")"))
            {
                string inner = trimmed.Substring(4, trimmed.Length - 5).Trim();
                return inner.Trim('"', '\'');
            }

            if (trimmed.StartsWith("sit ", StringComparison.OrdinalIgnoreCase))
                return trimmed.Substring(4).Trim().Trim('"', '\'');

            return string.Empty;
        }

        // ----------------------------------------------------------------
        // 逻辑实现
        // ----------------------------------------------------------------

        private static IEnumerator HandleAICommand(string input, bool isVisionRequest)
        {
            if (isVisionRequest)
            {
                ConsoleMain.ConsolePrintGame($"正在观察: {input}...");
                MitaVisionManager.SaveCurrentViewToDisk();
                yield return null;

                string visionPrompt = $"请给出当前视角中【{input}】的位置。请只返回 JSON 格式的坐标数组 [x1, y1, x2, y2]，不要返回任何其他文字。";
                var task = AIConversationManager.GetResponseAsync(visionPrompt);
                while (!task.IsCompleted) { yield return null; }
                string response = task.Result;

                ConsoleMain.ConsolePrintGame($"AI数据: {response}");

                float[] coords = ParseCoordinates(response);
                if (coords != null)
                {
                    // 获取列表
                    var objects = ObjectDetector.FindObjectsInBox(coords[0], coords[1], coords[2], coords[3]);

                    if (objects.Count > 0)
                    {
                        // === 修改开始：遍历打印所有候选者 ===
                        string msg = $"<color=green>区域内检测到 {objects.Count} 个目标:</color>\n";

                        for (int i = 0; i < objects.Count; i++)
                        {
                                msg += $"{i + 1}. {objects[i].name}\n";
                        }

                        ConsoleMain.ConsolePrintGame(msg);

                        // 在控制台看到列表
                        // GameObject bestTarget = objects[0];
                        // ObjectDetector.HighlightObject(bestTarget); // 高亮还是只高亮第一个，不然屏幕会乱
                        // === 修改结束 ===
                    }
                    else ConsoleMain.ConsolePrintGame("<color=red>区域内未找到匹配物体。</color>");
                }
                else ConsoleMain.ConsolePrintGame("<color=red>坐标解析失败。</color>");
            }
            else
            {
                ConsoleMain.ConsolePrintGame("Mita正在思考...");
                var task = AIConversationManager.GetResponseAsync(input);
                while (!task.IsCompleted) { yield return null; }
                string aiResponse = task.Result;
                ConsoleMain.ConsolePrintGame($"Mita: {aiResponse}");

                if (aiResponse.Contains("哈哈") || aiResponse.Contains("开心")) Plugin.Runner.StartCoroutine(SetFaceId(2).WrapToIl2Cpp());
                else if (aiResponse.Contains("生气")) Plugin.Runner.StartCoroutine(SetFaceId(3).WrapToIl2Cpp());
                else if (aiResponse.Contains("惊讶")) Plugin.Runner.StartCoroutine(SetFaceId(14).WrapToIl2Cpp());
                else if (aiResponse.Contains("调皮")) Plugin.Runner.StartCoroutine(SetFaceId(10).WrapToIl2Cpp());

                GameUIManager.ShowLongText(aiResponse);
            }
        }

        private static IEnumerator SetFaceId(int id)
        {
            var mita = UnityEngine.Object.FindObjectOfType<MitaPerson>();
            if (mita == null) yield break;

            float duration = 3.0f;
            float timer = 0f;
            var emotionType = typeof(MitaPerson).Assembly.GetType("EmotionType");
            var method = AccessTools.Method(typeof(MitaPerson), "FaceEmotionType");

            if (emotionType != null && method != null)
            {
                var enumVal = Enum.ToObject(emotionType, id);
                ConsoleMain.ConsolePrintGame($"表情切换: ID {id}");
                while (timer < duration)
                {
                    if (mita == null) break;
                    method.Invoke(mita, new object[] { enumVal });
                    timer += Time.deltaTime;
                    yield return null;
                }
            }
        }

        // --- 核心修复：向心算法 + 小半径采样 ---
        private static IEnumerator WalkToObj(string nameKey)
        {
            var mita = UnityEngine.Object.FindObjectOfType<MitaPerson>();
            if (mita == null) yield break;

            GameObject target = FindObjectSmart(nameKey);

            if (target != null)
            {
                ConsoleMain.ConsolePrintGame($"米塔正在走向: <color=yellow>{target.name}</color>");
                mita.MagnetOff();

                // 1. 智能寻找对应的地板
                GameObject floorObj = GetFloorForObject(target);
                Collider floorCollider = floorObj?.GetComponent<Collider>();
                Vector3 roomCenter = mita.transform.position; // 默认值

                if (floorCollider != null)
                {
                    // 使用地板包围盒中心作为“引力点”
                    roomCenter = floorCollider.bounds.center;
                    ConsoleMain.ConsolePrintGame($"已定位房间: {floorObj.transform.parent.name} (地板: {floorObj.name})");
                }
                else
                {
                    ConsoleMain.ConsolePrintGame("<color=yellow>警告: 未能定位所属房间，使用默认导航。</color>");
                }

                // 2. 向心计算 (关键修复!)
                // 计算方向：从 [物体] 指向 [房间中心]
                // 这样无论物体贴在哪面墙，目标点一定在房间里侧
                Vector3 dirToCenter = (roomCenter - target.transform.position).normalized;

                // 如果物体就在中心，则指向米塔
                if (dirToCenter == Vector3.zero) dirToCenter = (mita.transform.position - target.transform.position).normalized;

                // 3. 设置目标点
                // 物体中心 + 指向房间中心的方向 * 0.8米
                Vector3 roughPosition = target.transform.position + dirToCenter * 0.8f;
                // 修正高度
                roughPosition.y = mita.transform.position.y;

                // 4. 地板吸附 (防止出界)
                if (floorCollider != null)
                {
                    Vector3 clamped = floorCollider.ClosestPoint(roughPosition);
                    // 如果被大幅度修正，说明点在墙外
                    if (Vector3.Distance(clamped, roughPosition) > 0.05f)
                    {
                        roughPosition = clamped;
                    }
                }

                // 5. NavMesh 采样 (缩小半径！)
                NavMeshHit navHit;
                Vector3 finalPosition = roughPosition;
                // 将采样半径从 2.0f 缩小到 0.2f
                // 这样如果点在墙里，采样会失败，而不是吸附到隔壁房间
                if (NavMesh.SamplePosition(roughPosition, out navHit, 0.2f, -1))
                {
                    finalPosition = navHit.position;
                }
                else
                {
                    // 如果 0.2米内没有路，说明点可能还在家具里或者墙里
                    // 尝试往房间中心再拉远一点 (1.5米)
                    Vector3 fallbackPos = target.transform.position + dirToCenter * 1.5f;
                    fallbackPos.y = mita.transform.position.y;

                    if (NavMesh.SamplePosition(fallbackPos, out navHit, 1.0f, -1))
                    {
                        finalPosition = navHit.position;
                        ConsoleMain.ConsolePrintGame("目标点太靠近障碍物，已调整距离。");
                    }
                }

                // 创建导航点
                GameObject destPoint = new GameObject("AI_Nav_Target");
                destPoint.transform.position = finalPosition;

                // 6. 执行移动
                IsInternalCall = true;
                mita.AiWalkToTarget(destPoint.transform);
                IsInternalCall = false;

                float timeout = 20.0f;
                float timer = 0f;
                bool arrived = false;
                var agent = mita.GetComponent<NavMeshAgent>();

                while (timer < timeout)
                {
                    if (mita == null) break;
                    mita.MagnetOff();

                    float dist = Vector2.Distance(
                        new Vector2(mita.transform.position.x, mita.transform.position.z),
                        new Vector2(finalPosition.x, finalPosition.z)
                    );

                    if (dist < 0.6f)
                    {
                        arrived = true;
                        ConsoleMain.ConsolePrintGame("米塔已到达。");
                        mita.AiShraplyStop();
                        if (agent != null)
                        {
                            agent.velocity = Vector3.zero;
                            agent.isStopped = true;
                            agent.ResetPath();
                        }
                        break;
                    }
                    timer += Time.deltaTime;
                    yield return null;
                }

                if (!arrived) ConsoleMain.ConsolePrintGame("移动结束。");
                UnityEngine.Object.Destroy(destPoint);
            }
            else
            {
                ConsoleMain.ConsolePrintGame($"<color=red>未找到 '{nameKey}'。</color>");
                IsAIControllingMovement = false;
            }
        }

        // --- 辅助方法：智能寻找地板 ---
        private static GameObject GetFloorForObject(GameObject obj)
        {
            Transform current = obj.transform;
            while (current != null)
            {
                string name = current.name;
                if (_floorCache.ContainsKey(name)) return _floorCache[name];

                if (name.Contains("Bedroom")) return FindAndCacheFloor(current, "FloorBedroom", name);
                if (name.Contains("Toilet")) return FindAndCacheFloor(current, "FloorToilet", name);
                if (name.Contains("Kitchen")) return FindAndCacheFloor(current, "FloorKitchen", name);
                if (name.Contains("Main")) return FindAndCacheFloor(current, "RoomMain", name);

                current = current.parent;
            }
            return null;
        }

        private static GameObject FindAndCacheFloor(Transform roomRoot, string floorName, string cacheKey)
        {
            Transform floor = roomRoot.Find(floorName);
            if (floor != null)
            {
                _floorCache[cacheKey] = floor.gameObject;
                return floor.gameObject;
            }
            return null;
        }

        // --- 核心修复：完全弃用磁吸，还原自然跟随 ---
        private static IEnumerator ResumeFollowPlayer()
        {
            var mita = UnityEngine.Object.FindObjectOfType<MitaPerson>();
            var playerCam = Camera.main;

            if (mita != null && playerCam != null)
            {
                ConsoleMain.ConsolePrintGame("米塔恢复自由跟随...");

                // 1. 确保磁吸是关闭的
                mita.MagnetOff();

                // 2. 关键：解除我们的控制权锁定
                // 这样 DialoguePatches 里的 BlockWalkIfAIControlled 就会放行游戏原生的指令
                IsAIControllingMovement = false;

                // 3. 给一个“回归”的初速度
                // 我们调用一次原生的走路方法，让她立刻转身向你走来
                // 之后，游戏原本的 Update 逻辑会接管她，让她保持跟随

                // 我们需要使用通行证，因为虽然上面设为 false 了，
                // 但为了保险起见，我们明确告诉系统这是内部调用
                IsInternalCall = true;

                // 直接把目标设为摄像机（玩家位置）
                // 游戏原生的 AiWalkToTarget 应该处理了“走到附近就停下”的逻辑
                mita.AiWalkToTarget(playerCam.transform);

                IsInternalCall = false;
            }
            yield return null;
        }

        private static GameObject FindObjectSmart(string key)
        {
            var allRenderers = UnityEngine.Object.FindObjectsOfType<Renderer>();
            var matches = new System.Collections.Generic.List<GameObject>();
            foreach (var r in allRenderers)
                if (r.name.IndexOf(key, StringComparison.OrdinalIgnoreCase) >= 0) matches.Add(r.gameObject);
            if (matches.Count == 0) return null;
            return matches.OrderByDescending(g => g.name.StartsWith(key, StringComparison.OrdinalIgnoreCase))
                          .ThenBy(g => g.name.Length).FirstOrDefault();
        }

        private static IEnumerator PlayAnimTest(string inputName)
        {
            var mitaRoot = GameObject.Find("MitaPerson Mita");
            if (mitaRoot == null) yield break;
            var animator = mitaRoot.GetComponentInChildren<Animator>();
            if (animator == null) yield break;
            string realName = inputName;
            bool found = false;
            if (animator.runtimeAnimatorController != null)
            {
                foreach (var clip in animator.runtimeAnimatorController.animationClips)
                {
                    if (clip.name.Equals(inputName, StringComparison.OrdinalIgnoreCase)) { realName = clip.name; found = true; break; }
                }
            }
            if (found) ConsoleMain.ConsolePrintGame($"播放: [{realName}]");
            else ConsoleMain.ConsolePrintGame($"尝试强制播放: [{inputName}]");
            for (int i = 1; i < animator.layerCount; i++) animator.SetLayerWeight(i, 1.0f);
            for (int i = 0; i < animator.layerCount; i++) animator.CrossFadeInFixedTime(realName, 0.25f, i);
            yield return null;
        }

        private static IEnumerator ListAnimations()
        {
            var mitaRoot = GameObject.Find("MitaPerson Mita");
            if (mitaRoot != null)
            {
                var animator = mitaRoot.GetComponentInChildren<Animator>();
                if (animator != null && animator.runtimeAnimatorController != null)
                {
                    ConsoleMain.ConsolePrintGame("=== 动画列表 ===");
                    foreach (var clip in animator.runtimeAnimatorController.animationClips) ConsoleMain.ConsolePrintGame(clip.name);
                }
            }
            yield return null;
        }

        private static float[] ParseCoordinates(string input)
        {
            try
            {
                string clean = Regex.Replace(input, @"[^\d.,\s]", "");
                string[] parts = clean.Split(new char[] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length >= 4) return new float[] { float.Parse(parts[0]), float.Parse(parts[1]), float.Parse(parts[2]), float.Parse(parts[3]) };
            }
            catch { }
            return null;
        }
    }
}