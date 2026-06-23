/*
 * [文件说明]: 3D文字生成、分句演示与物理掉落
 * 
 * [分析过程]:
 * 1. 游戏原生的 Dialogue_3DText 是"逐字打印"的。
 * 2. [痛点解决]: 直接修改 textPrint 会导致文字堆叠，因为原生 Start() 方法负责计算间距。我们采用了"偷天换日"法：让原生 Start() 跑完初始化，再在 Postfix 中注入文本。
 * 3. 为了处理长文本，引入了队列 (Queue) 和正则分句。
 * 4. 为了保留游戏特色，我们在文字显示完毕后，启用 Rigidbody 实现"文字物理掉落"特效。
 * 
 * [主要功能]:
 * 1. ShowLongText(): 入口，将长文本按标点切分。
 * 2. PlaySequenceRoutine(): 协程，负责"生成 -> 打印 -> 等待阅读 -> 物理掉落 -> 销毁"的完整演出流程。
 * 3. FindAndCacheAssets(): 自动捕获游戏原生的 3DText 预制件。
 */
using System.Collections;
using System.Collections.Generic;
using System.Text.RegularExpressions;
using BepInEx.Unity.IL2CPP.Utils;
using UnityEngine;

namespace MakemitAGA.Dialogue
{
    public static class GameUIManager
    {
        private static GameObject _3dTextPrefab;
        private static Transform _dialogueParent;
        public static Dictionary<Dialogue_3DText, string> PendingInjections = new Dictionary<Dialogue_3DText, string>();
        private static Coroutine _currentSequenceRoutine;

        public static void FindAndCacheAssets()
        {
            if (_3dTextPrefab != null) return;
            var dialogueQuestMita = GameObject.Find("DialogueQuest Mita");
            if (dialogueQuestMita == null) return;
            _dialogueParent = dialogueQuestMita.transform;
            var prefabComponent = _dialogueParent.GetComponentInChildren<Dialogue_3DText>(true);
            if (prefabComponent != null) _3dTextPrefab = prefabComponent.gameObject;
        }

        public static void ShowLongText(string fullText)
        {
            if (_3dTextPrefab == null) { FindAndCacheAssets(); if (_3dTextPrefab == null) return; }

            string[] sentences = Regex.Split(fullText, @"(?<=[。！？!?.])");
            Queue<string> sentenceQueue = new Queue<string>();
            foreach (var s in sentences)
            {
                string trimmed = s.Trim();
                if (!string.IsNullOrWhiteSpace(trimmed)) sentenceQueue.Enqueue(trimmed);
            }

            if (sentenceQueue.Count == 0) return;
            if (_currentSequenceRoutine != null) Plugin.Runner.StopCoroutine(_currentSequenceRoutine);
            _currentSequenceRoutine = Plugin.Runner.StartCoroutine(PlaySequenceRoutine(sentenceQueue));
        }

        private static IEnumerator PlaySequenceRoutine(Queue<string> queue)
        {
            while (queue.Count > 0)
            {
                string currentLine = queue.Dequeue();

                var newInstance = Object.Instantiate(_3dTextPrefab, _dialogueParent);
                newInstance.name = "AI_3DText_Instance";
                newInstance.SetActive(false);

                var dialogueComponent = newInstance.GetComponent<Dialogue_3DText>();
                if (dialogueComponent != null)
                {
                    dialogueComponent.indexString = 1;
                    PendingInjections[dialogueComponent] = currentLine;
                    dialogueComponent.timeFinish = 9999f;
                }
                newInstance.SetActive(true);

                float typingDuration = currentLine.Length * 0.05f;
                float readingDuration = 2.0f + (currentLine.Length * 0.1f);
                yield return new WaitForSeconds(typingDuration + readingDuration);

                // 物理掉落
                if (dialogueComponent != null && dialogueComponent.symbolObjects != null)
                {
                    foreach (var symbolObj in dialogueComponent.symbolObjects)
                    {
                        if (symbolObj == null) continue;
                        var rb = symbolObj.GetComponent<Rigidbody>();
                        if (rb != null)
                        {
                            rb.isKinematic = false;
                            rb.useGravity = true;
                            rb.AddForce(UnityEngine.Random.insideUnitSphere, ForceMode.Impulse);
                        }
                        var collider = symbolObj.GetComponent<Collider>();
                        if (collider != null) collider.enabled = true;
                    }
                }
                yield return new WaitForSeconds(1.5f);
                if (newInstance != null) Object.Destroy(newInstance);
                yield return new WaitForSeconds(0.2f);
            }
            _currentSequenceRoutine = null;
        }
    }
}