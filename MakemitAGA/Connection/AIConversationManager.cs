/*
 * [文件说明]: HTTP 网络通信模块
 * 
 * [分析过程]:
 * 1. 游戏是 IL2CPP，直接嵌入 Python 引擎太复杂。我们选择了 C# (Client) <-> Python (Server) 的 HTTP 架构。
 * 2. 需要支持发送纯文本 (Chat) 和视觉请求 (Vision)。
 * 3. 考虑到多模态大模型的推理时间，设置了 5 分钟的超时时间。
 * 
 * [主要功能]:
 * 1. GetResponseAsync(): 向 http://localhost:8080 发送 POST 请求。
 * 2. 这里的逻辑很简单，因为图片传输是通过文件系统 (cache.jpg) 完成的，这里只负责发送 Prompt 触发后端读取。
 */
using System;
using System.Net.Http;
using System.Threading.Tasks;
using System.Text;

namespace MakemitAGA.Connection
{
    public static class AIConversationManager
    {
        private static readonly HttpClient httpClient = new HttpClient() { Timeout = TimeSpan.FromMinutes(5) };

        public static async Task<string> GetResponseAsync(string userInput)
        {
            try
            {
                var content = new StringContent(userInput, Encoding.UTF8, "text/plain");
                var response = await httpClient.PostAsync("http://localhost:8080/", content);
                response.EnsureSuccessStatusCode();
                return await response.Content.ReadAsStringAsync();
            }
            catch (Exception ex)
            {
                Plugin.Logger.LogError($"AI通信错误: {ex.Message}");
                return "（连接断开...）";
            }
        }
    }
}