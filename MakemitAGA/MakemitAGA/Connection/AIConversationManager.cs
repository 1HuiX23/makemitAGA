/*
 * AIConversationManager.cs
 * 旧 say/look 指令的轻量客户端。
 * look 请求会显式设置 X-MiSide-Include-Image=1，避免后端给普通对话附加旧 cache.jpg。
 */
using System;
using System.Net.Http;
using System.Threading.Tasks;
using System.Text;

namespace MakemitAGA.Connection
{
    public static class AIConversationManager
    {
        private static readonly HttpClientHandler Handler =
            new HttpClientHandler
            {
                UseProxy = false
            };

        private static readonly HttpClient HttpClient =
            new HttpClient(Handler)
            {
                Timeout = TimeSpan.FromMinutes(5)
            };

        public static async Task<string> GetResponseAsync(
            string userInput,
            bool includeImage = false)
        {
            try
            {
                using (var request = new HttpRequestMessage(
                    HttpMethod.Post,
                    "http://127.0.0.1:8080/"))
                {
                    request.Content = new StringContent(
                        userInput ?? "",
                        Encoding.UTF8,
                        "text/plain");

                    request.Headers.TryAddWithoutValidation(
                        "X-MiSide-Protocol",
                        "legacy-dialogue-v1");

                    request.Headers.TryAddWithoutValidation(
                        "X-MiSide-Include-Image",
                        includeImage ? "1" : "0");

                    using (HttpResponseMessage response =
                        await HttpClient.SendAsync(request))
                    {
                        response.EnsureSuccessStatusCode();
                        return await response.Content.ReadAsStringAsync();
                    }
                }
            }
            catch (Exception ex)
            {
                Plugin.Logger.LogError(
                    "AI通信错误: " +
                    ex.Message);
                return "（连接断开...）";
            }
        }
    }
}
