/*
 * SeatVlmAIClient.cs
 * Seat VLM 专用的本地 HTTP 客户端：支持 health 等待、状态 header、分块读取和有限重试。
 * 所有过程只写入 BepInEx 控制台，不创建 prompt/reply/error 文本文件。
 */
using System;
using System.IO;
using System.Net.Http;
using System.Text;
using System.Threading.Tasks;
using MakemitAGA.Mita_self.Mita_tools;

namespace MakemitAGA.Connection
{
    internal static class SeatVlmAIClient
    {
        private static readonly HttpClientHandler LocalHandler =
            new HttpClientHandler
            {
                UseProxy = false
            };

        private static readonly HttpClientHandler HealthHandler =
            new HttpClientHandler
            {
                UseProxy = false
            };

        private static readonly HttpClient Client =
            new HttpClient(LocalHandler)
            {
                Timeout = TimeSpan.FromSeconds(
                    SeatVlmConfig.ModelHttpTimeoutSeconds)
            };

        private static readonly HttpClient HealthClient =
            new HttpClient(HealthHandler)
            {
                Timeout = TimeSpan.FromSeconds(
                    SeatVlmConfig.BackendHealthRequestTimeoutSeconds)
            };

        public static async Task<string> GetResponseStreamingAsync(
            string prompt,
            bool includeImage,
            int runId,
            int requestId,
            string stateName,
            Action<string> onChunk,
            Action<string> onStage)
        {
            bool backendReady =
                await WaitForBackendReadyAsync(onStage);

            if (!backendReady)
            {
                return
                    "AI_ERROR:本地后端未能在限定时间内通过 /health 检查。" +
                    "详细过程已经输出到 BepInEx 控制台。";
            }

            int maxAttempts =
                1 + SeatVlmConfig.MaxTransientHttpRetries;

            string lastError = null;

            for (int attempt = 1; attempt <= maxAttempts; attempt++)
            {
                try
                {
                    onStage?.Invoke(
                        "正在连接 " +
                        SeatVlmConfig.BackendUrl +
                        " | attempt=" + attempt + "/" + maxAttempts);

                    using (var request = new HttpRequestMessage(
                        HttpMethod.Post,
                        SeatVlmConfig.BackendUrl))
                    {
                        request.Content = new StringContent(
                            prompt ?? "",
                            Encoding.UTF8,
                            "text/plain");

                        // v0.1.4 backend protocol:
                        // 后端只在这个 header 为 1 时附加 cache.jpg，避免第一步误带旧截图，
                        // 也避免“只要文件存在，每次请求都附图”的隐式行为。
                        request.Headers.TryAddWithoutValidation(
                            "X-MiSide-Protocol",
                            "vision-tool-v1");
                        request.Headers.TryAddWithoutValidation(
                            "X-MiSide-Include-Image",
                            includeImage ? "1" : "0");
                        request.Headers.TryAddWithoutValidation(
                            "X-MiSide-Run-Id",
                            runId.ToString());
                        request.Headers.TryAddWithoutValidation(
                            "X-MiSide-Request-Id",
                            requestId.ToString());
                        request.Headers.TryAddWithoutValidation(
                            "X-MiSide-State",
                            stateName ?? "");

                        using (HttpResponseMessage response =
                            await Client.SendAsync(
                                request,
                                HttpCompletionOption.ResponseHeadersRead))
                        {
                            int status = (int)response.StatusCode;
                            string reason = response.ReasonPhrase ?? "";

                            onStage?.Invoke(
                                "已收到响应头 HTTP " +
                                status + " " + reason +
                                " | attempt=" + attempt + "/" + maxAttempts);

                            if (!response.IsSuccessStatusCode)
                            {
                                string body = "";

                                try
                                {
                                    body =
                                        await response.Content.ReadAsStringAsync();
                                }
                                catch { }

                                lastError =
                                    "HTTP " + status + " " + reason +
                                    (string.IsNullOrWhiteSpace(body)
                                        ? ""
                                        : " / " + body);

                                if (IsTransientStatus(status) &&
                                    attempt < maxAttempts)
                                {
                                    float delay =
                                        SeatVlmConfig.HttpRetryBaseDelaySeconds *
                                        attempt;

                                    onStage?.Invoke(
                                        "临时后端错误，将自动重试。" +
                                        " status=" + status +
                                        " | delay=" + delay.ToString("0.0") + "s");

                                    await Task.Delay(
                                        (int)(delay * 1000f));

                                    continue;
                                }

                                return "AI_ERROR:" + lastError;
                            }

                            using (Stream stream =
                                await response.Content.ReadAsStreamAsync())
                            using (var reader = new StreamReader(
                                stream,
                                Encoding.UTF8,
                                true,
                                1024,
                                false))
                            {
                                char[] buffer = new char[256];
                                StringBuilder full = new StringBuilder();

                                while (true)
                                {
                                    int read = await reader.ReadAsync(
                                        buffer,
                                        0,
                                        buffer.Length);

                                    if (read <= 0)
                                        break;

                                    string chunk =
                                        new string(buffer, 0, read);

                                    full.Append(chunk);
                                    onChunk?.Invoke(chunk);
                                }

                                onStage?.Invoke(
                                    "响应流读取完成，字符数=" +
                                    full.Length +
                                    " | attempt=" + attempt + "/" + maxAttempts);

                                return full.ToString();
                            }
                        }
                    }
                }
                catch (Exception e)
                {
                    if (e is TaskCanceledException)
                    {
                        lastError =
                            "模型请求超过 " +
                            SeatVlmConfig.ModelHttpTimeoutSeconds +
                            " 秒，已停止等待。";

                        onStage?.Invoke(
                            lastError +
                            " 此类超时不会自动重试，避免一次请求阻塞数分钟。");

                        break;
                    }

                    lastError =
                        e.GetType().Name + " / " + e.Message;

                    if (attempt < maxAttempts &&
                        IsTransientException(e))
                    {
                        float delay =
                            SeatVlmConfig.HttpRetryBaseDelaySeconds *
                            attempt;

                        onStage?.Invoke(
                            "临时网络异常，将自动重试。" +
                            " error=" + lastError +
                            " | delay=" + delay.ToString("0.0") + "s");

                        await Task.Delay(
                            (int)(delay * 1000f));

                        continue;
                    }

                    break;
                }
            }

            return "AI_ERROR:" +
                (string.IsNullOrEmpty(lastError)
                    ? "unknown backend error"
                    : lastError);
        }

        private static async Task<bool> WaitForBackendReadyAsync(
            Action<string> onStage)
        {
            for (
                int attempt = 1;
                attempt <= SeatVlmConfig.BackendHealthMaxAttempts;
                attempt++)
            {
                try
                {
                    using (HttpResponseMessage response =
                        await HealthClient.GetAsync(
                            SeatVlmConfig.BackendHealthUrl))
                    {
                        if (response.IsSuccessStatusCode)
                        {
                            string body = "";

                            try
                            {
                                body =
                                    await response.Content.ReadAsStringAsync();
                            }
                            catch { }

                            onStage?.Invoke(
                                "本地后端 /health 已就绪" +
                                (string.IsNullOrWhiteSpace(body)
                                    ? ""
                                    : " | " + body));

                            return true;
                        }

                        onStage?.Invoke(
                            "本地后端 /health 未就绪：" +
                            (int)response.StatusCode +
                            " | attempt=" + attempt + "/" +
                            SeatVlmConfig.BackendHealthMaxAttempts);
                    }
                }
                catch (Exception e)
                {
                    if (
                        attempt == 1 ||
                        attempt % 5 == 0 ||
                        attempt ==
                        SeatVlmConfig.BackendHealthMaxAttempts)
                    {
                        onStage?.Invoke(
                            "等待本地后端启动：" +
                            e.GetType().Name + " / " + e.Message +
                            " | attempt=" + attempt + "/" +
                            SeatVlmConfig.BackendHealthMaxAttempts);
                    }
                }

                await Task.Delay(
                    SeatVlmConfig
                        .BackendHealthRetryDelayMilliseconds);
            }

            return false;
        }

        private static bool IsTransientStatus(int status)
        {
            return
                status == 408 ||
                status == 429 ||
                status == 500 ||
                status == 502 ||
                status == 503 ||
                status == 504;
        }

        private static bool IsTransientException(Exception e)
        {
            return
                e is HttpRequestException ||
                e is IOException;
        }
    }
}
