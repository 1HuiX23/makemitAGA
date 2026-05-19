import json
import logging
from http.server import BaseHTTPRequestHandler, HTTPServer
import requests
import urllib.parse
import os
import base64 # 用于将图片文件编码为 base64

# --- 配置文件处理 ---
CONFIG_FILE_PATH = "config.json"

def load_config():
    """加载配置文件，如果不存在则创建模板"""
    if not os.path.exists(CONFIG_FILE_PATH):
        print(f"配置文件 '{CONFIG_FILE_PATH}' 不存在，正在创建模板...")
        default_config = {
            "API_KEY": "YOUR_MODELSCOPE_ACCESS_TOKEN_HERE",
            "MODEL_ID": "Qwen/QVQ-72B-Preview",
            "SYSTEM_PROMPT": ""
        }
        with open(CONFIG_FILE_PATH, 'w', encoding='utf-8') as f:
            json.dump(default_config, f, ensure_ascii=False, indent=4)
        print(f"已创建 '{CONFIG_FILE_PATH}' 模板。请编辑此文件，填入你的 API_KEY, MODEL_ID 和 SYSTEM_PROMPT，然后重新启动服务器。")
        return None # 表示需要用户先配置
    else:
        try:
            with open(CONFIG_FILE_PATH, 'r', encoding='utf-8') as f:
                config = json.load(f)
            # 验证必要字段是否存在
            required_keys = ['API_KEY', 'MODEL_ID', 'SYSTEM_PROMPT']
            for key in required_keys:
                if key not in config:
                    print(f"错误：'{CONFIG_FILE_PATH}' 文件缺少必要的 '{key}' 字段。请检查文件格式。")
                    return None
            print(f"成功加载配置文件 '{CONFIG_FILE_PATH}'。")
            return config
        except json.JSONDecodeError as e:
            print(f"错误：'{CONFIG_FILE_PATH}' 文件格式不是有效的 JSON。{e}")
            return None
        except Exception as e:
            print(f"读取配置文件时发生错误: {e}")
            return None

# 加载配置
config = load_config()

# 如果配置加载失败或缺失，退出程序
if not config:
    exit(1)

# 从配置文件读取
API_KEY = config["API_KEY"]
MODEL_ID = config["MODEL_ID"]
SYSTEM_PROMPT = config["SYSTEM_PROMPT"] # 从配置文件加载

# 检查 API_KEY 是否为默认值，给出警告
if API_KEY == "YOUR_MODELSCOPE_ACCESS_TOKEN_HERE":
    print("警告：API_KEY 仍然是默认值 'YOUR_MODELSCOPE_ACCESS_TOKEN_HERE'。请更新 config.json 文件。")
    exit(1)

# 检查 MODEL_ID 是否为默认值（可选警告）
if MODEL_ID == "Qwen/QVQ-72B-Preview":
    print("提示：MODEL_ID 为默认值 'Qwen/QVQ-72B-Preview'。如需更换，请更新 config.json 文件。")

# --- 定义全局常量 ---
BASE_URL = "https://api-inference.modelscope.cn/v1/chat/completions"  # API 地址

# 全局变量存储聊天历史
chat_history = []

# 初始化聊天历史，加入 System Prompt (从配置文件加载)
chat_history.append({
    "role": "system",
    "content": SYSTEM_PROMPT # 使用配置文件中的 SYSTEM_PROMPT
})

print("System Prompt 已从配置文件加载。")

class RequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length).decode('utf-8')

        print(f"收到请求: {post_data}")

        # --- 检查 cache.jpg 是否存在 ---
        cache_img_path = "cache.jpg" # 定义图片路径
        img_exists = os.path.isfile(cache_img_path) # 检查文件是否存在
        img_base64_str = None # 存储纯 base64 字符串
        if img_exists:
             try:
                 # 读取图片文件并编码为 base64
                 with open(cache_img_path, "rb") as img_file:
                     img_base64_bytes = img_file.read()
                     img_base64_str = base64.b64encode(img_base64_bytes).decode('utf-8')
                 print(f"检测到图片: {cache_img_path}，已准备上传。")
             except Exception as e:
                 print(f"读取图片文件时出错: {e}")
                 # 如果读取失败，当作图片不存在处理
                 img_exists = False
                 img_base64_str = None
        else:
             print(f"未找到图片: {cache_img_path}，仅发送文本。")


        # --- 构建发送给 API 的消息 ---
        if img_exists and img_base64_str:
            # 构建包含图片和文本的消息
            # 使用标准的 data URL 格式: image/<format>;base64,<base64_string>
            full_data_url = f"data:image/jpeg;base64,{img_base64_str}"
            user_message_for_api = [
                {
                    "type": "image_url",
                    "image_url": {"url": full_data_url} # <-- 成功的关键：使用 image/jpeg;base64,...
                },
                {
                    "type": "text",
                    "text": post_data # 使用收到的文本
                }
            ]
        else:
            # 仅包含文本的消息
            user_message_for_api = post_data

        # 将构建好的消息（可能是字符串或列表）添加到聊天历史
        chat_history.append({"role": "user", "content": user_message_for_api})

        # 构建 API 请求体
        payload = {
            "model": MODEL_ID, # 从配置文件加载
            "messages": chat_history,
            "stream": False,  # 设置为 False 获取完整回复
            "max_tokens": 4096
        }

        headers = {
            "Authorization": f"Bearer {API_KEY}", # 从配置文件加载
            "User-Agent": "OnlineAIApiServer/1.0",
            "Content-Type": "application/json"
        }

        try:
            # 发送请求到 ModelScope API
            response = requests.post(BASE_URL, headers=headers, json=payload)

            if response.status_code == 200:
                api_response = response.json()

                if 'choices' in api_response and len(api_response['choices']) > 0:
                    ai_reply = api_response['choices'][0].get('message', {}).get('content', '')

                    # 将 AI 回复添加到聊天历史
                    chat_history.append({"role": "assistant", "content": ai_reply})

                    # 清理回复（移除换行符等，以防闪退）
                    # 参考 C# 版本的正则表达式逻辑，这里用 Python 的 re 模块实现类似效果
                    import re
                    # 正则表达式匹配需要保留的字符：中文、英文字母、数字、空白符、常见标点
                    pattern = r'[^\u4e00-\u9fff\w\s，。！？；：""''（）\[\]{}\-_]'
                    clean_reply = re.sub(pattern, '', ai_reply).strip() # 先过滤，再去除首尾空格
                    clean_reply = clean_reply.replace("\n", "").replace("\r", "") # 再移除换行符
                    clean_reply = clean_reply.strip() # 最后再 strip 一次

                    print(f"发送回复: {clean_reply}")

                    # 发送响应
                    self.send_response(200)
                    self.send_header('Content-type', 'text/plain; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(clean_reply.encode('utf-8'))
                else:
                    print("API 响应中没有有效内容或选择。")
                    error_msg = "Error: No valid response from API."
                    self.send_error(502, error_msg) # Bad Gateway
            else:
                print(f"API 请求失败，状态码: {response.status_code}, 内容: {response.text}")
                error_msg = "Error: Failed to get response from API."
                self.send_error(502, error_msg) # Bad Gateway

        except requests.exceptions.RequestException as e:
            print(f"请求 API 时发生错误: {e}")
            error_msg = "Error: Failed to connect to API."
            self.send_error(500, error_msg) # Internal Server Error
        except Exception as e:
            print(f"处理请求时发生未知错误: {e}")
            error_msg = "Error: An unexpected error occurred."
            self.send_error(500, error_msg) # Internal Server Error

    def log_message(self, format, *args):
        # 重写日志方法，抑制默认的访问日志输出到控制台
        # 如需详细日志，可以使用 logging 模块
        pass


def run_server(server_class=HTTPServer, handler_class=RequestHandler, port=8080):
    server_address = ('localhost', port)
    httpd = server_class(server_address, handler_class)
    print(f"AI服务器已在 http://localhost:{port} 启动，等待来自游戏的请求...")
    print(f"注意：若存在 'cache.jpg'，每次请求将携带此图片。")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n服务器关闭。")
        httpd.shutdown()


if __name__ == '__main__':
    # 可选：配置 logging 以便更灵活地管理日志
    # logging.basicConfig(level=logging.INFO)
    run_server()
    #nuitka --onefile --output-filename=OnlineAIApiServer.exe online_api_server.py
