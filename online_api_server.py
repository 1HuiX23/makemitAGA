import base64
import json
import os
import socket
import sys
import threading
import traceback
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import requests


# ======================================================================================
# MiSide Online AI API Server v2.1
# ======================================================================================
# v2.1 关键修复：
#
# 1. 正确识别 Nuitka --onefile。
#    PyInstaller 常用 sys.frozen，但 Nuitka 不保证提供这个标记。
#    旧版在 Nuitka onefile 下可能把 BASE_DIR 指向临时解包目录，导致找不到
#    plugins/config.json，程序创建临时模板后立即退出，最终 C# 只能看到
#    “localhost:8080 积极拒绝连接”。
#
# 2. 所有 config/cache/log 路径都固定为“实际 OnlineAIApiServer.exe 所在目录”。
#
# 3. 启动阶段无论成功或失败都写：
#       backend_boot.log
#       backend_startup_error.txt
#
# 4. 保持 v2.0 协议：
#       POST /
#       Content-Type: text/plain
#       X-MiSide-Include-Image
#       GET /health
#       原样返回 tool_call
# ======================================================================================


def is_nuitka_compiled() -> bool:
    return "__compiled__" in globals()


def application_base_dir() -> Path:
    # Nuitka onefile 下，__file__ 常常指向临时展开目录；
    # sys.argv[0] / sys.executable 才指向用户真正启动的 exe。
    if is_nuitka_compiled() or getattr(sys, "frozen", False):
        candidates = [
            Path(sys.argv[0]).resolve() if sys.argv else None,
            Path(sys.executable).resolve() if sys.executable else None,
        ]
        for candidate in candidates:
            if candidate is not None and candidate.suffix.lower() == ".exe":
                return candidate.parent

    return Path(__file__).resolve().parent


BASE_DIR = application_base_dir()
CONFIG_PATH = BASE_DIR / "config.json"
CACHE_IMAGE_PATH = BASE_DIR / "cache.jpg"
LAST_PROMPT_PATH = BASE_DIR / "backend_last_prompt.txt"
LAST_REPLY_PATH = BASE_DIR / "backend_last_reply.txt"
LAST_ERROR_PATH = BASE_DIR / "backend_last_error.txt"
BOOT_LOG_PATH = BASE_DIR / "backend_boot.log"
STARTUP_ERROR_PATH = BASE_DIR / "backend_startup_error.txt"

DEFAULT_BASE_URL = "https://api-inference.modelscope.cn/v1/chat/completions"

CONFIG = None
API_KEY = ""
MODEL_ID = ""
SYSTEM_PROMPT = ""
BASE_URL = DEFAULT_BASE_URL
MAX_TOKENS = 512
TEMPERATURE = 0.1
CONNECT_TIMEOUT = 20.0
READ_TIMEOUT = 240.0

REQUEST_LOCK = threading.Lock()


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def append_boot_log(message: str) -> None:
    text = f"[{timestamp()}] {message}"
    print(text, flush=True)
    try:
        with BOOT_LOG_PATH.open("a", encoding="utf-8") as f:
            f.write(text + "\n")
    except Exception:
        pass


def safe_write(path: Path, text: str) -> None:
    try:
        path.write_text(text or "", encoding="utf-8")
    except Exception:
        pass


def load_config() -> dict:
    if not CONFIG_PATH.exists():
        template = {
            "API_KEY": "YOUR_MODELSCOPE_ACCESS_TOKEN_HERE",
            "MODEL_ID": "Qwen/QVQ-72B-Preview",
            "SYSTEM_PROMPT": "",
            "BASE_URL": DEFAULT_BASE_URL,
            "MAX_TOKENS": 512,
            "TEMPERATURE": 0.1,
            "CONNECT_TIMEOUT_SECONDS": 20,
            "READ_TIMEOUT_SECONDS": 240,
        }

        CONFIG_PATH.write_text(
            json.dumps(template, ensure_ascii=False, indent=4),
            encoding="utf-8",
        )

        raise RuntimeError(
            f"config.json 不存在，已在 EXE 目录创建模板：{CONFIG_PATH}。"
            "请填写 API_KEY 后重新启动游戏或执行 vt_backend_restart。"
        )

    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    for key in ("API_KEY", "MODEL_ID", "SYSTEM_PROMPT"):
        if key not in config:
            raise RuntimeError(f"config.json 缺少字段：{key}")

    if config["API_KEY"] == "YOUR_MODELSCOPE_ACCESS_TOKEN_HERE":
        raise RuntimeError(
            f"API_KEY 仍是模板值，请编辑：{CONFIG_PATH}"
        )

    return config


def initialize_config() -> None:
    global CONFIG
    global API_KEY
    global MODEL_ID
    global SYSTEM_PROMPT
    global BASE_URL
    global MAX_TOKENS
    global TEMPERATURE
    global CONNECT_TIMEOUT
    global READ_TIMEOUT

    CONFIG = load_config()
    API_KEY = CONFIG["API_KEY"]
    MODEL_ID = CONFIG["MODEL_ID"]
    SYSTEM_PROMPT = CONFIG.get("SYSTEM_PROMPT", "")
    BASE_URL = CONFIG.get("BASE_URL", DEFAULT_BASE_URL)
    MAX_TOKENS = int(CONFIG.get("MAX_TOKENS", 512))
    TEMPERATURE = float(CONFIG.get("TEMPERATURE", 0.1))
    CONNECT_TIMEOUT = float(CONFIG.get("CONNECT_TIMEOUT_SECONDS", 20))
    READ_TIMEOUT = float(CONFIG.get("READ_TIMEOUT_SECONDS", 240))


def clean_transport_text(text: str) -> str:
    # 只清除 NUL，保留 < > / , . 与换行。
    return (text or "").replace("\x00", "").strip()


def extract_assistant_content(api_response: dict) -> str:
    choices = api_response.get("choices") or []
    if not choices:
        raise ValueError("API response has no choices")

    message = choices[0].get("message") or {}
    content = message.get("content", "")

    if isinstance(content, str):
        return content

    if isinstance(content, list):
        parts = []
        for item in content:
            if isinstance(item, dict):
                text = item.get("text")
                if isinstance(text, str):
                    parts.append(text)
            elif isinstance(item, str):
                parts.append(item)
        return "".join(parts)

    return str(content or "")


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "MiSideOnlineAI/2.1"

    def do_GET(self):
        if self.path.rstrip("/") == "/health":
            body = json.dumps(
                {
                    "ok": True,
                    "version": "2.1",
                    "model": MODEL_ID,
                    "base_dir": str(BASE_DIR),
                    "config_path": str(CONFIG_PATH),
                    "cache_path": str(CACHE_IMAGE_PATH),
                    "cache_exists": CACHE_IMAGE_PATH.is_file(),
                    "nuitka": is_nuitka_compiled(),
                },
                ensure_ascii=False,
            )
            self._write_response(
                200,
                body,
                "application/json; charset=utf-8",
            )
            return

        self._write_response(404, "not found")

    def do_POST(self):
        request_id = self.headers.get("X-MiSide-Request-Id", "?")
        run_id = self.headers.get("X-MiSide-Run-Id", "?")
        state = self.headers.get("X-MiSide-State", "")
        protocol = self.headers.get("X-MiSide-Protocol", "legacy")
        include_image = (
            self.headers.get("X-MiSide-Include-Image", "0") == "1"
        )

        try:
            length = int(self.headers.get("Content-Length", "0"))
            prompt = self.rfile.read(length).decode("utf-8")
        except Exception as exc:
            self._write_response(400, f"invalid request body: {exc}")
            return

        safe_write(LAST_PROMPT_PATH, prompt)
        safe_write(LAST_ERROR_PATH, "")

        append_boot_log(
            f"[request run={run_id} id={request_id}] "
            f"protocol={protocol} state={state} "
            f"include_image={include_image} prompt_chars={len(prompt)}"
        )

        try:
            user_content = self._build_user_content(
                prompt,
                include_image,
            )

            messages = []
            if SYSTEM_PROMPT.strip():
                messages.append(
                    {"role": "system", "content": SYSTEM_PROMPT}
                )
            messages.append(
                {"role": "user", "content": user_content}
            )

            payload = {
                "model": MODEL_ID,
                "messages": messages,
                "stream": False,
                "max_tokens": MAX_TOKENS,
                "temperature": TEMPERATURE,
            }

            headers = {
                "Authorization": f"Bearer {API_KEY}",
                "User-Agent": "MiSideOnlineAI/2.1",
                "Content-Type": "application/json",
            }

            with REQUEST_LOCK:
                response = requests.post(
                    BASE_URL,
                    headers=headers,
                    json=payload,
                    timeout=(CONNECT_TIMEOUT, READ_TIMEOUT),
                )

            append_boot_log(
                f"[request run={run_id} id={request_id}] "
                f"upstream_status={response.status_code} "
                f"response_bytes={len(response.content)}"
            )

            if response.status_code != 200:
                detail = response.text[:4000]
                error = (
                    f"upstream HTTP {response.status_code}: {detail}"
                )
                safe_write(LAST_ERROR_PATH, error)
                self._write_response(502, error)
                return

            data = response.json()
            reply = clean_transport_text(
                extract_assistant_content(data)
            )

            if not reply:
                raise ValueError(
                    "upstream returned empty assistant content"
                )

            safe_write(LAST_REPLY_PATH, reply)
            append_boot_log(
                f"[request run={run_id} id={request_id}] "
                f"reply_chars={len(reply)}"
            )
            append_boot_log(f"[assistant] {reply}")

            self._write_response(200, reply)

        except requests.RequestException as exc:
            error = (
                f"upstream request exception: "
                f"{type(exc).__name__}: {exc}"
            )
            safe_write(LAST_ERROR_PATH, error)
            append_boot_log(error)
            self._write_response(502, error)

        except Exception as exc:
            error = (
                f"backend exception: "
                f"{type(exc).__name__}: {exc}"
            )
            safe_write(
                LAST_ERROR_PATH,
                error + "\n" + traceback.format_exc(),
            )
            append_boot_log(error)
            append_boot_log(traceback.format_exc())
            self._write_response(500, error)

    def _build_user_content(
        self,
        prompt: str,
        include_image: bool,
    ):
        if not include_image:
            append_boot_log("image: disabled by request header")
            return prompt

        if not CACHE_IMAGE_PATH.is_file():
            raise FileNotFoundError(
                "X-MiSide-Include-Image=1 but cache.jpg "
                f"does not exist: {CACHE_IMAGE_PATH}"
            )

        image_bytes = CACHE_IMAGE_PATH.read_bytes()
        image_b64 = base64.b64encode(
            image_bytes
        ).decode("ascii")

        data_url = (
            "data:image/jpeg;base64," + image_b64
        )

        append_boot_log(
            f"image: {CACHE_IMAGE_PATH} "
            f"bytes={len(image_bytes)}"
        )

        return [
            {
                "type": "image_url",
                "image_url": {"url": data_url},
            },
            {
                "type": "text",
                "text": prompt,
            },
        ]

    def _write_response(
        self,
        status: int,
        body: str,
        content_type: str = "text/plain; charset=utf-8",
    ) -> None:
        data = (body or "").encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        pass


def run_server(port: int = 8080) -> None:
    address = ("127.0.0.1", port)

    append_boot_log("=" * 80)
    append_boot_log("MiSide Online AI API Server v2.1")
    append_boot_log(f"argv0: {sys.argv[0] if sys.argv else '<none>'}")
    append_boot_log(f"sys.executable: {sys.executable}")
    append_boot_log(f"__file__: {__file__}")
    append_boot_log(
        f"nuitka_compiled: {is_nuitka_compiled()}"
    )
    append_boot_log(f"base_dir: {BASE_DIR}")
    append_boot_log(f"config: {CONFIG_PATH}")
    append_boot_log(f"cache: {CACHE_IMAGE_PATH}")
    append_boot_log(f"listen: http://127.0.0.1:{port}")
    append_boot_log(
        f"health: http://127.0.0.1:{port}/health"
    )
    append_boot_log(f"model: {MODEL_ID}")
    append_boot_log("stateless mode: ON")
    append_boot_log("reply punctuation filtering: OFF")
    append_boot_log("=" * 80)

    server = ThreadingHTTPServer(
        address,
        RequestHandler,
    )

    safe_write(STARTUP_ERROR_PATH, "")
    append_boot_log("SERVER_READY")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        append_boot_log("server stopping...")
    finally:
        server.server_close()


def main() -> int:
    try:
        safe_write(STARTUP_ERROR_PATH, "")
        append_boot_log("process starting")
        initialize_config()
        run_server()
        return 0

    except Exception as exc:
        error = (
            f"startup exception: "
            f"{type(exc).__name__}: {exc}\n"
            f"{traceback.format_exc()}"
        )

        safe_write(STARTUP_ERROR_PATH, error)
        append_boot_log(error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
