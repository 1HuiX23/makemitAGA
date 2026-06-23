"""
MiSide Online AI API Server v2.3

正式 MakemitAGA 后端：
- 默认读取 BepInEx/plugins/config.json；
- 唯一必须生成的运行时文件是 plugins/cache.jpg；
- 默认不创建 backend_boot.log、backend_last_prompt.txt、backend_last_reply.txt、
  backend_last_error.txt 或 backend_startup_error.txt；
- config.json 中 WRITE_BACKEND_DEBUG_FILE=true 时，以上诊断内容统一写入
  plugins/backend_debug.txt；
- stdout/stderr 始终实时输出，由 Plugin.cs 转发到 BepInEx 控制台。
"""

import base64
import json
import os
import sys
import threading
import traceback
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import requests


def is_nuitka_compiled() -> bool:
    return "__compiled__" in globals()


def application_base_dir() -> Path:
    if is_nuitka_compiled() or getattr(sys, "frozen", False):
        for candidate in (
            Path(sys.argv[0]).resolve() if sys.argv else None,
            Path(sys.executable).resolve() if sys.executable else None,
        ):
            if candidate is not None and candidate.suffix.lower() == ".exe":
                return candidate.parent
    return Path(__file__).resolve().parent


BASE_DIR = application_base_dir()
CONFIG_PATH = Path(
    os.environ.get("MISIDE_CONFIG_PATH", str(BASE_DIR / "config.json"))
).resolve()
CACHE_IMAGE_PATH = Path(
    os.environ.get("MISIDE_CACHE_PATH", str(BASE_DIR / "cache.jpg"))
).resolve()

DEBUG_FILE_PATH = BASE_DIR / "backend_debug.txt"
LEGACY_DEBUG_PATHS = (
    BASE_DIR / "backend_boot.log",
    BASE_DIR / "backend_last_prompt.txt",
    BASE_DIR / "backend_last_reply.txt",
    BASE_DIR / "backend_last_error.txt",
    BASE_DIR / "backend_startup_error.txt",
)

DEFAULT_BASE_URL = "https://api-inference.modelscope.cn/v1/chat/completions"

API_KEY = ""
MODEL_ID = ""
SYSTEM_PROMPT = ""
BASE_URL = DEFAULT_BASE_URL
MAX_TOKENS = 512
TEMPERATURE = 0.1
CONNECT_TIMEOUT = 20.0
READ_TIMEOUT = 240.0
WRITE_BACKEND_DEBUG_FILE = False

REQUEST_LOCK = threading.Lock()
DEBUG_FILE_LOCK = threading.Lock()


def parse_bool(value, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def append_debug_record(kind: str, text: str) -> None:
    """Append one chronological record to the single optional debug file."""
    if not WRITE_BACKEND_DEBUG_FILE:
        return

    try:
        with DEBUG_FILE_LOCK:
            with DEBUG_FILE_PATH.open("a", encoding="utf-8", newline="\n") as handle:
                handle.write(f"\n[{timestamp()}] [{kind}]\n")
                handle.write(str(text or ""))
                if not str(text or "").endswith("\n"):
                    handle.write("\n")
    except Exception:
        # Debug-file failure must never break the HTTP backend or recurse through log().
        pass


def log(message: str, *, error: bool = False) -> None:
    """Always print to console; optionally mirror into backend_debug.txt."""
    line = str(message)
    print(line, file=sys.stderr if error else sys.stdout, flush=True)
    append_debug_record("STDERR" if error else "STDOUT", line)


def cleanup_legacy_debug_files() -> None:
    """Remove files produced by backend v2.1 and earlier."""
    for path in LEGACY_DEBUG_PATHS:
        try:
            if path.exists():
                path.unlink()
        except Exception:
            pass


def initialize_debug_file() -> None:
    cleanup_legacy_debug_files()

    if not WRITE_BACKEND_DEBUG_FILE:
        try:
            if DEBUG_FILE_PATH.exists():
                DEBUG_FILE_PATH.unlink()
        except Exception:
            pass
        return

    try:
        DEBUG_FILE_PATH.write_text(
            f"[{timestamp()}] MiSide backend unified debug file started\n",
            encoding="utf-8",
        )
    except Exception:
        pass


def load_config() -> dict:
    if not CONFIG_PATH.is_file():
        raise RuntimeError(
            f"AI config not found: {CONFIG_PATH}. "
            "Start the MakemitAGA plugin once so it creates plugins/config.json."
        )

    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))

    for key in ("API_KEY", "MODEL_ID", "SYSTEM_PROMPT"):
        if key not in config:
            raise RuntimeError(f"AI config missing field: {key}")

    if not str(config.get("API_KEY", "")).strip():
        raise RuntimeError(f"API_KEY is empty in {CONFIG_PATH}")

    return config


def initialize_config() -> None:
    global API_KEY, MODEL_ID, SYSTEM_PROMPT, BASE_URL
    global MAX_TOKENS, TEMPERATURE, CONNECT_TIMEOUT, READ_TIMEOUT
    global WRITE_BACKEND_DEBUG_FILE

    config = load_config()
    API_KEY = str(config["API_KEY"])
    MODEL_ID = str(config["MODEL_ID"])
    SYSTEM_PROMPT = str(config.get("SYSTEM_PROMPT", ""))
    BASE_URL = str(config.get("BASE_URL", DEFAULT_BASE_URL))
    MAX_TOKENS = int(config.get("MAX_TOKENS", 512))
    TEMPERATURE = float(config.get("TEMPERATURE", 0.1))
    CONNECT_TIMEOUT = float(config.get("CONNECT_TIMEOUT_SECONDS", 20))
    READ_TIMEOUT = float(config.get("READ_TIMEOUT_SECONDS", 240))
    WRITE_BACKEND_DEBUG_FILE = parse_bool(
        config.get("WRITE_BACKEND_DEBUG_FILE", False),
        False,
    )


def clean_transport_text(text: str) -> str:
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
                value = item.get("text")
                if isinstance(value, str):
                    parts.append(value)
            elif isinstance(item, str):
                parts.append(item)
        return "".join(parts)

    return str(content or "")


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "MiSideOnlineAI/2.3"

    def do_GET(self):
        if self.path.rstrip("/") == "/health":
            self._write_response(
                200,
                json.dumps(
                    {
                        "ok": True,
                        "version": "2.3",
                        "model": MODEL_ID,
                        "config_path": str(CONFIG_PATH),
                        "cache_path": str(CACHE_IMAGE_PATH),
                        "cache_exists": CACHE_IMAGE_PATH.is_file(),
                        "debug_file_enabled": WRITE_BACKEND_DEBUG_FILE,
                        "nuitka": is_nuitka_compiled(),
                    },
                    ensure_ascii=False,
                ),
                "application/json; charset=utf-8",
            )
            return

        self._write_response(404, "not found")

    def do_POST(self):
        request_id = self.headers.get("X-MiSide-Request-Id", "?")
        run_id = self.headers.get("X-MiSide-Run-Id", "?")
        state = self.headers.get("X-MiSide-State", "")
        protocol = self.headers.get("X-MiSide-Protocol", "legacy")
        include_image = self.headers.get("X-MiSide-Include-Image", "0") == "1"

        try:
            length = int(self.headers.get("Content-Length", "0"))
            prompt = self.rfile.read(length).decode("utf-8")
        except Exception as exc:
            self._write_response(400, f"invalid request body: {exc}")
            return

        log(
            f"[request run={run_id} id={request_id}] protocol={protocol} "
            f"state={state} include_image={include_image} prompt_chars={len(prompt)}"
        )
        append_debug_record(
            f"PROMPT run={run_id} id={request_id} state={state}",
            prompt,
        )

        try:
            user_content = self._build_user_content(prompt, include_image)
            messages = []
            if SYSTEM_PROMPT.strip():
                messages.append({"role": "system", "content": SYSTEM_PROMPT})
            messages.append({"role": "user", "content": user_content})

            payload = {
                "model": MODEL_ID,
                "messages": messages,
                "stream": False,
                "max_tokens": MAX_TOKENS,
                "temperature": TEMPERATURE,
            }

            headers = {
                "Authorization": f"Bearer {API_KEY}",
                "User-Agent": "MiSideOnlineAI/2.3",
                "Content-Type": "application/json",
            }

            with REQUEST_LOCK:
                response = requests.post(
                    BASE_URL,
                    headers=headers,
                    json=payload,
                    timeout=(CONNECT_TIMEOUT, READ_TIMEOUT),
                )

            log(
                f"[request run={run_id} id={request_id}] "
                f"upstream_status={response.status_code} bytes={len(response.content)}"
            )

            if response.status_code != 200:
                error_body = response.text[:4000]
                append_debug_record(
                    f"UPSTREAM_ERROR run={run_id} id={request_id}",
                    error_body,
                )
                self._write_response(
                    502,
                    f"upstream HTTP {response.status_code}: {error_body}",
                )
                return

            reply = clean_transport_text(extract_assistant_content(response.json()))
            if not reply:
                raise ValueError("upstream returned empty assistant content")

            log(
                f"[request run={run_id} id={request_id}] reply_chars={len(reply)}"
            )
            log(f"[assistant] {reply}")
            append_debug_record(
                f"REPLY run={run_id} id={request_id}",
                reply,
            )
            self._write_response(200, reply)

        except requests.RequestException as exc:
            error = f"upstream request exception: {type(exc).__name__}: {exc}"
            append_debug_record(
                f"REQUEST_EXCEPTION run={run_id} id={request_id}",
                error,
            )
            log(error, error=True)
            self._write_response(502, error)

        except Exception as exc:
            error = f"backend exception: {type(exc).__name__}: {exc}"
            details = traceback.format_exc()
            append_debug_record(
                f"BACKEND_EXCEPTION run={run_id} id={request_id}",
                error + "\n" + details,
            )
            log(error, error=True)
            log(details, error=True)
            self._write_response(500, error)

    def _build_user_content(self, prompt: str, include_image: bool):
        if not include_image:
            log("image: disabled by request header")
            return prompt

        if not CACHE_IMAGE_PATH.is_file():
            raise FileNotFoundError(
                f"image requested but cache.jpg does not exist: {CACHE_IMAGE_PATH}"
            )

        image_bytes = CACHE_IMAGE_PATH.read_bytes()
        log(f"image: {CACHE_IMAGE_PATH} bytes={len(image_bytes)}")

        data_url = "data:image/jpeg;base64," + base64.b64encode(
            image_bytes
        ).decode("ascii")

        return [
            {"type": "image_url", "image_url": {"url": data_url}},
            {"type": "text", "text": prompt},
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
    log("MiSide Online AI API Server v2.3")
    log(f"base_dir={BASE_DIR}")
    log(f"config={CONFIG_PATH}")
    log(f"cache={CACHE_IMAGE_PATH}")
    log(f"model={MODEL_ID}")
    log(f"debug_file_enabled={WRITE_BACKEND_DEBUG_FILE}")
    server = ThreadingHTTPServer(("127.0.0.1", port), RequestHandler)
    log(f"SERVER_READY http://127.0.0.1:{port}")
    try:
        server.serve_forever()
    finally:
        server.server_close()


def main() -> int:
    try:
        initialize_config()
        initialize_debug_file()
        run_server()
        return 0
    except Exception as exc:
        # Config may fail before the optional file switch is available,
        # but stderr still reaches the BepInEx console.
        log(f"startup exception: {type(exc).__name__}: {exc}", error=True)
        log(traceback.format_exc(), error=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
