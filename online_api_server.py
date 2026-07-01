"""
MiSide Online AI API Server v2.3.1

修复内容：
- 强制 stdout/stderr 使用 UTF-8；日志永不因 emoji/Unicode 失败；
- 新增带随机 token 的 POST /shutdown；
- 新增父进程 watchdog：MiSide 进程消失后后端自动退出；
- ThreadingHTTPServer 线程设为 daemon，端口允许快速复用；
- 保持 legacy-dialogue-v1 与 vision-tool-v1 协议兼容。
"""

import base64
import ctypes
import json
import os
import sys
import threading
import traceback
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import requests


BACKEND_VERSION = "2.3.1"


def configure_utf8_stdio() -> None:
    """Windows 默认 GBK 时，也强制标准输出使用 UTF-8。"""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(
                encoding="utf-8",
                errors="backslashreplace",
                line_buffering=True,
            )
        except Exception:
            pass


configure_utf8_stdio()


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

try:
    PARENT_PID = int(os.environ.get("MISIDE_PARENT_PID", "0") or "0")
except Exception:
    PARENT_PID = 0

SHUTDOWN_TOKEN = os.environ.get("MISIDE_SHUTDOWN_TOKEN", "")

REQUEST_LOCK = threading.Lock()
DEBUG_FILE_LOCK = threading.Lock()
SHUTDOWN_ONCE = threading.Event()


def parse_bool(value, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def append_debug_record(kind: str, text: str) -> None:
    """写入唯一的可选调试文件；失败绝不影响 HTTP。"""
    if not WRITE_BACKEND_DEBUG_FILE:
        return

    try:
        value = str(text or "")
        with DEBUG_FILE_LOCK:
            with DEBUG_FILE_PATH.open(
                "a", encoding="utf-8", newline="\n"
            ) as handle:
                handle.write(f"\n[{timestamp()}] [{kind}]\n")
                handle.write(value)
                if not value.endswith("\n"):
                    handle.write("\n")
    except Exception:
        pass


def safe_stream_write(stream, line: str) -> None:
    """多重兜底，日志输出不能向调用者抛异常。"""
    try:
        print(line, file=stream, flush=True)
        return
    except Exception:
        pass

    try:
        encoding = getattr(stream, "encoding", None) or "utf-8"
        safe = line.encode(
            encoding, errors="backslashreplace"
        ).decode(
            encoding, errors="replace"
        )
        stream.write(safe + "\n")
        stream.flush()
    except Exception:
        pass


def log(message: str, *, error: bool = False) -> None:
    """
    no-throw 日志。
    v2.3 中此处的 GBK UnicodeEncodeError 会把成功回复变成 HTTP 500。
    """
    try:
        line = str(message)
    except Exception:
        line = "<unprintable log message>"

    stream = sys.stderr if error else sys.stdout
    safe_stream_write(stream, line)
    append_debug_record("STDERR" if error else "STDOUT", line)


def cleanup_legacy_debug_files() -> None:
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
        config.get("WRITE_BACKEND_DEBUG_FILE", False), False
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


def is_parent_process_alive(pid: int) -> bool:
    if pid <= 0:
        return True

    if os.name == "nt":
        process_query_limited_information = 0x1000
        still_active = 259

        try:
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.OpenProcess(
                process_query_limited_information, False, pid
            )
            if not handle:
                return False

            try:
                exit_code = ctypes.c_ulong()
                ok = kernel32.GetExitCodeProcess(
                    handle, ctypes.byref(exit_code)
                )
                return bool(ok and exit_code.value == still_active)
            finally:
                kernel32.CloseHandle(handle)
        except Exception:
            # watchdog 自身异常时宁可暂不退出，避免误杀正常后端。
            return True

    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


class MiSideThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def request_server_shutdown(server, reason: str) -> None:
    """shutdown() 必须从 serve_forever() 所在线程以外调用。"""
    if SHUTDOWN_ONCE.is_set():
        return

    SHUTDOWN_ONCE.set()
    log(f"SERVER_SHUTDOWN_REQUESTED reason={reason}")

    def worker():
        try:
            server.shutdown()
        except Exception as exc:
            log(
                "server.shutdown failed: "
                f"{type(exc).__name__}: {exc}",
                error=True,
            )

    threading.Thread(
        target=worker,
        name="MiSideBackendShutdown",
        daemon=True,
    ).start()


def parent_watchdog(server) -> None:
    if PARENT_PID <= 0:
        log("parent watchdog disabled: MISIDE_PARENT_PID is not set")
        return

    log(f"parent watchdog started pid={PARENT_PID}")

    while not SHUTDOWN_ONCE.wait(1.0):
        if is_parent_process_alive(PARENT_PID):
            continue

        log(f"parent process exited pid={PARENT_PID}")
        request_server_shutdown(server, "parent-process-exited")
        return


class RequestHandler(BaseHTTPRequestHandler):
    server_version = "MiSideOnlineAI/" + BACKEND_VERSION

    def do_GET(self):
        if self.path.rstrip("/") == "/health":
            self._write_response(
                200,
                json.dumps(
                    {
                        "ok": True,
                        "version": BACKEND_VERSION,
                        "model": MODEL_ID,
                        "config_path": str(CONFIG_PATH),
                        "cache_path": str(CACHE_IMAGE_PATH),
                        "cache_exists": CACHE_IMAGE_PATH.is_file(),
                        "debug_file_enabled": WRITE_BACKEND_DEBUG_FILE,
                        "nuitka": is_nuitka_compiled(),
                        "parent_pid": PARENT_PID,
                        "shutdown_enabled": bool(SHUTDOWN_TOKEN),
                    },
                    ensure_ascii=False,
                ),
                "application/json; charset=utf-8",
            )
            return

        self._write_response(404, "not found")

    def do_POST(self):
        normalized_path = self.path.rstrip("/")

        if normalized_path == "/shutdown":
            self._handle_shutdown()
            return

        if normalized_path not in ("", "/"):
            self._write_response(404, "not found")
            return

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

        log(
            f"[request run={run_id} id={request_id}] protocol={protocol} "
            f"state={state} include_image={include_image} "
            f"prompt_chars={len(prompt)}"
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
                "User-Agent": "MiSideOnlineAI/" + BACKEND_VERSION,
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
                f"upstream_status={response.status_code} "
                f"bytes={len(response.content)}"
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

            reply = clean_transport_text(
                extract_assistant_content(response.json())
            )
            if not reply:
                raise ValueError("upstream returned empty assistant content")

            log(
                f"[request run={run_id} id={request_id}] "
                f"reply_chars={len(reply)}"
            )

            # log() 已经 no-throw；emoji 不会再把成功请求变成 500。
            log(f"[assistant] {reply}")
            append_debug_record(
                f"REPLY run={run_id} id={request_id}",
                reply,
            )
            self._write_response(200, reply)

        except requests.RequestException as exc:
            error = (
                "upstream request exception: "
                f"{type(exc).__name__}: {exc}"
            )
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

    def _handle_shutdown(self) -> None:
        supplied_token = self.headers.get("X-MiSide-Shutdown-Token", "")

        if not SHUTDOWN_TOKEN or supplied_token != SHUTDOWN_TOKEN:
            log("rejected unauthorized /shutdown", error=True)
            self._write_response(403, "forbidden")
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            reason = self.rfile.read(length).decode(
                "utf-8", errors="replace"
            )
        except Exception:
            reason = "no-reason"

        self._write_response(200, "shutting down")
        request_server_shutdown(
            self.server, reason or "authorized-request"
        )

    def _build_user_content(self, prompt: str, include_image: bool):
        if not include_image:
            log("image: disabled by request header")
            return prompt

        if not CACHE_IMAGE_PATH.is_file():
            raise FileNotFoundError(
                "image requested but cache.jpg does not exist: "
                f"{CACHE_IMAGE_PATH}"
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

        try:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, fmt, *args):
        # 关闭 BaseHTTPRequestHandler 默认访问日志。
        pass


def run_server(port: int = 8080) -> None:
    log(f"MiSide Online AI API Server v{BACKEND_VERSION}")
    log(f"base_dir={BASE_DIR}")
    log(f"config={CONFIG_PATH}")
    log(f"cache={CACHE_IMAGE_PATH}")
    log(f"model={MODEL_ID}")
    log(f"debug_file_enabled={WRITE_BACKEND_DEBUG_FILE}")
    log(f"parent_pid={PARENT_PID}")
    log(f"shutdown_token_enabled={bool(SHUTDOWN_TOKEN)}")

    server = MiSideThreadingHTTPServer(
        ("127.0.0.1", port), RequestHandler
    )

    threading.Thread(
        target=parent_watchdog,
        args=(server,),
        name="MiSideParentWatchdog",
        daemon=True,
    ).start()

    log(f"SERVER_READY http://127.0.0.1:{port}")

    try:
        server.serve_forever(poll_interval=0.25)
    finally:
        SHUTDOWN_ONCE.set()
        server.server_close()
        log("SERVER_STOPPED")


def main() -> int:
    try:
        initialize_config()
        initialize_debug_file()
        run_server()
        return 0
    except Exception as exc:
        log(
            f"startup exception: {type(exc).__name__}: {exc}",
            error=True,
        )
        log(traceback.format_exc(), error=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
