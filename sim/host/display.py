#!/usr/bin/env python3
"""实时跟随的显示服务：把每步的 body 位姿推给浏览器。

    srv = Display(port=8765, scene={"bodies": [...], "links": [...]}, open_path=None)
    srv.start()          # 后台线程起 HTTP + SSE
    srv.push(t, xpos, xquat)   # 每控制步一次
    srv.stop()

**为什么用 SSE 而不是 botlab 的 WebSocket**：botlab 用 WS 推 `[time, *qpos, *qvel]`（float32 二进制）。
我们这边没有 websockets 依赖，而 SSE 用标准库就能写对（一次 HTTP 响应、逐行 `data: {...}`），
浏览器 `EventSource` 原生支持。代价是 JSON 文本略宽一点——50 Hz × 16 个 body 完全够用。
形状对齐 botlab：「CPython 算物理，浏览器只当只读渲染客户端」。

只绑 127.0.0.1（和 botlab 一样）：这不是给人远程看的服务，只是本地回显。
"""

from __future__ import annotations

import json
import mimetypes
import os
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

LOOPBACK = ("127.0.0.1", "localhost")


class Display:
    def __init__(self, port: int = 8765, scene: dict | None = None, host: str = "127.0.0.1",
                 root: str | None = None, files_root: str | None = None,
                 files: list[str] | None = None, command: dict | None = None):
        if host not in LOOPBACK:
            raise ValueError(f"display 只允许绑本机（现在 {host}）")
        self.host = host
        self.port = int(port)
        self.scene = scene or {}
        self.files_root = files_root or ""
        self.files = list(files or [])          # 允许只读展示的相对路径
        self._command = dict(command or {})     # 实时指令（滑杆写这里）
        self._stats = {}                        # runner 每步塞进来的小指标
        self.root = root or os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), "client", "viewer")
        self._subs: list[queue.Queue] = []
        self._lock = threading.Lock()
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.frames_sent = 0

    # ── 生命周期 ─────────────────────────────────────────────
    def start(self) -> "Display":
        display = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args):  # 安静
                pass

            def _serve_file(self, rel, ctype=None, base=None):
                base = base or display.root
                target = os.path.normpath(os.path.join(base, rel))
                if not target.startswith(os.path.abspath(base)):
                    self.send_response(403)
                    self.send_header("content-length", "0")
                    self.end_headers()
                    return
                if not os.path.isfile(target):
                    self.send_response(404)
                    self.send_header("content-length", "0")
                    self.end_headers()
                    return
                if ctype is None:
                    ctype = mimetypes.guess_type(target)[0] or "application/octet-stream"
                    if target.endswith(".wasm"):
                        ctype = "application/wasm"
                    if target.endswith(".mjs") or target.endswith(".js"):
                        ctype = "text/javascript; charset=utf-8"
                data = open(target, "rb").read()
                self.send_response(200)
                self.send_header("content-type", ctype)
                self.send_header("content-length", str(len(data)))
                self.send_header("cache-control", "no-store")
                self.end_headers()
                if self.command != "HEAD":
                    self.wfile.write(data)

            def do_GET(self):  # noqa: N802
                if self.path.startswith("/scene"):
                    body = json.dumps(display.scene, ensure_ascii=False).encode()
                    self.send_response(200)
                    self.send_header("content-type", "application/json; charset=utf-8")
                    self.send_header("access-control-allow-origin", "*")
                    self.send_header("content-length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path in ("/", "/studio", "/studio.html"):
                    self._serve_file("studio.html", "text/html; charset=utf-8")
                    return
                if self.path in ("/follow", "/follow.html"):
                    self._serve_file("follow.html", "text/html; charset=utf-8")
                    return
                # 直开模式（页面在根路径）需要的别名：/vendor /data /assets 与几个脚本。
                # 顺序要紧：先 assets，再 vendor/data，最后 /static/ 兜底。
                if self.path.startswith("/assets/"):
                    rel = self.path[len("/assets/"):].split("?")[0]
                    repo_assets = os.path.abspath(os.path.join(display.root, "..", "..", "assets"))
                    self._serve_file(rel, None, base=repo_assets)
                    return
                if self.path.startswith("/static/assets/"):
                    rel = self.path[len("/static/assets/"):].split("?")[0]
                    repo_assets = os.path.abspath(os.path.join(display.root, "..", "..", "assets"))
                    self._serve_file(rel, None, base=repo_assets)
                    return
                if self.path.startswith("/vendor/") or self.path.startswith("/data/"):
                    self._serve_file(self.path.lstrip("/").split("?")[0])
                    return
                if self.path.startswith("/static/"):
                    rel = self.path[len("/static/"):].split("?")[0]
                    self._serve_file(rel)
                    return
                if self.path in ("/studio.js", "/follow.js", "/panel-meta.mjs", "/assets-map.mjs",
                                 "/live-core.mjs", "/play.js", "/play-core.mjs", "/inspect.js"):
                    self._serve_file(self.path.lstrip("/"))
                    return
                if self.path.startswith("/files"):
                    body = json.dumps({"root": os.path.basename(display.files_root),
                                       "files": display.files}, ensure_ascii=False).encode()
                    self.send_response(200)
                    self.send_header("content-type", "application/json; charset=utf-8")
                    self.send_header("content-length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path.startswith("/file"):
                    from urllib.parse import parse_qs, urlsplit
                    rel = (parse_qs(urlsplit(self.path).query).get("path") or [""])[0]
                    if rel not in display.files:
                        self.send_response(403)
                        self.send_header("content-length", "0")
                        self.end_headers()
                        return
                    self._serve_file(rel, None, base=display.files_root)
                    return
                if self.path.startswith("/command"):
                    from urllib.parse import parse_qs, urlsplit
                    q = parse_qs(urlsplit(self.path).query)
                    if q:
                        got = {k: float(v[0]) for k, v in q.items() if k in ("forward", "yaw")}
                        display.set_command(**got)
                    body = json.dumps(display.get_command(), ensure_ascii=False).encode()
                    self.send_response(200)
                    self.send_header("content-type", "application/json; charset=utf-8")
                    self.send_header("access-control-allow-origin", "*")
                    self.send_header("content-length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path.startswith("/stream"):
                    self.send_response(200)
                    self.send_header("content-type", "text/event-stream; charset=utf-8")
                    self.send_header("cache-control", "no-store")
                    self.send_header("access-control-allow-origin", "*")
                    self.end_headers()
                    q: queue.Queue = queue.Queue(maxsize=8)
                    with display._lock:
                        display._subs.append(q)
                    try:
                        self.wfile.write(b": connected\n\n")
                        self.wfile.flush()
                        while not display._stop.is_set():
                            try:
                                frame = q.get(timeout=0.5)
                            except queue.Empty:
                                self.wfile.write(b": keepalive\n\n")
                                self.wfile.flush()
                                continue
                            payload = "data: " + json.dumps(frame, separators=(",", ":")) + "\n\n"
                            self.wfile.write(payload.encode())
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    finally:
                        with display._lock:
                            if q in display._subs:
                                display._subs.remove(q)
                    return
                self.send_response(404)
                self.send_header("content-length", "0")
                self.end_headers()

        self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    # ── 实时指令（滑杆 → runner）────────────────────────────
    def set_command(self, **kw) -> None:
        with self._lock:
            for k in ("forward", "yaw"):
                if k in kw:
                    self._command[k] = float(kw[k])

    def get_command(self) -> dict:
        with self._lock:
            return dict(self._command)

    def set_stats(self, stats: dict) -> None:
        with self._lock:
            self._stats = dict(stats)

    @property
    def url(self) -> str:
        return f"http://{self.host}:{self.port}/stream"

    # ── 推帧 ────────────────────────────────────────────────
    def push(self, t: float, xpos, xquat) -> None:
        """`xpos`/`xquat` 是 MuJoCo 的整段数组（每 body 3 / 4 个 float），二维也接受。"""
        if hasattr(xpos, "reshape") and getattr(xpos, "ndim", 1) > 1:
            xpos = xpos.reshape(-1)
        if hasattr(xquat, "reshape") and getattr(xquat, "ndim", 1) > 1:
            xquat = xquat.reshape(-1)
        with self._lock:
            stats = dict(self._stats)
        frame = {
            "t": round(float(t), 4),
            "stats": stats,
            "pos": [round(float(v), 5) for v in xpos],
            "quat": [round(float(v), 5) for v in xquat],
        }
        with self._lock:
            subs = list(self._subs)
        for q in subs:
            try:
                q.put_nowait(frame)
                self.frames_sent += 1
            except queue.Full:
                try:
                    q.get_nowait()
                    q.put_nowait(frame)
                except queue.Empty:
                    pass


def wait_ready(url_base: str, timeout: float = 3.0) -> bool:
    """等 HTTP 服务起来（给自动开浏览器前用）。"""
    import urllib.request
    end = time.time() + timeout
    while time.time() < end:
        try:
            with urllib.request.urlopen(url_base + "/scene", timeout=0.5):
                return True
        except Exception:  # noqa: BLE001
            time.sleep(0.05)
    return False
