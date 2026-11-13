#!/usr/bin/env python3
"""ducksir 的统一后端（零依赖）：静态页 + 直播 SSE + 作业 + 文件树 + 诊断。

设计见 `docs/adr/001-unified-python-backend.md`。核心只有三条：

1. **一个进程**：页面、SSE、长任务全在这里；node 插件宿主只做同源反代；
2. **长任务 = 子进程 + 作业记录**：每个作业一行 JSON（状态/退出码/产物），日志逐行转 SSE；
3. **CLI 是真路径**：HTTP 只是把命令行包起来，AI 可以用 curl 和 CLI 互相对照。

    python3 sim/host/serve.py --port 8899          # 起服务
    curl -H "X-Ducksir-Secret: $(cat ~/.local/share/ducksir/serve.secret)" \
         http://127.0.0.1:8899/api/bundles
"""
from __future__ import annotations

import argparse
import json
import os
import secrets
import shutil
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlencode, urlparse

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SECRET_PATH = os.path.join(os.path.expanduser("~"), ".local/share/ducksir/serve.secret")
JOBS_DIR = os.path.join(ROOT, "runs", "jobs")
READ_ROOTS = ("build", "runs")            # 文件树白名单：只读这两个根

PYLIBS = os.path.join(os.path.expanduser("~"), ".local/share/ducksir/pylibs")


def _py_path() -> str:
    parts = [PYLIBS, ROOT, os.path.join(ROOT, "sim", "host"), os.environ.get("PYTHONPATH", "")]
    return os.pathsep.join(p for p in parts if p)


def _json(code: int, payload: dict) -> tuple[int, bytes, str]:
    return code, json.dumps(payload, ensure_ascii=False).encode(), "application/json; charset=utf-8"


def _err(code: int, error_code: str, message: str, **detail) -> tuple[int, bytes, str]:
    return _json(code, {"error": {"code": error_code, "message": message, "detail": detail}})


def list_bundles() -> list[dict]:
    """列出 build/release/* 里带 contract.json 的目录。`base` 必填（判据 4）。"""
    base_dir = os.path.join(ROOT, "build", "release")
    out = []
    if not os.path.isdir(base_dir):
        return out
    for name in sorted(os.listdir(base_dir)):
        dirpath = os.path.join(base_dir, name)
        if not os.path.isfile(os.path.join(dirpath, "contract.json")):
            continue
        out.append({
            "name": name,
            "base": f"/ducksir-static/build/release/{name}/",
            "dir": f"build/release/{name}",
        })
    return out


class Job:
    """一个子进程作业：状态、日志、退出码。"""

    def __init__(self, op: str, argv: list[str], env: dict | None = None):
        self.id = uuid.uuid4().hex[:12]
        self.op = op
        self.argv = argv
        self.status = "queued"
        self.started_at = None
        self.ended_at = None
        self.exit_code = None
        self.error = None
        self.lines: list[str] = []          # 重连时补发（只留尾部）
        self._lock = threading.Lock()
        self._subs: set = set()
        self._proc: subprocess.Popen | None = None
        self._thread: threading.Thread | None = None
        self._env = env
        self.metrics: dict = {}

    # ---- 生命周期 ----
    def start(self):
        self.status = "running"
        self.started_at = time.time()
        env = dict(os.environ)
        env["PYTHONPATH"] = _py_path()
        env.setdefault("MUJOCO_GL", "egl")
        env["PYTHONUNBUFFERED"] = "1"
        if self._env:
            env.update(self._env)
        try:
            self._proc = subprocess.Popen(
                self.argv, cwd=ROOT, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1, errors="replace")
        except Exception as exc:                       # 起不来也要有作业记录
            self.status = "failed"
            self.error = f"{type(exc).__name__}: {exc}"
            self.ended_at = time.time()
            return
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def _pump(self):
        assert self._proc and self._proc.stdout
        for line in self._proc.stdout:
            line = line.rstrip("\n")
            self._update_metrics(line)
            with self._lock:
                self.lines.append(line)
                if len(self.lines) > 4000:
                    self.lines = self.lines[-2000:]
                subs = list(self._subs)
            for fn in subs:
                try:
                    fn(line)
                except Exception:
                    pass
        code = self._proc.wait()
        self.exit_code = code
        self.status = "done" if code == 0 else "failed"
        if code != 0:
            self.error = f"退出码 {code}"
        self.ended_at = time.time()
        self._save()

    def _update_metrics(self, line: str):
        """从一帧 pose 里算业务指标：基座高度、直立度（判据 5：可自动断言）。"""
        if not line.startswith('{"type":"pose"'):
            return
        try:
            msg = json.loads(line)
            xpos, xquat = msg["xpos"], msg["xquat"]
            idx = 1 if len(xpos) >= 6 else 0            # body 1 = pelvis（world 之后）
            z = float(xpos[idx * 3 + 2])
            qw, qx, qy, _qz = xquat[idx * 4: idx * 4 + 4]
            upright = 1.0 - 2.0 * (qx * qx + qy * qy)
            self.metrics = {"t": msg.get("t"), "step": msg.get("step"),
                            "z": round(z, 4), "upright": round(upright, 4)}
        except (KeyError, ValueError, IndexError):
            pass

    def cancel(self):
        if self._proc and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self.status = "cancelled"
            self.ended_at = time.time()
            self._save()
            return True
        return False

    def subscribe(self, fn):
        with self._lock:
            self._subs.add(fn)
            tail = list(self.lines[-400:])
        for line in tail:
            try:
                fn(line)
            except Exception:
                pass

    def unsubscribe(self, fn):
        with self._lock:
            self._subs.discard(fn)

    # ---- 对外快照 ----
    def snapshot(self) -> dict:
        return {
            "id": self.id, "op": self.op, "status": self.status,
            "startedAt": self.started_at, "endedAt": self.ended_at,
            "exitCode": self.exit_code, "error": self.error,
            "argv": self.argv, "metrics": self.metrics,
        }

    def _save(self):
        os.makedirs(JOBS_DIR, exist_ok=True)
        path = os.path.join(JOBS_DIR, f"{self.id}.json")
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.snapshot(), f, ensure_ascii=False, indent=1)
        except OSError:
            pass


class Registry:
    """单并发队列：一次只跑一个作业（训练/仿真抢资源，慢会伪装成 bug）。"""

    def __init__(self):
        self.jobs: dict[str, Job] = {}
        self.queue: list[Job] = []
        self.current: Job | None = None
        self._lock = threading.Lock()

    def submit(self, job: Job) -> Job:
        with self._lock:
            self.jobs[job.id] = job
            if self.current is None:
                self.current = job
                job.start()
            else:
                self.queue.append(job)
        return job

    def finish_current(self):
        with self._lock:
            self.current = None
            while self.queue:
                nxt = self.queue.pop(0)
                if nxt.status == "queued":
                    self.current = nxt
                    nxt.start()
                    return

    def cancel(self, job_id: str) -> bool:
        job = self.jobs.get(job_id)
        if not job:
            return False
        if job.status == "queued":
            with self._lock:
                if job in self.queue:
                    self.queue.remove(job)
            job.status = "cancelled"
            return True
        return job.cancel()

    def snapshot(self) -> list[dict]:
        return [j.snapshot() for j in self.jobs.values()]


REGISTRY = Registry()
PROJECT: dict = {}          # 当前工程仿真的显示端口（studio 的兼容层）


def _watch_finish():
    """轮询当前作业是否结束（子进程读完即结束），结束就放行队列。"""
    while True:
        time.sleep(0.3)
        job = REGISTRY.current
        if job and job.status in ("done", "failed", "cancelled"):
            REGISTRY.finish_current()


def _free_port() -> int:
    import socket
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def _proxy_local(req, res, port: int, rest: str):
    """把请求原样转给 127.0.0.1:port（SSE 直接 pipe）。"""
    import http.client
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=30)
    try:
        conn.request(req.command, rest, headers={k: v for k, v in req.headers.items()
                                                  if k.lower() in ("accept", "content-type")})
        up = conn.getresponse()
        res.send_response(up.status)
        for k, v in up.getheaders():
            if k.lower() in ("content-length", "transfer-encoding", "connection"):
                continue
            res.send_header(k, v)
        res.end_headers()
        # 逐块转发：SSE 不能一次 read() 到底，那样会挂到超时
        while True:
            chunk = up.read(1024)
            if not chunk:
                break
            res.wfile.write(chunk)
            res.wfile.flush()
    except Exception as exc:
        res.send_response(504)
        res.send_header("content-type", "application/json; charset=utf-8")
        body = json.dumps({"error": {"code": "display_down", "message": str(exc)}}).encode()
        res.send_header("content-length", str(len(body)))
        res.end_headers()
        res.wfile.write(body)
    finally:
        conn.close()


def make_op(op: str, args: dict) -> tuple[list[str], dict]:
    """把作业请求翻译成命令行（CLI 是唯一真路径）。"""
    python = sys.executable or "python3"
    if op == "bundle_live":
        bundle = (args or {}).get("bundle")
        if not bundle:
            raise ValueError("缺 bundle")
        raw = str(bundle).strip()
        # 只收三种形状，不做「猜」（ADR-001 判据 4）：
        #   名字         g1-loco
        #   base         /ducksir-static/build/release/g1-loco/
        #   绝对/相对目录 build/release/g1-loco
        if raw.startswith("/ducksir-static/"):
            raw = raw[len("/ducksir-static/"):]
        d = raw if os.path.isabs(raw) else os.path.join(ROOT, raw)
        if not os.path.isfile(os.path.join(d, "contract.json")):
            if "/" in raw:
                raise ValueError(f"bundle 路径不存在或没有 contract.json：{raw}")
            d = os.path.join(ROOT, "build", "release", raw)
        name = os.path.basename(d.rstrip("/"))
        if not os.path.isfile(os.path.join(d, "contract.json")):
            raise ValueError(f"没有这个 bundle：{name}")
        seconds = int((args or {}).get("seconds") or 120)
        argv = [python, os.path.join(ROOT, "sim/host/run_bundle.py"),
                "--bundle", f"build/release/{name}",
                "--mjcf", f"build/release/{name}/robot.xml",
                "--seconds", str(seconds), "--stdio", "--pace"]
        return argv, {"bundle": name, "seconds": seconds}
    if op == "sim":
        project = (args or {}).get("project")
        if not project:
            raise ValueError("缺 project")
        seconds = int((args or {}).get("seconds") or 60)
        if not os.path.isfile(os.path.join(ROOT, str(project))):
            raise ValueError(f"没有这个 project：{project}")
        port = _free_port()
        argv = [python, os.path.join(ROOT, "sim/host/run_project.py"),
                "--project", str(project), "--seconds", str(seconds),
                "--port", str(port), "--no-open"]
        return argv, {"project": str(project), "seconds": seconds, "displayPort": port}
    if op == "train":
        # 训练方案还没定（docs/index.md 末尾），先把位置留出来：501 比将来改协议便宜
        raise NotImplementedError("训练方案未定：training/ 还不存在")
    raise ValueError(f"不认识的 op：{op}")


class Handler(BaseHTTPRequestHandler):
    server_version = "ducksir-serve/0.1"
    SECRET = ""

    # ---- 工具 ----
    def log_message(self, fmt, *args):
        sys.stderr.write("[serve] " + (fmt % args) + "\n")

    def _send(self, result):
        code, body, ctype = result
        self.send_response(code)
        self.send_header("content-type", ctype)
        self.send_header("content-length", str(len(body)))
        self.send_header("cache-control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _authorized(self) -> bool:
        if not self.SECRET:
            return True
        return self.headers.get("X-Ducksir-Secret") == self.SECRET

    def _read_json(self):
        n = int(self.headers.get("content-length") or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            return {}

    def do_GET(self):
        url = urlparse(self.path)
        path, qs = url.path, parse_qs(url.query)
        if path == "/api/health":
            return self._send(_json(200, {
                "ok": True, "version": "0.1", "secretRequired": bool(self.SECRET),
                "jobs": len(REGISTRY.jobs), "running": (REGISTRY.current.op if REGISTRY.current else None),
            }))
        if not self._authorized():
            return self._send(_err(401, "unauthorized", "缺 X-Ducksir-Secret 或不对"))
        if path.startswith("/api/project/"):
            return self._project(path[len("/api/project/"):], qs)
        if path == "/api/bundles":
            return self._send(_json(200, {"bundles": list_bundles()}))
        if path == "/api/jobs":
            return self._send(_json(200, {"jobs": REGISTRY.snapshot()}))
        if path.startswith("/api/jobs/"):
            rest = path[len("/api/jobs/"):]
            job_id, _, tail = rest.partition("/")
            job = REGISTRY.jobs.get(job_id)
            if not job:
                return self._send(_err(404, "no_such_job", "没有这个作业", id=job_id))
            if tail == "stream":
                return self._stream(job)
            return self._send(_json(200, job.snapshot()))
        if path == "/files":
            return self._files(qs.get("path", [""])[0])
        if path.startswith("/ducksir-static/"):
            return self._static(path[len("/ducksir-static/"):])
        return self._send(_err(404, "no_route", "没有这个路由", path=path))

    def do_POST(self):
        global PROJECT
        url = urlparse(self.path)
        path = url.path
        if not self._authorized():
            return self._send(_err(401, "unauthorized", "缺 X-Ducksir-Secret 或不对"))
        if path == "/api/jobs":
            body = self._read_json()
            op = body.get("op")
            try:
                argv, meta = make_op(op, body.get("args") or {})
            except NotImplementedError as exc:
                return self._send(_err(501, "not_implemented", str(exc), op=op))
            except ValueError as exc:
                return self._send(_err(400, "bad_request", str(exc), op=op))
            job = Job(op, argv)
            if op == "bundle_live":
                cur = REGISTRY.current
                if cur and cur.op == "bundle_live" and cur.status == "running":
                    cur.cancel()                      # 切台：旧的直接停，别排队
                    REGISTRY.finish_current()
            if op == "sim":
                PROJECT = {"job": job, "port": meta.get("displayPort"),
                           "project": meta.get("project")}
            REGISTRY.submit(job)
            return self._send(_json(200, {"id": job.id, "status": job.status, "meta": meta}))
        if path.startswith("/api/jobs/") and path.endswith("/cancel"):
            job_id = path[len("/api/jobs/"):-len("/cancel")]
            return self._send(_json(200, {"ok": REGISTRY.cancel(job_id)}))
        return self._send(_err(404, "no_route", "没有这个路由", path=path))

    # ---- 工程仿真的兼容层（studio 用；等它改成 /api/jobs 再删）----
    def _project(self, action: str, qs):
        global PROJECT
        if action == "status":
            job = PROJECT.get("job")
            if job and job.status in ("done", "failed", "cancelled"):
                PROJECT = {}
                job = None
            return self._send(_json(200, {
                "running": bool(job and job.status == "running"),
                "port": PROJECT.get("port"), "project": PROJECT.get("project"),
                "error": job.error if job else "", "job": job.id if job else None,
            }))
        if action == "start":
            return self._send(_err(405, "use_api_jobs", "请用 POST /api/jobs {op:'sim'}"))
        if action == "scene" or action == "stream" or action == "files" or action == "file":
            port = PROJECT.get("port")
            if not port:
                return self._send(_err(503, "no_project", "没有在跑的工程仿真",
                                       hint="POST /api/jobs {op:'sim',args:{project}}"))
            rest = "/" + action + (("?" + urlencode(qs, doseq=True)) if qs else "")
            return _proxy_local(self, self, port, rest)
        return self._send(_err(404, "no_route", "没有这个路由", path="/api/project/" + action))

    # ---- SSE ----
    def _stream(self, job: Job):
        self.send_response(200)
        self.send_header("content-type", "text/event-stream; charset=utf-8")
        self.send_header("cache-control", "no-cache")
        self.send_header("connection", "keep-alive")
        self.end_headers()
        self.wfile.write(b": connected\n\n")
        self.wfile.flush()

        def emit(line: str):
            try:
                self.wfile.write(b"data: " + json.dumps({"line": line}, ensure_ascii=False).encode() + b"\n\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                raise

        done = threading.Event()

        def watch():
            while not done.is_set():
                if job.status in ("done", "failed", "cancelled"):
                    try:
                        emit(json.dumps({"type": "end", "status": job.status,
                                         "exitCode": job.exit_code}))
                    except Exception:
                        pass
                    done.set()
                    return
                time.sleep(0.2)

        job.subscribe(emit)
        threading.Thread(target=watch, daemon=True).start()
        try:
            done.wait(timeout=3600)
        except Exception:
            pass
        finally:
            job.unsubscribe(emit)

    # ---- 静态文件（页面/网格/模型；只读）----
    def _static(self, rel: str):
        rel = rel.split("?")[0].strip("/")
        roots = [os.path.join(ROOT, "client", "viewer"), ROOT]
        for root in roots:
            full = os.path.realpath(os.path.join(root, rel))
            if not (full == os.path.realpath(root) or full.startswith(os.path.realpath(root) + os.sep)):
                continue
            if os.path.isfile(full):
                ctype = {
                    ".html": "text/html; charset=utf-8",
                    ".js": "text/javascript; charset=utf-8",
                    ".mjs": "text/javascript; charset=utf-8",
                    ".json": "application/json",
                    ".css": "text/css; charset=utf-8",
                    ".wasm": "application/wasm",
                    ".png": "image/png",
                }.get(os.path.splitext(full)[1].lower(), "application/octet-stream")
                with open(full, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("content-type", ctype)
                self.send_header("content-length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
        return self._send(_err(404, "no_file", "静态文件不存在", path=rel))

    # ---- 文件树（只读白名单）----
    def _files(self, rel: str):
        rel = rel.strip("/")
        if not rel:
            return self._send(_err(400, "bad_request", "缺 path"))
        full = os.path.realpath(os.path.join(ROOT, rel))
        allowed = [os.path.realpath(os.path.join(ROOT, r)) for r in READ_ROOTS]
        if not any(full == a or full.startswith(a + os.sep) for a in allowed):
            return self._send(_err(403, "outside_roots", "只允许读 build/ 与 runs/", path=rel))
        if os.path.isdir(full):
            names = []
            for n in sorted(os.listdir(full))[:500]:
                p = os.path.join(full, n)
                names.append({"name": n, "dir": os.path.isdir(p),
                              "size": os.path.getsize(p) if os.path.isfile(p) else None})
            return self._send(_json(200, {"dir": rel, "entries": names}))
        if not os.path.isfile(full):
            return self._send(_err(404, "no_file", "没有这个文件", path=rel))
        with open(full, "rb") as f:
            data = f.read()
        self.send_response(200)
        self.send_header("content-type", "application/octet-stream")
        self.send_header("content-length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main() -> int:
    ap = argparse.ArgumentParser(description="ducksir 统一后端")
    ap.add_argument("--port", type=int, default=8899)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--no-secret", action="store_true", help="不校验 secret（只在本机调试时用）")
    ap.add_argument("--static", action="store_true", help="同时把 client/viewer 当静态目录服务")
    args = ap.parse_args()

    if args.no_secret:
        Handler.SECRET = ""
    else:
        os.makedirs(os.path.dirname(SECRET_PATH), exist_ok=True)
        if os.path.exists(SECRET_PATH):
            Handler.SECRET = open(SECRET_PATH, encoding="utf-8").read().strip()
        if not Handler.SECRET:
            Handler.SECRET = secrets.token_urlsafe(24)
            with open(SECRET_PATH, "w", encoding="utf-8") as f:
                f.write(Handler.SECRET)
        print(f"secret: {SECRET_PATH}")

    threading.Thread(target=_watch_finish, daemon=True).start()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"ducksir serve → http://{args.host}:{args.port}/api/health")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
