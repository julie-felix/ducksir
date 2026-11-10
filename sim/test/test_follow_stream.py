#!/usr/bin/env python3
"""工作台/跟随页的 headless 验证：服务起来了、端点对了、SSE 有帧且形状对。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_follow_stream.py

浏览器里渲染成什么样只能人眼看，但「服务通不通、帧有没有、参数写进去读得回」这些都能在这里验掉。
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import display as DSP  # noqa: E402

PORT = int(os.environ.get("DUCKSIR_DISPLAY_PORT", "8807"))
PROJECT = os.environ.get("DUCKSIR_TEST_PROJECT", "sim/test/fixtures/project_demo.py")
BASE = f"http://127.0.0.1:{PORT}"
fails = 0


def ok(label, cond, detail=""):
    global fails
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'  ' + str(detail) if detail else ''}")
    if not cond:
        fails += 1


def get(path, timeout=4):
    return urllib.request.urlopen(BASE + path, timeout=timeout).read().decode()


env = dict(os.environ)
child = subprocess.Popen([sys.executable, "sim/host/run_project.py", "--project", PROJECT,
                          "--seconds", "60", "--port", str(PORT), "--no-open"],
                         cwd=ROOT, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    ok("显示服务就绪", DSP.wait_ready(BASE, timeout=15))

    ok("/follow 能拿到页面", "实时跟随" in get("/follow"))
    ok("/studio 能拿到工作台页", "工作台" in get("/studio"))
    ok("/static 能拿到脚本", "EventSource" in get("/static/studio.js"))

    files = json.loads(get("/files"))
    listed = files.get("files", [])
    ok("/files 列出项目文件", any(f.endswith(".py") for f in listed), listed[:4])

    py = next(f for f in listed if f.endswith(".py"))
    body = get("/file?path=" + py)
    ok("/file 能读内容", "Project" in body or "import" in body, body[:36].replace("\n", " "))
    try:
        get("/file?path=../../etc/passwd")
        ok("越界路径要拒绝", False)
    except urllib.error.HTTPError as e:
        ok("越界路径要拒绝", e.code in (403, 404), e.code)

    # 先订阅，再等帧
    frames = []

    stop = threading.Event()

    def reader():
        req = urllib.request.urlopen(BASE + "/stream", timeout=30)
        for line in req:
            if stop.is_set():
                break
            s = line.decode().strip()
            if s.startswith("data: "):
                frames.append(json.loads(s[6:]))

    threading.Thread(target=reader, daemon=True).start()
    deadline = time.time() + 15
    while len(frames) < 20 and time.time() < deadline:
        time.sleep(0.1)

    ok("收到 ≥5 帧", len(frames) >= 5, len(frames))
    if frames:
        pos, quat = frames[-1]["pos"], frames[-1]["quat"]
        ok("pos 是 3 的倍数", len(pos) % 3 == 0, len(pos))
        ok("quat 是 4 的倍数", len(quat) % 4 == 0, len(quat))
        ok("时间戳单调不减", all(frames[i]["t"] >= frames[i - 1]["t"] for i in range(1, len(frames))))
        stats = frames[-1].get("stats") or {}
        ok("帧里带 stats", all(k in stats for k in ("base_height", "upright", "step")), sorted(stats))

    cmd = json.loads(get("/command?forward=0.12&yaw=0.3"))
    ok("/command 写进去能读回", abs(cmd.get("forward", 0) - 0.12) < 1e-6 and abs(cmd.get("yaw", 0) - 0.3) < 1e-6, cmd)
    time.sleep(0.6)   # 让 runner 多跑几步，采纳新的滑杆值
    if frames:
        live = (frames[-1].get("stats") or {})
        ok("runner 真的采纳了滑杆值", abs(live.get("forward", -9) - 0.12) < 0.051, live.get("forward"))
finally:
    stop.set()
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        child.kill()

print()
print("全部通过" if fails == 0 else f"失败 {fails} 项")
raise SystemExit(0 if fails == 0 else 1)
