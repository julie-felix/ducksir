#!/usr/bin/env python3
"""实时跟随的 headless 验证：runner 起显示服务 → 订阅 SSE → 形状对。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_follow_stream.py

为什么要测：浏览器能不能看到画面，取决于「SSE 有没有帧、形状对不对」。
这两个我们能在没有浏览器的地方验掉；剩下的（three.js 渲染）才是只能人眼看的部分。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import display as DSP  # noqa: E402

PORT = int(os.environ.get("DUCKSIR_DISPLAY_PORT", "8807"))
PROJECT = os.environ.get("DUCKSIR_TEST_PROJECT", "sim/test/fixtures/project_demo.py")
fails = 0


def ok(label, cond, detail=""):
    global fails
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'  ' + str(detail) if detail else ''}")
    if not cond:
        fails += 1


env = dict(os.environ)
env["PYTHONPATH"] = os.environ.get("PYTHONPATH", "")
child = subprocess.Popen([sys.executable, "sim/host/run_project.py", "--project", PROJECT,
                          "--seconds", "1.5", "--port", str(PORT), "--no-open", "--pace", "realtime"],
                         cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
ready = DSP.wait_ready(f"http://127.0.0.1:{PORT}", timeout=10)
ok("显示服务就绪", ready)
if not ready:
    child.kill()
    print(child.stdout.read()[-600:])
    raise SystemExit(1)

html = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/follow", timeout=3).read().decode()
ok("/follow 能拿到页面", "实时跟随" in html, f"{len(html)} B")

static = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/static/follow.js", timeout=3).read().decode()
ok("/static 能拿到脚本", "EventSource" in static)

frames = []


def reader():
    req = urllib.request.urlopen(f"http://127.0.0.1:{PORT}/stream", timeout=12)
    for line in req:
        s = line.decode().strip()
        if s.startswith("data: "):
            frames.append(json.loads(s[6:]))
            if len(frames) >= 12:
                break


threading.Thread(target=reader, daemon=True).start()
try:
    child.wait(timeout=25)
except subprocess.TimeoutExpired:
    child.kill()

ok("收到 ≥5 帧", len(frames) >= 5, len(frames))
if frames:
    pos, quat = frames[0]["pos"], frames[0]["quat"]
    ok("pos 长度是 3 的倍数（每 body 三个）", len(pos) % 3 == 0, len(pos))
    ok("quat 长度是 4 的倍数", len(quat) % 4 == 0, len(quat))
    ok("时间戳单调不减", all(frames[i]["t"] >= frames[i - 1]["t"] for i in range(1, len(frames))))

print()
print("全部通过" if fails == 0 else f"失败 {fails} 项")
raise SystemExit(0 if fails == 0 else 1)
