#!/usr/bin/env python3
"""项目脚本跑通 + 中断语义。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_project_run.py

验三件事：
1. `ducksir run project.py` 那条链能跑完，报告带 panel 八项；
2. Ctrl-C（SIGINT）时**先落盘**、报告记 `interrupted: true`、退出码 130；
3. 项目脚本写错（没 policy / 频率不是整数倍）要**早报错**，别跑到一半。
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PY = sys.executable
fails = 0


def ok(label, cond, detail=""):
    global fails
    print(f"  {'PASS' if cond else 'FAIL'}  {label}{'  ' + str(detail) if detail else ''}")
    if not cond:
        fails += 1


GOOD = """
import sys
sys.path.insert(0, "sim/sdk")
from ducksir import Project

project = (Project("duck-hd1910")
           .policy("walking", mode=0)
           .command(forward=0.25)
           .rates(control_hz=50, physics_hz=200))
"""

BAD_RATE = """
import sys
sys.path.insert(0, "sim/sdk")
from ducksir import Project
project = Project().policy("walking").rates(control_hz=30, physics_hz=200)
"""

BAD_EMPTY = """
import sys
sys.path.insert(0, "sim/sdk")
from ducksir import Project
project = Project()
"""


def write(tmpdir, name, body):
    path = os.path.join(tmpdir, name)
    with open(path, "w", encoding="utf-8") as f:
        f.write(body)
    return path


def run(args, timeout=60):
    return subprocess.run([PY, "sim/host/run_project.py", *args], cwd=ROOT,
                          capture_output=True, text=True, timeout=timeout)


with tempfile.TemporaryDirectory() as tmp:
    good = write(tmp, "good.py", GOOD)
    print("== 正常跑完")
    rep = os.path.join(tmp, "rep.json")
    r = run(["--project", good, "--seconds", "1", "--no-browser", "--report", rep])
    ok("退出码 0", r.returncode == 0, r.returncode)
    ok("报告生成", os.path.exists(rep))
    if os.path.exists(rep):
        doc = json.load(open(rep, encoding="utf-8"))
        ok("报告带 panel 八项", len(doc.get("panel") or {}) == 8, sorted(doc.get("panel") or {}))
        ok("interrupted=False", doc.get("interrupted") is False)
        ok("completed_steps 正确（1s×50Hz）", doc.get("completed_steps") == 50, doc.get("completed_steps"))
        ok("verdict.pass", doc.get("verdict", {}).get("pass") is True)

    print("== Ctrl-C 中断：先落盘、记 interrupted、退出码 130")
    rep2 = os.path.join(tmp, "rep_int.json")
    proc = subprocess.Popen([PY, "sim/host/run_project.py", "--project", good,
                             "--seconds", "10", "--no-browser", "--pace", "realtime",
                             "--report", rep2],
                            cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.8)   # 实时节拍下 10 s 的跑活到 0.8 s 只走了一小半
    proc.send_signal(signal.SIGINT)
    code = proc.wait(timeout=20)
    ok("退出码 130", code == 130, code)
    ok("中断也写了报告", os.path.exists(rep2))
    if os.path.exists(rep2):
        doc = json.load(open(rep2, encoding="utf-8"))
        ok("报告记 interrupted=True", doc.get("interrupted") is True)
        ok("已完成步数 < 总步数", 0 < doc.get("completed_steps", 0) < doc.get("steps", 0),
           f"{doc.get('completed_steps')}/{doc.get('steps')}")

    print("== 写错要早报错")
    r = run(["--project", write(tmp, "bad_rate.py", BAD_RATE), "--seconds", "1", "--no-browser"])
    ok("频率不成整数倍 → 非 0", r.returncode != 0, r.returncode)
    ok("错误信息说清原因", "整数倍" in (r.stdout + r.stderr), (r.stdout + r.stderr).strip()[:80])
    ok("不是裸 traceback", "Traceback" not in (r.stdout + r.stderr), (r.stdout + r.stderr).strip()[:60])
    r = run(["--project", write(tmp, "bad_empty.py", BAD_EMPTY), "--seconds", "1", "--no-browser"])
    ok("没 policy → 非 0", r.returncode != 0, r.returncode)

print()
print("全部通过" if fails == 0 else f"失败 {fails} 项")
sys.exit(0 if fails == 0 else 1)
