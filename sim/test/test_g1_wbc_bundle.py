#!/usr/bin/env python3
"""G1 WBC 的 bundle 端到端：跑 `build/release/g1-wbc`，判「站稳」而不是「没报错」。

判据（2026-11-12 实测值）：600 步内基座不低于 0.55 m、末态直立度 > 0.9。
缺 bundle 或模型时跳过（这些是本地构建产物，不进仓库）。

    python3 sim/test/test_g1_wbc_bundle.py
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
BUNDLE = os.path.join(ROOT, "build", "release", "g1-wbc")
MJCF = os.path.join(ROOT, "build", "mjcf", "g1-29dof-torque.xml")

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


if not (os.path.isdir(BUNDLE) and os.path.exists(os.path.join(BUNDLE, "contract.json"))):
    print("  跳过：没有 build/release/g1-wbc（先跑 export_artifacts，见 docs/notes/g1-botlab.md §12）")
    raise SystemExit(0)
if not os.path.exists(MJCF):
    print("  跳过：没有 build/mjcf/g1-29dof-torque.xml")
    raise SystemExit(0)

report = os.path.join("/tmp", "g1-wbc-bundle-report.json")
cmd = [sys.executable, os.path.join(ROOT, "sim", "host", "run_bundle.py"),
       "--bundle", BUNDLE, "--mjcf", MJCF, "--steps", "300", "--report", report]
out = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
if out.returncode != 0:
    print(out.stdout[-800:])
    print(out.stderr[-800:])
    raise SystemExit(1)

with open(report, encoding="utf-8") as f:
    rep = json.load(f)
print(out.stdout.strip().splitlines()[-3] if out.stdout.strip() else "")

panel = rep.get("panel") or {}
zmin = panel.get("base_height_min")
upright = panel.get("upright_final")
ok("300 步跑完没报错", rep.get("completed_steps", 0) == 300, f"steps={rep.get('completed_steps')}")
ok("基座最低高度 > 0.55 m（没倒）", zmin is not None and zmin > 0.55, f"zmin={zmin}")
ok("末态直立度 > 0.9", upright is not None and upright > 0.9, f"upright={upright}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：G1 WBC bundle 端到端站稳（300 步）")
