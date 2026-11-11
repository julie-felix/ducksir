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
# 跑哪几个 bundle：wbc(mode3) / amp(mode4) / mjamp(mode5) 都已对齐；
# loco(mode2) 的 PD 细节还没对上（见 docs/notes/g1-botlab.md §16），先不纳入判据。
BUNDLES = ["g1-wbc", "g1-amp", "g1-mjamp"]
MJCF = os.path.join(ROOT, "build", "mjcf", "g1-29dof-torque.xml")

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


for bundle_name in BUNDLES:
    bundle = os.path.join(ROOT, "build", "release", bundle_name)
    if not (os.path.isdir(bundle) and os.path.exists(os.path.join(bundle, "contract.json"))):
        print(f"  跳过 {bundle_name}：没有 build/release/{bundle_name}")
        continue
    mjcf = os.path.join(bundle, "robot.xml")
    if not os.path.exists(mjcf):
        print(f"  跳过 {bundle_name}：没有 robot.xml")
        continue
    report = os.path.join("/tmp", f"{bundle_name}-report.json")
    cmd = [sys.executable, os.path.join(ROOT, "sim", "host", "run_bundle.py"),
           "--bundle", f"build/release/{bundle_name}", "--mjcf", mjcf,
           "--steps", "300", "--report", report]
    out = subprocess.run(cmd, capture_output=True, text=True, cwd=ROOT)
    if out.returncode != 0:
        print(out.stdout[-500:])
        print(out.stderr[-500:])
        ok(f"{bundle_name}：跑通", False)
        continue
    with open(report, encoding="utf-8") as f:
        rep = json.load(f)
    panel = rep.get("panel") or {}
    zmin = panel.get("base_height_min")
    upright = panel.get("upright_final")
    print(f"  {bundle_name}: 基座最低 {zmin:.3f}，直立 {upright:.3f}")
    ok(f"{bundle_name}：300 步跑完", rep.get("completed_steps", 0) == 300, str(rep.get("completed_steps")))
    ok(f"{bundle_name}：基座 ≥0.55 m（没倒）", zmin is not None and zmin > 0.55, f"zmin={zmin}")
    ok(f"{bundle_name}：末态直立 >0.9", upright is not None and upright > 0.9, f"upright={upright}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：G1 WBC bundle 端到端站稳（300 步）")
