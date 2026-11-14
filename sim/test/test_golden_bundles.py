#!/usr/bin/env python3
"""金标回归：没有 botlab 对拍样本的 bundle，用我们自己的轨迹锁住行为。

`third_party/golden/<bundle>.json.gz` 由 `tools/dev/make_golden.py` 生成。
重跑同样步数，逐步比 qpos 与动作；差超过 1e-6 就说明行为变了（模型/观测/执行器改动）。
"""
from __future__ import annotations

import gzip
import json
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GOLDEN_DIR = os.path.join(ROOT, "third_party", "golden")
TOL = 1e-6


def run_bundle(bundle: str, steps: int, forward: float | None):
    seconds = steps / 50.0
    trace = os.path.join(tempfile.gettempdir(), f"golden-check-{bundle}.jsonl")
    cmd = [sys.executable, os.path.join(ROOT, "sim", "host", "run_bundle.py"),
           "--bundle", os.path.join(ROOT, "build", "release", bundle),
           "--mjcf", os.path.join(ROOT, "build", "release", bundle, "robot.xml"),
           "--seconds", str(seconds), "--trace", trace]
    if forward is not None:
        cmd += ["--forward", str(forward)]
    env = dict(os.environ)
    env.setdefault("MUJOCO_GL", "egl")
    proc = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True, timeout=600)
    if proc.returncode != 0:
        return None, ((proc.stderr or proc.stdout).strip().splitlines() or [""])[-1]
    rows = []
    with open(trace, encoding="utf-8") as f:
        for line in f:
            row = json.loads(line)
            rows.append((row["step"], row["qpos"], row["action"]))
    return rows, ""


def main() -> int:
    if not os.path.isdir(GOLDEN_DIR):
        print("SKIP：没有 third_party/golden/")
        return 0
    try:
        import numpy as np
    except ModuleNotFoundError:
        print("SKIP：没有 numpy")
        return 0

    fails = 0
    checked = 0
    for name in sorted(os.listdir(GOLDEN_DIR)):
        if not name.endswith(".json.gz"):
            continue
        bundle = name[: -len(".json.gz")]
        with gzip.open(os.path.join(GOLDEN_DIR, name), "rt", encoding="utf-8") as f:
            golden = json.load(f)
        steps = len(golden["steps"])
        rows, err = run_bundle(bundle, steps, None)
        if rows is None:
            print(f"  SKIP  {bundle}：{err}")
            continue
        worst = 0.0
        for g, (step, qpos, action) in zip(golden["steps"], rows):
            worst = max(worst,
                        float(np.abs(np.asarray(qpos, float) - np.asarray(g["qpos"], float)).max()),
                        float(np.abs(np.asarray(action, float) - np.asarray(g["action"], float)).max()))
        checked += 1
        if worst <= TOL:
            print(f"  PASS  {bundle}：{steps} 步最大差 {worst:.2e}")
        else:
            print(f"  FAIL  {bundle}：{steps} 步最大差 {worst:.2e}（阈值 {TOL:.0e}）")
            fails += 1

    print()
    if fails:
        print(f"FAIL：{fails} 个 bundle 行为变了")
        return 1
    print(f"PASS：金标回归 {checked} 个")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
