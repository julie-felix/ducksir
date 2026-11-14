#!/usr/bin/env python3
"""为没有 botlab 录制源的 bundle 生成「金标轨迹」：锁住我们自己的行为。

为什么需要：WBC / AMP / MJAMP / Instinct / UniSuperMimic 这几个模板的 botlab 侧
录制受 SDK 版本限制（模板用了 0.4.1 没有的 `commands(mappings=...)`，或依赖
autoActuators 生成执行器），暂时拿不到对拍样本。先用**金标轨迹**防回归：
同样的初始状态 + 同样的指令，跑 60 步，任何一格 qpos/obs 变了都报警。

用法：
    python3 tools/dev/make_golden.py --bundle g1-wbc --steps 60
"""
from __future__ import annotations

import argparse
import gzip
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
GOLDEN_DIR = os.path.join(ROOT, "third_party", "golden")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle", required=True)
    ap.add_argument("--steps", type=int, default=60)
    ap.add_argument("--forward", type=float, default=None)
    args = ap.parse_args()

    seconds = args.steps / 50.0
    trace = f"/tmp/golden-{args.bundle}.jsonl"
    cmd = [sys.executable, os.path.join(ROOT, "sim", "host", "run_bundle.py"),
           "--bundle", os.path.join(ROOT, "build", "release", args.bundle),
           "--mjcf", os.path.join(ROOT, "build", "release", args.bundle, "robot.xml"),
           "--seconds", str(seconds), "--trace", trace]
    if args.forward is not None:
        cmd += ["--forward", str(args.forward)]
    env = dict(os.environ)
    env.setdefault("MUJOCO_GL", "egl")
    proc = subprocess.run(cmd, cwd=ROOT, env=env, capture_output=True, text=True, timeout=600)
    if proc.returncode != 0:
        print("✗ 跑不起来：" + ((proc.stderr or proc.stdout).strip().splitlines() or [""])[-1])
        return 1

    rows = []
    with open(trace, encoding="utf-8") as f:
        for line in f:
            row = json.loads(line)
            rows.append({"step": row["step"], "qpos": row["qpos"],
                         "action": [round(float(v), 6) for v in row["action"]]})
    rows = rows[:args.steps]
    if not rows:
        print("✗ 没产生轨迹")
        return 1

    os.makedirs(GOLDEN_DIR, exist_ok=True)
    out = os.path.join(GOLDEN_DIR, f"{args.bundle}.json.gz")
    with gzip.open(out, "wt", encoding="utf-8") as f:
        json.dump({"bundle": args.bundle, "steps": rows}, f)
    print(f"✓ {os.path.relpath(out, ROOT)}：{len(rows)} 步")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
