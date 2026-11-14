#!/usr/bin/env python3
"""从 botlab 录制轨迹里切出「对拍样本」，供 CI 直接验证 obs 装配。

用法：
    python3 tools/dev/make_parity_sample.py --bundle g1-gmtrack --format gmtrack \
        --trace /tmp/gmtrack.jsonl --obs-dim 1728 --out third_party/parity/g1-gmtrack.json

样本只保留前 N 步（默认 3 步），每步存 qpos/qvel/last_action 和 **botlab 当时的 obs**。
测试端（sim/test/test_parity.py）拿同样的状态重算一遍，差超过 tol 就算失败。

两个容易搞错的点（都踩过）：

1. `last_action` 要存**这一步 obs 里用的那份**（= 上一步算出来的动作），不是这一步算出的动作；
2. hiking 的同一行里有两个模型（encoder 8×18×32=4608、actor 896），都叫 `input`，只能按维度挑。
"""
from __future__ import annotations

import argparse
import json
import os

DEFAULT_DIM = {"gmtrack": 1728, "php": 140, "parkour": 896}
JOINTS = 29


def _feed_values(feed):
    if isinstance(feed, list):
        return feed
    if isinstance(feed, dict):
        return feed.get("vals")
    return None


def _pick_feed(row, dim):
    ins = row.get("in") or {}
    for name in ("obs", "input"):
        values = _feed_values(ins.get(name))
        if values and len(values) == dim:
            return values
    return None


def _rows(path):
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                yield json.loads(line)


def sample(trace, steps, obs_dim):
    out = []
    prev_action = [0.0] * JOINTS
    for row in _rows(trace):
        values = _pick_feed(row, obs_dim)
        action = (row.get("out") or {}).get("actions")
        if not values or not row.get("qpos"):
            if action:
                prev_action = action
            continue
        out.append({
            "step": int(row["step"]),
            "qpos": row["qpos"],
            "qvel": row["qvel"],
            "last_action": prev_action,
            "action_after": action or prev_action,
            "obs": values,
        })
        if action:
            prev_action = action
        if len(out) >= steps:
            break
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle", required=True)
    ap.add_argument("--trace", required=True)
    ap.add_argument("--format", required=True, choices=sorted(DEFAULT_DIM))
    ap.add_argument("--steps", type=int, default=3)
    ap.add_argument("--obs-dim", type=int, default=0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    obs_dim = args.obs_dim or DEFAULT_DIM[args.format]
    steps = sample(args.trace, args.steps, obs_dim)
    if not steps:
        print(f"✗ {args.trace} 里没有可用的样本（缺 obs 或 qpos）")
        return 1
    doc = {"bundle": args.bundle, "format": args.format,
           "source": os.path.basename(args.trace), "steps": steps}
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False)
    print(f"✓ {args.out}：{len(steps)} 步，obs {len(steps[0]['obs'])} 维")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
