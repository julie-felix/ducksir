#!/usr/bin/env python3
"""把 MicroDuck 的状态机编成一份「浏览器能独立跑」的图：`fsm.blob`。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_browser_fsm.py

为什么单独一份：浏览器里推理是异步的（ORT Web），而 C 核是同步的，所以图只能在**推理边界**切开。
状态机不吃策略输出，只吃「指令脉冲 + 姿态 + 速度指令」，所以它可以整块放进核里跑，
每步输出三样东西给 JS：

    输入 (3)   [policy_command 脉冲, 重力 z 分量, 前进指令 vx]
    输出 (15)  [state, policy_mode, command(13)]

`command(13)` 是喂给策略的指令向量，按状态分三种：行走用 [vx,0…]；坐/站用 [1,0…]；
地面拾取用 [phase,0…]（phase 来自 `phase_clock(machine.steps, 200)`）。这一层直接照 botlab 的
`_command()` 写。
"""

from __future__ import annotations

import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))

import ir  # noqa: E402

# 状态号与转移表：从 run_fsm.py 里复用（那份是逐条照抄 botlab 的）
sys.path.insert(0, HERE)
import run_fsm as F  # noqa: E402


def build() -> dict:
    p = ir.Plan(input_dim=3, output_dim=15, name="microduck-fsm")

    cmd = p.reorder(p.obs(0, 1), [0])        # policy_command 脉冲
    gz = p.reorder(p.obs(1, 1), [0])         # 重力 z 分量
    vx = p.reorder(p.obs(2, 1), [0])         # 前进指令

    tipped = p.compare(gz, "gt", p.constant([-0.3]))
    upright = p.compare(gz, "lt", p.constant([-0.85]))
    not_upright = p.compare(gz, "gte", p.constant([-0.85]))

    sm = p.state_machine(cmd, initial=F.WALK,
                         transitions=F.transitions(0, 1),
                         conditions=[tipped, upright, not_upright])
    state = p.reorder(sm, [0])
    steps = p.reorder(sm, [1])

    # ---- 状态 → 策略 mode（botlab 的 _policy_mode）----
    mode_by_state = {
        0: F.POLICY_WALK, 6: F.POLICY_WALK, 10: F.POLICY_WALK,
        1: F.POLICY_SITSTAND, 2: F.POLICY_SITSTAND, 3: F.POLICY_SITSTAND,
        4: F.POLICY_KICK_LEFT, 5: F.POLICY_KICK_RIGHT,
        7: F.POLICY_ROLL, 8: F.POLICY_ROLL,
        9: F.POLICY_GROUND_PICK,
        11: F.POLICY_STAND, 12: F.POLICY_STAND, 13: F.POLICY_STAND,
    }

    def select_by_state(mapping: dict):
        """按状态分块选常值向量；没命中的那几块输出全零，相加即得唯一一路。"""
        chunks = []
        items = list(mapping.items())
        for i in range(0, len(items), 3):
            part = items[i:i + 3]
            chunks.append(p.select(state, {float(k): v for k, v in part}, passive=-1))
        level = chunks
        while len(level) > 4:
            nxt = []
            for i in range(0, len(level), 4):
                c = level[i:i + 4]
                nxt.append(c[0] if len(c) == 1 else p.add(*c))
            level = nxt
        return level[0] if len(level) == 1 else p.add(*level)

    zero12 = p.constant([0.0] * 12)
    mode = select_by_state({k: p.constant([float(v)]) for k, v in mode_by_state.items()})

    walk_cmd = p.concat(vx, zero12)
    sit_cmd = p.concat(p.constant([1.0]), zero12)
    phase = p.phase_clock(steps, F.GROUND_PICK_PERIOD_STEPS)
    gp_cmd = p.concat(phase, zero12)

    cmd13 = p.add(
        select_by_state({0: walk_cmd, 6: walk_cmd, 10: walk_cmd}),
        select_by_state({1: sit_cmd, 2: sit_cmd, 3: sit_cmd}),
        select_by_state({9: gp_cmd}),
    )

    p.out_action(p.concat(state, mode, cmd13))
    return p.to_json()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="client/viewer/data/live")
    args = ap.parse_args()
    doc = build()
    blob = ir.compile_blob(doc)
    out = os.path.join(ROOT, args.out)
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "fsm.blob"), "wb") as f:
        f.write(blob)
    with open(os.path.join(out, "fsm.json"), "w", encoding="utf-8") as f:
        json.dump({"nodes": len(doc["nodes"]), "input_dim": doc["input_dim"],
                   "output_dim": doc["output_dim"], "plan_digest": ir.sha256_of_blob(blob),
                   "state_to_mode": {str(k): v for k, v in [
                       (0, F.POLICY_WALK), (1, F.POLICY_SITSTAND), (2, F.POLICY_SITSTAND),
                       (3, F.POLICY_SITSTAND), (4, F.POLICY_KICK_LEFT), (5, F.POLICY_KICK_RIGHT),
                       (6, F.POLICY_WALK), (7, F.POLICY_ROLL), (8, F.POLICY_ROLL),
                       (9, F.POLICY_GROUND_PICK), (10, F.POLICY_WALK),
                       (11, F.POLICY_STAND), (12, F.POLICY_STAND), (13, F.POLICY_STAND)]}},
                  f, ensure_ascii=False, indent=2)
    print(f"写入 {out}/fsm.blob（{len(blob)} B，{len(doc['nodes'])} 节点，3 → 15）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
