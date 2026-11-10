#!/usr/bin/env python3
"""把 MicroDuck 的状态机编成一份「浏览器能独立跑」的图：`fsm.blob`。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/export_browser_fsm.py

为什么单独一份：浏览器里推理是异步的（ORT Web），而 C 核是同步的，所以图只能在**推理边界**切开。
状态机不吃策略输出，只吃「指令脉冲 + 姿态 + 速度指令」，所以它可以整块放进核里跑，
每步输出三样东西给 JS：

    输入 (4)   [policy_command 脉冲, 重力 z 分量, 前进指令 vx, 转向指令 yaw_rate]
    输出 (16)  [state, policy_mode, command(13), clear_last_action]

前进/转向范围照上游 `app/src/game/constants.js`：`VEL_FWD=0.25`、`VEL_BACK=-0.2`、`VEL_ANG=1.0`。
超出这个范围（比如硬塞 0.5）策略没见过——它会一边走一边拐，看起来就是「走得歪」。

`clear_last_action` 是 botlab 图里那个 `where(clear_last_action, 0, last_action)`：
在 SIT_ENTER 的第一步、WALK_RESET、RECOVER_ENTER 这三种时刻，观测里的 `last_action` 必须清零。
漏了它，坐完站起来时会把坐姿策略的上一帧动作喂给行走策略——第一帧动作直接顶飞，看起来就是「站起来就滚」。

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
    p = ir.Plan(input_dim=4, output_dim=16, name="microduck-fsm")

    cmd = p.reorder(p.obs(0, 1), [0])        # policy_command 脉冲
    gz = p.reorder(p.obs(1, 1), [0])         # 重力 z 分量
    vx = p.reorder(p.obs(2, 1), [0])         # 前进指令
    wz = p.reorder(p.obs(3, 1), [0])         # 转向指令（yaw rate）

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

    # [vx, vy=0, yaw_rate, 0×10]
    walk_cmd = p.concat(vx, p.constant([0.0]), wz, p.constant([0.0] * 10))
    sit_cmd = p.concat(p.constant([1.0]), zero12)
    phase = p.phase_clock(steps, F.GROUND_PICK_PERIOD_STEPS)
    gp_cmd = p.concat(phase, zero12)

    # 严格照 botlab 的 `_command()`：只有 WALK/WALK_RESET 给速度、**只有 SIT** 给 sit_command、
    # 只有 GROUND_PICK 给相位；其余状态（含 SIT_ENTER / SIT_EXIT）一律**零指令**。
    # 之前把 SIT_ENTER/SIT_EXIT 也给了 [1,0…]，结果「坐下成功、起身就滚」——(2026-11-10 踩过)
    cmd13 = p.add(
        select_by_state({0: walk_cmd, 6: walk_cmd, 10: walk_cmd}),
        select_by_state({2: sit_cmd}),
        select_by_state({9: gp_cmd}),
    )

    # clear_last_action = (state==SIT_ENTER & steps==0) | (state==WALK_RESET) | (state==RECOVER_ENTER)
    def eq_const(x, v):
        return p.compare(x, "eq", p.constant([float(v)]))

    sit_enter_first = p.compare(p.add(eq_const(state, F.SIT_ENTER), eq_const(steps, 0.0)),
                                "gte", p.constant([2.0]))
    is_walk_reset = eq_const(state, F.WALK_RESET)
    is_recover_enter = eq_const(state, F.RECOVER_ENTER)
    clear = p.compare(p.add(sit_enter_first, is_walk_reset, is_recover_enter),
                      "gte", p.constant([1.0]))

    p.out_action(p.concat(state, mode, cmd13, clear))
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
    print(f"写入 {out}/fsm.blob（{len(blob)} B，{len(doc['nodes'])} 节点，3 → 16）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
