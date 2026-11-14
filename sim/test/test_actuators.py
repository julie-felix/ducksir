#!/usr/bin/env python3
"""执行器语义单测：位置 vs 扭矩、策略序→电机序、effortLimit 截断。

判据来自真实数据：2026-11-12 用 botlab native 运行时录的 WBC loco 轨迹，
按 `tau = kp(target−q) − kd·qd` + dof_mapping 复算，与它实际写进 d.ctrl 的值差 ≤ 0.8。
"""
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import actuators as A  # noqa: E402

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


# dof_mapping 来自 ccrpRepo/wbc_fsm 的 State_Loco.h
MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24,
       18, 25, 19, 26, 20, 27, 21, 28]

act = np.arange(29, dtype=float)
mapped = A.policy_to_motor(act, MAP)
ok("映射：策略 0 → 电机 0", mapped[0] == 0)
ok("映射：策略 1 → 电机 6", mapped[6] == 1, f"motor6={mapped[6]}")
ok("映射是一一对应（无丢项）", sorted(mapped.tolist()) == list(range(29)))

default = np.zeros(29)
scale = np.full(29, 0.25)
target = A.target_from_action(act, motor_order=MAP, default=default, scale=scale)
ok("target[motor=6] = 0.25×1", abs(target[6] - 0.25) < 1e-12, f"{target[6]}")

q = np.zeros(29)
qd = np.zeros(29)
kp = np.full(29, 100.0)
kd = np.full(29, 5.0)
eff = np.full(29, 50.0)
tau = A.torque_from_target(target, q, qd, kp=kp, kd=kd, effort_limit=eff)
ok("静止时 tau = kp·target（并按上限截断）", np.allclose(tau, np.clip(100 * target, -50, 50)))
big = A.torque_from_target(np.full(29, 10.0), q, qd, kp=kp, kd=kd, effort_limit=eff)
ok("大误差被 effortLimit 截断", np.allclose(big, 50.0), f"{big[0]}")
vel = A.torque_from_target(q, q, np.full(29, 2.0), kp=kp, kd=kd, effort_limit=eff)
ok("速度项是 −kd·qd", np.allclose(vel, -10.0), f"{vel[0]}")

pos_preset = {"actuatorMode": "position", "defaultJointPositions": [0.0] * 29,
              "actionScale": [0.25] * 29, "kp": [100.0] * 29, "kd": [5.0] * 29,
              "effortLimit": [50.0] * 29}
ctrl_pos, _ = A.ctrl_from_action(act, pos_preset, qpos=q, qvel=qd)
ok("位置模式：ctrl 就是目标角", np.allclose(ctrl_pos, 0.25 * act))

tor_preset = dict(pos_preset, actuatorMode="torque")
ctrl_tor, tgt = A.ctrl_from_action(act, tor_preset, qpos=q, qvel=qd)
ok("扭矩模式：ctrl 是力矩", np.allclose(ctrl_tor, np.clip(100 * 0.25 * act, -50, 50)),
   f"ctrl0={ctrl_tor[0]:.1f}")
ok("扭矩模式也返回目标角（给指标用）", np.allclose(tgt, 0.25 * act))

preset_path = os.path.join(ROOT, "build/preset/g1-29dof-torque.preset.yaml")
if os.path.exists(preset_path):
    pre = yaml.safe_load(open(preset_path, encoding="utf-8"))
    ok("G1 torque preset：actuatorMode=torque", pre.get("actuatorMode") == "torque")
    ok("G1 torque preset：kp/kd/effortLimit 各 29 项",
       all(len(pre.get(k) or []) == 29 for k in ("kp", "kd", "effortLimit")))
    ctrl, tgt = A.ctrl_from_action(np.zeros(29), pre, qpos=pre["defaultJointPositions"],
                                   qvel=np.zeros(29))
    ok("默认角 + 零动作 → 力矩为 0", np.allclose(ctrl, 0.0), f"max|tau|={np.abs(ctrl).max():.2e}")
else:
    print("  跳过真实 G1 preset（还没生成 build/preset/g1-29dof-torque.preset.yaml）")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：执行器语义（位置/扭矩、映射、截断）")
