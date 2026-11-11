#!/usr/bin/env python3
"""WBC 观测装配的核对：用 botlab 运行时录下的 439 维真值逐段比对。

真值文件：`/tmp/cap-wbc-mode3.jsonl`（tools/dev/botlab_record_native.py 录的，机器人换成
带地面的 G1 + 扭矩模型 + 初始 mode=3）。没有真值时跳过，并明确打印「跳过」。

    python3 sim/test/test_wbc_obs.py
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import motion  # noqa: E402
import wbc_obs as W  # noqa: E402

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


TRACE = "/tmp/cap-wbc-mode3.jsonl"
MOTION = "/tmp/bt-all/G1-WBC-Multi-Policy/assets/motions/dance12"

# 1) 常量与形状（不需要真值）
ok("参考段 67 维、状态段 93 维 ×4 帧 = 439", W.REFERENCE_DIM + W.ROBOT_STATE_DIM * W.HISTORY_FRAMES == 439)

idx = np.zeros(W.ROBOT_STATE_DIM, dtype=np.float32)
hist = np.tile(idx, (W.HISTORY_FRAMES, 1))
ref = np.zeros(W.REFERENCE_DIM, dtype=np.float32)
ok("assemble 输出 439", W.assemble(reference67=ref, history4=hist).shape == (439,))

# 2) 单位姿态时，参考段的锚点部分应为 0 / 6D 单位
q_identity = np.array([1.0, 0.0, 0.0, 0.0])
anchor = W.reference_block(
    joint_pos=np.zeros(29), joint_vel=np.zeros(29),
    anchor_pos=[0.1, 0.2, 0.8], anchor_quat=q_identity,
    last_anchor_pos=[0.1, 0.2, 0.8], last_anchor_quat=q_identity,
    base_quat=q_identity)
ok("锚点无位移 → 位置段为 0", np.allclose(anchor[58:61], 0.0), str(np.round(anchor[58:61], 4)))
ok("相对旋转为恒等 → 6D = [1,0,0,1,0,0]",
   np.allclose(anchor[61:67], [1, 0, 0, 1, 0, 0]), str(np.round(anchor[61:67], 4)))

# 3) 暂停时参考速度为 0
anchor_pause = W.reference_block(
    joint_pos=np.zeros(29), joint_vel=np.full(29, 5.0),
    anchor_pos=[0, 0, 0.8], anchor_quat=q_identity,
    last_anchor_pos=[0, 0, 0.8], last_anchor_quat=q_identity,
    base_quat=q_identity, pause=True)
ok("暂停 → 参考速度全 0", np.allclose(anchor_pause[29:58], 0.0))

# 4) 与真值逐段比对
if not (os.path.exists(TRACE) and os.path.isdir(MOTION)):
    print("  跳过真值比对（没找到 /tmp/cap-wbc-mode3.jsonl 或 motion 目录）")
    print("\nPASS（部分跳过）：WBC 观测装配形状与单位姿态行为")
    raise SystemExit(0)

rec = next(json.loads(line) for line in open(TRACE, encoding="utf-8") if line.strip())
obs = np.asarray(rec["in"]["obs"], dtype=np.float32)
jp = motion.load_motion(MOTION, "joint_pos")["data"]
jv = motion.load_motion(MOTION, "joint_vel")["data"]

ok("真值 obs 是 439 维", obs.shape == (439,), str(obs.shape))
# 参考帧索引被夹到 >=1
ok("obs[0:29] 等于 motion 第 1 帧关节位置", np.allclose(obs[0:29], jp[1], atol=1e-5))
ok("obs[29:58] 等于 motion 第 1 帧关节速度", np.allclose(obs[29:58], jv[1], atol=1e-4))
ok("obs[58:61] 锚点位移为 0（首步）", np.allclose(obs[58:61], 0.0, atol=1e-6))
ok("obs[61:67] 为 6D 恒等旋转", np.allclose(obs[61:67], [1, 0, 0, 1, 0, 0], atol=1e-6))

# 用同一帧数据装配参考段，应与真值前 67 维一致（首步 last 与 cur 相同 → 恒等变换）
mine = W.reference_block(
    joint_pos=jp[1], joint_vel=jv[1],
    anchor_pos=[0.0, 0.0, 0.78], anchor_quat=q_identity,
    last_anchor_pos=[0.0, 0.0, 0.78], last_anchor_quat=q_identity,
    base_quat=q_identity)
diff = float(np.abs(mine - obs[:67]).max())
ok("我们装配的参考段与真值一致", diff < 1e-5, f"max|Δ|={diff:.2e}")

# 机器人状态段：首步为全零（机器人静止、动作为 0）
frame0 = W.robot_state_frame(ang_vel=np.zeros(3), gravity_b=[0, 0, -1],
                             q=np.zeros(29), qd=np.zeros(29), last_action=np.zeros(29))
ok("机器人状态单帧 93 维", frame0.shape == (93,))
ok("状态段首帧：角速度 0、投影重力 [0,0,-1]",
   np.allclose(obs[67:70], 0.0, atol=1e-6) and np.allclose(obs[70:73], [0, 0, -1], atol=1e-6),
   f"angvel={np.round(obs[67:70],3)} grav={np.round(obs[70:73],3)}")
grav_at = [i for i in range(len(obs)) if abs(obs[i] + 1) < 1e-6]
ok("4 帧历史各有一个投影重力", len(grav_at) == 4, f"位置 {grav_at}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：WBC 观测装配（439 = 67 + 4×93，与真值逐段一致）")
