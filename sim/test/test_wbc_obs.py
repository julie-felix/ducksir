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

# 5) 播放头：推进 / 夹取 / 暂停
ph = W.Playhead(6574, start=0, frame_interval=1)
ok("播放头起始夹到 1", ph.current() == (1, 1))
ph.advance(); ok("推进一步 → (2,1)", ph.current() == (2, 1), str(ph.current()))
ph.pause(True)
before = ph.current()
ph.advance()
ok("暂停时不推进", ph.current() == before, str(ph.current()))
ok("暂停时 idx == last_idx", ph.current()[0] == ph.current()[1])
ph.pause(False)
for _ in range(10):
    ph.advance()
ok("推进到第 12 帧", ph.current()[0] == 12, str(ph.current()))

ph2 = W.Playhead(20, start=100, frame_interval=5)
ok("越界起点被夹到 end", ph2.current()[0] == 19, str(ph2.current()))
# frame_interval 只管锚点回看，不管推进步长（真值反推出来的）
ph3 = W.Playhead(20, start=2, frame_interval=5)
ph3.advance()
ok("推进一步只 +1 → (3,1)（回看 5 帧被夹到 1）", ph3.current() == (3, 1), str(ph3.current()))
ph4 = W.Playhead(20, start=10, frame_interval=5)
ok("回看：idx=10 → (10,5)", ph4.current() == (10, 5), str(ph4.current()))
ph5 = W.Playhead(20, start=3, frame_interval=5)
ok("回看被夹：idx=3 → (3,1)", ph5.current() == (3, 1), str(ph5.current()))

# 5b) 锚点回看距离：从真值反解出来的是 frame_interval=5（推 1 帧、回看 5 帧）
if os.path.exists(TRACE) and os.path.isdir(MOTION):
    meta = json.load(open(os.path.join(MOTION, "metadata.json"), encoding="utf-8"))
    names = meta["body_names"]
    bp = motion.load_motion(MOTION, "body_pos_w")["data"].reshape(-1, len(names), 3)
    bq = motion.load_motion(MOTION, "body_quat_w")["data"].reshape(-1, len(names), 4)
    jp_all = motion.load_motion(MOTION, "joint_pos")["data"]
    jv_all = motion.load_motion(MOTION, "joint_vel")["data"]
    line = [json.loads(x) for x in open(TRACE, encoding="utf-8") if x.strip()][10]
    obs10 = np.asarray(line["in"]["obs"], dtype=np.float32)
    base_q = np.asarray(line["quat"], dtype=np.float64)
    idx10, last10 = 11, 6      # 第 10 步：推 1 帧、回看 5
    ref10 = W.reference_block(joint_pos=jp_all[idx10], joint_vel=jv_all[idx10],
                              anchor_pos=bp[idx10][0], anchor_quat=bq[idx10][0],
                              last_anchor_pos=bp[last10][0], last_anchor_quat=bq[last10][0],
                              base_quat=base_q)
    d10 = float(np.abs(ref10 - obs10[:67]).max())
    ok("第 10 步：推 1 帧 + 回看 5 帧 → 参考段对齐", d10 < 1e-4, f"max|Δ|={d10:.2e}")

    # 反例：用「上一帧」当锚点（回看 1 帧）会明显偏
    ref1 = W.reference_block(joint_pos=jp_all[idx10], joint_vel=jv_all[idx10],
                             anchor_pos=bp[idx10][0], anchor_quat=bq[idx10][0],
                             last_anchor_pos=bp[idx10 - 1][0], last_anchor_quat=bq[idx10 - 1][0],
                             base_quat=base_q)
    d1 = float(np.abs(ref1 - obs10[:67]).max())
    ok("反例：回看 1 帧明显不符（证明确实是 5）", d1 > 5 * d10, f"max|Δ|={d1:.2e}")

# 6) 真值里的播放头：逐步匹配 motion 帧号
if os.path.exists(TRACE):
    jp_all = motion.load_motion(MOTION, "joint_pos")["data"]
    seq = []
    lines = [x for x in open(TRACE, encoding="utf-8") if x.strip()][:12]
    for line in lines:
        o = np.asarray(json.loads(line)["in"]["obs"][:29], dtype=np.float32)
        seq.append(int(np.abs(jp_all - o).max(axis=1).argmin()))
    ok("真值播放头 = 1,2,3…（每步 1 帧）", seq == list(range(1, 13)), str(seq))

# 7) 一步完整的 439 维观测：assemble_observation
if os.path.exists(TRACE) and os.path.isdir(MOTION):
    meta = json.load(open(os.path.join(MOTION, "metadata.json"), encoding="utf-8"))
    names = meta["body_names"]
    bundle_arrays = {
        "joint_pos": motion.load_motion(MOTION, "joint_pos")["data"],
        "joint_vel": motion.load_motion(MOTION, "joint_vel")["data"],
        "body_pos": motion.load_motion(MOTION, "body_pos_w")["data"].reshape(-1, len(names), 3),
        "body_quat": motion.load_motion(MOTION, "body_quat_w")["data"].reshape(-1, len(names), 4),
    }
    line = [json.loads(x) for x in open(TRACE, encoding="utf-8") if x.strip()][10]
    obs10 = np.asarray(line["in"]["obs"], dtype=np.float32)
    hist = obs10[67:].reshape(4, 93)     # 直接用真值里的状态历史
    ph = W.Playhead(bundle_arrays["joint_pos"].shape[0], start=11, frame_interval=5)
    full = W.assemble_observation(motion=bundle_arrays, playhead=ph,
                                  robot_state_history=hist,
                                  base_quat=np.asarray(line["quat"], dtype=np.float64))
    d_full = float(np.abs(full - obs10).max())
    ok("完整 439 观测（含状态历史）与真值一致", d_full < 1e-4, f"max|Δ|={d_full:.2e}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：WBC 观测装配（439 = 67 + 4×93，与真值逐段一致）")
