#!/usr/bin/env python3
"""G1 WBC（动作跟踪）端到端冒烟：439 观测 + 参考动作播放 + 扭矩 PD。

复刻 botlab `g1-wbc` 模板的 mode 3（WBC）：

    obs(439) = 参考 67（关节位置/速度 + 锚点 6D） + 机器人状态 4×93
    target   = referenceJointPositions + 0.25 × action[motor_order]     ← 注意不是 default_angles
    ctrl     = kp·(target − q) − kd·qd ，按 effortLimit 截断，**每个物理子步重算**

常量来源（都在 botlab 的 preset 里，已由 tools/robot/import_botlab_preset.py 转进我们的 preset）：

| 项 | 值 |
|---|---|
| 动作参考 | `assets/motions/dance12`（.bin 容器，fps 50） |
| 播放头 | 每控制步 +1 帧，锚点回看 5 帧（`frame_interval=5`），进 mode 前暂停 |
| 参考目标 | `referenceJointPositions`（不是 defaultJointPositions） |
| 关节增益 | `kp/kd`（mode 3 用 robot.kp/kd，不是 balance 那组） |

    python3 tools/dev/g1_wbc_smoke.py
"""
import json
import os
import sys

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
import actuators as A  # noqa: E402
import motion as M  # noqa: E402
import wbc_obs as W  # noqa: E402

MJCF = os.path.join(ROOT, "build/mjcf/g1-29dof-torque.xml")
PRESET = os.path.join(ROOT, "build/preset/g1-29dof-torque.preset.yaml")
MOTION_DIR = "/tmp/bt-all/G1-WBC-Multi-Policy/assets/motions/dance12"
MODEL = "/tmp/bt-all/G1-WBC-Multi-Policy/assets/models/dance12_0207_1.onnx"
REF_ANGLES = [-0.312, 0, 0, 0.669, -0.363, 0, -0.312, 0, 0, 0.669, -0.363, 0,
              0, 0, 0, 0.2, 0.2, 0, 0.6, 0, 0, 0, 0.2, -0.2, 0, 0.6, 0, 0, 0]
MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24,
       18, 25, 19, 26, 20, 27, 21, 28]


def load_motion_arrays(path):
    meta = json.load(open(os.path.join(path, "metadata.json"), encoding="utf-8"))
    names = meta["body_names"]
    return {
        "joint_pos": M.load_motion(path, "joint_pos")["data"],
        "joint_vel": M.load_motion(path, "joint_vel")["data"],
        "body_pos": M.load_motion(path, "body_pos_w", indices=names)["data"].reshape(-1, len(names), 3),
        "body_quat": M.load_motion(path, "body_quat_w", indices=names)["data"].reshape(-1, len(names), 4),
    }, meta


def main() -> int:
    if not (os.path.exists(MODEL) and os.path.isdir(MOTION_DIR)):
        print("跳过：没找到 dance12_0207_1.onnx 或动作目录（先解压 g1-wbc 模板包）")
        return 0
    pre = yaml.safe_load(open(PRESET, encoding="utf-8"))
    arrays, meta = load_motion_arrays(MOTION_DIR)

    spec = mujoco.MjSpec.from_file(MJCF)
    plane = spec.worldbody.add_geom()
    plane.type = mujoco.mjtGeom.mjGEOM_PLANE
    plane.size = [6, 6, 0.1]
    plane.condim = 3
    m = spec.compile()
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / pre.get("physicsHz", 200)
    d.qpos[:] = pre["initialQpos"]
    mujoco.mj_forward(m, d)

    sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    inputs = [i.name for i in sess.get_inputs()]
    outputs = [o.name for o in sess.get_outputs()]
    n_in = int(sess.get_inputs()[0].shape[1]) if sess.get_inputs()[0].shape[1] else 439
    print(f"模型输入 {inputs}（第一维 {n_in}），输出 {outputs}")

    playhead = W.Playhead(arrays["joint_pos"].shape[0], start=0, frame_interval=5)
    playhead.pause(True)          # 进 WBC 前参考停在起点（botlab: active_mode=3, pause=350）
    hist = np.zeros((W.HISTORY_FRAMES, W.ROBOT_STATE_DIM), dtype=np.float32)
    last_action = np.zeros(29, dtype=np.float32)
    DQ = np.asarray(pre["defaultJointPositions"], np.float64)
    REF = np.asarray(REF_ANGLES, np.float64)

    def quat_to_mat(q):
        w, x, y, z = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    zs, tracked = [], []
    steps = int(os.environ.get("WBC_STEPS", "200"))
    unpause = int(os.environ.get("WBC_UNPAUSE", "0"))
    for step in range(steps):
        if unpause and step == unpause:
            playhead.pause(False)   # 进 WBC：参考从头开始播放
        q = d.qpos[7:36]
        qd = d.qvel[6:35]
        R = quat_to_mat(d.qpos[3:7])
        grav = R.T @ np.array([0, 0, -1.0])
        hist = np.roll(hist, -1, axis=0)
        hist[-1] = W.robot_state_frame(
            ang_vel=d.qvel[3:6], gravity_b=grav,
            q=np.array([q[MAP[i]] - DQ[MAP[i]] for i in range(29)]),
            qd=np.array([qd[MAP[i]] for i in range(29)]),
            last_action=last_action)
        obs = W.assemble_observation(motion=arrays, playhead=playhead,
                                     robot_state_history=hist, base_quat=d.qpos[3:7])
        feeds = {inputs[0]: obs.reshape(1, -1).astype(np.float32)}
        if len(inputs) > 1:
            feeds[inputs[1]] = np.zeros((1, 1, 256), np.float32)
        if len(inputs) > 2:
            feeds[inputs[2]] = np.zeros((1, 1, 256), np.float32)
        res = sess.run(outputs, feeds)
        act = np.clip(np.asarray(res[0]).reshape(-1), -100, 100)
        last_action = act.astype(np.float32)
        playhead.advance()

        target = REF.copy()
        for i in range(29):
            target[MAP[i]] = REF[MAP[i]] + 0.25 * act[i]
        for _ in range(4):
            ctrl, _ = A.ctrl_from_action(act, pre, qpos=d.qpos[7:36], qvel=d.qvel[6:35])
            # ctrl_from_action 用的是 default；WBC 的目标基准是 referenceJointPositions
            ctrl = A.torque_from_target(target, d.qpos[7:36], d.qvel[6:35],
                                        kp=pre["kp"], kd=pre["kd"],
                                        effort_limit=pre["effortLimit"])
            d.ctrl[:] = ctrl
            mujoco.mj_step(m, d)
        zs.append(float(d.qpos[2]))
        # 跟踪误差：机器人关节角 vs 动作参考当前帧（电机序）
        idx, _ = playhead.current()
        ref_motor = np.array([arrays["joint_pos"][idx][i] for i in range(29)])
        robot_motor = np.array([d.qpos[7 + MAP[i]] for i in range(29)])
        tracked.append(float(np.abs(ref_motor - robot_motor).mean()))

    print(f"z 起 {pre['initialQpos'][2]:.3f} → {zs[-1]:.3f}（最低 {min(zs):.3f}），"
          f"直立 {1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2):.3f}，共 {len(zs)} 步")
    print(f"跟踪误差：前 20 步均值 {np.mean(tracked[:20]):.3f} rad，后 20 步均值 {np.mean(tracked[-20:]):.3f} rad")
    ok = len(zs) >= 60
    print(("PASS" if ok else "FAIL") + "：WBC 439 观测 + 参考目标 + 扭矩 PD（判据：≥60 步不倒）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
