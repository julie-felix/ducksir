#!/usr/bin/env python3
"""GMTrack Stage-II 的观测装配（1728 维）。

规格来自模板 `Unitree-G1-GMTrack-Stage-II-Local/controller.py`：

```
proprio (640) = history_by_term([
    projected_gravity,                  # 3
    body_angular_velocity(pelvis, local),  # 3（cvel 推出，比 qvel 慢一个物理子步）
    joint_position − default,           # 29
    joint_velocity,                     # 29
], 10)                                  # 每项 10 帧，按 term 平铺（最老在前）
action_hist (290) = last_action × 10（同样 by-term）
command_window (798) = 21 个参考 token，帧偏移 −10…+10；
    token = [R_refᵀ·v_ref(3), R_refᵀ·w_ref(3), R_refᵀ·g(3), q_ref(29)]
合计 1728
```

动作（模板里不是 default+scale，而是**参考姿态** + scale）：

```
target = q_ref(t + 0.02) + action_scale × action    # 再按 motor_order 重排
```

实测对齐（`/tmp/gmtrack.jsonl`，botlab 自己录的）：

- 命令窗中心 token 在控制步 K 对应动作第 **K** 帧（偏移 ±10 帧，越界钳到 0/末尾）；
- q / qd / action 段逐位一致；gravity 段一致；angvel 段用的是 cvel（不是 qvel）。
"""
from __future__ import annotations

import numpy as np

WINDOW_RADIUS = 10
HISTORY = 10
JOINT_COUNT = 29
TOKEN_DIM = 3 + 3 + 3 + JOINT_COUNT      # 38
PROPRIO_DIM = 3 + 3 + JOINT_COUNT + JOINT_COUNT  # 64
OBS_DIM = PROPRIO_DIM * HISTORY + JOINT_COUNT * HISTORY + TOKEN_DIM * (2 * WINDOW_RADIUS + 1)
CONTROL_DT = 0.02
GRAVITY_W = np.array([0.0, 0.0, -1.0])


def _qmat(q):
    w, x, y, z = (float(v) for v in q)
    n = w * w + x * x + y * y + z * z
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ], dtype=np.float64)


def inverse_rotate(vec, quat):
    """把世界系向量转到 quat 表示的 body 系：Rᵀ·v。"""
    return _qmat(quat).T @ np.asarray(vec, dtype=np.float64)


def sample(motion: dict, t: float, channel: str, *, body: int = 0, nearest: bool = True):
    fps = float(motion.get("fps", 50))
    frames = motion[channel].shape[0]
    idx = int(np.floor(t * fps + 0.5)) if nearest else int(t * fps)
    idx = min(max(idx, 0), frames - 1)
    row = np.asarray(motion[channel][idx], dtype=np.float64)
    if row.ndim >= 2:
        row = row[body]
    return row


def reference_token(motion: dict, t: float, joint_order=None) -> np.ndarray:
    q = sample(motion, t, "body_quat_w")
    v = sample(motion, t, "body_lin_vel_w")
    w = sample(motion, t, "body_ang_vel_w")
    qj = sample(motion, t, "joint_pos")
    if joint_order is not None:
        qj = qj[np.asarray(joint_order)]
    return np.concatenate([inverse_rotate(v, q), inverse_rotate(w, q),
                           inverse_rotate(GRAVITY_W, q), qj])


def command_window(motion: dict, t_center: float, joint_order=None) -> np.ndarray:
    toks = [reference_token(motion, t_center + off * CONTROL_DT, joint_order)
            for off in range(-WINDOW_RADIUS, WINDOW_RADIUS + 1)]
    return np.concatenate(toks)


class TermHistory:
    """按 term 平铺的 10 帧历史（最老在前）；重置时用第一帧填满（mjlab 语义）。"""

    def __init__(self, dims=(3, 3, JOINT_COUNT, JOINT_COUNT), frames: int = HISTORY):
        self.frames = frames
        self.dims = dims
        self.buf: list[np.ndarray | None] = [None] * len(dims)

    def push(self, values) -> np.ndarray:
        out = []
        for i, v in enumerate(values):
            v = np.asarray(v, dtype=np.float64)
            if self.buf[i] is None:
                self.buf[i] = np.tile(v, (self.frames, 1))
            else:
                self.buf[i] = np.roll(self.buf[i], -1, axis=0)
                self.buf[i][-1] = v
            out.append(self.buf[i].reshape(-1))
        return np.concatenate(out)

    def reset(self):
        self.buf = [None] * len(self.dims)


class ActionHistory:
    """动作的 10 帧历史（最老在前）。"""

    def __init__(self, dim: int = JOINT_COUNT, frames: int = HISTORY):
        self.frames = frames
        self.buf = np.zeros((frames, dim), dtype=np.float64)

    def push(self, action) -> np.ndarray:
        self.buf = np.roll(self.buf, -1, axis=0)
        self.buf[-1] = np.asarray(action, dtype=np.float64)
        return self.buf.reshape(-1)

    def reset(self):
        self.buf[:] = 0.0


def assemble(*, motion, t, joint_order, default_angles, joint_pos, joint_vel,
             gravity_b, base_ang_vel_local, last_action, proprio_hist, action_hist) -> np.ndarray:
    """一步的 1728 维观测。`t` 是**参考中心时刻**（= 控制步 × 0.02）。"""
    q = np.asarray(joint_pos, dtype=np.float64) - np.asarray(default_angles, dtype=np.float64)
    qd = np.asarray(joint_vel, dtype=np.float64)
    if joint_order is not None:
        q = q[np.asarray(joint_order)]
        qd = qd[np.asarray(joint_order)]
    proprio = proprio_hist.push([gravity_b, base_ang_vel_local, q, qd])
    ahist = action_hist.push(last_action)
    window = command_window(motion, t, joint_order)
    return np.concatenate([proprio, ahist, window]).astype(np.float32)
