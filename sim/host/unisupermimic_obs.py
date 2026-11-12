#!/usr/bin/env python3
"""UniSuperMimic 的观测装配（1720 维）：99 维本体 × 10 帧历史 + 10 × 73 维未来目标。

规格来源：模板 `g1-unisupermimic` 的 `controller.py`（`observations()`），并逐项对齐。

```
obs99 = q−default(29) + qd(29) + base_ang_vel(3) + projected_gravity(3)
        + last_action(29) + rotation_6d(relative_quaternion(r0, qpos_quat))(6)
goal73(k) = relative_position(pelvis_pos(t+k·0.02), p0, heading)(3)
          + rotation_6d(relative_quaternion(q(t+k·0.02), heading))(6)
          + rotate(lin_vel(t+k·0.02), inverse(q))(3)
          + rotate(ang_vel(t+k·0.02), inverse(q))(3)
          + joint_position(t+k·0.02)(29)
          + joint_velocity(t+k·0.02)·0.1(29)
total = history(obs99, 10, by-frame, oldest-first) + concat(goal73 × 10) = 990 + 730 = 1720
```

动作：`target = default_angles + action_scale × action`（它们的 MJCF 是**位置执行器**，不再走 PD）。
"""
from __future__ import annotations

import numpy as np

OBS_DIM = 99
GOAL_DIM = 73
HISTORY = 10
GOAL_FRAMES = 10
DT = 0.02


def _q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def _q_inv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def _q_to_mat(q):
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ], dtype=np.float64)


def heading_quat(q):
    """只保留偏航的朝向四元数（botlab `quaternion_heading`）。"""
    w, x, y, z = q
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)], dtype=np.float64)


def relative_position(pos, origin, heading):
    """把世界系位置搬到「以 origin 为原点、heading 为朝向」的坐标系。"""
    d = np.asarray(pos, dtype=np.float64) - np.asarray(origin, dtype=np.float64)
    return _q_to_mat(_q_inv(heading)) @ d


def relative_quaternion(q, reference):
    """q 相对 reference 的姿态差（`q_inv(ref) * q`）。"""
    return _q_mul(_q_inv(np.asarray(reference, dtype=np.float64)), np.asarray(q, dtype=np.float64))


def rotation_6d(q):
    """四元数 → 6D 旋转特征（旋转矩阵前两列，逐行取：m00,m01,m10,m11,m20,m21）。"""
    m = _q_to_mat(np.asarray(q, dtype=np.float64))
    return np.array([m[0, 0], m[0, 1], m[1, 0], m[1, 1], m[2, 0], m[2, 1]], dtype=np.float64)


def rotate(vec, q):
    return _q_to_mat(np.asarray(q, dtype=np.float64)) @ np.asarray(vec, dtype=np.float64)


def sample(motion: dict, t: float, channel: str, *, index: int = 0, nearest=True):
    """从动作里取 t 时刻某一通道（`interpolation="nearest"`，与模板一致）。

    `motion` 是 `{channel: ndarray}`；单值通道（joint_pos/vel）形状 (frames, n)，
    body 通道形状 (frames, bodies, k)。
    """
    fps = float(motion.get("fps", 50))
    frames = motion[channel].shape[0]
    if nearest:
        idx = int(np.floor(t * fps + 0.5))
        idx = min(max(idx, 0), frames - 1)
    else:
        idx = min(max(int(t * fps), 0), frames - 1)
    row = np.asarray(motion[channel][idx], dtype=np.float64)
    if row.ndim >= 2:
        row = row[index]          # body 通道：(bodies, k) → 取某一个 body
    return row


def build_obs(*, motion, joint_pos, joint_vel, base_ang_vel, gravity_b, last_action,
              qpos_quat, default_angles, t, body_index=0, joint_order=None) -> np.ndarray:
    """一步的 1720 维观测。`joint_order` 用于把动作里的关节序映射到策略序（缺省同序）。"""
    order = list(range(len(default_angles))) if joint_order is None else list(joint_order)
    q = np.asarray(joint_pos, dtype=np.float64) - np.asarray(default_angles, dtype=np.float64)
    qd = np.asarray(joint_vel, dtype=np.float64)
    obs = np.concatenate([
        q[order],
        qd[order],
        np.asarray(base_ang_vel, dtype=np.float64),
        np.asarray(gravity_b, dtype=np.float64),
        np.asarray(last_action, dtype=np.float64)[order],
    ])
    r0 = sample(motion, t, "body_quat_w", index=body_index)
    p0 = sample(motion, t, "body_pos_w", index=body_index)
    heading = heading_quat(r0)
    # 模板里是 `rotation_6d(relative_quaternion(r0, qpos))` = inv(r0)·qpos（参考相对的姿态差）
    obs = np.concatenate([obs, rotation_6d(relative_quaternion(r0, np.asarray(qpos_quat, dtype=np.float64)))])

    goals = []
    for k in range(1, GOAL_FRAMES + 1):
        tf = t + k * DT
        qk = sample(motion, tf, "body_quat_w", index=body_index)
        pk = sample(motion, tf, "body_pos_w", index=body_index)
        vk = sample(motion, tf, "body_lin_vel_w", index=body_index)
        wk = sample(motion, tf, "body_ang_vel_w", index=body_index)
        jp = sample(motion, tf, "joint_pos")
        jv = sample(motion, tf, "joint_vel")
        goals.append(np.concatenate([
            relative_position(pk, p0, heading),
            rotation_6d(relative_quaternion(qk, heading)),
            rotate(vk, _q_inv(qk)),
            rotate(wk, _q_inv(qk)),
            jp[order],
            jv[order] * 0.1,
        ]))
    return np.concatenate([obs, np.concatenate(goals)]).astype(np.float32)


class History:
    """99 维本体观测的 10 帧环形历史（最老在前，摊平成 990）。"""

    def __init__(self, dim: int = OBS_DIM, frames: int = HISTORY):
        self.dim = dim
        self.frames = frames
        self.buf = np.zeros((frames, dim), dtype=np.float64)

    def push(self, obs: np.ndarray) -> np.ndarray:
        self.buf = np.roll(self.buf, -1, axis=0)
        self.buf[-1] = obs
        return self.buf.reshape(-1)

    def reset(self):
        self.buf[:] = 0.0
