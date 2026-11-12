#!/usr/bin/env python3
"""Instinct-SuperMimic 的观测装配（1990 维）。

规格来源：模板 `g1-instinct-supermimic` 的 `controller.py`（`observations()`）：

```
jp10       = joint_pos(t+k·0.1) − CONFIG.default_angles           (k=0..9, 每帧 29)
jv10       = joint_vel(t+k·0.1)                                   (10 × 29)
pos10      = relative_position(torso_pos(t+k·0.1), p0, r0)         (10 × 3)
rot10      = tangent_normal(relative_quaternion(torso_quat(t+k·0.1), robot_r)) (10 × 6)
depth      = resize(camera.depth(44×23), 32×18) → 576
hist       = 5 个 term 各 8 帧（by-frame、最老在前）：
             [gravity(torso)、base_ang_vel、joint_pos−default、joint_vel、last_action]
合计 290+290+30+60+576+744 = 1990
```

动作：`target = default + action_scale × action`（它们的 MJCF 是位置执行器）。

深度相机（来自 `camera.preset.yaml`）：挂在 `torso_link`，offset (0.0488, 0.015, 0.4378)，
fov 58（竖直），先渲 48×27 再裁 2,2→44×23，再缩到 32×18，depthScale 2。
"""
from __future__ import annotations

import numpy as np

N_JOINTS = 29
LOOKAHEAD = 10
LOOKAHEAD_DT = 0.1
DEPTH_W, DEPTH_H = 32, 18
DEPTH_FULL_W, DEPTH_FULL_H = 48, 27
DEPTH_CROP_X, DEPTH_CROP_Y = 2, 2
HISTORY_FRAMES = 8
TERMS = (("gravity", 3), ("base_ang_vel", 3), ("joint_pos", N_JOINTS),
         ("joint_vel", N_JOINTS), ("last_action", N_JOINTS))


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


def relative_position(pos, origin, quat):
    return _q_to_mat(_q_inv(np.asarray(quat, dtype=np.float64))) @ (
        np.asarray(pos, dtype=np.float64) - np.asarray(origin, dtype=np.float64))


def relative_quaternion(q, reference):
    return _q_mul(_q_inv(np.asarray(reference, dtype=np.float64)), np.asarray(q, dtype=np.float64))


def tangent_normal(q):
    """6D 特征：取旋转矩阵的前两**行**（m00,m01,m02,m10,m11,m12）再归一化法向。

    botlab 的 `tangent_normal` 就是「切向 + 法向」表示：把前两行当作切向，第三行由叉乘得到。
    """
    m = _q_to_mat(np.asarray(q, dtype=np.float64))
    return np.array([m[0, 0], m[0, 1], m[0, 2], m[1, 0], m[1, 1], m[1, 2]], dtype=np.float64)


def sample(motion: dict, t: float, channel: str, *, body: int = 0, nearest: bool = True):
    fps = float(motion.get("fps", 50))
    frames = motion[channel].shape[0]
    idx = int(np.floor(t * fps + 0.5)) if nearest else int(t * fps)
    idx = min(max(idx, 0), frames - 1)
    row = np.asarray(motion[channel][idx], dtype=np.float64)
    if row.ndim >= 2:
        row = row[body]
    return row


def build_terms(*, motion, t, duration, joint_order, config_defaults,
                robot_joint_pos, robot_joint_vel, gravity_b, base_ang_vel, last_action,
                torso_index, robot_torso_quat):
    """返回 (前 670 维的块, 5 个历史 term 的当前帧)。"""
    p0 = sample(motion, t, "body_pos_w", body=torso_index)
    r0 = sample(motion, t, "body_quat_w", body=torso_index)
    jp, jv, pos, rot = [], [], [], []
    for k in range(LOOKAHEAD):
        ti = t + k * LOOKAHEAD_DT
        valid = ti < duration
        jpk = sample(motion, ti, "joint_pos")[joint_order] - np.asarray(config_defaults)
        jvk = sample(motion, ti, "joint_vel")[joint_order]
        jp.append(jpk if valid else np.zeros(N_JOINTS))
        jv.append(jvk if valid else np.zeros(N_JOINTS))
        pk = sample(motion, ti, "body_pos_w", body=torso_index)
        pos.append(relative_position(pk, p0, r0))
        qk = sample(motion, ti, "body_quat_w", body=torso_index)
        rk = tangent_normal(relative_quaternion(qk, robot_torso_quat))
        rot.append(rk if valid else np.zeros(6))
    block = np.concatenate([np.concatenate(jp), np.concatenate(jv),
                            np.concatenate(pos), np.concatenate(rot)])
    terms = {
        "gravity": np.asarray(gravity_b, dtype=np.float64),
        "base_ang_vel": np.asarray(base_ang_vel, dtype=np.float64),
        "joint_pos": np.asarray(robot_joint_pos, dtype=np.float64),
        "joint_vel": np.asarray(robot_joint_vel, dtype=np.float64),
        "last_action": np.asarray(last_action, dtype=np.float64),
    }
    return block, terms


class TermHistory:
    """5 个 term 的 8 帧历史：by-frame、最老在前；首次用第一帧填满（mjlab 语义）。"""

    def __init__(self, frames: int = HISTORY_FRAMES):
        self.frames = frames
        self.buf = {name: None for name, _ in TERMS}

    def push(self, terms: dict) -> np.ndarray:
        out = []
        for name, dim in TERMS:
            value = np.asarray(terms[name], dtype=np.float64)
            if self.buf[name] is None:
                self.buf[name] = np.tile(value, (self.frames, 1))
            else:
                self.buf[name] = np.roll(self.buf[name], -1, axis=0)
                self.buf[name][-1] = value
            out.append(self.buf[name].reshape(-1))
        return np.concatenate(out)

    def reset(self):
        for name in self.buf:
            self.buf[name] = None


def resize_depth(depth: np.ndarray, out_w=DEPTH_W, out_h=DEPTH_H) -> np.ndarray:
    """44×23 → 32×18 的双线性缩放（无 scipy，手写）。"""
    src_h, src_w = depth.shape
    ys = np.linspace(0, src_h - 1, out_h)
    xs = np.linspace(0, src_w - 1, out_w)
    y0 = np.floor(ys).astype(int); y1 = np.minimum(y0 + 1, src_h - 1)
    x0 = np.floor(xs).astype(int); x1 = np.minimum(x0 + 1, src_w - 1)
    wy = (ys - y0)[:, None]; wx = (xs - x0)[None, :]
    top = depth[y0][:, x0] * (1 - wx) + depth[y0][:, x1] * wx
    bot = depth[y1][:, x0] * (1 - wx) + depth[y1][:, x1] * wx
    return (top * (1 - wy) + bot * wy).astype(np.float32)


def crop_depth(full: np.ndarray) -> np.ndarray:
    """48×27 全幅 → 裁掉 2,2 边缘 → 44×23。"""
    h, w = full.shape
    return full[DEPTH_CROP_Y:h - DEPTH_CROP_Y, DEPTH_CROP_X:w - DEPTH_CROP_X]
