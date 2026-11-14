#!/usr/bin/env python3
"""WBC（mimic）模式的观测装配：439 = 动作参考 67 + 机器人状态 4×93。

规格不是猜的——来自 `ccrpRepo/wbc_fsm` 的 `State_WBC.{h,cpp}`，并用 botlab 运行时录下的
真值逐段核对过（2026-11-12）：

| 段 | 长度 | 内容 | 核对结果 |
|---|---|---|---|
| 参考：关节位置 | 29 | 动作参考当前帧的 `joint_pos` | 与 motion 第 1 帧逐位相同 |
| 参考：关节速度 | 29 | 同一帧的 `joint_vel` | 逐位相同（max diff 0.000） |
| 参考：锚点位置（机体系） | 3 | 相对上一参考帧的锚点位移，转到机体系 | 首步为 [0,0,0] ✓ |
| 参考：锚点朝向 6D | 6 | 相对旋转矩阵的 `[m00,m01,m10,m11,m20,m21]` | 首步为 [1,0,0,1,0,0] ✓ |
| 机器人状态 | 93×4 | 每帧 `[角速度3, 投影重力3, q29, qd29, 上一步动作29]`，最老在前 | 段边界与真值一致 |

参考实现里的常量：`_actor_state_history_length = 4`、`_robot_state_dim = 93`、
`_reference_dim = 67`、`_mimic_obs_predictive_horizon = 1`（只取当前参考帧，不做前瞻）。

参考帧索引：`idx = refer_idx + i*frame_interval`，夹在 `[1, end_refer_idx]`；
暂停时 `idx = last_idx = refer_idx`，且参考关节速度强制为 0。
"""
from __future__ import annotations

import numpy as np

GRAVITY = np.array([0.0, 0.0, -1.0])
ROBOT_STATE_DIM = 93
REFERENCE_DIM = 67
HISTORY_FRAMES = 4


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_quat(q):
    """只保留绕 z 的偏航分量。"""
    w, x, y, z = q
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])


def quat_apply_inverse(q, v):
    return quat_to_mat(q).T @ np.asarray(v, dtype=float)


def six_d(m):
    return np.array([m[0, 0], m[0, 1], m[1, 0], m[1, 1], m[2, 0], m[2, 1]], dtype=np.float32)


def reference_block(*, joint_pos, joint_vel, anchor_pos, anchor_quat,
                    last_anchor_pos, last_anchor_quat, base_quat=None, pause=False) -> np.ndarray:
    """动作参考 67 维——与 botlab 引擎 `write_motion_reference()` **逐字同公式**。

    权威实现在 `botlab/_runtime/_processors/_output.py`（不是 ccrpRepo 那份）：

        last_inv     = quat_inverse(last_quat)          # 只做「上一参考帧」的相对变换
        target_pos   = quat_apply(last_inv, cur_pos - last_pos)
        target_quat  = last_inv * cur_quat
        6D           = mat[0], mat[1], mat[3], mat[4], mat[6], mat[7]

    `base_quat` 只保留在签名里做兼容：botlab 这一版**不做基座偏航对齐**。
    核对：dance12 + G1，和他们的 obs 前 67 维差 **0.00000**（各步）。
    """
    if pause:
        joint_vel = np.zeros_like(np.asarray(joint_vel, float))
    last_inv = quat_conj(np.asarray(last_anchor_quat, float))
    # 注意：`quat_apply(last_inv, dp)` = R(last_inv) @ dp（**不是**转置）
    rel_pos = quat_to_mat(last_inv) @ (np.asarray(anchor_pos, float) - np.asarray(last_anchor_pos, float))
    rel_quat = quat_mul(last_inv, np.asarray(anchor_quat, float))
    m = quat_to_mat(rel_quat)
    return np.concatenate([
        np.asarray(joint_pos, np.float32),
        np.asarray(joint_vel, np.float32),
        rel_pos.astype(np.float32),
        np.array([m[0, 0], m[0, 1], m[1, 0], m[1, 1], m[2, 0], m[2, 1]], np.float32),
    ]).astype(np.float32)


def robot_state_frame(*, ang_vel, gravity_b, q, qd, last_action) -> np.ndarray:
    """单帧机器人状态 93 维。"""
    return np.concatenate([
        np.asarray(ang_vel, np.float32),
        np.asarray(gravity_b, np.float32),
        np.asarray(q, np.float32),
        np.asarray(qd, np.float32),
        np.asarray(last_action, np.float32),
    ]).astype(np.float32)


def assemble(*, reference67, history4) -> np.ndarray:
    """拼成 439：参考在前，机器人状态 4 帧在后（缓冲区由调用方维护，最老在前）。"""
    hist = np.asarray(history4, dtype=np.float32)
    if hist.shape != (HISTORY_FRAMES, ROBOT_STATE_DIM):
        raise ValueError(f"history 形状 {hist.shape} != ({HISTORY_FRAMES}, {ROBOT_STATE_DIM})")
    ref = np.asarray(reference67, dtype=np.float32)
    if ref.shape != (REFERENCE_DIM,):
        raise ValueError(f"reference 形状 {ref.shape} != ({REFERENCE_DIM},)")
    return np.concatenate([ref, hist.reshape(-1)]).astype(np.float32)


class Playhead:
    """动作参考的播放头：每控制步推进 `frame_interval` 帧，支持暂停与区间夹取。

    语义对齐 `ccrpRepo/wbc_fsm` 的 `State_WBC`：
    - `idx = refer_idx + i*frame_interval`，先夹到 `[1, end]` 再用；
    - 暂停时 `idx = last_idx = refer_idx`，且参考关节速度强制 0；
    - 进入 mode 时从 `start` 开始（参考实现 `start=0`，实际用之前夹到 1）。
    """

    def __init__(self, frame_count: int, *, start: int = 0, frame_interval: int = 5,
                 end: int | None = None):
        if frame_count < 2:
            raise ValueError("动作参考至少 2 帧")
        self.frame_count = int(frame_count)
        self.end = int(end) if end is not None else self.frame_count - 1
        # `frame_interval` 只用于算锚点的「上一帧」（参考实现里 last_idx = idx - _frame_interval），
        # **不**决定推进步长：播放头每个控制步 +1（真值反推：obs 的关节位置每步前进 1 帧）。
        self.frame_interval = max(1, int(frame_interval))
        self.refer_idx = max(1, int(start))   # 第 0 帧不用（参考实现也夹到 >=1）
        self.paused = False

    def _clamp(self, idx: int) -> int:
        if idx >= self.end:
            return self.end
        if idx <= 1:
            return 1
        return idx

    def current(self) -> tuple[int, int]:
        """返回 `(idx, last_idx)`；暂停时两者相同。"""
        idx = self._clamp(self.refer_idx)
        if self.paused:
            return idx, idx
        return idx, self._clamp(idx - self.frame_interval)

    def advance(self) -> None:
        """每控制步前进 **1 帧**（锚点的回看距离由 `frame_interval` 决定，见 current()）。"""
        if not self.paused:
            self.refer_idx += 1

    def pause(self, flag: bool = True) -> None:
        self.paused = bool(flag)

    def at_end(self) -> bool:
        return self.refer_idx >= self.end


def assemble_observation(*, motion, playhead: "Playhead", robot_state_history,
                         base_quat, anchor: int = 0,
                         joint_pos=None, joint_vel=None,
                         body_pos=None, body_quat=None) -> np.ndarray:
    """一步完整的 439 维观测：参考 67 + 机器人状态 4×93。

    `motion` 是 `sim/plan/motion.load_motion` 出来的原始数组（通道 → data）；
    也可以直接传 `joint_pos` / `joint_vel` / `body_pos` / `body_quat` 覆盖。
    `robot_state_history` 是 `(4, 93)` 的缓冲区（最老在前），由调用方滚动维护。
    """
    jp = joint_pos if joint_pos is not None else motion["joint_pos"]
    jv = joint_vel if joint_vel is not None else motion["joint_vel"]
    bp = body_pos if body_pos is not None else motion["body_pos"]
    bq = body_quat if body_quat is not None else motion["body_quat"]
    idx, last = playhead.current()
    ref = reference_block(
        joint_pos=jp[idx], joint_vel=jv[idx],
        anchor_pos=bp[idx][anchor], anchor_quat=bq[idx][anchor],
        last_anchor_pos=bp[last][anchor], last_anchor_quat=bq[last][anchor],
        base_quat=base_quat, pause=playhead.paused)
    return assemble(reference67=ref, history4=robot_state_history)
