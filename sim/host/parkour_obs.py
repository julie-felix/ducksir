#!/usr/bin/env python3
"""Hiking / PHP Parkour 的观测装配。

模板（`G1-Hiking-Parkour/controller.py`）：

```python
state = history([
    robot.base_angular_velocity(scale=0.25),   # 3
    robot.projected_gravity(),                 # 3
    robot.commands(...),                       # 3（velocity_command）
    robot.joint_position(default=...),         # 29
    robot.joint_velocity(scale=0.05),          # 29
    robot.last_action(),                       # 29
], 8, layout="by-term")                        # 96 × 8 = 768

depth = history(depth_camera.depth(max_age=5), 8, stride=5)   # 8 × 18 × 32
latent = depth_encoder(depth)                  # 128
action = actor(state, latent).clip(-100, 100).action()
target = default_angles + action_scale * action[motor_order]
```

关键点（与 Instinct/GMTrack 的差异）：

- 本体历史是 **96 × 8**，其中含 3 维速度指令；
- 深度历史是 **采样**（`stride=5`：每 5 个控制步取一帧），不是每步都取；
- 编码器和 actor 是**两个 ONNX**，要先过编码器再拼进 actor。
"""
from __future__ import annotations

import numpy as np

HISTORY = 8
DEPTH_STRIDE = 5
JOINT_COUNT = 29
STATE_PER_FRAME = 3 + 3 + 3 + 29 + 29 + 29      # 96
STATE_DIM = STATE_PER_FRAME * HISTORY            # 768
LATENT_DIM = 128
OBS_DIM = STATE_DIM + LATENT_DIM                 # 896
DEPTH_SHAPE = (HISTORY, 18, 32)

ANGVEL_SCALE = 0.25
JOINTVEL_SCALE = 0.05


class StateHistory:
    """按 term 平铺的本体历史：**最老在前、最新在后；未填的槽位补 0**。

    实测（botlab 录像 step 4 的 q 段 8 帧首值）：

    ```
    [0, 0, 0, 0, 0.003, 0.010, 0.026, 0.051]
    ```

    前 4 个还是 0（没填），后 4 个是已经发生的步、越靠后越新。
    注意这跟 GMTrack 那套「首帧填满」不同——按模板分别处理，别通用化。
    """

    def __init__(self, frames: int = HISTORY, dim: int = STATE_PER_FRAME):
        self.frames = frames
        self.dim = dim
        self.buf: list[np.ndarray] = [np.zeros(dim, dtype=np.float64) for _ in range(frames)]

    CONSTANT_TERMS = (2,)      # 第 3 段是速度指令：它不随历史变化，8 帧都填当前值

    def push(self, terms) -> np.ndarray:
        vec = np.concatenate([np.asarray(t, dtype=np.float64).reshape(-1) for t in terms])
        if vec.shape[0] != self.dim:
            raise ValueError(f"本体观测应为 {self.dim} 维，现在是 {vec.shape[0]}")
        self.buf.append(vec.copy())
        del self.buf[:max(0, len(self.buf) - self.frames)]
        # 常量段：把已填的所有帧都改成最新值（实测他们 command 段 8 帧全是 0.4）
        off = 0
        for idx, d in enumerate((3, 3, 3, JOINT_COUNT, JOINT_COUNT, JOINT_COUNT)):
            if idx in self.CONSTANT_TERMS:
                for f in self.buf:
                    f[off:off + d] = vec[off:off + d]
            off += d
        out = []
        off = 0
        for d in (3, 3, 3, JOINT_COUNT, JOINT_COUNT, JOINT_COUNT):
            for f in self.buf:
                out.append(f[off:off + d])
            off += d
        return np.concatenate(out)

    def reset(self):
        self.buf = [np.zeros(self.dim, dtype=np.float64) for _ in range(self.frames)]


class DepthHistory:
    """深度历史：每 `stride` 步采一帧（`max_age=5`），保持 8 帧，最老在前。"""

    def __init__(self, frames: int = HISTORY, stride: int = DEPTH_STRIDE):
        self.frames = frames
        self.stride = stride
        self.frames_buf: list[np.ndarray] | None = None
        self.counter = 0

    def push(self, frame: np.ndarray) -> np.ndarray:
        frame = np.asarray(frame, dtype=np.float32)
        if self.frames_buf is None:
            self.frames_buf = [frame.copy() for _ in range(self.frames)]
            self.counter = 1
            return np.stack(self.frames_buf)
        if self.counter % self.stride == 0:
            self.frames_buf.pop(0)
            self.frames_buf.append(frame.copy())
        self.counter += 1
        return np.stack(self.frames_buf)

    def reset(self):
        self.frames_buf = None
        self.counter = 0


def state_terms(*, angvel_b, gravity_b, command, joint_pos, default_angles,
                joint_vel, last_action) -> list[np.ndarray]:
    return [
        np.asarray(angvel_b, dtype=np.float64) * ANGVEL_SCALE,
        np.asarray(gravity_b, dtype=np.float64),
        np.asarray(command, dtype=np.float64),
        np.asarray(joint_pos, dtype=np.float64) - np.asarray(default_angles, dtype=np.float64),
        np.asarray(joint_vel, dtype=np.float64) * JOINTVEL_SCALE,
        np.asarray(last_action, dtype=np.float64),
    ]


def actor_obs(state_hist: np.ndarray, latent: np.ndarray) -> np.ndarray:
    return np.concatenate([state_hist, np.asarray(latent, dtype=np.float64).reshape(-1)]).astype(np.float32)
