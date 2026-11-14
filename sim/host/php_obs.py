#!/usr/bin/env python3
"""G1 PHP Parkour 的观测装配（140 维，两段推理）。

模板 `G1-PHP-Parkour/controller.py`：

```python
command_index = rule_index(command, COMMAND_RULES, default=0)   # 4 维指令 → 11 类
depth_frame = depth_camera.depth_frame(max_age=7)
depth = resize(depth_frame.data, input_size=(98,58), output_size=(87,58))
depth = 1.1111111111111112 * depth - 0.6111111111111112
latent = depth_backbone(depth)                  # 32
latent = delay(latent, total_steps=7, age=depth_frame.age_steps, prime=True)
action = actor(gravity, angvel, q-def, qd, last_action,
               one_hot(command_index, 11), constant(0, 4), latent,
               constants={"time_step": [0.0]})
target = default + (action_scale * action)[motor_order]
```

要点：

- **没有历史缓冲**：除 latent 的 7 步延迟外，全部是当前帧；
- 观测 = 3 + 3 + 29 + 29 + 29 + 11 + 4 + 32 = **140**；
- actor 还多一个输入 `time_step`（常量 0）；
- 深度先按 offset (4,0) 从 106×60 裁成 98×58，再缩到 87×58（`align_corners` 语义见 resize_depth）；
- 指令是 4 维（前进/横向/转向/高速标志），用规则表映射成 11 类 one-hot。
"""
from __future__ import annotations

import numpy as np

JOINT_COUNT = 29
LATENT_DIM = 32
COMMAND_CLASSES = 11
PAD = 4
OBS_DIM = 3 + 3 + JOINT_COUNT + JOINT_COUNT + JOINT_COUNT + COMMAND_CLASSES + PAD + LATENT_DIM  # 140
DEPTH_SIZE = (58, 87)          # 高 × 宽（backbone 输入）
DEPTH_AFFINE_SCALE = 1.1111111111111112
DEPTH_AFFINE_OFFSET = -0.6111111111111112
LATENT_DELAY = 7
EPS = 1e-4


def _cmp(index, op, value):
    return (index, op, value)


def _all(*conds):
    return ("all", conds)


def _any(*conds):
    return ("any", conds)


FORWARD = _cmp(0, "gt", EPS)
LEFT = _cmp(1, "gt", EPS)
RIGHT = _cmp(1, "lt", -EPS)
TURN_LEFT = _cmp(2, "gt", EPS)
TURN_RIGHT = _cmp(2, "lt", -EPS)
HIGH_SPEED = _cmp(3, "gte", 0.5)

LEFT_SKILL = _any(TURN_LEFT, _all(FORWARD, LEFT))
RIGHT_SKILL = _any(TURN_RIGHT, _all(FORWARD, RIGHT))

# 与 php_policy.py 的 COMMAND_RULES 同序（先匹配先赢）
COMMAND_RULES = [
    (7, _all(HIGH_SPEED, LEFT_SKILL)),
    (9, _all(HIGH_SPEED, RIGHT_SKILL)),
    (6, _all(HIGH_SPEED, FORWARD)),
    (8, _all(HIGH_SPEED, LEFT)),
    (10, _all(HIGH_SPEED, RIGHT)),
    (2, LEFT_SKILL),
    (4, RIGHT_SKILL),
    (1, FORWARD),
    (3, _all(LEFT)),
    (5, _all(RIGHT)),
]


def _test(cond, command) -> bool:
    kind = cond[0]
    if kind == "all":
        return all(_test(c, command) for c in cond[1])
    if kind == "any":
        return any(_test(c, command) for c in cond[1])
    index, op, value = cond
    v = float(command[index])
    return {"gt": v > value, "lt": v < value, "gte": v >= value}[op]


def command_index(command) -> int:
    cmd = np.asarray(command, dtype=np.float64).reshape(-1)
    for value, cond in COMMAND_RULES:
        if _test(cond, cmd):
            return value
    return 0


def one_hot(index: int, size: int = COMMAND_CLASSES) -> np.ndarray:
    out = np.zeros(size, dtype=np.float64)
    out[int(index) % size] = 1.0
    return out


class LatentDelay:
    """`delay(latent, total_steps=7, age=..., prime=True)` 的**逐行照抄**。

    取自 `botlab/_runtime/_processors/_temporal.py`：

    ```
    slots = total_steps + 1
    if prime and not primed: ring[:] = tile(source, slots); primed = True
    ring[cursor] = source
    cursor = (cursor + 1) % slots
    read_slot = (cursor - (total - age) - 1 + slots) % slots
    out = ring[read_slot]
    ```
    """

    def __init__(self, steps: int = LATENT_DELAY, dim: int = LATENT_DIM):
        self.total = int(steps)
        self.dim = int(dim)
        self.slots = self.total + 1
        self.ring = np.zeros((self.slots, self.dim), dtype=np.float64)
        self.cursor = 0
        self.primed = False

    def push(self, latent, age_steps: int = 0) -> np.ndarray:
        src = np.asarray(latent, dtype=np.float64).reshape(-1)
        if src.shape[0] != self.dim:
            raise ValueError(f"latent 应为 {self.dim} 维，现在是 {src.shape[0]}")
        if not self.primed:
            self.ring[:] = np.tile(src, (self.slots, 1))
            self.primed = True
        self.ring[self.cursor] = src
        self.cursor = (self.cursor + 1) % self.slots
        age = min(self.total, max(0, int(age_steps)))
        delay_steps = max(0, self.total - age)
        read_slot = (self.cursor - delay_steps - 1 + self.slots) % self.slots
        return self.ring[read_slot].copy()

    def reset(self):
        self.ring[:] = 0.0
        self.cursor = 0
        self.primed = False


def actor_obs(*, gravity_b, angvel_b, joint_pos, default_angles, joint_vel,
              last_action, command, latent) -> np.ndarray:
    cmd_index = command_index(command)
    parts = [
        np.asarray(gravity_b, dtype=np.float64),
        np.asarray(angvel_b, dtype=np.float64),
        np.asarray(joint_pos, dtype=np.float64) - np.asarray(default_angles, dtype=np.float64),
        np.asarray(joint_vel, dtype=np.float64),
        np.asarray(last_action, dtype=np.float64),
        one_hot(cmd_index),
        np.zeros(PAD, dtype=np.float64),
        np.asarray(latent, dtype=np.float64).reshape(-1),
    ]
    obs = np.concatenate(parts)
    if obs.shape[0] != OBS_DIM:
        raise ValueError(f"PHP obs 应为 {OBS_DIM} 维，现在 {obs.shape[0]}")
    return obs.astype(np.float32)
