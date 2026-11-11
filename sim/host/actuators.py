#!/usr/bin/env python3
"""执行器语义：位置（MuJoCo 自己算 PD）还是扭矩（我们在控制步里算 PD）。

两种模型不能混：

| 模型 | MJCF | ctrl 是什么 | PD 在哪 |
|---|---|---|---|
| 位置 | `<position kp kv>` | 关节目标角 | MuJoCo 在每个物理子步算 |
| 扭矩 | `<motor ctrlrange=±88>` | **力矩** | 我们在每个控制步算一次，4 个子步保持 |

G1（botlab 站点共享模型）是扭矩；MicroDuck 是位置。混用会让机器人直接倒
（2026-11-12 实测：把扭矩当目标角喂给位置执行器 → 58 rad 的目标，瞬间翻飞）。

本模块只做纯计算，宿主（`run_bundle.py` / `run_project.py`）和浏览器（`play-core.mjs`）
都按同一套公式，保证两端一致。
"""
from __future__ import annotations

from typing import Sequence

import numpy as np


def policy_to_motor(action: Sequence[float], motor_order: Sequence[int] | None) -> np.ndarray:
    """策略序 → 电机序：`out[motor_order[i]] = action[i]`。缺省视为同序。"""
    act = np.asarray(action, dtype=np.float64)
    if motor_order is None:
        return act.copy()
    out = np.zeros(len(motor_order), dtype=np.float64)
    for i, m in enumerate(motor_order):
        out[m] = act[i]
    return out


def target_from_action(action, *, motor_order=None, default=None, scale=None) -> np.ndarray:
    """目标角 = 默认角 + 缩放 × 动作（都在电机序里算）。"""
    mapped = policy_to_motor(action, motor_order)
    base = np.zeros_like(mapped) if default is None else np.asarray(default, dtype=np.float64)
    gain = np.ones_like(mapped) if scale is None else np.asarray(scale, dtype=np.float64)
    if base.shape != mapped.shape or gain.shape != mapped.shape:
        raise ValueError(f"形状不一致：action {mapped.shape} default {base.shape} scale {gain.shape}")
    return base + gain * mapped


def torque_from_target(target, qpos, qvel, *, kp, kd, effort_limit=None) -> np.ndarray:
    """tau = kp·(target − q) − kd·qd，再按力矩上限截断。"""
    tgt = np.asarray(target, dtype=np.float64)
    q = np.asarray(qpos, dtype=np.float64)
    qd = np.asarray(qvel, dtype=np.float64)
    k_p = np.asarray(kp, dtype=np.float64)
    k_d = np.asarray(kd, dtype=np.float64)
    tau = k_p * (tgt - q) - k_d * qd
    if effort_limit is not None:
        eff = np.asarray(effort_limit, dtype=np.float64)
        tau = np.clip(tau, -eff, eff)
    return tau


def ctrl_from_action(action, preset: dict, *, qpos, qvel) -> tuple[np.ndarray, np.ndarray]:
    """按 preset 的执行器模式算出要写进 `d.ctrl` 的数组。

    返回 `(ctrl, target)`：
    - `actuatorMode == "torque"`：ctrl 是力矩（用 qpos/qvel 现算），target 是中间量（给指标用）
    - 否则：ctrl 就是 target（位置执行器），qpos/qvel 忽略
    """
    target = target_from_action(
        action,
        motor_order=preset.get("motorJointOrder"),
        default=preset.get("defaultJointPositions"),
        scale=preset.get("actionScale"),
    )
    mode = (preset.get("actuatorMode") or "position").lower()
    if mode == "torque":
        ctrl = torque_from_target(
            target, qpos, qvel,
            kp=preset.get("kp") or [0.0] * len(target),
            kd=preset.get("kd") or [0.0] * len(target),
            effort_limit=preset.get("effortLimit") or None,
        )
        return ctrl, target
    return target, target
