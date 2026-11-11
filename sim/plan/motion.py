#!/usr/bin/env python3
"""动作参考（motion）加载：把 botlab 那两种素材格式读成 (frames, dim) + fps。

botlab 的 `Motion` 支持两种源（见 docs/notes/botlab-templates-inventory.md）：

1. **单文件 `.npz`**（Instinct / UniSuper / GMTrack）：键是 `fps`、`joint_pos`、`joint_vel`、
   `body_pos_w`、`body_quat_w`、`body_lin_vel_w`、`body_ang_vel_w`，第一维都是帧；
2. **目录 + 通道 `.bin`**（WBC 的 `dance12/`）：每个通道一个 float32 `.bin`，另加 `metadata.json`
   与 `fps.bin`。

本模块只做一件事：把其中**一个通道的若干列**取出来，变成 `ir.Plan.motion()` 吃的二维数组。
不解释语义、不做重定向——那是训练侧的事。
"""

from __future__ import annotations

import json
import os

import numpy as np

# 允许的通道名（与 botlab 的 Motion.sample 支持的 channel 对齐）
CHANNELS = ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w",
            "body_lin_vel_w", "body_ang_vel_w")


def _flatten(values: np.ndarray) -> np.ndarray:
    """(frames, ...) → (frames, dim)：把尾部维度压平成一行。"""
    arr = np.asarray(values, dtype=np.float32)
    if arr.ndim == 1:
        return arr.reshape(-1, 1)
    return arr.reshape(arr.shape[0], -1)


def _pick(raw: np.ndarray, names, indices) -> tuple[np.ndarray, list | None]:
    """按名字或下标选列；多维通道按名字选整块（body_* 的一个 body 是 3 或 4 个数）。"""
    if indices is None:
        return raw, names
    if names is None:
        return raw[:, list(indices)], None
    per = raw.shape[1] // len(names)
    picked, kept = [], []
    for idx in indices:
        i = idx if isinstance(idx, int) else names.index(idx)
        picked.extend(range(i * per, (i + 1) * per))
        kept.append(names[i])
    return raw[:, picked], kept


def load_npz(path: str, channel: str = "joint_pos", indices=None) -> dict:
    """读单文件 npz，返回 `{frames, fps, dim, names, data}`（data 是 (frames, dim)）。"""
    if channel not in CHANNELS:
        raise ValueError(f"npz 通道 {channel} 不支持，可选：{CHANNELS}")
    with np.load(path, allow_pickle=False) as z:
        if channel not in z:
            raise ValueError(f"{os.path.basename(path)} 里没有通道 {channel}（有 {list(z.keys())}）")
        raw = _flatten(z[channel])
        fps = float(np.asarray(z["fps"]).reshape(-1)[0]) if "fps" in z else 50.0
        names = None
        key = "dof_names" if channel in ("joint_pos", "joint_vel") else "body_names"
        if key in z:
            names = [str(x) for x in z[key]]
        if names is not None and raw.shape[1] % len(names) != 0:
            names = None
        data, kept = _pick(raw, names, indices)
    return {"frames": int(data.shape[0]), "fps": fps, "dim": int(data.shape[1]),
            "names": kept, "data": data}


def load_bin_dir(path: str, channel: str = "joint_pos", indices=None) -> dict:
    """「目录 + 每通道一个 .bin」形态——**暂不支持**，原因写在异常里。

    botlab 的 WBC 动作是这种容器：`NPZ\0` 魔数 + 若干 u32 头 + float32 数据，没有公开规范。
    我实测能推出帧数（跳过头后按 `4*宽` 整除，6574 帧、29 维对得上），但 **fps 与字段含义无法唯一确定**
    （头里有多个都落在 1–240 的整数：1 / 8 / 50 / 102 / 108）。宁可不读，也不猜着读——
    猜错会让「动作参考」整体错位却不报错。

    要支持它，需要 botlab 的加载器源码或一份权威说明。在此之前用 npz 形态
    （Instinct / UniSuper / GMTrack 三个项目都是 npz）。
    """
    raise NotImplementedError(
        f"{os.path.basename(path)} 是 botlab 的私有 .bin 动作容器（NPZ 魔数），"
        "fps 与字段含义没有公开规范，本项目暂不解析；请改用 .npz 形态的动作素材"
        "（见 docs/notes/botlab-templates-inventory.md 第 9 节）"
    )


def load_motion(path: str, channel: str = "joint_pos", indices=None) -> dict:
    """按路径形态自动选 npz / 目录，返回统一结构。"""
    if os.path.isdir(path):
        return load_bin_dir(path, channel, indices)
    return load_npz(path, channel, indices)
