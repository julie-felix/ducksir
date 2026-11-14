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


def _bin_header_len(path: str, width_floats: int) -> int:
    """botlab 的 `.bin` 通道：`NPZ\\0` 魔数 + 若干 u32 头 + float32 数据。

    头长度从 {16,20,24,28,32} 里挑**唯一**能让 `(len - h)` 被 `4*width` 整除的那个。
    实测（dance12/joint_pos.bin，6574×29）：h=24，且 fps=50 时「位置求导 = 速度」误差 **0.0**，
    说明容器就是「头 + 行主序 float32」。
    """
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        magic = f.read(4)
    if magic != b"NPZ\x00":
        raise ValueError(f"{os.path.basename(path)} 不是 NPZ 容器（前 4 字节 {magic!r}）")
    cands = [h for h in (16, 20, 24, 28, 32)
             if size > h and (size - h) % (4 * width_floats) == 0
             and (size - h) // (4 * width_floats) > 0]
    if len(cands) != 1:
        raise ValueError(f"{path} 头长度无法唯一确定（候选 {cands}，宽 {width_floats}）")
    return cands[0]


def _read_bin(path: str, width_floats: int):
    head = _bin_header_len(path, width_floats)
    size = os.path.getsize(path)
    frames = (size - head) // (4 * width_floats)
    with open(path, "rb") as f:
        f.seek(head)
        raw = f.read(frames * 4 * width_floats)
    return np.frombuffer(raw, dtype="<f4").reshape(frames, width_floats).copy(), frames


def _fps_candidates(path: str) -> list:
    """fps.bin 没有公开规范：把头里所有 4 字节对齐的 u32/f32（1..240）当候选列出来。"""
    if not os.path.exists(path):
        return []
    raw = open(path, "rb").read()
    out = []
    for off in range(0, len(raw) - 3, 4):
        u = int(np.frombuffer(raw[off:off + 4], dtype="<u4")[0])
        if 1 <= u <= 240:
            out.append(float(u))
        v = float(np.frombuffer(raw[off:off + 4], dtype="<f4")[0])
        if 1.0 <= v <= 240.0:
            out.append(float(v))
    seen, uniq = set(), []
    for x in out:
        if x not in seen:
            seen.add(x)
            uniq.append(x)
    return uniq


def load_bin_dir(path: str, channel: str = "joint_pos", indices=None) -> dict:
    """读「目录 + 每通道一个 .bin」形态（botlab 的 WBC 动作就是这种）。

    fps 的来源：目录里**同时有** `joint_pos.bin` 与 `joint_vel.bin` 时，
    挑「位置差分 × fps ≈ 速度」误差最小的候选（dance12 实测 50.0、误差 0.0）；
    否则退回 `metadata.json` 的 fps。两者都没有就报错——不瞎猜。
    """
    if channel not in CHANNELS:
        raise ValueError(f"bin 通道 {channel} 不支持，可选：{CHANNELS}")
    if not os.path.isdir(path):
        raise ValueError(f"{path} 不是目录")
    meta = {}
    meta_path = os.path.join(path, "metadata.json")
    if os.path.exists(meta_path):
        with open(meta_path, encoding="utf-8") as f:
            meta = json.load(f)
    names = meta.get("body_names") if channel.startswith("body_") else (
        meta.get("dof_names") or meta.get("joint_names"))
    if not names:
        raise ValueError(f"{path} 的 metadata.json 没有名字表，推不出每帧宽度")
    per = 4 if channel == "body_quat_w" else 3
    width = len(names) * per if channel.startswith("body_") else len(names)

    bin_path = os.path.join(path, f"{channel}.bin")
    if not os.path.exists(bin_path):
        raise ValueError(f"{path} 里没有 {channel}.bin")
    data, frames = _read_bin(bin_path, width)

    fps = None
    q_path = os.path.join(path, "joint_pos.bin")
    qd_path = os.path.join(path, "joint_vel.bin")
    if os.path.exists(q_path) and os.path.exists(qd_path):
        cands = _fps_candidates(os.path.join(path, "fps.bin")) or (
            [float(meta["fps"])] if meta.get("fps") else [])
        if cands:
            dof_names = meta.get("dof_names") or meta.get("joint_names")
            try:
                qp, _ = _read_bin(q_path, len(dof_names) if dof_names else len(names))
                qv, _ = _read_bin(qd_path, len(dof_names) if dof_names else len(names))
                n_min = min(len(qp), len(qv))
                if n_min > 4:
                    cent = (qp[2:n_min] - qp[0:n_min - 2]) / 2.0
                    ref = qv[1:n_min - 1]
                    scale = max(1e-6, float(np.abs(ref).mean()))
                    best, best_err = None, None
                    for cand in cands:
                        err = float(np.abs(cent * cand - ref).mean())
                        if best_err is None or err < best_err:
                            best, best_err = cand, err
                    if best_err is not None and best_err < 0.05 * scale:
                        fps = best
            except ValueError:
                fps = None
    if fps is None and meta.get("fps"):
        fps = float(meta["fps"])
    if not fps:
        raise ValueError(f"{path} 无法确定 fps（fps.bin / metadata.json 都没有可用值）")

    data, kept = _pick(data, names, indices)
    return {"frames": int(frames), "fps": float(fps), "dim": int(data.shape[1]),
            "names": kept, "data": data}


def load_motion(path: str, channel: str = "joint_pos", indices=None) -> dict:
    """按路径形态自动选 npz / 目录，返回统一结构。"""
    if os.path.isdir(path):
        return load_bin_dir(path, channel, indices)
    return load_npz(path, channel, indices)
