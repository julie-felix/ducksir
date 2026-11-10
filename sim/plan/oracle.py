#!/usr/bin/env python3
"""plan 的 numpy 参考实现 —— C++ 核对拍用的 oracle（计划 D18）。

作用只有一个：**把 8 个节点的语义写成可执行的规格**。核跑出来的数要能和这里逐点对上（≤1e-6），
否则就是核写歪了或语义漂了。它不进运行时、不参与部署。

约定（与 `contract/plan/README.md` 一致）：
- 全部 float32；arena 是固定长度的一维数组，节点输出按声明顺序落在固定偏移上；
- `history` 首步用「当前输入填充整条历史」（等于保持不动），之后才是环形缓冲；
- `model` 的 pass-through 只在测试里用，真实推理走宿主回调。
"""

from __future__ import annotations

import numpy as np

F32 = np.float32


def initial_state(doc: dict) -> dict:
    """每个 history 节点一份环形缓冲。"""
    state = {}
    for i, nd in enumerate(doc["nodes"]):
        if nd["name"] == "history":
            frames = int(doc["params"][nd["param"]["off"]])
            n = nd["out"]["len"] // frames
            state[i] = np.zeros((frames, n), dtype=F32)
    return state


def step(doc: dict, obs, state: dict | None = None, backend=None):
    """跑一步。返回 (action, state)。backend 是可选的 `f(inp) -> out` 回调，用于 model 节点。"""
    obs = np.asarray(obs, dtype=F32)
    if obs.shape != (doc["input_dim"],):
        raise ValueError(f"obs 维度 {obs.shape} != {doc['input_dim']}")
    if state is None:
        state = initial_state(doc)
    arena = np.zeros(doc["arena_len"], dtype=F32)
    params = doc["params"]
    action = None

    for i, nd in enumerate(doc["nodes"]):
        name = nd["name"]
        out = nd["out"]
        p_off, p_len = nd["param"]["off"], nd["param"]["len"]
        p = params[p_off:p_off + p_len] if p_len else []
        ins = [arena[k["off"]:k["off"] + k["len"]] for k in nd["inputs"]]

        if name == "obs":
            src_off, length = int(p[0]), int(p[1])
            value = obs[src_off:src_off + length]
        elif name == "concat":
            value = np.concatenate(ins) if len(ins) > 1 else ins[0]
        elif name == "affine":
            n = out["len"]
            scale = np.array(p[:n], dtype=F32)
            offset = np.array(p[n:2 * n], dtype=F32)
            value = ins[0] * scale + offset
        elif name == "clip":
            n = out["len"]
            lo = np.array(p[:n], dtype=F32)
            hi = np.array(p[n:2 * n], dtype=F32)
            value = np.minimum(np.maximum(ins[0], lo), hi)
        elif name == "reorder":
            perm = [int(v) for v in p]
            value = ins[0][perm]
        elif name == "history":
            frames = int(p[0])
            buf = state[i]
            n = buf.shape[1]
            buf = np.roll(buf, -1, axis=0)
            buf[-1] = ins[0][:n]
            state[i] = buf
            value = buf.reshape(-1)
        elif name == "model":
            if nd["flags"] & 1 or backend is None:   # FLAG_PASS_THROUGH
                value = ins[0].copy()
            else:
                value = np.asarray(backend(ins[0]), dtype=F32)
                if value.shape != (out["len"],):
                    raise ValueError(f"model 输出维度 {value.shape} != {out['len']}")
        elif name == "out_action":
            value = ins[0]
            action = value.copy()
        else:
            raise ValueError(f"未知节点 {name}")

        arena[out["off"]:out["off"] + out["len"]] = np.asarray(value, dtype=F32)

    if action is None:
        raise ValueError("plan 没有 out_action")
    return action, state
