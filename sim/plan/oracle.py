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
        if nd["name"] == "state_machine":
            p = doc["params"][nd["param"]["off"]:nd["param"]["off"] + nd["param"]["len"]]
            state[i] = {"state": int(p[1]), "steps": 0}
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
        elif name == "constant":
            value = np.array(p, dtype=F32)
        elif name == "select":
            mode_val = float(ins[0][0])
            n_choices = len(ins) - 1
            width = int(p[-2])
            passive = int(p[-1])
            picked = None
            for i in range(n_choices):
                if float(p[i]) == mode_val:
                    picked = ins[1 + i]
                    break
            if picked is None:
                picked = ins[1 + passive] if 0 <= passive < n_choices else np.zeros(width, dtype=F32)
            value = picked
        elif name == "where":
            value = ins[1] if float(ins[0][0]) != 0.0 else ins[2]
        elif name == "phase_clock":
            period = float(p[0])
            steps = float(ins[0][0])
            value = np.array([(steps % period) / period], dtype=F32)
        elif name == "motion":
            frames = int(p[0])
            dim = int(p[1])
            fps = float(p[2])
            loop = p[3] != 0
            table = np.asarray(p[4:4 + frames * dim], dtype=F32).reshape(frames, dim)
            pos = max(0.0, float(ins[0][0]) * fps)
            i0 = int(np.floor(pos))
            frac = pos - i0
            if loop:
                i0 %= frames
                i1 = (i0 + 1) % frames
            else:
                if i0 > frames - 1:
                    i0, frac = frames - 1, 0.0
                i1 = i0 + 1 if i0 + 1 < frames else i0
            value = table[i0] * (1 - frac) + table[i1] * frac
        elif name == "add":
            value = np.zeros(out["len"], dtype=F32)
            for x in ins:
                value = value + x
        elif name == "state_machine":
            n_tr = int(p[0])
            n_cond = int(p[2])
            cur = state[i]["state"]
            steps = state[i]["steps"]
            cmd = float(ins[0][0])
            cond_v = [float(ins[1 + k][0]) for k in range(n_cond)]
            switched, new_state, new_steps = False, cur, steps
            for kind_wanted in (0, 1, 2):
                if switched:
                    break
                for k in range(n_tr):
                    base = 3 + k * 6
                    frm, to, kind = int(p[base]), int(p[base + 1]), int(p[base + 2])
                    val, ci = float(p[base + 3]), int(p[base + 4])
                    reset = p[base + 5] != 0.0
                    if kind != kind_wanted:
                        continue
                    if frm >= 0 and frm != cur:
                        continue
                    hit = False
                    if kind == 0:
                        hit = cmd == val and cmd != 0.0
                    elif kind == 1:
                        hit = steps >= val
                    else:
                        hit = 0 <= ci < n_cond and cond_v[ci] != 0.0
                    if hit:
                        switched, new_state = True, to
                        new_steps = 0 if reset else steps + 1
                        break
            if not switched:
                new_steps = steps + 1
            state[i] = {"state": new_state, "steps": new_steps}
            value = np.array([float(new_state), float(new_steps)], dtype=F32)
        elif name == "compare":
            op = int(p[0])
            a_, b_ = float(ins[0][0]), float(ins[1][0])
            res = {0: a_ > b_, 1: a_ >= b_, 2: a_ < b_, 3: a_ <= b_, 4: a_ == b_}[op]
            value = np.array([1.0 if res else 0.0], dtype=F32)
        elif name == "out_action":
            value = ins[0]
            action = value.copy()
        else:
            raise ValueError(f"未知节点 {name}")

        arena[out["off"]:out["off"] + out["len"]] = np.asarray(value, dtype=F32)

    if action is None:
        raise ValueError("plan 没有 out_action")
    return action, state
