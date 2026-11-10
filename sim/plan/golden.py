#!/usr/bin/env python3
"""golden 快照：**先记录后验证**。

    from golden import record, check, cases_from_release
    cases = cases_from_release("build/release/latest")   # [(name, blob, [输入帧…])]
    record(cases, "contract/plan/fixtures/release-golden.json")   # 只在 --update-golden 时
    problems = check(cases, "contract/plan/fixtures/release-golden.json")

为什么需要它（2026-11-10 的决定）：我们原有的对拍是「有参考实现的测试」——每次都拿 numpy oracle 重新算期望值。
golden 是「上次满分的卷子存档」：把某一版 C 核的输出按帧存下来，之后改核只跑比对，
diff 直接告诉你第几步、第几个分量差多少。两者不冲突：oracle 证明「算得对」，golden 证明「没变」。

存储格式：JSON，值用 Python float 的字面量——float32 转 float 再序列化可以**精确往返**，
所以既可读（能 diff）又不丢精度。
"""

from __future__ import annotations

import ctypes
import json
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "host"))

import ir  # noqa: E402

VERSION = 1


def _load_core():
    """加载 native C 核（`engine/build/libducksim.so`）。"""
    import run_policy as RP
    return RP.load_libs()[0]


def _run_case(core, blob: bytes, frames: list[list[float]]) -> list[list[float]]:
    err = ctypes.create_string_buffer(512)
    handle = core.ds_load(blob, len(blob), err, 512)
    if not handle:
        raise RuntimeError("ds_load 失败：" + err.value.decode())
    try:
        info = ir.read_blob(blob)
        n_in, n_out = info["input_dim"], info["output_dim"]
        outs = []
        for frame in frames:
            if len(frame) != n_in:
                raise ValueError(f"输入帧长度 {len(frame)} != plan 的 input_dim {n_in}")
            bi = (ctypes.c_float * n_in)(*frame)
            bo = (ctypes.c_float * n_out)()
            rc = core.ds_step(handle, bi, bo, err)
            if rc != 0:
                raise RuntimeError(f"ds_step 返回 {rc}：" + err.value.decode())
            outs.append([float(np.float32(v)) for v in bo])
        return outs
    finally:
        core.ds_free(handle)


def record(cases, path: str, note: str = "") -> dict:
    """跑一遍 C 核，把输出存成快照。**只在明确要更新基准时调用。**"""
    core = _load_core()
    doc = {"kind": "ducksir-golden", "version": VERSION, "note": note, "cases": {}}
    for name, blob, frames in cases:
        doc["cases"][name] = {
            "blob_digest": ir.sha256_of_blob(blob),
            "input_dim": ir.read_blob(blob)["input_dim"],
            "output_dim": ir.read_blob(blob)["output_dim"],
            "frames": len(frames),
            "in": frames,
            "out": _run_case(core, blob, frames),
        }
    os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, ensure_ascii=False)   # 单行：快照不追求可读，追求稳定
    return doc


def check(cases, path: str) -> list[str]:
    """拿当前 C 核跑同样输入，和快照逐值比。返回问题清单（空 = 通过）。"""
    if not os.path.exists(path):
        return [f"快照不存在：{path}（用 --update-golden 生成）"]
    with open(path, encoding="utf-8") as f:
        doc = json.load(f)
    if doc.get("kind") != "ducksir-golden":
        return [f"{path} 不是 ducksir-golden"]
    core = _load_core()
    problems = []
    for name, blob, frames in cases:
        case = doc["cases"].get(name)
        if case is None:
            problems.append(f"{name}：快照里没有这个用例")
            continue
        digest = ir.sha256_of_blob(blob)
        if case["blob_digest"] != digest:
            problems.append(f"{name}：blob 变了（{case['blob_digest'][:12]} → {digest[:12]}）——"
                            "若是有意改图，重新 --update-golden；否则说明图被改坏了")
            continue
        got = _run_case(core, blob, frames)
        want = case["out"]
        if len(got) != len(want):
            problems.append(f"{name}：帧数不同 {len(got)} vs {len(want)}")
            continue
        worst, where = 0.0, None
        for i, (g, w) in enumerate(zip(got, want)):
            for j, (gv, wv) in enumerate(zip(g, w)):
                d = abs(gv - wv)
                if d > worst:
                    worst, where = d, (i, j, gv, wv)
        if worst != 0.0:
            i, j, gv, wv = where
            problems.append(f"{name}：第 {i} 帧第 {j} 个分量差 {worst:.3e}（现 {gv:.9g} / 存 {wv:.9g}）")
        else:
            problems.append(f"{name}：OK（{len(got)} 帧逐位相同）") if False else None
    return problems


def cases_from_release(release_dir: str, frames: int = 6, seed: int = 20261110):
    """从 release 目录构造用例：每个 plan 一组确定性输入。

    FSM 的输入是结构化播种（脉冲/重力/速度），其它 plan 用固定种子的随机帧——
    随机但可复现，避免「手写输入」只覆盖到边界内的一小块。
    """
    rel = os.path.join(HERE, "..", "..", release_dir) if not os.path.isabs(release_dir) else release_dir
    rel = os.path.abspath(rel)
    with open(os.path.join(rel, "contract.json"), encoding="utf-8") as f:
        con = json.load(f)
    rng = np.random.default_rng(seed)
    cases = []
    for name, p in con["plans"].items():
        with open(os.path.join(rel, p["file"]), "rb") as f:
            blob = f.read()
        n_in = p["input_dim"]
        if name == "fsm":
            in_frames = []
            scenarios = [
                (0.0, -1.0, 0.0, 0.0),    # 站立、无指令
                (1.0, -1.0, 0.0, 0.0),    # 按一下坐/站
                (0.0, -1.0, 0.25, 0.0),   # 前进
                (0.0, -1.0, 0.25, 1.0),   # 前进 + 转向
                (0.0, -0.5, 0.0, 0.0),    # 倾倒中
                (0.0, -0.9, 0.0, 0.0),    # 接近正立
            ]
            # 每个场景重复几帧，让状态机真正走起来
            for _ in range(frames):
                for s in scenarios:
                    in_frames.append(list(s[:n_in]) + [0.0] * max(0, n_in - 4))
            cases.append((name, blob, in_frames))
        else:
            in_frames = [list(rng.uniform(-1.0, 1.0, n_in).astype(np.float64)) for _ in range(frames)]
            cases.append((name, blob, in_frames))
    return cases


if __name__ == "__main__":
    print(__doc__)
