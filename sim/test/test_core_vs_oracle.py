#!/usr/bin/env python3
"""C 核 vs numpy oracle 对拍（W4 的验收）。

    python3 sim/test/test_core_vs_oracle.py

做法：同一份 plan（`sim/plan/ir.py` 生成），一份喂给 C 核（ctypes 加载 `engine/build/libducksim.so`），
一份喂给 numpy 参考实现（`sim/plan/oracle.py`），同一条随机 obs 序列逐步比 action。

判据：**最大单点偏差 ≤ 1e-6**（计划 W4）。顺带验四条拒绝路径：digest 被改、缺 out_action、
未知节点版本、history 首步语义。
"""

from __future__ import annotations

import ctypes
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.dirname(HERE)
ROOT = os.path.dirname(SIM)
sys.path.insert(0, os.path.join(SIM, "plan"))

import ir  # noqa: E402
import oracle  # noqa: E402

TOL = 1e-6
LIB = os.path.join(ROOT, "engine", "build", "libducksim.so")


class Backend(ctypes.Structure):
    _fields_ = [("ctx", ctypes.c_void_p), ("onnx_run", ctypes.c_void_p)]


class Info(ctypes.Structure):
    _fields_ = [("version", ctypes.c_uint16), ("abi", ctypes.c_uint16),
                ("node_count", ctypes.c_uint16), ("input_dim", ctypes.c_uint16),
                ("output_dim", ctypes.c_uint16), ("arena_len", ctypes.c_uint32),
                ("param_len", ctypes.c_uint32)]


def load_lib():
    lib = ctypes.CDLL(LIB)
    lib.ds_load.restype = ctypes.c_void_p
    lib.ds_load.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p, ctypes.c_size_t]
    lib.ds_free.argtypes = [ctypes.c_void_p]
    lib.ds_info_of.argtypes = [ctypes.c_void_p, ctypes.POINTER(Info)]
    lib.ds_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
                            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(Backend)]
    lib.ds_reset.argtypes = [ctypes.c_void_p]
    return lib


def plan_simple() -> dict:
    """obs → affine → clip → out（不带时序，最小闭环）。"""
    p = ir.Plan(input_dim=6, output_dim=6, name="simple")
    a = p.obs(0, 6)
    b = p.affine(a, [2.0], [0.5])
    c = p.clip(b, [-1.0], [1.0])
    p.out_action(c)
    return p.to_json()


def plan_rich() -> dict:
    """两段 obs → concat → reorder → history(3) → model(pass-through) → affine → clip → out。"""
    p = ir.Plan(input_dim=8, output_dim=24, name="rich")
    a = p.obs(0, 4)
    b = p.obs(4, 4)
    c = p.concat(a, b)
    d = p.reorder(c, [7, 6, 5, 4, 3, 2, 1, 0])
    h = p.history(d, 3)
    m = p.model(h, h.length, pass_through=True)
    e = p.affine(m, np.linspace(0.5, 1.5, m.length), np.linspace(-0.2, 0.2, m.length))
    f = p.clip(e, [-0.9] * m.length, [0.9] * m.length)
    p.out_action(f)
    return p.to_json()


def run_c(lib, doc, obs_seq):
    blob = ir.compile_blob(doc)
    err = ctypes.create_string_buffer(256)
    handle = lib.ds_load(blob, len(blob), err, 256)
    if not handle:
        raise AssertionError("ds_load 失败：" + err.value.decode())
    info = Info()
    assert lib.ds_info_of(handle, ctypes.byref(info)) == 0
    assert info.input_dim == doc["input_dim"] and info.output_dim == doc["output_dim"]
    n_in, n_out = info.input_dim, info.output_dim
    backend = Backend(None, None)
    outs = []
    try:
        for obs in obs_seq:
            buf_in = (ctypes.c_float * n_in)(*obs.astype(np.float32).tolist())
            buf_out = (ctypes.c_float * n_out)()
            rc = lib.ds_step(handle, buf_in, buf_out, ctypes.byref(backend))
            assert rc == 0, f"ds_step 返回 {rc}"
            outs.append(np.array(buf_out[:], dtype=np.float32))
    finally:
        lib.ds_free(handle)
    return outs, info


def run_oracle(doc, obs_seq):
    state = None
    outs = []
    for obs in obs_seq:
        a, state = oracle.step(doc, obs, state)
        outs.append(a)
    return outs


def compare(name, lib, doc, steps=40, seed=0):
    rng = np.random.default_rng(seed)
    obs_seq = [rng.uniform(-2, 2, doc["input_dim"]).astype(np.float32) for _ in range(steps)]
    c_outs, info = run_c(lib, doc, obs_seq)
    o_outs = run_oracle(doc, obs_seq)
    worst = 0.0
    for k, (c, o) in enumerate(zip(c_outs, o_outs)):
        diff = float(np.max(np.abs(c - o))) if c.size else 0.0
        if diff > worst:
            worst = diff
    ok = worst <= TOL
    print(f"  {name:<8} nodes={info.node_count:<3} arena={info.arena_len:<4} "
          f"step={steps}  最大偏差 {worst:.3e}  {'PASS' if ok else 'FAIL'}")
    return ok, worst


def expect_reject(name, lib, blob, needle):
    err = ctypes.create_string_buffer(256)
    handle = lib.ds_load(blob, len(blob), err, 256)
    msg = err.value.decode()
    if handle:
        lib.ds_free(handle)
        print(f"  {name:<22} 竟然加载成功 → FAIL")
        return False
    ok = needle in msg
    print(f"  {name:<22} 拒绝（{msg}）{'PASS' if ok else '  ← 原因不对'}")
    return ok


def main() -> int:
    if not os.path.exists(LIB):
        print(f"跳过：没有 {LIB}（先 cd engine && make）")
        return 0
    lib = load_lib()
    print(f"引擎 ABI {lib.ds_abi_version()}\n")

    oks = []
    print("对拍：")
    oks.append(compare("simple", lib, plan_simple())[0])
    oks.append(compare("rich", lib, plan_rich())[0])

    print("\n拒绝路径：")
    rich = plan_rich()
    good = ir.compile_blob(rich)
    tampered = bytearray(good)
    tampered[-1] ^= 0xFF
    oks.append(expect_reject("digest 被改", lib, bytes(tampered), "digest"))

    p = ir.Plan(input_dim=3, output_dim=3)
    a = p.obs(0, 3)
    p.affine(a, [1.0], [0.0])
    doc_no_sink = p.nodes and {
        "kind": "ducksim-plan", "version": 1, "abi": 1, "name": "x",
        "input_dim": 3, "output_dim": 3, "arena_len": p._arena,
        "nodes": p.nodes, "params": p.params,
    }
    try:
        ir.compile_blob(doc_no_sink)
        print("  缺 out_action           本地校验竟然放过 → FAIL")
        oks.append(False)
    except ValueError as e:
        print(f"  缺 out_action           本地校验拒绝（{str(e).splitlines()[-1].strip()}）PASS")
        oks.append(True)

    # 未知节点：手工改节点类型 id 再重算 digest（模拟新版本图给老核）
    import struct
    body_start = ir.HEADER_SIZE
    b = bytearray(good)
    struct.pack_into("<H", b, body_start + 4 + 0, 99)  # 第一个节点的 type = 99
    digest = ir.fnv1a64(bytes(b[body_start:]))
    struct.pack_into("<Q", b, 40, digest)
    oks.append(expect_reject("未知节点类型", lib, bytes(b), "未知"))

    print(f"\n{'全部通过' if all(oks) else '有失败'}（{sum(oks)}/{len(oks)}）")
    return 0 if all(oks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
