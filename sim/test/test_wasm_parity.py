#!/usr/bin/env python3
"""native vs wasm 对拍：同一份 C++ 源码、同一张图、同一串 obs，问「两边算出的 action 一样吗」。

    python3 sim/test/test_wasm_parity.py

做法：用 ctypes 跑 native 核，用 node 跑 emcc 编出来的 wasm（`engine/wasm/harness.mjs`），
逐点比 action。判据 **0 容差**——不是 1e-6，是必须逐位相同（`-ffp-contract=off` + 单线程 + 定点偏移）。

这条是 botlab 那条路最该有却做不到的东西：它两份引擎（TS/Python）靠 golden trace 对拍，
而那份测试在开源包里默认 skip；我们是一份 C++ 走两条编译目标，所以能直接比。

没有 emscripten 产物 / node / numpy 就跳过（exit 0）。
"""

from __future__ import annotations

import ctypes
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))

import ir  # noqa: E402

LIB = os.path.join(ROOT, "engine", "build", "libducksim.so")
WASM_JS = os.path.join(ROOT, "engine", "build", "wasm", "ducksim.mjs")
HARNESS = os.path.join(ROOT, "engine", "wasm", "harness.mjs")


def plan_doc() -> dict:
    """带一点算术与时序的图；model 节点走 pass-through（wasm 端不链 ORT）。"""
    p = ir.Plan(input_dim=10, output_dim=26, name="wasm-parity")
    a = p.obs(0, 4)
    b = p.obs(4, 4)
    mode = p.obs(8, 1)
    steps = p.obs(9, 1)
    c = p.concat(a, b)
    d = p.affine(c, [1.5, -0.5, 2.0, 0.25, 0.75, -1.25, 0.5, 1.0], [0.1, 0.2, -0.3, 0.4, 0.0, 0.1, -0.2, 0.3])
    e = p.clip(d, [-0.8] * 8, [0.8] * 8)
    h = p.history(e, 3)
    m = p.model(h, h.length, pass_through=True)
    ph = p.phase_clock(steps, 12)
    k = p.constant([2.0])
    sel = p.select(mode, {0.0: m, 1.0: m})   # 两条路相同；只是把 select 也带进 wasm
    out = p.concat(sel, ph, k)
    p.out_action(out)
    return p.to_json()


def run_native(doc: dict, obs_seq) -> tuple[list, dict]:
    lib = ctypes.CDLL(LIB)
    lib.ds_load.restype = ctypes.c_void_p
    lib.ds_load.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p, ctypes.c_size_t]
    lib.ds_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
                            ctypes.POINTER(ctypes.c_float), ctypes.c_void_p]
    lib.ds_free.argtypes = [ctypes.c_void_p]
    blob = ir.compile_blob(doc)
    err = ctypes.create_string_buffer(256)
    h = lib.ds_load(blob, len(blob), err, 256)
    if not h:
        raise AssertionError("native ds_load 失败：" + err.value.decode())
    n_in, n_out = doc["input_dim"], doc["output_dim"]
    outs = []
    try:
        for obs in obs_seq:
            bi = (ctypes.c_float * n_in)(*obs.tolist())
            bo = (ctypes.c_float * n_out)()
            rc = lib.ds_step(h, bi, bo, None)
            if rc != 0:
                raise AssertionError(f"native ds_step 返回 {rc}")
            outs.append(np.array(bo[:], dtype=np.float32))
    finally:
        lib.ds_free(h)
    return outs, blob


def run_wasm(blob: bytes, obs_seq) -> list:
    with tempfile.TemporaryDirectory() as td:
        bp = os.path.join(td, "plan.blob")
        op = os.path.join(td, "obs.json")
        with open(bp, "wb") as f:
            f.write(blob)
        with open(op, "w", encoding="utf-8") as f:
            json.dump([list(map(float, o)) for o in obs_seq], f)
        r = subprocess.run(["node", HARNESS, bp, op], capture_output=True, text=True)
        if r.returncode != 0:
            raise AssertionError("wasm 跑失败：" + (r.stderr or r.stdout)[-300:])
        data = json.loads(r.stdout)
    return [np.array(a, dtype=np.float32) for a in data["actions"]], data


def main() -> int:
    for path, what in ((LIB, "libducksim.so"), (WASM_JS, "engine/build/wasm/ducksim.mjs")):
        if not os.path.exists(path):
            print(f"跳过：缺 {what}（make -C engine && make -C engine wasm）")
            return 0
    if not os.path.exists(HARNESS):
        print("跳过：缺 engine/wasm/harness.mjs")
        return 0

    doc = plan_doc()
    rng = np.random.default_rng(11)
    obs_seq = []
    for k in range(30):
        v = rng.uniform(-1.5, 1.5, doc["input_dim"]).astype(np.float32)
        v[8] = float(k % 2)          # select 的两个分支都走
        v[9] = float(k * 3)          # phase_clock 会跨周期
        obs_seq.append(v)

    native, blob = run_native(doc, obs_seq)
    wasm, meta = run_wasm(blob, obs_seq)

    print(f"native {len(native)} 步｜wasm {len(wasm)} 步｜节点 {meta['nodes']}｜"
          f"{meta['input_dim']} → {meta['output_dim']}")
    worst, where = 0.0, -1
    for k, (a, b) in enumerate(zip(native, wasm)):
        d = float(np.max(np.abs(a - b)))
        if d > worst:
            worst, where = d, k
    ok = worst == 0.0
    print(f"最大偏差 {worst:.3e}（第 {where} 步）  {'PASS 逐位相同' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
