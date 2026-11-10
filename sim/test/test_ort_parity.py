#!/usr/bin/env python3
"""端到端对拍：C 核 + ORT 后端 vs numpy oracle + Python ORT。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/test/test_ort_parity.py

W5 的验收之一：**同一张图、同一份 ONNX、同一条 obs 序列**，两边算出的 action 逐点比。
判据 ≤1e-6（实际上两边跑的都是同一份 onnxruntime，差别只可能来自图运算的浮点顺序，
而图运算我们是逐元素、单线程、`-ffp-contract=off`，所以应当为 0）。

没有 ORT SDK / numpy / 模型就跳过（exit 0），CI 里不跑这条。
"""

from __future__ import annotations

import ctypes
import os
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))

import ir  # noqa: E402
import oracle  # noqa: E402

TOL = 1e-6
LIB_CORE = os.path.join(ROOT, "engine", "build", "libducksim.so")
LIB_ORT = os.path.join(ROOT, "engine", "build", "libducksim_ort.so")
MODEL = os.path.join(ROOT, "build", "models", "duck-hold.onnx")
OBS_DIM, N_JOINTS = 34, 14


class Info(ctypes.Structure):
    _fields_ = [("version", ctypes.c_uint16), ("abi", ctypes.c_uint16),
                ("node_count", ctypes.c_uint16), ("input_dim", ctypes.c_uint16),
                ("output_dim", ctypes.c_uint16), ("arena_len", ctypes.c_uint32),
                ("param_len", ctypes.c_uint32)]


def plan_doc() -> dict:
    p = ir.Plan(input_dim=OBS_DIM, output_dim=N_JOINTS, name="parity")
    a = p.obs(0, 3)
    g = p.obs(3, 3)
    q = p.obs(6, N_JOINTS)
    v = p.obs(20, N_JOINTS)
    c = p.concat(a, g, q, v)
    m = p.model(c, out_len=N_JOINTS, model_id=0)
    s = p.affine(m, [1.0] * N_JOINTS, [0.0] * N_JOINTS)
    k = p.clip(s, [-0.45] * N_JOINTS, [0.45] * N_JOINTS)
    p.out_action(k)
    return p.to_json()


def main() -> int:
    for path, what in ((LIB_CORE, "libducksim.so"), (LIB_ORT, "libducksim_ort.so"), (MODEL, "duck-hold.onnx")):
        if not os.path.exists(path):
            print(f"跳过：缺 {what}（见 docs/env.md：make -C engine ort && make_test_policy.py）")
            return 0
    try:
        import onnxruntime as ort_py
    except ImportError:
        print("跳过：没装 onnxruntime（见 docs/env.md）")
        return 0

    doc = plan_doc()
    blob = ir.compile_blob(doc)

    core = ctypes.CDLL(LIB_CORE)
    core.ds_load.restype = ctypes.c_void_p
    core.ds_load.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p, ctypes.c_size_t]
    core.ds_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
                             ctypes.POINTER(ctypes.c_float), ctypes.c_void_p]
    core.ds_free.argtypes = [ctypes.c_void_p]
    ort = ctypes.CDLL(LIB_ORT)
    ort.ds_ort_create.restype = ctypes.c_void_p
    ort.ds_ort_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
    ort.ds_ort_destroy.argtypes = [ctypes.c_void_p]

    err = ctypes.create_string_buffer(512)
    be = ort.ds_ort_create(MODEL.encode(), err, 512)
    if not be:
        print("ORT 后端创建失败：" + err.value.decode())
        return 1
    handle = core.ds_load(blob, len(blob), err, 512)
    if not handle:
        print("加载 plan 失败：" + err.value.decode())
        return 1

    sess = ort_py.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    in_name = sess.get_inputs()[0].name

    def py_backend(x):
        return sess.run(None, {in_name: np.asarray(x, dtype=np.float32).reshape(1, -1)})[0].reshape(-1)

    rng = np.random.default_rng(7)
    worst = 0.0
    state = None
    steps = 20
    try:
        for k in range(steps):
            obs = rng.uniform(-0.6, 0.6, OBS_DIM).astype(np.float32)
            c_in = (ctypes.c_float * OBS_DIM)(*obs.tolist())
            c_out = (ctypes.c_float * N_JOINTS)()
            rc = core.ds_step(handle, c_in, c_out, be)
            if rc != 0:
                print(f"第 {k} 步 ds_step 返回 {rc}")
                return 1
            got = np.array(c_out[:], dtype=np.float32)
            want, state = oracle.step(doc, obs, state, backend=py_backend)
            worst = max(worst, float(np.max(np.abs(got - want))))
    finally:
        core.ds_free(handle)
        ort.ds_ort_destroy(be)

    ok = worst <= TOL
    print(f"端到端对拍 {steps} 步（含 ONNX 推理）：最大偏差 {worst:.3e}  "
          f"{'PASS' if ok else 'FAIL'}（阈 {TOL}）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
