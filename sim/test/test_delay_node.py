#!/usr/bin/env python3
"""`delay` 节点单测：N 步延迟 + 「模型输出接回下一步」的最小递归验证。

为什么先做 delay 而不是直接做 RNN：图是 DAG，不能在一个时间步内成环。
把环用「延迟一步」拆开之后，LSTM/GRU 的
    concat(obs, h_prev, c_prev) → model → [action, h_new, c_new] → delay → h_prev
就能用现有节点拼出来，核里只需要这一个原语。
"""
import ctypes
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import ir  # noqa: E402
import oracle  # noqa: E402
from run_policy import load_libs  # noqa: E402

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


def run_core(doc, obs_seq):
    core, _ = load_libs()
    err = ctypes.create_string_buffer(512)
    blob = ir.compile_blob(doc)
    h = core.ds_load(blob, len(blob), err, 512)
    if not h:
        raise SystemExit("加载失败：" + err.value.decode())
    core.ds_reset(h)
    outs = []
    for obs in obs_seq:
        bi = (ctypes.c_float * doc["input_dim"])(*[float(v) for v in obs])
        bo = (ctypes.c_float * doc["output_dim"])()
        rc = core.ds_step(h, bi, bo, None)
        if rc != 0:
            raise SystemExit(f"ds_step={rc}")
        outs.append([float(x) for x in bo[:]])
    core.ds_free(h)
    return np.array(outs, dtype=np.float32)


def run_oracle(doc, obs_seq):
    state = oracle.initial_state(doc)
    outs = []
    for obs in obs_seq:
        val, state = oracle.step(doc, list(obs), state)
        outs.append([float(x) for x in val])
    return np.array(outs, dtype=np.float32)


# ── 1) 纯延迟：3 步，fill=0.5 ────────────────────────────────────────────
p = ir.Plan(input_dim=1, output_dim=1, name="delay-basic")
x = p.obs(0, 1)
d = p.delay(x, frames=3, fill=[0.5])
p.out_action(d)
doc = p.to_json()
seq = [[1.0], [2.0], [3.0], [4.0], [5.0]]
got = run_core(doc, seq)[:, 0]
want = np.array([0.5, 0.5, 0.5, 1.0, 2.0], dtype=np.float32)
ok("3 步延迟：前 3 步输出 fill，之后输出 3 步前的值", np.allclose(got, want), f"{got} vs {want}")
ok("核 = oracle（delay）", np.allclose(got, run_oracle(doc, seq)[:, 0]), "")

# ── 2) 1 步延迟 ─────────────────────────────────────────────────────────
p = ir.Plan(input_dim=1, output_dim=1, name="delay-1")
x = p.obs(0, 1)
p.out_action(p.delay(x, frames=1))
doc = p.to_json()
got = run_core(doc, seq)[:, 0]
ok("1 步延迟：0, x0, x1, x2, x3", np.allclose(got, [0, 1, 2, 3, 4]), f"{got}")

# ── 3) 延迟输入 + 加权（移动平均，不是递归）────────────────────────────────
# 注意：这里延迟的是**输入** x，不是输出 y。
# 真正把「模型输出喂回下一步」需要读/写分离，或让 model 节点自带状态槽——见文件末尾说明。
p = ir.Plan(input_dim=1, output_dim=1, name="avg")
x = p.obs(0, 1)
prev = p.delay(x, frames=1)
p.out_action(p.add(p.affine(prev, [0.5], [0.0]), p.affine(x, [0.5], [0.0])))
doc = p.to_json()
seq = [[1.0], [2.0], [3.0], [4.0], [5.0]]
got = run_core(doc, seq)[:, 0]
# y_k = 0.5*x_{k-1} + 0.5*x_k，x 从 1 开始 → 0.5, 1.5, 2.5, 3.5, 4.5
want = [0.5, 1.5, 2.5, 3.5, 4.5]
ok("延迟输入 + 加权（移动平均）与解析解一致", np.allclose(got, want, atol=1e-6), f"{np.round(got, 4)}")
ok("核 = oracle（移动平均）", np.allclose(got, run_oracle(doc, seq)[:, 0]), "")

# ── 4) reset 语义：重放同一段要得到同一串 ────────────────────────────────
core, _ = load_libs()
err = ctypes.create_string_buffer(512)
blob = ir.compile_blob(doc)


def play(handle):
    core.ds_reset(handle)
    out = []
    for obs in seq:
        bi = (ctypes.c_float * 1)(float(obs[0]))
        bo = (ctypes.c_float * 1)()
        core.ds_step(handle, bi, bo, None)
        out.append(float(bo[0]))
    return out


h1 = core.ds_load(blob, len(blob), err, 512)
first = play(h1)
second = play(h1)
core.ds_free(h1)
ok("ds_reset 后重放逐位相同", first == second, f"{np.round(first, 4)}")

# ── 5) 参数校验 ────────────────────────────────────────────────────────
try:
    p = ir.Plan(input_dim=1, output_dim=2, name="bad")
    x = p.obs(0, 1)
    p.delay(x, frames=0)
    ok("frames=0 报错", False)
except ValueError:
    ok("frames=0 报错", True)

try:
    p = ir.Plan(input_dim=1, output_dim=2, name="bad2")
    x = p.obs(0, 1)
    p.delay(x, frames=1, fill=[0.0, 0.0])
    ok("fill 宽度不符报错", False)
except ValueError:
    ok("fill 宽度不符报错", True)

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：delay 节点（N 步延迟 / 延迟输入加权 / reset / 校验）")
print()
print("说明：这个节点只能延迟**前序**节点的值。要把 LSTM/GRU 的 h_new 接回同一张图的 h_prev，")
print("同一个时间步内会成环（DAG 不允许），还要再做一步：模型节点自带状态槽，或读写分离。")
