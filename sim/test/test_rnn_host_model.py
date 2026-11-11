#!/usr/bin/env python3
"""`model_rnn` 接**真实**多张量 ONNX 模型的端到端验证。

用 botlab 下载包里的 G1-WBC `loco_0731.onnx`：
    obs[1,96] + h_in[1,1,256] + c_in[1,1,256]  →  actions[1,29] + h_out[1,1,256] + c_out[1,1,256]
这正是 `model_rnn` 的 `in=[obs,h,c] → out=[action,h_new,c_new]` 约定。

核只送一条 608 长的输入、只取一条 541 长的输出；ORT 后端按模型声明的张量切分/合并。
没下载模板包时自动跳过（不假装验过）。
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
from run_policy import load_libs  # noqa: E402

MODEL = "/tmp/bt-all/G1-WBC-Multi-Policy/assets/models/loco_0731.onnx"
OBS, ACT, H, C = 96, 29, 256, 256

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


if not os.path.exists(MODEL):
    print("  跳过：没找到 loco_0731.onnx（先跑 tools/dev/grab_botlab_templates.mjs 并解压）")
    print("PASS（跳过）：多张量模型端到端未验")
    raise SystemExit(0)

p = ir.Plan(input_dim=OBS, output_dim=ACT, name="wbc-loco")
obs = p.obs(0, OBS)
p.out_action(p.model_rnn(obs, ACT, h_len=H, c_len=C))
doc = p.to_json()

core, ort = load_libs()
err = ctypes.create_string_buffer(1024)
be = ort.ds_ort_create(MODEL.encode(), err, 1024)
if not be:
    print("  FAIL  加载模型：" + err.value.decode()[:200])
    raise SystemExit(1)
blob = ir.compile_blob(doc)
h = core.ds_load(blob, len(blob), err, 1024)
if not h:
    print("  FAIL  加载图：" + err.value.decode()[:200])
    raise SystemExit(1)
core.ds_reset(h)

ACTIONS = []


def step(obs_vec):
    bi = (ctypes.c_float * OBS)(*[float(x) for x in obs_vec])
    bo = (ctypes.c_float * ACT)()
    rc = core.ds_step(h, bi, bo, be)
    return rc, np.array(bo[:], dtype=np.float32)


zeros = np.zeros(OBS, dtype=np.float32)
rc0, a0 = step(zeros)
ok("多张量模型能跑通（obs+h+c 一个张量送进去）", rc0 == 0, f"rc={rc0}")
ok("输出 29 维、有限", a0.shape == (ACT,) and np.isfinite(a0).all(),
   f"范围 [{a0.min():.3f}, {a0.max():.3f}]")

# 同一个 obs 再来一次：若状态真的回灌了，输出应该变（h/c 已经不是零）
rc1, a1 = step(zeros)
ok("状态确实跨步（同一 obs 两次输出不同）", rc1 == 0 and not np.allclose(a0, a1),
   f"|Δ|={np.abs(a1 - a0).max():.4f}")

# 连跑 50 步不能发散/NaN
bad = 0
for _ in range(50):
    rc, a = step(zeros)
    if rc != 0 or not np.isfinite(a).all():
        bad += 1
ok("连跑 50 步无 NaN/报错", bad == 0, f"bad={bad}")

# reset 后重放第一步应与最初一致（状态清零）
core.ds_reset(h)
_, a_reset = step(zeros)
ok("ds_reset 后第一步与最初逐位相同", np.allclose(a_reset, a0, atol=0),
   f"max|Δ|={np.abs(a_reset - a0).max():.2e}")

core.ds_free(h)
ort.ds_ort_destroy(be)

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：model_rnn 接真实多张量模型（WBC loco_0731）")
