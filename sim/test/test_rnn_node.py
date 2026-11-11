#!/usr/bin/env python3
"""`model_rnn` 节点单测：带状态槽的模型（LSTM/GRU）在核里怎么跑。

约定：`in = [obs, h, c]`，`out = [action, h_new, c_new]`，h/c 存在 plan 里、跨步保持、`ds_reset` 清零。

测试用「假后端」实现一条可解析的递推，逐点核对：
    h_new = 0.5 * h + obs
    c_new = c + obs
    action = h_new
于是第 k 步 action 应等于 sum_{j<=k} obs_j * 0.5^(k-j)。
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


H, C = 2, 1
OBS = 1


def fake_backend(packed):
    """Python 侧参考实现：in=[obs,h,c] → out=[action,h_new,c_new]。"""
    obs = float(packed[0])
    h = np.asarray(packed[1:1 + H], dtype=np.float32)
    c = np.asarray(packed[1 + H:1 + H + C], dtype=np.float32)
    h_new = h * np.float32(0.5) + obs
    c_new = c + obs
    return np.concatenate([[h_new[0]], h_new, c_new]).astype(np.float32)


def c_backend(packed):
    return np.asarray(fake_backend(packed), dtype=np.float32).tolist()


CB = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
                      ctypes.POINTER(ctypes.c_float), ctypes.c_int,
                      ctypes.POINTER(ctypes.c_float), ctypes.c_int)


@CB
def _cb(_ctx, _model_id, in_ptr, in_len, out_ptr, out_len):
    packed = [in_ptr[i] for i in range(in_len)]
    res = c_backend(packed)
    if len(res) != out_len:
        return 1
    for i, v in enumerate(res):
        out_ptr[i] = v
    return 0


class Backend(ctypes.Structure):
    _fields_ = [("ctx", ctypes.c_void_p), ("onnx_run", CB)]


def build():
    p = ir.Plan(input_dim=OBS, output_dim=1, name="rnn-test")
    obs = p.obs(0, OBS)
    p.out_action(p.model_rnn(obs, 1, h_len=H, c_len=C))
    return p


def run_core(doc, seq):
    core, _ = load_libs()
    err = ctypes.create_string_buffer(512)
    blob = ir.compile_blob(doc)
    h = core.ds_load(blob, len(blob), err, 512)
    if not h:
        raise SystemExit("加载失败：" + err.value.decode())
    core.ds_reset(h)
    be = Backend(onnx_run=_cb)
    outs = []
    for x in seq:
        bi = (ctypes.c_float * OBS)(float(x))
        bo = (ctypes.c_float * 1)()
        rc = core.ds_step(h, bi, bo, ctypes.byref(be))
        if rc != 0:
            raise SystemExit(f"ds_step={rc}")
        outs.append(float(bo[0]))
    core.ds_free(h)
    return np.array(outs, dtype=np.float32)


doc = build().to_json()
seq = [1.0, 2.0, 3.0, 4.0, 5.0]
got = run_core(doc, seq)
want = []
acc = 0.0
for x in seq:
    acc = 0.5 * acc + x
    want.append(acc)
ok("递归策略：action = 0.5*h_prev + obs（跨步保持）", np.allclose(got, want), f"{np.round(got,4)} vs {want}")

# 与 oracle 对拍（同一个假后端）
state = oracle.initial_state(doc)
o_out = []
for x in seq:
    val, state = oracle.step(doc, [x], state, backend=fake_backend)
    o_out.append(float(val[0]))
ok("核 = oracle（model_rnn）", np.allclose(got, o_out), f"{np.round(o_out,4)}")

# 状态确实在累积：第 k 步依赖历史
ok("第 1 步 = obs0 + 0.5*0（不是只看当前 obs）", abs(got[1] - seq[1]) > 1e-6,
   f"step1={got[1]}（若状态没保持则为 {seq[1]}）")

# reset 后重放一致
core, _ = load_libs()
err = ctypes.create_string_buffer(512)
blob = ir.compile_blob(doc)


def replay():
    h = core.ds_load(blob, len(blob), err, 512)
    core.ds_reset(h)
    be = Backend(onnx_run=_cb)
    out = []
    for x in seq:
        bi = (ctypes.c_float * OBS)(float(x))
        bo = (ctypes.c_float * 1)()
        core.ds_step(h, bi, bo, ctypes.byref(be))
        out.append(float(bo[0]))
    core.ds_free(h)
    return out


ok("ds_reset 后重放逐位相同", replay() == replay(), f"{np.round(replay(), 4)}")

# 没给后端要明确报错
h = core.ds_load(blob, len(blob), err, 512)
core.ds_reset(h)
bi = (ctypes.c_float * OBS)(1.0)
bo = (ctypes.c_float * 1)()
rc = core.ds_step(h, bi, bo, None)
core.ds_free(h)
ok("没后端 → DS_ERR_NO_BACKEND", rc == -4, f"rc={rc}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：model_rnn 节点（状态保持 / 对拍 / reset / 无后端报错）")
