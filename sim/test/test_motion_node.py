#!/usr/bin/env python3
"""`motion` 节点单测：C 核 ↔ numpy 参考 ↔ 逐点插值。

这个节点承载 botlab 的 `Motion.sample` / `motion_reference`：把动作参考（npz/bin 的帧序列）
变成图里的一个可采样信号。它是 4/8 个 botlab 模板都要用的能力（见 docs/notes/botlab-templates-inventory.md）。
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


class Plan:
    def __init__(self, step, info):
        self.step = step
        self.info = info


def build(motion_rows, fps, loop):
    """一张最小图：obs(1) → motion → out_action。"""
    rows = np.asarray(motion_rows, dtype=np.float32)
    dim = rows.shape[1]
    p = ir.Plan(input_dim=1, output_dim=dim, name="motion-test")
    obs = p.obs(0, 1)
    ref = p.motion(obs, rows, dim, fps, loop)
    p.out_action(ref)
    return p


def run(plan_doc, times):
    core, _ = load_libs()
    err = ctypes.create_string_buffer(512)
    blob = ir.compile_blob(plan_doc)
    h = core.ds_load(blob, len(blob), err, 512)
    if not h:
        raise SystemExit("加载失败：" + err.value.decode())
    out = []
    for t in times:
        bi = (ctypes.c_float * 1)(float(t))
        bo = (ctypes.c_float * plan_doc["output_dim"])()
        rc = core.ds_step(h, bi, bo, None)
        if rc != 0:
            raise SystemExit(f"ds_step={rc}")
        out.append([float(x) for x in bo[:]])
    core.ds_free(h)
    return out


# 一条 4 帧、2 维的动作：故意让每帧取值不同，便于抓偏移/插值错误
ROWS = [[0.0, 10.0], [1.0, 11.0], [2.0, 12.0], [3.0, 13.0]]
FPS = 10.0

for loop in (False, True):
    plan = build(ROWS, FPS, loop)
    doc = plan.to_json()
    # 取整数帧、半帧、越界（前后）几个点
    times = [0.0, 0.05, 0.1, 0.15, 0.25, 0.9, 1.5, -0.2]
    c_out = run(doc, times)
    o_out = []
    state = oracle.initial_state(doc)
    for t in times:
        val, state = oracle.step(doc, [t], state)
        o_out.append([float(x) for x in val])
    same = np.allclose(np.array(c_out), np.array(o_out), atol=1e-6)
    ok(f"核 = numpy 参考（loop={loop}）", same,
       f"{np.array(c_out)[:4].round(3).tolist()} vs {np.array(o_out)[:4].round(3).tolist()}")

# 语义检查：整数帧精确命中、半帧插值、越界夹住
plan = build(ROWS, FPS, False)
doc = plan.to_json()
vals = run(doc, [0.0, 0.1, 0.15, 10.0, -1.0])
ok("t=0 → 第 0 帧", vals[0] == [0.0, 10.0], str(vals[0]))
ok("t=0.1 → 第 1 帧", vals[1] == [1.0, 11.0], str(vals[1]))
ok("t=0.15 → 两帧中点", np.allclose(vals[2], [1.5, 11.5], atol=1e-6), str(vals[2]))
ok("t=10（越界）→ 夹在末帧", vals[3] == [3.0, 13.0], str(vals[3]))
ok("t=-1（负）→ 夹在首帧", vals[4] == [0.0, 10.0], str(vals[4]))

# 回绕：末帧之后跳回首帧
plan = build(ROWS, FPS, True)
doc = plan.to_json()
wrap = run(doc, [0.4, 0.45, 0.5])
ok("loop：t=0.4 回绕到第 0 帧", wrap[0] == [0.0, 10.0], str(wrap[0]))
# 4 帧 @10fps → 周期 0.4 s；t=0.45 等价于周期内 0.05 s，即第 0/1 帧中点
ok("loop：t=0.45 折回周期中点", np.allclose(wrap[1], [0.5, 10.5], atol=1e-6), str(wrap[1]))
ok("loop：t=0.5 回到第 1 帧", np.allclose(wrap[2], [1.0, 11.0], atol=1e-6), str(wrap[2]))

# 形状/参数错误要早失败
try:
    build(ROWS, 0, False)
    ok("fps=0 报错", False)
except ValueError:
    ok("fps=0 报错", True)

try:
    p = ir.Plan(input_dim=1, output_dim=2, name="x")
    p.obs(0, 1)
    p.motion(p.obs(0, 1), np.zeros((4, 3), dtype=np.float32), 2, FPS)
    ok("dim 与数据不符报错", False)
except ValueError:
    ok("dim 与数据不符报错", True)

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：motion 节点（采样/插值/回绕/越界/校验）")
