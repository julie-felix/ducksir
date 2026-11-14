#!/usr/bin/env python3
"""带历史展开的 pre 图（WBC 那种 439）——结构 + 语义两层验证。

WBC 的观测要「参考 67 + 机器人状态 4 帧 × 93」，展开发生在 pre 图里（`history` 节点），
不是靠宿主重复喂。这里验两件事：

1. **结构**：`export_artifacts.build_pre` 生成的 blob，和手搓同一张图的 blob **逐字节相同**
   （防止哪天图悄悄变了）；
2. **语义**：用 `oracle` 跑这份图，喂 160 维，得到 439 维，且
   `out[0:67] == in[0:67]`、`out[67:]` 是 4 帧状态按「最老在前」的排列。

    python3 sim/test/test_pre_history.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

import export_artifacts as E  # noqa: E402
import ir  # noqa: E402
import oracle  # noqa: E402

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


MAN = {
    "obs": [
        {"slot": "motion_ref", "len": 67},
        {"slot": "angular_velocity", "len": 3},
        {"slot": "gravity", "len": 3},
        {"slot": "joint_position", "len": 29, "minus": "default"},
        {"slot": "joint_velocity", "len": 29},
        {"slot": "last_action", "len": 29},
    ],
    "obs_history": {"offset": 67, "length": 93, "frames": 4},
}


def hand_built_plan():
    p = ir.Plan(input_dim=160, output_dim=439, name="pre")
    ref = p.obs(0, 67)
    state = p.obs(67, 93)
    hist = p.history(state, frames=4)
    p.out_action(p.concat(ref, hist))
    return p


blob, slots = E.build_pre(MAN)
mine = ir.compile_blob(hand_built_plan().to_json())

ok("slot 声明：in:obs 160 / out:model_in 439",
   slots["in:obs"] == (0, 160) and slots["out:model_in"] == (0, 439), str(slots))
ok("导出的 pre blob 与手搓同图逐字节相同", blob == mine, f"{len(blob)} vs {len(mine)} 字节")

# 语义：用 oracle 跑这份图
doc = hand_built_plan().to_json()
obs = np.arange(160, dtype=np.float32)
state = None
out = None
for _ in range(4):
    out, state = oracle.step(doc, obs, state)
out = np.asarray(out)
ok("输出 439 维", out.shape == (439,), str(out.shape))
ok("前 67 维原样透传（参考段）", np.allclose(out[:67], obs[:67]), f"差 {np.abs(out[:67] - obs[:67]).max():.1f}")
# 第 1 帧填入后，内容应出现在历史的最后一帧（最老在前 → 尾部最新）
first_state = obs[67:]
ok("历史段最后 93 维是最新一帧", np.allclose(out[-93:], first_state),
   f"差 {np.abs(out[-93:] - first_state).max():.1f}")

# 换一帧再跑：历史应有位移（旧帧前移）
obs2 = obs + 100.0
out2, _ = oracle.step(doc, obs2, state)
out2 = np.asarray(out2)
ok("第 2 帧后：历史尾部变成新帧", np.allclose(out2[-93:], obs2[67:]),
   f"差 {np.abs(out2[-93:] - obs2[67:]).max():.1f}")
ok("第 2 帧后：历史前部变成上一帧", np.allclose(out2[67:67 + 93], obs[67:]),
   f"差 {np.abs(out2[67:67 + 93] - obs[67:]).max():.1f}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：历史展开的 pre 图（结构一致 + 语义正确）")
