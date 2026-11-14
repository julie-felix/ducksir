#!/usr/bin/env python3
"""loco（mode 2）与 botlab 的一步重放对拍：判据 1e-5（float32 精度内）。

需要两份东西（都是本地采的，不进仓库）：

- `/tmp/theirs-loco.jsonl`：botlab 自己的运行时按 mode 2 跑出来的 per-step trace
  （`tools/dev/botlab_record_native.py` + `patch_trace.py`，120 步即可）；
- `build/release/g1-loco`：`export_artifacts.py` 用 `contract/policy/g1-loco.manifest.yaml` 导出。

做法：对每一步，把他们的 qpos/qvel 装进我们的 sim，喂他们录的动作算 target
（`defaultJointPositions + 0.25×action[motor_order]`），按我们的 PD 走 4 个子步，
和他们**下一步**的 qpos 比。

    python3 sim/test/test_loco_parity.py
"""
import json
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))

TRACE = "/tmp/theirs-loco.jsonl"
BUNDLE = os.path.join(ROOT, "build", "release", "g1-loco")

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


if not os.path.exists(TRACE):
    print(f"  跳过：没有 {TRACE}（先用 botlab 录一份 mode 2 的 trace）")
    raise SystemExit(0)
if not os.path.exists(os.path.join(BUNDLE, "contract.json")):
    print("  跳过：没有 build/release/g1-loco")
    raise SystemExit(0)

import mujoco  # noqa: E402
import actuators as A  # noqa: E402

contract = json.load(open(os.path.join(BUNDLE, "contract.json"), encoding="utf-8"))
preset = yaml.safe_load(open(os.path.join(BUNDLE, "preset.yaml"), encoding="utf-8"))
DQ = np.asarray(preset["defaultJointPositions"])
MAP = preset["motorJointOrder"]
# 注意：这里刻意用 **botlab native 路径实际用的 robot.kp/kd**（他们的 trace 就是这么跑的），
# 不是我们 bundle 里的 balanceStiffness/Damping。两者差在增益：
#   - native 跑：kp=99/99/40…（robot.kp）→ 2 s 左右倒（他们的 trace 也是这样）
#   - plan 的 mode 2 分支：kp=balanceStiffness（200/150…）→ 站得住，浏览器用的是这一支
# 本测试只对拍「目标公式 + 物理」，所以增益要跟 trace 对齐。
_src = yaml.safe_load(open("/tmp/bt-all/G1-WBC-Multi-Policy/g1-wbc.preset.yaml", encoding="utf-8"))
KPS = np.asarray(_src["kp"])
KDS = np.asarray(_src["kd"])
EFF = np.asarray(preset["effortLimit"])
trace = [json.loads(line) for line in open(TRACE, encoding="utf-8") if line.strip()]

spec = mujoco.MjSpec.from_file(os.path.join(BUNDLE, "robot.xml"))
plane = spec.worldbody.add_geom()
plane.type = mujoco.mjtGeom.mjGEOM_PLANE
plane.size = [6, 6, 0.1]
plane.condim = 3
model = spec.compile()
data = mujoco.MjData(model)
model.opt.timestep = 1.0 / preset.get("physicsHz", 200)
jnt = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j) for j in contract["robot"]["joints"]]
qoff = [model.jnt_qposadr[k] for k in jnt]
doff = [model.jnt_dofadr[k] for k in jnt]

errs = []
for k in range(1, min(60, len(trace) - 1)):
    a, b = trace[k], trace[k + 1]
    if not a.get("qpos") or not b.get("qpos"):
        continue
    data.qpos[:] = a["qpos"]
    data.qvel[:] = a["qvel"]
    mujoco.mj_forward(model, data)
    target = DQ.copy()
    for i in range(29):
        target[MAP[i]] = DQ[MAP[i]] + 0.25 * float(a["out"]["actions"][i])
    for _ in range(4):
        q = np.array([data.qpos[i] for i in qoff])
        qv = np.array([data.qvel[i] for i in doff])
        data.ctrl[:] = A.torque_from_target(target, q, qv, kp=KPS, kd=KDS, effort_limit=EFF)
        mujoco.mj_step(model, data)
    errs.append(float(np.abs(data.qpos - np.asarray(b["qpos"])).max()))

ok("有足够的对拍步数", len(errs) > 20, f"{len(errs)} 步")
ok("一步重放误差 < 1e-5", max(errs) < 1e-5, f"平均 {np.mean(errs):.2e}，最大 {max(errs):.2e}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：loco 与 botlab 的一步重放一致")
