#!/usr/bin/env python3
"""Instinct 深度/观测与 botlab 真值对拍（需要本地录制的 trace，缺了自动跳过）。

真值：`/tmp/inst.jsonl`（`patch_inst.py` 录的 obs + qpos/qvel，来自 botlab 自己的运行时）。
判据：
- 深度段（归一化到 [0,1]）残差 < 0.15、相关 > 0.99；
- 前 670 段（jp/jv/pos/rot）残差 < 5e-3。

    MUJOCO_GL=egl python3 sim/test/test_instinct_depth.py
"""
import json
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
sys.path.insert(0, "/tmp/inst-run")

TRACE = "/tmp/inst.jsonl"
TPL = "/tmp/inst-run"

fails = []


def ok(name, cond, detail=""):
    print(("  PASS  " if cond else "  FAIL  ") + name + (f"  {detail}" if detail else ""))
    if not cond:
        fails.append(name)


if not os.path.exists(TRACE) or not os.path.exists(TPL):
    print(f"  跳过：没有 {TRACE} 或 {TPL}")
    raise SystemExit(0)

import mujoco  # noqa: E402
import depth_camera as D  # noqa: E402
import instinct_obs as I  # noqa: E402
import motion_choices as mc  # noqa: E402

cfg = mc.CONFIGS[mc.DEFAULT_MOTION]
z = np.load(f"/tmp/bt-all/Instinct-SuperMimic/assets/motions/case_{cfg['case']:02d}.npz",
            allow_pickle=True)
motion = {k: z[k] for k in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w")}
motion["fps"] = float(z["fps"][0])
dof = [str(x) for x in z["dof_names"]]
joint_order = [dof.index(n) for n in cfg["joint_names"]]
torso_idx = [str(b) for b in z["body_names"]].index("torso_link")

preset = yaml.safe_load(open(os.path.join(TPL, "robot.preset.yaml"), encoding="utf-8"))
DQ = np.asarray(preset["defaultJointPositions"])
cam_spec = yaml.safe_load(open(os.path.join(TPL, "camera.preset.yaml"), encoding="utf-8"))

spec = mujoco.MjSpec.from_file(os.path.join(TPL, "assets/robot/scene.xml"))
spec.meshdir = os.path.join(TPL, "assets/robot/meshes")
model = spec.compile()
data = mujoco.MjData(model)
cam = D.DepthCamera(model, cam_spec)
torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")

trace = [json.loads(line) for line in open(TRACE, encoding="utf-8") if line.strip()]
K = 10
row = trace[K]
theirs = np.asarray(row["in"]["obs"])
data.qpos[:] = row["qpos"]
data.qvel[:] = row["qvel"]
mujoco.mj_forward(model, data)

w, x, y, zq = data.xquat[torso_id]
R = np.array([
    [1 - 2 * (y * y + zq * zq), 2 * (x * y - zq * w), 2 * (x * zq + y * w)],
    [2 * (x * y + zq * w), 1 - 2 * (x * x + zq * zq), 2 * (y * zq - x * w)],
    [2 * (x * zq - y * w), 2 * (y * zq + x * w), 1 - 2 * (x * x + y * y)]])

block, terms = I.build_terms(
    motion=motion, t=(K - 1) * 0.02, duration=(cfg["frames"] - 0.5) / 50,
    joint_order=joint_order, config_defaults=cfg["default_angles"],
    robot_joint_pos=np.asarray(row["qpos"][7:36]) - DQ,
    robot_joint_vel=np.asarray(row["qvel"][6:35]),
    gravity_b=R.T @ np.array([0, 0, -1.0]), base_ang_vel=np.asarray(row["qvel"][3:6]),
    last_action=np.asarray(trace[K - 1]["out"]["actions"]),
    torso_index=torso_idx, robot_torso_quat=data.xquat[torso_id].copy())

ok("本体段（jp/jv/pos/rot）残差 < 5e-3",
   float(np.abs(block - theirs[:670]).max()) < 5e-3,
   f"{float(np.abs(block - theirs[:670]).max()):.2e}")

depth = cam.process(cam.render(data)).reshape(-1)
dmax = float(np.abs(depth - theirs[670:1246]).max())
corr = float(np.corrcoef(depth, theirs[670:1246])[0, 1])
ok("深度残差 < 0.15", dmax < 0.15, f"{dmax:.4f}")
ok("深度相关 > 0.99", corr > 0.99, f"{corr:.4f}")

print()
if fails:
    print(f"失败 {len(fails)} 项：" + "、".join(fails))
    raise SystemExit(1)
print("PASS：Instinct 观测（含深度）与 botlab 一致")
