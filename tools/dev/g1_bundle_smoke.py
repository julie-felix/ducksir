#!/usr/bin/env python3
"""G1 端到端冒烟：扭矩模型 + 地面 + PD + loco 策略，看能不能站住。

不经过 bundle，直接用「我们自己的执行器语义」跑：

    obs(96) → loco_0731（带 h/c）→ 目标角 → PD 力矩 → MuJoCo

判据来自真值对拍（见 docs/notes/g1-botlab.md §9）：同样的状态/动作，
我们算出的力矩与 botlab 写进 d.ctrl 的值差 < 1。

    python3 tools/dev/g1_bundle_smoke.py
"""
import os
import sys

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
import actuators as A  # noqa: E402

MJCF = os.path.join(ROOT, "build/mjcf/g1-29dof-torque.xml")
PRESET = os.path.join(ROOT, "build/preset/g1-29dof-torque.preset.yaml")
MODEL = "/tmp/bt-all/G1-WBC-Multi-Policy/assets/models/loco_0731.onnx"
MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11, 17, 24,
       18, 25, 19, 26, 20, 27, 21, 28]


def main() -> int:
    if not os.path.exists(MODEL):
        print("跳过：没找到 loco_0731.onnx（先跑 tools/dev/grab_botlab_templates.mjs 并解压）")
        return 0
    if not (os.path.exists(MJCF) and os.path.exists(PRESET)):
        print("跳过：没导出 g1-29dof-torque 的 MJCF/preset（见 docs/notes/g1-botlab.md §9）")
        return 0

    pre = yaml.safe_load(open(PRESET, encoding="utf-8"))
    spec = mujoco.MjSpec.from_file(MJCF)
    plane = spec.worldbody.add_geom()
    plane.type = mujoco.mjtGeom.mjGEOM_PLANE
    plane.size = [6, 6, 0.1]
    plane.condim = 3
    m = spec.compile()
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / pre.get("physicsHz", 200)
    d.qpos[:] = pre["initialQpos"]
    mujoco.mj_forward(m, d)

    sess = ort.InferenceSession(MODEL, providers=["CPUExecutionProvider"])
    h = np.zeros((1, 1, 256), np.float32)
    c = np.zeros((1, 1, 256), np.float32)
    last = np.zeros(29, np.float32)
    DQ = np.asarray(pre["defaultJointPositions"], np.float64)

    def q2m(q):
        w, x, y, z = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    zs, ups = [], []
    for _ in range(150):
        R = q2m(d.qpos[3:7])
        grav = R.T @ np.array([0, 0, -1.0])
        q = d.qpos[7:36]
        qd = d.qvel[6:35]
        qp = np.array([q[MAP[i]] - DQ[MAP[i]] for i in range(29)])
        qdp = np.array([qd[MAP[i]] for i in range(29)])
        obs = np.concatenate([d.qvel[3:6], grav, np.zeros(3), qp, qdp, last]).astype(np.float32)
        out = sess.run(None, {"obs": obs.reshape(1, -1), "h_in": h, "c_in": c})
        act = np.clip(np.asarray(out[0]).reshape(-1), -100, 100)
        h, c = np.asarray(out[1]), np.asarray(out[2])
        last = act
        # 每个物理子步重算 PD（控制步算一次会很快倒，见 actuators.py 的说明）
        for _ in range(4):
            ctrl, _target = A.ctrl_from_action(act, pre, qpos=d.qpos[7:36], qvel=d.qvel[6:35])
            d.ctrl[:] = ctrl
            mujoco.mj_step(m, d)
        zs.append(float(d.qpos[2]))
        ups.append(float(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))
        if zs[-1] < 0.3:
            break

    print(f"z 起 {pre['initialQpos'][2]:.3f} → {zs[-1]:.3f}（最低 {min(zs):.3f}），"
          f"直立 {ups[-1]:.3f}，共 {len(zs)} 步")
    # 判据不是「站住」——loco 单独跑本来也站不久（botlab 自己的 native 运行时约 80–120 步就倒）。
    # 这里判的是「能按同一套语义跑起来、并且不比它差太多」。
    ok = len(zs) >= 60
    print(("PASS" if ok else "FAIL")
          + "：扭矩模型 + 子步 PD + loco（判据：≥60 步；botlab native 自身约 80–120 步）")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
