#!/usr/bin/env python3
"""Instinct-SuperMimic 冒烟：用我们的观测装配（含**近似深度**）+ 他们的 policy 跑闭环。

深度还没和 botlab 逐像素对齐（见 docs/notes/g1-botlab.md §19：cols z+1 + scale 1/3，差 0.572），
这个脚本的目的是先回答「策略能不能站住」，而不是「像素是否一致」。

    MUJOCO_GL=egl python3 tools/dev/instinct_smoke.py
"""
import json
import os
import sys

import mujoco
import numpy as np
import onnxruntime as ort
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "sim", "host"))
sys.path.insert(0, "/tmp/inst-run")

import depth_camera  # noqa: E402
import instinct_obs as I  # noqa: E402
import motion_choices as mc  # noqa: E402
import actuators as A  # noqa: E402

TPL = "/tmp/inst-run"
MOTION_DIR = "/tmp/bt-all/Instinct-SuperMimic/assets/motions"


def mat2quat(M):
    tr = M[0, 0] + M[1, 1] + M[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return [0.25 * s, (M[2, 1] - M[1, 2]) / s, (M[0, 2] - M[2, 0]) / s, (M[1, 0] - M[0, 1]) / s]
    i = int(np.argmax([M[0, 0], M[1, 1], M[2, 2]]))
    if i == 0:
        s = np.sqrt(1 + M[0, 0] - M[1, 1] - M[2, 2]) * 2
        return [(M[2, 1] - M[1, 2]) / s, 0.25 * s, (M[0, 1] + M[1, 0]) / s, (M[0, 2] + M[2, 0]) / s]
    if i == 1:
        s = np.sqrt(1 + M[1, 1] - M[0, 0] - M[2, 2]) * 2
        return [(M[0, 2] - M[2, 0]) / s, (M[0, 1] + M[1, 0]) / s, 0.25 * s, (M[1, 2] + M[2, 1]) / s]
    s = np.sqrt(1 + M[2, 2] - M[0, 0] - M[1, 1]) * 2
    return [(M[1, 0] - M[0, 1]) / s, (M[0, 2] + M[2, 0]) / s, (M[1, 2] + M[2, 1]) / s, 0.25 * s]


def main() -> int:
    from depth_camera import DepthCamera

    cfg = mc.CONFIGS[mc.DEFAULT_MOTION]
    z = np.load(os.path.join(MOTION_DIR, f"case_{cfg['case']:02d}.npz"), allow_pickle=True)
    motion = {k: z[k] for k in ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w")}
    motion["fps"] = float(z["fps"][0])
    dof = [str(x) for x in z["dof_names"]]
    joint_order = [dof.index(n) for n in cfg["joint_names"]]
    torso_idx = [str(b) for b in z["body_names"]].index("torso_link")
    duration = (cfg["frames"] - 0.5) / 50

    cam = yaml.safe_load(open(os.path.join(TPL, "camera.preset.yaml"), encoding="utf-8"))
    preset = yaml.safe_load(open(os.path.join(TPL, "robot.preset.yaml"), encoding="utf-8"))
    DQ = np.asarray(preset["defaultJointPositions"])
    ASC = np.asarray(preset["actionScale"])
    KP = np.asarray(preset["kp"])
    KD = np.asarray(preset["kd"])
    EFF = np.asarray(preset.get("effortLimit") or [88.0] * 29)

    spec = mujoco.MjSpec.from_file(os.path.join(TPL, "assets/robot/scene.xml"))
    spec.meshdir = os.path.join(TPL, "assets/robot/meshes")
    model = spec.compile()
    data = mujoco.MjData(model)
    model.opt.timestep = 1.0 / 200

    data.qpos[:] = cfg["initial_qpos"] if cfg.get("initial_qpos") else preset["initialQpos"]
    mujoco.mj_forward(model, data)
    init_qpos = np.array(data.qpos[:7])

    sess = ort.InferenceSession(os.path.join(TPL, "assets/models/policy.onnx"),
                                providers=["CPUExecutionProvider"])
    depth_cam = DepthCamera(model, cam)

    hist = I.TermHistory()
    last = np.zeros(29, dtype=np.float64)
    bid_torso = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    qpos_off = [i for i in range(7, 7 + 29)]
    dof_off = [i for i in range(6, 6 + 29)]

    zs, tilts = [], []
    for k in range(300):
        q = np.array([data.qpos[i] for i in qpos_off])
        qd = np.array([data.qvel[i] for i in dof_off])
        w, qx, qy, qz = data.xquat[bid_torso]
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * w), 2 * (qx * qz + qy * w)],
            [2 * (qx * qy + qz * w), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * w)],
            [2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (qx * qx + qy * qy)]])
        block, terms = I.build_terms(
            motion=motion, t=k * 0.02, duration=duration, joint_order=joint_order,
            config_defaults=cfg["default_angles"],
            robot_joint_pos=q - DQ, robot_joint_vel=qd,
            gravity_b=R.T @ np.array([0, 0, -1.0]), base_ang_vel=data.qvel[3:6],
            last_action=last, torso_index=torso_idx, robot_torso_quat=data.xquat[bid_torso].copy())
        hist990 = hist.push(terms)

        depth = depth_cam.process(depth_cam.render(data)).reshape(-1)

        obs = np.concatenate([block, depth, hist990]).astype(np.float32)
        out = sess.run(None, {sess.get_inputs()[0].name: obs.reshape(1, -1)})
        action = np.asarray(out[0]).reshape(-1)
        last = np.clip(action, -100, 100)
        # 他们的 MJCF 是**位置执行器**（kp/kv 在 XML 里），所以 ctrl 直接写目标位置；
        # 早先这里喂扭矩，等于把 6 N·m 当目标角度，必倒。
        target = DQ + ASC * last
        for _ in range(4):
            data.ctrl[:] = target
            mujoco.mj_step(model, data)
        zs.append(float(data.qpos[2]))
        w0, qx0, qy0, qz0 = data.qpos[3:7]
        tilts.append(float(1 - 2 * (qx0 * qx0 + qy0 * qy0)))
        if zs[-1] < 0.2:
            break

    print(f"z 起 {zs[0]:.3f} → {zs[-1]:.3f}（最低 {min(zs):.3f}）｜直立 {tilts[-1]:.3f}｜{len(zs)} 步")
    ok = len(zs) >= 60 and min(zs) > 0.3
    print(("PASS" if ok else "FAIL") + "：Instinct（近似深度）闭环")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
