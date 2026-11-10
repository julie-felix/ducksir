#!/usr/bin/env python3
"""策略闭环：obs → 执行核（含 ONNX 推理）→ action → MuJoCo。W5 的验收脚本。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/run_policy.py \
        --mjcf build/mjcf/duck-hd1910-all.xml \
        --preset build/preset/duck-hd1910.preset.yaml \
        --onnx build/models/duck-hold.onnx \
        --seconds 30 --report build/report/policy.json \
        --trajectory client/viewer/data/replay-duck-policy.json

这条链上各段都在各自的位置上（D5/D8/D9）：
- **obs 组装 → 图 → action** 全在 C++ 核里（`libducksim.so`）；
- **推理**是核调 `ds_backend.onnx_run`，实现在 `libducksim_ort.so`（链 onnxruntime）；
- **物理**在宿主（这里就是本文件），核不知道 MuJoCo 存在。

动作映射（botlab 的约定，D13）：`ctrl = defaultJointPositions + actionScale * action`，
在图上用 `affine` 节点做，所以核吐出来的已经是可以直接写 `d.ctrl` 的关节目标。
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import json
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))

import entity as E  # noqa: E402
import ir  # noqa: E402

LIB_CORE = os.path.join(ROOT, "engine", "build", "libducksim.so")
LIB_ORT = os.path.join(ROOT, "engine", "build", "libducksim_ort.so")

OBS_ANG_VEL = (0, 3)
OBS_GRAVITY = (3, 6)
OBS_JOINT_POS = (6, 20)
OBS_JOINT_VEL = (20, 34)
OBS_DIM = 34


class Info(ctypes.Structure):
    _fields_ = [("version", ctypes.c_uint16), ("abi", ctypes.c_uint16),
                ("node_count", ctypes.c_uint16), ("input_dim", ctypes.c_uint16),
                ("output_dim", ctypes.c_uint16), ("arena_len", ctypes.c_uint32),
                ("param_len", ctypes.c_uint32)]


def load_libs():
    core = ctypes.CDLL(LIB_CORE)
    core.ds_load.restype = ctypes.c_void_p
    core.ds_load.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p, ctypes.c_size_t]
    core.ds_free.argtypes = [ctypes.c_void_p]
    core.ds_info_of.argtypes = [ctypes.c_void_p, ctypes.POINTER(Info)]
    core.ds_step.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float),
                             ctypes.POINTER(ctypes.c_float), ctypes.c_void_p]
    core.ds_reset.argtypes = [ctypes.c_void_p]

    ort = ctypes.CDLL(LIB_ORT)
    ort.ds_ort_create.restype = ctypes.c_void_p
    ort.ds_ort_create.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t]
    ort.ds_ort_destroy.argtypes = [ctypes.c_void_p]
    ort.ds_ort_input_len.restype = ctypes.c_int64
    ort.ds_ort_output_len.restype = ctypes.c_int64
    return core, ort


def build_plan(preset: dict, n_joints: int, clip_rad: float) -> dict:
    """34 维 obs → concat → model → affine(actionScale/default) → clip → out_action。"""
    p = ir.Plan(input_dim=OBS_DIM, output_dim=n_joints, name="duck-policy")
    a = p.obs(*OBS_ANG_VEL)
    g = p.obs(OBS_GRAVITY[0], OBS_GRAVITY[1] - OBS_GRAVITY[0])
    q = p.obs(OBS_JOINT_POS[0], OBS_JOINT_POS[1] - OBS_JOINT_POS[0])
    if q.length != n_joints:
        raise SystemExit(f"obs 关节段长度 {q.length} 与驱动关节数 {n_joints} 不符")
    v = p.obs(OBS_JOINT_VEL[0], OBS_JOINT_VEL[1] - OBS_JOINT_VEL[0])
    c = p.concat(a, g, q, v)
    m = p.model(c, out_len=n_joints, model_id=0)
    s = p.affine(m, preset["actionScale"], preset["defaultJointPositions"])
    k = p.clip(s, [-clip_rad] * n_joints, [clip_rad] * n_joints)
    p.out_action(k)
    return p.to_json()


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--preset", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--control-hz", type=float, default=50.0)
    ap.add_argument("--physics-hz", type=float, default=200.0)
    ap.add_argument("--clip-rad", type=float, default=0.5)
    ap.add_argument("--start-height", type=float, default=0.25)
    ap.add_argument("--report")
    ap.add_argument("--trajectory")
    args = ap.parse_args()

    import mujoco

    if args.physics_hz % args.control_hz != 0:
        raise SystemExit("physics_hz 必须是 control_hz 的整数倍")
    decim = int(args.physics_hz // args.control_hz)

    with open(args.preset, encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    n = len(driven)

    plan = build_plan(preset, n, args.clip_rad)
    blob = ir.compile_blob(plan)

    core, ort = load_libs()
    err = ctypes.create_string_buffer(512)
    be = ort.ds_ort_create(args.onnx.encode(), err, 512)
    if not be:
        raise SystemExit("ORT 后端创建失败：" + err.value.decode())
    if ort.ds_ort_input_len(be) != OBS_DIM or ort.ds_ort_output_len(be) != n:
        raise SystemExit(f"模型 IO 维度 {ort.ds_ort_input_len(be)}→{ort.ds_ort_output_len(be)} 与图 {OBS_DIM}→{n} 不符")

    handle = core.ds_load(blob, len(blob), err, 512)
    if not handle:
        raise SystemExit("加载 plan 失败：" + err.value.decode())
    info = Info()
    core.ds_info_of(handle, ctypes.byref(info))

    spec = mujoco.MjSpec.from_file(args.mjcf)
    g = spec.worldbody.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_PLANE
    g.size = [3.0, 3.0, 0.1]
    g.condim = 3
    m = spec.compile()
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / args.physics_hz

    d.qpos[:] = 0.0
    d.qpos[3] = 1.0
    d.qpos[2] = args.start_height
    mujoco.mj_forward(m, d)

    qpos_off = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]
    dof_off = [int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]

    steps = int(args.seconds * args.control_hz)
    obs = np.zeros(OBS_DIM, dtype=np.float32)
    act = np.zeros(n, dtype=np.float32)
    frames = []
    tilts = []
    heights = []
    dead = False
    t0 = time.time()

    for k in range(steps):
        R = quat_to_mat(d.qpos[3:7])
        obs[OBS_ANG_VEL[0]:OBS_ANG_VEL[1]] = d.qvel[3:6]
        obs[OBS_GRAVITY[0]:OBS_GRAVITY[1]] = -R.T @ np.array([0.0, 0.0, 1.0])
        obs[OBS_JOINT_POS[0]:OBS_JOINT_POS[1]] = [d.qpos[o] for o in qpos_off]
        obs[OBS_JOINT_VEL[0]:OBS_JOINT_VEL[1]] = [d.qvel[o] for o in dof_off]

        buf_in = (ctypes.c_float * OBS_DIM)(*obs.tolist())
        buf_out = (ctypes.c_float * n)()
        rc = core.ds_step(handle, buf_in, buf_out, be)
        if rc != 0:
            raise SystemExit(f"ds_step 返回 {rc}")
        act = np.array(buf_out[:], dtype=np.float32)

        d.ctrl[:] = act
        for _ in range(decim):
            mujoco.mj_step(m, d)

        if not np.isfinite(d.qpos).all():
            dead = True
            break

        z = float(d.qpos[2])
        up = float(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2))
        heights.append(z)
        tilts.append(up)
        if args.trajectory:
            frames.append({
                "t": round(k / args.control_hz, 4),
                "pos": [round(float(x), 6) for x in d.qpos[0:3]],
                "quat": [round(float(x), 6) for x in d.qpos[3:7]],
                "q": [round(float(d.qpos[o]), 6) for o in qpos_off],
            })

    wall = time.time() - t0
    heights = np.array(heights) if heights else np.array([np.nan])
    tilts = np.array(tilts) if tilts else np.array([np.nan])
    metrics = {
        "steps": len(heights),
        "wall_seconds": round(wall, 2),
        "rtf": round((len(heights) / args.control_hz) / wall, 2) if wall > 0 else None,
        "base_height_final": float(heights[-1]),
        "base_height_min": float(np.nanmin(heights)),
        "upright_final": float(tilts[-1]),
        "upright_min": float(np.nanmin(tilts)),
    }
    verdict = {
        "finite": not dead,
        "no_sink": bool(np.nanmin(heights) > -0.05),
        "ran_full": len(heights) == steps,
        "pass": (not dead) and bool(np.nanmin(heights) > -0.05) and len(heights) == steps,
    }
    report = {
        "incident_id": "",
        "plan_digest": ir.sha256_of_blob(blob),
        "policy_manifest_digest": "",
        "mjcf_digest": sha256(args.mjcf) if os.path.exists(args.mjcf) else "",
        "model_digest": sha256(args.onnx),
        "entity_digest": preset["generated"]["entity_sha256"],
        "control_hz": args.control_hz,
        "physics_hz": args.physics_hz,
        "steps": steps,
        "nodes": info.node_count,
        "metrics": metrics,
        "verdict": verdict,
    }

    print(f"策略闭环 {args.seconds}s @ {args.control_hz}/{args.physics_hz} Hz · 图 {info.node_count} 节点 · "
          f"模型 {os.path.basename(args.onnx)}")
    print(f"  帧数 {len(heights)}/{steps}｜墙钟 {wall:.2f}s（实时率 {metrics['rtf']}×）")
    print(f"  基座高度 终 {metrics['base_height_final']:+.3f} 最低 {metrics['base_height_min']:+.3f} m｜"
          f"直立度 终 {metrics['upright_final']:+.3f}")
    print(f"  {'PASS' if verdict['pass'] else 'FAIL'}  "
          f"finite={verdict['finite']} no_sink={verdict['no_sink']} full={verdict['ran_full']}")

    if args.report:
        out = os.path.join(ROOT, args.report)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(f"  报告 → {args.report}")
    if args.trajectory:
        out = os.path.join(ROOT, args.trajectory)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"meta": {"robot": doc["robot"]["id"], "joints": driven, "target": "policy",
                                "amp": 0, "gravity": "on", "seconds": args.seconds,
                                "control_hz": args.control_hz, "physics_hz": args.physics_hz,
                                "mjcf_digest": report["mjcf_digest"], "preset_digest": sha256(args.preset)},
                       "frames": frames}, f, ensure_ascii=False, separators=(",", ":"))
        print(f"  轨迹 → {args.trajectory}（{len(frames)} 帧）")

    core.ds_free(handle)
    ort.ds_ort_destroy(be)
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
