#!/usr/bin/env python3
"""策略闭环：manifest → 图 → C 核（含 ONNX 推理）→ MuJoCo。W5/W6 的验收脚本。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/run_policy.py \
        --mjcf build/mjcf/duck-hd1910-all.xml \
        --preset build/preset/duck-hd1910.preset.yaml \
        --manifest contract/policy/duck-hold.manifest.yaml \
        --seconds 30 --report build/report/policy.json \
        --trajectory client/viewer/data/replay-duck-policy.json

**图是从 manifest 长出来的**，不是写死在这份脚本里（D12）。manifest 定 obs 顺序与缩放、历史布局、
动作映射与夹取范围、以及三个频率。脚本只负责：把 MuJoCo 的状态按槽名喂进去、把 action 写回 `d.ctrl`。

这条链上各段的位置（D5/D8/D9）：obs 组装 → 图 → action 在 C++ 核里；推理是核调 `ds_backend.onnx_run`
（`libducksim_ort.so`）；物理在宿主，核不知道 MuJoCo 存在。

manifest 与模型/preset 对不上就**红着退出**，不猜。
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


def check_manifest(man: dict, preset: dict, n_joints: int) -> None:
    """manifest ↔ 模型 ↔ preset 的一致性。任一不过就停，不猜。"""
    errs = []
    in_shape = man["model"]["input"]["shape"]
    out_shape = man["model"]["output"]["shape"]
    total = sum(o["len"] for o in man["obs"])
    if in_shape[-1] != total:
        errs.append(f"obs 总长 {total} != 模型输入 {in_shape[-1]}")
    if out_shape[-1] != n_joints:
        errs.append(f"模型输出 {out_shape[-1]} != 驱动关节 {n_joints}")
    if preset["actionDimension"] != n_joints:
        errs.append(f"preset actionDimension {preset['actionDimension']} != {n_joints}")
    r = man["rates"]
    if r["physics_hz"] % r["control_hz"] != 0:
        errs.append("physics_hz 必须是 control_hz 的整数倍")
    if int(r["inference_decimation"]) < 1:
        errs.append("inference_decimation 必须 ≥ 1")
    if man["action"]["mapping"] != "default_plus_scale":
        errs.append(f"暂只支持 mapping=default_plus_scale（现在 {man['action']['mapping']}）")
    if man["action"]["order"] != "motor":
        errs.append(f"暂只支持 order=motor（现在 {man['action']['order']}）")
    if sorted(preset["motorJointOrder"]) != list(range(n_joints)):
        errs.append("motorJointOrder 不是 0..n-1 的置换")
    if errs:
        raise SystemExit("manifest 校验失败：\n  " + "\n  ".join(errs))


def build_plan(man: dict, preset: dict, n_joints: int, ctrl_lo, ctrl_hi) -> dict:
    total = sum(o["len"] for o in man["obs"])
    p = ir.Plan(input_dim=total, output_dim=n_joints, name=man["policy_id"])
    sl, off = [], 0
    for o in man["obs"]:
        sl.append(p.obs(off, o["len"]))
        off += o["len"]
    x = p.concat(*sl) if len(sl) > 1 else sl[0]

    h = man.get("history") or {}
    frames = int(h.get("length", 0))
    if frames > 0:
        if h.get("layout") != "by-frame":
            raise SystemExit(f"history.layout 只支持 by-frame（现在 {h.get('layout')}）")
        x = p.history(x, frames)

    m = p.model(x, out_len=n_joints, model_id=0)
    s = p.affine(m, preset["actionScale"], preset["defaultJointPositions"])
    k = p.clip(s, ctrl_lo, ctrl_hi)
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
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--start-height", type=float, default=0.25)
    ap.add_argument("--clip-rad", type=float, help="覆盖 manifest 的 ctrlrange 夹取（调试用）")
    ap.add_argument("--report")
    ap.add_argument("--trajectory")
    args = ap.parse_args()

    import mujoco

    with open(args.preset, encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    with open(args.manifest, encoding="utf-8") as f:
        man = yaml.safe_load(f)
    doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    n = len(driven)

    check_manifest(man, preset, n)
    actuators = doc.get("actuators") or []
    if actuators and all(a.get("ctrlrange") for a in actuators):
        lo = [float(a["ctrlrange"][0]) for a in actuators]
        hi = [float(a["ctrlrange"][1]) for a in actuators]
    else:
        lo = [-0.5] * n
        hi = [0.5] * n
    if args.clip_rad is not None:
        lo, hi = [-args.clip_rad] * n, [args.clip_rad] * n

    plan = build_plan(man, preset, n, lo, hi)
    blob = ir.compile_blob(plan)

    rates = man["rates"]
    control_hz, physics_hz = float(rates["control_hz"]), float(rates["physics_hz"])
    decim = int(physics_hz // control_hz)
    infer_every = int(rates["inference_decimation"])

    core, ort = load_libs()
    err = ctypes.create_string_buffer(512)
    model_path = os.path.join(ROOT, man["model"]["file"])
    be = ort.ds_ort_create(model_path.encode(), err, 512)
    if not be:
        raise SystemExit("ORT 后端创建失败：" + err.value.decode())
    obs_dim = sum(o["len"] for o in man["obs"])
    if ort.ds_ort_input_len(be) != obs_dim or ort.ds_ort_output_len(be) != n:
        raise SystemExit(
            f"模型 IO {ort.ds_ort_input_len(be)}→{ort.ds_ort_output_len(be)} 与 manifest {obs_dim}→{n} 不符")

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
    m.opt.timestep = 1.0 / physics_hz
    d.qpos[:] = 0.0
    d.qpos[3] = 1.0
    d.qpos[2] = args.start_height
    mujoco.mj_forward(m, d)

    qpos_off = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]
    dof_off = [int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]

    steps = int(args.seconds * control_hz)
    obs = np.zeros(obs_dim, dtype=np.float32)
    act = np.zeros(n, dtype=np.float32)
    frames, heights, tilts, efforts = [], [], [], []
    dead = False
    infer_calls = 0
    last_infer_step = -10 ** 9
    t0 = time.time()

    def gather(slot: str) -> np.ndarray:
        if slot == "base_ang_vel":
            return d.qvel[3:6].copy()
        if slot == "projected_gravity":
            return -quat_to_mat(d.qpos[3:7]).T @ np.array([0.0, 0.0, 1.0])
        if slot == "joint_pos":
            return np.array([d.qpos[o] for o in qpos_off])
        if slot == "joint_vel":
            return np.array([d.qvel[o] for o in dof_off])
        if slot == "last_action":
            return act.copy()
        if slot == "base_height":
            return np.array([d.qpos[2]])
        raise SystemExit(f"manifest 里出现未知 obs 槽：{slot}")

    for k in range(steps):
        off = 0
        for o in man["obs"]:
            v = gather(o["slot"]) * float(o.get("scale", 1.0))
            obs[off:off + o["len"]] = v
            off += o["len"]

        # 推理节流：inference_decimation=1 表示每控制步都推（与 botlab 的 actor 一致）
        if k - last_infer_step >= infer_every:
            buf_in = (ctypes.c_float * obs_dim)(*obs.tolist())
            buf_out = (ctypes.c_float * n)()
            rc = core.ds_step(handle, buf_in, buf_out, be)
            if rc != 0:
                raise SystemExit(f"ds_step 返回 {rc}")
            act = np.array(buf_out[:], dtype=np.float32)
            infer_calls += 1
            last_infer_step = k

        d.ctrl[:] = act
        for _ in range(decim):
            mujoco.mj_step(m, d)

        if not np.isfinite(d.qpos).all():
            dead = True
            break

        heights.append(float(d.qpos[2]))
        tilts.append(float(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))
        efforts.append(float(np.abs(d.actuator_force).mean()))
        if args.trajectory:
            frames.append({
                "t": round(k / control_hz, 4),
                "pos": [round(float(x), 6) for x in d.qpos[0:3]],
                "quat": [round(float(x), 6) for x in d.qpos[3:7]],
                "q": [round(float(d.qpos[o]), 6) for o in qpos_off],
            })

    wall = time.time() - t0
    heights = np.array(heights) if heights else np.array([np.nan])
    tilts = np.array(tilts) if tilts else np.array([np.nan])
    sim_seconds = len(heights) / control_hz
    metrics = {
        "steps": len(heights),
        "wall_seconds": round(wall, 2),
        "rtf": round(sim_seconds / wall, 2) if wall > 0 else None,
        "inference_calls": infer_calls,
        "inference_hz": round(infer_calls / sim_seconds, 2) if sim_seconds > 0 else None,
        "mean_abs_action": round(float(np.mean(np.abs(act))), 5),
        "mean_abs_torque": round(float(np.mean(efforts)) if efforts else 0.0, 5),
        "base_height_final": float(heights[-1]),
        "base_height_min": float(np.nanmin(heights)),
        "upright_final": float(tilts[-1]),
    }
    verdict = {
        "finite": not dead,
        "no_sink": bool(np.nanmin(heights) > -0.05),
        "ran_full": len(heights) == steps,
        "rates_match": abs((metrics["inference_hz"] or 0) - control_hz / infer_every) < 1e-6,
        "pass": (not dead) and bool(np.nanmin(heights) > -0.05) and len(heights) == steps,
    }
    report = {
        "incident_id": "",
        "plan_digest": ir.sha256_of_blob(blob),
        "policy_manifest_digest": sha256(args.manifest),
        "policy_id": man["policy_id"],
        "mjcf_digest": sha256(args.mjcf) if os.path.exists(args.mjcf) else "",
        "model_digest": sha256(model_path),
        "entity_digest": preset["generated"]["entity_sha256"],
        "rates": rates,
        "nodes": info.node_count,
        "steps": steps,
        "metrics": metrics,
        "verdict": verdict,
    }

    print(f"策略闭环 {args.seconds}s @ {control_hz:g}/{physics_hz:g} Hz · 推理 {infer_calls} 次"
          f"（{metrics['inference_hz']} Hz，decimation={infer_every}）· 图 {info.node_count} 节点")
    print(f"  帧数 {len(heights)}/{steps}｜墙钟 {wall:.2f}s（{metrics['rtf']}× 实时）")
    print(f"  |action| {metrics['mean_abs_action']:.4f} rad｜实际力矩 {metrics['mean_abs_torque']:.4f} Nm")
    print(f"  基座高度 终 {metrics['base_height_final']:+.3f} 最低 {metrics['base_height_min']:+.3f} m｜"
          f"直立度 终 {metrics['upright_final']:+.3f}")
    print(f"  {'PASS' if verdict['pass'] else 'FAIL'}  finite={verdict['finite']} "
          f"no_sink={verdict['no_sink']} full={verdict['ran_full']} rates={verdict['rates_match']}")

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
                                "control_hz": control_hz, "physics_hz": physics_hz,
                                "policy_id": man["policy_id"],
                                "mjcf_digest": report["mjcf_digest"],
                                "preset_digest": sha256(args.preset)},
                       "frames": frames}, f, ensure_ascii=False, separators=(",", ":"))
        print(f"  轨迹 → {args.trajectory}（{len(frames)} 帧）")

    core.ds_free(handle)
    ort.ds_ort_destroy(be)
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
