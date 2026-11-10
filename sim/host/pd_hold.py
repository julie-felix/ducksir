#!/usr/bin/env python3
"""PD 跟踪 + 物理链路验证（W3 的验收脚本，也是宿主仿真的第一块）。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/pd_hold.py \
        --mjcf build/mjcf/duck-hd1910.xml --preset build/preset/duck-hd1910.preset.yaml \
        --seconds 10 --target sine --report build/report/pd_hold.json

判据（2026-11-10 与庆喆改定，见计划 §9 W3）分两种跑法：

**链路验证**（`--gravity off --target step`）：关重力、基座不受力，把「actuator 映射 / 控制频率 /
观测读取」从平衡问题里隔离出来。判稳态误差 ≤ 0.005 rad。

**物理验证**（`--gravity on --target sine --floor`）：开重力、自由基座、有地面。判
① 10 s 不发散（qpos 全程有限）② 接触比 > 0.5（至少一半时间有接触）③ 基座全程不低于 -0.05 m（不穿地）。
跟踪误差只记录不设阈值——重力和 kp 是设备与标定的活，不是链路的事。

注意这不是「站 10 秒」：全零位形不是站姿，PD 也管不了自由基座的平衡（2026-11-10 探针实测，
kp 从 0.55 拉到 120 终点姿态不变）。站立要等策略。

物理在宿主（D9），这个脚本以后会被 C++ 核接管 obs→action 那一段，脚本本身的角色变成「喂观测、收动作」。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "tools/robot"))

import entity as E  # noqa: E402

OBS_NAMES = ["base_ang_vel", "projected_gravity", "joint_pos", "joint_vel", "last_action"]


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_model(mjcf: str, floor: bool):
    import mujoco

    if not floor:
        return mujoco.MjModel.from_xml_path(mjcf)
    spec = mujoco.MjSpec.from_file(mjcf)
    g = spec.worldbody.add_geom()
    g.name = "floor"
    g.type = mujoco.mjtGeom.mjGEOM_PLANE
    g.size = [3.0, 3.0, 0.1]
    g.rgba = [0.55, 0.55, 0.58, 1.0]
    g.condim = 3
    return spec.compile()


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def observe(m, d, preset, driven, last_action):
    """8 个观测槽里的前 5 个 + 两个基座量；语义与 node 集对齐（见 docs/node-parity.md）。"""
    up = quat_to_mat(d.qpos[3:7]) @ np.array([0.0, 0.0, 1.0])
    q = np.array([d.qpos[int(o)] for o in preset["jointQposOffset"]])
    v = np.array([d.qvel[int(o)] for o in preset["jointQvelOffset"]])
    return {
        "base_ang_vel": d.qvel[3:6].copy(),
        "projected_gravity": up,
        "joint_pos": q,
        "joint_vel": v,
        "last_action": np.asarray(last_action, dtype=float),
        "base_height": float(d.qpos[2]),
    }


def target_signal(kind: str, t: float, n: int, amp: float) -> np.ndarray:
    if kind == "zero":
        return np.zeros(n)
    if kind == "sine":
        return np.full(n, amp * math.sin(2 * math.pi * 0.5 * t))
    if kind == "step":
        return np.full(n, amp if t >= 1.0 else 0.0)
    raise SystemExit(f"未知 target: {kind}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--preset", required=True)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--control-hz", type=float, default=50.0)
    ap.add_argument("--physics-hz", type=float, default=200.0)
    ap.add_argument("--target", default="sine", choices=["zero", "sine", "step"])
    ap.add_argument("--amp", type=float, default=0.2, help="目标幅度（rad）")
    ap.add_argument("--gravity", default="on", choices=["on", "off"])
    ap.add_argument("--floor", action="store_true", default=True)
    ap.add_argument("--no-floor", dest="floor", action="store_false")
    ap.add_argument("--report", help="把结果写成 JSON")
    ap.add_argument("--trajectory", help="把轨迹写成 JSON（给 viewer 回放）")
    args = ap.parse_args()

    if args.physics_hz % args.control_hz != 0:
        raise SystemExit("physics_hz 必须是 control_hz 的整数倍")
    decim = int(args.physics_hz // args.control_hz)

    import mujoco

    m = load_model(os.path.join(ROOT, args.mjcf) if not os.path.isabs(args.mjcf) else args.mjcf, args.floor)
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / args.physics_hz
    if args.gravity == "off":
        m.opt.gravity[:] = 0.0

    with open(args.preset, encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    n = len(driven)

    if preset["actionDimension"] != n:
        raise SystemExit(f"preset actionDimension={preset['actionDimension']} != 驱动关节 {n}")
    if m.nu != n:
        raise SystemExit(f"MJCF nu={m.nu} != 驱动关节 {n}")

    default = np.array(preset["defaultJointPositions"], dtype=float)
    scale = np.array(preset["actionScale"], dtype=float)

    # 初始：基座抬到 0.25 m，关节在 default
    d.qpos[:] = 0.0
    d.qpos[3] = 1.0
    d.qpos[2] = 0.25
    for off, ang in zip(preset["jointQposOffset"], default):
        d.qpos[int(off)] = ang
    mujoco.mj_forward(m, d)

    steps = int(args.seconds * args.control_hz)
    act = np.zeros(n)
    last_action = np.zeros(n)
    frames = []
    err_idle, err_all = [], []
    contacts = []
    finite = True
    zs = []
    t0 = time.time()
    for k in range(steps):
        t = k / args.control_hz
        ref = target_signal(args.target, t, n, args.amp)
        act = (ref - default) / np.where(scale == 0, 1, scale)
        d.ctrl[:] = default + scale * act
        for _ in range(decim):
            mujoco.mj_step(m, d)
        if not np.isfinite(d.qpos).all():
            finite = False
            break
        obs = observe(m, d, preset, driven, last_action)
        for k2, v in obs.items():
            if isinstance(v, np.ndarray) and not np.isfinite(v).all():
                finite = False
        q = obs["joint_pos"]
        err = float(np.abs(q - ref).max())
        err_all.append(err)
        # 稳态窗口取后 25%：阶跃目标在 1 s 跳变，前面那一段是瞬态
        if k >= int(steps * 0.75):
            err_idle.append(err)
        contacts.append(d.ncon)
        zs.append(float(d.qpos[2]))
        if args.trajectory:
            frames.append({
                "t": round(t, 4),
                "pos": [round(float(x), 6) for x in d.qpos[0:3]],
                "quat": [round(float(x), 6) for x in d.qpos[3:7]],
                "q": [round(float(x), 6) for x in obs["joint_pos"]],
            })
        last_action = act

    wall = time.time() - t0
    err_all = np.array(err_all) if err_all else np.array([np.nan])
    err_idle = np.array(err_idle) if err_idle else np.array([np.nan])
    contacts = np.array(contacts) if contacts else np.array([0])
    zs = np.array(zs) if zs else np.array([np.nan])

    ok_finite = finite
    if args.target == "step":
        # 链路验证：无重力时位置伺服稳态误差应为 0
        ok_track = bool(np.nanmax(err_idle) <= 0.005)
        ok_contact = bool(np.nanmax(np.abs(zs)) < 1.0)  # 自由漂浮，只看不炸
        height_ok = True
    else:
        # 物理验证：跟踪误差只记录，判的是不发散 / 有接触 / 不穿地
        ok_track = True
        ok_contact = bool((contacts > 0).mean() >= 0.5)
        height_ok = bool(np.nanmin(zs) > -0.05)
    obs_dims = {"base_ang_vel": 3, "projected_gravity": 3, "joint_pos": n, "joint_vel": n, "last_action": n}
    ok_obs = finite and all(obs_dims[k] > 0 for k in obs_dims)

    report = {
        "incident_id": "",
        "plan_digest": "",
        "policy_manifest_digest": "",
        "mjcf_digest": sha256(args.mjcf) if os.path.exists(args.mjcf) else "",
        "preset_digest": sha256(args.preset),
        "entity_digest": preset["generated"]["entity_sha256"],
        "target": args.target,
        "amp": args.amp,
        "steps": len(err_all),
        "control_hz": args.control_hz,
        "physics_hz": args.physics_hz,
        "resets": [],
        "metrics": {
            "max_err_rad": float(np.nanmax(err_all)),
            "steady_err_rad": float(np.nanmax(err_idle)),
            "contact_ratio": float((contacts > 0).mean()),
            "height_ok": height_ok,
            "base_height_final": float(zs[-1]),
            "base_height_min": float(np.nanmin(zs)),
            "wall_seconds": round(wall, 2),
        },
        "verdict": {
            "finite": ok_finite,
            "track": ok_track,
            "contact": ok_contact,
            "observations": ok_obs,
            "geometry": height_ok,
            "pass": bool(ok_finite and ok_track and ok_contact and ok_obs and height_ok),
        },
    }

    print(f"target={args.target} amp={args.amp} gravity={args.gravity} {args.seconds}s @ {args.control_hz}/{args.physics_hz} Hz")
    print(f"  最大误差 {report['metrics']['max_err_rad']:.4f} rad｜稳态 {report['metrics']['steady_err_rad']:.4f} rad")
    print(f"  接触比 {report['metrics']['contact_ratio']:.2f}｜基座高度 终 {zs[-1]:+.3f} 最低 {np.nanmin(zs):+.3f} m")
    print(f"  {'PASS' if report['verdict']['pass'] else 'FAIL'}  "
          f"finite={ok_finite} track={ok_track} contact={ok_contact} obs={ok_obs} geom={height_ok}（墙钟 {wall:.2f}s）")
    if args.trajectory:
        out = os.path.join(ROOT, args.trajectory)
        os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
        traj = {
            "meta": {
                "robot": os.path.basename(args.mjcf).rsplit(".", 1)[0],
                "joints": driven,
                "target": args.target,
                "amp": args.amp,
                "gravity": args.gravity,
                "seconds": args.seconds,
                "control_hz": args.control_hz,
                "physics_hz": args.physics_hz,
                "mcpf_digest": sha256(args.mjcf) if os.path.exists(args.mjcf) else "",
                "preset_digest": sha256(args.preset),
            },
            "frames": frames,
        }
        with open(out, "w", encoding="utf-8") as f:
            json.dump(traj, f, ensure_ascii=False, separators=(",", ":"))
        print(f"  轨迹 → {args.trajectory}（{len(frames)} 帧）")

    if args.report:
        out = os.path.join(ROOT, args.report)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print(f"  报告 → {args.report}")
    return 0 if report["verdict"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
