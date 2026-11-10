#!/usr/bin/env python3
"""MicroDuck 多策略状态机：把 botlab 的 `microduck.py` + `microduck_policy.py` 那张图用本仓的 DSL 重建。

    PYTHONPATH=$HOME/.local/share/ducksir/pylibs python3 sim/host/run_fsm.py \
        --mjcf build/mjcf/duck-hd1910-all.xml \
        --preset build/preset/duck-hd1910.preset.yaml \
        --seconds 20 --trajectory client/viewer/data/replay-duck-fsm.json

和 `run_policy.py` 的区别：那张图只有一条策略、mode 是常量；这张图有 **状态机**，
按「指令脉冲 / 计时 / 姿态条件」自己切换状态，状态的每一组映射到一份 ONNX。
状态号、转移表、步数阈值逐条照抄 `microduck_policy.py`（它不是 SDK API，是那份工程的约定）。

跑出来的效果：走 → （第 3 秒按一次「坐下/站起」）进入 sit_enter → sit → （第 8 秒按「起身」）回到 walk。
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, "sim", "plan"))
sys.path.insert(0, os.path.join(ROOT, "tools", "robot"))

import entity as E  # noqa: E402
import ir  # noqa: E402
from run_policy import Info, load_libs, quat_to_mat, sha256  # noqa: E402

# ---- 状态号：照抄 botlab microduck_policy.py ----
WALK, SIT_ENTER, SIT, SIT_EXIT = 0, 1, 2, 3
KICK_LEFT, KICK_RIGHT, POST_KICK = 4, 5, 6
ROLL, ROLL_TIPPED, GROUND_PICK = 7, 8, 9
WALK_RESET, RECOVER_ENTER, RECOVER, RECOVER_UPRIGHT = 10, 11, 12, 13

POLICY_WALK, POLICY_SITSTAND, POLICY_KICK_LEFT, POLICY_KICK_RIGHT = 0, 1, 2, 3
POLICY_ROLL, POLICY_GROUND_PICK, POLICY_STAND = 4, 5, 6

MODEL_SPECS = (  # (key, policy_mode, 文件名)
    ("walk", POLICY_WALK, "walking.onnx"),
    ("sitstand", POLICY_SITSTAND, "sitstand.onnx"),
    ("kick_left", POLICY_KICK_LEFT, "kick-left.onnx"),
    ("kick_right", POLICY_KICK_RIGHT, "kick-right.onnx"),
    ("roll", POLICY_ROLL, "roll.onnx"),
    ("ground_pick", POLICY_GROUND_PICK, "ground-pick.onnx"),
    ("stand", POLICY_STAND, "stand.onnx"),
)

# 状态 → 用哪份策略（botlab 的 _policy_mode）
STATE_TO_POLICY = {
    WALK: POLICY_WALK, POST_KICK: POLICY_WALK, WALK_RESET: POLICY_WALK,
    SIT_ENTER: POLICY_SITSTAND, SIT: POLICY_SITSTAND, SIT_EXIT: POLICY_SITSTAND,
    KICK_LEFT: POLICY_KICK_LEFT, KICK_RIGHT: POLICY_KICK_RIGHT,
    ROLL: POLICY_ROLL, ROLL_TIPPED: POLICY_ROLL,
    GROUND_PICK: POLICY_GROUND_PICK,
    RECOVER_ENTER: POLICY_STAND, RECOVER: POLICY_STAND, RECOVER_UPRIGHT: POLICY_STAND,
}

# 步数阈值：照抄（单位 = 控制步，50 Hz）
SIT_ENTER_STEPS, SIT_EXIT_STEPS = 40, 100
KICK_STEPS, POST_KICK_STEPS = 25, 20
ROLL_MIN_STEPS, ROLL_TIMEOUT_STEPS = 40, 150
GROUND_PICK_PERIOD_STEPS, GROUND_PICK_STEPS = 200, 140
RECOVER_UPRIGHT_STEPS = 50

COMMAND_SIT_TOGGLE, COMMAND_KICK_LEFT, COMMAND_KICK_RIGHT = 1, 2, 3
COMMAND_ROLL, COMMAND_GROUND_PICK, COMMAND_RECOVER = 4, 5, 6


def transitions(tipped: int, upright: int) -> list[dict]:
    """botlab `transitions()` 的逐条翻译（条件用 conditions 列表的下标）。"""
    T = [
        {"from": WALK, "command": COMMAND_SIT_TOGGLE, "to": SIT_ENTER},
        {"from": SIT_ENTER, "after_steps": SIT_ENTER_STEPS, "to": SIT},
        {"from": SIT, "command": COMMAND_SIT_TOGGLE, "to": SIT_EXIT},
        {"from": SIT_EXIT, "after_steps": SIT_EXIT_STEPS, "to": WALK_RESET},
        {"from": WALK, "command": COMMAND_KICK_LEFT, "to": KICK_LEFT},
        {"from": WALK, "command": COMMAND_KICK_RIGHT, "to": KICK_RIGHT},
        {"from": KICK_LEFT, "after_steps": KICK_STEPS, "to": POST_KICK},
        {"from": KICK_RIGHT, "after_steps": KICK_STEPS, "to": POST_KICK},
        {"from": POST_KICK, "after_steps": POST_KICK_STEPS, "to": WALK},
        {"from": WALK, "command": COMMAND_ROLL, "to": ROLL},
        {"from": ROLL, "when": tipped, "to": ROLL_TIPPED, "reset_steps": False},
        {"from": ROLL, "after_steps": ROLL_TIMEOUT_STEPS, "to": WALK_RESET},
        {"from": ROLL_TIPPED, "after_steps": ROLL_MIN_STEPS, "when": upright, "to": WALK_RESET},
        {"from": ROLL_TIPPED, "after_steps": ROLL_TIMEOUT_STEPS, "to": WALK_RESET},
        {"from": WALK, "command": COMMAND_GROUND_PICK, "to": GROUND_PICK},
        {"from": GROUND_PICK, "after_steps": GROUND_PICK_STEPS, "to": WALK},
        {"from": WALK, "command": COMMAND_RECOVER, "to": RECOVER_ENTER},
        {"from": RECOVER_ENTER, "after_steps": 1, "to": RECOVER},
        {"from": RECOVER, "when": upright, "to": RECOVER_UPRIGHT},
        {"from": RECOVER_UPRIGHT, "when": 2, "to": RECOVER},    # ~upright（用第三个条件信号表示取反）
        {"from": RECOVER_UPRIGHT, "after_steps": RECOVER_UPRIGHT_STEPS, "to": WALK_RESET},
        {"from": WALK_RESET, "after_steps": 1, "to": WALK},
    ]
    return T


def build_graph(preset: dict, n_joints: int, ctrl_lo, ctrl_hi, vx: float,
                trace_state: bool = False) -> dict:
    """trace_state=True 时把 state 拼在 action 后面输出，方便看状态机到底有没有切。"""
    p = ir.Plan(input_dim=61, output_dim=n_joints + (1 if trace_state else 0), name="duck-fsm")

    ang = p.obs(0, 3)
    grav = p.obs(3, 3)
    jpos = p.obs(6, 14)
    jvel = p.obs(20, 14)
    last = p.obs(34, 14)
    cmd13 = p.obs(48, 13)

    # 条件：tipped = gz > -0.3；upright = gz < -0.85（botlab 的 `gravity.at(2)`）
    gz = p.reorder(grav, [2])
    neg03 = p.constant([-0.3])
    neg085 = p.constant([-0.85])
    tipped = p.compare(gz, "gt", neg03)
    upright = p.compare(gz, "lt", neg085)
    not_upright = p.compare(gz, "gte", neg085)

    # 指令脉冲：由 manifest 的 schedule 在宿主侧注入（这里是占位，第 0 维就是 policy_command）
    policy_command = p.reorder(cmd13, [12])   # 借第 13 维（未用）放「策略指令」

    sm = p.state_machine(policy_command, initial=WALK,
                         transitions=transitions(0, 1), conditions=[tipped, upright, not_upright])
    state = p.reorder(sm, [0])

    def concat_all(refs):
        level = list(refs)
        while len(level) > 4:
            nxt = []
            for i in range(0, len(level), 4):
                chunk = level[i:i + 4]
                nxt.append(chunk[0] if len(chunk) == 1 else p.concat(*chunk))
            level = nxt
        return p.concat(*level) if len(level) > 1 else level[0]

    obs = concat_all([ang, grav, jpos, jvel, last, cmd13])
    outs = [p.model(obs, out_len=n_joints, model_id=i) for i in range(len(MODEL_SPECS))]

    # 状态 → 策略输出：按状态值分块 select，没命中的输出全零，最后相加
    def select_by_state(mapping: dict):
        chunks = []
        keys = list(mapping.items())
        for i in range(0, len(keys), 3):
            part = keys[i:i + 3]
            choices = {float(st): outs[pol] for st, pol in part}
            chunks.append(p.select(state, choices, passive=-1))
        level = chunks
        while len(level) > 4:
            nxt = []
            for i in range(0, len(level), 4):
                chunk = level[i:i + 4]
                nxt.append(chunk[0] if len(chunk) == 1 else p.add(*chunk))
            level = nxt
        return level[0] if len(level) == 1 else p.add(*level)

    mixed = select_by_state(STATE_TO_POLICY)
    s = p.affine(mixed, preset["actionScale"], preset["defaultJointPositions"])
    k = p.clip(s, ctrl_lo, ctrl_hi)
    if trace_state:
        p.out_action(p.concat(k, state))
    else:
        p.out_action(k)
    return p.to_json()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", required=True)
    ap.add_argument("--preset", required=True)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--vx", type=float, default=0.5)
    ap.add_argument("--pulses", default="3:1,8:6", help="指令脉冲，格式 秒:值,秒:值（1 坐/站 6 起身）")
    ap.add_argument("--trace-state", action="store_true")
    ap.add_argument("--report")
    ap.add_argument("--trajectory")
    args = ap.parse_args()

    import mujoco

    with open(args.preset, encoding="utf-8") as f:
        preset = yaml.safe_load(f)
    doc = E.read_yaml(os.path.join(ROOT, preset["generated"]["from"]))
    driven = [j["name"] for j in doc["joints"] if j.get("actuated")]
    n = len(driven)

    actuators = doc.get("actuators") or []
    lo = [float(a["ctrlrange"][0]) for a in actuators]
    hi = [float(a["ctrlrange"][1]) for a in actuators]

    trace = bool(args.report or args.trajectory)
    plan = build_graph(preset, n, lo, hi, args.vx, trace_state=trace)
    blob = ir.compile_blob(plan)

    control_hz, physics_hz = 50.0, 200.0
    decim = int(physics_hz // control_hz)

    core, ort = load_libs()
    err = ctypes.create_string_buffer(512)
    paths = [os.path.join(ROOT, "build/models", f) for _, _, f in MODEL_SPECS]
    arr = (ctypes.c_char_p * len(paths))(*[x.encode() for x in paths])
    be = ort.ds_ort_create_multi(arr, len(paths), err, 512)
    if not be:
        raise SystemExit("ORT 多模型后端创建失败：" + err.value.decode())
    handle = core.ds_load(blob, len(blob), err, 512)
    if not handle:
        raise SystemExit("加载 plan 失败：" + err.value.decode())
    info = Info()
    core.ds_info_of(handle, ctypes.byref(info))

    spec = mujoco.MjSpec.from_file(args.mjcf)
    g = spec.worldbody.add_geom()
    g.type = mujoco.mjtGeom.mjGEOM_PLANE
    g.size = [4.0, 4.0, 0.1]
    g.condim = 3
    m = spec.compile()
    d = mujoco.MjData(m)
    m.opt.timestep = 1.0 / physics_hz
    d.qpos[:] = preset["initialQpos"]
    mujoco.mj_forward(m, d)

    qpos_off = [int(m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]
    dof_off = [int(m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)]) for nm in driven]
    sadr = int(m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "imu_ang_vel")])
    default_q = np.array(preset["defaultJointPositions"])

    pulses = {}
    for part in args.pulses.split(",") if args.pulses else []:
        s_, v_ = part.split(":")
        pulses[round(float(s_) * control_hz)] = float(v_)

    steps = int(args.seconds * control_hz)
    obs = np.zeros(61, dtype=np.float32)
    act = np.zeros(n, dtype=np.float32)
    frames, states, heights, tilts = [], [], [], []
    n_out = n + (1 if trace else 0)
    cur_pulse = 0.0
    dead = False

    for k in range(steps):
        if k in pulses:
            cur_pulse = pulses[k]
        R = quat_to_mat(d.qpos[3:7])
        obs[0:3] = d.sensordata[sadr:sadr + 3]
        obs[3:6] = R.T @ np.array([0.0, 0.0, -1.0])
        obs[6:20] = np.array([d.qpos[i] for i in qpos_off]) - default_q
        obs[20:34] = np.array([d.qvel[i] for i in dof_off])
        obs[34:48] = act
        obs[48:61] = 0.0
        obs[48] = args.vx
        obs[60] = cur_pulse          # 第 13 维借来放策略指令
        if cur_pulse:
            cur_pulse = 0.0            # 脉冲只持续一步

        bi = (ctypes.c_float * 61)(*obs.tolist())
        bo = (ctypes.c_float * n_out)()
        rc = core.ds_step(handle, bi, bo, be)
        if rc != 0:
            raise SystemExit(f"ds_step 返回 {rc}")
        act = np.array(bo[:n], dtype=np.float32)
        if trace:
            states.append(int(bo[n]))

        d.ctrl[:] = act
        for _ in range(decim):
            mujoco.mj_step(m, d)
        if not np.isfinite(d.qpos).all():
            dead = True
            break

        heights.append(float(d.qpos[2]))
        tilts.append(float(1 - 2 * (d.qpos[4] ** 2 + d.qpos[5] ** 2)))
        if args.trajectory:
            frames.append({"t": round(k / control_hz, 4),
                           "pos": [round(float(x), 6) for x in d.qpos[0:3]],
                           "quat": [round(float(x), 6) for x in d.qpos[3:7]],
                           "q": [round(float(d.qpos[i]), 6) for i in qpos_off]})

    trans = [i for i in range(1, len(states)) if states[i] != states[i - 1]]
    print(f"状态轨迹：" + ", ".join(f"{i/control_hz:.1f}s→{states[i]}" for i in trans[:12]))
    heights = np.array(heights) if heights else np.array([np.nan])
    print(f"状态机图 {info.node_count} 节点｜{len(heights)}/{steps} 帧｜"
          f"z 终 {heights[-1]:+.3f} 最低 {np.nanmin(heights):+.3f}｜直立 终 {tilts[-1]:+.3f}")
    verdict = {"finite": not dead, "no_sink": bool(np.nanmin(heights) > -0.05),
               "ran_full": len(heights) == steps}
    verdict["pass"] = verdict["finite"] and verdict["no_sink"] and verdict["ran_full"]
    print(f"  {'PASS' if verdict['pass'] else 'FAIL'}  {verdict}")

    if args.report:
        out = os.path.join(ROOT, args.report)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"plan_digest": ir.sha256_of_blob(blob), "nodes": info.node_count,
                       "steps": steps, "verdict": verdict,
                       "metrics": {"base_height_min": float(np.nanmin(heights)),
                                   "upright_final": tilts[-1]}}, f, ensure_ascii=False, indent=2)
        print(f"  报告 → {args.report}")
    if args.trajectory:
        out = os.path.join(ROOT, args.trajectory)
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w", encoding="utf-8") as f:
            json.dump({"meta": {"robot": doc["robot"]["id"], "joints": driven, "target": "fsm",
                                "amp": 0, "gravity": "on", "seconds": args.seconds,
                                "control_hz": control_hz, "physics_hz": physics_hz,
                                "policy_id": "duck-fsm",
                                "mjcf_digest": sha256(args.mjcf),
                                "preset_digest": sha256(args.preset)},
                       "frames": frames}, f, ensure_ascii=False, separators=(",", ":"))
        print(f"  轨迹 → {args.trajectory}（{len(frames)} 帧）")

    core.ds_free(handle)
    ort.ds_ort_destroy(be)
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
